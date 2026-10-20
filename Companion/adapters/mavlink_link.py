# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""
MavlinkAutopilotLink - the AutopilotLink implementation for ArduPlane over MAVLink.

Owns: the MAVLink connection, the _mav_lock, the message-queue subscription
system, the telemetry thread (sole reader), the parameter-handshake startup
quirks, the mission upload, the catapult-launch sequence, and the level-out's
stick overrides.

The public surface is the AutopilotLink interface: release_rc_overrides, set_mode_acro / set_mode_loiter, start_telemetry,
request_high_rate_streams, send_gcs_heartbeat, aircraft_world_pos_ned and
shutdown.  Steering does not pass through here: the control law runs on the FC
and LuaOffloadLink carries its inputs.  Interface attribute names (attitude, rates, lat/lon, relative_alt,
airspeed, groundspeed, armed, and servo_ch4 - the V-tail-right/rudder channel)
are defined in interfaces.py.
"""

# `MAVLINK_DISABLE_STREAMS` (constants.py) + `_disable_unread_streams()` send
# `SET_MESSAGE_INTERVAL(-1)` for messages nothing here consumes. The call runs
# LAST, after the stream-group requests and the per-message intervals.
#
# Four members cannot be removed at all - SIMSTATE, AHRS2, AOA_SSA (EXTRA1) and
# RC_CHANNELS (RC_CHAN group): ArduPilot will not honour a per-message disable
# for a member of a group being actively requested, and those groups also carry
# ATTITUDE and SERVO_OUTPUT_RAW, which are needed.  Reordering does not help.  A
# helper that disables SIMSTATE/PID_TUNING/SERVO_OUTPUT_RAW is therefore a
# silent no-op: do not add one.  The disabling saves companion parse work; it
# is not a lever on the attitude rate.
#
# READING TELEMETRY RATES.  A `[TELEM RATES]` line containing request/response
# messages (COMMAND_ACK, PARAM_VALUE, STATUSTEXT, GPS_GLOBAL_ORIGIN,
# HOME_POSITION) is a startup snapshot, not steady state.  Every stream scaling
# by the same factor - HEARTBEAT included, which is nowhere near any budget - is
# the host slowing down (SITL, Gazebo, the companion and WSLg share one
# machine), not a per-stream effect.  So no rate conclusion is safe from one run
# on a host whose speed is not known-good.  `mean(quat_fresh)` over a whole
# flight beats the 2-second startup probe: minutes of data, already a CSV
# column.

from __future__ import annotations
import logging
import math
import queue
import select
import struct
import sys
import threading
import time

import numpy as np
from pymavlink import mavutil

from ..constants import (
    PITCH_LIM_UP_DEFAULT, PITCH_LIM_DN_DEFAULT,
    ACRO_ROLL_RATE_DEFAULT, ACRO_PITCH_RATE_DEFAULT, RLL_RATE_FLTE_EXPECTED,
    SCHED_LOOP_RATE_EXPECTED,
    PTCH_RATE_FLTE_EXPECTED, GCS_PID_MASK_EXPECTED, MAV_STREAM_RATE_EXPECTED,
    AHRS_EKF_TYPE_EXPECTED, EK3_ENABLE_EXPECTED,
    PTCH_LIM_MIN_EXPECTED_DEG, PTCH_LIM_MAX_EXPECTED_DEG,
    BANK_LIM_DEG,
    MAVLINK_DISABLE_STREAMS,
    EXPECTED_RATE_PIDS, ARDUPLANE_AUTOTUNE_PARAMS, NATIVE_MODE_RATE_PIDS,
    ACRO_DEFAULT_PARAMS,
)
from ..controller_autotune_store import float32_text

log = logging.getLogger("balloon_chaser")

# How closely an FC rate gain must match a stored set (the Acro set or the
# Autopilot set) to pass the startup check: relative, with an absolute floor for
# the near-zero D terms.
RATE_PID_MATCH_REL_TOL = 0.02
RATE_PID_MATCH_ABS_TOL = 1e-4

# Parameter reads in flight at once.  ArduPlane queues 20 PARAM_REQUEST_READs
# across every link (GCS_MAVLINK::param_requests) and drops what does not fit,
# so a batch leaves room for a ground station reading at the same time.
PARAM_READ_BATCH = 10
# The longest a caller's RC override goes unrefreshed while a mode switch's
# parameter write waits on readbacks (_set_mode `hold`).  Well inside
# ArduPlane's RC_OVERRIDE_TIME, so the override never lapses mid-write.
HOLD_REFRESH_S = 0.2


def _rate_pid_matches(got, want):
    """True when an FC rate-loop value `got` passes for `want` at startup: within
    RATE_PID_MATCH_REL_TOL of it, or RATE_PID_MATCH_ABS_TOL for a value near
    zero.  None, a value the FC did not report, never matches."""
    want = float(want)
    tol = max(RATE_PID_MATCH_ABS_TOL, RATE_PID_MATCH_REL_TOL * abs(want))
    return got is not None and abs(got - want) <= tol


def _same_float32(a, b):
    """True when a and b are the same 32-bit float, the precision PARAM_VALUE
    carries.  None, a value the FC has not reported, is never the same."""
    if a is None or b is None:
        return False
    return struct.pack('<f', float(a)) == struct.pack('<f', float(b))


def _screen_log_handlers():
    """The log handlers that print to the terminal - not the console-log FILE,
    which keeps recording while the screen is held."""
    loggers = [logging.getLogger()] + [
        lg for lg in logging.Logger.manager.loggerDict.values()
        if isinstance(lg, logging.Logger)]
    found = {}
    for lg in loggers:
        for h in lg.handlers:
            if (isinstance(h, logging.StreamHandler)
                    and not isinstance(h, logging.FileHandler)
                    and getattr(h, 'stream', None) in (sys.stdout, sys.stderr)):
                found[id(h)] = h
    return list(found.values())


class _HeldScreenOutput(logging.Filter):
    """Installed on a terminal handler while a console prompt waits: every record
    is kept instead of printed, so the prompt stays the last line on screen.
    Other threads (telemetry rates, FC status text) keep logging throughout."""

    def __init__(self, handler):
        super().__init__()
        self.handler = handler
        self.records = []

    def filter(self, record):
        self.records.append(record)
        return False


def halt_on_param_mismatches(mismatches) -> None:
    """List the FC parameters that differ from what this code expects and HALT
    until the operator presses Enter on the console (continue with the FC's
    values) or Ctrl+C (stop).  With no console to confirm on, refuse to start.

    While it waits, nothing else is printed: log output bound for the terminal is
    held (_HeldScreenOutput) and printed, in order, once the prompt returns; the
    console-log file records it as it happens.

    Startup never corrects a saved parameter - the operator sets it by hand
    (MAVProxy `param set`).  `mismatches` is a list of
    (name, found_text, expected_text, reason)."""
    if not mismatches:
        return
    log.warning('')
    log.warning('=== %d FC parameter(s) differ from what this code expects ===',
                len(mismatches))
    for name, found, expected, reason in mismatches:
        log.warning('  %-18s = %-10s expected %-10s %s', name, found, expected, reason)
    log.warning('Set them by hand (MAVProxy: param set <NAME> <value>) and restart; '
                'some take effect only after an autopilot reboot.')
    holds = [_HeldScreenOutput(h) for h in _screen_log_handlers()]
    for hold in holds:
        hold.handler.addFilter(hold)
    try:
        input('>>> Press Enter to continue with the FC values shown above, '
              'or Ctrl+C to stop: ')
    except EOFError:
        raise RuntimeError(
            'FC parameters differ from what this code expects (%s) and there is no '
            'console to confirm on. Refusing to start.'
            % ', '.join(m[0] for m in mismatches))
    finally:
        for hold in holds:
            hold.handler.removeFilter(hold)
        held = sum(len(hold.records) for hold in holds)
        if held:
            log.info('(%d message(s) held while waiting for Enter:)', held)
        for hold in holds:
            for record in hold.records:
                hold.handler.handle(record)
    log.warning('Operator confirmed - continuing with the FC values shown above.')

_BUILD_SHA = None


def _build_sha() -> str:
    """First 8 hex of the package sha, computed once and cached.

    Short because it rides a line printed every ten seconds; eight hex tells two
    builds apart by eye, and the full digest is still printed once at startup
    for anyone who needs an exact match.  It walks every .py and .lua on disk,
    so it is computed lazily and only once.
    """
    global _BUILD_SHA
    if _BUILD_SHA is None:
        try:
            from ..csv_logger import package_sha256
            _BUILD_SHA = package_sha256()[:8]
        except Exception:                                  # noqa: BLE001
            _BUILD_SHA = "unknown"
    return _BUILD_SHA


def is_autopilot_heartbeat(msg, vehicle_sysid) -> bool:
    """True only for the HEARTBEAT the autopilot itself sends.

    Other senders on the same link - a GCS such as MAVProxy, a gimbal, another
    companion - send HEARTBEAT too, with custom_mode 0 and the armed bit clear.
    Reading theirs flips the FC mode to 0 (MANUAL) and `armed` to False between
    the autopilot's own beats, so _set_mode(MANUAL) confirms before the FC has
    switched.  ArduPilot sends its heartbeat from component 1
    (MAV_COMP_ID_AUTOPILOT1) with autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA, from
    the vehicle's own system id."""
    return (msg.get_srcSystem() == vehicle_sysid
            and msg.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
            and msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA)



class MavlinkAutopilotLink:
    """AutopilotLink over MAVLink/ArduPlane.  See module docstring."""

    def __init__(self, connection_string='udp:127.0.0.1:14550'):
        log.info("Connecting to ArduPilot at %s ...", connection_string)
        # Kept so the connection can be REBUILT after an autopilot reboot.
        # SITL's MAVLink endpoints are TCP listeners: a reboot tears them down
        # and rebinds them, so the old socket is dead but the address is not.
        self.connection_string = connection_string
        try:
            self.master = mavutil.mavlink_connection(connection_string)
        except Exception as e:
            log.error("Failed to open MAVLink connection '%s': %s", connection_string, e)
            raise ConnectionError(
                f"Could not connect to '{connection_string}'. "
                f"Check that the address is correct and the autopilot is reachable."
            ) from e

        # Lock protecting ALL reads and writes on self.master.
        self._mav_lock = threading.Lock()

        # -- Message queue system (telemetry thread is the sole reader) --
        self._msg_queues = {}
        self._msg_queues_lock = threading.Lock()

        # -- Telemetry state (AutopilotLink interface attributes) --
        self.attitude = np.array([0.0, 0.0, 0.0])      # roll, pitch, yaw (rad)
        # SITL ground truth from SIMSTATE, None until the first one arrives and
        # forever on real hardware.  Not used by any control path - it exists so
        # a disagreement between the estimated attitude and the reported gyro
        # can be settled against the truth instead of argued about.
        self.attitude_time_boot_s = None               # FC clock for `attitude`
        self.sim_attitude = None                       # roll, pitch, yaw (rad)
        self.sim_rates = None                          # p, q, r (rad/s)
        self.attitude_recv_t = 0.0                     # monotonic time of last ATTITUDE msg
        # Attitude quaternion (w,x,y,z) from ATTITUDE_QUATERNION.  Singularity-free
        # near vertical pitch, where the Euler (roll,yaw) above gimbal-lock.  Used to
        # build the body-frame balloon vector without the vertical-dive convulsion.
        self.attitude_quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.attitude_quat_recv_t = 0.0                # monotonic time of last ATTITUDE_QUATERNION
        self.attitude_quat_seq = 0                     # increments on EVERY ATTITUDE_QUATERNION received
                                                       # (per-tick freshness / effective-rate instrument)
        self.rates = np.array([0.0, 0.0, 0.0])         # body rates (rad/s)
        self.lat = 0.0
        self.lon = 0.0
        self.relative_alt = 0.0
        # Validity time of the CURRENT position fix, seconds since FC boot, from
        # GLOBAL_POSITION_INT.time_boot_ms.  None until the first fix arrives.
        #
        # This is the FC's OWN stamp for when the fix was true, so it is immune
        # to link jitter and to how long the control loop took to notice.  That
        # matters because lat/lon/relative_alt only change at the 5 Hz fix rate
        # while the control loop reads them at 50 Hz: consumers that need to know
        # WHICH fix they are looking at, or exactly when it was valid, cannot get
        # that from the wall clock at the moment they happened to read it.
        # BalloonChaser._update_balloon_range is the consumer - it uses this
        # both to detect a new fix and to time the closest-approach fit.
        self.pos_time_boot_s = None
        self.airspeed = 11.3
        # Latest PID_TUNING per axis (ArduPilot PID_TUNING_AXIS: 1=roll, 2=pitch,
        # 3=yaw).
        # Each entry: dict(desired, achieved, ff, p, i, d, srate).  Populated only
        # when the FC streams PID_TUNING (GCS_PID_MASK = 7 = roll bit0 + pitch bit1
        # + yaw bit2, checked at startup; the CSV columns cover roll and pitch).
        # These are the FC rate loop's OWN internal signals - the authoritative
        # inner-loop view, unlike the noisy passthrough gyro (see CLAUDE.md).
        self._pid_tuning = {}
        self.groundspeed = 11.3
        # NED velocity (m/s) from GLOBAL_POSITION_INT, for flight-path angle.
        self.vel_ned = np.array([0.0, 0.0, 0.0])   # north, east, down
        self.armed = False
        self.prearm_healthy = False
        self.current_mode = 0
        self.home_lat = None
        self.home_lon = None

        # -- ArduPilot's own aerodynamic angles (AOA_SSA) --
        # ArduPilot computes these WIND-CORRECTED; the sideslip_deg the script
        # computes (track.lua flow_angles) is ground-referenced and is not.
        # Both are logged so the difference exposes the wind contamination.
        # None until the first AOA_SSA, written blank: a 0.0 there would read
        # as an aircraft flying straight.
        self.fc_aoa_deg = None
        self.fc_ssa_deg = None

        # -- Saved ArduPlane parameters the Gain Tuner reads and writes --
        # name -> the value the FC last CONFIRMED (startup read, a Read, or the
        # readback after a Write), for ARDUPLANE_AUTOTUNE_PARAMS.  A name is
        # absent until the FC has answered for it.  The tuner shows these and the
        # flight log records them; nothing re-reads them when logging starts.
        self.native_params = {}
        # The same values as startup found them, kept apart so the tuner's reset
        # icon can write a parameter back to what the FC held when the program
        # started, whatever has been written since.  Filled once, at startup.
        self.native_params_at_startup = {}
        # The operator's own values: the startup read, then each Gain Tuner Write
        # or reset.  ACRO flies these, so a switch into ACRO writes back any the
        # FC no longer holds (_set_mode).
        self.native_params_operator = {}
        # True when startup found the FC holding the Autopilot rate gains whole
        # (_check_rate_pids); ACRO then takes the stored Acro defaults.
        self._fc_holds_autopilot_set = False
        # Off until enable_mode_param_sets().  The ArduPlane AUTOTUNE maintenance
        # mode switches modes through _set_mode too, and a write there would
        # replace the gains AUTOTUNE is tuning.
        self._mode_param_sets = False

        # -- Servo telemetry (HUD) --
        self.servo_aileron = 1500
        self.servo_elevator = 1500
        self.servo_ch3 = 1500
        self.servo_ch4 = 1500     # was servo_rudder in source
        self.servo_ch5 = 1500
        self.servo_ch6 = 1500
        self.servo_ch7 = 1500
        self.servo_ch8 = 1500

        # -- FC config (filled by read_fc_config) --
        self.pitch_lim_up = PITCH_LIM_UP_DEFAULT
        self.pitch_lim_dn = PITCH_LIM_DN_DEFAULT
        self.acro_roll_rate = ACRO_ROLL_RATE_DEFAULT
        self.acro_pitch_rate = ACRO_PITCH_RATE_DEFAULT
        self._rll_rate_flte = 0.0
        self._acro_locking = 0.0
        self._stab_pitch_down = 2.0
        self._ptch_trim_deg = 0.0
        self._kff_thr2ptch = 0.0
        self._servo1_trim = 1500.0
        self._servo1_min = 1100.0
        self._servo1_max = 1900.0
        self._servo2_trim = 1500.0
        self._servo2_min = 1100.0
        self._servo2_max = 1900.0
        # SERVO4 is the OTHER ruddervator.  Its trim is already checked by the
        # manifest gate, but the scaling was never fetched - so the rudder deflection,
        # which on a V-tail is the DIFFERENCE between SERVO2 and SERVO4, could
        # be computed in raw PWM and never converted to a percentage.  That is
        # the number the authority budget is written in: how much ruddervator
        # the yaw rate demand spends, and what is left for the elevator.
        self._servo4_trim = 1500.0
        self._servo4_min = 1100.0
        self._servo4_max = 1900.0
        # Saved FC parameters that differ from what this code expects, collected
        # by the read-only checks in read_fc_config and shown together at its end
        # (halt_on_param_mismatches).  (name, found_text, expected_text, reason).
        self._param_mismatches = []
        # SERVOn_FUNCTION: 4 = aileron, 19 = elevator, 77/78 = elevon L/R.
        # AIRFRAME NOTE (Talon V-tail): SERVO1=4 aileron, SERVO2=79 V-tail left,
        # SERVO4=80 V-tail right.  elevon_mixing() below therefore returns False,
        # and that is CORRECT only while nothing commands a rudder: the V-tail
        # mix then collapses to common-mode and SERVO2 carries elevator alone.
        # Nothing forces rudder demand to zero, so the moment yaw is commanded
        # SERVO2/SERVO4 move DIFFERENTIALLY and this branch mislabels them; the
        # HUD then needs pitch = (s2+s4)/2, yaw = (s2-s4)/2.
        # On an elevon airframe servos 1/2 are ELEVONS - roll is
        # their difference and pitch is their mean.  Labelling them
        # "Aileron"/"Elevator" reports both surfaces holding pitch trim as if
        # the aircraft were also holding roll.
        self._servo1_function = 4.0
        self._servo2_function = 19.0
        self._fs_gcs_enabl = 0.0
        # ArduPlane renamed SYSID_MYGCS -> MAV_GCS_SYSID and SYSID_THISMAV ->
        # MAV_SYSID; read_fc_config fills these from the FC (defaults: 255 =
        # accept any GCS, 1 = this vehicle).
        self._mav_gcs_sysid = 255.0
        self._mav_sysid = 1.0
        # The FC's full parameter NAME set, captured by the manifest gate, so the
        # rate-gain check can skip a param this FC does not register (YAW_RATE_*
        # with YAW_RATE_ENABLE off).  None until the gate has run.
        self._fc_param_names = None

        # -- HUD-facing SERVO range mirrors (interface names) --
        # Exposed read-only via properties below so the HUD can normalise.

        # -- Internals --
        self._telem_thread = None
        self._telem_stop = threading.Event()
        self._last_heartbeat_time = 0.0

        self._wait_for_heartbeat()

    # -- HUD-facing property mirrors for SERVO ranges --
    @property
    def servo1_min(self): return self._servo1_min
    @property
    def servo1_trim(self): return self._servo1_trim
    @property
    def servo1_max(self): return self._servo1_max
    @property
    def servo2_min(self): return self._servo2_min
    @property
    def servo2_trim(self): return self._servo2_trim
    @property
    def servo2_max(self): return self._servo2_max
    @property
    def servo4_min(self): return self._servo4_min
    @property
    def servo4_trim(self): return self._servo4_trim
    @property
    def servo4_max(self): return self._servo4_max

    @property
    def elevon_mixing(self):
        """True when servos 1/2 are ELEVONS rather than aileron/elevator.

        SERVOn_FUNCTION: 4 = aileron, 19 = elevator, 77/78 = elevon left/right.
        On an elevon wing both surfaces carry pitch AND roll, so reporting them
        channel-by-channel shows two identical numbers in level flight and reads
        as "holding roll and pitch" when the aircraft holds neither.
        """
        return (int(self._servo1_function) in (77, 78)
                or int(self._servo2_function) in (77, 78))

    def reconnect(self, timeout=45.0):
        """Rebuild the MAVLink connection after the autopilot reboots.

        A reboot drops the socket.  Without this, pymavlink spins on a dead fd
        printing "EOF on TCP socket" forever, forcing a manual sim_vehicle.py
        restart in the middle of an automated setup.

        SITL rebinds the SAME address, so the fix is simply to reopen it: stop
        the reader, close the old socket, retry the connect until the listener
        is back, then wait for a heartbeat and restart the reader.
        """
        log.info('reconnecting to %s ...', self.connection_string)
        self._telem_stop.set()
        t = self._telem_thread
        if t is not None and t.is_alive():
            t.join(timeout=3.0)
        self._telem_thread = None
        try:
            with self._mav_lock:
                self.master.close()
        except Exception:                              # noqa: BLE001
            pass

        deadline = time.monotonic() + timeout
        last_err = None
        while time.monotonic() < deadline:
            time.sleep(1.0)
            try:
                m = mavutil.mavlink_connection(self.connection_string)
                if m.wait_heartbeat(timeout=3.0) is not None:
                    with self._mav_lock:
                        self.master = m
                    self._telem_stop.clear()
                    self.start_telemetry()
                    log.info('reconnected (system %d, component %d)',
                             self.master.target_system,
                             self.master.target_component)
                    return True
                m.close()
            except Exception as exc:                   # noqa: BLE001
                last_err = exc
        log.error('could not reconnect to %s within %.0fs (%s)',
                  self.connection_string, timeout, last_err)
        return False

    def _subscribe(self, *msg_types, maxsize=50):
        """Register interest in message types. Returns a queue that will
            receive messages from the telemetry thread. Multiple subscribers
            to the same message type are supported.

            maxsize=0 means UNBOUNDED. The dispatcher uses put_nowait and
            silently discards on queue.Full, so any bulk transfer larger than
            the queue depth MUST subscribe unbounded or it will lose messages
            with no error. A full parameter download is ~1400 messages against
            the default depth of 50."""
        q = queue.Queue(maxsize=maxsize)
        with self._msg_queues_lock:
            for t in msg_types:
                if t not in self._msg_queues:
                    self._msg_queues[t] = []
                self._msg_queues[t].append(q)
        return q

    def _unsubscribe(self, q, *msg_types):
        """Remove a specific queue from the given message types."""
        with self._msg_queues_lock:
            for t in msg_types:
                lst = self._msg_queues.get(t)
                if lst:
                    try:
                        lst.remove(q)
                    except ValueError:
                        pass
                    if not lst:
                        del self._msg_queues[t]

    def _wait_msg(self, q, timeout):
        """Wait for a message from a subscribed queue.
            Returns the message or None on timeout.
            The queue only contains message types this subscriber registered for."""
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            try:
                return q.get(timeout=min(remaining, 0.1))
            except queue.Empty:
                continue

    def _wait_for_heartbeat(self, timeout=20.0):
        """Wait for heartbeat from vehicle.

        BOUNDED.  A bare wait_heartbeat() blocks forever when the autopilot is
        not running, and prints nothing while it does - indistinguishable from a
        slow start, and it leaves the operator with a frozen terminal and no
        idea which of several things is wrong.
        """
        log.info('Waiting for heartbeat...')
        deadline = time.monotonic() + timeout
        hb = None
        while time.monotonic() < deadline:
            with self._mav_lock:
                hb = self.master.wait_heartbeat(timeout=3.0)
            if hb is not None:
                break
            log.info('  no heartbeat yet - is the autopilot running?')
        if hb is None:
            raise RuntimeError(
                f'No MAVLink heartbeat after {timeout:.0f}s. The autopilot is '
                f'not running, or is not listening on this endpoint.\n'
                f'  Check it is alive : ps aux | grep arduplane\n'
                f'  Check the port    : ss -ltn | grep 576\n'
                f'  If ArduPlane died, restart it (and MAVProxy) before rerunning.')
        log.info('Heartbeat received (system %d, component %d)', self.master.target_system, self.master.target_component)

    def _set_mode(self, mode_id, mode_name, params=None, hold=None):
        """Switch to a mode by numeric ID and wait for confirmation, then write
        the rate-loop values that mode flies (_mode_param_set), or `params` when
        given.

        The mode comes FIRST.  A hand-back reaches LOITER without waiting on
        parameter round trips, and the balloon chaser does not start flying until
        enter_acro returns, which is after the write.  A write that fails is
        logged and leaves the mode change standing: the aircraft is in the mode
        the caller asked for either way.

        `hold` is for a caller that flies the new mode on RC overrides: it is
        called once the moment the mode is confirmed, and again at least every
        HOLD_REFRESH_S while the write waits for its readbacks.  Without it the
        aircraft would fly the new mode on the real sticks for as long as the
        round trip takes - seconds, if the FC is slow to answer."""
        log.info('Switching to %s mode...', mode_name)
        deadline = time.monotonic() + 30.0
        resend_time = 0
        with self._mav_lock:
            self.master.mav.set_mode_send(self.master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
        while True:
            if time.monotonic() > deadline:
                raise TimeoutError(f'Timed out waiting for {mode_name} mode after 30s')
            if self.current_mode == mode_id:
                log.info('%s mode active.', mode_name)
                break
            if time.monotonic() - resend_time > 2.0:
                with self._mav_lock:
                    self.master.mav.set_mode_send(self.master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
                resend_time = time.monotonic()
            time.sleep(0.05)
        if hold is not None:
            hold()
        if not self._mode_param_sets:
            return
        want = self._mode_param_set(mode_id) if params is None else params
        if want:
            try:
                self._write_param_set(want, mode_name, tick=hold)
            except Exception:                          # noqa: BLE001
                log.exception('[FC PARAM] %s: writing its rate-loop values FAILED - '
                              'the FC may hold the previous mode\'s', mode_name)

    def enable_mode_param_sets(self):
        """Write each mode's rate-loop values on every mode switch from now on.
        The controller calls this once startup has read what the FC holds; the
        ArduPlane AUTOTUNE maintenance mode never does."""
        self._mode_param_sets = True

    def _mode_param_set(self, mode_id):
        """The rate-loop values ArduPlane mode `mode_id` flies: the operator's in
        ACRO (4), NATIVE_MODE_RATE_PIDS in every other mode that runs the rate
        loop, and None for MANUAL (0), which drives the surfaces straight from
        the sticks, and AUTOTUNE (8), which tunes these values itself."""
        if mode_id == 4:
            return self.native_params_operator
        if mode_id in (0, 8):
            return None
        return NATIVE_MODE_RATE_PIDS

    def write_rate_loop_set(self, which):
        """The Gain Tuner's set Write buttons: write one whole stored set from
        constants.py and read it back - never the tuner's boxes.  'native' is
        NATIVE_MODE_RATE_PIDS, the set ArduPlane's own modes fly; 'operator' is
        ACRO_DEFAULT_PARAMS, the stored Acro defaults.  EVERY value goes out,
        whether or not the FC is thought to hold it.

        The Acro defaults also become the operator's values, the set every
        switch into ACRO writes: otherwise the next switch would put back the
        very values the Write replaced.  Each takes the value the FC reports,
        or, with no reply, the default, so the next switch writes it again.
        The native set leaves the operator's values alone.  Returns the names
        written."""
        if which == 'native':
            values, label = NATIVE_MODE_RATE_PIDS, 'Default Autopilot Parameters Write'
        elif which == 'operator':
            values, label = ACRO_DEFAULT_PARAMS, 'Default Acro Parameters Write'
        else:
            raise ValueError('unknown rate-loop set %r' % (which,))
        written = self._write_param_set(values, label, force=True)
        if which == 'operator':
            for name in written:
                self.native_params_operator[name] = self.native_params.get(
                    name, float(values[name]))
        return written

    def _write_param_set(self, values, label, force=False, tick=None):
        """Write each value in `values` the FC is not known to hold, or every
        one with `force`, then read them back.  What the FC holds is
        native_params: the startup read plus every write since, so a value
        already in place costs nothing.  The PARAM_SETs go out together and the
        readbacks are gathered together, one round trip for the set rather than
        one per parameter.

        A readback that differs from what was sent is recorded as the FC's
        value; a parameter that does not answer is dropped from native_params,
        because its value is unknown, so the log shows it blank and the next
        switch writes it again.  `tick` is called while the readbacks are
        awaited (see _set_mode's `hold`).  Returns the names written."""
        have = self._fc_param_names
        todo = {n: float(v) for n, v in dict(values).items()
                if (have is None or n in have)
                and (force or not _same_float32(self.native_params.get(n), v))}
        if not todo:
            return []
        q = self._subscribe('PARAM_VALUE', maxsize=0)
        try:
            with self._mav_lock:
                for name, value in todo.items():
                    self.master.mav.param_set_send(
                        self.master.target_system, self.master.target_component,
                        name.encode('utf-8'), value,
                        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            got = self._fetch_params(q, list(todo), tick=tick)
        finally:
            self._unsubscribe(q, 'PARAM_VALUE')
        done, wrong = [], []
        for name, value in todo.items():
            if name not in got:
                self.native_params.pop(name, None)
                wrong.append('%s = %s: no reply, value unknown' % (name, float32_text(value)))
                continue
            self.native_params[name] = got[name]
            if _same_float32(got[name], value):
                done.append('%s %s' % (name, float32_text(got[name])))
            else:
                wrong.append('%s = %s: the FC holds %s' % (
                    name, float32_text(value), float32_text(got[name])))
        if done:
            log.info('[FC PARAM] %s: wrote %s', label, ', '.join(done))
        if wrong:
            log.error('[FC PARAM] %s: %s', label, '; '.join(wrong))
        return list(todo)

    def _fetch_params(self, q, names, attempts=3, per_attempt_s=2.0, tick=None):
        """_fetch_param for several names at once: up to PARAM_READ_BATCH
        requests go out together and the replies are taken in whatever order
        they arrive.  `tick`, when given, is called at least every
        HOLD_REFRESH_S while waiting.  Returns {name: value} for the names that
        answered."""
        want = set(names)
        got = {}
        last_tick = time.monotonic()        # the caller has just called it
        for _ in range(attempts):
            missing = [n for n in names if n not in got]
            if not missing:
                break
            for i in range(0, len(missing), PARAM_READ_BATCH):
                batch = missing[i:i + PARAM_READ_BATCH]
                with self._mav_lock:
                    for name in batch:
                        self.master.mav.param_request_read_send(
                            self.master.target_system, self.master.target_component,
                            name.encode('utf-8'), -1)
                deadline = time.monotonic() + per_attempt_s
                while any(n not in got for n in batch):
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    if tick is None:
                        msg = self._wait_msg(q, remaining)
                        if msg is None:
                            break
                    else:
                        if time.monotonic() - last_tick >= HOLD_REFRESH_S:
                            tick()
                            last_tick = time.monotonic()
                        msg = self._wait_msg(q, min(remaining, HOLD_REFRESH_S))
                        if msg is None:
                            continue
                    name = msg.param_id.rstrip('\x00')
                    if name in want:
                        got[name] = float(msg.param_value)
        return got

    def _verify_params(self):
        """Read back the configuration the flight code depends on and log a
        PASS/FAIL table, so the whole contract is visible in one place.  A
        confirmation, not a gate: the halting checks are _check_param's.

            STAB_PITCH_DOWN  -> must be 0.0
            PTCH_TRIM_DEG    -> must be 0.0
            KFF_THR2PTCH     -> must be 0.0
            SERVO1/2/4_TRIM  -> must be 1500 (surface bias)
            PTCH_LIM_MIN_DEG -> must be <= -90.0 (steep dive pitch allowed)
            ACRO_LOCKING     -> must be 0 (pure rate, no Euler lock)
            ACRO_ROLL_RATE   -> informational; the FC owns roll authority
            ACRO_PITCH_RATE  -> informational; the FC owns pitch authority and
                                the law plans within it
            RLL_RATE_FLTE    -> must be 5 Hz to soften ACRO roll-rate shake
            SCHED_LOOP_RATE  -> >= SCHED_LOOP_RATE_EXPECTED
            RC_OPTIONS       -> bit 0 (IGNORE_OVERRIDES) must be clear

        Not read: ROLL_LIMIT_DEG (ACRO ignores it; the bank limit is the
        bank_lim_deg gain), KFF_RDDRMIX and YAW2SRV_* (they feed
        calc_nav_yaw_coordinated, which the nav_scripting path never calls, so
        they are inert under Lua and live in FBWA).
        """
        checks = [('STAB_PITCH_DOWN', 0.0, lambda v: abs(v) < 0.01, '== 0'), ('PTCH_TRIM_DEG', 0.0, lambda v: abs(v) < 0.01, '== 0'), ('KFF_THR2PTCH', 0.0, lambda v: abs(v) < 0.001, '== 0'), ('SERVO1_TRIM', 1500.0, lambda v: abs(v - 1500.0) < 2.0, '== 1500 (aileron)'), ('SERVO2_TRIM', 1500.0, lambda v: abs(v - 1500.0) < 2.0, '== 1500 (V-tail left)'), ('SERVO4_TRIM', 1500.0, lambda v: abs(v - 1500.0) < 2.0, '== 1500 (V-tail right)'), ('PTCH_LIM_MIN_DEG', -90.0, lambda v: v <= -89.9, '<= -90°'), ('ACRO_LOCKING', 0.0, lambda v: abs(v) < 0.01, '== 0 (pure rate)'), ('ACRO_ROLL_RATE', 180.0, lambda v: True, 'informational'), ('ACRO_PITCH_RATE', 90.0, lambda v: True, 'informational (FC is the authority, as for roll)'), ('RLL_RATE_FLTE', 5.0, lambda v: abs(v - 5.0) < 0.1, '== 5 Hz to reduce ACRO roll shake'), ('SCHED_LOOP_RATE', 400.0, lambda v: v >= SCHED_LOOP_RATE_EXPECTED - 0.5, '>= 400 Hz (ceiling on every telemetry stream)'), ('RC_OPTIONS', 0.0, lambda v: int(v) & 1 == 0, 'bit0 (IGNORE_OVERRIDES) clear')]
        log.info('=== PARAMETER READBACK VERIFICATION ===')
        q = self._subscribe('PARAM_VALUE')
        all_pass = True
        try:
            for param_name, expected, check_fn, desc in checks:
                value = None
                for attempt in range(3):
                    with self._mav_lock:
                        self.master.mav.param_request_read_send(self.master.target_system, self.master.target_component, param_name.encode('utf-8'), -1)
                    deadline = time.monotonic() + 2.0
                    while time.monotonic() < deadline:
                        msg = self._wait_msg(q, deadline - time.monotonic())
                        if msg is None:
                            break
                        if msg.param_id.rstrip('\x00') == param_name:
                            value = msg.param_value
                            break
                    if value is not None:
                        break
                if value is None:
                    log.warning('  %-20s  NOT FOUND — SKIP', param_name)
                    continue
                passed = check_fn(value)
                status = 'PASS' if passed else 'FAIL'
                if not passed:
                    all_pass = False
                log.info('  %-20s = %+10.4f  [%s]  (expected %s)', param_name, value, status, desc)
        finally:
            self._unsubscribe(q, 'PARAM_VALUE')
        if all_pass:
            log.info('=== ALL PARAMETER CHECKS PASSED ===')
        else:
            log.warning('=== ONE OR MORE PARAMETER CHECKS FAILED — review above ===')

    def _fetch_param(self, q, name, attempts=3, per_attempt_s=2.0):
        """Request a parameter by name and wait for its PARAM_VALUE reply.

            Sends up to `attempts` param_request_read_send requests, waiting
            `per_attempt_s` seconds after each for a matching reply before retrying.
            Unrelated PARAM_VALUE messages that arrive on the queue (autopilot-
            initiated broadcasts, replies to other readers) are consumed and
            discarded without burning the per-attempt deadline.

            Args:
                q:              queue already subscribed to 'PARAM_VALUE'
                name:           parameter name (str, no trailing nulls)
                attempts:       number of request retransmits before giving up
                per_attempt_s:  timeout per attempt, in seconds

            Returns:
                float parameter value, or None if no matching reply arrived within
                `attempts * per_attempt_s` seconds.
            """
        for _ in range(attempts):
            with self._mav_lock:
                self.master.mav.param_request_read_send(self.master.target_system, self.master.target_component, name.encode('utf-8'), -1)
            deadline = time.monotonic() + per_attempt_s
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                msg = self._wait_msg(q, remaining)
                if msg is None:
                    break
                if msg.param_id.rstrip('\x00') == name:
                    return float(msg.param_value)
        return None

    def _record_mismatch(self, name, found, expected, reason=''):
        """Note a saved FC parameter that differs from what this code expects.
        Logged now and shown with the others at the end of read_fc_config, where
        startup halts for the operator (halt_on_param_mismatches).  Never
        corrected: the operator sets it by hand."""
        log.warning('  %-16s = %-10s ✗  expected %s%s', name, found, expected,
                    ('  ' + reason) if reason else '')
        self._param_mismatches.append((name, found, expected, reason))

    def _check_param(self, name, expected, *, tol=0.1, int_compare=False, reason=''):
        """Read one FC parameter and compare it with `expected`.  READ-ONLY.

        A match logs ✓.  A difference (or an absent parameter) is recorded with
        _record_mismatch and NOT corrected - saved parameters are set by hand.
        `int_compare` matches on rounded integers (mask/enum parameters);
        otherwise the match is |value - expected| <= `tol`.  `reason` says WHY the
        value is required.  Returns the FC's value - the one the aircraft flies
        with - or `expected` when the FC does not report the parameter."""
        q = self._subscribe('PARAM_VALUE')
        try:
            current = self._fetch_param(q, name, attempts=3, per_attempt_s=2.0)
        finally:
            self._unsubscribe(q, 'PARAM_VALUE')
        if current is not None and (
                int(round(current)) == int(round(expected)) if int_compare
                else abs(current - expected) <= tol):
            log.info('  %-16s = %g ✓%s', name, current,
                     ('  ' + reason) if reason else '')
            return current
        self._record_mismatch(name, 'ABSENT' if current is None else '%g' % current,
                              '%g' % expected, reason)
        return expected if current is None else current

    def verify_param_manifest(self, timeout=45.0):
        """Stage 1+2 of the startup parameter gate. Call BEFORE read_fc_config.

        Stage 1 fetches the FC's entire parameter set and requires exact NAME set
        equality against fc_param_manifest.FLIGHT_PARAMS. Stage 2 then checks the
        BUCKET_1_IDENTITY values (servo FUNCTION and REVERSED). Either stage
        failing raises: per-parameter advisories cannot express "wrong aircraft".

        The BUCKET_1B_OUTPUT_CAL values (servo MIN/MAX/TRIM) are calibration, not
        identity - a trim of 1500 is the same on any airframe - so a difference
        there is recorded with _record_mismatch and halts for the operator at the
        end of read_fc_config, like every other value the flight code needs.

        SIM_* names are excluded from Stage 1 in both directions so that one
        manifest serves SITL and real hardware; their count is reported only.
        """
        from ..fc_param_manifest import (
            FLIGHT_PARAMS, SIM_PARAMS, BUCKET_1_IDENTITY, BUCKET_1B_OUTPUT_CAL,
            CONDITIONAL_GROUPS, CONDITIONAL_PARAMS, AIRFRAME_NAME)

        log.info('=== PARAMETER MANIFEST GATE ===')
        log.info('Expecting %d flight parameters for: %s',
                 len(FLIGHT_PARAMS), AIRFRAME_NAME)

        # ---- fetch the full table ----------------------------------------
        # UNBOUNDED queue: the dispatcher discards on queue.Full without error,
        # and this transfer is ~1400 messages against a default depth of 50.
        # Draining one message per 0.5 s wait loses most of the burst.
        found = {}
        expected_count = None
        q = self._subscribe('PARAM_VALUE', maxsize=0)
        try:
            with self._mav_lock:
                self.master.mav.param_request_list_send(
                    self.master.target_system, self.master.target_component)

            deadline = time.monotonic() + timeout
            last_rx = time.monotonic()
            while time.monotonic() < deadline:
                drained = 0
                while True:                      # drain everything available
                    try:
                        msg = q.get_nowait()
                    except queue.Empty:
                        break
                    found[msg.param_id.rstrip('\x00')] = msg.param_value
                    if msg.param_count:
                        expected_count = msg.param_count
                    drained += 1
                if drained:
                    last_rx = time.monotonic()
                    if expected_count and len(found) >= expected_count:
                        break
                elif time.monotonic() - last_rx > 3.0 and found:
                    break                        # transfer has gone quiet
                time.sleep(0.02)

            if expected_count:
                log.info('FC advertised %d parameters, received %d',
                         expected_count, len(found))

            # ---- focused retry -------------------------------------------
            # PARAM_REQUEST_LIST is lossy over a shared link. Rather than treat
            # a dropped packet as a configuration fault, re-request by name any
            # manifest entry still absent. Only a genuinely absent parameter
            # survives this, which is what the gate is meant to detect.
            for attempt in range(3):
                still = sorted(((FLIGHT_PARAMS | set(BUCKET_1_IDENTITY)
                                 | set(BUCKET_1B_OUTPUT_CAL)) - set(found)))
                if not still:
                    break
                log.info('Re-requesting %d parameter(s) individually (pass %d)...',
                         len(still), attempt + 1)
                for name in still:
                    with self._mav_lock:
                        self.master.mav.param_request_read_send(
                            self.master.target_system,
                            self.master.target_component,
                            name.encode('utf-8'), -1)
                    time.sleep(0.004)
                settle = time.monotonic() + 4.0
                while time.monotonic() < settle:
                    try:
                        msg = q.get(timeout=0.2)
                    except queue.Empty:
                        continue
                    found[msg.param_id.rstrip('\x00')] = msg.param_value
                    settle = min(settle, time.monotonic() + 1.0)
        finally:
            self._unsubscribe(q, 'PARAM_VALUE')

        if not found:
            raise RuntimeError(
                'Parameter manifest gate: the flight controller returned no '
                'parameters. Cannot verify the aircraft configuration; refusing '
                'to start.')

        fc_sim = {n for n in found if n.startswith('SIM_')}
        fc_flight = set(found) - fc_sim
        log.info('Received %d parameters (%d flight, %d SIM_*)',
                 len(found), len(fc_flight), len(fc_sim))
        self._fc_param_names = set(found)

        # ---- STAGE 1: name set equality ----------------------------------
        # Conditional subtrees exist only when their enabler is non-zero, and
        # this project's own startup enables two of them (SCR_ENABLE,
        # ARSPD_TYPE) after the gate runs. Require a dependent only when its
        # enabler is currently on; never call one unexpected.
        required = set(FLIGHT_PARAMS)
        active_groups = []
        for enabler, dependents in CONDITIONAL_GROUPS.items():
            if abs(found.get(enabler, 0.0)) > 0.001:
                required |= set(dependents)
                active_groups.append('%s=%g (+%d)' %
                                     (enabler, found[enabler], len(dependents)))
        if active_groups:
            log.info('Conditional groups active: %s', ', '.join(active_groups))
        else:
            log.info('No conditional groups active (%d dependents not required)',
                     len(CONDITIONAL_PARAMS))

        missing = sorted(required - fc_flight)                    # required, absent
        extra   = sorted(fc_flight - required - CONDITIONAL_PARAMS)  # unknown entirely

        if missing or extra:
            log.error('')
            log.error('*** PARAMETER MANIFEST MISMATCH — REFUSING TO START ***')
            log.error('')
            if missing:
                log.error('%d parameter(s) in the manifest are ABSENT from the FC:',
                          len(missing))
                for n in missing:
                    log.error('    - %s', n)
            if extra:
                log.error('%d parameter(s) on the FC are NOT in the manifest:',
                          len(extra))
                for n in extra:
                    log.error('    + %s', n)
            log.error('')
            log.error('These names survived a full download plus three rounds of '
                      'individual re-requests, so they are genuinely absent — not '
                      'lost in transit. A changed parameter SET means the firmware '
                      'or vehicle type is not the one this manifest was frozen '
                      'against. Values were not checked; the name gate failed first.')
            log.error('If this follows a deliberate firmware upgrade, re-freeze '
                      'the manifest from a known-good FC — see the REGENERATING '
                      'note in fc_param_manifest.py.')
            raise RuntimeError(
                'Parameter manifest gate: %d missing, %d unexpected. '
                'Refusing to start against an unverified configuration.'
                % (len(missing), len(extra)))

        log.info('Stage 1 — name set matches exactly (%d parameters) ✓',
                 len(required))
        if fc_sim and not SIM_PARAMS:
            log.info('  (%d SIM_* present, not gated)', len(fc_sim))
        elif not fc_sim:
            log.info('  (no SIM_* present — real hardware, as expected)')
        else:
            log.info('  (%d SIM_* present, not gated — SITL)', len(fc_sim))

        # ---- STAGE 2: identity ------------------------------------------
        wrong = []
        for name, (expected, purpose) in BUCKET_1_IDENTITY.items():
            actual = found.get(name)
            if actual is None or abs(actual - expected) > 0.5:
                wrong.append((name, expected, actual, purpose))
            else:
                log.info('  %-16s = %-4d ✓  %s', name, int(expected), purpose)

        if wrong:
            log.error('')
            log.error('*** WRONG AIRFRAME — REFUSING TO START ***')
            log.error('')
            log.error('Expected: %s', AIRFRAME_NAME)
            for name, expected, actual, purpose in wrong:
                shown = 'ABSENT' if actual is None else '%d' % int(actual)
                log.error('    %-16s expected %-4d  found %s', name, int(expected), shown)
                log.error('        %s', purpose)
            log.error('')
            log.error('Servo outputs are wired for a different aircraft. A wrong '
                      'FUNCTION sends a demand to the wrong surface; a wrong '
                      'REVERSED sends it to the right surface BACKWARDS.')
            log.error('')
            log.error('Most likely cause: eeprom.bin holds values from a previous '
                      'airframe. A --add-param-file supplies DEFAULTS only, and a '
                      'default never overrides a stored value.')
            log.error('Fix: restart SITL with -w to wipe stored parameters before '
                      'loading the airframe file.')
            raise RuntimeError(
                'Parameter manifest gate: wrong airframe (%s). Refusing to start.'
                % ', '.join(w[0] for w in wrong))

        log.info('Stage 2 — airframe identity confirmed ✓')

        # ---- output calibration: halts with the other differences --------
        for name, (expected, purpose) in BUCKET_1B_OUTPUT_CAL.items():
            actual = found.get(name)
            if actual is None or abs(actual - expected) > 0.5:
                self._record_mismatch(name, 'ABSENT' if actual is None else '%g' % actual,
                                      '%d' % int(expected), purpose)
            else:
                log.info('  %-16s = %-4d ✓  %s', name, int(expected), purpose)
        log.info('=== MANIFEST GATE PASSED ===')
        return found

    def _check_rate_pids(self):
        """Compare the FC's rate-loop settings with the two whole sets it can
        hold.  READ-ONLY: they are set by hand or by a mode switch, and this
        never writes them.

          * The Acro set, EXPECTED_RATE_PIDS: passes.
          * The Autopilot set - NATIVE_MODE_RATE_PIDS over the Acro set's
            filters, which is what a switch out of ACRO leaves and what the FC
            keeps when the program stops without its exit hand-back: passes,
            and ACRO takes the stored Acro defaults rather than these
            (_read_native_params).  Adopting them would fly the balloon chaser on the
            autopilot's gains, rate-loop integrators included.
          * Anything else - a set matched only in part: every value that
            differs from EXPECTED_RATE_PIDS is recorded with the other
            mismatches, and read_fc_config halts on them for the operator.

        A param the FC does not register (YAW_RATE_* with YAW_RATE_ENABLE=0) is
        skipped in both sets."""
        log.info('=== RATE PIDS: FC vs constants.EXPECTED_RATE_PIDS or '
                 'NATIVE_MODE_RATE_PIDS (read-only) ===')
        self._fc_holds_autopilot_set = False
        known = self._fc_param_names
        got = {}
        q = self._subscribe('PARAM_VALUE')
        try:
            for name in EXPECTED_RATE_PIDS:
                if known is not None and name not in known:
                    log.info('  %-14s not on this FC (YAW_RATE_* need YAW_RATE_ENABLE=1) '
                             '- skipped', name)
                    continue
                got[name] = self._fetch_param(q, name, attempts=3, per_attempt_s=2.0)
                if got[name] is not None and name in ARDUPLANE_AUTOTUNE_PARAMS:
                    self.native_params[name] = got[name]
        finally:
            self._unsubscribe(q, 'PARAM_VALUE')

        autopilot = {**EXPECTED_RATE_PIDS, **NATIVE_MODE_RATE_PIDS}
        if all(_rate_pid_matches(v, EXPECTED_RATE_PIDS[n]) for n, v in got.items()):
            for name, value in got.items():
                log.info('  %-14s = %-8g ✓', name, value)
            return
        if all(_rate_pid_matches(v, autopilot[n]) for n, v in got.items()):
            self._fc_holds_autopilot_set = True
            for name, value in got.items():
                log.info('  %-14s = %-8g ✓ Autopilot', name, value)
            log.info('  The FC holds the Autopilot gains (NATIVE_MODE_RATE_PIDS). ACRO flies '
                     'the stored Acro gains (ACRO_DEFAULT_PARAMS), written on the first '
                     'switch into ACRO.')
            return
        for name, value in got.items():
            want = float(EXPECTED_RATE_PIDS[name])
            if _rate_pid_matches(value, want):
                log.info('  %-14s = %-8g ✓', name, value)
            else:
                self._record_mismatch(name, 'ABSENT' if value is None else '%g' % value,
                                      '%g' % want, '(EXPECTED_RATE_PIDS)')

    def _read_native_params(self):
        """Read every ARDUPLANE_AUTOTUNE_PARAMS value _check_rate_pids has not
        already read, so the Gain Tuner and the flight log start from what the FC
        holds, then keep a copy as native_params_at_startup and as the
        operator's values.  READ-ONLY, like the rest of startup.

        When the FC holds the Autopilot gains (_check_rate_pids), the
        operator's values for NATIVE_MODE_RATE_PIDS are the stored Acro
        defaults instead of the FC's, so the first switch into ACRO writes
        them."""
        try:
            self._read_remaining_native_params()
        finally:
            self.native_params_at_startup = dict(self.native_params)
            self.native_params_operator = dict(self.native_params)
            if self._fc_holds_autopilot_set:
                for name in NATIVE_MODE_RATE_PIDS:
                    if name in self.native_params_operator:
                        self.native_params_operator[name] = float(ACRO_DEFAULT_PARAMS[name])

    def _read_remaining_native_params(self):
        known = self._fc_param_names
        todo = [n for n in ARDUPLANE_AUTOTUNE_PARAMS
                if n not in self.native_params and (known is None or n in known)]
        if not todo:
            return
        log.info('=== ArduPlane AUTOTUNE parameters (read-only here; the Gain Tuner and '
                 'mode switches write them) ===')
        q = self._subscribe('PARAM_VALUE')
        try:
            for name in todo:
                got = self._fetch_param(q, name, attempts=3, per_attempt_s=2.0)
                if got is None:
                    log.warning('  %-16s no reply - the tuner row stays empty until a Read',
                                name)
                    continue
                self.native_params[name] = got
                log.info('  %-16s = %g', name, got)
        finally:
            self._unsubscribe(q, 'PARAM_VALUE')

    def read_param(self, name):
        """Read one FC parameter for a check, recording nothing.  Returns its
        value, or None when the FC did not answer."""
        q = self._subscribe('PARAM_VALUE')
        try:
            return self._fetch_param(q, name, attempts=3, per_attempt_s=2.0)
        finally:
            self._unsubscribe(q, 'PARAM_VALUE')

    def read_native_param(self, name):
        """Read one saved ArduPlane parameter on the operator's command (the Gain
        Tuner's Read button).  Returns the FC's value and records it as the
        confirmed one, or None when the FC did not answer.  Console only."""
        q = self._subscribe('PARAM_VALUE')
        try:
            got = self._fetch_param(q, name, attempts=3, per_attempt_s=2.0)
        finally:
            self._unsubscribe(q, 'PARAM_VALUE')
        if got is None:
            log.error('[FC PARAM] read %s: no reply from the FC', name)
            return None
        self.native_params[name] = got
        log.info('[FC PARAM] read %s = %s', name, float32_text(got))
        return got

    def read_all_native_params(self):
        """Read every saved ArduPlane parameter the FC has, on the operator's
        command (the Gain Tuner's Read All): the requests go out in batches and
        the replies are taken in whatever order they arrive, as one round trip
        rather than one per parameter.  Each answer is recorded as the confirmed
        value; one that does not answer keeps the value it had, as a single Read
        does.  Like Read it changes nothing ACRO flies.  Returns {name: value}
        for the names that answered.  Console only."""
        known = self._fc_param_names
        names = [n for n in ARDUPLANE_AUTOTUNE_PARAMS if known is None or n in known]
        if not names:
            return {}
        q = self._subscribe('PARAM_VALUE', maxsize=0)
        try:
            got = self._fetch_params(q, names)
        finally:
            self._unsubscribe(q, 'PARAM_VALUE')
        self.native_params.update(got)
        if got:
            log.info('[FC PARAM] Read All: %s', ', '.join(
                '%s %s' % (n, float32_text(got[n])) for n in names if n in got))
        missing = [n for n in names if n not in got]
        if missing:
            log.error('[FC PARAM] Read All: no reply from the FC for %s - '
                      'those boxes keep their previous value', ', '.join(missing))
        return got

    def write_native_param(self, name, value):
        """Set one saved ArduPlane parameter on the operator's command and read it
        back.  The Gain Tuner's Write and reset are the ONLY callers, so this and
        the Default Acro Parameters Write (write_rate_loop_set) are the only
        places the operator's own values (native_params_operator) change; a mode
        switch writes through _write_param_set instead, and startup and shutdown
        write nothing (test_link_contract.py).

        ArduPlane saves every PARAM_SET (GCS_MAVLINK::handle_param_set -> save),
        so the value survives a reboot.  The handler itself sends no reply; the
        PARAM_VALUE the save broadcasts comes later, from the storage thread's
        save queue, so the value is confirmed by an explicit read instead.
        Returns the value the FC then reports, recorded as the confirmed one, or
        None when it did not answer."""
        value = float(value)
        q = self._subscribe('PARAM_VALUE')
        try:
            with self._mav_lock:
                self.master.mav.param_set_send(
                    self.master.target_system, self.master.target_component,
                    name.encode('utf-8'), value,
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            got = self._fetch_param(q, name, attempts=3, per_attempt_s=2.0)
        finally:
            self._unsubscribe(q, 'PARAM_VALUE')
        # ACRO flies this from now on: what the FC reports holding, or, when it
        # did not answer, what was asked for, so the next switch into ACRO writes
        # it again.
        self.native_params_operator[name] = value if got is None else got
        if got is None:
            # Unknown, not the value it held before the write: the box and the
            # log go blank, and the next mode switch writes it again.
            self.native_params.pop(name, None)
            log.error('[FC PARAM] write %s = %s: no reply to the readback - the FC value '
                      'is unknown; press Read', name, float32_text(value))
            return None
        self.native_params[name] = got
        # PARAM_VALUE carries a 32-bit float, so compare at that precision.
        if abs(got - value) <= 1e-6 * max(1.0, abs(value)):
            log.info('[FC PARAM] write %s = %s -> readback %s ✓',
                     name, float32_text(value), float32_text(got))
        else:
            log.warning('[FC PARAM] write %s = %s -> readback %s ✗  the FC holds the '
                        'readback value', name, float32_text(value), float32_text(got))
        return got

    def read_fc_config(self):
        """Read flight-controller configuration from the autopilot.

        Pulls a mix of parameters used by different paths of the code:

          * Pitch envelope (PTCH_LIM_*): clamps commanded angles.
            Operator-adjustable, so re-read at startup.
            ROLL_LIMIT_DEG is NOT read - ArduPlane ignores it in ACRO, and the
            roll law's bank limit is the bank_lim_deg gain.

          * Pitch modifiers and servo trims (STAB_PITCH_DOWN, PTCH_TRIM_DEG,
            KFF_THR2PTCH, SERVO1/2_TRIM): ArduPlane applies these outside the
            control law, so the law cannot account for them.  They MUST be zero
            (the trims at 1500).
            _verify_params() re-reads them as a final confirmation table - it
            logs PASS/FAIL, it does not itself abort.

          * RC override gating (RC_OPTIONS, FS_GCS_ENABL): governs whether
            RC_CHANNELS_OVERRIDE is accepted at all.  The FBWA
            level-out depends on it; _verify_params checks RC_OPTIONS bit0.

          * ACRO rates and smoothing (ACRO_ROLL_RATE, ACRO_PITCH_RATE,
            ACRO_LOCKING, RLL_RATE_FLTE): ACRO_*_RATE is the rate envelope the
            FC's law and set_target_throttle_rate_rpy clip to.  RLL_RATE_FLTE
            must be 5 Hz to reduce ACRO roll shake during fine corrections.

          * Rate-loop settings (<AXIS>_RATE_{FF,P,I,D,IMAX,FLTT,FLTE,FLTD,SMAX},
            every value plant-ID covers): compared by _check_rate_pids with the
            Acro set, EXPECTED_RATE_PIDS, and the Autopilot set,
            NATIVE_MODE_RATE_PIDS; either one whole passes.

          * The estimator (EK3_ENABLE, AHRS_EKF_TYPE): the script flies by the
            AHRS attitude, velocity and wind, which must be EKF3's - the
            aircraft's estimator - not SITL's truth (AHRS_EKF_TYPE 10).

        READ-ONLY.  Every saved parameter here is set BY HAND and only checked:
        _check_param (or _check_rate_pids) reads it, and a difference is recorded,
        never corrected.  At the end, halt_on_param_mismatches lists every
        difference and halts until the operator presses Enter (continue with the
        FC's values) or Ctrl+C (stop).  The one startup write is the stream-group
        rates in log_telemetry_rate_config, which do not survive between runs.
        """
        # SERVO1/2/4_TRIM/MIN/MAX convert SERVO_OUTPUT_RAW into a deflection
        # percentage for the HUD - the only view of what the surfaces do.
        # Grouped by WHY each is read; one dict, one fetch loop below.
        params = {
            # Envelope + ACRO rate scaling the cascade consumes (operator-set).
            'PTCH_LIM_MAX_DEG': ('pitch_lim_up', PITCH_LIM_UP_DEFAULT),
            'PTCH_LIM_MIN_DEG': ('pitch_lim_dn', PITCH_LIM_DN_DEFAULT),
            'ACRO_ROLL_RATE': ('acro_roll_rate', ACRO_ROLL_RATE_DEFAULT),
            'ACRO_PITCH_RATE': ('acro_pitch_rate', ACRO_PITCH_RATE_DEFAULT),
            'RLL_RATE_FLTE': ('_rll_rate_flte', 0.0),
            'ACRO_LOCKING': ('_acro_locking', 0.0),
            # Post-decode pitch modifiers - checked for zero below.
            'STAB_PITCH_DOWN': ('_stab_pitch_down', 2.0),
            'PTCH_TRIM_DEG': ('_ptch_trim_deg', 0.0),
            'KFF_THR2PTCH': ('_kff_thr2ptch', 0.0),
            # Servo trims/ranges/functions - HUD surface-deflection scaling and
            # elevon-vs-conventional detection.
            'SERVO1_TRIM': ('_servo1_trim', 1500.0),
            'SERVO1_MIN': ('_servo1_min', 1100.0),
            'SERVO1_MAX': ('_servo1_max', 1900.0),
            'SERVO2_TRIM': ('_servo2_trim', 1500.0),
            'SERVO2_MIN': ('_servo2_min', 1100.0),
            'SERVO2_MAX': ('_servo2_max', 1900.0),
            'SERVO4_TRIM': ('_servo4_trim', 1500.0),
            'SERVO4_MIN': ('_servo4_min', 1100.0),
            'SERVO4_MAX': ('_servo4_max', 1900.0),
            'SERVO1_FUNCTION': ('_servo1_function', 4.0),
            'SERVO2_FUNCTION': ('_servo2_function', 19.0),
            # Failsafe + MAVLink identity (informational).  MAV_GCS_SYSID /
            # MAV_SYSID are ArduPlane's current names for the former
            # SYSID_MYGCS / SYSID_THISMAV.
            'FS_GCS_ENABL': ('_fs_gcs_enabl', 0.0),
            'MAV_GCS_SYSID': ('_mav_gcs_sysid', 255.0),
            'MAV_SYSID': ('_mav_sysid', 1.0),
        }
        log.info('Reading flight-controller configuration from autopilot...')
        q = self._subscribe('PARAM_VALUE')
        try:
            for name, (attr, default) in params.items():
                value = self._fetch_param(q, name, attempts=3, per_attempt_s=2.0)
                if value is None:
                    setattr(self, attr, default)
                    log.warning('  %-20s  NOT FOUND — using fallback %.4f', name, default)
                    continue
                setattr(self, attr, float(value))
                if abs(value - default) < 0.0001:
                    log.info('  %-20s = %.4f  (matches fallback)', name, value)
                else:
                    log.info('  %-20s = %.4f  (fallback %.4f)', name, value, default)
        finally:
            self._unsubscribe(q, 'PARAM_VALUE')
        self._rll_rate_flte = self._check_param(
            'RLL_RATE_FLTE', RLL_RATE_FLTE_EXPECTED, tol=0.1,
            reason='softens ACRO roll-rate shake during fine corrections.')
        # PTCH_RATE_FLTE: the SAME error-term low-pass on the pitch rate PID that
        # RLL_RATE_FLTE gives roll.  Without it the pitch rate rings while the
        # filtered roll axis tracks cleanly, so both axes need the same plant
        # quality.
        self._check_param(
            'PTCH_RATE_FLTE', PTCH_RATE_FLTE_EXPECTED, tol=0.1,
            reason='pitch rate loop needs the same filtered error term as roll.')

        # SCHED_LOOP_RATE — the FC main-loop rate, and a hard ceiling on every
        # telemetry stream: ATTITUDE and ATTITUDE_QUATERNION CANNOT be produced
        # faster than the loop that generates them. ArduPlane defaults to 50,
        # at which the 50 Hz attitude the cascade reads every tick arrives at
        # 0 Hz and the control loop acts on stale attitude on every tick.
        # The FC diagnoses this itself: "Requested rate for message ID 31 too
        # fast. Increase SCHED_LOOP_RATE" (31 is ATTITUDE_QUATERNION).
        #
        # Takes effect only after an autopilot reboot: ArduPilot latches the loop
        # rate at boot.
        self._check_param(
            'SCHED_LOOP_RATE', SCHED_LOOP_RATE_EXPECTED, int_compare=True,
            reason='the ceiling on every telemetry stream (takes effect after a reboot).')

        # RUDDER: the yaw-coordination params (KFF_RDDRMIX, YAW2SRV_SLIP / _INT /
        # _DAMP / _RLL) are NOT enforced - they are left at whatever the FC holds
        # and only recorded.  They feed calc_nav_yaw_coordinated(), which the
        # nav_scripting path never calls, so they are inert while the Lua script
        # flies; but they ARE live in FBWA (the catapult launch and the autotune
        # level-out), where a non-zero value changes behaviour and nothing here
        # will stop it.
        #
        # V-TAIL: ArduPlane always mixes elevator + rudder onto SERVO2 (79,
        # VTailLeft) and SERVO4 (80, VTailRight); that mix cannot be disabled.
        # With rudder demand at zero it collapses to pure elevator and both
        # ruddervators move together; rudder demand drives them DIFFERENTIALLY,
        # so elevator and rudder share the same two surfaces' travel.
        #
        # ZERO YAW RATE IS NOT THE SAME AS ZERO RUDDER.  With YAW_RATE_ENABLE = 1
        # and the yaw rate controller running, a commanded 0.0 is an active
        # instruction to HOLD zero yaw rate, which the rate PID chases with real
        # rudder against the natural yaw of every banked turn.  balloon_chaser_fc.lua
        # therefore takes the yaw rate controller out of the loop
        # (set_rudder_offset(0, false)) whenever the pointing gains are zero.

        # ACRO_PITCH_RATE: the pitch stack's authority ceiling and integrator
        # anti-windup are designed against a 90 deg/s FC envelope, and the
        # gains were tuned against it.  The FC's own value is what the law then
        # uses.
        self.acro_pitch_rate = self._check_param(
            'ACRO_PITCH_RATE', ACRO_PITCH_RATE_DEFAULT, tol=0.1,
            reason='the cascade pitch stack is calibrated for this FC rate envelope.')
        # GCS_PID_MASK: make the FC stream PID_TUNING for roll, pitch AND yaw
        # (=7) so the rate-loop desired/achieved/PID terms are available for
        # diagnostics.
        #
        # Yaw is not optional here.  The rudder law's entire output is a rate
        # demand into ArduPlane's yaw rate controller, and the script can only
        # ever log what it SENT.  fc_yaw_des is what the FC received after its
        # own +-ACRO_YAW_RATE clamp, fc_yaw_ach is what the aircraft did, and
        # fc_yaw_ff/p/i is how the surface command was split - the only way to
        # check that the autotuned YAW_RATE_FF is not putting half the rudder
        # travel in open loop at the bank limit.
        self._check_param(
            'GCS_PID_MASK', GCS_PID_MASK_EXPECTED, int_compare=True,
            reason='FC streams roll+pitch+yaw PID_TUNING for rate-loop diagnostics.')
        self._stab_pitch_down = self._check_param(
            'STAB_PITCH_DOWN', 0.0, tol=0.01,
            reason='a throttle-linked nose-down elevator bias distorts the cascade pitch commands.')
        for param_name, attr_name in (('PTCH_TRIM_DEG', '_ptch_trim_deg'),
                                      ('KFF_THR2PTCH', '_kff_thr2ptch')):
            setattr(self, attr_name, self._check_param(
                param_name, 0.0, tol=0.001,
                reason='the cascade supplies its own pitch/aim offsets, so FC-side pitch biases must be 0.'))
        self._acro_locking = self._check_param(
            'ACRO_LOCKING', 0.0, tol=0.01,
            reason="ACRO's own path must be the rate controllers the script's demand reaches (C-01).")
        # The attitude, velocity and wind the script flies by are the AHRS
        # estimates.  EKF3 is the estimator the aircraft flies; at
        # AHRS_EKF_TYPE 10 SITL substitutes the simulator's truth, and nothing in
        # the flight itself shows the difference.
        self._check_param(
            'EK3_ENABLE', EK3_ENABLE_EXPECTED, int_compare=True,
            reason='EKF3 must run for AHRS_EKF_TYPE to select it (takes effect after a reboot).')
        self._check_param(
            'AHRS_EKF_TYPE', AHRS_EKF_TYPE_EXPECTED, int_compare=True,
            reason="the script must fly on EKF3's estimates, not the simulator's truth (10).")
        # PTCH_LIM_MIN/MAX: a bound, not an exact value - the envelope must be AT
        # LEAST this wide.  Checked only; never set, and never put back on
        # shutdown.  PTCH_LIM_MAX_DEG (pitch-UP limit) also governs the FBWA
        # full-up-stick launch climb, so a flattened value flattens the catapult
        # launch.
        if self.pitch_lim_dn > PTCH_LIM_MIN_EXPECTED_DEG:
            self._record_mismatch(
                'PTCH_LIM_MIN_DEG', '%g' % self.pitch_lim_dn,
                '<= %g' % PTCH_LIM_MIN_EXPECTED_DEG,
                'steep dive balloon chaser requires the full down-pitch envelope.')
        else:
            log.info('  PTCH_LIM_MIN_DEG = %g ✓', self.pitch_lim_dn)
        if self.pitch_lim_up < PTCH_LIM_MAX_EXPECTED_DEG:
            self._record_mismatch(
                'PTCH_LIM_MAX_DEG', '%g' % self.pitch_lim_up,
                '>= %g' % PTCH_LIM_MAX_EXPECTED_DEG,
                'the steep catapult-launch climb requires the pitch-up envelope.')
        else:
            log.info('  PTCH_LIM_MAX_DEG = %g ✓', self.pitch_lim_up)

        self._check_rate_pids()
        self._read_native_params()

        log.info('Surface and GCS configuration:')
        # The surface percentages in the HUD are (pwm - trim)/(max - trim).  A
        # wrong trim makes a NEUTRAL surface read as deflected, which looks like
        # a control bug and is not one - so print the scaling that will be used.
        log.info('  SERVO1 trim/min/max = %.0f/%.0f/%.0f  function=%d',
                 self._servo1_trim, self._servo1_min, self._servo1_max,
                 int(self._servo1_function))
        log.info('  SERVO2 trim/min/max = %.0f/%.0f/%.0f  function=%d',
                 self._servo2_trim, self._servo2_min, self._servo2_max,
                 int(self._servo2_function))
        log.info('  surface mixing: %s',
                 'ELEVON (roll = differential, pitch = common)'
                 if self.elevon_mixing else 'conventional aileron/elevator')
        log.info('  FS_GCS_ENABL     = %.0f  (GCS failsafe; 0=off)', self._fs_gcs_enabl)
        # Read live from the FC above (ArduPlane's current names; the former
        # SYSID_MYGCS / SYSID_THISMAV).  0/255 mean "accept any GCS"; a specific
        # id that is not ours means the FC may ignore our commands.
        log.info('  MAV_GCS_SYSID    = %.0f  (GCS sysid the FC accepts; our sysid=%d)',
                 self._mav_gcs_sysid, self.master.source_system)
        log.info('  MAV_SYSID        = %.0f  (this vehicle sysid)', self._mav_sysid)
        if (int(self._mav_gcs_sysid) not in (0, 255)
                and int(self._mav_gcs_sysid) != int(self.master.source_system)):
            log.warning('  *** MAV_GCS_SYSID=%d does not match our GCS sysid %d — the '
                        'FC may ignore our commands ***',
                        int(self._mav_gcs_sysid), int(self.master.source_system))
        self._verify_params()
        log.info('Pitch/roll envelope loaded:')
        log.info('  Pitch up  limit : %+.1f°  (PTCH_LIM_MAX_DEG)', self.pitch_lim_up)
        log.info('  Pitch down limit: %+.1f°  (PTCH_LIM_MIN_DEG)', self.pitch_lim_dn)
        log.info('  Bank limit default: %+.1f°  (BANK_LIM_DEG; the Bank Lim tuner row '
                 'flies; ROLL_LIMIT_DEG is ignored in ACRO)', BANK_LIM_DEG)
        log.info('ACRO rate parameters loaded:')
        log.info('  ACRO_ROLL_RATE  : %.1f°/s', self.acro_roll_rate)
        log.info('  ACRO_PITCH_RATE : %.1f°/s', self.acro_pitch_rate)
        log.info('  RLL_RATE_FLTE   : %.1f Hz', self._rll_rate_flte)
        # Last, so the list and the prompt are the final thing on screen.
        halt_on_param_mismatches(self._param_mismatches)

    def log_telemetry_rate_config(self):
        """Set the per-channel stream-group rates and log the parameters that
        govern telemetry delivery rate.

        The MAVx_EXTRA1 / MAVx_RC_CHAN rates are the ONE thing startup writes: a
        ground station's REQUEST_DATA_STREAM overwrites and saves them (ArduPlane
        persists stream-rate requests), so they do not survive between runs and
        must be set every session.  Every other saved FC parameter is only
        checked (read_fc_config).  The rest here is logged for diagnosis: it puts
        the rate-relevant configuration in the startup log (and the CSV header) so the
        ATTITUDE rate you actually observe is explainable without querying the FC by
        hand - most useful on real hardware, where the link is a bandwidth-limited
        serial UART.

        Logged:
          * SCHED_LOOP_RATE - the FC scheduler rate.  ATTITUDE cannot be generated
            faster than this, and under contention the delivered rate is often about
            half (27 Hz at loop=50, 35 Hz at loop=400 in SITL-over-WSL).
          * MAV1/2/3 EXTRA1 + RC_CHAN - per-channel stream-group rates for ATTITUDE
            and SERVO_OUTPUT_RAW.  All three channels are read because which one this
            controller occupies is inferred rather than certain; the log shows which
            carries the rates we requested.
          * SERIAL1/2 BAUD + PROTOCOL - physical link bandwidth and protocol.  On real
            hardware a low baud (e.g. 57600) throttles ATTITUDE the way the SITL TCP
            cap does.
        """
        log.info('Telemetry-rate configuration:')
        # Params we ENFORCE: per-channel EXTRA1 + RC_CHAN stream-group rates.  SITL
        # defaults channel 1 to 4 Hz, which throttled delivered attitude to ~32 Hz
        # against our 50 Hz request.  We raise all three MAV channels' groups to the
        # target so whichever channel this session connects on is uncapped; these
        # only lift stream ceilings and affect nothing else.  Set and verified, but
        # WARN-only on failure
        # (a low stream rate degrades diagnostics, it isn't unflyable) - not all
        # firmwares expose every MAVx param, so a missing one must not abort.
        enforce_rate = ['MAV1_EXTRA1', 'MAV1_RC_CHAN',
                        'MAV2_EXTRA1', 'MAV2_RC_CHAN',
                        'MAV3_EXTRA1', 'MAV3_RC_CHAN']
        q = self._subscribe('PARAM_VALUE')
        try:
            for name in enforce_rate:
                cur = self._fetch_param(q, name, attempts=2, per_attempt_s=1.5)
                if cur is None:
                    log.info('  %-18s = (not on this firmware — skipped)', name)
                    continue
                if abs(cur - MAV_STREAM_RATE_EXPECTED) < 0.5:
                    log.info('  %-18s = %g (OK)', name, cur)
                    continue
                confirmed = False
                for attempt in range(3):
                    with self._mav_lock:
                        self.master.mav.param_set_send(
                            self.master.target_system, self.master.target_component,
                            name.encode(), float(MAV_STREAM_RATE_EXPECTED),
                            mavutil.mavlink.MAV_PARAM_TYPE_INT16)
                    deadline = time.monotonic() + 2.0
                    while time.monotonic() < deadline:
                        msg = self._wait_msg(q, deadline - time.monotonic())
                        if msg is None:
                            break
                        if msg.param_id.rstrip('\x00') == name:
                            if abs(msg.param_value - MAV_STREAM_RATE_EXPECTED) < 0.5:
                                confirmed = True
                                log.info('  %-18s = %g → %d ✓', name, cur, MAV_STREAM_RATE_EXPECTED)
                            break
                    if confirmed:
                        break
                if not confirmed:
                    log.warning('  %-18s could not be set to %d Hz — attitude stream '
                                'may stay throttled (diagnostic quality only).',
                                name, MAV_STREAM_RATE_EXPECTED)
            # Read-only context (NOT ours to force): scheduler + serial link setup.
            for name in ['SCHED_LOOP_RATE', 'SERIAL1_BAUD', 'SERIAL1_PROTOCOL',
                         'SERIAL2_BAUD', 'SERIAL2_PROTOCOL']:
                value = self._fetch_param(q, name, attempts=2, per_attempt_s=1.5)
                log.info('  %-18s = %s', name,
                         '(not found)' if value is None else '%g' % value)
        finally:
            self._unsubscribe(q, 'PARAM_VALUE')
        # request_high_rate_streams() already ran (before read_fc_config), so the
        # first stream request was made while channel 1 was still capped low.  Now
        # that the caps are raised, re-issue the EXTRA1 / RC_CHANNELS group requests
        # so the higher rate takes effect this session without a restart.
        try:
            with self._mav_lock:
                self.master.mav.request_data_stream_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, MAV_STREAM_RATE_EXPECTED, 1)
                self.master.mav.request_data_stream_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, MAV_STREAM_RATE_EXPECTED, 1)
            log.info('  re-requested EXTRA1 + RC_CHANNELS @ %d Hz after raising caps',
                     MAV_STREAM_RATE_EXPECTED)
        except Exception as e:                                    # noqa: BLE001
            log.warning('  stream re-request failed: %s', e)

    def arm(self):
        """Arm the vehicle."""
        log.info('Arming...')
        with self._mav_lock:
            self.master.mav.command_long_send(self.master.target_system, self.master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            if self.armed:
                log.info('Armed!')
                return
            time.sleep(0.05)
        raise TimeoutError('Timed out waiting for arm confirmation after 30s')

    def _update_telemetry(self):
        """Drain pending messages, update aircraft state, dispatch to queues."""
        if not hasattr(self, '_msg_counts'):
            self._msg_counts = {}
            self._msg_count_start = time.monotonic()
        while True:
            with self._mav_lock:
                msg = self.master.recv_match(blocking=False)
            if msg is None:
                break
            msg_type = msg.get_type()
            self._msg_counts[msg_type] = self._msg_counts.get(msg_type, 0) + 1
            now = time.monotonic()
            elapsed = now - self._msg_count_start
            if elapsed >= 10.0:
                rates = {k: v / elapsed for k, v in sorted(self._msg_counts.items(), key=lambda x: -x[1])}
                rate_str = '  '.join((f'{k}={v:.1f}Hz' for k, v in rates.items()))
                # Build sha on EVERY rate line.  It is the most frequent line in
                # the log and the one most likely to be in a paste, so it is the
                # cheapest place to make 'which build is this?' answerable
                # without asking.  A crash report has been diagnosed against the
                # wrong source more than once because a stale extraction looked
                # identical to a current one.
                # Computed once and cached - it walks every .py and .lua on disk.
                log.info('[TELEM RATES] sha %s  %s', _build_sha(), rate_str)
                self._msg_counts = {}
                self._msg_count_start = now
            with self._msg_queues_lock:
                queues = list(self._msg_queues.get(msg_type, []))
            for q in queues:
                try:
                    q.put_nowait(msg)
                except queue.Full:
                    pass
            if msg_type == 'GLOBAL_POSITION_INT':
                self.lat = msg.lat / 10000000.0
                self.lon = msg.lon / 10000000.0
                self.relative_alt = msg.relative_alt / 1000.0
                # NED velocity, cm/s -> m/s (for flight-path angle).
                self.vel_ned = np.array([msg.vx, msg.vy, msg.vz]) / 100.0
                # The FC's validity stamp for this fix.  Written LAST of the
                # position fields so a reader that sees a new stamp is
                # guaranteed to be seeing the matching lat/lon/alt/vel (CPython
                # attribute stores are atomic; the ordering is what makes the
                # stamp a safe freshness key from another thread).
                self.pos_time_boot_s = msg.time_boot_ms / 1000.0
                if self.home_lat is None:
                    self.home_lat = self.lat
                    self.home_lon = self.lon
                    log.info('Home position set: %.6f, %.6f', self.home_lat, self.home_lon)
            elif msg_type == 'ATTITUDE':
                self.attitude = np.array([msg.roll, msg.pitch, msg.yaw])
                self.rates = np.array([msg.rollspeed, msg.pitchspeed, msg.yawspeed])
                self.attitude_recv_t = time.monotonic()
                # The FC's OWN clock for this sample.  Under SITL the simulator
                # does not advance in step with wall time, so a rate reported by
                # the FC and an angle differenced against time.monotonic() are
                # in different time bases - which silently scales every measured
                # frequency, time constant and pole.  Anything that divides by
                # elapsed time must use this, not the sampler's wall clock.
                self.attitude_time_boot_s = msg.time_boot_ms / 1000.0
            elif msg_type == 'SIMSTATE':
                # SITL ground truth: the simulator's ACTUAL attitude and body
                # rates, before any estimator.  ATTITUDE carries the EKF's
                # attitude alongside a gyro-derived rate, and nothing guarantees
                # those two agree - so this is the only reference that can say
                # WHICH of them is wrong when they disagree.  Absent on real
                # hardware, where the fields stay None and log blank.
                self.sim_attitude = np.array([msg.roll, msg.pitch, msg.yaw])
                self.sim_rates = np.array([msg.xgyro, msg.ygyro, msg.zgyro])
            elif msg_type == 'ATTITUDE_QUATERNION':
                # q1..q4 = w,x,y,z (ArduPilot body->NED convention).  Singularity-
                # free attitude for building the body-frame balloon vector.
                self.attitude_quat = np.array([msg.q1, msg.q2, msg.q3, msg.q4])
                self.attitude_quat_recv_t = time.monotonic()
                self.attitude_quat_seq += 1
            elif msg_type == 'VFR_HUD':
                if msg.airspeed > 1.0:
                    self.airspeed = msg.airspeed
            elif msg_type == 'AOA_SSA':
                # ArduPilot's OWN aerodynamic angles, and the SSA is the one
                # that matters: AP_AHRS_Backend.cpp update_AOA_SSA() fetches the
                # wind estimate, rotates it to body, and subtracts it from the
                # velocity BEFORE computing the angle.  So this sideslip is
                # airspeed-referenced, while the script's sideslip_deg
                # (track.lua flow_angles) is ground-referenced and moves with
                # the wind.  Their DIFFERENCE
                # is the wind contamination, which is why both are logged: it
                # is the only way to know whether sideslip_deg means anything
                # on a given flight without a separate wind feed.
                #
                # This costs nothing.  AOA_SSA rides in EXTRA1 with ATTITUDE,
                # and ArduPilot will not honour a per-message disable for a
                # member of a group being actively requested - so it has been
                # arriving at ~32 Hz all along and simply going unread.
                # ArduPilot updates it at 20 Hz internally regardless.
                self.fc_aoa_deg = float(msg.AOA)
                self.fc_ssa_deg = float(msg.SSA)
            elif msg_type == 'SERVO_OUTPUT_RAW':
                self.servo_aileron = msg.servo1_raw
                self.servo_elevator = msg.servo2_raw
                self.servo_ch4 = msg.servo4_raw
                self.servo_ch3 = msg.servo3_raw
                self.servo_ch5 = msg.servo5_raw
                self.servo_ch6 = msg.servo6_raw
                self.servo_ch7 = msg.servo7_raw
                self.servo_ch8 = msg.servo8_raw
            elif msg_type == 'PID_TUNING':
                # FC rate-loop internals, one message per axis (1=roll, 2=pitch,
                # 3=yaw).  Store the latest by axis; the controller snapshots all
                # three into the CSV each tick.  These are the FC's own
                # desired/achieved/PID terms - the definitive inner-loop signals.
                self._pid_tuning[int(msg.axis)] = {
                    'desired': float(msg.desired),
                    'achieved': float(msg.achieved),
                    'ff': float(msg.FF),
                    'p': float(msg.P),
                    'i': float(msg.I),
                    'd': float(msg.D),
                    'srate': float(getattr(msg, 'SRate', 0.0)),
                }
            elif msg_type == 'HEARTBEAT':
                if is_autopilot_heartbeat(msg, self.master.target_system):
                    self.current_mode = msg.custom_mode
                    self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif msg_type == 'SYS_STATUS':
                unhealthy = msg.onboard_control_sensors_enabled & ~msg.onboard_control_sensors_health
                self.prearm_healthy = unhealthy == 0

    def pid_tuning_field(self, axis, key):
        """Latest PID_TUNING value for an axis (1=roll, 2=pitch, 3=yaw) and field
        ('desired','achieved','ff','p','i','d','srate'), or None if the FC hasn't
        streamed that axis yet: written blank, never a 0.0 that reads as a rate
        loop measuring zero.  Never raises - safe to call every control tick."""
        try:
            return float(self._pid_tuning[axis][key])
        except (KeyError, TypeError, ValueError):
            return None

    def wait_until_prearm_ok(self):
        """Wait for pre-arm checks to pass."""
        log.info('Waiting for pre-arm checks...')
        while True:
            if self.prearm_healthy:
                log.info('Pre-arm checks passed.')
                return
            time.sleep(0.5)

    def start_loiter(self):
        """Switch to LOITER mode and hold position."""
        log.info('=== ENTERING LOITER ===')
        self._set_mode(12, 'LOITER')
        log.info('Loitering.')

    def upload_mission(self, lat, lon, alt):
        """Upload a 2-item AUTO mission using the MAVLink mission protocol:
              0: Home (reference, not executed)
              1: NAV_WAYPOINT — flies to the waypoint at cruise altitude
            """
        lat = lat
        lon = lon
        alt = alt
        log.info('=== UPLOADING MISSION ===')
        log.info('  Destination: %.6f, %.6f @ %sm', lat, lon, alt)
        log.info('  Getting current position...')
        while self.lat == 0.0 and self.lon == 0.0:
            time.sleep(0.1)
        home_lat = int(self.lat * 10000000.0)
        home_lon = int(self.lon * 10000000.0)
        items = {0: dict(frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, current=0, autocontinue=1, param1=0, param2=0, param3=0, param4=0, x=home_lat, y=home_lon, z=0), 1: dict(frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, current=1, autocontinue=1, param1=0, param2=50.0, param3=0, param4=0, x=int(lat * 10000000.0), y=int(lon * 10000000.0), z=int(alt))}
        with self._mav_lock:
            self.master.mav.mission_clear_all_send(self.master.target_system, self.master.target_component)
        time.sleep(0.5)
        q = self._subscribe('MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK')
        try:
            with self._mav_lock:
                self.master.mav.mission_count_send(self.master.target_system, self.master.target_component, len(items))
            sent = set()
            deadline = time.monotonic() + 10.0
            while len(sent) < len(items) and time.monotonic() < deadline:
                msg = self._wait_msg(q, deadline - time.monotonic())
                if msg is None:
                    break
                msg_type = msg.get_type()
                if msg_type in ('MISSION_REQUEST_INT', 'MISSION_REQUEST'):
                    seq = msg.seq
                    if seq not in items:
                        log.warning('  WARNING: unexpected request for seq %d', seq)
                        continue
                    it = items[seq]
                    with self._mav_lock:
                        self.master.mav.mission_item_int_send(self.master.target_system, self.master.target_component, seq, it['frame'], it['command'], it['current'], it['autocontinue'], it['param1'], it['param2'], it['param3'], it['param4'], it['x'], it['y'], it['z'])
                    sent.add(seq)
                elif msg_type == 'MISSION_ACK':
                    if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                        log.info('  Mission accepted by autopilot.')
                    else:
                        log.warning('  WARNING: mission rejected (type=%d)', msg.type)
                    break
        finally:
            self._unsubscribe(q, 'MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK')
        if len(sent) < len(items):
            log.warning('  WARNING: only sent %d/%d items', len(sent), len(items))
        log.info('  Mission uploaded.')

    def catapult_launch(self, goal_alt, final_mode=(10, 'AUTO')):
        """FBWA full-throttle climb to ~0.9 * goal_alt, then hand over.

        final_mode: (mode_number, name) entered once the altitude is reached, or
        None to leave the vehicle in FBWA for the caller to place.

        The AUTO default exists for the BALLOON CHASER path, which calls upload_mission()
        first and then flies that mission. ArduPlane REFUSES AUTO when no mission
        is loaded, so any caller without one must pass None — otherwise _set_mode
        waits out its full 30 s and raises.

        Missions live in eeprom exactly as parameters do, so a caller that never
        uploads one can appear to work by inheriting whatever is already stored.
        """
        """Catapult launch using FBWA mode with RC overrides.
            Climbs until target altitude is reached, then hands off to AUTO.

        LAUNCH METHOD - SIMULATION vs REAL LIFE:
        The real aircraft is an X-UAV Mini Talon and is HAND LAUNCHED: thrown by
        an operator, already at flying speed, climbing out under its own power.
        There is no ground roll and no catapult.

        The catapult / full-up-stick FBWA climb here is the closest thing SITL can
        do to that.  A hand launch has no ground roll to model and starts the
        airframe airborne at speed, which is what this sequence approximates - it
        gets the aircraft up quickly and repeatably so we can get straight to
        testing the balloon chaser steering.  Only this getting-airborne phase differs
        between sim and the field; everything downstream (waypoint navigation, the
        balloon chaser/AoA steering, gimbal tracking) is unchanged.

        What the approximation does NOT reproduce: launch attitude and airspeed are
        the thrower's, not a repeatable machine's, so the real handover to AUTO
        starts from a wider spread of initial conditions than SITL ever shows.
        """
        log.info('=== CATAPULT LAUNCH SEQUENCE ===')
        self._set_mode(5, 'FBWA')
        time.sleep(0.5)
        self.arm()
        log.info('  Climbing to %sm (FBWA, full throttle, pitch up)...', goal_alt)
        while True:
            with self._mav_lock:
                self.master.mav.rc_channels_override_send(self.master.target_system, self.master.target_component, 0, 1800, 2000, 0, 0, 0, 0, 0)
            alt = self.relative_alt
            if alt > 0:
                log.info('  Altitude: %.1fm', alt)
                if alt >= goal_alt * 0.9:
                    log.info('  Target altitude reached: %.1fm', alt)
                    break
            time.sleep(0.2)
        with self._mav_lock:
            self.master.mav.rc_channels_override_send(self.master.target_system, self.master.target_component, 0, 0, 0, 0, 0, 0, 0, 0)
        if final_mode is not None:
            self._set_mode(*final_mode)
        else:
            log.info('  holding FBWA — caller will select the next mode')
        log.info('Catapult launch complete!')

    def fly_to_waypoint(self, lat, lon, threshold_m=50):
        """Wait until the aircraft is within 50m of the destination waypoint."""
        log.info('=== FLYING TO WAYPOINT ===')
        while True:
            cur_lat = self.lat
            cur_lon = self.lon
            cur_alt = self.relative_alt
            if cur_lat != 0.0 or cur_lon != 0.0:
                dlat = (lat - cur_lat) * 111319.9
                dlon = (lon - cur_lon) * 111319.9 * math.cos(math.radians(cur_lat))
                dist = math.sqrt(dlat ** 2 + dlon ** 2)
                log.info('  Distance: %.0fm  Alt: %.1fm', dist, cur_alt)
                if dist < threshold_m:
                    log.info('  Waypoint reached!')
                    break
            time.sleep(0.5)


    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def start_telemetry(self):
        """Start the telemetry thread (sole MAVLink reader).  Idempotent."""
        if self._telem_thread is not None and self._telem_thread.is_alive():
            return
        # Report which read path the telemetry loop will use, so attitude-
        # freshness behaviour is visible rather than silently chosen: 'fast'
        # blocks on the socket fd and wakes on message arrival (low latency);
        # 'polled' is the fixed-interval fallback when no selectable fd exists.
        fd = getattr(self.master, 'fd', None)
        if fd is not None:
            log.info("Telemetry read path: FAST (select on socket fd=%s, "
                     "wake-on-arrival)", fd)
        else:
            log.info("Telemetry read path: POLLED (no selectable fd; "
                     "5ms fixed-interval fallback)")
        self._telem_thread = threading.Thread(
            target=self._telemetry_loop, daemon=True, name="telemetry")
        self._telem_thread.start()

    def _telemetry_loop(self):
        """Runs in background thread - sole reader of MAVLink messages.
        Updates state and dispatches messages to subscriber queues.

        Waits for data to ARRIVE on the connection's fd (select, outside the
        mav lock) rather than polling on a fixed sleep.  The thread wakes the
        instant a message lands and stamps attitude_recv_t immediately, while
        keeping CPU low (it blocks in select instead of spinning).  The lock is
        still held only around the non-blocking drain in _update_telemetry, so
        the control thread's sends are never blocked by this wait.

        Falls back to a fixed-sleep poll when the transport exposes no
        selectable fd (some pymavlink connection types return fd=None).
        """
        fd = getattr(self.master, 'fd', None)
        while not self._telem_stop.is_set():
            if fd is not None:
                try:
                    # Block until data is available (or a short timeout so we
                    # stay responsive to shutdown).  OUTSIDE the mav lock.
                    select.select([fd], [], [], 0.02)
                except (OSError, ValueError):
                    # fd went invalid (shutdown/reconnect) - fall back to sleep.
                    time.sleep(0.005)
            else:
                time.sleep(0.005)   # ~200Hz max drain rate (no selectable fd)
            self._update_telemetry()

    def request_high_rate_streams(self):
        """Ask ArduPlane for the messages the controller consumes, at the rate
        each needs.

        Two delivery mechanisms are needed in combination on this ArduPilot:

        1. MAV_CMD_SET_MESSAGE_INTERVAL works for VFR_HUD, GLOBAL_POSITION_INT,
           SYS_STATUS, HEARTBEAT.

        2. ATTITUDE and SERVO_OUTPUT_RAW IGNORE SET_MESSAGE_INTERVAL and stop
           arriving entirely if only that is used.  They live in
           MAV_DATA_STREAM_EXTRA1 (ATTITUDE/AHRS/AHRS2/SIMSTATE) and
           MAV_DATA_STREAM_RC_CHANNELS (SERVO_OUTPUT_RAW/RC_CHANNELS), where the
           stream-group rate is the only knob, so those two groups are requested
           at 50 Hz.  Their companion messages are dispatch-time no-ops in
           _update_telemetry - a small unavoidable bandwidth cost.

        Do NOT send STREAM_ALL=0: other groups already sit at FC defaults
        (~1-5 Hz), so it changes nothing, and STREAM_ALL=50 is a firehose that
        saturates every stream uniformly at ~26 Hz.

        Messages consumed by the controller (from _update_telemetry dispatch):
            ATTITUDE             -> via STREAM_EXTRA1     @50Hz
            SERVO_OUTPUT_RAW     -> via STREAM_RC_CHANNELS @50Hz
            VFR_HUD              -> via SET_MESSAGE_INTERVAL @10Hz
            GLOBAL_POSITION_INT  -> via SET_MESSAGE_INTERVAL @5Hz
            SYS_STATUS           -> via SET_MESSAGE_INTERVAL @2Hz
            HEARTBEAT            -> via SET_MESSAGE_INTERVAL @1Hz (also intrinsic)
        """
        def _set_interval_us(msg_id, interval_us):
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, msg_id, interval_us, 0, 0, 0, 0, 0)

        def _disable_unread_streams():
            """Tell the FC to stop sending what nothing here reads.

            SET_MESSAGE_INTERVAL(-1) per message ID, resolved BY NAME from the
            dialect (MAVLINK_MSG_ID_<NAME>) rather than hard-coded numbers, so
            an ArduPilot-only message missing from a base dialect is skipped
            instead of sending a wrong ID.  Runs BEFORE the wanted streams are
            requested, so a per-message request always has the last word."""
            names = MAVLINK_DISABLE_STREAMS or ()
            if not names:
                return
            done, unknown = [], []
            for nm in names:
                mid = getattr(mavutil.mavlink, 'MAVLINK_MSG_ID_' + nm, None)
                if mid is None:
                    # The default dialect is the common set; ArduPilot-only
                    # messages (AOA_SSA and friends) live in ardupilotmega.
                    try:
                        from pymavlink.dialects.v20 import ardupilotmega as _apm
                        mid = getattr(_apm, 'MAVLINK_MSG_ID_' + nm, None)
                    except Exception:                  # noqa: BLE001
                        mid = None
                if mid is None:
                    unknown.append(nm)
                    continue
                try:
                    _set_interval_us(mid, -1)          # -1 = stop sending
                    done.append(nm)
                except Exception:                      # noqa: BLE001
                    unknown.append(nm)
            log.info('Disabled %d unread MAVLink stream(s) to free send budget: %s',
                     len(done), ', '.join(done))
            if unknown:
                log.info('  not present in this dialect (skipped): %s',
                         ', '.join(unknown))

        # MAVLink message IDs (from common.xml) - for SET_MESSAGE_INTERVAL.
        ID_HEARTBEAT             = 0
        ID_SYS_STATUS            = 1
        ID_GLOBAL_POSITION_INT   = 33
        ID_VFR_HUD               = 74
        ID_ATTITUDE_QUATERNION   = 31
        ID_PID_TUNING            = 194

        with self._mav_lock:
            # 1. Stream-group requests for the two messages SET_MESSAGE_INTERVAL
            #    won't deliver: ATTITUDE (EXTRA1) and SERVO_OUTPUT_RAW (RC_CHANNELS).
            self.master.mav.request_data_stream_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 50, 1)        # ATTITUDE @ 50Hz
            self.master.mav.request_data_stream_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 50, 1)   # SERVO_OUTPUT_RAW @ 50Hz

            # 2. Per-message intervals for the four that SET_MESSAGE_INTERVAL
            #    handles correctly.
            _set_interval_us(ID_VFR_HUD,            100000)  # 10 Hz - airspeed
            _set_interval_us(ID_GLOBAL_POSITION_INT, 200000) # 5 Hz  - nav
            _set_interval_us(ID_SYS_STATUS,          500000) # 2 Hz  - prearm
            _set_interval_us(ID_HEARTBEAT,          1000000) # 1 Hz  - mode/armed
            # ATTITUDE_QUATERNION: singularity-free attitude for the body vector.
            # Shares EXTRA1 with ATTITUDE (so already streaming), but request it
            # explicitly at 50Hz in case the FC gates it off by default.
            _set_interval_us(ID_ATTITUDE_QUATERNION, 20000)  # 50 Hz
            # PID_TUNING: FC rate-loop internals (needs GCS_PID_MASK != 0, checked
            # at startup).  25 Hz is plenty to characterize the rate loop and keeps
            # bandwidth modest since it streams one message PER enabled axis.
            _set_interval_us(ID_PID_TUNING,          40000)  # 25 Hz per axis

            # 4. Stop the streams nothing reads.  This MUST come last: a
            #    stream-GROUP request enables every member of that group, so
            #    request_data_stream(EXTRA1) re-enables SIMSTATE and AHRS2 and
            #    request_data_stream(RC_CHANNELS) re-enables RC_CHANNELS.  Doing
            #    this first leaves all three arriving at the full group rate.
            #    Running last, the per-message interval is the final word on
            #    exactly the messages named, and the group rate still governs
            #    the members not named (ATTITUDE, SERVO_OUTPUT_RAW).
            _disable_unread_streams()
        log.info("Streams: EXTRA1+RC_CHANNELS @50Hz (ATTITUDE, SERVO_OUTPUT_RAW); "
                 "ATTITUDE_QUATERNION @50Hz, PID_TUNING @25Hz/axis, VFR_HUD @10Hz, "
                 "GLOBAL_POSITION_INT @5Hz, SYS_STATUS @2Hz, HEARTBEAT @1Hz")
        # Verify the attitude stream ACTUALLY arrives near the requested 50 Hz.
        # REQUESTING A RATE DOES NOT SET IT.  SET_MESSAGE_INTERVAL at 50 Hz has
        # been observed delivering ~33 Hz effective, with attitude and body-rate
        # values repeating unchanged on ~35% of control ticks (mean hold ~1.5
        # ticks).  Two consequences, both invisible unless measured:
        #
        #   1. HIDDEN LATENCY.  A loop consuming this acts on a stale attitude on
        #      roughly one tick in three: 10-20 ms of added lag, a real component
        #      of the ~200 ms command-to-response transport.
        #
        #   2. DIFFERENTIATED CHANNELS BECOME NOISE.  Rate telemetry carries
        #      16-24% of its spectral energy above 5 Hz against 1-2% for the ANGLE
        #      it came from, and its std (13-15 deg/s) exceeds the
        #      angle-derivative std (9-11 deg/s).  That excess is the staircase,
        #      not the airframe.  An apparent few-Hz oscillation in a raw rate FFT
        #      is this artifact; a REAL limit cycle appears identically in the
        #      angle, the rate COMMAND and the servo output.  Characterise
        #      oscillations from those, never from a raw rate channel.
        #
        # The steering loop no longer consumes any of this - the FC reads its own
        # sensors on-chip - but the HUD, the CSV and every analysis run against
        # them still do.  Measure for 2 s and WARN LOUDLY below 45 Hz so the
        # assumption is checked instead of silent.  The underlying cause of the
        # shortfall (FC scheduling, stream budget, SR-param precedence) is not
        # diagnosed; the warning exists because it recurs.
        att_q = self._subscribe('ATTITUDE_QUATERNION')
        try:
            t_end = time.monotonic() + 2.0
            n_att = 0
            while time.monotonic() < t_end:
                if self._wait_msg(att_q, t_end - time.monotonic()) is not None:
                    n_att += 1
            att_rate_hz = n_att / 2.0
            if att_rate_hz < 45.0:
                log.warning('ATTITUDE_QUATERNION measured at %.1f Hz (requested 50). '
                            'This is the WIRE feed the COMPANION reads (vision, '
                            'estimator feed-forward, HUD, CSV); the FC flies on its '
                            'OWN on-chip attitude (hz_quat ~49), so the inner control '
                            'loop is NOT affected. Off-chip consumers see hold-and-jump '
                            'staircase telemetry - judge attitude liveness from '
                            'mean(quat_fresh) over a whole flight, not this 2 s probe, '
                            'and do not trust raw-rate oscillation FFTs. The usual '
                            'cause is host load (SITL + Gazebo + companion sharing one '
                            'machine), not a setting.', att_rate_hz)
            else:
                log.info('ATTITUDE_QUATERNION measured at %.1f Hz ✓', att_rate_hz)
        finally:
            self._unsubscribe(att_q, 'ATTITUDE_QUATERNION')

    def send_gcs_heartbeat(self):
        """GCS heartbeat - required so ArduPlane registers our sysid as a valid
        GCS and accepts RC_CHANNELS_OVERRIDE."""
        with self._mav_lock:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0)

    # ------------------------------------------------------------------
    # World position helper
    # ------------------------------------------------------------------
    def aircraft_world_pos_ned(self):
        """[north, east, down] metres relative to home, or None if no GPS lock.
        Flat-earth approximation around home; adequate to a few km."""
        if self.home_lat is None or self.home_lon is None:
            return None
        north_m = (self.lat - self.home_lat) * 111320.0
        east_m = ((self.lon - self.home_lon)
                  * 111320.0 * math.cos(math.radians(self.home_lat)))
        down_m = -self.relative_alt
        return np.array([north_m, east_m, down_m])

    # ------------------------------------------------------------------
    # Tracking mode + rate command
    # ------------------------------------------------------------------
    def set_mode_acro(self, hold=None):
        """Switch to ACRO (mode 4) and wait for HEARTBEAT confirmation.  `hold`
        re-sends a caller's stick overrides once the mode is confirmed and
        through the rate-loop write (_set_mode).

        Commands UNCONDITIONALLY.  The previous `if current_mode != 4` guard
        skipped the command when the cached mode already said ACRO, but
        current_mode is fed by HEARTBEAT (~1 Hz), so it can be stale by up to a
        second - long enough to skip a command the aircraft genuinely needed
        after a mode change it had not yet reported.  Commanding regardless
        costs one MAVLink message: _set_mode sends, then its confirm loop exits
        on the FIRST check when the FC is already in the mode.  Symmetric with
        set_mode_loiter, which never had the guard."""
        self._set_mode(4, 'ACRO', hold=hold)

    def set_mode_loiter(self, keep_operator_params=False):
        """Switch to LOITER (mode 12) and wait for HEARTBEAT confirmation.

        keep_operator_params leaves the FC holding the operator's rate-loop
        values instead of NATIVE_MODE_RATE_PIDS.  That is the program's exit
        hand-back: the FC saves every write, so whatever it holds at exit is what
        the next startup reads as the operator's values."""
        self._set_mode(12, 'LOITER',
                       params=self.native_params_operator if keep_operator_params else None)

    def level_and_hold(self, roll_tol_deg, yaw_rate_tol_dps, hold_s, timeout_s,
                       throttle_frac=0.5, abort=None,
                       pitch_tol_deg=None, pitch_rate_tol_dps=None):
        """Fly STRAIGHT-AND-LEVEL in FBWA (the FC self-levels the wings) and HOLD it,
        VERIFYING from telemetry that it really is: the roll ANGLE and body YAW RATE
        must both stay within tolerance continuously for hold_s, and when
        pitch_tol_deg / pitch_rate_tol_dps are given the pitch ANGLE and RATE too, so
        the hold cannot pass while the nose is still pulling out of a dive (wings
        level but belly NOT parallel to the ground).  Returns True if the hold was
        achieved, False on timeout.  The caller restores ACRO afterwards.

        CH3 carries the OPERATOR's static throttle (throttle_frac, from the GUI
        throttle_pct) so the aircraft keeps the airspeed it flies with elsewhere; a
        fixed cruise PWM here slows it and makes the level-out misbehave.  FBWA only
        needs the roll/pitch sticks centered to command wings-level.

        _set_mode blocks until the FC HEARTBEAT confirms FBWA is active, so the
        centered override is never misread as an ACRO zero-rate command.

        The sweep calls this between trials so every trial starts from the same clean
        attitude - a trial that ended banked or diving would bias the next trial's
        measured line-up.

        Publishes self._straighten_level_since (monotonic instant the aircraft became
        and has stayed wings-level, else None) every loop, so the HUD "STRAIGHT" timer
        shows HELD-level time only: hidden while still rolling toward level, starting
        at 0 once the belly is parallel to the ground.
        """
        def _level_sticks():
            self.hold_level_sticks(throttle_frac)

        # Blocks until FBWA is confirmed active (HEARTBEAT); the sticks go on the
        # moment it is, before the mode's rate-loop write.
        self._set_mode(5, 'FBWA', hold=_level_sticks)
        t0 = time.monotonic()
        level_since = None
        self._straighten_level_since = None      # HUD: not level yet (timer hidden)
        achieved = False
        while True:
            now = time.monotonic()
            if abort is not None and abort.is_set():
                break
            if now - t0 >= timeout_s:
                log.info('  [LEVEL] level-out timed out after %.0fs at roll=%.1f deg, '
                         'pitch=%.1f deg, yaw_rate=%.1f deg/s, pitch_rate=%.1f deg/s',
                         timeout_s, math.degrees(self.attitude[0]),
                         math.degrees(self.attitude[1]), math.degrees(self.rates[2]),
                         math.degrees(self.rates[1]))
                break
            _level_sticks()
            roll_deg = abs(math.degrees(self.attitude[0]))
            yaw_rate_deg = abs(math.degrees(self.rates[2]))
            pitch_deg = abs(math.degrees(self.attitude[1]))
            pitch_rate_deg = abs(math.degrees(self.rates[1]))
            # Wings-level + yawless, AND (if asked) nose near level + nose not moving, so
            # the belly is actually parallel to the ground - not still pulling out of a dive.
            level = roll_deg <= roll_tol_deg and yaw_rate_deg <= yaw_rate_tol_dps
            if pitch_tol_deg is not None:
                level = level and pitch_deg <= pitch_tol_deg
            if pitch_rate_tol_dps is not None:
                level = level and pitch_rate_deg <= pitch_rate_tol_dps
            if level:
                if level_since is None:
                    level_since = now      # belly just became parallel -> start the count
                if now - level_since >= hold_s:
                    achieved = True
                    break                  # telemetry confirms straight-and-level for hold_s
            else:
                level_since = None         # disturbed -> restart the hold (timer hides)
            self._straighten_level_since = level_since   # publish for the HUD timer
            time.sleep(0.05)
        self._straighten_level_since = None      # level-out over -> hide the timer
        with self._mav_lock:               # release roll/pitch/rudder/throttle overrides
            self.master.mav.rc_channels_override_send(
                self.master.target_system, self.master.target_component,
                0, 0, 0, 0, 0, 0, 0, 0)
        return achieved

    def hold_level_sticks(self, throttle_frac):
        """One RC override: roll, pitch and rudder centred, CH3 at throttle_frac
        (0..1) - wings level in FBWA, zero rates in ACRO, neutral surfaces in
        MANUAL.  A caller holding them re-sends; release_rc_overrides ends it."""
        ch3 = max(1000, min(2000, int(1000 + throttle_frac * 1000)))
        with self._mav_lock:
            self.master.mav.rc_channels_override_send(
                self.master.target_system, self.master.target_component,
                1500, 1500, ch3, 1500, 0, 0, 0, 0)

    def climb_to(self, goal_alt, abort=None):
        """Climb to goal_alt (m AGL) using FBWA + full throttle + full pitch-up
        (the same getting-altitude pattern as catapult_launch, but mid-flight and
        abort-aware), then clear the RC overrides.  Leaves the aircraft in FBWA with
        overrides released; the caller restores its own flight mode (e.g. ACRO).

        Used by the Controller-Autotune altitude protection: aggressive dive trials
        bleed height, so when AGL drops below the floor the sweep climbs back to a
        safe ceiling here before re-flying.  Already airborne and armed, so no arm."""
        log.warning('  [PROTECT] climbing to %.0fm (FBWA, full throttle)...', goal_alt)

        def _climb_sticks():                 # full throttle, pitch stick up
            with self._mav_lock:
                self.master.mav.rc_channels_override_send(
                    self.master.target_system, self.master.target_component,
                    0, 1800, 2000, 0, 0, 0, 0, 0)

        # The climb starts the moment FBWA is confirmed, not after the mode's
        # rate-loop write: an aircraft below the floor must not wait on a
        # parameter round trip.
        self._set_mode(5, 'FBWA', hold=_climb_sticks)
        while True:
            if abort is not None and abort.is_set():
                log.warning('  [PROTECT] climb aborted at %.0fm', self.relative_alt)
                break
            _climb_sticks()
            alt = self.relative_alt
            if alt >= goal_alt:
                log.warning('  [PROTECT] reached %.0fm', alt)
                break
            time.sleep(0.2)
        with self._mav_lock:                 # release overrides (all channels)
            self.master.mav.rc_channels_override_send(
                self.master.target_system, self.master.target_component,
                0, 0, 0, 0, 0, 0, 0, 0)

    def release_rc_overrides(self):
        """Send all-zero RC override so the FC regains stick authority.
        Called on the catapult handoff, from go_to_loiter, and in run()'s finally."""
        with self._mav_lock:
            self.master.mav.rc_channels_override_send(
                self.master.target_system, self.master.target_component,
                0, 0, 0, 0, 0, 0, 0, 0)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def shutdown(self):
        """End-of-run hook.  Changes no FC parameter: startup leaves every saved
        parameter as the operator set it, so there is nothing to put back."""
