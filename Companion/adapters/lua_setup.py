# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
"""Scripting-parameter CHECK for the Lua steering path.

Reads SCR_ENABLE / SCR_HEAP_SIZE / SCR_VM_I_COUNT / SERIALx_* / ARSPD_* /
ACRO_YAW_RATE / YAW_RATE_ENABLE and compares each with the value the Lua path
needs.  READ-ONLY: these are saved FC parameters, set by hand (MAVProxy
`param set`), and several take effect only after an autopilot reboot.  Any
difference is listed and startup halts for the operator
(mavlink_link.halt_on_param_mismatches).

WHY CHECK AT ALL
  A parameter that is silently wrong is the worst case here: SCR_ENABLE off
  means the script never runs, nothing answers GUIDE, and ACRO flies neutral
  sticks while the operator believes the balloon chaser is flying.  Reading them back
  is what makes "the Lua is flying" an observation instead of an assumption.

DEPLOYMENT IS SEPARATE
  Copying the .lua files is lua_deploy.deploy(), which runs first and verifies
  every file by SHA-256 after writing.  main.py passes its `changed` result in
  as force_reboot, because ArduPilot reads scripts once at VM start - correct
  parameters are not enough to make NEW code run, so the VM is reloaded in
  place (restart_scripting).
"""

from __future__ import annotations

import logging
import time

log = logging.getLogger(__name__)

# Sized for the steering core: the ArduPilot aerobatics applet - a far smaller
# script - asks for 250000 heap and 200000 instructions.  This one carries the
# estimator, both cascades, the geometry and 183 constants.
SCR_HEAP_SIZE = 300000
SCR_VM_I_COUNT = 200000
SERIAL_PROTOCOL_SCRIPTING = 28
# ArduPilot encodes SERIALn_BAUD in units of 1000.  MUST match port:begin() in
# balloon_chaser_fc.lua and the pyserial baud in main.py - three places, one number.
#
# port:begin() does set the rate, so a mismatch here is not immediately fatal:
# the script re-bauds the port after boot and the link comes up anyway.  That is
# exactly what makes it worth keeping correct - a wrong value here is invisible
# in flight and shows up only as a parameter that disagrees with the code.
SERIAL_BAUD_921K = 921


def _read_param(link, name: str, timeout: float = 3.0):
    q = link._subscribe('PARAM_VALUE')
    try:
        with link._mav_lock:
            link.master.mav.param_request_read_send(
                link.master.target_system, link.master.target_component,
                name.encode('utf-8'), -1)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = link._wait_msg(q, deadline - time.monotonic())
            if msg is None:
                break
            if msg.param_id.rstrip('\x00') == name:
                return msg.param_value
        return None
    finally:
        link._unsubscribe(q, 'PARAM_VALUE')


# MAV_CMD_SCRIPTING param1 values, from AP_Scripting::handle_command_int_packet
SCRIPTING_CMD_STOP = 2
SCRIPTING_CMD_STOP_AND_RESTART = 3

# Sent at most ONCE per process - see restart_scripting() for why a second send
# is destructive rather than merely redundant.
_restart_sent = False
MAV_CMD_SCRIPTING = 42701


def restart_scripting(link, timeout: float = 5.0) -> bool:
    """Reload every Lua script WITHOUT rebooting the autopilot.

    AP_Scripting::thread() runs the VM inside a `while (true)` loop and rebuilds
    it whenever `_stop` is set - so MAV_CMD_SCRIPTING with
    SCRIPTING_CMD_STOP_AND_RESTART tears down the interpreter and re-scans the
    scripts directory in place.

    This is what makes an edit-test loop bearable.  A full reboot works too, but
    over SITL's TCP links it drops the connection permanently and forces a
    sim_vehicle.py restart, which is an absurd price for changing one line of
    Lua.  ArduPilot provides the cheaper mechanism; use it.
    """
    # MAV_CMD_SCRIPTING = 42701, verified against the ardupilotmega dialect:
    #   <entry value="42701" name="MAV_CMD_SCRIPTING">
    # param1 is SCRIPTING_CMD; 3 = STOP_AND_RESTART.
    #
    # SEND IT ONCE.  AP_Scripting::thread() is:
    #
    #     while (true) {
    #         _stop = false; _restart = false;
    #         lua = new lua_scripts(...); lua->run();      // loads the scripts
    #         "stopped"; delete lua;
    #         while (true) { delay(1000); if (_restart) { "restarted"; break; } }
    #     }
    #
    # A second STOP arriving while the VM is being rebuilt makes run() exit
    # immediately - and _restart has been cleared by then, so it parks in the
    # 1 Hz wait loop indefinitely.  Scripting goes silent and STAYS silent.
    # That is exactly what happened: balloon_chaser_fc.lua AND hello_selftest.lua both
    # stopped producing output after a double send.
    global _restart_sent
    if _restart_sent:
        log.info('scripting restart already requested this run - not sending '
                 'again (a second STOP would park the VM)')
        return True
    _restart_sent = True

    log.info('restarting the scripting engine (no reboot - scripts reload in place)')
    with link._mav_lock:
        link.master.mav.command_long_send(
            link.master.target_system, link.master.target_component,
            MAV_CMD_SCRIPTING, 0,
            float(SCRIPTING_CMD_STOP_AND_RESTART), 0, 0, 0, 0, 0, 0)
    # The wait loop polls _restart at 1 Hz; then the VM is rebuilt and every
    # script re-read.  Allow for the poll plus the scan.
    time.sleep(4.0)
    return True


def _is_sitl_tcp(link) -> bool:
    """True when the MAVLink connection is a TCP socket, i.e. SITL."""
    dev = getattr(getattr(link, 'master', None), 'address', None) \
        or getattr(link, 'connection_string', '') \
        or getattr(getattr(link, 'master', None), 'device', '')
    return 'tcp' in str(dev).lower()


def _ensure_pump(link) -> None:
    """Make sure something is feeding the subscriber queues.

    _subscribe()/_wait_msg() are fed by the TELEMETRY THREAD.  With no thread
    running, every queue stays empty, every _wait_msg times out, and a perfectly
    good param_set looks like "could not be set after 3 attempts" - which is
    exactly how this failed the first time.  read_fc_config() never hits it
    because the controller starts telemetry before calling it.
    """
    t = getattr(link, '_telem_thread', None)
    if t is not None and t.is_alive():
        return
    try:
        # start_telemetry() is already idempotent - it returns early when the
        # thread is alive - so calling it here is safe even if the controller
        # calls it again later.
        link.start_telemetry()
        time.sleep(0.5)                 # let the first messages arrive
        log.info('started the telemetry thread for parameter configuration')
    except Exception as exc:            # noqa: BLE001
        log.warning('could not start telemetry before configuring: %s', exc)


def ensure_scripting(link, serial_index: int = 5, *,
                     force_reboot: bool = False) -> bool:
    """Check the FC's saved configuration for the Lua steering path.

    READ-ONLY: every parameter here is saved on the FC and set BY HAND.  Each is
    read and compared; any difference is listed and startup halts until the
    operator presses Enter (continue with the FC's values) or Ctrl+C (stop) -
    halt_on_param_mismatches.  Nothing is written and the autopilot is never
    rebooted.  When the deployed scripts changed (`force_reboot`), the Lua VM is
    reloaded in place so the new code runs - that is a command, not a parameter.
    Returns what restart_scripting returns in that case, True otherwise.

    `serial_index` is the SERIALn used for the steering link.  It must NOT be
    the one carrying --connect: that is the MAVLink telemetry stream, and
    repurposing it to protocol 28 disconnects the companion mid-configuration.
    """
    _ensure_pump(link)

    want = {
        'SCR_ENABLE':      1,
        # AIRSPEED.  With ARSPD_TYPE 0 there is no airspeed SENSOR at all -
        # ahrs:airspeed_EAS() is an alias for airspeed_estimate(), the same
        # function ArduPlane's own speed scaler calls, and it returns false.
        # So the scaler falls back to throttle-based scaling and the script's
        # SCALING_SPEED pin never fires: loop gain then moves with THROTTLE,
        # which is exactly what the pin exists to prevent.
        # 100 = TYPE_SITL.  The backend reads
        # AP::sitl()->state.airspeed_raw_pressure, which SITL fills from the
        # airspeed it derives itself: Gazebo does NOT send airspeed in the JSON
        # (its field list is timestamp/gyro/accel/position/quaternion/velocity),
        # so SIM_JSON.cpp takes the else branch and computes it from the velocity
        # vector with zero wind.  A real number therefore exists to sense.
        #
        # Reboot-required, like SCR_ENABLE.
        'ARSPD_TYPE':      100,
        # SIM_RATE_HZ must match the physics step Gazebo actually delivers
        # (max_step_size 0.001 = 1000 Hz).  It does NOT affect pacing here -
        # SITL prints "Forcing use_time_sync=0", so sync_frame_time() never runs
        # - but ArduPilot reports simulation speedup as
        # achieved_rate_hz / rate_hz, so a mismatch makes a HEALTHY run read
        # 0.83 and puts that ratio in the dataflash ASpdU field.  The one number
        # you would use to spot a struggling simulator is then wrong by 17%.
        'SIM_RATE_HZ':     1000,
        # Bitmask, reboot-required: bit0 disables ROMFS, bit1 disables the
        # APM/scripts directory.  With bit1 set the engine runs and loads
        # NOTHING, and there is no error - the scripts simply never appear.
        # Indistinguishable from a wrong path unless it is checked.
        'SCR_DIR_DISABLE': 0,
        'SCR_HEAP_SIZE':   SCR_HEAP_SIZE,
        'SCR_VM_I_COUNT':  SCR_VM_I_COUNT,
        f'SERIAL{serial_index}_PROTOCOL': SERIAL_PROTOCOL_SCRIPTING,
        f'SERIAL{serial_index}_BAUD':     SERIAL_BAUD_921K,
        # YAW.  These are the two gates between a commanded yaw rate and the
        # rudder, and BOTH fail silently - the demand is discarded with no
        # warning anywhere, which reads as 'the law does nothing'.
        #
        # ACRO_YAW_RATE is the exact counterpart of ACRO_ROLL_RATE and
        # ACRO_PITCH_RATE - same GSCALAR family, same meaning, read by the same
        # cache in balloon_chaser_fc.lua.  The only difference is the shipped default:
        # 180 and 180 for roll and pitch, 0 for yaw.  ArduPlane applies
        # constrain_float(yaw_rate, -ACRO_YAW_RATE, +ACRO_YAW_RATE) inside
        # set_target_throttle_rate_rpy, so at 0 every yaw demand is constrained
        # to +-0 and deleted on arrival.  45 deg/s against a measured worst-case
        # demand of 24 deg/s - roughly the same headroom roll and pitch get.
        'ACRO_YAW_RATE':   45,
        # YAW_RATE_ENABLE has NO roll or pitch counterpart, and that asymmetry
        # is real rather than an oversight: it is the only *_RATE_ENABLE in the
        # whole parameter set.  Roll and pitch have exactly one inner loop, so
        # there is nothing to select.  Yaw has two mutually exclusive paths -
        # the rate controller, or calc_nav_yaw_coordinated() driven by
        # YAW2SRV_* - and this picks which one runs.  At 0 the rudder branch in
        # Attitude.cpp is skipped entirely and rudder is written as 0 every tick.
        #
        # Reboot-required in effect, like SCR_ENABLE: it carries
        # AP_PARAM_FLAG_ENABLE, so the 13 YAW_RATE_* PID parameters do not exist
        # in the parameter tree until it is non-zero.  fc_param_manifest's
        # CONDITIONAL_GROUPS entry is what stops their sudden appearance from
        # aborting the startup gate.
        'YAW_RATE_ENABLE': 1,
    }
    # These do not EXIST until ARSPD_TYPE is non-zero at boot - the parameter
    # tree registers them with the backend - so they read ABSENT until
    # ARSPD_TYPE is right and the autopilot has rebooted.
    #
    #   ARSPD_USE     1 = actually USE the reading for control, not just report
    #                 it.  Without this the sensor exists and changes nothing.
    #   ARSPD_RATIO   pressure-to-speed calibration; SITL needs the standard
    #                 value, and a stale one from a previous airframe would
    #                 scale every reading.
    #   ARSPD_AUTOCAL 0 = do not let ArduPilot re-learn the ratio in flight.
    #                 In-flight recalibration would move the speed scaler under
    #                 the steering while it is being tuned.
    _post = {'ARSPD_USE':     1,
             'ARSPD_RATIO':   2.0,
             'ARSPD_AUTOCAL': 0}

    from .mavlink_link import halt_on_param_mismatches
    mismatches = []
    for name, value in list(want.items()) + list(_post.items()):
        cur = _read_param(link, name)
        # tol scales with magnitude: heap sizes are large ints, flags are 0/1
        tol = 0.5 if abs(float(value)) < 100 else 1.0
        if cur is not None and abs(cur - float(value)) <= tol:
            log.info('  %-20s = %g ✓', name, cur)
        else:
            shown = 'ABSENT' if cur is None else f'{cur:g}'
            log.warning('  %-20s = %s ✗  expected %g', name, shown, float(value))
            mismatches.append((name, shown, f'{float(value):g}',
                               '(Lua path; ARSPD_USE/RATIO/AUTOCAL exist only once '
                               'ARSPD_TYPE is non-zero at boot)' if name in _post
                               else '(Lua path)'))
    halt_on_param_mismatches(mismatches)
    if force_reboot:
        # Only the SCRIPTS changed.  That needs the Lua VM reloaded, not the
        # autopilot rebooted - and the in-place restart does not drop the link.
        return restart_scripting(link)
    return True
