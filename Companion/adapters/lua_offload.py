# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
"""
LuaOffloadLink - the companion end of the Lua steering path.

Wraps an existing AutopilotLink rather than replacing it.  The wrapped link
keeps doing everything it already does - telemetry, parameters, arming, the
catapult launch, the attributes the HUD and CSV read - and this class adds the
steering link: send_guide() streams a GUIDE frame every tick carrying the
sensor, the operator's intent and the configuration, and the FC flies.  The
control law runs only on the FC; this side keeps no copy of it.

Delegation via __getattr__ is deliberate.  Reimplementing the AutopilotLink
protocol would mean reproducing dozens of attributes and methods that have
nothing to do with steering, and every one of them would be a chance to drift
from the real link.  Wrapping means there is nothing to drift.

────────────────────────────────────────────────────────────────────────────
LINK WATCHDOGS - ONE AT EACH END
────────────────────────────────────────────────────────────────────────────
The FC stands down when GUIDE stops arriving: no new seq for GUIDE_STALE_MS
releases control (gate 7).  It also stands down on a Lua error or when an
input the law was flying on disappears, and latches the cause in STATE flags
bits 5-7 (protocol.FAULT_NAMES), which halts this side too.  This side halts the program when STATE stops
arriving: no frame for FC_LINK_TIMEOUT_S, or a frame of the wrong size from
either end, sets `fatal_error`, and the control loop stops on it with a
message on the console.
"""

from __future__ import annotations

import logging
import re
import struct
import threading
import time
from typing import Any, Optional

from .. import protocol as P
from ..constants import FC_LINK_TIMEOUT_S

log = logging.getLogger(__name__)


def _mavutil():
    from pymavlink import mavutil
    return mavutil



# cs_flags bits 0-4, in bit order (protocol.STATE).  Bit 7 travels in
# lua_cs_flags only.
_CS_FLAG_BITS = ("roll_sat", "pan_i_frozen", "pan_i_clamped", "pitch_i_frozen",
                 "pitch_i_clamped")
# The first _CS_FLAG_BANK_BITS of them are bank-to-turn's.
_CS_FLAG_BANK_BITS = 3
# cs_flags bit 5: a PLAN set the bank, under either law; under the cascade
# bank-to-turn did not run.
_CS_FLAG_PLAN_BANK = 0x20
# cs_flags bit 6: Lift Vector Pointing or its version P or N flew this tick
# (steering_law 1, 2 or 3).
_CS_FLAG_PATH_ATTITUDE = 0x40

# The system ID line of ArduPilot's boot banner (send_banner() in
# GCS_Common.cpp sends hal.util->get_system_id()).  SITL sends the computer's
# /etc/machine-id, 32 lowercase hex digits (AP_HAL_SITL/Util.cpp); a ChibiOS
# board sends its short name and its chip's serial number as three groups of
# 8 uppercase hex digits (AP_HAL_ChibiOS/Util.cpp).  It identifies the
# machine, never the firmware or its Lua bindings.
_SYSTEM_ID = re.compile(r'[0-9a-f]{32}|\S+ [0-9A-F]{8} [0-9A-F]{8} [0-9A-F]{8}')

class LuaOffloadLink:
    """Decorates an AutopilotLink; offloads steering to a Lua script on the FC."""

    # ---- construction ------------------------------------------------------

    def __init__(self, inner, port, *,
                 send_hz: float = 50.0):
        """`port` is any object with write(bytes) / read(n) / in_waiting -
        pyserial satisfies it, and so does a SITL loopback socket wrapper."""
        self._inner = inner
        self._port = port
        self._seq = 0
        # True once we have ever transmitted a GUIDE frame.  Distinguishes the
        # normal idle state (control loop not started, so nothing sent - gate 1
        # is expected) from a real link fault (we ARE sending but the FC sees
        # none).
        self._guide_ever_sent = False
        # GUIDE frames sent this session, so a STATE can be told apart from one
        # the FC computed from a previous session's frame (see _apply_state).
        self._guide_sent_count = 0
        self._rx = bytearray()
        self._lock = threading.Lock()
        # Serialises every frame going OUT; see _send.
        self._tx_lock = threading.Lock()
        self._stop = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None

        # FC firmware identity, captured from the boot STATUSTEXT banner so a
        # flight log records the firmware it was flown against, next to the
        # code_sha256 of the companion.  The ArduPlane version line names the
        # firmware; the system ID line after it names the machine that ran it
        # (_SYSTEM_ID), and neither says anything about the Lua bindings.  The
        # last one seen before a log opens is the one stamped, so a reboot
        # mid-config leaves the head naming the build that actually flew.
        self._fc_firmware = ''
        self._fc_system_id = ''

        # --- mirrored FC state.  Named lua_* so nothing collides with the
        # controller's own attributes and the CSV can carry both side by side.
        # STATE (protocol.STATE): lua_<field> for every float field.  None
        # until the first frame, and None wherever the FC sent NaN - a reading
        # that is absent, or a law internal on a tick the law did not run - so
        # the CSV writes it blank, never as a zero.  The flag bytes decode to
        # None on a tick where they mean nothing (see _apply_state).
        self.lua_seq = 0
        for _f in P.STATE:
            if _f.fmt == "f":
                setattr(self, "lua_" + _f.name, None)
        self.lua_cs_flags = None
        for _n in _CS_FLAG_BITS:
            setattr(self, "lua_" + _n, None)
        self.lua_asp_en = None
        self.lua_asp_sat = None
        self.lua_dt_ms = 0
        self.lua_gate = 1
        self.lua_fc_millis = 0
        self.lua_gimbal_sim_age_ms = 0
        for _n in ("hz_quat", "hz_gyro", "hz_vel", "hz_asp"):
            setattr(self, "lua_" + _n, 0)
        # None until the first STATE, so the log writes them blank.
        self.lua_nav_active = None
        self.lua_est_valid = None
        self.lua_excite_running = None
        self.lua_law_ran = False
        self._rtf_last = 0.0
        self._last_gate_logged = -1
        # STATE2 (protocol.STATE2): lua_<field> for every field, plus the three
        # validity bits.  None until the first frame, and None wherever the FC
        # sent NaN - a reading that is absent, or a law internal on a tick the
        # law did not run - so the CSV writes it blank, never as a zero.
        self.lua_state2_seq = None
        for _f in P.STATE2:
            if _f.name not in ("seq", "flags"):
                setattr(self, "lua_" + _f.name, None)
        self.lua_coord_valid = None
        self.lua_accel_valid = None
        self.lua_lat_marker_valid = None
        # One FC tick's gyro and servo signals with that tick's FC clock, as a
        # single tuple (see _apply_state2).  None until the first paired frame.
        self.lua_tick_sample = None
        # The same tick's two decoded frames whole, (STATE, STATE2), for a
        # measurement that reads more of the tick than lua_tick_sample carries
        # (the trim map).  Raw as decoded: an absent reading is NaN.
        self._state_pending = None     # this tick's STATE, until its STATE2
        self.lua_tick_frames = None

        # --- link bookkeeping
        self.gains_synced = False
        self.lua_frames_rx = 0
        self.lua_frames_bad = 0
        self._foreign_reported: set = set()   # (msgid, payload length) already reported
        self.lua_last_rx_mono: Optional[float] = None
        # Why the link can no longer be trusted, in words for the operator, or
        # None.  Set from the receive thread; the control loop halts on it.
        self.fatal_error: Optional[str] = None

        # --- outbound state the controller sets through the properties below
        self._det_norm = (0.0, 0.0)
        self._det_valid = False
        self._tracking = False
        self._mode_code = 0
        self._throttle_pct = 0.0
        self._straight_yaw = 0.0
        self._manual_yaw = 0.0
        self._manual_pitch = 0.0
        self._oracle = False
        self._gimbal = (0.0, 0.0, True)
        self._gimbal_read_mono = time.monotonic()
        self._gimbal_stamp_ms = 0
        self._rtf_ref = None
        self._straight_level = False
        self._oracle_dir = (0.0, 0.0, 0.0)

        self._gain_index = P.gain_index_map(_control_gains_cls())

    # ---- everything not overridden goes to the wrapped link ---------------

    def __getattr__(self, name: str) -> Any:
        # Only reached for attributes this class does NOT define, so the
        # overrides below always win and the other 38 come from the real link.
        return getattr(self._inner, name)

    # ---- lifecycle ---------------------------------------------------------

    def start_telemetry(self) -> None:
        self._inner.start_telemetry()
        # Surface STATUSTEXT.  Every gcs:send_text() the script emits - startup
        # banner, gate report, error containment - arrives here and was being
        # dropped, so the operator saw a silent aircraft and no explanation.
        try:
            self._txt_q = self._inner._subscribe('STATUSTEXT')
            t = threading.Thread(target=self._statustext_loop, daemon=True,
                                 name="lua-statustext")
            t.start()
        except Exception:                       # noqa: BLE001
            self._txt_q = None
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True,
                                           name="lua-state-rx")
        self._rx_thread.start()

    def _statustext_loop(self) -> None:
        while not self._stop.is_set():
            msg = self._inner._wait_msg(self._txt_q, 0.5)
            if msg is None:
                continue
            txt = getattr(msg, 'text', '')
            if isinstance(txt, bytes):
                txt = txt.decode('utf-8', 'replace')
            txt = txt.rstrip('\x00').strip()
            if txt:
                log.info("[FC] %s", txt)
                # Keep the firmware identity for the CSV head: the version
                # banner names ArduPlane, the system ID is a line of its own.
                if 'ArduPlane' in txt:
                    self._fc_firmware = txt
                elif _SYSTEM_ID.fullmatch(txt):
                    self._fc_system_id = txt

    def firmware_banner_lines(self) -> list:
        """FC firmware identity for the CSV head: the ArduPlane version banner
        and the system ID line captured from the boot STATUSTEXT.  Empty
        entries are dropped, so a line the FC never sent is absent rather than
        a blank note."""
        lines = []
        if self._fc_firmware:
            lines.append(self._fc_firmware)
        if self._fc_system_id:
            lines.append('system id ' + self._fc_system_id)
        return lines

    def shutdown(self) -> None:
        self._stop.set()
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
        self._inner.shutdown()

    # ---- the per-tick frame ------------------------------------------------

    def send_guide(self, throttle_frac: float) -> None:
        """Send this tick's GUIDE frame.  `throttle_frac` is the OPERATOR's
        throttle, 0..1; the airspeed-hold loop that may modulate it runs on the
        FC."""
        self._throttle_pct = float(throttle_frac) * 100.0
        self._send_guide()

    def release_rc_overrides(self) -> None:
        # Steering never overrides the sticks, but the companion still does
        # outside it (level-and-hold), so the release is
        # forwarded to the link that sent them.
        self._inner.release_rc_overrides()

    # ---- outbound ----------------------------------------------------------

    def set_gimbal(self, pan_deg: float, tilt_deg: float,
                   readback: bool = True, read_mono: float = None,
                   stamp_ms: int = 0) -> None:
        """The measured encoder angles.

        The FC has no mount driver in this project - the gimbal is a Gazebo
        model driven by this process - so the script cannot read encoders and
        the companion must supply them.
        """
        self._gimbal = (float(pan_deg), float(tilt_deg), bool(readback))
        # When the angles were READ, so the frame can carry their age.  Falls
        # back to "now" if the caller does not know - which reports 0 age and is
        # honest about being a lower bound rather than silently optimistic.
        self._gimbal_read_mono = read_mono if read_mono is not None else time.monotonic()
        self._gimbal_stamp_ms = int(stamp_ms or 0)

    def set_oracle_dir(self, wx: float, wy: float, wz: float) -> None:
        """The exact world-frame bearing used by ptest.

        Not a detection: there is nothing to fold into the estimator, so the FC
        must be told the direction outright or it has no balloon at all.
        """
        self._oracle_dir = (float(wx), float(wy), float(wz))

    def set_detection(self, norm_x: float, norm_y: float, valid: bool) -> None:
        self._det_norm = (float(norm_x), float(norm_y))
        self._det_valid = bool(valid)

    def set_mode(self, mode_code: int, tracking: bool, *,
                 oracle: bool = False, straight_level: bool = False) -> None:
        self._mode_code = int(mode_code)
        self._tracking = bool(tracking)
        self._oracle = bool(oracle)
        self._straight_level = bool(straight_level)

    def set_manual_setpoints(self, straight_yaw_deg: float,
                             manual_yaw_deg: float,
                             manual_pitch_deg: float) -> None:
        self._straight_yaw = float(straight_yaw_deg)
        self._manual_yaw = float(manual_yaw_deg)
        self._manual_pitch = float(manual_pitch_deg)

    def _send_guide(self) -> None:
        self._seq = (self._seq + 1) & 0xFFFF
        self._guide_ever_sent = True
        self._guide_sent_count += 1
        flags = ((0x01 if self._det_valid else 0)
                 | (0x02 if self._tracking else 0)
                 | (0x04 if self._oracle else 0)
                 | (0x08 if self._straight_level else 0))
        frame = P.pack(P.MSG_GUIDE, dict(
            seq=self._seq, flags=flags, mode=self._mode_code,
            det_norm_x=self._det_norm[0], det_norm_y=self._det_norm[1],
            throttle_pct=self._throttle_pct,
            straight_yaw=self._straight_yaw,
            manual_yaw=self._manual_yaw,
            manual_pitch=self._manual_pitch,
            gimbal_pan=self._gimbal[0], gimbal_tilt=self._gimbal[1],
            gimbal_flags=(1 if self._gimbal[2] else 0), _pad2=0,
            oracle_x=self._oracle_dir[0], oracle_y=self._oracle_dir[1],
            oracle_z=self._oracle_dir[2],
            gimbal_stamp_ms=self._gimbal_stamp_ms & 0xFFFFFFFF,
            gimbal_age_ms=max(0, min(65535, int(
                (time.monotonic() - self._gimbal_read_mono) * 1000.0)))))
        self._send(frame)

    def _send(self, frame: bytes) -> bool:
        """Write one whole frame; True when all of it left.  GUIDE goes out
        from the control loop, the full gain push from the tracking start's
        thread, single gains from the tuner and the sweep.  Without the lock,
        two writes that overlap under back-pressure interleave their bytes, both
        frames fail the FC's CRC, and a lost SET_GAIN is never re-sent: the FC
        flies the old gain while the tuner shows the new one.

        A port's write returns how many bytes left (pyserial's blocking write,
        the SITL socket's bounded one); None from a port that does not say
        counts as whole.  A GUIDE that did not leave whole is followed by the
        next tick's; a gain that did not is the caller's to send again."""
        with self._tx_lock:
            n = self._port.write(frame)
        return n is None or n >= len(frame)

    # ---- gains -------------------------------------------------------------

    def push_gain(self, name: str, value: float) -> bool:
        """Push one gain live.  Returns False for an unknown name rather than
        raising: the autotune sweep drives this in a loop and a typo should
        show up as a refused push, not a crashed run."""
        idx = self._gain_index.get(name)
        if idx is None:
            return False
        return self._send(P.pack(P.MSG_SET_GAIN, dict(index=idx, value=float(value))))

    def sync_gains(self, gains) -> int:
        """Push the LIVE gains to the FC.  Call before every flight.

        balloon_chaser_fc.lua seeds itself from constants.lua at load, but the tuner
        edits gains at RUNTIME: anything changed since startup exists only in
        the companion's ControlGains object, and without this push the FC would
        fly the compiled-in defaults.  Pushing the object itself means whatever
        the operator is flying is what the FC gets.
        """
        n = self.push_all_gains(gains)
        self.gains_synced = True
        return n

    def push_all_gains(self, gains) -> int:
        """Push an entire ControlGains, one SET_GAIN frame per numeric field.
        Raises OSError when a frame did not leave whole: the FC confirms
        nothing, so a caller that counted it as sent would leave the FC on the
        old gain for good.  The controller's per-tick push sends the set again."""
        n = short = 0
        for name, idx in sorted(self._gain_index.items(), key=lambda kv: kv[1]):
            v = getattr(gains, name, None)
            if isinstance(v, (int, float)):
                if not self._send(P.pack(P.MSG_SET_GAIN, dict(index=idx, value=float(v)))):
                    short += 1
                n += 1
        if short:
            raise OSError("%d of %d SET_GAIN frames did not leave whole" % (short, n))
        return n

    # ---- excitation --------------------------------------------------------

    def start_excitation(self, kind: int, *, axis: int = P.AXIS_ROLL,
                         replace: bool = False, amp_dps: float = 0.0,
                         f0_hz: float = 0.0, f1_hz: float = 0.0,
                         duration_s: float = 0.0) -> None:
        """Command a waveform.  The FC generates it locally against millis(),
        so the companion's scheduling jitter never enters the excitation - the
        reason identification moved to the FC in the first place.

        `replace` picks the identification mode, on `axis`:
            False -> ADDS to that axis's command   (freqid: loop stays closed)
            True  -> REPLACES it                   (open loop on that axis)
        """
        self._send(P.pack(P.MSG_EXCITE, dict(
            kind=int(kind), axis=int(axis), replace=1 if replace else 0, _pad=0,
            amp_dps=float(amp_dps), f0_hz=float(f0_hz), f1_hz=float(f1_hz),
            dur_ms=int(round(duration_s * 1000.0)))))

    def stop_excitation(self) -> None:
        # dur_ms == 0 cancels; see the protocol contract.
        self._send(P.pack(P.MSG_EXCITE, dict(
            kind=P.EXC_NONE, axis=0, replace=0, _pad=0,
            amp_dps=0.0, f0_hz=0.0, f1_hz=0.0, dur_ms=0)))

    # ---- the Curve Planner's PLAN ------------------------------------------

    def send_plan(self, plan_id: int, law: int, eps0_deg: float, knots_deg,
                  runin_climb_deg: float, runin_dep_deg: float) -> None:
        """Send the Curve Planner's turn for the chase the FC is waiting on,
        made for steering law `law` (its knots in that law's measure of the
        bank), and its run-in (NaN for none).  The FC takes it only while
        waiting, and only for the law it has selected, so the caller re-sends
        it until STATE2's cf_plan_id echoes plan_id."""
        vals = dict(plan_id=int(plan_id), law=int(law), _pad=0, eps0_deg=float(eps0_deg),
                    runin_climb_deg=float(runin_climb_deg),
                    runin_dep_deg=float(runin_dep_deg))
        for k, b in enumerate(knots_deg):
            vals[f"knot_{k + 1}_deg"] = float(b)
        self._send(P.pack(P.MSG_PLAN, vals))

    # ---- inbound -----------------------------------------------------------

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                waiting = getattr(self._port, "in_waiting", 0)
                chunk = self._port.read(waiting or 1)
                if chunk:
                    with self._lock:
                        self._rx.extend(chunk)
                        self._drain()
                else:
                    time.sleep(0.002)
            except Exception:
                # A read error must not crash the thread; the link may recover.
                time.sleep(0.01)

    def _drain(self) -> None:
        buf = self._rx
        i = 0
        n = len(buf)
        while n - i >= P.HEADER_SIZE + P.CRC_SIZE:
            magic = buf[i] | (buf[i + 1] << 8)
            if magic != P.MAGIC:
                i += 1                       # resync a byte at a time
                continue
            plen = buf[i + 3]
            total = P.HEADER_SIZE + plen + P.CRC_SIZE
            if n - i < total:
                break
            try:
                msgid, vals = P.unpack(bytes(buf[i:i + total]))
            except ValueError:               # bad magic or checksum
                self.lua_frames_bad += 1
                i += 1
                continue
            except (struct.error, KeyError):
                # The checksum passed, so the frame is whole: a message id this
                # build does not know, or a payload of another size for its id.
                # The FC is running a different balloon_chaser_fc.lua.  Stepped over
                # like any bad frame: left unconsumed, it would sit at the head
                # of the buffer and hold every later frame behind it.
                self.lua_frames_bad += 1
                self._report_foreign_frame(buf[i + 2], plen)
                i += 1
                continue
            if msgid == P.MSG_STATE:
                self._apply_state(vals)
            elif msgid == P.MSG_STATE2:
                self._apply_state2(vals)
            i += total
        if i:
            del buf[:i]
        # Bound the buffer: a link emitting garbage must not grow it forever.
        if len(buf) > 4096:
            del buf[:-4096]

    def _report_foreign_frame(self, msgid: int, plen: int) -> None:
        """Say once per kind that the FC speaks another build's protocol."""
        if (msgid, plen) in self._foreign_reported:
            return
        self._foreign_reported.add((msgid, plen))
        known = P.MESSAGES.get(msgid)
        want = ("%d bytes" % P.payload_size(known[1])) if known else "no such message"
        log.error("[lua] the FC sent message %#04x with a %d-byte payload; this "
                  "build expects %s.  The FC is running a different "
                  "balloon_chaser_fc.lua: restart its scripting engine.",
                  msgid, plen, want)
        if self.fatal_error is None:
            self.fatal_error = (
                "The flight controller sent a message this program cannot read "
                "(message %#04x, %d bytes; this build expects %s).  The FC is "
                "running a different version of balloon_chaser_fc.lua than this "
                "program.  Deploy the scripts from this build and restart the "
                "FC's scripting engine." % (msgid, plen, want))

    def _apply_state(self, v: dict) -> None:
        """Decode STATE: every float field to lua_<name>, NaN to None; the
        integer fields as they stand; the flag bytes to their bits."""
        self.lua_seq = v["seq"]
        self._state_pending = v
        f = v["flags"]
        self.lua_nav_active = bool(f & 0x01)
        self.lua_est_valid = bool(f & 0x02)
        self.lua_excite_running = bool(f & 0x04)
        self.lua_law_ran = bool(f & 0x08)
        # The FC's latched flags (bit 4, bits 5-7) count only on a STATE it
        # computed from one of THIS session's GUIDE frames - its seq echo is
        # one we sent.  The script outlives the program: a flag latched when
        # the previous session ended is still set in the first frames a new
        # session reads, and the FC clears it only once this session's first
        # GUIDE arrives.
        mine = ((self._seq - v["seq"]) & 0xFFFF) < self._guide_sent_count
        if not mine:
            f &= 0x0F
        if (f & 0x10) and self.fatal_error is None:
            self.fatal_error = (
                "The flight controller rejected a message from this program: "
                "its balloon_chaser_fc.lua expects a different message layout "
                "(see the FC console for the message id and size).  The FC is "
                "running a different version of the scripts than this program.  "
                "Deploy the scripts from this build and restart the FC's "
                "scripting engine.")
        fault = (f >> 5) & 0x07
        if fault and self.fatal_error is None:
            self.fatal_error = (
                "The flight controller script stopped flying the aircraft and "
                "handed control back: %s (fault %d).  The FC console carries the "
                "line 'balloon chaser: FAULT, ...' with the detail; a script "
                "error names its file and line."
                % (P.FAULT_NAMES.get(fault, "an unknown fault"), fault))
        for _f in P.STATE:
            if _f.fmt == "f":
                x = v[_f.name]
                setattr(self, "lua_" + _f.name, None if x != x else x)
        self.lua_dt_ms = int(v["dt_ms"])
        self.lua_gate = int(v["gate"])
        self.lua_fc_millis = int(v["fc_millis"])
        self.lua_gimbal_sim_age_ms = int(v["gimbal_sim_age_ms"])
        for _n in ("hz_quat", "hz_gyro", "hz_vel", "hz_asp"):
            setattr(self, "lua_" + _n, int(v[_n]))
        # The flag bytes are integers and cannot carry NaN.  cs_flags means
        # something only on a tick the law ran (flags bit 3), asp_flags only on
        # a commanded tick (gate 0); on any other tick every bit is None -
        # written blank - never a confident 0.  cs_flags bit 6 says Lift
        # Vector Pointing flew, not the cascade, so the cascade's bits mean
        # nothing; bit 5 that a PLAN set the bank, so under the cascade
        # bank-to-turn's mean nothing.
        f2 = v["cs_flags"]
        ran = self.lua_law_ran
        self.lua_cs_flags = f2 if ran else None
        cascade_ran = ran and not (f2 & _CS_FLAG_PATH_ATTITUDE)
        bank_ran = cascade_ran and not (f2 & _CS_FLAG_PLAN_BANK)
        for bit, name in enumerate(_CS_FLAG_BITS):
            meant = bank_ran if bit < _CS_FLAG_BANK_BITS else cascade_ran
            setattr(self, "lua_" + name, bool(f2 & (1 << bit)) if meant else None)
        fa = v["asp_flags"]
        commanded = self.lua_gate == 0
        self.lua_asp_en = bool(fa & 0x01) if commanded else None
        self.lua_asp_sat = bool(fa & 0x02) if commanded else None
        # Report the gate whenever it CHANGES.  The script's own gcs:send_text
        # goes to STATUSTEXT, which nothing on this side was surfacing - so
        # every diagnostic it emitted was invisible in the console the operator
        # actually watches.  This puts the reason in the same log as everything
        # else, and only on transitions so it cannot spam a 50 Hz loop.
        if self.lua_gate != self._last_gate_logged:
            self._last_gate_logged = self.lua_gate
            name = P.GATE_NAMES.get(self.lua_gate, f"unknown ({self.lua_gate})")
            if self.lua_gate == 0:
                log.info("[lua] FC is commanding")
            elif self.lua_gate == 1 and not self._guide_ever_sent:
                # Idle, not a fault: the control loop has not started, so the
                # companion has not sent a GUIDE frame - gate 1 is exactly what
                # to expect.  It becomes a real link fault only once we ARE
                # sending (below).
                log.info("[lua] FC idle: no steering sent yet "
                         "(normal until the control loop starts)")
            elif self.lua_gate == 1:
                log.warning("[lua] FC is NOT commanding: no GUIDE frames received "
                            "- the companion is sending them but they are not "
                            "reaching the FC (check the serial link / baud)")
            elif self.lua_gate == 5:
                # GUIDE flows every tick, so "not tracking" is the ordinary
                # state between modes, not a fault.
                log.info("[lua] FC idle: not tracking")
            else:
                log.warning("[lua] FC is NOT commanding: %s", name)
        self.lua_frames_rx += 1
        self.lua_last_rx_mono = time.monotonic()

    def _apply_state2(self, v: dict) -> None:
        """Decode STATE2: every field to lua_<name>, NaN to None."""
        self.lua_state2_seq = v["seq"]
        f = v["flags"]
        self.lua_coord_valid = bool(f & 0x01)
        self.lua_accel_valid = bool(f & 0x02)
        self.lua_lat_marker_valid = bool(f & 0x04)
        for _f in P.STATE2:
            if _f.name in ("seq", "flags"):
                continue
            x = v[_f.name]
            setattr(self, "lua_" + _f.name, None if x != x else x)
        # The FC's gyro and servo signals with the FC clock of the tick they
        # were read on, stored as ONE tuple so a reader on another thread
        # (plant-ID) never pairs values from two ticks.  STATE, which carries
        # fc_millis, is sent right before STATE2 on the same tick, so the pair
        # is the STATE that arrived IMMEDIATELY before this frame, consumed
        # here.  Equal seqs alone do not pair them: seq echoes the GUIDE frame,
        # which several FC ticks can share, so a lost frame would pair one
        # tick's STATE with the next tick's STATE2.
        state, self._state_pending = self._state_pending, None
        if state is None or state["seq"] != v["seq"]:
            return
        if (self.lua_gyro_x_rads is not None and self.lua_gyro_y_rads is not None
                and self.lua_gyro_z_rads is not None):
            self.lua_tick_sample = (
                int(state["fc_millis"]), self.lua_gyro_x_rads,
                self.lua_gyro_y_rads, self.lua_gyro_z_rads, self.lua_pwm_aileron,
                self.lua_pwm_vtail_l, self.lua_pwm_vtail_r)
        self.lua_tick_frames = (state, v)

    def wait_for_fc(self, timeout: float = 8.0) -> bool:
        """Block until the FC script reports in, or give up and say why.

        Silence is the one outcome that explains nothing.  Without this the
        aircraft arms, flies, and does not track - and every possible cause
        looks identical from the console.
        """
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.lua_frames_rx:
                gate = P.GATE_NAMES.get(self.lua_gate, "?")
                log.info("[lua] FC script is alive (%d STATE frames, gate: %s)",
                         self.lua_frames_rx, gate)
                return True
            time.sleep(0.2)
        log.error("[lua] NO STATE frames from the flight controller after %.0fs. "
                  "The script is not running, or the steering link is not "
                  "connected. Check, in order:", timeout)
        log.error("  1. Did the console show '[FC] balloon chaser: v%d, ... Hz'? "
                  "If not, ArduPilot never loaded balloon_chaser_fc.lua.", P.PROTO_VERSION)
        log.error("  2. balloon_chaser_fc.lua must be in scripts/ and its modules "
                  "in scripts/modules/ - require() looks ONLY in modules/.")
        log.error("  3. SCR_ENABLE=1 and a reboot must have happened.")
        log.error("  4. SERIALn_PROTOCOL=28 and --lua-path must name the same "
                  "endpoint.")
        log.error("")
        log.error("  NOTE: ArduPilot reports a Lua LOAD failure as a CRITICAL "
                  "STATUSTEXT, which would appear above as '[FC] Error: ...'. "
                  "Complete silence means the scripts directory was never "
                  "SCANNED - scripting was off at boot, SCR_DIR_DISABLE bit 1 "
                  "was set, or the FC has not booted since the files landed.")
        return False

    def drop_telemetry_the_fc_does_not_need(self) -> None:
        """Deliberately does nothing.  Kept so the reason is written down.

        There is no companion-side telemetry left to drop:

          SIMSTATE / AHRS / AHRS2 share MAV_DATA_STREAM_EXTRA1 with ATTITUDE,
            and mavlink_link's own docstring records that ATTITUDE IGNORES
            SET_MESSAGE_INTERVAL - the stream-group rate is the only knob.
            Dropping SIMSTATE means dropping ATTITUDE, which the HUD and the
            estimator both need.  It is, in that docstring's words, "a small
            unavoidable bandwidth cost".

          SERVO_OUTPUT_RAW shares MAV_DATA_STREAM_RC_CHANNELS and feeds the
            HUD's only remaining view of the surfaces.

          PID_TUNING is the FC's OWN desired-vs-achieved rate.  With the whole
            cascade on the FC it is the ONLY window into whether the inner loop
            tracks what the steering asked for - the last thing to drop, not
            the first.
        """
        log.info("[lua] no telemetry dropped: SIMSTATE/AHRS2 share EXTRA1 with "
                 "ATTITUDE (group rate is the only knob), SERVO_OUTPUT_RAW "
                 "feeds the surface readout, PID_TUNING is the FC inner-loop "
                 "view")


    @property
    def gimbal_age_ms(self) -> int:
        """Age of the gimbal angles most recently sent, milliseconds."""
        return max(0, int((time.monotonic() - self._gimbal_read_mono) * 1000.0))

    @property
    def rtf(self) -> float:
        """Real-time factor: simulated seconds per wall second.

        ONE interval, measured on TWO clocks - d(FC millis) / d(monotonic).

        The obvious-looking alternative, dividing lua_dt_ms by the companion
        tick, measures nothing: both are ~20 ms BY CONSTRUCTION (one is the FC
        scheduler's period, the other the companion's sleep target), so the
        ratio is ~1 regardless of what the simulator is doing, and its
        stability is an artefact rather than evidence, never a real-time factor.

        Returns 0.0 until two STATE frames have arrived.
        """
        now_fc, now_wall = self.lua_fc_millis, time.monotonic()
        ref = self._rtf_ref
        if ref is None or now_fc == 0:
            if now_fc:
                self._rtf_ref = (now_fc, now_wall)
            return 0.0
        d_fc = (now_fc - ref[0]) & 0xFFFFFFFF
        d_wall = now_wall - ref[1]
        if d_wall < 0.5:                 # need a window worth measuring over
            return getattr(self, "_rtf_last", 0.0)
        self._rtf_ref = (now_fc, now_wall)
        self._rtf_last = (d_fc / 1000.0) / d_wall
        return self._rtf_last

    @property
    def guide_seq(self) -> int:
        """Sequence number of the last GUIDE frame sent."""
        return self._seq

    @property
    def lua_seq_lag(self) -> int:
        """GUIDE frames sent since the one the FC last answered.

        The round-trip lag in ticks, measured directly: STATE echoes the seq of
        the GUIDE frame it was computed from, so this is transport out + FC
        compute + transport back, with no clock synchronisation needed.
        """
        return (self._seq - self.lua_seq) & 0xFFFF

    @property
    def lua_link_age_s(self) -> Optional[float]:
        if self.lua_last_rx_mono is None:
            return None
        return time.monotonic() - self.lua_last_rx_mono

    def link_fault(self) -> Optional[str]:
        """Why the FC link can no longer be trusted, for the operator to read,
        or None while it is healthy.  The control loop asks every tick and
        halts the program on an answer.

        The FC sends STATE every tick, gated or not, so a silence longer than
        FC_LINK_TIMEOUT_S means it has stopped reporting.  Every lua_* value
        would then keep its last reading, and the HUD, the flight log and the
        'p' verdict would all present it as live."""
        if self.fatal_error is not None:
            return self.fatal_error
        age = self.lua_link_age_s
        if age is not None and age > FC_LINK_TIMEOUT_S:
            return ("No report from the flight controller for %.1f s "
                    "(FC_LINK_TIMEOUT_S = %g s).  The FC script has stopped, the "
                    "scripting engine restarted, or the link to the FC broke "
                    "(serial cable, TCP connection, or SITL paused or closed).  "
                    "A Lua error stops the script: the FC console shows its "
                    "file and line.  "
                    "Every FC value on the HUD and in the flight log would "
                    "otherwise be its last reading, shown as if live."
                    % (age, FC_LINK_TIMEOUT_S))
        return None


def _control_gains_cls():
    from ..control_laws import ControlGains
    return ControlGains
