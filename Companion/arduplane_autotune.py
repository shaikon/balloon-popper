# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""Arduplane Autotune - ArduPlane AUTOTUNE driver (roll + pitch), driven through MavlinkAutopilotLink.

Reuses the link's proven plumbing - catapult_launch (FBWA climb; avoids the
TAKEOFF-mode "Bad launch" rejection on the catapult-positioned Zephyr), mode
switching, parameter IO and RC override - so there is no duplicated launch logic
and no separate MAVLink connection.

WHAT IT TUNES:
AUTOTUNE tunes ArduPlane's RATE-controller PIDs (RLL_RATE_*, PTCH_RATE_*). That
rate loop is exactly what executes the body-rate commands the balloon chaser
sends in ACRO, so a better rate tune makes those commanded rates be achieved
more accurately WITHOUT changing any steering/vision logic.  AUTOTUNE is an
ArduPlane *flight mode* that flies FBWA-style (stick = ATTITUDE-ANGLE demand) and
learns from sharp attitude changes; this driver plays "pilot" via RC overrides.

ENTRY POINT: run_arduplane_autotune(link, goal_alt).  Invoked from main.py --arduplane-autotune.

UNTESTED end-to-end by the author (no SITL in the build env).  It launches and
commands aggressive maneuvers - watch the first run with the MAVProxy map up and
be ready to take over (mode RTL) / Ctrl-C.
"""
import logging
import os
import time
from datetime import datetime

from pymavlink import mavutil

log = logging.getLogger("balloon_chaser")

# ---- ArduPlane flight-mode numbers ----
MODE_LOITER = 12
MODE_AUTOTUNE = 8

# ===================== CONFIG (pre-set from your uploaded params) =====================
# AUTOTUNE_LEVEL and AUTOTUNE_AXES are READ, not written.
#
# Both were being set to ArduPlane's own shipped defaults - LEVEL 6 and AXES 7 -
# so the writes changed nothing on a stock FC and, on any other, silently threw
# away whatever the operator had chosen.  A gentler LEVEL 4 typed into a GCS
# would be stamped back to 6 on the next run with no indication.
#
# AXES now DRIVES the run instead of being overwritten by it: the script flies
# doublets on exactly the axes its bitmask selects.  That is the operator's
# control, it is one source of truth, and it removes the failure where the
# bitmask and the doublet loop disagree about which axes are being tuned.
AXIS_BIT = {"roll": 1, "pitch": 2, "yaw": 4}

# Doublets: in AUTOTUNE, full stick = full bank/pitch ANGLE demand. A sharp PWM STEP
# gives a high demanded RATE even at moderate amplitude (the tuner only learns above
# ~40-80% of its max tuning-target rate). Your RC channels are 1100..1900 (centre 1500).
#   ROLL : strong (+-300 -> ~+-34 deg bank; ROLL_LIMIT_DEG=45).
#   PITCH: DELIBERATELY GENTLE. Your PTCH_LIM is +85/-90 deg (set wide for balloon chaser
#          dives), so full pitch stick would command near-vertical zoom/dive. +-100
#          PWM -> only ~+-21 deg. Do NOT enlarge pitch without first narrowing PTCH_LIM.
PWM_MID = 1500
# Doublet amplitude per axis.  A DICT, not a chain of ifs: an axis missing from
# here is a KeyError at the top of the run, whereas an if/else silently flew a
# different axis's amplitudes and looked like it worked.
DOUBLET_PWM = {
    "roll":  (1800, 1200),
    "pitch": (1600, 1400),
    # Yaw needs its own excitation or ArduPlane never tunes it.  AUTOTUNE_AXES
    # only says which axes are ELIGIBLE - plane refines an axis when the PILOT
    # demands on it, so an axis with no stick input is simply never visited, and
    # the run ends reporting success on the axes it did reach.
    #
    # Full stick is deliberate despite the lightly damped yaw axis (Dutch roll
    # zeta ~0.26): the demand is already capped at ACRO_YAW_RATE, so full
    # deflection asks for 45 deg/s, not an unbounded rate.
    "yaw":   (1800, 1200),
}
THROTTLE_PWM = 1700         # held during doublets (AUTOTUNE = FBWA-style MANUAL throttle)
HOLD_S = 0.7                # dwell at each extreme (the "slight pause" the tuner wants)
AXIS_SECONDS = 90           # doublets per axis (roll, pitch, yaw); extend if not done
#
# ALTITUDE BUDGET.  Full-deflection doublets with no pitch demand bleed height
# fast - in FBWA a hard bank with zero pitch demand simply descends.  Measured
# against ALT_FLOOR_M below, roll doublets alone have consumed ~460 m in 67 s,
# about 7 m/s.  THREE axes at AXIS_SECONDS each is 270 s, so the launch altitude
# has to clear the floor by roughly 2000 m for a full run to finish.
#
# It does not fail loudly.  An axis that runs out of height reports "abort", the
# axes after it report "not run", and the summary still prints VERDICT: TUNED
# because the axes that DID run changed their gains.  Read the phases line, not
# the verdict.

ALT_FLOOR_M = 80.0          # abort an axis if rel-alt drops below this
SETTLE_AFTER_LAUNCH_S = 6.0 # stabilise in LOITER after the climb before exciting
# =====================================================================================

_RATE_PIDS = ["RLL_RATE_P", "RLL_RATE_I", "RLL_RATE_D", "RLL_RATE_FF",
              "PTCH_RATE_P", "PTCH_RATE_I", "PTCH_RATE_D", "PTCH_RATE_FF",
              # Yaw. These exist ONLY while YAW_RATE_ENABLE is non-zero -
              # AP_PARAM_FLAG_ENABLE hides the whole YAW_RATE_ subtree at 0 - so
              # _fetch_param returns None for them on an FC that has never had
              # the rudder enabled. Both readers already handle that: the backup
              # skips a None, and the before/after diff prints it. Without these
              # a yaw tune could not be SEEN in the summary, and the backup
              # could not restore the gains it changed.
              "YAW_RATE_P", "YAW_RATE_I", "YAW_RATE_D", "YAW_RATE_FF"]
_BACKUP_PARAMS = _RATE_PIDS + ["AUTOTUNE_LEVEL", "AUTOTUNE_AXES", "YAW_RATE_ENABLE"]
_COMPLETION_SUBSTRINGS = ["complete", "success", "saved", "finished", "done"]


def _set_param(link, pq, name, value, ptype=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
    with link._mav_lock:
        link.master.mav.param_set_send(
            link.master.target_system, link.master.target_component,
            name.encode("utf-8"), float(value), ptype)
    got = link._fetch_param(pq, name)
    ok = got is not None and abs(got - float(value)) <= max(1e-6, abs(value) * 1e-3)
    log.info("  set %-16s = %-8s (readback %s%s)", name, value, got,
             "" if ok else "  <-- MISMATCH")
    return ok


def _rc(link, roll=PWM_MID, pitch=PWM_MID, throttle=THROTTLE_PWM, yaw=PWM_MID):
    # channel order matches catapult_launch: ch1 roll, ch2 pitch, ch3 throttle, ch4 yaw
    with link._mav_lock:
        link.master.mav.rc_channels_override_send(
            link.master.target_system, link.master.target_component,
            int(roll), int(pitch), int(throttle), int(yaw), 0, 0, 0, 0)


def _read_pids(link, pq):
    return {n: link._fetch_param(pq, n) for n in _RATE_PIDS}


def _print_pids(tag, pids):
    log.info("----- rate PIDs %s -----", tag)
    for k, v in pids.items():
        log.info("    %-14s %s", k, v)


def _param_changed(b, a):
    """True if a rate PID moved meaningfully (2% relative, or any change off 0)."""
    if b is None or a is None:
        return False
    return abs(a - b) > max(1e-4, 0.02 * abs(b))


def _report_result(before, after):
    """Decide success from the BEFORE->AFTER rate-PID change - the only reliable
    signal for plane autotune (it does not emit per-axis 'complete' STATUSTEXT).
    Returns True if the gains were tuned."""
    log.info("----- rate PID change (before -> after) -----")
    n_changed = 0
    for n in _RATE_PIDS:
        b, a = before.get(n), after.get(n)
        changed = _param_changed(b, a)
        n_changed += 1 if changed else 0
        log.info("    %-14s %-22s -> %-22s%s", n, b, a, "  <-- changed" if changed else "")
    d_roll, d_ptch = after.get("RLL_RATE_D"), after.get("PTCH_RATE_D")
    # THIS RUN's changes, and nothing else.  D is PERSISTENT: it is non-zero
    # because some EARLIER run set it, so a non-zero D is NOT evidence that THIS
    # flight tuned anything - counting it would report "TUNED" on a flight that
    # changed nothing.  A verdict must describe what this flight did, not the
    # state it found.
    tuned = n_changed > 0
    if tuned:
        log.info("VERDICT: TUNED - %d rate params changed; D now roll=%s pitch=%s. "
                 "Gains were updated and persist in the FC (read back AFTER leaving "
                 "AUTOTUNE).", n_changed, d_roll, d_ptch)
    else:
        log.warning("VERDICT: NOT TUNED - every rate PID is byte-identical to "
                    "before the flight, so ArduPlane never updated a gain.")
        log.warning("  This is NOT the same as 'the tune converged'. ArduPlane "
                    "rewrites a gain whenever it learns anything, so zero changes "
                    "means it learned nothing at all.")
        log.warning("  Likely causes, in order:")
        log.warning("    1. TOO LITTLE DYNAMIC PRESSURE. The tuner learns from the "
                    "rate the aircraft ACHIEVES against the rate demanded. High and "
                    "slow, full stick buys little rate and it sees nothing to fit. "
                    "Tune in dense air with margin for the descent, not at ceiling.")
        log.warning("    2. Doublet amplitude too small - raise DOUBLET_PWM for the "
                    "axis that did not move.")
        log.warning("    3. RC override not reaching the FC - check RC_OPTIONS bit0 "
                    "and RC_OVERRIDE_TIME.")
    return tuned


def _backup_params(link, pq, path):
    lines = ["# autotune param backup %s" % datetime.now().isoformat(timespec="seconds")]
    n_ok = 0
    for name in _BACKUP_PARAMS:
        v = link._fetch_param(pq, name)
        if v is not None:
            lines.append("%s\t%s" % (name, v))
            n_ok += 1
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    log.info("  param backup written: %s (%d params)", path, n_ok)


def _drain_statustext(sq):
    """Return True if a completion phrase was seen; print any autotune messages."""
    import queue as _q
    done = False
    while True:
        try:
            msg = sq.get_nowait()
        except _q.Empty:
            break
        if getattr(msg, "get_type", lambda: "")() != "STATUSTEXT":
            continue
        text = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
        low = text.lower()
        if "tune" in low:
            log.info("  [AUTOTUNE] %s", text)
            if any(s in low for s in _COMPLETION_SUBSTRINGS):
                done = True
    return done


def _fly_doublets(link, axis, seconds, sq):
    hi, lo = DOUBLET_PWM[axis]
    log.info("AUTOTUNE %s doublets ~%ds (HIGH=%d LOW=%d hold=%.1fs)",
             axis, seconds, hi, lo, HOLD_S)
    t_end = time.time() + seconds
    sign = 1
    while time.time() < t_end:
        _rc(link, **{axis: (hi if sign > 0 else lo)})   # drive axis; others neutral
        t_hold = time.time() + HOLD_S
        while time.time() < t_hold:
            if _drain_statustext(sq):
                return "done"
            a = link.relative_alt
            if a is not None and a < ALT_FLOOR_M:
                log.warning("  rel-alt %.1f < floor %.1f -> abort axis", a, ALT_FLOOR_M)
                return "abort"
            time.sleep(0.02)
        sign = -sign
    # Ran the full configured duration. For PLANE autotune this is the NORMAL end
    # of a doublet phase - plane refines continuously and has no discrete per-axis
    # "complete" event, so this is not a failure (unlike copter).
    return "ran_full_duration"


def run_arduplane_autotune(link, goal_alt=150.0):
    """Bring the link up, launch, climb, run AUTOTUNE on roll then pitch, persist
    the gains, and report before/after.  Reuses the link's catapult_launch."""
    log.info("=== ArduPlane AUTOTUNE (roll+pitch) - main.py --arduplane-autotune ===")

    # Link bring-up - same order the controller uses before catapult_launch.
    link.start_telemetry()
    time.sleep(1.0)
    link.request_high_rate_streams()
    # Manifest gate FIRST. AUTOTUNE against a mis-wired airframe measures
    # nothing and wastes a flight.
    link.verify_param_manifest()
    link.read_fc_config()

    pq = link._subscribe("PARAM_VALUE")
    sq = link._subscribe("STATUSTEXT")
    backup = os.path.join(os.getcwd(),
                          "params_backup_%s.parm" % datetime.now().strftime("%Y%m%d_%H%M%S"))
    results = {"roll": "not run", "pitch": "not run"}
    before = {n: None for n in _RATE_PIDS}
    try:
        _backup_params(link, pq, backup)
        before = _read_pids(link, pq)
        _print_pids("BEFORE", before)

        # READ the tune configuration; nothing here writes it.
        log.info("[setup] reading AUTOTUNE configuration ...")
        _lvl = link._fetch_param(pq, "AUTOTUNE_LEVEL")
        _axes = link._fetch_param(pq, "AUTOTUNE_AXES")
        _yaw_en = link._fetch_param(pq, "YAW_RATE_ENABLE")
        log.info("  AUTOTUNE_LEVEL   = %s  (operator's; higher = more aggressive)", _lvl)
        log.info("  AUTOTUNE_AXES    = %s  (bit0 roll, bit1 pitch, bit2 yaw)", _axes)
        _mask = int(_axes) if _axes is not None else 0
        axes = [a for a in ("roll", "pitch", "yaw") if _mask & AXIS_BIT[a]]
        if not axes:
            log.error("AUTOTUNE_AXES = %s selects no axis - nothing to tune. "
                      "Set bit0/1/2 for roll/pitch/yaw.", _axes)
            return
        # The yaw axis needs the rate controller RUNNING, not merely selected.
        # At YAW_RATE_ENABLE = 0 the rudder branch in Attitude.cpp is skipped
        # entirely, so rudder doublets excite nothing and the axis would burn
        # its full duration - and its share of the altitude budget - learning
        # nothing. Drop it loudly rather than flying a tune that cannot work.
        if "yaw" in axes and not (_yaw_en and float(_yaw_en) != 0.0):
            log.warning("  YAW_RATE_ENABLE = %s - the yaw rate controller is OFF, "
                        "so the yaw axis cannot tune. Skipping it. Set "
                        "YAW_RATE_ENABLE = 1 to include yaw.", _yaw_en)
            axes = [a for a in axes if a != "yaw"]
        log.info("  axes to fly       = %s", ", ".join(axes))

        # Get airborne with the PROVEN launch (FBWA climb to ~0.9*alt).
        # final_mode=None: this path uploads NO mission, and ArduPlane refuses
        # AUTO without one. The next line goes to LOITER anyway, so AUTO is only
        # a transient here.
        link.wait_until_prearm_ok()
        link.catapult_launch(goal_alt, final_mode=None)

        # Stabilise in LOITER (auto-throttle, holds station) before exciting.
        link._set_mode(MODE_LOITER, "LOITER")
        log.info("  stabilising %.0fs in LOITER before AUTOTUNE ...", SETTLE_AFTER_LAUNCH_S)
        time.sleep(SETTLE_AFTER_LAUNCH_S)

        # Enter AUTOTUNE and immediately start holding throttle (manual-throttle mode).
        link._set_mode(MODE_AUTOTUNE, "AUTOTUNE")
        t_hold = time.time() + 2.0
        while time.time() < t_hold:
            _rc(link)                 # neutral sticks, throttle held
            time.sleep(0.1)

        # Yaw LAST (the list is built in this order): it is the axis whose PID
        # has never been tuned, and by then roll and pitch are refined, so a
        # yaw excursion is flown by a better-behaved aircraft.
        for axis in axes:
            results[axis] = _fly_doublets(link, axis, AXIS_SECONDS, sq)
            _rc(link)                 # neutralise between axes
            time.sleep(1.0)
            if results[axis] == "abort":
                break
    finally:
        # ALWAYS release RC and leave AUTOTUNE (leaving AUTOTUNE is what saves FF/I).
        log.info("[cleanup] releasing RC override, switching to LOITER ...")
        try:
            link.release_rc_overrides()
        except Exception:
            pass
        try:
            link._set_mode(MODE_LOITER, "LOITER")
        except Exception:
            pass

    after = _read_pids(link, pq)
    _print_pids("AFTER", after)
    link._unsubscribe(pq, "PARAM_VALUE")
    link._unsubscribe(sq, "STATUSTEXT")

    # Success is judged from the BEFORE->AFTER rate-PID change (read back AFTER
    # leaving AUTOTUNE), not from STATUSTEXT - plane autotune emits no per-axis
    # "complete" message, refines continuously, and saves on mode exit.
    tuned = _report_result(before, after)
    log.info("doublet phases: %s", results)
    if any(v == "abort" for v in results.values()):
        log.info("NOTE: an axis was aborted early (altitude floor / Ctrl-C). The tune "
                 "is valid but less refined on that axis; re-run to refine further if "
                 "the flight test shows it needs it.")
    log.info("Original values backed up at %s  (restore in MAVProxy: param load <file>)",
             backup)
    if tuned:
        log.info("NEXT: fly a normal run (no --arduplane-autotune) and check for clean attitude "
                 "tracking without oscillation. If it oscillates, lower AUTOTUNE_LEVEL "
                 "and re-run, or restore the backup.")
    return tuned
