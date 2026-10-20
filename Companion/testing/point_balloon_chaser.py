# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""
Point-balloon-chaser specification test (the 'p' key).

Free function run_point_balloon_chaser_test(controller, abort): reads and writes
controller state through public accessors and drives the SyntheticBalloonInjector
through the oracle interface.

Spec:
    1. Combined steady-state error hypot(pan_err, tilt_err) < ALIGN_TOL_DEG.
       This is the ANGULAR MISS ANGLE - the single angle between the FLIGHT
       PATH and the balloon, from ctrl.alignment_errors() - not two independent
       per-axis budgets.  It is the same criterion and tolerance behind the
       HUD's "Path Aligned" indicator and timer, the align-seconds counter, the
       log's in_tol column and the autotune sweep's align gate, so a PASS here
       means the same thing the operator saw on screen.  Note it is STRICTER
       than "each axis under the tolerance": pan=0.4 with tilt=0.4 leaves both
       axes inside 0.5 but the path 0.57 deg off the balloon, which is a miss.
       Treating the axes separately would also make the threshold depend on how
       the error happened to split between them.
    2. Held for 0.5 s.  The competition asks for best effort, a missed balloon
       being the worst result, and sets no time limit: the test reports how
       long the path took to line up, never a deadline verdict.
"""
from __future__ import annotations
import logging
import math
import time

import numpy as np

from ..constants import ALIGN_TOL_DEG
from .synthetic_injector import SyntheticBalloonInjector

log = logging.getLogger("balloon_chaser")


def run_point_balloon_chaser_test(ctrl, abort):
    """Run the point-balloon-chaser spec test.  ctrl is the BalloonChaser; abort is
    a threading.Event.  Designed to run on a daemon thread.

    Uses these PUBLIC controller hooks:
        ctrl.link                         AutopilotLink
        ctrl.sensor                       BalloonSensor (oracle interface)
        ctrl.ptest_enter_acro()           -> bool   (set ACRO, seed commands)
        ctrl.ptest_begin(lon, lat)        arm tracking after injection
        ctrl.ptest_finish()               restore to loiter if still ptest
        ctrl.alignment_errors()           -> (pan_err_deg, tilt_err_deg) or None
        ctrl.point_lon_deg / point_lat_deg   (balloon POSITION bearing)
        ctrl.mode                         current mode string
        ctrl.set_point_injector(inj)      stash handle for abort cleanup
    """
    SAMPLE_DT = 0.02   # 50 Hz
    # Tolerance on the COMBINED miss angle hypot(pan_err, tilt_err), not on each
    # axis separately - see the Spec block in the module docstring.
    SS_ERROR_DEG = ALIGN_TOL_DEG
    HOLD_S = 0.5
    WARMUP_S = 0.5     # longest wait for the FC's first commanded tick

    def _end(reason):
        log.info("[PTEST] %s", reason)
        ctrl.ptest_finish()

    lon_deg = ctrl.point_lon_deg
    lat_deg = ctrl.point_lat_deg
    init_range = float(getattr(ctrl, 'point_balloon_range', float('inf')))

    # Anchor the balloon BEFORE switching to ACRO.  Anchoring needs only the current
    # heading (plus GPS for a finite distance), and it sets sensor.oracle_active
    # immediately, so the HUD marker is correct from the first frame instead of
    # sitting on the boresight for the ~0.7 s the blocking ACRO switch takes.  It
    # also captures the heading the operator actually intended.  tracking is still
    # off, so the GUIDE frames the run loop sends carry tracking = 0 and the FC
    # does not fly them yet.
    # Master ON/OFF for balloon motion, and only motion: point_balloon_range alone
    # decides WHERE the balloon is anchored, by the min(range, ground) rule.
    # move_enable_fn tracks the switch LIVE so the operator can pause and resume
    # motion mid-test from the Gain Tuner.  speed_fn tracks POINT_BALLOON_SPEED the
    # same way - pausing is itself a speed change to zero, so the speed cell should
    # not be the one control that freezes once a run starts, and a run pressed with
    # speed 0 can have a speed dialled in rather than reporting "motion RESUMED"
    # while sitting still.  The DIRECTION is polled the same way: polling it live
    # (not freezing it at start()) is what lets the bearing slider move the
    # BALLOON mid-run rather than the tuner value alone.  All three go through
    # the same leg boundary, so the balloon keeps its position across any of them.
    move_enable_fn = lambda: bool(getattr(ctrl, 'point_balloon_move_enable', 0))
    speed_fn = lambda: float(getattr(ctrl, 'point_balloon_speed', 0.0))
    vel_dir_fn = lambda: (float(getattr(ctrl, 'point_balloon_vel_pan', 0.0)),
                          float(getattr(ctrl, 'point_balloon_vel_tilt', 0.0)))
    inj = SyntheticBalloonInjector(
        ctrl.link, ctrl.sensor, lon_deg, lat_deg,
        speed=getattr(ctrl, 'point_balloon_speed', 0.0),
        init_range=init_range,
        vel_pan_deg=getattr(ctrl, 'point_balloon_vel_pan', 0.0),
        vel_tilt_deg=getattr(ctrl, 'point_balloon_vel_tilt', 0.0),
        move_enable_fn=move_enable_fn,
        speed_fn=speed_fn,
        vel_dir_fn=vel_dir_fn)
    ctrl.set_point_injector(inj)
    if not inj.start():
        return _end("injector precondition failed")
    # Announce only AFTER a successful start, logging the anchored bearing read
    # back from the injector (inj.lon/inj.lat).  Every (lon, lat) is valid now,
    # so this equals the request modulo a (-180, +180] wrap - but reading it back
    # keeps the log honest by construction: a bearing the injector altered can
    # never be reported at its requested value.
    log.info("[PTEST] Injecting balloon at longitude=%+.1f deg latitude=%+.1f deg "
             "range=%s", inj.lon, inj.lat,
             "inf" if math.isinf(init_range) else "%.1f m" % init_range)

    # Now switch to ACRO.  This blocks until the FC confirms the mode, but the
    # balloon is already anchored and visible throughout the wait.
    if not ctrl.ptest_enter_acro():
        return _end("failed to enter ACRO")

    # In ACRO now - arm tracking so the cascade starts driving toward the balloon.
    ctrl.ptest_begin(lon_deg, lat_deg)

    # Watch for the path to line up, for as long as the test runs.
    t_inject = time.monotonic()
    t_hold_start = None
    converge_time = None

    # Wait for the FC to answer a GUIDE frame sent after the injection with a
    # commanded tick, so the clock starts when the law is flying THIS balloon.
    # Bounded, so a genuinely stuck test still proceeds and reports honestly.
    warmup_deadline = t_inject + WARMUP_S
    answered = False
    while time.monotonic() < warmup_deadline:
        if abort.is_set() or ctrl.mode != 'ptest':
            return _end("aborted")
        if ctrl.cascade_has_run_since_inject():
            answered = True
            break
        time.sleep(SAMPLE_DT)
    # Start the clock at the first commanded tick.  Without one the clock keeps
    # the injection time: restarting it would leave the wait out of the time
    # the test reports.
    if answered:
        t_inject = time.monotonic()
    else:
        log.warning("[PTEST] the FC did not answer the injection within the "
                    "warm-up; convergence is timed from the injection")

    closest = math.inf
    while True:
        if abort.is_set() or ctrl.mode != 'ptest':
            log.info("=" * 60)
            log.info("[PTEST] NOT ALIGNED")
            log.info("  Balloon:  longitude=%+.1f deg  latitude=%+.1f deg", inj.lon, inj.lat)
            log.info("  Stopped at %.2f s; closest miss angle %.3f deg (aligned: < %.1f deg "
                     "held %.1f s)", time.monotonic() - t_inject, closest,
                     SS_ERROR_DEG, HOLD_S)
            log.info("=" * 60)
            return _end("aborted")
        err = ctrl.alignment_errors()
        # The combined miss ANGLE: how far the flight path is off the balloon,
        # one number.  hypot is already sign-agnostic, so the axes go in raw.
        # No flight path (no FC velocity, or too slow) is not aligned.
        err_mag = math.hypot(err[0], err[1]) if err is not None else math.inf
        closest = min(closest, err_mag)
        if err_mag < SS_ERROR_DEG:
            if t_hold_start is None:
                t_hold_start = time.monotonic()
            if time.monotonic() - t_hold_start >= HOLD_S:
                converge_time = t_hold_start - t_inject
                break
        else:
            t_hold_start = None
        time.sleep(SAMPLE_DT)

    # Measure steady-state and report.  The per-axis means are kept because they
    # say WHERE the residual sits (a pan-only bias is an integrator/bank story, a
    # tilt-only one is a pitch-trim story), but the combined miss angle is added
    # because that is the quantity the verdict was actually gated on.
    ss_pan = []
    ss_tilt = []
    ss_mag = []
    t_ss = time.monotonic()
    while time.monotonic() - t_ss < 0.5:
        if abort.is_set() or ctrl.mode != 'ptest':
            break
        err = ctrl.alignment_errors()
        if err is not None:
            ss_pan.append(abs(err[0]))
            ss_tilt.append(abs(err[1]))
            ss_mag.append(math.hypot(err[0], err[1]))
        time.sleep(SAMPLE_DT)

    mean_pan = float(np.mean(ss_pan)) if ss_pan else float('nan')
    mean_tilt = float(np.mean(ss_tilt)) if ss_tilt else float('nan')
    mean_mag = float(np.mean(ss_mag)) if ss_mag else float('nan')

    log.info("=" * 60)
    log.info("[PTEST] ALIGNED")
    log.info("  Balloon:  longitude=%+.1f deg  latitude=%+.1f deg", inj.lon, inj.lat)
    log.info("  Aligned at:  %.2f s  (within %.1f deg, held %.1f s)",
             converge_time, SS_ERROR_DEG, HOLD_S)
    log.info("  SS miss angle: %.3f deg  (criterion: < %.1f deg)",
             mean_mag, SS_ERROR_DEG)
    log.info("  SS pan error:  %.3f deg   SS tilt error:  %.3f deg", mean_pan, mean_tilt)
    log.info("=" * 60)

    # Keep pursuing until aborted or mode changed.
    while not abort.is_set() and ctrl.mode == 'ptest':
        time.sleep(0.1)

    _end("aborted")
