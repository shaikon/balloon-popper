# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""Gimbal calibration sweep, extracted from controller.py (no behavioural
change).

Sweeps the gimbal through a known (pan, tilt) grid, records the gimbal_cam_link
quaternion at each settled point, and writes gimbal_calib.csv.  Drives the
gimbal driver directly, so balloon chaser/ptest must be off while it runs.

Two entry points, mirroring the original two call paths:
  run_calibration_sweep(gimbal)  - blocking; runs on the caller's thread (used
                                   by the pre-launch path, which must finish
                                   before the catapult fires).
  start_calibration(gimbal)      - the 'c'-key launcher: re-entrancy guard +
                                   spawns the sweep on a daemon thread.  Returns
                                   True if started, False if it was a no-op
                                   (driver lacks readback, or already running).
"""
from __future__ import annotations
import csv as _csv
import logging
import threading
import time

log = logging.getLogger("balloon_chaser")

# Module-level re-entrancy guard for the threaded launcher (a single gimbal /
# controller exists; only one sweep may run at a time).
_calibrating = False


def run_calibration_sweep(gimbal):
    """Blocking gimbal calibration sweep.  Drives `gimbal` directly and writes
    gimbal_calib.csv.  Safe to call on any thread; the caller is responsible for
    ensuring balloon chaser/ptest is not also driving the gimbal."""
    log.info("*** Gimbal calibration START - do not touch balloon chaser/ptest ***")
    try:
        def settle_capture(pan, tilt, settle_s=2.0, hz=50):
            gimbal.command(pan, tilt)
            prev = None
            stable = 0
            for _ in range(int(settle_s * hz)):
                time.sleep(1.0 / hz)
                q = getattr(gimbal, 'measured_quat', None)
                if q is not None and prev is not None:
                    d = sum((a - b) ** 2 for a, b in zip(q, prev)) ** 0.5
                    stable = stable + 1 if d < 1e-5 else 0
                    if stable >= 10:
                        break
                prev = q
            return getattr(gimbal, 'measured_quat', None)

        # Center and grab reference.
        settle_capture(0.0, 0.0, settle_s=3.0)
        q_ref = getattr(gimbal, 'measured_quat', None)
        log.info(f"  calib reference quat: {q_ref}")

        pans = [-60, -40, -20, -10, 0, 10, 20, 40, 60]
        tilts = [-60, -40, -20, -10, 0, 10, 20, 40, 60]
        grid = [(p, 0) for p in pans] + [(0, t) for t in tilts]
        for v in [-40, -20, 20, 40]:
            grid += [(v, v), (v, -v)]

        rows = [('pan_cmd', 'tilt_cmd', 'qx', 'qy', 'qz', 'qw',
                 'ref_qx', 'ref_qy', 'ref_qz', 'ref_qw')]
        for i, (p, t) in enumerate(grid):
            q = settle_capture(p, t)
            if q is None:
                log.info(f"  [{i+1}/{len(grid)}] pan={p:+4} tilt={t:+4} NO POSE")
                continue
            rq = q_ref if q_ref is not None else (0, 0, 0, 0)
            rows.append((p, t, q[0], q[1], q[2], q[3], rq[0], rq[1], rq[2], rq[3]))
            log.info(f"  [{i+1}/{len(grid)}] pan={p:+4} tilt={t:+4} "
                     f"q=({q[0]:+.4f},{q[1]:+.4f},{q[2]:+.4f},{q[3]:+.4f})")

        gimbal.command(0.0, 0.0)  # recenter
        with open('gimbal_calib.csv', 'w', newline='') as f:
            _csv.writer(f).writerows(rows)
        log.info(f"*** Gimbal calibration DONE: gimbal_calib.csv "
                 f"({len(rows)-1} points) ***")
    except Exception as e:                       # noqa: BLE001
        log.error(f"Gimbal calibration failed: {e}")


def start_calibration(gimbal):
    """'c'-key launcher: validate, guard against re-entrancy, and run the sweep
    on a daemon thread.  Returns True if a sweep was started, else False."""
    global _calibrating
    if getattr(gimbal, 'measured_quat', None) is None and \
       not hasattr(gimbal, 'measured_quat'):
        log.warning("Gimbal calibration: driver has no measured_quat; skipping.")
        return False
    if _calibrating:
        log.info("Gimbal calibration already running.")
        return False
    _calibrating = True

    def _worker():
        global _calibrating
        try:
            run_calibration_sweep(gimbal)
        finally:
            _calibrating = False

    threading.Thread(target=_worker, daemon=True, name="gimbal-calib").start()
    return True
