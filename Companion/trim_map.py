# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail).
"""Pitch-trim map ('e' key): the elevator the airframe needs to hold its
attitude, measured across climb angle and airspeed.

WHAT IT MEASURES
    The pitch trim (balloon_chaser_fc.lua, "PITCH TRIM") supplies the elevator the
    rate loop cannot find by itself - the loop has no integrator in ACRO - as

        trim_elev = offset + (VREF / V)^2 * (lift     * L
                                            + attitude * sin(pitch)
                                            + thrust   * throttle / EAS2TAS^2)

    in ArduPlane's elevator units (+-constants.ELEV_FULL_TRAVEL at full
    travel), V the EAS, VREF constants.PITCH_TRIM_VREF_MPS, 1 / EAS2TAS^2
    the air density as a fraction of sea level's and L the lift term the trim
    read (pitch_trim_lift_term: cos(pitch) under the cascade, the load Lift
    Vector Pointing asks of the wing under it).  This run measures what the
    trim must be:
    for each cell of a (climb angle x airspeed) grid it flies the aircraft
    steady on that climb angle at that airspeed and records the elevator
    ArduPlane is flying.  With the nose not rotating, that elevator IS the trim
    for the pitch, airspeed and throttle flown, whatever the gains or the trim
    setting were - they decide how fast the aircraft gets there, not where
    "there" is.

    The result is a map, (pitch, airspeed, throttle) -> elevator.  The report
    fits the formula above to every steady cell and suggests its four gains; the
    cell lines and the log carry everything needed for another fit or a table
    under the cascade, and the flight log the lift term under Lift Vector
    Pointing.

HOW A CELL IS FLOWN
    Wings level, closed loop, in ACRO: the balloon chaser law flies the flight path
    onto a fixed direction dead ahead at the cell's climb angle (the injector's
    force_direction anchor), and the FC's airspeed hold (AIRSPEED_CMD_ENABLE,
    throttle only) holds the cell's airspeed.  Both are the run's
    PRIVATE gains (begin_measurement_gains); the operator's set, trim included,
    is otherwise flown as it stands.  Before each cell the aircraft levels out
    in FBWA, and climbs back to the climb-to altitude first whenever it is below
    the protection floor.

WHAT COUNTS AS STEADY
    Every FC tick after the pull onto the bearing is sampled - both frames of
    ONE tick (lua_offload.lua_tick_frames), stamped with the FC clock, so the
    elevator is always paired with the attitude and airspeed it held.  A cell
    is HELD once a trailing STEADY_S window has the nose not rotating (mean
    |pitch rate| and pitch drift small), wings level, the airspeed at its setpoint
    and not drifting, and the elevator off its stops; the cell's numbers are the
    means over that window.  A cell that cannot get there - the airspeed
    moving away from its goal (out of thrust or lift in a climb, running
    away in a dive) or the throttle pinned without the gap closing
    (speed_stop), the
    altitude floor on every attempt, or the timeout - is recorded NOT HELD with
    its last window's means.  Those are still a trim measurement when the
    window is steady at the airspeed the aircraft could reach (STEADY): only
    the airspeed goal was out of reach, and the fit uses them.  A cell that is out of
    reach because a flown cell already showed it (out_of_reach) is SKIPPED.

OUTPUTS (the tuning-run convention, ARCHITECTURE.md)
    one report (trim_map_report_<ts>.txt), one log CSV
    (trim_map_log_<ts>.csv, every tick sampled - a re-flown or stopped
    attempt's too - tagged with its cell), and one
    resume checkpoint (incomplete_trim_map_run_<ts>.json) saved after every
    cell.  A second 'e', a crash or Ctrl+C keeps the checkpoint; the next 'e'
    resumes at the next unflown cell when the code and the grid are unchanged.

SIM ONLY until validated.  A steep climb at a low airspeed can run out of
thrust, and a slow cell out of elevator; the underspeed stop and the altitude
floor are the guards.
"""
from __future__ import annotations

import glob
import json
import logging
import math
import os
import time

import numpy as np

from . import constants as C
from . import file_export
from .csv_logger import PACKAGE_SHA256

log = logging.getLogger("balloon_chaser")

# ---------------------------------------------------------------------------
# Cell flight and steady-state criteria
# ---------------------------------------------------------------------------
POLL_S = 0.01             # poll twice per FC tick so none is missed
WARMUP_S = 0.5            # wait for the first commanded tick after injection
MIN_FLY_S = 4.0           # the pull onto the bearing, before steadiness is judged
CELL_TIMEOUT_S = 45.0     # give up on a cell that never holds
STEADY_S = 3.0            # trailing window that must be steady; the cell's numbers
Q_TOL_DPS = 1.0           # mean |pitch rate| over the window
PITCH_DRIFT_DEG = 1.0     # pitch change across the window
ROLL_TOL_DEG = 10.0       # |roll| everywhere in the window: 1.5% more lift at 10
SPEED_TOL_MPS = 0.75      # |mean airspeed - goal|
SPEED_DRIFT_MPS = 0.5     # airspeed change across the window
ELEV_STOP_MARGIN = 1.0    # elevator units short of full travel that count as on the stop
ELEV_STOP = C.ELEV_FULL_TRAVEL - ELEV_STOP_MARGIN   # |elevator| at or past this: on the stop
SAT_GIVEUP_S = 6.0        # throttle pinned this long, speed off goal, not closing
UNDERSPEED_MPS = 2.0      # this far below goal AND falling: out of thrust, stop
OVERSPEED_MPS = 8.0       # this far above goal AND rising: a dive running away
TREND_S = 1.0             # span the airspeed trend is judged over
CLOSING_MPS = 0.2         # the error must shrink by this per TREND_S to count as closing
THR_FULL_PCT = 99.0       # a window's mean throttle at or above this: the motor's limit
THR_IDLE_PCT = 1.0        # at or below this: idle, the dive's limit
FLOOR_RETRIES = 1         # re-flies after an altitude-floor breach


# ---------------------------------------------------------------------------
# The grid
# ---------------------------------------------------------------------------
def _axis(lo, hi, step):
    """Inclusive values from lo to hi in steps of |step|; a single value when
    step <= 0 or lo == hi."""
    lo, hi, step = float(lo), float(hi), abs(float(step))
    if step <= 0.0 or lo == hi:
        return [lo]
    a, b = min(lo, hi), max(lo, hi)
    n = int(math.floor((b - a) / step + 1e-9))
    return [round(a + i * step, 6) for i in range(n + 1)]


def grid_of(ctrl):
    """(climb angles, airspeeds) from the live Gain-Tuner rows."""
    climbs = _axis(ctrl.trim_map_climb_min_deg, ctrl.trim_map_climb_max_deg,
                   ctrl.trim_map_climb_step_deg)
    speeds = _axis(ctrl.trim_map_speed_min_mps, ctrl.trim_map_speed_max_mps,
                   ctrl.trim_map_speed_step_mps)
    return climbs, speeds


def fly_order(climbs):
    """Climb angles in flight order: the steepest climb, then the steepest dive,
    then the next of each, level last.  Alternating keeps the height a climb
    gains for the dive after it, so the floor trips less."""
    ups = sorted((c for c in climbs if c > 0), reverse=True)
    downs = sorted(c for c in climbs if c < 0)
    order = []
    for i in range(max(len(ups), len(downs))):
        if i < len(ups):
            order.append(ups[i])
        if i < len(downs):
            order.append(downs[i])
    return order + [c for c in climbs if c == 0]


def cells_of(climbs, speeds):
    """Every (climb, airspeed) cell in flight order: each airspeed in turn,
    ascending, with its climb angles in fly_order."""
    return [(c, v) for v in sorted(speeds) for c in fly_order(climbs)]


def cell_key(climb, speed):
    return "%+.1f@%.1f" % (float(climb), float(speed))


# ---------------------------------------------------------------------------
# One FC tick
# ---------------------------------------------------------------------------
def _num(x):
    """A decoded field as a float, None when the FC sent NaN."""
    try:
        x = float(x)
    except (TypeError, ValueError):
        return None
    return None if x != x else x


def sample_from_frames(frames):
    """One FC tick's reading from its (STATE, STATE2) pair, or None when the
    tick lacks the attitude, airspeed or elevator.

    Pitch and roll are the Euler angles of the FC's own quaternion that tick,
    the angle the trim formula takes the sine of; the elevator is ArduPlane's
    scaled output before the V-tail mix, in the formula's units."""
    st, st2 = frames
    qw, qx, qy, qz = (_num(st2[k]) for k in ("q_w", "q_x", "q_y", "q_z"))
    eas, elev = _num(st2["eas_mps"]), _num(st2["out_elevator"])
    gy = _num(st2["gyro_y_rads"])
    if None in (qw, qx, qy, qz, eas, elev, gy):
        return None
    vn, ve, vd = (_num(st2[k]) for k in ("vel_n_mps", "vel_e_mps", "vel_d_mps"))
    path = (None if None in (vn, ve, vd)
            else math.degrees(math.atan2(-vd, math.hypot(vn, ve))))
    gate = int(st["gate"])
    return {
        "t_s": int(st["fc_millis"]) / 1000.0,
        "pitch_deg": math.degrees(math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))),
        "roll_deg": math.degrees(math.atan2(2.0 * (qw * qx + qy * qz),
                                            1.0 - 2.0 * (qx * qx + qy * qy))),
        "q_dps": math.degrees(gy),
        "eas_mps": eas,
        "aoa_deg": _num(st2["aoa_deg"]),
        "path_deg": path,
        "elevator": elev / C.ELEV_OUTPUT_PER_UNIT,
        "throttle_pct": _num(st2["out_throttle"]),
        "eas2tas": _num(st2["eas2tas"]),
        "trim_elev": _num(st["pitch_trim_elev"]),
        # the lift term the trim read this tick: cos(pitch) under the cascade,
        # the lift load under Lift Vector Pointing
        "lift_term": _num(st["pitch_trim_lift_term"]),
        "omega_y_dps": _num(st["omega_y_cmd_dps"]),
        "gate": gate,
        # asp_flags bit 1: the airspeed loop's output is on 0 or 100 %.  It
        # means something only on a commanded tick.
        "asp_sat": (bool(int(st["asp_flags"]) & 0x02) if gate == 0 else None),
    }


# ---------------------------------------------------------------------------
# Steadiness and the cell's numbers
# ---------------------------------------------------------------------------
def _mean(xs):
    xs = [x for x in xs if x is not None]
    return sum(xs) / len(xs) if xs else None


def _sd(xs):
    xs = [x for x in xs if x is not None]
    if len(xs) < 2:
        return None
    m = sum(xs) / len(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def trailing(samples, span_s):
    """The samples in the last span_s seconds of FC time."""
    if not samples:
        return []
    t_end = samples[-1]["t_s"]
    return [s for s in samples if s["t_s"] >= t_end - span_s]


def eas_trend(samples, span_s=TREND_S):
    """Airspeed change over the last span_s of FC time, m/s; None until the
    samples span it."""
    w = trailing(samples, span_s)
    if len(w) < 2 or w[-1]["t_s"] - w[0]["t_s"] < span_s - 0.05:
        return None
    return w[-1]["eas_mps"] - w[0]["eas_mps"]


def speed_stop(samples, speed_goal, sat_s):
    """Why the cell must stop on airspeed, or None.

    Every cell starts at the level-out's airspeed, not its own, so being far
    from the goal is normal while the airspeed hold closes the gap.  Each
    stop therefore also needs the airspeed to be going the WRONG way: below
    the goal and falling (out of thrust for this climb - the stall guard),
    above it and rising (a dive the idle throttle cannot hold back), or pinned
    throttle for SAT_GIVEUP_S without the gap closing.  sat_s is how long the
    airspeed loop's output has been pinned."""
    v = samples[-1]["eas_mps"]
    trend = eas_trend(samples)
    if trend is None:
        return None
    err = v - speed_goal
    thr = samples[-1]["throttle_pct"]
    thr_s = "unknown" if thr is None else "%.0f%%" % thr
    if err < -UNDERSPEED_MPS and trend < 0.0:
        return "underspeed (%.1f m/s and falling, throttle %s)" % (v, thr_s)
    if err > OVERSPEED_MPS and trend > 0.0:
        return "overspeed (%.1f m/s and rising, throttle %s)" % (v, thr_s)
    closing = (trend * (-1.0 if err > 0 else 1.0)) >= CLOSING_MPS
    if sat_s >= SAT_GIVEUP_S and abs(err) >= SPEED_TOL_MPS and not closing:
        return ("airspeed unreachable: throttle held at %.0f%% with %.1f m/s"
                % (samples[-1]["throttle_pct"] or 0.0, v))
    return None


def steady_why_not(window, speed_goal=None):
    """None when the window is steady, else the first reason it is not.  The
    window must span STEADY_S of FC time.  Without a speed_goal it asks only
    whether the aircraft is steady - at whatever airspeed it holds - which is
    what makes a window a trim measurement."""
    if len(window) < 2 or window[-1]["t_s"] - window[0]["t_s"] < STEADY_S - 0.05:
        return "window too short"
    if _mean(abs(s["q_dps"]) for s in window) >= Q_TOL_DPS:
        return "nose rotating"
    if abs(window[-1]["pitch_deg"] - window[0]["pitch_deg"]) >= PITCH_DRIFT_DEG:
        return "pitch drifting"
    if max(abs(s["roll_deg"]) for s in window) >= ROLL_TOL_DEG:
        return "wings not level"
    if (speed_goal is not None
            and abs(_mean(s["eas_mps"] for s in window) - speed_goal) >= SPEED_TOL_MPS):
        return "airspeed off goal"
    if abs(window[-1]["eas_mps"] - window[0]["eas_mps"]) >= SPEED_DRIFT_MPS:
        return "airspeed drifting"
    if max(abs(s["elevator"]) for s in window) >= ELEV_STOP:
        return "elevator on its stop"
    return None


def summarize(window, climb, speed, status):
    """A cell's result from the samples of its recorded window."""
    return {
        "climb_deg": float(climb), "speed_goal_mps": float(speed),
        "status": status, "n": len(window),
        "pitch_deg": _mean(s["pitch_deg"] for s in window),
        "pitch_sd": _sd(s["pitch_deg"] for s in window),
        "eas_mps": _mean(s["eas_mps"] for s in window),
        "eas_sd": _sd(s["eas_mps"] for s in window),
        "elevator": _mean(s["elevator"] for s in window),
        "elevator_sd": _sd(s["elevator"] for s in window),
        "elevator_peak": max(abs(s["elevator"]) for s in window),
        "aoa_deg": _mean(s["aoa_deg"] for s in window),
        "path_deg": _mean(s["path_deg"] for s in window),
        "throttle_pct": _mean(s["throttle_pct"] for s in window),
        "eas2tas": _mean(s["eas2tas"] for s in window),
        "q_dps": _mean(s["q_dps"] for s in window),
        "trim_elev": _mean(s["trim_elev"] for s in window),
        "lift_term": _mean(s.get("lift_term") for s in window),
    }


def held(result):
    return result.get("status") == "held" and result.get("elevator") is not None


def steady(result):
    """A trim measurement: held, or steady at an airspeed other than the one
    asked for."""
    return bool(result.get("steady")) and result.get("elevator") is not None


def skipped(result):
    return str(result.get("status", "")).startswith("skipped")


def limit_of(result, why):
    """What stopped the airspeed at this climb, for out_of_reach: 'thrust'
    when the motor was at its top - full throttle over the window, or the
    airspeed loop pinned with the airspeed still below its goal - 'idle' for
    the same at the bottom, else None.  The pinned loop (the 'airspeed
    unreachable' stop) is what catches a throttle ceiling below THR_FULL_PCT:
    the loop pins at its own 0-100 % whatever the throttle output is then held
    to.

    An airspeed running away is not by itself the motor's limit, and taking it
    for one skips cells that are well within reach.  With the elevator on its
    stop the wing has run out of lift at this airspeed: the aircraft mushes and
    sinks, and the airspeed falls or rises at any throttle, so such a window
    says nothing about the motor.  An underspeed or overspeed stop with the
    throttle off its end is the same: marked by the throttle, not the stop."""
    why = str(why or "")
    peak = result.get("elevator_peak")
    if peak is not None and peak >= ELEV_STOP:
        return None
    thr = result.get("throttle_pct")
    v, goal = result.get("eas_mps"), result.get("speed_goal_mps")
    pinned = why.startswith("airspeed unreachable") and None not in (v, goal)
    if (thr is not None and thr >= THR_FULL_PCT) or (pinned and v < goal):
        return "thrust"
    if (thr is not None and thr <= THR_IDLE_PCT) or (pinned and v > goal):
        return "idle"
    return None


def out_of_reach(results, climb, speed):
    """Why a cell need not be flown, or None.  A climb the motor held at full
    throttle at airspeed v cannot be held faster, nor can a steeper one; a dive
    at idle at airspeed v cannot be held slower, nor can a steeper one."""
    for r in results:
        v = r.get("eas_mps")
        if v is None:
            continue
        if (r.get("limit") == "thrust" and r["climb_deg"] <= climb
                and speed > v + SPEED_TOL_MPS):
            return ("skipped: the motor holds at most %.1f m/s at climb %+.0f"
                    % (v, r["climb_deg"]))
        if (r.get("limit") == "idle" and r["climb_deg"] >= climb
                and speed < v - SPEED_TOL_MPS):
            return ("skipped: idle throttle holds no less than %.1f m/s at climb %+.0f"
                    % (v, r["climb_deg"]))
    return None


# ---------------------------------------------------------------------------
# The trim formula, fitted to the map
# ---------------------------------------------------------------------------
TRIM_GAINS = ("pitch_trim_offset", "pitch_trim_lift", "pitch_trim_attitude",
              "pitch_trim_thrust")
TRIM_ROWS = ("Pitch Trim Offset elev deg", "Pitch Trim Lift elev deg",
             "Pitch Trim Attitude elev deg", "Pitch Trim Thrust elev deg")


def trim_terms(pitch_deg, eas_mps, throttle_pct, eas2tas, lift_term=None):
    """The formula's four regressors - what each gain multiplies - exactly as
    balloon_chaser_fc.lua forms them: 1, and (VREF/V)^2 times the lift term,
    sin(pitch) and the throttle fraction times the density ratio 1 / EAS2TAS^2.
    The lift term is the one the trim read (pitch_trim_lift_term: cos(pitch)
    under the cascade, the lift load under Lift Vector Pointing); None, where
    no tick of the cell logged one, reads cos(pitch), the cascade's."""
    th = math.radians(pitch_deg)
    q = (C.PITCH_TRIM_VREF_MPS / eas_mps) ** 2
    lift = math.cos(th) if lift_term is None else lift_term
    return (1.0, q * lift, q * math.sin(th),
            q * throttle_pct / C.THROTTLE_OUTPUT_FULL / eas2tas ** 2)


def fit_trim(results):
    """Least squares of the trim formula over every STEADY cell, each weighted
    equally: ({gain: value}, rms, max |residual|, {cell_key: residual}), or
    None with fewer than five usable cells or too little spread to separate
    the four terms."""
    rows, ys, keys = [], [], []
    for r in results:
        if (not steady(r)
                or None in (r.get("pitch_deg"), r.get("eas_mps"),
                            r.get("throttle_pct"), r.get("eas2tas"))
                or r["eas_mps"] <= 0.0 or r["eas2tas"] <= 0.0):
            continue
        rows.append(trim_terms(r["pitch_deg"], r["eas_mps"], r["throttle_pct"],
                               r["eas2tas"], r.get("lift_term")))
        ys.append(r["elevator"])
        keys.append(cell_key(r["climb_deg"], r["speed_goal_mps"]))
    if len(rows) < len(TRIM_GAINS) + 1:
        return None
    X, y = np.array(rows), np.array(ys)
    coef, _res, rank, _sv = np.linalg.lstsq(X, y, rcond=None)
    if rank < len(TRIM_GAINS):
        return None
    resid = y - X @ coef
    return (dict(zip(TRIM_GAINS, (float(c) for c in coef))),
            float(np.sqrt(np.mean(resid ** 2))), float(np.max(np.abs(resid))),
            dict(zip(keys, (float(e) for e in resid))))


# ---------------------------------------------------------------------------
# The report
# ---------------------------------------------------------------------------
def _mark(r):
    """' ' held, '~' steady at another airspeed than asked (used in the fit),
    '*' not steady (left out)."""
    return " " if held(r) else ("~" if steady(r) else "*")


def _table(title, results, climbs, speeds, value, fmt="%6.1f"):
    """A climb x airspeed table of one quantity, each value marked by _mark;
    '-' is a cell skipped as out of reach or not flown."""
    by = {cell_key(r["climb_deg"], r["speed_goal_mps"]): r for r in results}
    L = [title, "  climb \\ airspeed " + "".join("%8.0f" % v for v in sorted(speeds))]
    for c in sorted(climbs, reverse=True):
        row = "  %+6.0f deg        " % c
        for v in sorted(speeds):
            r = by.get(cell_key(c, v))
            x = None if (r is None or skipped(r)) else value(r)
            if r is None or skipped(r):
                row += "%8s" % "-"
            elif x is None:
                row += "%8s" % "?"
            else:
                row += "%7s%s" % ((fmt % x).strip(), _mark(r))
        L.append(row)
    return L + [""]


def report_lines(results, climbs, speeds, run_ts, log_name, trim_flown,
                 transcript, resumed):
    """The report as a list of lines.  trim_flown is {gain: value} of the
    operator's trim during the run."""
    total = len(climbs) * len(speeds)
    n_skip = sum(1 for r in results if skipped(r))
    n_flown = len(results) - n_skip
    n_held = sum(1 for r in results if held(r))
    n_steady = sum(1 for r in results if steady(r))
    L = ["TRIM MAP REPORT",
         "code_sha256 %s" % PACKAGE_SHA256,
         "run         %s" % run_ts,
         "grid        climb %s deg x airspeed %s m/s = %d cells"
         % (", ".join("%+.0f" % c for c in sorted(climbs)),
            ", ".join("%.0f" % v for v in sorted(speeds)), total),
         "flown       %d cells: %d held, %d steady at another airspeed, %d not steady"
         % (n_flown, n_held, n_steady - n_held, n_flown - n_steady),
         "skipped     %d cells out of reach; %d not flown"
         % (n_skip, total - len(results)),
         "log         %s" % log_name,
         "pitch trim flown during the run: %s" % ", ".join(
             "%s %.2f" % (g.replace("pitch_trim_", ""), trim_flown.get(g, 0.0))
             for g in TRIM_GAINS),
         "            (it changes how fast a cell settles, not the elevator a",
         "            steady cell needs)",
         ""]
    L += ["=" * 72,
          "ELEVATOR TO HOLD THE ATTITUDE - ArduPlane elevator units, %g = full"
          % C.ELEV_FULL_TRAVEL,
          "travel, positive = nose up: the number the trim formula must produce.",
          "  (blank) held   ~ steady at another airspeed than asked (used in the fit)",
          "  * not steady (best estimate from its last window; left out of the fit)",
          "  - skipped as out of reach, or not flown", ""]
    L += _table("elevator", results, climbs, speeds, lambda r: r.get("elevator"))
    L += _table("pitch flown, deg", results, climbs, speeds, lambda r: r.get("pitch_deg"))
    L += _table("airspeed flown, m/s", results, climbs, speeds, lambda r: r.get("eas_mps"))
    L += _table("angle of attack, deg", results, climbs, speeds, lambda r: r.get("aoa_deg"))
    L += _table("flight path, deg", results, climbs, speeds, lambda r: r.get("path_deg"))
    L += _table("throttle, %", results, climbs, speeds, lambda r: r.get("throttle_pct"),
                "%6.0f")

    fit = fit_trim(results)
    L += ["=" * 72,
          "THE TRIM FORMULA FITTED TO EVERY STEADY CELL      (REVIEW; nothing is applied)",
          "  elevator = offset + (VREF / airspeed)^2 x (lift x L",
          "                          + attitude x sin(pitch)",
          "                          + thrust x throttle / EAS2TAS^2)",
          "  with VREF = %g m/s (PITCH_TRIM_VREF_MPS), the airspeed EAS in m/s,"
          % C.PITCH_TRIM_VREF_MPS,
          "  the throttle a fraction of full, 1 / EAS2TAS^2 the air density as",
          "  a fraction of sea level's, and L the lift term the trim read",
          "  (pitch_trim_lift_term): cos(pitch) under the cascade, the load the",
          "  law asks of the wing under Lift Vector Pointing.", ""]
    if fit is None:
        L += ["  not enough steady cells to fit (need five, spread in pitch, airspeed",
              "  and throttle).", ""]
    else:
        gains, rms, worst, resid = fit
        L += ["  %-28s = %.2f" % (row, gains[g]) for row, g in zip(TRIM_ROWS, TRIM_GAINS)]
        L += ["  %-28s = %.2f elevator units over %d cells (worst %.2f)"
              % ("rms residual", rms, len(resid), worst), "",
              "  Measured minus fitted, per steady cell.  A pattern across a row or",
              "  a column is a dependence the formula lacks.", ""]
        L += _table("residual, elevator units", results, climbs, speeds,
                    lambda r: resid.get(cell_key(r["climb_deg"], r["speed_goal_mps"])),
                    "%6.2f")

    cols = ("climb_deg", "speed_goal_mps", "status", "steady", "n", "pitch_deg",
            "pitch_sd", "eas_mps", "eas_sd", "elevator", "elevator_sd", "aoa_deg",
            "path_deg", "throttle_pct", "eas2tas", "q_dps", "trim_elev")
    L += ["=" * 72,
          "CELL DATA - one line per cell, for a fit or a lookup table.", "",
          ",".join(cols)]
    for r in sorted(results, key=lambda r: (r["speed_goal_mps"], -r["climb_deg"])):
        vals = []
        for c in cols:
            x = r.get(c)
            if c == "steady":
                vals.append("1" if steady(r) else "0")
            elif x is None:
                vals.append("")
            elif isinstance(x, str):
                vals.append(x)
            else:
                vals.append("%d" % x if c == "n" else "%.4f" % x)
        L.append(",".join(vals))
    L += ["", "=" * 72,
          "RUN TRANSCRIPT - this session's console output%s."
          % (" (the run was resumed; earlier sessions' cells are in the tables above)"
             if resumed else ""), ""]
    return L + list(transcript)


# ---------------------------------------------------------------------------
# Checkpoint
# ---------------------------------------------------------------------------
_CKPT_NAME = "incomplete_trim_map_run"
_CKPT_GLOB = "incomplete_trim_map_run_*.json"
_CKPT_VERSION = 1


def grid_signature(climbs, speeds):
    return [list(map(float, climbs)), list(map(float, speeds))]


def ckpt_load(signature, cwd=None):
    """(run_ts, [results]) from a checkpoint this run may resume, or
    (None, []).  A different code fingerprint or grid starts afresh: the cells
    were flown against something else."""
    found = sorted(glob.glob(os.path.join(cwd or os.getcwd(), _CKPT_GLOB)))
    if not found:
        return None, []
    try:
        with open(found[-1]) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return None, []
    if not isinstance(data, dict) or data.get("version") != _CKPT_VERSION:
        return None, []
    if data.get("fingerprint") != PACKAGE_SHA256:
        log.info("[TRIMMAP] checkpoint %s was flown with different code - "
                 "starting afresh", os.path.basename(found[-1]))
        return None, []
    if data.get("grid") != signature:
        log.info("[TRIMMAP] checkpoint %s has a different grid - starting afresh",
                 os.path.basename(found[-1]))
        return None, []
    return data.get("run_ts"), list(data.get("results") or [])


def ckpt_save(run_ts, signature, results, cwd=None):
    path = os.path.join(cwd or os.getcwd(), "%s_%s.json" % (_CKPT_NAME, run_ts))
    try:
        with open(path, "w") as f:
            json.dump({"version": _CKPT_VERSION, "fingerprint": PACKAGE_SHA256,
                       "run_ts": run_ts, "grid": signature, "results": results},
                      f, indent=1)
        for old in file_export.claim(_CKPT_GLOB, path, src_dir=cwd):
            log.info("[TRIMMAP] cleared previous checkpoint %s", old)
    except OSError as e:
        log.warning("[TRIMMAP] could not save the checkpoint (%s)", e)


def ckpt_clear(cwd=None):
    for p in glob.glob(os.path.join(cwd or os.getcwd(), _CKPT_GLOB)):
        try:
            os.remove(p)
        except OSError:
            pass


# ---------------------------------------------------------------------------
# The run's two completed outputs
# ---------------------------------------------------------------------------
_LOG_COLS = ("cell", "climb_deg", "speed_goal_mps", "t_s", "pitch_deg",
             "roll_deg", "q_dps", "eas_mps", "aoa_deg", "path_deg", "elevator",
             "throttle_pct", "eas2tas", "trim_elev", "omega_y_dps", "gate",
             "asp_sat", "in_window")


class _Outputs:
    """The ONE report and ONE log CSV a run emits.  A resumed run appends
    to the CSV its first session opened (same run stamp), so the file holds
    every cell's samples; the report is rewritten from every cell's result."""

    def __init__(self, run_ts):
        self.run_ts = run_ts
        self.csv_path = file_export.stamped("trim_map_log", "csv", run_ts)
        self.report_path = file_export.stamped("trim_map_report", "txt", run_ts)
        self._csv = None

    def open_csv(self):
        new = not os.path.exists(self.csv_path)
        try:
            self._csv = open(self.csv_path, "a")
            if new:
                self._csv.write("# code_sha256: %s\n" % PACKAGE_SHA256)
                self._csv.write(",".join(_LOG_COLS) + "\n")
        except OSError as e:
            log.warning("[TRIMMAP] could not open the log CSV (%s)", e)
            self._csv = None

    def write_cell(self, climb, speed, samples, window_t0):
        """Append one attempt's samples; in_window marks the window the kept
        result was averaged over, and nothing when window_t0 is None."""
        if self._csv is None:
            return
        key = cell_key(climb, speed)
        try:
            for s in samples:
                vals = [key, "%.1f" % climb, "%.1f" % speed]
                for c in _LOG_COLS[3:-1]:
                    x = s.get(c)
                    vals.append("" if x is None else
                                ("%d" % x if isinstance(x, (bool, int)) else "%.4f" % x))
                vals.append("1" if (window_t0 is not None and s["t_s"] >= window_t0) else "0")
                self._csv.write(",".join(vals) + "\n")
            self._csv.flush()
        except Exception:                                     # noqa: BLE001
            pass                                              # never break a run to log

    def finish(self, lines):
        if self._csv is not None:
            try:
                self._csv.close()
            except Exception:                                 # noqa: BLE001
                pass
        # Completed-run outputs are KEPT (timestamped, never claimed away);
        # only the checkpoint is cleaned.
        try:
            with open(self.report_path, "w") as f:
                f.write("\n".join(lines) + "\n")
            log.info("[TRIMMAP] report: %s  (Copy it out with the Trim-map report "
                     "button)", os.path.basename(self.report_path))
        except OSError as e:
            log.warning("[TRIMMAP] could not write the report (%s)", e)


# ---------------------------------------------------------------------------
# Flight
# ---------------------------------------------------------------------------
class _Aborted(Exception):
    """The run was stopped mid-run: 'e' again, or another mode key switching
    straight to its own mode."""


def _below_floor(ctrl):
    """True if AGL is below the trim map's live protection floor; missing
    telemetry is False, so a link without altitude cannot false-trip a climb."""
    try:
        return float(ctrl.link.relative_alt) < float(ctrl.trim_map_protect_floor_m)
    except Exception:                                         # noqa: BLE001
        return False


def _set_phase(ctrl, base, phase):
    """HUD status: the cell line and its phase.  The altitude protection
    appends after ' | ', so the base must not contain one."""
    ctrl.autotune_status = "%s - %s" % (base, phase) if phase else base


def _protect(ctrl, abort):
    """Climb to the climb-to altitude when below the floor - never
    pre-emptively.  Leaves the aircraft in FBWA; the level-out follows."""
    climb = getattr(ctrl.link, "climb_to", None)
    if climb is None or not _below_floor(ctrl):
        return
    climb_to = float(ctrl.trim_map_protect_climb_to_m)
    log.info("[TRIMMAP] altitude %.0fm < floor %.0fm - climbing to %.0fm",
             getattr(ctrl.link, "relative_alt", -1.0),
             float(ctrl.trim_map_protect_floor_m), climb_to)
    base = (getattr(ctrl, "autotune_status", "") or "").split(" | ")[0]
    ctrl.autotune_status = ((base + " | ") if base else "") + \
        "Alt protect: climbing to %.0fm" % climb_to
    ctrl.autotune_pause_tracking()
    try:
        climb(climb_to, abort)
    except Exception as e:                                    # noqa: BLE001
        log.warning("[TRIMMAP] protection climb failed (%s) - continuing", e)
    if base:
        ctrl.autotune_status = base
    if abort.is_set():
        raise _Aborted


def _level_out(ctrl, abort):
    """The sweep's straight-and-level hold in FBWA (same tolerances, imported),
    then ACRO for the cell.  A level-out timeout logs and proceeds."""
    from .controller_autotune_sweep import (
        STRAIGHT_HOLD_S, STRAIGHT_PITCHRATE_TOL_DPS, STRAIGHT_PITCH_TOL_DEG,
        STRAIGHT_ROLL_TOL_DEG, STRAIGHT_TIMEOUT_S, STRAIGHT_YAWRATE_TOL_DPS)
    ctrl.autotune_pause_tracking()
    throttle_frac = max(0.0, min(1.0, float(ctrl.throttle_pct) / 100.0))
    ctrl.straighten_start_t = time.monotonic()        # the HUD "Straight" timer
    try:
        achieved = ctrl.link.level_and_hold(
            STRAIGHT_ROLL_TOL_DEG, STRAIGHT_YAWRATE_TOL_DPS, STRAIGHT_HOLD_S,
            STRAIGHT_TIMEOUT_S, throttle_frac=throttle_frac, abort=abort,
            pitch_tol_deg=STRAIGHT_PITCH_TOL_DEG,
            pitch_rate_tol_dps=STRAIGHT_PITCHRATE_TOL_DPS)
    finally:
        ctrl.straighten_start_t = None
    if abort.is_set():
        raise _Aborted
    if not achieved:
        log.info("[TRIMMAP] could not hold wings-level within %.0fs - the cell "
                 "starts from a non-level attitude", STRAIGHT_TIMEOUT_S)
    if not ctrl.ptest_enter_acro():
        raise RuntimeError("ACRO re-entry after the level-out failed")


def _fly_cell(ctrl, abort, climb, speed, base, samples):
    """Fly one cell once, appending every tick sampled to the caller's samples
    list - so a stop mid-cell still leaves them for the log.  Returns (result,
    window_t0, why): why is None for a held cell, 'floor' when the altitude
    floor stopped it (the caller climbs and re-flies), else the reason it was
    not held."""
    from .testing.synthetic_injector import SyntheticBalloonInjector
    link = ctrl.link
    ctrl.update_measurement_gains(airspeed_cmd_enable=1, airspeed_cmd_mps=float(speed))
    inj = SyntheticBalloonInjector(link, ctrl.sensor, 0.0, float(climb),
                                  force_direction=True)
    ctrl.set_point_injector(inj)
    if not inj.start():
        ctrl.autotune_pause_tracking()
        return None, None, "injector precondition failed"
    ctrl.ptest_begin(0.0, float(climb))
    ctrl.sensor.set_gimbal_locked(True)
    t_w = time.monotonic()
    while time.monotonic() - t_w < WARMUP_S and not ctrl.cascade_has_run_since_inject():
        if abort.is_set():
            raise _Aborted
        time.sleep(POLL_S)

    last_key = None
    t0 = time.monotonic()
    sat_since = None
    why = "timeout"
    ctrl.autotune_test_t0, ctrl.autotune_test_done_s = t0, None
    try:
        while time.monotonic() - t0 < CELL_TIMEOUT_S:
            if abort.is_set():
                raise _Aborted
            if _below_floor(ctrl):
                why = "floor"
                break
            frames = getattr(link, "lua_tick_frames", None)
            key = None if frames is None else (frames[0]["seq"], frames[0]["fc_millis"])
            if key is None or key == last_key:
                time.sleep(POLL_S)
                continue
            last_key = key
            s = sample_from_frames(frames)
            if s is None:
                continue
            samples.append(s)
            flown = time.monotonic() - t0
            if flown < MIN_FLY_S:
                _set_phase(ctrl, base, "pulling onto the bearing")
                continue
            if s["asp_sat"]:
                sat_since = sat_since if sat_since is not None else s["t_s"]
            else:
                sat_since = None
            stop = speed_stop(samples, speed,
                              0.0 if sat_since is None else s["t_s"] - sat_since)
            if stop is not None:
                why = stop
                break
            window = trailing(samples, STEADY_S)
            no = steady_why_not(window, speed)
            if no is None:
                ctrl.autotune_test_done_s = time.monotonic() - t0
                result = summarize(window, climb, speed, "held")
                result["steady"] = True
                result["limit"] = limit_of(result, None)
                return result, window[0]["t_s"], None
            _set_phase(ctrl, base, "settling (%s)" % no)
    finally:
        ctrl.autotune_pause_tracking()
        if ctrl.autotune_test_done_s is None:
            ctrl.autotune_test_done_s = time.monotonic() - t0
    window = trailing(samples, STEADY_S)
    if not window:
        return None, None, why
    result = summarize(window, climb, speed, "not held: %s" % why)
    # Steady at the airspeed it could reach: still a trim measurement.
    result["steady"] = steady_why_not(window) is None
    result["limit"] = limit_of(result, why)
    return result, window[0]["t_s"], why


def run_trim_map(ctrl, abort):
    """The 'e' run: every cell of the grid not already in the checkpoint, each
    self-contained (protection climb if low, level-out, the cell), a checkpoint
    after each, and the report at the end - whether the run finished, was
    stopped (by 'e', or by another mode key switching away), or failed."""
    from .plant_id import _LogCapture
    cap = _LogCapture()
    log.addHandler(cap)
    ctrl.autotune_start_t = time.monotonic()
    # The run is underway: a second 'e' must reach the abort, not the
    # transition guard, even before the first level-out's ACRO entry clears it.
    ctrl._tracking_transition = False
    # All setup is inside the try, so the finally always runs: a failure
    # before it would skip the hand-back and leave the HUD on a run that is
    # not flying.
    out, owned, results, total = None, False, [], 0
    try:
        climbs, speeds = grid_of(ctrl)
        signature = grid_signature(climbs, speeds)
        ckpt_ts, results = ckpt_load(signature)
        run_ts = ctrl.adopt_run_stamp("e", ckpt_ts) if ckpt_ts else ctrl.run_stamp_for("e")
        resumed = bool(results)
        done = {cell_key(r["climb_deg"], r["speed_goal_mps"]) for r in results}
        todo = [c for c in cells_of(climbs, speeds) if cell_key(*c) not in done]
        total = len(climbs) * len(speeds)
        log.info("=== Trim map: %d cells (climb %s deg x airspeed %s m/s)%s ===", total,
                 ", ".join("%+.0f" % c for c in sorted(climbs)),
                 ", ".join("%.0f" % v for v in sorted(speeds)),
                 ("  (resuming; %d already flown)" % len(results)) if resumed else "")
        g = ctrl.operator_gains()
        trim_flown = {k: float(getattr(g, k, 0.0)) for k in TRIM_GAINS}
        out = _Outputs(run_ts)
        out.open_csv()
        ctrl.begin_measurement_gains("trim map", airspeed_cmd_enable=1,
                                     airspeed_cmd_mps=float(todo[0][1] if todo else speeds[0]))
        owned = True
        for climb, speed in todo:
            n = len(results) + 1
            base = "cell %d/%d: climb %+.0f @ %.0f m/s" % (n, total, climb, speed)
            skip = out_of_reach(results, climb, speed)
            if skip is not None:
                results.append({"climb_deg": float(climb), "speed_goal_mps": float(speed),
                                "status": skip, "n": 0})
                log.info("[TRIMMAP] cell %s %s", cell_key(climb, speed), skip)
                ckpt_save(run_ts, signature, results)
                continue
            for attempt in range(FLOOR_RETRIES + 1):
                _set_phase(ctrl, base, "")
                _protect(ctrl, abort)
                _set_phase(ctrl, base, "level-out")
                _level_out(ctrl, abort)
                samples, kept_t0 = [], None
                try:
                    result, window_t0, why = _fly_cell(ctrl, abort, climb, speed,
                                                       base, samples)
                    last = why != "floor" or attempt == FLOOR_RETRIES
                    kept_t0 = window_t0 if last else None
                finally:
                    # Every tick flown goes in the log - a re-flown attempt's and
                    # a stopped one's too; in_window marks only the window of the
                    # result that is kept.
                    out.write_cell(climb, speed, samples, kept_t0)
                if last:
                    break
                log.warning("[TRIMMAP] cell %s hit the altitude floor - climbing and "
                            "re-flying it", cell_key(climb, speed))
            if why == "floor":
                why = "altitude floor on every attempt"
                if result is not None:
                    result["status"] = "not held: %s" % why
            if result is None:
                result = {"climb_deg": float(climb), "speed_goal_mps": float(speed),
                          "status": "not held: %s" % why, "n": 0}
            results.append(result)
            if steady(result):
                log.info("[TRIMMAP] cell %s %s: elevator %.2f at pitch %.1f deg, "
                         "%.1f m/s, AoA %s, throttle %.0f%%", cell_key(climb, speed),
                         "held" if held(result) else
                         "steady at another airspeed (%s)" % result["status"],
                         result["elevator"], result["pitch_deg"], result["eas_mps"],
                         ("%.1f" % result["aoa_deg"]) if result.get("aoa_deg") is not None
                         else "n/a", result.get("throttle_pct") or 0.0)
            else:
                log.warning("[TRIMMAP] cell %s %s", cell_key(climb, speed), result["status"])
            ckpt_save(run_ts, signature, results)
    except _Aborted:
        log.info("[TRIMMAP] stopped after %d of %d cells - the checkpoint is kept, "
                 "press 'e' again to resume", len(results), total)
    except Exception:                                         # noqa: BLE001
        log.exception("[TRIMMAP] run failed after %d of %d cells - the checkpoint is "
                      "kept", len(results), total)
    finally:
        if owned:
            ctrl.end_measurement_gains()
        try:
            ctrl.autotune_finish()
        except Exception:                                     # noqa: BLE001
            log.exception("[TRIMMAP] hand-back to LOITER failed - check the aircraft")
        log.removeHandler(cap)
        if out is not None:
            complete = len(results) >= total
            out.finish(report_lines(results, climbs, speeds, run_ts,
                                    os.path.basename(out.csv_path), trim_flown,
                                    cap.lines, resumed))
            if complete:
                ckpt_clear()
                log.info("[TRIMMAP] run complete: %d cells", total)
