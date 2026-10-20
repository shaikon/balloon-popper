# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""Frequency-response ID for the roll axis ('k' key).

WHY THIS EXISTS, GIVEN PLANT-ID ALREADY MEASURES THE ROLL AXIS

    Plant-ID returns one number per cell: pan-error rate per degree of bank, at
    DC.  That is a GAIN.  Whether a loop rings is decided by gain AND PHASE at
    the frequency it rings at, and no amount of DC gain data contains phase.

    Worse, phase cannot be recovered from CLOSED-loop step responses either.
    With the balloon chaser loop closed the bank is what it is because of the error and
    the error is what it is because of the bank; regressing one on the other
    returns the loop's own phase relationship dressed as a plant gain.  The
    tell is that such a "gain" MOVES when a controller gain is changed, which a
    plant property cannot do.  So the excitation has to be open-loop and it has
    to sweep frequency.  This module does both.

WHAT IS MEASURED, AND WHAT DELIBERATELY IS NOT

    The roll axis decomposes into two measured links and two exact kinematic
    ones.  Measured, because they are aerodynamics and no closed form exists:

        roll response = roll rate / roll-rate command
        bank->yaw = yaw rate / bank angle.  No rudder is commanded, so
                              yaw here is sideslip on the fin: yaw stiffness,
                              roll-yaw coupling, adverse yaw, dutch roll.

    Not measured, because differentiating the definitions gives them exactly:

        bank_rate = ( cos(roll)cos(pitch)*p + sin(pitch)*r ) / cos(bank)
        pan_rate  = -r + tz*(p*tx + q*ty)/(tx^2 + ty^2)

    The first is checked against the flown data anyway (`kin_err` in the
    report): it is an identity, so a large residual means the sampling or the
    rate source is wrong, not that the physics changed.

    roll-response * kinematics * bank>yaw * kinematics is the whole open loop.  Flying it at
    +pitch and -pitch is what turns "climbs behave differently" into a term.

EXCITATION, AND WHY THE LOOP STAYS CLOSED

    Roll-rate square waves ADDED to the roll law's output, one frequency at
    a time.  The doublet's full override is deliberately not reused: the bank
    axis is open-loop UNSTABLE nose-up, because sin(pitch)*r reinforces the
    bank that produced it, so an overriding excitation diverges inside the
    window meant to measure it - and nose-up is the condition the run exists
    to characterise.  Leaving the bank loop closed holds the aircraft.

    Closing the loop does not reintroduce the problem that makes closed-loop
    STEP data useless, because the excitation is EXTERNAL to the loop.  Every
    signal is fitted at the drive frequency, and at that frequency the only
    common cause is the injection: the loop's own correlations, its ring and
    broadband noise all sit elsewhere and average away over the scored cycles.
    That is the difference between an instrument and a coincidence.  The drive
    frequencies are chosen off the observed ring for the same reason; a point
    that lands on it shows up as low coherence rather than as a quiet lie.

    Square because the command is a RATE and a square wave is what it holds
    cleanly against a 50 Hz sampler; the fit takes the fundamental, so the odd
    harmonics land in bins nothing reads.

    Amplitude scales as 1/T to hold ONE bank amplitude across the sweep, since
    a rate square wave integrates to a triangular bank of rate*T/4.  A fixed
    rate would excite 40 deg of bank at the slow end and 2 deg at the fast end
    and call the result one curve.

    FREQID_SETTLE_CYCLES leading cycles are flown and discarded so the fit sees
    steady state rather than the start transient.

COHERENCE

    Each point carries the fraction of the response variance explained by the
    drive fundamental.  A point below FREQID_MIN_COHERENCE is reported with its
    numbers but excluded from any conclusion: it means something other than the
    excitation moved the aircraft during that window (a gust, the pitch axis
    fighting for the cell, a rate the airframe could not follow), and its phase
    is then meaningless rather than merely noisy.

SIM ONLY until validated.  Open-loop roll excitation at a held steep attitude
has no outer loop watching the aim; the altitude floor and bank abort are the
only guards.
"""

import datetime
import glob
import json
import logging
import math
import os
import time

from . import constants as C
from . import file_export
from .csv_logger import PACKAGE_SHA256
from .controller_autotune_sweep import (
    STRAIGHT_HOLD_S, STRAIGHT_PITCHRATE_TOL_DPS, STRAIGHT_PITCH_TOL_DEG,
    STRAIGHT_ROLL_TOL_DEG, STRAIGHT_TIMEOUT_S, STRAIGHT_YAWRATE_TOL_DPS)
from .testing.synthetic_injector import SyntheticBalloonInjector

log = logging.getLogger(__name__)

# --------------------------------------------------------------------------
# Steep-cell ferry + sampling helpers.  They ferry the aircraft to a fixed
# pitch cell under the normal balloon chaser loop, hold it, sample the
# attitude/rates/flow angles, and keep altitude above the protection floor, all
# keyed off (ctrl, abort) so they carry no state of their own.
# --------------------------------------------------------------------------
SAMPLE_DT = 0.02          # 50 Hz sampling, matches the control loop
WARMUP_S = 0.5            # let the first real balloon chaser tick land after ptest_begin


def _ground_speed(ctrl):
    """Magnitude of the NED ground velocity, or NaN if nav has not fixed."""
    v = getattr(ctrl.link, 'vel_ned', None)
    if v is None:
        return float('nan')
    try:
        return float(math.sqrt(sum(float(c) ** 2 for c in v)))
    except (TypeError, ValueError):
        return float('nan')


def _flow_angles(ctrl):
    """(sideslip, alpha) in degrees from the velocity vector in BODY axes, or
    (NaN, NaN) if nav has not fixed.

    Rotating the NED ground velocity into body axes gives the airflow direction
    the airframe actually sees (in still air), so:
        sideslip = asin(v_body / |V|)      alpha = atan2(w_body, u_body)

    Sideslip is the reason these exist.  No rudder is commanded, so every degree
    of yaw the aircraft develops comes from sideslip acting on the fin - and it
    is the only aerodynamic state never recorded.  It is also the last candidate
    that can carry a dependence on pitch ATTITUDE without routing through the
    velocity magnitude, which is what every force-balance account of the
    measured climb/dive difference has failed on.

    Ground velocity, not airspeed, so this is true only in still air.  With wind
    the angles are contaminated by the wind vector; the magnitudes are still
    comparable BETWEEN cells flown in the same conditions, which is what the
    climb-versus-dive question needs.
    """
    v = getattr(ctrl.link, 'vel_ned', None)
    if v is None:
        return float('nan'), float('nan')
    try:
        vw = [float(c) for c in v]
        sp = math.sqrt(sum(c * c for c in vw))
        if sp < 1.0:                       # stationary: the angles are noise
            return float('nan'), float('nan')
        R = ctrl._body_from_world_R(ctrl.link.attitude)
        vb = [sum(float(R[i][j]) * vw[j] for j in range(3)) for i in range(3)]
        beta = math.degrees(math.asin(max(-1.0, min(1.0, vb[1] / sp))))
        alpha = math.degrees(math.atan2(vb[2], vb[0]))
        return beta, alpha
    except (TypeError, ValueError, AttributeError, IndexError):
        return float('nan'), float('nan')


def _sim_deg(ctrl, attr, idx):
    """One SITL-truth channel in degrees, or NaN where there is no simulator."""
    v = getattr(ctrl.link, attr, None)
    return float('nan') if v is None else math.degrees(float(v[idx]))


def _sample(ctrl):
    _beta, _alpha = _flow_angles(ctrl)
    att = ctrl.link.attitude
    pan, tilt = ctrl.steering_errors()
    pan = float('nan') if pan is None else pan       # the law did not run
    tilt = float('nan') if tilt is None else tilt
    rates = getattr(ctrl.link, 'rates', (0.0, 0.0, 0.0))
    return {
        't': time.monotonic(),
        # The FC's clock for this same attitude.  Under SITL the simulator does
        # not advance in step with wall time, so differencing an angle against
        # 't' while the rates come from the FC mixes two time bases and scales
        # every frequency, time constant and pole by the ratio between them.
        't_fc': float(getattr(ctrl.link, 'attitude_time_boot_s', None) or float('nan')),
        'roll': math.degrees(att[0]),
        # Bank PROXY - the quantity the roll law commands and feeds back.  Recorded so k_p can be fitted in the SAME coordinate the
        # law commands.  Euler roll and the proxy diverge as
        # cos(pitch): the same physical bank needs 3.05 deg of Euler roll at 10
        # deg pitch but 8.80 at 70, and 36.9 at 85, so Euler-frame measurements
        # look increasingly direction-dependent as cells steepen.
        'proxy': math.degrees(math.asin(max(-1.0, min(1.0,
                     math.sin(att[0]) * math.cos(att[1]))))),
        'pitch': math.degrees(att[1]),
        # cos(roll)*cos(pitch): the 'down-ness' of the body z axis.  Negative
        # means inverted.  Needed because the proxy cannot see inversion -
        # sin(roll) is symmetric about 90 deg, so belly-up reads as a small
        # proxy.  Passes through zero at vertical, where inversion is undefined.
        'upright': math.cos(att[0]) * math.cos(att[1]),
        'pan': float(pan),
        'tilt': float(tilt),
        'p': math.degrees(rates[0]),
        'q': math.degrees(rates[1]),
        # Yaw rate.  Half of the bank-rate identity (sin(pitch)*r) and the whole
        # of the yaw-response link the frequency sweep measures, so it is sampled
        # here where the attitude and the gyro come from one tick.
        'r': math.degrees(rates[2]) if len(rates) > 2 else 0.0,
        'spd': float(getattr(ctrl.link, 'airspeed', 0.0)),
        # Total roll-rate command actually committed this tick, law output plus
        # any excitation.  THE FLOWN command, from the FC's STATE frame.
        # Identification must fit against the rate the aircraft got.
        # NaN on a tick the FC did not command.
        'cmd': float(ctrl.link.lua_omega_x_cmd_dps)
               if getattr(ctrl.link, 'lua_omega_x_cmd_dps', None) is not None
               else float('nan'),
        # Throttle as the OPERATOR set it.  Not what the aircraft flew: the
        # airspeed loop owns CH3 when it is enabled, so this is a setting, not a
        # measurement.
        'thr_set': float(getattr(ctrl, 'throttle_pct', float('nan'))),
        # Throttle as COMMANDED, straight off the servo channel.  This is the
        # one that moved the aeroplane.
        'thr_out': float(getattr(ctrl.link, 'servo_ch3', float('nan'))),
        # Speed over the ground from the nav filter.  Reported airspeed is a
        # separate estimate and, with no airspeed sensor, need not track the
        # aircraft at all - so anything that divides by V is only as good as
        # the agreement between these two.
        'gspd': _ground_speed(ctrl),
        'sideslip': _beta,
        'aoa': _alpha,
        # SITL truth beside the estimate, so the kinematic identity can be
        # evaluated against a reference rather than against itself.
        'sim_roll': _sim_deg(ctrl, 'sim_attitude', 0),
        'sim_pitch': _sim_deg(ctrl, 'sim_attitude', 1),
        'sim_p': _sim_deg(ctrl, 'sim_rates', 0),
        'sim_r': _sim_deg(ctrl, 'sim_rates', 2),
    }


def _below_floor(ctrl):
    """True if AGL has fallen below the protection floor.  Missing telemetry ->
    False (cannot floor-check; do not false-trip)."""
    try:
        return float(ctrl.link.relative_alt) < float(ctrl.autotune_protect_floor_m)
    except Exception:
        return False


def _gate_settled(ctrl, abort, timeout_s):
    """Ferry gate: pitch error inside tolerance and body rates quiet, held for
    PLANTID_GATE_HOLD_S.  Returns 'ok' when the window may open, 'floor' if AGL
    dropped below the protection floor while ferrying (caller must climb and
    re-attempt the SAME cell, NOT skip it), or 'timeout' if the gate never held.

    The floor check matters most HERE: ferrying to a nose-down cell is a
    continuous descent, and without this the aircraft would keep diving for the
    whole PLANTID_FERRY_TIMEOUT_S with nothing watching altitude - the exact
    path by which a dive cell that cannot settle flies into the ground."""
    t0 = time.monotonic()
    held = 0.0
    last = t0
    while time.monotonic() - t0 < timeout_s:
        if abort.is_set():
            return 'abort'
        if _below_floor(ctrl):
            return 'floor'
        s = _sample(ctrl)
        ok = (abs(s['tilt']) < C.PLANTID_GATE_PITCH_TOL and
              abs(s['p']) < C.PLANTID_GATE_RATE_TOL and
              abs(s['q']) < C.PLANTID_GATE_RATE_TOL)
        now = time.monotonic()
        held = held + (now - last) if ok else 0.0
        last = now
        if held >= C.PLANTID_GATE_HOLD_S:
            return 'ok'
        time.sleep(SAMPLE_DT)
    return 'timeout'


def _straighten(ctrl, abort):
    """Inter-cell level-out - the SAME mechanism the sweep runs between trials, with
    the SAME tolerances, hold time, timeout and HUD counter (imported from the sweep,
    not copied, so the two can never drift apart).  FBWA lets the flight controller
    self-level the wings; link.level_and_hold VERIFIES straight-and-level from
    telemetry (roll angle + yaw rate + pitch angle + pitch rate) held continuously for
    STRAIGHT_HOLD_S, bounded by STRAIGHT_TIMEOUT_S; ACRO is restored for the cell.
    Operator throttle throughout.  Returns False only on abort / failed ACRO re-entry;
    a level-out TIMEOUT logs and proceeds, exactly as the sweep does.

    WHY the frequency sweep needs this: a cell's excitation leaves the aircraft
    banked and nothing removed that before the next cell.  The ferry gate checks
    tilt error and body RATES but not bank ANGLE, so a steadily banked aircraft
    passed the gate, the next cell's excitation added bank on top, and the window
    died on the bank abort - mis-anchoring the comparison this run exists to make."""
    ctrl.autotune_pause_tracking()
    throttle_frac = max(0.0, min(1.0, float(ctrl.throttle_pct) / 100.0))
    ctrl.straighten_start_t = time.monotonic()     # drives the HUD "Straight" timer
    try:
        achieved = ctrl.link.level_and_hold(
            STRAIGHT_ROLL_TOL_DEG, STRAIGHT_YAWRATE_TOL_DPS, STRAIGHT_HOLD_S,
            STRAIGHT_TIMEOUT_S, throttle_frac=throttle_frac, abort=abort,
            pitch_tol_deg=STRAIGHT_PITCH_TOL_DEG,
            pitch_rate_tol_dps=STRAIGHT_PITCHRATE_TOL_DPS)
    except Exception as e:                                    # noqa: BLE001
        log.warning('[FREQID] level-out failed (%s) - continuing from the current '
                    'attitude.', e)
        achieved = False
    finally:
        ctrl.straighten_start_t = None             # stop/hide the HUD timer
    if not achieved and not abort.is_set():
        log.info('[FREQID] could not hold wings-level+yawless within %.0fs before '
                 'this cell - it starts from a non-level attitude.', STRAIGHT_TIMEOUT_S)
    if abort.is_set():
        return False
    if not ctrl.ptest_enter_acro():
        log.error('[FREQID] ACRO re-entry after the level-out failed - stopping')
        return False
    return True


def _protect_if_below_floor(ctrl, abort):
    """Altitude protection, IDENTICAL in spirit to the sweep's: climb ONLY when
    AGL has fallen below the protection floor, and then up to the climb-to
    altitude.  No pre-emptive / per-cell climbing - a cell is interrupted for a
    climb only if it actually breaches the floor.  Floor and climb-to are the live
    Gain-Tuner knobs (ctrl.autotune_protect_floor_m / ...climb_to_m), read here so
    an edit takes effect on the next check.  Returns False on abort (so the caller
    stops), True otherwise.  A no-op when the link exposes neither relative_alt nor
    climb_to."""
    climb = getattr(ctrl.link, 'climb_to', None)
    if climb is None:
        return not abort.is_set()
    try:
        alt = float(ctrl.link.relative_alt)
    except Exception:
        return not abort.is_set()            # no altitude telemetry -> cannot floor-check
    floor = float(ctrl.autotune_protect_floor_m)
    climb_to = float(ctrl.autotune_protect_climb_to_m)
    if alt >= floor:
        return not abort.is_set()            # above floor -> no climb (symmetric with 't')
    log.info('[FREQID] altitude %.0fm < floor %.0fm - climbing to %.0fm and '
             're-flying.', alt, floor, climb_to)
    # APPEND to the HUD line rather than replace it, so the operator does not
    # lose track of which cell is running - the climb can take a while.  Split
    # on ' | ' first so repeated protections do not stack suffixes.
    _status_base = (getattr(ctrl, 'autotune_status', '') or '').split(' | ')[0]
    ctrl.autotune_status = ((_status_base + ' | ') if _status_base else '') + \
        'Alt protect: climbing to %.0fm' % climb_to
    ctrl.autotune_pause_tracking()
    try:
        climb(climb_to, abort)
    except Exception as e:                    # noqa: BLE001
        log.warning('[FREQID] protection climb failed (%s) - continuing', e)
    if _status_base:                          # climb over: back to the cell line
        ctrl.autotune_status = _status_base
    if abort.is_set():
        return False
    if not ctrl.ptest_enter_acro():
        log.error('[FREQID] ACRO re-entry after climb failed - stopping')
        return False
    return True


# --------------------------------------------------------------------------
# single-frequency fit
# --------------------------------------------------------------------------

def _fit_sinusoid(ts, ys, omega):
    """Amplitude and phase of `ys` at `omega`, plus the fraction of its
    variance that fundamental explains.

    Least squares against [1, cos(wt), sin(wt)] rather than a plain DFT: the
    windows are a whole number of drive cycles but the SAMPLING is not exactly
    uniform (a 50 Hz loop with occasional stale ticks), and the constant column
    absorbs any offset so a drifting mean is not read as amplitude.
    """
    n = len(ts)
    if n < 8:
        return None
    t0 = ts[0]
    c = [math.cos(omega * (t - t0)) for t in ts]
    s = [math.sin(omega * (t - t0)) for t in ts]
    # normal equations for the 3-column design; small and fixed, so written out
    sums = {}
    cols = {'1': [1.0] * n, 'c': c, 's': s}
    for a in ('1', 'c', 's'):
        for b in ('1', 'c', 's'):
            sums[a + b] = sum(cols[a][i] * cols[b][i] for i in range(n))
        sums[a + 'y'] = sum(cols[a][i] * ys[i] for i in range(n))
    m = [[sums['11'], sums['1c'], sums['1s'], sums['1y']],
         [sums['c1'], sums['cc'], sums['cs'], sums['cy']],
         [sums['s1'], sums['sc'], sums['ss'], sums['sy']]]
    # Gaussian elimination with partial pivoting
    for i in range(3):
        piv = max(range(i, 3), key=lambda r: abs(m[r][i]))
        if abs(m[piv][i]) < 1e-12:
            return None
        m[i], m[piv] = m[piv], m[i]
        for r in range(3):
            if r == i:
                continue
            f = m[r][i] / m[i][i]
            for k in range(i, 4):
                m[r][k] -= f * m[i][k]
    coef = [m[i][3] / m[i][i] for i in range(3)]
    dc, a_c, a_s = coef
    amp = math.hypot(a_c, a_s)
    # phase of amp*cos(wt + ph): positive ph = leads the drive
    ph = math.atan2(-a_s, a_c)
    ymean = sum(ys) / n
    ss_tot = sum((y - ymean) ** 2 for y in ys)
    fit = [dc + a_c * c[i] + a_s * s[i] for i in range(n)]
    ss_res = sum((ys[i] - fit[i]) ** 2 for i in range(n))
    coh = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0
    return {'amp': amp, 'phase': ph, 'coh': max(0.0, min(1.0, coh)), 'dc': dc}


def _clock_ratio(scored):
    """FC seconds elapsed per wall second over this window, or None if the FC
    clock is unavailable.  Anything but 1.0 means the two clocks disagree, and
    every rate-like result - frequency, time constant, pole - is scaled by it.

    Least-squares slope rather than an endpoint difference: the FC stamp only
    advances when a new ATTITUDE arrives, so it is a staircase against a
    uniform sampler and its endpoints land wherever the last step happened to
    fall.  The slope uses every sample and is unbiased by that.
    """
    try:
        tw = [s['t'] for s in scored]
        tf = [s['t_fc'] for s in scored]
        if len(tw) < 8 or any(v != v for v in tf):
            return None
        n = len(tw)
        mw, mf = sum(tw) / n, sum(tf) / n
        sxx = sum((t - mw) ** 2 for t in tw)
        if sxx <= 1e-9:
            return None
        r = sum((tw[i] - mw) * (tf[i] - mf) for i in range(n)) / sxx
        return r if 0.05 < r < 20.0 else None
    except (KeyError, IndexError, TypeError):
        return None


# --------------------------------------------------------------------------
# checkpoint / resume
# --------------------------------------------------------------------------
# A six-cell sweep is 12-15 minutes of flying.  Losing it to an abort, a stall
# or a closed window on the last cell is the difference between a procedure an
# operator will run and one they will avoid.  Same design as plant-ID's: save
# after every completed cell, refuse to resume across anything that would make
# the halves incomparable, and NAME the field that changed so a refusal is
# actionable rather than mysterious.
FREQID_CHECKPOINT_VERSION = 1
FREQID_CHECKPOINT_NAME = "incomplete_freq_id"            # + _<run stamp>.json
FREQID_CHECKPOINT_GLOB = "incomplete_freq_id*.json"


def _freqid_signature(ctrl):
    """Everything that would make two half-runs incomparable.

    Stored plainly rather than hashed, so a refusal can say WHICH field moved.
    The excitation rig is in here because a cell flown at a different bank
    amplitude or a different frequency set measured a different thing; so is
    the airspeed command, because bank>yaw scales as 1/V and cells flown at different
    speeds cannot be pooled.
    """
    g = getattr(ctrl, '_gains', None)
    return {
        "fingerprint": PACKAGE_SHA256,
        "cells": [float(c) for c in C.FREQID_PITCH_CELLS_DEG],
        "periods_s": [float(p) for p in C.FREQID_PERIODS_S],
        "cycles": int(C.FREQID_CYCLES),
        "settle_cycles": int(C.FREQID_SETTLE_CYCLES),
        "bank_goal_deg": float(C.FREQID_BANK_GOAL_DEG),
        "rate_max_dps": float(C.FREQID_RATE_MAX_DPS),
        # The excitation is ADDED to the law's output, so the law's own gain is
        # part of the rig: change it and the closed loop the injection rides on
        # is a different one.
        "roll_gain": round(float(getattr(g, 'roll_gain', 0.0)), 3),
        "airspeed_cmd": [int(getattr(g, 'airspeed_cmd_enable', 0)),
                         round(float(getattr(g, 'airspeed_cmd_mps', 0.0)), 2)],
    }


def _ckpt_path(ctrl):
    return os.path.join(os.getcwd(),
                        "%s_%s.json" % (FREQID_CHECKPOINT_NAME,
                                        ctrl.run_stamp_for('k')))


def _ckpt_save(ctrl, results, done_cells):
    """Persist after each completed cell.  Written whole and renamed into
    place, so a crash mid-save cannot leave a half-parsed file behind."""
    path = _ckpt_path(ctrl)
    data = {
        "version": FREQID_CHECKPOINT_VERSION,
        "saved_at": datetime.datetime.now().isoformat(timespec="seconds"),
        "signature": _freqid_signature(ctrl),
        "done_cells": [float(c) for c in done_cells],
        "cells": results.get('cells', []),
        "t_start": results.get('t_start'),
    }
    tmp = path + ".tmp"
    try:
        with open(tmp, 'w') as f:
            json.dump(data, f, indent=1, default=str)
        os.replace(tmp, path)
    except Exception:                                     # noqa: BLE001
        log.exception('[FREQID] could not write the checkpoint - the run '
                      'continues, but a crash from here would lose it')


def _ckpt_load(ctrl):
    """Adopt a matching checkpoint, or None to start fresh.  On acceptance the
    run adopts the checkpoint's stamp so the whole procedure keeps ONE identity
    however many launches it took."""
    found = sorted(glob.glob(os.path.join(os.getcwd(), FREQID_CHECKPOINT_GLOB)))
    if not found:
        return None
    path = found[-1]
    try:
        with open(path) as f:
            data = json.load(f)
    except Exception:                                     # noqa: BLE001
        log.exception('[FREQID] checkpoint %s unreadable; starting fresh', path)
        return None
    if data.get("version") != FREQID_CHECKPOINT_VERSION:
        log.warning('[FREQID] checkpoint version differs; starting fresh.')
        return None
    old, new = data.get("signature") or {}, _freqid_signature(ctrl)
    diff = [k for k in new if old.get(k) != new[k]]
    if diff:
        log.warning('[FREQID] the checkpoint was flown under different '
                    'conditions (%s) - starting FRESH rather than pooling cells '
                    'that measured different plants.  Was %s, now %s.',
                    ', '.join(diff),
                    {k: old.get(k) for k in diff}, {k: new[k] for k in diff})
        return None
    try:
        ctrl.adopt_run_stamp('k', file_export.stamp_from_name(path))
    except Exception:                                     # noqa: BLE001
        log.exception('[FREQID] could not adopt the checkpoint stamp')
        return None
    log.warning('[FREQID] RESUMING the run that started at %s: %d of %d cell(s) '
                'already flown (%s).  The rest are flown now and every file '
                'keeps that original timestamp.',
                file_export.stamp_display(ctrl.run_stamp_for('k')),
                len(data.get("done_cells") or []),
                len(C.FREQID_PITCH_CELLS_DEG),
                ', '.join('%+.0f' % c for c in (data.get("done_cells") or [])))
    return data


def _ckpt_clear(ctrl):
    """Remove the checkpoint - the procedure finished rather than stopped."""
    for p in glob.glob(os.path.join(os.getcwd(), FREQID_CHECKPOINT_GLOB)):
        try:
            os.remove(p)
        except OSError:
            pass


class _LogCapture(logging.Handler):
    """Collects this run's log lines so the report can carry them verbatim.

    A sweep is eight minutes of flying and its failures are operational - a
    ferry that would not gate, a cell that hit the floor, a frequency that
    stopped short.  Those only exist in the console, so a report without them
    tells you a cell is missing but not why, and the console has usually
    scrolled by the time anyone looks.
    """

    def __init__(self):
        logging.Handler.__init__(self)
        self.lines = []

    def emit(self, record):
        try:
            self.lines.append('%s  %s' % (
                time.strftime('%H:%M:%S', time.localtime(record.created)),
                record.getMessage()))
        except Exception:                                     # noqa: BLE001
            pass                                              # never break a run to log


def _wrap_deg(d):
    """Phase into (-180, 180]; a lag of 190 deg is a lag of -170, not a lead."""
    while d <= -180.0:
        d += 360.0
    while d > 180.0:
        d -= 360.0
    return d


# --------------------------------------------------------------------------
# one frequency at one cell
# --------------------------------------------------------------------------

def _run_frequency(ctrl, abort, period_s, rows):
    """Fly FREQID_SETTLE_CYCLES + FREQID_CYCLES of a roll-rate square wave.
    Returns (scored_samples, why_stopped).  'ov' is the injected excitation and
    'cmd' the total rate command the axis received, so the fit reads the roll response's input
    from the same record as its output.
    """
    amp = min(C.FREQID_RATE_MAX_DPS,
              4.0 * C.FREQID_BANK_GOAL_DEG / max(0.05, period_s))
    total = C.FREQID_SETTLE_CYCLES + C.FREQID_CYCLES
    scored_from = C.FREQID_SETTLE_CYCLES * period_s
    t_start = time.monotonic()
    scored = []
    while True:
        el = time.monotonic() - t_start
        if el >= total * period_s:
            return scored, ''
        if abort.is_set():
            return scored, 'abort'
        # square wave: + for the first half of each cycle, - for the second
        rate = amp if (el % period_s) < 0.5 * period_s else -amp
        ctrl.freqid_rate_inject(rate)
        s = _sample(ctrl)
        s['ov'] = rate
        s['period'] = period_s
        rows.append(s)
        if abs(s['proxy']) > C.FREQID_BANK_ABORT_DEG:
            return scored, 'bank %.0f deg' % abs(s['proxy'])
        if _below_floor(ctrl):
            return scored, 'floor'
        if el >= scored_from:
            scored.append(s)
        time.sleep(SAMPLE_DT)


def _analyse(scored, period_s):
    """Gain and phase of each link at this drive frequency."""
    omega = 2.0 * math.pi / period_s
    # FC clock where it exists.  The drive is generated on wall time, so its
    # period in FC seconds is period_s * clock_ratio and the frequency the
    # AIRCRAFT saw is omega / clock_ratio; fitting on FC time puts every result
    # on the axis the aircraft actually flew.
    # The wall clock RESCALED, not the FC stamp itself.  The stamp is a
    # staircase at the FC's message rate, so differencing it divides by zero on
    # every repeat; the wall clock is uniform and one scalar puts it on the FC's
    # axis.  The drive is generated on wall time, so its period in FC seconds is
    # period_s * ratio and the frequency the AIRCRAFT saw is omega / ratio.
    ratio = _clock_ratio(scored)
    t0 = scored[0]['t']
    if ratio is not None:
        ts = [(s['t'] - t0) * ratio for s in scored]
        omega = omega / ratio
    else:
        ts = [s['t'] - t0 for s in scored]
    fits = {}
    for name, key in (('inj', 'ov'), ('cmd', 'cmd'), ('p', 'p'),
                      ('bank', 'proxy'), ('r', 'r')):
        fits[name] = _fit_sinusoid(ts, [s[key] for s in scored], omega)
    if any(f is None for f in fits.values()):
        return None
    out = {'period_s': period_s, 'omega': omega, 'n': len(scored),
           'clock_ratio': ratio}
    for name, f in fits.items():
        out['amp_' + name] = f['amp']
        out['coh_' + name] = f['coh']
    # Roll response = roll rate / roll-rate command, both at the drive frequency.  No square-wave
    # correction: the fit already returns the FUNDAMENTAL of a square (an
    # amplitude-A square fits as 4A/pi), so this is fundamental over
    # fundamental and correcting again would report every roll-response magnitude 27% low
    # while leaving the phase right - an error that survives a sanity check.
    drive = fits['cmd']['amp']
    out['roll_resp_gain'] = fits['p']['amp'] / drive if drive > 1e-6 else None
    out['roll_resp_phase_deg'] = _wrap_deg(math.degrees(fits['p']['phase']
                                                 - fits['cmd']['phase']))
    # bank>yaw = r / bank.  Both are responses, so no square-wave correction.
    b = fits['bank']['amp']
    out['bank_yaw_gain'] = fits['r']['amp'] / b if b > 1e-6 else None
    out['bank_yaw_phase_deg'] = _wrap_deg(math.degrees(fits['r']['phase']
                                                 - fits['bank']['phase']))
    # Coherence against the INJECTION, not against the total command: what
    # matters is how much of the response this run put there.  A square drive
    # carries harmonics, so even a perfect single-frequency response cannot
    # explain all the variance; the figure is a floor on trustworthiness, not
    # a fit quality.
    out['amp_inj'] = fits['inj']['amp']
    out['coh'] = min(fits['p']['coh'], fits['bank']['coh'])
    # OFF-FREQUENCY PROBE.  Coherence alone cannot separate "something else was
    # moving the aircraft" from "a square drive has harmonics", and the second
    # is by design.  So the bank is also fitted at 1.4x the drive - not a
    # harmonic of it, and nothing should live there.  A large ratio means the
    # window caught the loop's own ring or a gust, which is the one failure
    # that makes a phase reading quietly wrong rather than obviously noisy.
    off = _fit_sinusoid(ts, [s['proxy'] for s in scored], 1.4 * omega)
    out['off_frac'] = (off['amp'] / fits['bank']['amp']
                       if (off and fits['bank']['amp'] > 1e-6) else None)
    # Identity check.  bank_rate = (upright*p + sin(pitch)*r)/cos(bank) is a
    # definition, so the two sides agreeing is a statement about the DATA, not
    # about the aircraft: a large residual means the attitude and the gyro are
    # not the same instant, or the sampler dropped ticks.
    # The identity as a SLOPE, not a residual.  Scatter is expected - the
    # attitude updates slower than the sampler, so a point-wise derivative is
    # noisy no matter what - but scatter leaves the slope at 1.  A slope away
    # from 1 means a scale error, and the only two candidates are a mis-scaled
    # signal or a mismatched time base.  Reporting the residual instead is what
    # let a 1.3x clock error read as plausible noise for the whole project.
    sxy = sxx = 0.0
    mo = mp = 0.0
    obs_l, pred_l = [], []
    for i in range(1, len(scored) - 1):
        dt = ts[i + 1] - ts[i - 1]
        if dt <= 1e-9:
            continue
        obs_l.append((scored[i + 1]['proxy'] - scored[i - 1]['proxy']) / dt)
        th = math.radians(scored[i]['pitch'])
        cb = math.cos(math.radians(scored[i]['proxy'])) or 1e-3
        pred_l.append((scored[i]['upright'] * scored[i]['p']
                       + math.sin(th) * scored[i]['r']) / cb)
    if len(obs_l) > 8:
        mo = sum(obs_l) / len(obs_l)
        mp = sum(pred_l) / len(pred_l)
        for i in range(len(obs_l)):
            sxy += (pred_l[i] - mp) * (obs_l[i] - mo)
            sxx += (pred_l[i] - mp) ** 2
        out['kin_slope'] = sxy / sxx if sxx > 1e-9 else None
        rr = sum((obs_l[i] - pred_l[i]) ** 2 for i in range(len(obs_l)))
        rf = sum(o * o for o in obs_l)
        out['kin_err'] = math.sqrt(rr / rf) if rf > 1e-9 else None
    else:
        out['kin_slope'] = None
        out['kin_err'] = None
    return out


# --------------------------------------------------------------------------
# output
# --------------------------------------------------------------------------

def _dump_rows(cell_deg, rows, run_ts):
    tag = str(int(round(cell_deg))).replace('-', 'm')
    name = 'freq_id_cell_%s_%s.csv' % (tag, run_ts)
    try:
        with open(name, 'w') as f:
            f.write('# code_sha256: %s\n' % PACKAGE_SHA256)
            f.write('t,t_fc,period,ov,cmd,roll,pitch,proxy,upright,p,q,r,pan,'
                    'tilt,spd,gspd,sideslip,aoa,thr_set,thr_out,sim_roll,'
                    'sim_pitch,sim_p,sim_r\n')
            for r in rows:
                f.write('%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.4f,%.2f,'
                        '%.2f,%.2f,%.3f,%.3f,%.2f,%.2f,%.3f,%.3f,%.1f,%.0f,'
                        '%.2f,%.2f,%.2f,%.2f\n' % (
                            r['t'], r.get('t_fc', float('nan')),
                            r.get('period', 0.0), r.get('ov', 0.0),
                            r.get('cmd', 0.0), r['roll'], r['pitch'],
                            r['proxy'], r['upright'], r['p'], r['q'],
                            r.get('r', 0.0), r['pan'], r['tilt'], r['spd'],
                            r.get('gspd', float('nan')),
                            r.get('sideslip', float('nan')),
                            r.get('aoa', float('nan')),
                            r.get('thr_set', float('nan')),
                            r.get('thr_out', float('nan')),
                            r.get('sim_roll', float('nan')),
                            r.get('sim_pitch', float('nan')),
                            r.get('sim_p', float('nan')),
                            r.get('sim_r', float('nan'))))
    except Exception as e:                                    # noqa: BLE001
        log.warning('[FREQID] could not write %s: %s', name, e)
        return
    # Each cell CSV carries the run stamp and is a COMPLETED-RUN output: it is
    # KEPT, not claimed away, so a rerun's set sits beside the previous run's
    # (the 'freqid_cells' export pick='run' copies only the newest run's whole
    # set).  Only the resume checkpoint is cleaned.


def _report(results, run_ts, lines=None):
    name = 'freq_id_report_%s.txt' % run_ts
    L = []
    L.append('FREQUENCY-RESPONSE ID - roll axis')
    L.append('code_sha256: %s' % PACKAGE_SHA256)
    L.append('')
    L.append('ROLLRESP = roll rate / roll-rate COMMAND.  FC rate loop, servo'
             ' and airframe')
    L.append('           roll response, lumped.  Symmetric climb vs dive, so it'
             ' carries no')
    L.append('           asymmetry - but it carries the delay that caps'
             ' everything else.')
    L.append('BANK>YAW = yaw rate / bank angle, the bank-to-turn conversion.'
             '  No rudder')
    L.append('           is commanded, so this is sideslip on the fin, and'
             ' it is where the')
    L.append('           climb/dive asymmetry lives.')
    L.append('coh     = variance the drive fundamental explains. A square drive'
             ' has')
    L.append('          harmonics, so this never reaches 1 and is a floor, not a'
             ' fit.')
    L.append('off     = bank amplitude at 1.4x the drive, where nothing should'
             ' live.')
    L.append('          Large means the window caught the loop ring or a gust,'
             ' which')
    L.append('          is what makes a phase reading wrong rather than noisy.')
    L.append('          A clean square-driven window sits near 0.15; suspect'
             ' above 0.4.')
    L.append('kin_sl  = SLOPE of the exact bank-rate identity, 1.00 = closes.'
             '  It is a')
    L.append('          definition, so anything else indicts the data. Scatter'
             ' from the')
    L.append('          attitude updating slower than the sampler leaves the'
             ' slope at 1;')
    L.append('          a slope away from 1 is a scale error or a mismatched'
             ' clock.')
    L.append('')
    for cell in results['cells']:
        L.append('-' * 72)
        rr = [p.get('clock_ratio') for p in cell['points']
              if p.get('clock_ratio')]
        cr = sum(rr) / len(rr) if rr else None
        L.append('CELL %+.0f deg   flew %+.1f   V %.1f m/s' % (
            cell['cell_deg'], cell.get('pitch_flown', float('nan')),
            cell.get('v_med', float('nan'))))
        if cr is None:
            L.append('  CLOCK: no FC timestamp - results are on WALL time and'
                     ' any disagreement')
            L.append('         between the two clocks is folded into every'
                     ' frequency below.')
        else:
            L.append('  CLOCK: the FC advances %.3f s per wall second.'
                     '  Frequencies below are' % cr)
            L.append('         on the FC axis, i.e. what the AIRCRAFT flew.')
            if abs(cr - 1.0) > 0.02:
                L.append('         Wall-clock analysis of this run would'
                         ' misreport every rate,')
                L.append('         frequency and time constant by %.2fx.'
                         % (1.0 / cr))
        L.append('%8s %8s %9s %9s %9s %9s %7s %7s %8s' % (
            'w rad/s', 'period', 'rollresp', 'phase', 'bank>yaw', 'phase',
            'coh', 'off', 'kin_sl'))
        for pt in cell['points']:
            flag = '' if pt.get('coh', 0) >= C.FREQID_MIN_COHERENCE else '  LOW COH'
            L.append('%8.2f %8.2f %9s %9s %9s %9s %7.2f %7s %8s%s' % (
                pt['omega'], pt['period_s'],
                '%.3f' % pt['roll_resp_gain'] if pt.get('roll_resp_gain') else '-',
                '%+.0f' % pt['roll_resp_phase_deg'] if pt.get('roll_resp_phase_deg') is not None else '-',
                '%.3f' % pt['bank_yaw_gain'] if pt.get('bank_yaw_gain') else '-',
                '%+.0f' % pt['bank_yaw_phase_deg'] if pt.get('bank_yaw_phase_deg') is not None else '-',
                pt.get('coh', 0.0),
                '%.2f' % pt['off_frac'] if pt.get('off_frac') is not None else '-',
                '%.2f' % pt['kin_slope'] if pt.get('kin_slope') is not None else '-',
                flag))
        L.append('')
    # ---- across-cell summary: the two numbers the run exists to test ----
    # PER-CELL VERDICT.  A point below FREQID_MIN_COHERENCE means something
    # other than the excitation moved the aircraft in that window, so its phase
    # is meaningless rather than merely noisy.  One such point is survivable;
    # a cell that cannot muster FREQID_MIN_GOOD_POINTS of them measured nothing
    # and must not be pooled with cells that did.  Flagging the point and using
    # the cell anyway is what lets a bad cell set a coefficient.
    unmeasured = []
    for c in results['cells']:
        n_ok = sum(1 for p in c['points']
                   if p.get('coh', 0.0) >= C.FREQID_MIN_COHERENCE)
        c['n_good_points'] = n_ok
        c['measurable'] = n_ok >= C.FREQID_MIN_GOOD_POINTS
        if not c['measurable']:
            unmeasured.append((c['cell_deg'], n_ok, len(c['points'])))
    if unmeasured:
        L.append('=' * 72)
        L.append('UNMEASURED CELLS - excluded from every table below')
        for cd, n_ok, n in unmeasured:
            L.append('  cell %+6.1f : only %d of %d points reached coherence'
                     ' %.2f' % (cd, n_ok, n, C.FREQID_MIN_COHERENCE))
        L.append('')
        L.append('A cell needs %d.  Fly it again, or lower the excitation'
                 ' frequency so the' % C.FREQID_MIN_GOOD_POINTS)
        L.append('closed loop cancels less of the injection.')
        L.append('')

    rows = []
    for c in results['cells']:
        if not c['points'] or not c.get('measurable', True):
            continue
        th = math.radians(c.get('pitch_flown', 0.0))
        p0 = c['points'][0]
        g2 = p0.get('bank_yaw_gain')
        if g2 is None:
            continue
        ct = math.cos(th)
        # Ground speed in preference to reported airspeed: bank>yaw * V only means
        # something if V is what the aircraft was actually doing.
        v = c.get('gspd_med', float('nan'))
        if not (v == v and v > 1.0):
            v = c.get('v_med', float('nan'))
        rows.append({
            'v_rep': c.get('v_med', float('nan')),
            'ss': c.get('sideslip_p90', float('nan')),
            'aoa': c.get('aoa_med', float('nan')),
            'pitch': math.degrees(th), 'g2': g2, 'v': v, 'g2v': g2 * v,
            'thr': c.get('thr_med', float('nan')),
            # Kept only as a diagnostic.  It was introduced on the assumption
            # that bank>yaw carries a cos(pitch) factor, and the matched-pitch
            # Measurement shows otherwise: the raw value varies only
            # 27% across +/-56 deg, and dividing by cos(pitch) widens the spread
            # to 94%.  Do NOT read a constant value here as evidence of one
            # airframe - that reasoning once explained away a real asymmetry.
            'g2n': g2 / ct if abs(ct) > 1e-6 else None,
            # The destabilising pole, sin(pitch) * bank>yaw.  It grows
            # MONOTONICALLY with |pitch| as far as this has been flown (+0.24,
            # +0.46, +0.63 at 20/40/55 deg): a steeper climb is a worse one.
            # An earlier prediction that it would peak near 45 deg and ease off
            # rested on the cos(pitch) assumption above and was wrong.
            'pole': math.sin(th) * g2,
            'g1': p0.get('roll_resp_gain')})
    if rows:
        L.append('=' * 72)
        L.append('ACROSS CELLS')
        L.append('%9s %8s %7s %8s %9s %11s %8s %7s' % (
            'pitch', 'gamma', 'V grnd', 'bank>yaw', 'x V grnd', 'plant pole',
            'sideslip', 'thr PWM'))
        for r in sorted(rows, key=lambda r: r['pitch']):
            gam = (r['pitch'] - r['aoa']) if r['aoa'] == r['aoa'] else float('nan')
            L.append('%9.1f %8s %7.1f %8.3f %9.2f %11s %8s %7s' % (
                r['pitch'], '%+.1f' % gam if gam == gam else '-', r['v'],
                r['g2'], r['g2v'],
                '%+.2f %s' % (r['pole'], 'DIV' if r['pole'] > 0 else '   '),
                '%.2f' % r['ss'] if r['ss'] == r['ss'] else '-',
                '%.0f' % r['thr'] if r['thr'] == r['thr'] else '-'))
        # WHICH quantity the pairs were matched on.  Pitch and flight path
        # differ by the angle of attack, which keeps its sign in both
        # directions, so matching one mismatches the other by ~2*|AoA|.  A
        # climb-vs-dive number is not interpretable without knowing which.
        gp = [r['pitch'] - r['aoa'] for r in rows
              if r['aoa'] == r['aoa'] and r['pitch'] > 0]
        gn = [r['pitch'] - r['aoa'] for r in rows
              if r['aoa'] == r['aoa'] and r['pitch'] < 0]
        pp = [r['pitch'] for r in rows if r['pitch'] > 0]
        pn = [r['pitch'] for r in rows if r['pitch'] < 0]
        if gp and gn and pp and pn:
            dg = abs(sum(abs(x) for x in gp) / len(gp)
                     - sum(abs(x) for x in gn) / len(gn))
            dp = abs(sum(abs(x) for x in pp) / len(pp)
                     - sum(abs(x) for x in pn) / len(pn))
            L.append('')
            L.append('MATCHED ON: %s.  |pitch| differs by %.1f deg between the'
                     ' climb and dive'
                     % ('FLIGHT PATH' if dg < dp else 'PITCH ATTITUDE', dp))
            L.append('halves, |flight path| by %.1f deg.  They cannot both be'
                     ' zero - the angle of' % dg)
            L.append('attack keeps its sign in both directions - so any'
                     ' asymmetry below belongs to')
            L.append('whichever of the two is matched here.')
        L.append('')
        L.append('sideslip is the 90th-percentile MAGNITUDE over the cell -'
                 ' how much sideslip')
        L.append('an 8 deg bank buys.  With no rudder it is the entire source'
                 ' of yaw, and it')
        L.append('is the last state that could carry a pitch-ATTITUDE'
                 ' dependence without')
        L.append('routing through the velocity vector - which every attempted'
                 ' derivation of')
        L.append('the odd term has failed on.  If it is larger climbing than'
                 ' diving at')
        L.append('matched |pitch|, that is the mechanism.  If it is symmetric,'
                 ' the effect is')
        L.append('kinematic and the fin is not the answer.')
        bad = [r for r in rows
               if r['v'] == r['v'] and r['v_rep'] == r['v_rep']
               and abs(r['v'] - r['v_rep']) > 0.5]
        if bad:
            L.append('')
            L.append('WARNING: ground speed and reported airspeed disagree by'
                     ' more than 0.5 m/s')
            L.append('on %d cell(s).  Every result that divides by V is'
                     ' suspect until that is' % len(bad))
            L.append('resolved, and a V that does not move with flight'
                     ' condition is not a')
            L.append('measurement.')
        L.append('')
        L.append('bank>yaw x V is the diagnostic.  If the yaw rate came only from'
                 ' bank-to-turn, it')
        L.append('would equal g = 9.81 at every cell, both signs, every speed.'
                 '  An excess is')
        L.append('yaw the turn kinematics do not account for, and throttle is'
                 ' the first')
        L.append('suspect because thrust tracks the SIGN of the flight path'
                 ' while attitude')
        L.append('and airspeed do not.')
        L.append('')
        L.append('The pole is sin(pitch) * bank>yaw(0) exactly.  It grows with |pitch|'
                 ' as far as this')
        L.append('run reaches, so a steeper climb is a worse one - there is no'
                 ' turnover.')
        L.append('')

    # ---- anchor repeat: did the aircraft drift across the run? ----
    ar = results.get('anchor_repeat')
    if ar and ar.get('points') and results.get('cells'):
        first = next((c for c in results['cells']
                      if abs(c['cell_deg'] - ar['cell_deg']) < 1e-6), None)
        if first and first.get('points'):
            a0, b0 = first['points'][0], ar['points'][0]
            L.append('=' * 72)
            L.append('ANCHOR REPEAT - cell %+.0f flown first and again last'
                     % ar['cell_deg'])
            L.append('%10s %12s %12s %10s' % ('', 'first', 'last', 'drift'))
            for nm, k in (('bank>yaw', 'bank_yaw_gain'),
                          ('rollresp', 'roll_resp_gain')):
                if a0.get(k) and b0.get(k):
                    L.append('%10s %12.3f %12.3f %9.0f%%'
                             % (nm, a0[k], b0[k], 100 * (b0[k] / a0[k] - 1)))
            L.append('%10s %12.1f %12.1f %9.1f' % (
                'pitch', first.get('pitch_flown', float('nan')),
                ar.get('pitch_flown', float('nan')),
                ar.get('pitch_flown', 0) - first.get('pitch_flown', 0)))
            L.append('%10s %12.1f %12.1f %9.1f' % (
                'V', first.get('v_med', float('nan')),
                ar.get('v_med', float('nan')),
                ar.get('v_med', 0) - first.get('v_med', 0)))
            _d = (abs(b0['bank_yaw_gain'] / a0['bank_yaw_gain'] - 1)
                  if a0.get('bank_yaw_gain') and b0.get('bank_yaw_gain') else 0.0)
            L.append('')
            if _d > 0.10:
                L.append('DRIFT: the same cell measured %.0f%% differently at the'
                         ' end of the run.' % (100 * _d))
                L.append('Every cross-cell comparison below assumes the aircraft'
                         ' did not change,')
                L.append('and this says it did.  Treat the asymmetry as no better'
                         ' than that figure.')
            else:
                L.append('Repeatable to %.0f%% - cross-cell comparisons in this'
                         ' run are sound.' % (100 * _d))
            L.append('')

    # ---- matched pairs, dive against climb at the same |pitch| ----
    pos = [c for c in results['cells']
           if c.get('pitch_flown', 0) > 0 and c['points']
           and c.get('measurable', True)]
    neg = [c for c in results['cells']
           if c.get('pitch_flown', 0) < 0 and c['points']
           and c.get('measurable', True)]
    for pc in sorted(pos, key=lambda c: c['pitch_flown']):
        # pair on |pitch| FLOWN, not on the cell that was asked for: the whole
        # reason these cells are pre-compensated is that the two differ.
        nc = min(neg, key=lambda c: abs(abs(c['pitch_flown'])
                                        - abs(pc['pitch_flown'])), default=None)
        if nc is None:
            continue
        L.append('=' * 72)
        L.append('CLIMB %+.1f vs DIVE %+.1f  (mismatch %.1f deg, cos factor'
                 ' %.3f)' % (
                     pc['pitch_flown'], nc['pitch_flown'],
                     abs(pc['pitch_flown']) - abs(nc['pitch_flown']),
                     math.cos(math.radians(pc['pitch_flown']))
                     / max(1e-6, math.cos(math.radians(nc['pitch_flown'])))))
        L.append('%8s %8s %11s %11s %11s %11s' % (
            'period', 'w climb', 'rollresp +/-', 'phase diff', 'bank>yaw +/-',
            'phase diff'))
        for pu in pc['points']:
            # Matched on the COMMANDED period, not on omega: each window carries
            # its own clock ratio, so the cells do not land on identical
            # frequencies even though they flew identical excitations.
            pd_ = next((x for x in nc['points']
                        if abs(x['period_s'] - pu['period_s']) < 1e-6), None)
            if pd_ is None:
                continue

            def _rat(a, b):
                return '%.2f' % (a / b) if (a and b) else '-'

            def _dif(a, b):
                return ('%+.0f' % _wrap_deg(a - b)
                        if (a is not None and b is not None) else '-')

            L.append('%8.2f %8.2f %11s %11s %11s %11s' % (
                pu['period_s'], pu['omega'],
                _rat(pu.get('roll_resp_gain'), pd_.get('roll_resp_gain')),
                _dif(pu.get('roll_resp_phase_deg'), pd_.get('roll_resp_phase_deg')),
                _rat(pu.get('bank_yaw_gain'), pd_.get('bank_yaw_gain')),
                _dif(pu.get('bank_yaw_phase_deg'), pd_.get('bank_yaw_phase_deg'))))
        L.append('')
    L.append('A ratio near 1 and a phase difference near 0 mean that link is')
    L.append('the SAME in both directions, and the climb/dive difference is')
    L.append('fully accounted for by the kinematic sin(pitch) term. Anything')
    L.append('else is airframe behaviour the kinematics do not contain.')
    if lines:
        L.append('=' * 72)
        L.append('RUN TRANSCRIPT - everything the console printed, so a cell')
        L.append('that is missing above can be explained from this file alone.')
        L.append('')
        L.extend(lines)
    try:
        with open(name, 'w') as f:
            f.write('\n'.join(L) + '\n')
    except Exception as e:                                    # noqa: BLE001
        log.warning('[FREQID] could not write %s: %s', name, e)
        return
    # A completed run's report is KEPT, not claimed away: timestamped, so runs
    # accumulate rather than collide, cleared by Delete when the operator
    # chooses.  Only the resume checkpoint is cleaned.
    for ln in L:
        log.info('[FREQID] %s', ln)


# --------------------------------------------------------------------------
# entry point
# --------------------------------------------------------------------------

def run_freq_id(ctrl, abort):
    _cap = _LogCapture()                    # transcript for the report file
    log.addHandler(_cap)
    ckpt = _ckpt_load(ctrl)
    run_ts = ctrl.run_stamp_for('k')
    done_cells = [float(c) for c in (ckpt or {}).get('done_cells', [])]
    results = {'fingerprint': PACKAGE_SHA256,
               't_start': (ckpt or {}).get('t_start') or time.time(),
               'cells': list((ckpt or {}).get('cells') or []),
               'periods_s': list(C.FREQID_PERIODS_S),
               'bank_goal_deg': C.FREQID_BANK_GOAL_DEG}
    _bank_lim_saved = ctrl._gains.bank_lim_deg
    try:
        ctrl.begin_measurement_gains(
            'freq-ID',
            bank_lim_deg=float(C.PLANTID_FERRY_BANK_LIM_DEG))
    except Exception:                                         # noqa: BLE001
        log.exception('[FREQID] could not take ownership of the measurement gains')
    try:
        log.info('[FREQID] start: cells %s deg, periods %s s, %d scored cycles '
                 'each (+%d discarded), %.0f deg bank amplitude. Abort with k.',
                 C.FREQID_PITCH_CELLS_DEG, C.FREQID_PERIODS_S, C.FREQID_CYCLES,
                 C.FREQID_SETTLE_CYCLES, C.FREQID_BANK_GOAL_DEG)
        ctrl.autotune_status = 'Freq ID: starting'
        if not ctrl.ptest_enter_acro():
            log.error('[FREQID] could not enter ACRO - aborting')
            return
        # The first cell is flown again at the END as an anchor repeat.  A
        # sweep is long enough that the aircraft, the trim or the host clock can
        # drift across it, and every cross-cell comparison assumes they did not.
        # Re-flying one cell is the only way to find out.
        _plan = list(C.FREQID_PITCH_CELLS_DEG)
        if len(_plan) > 1:
            _plan = _plan + [_plan[0]]
        for _idx, cell in enumerate(_plan):
            if abort.is_set():
                break
            _is_anchor = (_idx == len(_plan) - 1 and len(_plan) > 1)
            if not _is_anchor and float(cell) in done_cells:
                log.info('[FREQID] cell %+.0f already flown - skipping', cell)
                continue
            ctrl.autotune_status = ('Freq ID: anchor repeat %+.0f' % cell
                                    if _is_anchor
                                    else 'Freq ID: cell %+.0f' % cell)
            if not _straighten(ctrl, abort):
                break
            _protect_if_below_floor(ctrl, abort)
            # force_direction: a descending ray hits the ground and gets range-
            # capped while a climbing one does not, which would anchor the two
            # halves of the pair differently and put a rig artefact on exactly
            # the comparison this run exists to make.
            inj = SyntheticBalloonInjector(
                ctrl.link, ctrl.sensor, 0.0, float(cell),
                force_direction=True)
            ctrl.set_point_injector(inj)
            if not inj.start():
                log.warning('[FREQID] cell %+.0f: injector precondition failed '
                            '- skipped', cell)
                continue
            ctrl.ptest_begin(0.0, float(cell))
            ctrl.sensor.set_gimbal_locked(True)
            time.sleep(WARMUP_S)
            gate = _gate_settled(ctrl, abort, C.PLANTID_FERRY_TIMEOUT_S)
            if gate == 'abort':
                break
            if gate != 'ok':
                log.warning('[FREQID] cell %+.0f: ferry gate %s - skipped',
                            cell, gate)
                continue
            rows, points = [], []
            for period in C.FREQID_PERIODS_S:
                if abort.is_set():
                    break
                ctrl.autotune_status = ('Freq ID: cell %+.0f, %.1fs'
                                        % (cell, period))
                # A floor breach interrupts the frequency, it does not abandon
                # the cell: climb back and re-fly it once.  Abandoning left the
                # aircraft low AND lost the remaining frequencies, which is the
                # worst of both.  One retry only - a cell that breaches twice is
                # being flown too low to measure, and saying so beats grinding.
                scored, why = _run_frequency(ctrl, abort, period, rows)
                ctrl.freqid_rate_inject(0.0)
                if why == 'floor' and not abort.is_set():
                    log.warning('[FREQID] cell %+.0f period %.1fs hit the '
                                'altitude floor - climbing and re-flying it',
                                cell, period)
                    if not _protect_if_below_floor(ctrl, abort):
                        break
                    if _gate_settled(ctrl, abort,
                                     C.PLANTID_FERRY_TIMEOUT_S) == 'ok':
                        scored, why = _run_frequency(ctrl, abort, period, rows)
                        ctrl.freqid_rate_inject(0.0)
                if why:
                    log.warning('[FREQID] cell %+.0f period %.1fs stopped: %s',
                                cell, period, why)
                    if why in ('abort', 'floor'):
                        break
                pt = _analyse(scored, period) if len(scored) >= 8 else None
                if pt is None:
                    log.warning('[FREQID] cell %+.0f period %.1fs: not fittable '
                                '(%d scored samples)', cell, period, len(scored))
                    continue
                points.append(pt)
                log.info('[FREQID] cell %+.0f  w=%.2f  rollresp %.3f @ %+.0f deg  '
                         'bank>yaw %.3f @ %+.0f deg  coh %.2f',
                         cell, pt['omega'], pt['roll_resp_gain'] or 0.0,
                         pt['roll_resp_phase_deg'], pt['bank_yaw_gain'] or 0.0,
                         pt['bank_yaw_phase_deg'], pt['coh'])
                # let the cell re-settle before the next frequency
                if not _gate_settled(ctrl, abort, C.PLANTID_FERRY_TIMEOUT_S) == 'ok':
                    log.info('[FREQID] cell %+.0f: did not re-settle after '
                             '%.1fs - remaining frequencies may be degraded',
                             cell, period)
            ctrl.freqid_rate_inject(0.0)
            if not _is_anchor:
                _dump_rows(cell, rows, run_ts)
            pitches = [r['pitch'] for r in rows] or [float('nan')]
            spds = [r['spd'] for r in rows] or [float('nan')]
            thrs = [r.get('thr_out', float('nan')) for r in rows]
            thrs = [x for x in thrs if x == x] or [float('nan')]
            gs = [r.get('gspd', float('nan')) for r in rows]
            gs = [x for x in gs if x == x] or [float('nan')]
            # Sideslip is signed and swings with the excitation, so the median
            # would sit near zero and say nothing.  What matters is how MUCH
            # sideslip a given bank buys, so take the amplitude.
            ss = sorted(abs(r.get('sideslip', float('nan'))) for r in rows
                        if r.get('sideslip', float('nan')) ==
                        r.get('sideslip', float('nan'))) or [float('nan')]
            aoa = sorted(r.get('aoa', float('nan')) for r in rows
                         if r.get('aoa', float('nan')) ==
                         r.get('aoa', float('nan'))) or [float('nan')]
            _entry = {
                'cell_deg': float(cell), 'points': points,
                'anchor_repeat': bool(_is_anchor),
                'thr_med': sorted(thrs)[len(thrs) // 2],
                'gspd_med': sorted(gs)[len(gs) // 2],
                'sideslip_p90': ss[int(0.90 * (len(ss) - 1))],
                'aoa_med': aoa[len(aoa) // 2],
                'pitch_flown': sorted(pitches)[len(pitches) // 2],
                'v_med': sorted(spds)[len(spds) // 2],
                'n_rows': len(rows)}
            if _is_anchor:
                results['anchor_repeat'] = _entry
            else:
                results['cells'].append(_entry)
                done_cells.append(float(cell))
                _ckpt_save(ctrl, results, done_cells)
        if not abort.is_set():
            _ckpt_clear(ctrl)
        else:
            log.warning('[FREQID] aborted with %d of %d cell(s) flown - the '
                        'checkpoint is kept, press k again to resume',
                        len(done_cells), len(C.FREQID_PITCH_CELLS_DEG))
        _report(results, run_ts, _cap.lines)
        name = 'freq_id_results_%s.json' % run_ts
        try:
            with open(name, 'w') as f:
                json.dump(results, f, indent=1)
            # Completed-run output: KEPT, not claimed away (timestamped, so
            # runs accumulate).  Only the resume checkpoint is cleaned.
        except Exception as e:                                # noqa: BLE001
            log.warning('[FREQID] could not write %s: %s', name, e)
    finally:
        log.removeHandler(_cap)
        try:
            ctrl.freqid_rate_inject(0.0)
        except Exception:                                     # noqa: BLE001
            pass
        try:
            ctrl.end_measurement_gains()
        except Exception:                                     # noqa: BLE001
            log.exception('[FREQID] could not release the measurement gains')
        ctrl.autotune_status = ''
        ctrl._freqid_active = False
        ctrl._reset_to_idle()
        log.info('[FREQID] done (bank limit back to %.1f)', _bank_lim_saved)
