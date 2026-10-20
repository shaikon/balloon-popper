# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail).
"""Stage-A open-loop plant identification ('i' key, in flight) - pitch, yaw, roll.

The 'i' key runs a unified plant-ID: the same open-loop MANUAL doublet
measurement on each of the three axes in turn (pitch elevator, yaw rudder, roll
aileron), gated by the Gain-Tuner Do Pitch / Do Yaw / Do Roll switches and
sequenced by run_plant_id() at the foot of this file.  Every axis uses the one
short-period estimator below (fit_pitch_plant, axis-agnostic).  All three axes
share ArduPlane's AC_PID rate loop (AP_FW_Controller for pitch/roll, the
math-identical AP_YawController::get_rate_out for yaw), so each suggests the FULL
ArduPlane rate PID (<PREFIX>_RATE_{FF,P,I,D,IMAX,FLTT,FLTE,FLTD,SMAX}) computed
from its plant, with one operator aggressiveness knob (see suggest_rate_pid).
The narrative below is written for the pitch axis; yaw and roll are its direct
analogs.

WHY THIS EXISTS
    ArduPlane AUTOTUNE tunes the rate loop for buzz-free tracking of small, fast
    doublets.  Our balloon chaser maneuver is the opposite - a large, sustained pitch
    slew slammed to a stop - so AUTOTUNE optimizes the wrong operating point and
    can leave the loop too soft to brake (it drove PTCH_RATE_P to 0.03, doubling
    the slew overshoot).  To tune the rate loop for OUR task we measure the
    airframe cleanly, then COMPUTE the rate PID from it for the sustained slew -
    the P AUTOTUNE's doublet-ring search cannot find (see suggest_rate_pid).

    This module identifies the pitch short-period plant with NO flight-controller
    loop in the way, so the scaler, integrator, target filters and our own Lua
    cascade cannot distort the picture.  It excites the ELEVATOR directly in
    MANUAL (stick -> V-tail mixer -> surface, nothing else) and measures the body
    pitch rate with the FC's own gyro, read on the same FC tick as the servo
    signal it responds to (STATE2), then maps the fitted plant to the full
    ArduPlane rate PID for the operator to REVIEW.  Nothing here needs a
    simulator, so the same run identifies the real aircraft.

    Invoked with the 'i' key on an aircraft ALREADY IN FLIGHT, exactly like the
    other in-flight measurements ('i' plant-ID, 'k' freq-ID, 't' the sweep): it
    stands the balloon chaser cascade down, takes the airframe into MANUAL for a handful
    of open-loop doublets, then hands back to LOITER.  It launches nothing and
    changes no stored gains.

THE MODEL (grey-box short period, fitted about its operating point)
    q_ss is the steady pitch rate the doublet rides on: 0 for a trimmed,
    wings-level entry, non-zero when the airframe enters untrimmed or turning.
    The short period acts on the PERTURBATION p = q - q_ss:
        alpha_dot = p - Za*alpha
        p_dot     = M_delta*delta + M_q*p + M_alpha*alpha
        q(t)      = q_ss + p(t)
    delta = elevator surface (fraction of half-throw, +-1)
    q     = pitch rate (deg/s), alpha = angle of attack (deg)
    Fit (M_delta, M_q, M_alpha, Za, q_ss) by OUTPUT ERROR: simulate q(t) forward
    from delta(t) and minimise the mismatch to the measured q(t).  Output error -
    not differentiating q - is what keeps the fit unbiased under gyro noise; the
    naive q_dot regression is ~24% biased and was rejected.
    q_ss is an OUTPUT offset, not a moment bias, so the damping M_q acts on the
    perturbation and the airframe's background pitch rate is NOT charged to M_q or
    M_delta.  The model's only equilibrium is q=q_ss at delta=0, so a doublet that
    settles to a steady non-zero rate is fitted about that point instead of being
    forced through q=0 - forcing q=0 drives R^2 negative and rails M_delta, which
    is what an untrimmed entry does to the fit.  A single short doublet does not
    separate M_q from M_alpha, so the fit is multi-started across that trade (a
    damping-dominated seed, a stability-dominated seed, balanced ones) and the
    best-residual solution is kept.

WHAT IT YIELDS (airframe facts, independent of any FC gain)
    M_delta  control power  -> how hard pitch rate can be pushed AND braked
    M_q      pitch damping
    FF = -M_q/M_delta        surface needed to HOLD a steady rate (the honest
                             feedforward, fit at the sustained operating point)
    a_max = M_delta*delta_max  max pitch deceleration -> the OVERSHOOT FLOOR:
                             a rate R cannot be stopped in less than R^2/(2*a_max).

TIME BASE
    Samples are stamped with the FC clock of the tick they were read on (STATE's
    fc_millis, paired with STATE2 by seq), never the wall clock: SITL runs below
    real time, and a wall-clock dt would scale every identified
    rate/acceleration by 1/rtf.  Same rule as the POSITION fit in the
    controller.
"""
from __future__ import annotations

import logging
import math
import time

import numpy as np

from . import file_export
from .csv_logger import PACKAGE_SHA256

# NB: the ESTIMATOR below is pure numpy - the RC-override helper (_rc) lives in
# arduplane_autotune, which imports pymavlink, so it is imported lazily INSIDE the
# flight driver.  Keeping it off the module top lets the estimator and its selftest
# import and run with no flight stack installed.

log = logging.getLogger("balloon_chaser")

# ArduPlane MANUAL mode number for the open-loop excitation.  The wings-level
# settle between doublets is FBWA, entered inside link.level_and_hold (see
# _relevel); the hand-back to LOITER goes through the controller's canonical path.
MODE_MANUAL = 0
PWM_MID = 1500
# ArduPlane floors AIRSPEED_MIN at this wherever it uses it (defines.h
# MIN_AIRSPEED_MIN); it is not a parameter.
MIN_AIRSPEED_MIN_MPS = 5.0


def _airspeed_min(link):
    """The FC's AIRSPEED_MIN, floored as ArduPlane floors it, or None when the
    FC does not answer.  Read from the FC: the companion keeps no copy.  Read
    before MANUAL is entered, so the round trip is not flown open loop."""
    read = getattr(link, 'read_param', None)
    v = read('AIRSPEED_MIN') if read is not None else None
    return None if v is None else max(float(v), MIN_AIRSPEED_MIN_MPS)


def _airspeed_clear(link, aspd_min):
    """Gate step: the airspeed is at or above AIRSPEED_MIN.  With AIRSPEED_MIN
    unread the check cannot run, and the gate refuses rather than guess one."""
    asp = float(getattr(link, "airspeed", 0.0) or 0.0)
    if aspd_min is None:
        log.error("  AIRSPEED_MIN was not read from the FC - the underspeed check "
                  "cannot run")
        return False
    if asp < aspd_min:
        log.error("  airspeed %.1f below AIRSPEED_MIN %.1f - plant changes / "
                  "underspeed risk", asp, aspd_min)
        return False
    log.info("  airspeed %.1f m/s, AIRSPEED_MIN %.1f - clear", asp, aspd_min)
    return True

# Airframe parameter bounds for the fit (physical: control power positive,
# damping and static stability negative, AoA-response non-negative).  q_ss (the
# operating-point rate) is bounded per fit to the sampled rate range: its only
# honest bound is the data, since an output offset neither inverts a loop nor has
# a physical ceiling of its own.
_LO = (1.0, -50.0, -300.0, 0.0)
_HI = (2000.0, 0.0, 0.0, 40.0)
_QSS_PAD_DPS = 10.0    # q_ss may sit just outside the sampled range at the extremes

# Multi-start seeds for (M_delta, M_q, M_alpha, Za).  A short doublet does not
# separate damping from static stability, so the fit starts from a damping-
# dominated seed, a stability-dominated seed and two balanced ones; the q_ss seed
# is the sample median, appended per fit.  Best residual wins.
_FIT_SEEDS = ((150.0, -4.0, -30.0, 3.0),
              (150.0, -15.0, -5.0, 3.0),
              (150.0, -1.0, -120.0, 8.0),
              (400.0, -8.0, -60.0, 5.0))
_P0 = _FIT_SEEDS[0]


# ---------------------------------------------------------------------------
# Estimator (numpy-only; validated by selftest() to recover a known plant to
# ~1% control power over 20 noise seeds).
# ---------------------------------------------------------------------------
def _simulate_pitch_rate(p, t, delta, q0):
    """Forward-integrate the short-period model about its operating point and
    return q(t) (deg/s).  The perturbation pr = q - q_ss carries the dynamics and
    the measurement is pr + q_ss, so the damping M_q acts on the perturbation
    rather than on the raw rate: a doublet flown while the aircraft holds a steady
    q_ss is still identified about the right point."""
    Md, Mq, Ma, Za, qss = p
    dt = t[1] - t[0]
    pr = np.empty_like(t); al = np.empty_like(t)
    pr[0] = q0 - qss; al[0] = 0.0
    for i in range(1, len(t)):
        pr[i] = pr[i-1] + (Md*delta[i-1] + Mq*pr[i-1] + Ma*al[i-1]) * dt
        al[i] = al[i-1] + (pr[i-1] - Za*al[i-1]) * dt
    return pr + qss


def _lm_fit(seed, lo, hi, t, delta, q0, q_meas, iters):
    """One Levenberg-Marquardt run from a single seed. Returns (params, cost)."""
    p = np.array(seed, float)
    lam = 1e-3

    def res(pp):
        return _simulate_pitch_rate(pp, t, delta, q0) - q_meas

    r = res(p); cost = float(r @ r)
    for _ in range(iters):
        J = np.empty((len(r), len(p)))
        for k in range(len(p)):
            dp = max(1e-4, abs(p[k]) * 1e-4)
            pk = p.copy(); pk[k] += dp
            J[:, k] = (res(pk) - r) / dp
        JTJ = J.T @ J; grad = J.T @ r
        for _ls in range(30):
            # The added q_ss dimension can make JTJ singular for a degenerate
            # window; grow lam until it conditions rather than raising.
            try:
                step = np.linalg.solve(JTJ + lam * np.diag(np.diag(JTJ)), -grad)
            except np.linalg.LinAlgError:
                lam *= 3.0; continue
            pn = np.clip(p + step, lo, hi)
            rn = res(pn); cn = float(rn @ rn)
            if cn < cost:
                p, r, cost = pn, rn, cn; lam = max(lam * 0.5, 1e-9); break
            lam *= 3.0
        else:
            break
    return p, cost


def fit_pitch_plant(t, delta, q_meas, seeds=_FIT_SEEDS, iters=80, md_max=None):
    """Output-error fit of the pitch short-period about its operating point.
    Returns a dict of airframe parameters, the operating-point rate q_ss, the
    derived feedforward and the fit R^2.  Multi-start Levenberg-Marquardt, numpy-
    only: the fit runs from every seed in `seeds` (each is (M_delta, M_q, M_alpha,
    Za), with the sample median appended as the q_ss seed) and the best-residual
    solution is kept.

    `md_max` raises the control-power ceiling above the default `_HI[0]` for an
    axis whose authority exceeds it (roll); leaving it None keeps the pitch/yaw
    bound unchanged."""
    t = np.asarray(t, float); delta = np.asarray(delta, float); q_meas = np.asarray(q_meas, float)
    q0 = float(q_meas[0])
    # Airframe bounds are physical; q_ss is bounded to the sampled rate range.
    hi_md = _HI[0] if md_max is None else float(md_max)
    lo = np.array(_LO + (float(q_meas.min()) - _QSS_PAD_DPS,))
    hi = np.array((hi_md,) + _HI[1:] + (float(q_meas.max()) + _QSS_PAD_DPS,))
    qss0 = float(np.median(q_meas))

    best_p = None; best_cost = float("inf")
    for seed in seeds:
        p, cost = _lm_fit(tuple(seed) + (qss0,), lo, hi, t, delta, q0, q_meas, iters)
        if cost < best_cost:
            best_cost = cost; best_p = p

    Md, Mq, Ma, Za, qss = best_p
    ss_tot = float(np.sum((q_meas - q_meas.mean())**2))
    r2 = 1.0 - best_cost / ss_tot if ss_tot > 0 else float("nan")
    return dict(M_delta=Md, M_q=Mq, M_alpha=Ma, Za=Za, q_ss=qss,
                FF_surface_per_rate=(-Mq / Md if Md else float("nan")), R2=r2)


def _fit_roll(t, delta, rate):
    """Fit the roll plant, ESCALATING the control-power ceiling when the optimizer
    rails against it (see ROLL_MD_MAX / ROLL_MD_CAP).  Roll authority spans a wide
    range between airframes; a fixed ceiling that is too low clips a powerful roll
    axis and reports the ceiling instead of the plant.  Start at ROLL_MD_MAX and
    double whenever the fit sits at its ceiling, up to ROLL_MD_CAP, so the fit finds
    where it stops wanting more.  Adds `md_ceiling` (the ceiling this fit converged
    under) and `railed` (still at the CAP, i.e. genuinely non-linear - the rate
    saturates within the hold and no finite control power fits)."""
    md = ROLL_MD_MAX
    e = fit_pitch_plant(t, delta, rate, md_max=md)
    while e["M_delta"] >= 0.99 * md and md < ROLL_MD_CAP:
        md = min(md * 2.0, ROLL_MD_CAP)
        e = fit_pitch_plant(t, delta, rate, md_max=md)
    e["md_ceiling"] = md
    e["railed"] = bool(e["M_delta"] >= 0.99 * md and md >= ROLL_MD_CAP)
    return e


def overshoot_floor_deg(arrival_rate_dps, M_delta, delta_max=1.0):
    """Minimum stop angle for a pitch rate `arrival_rate_dps`, given the control
    power.  This is the physical floor no rate-loop tune can beat."""
    a_max = abs(M_delta * delta_max)
    return arrival_rate_dps**2 / (2.0 * a_max) if a_max > 0 else float("inf")


# ArduPlane RATE-controller mapping constants (verified against ArduPlane
# V4.7.0-dev).  All three fixed-wing axes run the SAME AC_PID rate loop:
# AP_FW_Controller::_get_rate_out for pitch and roll, AP_YawController::get_rate_out
# - byte-for-byte the same rate math - for yaw (used when YAW_RATE_ENABLE=1).
#   * the loop returns out*100 clamped to +-4500 centidegrees, and +-4500 IS full
#     surface throw, so a surface fraction maps to output as frac = out_deg/45.
#   * scaler = SCALING_SPEED / airspeed_EAS; the P/I/D see target*scaler^2 while
#     FF is divided back by scaler*eas2tas, so FF output = rate*scaler*FF/eas2tas.
_FULL_THROW_DEG = 45.0                       # +-4500 centidegrees = +-1.0 throw
_ARDUPLANE_SCALING_SPEED_DEFAULT = 15.0      # ArduPlane config.h SCALING_SPEED default


# --- From plant to the FULL ArduPlane rate-PID variable set -----------------
# ArduPlane AUTOTUNE (AP_AutoTune.cpp) and this module tune the SAME variables:
# the AC_PID rate loop <PREFIX>_RATE_{FF,P,I,D,IMAX,FLTT,FLTE,FLTD,SMAX} that every
# fixed-wing axis shares.  We reproduce AUTOTUNE's FF, I, IMAX and filter formulas
# verbatim, and REPLACE only its P and D.
#
# AUTOTUNE finds P and D by an in-flight search: raise the gain x1.3 until a
# ~0.7 s doublet makes the slew-rate limiter ring, then back off.  On the
# sustained balloon chaser slew that search settles ~30x too LOW - the hand-found
# PTCH_RATE_P=1.0 cut arrival overshoot where AUTOTUNE's 0.033 doubled it
# (Notes/autotune_mismatch_report.md).  Any cost that reads "no ring on a short
# doublet" converges near AUTOTUNE's answer, so the search is exactly what must
# not be reused; the identified plant lets us COMPUTE P and D for the maneuver we
# actually fly.
#
# Surface mapping, for a throw-fraction gain G [throw per (deg/s)]: the loop runs
# AC_PID in radians on target*scaler^2, scales P/I/D back by degrees(1) and FF by
# 1/(scaler*eas2tas), so
#     kFF = G_FF * 45 * eas2tas / scaler      (FF sees scaler^1)
#     kP  = G_P  * 45 / scaler^2              (P and D see scaler^2)
#     kD  = G_D  * 45 / scaler^2 .
#
# P - closed-loop rate-bandwidth placement.  With FF making the DC gain 1, the
# first-order rate plant  q_dot = M_delta*delta + M_q*p  (M_q<0) closes under P to
# a single pole at omega_cl = M_delta*G_P - M_q, so
#     G_P = (omega_cl + M_q) / M_delta ,   omega_cl = aggressiveness*RATE_BW_REF.
# The BANDWIDTH is the transferable target, not the raw gain: ArduPlane's speed
# scaler already makes M_delta*scaler^2 airspeed-invariant, so one kP holds over
# the speed envelope and one omega_cl transfers between airframes.
#
# D - critically-damped closing against the derivative-filter pole.  PD on a pure
# first-order plant stays first-order (D just rescales the one pole), so D is
# defined against the one extra pole the loop really has: the derivative low-pass
# at tau_a = 1/(2*pi*FLTD).  Placing that 2nd-order loop at (omega_cl, zeta=1)
# gives G_D = (2*zeta*omega_cl*tau_a - 1 + M_q*tau_a)/M_delta, clamped >=0 so D can
# never invert the loop.  Near the validated operating point this is ~0 - which is
# why the hand tune left D at ArduPlane's default - and it grows only as
# aggressiveness raises the bandwidth.  D is a damping trim; P is the lever.
RATE_BW_REF_RADPS = 34.5    # rad/s: the closed-loop rate pole PTCH_RATE_P=1.0 places
#                             on the reference airframe (M_delta~1786 deg/s^2/throw,
#                             M_q~-5.62/s at scaler~0.85: 1786*(1.0*0.85^2/45)+5.62
#                             ~ 34.5).  aggressiveness 1.0 reproduces that flight-
#                             validated operating point; it is the one point we
#                             have, not a universal law - re-derive it (and the
#                             selftest pin) if the reference airframe changes.
RATE_ZETA_CL = 1.0          # critically damped: no rate-loop overshoot, the balloon chaser
#                             failure mode this tune exists to remove.
_RATE_AGG_DEFAULT = 1.0     # aggressiveness that reproduces the validated tune

# ArduPlane AUTOTUNE's own derivations, reproduced verbatim (AP_AutoTune.cpp):
_AT_I_RATIO      = 0.75     # pitch/yaw: I = max(0.75*P, FF/TRIM_TCONST)
_AT_TRIM_TCONST  = 1.0      # roll:      I = min(P,      FF/TRIM_TCONST)
_AT_IMAX_MIN, _AT_IMAX_MAX = 0.4, 0.9   # AUTOTUNE clamps IMAX into this at start
_AT_SMAX_DEFAULT = 150.0    # slew limit AUTOTUNE installs when none is set
_AC_PID_IMAX_DEFAULT = 0.666   # shipped AC_PID IMAX (pitch/roll/yaw) - clamps to itself
_ATT_TCONST_DEFAULT  = 0.5     # attitude-loop tconst (shipped 2SRV_TCONST); sets FLTT
_GYRO_FILTER_HZ_DEFAULT = 20.0 # INS_GYRO_FILTER default -> FLTD = 0.5*this
_RATE_PID_ORDER = ('FF', 'P', 'I', 'D', 'IMAX', 'FLTT', 'FLTE', 'FLTD', 'SMAX')
_RATE_AGG_TABLE = (0.7, 1.0, 1.5, 2.0)   # aggressiveness rows shown in the report


def suggest_rate_pid(prefix, control_power, damping, aspeed_mps, scaling_speed,
                     *, roll_axis=False, aggressiveness=_RATE_AGG_DEFAULT,
                     eas2tas=1.0, gyro_filter_hz=_GYRO_FILTER_HZ_DEFAULT,
                     tconst=_ATT_TCONST_DEFAULT):
    """Map an identified rate plant to the FULL ArduPlane rate-PID variable set
    <prefix>_RATE_{FF,P,I,D,IMAX,FLTT,FLTE,FLTD,SMAX}, for the operator to REVIEW
    and program.  Applies nothing (measurement-only; real-hardware config is
    read-only by policy).  prefix is 'PTCH' / 'RLL' / 'YAW'.

    control_power is M_delta [deg/s^2 per full throw], damping is M_q [1/s]
    (negative).  All three axes share ArduPlane's AC_PID rate loop, so one
    derivation serves them all; roll_axis only switches AUTOTUNE's I formula.
    FF/I/IMAX/filters reproduce AUTOTUNE's formulas verbatim; P and D are COMPUTED
    from the plant (see the block above) rather than found by AUTOTUNE's
    raise-until-ring doublet search, which drives P the wrong way for the balloon chaser
    slew.  Returns the named parameters plus the physics behind them."""
    Md = float(control_power)
    Mq = float(damping)
    aspeed = max(1.0, float(aspeed_mps))
    ss = float(scaling_speed) if scaling_speed and scaling_speed > 0 else _ARDUPLANE_SCALING_SPEED_DEFAULT
    e2t = float(eas2tas) if eas2tas and eas2tas > 0 else 1.0
    scaler = ss / aspeed
    agg = max(0.0, float(aggressiveness))
    gyro_hz = float(gyro_filter_hz) if gyro_filter_hz and gyro_filter_hz > 0 else _GYRO_FILTER_HZ_DEFAULT
    tc = float(tconst) if tconst and tconst > 0 else _ATT_TCONST_DEFAULT

    have = bool(Md) and math.isfinite(Md) and math.isfinite(Mq)
    ff_plant = (-Mq / Md) if have else float("nan")          # throw per deg/s
    kff = ff_plant * _FULL_THROW_DEG * e2t / scaler if have else float("nan")

    # P: place the first-order closed-loop rate pole at omega_cl.
    omega_cl = agg * RATE_BW_REF_RADPS
    g_p = ((omega_cl + Mq) / Md) if have else float("nan")   # throw per deg/s error
    if math.isfinite(g_p):
        g_p = max(0.0, g_p)                                  # never invert the loop
    kp = g_p * _FULL_THROW_DEG / (scaler * scaler)

    # D: critically damp the plant + derivative-filter (FLTD) pole; clamp >= 0.
    fltd = 0.5 * gyro_hz
    tau_a = 1.0 / (2.0 * math.pi * fltd) if fltd > 0 else 0.0
    g_d = (((2.0 * RATE_ZETA_CL * omega_cl * tau_a - 1.0 + Mq * tau_a) / Md)
           if have else 0.0)
    g_d = max(0.0, g_d) if math.isfinite(g_d) else 0.0
    kd = g_d * _FULL_THROW_DEG / (scaler * scaler)

    # I, IMAX and filters: ArduPlane AUTOTUNE's formulas, verbatim.
    if math.isfinite(kff):
        ki = min(kp, kff / _AT_TRIM_TCONST) if roll_axis else max(kp * _AT_I_RATIO, kff / _AT_TRIM_TCONST)
    else:
        ki = kp if roll_axis else kp * _AT_I_RATIO
    kimax = min(max(_AC_PID_IMAX_DEFAULT, _AT_IMAX_MIN), _AT_IMAX_MAX)
    fltt = 10.0 / (2.0 * math.pi * tc)
    flte = 0.0

    params = {'FF': kff, 'P': kp, 'I': ki, 'D': kd, 'IMAX': kimax,
              'FLTT': fltt, 'FLTE': flte, 'FLTD': fltd, 'SMAX': _AT_SMAX_DEFAULT}
    named = {('%s_RATE_%s' % (prefix, k)): params[k] for k in _RATE_PID_ORDER}
    return dict(prefix=prefix, params=params, named=named, FF_plant=ff_plant,
                G_P=g_p, G_D=g_d, omega_cl=omega_cl, aggressiveness=agg,
                zeta_cl=RATE_ZETA_CL, scaler=scaler, scaling_speed=ss,
                aspeed=aspeed, eas2tas=e2t, gyro_filter_hz=gyro_hz, tconst=tc)


def _emit_rate_pid(prefix, control_power, damping, unreliable_note, ctrl, out,
                   *, roll_axis=False):
    """Compute the full ArduPlane rate-PID suggestion for one axis, add its
    paste-ready lines to the run report, and log an aggressiveness sensitivity
    table.  Shared by the pitch, yaw and roll stages - one law, three axes.  Reads
    the operator's aggressiveness knob (plantid_rate_agg) live; SCALING_SPEED is
    not on the wire, so the ArduPlane default is assumed and stated."""
    aspeed = float(getattr(ctrl.link, "airspeed", 0.0) or 0.0)
    if aspeed < 1.0 or not control_power:
        log.info("  (no airspeed available - skipping the suggested %s rate PID)", prefix)
        return
    try:
        agg = float(getattr(ctrl, "plantid_rate_agg", _RATE_AGG_DEFAULT) or _RATE_AGG_DEFAULT)
    except (TypeError, ValueError):
        agg = _RATE_AGG_DEFAULT
    ss = _ARDUPLANE_SCALING_SPEED_DEFAULT
    s = suggest_rate_pid(prefix, control_power, damping, aspeed, ss,
                         roll_axis=roll_axis, aggressiveness=agg)
    log.info("  --- suggested ArduPlane %s rate PID (REVIEW; nothing applied) ---", prefix)
    log.info("    at airspeed %.1f m/s, SCALING_SPEED %.1f (ArduPlane default assumed), "
             "eas2tas 1.0, gyro filter %.0f Hz assumed", aspeed, ss, s["gyro_filter_hz"])
    log.info("    aggressiveness %.2f -> closed-loop rate bandwidth %.1f rad/s (%.1f Hz); "
             "1.0 reproduces the flight-validated balloon chaser tune",
             agg, s["omega_cl"], s["omega_cl"] / (2.0 * math.pi))
    for k in _RATE_PID_ORDER:
        name = '%s_RATE_%s' % (prefix, k)
        out.add_param(name, s["params"][k])
        log.info("    %-14s = %.4f", name, s["params"][k])
    log.info("    basis: FF = -M_q/M_delta (measured; equals AUTOTUNE's FF); P places the "
             "rate bandwidth from the plant; D critically damps it (~0 near the validated "
             "point); I = %s per AUTOTUNE; IMAX/FLTT/FLTE/FLTD/SMAX are AUTOTUNE's formulas.",
             "min(P, FF)" if roll_axis else "max(0.75*P, FF)")
    if prefix == 'PTCH':
        log.info("    attitude loop (design defaults, not plant-derived): PTCH2SRV_TCONST = "
                 "%.2f, PTCH2SRV_RMAX_UP/DN = 0 (no limit); FLTT follows TCONST.", s["tconst"])
    elif prefix == 'RLL':
        log.info("    attitude loop (design defaults, not plant-derived): RLL2SRV_TCONST = "
                 "%.2f, RLL2SRV_RMAX = 0 (no limit); FLTT follows TCONST.", s["tconst"])
    else:  # yaw has no stored attitude-loop TCONST/RMAX params
        log.info("    yaw has no stored attitude-loop TCONST/RMAX; AUTOTUNE assumes tconst "
                 "%.2f, used here only to set FLTT.", s["tconst"])
        log.info("    NOTE: these YAW_RATE_* gains drive ArduPlane's direct yaw-rate "
                 "controller, active only with YAW_RATE_ENABLE=1; under the default "
                 "sideslip damper (YAW2SRV_*) they are inert.")
    log.info("    aggressiveness sensitivity:")
    for a in _RATE_AGG_TABLE:
        sa = suggest_rate_pid(prefix, control_power, damping, aspeed, ss,
                              roll_axis=roll_axis, aggressiveness=a)
        log.info("      agg %.2f -> %s_RATE_P %.4f  _D %.4f  (bandwidth %.1f rad/s)",
                 a, prefix, sa["params"]["P"], sa["params"]["D"], sa["omega_cl"])
    if unreliable_note:
        log.warning("    NOTE: %s", unreliable_note)


def selftest():
    """Recover a known synthetic plant from an open-loop doublet - once from a
    trimmed entry (q_ss=0) and once riding a steady offset (q_ss=-25) - and
    assert both the control power and the operating-point rate come back
    accurately, so a regression in the estimator fails loudly.  The offset case
    guards the very failure this fit exists for: a doublet flown about a non-zero
    pitch rate must NOT charge that rate to the airframe."""
    fs = 50.0; t = np.arange(0, 4, 1 / fs)
    d = np.zeros_like(t)
    d[(t >= 0.3) & (t < 0.8)] = 0.5
    d[(t >= 0.8) & (t < 1.3)] = -0.5
    mean_err = max_err = 0.0
    for qss_true in (0.0, -25.0):
        true = (140.0, -4.5, -35.0, 3.0, qss_true)
        q = _simulate_pitch_rate(true, t, d, qss_true)
        errs = []; qss_errs = []; mq_errs = []
        for s in range(20):
            qm = q + np.random.default_rng(s).normal(0, 0.3, q.shape)
            e = fit_pitch_plant(t, d, qm)
            errs.append(100 * abs(e["M_delta"] - true[0]) / true[0])
            qss_errs.append(abs(e["q_ss"] - qss_true))
            mq_errs.append(abs(e["M_q"] - true[1]))
        me, xe, xqe = float(np.mean(errs)), float(np.max(errs)), float(np.max(qss_errs))
        xmqe = float(np.max(mq_errs))
        log.info("plant_id selftest (q_ss=%.0f): control-power error mean %.1f%% max %.1f%%,"
                 " q_ss error max %.1f deg/s, M_q error max %.2f /s", qss_true, me, xe, xqe, xmqe)
        assert xe < 5.0, "estimator regression: control-power error %.1f%% at q_ss=%.0f" % (xe, qss_true)
        assert xqe < 2.0, "estimator regression: q_ss error %.1f deg/s at q_ss=%.0f" % (xqe, qss_true)
        # Damping observability from a STEADY entry is what the pitch-trim search
        # (_trim_pitch_manual) buys: a drifting entry rails M_q at 0 and collapses
        # the feedforward, so guard that a clean doublet still recovers M_q.
        assert xmqe < 1.0, "estimator regression: M_q error %.2f /s at q_ss=%.0f" % (xmqe, qss_true)
        mean_err, max_err = max(mean_err, me), max(max_err, xe)
    # The plant -> ArduPlane FF map must invert: the FF elevator ArduPlane produces
    # for a demanded rate must equal the surface the plant needs to hold it.
    sp = suggest_rate_pid('PTCH', 1800.0, -4.0, 20.0, 15.0)
    R, scaler = 30.0, 15.0 / 20.0
    ff_out_throw = R * scaler * sp["named"]["PTCH_RATE_FF"] / _FULL_THROW_DEG   # ArduPlane FF, as throw
    plant_throw = (4.0 / 1800.0) * R                                  # FF_plant * R
    assert abs(ff_out_throw - plant_throw) < 1e-6, \
        "PTCH_RATE_FF map does not invert: %g vs %g" % (ff_out_throw, plant_throw)
    # Same inversion pin for the roll FF map: RLL_RATE_FF shares AP_FW_Controller
    # with the pitch one, so the FF surface ArduPlane produces for a demanded roll
    # rate must equal the aileron the plant needs to hold it.
    rp = suggest_rate_pid('RLL', 900.0, -6.0, 20.0, 15.0, roll_axis=True)
    roll_ff_out_throw = R * scaler * rp["named"]["RLL_RATE_FF"] / _FULL_THROW_DEG
    roll_plant_throw = (6.0 / 900.0) * R
    assert abs(roll_ff_out_throw - roll_plant_throw) < 1e-6, \
        "RLL_RATE_FF map does not invert: %g vs %g" % (roll_ff_out_throw, roll_plant_throw)
    # The P placement must reproduce the flight-validated PTCH_RATE_P=1.0 at
    # aggressiveness 1.0 on the mismatch-report reference plant (M_delta~1786,
    # M_q~-5.62 at the balloon chaser scaler ~0.85); if it drifts, RATE_BW_REF_RADPS no
    # longer encodes that operating point (Notes/autotune_mismatch_report.md).
    ref = suggest_rate_pid('PTCH', 1786.0, -5.62, 17.6, 15.0, aggressiveness=1.0)
    assert abs(ref["named"]["PTCH_RATE_P"] - 1.0) < 0.05, \
        "RATE_BW_REF_RADPS no longer reproduces the validated P=1.0: got %.3f" \
        % ref["named"]["PTCH_RATE_P"]
    # D must never invert the loop and must vanish at the validated point (the hand
    # tune left D at ArduPlane's default), then grow as aggressiveness climbs.
    assert ref["named"]["PTCH_RATE_D"] >= 0.0, "rate D went negative"
    hot = suggest_rate_pid('PTCH', 1786.0, -5.62, 17.6, 15.0, aggressiveness=2.0)
    assert hot["named"]["PTCH_RATE_D"] > ref["named"]["PTCH_RATE_D"], \
        "rate D should grow with aggressiveness"
    # Roll authority can exceed the default fit ceiling; _fit_roll must ESCALATE and
    # recover a high-authority plant that the fixed default bound would clip - the
    # railing that a fixed ceiling produced on a powerful roll axis.
    tr = np.arange(0, 3, 1 / fs)
    dr = np.zeros_like(tr)
    dr[(tr >= 0.3) & (tr < 0.7)] = 0.1
    dr[(tr >= 0.7) & (tr < 1.1)] = -0.1
    roll_true = (14000.0, -15.0, 0.0, 0.0, 0.0)      # M_delta well above ROLL_MD_MAX
    pr = _simulate_pitch_rate(roll_true, tr, dr, 0.0)
    clipped = fit_pitch_plant(tr, dr, pr)            # default ceiling _HI[0]
    assert clipped["M_delta"] >= 0.99 * _HI[0], \
        "expected the default ceiling to clip a 14000 deg/s^2 roll plant"
    er = _fit_roll(tr, dr, pr)
    assert not er["railed"], "escalating roll fit railed at the cap on a linear plant"
    assert abs(er["M_delta"] - 14000.0) / 14000.0 < 0.05, \
        "escalating roll fit did not recover control power: %.0f" % er["M_delta"]
    return mean_err, max_err


# The estimator is fixed within a process, so it need only be validated ONCE.
# selftest() is a pure-Python multi-start fit that holds the GIL for a few
# seconds; run from an axis stage it stalls the live control loop, the telemetry
# reader and the GUI right after 'i' is pressed, and repeats the same check once
# per axis.  main.py calls selftest_once() at startup - nothing is flying, so the
# GIL is free - and each stage calls it too but pays only the cached result, so
# 'i' is instant.  A regression still fails loudly (at startup, before flight),
# and the failure is REMEMBERED so the 'i' key keeps refusing until it is fixed.
_selftest_done = False
_selftest_error = None


def selftest_once():
    """Run selftest() at most once per process and replay the cached outcome:
    return on a validated estimator, re-raise the AssertionError on a failed one."""
    global _selftest_done, _selftest_error
    if not _selftest_done:
        try:
            selftest()
        except AssertionError as e:                # remember the regression
            _selftest_error = e
        _selftest_done = True
    if _selftest_error is not None:
        raise _selftest_error


# ---------------------------------------------------------------------------
# In-flight driver ('i' key).  The aircraft is ALREADY airborne and the balloon chaser
# cascade is ticking; this stands the cascade down, excites the elevator in
# MANUAL, records the FC's own pitch-rate response one clean doublet at a time,
# and hands back to LOITER.  Contract mirrors run_plant_id/run_freq_id:
# (ctrl, abort), no bring-up, teardown through the controller's canonical path.
# ---------------------------------------------------------------------------
DOUBLET_HOLD_S = 0.6     # each half of the doublet (pitch/yaw)
# Pitch and yaw need OPPOSITE amplitudes, so they do not share a ladder.
# PITCH: a large open-loop elevator deflection stalls the airframe at the nose-up
# peak in MANUAL, where nothing holds the wings level; it then drops a wing and
# spirals into a dive that the between-doublet level-out cannot recover inside its
# timeout, losing hundreds of metres and leaving the next doublet's fit railed.
# So pitch stays below that stall boundary - the largest amplitude here is about
# the deflection the directness gate proved controllable, no more.  The linearity
# check lives in the spread across these smaller rungs, not in reaching for a big
# one.  YAW: the rudder is a much WEAKER axis (a full-throw rudder commands a
# fraction of the elevator's pitch rate), so it needs the LARGER deflection for a
# rate swing well clear of gyro noise, and being weak it does not depart the
# aircraft, so it keeps the bigger ladder.
PITCH_DOUBLET_AMPS_PWM = (90, 120, 150)   # elevator deflections from trim (stall-bounded)
YAW_DOUBLET_AMPS_PWM = (120, 180, 240)    # rudder deflections from trim (weak axis, needs the swing)
REPS_PER_AMP = 2
SAMPLE_SECONDS = 2.2     # record window per doublet (covers rise + free decay)
# Roll is a FAR more powerful axis than pitch/yaw - a full-throw aileron commands
# hundreds of deg/s of roll rate where the elevator commands tens - so the shared
# amplitudes roll the aircraft clean past the linear regime (inverted, large-angle)
# within one hold, which the short-period model cannot fit: it rails the control
# power and returns R^2 ~ 0.77.  Roll therefore uses SMALLER amplitudes and a
# SHORTER hold to keep the bank excursion inside the linear roll-subsidence regime,
# scaled from the directness gate's measured roll authority (~4x pitch's).
ROLL_DOUBLET_HOLD_S = 0.4
ROLL_DOUBLET_AMPS_PWM = (30, 45, 60)
# Roll control power exceeds the pitch/yaw fit ceiling (_HI[0]) and varies widely
# between airframes, so no single fixed ceiling fits all: too low clips a genuinely
# high-authority roll axis (the optimizer rails at the bound and the fit reports the
# bound, not the airframe).  _fit_roll therefore ESCALATES - it starts the fit at
# ROLL_MD_MAX and doubles the ceiling whenever the optimizer rails against it, up to
# ROLL_MD_CAP, so the fit finds where it stops wanting more.  These are fit guards,
# not physical limits.  A fit that still rails at the CAP is genuinely non-linear
# (the roll rate saturates within the hold - no finite control power fits) and is
# flagged, not trusted; ROLL_MD_CAP sits well beyond any real airframe so reaching
# it means saturation, not authority.
ROLL_MD_MAX = 10000.0
ROLL_MD_CAP = 100000.0
# Above this fitted operating-point rate the doublets were not flown from trimmed
# level flight.  The fit still identifies the plant about that point, but the
# result is flagged so the operator can re-fly from a cleaner state.
SETTLE_MAX_Q_DPS = 5.0


class _Aborted(Exception):
    """The run was stopped mid-stage: 'i' again, or another mode key switching
    straight to its own mode."""


def _op_throttle_pwm(ctrl):
    """The OPERATOR's throttle as a CH3 PWM, mapped exactly as the controller
    maps it (1000 + pct/100 * 1000).  Held steady through the probe so airspeed -
    and therefore the plant - does not wander; we do not invent a cruise value."""
    pct = max(0.0, min(100.0, float(getattr(ctrl, "throttle_pct", 0.0) or 0.0)))
    return int(1000 + pct * 10.0)


def _tick(link):
    """The FC's latest tick as (t_s, p_dps, q_dps, r_dps, pwm_aileron,
    pwm_vtail_l, pwm_vtail_r), or None before the first STATE2 frame.

    The FC's OWN readings - its gyro and the servo signals it sent, stamped with
    its clock (lua_offload.lua_tick_sample) - so each surface position is paired
    with the rate it produced on the tick both happened, and the measurement
    needs no simulator: the same run works on hardware."""
    s = getattr(link, "lua_tick_sample", None)
    if s is None:
        return None
    ms, gx, gy, gz, pa, pl, pr = s
    return (ms / 1000.0, math.degrees(gx), math.degrees(gy), math.degrees(gz),
            pa, pl, pr)


def _servo_frac(pwm, trim):
    """A servo signal as a fraction of half-throw about its trim; None for PWM 0
    (no channel carries the function)."""
    return None if not pwm else (float(pwm) - trim) / 500.0


def _pitch_rate_dps(link):
    """Body pitch rate from the FC's gyro (deg/s), or None with no STATE2."""
    s = _tick(link)
    return None if s is None else s[2]


def _elevator_frac(link, s=None):
    """Elevator surface as a fraction of half-throw, from the V-tail left
    servo signal the FC sent (STATE2 pwm_vtail_l); None when absent.  `s` is a
    _tick() sample, so a caller can take the surface and the rate from one tick."""
    s = _tick(link) if s is None else s
    if s is None:
        return None
    return _servo_frac(s[5], float(getattr(link, "_servo2_trim", 1500.0) or 1500.0))


def _directness_gate(ctrl, aspd_min):
    """Prove, before any measurement, that our command reaches the surface and
    our measurement reflects the airframe - or refuse to run. Returns True/PASS.
    Assumes the aircraft is already in MANUAL."""
    from .arduplane_autotune import _rc
    link = ctrl.link
    thr = _op_throttle_pwm(ctrl)
    log.info("=== directness self-test (open-loop) ===")
    ok = True
    # 1. a small elevator step must move the surface, symmetric on both V-tail
    #    halves (no rudder contamination), with a rate response of the right sign.
    q0 = _pitch_rate_dps(link)
    if q0 is None:
        log.error("  no FC gyro (STATE2) - is the FC script running? identification "
                  "impossible")
        return False
    _rc(link, pitch=PWM_MID + 150, throttle=thr)
    time.sleep(0.4)
    s = _tick(link)
    d = _elevator_frac(link, s)
    if d is None or not s[6]:
        log.error("  no V-tail servo signal from the FC - identification impossible")
        _rc(link, throttle=thr)
        return False
    diff = abs(float(s[5]) - float(s[6]))
    q1 = s[2]
    _rc(link, throttle=thr)                     # neutral elevator
    log.info("  elevator step: surface frac %.2f, V-tail asymmetry %.0f PWM, dq %.1f deg/s",
             d, diff, (q1 - q0))
    if abs(d) < 0.05:
        log.error("  surface did not move for a stick step - not in MANUAL / not commanding?"); ok = False
    if diff > 15:
        log.error("  V-tail halves disagree by %.0f PWM - rudder/mix contaminating pitch", diff); ok = False
    # 2. airspeed sane and clear of the underspeed lock
    if not _airspeed_clear(link, aspd_min):
        ok = False
    log.info("  directness gate: %s", "PASS" if ok else "FAIL")
    return ok


def _below_floor(ctrl):
    """True if AGL has fallen below the (live) protection floor.  Mirrors the
    sweep and roll plant-ID; missing telemetry -> False, so a link without
    altitude cannot false-trip a climb."""
    try:
        return float(ctrl.link.relative_alt) < float(ctrl.autotune_protect_floor_m)
    except Exception:
        return False


def _relevel(ctrl, abort):
    """Wings-level settle between doublets, VERIFIED from telemetry so each doublet
    starts from the same trimmed, wings-level, nose-steady state instead of a fixed
    open-loop wait that could hand the excitation a residual bank or dive.  Reuses
    the sweep's straight-and-level mechanism and tolerances - link.level_and_hold
    holds roll angle + yaw rate + pitch angle + pitch rate all within tolerance for
    STRAIGHT_HOLD_S in FBWA, bounded by STRAIGHT_TIMEOUT_S - imported rather than
    copied so the plant-ID stages and 't' cannot drift apart.

    A residual bank or dive left by the previous doublet or the LOITER entry is
    exactly what the open-loop MANUAL excitation then rides, and it surfaces as a
    non-zero operating-point rate q_ss in the fit; verifying level here removes that
    entry variable.  A level-out TIMEOUT logs and proceeds - the fit still
    identifies about whatever point it gets and flags a large q_ss - exactly as the
    sweep and 'i' do.  Honours the abort ('i' again).

    Imported inside the function, like _rc, so the estimator and its selftest stay
    importable with no flight stack installed."""
    from .controller_autotune_sweep import (
        STRAIGHT_HOLD_S, STRAIGHT_PITCHRATE_TOL_DPS, STRAIGHT_PITCH_TOL_DEG,
        STRAIGHT_ROLL_TOL_DEG, STRAIGHT_TIMEOUT_S, STRAIGHT_YAWRATE_TOL_DPS)
    link = ctrl.link
    # Altitude protection, symmetric with the sweep ('t') and roll plant-ID ('i'):
    # the open-loop nose-down doublets bleed height, so climb back to the climb-to
    # altitude BEFORE levelling whenever AGL is below the floor - never
    # pre-emptively.  Floor and climb-to are the live Gain-Tuner knobs, read here.
    climb = getattr(link, 'climb_to', None)
    if climb is not None and _below_floor(ctrl):
        climb_to = float(ctrl.autotune_protect_climb_to_m)
        log.info("[PPID] altitude %.0fm < floor %.0fm - climbing to %.0fm before the "
                 "doublet", getattr(link, 'relative_alt', -1.0),
                 float(ctrl.autotune_protect_floor_m), climb_to)
        _base = (getattr(ctrl, 'autotune_status', '') or '').split(' | ')[0]
        ctrl.autotune_status = ((_base + ' | ') if _base else '') + \
            'Alt protect: climbing to %.0fm' % climb_to
        try:
            climb(climb_to, abort)
        except Exception as e:                          # noqa: BLE001
            log.warning("[PPID] protection climb failed (%s) - continuing", e)
        if _base:
            ctrl.autotune_status = _base
        if abort.is_set():
            raise _Aborted
    throttle_frac = max(0.0, min(1.0, float(getattr(ctrl, "throttle_pct", 0.0) or 0.0) / 100.0))
    ctrl.straighten_start_t = time.monotonic()      # drives the HUD "Straight" timer
    try:
        achieved = link.level_and_hold(
            STRAIGHT_ROLL_TOL_DEG, STRAIGHT_YAWRATE_TOL_DPS, STRAIGHT_HOLD_S,
            STRAIGHT_TIMEOUT_S, throttle_frac=throttle_frac, abort=abort,
            pitch_tol_deg=STRAIGHT_PITCH_TOL_DEG,
            pitch_rate_tol_dps=STRAIGHT_PITCHRATE_TOL_DPS)
    finally:
        ctrl.straighten_start_t = None              # stop/hide the HUD timer
    if abort.is_set():
        raise _Aborted
    if not achieved:
        log.warning("[PPID] could not hold wings-level + nose-steady within %.0fs before a "
                    "doublet - it starts from a non-level attitude", STRAIGHT_TIMEOUT_S)


# Pitch-trim search bounds/gains.  The correction is CLAMPED to +-TRIM_MAX_OFFSET
# of neutral so a wrong response cannot command a runaway, and the sign is fixed
# by the doublet's own convention (PWM_MID + amp is nose-up, so higher elevator
# PWM raises the rate): to null a rate q, move the PWM by -Kp*q.
TRIM_KP_PWM_PER_DPS = 3.0
TRIM_MAX_OFFSET_PWM = 250.0
TRIM_NULL_TOL_DPS = 3.0
TRIM_HOLD_S = 0.4
TRIM_TIMEOUT_S = 5.0


def _trim_pitch_manual(ctrl, abort):
    """In MANUAL, return the elevator PWM that holds a steady (non-drifting) pitch
    rate, so the doublet perturbs a settled operating point instead of bunting.

    FBWA holds level with a NON-ZERO elevator; MANUAL's mechanical neutral is a
    different angle, so a doublet centred on neutral rides a large drifting q_ss
    that the constant-offset short-period model cannot represent - the damping M_q
    rails at 0 and the derived feedforward collapses (as the first SITL run showed:
    q_ss ~ -23 deg/s, FF ~ 0).  A short proportional null on the MEASURED body
    pitch rate finds the settled elevator without the RC->servo->V-tail-mix
    calibration, whose reversals make an open-loop servo match fragile.

    Bounded and self-limiting: a search that cannot null within the timeout returns
    the PWM that gave the smallest rate it saw, so even a poorly-behaved search
    hands back its best steady point rather than a runaway.  Returns PWM_MID
    when the FC sends no gyro, where there is no rate to null."""
    from .arduplane_autotune import _rc
    link = ctrl.link
    thr = _op_throttle_pwm(ctrl)
    pwm = float(PWM_MID)
    best_pwm, best_absq = float(PWM_MID), float("inf")
    t0 = time.time(); held_since = None
    while time.time() - t0 < TRIM_TIMEOUT_S:
        if abort.is_set():
            raise _Aborted
        _rc(link, pitch=int(round(pwm)), throttle=thr)
        time.sleep(0.12)                              # let the short period respond
        q = _pitch_rate_dps(link)
        if q is None:
            return PWM_MID                            # no rate feed -> cannot trim
        if abs(q) < best_absq:
            best_absq, best_pwm = abs(q), pwm
        if abs(q) < TRIM_NULL_TOL_DPS:
            held_since = held_since if held_since is not None else time.time()
            if time.time() - held_since >= TRIM_HOLD_S:
                return int(round(pwm))
        else:
            held_since = None
            pwm -= TRIM_KP_PWM_PER_DPS * q
            pwm = max(PWM_MID - TRIM_MAX_OFFSET_PWM,
                      min(PWM_MID + TRIM_MAX_OFFSET_PWM, pwm))
    log.info("[PPID] pitch trim did not fully null within %.0fs - centring the "
             "doublet on the steadiest elevator found (residual %.0f deg/s)",
             TRIM_TIMEOUT_S, best_absq)
    return int(round(best_pwm))


def _fly_one_doublet(ctrl, amp_pwm, abort):
    """Fly one elevator doublet in MANUAL and record the FC-time-stamped
    (delta, q) response.  `delta` is the elevator DEVIATION from the level-holding
    trim (so it is 0 at the operating point, matching the model's delta=0
    equilibrium) and q rides ~level (see _trim_pitch_manual).  Returns (t, delta,
    q) arrays or None if too few samples.  Honours the abort ('i' again)."""
    from .arduplane_autotune import _rc
    link = ctrl.link
    thr = _op_throttle_pwm(ctrl)
    link._set_mode(MODE_MANUAL, "MANUAL")
    trim_pwm = _trim_pitch_manual(ctrl, abort)     # find the level-holding elevator PWM
    # The trim search flew MANUAL open-loop for up to a few seconds with the
    # ailerons uncommanded, so the wings can have drifted off the level the loop's
    # _relevel verified.  RE-LEVEL in FBWA so the maneuver STARTS wings-level, then
    # return to MANUAL and settle at the trimmed elevator - still valid, since the
    # brief level-out holds airspeed - so the doublet begins both level AND steady.
    _relevel(ctrl, abort)
    link._set_mode(MODE_MANUAL, "MANUAL")
    _rc(link, pitch=max(1000, min(2000, trim_pwm)), throttle=thr)
    time.sleep(0.5)                                # settle level at the trimmed elevator
                                                   # (also lets q re-null after the brief
                                                   # neutral-elevator MANUAL-entry transient)
    delta0 = _elevator_frac(link)                  # the trimmed elevator: the doublet's zero
    if delta0 is None:
        return None

    ts, ds, qs = [], [], []
    last_stamp = None
    seq = [(trim_pwm + amp_pwm, DOUBLET_HOLD_S),   # nose-up half
           (trim_pwm - amp_pwm, DOUBLET_HOLD_S),   # nose-down half
           (trim_pwm, SAMPLE_SECONDS - 2 * DOUBLET_HOLD_S)]  # release, watch free decay
    for pwm, hold in seq:
        _rc(link, pitch=max(1000, min(2000, int(pwm))), throttle=thr)
        t_hold = time.time() + hold
        while time.time() < t_hold:
            if abort.is_set():
                raise _Aborted
            s = _tick(link)                        # one FC tick: clock, rate, surface
            d = _elevator_frac(link, s)
            if s is not None and s[0] != last_stamp and d is not None:
                last_stamp = s[0]
                ts.append(s[0]); ds.append(d - delta0); qs.append(s[2])
            time.sleep(0.004)
    _rc(link, throttle=thr)
    if len(ts) < 40:
        return None
    t = np.array(ts); t -= t[0]
    return t, np.array(ds), np.array(qs)


def run_pitch_plant_id(ctrl, abort, out):
    """Identify the pitch plant from open-loop elevator doublets - the pitch stage
    of the unified plant-ID ('i' key).  Stands the balloon chaser cascade down, excites
    the elevator in MANUAL, and reports the airframe parameters, the honest
    feedforward and the physical overshoot floor.  Changes NO stored gains;
    hands back to LOITER through the controller's canonical teardown.

    Writes its doublet samples to the run's shared signals CSV (`out`) tagged
    'pitch', and its suggested PTCH_RATE_FF to the run's report params block;
    the run emits ONE CSV and ONE report across all axes (see run_plant_id).
    The shared abort Event ('i' again) is honoured by every phase."""
    link = ctrl.link
    log.info("=== Pitch plant-ID (open loop) ===")
    # The estimator must be sound before we spend air time.  A regression here
    # fails loudly rather than fitting garbage to real flight data.
    try:
        selftest_once()
    except AssertionError as e:                    # noqa: BLE001
        log.error("[PPID] estimator selftest FAILED (%s) - refusing to identify", e)
        ctrl.autotune_finish()
        ctrl._pitch_plantid_active = False
        return

    # Read before the aircraft is touched: a round trip of up to seconds is
    # not one to fly in MANUAL, or between the level-out and MANUAL.
    aspd_min = _airspeed_min(link)
    ctrl.autotune_status = 'Pitch Plant ID: starting'
    ctrl.autotune_pause_tracking()                 # the cascade stands down; the
                                                   # 50 Hz loop stops commanding
    # Clear the start-up transition latch now the run is underway, so a second
    # 'i' reaches the abort path instead of the transition guard.  This run has
    # no ptest_enter_acro (it flies MANUAL, not ACRO), which is what clears it
    # for 'i'/'k'.
    ctrl._tracking_transition = False
    fits = []
    try:
        _relevel(ctrl, abort)                      # wings-level, then MANUAL for the gate
        link._set_mode(MODE_MANUAL, "MANUAL")
        if not _directness_gate(ctrl, aspd_min):
            log.error("[PPID] directness gate FAILED - refusing to identify a distorted plant")
            return
        idx = 0
        for amp in PITCH_DOUBLET_AMPS_PWM:
            for _ in range(REPS_PER_AMP):
                ctrl.autotune_status = 'Pitch Plant ID: doublet %d (amp %d)' % (idx, amp)
                _relevel(ctrl, abort)
                rec = _fly_one_doublet(ctrl, amp, abort)
                if rec is None:
                    log.warning("[PPID] doublet %d (amp %d): too few samples, skipped", idx, amp)
                    idx += 1
                    continue
                t, d, q = rec
                out.write_samples('pitch', idx, t, d, q)
                e = fit_pitch_plant(t, d, q)
                log.info("[PPID] doublet %d (amp %d): M_delta=%.0f M_q=%.2f FF=%.4f "
                         "q_ss=%.0f R2=%.3f", idx, amp, e["M_delta"], e["M_q"],
                         e["FF_surface_per_rate"], e["q_ss"], e["R2"])
                if e["R2"] > 0.9:                  # only trust well-explained fits
                    fits.append(e)
                idx += 1
    except _Aborted:
        log.info("[PPID] stopped - ending the pitch stage")
    finally:
        # Canonical hand-back: tracking off, LOITER commanded and confirmed, RC
        # overrides released in a finally - or the hand-over in ACRO when another
        # mode key stopped the run (autotune_finish).  Clears the HUD banner and
        # label too.
        try:
            ctrl.autotune_finish()
        except Exception:                          # noqa: BLE001
            log.exception("[PPID] hand-back to LOITER failed - check the aircraft")
        ctrl._pitch_plantid_active = False

    if not fits:
        log.error("[PPID] no usable doublets - plant not identified (check the gate output)")
        return
    # A doublet whose M_delta rails at the fit ceiling was driven past the linear
    # short-period (the largest amplitudes reach 100+ deg/s), so its control power
    # is a clip, not a measurement.  The airframe fact comes from the linear-regime
    # doublets when at least two remain; the railed ones only flag the non-linearity.
    railed = [f for f in fits if f["M_delta"] >= 0.99 * _HI[0]]
    linear = [f for f in fits if f["M_delta"] < 0.99 * _HI[0]]
    use = linear if len(linear) >= 2 else fits
    Md = float(np.median([f["M_delta"] for f in use]))
    Mq = float(np.median([f["M_q"] for f in use]))
    FF = float(np.median([f["FF_surface_per_rate"] for f in use]))
    qss = float(np.median([f["q_ss"] for f in fits]))
    spread = float(np.std([f["M_delta"] for f in use]) / Md) if Md else float("nan")
    log.info("=== PITCH PLANT (median of %d doublets, spread %.0f%%) ===", len(use), 100 * spread)
    log.info("  control power M_delta = %.0f deg/s^2 per full elevator throw", Md)
    log.info("  pitch damping M_q     = %.2f /s", Mq)
    log.info("  feedforward  -M_q/M_d = %.4f throw per deg/s (the honest sustained-rate FF)", FF)
    for R in (10.0, 20.0):
        log.info("  overshoot floor for a %.0f deg/s arrival, full throw: %.1f deg",
                 R, overshoot_floor_deg(R, Md, 1.0))
    if railed and len(linear) >= 2:
        log.warning("  %d/%d doublets railed M_delta at the %.0f ceiling (amplitudes past the "
                    "linear short-period); excluded from the airframe estimate",
                    len(railed), len(fits), _HI[0])
    elif railed:
        log.warning("  %d/%d doublets railed M_delta at the %.0f ceiling; the linear-regime "
                    "control power may exceed it - re-run with smaller amplitudes, or raise the "
                    "ceiling only with evidence", len(railed), len(fits), _HI[0])
    if abs(qss) > SETTLE_MAX_Q_DPS:
        log.warning("  operating point q_ss = %.0f deg/s: the doublets were NOT flown from "
                    "trimmed level flight, so the plant is identified about that pitching "
                    "state.  Re-fly trimmed (wings level, q~0 between doublets) for a "
                    "measurement about the cruise condition.", qss)
    else:
        log.info("  operating point q_ss  = %.0f deg/s (trimmed entry)", qss)

    # Suggested ArduPlane pitch rate PID, DERIVED from the plant for the operator to
    # review and load by hand - nothing is applied.  The full set goes to the run's
    # single report (params block); the log lines here are captured into it.
    note = None
    if abs(qss) > SETTLE_MAX_Q_DPS:
        note = ("this run was untrimmed (q_ss=%.0f), so M_q - and thus the whole rate "
                "PID - is unreliable; re-fly trimmed before programming it." % qss)
    _emit_rate_pid('PTCH', Md, Mq, note, ctrl, out)
    log.info("  raw pitch doublets recorded to the run's signals CSV (axis 'pitch')")


# ---------------------------------------------------------------------------
# Yaw stage - the open-loop analog of the pitch measurement, on the RUDDER axis.
# Same estimator and wings-level settle; the axis-specific parts are the rudder
# surface (the DIFFERENTIAL of the two V-tail halves), the yaw rate (the FC's
# gyro z) and the yaw RC channel.  ArduPlane's direct yaw-rate controller
# (AP_YawController::get_rate_out, active with YAW_RATE_ENABLE=1) is the SAME
# AC_PID rate loop as pitch/roll, so this stage suggests the full YAW_RATE_* set,
# gated by that enable (with the default sideslip damper the gains are inert).
# ---------------------------------------------------------------------------
def _yaw_rate_dps(link):
    """Body yaw rate from the FC's gyro (deg/s), or None with no STATE2."""
    s = _tick(link)
    return None if s is None else s[3]


def _rudder_frac(link, s=None):
    """Rudder surface as a fraction of half-throw, from the V-tail DIFFERENTIAL
    of the servo signals the FC sent: pure elevator moves both halves together
    (differential 0), pure rudder moves them oppositely, so
    ((left-trim2) - (right-trim4))/1000 is the rudder part on the same half-throw
    scale _elevator_frac uses for the symmetric part.  None when absent."""
    s = _tick(link) if s is None else s
    if s is None or not s[5] or not s[6]:
        return None
    t2 = float(getattr(link, "_servo2_trim", 1500.0) or 1500.0)
    t4 = float(getattr(link, "_servo4_trim", 1500.0) or 1500.0)
    return ((float(s[5]) - t2) - (float(s[6]) - t4)) / 1000.0


def _directness_gate_yaw(ctrl, aspd_min):
    """Prove the rudder command reaches the surface and moves yaw before any
    measurement. Assumes MANUAL. Returns True/PASS."""
    from .arduplane_autotune import _rc
    link = ctrl.link
    thr = _op_throttle_pwm(ctrl)
    log.info("=== yaw directness self-test (open-loop) ===")
    ok = True
    r0 = _yaw_rate_dps(link)
    if r0 is None:
        log.error("  no FC gyro (STATE2) - is the FC script running? identification "
                  "impossible")
        return False
    _rc(link, yaw=PWM_MID + 150, throttle=thr)
    time.sleep(0.4)
    s = _tick(link)
    d = _rudder_frac(link, s)
    _rc(link, throttle=thr)
    if d is None:
        log.error("  no V-tail servo signal from the FC - identification impossible")
        return False
    log.info("  rudder step: surface frac %.2f, dr %.1f deg/s", d, (s[3] - r0))
    if abs(d) < 0.05:
        log.error("  rudder did not move for a yaw step - not in MANUAL / not commanding?"); ok = False
    if not _airspeed_clear(link, aspd_min):
        ok = False
    log.info("  yaw directness gate: %s", "PASS" if ok else "FAIL")
    return ok


def _fly_one_yaw_doublet(ctrl, amp_pwm, abort):
    """Fly one rudder doublet in MANUAL and record the FC-time-stamped
    (rudder_frac, yaw_rate) response. Same shape as _fly_one_doublet on the yaw
    axis. Returns (t, delta, r) arrays or None if too few samples."""
    from .arduplane_autotune import _rc
    link = ctrl.link
    thr = _op_throttle_pwm(ctrl)
    link._set_mode(MODE_MANUAL, "MANUAL")
    _rc(link, throttle=thr)
    time.sleep(0.3)
    ts, ds, rs = [], [], []
    last_stamp = None
    seq = [(PWM_MID + amp_pwm, DOUBLET_HOLD_S),
           (PWM_MID - amp_pwm, DOUBLET_HOLD_S),
           (PWM_MID, SAMPLE_SECONDS - 2 * DOUBLET_HOLD_S)]
    for pwm, hold in seq:
        _rc(link, yaw=pwm, throttle=thr)
        t_hold = time.time() + hold
        while time.time() < t_hold:
            if abort.is_set():
                raise _Aborted
            s = _tick(link)                        # one FC tick: clock, rate, surface
            d = _rudder_frac(link, s)
            if s is not None and s[0] != last_stamp and d is not None:
                last_stamp = s[0]
                ts.append(s[0]); ds.append(d); rs.append(s[3])
            time.sleep(0.004)
    _rc(link, throttle=thr)
    if len(ts) < 40:
        return None
    t = np.array(ts); t -= t[0]
    return t, np.array(ds), np.array(rs)


def run_yaw_plant_id(ctrl, abort, out):
    """Identify the yaw plant from open-loop rudder doublets - the yaw analog of
    run_pitch_plant_id.  Reports yaw control power, damping and the overshoot
    floor, and suggests the full ArduPlane yaw rate PID (YAW_RATE_*; used when
    YAW_RATE_ENABLE=1, ArduPlane's direct-rate controller being the same AC_PID
    loop as pitch/roll).  Changes no stored gains; hands back to LOITER through
    the canonical teardown.  Writes its samples to the run's shared signals CSV
    (`out`) tagged 'yaw', and its YAW_RATE_* to the run's report params block."""
    link = ctrl.link
    log.info("=== Yaw plant-ID (open loop) ===")
    try:
        selftest_once()
    except AssertionError as e:                    # noqa: BLE001
        log.error("[YPID] estimator selftest FAILED (%s) - refusing to identify", e)
        ctrl.autotune_finish()
        return
    # Read before the aircraft is touched: a round trip of up to seconds is
    # not one to fly in MANUAL, or between the level-out and MANUAL.
    aspd_min = _airspeed_min(link)
    ctrl.autotune_status = 'Yaw Plant ID: starting'
    ctrl.autotune_pause_tracking()
    ctrl._tracking_transition = False
    fits = []
    try:
        _relevel(ctrl, abort)
        link._set_mode(MODE_MANUAL, "MANUAL")
        if not _directness_gate_yaw(ctrl, aspd_min):
            log.error("[YPID] yaw directness gate FAILED - refusing to identify")
            return
        idx = 0
        for amp in YAW_DOUBLET_AMPS_PWM:
            for _ in range(REPS_PER_AMP):
                ctrl.autotune_status = 'Yaw Plant ID: doublet %d (amp %d)' % (idx, amp)
                _relevel(ctrl, abort)
                rec = _fly_one_yaw_doublet(ctrl, amp, abort)
                if rec is None:
                    log.warning("[YPID] doublet %d (amp %d): too few samples, skipped", idx, amp)
                    idx += 1; continue
                t, d, r = rec
                out.write_samples('yaw', idx, t, d, r)
                e = fit_pitch_plant(t, d, r)       # axis-agnostic short-period fit
                log.info("[YPID] doublet %d (amp %d): N_delta=%.0f N_r=%.2f r_ss=%.0f R2=%.3f",
                         idx, amp, e["M_delta"], e["M_q"], e["q_ss"], e["R2"])
                if e["R2"] > 0.9:
                    fits.append(e)
                idx += 1
    except _Aborted:
        log.info("[YPID] stopped - ending the yaw stage")
    finally:
        try:
            ctrl.autotune_finish()
        except Exception:                          # noqa: BLE001
            log.exception("[YPID] hand-back to LOITER failed - check the aircraft")
    if not fits:
        log.error("[YPID] no usable doublets - yaw plant not identified")
        return
    linear = [f for f in fits if f["M_delta"] < 0.99 * _HI[0]]
    use = linear if len(linear) >= 2 else fits
    Nd = float(np.median([f["M_delta"] for f in use]))
    Nr = float(np.median([f["M_q"] for f in use]))
    rss = float(np.median([f["q_ss"] for f in fits]))
    log.info("=== YAW PLANT (median of %d doublets) ===", len(use))
    log.info("  yaw control power N_delta = %.0f deg/s^2 per full rudder throw", Nd)
    log.info("  yaw damping       N_r     = %.2f /s", Nr)
    for R in (10.0, 20.0):
        log.info("  overshoot floor for a %.0f deg/s arrival, full throw: %.1f deg",
                 R, overshoot_floor_deg(R, Nd, 1.0))
    if abs(rss) > SETTLE_MAX_Q_DPS:
        log.warning("  operating point r_ss = %.0f deg/s: not flown from coordinated "
                    "flight; re-fly cleaner for a trustworthy yaw plant.", rss)

    # Suggested ArduPlane yaw rate PID.  Yaw's direct-rate controller
    # (AP_YawController::get_rate_out, active with YAW_RATE_ENABLE=1) is the SAME
    # AC_PID rate loop as pitch/roll and AUTOTUNE_YAW tunes it identically, so the
    # yaw stage emits the full YAW_RATE_* set - gated by that enable, which the
    # emitter states.  Yaw is a naturally-damped axis, so it uses AUTOTUNE's
    # pitch/yaw I formula (roll_axis=False).
    note = None
    if abs(rss) > SETTLE_MAX_Q_DPS:
        note = ("this run was not from coordinated flight (r_ss=%.0f), so N_r - and thus "
                "the rate PID - is unreliable; re-fly cleaner before programming it." % rss)
    _emit_rate_pid('YAW', Nd, Nr, note, ctrl, out, roll_axis=False)
    log.info("  raw yaw doublets recorded to the run's signals CSV (axis 'yaw')")


# ---------------------------------------------------------------------------
# Roll stage - the open-loop analog of the pitch measurement, on the AILERON
# axis.  Same estimator and wings-level settle; the axis-specific parts are the
# aileron surface (CH1/SERVO1, a single surface - not the V-tail), the roll rate
# (the FC's gyro x) and the roll RC channel.  ArduPlane's roll controller
# (AP_RollController) IS the same AP_FW_Controller/AC_PID rate structure as pitch,
# so this stage suggests the full roll rate PID (RLL_RATE_*), derived exactly as
# the pitch one is - roll only takes AUTOTUNE's roll I formula, I = min(P, FF).
# ---------------------------------------------------------------------------
def _roll_rate_dps(link):
    """Body roll rate from the FC's gyro (deg/s), or None with no STATE2."""
    s = _tick(link)
    return None if s is None else s[1]


def _aileron_frac(link, s=None):
    """Aileron surface as a fraction of half-throw, from the aileron servo
    signal the FC sent (STATE2 pwm_aileron).  Roll is a SINGLE surface (not the
    V-tail differential the elevator/rudder share), so this is the direct
    (servo1 - trim)/500 on the same half-throw scale the pitch and yaw fractions
    use.  None when absent."""
    s = _tick(link) if s is None else s
    if s is None:
        return None
    return _servo_frac(s[4], float(getattr(link, "_servo1_trim", 1500.0) or 1500.0))


def _directness_gate_roll(ctrl, aspd_min):
    """Prove the aileron command reaches the surface and moves roll before any
    measurement. Assumes MANUAL. Returns True/PASS."""
    from .arduplane_autotune import _rc
    link = ctrl.link
    thr = _op_throttle_pwm(ctrl)
    log.info("=== roll directness self-test (open-loop) ===")
    ok = True
    p0 = _roll_rate_dps(link)
    if p0 is None:
        log.error("  no FC gyro (STATE2) - is the FC script running? identification "
                  "impossible")
        return False
    _rc(link, roll=PWM_MID + 150, throttle=thr)
    time.sleep(0.4)
    s = _tick(link)
    d = _aileron_frac(link, s)
    _rc(link, throttle=thr)
    if d is None:
        log.error("  no aileron servo signal from the FC - identification impossible")
        return False
    log.info("  aileron step: surface frac %.2f, dp %.1f deg/s", d, (s[1] - p0))
    if abs(d) < 0.05:
        log.error("  aileron did not move for a roll step - not in MANUAL / not commanding?"); ok = False
    if not _airspeed_clear(link, aspd_min):
        ok = False
    log.info("  roll directness gate: %s", "PASS" if ok else "FAIL")
    return ok


# Roll trim gain.  Roll control power is several times the pitch axis's, so a
# doublet centred on mechanical-neutral aileron (PWM_MID) rides the steady roll
# the prop torque - and any aileron mis-trim - leaves the instant FBWA lets go:
# it surfaces as a large operating-point rate p_ss and, because the constant-
# offset model then has to explain a steady roll at zero input, corrupts the
# damping L_p.  Null the MEASURED roll rate to the aileron that holds it steady,
# exactly as _trim_pitch_manual does for the elevator.  The gain is a fraction of
# pitch's TRIM_KP_PWM_PER_DPS because roll answers that much harder per PWM; it is
# set conservative (below the deadbeat value) so the null converges monotonically
# even though L_p - the roll sensitivity it depends on - is the very quantity this
# run measures.  Sign: PWM_MID + x is right-roll aileron and raises the rate, so
# to null a rate p, move the PWM by -Kp*p (same convention as pitch).
ROLL_TRIM_KP_PWM_PER_DPS = 0.6


def _trim_roll_manual(ctrl, abort):
    """In MANUAL, return the aileron PWM that holds a steady (non-rolling) roll
    rate, so the doublet perturbs a settled p~0 operating point instead of the
    steady roll the airframe holds at neutral aileron.

    The roll analog of _trim_pitch_manual: a short proportional null on the
    MEASURED body roll rate finds the settled aileron without the
    RC->servo calibration an open-loop surface match would need, whose reversals
    make such a match fragile.  Bounded and self-limiting: a search that cannot
    null within the timeout returns the PWM that gave the smallest rate it saw, so
    even a poorly-behaved search hands back its best steady point rather than a
    runaway.  Returns PWM_MID when the FC sends no gyro, where there is no rate
    to null."""
    from .arduplane_autotune import _rc
    link = ctrl.link
    thr = _op_throttle_pwm(ctrl)
    pwm = float(PWM_MID)
    best_pwm, best_absp = float(PWM_MID), float("inf")
    t0 = time.time(); held_since = None
    while time.time() - t0 < TRIM_TIMEOUT_S:
        if abort.is_set():
            raise _Aborted
        _rc(link, roll=int(round(pwm)), throttle=thr)
        time.sleep(0.12)                              # let roll subsidence settle
        p = _roll_rate_dps(link)
        if p is None:
            return PWM_MID                            # no rate feed -> cannot trim
        if abs(p) < best_absp:
            best_absp, best_pwm = abs(p), pwm
        if abs(p) < TRIM_NULL_TOL_DPS:
            held_since = held_since if held_since is not None else time.time()
            if time.time() - held_since >= TRIM_HOLD_S:
                return int(round(pwm))
        else:
            held_since = None
            pwm -= ROLL_TRIM_KP_PWM_PER_DPS * p
            pwm = max(PWM_MID - TRIM_MAX_OFFSET_PWM,
                      min(PWM_MID + TRIM_MAX_OFFSET_PWM, pwm))
    log.info("[RPID] roll trim did not fully null within %.0fs - centring the "
             "doublet on the steadiest aileron found (residual %.0f deg/s)",
             TRIM_TIMEOUT_S, best_absp)
    return int(round(best_pwm))


def _fly_one_roll_doublet(ctrl, amp_pwm, abort):
    """Fly one aileron doublet in MANUAL and record the FC-time-stamped
    (aileron DEVIATION, roll_rate) response.  `delta` is the aileron deviation
    from the roll-trim that holds a steady roll rate (so it is 0 at the operating
    point, matching the model's delta=0 equilibrium), and p rides ~level - so the
    doublet perturbs a settled p~0 point rather than the steady roll the airframe
    holds at neutral aileron (see _trim_roll_manual).  Returns (t, delta, p)
    arrays or None if too few samples.

    Roll is a powerful axis: a large-amplitude open-loop aileron doublet can
    build a substantial bank inside the hold, which the wings-level settle
    (_relevel, FBWA) then removes before the next doublet.  A doublet driven past
    the linear roll-subsidence regime rails the fit's control power and is
    excluded downstream, exactly as on the pitch axis."""
    from .arduplane_autotune import _rc
    link = ctrl.link
    thr = _op_throttle_pwm(ctrl)
    link._set_mode(MODE_MANUAL, "MANUAL")
    trim_pwm = _trim_roll_manual(ctrl, abort)      # aileron that holds a steady roll rate
    # Nulling the roll RATE does not level the wings - roll is neutrally stable in
    # bank, so the search settles the rate at whatever bank it drifted to (and the
    # uncommanded elevator lets pitch drift too).  RE-LEVEL in FBWA so the maneuver
    # STARTS wings-level, then return to MANUAL and settle at the trimmed aileron -
    # still valid, since the brief level-out holds airspeed - so the doublet begins
    # both level AND steady.  Mirrors _fly_one_doublet.
    _relevel(ctrl, abort)
    link._set_mode(MODE_MANUAL, "MANUAL")
    _rc(link, roll=max(1000, min(2000, trim_pwm)), throttle=thr)
    time.sleep(0.5)                                # settle level at the trimmed aileron
                                                   # (also lets p re-null after the brief
                                                   # neutral-aileron MANUAL-entry transient)
    delta0 = _aileron_frac(link)                   # the trimmed aileron: the doublet's zero
    if delta0 is None:
        return None

    ts, ds, ps = [], [], []
    last_stamp = None
    seq = [(trim_pwm + amp_pwm, ROLL_DOUBLET_HOLD_S),
           (trim_pwm - amp_pwm, ROLL_DOUBLET_HOLD_S),
           (trim_pwm, SAMPLE_SECONDS - 2 * ROLL_DOUBLET_HOLD_S)]
    for pwm, hold in seq:
        _rc(link, roll=max(1000, min(2000, int(pwm))), throttle=thr)
        t_hold = time.time() + hold
        while time.time() < t_hold:
            if abort.is_set():
                raise _Aborted
            s = _tick(link)                        # one FC tick: clock, rate, surface
            d = _aileron_frac(link, s)
            if s is not None and s[0] != last_stamp and d is not None:
                last_stamp = s[0]
                ts.append(s[0]); ds.append(d - delta0); ps.append(s[1])
            time.sleep(0.004)
    _rc(link, throttle=thr)
    if len(ts) < 40:
        return None
    t = np.array(ts); t -= t[0]
    return t, np.array(ds), np.array(ps)


def run_roll_plant_id(ctrl, abort, out):
    """Identify the roll plant from open-loop aileron doublets - the roll analog
    of run_pitch_plant_id.  Reports roll control power, damping and the overshoot
    floor, and (because ArduPlane's roll controller shares the pitch rate PID
    structure) suggests the full roll rate PID (RLL_RATE_*).  Changes no stored
    gains; hands back to LOITER through the canonical teardown.  Writes its
    samples to the run's shared signals CSV (`out`) tagged 'roll', and its
    RLL_RATE_* to the run's report params block."""
    link = ctrl.link
    log.info("=== Roll plant-ID (open loop) ===")
    try:
        selftest_once()
    except AssertionError as e:                    # noqa: BLE001
        log.error("[RPID] estimator selftest FAILED (%s) - refusing to identify", e)
        ctrl.autotune_finish()
        return
    # Read before the aircraft is touched: a round trip of up to seconds is
    # not one to fly in MANUAL, or between the level-out and MANUAL.
    aspd_min = _airspeed_min(link)
    ctrl.autotune_status = 'Roll Plant ID: starting'
    ctrl.autotune_pause_tracking()
    ctrl._tracking_transition = False
    fits = []
    try:
        _relevel(ctrl, abort)
        link._set_mode(MODE_MANUAL, "MANUAL")
        if not _directness_gate_roll(ctrl, aspd_min):
            log.error("[RPID] roll directness gate FAILED - refusing to identify")
            return
        idx = 0
        for amp in ROLL_DOUBLET_AMPS_PWM:
            for _ in range(REPS_PER_AMP):
                ctrl.autotune_status = 'Roll Plant ID: doublet %d (amp %d)' % (idx, amp)
                _relevel(ctrl, abort)
                rec = _fly_one_roll_doublet(ctrl, amp, abort)
                if rec is None:
                    log.warning("[RPID] doublet %d (amp %d): too few samples, skipped", idx, amp)
                    idx += 1; continue
                t, d, p = rec
                out.write_samples('roll', idx, t, d, p)
                e = _fit_roll(t, d, p)             # escalating-ceiling short-period fit
                log.info("[RPID] doublet %d (amp %d): L_delta=%.0f L_p=%.2f FF=%.4f "
                         "p_ss=%.0f R2=%.3f%s", idx, amp, e["M_delta"], e["M_q"],
                         e["FF_surface_per_rate"], e["q_ss"], e["R2"],
                         (" [railed at cap %.0f]" % e["md_ceiling"]) if e["railed"] else "")
                if e["R2"] > 0.9:
                    fits.append(e)
                idx += 1
    except _Aborted:
        log.info("[RPID] stopped - ending the roll stage")
    finally:
        try:
            ctrl.autotune_finish()
        except Exception:                          # noqa: BLE001
            log.exception("[RPID] hand-back to LOITER failed - check the aircraft")
    if not fits:
        log.error("[RPID] no usable doublets - roll plant not identified")
        return
    railed = [f for f in fits if f.get("railed")]
    linear = [f for f in fits if not f.get("railed")]
    use = linear if len(linear) >= 2 else fits
    Ld = float(np.median([f["M_delta"] for f in use]))
    Lp = float(np.median([f["M_q"] for f in use]))
    pss = float(np.median([f["q_ss"] for f in fits]))
    log.info("=== ROLL PLANT (median of %d doublets) ===", len(use))
    log.info("  roll control power L_delta = %.0f deg/s^2 per full aileron throw", Ld)
    log.info("  roll damping       L_p     = %.2f /s", Lp)
    log.info("  feedforward  -L_p/L_delta  = %.4f throw per deg/s (the honest sustained-rate FF)",
             (-Lp / Ld) if Ld else float("nan"))
    for R in (30.0, 60.0):
        log.info("  overshoot floor for a %.0f deg/s arrival, full throw: %.1f deg",
                 R, overshoot_floor_deg(R, Ld, 1.0))
    if railed and len(linear) >= 2:
        log.warning("  %d/%d doublets saturated (still railing L_delta at the %.0f cap after "
                    "escalation - the roll rate ran non-linear within the hold); excluded from "
                    "the airframe estimate", len(railed), len(fits), ROLL_MD_CAP)
    elif railed:
        log.warning("  %d/%d doublets saturated (railing at the %.0f cap after escalation, i.e. "
                    "non-linear within the hold); the estimate rests on them for want of cleaner "
                    "doublets - re-fly with SMALLER amplitudes for a linear-regime roll plant",
                    len(railed), len(fits), ROLL_MD_CAP)
    if abs(pss) > SETTLE_MAX_Q_DPS:
        log.warning("  operating point p_ss = %.0f deg/s: not flown from wings-level flight; "
                    "re-fly cleaner for a trustworthy roll plant.", pss)

    # Suggested ArduPlane roll rate PID, DERIVED from the plant for the operator to
    # review and load by hand - nothing is applied.  ArduPlane's roll controller is
    # the same AC_PID rate loop as pitch, so roll uses the identical derivation;
    # roll is the aileron-damped axis, so it takes AUTOTUNE's roll I formula
    # (I = min(P, FF), roll_axis=True).
    note = None
    if abs(pss) > SETTLE_MAX_Q_DPS:
        note = ("this run was not wings-level (p_ss=%.0f), so L_p - and thus the rate "
                "PID - is unreliable; re-fly level before programming it." % pss)
    _emit_rate_pid('RLL', Ld, Lp, note, ctrl, out, roll_axis=True)
    log.info("  raw roll doublets recorded to the run's signals CSV (axis 'roll')")


# ---------------------------------------------------------------------------
# Unified plant-ID orchestrator ('i' key).  Runs the enabled axis stages in
# sequence, each self-contained (own flight, altitude protection and LOITER
# teardown), with a top-level checkpoint so a re-launch after a crash or a
# second 'i' resumes at the next unfinished axis.
# ---------------------------------------------------------------------------
_RUN_CKPT_NAME = "incomplete_plant_id_run"         # the run's single checkpoint
_RUN_CKPT_GLOB = "incomplete_plant_id_run_*.json"
_RUN_CKPT_VERSION = 1


def _run_ckpt_load(ctrl):
    """(run_ts, set(done_stages)) from a resumable unified-run checkpoint, or
    (None, set()).  A code change (fingerprint mismatch) discards it: the plant
    must be re-flown against the code that will fit it."""
    import glob, json, os
    found = sorted(glob.glob(os.path.join(os.getcwd(), _RUN_CKPT_GLOB)))
    if not found:
        return None, set()
    try:
        data = json.load(open(found[-1]))
    except (OSError, ValueError):
        return None, set()
    if data.get("version") != _RUN_CKPT_VERSION or data.get("fingerprint") != PACKAGE_SHA256:
        return None, set()
    return data.get("run_ts"), set(data.get("done") or [])


def _run_ckpt_save(run_ts, done, enabled):
    import json, os
    path = os.path.join(os.getcwd(), "%s_%s.json" % (_RUN_CKPT_NAME, run_ts))
    try:
        with open(path, "w") as f:
            json.dump({"version": _RUN_CKPT_VERSION, "fingerprint": PACKAGE_SHA256,
                       "run_ts": run_ts, "done": sorted(done),
                       "enabled": list(enabled)}, f, indent=2)
        for old in file_export.claim(_RUN_CKPT_GLOB, path):
            log.info("[PLANTID] cleared previous run checkpoint %s", old)
    except OSError as e:                               # noqa: BLE001
        log.warning("[PLANTID] could not save run checkpoint (%s)", e)


def _run_ckpt_clear():
    import glob, os
    for p in glob.glob(os.path.join(os.getcwd(), _RUN_CKPT_GLOB)):
        try:
            os.remove(p)
        except OSError:
            pass


class _LogCapture(logging.Handler):
    """Collects this run's log lines so the single report can carry them verbatim
    - the per-axis summaries, the doublet-by-doublet fits and any warnings all go
    to the console and would otherwise scroll away before anyone reads them."""

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


class _RunOutputs:
    """The ONE report + ONE signals CSV a plant-ID run emits.  The axis stages
    share a single instance, so a run produces one of each and its content
    reflects the axes that ran, not a file per axis - the tuning-run output
    convention in ARCHITECTURE.md.  Suggested FC feedforwards land in the report's
    params block, not in separate .param files.  (The third per-run file, the
    resume checkpoint, is owned by the _run_ckpt_* helpers.)"""

    def __init__(self, run_ts):
        self.run_ts = run_ts
        self.csv_path = file_export.stamped('plant_id_signals', 'csv', run_ts)
        self.report_path = file_export.stamped('plant_id_report', 'txt', run_ts)
        self._csv = None
        self.param_lines = []          # loadable "NAME,value" lines for the report

    def open_csv(self):
        try:
            self._csv = open(self.csv_path, 'w')
            self._csv.write('# code_sha256: %s\n' % PACKAGE_SHA256)
            self._csv.write('axis,doublet,t_s,delta_frac,rate_dps\n')
        except OSError as e:                                  # noqa: BLE001
            log.warning('[PLANTID] could not open the signals CSV (%s)', e)
            self._csv = None

    def write_samples(self, axis, idx, t, d, rate):
        """Append one doublet's samples, tagged with the axis, to the shared CSV.
        Delta is that axis's surface fraction, rate its body rate (deg/s)."""
        if self._csv is None:
            return
        try:
            for i in range(len(t)):
                self._csv.write('%s,%d,%.4f,%.4f,%.3f\n' % (axis, idx, t[i], d[i], rate[i]))
        except Exception:                                     # noqa: BLE001
            pass                                              # never break a run to log

    def add_param(self, name, value):
        """Record a suggested FC parameter for the report's paste-ready block."""
        self.param_lines.append('%s,%.4f' % (name, value))

    def finish(self, enabled, done, transcript):
        """Close the CSV and write the single report: a header, the paste-ready
        suggested-parameter block (whichever axes produced one), then the full run
        transcript so a per-axis result can be explained from this file alone."""
        import os
        if self._csv is not None:
            try:
                self._csv.close()
            except Exception:                                 # noqa: BLE001
                pass
            # The signals CSV is a COMPLETED-RUN output: it is timestamped and
            # KEPT, not claimed away, so earlier runs survive on disk for the
            # operator to compare or export.  Only the resume checkpoint is
            # cleaned (see _run_ckpt_save).
        L = ['PLANT-ID REPORT',
             'code_sha256 %s' % PACKAGE_SHA256,
             'axes requested: %s' % (', '.join(enabled) if enabled else '(none)'),
             'axes completed: %s' % (', '.join(sorted(done)) if done else '(none)'),
             'raw signals:    %s' % os.path.basename(self.csv_path),
             '']
        if self.param_lines:
            L += ['=' * 68,
                  'SUGGESTED ArduPlane RATE PID (REVIEW before loading; nothing applied)',
                  'The full per-axis rate loop. FF, I, IMAX and the filters reproduce',
                  'ArduPlane AUTOTUNE\'s own formulas; P and D are COMPUTED from the',
                  'identified plant for the sustained balloon chaser slew - not AUTOTUNE\'s',
                  'raise-until-ring doublet search, which drives P the wrong way (see the',
                  'transcript and Notes/autotune_mismatch_report.md). The aggressiveness',
                  'knob default reproduces the flight-validated PTCH_RATE_P=1.0; the',
                  'transcript carries its sensitivity table. YAW_RATE_* apply only with',
                  'YAW_RATE_ENABLE=1. The 2SRV_TCONST/RMAX attitude-loop values are design',
                  'defaults (in the transcript), not plant-derived.',
                  '']
            L += self.param_lines
            L.append('')
        L += ['=' * 68,
              'RUN TRANSCRIPT - the full console output, so a per-axis result',
              'missing from the block above can be explained from this file alone.',
              '']
        L += list(transcript)
        try:
            with open(self.report_path, 'w') as f:
                f.write('\n'.join(L) + '\n')
            # A completed run's report is KEPT, not claimed away: it is
            # timestamped, so runs accumulate rather than collide, and the
            # operator clears them with Delete when they choose.  Only the
            # resume checkpoint is cleaned.
            log.info('[PLANTID] report: %s  (Copy it out with the Plant-ID report button)',
                     os.path.basename(self.report_path))
        except OSError as e:                                  # noqa: BLE001
            log.warning('[PLANTID] could not write the report (%s)', e)


def run_plant_id(ctrl, abort):
    """Unified plant-ID ('i' key).  Runs the enabled axis stages - pitch, yaw,
    roll - back to back, each self-contained (own flight, altitude protection and
    hand-back to LOITER), recording a top-level checkpoint after each so a crash
    or a second 'i' resumes at the next unfinished axis.  Which axes run is set by
    the Gain-Tuner Do Pitch / Do Yaw / Do Roll switches.  All three are the same
    open-loop MANUAL doublet measurement, one per axis.

    The run emits exactly THREE files whatever axes ran (the tuning-run output
    convention, ARCHITECTURE.md): one report, one signals CSV (both owned by
    `out`, shared across the stages), and one resume checkpoint.  Altitude
    protection and the per-axis flight live in the stage runners; this sequences
    them, records progress, and writes the shared report at the end."""
    stages = [("pitch", "plantid_do_pitch", run_pitch_plant_id),
              ("yaw",   "plantid_do_yaw",   run_yaw_plant_id),
              ("roll",  "plantid_do_roll",  run_roll_plant_id)]
    enabled = [name for name, attr, _ in stages if int(getattr(ctrl, attr, 1) or 0)]
    ckpt_ts, done = _run_ckpt_load(ctrl)
    run_ts = ckpt_ts or ctrl.run_stamp_for('i')
    log.info("=== Plant-ID run: axes %s%s ===",
             enabled or "(none - enable Do Pitch/Yaw/Roll in the tuner)",
             ("  (resuming; %s already done)" % sorted(done)) if done else "")
    cap = _LogCapture()
    log.addHandler(cap)                                # capture the run for the report
    out = _RunOutputs(run_ts)
    out.open_csv()
    ran = False
    try:
        for name, attr, runner in stages:
            if name not in enabled or name in done:
                continue
            if abort.is_set():
                break
            log.info("[PLANTID] === %s stage (%d of %d) ===",
                     name, enabled.index(name) + 1, len(enabled))
            ctrl._mode = 'autotune'                    # each stage flies the sweep's mode regime
            ctrl._mode_label = 'Plant ID: %s' % name
            ran = True
            runner(ctrl, abort, out)                   # sets up, flies, hands back
            if abort.is_set():
                log.info("[PLANTID] aborted during the %s stage", name)
                break
            done.add(name)
            _run_ckpt_save(run_ts, done, enabled)
    finally:
        log.removeHandler(cap)                         # freeze the transcript before the report
        out.finish(enabled, done, cap.lines)           # the run's single report + CSV
        if not enabled or set(enabled) <= done:
            _run_ckpt_clear()                          # nothing to do, or all stages finished
        # Each stage hands back as it ends.  With none flown - nothing enabled,
        # everything already done, or stopped before the first - the run still
        # must: toggle_plant_id set the 'autotune' mode and the start-up
        # transition, and only a hand-back clears them.
        if not ran:
            try:
                ctrl.autotune_finish()
            except Exception:                          # noqa: BLE001
                log.exception("[PLANTID] hand-back failed - check the aircraft")
        ctrl._mode_label = None
    if enabled and set(enabled) <= done:
        log.info("[PLANTID] run complete: %s", sorted(done))


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    selftest()
    print("plant_id selftest passed.")
