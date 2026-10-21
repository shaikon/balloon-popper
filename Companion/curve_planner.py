"""Curve Planner (companion): the soonest turn that brings the flight path onto the
balloon, under the rules of a chase (CLAUDE.md).

It plans only the TURN: the curve ends when the flight path points within
CURVE_TURN_DONE_DEG of the balloon, and from there the steering law flies the
chase as it flies one without a PLAN.  The PLAN it makes is run on the FC by
the Curve Flyer (LUA/curve_flyer.lua) and flown by the steering law selected
(steering_law), in that law's PLAN MODE.

A plan is a BANK SCHEDULE: CURVE_N_KNOTS banks, each tied to how far through
the turn the aircraft is (progress 0 at the start, 1 when the path points at
the balloon), in the law's own measure of the bank - the bank proxy (the wing
tip's angle below the horizon) under the cascade, the bank round the path
under Lift Vector Pointing and its versions P and N, which fly its plan mode
(a PLAN made for one of the three is labelled with it, and the FC takes it
only under that law).  The Curve Flyer works out the progress every tick,
1 - (angle off the balloon now) / (angle off at the start), and hands the law
the schedule's bank there; the law flies it with its own roll, pitch, rudder
and bank limit, its pitch turning the path toward the balloon as it always
does (cascade.lua, path_attitude.lua).  Progress, not time, indexes the
schedule: an aircraft that turns slower than planned still meets each bank at
the point of the turn it was planned for.

THE PLANNER FLIES EVERY CANDIDATE THROUGH A MODEL OF THE LAW'S PLAN MODE AND OF
THE CURVE FLYER (fly(), PlanMode), from the same constants and gains: a change
to either law's plan mode or to the Curve Flyer is a change here in the same
commit, or the FC flies turns the planner never made.

The aircraft is a point mass with its nose on its flight path, steered by a
roll rate about the path and a pitch rate turning the path toward the canopy,
each following its command after a first-order lag; gravity across the wings
turns the path sideways, as it does in any banked flight.  Its speed follows a fit
to SITL (a climb costs speed, a dive gains it).  The load is whatever the
pitch rate takes; the rules set no limit on it.

The planner chooses the schedule that completes the turn soonest, the least
roll breaking a tie, subject to:
  - at least ALT_MIN_M above the ground and SPEED_MIN_MPS fast throughout;
  - where rule 1 or 2 binds, no Suboptimal Curve: the bank proxy within the
    bank limit (bank_lim_deg either way, or the starting proxy where that is
    beyond it), never further upside down than at the start, as the rules
    of a chase define it (the proxy reads level wings upside down as no
    bank), the tilt never coming back by more than the Tilt Back Travel, and
    for SETTLE_S after the turn, the law still aiming at the balloon, the
    heading within CURVE_TURN_DONE_DEG of the turn end's;
  - for SETTLE_S after the turn, the path still within CURVE_TURN_DONE_DEG of the
    balloon: the turn arrives on the line of sight rather than swinging
    through it, so what takes over has nothing left to turn.
A turn that completes beats every turn that does not, whatever it breaks: a
missed balloon is the worst result (NEVER_DONE_S).
With Transfer Control When Done off (cf_transfer_when_done) the law is aimed at
the line the path had when the turn completed, banked toward it by
CF_HOLD_BANK_PER_DEG per degree it lies across the wings; for SETTLE_S the
model flies that hold, as the FC does.  On, the law flies the chase without
the PLAN, and the model keeps its plan mode aiming at the balloon in its place.
Every bank is flown within the law's own bank limit.

THE APPROACH ANGLE (cf_approach_deg, CF_APPROACH_DEG).  A balloon that starts
below the horizon, less far below than the Approach Angle, is flown with a
RUN-IN, so the turn ends diving at that angle onto it:
  - the scheduled turn (the LINE-UP) brings the path onto the run-in line: the
    run-in climb above the horizon, toward the balloon's compass direction,
    which moves as the aircraft does;
  - the RUN-IN holds that line, as the hold holds its own, until the balloon
    sits the PLAN's run-in depression below the horizon;
  - the DIVE then turns the path onto the balloon, banked toward it by
    CF_HOLD_BANK_PER_DEG per degree it lies across the wings; the turn is
    complete when the path points at the balloon.
The run-in climb is level, or where rule 1 binds the starting climb if that
is a dive: levelling it would bring the tilt back.  Where rule 2 binds, the
run-in and dive would bring it back too, and the chase is flown without one.
The planner plans the line-up as the turn onto the run-in line, then the
run-in depression that ends the turn soonest with the line from the aircraft
to the balloon at the Approach Angle, each degree off it costing
APPROACH_PENALTY_PER_DEG.  The run-in needs the balloon's range: with none
known (x) it is planned with the balloon on the ground along its line of
sight, the farthest it can be, and a nearer balloon ends steeper.

World frame NED (north, east, down), metres and seconds.
"""

import math
import os
import pickle
import subprocess
import sys
import threading

import numpy as np

from . import constants as C

G = C.GRAVITY_MPS2
SPEED0_MPS = 19.8          # the p test's entry speed in SITL, for planning a chase offline
# The speed along the path, fitted to SITL p-test flights at 85% throttle:
#   dV/dt = SPEED_GRAV_MPS2 * sin(dive) + THRUST_MPS2 - DRAG_PER_V2 * V^2
# which holds 20.3 m/s level and about 23 m/s in a 70 deg dive.
SPEED_GRAV_MPS2 = 9.5
THRUST_MPS2 = 31.0
DRAG_PER_V2 = 0.075
SPEED_MIN_MPS = 15.0       # below it the SITL airframe's wing cannot hold level flight
START_ALT_M = 100.0        # height above the ground of an offline chase's start (placeholder)
ALT_MIN_M = 30.0           # the route stays at least this high (placeholder)
STRAIGHT_TOL_DEG = C.CURVE_TURN_DONE_DEG
N_KNOTS = C.CURVE_N_KNOTS
SETTLE_S = 0.5             # how long after the turn the plan is checked against the rules
# The rule decision's slices (rule_binds; constants.py says what they are).
RULE_PLANE_DEG = C.RULE_PLANE_DEG
RULE_POLE_DEG = C.RULE_POLE_DEG
ROLL_LAG_S, PITCH_LAG_S = 0.12, 0.25     # the closed-loop test aircraft's lags
# How far ahead the planner flies a candidate's turn, and a dive's, s: one not
# complete by then is scored as one that never completes.  The FC flies a PLAN
# until its turn completes, however long that takes, so this bounds only the
# search.
TURN_HORIZON_S = 8.0
DT_PLAN_S = 0.02           # the planner's integration step (the FC's tick)
DT_CHECK_S = 0.002         # the check's integration step
# The steering laws (steering_law), each with the PLAN MODE a PLAN is flown in:
# the cascade (LUA/cascade.lua), and Lift Vector Pointing (LUA/path_attitude.lua)
# for itself and its versions P and N.
LAW_CASCADE, LAW_LVP = C.LAW_CASCADE, C.LAW_LVP
LAW_LVP_P, LAW_LVP_N = C.LAW_LVP_P, C.LAW_LVP_N
PLAN_MODE = {LAW_CASCADE: LAW_CASCADE, LAW_LVP: LAW_LVP,
             LAW_LVP_P: LAW_LVP, LAW_LVP_N: LAW_LVP}
# The gains each law's plan mode reads, by their ControlGains names.  A Chase
# takes them from the set the law flies; one not given is its constant's
# default.
_LVP_PLAN_GAINS = ('bank_lim_deg', 'att_k_path', 'att_path_rate_max_dps', 'att_lead_s',
                   'att_k_roll', 'att_roll_ki',
                   'att_pitch_ki', 'att_i_zone_deg')
PLAN_GAINS = {
    LAW_CASCADE: ('bank_lim_deg', 'roll_gain', 'pitch_gain', 'pitch_ki',
                  'windup_limit_pitch', 'pitch_deadband_deg', 'pitch_kd',
                  'pitch_kd_rate', 'pitch_turn_ff'),
    LAW_LVP: _LVP_PLAN_GAINS,
    LAW_LVP_P: _LVP_PLAN_GAINS,
    LAW_LVP_N: _LVP_PLAN_GAINS,
}
# control_pitch.lua's own floors: a pitch Ki at or below PITCH_KI_OFF is no
# integrator, a Kd at or below PITCH_KD_OFF no derivative, and a step no longer
# than DT_MIN_S has no rate to differentiate over.
PITCH_KI_OFF = 1e-6
PITCH_KD_OFF = 1e-9
DT_MIN_S = 1e-6
# path_attitude.lua's: below this a direction across the path has no length.
DIR_MIN = 1e-9
# Within CURVE_POLE_FADE_DEG of vertical the bank against the horizon has no
# direction: world up's part across the path is shorter than POLE_SIN, and the
# path's angle to the horizon has a tangent past TAN_CAP.
POLE_SIN = math.sin(math.radians(C.CURVE_POLE_FADE_DEG))
TAN_CAP = math.tan(math.radians(90.0 - C.CURVE_POLE_FADE_DEG))
# The search (plan()).  A batch costs about the same however many plans it
# flies - the time goes to the step loop - so the search runs few, wide
# batches: the planning time is time the aircraft holds straight.
#   Stage 1: every bank in the law's range, PLAN_SCAN_STEP_DEG apart, held from
#   the start and ramped to from the starting bank, flown at the coarser
#   PLAN_SCAN_DT_S: it only ranks the valleys.  Where a rule binds the bank, it
#   also flies every reversal - one bank to the turn's midpoint, another after -
#   with banks PLAN_SWITCH_STEP_DEG apart.  The best plan of each of the
#   PLAN_VALLEYS best valleys, valleys PLAN_VALLEY_SEP_DEG of bank apart at
#   least, goes on to stage 2.
#   Stage 2: PLAN_REFINE_ROUNDS rounds, each one batch: every knot of each plan
#   stepped up and down by its step, and PLAN_REFINE_RANDOM random nudges of
#   the same size; a plan takes the best that beats it, and its steps halve
#   when none does.  Steps start at PLAN_REFINE_KNOT_STEP_DEG.
PLAN_SCAN_STEP_DEG = 5.0
PLAN_SCAN_DT_S = 0.04
PLAN_SWITCH_STEP_DEG = 15.0
PLAN_VALLEYS = 3
PLAN_VALLEY_SEP_DEG = 15.0
PLAN_REFINE_ROUNDS = 5
PLAN_REFINE_RANDOM = 64
PLAN_REFINE_KNOT_STEP_DEG = 10.0
# The run-in's search (plan_dive()), on the line-up plan_turn() chose: every
# run-in depression from the balloon's starting depression to the Approach
# Angle, PLAN_DEP_STEP_DEG apart, flown at PLAN_SCAN_DT_S; the best is
# polished as stage 2 polishes a turn, its step starting at PLAN_DEP_STEP_DEG.
PLAN_DEP_STEP_DEG = 2.0
# How often, in steps, a batch in flight lowers its cap to the cost of its
# best finished plan (fly()).
CAP_EVERY_STEPS = 5
PENALTY_PER_UNIT = 100.0   # cost, in seconds, of each deg, m or m/s a plan breaks a limit by
# Cost, in seconds, of each deg the turn's end lies off the Approach Angle:
# more than any turn takes, so the plan meets it wherever it can, and less
# than PENALTY_PER_UNIT, so where it cannot meet it without breaking a rule
# of a chase the rule wins.
APPROACH_PENALTY_PER_DEG = 10.0
# The furthest the line to a balloon below the horizon can end from the
# Approach Angle, both between level and straight down: what a turn that
# never ends is charged, so it costs more than any that does.
APPROACH_MISS_MAX_DEG = 90.0
# What a turn that never completes is charged on top, s.  A missed balloon is
# the worst result (CLAUDE.md "What the competition judges"), so it costs more
# than any turn that completes, whatever rule that turn breaks: past
# PENALTY_PER_UNIT on every degree, metre and m/s any flight can break its
# limits by.
NEVER_DONE_S = 1e6
# A tie-breaker, not a trade: among turns equally soon, the one that rolls
# least.  100 deg of roll travel costs a hundredth of a second.
ROLL_TRAVEL_S_PER_DEG = 1e-4
# The integration step while every plan in a batch is on its run-in, settled
# on the line - within RUNIN_CALM_DEG of it and rolling slower than
# RUNIN_CALM_DPS - and further from its dive than two steps can bring it: a
# run-in is most of a long chase's flight, and on it the commands barely move.
DT_RUNIN_S = 0.1
RUNIN_CALM_DEG = 1.0
RUNIN_CALM_DPS = 2.0
# A plan's phase in fly(), as the Curve Flyer's states: the scheduled turn
# (CF_STATE_FLY), the run-in (CF_STATE_RUNIN), the dive (CF_STATE_DIVE), and
# after the turn is complete.
FLY, RUNIN, DIVE, AFTER = 0, 1, 2, 3
UP = np.array([0.0, 0.0, -1.0])


def cross3(a, b):
    """a x b over the last axis.  Written out because np.cross's set-up costs
    more than the arithmetic on the planner's small batches, every step."""
    a0, a1, a2 = a[..., 0], a[..., 1], a[..., 2]
    b0, b1, b2 = b[..., 0], b[..., 1], b[..., 2]
    return np.stack((a1 * b2 - a2 * b1, a2 * b0 - a0 * b2, a0 * b1 - a1 * b0), axis=-1)


def norm3(a, keepdims=False):
    """|a| over the last axis, as cross3 and for the same reason."""
    n = np.sqrt(a[..., 0] * a[..., 0] + a[..., 1] * a[..., 1] + a[..., 2] * a[..., 2])
    return n[..., None] if keepdims else n


def unit(a):
    return a / norm3(a, True)


def across(a, v):
    return a - np.sum(a * v, axis=-1, keepdims=True) * v


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def start_state(climb_deg, bank_deg, heading_deg=0.0):
    """Path direction v and canopy direction c of a start."""
    cl, hd = math.radians(climb_deg), math.radians(heading_deg)
    v = np.array([math.cos(cl) * math.cos(hd), math.cos(cl) * math.sin(hd), -math.sin(cl)])
    c = unit(across(UP, v))
    y = cross3(v, c)                          # right wing
    b = math.radians(bank_deg)
    return v, unit(c * math.cos(b) + y * math.sin(b))


def bank_deg(v, c):
    """Bank against the horizon round the path, right wing down positive, and
    |up across v|."""
    u = across(UP, v)
    um = norm3(u)
    cu = np.sum(c * u, axis=-1)
    cs = np.sum(c * cross3(v, UP * np.ones_like(v)), axis=-1)   # um sin b
    b = np.degrees(np.arctan2(cs, cu))
    return b, um


def proxy_deg(v, c):
    """The bank proxy: the wing tip's angle below the horizon, world down along
    the right wing v x c, deg - the bank the cascade feeds back and both laws
    hold the bank limit on."""
    return np.degrees(np.arcsin(np.clip(cross3(v, c)[..., 2], -1.0, 1.0)))


def rule_binds(v, c, b_dir):
    """The planner's rule decision: 1 for a balloon below the path, 2 above
    it, 0 neither (RULE_PLANE_DEG, RULE_POLE_DEG)."""
    eps = math.degrees(math.acos(np.clip(np.dot(v, b_dir), -1, 1)))
    if eps >= 90.0:
        return 0
    u = unit(across(UP, v))
    e = across(b_dir, v)
    if norm3(e) < 1e-9:
        return 0
    e = unit(e)
    clock_up = math.degrees(math.acos(np.clip(np.dot(e, u), -1, 1)))   # 0 straight up
    if clock_up <= RULE_PLANE_DEG:
        rule, pole = 2, UP
    elif clock_up >= 180.0 - RULE_PLANE_DEG:
        rule, pole = 1, -UP
    else:
        return 0
    # through the pole: the great circle passes within RULE_POLE_DEG of it before the balloon
    pv, pe = np.dot(pole, v), np.dot(pole, e)
    s_near = math.degrees(math.atan2(pe, pv))
    d_near = math.degrees(math.acos(min(1.0, math.hypot(pv, pe))))
    if 0.0 <= s_near <= eps and d_near < RULE_POLE_DEG:
        return 0
    return rule


class Chase:
    """One chase to plan: the aircraft's path and canopy directions, height and
    speed at the start, the balloon's direction and range from it, and the
    settings the plan is made under - the steering law that flies it (law, as
    steering_law) and the gains its plan mode reads (gains, PLAN_GAINS names,
    as the set the law flies), the operator's Tilt Back Travel, Transfer
    Control When Done and Approach Angle, and the FC's ACRO rate limits.
    runin_climb_deg is the run-in line's climb for a chase flown with a
    run-in, None for one without; turn0_deg is the scheduled turn's angle off
    the balloon at the start - the run-in line, or the balloon."""

    def __init__(self, climb_deg, bank_deg0, balloon, rng=math.inf, heading_deg=0.0,
                 rule=None, alt_m=START_ALT_M, speed_mps=SPEED0_MPS, **settings):
        v0, c0 = start_state(climb_deg, bank_deg0, heading_deg)
        self._setup(v0, c0, balloon, rng, rule, alt_m, speed_mps, **settings)

    @classmethod
    def from_state(cls, v0, c0, balloon, rng=math.inf, alt_m=START_ALT_M,
                   speed_mps=SPEED0_MPS, **settings):
        """A chase from the aircraft's state as the FC reads it: v0 the flight
        path and c0 the canopy direction, world NED."""
        self = cls.__new__(cls)
        v0 = unit(np.asarray(v0, float))
        c0 = unit(across(np.asarray(c0, float), v0))
        self._setup(v0, c0, balloon, rng, None, alt_m, speed_mps, **settings)
        return self

    def _setup(self, v0, c0, balloon, rng, rule, alt_m, speed_mps,
               law=C.STEERING_LAW, gains=None, tilt_back_deg=C.TILT_BACK_TRAVEL_DEG,
               cf_transfer_when_done=C.CF_TRANSFER_WHEN_DONE,
               approach_deg=C.CF_APPROACH_DEG,
               roll_max_dps=C.ACRO_ROLL_RATE_DEFAULT,
               pitch_max_dps=C.ACRO_PITCH_RATE_DEFAULT):
        self.v0, self.c0 = v0, c0
        self.law = int(law)
        if self.law not in PLAN_GAINS:
            raise ValueError(f"no plan mode for steering law {law!r}")
        gains = dict(gains or {})
        names = set(PLAN_GAINS[LAW_CASCADE]) | set(PLAN_GAINS[LAW_LVP])
        unknown = set(gains) - names
        if unknown:
            raise ValueError(f"no plan mode reads {sorted(unknown)}")
        for name in sorted(names):
            setattr(self, name, float(gains.get(name, getattr(C, name.upper()))))
        self.tilt_back_deg = tilt_back_deg
        self.transfer = bool(cf_transfer_when_done)
        self.approach_deg = approach_deg
        self.roll_max_dps = roll_max_dps
        self.pitch_max_dps = pitch_max_dps
        self.alt0 = alt_m
        self.speed0 = speed_mps
        self.b_dir = unit(np.asarray(balloon, float))
        # rule: None reads it from the geometry; 1 or 2 holds the chase to it
        # (the default p balloon sits just outside RULE_PLANE_DEG and is still
        # judged by rule 1)
        self.rule = rule_binds(self.v0, self.c0, self.b_dir) if rule is None else rule
        # The bank limit's band, on the bank proxy, widened to a starting proxy
        # beyond it, as Lift Vector Pointing decides its band on the chase's
        # first tick; and the starting bank in the law's own measure.
        self.proxy0 = float(proxy_deg(v0, c0))
        self.lo = min(-self.bank_lim_deg, self.proxy0)
        self.hi = max(self.bank_lim_deg, self.proxy0)
        self.bank0 = (self.proxy0 if self.law == LAW_CASCADE
                      else float(bank_deg(v0[None], c0[None])[0][0]))
        # The canopy's angle below the horizon at the start, deg: past 0 the
        # aircraft is upside down.  It may never go further below than at the
        # start, which for a start the right way up is not below at all.
        self.down0_deg = math.degrees(math.asin(np.clip(c0[2], -1.0, 1.0)))
        self.down_allow = max(0.0, self.down0_deg)
        self.off0_deg = math.degrees(math.acos(np.clip(np.dot(self.v0, self.b_dir), -1, 1)))
        # the tilt travels toward the balloon's side of the path: up when the
        # balloon is above the path's tilt, down when below
        self.tilt_dir = 1.0 if -self.b_dir[2] >= -self.v0[2] else -1.0
        # The run-in: for a balloon below the horizon, shallower than the
        # Approach Angle, unless rule 2 binds.  Its line is level, or the
        # starting dive where rule 1 binds.
        self.dep0_deg = math.degrees(math.asin(np.clip(self.b_dir[2], -1.0, 1.0)))
        climb0 = math.degrees(math.asin(np.clip(-self.v0[2], -1.0, 1.0)))
        self.runin_climb_deg = None
        if 0.0 < self.dep0_deg < approach_deg and self.rule != 2:
            self.runin_climb_deg = min(climb0, 0.0) if self.rule == 1 else 0.0
            if math.isinf(rng):
                rng = alt_m / math.sin(math.radians(self.dep0_deg))
        self.rng = rng
        self.balloon_pos = None if math.isinf(rng) else self.b_dir * rng
        self.turn0_deg = self.off0_deg
        if self.runin_climb_deg is not None:
            line0 = runin_line(self, self.b_dir[None], self.v0[None])[0]
            self.turn0_deg = math.degrees(math.acos(np.clip(np.dot(self.v0, line0), -1, 1)))
            # How far ahead a plan with a run-in is flown: each turn to
            # TURN_HORIZON_S, the run-in across the whole distance to the
            # balloon at the slowest speed a plan may fly.
            self.t_cap0 = (2.0 * TURN_HORIZON_S
                           + math.hypot(self.balloon_pos[0], self.balloon_pos[1]) / SPEED_MIN_MPS)


def knot_range(chase):
    """The banks a plan's knots are searched within, in the law's measure: the
    cascade's own bank limit on the proxy, which it clips its setpoint to, and
    every bank round the path for Lift Vector Pointing, whose band holds the
    limit on the proxy wherever the path is."""
    if chase.law == LAW_CASCADE:
        return -chase.bank_lim_deg, chase.bank_lim_deg
    return -180.0, 180.0


def bank_room(bank, um, band_lo, band_hi):
    """path_attitude.lua's bank_room on the planner's aircraft, whose nose is on
    its path: there the bank proxy's sine after a roll that takes the bank
    round the path to t is um * sin(t), and the aircraft is upside down, as the
    rules of a chase define it, past 90 deg of bank round the path either way.
    How far the roll may go from `bank` (deg) each way before the proxy leaves
    [band_lo, band_hi] or the aircraft is upside down: (lo, hi), deg, -180 and
    180 where nothing limits it."""
    live = um > DIR_MIN
    ks = np.where(live, um, 1.0)
    s_hi = math.sin(math.radians(band_hi)) / ks
    s_lo = math.sin(math.radians(band_lo)) / ks
    h_hi = np.where(live & (np.abs(s_hi) < 1.0),
                    90.0 - np.degrees(np.arcsin(np.clip(s_hi, -1.0, 1.0))), 0.0)
    h_lo = np.where(live & (np.abs(s_lo) < 1.0),
                    90.0 + np.degrees(np.arcsin(np.clip(s_lo, -1.0, 1.0))), 0.0)
    arcs = h_hi + h_lo > 0.0
    gap = np.where((h_hi > 0.0) & (h_lo > 0.0), 180.0 - h_hi - h_lo, 360.0 - 2.0 * (h_hi + h_lo))
    r_hi, r_lo = wrap180(bank - 90.0), wrap180(bank + 90.0)
    in_hi = np.abs(r_hi) < h_hi
    in_lo = ~in_hi & (np.abs(r_lo) < h_lo)

    def leave(h, r):
        # banked inside an arc: out by its nearer end, and no further than the
        # room past it
        e_p, e_n = h - r, h + r
        return (np.where(r >= 0.0, e_p, np.maximum(-180.0, -e_n - gap)),
                np.where(r >= 0.0, np.minimum(180.0, e_p + gap), -e_n))

    lo_h, hi_h = leave(h_hi, r_hi)
    lo_l, hi_l = leave(h_lo, r_lo)
    # outside both: up to the nearest arc's near end, each way
    hi_f = np.minimum(180.0, np.minimum(np.where(h_hi > 0.0, (-r_hi - h_hi) % 360.0, 180.0),
                                        np.where(h_lo > 0.0, (-r_lo - h_lo) % 360.0, 180.0)))
    lo_f = np.maximum(-180.0, np.maximum(np.where(h_hi > 0.0, -((r_hi - h_hi) % 360.0), -180.0),
                                         np.where(h_lo > 0.0, -((r_lo - h_lo) % 360.0), -180.0)))
    lo = np.where(arcs, np.where(in_hi, lo_h, np.where(in_lo, lo_l, lo_f)), -180.0)
    hi = np.where(arcs, np.where(in_hi, hi_h, np.where(in_lo, hi_l, hi_f)), 180.0)
    # upside down (upside_room): a bank past 90 may only roll back toward up;
    # within CURVE_POLE_FADE_DEG of vertical nothing is cut
    cut = um > POLE_SIN
    up_lo = np.where(bank < -90.0, 0.0, -90.0 - bank)
    up_hi = np.where(bank > 90.0, 0.0, 90.0 - bank)
    return (np.where(cut, np.maximum(lo, up_lo), lo),
            np.where(cut, np.minimum(hi, up_hi), hi))


class PlanMode:
    """The selected law's PLAN MODE for a batch of plans, on the planner's
    aircraft, whose nose is on its path, so the path frame is the body frame:
    x' the path v, y' the right wing v x c, z' the belly -c.  It carries what
    the law carries between ticks - the cascade's pitch integrator and
    derivative, Lift Vector Pointing's two integrators - each (B, 1)."""

    _STATE = ('pitch_I', 'pitch_prev', 'pitch_D', 'pitch_seeded', 'roll_i', 'pitch_i')

    def __init__(self, B):
        z = np.zeros((B, 1))
        self.pitch_I, self.pitch_prev, self.pitch_D = z.copy(), z.copy(), z.copy()
        self.pitch_seeded = np.zeros((B, 1), bool)
        self.roll_i, self.pitch_i = z.copy(), z.copy()

    def copy(self):
        out = PlanMode.__new__(PlanMode)
        for k in self._STATE:
            setattr(out, k, getattr(self, k).copy())
        return out

    @classmethod
    def stack(cls, modes):
        """One batch from the first plan of each of `modes`."""
        out = cls.__new__(cls)
        for k in cls._STATE:
            setattr(out, k, np.vstack([getattr(m, k)[:1] for m in modes]))
        return out

    def re_aim(self, changed):
        """The aim point changed between the balloon and a line on these plans
        (B,): the cascade's pitch derivative re-seeds, as the FC's does."""
        self.pitch_seeded = self.pitch_seeded & ~changed[:, None]

    def commands(self, chase, v, c, qa, sp, bd, b_ref, dt):
        """The roll and pitch rates, rad/s, the law commands toward balloon direction bd
        (B, 3) at the PLAN's bank b_ref (B, 1), deg in its own measure, after a
        tick of dt since the last."""
        if PLAN_MODE[chase.law] == LAW_LVP:
            return self._lvp(chase, v, c, qa, sp, bd, b_ref, dt)
        return self._cascade(chase, v, c, sp, bd, b_ref, dt)

    def _cascade(self, chase, v, c, sp, bd, b_ref, dt):
        # cascade.lua: the PLAN's bank within the limit is the roll setpoint
        # against the bank proxy; the pitch law's P + I + D (control_pitch.lua)
        # closes on the balloon's pitch error, the balloon's elevation above the
        # path frame's horizontal plane; then the turn's own body pitch rate,
        # which the FC script adds after the law's clip.
        w = cross3(v, c)
        wz, cz = w[:, 2:3], c[:, 2:3]
        proxy = np.degrees(np.arcsin(np.clip(wz, -1.0, 1.0)))
        lim = chase.bank_lim_deg
        omega_x = (np.clip(b_ref, -lim, lim) - proxy) * max(0.0, chase.roll_gain)
        e = np.degrees(np.arctan2(np.sum(bd * c, -1, keepdims=True),
                                  np.hypot(np.sum(bd * v, -1, keepdims=True),
                                           np.sum(bd * w, -1, keepdims=True))))
        acro = chase.pitch_max_dps
        g_eff = max(0.0, chase.pitch_gain)
        ki = chase.pitch_ki
        if abs(ki) <= PITCH_KI_OFF:
            self.pitch_I = np.zeros_like(self.pitch_I)
            integral = 0.0
        else:
            pred = e * g_eff + ki * (self.pitch_I + e * dt)
            freeze = ((acro > 0.0) & (np.abs(pred) >= acro)
                      & (((e > 0.0) & (pred > 0.0)) | ((e < 0.0) & (pred < 0.0))))
            grow = (np.abs(e) > chase.pitch_deadband_deg) & ~freeze
            self.pitch_I = np.where(grow, self.pitch_I + e * dt, self.pitch_I)
            m = chase.windup_limit_pitch * max(0.0, acro) / ki
            self.pitch_I = np.where(self.pitch_I < -m, -m,
                                    np.where(self.pitch_I > m, m, self.pitch_I))
            integral = ki * self.pitch_I
        omega_y = e * g_eff + integral
        raw = (e - self.pitch_prev) / dt if dt > DT_MIN_S else np.zeros_like(e)
        a = chase.pitch_kd_rate * dt / (1.0 + chase.pitch_kd_rate * dt)
        self.pitch_D = np.where(self.pitch_seeded, self.pitch_D + a * (raw - self.pitch_D), 0.0)
        self.pitch_seeded = np.ones_like(self.pitch_seeded)
        self.pitch_prev = e
        if abs(chase.pitch_kd) > PITCH_KD_OFF:
            omega_y = omega_y + chase.pitch_kd * self.pitch_D
        omega_x = np.clip(omega_x, -chase.roll_max_dps, chase.roll_max_dps)
        omega_y = np.clip(omega_y, -acro, acro)
        roll = np.arctan2(wz, -cz)
        pitch = np.arcsin(np.clip(-v[:, 2:3], -1.0, 1.0))
        turn = np.where(np.abs(np.degrees(roll)) <= C.PITCH_TURN_FF_BANK_LIM_DEG,
                        np.degrees(G * np.sin(roll) * np.cos(pitch) / sp) * np.tan(roll), 0.0)
        omega_y = np.clip(omega_y + chase.pitch_turn_ff * turn, -acro, acro)
        return np.radians(omega_x), np.radians(omega_y)

    def _lvp(self, chase, v, c, qa, sp, bd, b_ref, dt):
        # path_attitude.lua under a PLAN: the guidance and the pitch its own,
        # the roll to the PLAN's bank round the path, held to the room, the
        # bank's drift cancelled, faded near vertical.
        V = np.maximum(sp, C.FLIGHT_PATH_MIN_SPEED_MPS)
        w = cross3(v, c)
        up_x, upy, upz = -v[:, 2:3], -w[:, 2:3], c[:, 2:3]
        um = np.sqrt(upy * upy + upz * upz)
        bank = np.degrees(np.arctan2(-upy, -upz))
        gy = G * w[:, 2:3]
        # the path att_lead_s from now, as the accelerometer sees it turn
        dv = qa * c + (gy / sp) * w
        xp = unit(v + chase.att_lead_s * (sp / V) * dv)
        cth = np.clip(np.sum(bd * xp, -1, keepdims=True), -1.0, 1.0)
        tb = bd - cth * xp
        theta = np.arctan2(norm3(tb, True), cth)
        ty = np.sum(tb * w, -1, keepdims=True)
        tz = -np.sum(tb * c, -1, keepdims=True)
        dn = np.hypot(ty, tz)
        has = dn > DIR_MIN
        dn = np.where(has, dn, 1.0)
        uz = np.where(has, tz / dn, -1.0)
        om = np.minimum(chase.att_k_path * theta, math.radians(chase.att_path_rate_max_dps))
        lo, hi = bank_room(bank, um, chase.lo, chase.hi)
        mu = np.radians(np.clip(wrap180(b_ref - bank), lo, hi))
        fade = np.clip(um / POLE_SIN, 0.0, 1.0)
        zone = theta < math.radians(chase.att_i_zone_deg)
        if chase.att_roll_ki != 0.0:
            rmax = math.radians(chase.roll_max_dps)
            self.roll_i = np.where(zone, np.clip(self.roll_i + chase.att_roll_ki * mu * fade * dt,
                                                 -rmax, rmax), 0.0)
        else:
            self.roll_i = np.zeros_like(self.roll_i)
        if chase.att_pitch_ki != 0.0:
            qmax = math.radians(chase.pitch_max_dps)
            self.pitch_i = np.where(zone, np.clip(self.pitch_i + chase.att_pitch_ki * theta * (-uz) * dt,
                                                  -qmax, qmax), 0.0)
        else:
            self.pitch_i = np.zeros_like(self.pitch_i)
        wx = chase.att_k_roll * mu + self.roll_i
        wy = om * (-uz) + self.pitch_i
        wz = gy / V
        live = um > DIR_MIN
        ums = np.where(live, um, 1.0)
        tg = np.clip(up_x / ums, -TAN_CAP, TAN_CAP)
        drift = np.where(live, -tg * (upy * wy + upz * wz) / ums, 0.0)
        wx = (wx - drift) * fade
        p = np.clip(np.degrees(wx), -chase.roll_max_dps, chase.roll_max_dps)
        q = np.clip(np.degrees(wy), -chase.pitch_max_dps, chase.pitch_max_dps)
        return np.radians(p), np.radians(q)


def balloon_dir(chase, r):
    if chase.balloon_pos is None:
        return np.broadcast_to(chase.b_dir, r.shape)
    return unit(chase.balloon_pos - r)


def runin_line(chase, bd, prev):
    """The run-in line for balloon directions bd (B, 3): runin_climb_deg above
    the horizon, toward the balloon's compass direction.  Where the balloon is
    straight above or below it has no compass direction, and the line stays
    prev."""
    h = np.hypot(bd[:, 0], bd[:, 1])[:, None]
    cl = math.radians(chase.runin_climb_deg)
    ld = np.concatenate([math.cos(cl) * bd[:, :2] / np.maximum(h, 1e-12),
                         np.full_like(h, -math.sin(cl))], axis=-1)
    return np.where(h > 1e-9, ld, prev)


def fly(chase, knots, dt=DT_PLAN_S, record=False, t_cap=TURN_HORIZON_S, dep=None, start=None):
    """The selected law flying a batch of plans in its plan mode, run by the
    Curve Flyer, on the planner's aircraft.  knots (B, N_KNOTS) deg, in the
    law's measure; for a chase with a run-in, dep (B,) the balloon's
    depression, deg, at which each plan's run-in ends.  A run-in chase flown
    without dep is its LINE-UP alone: the turn onto the run-in line, complete
    within STRAIGHT_TOL_DEG of it, and then the run-in holding that line.
    Returns a dict of per-plan results; with record, also the sampled
    trajectories.  The flight stops at t_cap for a plan whose turn has not
    completed by then, and t_cap comes down as the batch flies: once a plan's
    turn and its SETTLE_S after it are flown its cost is known, and a plan
    still turning past that cost cannot beat it, since a plan costs at least
    its turn time.  A batch therefore flies only until the cost of its best
    finished plan, not as long as its slowest.  start (dive_starts())
    flies each plan from the moment its dive starts, its run-in already
    flown."""
    knots = np.atleast_2d(knots)
    B = knots.shape[0]
    runin = chase.runin_climb_deg is not None
    full = runin and dep is not None
    r = np.zeros((B, 3)); v = np.tile(chase.v0, (B, 1)); c = np.tile(chase.c0, (B, 1))
    sp = np.full((B, 1), chase.speed0)
    gvec = np.array([0.0, 0.0, G])
    pa = np.zeros((B, 1))
    qa = (G + np.sum(c * gvec, -1, keepdims=True)) / sp     # the pitch rate of 1 g along the canopy
    law = PlanMode(B)
    eps0 = math.radians(chase.turn0_deg)
    tol = math.radians(STRAIGHT_TOL_DEG)
    pole_sin = POLE_SIN
    rows = np.arange(B)
    lim = chase.bank_lim_deg
    # The line each plan holds once its turn is complete (Transfer Control When
    # Done off): the path at that moment.
    hold = np.zeros((B, 3)); held = np.zeros((B, 1), bool)
    phase = np.full(B, FLY)
    # What each plan's law aimed at on the last step, a line or the balloon.
    aim_prev = [None]
    if runin:
        line = runin_line(chase, np.tile(chase.b_dir, (B, 1)), np.tile(chase.v0, (B, 1)))
    if full:
        dep = np.asarray(dep, float)
    t0 = 0.0
    if start is not None:
        r, v, c, pa, qa, sp = (start[k].copy() for k in ('r', 'v', 'c', 'pa', 'qa', 'sp'))
        law = start['law'].copy()
        phase = np.full(B, DIVE)
        t0 = start['t']
        line = runin_line(chase, balloon_dir(chase, r), v)
        aim_prev[0] = np.ones(B, bool)             # the run-in aimed at its line

    def commands(r_, v_, c_, qa_, sp_, step):
        bd_bal = balloon_dir(chase, r_)
        eps_bal = np.arccos(np.clip(np.sum(v_ * bd_bal, -1, keepdims=True), -1.0, 1.0))
        ld = None
        if runin:
            # The scheduled turn and the run-in aim at the run-in line, and so
            # does the run-in after a line-up flown alone; the dive aims at the
            # balloon.
            ld = runin_line(chase, bd_bal, line)
            on_line = (phase == FLY) | (phase == RUNIN) | ((phase == AFTER) & (not full))
            bd = np.where(held, hold, np.where(on_line[:, None], ld, bd_bal))
            aim = held[:, 0] | on_line
        else:
            bd = np.where(held, hold, bd_bal)
            aim = held[:, 0]
        cos_e = np.clip(np.sum(v_ * bd, -1, keepdims=True), -1.0, 1.0)
        eps = np.arccos(cos_e)
        s = np.clip(1.0 - eps / eps0, 0.0, 1.0) if eps0 > 0 else np.ones_like(eps)
        x = s[:, 0] * (N_KNOTS - 1)
        i0 = np.minimum(x.astype(int), N_KNOTS - 2)
        w = (x - i0)[:, None]
        b_ref = knots[rows, i0][:, None] * (1.0 - w) + knots[rows, i0 + 1][:, None] * w
        # A held line, the run-in's line and the dive's balloon: banked
        # CF_HOLD_BANK_PER_DEG per deg the balloon lies across the wings
        # (v x c, + right), level on it, within the bank limit.
        across_w = np.sum(bd * cross3(v_, c_), -1, keepdims=True)
        b_toward = np.clip(C.CF_HOLD_BANK_PER_DEG * np.degrees(np.arcsin(np.clip(across_w, -1, 1))),
                           -lim, lim)
        if runin:
            b_ref = np.where((phase == FLY)[:, None], b_ref, b_toward)
        else:
            b_ref = np.where(held, b_toward, b_ref)
        if aim_prev[0] is not None:
            law.re_aim(aim != aim_prev[0])
        aim_prev[0] = aim
        p_c, q_c = law.commands(chase, v_, c_, qa_, sp_, bd, b_ref, step)
        return p_c, q_c, eps_bal, eps, ld, bd_bal

    def deriv(v_, c_, pa_, qa_, sp_, p_c, q_c):
        # The pitch rate turns the path toward the canopy; gravity across the
        # wings turns it sideways.
        w = cross3(v_, c_)
        dv = qa_ * c_ + (np.sum(gvec * w, -1, keepdims=True) / sp_) * w
        dc = pa_ * cross3(v_, c_) - qa_ * v_
        dsp = SPEED_GRAV_MPS2 * v_[:, 2:3] + THRUST_MPS2 - DRAG_PER_V2 * sp_ * sp_
        return sp_ * v_, dv, dc, (p_c - pa_) / ROLL_LAG_S, (q_c - qa_) / PITCH_LAG_S, dsp

    t_end = t_cap + SETTLE_S
    # A step past t_end: a plan whose turn completes by t_cap is flown until
    # its SETTLE_S after the turn has passed.
    n_max = int(round(t_end / dt)) + 2
    t_done = np.full(B, np.inf)
    eps_prev = None
    alt_min = np.full(B, chase.alt0); sp_min = np.full(B, chase.speed0)
    tilt = chase.tilt_dir * np.degrees(np.arcsin(np.clip(-v[:, 2], -1, 1)))
    tilt_peak = tilt.copy(); tilt_back = np.zeros(B)
    pole_min = np.full(B, 90.0)
    h_end = np.full((B, 2), np.nan); head_dev = np.zeros(B)
    eps_min = np.full(B, np.inf)
    eps_after = np.zeros(B)
    roll_travel = np.zeros(B)
    bank_over = np.zeros(B)
    # How far each plan goes upside down past the start's allowance, and the
    # band its bank proxy is judged against: a start upside down is judged
    # against it only once the right way up.
    upside_over = np.zeros(B)
    righted = np.full(B, chase.down0_deg <= 0.0)
    band_lo, band_hi = np.full(B, chase.lo), np.full(B, chase.hi)
    t_lined = np.full(B, np.inf); t_dive = np.full(B, np.inf)
    dep_done = np.full(B, np.nan)
    if start is not None:
        (alt_min, sp_min, tilt_peak, tilt_back, pole_min, bank_over, upside_over, righted,
         band_lo, band_hi, roll_travel, eps_min, t_lined) = (
            start[k].copy() for k in ('alt_min', 'sp_min', 'tilt_peak', 'tilt_back', 'pole_min',
                                      'bank_over', 'upside_over', 'righted', 'band_lo',
                                      'band_hi', 'roll_travel', 'eps_min', 't_lined'))
        t_dive = t0.copy()
    rec = []
    # The clock: k steps of dt, but for the run-in's longer ones (DT_RUNIN_S),
    # each of which moves it on and restarts the count; each plan's own time
    # from its start, where it has one.  The law's tick is the step just
    # flown, as the FC's is the interval since its last.
    k, k_at, t_at, dt_prev = 0, 0, 0.0, dt
    while True:
        elapsed = t_at + (k - k_at) * dt
        t = elapsed if start is None else t0 + elapsed
        if (np.min(t) > t_end) if full else (k >= n_max):
            break
        p_c, q_c, eps, eps_tgt, ld, bd_bal = commands(r, v, c, qa, sp, dt_prev)
        # What completes the turn: the balloon, or for a line-up flown alone
        # the run-in line.
        e = eps_tgt[:, 0] if (runin and not full) else eps[:, 0]
        eps_min = np.minimum(eps_min, e)
        if not runin:
            if eps_prev is not None:
                hit = np.isinf(t_done) & (e <= tol)
                frac = np.where(hit, (eps_prev - tol) / np.maximum(eps_prev - e, 1e-12), 0.0)
                t_done = np.where(hit, t - dt + frac * dt, t_done)
                newly = hit
                h_end[newly] = v[newly, :2]
            elif e[0] <= tol:
                newly = np.ones(B, bool)
                t_done[:] = 0.0; h_end[:] = v[:, :2]
            else:
                newly = np.zeros(B, bool)
        else:
            # The scheduled turn ends on the run-in line; a run-in ends at its
            # depression; the dive, and the turn with it, on the balloon.  One
            # step may pass through several, as one FC tick may.
            dep_now = np.degrees(np.arcsin(np.clip(bd_bal[:, 2], -1.0, 1.0)))
            lined = (phase == FLY) & (eps_tgt[:, 0] <= tol)
            t_lined = np.where(lined, t, t_lined)
            if full:
                phase = np.where(lined, RUNIN, phase)
                trig = (phase == RUNIN) & (dep_now >= dep)
                t_dive = np.where(trig, t, t_dive)
                phase = np.where(trig, DIVE, phase)
                hit = (phase == DIVE) & (e <= tol) & np.isinf(t_done)
            else:
                hit = lined & np.isinf(t_done)
            if eps_prev is None:
                t_done = np.where(hit, t, t_done)
            else:
                frac = np.clip(np.where(hit, (eps_prev - tol) / np.maximum(eps_prev - e, 1e-12), 0.0),
                               0.0, 1.0)
                t_done = np.where(hit, t - dt_prev + frac * dt_prev, t_done)
            newly = hit
            h_end[newly] = v[newly, :2]
            if full:
                dep_done = np.where(newly, dep_now, dep_done)
            phase = np.where(newly, AFTER, phase)
            line = ld
        # Only the turn's own end starts a hold: a line-up's end starts its run-in.
        if not chase.transfer and (full or not runin):
            hold[newly] = v[newly]; held[newly] = True
        eps_prev = e
        settled = t > t_done + SETTLE_S
        if k % CAP_EVERY_STEPS == 0 and np.any(settled):
            sofar = dict(t_done=t_done, t_cap=t_cap, eps_min=np.degrees(eps_min),
                         alt_min=alt_min, speed_min=sp_min, tilt_back=tilt_back,
                         pole_min=pole_min, heading_dev=head_dev, eps_after=eps_after,
                         roll_travel=roll_travel, bank_over=bank_over,
                         upside_over=upside_over,
                         dep_done=dep_done if full else None)
            t_cap = min(t_cap, float(np.min(cost(chase, sofar)[settled])))
        if np.all(settled | (t > t_cap) & np.isinf(t_done)):
            break
        # rule checks over the turn and the settle window
        live = ~settled
        alt_min = np.where(live, np.minimum(alt_min, chase.alt0 - r[:, 2]), alt_min)
        sp_min = np.where(live, np.minimum(sp_min, sp[:, 0]), sp_min)
        tilt = chase.tilt_dir * np.degrees(np.arcsin(np.clip(-v[:, 2], -1, 1)))
        tilt_peak = np.maximum(tilt_peak, tilt)
        tilt_back = np.where(live, np.maximum(tilt_back, tilt_peak - tilt), tilt_back)
        pole_min = np.where(live, np.minimum(pole_min, np.degrees(np.arccos(np.minimum(1.0, np.abs(v[:, 2]))))), pole_min)
        # upside down: the canopy's angle below the horizon, never further
        # than at the start.  The proxy cannot see it: it reads level wings
        # upside down as no bank.
        down = np.degrees(np.arcsin(np.clip(c[:, 2], -1.0, 1.0)))
        upside_over = np.where(live, np.maximum(upside_over, down - chase.down_allow), upside_over)
        # the bank proxy flown, against the band, which it measures at every
        # attitude.  A start upside down rolls the right way up through
        # whatever bank that takes: its band holds from its first tick the
        # right way up, widened to the proxy there, as a banked start's is.
        prox = proxy_deg(v, c)
        now_up = live & ~righted & (down <= 0.0)
        band_lo = np.where(now_up, np.minimum(-lim, prox), band_lo)
        band_hi = np.where(now_up, np.maximum(lim, prox), band_hi)
        righted = righted | now_up
        over = np.where(righted, np.maximum(band_lo - prox, prox - band_hi), 0.0)
        bank_over = np.where(live, np.maximum(bank_over, over), bank_over)
        after = ~np.isinf(t_done) & live
        if full or not runin:
            # (a line-up flown alone ends on the run-in line, not the balloon:
            # the heading is the turn's to keep, after the dive)
            hz = v[:, :2]; hn = np.hypot(hz[:, 0], hz[:, 1]); he = np.hypot(h_end[:, 0], h_end[:, 1])
            cosd = np.sum(hz * np.nan_to_num(h_end), -1) / np.maximum(hn * he, 1e-12)
            dev = np.degrees(np.arccos(np.clip(cosd, -1, 1)))
            weigh = np.clip(hn / pole_sin, 0.0, 1.0)              # heading fades near vertical
            head_dev = np.where(after, np.maximum(head_dev, dev * weigh), head_dev)
        eps_after = np.where(after, np.maximum(eps_after, np.degrees(e)), eps_after)
        if record:
            b, _ = bank_deg(v, c)
            rec.append(dict(t=t, r=r.copy(), v=v.copy(), c=c.copy(), sp=sp[:, 0].copy(),
                            eps=np.degrees(e), bank=b.copy(), proxy=prox.copy(),
                            p=np.degrees(pa[:, 0]), q=np.degrees(qa[:, 0]),
                            load=((qa * sp - np.sum(c * gvec, -1, keepdims=True)) / G)[:, 0],
                            phase=phase.copy()))
            if full:
                # what dive_starts() needs to fly a dive on from here
                rec[-1].update(pa=pa.copy(), qa=qa.copy(), sp_col=sp.copy(), dep=dep_now.copy(),
                               law=law.copy(),
                               alt_min=alt_min.copy(), sp_min=sp_min.copy(),
                               tilt_peak=tilt_peak.copy(), tilt_back=tilt_back.copy(),
                               pole_min=pole_min.copy(), bank_over=bank_over.copy(),
                               upside_over=upside_over.copy(), righted=righted.copy(),
                               band_lo=band_lo.copy(), band_hi=band_hi.copy(),
                               roll_travel=roll_travel.copy(), eps_min=eps_min.copy(),
                               t_lined=t_lined.copy())
        # The run-in's longer step, while every plan still flying is settled on
        # its run-in line and further from its dive than two such steps can
        # bring the balloon's depression.
        step = dt
        if full and np.all(~live | (phase == RUNIN)):
            dist = norm3(chase.balloon_pos - r)
            reach = 2.0 * np.degrees(sp[:, 0] / np.maximum(dist, 1e-9)) * DT_RUNIN_S
            calm = ((np.degrees(eps_tgt[:, 0]) < RUNIN_CALM_DEG)
                    & (np.abs(np.degrees(pa[:, 0])) < RUNIN_CALM_DPS))
            if np.all(~live | (calm & (dep - dep_now > reach))):
                step = DT_RUNIN_S
        # midpoint step, the commands held over the step as the FC holds them over a tick
        y0 = (r, v, c, pa, qa, sp)
        k1 = deriv(v, c, pa, qa, sp, p_c, q_c)
        ym = [a + 0.5 * step * d for a, d in zip(y0, k1)]
        k2 = deriv(unit(ym[1]), unit(ym[2]), ym[3], ym[4], ym[5], p_c, q_c)
        r, v, c, pa, qa, sp = [a + step * d for a, d in zip(y0, k2)]
        roll_travel = np.where(live, roll_travel + np.degrees(np.abs(pa[:, 0])) * step, roll_travel)
        v = unit(v); c = unit(across(c, v))
        k += 1
        if step != dt:
            k_at, t_at = k, elapsed + step
        dt_prev = step
    return dict(t_done=t_done, eps_min=np.degrees(eps_min), alt_min=alt_min, speed_min=sp_min,
                tilt_back=tilt_back, pole_min=pole_min, heading_dev=head_dev, eps_after=eps_after, roll_travel=roll_travel, bank_over=bank_over,
                upside_over=upside_over, rec=rec,
                t_cap=t_cap, t_lined=t_lined, t_dive=t_dive,
                dep_done=dep_done if full else None)


def cost(chase, res):
    """Time to complete the turn, plus PENALTY_PER_UNIT per unit of rule
    broken, and with a run-in APPROACH_PENALTY_PER_DEG per degree the line
    from the aircraft to the balloon ends off the Approach Angle.  A turn not
    completed by the flight's cap costs the cap plus its closest miss, with a
    run-in APPROACH_MISS_MAX_DEG of Approach Angle besides, and NEVER_DONE_S
    on top: more than any turn that completes."""
    t = np.where(np.isinf(res['t_done']), res['t_cap'] + res['eps_min'] / 10.0, res['t_done'])
    pen = (np.maximum(0.0, ALT_MIN_M - res['alt_min'])
           + np.maximum(0.0, res['eps_after'] - STRAIGHT_TOL_DEG)
           + np.maximum(0.0, SPEED_MIN_MPS - res['speed_min']))
    if chase.rule:
        pen = pen + (res['bank_over']
                     + res['upside_over']
                     + np.maximum(0.0, res['tilt_back'] - chase.tilt_back_deg)
                     + np.maximum(0.0, res['heading_dev'] - STRAIGHT_TOL_DEG)
                     + np.maximum(0.0, C.CURVE_POLE_FADE_DEG - res['pole_min']))
    total = t + PENALTY_PER_UNIT * pen + ROLL_TRAVEL_S_PER_DEG * res['roll_travel']
    if res.get('dep_done') is not None:
        miss = np.abs(res['dep_done'] - chase.approach_deg)
        total = total + APPROACH_PENALTY_PER_DEG * np.where(
            np.isinf(res['t_done']), APPROACH_MISS_MAX_DEG, miss)
    return total + np.where(np.isinf(res['t_done']), NEVER_DONE_S, 0.0)


class PlanResult:
    """The best candidate a search found: x (its parameters), fun (its
    cost), nfev (candidates flown) and nit (batches)."""

    def __init__(self, x, fun, nfev, nit):
        self.x, self.fun, self.nfev, self.nit = x, fun, nfev, nit


def polish(flown, X, step, blo, bhi, rng):
    """Stage 2 of a search: PLAN_REFINE_ROUNDS rounds, each one batch, every
    parameter of each plan in X stepped up and down by its step and
    PLAN_REFINE_RANDOM random nudges of the same size; a plan takes the best
    that beats it, and its steps halve when none does.  Returns the best plan
    and its cost."""
    CX = np.full(len(X), np.inf)
    dim = X.shape[1]
    for _ in range(PLAN_REFINE_ROUNDS):
        batch = [np.clip(np.vstack([x[None], x + np.diag(d), x - np.diag(d),
                                    x + rng.normal(size=(PLAN_REFINE_RANDOM, dim)) * d]), blo, bhi)
                 for x, d in zip(X, step)]
        per = len(batch[0])
        cy = flown(np.vstack(batch))
        for j in range(len(X)):
            seg = cy[j * per:(j + 1) * per]
            a = int(np.argmin(seg))
            if seg[a] < CX[j]:
                X[j], CX[j] = batch[j][a], seg[a]
            else:
                step[j] *= 0.5
    j = int(np.argmin(CX))
    return X[j].copy(), float(CX[j])


def plan_turn(chase, rng):
    """The soonest bank schedule found for the scheduled turn - onto the
    balloon, or with a run-in onto the run-in line.  Returns (knots deg,
    result).

    The turn's cost has several valleys - a left and a right turn, a roll far
    over - so a search that only runs downhill keeps whichever it starts in;
    and within a valley it is nearly one-dimensional: the best schedule is
    close to one steady bank.  Stage 1 therefore finds the valleys with
    steady banks, and stage 2 shapes the schedule inside the best of them
    (PLAN_* above).  Across batches, a batch is flown only until the best
    plan found so far could no longer be beaten: a plan costs at least its
    turn time, so one still turning at the best cost cannot beat it.  The
    bound is that cost, never the soonest turn's time: where a rule binds,
    the soonest turn can carry a rule's penalty, and a plan still turning
    then can yet complete within the rules at a lower cost."""
    lo, hi = knot_range(chase)
    blo, bhi = np.full(N_KNOTS, lo), np.full(N_KNOTS, hi)
    best_t = [TURN_HORIZON_S]
    nfev = [0]

    def flown(Z, dt=DT_PLAN_S):
        res = fly(chase, Z, dt=dt, t_cap=best_t[0])
        nfev[0] += len(Z)
        c = cost(chase, res)
        best_t[0] = min(best_t[0], float(np.min(c)))
        return c

    # Stage 1: steady banks.  Its costs are on the coarser step, so its cap is
    # not carried into stage 2.
    banks = np.arange(lo, hi + PLAN_SCAN_STEP_DEG / 2, PLAN_SCAN_STEP_DEG)
    # Each bank twice: held from the start, and reached by a ramp from the
    # starting bank, which a start already banked may fly sooner.
    ramp = np.linspace(0.0, 1.0, N_KNOTS)[None, :]
    b0 = min(hi, max(lo, chase.bank0))
    Z = np.vstack([np.repeat(banks[:, None], N_KNOTS, 1), b0 + (banks[:, None] - b0) * ramp])
    keys = np.r_[banks, banks]
    if chase.rule:
        # Within the bank limit a short turn may be soonest rolling one way
        # and then the other: a first bank to the turn's midpoint and a second
        # after it, every pair PLAN_SWITCH_STEP_DEG apart.
        grid = np.arange(lo, hi + PLAN_SWITCH_STEP_DEG / 2, PLAN_SWITCH_STEP_DEG)
        b1, b2 = (a.ravel() for a in np.meshgrid(grid, grid))
        half = N_KNOTS // 2
        sw = np.column_stack([np.repeat(b1[:, None], half, 1),
                              np.repeat(b2[:, None], N_KNOTS - half, 1)])
        Z = np.vstack([Z, sw])
        keys = np.r_[keys, b2]
    c = flown(Z, PLAN_SCAN_DT_S)
    best_t[0] = TURN_HORIZON_S
    picked = []
    for i in np.argsort(c):
        if all(abs(wrap180(keys[i] - keys[j])) >= PLAN_VALLEY_SEP_DEG for j in picked):
            picked.append(i)
        if len(picked) == PLAN_VALLEYS:
            break

    # Stage 2: each plan's own costs start unknown, so the first round flies
    # the plans themselves beside their neighbours, on the planner's step.
    X = Z[picked].copy()
    step = np.full((len(X), N_KNOTS), PLAN_REFINE_KNOT_STEP_DEG)
    x, fun = polish(flown, X, step, blo, bhi, rng)
    return x, PlanResult(x, fun, nfev[0], 1 + PLAN_REFINE_ROUNDS)


def dive_starts(rec, deps):
    """Where one plan's run-in, flown and recorded (fly(record=True) of a
    single plan whose dive starts at or past every depression in deps),
    reaches each run-in depression in deps: the state there and the checks
    flown so far, for fly(start=...) to fly each dive on from.  A depression
    the run-in starts at or past starts the dive where the line-up ends.  The
    aircraft's state is interpolated between the samples either side of the
    depression; the law's and the checks are the earlier sample's.  None when
    the recorded plan never reached its run-in."""
    run = [i for i, smp in enumerate(rec) if smp['phase'][0] != FLY]
    if not run:
        return None
    i0 = run[0]
    i1 = next((i for i in run if rec[i]['phase'][0] in (DIVE, AFTER)), run[-1])
    seg = rec[i0:i1 + 1]
    dep_s = np.array([smp['dep'][0] for smp in seg])
    deps = np.asarray(deps, float)
    # the first sample at or past each depression, and the one before it
    j = np.array([int(np.argmax(dep_s >= d)) if np.any(dep_s >= d) else len(seg) - 1
                  for d in deps])
    a = np.maximum(j - 1, 0)
    w = np.where(j > 0, (deps - dep_s[a]) / np.maximum(dep_s[j] - dep_s[a], 1e-12), 0.0)
    w = np.clip(w, 0.0, 1.0)[:, None]

    def at(key):
        x = np.array([smp[key] for smp in seg]) if key == 't' else np.array([smp[key][0] for smp in seg])
        return x[a] + (x[j] - x[a]) * (w if x.ndim > 1 else w[:, 0])

    v = unit(at('v')); c = unit(across(at('c'), v))
    out = dict(r=at('r'), v=v, c=c, pa=at('pa'), qa=at('qa'), sp=at('sp_col'), t=at('t'),
               law=PlanMode.stack([seg[i]['law'] for i in a]))
    for key in ('alt_min', 'sp_min', 'tilt_peak', 'tilt_back', 'pole_min', 'bank_over',
                'upside_over', 'righted', 'band_lo', 'band_hi', 'roll_travel', 'eps_min',
                't_lined'):
        out[key] = np.array([seg[i][key][0] for i in a])
    return out


def plan_dive(chase, knots, rng):
    """The run-in's end, on the line-up plan_turn() chose: the balloon's
    depression at which the dive starts, so the turn ends at the Approach
    Angle as soon as it can.  The line-up and the run-in are the same for
    every such plan: they are flown once, to a dive at the Approach Angle, and
    each plan's dive is flown on from where its run-in ends.  Returns (run-in
    depression deg, result)."""
    blo, bhi = np.array([chase.dep0_deg]), np.array([chase.approach_deg])
    one = fly(chase, knots, record=True, t_cap=chase.t_cap0, dep=np.array([chase.approach_deg]))
    best_t = [chase.t_cap0]
    nfev = [1]

    # A plan costs at least its turn time, so no plan is flown past the best
    # cost found so far: a turn that ends soon but off the Approach Angle
    # bounds nothing.
    def flown(Z, dt=DT_PLAN_S):
        n = len(Z)
        res = fly(chase, np.tile(knots, (n, 1)), dt=dt, t_cap=best_t[0],
                  dep=Z[:, 0], start=dive_starts(one['rec'], Z[:, 0]))
        nfev[0] += n
        c = cost(chase, res)
        best_t[0] = min(best_t[0], float(np.min(c)))
        return c

    deps = np.minimum(np.arange(blo[0], bhi[0] + PLAN_DEP_STEP_DEG / 2, PLAN_DEP_STEP_DEG), bhi[0])
    Z = deps[:, None]
    c = flown(Z, PLAN_SCAN_DT_S)
    best_t[0] = chase.t_cap0
    X = Z[[int(np.argmin(c))]].copy()
    step = np.full((1, 1), PLAN_DEP_STEP_DEG)
    x, fun = polish(flown, X, step, blo, bhi, rng)
    return float(x[0]), PlanResult(x, fun, nfev[0], 2 + PLAN_REFINE_ROUNDS)


def plan(chase, seed=0):
    """The chase's PLAN: plan_turn()'s bank schedule, and with a run-in
    plan_dive()'s run-in depression.  Returns a dict - law, knots (deg, in its
    measure), runin_climb_deg and runin_dep_deg (NaN without a run-in), cost,
    evals - as the PLAN message carries it."""
    rng = np.random.default_rng(seed)
    knots, res = plan_turn(chase, rng)
    out = dict(law=chase.law, knots=[float(k) for k in knots],
               runin_climb_deg=math.nan, runin_dep_deg=math.nan,
               cost=res.fun, evals=res.nfev)
    if chase.runin_climb_deg is not None:
        dep, res2 = plan_dive(chase, knots, rng)
        out.update(runin_climb_deg=float(chase.runin_climb_deg), runin_dep_deg=dep,
                   cost=res2.fun, evals=res.nfev + res2.nfev)
    return out


def check(chase, p):
    """One plan (plan()'s dict) flown at DT_CHECK_S, finer than the planner
    flies it."""
    runin = not math.isnan(p['runin_dep_deg'])
    res = fly(chase, np.atleast_2d(p['knots']), dt=DT_CHECK_S,
              t_cap=chase.t_cap0 if runin else TURN_HORIZON_S,
              dep=np.atleast_1d(p['runin_dep_deg']) if runin else None)
    return {k: float(np.atleast_1d(v)[0]) for k, v in res.items()
            if k != 'rec' and v is not None}


def serve():
    """The planner process: requests arrive pickled on stdin, replies leave
    pickled on stdout, until stdin closes.  stdout is the channel, so the
    process's own stdout is pointed at stderr: a stray print cannot corrupt a
    reply."""
    channel = os.fdopen(os.dup(1), 'wb')
    os.dup2(2, 1)
    inp = sys.stdin.buffer
    while True:
        try:
            kind, args = pickle.load(inp)
        except EOFError:
            return
        try:
            if kind == 'plan':
                reply = ('ok', plan(*args))
            elif kind == 'check':
                reply = ('ok', check(*args))
            else:
                reply = ('error', f"unknown request {kind!r}")
        except Exception as err:              # reported to the caller, which raises it
            reply = ('error', f"{type(err).__name__}: {err}")
        pickle.dump(reply, channel)
        channel.flush()


class PlannerProcess:
    """The Curve Planner in a process of its own.

    Planning is pure Python stepping small numpy arrays.  On a thread of the
    companion it holds the interpreter lock almost throughout, so the 50 Hz
    control loop - GUIDE, the gimbal, the log - runs at a fraction of its rate
    for as long as a plan takes, and the plan itself slows by the same
    contention.  In its own process it shares nothing.  Calls block the
    calling thread (never the control loop's) until the reply arrives; one
    request is in flight at a time.  The process ends when its stdin closes,
    which the companion's exit does by itself."""

    def __init__(self):
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self._proc = subprocess.Popen(
            [sys.executable, '-W', 'ignore::RuntimeWarning', '-m',
             'Companion.curve_planner', '--serve'],
            cwd=root, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self._lock = threading.Lock()

    def _call(self, kind, *args):
        with self._lock:
            try:
                pickle.dump((kind, args), self._proc.stdin)
                self._proc.stdin.flush()
                status, value = pickle.load(self._proc.stdout)
            except (EOFError, BrokenPipeError, OSError) as err:
                raise RuntimeError(f"the Curve Planner process stopped (exit code "
                                   f"{self._proc.poll()}): {err}") from err
        if status != 'ok':
            raise RuntimeError(f"the Curve Planner process failed: {value}")
        return value

    def plan(self, chase):
        """plan() on the planner process: the PLAN for chase, as a dict."""
        return self._call('plan', chase)

    def check(self, chase, p):
        """check() on the planner process."""
        return self._call('check', chase, p)

    def close(self):
        try:
            self._proc.stdin.close()
            self._proc.wait(timeout=2.0)
        except (OSError, subprocess.TimeoutExpired):
            self._proc.kill()


if __name__ == "__main__":
    if "--serve" in sys.argv:
        # Run the server from the package's own module, so the Chase a request
        # unpickles and the plan() that flies it are the same module's.
        from Companion.curve_planner import serve as _serve
        _serve()
        sys.exit(0)
    import time
    el, az = math.radians(-70), math.radians(45)
    b = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), -math.sin(el)])
    for law_ in (LAW_CASCADE, LAW_LVP):
        ch = Chase(0, 0, b, rng=550 / math.sin(-el), alt_m=600.0, law=law_)
        t0 = time.time()
        out = plan(ch)
        print("law", law_, "rule", ch.rule, "knots", np.round(out['knots'], 1),
              "cost", round(out['cost'], 3), "plan s", round(time.time() - t0, 1),
              "evals", out['evals'])
        print(check(ch, out))
