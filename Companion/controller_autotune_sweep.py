# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""Controller Autotune - automatic gain sweep (the 't' key).

This is the AUTOMATIC half of Controller Autotune (the manual gain-tuner window
in ui/controller_autotune.py is the interactive half).  It mirrors the *shape*
of ArduPlane's AUTOTUNE, but for OUR steering gains instead of the autopilot's
rate PIDs:

    for each parameter, try a FIXED bracket of candidate values around its default,
    fly a repeatable balloon-tracking step at EACH, score how well the aircraft aligns
    onto the balloon, and keep the lowest-cost candidate.  No edge-finding, no
    un-flown backoff: every value that gets kept was actually flown and scored.

It walks the parameter set (SWEEP_PLAN); each parameter carries a test bearing
and an error axis to score.

WHY BRACKET-AND-SCORE, NOT EDGE-FIND
    "Step a gain until the trial breaks, then back off" is fragile: it accepts a
    backoff value it never actually flew, it assumes a monotone "more = worse" edge
    that is false for limits/filters/decay thresholds, and with a knife-edge align
    gate a no-align backs off in one direction regardless - which made it RATCHET
    every parameter geometrically across passes and runs.  Trying a fixed, bounded
    bracket (always including "leave it at default") and keeping the lowest-COST
    flown candidate removes all of that: it cannot ratchet, it is repeatable run to
    run, and every kept value is self-validating.

MULTI-PASS (the gains are NOT orthogonal)
    P / I / D / limits within an axis are strongly coupled, so a parameter's best
    candidate depends on where the others currently sit.  The sweep runs up to
    MAX_PASSES coordinate-descent passes, re-picking each parameter's best candidate
    (over the SAME fixed bracket) against the others' CURRENT values, until no
    parameter's choice changes - a discrete coordinate-wise optimum.  Because the
    bracket is fixed and bounded, this converges to a repeatable fixed point and
    cannot run away.  A parameter is frozen once its choice is stable vs the previous
    pass (to bound runtime); a choice could in principle still shift when a coupled
    gain moves later - the COMBINED-SET validation (_validate_combined, both axes at
    once, must hold a sustained alignment) is the safety net for that.  This is a
    coordinate-wise optimizer with a combined check, NOT a rigorous joint optimum
    (that would need system-id + model-based or a per-axis simplex over P/I/D); use
    it as a strong starting point and hand-refine the coupled core if needed.

HOW A TRIAL WORKS
    The sweep RESETS every swept parameter to its constants.py default at the start
    (so every run begins from the same baseline), enters ACRO ONCE, and holds the
    gimbal LOCKED throughout (the oracle drives the cascade; gimbal aim is
    irrelevant).  Each trial anchors the balloon via SyntheticBalloonInjector and arms
    the cascade through the controller's public hooks (ptest_begin resets
    integrators for a clean start); between trials autotune_pause_tracking() drops
    tracking and the oracle but leaves ACRO and the gimbal lock untouched.

    MANEUVER GRID + WORST-CASE: a gain tuned at one operating point can be soft and
    underperform when the real maneuver is more aggressive.  So each candidate is
    flown at EVERY cell of a tuner-configured pan x tilt grid (autotune_pan/tilt_min
    /max/step; pan-axis entries capped at GRID_TILT_CAP_DEG) and scored by its
    WORST-CASE cost over the cells - the kept value must
    hold up at the hardest maneuver, not just on average.  The report additionally
    gives the best value AT EACH cell.  Trial count scales with the grid, so the
    ranges/steps are the runtime knob.

    ALTITUDE PROTECTION: the aggressive dives bleed height, so if AGL altitude drops
    below the protection floor (pre-trial or mid-trial) the sweep pauses, climbs back
    to the climb-to altitude (climb_to), and RE-FLIES the interrupted cell - up to
    PROTECT_MAX_RETRIES, after which the breached result is used (scores as no-align).
    Floor and climb-to are live Gain-Tuner knobs (autotune_protect_floor_m /
    autotune_protect_climb_to_m). Re-flies don't double-count on the progress meter.

    The EXCITATION is a STEP: anchor a constant world bearing and drive to a
    sustained alignment.  (Plant ID's doublet is an unrelated maneuver in its own
    module - a roll-RATE override, not a balloon re-aim.)

SCORING (per candidate) - LOCK-GATED + CONTINUOUS COST
    Each candidate drives to the balloon and WAITS for a genuine, sustained alignment:
    ctrl.alignment_errors() - the flight-path error, the one alignment measure -
    must hold within ALIGN_TOL_DEG (Pythagorean, the SAME tolerance as the HUD
    "Path Aligned" indicator and the 'p' verdict) for ALIGN_DWELL_S continuously, else it times out
    at ALIGN_TIMEOUT_S.  The candidate's COST (lower = better) rewards a fast,
    low-JITTER, non-oscillating alignment (COST_W_TLOCK/JITTER/OVERSHOOT/RING); a no-align
    candidate is penalised above ANY aligned one (NOALIGN_BASE) but still ordered by
    its closest approach (min_err), so even an aggressive bearing where nothing fully
    aligns yields a usable gradient.  Rewarding low jitter - not merely "aligned at
    all" - is what stops the search from keeping a barely-holding value (the
    marginal-align coin-flip that made two earlier runs disagree).  The lowest-cost
    candidate is kept; it was flown, so it is self-validating.

RESULTS
    Final values are applied LIVE to the running controller (via the same
    ControllerAutotuneStore the manual tuner uses) and written to a timestamped
    file controller_autotune_<ts>.txt for you to transcribe into constants.py.
    Like every Controller Autotune change, they are NOT auto-persisted - on
    restart the gains revert to the constants.py defaults.

INVOCATION
    Press 't' during a normal sim run (after launch, once airborne and able to
    track).  Press 't' again to abort; the in-progress gain is restored to its
    baseline and the aircraft returns to LOITER.

SAFETY / STATUS
    Intended for SIM (--platform sim).  It deliberately drives the aircraft to the
    edge of overshoot/instability on each gain, which is not something to do near
    the ground on real hardware.  UNTESTED end-to-end in this build env (no SITL,
    same caveat as arduplane_autotune) - trial it in sim, watch the first run, and
    refine SWEEP_PLAN / thresholds before trusting the numbers.

For moving the tuned values to the real (different) airframe, see
plant_id_rescale.py - the offline sim->real bridge.
"""
from __future__ import annotations
import glob
import json
import logging
import math
import os
import time
from datetime import datetime

from . import constants as C
from . import file_export
from .constants_suggest import write_suggested_constants
from .constants import AUTOTUNE_CELL_MAX_SAT_FRAC
from .csv_logger import PACKAGE_SHA256
from .controller_autotune_store import ControllerAutotuneStore, DEFAULT_CONST
from .testing.synthetic_injector import SyntheticBalloonInjector

log = logging.getLogger("balloon_chaser")

# ----------------------------- timing / loop -----------------------------
SAMPLE_DT = 0.02          # 50 Hz error sampling (matches the injector / ptest)
# Counts error samples actually taken, so the noise-floor pass can report the
# ACHIEVED rate.  A list, not an int, so the closure can mutate it without a
# global statement; a single append-free counter is atomic enough under the GIL.
_SAMPLE_COUNT = [0]
WARMUP_S = 0.5            # wait for the first real balloon chaser tick before scoring

# Between EVERY trial the aircraft is flown back to STRAIGHT flight - wings level and
# yawless - and held there for STRAIGHT_HOLD_S, so each trial starts from the same
# clean initial attitude (otherwise a trial that ended banked/diving would bias the
# next trial's measured line-up).  Done in FBWA (the FC self-levels), with
# link.level_and_hold VERIFYING from telemetry (roll angle + body yaw rate) that the
# aircraft is really straight.  Throttle is left to the pilot (not commanded).
STRAIGHT_HOLD_S = 3.0            # hold wings-level + yawless this long before each trial
STRAIGHT_TIMEOUT_S = 25.0        # give up trying to level after this and proceed (recovery
                                 # from a steep dive can take a while)
STRAIGHT_ROLL_TOL_DEG = 3.0      # |roll| within this counts as wings level
STRAIGHT_YAWRATE_TOL_DPS = 3.0   # |yaw rate| within this counts as yawless (not turning)
# The belly is only "parallel to the ground" once the NOSE is near level too - otherwise
# the level-out gate would pass while the aircraft is still pulling out of a dive (wings
# level but nose 20 deg down).  So also require the pitch ANGLE near level and the pitch
# RATE low (nose has stopped moving).  The settled FBWA cruise pitch is not exactly 0
# (it is the trim attitude, a few degrees off), so these are intentionally a little loose;
# tighten them if your trim sits closer to 0.  If the nose never settles within tolerance
# the level-out just times out (STRAIGHT_TIMEOUT_S) and proceeds, as before.
STRAIGHT_PITCH_TOL_DEG = 5.0     # |pitch| within this counts as nose level (belly parallel)
STRAIGHT_PITCHRATE_TOL_DPS = 3.0 # |pitch rate| within this counts as nose settled (not moving)

# --------------------------- align-gated scoring --------------------------
# A trial is NOT scored on a fixed window; it drives to the balloon and waits for a
# real, sustained alignment - the flight-path alignment error (ctrl.alignment_errors())
# must hold within ALIGN_TOL_DEG (Pythagorean, the SAME tolerance as the HUD
# "Path Aligned" indicator and the 'p' verdict) for ALIGN_DWELL_S
# continuously - then the sweep skips to the next trial.  If that never happens
# within ALIGN_TIMEOUT_S the trial FAILS (the gain can't hold a stable alignment).  This is
# what makes the score trustworthy: overshoot/ring are read against a response that
# actually settled, not one captured mid-flight.
ALIGN_TOL_DEG = C.ALIGN_TOL_DEG   # alignment tolerance == the one alignment tolerance
ALIGN_DWELL_S = 5.0               # must hold within tolerance this long = a real alignment
ALIGN_TIMEOUT_S = 30.0            # give up waiting for alignment => trial fails (trips)

# ----------------------- cost-based bracket search -----------------------
# We do NOT step-until-it-breaks (edge-finding ratchets, and accepts a backoff
# value it never flew).  Instead each parameter is tried at a FIXED bracket of
# candidate values around its constants.py DEFAULT (always including the default,
# so "leave it alone" can win), every candidate is FLOWN, and the lowest-COST
# candidate is kept - self-validating, bounded, and ratchet-proof.
#   factors apply to a non-zero default; a zero-baseline param (e.g. kd) uses an
#   additive bracket scaled from its 'seed'.
BRACKET_FACTORS = [0.5, 0.7, 1.0, 1.4, 2.0]   # 5-point (finer); 1.0 = unchanged

# Per-entry bracket override.  A parameter whose optimum lies OUTSIDE
# +/-2x of its baseline cannot be found in one pass, and coordinate descent
# only walks toward it if the cost rewards the intermediate steps - which it
# will not if the deciding regime is outside the flown grid.  Entries listed
# here fly a wider net.
# WIDE bracket (tuner switch "Autotune Wide Bracket" = On).  Reaches x5 either
# way instead of x2, for 9 candidates instead of 5 - 1.8x the flight time.
#
# 9 points is the useful choice, not 5-at-wider-spacing.  The headroom gate reads
# the NEXT CANDIDATE UP straight out of the flown matrix, so the step ratio IS the
# gate: at 5 points spanning x25 the ratio is 2.24 and the gate silently becomes
# "must still align at 2.24x", which rejects gains that are perfectly good.  At 9
# points the ratio is 1.495 against the narrow set's 1.400, so the gate keeps
# almost exactly the meaning it was tuned for and only the reach changes.
BRACKET_FACTORS_WIDE = [0.2, 0.3, 0.45, 0.67, 1.0, 1.5, 2.24, 3.34, 5.0]

ENTRY_BRACKETS = {
    "windup_limit_pan": [0.5, 1.0, 1.75, 3.0, 5.0],
}


def _bracket_for(entry, wide=False):
    """Bracket factors THIS entry flies.

    A per-entry override wins over the global set either way: those are hand-
    chosen for a parameter whose optimum is known to sit somewhere awkward, and
    the wide set is a blunter instrument than that knowledge."""
    override = ENTRY_BRACKETS.get(entry.get("attr"))
    if override is not None:
        return override
    return BRACKET_FACTORS_WIDE if wide else BRACKET_FACTORS

# ------------- decision-noise handling (hysteresis + confirmation re-fly) -------------
# A trial's cost is a NOISY sample of a random variable; the argmin over single
# draws favours lucky candidates (winner's curse) and the MAX over grid cells is
# biased upward.  Three guards:
#   1. a windless sim is near-deterministic and the between-trial level-out
#      standardises initial conditions, so every candidate already faces nearly
#      identical conditions (the common-random-numbers idea);
#   2. HYSTERESIS: a challenger only displaces the pass-start baseline when its
#      worst-case cost beats the baseline's by SWITCH_MARGIN_REL plus an absolute
#      floor.  Fewer noise-driven "changed" flags also means parameters freeze
#      earlier for the RIGHT reason, cutting pass count;
#   3. CONFIRMATION RE-FLY: when a switch IS proposed, challenger and baseline are
#      each re-flown once at their OWN worst cell, the two samples averaged, and
#      the decision re-made between the TWO of them only - promoting a third,
#      un-re-flown candidate would re-open the single-sample curse.
SWITCH_MARGIN_REL = 0.10   # challenger must beat baseline worst-case by 10% ...
SWITCH_MARGIN_ABS = 1.0    # ... plus this (cost units, ~seconds of t_align)
CONFIRM_REFLY = 1          # extra samples for challenger+baseline on a switch (0 = off)

# ---------------- measured noise floor (calibrates the margins) ---------------
# SWITCH_MARGIN_REL/ABS above are hand-set numbers guarding a SINGLE-SAMPLE
# comparison, and nothing in the sweep ever measured how repeatable that sample
# is.  If the trial-to-trial spread exceeds the margin the sweep selects noise;
# if it is far below, real improvements are rejected.  Neither was knowable.
# The noise-floor pass flies the UNCHANGED baseline over the whole grid several
# times before any parameter is touched, and reports the spread of the deciding
# statistic (worst-case cost over cells).  The absolute margin is then raised to
# whichever is larger: the hand-set floor, or the measured noise.
# Two independent samples are compared, so the difference has sigma*sqrt(2).
AUTOTUNE_NOISE_FLOOR_REPS = 3    # 0 = skip (and keep the hand-set margins)
AUTOTUNE_NOISE_MARGIN_SIGMAS = 2.0
_NOISE_SIGMA = None              # measured spread of worst-case cost; None = not measured
_MARGIN_ABS_EFF = SWITCH_MARGIN_ABS   # effective absolute margin actually applied
NOALIGN_TIE_MARGIN_DEG = 0.5  # both-grazed comparisons: the challenger must be
                             # this many DEG better to displace the baseline
                             # (at the 1000-scale no-align cost the relative
                             # margin above is ~100 units - unreachable without
                             # aligning - which erased the sub-align gradient the
                             # no-align cost encodes).  Converted to cost units
                             # at the point of use: the no-align cost is not
                             # numerically equal to degrees, so comparing a
                             # degree margin against it directly would be ~25x
                             # too permissive.

# ---------------- near-miss confirmation ("photo-finish" harvest) ----------------
# When a challenger measures strictly BETTER at the worst case but falls short of
# the 10%+1 hysteresis bar, a single noisy sample must not decide the photo-finish
# in EITHER direction.  Instead of keeping-by-default the sweep buys information:
# re-fly BOTH deciders NEARMISS_REFLY more times at their own worst cells, replace
# the deciding cell's cost with the sample MEDIAN, and re-decide at the small
# margin below.  Aligned-regime only: in the no-align regime the closest-approach
# tie-break already IS the margin, and sub-half-degree median gaps are noise.  An
# adopted near-miss skips the standard confirm and still faces the headroom gate.
NEARMISS_REFLY = 2         # extra samples EACH for challenger + baseline (0 = off)
NEARMISS_MARGIN_REL = 0.03 # adoption: challenger median-worst must beat the
NEARMISS_MARGIN_ABS = 0.5  #   baseline's by 3% + 0.5 cost units

# ---------------- successive elimination (worst-first pruning) ----------------
# Cells are flown HARDEST-FIRST and a non-baseline candidate is ELIMINATED the
# moment its running worst-case already exceeds the best completed candidate's
# final worst-case: its true worst can only grow, so it can never be the argmin.
# EXACT - the chosen value is bit-identical to exhaustive evaluation; only
# flights that provably cannot change the outcome are skipped (typically the
# bracket extremes, condemned at the hardest cell).  The BASELINE row always
# flies in full (every decision gate compares against it), and the headroom gate
# completes a pruned neighbour row on demand before reading it.  Checkpoints are
# unchanged: pruned cells simply remain unflown and a resume re-prunes.
SH_PRUNE = True

# ---------------- headroom gate (crude empirical gain margin) ----------------
# The cost is pure performance: nothing in it measures distance to instability.
# Entries flagged "margin": True (gains whose destabilising direction is UP) must
# therefore show HEADROOM: the next candidate UP in value must have aligned at
# EVERY grid cell.  The bracket is ~log-uniform with ratio ~1.4, so that neighbour
# IS the "fly it at 1.4x and require it still align" probe - and it is read FREE
# from the already-flown cost matrix.  Only a winner at the TOP of the bracket
# flies one extra probe, at HEADROOM_FACTOR x its value, at the hardest cell.  On
# failure the choice is demoted ONE bracket step - whose own headroom is then
# proved by the failed winner's everywhere-aligned row - unless that lower step
# itself had no-align cells, in which case the winner is kept and flagged loudly
# for hand-tuning.
HEADROOM_FACTOR = 1.4

# Cost from the align-gated metrics (lower = better).  A no-align candidate is
# penalised above ANY aligned one but still ordered by how close it got (min_err),
# so an AGGRESSIVE bearing where nothing fully aligns still yields a usable gradient.
# Rewarding low jitter (not just "aligned at all") avoids picking a barely-holding
# value.
#
# Weights are biased toward DAMPING: a well-damped tune keeps working when the
# plant gain shifts (competition airframe != SITL model), so overshoot and ring
# are priced to stop the argmin drifting toward fast, lightly-damped tunes.
# t_align is a flight cost here (height spent getting aligned is height not
# available for the dive), not a comfort metric.
# FIXED, and the RATIO between the two is the real setting: it decides which
# gains every sweep selects, so changing it silently re-decides every past
# result.
#
# Time to line up is a flight cost, not a comfort metric - height spent
# lining up is height unavailable for the dive. Overshoot OVERLAPS with ring:
# both are symptoms of a loop that is too hot, but ring counts the
# oscillations directly while overshoot is only a proxy for them. Weighting
# overshoot far above tlock double-counts one symptom and starves the other.
COST_W_TLOCK    = 3.0
COST_W_JITTER = 25.0      # per deg of worst error during the held alignment
# Overshoot overlaps with RING (both symptoms of a too-hot loop, and RING counts
# the oscillations directly), so it is priced below RING; overshoot that settles
# is not a defect, overshoot that keeps ringing is.
COST_W_OVERSHOOT = 8.0
COST_W_RING = 10.0        # per post-approach sign change
NOALIGN_BASE = 1000.0      # flat penalty for never aligning (>> any aligned cost)
NOALIGN_W_MINERR = 1.0     # + per deg of closest approach, to order no-align cases
# SETTLED-ERROR (droop): min_err is the CLOSEST approach, so without this term a
# candidate that dips to 0.51 deg then sits at 3 deg scores like one that settles
# at 0.55 - backwards for a steady-state offset.  Weighted like jitter, since it
# is the same quantity measured after settling instead of during the align dwell.
COST_W_SETTLED = 25.0     # per deg of median |error| over the settled tail
SETTLED_WINDOW_S = 2.0    # tail of the attempt treated as "settled"
# Ring-counter noise deadband: samples within this fraction of the alignment tolerance
# of zero count as sign 0, so sensor noise wiggling about zero (real hardware) is
# not scored as oscillation.  In a clean sim this changes nothing material.
RING_DEADBAND_FRAC = 0.25
# Overshoot yardstick floor: at cells where the scored axis starts nearly aligned
# |e0| is a fraction of a degree, and excursion/|e0| explodes into phantom costs.
OVERSHOOT_E0_FLOOR_DEG = 2.0

# Coordinate descent over the FIXED brackets: the gains are not orthogonal, so we
# re-pick each parameter's best candidate against the others' CURRENT values, pass
# after pass, until no parameter's choice changes (a discrete coordinate-wise
# optimum) or MAX_PASSES.  Because the candidate set is fixed and includes the
# default, this converges to a repeatable fixed point and CANNOT ratchet.
MAX_PASSES = 4            # cap on coordinate-descent passes over the whole plan

# Graceful stop / resume.  Pressing 't' during a sweep aborts it and writes the full
# progress to this checkpoint file (prefix "incomplete_").  On the NEXT launch the
# sweep looks for it: if present (and the config still matches) it RESUMES from where
# it stopped; if absent it starts fresh.  A completed sweep deletes the checkpoint and
# writes the normal controller_autotune_<ts>.txt report instead.
CHECKPOINT_NAME = "incomplete_controller_autotune"      # + _<run stamp>.json
CHECKPOINT_GLOB = "incomplete_controller_autotune*.json"


def _checkpoint_path(ctrl, profile):
    """Where this run's checkpoint lives.

    THE STAMP BELONGS TO THE PROCEDURE, NOT THE INVOCATION.  A resumable process
    keeps one timestamp for its whole life - across aborts, crashes and
    relaunches - so every file it writes carries the moment the procedure
    ORIGINALLY began.  Only a genuinely fresh start takes a new stamp.

    The answer therefore lives in ctrl.run_stamp_for(), which is set from the
    ADOPTED checkpoint when one is resumed.  Deriving it here from "is there a
    file on disk" was wrong twice over: a second run in the same session reused
    the first run's stamp from a stale cache, and a resume flag that was never
    cleared made every later run adopt whatever checkpoint happened to be
    lying there."""
    ts = ctrl.run_stamp_for(profile.key)
    return os.path.join(os.getcwd(), "%s_%s.json" % (profile.checkpoint_name, ts))

# The sweep engine serves two operator-facing tools.  Everything about them is
# identical - same SWEEP_PLAN, same coordinate descent, same cost, same climb
# protection, same checkpoint/resume, same live-gain behaviour - except WHICH
# CELLS a candidate is flown at, and therefore which files the run owns.
#
#   't' (SWEEP_PROFILE)  flies the tuner's pan x tilt grid and scores each
#                        candidate on its WORST cell, so a winner is validated
#                        across the envelope.
#   'y' (CELL_PROFILE)   flies ONE operator-chosen bearing and scores there.
#                        The result is the best achievable AT THAT BEARING; it
#                        carries no envelope guarantee, which is the whole point
#                        when the question is "what is wrong at this cell".
#
# The file names are per-profile because the two runs must never resume from or
# overwrite each other's state: a 'y' checkpoint holds a single-cell result
# matrix that means nothing to a 't' resume.  _grid_signature also records the
# profile, so even a hand-renamed file is refused rather than misread.
class _Profile:
    __slots__ = ('name', 'key', 'checkpoint_name', 'checkpoint_glob',
                 'report_prefix', 'suggest_name', 'single_cell',
                 'use_constants_attr')

    def __init__(self, name, key, checkpoint_name, checkpoint_glob,
                 report_prefix, suggest_name, single_cell,
                 use_constants_attr):
        self.name = name
        self.key = key
        self.checkpoint_name = checkpoint_name
        self.checkpoint_glob = checkpoint_glob
        self.report_prefix = report_prefix
        self.suggest_name = suggest_name
        self.single_cell = single_cell
        self.use_constants_attr = use_constants_attr


SWEEP_PROFILE = _Profile(
    "CONTROLLER AUTOTUNE SWEEP", "t",
    CHECKPOINT_NAME, CHECKPOINT_GLOB,
    "controller_autotune", "suggested_constants_t", False,
    "autotune_use_constants")

CELL_PROFILE = _Profile(
    "SINGLE-CELL AUTOTUNE", "y",
    "incomplete_cell_autotune", "incomplete_cell_autotune*.json",
    "cell_autotune", "suggested_constants_y", True,
    "cell_autotune_use_constants")
CHECKPOINT_VERSION = 6   # bumped when the decision rules change - older
                         # checkpoints are incomparable.

# Altitude protection: if AGL altitude falls below the protection floor during a
# trial (the aggressive dives bleed height), pause and climb back to the climb-to
# altitude, then RE-FLY the interrupted trial.  After PROTECT_MAX_RETRIES the last
# (breached) result is used as-is (it scores as a no-align - the maneuver can't be
# completed from a safe altitude, which is itself meaningful).  The floor and
# climb-to are live Gain-Tuner knobs (ctrl.autotune_protect_floor_m /
# ctrl.autotune_protect_climb_to_m, defaulting to the AUTOTUNE_PROTECT_* constants);
# read them from the controller at the moment of the check, never cache a constant.
PROTECT_MAX_RETRIES = 2

# The sweep plan: every parameter to tune, in order, with the bearing that
# exercises it, the error axis to score, and the search DIRECTION.
#   axis : 'pan' (lateral) or 'tilt' (dive) - the dominant error for that bearing
#   dir  : 'raise' (gains/limits: more = more aggressive) or 'lower' (damping/
#          leak/filter terms: less = more overshoot); default 'raise'
#   seed : starting value when the baseline is 0 (can't be scaled), e.g. kd
# Every trial is flown against a constant world BEARING (force_direction=True on
# the injector): a clean step->settle with no overflight.  That is not a choice
# per entry - the grid runs down to lat=-90, and under the ptest's distance rule
# those cells would resolve to reachable ground points, so a trial would fly
# THROUGH its own balloon and corrupt the settle/jitter metrics it is measuring.
#
# Scope note: the kd-LPF / FF-LPF entries are filter parameters - the
# overshoot criterion is weakly defined for them; treat
# their results as suggestive, and refine the bearing/dir or use a different
# objective if they don't behave.
PAN = 40.0        # AGGRESSIVE lateral demand for pan-axis trials
TILT = 25.0       # dive during the lateral trials
PITCH_TILT_DEG = 40.0     # AGGRESSIVE dive demand for pitch-axis trials
# Combined-validation bearing: BOTH axes off-boresight at once, so the final trial
# exercises the lateral and pitch loops together and confirms the full gain set
# holds (single-gain trials can each pass while the combination is marginal).
VALIDATE_PAN = 40.0
VALIDATE_TILT = 25.0

# PLAN ORDER = SEQUENTIAL LOOP CLOSURE (inner -> outer), the cascade-tuning rule:
# close the inner loop first so the outer loop is tuned against a SETTLED plant.
# Tuning the outer bank gain before the inner roll loop settles re-tunes it
# against a loop still moving underneath - extra passes, premature freezes.  Groups:
#   1. INNER ROLL       bank error -> roll-rate demand (roll_gain)
#   2. OUTER PAN        pan error  -> bank command (small-signal + authority + lead)
#   3. PAN INTEGRATORS  steady-disturbance machinery (weakly identified - see note)
#   4. PITCH            P first, then its integrator machinery
# Per-entry flag:
#   margin : True = destabilising direction is UP -> headroom-gated (see above)
SWEEP_PLAN = [
    # === 1. INNER ROLL loop (score the pan error - roll quality shows there) ===
    {"attr": "roll_gain",             "pan": PAN, "tilt": TILT, "axis": "pan", "dir": "raise", "margin": True},
    # === 2. OUTER PAN loop ===
    # Proportional gain: the single linear knob pan_p_gain (deg bank per
    # deg pan error).
    {"attr": "pan_p_gain",        "pan": PAN, "tilt": TILT, "axis": "pan", "dir": "raise", "margin": True},
    # bank_lim_deg is intentionally NOT swept: it is the manual bank-angle
    # clip (an authority/envelope limit), not a loop-shaping gain.
    {"attr": "pan_kd",                "pan": PAN, "tilt": TILT, "axis": "pan", "dir": "lower", "seed": 0.05, "margin": True},  # lead: damps; UP also amplifies noise -> gated
    {"attr": "pan_kd_rate",           "pan": PAN, "tilt": TILT, "axis": "pan", "dir": "raise", "margin": True},  # D-term LPF rate: INERT while pan_kd = 0, hence ordered after kd
    # === 3. PAN INTEGRATOR group ===
    # These null STEADY disturbances (wind, trim asymmetry) that a clean sim does
    # not have, so a step test exercises them only weakly.  Expect the hysteresis
    # gate to keep their defaults; treat any change they DO make as suggestive.
    # They become properly observable on the real aircraft (real wind exists
    # outdoors) - re-verify them there with the manual tuner.
    {"attr": "pan_ki",            "pan": PAN, "tilt": TILT, "axis": "pan", "dir": "raise", "margin": True},
    {"attr": "windup_limit_pan",  "pan": PAN, "tilt": TILT, "axis": "pan", "dir": "raise"},
    # === 4. PITCH (score the tilt error): P first, then integrator machinery ===
    {"attr": "pitch_gain",                    "pan": 0.0, "tilt": PITCH_TILT_DEG, "axis": "tilt", "dir": "raise", "margin": True},
    {"attr": "pitch_ki",                      "pan": 0.0, "tilt": PITCH_TILT_DEG, "axis": "tilt", "dir": "raise", "margin": True},
    {"attr": "windup_limit_pitch",            "pan": 0.0, "tilt": PITCH_TILT_DEG, "axis": "tilt", "dir": "raise"},
]

# STANDING CAVEATS:
#   - the LPF rates trade lag against sensor noise the clean sim does not model,
#     so sim runs mildly favour LARGER values (less filtering); the fixed bracket
#     bounds any drift to ONE step per bake cycle and hysteresis usually holds the
#     default.  Re-verify both rates on the real aircraft.
#   - pan_kd_rate only does anything while pan_kd != 0.

# ---------------- pan-axis grid cap ----------------
# Pan-axis entries only fly maneuver cells with tilt >= this value; pitch entries
# keep the FULL grid (steep cells are legitimate pitch tests; the no-lock
# tie-break keeps them informative), and the combined VALIDATION still flies the
# hardest GLOBAL cell.  The cap is set to cover the sphere.  It is a plain constant: the
# sweep flies what this says.  Steep rows that tie at the no-align penalty teach
# nothing - a visible outcome in the report, not a reason to narrow the sweep
# silently.
GRID_TILT_CAP_DEG = -90.0


def _noise_floor_pass(ctrl, grid, axis, abort, progress, reps):
    """Fly the UNCHANGED baseline over the whole grid `reps` times and measure
    the spread of the deciding statistic (worst-case cost over cells).

    Nothing is tuned here.  The point is to answer a question the sweep has
    never asked: how much does an identical configuration vary between runs?
    Every switch decision downstream is a comparison of two single samples, so
    without this number the margins are guesses and a 'winner' may be noise.

    Replicates the FULL grid rather than one cell because the statistic being
    compared is a max over cells, and a max over noisy values has a spread of
    its own that a single cell cannot reproduce.

    Sets _NOISE_SIGMA and raises _MARGIN_ABS_EFF.  Returns a dict for the
    report, or None if it could not complete (abort / no metrics)."""
    global _NOISE_SIGMA, _MARGIN_ABS_EFF
    if reps < 2:
        log.info("[CATUNE] noise-floor pass disabled (reps=%d) - margins stay at "
                 "their hand-set values.", reps)
        return None
    log.info("=" * 60)
    log.info("[CATUNE] NOISE FLOOR: flying the baseline %d x over %d cell(s). "
             "Nothing is tuned; this calibrates the switch margins.", reps, len(grid))
    # Pass 0.  Its horizon IS known exactly - reps x cells - unlike the tuning
    # passes.  Without this the counter still read 0/0 here, because start_pass()
    # is only called inside the tuning loop that runs afterwards.
    progress.start_pass(0, reps * len(grid))
    worsts, per_rep = [], []
    _t_pass0 = time.monotonic()
    _samples0 = _SAMPLE_COUNT[0]
    for r in range(reps):
        costs = []
        for gi in range(len(grid)):
            if abort.is_set() or ctrl.mode != 'autotune':
                log.warning("[CATUNE] noise-floor pass aborted.")
                return None
            pan, tilt = grid[gi]
            m = _fly_with_protection(ctrl, pan, tilt, axis,
                                     abort, progress, "noise_floor")
            if m is None:
                log.warning("[CATUNE] noise-floor pass: cell %d/%d produced no "
                            "metrics - abandoning the pass.", gi + 1, len(grid))
                return None
            costs.append(_cost(m))
        w = max(costs)
        worsts.append(w)
        per_rep.append(costs)
        log.info("[CATUNE] noise floor rep %d/%d: worst-case cost %.2f "
                 "(per-cell %s)", r + 1, reps, w,
                 ", ".join("%.1f" % c for c in costs))

    n = len(worsts)
    mean = sum(worsts) / n
    # Sample standard deviation (n-1).  With 3 reps this is a coarse estimate -
    # it is reported with the raw values so it can be judged, not just trusted.
    var = sum((w - mean) ** 2 for w in worsts) / max(1, n - 1)
    sigma = var ** 0.5
    spread = max(worsts) - min(worsts)
    _NOISE_SIGMA = sigma
    # Two independent samples are compared, so the difference carries sigma*sqrt(2).
    needed = AUTOTUNE_NOISE_MARGIN_SIGMAS * sigma * (2.0 ** 0.5)
    _MARGIN_ABS_EFF = max(SWITCH_MARGIN_ABS, needed)
    # Achieved error-sampling rate over the pass.  time.sleep(SAMPLE_DT) is a
    # FLOOR on the interval, not a rate, so the real figure is always below
    # 1/SAMPLE_DT and was never measured anywhere.  It matters because every
    # trial metric (t_align, jitter, settled) is computed from these samples.
    _elapsed = max(1e-6, time.monotonic() - _t_pass0)
    samp_hz = (_SAMPLE_COUNT[0] - _samples0) / _elapsed
    log.info("[CATUNE] NOISE FLOOR: worst-case cost %.2f +/- %.2f (1 sigma), "
             "range %.2f over %d reps; error sampling achieved %.1f Hz "
             "(nominal %.0f).", mean, sigma, spread, n, samp_hz, 1.0 / SAMPLE_DT)
    if samp_hz < 0.7 / SAMPLE_DT:
        log.warning("[CATUNE] sampling is %.0f%% of nominal - trial metrics are built "
                    "from correspondingly fewer points, which widens their scatter and "
                    "therefore the switch margin measured above.",
                    100.0 * samp_hz * SAMPLE_DT)
    if _MARGIN_ABS_EFF > SWITCH_MARGIN_ABS + 1e-9:
        log.warning("[CATUNE] measured noise EXCEEDS the hand-set margin: absolute "
                    "switch margin raised %.2f -> %.2f (%.1f sigma on a "
                    "two-sample difference).  Differences smaller than this are "
                    "not distinguishable from run-to-run scatter.",
                    SWITCH_MARGIN_ABS, _MARGIN_ABS_EFF, AUTOTUNE_NOISE_MARGIN_SIGMAS)
    else:
        log.info("[CATUNE] measured noise is below the hand-set margin %.2f - "
                 "margins unchanged.", SWITCH_MARGIN_ABS)
    if mean > 0 and sigma / max(1e-9, mean) > 0.25:
        log.warning("[CATUNE] noise is %.0f%% of the mean cost.  A sweep at this "
                    "repeatability will struggle to resolve anything smaller "
                    "than a large effect; consider fixing the source (attitude "
                    "stream rate, trial initial conditions) before a long run.",
                    100.0 * sigma / mean)
    log.info("=" * 60)
    _sats = [m.get("sat_frac") for row in per_rep for m in (row.values() if isinstance(row, dict) else [])
             if isinstance(m, dict) and m.get("sat_frac") is not None]
    return {"sat_frac": (sum(_sats) / len(_sats)) if _sats else None,
            "reps": n, "worsts": worsts, "per_cell": per_rep, "mean": mean,
            "sigma": sigma, "range": spread, "sample_hz": samp_hz,
            "margin_abs_before": SWITCH_MARGIN_ABS,
            "margin_abs_after": _MARGIN_ABS_EFF}


def _entry_grid(entry, grid):
    """The maneuver cells THIS entry flies: the full grid for pitch entries, the
    tilt-capped subset (tilt >= GRID_TILT_CAP_DEG) for pan-axis
    entries.  A deterministic function of the global grid and a CONSTANT, so
    checkpoint cell indices stay consistent across resumes; gentlest->hardest
    order is preserved, so grid[-1] is still the hardest cell the entry flies."""
    if entry.get("axis") == "pan":
        capped = [c for c in grid if c[1] >= GRID_TILT_CAP_DEG]
        if capped:                      # defensive: never hand back an empty grid
            return capped
    return grid


def _pass_trials(entries, grid, wide=False):
    """EXACT planned trial count for one pass over `entries` - summed per entry,
    since the pan-axis grid cap means entries fly different cell counts."""
    return sum(len(_entry_grid(e, grid)) * len(_bracket_for(e, wide)) for e in entries)


class _Progress:
    """Trial counter for the sweep's log lines.  The only horizon known ahead of
    time is the CURRENT pass (params-still-moving x candidates-per-param); the grand
    total is data-dependent, since later passes re-tune only the gains whose choice
    is still changing.  So the counter shows 'P<pass> trial x/y' with y reset each
    pass, plus a cumulative running total across all passes."""
    def __init__(self):
        self.pass_idx = 0
        self.pass_total = 0          # trials planned this pass (summed per entry)
        self.in_pass = 0             # trials completed this pass
        self.total = 0               # cumulative trials completed, all passes

    def start_pass(self, pass_idx, pass_trials):
        self.pass_idx = pass_idx
        self.pass_total = int(pass_trials)
        self.in_pass = 0

    def tick(self):
        """Count one completed trial; return a compact 'P1 trial 12/100' tag."""
        self.in_pass += 1
        self.total += 1
        return self.tag()

    def counts(self):
        """'12/100' when this pass's horizon is known, '12/Total not yet known'
        before a pass has been planned.  The GRAND total across all passes is
        never shown because it is not knowable: later passes re-tune only the
        gains whose choice is still moving, so the horizon is data-dependent.
        Better an honest 'not yet known' than a number that is wrong."""
        if self.pass_total > 0:
            return "%d/%d" % (self.in_pass, self.pass_total)
        return "%d/Total not yet known" % self.in_pass

    def tag(self):
        """Current 'P1 trial 12/100' tag without advancing the counter."""
        return "P%d trial %s" % (self.pass_idx, self.counts())


def _straighten(ctrl, abort):
    """Between trials: fly the aircraft back to STRAIGHT-AND-LEVEL in FBWA - the FC
    self-levels and link.level_and_hold VERIFIES from telemetry that it is really
    straight AND level (roll angle + yaw rate + pitch angle + pitch rate, i.e. wings
    level, not turning, nose near level, nose not moving) for STRAIGHT_HOLD_S, then
    restores ACRO for the trial.  Throttle is the operator's set value throughout.  The
    HUD "STRAIGHT" timer (driven by ctrl.straighten_start_t and the link's published
    held-level instant) only counts once the belly is parallel to the ground, so it no
    longer ticks while the aircraft is still pulling out of a dive.  Abort-aware."""
    ctrl.autotune_pause_tracking()
    throttle_frac = max(0.0, min(1.0, float(ctrl.throttle_pct) / 100.0))
    ctrl.straighten_start_t = time.monotonic()     # start the HUD "STRAIGHT" timer
    try:
        achieved = ctrl.link.level_and_hold(
            STRAIGHT_ROLL_TOL_DEG, STRAIGHT_YAWRATE_TOL_DPS, STRAIGHT_HOLD_S,
            STRAIGHT_TIMEOUT_S, throttle_frac=throttle_frac, abort=abort,
            pitch_tol_deg=STRAIGHT_PITCH_TOL_DEG,
            pitch_rate_tol_dps=STRAIGHT_PITCHRATE_TOL_DPS)
    finally:
        ctrl.straighten_start_t = None             # stop the timer (hides the HUD line)
    if not achieved and not abort.is_set():
        log.info("[CATUNE] could not hold wings-level+yawless within %.0fs before "
                 "this trial - it starts from a non-level attitude.", STRAIGHT_TIMEOUT_S)
    if abort.is_set() or ctrl.mode != 'autotune':
        return
    ctrl.enter_acro('back to ACRO for the trial')


def _frange(lo, hi, step):
    """Inclusive float range from lo to hi in increments of |step|, in WHICHEVER
    direction hi lies (so lo=0, hi=-80, step=5 -> [0, -5, ... -80], a dive sweep).
    step<=0 or hi==lo -> just [lo]."""
    if step <= 0 or hi == lo:
        return [round(lo, 4)]
    n = int(math.floor(abs(hi - lo) / step + 1e-9))
    sign = 1.0 if hi >= lo else -1.0
    return [round(lo + sign * step * i, 4) for i in range(n + 1)]


def _coarse_axis(vals):
    """Reduce a swept axis to its endpoints and midpoint: {min, mid, max}.
    Fewer if the axis has <3 distinct values (deduped by the set)."""
    if len(vals) <= 3:
        return sorted(set(vals))
    return sorted({vals[0], vals[len(vals) // 2], vals[-1]})


def _maneuver_grid(ctrl, profile=SWEEP_PROFILE):
    """The autotune maneuver grid: the (pan, tilt) balloon bearings each candidate
    gain set is flown at, spherical az/el.  Tilt is the elevation (NEGATIVE =
    nose-down); pan is the azimuth (may be past +-90, rear hemisphere, C-04).
    Every cell is a valid direction - nothing is dropped, clamped, or budget-
    limited.  Ordered gentlest -> most aggressive by hypot(), so the aircraft
    warms up gently and grid[-1] is the hardest cell (the combined-gain
    validation flies it).  The hypot() order is a warmup heuristic, not a
    correctness requirement.

    DEFAULT is a COARSE 3x3: {min, mid, max} pan x {min, mid, max} tilt (<=9
    cells).  The per-candidate choice keys only on the WORST cell (see _tune_one),
    and coupled-gain failures surface at the ENVELOPE - the corners and edges - so
    a 3x3 brackets the worst case the cost actually reads while cutting a 17x17
    grid ~32x.  It cannot miss an EDGE failure (only a deep-interior one); the
    full cross-product below is the audit that would catch that.

    AUDIT: set autotune_full_grid truthy to fly the FULL pan x tilt cross-product
    (the original dense grid) - a periodic ground-truth check that a winner tuned
    on the coarse grid still holds everywhere.  Coordinate descent, the multi-pass
    coupling handling, and worst-case scoring are identical in both modes; only
    the set of test conditions differs.  The grid mode is part of the checkpoint
    signature (_grid_signature), so switching modes starts a fresh sweep rather
    than resuming across incomparable cell sets."""
    if profile.single_cell:
        # 'y': the single operator-chosen bearing, in the same spherical frame
        # and the same sign convention as every other cell (latitude NEGATIVE =
        # nose-down).  No ordering to do and no worst-case to take.
        return [(float(ctrl.autotune_cell_lon_deg),
                 float(ctrl.autotune_cell_lat_deg))]
    pans = sorted(set(_frange(float(ctrl.autotune_pan_min), float(ctrl.autotune_pan_max),
                              float(ctrl.autotune_pan_step))))
    tilts = sorted(set(_frange(float(ctrl.autotune_tilt_min), float(ctrl.autotune_tilt_max),
                               float(ctrl.autotune_tilt_step))))
    if bool(getattr(ctrl, 'autotune_full_grid', False)):
        grid = [(p, t) for t in tilts for p in pans]          # AUDIT: full cross-product
    else:
        grid = [(p, t) for t in _coarse_axis(tilts) for p in _coarse_axis(pans)]  # coarse 3x3
    grid.sort(key=lambda c: math.hypot(c[0], c[1]))    # gentlest first, hardest last
    return grid


def _below_floor(ctrl):
    """True if AGL altitude has fallen below the (live) protection floor."""
    try:
        return float(ctrl.link.relative_alt) < float(ctrl.autotune_protect_floor_m)
    except Exception:
        return False


def _protect_climb(ctrl, abort):
    """Altitude protection: pause tracking, climb back to the ceiling, restore ACRO.
    Abort-aware (a 't' abort during the climb stops it)."""
    climb_to = float(ctrl.autotune_protect_climb_to_m)
    log.info("[CATUNE] altitude %.0fm < floor %.0fm - climbing to %.0fm and re-flying.",
             getattr(ctrl.link, 'relative_alt', -1.0),
             float(ctrl.autotune_protect_floor_m), climb_to)
    # APPEND, do not replace: the operator loses track of which test is running
    # otherwise, and the climb can take a while.  Split on ' | ' first so repeated
    # protections do not stack suffixes.
    _status_base = (getattr(ctrl, 'autotune_status', '') or '').split(' | ')[0]
    ctrl.autotune_status = ((_status_base + " | ") if _status_base else "") + \
        "Alt protect: climbing to %.0fm" % climb_to
    ctrl.autotune_pause_tracking()                 # drop injector/tracking during the climb
    ctrl.link.climb_to(climb_to, abort)
    if _status_base:                               # climb over: back to the test line
        ctrl.autotune_status = _status_base
    if abort.is_set() or ctrl.mode != 'autotune':
        return
    ctrl.enter_acro('restore ACRO for the next trial')


# (pan, tilt) -> {'aligned': n, 'attempts': n}.  Filled by _fly_with_protection so
# EVERY trial is counted, including the near-miss, confirm and headroom re-flies.
# worst[] aggregates across cells and discards which one failed, so without this
# a cell that no candidate can ever win is otherwise invisible until someone
# goes through the logs by hand.
_CELL_ALIGNS = {}


def _fly_with_protection(ctrl, pan, tilt, axis, abort, progress, attr):
    """Fly ONE grid-cell trial at (pan,tilt) with altitude protection: climb first if
    already below the floor, fly, and if the trial breaches the floor mid-way climb
    back to the ceiling and RE-FLY - up to PROTECT_MAX_RETRIES.  Counts as ONE grid
    evaluation on the progress meter regardless of re-flies (when progress is given).
    Returns the metrics dict (the last one if retries are exhausted, which scores as
    no-align), or None on abort."""
    if progress is not None:
        progress.tick()                            # count this grid-eval ONCE, up front
    m = None
    for _attempt in range(PROTECT_MAX_RETRIES + 1):
        if _below_floor(ctrl):
            _protect_climb(ctrl, abort)
        if abort.is_set() or ctrl.mode != 'autotune':
            return None
        if progress is not None:               # show THIS test through level-out + line-up
            ctrl.autotune_status = "test %s  %s @(%+.0f,%+.0f)" % (
                progress.counts(), attr, pan, tilt)
        _straighten(ctrl, abort)               # active level-out (no HUD message)
        if abort.is_set() or ctrl.mode != 'autotune':
            return None
        # Straight achieved -> start the HUD's per-test SECONDS counter: it counts up
        # from here toward ALIGN_TIMEOUT_S, then freezes at the test's end.
        ctrl.autotune_test_t0 = time.monotonic()
        ctrl.autotune_test_done_s = None
        m = _run_trial(ctrl, pan, tilt, axis, abort)
        if m is not None:
            _rec = _CELL_ALIGNS.setdefault((round(float(pan), 1), round(float(tilt), 1)),
                                          {'aligned': 0, 'attempts': 0})
            _rec['attempts'] += 1
            if m.get('aligned'):
                _rec['aligned'] += 1
        if m is None:
            return None
        # Test ended -> freeze the counter at the elapsed time (time-to-align on a pass,
        # ~the timeout on a skip).  A re-fly (altitude breach) resets it at the next level-out.
        ctrl.autotune_test_done_s = time.monotonic() - ctrl.autotune_test_t0
        if not m.get("protect"):
            break                                  # clean trial (no altitude breach)
        # breached mid-trial -> loop: the floor check above climbs, then re-fly
    return m


def _inject_arm_warmup(ctrl, lon_deg, lat_deg, abort):
    """Shared trial preamble: anchor the synthetic balloon, arm tracking, re-lock
    the gimbal, and wait for the first real balloon chaser tick.  Returns the live
    injector, or None on precondition failure / abort (caller pauses + bails).

    VOCABULARY.  This is the boundary where an autotune GRID CELL becomes an
    injected BALLOON POSITION, and the naming changes with it.  The grid above
    calls its two axes pan/tilt because they name which control loop a cell is
    stressing; from here down they are the balloon's longitude and latitude,
    because that is where the balloon IS.  Same two numbers, same spherical frame -
    only the thing being described changes."""
    # NOTE: the moving-balloon switch (point_balloon_move_enable) is deliberately
    # NOT consulted here - the autotune balloon is ALWAYS stationary.  Balloon motion
    # is a ptest-only feature (applied in point_balloon_chaser.py); the sweep flies a fixed
    # DIRECTION anchor, which the moving position path must never touch.  Do not add
    # speed= to this call.
    #
    # force_direction keeps the sweep independent of the operator's Distance m
    # cell: a trial is always flown against an unreachable bearing.  Without it the
    # grid's downward cells (tilt runs to -90) resolve to ground points the aircraft
    # flies through mid-trial.
    inj = SyntheticBalloonInjector(
        ctrl.link, ctrl.sensor, lon_deg, lat_deg,
        force_direction=True)
    ctrl.set_point_injector(inj)
    if not inj.start():
        ctrl.autotune_pause_tracking()
        return None
    # VERIFY the FC is actually in ACRO before flying the trial.  Seconds of
    # level-out / protection-climb elapse between the ACRO restore and this point,
    # and the script claims control only in ACRO, so a trial silently flown in
    # another mode flies that mode, not the gains under test.  current_mode is fed by
    # HEARTBEAT; set_mode_acro() blocks until the FC confirms and raises
    # TimeoutError after 30 s, so a persistent refusal stops the sweep loudly
    # instead of flying the trial in the wrong mode.
    if ctrl.link.current_mode != 4:                # ArduPlane ACRO = 4
        log.warning('[CATUNE] FC not in ACRO at trial start (mode %s) - re-commanding.',
                    ctrl.link.current_mode)
    ctrl.enter_acro('trial start')                 # unconditional: see enter_acro
    # ptest_begin resets integrators so every trial starts clean and comparable.
    ctrl.ptest_begin(lon_deg, lat_deg)
    # ptest_begin's sensor.set_active(True) frees the gimbal in the sim sensor;
    # re-lock so it stays frozen (oracle drives the cascade; gimbal aim irrelevant).
    ctrl.sensor.set_gimbal_locked(True)
    t0 = time.monotonic()
    while time.monotonic() - t0 < WARMUP_S:
        if abort.is_set() or ctrl.mode != 'autotune':
            ctrl.autotune_pause_tracking()
            return None
        if ctrl.cascade_has_run_since_inject():
            break
        time.sleep(SAMPLE_DT)
    return inj


def _settled_err(mags):
    """Median |error| over the last SETTLED_WINDOW_S of an attempt.

    MEDIAN, not mean: the tail can contain a few large samples if the aircraft
    is still ringing, and the quantity of interest is the standing offset the
    loop cannot remove, not the ring amplitude (which overshoot/ring already
    score).  Returns 180.0 when there is no usable sample, matching min_err's
    convention so a dead trial is not accidentally cheap."""
    if not mags:
        return 180.0
    t_end = mags[-1][0]
    tail = [m for (tt, m) in mags if tt >= t_end - SETTLED_WINDOW_S]
    if not tail:
        tail = [mags[-1][1]]
    tail.sort()
    n = len(tail)
    return tail[n // 2] if n % 2 else 0.5 * (tail[n // 2 - 1] + tail[n // 2])


def _overshoot_ring(errs, e0):
    """Overshoot (furthest excursion past zero as a fraction of |e0|) and ring
    count (sign changes after the error first nears zero) from a signed-error
    trajectory on the excited axis."""
    if not errs or abs(e0) < 1e-6:
        return 0.0, 0
    opp = max(0.0, -min(errs)) if e0 > 0 else max(0.0, max(errs))
    # Floor the yardstick (see OVERSHOOT_E0_FLOOR_DEG): overshoot means
    # "excursion relative to a MEANINGFUL initial error", never division by
    # a near-zero |e0|.
    overshoot = opp / max(abs(e0), OVERSHOOT_E0_FLOOR_DEG)
    ring = 0
    prev = None
    reached = False
    for x in errs:
        if abs(x) < 0.3 * abs(e0):
            reached = True
        if reached:
            # Noise deadband (RING_DEADBAND_FRAC x align tol): samples wiggling
            # inside a small band about zero count as sign 0, so sensor noise on
            # real hardware is not scored as oscillation.  No-op in a clean sim.
            if abs(x) < RING_DEADBAND_FRAC * ALIGN_TOL_DEG:
                s = 0
            else:
                s = (x > 0) - (x < 0)
            if s != 0:
                if prev is not None and s != prev:
                    ring += 1
                prev = s
    return overshoot, ring


def _line_up(ctrl, axis, abort):
    """Drive-to-align measurement.  The balloon must already be set; this polls
    ctrl.alignment_errors() (the flight-path error) until the Pythagorean error
    holds within ALIGN_TOL_DEG for ALIGN_DWELL_S CONTINUOUSLY - a real, sustained
    alignment that matches the HUD 'Path Aligned' indicator - or ALIGN_TIMEOUT_S
    elapses.  Returns a metrics dict, or
    None on abort:
        aligned  : True iff a sustained alignment was achieved before the timeout
        overshoot: peak excursion past the balloon on the excited axis, /|e0|
        ring     : sign-change count on the excited axis after first nearing zero
        t_align   : seconds until the (held) alignment band was first entered (or None)
        jitter   : worst error magnitude (deg) seen during the held alignment (or None)
        min_err  : closest approach (deg, Pythagorean) over the whole attempt -
                   orders no-align candidates so an aggressive bearing where nothing
                   fully aligns still gives the cost a usable gradient
        settled  : median |error| (deg) over the last SETTLED_WINDOW_S of the
                   attempt.  Unlike min_err this is a SETTLED measure, so a
                   standing offset (integrator-clamp droop) is visible to the
                   cost instead of being hidden by one lucky close approach
    A failure to align IS a poor-cost signal: the gain can't hold the aircraft on
    target.  Overshoot/ring are computed against a response that actually settled
    (or against the whole timeout window if it never did)."""
    errs = []                 # signed error on the excited axis, over the attempt
    mags = []                 # (t, Pythagorean magnitude) for the settled-tail measure
    e0 = None
    align_start = None         # t at which the current in-band run began
    t_align = None
    jitter = 0.0
    min_err = float("inf")
    n_sat = 0                 # samples with the roll command against the bank limit
    n_samp = 0
    t0 = time.monotonic()
    while True:
        now = time.monotonic()
        if now - t0 >= ALIGN_TIMEOUT_S:
            break
        if abort.is_set() or ctrl.mode != 'autotune':
            return None
        if _below_floor(ctrl):                # altitude breached mid-trial -> bail for a climb
            overshoot, ring = _overshoot_ring(errs, e0 if e0 is not None else 0.0)
            return {"aligned": False, "overshoot": overshoot, "ring": ring,
                    "t_align": None, "jitter": None,
                    "min_err": (min_err if min_err != float("inf") else 180.0),
                    "settled": _settled_err(mags), "protect": True,
                    "sat_frac": (n_sat / n_samp) if n_samp else 0.0}
        err = ctrl.alignment_errors()
        if err is None:                       # flight path undefined -> wait
            time.sleep(SAMPLE_DT)
            continue
        pe, te = err
        axis_err = pe if axis == "pan" else te
        mag = math.hypot(pe, te)
        min_err = min(min_err, mag)
        _SAMPLE_COUNT[0] += 1
        # Saturation fraction.  A sample taken while the roll command is pinned at
        # the bank limit carries no gain information: the command is the limit, not
        # the law's output, so the trial is momentarily open-loop.  't' tolerates
        # this because it scores the WORST of several cells; 'y' has only this one,
        # so it checks the fraction before tuning (see AUTOTUNE_CELL_MAX_SAT_FRAC).
        n_samp += 1
        if getattr(ctrl, 'roll_saturated', False):
            n_sat += 1
        t = now - t0
        mags.append((t, mag))
        if e0 is None:
            e0 = axis_err
        errs.append(axis_err)
        if mag <= ALIGN_TOL_DEG:
            if align_start is None:
                align_start = t
                jitter = 0.0
            jitter = max(jitter, mag)
            if t - align_start >= ALIGN_DWELL_S:
                t_align = align_start
                break
        else:
            align_start = None                 # fell out of the band - alignment resets
        time.sleep(SAMPLE_DT)
    overshoot, ring = _overshoot_ring(errs, e0 if e0 is not None else 0.0)
    if min_err == float("inf"):
        min_err = 180.0                       # never got a usable sample
    return {"aligned": t_align is not None, "overshoot": overshoot, "ring": ring,
            "t_align": t_align, "jitter": (jitter if t_align is not None else None),
            "min_err": min_err, "settled": _settled_err(mags), "protect": False,
            "sat_frac": (n_sat / n_samp) if n_samp else 0.0}


def _run_trial(ctrl, lon_deg, lat_deg, axis, abort):
    """STEP trial: anchor a constant bearing and drive to a
    sustained alignment.  Returns the _line_up metrics dict, or None on failure/abort.

    lon_deg/lat_deg place the BALLOON; `axis` names the CONTROL loop being scored
    ('pan' = lateral, 'tilt' = dive).  Two different meanings, hence two different
    vocabularies - see _inject_arm_warmup."""
    if _inject_arm_warmup(ctrl, lon_deg, lat_deg, abort) is None:
        return None
    m = _line_up(ctrl, axis, abort)
    ctrl.autotune_pause_tracking()
    return m


def _cost(m):
    """Scalar cost of a trial's align-gated metrics; LOWER is better.  An aligned
    candidate is rewarded for fast, low-jitter, non-oscillating line-up; a
    no-align candidate is penalised above ANY aligned one but still ordered by its
    closest approach, so the search keeps a usable gradient even on an aggressive
    bearing where nothing fully aligns."""
    if m is None:
        return float("inf")
    if m.get("aligned"):
        # An aligned trial ends at align+dwell, so its settled tail IS the alignment
        # dwell and `settled` would double-count `jitter`; jitter is the
        # stricter (worst-case) of the two, so it alone carries that term.
        return (COST_W_TLOCK * (m["t_align"] or 0.0)
                + COST_W_JITTER * (m["jitter"] or 0.0)
                + COST_W_OVERSHOOT * max(0.0, m["overshoot"])
                + COST_W_RING * m["ring"])
    # No alignment: `settled` is the only term that sees a standing offset, and it
    # is what separates "settled just outside the band" from "never converged".
    return (NOALIGN_BASE
            + NOALIGN_W_MINERR * m.get("min_err", 180.0)
            + COST_W_SETTLED * m.get("settled", 180.0))


# ---- pure decision helpers (unit-testable; no flight / no side effects) ----

def _value_rank(cands):
    """Candidate indices sorted by VALUE ascending.  Neighbour lookups for the
    headroom gate run in value order, independent of flight order."""
    return sorted(range(len(cands)), key=lambda i: cands[i])


def _aligned_everywhere(row):
    """True when a fully-flown candidate row aligned at EVERY grid cell (a no-align
    cell costs >= NOALIGN_BASE; an unflown / failed-to-set row is inf)."""
    return max(row) < NOALIGN_BASE


def _beats_with_margin(challenger_worst, incumbent_worst):
    """Hysteresis: the challenger displaces the incumbent only when its worst-case
    cost is better by SWITCH_MARGIN_REL of the incumbent's, plus an absolute
    floor.  Guards the argmin against single-sample evaluation noise.  The
    absolute floor is _MARGIN_ABS_EFF, which the noise-floor pass raises to the
    MEASURED trial-to-trial spread when that is larger than the hand-set value.
    When BOTH worst-cases are no-align costs the comparison switches to a DEGREE
    margin: the relative margin would be ~100 cost units there, reachable only
    by actually aligning, which silently discarded the sub-align gradient.  The
    degree margin is scaled into cost units by the heaviest per-degree weight
    in the no-align cost, so the hysteresis stays as strict as it reads."""
    if challenger_worst >= NOALIGN_BASE and incumbent_worst >= NOALIGN_BASE:
        margin = NOALIGN_TIE_MARGIN_DEG * max(NOALIGN_W_MINERR, COST_W_SETTLED)
        return challenger_worst < incumbent_worst - margin
    return challenger_worst < incumbent_worst * (1.0 - SWITCH_MARGIN_REL) - _MARGIN_ABS_EFF


def _candidates(default, seed, factors=None):
    """The FIXED bracket of candidate values for a parameter.  Non-zero default:
    multiplicative `factors` (default BRACKET_FACTORS; includes 1.0 = unchanged).
    Zero default (e.g. kd): an additive bracket scaled from the seed, INCLUDING 0
    (unchanged).  Always anchored to the DEFAULT (not the current value), so the
    set is identical every pass and the search cannot ratchet.

    `factors` lets one entry cast a wider net than the rest - see
    ENTRY_BRACKETS - without loosening the bracket for every parameter, which
    would multiply flight time and weaken the headroom probe (that probe reads
    the next candidate UP, so its meaning depends on the step ratio)."""
    if abs(default) < 1e-9:
        s = float(seed) if seed else 0.05
        return [0.0, 0.5 * s, s, 2.0 * s, 4.0 * s]
    return [default * f for f in (factors or BRACKET_FACTORS)]


def _tune_one(ctrl, store, entry, abort, progress, grid,
              resume_state=None, save_cb=None, anchors=None, wide=False):
    """Cost-based bracket search for one parameter, evaluated across the FULL
    maneuver grid.  Tries a FIXED bracket of candidates around the parameter's
    DEFAULT (always including the default); each candidate is flown at EVERY grid
    cell and scored by its WORST-CASE cost over the cells, so the kept value has to
    hold up at the most aggressive maneuver - not just on average.  The raw argmin
    is then run through three decision gates (see the module constants):
      1. HYSTERESIS vs the pass-start baseline,
      2. a CONFIRMATION RE-FLY of challenger + baseline when a switch is proposed,
      3. the HEADROOM gate for entries flagged "margin": True.
    Returns (status, baseline, chosen, changed, per_step_best, notes):
      changed       - chosen value differs from where this pass started (convergence)
      per_step_best - [((pan,tilt), best_candidate, cost), ...] one per grid cell,
                      the lowest-cost candidate AT that cell (for the report).
      notes         - {'hysteresis'/'confirm'/'headroom': str} decision-gate audit
                      strings for the report (empty when nothing fired).

    CRASH-SAFE / RESUMABLE.  The (candidate x cell) cost matrix is the ONLY state the
    final choice depends on, and a finite cell is one that was actually flown (an
    aborted trial returns immediately and is never stored).  So after EVERY trial the
    partial matrix is handed to save_cb (which folds it into the checkpoint), and on
    relaunch resume_state re-seeds it: already-flown cells are SKIPPED and only the
    missing ones re-flown.  The chosen value is recomputed from the full matrix at the
    end, so it is identical whether the parameter was flown in one go or resumed after
    a crash / Ctrl+C.  (A confirmation re-fly folds its extra sample into the matrix
    as a running average and checkpoints it; a resume that re-triggers confirmation
    simply folds in another sample - slight extra weight on later draws, harmless.)"""
    attr, axis = entry["attr"], entry["axis"]
    seed = entry.get("seed", None)
    notes = {}
    try:
        baseline = float(store.get(attr))          # value at the start of THIS pass
        # The bracket's ANCHOR.  Normally the constants.py default; with the
        # profile's "Use Constants" switch OFF it is the operator's GUI value,
        # snapshotted once at run start and carried in the checkpoint.  Either
        # way it is FIXED for the whole run - re-reading the live value each pass
        # would re-centre the bracket on the last winner and let the search
        # ratchet away instead of converging.
        default = float(anchors[attr]) if (anchors and attr in anchors) \
            else float(store.default_for(attr))
    except Exception:
        log.warning("[CATUNE] %s is not a tunable parameter; skipping.", attr)
        return ("skip", None, None, False, [], {})

    cands = _candidates(default, seed, _bracket_for(entry, wide))
    # On resume the store may hold the last in-flight CANDIDATE for this parameter (the
    # per-trial checkpoint records live param values), not its pass-start value - so the
    # saved baseline is authoritative for the convergence ("changed") test and the abort
    # restore.  It is constant across the parameter's trials, so every snapshot carries it.
    if (resume_state is not None and resume_state.get("attr") == attr
            and resume_state.get("baseline") is not None):
        baseline = float(resume_state["baseline"])
    cell_cost = [[float("inf")] * len(grid) for _ in cands]   # cell_cost[ci][gi]
    # Re-seed already-flown cells from a checkpoint, but ONLY when it is unambiguously
    # the SAME work: same parameter AND an identical candidate bracket (a changed
    # default/seed would shift the brackets and make the saved costs incomparable).
    if (resume_state is not None and resume_state.get("attr") == attr
            and resume_state.get("cands") == cands):
        for ci, gi, c in resume_state.get("filled", []):
            if 0 <= ci < len(cands) and 0 <= gi < len(grid):
                cell_cost[ci][gi] = float(c)
        done = sum(1 for row in cell_cost for v in row if v != float("inf"))
        if done:
            log.info("[CATUNE] resuming %s: %d/%d cell-trials already flown; re-flying "
                     "only the remaining %d.", attr, done, len(cands) * len(grid),
                     len(cands) * len(grid) - done)

    def _snapshot():
        """Sparse form of cell_cost for the checkpoint (skip the unflown inf cells)."""
        return {"attr": attr, "cands": cands, "baseline": baseline,
                "filled": [[ci, gi, cell_cost[ci][gi]]
                           for ci in range(len(cands)) for gi in range(len(grid))
                           if cell_cost[ci][gi] != float("inf")]}

    log.info("[CATUNE] --- %s  (default=%.4g, now=%.4g)  candidates=[%s]  axis=%s  "
             "across %d grid cell(s) ---", attr, default, baseline,
             ", ".join("%.4g" % c for c in cands), axis, len(grid))
    bci = next((i for i, c in enumerate(cands)
                if abs(c - baseline) <= 1e-9 * max(1.0, abs(baseline))), None)
    order = list(range(len(cands)))
    if bci is not None:                    # baseline first - it seeds the champion
        order.remove(bci)
        order.insert(0, bci)
    champion = None                        # best COMPLETED row's worst-case so far
    for ci in order:
        cand = cands[ci]
        need = [gi for gi in range(len(grid)) if cell_cost[ci][gi] == float("inf")]
        if not need:
            w = max(cell_cost[ci])
            champion = w if champion is None else min(champion, w)
            continue                                # candidate already fully flown (resumed)
        if abort.is_set() or ctrl.mode != 'autotune':
            store.set(attr, baseline)               # restore the in-progress parameter
            return ("abort", baseline, baseline, False, [], {})
        try:
            store.set(attr, cand)
        except Exception:
            log.warning("[CATUNE] could not set %s=%.4g; skipping candidate.", attr, cand)
            continue
        pruned = False
        for gi in sorted(need, reverse=True):       # hardest cell FIRST
            pan, tilt = grid[gi]
            m = _fly_with_protection(ctrl, pan, tilt, axis, abort, progress, attr)
            if m is None:
                store.set(attr, baseline)
                return ("abort", baseline, baseline, False, [], {})
            c = _cost(m)
            cell_cost[ci][gi] = c
            if save_cb is not None:
                save_cb(_snapshot())                # CHECKPOINT after every trial
            if SH_PRUNE and ci != bci and champion is not None:
                run_max = max(v for v in cell_cost[ci] if v != float("inf"))
                if run_max > champion:
                    flown = sum(1 for v in cell_cost[ci] if v != float("inf"))
                    prev = notes.get("pruned", "")
                    notes["pruned"] = (prev + ("; " if prev else "") +
                                       "%.4g after %d cell(s) (%.1f > champion %.1f)"
                                       % (cand, flown, run_max, champion))
                    log.info("[CATUNE]   %s=%.4g PRUNED after %d cell(s): running "
                             "worst %.1f already exceeds champion %.1f - cannot win.",
                             attr, cand, flown, run_max, champion)
                    pruned = True
                    break
            if m["aligned"]:
                detail = "aligned@%.1fs jit=%.2f ovs=%2.0f%% ring=%d" % (
                    m["t_align"], m["jitter"], 100.0 * m["overshoot"], m["ring"])
            else:
                detail = "NO-ALIGN min_err=%.1f" % m["min_err"]
            log.info("[CATUNE] [%s] %s=%.4g @(%+.0f,%+.0f) -> cost=%.1f  (%s)",
                     progress.tag(), attr, cand, pan, tilt, c, detail)
        if not pruned:
            w = max(cell_cost[ci])
            champion = w if champion is None else min(champion, w)
            log.info("[CATUNE]   %s=%.4g -> worst-cell cost=%.1f", attr, cand, w)

    # DECISION 1/3 - raw argmin of the worst-case over the (now full) matrix - a
    # pure function of cell_cost, so the result is identical for a one-shot or a
    # resumed parameter.  A candidate that never set keeps an all-inf row and is
    # naturally excluded.
    best_ci, best_worst = None, float("inf")
    for ci in range(len(cands)):
        worst = max(cell_cost[ci])
        if worst < best_worst:
            best_worst, best_ci = worst, ci
    if best_ci is None:                             # every candidate failed to set
        store.set(attr, baseline)
        return ("ok", baseline, baseline, False, [], {"note": "no candidate could be set"})

    # DECISION 2/3 - HYSTERESIS vs the pass-start baseline.  The baseline is
    # always in the fixed candidate set (the default on pass 1, the last chosen
    # candidate afterwards) unless hand-edited mid-sweep, in which case
    # hysteresis is skipped and the raw argmin stands.
    base_ci = next((i for i, c in enumerate(cands)
                    if abs(c - baseline) <= 1e-9 * max(1.0, abs(baseline))), None)
    chosen_ci = best_ci
    nearmiss_adopted = False
    if base_ci is not None and chosen_ci != base_ci:
        base_worst = max(cell_cost[base_ci])
        if not _beats_with_margin(best_worst, base_worst):
            # DECISION 2.4 - NEAR-MISS CONFIRMATION (see the constants note).
            # Strictly-better-but-short is a photo-finish: buy samples, decide
            # on medians, adopt only past the small margin.
            if (NEARMISS_REFLY > 0 and best_worst < base_worst
                    and best_worst < NOALIGN_BASE and base_worst < NOALIGN_BASE):
                bail = False
                for ci in (chosen_ci, base_ci):
                    gi = max(range(len(grid)), key=lambda g: cell_cost[ci][g])
                    pan, tilt = grid[gi]
                    try:
                        store.set(attr, cands[ci])
                    except Exception:
                        bail = True
                        break
                    samples = [cell_cost[ci][gi]]
                    for _rep in range(NEARMISS_REFLY):
                        ctrl.autotune_status = ("near-miss re-fly  %s=%.4g @(%+.0f,%+.0f)"
                                                % (attr, cands[ci], pan, tilt))
                        m = _fly_with_protection(ctrl, pan, tilt, axis,
                                                 abort, None, attr)
                        if m is None:
                            store.set(attr, baseline)
                            return ("abort", baseline, baseline, False, [], {})
                        samples.append(_cost(m))
                        log.info("[CATUNE] near-miss: %s=%.4g @(%+.0f,%+.0f) re-fly "
                                 "cost=%.1f", attr, cands[ci], pan, tilt, samples[-1])
                    samples.sort()
                    cell_cost[ci][gi] = samples[len(samples) // 2]     # median
                    if save_cb is not None:
                        save_cb(_snapshot())
                if not bail:
                    ch_med = max(cell_cost[chosen_ci])
                    in_med = max(cell_cost[base_ci])
                    if ch_med < in_med * (1.0 - NEARMISS_MARGIN_REL) - NEARMISS_MARGIN_ABS:
                        nearmiss_adopted = True
                        notes["nearmiss"] = ("photo-finish ADOPTED %.4g on medians, "
                                             "%d samples/side (%.1f vs %.1f)"
                                             % (cands[chosen_ci], NEARMISS_REFLY + 1,
                                                ch_med, in_med))
                        log.info("[CATUNE] %s: near-miss confirmation ADOPTED the "
                                 "challenger (%s).", attr, notes["nearmiss"])
                    else:
                        notes["nearmiss"] = ("photo-finish kept %.4g on medians, "
                                             "%d samples/side (%.1f vs %.1f)"
                                             % (baseline, NEARMISS_REFLY + 1,
                                                ch_med, in_med))
            if not nearmiss_adopted:
                log.info("[CATUNE] %s: challenger %.4g (worst %.1f) does not beat baseline "
                         "%.4g (worst %.1f) by the hysteresis margin - keeping the baseline.",
                         attr, cands[best_ci], best_worst, baseline, base_worst)
                notes["hysteresis"] = ("kept %.4g: challenger %.4g short of margin "
                                       "(%.1f vs %.1f)" % (baseline, cands[best_ci],
                                                           best_worst, base_worst))
                chosen_ci = base_ci

    # DECISION 2.5 - CONFIRMATION RE-FLY (winner's-curse guard).  A switch is
    # being proposed off SINGLE noisy samples whose per-candidate score is a MAX
    # over cells (biased upward, high variance).  Re-fly the two deciders - the
    # challenger at ITS worst cell, the baseline at ITS worst cell - once each,
    # average the samples into the matrix (checkpointed), and re-decide between
    # the TWO of them only: promoting a third, un-re-flown candidate would
    # re-open the single-sample curse.
    if (CONFIRM_REFLY > 0 and base_ci is not None and chosen_ci != base_ci
            and not nearmiss_adopted):
        for ci in (chosen_ci, base_ci):
            gi = max(range(len(grid)), key=lambda g: cell_cost[ci][g])
            pan, tilt = grid[gi]
            try:
                store.set(attr, cands[ci])
            except Exception:
                continue
            for _rep in range(CONFIRM_REFLY):
                ctrl.autotune_status = "confirm re-fly  %s=%.4g @(%+.0f,%+.0f)" % (
                    attr, cands[ci], pan, tilt)
                m = _fly_with_protection(ctrl, pan, tilt, axis,
                                         abort, None, attr)
                if m is None:
                    store.set(attr, baseline)
                    return ("abort", baseline, baseline, False, [], {})
                new_c = _cost(m)
                cell_cost[ci][gi] = 0.5 * (cell_cost[ci][gi] + new_c)
                if save_cb is not None:
                    save_cb(_snapshot())
                log.info("[CATUNE] confirm: %s=%.4g @(%+.0f,%+.0f) re-fly cost=%.1f "
                         "-> cell avg %.1f", attr, cands[ci], pan, tilt, new_c,
                         cell_cost[ci][gi])
        chal_worst = max(cell_cost[chosen_ci])
        base_worst = max(cell_cost[base_ci])
        if _beats_with_margin(chal_worst, base_worst):
            notes["confirm"] = ("switch CONFIRMED on averaged worst-cases "
                                "(%.1f vs %.1f)" % (chal_worst, base_worst))
        else:
            notes["confirm"] = ("switch RETRACTED on averaged worst-cases "
                                "(%.1f vs %.1f) - keeping %.4g"
                                % (chal_worst, base_worst, baseline))
            log.info("[CATUNE] %s: confirmation re-fly retracted the switch (%s).",
                     attr, notes["confirm"])
            chosen_ci = base_ci

    # DECISION 3/3 - HEADROOM gate for "margin": True entries (crude empirical
    # gain margin; see the module constants).  Applied to whatever value is being
    # KEPT - a retained baseline can lack headroom too.
    final_ci = chosen_ci
    if entry.get("margin") and _aligned_everywhere(cell_cost[final_ci]):
        rank = _value_rank(cands)
        pos = rank.index(final_ci)
        if pos + 1 < len(rank):
            # Free from the matrix: the next candidate UP is the ~x1.4 probe.
            up_ci = rank[pos + 1]
            # The neighbour may have been PRUNED (partial row): complete it
            # before reading, so this gate sees the same evidence an exhaustive
            # run would have.
            for gi in sorted(range(len(grid)), reverse=True):
                if cell_cost[up_ci][gi] != float("inf"):
                    continue
                pan, tilt = grid[gi]
                ctrl.autotune_status = ("headroom fill  %s=%.4g @(%+.0f,%+.0f)"
                                        % (attr, cands[up_ci], pan, tilt))
                try:
                    store.set(attr, cands[up_ci])
                    m = _fly_with_protection(ctrl, pan, tilt, axis,
                                             abort, None, attr)
                finally:
                    store.set(attr, cands[final_ci])
                if m is None:
                    store.set(attr, baseline)
                    return ("abort", baseline, baseline, False, [], {})
                cell_cost[up_ci][gi] = _cost(m)
                if save_cb is not None:
                    save_cb(_snapshot())
            passed = _aligned_everywhere(cell_cost[up_ci])
            how = "matrix: %.4g %s" % (cands[up_ci], "aligned everywhere" if passed
                                       else "had a no-align cell")
        else:
            # Winner at the TOP of the bracket: one extra probe flight at
            # HEADROOM_FACTOR x its value, at the hardest cell.
            hv = HEADROOM_FACTOR * cands[final_ci]
            pan, tilt = grid[-1]
            ctrl.autotune_status = "headroom probe  %s=%.4g @(%+.0f,%+.0f)" % (
                attr, hv, pan, tilt)
            m = None
            try:
                store.set(attr, hv)
                m = _fly_with_protection(ctrl, pan, tilt, axis,
                                         abort, None, attr)
            finally:
                store.set(attr, cands[final_ci])    # ALWAYS undo the probe value
            if m is None:
                store.set(attr, baseline)
                return ("abort", baseline, baseline, False, [], {})
            passed = bool(m.get("aligned"))
            how = "probe flight: %.4g %s" % (hv, "aligned" if passed else "did NOT align")
        if passed:
            notes["headroom"] = "ok (%s)" % how
        else:
            down_ci = rank[pos - 1] if pos > 0 else None
            if down_ci is not None and _aligned_everywhere(cell_cost[down_ci]):
                # The demoted value's OWN headroom is proved by construction: its
                # upper neighbour is the kept value, whose row aligned everywhere.
                log.warning("[CATUNE] %s: NO HEADROOM at %.4g (%s) - demoting one "
                            "bracket step to %.4g.", attr, cands[final_ci], how,
                            cands[down_ci])
                notes["headroom"] = ("FAILED at %.4g (%s) -> demoted to %.4g"
                                     % (cands[final_ci], how, cands[down_ci]))
                final_ci = down_ci
            else:
                log.warning("[CATUNE] %s: NO HEADROOM at %.4g (%s) and no viable "
                            "lower candidate - KEEPING it; hand-check this gain "
                            "before flying the real aircraft.", attr,
                            cands[final_ci], how)
                notes["headroom"] = ("FAILED at %.4g (%s); no viable lower "
                                     "candidate - kept, HAND-CHECK"
                                     % (cands[final_ci], how))
    elif entry.get("margin"):
        notes["headroom"] = "skipped (kept value itself had no-align cells)"

    best_val = cands[final_ci]
    store.set(attr, best_val)
    changed = abs(best_val - baseline) > 1e-9
    per_step_best = []                              # lowest-cost candidate per grid cell
    for gi, cell in enumerate(grid):
        bci = min(range(len(cands)), key=lambda ci: cell_cost[ci][gi])
        per_step_best.append((cell, cands[bci], cell_cost[bci][gi]))
    log.info("[CATUNE] === %s: %.4g -> %.4g  (worst-case cost=%.1f over %d cells)%s ===",
             attr, baseline, best_val, max(cell_cost[final_ci]), len(grid),
             "".join("  [%s: %s]" % kv for kv in notes.items()))
    return ("ok", baseline, best_val, changed, per_step_best, notes)


def _validate_combined(ctrl, abort, grid):
    """After tuning, confirm the FULL gain set holds together at the MOST AGGRESSIVE
    maneuver cell in the grid (its last cell = max pan, max tilt).  Single-gain trials can
    each pass while the combination is marginal - this catches that.  Altitude-
    protected and re-flown like any trial.  Returns the metrics dict, or None."""
    pan, tilt = grid[-1] if grid else (VALIDATE_PAN, VALIDATE_TILT)
    log.info("[CATUNE] combined validation at the hardest cell (%+.0f,%+.0f).", pan, tilt)
    return _fly_with_protection(ctrl, pan, tilt, "pan", abort, None, "validate")


def _grid_signature(ctrl, profile=SWEEP_PROFILE):
    """The maneuver-grid config as plain JSON-safe values, used to tell whether
    a saved checkpoint was produced under the same grid (resuming under a
    different grid would mix incomparable results).  Includes the coarse/full
    grid MODE: the two modes have different cell counts and indices, so a mode
    switch must invalidate an old checkpoint rather than resume across it."""
    sig = {k: round(float(getattr(ctrl, "autotune_" + k)), 4)
           for k in ("pan_min", "pan_max", "pan_step", "tilt_min", "tilt_max", "tilt_step")}
    sig["full_grid"] = bool(getattr(ctrl, "autotune_full_grid", False))
    sig["profile"] = profile.name          # 't' and 'y' state are not interchangeable
    # 'y' does not read the pan/tilt grid at all, so without its cell here a
    # checkpoint from one bearing would resume onto another with an identical
    # signature - every saved result attributed to the wrong cell.
    if profile.single_cell:
        sig["cell"] = [round(float(ctrl.autotune_cell_lon_deg), 4),
                       round(float(ctrl.autotune_cell_lat_deg), 4)]
    # The bracket decides WHICH candidate values exist and how many; results are
    # stored per candidate, so narrow and wide matrices do not line up.
    sig["bracket_wide"] = int(bool(int(round(float(
        getattr(ctrl, "autotune_bracket_wide", 0))))))
    g = getattr(ctrl, "_gains", None)
    # The resolved pan-axis grid cap: cells flown under a different cap are not
    # comparable and their indices do not line up.
    sig["tilt_cap"] = float(GRID_TILT_CAP_DEG)
    # Airspeed command changes the OPERATING POINT of every trial: holding a
    # speed removes the dive/climb speed split that otherwise makes the two
    # branches different plants.  Gains swept at held airspeed are not
    # comparable with gains swept at fixed throttle.
    sig["airspeed_cmd"] = [int(getattr(g, "airspeed_cmd_enable", 0)),
                           round(float(getattr(g, "airspeed_cmd_mps", 0.0)), 2)]
    return sig


def _save_checkpoint(store, settled, pass_idx, tuned_this_pass, moved, results, total,
                     ctrl, in_pass, in_progress=None, profile=SWEEP_PROFILE,
                     anchors=None):
    """Write the full sweep progress to the incomplete_ checkpoint so a later launch
    can RESUME from here.  Captures every swept parameter's CURRENT value (the gains
    tuned so far), the frozen set, the in-progress pass position, the per-pass trial
    counter (in_pass, so the HUD "test x/y" continues instead of restarting), and the
    accumulated per-parameter results for the eventual report.  in_progress (when set)
    is the partial (candidate x cell) cost matrix of the parameter being tuned RIGHT
    NOW, stored sparsely as {"attr","cands","filled":[[ci,gi,cost],...]}, so a crash
    mid-parameter re-flies only the cells that had not been flown yet."""
    param_values = {}
    for entry in SWEEP_PLAN:
        a = entry["attr"]
        try:
            param_values[a] = float(store.get(a))
        except Exception:
            pass
    data = {
        "version": CHECKPOINT_VERSION,
        "saved_at": datetime.now().isoformat(timespec="seconds"),
        "fingerprint": PACKAGE_SHA256,
        "sweep_plan_attrs": [e["attr"] for e in SWEEP_PLAN],
        "grid": _grid_signature(ctrl, profile),
        # The bracket anchors this run is searching around.  Saved because with
        # "Use Constants" OFF they are the operator's start-of-run GUI values and
        # exist nowhere else - a resume that fell back to constants.py would
        # search a different bracket from the one the completed trials used, and
        # silently mix two incomparable searches into one result.
        "anchors": dict(anchors) if anchors else None,
        "pass_idx": int(pass_idx),
        "tuned_this_pass": sorted(tuned_this_pass),
        "moved": int(moved),
        "settled": sorted(settled),
        "total_trials": int(total),
        "in_pass": int(in_pass),
        "param_values": param_values,
        "results": results,
        "in_progress": in_progress,
    }
    # Stamped with the run that STARTED it - a resumed run keeps writing to the
    # same file, so the name still answers "when did this procedure begin".
    path = _checkpoint_path(ctrl, profile)
    try:
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        log.info("[CATUNE] progress checkpoint saved -> %s", path)
    except Exception:
        log.exception("[CATUNE] could not write checkpoint file")


def _load_checkpoint(ctrl, profile=SWEEP_PROFILE):
    """If an incomplete_ checkpoint exists AND its config still matches the current
    sweep, parse and return the saved state; otherwise return None (start fresh).  A
    mismatch (different parameter set or grid) is reported and the stale checkpoint
    ignored rather than producing inconsistent results."""
    matches = sorted(glob.glob(os.path.join(os.getcwd(), profile.checkpoint_glob)))
    if not matches:
        return None
    path = matches[-1]
    try:
        with open(path) as f:
            data = json.load(f)
    except Exception:
        log.exception("[CATUNE] checkpoint %s unreadable; starting fresh", path)
        return None
    if data.get("version") != CHECKPOINT_VERSION:
        log.warning("[CATUNE] checkpoint version mismatch; starting fresh.")
        return None
    if data.get("sweep_plan_attrs") != [e["attr"] for e in SWEEP_PLAN]:
        log.warning("[CATUNE] checkpoint parameter set differs from the current plan; "
                    "starting fresh (delete %s to silence).", path)
        return None
    if data.get("grid") != _grid_signature(ctrl, profile):
        log.warning("[CATUNE] checkpoint grid differs from current; starting fresh "
                    "(delete %s to silence).", path)
        return None
    # A checkpoint recording a non-step excitation cannot be resumed: its
    # completed trials measured a different maneuver, and mixing them into one
    # cost would pick a gain from the mixture.  Absent key = step.
    if int(data.get("maneuver", 0)) != 0:
        log.warning("[CATUNE] checkpoint %s was flown with the removed DOUBLET "
                    "excitation; starting fresh rather than mixing excitations.", path)
        return None
    # ACCEPTED.  This run continues that procedure, so it adopts the stamp the
    # checkpoint was named with and every file it writes from here carries it.
    ctrl.adopt_run_stamp(profile.key, file_export.stamp_from_name(path))
    return data


def _clear_checkpoint(profile=SWEEP_PROFILE):
    """Remove any incomplete_ checkpoint(s) - called when a sweep completes so the
    next launch starts fresh."""
    for path in glob.glob(os.path.join(os.getcwd(), profile.checkpoint_glob)):
        try:
            os.remove(path)
            log.info("[CATUNE] cleared checkpoint %s", path)
        except Exception:
            log.exception("[CATUNE] could not remove checkpoint %s", path)


def _winning_set_lines(results):
    """THE WINNING SET: one paste-ready constants.py line per tuned parameter -
    the final chosen value, translated from tuner attr name to its constants.py
    name via DEFAULT_CONST.  Changed values are marked so the eye lands on them.
    Returns (lines, n_changed, n_ok)."""
    lines, n_changed, n_ok = [], 0, 0
    for attr, r in results.items():
        if r["status"] != "ok":
            continue
        n_ok += 1
        changed = abs(float(r["final"]) - float(r["orig"])) > 1e-9
        n_changed += 1 if changed else 0
        tag = ("    # <- CHANGED (was %s)" % repr(float(r["orig"]))) if changed \
              else "    # unchanged"
        const = DEFAULT_CONST.get(attr)
        if const:
            lines.append("%s = %s%s" % (const, repr(float(r["final"])), tag))
        else:   # defensive: attr with no constants.py home (tuner-only)
            lines.append("# %s = %s%s  (tuner-only attr; set it in the manual tuner)"
                         % (attr, repr(float(r["final"])), tag))
    return lines, n_changed, n_ok


def _report_and_save(results, validation=None, total_trials=None, noise_floor=None,
                     ctrl=None, grid=None, profile=SWEEP_PROFILE, anchors=None):
    """Log the final table + validation and write controller_autotune_<ts>.txt.
    results is an insertion-ordered dict: attr -> {'orig', 'final', 'status'}.
    noise_floor (if measured) is reported FIRST, because it sets the resolution
    limit for every other number in the file."""
    log.info("=" * 60)
    log.info("[CATUNE] Controller Autotune sweep complete.")
    if total_trials is not None:
        log.info("[CATUNE] %d tuning trials flown%s.", total_trials,
                 " + 1 validation" if validation is not None else "")
    log.info("[CATUNE] %-28s %12s -> %-12s", "gain", "baseline", "tuned (worst-case)")
    for attr, r in results.items():
        if r["status"] == "ok":
            log.info("[CATUNE]   %-26s %12.5g -> %-12.5g", attr, r["orig"], r["final"])
            for (pan, tilt), val, cost in r.get("per_step", []):
                log.info("[CATUNE]        @(%+.0f,%+.0f): best %.5g  (cost %.1f)",
                         pan, tilt, val, cost)
            for k in ("hysteresis", "nearmiss", "confirm", "headroom", "note"):
                v = (r.get("notes") or {}).get(k)
                if v:
                    log.info("[CATUNE]        %s: %s", k, v)
        else:
            log.info("[CATUNE]   %-26s %12s    (%s)", attr, "-", r["status"])
    if validation is not None:
        if validation.get("aligned"):
            log.info("[CATUNE] combined validation: PASS - full set aligned @%.1fs, "
                     "jitter %.2f deg.", validation["t_align"], validation["jitter"])
        else:
            log.warning("[CATUNE] combined validation: MARGINAL - the full gain set did "
                        "NOT hold alignment together.  Treat the values with caution and "
                        "hand-refine the coupled gains (P/I/D per axis).")
    ws_lines, ws_changed, ws_ok = _winning_set_lines(results)
    log.info("[CATUNE] WINNING SET - paste into constants.py (%d of %d changed):",
             ws_changed, ws_ok)
    for ln in ws_lines:
        log.info("[CATUNE]   %s", ln)
    # The RUN's stamp: a sweep runs for a long time, and dating the report by
    # when it finished makes it disagree with the suggested-constants file and
    # the checkpoint from the same run.
    run_ts = ctrl.run_stamp_for(profile.key)
    path = os.path.join(os.getcwd(),
                        "%s_%s.txt" % (profile.report_prefix, run_ts))
    try:
        with open(path, "w") as f:
            f.write("# Controller Autotune results %s\n" % datetime.now().isoformat(timespec="seconds"))
            f.write("# code_sha256: %s\n" % PACKAGE_SHA256)
            if total_trials is not None:
                f.write("# %d tuning trials flown%s\n" % (total_trials,
                        " + 1 validation" if validation is not None else ""))
            f.write("# Applied live to the running controller; NOT auto-persisted.\n")
            f.write("# The WINNING SET block below is paste-ready for constants.py.\n")
            f.write("# 'tuned' = best WORST-CASE candidate over the maneuver grid; the\n")
            f.write("# per-(pan,tilt) lines below each gain are the best value AT that cell.\n")
            if validation is not None:
                f.write("# combined validation: %s\n" % ("PASS" if validation.get("aligned")
                        else "MARGINAL - full set did not hold alignment"))
            # RUN CONFIGURATION: the as-flown conditions.  code_sha256 above pins
            # the code and the DEFAULT constants, but NOT the runtime tuner/slider
            # state below - two runs with the same hash can differ in every line
            # here (grid, throttle, bank clip).  Recorded so a
            # report can be reproduced, and so a NULL result (e.g. no-align cells)
            # can be read against the conditions that produced it.
            if ctrl is not None and grid is not None:
                f.write("#\n# ==================== RUN CONFIGURATION ====================\n")
                try:
                    # WHERE THIS RUN STARTED.  Two runs with identical gains in
                    # the table can have searched completely different brackets,
                    # and nothing else in the file records which.  With the
                    # switch OFF the anchors are the operator's start-of-run Gain
                    # Tuner values, which exist nowhere else once the run
                    # overwrites them - so they are written out in full.
                    f.write("# mechanism: %s (the '%s' key)\n"
                            % (profile.name, profile.key))
                    _uc = bool(int(round(float(
                        getattr(ctrl, profile.use_constants_attr, 1)))))
                    f.write("# Use Constants: %s  (%s)\n"
                            % ("ON" if _uc else "OFF", profile.use_constants_attr))
                    if _uc:
                        f.write("#   swept gains were reset to their constants.py values\n"
                                "#   before the run, and every bracket is anchored to those\n"
                                "#   values - so each candidate is constants x one of\n"
                                "#   %s.\n" % (BRACKET_FACTORS,))
                    else:
                        f.write("#   swept gains were NOT reset.  The run started from the\n"
                                "#   Gain Tuner's values and anchored every bracket to them,\n"
                                "#   so the constants.py values were NOT necessarily among\n"
                                "#   the candidates.  Anchors searched around:\n")
                        if anchors:
                            for _a in sorted(anchors):
                                f.write("#     %-30s %.6g\n" % (_a, anchors[_a]))
                        else:
                            f.write("#     (not recorded)\n")
                    if profile.single_cell:
                        # The pan/tilt grid cells are not read by 'y'; printing
                        # their ranges here would describe a search it did not do.
                        f.write("# cell: longitude %+.1f, latitude %+.1f  "
                                "(one bearing; no worst-case vote across cells,\n"
                                "#   so these gains are the best AT THIS BEARING only)\n"
                                % (grid[0][0], grid[0][1]))
                    else:
                        _full = bool(getattr(ctrl, "autotune_full_grid", False))
                        f.write("# maneuver grid: pan [%+.0f .. %+.0f] step %.0f, "
                                "tilt [%+.0f .. %+.0f] step %.0f\n"
                                % (ctrl.autotune_pan_min, ctrl.autotune_pan_max,
                                   ctrl.autotune_pan_step, ctrl.autotune_tilt_min,
                                   ctrl.autotune_tilt_max, ctrl.autotune_tilt_step))
                        f.write("#   mode: %s  (%d cell%s flown)\n"
                                % ("FULL cross-product" if _full else
                                   "coarse 3x3 {min,mid,max}",
                                   len(grid), "" if len(grid) == 1 else "s"))

                    _w = bool(int(round(float(
                        getattr(ctrl, 'autotune_bracket_wide', 0)))))
                    _bf = BRACKET_FACTORS_WIDE if _w else BRACKET_FACTORS
                    f.write("# bracket: %s - %d candidates, x%.3g .. x%.3g of the "
                            "anchor\n" % ("WIDE" if _w else "narrow", len(_bf),
                                          _bf[0], _bf[-1]))
                    f.write("# excitation: anchored-balloon STEP\n")
                    f.write("# throttle: %.0f%%\n" % float(ctrl.throttle_pct))
                    _g = ctrl._gains
                    f.write("# bank limit (bank_lim_deg): %.1f deg  "
                            "<- the single roll clip in force\n"
                            % float(getattr(_g, "bank_lim_deg", float("nan"))))
                except Exception:
                    f.write("# (run configuration partially unavailable)\n")
            # The noise floor goes FIRST: it is the resolution limit for every
            # comparison in this file.  A "winner" whose margin is inside the
            # measured scatter is not a result, and the reader should know that
            # before reading the table.
            if noise_floor:
                nf = noise_floor
                f.write("#\n# ==================== NOISE FLOOR ====================\n")
                f.write("# Baseline flown %d x over the grid BEFORE tuning.\n" % nf["reps"])
                f.write("# worst-case cost per rep: %s\n"
                        % ", ".join("%.2f" % w for w in nf["worsts"]))
                f.write("# mean %.2f   1 sigma %.2f   range %.2f\n"
                        % (nf["mean"], nf["sigma"], nf["range"]))
                if nf.get("sample_hz"):
                    f.write("# error sampling achieved %.1f Hz (nominal %.0f)%s\n"
                            % (nf["sample_hz"], 1.0 / SAMPLE_DT,
                               "   <- well below nominal; metrics built from fewer points"
                               if nf["sample_hz"] < 0.7 / SAMPLE_DT else ""))
                f.write("# absolute switch margin: %.2f -> %.2f%s\n"
                        % (nf["margin_abs_before"], nf["margin_abs_after"],
                           "  (RAISED to the measured noise)"
                           if nf["margin_abs_after"] > nf["margin_abs_before"] + 1e-9
                           else "  (hand-set value kept; measured noise was smaller)"))
                if nf["mean"] > 0 and nf["sigma"] / nf["mean"] > 0.25:
                    f.write("# WARNING: scatter is %.0f%% of the mean. Differences below\n"
                            "#   ~%.1f cost units in this run are not distinguishable from\n"
                            "#   run-to-run noise, whatever the table says.\n"
                            % (100.0 * nf["sigma"] / nf["mean"], nf["margin_abs_after"]))
            else:
                f.write("#\n# NOISE FLOOR: not measured (resumed run, or reps < 2).\n"
                        "#   Switch margins are the hand-set defaults, so the\n"
                        "#   resolution of every comparison below is unknown.\n")
            # Cells no candidate could ever win.  A cell like this makes the
            # min-max cost identical for EVERY candidate, so it contributes
            # nothing to the comparison while still costing a trial each time -
            # and it silently caps what the sweep can conclude.  Worth seeing.
            if _CELL_ALIGNS:
                never = sorted((k for k, v in _CELL_ALIGNS.items() if v['aligned'] == 0),
                               key=lambda k: (abs(k[1]), abs(k[0])))
                f.write("#\n# ============ CELLS THAT NEVER CONVERGED ============\n")
                if not never:
                    f.write("# none - every (pan, tilt) cell aligned at least once.\n")
                else:
                    f.write("#   pan    tilt   attempts   aligned\n")
                    for pan, tilt in never:
                        v = _CELL_ALIGNS[(pan, tilt)]
                        f.write("# %+6.0f %+7.0f %10d %8d\n"
                                % (pan, tilt, v['attempts'], v['aligned']))
                    f.write("#   These cells were flown %d time(s) in total and NEVER\n"
                            "#   aligned, under any gain the sweep tried.  Because the cost\n"
                            "#   is a max over cells, each one pins every candidate at the\n"
                            "#   same no-align penalty - so it cannot separate good gains\n"
                            "#   from bad, and every trial spent there is wasted.\n"
                            "#   Either the law lacks the authority to hold that bearing\n"
                            "#   at all, or the cell is outside the flyable envelope.\n"
                            % sum(_CELL_ALIGNS[c]['attempts'] for c in never))
                    won = [k for k, v in _CELL_ALIGNS.items() if v['aligned'] > 0]
                    f.write("#   (%d of %d cells did converge at least once.)\n"
                            % (len(won), len(_CELL_ALIGNS)))
            f.write("#\n# ==================== WINNING SET ====================\n")
            f.write("# The final chosen value for every tuned parameter, in\n")
            f.write("# constants.py form (%d of %d changed from the defaults).\n"
                    % (ws_changed, ws_ok))
            f.write("# Copy these lines straight into constants.py.\n")
            for ln in ws_lines:
                f.write(ln + "\n")
            f.write("# ========== end winning set - details follow ==========\n#\n")
            for attr, r in results.items():
                if r["status"] == "ok":
                    f.write("%s = %.6g    # was %.6g\n" % (attr, r["final"], r["orig"]))
                    for (pan, tilt), val, cost in r.get("per_step", []):
                        f.write("#     @(%+.0f,%+.0f): best %.6g  (cost %.1f)\n" % (pan, tilt, val, cost))
                    for k in ("hysteresis", "nearmiss", "confirm", "headroom", "note"):
                        v = (r.get("notes") or {}).get(k)
                        if v:
                            f.write("#     %s: %s\n" % (k, v))
                else:
                    f.write("# %s skipped (%s)\n" % (attr, r["status"]))
        # A completed sweep/cell report is KEPT, not claimed away: it is
        # timestamped, so a rerun writes a new report beside the previous one
        # rather than overwriting it, and the operator clears them with Delete.
        # Only the resume checkpoint is cleaned (see _save_checkpoint).  The
        # 'autotune_report'/'cell_report' exports pick the newest.
        log.info("[CATUNE] Tuned values written to %s", path)
    except Exception:
        log.exception("[CATUNE] could not write results file")
    # The suggested-constants file: the WINNING SET's CHANGED values substituted into
    # a copy of the live constants.py (unchanged parameters already equal the
    # file's values, so only the changed lines are touched - minimal diff).
    # Guarded: a generation failure must never break the report.
    try:
        scalar_subs, note_vals = {}, []
        for attr, r in results.items():
            if r["status"] != "ok":
                continue
            if abs(float(r["final"]) - float(r["orig"])) <= 1e-9:
                continue
            const = DEFAULT_CONST.get(attr)
            if const is None:
                continue
            scalar_subs[const] = repr(float(r["final"]))
            note_vals.append("%s = %s (was %s)" % (const, repr(float(r["final"])),
                                                   repr(float(r["orig"]))))
        if scalar_subs:
            notes = (["WINNING SET, changed values only (%d of %d tuned parameters):"
                      % (len(scalar_subs), sum(1 for r in results.values()
                                               if r["status"] == "ok"))]
                     + ["  " + v for v in note_vals]
                     + ["per-cell details + validation verdict: %s"
                        % os.path.basename(path)])
            write_suggested_constants(
                file_export.stamped(profile.suggest_name, "py", run_ts),
                "%s ('%s')" % (profile.name.lower(), profile.key), notes, log,
                scalar_subs=scalar_subs)
        else:
            log.info("[CATUNE] no parameters changed - %s " % profile.suggest_name +
                     "not written.")
    except Exception:
        log.exception("[CATUNE] %s generation failed - use the " % profile.suggest_name +
                      "WINNING SET block in the report instead.")
    log.info("[CATUNE] These are LIVE but not saved - bake them into constants.py to keep them.")
    log.info("=" * 60)


def run_controller_autotune_sweep(ctrl, abort, profile=SWEEP_PROFILE):
    """Entry point (daemon thread, started by controller.toggle_autotune via 't').

    ctrl is the BalloonChaser; abort is a threading.Event.  Runs up to
    MAX_PASSES coordinate-descent passes over SWEEP_PLAN - re-tuning each gain
    against the others' CURRENT values until the values stop moving (the gains are
    not orthogonal, so one pass is not enough) - then a combined-set validation
    trial.  Winners are applied live.  Always returns to LOITER (autotune_finish)."""
    log.info("=" * 60)
    log.info("*** CONTROLLER AUTOTUNE SWEEP - %d params, cost-based bracket search "
             "(inner->outer), multi-pass coordinate descent (<=%d passes) with "
             "hysteresis + confirmation re-fly + headroom gates. SIM ONLY. "
             "Press 't' to abort. ***", len(SWEEP_PLAN), MAX_PASSES)
    log.info("=" * 60)
    sweep_t0 = time.monotonic()
    ctrl.autotune_start_t = sweep_t0                # start the HUD MM:SS timer
    store = ControllerAutotuneStore(ctrl)
    log.info("[CATUNE] pan-axis entries fly cells down to %.0f deg tilt "
             "(GRID_TILT_CAP_DEG).", GRID_TILT_CAP_DEG)
    _CELL_ALIGNS.clear()          # per-sweep, not per-process

    # Enter ACRO ONCE for the whole sweep (stays in ACRO between trials).
    if not ctrl.ptest_enter_acro():
        log.error("[CATUNE] could not enter ACRO; aborting sweep.")
        ctrl.autotune_finish()
        return

    # Build the maneuver grid (full pan x tilt cross-product from the tuner cells).
    grid = _maneuver_grid(ctrl, profile)
    if not grid:
        log.error("[CATUNE] empty maneuver grid; aborting sweep.")
        ctrl.autotune_finish()
        return
    progress = _Progress()

    # NOISE FLOOR - before a single parameter is touched.  Uses the axis
    # of the plan's dominant entry so the trials are the same shape as the ones
    # every later decision is made from.  Skipped on RESUME: the margins that
    # matter are the ones the interrupted run measured, and re-flying them would
    # both cost time and risk a different answer mid-sweep.
    noise_floor = None
    _nf_axis = SWEEP_PLAN[0]["axis"] if SWEEP_PLAN else "pan"

    # RESUME from an incomplete_ checkpoint if one exists and still matches this sweep;
    # otherwise start FRESH (reset the swept params to their constants.py DEFAULTS so
    # the brackets anchor to the real defaults and there is no run-to-run drift).
    results = {}            # attr -> {'orig','final','status','per_step'}  (insertion-ordered)
    settled = set()         # attrs frozen: converged, maxed out, or not tunable
    start_pass = 1
    resume_tuned = set()    # params already tuned in the (resumed) pass
    resume_moved = 0
    resume_in_pass = 0      # per-pass trial counter at the abort point (for "test x/y")
    in_progress = None      # partial (cand x cell) matrix of the parameter cut off mid-flight
    # Exactly ONE tuning (checkpoint) file should ever exist.  If a relaunch finds MORE
    # than one - stale files left by an older/!=version run - the resume balloon is
    # ambiguous: delete them all and start this sweep fresh rather than guess.
    _ckpts = glob.glob(os.path.join(os.getcwd(), profile.checkpoint_glob))
    if len(_ckpts) > 1:
        log.warning("[CATUNE] %d tuning files present (%s) - ambiguous; deleting all and "
                    "starting fresh.", len(_ckpts),
                    ", ".join(os.path.basename(p) for p in sorted(_ckpts)))
        for _p in _ckpts:
            try:
                os.remove(_p)
            except Exception:
                log.exception("[CATUNE] could not delete stale tuning file %s", _p)
    anchors = None      # None = brackets anchored to constants.py
    wide = bool(int(round(float(getattr(ctrl, 'autotune_bracket_wide', 0)))))
    log.info("[CATUNE] bracket: %s - %d candidates per parameter, reach x%.3g",
             "WIDE" if wide else "narrow",
             len(BRACKET_FACTORS_WIDE if wide else BRACKET_FACTORS),
             (BRACKET_FACTORS_WIDE if wide else BRACKET_FACTORS)[-1])
    ckpt = _load_checkpoint(ctrl, profile)
    if ckpt is not None:
        for a, v in ckpt.get("param_values", {}).items():
            try:
                store.set(a, float(v))
            except Exception:
                pass
        settled = set(ckpt.get("settled", []))
        results = ckpt.get("results", {})
        start_pass = int(ckpt.get("pass_idx", 1))
        resume_tuned = set(ckpt.get("tuned_this_pass", []))
        resume_moved = int(ckpt.get("moved", 0))
        resume_in_pass = int(ckpt.get("in_pass", 0))
        in_progress = ckpt.get("in_progress")
        progress.total = int(ckpt.get("total_trials", 0))
        # Resume with the SAME bracket anchors the interrupted run used.  Absent
        # (a checkpoint from before this field existed) means it was a
        # constants-anchored run, which is what anchors=None selects.
        _a = ckpt.get("anchors")
        anchors = {k: float(v) for k, v in _a.items()} if _a else None
        log.info("[CATUNE] RESUMING from checkpoint (saved %s): pass %d, %d/%d settled, "
                 "%d param(s) already done this pass, %d trials so far.  Brackets "
                 "anchored to %s.",
                 ckpt.get("saved_at", "?"), start_pass, len(settled), len(SWEEP_PLAN),
                 len(resume_tuned), progress.total,
                 "the run's saved start values" if anchors else "constants.py")
    else:
        # WHERE THE SEARCH STARTS.  "Use Constants" ON (the default) wipes the
        # swept gains back to constants.py, so a run is reproducible and
        # independent of whatever the Gain Tuner happened to be showing.  OFF
        # leaves the operator's GUI values in place and anchors every bracket to
        # them instead - the search then genuinely starts from the operator's
        # configuration rather than being handed the constants value as
        # candidate 3 of 5.
        use_const = bool(int(round(float(
            getattr(ctrl, profile.use_constants_attr, 1)))))
        if use_const:
            for entry in SWEEP_PLAN:
                store.reset_one(entry["attr"])
            log.info("[CATUNE] no checkpoint - starting fresh; reset %d swept "
                     "parameters to their defaults (Use Constants ON).",
                     len(SWEEP_PLAN))
        else:
            anchors = {}
            for entry in SWEEP_PLAN:
                try:
                    anchors[entry["attr"]] = float(store.get(entry["attr"]))
                except Exception:
                    pass
            log.warning("[CATUNE] Use Constants is OFF - starting from the Gain "
                        "Tuner's CURRENT values and anchoring every bracket to "
                        "them.  constants.py is not consulted for these %d gains, "
                        "so its values are NOT guaranteed to be among the "
                        "candidates.", len(anchors))
            for a in sorted(anchors):
                log.info("[CATUNE]   anchor %-28s = %.6g", a, anchors[a])
        # Measure repeatability on the freshly-reset defaults, before anything
        # is tuned - so the number describes the aircraft, not a half-tuned
        # configuration.
        noise_floor = _noise_floor_pass(ctrl, grid, _nf_axis,
                                        abort, progress,
                                        AUTOTUNE_NOISE_FLOOR_REPS)
        if noise_floor is None and abort.is_set():
            ctrl.autotune_finish()
            return
        # 'y' ONLY: refuse a cell whose trials are mostly bank-limited.  At the
        # limit the roll command is the limit rather than the law's output, so
        # every candidate gain produces the same trajectory and the cost cannot
        # tell them apart - "the best achievable gain set" is not defined there.
        # Better to say so before flying a full descent than to return a
        # confident number derived from an open loop.  Measured at the ENTRY
        # gains during the noise floor, which is the cheapest honest sample.
        if profile.single_cell and noise_floor is not None:
            sat = noise_floor.get("sat_frac")
            if sat is not None and sat > AUTOTUNE_CELL_MAX_SAT_FRAC:
                log.error("[CATUNE] cell (%+.0f,%+.0f) is bank-limited for %.0f%% of "
                          "the scored window (limit %.0f%%).  The loop is open there, "
                          "so every candidate scores alike and there is no best to "
                          "find.  Pick a gentler bearing, or raise the bank limit.",
                          grid[0][0], grid[0][1], 100.0 * sat,
                          100.0 * AUTOTUNE_CELL_MAX_SAT_FRAC)
                ctrl.autotune_finish()
                return

    log.info("[CATUNE] maneuver grid = %d cell(s): %s", len(grid),
             ", ".join("(%+.0f,%+.0f)" % c for c in grid))
    if profile.single_cell and grid[0][1] < GRID_TILT_CAP_DEG:
        # _entry_grid refuses to hand back an empty grid, so a single cell below
        # the pan-axis grid cap is flown anyway rather than skipping the eight
        # pan-axis gains entirely.  That is the right call for 'y' - the
        # operator picked this bearing deliberately - but it means the pan-axis
        # gains are being tuned outside the domain the cap exists to enforce, so
        # it is said out loud rather than left to be inferred from the report.
        log.warning("[CATUNE] cell latitude %+.0f is steeper than the pan-axis "
                    "grid cap (%.0f); the pan-axis gains will be tuned "
                    "OUTSIDE their documented domain.",
                    grid[0][1], GRID_TILT_CAP_DEG)
    _p1 = _pass_trials(SWEEP_PLAN, grid, wide)
    _pan_cells = len(_entry_grid({"axis": "pan"}, grid))
    log.info("[CATUNE] pass 1 = up to %d trials (successive elimination usually "
             "finishes well under plan): pan-axis entries fly %d cell(s) "
             "(capped at tilt >= %.0f deg, the pan-axis cap), pitch entries "
             "all %d, x %d candidates each. Each later pass re-tunes ONLY the gains "
             "whose choice is still moving and prints its own exact count at its "
             "start; the GRAND total is convergence-dependent (typically 2-3 "
             "passes), bounded by %d. Counts are grid evaluations - altitude "
             "re-flies, climbs, the 3s level-out, the final validation, "
             "confirmation re-flies (<=2/param on a proposed switch), near-miss "
             "re-flies (<=4/param on a photo-finish) and headroom probes "
             "(<=1/param, top-of-bracket winners only) are extra (not "
             "counted).",
             _p1, _pan_cells, GRID_TILT_CAP_DEG, len(grid),
             len(BRACKET_FACTORS_WIDE if wide else BRACKET_FACTORS),
             MAX_PASSES * _p1)

    # Per-trial checkpoint writer, handed to _tune_one so the file is rewritten after
    # EVERY flown cell - the crash / Ctrl+C safety net.  (The sweep runs in a daemon
    # thread that is terminated outright on Ctrl+C, and the checkpoint is only ever cleared
    # on NORMAL completion, so whatever the last per-trial write left on disk is exactly
    # what a relaunch resumes from.)  Reads the live pass state by closure - the params
    # settled / done SO FAR - and folds in the in-progress parameter's partial matrix.
    def _save_progress(in_progress_now):
        _save_checkpoint(store, settled, pass_idx, tuned_this_pass, moved, results,
                         progress.total, ctrl, progress.in_pass, in_progress_now,
                         profile, anchors)

    validation = None
    aborted = False
    crashed = False
    try:
        for pass_idx in range(start_pass, MAX_PASSES + 1):
            to_tune = [e for e in SWEEP_PLAN if e["attr"] not in settled]
            progress.start_pass(pass_idx, _pass_trials(to_tune, grid, wide))
            # On the FIRST pass after a resume, skip params already tuned in that pass,
            # carry the moved-count forward, and CONTINUE the per-pass trial counter from
            # where it paused (so the HUD "test x/y" picks up instead of restarting at 0).
            # pass_total is re-sized to (already-done + work-remaining) so x still ends at
            # y even though the interrupted parameter is re-flown.  Later passes are clean.
            if ckpt is not None and pass_idx == start_pass:
                moved = resume_moved
                tuned_this_pass = set(resume_tuned)
                pending = [e for e in to_tune if e["attr"] not in resume_tuned]
                progress.in_pass = resume_in_pass
                progress.pass_total = resume_in_pass + _pass_trials(pending, grid, wide)
                # The interrupted parameter re-flies ONLY its unflown cells, so drop the
                # cells it already has from the planned count (keeps "test x/y" ending at y).
                if in_progress and pending and in_progress.get("attr") == pending[0]["attr"]:
                    progress.pass_total -= len(in_progress.get("filled", []))
            else:
                moved = 0
                tuned_this_pass = set()
                pending = to_tune
            log.info("[CATUNE] ===== pass %d/<=%d - %d param(s) to tune, %d trial(s) "
                     "this pass (cumulative so far: %d) =====", pass_idx, MAX_PASSES,
                     len(pending), progress.pass_total, progress.total)
            for entry in pending:
                attr = entry["attr"]
                if abort.is_set() or ctrl.mode != 'autotune':
                    aborted = True
                    break
                # Resume the interrupted parameter from its saved partial matrix (only the
                # first pending parameter can carry one); every parameter writes the
                # checkpoint after each of its trials via _save_progress.
                rs = in_progress if (in_progress and in_progress.get("attr") == attr) else None
                status, base, final, changed, per_step, notes = _tune_one(
                    ctrl, store, entry, abort, progress,
                    _entry_grid(entry, grid), rs, _save_progress, anchors, wide)
                in_progress = None              # consumed - later params start clean
                if attr not in results:
                    results[attr] = {"orig": base, "final": final, "status": status,
                                     "per_step": per_step, "notes": notes}
                else:
                    results[attr]["final"] = final
                    results[attr]["status"] = status
                    results[attr]["per_step"] = per_step
                    results[attr]["notes"] = notes
                if status == "abort":
                    aborted = True              # interrupted mid-param: NOT marked done,
                    break                       # so a resume re-tunes only its unflown cells
                tuned_this_pass.add(attr)       # fully processed this pass
                if status == "skip":
                    settled.add(attr)           # not tunable - don't retry
                elif not changed:
                    # Freeze a parameter once its chosen candidate is stable vs the
                    # previous pass (its argmin over the fixed bracket stopped moving).
                    settled.add(attr)
                else:
                    moved += 1                  # still moving: re-tune against the others next pass
                # Commit this finished parameter (and clear the in-progress matrix) so a
                # crash now resumes at the NEXT parameter rather than re-flying this one.
                _save_progress(None)
            if aborted:
                # The per-trial / per-parameter writes already left the live state on disk
                # (with the in-progress matrix when a parameter was mid-flight), so there is
                # nothing extra to save here - just stop and let a relaunch resume.
                log.info("[CATUNE] aborted during pass %d - progress is saved to %s; "
                         "relaunch to resume from here.", pass_idx, profile.checkpoint_name)
                break
            log.info("[CATUNE] pass %d done: %d gain(s) still moving, %d/%d settled "
                     "(%d trials so far).", pass_idx, moved, len(settled),
                     len(SWEEP_PLAN), progress.total)
            if moved == 0:
                log.info("[CATUNE] converged after %d pass(es).", pass_idx)
                break
        # Combined-set validation: confirm the FULL tuned set holds alignment together
        # at the most aggressive grid cell.
        if not aborted and results:
            log.info("[CATUNE] running combined-set validation (hardest cell, 1 trial) ...")
            ctrl.autotune_status = "combined validation (hardest cell)"
            validation = _validate_combined(ctrl, abort, grid)
    except Exception:
        crashed = True
        log.exception("[CATUNE] sweep crashed; returning to loiter")
    finally:
        ctrl.autotune_finish()
        ctrl.autotune_status = None      # clear the HUD banner (sweep is over)
    _dur = int(time.monotonic() - sweep_t0)
    log.info("[CATUNE] sweep wall-clock: %02d:%02d (MM:SS).", _dur // 60, _dur % 60)
    if aborted:
        # Incomplete: the checkpoint was written at the abort point; do NOT write the
        # final report (the run isn't done).  Relaunching resumes from the checkpoint.
        log.info("[CATUNE] sweep INCOMPLETE (%d trials so far) - relaunch to resume from %s.",
                 progress.total, profile.checkpoint_name)
    elif crashed:
        # Crash: keep any existing checkpoint so progress isn't lost; still write a
        # partial report for diagnosis.
        log.warning("[CATUNE] sweep crashed mid-run; any saved checkpoint is preserved "
                    "for resume. Writing a partial report.")
        _report_and_save(results, validation, progress.total, noise_floor=noise_floor,
                         ctrl=ctrl, grid=grid, profile=profile, anchors=anchors)
    else:
        # Completed (or converged): drop any checkpoint and write the final report.
        _clear_checkpoint(profile)
        _report_and_save(results, validation, progress.total, noise_floor=noise_floor,
                         ctrl=ctrl, grid=grid, profile=profile, anchors=anchors)
