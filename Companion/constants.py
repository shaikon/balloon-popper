# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""
Constants and tuning defaults for the vision balloon chaser.

Pure data; only numpy is imported, for the HSV arrays (tagged SIM-ONLY).

Tagging: SHARED applies to sim and real; SIM-ONLY is Gazebo/HSV-specific;
REAL-AIRFRAME values are overwritten from the FC at startup or are airframe
defaults.
"""
import numpy as np

# == SHARED: roll outer steering ==
# Linear proportional gain: deg of bank per deg of pan error.
PAN_P_GAIN           = 3.5
PAN_KI               = 0.6
# Integrator state-clamp reference: max_ri = BANK_LIM_DEG *
# WINDUP_LIMIT_PAN / PAN_KI (pan_ki cancels in the I-term
# contribution).  Re-sweep if the loop gain changes.
WINDUP_LIMIT_PAN     = 0.2
# The single bank-angle clip on the bank-to-turn command.  Set by hand (the
# Gain Tuner's Bank Lim row), never swept: no sweep may search over it.  At 90
# the clip is effectively inert - the bank proxy cannot exceed 90 by
# construction, and above 67.5 deg pitch the reachable proxy (90 - |pitch|) is
# tighter anyway; lower it to cap authority directly.  A large bank demand can
# put the aircraft inverted, where the law's upright geometry stops meaning
# anything and height sheds fast; during a SWEEP, altitude protection
# (AUTOTUNE_PROTECT_FLOOR_M) is what catches that, not this number.
#
# It sets control AUTHORITY, not safety. A sweep free to lower it will,
# because a smaller limit produces less overshoot and overshoot is scored:
# it buys score by spending the authority the aircraft needs to line up.
# Load-factor protection is apply_load_factor_roll_limits() in ArduPlane and
# is not this value's job.
BANK_LIM_DEG         = 45.0
ROLL_GAIN            = 4.0
# == 'p' TEST THROTTLE OVERRIDE ==
# With SPECIAL_THROTTLE_ENABLE on, a 'p' test flies at SPECIAL_THROTTLE_PCT
# instead of the operator's THROTTLE_PCT, and reverts the moment the test ends.
#
# Exists because a capture is where throttle matters most and is hardest to set:
# a climb and a dive at the same throttle end up at very different airspeeds
# (measured 15.6 vs 20.1 m/s on a matched +/-40 pair at 1850 PWM), and pan
# authority goes as 1/V, so the two halves of a comparison run at different loop
# gains through no fault of the gains.  Setting the test throttle separately
# lets a p-test be flown at a chosen energy without disturbing the throttle the
# aircraft uses everywhere else.
#
# The override is applied where the throttle is USED, not by writing
# THROTTLE_PCT - so an aborted or crashed test cannot leave the aircraft
# carrying a test setting.
SPECIAL_THROTTLE_ENABLE      = 0      # 1 = use SPECIAL_THROTTLE_PCT during 'p'
SPECIAL_THROTTLE_PCT         = 0.0    # percent, 0-100
# ROLL-TO-PLANE.  On a steep flight path the roll axis stops banking toward
# the balloon and rolls it round the nose instead: to 12 o'clock (over the
# canopy, then the pitch law pulls) or 6 o'clock (under the belly, then it
# pushes), whichever is nearer.  Its share of the roll command is the share
# from the flight-path angle (STEEP_START / STEEP_FULL below), and
# bank-to-turn has the rest (LUA/cascade.lua).
# The 12-or-6 choice flips only once the other end is nearer by this much, so a
# balloon near 3 or 9 o'clock does not swing the roll command from full one way
# to full the other.
PLANE_SIDE_HOLD_DEG   = 15.0   # deg past the wingtip line before 12 <-> 6 changes
# Roll-to-plane's own roll-rate gain, deg/s of roll per deg still to roll to 12
# or 6.
PLANE_GAIN            = 9.0
# Roll-to-plane's feedforward on the balloon's drift round the clock: deg/s of
# roll per deg/s of drift, 1 matching the drift exactly.  0 is no feedforward.
PLANE_FF_GAIN         = 0.0
# Low-pass on that drift, expressed as a RATE (1/s) rather than a time
# constant, so the knob grows the way the effect does: bigger = snappier (less
# smoothing), smaller = more smoothing, 0 = frozen.  rate = 1 / time-constant.
PLANE_FF_LPF_RATE     = 2.5
# The blend: roll-to-plane's share of the roll command, from the flight path's
# angle above or below the horizon.  None below STEEP_START, so banking toward
# the balloon flies alone wherever the balloon is; all of it at and beyond
# STEEP_FULL, where bank-to-turn has none; a straight line between.  The
# cosine of that angle is the share of gravity acting across the path, the
# bank does not enter it, and the nose's pitch stands in below
# FLIGHT_PATH_MIN_SPEED_MPS.  Neither angle can pass 90, so 90 / 90 makes the
# share whole only moving exactly vertically, which switches roll-to-plane off.
PLANE_STEEP_START_DEG = 65.0   # deg of flight-path angle where roll-to-plane starts taking a share
PLANE_STEEP_FULL_DEG  = 80.0   # deg of flight-path angle where roll-to-plane has the whole share
# At or below this speed the velocity's direction is noise, and nothing that
# reads the flight path from it (the path frame every balloon error reads, the
# steep share, the companion's alignment errors) trusts it: the nose stands in.
FLIGHT_PATH_MIN_SPEED_MPS = 3.0
# Roll-to-plane's integrator, PARALLEL form like the yaw pointing term's: deg/s
# of roll per second of clock error held, added to the rate demand directly.
PLANE_KI              = 0.0    # 1/s
# Anti-windup ceiling for that integrator, as a fraction (0-1) of the ACRO roll
# rate: the share of the roll rate the integrator alone may command.  The
# share the pan and yaw-pointing integrators own of their envelopes.
PLANE_WINDUP          = WINDUP_LIMIT_PAN
# Roll-to-plane's derivative (damping) term: deg/s of roll per deg/s of the
# clock error's rate, DIMENSIONLESS like PITCH_KD.  While a roll carries the
# balloon toward 12 or 6 the error's rate opposes the error, so a positive gain
# takes rate off the command before the balloon arrives.  0 is no derivative
# term.
PLANE_KD              = 0.0
# 1/s; low-pass on the clock error's rate, expressed as a rate (bigger = less
# smoothing, 0 = frozen).  rate = 1 / time-constant.
PLANE_KD_RATE         = 10.0

# Runtime-switch default (the tuner's reset-to-default; controller __init__
# reads it).
DRAW_ROLL_ERROR = 1      # HUD roll-error overlay on by default
# Controller-Autotune MANEUVER GRID (GUI-configurable via the tuner).  The sweep
# tests every swept gain across the FULL cross-product of these pan x tilt bearings
# (degrees), so each gain is scored over the whole maneuver envelope (worst-case),
# not at a single operating point.  Tilt is the dive angle below the horizon
# (NEGATIVE = nose-down), e.g. min=0 max=-80 step=5 sweeps 0..-80 deg dives.  Grid
# size = pan-count x tilt-count and the trial count scales with it (each cell is
# flown for every candidate of every gain on every pass), so widen ranges / shrink
# steps DELIBERATELY - a 5 deg step over 0..80 is 17 values; 17 x 17 = 289 cells is
# enormous.
# == WHERE A TUNING RUN STARTS ==
# 1 = ON (default): the run resets the swept gains to their constants.py values
#     first, so it is reproducible and independent of whatever the Gain Tuner
#     was showing.  Every bracket is anchored to the constants value.
# 0 = OFF: the run keeps the Gain Tuner's CURRENT values and anchors every
#     bracket to them instead.  constants.py is not consulted for the swept
#     gains, so its values are NOT guaranteed to be among the candidates - the
#     search has to find them.  That is what makes it a real test of the
#     mechanism rather than a test of recognising a value it was handed.
# The anchors are written into the run's checkpoint, so an interrupted run
# resumes searching the same brackets rather than falling back to constants.
# One switch per mechanism; plant-ID ('i') has none, because it measures the
# plant rather than searching gains and has no starting gain set.
# == HOW WIDE A NET EACH GAIN IS SEARCHED ON ==
# 0 = narrow (default): 5 candidates spanning x0.5 .. x2.0 of the anchor.
# 1 = WIDE: 9 candidates spanning x0.2 .. x5.0 - reach x5 instead of x2, for
#     1.8x the flight time.  Use it when a gain may be badly off (recovering a
#     spoiled set, or a new airframe), not for routine tuning.
# 9 points rather than 5-at-wider-spacing on purpose: the headroom gate reads the
# next candidate UP from the flown matrix, so the STEP RATIO is the gate.  9
# points gives 1.495 against the narrow set's 1.400, leaving that gate the
# meaning it was tuned for; 5 points spanning the same range would make it 2.24
# and quietly start rejecting good gains.
# Shared by 't' and 'y'; per-entry ENTRY_BRACKETS overrides still win.
AUTOTUNE_BRACKET_WIDE = 0

AUTOTUNE_USE_CONSTANTS      = 1     # 't' grid sweep
CELL_AUTOTUNE_USE_CONSTANTS = 1     # 'y' single cell

# == 'y' SINGLE-CELL AUTOTUNE: the one bearing it tunes at ==
# Same engine, same SWEEP_PLAN, same scoring as 't' - but flown at ONE cell
# instead of the grid, so the answer is the best achievable AT THIS BEARING.
# Same spherical frame as the 'p' test and the 't' grid: longitude is the
# heading-relative azimuth, latitude the horizon-referenced elevation with
# NEGATIVE = nose-down.  Editable live in the Gain Tuner.
AUTOTUNE_CELL_LON_DEG = 0.0
AUTOTUNE_CELL_LAT_DEG = -20.0

# 'y' refuses to tune a cell whose trials spend more than this fraction of the
# scored window at the bank limit.  A saturated trial is open-loop - the command
# is pinned regardless of the error - so every candidate gain scores the same and
# "best achievable" is not defined.  't' does not need this: it scores on the
# WORST cell over a grid, so an unusable cell is one voice among several, whereas
# for 'y' it is the entire measurement.  Measured during the noise-floor pass, at
# the entry gains, before any tuning is attempted.
AUTOTUNE_CELL_MAX_SAT_FRAC = 0.5

AUTOTUNE_PAN_MIN   = 0.0
AUTOTUNE_PAN_MAX   = 80.0
AUTOTUNE_PAN_STEP  = 5.0
AUTOTUNE_TILT_MIN  = 0.0
AUTOTUNE_TILT_MAX  = -90.0   # swept to vertical
AUTOTUNE_TILT_STEP = 5.0
# Altitude protection for the sweep AND plant-ID: aggressive dive trials lose
# height, so if AGL altitude (link.relative_alt, metres) falls below FLOOR the
# routine pauses, climbs back to CLIMB_TO, and RE-FLIES / re-attempts the
# interrupted trial (it does NOT skip it).  Climb high once, dive many times.
# CLIMB_TO must leave headroom above FLOOR; launch (--alt) should be >= it.
# The simulated air thins with height as real air does (AltitudeLiftDrag), so
# the propeller pushes less and every margin shrinks up high: CLIMB_TO is kept
# as low as the manoeuvres' height loss allows, near the air the aircraft
# competes in.
AUTOTUNE_PROTECT_FLOOR_M    = 150.0
AUTOTUNE_PROTECT_CLIMB_TO_M = 1000.0
# Unified plant-ID ('i' key): which axes the run includes, each a Gain-Tuner
# on/off switch.  All on by default; the run flies only the enabled axes, in the
# order pitch -> yaw -> roll (cheap open-loop doublets before the long roll sweep).
PLANTID_DO_PITCH            = 1
PLANTID_DO_YAW             = 1
PLANTID_DO_ROLL            = 1
# Plant-ID rate-PID aggressiveness: the target closed-loop rate bandwidth as a
# multiple of the flight-validated reference (plant_id.RATE_BW_REF_RADPS), which
# maps the identified plant to the suggested <AXIS>_RATE_P / _D.  1.0 reproduces
# the hand-found PTCH_RATE_P=1.0 (Notes/autotune_mismatch_report.md); >1 is a
# faster/harder rate loop, <1 softer.  Read only when the report is generated;
# it changes no flight behaviour and no stored gain.
PLANTID_RATE_AGG           = 1.0
# Trim map ('e' key): the grid of cells it flies, each a climb angle (deg,
# NEGATIVE = dive, world-referenced) held at an airspeed (m/s EAS) by the FC's
# airspeed hold.  Each axis runs min to max in steps; see trim_map.py.
TRIM_MAP_CLIMB_MIN_DEG  = -45.0
TRIM_MAP_CLIMB_MAX_DEG  = 60.0
TRIM_MAP_CLIMB_STEP_DEG = 15.0
# Below about 15 m/s EAS the SITL airframe's elevator reaches its stop before
# the wing holds level flight, and a slower cell only mushes and sinks.
TRIM_MAP_SPEED_MIN_MPS  = 15.0
TRIM_MAP_SPEED_MAX_MPS  = 21.0
TRIM_MAP_SPEED_STEP_MPS = 2.0
# Its own altitude protection, metres AGL: below FLOOR it climbs to CLIMB_TO
# before the next cell (and re-flies a cell that breaches it).  The same rule as
# AUTOTUNE_PROTECT_* for 't' and 'i', with values of its own.
TRIM_MAP_PROTECT_FLOOR_M    = 150.0
TRIM_MAP_PROTECT_CLIMB_TO_M = 1000.0
# Shared alignment tolerance (deg, Pythagorean over pan+tilt error).  The HUD
# "Nose Aligned"/"Path Aligned" indicators light below this, and the Controller
# Autotune sweep uses it as its lock tolerance - one source of truth so "locked"
# means exactly the same as the on-screen aligned indicators.
ALIGN_TOL_DEG = 0.5

# Lateral-error derivative (lead) term on the bank command (0 = off).
# Rolls wings level before the pointing error reaches zero, suppressing the
# turn-overshoot oscillation.  Derivative is low-pass filtered.
PAN_KD          = 0.4
# 1/s; lateral-derivative low-pass, expressed as a rate (bigger = less
# smoothing, 0 = frozen).  rate = 1 / time-constant.
PAN_KD_RATE     = 10.0

# == AIRSPEED COMMAND (throttle loop) ==
# Which quantity the OPERATOR commands.
#   0 = THROTTLE (default).  throttle_pct goes to CH3 unmodified and nothing in
#       the program computes throttle.
#   1 = AIRSPEED.  The operator sets AIRSPEED_CMD_MPS and a PI loop drives
#       THROTTLE to hold it.  throttle_pct becomes the loop's output rather than
#       its input.
# STRICTLY THROTTLE-ONLY.  This loop's single actuator is CH3.  It never touches
# pitch, roll, the steering error, the cascade, or any acceptance window, so it
# cannot buy airspeed by trading altitude or by shallowing a turn.  That is not a
# tuning preference: a speed loop that is allowed to steer competes with the
# balloon chaser law for the same surfaces and the two wind each other up.  The pointing
# law owns the airframe; this owns the motor.  Consequence worth stating: if the
# commanded speed is unreachable at the attitude the balloon chaser law is holding, the
# throttle saturates and the speed is simply missed.  That is the intended
# failure - missing the speed target is recoverable, fighting the balloon chaser law is
# not.
AIRSPEED_CMD_ENABLE = 0        # 0 = user commands throttle (default), 1 = user commands airspeed
AIRSPEED_CMD_MPS    = 15.0     # commanded airspeed when AIRSPEED_CMD_ENABLE = 1
# PI on airspeed error -> throttle percent.  The INTEGRATOR carries the trim
# throttle (it is seeded from the operator's setting on start and ranges over
# the throttle bounds), so P only has to cover the deviation from trim.  No D:
# airspeed telemetry arrives at ~8 Hz (VFR_HUD), far too slow and too quantised
# to differentiate.
AIRSPEED_CMD_KP     = 5.0      # throttle % per m/s of error
AIRSPEED_CMD_KI     = 5.0      # throttle % per m/s per second
# Airspeed below this is treated as a dead/stale sensor and the loop hands
# control back to throttle_pct rather than firewalling the motor against a
# reading it does not believe.
AIRSPEED_CMD_MIN_VALID = 2.0   # m/s

# ---- Camera VIEW switches (display only; never control) ---------------------
# Both OpenCV windows cost real time on the companion: at a 320x240 render
# scaled to a 1280x720 display, the per-frame colour convert + resize is
# ~1.01 ms (the resize is 0.98 of that), and each tick adds a 0.21 ms frame
# copy plus ~0.16 ms of overlay drawing at 50 Hz.  That is ~5% of a core at
# 30 fps before cv2.imshow, whose cost under WSLg (every frame through the
# Wayland compositor) is larger still.
# The NOSE camera is display-only in the strict sense - detection runs off the
# GIMBAL camera, so switching the nose view off cannot affect control.  The
# gimbal CAMERA must keep running for detection; only its WINDOW is optional.
#
# SAFETY INVARIANT: the keyboard (q / i / t / arrows) survives both
# windows closed.  cv2.waitKeyEx can deliver keys only to a live HighGUI window,
# so the Gain Tuner queues keystrokes too and the keyboard handler drains them
# when waitKeyEx has none.  See _apply_camera_views and KeyboardHandler.
NOSE_CAM_DISPLAY   = 0    # 'Balloon Chaser Camera' window: 0 = OFF (default), 1 = ON
GIMBAL_CAM_DISPLAY = 1    # 'Gimbal Camera'  window: 0 = OFF, 1 = ON (default)
# 1 = the controller may auto-move the gimbal while vision-tracking in l/x
# (default); 0 = it must not.  'g' (gimbal freeze) and mouse aiming are
# unaffected either way - this gates only the automatic l/x tracking slew.
AUTO_GIMBAL        = 1
# Background FOV estimator (DIAGNOSTIC ONLY, default OFF).  1 = on: a passive
# observer reads the camera's horizontal FOV out of motion that is already
# happening (aircraft rotation and gimbal slew vs. the pixel shift) and writes
# fov_est_* to the q CSV log for post-run comparison against ground truth.  It
# NEVER commands motion and its output NEVER enters the control law (C-02) - see
# FovEstimator in controller.py and Documentation/encoder_rate_lead.md.
FOV_EST_ENABLE     = 0

# =============================================================================
# MEASURED AIRFRAME CHARACTERISATION - roll axis
# =============================================================================
# Every number below is a MEASUREMENT, not a tuning choice and not a derivation.
# Source: three frequency-response ID runs ('k' key, plant_freq_id.py) on
# 36 cells total = 6 flights x 6 pitch attitudes (+/-20, +/-40,
# +/-55 deg), airspeeds 12/16/20 m/s, all on the FC clock.
#
# TOLERANCES ARE RUN-TO-RUN where a run-to-run figure exists, not the standard
# error of a pooled fit.  Cells flown in one flight share that flight's
# conditions, so pooling them as independent samples understates the real
# uncertainty - for the asymmetry coefficient, by more than double.
#
# Characterisation only: no control path reads them.  They live here so the
# plant model has one home with provenance attached instead of numbers
# scattered through analysis scripts.  Re-measure with 'k' after any airframe
# change.
#
# WHAT THEY ARE FOR.  Differentiating the bank proxy gives an exact identity:
#     bank_rate = ( cos(roll)cos(pitch)*p + sin(pitch)*r ) / cos(bank)
# The second term is uncommanded and flips sign with pitch, so a
# climb and a descent are different plants.  Its size is sin(pitch) times the
# bank-to-yaw response, which is the one thing that had to be measured to close
# the model.

# --- ROLL RESPONSE: roll-rate command -> roll rate ------------------------
# The FC rate loop, the servo and the airframe roll response, lumped.
# Symmetric climb vs dive: ratio 0.96-1.02 and phase within a few degrees at
# every attitude, so this link carries NO part of the climb/dive difference.
#
# A roll-rate command arrives multiplied by the DC gain.
AIRFRAME_ROLL_RESP_GAIN      = 0.934   # +/- 0.004 (sd 0.019 over 18 cells)
AIRFRAME_ROLL_RESP_DELAY_S      = 0.102   # +/- 0.001 s, effective transport delay
#
# --- BANK-TO-YAW: bank angle -> yaw rate ----------------------------------
#     No rudder is commanded, so this is sideslip acting on the fin, and it is
#     the link that carries the asymmetry.
#
# It scales as exactly 1/V (confirmed 12-20 m/s, 0.7% scatter).
#
# NO FITTED LAW IS RECORDED HERE.  One was: a two-term fit in sin(pitch) whose
# odd coefficient WAS the climb/dive asymmetry.  It is deliberately gone.  The
# asymmetry is real and repeatable - measured across six sweeps, surviving
# matched pitch and matched flight path - but four force-balance derivations of
# it all failed, so the coefficient was an empirical number with no mechanism
# behind it.  A constant nobody can derive should not sit where a future reader
# will take it for physics.
#
# What survives is the raw per-cell measurement, which the 'k' report prints
# from the run in front of it.  If a law is wanted, fit it from those cells at
# the time, with that run's own scatter attached.
AIRFRAME_BANK_YAW_DELAY_S      = 0.21    # effective transport delay, both directions
#
# Yaw resonance (dutch roll), from the bank-to-yaw phase crossing -90 deg.  Same both
# directions.  Relevant because raising roll gain pushes the loop crossover
# toward it, so more inner gain measures WORSE rather than better.
AIRFRAME_YAW_WN_RADS     = 7.4     # +/- 0.2 rad/s (1.18 Hz)
AIRFRAME_YAW_ZETA        = 0.26    # +/- 0.02
#
# Yaw-axis plant seen by the rudder loop: a commanded yaw rate produces sideslip,
# and sideslip is what the lateral marker measures.  These two set that loop's
# gain and its dominant pole.  AIRFRAME_MARKER_PER_DPS is the reciprocal of the
# measured turn rate per degree of sideslip.
AIRFRAME_TURN_PER_SIDESLIP  = 0.819  # deg/s of turn per deg of sideslip
AIRFRAME_MARKER_PER_DPS     = 1.221  # deg of marker per deg/s of commanded rate
AIRFRAME_MARKER_TAU_S       = 1.22   # s, sideslip build-up to a rate command
AIRFRAME_SIDESLIP_MAX_DEG   = 9.0    # deg of sideslip at full rudder deflection
#
# == FREQUENCY-RESPONSE ID: roll-axis transfer functions ('k' key; SIM ONLY) ==
#
# Plant-ID answers "how much pan authority per degree of bank" -- one number per
# cell, at DC.  A gain is not a plant.  The roll loop rings near 2.5 rad/s,
# and what decides whether it rings is the GAIN AND PHASE of the roll axis at
# that frequency, which a DC number cannot express.
#
# This mode measures the two links the kinematics cannot supply:
#     roll response = p / p_cmd  -- rate loop, servo and roll damping
#     bank>yaw      = r / bank   -- yaw response to bank.  With no rudder this is
#                            sideslip acting on the fin: yaw stiffness,
#                            roll-yaw coupling, adverse yaw, dutch-roll damping.
# Everything BETWEEN them is exact kinematics and needs no measurement:
#     bank_rate = ( cos(roll)cos(pitch)*p + sin(pitch)*r ) / cos(bank)
#     pan_rate  = -r + tz*(p*tx + q*ty)/(tx^2 + ty^2)
# so the two together close the model.  Flown at +pitch and -pitch, they say whether a
# climb/dive difference survives once the kinematic terms are accounted for.
#
# Roll-rate square waves, one frequency at a time, balloon chaser roll loop overridden
# exactly as the doublet is.  Square rather than sine because the override
# commands a rate and a square wave is what it can hold cleanly; the fit takes
# the fundamental and ignores the harmonics.
# A cell is a BALLOON ELEVATION.  TWO offsets sit between it and what the aircraft
# flies, and both keep their sign in a climb and a dive, so neither cancels in a
# climb-vs-dive comparison:
#
#   cell  --(the gimbal aims, not the nose: -FREQID_TILT_OFFSET_DEG)--> pitch
#   pitch --(angle of attack: -FREQID_VERT_COMP_DEG)-------------------> flight path
#
# The two nearly cancel, so cell ~= FLIGHT PATH ANGLE while the pitch attitude
# sits about 4.5 deg below it.
#
# THIS HAS ALREADY CAUGHT ONE ANALYSIS, TWICE.  Because both offsets keep their
# sign, MATCHING |pitch| BETWEEN A CLIMB AND A DIVE NECESSARILY MISMATCHES
# |flight path| BY 2*|AOA| (~8.9 deg measured), AND VICE VERSA.  No cell set
# matches both.  Every climb-vs-dive result must therefore say which one was
# held, because the other is 9 deg out, and 9 deg is not small beside the ~13%
# asymmetry under investigation.
#
# The set below matches FLIGHT PATH at 20/40/55 deg.  The matched-|pitch| set,
#     (-50.5, 59.5, -35.5, 44.5, -15.5, 24.5)
# has already been flown.  Comparing the two runs is what separates
# a genuine pitch-ATTITUDE dependence from a flight-path one.
#
# Ordered dive, climb, dive, climb: each climb cell trades altitude for the
# manoeuvre and the following dive gives it back, so a six-cell run does not
# finish thousands of feet above where it started.
FREQID_TILT_OFFSET_DEG   = 4.5    # measured; cell - this = pitch actually flown
FREQID_VERT_COMP_DEG      = -4.7   # measured; roughly EVEN in pitch
                                  # (about -4.2 at 20 deg to -5.6 at 55 deg, both signs)
FREQID_PITCH_CELLS_DEG   = (-55.0, 55.0, -40.0, 40.0, -20.0, 20.0)
# Periods (s) -> 1.05 .. 8.98 rad/s, bracketing the observed ring.  Ordered slow
# to fast so the first cycles at a fresh cell are the gentlest.
FREQID_PERIODS_S         = (6.0, 3.5, 2.0, 1.2, 0.7)
FREQID_CYCLES            = 4      # scored cycles per frequency
FREQID_SETTLE_CYCLES     = 1      # leading cycles flown and discarded
# A rate square wave integrates to a triangular bank of amplitude rate*T/4, so
# the rate is scaled as 1/T to hold ONE bank amplitude across the sweep.  Equal
# excitation at every frequency is what makes the gains comparable; a fixed rate
# would drive 40 deg of bank at the slow end and 2 at the fast end, measuring
# the response at two different amplitudes and calling it one curve.
FREQID_BANK_GOAL_DEG   = 8.0
FREQID_RATE_MAX_DPS      = 90.0   # ceiling on the 1/T scaling at the fast end
FREQID_BANK_ABORT_DEG    = 30.0   # same guard the doublet uses, tighter
FREQID_MIN_COHERENCE     = 0.5    # below this a point is reported but not fitted
# A cell needs this many points above that coherence to count as measured at
# all.  The slowest drive is routinely the weakest - the closed loop cancels
# most of a 6-second injection, so its coherence sits near 0.4 - and one such
# point is survivable.  A cell that cannot muster three measured nothing.
FREQID_MIN_GOOD_POINTS   = 3

# ---- frequency sweep: ferry to a cell ----------------------------------------
# The frequency sweep flies each cell by pointing the law at it, then waits for
# the aircraft to settle before a measurement window may open.
#
# Pitch tolerance: the gate's tilt error is attitude-vs-BALLOON, but the aircraft
# flies at a few degrees angle of attack (attitude != velocity vector), so the
# settled error sits at the flight-path/AoA gap, which grows with dive angle.
# The tolerance accepts that gap with margin; the measurement needs steady BANK,
# not tight pitch.
PLANTID_GATE_PITCH_TOL    = 10.0    # deg    |pitch error| ferry gate
PLANTID_GATE_RATE_TOL     = 8.0     # deg/s  |p|,|q| ferry gate
PLANTID_GATE_HOLD_S       = 0.6     # s      gate must hold this long
PLANTID_FERRY_TIMEOUT_S   = 25.0    # s      give up ferrying to a cell (then re-attempt)
# Bank limit for the DURATION of a frequency-sweep run.  The ferry gate requires
# |p| < PLANTID_GATE_RATE_TOL, and the inner roll loop commands
#     roll_rate = ROLL_GAIN * (requested_bank - bank_proxy).
# The proxy goes to 0 at vertical, so the rate command tends to
# ROLL_GAIN * requested_bank, and the gate can only hold when
#     ROLL_GAIN * PLANTID_FERRY_BANK_LIM_DEG < PLANTID_GATE_RATE_TOL.
# A flat constant for the run - not a schedule.  It costs lateral authority,
# which is acceptable because the balloon is injected at pan = 0.  RE-DERIVE
# THIS if ROLL_GAIN or PLANTID_GATE_RATE_TOL change.
PLANTID_FERRY_BANK_LIM_DEG = 3.5

# == PITCH: one gain set at every attitude ====================================
# The body-plane pitch law - P + I + D acting on the in-plane pointing angle
# theta_ip (the balloon's elevation above the path frame's horizontal plane,
# valid straight through vertical) - consumes a body-frame pointing error and
# commands a body pitch rate, and neither changes meaning with attitude, so one
# tuning serves the whole sphere.  The rate command is saturated to the
# available pitch authority (ACRO_PITCH_RATE) but is NOT velocity/acceleration
# shaped.
PITCH_GAIN                     = 4.0
# Integral gain, 1/s, PARALLEL form: deg/s of pitch-rate demand per deg x s of
# accumulated pointing error, added to the rate demand DIRECTLY (not through
# PITCH_GAIN), so its strength is independent of the P gain - matching the
# roll and yaw laws.
PITCH_KI                       = 0.0
# Anti-windup ceiling for the pitch integrator, as a fraction (0-1) of the
# ACHIEVABLE pitch rate (the same authority the law is saturated to): the
# integral's rate ki*state may own at most this share of the rate the airframe
# can deliver.  Independent of PITCH_GAIN.
WINDUP_LIMIT_PITCH             = 0.9
PITCH_DEADBAND_DEG             = 0.25
# Pitch-error derivative (damping) term on the body-plane pitch-RATE command
# (0 = off).  As the flight-path pitch error theta_ip closes, d(theta_ip)/dt is
# negative, so a positive gain subtracts from the nose-up rate command and the
# aircraft arrives at the balloon carrying less rate - the anticipation the pure
# P law lacks, which is what leaves the steep captures overshooting.
# DIMENSIONLESS: deg/s of pitch-rate command per deg/s of error rate, i.e. a
# direct pitch-rate damping (the sibling of the roll axis's PAN_KD, which is in
# seconds only because its output is a bank ANGLE, not a rate).
PITCH_KD                       = 0.0
# 1/s; pitch-error-derivative low-pass, expressed as a rate (bigger = less
# smoothing, 0 = frozen).  rate = 1 / time-constant.  Mirrors PAN_KD_RATE.
PITCH_KD_RATE                  = 10.0

# == REAL-AIRFRAME: Pitch envelope fallbacks (overwritten from PTCH_LIM_*_DEG) ==
PITCH_LIM_UP_DEFAULT = 85.0
PITCH_LIM_DN_DEFAULT = -90.0

# == REAL-AIRFRAME: ACRO rate fallbacks (overwritten from ACRO_*_RATE) ==
ACRO_ROLL_RATE_DEFAULT  = 180.0

# Continuous flight-path alignment the HUD lock counter needs before it freezes
# at the time-to-lock.
ALIGN_FREEZE_HOLD_S  = 1.0      # s

# == YAW: no rudder feedforward ==
# There is no rudder feedforward here: the yaw law commands a RATE, and
# cancelling the propeller's parasitic yaw MOMENT takes a steady DEFLECTION,
# which ArduPlane's own YAW_RATE_I trims out instead.

# == CONTROL OVERRIDE (debug) ==
AILERON_OVERRIDE       = 0.0   # 1 = operator drives the aileron
AILERON_DEFLECTION_PCT = 0.0   # surface %, -100..+100
ELEVATOR_OVERRIDE      = 0.0   # 1 = operator drives the elevator
ELEVATOR_DEFLECTION_PCT = 0.0  # surface %, -100..+100
RUDDER_OVERRIDE        = 0.0   # 1 = operator drives the rudder
RUDDER_DEFLECTION_PCT  = 0.0   # surface %, -100..+100
# -- the turn's pitch rate and the yaw pointing law.  See ControlGains in
# control_laws.py for the derivations; these are the flown defaults.
PITCH_TURN_FF       = 0.0      # scale on the turn's own body pitch rate
# Above this bank a LEVEL coordinated turn needs more than 2 g and the aircraft
# is no longer flying the geometry this term describes, so it is not applied.
# tan() is also climbing steeply by then: 60 deg gives 1.73, 80 gives 5.67.
PITCH_TURN_FF_BANK_LIM_DEG = 60.0
YAW_POINT_KP        = 0.0      # deg/s per deg of path-to-balloon error
YAW_POINT_KI        = 0.0      # 1/s
YAW_POINT_LIM       = 15.0     # deg/s, the rudder's share of the slip budget
YAW_POINT_WINDUP    = 0.2      # fraction of YAW_POINT_LIM the point integrator may own
YAW_POINT_PITCH_SCHED = 0     # 0 = OFF, 1 = ON; the steep share scales the pointing output
# Scale on the yaw the aircraft needs to fly clean, added to the yaw command:
# the turn's own yaw rate and the yaw that turns a roll about the flight path
# instead of the nose.  1 is the geometry, 0 no coordination.
YAW_COORDINATION_FF = 0.0     # dimensionless
# ArduPlane's scaled servo outputs (SRV_Channels get_output_scaled), as the
# pitch trim and the trim map read them.  The elevator's, divided by
# ELEV_OUTPUT_PER_UNIT, is in the elevator units the rate loop and PID_TUNING
# work in, +-ELEV_FULL_TRAVEL at full travel.  The throttle's runs from 0 to
# THROTTLE_OUTPUT_FULL.
ELEV_OUTPUT_PER_UNIT = 100.0
ELEV_FULL_TRAVEL     = 45.0
THROTTLE_OUTPUT_FULL = 100.0
# Pitch trim, in elevator units:
#   elevator = OFFSET + (VREF / EAS)^2 * (LIFT * cos(pitch)
#                                         + ATTITUDE * sin(pitch)
#                                         + THRUST * throttle fraction / EAS2TAS^2)
# with VREF = PITCH_TRIM_VREF_MPS, the throttle fraction ArduPlane's throttle
# output / THROTTLE_OUTPUT_FULL, and 1 / EAS2TAS^2 the air density as a
# fraction of sea level's (the propeller's push at one throttle scales with
# it; see balloon_chaser_fc.lua, PITCH TRIM).  The four gains are the trim map's ('e')
# least-squares fit on the SITL airframe (ARCHITECTURE.md, "Trim map design"),
# over 30 steady cells (-45..+60 deg climb, 15..21 m/s, flown 600-900 m above
# the field): 0.42 elevator units rms, worst 1.00, against 2.5 for a formula
# without airspeed, and cells flown up to 3000 m are predicted to 0.61 rms.
# THRUST is +-2.3 in that fit, not separable from the other terms.  Refitting
# replaces these figures with the new fit's.
# All four 0 is no trim.  VREF only scales the other three; changing it
# without refitting changes the trim.
PITCH_TRIM_VREF_MPS  = 17.0
PITCH_TRIM_OFFSET    = -3.16
PITCH_TRIM_LIFT      = 20.71
PITCH_TRIM_ATTITUDE  = -27.22
PITCH_TRIM_THRUST    = 0.51
ACRO_PITCH_RATE_DEFAULT = 90.0
SCHED_LOOP_RATE_EXPECTED  = 400    # FC main-loop rate. ArduPlane defaults to 50, at
                                # which ATTITUDE/ATTITUDE_QUATERNION CANNOT be
                                # generated at the 50 Hz the cascade reads them.
                                # Reboot-required.
GCS_PID_MASK_EXPECTED     = 7      # FC streams PID_TUNING for roll (bit0=1), pitch (bit1=2)
                                 # and YAW (bit2=4); 1+2+4=7.  Bit assignment is
                                 # ArduPlane's own (@Bitmask 0:Roll,1:Pitch,2:Yaw).
                                 # Yaw is included because the rudder's whole
                                 # output is a rate
                                 # demand handed to ArduPlane's yaw rate
                                 # controller, so without fc_yaw_des/ach there is
                                 # no way to tell "the law asked for the right
                                 # thing and the loop tracked it" from "the law
                                 # asked for the right thing and the rudder never
                                 # moved".  Both look identical from the script
                                 # side, which only ever sees what it sent.
                                 # NOTE: this makes the FC SEND the stream; logging those
                                # messages to CSV columns is a separate change.
# The estimator the script reads.  AHRS_EKF_TYPE selects it (ArduPlane's
# @Values: 3 EKF3, 10 Sim) and EK3_ENABLE makes EKF3 run at all (reboot-required).
# At 10, SITL hands the script the simulator's truth for attitude, velocity and
# wind (AP_AHRS_SIM), so a flight that depends on the estimator says nothing
# about the aircraft, which flies EKF3.
AHRS_EKF_TYPE_EXPECTED    = 3
EK3_ENABLE_EXPECTED       = 1
MAV_STREAM_RATE_EXPECTED  = 50    # Hz.  Per-MAVLink-channel EXTRA1 + RC_CHAN stream-group
                                # rate (MAVx_EXTRA1 / MAVx_RC_CHAN).  These are FC-side caps
                                # on the ATTITUDE/AHRS/SIMSTATE (EXTRA1) and SERVO_OUTPUT_RAW/
                                # RC_CHANNELS (RC_CHAN) groups.  SITL's default on channel 1 is
                                # 4 Hz, which throttled delivered attitude to ~32 Hz against a
                                # 50 Hz request.  Set at startup EVERY session: a ground
                                # station's stream request overwrites and saves it, so it
                                # does not survive between runs (see CLAUDE.md).
RLL_RATE_FLTE_EXPECTED    = 5.0
PTCH_RATE_FLTE_EXPECTED   = 5.0   # symmetric error-term LPF on the PITCH rate PID: flight
                                # logs show 1.6-5 Hz pitch-rate ringing with FLTE=0 while
                                # the filtered roll axis tracked cleanly (corr 0.99); the
                                # startup config checks both (mavlink_link.read_fc_config)
# Pitch-envelope bounds read_fc_config checks at startup (set by hand, never
# written or restored).  MIN must allow the full down-pitch envelope for the steep
# dive; MAX keeps the steep catapult-launch climb from being flattened by a GCS or
# param-file change.
PTCH_LIM_MIN_EXPECTED_DEG = -90.0
PTCH_LIM_MAX_EXPECTED_DEG = 85.0
# Rate-loop settings the FC is EXPECTED to carry: every <AXIS>_RATE_* value the
# plant-ID report ('i') covers.  They are set on the FC by hand (MAVProxy
# `param set`); startup never writes them.  It reads each one back and passes
# the FC holding this set whole, or the Autopilot set (NATIVE_MODE_RATE_PIDS
# over these filters) whole; anything else it lists against this set and waits
# for Enter before flying (mavlink_link._check_rate_pids).  The report suggests values; copying them
# here is the operator's decision.  YAW_RATE_* exist on the FC only with
# YAW_RATE_ENABLE=1; one the FC does not have is skipped.
#
# PTCH_RATE_FLTE and RLL_RATE_FLTE are not listed: read_fc_config checks them
# against PTCH_RATE_FLTE_EXPECTED / RLL_RATE_FLTE_EXPECTED, which differ from the
# report on purpose.
#
# PTCH_RATE_IMAX = 0 holds the pitch rate integrator at zero, so PTCH_RATE_I has
# no effect and is 0 to match.  The rate-loop integrator would otherwise store
# the elevator trim of the attitude it started from and carry it, wrong, into
# the attitude the pull-up ends at; with it at zero the pointing law's own
# integrator holds pitch trim.  RLL_RATE_IMAX and RLL_RATE_I are 0 the same
# way, so any steady aileron the aircraft needs (propeller torque, a sideslip)
# falls to the roll law's pan integrator (LUA/control_roll.lua) and has to fit
# inside its windup limit.
EXPECTED_RATE_PIDS = {
    'PTCH_RATE_FF':   0.0,
    'PTCH_RATE_P':    1.36,
    'PTCH_RATE_I':    0.0,
    'PTCH_RATE_D':    0.0,
    'PTCH_RATE_IMAX': 0.0,
    'PTCH_RATE_FLTT': 3.1831,
    'PTCH_RATE_FLTD': 10.0,
    'PTCH_RATE_SMAX': 150.0,
    'RLL_RATE_FF':    0.0,
    'RLL_RATE_P':     0.2,
    'RLL_RATE_I':     0.0,
    'RLL_RATE_D':     0.0,
    'RLL_RATE_IMAX':  0.0,
    'RLL_RATE_FLTT':  3.1831,
    'RLL_RATE_FLTD':  10.0,
    'RLL_RATE_SMAX':  150.0,
    'YAW_RATE_FF':    0.0,
    'YAW_RATE_P':     2.1483,
    'YAW_RATE_I':     0.0,
    'YAW_RATE_D':     0.0,
    'YAW_RATE_IMAX':  0.0,
    'YAW_RATE_FLTT':  3.1830988,
    'YAW_RATE_FLTE':  0.0,
    'YAW_RATE_FLTD':  10.0,
    'YAW_RATE_SMAX':  150.0,
}

# The stored Acro defaults: every value startup expects the FC to hold -
# EXPECTED_RATE_PIDS and the PTCH/RLL _FLTE targets.  The Gain Tuner's "Default
# Acro Parameters" Write sends the whole set and makes it the operator's values,
# which every switch into ACRO then writes; each row's reset icon writes its own
# entry.  A parameter not listed here (the PTCH2SRV_ / RLL2SRV_ attitude
# settings) has no stored default.
ACRO_DEFAULT_PARAMS = {
    **EXPECTED_RATE_PIDS,
    'PTCH_RATE_FLTE': PTCH_RATE_FLTE_EXPECTED,
    'RLL_RATE_FLTE':  RLL_RATE_FLTE_EXPECTED,
}

# The saved ArduPlane parameters ArduPlane AUTOTUNE tunes (AP_AutoTune::save_gains,
# plus the SMAX it installs when unset): per axis the rate PID and its filters,
# and for pitch and roll the attitude time constant and rate limits (yaw's are
# fixed inside AUTOTUNE, not parameters).  The Gain Tuner reads and writes each
# one on the operator's command, and the flight log records the value the FC
# last confirmed for it.  YAW_RATE_* exist on the FC only with YAW_RATE_ENABLE=1.
ARDUPLANE_AUTOTUNE_PARAMS = (
    'PTCH2SRV_TCONST', 'PTCH2SRV_RMAX_UP', 'PTCH2SRV_RMAX_DN',
    'PTCH_RATE_FF', 'PTCH_RATE_P', 'PTCH_RATE_I', 'PTCH_RATE_D', 'PTCH_RATE_IMAX',
    'PTCH_RATE_FLTT', 'PTCH_RATE_FLTE', 'PTCH_RATE_FLTD', 'PTCH_RATE_SMAX',
    'RLL2SRV_TCONST', 'RLL2SRV_RMAX',
    'RLL_RATE_FF', 'RLL_RATE_P', 'RLL_RATE_I', 'RLL_RATE_D', 'RLL_RATE_IMAX',
    'RLL_RATE_FLTT', 'RLL_RATE_FLTE', 'RLL_RATE_FLTD', 'RLL_RATE_SMAX',
    'YAW_RATE_FF', 'YAW_RATE_P', 'YAW_RATE_I', 'YAW_RATE_D', 'YAW_RATE_IMAX',
    'YAW_RATE_FLTT', 'YAW_RATE_FLTE', 'YAW_RATE_FLTD', 'YAW_RATE_SMAX',
)

# The rate loop ArduPlane's OWN modes fly (LOITER, FBWA, AUTO, ...): AUTOTUNE's
# result for this airframe.  Every mode switch writes the set the new mode flies,
# and only the values the FC does not already hold (MavlinkAutopilotLink._set_mode):
# ACRO, the balloon chaser's mode, gets the operator's values back (the startup read,
# then each Gain Tuner Write or reset, or the Default Acro Parameters Write);
# MANUAL has no rate loop and AUTOTUNE tunes these itself, so neither writes
# anything.  The Gain Tuner's "Default Autopilot Parameters" Write sends this
# whole set.  Startup passes the FC holding this set whole - what a program
# stopped outside ACRO without its exit hand-back leaves - and ACRO then flies
# ACRO_DEFAULT_PARAMS, not these.  The I values follow AUTOTUNE's own
# rule (pitch and yaw max(0.75 P, FF), roll min(P, FF)).  IMAX is ArduPlane's
# default, which AUTOTUNE keeps as long as it lies within 0.4-0.9.
NATIVE_MODE_RATE_PIDS = {
    'PTCH_RATE_FF':   0.153,
    'PTCH_RATE_P':    1.3634,
    'PTCH_RATE_I':    1.0225,
    'PTCH_RATE_D':    0.0013,
    'PTCH_RATE_IMAX': 0.666,
    'RLL_RATE_FF':    0.1157,
    'RLL_RATE_P':     0.0924,
    'RLL_RATE_I':     0.0924,
    'RLL_RATE_D':     0.0,
    'RLL_RATE_IMAX':  0.666,
    'YAW_RATE_FF':    0.1162,
    'YAW_RATE_P':     2.3131,
    'YAW_RATE_I':     1.7348,
    'YAW_RATE_D':     0.0044,
    'YAW_RATE_IMAX':  0.666,
}

# == SHARED: Throttle ==
# User-set throttle (percent) applied to the throttle channel.  This is a STATIC
# operator setting (set via the GUI); the control law never computes or modulates
# throttle to meet a maneuver demand.
THROTTLE_PCT = 85.0

# == SHARED: World-balloon estimator ==
WORLD_EST_ALPHA_NOM = 0.5

# == SHARED: Manual-mode arrow-key step sizes ==
ARROW_LEFT_DEG  = 20.0
ARROW_RIGHT_DEG = 20.0
ARROW_UP_DEG    = 20.0
ARROW_DOWN_DEG  = 20.0

# == SHARED: Point-balloon-chaser test balloon POSITION ==
# WHERE the injected test balloon sits, as a spherical LONGITUDE / LATITUDE
# bearing.  Named for position, not pan/tilt: pan and tilt are reserved in this
# codebase for MOTION or DESIRED motion (steering error, gimbal aim, the balloon's
# direction of travel), never for where something is.
#
#   LONGITUDE  around the aircraft.  0 = straight ahead, +90 = right,
#              180 = dead astern, -90 = left.  Referenced to the HEADING at
#              injection, so 0 means "ahead" on any compass course.
#   LATITUDE   up and down.  +90 = straight up, 0 = level with the horizon,
#              -90 = straight down.  Referenced to the WORLD horizon, so it means
#              the same thing at any aircraft attitude.
#
# NOT WGS84: this is a bearing on a sphere centred on the aircraft, borrowing the
# globe convention by analogy only.  The aircraft's real geographic position is
# link.lat / link.lon and is unrelated.
#
# EVERY (lon, lat) pair is a valid direction over the full sphere (C-04) - no
# restricted region, no budget coupling, nothing clamped or rejected.  This is the
# SAME geometry the real gimbal encoders feed the controller
# (geometry.pan_tilt_to_vector), so the test path matches the flight geometry.
# Astern is reachable either way: lon=180,lat=0 (around the side) or
# lon=0,lat=180 (over the top) - the same direction.
POINT_LON_DEG = 75.0
POINT_LAT_DEG = -70.0
# -- ptest balloon DISTANCE: one rule, one knob (POINT_BALLOON_RANGE) --
# How far along the (POINT_LON_DEG, POINT_LAT_DEG) bearing the balloon sits.  ONE
# rule covers every case, moving or stationary:
#
#     distance = min(POINT_BALLOON_RANGE, ground-intersection distance)
#
# A balloon cannot start underground, so the ground bounds every range.  Then:
#   - a NUMBER puts the balloon there, floating in the air if the ground is
#     farther away.  Repeatable across altitudes: the geometry relative to the
#     aircraft is identical every run.
#   - a number past the ground is capped onto it.
#   - inf is the largest possible range, not an exception.  Descending, the cap
#     lands it on the ground, altitude-adaptive with no number to type.  Level
#     or upward, nothing caps it: a fixed world BEARING the aircraft can never
#     reach, holding a constant-angle lock instead of overflying anything.
#
# A finite result needs GPS lock + above-home; a direction anchor needs neither.
# Editable live in the Gain Tuner ('Distance m'), which accepts the word 'inf'.
POINT_BALLOON_RANGE = float('inf')

# -- ptest MOVING balloon: simple linear motion at a constant speed --
# POINT_BALLOON_MOVE_ENABLE is the master switch.  0 (default) = the balloon is
# STATIONARY, whatever the speed is set to.  1 = the balloon TRANSLATES in a
# straight line at POINT_BALLOON_SPEED m/s along the (POINT_BALLOON_VEL_PAN,
# POINT_BALLOON_VEL_TILT) direction.  It does NOT affect WHERE the balloon starts -
# POINT_BALLOON_RANGE alone decides that, by the rule above.  The travel direction
# keeps the pan/tilt naming because it describes MOTION; it uses the same
# spherical frame as the position bearing - pan is heading-relative, tilt is
# horizon-referenced - and both are frozen at injection, so the path is one
# fixed, repeatable world-frame line.  vel_tilt 0 = level drift (the
# balloon-in-wind case); vel_pan 90 = crossing to the right.
#
# Motion is IGNORED while the range is infinite: nothing at infinity has a
# position to translate.  The switch and speed stay set and apply once a finite
# range is typed; start() logs this rather than doing nothing silently.
POINT_BALLOON_MOVE_ENABLE = 0    # master switch: 0 = stationary (default), 1 = moving
POINT_BALLOON_SPEED = 5.0        # m/s along the velocity bearing (used when switch ON)
POINT_BALLOON_VEL_PAN = 90.0     # deg, motion azimuth rel. heading (90 = cross right)
POINT_BALLOON_VEL_TILT = 0.0     # deg, motion elevation rel. horizon (0 = level)

# == SHARED: rear-hemisphere azimuth tie-break (NOT tunables) ==
# A balloon dead astern has ty ~ 0: left and right are equally correct and the
# geometry offers no answer.  These resolve the fork deterministically instead
# of letting the sign of a 1e-16 float decide.  Below REAR_AZIMUTH_TIE_TY the
# lateral carries no usable sign, so the turn continues the bank already held;
# below REAR_AZIMUTH_TIE_ROLL_RAD of bank there is no bank to continue either
# and the law turns right.  Numerical thresholds, not knobs - do not sweep them.
REAR_AZIMUTH_TIE_TY       = 1e-3      # |ty| under which the lateral sign is noise
REAR_AZIMUTH_TIE_ROLL_RAD = 1e-3      # |roll| under which "continue the bank" is noise

# == SHARED: Gimbal envelope and timing ==
# PHYSICAL servo travel of the camera gimbal.  Consumers: the gimbal command
# path only (adapters clamp what they publish to the pan/tilt joints).  These
# bound where the CAMERA can look, NOT where a test balloon may be placed - the
# injected 'p'-test bearing is a world direction for the oracle and spans the
# full sphere regardless of these (a pan of 180 is a valid rear balloon even
# though the gimbal servo cannot pan that far).
GIMBAL_PAN_MAX   = 90.0
GIMBAL_TILT_MAX  = 90.0
GIMBAL_HZ        = 50
GIMBAL_SLEW_RATE = 120.0   # deg/s - OVERRIDE per real gimbal spec
GIMBAL_DT_MAX_S  = 0.1

# ---- MAVLink streams the controller never reads -----------------------------
# Of 32 inbound message types only 9 are consumed; the unread 23 carry 77% of
# the traffic (931 of 1201 msg/s).  Every stream not set individually pins at
# the same 42.3 Hz against a requested 50 - unrelated streams sharing one
# ceiling means a shared send budget, so the unread traffic is why
# ATTITUDE_QUATERNION cannot reach 50 Hz, and that is the noise floor under
# every plant-ID and sweep measurement.
#
# Disabled with SET_MESSAGE_INTERVAL(-1) per message ID.  The per-message
# interval overrides the group rate: VFR_HUD and GLOBAL_POSITION_INT run at
# 8.5 and 4.2 Hz while their groupmates run at 42.3, because they are set
# individually.
#
# Per MESSAGE, not per GROUP: the groups bundle wanted with unwanted (EXTRA1
# carries ATTITUDE and SIMSTATE, RC_CHANNELS carries SERVO_OUTPUT_RAW and
# RC_CHANNELS), so zeroing a group takes a needed message with it.
#
# Request/response traffic must NOT be listed - PARAM_VALUE, COMMAND_ACK,
# STATUSTEXT and the MISSION_* exchange are replies to our own requests, and
# disabling them breaks parameter reads and mission upload.  Set to () to
# disable this entirely.
MAVLINK_DISABLE_STREAMS = (
    'RAW_IMU', 'SCALED_IMU2', 'SCALED_IMU3',            # inertial raw
    'SCALED_PRESSURE', 'SCALED_PRESSURE2',              # baro
    'GPS_RAW_INT',                                      # GLOBAL_POSITION_INT is the one used
    'AHRS', 'AHRS2', 'EKF_STATUS_REPORT', 'SIMSTATE',   # estimator internals
    'LOCAL_POSITION_NED', 'POSITION_TARGET_GLOBAL_INT',
    'NAV_CONTROLLER_OUTPUT', 'MISSION_CURRENT',         # nav status (mission upload uses
                                                        # MISSION_* replies, not this)
    'WIND', 'TERRAIN_REPORT', 'VIBRATION',
    'BATTERY_STATUS', 'POWER_STATUS', 'MEMINFO',        # housekeeping
    'SYSTEM_TIME',
    # AOA_SSA is deliberately NOT listed: it carries ArduPilot's wind-corrected
    # sideslip, which is now read and logged.  Listing it achieved nothing
    # anyway - it rides in EXTRA1 with ATTITUDE, and ArduPilot will not honour a
    # per-message disable for a member of a group being actively requested.
    'RC_CHANNELS',                                      # we SEND RC_CHANNELS_OVERRIDE; the
                                                        # inbound RC_CHANNELS is never read
)

# == SHARED: link watchdogs (companion <-> FC) ==
# The FC stands down when no NEW GUIDE frame (a seq it has not seen) has
# arrived for this long: it releases nav_scripting, so the aircraft stops flying
# the last command instead of chasing it with frozen inputs.  The companion
# sends GUIDE every control tick, so this is many missed frames in a row, and
# it is shorter than ArduPilot's own nav_scripting timeout, which never fires
# while the script keeps calling.
GUIDE_STALE_MS = 500
# The companion halts when no STATE frame has arrived from the FC for this long.
# The FC sends STATE every tick, gated or not, so silence this long means the
# script stopped or the link broke - and every lua_* value would otherwise keep
# its last reading as if it were live.
FC_LINK_TIMEOUT_S = 1.0

# == SHARED: the mode codes GUIDE carries (controller.MODE_CODE) ==
GUIDE_MODE_LOITER = 0
GUIDE_MODE_STRAIGHT = 1
GUIDE_MODE_MANUAL = 2
GUIDE_MODE_BALLOON_CHASER = 3
GUIDE_MODE_PTEST = 4
GUIDE_MODE_AUTOTUNE = 5

# == SHARED: the Curve Planner (companion) and the Curve Flyer (FC) ==
# The Curve Planner flies every candidate plan through a model of the Curve
# Flyer and of the steering law's plan mode, so every number either reads is
# defined once here and read by both: a plan made against one law and flown by
# another is a plan for an aircraft that does not exist.
#
# Curve Flyer on (1) or off (0).  On, a fresh chase holds its rates at zero
# until a PLAN arrives and then the steering law flies its turn; off, the law
# flies every chase from its first tick as without a PLAN.  It runs p and x
# chases only.
CURVE_FLYER_ENABLE = 0
# Transfer Control When Done on (1) or off (0).  On, the steering law flies the
# chase as without a PLAN from the moment the turn is complete; off, the Curve
# Flyer aims the law at the line the flight path had at that moment, until the
# chase ends.
CF_TRANSFER_WHEN_DONE = 1
# Bank the Curve Flyer gives the law to hold a line with, deg per deg the line
# lies to the side of the path, within the bank limit: wings level on the
# line, banked toward it off it, so a drift off the line across the wings is
# turned back rather than kept.
CF_HOLD_BANK_PER_DEG = 3.0
# CLAUDE.md "The rules of a chase": the tilt never comes back by more
# than this from the furthest it has reached in its direction of travel.
TILT_BACK_TRAVEL_DEG = 2.0
# The turn is complete when the flight path points within this of the balloon
# (CLAUDE.md "The rules of a chase"); the heading stays within it after.
CURVE_TURN_DONE_DEG = 5.0
# The Curve Planner's rule decision (curve_planner.rule_binds): a balloon
# "below" or "above" lies within RULE_PLANE_DEG, round the path, of the
# vertical plane through it, and a turn goes through straight down or up when
# its great circle passes within RULE_POLE_DEG of it before reaching the
# balloon.  The planner holds a chase to the rules of a chase (CLAUDE.md) only
# where these say rule 1 (below) or rule 2 (above) binds, and the FC test
# judges flown chases by them.
RULE_PLANE_DEG = 15.0
RULE_POLE_DEG = 10.0
# Bank angles in a PLAN's schedule, evenly spaced in progress through the turn
# from 0 (the start) to 1 (pointing at the balloon).
CURVE_N_KNOTS = 5
# Within this of straight up or down the bank against the horizon and the
# horizontal across the path are unmeasurable: Lift Vector Pointing's roll to
# a PLAN's bank fades to zero over it, it weighs no sideways turn and holds no
# climb floor there, and the heading rule weighs a heading change by the
# path's distance from vertical.
CURVE_POLE_FADE_DEG = 10.0
# How long a fresh chase holds for its PLAN before the steering law flies it
# without one, s (the Gain Tuner's Plan Wait, cf_plan_wait_s).
CF_PLAN_WAIT_S = 20.0
# The Approach Angle (the Gain Tuner's cf_approach_deg): the dive, deg below
# the horizon, the turn of a chase ends on.  A balloon that starts below the
# horizon but less far below than this is flown with a RUN-IN: the turn lines
# the path up level toward the balloon's compass direction, the run-in holds
# that line until the balloon sits far enough below, and a dive then brings
# the path onto the balloon at this angle.  A balloon this far below or
# further, at or above the horizon, or where rule 2 binds is flown as
# without it - and at 0 no balloon is shallower, so every chase is.
CF_APPROACH_DEG = 0.0
# The Curve Flyer's states, as STATE2's cf_state carries them: off; a fresh
# chase waiting for its PLAN; flying the turn; the turn complete; no PLAN in
# time; the turn complete and its line held (Transfer Control When Done off);
# a run-in holding its line; the dive after it.  Each value keeps its meaning
# in every log, so the numbers are not packed.
CF_STATE_OFF = 0
CF_STATE_WAIT = 1
CF_STATE_FLY = 2
CF_STATE_DONE = 3
CF_STATE_TIMEOUT = 4
CF_STATE_HOLD = 6
CF_STATE_RUNIN = 7
CF_STATE_DIVE = 8

# == THE STEERING LAW, AND LIFT VECTOR POINTING (LUA/path_attitude.lua) ==
# Which law steers every mode, one switch for every mode at once (C-03):
#   LAW_CASCADE  the cascade: bank-to-turn and roll-to-plane, with the yaw law
#                beside them;
#   LAW_LVP      Lift Vector Pointing, version B (the rudder held at zero): the
#                roll aims the lift line at the force line, round the path;
#   LAW_LVP_P    Lift Vector Pointing P: the roll aims the lift line at the
#                force the wing must make - the force line less the
#                propeller's push across the path - round the path;
#   LAW_LVP_N    Lift Vector Pointing N: the roll aims it at the force the
#                body's own forces must make, round the nose, and goes out as
#                the body roll rate.
# P and N fly Lift Vector Pointing's plan mode and are otherwise its law.
LAW_CASCADE = 0
LAW_LVP = 1
LAW_LVP_P = 2
LAW_LVP_N = 3
STEERING_LAW = LAW_CASCADE
# Standard gravity, m/s^2: the value ArduPilot's GRAVITY_MSS carries.
GRAVITY_MPS2 = 9.80665
# Guidance.  The flight path is asked to turn toward the balloon at
# ATT_K_PATH rad/s per rad off it, capped at ATT_PATH_RATE_MAX_DPS, with the
# angle off read from where the path will point ATT_LEAD_S from now (0 is the
# path as read).
ATT_K_PATH = 3.0
ATT_PATH_RATE_MAX_DPS = 70.0
ATT_LEAD_S = 0.2
# The roll about the path: rad/s per rad the lift line is off the force line.
ATT_K_ROLL = 5.0
# The bank limit (bank_lim_deg) holds in every chase, on the bank proxy as
# the cascade measures it (the wing tip's angle below the horizon), the band
# widened to a starting proxy beyond it: a turn that wants more bank aims at
# the limit itself.  The turn's sideways part (along the horizontal across
# the path) weighs ATT_K_SIDE times its share, so the heading turns while the
# path is shallow; 1 is no weighting.
ATT_K_SIDE = 3.0
# The integrators on the lift line's error (roll) and on the path's error in
# the lift plane (pitch), 1/s, running only within ATT_I_ZONE_DEG of the
# balloon, each held within its ACRO rate.
ATT_ROLL_KI = 0.5
ATT_PITCH_KI = 2.0
ATT_I_ZONE_DEG = 3.0
# The propeller's push per unit mass, as Lift Vector Pointing P and N
# estimate it:
#     T = (ATT_PROP_SLOPE / EAS2TAS^2) * tau * (ATT_PROP_SPEED_MPS * tau - u)
# tau the throttle output as a fraction of THROTTLE_OUTPUT_FULL, u the true
# airspeed along the nose.  At one throttle the propeller turns at about one
# speed, so its pitch speed and its slope both follow the throttle, and its
# push the air density, 1 / EAS2TAS^2 of sea level's.  ATT_PROP_SPEED_MPS is
# the true airspeed at which full throttle pushes nothing, m/s;
# ATT_PROP_SLOPE the push per unit mass, m/s^2, that each m/s below it adds at
# full throttle in sea-level air, 1/s.  The defaults are test_plant.lua's
# propeller line, fitted to SITL's climb and dive speeds at 85 % throttle;
# the aircraft needs its own.
ATT_PROP_SPEED_MPS = 21.141
ATT_PROP_SLOPE = 3.4713

# == SIM-ONLY / HUD-COSMETIC: camera horizontal FOV (not in control path) ==
GIMBAL_FOV_DEG = 60.0

# == SIM-ONLY: HSV thresholds for yellow chevron detection ==
BALLOON_HSV_LOWER = np.array([15, 150, 100])
BALLOON_HSV_UPPER = np.array([45, 255, 255])
BALLOON_MIN_AREA  = 100


