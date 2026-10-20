# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""
The control law's gains.

The law itself runs on the flight controller (LUA/control_roll.lua,
LUA/control_pitch.lua, LUA/cascade.lua).  ControlGains is the operator's set -
what the Gain Tuner edits and SET_GAIN carries to the FC - and PitchGains is
the pitch stack's view of it.

ROLL is one law feeding back the gravity-referenced bank proxy
asin(sin(roll)cos(pitch)), which has no pole at vertical.  PITCH is ONE body-plane law acting
on the in-plane pointing angle theta_ip rather than absolute Euler pitch, so one
gain set serves every attitude and stays valid straight through vertical.
"""

# Standing rules for running the balloon chaser law and its tuning ('t' sweep, 'k'
# measurement).  Present-tense constraints; name the constants, not their live
# values.
#
# - The pan integrator's authority ceiling is bank_lim * WINDUP_LIMIT_PAN; pan_ki
#   cancels out of it, so raising pan_ki buys NO integral authority.
#   WINDUP_LIMIT_PAN must clear the standing bank the balloon chaser turn needs or the
#   P term covers the shortfall as a standing error.
#
# - GRID_TILT_CAP_DEG is a plain constant the sweep obeys as-is - a setting, not
#   something the program adjusts.  Steep rows that tie at the no-align penalty
#   teach nothing; the report's CELLS THAT NEVER CONVERGED section lists them
#   rather than hiding it.
#
# - The 't' sweep measures a noise floor over the whole grid
#   (AUTOTUNE_NOISE_FLOOR_REPS) before touching any parameter and raises the
#   switch margin to the measured spread; it is skipped on RESUME.  The cost
#   scores settled error (COST_W_SETTLED), not just closest approach.  A
#   parameter's optimum can sit outside its bracket - ENTRY_BRACKETS overrides
#   BRACKET_FACTORS per entry, and windup uses a wider net.

from __future__ import annotations
from collections import namedtuple
from dataclasses import dataclass

from . import constants as C

# Resolved PITCH stack gains (see ControlGains.pitch_gains).  The body-plane
# pitch law - P + I + D on the in-plane pointing angle theta_ip - is
# attitude-independent.  The rate command is saturated to the available pitch
# authority but is NOT velocity/acceleration shaped.
#   gain             deg/s of pitch rate per deg of pointing error (P)
#   ki               integral gain, 1/s: deg/s of pitch rate per deg x s of
#                    accumulated pointing error.  PARALLEL form - its term is
#                    added to the rate demand DIRECTLY, not through gain, so the
#                    integral's strength is independent of the P gain (matching
#                    the roll and yaw laws, which are already parallel).
#   windup_limit     state clamp = (windup_limit * acro_pitch_rate)/ki, so the
#                    integral's rate ki*state <= windup_limit of the ACHIEVABLE
#                    pitch rate (ki cancels; independent of the P gain)
#   deadband_deg     |theta_ip| below which the integrator freezes (noise guard)
#   kd               pitch-error DERIVATIVE (damping) gain, dimensionless
#   kd_rate          derivative low-pass rate (1/s); bigger = less smoothing
PitchGains = namedtuple('PitchGains', (
    'gain', 'ki', 'windup_limit', 'deadband_deg', 'kd', 'kd_rate'))


# -----------------------------------------------------------------------------
# Gains: all runtime-tunable cascade parameters in one place.
# -----------------------------------------------------------------------------
@dataclass
class ControlGains:
    # Outer roll steering (linear P + I + FF -> bank -> rate)
    pan_p_gain: float
    pan_ki: float
    windup_limit_pan: float
    bank_lim_deg: float

    # Pitch stack - ONE gain set at every attitude.  The body-plane pitch law
    # (P + I + D on the in-plane pointing angle theta_ip) is attitude-
    # independent; ControlGains.pitch_gains resolves the set each tick so tuner
    # edits take effect live.  The set is: gain, ki, windup_limit, deadband, kd,
    # kd_rate.  The rate command is saturated to the available
    # pitch authority (ACRO x pitch suppression) but is NOT velocity/acceleration
    # shaped; the integrator's windup ceiling is a fraction (windup_limit) of
    # that same achievable pitch rate.
    pitch_gain: float
    pitch_ki: float
    windup_limit_pitch: float
    pitch_deadband_deg: float
    # Pitch-error derivative (damping) term, dimensionless: deg/s of pitch-rate
    # command per deg/s of error rate.  d(theta_ip)/dt is low-pass filtered
    # (pitch_kd_rate) and added to the raw P+I rate demand.
    pitch_kd: float
    pitch_kd_rate: float   # 1/s; derivative low-pass rate (bigger = less smoothing)

    # Inner roll gain (bank error -> roll-rate demand)
    roll_gain: float

    # AIRSPEED COMMAND (throttle loop).  Its only actuator is the throttle;
    # see compute_throttle_command in LUA/control_pitch.lua.  Never read by the
    # roll or pitch laws.
    airspeed_cmd_enable: int = 0
    airspeed_cmd_mps: float = 15.0
    airspeed_cmd_kp: float = 5.0
    airspeed_cmd_ki: float = 5.0
    airspeed_cmd_min_valid: float = 2.0

    # Lateral-error derivative (lead) term on the bank command (default
    # 0 = off).  Subtracts kd * d(pan_err)/dt from the commanded bank so the
    # aircraft rolls its wings level BEFORE the pointing error reaches zero,
    # arriving at ty=0 already wings-level instead of still banked/turning -
    # which suppresses the turn-overshoot limit cycle.  The derivative is low-pass
    # filtered (pan_kd_rate) for noise robustness.
    pan_kd: float = 0.0
    pan_kd_rate: float = 10.0   # 1/s; derivative low-pass rate (bigger = less smoothing)


    # ---- PITCH: the turn's own rotation ------------------------------------
    # A banked aeroplane pitches in BODY axes purely because it is turning.  It
    # is not a disturbance and not an error - it is what turning is - and a
    # pointing law that does not know about it spends its whole time chasing
    # rotation it could have predicted: asked for zero in a bank, the rate loop
    # spends elevator resisting the turn's own pitch rate.
    #
    # q = r * tan(bank), where r is the coordinated-turn YAW rate the FC
    # computes (yaw_ff_coord_dps).  Both come from the same heading rate
    # resolved into body axes, so the pitch half is the yaw half times tan(bank)
    # exactly - no second airspeed division, no second attitude read, and it
    # inherits that term's airspeed floor and validity for free.
    #
    # 1.0 is the geometry.  It is a feedforward, so it costs no phase and cannot
    # destabilise the loop; a wrong value biases the aim and nothing else.
    pitch_turn_ff: float = 1.0            # dimensionless

    # ---- YAW: POINTING -----------------------------------------------------
    # The rudder steering, in parallel with the wing, on the SAME error the roll
    # axis closes: pan_err, which is path - balloon.  Bank-to-turn is the sword;
    # this is the scalpel.
    #
    # TWO ACTUATORS ON ONE ERROR IS FINE.  Two INTEGRATORS on one error is the
    # thing to be careful about: the equilibrium is unique (both zero) but the
    # SPLIT between them is not determined by anything.  It is accepted here
    # deliberately, because the split settles by saturation rather than by
    # chance - the rudder's integral fills its own small bound first and the roll
    # axis takes the remainder - and because roll's integrator is itself clipped
    # and cannot always finish the job.  yaw_point_lim is what makes that
    # allocation definite rather than a race.
    #
    # WHY THE AUTHORITY IS SMALL AND WHY THAT IS NOT A TUNING PROBLEM.  The
    # rudder moves the flight path only through sideslip, and it runs out of
    # sideslip at AIRFRAME_SIDESLIP_MAX_DEG.  Its fast effect - rotating the nose
    # - cancels out of path - balloon entirely, since the nose appears in both
    # halves of that difference.  So this term is a bounded assist, never the
    # primary steering, and asking it for more only buys drag and slip.
    #
    # SIGN.  pan_err > 0 is the path lying RIGHT of the balloon, which wants a
    # nose-LEFT (negative) rate: the nose leads, sideslip builds, the side force
    # curves the path left after it.
    #
    # Both gains default to 0: the axis comes up disarmed and is dialled in from
    # the tuner, like every other new term here.
    yaw_point_kp: float = 0.0             # deg/s per deg
    yaw_point_ki: float = 0.0             # 1/s

    # THE RUDDER'S SHARE OF THE AIRFRAME'S SIDESLIP BUDGET, deg/s: the bound
    # on the pointing OUTPUT.  0 is no pointing authority, not "no ceiling".
    # Past what AIRFRAME_SIDESLIP_MAX_DEG allows, more buys drag, not turn.
    yaw_point_lim: float = 15.0           # deg/s, bounds pointing OUTPUT




    # ---- CONTROL OVERRIDE (bench / plant ID) ------------------------------
    # One switch per surface.  A surface whose switch is 0 is not written at
    # all and stays under ArduPlane.  A surface whose switch is 1 takes its
    # deflection percentage straight to the servo channel, with the law and
    # every bound skipped: a plant-ID step a limit quietly reshapes identifies
    # the limit, not the airframe.  Each channel is still held inside its own
    # SERVOn_MIN/MAX, past which the servo cannot go anyway.
    #
    # Overriding a surface bypasses ArduPlane's stabilisation on that axis.
    # The aileron is clean - its own channel - so overriding it leaves pitch
    # and yaw alone.  The tail is not: a V-tail has no elevator channel and no
    # rudder channel, both are mixed onto the two ruddervators, so overriding
    # EITHER takes both channels and pitch stabilisation goes with them.  The
    # un-overridden tail axis is then HELD at the value it had when the switch
    # was flipped, recovered by inverting the mix from the live ruddervator
    # PWMs on that tick.
    #
    # Throttle is a different channel and keeps working.  The servo override
    # carries a 200 ms timeout refreshed every tick, so if the script stops the
    # surfaces return to ArduPlane on their own.
    #
    # OFF for any scored run.  cs_flags bit 7 records when ANY of them was on,
    # because gains are not logged and an override deflection would otherwise
    # be indistinguishable in the CSV from one the law produced.
    aileron_override: float = 0.0          # 0 = law, 1 = operator
    aileron_deflection_pct: float = 0.0    # surface %, -100..+100
    elevator_override: float = 0.0         # 0 = law, 1 = operator
    elevator_deflection_pct: float = 0.0   # surface %, -100..+100
    rudder_override: float = 0.0           # 0 = law, 1 = operator
    rudder_deflection_pct: float = 0.0     # surface %, -100..+100

    # 0 = OFF (default), 1 = ON.  Multiplies the pointing output by the roll
    # blend's steep share (the path's angle to the horizon between
    # plane_steep_start_deg and plane_steep_full_deg): none on a shallow path,
    # all of it on a steep one, where banking has little gravity across the
    # path to work with.
    yaw_point_pitch_sched: int = 0

    # Anti-windup on the pointing integrator, as a fraction of the term's output
    # limit that the integrator alone may own.  1.0 lets the integrator fill the
    # whole limit; below that it reserves headroom for the Kp part.  The output
    # limit itself is yaw_point_lim.
    yaw_point_windup: float = 0.2         # fraction of yaw_point_lim

    # ---- PITCH TRIM ---------------------------------------------------------
    # The elevator the airframe needs to hold its nose still, in ArduPlane's
    # elevator units (45 = full travel, the units PID_TUNING logs):
    #   offset + (PITCH_TRIM_VREF_MPS / EAS)^2 * (lift * cos(pitch)
    #            + attitude * sin(pitch) + thrust * throttle fraction)
    # balloon_chaser_fc.lua turns it into the pitch-rate request that makes the FC's
    # rate loop output exactly that elevator, so the pointing law does not have
    # to sit off the balloon to produce it.  All four 0 is no trim.
    pitch_trim_offset: float = 0.0        # elevator units, at any speed
    pitch_trim_lift: float = 0.0          # elevator units x cos(pitch) at VREF
    pitch_trim_attitude: float = 0.0      # elevator units x sin(pitch) at VREF
    pitch_trim_thrust: float = 0.0        # elevator units x throttle at VREF, sea-level air

    # ---- ROLL-TO-PLANE ------------------------------------------------------
    # The 12/6 hold band (deg past the wingtip line), roll-to-plane's own gain
    # (deg/s per deg), its feedforward on the balloon's drift round the clock
    # (1 = match the drift) and that drift's low-pass rate (1/s).
    plane_side_hold_deg: float = C.PLANE_SIDE_HOLD_DEG
    plane_gain: float = C.PLANE_GAIN
    plane_ff_gain: float = C.PLANE_FF_GAIN
    plane_ff_lpf_rate: float = C.PLANE_FF_LPF_RATE
    # The blend: roll-to-plane's share of the roll command from the flight
    # path's angle (deg above or below the horizon), bank-to-turn's the rest;
    # roll-to-plane's integrator gain (1/s) and its anti-windup ceiling as a
    # fraction (0-1) of the ACRO roll rate.
    plane_steep_start_deg: float = C.PLANE_STEEP_START_DEG
    plane_steep_full_deg: float = C.PLANE_STEEP_FULL_DEG
    plane_ki: float = C.PLANE_KI
    plane_windup: float = C.PLANE_WINDUP
    # Roll-to-plane's derivative gain on the clock error's rate (deg/s per
    # deg/s) and that rate's low-pass rate (1/s).
    plane_kd: float = C.PLANE_KD
    plane_kd_rate: float = C.PLANE_KD_RATE

    # ---- YAW: COORDINATION ---------------------------------------------------
    # The yaw command is the WHOLE body yaw rate ArduPlane's rate loop holds, so
    # the yaw the aircraft needs to fly clean has to be asked for: the turn's
    # own yaw rate, g*sin(roll)*cos(pitch)/V, and the yaw that turns a roll
    # about the flight path instead of the nose, p*tan(alpha) with p the
    # commanded roll rate and alpha the path's angle below the nose.  Without
    # it the rate loop holds the yaw at the pointing term's value, fighting
    # every turn and every roll with rudder - on the V-tail, deflection the
    # elevator shares.  1.0 is the geometry; a feedforward, so it costs no phase
    # and a wrong value only slips the aircraft.  Any value but 0 runs the yaw
    # rate loop.
    yaw_coordination_ff: float = C.YAW_COORDINATION_FF   # dimensionless

    # ---- THE CURVE FLYER -----------------------------------------------------
    # On (1), a fresh chase holds until the Curve Planner's PLAN arrives and the
    # steering law flies its turn in its plan mode; off (0), the law flies every
    # chase as without a PLAN.  The plan wait is how long a fresh chase holds
    # for its PLAN before the law flies it without one, s.  Transfer Control
    # When Done on (1) hands the chase back to the law without the PLAN when
    # the turn is complete; off (0) the Curve Flyer aims the law at the line
    # the path had then.  The Tilt Back Travel and the Approach Angle are the
    # Curve Planner's: how far the tilt may come back from the furthest it has
    # reached (CLAUDE.md "The rules of a chase"), deg, and the dive below
    # the horizon a chase's turn ends on, flying a run-in first where the
    # balloon starts shallower (CF_APPROACH_DEG), deg.
    curve_flyer_enable: int = C.CURVE_FLYER_ENABLE
    tilt_back_travel_deg: float = C.TILT_BACK_TRAVEL_DEG
    cf_transfer_when_done: int = C.CF_TRANSFER_WHEN_DONE
    cf_plan_wait_s: float = C.CF_PLAN_WAIT_S
    cf_approach_deg: float = C.CF_APPROACH_DEG

    # ---- THE STEERING LAW ------------------------------------------------------
    # Which law steers every mode: 0 the cascade above, 1 Lift Vector Pointing
    # (LUA/path_attitude.lua), version B with the rudder held at zero.
    steering_law: int = C.STEERING_LAW

    # ---- LIFT VECTOR POINTING --------------------------------------------------
    # Guidance: the path's turn rate per rad off the balloon (1/s), its cap
    # (deg/s), and how far ahead the angle off is read (s, 0 = the path as read).
    att_k_path: float = C.ATT_K_PATH
    att_path_rate_max_dps: float = C.ATT_PATH_RATE_MAX_DPS
    att_lead_s: float = C.ATT_LEAD_S
    # The roll about the path per rad of lift-line error (1/s).
    att_k_roll: float = C.ATT_K_ROLL
    # The weight on the turn's sideways part.
    att_k_side: float = C.ATT_K_SIDE
    # The final approach's integrators (1/s) and the zone they run in (deg off
    # the balloon).
    att_roll_ki: float = C.ATT_ROLL_KI
    att_pitch_ki: float = C.ATT_PITCH_KI
    att_i_zone_deg: float = C.ATT_I_ZONE_DEG

    # @property, not a plain method.  Every caller reads it as an attribute
    # (self._gains.pitch_gains, no parens), so without it they get the
    # FUNCTION OBJECT and the failure surfaces one frame later as
    # "'function' object has no attribute 'ki'".
    @property
    def pitch_gains(self) -> "PitchGains":
        """Return the body-plane pitch stack gain set - one set at every
        attitude.  Read live each tick so tuner edits take effect."""
        return PitchGains(
            self.pitch_gain, self.pitch_ki,
            self.windup_limit_pitch, self.pitch_deadband_deg,
            self.pitch_kd, self.pitch_kd_rate)
