# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
"""
Wire protocol: companion <-> FC.

ONE definition here generates the Python packer, the Lua parser, and the field
documentation.  Hand-writing both ends is how struct layouts drift, and a
drifted layout does not fail loudly - it silently reinterprets a float as an
int and flies on.

────────────────────────────────────────────────────────────────────────────
DESIGN DECISIONS BEHIND THIS LAYOUT
────────────────────────────────────────────────────────────────────────────

FRAMEWORK-AGNOSTIC.  Fixed-layout little-endian binary over a dedicated serial
port (SERIALx_PROTOCOL = 28), read with serial:find_serial().  Nothing here
knows about Python, MAVLink or ROS 2 - a future ROS 2 node emits the same bytes
and the Lua side does not change by a line.

EXCITATION IS COMMANDED, NOT STREAMED.  The obvious design streams the current
injection value every tick.  That is wrong: it puts the companion's scheduling
jitter INTO the excitation waveform, and the whole reason identification moves
to the FC is to remove that jitter.  So EXCITE carries a DESCRIPTION of a
waveform - type, amplitude, frequency, duration - and the script generates it
locally against millis().  The excitation is then as clean as the FC's clock.

GAINS ARE PUSHED LIVE.  controller_autotune_sweep and the plant-ID / freq-ID
measurements work by mutating gains hundreds of times per run.  Without SET_GAIN
they cannot drive the Lua path at all.  Indices are generated from ControlGains
field order, so both ends agree by construction.

NO COMPANION WATCHDOG.  Absent in the simulation build: the FC keeps flying the
last GUIDE it received.  Recovery is a mode change - the script claims control
only in ACRO, so any other mode takes the surfaces back.  balloon_chaser_fc.lua's
header states what must be added before a hardware flight.

CRC ON EVERY FRAME.  A serial line with a loose connector delivers plausible
garbage, and a garbage det_norm is a garbage rate command.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Any

from .constants import CURVE_N_KNOTS

PROTO_VERSION = 4
MAGIC = 0xA5C3

# message ids
MSG_GUIDE = 0x01      # companion -> FC, every tick
MSG_SET_GAIN = 0x02   # companion -> FC, on change
MSG_EXCITE = 0x03     # companion -> FC, starts a waveform
MSG_PLAN = 0x04       # companion -> FC, the Curve Planner's turn for this chase
MSG_STATE = 0x81      # FC -> companion, mirrored state
MSG_STATE2 = 0x82     # FC -> companion, the rest of the mirror, same tick

# excitation waveform types (generated FC-side against millis())
EXC_NONE = 0
EXC_DOUBLET = 1       # plant-ID: +A for T/2, -A for T/2
EXC_CHIRP = 2         # frequency sweep f0 -> f1 over duration
EXC_SINE = 3          # fixed-frequency dwell
EXC_STEP = 4
GATE_NAMES = {
    0: "commanding",
    1: "no GUIDE frames received",
    2: "no gimbal angles - no ArduPilot mount and none sent over the wire",
    3: "no attitude from ahrs:get_quaternion()",
    4: "estimator has no balloon yet",
    5: "not tracking",
    6: "does not own the control loop - nav_scripting_enable refused",
    7: "GUIDE stale - no new frame within GUIDE_STALE_MS, control released",
    8: "held back after a fault until GUIDE says tracking has stopped",
}

# What took control away mid-chase: STATE flags bits 5-7, latched on the FC
# until GUIDE says tracking has stopped.  The companion halts on any of them.
# Codes 2-4 are the gates whose input disappeared; 1 is a Lua error.
FAULT_SCRIPT_ERROR = 1
FAULT_NAMES = {
    FAULT_SCRIPT_ERROR: "a Lua error in the steering script",
    2: "the gimbal angles stopped arriving",
    3: "no attitude from ahrs:get_quaternion()",
    4: "the balloon estimate (or the oracle bearing) was lost",
}

EXC_NAMES = {EXC_NONE: "none", EXC_DOUBLET: "doublet", EXC_CHIRP: "chirp",
             EXC_SINE: "sine", EXC_STEP: "step"}

# excitation axis
AXIS_ROLL = 0         # adds to omega_x  (freqid) or replaces it (plantid)
AXIS_PITCH = 1


@dataclass(frozen=True)
class Field:
    name: str
    fmt: str          # struct char
    scale: float = 1.0
    note: str = ""

    @property
    def size(self) -> int:
        return struct.calcsize("<" + self.fmt)


# ---------------------------------------------------------------------------
# MSG_GUIDE - companion -> FC, one per CV frame
# ---------------------------------------------------------------------------
# SENSOR readings (the detection offset, the gimbal encoder angles, the oracle
# bearing) and OPERATOR INTENT - never a control-law output.  Configuration
# travels in SET_GAIN (gains) and EXCITE (an excitation's description).  The
# attitude is read on the FC, in the tick that uses it.
GUIDE = (
    Field("seq",            "H", 1.0,     "wraps at 65535; staleness detector"),
    Field("flags",          "B", 1.0,     "bit0 det_valid, bit1 tracking, bit2 oracle_active, bit3 straight_level"),
    Field("mode",           "B", 1.0,     "0 loiter 1 straight 2 manual 3 balloon chaser 4 ptest 5 autotune"),
    Field("det_norm_x",     "h", 10000.0, "normalised frame offset, +-1 at edge"),
    Field("det_norm_y",     "h", 10000.0, ""),
    Field("throttle_pct",   "h", 100.0,   "operator throttle; streamed every tick"),
    Field("straight_yaw",   "h", 100.0,   "deg, _control_straight_level setpoint"),
    Field("manual_yaw",     "h", 100.0,   "deg, _control_manual setpoint"),
    Field("manual_pitch",   "h", 100.0,   "deg, _control_manual setpoint"),
    # GIMBAL ENCODERS.
    #
    # C-02 wants the body-frame balloon direction from encoders, and on a vehicle
    # with an ArduPilot mount the script would read them itself with
    # mount:get_attitude_euler().  In THIS project the gimbal is simulated in
    # Gazebo and driven by the COMPANION over gz-transport, so the flight
    # controller has no mount driver and no encoders to read - the script's
    # mount call returns nil and it stalls before the cascade.
    #
    # So the companion sends what it measures.  0.01 deg resolution over +-327
    # deg, which is far finer than the gimbal resolves.
    Field("gimbal_pan",     "h", 100.0,   "deg, measured pan encoder"),
    Field("gimbal_tilt",    "h", 100.0,   "deg, measured tilt encoder"),
    Field("gimbal_flags",   "B", 1.0,     "bit0 readback live"),
    Field("_pad2",          "B", 1.0,     "alignment"),
    # ORACLE BALLOON DIRECTION (world NED unit vector).
    #
    # _balloon_world_dir bypasses the estimator entirely when the oracle is
    # active - ptest injects an exact bearing rather than detecting anything, so
    # there is no measurement to fold and the estimator NEVER becomes valid.
    # Sending only the oracle_active FLAG left the FC knowing a balloon existed
    # but not where, so it stalled on noest for the whole run.
    #
    # A unit vector at 1/10000 resolution is ~0.006 deg - far finer than the
    # steering resolves.
    Field("oracle_x",       "h", 10000.0, "world NED unit vector, x"),
    Field("oracle_y",       "h", 10000.0, "world NED unit vector, y"),
    Field("oracle_z",       "h", 10000.0, "world NED unit vector, z"),
    # AGE OF THE GIMBAL ANGLES, milliseconds, at the moment this frame was packed.
    #
    # The FC rotates a body-frame direction built from these angles into the
    # world using an attitude read on-chip THIS tick.  The two are sampled at
    # different times, so the pairing error is (this age) x (body rate) - the
    # exact error C-02 was written to prevent, reintroduced because this airframe
    # has no ArduPilot mount and the "encoders" therefore cross the wire.
    #
    # Sending the age does not fix it.  It makes it MEASURABLE, which is the
    # precondition for deciding whether it needs fixing.
    # SIMULATED-TIME STAMP of the gimbal pose, from the Pose_V header, in ms.
    #
    # A stamp, not a computed age - the age is then differenced ON THE FC
    # against its own millis(), so it comes out in SIMULATED milliseconds
    # natively and the real-time factor drops out of the calculation entirely.
    # A companion-computed age is necessarily WALL time (the companion has no
    # other clock) and would have to be multiplied by an RTF nobody has measured.
    #
    # CAVEAT, and it is why the raw age is kept alongside: the two clocks share
    # a RATE but not an EPOCH.  SITL accumulates its clock from deltas -
    # SIM_JSON.cpp:514, `time_now_us += deltat * 1.0e6` - so it starts from
    # ArduPilot boot, while the Gazebo stamp starts from world load.  The FC
    # therefore estimates the constant offset as the running MINIMUM of
    # (millis() - stamp), which is the freshest sample seen, and reports age
    # relative to that.  Until enough samples have arrived the estimate is
    # optimistic, so gimbal_age_ms remains the conservative fallback.
    Field("gimbal_stamp_ms", "I", 1.0,    "gz sim-time stamp of the gimbal pose"),
    Field("gimbal_age_ms",  "H", 1.0,     "WALL ms since read (fallback; needs RTF)"),
)

# ---------------------------------------------------------------------------
# MSG_SET_GAIN - companion -> FC
# ---------------------------------------------------------------------------
SET_GAIN = (
    Field("index", "H", 1.0, "position in ControlGains field order"),
    Field("value", "f", 1.0, "IEEE754 single; gains do not need double"),
)

# ---------------------------------------------------------------------------
# MSG_EXCITE - companion -> FC
# ---------------------------------------------------------------------------
# Starts a locally-generated waveform on `axis`.  `replace` distinguishes the
# two identification modes:
#   replace=0 -> ADDS to that axis's rate command   (loop stays closed,
#                excitation is external to it)
#   replace=1 -> REPLACES that axis's rate command  (open loop on that axis,
#                the other keeps flying the aim)
EXCITE = (
    Field("kind",      "B", 1.0,   "EXC_* waveform type"),
    Field("axis",      "B", 1.0,   "0 roll, 1 pitch"),
    Field("replace",   "B", 1.0,   "0 = add to command, 1 = replace it"),
    Field("_pad",      "B", 1.0,   "alignment"),
    Field("amp_dps",   "f", 1.0,   "amplitude, deg/s"),
    Field("f0_hz",     "f", 1.0,   "start frequency (chirp/sine)"),
    Field("f1_hz",     "f", 1.0,   "end frequency (chirp)"),
    Field("dur_ms",    "I", 1.0,   "total duration; 0 cancels any running excitation"),
)

# ---------------------------------------------------------------------------
# MSG_PLAN - companion -> FC
# ---------------------------------------------------------------------------
# The Curve Planner's turn for this chase, run by the Curve Flyer and flown by
# the steering law it was planned for (CLAUDE.md "Lua steering offload"):
# bank angles, knot k at progress (k-1)/(CURVE_N_KNOTS-1) through the turn,
# where progress is 1 - (angle off the balloon) / eps0_deg, in that
# law's own measure of the bank - the bank proxy under the cascade
# (steering_law 0), the bank round the path under Lift Vector Pointing (1)
# and its versions P (2) and N (3), which fly its plan mode.  The FC takes a
# PLAN only for the law it has selected.  The turn aims at
# the balloon, or for a chase with a run-in (CF_APPROACH_DEG) the run-in line:
# runin_climb_deg above the horizon toward the balloon's compass direction,
# held after the turn until the balloon sits runin_dep_deg below the horizon,
# when a dive turns the path onto the balloon.  The two run-in fields are NaN
# for a chase without one.  The one control-law output the companion sends.
# It is re-sent until STATE2's cf_plan_id echoes plan_id.
PLAN = (
    Field("plan_id",   "H", 1.0, "goes up by one per plan; STATE2 echoes it once accepted"),
    Field("law",       "B", 1.0, "the steering law it was planned for: 0 the cascade, 1 Lift Vector Pointing"),
    Field("_pad",      "B", 1.0, "alignment"),
    Field("eps0_deg",  "f", 1.0, "angle off the balloon the plan's progress counts from"),
) + tuple(Field(f"knot_{k + 1}_deg", "f", 1.0,
                f"scheduled bank at progress {k}/{CURVE_N_KNOTS - 1}, + right wing down")
          for k in range(CURVE_N_KNOTS)) + (
    Field("runin_climb_deg", "f", 1.0, "the run-in line's angle above the horizon, deg; NaN for no run-in"),
    Field("runin_dep_deg",   "f", 1.0, "the balloon's angle below the horizon that ends the run-in, deg"),
)

# ---------------------------------------------------------------------------
# MSG_STATE - FC -> companion
# ---------------------------------------------------------------------------
# Mirrors the script's internals so the existing HUD, CSV logger, autotune and
# plant-ID keep working against a Lua-flown aircraft.  Without this the
# companion is blind and every measurement tool the project owns stops.
#
# Floats, not fixed-point: this is diagnostics, the link has headroom, and
# quantising the very signals used to fit a plant model would be perverse.
STATE = (
    Field("seq",             "H", 1.0, "echoes the GUIDE seq that produced it"),
    # bit 3 says the law ran this tick (gates 0, 6 and 8).  With it clear, every
    # law-internal float below is NaN and cs_flags is 0: the companion writes
    # them blank rather than as a measurement.
    Field("flags",           "B", 1.0, "bit0 nav_scripting_active, bit1 est_valid, bit2 excite_running, bit3 law_ran, "
                                       "bit4 a frame from the companion was rejected (another protocol build), "
                                       "bits5-7 what dropped control mid-chase (latched; 0 = none, see FAULT_NAMES)"),
    # PRE-CLAMP rate demand, captured in cascade.lua immediately before
    # clip(omega, -acro_rate, +acro_rate).  Paired with omega_*_cmd_dps below,
    # the two say whether the ACRO envelope is cutting the law off: equal means
    # the law got what it asked for, different means it is clipped and no gain
    # change will help.  Without this the clamp is invisible - the command
    # simply sits at the limit looking like a deliberate choice.
    Field("omega_x_presat_dps", "f", 1.0, ""),
    Field("omega_y_presat_dps", "f", 1.0, ""),
    Field("omega_x_cmd_dps", "f", 1.0, "ACTED"),
    Field("omega_y_cmd_dps", "f", 1.0, "ACTED"),
    Field("throttle_out",    "f", 1.0, "ACTED"),
    Field("requested_roll",  "f", 1.0, "rad"),
    Field("requested_pitch", "f", 1.0, "rad"),
    Field("steering_pan",    "f", 1.0, "deg"),
    Field("steering_tilt",   "f", 1.0, "deg"),
    Field("theta_ip_deg",    "f", 1.0, ""),
    Field("world_est_trust", "f", 1.0, ""),
    Field("pan_integrator",  "f", 1.0, ""),
    Field("pitch_integrator","f", 1.0, ""),
    Field("vel_marker_deg",  "f", 1.0, ""),
    Field("speed_scaler",    "f", 1.0, "what the FC actually applied; NaN with no airspeed estimate"),
    Field("excite_value",    "f", 1.0, "current excitation output, for the fit"),
    Field("dt_ms",           "H", 1.0, "measured script tick, NOT nominal"),
    # WHY the script is not commanding.  Sent even when gated - a STATE frame
    # that only appears on success cannot explain a failure, which is exactly
    # the hole that made three rounds of guessing necessary.
    #   0 commanding  1 no GUIDE  2 no gimbal  3 no attitude
    #   4 no estimate 5 not tracking  6 does not own the control loop
    Field("gate",            "B", 1.0, "0 = commanding; see protocol.GATE_NAMES"),
    Field("cs_flags",        "B", 1.0, "bit0 roll_sat, 1 pan_i_frozen, 2 pan_i_clamped, "
                                       "3 pitch_i_frozen, 4 pitch_i_clamped, "
                                       "5 a PLAN set the bank (the cascade's bits 0-2 mean nothing), "
                                       "6 Lift Vector Pointing or its version P or N flew (bits 0-4 mean nothing), "
                                       "7 surface_override"),
    # CASCADE INTERNALS: the ones that matter for tuning - the roll term
    # breakdown, the bank proxy the law actually feeds back, and the
    # pre-saturation command.  The bank proxy is Lift Vector Pointing's too:
    # the bank its band holds.
    Field("p_term_roll",     "f", 1.0, "roll P contribution, deg"),
    Field("d_term_roll",     "f", 1.0, "roll D contribution, deg"),
    Field("d_term_pitch",    "f", 1.0, "pitch D contribution, deg/s"),
    Field("bank_proxy_deg",  "f", 1.0, "asin(R[1][2]): the wing tip's angle below the horizon, the bank Bank Lim holds"),
    Field("roll_cmd_presat", "f", 1.0, "roll command before the acro clamp"),
    Field("roll_err_deg",    "f", 1.0, "roll setpoint minus feedback"),
    Field("upright",         "f", 1.0, "cos(roll)cos(pitch); negative = inverted"),
    # SENSOR FRESHNESS.
    #
    # Reading a sensor at 50 Hz does not make it 50 Hz.  airspeed_EAS() is often
    # 10-20 Hz on real hardware, and EKF velocity is GPS-corrected at ~5 Hz -
    # yet the AoA and velocity-marker estimators integrate them EVERY tick
    # with ki*dt.  A signal that only changes every 10th tick is then filtered
    # against nine repeats and one jump, which is not the same filter response
    # as the design assumed.
    #
    # These count how often each value ACTUALLY CHANGED in the last second, so
    # the effective rate is measured rather than assumed.
    Field("hz_quat",         "B", 1.0, "attitude changes/sec (<=50)"),
    Field("hz_gyro",         "B", 1.0, "gyro changes/sec"),
    Field("hz_vel",          "B", 1.0, "EKF velocity changes/sec"),
    Field("hz_asp",          "B", 1.0, "airspeed changes/sec"),
    # ABSOLUTE FC clock (millis, wrapped to 32 bits).
    #
    # dt_ms alone CANNOT measure the real-time factor: it is the FC scheduler's
    # own inter-invocation interval, ~20 ms by construction, and the companion
    # tick is its own 20 ms sleep target.  Dividing them compares two self-paced
    # loops and yields ~1 whatever the simulator is doing - a stable number that
    # measures nothing.
    #
    # RTF needs one INTERVAL measured on two clocks:
    #     RTF = d(fc_millis) / d(companion monotonic)
    Field("fc_millis",       "I", 1.0, "absolute FC clock, ms (simulated time)"),
    # The C-02 pairing error input, in SIMULATED ms, computed on the FC.
    # Multiply by body rate for the error in degrees.
    Field("gimbal_sim_age_ms", "H", 1.0, "gimbal-angle age in SIMULATED ms"),
    Field("_pad5",           "H", 1.0, "alignment"),

    # ---- YAW / COORDINATION INSTRUMENTATION -------------------------------
    # Appended, so every offset above is unchanged.
    #
    # SERIAL BUDGET: see STATE2, which states it for both frames together.
    # A payload's length travels in ONE byte, so a frame holds at most 255
    # payload bytes; STATE is close to that, and further fields go in STATE2.
    #
    # accel_* are ahrs:get_accel(), the PRIMARY IMU in BODY axes - specific
    # force, not acceleration, so level flight reads about (0, 0, -9.81).
    # accel_y is the turn-coordination signal: it is what the ball in a
    # turn-and-slip indicator measures, it is zero in a coordinated turn, and
    # unlike a velocity-derived sideslip it does NOT move with wind. That last
    # property is the whole reason it is here - sideslip_deg is computed from
    # GROUND velocity, so a 3 m/s crosswind at 15 m/s fabricates 11.5 deg of
    # sideslip that is not there, which is larger than the entire quantity a
    # coordination loop would be regulating.
    #
    # ArduPilot exposes an accel bias (AP_AHRS.h: "should be subtracted from
    # get_accel()") but bindings.desc has no binding for it, so this is the raw
    # primary reading: do not assume a standing offset here is zero.  NaN when
    # the IMU read fails.
    Field("accel_y_mps2",    "f", 1.0, "body lateral specific force"),
    # The sibling axis, kept so accel_y can be sanity-checked rather than
    # trusted: in a coordinated turn this should read about -9.81/cos(bank),
    # and it gives the TRUE load factor instead of one inferred from bank.
    Field("accel_z_mps2",    "f", 1.0, "body normal specific force"),

    # Coordinated turn: r = g*sin(roll)*cos(pitch) / V, the turning half of the
    # yaw law's coordination; the pitch axis reads it too, as the turn's body
    # pitch rate r*tan(bank) (pitch_turn_dps below).  NaN with no airspeed
    # estimate.
    Field("yaw_ff_coord_dps", "f", 1.0, "coordinated-turn yaw rate, deg/s"),
    # The rolling half of the coordination: r = p*tan(alpha), p the roll rate
    # this tick commands, alpha the path's angle below the nose in the body
    # pitch plane (zero at or below FLIGHT_PATH_MIN_SPEED_MPS).  Unscaled: the
    # yaw command carries yaw_coordination_ff times it.  NaN on a tick the FC
    # did not command.
    Field("yaw_ff_ari_dps",  "f", 1.0, "rolling yaw p*tan(alpha), deg/s"),

    # ---- YAW LAW OUTPUT ---------------------------------------------------
    # The integrator SEPARATELY from the command, never just the sum.  In a sum a
    # term doing nothing and a term cancelling another are indistinguishable, and
    # nothing else in the log can reconstruct an integrator.
    #
    # pan_err_deg is the ROLL axis's error - the gap between where the aircraft
    # is GOING and where it should go - and the rudder's pointing term reads it
    # too.  One signal serving two axes, so it must stay in the frame its name
    # claims.
    Field("pan_err_deg",      "f", 1.0, "path minus balloon, deg; HUD sign, positive = path right"),
    Field("yaw_rate_cmd_dps", "f", 1.0, "commanded body yaw rate, deg/s, post-clamp"),
    # The pointing term SEPARATELY from the command.  In a sum a term doing
    # nothing and a term cancelling another are indistinguishable, and nothing
    # else in the log can reconstruct the integrator behind it.
    Field("yaw_point_dps",    "f", 1.0, "pointing term, deg/s, post-clamp"),
    # The turn's own body pitch rate, RAW, before pitch_turn_ff scales it and
    # before it is added to omega_y.  Logged because omega_y_cmd_dps alone cannot
    # say how much of the pitch demand was aiming and how much was the turn.
    Field("pitch_turn_dps",   "f", 1.0, "turn-induced body pitch rate, deg/s"),

    # The LATERAL flight-path marker, deg, positive when the flight path lies
    # to the RIGHT of the nose; vel_marker_deg is its vertical twin.  The laws
    # read the path frame, not the markers; zero in both means no usable
    # velocity, where the nose stands in for the path.
    Field("lat_marker_deg",   "f", 1.0, "lateral velocity marker, deg"),

    # The pitch trim: the elevator the trim law asked for (ArduPlane units, 45 =
    # full travel) and the rate request that carries it.  The request is part of
    # omega_y_cmd_dps but is never flown as a rotation, so without it here the
    # rate loop's desired-vs-achieved gap reads as a fault.
    Field("pitch_trim_elev",  "f", 1.0, "trim elevator asked for, ArduPlane units"),
    Field("pitch_trim_dps",   "f", 1.0, "pitch-rate request carrying the trim, deg/s"),

    # The airspeed-hold throttle loop, which runs on the FC (its only actuator
    # is the throttle).  throttle_out above is its output.
    Field("asp_flags",        "B", 1.0, "bit0 loop active, bit1 output clipped"),
    Field("asp_err_mps",      "f", 1.0, "airspeed setpoint minus airspeed, m/s"),
    Field("asp_i_term",       "f", 1.0, "integrator, throttle percent"),

    # The coordination the yaw command carried: yaw_coordination_ff times the
    # turning and rolling yaw above, before the ACRO_YAW_RATE ceiling clips the
    # sum with the pointing term.  0 under a surface override.
    Field("yaw_coordination_dps", "f", 1.0, "coordination term, deg/s, pre-ceiling"),

    # The lift term the pitch trim read: cos(pitch) under the cascade; under
    # Lift Vector Pointing its lift load (att_n_lift, below), or the load of the
    # rates flown on a tick the Curve Flyer owns the chase.  The trim map fits
    # against what the trim read.
    Field("pitch_trim_lift_term", "f", 1.0, "the pitch trim's lift regressor this tick"),

    # Lift Vector Pointing (LUA/path_attitude.lua); NaN on a tick it did not
    # run.  Its other internals are in STATE2.
    Field("att_roll_i_dps",    "f", 1.0, "roll integrator, deg/s of roll about the path"),
    Field("att_pitch_i_dps",   "f", 1.0, "pitch integrator, deg/s of the path's turn"),
    Field("att_wx_dps",        "f", 1.0, "the law's roll: about the path, or about the nose under N without a PLAN"),
    Field("att_wy_dps",        "f", 1.0, "path-frame rate: the path's turn in the lift plane"),
    Field("att_wz_dps",        "f", 1.0, "path-frame rate: the frame's turn about z' under gravity"),
    Field("att_bank_deg",      "f", 1.0, "bank the law rolls from: the canopy's angle from up, + right wing down; "
                                         "round the path, or round the nose under N without a PLAN"),
    Field("att_band_lo_deg",   "f", 1.0, "lowest bank proxy the chase allows: the bank limit, or the starting proxy beyond it"),
    Field("att_band_hi_deg",   "f", 1.0, "highest bank proxy the chase allows: the bank limit, or the starting proxy beyond it"),
    Field("att_drift_dps",     "f", 1.0, "the bank's drift the roll cancels while the room holds it, "
                                         "and under a PLAN; NaN otherwise"),
    Field("att_theta_now_deg", "f", 1.0, "the path's angle off the balloon as read, no lead"),
    Field("att_p_dps",         "f", 1.0, "body roll rate the law asks for, before the ACRO clip"),
    Field("att_q_dps",         "f", 1.0, "body pitch rate the law asks for, before the ACRO clip"),
    Field("att_w_f",           "f", 1.0, "the roll's weight under a PLAN, its fade near vertical, 0..1; "
                                         "NaN without one"),
    Field("att_n_lift",        "f", 1.0, "load the law asks of the wing, g"),
    Field("accel_x_mps2",      "f", 1.0, "body longitudinal specific force"),
    Field("att_room_lo_deg",   "f", 1.0, "farthest roll left from the bank now before the bank proxy leaves the band "
                                         "or the aircraft is upside down; -180 unlimited"),
    Field("att_room_hi_deg",   "f", 1.0, "farthest roll right from the bank now before the bank proxy leaves the band "
                                         "or the aircraft is upside down; 180 unlimited"),
)

# ---------------------------------------------------------------------------
# MSG_STATE2 - FC -> companion, sent right after STATE on the same tick
# ---------------------------------------------------------------------------
# The rest of what a flight is analysed from, which does not fit in STATE's 255
# payload bytes.  `seq` echoes the same GUIDE seq as STATE, so a row whose two
# seq columns differ paired frames from different ticks.
#
# Three groups:
#   * The FC's own sensors, read every tick whatever the gates decide: the
#     attitude, rotation rate, velocity and airspeed the law read, on the FC's
#     clock.  The companion's MAVLink copies of the same quantities arrive late
#     and in steps (CLAUDE.md: raw rate telemetry is hold-and-jump), so these
#     are the ones to analyse the law against.
#   * Law internals the log needs.  NaN on a tick the law did not run - written
#     blank in the CSV, never a stale value from an earlier tick.
#   * What ArduPlane asked each surface for (SRV_Channels:get_output_scaled,
#     before the V-tail mix; +-4500 = full travel, throttle in percent) and the
#     servo signal after the mix (get_output_pwm; 0 = no channel carries it).
#
# Units are the ones the FC holds: gyro in rad/s, velocity in m/s, angles in
# degrees where the Lua computes them in degrees.
#
# SERIAL BUDGET.  STATE and STATE2 go out on a raw serial port every tick, not
# through MAVLink.  frame = 4 header + payload + 2 CRC, at 10 bits/byte, 50 Hz,
# on the 921600 link (port:begin in balloon_chaser_fc.lua, pyserial in main.py):
#     STATE 240 B + STATE2 242 B payload -> 494 B per tick -> 247 kbps -> 27% of the line   <- here
# test_link_contract recomputes these figures and fails when they drift.
STATE2 = (
    Field("seq",               "H", 1.0, "echoes the GUIDE seq, as STATE's does"),
    Field("flags",             "B", 1.0, "bit0 coord_valid, bit1 accel_valid, bit2 lat_marker_valid"),
    Field("fc_mode",           "B", 1.0, "vehicle:get_mode() this tick"),
    # The FC's sensors, this tick.
    Field("q_w",               "f", 1.0, "ahrs:get_quaternion(), world->body"),
    Field("q_x",               "f", 1.0, ""),
    Field("q_y",               "f", 1.0, ""),
    Field("q_z",               "f", 1.0, ""),
    Field("gyro_x_rads",       "f", 1.0, "ahrs:get_gyro(), body roll rate"),
    Field("gyro_y_rads",       "f", 1.0, "body pitch rate"),
    Field("gyro_z_rads",       "f", 1.0, "body yaw rate"),
    Field("vel_n_mps",         "f", 1.0, "ahrs:get_velocity_NED(); NaN when absent"),
    Field("vel_e_mps",         "f", 1.0, ""),
    Field("vel_d_mps",         "f", 1.0, ""),
    Field("eas_mps",           "f", 1.0, "ahrs:airspeed_EAS(); NaN when absent"),
    Field("eas2tas",           "f", 1.0, "ahrs:get_EAS2TAS(), true airspeed per EAS"),
    # Law internals; NaN on a tick the law did not run.
    Field("balloon_body_x",        "f", 1.0, "balloon direction the law flew, body frame"),
    Field("balloon_body_y",        "f", 1.0, ""),
    Field("balloon_body_z",        "f", 1.0, ""),
    Field("los_elev_deg",      "f", 1.0, "line-of-sight elevation, before the AoA trim"),
    Field("bank_rate_cmd_dps", "f", 1.0, "bank rate the roll rate buys"),
    Field("bank_rate_coupling_dps", "f", 1.0, "bank rate the yaw rate drives"),
    Field("sideslip_deg",      "f", 1.0, "from FC velocity; NaN below 1 m/s"),
    Field("aoa_deg",           "f", 1.0, "atan2(v_body_z, v_body_x); NaN below 1 m/s"),
    Field("yaw_point_i_dps",   "f", 1.0, "yaw pointing integrator"),
    Field("plane_off_nose_deg", "f", 1.0, "balloon's angle off the nose, 0..180"),
    Field("plane_err_deg",     "f", 1.0, "roll-to-plane: deg still to roll to 12 or 6 round the nose"),
    Field("plane_axis_up",     "f", 1.0, "world-vertical part of the unit turn axis, 0..1"),
    Field("plane_drift_dps",   "f", 1.0, "balloon's filtered drift round the clock, deg/s"),
    Field("plane_ff_dps",      "f", 1.0, "roll-to-plane feedforward, deg/s of roll"),
    Field("plane_err_rate_dps", "f", 1.0, "roll-to-plane: the clock error's filtered rate, deg/s"),
    Field("plane_d_dps",       "f", 1.0, "roll-to-plane derivative term, deg/s of roll"),
    Field("plane_path_share",  "f", 1.0, "its share of the roll command, from the flight-path angle; "
                                         "bank-to-turn has the rest, 0..1"),
    Field("plane_i_dps",       "f", 1.0, "roll-to-plane integrator, deg/s of roll"),
    Field("plane_i_frozen",    "f", 1.0, "1 = integrator held, the command saturated its way"),
    Field("plane_path_deg",    "f", 1.0, "angle the path share read, deg, + up"),
    Field("plane_path_from_vel", "f", 1.0, "1 = the velocity's, 0 = the nose's pitch stood in"),
    Field("plane_side",        "f", 1.0, "roll-to-plane's end: 1 = 12 o'clock, -1 = 6"),
    # The surfaces.
    Field("out_aileron",       "f", 1.0, "get_output_scaled(k_aileron)"),
    Field("out_elevator",      "f", 1.0, "get_output_scaled(k_elevator)"),
    Field("out_rudder",        "f", 1.0, "get_output_scaled(k_rudder)"),
    Field("out_throttle",      "f", 1.0, "get_output_scaled(k_throttle), percent"),
    Field("pwm_aileron",       "H", 1.0, "get_output_pwm(k_aileron)"),
    Field("pwm_vtail_l",       "H", 1.0, "get_output_pwm(k_vtail_left)"),
    Field("pwm_vtail_r",       "H", 1.0, "get_output_pwm(k_vtail_right)"),
    # The Curve Flyer.  cf_state is always meaningful; what it hands the law
    # is NaN on a tick the law flew no PLAN.
    Field("cf_state",          "B", 1.0, "Curve Flyer: 0 off, 1 waiting for the PLAN, 2 flying the turn, "
                                         "3 turn complete, 4 no PLAN in time, "
                                         "6 turn complete and its line held, 7 run-in, 8 dive"),
    Field("_cf_pad",           "B", 1.0, "alignment"),
    Field("cf_plan_id",        "H", 1.0, "plan_id of the PLAN accepted this chase, 0 for none"),
    Field("cf_progress",       "f", 1.0, "progress through the turn (state 2) or the dive (8), 0-1"),
    Field("cf_eps_deg",        "f", 1.0, "flight path's angle off what the law is aimed at: the line in "
                                         "states 6 and 7 and in 2 with a run-in, else the balloon"),
    Field("cf_bank_ref_deg",   "f", 1.0, "the bank the law is given, in its own measure: the schedule's "
                                         "in state 2, CF_HOLD_BANK_PER_DEG per deg across the wings in 6-8"),
    Field("cf_hold_n",         "f", 1.0, "the line the law is aimed at (states 6, 7, and 2 with a run-in), "
                                         "world unit vector, north"),
    Field("cf_hold_e",         "f", 1.0, "east"),
    Field("cf_hold_d",         "f", 1.0, "down"),
    # The flight path's own turn since the previous tick, toward the canopy and
    # the right wing (track.lua), measured from the velocity, never the gyro.
    # Law internals, NaN on a tick the law did not run or the turn is unknown.
    Field("path_turn_up_dps",    "f", 1.0, "flight path's turn rate toward the canopy"),
    Field("path_turn_right_dps", "f", 1.0, "flight path's turn rate toward the right wing"),
    # Lift Vector Pointing (LUA/path_attitude.lua); NaN on a tick it did not
    # run.  Its other internals are in STATE.
    Field("att_theta_deg",     "f", 1.0, "path's angle off the balloon, read att_lead_s ahead"),
    Field("att_u_y",           "f", 1.0, "turn direction across the path, toward the right wing (unit)"),
    Field("att_u_z",           "f", 1.0, "turn direction across the path, toward the belly (unit)"),
    Field("att_omega_dps",     "f", 1.0, "turn rate the path is asked for"),
    Field("att_f_y_mps2",      "f", 1.0, "force line: push across the path toward the right wing, m/s^2"),
    Field("att_f_z_mps2",      "f", 1.0, "force line: push across the path toward the belly, m/s^2"),
    Field("att_mu_deg",        "f", 1.0, "roll still to go for the lift side's end of the lift line to "
                                         "face the aim (att_aim_*); under a PLAN, to the PLAN's bank"),
    Field("att_side",          "f", 1.0, "lift side: 1 canopy (pull), -1 belly (push); NaN under a PLAN"),
    Field("att_thrust_mps2",   "f", 1.0, "the propeller's push per unit mass P and N estimate, m/s^2; "
                                         "NaN under law 1 or with no airspeed estimate"),
    Field("att_aim_y_mps2",    "f", 1.0, "what the roll aims the lift line at, toward the right wing, m/s^2: "
                                         "the force line (law 1), less the propeller's push across the path "
                                         "(P), in the path frame; on the body axes (N)"),
    Field("att_aim_z_mps2",    "f", 1.0, "its part toward the belly, m/s^2; att_n_lift is minus it over g"),
)

MESSAGES = {
    MSG_GUIDE: ("GUIDE", GUIDE),
    MSG_SET_GAIN: ("SET_GAIN", SET_GAIN),
    MSG_EXCITE: ("EXCITE", EXCITE),
    MSG_PLAN: ("PLAN", PLAN),
    MSG_STATE: ("STATE", STATE),
    MSG_STATE2: ("STATE2", STATE2),
}

# frame: magic(H) msgid(B) len(B) payload... crc(H)
HEADER_FMT = "<HBB"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
CRC_SIZE = 2


def payload_fmt(fields: tuple[Field, ...]) -> str:
    return "<" + "".join(f.fmt for f in fields)


def payload_size(fields: tuple[Field, ...]) -> int:
    return struct.calcsize(payload_fmt(fields))


# ---------------------------------------------------------------------------
# CRC-16/CCITT-FALSE.  Chosen because it is trivial to write in Lua without a
# table (256-entry tables cost heap that SCR_HEAP_SIZE would rather keep).
# ---------------------------------------------------------------------------

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def pack(msgid: int, values: dict[str, Any]) -> bytes:
    name, fields = MESSAGES[msgid]
    raw = []
    for f in fields:
        v = values.get(f.name, 0)
        if f.fmt in "bhiq" or f.fmt in "BHIQ":
            raw.append(int(round(float(v) * f.scale)))
        else:
            raw.append(float(v))
    body = struct.pack(payload_fmt(fields), *raw)
    head = struct.pack(HEADER_FMT, MAGIC, msgid, len(body))
    return head + body + struct.pack("<H", crc16(head + body))


def unpack(frame: bytes) -> tuple[int, dict[str, Any]]:
    magic, msgid, ln = struct.unpack(HEADER_FMT, frame[:HEADER_SIZE])
    if magic != MAGIC:
        raise ValueError(f"bad magic {magic:#06x}")
    body = frame[HEADER_SIZE:HEADER_SIZE + ln]
    got = struct.unpack("<H", frame[HEADER_SIZE + ln:HEADER_SIZE + ln + 2])[0]
    want = crc16(frame[:HEADER_SIZE + ln])
    if got != want:
        raise ValueError(f"crc {got:#06x} != {want:#06x}")
    name, fields = MESSAGES[msgid]
    vals = struct.unpack(payload_fmt(fields), body)
    out = {}
    for f, v in zip(fields, vals):
        out[f.name] = (v / f.scale) if f.scale != 1.0 else v
    return msgid, out


def gain_index_map(gains_cls) -> dict[str, int]:
    """ControlGains field order IS the SET_GAIN index space.  Generated, so the
    two ends cannot disagree - and so adding a gain upstream shifts both."""
    from dataclasses import fields as dc_fields
    return {f.name: i for i, f in enumerate(dc_fields(gains_cls))}


# ---------------------------------------------------------------------------
# Lua code generation
# ---------------------------------------------------------------------------

_LUA_READER = {
    "B": ("rd_u8",  1), "b": ("rd_i8",  1),
    "H": ("rd_u16", 2), "h": ("rd_i16", 2),
    "I": ("rd_u32", 4), "i": ("rd_i32", 4),
    "f": ("rd_f32", 4),
}


def generate_lua(gains_cls=None) -> str:
    L = []
    A = L.append
    A("-- Auckland University Electrical Engineering Robotics Association"
      " - Undergrad Balloon Popper Competition.")
    A("--")
    A("-- GENERATED by Companion/protocol.py -- DO NOT EDIT.")
    A("-- Regenerate instead; hand edits will be silently overwritten and the")
    A("-- two ends of the wire will disagree without failing loudly.")
    A("")
    A("local M = {}")
    A("")
    A(f"M.PROTO_VERSION = {PROTO_VERSION}")
    A(f"M.MAGIC = {MAGIC}")
    for k, v in (("MSG_GUIDE", MSG_GUIDE), ("MSG_SET_GAIN", MSG_SET_GAIN),
                 ("MSG_EXCITE", MSG_EXCITE), ("MSG_PLAN", MSG_PLAN),
                 ("MSG_STATE", MSG_STATE),
                 ("MSG_STATE2", MSG_STATE2)):
        A(f"M.{k} = {v}")
    A("")
    A("-- Payload size of every message, by id.  A frame whose length or id does")
    A("-- not match was written by another build of this protocol: it is rejected,")
    A("-- never decoded, because decoding it reads the fields at the wrong offsets.")
    A("M.PAYLOAD_SIZE = {")
    for mid in sorted(MESSAGES):
        A(f"  [{mid}] = {payload_size(MESSAGES[mid][1])},   -- {MESSAGES[mid][0]}")
    A("}")
    A("")
    A(f"M.FAULT_SCRIPT_ERROR = {FAULT_SCRIPT_ERROR}")
    for k, v in (("EXC_NONE", EXC_NONE), ("EXC_DOUBLET", EXC_DOUBLET),
                 ("EXC_CHIRP", EXC_CHIRP), ("EXC_SINE", EXC_SINE),
                 ("EXC_STEP", EXC_STEP)):
        A(f"M.{k} = {v}")
    A("")
    A("-- CRC-16/CCITT-FALSE.  Table-free on purpose: a 256-entry table is heap")
    A("-- that SCR_HEAP_SIZE would rather spend on the steering core.")
    A("function M.crc16(s, from, to)")
    A("  local crc = 0xFFFF")
    A("  for i = from, to do")
    A("    crc = crc ~ (string.byte(s, i) << 8)")
    A("    for _ = 1, 8 do")
    A("      if crc & 0x8000 ~= 0 then crc = ((crc << 1) ~ 0x1021) & 0xFFFF")
    A("      else crc = (crc << 1) & 0xFFFF end")
    A("    end")
    A("  end")
    A("  return crc")
    A("end")
    A("")
    A("-- little-endian readers; `p` is a 1-based byte offset")
    A("local function rd_u8(s, p)  return string.byte(s, p), p + 1 end")
    A("local function rd_i8(s, p)")
    A("  local v = string.byte(s, p); if v >= 0x80 then v = v - 0x100 end")
    A("  return v, p + 1")
    A("end")
    A("local function rd_u16(s, p)")
    A("  return string.byte(s, p) | (string.byte(s, p + 1) << 8), p + 2")
    A("end")
    A("local function rd_i16(s, p)")
    A("  local v = string.byte(s, p) | (string.byte(s, p + 1) << 8)")
    A("  if v >= 0x8000 then v = v - 0x10000 end")
    A("  return v, p + 2")
    A("end")
    A("local function rd_u32(s, p)")
    A("  return (string.byte(s, p) | (string.byte(s, p+1) << 8)")
    A("       | (string.byte(s, p+2) << 16) | (string.byte(s, p+3) << 24)), p + 4")
    A("end")
    A("local function rd_i32(s, p)")
    A("  local v; v, p = rd_u32(s, p)")
    A("  if v >= 0x80000000 then v = v - 0x100000000 end")
    A("  return v, p")
    A("end")
    A("-- IEEE754 single decode.  string.unpack('<f') exists in 5.3+ and is")
    A("-- both faster and exact; the manual path is kept only as documentation")
    A("-- of the layout.")
    A("local function rd_f32(s, p) return (string.unpack('<f', s, p)), p + 4 end")
    A("")

    for msgid, (name, fields) in sorted(MESSAGES.items()):
        A(f"-- {name}: {payload_size(fields)} payload bytes")
        A("-- into a CALLER-SUPPLIED table so parsing allocates nothing.")
        A(f"function M.parse_{name.lower()}(s, p, out)")
        for f in fields:
            reader, _ = _LUA_READER[f.fmt]
            if f.scale != 1.0:
                A(f"  out.{f.name}, p = {reader}(s, p); "
                  f"out.{f.name} = out.{f.name} / {f.scale}")
            else:
                A(f"  out.{f.name}, p = {reader}(s, p)")
        A("  return out, p")
        A("end")
        A("")

    # STATE and STATE2 are the messages the FC SENDS.  string.pack allocates a
    # string per call, which transmission cannot avoid.
    for msg_const, fn_name, fields in (("MSG_STATE", "pack_state", STATE),
                                       ("MSG_STATE2", "pack_state2", STATE2)):
        A(f"function M.{fn_name}(t)")
        fmtparts, args = [], []
        for f in fields:
            fmtparts.append({"B": "B", "H": "I2", "h": "i2",
                             "I": "I4", "i": "i4", "f": "f"}[f.fmt])
            # SCALE ON THE WAY OUT.  parse_* divides by f.scale, so this writer
            # must multiply by it; a scaled field emitted raw reads back f.scale
            # times too small.
            #
            # The integer formats also need an explicit round: string.pack('i2',
            # x) raises "number has no integer representation" on any
            # non-integral x, which would crash the script mid-flight the first
            # time the value was not a whole number.
            if f.scale != 1.0:
                expr = f"(t.{f.name} or 0) * {f.scale}"
                if f.fmt != "f":
                    expr = f"math.floor({expr} + 0.5)"
                args.append(expr)
            elif f.fmt != "f":
                args.append(f"math.floor((t.{f.name} or 0) + 0.5)")
            else:
                args.append(f"t.{f.name} or 0")
        A(f"  local body = string.pack('<{''.join(fmtparts)}', "
          + ", ".join(args) + ")")
        A(f"  local head = string.pack('<I2BB', M.MAGIC, M.{msg_const}, #body)")
        A("  local hb = head .. body")
        A("  return hb .. string.pack('<I2', M.crc16(hb, 1, #hb))")
        A("end")
        A("")

    if gains_cls is not None:
        idx = gain_index_map(gains_cls)
        A("-- SET_GAIN index space, generated from ControlGains field order")
        A(f"-- ({len(idx)} gains).  Both ends derive from the same dataclass, so")
        A("-- inserting a gain upstream shifts Python and Lua together.")
        A("M.GAIN_NAMES = {")
        for n, i in sorted(idx.items(), key=lambda kv: kv[1]):
            A(f"  [{i}] = '{n}',")
        A("}")
        A("")

    A("return M")
    return "\n".join(L) + "\n"


if __name__ == "__main__":
    import os
    from .control_laws import ControlGains
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.abspath(os.path.join(here, "..", "LUA", "protocol.lua"))
    src = generate_lua(ControlGains)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    open(out, "w").write(src)
    print(f"GUIDE    {payload_size(GUIDE):>3} B payload -> "
          f"{HEADER_SIZE + payload_size(GUIDE) + CRC_SIZE} B on the wire")
    print(f"SET_GAIN {payload_size(SET_GAIN):>3} B payload")
    print(f"EXCITE   {payload_size(EXCITE):>3} B payload")
    print(f"PLAN     {payload_size(PLAN):>3} B payload")
    print(f"STATE    {payload_size(STATE):>3} B payload -> "
          f"{HEADER_SIZE + payload_size(STATE) + CRC_SIZE} B on the wire")
    print(f"STATE2   {payload_size(STATE2):>3} B payload -> "
          f"{HEADER_SIZE + payload_size(STATE2) + CRC_SIZE} B on the wire")
    print(f"gains indexed: {len(gain_index_map(ControlGains))}")
    print(f"generated {out} ({src.count(chr(10))} lines)")
