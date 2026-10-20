# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""Controller Autotune parameter store: routes ControllerAutotune attribute names to their backing
state on the controller.

A controller-autotune row name maps to one of three places:
  - the gimbal driver's slew rate          (_GIMBAL)
  - a runtime knob attribute on the controller (_RUNTIME)
  - a field on the controller's ControlGains   (everything else)

DEFAULT_CONST maps each controller-autotune attr name to the constants.py name that holds its
default, used by reset_all()/reset_one().
"""
from __future__ import annotations
import logging
import math
import struct

from . import constants as C
from . import protocol as P

log = logging.getLogger("balloon_chaser")


# Mapping from tuner attr name -> the constants.py default name, used by the
# ParamStore reset_all()/reset_one().
DEFAULT_CONST = {
    'aileron_override': 'AILERON_OVERRIDE',
    'aileron_deflection_pct': 'AILERON_DEFLECTION_PCT',
    'elevator_override': 'ELEVATOR_OVERRIDE',
    'elevator_deflection_pct': 'ELEVATOR_DEFLECTION_PCT',
    'rudder_override': 'RUDDER_OVERRIDE',
    'rudder_deflection_pct': 'RUDDER_DEFLECTION_PCT',
    'pitch_turn_ff': 'PITCH_TURN_FF',
    'yaw_point_kp': 'YAW_POINT_KP',
    'yaw_point_ki': 'YAW_POINT_KI',
    'yaw_point_lim': 'YAW_POINT_LIM',
    'yaw_point_windup': 'YAW_POINT_WINDUP',
    'yaw_point_pitch_sched': 'YAW_POINT_PITCH_SCHED',
    'yaw_coordination_ff': 'YAW_COORDINATION_FF',
    'curve_flyer_enable': 'CURVE_FLYER_ENABLE',
    'tilt_back_travel_deg': 'TILT_BACK_TRAVEL_DEG',
    'cf_transfer_when_done': 'CF_TRANSFER_WHEN_DONE',
    'cf_plan_wait_s': 'CF_PLAN_WAIT_S',
    'cf_approach_deg': 'CF_APPROACH_DEG',
    'steering_law': 'STEERING_LAW',
    'att_k_path': 'ATT_K_PATH',
    'att_path_rate_max_dps': 'ATT_PATH_RATE_MAX_DPS',
    'att_lead_s': 'ATT_LEAD_S',
    'att_k_roll': 'ATT_K_ROLL',
    'att_k_side': 'ATT_K_SIDE',
    'att_roll_ki': 'ATT_ROLL_KI',
    'att_pitch_ki': 'ATT_PITCH_KI',
    'att_i_zone_deg': 'ATT_I_ZONE_DEG',
    'pitch_trim_offset': 'PITCH_TRIM_OFFSET',
    'pitch_trim_lift': 'PITCH_TRIM_LIFT',
    'pitch_trim_attitude': 'PITCH_TRIM_ATTITUDE',
    'pitch_trim_thrust': 'PITCH_TRIM_THRUST',
    'plane_side_hold_deg': 'PLANE_SIDE_HOLD_DEG',
    'plane_gain': 'PLANE_GAIN',
    'plane_ff_gain': 'PLANE_FF_GAIN',
    'plane_ff_lpf_rate': 'PLANE_FF_LPF_RATE',
    'plane_steep_start_deg': 'PLANE_STEEP_START_DEG',
    'plane_steep_full_deg': 'PLANE_STEEP_FULL_DEG',
    'plane_ki': 'PLANE_KI',
    'plane_windup': 'PLANE_WINDUP',
    'plane_kd': 'PLANE_KD',
    'plane_kd_rate': 'PLANE_KD_RATE',
    'bank_lim_deg': 'BANK_LIM_DEG',
    'pan_p_gain': 'PAN_P_GAIN',
    'roll_gain': 'ROLL_GAIN',
    'pan_ki': 'PAN_KI',
    'windup_limit_pan': 'WINDUP_LIMIT_PAN',
    'pan_kd': 'PAN_KD',
    'pan_kd_rate': 'PAN_KD_RATE',
    'pitch_kd': 'PITCH_KD',
    'pitch_kd_rate': 'PITCH_KD_RATE',
    'pitch_gain': 'PITCH_GAIN',
    'pitch_ki': 'PITCH_KI',
    'windup_limit_pitch': 'WINDUP_LIMIT_PITCH',
    'gimbal_slew_rate': 'GIMBAL_SLEW_RATE',
    'point_lon_deg': 'POINT_LON_DEG',
    'point_lat_deg': 'POINT_LAT_DEG',
    'point_balloon_move_enable': 'POINT_BALLOON_MOVE_ENABLE',
    'point_balloon_speed': 'POINT_BALLOON_SPEED',
    'point_balloon_range': 'POINT_BALLOON_RANGE',
    'point_balloon_vel_pan': 'POINT_BALLOON_VEL_PAN',
    'point_balloon_vel_tilt': 'POINT_BALLOON_VEL_TILT',
    'arrow_left_deg': 'ARROW_LEFT_DEG',
    'arrow_right_deg': 'ARROW_RIGHT_DEG',
    'arrow_up_deg': 'ARROW_UP_DEG',
    'arrow_down_deg': 'ARROW_DOWN_DEG',
    'throttle_pct': 'THROTTLE_PCT',
    'special_throttle_enable': 'SPECIAL_THROTTLE_ENABLE',
    'special_throttle_pct': 'SPECIAL_THROTTLE_PCT',
    'airspeed_cmd_enable': 'AIRSPEED_CMD_ENABLE',
    'airspeed_cmd_mps': 'AIRSPEED_CMD_MPS',
    'airspeed_cmd_kp': 'AIRSPEED_CMD_KP',
    'airspeed_cmd_ki': 'AIRSPEED_CMD_KI',
    'draw_roll_error': 'DRAW_ROLL_ERROR',
    'autotune_bracket_wide': 'AUTOTUNE_BRACKET_WIDE',
    'autotune_use_constants': 'AUTOTUNE_USE_CONSTANTS',
    'cell_autotune_use_constants': 'CELL_AUTOTUNE_USE_CONSTANTS',
    'autotune_cell_lon_deg': 'AUTOTUNE_CELL_LON_DEG',
    'autotune_cell_lat_deg': 'AUTOTUNE_CELL_LAT_DEG',
    'autotune_pan_min': 'AUTOTUNE_PAN_MIN',
    'autotune_pan_max': 'AUTOTUNE_PAN_MAX',
    'autotune_pan_step': 'AUTOTUNE_PAN_STEP',
    'autotune_tilt_min': 'AUTOTUNE_TILT_MIN',
    'autotune_tilt_max': 'AUTOTUNE_TILT_MAX',
    'autotune_tilt_step': 'AUTOTUNE_TILT_STEP',
    'autotune_protect_floor_m': 'AUTOTUNE_PROTECT_FLOOR_M',
    'autotune_protect_climb_to_m': 'AUTOTUNE_PROTECT_CLIMB_TO_M',
    'plantid_do_pitch': 'PLANTID_DO_PITCH',
    'plantid_do_yaw': 'PLANTID_DO_YAW',
    'plantid_do_roll': 'PLANTID_DO_ROLL',
    'plantid_rate_agg': 'PLANTID_RATE_AGG',
    'trim_map_climb_min_deg': 'TRIM_MAP_CLIMB_MIN_DEG',
    'trim_map_climb_max_deg': 'TRIM_MAP_CLIMB_MAX_DEG',
    'trim_map_climb_step_deg': 'TRIM_MAP_CLIMB_STEP_DEG',
    'trim_map_speed_min_mps': 'TRIM_MAP_SPEED_MIN_MPS',
    'trim_map_speed_max_mps': 'TRIM_MAP_SPEED_MAX_MPS',
    'trim_map_speed_step_mps': 'TRIM_MAP_SPEED_STEP_MPS',
    'trim_map_protect_floor_m': 'TRIM_MAP_PROTECT_FLOOR_M',
    'trim_map_protect_climb_to_m': 'TRIM_MAP_PROTECT_CLIMB_TO_M',
    'nose_cam_display': 'NOSE_CAM_DISPLAY',
    'gimbal_cam_display': 'GIMBAL_CAM_DISPLAY',
    'auto_gimbal': 'AUTO_GIMBAL',
    'fov_est_enable': 'FOV_EST_ENABLE',
}



def guide_field_range(name):
    """(lo, hi): the values the GUIDE field `name` can carry to the FC.  A
    value outside it does not reach the aircraft - struct refuses to pack it and
    the control loop dies on the tick it is sent."""
    f = next(f for f in P.GUIDE if f.name == name)
    bits = 8 * struct.calcsize('<' + f.fmt)
    if f.fmt.isupper():
        return 0.0, (2 ** bits - 1) / f.scale
    return -(2 ** (bits - 1)) / f.scale, (2 ** (bits - 1) - 1) / f.scale


# The largest finite 32-bit float.  SET_GAIN carries every gain as one, so a
# larger typed value cannot be sent at all: struct refuses to pack it and the
# control loop dies on the spot.  It is refused like infinity, which is what a
# 32-bit float makes of it.
FLOAT32_MAX = struct.unpack('<f', b'\xff\xff\x7f\x7f')[0]


def float32_text(value):
    """The shortest text that reproduces `value` as a 32-bit float - the form
    the FC stores a parameter in.  One can need nine significant figures, so
    six-figure text (`%g`) can name a neighbouring value: written, it lands
    there; printed, it hides the difference."""
    try:
        bits = struct.pack('<f', value)
    except (OverflowError, struct.error):
        return repr(value)
    for digits in range(1, 10):
        near = float(f"{value:.{digits}g}")
        if struct.pack('<f', near) == bits:
            return repr(near)
    return repr(value)


# Where each stored Acro default lives in constants.py, for the GUI Live Values
# copy to rewrite: (dict, key) for an EXPECTED_RATE_PIDS entry, (constant, None)
# for a scalar.  ACRO_DEFAULT_PARAMS is built from exactly these.
ACRO_DEFAULT_ENTRIES = {
    **{n: ('EXPECTED_RATE_PIDS', n) for n in C.EXPECTED_RATE_PIDS},
    'PTCH_RATE_FLTE': ('PTCH_RATE_FLTE_EXPECTED', None),
    'RLL_RATE_FLTE':  ('RLL_RATE_FLTE_EXPECTED', None),
}


def acro_default_value(name):
    """The stored Acro default for the saved ArduPlane parameter `name`
    (constants.ACRO_DEFAULT_PARAMS) - the value startup checks it against, the
    one its reset icon writes and the one the Default Acro Parameters Write
    sends - or None where there is none."""
    value = C.ACRO_DEFAULT_PARAMS.get(name)
    return None if value is None else float(value)


# ---------------------------------------------------------------------------
# PARAM_LIMITS - hard bounds on tunables, applied on every set().
#
# Nothing else stands between a Gain Tuner keystroke and the next control tick,
# so 10 typed for 1.0 or a stray minus sign would reach the aircraft directly.
# Out-of-range values are CLAMPED to the nearest bound and logged loudly rather
# than refused: a refusal mid-flight leaves the operator unsure of the value.
#
# Bounds are DERIVED, not invented - each is either a documented fraction, a
# physical limit, or the sign the maths requires.  Where no real bound exists
# the entry is (0.0, None): non-negative only, because every gain, time
# constant and rate limit here inverts or destabilises its loop if negative.
# None on either side means unbounded in that direction.
#
# Discrete params (DISCRETE_PARAMS in the tuner) are NOT listed: they cycle
# through an explicit allowed list and cannot leave it.
PARAM_LIMITS = {
    # A scale on a computed geometric quantity: 1.0 is the geometry, and more
    # over-drives it - a choice, not an impossibility, so no ceiling.
    # Non-negative: below 0 it adds to the rotation instead of supplying it, and
    # the pitch loop then fights twice as hard.
    'pitch_turn_ff':                (0.0, None),
    # Non-negative: the sign of the pointing loop lives in the law, so a negative
    # gain inverts it rather than describing the airframe.
    'yaw_point_kp':                      (0.0, None),
    'yaw_point_ki':                      (0.0, None),
    # deg/s. Non-negative only, no upper clamp: it bounds a DEMAND (the rudder's
    # share of the slip budget). Asking past what the airframe can deliver
    # saturates at the surface - a choice, not an impossibility - so no derived
    # ceiling exists.
    'yaw_point_lim':                     (0.0, None),
    # documented fraction: the integrator's share of yaw_point_lim, 0..1.
    'yaw_point_windup':                  (0.0, 1.0),
    # Non-negative: a negative scale asks for yaw against the turn and the roll,
    # which is the slip coordination removes.  No ceiling: past 1 over-drives
    # the geometry, a choice, not an impossibility.
    'yaw_coordination_ff':               (0.0, None),
    # A distance the tilt may come back, so non-negative; no ceiling.
    'tilt_back_travel_deg':              (0.0, None),
    'cf_plan_wait_s':                    (0.0, None),
    # An angle below the horizon: from level to straight down.
    'cf_approach_deg':                   (0.0, 90.0),
    # Lift Vector Pointing.  Non-negative, no ceiling: a negative gain turns
    # the path or rolls the lift line away from where it should go, a negative
    # cap has no meaning, and the ACRO rates already bound what is flown.
    'att_k_path':                        (0.0, None),
    'att_path_rate_max_dps':             (0.0, None),
    'att_k_roll':                        (0.0, None),
    'att_k_side':                        (0.0, None),
    # A lead time: negative would read the path behind where it is.
    'att_lead_s':                        (0.0, None),
    # Integrator gains: a negative one winds away from the balloon.  The zone
    # is an angle off the path (at most 180 deg).
    'att_roll_ki':                       (0.0, None),
    'att_pitch_ki':                      (0.0, None),
    'att_i_zone_deg':                    (0.0, 180.0),
    # Elevator units, either sign: the airframe needs up-elevator level and down
    # in a steep climb, so attitude is negative on this airframe, and thrust is
    # negative when the motor pitches the nose up.  No bound on any: ArduPlane
    # clips the SUM at full travel, and a term past 45 still means something
    # where the sum stays inside it.  SET_GAIN's 32-bit float is the only limit.
    'pitch_trim_offset':                 (None, None),
    'pitch_trim_lift':                   (None, None),
    'pitch_trim_attitude':               (None, None),
    'pitch_trim_thrust':                 (None, None),
    # The 12/6 choice changes at 90 deg round the clock plus or minus the band.
    # Negative crosses the two thresholds, so the choice flips back and forth;
    # past 90 the threshold is beyond the clock's own 180 and the choice never
    # changes, which 90 already gives.
    'plane_side_hold_deg':               (0.0, 90.0),
    # Non-negative: a negative gain or feedforward rolls away from the line, a
    # negative derivative gain rolls harder the faster the balloon closes on it,
    # and a negative rate makes a low-pass grow without bound.
    'plane_gain':                        (0.0, None),
    'plane_ff_gain':                     (0.0, None),
    'plane_ff_lpf_rate':                 (0.0, None),
    'plane_kd':                          (0.0, None),
    'plane_kd_rate':                     (0.0, None),
    # A flight-path angle above or below the horizon cannot pass 90, nor can
    # the Euler pitch that stands in for it.
    'plane_steep_start_deg':             (0.0, 90.0),
    'plane_steep_full_deg':              (0.0, 90.0),
    # A negative integrator gain winds away from the line; the ceiling is a
    # fraction of the roll rate.
    'plane_ki':                          (0.0, None),
    'plane_windup':                      (0.0, 1.0),
    # switches: 0 the law flies the surface, 1 the deflection below does.
    'aileron_override':             (0.0, 1.0),
    'elevator_override':            (0.0, 1.0),
    'rudder_override':              (0.0, 1.0),
    # full surface travel, both ways: these are the plant-ID knobs.
    'aileron_deflection_pct':       (-100.0, 100.0),
    'elevator_deflection_pct':      (-100.0, 100.0),
    'rudder_deflection_pct':        (-100.0, 100.0),
    # -- documented fractions in [0,1] --
    # windup_limit is the fraction of the authority envelope the integrator may
    # own: max_ri = (envelope * windup_limit) / ki.  Above 1.0 the integrator
    # alone could exceed the full envelope.
    'windup_limit_pan':     (0.0, 1.0),
    'windup_limit_pitch':           (0.0, 1.0),

    # -- angles with physical meaning --
    # Bank beyond 90 deg is inverted flight; this is the single bank-angle clip.
    'bank_lim_deg':         (0.0, 90.0),
    # an angular error cannot exceed 180 deg, so 180 already never bleeds.
    'autotune_cell_lon_deg':        (-180.0, 180.0),
    'autotune_cell_lat_deg':        (-90.0, 90.0),
    'point_lon_deg':                (-180.0, 180.0),
    'point_lat_deg':                (-90.0, 90.0),
    # m/s; a speed is non-negative and has no ceiling of its own.
    'point_balloon_speed':           (0.0, None),
    # No upper bound: the GROUND is the ceiling, applied at injection where the
    # altitude is known (min(range, ground) - see constants.POINT_BALLOON_RANGE).
    # The 10 m floor stays - closer than that the balloon is inside the aircraft.
    'point_balloon_range':           (10.0, None),
    'point_balloon_vel_pan':         (-180.0, 180.0),
    'point_balloon_vel_tilt':        (-90.0, 90.0),
    'arrow_left_deg':               (0.0, 180.0),
    'arrow_right_deg':              (0.0, 180.0),
    'arrow_up_deg':                 (0.0, 180.0),
    'arrow_down_deg':               (0.0, 180.0),

    # -- sweep grid: pan is a magnitude, tilt is nose-DOWN so it is negative --
    'autotune_pan_min':             (0.0, 180.0),
    'autotune_pan_max':             (0.0, 180.0),
    'autotune_pan_step':            (0.1, 180.0),   # 0 would be an infinite grid
    'autotune_tilt_min':            (-90.0, 90.0),
    'autotune_tilt_max':            (-90.0, 90.0),
    'autotune_tilt_step':           (0.1, 180.0),

    # Altitude-protection floor and climb-to, metres AGL, shared by 't' and 'i'.
    # Non-negative only: an altitude below ground is not a floor, and a
    # negative climb goal is not a ceiling. No upper bound - how much clear air
    # to demand is a mission choice, not a property of the loop. The floor-below-
    # climb-to ordering is a cross-parameter rule a single-value clamp cannot
    # state, so it lives in the info text and is bounded in flight by the retry cap.
    'autotune_protect_floor_m':     (0.0, None),
    'autotune_protect_climb_to_m':  (0.0, None),

    # Plant-ID rate-PID aggressiveness (multiple of the reference rate bandwidth).
    # Non-negative only: a negative bandwidth is not a bandwidth, and the sign the
    # maths requires (the computed P is clamped >=0 anyway). No upper bound - a
    # harder rate loop is an operator choice, not a physical impossibility, so any
    # ceiling would be invented.
    'plantid_rate_agg':             (0.0, None),

    # Trim-map grid.  A climb angle is an elevation above the horizon, and
    # past +-90 it is no longer one.  A step of 0 flies the minimum alone; a
    # step past the widest span (180) flies nothing more than 180 does.  An
    # airspeed and its step are non-negative with no ceiling of their own.
    'trim_map_climb_min_deg':       (-90.0, 90.0),
    'trim_map_climb_max_deg':       (-90.0, 90.0),
    'trim_map_climb_step_deg':      (0.0, 180.0),
    'trim_map_speed_min_mps':       (0.0, None),
    'trim_map_speed_max_mps':       (0.0, None),
    'trim_map_speed_step_mps':      (0.0, None),
    # Its altitude protection: the same grounds as autotune_protect_* above.
    'trim_map_protect_floor_m':     (0.0, None),
    'trim_map_protect_climb_to_m':  (0.0, None),

    # -- percentages and rates --
    'throttle_pct':                 (0.0, 100.0),
    'special_throttle_enable':      (0.0, 1.0),
    'special_throttle_pct':         (0.0, 100.0),
    # m/s; an airspeed is non-negative.  How slow or fast is too slow or fast
    # is the airframe's, not the maths', so no other bound.  The gains, and the
    # gimbal slew rate, take the non-negative default below.
    'airspeed_cmd_mps':             (0.0, None),
}

# Everything else continuous: non-negative.  A negative gain inverts the loop,
# a negative time constant or rate limit is meaningless, and a negative leak
# GROWS the integrator without bound.
_NON_NEGATIVE_DEFAULT = (0.0, None)

# Parameters where +inf is a real value rather than garbage.  Kept separate from
# PARAM_LIMITS because `None` there means "no upper bound", which is weaker than
# "may HOLD infinity".  point_balloon_range is the only one: infinitely far is a
# direction, which the injector can anchor (see constants.POINT_BALLOON_RANGE).
# -inf and NaN stay rejected.
INFINITY_ALLOWED = {'point_balloon_range'}


def limits_for(attr):
    """(lo, hi) for a tunable; None on a side means unbounded there."""
    return PARAM_LIMITS.get(attr, _NON_NEGATIVE_DEFAULT)


def clamp_param(attr, value):
    """Bound `value` for `attr`.  Returns (value, status) where status is
    'ok' | 'clamped' | 'rejected'.

    Two different failures need two different answers:

    * FINITE but out of range -> CLAMP to the nearest bound.  The operator
      meant a real number and overshot; the nearest legal value is the closest
      thing to their intent, and refusing mid-flight leaves them unsure what
      the parameter even is.
    * NON-FINITE or unparseable (NaN, inf, '1e999', text) -> REJECT, keeping
      the current value.  So is anything past FLOAT32_MAX, which the link
      can only carry as infinity.  There is no "nearest bound" to a NaN, and the
      tempting fallbacks are both dangerous: clamping +inf down to lo would
      turn a runaway gain into ZERO, silently disabling that term (a
      bank_lim of 0 is no roll authority at all), while letting it through
      poisons every downstream state and cannot be recovered from in flight.
      Keeping the last known-good value is the only safe reading of garbage.

      ONE EXCEPTION: +inf is accepted for the attrs in INFINITY_ALLOWED, where
      it is a real value.  NaN and -inf are still rejected everywhere.  A
      fat-fingered '1e999' would also overflow to inf, so the UI separates a
      typed infinity from an overflow before it reaches here (see
      ControllerAutotune._on_entry_commit).
    """
    lo, hi = limits_for(attr)
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None, 'rejected'
    if v != v:                                     # NaN: no nearest bound
        return None, 'rejected'
    if v == float('inf') and attr in INFINITY_ALLOWED:
        return v, 'ok'                             # a declared, meaningful infinity
    if v in (float('inf'), float('-inf')):
        return None, 'rejected'
    if abs(v) > FLOAT32_MAX:                       # infinity to the FC; see FLOAT32_MAX
        return None, 'rejected'
    if lo is not None and v < lo:
        return lo, 'clamped'
    if hi is not None and v > hi:
        return hi, 'clamped'
    return v, 'ok'


class ControllerAutotuneStore:
    """Routes controller-autotune attr names to ControlGains / runtime knobs / gimbal.
    Implements the ParamStore protocol the ControllerAutotune expects."""

    # attrs that live on ControlGains (everything not in the runtime/gimbal sets)
    _RUNTIME = {'throttle_pct', 'special_throttle_enable', 'special_throttle_pct',
                # point_lon_deg / point_lat_deg live on the CONTROLLER, like the
                # five point_balloon_* below them.  They were the only two of the
                # seven left out, so get() fell through to the ControlGains
                # object - which has no such field - and the tuner showed 0 while
                # the injection read the controller's real 75 / -40.  Setting
                # them from the GUI did nothing at all.
                'point_lon_deg', 'point_lat_deg',
                'point_balloon_move_enable', 'point_balloon_speed', 'point_balloon_range',
                'point_balloon_vel_pan', 'point_balloon_vel_tilt',
                'arrow_left_deg', 'arrow_right_deg', 'arrow_up_deg', 'arrow_down_deg', 'draw_roll_error',
                'autotune_pan_min', 'autotune_pan_max', 'autotune_pan_step',
                'autotune_tilt_min', 'autotune_tilt_max', 'autotune_tilt_step',
                'autotune_cell_lon_deg', 'autotune_cell_lat_deg',
                'autotune_use_constants', 'cell_autotune_use_constants',
                'autotune_bracket_wide',
                'autotune_protect_floor_m', 'autotune_protect_climb_to_m',
                'plantid_do_pitch', 'plantid_do_yaw', 'plantid_do_roll',
                'plantid_rate_agg',
                'trim_map_climb_min_deg', 'trim_map_climb_max_deg',
                'trim_map_climb_step_deg', 'trim_map_speed_min_mps',
                'trim_map_speed_max_mps', 'trim_map_speed_step_mps',
                'trim_map_protect_floor_m', 'trim_map_protect_climb_to_m',
                'nose_cam_display', 'gimbal_cam_display',
                'auto_gimbal', 'fov_est_enable'}
    _GIMBAL = {'gimbal_slew_rate'}

    def __init__(self, controller):
        self._c = controller

    def _active_gains(self):
        """The gains the TUNER acts on.  While a measurement (plant-ID) owns
        the configuration, the control law flies a private copy and this keeps
        returning the operator's own set - so the switches never appear to move
        on their own, and an edit made during a run is not overwritten when the
        run restores its copy."""
        og = getattr(self._c, 'operator_gains', None)
        return og() if callable(og) else self._c._gains

    def get(self, attr):
        if attr in self._GIMBAL:
            return float(self._c.gimbal.slew_rate_dps)
        if attr in self._RUNTIME:
            return float(getattr(self._c, attr))
        return float(getattr(self._active_gains(), attr))

    def set(self, attr, value):
        # Clamp FIRST - before anything reaches the live gains.  This is the
        # single choke point every setter goes through (tuner, reset, sweep),
        # so bounding here bounds all of them.
        requested = value          # keep the ORIGINAL for the message: clamp_param
                                   # returns None on rejection, and reporting that
                                   # tells the operator nothing about what they typed
        value, status = clamp_param(attr, requested)
        lo, hi = limits_for(attr)
        if status == 'rejected':
            log.error("Gain Tuner: %s = %r is not a finite number the link can carry - "
                      "REFUSED, the parameter keeps its current value (%s).",
                      attr, requested, self.get(attr))
            return
        if status == 'clamped':
            log.warning("Gain Tuner: %s out of range - CLAMPED to %s (allowed %s..%s)",
                        attr, value,
                        "-inf" if lo is None else lo, "+inf" if hi is None else hi)
        if attr in self._GIMBAL:
            # Single source of truth: the gimbal driver owns the slew rate.
            # It drives BOTH the visual-servo slew AND the trust formula's
            # alpha_C, exactly as the source's self.gimbal_slew_rate did.
            self._c.gimbal.slew_rate_dps = value
        elif attr in self._RUNTIME:
            setattr(self._c, attr, value)
        else:
            setattr(self._active_gains(), attr, value)
        if getattr(self._c, 'measurement_owns_gains', lambda: False)():
            log.info("Controller Autotune set: %s = %s  (a measurement run owns the "
                     "gains right now, so this takes effect when it finishes)",
                     attr, value)
        else:
            log.info("Controller Autotune set: %s = %s", attr, value)

    def reset_all(self):
        for attr, const in DEFAULT_CONST.items():
            val = getattr(C, const)
            self.set(attr, val)

    def reset_one(self, attr):
        """Reset a single parameter to its constant default."""
        const = DEFAULT_CONST.get(attr)
        if const is None:
            return
        self.set(attr, getattr(C, const))

    # ----- saved ArduPlane parameters (constants.ARDUPLANE_AUTOTUNE_PARAMS) -----
    # These live on the FC, not in this program: the tuner shows the value the FC
    # last CONFIRMED and writes when the operator presses a Write, and each mode
    # switch writes the rate loop that mode flies.  Reads and writes BLOCK for a
    # MAVLink round trip, so the tuner runs them on a worker thread, and every
    # outcome goes to the console.

    def native_value(self, name):
        """The value the FC last confirmed for `name`, or None if it has not."""
        known = getattr(self._c.link, 'native_params', None)
        return known.get(name) if isinstance(known, dict) else None

    def native_startup_value(self, name):
        """The value the FC held for `name` when this program started, or None
        if it did not answer then."""
        known = getattr(self._c.link, 'native_params_at_startup', None)
        return known.get(name) if isinstance(known, dict) else None

    def read_native(self, name):
        """Read `name` from the FC.  BLOCKS; call from a worker thread."""
        if not callable(getattr(self._c.link, 'read_native_param', None)):
            log.error("[FC PARAM] read %s: this link cannot read FC parameters", name)
            return None
        return self._c.link.read_native_param(name)

    def read_all_native(self):
        """Read every saved ArduPlane parameter from the FC.  BLOCKS; call from a
        worker thread."""
        if not callable(getattr(self._c.link, 'read_all_native_params', None)):
            log.error("[FC PARAM] Read All: this link cannot read FC parameters")
            return None
        return self._c.link.read_all_native_params()

    def write_native(self, name, text):
        """Write the operator's value for `name` to the FC and read it back.
        BLOCKS; call from a worker thread.

        Refused, with nothing sent, unless it is a finite number from 0 to
        FLOAT32_MAX: every one of these is a gain, a filter frequency, a time
        constant or a rate limit, none of which means anything below zero -
        a negative filter frequency is an internal error in ArduPilot
        (calc_lowpass_alpha_dt) - and PARAM_SET carries a 32-bit float.  A
        refused value is not clamped: this writes exactly what was typed or
        nothing."""
        try:
            value = float(str(text).strip())
        except (TypeError, ValueError):
            log.error("[FC PARAM] write %s = %r refused: not a number - nothing sent",
                      name, text)
            return None
        if not (math.isfinite(value) and 0.0 <= value <= FLOAT32_MAX):
            log.error("[FC PARAM] write %s = %r refused: must be a finite number from 0 "
                      "up - nothing sent", name, text)
            return None
        if not callable(getattr(self._c.link, 'write_native_param', None)):
            log.error("[FC PARAM] write %s: this link cannot write FC parameters", name)
            return None
        return self._c.link.write_native_param(name, value)

    def write_param_set(self, which):
        """Write one whole stored set from constants.py to the FC and read it
        back: 'native' (NATIVE_MODE_RATE_PIDS, what ArduPlane's own modes fly)
        or 'operator' (ACRO_DEFAULT_PARAMS, the stored Acro defaults, which
        then become what ACRO flies).  BLOCKS; call from a worker thread."""
        if not callable(getattr(self._c.link, 'write_rate_loop_set', None)):
            log.error("[FC PARAM] %s set Write: this link cannot write FC parameters",
                      which)
            return None
        return self._c.link.write_rate_loop_set(which)

    def flush_csv(self):
        """Flush the live balloon chaser CSV without closing it.  The Gain Tuner's
        file-export buttons call this before copying flight_log.csv so the
        copy is not missing the rows still sitting in the write buffer (the
        logger batches ~0.5 s of rows between flushes).  No-op when logging is
        not running."""
        csv = getattr(self._c, '_csv', None)
        if csv is not None:
            try:
                csv.flush()
            except Exception:                          # noqa: BLE001
                log.exception("CSV flush before export failed")

    def default_for(self, attr):
        """The constants.py default value for a parameter (for reset / bracketing).
        Falls back to the current value if the attr has no DEFAULT_CONST entry."""
        const = DEFAULT_CONST.get(attr)
        if const is None:
            return self.get(attr)
        return float(getattr(C, const))

    def write_current_constants(self, out_name=None):
        """Snapshot the LIVE Gain-Tuner configuration to CWD/<out_name>: a copy of
        the running constants.py with every GUI tunable rewritten to its CURRENT
        setting - the inverse of reset_all() - and the operator's ACRO values of
        the saved ArduPlane parameters as the stored Acro defaults
        (_acro_value_subs).  Returns the written path, or None if
        nothing matched.  Non-tunable constants and all comments/structure are
        preserved verbatim (constants_suggest copies the file and substitutes only
        the named assignments), so the result is a drop-in constants.py.  Nothing
        is applied automatically; the file's banner carries the cp command to
        overwrite constants.py after review.

        Each value is emitted in the type the constant is declared as (int / bool /
        float) so flags and mode selectors read as 1 / True / 0, not 1.0."""
        from .constants_suggest import write_suggested_constants
        scalar_subs = {}
        for attr, const in DEFAULT_CONST.items():
            try:
                live = self.get(attr)                  # get() returns float
            except Exception:                          # noqa: BLE001
                log.debug("Save config: could not read %s; leaving %s untouched",
                          attr, const)
                continue
            default = getattr(C, const, None)
            if isinstance(default, bool):              # bool is-a int: check first
                lit = repr(bool(round(live)))
            elif isinstance(default, int):
                lit = repr(int(round(live)))
            elif not math.isfinite(live):
                # repr() of an infinite float is the bare word inf, which is no
                # Python name: the copy would fail to import as constants.py.
                lit = "float('%r')" % float(live)
            else:
                lit = repr(float(live))
            scalar_subs[const] = lit
        notes = ["every tunable below is the LIVE Gain-Tuner value at save time",
                 "non-tunable constants are copied from constants.py unchanged"]
        dict_subs, arduplane_notes = self._acro_value_subs(scalar_subs)
        notes += arduplane_notes
        # Stamped with the moment Copy was pressed.  A snapshot has no run to
        # date it by, so the instant it was taken IS its start - and stamping it
        # keeps every generated file traceable to when it was made.
        from . import file_export as _fx
        if out_name is None:
            out_name = _fx.stamped('gui_live_values', 'py', _fx.run_stamp())
        return write_suggested_constants(out_name, "GUI live values", notes, log,
                                          scalar_subs=scalar_subs, from_run=False,
                                          dict_subs=dict_subs)

    def _acro_value_subs(self, scalar_subs):
        """The saved ArduPlane parameters for the GUI Live Values copy: the
        operator's ACRO values, not what the FC holds this instant - in LOITER or
        FBWA the FC holds the Autopilot set, and copying that would store it as
        the Acro defaults.  The program keeps the Acro set in memory
        (native_params_operator) in every mode.

        Each value with a stored Acro default goes into its entry
        (ACRO_DEFAULT_ENTRIES), so installing the copy makes it what startup
        checks, a row's reset writes and the Default Acro Parameters Write sends.
        The attitude settings have no stored default and are listed in the
        banner.  Adds the scalar entries to `scalar_subs`; returns (dict_subs,
        banner notes)."""
        link = getattr(getattr(self, '_c', None), 'link', None)
        acro = getattr(link, 'native_params_operator', None)
        if not isinstance(acro, dict) or not acro:
            return {}, ["ArduPlane parameters: no values from the FC - the stored "
                        "Acro defaults are copied unchanged"]
        dict_subs, missing = {}, []
        for name in C.ACRO_DEFAULT_PARAMS:
            if name not in acro:
                missing.append(name)
                continue
            where, key = ACRO_DEFAULT_ENTRIES[name]
            if key is None:
                scalar_subs[where] = float32_text(acro[name])
            else:
                dict_subs.setdefault(where, {})[key] = float32_text(acro[name])
        notes = ["ArduPlane rate loop: your ACRO values, written as the stored Acro "
                 "defaults (EXPECTED_RATE_PIDS and the _FLTE expected values)"]
        if missing:
            notes.append("not reported by the FC, left as they were: "
                         + ", ".join(missing))
        other = [n for n in C.ARDUPLANE_AUTOTUNE_PARAMS
                 if n not in C.ACRO_DEFAULT_PARAMS and n in acro]
        if other:
            notes.append("no stored default - your ACRO values, set on the FC by hand:")
            notes += ["%s = %s" % (n, float32_text(acro[n])) for n in other]
        return dict_subs, notes
