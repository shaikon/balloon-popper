# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""
BalloonChaser - the orchestrator.

Holds the adapters (AutopilotLink, GimbalDriver, BalloonSensor, two Cameras),
the world-balloon estimator, and the UI (HUD, ControllerAutotune,
KeyboardHandler, CsvLogger).  Implements the per-mode dispatch, the public
mode-transition methods the keyboard calls, the ptest hooks the point-balloon-chaser
test calls, and the main loop.

The control law runs on the flight controller (LUA/balloon_chaser_fc.lua).  Every
tick this side sends it GUIDE - the sensor, the operator's intent and the
configuration - and reads its values back from STATE for the HUD, the CSV and
the measurement tools.  It keeps no copy of the law.
"""
from __future__ import annotations
import logging
import math
import threading
import time


import cv2
import numpy as np

from . import constants as C

from .geometry import (euler_to_rotation_matrix, quat_to_rotation_matrix,
                       vector_to_pan_tilt, yaw_pitch_to_world_vec, aspect_fy_scale)
from .constants import ALIGN_FREEZE_HOLD_S
from .geometry import roll_align_error_deg
from .estimator import WorldBalloonEstimator
from .control_laws import ControlGains
from .csv_logger import CsvLogger, TickSnapshot
from .ui.hud import (HudSnapshot, render_nose_camera, render_gimbal_camera,
                     get_gimbal_frame)
from .ui.controller_autotune import ControllerAutotune
from .controller_autotune_store import ControllerAutotuneStore
from . import gimbal_calibration
from .ui.keyboard import KeyboardHandler
from .testing.point_balloon_chaser import run_point_balloon_chaser_test
from . import file_export
from .controller_autotune_sweep import run_controller_autotune_sweep, CELL_PROFILE
from . import curve_planner

log = logging.getLogger("balloon_chaser")


# GUIDE's `mode` byte for each controller mode (protocol.GUIDE "mode").  Looked
# up directly: a mode missing here stops the loop naming it, rather than
# sending the FC a code it does not know.
MODE_CODE = {"loiter": C.GUIDE_MODE_LOITER, "straight": C.GUIDE_MODE_STRAIGHT,
             "manual": C.GUIDE_MODE_MANUAL,
             "balloon_chaser": C.GUIDE_MODE_BALLOON_CHASER,
             "ptest": C.GUIDE_MODE_PTEST, "autotune": C.GUIDE_MODE_AUTOTUNE}


class LinkFault(RuntimeError):
    """The FC link can no longer be trusted (link.link_fault() said why).  The
    control loop stops on it, hands the aircraft back, and halts on the console
    with the reason."""


def _halt_on_console(reason):
    """Print why the program stopped, framed so it stands out, and wait for
    Enter: the operator reads the reason before the console can scroll it away.
    Without a console to wait on, the message is still printed and logged."""
    rule = "=" * 76
    log.critical(rule)
    log.critical("THE PROGRAM HAS STOPPED")
    for line in str(reason).splitlines() or [""]:
        log.critical("  %s", line)
    log.critical("The aircraft was handed back to LOITER (see the lines above).")
    log.critical(rule)
    try:
        input(">>> Read the message above, then press Enter to close: ")
    except (EOFError, KeyboardInterrupt):
        pass


def _bit(flag):
    """A decoded flag as 0/1, or None where the FC sent none."""
    return None if flag is None else int(bool(flag))


def _fc(x):
    """A decoded FC value as a float, or None where the FC sent none - before
    the first frame, a reading it could not take, or a law internal on a tick
    the law did not run.  Written blank, never as a zero."""
    return None if x is None else float(x)


def _sim_deg(link, attr, idx):
    """One SITL-truth channel in degrees, or None where there is no simulator."""
    v = getattr(link, attr, None)
    return None if v is None else math.degrees(float(v[idx]))


# The runs that fly the 'autotune' regime, by the key that starts and stops
# each: BalloonChaser._run_key while the run's thread is alive.
_AUTOTUNE_RUNS = {
    't': "Controller autotune",
    'y': "Single-cell autotune",
    'i': "Plant ID",
    'e': "The trim map",
}
# How long a switch waits for the run it stops to end.  A mode change already
# under way cannot be interrupted - it waits on HEARTBEAT, then writes its rate
# loop - so this is generous; past it the switch gives up and the aircraft is
# handed back to LOITER (_service_switch).
_SWITCH_WAIT_S = 60.0


# --- Background FOV estimator (DIAGNOSTIC ONLY - see the module note below) ---
# ~160 ms window at 50 Hz: long enough that a normal turn/slew clears the motion
# and pixel floors, short enough to stay near frame centre and keep the linear
# pinhole approximation valid.
_FOV_EST_WINDOW_TICKS = 8
_FOV_EST_NEAR_CENTER  = 0.5    # normalised; the small-angle pinhole map holds here
_FOV_EST_MIN_PIX      = 0.02   # normalised horizontal pixel shift floor (avoids 0/0)
_FOV_EST_MIN_ROT_DEG  = 0.5    # camera rotation floor over the window (avoids 0/0)
_FOV_EST_FORGET       = 0.995  # per-accepted-window forgetting, so a zoom is tracked
_FOV_EST_DOMINANCE    = 3.0    # rotation ratio to attribute a window to method 1/2


def _antisym_vee(dR):
    """Small-angle rotation vector of a near-identity 3x3 rotation, in the frame
    dR is expressed in.  For a short window dR ~ I + [w]_x, so the antisymmetric
    part recovers w = (wx, wy, wz) directly - no matrix log needed."""
    return (0.5 * (dR[2, 1] - dR[1, 2]),
            0.5 * (dR[0, 2] - dR[2, 0]),
            0.5 * (dR[1, 0] - dR[0, 1]))


class FovEstimator:
    """Opportunistic, PASSIVE observer of the camera's horizontal field of view.

    It reads FOV out of motion that is ALREADY happening.  Over a short window
    the balloon's true bearing sweeps by the camera's rotation (perpendicular to
    the line of sight), while its normalised pixel position shifts by that angle
    divided by k = tan(FOV/2).  So k is the slope of (true angle) vs (pixel
    shift), fitted through the origin; FOV = 2*atan(k).

    Two observers, per the design doc: METHOD 1 windows are those where the
    aircraft's own rotation dominates the camera motion (the on-chip attitude is
    the trusted driver); METHOD 2 windows are those where the gimbal slew
    dominates.  Both fold into a combined estimate; each also keeps its own, so a
    post-run comparison can see which observer did better.

    HARD RULES (this class is why C-02 is not violated):
      * It NEVER commands the gimbal or the aircraft.  It only reads.
      * Its output is DIAGNOSTIC ONLY.  Nothing here may ever be read by the
        control law, the world-balloon estimator, the steering error, or the
        GUIDE frame.  It exists to be logged and compared, nothing more.
      * The SDF/CameraInfo ground truth is NEVER fed in here - it is logged
        separately, purely to score this estimate after the fact.

    Zero allocation is not required (companion side), but the fit is recursive:
    per-axis least-squares accumulators with exponential forgetting so the
    estimate tracks a lens/zoom change within seconds instead of averaging it
    away.
    """

    def __init__(self):
        from collections import deque
        self._buf = deque(maxlen=_FOV_EST_WINDOW_TICKS + 1)
        self._last_px = None
        # each accumulator: [Sxx, Sxy, n] for the horizontal least-squares slope
        self._acc = {'all': [0.0, 0.0, 0],
                     'm1':  [0.0, 0.0, 0],
                     'm2':  [0.0, 0.0, 0]}

    def reset(self):
        self._buf.clear()
        self._last_px = None
        for key in self._acc:
            self._acc[key] = [0.0, 0.0, 0]

    def update(self, valid, nx, ny, R_ac_world, R_gimbal_body):
        """Fold one tick.  R_ac_world is body->world (columns = body axes in
        world); R_gimbal_body has columns [right, up, forward] of the camera in
        the body frame, so R_cam_world = R_ac_world @ R_gimbal_body."""
        if not valid:
            # A detection dropout voids any pairing across it - restart clean.
            self._buf.clear()
            self._last_px = None
            return
        nx = float(nx); ny = float(ny)
        # FRESH-FRAME GATE.  The gimbal camera runs at 30 Hz under a 50 Hz loop,
        # so the detection pixel is HELD between frames.  Sample only when it has
        # actually changed, so a window pairs DISTINCT observations, each with the
        # attitude of the tick it arrived on - never a stale pixel against a newer
        # attitude, which is the 30-vs-50 Hz skew the pairing exists to avoid.
        if (self._last_px is not None
                and abs(nx - self._last_px[0]) < 1e-6
                and abs(ny - self._last_px[1]) < 1e-6):
            return
        self._last_px = (nx, ny)
        self._buf.append((nx, ny, R_ac_world, R_gimbal_body))
        if len(self._buf) < self._buf.maxlen:
            return
        # Every buffered sample is a valid, distinct detection by construction.
        s0, s1 = self._buf[0], self._buf[-1]
        nx0, nx1 = s0[0], s1[0]
        if abs(nx0) > _FOV_EST_NEAR_CENTER or abs(nx1) > _FOV_EST_NEAR_CENTER:
            return
        dnx = nx1 - nx0
        if abs(dnx) < _FOV_EST_MIN_PIX:
            return

        Rac0, Rg0 = s0[2], s0[3]
        Rac1, Rg1 = s1[2], s1[3]
        Rcam0 = Rac0 @ Rg0
        Rcam1 = Rac1 @ Rg1
        w = _antisym_vee(Rcam0.T @ Rcam1)          # camera rotation, cam-0 frame
        rot_deg = math.degrees(math.sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]))
        if rot_deg < _FOV_EST_MIN_ROT_DEG:
            return
        # Horizontal true angular displacement of a world-fixed balloon: a camera
        # yaw (about its up-axis, w[1]) moves the balloon the opposite way.  The
        # sign folds into the slope; FOV uses |k|.
        ax = -w[1]

        # The pinhole map nx = tan(theta)/k is nonlinear, so d(theta) =
        # k*dnx*cos^2(theta): pairing d(theta) with the raw dnx alone biases the
        # slope LOW off-centre.  Correct it with cos^2 at the window midpoint,
        # using the running estimate as a self-consistent seed (no FOV constant,
        # so not circular; near cold-start it falls back to this window's raw
        # ratio, where theta is small and the correction is ~1 anyway).
        sxx_all, sxy_all, n_all = self._acc['all']
        k_seed = (abs(sxy_all / sxx_all)
                  if (n_all >= 2 and sxx_all > 1e-9) else abs(ax / dnx))
        th_mid = math.atan(k_seed * 0.5 * (nx0 + nx1))
        dnx_eff = dnx * math.cos(th_mid) ** 2

        # Attribute the window: which source drove the camera this window?
        w_ac = _antisym_vee(Rac0.T @ Rac1)
        w_g  = _antisym_vee(Rg0.T @ Rg1)
        mac = math.sqrt(w_ac[0]**2 + w_ac[1]**2 + w_ac[2]**2)
        mg  = math.sqrt(w_g[0]**2 + w_g[1]**2 + w_g[2]**2)
        keys = ['all']
        if mac > _FOV_EST_DOMINANCE * mg:
            keys.append('m1')          # aircraft-rotation dominant
        elif mg > _FOV_EST_DOMINANCE * mac:
            keys.append('m2')          # gimbal-slew dominant

        for key in keys:
            a = self._acc[key]
            a[0] = _FOV_EST_FORGET * a[0] + dnx_eff * dnx_eff
            a[1] = _FOV_EST_FORGET * a[1] + dnx_eff * ax
            a[2] += 1

    def _fov(self, key):
        sxx, sxy, n = self._acc[key]
        if n < 2 or sxx < 1e-9:
            return 0.0
        k = abs(sxy / sxx)                          # k = tan(FOV/2)
        return math.degrees(2.0 * math.atan(k))

    @property
    def fov_deg(self):    return self._fov('all')
    @property
    def fov_m1_deg(self): return self._fov('m1')
    @property
    def fov_m2_deg(self): return self._fov('m2')
    @property
    def n(self):          return self._acc['all'][2]


class BalloonChaser:
    """Orchestrates adapters + the FC's control law + UI."""

    def __init__(self, link, gimbal, sensor, nose_camera, gimbal_camera,
                 *, takeoff_alt=100.0, waypoint_lat=None, waypoint_lon=None):
        self.link = link
        self.gimbal = gimbal
        self.sensor = sensor
        self.nose_camera = nose_camera
        # Camera VIEW switches (Gain Tuner).  Display only - see constants.py.
        # Written by the Tk tuner thread, read and ACTED ON only by the control
        # thread in _apply_camera_views(); every OpenCV window call stays on the
        # one thread that owns HighGUI.
        self.nose_cam_display = int(C.NOSE_CAM_DISPLAY)
        self.gimbal_cam_display = int(C.GIMBAL_CAM_DISPLAY)
        # Auto Gimbal: 1 = controller may auto-move the gimbal while vision-
        # tracking (l/x); 0 = it must not.  'g' and mouse aiming stay live.
        self.auto_gimbal = int(C.AUTO_GIMBAL)
        # Background FOV estimator (DIAGNOSTIC ONLY; default off).  Its output is
        # logged and compared, never read by control - see FovEstimator.
        self.fov_est_enable = int(C.FOV_EST_ENABLE)
        self._fov_estimator = FovEstimator()
        # GROUND-TRUTH FOV source (Gazebo CameraInfo, from the SDF), wired by main
        # on the sim path.  Read ONLY to log fov_truth_deg for comparison; it is
        # NEVER fed into the estimator or any calculation.
        self.gimbal_fov_truth = None
        self._nose_win_open = False
        self._gimbal_win_open = False

        self.gimbal_camera = gimbal_camera
        # DISPLAY-ONLY nose camera intrinsics (set by main.py for sim; None on real
        # or if unavailable).  Consumed solely by the HUD projection - never by the
        # control path, which is FOV-agnostic.
        self.nose_intrinsics = None

        self.takeoff_alt = takeoff_alt
        self.waypoint_lat = waypoint_lat
        self.waypoint_lon = waypoint_lon

        # -- Control core --
        # Operator-facing gains.  Normally None, meaning _gains IS the
        # operator's set.  While a measurement owns the configuration this
        # holds the operator's untouched copy and _gains is a private one -
        # see begin_measurement_gains().
        self._gains_operator = None
        # DISPLAY-ONLY override for the HUD's mode line.  plant-ID deliberately
        # runs as _mode = 'autotune' ("reuse the sweep's proven mode regime"),
        # so the mode string drives CONTROL behaviour and must not be changed -
        # but the operator should still be told which of the two is running.
        self._mode_label = None
        self._gains = ControlGains(
            pan_p_gain=C.PAN_P_GAIN,
            pan_ki=C.PAN_KI,
            windup_limit_pan=C.WINDUP_LIMIT_PAN,
            bank_lim_deg=C.BANK_LIM_DEG,
            pitch_gain=C.PITCH_GAIN,
            pitch_ki=C.PITCH_KI,
            windup_limit_pitch=C.WINDUP_LIMIT_PITCH,
            pitch_deadband_deg=C.PITCH_DEADBAND_DEG,
            pitch_kd=C.PITCH_KD,
            pitch_kd_rate=C.PITCH_KD_RATE,
            roll_gain=C.ROLL_GAIN,
            pan_kd=C.PAN_KD,
            airspeed_cmd_enable=C.AIRSPEED_CMD_ENABLE,
            airspeed_cmd_mps=C.AIRSPEED_CMD_MPS,
            airspeed_cmd_kp=C.AIRSPEED_CMD_KP,
            airspeed_cmd_ki=C.AIRSPEED_CMD_KI,
            airspeed_cmd_min_valid=C.AIRSPEED_CMD_MIN_VALID,
            pan_kd_rate=C.PAN_KD_RATE,
            pitch_trim_offset=C.PITCH_TRIM_OFFSET,
            pitch_trim_lift=C.PITCH_TRIM_LIFT,
            pitch_trim_attitude=C.PITCH_TRIM_ATTITUDE,
            pitch_trim_thrust=C.PITCH_TRIM_THRUST,
            plane_side_hold_deg=C.PLANE_SIDE_HOLD_DEG,
            plane_gain=C.PLANE_GAIN,
            plane_ff_gain=C.PLANE_FF_GAIN,
            plane_ff_lpf_rate=C.PLANE_FF_LPF_RATE,
            plane_steep_start_deg=C.PLANE_STEEP_START_DEG,
            plane_steep_full_deg=C.PLANE_STEEP_FULL_DEG,
            plane_ki=C.PLANE_KI,
            plane_windup=C.PLANE_WINDUP,
            plane_kd=C.PLANE_KD,
            plane_kd_rate=C.PLANE_KD_RATE,
            yaw_coordination_ff=C.YAW_COORDINATION_FF,
            pitch_turn_ff=C.PITCH_TURN_FF,
        )
        self._gains_pushed_sig = None   # last set sent to the FC; see _push_gains_if_changed
        # The Curve Planner (_service_curve_planner): the PLAN for the chase the
        # FC is waiting on, sent every tick until STATE2 echoes its plan_id; the
        # worker thread planning it; the last plan_id handed out; the chase
        # (mode, tracking, steering law) the plan belongs to.  _cf_plan is also
        # what the log records as the plan in force.
        self._cf_plan = None
        self._cf_thread = None
        self._cf_last_id = 0
        self._cf_fc_state_prev = None
        self._cf_chase = None
        # The Curve Planner's own process (curve_planner.PlannerProcess),
        # started the first time the Curve Flyer is on.
        self._cf_proc = None
        self._world_estimator = WorldBalloonEstimator()

        # -- Runtime (non-cascade) tunables --
        # throttle_pct is a STATIC operator setting (set via the GUI) applied to the
        # throttle channel; the control law never modulates throttle for a maneuver.
        self.throttle_pct = C.THROTTLE_PCT
        # 'p'-test throttle override (tuner switches).  Applied at the point of
        # use so it reverts by itself when the test ends; self.throttle_pct is
        # never written, so an aborted test cannot leave a setting behind.
        self.special_throttle_enable = C.SPECIAL_THROTTLE_ENABLE
        self.special_throttle_pct = C.SPECIAL_THROTTLE_PCT
        self.draw_roll_error = C.DRAW_ROLL_ERROR   # tuner switch: 0=off, 1=draw roll-error rays
        self.autotune_status = None    # live Controller-Autotune sweep progress for the HUD (str or None)
        self.autotune_test_t0 = None   # monotonic time the current test reached straight (per-test seconds counter), or None
        self.autotune_test_done_s = None  # frozen per-test elapsed seconds at the test's end, or None while it runs
        self.autotune_start_t = None   # monotonic start time of the running sweep (for the HUD MM:SS timer), or None
        # HUD inter-test "STRAIGHT" timer: monotonic start of the current FBWA level-out
        # between autotune trials (set by the sweep's _straighten), or None when not
        # straightening.  The snapshot turns it into elapsed seconds for the HUD.
        self.straighten_start_t = None
        # HUD "Path Aligned" timer: monotonic instant the flight path last entered
        # ALIGN_TOL_DEG of the balloon; counts up while held, resets when it leaves
        # tolerance (see _resolve_aligned_seconds).
        self._aligned_seconds_t0 = None
        # Controller-Autotune maneuver grid (tuner-configurable test aggressiveness):
        # the sweep tests every gain across the full pan x tilt cross-product.
        # Where a tuning run starts: 1 = reset to constants.py, 0 = keep the Gain
        # Tuner's values and anchor the brackets to them (see constants.py).
        self.autotune_bracket_wide = C.AUTOTUNE_BRACKET_WIDE
        self.autotune_use_constants = C.AUTOTUNE_USE_CONSTANTS
        self.cell_autotune_use_constants = C.CELL_AUTOTUNE_USE_CONSTANTS
        # 'y' single-cell autotune: the one bearing it tunes at (same frame as
        # the 'p' test; latitude NEGATIVE = nose-down).
        self.autotune_cell_lon_deg = C.AUTOTUNE_CELL_LON_DEG
        self.autotune_cell_lat_deg = C.AUTOTUNE_CELL_LAT_DEG
        self.autotune_pan_min = C.AUTOTUNE_PAN_MIN
        self.autotune_pan_max = C.AUTOTUNE_PAN_MAX
        self.autotune_pan_step = C.AUTOTUNE_PAN_STEP
        self.autotune_tilt_min = C.AUTOTUNE_TILT_MIN
        self.autotune_tilt_max = C.AUTOTUNE_TILT_MAX
        self.autotune_tilt_step = C.AUTOTUNE_TILT_STEP
        # Altitude protection floor / climb-to (metres AGL), shared by the sweep
        # ('t') and the plant-ID ('i') and live-editable in the Gain Tuner.
        # The flight code reads these attributes, never the constants directly, so
        # a tuner edit takes effect on the next climb check.
        self.autotune_protect_floor_m = C.AUTOTUNE_PROTECT_FLOOR_M
        self.autotune_protect_climb_to_m = C.AUTOTUNE_PROTECT_CLIMB_TO_M
        # Unified plant-ID axis switches: which axes the 'i' run includes. The
        # orchestrator reads these live, so they can be flipped before a run.
        self.plantid_do_pitch = C.PLANTID_DO_PITCH
        self.plantid_do_yaw = C.PLANTID_DO_YAW
        self.plantid_do_roll = C.PLANTID_DO_ROLL
        # Rate-PID aggressiveness knob read when a plant-ID report is generated:
        # scales the target rate-loop bandwidth that maps the plant to <AXIS>_RATE_P/_D.
        self.plantid_rate_agg = C.PLANTID_RATE_AGG
        # Trim map ('e' key): the climb-angle x airspeed grid it flies, read
        # when a run starts (and compared with a checkpoint's to resume it).
        self.trim_map_climb_min_deg = C.TRIM_MAP_CLIMB_MIN_DEG
        self.trim_map_climb_max_deg = C.TRIM_MAP_CLIMB_MAX_DEG
        self.trim_map_climb_step_deg = C.TRIM_MAP_CLIMB_STEP_DEG
        self.trim_map_speed_min_mps = C.TRIM_MAP_SPEED_MIN_MPS
        self.trim_map_speed_max_mps = C.TRIM_MAP_SPEED_MAX_MPS
        self.trim_map_speed_step_mps = C.TRIM_MAP_SPEED_STEP_MPS
        # Its own altitude floor / climb-to, read at every protection check.
        self.trim_map_protect_floor_m = C.TRIM_MAP_PROTECT_FLOOR_M
        self.trim_map_protect_climb_to_m = C.TRIM_MAP_PROTECT_CLIMB_TO_M
        # WHERE the 'p'-test balloon sits: spherical longitude (0 ahead, +90
        # right, 180 astern) / latitude (+90 up, 0 level, -90 down).  Position,
        # so not named pan/tilt - those are reserved for motion.  Not WGS84.
        self.point_lon_deg = C.POINT_LON_DEG
        self.point_lat_deg = C.POINT_LAT_DEG
        # HOW FAR along that bearing: min(range, ground distance), the single
        # distance rule - see constants.py.  inf (the default) means "as far as
        # this bearing goes": the ground on a descending bearing, true infinity
        # on a level or upward one.  This is the ONE knob that decides distance,
        # for moving and stationary balloons alike.
        self.point_balloon_range = C.POINT_BALLOON_RANGE
        # Moving-balloon motion (simple linear, constant speed).  MOVE_ENABLE is
        # the master on/off (off by default).  Motion only; it does NOT affect
        # where the balloon starts, and is ignored while the range is infinite.
        self.point_balloon_move_enable = C.POINT_BALLOON_MOVE_ENABLE
        self.point_balloon_speed = C.POINT_BALLOON_SPEED
        self.point_balloon_vel_pan = C.POINT_BALLOON_VEL_PAN
        self.point_balloon_vel_tilt = C.POINT_BALLOON_VEL_TILT
        self.arrow_left_deg = C.ARROW_LEFT_DEG
        self.arrow_right_deg = C.ARROW_RIGHT_DEG
        self.arrow_up_deg = C.ARROW_UP_DEG
        self.arrow_down_deg = C.ARROW_DOWN_DEG

        # -- Mode state --
        self.tracking = False
        self._tracking_transition = False
        self._straight_level = False
        self._mode = 'loiter'
        self._manual_desired_pitch = 0.0
        self._manual_desired_yaw = 0.0
        self._straight_desired_yaw = 0.0
        self.flight_complete = False

        # -- Balloon geometry --
        # The body-frame balloon this tick, stamped by _track_world_balloon.
        # C-02: encoder/oracle-derived, never FOV-derived - the scoring
        # accessors (flight_path_errors / alignment_errors) and the CSV read
        # this, so nothing that touches camera intrinsics may land here.
        self._last_balloon_body = (1.0, 0.0, 0.0)
        # DISPLAY ONLY.  The detection-derived (FOV-corrected) balloon used to
        # draw the green nose-camera marker, or None when there is no live
        # detection this tick (the HUD then falls back to _last_balloon_body).
        # C-02 permits FOV in rendering; it does NOT permit it in the balloon
        # geometry above, which is why these are two variables and not one.
        self._display_balloon_body = None
        # -- balloon range + closest-approach ("passed it") distance --
        # _balloon_range_m is None whenever the balloon has no finite position:
        # a direction anchor (at infinity) or real vision, which is bearing-only
        # and gives no range at all.  The HUD renders None as "Infinity".
        self._balloon_range_m = None
        self._pass_distance_m = None      # latched closest distance at closest approach
        self._pass_reported = False       # console line printed once per run
        # Last 3 (t, range^2, fix_key) for the vertex fit.  One entry per
        # POSITION FIX, not per control tick - see _update_balloon_range.
        self._rng_hist = []
        self._log_det_norm_x = 0.0
        self._log_det_norm_y = 0.0
        self._gimbal_mouse_x = nose_camera.width // 2 if hasattr(nose_camera, 'width') else 640
        self._gimbal_mouse_y = nose_camera.height // 2 if hasattr(nose_camera, 'height') else 360

        # -- ptest --
        self._point_injector = None
        self._ptest_abort = threading.Event()
        self._autotune_abort = threading.Event()
        # The run flying the 'autotune' regime ('t', 'y', 'i', 'e'; None when
        # none is), the abort Event it watches and its thread: set by
        # _launch_run, the key cleared when the thread ends - teardown, report
        # and hand-back included.
        self._run_key = None
        self._run_abort = None
        self._run_thread = None
        # A key switching from a run straight to another mode (_switch_from_run):
        # {'again', 'what', 't0', 'state'}, None when no switch is pending.  The
        # run's teardown and the control loop both act on it, so its state
        # changes only under the lock.
        self._switch = None
        self._switch_lock = threading.Lock()
        # GUIDE sequence number at the last ptest injection; see
        # cascade_has_run_since_inject.  None before the first injection.
        self._inject_guide_seq = None

        # -- Launch gate / gimbal calibration --
        # _launch_gate is OPEN by default so the flight sequence arms normally.
        # A pre-launch calibration (run() with calibrate_gimbal_on_startup) clears
        # it, runs the sweep, then sets it - holding the catapult until done.
        self._launch_gate = threading.Event()
        self._launch_gate.set()
        self.calibrate_gimbal_on_startup = False

        # -- Align-counter state machine (moved out of the HUD render) --
        self._align_seconds_t0 = None
        self._align_seconds_frozen = None
        self._align_aligned_t0 = None

        # -- Timing --
        self._control_dt = 1.0 / 50
        self._last_control_time = time.monotonic()
        self._last_heartbeat_time = 0.0

        # -- UI --
        self._csv = CsvLogger()
        # PER-PROCEDURE RUN STAMPS.  Keyed by procedure ('t', 'y', 'i', 'e'): the
        # stamp is taken when a procedure starts fresh and REPLACED by the
        # checkpoint's own stamp when one is resumed, so every file a procedure
        # writes - first attempt or fifth relaunch - carries the moment it
        # originally began.
        self._run_ts = {}
        self._csv_prev_mode = None
        self._pending_event = ''
        self._controller_autotune = ControllerAutotune(ControllerAutotuneStore(self))
        self._keyboard = KeyboardHandler(self, self._controller_autotune)


    # =====================================================================
    # World-balloon accessor
    # =====================================================================
    def _aircraft_world_pos_ned(self):
        return self.link.aircraft_world_pos_ned()

    # =====================================================================
    # Balloon range + closest-approach ("passed it") distance
    # =====================================================================
    def _update_balloon_range(self):
        """Per-tick: refresh _balloon_range_m and detect the moment the aircraft
        PASSED the balloon, latching the distance at that instant.

        WHAT "PASSED IT" MEANS
            Reaching it exactly needs two point objects to coincide, so it has probability zero.
            The well-defined substitute is the CLOSEST POINT OF APPROACH: where
            d(range)/dt changes sign, equivalently where relative position and
            relative velocity become perpendicular.  The range there is the miss
            distance.

        WHY THE PARABOLA FIT
            The true minimum falls BETWEEN two position fixes - five metres apart
            at 25 m/s and 5 Hz, orders above the closest distance - so the smallest
            SAMPLED range would overstate it and quantise with the telemetry rate.
            For constant relative velocity

                range^2(t) = |r0 + v_rel * t|^2

            is EXACTLY a quadratic in t, so three samples bracketing a minimum give
            the vertex analytically, independent of where the fixes landed.  Fit
            range^2, not range: range is a hyperbola and fits poorly at the vertex.

        WHAT COUNTS AS A SAMPLE
            One per FRESH AIRCRAFT POSITION, not per control tick - the rates
            differ 10x (50 Hz loop, 5 Hz GLOBAL_POSITION_INT) and re-reading an
            unchanged position is not a second observation.  See the gate below.

            SAMPLING PER TICK DEFEATS EVERY GUARD BELOW, silently and on the
            first fix of every run.  With a staircase position the window after
            any downward step holds (y0 high, y1 low, y2 = y1), because the
            frozen position makes the third sample equal the second.  That
            satisfies the bracket test (y1 < y0 and y1 <= y2, equality counting),
            gives a positive second difference y0 - y1, and puts the vertex at
            exactly x* = +0.5, inside the bracket.  Nothing rejects it: the fit
            latches a 'closest approach' about 0.2 s in, at essentially the full
            starting range, and since the latch is once-per-run the real closest
            approach can never be recorded afterwards.  It will also report
            passes on balloons that only ever close, and on balloons that only ever
            recede.

            The vertex arithmetic is not what fails here, so testing it harder
            proves nothing.  A test whose aircraft advances every tick models a
            50 Hz position source that does not exist, and will pass.

        _balloon_range_m refreshes every tick regardless, for the HUD and CSV.
        Only the FIRST closest approach of a run latches; a later restart is
        a different event.  Cleared on mode entry by _reset_to_idle.
        """
        # Read the fix stamp BEFORE the position it labels.  The telemetry thread
        # writes the stamp LAST, so this ordering means a fix landing mid-read
        # can only pair a NEW position with the PREVIOUS stamp - which the gate
        # below reads as "no new fix" and skips, picking it up correctly one tick
        # later.  The opposite order could pair a STALE position with a new
        # stamp, silently mistiming a sample the fit then trusts.
        t_fix = getattr(self.link, 'pos_time_boot_s', None)
        ac = self._aircraft_world_pos_ned()
        balloon = getattr(self.sensor, 'oracle_world_pos', None)
        if ac is None or balloon is None:
            # No finite balloon position -> no range.  Drop the history so a later
            # chase starts its own fit rather than bridging the gap.
            self._balloon_range_m = None
            self._rng_hist = []
            return

        vec = np.asarray(balloon, dtype=float) - np.asarray(ac, dtype=float)
        r2 = float(np.dot(vec, vec))
        self._balloon_range_m = math.sqrt(r2)

        if self._pass_distance_m is not None:
            return                                  # already latched this run

        # ---- sample ONLY on a genuinely fresh aircraft position -------------
        # This runs at 50 Hz but aircraft_world_pos_ned() comes from
        # GLOBAL_POSITION_INT at 5 Hz, so the position is a STAIRCASE: identical
        # for ~10 ticks, then a step.  Fed to the vertex fit that is fatal - on
        # the tick after any downward step the window is (y0=high, y1=low,
        # y2=y1), which satisfies every guard and forces x*=+0.5 exactly, so the
        # once-per-run latch fires on the FIRST GPS step at nearly full range and
        # the real closest approach can never be recorded.
        #
        # So sample once per FIX, not once per tick: between fixes there is no
        # new range information, and a repeated sample is the same observation
        # counted twice.  The fix identity rides IN the history entry, so
        # clearing _rng_hist clears the gate.  A stationary balloon is unaffected;
        # a moving one is read at the fix instant, at most 20 ms stale.
        #
        # TIME BASE: the fit is timed against the FC's own validity stamp
        # (GLOBAL_POSITION_INT.time_boot_ms), not arrival.  The FC emits on a
        # uniform schedule; arrival does not, and the loop adds 0-20 ms more, so
        # arrival timing would smear the true spacing by up to 10% straight into
        # the vertex.  Without a stamp, fall back to the loop clock and the
        # position value as identity - correct, just less precise.
        if t_fix is None:
            t_fix, key = time.monotonic(), np.asarray(ac, dtype=float).tobytes()
        else:
            t_fix, key = float(t_fix), float(t_fix)
        if self._rng_hist and key == self._rng_hist[-1][2]:
            return                                  # no new fix: nothing to add

        self._rng_hist.append((t_fix, r2, key))
        if len(self._rng_hist) > 3:
            self._rng_hist.pop(0)
        if len(self._rng_hist) < 3:
            return

        (t0, y0, _), (t1, y1, _), (t2, y2, _) = self._rng_hist
        # A minimum is bracketed only if the middle sample is the smallest, i.e.
        # the aircraft was closing and is now opening.  Strict on the way in so a
        # flat start cannot trigger it.
        if not (y1 < y0 and y1 <= y2):
            return
        # ---- exact vertex of the parabola through three NON-UNIFORM samples --
        # Position fixes arrive at a nominal 5 Hz over a link that jitters, so
        # the three samples are not equally spaced in time and the equally-
        # spaced closed form (which assumes x = -1, 0, +1) does not apply.
        # Fit in real time instead, via the Newton form
        #     y(t) = y0 + d01*(t - t0) + a*(t - t0)*(t - t1)
        # whose leading coefficient is the second divided difference.  On a
        # uniform grid this is algebraically identical to the closed form.
        h0, h1 = t1 - t0, t2 - t1
        if h0 <= 0.0 or h1 <= 0.0:                  # non-monotonic clock
            return
        d01 = (y1 - y0) / h0
        d12 = (y2 - y1) / h1
        a = (d12 - d01) / (t2 - t0)                 # leading coeff (m^2/s^2)
        if a <= 1e-9:                               # not convex / degenerate
            return
        t_star = 0.5 * (t0 + t1) - d01 / (2.0 * a)  # y'(t*) = 0
        if not (t0 < t_star < t2):                  # vertex outside the bracket
            return
        y_star = y0 + d01 * (t_star - t0) + a * (t_star - t0) * (t_star - t1)
        r_min = math.sqrt(max(0.0, y_star))

        self._pass_distance_m = r_min
        if not self._pass_reported:
            self._pass_reported = True
            log.info("=" * 60)
            log.info("[PASS] Balloon passed - closest approach %.3f m", r_min)
            log.info("  sampled range at the bracketing fixes: "
                     "%.3f / %.3f / %.3f m", math.sqrt(y0), math.sqrt(y1),
                     math.sqrt(y2))
            log.info("  fix spacing %.3f / %.3f s; vertex %+.4f s from the "
                     "middle fix", h0, h1, t_star - t1)
            log.info("=" * 60)

    @property
    def _balloon_world_dir(self):
        """The authoritative world-frame balloon.  Oracle (ptest) wins over the
        filtered estimate, exactly as the source _balloon_world_dir property."""
        if self.sensor.oracle_active:
            return self.sensor.oracle_world_dir(self._aircraft_world_pos_ned())
        return self._world_estimator.get()

    def _control_balloon_body_for_display(self, att):
        """Body-frame balloon the CONTROL PATH is tracking, for the orange HUD
        marker: the authoritative world-frame balloon (oracle, else the
        WorldBalloonEstimator output) rotated into the current body frame
        (world_vec = _balloon_world_dir; body_vec = R_body @ world_vec) - this
        side's estimate of the vector the FC's balloon chaser law flies.  Returns None
        when there is no estimate.

        DISPLAY ONLY: it does not feed anything back into control.  It trails the
        raw detection (green/yellow) by the estimator's filtering, and converges
        only at steady state."""
        wd = self._balloon_world_dir
        if wd is None:
            return None
        R_body = self._body_from_world_R(att)
        return R_body @ np.asarray(wd, dtype=float)

    # =====================================================================
    # Per-tick balloon geometry
    # =====================================================================
    def _effective_throttle_pct(self):
        """Throttle the aircraft should fly on this tick.

        SPECIAL_THROTTLE_PCT during a 'p' test when the override is enabled,
        the operator's throttle otherwise.  Resolved here rather than by writing
        self.throttle_pct, so leaving ptest - by finishing, aborting or crashing
        - restores the normal throttle with no cleanup step to forget.
        """
        if (getattr(self, 'special_throttle_enable', 0)
                and self._mode == 'ptest'):
            return float(np.clip(
                getattr(self, 'special_throttle_pct', 0.0), 0.0, 100.0))
        return self.throttle_pct

    def _body_from_world_R(self, att):
        """World->body rotation (v_body = R @ v_world).  Prefers the attitude
        QUATERNION, which is singularity-free at vertical pitch where the Euler
        (roll,yaw) gimbal-lock and make the body-frame balloon vector convulse.
        Falls back to the Euler matrix if no fresh quaternion is available (older
        telemetry, FC not streaming it).  Agrees with the Euler matrix to machine
        precision away from the singularity."""
        q = getattr(self.link, 'attitude_quat', None)
        q_t = getattr(self.link, 'attitude_quat_recv_t', 0.0)
        att_t = getattr(self.link, 'attitude_recv_t', 0.0)
        # Use the quaternion only if it is fresh relative to the Euler attitude
        # (within 0.5s of the latest ATTITUDE), else fall back.
        if q is not None and q_t > 0.0 and (att_t - q_t) < 0.5:
            return quat_to_rotation_matrix(float(q[0]), float(q[1]),
                                           float(q[2]), float(q[3]))
        return euler_to_rotation_matrix(*att)

    def _update_control(self):
        """Dispatch by mode."""
        att = self.link.attitude    # snapshot (telemetry thread replaces the ref atomically)
        dt = self._control_dt
        # Passive FOV observation runs before dispatch, whenever enabled, so it
        # can harvest motion in any mode.  It reads only; it changes nothing.
        if self.fov_est_enable:
            self._fov_estimator_tick(att)
        if not self.tracking:
            self._control_idle(att); return
        if self._straight_level:
            self._control_straight_level(att, dt); return
        if self._mode == 'manual':
            self._control_manual(att, dt); return
        self._control_balloon_chaser(att, dt)

    def _fov_estimator_tick(self, att):
        """Feed one tick to the passive FOV estimator (DIAGNOSTIC ONLY).

        Gathers the three tightly-coupled inputs read in THIS tick - the
        detection pixel, the gimbal encoder angles, and the aircraft attitude -
        and hands them to FovEstimator, which reads the FOV out of how they move
        together.  Reads only; commands nothing.  Skips the oracle path (ptest
        has no real pixel).  The camera-frame construction matches
        geometry.pan_tilt_to_vector: forward = [ct*cp, ct*sp, -st], a horizontal
        right axis, and up = forward x right."""
        sensor = self.sensor
        valid = (bool(getattr(sensor, 'active', False))
                 and not bool(getattr(sensor, 'oracle_active', False))
                 and bool(getattr(sensor, 'have_detection', False)))
        nx, ny = sensor.detection_norm()
        pan = math.radians(self.gimbal.measured_pan_deg)
        tilt = math.radians(self.gimbal.measured_tilt_deg)
        cp, sp = math.cos(pan), math.sin(pan)
        ct, st = math.cos(tilt), math.sin(tilt)
        fwd = np.array([ct * cp, ct * sp, -st])
        right = np.array([-sp, cp, 0.0])
        up = np.cross(fwd, right)
        R_gimbal_body = np.column_stack((right, up, fwd))   # cols: cam axes in body
        # euler_to_rotation_matrix is world->body; its transpose is body->world.
        R_ac_world = euler_to_rotation_matrix(att[0], att[1], att[2]).T
        self._fov_estimator.update(valid, nx, ny, R_ac_world, R_gimbal_body)

    def _track_world_balloon(self, world_vec, att):
        """Record where this tick's balloon sits relative to the aircraft, for
        the HUD and the scoring accessors.  Flies nothing: the FC runs the
        control law on the inputs GUIDE carries, and logs its own view of the
        same geometry (lua_balloon_body_*, los_elev_deg) in STATE2."""
        self._last_balloon_body = self._body_from_world_R(att) @ world_vec
        dnx, dny = self.sensor.detection_norm()
        self._log_det_norm_x = dnx
        self._log_det_norm_y = dny

    # =====================================================================
    # Mode helpers
    # =====================================================================
    def _nose_fy_scale(self):
        """Vertical focal-length scale (fy/fx) for the HUD nose-camera projection.

        DISPLAY ONLY.  The control path never calls this and stays FOV-agnostic.

        The Gazebo cameras render at a native 4:3 resolution (e.g. 320x240) but
        frames are resized to the 16:9 display (1280x720) before projection.  The
        resize stretches horizontal more than vertical, so the single focal length
        (correct for horizontal) over-scales the vertical.  The correction is the
        ratio of the per-axis stretch, which reduces to:
            fy_scale = native_aspect / display_aspect = (4/3)/(16/9) = 0.75
        Computed from the camera's reported native and display dimensions so it
        tracks the actual resolution rather than a hard-coded value.
        """
        from .ui.hud import NOSE_FY_SCALE
        ni = getattr(self, 'nose_intrinsics', None)
        cam = getattr(self, 'nose_camera', None)
        # Native dimensions: prefer the camera frame's, fall back to camera_info.
        nw = getattr(cam, 'native_w', None) or getattr(ni, 'native_w', None)
        nh = getattr(cam, 'native_h', None) or getattr(ni, 'native_h', None)
        dw = getattr(cam, 'width', None)
        dh = getattr(cam, 'height', None)
        try:
            if nw and nh and dw and dh and nh > 0 and dw > 0:
                native_aspect = nw / nh
                display_aspect = dw / dh
                if display_aspect > 0:
                    return native_aspect / display_aspect
        except (TypeError, ZeroDivisionError):
            pass
        return NOSE_FY_SCALE

    def _gimbal_fy_scale(self):
        """Vertical focal-length scale (fy/fx) for the HUD GIMBAL-camera debug
        projections - the gimbal-side analogue of _nose_fy_scale().

        DISPLAY ONLY.  The control path never calls this and stays FOV-agnostic.

        Like the nose camera, the Gazebo gimbal camera renders at a native 4:3
        resolution and is resized to the 16:9 display before the HUD projects
        onto it.  The gimbal-view debug overlays (nose marker, belly-canopy axis,
        balloon ray/marker) derive a single focal length from the display width
        and horizontal FOV, which is correct horizontally but over-scales the
        vertical axis; this returns the native_aspect/display_aspect correction
        that cancels the non-uniform resize.  Computed from the gimbal camera's
        reported native and display dimensions, falling back to the nominal 4:3
        ->16:9 ratio when a frame hasn't arrived yet.
        """
        from .ui.hud import NOSE_FY_SCALE
        cam = getattr(self, 'gimbal_camera', None)
        return aspect_fy_scale(
            getattr(cam, 'native_w', None), getattr(cam, 'native_h', None),
            getattr(cam, 'width', None), getattr(cam, 'height', None),
            fallback=NOSE_FY_SCALE)

    def _balloon_projection_live(self):
        """True when there is a genuine live balloon to PROJECT on the HUD:
        balloon chaser/ptest, manual and straight (the control law is tracking a real
        world-frame balloon in all of them - the flown balloon lives in
        _last_balloon_body), or vision tracking active (the gimbal is following a
        detection).  In plain loiter/idle - e.g. after just pressing 'g' to free
        the gimbal, with no 'l'/'x'/'p' and no manual/straight heading - there is
        no balloon, so the stale _last_balloon_body (initial straight-ahead, or a
        value left over from a previous lock) must NOT be drawn as a ray."""
        if self._mode in ('balloon_chaser', 'ptest', 'manual', 'straight'):
            return True
        return bool(getattr(self.sensor, 'active', False))

    def _refresh_vision_display_balloon(self, att):
        """When vision tracking ('l') is active, refresh the DISPLAY balloon from the
        image-processing detection (detection pixel + gimbal angles -> true balloon
        body direction) and keep the shared estimator current.  Updates only the
        cached body-frame balloon used by the HUD projections; it does NOT change
        any control output.  Safe to call from any non-balloon-chaser control path
        (idle/straight/manual).  No-op when vision tracking is off or an oracle is
        driving the sensor.
        """
        if not self.sensor.active or self.sensor.oracle_active:
            return
        R = self._body_from_world_R(att)
        meas = self.sensor.measurement()
        if meas is not None:
            body_meas, trust = meas
            world_meas = R.T @ body_meas
            # Keep the shared estimator warm so starting balloon chaser next is seeded.
            self._world_estimator.update(world_meas, trust, self._control_dt)
        # DISPLAY balloon from the image-processing detection (drawn YELLOW).  The
        # orange marker is the control-path balloon and is computed separately at
        # snapshot time (see _control_balloon_body_for_display); it is not derived
        # here.  This does not affect control.
        #
        # C-02: balloon_body_from_detection() adds a pixel->angle offset through
        # the camera FOV, so its result may drive RENDERING only.  It is kept in
        # _display_balloon_body and must never be written back into
        # _last_balloon_body, which the scoring accessors and the CSV read.
        # Cleared when the detection is lost so the HUD falls back to the flown
        # balloon rather than freezing on a stale detection.
        det_body = self.sensor.balloon_body_from_detection()
        self._display_balloon_body = (
            np.asarray(det_body, dtype=float) if det_body is not None else None)

    def _control_idle(self, att):
        # Vision tracking without balloon chaser: refresh the display balloon (control idle).
        self._refresh_vision_display_balloon(att)

    def _control_straight_level(self, att, dt):
        world_vec = yaw_pitch_to_world_vec(self._straight_desired_yaw, 0.0)
        self._track_world_balloon(world_vec, att)
        # _track_world_balloon set _last_balloon_body to the straight-ahead HEADING
        # (what we fly).  If vision tracking is active, override the DISPLAY balloon
        # with the image-processing result so the marker shows the real balloon
        # while control keeps flying the locked heading.
        self._refresh_vision_display_balloon(att)

    def _control_manual(self, att, dt):
        world_vec = yaw_pitch_to_world_vec(self._manual_desired_yaw, self._manual_desired_pitch)
        self._track_world_balloon(world_vec, att)
        # Vision tracking in manual: override the display balloon with the detection
        # (control still flies the manual stick commands).
        self._refresh_vision_display_balloon(att)

    def _control_balloon_chaser(self, att, dt):
        """Balloon Chaser/ptest: fold sensor measurement into the estimator (or snap to
        oracle), then track.

        NOTE: the sensor is pumped once per tick in run()'s main loop (matching
        the source, where _update_gimbal ran unconditionally), not here - so the
        gimbal tracks in every mode, not only during balloon chaser.  This method only
        folds the resulting measurement into the estimator.
        """
        pos = self._aircraft_world_pos_ned()

        if self.sensor.oracle_active:
            # Oracle path (ptest): snap the estimate to the exact balloon.
            wd = self.sensor.oracle_world_dir(pos)
            if wd is not None:
                self._world_estimator.snap_to(wd)
        else:
            # Live path: fold the sensor measurement into the trust-weighted
            # filter.  (set_last_world_dir for detection-lost reprojection is
            # handed over in the main loop before the pump.)
            meas = self.sensor.measurement()
            if meas is not None:
                body_meas, trust = meas
                R = self._body_from_world_R(att)
                world_meas = R.T @ body_meas
                self._world_estimator.update(world_meas, trust, dt)

        world_vec = self._balloon_world_dir
        if world_vec is None:
            return
        self._track_world_balloon(world_vec, att)
        # _track_world_balloon has just set _last_balloon_body to the FLOWN balloon.
        # The raw detection is recorded SEPARATELY, in _display_balloon_body, so
        # the GREEN nose marker shows the actual balloon instead of coinciding
        # with the yellow control marker and occluding it.
        #
        # C-02: the detection must NOT go into _last_balloon_body.
        # balloon_body_from_detection() derives its offset from the detection
        # pixel and the camera FOV, so writing it back would put FOV-derived
        # geometry into flight_path_errors() / alignment_errors() - the "x-test
        # truth" - and into the dbg_balloon_body_* / roll_align_err CSV columns,
        # for what is only a marker-occlusion concern.  Control is unaffected
        # either way (the FC runs on the encoder-derived vector), but the
        # ALIGNMENT readout diverges exactly when the gimbal lags, i.e. under
        # hard manoeuvring.  Oracle (ptest/sweep) has no detection pipeline, so
        # its estimator balloon stands.
        #
        # Inline rather than _refresh_vision_display_balloon: that helper also
        # folds a measurement into the estimator, which _control_balloon_chaser has
        # already done this tick, and would double the filter update.
        if not self.sensor.oracle_active:
            det_body = self.sensor.balloon_body_from_detection()
            self._display_balloon_body = (
                np.asarray(det_body, dtype=float) if det_body is not None else None)


    # =====================================================================
    # Mode transitions (called by KeyboardHandler)
    # =====================================================================
    def begin_run_stamp(self, key):
        """Start a fresh procedure: take a new stamp, discarding any previous one.

        Called when the operator launches the procedure.  If a checkpoint is
        then found and accepted, adopt_run_stamp replaces this with the one the
        procedure originally started under."""
        self._run_ts[key] = file_export.run_stamp()
        return self._run_ts[key]

    def adopt_run_stamp(self, key, ts):
        """Continue an earlier procedure under ITS stamp (resume)."""
        if ts:
            self._run_ts[key] = ts
        return self._run_ts.get(key)

    def run_stamp_for(self, key):
        """The stamp every file of this procedure is named with."""
        got = self._run_ts.get(key)
        if not got:
            got = self.begin_run_stamp(key)
        return got

    def set_gimbal_mouse(self, x, y):
        self._gimbal_mouse_x = x
        self._gimbal_mouse_y = y
        self.sensor.set_mouse(x, y)

    def handle_arrow(self, direction):
        """Arrow-key manual target nudge.  From a run in the 'autotune' regime
        it switches straight into manual (_switch_from_run); pressed again it
        nudges further - arrows never hand back to LOITER."""
        if self._switch_pending():
            return
        if self._tracking_transition:
            log.info("Transition in progress..."); return
        if not self.flight_complete:
            log.info("Still in flight sequence..."); return
        if self._switch_from_run('arrow', lambda: self.handle_arrow(direction),
                                 "manual flight (arrow keys)"):
            return
        if self._mode != 'manual':
            # Entering manual from another mode.  _reset_to_idle() clears
            # several pieces of state as part of its teardown - sensor-active,
            # gimbal-lock flag, world estimator.
            # But 'l' is an OVERLAY, not a mode: an arrow-key turn that happens
            # to enter manual must leave the lock and its tracking state EXACTLY
            # as they were before the arrow press - no flag drop, no estimator
            # re-converge transient.  Snapshot the lock-related state, run the
            # normal teardown, then restore.
            #
            # sensor.active alone does NOT mean 'l' vision tracking is on: 'p' and 'x'
            # activate the sensor themselves.  toggle_vision_tracking refuses to run
            # in those modes, so 'l' vision tracking is only possible OUTSIDE them, and
            # that is the test.  Without it, arrowing out of 'p' or 'x' restores
            # the sensor into manual flight and the vision markers stay on screen
            # in a mode that has no vision balloon.
            #
            # Snapshot only what the lock owns; everything else (_straight_level,
            # ptest_abort, point_injector, oracle, _manual_target_*) is correctly
            # reset by _reset_to_idle for manual entry.
            vision_was_active = (self.sensor.active
                               and self._mode not in ('balloon_chaser', 'ptest'))
            saved_gimbal_locked = self.sensor.gimbal_locked
            saved_estimate = self._world_estimator.get()
            self._reset_to_idle()
            self._mode = 'manual'
            if vision_was_active:
                self.sensor.set_active(True)
                self.sensor.set_gimbal_locked(saved_gimbal_locked)
                if saved_estimate is not None:
                    self._world_estimator.snap_to(saved_estimate)
            self._tracking_transition = True
            threading.Thread(target=self._begin_tracking, daemon=True, name="tracking-start").start()
        if direction == 'up':
            self._manual_desired_pitch = min(
                self._manual_desired_pitch + math.radians(self.arrow_up_deg), math.radians(85))
            self._pending_event = f'ARROW_UP_pitch={math.degrees(self._manual_desired_pitch):+.0f}'
        elif direction == 'down':
            self._manual_desired_pitch = max(
                self._manual_desired_pitch - math.radians(self.arrow_down_deg), math.radians(-85))
            self._pending_event = f'ARROW_DOWN_pitch={math.degrees(self._manual_desired_pitch):+.0f}'
        elif direction == 'right':
            desired = self._manual_desired_yaw + math.radians(self.arrow_right_deg)
            self._manual_desired_yaw = math.atan2(math.sin(desired), math.cos(desired))
            self._pending_event = f'ARROW_RIGHT_yaw={math.degrees(self._manual_desired_yaw):+.0f}'
        elif direction == 'left':
            desired = self._manual_desired_yaw - math.radians(self.arrow_left_deg)
            self._manual_desired_yaw = math.atan2(math.sin(desired), math.cos(desired))
            self._pending_event = f'ARROW_LEFT_yaw={math.degrees(self._manual_desired_yaw):+.0f}'

    def toggle_straight(self):
        """Toggle straight-flight mode ('s'): again goes to LOITER; from any
        other mode, a run included, straight there."""
        if self._switch_pending():
            return
        if self._tracking_transition:
            log.info("Transition in progress..."); return
        if not self.flight_complete:
            log.info("Still in flight sequence..."); return
        if self._mode == 'straight':
            self._reset_to_idle()
            self._mode = 'loiter'
            self._pending_event = 'KEY_S_OFF'
            self._tracking_transition = True
            threading.Thread(target=self._go_to_loiter, daemon=True, name="go-loiter").start()
        elif not self._switch_from_run('s', self.toggle_straight, "straight & level ('s')"):
            self._reset_to_idle()
            self._mode = 'straight'
            self._straight_level = True
            self._straight_desired_yaw = self.link.attitude[2]
            self._tracking_transition = True
            threading.Thread(target=self._begin_tracking, daemon=True, name="tracking-start").start()
            log.info("*** STRAIGHT & LEVEL - heading %.0f deg ***", math.degrees(self._straight_desired_yaw))
            self._pending_event = 'KEY_S'

    def toggle_balloon_chaser(self):
        """Toggle vision balloon chaser tracking ('x'): again goes to LOITER; from any
        other mode, a run included, straight there."""
        if self._switch_pending():
            return
        if self._tracking_transition:
            log.info("Transition in progress..."); return
        if not self.flight_complete:
            log.info("Still in flight sequence..."); return
        if self._mode == 'balloon_chaser':
            self._reset_to_idle()
            self._mode = 'loiter'
            self._pending_event = 'KEY_X_OFF'
            self._tracking_transition = True
            self._clear_align_counter()
            threading.Thread(target=self._go_to_loiter, daemon=True, name="go-loiter").start()
        elif not self._switch_from_run('x', self.toggle_balloon_chaser, "balloon chaser ('x')"):
            # Entering balloon chaser from another mode.  If the gimbal vision tracking ('l')
            # was held going in, its accumulated tracking state must survive the
            # transition: the vision tracking's filtered world estimate and the
            # gimbal-locked flag.  Without this, _reset_to_idle() wipes
            # the estimator and the first balloon chaser tick cold-snaps to a fresh
            # measurement, producing a small visible jump on both balloon markers.
            # Same snapshot/restore pattern as handle_arrow's manual-entry path.
            #
            # We restore ONLY the vision tracking's state.  Balloon Chaser's own
            # initial conditions (_pending_initial_phase, lock counter,
            # _begin_tracking) are balloon-chaser-entry bookkeeping, not part of the
            # vision tracking, and are set unconditionally below as before.
            #
            # Same test as handle_arrow: sensor.active is not evidence of 'l'
            # vision tracking, because 'p' sets it too.  Entering balloon chaser from ptest would
            # otherwise seed the vision estimator with the SYNTHETIC balloon's
            # world estimate and carry ptest's gimbal state in; a cold snap to
            # the first real measurement is the correct behaviour there.
            vision_was_active = (self.sensor.active
                               and self._mode not in ('balloon_chaser', 'ptest'))
            saved_gimbal_locked = self.sensor.gimbal_locked
            saved_estimate = self._world_estimator.get()
            self._reset_to_idle()
            self._mode = 'balloon_chaser'
            self.sensor.set_active(True)
            if vision_was_active:
                # 'l' vision tracking carried its gimbal state in (possibly 'g'-frozen);
                # preserve it exactly rather than force-unlocking.
                self.sensor.set_gimbal_locked(saved_gimbal_locked)
                if saved_estimate is not None:
                    self._world_estimator.snap_to(saved_estimate)
            else:
                # Fresh balloon chaser unlocks the gimbal so it slews to the balloon.
                # Explicit because activation and gimbal lock are decoupled:
                # ptest activates the sensor without unlocking.
                self.sensor.set_gimbal_locked(False)
            self._tracking_transition = True
            self._start_align_counter()
            threading.Thread(target=self._begin_tracking, daemon=True, name="tracking-start").start()
            log.info("*** BALLOON CHASER ***")
            self._pending_event = 'KEY_X'

    # =====================================================================
    # Runs in the 'autotune' regime, and switching straight out of one
    # =====================================================================
    def _launch_run(self, key, name, run_fn, abort, *args):
        """Start a run in the 'autotune' regime on its own thread, as
        run_fn(self, abort, *args).  _run_key names it until the thread has
        ended - teardown and report included - which is what a switch to
        another mode waits for; _run_abort is the Event that stops it."""
        def body():
            try:
                run_fn(self, abort, *args)
            finally:
                if self._run_thread is thread:
                    self._run_key = None
        thread = threading.Thread(target=body, daemon=True, name=name)
        self._run_key, self._run_abort, self._run_thread = key, abort, thread
        thread.start()

    def _run_key_pressed(self, key, again, what, stop_msg):
        """The start of every run key ('t', 'y', 'i', 'e').  True when the press
        is dealt with here: refused while a switch is pending or a transition
        is under way, this run's own key stopping it (it hands back to LOITER),
        or a switch away from another run.  False: the caller starts its run.
        Its own key is checked before the transition guard, so it stops the
        run even while the run is starting."""
        if self._switch_pending():
            return True
        if self._run_key == key:
            self._run_abort.set()
            log.info(stop_msg)
            return True
        if self._tracking_transition:
            log.info("Transition in progress - cannot start %s", what)
            return True
        return self._switch_from_run(key, again, what)

    def _switch_pending(self):
        """True, after saying so, while a switch waits for its run to end: a
        key pressed then would race the mode already on its way."""
        sw = self._switch
        if sw is None:
            return False
        log.info("Switching to %s - wait for it to start", sw['what'])
        return True

    def _refuse_during_run(self, what):
        """True, after saying so, while a run in the 'autotune' regime is
        flying - for what is not a mode of its own and cannot switch from one
        ('l').  The run's key stops it first."""
        if self._run_key is None:
            return False
        log.info("%s is flying - stop it ('%s') before %s",
                 _AUTOTUNE_RUNS[self._run_key], self._run_key, what)
        return True

    def _switch_from_run(self, key, again, what):
        """A mode key pressed while a run in the 'autotune' regime is flying:
        go to `what` directly, without LOITER in between.  False when no run is
        up (the caller starts `what` itself); True when the switch is pending.

        The run is stopped with its own abort, checkpoint kept, as its key
        would stop it.  The switch is then settled in two places, once each:
          - the run's teardown (autotune_finish, on the run's thread) hands the
            aircraft over in ACRO rather than LOITER (_hand_over);
          - the control loop (_service_switch, on this thread - the keys' own)
            presses `again` once the run's thread has ended, which finds no run
            and starts `what`.
        Starting the new mode before the thread has ended puts a second driver
        beside the run's teardown; starting it off this thread races the keys
        and the tick."""
        run = self._run_key
        if run is None:
            return False
        log.info("*** %s: stopping %s (its checkpoint is kept) - no LOITER in "
                 "between ***", what, _AUTOTUNE_RUNS[run])
        self._pending_event = 'KEY_%s_SWITCH_FROM_%s' % (key.upper(), run.upper())
        with self._switch_lock:
            self._switch = {'again': again, 'what': what,
                            't0': time.monotonic(), 'state': 'stopping'}
        self._run_abort.set()
        return True

    def _hand_over(self, sw):
        """Release a stopped run's aircraft to the mode its switch starts, in
        go_to_loiter's order but with ACRO for LOITER: tracking off; the sticks
        centred at the operator's throttle, because a level-out's or a doublet's
        override must not fly on and ACRO reads a deflected stick as a rate
        command; ACRO commanded and confirmed with the sticks held; then the
        sticks released.  Every mode a switch starts either flies ACRO or passes
        through it.  ACRO refused: the aircraft is handed back to LOITER and
        the switch cancelled."""
        self.tracking = False
        self._mode_label = None
        thr = max(0.0, min(1.0, float(self.throttle_pct) / 100.0))
        def hold():
            self.link.hold_level_sticks(thr)
        try:
            hold()
            self.enter_acro('switch to %s' % sw['what'], hold=hold)
        except Exception:
            log.exception("[MODE] ACRO refused for the switch to %s - handing back "
                          "to LOITER", sw['what'])
            with self._switch_lock:
                sw['state'] = 'handed_back'
            self.go_to_loiter('switch to %s refused' % sw['what'])
            return
        self._release_rc_overrides()
        with self._switch_lock:
            sw['state'] = 'handed_over'
        log.info("[MODE] handed over in ACRO, RC overrides released - %s starts "
                 "once the run has ended", sw['what'])

    def _service_switch(self):
        """Settle a pending switch - once per tick, on the control thread.

        Once the run's thread has ended, press the key again (`again`) - unless
        its teardown handed back to LOITER, which cancels the switch.  A run
        still up after _SWITCH_WAIT_S: if its teardown has not begun, the switch
        is dropped and the teardown hands back to LOITER when it comes; if it
        has handed over, the aircraft - in ACRO with nothing flying it - is
        handed back now.  A mode that then does not start is handed back too.
        Never blocks: the hand-backs run on their own thread."""
        sw = self._switch
        if sw is None:
            return
        if self._run_key is not None:
            if time.monotonic() - sw['t0'] < _SWITCH_WAIT_S:
                return
            with self._switch_lock:
                state = sw['state']
                if state == 'handing_over':
                    return                  # its ACRO entry has its own timeout
                self._switch = None
            log.error("[MODE] %s did not end within %.0fs - %s NOT started",
                      _AUTOTUNE_RUNS.get(self._run_key, "the run"),
                      _SWITCH_WAIT_S, sw['what'])
            if state == 'handed_over':
                self._hand_back_async('switch to %s timed out' % sw['what'])
            return
        with self._switch_lock:
            state = sw['state']
            self._switch = None
        if state == 'handed_back':
            log.info("[MODE] switch to %s cancelled - the aircraft is in LOITER",
                     sw['what'])
            return
        sw['again']()
        if self._mode == 'autotune' and self._run_key is None:
            log.error("[MODE] %s did not start - handing back to LOITER", sw['what'])
            self._hand_back_async('switch to %s did not start' % sw['what'])

    def _cancel_switch(self):
        """At shutdown: no mode may start after the exit hand-back.  A teardown
        already entering ACRO for the switch is waited for, so the hand-back
        that follows is the last mode command."""
        with self._switch_lock:
            sw, self._switch = self._switch, None
        if (sw is not None and sw['state'] == 'handing_over'
                and self._run_thread is not None):
            self._run_thread.join(_SWITCH_WAIT_S)

    def _hand_back_async(self, reason):
        """go_to_loiter on its own thread, for the control loop."""
        self._tracking_transition = True
        threading.Thread(target=self._go_to_loiter, args=(reason,),
                         daemon=True, name="go-loiter").start()

    def toggle_ptest(self):
        """Toggle the point-balloon-chaser spec test ('p'): again goes to LOITER; from
        any other mode, a run included, straight there."""
        if self._switch_pending():
            return
        if self._tracking_transition:
            log.info("Transition in progress - cannot start point balloon chaser test"); return
        if self._mode == 'ptest':
            self._reset_to_idle()
            self._mode = 'loiter'
            self._pending_event = 'KEY_P_OFF'
            self._tracking_transition = True
            self._clear_align_counter()
            threading.Thread(target=self._go_to_loiter, daemon=True, name="go-loiter").start()
        elif not self._switch_from_run('p', self.toggle_ptest,
                                       "the point-balloon-chaser test ('p')"):
            self._reset_to_idle()
            self._mode = 'ptest'
            self._ptest_abort.clear()
            self._tracking_transition = True
            self._pending_event = 'KEY_P_START'
            self._start_align_counter()
            threading.Thread(target=run_point_balloon_chaser_test, args=(self, self._ptest_abort),
                             daemon=True, name="point-balloon-chaser-test").start()
            log.info("*** POINT BALLOON CHASER TEST STARTED - press 'p' to abort ***")

    def toggle_autotune(self):
        """'t' key: start/stop the automatic Controller Autotune gain sweep.
        Mirrors toggle_ptest: spawns the sweep on a daemon thread while the run
        loop keeps ticking.  Pressing 't' again sets the abort Event; the sweep
        thread restores the in-progress gain and calls autotune_finish() on exit.
        Intended for SIM - it drives gains to the edge of overshoot on purpose."""
        if self._run_key_pressed('t', self.toggle_autotune, "controller autotune ('t')",
                                 "*** CONTROLLER AUTOTUNE - abort requested ***"):
            return
        self._reset_to_idle()
        self._mode = 'autotune'
        # A FRESH stamp for this launch.  If a checkpoint is then found and
        # accepted, _load_checkpoint replaces it with the stamp the
        # procedure originally began under, so a resume keeps one identity.
        self.begin_run_stamp('t')
        self._autotune_abort.clear()
        self._tracking_transition = True   # first trial's ptest_enter_acro clears it
        self._pending_event = 'KEY_T_AUTOTUNE'
        self._launch_run('t', "controller-autotune", run_controller_autotune_sweep,
                         self._autotune_abort)

    def toggle_cell_autotune(self):
        """'y' key: start/stop a SINGLE-CELL autotune at (autotune_cell_lon_deg,
        autotune_cell_lat_deg).

        The same engine as 't' - same SWEEP_PLAN, same coordinate descent, same
        cost, same climb protection, same per-trial checkpointing and resume,
        same live application of winners - flown at one operator-chosen bearing
        instead of the grid.  The result is the best achievable AT THAT BEARING
        and carries no envelope guarantee, which is what makes it the right tool
        for "what is wrong at this cell" and the wrong one for shipping a gain.

        Shares mode 'autotune' with 't' and 'i' so the HUD timers and banner work
        unchanged, and the abort Event with 't'.  'y' again stops it; 't', or any
        other mode key, switches straight to its own mode (_switch_from_run).
        Its checkpoint and report files are separate (CELL_PROFILE), because a
        single-cell result matrix must never resume a grid sweep."""
        if self._run_key_pressed('y', self.toggle_cell_autotune,
                                 "single-cell autotune ('y')",
                                 "*** SINGLE-CELL AUTOTUNE - abort requested ***"):
            return
        # Say WHICH bearing this is about to fly, and name the Gain Tuner rows
        # it came from.  The tuner carries two operator-set lat/lon pairs -
        # these and the 'p' test's - so an unset cell silently flying the
        # default is the easiest mistake to make here, and the cheapest to
        # catch: this line is the first thing printed after the keypress.
        _lon = float(self.autotune_cell_lon_deg)
        _lat = float(self.autotune_cell_lat_deg)
        log.info("*** SINGLE-CELL AUTOTUNE 'y' at longitude %+.1f, latitude %+.1f "
                 "(Gain Tuner rows \"Cell Longitude/Latitude deg\") ***",
                 _lon, _lat)
        if (_lon == C.AUTOTUNE_CELL_LON_DEG and _lat == C.AUTOTUNE_CELL_LAT_DEG):
            log.warning("[CATUNE] the 'y' cell is still at its constants.py "
                        "default (%+.1f, %+.1f).  If you meant a different "
                        "bearing, set the \"Cell\" rows - NOT the \"Injected "
                        "Balloon\" rows, which only aim the 'p' test.",
                        _lon, _lat)
        self._reset_to_idle()
        self._mode = 'autotune'
        # A FRESH stamp for this launch.  If a checkpoint is then found and
        # accepted, _load_checkpoint replaces it with the stamp the
        # procedure originally began under, so a resume keeps one identity.
        self.begin_run_stamp('y')
        self._autotune_abort.clear()
        self._tracking_transition = True   # first trial's ptest_enter_acro clears it
        self._pending_event = 'KEY_Y_CELL_AUTOTUNE'
        self._launch_run('y', "cell-autotune", run_controller_autotune_sweep,
                         self._autotune_abort, CELL_PROFILE)

    def toggle_plant_id(self):
        """'i' key: start/stop the UNIFIED plant-ID (plant_id.py orchestrator).
        Runs whichever axes the Gain-Tuner Do Pitch / Do Yaw / Do Roll switches
        enable, as sequential self-contained stages sharing altitude protection
        and a top-level checkpoint (a second 'i', or a crash, resumes at the next
        unfinished axis).  All three axes are the same open-loop MANUAL doublet
        measurement (elevator, rudder, aileron).  Nothing is applied automatically
        - each stage writes its own artefacts for HUMAN review.  Reuses the sweep's
        proven 'autotune' mode regime.  SIM ONLY until validated."""
        if self._run_key_pressed('i', self.toggle_plant_id, "plant ID ('i')",
                                 "*** PLANT ID - abort requested ***"):
            return
        from .plant_id import run_plant_id            # the unified orchestrator
        self._reset_to_idle()
        self._mode = 'autotune'            # reuse the sweep's proven mode regime
        self._mode_label = 'Plant ID'      # ...but say so on the HUD
        self.begin_run_stamp('i')          # fresh; a resume adopts the checkpoint's
        self._tracking_transition = True   # first ptest_enter_acro clears it
        self._pending_event = 'KEY_I_PLANTID'
        self._launch_run('i', "plant-id", run_plant_id, threading.Event())

    def toggle_freq_id(self):
        """'k' key: the roll-axis frequency-response measurement (plant_freq_id.py).

        REFUSED.  Its excitation is a roll-rate perturbation added to the roll
        law's output, and the roll law runs on the FC: nothing on this side can
        add to it.  A run would fly no excitation at all and fit the loop's own
        noise as the plant.  It stays refused until the excitation is carried to
        the FC (EXCITE)."""
        log.error("[FREQ-ID] 'k' is not available: the roll law runs on the FC "
                  "and freq-ID cannot inject its excitation there yet - nothing "
                  "was started")

    def toggle_trim_map(self):
        """'e' key: start/stop the trim map (trim_map.py) - the elevator the
        airframe needs to hold its attitude, measured over a climb-angle x
        airspeed grid for the pitch-trim formula.  Pressing 'e' again stops it;
        the checkpoint keeps every flown cell and the next 'e' resumes.  Runs
        the sweep's 'autotune' mode regime under the HUD label "Trim Map".
        Nothing is applied: the report suggests, the operator decides.  SIM
        ONLY until validated."""
        if self._run_key_pressed('e', self.toggle_trim_map, "the trim map ('e')",
                                 "*** TRIM MAP - stop requested ***"):
            return
        from .trim_map import run_trim_map
        self._reset_to_idle()
        self._mode = 'autotune'            # the sweep's proven mode regime
        self._mode_label = 'Trim Map'      # ...but say so on the HUD
        self.begin_run_stamp('e')          # fresh; a resume adopts the checkpoint's
        self._tracking_transition = True   # the run clears it once underway
        self._pending_event = 'KEY_E_TRIM_MAP'
        self._launch_run('e', "trim-map", run_trim_map, threading.Event())

    # Pitch and yaw open-loop measurements are stages of the unified plant-ID
    # ('i' key, toggle_plant_id above), gated by the Gain-Tuner Do Pitch / Do Yaw
    # switches - not separate keys.

    def toggle_gimbal_lock(self):
        """Toggle the gimbal-lock hold ('g')."""
        locked = not self.sensor.gimbal_locked
        self.sensor.set_gimbal_locked(locked)
        state = "LOCKED" if locked else "FREE"
        log.info("*** Gimbal %s ***", state)
        self._pending_event = f'KEY_G_{state}'

    def toggle_vision_tracking(self):
        """Toggle the gimbal vision tracking ('l')."""
        if self._mode in ('balloon_chaser', 'ptest'):
            log.info("Gimbal vision tracking not available in balloon chaser or spec-test mode"); return
        # A run switches the sensor on for its own balloon, as 'p' and 'x' do;
        # toggling it here would drop that balloon mid-step.
        if self._switch_pending() or self._refuse_during_run("the vision tracking ('l')"):
            return
        active = not self.sensor.active
        self.sensor.set_active(active, manual_search=active)
        if active:
            # 'l' arms a manual-search vision tracking: the gimbal follows the operator's
            # mouse until vision first detects the chevron, then vision owns it
            # (holds/tracks the balloon) until 'l' OFF or a mode change.  set_active
            # does not unlock the gimbal, so unlock it explicitly for the search
            # slew.  ('l' OFF leaves the gimbal as-is, exactly as before - 'g'
            # re-locks it.)
            self.sensor.set_gimbal_locked(False)
            log.info("*** Gimbal vision tracking ON ***")
            self._pending_event = 'KEY_L_ON'
        else:
            log.info("*** Gimbal vision tracking OFF ***")
            self._pending_event = 'KEY_L_OFF'

    def toggle_csv_logging(self):
        """Start or stop flight-log CSV recording ('q')."""
        if self._csv.active:
            self._csv.stop()
            log.info("*** logging STOPPED -> %s (%d ticks) ***",
                     self._csv.path, self._csv.tick_count)
        else:
            # Stamp the FC firmware identity into the head so the flight log
            # names the build it was flown against, beside the companion's
            # code_sha256.  Absent on the RC-override link, which has no Lua
            # banner to capture - getattr keeps that path silent.
            notes = []
            fw = getattr(self.link, 'firmware_banner_lines', None)
            if callable(fw):
                notes.extend(fw())
            # Capture camera native resolutions into the log header so the
            # nose-vs-gimbal resolution check travels with the data file.
            for label, cam in (('nose', self.nose_camera), ('gimbal', self.gimbal_camera)):
                nw = getattr(cam, 'native_w', None); nh = getattr(cam, 'native_h', None)
                dw = getattr(cam, 'width', None); dh = getattr(cam, 'height', None)
                if nw and nh:
                    flag = '' if (nw == dw and nh == dh) else ' STRETCHED'
                    notes.append(f"{label} native {nw}x{nh} display {dw}x{dh}{flag}")
                else:
                    notes.append(f"{label} native UNKNOWN (no frame yet) display {dw}x{dh}")
            ni = getattr(self, 'nose_intrinsics', None)
            if ni is not None and getattr(ni, 'have_intrinsics', False):
                notes.append(f"nose caminfo fx={ni.fx:.1f} fy={ni.fy:.1f} "
                             f"native {getattr(ni,'native_w',None)}x{getattr(ni,'native_h',None)}")
            self._csv.header_notes = notes
            self._csv.start()
            self._csv_prev_mode = None
            log.info("*** logging STARTED -> %s ***", self._csv.path)

    def reset_gimbal_center(self):
        """'r' key: recenter the gimbal to (0,0), and re-center the operator MOUSE
        joystick.  When the gimbal is FREE the mouse is a RATE joystick: an
        off-centre mouse continuously slews the gimbal in that direction (so a
        bounded input can drive the full pan range).  Re-centering the mouse here
        zeroes that rate command so the gimbal holds centre after the reset."""
        self.gimbal.reset()
        gw = getattr(self.gimbal_camera, 'width', 1280)
        gh = getattr(self.gimbal_camera, 'height', 720)
        self.set_gimbal_mouse(gw // 2, gh // 2)   # neutralise the rate joystick
        log.info("*** Gimbal reset to center ***")
        self._pending_event = 'KEY_R_GIMBAL_CENTER'

    def run_gimbal_calibration(self):
        """'c' key: sweep the gimbal through a known (pan,tilt) grid and record
        the gimbal_cam_link quaternion at each settled point, writing
        gimbal_calib.csv.  Runs on a background thread using THIS controller's
        gimbal driver (the sole publisher).  Balloon Chaser/ptest should be off.
        Delegates to Companion.gimbal_calibration."""
        gimbal_calibration.start_calibration(self.gimbal)

    # =====================================================================
    # ptest hooks (called by run_point_balloon_chaser_test)
    # =====================================================================
    def steering_errors(self):
        """(pan, tilt) deg steering error, as the FC computed it this tick; each
        None on a tick the law did not run.  The FC's view through its own
        sensor path - NOT the alignment measure, which is alignment_errors()."""
        return (_fc(getattr(self.link, 'lua_steering_pan', None)),
                _fc(getattr(self.link, 'lua_steering_tilt', None)))

    def flight_path_errors(self):
        """(pan, tilt) deg angular error of the FLIGHT-PATH (velocity) vector vs
        the balloon - the SAME quantity the HUD 'Path Aligned' uses
        (tg_pan-fp_pan, tg_tilt-fp_tilt).  This is the cyan-marker-on-balloon
        error, which the laws drive to zero.  None when the flight path is
        undefined (no FC velocity, or speed at or below
        FLIGHT_PATH_MIN_SPEED_MPS)."""
        att = self.link.attitude
        fpb = self._flight_path_body_vec(att)
        if fpb is None:
            return None
        fp_pan, fp_tilt = vector_to_pan_tilt(fpb)
        tg_pan, tg_tilt = vector_to_pan_tilt(np.asarray(self._last_balloon_body, dtype=float))
        return (tg_pan - fp_pan, tg_tilt - fp_tilt)

    def alignment_errors(self):
        """THE alignment measure: the (pan, tilt) deg error of the FLIGHT PATH
        against the balloon, from the companion's own balloon geometry (truth
        under 'p') and the FC's velocity.  Everything that asks "aligned?"
        reads this, with hypot(pan, tilt) < ALIGN_TOL_DEG: the 'p' verdict, the
        lock-seconds counter, the HUD 'Path Aligned' indicator and timer, the
        sweep's lock gate and the log's in_tol column.  None if undefined."""
        return self.flight_path_errors()

    def aligned_now(self):
        """True while tracking and the flight path is within ALIGN_TOL_DEG of
        the balloon - the one criterion every "aligned?" check applies."""
        err = self.alignment_errors()
        return (self.tracking and err is not None
                and math.hypot(err[0], err[1]) < C.ALIGN_TOL_DEG)

    def cascade_has_run_since_inject(self):
        """True once the FC has answered a GUIDE frame sent after the last ptest
        injection with a commanded tick (lua_gate 0).  The point-balloon-chaser test
        waits on this so steering_errors() is read for THIS balloon, not the
        zeros - or the previous balloon - held before the injection."""
        seq0 = self._inject_guide_seq
        if seq0 is None:
            return False
        # STATE echoes the GUIDE seq it was computed from, so an answer to a
        # frame sent after the injection is strictly ahead of seq0 (mod 2^16).
        ahead = (int(getattr(self.link, 'lua_seq', 0)) - seq0) & 0xFFFF
        return 0 < ahead < 0x8000 and int(getattr(self.link, 'lua_gate', -1)) == 0

    def set_point_injector(self, inj):
        self._point_injector = inj

    @property
    def mode(self):
        return self._mode

    @property
    def roll_saturated(self):
        """True while the FC's roll command is against the bank limit this tick.

        A saturated tick is momentarily open-loop: the command is the limit, not
        the law's output.  The single-cell autotune ('y') samples this to decide
        whether a cell can be tuned at all - see AUTOTUNE_CELL_MAX_SAT_FRAC."""
        return bool(getattr(self.link, 'lua_roll_sat', False))

    # =====================================================================
    # Mode transfer - THE two canonical paths.  Everything that changes the
    # FC's flight mode goes through one of these; nothing calls
    # link.set_mode_* directly.  Conformity here is not cosmetic: a finisher
    # that sets the INTERNAL label to 'loiter' without commanding the FC leaves
    # the aircraft flying in ACRO while the controller believes it has handed
    # back.
    # =====================================================================
    def enter_acro(self, reason='', hold=None):
        """Put the FC in ACRO - "the controller is flying it".

        Commands and CONFIRMS (link.set_mode_acro blocks until HEARTBEAT agrees
        and raises TimeoutError after 30 s).  The exception PROPAGATES: a
        persistent refusal must stop the caller loudly rather than let it track
        in a mode the script will not claim - the aircraft would fly that mode,
        not the balloon chaser, while the controller believes it is flying.  `hold`
        re-sends a caller's stick overrides through the rate-loop write, as
        link._set_mode describes.

        BLOCKING (up to 30 s).  Never call from the 50 Hz control thread.
        """
        self.link.set_mode_acro(hold=hold)
        if reason:
            log.info("[MODE] ACRO active (%s)", reason)
        return True

    def go_to_loiter(self, reason='', keep_operator_params=False):
        """Hand the aircraft back to the autopilot - the ONLY way to stop
        flying it.

        Order matters, and so does the finally:
          1. tracking off FIRST: the next GUIDE frame says so and the script
             releases control on that tick, before anything else changes.
          2. LOITER commanded and confirmed against HEARTBEAT, then the rate
             loop LOITER flies written (NATIVE_MODE_RATE_PIDS), or the
             operator's own values with keep_operator_params - the exit
             hand-back, so the FC is left holding them for the next startup.
          3. RC overrides (the level-out's) released in a
             FINALLY, so they are released even when the mode change times out
             or raises.  A latched override with no mode change leaves the FC
             honouring the last stick values with nothing commanding it.
        The internal label is set regardless, because it means "the controller
        is no longer flying"; that is true even if the FC refused the mode.  A
        refusal is logged at ERROR, not swallowed.

        BLOCKING (up to 30 s).  Never call from the 50 Hz control thread - the
        ptest / plant-ID / sweep teardowns all run on their own worker threads,
        and the keyboard path uses the _go_to_loiter() wrapper below.
        """
        self.tracking = False
        self._mode_label = None          # display override ends with the run
        ok = False
        try:
            self.link.set_mode_loiter(keep_operator_params=keep_operator_params)
            ok = True
        except Exception:
            log.exception("[MODE] LOITER was REFUSED or timed out (%s) - the FC may "
                          "still be in ACRO; releasing RC overrides anyway", reason or '-')
        finally:
            self._release_rc_overrides()
            self._mode = 'loiter'
        if ok:
            log.info("[MODE] LOITER active, RC overrides released%s",
                     (" (%s)" % reason) if reason else "")
        return ok

    def _release_rc_overrides(self):
        """Release every RC override, logging - never raising - on failure."""
        try:
            self.link.release_rc_overrides()
        except Exception:
            log.exception("[MODE] FAILED to release RC overrides - the FC may still "
                          "be honouring the last stick values")

    def ptest_enter_acro(self):
        """Enter ACRO, keep tracking off until injection.  Returns True on
        success."""
        self.tracking = False
        self._straight_level = False
        self.sensor.set_active(False)
        try:
            self.enter_acro('ptest/sweep entry')
        except Exception:
            log.exception("[MODE] could not enter ACRO for ptest/sweep")
            return False
        finally:
            self._tracking_transition = False
        return True

    def ptest_begin(self, lon_deg, lat_deg):
        """Arm tracking after the injector is live.

        lon_deg/lat_deg are the balloon's anchored longitude/latitude.  Both are
        accepted and deliberately unused: the injector has already anchored the
        balloon and GUIDE carries the bearing to the FC through the oracle, so
        this hook only resets state.  They stay in the signature because every caller
        has the pair to hand and dropping them would hide which balloon a reset
        belongs to in a stack trace."""
        # Steering read before the FC answers a frame carrying this injection
        # belongs to the previous balloon; see cascade_has_run_since_inject.
        self._inject_guide_seq = int(getattr(self.link, 'guide_seq', 0))
        # Re-arm the closest-approach report.  An injection IS a new run:
        # _pass_reported is a once-per-RUN latch, and without this a second
        # injection with no drop to idle in between passes the balloon with no
        # console line while the HUD and CSV still update.
        self._pass_reported = False
        self._pass_distance_m = None
        self._rng_hist = []
        self.sensor.set_active(True)
        self._pending_event = 'PTEST_INJECT'
        self.tracking = True

    def ptest_finish(self):
        """Restore to loiter if still in ptest.

        SETTING self._mode IS NOT FINISHING.  The label is what the HUD renders
        and nothing else; it commands no mode change and clears no tracking.  A
        finisher that only sets it leaves GUIDE saying "tracking", so the script
        keeps flying the balloon chaser in ACRO - while the HUD reads "Mode: Loiter".
        The two agree on screen and disagree in the air, which is why it
        survives a visual check.

        Every exit path must COMMAND LOITER, CONFIRM it, and release the
        overrides in a `finally` so they are released even when the mode change
        times out.  A docstring promising the restore is not the restore.
        """
        if self._point_injector is not None:
            self._point_injector.stop()
            self._point_injector = None
        if self._mode == 'ptest':
            self._straight_level = False
            self.sensor.set_active(False)
            # Clears tracking, commands the FC and releases the sticks.
            # Setting _mode alone leaves the script flying the balloon chaser.
            self.go_to_loiter('ptest finish')
        self._tracking_transition = False

    # =====================================================================
    # Controller-autotune hooks (called by run_controller_autotune_sweep)
    # =====================================================================
    def autotune_pause_tracking(self):
        """Between autotune trials: drop the injector, stop tracking, and
        deactivate the sensor so the aircraft holds attitude (no chase of a
        cleared balloon) during the inter-trial settle."""
        if self._point_injector is not None:
            self._point_injector.stop()
            self._point_injector = None
        self.tracking = False
        self.sensor.set_active(False)

    def autotune_finish(self):
        """End a run in the 'autotune' regime: stop injector, tracking off, and
        hand back to LOITER if still in autotune mode - or, when a key is
        switching straight to another mode, hand the aircraft over to it in
        ACRO (_hand_over).  Mirrors ptest_finish for the 'autotune' mode.

        A teardown may call this twice on one stop (the sweep does: its abort
        branch, then its finally).  Once handed over, _mode stays 'autotune'
        until the new mode starts, so the second call must see the switch
        already handed over and leave the aircraft alone - a LOITER there
        would undo the hand-over."""
        if self._point_injector is not None:
            self._point_injector.stop()
            self._point_injector = None
        if self._mode == 'autotune':
            self._straight_level = False
            self.sensor.set_active(False)
            with self._switch_lock:
                sw = self._switch
                state = None if sw is None else sw['state']
                if state == 'stopping':
                    sw['state'] = 'handing_over'
            if state == 'stopping':
                self._hand_over(sw)
            elif state not in ('handing_over', 'handed_over'):
                # Commands the FC and releases the sticks - see ptest_finish.
                self.go_to_loiter('autotune/plant-ID finish')
        self._tracking_transition = False
        self.autotune_status = None        # clear the HUD banner + timer (sweep over)
        self.autotune_test_t0 = None       # clear the per-test seconds counter too
        self.autotune_test_done_s = None
        self.autotune_start_t = None
        self.straighten_start_t = None     # clear the inter-test STRAIGHT timer too

    # =====================================================================
    # Align-counter state machine (lives here, not in the HUD render)
    # =====================================================================
    def _start_align_counter(self):
        self._align_seconds_t0 = time.monotonic()
        self._align_seconds_frozen = None
        self._align_aligned_t0 = None

    def _clear_align_counter(self):
        self._align_seconds_t0 = None
        self._align_seconds_frozen = None
        self._align_aligned_t0 = None

    def _resolve_align_seconds(self):
        """Advance the align-counter state machine and return the value to
        display (float) or None if the counter isn't running.  Runs once per
        tick from _build_hud_snapshot."""
        if self._align_seconds_t0 is None:
            return None
        now = time.monotonic()
        if self._align_seconds_frozen is None:
            if self.aligned_now():
                if self._align_aligned_t0 is None:
                    self._align_aligned_t0 = now
                elif now - self._align_aligned_t0 >= ALIGN_FREEZE_HOLD_S:
                    self._align_seconds_frozen = self._align_aligned_t0 - self._align_seconds_t0
            else:
                self._align_aligned_t0 = None
        if self._align_seconds_frozen is not None:
            return self._align_seconds_frozen
        return now - self._align_seconds_t0

    def _resolve_aligned_seconds(self):
        """Continuous time (s) the flight path has held within ALIGN_TOL_DEG of the
        balloon (alignment_errors()).  Counts up while inside tolerance, resets to
        0.0 the instant it leaves.  Returns None when not tracking or the error is
        undefined.  Runs once per tick from _build_hud_snapshot.  This is the same
        quantity the autotune align gate dwells on, so it visibly climbs toward
        ALIGN_DWELL_S during a trial."""
        if not self.tracking:
            self._aligned_seconds_t0 = None
            return None
        err = self.alignment_errors()
        if err is None:
            self._aligned_seconds_t0 = None
            return None
        now = time.monotonic()
        if math.hypot(err[0], err[1]) < C.ALIGN_TOL_DEG:
            if self._aligned_seconds_t0 is None:
                self._aligned_seconds_t0 = now
            return now - self._aligned_seconds_t0
        self._aligned_seconds_t0 = None
        return 0.0

    def _resolve_straighten_seconds(self):
        """HELD-level time (s) in the current inter-test FBWA level-out: counts up only
        while the aircraft is actually wings-level (the level_and_hold dwell, published
        as link._straighten_level_since), climbing from 0 toward STRAIGHT_HOLD_S.
        Returns None when not straightening OR not yet level - so the HUD shows nothing
        until the belly is parallel to the ground, then starts the count at 0."""
        if self.straighten_start_t is None:
            return None
        ls = getattr(self.link, '_straighten_level_since', None)
        if ls is None:
            return None
        return time.monotonic() - ls

    def _resolve_autotune_test_seconds(self):
        """Per-test SECONDS counter for the HUD.  Counts up from the moment the current
        test began (autotune_test_t0: the sweep sets it once the level-out reached
        straight, the trim map once the cell's balloon is live) toward that run's own
        timeout (the sweep's ALIGN_TIMEOUT_S, the trim map's CELL_TIMEOUT_S), then
        FREEZES at the elapsed value the test ended with (autotune_test_done_s - the
        time-to-align on a sweep pass, the time a trim-map cell held or stopped).
        Returns None when no test is in flight."""
        if self.autotune_test_done_s is not None:
            return self.autotune_test_done_s          # frozen at the test's end
        if self.autotune_test_t0 is not None:
            return time.monotonic() - self.autotune_test_t0   # counting up live
        return None

    # =====================================================================
    # Background threads
    # =====================================================================
    def _begin_tracking(self):
        """Begin tracking for the active mode (re-syncs FC gains first on the Lua path)."""
        # Lua path: the FC flies whatever gains it was last given, and the
        # tuner can have changed them since startup.  Re-sync on every tracking
        # start - 51 frames, ~600 bytes, once per flight.  Also covers the five
        # gains whose constants.py names do not match the field name, which the
        # script's own defaults cannot resolve.
        _sync = getattr(self.link, "sync_gains", None)
        if _sync is not None:
            try:
                log.info("[lua] synced %d gains to the FC", _sync(self._gains))
            except Exception as exc:                       # noqa: BLE001
                # Clearing the signature makes the per-tick push send the set
                # again on the next tick instead of waiting for an edit.
                self._gains_pushed_sig = None
                log.error("[lua] gain sync FAILED - FC is flying stale gains: %s",
                          exc)
        try:
            self.enter_acro('begin tracking')
            self.tracking = True
            log.info("*** ACRO MODE ACTIVE ***")
        except Exception:
            # Hand back rather than leave the mode set with nothing flying it:
            # after a switch the FC is in whatever mode the stopped run left.
            log.exception("Failed to begin tracking - handing back to LOITER")
            self.tracking = False
            try:
                self.go_to_loiter('begin tracking failed')
            except Exception:                              # noqa: BLE001
                log.exception("[MODE] hand-back after a failed start failed")
        finally:
            self._tracking_transition = False

    def _go_to_loiter(self, reason='mode key'):
        """Thread body for the keyboard mode keys: go_to_loiter() plus clearing
        the transition flag.  Spawned on its own thread because go_to_loiter()
        blocks on HEARTBEAT confirmation and the keys are handled on the 50 Hz
        control thread."""
        try:
            self.go_to_loiter(reason)
        finally:
            self._tracking_transition = False

    def _flight_sequence(self):
        """Run the launch sequence, holding at the pre-launch gimbal-calibration gate."""
        # Launch gate: if a pre-launch gimbal calibration was requested, hold here
        # (do NOT arm/launch) until it completes and opens the gate.  Normally the
        # gate is already open, so this returns immediately.
        if not self._launch_gate.wait(timeout=120.0):
            log.warning("Launch gate timed out (120s); proceeding with launch.")
        try:
            self.link.upload_mission(self.waypoint_lat, self.waypoint_lon, self.takeoff_alt)
            self.link.wait_until_prearm_ok()
            self.link.arm()
            self.link.catapult_launch(self.takeoff_alt)
            self.link.fly_to_waypoint(self.waypoint_lat, self.waypoint_lon)
            self.link.start_loiter()
        except Exception:
            log.exception("Flight sequence failed!")
            return
        self.flight_complete = True

    def _reset_to_idle(self):
        """Tear tracking down and return to the idle / loiter state."""
        self._ptest_abort.set()
        self.sensor.set_active(False)
        self._straight_level = False
        self.sensor.set_gimbal_locked(True)
        if self._point_injector is not None:
            self._point_injector.stop()
            self._point_injector = None
        self.sensor.clear_oracle()
        self._world_estimator.reset()
        att = self.link.attitude
        self._manual_desired_pitch = att[1]
        self._manual_desired_yaw = att[2]
        self._log_det_norm_x = 0.0
        self._log_det_norm_y = 0.0
        self._display_balloon_body = None
        # A new run gets its own pass: re-arm the closest-approach latch and drop
        # the range history so the fit cannot bridge across the mode change.
        self._balloon_range_m = None
        self._pass_distance_m = None
        self._pass_reported = False
        self._rng_hist = []

    # =====================================================================
    # Snapshot builders
    # =====================================================================
    def _flight_path_body_vec(self, att):
        """Velocity vector (flight path) as a unit vector in the BODY frame.
        None when the FC has sent no velocity or the speed is too low to mean
        anything.

        The FC's OWN velocity (STATE2, read on the FC every 50 Hz tick), never
        the MAVLink vel_ned: that comes from GLOBAL_POSITION_INT at 5 Hz and
        holds a turning path up to 200 ms stale - degrees, against a 0.5 deg
        alignment tolerance.  Rotated into the body frame with the same
        world->body transform as the balloon geometry (_body_from_world_R), so
        the path and the balloon share one attitude and the angle between them
        does not depend on it."""
        L = self.link
        vn = getattr(L, 'lua_vel_n_mps', None)
        ve = getattr(L, 'lua_vel_e_mps', None)
        vd = getattr(L, 'lua_vel_d_mps', None)
        if vn is None or ve is None or vd is None:
            return None
        v = np.array([vn, ve, vd], dtype=float)
        speed = float(np.linalg.norm(v))
        # The FC's own test: a path only above FLIGHT_PATH_MIN_SPEED_MPS, where
        # the velocity's direction is more than noise.
        if speed <= C.FLIGHT_PATH_MIN_SPEED_MPS:
            return None
        R_body = self._body_from_world_R(att)
        vb = R_body @ (v / speed)
        n = float(np.linalg.norm(vb))
        return vb / n if n > 1e-9 else None

    def _build_hud_snapshot(self):
        att = self.link.attitude
        return HudSnapshot(
            mode=self._mode, mode_label=(self._mode_label or self._mode),
            tracking=self.tracking, vision_tracking=self.sensor.active,
            gimbal_locked=self.sensor.gimbal_locked,
            gimbal_bearing_pan=self.gimbal.measured_pan_deg,
            gimbal_bearing_tilt=self.gimbal.measured_tilt_deg,
            detection_pixel=self.sensor.detection_pixel_for_hud(),
            elevon_mixing=bool(getattr(self.link, 'elevon_mixing', False)),
            lua_active=bool(getattr(self.link, 'lua_nav_active', False)),
            lua_omega_x=_fc(getattr(self.link, 'lua_omega_x_cmd_dps', None)),
            lua_requested_roll=_fc(getattr(self.link, 'lua_requested_roll', None)),
            lua_requested_pitch=_fc(getattr(self.link, 'lua_requested_pitch', None)),
            lua_roll_cmd_presat=_fc(getattr(self.link, 'lua_roll_cmd_presat', None)),
            lua_bank_lim_deg=float(self._gains.bank_lim_deg),
            lua_omega_x_presat=_fc(getattr(self.link, 'lua_omega_x_presat_dps', None)),
            lua_omega_y_presat=_fc(getattr(self.link, 'lua_omega_y_presat_dps', None)),
            lua_omega_y=_fc(getattr(self.link, 'lua_omega_y_cmd_dps', None)),
            lua_lat_marker_deg=_fc(getattr(self.link, 'lua_lat_marker_deg', None)),
            lua_vel_marker_deg=_fc(getattr(self.link, 'lua_vel_marker_deg', None)),
            lua_yaw_rate_cmd_dps=_fc(getattr(self.link, 'lua_yaw_rate_cmd_dps', None)),
            servo_ch4=getattr(self.link, 'servo_ch4', None),
            mouse_x=self._gimbal_mouse_x, mouse_y=self._gimbal_mouse_y,
            aircraft_att=att, aircraft_airspeed=self.link.airspeed,
            aircraft_relative_alt=self.link.relative_alt,
            last_balloon_body=(self._display_balloon_body
                              if self._display_balloon_body is not None
                              else self._last_balloon_body),
            balloon_range_m=self._balloon_range_m,
            control_balloon_body=self._control_balloon_body_for_display(att),
            balloon_projection_live=self._balloon_projection_live(),
            flight_path_body=self._flight_path_body_vec(att),
            nose_fy_scale=self._nose_fy_scale(),
            servo_aileron=self.link.servo_aileron, servo_elevator=self.link.servo_elevator,
            servo1_min=self.link.servo1_min, servo1_trim=self.link.servo1_trim, servo1_max=self.link.servo1_max,
            servo2_min=self.link.servo2_min, servo2_trim=self.link.servo2_trim, servo2_max=self.link.servo2_max,
            csv_tick_count=self._csv.tick_count,
            align_seconds=self._resolve_align_seconds(),
            draw_roll_error=int(self.draw_roll_error),
            autotune_status=self.autotune_status,
            autotune_test_seconds=self._resolve_autotune_test_seconds(),
            autotune_elapsed_s=(None if self.autotune_start_t is None
                                else time.monotonic() - self.autotune_start_t),
            straighten_elapsed_s=self._resolve_straighten_seconds(),
            aligned_elapsed_s=self._resolve_aligned_seconds(),
            gimbal_fy_scale=self._gimbal_fy_scale(),
        )

    def _build_csv_snapshot(self, now):
        # Every control-law column carries the FC's own value (lua_*).  A column
        # whose quantity the FC does not send is None - written blank, because
        # a zero there would read as a measurement.
        L = self.link
        # The aim error the laws regulate - the flight path against the balloon,
        # alignment_errors().  None when the flight path is undefined, and blank
        # in the CSV then.
        try:
            _align = self.alignment_errors()
        except Exception:                                     # noqa: BLE001
            _align = None
        if _align is None:
            _align = (None, None)
        att = self.link.attitude
        rates = self.link.rates
        event = self._pending_event
        self._pending_event = ''
        if self._mode != self._csv_prev_mode:
            mode_event = f'MODE_{self._mode.upper()}'
            event = f'{mode_event},{event}' if event else mode_event
            self._csv_prev_mode = self._mode
        inj_pos = None
        if self.sensor.oracle_active:
            # Recover the oracle's NED position for the diagnostic columns, if ground-anchored.
            inj_pos = getattr(self.sensor, '_world_pos', None)

        # Nose-projection debug: capture the exact pixels the HUD draws this tick,
        # so the red-dot mismatch can be analysed from the log.  Uses the same
        # helper the HUD uses (single source of truth).
        from .ui.hud import _project_nose_to_gimbal_pixel
        gw = getattr(self.gimbal_camera, 'width', 1280)
        gh = getattr(self.gimbal_camera, 'height', 720)
        nose_status, n_px, n_py = _project_nose_to_gimbal_pixel(
            self.gimbal.measured_pan_deg, self.gimbal.measured_tilt_deg, gw, gh,
            self._gimbal_fy_scale())
        # Log -1 when behind; otherwise the (possibly edge-clamped) pixel.  The
        # 'off' vs 'on' distinction is recoverable from whether the pixel sits on
        # a frame border together with meas_pan/tilt, and the raw quaternion is
        # also logged for exact offline reconstruction.
        if nose_status == 'behind':
            dbg_nose_px, dbg_nose_py = -1, -1
        else:
            dbg_nose_px, dbg_nose_py = n_px, n_py
        det = self.sensor.detection_pixel_for_hud()
        dbg_det_px, dbg_det_py = det if det is not None else (-1, -1)
        dbg_tb = self._last_balloon_body if self._last_balloon_body is not None else (0.0, 0.0, 0.0)
        # Yellow (control-path) marker: the world-frame control balloon rotated
        # into the current body frame.
        # Logged alongside the green (actual) dbg_balloon_body_* so neither circle
        # has to be reconstructed offline.  None (no estimate) -> zeros.
        _ctb = self._control_balloon_body_for_display(att)
        dbg_ctb = (0.0, 0.0, 0.0) if _ctb is None else (float(_ctb[0]), float(_ctb[1]), float(_ctb[2]))

        # Raw gimbal quaternion for offline sign-verification (Gazebo driver only).
        _gq = getattr(self.gimbal, 'measured_quat', None)
        if _gq is None:
            _gq = (0.0, 0.0, 0.0, 0.0)

        # Aircraft attitude quaternion diagnostics: recompute the TRUE bank/pitch
        # from the quaternion (singularity-free) so the log shows whether a wild
        # act_roll near vertical is real or an Euler gimbal-lock artifact.
        _aq = getattr(self.link, 'attitude_quat', np.array([1.0, 0.0, 0.0, 0.0]))
        _aqw, _aqx, _aqy, _aqz = (float(_aq[0]), float(_aq[1]),
                                  float(_aq[2]), float(_aq[3]))
        _qn = math.sqrt(_aqw*_aqw + _aqx*_aqx + _aqy*_aqy + _aqz*_aqz)
        if _qn > 1e-9:
            _w, _x, _y, _z = _aqw/_qn, _aqx/_qn, _aqy/_qn, _aqz/_qn
            # true pitch/bank via the same 3-2-1 extraction, but FROM the quaternion.
            _sp = max(-1.0, min(1.0, 2.0*(_x*_z - _w*_y)))   # -sin(pitch) term
            _true_pitch = math.degrees(math.asin(-_sp))
            _true_bank = math.degrees(math.atan2(2.0*(_y*_z + _w*_x),
                                                 1.0 - 2.0*(_x*_x + _y*_y)))
        else:
            _true_pitch = math.degrees(att[1])
            _true_bank = math.degrees(att[0])
        _q_t = getattr(self.link, 'attitude_quat_recv_t', 0.0)
        # PER-TICK freshness: 1 only when a NEW quaternion arrived since the previous
        # control tick.  The attitude stream delivers slower than the 50 Hz loop,
        # so values repeat on some ticks - see CLAUDE.md, "Raw rate telemetry is
        # hold-and-jump noisy".  mean(quat_fresh) over a log is the effective
        # attitude rate as a fraction of the loop rate.  It does not affect the
        # quat-vs-Euler source selection above.
        _att_seq = int(getattr(self.link, 'attitude_quat_seq', 0))
        _quat_fresh = 1 if (_q_t > 0.0 and _att_seq != getattr(self, '_last_att_quat_seq', -1)) else 0
        self._last_att_quat_seq = _att_seq
        # Log the pitch stack gain set (the four named knobs; one set, both regimes).
        _pg = self._gains.pitch_gains
        # MAVLink NED velocity (5 Hz).  None, written blank, before the first
        # GLOBAL_POSITION_INT.
        _v = getattr(self.link, 'vel_ned', None)
        _vn, _ve, _vd = ((float(_v[0]), float(_v[1]), float(_v[2]))
                         if _v is not None else (None, None, None))

        # The saved ArduPlane parameters as the Gain Tuner holds them - the value
        # the FC last confirmed.  Not re-read here: the tuner's Write and Read and
        # each mode switch's rate-loop write are the only paths that change them.
        _np = getattr(self.link, 'native_params', None)
        _np = _np if isinstance(_np, dict) else {}
        return TickSnapshot(
            t=self._csv.elapsed(now), mode=self._mode, event=event,
            requested_roll=_fc(L.lua_requested_roll), act_roll=att[0],
            requested_pitch=_fc(L.lua_requested_pitch), log_req_pitch_raw=None,
            act_pitch=att[1],
            log_p_term_roll=_fc(L.lua_p_term_roll),
            pan_integrator=_fc(L.lua_pan_integrator),
            pitch_integrator=_fc(L.lua_pitch_integrator),
            log_roll_cmd_presat=_fc(L.lua_roll_cmd_presat),
            log_roll_sat=_bit(L.lua_roll_sat),
            log_d_term_roll=_fc(L.lua_d_term_roll),
            log_d_term_pitch=_fc(L.lua_d_term_pitch),
            log_pan_i_frozen=_bit(L.lua_pan_i_frozen),
            log_pan_i_clamped=_bit(L.lua_pan_i_clamped),
            log_pitch_i_frozen=_bit(L.lua_pitch_i_frozen),
            log_pitch_i_clamped=_bit(L.lua_pitch_i_clamped),
            log_in_tol=int(self.aligned_now()),
            log_align_tol_deg=float(C.ALIGN_TOL_DEG),
            g_shallow_bank_lim=self._gains.bank_lim_deg,
            g_shallow_pan_kd=self._gains.pan_kd,
            g_pitch_kd=self._gains.pitch_kd,
            steering_pan_deg=_fc(L.lua_steering_pan), steering_tilt_deg=_fc(L.lua_steering_tilt),
            roll_align_err=roll_align_error_deg(self._last_balloon_body),
            log_balloon_elev=None, log_aoa_lat=None,
            log_det_norm_x=self._log_det_norm_x, log_det_norm_y=self._log_det_norm_y,
            world_est_trust=_fc(L.lua_world_est_trust),
            aircraft_lat=self.link.lat, aircraft_lon=self.link.lon,
            aircraft_relative_alt=self.link.relative_alt, injected_world_pos=inj_pos,
            gimbal_bearing_pan=self.gimbal.commanded_pan_deg,
            gimbal_bearing_tilt=self.gimbal.commanded_tilt_deg,
            gimbal_meas_pan=self.gimbal.measured_pan_deg,
            gimbal_meas_tilt=self.gimbal.measured_tilt_deg,
            gimbal_quat_x=_gq[0], gimbal_quat_y=_gq[1],
            gimbal_quat_z=_gq[2], gimbal_quat_w=_gq[3],
            dbg_nose_px=dbg_nose_px, dbg_nose_py=dbg_nose_py,
            dbg_det_px=dbg_det_px, dbg_det_py=dbg_det_py,
            dbg_balloon_body_x=float(dbg_tb[0]), dbg_balloon_body_y=float(dbg_tb[1]),
            dbg_balloon_body_z=float(dbg_tb[2]),
            dbg_control_body_x=dbg_ctb[0], dbg_control_body_y=dbg_ctb[1],
            dbg_control_body_z=dbg_ctb[2],
            dbg_aim_body_x=None, dbg_aim_body_y=None, dbg_aim_body_z=None,
            dbg_theta_ip_deg=_fc(L.lua_theta_ip_deg),
            dbg_theta_ip_body_deg=None,
            los_elev_deg=L.lua_los_elev_deg,
            vel_n=_vn, vel_e=_ve, vel_d=_vd,
            aircraft_airspeed=self.link.airspeed, act_yaw=att[2],
            roll_rate=rates[0], pitch_rate=rates[1], yaw_rate=rates[2],
            aircraft_quat_w=_aqw, aircraft_quat_x=_aqx,
            aircraft_quat_y=_aqy, aircraft_quat_z=_aqz,
            true_bank_deg=_true_bank, true_pitch_deg=_true_pitch,
            quat_fresh=_quat_fresh,
            control_dt=float(self._control_dt),
            servo_aileron=self.link.servo_aileron, servo_elevator=self.link.servo_elevator,
            servo_ch3=self.link.servo_ch3, servo_rudder=self.link.servo_ch4,
            servo_ch5=self.link.servo_ch5, servo_ch6=self.link.servo_ch6,
            servo_ch7=self.link.servo_ch7, servo_ch8=self.link.servo_ch8,
            last_pwm_ch1=None, last_pwm_ch2=None,
            fc_mode=int(getattr(self.link, 'current_mode', 0) or 0),
            fc_roll_des=self.link.pid_tuning_field(1, 'desired'),
            fc_roll_ach=self.link.pid_tuning_field(1, 'achieved'),
            fc_roll_ff=self.link.pid_tuning_field(1, 'ff'),
            fc_roll_p=self.link.pid_tuning_field(1, 'p'),
            fc_roll_i=self.link.pid_tuning_field(1, 'i'),
            fc_roll_d=self.link.pid_tuning_field(1, 'd'),
            fc_pitch_des=self.link.pid_tuning_field(2, 'desired'),
            fc_pitch_ach=self.link.pid_tuning_field(2, 'achieved'),
            fc_pitch_ff=self.link.pid_tuning_field(2, 'ff'),
            fc_pitch_p=self.link.pid_tuning_field(2, 'p'),
            fc_pitch_i=self.link.pid_tuning_field(2, 'i'),
            fc_pitch_d=self.link.pid_tuning_field(2, 'd'),
            fc_yaw_des=self.link.pid_tuning_field(3, 'desired'),
            fc_yaw_ach=self.link.pid_tuning_field(3, 'achieved'),
            fc_yaw_ff=self.link.pid_tuning_field(3, 'ff'),
            fc_yaw_p=self.link.pid_tuning_field(3, 'p'),
            fc_yaw_i=self.link.pid_tuning_field(3, 'i'),
            fc_yaw_d=self.link.pid_tuning_field(3, 'd'),
            balloon_range_m=self._balloon_range_m,
            pass_distance_m=self._pass_distance_m,
            bank_proxy_deg=_fc(L.lua_bank_proxy_deg),
            roll_err_proxy_deg=_fc(L.lua_roll_err_deg),
            bank_rate_cmd_dps=L.lua_bank_rate_cmd_dps,
            bank_rate_coupling_dps=L.lua_bank_rate_coupling_dps,
            upright=_fc(L.lua_upright),
            acro_roll_rate_lim=float(getattr(self.link, 'acro_roll_rate', 0.0)),
            align_pan_deg=_align[0],
            align_tilt_deg=_align[1],
            sideslip_deg=L.lua_sideslip_deg,
            aoa_deg=L.lua_aoa_deg,
            sim_roll_deg=_sim_deg(self.link, 'sim_attitude', 0),
            sim_pitch_deg=_sim_deg(self.link, 'sim_attitude', 1),
            sim_p_dps=_sim_deg(self.link, 'sim_rates', 0),
            sim_r_dps=_sim_deg(self.link, 'sim_rates', 2),
            # Background FOV estimator (DIAGNOSTIC ONLY).  The estimate is read
            # from the passive observer; the truth is parsed from the active model
            # SDF at launch, logged beside it PURELY for comparison - never fed in.
            fov_est_deg=self._fov_estimator.fov_deg,
            fov_est_m1_deg=self._fov_estimator.fov_m1_deg,
            fov_est_m2_deg=self._fov_estimator.fov_m2_deg,
            fov_truth_deg=float(getattr(self.gimbal_fov_truth, 'hfov_deg', None) or 0.0),
            fov_est_n=self._fov_estimator.n,
            log_rate_cmd_pitch=None, log_rate_cmd_roll=None,
            # Mirrored FC state.
            lua_omega_x=_fc(getattr(self.link, 'lua_omega_x_cmd_dps', None)),
            lua_requested_roll=_fc(getattr(self.link, 'lua_requested_roll', None)),
            lua_requested_pitch=_fc(getattr(self.link, 'lua_requested_pitch', None)),
            lua_roll_cmd_presat=_fc(getattr(self.link, 'lua_roll_cmd_presat', None)),
            lua_bank_lim_deg=float(self._gains.bank_lim_deg),
            lua_omega_x_presat=_fc(getattr(self.link, 'lua_omega_x_presat_dps', None)),
            lua_omega_y_presat=_fc(getattr(self.link, 'lua_omega_y_presat_dps', None)),
            lua_roll_err_deg=_fc(L.lua_roll_err_deg),
            lua_throttle=_fc(getattr(self.link, 'lua_throttle_out', None)),
            lua_fc_millis=int(getattr(self.link, 'lua_fc_millis', 0)),
            lua_cs_flags=getattr(self.link, 'lua_cs_flags', None),
            # Yaw/coordination.
            lua_accel_y=_fc(getattr(self.link, 'lua_accel_y_mps2', None)),
            lua_accel_z=_fc(getattr(self.link, 'lua_accel_z_mps2', None)),
            lua_yaw_ff_coord=_fc(getattr(self.link, 'lua_yaw_ff_coord_dps', None)),
            lua_yaw_ff_ari=_fc(getattr(self.link, 'lua_yaw_ff_ari_dps', None)),
            lua_lat_marker=_fc(getattr(self.link, 'lua_lat_marker_deg', None)),
            lua_pan_err_deg=_fc(getattr(self.link, 'lua_pan_err_deg', None)),
            lua_yaw_rate_cmd_dps=_fc(getattr(self.link, 'lua_yaw_rate_cmd_dps', None)),
            lua_yaw_point_dps=_fc(getattr(self.link, 'lua_yaw_point_dps', None)),
            lua_pitch_turn_dps=_fc(getattr(self.link, 'lua_pitch_turn_dps', None)),
            fc_aoa=getattr(self.link, 'fc_aoa_deg', None),
            fc_ssa=getattr(self.link, 'fc_ssa_deg', None),
            lua_vel_marker=_fc(getattr(self.link, 'lua_vel_marker_deg', None)),
            g_pitch_trim_offset=self._gains.pitch_trim_offset,
            g_pitch_trim_attitude=self._gains.pitch_trim_attitude,
            lua_pitch_trim_elev=_fc(getattr(self.link, 'lua_pitch_trim_elev', None)),
            lua_pitch_trim_dps=_fc(getattr(self.link, 'lua_pitch_trim_dps', None)),
            # STATE2: the FC's own sensors, law internals and surfaces.
            lua_state2_seq=L.lua_state2_seq, lua_fc_mode=L.lua_fc_mode,
            lua_q_w=L.lua_q_w, lua_q_x=L.lua_q_x, lua_q_y=L.lua_q_y, lua_q_z=L.lua_q_z,
            lua_gyro_x_rads=L.lua_gyro_x_rads, lua_gyro_y_rads=L.lua_gyro_y_rads,
            lua_gyro_z_rads=L.lua_gyro_z_rads,
            lua_vel_n_mps=L.lua_vel_n_mps, lua_vel_e_mps=L.lua_vel_e_mps,
            lua_vel_d_mps=L.lua_vel_d_mps, lua_eas_mps=L.lua_eas_mps,
            lua_eas2tas=L.lua_eas2tas,
            lua_balloon_body_x=L.lua_balloon_body_x, lua_balloon_body_y=L.lua_balloon_body_y,
            lua_balloon_body_z=L.lua_balloon_body_z,
            lua_yaw_point_i_dps=L.lua_yaw_point_i_dps,
            lua_plane_off_nose_deg=L.lua_plane_off_nose_deg,
            lua_plane_err_deg=L.lua_plane_err_deg,
            lua_plane_side=L.lua_plane_side,
            lua_plane_axis_up=L.lua_plane_axis_up,
            lua_plane_drift_dps=L.lua_plane_drift_dps,
            lua_plane_ff_dps=L.lua_plane_ff_dps,
            lua_plane_err_rate_dps=L.lua_plane_err_rate_dps,
            lua_plane_d_dps=L.lua_plane_d_dps,
            lua_plane_path_share=L.lua_plane_path_share,
            lua_plane_i_dps=L.lua_plane_i_dps,
            lua_plane_i_frozen=L.lua_plane_i_frozen,
            lua_plane_path_deg=L.lua_plane_path_deg,
            lua_plane_path_from_vel=L.lua_plane_path_from_vel,
            lua_excite_value=_fc(L.lua_excite_value),
            g_yaw_coordination_ff=self._gains.yaw_coordination_ff,
            lua_yaw_coordination_dps=_fc(getattr(self.link, 'lua_yaw_coordination_dps', None)),
            g_curve_flyer_enable=int(self._gains.curve_flyer_enable),
            g_cf_transfer_when_done=int(self._gains.cf_transfer_when_done),
            g_cf_plan_wait_s=self._gains.cf_plan_wait_s,
            g_cf_approach_deg=self._gains.cf_approach_deg,
            g_tilt_back_travel_deg=self._gains.tilt_back_travel_deg,
            lua_cf_state=getattr(L, 'lua_cf_state', None),
            lua_cf_plan_id=getattr(L, 'lua_cf_plan_id', None),
            lua_cf_progress=getattr(L, 'lua_cf_progress', None),
            lua_cf_eps_deg=getattr(L, 'lua_cf_eps_deg', None),
            lua_cf_bank_ref_deg=getattr(L, 'lua_cf_bank_ref_deg', None),
            lua_cf_hold_n=getattr(L, 'lua_cf_hold_n', None),
            lua_cf_hold_e=getattr(L, 'lua_cf_hold_e', None),
            lua_cf_hold_d=getattr(L, 'lua_cf_hold_d', None),
            lua_path_turn_up_dps=getattr(L, 'lua_path_turn_up_dps', None),
            lua_path_turn_right_dps=getattr(L, 'lua_path_turn_right_dps', None),
            g_steering_law=int(self._gains.steering_law),
            g_att_k_path=self._gains.att_k_path,
            g_att_path_rate_max_dps=self._gains.att_path_rate_max_dps,
            g_att_lead_s=self._gains.att_lead_s,
            g_att_k_roll=self._gains.att_k_roll,
            g_att_k_side=self._gains.att_k_side,
            g_att_roll_ki=self._gains.att_roll_ki,
            g_att_pitch_ki=self._gains.att_pitch_ki,
            g_att_i_zone_deg=self._gains.att_i_zone_deg,
            lua_pitch_trim_lift_term=getattr(L, 'lua_pitch_trim_lift_term', None),
            lua_att_roll_i_dps=getattr(L, 'lua_att_roll_i_dps', None),
            lua_att_pitch_i_dps=getattr(L, 'lua_att_pitch_i_dps', None),
            lua_att_wx_dps=getattr(L, 'lua_att_wx_dps', None),
            lua_att_wy_dps=getattr(L, 'lua_att_wy_dps', None),
            lua_att_wz_dps=getattr(L, 'lua_att_wz_dps', None),
            lua_att_bank_deg=getattr(L, 'lua_att_bank_deg', None),
            lua_att_band_lo_deg=getattr(L, 'lua_att_band_lo_deg', None),
            lua_att_band_hi_deg=getattr(L, 'lua_att_band_hi_deg', None),
            lua_att_drift_dps=getattr(L, 'lua_att_drift_dps', None),
            lua_att_theta_now_deg=getattr(L, 'lua_att_theta_now_deg', None),
            lua_att_p_dps=getattr(L, 'lua_att_p_dps', None),
            lua_att_q_dps=getattr(L, 'lua_att_q_dps', None),
            lua_att_theta_deg=getattr(L, 'lua_att_theta_deg', None),
            lua_att_u_y=getattr(L, 'lua_att_u_y', None),
            lua_att_u_z=getattr(L, 'lua_att_u_z', None),
            lua_att_omega_dps=getattr(L, 'lua_att_omega_dps', None),
            lua_att_f_y_mps2=getattr(L, 'lua_att_f_y_mps2', None),
            lua_att_f_z_mps2=getattr(L, 'lua_att_f_z_mps2', None),
            lua_att_mu_deg=getattr(L, 'lua_att_mu_deg', None),
            lua_att_side=getattr(L, 'lua_att_side', None),
            lua_att_w_f=getattr(L, 'lua_att_w_f', None),
            lua_att_n_lift=getattr(L, 'lua_att_n_lift', None),
            lua_accel_x=_fc(getattr(self.link, 'lua_accel_x_mps2', None)),
            lua_att_room_lo_deg=getattr(L, 'lua_att_room_lo_deg', None),
            lua_att_room_hi_deg=getattr(L, 'lua_att_room_hi_deg', None),
            g_att_prop_speed_mps=self._gains.att_prop_speed_mps,
            g_att_prop_slope=self._gains.att_prop_slope,
            lua_att_thrust_mps2=getattr(L, 'lua_att_thrust_mps2', None),
            lua_att_aim_y_mps2=getattr(L, 'lua_att_aim_y_mps2', None),
            lua_att_aim_z_mps2=getattr(L, 'lua_att_aim_z_mps2', None),
            **self._cf_plan_columns(),
            lua_coord_valid=_bit(L.lua_coord_valid),
            lua_accel_valid=_bit(L.lua_accel_valid),
            lua_lat_marker_valid=_bit(L.lua_lat_marker_valid),
            lua_out_aileron=L.lua_out_aileron, lua_out_elevator=L.lua_out_elevator,
            lua_out_rudder=L.lua_out_rudder, lua_out_throttle=L.lua_out_throttle,
            lua_pwm_aileron=L.lua_pwm_aileron, lua_pwm_vtail_l=L.lua_pwm_vtail_l,
            lua_pwm_vtail_r=L.lua_pwm_vtail_r,
            lua_est_valid=_bit(L.lua_est_valid),
            lua_excite_running=_bit(L.lua_excite_running),
            lua_nav_active=_bit(L.lua_nav_active),
            g_pitch_trim_lift=self._gains.pitch_trim_lift,
            g_pitch_trim_thrust=self._gains.pitch_trim_thrust,
            g_plane_side_hold_deg=self._gains.plane_side_hold_deg,
            g_plane_gain=self._gains.plane_gain,
            g_plane_ff_gain=self._gains.plane_ff_gain,
            g_plane_ff_lpf_rate=self._gains.plane_ff_lpf_rate,
            g_plane_steep_start_deg=self._gains.plane_steep_start_deg,
            g_plane_steep_full_deg=self._gains.plane_steep_full_deg,
            g_plane_ki=self._gains.plane_ki,
            g_plane_windup=self._gains.plane_windup,
            g_plane_kd=self._gains.plane_kd,
            g_plane_kd_rate=self._gains.plane_kd_rate,
            lua_omega_y=_fc(getattr(self.link, 'lua_omega_y_cmd_dps', None)),
            lua_gate=int(getattr(self.link, 'lua_gate', 0)),
            lua_trust=_fc(getattr(self.link, 'lua_world_est_trust', None)),
            lua_dt_ms=int(getattr(self.link, 'lua_dt_ms', 0)),
            lua_frames=int(getattr(self.link, 'lua_frames_rx', 0)),
            lua_scaler=_fc(getattr(self.link, 'lua_speed_scaler', None)),
            # STATE echoes the GUIDE seq it was computed from.  Two consecutive
            # rows with the same lua_seq means the second is a REPEAT - no new
            # frame arrived between them.  lua_seq_lag is how many GUIDE frames
            # the companion has sent since the one the FC last answered, i.e.
            # the round-trip lag in ticks, measured rather than assumed.
            lua_seq=int(getattr(self.link, 'lua_seq', 0)),
            lua_seq_lag=int(getattr(self.link, 'lua_seq_lag', 0)),
            servo1_trim=float(self.link.servo1_trim),
            servo1_min=float(self.link.servo1_min),
            servo1_max=float(self.link.servo1_max),
            servo2_trim=float(self.link.servo2_trim),
            servo4_trim=float(self.link.servo4_trim),
            servo4_min=float(self.link.servo4_min),
            servo4_max=float(self.link.servo4_max),
            servo2_min=float(self.link.servo2_min),
            servo2_max=float(self.link.servo2_max),
            elevon_mixing=1 if getattr(self.link, 'elevon_mixing', False) else 0,
            hz_quat=int(getattr(self.link, 'lua_hz_quat', 0)),
            hz_gyro=int(getattr(self.link, 'lua_hz_gyro', 0)),
            hz_vel=int(getattr(self.link, 'lua_hz_vel', 0)),
            hz_asp=int(getattr(self.link, 'lua_hz_asp', 0)),
            gimbal_age_ms=int(getattr(self.link, 'gimbal_age_ms', 0)),
            rtf=float(getattr(self.link, 'rtf', 0.0)),
            gimbal_sim_age_ms=int(getattr(self.link, 'lua_gimbal_sim_age_ms', 0)),
            g_shallow_pan_p_gain=self._gains.pan_p_gain, g_shallow_pan_ki=self._gains.pan_ki,
            g_shallow_windup_limit_pan=self._gains.windup_limit_pan,
            g_pitch_ki=_pg.ki, g_windup_limit_pitch=_pg.windup_limit,
            g_shallow_roll_gain=self._gains.roll_gain, g_pitch_gain=_pg.gain,
            g_pitch_turn_ff=self._gains.pitch_turn_ff, g_yaw_point_kp=self._gains.yaw_point_kp, g_yaw_point_ki=self._gains.yaw_point_ki,
            g_yaw_point_lim=self._gains.yaw_point_lim, g_yaw_point_windup=self._gains.yaw_point_windup,
            g_throttle_pct=self._effective_throttle_pct(),
            g_pitch_deadband_deg=self._gains.pitch_deadband_deg,
            g_pitch_kd_rate=self._gains.pitch_kd_rate,
            g_airspeed_cmd_enable=self._gains.airspeed_cmd_enable,
            g_airspeed_cmd_mps=self._gains.airspeed_cmd_mps,
            g_airspeed_cmd_kp=self._gains.airspeed_cmd_kp,
            g_airspeed_cmd_ki=self._gains.airspeed_cmd_ki,
            g_airspeed_cmd_min_valid=self._gains.airspeed_cmd_min_valid,
            g_pan_kd_rate=self._gains.pan_kd_rate,
            g_aileron_override=self._gains.aileron_override,
            g_aileron_deflection_pct=self._gains.aileron_deflection_pct,
            g_elevator_override=self._gains.elevator_override,
            g_elevator_deflection_pct=self._gains.elevator_deflection_pct,
            g_rudder_override=self._gains.rudder_override,
            g_rudder_deflection_pct=self._gains.rudder_deflection_pct,
            g_yaw_point_pitch_sched=self._gains.yaw_point_pitch_sched,
            fc_param_ptch2srv_tconst=_np.get('PTCH2SRV_TCONST'),
            fc_param_ptch2srv_rmax_up=_np.get('PTCH2SRV_RMAX_UP'),
            fc_param_ptch2srv_rmax_dn=_np.get('PTCH2SRV_RMAX_DN'),
            fc_param_ptch_rate_ff=_np.get('PTCH_RATE_FF'),
            fc_param_ptch_rate_p=_np.get('PTCH_RATE_P'),
            fc_param_ptch_rate_i=_np.get('PTCH_RATE_I'),
            fc_param_ptch_rate_d=_np.get('PTCH_RATE_D'),
            fc_param_ptch_rate_imax=_np.get('PTCH_RATE_IMAX'),
            fc_param_ptch_rate_fltt=_np.get('PTCH_RATE_FLTT'),
            fc_param_ptch_rate_flte=_np.get('PTCH_RATE_FLTE'),
            fc_param_ptch_rate_fltd=_np.get('PTCH_RATE_FLTD'),
            fc_param_ptch_rate_smax=_np.get('PTCH_RATE_SMAX'),
            fc_param_rll2srv_tconst=_np.get('RLL2SRV_TCONST'),
            fc_param_rll2srv_rmax=_np.get('RLL2SRV_RMAX'),
            fc_param_rll_rate_ff=_np.get('RLL_RATE_FF'),
            fc_param_rll_rate_p=_np.get('RLL_RATE_P'),
            fc_param_rll_rate_i=_np.get('RLL_RATE_I'),
            fc_param_rll_rate_d=_np.get('RLL_RATE_D'),
            fc_param_rll_rate_imax=_np.get('RLL_RATE_IMAX'),
            fc_param_rll_rate_fltt=_np.get('RLL_RATE_FLTT'),
            fc_param_rll_rate_flte=_np.get('RLL_RATE_FLTE'),
            fc_param_rll_rate_fltd=_np.get('RLL_RATE_FLTD'),
            fc_param_rll_rate_smax=_np.get('RLL_RATE_SMAX'),
            fc_param_yaw_rate_ff=_np.get('YAW_RATE_FF'),
            fc_param_yaw_rate_p=_np.get('YAW_RATE_P'),
            fc_param_yaw_rate_i=_np.get('YAW_RATE_I'),
            fc_param_yaw_rate_d=_np.get('YAW_RATE_D'),
            fc_param_yaw_rate_imax=_np.get('YAW_RATE_IMAX'),
            fc_param_yaw_rate_fltt=_np.get('YAW_RATE_FLTT'),
            fc_param_yaw_rate_flte=_np.get('YAW_RATE_FLTE'),
            fc_param_yaw_rate_fltd=_np.get('YAW_RATE_FLTD'),
            fc_param_yaw_rate_smax=_np.get('YAW_RATE_SMAX'),
            # The airspeed-hold loop, which runs on the FC.
            log_asp_en=_bit(L.lua_asp_en),
            log_asp_err_mps=_fc(L.lua_asp_err_mps),
            log_asp_i_term=_fc(L.lua_asp_i_term),
            log_asp_thr_pct=_fc(L.lua_throttle_out),
            log_asp_sat=_bit(L.lua_asp_sat),
        )

    # =====================================================================
    # Main loop
    # =====================================================================
    # Measurement gain ownership
    # =====================================================================
    def begin_measurement_gains(self, reason, **overrides):
        """Take private ownership of specific gains for the duration of a
        measurement, WITHOUT the Gain Tuner appearing to change.

        Never set the values in place: the tuner reads and writes the same
        object, so the switches would flip to the measurement values in front
        of the operator, inviting a "correction" mid-run that the restore at
        the end then silently overwrites.  Two writers, one variable, no way to
        tell whose value won.

        So the control law flies a PRIVATE COPY while the tuner keeps reading
        and editing the operator's own set.  Nothing appears to change on
        screen, operator edits are never lost, and they take effect when the
        run finishes.  The override is stated in the LOG, which is where a
        transient procedural fact belongs.

        Deliberately NOT re-entrant: nested measurements would make "the
        operator's set" ambiguous."""
        import dataclasses
        if self._gains_operator is not None:
            raise RuntimeError("measurement gains already owned - nested "
                               "measurement runs are not supported")
        self._gains_operator = self._gains
        self._gains = dataclasses.replace(self._gains, **overrides)
        log.info("[GAINS] %s owns %d gain(s) for this run (tuner unaffected, "
                 "restored after): %s", reason, len(overrides),
                 ", ".join("%s %s->%s" % (k, getattr(self._gains_operator, k, '?'), v)
                           for k, v in sorted(overrides.items())))

    def update_measurement_gains(self, **overrides):
        """Change fields of the private set a measurement owns - a survey that
        flies each cell at its own setting.  The operator's set is untouched,
        and the change reaches the FC on the next tick like any other.  Raises
        when no measurement owns the gains: writing the operator's set from a
        measurement is the two-writer bug begin_measurement_gains exists to
        prevent."""
        import dataclasses
        if self._gains_operator is None:
            raise RuntimeError("no measurement owns the gains - call "
                               "begin_measurement_gains first")
        self._gains = dataclasses.replace(self._gains, **overrides)

    def _service_curve_planner(self):
        """Plan the turn the FC is waiting for, and send it until it is taken.

        The FC asks for a PLAN by waiting: a fresh chase with the Curve Flyer on
        holds its rates at zero (STATE2 cf_state = CF_STATE_WAIT) until a PLAN
        arrives, or until the Plan Wait (cf_plan_wait_s) has passed and the
        law flies without one.
        Each entry into the wait is a new chase and gets its own plan: a plan
        still being worked out for an earlier one is dropped when it finishes.
        The plan is worked out on a worker thread from the FC's own attitude
        and velocity (STATE2) and the balloon this tick's GUIDE carries, and
        sent every tick until STATE2 echoes its plan_id, because a lost frame
        is not re-sent otherwise.

        Only p and x chases are planned (curve_flyer.flies_mode on the FC), each
        for the steering law selected, which flies it.  A plan belongs to one
        chase: leaving p or x, a mode change between them, tracking stopping, a
        change of steering law or the FC's Curve Flyer going off drops it, so
        the next chase plans from scratch.  The FC drops its own copy on the
        same tick.

        A balloon that starts shallower than the Approach Angle gets a plan with
        a run-in (curve_planner.py): its turn lines up on the run-in line, and
        its progress counts from the angle off that line, not the balloon.
        """
        L = self.link
        if self._gains.curve_flyer_enable and self._cf_proc is None:
            self._cf_proc = curve_planner.PlannerProcess()
        fc_state = getattr(L, 'lua_cf_state', None)
        chase_now = (self._mode, bool(self.tracking), int(self._gains.steering_law))
        if (chase_now != self._cf_chase
                or self._mode not in ('ptest', 'balloon_chaser') or not self.tracking
                or fc_state in (None, C.CF_STATE_OFF)):
            self._cf_plan = None
            self._cf_thread = None
        self._cf_chase = chase_now
        entered = (fc_state == C.CF_STATE_WAIT
                   and self._cf_fc_state_prev != C.CF_STATE_WAIT)
        self._cf_fc_state_prev = fc_state
        if fc_state != C.CF_STATE_WAIT or self._mode not in ('ptest', 'balloon_chaser'):
            return
        if entered:
            self._cf_plan = None
            self._cf_thread = None
        plan = self._cf_plan
        if plan is not None and plan.get('ready'):
            if getattr(L, 'lua_cf_plan_id', None) != plan['plan_id']:
                L.send_plan(plan['plan_id'], plan['law'], plan['eps0_deg'], plan['knots_deg'],
                            plan['runin_climb_deg'], plan['runin_dep_deg'])
            return
        if self._cf_thread is not None or self._cf_proc is None:
            return
        chase = self._curve_chase_now()
        if chase is None:
            return
        self._cf_last_id = self._cf_last_id % 65535 + 1
        plan = dict(plan_id=self._cf_last_id, ready=False, law=chase.law,
                    eps0_deg=float(chase.turn0_deg))
        self._cf_plan = plan

        proc = self._cf_proc

        def work():
            # The plan goes out the moment it is made; the finer check of its
            # turn time is for the console only and runs after.  A planner
            # that fails sends nothing: the law flies the chase without a
            # plan once the Plan Wait has passed.
            t0 = time.monotonic()
            try:
                made = proc.plan(chase)
            except RuntimeError as err:
                log.error("[curve] plan %d failed, the law flies this chase without one: %s",
                          plan['plan_id'], err)
                return
            knots = made['knots']
            plan.update(knots_deg=list(knots),
                        runin_climb_deg=made['runin_climb_deg'],
                        runin_dep_deg=made['runin_dep_deg'],
                        plan_s=time.monotonic() - t0)
            plan['ready'] = True
            runin = not math.isnan(made['runin_dep_deg'])
            parts = []
            try:
                res = proc.check(chase, made)
                parts.append("turn %.2f s" % res['t_done'])
                if runin:
                    parts.append("lined up at %.2f s, dives at %.2f s, ends %.1f deg below"
                                 % (res['t_lined'], res['t_dive'], res['dep_done']))
            except RuntimeError as err:
                parts.append("turn not checked (%s)" % err)
            schedule = "%s bank schedule %s deg" % (
                "proxy" if made['law'] == curve_planner.LAW_CASCADE else "path",
                " ".join(f"{k:+.0f}" for k in knots))
            if runin:
                parts.append("line-up " + schedule)
                parts.append("run-in %+.0f deg above the horizon until the balloon is %.1f "
                             "deg below" % (made['runin_climb_deg'], made['runin_dep_deg']))
            else:
                parts.append(schedule)
            parts.append("planned in %.1f s" % plan['plan_s'])
            log.info("[curve] plan %d: %s", plan['plan_id'],
                     ("; " if runin else ", ").join(parts))

        self._cf_thread = threading.Thread(target=work, daemon=True,
                                           name="curve-planner")
        self._cf_thread.start()

    def _cf_plan_columns(self):
        """The plan in force, for the log: blank until one is ready."""
        p = self._cf_plan
        if p is None or not p.get('ready'):
            return {}
        def present(x):
            # a plan without a run-in carries NaN there: blank in the log
            return None if math.isnan(x) else x
        return dict(cf_plan_id=p['plan_id'], cf_plan_law=p['law'],
                    cf_plan_eps0_deg=p['eps0_deg'],
                    cf_plan_knots_deg=tuple(p['knots_deg']), cf_plan_s=p['plan_s'],
                    cf_plan_runin_climb_deg=present(p['runin_climb_deg']),
                    cf_plan_runin_dep_deg=present(p['runin_dep_deg']))

    def _curve_chase_now(self):
        """The chase to plan, from the FC's own state (STATE2) and the balloon
        this tick's GUIDE carries, for the steering law selected and under the
        gains it flies.  None while an input is missing."""
        L = self.link
        q = [getattr(L, 'lua_q_' + k, None) for k in 'wxyz']
        vel = [getattr(L, 'lua_vel_' + k + '_mps', None) for k in 'ned']
        if any(x is None for x in q + vel):
            return None
        vel = np.asarray(vel, dtype=float)
        speed = float(np.linalg.norm(vel))
        if speed < C.FLIGHT_PATH_MIN_SPEED_MPS:
            return None
        R = quat_to_rotation_matrix(*q)              # world -> body
        canopy = -R[2]                               # body -z, in world axes
        rng = math.inf
        if self._straight_level:
            b = np.array([math.cos(self._straight_desired_yaw),
                          math.sin(self._straight_desired_yaw), 0.0])
        else:
            b = self._balloon_world_dir
            if b is None:
                return None
            ac = self._aircraft_world_pos_ned()
            pos = getattr(self.sensor, 'oracle_world_pos', None)
            if self.sensor.oracle_active and ac is not None and pos is not None:
                d = np.asarray(pos, dtype=float) - np.asarray(ac, dtype=float)
                if np.isfinite(d).all():
                    rng = float(np.linalg.norm(d))
        g = self._gains
        law = int(g.steering_law)
        return curve_planner.Chase.from_state(
            vel / speed, canopy, np.asarray(b, dtype=float), rng=rng,
            alt_m=float(getattr(L, 'relative_alt', curve_planner.START_ALT_M)),
            speed_mps=speed, law=law,
            gains={n: getattr(g, n) for n in curve_planner.PLAN_GAINS[law]},
            tilt_back_deg=g.tilt_back_travel_deg,
            cf_transfer_when_done=g.cf_transfer_when_done,
            approach_deg=g.cf_approach_deg,
            roll_max_dps=float(getattr(L, 'acro_roll_rate', C.ACRO_ROLL_RATE_DEFAULT)),
            pitch_max_dps=float(getattr(L, 'acro_pitch_rate', C.ACRO_PITCH_RATE_DEFAULT)))

    def _push_gains_if_changed(self):
        """Send the gains the law flies to the FC the moment they change.

        The tuner edits a ControlGains on the COMPANION.  On the Lua path the FC
        flies its own copy, so without this an edit made in flight does nothing
        until tracking is restarted - the operator turns a knob, the aircraft
        does not respond, and the natural conclusion is that the gain has no
        effect.

        Pushes self._gains, the set the law FLIES: the operator's, or the
        private set a measurement installed (begin_measurement_gains).  The
        tuner writes the operator's set (operator_gains()), so an edit made
        during a measurement changes nothing here and lands when the run gives
        the gains back.  Skipping the push while a measurement owns the gains
        is the trap: the FC then never receives the private set and flies the
        operator's for the whole run, while the log records the measurement's.

        Compared by VALUE, so a slider dragged back to where it started costs
        nothing.  ~600 bytes when it does fire, and only on the tick it changes.
        """
        _sync = getattr(self.link, "sync_gains", None)
        if _sync is None:
            return                      # not the Lua path; the gains are local
        import dataclasses
        sig = dataclasses.astuple(self._gains)
        if sig == self._gains_pushed_sig:
            return
        try:
            n = _sync(self._gains)
        except Exception as exc:                            # noqa: BLE001
            # Do NOT record the signature: a failed push must retry next tick
            # rather than leaving the FC on stale gains believing it is current.
            log.error("[lua] live gain push FAILED - FC still flying the "
                      "previous set: %s", exc)
            return
        self._gains_pushed_sig = sig
        log.info("[lua] gains changed in flight - pushed %d to the FC", n)

    def end_measurement_gains(self):
        """Give the gains back.  Idempotent, and safe to call from a finally."""
        if self._gains_operator is None:
            return
        self._gains = self._gains_operator
        self._gains_operator = None
        # Force a push next tick: the FC is holding the measurement's private
        # set, and any tuner edit made during the run has not reached it.
        self._gains_pushed_sig = None
        log.info("[GAINS] measurement gains released; the operator's set is live again")

    def operator_gains(self):
        """The set the Gain Tuner reads and writes: the operator's own, even
        while a measurement is flying a private copy."""
        return self._gains_operator if self._gains_operator is not None else self._gains

    def measurement_owns_gains(self):
        return self._gains_operator is not None

    def _apply_camera_views(self):
        """Reconcile the two OpenCV windows with the tuner switches.

        CONTROL THREAD ONLY.  cv2.namedWindow / destroyWindow / imshow /
        waitKey are not thread-safe and must all run on the thread that pumps
        waitKey, so the Tk tuner only ever writes the two ints and this runs
        once per tick to act on them.

        Both views off is allowed: the keyboard does not depend on a camera
        window, because the Gain Tuner captures keystrokes too (see below).

        Returns (nose_on, gimbal_on) as actually applied."""
        want_nose = bool(int(getattr(self, 'nose_cam_display', 0)))
        want_gimbal = bool(int(getattr(self, 'gimbal_cam_display', 1)))

        # Both camera views off is a legitimate configuration - it is the point
        # of the switches, and with the shipped default (nose off) it is the
        # only way to close the gimbal view at all.
        # No stand-in window is needed: cv2.waitKey can only deliver keys to an
        # OpenCV window, so the Gain Tuner captures keystrokes as well and the
        # keyboard handler drains it when waitKey has nothing.
        if want_nose != self._nose_win_open:
            try:
                if want_nose:
                    cv2.namedWindow('Balloon Chaser Camera', cv2.WINDOW_NORMAL)
                else:
                    cv2.destroyWindow('Balloon Chaser Camera')
                self._nose_win_open = want_nose
                log.info("[VIEW] Balloon Chaser Camera %s", "ON" if want_nose else "OFF")
            except Exception:                        # noqa: BLE001
                log.exception("[VIEW] could not toggle the Balloon Chaser Camera window")
                self._nose_win_open = want_nose      # don't retry every tick
        if want_gimbal != self._gimbal_win_open:
            try:
                if want_gimbal:
                    cv2.namedWindow('Gimbal Camera', cv2.WINDOW_NORMAL)
                    # The mouse callback belongs to the window, so it has to be
                    # re-attached every time the window is recreated.
                    cv2.setMouseCallback('Gimbal Camera',
                                         self._keyboard.gimbal_mouse_callback)
                else:
                    cv2.destroyWindow('Gimbal Camera')
                self._gimbal_win_open = want_gimbal
                log.info("[VIEW] Gimbal Camera %s", "ON" if want_gimbal else "OFF")
            except Exception:                        # noqa: BLE001
                log.exception("[VIEW] could not toggle the Gimbal Camera window")
                self._gimbal_win_open = want_gimbal

        # Decode gate on the NOSE camera only.  The gimbal camera must keep
        # decoding whatever its window is doing: HsvDetectionSensor reads it and
        # that IS the control path.
        cam = getattr(self, 'nose_camera', None)
        if cam is not None and hasattr(cam, 'decode_enabled'):
            cam.decode_enabled = self._nose_win_open
        return (self._nose_win_open, self._gimbal_win_open)

    # =====================================================================
    def run(self):
        """The 50 Hz control loop: read telemetry, run the active mode's law,
        command the FC, log the tick, and pump the keyboard/UI."""
        log.info("=== Vision Balloon Chaser - GIMBAL TRACKING ===")
        log.info("Waypoint: %s, %s @ %sm", self.waypoint_lat, self.waypoint_lon, self.takeoff_alt)

        # Open the windows the switches ask for.  _apply_camera_views owns the
        # window lifecycle from here on; calling it before the startup wait means
        # the wait already has any requested HighGUI window for waitKey to pump.
        self._apply_camera_views()
        self._controller_autotune.create_window()

        # The camera records arrival BEFORE its decode gate, so this wait
        # completes whether or not the nose view is switched on - a disabled
        # camera must not hang the boot.
        log.info("Waiting for nose camera feed...%s",
                 "" if self._nose_win_open else "  (view OFF; waiting for the topic only)")
        while not self.nose_camera.received_first_frame:
            cv2.waitKey(1)
            time.sleep(0.05)
        log.info("Nose camera feed active!")

        self.flight_complete = False
        self.link.start_telemetry()
        time.sleep(1.0)
        self.link.request_high_rate_streams()
        # Manifest gate FIRST: name-set equality, then airframe identity.
        # Raises before any per-parameter work if the FC is not the aircraft
        # this build was frozen against. See fc_param_manifest.py.
        self.link.verify_param_manifest()
        self.link.read_fc_config()
        # From here on each mode switch writes the rate loop that mode flies:
        # AUTOTUNE's in ArduPlane's own modes, the operator's in ACRO.  After the
        # startup read, because "only what differs" is measured against it.
        self.link.enable_mode_param_sets()
        self.link.log_telemetry_rate_config()
        # Validate the plant-ID estimator ONCE, here - after the FC and airframe
        # are confirmed, before the flight sequence starts.  Nothing is flying
        # yet, so this pure-Python multi-start fit (a few seconds, GIL-holding)
        # costs no live-loop time; the 'i' key later reads the cached result and
        # is instant.  A regression fails loudly before launch.  It only gates
        # plant-ID, so a failure is logged and flight proceeds - the 'i' key
        # refuses on its own via the same cache.  Placed AFTER the manifest gate
        # so a wrong-airframe abort skips it rather than spending the seconds.
        from .plant_id import selftest_once as _plantid_selftest
        try:
            _plantid_selftest()
            log.info("[SELFTEST] plant-ID estimator validated")
        except AssertionError as _st_err:
            log.error("[SELFTEST] plant-ID estimator FAILED (%s) - the 'i' key "
                      "will refuse to run until this is fixed", _st_err)
        # Sim-side analogue: read the ACTIVE model SDF from file and report its
        # camera/gimbal parameters.  Ground truth for the FOV estimator's CSV
        # comparison ONLY - never used in any calculation.  Warns, never raises.
        if self.gimbal_fov_truth is not None and hasattr(
                self.gimbal_fov_truth, 'verify_and_report'):
            self.gimbal_fov_truth.verify_and_report()

        # Pre-launch gimbal calibration: hold the launch, sweep the grid, then
        # open the gate so the catapult fires only AFTER calibration completes.
        if self.calibrate_gimbal_on_startup:
            log.info("=== Pre-launch gimbal calibration requested ===")
            self._launch_gate.clear()
            threading.Thread(target=self._flight_sequence, daemon=True, name="flight").start()
            # Run the sweep on THIS thread (blocking) so launch can't race it.
            gimbal_calibration.run_calibration_sweep(self.gimbal)
            self._launch_gate.set()      # release the held flight sequence
            log.info("=== Calibration done; launch released ===")
        else:
            threading.Thread(target=self._flight_sequence, daemon=True, name="flight").start()

        MAIN_LOOP_HZ = 50
        tick_duration = 1.0 / MAIN_LOOP_HZ
        self._control_dt = tick_duration
        self._last_control_time = time.monotonic()
        next_tick = time.monotonic()

        halt_reason = None
        try:
            while True:
                # The FC link first: a silent FC, or one speaking another
                # protocol build, makes every lua_* value this tick reads stale
                # or wrong.  Stop rather than fly or log on it.
                _link_fault = getattr(self.link, "link_fault", None)
                _why = _link_fault() if callable(_link_fault) else None
                if _why:
                    raise LinkFault(_why)

                # Pump the sensor every tick (matches source _update_gimbal, which
                # ran unconditionally) so the gimbal slews and detection state
                # refreshes in ALL modes - including loiter/straight with the 'l'
                # vision tracking, where _update_control takes the idle path but the
                # gimbal must still track.  The sensor uses its own wall-clock for
                # the slew; the passed dt is the control dt for any dt-based logic.
                # Hand over the last estimate first for detection-lost reprojection.
                self.sensor.set_last_world_dir(self._world_estimator.get())
                self.sensor.set_auto_gimbal(self.auto_gimbal)
                self.sensor.update_per_tick(
                    self._control_dt, self.link.attitude,
                    self._aircraft_world_pos_ned(), self.link.rates)

                now = time.monotonic()
                self._control_dt = now - self._last_control_time
                self._last_control_time = now

                # Feed the FC this tick's inputs: the sensor, the operator's
                # intent and the configuration.  GUIDE goes out below.
                _dnx, _dny = self.sensor.detection_norm()
                _meas = self.sensor.measurement()
                self.link.set_detection(_dnx, _dny, _meas is not None)
                # The oracle bearing, when ptest is driving.  Same source
                # _balloon_world_dir uses, so the FC flies what the HUD shows.
                if getattr(self.sensor, "oracle_active", False):
                    _od = self.sensor.oracle_world_dir(
                        self._aircraft_world_pos_ned())
                    if _od is not None:
                        self.link.set_oracle_dir(float(_od[0]), float(_od[1]),
                                                 float(_od[2]))
                self.link.set_mode(
                    MODE_CODE[self._mode], self.tracking,
                    oracle=bool(getattr(self.sensor, "oracle_active", False)),
                    straight_level=bool(self._straight_level))
                self.link.set_manual_setpoints(
                    math.degrees(self._straight_desired_yaw),
                    math.degrees(self._manual_desired_yaw),
                    math.degrees(self._manual_desired_pitch))
                # The FC has no mount driver here - the gimbal is a Gazebo model
                # driven from this process - so it cannot read the encoders and
                # is sent what we measure.  `_gimbal` is private to the sensor,
                # hence getattr rather than a new accessor on the Protocol.
                _gd = getattr(self.sensor, "_gimbal", None)
                if _gd is not None:
                    # When the angles were READ, so GUIDE can carry their age.
                    # The FC pairs them with an on-chip attitude, so this age IS
                    # the attitude-pairing error C-02 exists to prevent.
                    self.link.set_gimbal(_gd.measured_pan_deg, _gd.measured_tilt_deg,
                                         _gd.has_readback,
                                         getattr(_gd, 'measured_read_mono', None),
                                         getattr(_gd, '_gimbal_stamp_ms', 0))

                self._push_gains_if_changed()
                self._service_curve_planner()
                self._update_control()

                # Range + closest-approach detection run at CONTROL rate, right
                # after control, so the HUD and CSV snapshots built below both
                # see this tick's value rather than lagging one tick.
                self._update_balloon_range()

                # Build the HUD snapshot once and share it between both camera
                # views so the nose and gimbal overlays use the same tick's data.
                # Reconcile the view switches first, so this tick renders to
                # exactly the windows that exist.
                nose_on, gimbal_on = self._apply_camera_views()
                hud = self._build_hud_snapshot() if (nose_on or gimbal_on) else None
                if nose_on:
                    render_nose_camera(self.nose_camera.render(), hud)
                if gimbal_on:
                    # get_gimbal_frame() is the DISPLAY path.  Detection reads
                    # the camera through the sensor on its own, so skipping this
                    # costs nothing but the view.
                    gimbal_frame = get_gimbal_frame(self.gimbal_camera)
                    render_gimbal_camera(gimbal_frame, hud)

                # EVERY tick, tracking or not.  The frame is the FC's evidence
                # that the companion is alive - with no new one for
                # GUIDE_STALE_MS it stands down - and its tracking flag is the
                # live instruction: the FC stands down the tick it clears.  The
                # throttle is the operator's; the airspeed-hold loop that may
                # modulate it runs on the FC.
                self.link.send_guide(
                    float(np.clip(self._effective_throttle_pct(), 0, 100)) / 100.0)

                if now - self._last_heartbeat_time >= 1.0:
                    self.link.send_gcs_heartbeat()
                    self._last_heartbeat_time = now

                if self._csv.active:
                    self._csv.write_tick(self._build_csv_snapshot(now))
                else:
                    self._pending_event = ''

                self._controller_autotune.render()
                self._service_switch()
                self._keyboard.handle_key()

                next_tick += tick_duration
                sleep_time = next_tick - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    next_tick = time.monotonic()

        except KeyboardInterrupt:
            log.info("Ctrl+C - shutting down...")
        except LinkFault as fault:
            halt_reason = str(fault)
            log.critical("FC link fault - stopping: %s", fault)
        except Exception as exc:
            halt_reason = ("The control loop crashed: %s: %s\n"
                           "The full traceback is in the console above."
                           % (type(exc).__name__, exc))
            log.exception("Main loop crashed!")
        finally:
            # Same canonical path as every other hand-back: go_to_loiter()
            # releases the overrides in its own finally, so a refused or
            # timed-out LOITER still cannot leave the sticks latched.  It
            # leaves the FC holding the operator's rate-loop values, which the
            # next startup reads back as theirs.
            self._cancel_switch()
            try:
                self.go_to_loiter('shutdown', keep_operator_params=True)
            except Exception:
                log.exception("Failed to hand back to LOITER on shutdown!")
            try:
                self.link.shutdown()
            except Exception:
                log.exception("Link shutdown failed")
            self._csv.stop()
            try:
                self._controller_autotune.close()
            except Exception:
                log.exception("Failed to close Gain Tuner")
            cv2.destroyAllWindows()
            if halt_reason is not None:
                _halt_on_console(halt_reason)



