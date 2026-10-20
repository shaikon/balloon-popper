# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""
HUD overlays for the nose camera and gimbal camera windows.

Module-level render functions driven by a HudSnapshot the controller builds
once per tick.  The HUD never reaches into controller/link/sensor internals -
it only reads the snapshot.  This keeps rendering a pure function of state.

The align-counter STATE MACHINE lives in the controller, not here: tracking
progress is state, not rendering.  The HUD receives the already-resolved
`align_seconds` (float or None) and `align_frozen` (bool) and only displays them.
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

from ..constants import GIMBAL_FOV_DEG, ALIGN_TOL_DEG
from ..geometry import project_body_vec_to_pixel, vector_to_pan_tilt, pan_tilt_to_vector

# Balloon-marker colours (BGR).  Unified scheme across BOTH camera views:
#   GREEN  = the ACTUAL balloon (raw detection / detection-derived body vector).
#   YELLOW = the CONTROL balloon (what the control loop is chasing this tick).
# The gimbal view additionally draws its own raw-detection circle in green
# (the 'not gimbal_locked' path), so green is the actual-balloon colour there too.
BALLOON_ACTUAL_BGR = (0, 0, 255)      # red    - actual balloon
BALLOON_CONTROL_BGR = (0, 255, 255)   # yellow - control-path (filtered) balloon

# Belly-canopy axis colour (BGR) - orange.  The belly-canopy axis is an airframe
# reference (body +/-Z), not a balloon marker, so it gets its own distinct colour.
BELLY_AXIS_BGR = (0, 140, 255)           # orange

# Nose-camera vertical focal-length scale fy/fx for the display projection.
# The Gazebo cameras render at a NATIVE 4:3 resolution (320x240) but frames are
# resized to a 16:9 display (1280x720) before projection, stretching horizontal
# (4x) more than vertical (3x).  The vertical focal length must therefore be
# scaled by native_aspect/display_aspect = (4/3)/(16/9) = 0.75.  This fallback is
# used only when the live native dimensions aren't available; the controller
# normally computes the exact ratio from the camera's reported resolutions.
NOSE_FY_SCALE = 0.75


# -----------------------------------------------------------------------------
# HudSnapshot - one tick's worth of values the HUD needs.
# Built by the controller; the HUD treats it as read-only.
# -----------------------------------------------------------------------------
@dataclass
class HudSnapshot:
    # Mode / phase
    mode: str
    tracking: bool
    vision_tracking: bool
    gimbal_locked: bool

    # Gimbal
    gimbal_bearing_pan: float
    gimbal_bearing_tilt: float
    detection_pixel: Optional[Tuple[int, int]]
    mouse_x: int
    mouse_y: int

    # Aircraft state
    aircraft_att: np.ndarray            # (roll, pitch, yaw) rad
    aircraft_airspeed: float
    aircraft_relative_alt: float
    last_balloon_body: Optional[np.ndarray]
    # The CONTROL-PATH balloon in the body frame (estimator/oracle rotated into
    # body) - what the cascade actually flies on.  Drawn ORANGE.  This is the
    # real control balloon, not a display smoothing, so it trails the raw
    # detection by the estimator's filtering.
    control_balloon_body: Optional[np.ndarray]
    balloon_projection_live: bool
    # Flight-path (velocity vector) direction in the BODY frame, unit vector.
    # Projects to the "where the aircraft is actually going" marker (triangle),
    # which sits ~AoA below the boresight because the wing flies shallower than
    # the nose.  None when speed is too low for a meaningful direction.
    flight_path_body: Optional[np.ndarray]
    # Nose-camera vertical focal scale (native_aspect / display_aspect) applied to
    # the display projection to correct the non-uniform resize.  DISPLAY ONLY.
    nose_fy_scale: float

    # Servo outputs + ranges (for deflection blocks)
    servo_aileron: Optional[int]
    servo_elevator: Optional[int]
    servo1_min: float; servo1_trim: float; servo1_max: float
    servo2_min: float; servo2_trim: float; servo2_max: float

    # Counters
    csv_tick_count: int
    align_seconds: Optional[float]      # resolved by controller; None = not shown

    # Debug overlay switch (tuner 'Draw Debug'): 0=off, 1=draw nose proj + rays.
    draw_roll_error: int = 0

    # Live status of the run in the 'autotune' regime (the sweep, plant-ID, the
    # trim map), or None when none is running.  Shown only while mode == 'autotune'.
    autotune_status: Optional[str] = None

    # Live per-test SECONDS counter: time since the current test began, counting up
    # toward the run's own timeout and FROZEN at the elapsed value when the test ends
    # (see BalloonChaser._resolve_autotune_test_seconds).  None when no test is in
    # flight.  Shown on its own HUD line above the run banner.
    autotune_test_seconds: Optional[float] = None

    # Elapsed seconds since the running run started (for the HUD MM:SS timer), or
    # None when none is running.  Recomputed each frame so the timer ticks.
    autotune_elapsed_s: Optional[float] = None

    # Elapsed seconds in the current inter-test FBWA level-out ("Straight" timer), or
    # None when not straightening between autotune trials.  Recomputed each frame.
    straighten_elapsed_s: Optional[float] = None

    # Continuous seconds the flight path has held within ALIGN_TOL_DEG of the
    # balloon: >0 while aligned (counting up), 0.0 when momentarily out of
    # tolerance, None when not tracking / the error is undefined.
    aligned_elapsed_s: Optional[float] = None
    # Slant range to the balloon in metres, or None when the balloon has no finite
    # position (direction anchor = at infinity; real vision = bearing-only, no
    # range).  None renders as "Infinity".
    balloon_range_m: Optional[float] = None

    # Gimbal-camera vertical focal scale (native_aspect / display_aspect),
    # applied to the gimbal-view debug projections to correct the non-uniform
    # native->display resize - the gimbal-side analogue of nose_fy_scale.
    # DISPLAY ONLY; 1.0 = no correction (native and display aspect equal).
    gimbal_fy_scale: float = 1.0




    # DISPLAY-ONLY label for the mode line.  plant-ID runs as mode
    # 'autotune' on purpose (it reuses the sweep's proven mode regime), so
    # this is how the operator is told which of the two is actually running.
    # Appended LAST and defaulted: a field with a default cannot precede
    # fields without one, and '' falls back to mode at render time.
    mode_label: str = ''

    # ---- The FC's own values ----------------------------------------------
    # Defaulted, therefore LAST: every field without a default must precede
    # every field with one.
    # None - drawn "N/A" - where the FC sent none: before the first frame, and
    # on a tick the law did not run or did not command.
    # lua_active: the FC is currently commanding.
    lua_active: bool = False
    lua_omega_x: Optional[float] = None
    lua_requested_roll: Optional[float] = None    # bank AFTER the limit (rad, from the FC)
    lua_requested_pitch: Optional[float] = None   # pitch setpoint the rate loop tracks (rad)
    lua_roll_cmd_presat: Optional[float] = None   # bank the law asked for, BEFORE the limit (deg)
    lua_bank_lim_deg: float = 45.0     # the limit it was clipped to (deg)
    lua_omega_x_presat: Optional[float] = None
    lua_omega_y_presat: Optional[float] = None
    # The FC's OWN flight-path markers - the two angles the control laws close.
    # The triangle is drawn from these, so the picture and the law are the same
    # thing rather than two answers to one question.
    lua_lat_marker_deg: Optional[float] = None
    lua_vel_marker_deg: Optional[float] = None
    lua_yaw_rate_cmd_dps: Optional[float] = None
    # V-tail RIGHT. SERVO2 and SERVO4 both carry elevator + rudder mixed, so
    # rudder is their DIFFERENCE and elevator their mean. Declared HERE, with
    # the other defaulted fields, because a dataclass cannot have a defaulted
    # field before a non-defaulted one.
    servo_ch4: Optional[int] = None
    lua_omega_y: Optional[float] = None
    elevon_mixing: bool = False

def get_gimbal_frame(gimbal_camera):
    """Fetch the current gimbal camera frame (or a placeholder)."""
    if gimbal_camera.received_first_frame:
        return gimbal_camera.render()
    placeholder = np.zeros((720, 1280, 3), dtype=np.uint8)
    cv2.putText(placeholder, "Gimbal camera not connected",
                (380, 360), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 255), 2)
    return placeholder


def _pwm_to_deflection_pct(pwm, trim, lo, hi):
    """Map PWM to signed deflection percentage relative to a trim point, with
    asymmetric ranges handled by normalising on whichever side of trim the PWM
    is on.  Returns None on bad input."""
    try:
        pwm_f = float(pwm)
        trim_f = float(trim)
        lo_f = float(lo)
        hi_f = float(hi)
        if pwm_f >= trim_f:
            span = max(hi_f - trim_f, 1.0)
            pct = (pwm_f - trim_f) / span * 100.0
        else:
            span = max(trim_f - lo_f, 1.0)
            pct = (pwm_f - trim_f) / span * 100.0
        return max(-100.0, min(100.0, pct))
    except (AttributeError, TypeError, ValueError):
        return None


def _mode_name(s):
    """The mode as the HUD names it: a run's own label ("Trim Map", "Plant
    ID") when it set one, else the mode title-cased ("Autotune", "Ptest")."""
    label = getattr(s, 'mode_label', '') or ''
    mode = str(getattr(s, 'mode', '') or '')
    return label if label and label != mode else mode.title()


def _dir_rl(v):
    """Right/Left for a roll-axis quantity. Both words are 4-5 chars so the
    numbers stay column-aligned down the block."""
    return "Right" if v >= 0 else "Left "


def _dir_ud(v):
    """Up/Down for a pitch-axis quantity, padded to match _dir_rl."""
    return "Up   " if v >= 0 else "Down "


def _cmd_line(label, v, dir_fn, unit):
    """One command line: direction, signed value and unit, or N/A where the
    FC sent no value this tick."""
    if v is None:
        return f"{label}: N/A"
    return f"{label}: {dir_fn(v)} {v:+6.1f} {unit}"


def _format_aileron_line(label, pct):
    if pct is None:
        return f"{label}: ---"
    direction = "Right" if pct >= 0 else "Left"
    return f"{label}: {direction} {pct:+6.1f}%"


def _format_elevator_line(label, pct):
    if pct is None:
        return f"{label}: ---"
    direction = "Up" if pct >= 0 else "Down"
    return f"{label}: {direction} {pct:+6.1f}%"


def render_nose_camera(frame, s=None):
    """Draw centre mark + label on the nose camera feed and display it.


    If a HudSnapshot s is supplied, also overlay the BALLOON projection:
      - balloon in frame  -> green circle (like the gimbal detection marker);
      - balloon off frame  -> red marker clamped to the nearest edge (mirrors the
        gimbal view's off-frame nose marker);
      - balloon behind     -> red 'X' + label at centre-top.
    And, when the 'Draw Roll Error' switch is on, the same two dashed yellow rays
    as the gimbal view: a finite ray from the nose (image centre, since the nose
    camera looks down body +X) to the balloon, and an infinite ray along the
    belly-canopy axis (body +/-Z), which in this fixed camera is vertical.
    """
    h, w = frame.shape[:2]
    cx, cy = w / 2, h / 2
    cv2.line(frame, (int(cx - 20), int(cy)), (int(cx + 20), int(cy)), (0, 255, 0), 2)
    cv2.line(frame, (int(cx), int(cy - 20)), (int(cx), int(cy + 20)), (0, 255, 0), 2)
    cv2.putText(frame, "Nose Camera", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

    if s is not None:
        # Flight-path (velocity vector) marker: a triangle showing where the
        # aircraft is actually GOING, vs the boresight centre mark (where the nose
        # points).  The gap between them is the angle of attack - the wing flies
        # shallower than the nose - so the triangle normally sits a few degrees
        # below the centre mark.  Always shown when speed is sufficient (not gated
        # on the debug switch); it is primary flight information.
        fpb = _flight_path_body(s)
        if fpb is not None:
            fy_scale = float(getattr(s, 'nose_fy_scale', 1.0))
            _draw_flight_path_marker(frame, fpb, w, h, cx, cy, fy_scale)

        draw_debug = bool(getattr(s, 'draw_roll_error', 0))
        # Everything below (belly axis, balloon marker + ray) is the DEBUG overlay,
        # shown only when the 'Draw Debug' switch is on.
        if draw_debug:
            # Belly-canopy axis: a property of the AIRFRAME, independent of any
            # balloon/lock/mode.  On the fixed forward nose camera the body +/-Z
            # axis projects to the image vertical through centre.
            L = float(h)
            _draw_dashed_line(frame, (int(cx), int(cy - L)),
                              (int(cx), int(cy + L)), BELLY_AXIS_BGR)
            # Balloon marker + ray: only when a balloon is actually live.
            if getattr(s, 'balloon_projection_live', False):
                fy_scale = float(getattr(s, 'nose_fy_scale', 1.0))
                tb_u = getattr(s, 'last_balloon_body', None)
                tb_f = getattr(s, 'control_balloon_body', None)
                # GREEN  = actual balloon (raw/unfiltered, detection-derived);
                # YELLOW = control-path balloon (estimator/oracle in body frame).
                # Control drawn first so the actual-balloon marker sits on top.
                _draw_nose_balloon(frame, tb_f, w, h, cx, cy,
                                  BALLOON_CONTROL_BGR, True, fy_scale)
                _draw_nose_balloon(frame, tb_u, w, h, cx, cy,
                                  BALLOON_ACTUAL_BGR, True, fy_scale)

    cv2.imshow('Balloon Chaser Camera', frame)


# --- Flight-path marker (velocity vector indicator) ---------------------------
# The flight-path body vector is the raw velocity direction.  The HUD projects it
# for each camera.  It updates at the velocity-telemetry rate (~5 Hz), so it
# moves more than the attitude-derived markers.


def _flight_path_body(s):
    """The flight-path direction to DRAW, as a unit vector in body axes.

    THE FC'S OWN MARKERS FIRST.  lat_marker_deg and vel_marker_deg are the two
    angles the control laws actually close - computed on the aircraft from the
    EKF read at 50 Hz, both over the ground - and they arrive in every STATE
    frame.  Placing the triangle from them makes the picture and the law the same
    thing: the sideways gap you see IS the rudder's error, and the height IS the
    angle of attack.

    The companion can rebuild the vector itself from `vel_ned`, and that is the
    fallback, but it is a different answer to the same question: MAVLink velocity
    at a configured rate rather than 50 Hz and the companion's attitude rather
    than the FC's.  A marker drawn from it can sit still while the aircraft
    manoeuvres, which makes "the triangle is off" and "the law has an error"
    impossible to tell apart.
    """
    lat = getattr(s, 'lua_lat_marker_deg', None)
    vert = getattr(s, 'lua_vel_marker_deg', None)
    if lat is not None and vert is not None:
        return pan_tilt_to_vector(float(lat), float(vert))
    return getattr(s, 'flight_path_body', None)


def _draw_flight_path_triangle(frame, px, py, r=20):
    """Draw the upward-pointing flight-path triangle (cyan) centred at (px, py)."""
    FP_BGR = (255, 255, 0)        # cyan - distinct from centre mark/balloon
    px, py = int(px), int(py)
    pts = np.array([
        [px,                 py - r],
        [px - int(0.87 * r), py + int(0.5 * r)],
        [px + int(0.87 * r), py + int(0.5 * r)],
    ], dtype=np.int32)
    cv2.polylines(frame, [pts], isClosed=True, color=FP_BGR, thickness=2)
    cv2.circle(frame, (px, py), 2, FP_BGR, -1)


def _draw_flight_path_marker(frame, fpb, w, h, cx, cy, fy_scale=1.0):
    """Nose-camera flight-path marker: project the RAW flight-path body vector
    through the fixed nose camera (gimbal pan/tilt = 0)."""
    if fpb is None:
        return
    status, px, py = project_body_vec_to_pixel(
        fpb, 0.0, 0.0, w, h, GIMBAL_FOV_DEG, fy_scale)
    # 'on' = in front and on-screen; 'off' = in front but outside; 'behind'.
    if status != 'on':
        return
    _draw_flight_path_triangle(frame, px, py)


def _draw_flight_path_marker_gimbal(frame, fpb, gimbal_pan_deg, gimbal_tilt_deg,
                                    gw, gh, fy_scale=1.0):
    """Gimbal-camera flight-path marker: SAME body vector, projected through the
    gimbal's CURRENT pan/tilt (the gimbal camera is not rigid to the airframe),
    so the triangle lands correctly in the slewed view."""
    if fpb is None:
        return
    status, px, py = project_body_vec_to_pixel(
        fpb, gimbal_pan_deg, gimbal_tilt_deg, gw, gh, GIMBAL_FOV_DEG, fy_scale)
    if status != 'on':
        return
    _draw_flight_path_triangle(frame, px, py)


def _draw_nose_balloon(frame, tb, w, h, cx, cy, color, draw_ray, fy_scale=1.0):
    """Draw one balloon projection on the nose camera: the balloon marker (filled
    circle if on-screen, else an edge marker placed ON the ray where it exits the
    frame) and the finite nose->balloon ray.  No text labels.  `color` selects
    the marker/ray colour.  `fy_scale` corrects the vertical native-to-display
    resize.  The belly-canopy axis is drawn separately in render_nose_camera (it
    exists independently of any balloon).

    The pinhole projection is shared with the gimbal-view sites via
    project_body_vec_to_pixel - the nose camera is rigid to the airframe so
    pan=tilt=0 (identity rotation: camera frame == body frame)."""
    if tb is None:
        return
    status, raw_tx, raw_ty = project_body_vec_to_pixel(
        tb, 0.0, 0.0, w, h, GIMBAL_FOV_DEG, fy_scale)

    if status != 'behind':
        if status == 'on':
            cv2.circle(frame, (int(raw_tx), int(raw_ty)), 20, color, 2)
            cv2.circle(frame, (int(raw_tx), int(raw_ty)), 3, color, -1)
        else:
            # Edge marker placed ON the ray: clip the nose->balloon segment to the
            # frame and mark where it exits, so the marker always lies on the ray.
            seg = _clip_line_to_rect(cx, cy, *_unit(raw_tx - cx, raw_ty - cy), w, h)
            if seg is not None:
                ex, ey = seg[1]   # far end = where the ray leaves the frame
                cv2.circle(frame, (int(ex), int(ey)), 12, color, 3)
        if draw_ray:
            _draw_dashed_line(frame, (int(cx), int(cy)),
                              (int(raw_tx), int(raw_ty)), color)
    else:
        # Balloon behind: small X at top-centre, no label.
        bx, by = int(w / 2), 50
        cv2.line(frame, (bx - 12, by - 12), (bx + 12, by + 12), color, 3)
        cv2.line(frame, (bx - 12, by + 12), (bx + 12, by - 12), color, 3)


def _unit(dx, dy):
    d = math.hypot(dx, dy)
    if d < 1e-9:
        return (0.0, 1.0)
    return (dx / d, dy / d)


def _draw_dashed_line(img, p0, p1, color, thickness=2, dash=12, gap=8):
    """Draw a dashed line from p0 to p1.  Guards against non-finite or absurdly
    large coordinates (e.g. a balloon projecting near the camera horizon, which
    can blow the pixel value past the C-int range and crash cv2.line)."""
    x0, y0 = p0
    x1, y1 = p1
    for v in (x0, y0, x1, y1):
        if not math.isfinite(v) or abs(v) > 1e6:
            return
    x0, y0, x1, y1 = int(x0), int(y0), int(x1), int(y1)
    dx, dy = x1 - x0, y1 - y0
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return
    ux, uy = dx / dist, dy / dist
    n = int(dist // (dash + gap)) + 1
    for i in range(n):
        s = i * (dash + gap)
        e = min(s + dash, dist)
        xa, ya = int(x0 + ux * s), int(y0 + uy * s)
        xb, yb = int(x0 + ux * e), int(y0 + uy * e)
        cv2.line(img, (xa, ya), (xb, yb), color, thickness)


def _draw_roll_error_rays(gimbal_frame, s, nose_px, nose_py, gw, gh,
                          tb, color, use_detection_pixel):
    """Draw the BALLOON ray + marker for ONE balloon vector `tb` in `color`:
    a ray from the nose to the balloon (clipped to the frame so a balloon near the
    camera horizon can't blow the pixel coords past the int range), with an edge
    marker placed ON the ray where it exits the frame when the balloon is
    off-screen, and a filled circle when it's on-screen.

    Anchored at the TRUE (unclamped) nose projection so the geometry stays correct
    when the nose itself is off-frame.  The belly-canopy axis is drawn separately
    by _draw_belly_canopy_axis (it exists independently of any balloon).
    """
    if tb is None:
        return

    fy_scale = getattr(s, 'gimbal_fy_scale', 1.0)
    raw_nose = _project_body_vec_to_gimbal_pixel_raw(
        (1.0, 0.0, 0.0), s.gimbal_bearing_pan, s.gimbal_bearing_tilt, gw, gh, fy_scale)
    if raw_nose is None:
        return   # nose behind gimbal - rays are meaningless
    anchor_x, anchor_y = raw_nose

    balloon_px = None
    if use_detection_pixel and s.detection_pixel is not None:
        balloon_px = (float(s.detection_pixel[0]), float(s.detection_pixel[1]))
    else:
        balloon = _project_body_vec_to_gimbal_pixel_raw(
            tb, s.gimbal_bearing_pan, s.gimbal_bearing_tilt, gw, gh, fy_scale)
        if balloon is not None:
            balloon_px = balloon
    if balloon_px is not None:
        on_screen = (0 <= balloon_px[0] <= gw - 1) and (0 <= balloon_px[1] <= gh - 1)
        # FINITE ray from the nose anchor to the balloon (it must end ON the balloon,
        # not run edge-to-edge).  Clip the SEGMENT (anchor->balloon) to the frame -
        # only to keep coordinates bounded when the balloon is far off-screen (a
        # balloon near the camera horizon projects to ~1e9 px and would crash
        # cv2.line); the visible end still lands at the balloon / its frame-exit.
        seg = _clip_segment_to_rect(anchor_x, anchor_y, balloon_px[0], balloon_px[1], gw, gh)
        if seg is not None:
            _draw_dashed_line(gimbal_frame,
                              (int(seg[0][0]), int(seg[0][1])),
                              (int(seg[1][0]), int(seg[1][1])), color)
        # Balloon marker.
        if on_screen:
            cv2.circle(gimbal_frame, (int(balloon_px[0]), int(balloon_px[1])), 20, color, 2)
            cv2.circle(gimbal_frame, (int(balloon_px[0]), int(balloon_px[1])), 3, color, -1)
        elif seg is not None:
            # Edge marker where the ray exits the frame toward the balloon.
            ex, ey = seg[1]
            cv2.circle(gimbal_frame, (int(ex), int(ey)), 12, color, 3)


def _draw_belly_canopy_axis(gimbal_frame, s, gw, gh, color):
    """Draw the infinite belly-canopy axis (the body X-Z plane) projected into the
    gimbal view, through the nose.  This is a property of the AIRFRAME and exists
    whenever the gimbal is viewed - independent of any balloon, vision tracking, or
    flight mode.  Anchored at the raw (unclamped) nose projection."""
    fy_scale = getattr(s, 'gimbal_fy_scale', 1.0)
    raw_nose = _project_body_vec_to_gimbal_pixel_raw(
        (1.0, 0.0, 0.0), s.gimbal_bearing_pan, s.gimbal_bearing_tilt, gw, gh, fy_scale)
    if raw_nose is None:
        return
    anchor_x, anchor_y = raw_nose
    margin = 90.0 - abs(s.gimbal_bearing_tilt)
    if margin <= 2.0:
        return
    th = math.radians(min(40.0, margin * 0.6))
    pp = _project_body_vec_to_gimbal_pixel_raw(
        (math.cos(th), 0.0, math.sin(th)), s.gimbal_bearing_pan, s.gimbal_bearing_tilt, gw, gh, fy_scale)
    pm = _project_body_vec_to_gimbal_pixel_raw(
        (math.cos(th), 0.0, -math.sin(th)), s.gimbal_bearing_pan, s.gimbal_bearing_tilt, gw, gh, fy_scale)
    if pp is None or pm is None:
        return
    dx = pp[0] - pm[0]
    dy = pp[1] - pm[1]
    d = math.hypot(dx, dy)
    if d <= 1e-6:
        return
    ux, uy = dx / d, dy / d
    clipped = _clip_line_to_rect(anchor_x, anchor_y, ux, uy, gw, gh)
    if clipped is not None:
        pa, pb = clipped
        _draw_dashed_line(gimbal_frame,
                          (int(pa[0]), int(pa[1])),
                          (int(pb[0]), int(pb[1])), color)


def _clip_line_to_rect(px, py, ux, uy, gw, gh):
    """Clip the infinite line through (px,py) with unit direction (ux,uy) to the
    rectangle [0,gw]x[0,gh].  Returns ((x0,y0),(x1,y1)) of the segment inside the
    frame, or None if the line misses the frame entirely.  Parametric form
    P = (px,py) + t*(ux,uy); find the t-range where P is inside the rectangle."""
    t_min = -1e9
    t_max = 1e9
    # For each boundary pair, narrow [t_min, t_max].  (Liang-Barsky.)
    for p, q in (
        (-ux, px - 0),        # x >= 0
        (ux, gw - px),        # x <= gw
        (-uy, py - 0),        # y >= 0
        (uy, gh - py),        # y <= gh
    ):
        if abs(p) < 1e-12:
            if q < 0:
                return None    # line parallel to this edge and outside it
            continue
        t = q / p
        if p < 0:
            if t > t_max:
                return None
            if t > t_min:
                t_min = t
        else:
            if t < t_min:
                return None
            if t < t_max:
                t_max = t
    if t_min > t_max:
        return None
    return ((px + t_min * ux, py + t_min * uy),
            (px + t_max * ux, py + t_max * uy))


def _clip_segment_to_rect(x0, y0, x1, y1, gw, gh):
    """Clip the FINITE segment (x0,y0)->(x1,y1) to the rectangle [0,gw]x[0,gh].
    Returns ((ax,ay),(bx,by)) preserving direction (first point stays nearest the
    original start), or None if the segment lies entirely outside.  Unlike
    _clip_line_to_rect this keeps the segment finite (parametric t in [0,1]), so a
    nose->balloon ray ends AT the balloon (or at the frame edge toward it), never
    running edge-to-edge.  (Liang-Barsky on a segment.)"""
    dx = x1 - x0
    dy = y1 - y0
    t0, t1 = 0.0, 1.0
    for p, q in (
        (-dx, x0 - 0),        # x >= 0
        (dx, gw - x0),        # x <= gw
        (-dy, y0 - 0),        # y >= 0
        (dy, gh - y0),        # y <= gh
    ):
        if abs(p) < 1e-12:
            if q < 0:
                return None    # parallel to this edge and outside it
            continue
        t = q / p
        if p < 0:
            if t > t1:
                return None
            if t > t0:
                t0 = t
        else:
            if t < t0:
                return None
            if t < t1:
                t1 = t
    if t0 > t1:
        return None
    return ((x0 + t0 * dx, y0 + t0 * dy),
            (x0 + t1 * dx, y0 + t1 * dy))


def _project_body_vec_to_gimbal_pixel_raw(vec_body, gimbal_pan_deg, gimbal_tilt_deg,
                                          gw, gh, fy_scale=1.0):
    """Project a body-frame vector to a RAW unclamped (px, py) float pixel on
    the gimbal camera, or None if behind.  Used for ray geometry, where
    clamping the anchor to the frame edge would distort the line.  Thin
    wrapper around geometry.project_body_vec_to_pixel - the projection math
    is shared with every other camera projection site."""
    status, px, py = project_body_vec_to_pixel(
        vec_body, gimbal_pan_deg, gimbal_tilt_deg, gw, gh,
        GIMBAL_FOV_DEG, fy_scale)
    return None if status == 'behind' else (px, py)


def _offscreen_nose_marker_on_belly(s, gw, gh):
    """When the nose projects OFF-frame, return the pixel where the belly-canopy
    axis crosses the frame edge on the nose's side, so the red off-frame nose
    marker sits exactly on that axis (not on an independently-clamped edge point,
    which drifts off the axis the further off-screen the nose is).  Returns
    (px, py) or None if it can't be computed."""
    fy_scale = getattr(s, 'gimbal_fy_scale', 1.0)
    raw_nose = _project_body_vec_to_gimbal_pixel_raw(
        (1.0, 0.0, 0.0), s.gimbal_bearing_pan, s.gimbal_bearing_tilt, gw, gh, fy_scale)
    if raw_nose is None:
        return None
    anchor_x, anchor_y = raw_nose
    # Belly-canopy direction in the image (same construction as the belly ray).
    margin = 90.0 - abs(s.gimbal_bearing_tilt)
    if margin <= 2.0:
        return None
    th = math.radians(min(40.0, margin * 0.6))
    pp = _project_body_vec_to_gimbal_pixel_raw(
        (math.cos(th), 0.0, math.sin(th)), s.gimbal_bearing_pan, s.gimbal_bearing_tilt, gw, gh, fy_scale)
    pm = _project_body_vec_to_gimbal_pixel_raw(
        (math.cos(th), 0.0, -math.sin(th)), s.gimbal_bearing_pan, s.gimbal_bearing_tilt, gw, gh, fy_scale)
    if pp is None or pm is None:
        return None
    ux, uy = _unit(pp[0] - pm[0], pp[1] - pm[1])
    clipped = _clip_line_to_rect(anchor_x, anchor_y, ux, uy, gw, gh)
    if clipped is None:
        return None
    # Pick the clipped endpoint nearer the (off-screen) raw nose anchor - that's
    # the frame edge on the nose's side of the axis.
    a, b = clipped
    da = (a[0] - anchor_x) ** 2 + (a[1] - anchor_y) ** 2
    db = (b[0] - anchor_x) ** 2 + (b[1] - anchor_y) ** 2
    return a if da <= db else b


def _project_nose_to_gimbal_pixel(gimbal_pan_deg, gimbal_tilt_deg, gw, gh, fy_scale=1.0):
    """Project the aircraft NOSE (body +X axis) onto the gimbal camera image.

    The nose is [1, 0, 0] in the body frame, so this is just
    project_body_vec_to_pixel((1, 0, 0), ...) with edge-clamping for the 'off'
    case (so a marker can be drawn at the nearest edge pointing at the nose).

    Returns (status, px, py):
      status 'on'      -> nose is within the frame; (px, py) are valid pixels.
      status 'off'     -> nose is in front but outside the frame; (px, py) are
                          CLAMPED to the nearest edge.
      status 'behind'  -> nose is behind the gimbal (>90 deg off); px, py = None.
    """
    status, raw_px, raw_py = project_body_vec_to_pixel(
        (1.0, 0.0, 0.0), gimbal_pan_deg, gimbal_tilt_deg, gw, gh,
        GIMBAL_FOV_DEG, fy_scale)
    if status == 'behind':
        return ('behind', None, None)
    px = int(max(0, min(gw - 1, raw_px)))
    py = int(max(0, min(gh - 1, raw_py)))
    return (status, px, py)


def render_gimbal_camera(gimbal_frame, s: HudSnapshot):
    """Draw overlays on the gimbal camera frame and display it.

    Reads only HudSnapshot fields.  The align-counter STATE MACHINE lives in the
    controller; here we only display s.align_seconds.
    """
    MAGENTA = (255, 0, 255)
    YELLOW = (0, 255, 255)
    gh, gw = gimbal_frame.shape[:2]
    gcx, gcy = gw / 2, gh / 2

    # Centre mark fixed at centre - where the gimbal is pointing (RED cursor,
    # gimbal camera only; distinct from the GREEN nose cursor).
    cv2.line(gimbal_frame, (int(gcx - 20), int(gcy)), (int(gcx + 20), int(gcy)), (0, 0, 255), 2)
    cv2.line(gimbal_frame, (int(gcx), int(gcy - 20)), (int(gcx), int(gcy + 20)), (0, 0, 255), 2)

    # Flight-path (velocity vector) marker - SAME body vector as the nose camera,
    # but projected through the gimbal's current pan/tilt so it lands correctly
    # in the slewed view.  Shown whenever the flight path is in the gimbal FOV
    # (not gated on the debug switch); primary flight information.
    _draw_flight_path_marker_gimbal(
        gimbal_frame, getattr(s, 'flight_path_body', None),
        s.gimbal_bearing_pan, s.gimbal_bearing_tilt, gw, gh,
        getattr(s, 'gimbal_fy_scale', 1.0))

    # Debug overlay (tuner switch 'Draw Debug' = 1): the nose projection marker,
    # the belly-canopy axis, and the balloon ray/marker are all controlled by it.
    nose_status, nose_px, nose_py = _project_nose_to_gimbal_pixel(
        s.gimbal_bearing_pan, s.gimbal_bearing_tilt, gw, gh,
        getattr(s, 'gimbal_fy_scale', 1.0))
    if getattr(s, 'draw_roll_error', 0):
        # GREEN cursor (centre mark): where the aircraft nose (body +X) projects in
        # the gimbal view - matching the green nose centre mark on the nose camera.
        # 'on'     -> green centre mark at the projected pixel.
        # 'off'    -> hollow green ring placed ON the belly axis at the frame edge.
        # 'behind' -> a green 'X' at centre-top (gimbal slewed >90 deg off nose).
        NOSE_BGR = (0, 255, 0)
        if nose_status == 'on':
            cv2.line(gimbal_frame, (nose_px - 20, nose_py), (nose_px + 20, nose_py), NOSE_BGR, 2)
            cv2.line(gimbal_frame, (nose_px, nose_py - 20), (nose_px, nose_py + 20), NOSE_BGR, 2)
        elif nose_status == 'off':
            mk = _offscreen_nose_marker_on_belly(s, gw, gh)
            mx, my = mk if mk is not None else (nose_px, nose_py)
            cv2.circle(gimbal_frame, (int(mx), int(my)), 10, NOSE_BGR, 3)
        else:  # behind
            bx, by = int(gw / 2), 14
            cv2.line(gimbal_frame, (bx - 12, by - 12), (bx + 12, by + 12), NOSE_BGR, 3)
            cv2.line(gimbal_frame, (bx - 12, by + 12), (bx + 12, by - 12), NOSE_BGR, 3)

        # The belly-canopy axis is a property of the AIRFRAME and is drawn whenever
        # the nose is in view - regardless of any balloon, vision tracking, or mode.
        # The balloon rays + markers are drawn only when a balloon is actually live
        # (balloon chaser/ptest, or vision tracking active): the CONTROL balloon in yellow
        # (projected control_balloon_body) and the ACTUAL balloon in green (the raw
        # detection pixel).  Both go through _draw_roll_error_rays so each gets a
        # matching ray from the projected nose - the green here REPLACES the
        # standalone raw-detection circle below when Draw Debug is on.
        if nose_status != 'behind':
            _draw_belly_canopy_axis(gimbal_frame, s, gw, gh, BELLY_AXIS_BGR)
            if getattr(s, 'balloon_projection_live', False):
                _draw_roll_error_rays(gimbal_frame, s, nose_px, nose_py, gw, gh,
                                      getattr(s, 'control_balloon_body', None),
                                      BALLOON_CONTROL_BGR, use_detection_pixel=False)
                # Actual balloon (green): marker pinned to the raw detection pixel,
                # with a green ray from the projected nose.  Drawn last so the
                # green actual-balloon marker sits on top of the yellow one.
                _draw_roll_error_rays(gimbal_frame, s, nose_px, nose_py, gw, gh,
                                      getattr(s, 'last_balloon_body', None),
                                      BALLOON_ACTUAL_BGR, use_detection_pixel=True)

    if not s.gimbal_locked:
        if s.vision_tracking and s.detection_pixel:
            # Raw-detection (actual-balloon) circle.  When Draw Debug is ON the
            # debug block above already draws this in green via _draw_roll_error_rays
            # (marker + ray), so only draw the standalone circle when debug is OFF
            # to avoid stacking two circles on the same pixel.
            if not getattr(s, 'draw_roll_error', 0):
                dx, dy = s.detection_pixel
                cv2.circle(gimbal_frame, (dx, dy), 20, (0, 0, 255), 2)
                cv2.circle(gimbal_frame, (dx, dy), 3, (0, 0, 255), -1)
        elif not s.vision_tracking:
            mx, my = s.mouse_x, s.mouse_y
            dot_color = (0, 0, 255) if s.tracking else (0, 255, 255)
            cv2.circle(gimbal_frame, (mx, my), 10, dot_color, 2)
            cv2.circle(gimbal_frame, (mx, my), 3, dot_color, -1)

    # Angle readout.
    cv2.putText(gimbal_frame,
                f"Gimbal  Pan: {s.gimbal_bearing_pan:+6.1f}  Tilt: {s.gimbal_bearing_tilt:+6.1f}",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, MAGENTA, 2)

    # Top-right yellow block: roll error, airspeed, altitude, pitch, roll.  The
    # mode is on the bottom-left line only.
    # Roll-error readouts are only meaningful when there is a live balloon
    # (vision tracking or balloon chaser/ptest); otherwise last_balloon_body is stale, so
    # show N/A rather than a meaningless angle.
    balloon_live = bool(getattr(s, 'balloon_projection_live', False))
    if not balloon_live:
        roll_err_text = "Roll err: N/A"
    else:
        try:
            ty = float(s.last_balloon_body[1])
            tz = float(s.last_balloon_body[2])
            # Unwrapped: 0 = balloon on the belly, +-180 = on the canopy.
            roll_err_deg = math.degrees(math.atan2(ty, tz))
            roll_err_text = f"Roll err: {roll_err_deg:+6.2f} deg"
        except (AttributeError, TypeError, IndexError):
            roll_err_text = "Roll err:    --- deg"
    # The balloon's sideways angle off the NOSE, -asin(ty)*57.3, from the
    # companion's own balloon geometry.  The roll cascade closes the same angle
    # measured from the flight path (lua_pan_err_deg); the two agree while nose
    # and path agree.  Distinct from "Roll err" above, which is the geometric
    # body Y-Z plane angle (wrapped atan2); the two agree at alignment (ty->0)
    # but differ off-axis.
    if not balloon_live:
        ctrl_roll_err_text = "Ctrl roll err: N/A"
    else:
        try:
            ty_c = float(s.last_balloon_body[1])
            ctrl_roll_err_deg = -math.degrees(math.asin(max(-1.0, min(1.0, ty_c))))
            ctrl_roll_err_text = f"Ctrl roll err: {ctrl_roll_err_deg:+6.2f} deg"
        except (AttributeError, TypeError, IndexError):
            ctrl_roll_err_text = "Ctrl roll err:    --- deg"
    try:
        airspeed_text = f"Airspeed: {s.aircraft_airspeed:5.1f} m/s"
    except (AttributeError, TypeError):
        airspeed_text = "Airspeed:   --- m/s"
    try:
        alt_text = f"Alt: {s.aircraft_relative_alt:6.1f} m"
    except (AttributeError, TypeError):
        alt_text = "Alt:     --- m"
    try:
        pitch_deg = math.degrees(float(s.aircraft_att[1]))
        pitch_text = f"Pitch: {_dir_ud(pitch_deg)} {pitch_deg:+6.1f} deg"
    except (AttributeError, TypeError, IndexError):
        pitch_text = "Pitch:    --- deg"
    try:
        roll_deg = math.degrees(float(s.aircraft_att[0]))
        roll_text = f"Roll: {_dir_rl(roll_deg)} {roll_deg:+6.1f} deg"
    except (AttributeError, TypeError, IndexError):
        roll_text = "Roll:     --- deg"

    _status_lines = [roll_err_text, ctrl_roll_err_text, airspeed_text,
                     alt_text, pitch_text, roll_text]
    for line_idx, text in enumerate(_status_lines):
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        x = gw - tw - 10
        y = 25 + line_idx * 25
        cv2.putText(gimbal_frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, YELLOW, 2)

    # ---- Aileron / elevator blocks -----------------------------------------
    #
    # There is no requested surface PWM: the FC is commanded in deg/s and
    # converts to deflection itself.  The demand lines carry the FC's own
    # values, and "Actual" (SERVO_OUTPUT_RAW) is the only view of what the
    # surfaces are doing.
    _p1 = _pwm_to_deflection_pct(s.servo_aileron, s.servo1_trim,
                                 s.servo1_min, s.servo1_max)
    _p2 = _pwm_to_deflection_pct(s.servo_elevator, s.servo2_trim,
                                 s.servo2_min, s.servo2_max)
    if s.elevon_mixing and _p1 is not None and _p2 is not None:
        # ELEVONS.  Servos 1/2 are not aileron and elevator - they are the left
        # and right elevon, and BOTH carry pitch and roll.  Reporting them
        # channel-by-channel shows two identical numbers in level flight and
        # reads as "holding 18% aileron AND 18% elevator" when the aircraft is
        # in fact holding pure pitch trim with zero roll.
        #   roll  = differential (they oppose)
        #   pitch = common mode  (they move together)
        real_pct = (_p1 - _p2) / 2.0
        elev_real_pct = (_p1 + _p2) / 2.0
        _roll_label, _pitch_label = "Elevon Roll", "Elevon Pitch"
    else:
        real_pct, elev_real_pct = _p1, _p2
        _roll_label, _pitch_label = "Aileron", "Elevator"

    # Read down the chain; the GAPS are the diagnosis.
    #
    #   Requested Bank    what the law asked for, before BANK_LIM_DEG
    #   Clamped Bank      what survived it - this is what the rate loop tracks
    #   Commanded Roll    the rate handed to set_target_throttle_rate_rpy
    #   Actual Aileron    what the surface did
    #
    # Requested != Clamped means the bank limit is cutting the law off and no
    # gain change will help. Commanded != Actual means the FC's inner rate loop
    # is not delivering - saturated, under-gained, or out of authority at this
    # airspeed. Two numbers side by side say it; a marker would only repeat what
    # the reader can already see.
    #
    # The BANK clip is the one that fires in ordinary balloon chaser (45 deg against a
    # law that routinely asks for more). The ACRO RATE clamp at 180 deg/s is a
    # runaway backstop that never fires in normal flight, which is why it is
    # logged (lua_omega_*_presat) but not shown.
    #
    # Pitch has no bank stage - it is a rate law throughout - so it gets three
    # lines, not four.
    #
    # YAW's margin is NOT the roll story.  ACRO_YAW_RATE is 45 against a law
    # that asks for 26.5 at the bank limit at cruise and 33 at AIRSPEED_MIN -
    # a factor of 1.4, not the factor of eight roll enjoys.  That number is the
    # coordinated turn's own rotation, so it is not headroom anyone can trade
    # away.  Actual Rudder is the V-TAIL SPLIT: SERVO2
    # and SERVO4 carry elevator + rudder mixed, so rudder is the DIFFERENCE
    # between them and elevator is their mean.  A split of zero means no
    # rudder, whatever the commanded rate says.
    _req_bank = s.lua_roll_cmd_presat
    _cl_bank  = (None if s.lua_requested_roll is None
                 else math.degrees(s.lua_requested_roll))
    req_roll_line   = _cmd_line("Requested Bank", _req_bank, _dir_rl, "deg")
    clamp_roll_line = _cmd_line("Clamped Bank", _cl_bank, _dir_rl, "deg")
    cmd_roll_line   = _cmd_line("Commanded Roll", s.lua_omega_x, _dir_rl, "deg/s")

    # One line: what was actually commanded. The SPLIT between the computed
    # coordination term and the trim is in the CSV, not here - on the HUD
    # the question is what the rudder was asked for, and whether the cyan
    # marker moved to the boresight answers whether it worked.
    req_yaw_line   = _cmd_line("Yaw Rate Command", s.lua_yaw_rate_cmd_dps,
                               _dir_rl, "deg/s")

    _req_pitch = (None if s.lua_requested_pitch is None
                  else math.degrees(s.lua_requested_pitch))
    req_pitch_line = _cmd_line("Requested Pitch", _req_pitch, _dir_ud, "deg")
    cmd_pitch_line = _cmd_line("Commanded Pitch", s.lua_omega_y, _dir_ud, "deg/s")

    aileron_lines  = [req_roll_line, clamp_roll_line, cmd_roll_line]
    elevator_lines = [req_pitch_line, cmd_pitch_line]
    rudder_lines   = [req_yaw_line]
    # V-tail: rudder is the DIFFERENCE of the two ruddervators, in the same
    # deflection-% units the aileron and elevator lines use.
    _s2 = _pwm_to_deflection_pct(s.servo_elevator, s.servo2_trim,
                                 s.servo2_min, s.servo2_max)
    _s4 = _pwm_to_deflection_pct(s.servo_ch4, s.servo2_trim,
                                 s.servo2_min, s.servo2_max)
    if _s2 is not None and _s4 is not None:
        # V-tail decomposition.  ArduPlane's V-tail mixer applies MIXING_GAIN
        # (default 0.5):
        #     ch2 = (elev + rud) * 0.5      ch4 = (elev - rud) * 0.5
        # so ch2 - ch4 = rud exactly - the raw difference of the two travel
        # percentages already equals the commanded rudder percent 1:1, and
        # halving it would read half the true demand.
        _rud = _s2 - _s4
        rudder_lines.append(f"Actual Rudder: {_dir_rl(_rud)} "
                            f"{_rud:+6.1f} %")
    else:
        rudder_lines.append("Actual Rudder: --")
    aileron_lines.append(_format_aileron_line("Actual " + _roll_label, real_pct))
    elevator_lines.append(_format_elevator_line("Actual " + _pitch_label,
                                                elev_real_pct))

    # Running cursor rather than hardcoded offsets: the roll block is 4 lines
    # and the pitch block 3, and a fixed `y = 200 + 25 + 25 + ...` breaks the
    # moment either changes.  It starts one blank row below the status block.
    _y = 25 + (len(_status_lines) + 1) * 25
    for _block in (aileron_lines, elevator_lines, rudder_lines):
        if not _block:
            continue
        for text in _block:
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.putText(gimbal_frame, text, (gw - tw - 10, _y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, YELLOW, 2)
            _y += 25
        _y += 25                     # blank line between the two blocks

    # Controller error readouts (body-frame pan/tilt to balloon), top-left.
    # Both readouts below are measured from the SAME balloon vector, differing only
    # in the reference direction: the boresight for Nose, the velocity vector for
    # Path.  Fetched once, here, so neither can drift onto a different source.
    fpb = getattr(s, 'flight_path_body', None)
    balloon_body = getattr(s, 'last_balloon_body', None)

    # NOSE Error: the balloon's bearing measured from the BORESIGHT, which is
    # balloon_body itself - the boresight is (0,0) by definition.  Computed the same
    # way as Path Error one block below, from the same balloon vector, differing
    # only in the reference direction.
    #
    # NOT the FC's steering error (lua_steering_pan).  That is the cascade's
    # error signal, and it is the FLIGHT-PATH lateral error - track.lua reads
    # the balloon in the path frame, because the lateral axis aims where the
    # aircraft TRAVELS.  Reading it here made "Nose Error Pan" and "Path Error Pan"
    # display the same number while the boresight and the flight-path marker were
    # visibly degrees apart on screen.
    #
    # Two labels must never share one source.  If a readout claims a reference
    # direction, it has to be measured from that direction.
    # SIGN: MARKER relative to BALLOON, not the reverse.  Right and above are
    # POSITIVE, left and below NEGATIVE - so the number says where the thing you
    # are steering has got to, and its sign is the direction you must correct
    # AGAINST.  vector_to_pan_tilt returns the balloon's bearing FROM the marker,
    # which is the opposite sense, hence the negation.  The boresight is (0,0)
    # by definition, so nose error is simply minus the balloon's bearing.
    nose_pan_err = nose_tilt_err = None
    if balloon_body is not None:
        _tp, _tt = vector_to_pan_tilt(balloon_body)
        nose_pan_err, nose_tilt_err = -_tp, -_tt

    # THE THREE GAPS, all in the same "A minus B" sense, so each name reads as
    # what it is: positive means A sits right of / above B.
    #
    #   Nose to Path    nose - path     the skew the rudder closes
    #   Nose to Balloon  nose - balloon   where the boresight is pointing
    #   Path to Balloon  path - balloon   the one that decides whether you reach it
    #
    # The third is the sum of the first two with the nose cancelling, so any two
    # determine the third - they are shown together because reading which pair
    # is non-zero is the diagnosis.
    fp_pan = fp_tilt = None
    if fpb is not None:
        fp_pan, fp_tilt = vector_to_pan_tilt(fpb)

    # Nose to Path: the boresight is (0,0) by definition, so this is minus the
    # flight-path marker's own bearing.  "---" at or below
    # FLIGHT_PATH_MIN_SPEED_MPS, where the cyan marker is not drawn either.
    if s.tracking:
        if fp_pan is None:
            n2p_txt = "Nose to Path Error    Pan:   ---  Tilt:   ---"
        else:
            n2p_txt = (f"Nose to Path Error    Pan: {-fp_pan:+6.1f}"
                       f"  Tilt: {-fp_tilt:+6.1f}")
        cv2.putText(gimbal_frame, n2p_txt,
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, MAGENTA, 2)

    if s.tracking and nose_pan_err is not None:
        cv2.putText(gimbal_frame,
                    f"Nose to Balloon Error  Pan: {nose_pan_err:+6.1f}"
                    f"  Tilt: {nose_tilt_err:+6.1f}",
                    (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, MAGENTA, 2)

    # Path to Balloon: the quantity that decides whether the aircraft reaches the
    # balloon, and the one every alignment check applies ALIGN_TOL_DEG to
    # (controller.alignment_errors).
    path_pan_err = path_tilt_err = None
    if fp_pan is not None and balloon_body is not None:
        tg_pan, tg_tilt = vector_to_pan_tilt(balloon_body)
        path_pan_err = fp_pan - tg_pan
        path_tilt_err = fp_tilt - tg_tilt
    if s.tracking:
        if path_pan_err is None:
            p2t_txt = "Path to Balloon Error  Pan:   ---  Tilt:   ---"
        else:
            p2t_txt = (f"Path to Balloon Error  Pan: {path_pan_err:+6.1f}"
                       f"  Tilt: {path_tilt_err:+6.1f}")
        cv2.putText(gimbal_frame, p2t_txt,
                    (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, MAGENTA, 2)

    # "Nose Aligned": the balloon's bearing FROM THE BORESIGHT, which is balloon_body
    # itself - the boresight is (0,0) by definition.  Symmetric with Path
    # Aligned, which is the same bearing measured from the flight-path vector.
    #
    # NOT the FC's steering error (lua_steering_pan).  That is the CASCADE
    # RESIDUAL - whatever the control law is currently driving to zero - and the
    # law drives the FLIGHT PATH.  The residual therefore nulls while the nose is
    # still alpha above the balloon and, with any sideslip, beta to the side of
    # it: the lamp lit on convergence of a different quantity than the one it
    # names.  It also reads ~0 whenever the law is converged for ANY reason,
    # which is exactly when a nose-alignment indicator should be most trusted.
    nose_err_mag = None
    if balloon_body is not None:
        _n_pan, _n_tilt = vector_to_pan_tilt(balloon_body)
        nose_err_mag = math.hypot(_n_pan, _n_tilt)
    if s.tracking and nose_err_mag is not None and nose_err_mag < ALIGN_TOL_DEG:
        cv2.putText(gimbal_frame, "Nose Aligned",
                    (10, gh - 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # "Path Aligned": the SAME alignment test for the CYAN flight-path marker - it
    # lights when the cyan marker (where the aircraft is actually GOING) sits on the
    # balloon.  Reuses the path error computed above; one line (25 px) above Nose
    # Aligned, and naturally hidden when the marker is undefined (path err is None).
    if (s.tracking and path_pan_err is not None
            and math.hypot(path_pan_err, path_tilt_err) < ALIGN_TOL_DEG):
        cv2.putText(gimbal_frame, "Path Aligned",
                    (10, gh - 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Shared counter at y=100, occupied by EITHER the inter-test Straight timer or
    # the active-alignment Aligned timer - they are mutually exclusive (tracking is
    # paused while straightening, so only one can be counting).  Each is drawn ONLY
    # while it is actively counting; nothing is shown when neither counts (no idle text).
    if s.straighten_elapsed_s is not None:
        # Straightening between tests AND the belly is wings-level: count the held-level
        # time from 0 toward STRAIGHT_HOLD_S (resolver returns None until level).
        cv2.putText(gimbal_frame, f"Straight: {s.straighten_elapsed_s:.1f}s",
                    (10, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.6, MAGENTA, 2)
    elif s.tracking and s.aligned_elapsed_s is not None and s.aligned_elapsed_s > 0.0:
        # Tracking AND the flight path is within ALIGN_TOL_DEG of the balloon: how
        # long it has held (climbs toward the lock dwell).
        cv2.putText(gimbal_frame, f"Path Aligned: {s.aligned_elapsed_s:.1f}s",
                    (10, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.6, MAGENTA, 2)

    # Balloon slant range, y=150: the slot below the alignment counter, same
    # column and font.  A FIXED slot, not one that flows up when the counter is
    # absent - the counter blinks with alignment, and a readout that jumped a
    # line each time would be hard to read.  Shown ONLY in 'p' and 'x'.  Keyed
    # on the mode, not the sensor: the sweep and the trim map switch the sensor
    # on for their oracle balloon, which has no distance worth reading.
    if s.mode in ('ptest', 'balloon_chaser'):
        if s.balloon_range_m is None:
            # No finite balloon position: an infinite Distance m is a
            # bearing, and real vision is bearing-only.
            rng_txt = f"Balloon Distance: {'Infinity':>8}"
        else:
            rng_txt = f"Balloon Distance: {s.balloon_range_m:8.1f} m"
        cv2.putText(gimbal_frame, rng_txt,
                    (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, MAGENTA, 2)

    # Per-test seconds counter and run banner, shown only while a run in the
    # 'autotune' regime (the sweep, plant-ID, the trim map) is flying, in the
    # slot the Balloon Distance takes in 'p' and 'x'.  The seconds counter times
    # the current test toward the run's own timeout (the sweep's ALIGN_TIMEOUT_S,
    # the trim map's CELL_TIMEOUT_S) and freezes when the test ends.  The banner
    # names the run, not the regime.
    if s.mode == 'autotune':
        y_at = 150
        if s.autotune_test_seconds is not None:
            cv2.putText(gimbal_frame, f"Test: {s.autotune_test_seconds:.1f}s",
                        (10, y_at), cv2.FONT_HERSHEY_SIMPLEX, 0.6, MAGENTA, 2)
            y_at += 25
        if s.autotune_elapsed_s is not None:
            secs = int(s.autotune_elapsed_s)
            mmss = "%02d:%02d" % (secs // 60, secs % 60)
            cv2.putText(gimbal_frame, f"{_mode_name(s)}  {mmss}  {s.autotune_status or ''}",
                        (10, y_at), cv2.FONT_HERSHEY_SIMPLEX, 0.6, MAGENTA, 2)

    cv2.putText(gimbal_frame,
                f"Mode: {_mode_name(s)}  [s]straight [x]balloon chaser [p]spec-test [t]autotune  arrows: manual",
                (10, gh - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.50, MAGENTA, 2)

    # Log-tick counter (bottom-right).
    tick_text = f"Log tick: {s.csv_tick_count}"
    (tw, th), _ = cv2.getTextSize(tick_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    cv2.putText(gimbal_frame, tick_text, (gw - tw - 10, gh - 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, YELLOW, 2)

    # "Align:" seconds counter (display only; state machine is in the controller).
    if s.align_seconds is not None:
        align_text = f"Align: {s.align_seconds:5.2f}s"
        (tw2, _), _ = cv2.getTextSize(align_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.putText(gimbal_frame, align_text, (gw - tw2 - 10, gh - 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, YELLOW, 2)

    cv2.imshow('Gimbal Camera', gimbal_frame)

