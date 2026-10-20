# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""
Pure-math helpers used by the control core.  No I/O, no hardware, no
adapters.  Trivially unit-testable.

"""
import math
import numpy as np


def euler_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX rotation matrix from Euler angles (radians).
    Transforms a vector from world frame to body frame: v_body = R @ v_world."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy*cp,               sy*cp,              -sp    ],
        [cy*sp*sr - sy*cr,    sy*sp*sr + cy*cr,    cp*sr ],
        [cy*sp*cr + sy*sr,    sy*sp*cr - cy*sr,    cp*cr ],
    ])


def quat_to_rotation_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    """World->body rotation matrix from an attitude quaternion (w,x,y,z), i.e.
    v_body = R @ v_world - the SAME mapping euler_to_rotation_matrix returns, so
    it is a drop-in replacement that agrees to machine precision away from the
    Euler singularity.

    The quaternion is the ArduPilot ATTITUDE_QUATERNION convention: (w,x,y,z) is
    the body->NED(world) rotation M, so world->body is M^T (built directly below).
    Unlike the Euler path this does NOT gimbal-lock at +-90deg pitch: the logged
    Euler (roll,yaw) slew at hundreds of deg/s through a vertical dive and make the
    body-frame balloon vector convulse, whereas the quaternion stays well-conditioned
    and the body vector varies smoothly.
    """
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = w/n, x/n, y/n, z/n
    # M^T (world->body): transpose of the standard body->world quaternion matrix.
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y + z*w),       2*(x*z - y*w)    ],
        [2*(x*y - z*w),       1 - 2*(x*x + z*z),   2*(y*z + x*w)    ],
        [2*(x*z + y*w),       2*(y*z - x*w),       1 - 2*(x*x + y*y)],
    ])


def pan_tilt_to_vector(pan_deg: float, tilt_deg: float) -> np.ndarray:
    """Convert pan/tilt (degrees) to a unit direction vector in body frame.
    Pan: 0=forward, positive=right.  Tilt: 0=level, positive=up.

    This is the spherical (azimuth/elevation) map, and it is the ONE geometry the
    whole system uses: the gimbal-encoder body direction (the real C-02 sensing
    path in the adapter) and the controller's vector_to_pan_tilt are this same
    convention, so the synthetic-injector test path uses it too.

    Every (pan, tilt) pair maps to a valid unit vector - the FULL sphere, with no
    restricted region and nothing to clamp or reject (C-04).  pan and tilt are
    both full-range rotation angles and either can reach any hemisphere, so a
    balloon directly astern is BOTH pan=180,tilt=0 (around the side) AND
    pan=0,tilt=180 (up and over the top) - identical directions.  Behind-and-up
    45 deg is pan=180,tilt=+45 or equivalently pan=0,tilt=+135.

    The only unavoidable degeneracy is the pole: at tilt=+-90 (zenith/nadir) the
    down component is +-1 and pan has no effect (every pan gives the same point).
    That is intrinsic to any two-angle map of a sphere and is physically
    harmless - a heading is meaningless when you are pointed straight up or down.
    The forward/behind split is carried by BOTH cos(pan) and cos(tilt), so unlike
    a lateral-offset parameterization there is no 'budget' coupling that forbids
    combining a large pan with a large tilt.
    """
    pan  = math.radians(pan_deg)
    tilt = math.radians(tilt_deg)
    return np.array([
        math.cos(tilt) * math.cos(pan),
        math.cos(tilt) * math.sin(pan),
       -math.sin(tilt),
    ])


def vector_to_pan_tilt(v: np.ndarray) -> tuple[float, float]:
    """Convert a unit direction vector in body frame to pan/tilt (degrees).
    Inverse of pan_tilt_to_vector, returning the CANONICAL representative with
    pan in (-180, +180] and tilt in [-90, +90] (so the astern direction comes
    back as pan=180, tilt=0 rather than pan=0, tilt=180 - the same point).  At
    the poles pan is set to 0 since it is degenerate there."""
    pan  = math.atan2(v[1], v[0])
    tilt = -math.asin(np.clip(v[2], -1.0, 1.0))
    return math.degrees(pan), math.degrees(tilt)


def wrap180(angle_deg: float) -> float:
    """Wrap an angle in degrees to (-180, +180]."""
    a = math.fmod(angle_deg + 180.0, 360.0)
    if a <= 0.0:
        a += 360.0
    return a - 180.0


def aspect_fy_scale(native_w, native_h, display_w, display_h,
                    fallback: float = 1.0) -> float:
    """Vertical focal-length correction (fy/fx) for a camera whose native render
    resolution is resized to a *different* display resolution before projection.

    DISPLAY ONLY.  A camera renders with square pixels at its native resolution
    (fx_native == fy_native).  When the frame is resized to a display resolution
    with a different aspect ratio, the horizontal and vertical stretch factors
    differ, so on the display frame fx_display != fy_display.  A pinhole
    projection that derives a single focal length from the display width and the
    horizontal FOV gets the horizontal axis right but over- or under-scales the
    vertical axis.  The per-axis ratio works out to:

        fy_display / fx_display = native_aspect / display_aspect

    Returns that ratio, or `fallback` when any dimension is missing/invalid (so
    the caller decides the no-information default - 1.0 for "assume no resize").
    """
    try:
        if (native_w and native_h and display_w and display_h
                and native_h > 0 and display_h > 0):
            native_aspect = native_w / native_h
            display_aspect = display_w / display_h
            if display_aspect > 0:
                return native_aspect / display_aspect
    except (TypeError, ZeroDivisionError):
        pass
    return fallback


def yaw_pitch_to_world_vec(yaw_rad: float, pitch_rad: float) -> np.ndarray:
    """Build a world-frame (NED) unit vector from heading + elevation.
        x =  cos(pitch) * cos(yaw)
        y =  cos(pitch) * sin(yaw)
        z = -sin(pitch)              (NED: positive z is down)
    """
    cp = math.cos(pitch_rad)
    sp = math.sin(pitch_rad)
    cy = math.cos(yaw_rad)
    sy = math.sin(yaw_rad)
    return np.array([cp * cy, cp * sy, -sp])


# -----------------------------------------------------------------------------
# Unified pinhole projection primitives.
#
# All HUD overlays and sim-side detection synthesis project a 3D direction
# (body-frame or world-frame) onto a camera image plane.  Before consolidation
# the same pinhole math lived in FIVE different call sites with four slightly
# different return contracts and a recurring fy_scale oversight (the
# non-uniform native->display resize correction was missed in some sites and
# applied in others).  These two primitives are the single source of truth:
#
#   rotate_body_to_camera_frame(body_vec, pan_deg, tilt_deg)
#       Apply the gimbal pan/tilt rotation to take a body-frame vector into
#       the camera frame (camera +X = optical axis, +Y = right, +Z = down).
#       Identity rotation for the nose camera (pan=tilt=0).
#
#   pinhole_project(cam_vec, frame_w, frame_h, hfov_deg, fy_scale=1.0)
#       Pinhole projection of a camera-frame vector onto a frame of size
#       (frame_w, frame_h) with horizontal FOV `hfov_deg`.  fy_scale corrects
#       the vertical axis for non-uniform native->display resize (1.0 = no
#       correction).  Returns ('behind', None, None) when the vector is at or
#       behind the camera plane (camera_x <= 0); otherwise ('on', px, py) when
#       the pixel falls inside the frame and ('off', px, py) when it doesn't.
#       Pixels are raw floats - callers clamp or use them for ray construction
#       as needed.
#
# All higher-level adapters (project_body_vec_to_pixel, project_nose_to_pixel,
# project_world_vec_to_gimbal_pixel) are thin wrappers around these two.
# -----------------------------------------------------------------------------

def rotate_body_to_camera_frame(body_vec, gimbal_pan_deg: float,
                                gimbal_tilt_deg: float):
    """Rotate a body-frame vector by the gimbal pan/tilt into camera frame.

    Pan is applied first (rotation about body Z), then tilt (rotation about
    the post-pan Y).  The nose camera is rigid to the airframe, so callers
    pass pan=tilt=0 to get an identity rotation (camera frame == body frame).

    Returns a 3-tuple (camera_x, camera_y, camera_z).  Camera frame is
    +X=optical axis, +Y=right, +Z=down - the standard pinhole convention.
    """
    bx, by, bz = float(body_vec[0]), float(body_vec[1]), float(body_vec[2])
    pan_r = math.radians(gimbal_pan_deg)
    tilt_r = math.radians(gimbal_tilt_deg)
    cp, sp = math.cos(pan_r), math.sin(pan_r)
    x1 = cp * bx + sp * by
    y1 = -sp * bx + cp * by
    z1 = bz
    ct, st = math.cos(tilt_r), math.sin(tilt_r)
    x2 = ct * x1 - st * z1
    y2 = y1
    z2 = st * x1 + ct * z1
    return (x2, y2, z2)


def pinhole_project(cam_vec, frame_w: int, frame_h: int,
                    hfov_deg: float, fy_scale: float = 1.0):
    """Project a camera-frame vector to a (status, px, py) triple.

    Args:
        cam_vec:    (x, y, z) in camera frame (+X = optical axis).
        frame_w,
        frame_h:    pixel dimensions of the display frame.
        hfov_deg:   horizontal field-of-view in degrees.
        fy_scale:   vertical focal-length correction for non-uniform
                    native->display resize (fy_display / fx_display).  1.0
                    means the frame has the camera's native aspect ratio and
                    no correction is needed.

    Returns:
        ('behind', None, None) if cam_vec is at or behind the camera plane
        (cam_vec[0] <= 0).  Otherwise ('on' | 'off', px, py) where px, py are
        raw floats: 'on' if the pixel is within the frame bounds, 'off' if it
        falls outside.  Callers decide whether to clamp, draw an edge marker,
        or use the raw float for ray construction.
    """
    cx_val, cy_val, cz_val = float(cam_vec[0]), float(cam_vec[1]), float(cam_vec[2])
    if cx_val <= 0.0:
        return ('behind', None, None)
    fx = (frame_w / 2.0) / math.tan(math.radians(hfov_deg / 2.0))
    fy = fx * fy_scale
    cx0 = frame_w / 2.0
    cy0 = frame_h / 2.0
    px = cx0 + fx * (cy_val / cx_val)
    py = cy0 + fy * (cz_val / cx_val)
    on = (0 <= px <= frame_w - 1) and (0 <= py <= frame_h - 1)
    return ('on' if on else 'off', px, py)


def project_body_vec_to_pixel(body_vec, gimbal_pan_deg: float,
                              gimbal_tilt_deg: float,
                              frame_w: int, frame_h: int,
                              hfov_deg: float, fy_scale: float = 1.0):
    """Convenience wrapper: body-frame vector through a (possibly slewing)
    camera onto its image plane.  Combines rotate_body_to_camera_frame +
    pinhole_project.  Returns the same (status, px, py) triple.  For the
    fixed nose camera, pass gimbal_pan_deg=gimbal_tilt_deg=0."""
    cam_vec = rotate_body_to_camera_frame(body_vec, gimbal_pan_deg, gimbal_tilt_deg)
    return pinhole_project(cam_vec, frame_w, frame_h, hfov_deg, fy_scale)


def project_world_vec_to_gimbal_pixel(world_vec, aircraft_att,
                                      gimbal_pan_deg: float,
                                      gimbal_tilt_deg: float,
                                      frame_w: int, frame_h: int,
                                      hfov_deg: float, fy_scale: float = 1.0):
    """Convenience wrapper: world-frame vector -> body frame (via attitude) ->
    camera frame (via gimbal) -> pixel.  Used by the sim's synthetic-detection
    path (the sim KNOWS the world balloon and asks 'where would it land?').
    Returns the same (status, px, py) triple."""
    R = euler_to_rotation_matrix(*aircraft_att)
    body_vec = R @ np.asarray(world_vec, dtype=float)
    return project_body_vec_to_pixel(body_vec, gimbal_pan_deg, gimbal_tilt_deg,
                                     frame_w, frame_h, hfov_deg, fy_scale)


def roll_align_error_deg(balloon_body) -> float:
    """Geometric roll-misalignment angle (deg) - a CSV/HUD diagnostic.

    The un-weighted, un-wrapped signed angle in the body Y-Z plane from belly
    (+body-Z) to the balloon: degrees(atan2(ty, tz)).  0 = belly-aligned,
    +-180 = inverted (balloon on the canopy side).  Returns 0.0 if the balloon is
    degenerate (on the roll axis).  Pure geometry, for the log and the HUD; it
    does not drive the control law.
    """
    ty = float(balloon_body[1])
    tz = float(balloon_body[2])
    if abs(ty) < 1e-9 and abs(tz) < 1e-9:
        return 0.0
    return math.degrees(math.atan2(ty, tz))
