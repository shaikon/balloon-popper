-- Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
--
-- geometry.lua - port of the hot-path subset of Companion/geometry.py
--
-- Only the seven functions the steering core reaches are here.  The rest of
-- geometry.py is display/projection work that stays on the companion.
--
-- ============================================================================
-- TWO PYTHON/LUA TRAPS LIVE IN THIS FILE.  Read before editing.
-- ============================================================================
--
-- 1. wrap180 MUST use math.fmod, NOT the % operator.
--
--    Python  math.fmod(-10, 360) = -10      (sign of the DIVIDEND, C fmod)
--    Python  -10 % 360           = 350      (sign of the DIVISOR, floored)
--    Lua     math.fmod(-10, 360) = -10      (matches Python's fmod)
--    Lua     -10 % 360           = 350      (matches Python's %)
--
--    geometry.py uses math.fmod.  Reaching for `%` because it reads more
--    cleanly silently changes every wrapped angle in the system.
--
-- 2. Rotation matrices are returned as SIX/NINE SCALARS, not tables.
--
--    A Lua table per tick is heap churn at loop rate, and heap pressure
--    presents as "exceeded time limit" during GC - a symptom with no visible
--    connection to its cause.  Callers destructure the return values.
--
-- ============================================================================
-- CONVENTION (identical to the Python, do not "improve" it)
--   Rotation matrices map WORLD -> BODY:  v_body = R * v_world
--   NED world frame: +z is DOWN.
--   pan: 0 = forward, positive = right.   tilt: 0 = level, positive = up.
-- ============================================================================

local M = {}

local rad = math.rad
local deg = math.deg
local sin, cos, asin, atan = math.sin, math.cos, math.asin, math.atan
local sqrt, fmod, abs = math.sqrt, math.fmod, math.abs

-- ---------------------------------------------------------------------------
--- ZYX world->body rotation from Euler angles (radians).
--- Returns the nine elements in row-major order:
---   r00 r01 r02  r10 r11 r12  r20 r21 r22
function M.euler_to_rotation_matrix(roll, pitch, yaw)
  local cr, sr = cos(roll),  sin(roll)
  local cp, sp = cos(pitch), sin(pitch)
  local cy, sy = cos(yaw),   sin(yaw)
  return  cy * cp,              sy * cp,             -sp,
          cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr,
          cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr
end

-- ---------------------------------------------------------------------------
--- World->body rotation from an ArduPilot ATTITUDE_QUATERNION (w,x,y,z).
--- Same mapping as euler_to_rotation_matrix, but without the +-90 deg pitch
--- singularity - which is the entire reason the controller prefers it.
function M.quat_to_rotation_matrix(w, x, y, z)
  local n = sqrt(w * w + x * x + y * y + z * z)
  if n < 1e-12 then
    return 1, 0, 0,  0, 1, 0,  0, 0, 1      -- identity, matching np.eye(3)
  end
  w, x, y, z = w / n, x / n, y / n, z / n
  return 1 - 2 * (y * y + z * z),  2 * (x * y + z * w),      2 * (x * z - y * w),
         2 * (x * y - z * w),      1 - 2 * (x * x + z * z),  2 * (y * z + x * w),
         2 * (x * z + y * w),      2 * (y * z - x * w),      1 - 2 * (x * x + y * y)
end

--- Apply a row-major world->body matrix to (vx,vy,vz).  Returns three scalars.
function M.rotate(r00, r01, r02, r10, r11, r12, r20, r21, r22, vx, vy, vz)
  return r00 * vx + r01 * vy + r02 * vz,
         r10 * vx + r11 * vy + r12 * vz,
         r20 * vx + r21 * vy + r22 * vz
end

-- ---------------------------------------------------------------------------
--- pan/tilt (degrees) -> body-frame unit vector.  Full sphere, nothing to
--- clamp or reject (C-04).  The only degeneracy is the pole, where pan has no
--- effect - intrinsic to any two-angle map of a sphere.
function M.pan_tilt_to_vector(pan_deg, tilt_deg)
  local pan  = rad(pan_deg)
  local tilt = rad(tilt_deg)
  local ct = cos(tilt)
  return ct * cos(pan), ct * sin(pan), -sin(tilt)
end

--- Body-frame vector -> canonical (pan, tilt) in degrees, with
--- pan in (-180, 180] and tilt in [-90, 90].
--- NOTE: math.atan(y, x) IS atan2 in Lua 5.3+.  `math.atan2` was deprecated in
--- 5.3 and REMOVED in 5.4 - using it would work in ArduPilot's bundled 5.3 and
--- break on any 5.4 desktop harness, so never use it.
function M.vector_to_pan_tilt(vx, vy, vz)
  local c = vz
  if c > 1.0 then c = 1.0 elseif c < -1.0 then c = -1.0 end   -- np.clip
  return deg(atan(vy, vx)), deg(-asin(c))
end

-- ---------------------------------------------------------------------------
--- Wrap degrees to (-180, +180].  See trap 1 in the header.
function M.wrap180(angle_deg)
  local a = fmod(angle_deg + 180.0, 360.0)
  if a <= 0.0 then a = a + 360.0 end
  return a - 180.0
end

-- ---------------------------------------------------------------------------
--- World-frame (NED) unit vector from heading + elevation (radians).
function M.yaw_pitch_to_world_vec(yaw_rad, pitch_rad)
  local cp, sp = cos(pitch_rad), sin(pitch_rad)
  return cp * cos(yaw_rad), cp * sin(yaw_rad), -sp
end

-- ---------------------------------------------------------------------------
--- Vertical focal-length correction.  DISPLAY ONLY - carried across so the
--- module is complete, but the steering core never calls it (C-02 forbids
--- deriving balloon geometry from the image).
function M.aspect_fy_scale(native_w, native_h, display_w, display_h, fallback)
  fallback = fallback or 1.0
  if native_w and native_h and display_w and display_h
     and native_w ~= 0 and display_w ~= 0
     and native_h > 0 and display_h > 0 then
    local display_aspect = display_w / display_h
    if display_aspect > 0 then
      return (native_w / native_h) / display_aspect
    end
  end
  return fallback
end

return M
