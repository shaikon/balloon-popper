-- Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
--
-- ap_shim.lua - desktop stand-in for ArduPilot's Lua userdata types.
--
-- PURPOSE
--   The Lua steering core must be exercisable on a laptop, but the code that is
--   exercised has to be THE SAME CODE that runs on the flight controller.  So
--   the core is written against ArduPilot's real API - Vector3f, Quaternion -
--   and this shim provides that API on desktop.
--
--   On the FC this file is never loaded; ArduPilot's own bindings are already
--   global.  balloon_chaser_core.lua therefore requires it only when it is absent.
--
-- FIDELITY - transcribed from libraries/AP_Scripting/generator/description/bindings.desc
--
--   userdata Vector3f field  x/y/z          read write   -> v:x()  getter, v:x(n) setter
--   userdata Vector3f method length         float
--   userdata Vector3f method normalize      void         <- IN PLACE
--   userdata Vector3f method is_nan/is_inf/is_zero boolean
--   userdata Vector3f method dot            float Vector3f
--   userdata Vector3f method cross          Vector3f Vector3f      <- ALLOCATES
--   userdata Vector3f method scale          Vector3f float         <- ALLOCATES
--   userdata Vector3f method copy           Vector3f               <- ALLOCATES
--   userdata Vector3f method angle          float Vector3f
--   userdata Vector3f operator + and -                             <- ALLOCATE
--
--   userdata Quaternion field  q1..q4       read write
--   userdata Quaternion method length       float
--   userdata Quaternion method normalize    void         <- IN PLACE
--   userdata Quaternion method get_euler_roll/pitch/yaw  float
--   userdata Quaternion method from_euler   void         <- IN PLACE
--   userdata Quaternion method inverse      Quaternion             <- ALLOCATES
--   userdata Quaternion method earth_to_body void Vector3f  <- IN PLACE ON THE ARG
--   userdata Quaternion method to_axis_angle void Vector3f  <- IN PLACE ON THE ARG
--   userdata Quaternion method from_axis_angle / from_angular_velocity void
--   userdata Quaternion operator *                                 <- ALLOCATES
--
-- THE TRAP THIS SHIM IS DESIGNED TO EXPOSE
--   `q:earth_to_body(v)` returns NOTHING and rewrites `v`.  Code written as
--       local body = q:earth_to_body(world)        -- body == nil
--   fails silently in a way that is easy to miss and hard to debug in flight.
--   The shim reproduces the void return exactly so that mistake surfaces on a
--   laptop instead of at 20 m/s.
--
-- ALLOCATION ACCOUNTING
--   Safety requirement S2 is "zero allocation in update()".  ArduPilot's heap
--   is hard to introspect from a script, but every allocation goes through a
--   constructor here, so on desktop it can simply be COUNTED.  That turns S2
--   from an aspiration into an assertion:
--
--       ap.alloc_reset()
--       core.update(...)
--       assert(ap.alloc_count() == 0)

local M = {}

-- ---------------------------------------------------------------------------
-- allocation accounting
-- ---------------------------------------------------------------------------
local _allocs = 0
local _tracking = true

function M.alloc_reset()   _allocs = 0 end
function M.alloc_count()   return _allocs end
function M.alloc_pause()   _tracking = false end
function M.alloc_resume()  _tracking = true end

local function bump()
  if _tracking then _allocs = _allocs + 1 end
end

-- ---------------------------------------------------------------------------
-- Vector3f
-- ---------------------------------------------------------------------------
local V = {}
V.__index = V

local function newv(x, y, z)
  bump()
  return setmetatable({ _x = x or 0.0, _y = y or 0.0, _z = z or 0.0 }, V)
end

-- field accessors: v:x() reads, v:x(n) writes and returns nothing
function V:x(n) if n == nil then return self._x end self._x = n end
function V:y(n) if n == nil then return self._y end self._y = n end
function V:z(n) if n == nil then return self._z end self._z = n end

function V:length()
  return math.sqrt(self._x * self._x + self._y * self._y + self._z * self._z)
end

function V:normalize()                     -- void, in place
  local l = self:length()
  if l > 0 then
    self._x, self._y, self._z = self._x / l, self._y / l, self._z / l
  end
end

function V:is_nan()
  return self._x ~= self._x or self._y ~= self._y or self._z ~= self._z
end

function V:is_inf()
  local inf = math.huge
  return self._x == inf or self._x == -inf or self._y == inf or self._y == -inf
      or self._z == inf or self._z == -inf
end

function V:is_zero()
  return self._x == 0.0 and self._y == 0.0 and self._z == 0.0
end

function V:dot(o) return self._x * o._x + self._y * o._y + self._z * o._z end

function V:cross(o)                        -- ALLOCATES
  return newv(self._y * o._z - self._z * o._y,
              self._z * o._x - self._x * o._z,
              self._x * o._y - self._y * o._x)
end

function V:scale(f)  return newv(self._x * f, self._y * f, self._z * f) end -- ALLOCATES
function V:copy()    return newv(self._x, self._y, self._z) end             -- ALLOCATES

function V:angle(o)                        -- float, radians
  local d = self:dot(o)
  local l = self:length() * o:length()
  if l <= 0 then return 0.0 end
  local c = d / l
  if c > 1.0 then c = 1.0 elseif c < -1.0 then c = -1.0 end
  return math.acos(c)
end

function V:rotate_xy(a)                    -- void, in place
  local c, s = math.cos(a), math.sin(a)
  local x, y = self._x, self._y
  self._x = c * x - s * y
  self._y = s * x + c * y
end

V.__add = function(a, b) return newv(a._x + b._x, a._y + b._y, a._z + b._z) end
V.__sub = function(a, b) return newv(a._x - b._x, a._y - b._y, a._z - b._z) end
V.__tostring = function(v)
  return string.format("Vector3f(%.9g, %.9g, %.9g)", v._x, v._y, v._z)
end

-- ---------------------------------------------------------------------------
-- Quaternion  (q1=w, q2=x, q3=y, q4=z)
-- ---------------------------------------------------------------------------
local Q = {}
Q.__index = Q

local function newq(w, x, y, z)
  bump()
  return setmetatable({ _1 = w or 1.0, _2 = x or 0.0, _3 = y or 0.0,
                        _4 = z or 0.0 }, Q)
end

function Q:q1(n) if n == nil then return self._1 end self._1 = n end
function Q:q2(n) if n == nil then return self._2 end self._2 = n end
function Q:q3(n) if n == nil then return self._3 end self._3 = n end
function Q:q4(n) if n == nil then return self._4 end self._4 = n end

function Q:length()
  return math.sqrt(self._1 * self._1 + self._2 * self._2
                 + self._3 * self._3 + self._4 * self._4)
end

function Q:normalize()                     -- void, in place
  local l = self:length()
  if l > 0 then
    self._1, self._2 = self._1 / l, self._2 / l
    self._3, self._4 = self._3 / l, self._4 / l
  end
end

function Q:inverse()                       -- ALLOCATES
  return newq(self._1, -self._2, -self._3, -self._4)
end

function Q:from_euler(r, p, y)             -- void, in place
  local cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
  local cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
  local cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
  self._1 = cr * cp * cy + sr * sp * sy
  self._2 = sr * cp * cy - cr * sp * sy
  self._3 = cr * sp * cy + sr * cp * sy
  self._4 = cr * cp * sy - sr * sp * cy
end

function Q:get_euler_roll()
  local w, x, y, z = self._1, self._2, self._3, self._4
  return math.atan(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
end

function Q:get_euler_pitch()
  local w, x, y, z = self._1, self._2, self._3, self._4
  local s = 2.0 * (w * y - z * x)
  if s > 1.0 then s = 1.0 elseif s < -1.0 then s = -1.0 end
  return math.asin(s)
end

function Q:get_euler_yaw()
  local w, x, y, z = self._1, self._2, self._3, self._4
  return math.atan(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
end

-- VOID.  Rewrites `v` in place.  Returns nothing - deliberately.
function Q:earth_to_body(v)
  local w, x, y, z = self._1, self._2, self._3, self._4
  -- world->body is the transpose of the body->world rotation matrix
  local r00 = 1.0 - 2.0 * (y * y + z * z)
  local r01 = 2.0 * (x * y + w * z)
  local r02 = 2.0 * (x * z - w * y)
  local r10 = 2.0 * (x * y - w * z)
  local r11 = 1.0 - 2.0 * (x * x + z * z)
  local r12 = 2.0 * (y * z + w * x)
  local r20 = 2.0 * (x * z + w * y)
  local r21 = 2.0 * (y * z - w * x)
  local r22 = 1.0 - 2.0 * (x * x + y * y)
  local vx, vy, vz = v._x, v._y, v._z
  v._x = r00 * vx + r01 * vy + r02 * vz
  v._y = r10 * vx + r11 * vy + r12 * vz
  v._z = r20 * vx + r21 * vy + r22 * vz
end

function Q:to_axis_angle(v)                -- void, in place on v
  local l = math.sqrt(self._2 * self._2 + self._3 * self._3 + self._4 * self._4)
  if l < 1e-12 then v._x, v._y, v._z = 0.0, 0.0, 0.0 return end
  local w = self._1
  if w > 1.0 then w = 1.0 elseif w < -1.0 then w = -1.0 end
  local ang = 2.0 * math.acos(w)
  v._x, v._y, v._z = self._2 / l * ang, self._3 / l * ang, self._4 / l * ang
end

function Q:from_axis_angle(v, ang)         -- void, in place
  local l = v:length()
  if l < 1e-12 then self._1, self._2, self._3, self._4 = 1, 0, 0, 0 return end
  local s = math.sin(ang * 0.5)
  self._1 = math.cos(ang * 0.5)
  self._2 = v._x / l * s
  self._3 = v._y / l * s
  self._4 = v._z / l * s
end

function Q:from_angular_velocity(v, dt)    -- void, in place
  local scratch_len = math.sqrt(v._x * v._x + v._y * v._y + v._z * v._z) * dt
  if scratch_len < 1e-12 then
    self._1, self._2, self._3, self._4 = 1, 0, 0, 0
    return
  end
  local s = math.sin(scratch_len * 0.5) / (scratch_len / dt)
  self._1 = math.cos(scratch_len * 0.5)
  self._2 = v._x * s
  self._3 = v._y * s
  self._4 = v._z * s
end

Q.__mul = function(a, b)                   -- ALLOCATES
  return newq(a._1 * b._1 - a._2 * b._2 - a._3 * b._3 - a._4 * b._4,
              a._1 * b._2 + a._2 * b._1 + a._3 * b._4 - a._4 * b._3,
              a._1 * b._3 - a._2 * b._4 + a._3 * b._1 + a._4 * b._2,
              a._1 * b._4 + a._2 * b._3 - a._3 * b._2 + a._4 * b._1)
end

Q.__tostring = function(q)
  return string.format("Quaternion(%.9g, %.9g, %.9g, %.9g)",
                       q._1, q._2, q._3, q._4)
end

-- ---------------------------------------------------------------------------
-- install as globals, matching ArduPilot
-- ---------------------------------------------------------------------------
function M.install()
  if rawget(_G, "Vector3f") == nil then
    _G.Vector3f = function(x, y, z) return newv(x, y, z) end
  end
  if rawget(_G, "Quaternion") == nil then
    _G.Quaternion = function(w, x, y, z) return newq(w, x, y, z) end
  end
  return M
end

M.Vector3f = newv
M.Quaternion = newq
return M
