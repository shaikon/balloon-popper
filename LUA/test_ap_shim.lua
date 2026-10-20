-- Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
--
-- Conformance test for ap_shim.lua.
--
-- The shim is about to be the foundation the whole Lua port is written against.
-- If its semantics differ from ArduPilot's in any way, the port will validate
-- perfectly on a laptop and misbehave on the aircraft - the worst possible
-- failure mode, because the harness will say PASS.
--
-- So: assert the semantics that MATTER, especially the ones that differ from
-- what a Lua programmer would naturally assume.
--
-- Run:  lua5.4 LUA/test_ap_shim.lua

package.path = "LUA/?.lua;" .. package.path
local ap = require("ap_shim").install()

local fails, checks = {}, 0
local function check(name, cond, detail)
  checks = checks + 1
  print(string.format("  [%s] %s%s", cond and "PASS" or "FAIL", name,
                      detail and ("   " .. detail) or ""))
  if not cond then fails[#fails + 1] = name end
end

local function close(a, b, tol)
  return math.abs(a - b) <= (tol or 1e-12)
end

print("\n" .. string.rep("=", 74))
print("ap_shim CONFORMANCE")
print(string.rep("=", 74))

-- ---------------------------------------------------------------------------
print("\n1. Vector3f FIELD ACCESS (ArduPilot style: v:x() get, v:x(n) set)")
local v = Vector3f(1.0, 2.0, 3.0)
check("v:x() reads", close(v:x(), 1.0), string.format("%.1f", v:x()))
v:x(7.5)
check("v:x(n) writes", close(v:x(), 7.5))
check("setter returns nothing", select("#", v:y(1.0)) == 0)

-- ---------------------------------------------------------------------------
print("\n2. IN-PLACE vs ALLOCATING (the distinction the hot path lives on)")
local a = Vector3f(3.0, 4.0, 0.0)
check("length()", close(a:length(), 5.0))
local ret = a:normalize()
check("normalize() returns NOTHING", ret == nil)
check("normalize() mutated in place", close(a:length(), 1.0)
      and close(a:x(), 0.6) and close(a:y(), 0.8))

local b = Vector3f(1.0, 0.0, 0.0)
local c = b:scale(2.0)
check("scale() returns a NEW vector", close(c:x(), 2.0) and close(b:x(), 1.0))
local d = Vector3f(1.0, 2.0, 3.0)
local e = d:copy()
e:x(99.0)
check("copy() is a deep copy", close(d:x(), 1.0) and close(e:x(), 99.0))

-- ---------------------------------------------------------------------------
print("\n3. THE VOID-RETURN TRAP  (q:earth_to_body mutates and returns nil)")
local q = Quaternion()
q:from_euler(0.0, 0.0, math.pi / 2)          -- yaw +90 deg
local w = Vector3f(1.0, 0.0, 0.0)            -- world north
local got = q:earth_to_body(w)
check("earth_to_body returns NIL", got == nil,
      "code written as `local b = q:earth_to_body(v)` gets nil - by design")
check("earth_to_body rewrote the argument",
      close(w:x(), 0.0, 1e-9) and close(w:y(), -1.0, 1e-9),
      string.format("north -> body (%.6f, %.6f, %.6f)", w:x(), w:y(), w:z()))

-- round trip through a non-trivial attitude
local q2 = Quaternion()
q2:from_euler(0.3, -0.4, 1.1)
local orig = Vector3f(0.36, 0.48, 0.8)
local vv = orig:copy()
q2:earth_to_body(vv)
check("earth_to_body preserves length", close(vv:length(), orig:length(), 1e-12),
      string.format("%.12f", vv:length()))

-- inverse rotation must undo it
local qi = q2:inverse()
qi:earth_to_body(vv)
check("inverse() undoes the rotation",
      close(vv:x(), orig:x(), 1e-9) and close(vv:y(), orig:y(), 1e-9)
      and close(vv:z(), orig:z(), 1e-9))

-- ---------------------------------------------------------------------------
print("\n4. EULER ROUND TRIP")
for _, t in ipairs({ { 0.0, 0.0, 0.0 }, { 0.3, -0.4, 1.1 },
                     { -1.2, 0.9, -2.5 }, { 0.05, 1.5, 0.02 } }) do
  local qq = Quaternion()
  qq:from_euler(t[1], t[2], t[3])
  local ok = close(qq:get_euler_roll(),  t[1], 1e-9)
         and close(qq:get_euler_pitch(), t[2], 1e-9)
         and close(qq:get_euler_yaw(),   t[3], 1e-9)
  check(string.format("euler (%.2f, %.2f, %.2f) round-trips", t[1], t[2], t[3]), ok)
end

-- near-vertical: the case the whole project exists for
local qv = Quaternion()
qv:from_euler(0.2, math.rad(89.0), 0.7)
check("q survives 89 deg pitch", close(qv:get_euler_pitch(), math.rad(89.0), 1e-9),
      string.format("pitch back = %.6f deg", math.deg(qv:get_euler_pitch())))
check("q is unit at 89 deg pitch", close(qv:length(), 1.0, 1e-12))

-- ---------------------------------------------------------------------------
print("\n5. dot / cross / angle")
local i, j = Vector3f(1, 0, 0), Vector3f(0, 1, 0)
check("dot orthogonal = 0", close(i:dot(j), 0.0))
local k = i:cross(j)
check("i x j = k", close(k:x(), 0) and close(k:y(), 0) and close(k:z(), 1))
check("angle(i,j) = pi/2", close(i:angle(j), math.pi / 2, 1e-12))
check("operators + and -",
      close((i + j):x(), 1.0) and close((i - j):y(), -1.0))

-- ---------------------------------------------------------------------------
print("\n6. ALLOCATION ACCOUNTING  (this is how S2 becomes testable)")
ap.alloc_reset()
local scratch = Vector3f(1, 2, 3)
local sq = Quaternion()
check("constructors counted", ap.alloc_count() == 2,
      "counted " .. ap.alloc_count())

-- a deliberately allocation-free block: only in-place ops and field writes
ap.alloc_reset()
for _ = 1, 1000 do
  scratch:x(0.5); scratch:y(0.5); scratch:z(0.5)
  scratch:normalize()
  sq:from_euler(0.1, 0.2, 0.3)
  sq:earth_to_body(scratch)
  local _ = scratch:dot(scratch)
  local _ = scratch:length()
end
check("1000 in-place iterations allocate NOTHING", ap.alloc_count() == 0,
      "counted " .. ap.alloc_count())

-- and the counter really does catch the forbidden calls
ap.alloc_reset()
local _ = scratch:copy()
local _ = scratch:scale(2.0)
local _ = scratch:cross(scratch)
local _ = sq:inverse()
local _ = scratch + scratch
check("copy/scale/cross/inverse/+ ARE caught", ap.alloc_count() == 5,
      "counted " .. ap.alloc_count() .. " - these five are banned in update()")

-- ---------------------------------------------------------------------------
print("\n" .. string.rep("=", 74))
if #fails > 0 then
  print(string.format("CONFORMANCE FAILED: %d of %d checks: %s",
                      #fails, checks, table.concat(fails, ", ")))
  os.exit(1)
end
print(string.format("CONFORMANCE PASSED - %d checks", checks))
print("The shim matches ArduPilot's documented semantics, so Lua validated")
print("on desktop is the same Lua that runs on the flight controller.")
print(string.rep("=", 74))
