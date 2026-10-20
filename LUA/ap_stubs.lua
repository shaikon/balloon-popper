-- Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
--
-- ap_stubs.lua - desktop stand-ins for the ArduPilot SINGLETONS.
--
-- ap_shim.lua covers the userdata TYPES (Vector3f, Quaternion).  This covers
-- the singletons the FC shell calls: ahrs, mount, vehicle, serial, param,
-- gcs, millis, Parameter.
--
-- Purpose: run balloon_chaser_fc.lua end to end on a laptop.  Without this the shell
-- is the one file in the system that cannot be exercised until it is on
-- hardware - which is exactly the wrong place to discover a nil method name or
-- a swapped argument.
--
-- The stubs RECORD what was called so a test can assert on it.

local ap = require("ap_shim").install()
local M = {}

M.log = { rate_cmds = {}, mount_rates = {}, writes = {}, texts = {},
          params_set = {}, nav_enable = {}, rudder_offsets = {} }

local sim = {
  now_ms = 0,
  roll = 0.0, pitch = 0.0, yaw = 0.0,
  p = 0.0, q = 0.0, r = 0.0,
  vn = 18.0, ve = 0.0, vd = 0.0,
  eas = 18.0, eas_valid = true, eas2tas = 1.0,
  ax = 0.0, ay = 0.0, az = -9.80665, accel_valid = true,
  wind_n = 0.0, wind_e = 0.0, wind_d = 0.0,
  mount_pan = 0.0, mount_tilt = 0.0,
  mode = 4,            -- ACRO
  nav_owned = false,
  rx = "",
}
M.sim = sim

function M.advance(ms) sim.now_ms = sim.now_ms + ms end
function M.feed(bytes) sim.rx = sim.rx .. bytes end

-- millis() returns an object with :toint()
local ms_obj = {}
ms_obj.__index = ms_obj
function ms_obj:toint() return sim.now_ms end
_G.millis = function() return setmetatable({}, ms_obj) end

-- ahrs
-- The attitude is sim.quat {w, x, y, z} where a test sets one - a plant's
-- own quaternion, exact through the vertical, where the Euler angles fold -
-- and otherwise built from sim.roll, sim.pitch and sim.yaw.
_G.ahrs = {}
function _G.ahrs:get_quaternion()
  if sim.quat then return Quaternion(sim.quat[1], sim.quat[2], sim.quat[3], sim.quat[4]) end
  local q = Quaternion()
  q:from_euler(sim.roll, sim.pitch, sim.yaw)
  return q
end
function _G.ahrs:get_gyro() return Vector3f(sim.p, sim.q, sim.r) end
-- Euler attitude in RADIANS.  The coordination feedforward is built from these
-- two, so without them the whole yaw law reads zero.
-- EKF wind solution in NED, m/s.  Never nil: zero before the estimate settles.
-- The flight code does not read it; test_balloon_chaser_fc.lua sets it to show the
-- lateral marker does not move with it.
function _G.ahrs:wind_estimate() return Vector3f(sim.wind_n, sim.wind_e, sim.wind_d) end
function _G.ahrs:get_roll_rad()  return sim.roll  end
function _G.ahrs:get_pitch_rad() return sim.pitch end
function _G.ahrs:get_yaw_rad()   return sim.yaw   end
function _G.ahrs:get_velocity_NED() return Vector3f(sim.vn, sim.ve, sim.vd) end
-- SINGLE return, number or nil - NOT a (success, value) pair.  bindings.desc
-- reads `boolean float'Null`, the generator's shape for a C++
-- `bool airspeed_EAS(float &out)`: the bool decides whether Lua sees the float
-- or nil and never reaches the script.  ArduPilot's docs.lua is explicit
-- (`---@return number|nil`).
--
-- Returning `true, sim.eas` here hands the script a BOOLEAN as its airspeed.
-- Every comparison against it throws, sample_sensors_and_report_scaler's pcall
-- swallows the error into a GCS message, and the speed scaler, the
-- accelerometers and both yaw feedforwards go dead in every Lua test - silently.
-- A stub that mocks a binding WRONGLY is worse than one that does not mock it:
-- the path looks covered.
function _G.ahrs:airspeed_EAS() if sim.eas_valid == false then return nil end return sim.eas end
-- A plain float, never nil (bindings.desc: `method get_EAS2TAS float`).
function _G.ahrs:get_EAS2TAS() return sim.eas2tas end

-- BODY-frame SPECIFIC FORCE, which is what the real binding returns: level
-- flight reads about (0, 0, -9.81), and accel_y is 0 in a coordinated turn at
-- any bank.  Drivable from a test, including a failed read (nil).
function _G.ahrs:get_accel()
  if sim.accel_valid == false then return nil end
  return Vector3f(sim.ax, sim.ay, sim.az)
end

-- mount
_G.mount = {}
-- Five arguments, matching bindings.desc:
--   set_rate_target(instance, roll, pitch, yaw, earth_frame)
function _G.mount:set_rate_target(inst, roll, pitch, yaw, earth_frame)
  assert(earth_frame ~= nil,
         "set_rate_target needs 5 args: instance, roll, pitch, yaw, earth_frame")
  M.log.mount_rates[#M.log.mount_rates + 1] = { roll, pitch, yaw }
end
function _G.mount:get_attitude_euler(inst)
  return 0.0, sim.mount_tilt, sim.mount_pan
end

-- vehicle
-- Servo channel access, used only by the manual surface override.  find_channel
-- maps a SERVOn_FUNCTION to the channel carrying it; the stub answers for the
-- V-tail layout this airframe uses (aileron on 1, ruddervators on 2 and 4) and
-- records what was written so a test can read it back.
_G.SRV_Channels = {}
_G.srv_written = {}
function _G.SRV_Channels:find_channel(fn)
  local map = { [4] = 0, [79] = 1, [80] = 3 }
  return map[fn]
end
function _G.SRV_Channels:set_output_pwm_chan_timeout(chan, pwm, timeout_ms)
  _G.srv_written[chan] = { pwm = pwm, timeout_ms = timeout_ms }
end
-- What the autopilot is currently driving a function to.  The override reads
-- this ONCE, before it writes, to recover the tail axis it is not taking.
_G.srv_current = { [4] = 1500, [79] = 1500, [80] = 1500 }
function _G.SRV_Channels:get_output_pwm(fn) return _G.srv_current[fn] end
-- The scaled demand per function (+-4500, throttle in percent), as ArduPlane's
-- rate controllers last set it.  A function nothing sets reads 0.
_G.srv_scaled = { [4] = 0.0, [19] = 0.0, [21] = 0.0, [70] = 0.0 }
function _G.SRV_Channels:get_output_scaled(fn) return _G.srv_scaled[fn] or 0.0 end

_G.vehicle = {}
function _G.vehicle:set_target_throttle_rate_rpy(thr, r, p, y)
  M.log.rate_cmds[#M.log.rate_cmds + 1] = { thr, r, p, y }
end
-- WITHOUT THIS THE WHOLE ACTUATION PATH IS UNTESTED.  balloon_chaser_fc calls
-- set_rudder_offset on every commanded tick, immediately before
-- set_target_throttle_rate_rpy.  Absent, the call raises "attempt to call a nil
-- value", step()'s pcall swallows it into a GCS message, and the shell test
-- reports "0 commands" rather than an error - so neither the rate command nor
-- the yaw law is exercised at all.
--
-- Mirrors the FC: the pair is (deflection percent, run_yaw_rate_controller),
-- and BOTH are recorded.  The second is not a detail - it decides whether the
-- rate demand reaches the rudder at all, and false makes the FC reset the yaw
-- rate controller's integrator instead.
function _G.vehicle:set_rudder_offset(pct, run_yaw_rate_controller)
  M.log.rudder_offsets[#M.log.rudder_offsets + 1] = { pct, run_yaw_rate_controller }
end
function _G.vehicle:get_mode() return sim.mode end
-- Mirrors ArduPlane: succeeds only when the mode passed is the CURRENT mode and
-- that mode is scripting-capable (CIRCLE 1, STABILIZE 2, ACRO 4, FBWA 5,
-- FBWB 6, CRUISE 7, LOITER 12).
local _SCRIPTABLE = { [1]=true, [2]=true, [4]=true, [5]=true, [6]=true,
                      [7]=true, [12]=true }
function _G.vehicle:nav_scripting_enable(mode)
  M.log.nav_enable[#M.log.nav_enable + 1] = mode
  -- mode 255 is used as an explicit RELEASE: it never matches the current
  -- mode, so ArduPilot sets enabled = false and hands the surfaces back.
  local ok = (mode == sim.mode) and (_SCRIPTABLE[mode] or false)
  sim.nav_owned = ok
  return ok
end

-- serial
local port = {}
port.__index = port
function port:begin(b) end
function port:set_flow_control(f) end
-- available() is uint32_t in bindings.desc, and ArduPilot returns uint32_t as a
-- USERDATA with :toint().  Returning a plain number here let the script pass it
-- straight to readstring on desktop, while the real binding rejected it.
local _u32 = {}
_u32.__index = _u32
function _u32:toint() return self.v end
function port:available() return setmetatable({v = #sim.rx}, _u32) end
function port:read()
  if #sim.rx == 0 then return -1 end
  local b = string.byte(sim.rx, 1)
  sim.rx = string.sub(sim.rx, 2)
  return b
end
-- Real binding names, from bindings.desc: writestring / readstring, with NO
-- underscore between the words.  Match them EXACTLY: an underscored stub name
-- would pass every desktop test against a method that does not exist on the
-- flight controller, failing only in flight, at 50 Hz.
function port:writestring(s) M.log.writes[#M.log.writes + 1] = s ; return #s end
function port:readstring(n)
  if #sim.rx == 0 then return nil end
  local out = string.sub(sim.rx, 1, n)
  sim.rx = string.sub(sim.rx, n + 1)
  return out
end
_G.serial = {}
function _G.serial:find_serial(i) return setmetatable({}, port) end

-- param
-- The FLOWN values, so the shell exercises the same envelope the aircraft has.
-- ACRO_YAW_RATE is the ceiling the yaw law clamps its command to;
-- AIRSPEED_MIN floors the 1/V in the coordination feedforward, and with
-- AIRSPEED_MAX bounds the speed scaler.  YAW_RATE_ENABLE and YAW_RATE_FF exist
-- so init()'s precondition check sees a healthy FC here and its warnings stay
-- meaningful when they do fire.
local pv = { ACRO_ROLL_RATE = 180.0, ACRO_PITCH_RATE = 90.0, ACRO_YAW_RATE = 45.0,
             AIRSPEED_MIN = 12.0, AIRSPEED_MAX = 22.0,
             YAW_RATE_ENABLE = 1.0, YAW_RATE_FF = 0.924,
             SCALING_SPEED = 15.0, PTCH_RATE_FF = 0.1744, PTCH_RATE_P = 1.0 }
_G.param = {}
function _G.param:get(n) return pv[n] end
function _G.param:set(n, v) pv[n] = v ; M.log.params_set[#M.log.params_set+1] = {n,v} ; return true end

local P = {}
P.__index = P
function P:init(n) self._n = n ; return pv[n] ~= nil end
function P:get() return pv[self._n] end
function P:set(v) pv[self._n] = v ; M.log.params_set[#M.log.params_set+1] = {self._n, v} ; return true end
_G.Parameter = function() return setmetatable({}, P) end

-- gcs
_G.gcs = {}
function _G.gcs:send_text(sev, txt) M.log.texts[#M.log.texts + 1] = txt end

M.param_values = pv
return M
