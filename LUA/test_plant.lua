-- Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
--
-- test_plant.lua - a six-degree-of-freedom model of the simulated Mini Talon,
-- for flying balloon_chaser_fc.lua closed loop on the desktop.
--
-- THE AIRFRAME is Arduplane/SDF/model.sdf's, read from the file at load: every
-- lift-drag element on base_link (the wings, the V-tail and the fuselage's
-- side force), the links' masses and base_link's inertia, and the world's
-- elevation, whose standard air every element flies in.  Each element's force
-- is AltitudeLiftDrag::Update's (upstream LiftDrag's) with the stall left
-- out: past the stall angle the lift keeps its slope.  The propeller is a
-- thrust line through SITL's climb and dive speeds rather than the SDF's two
-- blade elements.  A test can add a steady side force along the body y
-- axis (ac.side_force), standing in for what the propeller's slipstream and
-- torque do to SITL's airframe at high throttle, which the thrust line leaves
-- out.
--
-- THE RATE LOOP is ArduPlane's: AC_PID on the rate error times the speed
-- scaler squared (AP_FW_Controller::_get_rate_out), its FLTT / FLTE / FLTD
-- filters and the SMAX slew limiter (libraries/Filter/SlewLimiter.cpp), with
-- the Acro set (constants.EXPECTED_RATE_PIDS and the FLTE targets), feeding the
-- V-tail mixer.  The rudder is zero, as on the nav_scripting path with
-- run_yaw_rate_controller false.  The surfaces follow after a delay and a lag.
--
-- NOT SITL.  A pass here is evidence about this model.  Two of its numbers are
-- fitted to SITL rather than read from the SDF: MIXING_GAIN, the V-tail
-- mixer's gain on the elevator and rudder demands, sized so this model's trim
-- elevator matches the SITL trim map's; and AIL_GAIN, the aileron's
-- effectiveness, sized so its roll-rate feedforward matches the RLL_RATE_FF
-- ArduPlane AUTOTUNE finds in SITL.
--
-- Frames: NED world, FRD body (the SDF's FLU with y and z negated), the
-- attitude a body-to-NED quaternion (w, x, y, z), as ArduPilot's AHRS.

local M = {}

local sqrt, abs, acos, atan, asin = math.sqrt, math.abs, math.acos, math.atan, math.asin
local sin, cos, exp, min, max = math.sin, math.cos, math.exp, math.min, math.max
local rad, deg, pi = math.rad, math.deg, math.pi

local G0 = 9.80665
M.G0 = G0

local SDF_MODEL = "Arduplane/SDF/model.sdf"
local SDF_WORLD = "Arduplane/SDF/talon_runway.sdf"

-- ---------------------------------------------------------------------------
-- The SDF.  Its XML comments are removed first: several quote earlier values
-- of the very tags read here.
local function read_sdf(path)
  local f = assert(io.open(path, "r"), "test_plant: cannot open " .. path)
  local x = f:read("a")
  f:close()
  return (x:gsub("<!%-%-.-%-%->", ""))
end

local function tag(body, name)
  return body:match("<" .. name .. ">%s*(.-)%s*</" .. name .. ">")
end

local function num(body, name)
  return assert(tonumber(tag(body, name)), "test_plant: no <" .. name .. ">")
end

local function vec(body, name)
  local a, b, c = assert(tag(body, name), "test_plant: no <" .. name .. ">")
                    :match("(%S+)%s+(%S+)%s+(%S+)")
  return tonumber(a), tonumber(b), tonumber(c)
end

local function unit3(x, y, z)
  local n = sqrt(x * x + y * y + z * z)
  return x / n, y / n, z / n
end

local model = read_sdf(SDF_MODEL)
local world = read_sdf(SDF_WORLD)

-- The control surfaces' slots in a plant's joint list.
local SLOT = { left_aileron_joint = 1, right_aileron_joint = 2,
               left_ruddervator_joint = 3, right_ruddervator_joint = 4 }

-- Every lift-drag element on base_link, in FRD.  base_link's inertial pose is
-- its origin, so cp is measured from the centre of gravity.
local EL = {}
for body in model:gmatch('<plugin%s+filename="AltitudeLiftDrag"[^>]*>(.-)</plugin>') do
  if tag(body, "link_name") == "base_link" then
    local cx, cy, cz = vec(body, "cp")
    local fx, fy, fz = unit3(vec(body, "forward"))
    local ux, uy, uz = unit3(vec(body, "upward"))
    fy, fz, uy, uz = -fy, -fz, -uy, -uz
    -- span = forward x upward
    local sx, sy, sz = unit3(fy * uz - fz * uy, fz * ux - fx * uz, fx * uy - fy * ux)
    local joint = tag(body, "control_joint_name")
    EL[#EL + 1] = {
      cx = cx, cy = -cy, cz = -cz, fx = fx, fy = fy, fz = fz,
      ux = ux, uy = uy, uz = uz, sx = sx, sy = sy, sz = sz,
      area = num(body, "area"), a0 = num(body, "a0"), cla = num(body, "cla"),
      cda = num(body, "cda"),
      slot = joint and assert(SLOT[joint], "test_plant: unknown joint " .. joint),
      rad_to_cl = joint and num(body, "control_joint_rad_to_cl") or 0.0,
    }
  end
end
M.n_elements = #EL

-- The mass of every link, and base_link's inertia (the rest are grams on short
-- arms).
local MASS = 0.0
for m in model:gmatch("<mass>%s*(.-)%s*</mass>") do MASS = MASS + tonumber(m) end
local base = assert(model:match('<link name="base_link">(.-)</link>'), "test_plant: no base_link")
local IXX, IYY, IZZ = num(base, "ixx"), num(base, "iyy"), num(base, "izz")
M.mass = MASS

-- The air: the 1976 standard troposphere at the world's elevation, sea-level
-- density RHO0.
local ELEV_M = num(world, "elevation")
local RHO0, T0_K, P0_PA = 1.225, 288.15, 101325.0
local LAPSE_K_PER_M, R_AIR = 0.0065, 287.053
local T_K = T0_K - LAPSE_K_PER_M * ELEV_M
local RHO = P0_PA * (T_K / T0_K) ^ (G0 / (LAPSE_K_PER_M * R_AIR)) / (R_AIR * T_K)
local EAS2TAS = sqrt(RHO0 / RHO)
M.eas2tas = EAS2TAS

-- The propeller: a straight line in airspeed through SITL's 15.6 m/s in a 40 deg
-- climb and 20.1 m/s in a 40 deg dive at 85 % throttle, its pitch speed and
-- slope both proportional to the throttle, as for a blade element.  It pushes
-- along the body x axis at MOTOR_*, and its reaction rolls the airframe.
M.THR_REF = 0.85
local VP_REF, KT_REF = 17.97, 5.24
local MOTOR_X, MOTOR_Z = -0.41, 0.03
local PROP_TORQUE_PER_N = 0.003

-- ArduPlane's mixer and the ArduPilotPlugin's servo-to-joint mapping: a
-- surface at full scaled output (SCALED_FULL) is at JOINT_MAX.
local SCALED_FULL = 4500.0
local JOINT_MAX = rad(30.0)
local MIXING_GAIN = 0.82
local AIL_GAIN = 1.35
local SERVO_TAU_S = 0.005

-- ArduPlane's main loop, the rate loop's rate, and the script's tick.
local DT = 0.0025
M.DT = DT
local DELAY_STEPS = 5                -- the surfaces' transport delay, in main-loop steps

-- The speed scaler (ArduPlane/Attitude.cpp calc_speed_scaler), from the stub
-- parameters the FC script reads too.
local SCALING_SPEED, AIRSPEED_MIN, AIRSPEED_MAX = 15.0, 12.0, 22.0
local MIN_AIRSPEED_MIN = 5.0
M.SCALING_SPEED, M.AIRSPEED_MIN, M.AIRSPEED_MAX = SCALING_SPEED, AIRSPEED_MIN, AIRSPEED_MAX

local function speed_scaler(eas)
  local amin = max(AIRSPEED_MIN, MIN_AIRSPEED_MIN)
  local lo = min(0.5, SCALING_SPEED / (2.0 * AIRSPEED_MAX))
  local hi = max(2.0, SCALING_SPEED / (0.7 * amin))
  if eas <= 1e-4 then return hi end
  local s = SCALING_SPEED / eas
  if s < lo then return lo elseif s > hi then return hi end
  return s
end

local function clip(v, lo, hi)
  if v < lo then return lo elseif v > hi then return hi end
  return v
end

-- ---------------------------------------------------------------------------
-- Rotation: the body-to-NED matrix of a quaternion, row by row.
local function rot(qw, qx, qy, qz)
  return 1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw),
         2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw),
         2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)
end
M.rot = rot

--- ArduPilot's Quaternion::from_euler.
function M.euler_to_q(roll, pitch, yaw)
  local cr, sr = cos(roll / 2), sin(roll / 2)
  local cp, sp = cos(pitch / 2), sin(pitch / 2)
  local cy, sy = cos(yaw / 2), sin(yaw / 2)
  return cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
         cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy
end

--- ArduPilot's Quaternion::to_euler.
function M.q_to_euler(qw, qx, qy, qz)
  return atan(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy)),
         asin(clip(2 * (qw * qy - qz * qx), -1.0, 1.0)),
         atan(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
end

local function q_mul(aw, ax, ay, az, bw, bx, by, bz)
  return aw * bw - ax * bx - ay * by - az * bz,
         aw * bx + ax * bw + ay * bz - az * by,
         aw * by - ax * bz + ay * bw + az * bx,
         aw * bz + ax * by - ay * bx + az * bw
end
M.q_mul = q_mul

-- ---------------------------------------------------------------------------
--- The force and moment about the centre of gravity, body axes, for the body
--- velocity (u, v, w), rates (p, q, r), joints j and throttle thr; and the
--- left wing's angle of attack.
local function forces(u, v, w, p, q, r, j, thr, prop_torque)
  local fx, fy, fz, mx, my, mz = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
  local alpha_wing = 0.0
  for i = 1, #EL do
    local e = EL[i]
    local vx = u + q * e.cz - r * e.cy
    local vy = v + r * e.cx - p * e.cz
    local vz = w + p * e.cy - q * e.cx
    local sp2 = vx * vx + vy * vy + vz * vz
    if sp2 > 1e-4 and vx * e.fx + vy * e.fy + vz * e.fz > 0.0 then
      local spd = sqrt(sp2)
      local vs = vx * e.sx + vy * e.sy + vz * e.sz
      local sin_sw = clip(vs / spd, -1.0, 1.0)
      local cos2 = 1.0 - sin_sw * sin_sw
      local lx, ly, lz = vx - vs * e.sx, vy - vs * e.sy, vz - vs * e.sz
      local ld = sqrt(lx * lx + ly * ly + lz * lz)
      if ld < 1e-9 then ld = 1e-9 end
      local ax = e.sy * lz - e.sz * ly
      local ay = e.sz * lx - e.sx * lz
      local az = e.sx * ly - e.sy * lx
      local an = sqrt(ax * ax + ay * ay + az * az)
      if an < 1e-12 then an = 1e-12 end
      ax, ay, az = ax / an, ay / an, az / an
      local a = acos(clip(ax * e.ux + ay * e.uy + az * e.uz, -1.0, 1.0))
      local alpha = (ax * e.fx + ay * e.fy + az * e.fz >= 0.0) and e.a0 + a or e.a0 - a
      if alpha > 0.5 * pi then alpha = alpha - pi elseif alpha < -0.5 * pi then alpha = alpha + pi end
      if i == 1 then alpha_wing = alpha end
      local qa = 0.5 * RHO * ld * ld * e.area
      local cl = e.cla * alpha * cos2
      if e.slot then cl = cl + e.rad_to_cl * j[e.slot] end
      local lift, drag = cl * qa, abs(e.cda * alpha * cos2) * qa / ld
      local ex, ey, ez = lift * ax - drag * lx, lift * ay - drag * ly, lift * az - drag * lz
      fx, fy, fz = fx + ex, fy + ey, fz + ez
      mx = mx + e.cy * ez - e.cz * ey
      my = my + e.cz * ex - e.cx * ez
      mz = mz + e.cx * ey - e.cy * ex
    end
  end
  local kt = KT_REF * thr / M.THR_REF
  local t = kt * (VP_REF * thr / M.THR_REF - u)
  fx = fx + t
  my = my + MOTOR_Z * t
  mx = mx + prop_torque * t
  return fx, fy, fz, mx, my, mz, alpha_wing
end
M.forces = forces

--- The four joint angles for ArduPlane's scaled aileron, elevator and rudder
--- outputs, written into j.
local function mix(j, ail, elev, rud)
  local vtr = clip((elev - rud) * MIXING_GAIN, -SCALED_FULL, SCALED_FULL)
  local vtl = clip((elev + rud) * MIXING_GAIN, -SCALED_FULL, SCALED_FULL)
  j[1] = -JOINT_MAX * AIL_GAIN * ail / SCALED_FULL
  j[2] = JOINT_MAX * AIL_GAIN * ail / SCALED_FULL
  j[3] = JOINT_MAX * vtl / SCALED_FULL
  j[4] = JOINT_MAX * vtr / SCALED_FULL
  return j
end
M.mix = mix

-- ---------------------------------------------------------------------------
-- ArduPilot's AC_PID as the fixed-wing rate controllers use it, and the slew
-- limiter that modifies its P and D.
local function lp_alpha(f_hz)
  if f_hz <= 0.0 then return 1.0 end
  local rc = 1.0 / (2.0 * pi * f_hz)
  return DT / (DT + rc)
end

local SLEW_WINDOW_MS, SLEW_MOD_GAIN, SLEW_N_EVENTS = 300.0, 1.5, 2
local SLEW_FILTER_HZ, SLEW_TAU_S, SLEW_LIMIT_MULT = 25.0, 1.0, 10.0
local SLEW_LONG_AGO_MS = -1e9

local function new_slew(smax)
  return { smax = smax, a_f = lp_alpha(SLEW_FILTER_HZ), slew = 0.0, last = 0.0,
           max_pos = 0.0, max_neg = 0.0, pos_ms = SLEW_LONG_AGO_MS, neg_ms = SLEW_LONG_AGO_MS,
           out_rate = 0.0, mod_rate = 0.0,
           pos_ev = { SLEW_LONG_AGO_MS, SLEW_LONG_AGO_MS },
           neg_ev = { SLEW_LONG_AGO_MS, SLEW_LONG_AGO_MS },
           pos_i = 0, neg_i = 0, pos_st = false, neg_st = false, now = 0.0 }
end

local function slew_modifier(sl, sample)
  sl.now = sl.now + DT * 1000.0
  local now = sl.now
  sl.slew = sl.slew + sl.a_f * ((sample - sl.last) / DT - sl.slew)
  sl.last = sample
  local sr = sl.slew
  local decay = min(DT, SLEW_TAU_S) / SLEW_TAU_S
  local attack = min(2.0 * decay, 1.0)
  if sr > sl.max_pos then
    sl.max_pos, sl.pos_ms = sr, now
  elseif now - sl.pos_ms > SLEW_WINDOW_MS then
    sl.max_pos = sl.max_pos * (1.0 - decay)
  end
  if -sr > sl.max_neg then
    sl.max_neg, sl.neg_ms = -sr, now
  elseif now - sl.neg_ms > SLEW_WINDOW_MS then
    sl.max_neg = sl.max_neg * (1.0 - decay)
  end
  local raw = 0.5 * (sl.max_pos + sl.max_neg)
  sl.out_rate = min((1.0 - attack) * sl.out_rate + attack * raw, raw)
  if sl.smax <= 0.0 then return 1.0 end
  local cap = SLEW_LIMIT_MULT * sl.smax
  local lim = 0.5 * (min(sl.max_pos, cap) + min(sl.max_neg, cap))
  if not sl.pos_st and sr > sl.smax then
    local idx = (sl.pos_i >= SLEW_N_EVENTS) and 0 or sl.pos_i
    sl.pos_ev[idx + 1] = now
    sl.pos_i, sl.pos_st, sl.neg_st = idx + 1, true, false
  end
  if not sl.neg_st and -sr > sl.smax then
    local idx = (sl.neg_i >= SLEW_N_EVENTS) and 0 or sl.neg_i
    sl.neg_ev[idx + 1] = now
    sl.neg_i, sl.neg_st, sl.pos_st = idx + 1, true, false
  end
  local oldest = min(sl.pos_ev[1], sl.pos_ev[2], sl.neg_ev[1], sl.neg_ev[2], now)
  local mod_in = lim
  local span = (SLEW_N_EVENTS + 1) * SLEW_WINDOW_MS
  if now - oldest > span then
    mod_in = mod_in * exp(-0.001 * (now - oldest - span) / SLEW_TAU_S)
  end
  sl.mod_rate = min((1.0 - attack) * sl.mod_rate + attack * mod_in, mod_in)
  if sl.mod_rate > sl.smax then
    return sl.smax / (sl.smax + SLEW_MOD_GAIN * (sl.mod_rate - sl.smax))
  end
  return 1.0
end

local function new_pid(kp, kff, fltt, flte, fltd, smax)
  return { kp = kp, kff = kff, at = lp_alpha(fltt), ae = lp_alpha(flte), ad = lp_alpha(fltd),
           tgt = 0.0, err = 0.0, err_last = 0.0, der = 0.0, p_info = 0.0, d_info = 0.0,
           slew = new_slew(smax), started = false, dmod = 1.0 }
end

-- One update: desired rate deg/s, measured rad/s; ArduPlane's scaled output.
-- The Acro set has no I and no D gain.
local function pid_out(c, desired_dps, meas_rads, scaler)
  local s2 = scaler * scaler
  local setpoint = rad(desired_dps) * s2
  local meas = meas_rads * s2
  if not c.started then
    c.tgt = setpoint
    c.err = setpoint - meas
    c.err_last = c.err
    c.der = 0.0
    c.started = true
  else
    c.tgt = c.tgt + c.at * (setpoint - c.tgt)
    c.err = c.err + c.ae * ((c.tgt - meas) - c.err)
    c.der = c.der + c.ad * ((c.err - c.err_last) / DT - c.der)
  end
  c.err_last = c.err
  -- the modifier reads the previous update's P and D, in degrees x 100 / 45
  c.dmod = slew_modifier(c.slew, (c.p_info + c.d_info) * 45.0)
  local p = c.kp * c.err * c.dmod
  c.p_info, c.d_info = p, 0.0
  local ff = deg(c.kff * c.tgt / (scaler * EAS2TAS))
  return clip((deg(p) + ff) * 100.0, -SCALED_FULL, SCALED_FULL)
end

-- The Acro set, which ACRO flies (constants.ACRO_DEFAULT_PARAMS).
M.ACRO = { RLL_RATE_P = 0.2, RLL_RATE_FF = 0.0, PTCH_RATE_P = 1.36, PTCH_RATE_FF = 0.0,
           FLTT_HZ = 3.1831, FLTE_HZ = 5.0, FLTD_HZ = 10.0, SMAX = 150.0 }

-- ---------------------------------------------------------------------------
-- The rigid body: state derivatives for velocity (NED), attitude and rates.
local function deriv(ac, vn, ve, vd, qw, qx, qy, qz, p, q, r)
  local r00, r01, r02, r10, r11, r12, r20, r21, r22 = rot(qw, qx, qy, qz)
  local u = r00 * vn + r10 * ve + r20 * vd
  local v = r01 * vn + r11 * ve + r21 * vd
  local w = r02 * vn + r12 * ve + r22 * vd
  local fx, fy, fz, mx, my, mz = forces(u, v, w, p, q, r, ac.j, ac.thr, ac.prop_torque)
  fy = fy + ac.side_force * MASS
  local an = (r00 * fx + r01 * fy + r02 * fz) / MASS
  local ae = (r10 * fx + r11 * fy + r12 * fz) / MASS
  local ad = (r20 * fx + r21 * fy + r22 * fz) / MASS + G0
  local dp = (mx - (q * IZZ * r - r * IYY * q)) / IXX
  local dq = (my - (r * IXX * p - p * IZZ * r)) / IYY
  local dr = (mz - (p * IYY * q - q * IXX * p)) / IZZ
  local dqw = 0.5 * (-qx * p - qy * q - qz * r)
  local dqx = 0.5 * (qw * p + qy * r - qz * q)
  local dqy = 0.5 * (qw * q - qx * r + qz * p)
  local dqz = 0.5 * (qw * r + qx * q - qy * p)
  return an, ae, ad, dqw, dqx, dqy, dqz, dp, dq, dr, fx, fy, fz
end

--- A new aircraft at rest; M.start sets it flying.
function M.new()
  local ac = {
    vn = 0.0, ve = 0.0, vd = 0.0, qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0,
    p = 0.0, q = 0.0, r = 0.0, j = { 0.0, 0.0, 0.0, 0.0 }, thr = M.THR_REF,
    prop_torque = PROP_TORQUE_PER_N, fx = 0.0, fy = 0.0, fz = 0.0,
    side_force = 0.0,                  -- m/s^2 along the body y axis, + toward the right wing
    roll_pid = new_pid(M.ACRO.RLL_RATE_P, M.ACRO.RLL_RATE_FF, M.ACRO.FLTT_HZ,
                       M.ACRO.FLTE_HZ, M.ACRO.FLTD_HZ, M.ACRO.SMAX),
    pitch_pid = new_pid(M.ACRO.PTCH_RATE_P, M.ACRO.PTCH_RATE_FF, M.ACRO.FLTT_HZ,
                        M.ACRO.FLTE_HZ, M.ACRO.FLTD_HZ, M.ACRO.SMAX),
    delay = {}, jcmd = { 0.0, 0.0, 0.0, 0.0 },
    ail_out = 0.0, elev_out = 0.0,     -- the rate loop's last scaled outputs
  }
  return ac
end

local function rk4(ac)
  local h = DT
  local vn, ve, vd = ac.vn, ac.ve, ac.vd
  local qw, qx, qy, qz = ac.qw, ac.qx, ac.qy, ac.qz
  local p, q, r = ac.p, ac.q, ac.r
  local a1, b1, c1, w1, x1, y1, z1, p1, q1, r1, fx, fy, fz =
    deriv(ac, vn, ve, vd, qw, qx, qy, qz, p, q, r)
  local k = 0.5 * h
  local a2, b2, c2, w2, x2, y2, z2, p2, q2, r2 =
    deriv(ac, vn + k * a1, ve + k * b1, vd + k * c1, qw + k * w1, qx + k * x1,
          qy + k * y1, qz + k * z1, p + k * p1, q + k * q1, r + k * r1)
  local a3, b3, c3, w3, x3, y3, z3, p3, q3, r3 =
    deriv(ac, vn + k * a2, ve + k * b2, vd + k * c2, qw + k * w2, qx + k * x2,
          qy + k * y2, qz + k * z2, p + k * p2, q + k * q2, r + k * r2)
  local a4, b4, c4, w4, x4, y4, z4, p4, q4, r4 =
    deriv(ac, vn + h * a3, ve + h * b3, vd + h * c3, qw + h * w3, qx + h * x3,
          qy + h * y3, qz + h * z3, p + h * p3, q + h * q3, r + h * r3)
  local s = h / 6.0
  ac.vn = vn + s * (a1 + 2 * a2 + 2 * a3 + a4)
  ac.ve = ve + s * (b1 + 2 * b2 + 2 * b3 + b4)
  ac.vd = vd + s * (c1 + 2 * c2 + 2 * c3 + c4)
  qw = qw + s * (w1 + 2 * w2 + 2 * w3 + w4)
  qx = qx + s * (x1 + 2 * x2 + 2 * x3 + x4)
  qy = qy + s * (y1 + 2 * y2 + 2 * y3 + y4)
  qz = qz + s * (z1 + 2 * z2 + 2 * z3 + z4)
  local qn = sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
  ac.qw, ac.qx, ac.qy, ac.qz = qw / qn, qx / qn, qy / qn, qz / qn
  ac.p = p + s * (p1 + 2 * p2 + 2 * p3 + p4)
  ac.q = q + s * (q1 + 2 * q2 + 2 * q3 + q4)
  ac.r = r + s * (r1 + 2 * r2 + 2 * r3 + r4)
  -- the accelerometer: specific force, body axes
  ac.fx, ac.fy, ac.fz = fx / MASS, fy / MASS, fz / MASS
end

--- One main-loop step of the rigid body alone, the surfaces where they are.
M.coast = rk4

function M.eas(ac)
  return sqrt(ac.vn * ac.vn + ac.ve * ac.ve + ac.vd * ac.vd) / EAS2TAS
end

--- One main-loop step: the rate loop on the commanded body rates (deg/s), the
--- mixer, the delay, the servos, then the rigid body.
local function step(ac, p_dps, q_dps)
  local sc = speed_scaler(M.eas(ac))
  local ail = pid_out(ac.roll_pid, p_dps, ac.p, sc)
  local elev = pid_out(ac.pitch_pid, q_dps, ac.q, sc)
  ac.ail_out, ac.elev_out = ail, elev
  -- the oldest command in the delay line reaches the servos; its slot takes
  -- this step's
  local d, jc = ac.delay, ac.jcmd
  local slot = table.remove(d)
  jc[1], jc[2], jc[3], jc[4] = slot[1], slot[2], slot[3], slot[4]
  table.insert(d, 1, mix(slot, ail, elev, 0.0))
  local a = DT / (SERVO_TAU_S + DT)
  local j = ac.j
  for i = 1, 4 do
    j[i] = j[i] + (clip(jc[i], -JOINT_MAX, JOINT_MAX) - j[i]) * a
  end
  rk4(ac)
end

--- Fly ms milliseconds of main loop on one FC command: throttle percent, body
--- roll and pitch rates deg/s.
function M.fly(ac, ms, thr_pct, p_dps, q_dps)
  ac.thr = thr_pct / 100.0
  for _ = 1, math.floor(ms * 0.001 / DT + 0.5) do step(ac, p_dps, q_dps) end
end

-- ---------------------------------------------------------------------------
-- STEADY FLIGHT.  The residuals of a steady pull at load n_load along a
-- straight path gamma above the horizon, wings level, nose alpha above the
-- path, at airspeed V, elevator elev (scaled) and throttle thr: the force along
-- the path, the force across it short of the load, and the pitching moment.
local function residual(V, gamma, alpha, elev, thr, n_load)
  local qw, qx, qy, qz = M.euler_to_q(0.0, gamma + alpha, 0.0)
  local r00, r01, r02, r10, r11, r12, r20, r21, r22 = rot(qw, qx, qy, qz)
  local vn, vd = V * cos(gamma), -V * sin(gamma)
  local u = r00 * vn + r20 * vd
  local v = r01 * vn + r21 * vd
  local w = r02 * vn + r22 * vd
  -- in a steady pull the body turns with the path, about its wing axis
  local q = (n_load - cos(gamma)) * G0 / V
  local j = mix({ 0.0, 0.0, 0.0, 0.0 }, 0.0, elev, 0.0)
  local fx, fy, fz, _, my = forces(u, v, w, 0.0, q, 0.0, j, thr, 0.0)
  local fn = r00 * fx + r01 * fy + r02 * fz
  local fd = r20 * fx + r21 * fy + r22 * fz
  local along = (fn * vn + fd * vd) / V - MASS * G0 * sin(gamma)
  local across = -fn * sin(gamma) - fd * cos(gamma) - n_load * MASS * G0
  return along, across, my
end

local function solve3(J, b)
  -- Gaussian elimination with partial pivoting, 3x3
  local a = { { J[1][1], J[1][2], J[1][3], b[1] }, { J[2][1], J[2][2], J[2][3], b[2] },
              { J[3][1], J[3][2], J[3][3], b[3] } }
  for c = 1, 3 do
    local piv = c
    for rr = c + 1, 3 do if abs(a[rr][c]) > abs(a[piv][c]) then piv = rr end end
    a[c], a[piv] = a[piv], a[c]
    for rr = c + 1, 3 do
      local f = a[rr][c] / a[c][c]
      for k = c, 4 do a[rr][k] = a[rr][k] - f * a[c][k] end
    end
  end
  local x = {}
  for rr = 3, 1, -1 do
    local s = a[rr][4]
    for k = rr + 1, 3 do s = s - a[rr][k] * x[k] end
    x[rr] = s / a[rr][rr]
  end
  return x
end

local TRIM_STEP = { 1e-4, 1e-6, 1e-2 }
local TRIM_ITER, TRIM_DONE = 60, 1e-3

--- Steady flight on a straight path gamma (rad) at load n_load (cos(gamma)
--- when nil): at throttle thr it solves (V, alpha, elev), at airspeed V
--- (throttle nil) it solves (thr, alpha, elev).  Returns the three, and the
--- largest residual.
function M.trim(gamma, thr, V, n_load)
  n_load = n_load or cos(gamma)
  local x, f
  if V == nil then
    x = { 17.0, 0.0, 1500.0 }
    f = function(z) return residual(z[1], gamma, z[2], z[3], thr, n_load) end
  else
    x = { M.THR_REF, 0.0, 1500.0 }
    f = function(z) return residual(V, gamma, z[2], z[3], z[1], n_load) end
  end
  for _ = 1, TRIM_ITER do
    local r1, r2, r3 = f(x)
    local J = { {}, {}, {} }
    for k = 1, 3 do
      local z = { x[1], x[2], x[3] }
      z[k] = z[k] + TRIM_STEP[k]
      local s1, s2, s3 = f(z)
      J[1][k] = (s1 - r1) / TRIM_STEP[k]
      J[2][k] = (s2 - r2) / TRIM_STEP[k]
      J[3][k] = (s3 - r3) / TRIM_STEP[k]
    end
    local dx = solve3(J, { -r1, -r2, -r3 })
    local big = 0.0
    for k = 1, 3 do
      x[k] = x[k] + dx[k]
      big = max(big, abs(dx[k] / TRIM_STEP[k]))
    end
    if big < TRIM_DONE then break end
  end
  local r1, r2, r3 = f(x)
  return x[1], x[2], x[3], max(abs(r1), abs(r2), abs(r3))
end

--- The pitch trim's four gains fitted to this airframe as the trim map fits
--- them (trim_map.fit_trim), against the lift term Lift Vector Pointing
--- feeds the trim: its lift load, cos(gamma) on a straight path.  Steady
--- climbs and dives from gamma_lo to gamma_hi deg, at each airspeed listed.
--- Returns offset, lift, attitude, thrust (elevator in ArduPlane's +-45 units)
--- and the fit's rms error.
function M.fit_trim_map(C, gamma_lo, gamma_hi, gamma_step, speeds)
  local rows, ys = {}, {}
  for gd = gamma_lo, gamma_hi, gamma_step do
    for _, V in ipairs(speeds) do
      local g = rad(gd)
      local thr, alpha, elev, res = M.trim(g, nil, V)
      if res < 1e-6 and thr > 0.0 and thr < 1.2 then
        local qr = (C.PITCH_TRIM_VREF_MPS / V) ^ 2
        local pitch = g + alpha
        rows[#rows + 1] = { 1.0, qr * cos(g), qr * sin(pitch), qr * thr / (EAS2TAS * EAS2TAS) }
        ys[#ys + 1] = elev * C.ELEV_FULL_TRAVEL / SCALED_FULL
      end
    end
  end
  -- normal equations, 4x4
  local A, b = {}, {}
  for i = 1, 4 do
    A[i], b[i] = {}, 0.0
    for k = 1, 4 do A[i][k] = 0.0 end
  end
  for n = 1, #rows do
    local rw = rows[n]
    for i = 1, 4 do
      b[i] = b[i] + rw[i] * ys[n]
      for k = 1, 4 do A[i][k] = A[i][k] + rw[i] * rw[k] end
    end
  end
  for c = 1, 4 do
    local piv = c
    for rr = c + 1, 4 do if abs(A[rr][c]) > abs(A[piv][c]) then piv = rr end end
    A[c], A[piv] = A[piv], A[c]
    b[c], b[piv] = b[piv], b[c]
    for rr = c + 1, 4 do
      local f = A[rr][c] / A[c][c]
      for k = c, 4 do A[rr][k] = A[rr][k] - f * A[c][k] end
      b[rr] = b[rr] - f * b[c]
    end
  end
  local x = {}
  for rr = 4, 1, -1 do
    local s = b[rr]
    for k = rr + 1, 4 do s = s - A[rr][k] * x[k] end
    x[rr] = s / A[rr][rr]
  end
  local se = 0.0
  for n = 1, #rows do
    local rw, e = rows[n], -ys[n]
    for i = 1, 4 do e = e + rw[i] * x[i] end
    se = se + e * e
  end
  return x[1], x[2], x[3], x[4], sqrt(se / #rows), #rows
end

-- ---------------------------------------------------------------------------
-- A STEADY LEVEL TURN, every axis trimmed.  Flying north at V (true) banked
-- bank about the velocity, the body set off it by the sideslip beta and the
-- angle of attack alpha, turning at om about world down: the attitude, the
-- body rates and the surfaces of the state x = {alpha, beta, elev, ail, om}.
local function turn_state(V, bank, x)
  local alpha, beta, om = x[1], x[2], x[5]
  local aw, ax, ay, az = cos(bank / 2), sin(bank / 2), 0.0, 0.0
  local bw, bx, by, bz = cos(-beta / 2), 0.0, 0.0, sin(-beta / 2)
  local cw, cx, cy, cz = cos(alpha / 2), 0.0, sin(alpha / 2), 0.0
  local qw, qx, qy, qz = q_mul(aw, ax, ay, az, bw, bx, by, bz)
  qw, qx, qy, qz = q_mul(qw, qx, qy, qz, cw, cx, cy, cz)
  local r00, r01, r02, r10, r11, r12, r20, r21, r22 = rot(qw, qx, qy, qz)
  return qw, qx, qy, qz, r20 * om, r21 * om, r22 * om,
         r00, r01, r02, r10, r11, r12, r20, r21, r22
end

-- Its residuals: no vertical acceleration, the turn's own acceleration
-- across the path, and no moment about any axis.
local function turn_residual(V, bank, thr, prop_torque, x)
  local _, _, _, _, p, q, r, r00, r01, r02, r10, r11, r12, r20, r21, r22 = turn_state(V, bank, x)
  local j = mix({ 0.0, 0.0, 0.0, 0.0 }, x[4], x[3], 0.0)
  local fx, fy, fz, mx, my, mz = forces(r00 * V, r01 * V, r02 * V, p, q, r, j, thr, prop_torque)
  local fe = r10 * fx + r11 * fy + r12 * fz
  local fd = r20 * fx + r21 * fy + r22 * fz
  return fd + MASS * G0, fe - MASS * x[5] * V,
         mx - (q * IZZ * r - r * IYY * q), my - (r * IXX * p - p * IZZ * r),
         mz - (p * IYY * q - q * IXX * p)
end

local TURN_STEP = { 1e-6, 1e-6, 1e-2, 1e-2, 1e-6 }

local function solve(A, b, n)
  for c = 1, n do
    local piv = c
    for rr = c + 1, n do if abs(A[rr][c]) > abs(A[piv][c]) then piv = rr end end
    A[c], A[piv] = A[piv], A[c]
    b[c], b[piv] = b[piv], b[c]
    for rr = c + 1, n do
      local f = A[rr][c] / A[c][c]
      for k = c, n do A[rr][k] = A[rr][k] - f * A[c][k] end
      b[rr] = b[rr] - f * b[c]
    end
  end
  local x = {}
  for rr = n, 1, -1 do
    local s = b[rr]
    for k = rr + 1, n do s = s - A[rr][k] * x[k] end
    x[rr] = s / A[rr][rr]
  end
  return x
end

--- Trim a steady level turn at true airspeed V, bank (rad) and throttle thr:
--- returns {alpha, beta, elev, ail, om} and the largest residual.  The along-
--- path force is left free: at a fixed throttle the speed is the chase's.
function M.trim_turn(V, bank, thr, prop_torque)
  local n_load = 1.0 / cos(bank)
  local _, alpha, elev = M.trim(0.0, nil, V, n_load)
  local x = { alpha, 0.0, elev, 0.0, G0 * math.tan(bank) / V }
  local res = math.huge
  for _ = 1, TRIM_ITER do
    local r0 = { turn_residual(V, bank, thr, prop_torque, x) }
    local J = {}
    for i = 1, 5 do J[i] = {} end
    for k = 1, 5 do
      local z = { x[1], x[2], x[3], x[4], x[5] }
      z[k] = z[k] + TURN_STEP[k]
      local r1 = { turn_residual(V, bank, thr, prop_torque, z) }
      for i = 1, 5 do J[i][k] = (r1[i] - r0[i]) / TURN_STEP[k] end
    end
    local dx = solve(J, { -r0[1], -r0[2], -r0[3], -r0[4], -r0[5] }, 5)
    local big = 0.0
    for k = 1, 5 do
      x[k] = x[k] + dx[k]
      big = max(big, abs(dx[k] / TURN_STEP[k]))
    end
    if big < TRIM_DONE then break end
  end
  local r = { turn_residual(V, bank, thr, prop_torque, x) }
  res = 0.0
  for i = 1, 5 do res = max(res, abs(r[i])) end
  return x, res
end

--- Set the aircraft in a steady level turn (M.trim_turn) flying north at true
--- airspeed V_tas, banked bank (rad) about the velocity, at throttle thr: its
--- surfaces at the turn's trim, already through the delay line.  Returns the
--- trim and its largest residual.
function M.start(ac, V_tas, thr, bank)
  local x, res = M.trim_turn(V_tas, bank, thr, ac.prop_torque)
  local qw, qx, qy, qz, p, q, r, r00, r01, r02 = turn_state(V_tas, bank, x)
  ac.qw, ac.qx, ac.qy, ac.qz = qw, qx, qy, qz
  ac.p, ac.q, ac.r = p, q, r
  ac.vn, ac.ve, ac.vd = V_tas, 0.0, 0.0
  ac.thr = thr
  mix(ac.j, x[4], x[3], 0.0)
  ac.ail_out, ac.elev_out = x[4], x[3]
  ac.delay = {}
  for i = 1, DELAY_STEPS do ac.delay[i] = { ac.j[1], ac.j[2], ac.j[3], ac.j[4] } end
  -- the specific force of the turn, for the first reading
  local fx, fy, fz = forces(r00 * V_tas, r01 * V_tas, r02 * V_tas, p, q, r,
                            ac.j, thr, ac.prop_torque)
  ac.fx, ac.fy, ac.fz = fx / MASS, fy / MASS, fz / MASS
  ac.roll_pid = new_pid(M.ACRO.RLL_RATE_P, M.ACRO.RLL_RATE_FF, M.ACRO.FLTT_HZ,
                        M.ACRO.FLTE_HZ, M.ACRO.FLTD_HZ, M.ACRO.SMAX)
  ac.pitch_pid = new_pid(M.ACRO.PTCH_RATE_P, M.ACRO.PTCH_RATE_FF, M.ACRO.FLTT_HZ,
                         M.ACRO.FLTE_HZ, M.ACRO.FLTD_HZ, M.ACRO.SMAX)
  return x, res
end

--- Hand the aircraft's state to the AHRS stubs: attitude (the quaternion and
--- its Euler angles), gyro, velocity, airspeed and specific force; and to the
--- servo-output stubs (scaled), ArduPlane's aileron, elevator, rudder and
--- throttle outputs.
function M.to_stubs(ac, sim, scaled)
  scaled[4], scaled[19], scaled[21], scaled[70] = ac.ail_out, ac.elev_out, 0.0, ac.thr * 100.0
  sim.quat = sim.quat or {}
  sim.quat[1], sim.quat[2], sim.quat[3], sim.quat[4] = ac.qw, ac.qx, ac.qy, ac.qz
  sim.roll, sim.pitch, sim.yaw = M.q_to_euler(ac.qw, ac.qx, ac.qy, ac.qz)
  sim.p, sim.q, sim.r = ac.p, ac.q, ac.r
  sim.vn, sim.ve, sim.vd = ac.vn, ac.ve, ac.vd
  sim.eas, sim.eas_valid, sim.eas2tas = M.eas(ac), true, EAS2TAS
  sim.ax, sim.ay, sim.az, sim.accel_valid = ac.fx, ac.fy, ac.fz, true
end

--- The path frame of the aircraft, as track.lua builds it, in NED: the path
--- (x), the right wing square to it (y), the belly (z).
function M.path_frame(ac)
  local r00, r01, r02, r10, r11, r12, r20, r21, r22 = rot(ac.qw, ac.qx, ac.qy, ac.qz)
  local V = sqrt(ac.vn * ac.vn + ac.ve * ac.ve + ac.vd * ac.vd)
  local xn, xe, xd = ac.vn / V, ac.ve / V, ac.vd / V
  -- y = z_body x x, unit
  local zbn, zbe, zbd = r02, r12, r22
  local yn, ye, yd = unit3(zbe * xd - zbd * xe, zbd * xn - zbn * xd, zbn * xe - zbe * xn)
  local zn, ze, zd = xe * yd - xd * ye, xd * yn - xn * yd, xn * ye - xe * yn
  return xn, xe, xd, yn, ye, yd, zn, ze, zd, V
end

return M
