-- Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
--
-- path_attitude.lua - Lift Vector Pointing, version B (rudder held at zero).
--
-- Named for what it does: it points the wing's lift vector at the push the
-- flight path's turn needs.
--
-- One law steers the FLIGHT PATH's own frame, the one track.lua builds every
-- tick: x' along the velocity, y' the right wing kept square to the
-- belly-canopy axis, z' the belly.  Every quantity it closes is a component in
-- that frame, so a balloon overhead, underneath or behind is no special case:
-- no Euler angle enters the law, and a bank appears only where the bank limit
-- and a PLAN read it.
--
-- Each tick answers three questions in order:
--
--   1. GUIDANCE.  Where should the path go, and how fast should it turn?
--      theta is the path's angle off the balloon, read from where the path
--      will point att_lead_s from now (the accelerometer sees the load before
--      the velocity has turned with it); u is the direction across the path
--      toward the balloon.  The path is asked to turn toward u at
--          Omega = min(att_k_path * theta, att_path_rate_max_dps).
--
--   2. THE FORCE LINE.  What push must the wing make to turn it that way?
--          f = V * Omega * u - g_perp           (per unit mass, across the path)
--      The turn the path needs, less what gravity across the path already
--      does.  Its direction is the lift line to aim for, its size the load.
--
--   3. THE ATTITUDE.  How must the aircraft rotate to make that push?  The
--      wing pushes only along its lift line, either way: out of the canopy
--      (pull) or out of the belly (push).  The side is the one of the wing
--      line f lies on (THE LIFT SIDE), and the roll axis rolls about the path
--      until that end of the lift line faces f; the pitch axis turns the path
--      at Omega's share in the lift plane, which the wing makes by pulling or
--      pushing; and the path frame turns about z' with gravity's sideways
--      part.
--
-- The three path-frame rates are rotated into body axes and sent as body roll
-- and pitch rates.  Version B sends no yaw: set_rudder_offset(0, false) keeps
-- the rudder at zero, the fin keeps the aircraft straight, and a roll about
-- the nose swings the path round it, which the law reads as path error and
-- steers out like any other.
--
-- Why one formula covers shallow and steep paths: on a shallow path gravity is
-- most of the push, so the lift line stays near canopy-up and a modest bank
-- turns the aircraft; near vertical gravity does almost nothing across the
-- path, so the lift line rolls round to face the balloon.
--
-- THE BANK LIMIT holds in every chase, on the bank as the cascade measures
-- it: the wing tip's angle below the horizon, asin(sin(roll) * cos(pitch)),
-- the bank proxy (world down along the body's wing axis, track.lua's
-- down_by).  Level it is the bank; the steeper the path, the more roll it
-- takes to dip a wing tip that far, and with the path no further than the
-- limit from vertical no roll about it does, so there the roll is free.
-- Upside down with the wings level reads as no bank.  On the chase's first
-- tick the band is [min(-limit, b0), max(limit, b0)], b0 the proxy it starts
-- at, so a start banked beyond the limit may roll back toward it but never
-- further out.
-- UPSIDE DOWN, as the rules of a chase define it (CLAUDE.md), holds in every
-- chase too: the law never rolls the aircraft upside down, and one that is
-- may only roll back.  Within CURVE_POLE_FADE_DEG of vertical the rule sets
-- no limit: there the axis is near level and the bank has no meaning.
-- The roll's aim is held to both: the roll goes no further than the first
-- bank where the wing tip would pass the band or the aircraft would be upside
-- down, so a turn that wants more bank aims at that bank itself, and where
-- that stops a pull from facing f the turn is slowed (THE TURN THE ROOM
-- ALLOWS).  One more term works with the limit: the turn's sideways part
-- weighs att_k_side times its share, so the heading turns while the path is
-- still shallow and a bank still turns it.  It does not act within
-- CURVE_POLE_FADE_DEG of vertical, where the horizontal across the path has
-- no direction.  The law steers by these two rules of a chase; the others
-- judge the flown curve afterwards, and the law does not sort chases by them.
--
-- PLAN MODE.  While the Curve Flyer runs a PLAN (curve_flyer.lua) it hands
-- the law the balloon to turn the path toward and the bank to hold, measured
-- round the path as the law measures it.  The guidance and the pitch are the
-- law's own; the roll goes to the PLAN's bank instead of facing the force
-- line, held to the band and upside down as any aim point is, so no lift side
-- is chosen and the turn is not slowed; the turn's sideways part is not
-- weighed, since that weighting serves the law's own choice of bank; the
-- roll cancels the bank's drift on every tick, and
-- fades to nothing within CURVE_POLE_FADE_DEG of vertical, where the bank
-- cannot be measured.  The Curve Planner flies a model of this mode
-- (curve_planner.py), so a change here is a change there in the same commit.
--
-- The law allocates nothing per tick: its state and diagnostics live in the
-- table M.new() builds at load, filled in place.

local C = require("constants")

local M = {}

local NAN = 0 / 0
local abs, sqrt, atan = math.abs, math.sqrt, math.atan
local sin, cos, asin, acos = math.sin, math.cos, math.asin, math.acos
local rad, deg = math.rad, math.deg
local min, max = math.min, math.max

-- Below this the length of a direction across the path is taken as none: the
-- balloon on the path, or the path along the vertical.
local DIR_MIN = 1e-9
-- Within CURVE_POLE_FADE_DEG of vertical the horizontal across the path and
-- the bank against the horizon have no direction: world up's part across the
-- path is shorter than POLE_SIN, and the path's angle to the horizon has a
-- tangent past TAN_CAP.
local POLE_SIN = sin(rad(C.CURVE_POLE_FADE_DEG))
local TAN_CAP = math.tan(rad(90.0 - C.CURVE_POLE_FADE_DEG))

local function clip(v, lo, hi)
  if v < lo then return lo elseif v > hi then return hi end
  return v
end

local function wrap180(a)
  return (a + 180.0) % 360.0 - 180.0
end

-- ---------------------------------------------------------------------------
--- The law's state: what it carries between ticks, and what it logs.
function M.new()
  local st = {
    band_lo = NAN, band_hi = NAN,   -- deg of bank proxy: the chase's band, NaN until its first tick
    side = 0,                -- this tick's lift side: 1 canopy (pull), -1 belly (push), 0 none
    roll_i = 0.0,            -- rad/s: roll integrator
    pitch_i = 0.0,           -- rad/s: pitch integrator
    n_lift = NAN,            -- g: the load the law asks for, for the pitch trim
    -- one tick's working values, passed between its three stages
    w_V = 0.0, w_um = 0.0, w_up_x = 0.0, w_upy = 0.0, w_upz = 0.0, w_bank = 0.0,
    w_pz = 0.0, w_zz = 0.0,
    w_gy = 0.0, w_gz = 0.0, w_theta = 0.0, w_ty = 0.0, w_tz = 0.0,
    w_horiz = false, w_uy = 0.0, w_uz = 0.0, w_om = 0.0, w_om_ask = 0.0,
    w_fy = 0.0, w_fz = 0.0, w_slowed = false,
    w_mu = 0.0, w_clamped = false, w_wf = 0.0, w_plan = false,
    w_proxy = 0.0, w_tip_c = 0.0, w_tip_k = 0.0, w_tip_d = 0.0,
    w_room_lo = 0.0, w_room_hi = 0.0,
  }
  M.clear_log(st)
  return st
end

--- Every diagnostic blank: a tick the law did not run.
function M.clear_log(st)
  st.ran = false
  st.log_theta_deg, st.log_theta_now_deg = NAN, NAN
  st.log_u_y, st.log_u_z, st.log_omega_dps = NAN, NAN, NAN
  st.log_omega_ask_dps = NAN
  st.log_f_y, st.log_f_z = NAN, NAN
  st.log_mu_deg, st.log_side, st.log_w_f = NAN, NAN, NAN
  st.log_wx_dps, st.log_wy_dps, st.log_wz_dps = NAN, NAN, NAN
  st.log_bank_deg, st.log_drift_dps = NAN, NAN
  st.log_p_dps, st.log_q_dps = NAN, NAN
  st.log_roll_i_dps, st.log_pitch_i_dps = NAN, NAN
  st.log_band_lo_deg, st.log_band_hi_deg = NAN, NAN
  st.log_proxy_deg, st.log_room_lo_deg, st.log_room_hi_deg = NAN, NAN, NAN
  st.log_sideslip_deg, st.log_aoa_deg = NAN, NAN
end

--- A fresh chase: nothing carries over from the last one.  Its band was
--- decided from another start.
function M.reset(st)
  st.band_lo, st.band_hi = NAN, NAN
  M.clear_integrators(st)
end

--- A tick whose output does not reach the aircraft: both integrators are
--- inactive and are cleared.
function M.clear_integrators(st)
  st.roll_i, st.pitch_i = 0.0, 0.0
  st.log_roll_i_dps, st.log_pitch_i_dps = 0.0, 0.0
end

-- ---------------------------------------------------------------------------
--- The chase's band, from its first tick, in bank proxy: the bank limit
--- either way, widened to the starting proxy where that is beyond it.
local function decide_band(st, proxy_now, lim)
  st.band_lo, st.band_hi = min(-lim, proxy_now), max(lim, proxy_now)
end

--- Banked inside a forbidden arc (half-width h, the bank r from its middle):
--- the roll may only leave it, by its nearer end, and go on no further than
--- the room past that end.
local function leave(st, h, r, gap)
  if r >= 0.0 then
    local e = h - r
    st.w_room_lo, st.w_room_hi = e, min(180.0, e + gap)
  else
    local e = h + r
    st.w_room_lo, st.w_room_hi = max(-180.0, -e - gap), -e
  end
end

--- How far the roll may go from the bank now, either way, before the wing
--- tip passes the band: st.w_room_lo (deg, the left) and st.w_room_hi (the
--- right), +-180 where nothing limits it.  After a roll that takes the bank
--- round the path to t, the proxy's sine is c + k * sin(t + d) (guidance), so
--- the band forbids at most two arcs of t: one round the right wing straight
--- down (t + d = 90), one round the left (-90).  No roll may reach into one,
--- so a turn that wants more bank stops at the limit and cannot roll on past
--- the wing tip's lowest point to the far side.  With the path no further
--- than the limit from vertical the arcs vanish.
local function band_room(st)
  st.w_room_lo, st.w_room_hi = -180.0, 180.0
  local k = st.w_tip_k
  if k <= DIR_MIN then return end
  local x = st.w_bank + st.w_tip_d
  local s_hi = (sin(rad(st.band_hi)) - st.w_tip_c) / k
  local s_lo = (sin(rad(st.band_lo)) - st.w_tip_c) / k
  local h_hi = (abs(s_hi) < 1.0) and 90.0 - deg(asin(s_hi)) or 0.0
  local h_lo = (abs(s_lo) < 1.0) and 90.0 + deg(asin(s_lo)) or 0.0
  if h_hi + h_lo <= 0.0 then return end
  -- the room between the arcs: the same on both sides when there are two
  local gap = (h_hi > 0.0 and h_lo > 0.0) and 180.0 - h_hi - h_lo
              or 360.0 - 2.0 * (h_hi + h_lo)
  local r_hi, r_lo = wrap180(x - 90.0), wrap180(x + 90.0)
  if abs(r_hi) < h_hi then
    leave(st, h_hi, r_hi, gap)
  elseif abs(r_lo) < h_lo then
    leave(st, h_lo, r_lo, gap)
  else
    -- up to the nearest arc's near end, each way
    if h_hi > 0.0 then
      st.w_room_hi = min(st.w_room_hi, (-r_hi - h_hi) % 360.0)
      st.w_room_lo = max(st.w_room_lo, -((r_hi - h_hi) % 360.0))
    end
    if h_lo > 0.0 then
      st.w_room_hi = min(st.w_room_hi, (-r_lo - h_lo) % 360.0)
      st.w_room_lo = max(st.w_room_lo, -((r_lo - h_lo) % 360.0))
    end
  end
end

--- The room cut further to upside down, as the rules of a chase define it.
--- The canopy axis, -z of the body, is -pz x' - zz z' in the path frame (y'
--- is square to it), so after a roll that takes the bank round the path to t,
--- world down along it is pz * up_x - zz * um * cos(t): only its part across
--- the path turns with the roll.  Where that is positive the axis meets the
--- ground on the canopy side, so the rule leaves the banks with cos(t) >= q,
--- q = pz * up_x / (zz * um): within h = acos(q) of up either way, every bank
--- for q <= -1, and up alone for q >= 1.  A bank already past h may only roll
--- back toward up.  h is 90 with the nose on the path; pushing in a dive the
--- nose sits below it and h is less: with the wings square to up across the
--- path, the canopy axis tips toward the ground.  Within CURVE_POLE_FADE_DEG
--- of vertical the rule cuts nothing.
local function upside_room(st)
  local um, zz = st.w_um, st.w_zz
  if um <= POLE_SIN or zz <= DIR_MIN then return end
  local q = st.w_pz * st.w_up_x / (zz * um)
  if q <= -1.0 then return end
  local h = (q < 1.0) and deg(acos(q)) or 0.0
  local b = st.w_bank
  if b > h then
    st.w_room_lo, st.w_room_hi = max(st.w_room_lo, -h - b), min(st.w_room_hi, 0.0)
  elseif b < -h then
    st.w_room_lo, st.w_room_hi = max(st.w_room_lo, 0.0), min(st.w_room_hi, h - b)
  else
    st.w_room_lo, st.w_room_hi = max(st.w_room_lo, -h - b), min(st.w_room_hi, h - b)
  end
end

--- How far the roll may go from the bank now, either way: the band's room,
--- cut to upside down.
local function bank_room(st)
  band_room(st)
  upside_room(st)
end

--- One side's roll still to go, held to the room: mu cut to it either way.
--- Returns the held mu and whether the room moved it.
local function held(mu, st)
  local d = deg(mu)
  local h = clip(d, st.w_room_lo, st.w_room_hi)
  if h ~= d then return rad(h), true end
  return mu, false
end

-- ---------------------------------------------------------------------------
-- One tick, in three stages.  Each passes what the next reads through `st`
-- (the w_* fields): ArduPilot's Lua allows 100 locals in a function, and one
-- function holding the whole tick would pass it.

--- 1. The frame and the guidance.  Gravity and world up in the path frame, the
--- bank round the path and the bank proxy, the path att_lead_s from now and
--- the balloon's angle off it (theta) and direction across it (ty, tz); on the
--- chase's first tick, its bank band.
local function guidance(st, s, cfg, g, bx, by, bz)
  local G0 = C.GRAVITY_MPS2
  local px, py, pz = s.path_bx, s.path_by, s.path_bz
  local yx, yy = s.path_yx, s.path_yy
  local zx, zy, zz = s.path_zx, s.path_zy, s.path_zz
  local V = max(s.path_speed or 0.0, C.FLIGHT_PATH_MIN_SPEED_MPS)
  local dnx, dny, dnz = s.down_bx, s.down_by, s.down_bz
  local up_x = -(dnx * px + dny * py + dnz * pz)
  local upy, upz = s.up_path_y, s.up_path_z
  local um = sqrt(upy * upy + upz * upz)
  local bank_now = deg(atan(-upy, -upz))
  st.w_V, st.w_um, st.w_up_x, st.w_upy, st.w_upz = V, um, up_x, upy, upz
  -- The bank proxy, and what a roll about the path does to it.  The wing axis
  -- turns with the path frame, its parts in it fixed: py along the path,
  -- (yy, zy) across it.  Rolling the bank round the path to t leaves world
  -- down along the wing axis at c + k * sin(t + d): c = py * (down along the
  -- path), which no such roll changes, and k = um * |(yy, zy)|,
  -- d = atan2(zy, yy) from the part across.
  st.w_proxy = deg(asin(clip(dny, -1.0, 1.0)))
  st.w_tip_c = -py * up_x
  st.w_tip_k = um * sqrt(yy * yy + zy * zy)
  st.w_tip_d = deg(atan(zy, yy))
  st.w_bank = bank_now
  -- the canopy axis in the path frame, for upside down (upside_room)
  st.w_pz, st.w_zz = pz, zz
  st.w_gy = G0 * (dnx * yx + dny * yy)
  st.w_gz = G0 * (dnx * zx + dny * zy + dnz * zz)
  -- The path att_lead_s from now.  The path's acceleration is the specific
  -- force plus gravity; its part across the path, over V, is the rate the path
  -- is turning at, which the accelerometer sees as soon as the wing loads.
  -- With no accelerometer reading (NaN) the path is read as it is.
  local xpx, xpy, xpz = px, py, pz
  local fx, fy, fz = cfg.acc_x, cfg.acc_y, cfg.acc_z
  if fx and fy and fz and fx == fx and fy == fy and fz == fz then
    local ax, ay, az = fx + G0 * dnx, fy + G0 * dny, fz + G0 * dnz
    local along = ax * px + ay * py + az * pz
    local lead = (g.att_lead_s or 0.0) / V
    xpx = px + lead * (ax - along * px)
    xpy = py + lead * (ay - along * py)
    xpz = pz + lead * (az - along * pz)
    local xn = sqrt(xpx * xpx + xpy * xpy + xpz * xpz)
    xpx, xpy, xpz = xpx / xn, xpy / xn, xpz / xn
  end
  local cth = clip(bx * xpx + by * xpy + bz * xpz, -1.0, 1.0)
  local tbx, tby, tbz = bx - cth * xpx, by - cth * xpy, bz - cth * xpz
  st.w_theta = atan(sqrt(tbx * tbx + tby * tby + tbz * tbz), cth)
  -- the balloon's direction across the predicted path, read in the path frame
  st.w_ty = tbx * yx + tby * yy
  st.w_tz = tbx * zx + tby * zy + tbz * zz
  -- the angle off the path as read, for the log
  local c0 = clip(bx * px + by * py + bz * pz, -1.0, 1.0)
  st.log_theta_now_deg = deg(acos(c0))
  if st.band_lo ~= st.band_lo then
    decide_band(st, st.w_proxy, g.bank_lim_deg or 0.0)
  end
  if s.path_from_vel then
    st.log_sideslip_deg = deg(math.asin(clip(py, -1.0, 1.0)))
    st.log_aoa_deg = deg(atan(pz, px))
  else
    st.log_sideslip_deg, st.log_aoa_deg = NAN, NAN
  end
end

--- 2a. The turn the path is asked for, and the force line it needs.  The
--- turn direction's sideways part (along h, the horizontal across the path)
--- weighs att_k_side when `weigh`, so it finishes first.  A balloon dead
--- astern has no direction across the path, and the turn goes toward the
--- canopy: up and over.
local function turn(st, g, weigh)
  local V, um, upy, upz = st.w_V, st.w_um, st.w_upy, st.w_upz
  -- The horizontal across the path has a direction only this far from
  -- vertical.
  st.w_horiz = um > POLE_SIN
  local dy, dz = st.w_ty, st.w_tz
  if weigh and st.w_horiz then
    local hy, hz = -upz / um, upy / um
    local side_part = ((g.att_k_side or 1.0) - 1.0) * (dy * hy + dz * hz)
    dy, dz = dy + side_part * hy, dz + side_part * hz
  end
  local dn = sqrt(dy * dy + dz * dz)
  local uy, uz = 0.0, -1.0
  if dn > DIR_MIN then uy, uz = dy / dn, dz / dn end
  local om = min((g.att_k_path or 0.0) * st.w_theta, rad(g.att_path_rate_max_dps or 0.0))
  st.w_uy, st.w_uz, st.w_om, st.w_om_ask = uy, uz, om, om
  -- THE FORCE LINE
  st.w_fy = V * om * uy - st.w_gy
  st.w_fz = V * om * uz - st.w_gz
end

--- THE TURN THE ROOM ALLOWS.  Where the room stops a pull from facing f, the
--- path is asked to turn toward the same u at the fastest rate a pull inside
--- the room makes, and f becomes that turn's.  Turning at a (m/s^2 across
--- the path) asks f = a * u - g_perp, which is a * u_h along h, the horizontal
--- across the path, and G + a * u_v along up across it (G gravity across the
--- path).  A pull makes it with the canopy leaning toward u's side by no more
--- than B, the farthest bank the room reaches there, while
--- G + a * u_v >= cot(B) * a * |u_h|: up to
---     a = G / (cot(B) * |u_h| - u_v),
--- and at every rate where the denominator is not positive.  The wing then
--- makes all of f, along the turn.  Pulling at the room's end with f as asked
--- would leave the rest of f undone across the turn, and that bends the path
--- off it so that f moves further out of the room.  A push is not slowed: no
--- push turns the path toward u slower than gravity alone does, and a push
--- the room stops from facing f is a pull instead (THE LIFT SIDE).  Within
--- CURVE_POLE_FADE_DEG of vertical up across the path has no direction, and
--- the turn is not slowed.
local function slow_to_room(st)
  st.w_slowed = false
  local um = st.w_um
  if st.side < 0 or um <= POLE_SIN then return end
  local vy, vz = st.w_upy / um, st.w_upz / um
  local uy, uz = st.w_uy, st.w_uz
  local uh, uv = uz * vy - uy * vz, uy * vy + uz * vz
  local b = (uh >= 0.0) and st.w_bank + st.w_room_hi or -(st.w_bank + st.w_room_lo)
  if b >= 180.0 then return end
  local a = 0.0
  if b > 0.0 then
    local den = cos(rad(b)) / sin(rad(b)) * abs(uh) - uv
    if den <= 0.0 then return end
    a = C.GRAVITY_MPS2 * um / den
  end
  local V = st.w_V
  if a >= V * st.w_om then return end
  st.w_om, st.w_slowed = a / V, true
  st.w_fy, st.w_fz = a * uy - st.w_gy, a * uz - st.w_gz
end

--- 2b. THE LIFT SIDE, and the roll still to go for its end of the lift line
--- to face the force line.  The wing pushes along its lift line either way,
--- so the side is the one of the wing line f lies on: the canopy's pulls, the
--- belly's pushes.  A push the room stops from facing f is a pull instead:
--- the room never stops a pull, which it slows until it can face f (THE TURN
--- THE ROOM ALLOWS), while a cut push turns the path off the turn.  The side
--- changes only where f crosses the wing line, or where the belly's facing
--- roll enters or leaves the room as f moves, and the law's own roll brings
--- neither about: rolling the lift line toward f turns the wing line away
--- from it, and which banks the room allows is set by the path, not by the
--- roll.  Choosing instead the side whose push fits f better once the room
--- cuts each side's roll makes a tie the law steers itself onto: between the
--- room's ends lies a range of f no roll faces, and there each cut push bends
--- the path so that f moves toward the middle of that range, where the two
--- sides fit equally, and the side switches on every tick it can.
local function force_and_side(st)
  local fy, fz = st.w_fy, st.w_fz
  bank_room(st)
  local sd = (fz <= 0.0) and 1 or -1
  if sd < 0 then
    local m = deg(atan(-fy, fz))
    if m < st.w_room_lo or m > st.w_room_hi then sd = 1 end
  end
  st.side = sd
  slow_to_room(st)
  st.w_mu, st.w_clamped = held(atan(sd * st.w_fy, -sd * st.w_fz), st)
  -- a slowed turn's f lies at the room's end, where the bank is held
  st.w_clamped = st.w_clamped or st.w_slowed
  -- the roll has a weight only under a PLAN
  st.w_wf = NAN
end

--- 2b, under a PLAN.  The roll still to go to the PLAN's bank (deg, round the
--- path), held to the room; no lift side; the roll's weight the fade near
--- vertical.
local function planned(st, plan_bank)
  bank_room(st)
  st.w_mu, st.w_clamped = held(rad(wrap180(plan_bank - st.w_bank)), st)
  st.side = 0
  st.w_wf = clip(st.w_um / POLE_SIN, 0.0, 1.0)
end

--- 3. The path-frame rates, into body axes and limited.  Returns the body
--- roll and pitch rates to send, deg/s.
local function rates(st, s, cfg, g, dt)
  local V, um, up_x, upy, upz = st.w_V, st.w_um, st.w_up_x, st.w_upy, st.w_upz
  local theta, mu, om, uz = st.w_theta, st.w_mu, st.w_om, st.w_uz
  local acro_roll = cfg.acro_roll_rate_dps or 0.0
  local acro_pitch = cfg.acro_pitch_rate_dps or 0.0
  -- Each integrator runs only inside att_i_zone_deg of the balloon, where
  -- what is left is trim and the propeller's torque; wound through the turn
  -- it overshoots.  Outside the zone, and at a gain of 0, it is cleared.
  -- Each is held within its ACRO rate: past it nothing it stores is flown.
  -- Under a PLAN the roll's weight fades it near vertical, and the
  -- integrator's input with it.
  local plan = st.w_plan
  local zone = theta < rad(g.att_i_zone_deg or 0.0)
  local roll_ki, pitch_ki = g.att_roll_ki or 0.0, g.att_pitch_ki or 0.0
  if zone and roll_ki ~= 0.0 then
    local rmax = rad(acro_roll)
    local mu_i = plan and mu * st.w_wf or mu
    st.roll_i = clip(st.roll_i + roll_ki * mu_i * dt, -rmax, rmax)
  else
    st.roll_i = 0.0
  end
  if zone and pitch_ki ~= 0.0 then
    local qmax = rad(acro_pitch)
    st.pitch_i = clip(st.pitch_i + pitch_ki * theta * (-uz) * dt, -qmax, qmax)
  else
    st.pitch_i = 0.0
  end
  local wx = (g.att_k_roll or 0.0) * mu + st.roll_i
  local wy = om * (-uz) + st.pitch_i
  local wz = st.w_gy / V
  -- The bank round the path moves as the path turns, with no roll about it
  -- at all:
  --     d(bank)/dt = wx - up_x * (up_y * wy + up_z * wz) / um^2
  -- (world up in the path frame, up_x along the path): up_x / um is the
  -- tangent of the path's angle to the horizon, capped at TAN_CAP, where the
  -- bank fades out near vertical.  While the room holds the bank, and on
  -- every tick a PLAN sets the bank, the roll cancels that drift, a larger
  -- share of the roll the steeper the path.
  st.log_drift_dps = NAN
  if (st.w_clamped or plan) and um > DIR_MIN then
    local tg = clip(up_x / um, -TAN_CAP, TAN_CAP)
    local drift = -tg * (upy * wy + upz * wz) / um
    st.log_drift_dps = deg(drift)
    wx = wx - drift
  end
  if plan then wx = wx * st.w_wf end
  st.log_wx_dps, st.log_wy_dps, st.log_wz_dps = deg(wx), deg(wy), deg(wz)

  -- Into body axes, then cut at the ACRO rates as ArduPlane cuts them.
  local p_dps = deg(wx * s.path_bx + wy * s.path_yx + wz * s.path_zx)
  local q_dps = deg(wx * s.path_by + wy * s.path_yy + wz * s.path_zy)
  st.log_p_dps, st.log_q_dps = p_dps, q_dps
  p_dps = clip(p_dps, -acro_roll, acro_roll)
  q_dps = clip(q_dps, -acro_pitch, acro_pitch)
  return p_dps, q_dps
end

--- The load the body rates (p_dps, q_dps) ask of the wing, g: the path's turn
--- in the lift plane they make, plus gravity's part across the path.  The
--- pitch trim reads it on a tick the Curve Flyer flies under this law.
function M.load_of(s, p_dps, q_dps)
  local G0 = C.GRAVITY_MPS2
  local V = max(s.path_speed or 0.0, C.FLIGHT_PATH_MIN_SPEED_MPS)
  local wy = rad(p_dps * s.path_yx + q_dps * s.path_yy)
  local gz = G0 * (s.down_bx * s.path_zx + s.down_by * s.path_zy + s.down_bz * s.path_zz)
  return (V * wy + gz) / G0
end

--- One tick.
---
--- `s` is the track state track.lua filled this tick: the path frame in body
--- axes (path_bx..path_zz), world down in body axes (down_bx..down_bz), world
--- up across the path (up_path_y, up_path_z), the speed (path_speed) and
--- whether the velocity gave the path (path_from_vel).  (bx, by, bz) is the
--- balloon in body axes: the balloon, or under a PLAN the Curve Flyer's.
--- `cfg` carries this tick's specific force (acc_x..z, m/s^2, body) and the
--- ACRO rate limits; `g` the gains; `plan_bank`, deg round the path, the
--- PLAN's bank on a tick the Curve Flyer runs one (PLAN MODE), else nil.
--- Returns the body roll and pitch rates to send, deg/s, after ArduPlane's
--- ACRO limits; st.n_lift is the load the force line asks of the wing, which
--- the pitch trim's lift term reads.
function M.step(st, s, cfg, g, bx, by, bz, dt, plan_bank)
  st.ran = true
  guidance(st, s, cfg, g, bx, by, bz)
  st.w_plan = plan_bank ~= nil
  turn(st, g, not st.w_plan)
  if st.w_plan then
    planned(st, plan_bank)
  else
    force_and_side(st)
  end
  local p_dps, q_dps = rates(st, s, cfg, g, dt)
  st.n_lift = -st.w_fz / C.GRAVITY_MPS2
  st.log_theta_deg = deg(st.w_theta)
  st.log_u_y, st.log_u_z, st.log_omega_dps = st.w_uy, st.w_uz, deg(st.w_om)
  st.log_omega_ask_dps = deg(st.w_om_ask)
  st.log_f_y, st.log_f_z = st.w_fy, st.w_fz
  st.log_mu_deg, st.log_w_f = deg(st.w_mu), st.w_wf
  -- under a PLAN no lift side is chosen
  st.log_side = st.w_plan and NAN or st.side
  st.log_bank_deg = st.w_bank
  st.log_roll_i_dps, st.log_pitch_i_dps = deg(st.roll_i), deg(st.pitch_i)
  st.log_band_lo_deg, st.log_band_hi_deg = st.band_lo, st.band_hi
  st.log_proxy_deg = st.w_proxy
  st.log_room_lo_deg, st.log_room_hi_deg = st.w_room_lo, st.w_room_hi
  return p_dps, q_dps
end

return M
