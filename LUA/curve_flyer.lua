-- Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
--
-- curve_flyer.lua - the Curve Flyer: runs the Curve Planner's turn.
--
-- The Curve Planner (Companion/curve_planner.py) plans the TURN of a chase
-- under the rules of a chase (CLAUDE.md) and sends it as a PLAN: bank
-- angles, each tied to the turn's PROGRESS, for the steering law selected
-- (steering_law), in that law's own measure of the bank - the bank proxy
-- under the cascade, the bank round the path under Lift Vector Pointing.
-- This module runs it every tick and hands that law two things to fly:
--
--   the aim point  the direction to turn the flight path toward - the balloon,
--                  or a line (below) - in the path frame;
--   the bank       the schedule's bank at the progress s = 1 - eps / eps0,
--                  clipped to 0..1, with eps the flight path's angle off the
--                  balloon now and eps0 the PLAN's, linear between knots.
--
-- The law flies them with its own roll, pitch, rudder and bank limit - its
-- PLAN MODE (cascade.lua, path_attitude.lua) - and this module commands no
-- rate of its own.  Progress, not time, indexes the schedule, so an aircraft
-- that turns slower than planned still meets each bank where the Curve
-- Planner placed it.
--
-- The turn is complete when eps is within CURVE_TURN_DONE_DEG.  With
-- Transfer Control When Done on (cf_transfer_when_done) the law flies the
-- chase from there as it flies one without a PLAN; off, the law is aimed at
-- the line the flight path had at that moment, banked CF_HOLD_BANK_PER_DEG
-- per degree the line lies across the wings (level on the line), until the
-- chase ends or the switch goes on.  The turn, a run-in and its dive each
-- run until their own condition ends them: no clock hands the chase back.
--
-- A PLAN WITH A RUN-IN (the Approach Angle, CF_APPROACH_DEG) aims its
-- scheduled turn at the RUN-IN LINE instead of the balloon: runin_climb_deg
-- above the horizon, toward the balloon's compass direction, worked out
-- afresh every tick (M.line), so it follows the balloon as the aircraft
-- moves.  With the path within CURVE_TURN_DONE_DEG of it the RUN-IN holds it,
-- as a hold holds its line, until the balloon sits runin_dep_deg below the
-- horizon; the DIVE then aims the law at the balloon, banked toward it
-- CF_HOLD_BANK_PER_DEG per degree it lies across the wings.  The dive ends
-- the turn: within CURVE_TURN_DONE_DEG of the balloon the law flies as
-- without a PLAN, or holds.  The run-in ends on the balloon's angle below the
-- horizon, not on time, so a slower aircraft or a nearer balloon than planned
-- still dives where the line to the balloon is as steep as planned.
--
-- THE CURVE PLANNER FLIES EVERY CANDIDATE THROUGH A MODEL OF EACH LAW'S PLAN
-- MODE AND OF THIS MODULE (fly() in curve_planner.py), from the same
-- constants and gains.  A change to either is a change there in the same
-- commit, or the plans it receives were made for a different aircraft.
--
-- Every quantity is read in the path frame track.lua builds once per tick
-- (CLAUDE.md "Every balloon error but roll-to-plane's clock is read in the
-- path frame"): the balloon
-- (t'_x, t'_y, t'_z) and the line (hold_path_x/y/z).
--
-- STATES (STATE2 cf_state):
--   OFF      the Curve Flyer is off, or the chase is not one it flies
--   WAIT     a fresh chase, holding its rates at zero for the PLAN
--   FLY      the law flying the PLAN's turn
--   DONE     the turn is complete; the law flies as without a PLAN
--   TIMEOUT  no PLAN within the Plan Wait (cf_plan_wait_s); the law flies
--   HOLD     the turn complete, its line held (Transfer Control When Done off)
--   RUNIN    a PLAN with a run-in: its scheduled turn done, the run-in line held
--   DIVE     the run-in done, the dive onto the balloon

local C = require("constants")

local M = {}

M.OFF, M.WAIT, M.FLY = C.CF_STATE_OFF, C.CF_STATE_WAIT, C.CF_STATE_FLY
M.DONE, M.TIMEOUT = C.CF_STATE_DONE, C.CF_STATE_TIMEOUT
M.HOLD, M.RUNIN, M.DIVE = C.CF_STATE_HOLD, C.CF_STATE_RUNIN, C.CF_STATE_DIVE

-- What the law aims at this tick: the balloon, or the Curve Flyer's line.
local AIM_BALLOON, AIM_LINE = 0, 1

local NAN = 0 / 0
local MS_PER_S = 1000.0
local asin, atan, deg, rad = math.asin, math.atan, math.deg, math.rad
local floor, min, max, sqrt, sin, cos = math.floor, math.min, math.max, math.sqrt, math.sin, math.cos

--- A Curve Flyer's state, one per script.  `msg` is the PLAN parser's
--- caller-supplied table, so parsing allocates nothing.
function M.new()
  local cf = { state = M.OFF, plan_id = 0, wait_ms = 0, have_plan = false,
               eps0 = 0.0, knots = {},
               runin = false, runin_climb = 0.0, runin_dep = 0.0,
               dive_eps0 = 0.0,
               hold_n = NAN, hold_e = NAN, hold_d = NAN, msg = {},
               active = false, plan = false, aim = AIM_BALLOON, retarget = false,
               tx = 1.0, ty = 0.0, tz = 0.0 }
  M.clear_log(cf)
  return cf
end

--- The logged internals, blank on a tick the law flies no PLAN.
function M.clear_log(cf)
  cf.progress, cf.eps_deg, cf.bank_ref_deg = NAN, NAN, NAN
end

--- The chases the Curve Flyer flies: p and x.
function M.flies_mode(mode)
  return mode == C.GUIDE_MODE_PTEST or mode == C.GUIDE_MODE_BALLOON_CHASER
end

--- A fresh chase: WAIT for its PLAN when the Curve Flyer flies it, else OFF.
--- Any PLAN and held line from an earlier chase are discarded.
function M.start_chase(cf, now_ms, flies)
  cf.state = flies and M.WAIT or M.OFF
  cf.wait_ms = now_ms
  cf.have_plan = false
  cf.plan_id = 0
  cf.hold_n, cf.hold_e, cf.hold_d = NAN, NAN, NAN
  cf.aim = AIM_BALLOON
end

--- A PLAN from the link, parsed into cf.msg, for steering law `law`.  Taken
--- only while waiting for one, and only when it was made for that law: a
--- PLAN arriving at any other time belongs to no chase this script is
--- flying, and one made for the other law is in another measure of the bank.
function M.accept(cf, law)
  if cf.state ~= M.WAIT then return end
  local m = cf.msg
  if m.law ~= law then return end
  cf.plan_id = m.plan_id
  cf.eps0 = m.eps0_deg
  for k = 1, C.CURVE_N_KNOTS do cf.knots[k] = m["knot_" .. k .. "_deg"] end
  -- A run-in only with both of its numbers: NaN in either is a PLAN without
  -- one.
  local cl, dp = m.runin_climb_deg, m.runin_dep_deg
  cf.runin = cl == cl and dp == dp
  cf.runin_climb, cf.runin_dep = cl, dp
  cf.have_plan = true
  cf.state = M.FLY
end

--- The line the Curve Flyer aims the law at this tick, left in
--- cf.hold_n/e/d for track.lua to read in the path frame: in the scheduled
--- turn of a PLAN with a run-in and on its run-in, the run-in line, from the
--- balloon's world direction this tick (wx north, wy east); in a hold, the
--- line it keeps.  True while there is one.
function M.line(cf, wx, wy)
  if cf.runin and (cf.state == M.FLY or cf.state == M.RUNIN) then
    -- Straight above or below, the balloon has no compass direction, and the
    -- line stays where it was.
    local h = sqrt(wx * wx + wy * wy)
    if h > 0.0 then
      local cl = rad(cf.runin_climb)
      cf.hold_n, cf.hold_e, cf.hold_d = cos(cl) * wx / h, cos(cl) * wy / h, -sin(cl)
    end
  elseif cf.state ~= M.HOLD then
    cf.hold_n, cf.hold_e, cf.hold_d = NAN, NAN, NAN
  end
  return cf.hold_n == cf.hold_n
end

--- The schedule's bank at progress s, linear between knots.
local function scheduled_bank(cf, s)
  local n = C.CURVE_N_KNOTS
  local x = s * (n - 1)
  local i0 = min(floor(x), n - 2)
  local w = x - i0
  return cf.knots[i0 + 1] * (1.0 - w) + cf.knots[i0 + 2] * w
end

--- The angle off the path, deg, of a direction the path frame holds at
--- (tx, ty, tz).
local function angle_off(tx, ty, tz)
  return deg(atan(sqrt(ty * ty + tz * tz), tx))
end

--- Aim the law at a balloon the path frame holds at (tx, ty, tz), with bank b
--- at `progress`.
local function aim(cf, tx, ty, tz, progress, b)
  cf.plan = true
  cf.tx, cf.ty, cf.tz = tx, ty, tz
  cf.progress, cf.eps_deg, cf.bank_ref_deg = progress, angle_off(tx, ty, tz), b
end

--- The bank toward a balloon `ty` across the wings (y', + right):
--- CF_HOLD_BANK_PER_DEG per degree it lies across them, level on it, within
--- the bank limit either way.  A bank + right wing down turns the path right.
local function toward(ty, g)
  local lim = g.bank_lim_deg or 0.0
  local b = C.CF_HOLD_BANK_PER_DEG * deg(asin(max(-1.0, min(1.0, ty))))
  return max(-lim, min(lim, b))
end

--- A hold of the line track.lua read this tick: the hold's own, or the
--- run-in's.
local function hold(cf, s, g)
  cf.aim = AIM_LINE
  aim(cf, s.hold_path_x, s.hold_path_y, s.hold_path_z, 1.0, toward(s.hold_path_y, g))
end

--- The turn complete: the law flies as without a PLAN, or with Transfer
--- Control When Done off the hold keeps the line the path has now, which on
--- this tick lies straight ahead along the path.
local function complete(cf, s, g, transfer)
  if transfer then
    cf.state = M.DONE
    return false
  end
  cf.state = M.HOLD
  cf.hold_n, cf.hold_e, cf.hold_d = s.path_n, s.path_e, s.path_d
  cf.aim = AIM_LINE
  aim(cf, 1.0, 0.0, 0.0, 1.0, toward(0.0, g))
  return true
end

--- The states, one tick.  One tick may pass from the turn to the run-in to
--- the dive, as the conditions for each hold.
local function run(cf, now_ms, s, g)
  -- Hold only on the operator's word: with no setting received, the law
  -- flies after the turn as without a PLAN.
  local transfer = not (g.cf_transfer_when_done ~= nil and g.cf_transfer_when_done < 0.5)
  if cf.state == M.WAIT then
    -- With no Plan Wait received the law flies: the FC waits only as long
    -- as the operator's setting says.
    if now_ms - cf.wait_ms > (g.cf_plan_wait_s or 0.0) * MS_PER_S then
      cf.state = M.TIMEOUT
      return false
    end
    return true
  end
  if cf.state == M.HOLD then
    if transfer then
      cf.state = M.DONE
      cf.hold_n, cf.hold_e, cf.hold_d = NAN, NAN, NAN
      return false
    end
    hold(cf, s, g)
    return true
  end
  if cf.state == M.FLY then
    -- The turn aims at the run-in line where the PLAN has a run-in, else at
    -- the balloon.
    local tpx, tpy, tpz = s.balloon_path_x, s.balloon_path_y, s.balloon_path_z
    local on_line = cf.runin and s.hold_valid
    if on_line then
      tpx, tpy, tpz = s.hold_path_x, s.hold_path_y, s.hold_path_z
    end
    local eps = angle_off(tpx, tpy, tpz)
    if eps > C.CURVE_TURN_DONE_DEG then
      local prog = 1.0
      if cf.eps0 > 0.0 then prog = max(0.0, min(1.0, 1.0 - eps / cf.eps0)) end
      if on_line then cf.aim = AIM_LINE end
      aim(cf, tpx, tpy, tpz, prog, scheduled_bank(cf, prog))
      return true
    end
    if not cf.runin then
      return complete(cf, s, g, transfer)
    end
    cf.state = M.RUNIN
  end
  if cf.state == M.RUNIN then
    if s.hold_valid and -s.balloon_elev_deg < cf.runin_dep then
      hold(cf, s, g)
      return true
    end
    cf.state = M.DIVE
    cf.dive_eps0 = angle_off(s.balloon_path_x, s.balloon_path_y, s.balloon_path_z)
  end
  if cf.state ~= M.DIVE then return false end
  local tpx, tpy, tpz = s.balloon_path_x, s.balloon_path_y, s.balloon_path_z
  local eps = angle_off(tpx, tpy, tpz)
  if eps <= C.CURVE_TURN_DONE_DEG then
    return complete(cf, s, g, transfer)
  end
  local prog = 1.0
  if cf.dive_eps0 > 0.0 then prog = max(0.0, min(1.0, 1.0 - eps / cf.dive_eps0)) end
  aim(cf, tpx, tpy, tpz, prog, toward(tpy, g))
  return true
end

--- One tick, before the law runs.  s: the path frame track.lua left this
--- tick - the balloon (balloon_path_x/y/z), its elevation above the horizon
--- (balloon_elev_deg), the path's world direction (path_n/e/d), and the line
--- M.line left (hold_path_x/y/z, while hold_valid).
---
--- Leaves cf.active true while the Curve Flyer owns the chase: holding the
--- rates at zero for the PLAN (cf.plan false), or with cf.plan true, the law
--- to fly its PLAN MODE toward cf.tx/ty/tz (path frame) at the bank
--- cf.bank_ref_deg.  cf.retarget is true on the tick the law's aim point
--- changes between the balloon and a line, so a derivative of the law's
--- error re-seeds instead of differentiating across the jump.  Returns
--- cf.active.
function M.step(cf, now_ms, s, g)
  M.clear_log(cf)
  cf.plan = false
  local aim_was = cf.aim
  cf.aim = AIM_BALLOON
  cf.active = run(cf, now_ms, s, g)
  cf.retarget = cf.aim ~= aim_was
  return cf.active
end

return M
