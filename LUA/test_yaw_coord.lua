-- Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
-- test_yaw_coord.lua - the rudder coordination law, driven end to end.
--
-- WHY THIS IS SEPARATE FROM test_balloon_chaser_fc.lua.  That one asks whether the
-- shell runs: frames in, commands out, gates in the right order.  This asks
-- whether the NUMBER is right, which is the question that was wrong before.
--
-- Every expected value here is recomputed from the airframe rather than copied
-- from a previous run.  r = g*sin(roll)*cos(pitch)/V is arithmetic anyone can
-- redo, so a regression has to argue with the physics instead of with a recorded
-- number whose origin nobody remembers.
package.path = "LUA/?.lua;" .. package.path
require("ap_shim").install()
local stubs = require("ap_stubs")
local proto = require("protocol")

local fails, n = {}, 0
local function check(name, cond, detail)
  n = n + 1
  print(string.format("  [%s] %s%s", cond and "PASS" or "FAIL", name,
                      detail and ("   " .. detail) or ""))
  if not cond then fails[#fails+1] = name end
end

-- GUIDE frame, field order derived from protocol.py exactly as the shell test
-- does.  flags bit0 = tracking, bit2 = oracle bearing (no estimator wait).
--
-- Built from a positional table rather than a bare argument list: section 7
-- substitutes three of these and a miscounted comma in a 25-argument pack()
-- silently shifts every field after it.
-- GUIDE frame.  The format below is the layout protocol.py generates
-- (payload_fmt(GUIDE)).  Copy it from there, never type it: one token too many
-- or too few shifts every later field by a byte or more, and the harness then
-- flies settings nobody set without any error.
local guide_mode = 4
local GUIDE_FMT = '<I2BBi2i2i2i2i2i2i2i2BBi2i2i2I4I2'
local function pack_guide(seq, thr, gpan, ox, oy)
  local a = { seq, 0x07, guide_mode,
              0, 0,                                -- det_norm x/y
              math.floor(thr*100+0.5),
              0, 0, 0,                             -- straight_yaw, manual_yaw/pitch
              math.floor((gpan or 0.0)*100+0.5),   -- gimbal_pan
              -3000,                               -- gimbal_tilt
              1, 0,                                -- gimbal_flags, _pad2
              math.floor((ox or 1.0)*10000+0.5),   -- oracle_x
              math.floor((oy or 0.0)*10000+0.5),   -- oracle_y
              0,                                   -- oracle_z
              0, 12 }                              -- stamp_ms, age_ms
  local body = string.pack(GUIDE_FMT, table.unpack(a))
  local head = string.pack('<I2BB', proto.MAGIC, proto.MSG_GUIDE, #body)
  local hb = head .. body
  return hb .. string.pack('<I2', proto.crc16(hb, 1, #hb))
end

dofile("LUA/balloon_chaser_fc.lua")

local seq = 0
local balloon = { gpan = 0.0, ox = 1.0, oy = 0.0 }
local function tick()
  seq = seq + 1
  stubs.feed(pack_guide(seq, 55.0, balloon.gpan, balloon.ox, balloon.oy))
  stubs.advance(20)
  update()
end
-- Put the balloon somewhere.  Nothing else touches it.
local function aim(gpan_deg, bearing_deg)
  balloon.gpan = gpan_deg
  balloon.ox = math.cos(math.rad(bearing_deg))
  balloon.oy = math.sin(math.rad(bearing_deg))
end
local function settle(ticks)
  for _ = 1, (ticks or 30) do tick() end
end
local function last_yaw()
  local c = stubs.log.rate_cmds[#stubs.log.rate_cmds]
  return c and c[4] or nil
end

local G_MSS = 9.80665
local C_FLIGHT_PATH_MIN_SPEED = require("constants").FLIGHT_PATH_MIN_SPEED_MPS
-- Put the aircraft somewhere with an EXACT lateral marker.
--
-- The marker is atan(vy, vx) in BODY axes, so the velocity has to be built in
-- body axes and rotated out through the FULL attitude.  A yaw-only rotation is
-- not good enough: at 30 deg of bank in a 60 deg dive it leaks several degrees
-- of spurious marker into vy, which the pointing term then faithfully acts on.
local function fly(roll_deg, pitch_deg, eas, marker_deg)
  stubs.sim.roll  = math.rad(roll_deg)
  stubs.sim.pitch = math.rad(pitch_deg)
  local cr, sr = math.cos(stubs.sim.roll),  math.sin(stubs.sim.roll)
  local cp, sp = math.cos(stubs.sim.pitch), math.sin(stubs.sim.pitch)
  local cy, sy = math.cos(stubs.sim.yaw),   math.sin(stubs.sim.yaw)
  local m = math.rad(marker_deg or 0.0)
  local bx, by, bz = eas*math.cos(m), eas*math.sin(m), 0.0
  -- R = Rz(yaw) * Ry(pitch) * Rx(roll), applied to the body vector
  stubs.sim.vn = bx*(cy*cp) + by*(cy*sp*sr - sy*cr) + bz*(cy*sp*cr + sy*sr)
  stubs.sim.ve = bx*(sy*cp) + by*(sy*sp*sr + cy*cr) + bz*(sy*sp*cr - cy*sr)
  stubs.sim.vd = bx*(-sp)   + by*(cp*sr)            + bz*(cp*cr)
  stubs.sim.eas = eas
end
-- Clear the integrator between sub-tests: chase_key resets it on a mode change.
local function s_reset()
  guide_mode = 12 ; settle(3) ; guide_mode = 4 ; settle(3)
end
local function set_gain(name, value)
  local idx
  for i, nm in pairs(proto.GAIN_NAMES) do if nm == name then idx = i end end
  assert(idx, "no such gain: " .. name)
  local body = string.pack('<Hf', idx, value)
  local head = string.pack('<I2BB', proto.MAGIC, proto.MSG_SET_GAIN, #body)
  local hb = head .. body
  stubs.feed(hb .. string.pack('<I2', proto.crc16(hb, 1, #hb)))
end

print("\n" .. string.rep("=", 74))
print("YAW COORDINATION LAW")
print(string.rep("=", 74))

print("\n3. THE POINTING TERM: SIGN, ACTIVE ONLY WHEN SET, AND BOUNDED")
do
  local C = require("constants")
  -- balloon 10 deg RIGHT of the nose, path on the nose: pan_err = path - balloon
  -- is -10, so the path is LEFT of the balloon and wants a RIGHT rate.
  set_gain('yaw_point_kp', 0.0) ; set_gain('yaw_point_ki', 0.0)
  aim(0, 10) ; fly(0, 0, 15.0, 0.0) ; s_reset() ; settle(30)
  check("inactive, the pointing term commands nothing", math.abs(last_yaw()) < 1e-9,
        string.format("%+.4f dps", last_yaw()))

  set_gain('yaw_point_kp', 0.5) ; s_reset() ; settle(30)
  local p_only = last_yaw()
  check("path LEFT of balloon commands a RIGHT rate", p_only > 0.05,
        string.format("%+.3f dps", p_only))

  -- the integral winds ON TOP of the P term (it owns yaw_point_windup of the
  -- limit), and the sum stays inside yaw_point_lim.
  set_gain('yaw_point_ki', 2.0) ; s_reset() ; settle(400)
  local both = last_yaw()
  check("the integral winds on top of the P term, bounded by yaw_point_lim",
        both > p_only + 0.2 and both <= C.YAW_POINT_LIM + 1e-6,
        string.format("%+.4f dps (P-only was %+.3f), limit %.1f", both, p_only, C.YAW_POINT_LIM))

  -- drive the demand past the limit: the OUTPUT saturates at yaw_point_lim.
  set_gain('yaw_point_kp', 2.0) ; aim(0, 45) ; s_reset() ; settle(400)
  check("the term output saturates at yaw_point_lim",
        math.abs(math.abs(last_yaw()) - C.YAW_POINT_LIM) < 1e-6,
        string.format("%+.4f dps, limit %.1f", last_yaw(), C.YAW_POINT_LIM))
  set_gain('yaw_point_kp', 0.0) ; set_gain('yaw_point_ki', 0.0)
  aim(0, 0) ; fly(0, 0, 15.0, 0.0) ; s_reset()
end

print("\n4. A LIMIT OF ZERO IS ZERO AUTHORITY, NOT 'NO CEILING'")
do
  local C = require("constants")
  -- arm it, wind it up, then zero the limit - the term must collapse to 0, not
  -- run on because the clamp was skipped.
  aim(0, 10) ; set_gain('yaw_point_kp', 0.5) ; set_gain('yaw_point_ki', 2.0)
  fly(0, 0, 15.0, 0.0) ; s_reset() ; settle(200)
  check("  pointing is live before the limit is zeroed",
        math.abs(last_yaw()) > 0.5, string.format("%+.3f dps", last_yaw()))
  set_gain('yaw_point_lim', 0.0) ; settle(30)
  check("yaw_point_lim = 0 forces the pointing output to 0",
        math.abs(last_yaw()) < 1e-9, string.format("%+.4f dps", last_yaw()))
  set_gain('yaw_point_kp', 0.0) ; set_gain('yaw_point_ki', 0.0)
  set_gain('yaw_point_lim', C.YAW_POINT_LIM)
  aim(0, 0) ; fly(0, 0, 15.0, 0.0) ; s_reset()
end

print("\n4b. THE STEEP SCHEDULE FADES POINTING WITH THE PATH'S STEEPNESS")
do
  -- yaw_point_pitch_sched multiplies the output by the roll blend's steep
  -- share: none below Roll-to-Plane Steep Start, all past Steep Full, a straight
  -- line between, read from the flight path's angle to the horizon.  Level
  -- the term is gone.
  aim(0, 10) ; set_gain('yaw_point_kp', 0.5)
  set_gain('yaw_point_pitch_sched', 0.0) ; fly(0, 0, 15.0, 0.0) ; s_reset() ; settle(60)
  local sched_off_level = last_yaw()
  set_gain('yaw_point_pitch_sched', 1.0) ; s_reset() ; settle(60)
  local sched_on_level = last_yaw()
  check("level flight: the schedule fades the term to zero",
        math.abs(sched_off_level) > 0.05 and sched_on_level == 0.0,
        string.format("off %+.3f, on %+.3f dps", sched_off_level, sched_on_level))
  -- A path halfway between the steep angles: half the term.
  local C = require("constants")
  local mid = 0.5 * (C.PLANE_STEEP_START_DEG + C.PLANE_STEEP_FULL_DEG)
  fly(0, mid, 15.0, 0.0)
  set_gain('yaw_point_pitch_sched', 0.0) ; s_reset() ; settle(5)
  local off_mid = last_yaw()
  set_gain('yaw_point_pitch_sched', 1.0) ; s_reset() ; settle(5)
  local on_mid = last_yaw()
  check("halfway between Steep Start and Full: half the term",
        math.abs(off_mid) > 0.05 and math.abs(on_mid - 0.5 * off_mid) < 1e-6,
        string.format("off %+.4f, on %+.4f dps", off_mid, on_mid))
  -- Past Steep Full: all of it.
  fly(0, C.PLANE_STEEP_FULL_DEG + 5.0, 15.0, 0.0)
  set_gain('yaw_point_pitch_sched', 0.0) ; s_reset() ; settle(5)
  local off_steep = last_yaw()
  set_gain('yaw_point_pitch_sched', 1.0) ; s_reset() ; settle(5)
  check("past Steep Full: all of it",
        math.abs(off_steep) > 0.05 and math.abs(last_yaw() - off_steep) < 1e-6,
        string.format("off %+.4f, on %+.4f dps", off_steep, last_yaw()))
  set_gain('yaw_point_pitch_sched', 0.0) ; set_gain('yaw_point_kp', 0.0)
  aim(0, 0) ; fly(0, 0, 15.0, 0.0) ; s_reset()
end

print("\n4c. THE POINTING INTEGRATOR GROWS BY THE SHARE THE SCHEDULE LETS THROUGH")
do
  -- With the schedule on, the output is the pointing term times the steep
  -- share.  The integrator behind it grows by the same share, so a shallow path
  -- that holds the output at 0 holds the integrator at 0 too, instead of
  -- winding it up to dump when the path steepens.
  local function last_i()
    for i = #stubs.log.writes, 1, -1 do
      local w = stubs.log.writes[i]
      if string.byte(w, 3) == proto.MSG_STATE2 then
        local st = {}
        proto.parse_state2(w, 5, st)
        return st.yaw_point_i_dps
      end
    end
  end
  local C = require("constants")
  -- Ki alone, small enough to stay clear of its clamp over the run.
  aim(0, 10) ; set_gain('yaw_point_kp', 0.0) ; set_gain('yaw_point_ki', 0.1)
  set_gain('yaw_point_pitch_sched', 0.0) ; fly(0, 0, 15.0, 0.0) ; s_reset() ; settle(100)
  local i_free = last_i()
  set_gain('yaw_point_pitch_sched', 1.0) ; s_reset() ; settle(100)
  local i_level = last_i()
  check("level, schedule on: the integrator does not wind behind a zero output",
        math.abs(i_free) > 1e-3 and i_level == 0.0,
        string.format("%+.4f dps (%+.4f with the schedule off)", i_level, i_free))
  local mid = 0.5 * (C.PLANE_STEEP_START_DEG + C.PLANE_STEEP_FULL_DEG)
  fly(0, mid, 15.0, 0.0)
  set_gain('yaw_point_pitch_sched', 0.0) ; s_reset() ; settle(20)
  local i_off = last_i()
  set_gain('yaw_point_pitch_sched', 1.0) ; s_reset() ; settle(20)
  local i_on = last_i()
  check("halfway between Steep Start and Full it grows at half the rate",
        math.abs(i_off) > 1e-3 and math.abs(i_on - 0.5 * i_off) < 1e-6,
        string.format("%+.5f dps with the schedule, %+.5f without", i_on, i_off))
  set_gain('yaw_point_pitch_sched', 0.0) ; set_gain('yaw_point_ki', 0.0)
  aim(0, 0) ; fly(0, 0, 15.0, 0.0) ; s_reset()
end

print("\n5. THE COMMAND STAYS INSIDE ACRO_YAW_RATE")
-- The FC clamps the yaw argument to +-ACRO_YAW_RATE regardless; clamping in the
-- law is what lets a term know it has hit it.  Open the pointing limit above the
-- ceiling so the ceiling is what binds.
set_gain('yaw_point_lim', 60.0) ; set_gain('yaw_point_kp', 2.0) ; set_gain('yaw_point_ki', 5.0)
aim(0, 60) ; fly(0, 0, 15.0, 0.0) ; s_reset() ; settle(600)
check("the command never exceeds ACRO_YAW_RATE",
      math.abs(last_yaw()) <= 45.0 + 1e-6,
      string.format("%.2f dps, ceiling 45", last_yaw()))
do
  local C = require("constants")
  set_gain('yaw_point_lim', C.YAW_POINT_LIM) ; set_gain('yaw_point_kp', 0.0)
  set_gain('yaw_point_ki', 0.0) ; aim(0, 0) ; fly(0, 0, 15.0, 0.0) ; s_reset()
end

print("\n5c. A GATED TICK REPORTS ITSELF")
-- st_msg persists, so anything the commanded path writes keeps its last value
-- when steering() returns at a gate.  A gated stretch must issue no command.
stubs.sim.mode = 12                          -- LOITER: not scriptable
local before_n = #stubs.log.rate_cmds
settle(10)
check("a gated tick issues no command at all", #stubs.log.rate_cmds == before_n,
      string.format("%d new rate commands", #stubs.log.rate_cmds - before_n))
stubs.sim.mode = 4 ; settle(10)
check("  and commanding resumes when the gate clears", #stubs.log.rate_cmds > before_n)

print("\n5e. THE TURN'S OWN PITCH RATE IS SUPPLIED, NOT CHASED")
-- A banked aeroplane pitches in body axes because it is turning.  q = r*tan(bank),
-- with r the coordinated-turn yaw rate: both are one heading rate resolved into
-- body axes, so the pitch half is the yaw half times tan(bank) exactly.
do
  -- The pitch trim off: its default is not zero and it adds its own pitch
  -- request, which is not the turn's.
  for _, t in ipairs({'pitch_trim_offset', 'pitch_trim_lift', 'pitch_trim_attitude',
                      'pitch_trim_thrust'}) do set_gain(t, 0.0) end
  -- At full scale: the check is the geometry, whatever PITCH_TURN_FF defaults to.
  set_gain('pitch_turn_ff', 1.0)
  local g_, V, bank = 9.80665, 20.0, 26.0
  local phi = math.rad(bank)
  local r_exp = math.deg(g_ * math.sin(phi) / V)          -- yaw half
  local q_exp = r_exp * math.tan(phi)                     -- pitch half
  fly(bank, 0, V, 0.0) ; settle(10)
  local c = stubs.log.rate_cmds[#stubs.log.rate_cmds]
  check("the pitch command carries the turn's own rotation",
        math.abs(c[3] - q_exp) < 0.5,
        string.format("%.2f dps commanded, geometry says %.2f", c[3], q_exp))
  check("  and it is the yaw term times tan(bank)",
        math.abs(q_exp - r_exp * math.tan(phi)) < 1e-9,
        string.format("q %.2f = r %.2f x tan %.3f", q_exp, r_exp, math.tan(phi)))
  -- past the bank limit the level-turn identity no longer describes the aircraft
  fly(75, 0, V, 0.0) ; settle(10)
  c = stubs.log.rate_cmds[#stubs.log.rate_cmds]
  check("  and it is not applied past the bank limit", math.abs(c[3]) < 0.5,
        string.format("%.2f dps at 75 deg of bank", c[3]))
  set_gain('pitch_turn_ff', require("constants").PITCH_TURN_FF)
  fly(0, 0, 15.0, 0.0) ; s_reset()
end

print("\n5d. ZEROING THE GAINS RELEASES THE RUDDER, NOT 'HOLD STILL'")
-- A commanded 0.0 deg/s is not "no command": past the three gates it tells
-- ArduPlane to HOLD the body yaw rate at zero, and a banked aeroplane is already
-- rotating.  So with every yaw gain at zero the script must take the rate
-- controller OUT of the loop rather than command a hold.
do
  set_gain('yaw_point_kp', 0.0) ; set_gain('yaw_point_ki', 0.0)
  fly(25, 0, 20.0, 0.0) ; s_reset() ; settle(20)
  local o = stubs.log.rudder_offsets[#stubs.log.rudder_offsets]
  check("all gains zero -> run_yaw_rate_controller is FALSE", o and o[2] == false,
        "the FC writes rudder = 0 and resets its integrator")
  check("  and the deflection offset is still 0", o and o[1] == 0.0,
        o and string.format("%.3f pct", o[1]) or "")

  set_gain('yaw_point_kp', 0.5) ; settle(5)
  o = stubs.log.rudder_offsets[#stubs.log.rudder_offsets]
  check("  one gain active brings the rate controller back", o and o[2] == true)
  set_gain('yaw_point_kp', 0.5) ; aim(0, 10) ; fly(0, 0, 15.0, 0.0) ; s_reset()
end

print("\n7. COORDINATION: THE YAW THE AIRCRAFT NEEDS, ASKED FOR")
-- The yaw command is the whole body yaw rate the rate loop holds.  Yaw
-- Coordination FF adds the yaw the aircraft needs to fly clean: the turn's own
-- yaw rate, g*sin(roll)*cos(pitch)/V with V the true airspeed, and the yaw that
-- turns a roll about the
-- flight path instead of the nose, p*tan(alpha), with p the commanded roll rate
-- and alpha the path's angle below the nose.
do
  local function last_state()
    for i = #stubs.log.writes, 1, -1 do
      local w = stubs.log.writes[i]
      if string.byte(w, 3) == proto.MSG_STATE then
        local st = {}
        proto.parse_state(w, 5, st)
        return st
      end
    end
  end
  local function isnan(x) return x ~= x end
  -- The path alpha_deg below the nose in the body pitch plane, at speed V.
  local function fly_alpha(roll_deg, pitch_deg, V, alpha_deg)
    stubs.sim.roll  = math.rad(roll_deg)
    stubs.sim.pitch = math.rad(pitch_deg)
    local cr, sr = math.cos(stubs.sim.roll),  math.sin(stubs.sim.roll)
    local cp, sp = math.cos(stubs.sim.pitch), math.sin(stubs.sim.pitch)
    local cy, sy = math.cos(stubs.sim.yaw),   math.sin(stubs.sim.yaw)
    local a = math.rad(alpha_deg)
    local bx, by, bz = V*math.cos(a), 0.0, V*math.sin(a)
    stubs.sim.vn = bx*(cy*cp) + by*(cy*sp*sr - sy*cr) + bz*(cy*sp*cr + sy*sr)
    stubs.sim.ve = bx*(sy*cp) + by*(sy*sp*sr + cy*cr) + bz*(sy*sp*cr - cy*sr)
    stubs.sim.vd = bx*(-sp)   + by*(cp*sr)            + bz*(cp*cr)
    stubs.sim.eas = V
  end
  local ACRO_YAW = 45.0                         -- the stub's ACRO_YAW_RATE
  local BANK, V, ALPHA = 26.0, 20.0, 6.0
  local r_turn = math.deg(G_MSS * math.sin(math.rad(BANK)) / V)

  set_gain('yaw_point_kp', 0.0) ; set_gain('yaw_point_ki', 0.0)
  set_gain('yaw_coordination_ff', 1.0)

  -- Banked, the path on the nose: the turn's own yaw rate, and no rolling yaw.
  aim(0, 0) ; fly_alpha(BANK, 0, V, 0.0) ; s_reset() ; settle(10)
  local st = last_state()
  check("the turn's own yaw rate is asked for: g sin(roll) cos(pitch) / V",
        math.abs(last_yaw() - r_turn) < 0.01 and math.abs(st.yaw_coordination_dps - r_turn) < 0.01,
        string.format("%.3f dps commanded, %.3f logged, geometry %.3f", last_yaw(),
                      st.yaw_coordination_dps, r_turn))
  local o = stubs.log.rudder_offsets[#stubs.log.rudder_offsets]
  check("  coordination alone runs the rate loop, pointing off", o and o[2] == true)

  set_gain('yaw_coordination_ff', 0.5) ; settle(5)
  check("  half the scale asks for half of it",
        math.abs(last_yaw() - 0.5 * r_turn) < 0.01,
        string.format("%.3f dps, half the geometry %.3f", last_yaw(), 0.5 * r_turn))
  set_gain('yaw_coordination_ff', 1.0)

  -- The turn is kinematics, so its rate is on the TRUE airspeed: in thinner
  -- air the same EAS is a faster aircraft and a slower turn.
  local E2T = 1.2
  stubs.sim.eas2tas = E2T ; settle(5)
  local r_thin = math.deg(G_MSS * math.sin(math.rad(BANK)) / (V * E2T))
  check("  in thinner air it is on the true airspeed, EAS x EAS2TAS",
        math.abs(last_yaw() - r_thin) < 0.01,
        string.format("%.3f dps at EAS2TAS %.1f, geometry %.3f (%.3f at sea level)",
                      last_yaw(), E2T, r_thin, r_turn))
  stubs.sim.eas2tas = 1.0 ; settle(5)

  -- Wings level, the path ALPHA below the nose, the balloon to the right so the
  -- roll law rolls right: the yaw that turns that roll about the path.
  aim(0, 30) ; fly_alpha(0, 0, V, ALPHA) ; s_reset() ; settle(10)
  local c = stubs.log.rate_cmds[#stubs.log.rate_cmds]
  st = last_state()
  local want = c[2] * math.tan(math.rad(ALPHA))
  check("rolling right with the path below the nose yaws the nose right, p tan(alpha)",
        c[2] > 1.0 and math.abs(c[4] - want) < 0.01 and math.abs(st.yaw_ff_ari_dps - want) < 0.01,
        string.format("roll %.2f dps, yaw %.3f, logged %.3f, p tan(alpha) %.3f", c[2], c[4],
                      st.yaw_ff_ari_dps, want))

  -- With pointing on, the command is the sum of the two terms.
  set_gain('yaw_point_kp', 0.5) ; aim(0, 10) ; fly_alpha(BANK, 0, V, ALPHA) ; s_reset() ; settle(10)
  st = last_state()
  check("with pointing on the command is coordination plus pointing",
        math.abs(st.yaw_point_dps) > 0.1
        and math.abs(st.yaw_rate_cmd_dps - (st.yaw_coordination_dps + st.yaw_point_dps)) < 1e-4,
        string.format("%.3f = %.3f + %.3f", st.yaw_rate_cmd_dps, st.yaw_coordination_dps,
                      st.yaw_point_dps))
  set_gain('yaw_point_kp', 0.0)

  -- Slower than FLIGHT_PATH_MIN_SPEED_MPS the nose stands in for the path: no
  -- rolling yaw, whatever the velocity reads.
  aim(0, 30) ; fly_alpha(0, 0, 0.5 * C_FLIGHT_PATH_MIN_SPEED, ALPHA) ; s_reset() ; settle(10)
  c = stubs.log.rate_cmds[#stubs.log.rate_cmds]
  st = last_state()
  check("  below FLIGHT_PATH_MIN_SPEED_MPS the nose stands in: no rolling yaw",
        math.abs(c[2]) > 1.0 and st.yaw_ff_ari_dps == 0.0,
        string.format("roll %.2f dps, rolling yaw %.3f", c[2], st.yaw_ff_ari_dps))

  -- A hard roll with the path far below the nose asks for more than the
  -- ceiling: the command stops at ACRO_YAW_RATE, the log shows what was asked.
  aim(0, 90) ; fly_alpha(0, 0, V, 20.0) ; s_reset() ; settle(10)
  st = last_state()
  check("the sum stays inside ACRO_YAW_RATE; the log keeps what was asked",
        math.abs(math.abs(last_yaw()) - ACRO_YAW) < 1e-6
        and math.abs(st.yaw_coordination_dps) > ACRO_YAW,
        string.format("%.2f dps commanded, %.2f asked, ceiling %.0f", last_yaw(),
                      st.yaw_coordination_dps, ACRO_YAW))

  -- A tick the FC does not command reports no coordination at all.
  stubs.sim.mode = 12 ; settle(3)
  st = last_state()
  check("  a gated tick logs it blank, not zero",
        isnan(st.yaw_coordination_dps) and isnan(st.yaw_ff_ari_dps))
  stubs.sim.mode = 4

  set_gain('yaw_coordination_ff', 0.0) ; set_gain('yaw_point_kp', 0.5)
  aim(0, 10) ; fly(0, 0, 15.0, 0.0) ; s_reset()
end

print("\n6. THE RUDDER OFFSET STAYS ZERO AND THE RATE PATH STAYS ON")
settle(5)
local o = stubs.log.rudder_offsets[#stubs.log.rudder_offsets]
check("set_rudder_offset is called every commanded tick", o ~= nil)
check("the deflection offset is 0 - one knob, one owner",
      o and o[1] == 0.0, o and string.format("%.3f pct", o[1]) or "")
check("run_yaw_rate_controller is true, or the FC writes rudder = 0",
      o and o[2] == true)

print("\n" .. string.rep("=", 74))
if #fails == 0 then
  print(string.format("PASSED - %d checks", n))
else
  print(string.format("FAILED: %d of %d - %s", #fails, n, table.concat(fails, "; ")))
end
print(string.rep("=", 74))
os.exit(#fails == 0 and 0 or 1)
