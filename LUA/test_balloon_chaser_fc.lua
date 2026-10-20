-- Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
-- test_balloon_chaser_fc.lua - run the FC shell end to end against desktop stubs.
package.path = "LUA/?.lua;" .. package.path
local ap    = require("ap_shim").install()
local stubs = require("ap_stubs")
local proto = require("protocol")

local fails, n = {}, 0
local function check(name, cond, detail)
  n = n + 1
  print(string.format("  [%s] %s%s", cond and "PASS" or "FAIL", name,
                      detail and ("   " .. detail) or ""))
  if not cond then fails[#fails+1] = name end
end

print("\n" .. string.rep("=", 74))
print("FC SHELL - end to end against stubs")
print(string.rep("=", 74))

-- helper: build a GUIDE frame exactly as the Python packer does
local function pack_guide(seq, flags, mode, dnx, dny, thr)
  -- The layout is typed by hand, so it must follow protocol.py's GUIDE field
  -- for field: a stale layout still decodes, as silently wrong settings.
  -- test_control_path.py compares this format string (and pack_oracle_guide's)
  -- with protocol.py and fails the build when they differ.
  local body = string.pack('<I2BBi2i2i2i2i2i2i2i2BBi2i2i2I4I2',
    seq,
    flags,
    mode,
    math.floor(dnx*10000+0.5),
    math.floor(dny*10000+0.5),
    math.floor(thr*100+0.5),
    0,
    0,
    0,
    1000,
    -3000,
    1,
    0,
    10000,
    0,
    0,
    0,
    12)
  local head = string.pack('<I2BB', proto.MAGIC, proto.MSG_GUIDE, #body)
  local hb = head .. body
  return hb .. string.pack('<I2', proto.crc16(hb, 1, #hb))
end

print("\n1. LOAD")
local ok, upd = pcall(dofile, "LUA/balloon_chaser_fc.lua")
check("script loads without error", ok, ok and "" or tostring(upd))
check("init reported startup", #stubs.log.texts > 0,
      stubs.log.texts[1] or "")
check("SCALING_SPEED readable", stubs.param_values.SCALING_SPEED ~= nil)
local _SCALING_SPEED_AT_INIT = stubs.param_values.SCALING_SPEED

print("\n2. NO INPUT -> NO COMMAND")
stubs.advance(20); update()
check("no rate command before a GUIDE frame", #stubs.log.rate_cmds == 0)

print("\n3. STEADY GUIDE FRAMES")
ap.alloc_reset()
local base_alloc = 0
for i = 1, 200 do
  stubs.feed(pack_guide(i, 0x07, 4, 0.1*math.sin(i*0.1), -0.05, 55.0))
  stubs.sim.mount_pan  = 10.0*math.sin(i*0.03)
  stubs.sim.mount_tilt = -5.0*math.cos(i*0.02)
  stubs.sim.roll  = 0.3*math.sin(i*0.05)
  stubs.sim.pitch = 0.2*math.sin(i*0.02)
  stubs.sim.p = 0.1*math.cos(i*0.05)
  stubs.advance(20)
  update()
  if i == 20 then base_alloc = ap.alloc_count() end
end
check("rate commands issued", #stubs.log.rate_cmds > 100,
      #stubs.log.rate_cmds .. " commands")
-- The FC must NOT drive the gimbal: it is a Gazebo model commanded by the
-- companion over gz-transport, so a mount call here is either a no-op (there is
-- no mount driver) or a second controller fighting the first.  It consumes the
-- ENCODER angles from the GUIDE frame instead.
check("does NOT drive the gimbal", #stubs.log.mount_rates == 0,
      "gimbal pointing is companion-owned tier-1 sensor work")
local last = stubs.log.rate_cmds[#stubs.log.rate_cmds]
check("commands are finite", last[2] == last[2] and last[3] == last[3],
      string.format("thr=%.1f p=%.3f q=%.3f", last[1], last[2], last[3]))
-- The script must NEVER write SCALING_SPEED. Borrowing a parameter as a control
-- input disables ArduPlane's speed compensation and identifies an aircraft that
-- does not exist. The scaler is observed and reported, not suppressed.
check("SCALING_SPEED NOT overwritten",
      math.abs(stubs.param_values.SCALING_SPEED - _SCALING_SPEED_AT_INIT) < 1e-9,
      string.format("%.3f changed from %.3f - the pin is back",
                    stubs.param_values.SCALING_SPEED, _SCALING_SPEED_AT_INIT))
check("STATE telemetry emitted", #stubs.log.writes > 20,
      #stubs.log.writes .. " frames")

-- The FC writes STATE and then STATE2 every tick: pick the latest of one kind.
local function last_write(msgid)
  for i = #stubs.log.writes, 1, -1 do
    local w = stubs.log.writes[i]
    if string.byte(w, 3) == msgid then return w end
  end
end

print("\n4. STATE FRAME IS WELL FORMED")
local sf = last_write(proto.MSG_STATE)
local magic = string.byte(sf,1) | (string.byte(sf,2) << 8)
local msgid = string.byte(sf,3)
local plen  = string.byte(sf,4)
local want  = proto.crc16(sf, 1, 4+plen)
local got   = string.byte(sf,4+plen+1) | (string.byte(sf,4+plen+2) << 8)
check("magic correct", magic == proto.MAGIC)
check("msgid is STATE", msgid == proto.MSG_STATE)
check("crc valid", want == got)

print("\n5. SET_GAIN APPLIED LIVE")
local function pack_gain(idx, val)
  local body = string.pack('<I2f', idx, val)
  local head = string.pack('<I2BB', proto.MAGIC, proto.MSG_SET_GAIN, #body)
  local hb = head .. body
  return hb .. string.pack('<I2', proto.crc16(hb, 1, #hb))
end
local before = #stubs.log.rate_cmds
stubs.feed(pack_gain(0, 99.0))       -- index 0 = first ControlGains field
stubs.feed(pack_guide(300, 0x07, 4, 0.4, 0.0, 55.0))
stubs.advance(20); update()
check("gain frame did not break the loop", #stubs.log.rate_cmds > before)

-- The command must MOVE with the gain, not merely survive the frame.  ptest
-- geometry: wings level, nose level, flying north, the oracle balloon 5 deg
-- above the horizon dead ahead.  With pitch_ki = 0 the pitch command is
-- pitch_gain x 5 deg, so doubling the gain doubles the command on the tick
-- the frame lands.
local GAIN_IDX = {}
for i, name in pairs(proto.GAIN_NAMES) do GAIN_IDX[name] = i end
local ORACLE_EL = math.rad(5.0)
local function pack_oracle_guide(seq)
  local body = string.pack('<I2BBi2i2i2i2i2i2i2i2BBi2i2i2I4I2',
    seq, 0x06, 4,                               -- tracking + oracle, ptest
    0, 0, math.floor(55.0*100+0.5), 0, 0, 0, 0, 0, 1, 0,
    math.floor(math.cos(ORACLE_EL)*10000+0.5), 0,
    math.floor(-math.sin(ORACLE_EL)*10000+0.5),
    0, 12)
  local head = string.pack('<I2BB', proto.MAGIC, proto.MSG_GUIDE, #body)
  local hb = head .. body
  return hb .. string.pack('<I2', proto.crc16(hb, 1, #hb))
end
local function pitch_cmd()
  local c = stubs.log.rate_cmds[#stubs.log.rate_cmds]
  return c and c[3] or 0.0
end
local function oracle_tick(seq, extra)
  stubs.feed((extra or "") .. pack_oracle_guide(seq))
  stubs.advance(20); update()
  return pitch_cmd()
end
stubs.sim.roll, stubs.sim.pitch, stubs.sim.yaw = 0.0, 0.0, 0.0
stubs.sim.p, stubs.sim.q, stubs.sim.r = 0.0, 0.0, 0.0
-- The pitch trim off: its default is not zero, and it adds a part of the
-- command that does not scale with the gain.
stubs.feed(pack_gain(GAIN_IDX.pitch_ki, 0.0) .. pack_gain(GAIN_IDX.pitch_gain, 3.0)
           .. pack_gain(GAIN_IDX.pitch_trim_offset, 0.0) .. pack_gain(GAIN_IDX.pitch_trim_lift, 0.0)
           .. pack_gain(GAIN_IDX.pitch_trim_attitude, 0.0) .. pack_gain(GAIN_IDX.pitch_trim_thrust, 0.0))
for i = 1, 10 do oracle_tick(310 + i) end
local q3 = pitch_cmd()
local q6 = oracle_tick(330, pack_gain(GAIN_IDX.pitch_gain, 6.0))
check("pitch command follows a live pitch_gain edit on the same tick",
      q3 > 1.0 and math.abs(q6 / q3 - 2.0) < 0.02,
      string.format("gain 3 -> %.3f dps, gain 6 -> %.3f dps", q3, q6))

-- A frame split across two reads must survive a full read behind it.  With a
-- backlog on the port (a burst of gain frames, or a late script tick) the
-- next read takes RX_BUF_MAX bytes while the previous read left the head of a
-- frame waiting; trimming the buffer to RX_BUF_MAX at that point cuts the head
-- off and that frame - here the pitch_gain edit - is lost without a trace.
local split = pack_gain(GAIN_IDX.pitch_gain, 9.0)
stubs.feed(string.sub(split, 1, 5))
stubs.advance(20); update()                      -- takes the 5-byte head only
local filler = {}
for i = 1, 45 do filler[i] = pack_gain(GAIN_IDX.pitch_ki, 0.0) end
stubs.feed(string.sub(split, 6) .. table.concat(filler))   -- > 512 bytes behind it
stubs.advance(20); update()
local q9
for i = 1, 3 do q9 = oracle_tick(340 + i) end
-- 5%: a small part of the command does not scale with the gain, and the old
-- value (6) would read as 2x, far outside it.
check("a gain frame split across reads survives a full read behind it",
      math.abs(q9 / q3 - 3.0) < 0.15,
      string.format("gain 9 expected %.3f dps, got %.3f dps", 3.0 * q3, q9))

print("\n5b. PITCH TRIM")
-- The trim goes in as the pitch-rate request that makes ArduPlane's rate loop
-- output the trim elevator: trim_elev / (FF*scaler/EAS2TAS + P*scaler^2), added
-- after the envelope clip.  The stubs fly at eas 18 against SCALING_SPEED 15,
-- with the flown PTCH_RATE_FF 0.1744 and PTCH_RATE_P 1.0.
-- The four trim gains, and the trim they make at a pitch, airspeed, throttle
-- and EAS2TAS: offset + (VREF/V)^2 * (lift*cos + attitude*sin
-- + thrust*throttle/EAS2TAS^2), V floored at MAX(AIRSPEED_MIN, 5).
local TRIM_O, TRIM_L, TRIM_A, TRIM_T = 2.07, 16.04, -17.89, -6.99
local VREF = require("constants").PITCH_TRIM_VREF_MPS
local THR_PCT = 60.0
srv_scaled[70] = THR_PCT            -- ArduPlane's throttle output, percent
local function trim_at(pitch_rad, eas, e2t)
  local v = math.max(eas, math.max(stubs.param_values.AIRSPEED_MIN, 5.0))
  e2t = e2t or 1.0
  return TRIM_O + (VREF / v) ^ 2 * (TRIM_L * math.cos(pitch_rad)
                                    + TRIM_A * math.sin(pitch_rad)
                                    + TRIM_T * THR_PCT / 100.0 / (e2t * e2t))
end
local function per_dps(e2t)
  local sc = stubs.param_values.SCALING_SPEED / stubs.sim.eas
  return stubs.param_values.PTCH_RATE_FF * sc / e2t
       + stubs.param_values.PTCH_RATE_P * sc * sc
end
local function last_state()
  local sf = last_write(proto.MSG_STATE)
  local st = {}
  proto.parse_state(sf, 5, st)
  return st
end
local function last_state2()
  local sf = last_write(proto.MSG_STATE2)
  local st = {}
  proto.parse_state2(sf, 5, st)
  return st
end
local function trim_on()  return pack_gain(GAIN_IDX.pitch_trim_offset, TRIM_O)
                              .. pack_gain(GAIN_IDX.pitch_trim_lift, TRIM_L)
                              .. pack_gain(GAIN_IDX.pitch_trim_attitude, TRIM_A)
                              .. pack_gain(GAIN_IDX.pitch_trim_thrust, TRIM_T) end
local function trim_off() return pack_gain(GAIN_IDX.pitch_trim_offset, 0.0)
                              .. pack_gain(GAIN_IDX.pitch_trim_lift, 0.0)
                              .. pack_gain(GAIN_IDX.pitch_trim_attitude, 0.0)
                              .. pack_gain(GAIN_IDX.pitch_trim_thrust, 0.0) end
local function settle(seq0) for i = 1, 5 do oracle_tick(seq0 + i) end end

-- Compared WITHIN one tick: the law's own demand moves from tick to tick while
-- its estimates settle, so a before/after pair across two ticks measures that
-- drift too.  With wings level (no turn term), what reached the FC is the
-- law's clipped demand plus the trim.
local function added_this_tick()
  local st = last_state()
  local env = stubs.param_values.ACRO_PITCH_RATE
  local law = math.max(-env, math.min(env, st.omega_y_presat_dps))
  return pitch_cmd() - law, st
end

-- Level: the request adds trim / per_dps, and the STATE frame reports both.
stubs.feed(trim_off()); settle(400)
oracle_tick(410, trim_on())
local lvl = trim_at(0.0, stubs.sim.eas)
local want = lvl / per_dps(1.0)
local added, st = added_this_tick()
check("level: the trim adds trim / (FF*sc/EAS2TAS + P*sc^2) to the pitch request",
      math.abs(added - want) < 1e-3,
      string.format("added %.4f dps, expected %.4f", added, want))
check("level: STATE reports the trim elevator and the request carrying it",
      math.abs(st.pitch_trim_elev - lvl) < 1e-4 and math.abs(st.pitch_trim_dps - want) < 1e-3,
      string.format("elev %.4f (expected %.4f) dps %.4f", st.pitch_trim_elev, lvl,
                    st.pitch_trim_dps))

-- The airspeed share scales as (VREF/V)^2 and the offset does not: at VREF the
-- terms are the gains themselves.
local function elev_at_eas(eas, seq)
  stubs.sim.eas = eas
  oracle_tick(seq)
  return last_state().pitch_trim_elev
end
local e_ref = elev_at_eas(VREF, 411)
check("at VREF the trim is offset + lift + thrust x throttle (level)",
      math.abs(e_ref - (TRIM_O + TRIM_L + TRIM_T * THR_PCT / 100.0)) < 1e-4,
      string.format("elev %.4f", e_ref))
local e_2v = elev_at_eas(2.0 * VREF, 412)
check("at twice VREF the airspeed share is a quarter",
      math.abs((e_2v - TRIM_O) - 0.25 * (e_ref - TRIM_O)) < 1e-4,
      string.format("share %.4f at %.0f m/s, %.4f at %.0f m/s",
                    e_2v - TRIM_O, 2 * VREF, e_ref - TRIM_O, VREF))
local e_floor = elev_at_eas(3.0, 413)
check("below the airspeed floor the trim stops growing",
      math.abs(e_floor - trim_at(0.0, 3.0)) < 1e-4
      and math.abs(e_floor - trim_at(0.0, stubs.param_values.AIRSPEED_MIN)) < 1e-4,
      string.format("elev %.4f at 3 m/s", e_floor))
srv_scaled[70] = 0.0
local e_idle = elev_at_eas(VREF, 414)
srv_scaled[70] = THR_PCT
check("the thrust share follows ArduPlane's throttle output",
      math.abs((e_ref - e_idle) - TRIM_T * THR_PCT / 100.0) < 1e-4,
      string.format("idle %.4f, %.0f%% %.4f", e_idle, THR_PCT, e_ref))
stubs.sim.eas = 18.0
oracle_tick(415)

-- EAS2TAS divides the FF part of the conversion, and the thrust share by its
-- square: 1 / EAS2TAS^2 is the air density as a fraction of sea level's, and
-- the propeller's push at one throttle scales with it.  The wing shares
-- already follow the density through the EAS and take no factor.
stubs.sim.eas2tas = 1.2
oracle_tick(420)
want = trim_at(0.0, stubs.sim.eas, 1.2) / per_dps(1.2)
added = added_this_tick()
check("EAS2TAS divides the FF part of the conversion",
      math.abs(added - want) < 1e-3,
      string.format("added %.4f dps, expected %.4f", added, want))
local e_thin = elev_at_eas(VREF, 421)
check("thinner air shrinks the thrust share by 1/EAS2TAS^2, and nothing else",
      math.abs((e_thin - e_ref) - TRIM_T * THR_PCT / 100.0 * (1.0 / 1.44 - 1.0)) < 1e-4,
      string.format("elev %.4f at EAS2TAS 1.2, %.4f at 1.0", e_thin, e_ref))
check("STATE2 reports the EAS2TAS the trim used",
      math.abs(last_state2().eas2tas - 1.2) < 1e-6,
      string.format("eas2tas %.4f", last_state2().eas2tas))
stubs.sim.eas = 18.0
stubs.sim.eas2tas = 1.0
local q_off, q_on

-- 60 deg nose-up, flying along the nose so the markers stay at zero.  The law's
-- own demand is pinned at the envelope (the balloon is 55 deg below the nose),
-- and the trim is nose-down at this attitude.
stubs.sim.pitch = math.rad(60.0)
stubs.sim.vn, stubs.sim.vd = 18.0 * math.cos(math.rad(60.0)), -18.0 * math.sin(math.rad(60.0))
stubs.feed(trim_off()); settle(430)
q_off = pitch_cmd()
q_on = oracle_tick(440, trim_on())
want = trim_at(math.rad(60.0), stubs.sim.eas) / per_dps(1.0)
local st60 = last_state()
check("60 deg: the trim follows the pitch through cos and sin",
      math.abs(st60.pitch_trim_dps - want) < 1e-3,
      string.format("trim request %.4f dps, expected %.4f", st60.pitch_trim_dps, want))
-- ArduPlane holds the sum at +-ACRO_PITCH_RATE on receipt, so with the law
-- already at the envelope the trim is cut there.  The script makes the same
-- cut, so STATE logs the command flown rather than the sum asked for.
local ACRO_Q = stubs.param_values.ACRO_PITCH_RATE
check("60 deg, law at the envelope: the sum is held at ACRO_PITCH_RATE and logged as flown",
      math.abs(q_off + ACRO_Q) < 1e-6 and math.abs(q_on + ACRO_Q) < 1e-6
      and math.abs(st60.omega_y_cmd_dps - q_on) < 1e-6,
      string.format("law %.3f dps, sent %.3f, logged %.3f (ACRO_PITCH_RATE %.0f)",
                    q_off, q_on, st60.omega_y_cmd_dps, ACRO_Q))
stubs.feed(trim_off()); oracle_tick(448)
-- The law's envelope is ArduPlane's ACRO_PITCH_RATE at any setting, never a
-- constant of the script's: raised above the shipped value, the law uses it.
-- The script re-reads the ACRO rates once a second.
local function acro_pitch_to(v)
  stubs.param_values.ACRO_PITCH_RATE = v
  for _ = 1, 55 do oracle_tick(449) end
end
acro_pitch_to(ACRO_Q + 30.0)
check("the law's pitch envelope follows ACRO_PITCH_RATE above its shipped value",
      math.abs(pitch_cmd() + (ACRO_Q + 30.0)) < 1e-6,
      string.format("sent %.3f dps with ACRO_PITCH_RATE %.0f", pitch_cmd(), ACRO_Q + 30.0))
acro_pitch_to(ACRO_Q)

-- No airspeed: no scaler, so no conversion and no trim; the trim elevator,
-- which needs the airspeed, is NaN - blank in the log.
stubs.sim.eas_valid = false
q_off = oracle_tick(450, trim_off())
local n_cmds = #stubs.log.rate_cmds
q_on = oracle_tick(451, trim_on())
local st_na = last_state()
check("no airspeed: still commanded, but no trim is sent",
      #stubs.log.rate_cmds == n_cmds + 1
      and math.abs(q_on - q_off) < 1e-9 and st_na.pitch_trim_dps == 0.0,
      string.format("added %.6f dps", q_on - q_off))
check("no airspeed: the trim elevator reads NaN, not a number",
      st_na.pitch_trim_elev ~= st_na.pitch_trim_elev)

stubs.sim.eas_valid = true
stubs.sim.pitch = 0.0
stubs.sim.vn, stubs.sim.vd = 18.0, 0.0
stubs.feed(trim_off()); settle(460)

print("\n5c. THROTTLE: AIRSPEED HOLD ON THE FC")
-- GUIDE carries the OPERATOR's throttle (55 % in these frames).  The airspeed
-- loop runs on the FC; off, it hands the operator's throttle back unchanged.
local function thr_cmd()
  local c = stubs.log.rate_cmds[#stubs.log.rate_cmds]
  return c and c[1] or -1.0
end
stubs.feed(pack_gain(GAIN_IDX.airspeed_cmd_enable, 0.0))
oracle_tick(470)
check("airspeed loop off: the operator's throttle is flown unchanged",
      math.abs(thr_cmd() - 55.0) < 1e-6, string.format("throttle %.3f", thr_cmd()))
-- On, with airspeed below the setpoint: the loop adds throttle, and STATE says
-- it is active.
stubs.sim.eas = 12.0
stubs.feed(pack_gain(GAIN_IDX.airspeed_cmd_enable, 1.0)
        .. pack_gain(GAIN_IDX.airspeed_cmd_mps, 15.0)
        .. pack_gain(GAIN_IDX.airspeed_cmd_kp, 5.0)
        .. pack_gain(GAIN_IDX.airspeed_cmd_ki, 0.0)
        .. pack_gain(GAIN_IDX.airspeed_cmd_min_valid, 2.0))
oracle_tick(471)
local st_a = last_state()
check("airspeed loop on: it raises the throttle toward the setpoint, on the FC",
      thr_cmd() > 55.0 and (st_a.asp_flags & 1) == 1
      and math.abs(st_a.asp_err_mps - 3.0) < 1e-6
      and math.abs(st_a.throttle_out - thr_cmd()) < 1e-6,
      string.format("throttle %.3f, flags %d, err %.3f", thr_cmd(), st_a.asp_flags, st_a.asp_err_mps))
do
  -- The hold's integrator is inactive while its throttle is not flown (here
  -- a stand-down) and is cleared: the next engagement re-seeds from the
  -- operator's throttle, never resumes the integral it built before.
  stubs.feed(pack_gain(GAIN_IDX.airspeed_cmd_ki, 1.0))
  for i = 1, 3 do oracle_tick(4720 + i) end
  local built = last_state().asp_i_term
  stubs.feed(pack_guide(4724, 0x05, 4, 0.2, 0.0, 55.0)); stubs.advance(20); update()
  oracle_tick(4725)
  local resumed = last_state().asp_i_term
  local one_tick = (built - 55.0) / 3
  check("airspeed-hold integrator cleared by a stand-down, re-seeded from the operator",
        built > 55.0 and math.abs(resumed - (55.0 + one_tick)) < 0.5 * one_tick,
        string.format("built %.3f, resumed %.3f, one tick %.3f", built, resumed, one_tick))
  stubs.feed(pack_gain(GAIN_IDX.airspeed_cmd_ki, 0.0))
end
stubs.sim.eas = 18.0
stubs.feed(pack_gain(GAIN_IDX.airspeed_cmd_enable, 0.0))
oracle_tick(473)

print("\n5d. STATE ECHOES THE GUIDE SEQ IT WAS COMPUTED FROM")
-- The companion decides which of its frames an answer belongs to by this
-- echo (the 'p' test waits on it), so a counter of the FC's own ticks here
-- would pass a test before the FC had seen its balloon.
oracle_tick(40001)
local seq_a = last_state().seq
stubs.advance(20); update()                     -- a tick with no new frame
local seq_b = last_state().seq
check("STATE seq echoes the GUIDE seq, and holds until a new frame arrives",
      seq_a == 40001 and seq_b == 40001,
      string.format("after frame 40001: %d, one tick later: %d", seq_a, seq_b))

print("\n5e. STATE2: THE FC'S SENSORS, LAW INTERNALS AND SURFACES")
-- Sent right after STATE every tick, echoing the same GUIDE seq so the two
-- pair up; law internals are NaN (blank in the CSV) on a tick the law did
-- not run, never a stale value.
local function isnan(x) return x ~= x end
local sim0 = { p = stubs.sim.p, q = stubs.sim.q, r = stubs.sim.r,
               vn = stubs.sim.vn, ve = stubs.sim.ve, vd = stubs.sim.vd }
stubs.sim.p, stubs.sim.q, stubs.sim.r = 0.1, -0.2, 0.3
stubs.sim.vn, stubs.sim.ve, stubs.sim.vd = 17.0, 1.5, -0.5
srv_scaled[19] = -1234.5
srv_current[79] = 1620
oracle_tick(41001)
local a, b = last_state(), last_state2()
local s2f = last_write(proto.MSG_STATE2)
local plen2 = string.byte(s2f, 4)
check("STATE2 frame follows STATE with a valid crc",
      string.byte(s2f, 3) == proto.MSG_STATE2
      and proto.crc16(s2f, 1, 4 + plen2)
          == (string.byte(s2f, 5 + plen2) | (string.byte(s2f, 6 + plen2) << 8)))
check("STATE2 echoes the same GUIDE seq as STATE", b.seq == 41001 and a.seq == 41001,
      string.format("STATE %d, STATE2 %d", a.seq, b.seq))
check("STATE2 carries the FC's own gyro, velocity, airspeed and mode",
      math.abs(b.gyro_x_rads - 0.1) < 1e-6 and math.abs(b.gyro_z_rads - 0.3) < 1e-6
      and math.abs(b.vel_n_mps - 17.0) < 1e-5 and math.abs(b.vel_d_mps + 0.5) < 1e-5
      and math.abs(b.eas_mps - stubs.sim.eas) < 1e-5 and b.fc_mode == stubs.sim.mode,
      string.format("gyro %.3f/%.3f vel %.2f/%.2f eas %.2f mode %d",
                    b.gyro_x_rads, b.gyro_z_rads, b.vel_n_mps, b.vel_d_mps,
                    b.eas_mps, b.fc_mode))
check("STATE2 carries the surface demand and the servo signal",
      math.abs(b.out_elevator + 1234.5) < 1e-3 and b.pwm_vtail_l == 1620,
      string.format("elevator %.1f, vtail L %d", b.out_elevator, b.pwm_vtail_l))
local tn = math.sqrt(b.balloon_body_x^2 + b.balloon_body_y^2 + b.balloon_body_z^2)
check("on a commanded tick the law internals are numbers",
      math.abs(tn - 1.0) < 1e-4 and not isnan(b.bank_rate_cmd_dps)
      and not isnan(b.sideslip_deg) and not isnan(b.los_elev_deg)
      and not isnan(b.yaw_point_i_dps)
      and not isnan(b.plane_off_nose_deg) and not isnan(b.plane_err_deg)
      and not isnan(b.plane_side) and not isnan(b.plane_axis_up)
      and not isnan(b.plane_drift_dps) and not isnan(b.plane_ff_dps)
      and not isnan(b.plane_err_rate_dps) and not isnan(b.plane_d_dps)
      and not isnan(b.plane_path_share) and not isnan(b.plane_i_dps)
      and not isnan(b.plane_i_frozen) and not isnan(b.plane_path_deg)
      and not isnan(b.plane_path_from_vel),
      string.format("|balloon| %.4f", tn))
check("the quaternion is the one the law read",
      math.abs(b.q_w^2 + b.q_x^2 + b.q_y^2 + b.q_z^2 - 1.0) < 1e-4)
stubs.feed(pack_guide(41002, 0x05, 4, 0.2, 0.0, 55.0))   -- tracking clear
stubs.advance(35); update()
local c = last_state2()
check("on a tick the law did not run they are NaN, the sensors still live",
      isnan(c.balloon_body_x) and isnan(c.bank_rate_cmd_dps) and isnan(c.yaw_point_i_dps)
      and isnan(c.plane_off_nose_deg) and isnan(c.plane_err_deg)
      and isnan(c.plane_side) and isnan(c.plane_axis_up)
      and isnan(c.plane_drift_dps) and isnan(c.plane_ff_dps)
      and isnan(c.plane_err_rate_dps) and isnan(c.plane_d_dps)
      and isnan(c.plane_path_share) and isnan(c.plane_i_dps) and isnan(c.plane_i_frozen)
      and isnan(c.plane_path_deg) and isnan(c.plane_path_from_vel)
      and math.abs(c.gyro_y_rads + 0.2) < 1e-6)
check("dt_ms is the measured interval, not the nominal",
      last_state().dt_ms == 35, "dt_ms " .. last_state().dt_ms)
-- STATE's law internals follow the same rule: flags bit 3 clear, the floats
-- NaN and cs_flags 0 - never the value the last commanded tick left behind.
local cst = last_state()
check("STATE on a tick the law did not run: bit 3 clear, law internals NaN",
      (cst.flags & 0x08) == 0 and cst.cs_flags == 0
      and isnan(cst.steering_pan) and isnan(cst.theta_ip_deg)
      and isnan(cst.pan_integrator) and isnan(cst.requested_roll)
      and isnan(cst.omega_x_presat_dps) and isnan(cst.lat_marker_deg)
      and isnan(cst.omega_x_cmd_dps) and isnan(cst.throttle_out)
      and isnan(cst.pan_err_deg) and isnan(cst.world_est_trust),
      string.format("flags %d cs_flags %d steering_pan %s",
                    cst.flags, cst.cs_flags, tostring(cst.steering_pan)))
check("STATE's readings stay live on that tick",
      not isnan(cst.speed_scaler) and not isnan(cst.accel_z_mps2)
      and not isnan(cst.yaw_ff_coord_dps))
check("STATE on a commanded tick: bit 3 set, law internals are numbers",
      (a.flags & 0x08) ~= 0 and not isnan(a.steering_pan)
      and not isnan(a.requested_roll) and not isnan(a.omega_x_cmd_dps),
      string.format("flags %d", a.flags))
stubs.sim.p, stubs.sim.q, stubs.sim.r = sim0.p, sim0.q, sim0.r
stubs.sim.vn, stubs.sim.ve, stubs.sim.vd = sim0.vn, sim0.ve, sim0.vd
srv_scaled[19] = 0.0
srv_current[79] = 1500
oracle_tick(41003)

print("\n5f. SPEED SCALER: ARDUPLANE'S CLAMP, FROM AIRSPEED_MIN / AIRSPEED_MAX")
-- Attitude.cpp calc_speed_scaler: SCALING_SPEED / EAS, clamped to
-- [min(0.5, S/(2*AIRSPEED_MAX)), max(2, S/(0.7*max(AIRSPEED_MIN, 5)))], and
-- scale_max itself at ~0 airspeed.  The bounds need both parameters bound under
-- this firmware's names; a name that does not exist leaves them unclamped.
local S_REF = stubs.param_values.SCALING_SPEED
local function scaler_at(eas)
  stubs.sim.eas = eas
  oracle_tick(41100)
  return last_state().speed_scaler
end
local pv = stubs.param_values
local lo_b = math.min(0.5, S_REF / (2.0 * pv.AIRSPEED_MAX))
local hi_b = math.max(2.0, S_REF / (0.7 * math.max(pv.AIRSPEED_MIN, 5.0)))
check("inside the bounds: SCALING_SPEED / EAS",
      math.abs(scaler_at(18.0) - S_REF / 18.0) < 1e-5)
check("slow: clamped to max(2, S/(0.7*AIRSPEED_MIN))",
      math.abs(scaler_at(3.0) - hi_b) < 1e-5,
      string.format("%.4f, expected %.4f", last_state().speed_scaler, hi_b))
check("fast: clamped to min(0.5, S/(2*AIRSPEED_MAX))",
      math.abs(scaler_at(80.0) - lo_b) < 1e-5,
      string.format("%.4f, expected %.4f", last_state().speed_scaler, lo_b))
local amin0 = pv.AIRSPEED_MIN
pv.AIRSPEED_MIN = 2.0                 -- below ArduPlane's own floor of 5 m/s
check("AIRSPEED_MIN is floored at 5 m/s, as ArduPlane does",
      math.abs(scaler_at(1.0) - math.max(2.0, S_REF / (0.7 * 5.0))) < 1e-5,
      string.format("%.4f", last_state().speed_scaler))
pv.AIRSPEED_MIN = amin0
stubs.sim.eas_valid = false
check("no airspeed estimate: the scaler is NaN, not 0",
      isnan(scaler_at(18.0)))
stubs.sim.eas_valid = true
stubs.sim.eas = 18.0
oracle_tick(41101)

print("\n5g. THE PITCH RATE LOOP'S FF AND P ARE READ EVERY TICK")
-- A mode switch writes a different rate-loop set; the trim conversion must use
-- the values the FC holds on the tick it runs, not a copy from up to a second
-- earlier.
stubs.feed(trim_on()); settle(41110)
local p0 = pv.PTCH_RATE_P
pv.PTCH_RATE_P = 2.0 * p0
oracle_tick(41120)
local got_dps = last_state().pitch_trim_dps
local want_dps = trim_at(stubs.sim.pitch, stubs.sim.eas) / per_dps(1.0)
check("a changed PTCH_RATE_P reaches the trim on the next tick",
      math.abs(got_dps - want_dps) < 1e-3,
      string.format("%.4f dps, expected %.4f", got_dps, want_dps))
pv.PTCH_RATE_P = p0
stubs.feed(trim_off()); settle(41130)

print("\n5h. THE LATERAL MARKER IS GROUND-RELATIVE")
-- EKF3 books a steady sideslip as wind, so a marker taken against its wind
-- estimate reads the nose direction and the laws never see the sideslip.  The
-- marker is the ground track's angle to the nose, whatever the wind reads.
local ve0, wind_e0 = stubs.sim.ve, stubs.sim.wind_e
stubs.sim.ve = 0.3
oracle_tick(41140)
local lat_still = last_state().lat_marker_deg
local lat_want = math.deg(math.atan(0.3, stubs.sim.vn))
stubs.sim.wind_e = 0.3
oracle_tick(41141)
local lat_windy = last_state().lat_marker_deg
check("the lateral marker is the ground track's angle to the nose",
      math.abs(lat_still - lat_want) < 1e-4,
      string.format("%.4f deg, expected %.4f", lat_still, lat_want))
check("the EKF's wind estimate does not move the lateral marker",
      lat_windy == lat_still,
      string.format("%.4f deg with wind, %.4f without", lat_windy, lat_still))
stubs.sim.ve, stubs.sim.wind_e = ve0, wind_e0
oracle_tick(41142)

-- Both markers are taken as read, however far the path is off the nose: a
-- held or clipped reading misreports the path the aircraft is flying.
do
local Gm = require("geometry")
local function body_vec(n, e, d)
  local r00,r01,r02,r10,r11,r12,r20,r21,r22 =
    Gm.euler_to_rotation_matrix(stubs.sim.roll, stubs.sim.pitch, stubs.sim.yaw)
  return Gm.rotate(r00,r01,r02,r10,r11,r12,r20,r21,r22, n, e, d)
end
local vn0, vd0, yaw0 = stubs.sim.vn, stubs.sim.vd, stubs.sim.yaw
stubs.sim.ve = stubs.sim.vn * math.tan(math.rad(70.0))
stubs.sim.vd = -stubs.sim.vn * math.tan(math.rad(70.0))
oracle_tick(41143)
local mbx, mby, mbz = body_vec(stubs.sim.vn, stubs.sim.ve, stubs.sim.vd)
local lat_got, vel_got = last_state().lat_marker_deg, last_state().vel_marker_deg
check("a lateral marker far off the nose is taken as read",
      math.abs(lat_got - math.deg(math.atan(mby, mbx))) < 1e-3,
      string.format("%.3f deg, expected %.3f", lat_got, math.deg(math.atan(mby, mbx))))
check("a vertical marker far off the nose is taken as read",
      math.abs(vel_got - math.deg(math.atan(-mbz, mbx))) < 1e-3,
      string.format("%.3f deg, expected %.3f", vel_got, math.deg(math.atan(-mbz, mbx))))

-- THE PATH FRAME: the balloon as seen along the path, x' the path, y' square
-- to it and to the body z axis, z' = x' x y'.  The pan and pitch errors read
-- the balloon in it, from this tick's velocity.
local function in_path_frame(tbx, tby, tbz, pbx, pby, pbz)
  local sp = math.sqrt(pbx * pbx + pby * pby + pbz * pbz)
  local px, py, pz = pbx / sp, pby / sp, pbz / sp
  local h = math.sqrt(px * px + py * py)
  return tbx * px + tby * py + tbz * pz,
         (-tbx * py + tby * px) / h,
         (-tbx * pz * px - tby * pz * py) / h + tbz * h
end
local ORACLE_N, ORACLE_D = math.cos(ORACLE_EL), -math.sin(ORACLE_EL)
-- Path 10 deg right of the nose and 8 deg below it.
stubs.sim.ve = stubs.sim.vn * math.tan(math.rad(10.0))
stubs.sim.vd = stubs.sim.vn * math.tan(math.rad(8.0))
oracle_tick(41144)
local tbx1, tby1, tbz1 = body_vec(ORACLE_N, 0.0, ORACLE_D)
local tx1, ty1, tz1 = in_path_frame(tbx1, tby1, tbz1,
                                    body_vec(stubs.sim.vn, stubs.sim.ve, stubs.sim.vd))
local pan_want = -math.deg(math.asin(ty1))
local pitch_want = math.deg(math.atan(-tz1, math.sqrt(tx1 * tx1 + ty1 * ty1)))
local st5 = last_state()
check("the pan error is the balloon's angle to the side of the path, on this tick",
      math.abs(st5.pan_err_deg - pan_want) < 1e-2,
      string.format("%.3f deg, expected %.3f", st5.pan_err_deg, pan_want))
check("the pitch error is its elevation above the path, on this tick",
      math.abs(st5.theta_ip_deg - pitch_want) < 1e-2,
      string.format("%.3f deg, expected %.3f", st5.theta_ip_deg, pitch_want))
-- Yawing the nose turns the balloon and the path together about the body z
-- axis the frame is built on: the pan error does not move.
stubs.sim.yaw = yaw0 + math.rad(5.0)
oracle_tick(41145)
check("yawing the nose does not move the pan error",
      math.abs(last_state().pan_err_deg - st5.pan_err_deg) < 1e-3,
      string.format("%.4f deg, was %.4f", last_state().pan_err_deg, st5.pan_err_deg))
stubs.sim.yaw = yaw0
-- Moving backward and to the right of the nose, the balloon well to the left:
-- 130 deg right of the path the short way, 230 deg left the long way.  The
-- rear-hemisphere continuation reads it the short way, no further than 90.
stubs.sim.yaw = math.rad(80.0)
stubs.sim.vn, stubs.sim.ve, stubs.sim.vd = 18.0 * math.cos(math.rad(80.0 + 150.0)),
                                           18.0 * math.sin(math.rad(80.0 + 150.0)), 0.0
oracle_tick(41146)
local pan_back = last_state().pan_err_deg
check("a balloon behind the path reads the short way round, no further than 90",
      pan_back < 0.0 and pan_back >= -90.0,
      string.format("%.3f deg", pan_back))
stubs.sim.vn, stubs.sim.ve, stubs.sim.vd, stubs.sim.yaw = vn0, ve0, vd0, yaw0
settle(41147)
end

print("\n5i. PITCH KI IS 1/s: THE INTEGRATOR SUMS DEGREES x SECONDS")
-- A balloon 10 deg above dead ahead, held: the integrator grows by the error in
-- degrees times the time, with no cap at any settle angle, and the command
-- grows by Ki times that.
local KI_P, EL_P, N_P = 1.0, math.rad(10.0), 25
local function pack_el_guide(seq, el)
  local body = string.pack('<I2BBi2i2i2i2i2i2i2i2BBi2i2i2I4I2',
    seq, 0x06, 4, 0, 0, math.floor(55.0*100+0.5), 0, 0, 0, 0, 0, 1, 0,
    math.floor(math.cos(el)*10000+0.5), 0, math.floor(-math.sin(el)*10000+0.5),
    0, 12)
  local head = string.pack('<I2BB', proto.MAGIC, proto.MSG_GUIDE, #body)
  local hb = head .. body
  return hb .. string.pack('<I2', proto.crc16(hb, 1, #hb))
end
stubs.sim.roll, stubs.sim.pitch, stubs.sim.yaw = 0.0, 0.0, 0.0
stubs.feed(pack_gain(GAIN_IDX.pitch_ki, KI_P) .. pack_gain(GAIN_IDX.pitch_gain, 0.5))
local sum_err, first, last = 0.0, nil, nil
for i = 1, N_P do
  stubs.feed(pack_el_guide(44000 + i, EL_P))
  stubs.advance(20); update()
  local st5 = last_state()
  if i == 1 then first = st5 else sum_err = sum_err + st5.theta_ip_deg * 0.020 end
  last = st5
end
local grew = last.pitch_integrator - first.pitch_integrator
check("the integrator grows by the error in degrees x seconds, no 5 deg cap",
      math.abs(grew - sum_err) < 1e-3 and grew > 4.0,
      string.format("grew %.4f deg.s, error x time %.4f", grew, sum_err))
check("and the command by Ki times that",
      math.abs((last.omega_y_presat_dps - first.omega_y_presat_dps) - KI_P * grew) < 0.05,
      string.format("command grew %.3f dps, Ki x integrator %.3f",
                    last.omega_y_presat_dps - first.omega_y_presat_dps, KI_P * grew))
stubs.feed(pack_gain(GAIN_IDX.pitch_ki, 0.0) .. pack_gain(GAIN_IDX.pitch_gain, 3.0))

print("\n6. EXCITE RUNS LOCALLY")
local function pack_excite(kind, axis, replace, amp, f0, dur)
  local body = string.pack('<BBBBfffI4', kind, axis, replace, 0, amp, f0, 0.0, dur)
  local head = string.pack('<I2BB', proto.MAGIC, proto.MSG_EXCITE, #body)
  local hb = head .. body
  return hb .. string.pack('<I2', proto.crc16(hb, 1, #hb))
end
stubs.feed(pack_excite(5, 0, 0, 40.0, 0.5, 4000))   -- SQUARE, add, 2 s period
-- Measure the SWING, not the sign.  Now that steering actually commands, the
-- square wave rides on a steering offset instead of straddling zero - the old
-- "went positive AND negative" assertion only held while the steering gate was
-- blocking every tick, i.e. while nothing worked.
local lo, hi = math.huge, -math.huge
for i = 1, 150 do
  stubs.feed(pack_guide(400+i, 0x07, 4, 0.0, 0.0, 55.0))
  stubs.advance(20); update()
  local c = stubs.log.rate_cmds[#stubs.log.rate_cmds]
  if c then
    if c[2] < lo then lo = c[2] end
    if c[2] > hi then hi = c[2] end
  end
end
-- amplitude 40 dps, so a full square wave spans ~80 dps peak-to-peak
check("square wave swings the roll command", (hi - lo) > 60.0,
      string.format("peak-to-peak %.1f dps (%.1f .. %.1f)", hi - lo, lo, hi))
stubs.feed(pack_excite(0, 0, 0, 0.0, 0.0, 0))       -- dur 0 cancels
for i = 1, 5 do oracle_tick(560 + i) end

-- The axis EXCITE names is the axis excited.  A step on PITCH, added: the pitch
-- command moves by the amplitude and the roll command does not.
local function cmds() local c = stubs.log.rate_cmds[#stubs.log.rate_cmds]; return c[2], c[3] end
for i = 1, 5 do oracle_tick(570 + i) end
local r0, q0 = cmds()
stubs.feed(pack_excite(4, 1, 0, 7.0, 0.0, 2000))    -- STEP, pitch, add
oracle_tick(580)
local r1, q1 = cmds()
check("a pitch excitation adds to the pitch command, not the roll",
      math.abs((q1 - q0) - 7.0) < 0.5 and math.abs(r1 - r0) < 0.5,
      string.format("pitch %+.2f dps, roll %+.2f dps", q1 - q0, r1 - r0))
stubs.feed(pack_excite(4, 1, 1, -12.0, 0.0, 2000))  -- STEP, pitch, replace
stubs.feed(trim_on())
oracle_tick(581)
local _, q2 = cmds()
check("a pitch excitation that replaces is the whole pitch command",
      math.abs(q2 + 12.0) < 1e-6, string.format("pitch %.3f dps", q2))
stubs.feed(pack_excite(0, 0, 0, 0.0, 0.0, 0) .. trim_off())
for i = 1, 5 do oracle_tick(590 + i) end

print("\n7. ERROR CONTAINMENT")
local saved = _G.ahrs.get_quaternion
_G.ahrs.get_quaternion = function() error("simulated AHRS fault") end
local before2 = #stubs.log.rate_cmds
stubs.feed(pack_guide(900, 0x07, 4, 0.0, 0.0, 55.0))
stubs.advance(20)
local ok2 = pcall(update)
check("update() survives an internal fault", ok2)
check("no command issued during the fault", #stubs.log.rate_cmds == before2)
check("a script error releases control", not stubs.sim.nav_owned)
check("and STATE still goes out that tick, carrying the fault (bits 5-7)",
      (last_state().flags >> 5) == proto.FAULT_SCRIPT_ERROR,
      "flags " .. last_state().flags)
_G.ahrs.get_quaternion = saved
-- The fault holds until the companion stands down: it halts on the flag and
-- its hand-back clears tracking, and only then may the script claim again.
stubs.feed(pack_guide(901, 0x07, 4, 0.0, 0.0, 55.0))
stubs.advance(20); update()
check("no claim while the fault is latched", not stubs.sim.nav_owned
      and #stubs.log.rate_cmds == before2 and last_state().gate == 8,
      "gate " .. last_state().gate)
stubs.feed(pack_guide(902, 0x05, 4, 0.0, 0.0, 55.0))
stubs.advance(20); update()
check("tracking off clears the fault", (last_state().flags >> 5) == 0)
stubs.feed(pack_guide(903, 0x07, 4, 0.0, 0.0, 55.0))
stubs.advance(20); update()
check("recovers once tracking resumes", #stubs.log.rate_cmds > before2)




-- ---------------------------------------------------------------------------
print("\n8. THE SCRIPT MUST CLAIM CONTROL, NOT JUST COMPUTE")
-- set_target_throttle_rate_rpy only STORES the demand.  ArduPlane uses it only
-- while nav_scripting_active(), which nav_scripting_enable() sets.  Without the
-- claim the aircraft flies control_mode->run() - ACRO with neutral sticks -
-- while the script looks like it is working.
local n_before = #stubs.log.nav_enable
stubs.sim.mode = 4                                    -- ACRO
for i = 1, 5 do
  stubs.feed(pack_guide(1000 + i, 0x07, 4, 0.2, 0.0, 55.0))
  stubs.advance(20); update()
end
check("claims control every tick", #stubs.log.nav_enable >= n_before + 5,
      (#stubs.log.nav_enable - n_before) .. " claims in 5 ticks")
check("claims with ACRO specifically, not whatever is live",
      stubs.log.nav_enable[#stubs.log.nav_enable] == 4)
check("owns the loop in ACRO", stubs.sim.nav_owned)

-- LOITER is in nav_scripting_enable's ACCEPTED list, so claiming with the LIVE
-- mode would hijack it.  This project switches to LOITER to recover from an
-- aborted test, so a script that re-took control on the next tick would fly the
-- last steering command into the ground - it must claim ACRO specifically.
local cmds_loiter = #stubs.log.rate_cmds
stubs.sim.mode = 12                                   -- LOITER
for i = 1, 5 do
  stubs.feed(pack_guide(1500 + i, 0x07, 4, 0.2, 0.0, 55.0))
  stubs.advance(20); update()
end
check("RELEASES control in LOITER", #stubs.log.rate_cmds == cmds_loiter,
      "LOITER is scriptable - the script must stand down anyway")
check("ownership dropped in LOITER", not stubs.sim.nav_owned)

stubs.sim.mode = 4
stubs.feed(pack_guide(1600, 0x07, 4, 0.2, 0.0, 55.0))
stubs.advance(20); update()
check("resumes when ACRO returns", stubs.sim.nav_owned)

local cmds_before = #stubs.log.rate_cmds
stubs.sim.mode = 11                                   -- RTL: not scriptable
for i = 1, 5 do
  stubs.feed(pack_guide(2000 + i, 0x07, 4, 0.2, 0.0, 55.0))
  stubs.advance(20); update()
end
check("REFUSES to command when it does not own the loop",
      #stubs.log.rate_cmds == cmds_before,
      "a failsafe that switches mode must keep control")
check("ownership flag cleared", not stubs.sim.nav_owned)

stubs.sim.mode = 4
stubs.feed(pack_guide(3000, 0x07, 4, 0.2, 0.0, 55.0))
stubs.advance(20); update()
check("takes control back when the mode returns", stubs.sim.nav_owned
      and #stubs.log.rate_cmds > cmds_before)

-- GUIDE arrives every tick, tracking or not.  "Not tracking" in ACRO must hand
-- the surfaces back on that tick, not leave ArduPlane flying the last command
-- until its 1000 ms nav_scripting timeout.
local cmds_track = #stubs.log.rate_cmds
stubs.feed(pack_guide(3001, 0x05, 4, 0.2, 0.0, 55.0))  -- tracking bit clear
stubs.advance(20); update()
check("releases control in ACRO the tick tracking stops",
      not stubs.sim.nav_owned and stubs.log.nav_enable[#stubs.log.nav_enable] == 255
      and #stubs.log.rate_cmds == cmds_track,
      "last nav_enable " .. tostring(stubs.log.nav_enable[#stubs.log.nav_enable]))
for i = 2, 5 do
  stubs.feed(pack_guide(3000 + i, 0x05, 4, 0.2, 0.0, 55.0))
  stubs.advance(20); update()
end
check("stays released while not tracking", not stubs.sim.nav_owned
      and #stubs.log.rate_cmds == cmds_track)
stubs.feed(pack_guide(3010, 0x07, 4, 0.2, 0.0, 55.0))
stubs.advance(20); update()
check("re-claims when tracking resumes", stubs.sim.nav_owned
      and #stubs.log.rate_cmds == cmds_track + 1)

do
  -- GUIDE staleness: the companion sends a new frame every tick, so a gap past
  -- GUIDE_STALE_MS means it has stopped, and the script must stand down rather
  -- than fly the frozen frame.  ArduPilot's own timeout cannot do it: it never
  -- fires while the script keeps calling.
  local STALE_MS = require("constants").GUIDE_STALE_MS
  local stale_ticks = math.floor(STALE_MS / 20) + 3
  for i = 1, stale_ticks do stubs.advance(20); update() end
  check("no GUIDE for GUIDE_STALE_MS releases control",
        not stubs.sim.nav_owned and last_state().gate == 7,
        "gate " .. last_state().gate)
  stubs.feed(pack_guide(3011, 0x07, 4, 0.2, 0.0, 55.0))
  stubs.advance(20); update()
  check("a fresh GUIDE takes control back", stubs.sim.nav_owned
        and last_state().gate == 0, "gate " .. last_state().gate)
  -- The same frame again and again is no fresher than none: a companion stuck
  -- mid-loop can resend one, and only a NEW seq counts.
  for i = 1, stale_ticks do
    stubs.feed(pack_guide(3011, 0x07, 4, 0.2, 0.0, 55.0)); stubs.advance(20); update()
  end
  check("a repeated GUIDE seq goes stale too",
        not stubs.sim.nav_owned and last_state().gate == 7,
        "gate " .. last_state().gate)
  stubs.feed(pack_guide(3012, 0x07, 4, 0.2, 0.0, 55.0))
  stubs.advance(20); update()
  check("and the next new seq takes control back", stubs.sim.nav_owned)
end

print("\n8b. A FRESH START: STAND-DOWN, RETURN TO ACRO, CEILINGS, NaN")
-- A GUIDE frame aiming at a world direction, with the flags given.
local function pack_flags_aim(seq, flags, wx, wy, wz)
  local body = string.pack('<I2BBi2i2i2i2i2i2i2i2BBi2i2i2I4I2',
    seq, flags, 3, 0, 0, math.floor(55.0*100+0.5), 0, 0, 0, 0, 0, 1, 0,
    math.floor(wx*10000+0.5), math.floor(wy*10000+0.5), math.floor(wz*10000+0.5),
    0, 12)
  local head = string.pack('<I2BB', proto.MAGIC, proto.MSG_GUIDE, #body)
  local hb = head .. body
  return hb .. string.pack('<I2', proto.crc16(hb, 1, #hb))
end
local seq8 = 8000
local function tick8(frame)
  seq8 = seq8 + 1
  stubs.feed(frame); stubs.advance(20); update()
end
stubs.sim.mode = 4
stubs.sim.roll, stubs.sim.pitch, stubs.sim.yaw = 0.0, 0.0, 0.0
stubs.sim.p, stubs.sim.q, stubs.sim.r = 0.0, 0.0, 0.0
stubs.sim.vn, stubs.sim.ve, stubs.sim.vd, stubs.sim.eas = 18.0, 0.0, 0.0, 18.0

-- The balloon estimate: built from detections, ended by a stand-down, so the
-- next balloon chaser never starts from the previous balloon's bearing.
for i = 1, 10 do tick8(pack_guide(seq8, 0x03, 3, 0.0, 0.0, 55.0)) end
check("a balloon chaser builds a balloon estimate", (last_state().flags & 0x02) ~= 0)
tick8(pack_guide(seq8, 0x00, 3, 0.0, 0.0, 55.0))
check("standing down ends it", (last_state().flags & 0x02) == 0)
tick8(pack_guide(seq8, 0x02, 3, 0.0, 0.0, 55.0))
local st8 = last_state()
check("the next balloon chaser waits for its own balloon", st8.gate == 4
      and (st8.flags & 0x02) == 0, "gate " .. st8.gate)

-- The integrators: the law runs whatever the FC's mode, but in a mode the
-- script does not fly its output reaches nothing, so every integrator is
-- inactive there and stays cleared; back in ACRO it builds from zero.  Gains
-- that keep the roll command off its limit, where the integrator would freeze.
stubs.feed(pack_gain(GAIN_IDX.pan_p_gain, 3.5) .. pack_gain(GAIN_IDX.pan_ki, 0.6)
           .. pack_gain(GAIN_IDX.pan_kd, 0.0) .. pack_gain(GAIN_IDX.roll_gain, 4.0)
           .. pack_gain(GAIN_IDX.bank_lim_deg, 45.0))
local w5x, w5y = math.cos(math.rad(5.0)), math.sin(math.rad(5.0))
stubs.sim.mode = 5                                    -- FBWA
for i = 1, 25 do tick8(pack_flags_aim(seq8, 0x06, w5x, w5y, 0.0)) end
local pi_away = last_state().pan_integrator
stubs.sim.mode = 4
for i = 1, 25 do tick8(pack_flags_aim(seq8, 0x06, w5x, w5y, 0.0)) end
local pi_back = last_state().pan_integrator
check("the pan integrator stays cleared while FBWA flies", pi_away == 0.0,
      string.format("%.3f", pi_away))
check("and builds from zero once ACRO flies the law", math.abs(pi_back) > 0.5,
      string.format("%.3f -> %.3f", pi_away, pi_back))

do
  -- An integrator whose gain is switched off is inactive and is cleared, not
  -- held: held, the log shows a stale value and turning the gain back on adds
  -- it to the command at once.  The pitch integrator, built on a balloon 5 deg
  -- above the path, then Ki to 0.
  local e5x, e5z = math.cos(math.rad(5.0)), -math.sin(math.rad(5.0))
  stubs.feed(pack_gain(GAIN_IDX.pitch_ki, 0.5))
  for i = 1, 25 do tick8(pack_flags_aim(seq8, 0x06, e5x, 0.0, e5z)) end
  local qi_on = last_state().pitch_integrator
  stubs.feed(pack_gain(GAIN_IDX.pitch_ki, 0.0))
  tick8(pack_flags_aim(seq8, 0x06, e5x, 0.0, e5z))
  local qi_off = last_state().pitch_integrator
  check("the pitch integrator builds while its gain is on", math.abs(qi_on) > 0.1,
        string.format("%.3f", qi_on))
  check("and is cleared the tick its gain goes to 0", qi_off == 0.0,
        string.format("%.3f -> %.3f", qi_on, qi_off))
  stubs.feed(pack_gain(GAIN_IDX.pitch_ki, require("constants").PITCH_KI))
end

-- The pitch integrator freezes at the ceiling the command is clipped to,
-- ACRO_PITCH_RATE at whatever it is set to.  10 deg of error at the default
-- gain asks for more than a 20 deg/s ceiling.  The script re-reads the ACRO
-- rates once a second.
local acro_q8 = stubs.param_values.ACRO_PITCH_RATE
stubs.param_values.ACRO_PITCH_RATE = 20.0
stubs.advance(1100)
stubs.feed(pack_gain(GAIN_IDX.pitch_ki, 0.5))
local e10 = math.rad(10.0)
for i = 1, 5 do tick8(pack_flags_aim(seq8, 0x06, math.cos(e10), 0.0, -math.sin(e10))) end
st8 = last_state()
local c8 = stubs.log.rate_cmds[#stubs.log.rate_cmds]
check("a pitch demand past ACRO_PITCH_RATE freezes the pitch integrator",
      (st8.cs_flags & 0x08) ~= 0 and math.abs(c8[3]) <= 20.0 + 1e-6,
      string.format("cs_flags %d, pitch command %.2f", st8.cs_flags, c8[3]))
stubs.param_values.ACRO_PITCH_RATE = acro_q8
stubs.advance(1100)
stubs.feed(pack_gain(GAIN_IDX.pitch_ki, 0.0))
tick8(pack_flags_aim(seq8, 0x06, math.cos(e10), 0.0, -math.sin(e10)))

-- The airspeed floor: an AIRSPEED_MIN of 0 with no airspeed must not divide
-- by zero, and nothing that is not a number may reach ArduPlane.
local aspd_min0 = stubs.param_values.AIRSPEED_MIN
stubs.param_values.AIRSPEED_MIN = 0.0
stubs.sim.eas = 0.0
stubs.advance(1100)                       -- the cached parameters are re-read once a second
for i = 1, 3 do tick8(pack_flags_aim(seq8, 0x06, w5x, w5y, 0.0)) end
c8 = stubs.log.rate_cmds[#stubs.log.rate_cmds]
check("AIRSPEED_MIN 0 with no airspeed still commands numbers",
      c8[2] == c8[2] and c8[3] == c8[3] and c8[4] == c8[4],
      string.format("roll %s, pitch %s, yaw %s", c8[2], c8[3], c8[4]))
stubs.param_values.AIRSPEED_MIN = aspd_min0
stubs.sim.eas = 18.0
stubs.advance(1100)
tick8(pack_flags_aim(seq8, 0x06, w5x, w5y, 0.0))

print("\n9. THE LAW ALLOCATES NOTHING PER TICK")
-- Garbage collection in the tick surfaces on the FC as "exceeded time limit",
-- and ArduPilot's sandbox has no collectgarbage to see it coming.  So it is
-- measured here: the geometry and the law, wrapped, with the collector stopped
-- around each call.
local T_mod, K_mod = require("track"), require("cascade")
local real_track, real_law = T_mod.track_world_balloon, K_mod.apply_control_law
local grew_kb, calls = 0.0, 0
local function measured(f)
  return function(...)
    collectgarbage("stop")
    local before = collectgarbage("count")
    local r1, r2, r3, r4, r5, r6, r7 = f(...)
    grew_kb = grew_kb + (collectgarbage("count") - before)
    calls = calls + 1
    collectgarbage("restart")
    return r1, r2, r3, r4, r5, r6, r7
  end
end
local function fly(seq0)
  for i = 1, 50 do
    stubs.sim.roll = 0.3 * math.sin(i * 0.1)
    stubs.sim.pitch = 0.2 * math.sin(i * 0.07)
    oracle_tick(seq0 + i)
  end
end
-- Warm up first, wrapped: a table grows once, the first time a branch assigns
-- a key it has not held before, and the Lua stack grows once when the wrapper
-- first adds depth.  Both are one-offs, not per-tick allocations.
T_mod.track_world_balloon = measured(real_track)
K_mod.apply_control_law = measured(real_law)
fly(600)
grew_kb, calls = 0.0, 0
fly(700)
T_mod.track_world_balloon, K_mod.apply_control_law = real_track, real_law
check("track_world_balloon and apply_control_law allocate nothing",
      calls == 100 and grew_kb == 0.0,
      string.format("%d calls, %.3f KB allocated", calls, grew_kb))

print("\n10. ROLL-TO-PLANE BLEND")
-- On a steep flight path the roll axis rolls the balloon to whichever of 12 and
-- 6 o'clock round the nose is nearer; on a shallow one it banks toward the
-- balloon.  Roll-to-plane's share is the flight-path share, a straight line
-- between its two angles, and bank-to-turn has the rest.
local G = require("geometry")
-- Roll-to-plane's gain differs from bank-to-turn's, so a command built with
-- the wrong one fails the blend checks.  Its feedforward and integrator are
-- off until their own checks.  The log's 74 deg climb is past STEEP_FULL.
local RG, PG, HOLD = 4.0, 6.0, 15.0
local STEEP_START, STEEP_FULL = 60.0, 70.0
stubs.feed(pack_gain(GAIN_IDX.pan_p_gain, 3.5) .. pack_gain(GAIN_IDX.pan_ki, 0.6)
           .. pack_gain(GAIN_IDX.pan_kd, 0.4) .. pack_gain(GAIN_IDX.roll_gain, RG)
           .. pack_gain(GAIN_IDX.bank_lim_deg, 45.0)
           .. pack_gain(GAIN_IDX.plane_steep_start_deg, STEEP_START)
           .. pack_gain(GAIN_IDX.plane_steep_full_deg, STEEP_FULL)
           .. pack_gain(GAIN_IDX.plane_side_hold_deg, HOLD)
           .. pack_gain(GAIN_IDX.plane_gain, PG)
           .. pack_gain(GAIN_IDX.plane_ki, 0.0)
           .. pack_gain(GAIN_IDX.plane_ff_gain, 0.0))
local sim2 = { vn = stubs.sim.vn, ve = stubs.sim.ve, vd = stubs.sim.vd, eas = stubs.sim.eas }
-- The world direction that sits OFF deg off the nose at clock CLOCK (0 over the
-- canopy, +90 the right wingtip) for the aircraft's attitude now.
local function world_at(off_deg, clock_deg)
  local o, a = math.rad(off_deg), math.rad(clock_deg)
  local bx, by, bz = math.cos(o), math.sin(o) * math.sin(a), -math.sin(o) * math.cos(a)
  local r00,r01,r02,r10,r11,r12,r20,r21,r22 =
    G.euler_to_rotation_matrix(stubs.sim.roll, stubs.sim.pitch, stubs.sim.yaw)
  return r00*bx + r10*by + r20*bz, r01*bx + r11*by + r21*bz, r02*bx + r12*by + r22*bz
end
local function pack_aim(seq, mode, wx, wy, wz)
  local body = string.pack('<I2BBi2i2i2i2i2i2i2i2BBi2i2i2I4I2',
    seq, 0x06, mode, 0, 0, math.floor(85.0*100+0.5), 0, 0, 0, 0, 0, 1, 0,
    math.floor(wx*10000+0.5), math.floor(wy*10000+0.5), math.floor(wz*10000+0.5),
    0, 12)
  local head = string.pack('<I2BB', proto.MAGIC, proto.MSG_GUIDE, #body)
  local hb = head .. body
  return hb .. string.pack('<I2', proto.crc16(hb, 1, #hb))
end
local seq10 = 43000
local function aim_tick(mode, wx, wy, wz)
  seq10 = seq10 + 1
  stubs.feed(pack_aim(seq10, mode, wx, wy, wz))
  stubs.advance(20); update()
  return last_state(), last_state2()
end
local function blended(st, s2)
  local branch = RG * st.roll_err_deg
  return (1.0 - s2.plane_path_share) * branch
         + s2.plane_path_share * (PG * s2.plane_err_deg + s2.plane_i_dps
                                  + s2.plane_d_dps + s2.plane_ff_dps)
end
stubs.sim.roll, stubs.sim.pitch, stubs.sim.yaw = 0.0, 0.0, 0.0
stubs.sim.p, stubs.sim.q, stubs.sim.r = 0.0, 0.0, 0.0
stubs.sim.vn, stubs.sim.ve, stubs.sim.vd, stubs.sim.eas = 18.0, 0.0, 0.0, 18.0

local function btt_exactly(st, s2)
  return math.abs(st.omega_x_presat_dps - RG * st.roll_err_deg) < 1e-3
end
local function wv(heading_deg, elev_deg)
  local h, e = math.rad(heading_deg), math.rad(elev_deg)
  return math.cos(e) * math.cos(h), math.cos(e) * math.sin(h), -math.sin(e)
end
-- The nose at PITCH and the flight path at PATH (along the nose unless
-- given), heading north at SPEED.
local SPEED = 18.0
local function fly(pitch_deg, path_deg)
  local p = math.rad(path_deg or pitch_deg)
  stubs.sim.pitch = math.rad(pitch_deg)
  stubs.sim.vn, stubs.sim.ve, stubs.sim.vd = SPEED * math.cos(p), 0.0, -SPEED * math.sin(p)
end

-- SHALLOW FLIGHT: banking toward the balloon flies alone.
local st, s2 = aim_tick(3, wv(0.0, 60.0))
check("level, balloon 60 deg above ahead: shallow, so no share, bank-to-turn's command",
      s2.plane_path_share == 0.0 and btt_exactly(st, s2),
      string.format("path share %.3f", s2.plane_path_share))
st, s2 = aim_tick(3, wv(100.0, 0.0))
check("level, balloon 100 deg right at the same height: no share",
      s2.plane_path_share == 0.0 and btt_exactly(st, s2),
      string.format("path share %.3f", s2.plane_path_share))

-- STEEP FLIGHT, 85 deg down along the nose: roll-to-plane has the whole share.
fly(-85.0)
st, s2 = aim_tick(3, world_at(20.0, 90.0))
check("85 deg nose-down, balloon 20 deg off the nose: roll-to-plane's rate is the "
      .. "whole command",
      s2.plane_path_share == 1.0
      and math.abs(st.omega_x_presat_dps - PG * s2.plane_err_deg) < 1e-2
      and math.abs(st.omega_x_presat_dps - blended(st, s2)) < 1e-2,
      string.format("path share %.3f, command %.3f vs %.3f", s2.plane_path_share,
                    st.omega_x_presat_dps, PG * s2.plane_err_deg))
check("  and the pan integrator is held, and cleared with no share left to it",
      (st.cs_flags & 0x02) ~= 0 and st.pan_integrator == 0.0,
      string.format("cs_flags %d, pan integrator %.4f", st.cs_flags, st.pan_integrator))
-- A balloon a degree to the right of the nose at its height reads 3 o'clock,
-- 90 deg still to roll, and roll-to-plane asks for all of it.
st, s2 = aim_tick(3, world_at(1.0, 90.0))
check("a balloon a degree off the nose: the clock's 90 deg, roll-to-plane's whole rate",
      math.abs(s2.plane_off_nose_deg - 1.0) < 1e-2 and math.abs(s2.plane_err_deg - 90.0) < 0.5
      and math.abs(st.omega_x_presat_dps - PG * s2.plane_err_deg) < 1e-2,
      string.format("off %.2f, clock %.2f, command %.3f", s2.plane_off_nose_deg,
                    s2.plane_err_deg, st.omega_x_presat_dps))

-- THE PATH LINE.
local STEEP_MID = 0.5 * (STEEP_START + STEEP_FULL)
fly(-STEEP_MID)
st, s2 = aim_tick(3, world_at(20.0, 90.0))
check("halfway between the steep angles nose-down: half bank-to-turn's command and "
      .. "half roll-to-plane's",
      math.abs(s2.plane_path_share - 0.5) < 1e-4
      and math.abs(st.omega_x_presat_dps - blended(st, s2)) < 1e-2
      and math.abs(st.omega_x_presat_dps - 0.5 * RG * st.roll_err_deg
                   - 0.5 * PG * s2.plane_err_deg) < 1e-2
      and math.abs(st.roll_err_deg) > 1.0,
      string.format("path share %.4f, command %.3f vs %.3f", s2.plane_path_share,
                    st.omega_x_presat_dps, blended(st, s2)))
check("the integrator is held while roll-to-plane has a share",
      (st.cs_flags & 0x02) ~= 0, "cs_flags " .. st.cs_flags)
fly(STEEP_MID)
st, s2 = aim_tick(3, world_at(80.0, 30.0))
check("climbing the same: the path's size counts, not its sign",
      math.abs(s2.plane_path_share - 0.5) < 1e-4,
      string.format("path share %.4f", s2.plane_path_share))
fly(-50.0)
st, s2 = aim_tick(3, world_at(80.0, 30.0))
check("below Steep Start, balloon far off the nose: no share",
      s2.plane_path_share == 0.0 and btt_exactly(st, s2),
      string.format("path share %.4f", s2.plane_path_share))
stubs.feed(pack_gain(GAIN_IDX.plane_steep_start_deg, 90.0)
           .. pack_gain(GAIN_IDX.plane_steep_full_deg, 90.0))
fly(-85.0)
st, s2 = aim_tick(3, world_at(80.0, 30.0))
check("Steep 90 / 90: roll-to-plane does not fly",
      s2.plane_path_share == 0.0 and btt_exactly(st, s2),
      string.format("path share %.4f", s2.plane_path_share))
stubs.feed(pack_gain(GAIN_IDX.plane_steep_start_deg, STEEP_START)
           .. pack_gain(GAIN_IDX.plane_steep_full_deg, STEEP_FULL))

-- THE FLIGHT PATH, not the nose and not the bank.
stubs.sim.roll = math.rad(90.0); fly(0.0)
st, s2 = aim_tick(3, world_at(80.0, 0.0))
check("knife-edge, nose on the horizon, moving level: gravity all across the "
      .. "path, no share",
      s2.plane_path_share == 0.0
      and math.abs(s2.plane_path_deg) < 1e-3 and s2.plane_path_from_vel == 1.0,
      string.format("path %.3f deg, path share %.3f", s2.plane_path_deg,
                    s2.plane_path_share))
stubs.sim.roll = 0.0
fly(-75.0, -50.0)
st, s2 = aim_tick(3, world_at(80.0, 30.0))
check("nose 75 deg down, path 50 deg down: the path decides, no share",
      math.abs(s2.plane_path_deg + 50.0) < 1e-3 and s2.plane_path_share == 0.0,
      string.format("path %.3f deg, path share %.3f", s2.plane_path_deg, s2.plane_path_share))
fly(0.0, -75.0)
st, s2 = aim_tick(3, world_at(80.0, 30.0))
check("nose on the horizon, path 75 deg down: the path decides, all of it",
      math.abs(s2.plane_path_deg + 75.0) < 1e-3 and s2.plane_path_share == 1.0,
      string.format("path %.3f deg, path share %.3f", s2.plane_path_deg, s2.plane_path_share))
local CONST = require("constants")
fly(-85.0, 0.0)
local slow = 0.5 * CONST.FLIGHT_PATH_MIN_SPEED_MPS / SPEED
stubs.sim.vn, stubs.sim.vd = stubs.sim.vn * slow, stubs.sim.vd * slow
st, s2 = aim_tick(3, world_at(80.0, 30.0))
check("slower than FLIGHT_PATH_MIN_SPEED_MPS: the nose stands in for the path",
      s2.plane_path_from_vel == 0.0 and math.abs(s2.plane_path_deg + 85.0) < 1e-3
      and s2.plane_path_share == 1.0,
      string.format("from velocity %d, path %.3f deg, path share %.3f",
                    s2.plane_path_from_vel, s2.plane_path_deg, s2.plane_path_share))
fly(0.0)

-- ROLL-TO-PLANE'S CLOCK IS READ ROUND THE NOSE, in body axes.  The balloons
-- are placed in the path frame track.lua builds - x' the path, y' square to
-- it and to the body z axis, z' = x' x y' - where the other errors read them.
do
local function world_at_path(off_deg, clock_deg)
  local o, a = math.rad(off_deg), math.rad(clock_deg)
  local tpx, tpy, tpz = math.cos(o), math.sin(o) * math.sin(a), -math.sin(o) * math.cos(a)
  local r00,r01,r02,r10,r11,r12,r20,r21,r22 =
    G.euler_to_rotation_matrix(stubs.sim.roll, stubs.sim.pitch, stubs.sim.yaw)
  local vx, vy, vz = G.rotate(r00,r01,r02,r10,r11,r12,r20,r21,r22,
                              stubs.sim.vn, stubs.sim.ve, stubs.sim.vd)
  local sp = math.sqrt(vx * vx + vy * vy + vz * vz)
  local px, py, pz = vx / sp, vy / sp, vz / sp
  local h = math.sqrt(px * px + py * py)
  local yx, yy = -py / h, px / h
  local zx, zy, zz = -pz * px / h, -pz * py / h, h
  local bx = tpx * px + tpy * yx + tpz * zx
  local by = tpx * py + tpy * yy + tpz * zy
  local bz = tpx * pz + tpz * zz
  return r00*bx + r10*by + r20*bz, r01*bx + r11*by + r21*bz, r02*bx + r12*by + r22*bz
end
local function nose_clock(s2x)
  return math.deg(math.atan(s2x.balloon_body_y, -s2x.balloon_body_z))
end
-- A near-vertical climb, nose 82 deg up and the path 89, 7 deg toward the
-- canopy from it; the balloon 1.25 deg off the path, 56 deg round the path's
-- clock and about 8 round the nose's.  The law reads the nose's.
local OFF, CLK, NOSE_TO_PATH, CLK_TOL = 1.25, 56.0, 7.0, 1e-3
stubs.sim.roll = 0.0; fly(82.0, 82.0 + NOSE_TO_PATH)
local cx, cy, cz = world_at_path(OFF, CLK)
st, s2 = aim_tick(3, cx, cy, cz)
local clock_nose = s2.plane_err_deg
check("the clock is read round the nose, not the path",
      s2.plane_side == 1
      and math.abs(clock_nose - nose_clock(s2)) < CLK_TOL and math.abs(clock_nose) < 10.0,
      string.format("clock %.2f, round the nose %.2f, round the path %.0f",
                    clock_nose, nose_clock(s2), CLK))
-- The same climb with the balloon halfway between the nose and the path, 1 deg
-- to the right: below the path and above the nose.  Round the path it sits
-- near 164 deg, where 6 is nearer and the roll to it is LEFT; round the nose
-- near 16, at 12's side, and the law rolls right, toward the balloon's side
-- (ARCHITECTURE.md, "Roll-to-plane").
local MID, MID_SIDE, MID_NOSE_CLK, MID_TOL = NOSE_TO_PATH / 2.0, 1.0, 16.0, 0.5
local mx, my, mz = world_at_path(math.sqrt(MID * MID + MID_SIDE * MID_SIDE),
                                 180.0 - math.deg(math.atan(MID_SIDE, MID)))
st, s2 = aim_tick(3, mx, my, mz)
check("a balloon between the nose and the path, to the right: 12 round the nose, "
      .. "and the law rolls right, where the path's clock would roll left to 6",
      s2.plane_side == 1 and math.abs(s2.plane_err_deg - MID_NOSE_CLK) < MID_TOL
      and st.omega_x_presat_dps > 0.0,
      string.format("side %d, clock %.2f, roll command %.1f dps", s2.plane_side,
                    s2.plane_err_deg, st.omega_x_presat_dps))
-- The rate loop rolls the aircraft about the nose, so a roll turns the
-- balloon round the nose's clock by exactly the angle rolled.
local ROLLED = 10.0
stubs.sim.roll = math.rad(ROLLED)
st, s2 = aim_tick(4, cx, cy, cz)
local turned = clock_nose - s2.plane_err_deg
check("a roll about the nose turns the clock by exactly the angle rolled",
      math.abs(turned - ROLLED) < CLK_TOL,
      string.format("rolled %.2f, clock turned %.4f", ROLLED, turned))
-- Sideslipping, path 8 deg right of the nose, nose 80 deg down: a balloon over
-- the path at 12 o'clock round it is where bank-to-turn is done - no
-- sideways error - but round the nose it is off 12, so roll-to-plane still
-- rolls.
stubs.sim.roll = 0.0; fly(-80.0)
stubs.sim.ve = SPEED * math.tan(math.rad(8.0))
cx, cy, cz = world_at_path(57.5, 0.0)
st, s2 = aim_tick(3, cx, cy, cz)
check("sideslipping, the balloon over the path: bank-to-turn is done, roll-to-plane "
      .. "reads it off 12 round the nose",
      math.abs(st.pan_err_deg) < 0.02 and math.abs(s2.plane_err_deg - nose_clock(s2)) < CLK_TOL
      and math.abs(s2.plane_err_deg) > 4.0,
      string.format("pan error %.3f, clock %.3f, round the nose %.3f",
                    st.pan_err_deg, s2.plane_err_deg, nose_clock(s2)))
stubs.sim.roll = 0.0; fly(0.0)
end

-- The 12-or-6 choice and its hold band, balloon 80 deg off the nose, fixed in
-- the world; rolling the aircraft moves it round the clock.  A new chase
-- (mode 4) makes a fresh choice.
local tx, ty, tz = world_at(80.0, 80.0)
st, s2 = aim_tick(4, tx, ty, tz)
local ok1 = s2.plane_side == 1 and math.abs(s2.plane_err_deg - 80.0) < 0.05
stubs.sim.roll = math.rad(-20.0); st, s2 = aim_tick(4, tx, ty, tz)
local ok2 = s2.plane_side == 1 and math.abs(s2.plane_err_deg - 100.0) < 0.05
stubs.sim.roll = math.rad(-30.0); st, s2 = aim_tick(4, tx, ty, tz)
local ok3 = s2.plane_side == -1 and math.abs(s2.plane_err_deg + 70.0) < 0.05
stubs.sim.roll = math.rad(-10.0); st, s2 = aim_tick(4, tx, ty, tz)
local ok4 = s2.plane_side == -1 and math.abs(s2.plane_err_deg + 90.0) < 0.05
stubs.sim.roll = math.rad(10.0); st, s2 = aim_tick(4, tx, ty, tz)
local ok5 = s2.plane_side == 1 and math.abs(s2.plane_err_deg - 70.0) < 0.05
check("12 o'clock chosen at 80 deg round the clock", ok1)
check("still 12 at 100 deg: inside the hold band", ok2)
check("6 at 110 deg: past the band, 70 deg to roll back", ok3)
check("still 6 at the wingtip line", ok4)
check("12 again at 70 deg: past the band the other way", ok5)
stubs.sim.roll = math.rad(-10.0); st, s2 = aim_tick(4, tx, ty, tz)
stubs.sim.roll = math.rad(-30.0); st, s2 = aim_tick(4, tx, ty, tz)       -- now 6
-- 85 deg round: inside the band, so the choice held from before is 6; a
-- fresh one is 12.
stubs.sim.roll = math.rad(-5.0); st, s2 = aim_tick(3, tx, ty, tz)        -- new mode
check("a new chase chooses afresh (85 deg round: 12, not the 6 held before)",
      s2.plane_side == 1 and math.abs(s2.plane_err_deg - 85.0) < 0.05,
      string.format("side %d, err %.2f", s2.plane_side, s2.plane_err_deg))

-- The hold band is a live setting: at 30 deg, 110 deg round the clock is
-- still inside it and 12 holds; 125 is past it.
stubs.feed(pack_gain(GAIN_IDX.plane_side_hold_deg, 30.0))
stubs.sim.roll = 0.0; st, s2 = aim_tick(4, tx, ty, tz)                -- new mode: 12
stubs.sim.roll = math.rad(-30.0); st, s2 = aim_tick(4, tx, ty, tz)
local okh1 = s2.plane_side == 1 and math.abs(s2.plane_err_deg - 110.0) < 0.05
stubs.sim.roll = math.rad(-45.0); st, s2 = aim_tick(4, tx, ty, tz)
local okh2 = s2.plane_side == -1 and math.abs(s2.plane_err_deg + 55.0) < 0.05
check("the hold band reaches the FC over SET_GAIN (30 deg: 12 holds at 110)", okh1)
check("and 125 deg round is past it: 6", okh2)
stubs.feed(pack_gain(GAIN_IDX.plane_side_hold_deg, HOLD))

-- THE FEEDFORWARD is the balloon's own drift round the clock, filtered: the
-- clock angle's rate plus the aircraft's roll rate.  A balloon that moves
-- round the clock while the wings hold still drifts; a balloon fixed in the
-- world while the aircraft rolls does not, though its clock angle moves just
-- as fast.  Nose 80 deg down, so roll-to-plane has the whole command.
local DRIFT, FFG, FF_RATE, FF_TICKS = 30.0, 0.5, 50.0, 25
stubs.feed(pack_gain(GAIN_IDX.plane_ff_gain, FFG)
           .. pack_gain(GAIN_IDX.plane_ff_lpf_rate, FF_RATE))
fly(-80.0)
st, s2 = aim_tick(3, world_at(80.0, 20.0))                            -- new mode
for i = 1, FF_TICKS do
  st, s2 = aim_tick(3, world_at(80.0, 20.0 + DRIFT * i * 0.020))
end
check("a balloon drifting round the clock: the filter settles on its rate",
      s2.plane_path_share > 0.0 and math.abs(s2.plane_drift_dps - DRIFT) < 1.0,
      string.format("path share %.3f, drift %.2f dps", s2.plane_path_share,
                    s2.plane_drift_dps))
check("the feedforward is its gain times the drift",
      math.abs(s2.plane_ff_dps - FFG * s2.plane_drift_dps) < 1e-4,
      string.format("ff %.3f, drift %.3f", s2.plane_ff_dps, s2.plane_drift_dps))
check("and the command carries roll-to-plane's share of it",
      math.abs(st.omega_x_presat_dps - blended(st, s2)) < 1e-2,
      string.format("command %.3f vs %.3f", st.omega_x_presat_dps, blended(st, s2)))
local ROLL_RATE = math.rad(40.0)
local fx, fy, fz = world_at(80.0, 20.0)
st, s2 = aim_tick(4, fx, fy, fz)                                      -- new mode
local clock0 = s2.plane_err_deg
stubs.sim.p = ROLL_RATE
for _ = 1, FF_TICKS do
  stubs.sim.roll = stubs.sim.roll + ROLL_RATE * 0.020
  st, s2 = aim_tick(4, fx, fy, fz)
end
local moved = s2.plane_err_deg - clock0
check("a balloon fixed in the world while the aircraft rolls: no drift",
      math.abs(s2.plane_drift_dps) < 1.0
      and math.abs(moved + math.deg(ROLL_RATE) * FF_TICKS * 0.020) < 0.1,
      string.format("drift %.2f dps, clock moved %.2f deg", s2.plane_drift_dps, moved))
stubs.sim.roll, stubs.sim.p = 0.0, 0.0; fly(0.0)
for i = 1, 3 do st, s2 = aim_tick(3, world_at(20.0, 90.0 + 10.0 * i)) end
check("with no roll-to-plane share the feedforward is held at zero",
      s2.plane_path_share == 0.0 and s2.plane_drift_dps == 0.0 and s2.plane_ff_dps == 0.0,
      string.format("path share %.3f, drift %.3f, ff %.3f", s2.plane_path_share,
                    s2.plane_drift_dps, s2.plane_ff_dps))
stubs.feed(pack_gain(GAIN_IDX.plane_ff_gain, 0.0))

-- THE DERIVATIVE TERM is the clock error's own rate, filtered, times its gain.
-- Unlike the feedforward it keeps the roll's share: a balloon fixed in the
-- world while the aircraft rolls toward it moves round the clock at minus the
-- roll rate, and the term takes rate off the roll.  Nose 80 deg down, so
-- roll-to-plane has the whole command.
do
local KD, KD_RATE, FLIP_TICKS = 0.3, 50.0, 30
stubs.feed(pack_gain(GAIN_IDX.plane_kd, KD) .. pack_gain(GAIN_IDX.plane_kd_rate, KD_RATE))
fly(-80.0)
st, s2 = aim_tick(3, world_at(80.0, 20.0))                            -- new mode
for i = 1, FF_TICKS do
  st, s2 = aim_tick(3, world_at(80.0, 20.0 + DRIFT * i * 0.020))
end
check("a balloon drifting round the clock: the error's rate settles on the drift",
      s2.plane_path_share > 0.0 and math.abs(s2.plane_err_rate_dps - DRIFT) < 1.0,
      string.format("path share %.3f, error rate %.2f dps", s2.plane_path_share,
                    s2.plane_err_rate_dps))
check("the derivative term is its gain times that rate",
      math.abs(s2.plane_d_dps - KD * s2.plane_err_rate_dps) < 1e-4,
      string.format("d %.3f, error rate %.3f", s2.plane_d_dps, s2.plane_err_rate_dps))
check("and the command carries roll-to-plane's share of it",
      math.abs(st.omega_x_presat_dps - blended(st, s2)) < 1e-2,
      string.format("command %.3f vs %.3f", st.omega_x_presat_dps, blended(st, s2)))
local dx, dy, dz = world_at(80.0, 30.0)
st, s2 = aim_tick(4, dx, dy, dz)                                      -- new mode
stubs.sim.p = ROLL_RATE
for _ = 1, FF_TICKS do
  stubs.sim.roll = stubs.sim.roll + ROLL_RATE * 0.020
  st, s2 = aim_tick(4, dx, dy, dz)
end
check("a balloon fixed in the world while the aircraft rolls toward it: the "
      .. "rate is minus the roll rate, and the term rolls against the roll",
      math.abs(s2.plane_err_rate_dps + math.deg(ROLL_RATE)) < 1.0
      and s2.plane_err_deg > 0.0 and s2.plane_d_dps < 0.0,
      string.format("error %.2f deg, error rate %.2f dps, d %.2f dps", s2.plane_err_deg,
                    s2.plane_err_rate_dps, s2.plane_d_dps))
-- The rate is the clock's, so a 12 <-> 6 change, where the error steps by 180,
-- does not reach it.  Rolling left carries the balloon from 85 deg round the
-- clock past the hold band at 90 + HOLD.
local hx, hy, hz = world_at(80.0, 85.0)
st, s2 = aim_tick(3, hx, hy, hz)                                      -- new mode: 12
stubs.sim.p = -ROLL_RATE
for _ = 1, FLIP_TICKS do
  stubs.sim.roll = stubs.sim.roll - ROLL_RATE * 0.020
  st, s2 = aim_tick(3, hx, hy, hz)
end
local clock_end = 85.0 + math.deg(ROLL_RATE) * FLIP_TICKS * 0.020
check("a 12 <-> 6 change steps the error by 180 and leaves the rate the roll's",
      s2.plane_side == -1 and math.abs(s2.plane_err_deg - (clock_end - 180.0)) < 0.1
      and math.abs(s2.plane_err_rate_dps - math.deg(ROLL_RATE)) < 1.0,
      string.format("side %d, error %.2f deg, error rate %.2f dps", s2.plane_side,
                    s2.plane_err_deg, s2.plane_err_rate_dps))
stubs.sim.roll, stubs.sim.p = 0.0, 0.0; fly(0.0)
for i = 1, 3 do st, s2 = aim_tick(3, world_at(20.0, 90.0 + 10.0 * i)) end
check("with no roll-to-plane share the derivative term is held at zero",
      s2.plane_path_share == 0.0 and s2.plane_err_rate_dps == 0.0 and s2.plane_d_dps == 0.0,
      string.format("path share %.3f, error rate %.3f, d %.3f", s2.plane_path_share,
                    s2.plane_err_rate_dps, s2.plane_d_dps))
stubs.feed(pack_gain(GAIN_IDX.plane_kd, CONST.PLANE_KD)
           .. pack_gain(GAIN_IDX.plane_kd_rate, CONST.PLANE_KD_RATE))
end

-- THE INTEGRATOR, nose 85 deg down so roll-to-plane has the whole command; the
-- wings held, so the error holds.  In deg/s of roll, Ki x error x seconds.
local KI, WINDUP, ACRO_ROLL = 2.0, 0.2, 180.0
local I_TICKS = 10
stubs.feed(pack_gain(GAIN_IDX.plane_ki, KI) .. pack_gain(GAIN_IDX.plane_windup, WINDUP))
fly(-85.0)
local ix, iy, iz = world_at(75.0, 10.0)
st, s2 = aim_tick(3, ix, iy, iz)                                      -- new mode
local e10 = s2.plane_err_deg
check("a new chase starts the integrator from zero: one tick's worth",
      math.abs(s2.plane_i_dps - KI * e10 * 0.020) < 1e-4,
      string.format("integrator %.4f, expected %.4f", s2.plane_i_dps, KI * e10 * 0.020))
for _ = 2, I_TICKS do st, s2 = aim_tick(3, ix, iy, iz) end
check("it grows at Ki x error, and the command carries it",
      math.abs(s2.plane_i_dps - KI * e10 * 0.020 * I_TICKS) < 1e-3
      and s2.plane_i_frozen == 0.0
      and math.abs(st.omega_x_presat_dps - blended(st, s2)) < 1e-2,
      string.format("integrator %.3f, expected %.3f, command %.3f vs %.3f",
                    s2.plane_i_dps, KI * e10 * 0.020 * I_TICKS,
                    st.omega_x_presat_dps, blended(st, s2)))
for _ = 1, 200 do st, s2 = aim_tick(3, ix, iy, iz) end
check("and stops at its windup share of the ACRO roll rate",
      math.abs(s2.plane_i_dps - WINDUP * ACRO_ROLL) < 1e-3,
      string.format("integrator %.3f, ceiling %.1f", s2.plane_i_dps, WINDUP * ACRO_ROLL))
local fx2, fy2, fz2 = world_at(75.0, 45.0)
st, s2 = aim_tick(4, fx2, fy2, fz2)                                   -- new mode
check("held while the command is already at the ACRO roll rate its way",
      st.omega_x_presat_dps >= ACRO_ROLL and s2.plane_i_frozen == 1.0
      and s2.plane_i_dps == 0.0,
      string.format("command %.1f, frozen %d, integrator %.3f", st.omega_x_presat_dps,
                    s2.plane_i_frozen, s2.plane_i_dps))
st, s2 = aim_tick(3, ix, iy, iz)
stubs.feed(pack_gain(GAIN_IDX.plane_ki, 0.0))
st, s2 = aim_tick(3, ix, iy, iz)
check("Ki 0 removes it", s2.plane_i_dps == 0.0, string.format("integrator %.3f", s2.plane_i_dps))
stubs.feed(pack_gain(GAIN_IDX.plane_ki, KI))
st, s2 = aim_tick(3, ix, iy, iz)
fly(0.0)
st, s2 = aim_tick(3, world_at(20.0, 90.0))
check("no roll-to-plane share clears it",
      s2.plane_path_share == 0.0 and s2.plane_i_dps == 0.0,
      string.format("path share %.3f, integrator %.3f", s2.plane_path_share, s2.plane_i_dps))
stubs.feed(pack_gain(GAIN_IDX.plane_ki, 0.0))
stubs.sim.roll = 0.0; fly(0.0)

-- The flight that rolled over: `s` pressed at 74 deg nose-up, 14 m/s, wings
-- 7.5 deg right, the level balloon on the heading 72 deg below the nose, under
-- the belly.  Banking toward it swings it further out and, at that pitch, on
-- over the top.  Flown closed loop on roll alone, pitch held: roll-to-plane
-- has all of it, takes the balloon to 6 o'clock by levelling the wings, and the
-- pitch law pushes.
local CLIMB = math.rad(74.0)
local function pack_straight(seq)
  local body = string.pack('<I2BBi2i2i2i2i2i2i2i2BBi2i2i2I4I2',
    seq, 0x0A, 1,                               -- tracking + straight_level
    0, 0, math.floor(85.0*100+0.5), 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 12)
  local head = string.pack('<I2BB', proto.MAGIC, proto.MSG_GUIDE, #body)
  local hb = head .. body
  return hb .. string.pack('<I2', proto.crc16(hb, 1, #hb))
end
local function wrap_pi(a) return (a + math.pi) % (2.0 * math.pi) - math.pi end
local function fly_log_case(seq0)
  stubs.sim.roll, stubs.sim.pitch, stubs.sim.yaw, stubs.sim.p = math.rad(7.5), CLIMB, 0.0, 0.0
  stubs.sim.vn, stubs.sim.ve, stubs.sim.vd = 14.0 * math.cos(CLIMB), 0.0, -14.0 * math.sin(CLIMB)
  stubs.sim.eas = 14.0
  local largest = 0.0
  for i = 1, 150 do                             -- 3 s
    stubs.feed(pack_straight(seq0 + i))
    stubs.advance(20); update()
    local c = stubs.log.rate_cmds[#stubs.log.rate_cmds]
    stubs.sim.p = math.rad(c[2])
    stubs.sim.roll = wrap_pi(stubs.sim.roll + stubs.sim.p * 0.020)
    largest = math.max(largest, math.abs(math.deg(stubs.sim.roll)))
  end
  return largest
end
local max_roll = fly_log_case(43500)
st, s2 = last_state(), last_state2()
check("the log's case: roll-to-plane has the whole command and chooses 6",
      s2.plane_path_share == 1.0 and s2.plane_side == -1,
      string.format("path share %.2f, side %d, %.1f deg off", s2.plane_path_share,
                    s2.plane_side, s2.plane_off_nose_deg))
check("it levels the wings instead of rolling over", max_roll <= 7.6
      and math.abs(math.deg(stubs.sim.roll)) < 1.0,
      string.format("largest roll %.1f deg, roll now %.2f deg", max_roll,
                    math.deg(stubs.sim.roll)))
check("and the pitch law pushes toward the balloon under the belly",
      st.omega_y_presat_dps < 0.0, string.format("pitch command %.1f dps", st.omega_y_presat_dps))
-- The same flight at the stored defaults for the blend and roll-to-plane's
-- gain, the path the nose's 74 deg.  On a path under Steep Full
-- roll-to-plane's share shrinks and the mix holds a bank short of level, which
-- grows as the share falls (TODO.md section 7).
stubs.feed(pack_gain(GAIN_IDX.plane_steep_start_deg, CONST.PLANE_STEEP_START_DEG)
           .. pack_gain(GAIN_IDX.plane_steep_full_deg, CONST.PLANE_STEEP_FULL_DEG)
           .. pack_gain(GAIN_IDX.plane_gain, CONST.PLANE_GAIN))
max_roll = fly_log_case(43700)
st, s2 = last_state(), last_state2()
check("at the stored defaults the log's case levels the wings too",
      s2.plane_path_share == 1.0 and s2.plane_side == -1 and max_roll <= 7.6
      and math.abs(math.deg(stubs.sim.roll)) < 1.0,
      string.format("path share %.2f, largest roll %.1f deg, roll now %.2f deg",
                    s2.plane_path_share, max_roll, math.deg(stubs.sim.roll)))
stubs.sim.roll, stubs.sim.pitch, stubs.sim.p = 0.0, 0.0, 0.0
stubs.sim.vn, stubs.sim.ve, stubs.sim.vd, stubs.sim.eas = sim2.vn, sim2.ve, sim2.vd, sim2.eas
stubs.feed(pack_gain(GAIN_IDX.plane_gain, CONST.PLANE_GAIN))

print("\n12. A FRAME FROM ANOTHER PROTOCOL BUILD IS REJECTED, NOT DECODED")
-- In a function of its own: the main chunk is at Lua's 200-local limit.
;(function()
  -- A GUIDE frame one byte longer than this build's, with a valid checksum: what
  -- a companion built against another protocol layout sends.  Decoded, every
  -- field would be read at the wrong offset; the script must step over it, keep
  -- its last GUIDE, and say so in STATE (flags bit 4) for the companion to halt on.
  local seq_before = last_state().seq
  local long_body = string.rep("\0", proto.PAYLOAD_SIZE[proto.MSG_GUIDE] + 1)
  local long_head = string.pack('<I2BB', proto.MAGIC, proto.MSG_GUIDE, #long_body)
  local long_hb = long_head .. long_body
  check("STATE does not flag a rejection before one happens",
        (last_state().flags & 0x10) == 0)
  stubs.feed(long_hb .. string.pack('<I2', proto.crc16(long_hb, 1, #long_hb)))
  stubs.advance(20); update()
  check("a wrong-size GUIDE is not decoded", last_state().seq == seq_before,
        string.format("seq %d -> %d", seq_before, last_state().seq))
  check("and STATE flags the rejection (bit 4)", (last_state().flags & 0x10) ~= 0)
  local rejects = 0
  for _, t in ipairs(stubs.log.texts) do
    if t:find("different protocol builds", 1, true) then rejects = rejects + 1 end
  end
  stubs.feed(long_hb .. string.pack('<I2', proto.crc16(long_hb, 1, #long_hb)))
  stubs.advance(20); update()
  local rejects2 = 0
  for _, t in ipairs(stubs.log.texts) do
    if t:find("different protocol builds", 1, true) then rejects2 = rejects2 + 1 end
  end
  check("and it is reported once on the FC console", rejects == 1 and rejects2 == 1,
        string.format("%d, then %d after a second", rejects, rejects2))
end)()

print("\n13. AN INPUT LOST MID-CHASE RELEASES CONTROL AND LATCHES A FAULT")
;(function()
  local seq = 13000
  local function tick(flags)
    seq = seq + 1
    stubs.feed(pack_guide(seq, flags, 4, 0.2, 0.0, 55.0)); stubs.advance(20); update()
  end
  for _ = 1, 3 do tick(0x07) end
  check("flying before the fault", stubs.sim.nav_owned and last_state().gate == 0,
        "gate " .. last_state().gate)
  local saved_q = _G.ahrs.get_quaternion
  _G.ahrs.get_quaternion = function() return nil end
  tick(0x07)
  check("no attitude while flying releases control at once",
        not stubs.sim.nav_owned and stubs.log.nav_enable[#stubs.log.nav_enable] == 255)
  check("and latches the gate as the fault (bits 5-7)",
        (last_state().flags >> 5) == 3, "flags " .. last_state().flags)
  _G.ahrs.get_quaternion = saved_q
  tick(0x07)
  check("the fault holds with the attitude back", not stubs.sim.nav_owned
        and (last_state().flags >> 5) == 3)
  tick(0x05)
  tick(0x07)
  check("cleared by a stand-down, the chase resumes", stubs.sim.nav_owned
        and (last_state().flags >> 5) == 0)
  -- Waiting for the first estimate after a fresh chase is not a fault: from
  -- straight-and-level (flying, no detection) straight into the balloon chaser
  -- with no estimate yet, the script lets go and waits.
  tick(0x05)
  tick(0x0A)
  check("straight-and-level flies with no estimate", stubs.sim.nav_owned)
  tick(0x02)
  check("a fresh chase with no estimate yet releases, without a fault",
        not stubs.sim.nav_owned and last_state().gate == 4
        and (last_state().flags >> 5) == 0,
        string.format("gate %d, flags %d", last_state().gate, last_state().flags))
  tick(0x05)
end)()

print("\n14. A NEW SESSION STARTS WITH THE LATCHES CLEAR")
-- The script outlives the program.  A fault latched as one session ends must
-- not be read by the next as its own: the first new GUIDE after a stale gap
-- clears it, even with tracking on.
;(function()
  local seq = 16000
  local function tick(flags)
    seq = seq + 1
    stubs.feed(pack_guide(seq, flags, 4, 0.2, 0.0, 55.0)); stubs.advance(20); update()
  end
  for _ = 1, 3 do tick(0x07) end
  local saved_q = _G.ahrs.get_quaternion
  _G.ahrs.get_quaternion = function() return nil end
  tick(0x07)
  _G.ahrs.get_quaternion = saved_q
  check("a fault is latched as the session ends", (last_state().flags >> 5) == 3,
        "flags " .. last_state().flags)
  local stale = math.floor(require("constants").GUIDE_STALE_MS / 20) + 3
  for _ = 1, stale do stubs.advance(20); update() end
  check("  and stays latched while nothing arrives", (last_state().flags >> 5) == 3)
  tick(0x07)
  check("the next session's first GUIDE clears it, and the chase is flown",
        (last_state().flags >> 5) == 0 and stubs.sim.nav_owned,
        string.format("flags %d, owned %s", last_state().flags,
                      tostring(stubs.sim.nav_owned)))
end)()

print("\n15. THE RULES OF A CHASE, FLOWN CLOSED-LOOP")
-- CLAUDE.md "The rules of a chase".  The whole FC script, with its
-- default gains, steers a simple aircraft through `p` chases: its nose on the
-- flight path at a constant speed, the roll rate following the commanded body
-- roll rate about the path and the load following what the commanded body
-- pitch rate needs, both after a lag and within the ACRO rates and the load
-- the wing can make; the path bends with the load along the canopy plus
-- gravity.  No sideslip, rudder or trim error.
--   Rules 1 and 2: a balloon below the belly, its turn short of straight
-- down, or above the canopy, its turn short of straight up, is chased without
-- a roll.  "Below" and "above" are the world's: the balloon in the vertical
-- plane through the path, within RULE_PLANE_DEG of it round the path.  The
-- turn goes through straight down or up when its great circle passes within
-- RULE_POLE_DEG of it before the balloon; chases within RULE_POLE_MARGIN_DEG
-- of that boundary are left out, so rounding does not judge them.
--   "A roll": the bank against the horizon moving more than ROLL_RULE_DEG
-- outside the range between the starting bank and level, read while the path
-- is more than STEEP_DEG from vertical, or the chase ending more than
-- ROLL_RULE_DEG from upright.  A roll toward where the chase ends is a roll
-- like any other.  Where the balloon ends is not judged.
--   The exception: a balloon more than 90 deg off the path may be chased
-- with a roll, so balloons behind are not judged.
;(function()
  local C3 = require("constants")
  local sin, cos, sqrt, rad, deg = math.sin, math.cos, math.sqrt, math.rad, math.deg
  local GR = 9.80665
  -- the rules
  local ROLL_RULE_DEG = 20
  local RULE_PLANE_DEG = 15
  local RULE_POLE_DEG = 10
  local RULE_POLE_MARGIN_DEG = 5
  local RULE_BANKS_DEG = { 0, 30, -30, 60, -60 }
  local RULE_CLIMBS_DEG = { -60, 0, 60 }
  local RULE_OFFS_DEG = { 5, 15, 30, 45, 60, 75, 89 }
  local RULE_CLOCKS_DEG = { -RULE_PLANE_DEG, 0, RULE_PLANE_DEG }
  local STEEP_DEG = 20      -- the bank is judged further than this from vertical
  local CHASE_S = 12
  -- the aircraft
  local SPEED = 18.0
  local DT_MS = 20
  local ROLL_LAG_S, LOAD_LAG_S = 0.12, 0.25
  local LOAD_MIN_G, LOAD_MAX_G = -1.5, 2.5
  local ACRO_ROLL = stubs.param_values.ACRO_ROLL_RATE

  local function dot(a, b) return a[1] * b[1] + a[2] * b[2] + a[3] * b[3] end
  local function cross(a, b)
    return { a[2] * b[3] - a[3] * b[2], a[3] * b[1] - a[1] * b[3], a[1] * b[2] - a[2] * b[1] }
  end
  local function unit(a) local m = sqrt(dot(a, a)); return { a[1] / m, a[2] / m, a[3] / m } end
  local function across(a, v)
    local k = dot(a, v); return { a[1] - k * v[1], a[2] - k * v[2], a[3] - k * v[3] }
  end
  local function angle(a, b) return deg(math.acos(math.max(-1, math.min(1, dot(a, b))))) end
  local function rot(c, v, th)    -- c turned about v by th
    local vc = cross(v, c)
    return { c[1] * cos(th) + vc[1] * sin(th), c[2] * cos(th) + vc[2] * sin(th),
             c[3] * cos(th) + vc[3] * sin(th) }
  end
  local UP = { 0.0, 0.0, -1.0 }

  local function pack_oracle(seq, b)
    local body = string.pack('<I2BBi2i2i2i2i2i2i2i2BBi2i2i2I4I2',
      seq, 0x06, 4, 0, 0, math.floor(55.0 * 100 + 0.5), 0, 0, 0, 0, 0, 1, 0,
      math.floor(b[1] * 10000 + 0.5), math.floor(b[2] * 10000 + 0.5),
      math.floor(b[3] * 10000 + 0.5), 0, 12)
    local hb = string.pack('<I2BB', proto.MAGIC, proto.MSG_GUIDE, #body) .. body
    return hb .. string.pack('<I2', proto.crc16(hb, 1, #hb))
  end

  -- every gain back to its default: earlier sections changed some
  local frames = {}
  for i, name in pairs(proto.GAIN_NAMES) do
    local v = C3[string.upper(name)]
    if type(v) == "number" then frames[#frames + 1] = pack_gain(i, v) end
  end
  for k = 1, #frames, 8 do
    stubs.feed(table.concat(frames, "", k, math.min(k + 7, #frames)))
    stubs.advance(DT_MS); update()
  end

  -- GUIDE's seq is 16 bits and only has to differ from the last
  local seq = 0
  local function next_seq() seq = seq % 60000 + 1; return seq end
  -- the bank against the horizon, signed right-wing-down positive, and the
  -- path's distance from vertical
  local function bank_of(v, c)
    local u = across(UP, v)
    local um = sqrt(dot(u, u))
    if um < 1e-9 then return 0.0, 0.0 end
    u = { u[1] / um, u[2] / um, u[3] / um }
    local b = angle(c, u)
    if dot(cross(u, c), v) < 0.0 then b = -b end
    return b, um
  end

  -- one chase from a start (climb, bank) toward the world unit vector bal
  local function fly(climb_d, bank_d, bal, secs)
    local cl = rad(climb_d)
    local v = { cos(cl), 0.0, -sin(cl) }
    local c = rot(unit(across(UP, v)), v, rad(bank_d))
    local p_act, n_act = 0.0, 1.0
    local r = { bank_lo = bank_d, bank_hi = bank_d }
    -- the chase starts from rest: a GUIDE with tracking off ends whatever came before
    stubs.feed(pack_guide(next_seq(), 0x01, 4, 0.0, 0.0, 55.0)); stubs.advance(DT_MS); update()
    for _ = 1, math.floor(secs * 1000 / DT_MS) do
      local z = { -c[1], -c[2], -c[3] }
      local y = cross(z, v)
      stubs.sim.pitch = math.asin(math.max(-1, math.min(1, -v[3])))
      stubs.sim.roll = math.atan(y[3], z[3])
      stubs.sim.yaw = math.atan(v[2], v[1])
      stubs.sim.vn, stubs.sim.ve, stubs.sim.vd = v[1] * SPEED, v[2] * SPEED, v[3] * SPEED
      stubs.sim.eas = SPEED
      stubs.sim.p, stubs.sim.q, stubs.sim.r = rad(p_act), 0.0, 0.0
      stubs.feed(pack_oracle(next_seq(), bal)); stubs.advance(DT_MS); update()
      local cmd = stubs.log.rate_cmds[#stubs.log.rate_cmds]
      local ox, oy = 0.0, 0.0
      if stubs.sim.nav_owned and cmd then ox, oy = cmd[2], cmd[3] end
      local bank, um = bank_of(v, c)
      if um > sin(rad(STEEP_DEG)) then
        r.bank_lo, r.bank_hi = math.min(r.bank_lo, bank), math.max(r.bank_hi, bank)
      end
      r.end_bank = (um > sin(rad(STEEP_DEG))) and bank or nil
      local dt = DT_MS / 1000
      p_act = p_act + (ox - p_act) * dt / ROLL_LAG_S
      p_act = math.max(-ACRO_ROLL, math.min(ACRO_ROLL, p_act))
      c = rot(c, v, rad(p_act) * dt)
      local n_want = (rad(oy) * SPEED + GR * z[3]) / GR
      n_act = n_act + (n_want - n_act) * dt / LOAD_LAG_S
      n_act = math.max(LOAD_MIN_G, math.min(LOAD_MAX_G, n_act))
      local a = across({ GR * n_act * c[1], GR * n_act * c[2], GR * n_act * c[3] + GR }, v)
      v = unit({ v[1] + a[1] * dt / SPEED, v[2] + a[2] * dt / SPEED, v[3] + a[3] * dt / SPEED })
      c = unit(across(c, v))
    end
    r.eps_end = angle(v, bal)
    r.roll_out = math.max(math.min(0.0, bank_d) - r.bank_lo, r.bank_hi - math.max(0.0, bank_d))
    r.end_up = r.end_bank and math.abs(r.end_bank) or 0.0
    return r
  end

  -- the balloons rules 1 and 2 cover from a start (climb): in the world's
  -- vertical plane through the path, below (belly) or above, ahead of 90 deg
  local function rule_chases(belly, climb_d)
    local list = {}
    local cl = rad(climb_d)
    local v = { cos(cl), 0.0, -sin(cl) }
    local cup = unit(across(UP, v))
    local y = cross({ -cup[1], -cup[2], -cup[3] }, v)
    local pole = belly and { 0.0, 0.0, 1.0 } or { 0.0, 0.0, -1.0 }
    local m, clear = rad(RULE_POLE_MARGIN_DEG), rad(RULE_POLE_DEG)
    for _, off in ipairs(RULE_OFFS_DEG) do
      for _, dclk in ipairs(RULE_CLOCKS_DEG) do
        local clk = rad((belly and 180 or 0) + dclk)
        local u = { cos(clk) * cup[1] + sin(clk) * y[1], cos(clk) * cup[2] + sin(clk) * y[2],
                    cos(clk) * cup[3] + sin(clk) * y[3] }
        local e = rad(off)
        local pv, pu = dot(pole, v), dot(pole, u)
        local s_near = math.atan(pu, pv)
        local d_near = math.acos(math.min(1.0, sqrt(pv * pv + pu * pu)))
        if not (s_near > -m and s_near < e + m and d_near < clear + m) then
          list[#list + 1] = { off = off, dclk = dclk,
            b = { cos(e) * v[1] + sin(e) * u[1], cos(e) * v[2] + sin(e) * u[2],
                  cos(e) * v[3] + sin(e) * u[3] } }
        end
      end
    end
    return list
  end

  for _, belly in ipairs({ true, false }) do
    local bad, n_ch, worst = {}, 0, 0.0
    for _, climb in ipairs(RULE_CLIMBS_DEG) do
      for _, bank in ipairs(RULE_BANKS_DEG) do
        for _, ch in ipairs(rule_chases(belly, climb)) do
          n_ch = n_ch + 1
          local rr = fly(climb, bank, ch.b, CHASE_S)
          worst = math.max(worst, rr.roll_out)
          if rr.roll_out > ROLL_RULE_DEG or rr.end_up > ROLL_RULE_DEG then
            bad[#bad + 1] = string.format("climb %d bank %d off %d clock %+d: rolled %.0f, ends %.0f from level",
                                          climb, bank, ch.off, ch.dclk, rr.roll_out, rr.end_up)
          end
        end
      end
    end
    check(string.format("rule %d: balloon %s (%d chases, wings level and banked up to 60): no roll past %d deg",
                        belly and 1 or 2,
                        belly and "below the belly, short of straight down"
                        or "above the canopy, short of straight up", n_ch, ROLL_RULE_DEG),
          #bad == 0, string.format("%d roll, largest %.0f; %s", #bad, worst,
                                   table.concat(bad, " | ", 1, math.min(#bad, 12))))
  end
  do
    local el, az = rad(C3.POINT_LAT_DEG), rad(C3.POINT_LON_DEG)
    local b = { cos(el) * cos(az), cos(el) * sin(az), -sin(el) }
    local rr = fly(0, 0, b, CHASE_S)
    check(string.format("the default p balloon (%g, %g) from level: no roll past %d deg",
                        C3.POINT_LAT_DEG, C3.POINT_LON_DEG, ROLL_RULE_DEG),
          rr.roll_out <= ROLL_RULE_DEG and rr.end_up <= ROLL_RULE_DEG,
          string.format("rolled %.0f, ends %.0f from level, %.1f deg off the balloon at the end",
                        rr.roll_out, rr.end_up, rr.eps_end))
  end
end)()

print("\n16. THE CURVE FLYER, FLOWN CLOSED-LOOP BY EACH LAW")
-- CLAUDE.md "Lua steering offload": with the Curve Flyer on, a fresh chase
-- holds its rates for the Curve Planner's PLAN, the selected law flies the
-- turn in its plan mode - its own roll, pitch and bank limit, the bank the
-- PLAN's at the turn's progress - until the path points within
-- CURVE_TURN_DONE_DEG of the balloon, however long that takes, and then flies
-- the chase without the PLAN; with no PLAN in time it flies without one
-- throughout.  Every check runs under each law.
--   The PLANs are the Curve Planner's (Companion/curve_planner.py:
-- Chase(0, 0, b, speed_mps=18, law=L) and plan(), the times from check()),
-- each in its law's measure of the bank: for the default p balloon, which no
-- rule binds, for BAL_R1, 60 deg off the path and 10 deg round from straight
-- below, which rule 1 binds, for BAL_G, 30 deg to the right and level, and
-- for the two run-ins below.
--   Two aircraft fly them, their noses on the path, the roll rate and the
-- path's turn following their commands after a lag, gravity bending the path,
-- the accelerometer reading the specific force.  Section 15's holds its speed
-- and lags the load; the planner's own lags the pitch rate and keeps the
-- planner's SITL speed fit from the moment the PLAN is taken.  On the
-- planner's own aircraft every turn takes what the planner predicts, within
-- TURN_AGREE_S: the planner flies every candidate through a model of each
-- law's plan mode, and a law that drifts from that model flies turns the
-- planner never made.  On section 15's the law, closed loop, still completes
-- each turn.
;(function()
  local C3 = require("constants")
  local sin, cos, sqrt, rad, deg = math.sin, math.cos, math.sqrt, math.rad, math.deg
  local GR = 9.80665
  local SPEED0, DT_MS = 18.0, 20
  local ROLL_LAG_S, LOAD_LAG_S = 0.12, 0.25
  local LOAD_MIN_G, LOAD_MAX_G = -3.5, 3.5
  -- the planner's speed fit (curve_planner.py SPEED_GRAV_MPS2, THRUST_MPS2,
  -- DRAG_PER_V2)
  local SPEED_GRAV, THRUST, DRAG = 9.5, 31.0, 0.075
  local ACRO_ROLL = stubs.param_values.ACRO_ROLL_RATE
  local BANK_SLOP_DEG = 5     -- the roll lag's overshoot past the limit
  local TURN_AGREE_S = 0.15
  local CHASE_S = 8.0         -- how long a chase is flown, longer than any turn here
  local LAW_NAMES = { [0] = "the cascade", [1] = "Lift Vector Pointing" }
  -- run-in chases: the balloon's position from the start, metres north, east
  -- and down
  local BALLOON_RS = { 142.763, 82.424, 60.0 }   -- 20 deg below, 30 deg right: no rule binds
  local BALLOON_RA = { 149.282, 0.0, 40.0 }      -- 15 deg below, dead ahead: rule 1
  local PLANS = {
    [0] = {
      P  = { eps0 = 84.92, turn_s = 1.23, knots = { -31.37, -31.23, -24.26, -45.00, 9.37 } },
      R1 = { eps0 = 60.00, turn_s = 1.63, knots = { -11.89, -45.00, -45.00, -21.35, -3.09 } },
      G  = { eps0 = 30.00, turn_s = 1.15, knots = { 45.00, 45.00, 45.00, 45.00, 6.00 } },
      RS = { eps0 = 30.00, knots = { 45.00, 45.00, 45.00, 45.00, 45.00 }, climb = 0.00, dep = 40.88,
             lined_s = 1.24, dive_s = 4.72, done_s = 5.43, end_dep = 45.00 },
      RA = { eps0 = 0.00, knots = { 0.00, 0.00, 0.00, 0.00, 0.00 }, climb = 0.00, dep = 26.15,
             lined_s = 0.00, dive_s = 3.37, done_s = 3.90, end_dep = 28.28 },
    },
    [1] = {
      P  = { eps0 = 84.92, turn_s = 1.64, knots = { -32.40, -26.59, -35.64, -60.63, -106.45 } },
      R1 = { eps0 = 60.00, turn_s = 1.27, knots = { -20.41, -7.96, -17.34, -53.53, -58.33 } },
      G  = { eps0 = 30.00, turn_s = 1.11, knots = { 55.67, 64.98, 50.27, 48.98, 32.02 } },
      RS = { eps0 = 30.00, knots = { 86.61, 51.61, 56.72, 66.62, -35.43 }, climb = 0.00, dep = 39.40,
             lined_s = 1.20, dive_s = 4.61, done_s = 5.77, end_dep = 45.07 },
      RA = { eps0 = 0.00, knots = { 0.00, 0.00, 0.00, 0.00, 0.00 }, climb = 0.00, dep = 36.81,
             lined_s = 0.00, dive_s = 4.75, done_s = 5.95, end_dep = 45.15 },
    },
  }
  local BAL_R1 = { 0.5, 0.1504, 0.8529 }
  local BAL_G = { cos(rad(30.0)), sin(rad(30.0)), 0.0 }
  -- abeam, level: a PLAN that holds the wings level never turns the path to it
  local BAL_ABEAM = { 0.0, 1.0, 0.0 }
  local NO_CLOCK_S = 2.0 * CHASE_S
  local UP = { 0.0, 0.0, -1.0 }
  local function dot(a, b) return a[1] * b[1] + a[2] * b[2] + a[3] * b[3] end
  local function cross(a, b)
    return { a[2] * b[3] - a[3] * b[2], a[3] * b[1] - a[1] * b[3], a[1] * b[2] - a[2] * b[1] }
  end
  local function unit(a) local m = sqrt(dot(a, a)); return { a[1] / m, a[2] / m, a[3] / m } end
  local function across(a, v)
    local k = dot(a, v); return { a[1] - k * v[1], a[2] - k * v[2], a[3] - k * v[3] }
  end
  local function angle(a, b) return deg(math.acos(math.max(-1, math.min(1, dot(a, b))))) end
  local function rot(c, v, th)
    local vc = cross(v, c)
    return { c[1] * cos(th) + vc[1] * sin(th), c[2] * cos(th) + vc[2] * sin(th),
             c[3] * cos(th) + vc[3] * sin(th) }
  end
  local function pack_oracle(seq, b, mode)
    local body = string.pack('<I2BBi2i2i2i2i2i2i2i2BBi2i2i2I4I2',
      seq, 0x06, mode or 4, 0, 0, math.floor(55.0 * 100 + 0.5), 0, 0, 0, 0, 0, 1, 0,
      math.floor(b[1] * 10000 + 0.5), math.floor(b[2] * 10000 + 0.5),
      math.floor(b[3] * 10000 + 0.5), 0, 12)
    local hb = string.pack('<I2BB', proto.MAGIC, proto.MSG_GUIDE, #body) .. body
    return hb .. string.pack('<I2', proto.crc16(hb, 1, #hb))
  end
  -- A PLAN for steering law `law`; p.climb and p.dep are its run-in, NaN
  -- without one.  The layout is typed by hand and follows protocol.py's PLAN.
  local function pack_plan(id, law, p)
    local NAN = 0 / 0
    local args = { id, law, 0, p.eps0 }
    for k = 1, C3.CURVE_N_KNOTS do args[#args + 1] = p.knots[k] end
    args[#args + 1] = p.climb or NAN
    args[#args + 1] = p.dep or NAN
    local body = string.pack('<I2BBf' .. string.rep('f', C3.CURVE_N_KNOTS) .. 'ff',
                             table.unpack(args))
    local hb = string.pack('<I2BB', proto.MAGIC, proto.MSG_PLAN, #body) .. body
    return hb .. string.pack('<I2', proto.crc16(hb, 1, #hb))
  end
  local seq = 0
  local function next_seq() seq = seq % 60000 + 1; return seq end

  local el, az = rad(C3.POINT_LAT_DEG), rad(C3.POINT_LON_DEG)
  local BAL_P = { cos(el) * cos(az), cos(el) * sin(az), -sin(el) }

  -- One chase from level under the law selected.  plan_at: the tick a PLAN
  -- is fed (nil for none); plan: the PLAN, for steering law plan_law.  opts,
  -- all optional: own - the planner's own aircraft, not section 15's;
  -- nose_q, nose_r - functions of the tick giving a pitch and yaw rate of
  -- the NOSE alone, deg/s, added to the gyro while the path flies on as it
  -- would (a sideslip the path does not follow); pitch_share and nose_lag_s
  -- - the share of the commanded pitch rate the aircraft delivers and a lag
  -- before it follows, a slower aircraft than this one; balloon_pos - the
  -- balloon's position from the start, metres north, east and down, its
  -- direction worked out every tick as the aircraft moves (bal is then
  -- unused).  Returns the per-tick STATE2 states, what was commanded, and
  -- what was flown.
  local function chase(bal, ticks, plan_at, plan, plan_id, plan_law, opts)
    opts = opts or {}
    local v = { 1.0, 0.0, 0.0 }
    local c = { 0.0, 0.0, -1.0 }
    local speed = SPEED0
    local p_act, n_act, q_act = 0.0, 1.0, 0.0
    local q_lagged = 0.0               -- the commanded pitch rate after the nose's lag, rad/s
    local q_rad, r_rad = 0.0, 0.0      -- the path's turn toward the canopy and across the wings
    local r = { states = {}, prox_lo = 0.0, prox_hi = 0.0, waits_still = true,
                done_tick = nil, plan_echo = nil, unplanned_after = true, plan_mode = true,
                hold_tick = nil, hold_off = 0.0, hold_head = 0.0, hold_aimed = true,
                roll_cmd = {}, pitch_cmd = {}, cf_eps = {}, turn_err = 0.0, turn_seen = 0,
                dep = {}, climb = {}, head_err = {}, cf_prox_lo = 0.0, cf_prox_hi = 0.0 }
    local pos = { 0.0, 0.0, 0.0 }
    stubs.feed(pack_guide(next_seq(), 0x01, 4, 0.0, 0.0, 55.0)); stubs.advance(DT_MS); update()
    for t = 1, ticks do
      if opts.balloon_pos then
        bal = unit({ opts.balloon_pos[1] - pos[1], opts.balloon_pos[2] - pos[2], opts.balloon_pos[3] - pos[3] })
      end
      -- The balloon's angle below the horizon, the path's above it, and the
      -- path's compass direction off the balloon's, this tick.
      r.dep[t] = deg(math.asin(math.max(-1, math.min(1, bal[3]))))
      r.climb[t] = deg(math.asin(math.max(-1, math.min(1, -v[3]))))
      r.head_err[t] = deg(math.atan(v[1] * bal[2] - v[2] * bal[1], v[1] * bal[1] + v[2] * bal[2]))
      local z = { -c[1], -c[2], -c[3] }
      local y = cross(z, v)
      stubs.sim.pitch = math.asin(math.max(-1, math.min(1, -v[3])))
      stubs.sim.roll = math.atan(y[3], z[3])
      stubs.sim.yaw = math.atan(v[2], v[1])
      stubs.sim.vn, stubs.sim.ve, stubs.sim.vd = v[1] * speed, v[2] * speed, v[3] * speed
      stubs.sim.eas = speed
      stubs.sim.p = rad(p_act)
      stubs.sim.q = q_rad + rad(opts.nose_q and opts.nose_q(t) or 0.0)
      stubs.sim.r = r_rad + rad(opts.nose_r and opts.nose_r(t) or 0.0)
      -- the specific force in body axes: the path's acceleration across it
      -- less gravity
      local a = across({ GR * n_act * c[1], GR * n_act * c[2], GR * n_act * c[3] + GR }, v)
      local f = { a[1], a[2], a[3] - GR }
      stubs.sim.ax, stubs.sim.ay, stubs.sim.az = dot(f, v), dot(f, y), dot(f, z)
      local turned_up, turned_right = deg(q_rad), deg(r_rad)   -- over the step just flown
      local feed = pack_oracle(next_seq(), bal)
      if plan_at and t >= plan_at and r.plan_echo == nil then
        feed = feed .. pack_plan(plan_id, plan_law, plan)
      end
      stubs.feed(feed); stubs.advance(DT_MS); update()
      local s2, st = last_state2(), last_state()
      r.states[t] = s2.cf_state
      local cmd = stubs.log.rate_cmds[#stubs.log.rate_cmds]
      local ox, oy = 0.0, 0.0
      if stubs.sim.nav_owned and cmd then ox, oy = cmd[2], cmd[3] end
      r.roll_cmd[t], r.pitch_cmd[t], r.cf_eps[t] = ox, oy, s2.cf_eps_deg
      if t > 1 and s2.path_turn_up_dps == s2.path_turn_up_dps then
        -- The turn track.lua measured against the turn this aircraft flew over
        -- the same step, as one rate: the frame rolls during the step, so the
        -- split between the two parts is the frame's, the size is not.
        local meas = sqrt(s2.path_turn_up_dps ^ 2 + s2.path_turn_right_dps ^ 2)
        local flown = sqrt(turned_up ^ 2 + turned_right ^ 2)
        r.turn_err = math.max(r.turn_err, math.abs(meas - flown))
        r.turn_seen = r.turn_seen + 1
      end
      if s2.cf_state == C3.CF_STATE_WAIT then
        r.waits_still = r.waits_still and ox == 0.0 and oy == 0.0
      end
      if s2.cf_plan_id ~= 0 then r.plan_echo = s2.cf_plan_id end
      -- Under a PLAN the law says it flies one: cs_flags bit 5, and Lift
      -- Vector Pointing's lift side blank.
      if s2.cf_state == C3.CF_STATE_FLY then
        local planned = s2.cf_bank_ref_deg == s2.cf_bank_ref_deg and (st.cs_flags & 0x20) ~= 0
        if (st.cs_flags & 0x40) ~= 0 then
          planned = planned and s2.att_side ~= s2.att_side
        end
        r.plan_mode = r.plan_mode and planned
      end
      if s2.cf_state == C3.CF_STATE_HOLD then
        if not r.hold_tick then
          r.hold_tick = t
          r.hold = { s2.cf_hold_n, s2.cf_hold_e, s2.cf_hold_d }
          r.hold_vs_path = angle(v, r.hold)
          r.eps_done = angle(v, bal)
        end
        r.hold_end_off = angle(v, r.hold)
        r.hold_end_head = math.abs(deg(
          math.atan(v[1] * r.hold[2] - v[2] * r.hold[1], v[1] * r.hold[1] + v[2] * r.hold[2])))
        r.hold_off = math.max(r.hold_off, r.hold_end_off)
        r.hold_head = math.max(r.hold_head, r.hold_end_head)
        r.hold_aimed = r.hold_aimed and s2.cf_bank_ref_deg == s2.cf_bank_ref_deg
      end
      if s2.cf_state == C3.CF_STATE_DONE and not r.done_tick then
        r.done_tick = t
        r.eps_done = angle(v, bal)
      end
      if (s2.cf_state == C3.CF_STATE_DONE or s2.cf_state == C3.CF_STATE_HOLD) and not r.end_dep then
        r.end_dep = r.dep[t]
      end
      if s2.cf_state == C3.CF_STATE_RUNIN and not r.runin_tick then r.runin_tick = t end
      if s2.cf_state == C3.CF_STATE_DIVE and not r.dive_tick then r.dive_tick = t end
      if r.done_tick and s2.cf_bank_ref_deg == s2.cf_bank_ref_deg then
        r.unplanned_after = false      -- the Curve Flyer still handing the law a bank
      end
      -- the bank proxy: the wing tip's angle below the horizon
      local prox = deg(math.asin(math.max(-1, math.min(1, y[3]))))
      if s2.cf_state == C3.CF_STATE_FLY then
        r.prox_lo, r.prox_hi = math.min(r.prox_lo, prox), math.max(r.prox_hi, prox)
      end
      if s2.cf_state == C3.CF_STATE_FLY or s2.cf_state == C3.CF_STATE_RUNIN
         or s2.cf_state == C3.CF_STATE_DIVE then
        r.cf_prox_lo, r.cf_prox_hi = math.min(r.cf_prox_lo, prox), math.max(r.cf_prox_hi, prox)
      end
      local dt = DT_MS / 1000
      p_act = p_act + (ox - p_act) * dt / ROLL_LAG_S
      p_act = math.max(-ACRO_ROLL, math.min(ACRO_ROLL, p_act))
      c = rot(c, v, rad(p_act) * dt)
      local q_want = rad(oy) * (opts.pitch_share or 1.0)
      if opts.nose_lag_s then
        q_lagged = q_lagged + (q_want - q_lagged) * dt / opts.nose_lag_s
      else
        q_lagged = q_want
      end
      if opts.own then
        -- the planner's: the path turns toward the canopy at the lagged pitch
        -- rate, whatever load that takes
        q_act = q_act + (q_lagged - q_act) * dt / LOAD_LAG_S
        n_act = (q_act * speed - GR * c[3]) / GR
      else
        local n_want = (q_lagged * speed + GR * z[3]) / GR
        n_act = n_act + (n_want - n_act) * dt / LOAD_LAG_S
        n_act = math.max(LOAD_MIN_G, math.min(LOAD_MAX_G, n_act))
      end
      a = across({ GR * n_act * c[1], GR * n_act * c[2], GR * n_act * c[3] + GR }, v)
      q_rad, r_rad = dot(a, c) / speed, dot(a, cross(v, c)) / speed
      pos = { pos[1] + v[1] * speed * dt, pos[2] + v[2] * speed * dt, pos[3] + v[3] * speed * dt }
      v = unit({ v[1] + a[1] * dt / speed, v[2] + a[2] * dt / speed, v[3] + a[3] * dt / speed })
      c = unit(across(c, v))
      if opts.own and r.plan_echo then
        speed = speed + (SPEED_GRAV * v[3] + THRUST - DRAG * speed * speed) * dt
      end
    end
    return r
  end

  local function ticks_of(s) return math.floor(s * 1000 / DT_MS) end
  local function secs(ticks) return ticks * DT_MS / 1000 end
  local lim = C3.BANK_LIM_DEG

  -- This aircraft turns at exactly the rate commanded and needs no elevator to
  -- hold its nose, so the pitch trim the script adds for a real airframe would
  -- be a rate error here: off for this section.
  stubs.feed(pack_gain(GAIN_IDX.pitch_trim_offset, 0.0) .. pack_gain(GAIN_IDX.pitch_trim_lift, 0.0)
             .. pack_gain(GAIN_IDX.pitch_trim_attitude, 0.0) .. pack_gain(GAIN_IDX.pitch_trim_thrust, 0.0)
             .. pack_gain(GAIN_IDX.curve_flyer_enable, 1.0)
             .. pack_gain(GAIN_IDX.cf_plan_wait_s, C3.CF_PLAN_WAIT_S))
  stubs.advance(DT_MS); update()

  local WAIT_TICKS = 10
  local OWN = { own = true }
  local APPROACH_DEG = 45.0
  local APPROACH_TOL_DEG = 1.0
  -- The run-in is judged from this long into it: the line-up's end carries
  -- the path past the line for a moment, as a turn's end does the hold's.
  local RUNIN_SETTLE_S = 2.0
  local RUNIN_TOL_DEG = 1.0
  local DEP_QUANT_DEG = 0.01       -- GUIDE carries the balloon to 1e-4 of a unit vector
  local HOLD_S = 3.0
  local HOLD_TOL_DEG = 1.0
  local WOBBLE_DPS, WOBBLE_HZ = 35.0, 2.0
  local TURN_RATE_TOL_DPS = 0.1
  local SLOW = { pitch_share = 0.6, nose_lag_s = 0.1 }
  local SETTLED_AFTER_S = 1.0
  local SWING_DPS = 2.0
  local function agrees(rr, plan)
    return rr.done_tick ~= nil
       and math.abs(secs(rr.done_tick - WAIT_TICKS) - plan.turn_s) <= TURN_AGREE_S
  end
  local function states_in_order(rr)
    local list, last = {}, nil
    for t = WAIT_TICKS - 1, #rr.states do
      if rr.states[t] ~= last then list[#list + 1] = rr.states[t]; last = rr.states[t] end
    end
    return table.concat(list, " ")
  end
  local function order_of(...) return table.concat({ ... }, " ") end
  local function runin_off_line(rr, plan)
    local climb_err, head_err = 0.0, 0.0
    if not (rr.runin_tick and rr.dive_tick) then return math.huge, math.huge end
    for t = rr.runin_tick + ticks_of(RUNIN_SETTLE_S), rr.dive_tick - 1 do
      climb_err = math.max(climb_err, math.abs(rr.climb[t] - plan.climb))
      head_err = math.max(head_err, math.abs(rr.head_err[t]))
    end
    return climb_err, head_err
  end

  for _, law in ipairs({ 0, 1 }) do
    local who = LAW_NAMES[law]
    local plans = PLANS[law]
    local PLAN_P, PLAN_R1 = plans.P, plans.R1
    print(string.format("\n  -- steering_law %d, %s --", law, who))
    stubs.feed(pack_gain(GAIN_IDX.steering_law, law)); stubs.advance(DT_MS); update()
    local id = 20 * law

    local r = chase(BAL_P, ticks_of(CHASE_S), WAIT_TICKS, PLAN_P, id + 1, law, OWN)
    check(who .. ": a fresh chase waits for its PLAN, rates held at zero",
          r.states[WAIT_TICKS - 1] == C3.CF_STATE_WAIT and r.waits_still,
          string.format("state %s before the PLAN", tostring(r.states[WAIT_TICKS - 1])))
    check(who .. ": the PLAN is taken and its plan_id echoed in STATE2",
          r.plan_echo == id + 1 and r.states[WAIT_TICKS + 1] == C3.CF_STATE_FLY,
          string.format("echo %s, state %s", tostring(r.plan_echo),
                        tostring(r.states[WAIT_TICKS + 1])))
    check(who .. ": the law flies the turn in its plan mode, and says so",
          r.plan_mode)
    check(string.format("%s, the p balloon: the turn completes, the path within %g deg of it",
                        who, C3.CURVE_TURN_DONE_DEG),
          r.done_tick ~= nil and r.eps_done <= C3.CURVE_TURN_DONE_DEG + 1.0,
          r.done_tick and string.format("after %.2f s, %.1f deg off",
                                        secs(r.done_tick - WAIT_TICKS), r.eps_done)
          or "never completed")
    check(who .. ": once the turn is complete the law flies on without the PLAN", r.unplanned_after)
    check(string.format("%s, the p balloon: the turn takes what the planner predicts, within %g s",
                        who, TURN_AGREE_S),
          agrees(r, PLAN_P), string.format("%s s, planned %.2f s",
            r.done_tick and string.format("%.2f", secs(r.done_tick - WAIT_TICKS)) or "-",
            PLAN_P.turn_s))

    r = chase(BAL_R1, ticks_of(CHASE_S), WAIT_TICKS, PLAN_R1, id + 2, law, OWN)
    check(string.format("%s, rule 1: the turn completes, the path within %g deg of the balloon",
                        who, C3.CURVE_TURN_DONE_DEG),
          r.done_tick ~= nil and r.eps_done <= C3.CURVE_TURN_DONE_DEG + 1.0,
          r.done_tick and string.format("after %.2f s, %.1f deg off",
                                        secs(r.done_tick - WAIT_TICKS), r.eps_done)
          or "never completed")
    check(string.format("%s, rule 1: the turn takes what the planner predicts, within %g s",
                        who, TURN_AGREE_S),
          agrees(r, PLAN_R1), string.format("%s s, planned %.2f s",
            r.done_tick and string.format("%.2f", secs(r.done_tick - WAIT_TICKS)) or "-",
            PLAN_R1.turn_s))
    check(string.format("%s, rule 1: the bank proxy stays inside the %g deg limit while it flies",
                        who, lim),
          r.prox_lo >= -lim - BANK_SLOP_DEG and r.prox_hi <= lim + BANK_SLOP_DEG,
          string.format("bank proxy %.1f .. %.1f deg", r.prox_lo, r.prox_hi))

    -- Section 15's aircraft: other physics, the same closed loop.
    for _, case in ipairs({ { "the p balloon", BAL_P, PLAN_P }, { "rule 1", BAL_R1, PLAN_R1 } }) do
      r = chase(case[2], ticks_of(CHASE_S), WAIT_TICKS, case[3], id + 3, law)
      check(string.format("%s, %s on section 15's aircraft: the turn completes", who, case[1]),
            r.done_tick ~= nil and r.eps_done <= C3.CURVE_TURN_DONE_DEG + 1.0,
            r.done_tick and string.format("after %.2f s (%.2f on the planner's), %.1f deg off",
                                          secs(r.done_tick - WAIT_TICKS), case[3].turn_s, r.eps_done)
            or "never completed")
    end

    local wide = { eps0 = PLAN_R1.eps0, knots = { -80, -80, -80, -80, -80 } }
    r = chase(BAL_R1, ticks_of(CHASE_S), WAIT_TICKS, wide, id + 4, law)
    check(who .. ": a schedule past the bank limit is flown at the limit",
          r.prox_lo >= -lim - BANK_SLOP_DEG and r.prox_hi <= lim + BANK_SLOP_DEG,
          string.format("bank proxy %.1f .. %.1f deg", r.prox_lo, r.prox_hi))

    r = chase(BAL_P, WAIT_TICKS + 2, WAIT_TICKS, PLAN_P, id + 5, 1 - law)
    check(who .. ": a PLAN made for the other law is not taken",
          r.plan_echo == nil and r.states[WAIT_TICKS + 2] == C3.CF_STATE_WAIT,
          string.format("echo %s, state %s", tostring(r.plan_echo),
                        tostring(r.states[WAIT_TICKS + 2])))

    -- No clock hands a turn back: one that cannot complete is still flown.
    local level = { eps0 = 90.0, knots = { 0.0, 0.0, 0.0, 0.0, 0.0 } }
    r = chase(BAL_ABEAM, WAIT_TICKS + ticks_of(NO_CLOCK_S), WAIT_TICKS, level, id + 10, law)
    local s2 = last_state2()
    check(string.format("%s: a turn that never completes is still flown %g s on: no clock hands it back",
                        who, NO_CLOCK_S),
          s2.cf_state == C3.CF_STATE_FLY and s2.cf_plan_id == id + 10
            and s2.cf_bank_ref_deg == s2.cf_bank_ref_deg,
          string.format("state %d, plan %d, %.1f deg off the balloon", s2.cf_state, s2.cf_plan_id,
                        s2.cf_eps_deg))

    r = chase(BAL_P, ticks_of(C3.CF_PLAN_WAIT_S) + WAIT_TICKS, nil)
    check(who .. ": no PLAN in time: the law flies the chase without one",
          r.states[#r.states] == C3.CF_STATE_TIMEOUT,
          string.format("state %s", tostring(r.states[#r.states])))

    r = chase(BAL_P, WAIT_TICKS, nil)
    local late = pack_plan(id + 6, law, PLAN_P)
    stubs.feed(pack_guide(next_seq(), 0x01, 4, 0.0, 0.0, 55.0) .. late)
    stubs.advance(DT_MS); update()
    s2 = last_state2()
    check(who .. ": tracking off ends the chase; a PLAN then is not taken",
          s2.cf_state == C3.CF_STATE_OFF and s2.cf_plan_id == 0,
          string.format("state %d, plan %d", s2.cf_state, s2.cf_plan_id))

    -- Transfer Control When Done off: the turn's line is held.
    stubs.feed(pack_gain(GAIN_IDX.cf_transfer_when_done, 0.0)); stubs.advance(DT_MS); update()
    local turn_ticks = WAIT_TICKS + math.ceil(PLAN_P.turn_s * 1000 / DT_MS) + ticks_of(TURN_AGREE_S)
    r = chase(BAL_P, turn_ticks + ticks_of(HOLD_S), WAIT_TICKS, PLAN_P, id + 7, law, OWN)
    check(who .. ", transfer off: the turn completes and the law is aimed at its line",
          r.hold_tick ~= nil and r.done_tick == nil and r.hold_aimed,
          r.hold_tick and string.format("held from %.2f s", secs(r.hold_tick - WAIT_TICKS))
          or "never held")
    check(who .. ", transfer off: the line held is the path where the turn completed",
          r.hold_tick ~= nil and r.hold_vs_path < 0.1
            and r.eps_done <= C3.CURVE_TURN_DONE_DEG + 1.0,
          r.hold_tick and string.format("%.2f deg off the path, %.1f deg off the balloon",
                                        r.hold_vs_path, r.eps_done) or "never held")
    -- The turn's own end carries the path off the line for a moment (its bank
    -- and pitch rate do not stop on one tick); the hold brings it back and
    -- keeps it.
    check(string.format("%s, transfer off: %g s on, the path is back within %g deg of the line and its heading",
                        who, HOLD_S, HOLD_TOL_DEG),
          r.hold_tick ~= nil and r.hold_end_off <= HOLD_TOL_DEG and r.hold_end_head <= HOLD_TOL_DEG,
          r.hold_tick and string.format("%.2f deg off the line, heading %.2f deg (at worst %.1f and %.1f)",
                                        r.hold_end_off, r.hold_end_head, r.hold_off, r.hold_head)
          or "never held")

    stubs.feed(pack_gain(GAIN_IDX.cf_transfer_when_done, 1.0) .. pack_oracle(next_seq(), BAL_P))
    stubs.advance(DT_MS); update()
    s2 = last_state2()
    check(who .. ": transfer turned on while holding: the law flies without the PLAN from the next tick",
          s2.cf_state == C3.CF_STATE_DONE and s2.cf_bank_ref_deg ~= s2.cf_bank_ref_deg
            and s2.cf_hold_n ~= s2.cf_hold_n,
          string.format("state %d", s2.cf_state))

    -- Leaving p hands over on that tick, transfer off or not, and drops the plan.
    stubs.feed(pack_gain(GAIN_IDX.cf_transfer_when_done, 0.0)); stubs.advance(DT_MS); update()
    r = chase(BAL_P, turn_ticks + ticks_of(1.0), WAIT_TICKS, PLAN_P, id + 8, law, OWN)
    stubs.feed(pack_guide(next_seq(), 0x0A, 1, 0.0, 0.0, 55.0)); stubs.advance(DT_MS); update()
    s2 = last_state2()
    check(who .. ": holding, then straight: the law flies without the PLAN at once and the plan is gone",
          r.hold_tick ~= nil and s2.cf_state == C3.CF_STATE_OFF and s2.cf_plan_id == 0
            and s2.cf_bank_ref_deg ~= s2.cf_bank_ref_deg and s2.cf_hold_n ~= s2.cf_hold_n,
          string.format("held %s, state %d, plan %d", tostring(r.hold_tick ~= nil),
                        s2.cf_state, s2.cf_plan_id))
    local straight_off = true
    for _ = 1, WAIT_TICKS do
      stubs.feed(pack_guide(next_seq(), 0x0A, 1, 0.0, 0.0, 55.0)); stubs.advance(DT_MS); update()
      straight_off = straight_off and last_state2().cf_state == C3.CF_STATE_OFF
    end
    check(who .. ": straight never uses the Curve Flyer", straight_off)

    r = chase(BAL_P, WAIT_TICKS + ticks_of(0.3), WAIT_TICKS, PLAN_P, id + 9, law, OWN)
    stubs.feed(pack_oracle(next_seq(), BAL_P, 3)); stubs.advance(DT_MS); update()
    s2 = last_state2()
    check(who .. ": flying a p turn, then x: a fresh chase waits for its own PLAN",
          r.states[#r.states] == C3.CF_STATE_FLY and s2.cf_state == C3.CF_STATE_WAIT
            and s2.cf_plan_id == 0,
          string.format("state %d, plan %d", s2.cf_state, s2.cf_plan_id))

    -- The plan mode reads the PATH.  A nose that pitches and yaws in a
    -- sideslip the path does not follow - the gyro reading it, the path
    -- flying on - changes nothing the law commands, through the turn and the
    -- hold; and track.lua's turn rate is the turn the path flew.
    local function wobble(t) return WOBBLE_DPS * sin(2 * math.pi * WOBBLE_HZ * t * DT_MS / 1000) end
    local still = chase(BAL_P, turn_ticks + ticks_of(HOLD_S), WAIT_TICKS, PLAN_P, id + 1, law, OWN)
    local wobbly = chase(BAL_P, turn_ticks + ticks_of(HOLD_S), WAIT_TICKS, PLAN_P, id + 2, law,
                         { own = true, nose_q = wobble, nose_r = wobble })
    local cmd_diff, flown = 0.0, 0
    for t = WAIT_TICKS, #still.roll_cmd do
      flown = flown + 1
      cmd_diff = math.max(cmd_diff, math.abs(still.roll_cmd[t] - wobbly.roll_cmd[t]),
                          math.abs(still.pitch_cmd[t] - wobbly.pitch_cmd[t]))
    end
    check(string.format("%s: a nose wobbling %g deg/s in a sideslip the path does not follow "
                        .. "changes no command it flies under the PLAN", who, WOBBLE_DPS),
          still.hold_tick ~= nil and flown > ticks_of(HOLD_S) and cmd_diff < 1e-9,
          string.format("largest difference %.3g deg/s over %d ticks flown", cmd_diff, flown))
    check(string.format("%s: track.lua's turn rate is the turn the path flew, within %g deg/s",
                        who, TURN_RATE_TOL_DPS),
          still.turn_seen > ticks_of(HOLD_S) and still.turn_err < TURN_RATE_TOL_DPS,
          string.format("worst %.4f deg/s over %d ticks", still.turn_err, still.turn_seen))

    -- The hold on a slower aircraft - a share of the commanded pitch rate,
    -- after a lag: the path settles onto the line rather than swinging across
    -- it.  The turn is BAL_G's, which this aircraft completes too.
    r = chase(BAL_G, ticks_of(CHASE_S) + ticks_of(HOLD_S), WAIT_TICKS, plans.G, id + 3,
              law, SLOW)
    local swings, last_sign = 0, 0
    if r.hold_tick then
      for t = r.hold_tick + ticks_of(SETTLED_AFTER_S), #r.pitch_cmd do
        local q = r.pitch_cmd[t]
        if math.abs(q) > SWING_DPS then
          local sg = q > 0 and 1 or -1
          if last_sign ~= 0 and sg ~= last_sign then swings = swings + 1 end
          last_sign = sg
        end
      end
    end
    check(string.format("%s, a slower aircraft: the hold settles onto the line, "
                        .. "its path within %g deg and no pitch reversal after %g s",
                        who, HOLD_TOL_DEG, SETTLED_AFTER_S),
          r.hold_tick ~= nil and r.hold_end_off <= HOLD_TOL_DEG and swings == 0,
          r.hold_tick and string.format("%.2f deg off the line, %d reversals (at worst %.1f deg off)",
                                        r.hold_end_off, swings, r.hold_off) or "never held")
    stubs.feed(pack_gain(GAIN_IDX.cf_transfer_when_done, 1.0)); stubs.advance(DT_MS); update()

    -- THE APPROACH ANGLE.  PLAN_RS and PLAN_RA are the Curve Planner's plans
    -- at an Approach Angle of APPROACH_DEG, from level at this speed, for a
    -- balloon at BALLOON_RS - 20 deg below and 30 deg right, 60 m down, which
    -- no rule binds - and at BALLOON_RA - 15 deg below dead ahead, 40 m down,
    -- which rule 1 binds (curve_planner.py: Chase(0, 0, b, rng=R,
    -- speed_mps=18, approach_deg=45, law=L) and plan()), with the line-up,
    -- the dive's start and the turn's end the planner predicts, and the
    -- balloon's angle below the horizon it ends on.  Both start shallower than
    -- the Approach Angle, so the turn lines the path up level toward the
    -- balloon, the run-in holds that line while the balloon sinks below the
    -- horizon as the aircraft closes, and the dive starts when it sits the
    -- PLAN's depression below; the turn then ends where the planner
    -- predicts: on the Approach Angle where the law's dive reaches it
    -- without swinging through the balloon's line, shallower where it cannot.
    local PLAN_RS, PLAN_RA = plans.RS, plans.RA
    local function judge(name, rr, plan, order)
      check(string.format("%s, %s: the turn lines up, the run-in holds, the dive follows and the turn "
                          .. "completes, in that order", who, name),
            states_in_order(rr) == order, string.format("states %s", states_in_order(rr)))
      local climb_err, head_err = runin_off_line(rr, plan)
      check(string.format("%s, %s: %g s into the run-in, the path holds its line - level, heading at the "
                          .. "balloon - within %g deg", who, name, RUNIN_SETTLE_S, RUNIN_TOL_DEG),
            climb_err <= RUNIN_TOL_DEG and head_err <= RUNIN_TOL_DEG,
            string.format("climb off by %.2f deg, heading off by %.2f deg at worst", climb_err, head_err))
      local dt_ = rr.dive_tick
      check(string.format("%s, %s: the dive starts on the tick the balloon sits %g deg below the horizon",
                          who, name, plan.dep),
            dt_ ~= nil and dt_ > 1 and rr.dep[dt_] >= plan.dep - DEP_QUANT_DEG
              and rr.dep[dt_ - 1] < plan.dep + DEP_QUANT_DEG,
            dt_ and string.format("%.3f deg on that tick, %.3f the tick before", rr.dep[dt_], rr.dep[dt_ - 1])
            or "never dived")
      check(string.format("%s, %s: the turn ends where the planner predicts, the line to the balloon "
                          .. "%g deg below the horizon (Approach Angle %g) within %g deg",
                          who, name, plan.end_dep, APPROACH_DEG, APPROACH_TOL_DEG),
            rr.end_dep ~= nil and math.abs(rr.end_dep - plan.end_dep) <= APPROACH_TOL_DEG,
            rr.end_dep and string.format("%.2f deg", rr.end_dep) or "never completed")
      local lined = rr.runin_tick and secs(rr.runin_tick - WAIT_TICKS)
      local dived = rr.dive_tick and rr.done_tick and secs(rr.done_tick - rr.dive_tick)
      check(string.format("%s, %s: the line-up and the dive take what the planner predicts, within %g s",
                          who, name, TURN_AGREE_S),
            lined ~= nil and dived ~= nil and math.abs(lined - plan.lined_s) <= TURN_AGREE_S
              and math.abs(dived - (plan.done_s - plan.dive_s)) <= TURN_AGREE_S,
            string.format("line-up %s s (planned %.2f), dive %s s (planned %.2f)",
                          lined and string.format("%.2f", lined) or "-", plan.lined_s,
                          dived and string.format("%.2f", dived) or "-", plan.done_s - plan.dive_s))
    end
    local run_ticks = WAIT_TICKS + ticks_of(PLAN_RS.done_s + 2.0)
    r = chase(nil, run_ticks, WAIT_TICKS, PLAN_RS, id + 4, law, { own = true, balloon_pos = BALLOON_RS })
    judge("a run-in to the side", r, PLAN_RS,
          order_of(C3.CF_STATE_WAIT, C3.CF_STATE_FLY, C3.CF_STATE_RUNIN, C3.CF_STATE_DIVE, C3.CF_STATE_DONE))
    r = chase(nil, WAIT_TICKS + ticks_of(PLAN_RA.done_s + 2.0), WAIT_TICKS, PLAN_RA, id + 5, law,
              { own = true, balloon_pos = BALLOON_RA })
    -- Already on the run-in line at the start: the turn is done on the tick the
    -- PLAN is taken, and the run-in flies from that tick.
    judge("rule 1, a run-in ahead", r, PLAN_RA,
          order_of(C3.CF_STATE_WAIT, C3.CF_STATE_RUNIN, C3.CF_STATE_DIVE, C3.CF_STATE_DONE))
    check(string.format("%s, rule 1, a run-in ahead: the bank proxy stays inside the %g deg limit "
                        .. "through the turn, the run-in and the dive", who, lim),
          r.cf_prox_lo >= -lim - BANK_SLOP_DEG and r.cf_prox_hi <= lim + BANK_SLOP_DEG,
          string.format("bank proxy %.1f .. %.1f deg", r.cf_prox_lo, r.cf_prox_hi))
    stubs.feed(pack_gain(GAIN_IDX.cf_transfer_when_done, 0.0)); stubs.advance(DT_MS); update()
    r = chase(nil, run_ticks, WAIT_TICKS, PLAN_RS, id + 6, law, { own = true, balloon_pos = BALLOON_RS })
    check(who .. ": a run-in with transfer off: the dive's end is held",
          states_in_order(r) == order_of(C3.CF_STATE_WAIT, C3.CF_STATE_FLY, C3.CF_STATE_RUNIN,
                                         C3.CF_STATE_DIVE, C3.CF_STATE_HOLD)
            and r.hold_vs_path ~= nil and r.hold_vs_path < 0.1,
          string.format("states %s", states_in_order(r)))
    stubs.feed(pack_gain(GAIN_IDX.cf_transfer_when_done, 1.0)); stubs.advance(DT_MS); update()
  end

  stubs.feed(pack_gain(GAIN_IDX.steering_law, C3.STEERING_LAW)
             .. pack_gain(GAIN_IDX.curve_flyer_enable, 0.0)); stubs.advance(DT_MS); update()
  local r = chase(BAL_P, WAIT_TICKS, nil)
  check("Curve Flyer off: the law flies without a PLAN from the first tick",
        r.states[WAIT_TICKS] == C3.CF_STATE_OFF,
        string.format("state %s", tostring(r.states[WAIT_TICKS])))
  stubs.sim.ax, stubs.sim.ay, stubs.sim.az = 0.0, 0.0, -GR
end)()

print("\n17. LIFT VECTOR POINTING: SELECTED, LOGGED, LIMITED AND CLEARED")
-- steering_law 1 hands every mode to Lift Vector Pointing (path_attitude.lua)
-- at once (C-03).  Its internals reach the log and the cascade's are blank;
-- it sends no yaw and holds the rudder at zero; it holds the bank proxy to
-- the bank limit in every chase, and aims at the limit itself when the turn
-- wants more; it never rolls the aircraft upside down; its lift side is the
-- side of the wing line the force line lies on, a push the room stops from
-- facing it a pull instead; its command is its own rate, cut only at the ACRO
-- rates; its integrators run only near the balloon and are cleared on every
-- tick their output does not reach the aircraft; and it allocates nothing per tick.
;(function()
  local C3 = require("constants")
  local PA = require("path_attitude")
  local sin, cos, asin, rad, deg = math.sin, math.cos, math.asin, math.rad, math.deg
  local DT_MS = 20
  local function isnan(x) return x ~= x end
  local frames = {}
  for i, name in pairs(proto.GAIN_NAMES) do
    local v = C3[string.upper(name)]
    if type(v) == "number" then frames[#frames + 1] = pack_gain(i, v) end
  end
  for k = 1, #frames, 8 do
    stubs.feed(table.concat(frames, "", k, math.min(k + 7, #frames)))
    stubs.advance(DT_MS); update()
  end
  local seq = 0
  local function next_seq() seq = seq % 60000 + 1; return seq end
  -- an oracle chase (ptest) toward the world unit vector (bx, by, bz)
  local function pack_aim(sq, bx, by, bz, mode)
    local body = string.pack('<I2BBi2i2i2i2i2i2i2i2BBi2i2i2I4I2',
      sq, 0x06, mode or 4, 0, 0, math.floor(55.0 * 100 + 0.5), 0, 0, 0, 0, 0, 1, 0,
      math.floor(bx * 10000 + 0.5), math.floor(by * 10000 + 0.5),
      math.floor(bz * 10000 + 0.5), 0, 12)
    local hb = string.pack('<I2BB', proto.MAGIC, proto.MSG_GUIDE, #body) .. body
    return hb .. string.pack('<I2', proto.crc16(hb, 1, #hb))
  end
  local function aim(el_deg, az_deg, extra)
    local el, az = rad(el_deg), rad(az_deg)
    stubs.feed((extra or "") .. pack_aim(next_seq(), cos(el) * cos(az), cos(el) * sin(az), -sin(el)))
    stubs.advance(DT_MS); update()
    return last_state(), last_state2(), stubs.log.rate_cmds[#stubs.log.rate_cmds]
  end
  local function stand_down()
    stubs.feed(pack_guide(next_seq(), 0x01, 4, 0.0, 0.0, 55.0)); stubs.advance(DT_MS); update()
  end
  -- level flight on the nose, 1 g on the accelerometer, no rotation
  local sim = stubs.sim
  local saved = { roll = sim.roll, pitch = sim.pitch, yaw = sim.yaw, p = sim.p, q = sim.q,
                  r = sim.r, vn = sim.vn, ve = sim.ve, vd = sim.vd, eas = sim.eas,
                  ax = sim.ax, ay = sim.ay, az = sim.az }
  sim.quat = nil
  sim.roll, sim.pitch, sim.yaw, sim.p, sim.q, sim.r = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
  sim.vn, sim.ve, sim.vd, sim.eas = 18.0, 0.0, 0.0, 18.0
  sim.ax, sim.ay, sim.az = 0.0, 0.0, -C3.GRAVITY_MPS2

  stand_down()
  local st, s2 = aim(20, 0)
  check("the cascade flies by default: cs_flags bit 6 clear, the law's internals blank",
        (st.cs_flags & 0x40) == 0 and isnan(st.att_theta_now_deg) and isnan(st.att_n_lift)
        and isnan(st.att_band_lo_deg) and isnan(s2.att_theta_deg) and isnan(s2.att_side)
        and not isnan(st.requested_roll),
        string.format("cs_flags %d, att_theta_now %s", st.cs_flags, tostring(st.att_theta_now_deg)))
  check("under the cascade the pitch trim's lift term is cos(pitch)",
        math.abs(st.pitch_trim_lift_term - 1.0) < 1e-9,
        string.format("lift term %.6f", st.pitch_trim_lift_term))

  stubs.feed(pack_gain(GAIN_IDX.steering_law, 1.0))
  stand_down()
  local cmd
  st, s2, cmd = aim(20, 0)
  local nr = stubs.log.rudder_offsets[#stubs.log.rudder_offsets]
  check("with steering_law 1 Lift Vector Pointing flies: cs_flags bit 6, its internals numbers",
        st.cs_flags == 0x40 and math.abs(st.att_theta_now_deg - 20.0) < 0.01
        and math.abs(s2.att_theta_deg - 20.0) < 0.01 and s2.att_side == 1
        and math.abs(s2.att_u_z + 1.0) < 1e-3 and math.abs(s2.att_mu_deg) < 0.01
        and not isnan(st.att_wy_dps) and not isnan(st.att_p_dps),
        string.format("cs_flags %d, theta %.3f / %.3f, side %s, u_z %.4f, mu %.3f",
                      st.cs_flags, st.att_theta_now_deg, s2.att_theta_deg, tostring(s2.att_side),
                      s2.att_u_z, s2.att_mu_deg))
  check("the path turns at att_k_path times its angle off the balloon, below the cap",
        math.abs(s2.att_omega_dps - math.min(C3.ATT_K_PATH * 20.0, C3.ATT_PATH_RATE_MAX_DPS)) < 0.01,
        string.format("%.3f deg/s", s2.att_omega_dps))
  check("and the cascade's internals are blank",
        isnan(st.requested_roll) and isnan(st.pan_integrator) and isnan(st.theta_ip_deg)
        and isnan(s2.bank_rate_cmd_dps) and isnan(s2.plane_err_deg) and isnan(s2.yaw_point_i_dps)
        and not isnan(s2.sideslip_deg) and not isnan(s2.aoa_deg))
  check("version B sends no yaw and holds the rudder at zero: yaw 0, set_rudder_offset(0, false)",
        cmd[4] == 0.0 and nr[1] == 0.0 and nr[2] == false and isnan(st.yaw_rate_cmd_dps)
        and isnan(st.yaw_point_dps) and isnan(st.yaw_coordination_dps),
        string.format("yaw %s, rudder offset (%s, %s)", tostring(cmd[4]), tostring(nr[1]),
                      tostring(nr[2])))
  check("the pitch trim's lift term is the load the law asks for",
        math.abs(st.pitch_trim_lift_term - st.att_n_lift) < 1e-9
        and math.abs(st.att_n_lift + s2.att_f_z_mps2 / C3.GRAVITY_MPS2) < 1e-4,
        string.format("lift term %.4f, n_lift %.4f", st.pitch_trim_lift_term, st.att_n_lift))

  -- THE LEAD: the error is read from the path att_lead_s ahead.  Pulling 2 g
  -- level, the path turns up at g / V, so the balloon 20 deg up reads
  -- att_lead_s * g / V nearer; with no accelerometer reading it reads as it is.
  sim.az = -2.0 * C3.GRAVITY_MPS2
  st, s2 = aim(20, 0)
  local ahead = 20.0 - deg(math.atan(C3.ATT_LEAD_S * C3.GRAVITY_MPS2 / sim.vn))
  check("the error is read from the path att_lead_s ahead, as the accelerometer sees it turn",
        math.abs(s2.att_theta_deg - ahead) < 0.01 and math.abs(st.att_theta_now_deg - 20.0) < 0.01,
        string.format("%.3f deg ahead (%.3f expected), %.3f as read", s2.att_theta_deg, ahead,
                      st.att_theta_now_deg))
  sim.accel_valid = false
  local sa, s2a, ca = aim(20, 0)
  sim.accel_valid, sim.az = true, -C3.GRAVITY_MPS2
  check("with no accelerometer reading it flies without the lead, not on NaN",
        math.abs(s2a.att_theta_deg - sa.att_theta_now_deg) < 1e-4 and ca[3] == ca[3]
        and ca[3] ~= 0.0,
        string.format("%.3f ahead, %.3f as read, pitch %.2f deg/s", s2a.att_theta_deg,
                      sa.att_theta_now_deg, ca[3]))

  -- THE BANK BAND, decided on the first tick of every chase, wherever the
  -- balloon is, on the bank proxy - the wing tip's angle below the horizon,
  -- asin(sin(roll) * cos(pitch)), as the cascade measures it: the bank limit
  -- either way, widened to a starting proxy beyond it.
  local lim = C3.BANK_LIM_DEG
  local function band_of(el, az, bank_deg)
    sim.roll = rad(bank_deg or 0.0)
    stand_down()
    local a = aim(el, az)
    sim.roll = 0.0
    return { a.att_band_lo_deg, a.att_band_hi_deg, a.bank_proxy_deg }
  end
  -- the path on the nose, pitched, in steady flight: no acceleration to lead
  local function pitch_to(deg_up)
    local p = rad(deg_up)
    sim.pitch, sim.vn, sim.vd = p, 18.0 * cos(p), -18.0 * sin(p)
    sim.ax, sim.az = C3.GRAVITY_MPS2 * sin(p), -C3.GRAVITY_MPS2 * cos(p)
  end
  local function is_lim(b) return b[1] == -lim and b[2] == lim end
  local function band_txt(b) return string.format("%s..%s", tostring(b[1]), tostring(b[2])) end
  local above, below, off_plane = band_of(30, 0), band_of(-30, 0), band_of(-30, 15)
  local side, behind = band_of(0, 60), band_of(-30, 120)
  check("every chase is given the bank limit: above, below, off the plane, to the side, behind",
        is_lim(above) and is_lim(below) and is_lim(off_plane) and is_lim(side) and is_lim(behind),
        string.format("above %s, below %s, off the plane %s, side %s, behind %s", band_txt(above),
                      band_txt(below), band_txt(off_plane), band_txt(side), band_txt(behind)))
  local right, left = band_of(-30, 0, 60), band_of(-30, 0, -60)
  check("a start banked beyond the limit widens the band to the starting bank, on that side only",
        right[1] == -lim and math.abs(right[2] - 60.0) < 1e-3
        and math.abs(left[1] + 60.0) < 1e-3 and left[2] == lim,
        string.format("banked 60 right %s, 60 left %s", band_txt(right), band_txt(left)))
  pitch_to(30.0)
  local up30 = band_of(0, 0, 60)
  local proxy30 = deg(asin(sin(rad(60.0)) * cos(rad(30.0))))
  check("the starting bank is the proxy: banked 60 with the nose 30 up widens the band to "
        .. "asin(sin 60 cos 30), and the log's bank_proxy_deg reads it",
        up30[1] == -lim and math.abs(up30[2] - proxy30) < 1e-3
        and math.abs(up30[3] - proxy30) < 1e-3,
        string.format("%s, proxy %.3f (expected %.3f)", band_txt(up30), up30[3], proxy30))
  local steep_deg = 90.0 - C3.CURVE_POLE_FADE_DEG / 2.0
  pitch_to(steep_deg)
  local steep = band_of(0, 0, 60)
  pitch_to(0.0)
  check("near vertical a start reads almost no proxy whatever its roll, and takes the limit alone",
        is_lim(steep), string.format("banked 60, path %g deg up: %s, proxy %.2f", steep_deg,
                                     band_txt(steep), steep[3]))

  -- Level, a balloon square to the right asks for more bank than the limit:
  -- the roll still to go is the limit itself.
  stand_down()
  local held_st, held_s2 = aim(0, 90)
  check("a turn that wants more bank than the limit aims at the limit itself",
        math.abs(held_s2.att_mu_deg - lim) < 1e-3,
        string.format("roll to go %.3f, limit %g", held_s2.att_mu_deg, lim))
  check("level, the roll may go the limit either way and no further: the wing tip's lowest "
        .. "point is not rolled through to the far side",
        held_st.att_room_lo_deg == -lim and held_st.att_room_hi_deg == lim,
        string.format("room %.3f .. %.3f", held_st.att_room_lo_deg, held_st.att_room_hi_deg))

  -- THE LIFT SIDE is the side of the wing line f lies on.  With the guidance at
  -- its cap and the balloon square to the right, f lies on the wing line where
  -- the turn's downward part cancels gravity: u's downward part is
  -- g / (V * cap), and with the sideways part weighed att_k_side times that is
  -- a dive of atan(att_k_side * tan(asin(g / (V * cap)))).
  sim.accel_valid = false
  local on_line = deg(math.atan(C3.ATT_K_SIDE * math.tan(asin(C3.GRAVITY_MPS2
                                / (sim.vn * rad(C3.ATT_PATH_RATE_MAX_DPS))))))
  stand_down()
  local _, s2above = aim(-(on_line - 5.0), 90)
  stand_down()
  local _, s2down = aim(-90, 0)
  check(string.format("wings level, f on the canopy's side of the wing line pulls, and on the "
                      .. "belly's side, where the belly faces it, pushes: the balloon square to the "
                      .. "right %.1f below, and straight below", on_line - 5.0),
        s2above.att_side == 1 and s2above.att_f_z_mps2 < 0.0
        and s2down.att_side == -1 and s2down.att_f_z_mps2 > 0.0 and math.abs(s2down.att_mu_deg) < 1e-3,
        string.format("square: side %s, f_z %.3f; below: side %s, f_z %.3f, roll to go %.4f",
                      tostring(s2above.att_side), s2above.att_f_z_mps2, tostring(s2down.att_side),
                      s2down.att_f_z_mps2, s2down.att_mu_deg))
  -- f on the belly's side where the room stops the belly from facing it: a
  -- pull instead, held at the limit.
  stand_down()
  local _, s2below = aim(-(on_line + 5.0), 90)
  check(string.format("wings level, f on the belly's side but past the roll the room allows: "
                      .. "the law pulls instead, held at the limit (the balloon square to the "
                      .. "right %.1f below)", on_line + 5.0),
        s2below.att_side == 1 and math.abs(s2below.att_mu_deg - lim) < 1e-3,
        string.format("side %s, roll to go %.4f (limit %g)",
                      tostring(s2below.att_side), s2below.att_mu_deg, lim))
  -- Banked to the limit with f below the horizon but on the canopy's side of
  -- the wing line, the law pulls, and the roll is held at the limit.  The push
  -- the room allows fits f better there; choosing the better fit is the tie
  -- the law would steer itself onto.
  sim.roll = rad(lim)
  stand_down()
  local _, s2bank = aim(-(on_line + 10.0), 90)
  sim.roll = 0.0
  check("banked to the limit, f below the horizon but on the canopy's side of the wing line: "
        .. "the law pulls, the roll held at the limit",
        s2bank.att_side == 1 and math.abs(s2bank.att_mu_deg) < 1e-3
        and s2bank.att_f_z_mps2 < 0.0,
        string.format("side %s, roll to go %.4f, f (%.4f, %.4f)",
                      tostring(s2bank.att_side), s2bank.att_mu_deg, s2bank.att_f_y_mps2,
                      s2bank.att_f_z_mps2))

  -- UPSIDE DOWN, as the rules of a chase define it, on the aircraft's own
  -- belly-canopy axis.  With the path within the limit of vertical the band
  -- cuts nothing; the rule leaves the banks within acos(q) of up, q =
  -- tan(aoa) * tan(climb).  Pushing in a dive (the path above the nose) q is
  -- positive and the room is under 90; pulling in it, over.
  local function dive(nose_deg, path_deg)
    sim.pitch = rad(nose_deg)
    sim.vn, sim.vd = 18.0 * cos(rad(path_deg)), -18.0 * sin(rad(path_deg))
    stand_down()
    local a = aim(-90, 0)
    return a
  end
  local path_dn = -(90.0 - lim + 5.0)
  local push_dn, pull_dn = dive(path_dn - 5.0, path_dn), dive(path_dn + 5.0, path_dn)
  local h_push = deg(math.acos(math.tan(rad(-5.0)) * math.tan(rad(path_dn))))
  local h_pull = deg(math.acos(math.tan(rad(5.0)) * math.tan(rad(path_dn))))
  check(string.format("diving %g deg, the canopy axis meets the ground on the canopy side "
                      .. "before 90 deg of bank pushing and after it pulling: acos(tan(aoa) * "
                      .. "tan(climb)) either way", -path_dn),
        h_push < 90.0 and h_pull > 90.0
        and math.abs(push_dn.att_room_hi_deg - h_push) < 1e-3 and math.abs(push_dn.att_room_lo_deg + h_push) < 1e-3
        and math.abs(pull_dn.att_room_hi_deg - h_pull) < 1e-3 and math.abs(pull_dn.att_room_lo_deg + h_pull) < 1e-3,
        string.format("pushing: room %.3f .. %.3f (expected +-%.3f); pulling: %.3f .. %.3f (+-%.3f)",
                      push_dn.att_room_lo_deg, push_dn.att_room_hi_deg, h_push,
                      pull_dn.att_room_lo_deg, pull_dn.att_room_hi_deg, h_pull))
  -- Already upside down on a steep path, the roll may only come back.
  pitch_to(90.0 - lim + 5.0)
  stand_down()
  aim(0, 0)
  sim.roll = rad(120.0)
  local stinv = aim(0, 90)
  sim.roll = 0.0
  check("upside down on a steep path, banked 120, the roll may only come back toward up",
        stinv.att_room_hi_deg == 0.0 and stinv.att_room_lo_deg < -(120.0 - 90.0),
        string.format("room %.3f .. %.3f", stinv.att_room_lo_deg, stinv.att_room_hi_deg))
  pitch_to(0.0)
  sim.accel_valid = true
  -- The steeper the path, the more roll the same proxy takes: asin(sin(limit)
  -- / cos(pitch)) with the path on the nose; with the path no further than the
  -- limit from vertical, no roll reaches it.
  pitch_to(30.0)
  stand_down()
  local st30, s230 = aim(0, 90)
  local edge30 = deg(asin(sin(rad(lim)) / cos(rad(30.0))))
  check("nose 30 up the same turn rolls to asin(sin(limit) / cos 30), where the proxy reaches "
        .. "the limit",
        math.abs(s230.att_mu_deg - edge30) < 1e-3
        and math.abs(st30.att_room_hi_deg - edge30) < 1e-3,
        string.format("roll to go %.3f, room %.3f (expected %.3f)",
                      s230.att_mu_deg, st30.att_room_hi_deg, edge30))
  pitch_to(90.0 - lim + 5.0)
  stand_down()
  local stfree, s2free = aim(0, 90)
  check(string.format("nose %g up no roll takes the proxy to the limit, and only upside down "
                      .. "limits the roll, 90 deg either way: the roll goes as far as the turn "
                      .. "needs", 90.0 - lim + 5.0),
        math.abs(stfree.att_room_lo_deg + 90.0) < 1e-3 and math.abs(stfree.att_room_hi_deg - 90.0) < 1e-3
        and s2free.att_mu_deg > edge30,
        string.format("room %.1f .. %.1f, roll to go %.3f", stfree.att_room_lo_deg,
                      stfree.att_room_hi_deg, s2free.att_mu_deg))
  -- Banked past the band mid-chase, the roll may only come back: by the
  -- nearer end of the arc it is in, and on no further than the far side's
  -- limit.
  pitch_to(0.0)
  stand_down()
  aim(0, 0)
  sim.roll = rad(60.0)
  local stpast, s2past = aim(0, 90)
  sim.roll = 0.0
  check("banked 60 mid-chase with the band at the limit, the roll may only come back: 15 to "
        .. "105 deg left",
        math.abs(stpast.att_room_hi_deg + (60.0 - lim)) < 1e-3
        and math.abs(stpast.att_room_lo_deg + (60.0 + lim)) < 1e-3
        and s2past.att_mu_deg <= -(60.0 - lim) + 1e-3,
        string.format("room %.3f .. %.3f, roll to go %.3f", stpast.att_room_lo_deg,
                      stpast.att_room_hi_deg, s2past.att_mu_deg))
  stand_down()

  -- THE COMMAND is the law's own rate, cut only at ACRO_ROLL_RATE: the first
  -- tick of a chase sends it whatever the gyro reads.
  local acro_roll = stubs.param_values.ACRO_ROLL_RATE
  sim.p = rad(10.0)
  stand_down()
  local stj, _, cj = aim(0, 90)
  check("a fresh chase's first roll command is the law's own rate, limited only by ACRO_ROLL_RATE",
        math.abs(cj[2] - math.max(-acro_roll, math.min(acro_roll, stj.att_p_dps))) < 1e-6,
        string.format("gyro 10, law %.3f, sent %.3f deg/s", stj.att_p_dps, cj[2]))
  sim.p = 0.0

  -- THE INTEGRATORS run inside att_i_zone_deg of the balloon only.
  stand_down()
  local far = aim(20, 0)
  local near
  for _ = 1, 10 do near = aim(1, 0) end
  check("outside the zone the integrators are zero; inside it the pitch one runs",
        far.att_pitch_i_dps == 0.0 and far.att_roll_i_dps == 0.0 and near.att_pitch_i_dps > 0.0,
        string.format("20 deg off: %.4f, 1 deg off after 10 ticks: %.4f deg/s",
                      far.att_pitch_i_dps, near.att_pitch_i_dps))
  local out = aim(20, 0)
  check("leaving the zone clears them", out.att_pitch_i_dps == 0.0,
        string.format("%.4f deg/s", out.att_pitch_i_dps))
  for _ = 1, 10 do near = aim(1, 0) end
  local held = aim(1, 0, pack_gain(GAIN_IDX.att_pitch_ki, 0.0))
  check("a gain of 0 clears the integrator rather than holding it",
        near.att_pitch_i_dps > 0.0 and held.att_pitch_i_dps == 0.0,
        string.format("%.4f -> %.4f deg/s", near.att_pitch_i_dps, held.att_pitch_i_dps))
  stubs.feed(pack_gain(GAIN_IDX.att_pitch_ki, C3.ATT_PITCH_KI))
  for _ = 1, 10 do near = aim(1, 0) end
  sim.mode = 5                                   -- FBWA: the law runs, ArduPlane flies FBWA
  local fbwa = aim(1, 0)
  sim.mode = 4
  local back = aim(1, 0)
  check("a tick out of ACRO clears them, and the return to ACRO starts from zero",
        near.att_pitch_i_dps > 0.0 and fbwa.att_pitch_i_dps == 0.0
        and back.att_pitch_i_dps < near.att_pitch_i_dps,
        string.format("%.4f, out of ACRO %.4f, back %.4f deg/s", near.att_pitch_i_dps,
                      fbwa.att_pitch_i_dps, back.att_pitch_i_dps))
  for _ = 1, 10 do near = aim(1, 0) end
  stubs.feed(pack_excite(4, 1, 1, -12.0, 0.0, 2000))  -- STEP, pitch, replace
  local ste, _, ce = aim(1, 0)
  check("a pitch excitation that replaces is the whole pitch command, its integrator cleared",
        math.abs(ce[3] + 12.0) < 1e-6 and ste.att_pitch_i_dps == 0.0,
        string.format("pitch %.3f deg/s, integrator %.4f", ce[3], ste.att_pitch_i_dps))
  stubs.feed(pack_excite(0, 0, 0, 0.0, 0.0, 0))

  -- STRAIGHT flies the same law (C-03): the mode changes only the world vector.
  stubs.feed(pack_straight(next_seq())); stubs.advance(DT_MS); update()
  local sts = last_state()
  check("straight flies the same law", (sts.cs_flags & 0x40) ~= 0,
        string.format("cs_flags %d", sts.cs_flags))

  -- NOTHING ALLOCATED PER TICK.
  local real_step = PA.step
  local grew_kb, calls = 0.0, 0
  PA.step = function(...)
    collectgarbage("stop")
    local before = collectgarbage("count")
    local p1, q1 = real_step(...)
    grew_kb = grew_kb + (collectgarbage("count") - before)
    calls = calls + 1
    collectgarbage("restart")
    return p1, q1
  end
  stand_down()
  for i = 1, 50 do sim.roll = 0.3 * sin(i * 0.1); aim(-40 + i, 30 + 2 * i) end
  grew_kb, calls = 0.0, 0
  for i = 1, 50 do sim.roll = 0.3 * sin(i * 0.1); aim(-40 + i, 30 + 2 * i) end
  PA.step = real_step
  check("PA.step allocates nothing", calls == 50 and grew_kb == 0.0,
        string.format("%d calls, %.3f KB allocated", calls, grew_kb))

  stubs.feed(pack_gain(GAIN_IDX.steering_law, C3.STEERING_LAW))
  stand_down()
  for k, v in pairs(saved) do sim[k] = v end
end)()

print("\n18. LIFT VECTOR POINTING, FLOWN CLOSED-LOOP ON A RIGID-BODY AIRFRAME")
-- CLAUDE.md "The rules of a chase", with steering_law 1.  The whole FC
-- script, its default gains and the pitch trim fitted to the airframe it flies,
-- steers test_plant.lua's model of the simulated Mini Talon through `p`
-- chases: the SDF's lift-drag elements, ArduPlane's rate loop on the Acro set,
-- the V-tail mixer, the surfaces' delay and lag, the rudder at zero.  The
-- balloons cover the sphere, from wings level and from steady level turns
-- banked RULE_BANK_DEG either way, and the default p balloon from level.
--   Every chase lines up: the alignment criterion (CLAUDE.md "What the p and
-- x tests are for") held HOLD_S.  When it did is reported, never judged - the
-- competition sets no deadline - and a chase that never did, by how close it
-- came.
--   The rule, for every chase but one whose balloon starts behind - more than
-- 90 deg from the path at the start - and for the default p balloon: no
-- Suboptimal Curve at any point in the chase - the bank proxy, the wing tip's
-- angle below the horizon the law's band holds, inside the band
-- [min(-limit, b0), max(limit, b0)], b0 the proxy at the start; once the turn
-- is complete, the heading within CURVE_TURN_DONE_DEG of the turn end's,
-- weighed by the path's distance from vertical; the tilt never back by
-- more than TILT_BACK_TRAVEL_DEG from the furthest it has reached toward the
-- balloon's side; and never upside down, the belly-canopy axis meeting the
-- ground on the canopy side - every chase here starts the right way up.  The
-- rules judge the flown chase; the law steers by the bank limit and upside
-- down, and does not sort chases by them.
--   A balloon right and below whose force line lies between the band's two
-- ends keeps its lift side: the side is the one of the wing line the force
-- line lies on, and the law's own flight never moves it back and forth.
--   How far the flown proxy goes past the band in every chase is reported,
-- not judged: the law aims at the limit itself, and the roll can carry past
-- it before it settles.
;(function()
  local C3 = require("constants")
  local PL = require("test_plant")
  local sin, cos, sqrt, rad, deg = math.sin, math.cos, math.sqrt, math.rad, math.deg
  local acos, asin = math.acos, math.asin
  local DT_MS = 20
  local CHASE_S = 12
  local HOLD_S = 0.5                         -- the alignment criterion's hold
  local RULE_BANK_DEG = 60
  local BANKS_DEG = { 0, RULE_BANK_DEG, -RULE_BANK_DEG }
  -- a balloon right and below whose force line, from level, lies between the
  -- band's two ends
  local GAP_LAT_DEG, GAP_LON_DEG = -41, 44
  local LATS_DEG = { -90, -75, -60, -40, -20, 0, 20, 40, 60, 75, 90 }
  local LONS_DEG = { 0, 10, 30, 60, 90, 120, 150, 170, 180, -90 }
  local TRIM_LO_DEG, TRIM_HI_DEG, TRIM_STEP_DEG = -90, 90, 15
  local TRIM_SPEEDS = { 15.0, 17.0, 19.0, 21.0 }
  local THROTTLE_PCT = C3.THROTTLE_PCT
  local lim = C3.BANK_LIM_DEG
  local pole_sin = sin(rad(C3.CURVE_POLE_FADE_DEG))
  local hold_ticks = math.floor(HOLD_S * 1000 / DT_MS + 0.5)
  local BEHIND_EPS = 1e-9                    -- a cosine's rounding, not an angle
  local function clip(v, lo, hi) return math.max(lo, math.min(hi, v)) end
  local function wrap180(a) return (a + 180.0) % 360.0 - 180.0 end

  -- The FC's pitch trim converts elevator into a rate request through the
  -- rate loop's FF and P: the ones the airframe flies.
  local pv = stubs.param_values
  local saved_ff, saved_p = pv.PTCH_RATE_FF, pv.PTCH_RATE_P
  pv.PTCH_RATE_FF, pv.PTCH_RATE_P = PL.ACRO.PTCH_RATE_FF, PL.ACRO.PTCH_RATE_P

  -- Every gain to its default, then the law, and the pitch trim fitted to
  -- this airframe as the trim map ('e') fits it under the law.
  local t_o, t_l, t_a, t_t = PL.fit_trim_map(C3, TRIM_LO_DEG, TRIM_HI_DEG, TRIM_STEP_DEG,
                                             TRIM_SPEEDS)
  local frames = {}
  for i, name in pairs(proto.GAIN_NAMES) do
    local v = C3[string.upper(name)]
    if type(v) == "number" then frames[#frames + 1] = pack_gain(i, v) end
  end
  frames[#frames + 1] = pack_gain(GAIN_IDX.steering_law, 1.0)
  frames[#frames + 1] = pack_gain(GAIN_IDX.pitch_trim_offset, t_o)
  frames[#frames + 1] = pack_gain(GAIN_IDX.pitch_trim_lift, t_l)
  frames[#frames + 1] = pack_gain(GAIN_IDX.pitch_trim_attitude, t_a)
  frames[#frames + 1] = pack_gain(GAIN_IDX.pitch_trim_thrust, t_t)
  for k = 1, #frames, 8 do
    stubs.feed(table.concat(frames, "", k, math.min(k + 7, #frames)))
    stubs.advance(DT_MS); update()
  end

  local seq = 0
  local function next_seq() seq = seq % 60000 + 1; return seq end
  local function pack_oracle(sq, bx, by, bz)
    local body = string.pack('<I2BBi2i2i2i2i2i2i2i2BBi2i2i2I4I2',
      sq, 0x06, 4, 0, 0, math.floor(THROTTLE_PCT * 100 + 0.5), 0, 0, 0, 0, 0, 1, 0,
      math.floor(bx * 10000 + 0.5), math.floor(by * 10000 + 0.5),
      math.floor(bz * 10000 + 0.5), 0, 12)
    local hb = string.pack('<I2BB', proto.MAGIC, proto.MSG_GUIDE, #body) .. body
    return hb .. string.pack('<I2', proto.crc16(hb, 1, #hb))
  end

  local ac = PL.new()
  local V0 = PL.trim(0.0, THROTTLE_PCT / 100.0)
  local ticks = math.floor(CHASE_S * 1000 / DT_MS + 0.5)
  -- the bank proxy: the wing tip's angle below the horizon, asin of world
  -- down along the body's wing axis (row 3 of body -> NED, column 2)
  local function proxy_of(a)
    local _, _, _, _, _, _, _, r21 = PL.rot(a.qw, a.qx, a.qy, a.qz)
    return deg(asin(clip(r21, -1.0, 1.0)))
  end
  -- upside down: the canopy's angle below the horizon, asin of world down
  -- along the body's canopy axis (-z: row 3 of body -> NED, column 3, negated)
  local function canopy_down_of(a)
    local _, _, _, _, _, _, _, _, r22 = PL.rot(a.qw, a.qx, a.qy, a.qz)
    return deg(asin(clip(-r22, -1.0, 1.0)))
  end

  -- count_sides: also count how often the lift side changes (STATE2 att_side)
  local function chase(lat_d, lon_d, bank_d, count_sides)
    PL.start(ac, V0, THROTTLE_PCT / 100.0, rad(bank_d))
    local la, lo = rad(lat_d), rad(lon_d)
    local bn, be, bd = cos(la) * cos(lo), cos(la) * sin(lo), -sin(la)
    local r = { lat = lat_d, lon = lon_d, bank0 = bank_d, bank_over = -math.huge,
                upside = -math.huge, head_dev = 0.0, tilt_back = 0.0, t_align = nil,
                eps_min = math.huge, side = nil, side_switches = 0 }
    -- ahead of the wing line, cos(lat) * cos(lon) > 0; on it, square to the side
    r.ahead = math.abs(lat_d) < 90 and math.abs(lon_d) < 90
    r.square = not r.ahead and (math.abs(lat_d) == 90 or math.abs(lon_d) == 90)
    -- the chase starts from rest: a GUIDE with tracking off ends whatever came before
    PL.to_stubs(ac, stubs.sim, srv_scaled)
    stubs.feed(pack_guide(next_seq(), 0x01, 4, 0.0, 0.0, THROTTLE_PCT))
    stubs.advance(DT_MS); update()
    local xn, xe, xd = PL.path_frame(ac)
    local p0 = proxy_of(ac)                   -- the starting bank proxy
    -- behind: more than a right angle from the path at the start (exactly
    -- square, read through rounding, is not)
    r.behind = xn * bn + xe * be + xd * bd < -BEHIND_EPS
    local tilt_dir = (-bd >= -xd) and 1.0 or -1.0
    local band_lo, band_hi = math.min(-lim, p0), math.max(lim, p0)
    local run, peak, hd_n, hd_e = 0, -math.huge, nil, nil
    for k = 1, ticks do
      PL.to_stubs(ac, stubs.sim, srv_scaled)
      xn, xe, xd = PL.path_frame(ac)
      local theta = deg(acos(clip(xn * bn + xe * be + xd * bd, -1.0, 1.0)))
      r.eps_min = math.min(r.eps_min, theta)
      local proxy = proxy_of(ac)
      local tilt = deg(asin(clip(-xd, -1.0, 1.0)))
      local hn = sqrt(xn * xn + xe * xe)
      r.bank_over = math.max(r.bank_over, math.max(band_lo - proxy, proxy - band_hi))
      r.upside = math.max(r.upside, canopy_down_of(ac))
      local td = tilt_dir * tilt
      if td > peak then peak = td end
      r.tilt_back = math.max(r.tilt_back, peak - td)
      if hd_n == nil and theta < C3.CURVE_TURN_DONE_DEG then
        hd_n, hd_e, r.t_done = xn, xe, (k - 1) * DT_MS / 1000
      end
      if hd_n ~= nil then
        local he = sqrt(hd_n * hd_n + hd_e * hd_e)
        local dev = deg(acos(clip((xn * hd_n + xe * hd_e) / math.max(hn * he, 1e-12), -1.0, 1.0)))
        r.head_dev = math.max(r.head_dev, dev * clip(hn / pole_sin, 0.0, 1.0))
      end
      run = (theta < C3.ALIGN_TOL_DEG) and run + 1 or 0
      if run >= hold_ticks and r.t_align == nil then r.t_align = (k - hold_ticks) * DT_MS / 1000 end
      stubs.feed(pack_oracle(next_seq(), bn, be, bd))
      stubs.advance(DT_MS); update()
      local cmd = stubs.log.rate_cmds[#stubs.log.rate_cmds]
      local thr, p_c, q_c = THROTTLE_PCT, 0.0, 0.0
      if stubs.sim.nav_owned and cmd then thr, p_c, q_c = cmd[1], cmd[2], cmd[3] end
      PL.fly(ac, DT_MS, thr, p_c, q_c)
      if count_sides then
        local sd = last_state2().att_side
        if sd == sd then
          if r.side and sd ~= r.side then r.side_switches = r.side_switches + 1 end
          r.side = sd
        end
      end
    end
    return r
  end

  local all, t0 = {}, os.clock()
  for _, bank in ipairs(BANKS_DEG) do
    for _, lat in ipairs(LATS_DEG) do
      local lons = (math.abs(lat) == 90) and { 0 } or LONS_DEG
      for _, lon in ipairs(lons) do all[#all + 1] = chase(lat, lon, bank) end
    end
  end
  local pball = chase(C3.POINT_LAT_DEG, C3.POINT_LON_DEG, 0)
  local logc = chase(GAP_LAT_DEG, GAP_LON_DEG, 0, true)
  local secs = os.clock() - t0

  local missed, t_sum, t_max, n_al = {}, 0.0, 0.0, 0
  for _, r in ipairs(all) do
    if r.t_align then
      n_al = n_al + 1
      t_sum, t_max = t_sum + r.t_align, math.max(t_max, r.t_align)
    else
      missed[#missed + 1] = string.format("lat %+d lon %+d bank %+d: never held, %.2f deg at the closest",
                                          r.lat, r.lon, r.bank0, r.eps_min)
    end
  end
  check(string.format("every chase lines up (%d chases over the sphere, wings level and "
                      .. "banked %d either way)", #all, RULE_BANK_DEG),
        #missed == 0,
        string.format("%d lined up, after %.2f s on average and %.2f s at the latest (%.0f s to fly); %s",
                      n_al, n_al > 0 and t_sum / n_al or 0, t_max, secs,
                      table.concat(missed, " | ", 1, math.min(#missed, 12))))
  -- the rule: every chase but a balloon behind, and the default p balloon
  local bad, judged = {}, 0
  local function judge(r, name)
    judged = judged + 1
    local why = {}
    if r.bank_over > 0.0 then
      why[#why + 1] = string.format("bank proxy %.2f past the band", r.bank_over)
    end
    if r.t_done == nil then why[#why + 1] = "the turn never completes"
    elseif r.head_dev > C3.CURVE_TURN_DONE_DEG then
      why[#why + 1] = string.format("heading %.1f off the turn end's", r.head_dev)
    end
    if r.tilt_back > C3.TILT_BACK_TRAVEL_DEG then
      why[#why + 1] = string.format("tilt back %.1f", r.tilt_back)
    end
    if r.upside > 0.0 then
      why[#why + 1] = string.format("upside down, the canopy %.1f below the horizon", r.upside)
    end
    if #why > 0 then bad[#bad + 1] = name .. ": " .. table.concat(why, ", ") end
  end
  for _, r in ipairs(all) do
    if not r.behind then
      judge(r, string.format("lat %+d lon %+d bank %+d", r.lat, r.lon, r.bank0))
    end
  end
  judge(pball, "the default p balloon")
  check(string.format("the rule: every chase but a balloon behind, and the default p balloon "
                      .. "(%d chases): no Suboptimal Curve", judged),
        #bad == 0,
        string.format("%d with one; %s", #bad, table.concat(bad, " | ", 1, math.min(#bad, 12))))
  -- past the band, by where the balloon starts: ahead of the wing line,
  -- square to the side, behind
  local past = { ahead = { n = 0, past = 0, most = 0.0 }, square = { n = 0, past = 0, most = 0.0 },
                 behind = { n = 0, past = 0, most = 0.0 } }
  for _, r in ipairs(all) do
    local p = past[r.ahead and "ahead" or (r.square and "square" or "behind")]
    p.n = p.n + 1
    if r.bank_over > 0.0 then p.past = p.past + 1 end
    p.most = math.max(p.most, r.bank_over)
  end
  local function past_txt(p) return string.format("%d of %d past it, %.2f deg at most", p.past, p.n, p.most) end
  print(string.format("  [INFO] the flown bank proxy past the band, not judged   balloon ahead: %s; "
                      .. "square to the side: %s; behind: %s", past_txt(past.ahead),
                      past_txt(past.square), past_txt(past.behind)))
  check(string.format("the default p balloon (%g, %g) from level lines up",
                      C3.POINT_LAT_DEG, C3.POINT_LON_DEG),
        pball.t_align ~= nil,
        pball.t_align and string.format("after %.2f s", pball.t_align)
        or string.format("never held, %.2f deg at the closest", pball.eps_min))
  check(string.format("a balloon right and below (%g, %g) from level, its force line between "
                      .. "the band's two ends, keeps its lift side and lines up", GAP_LAT_DEG,
                      GAP_LON_DEG),
        logc.side_switches <= 1 and logc.t_align ~= nil,
        string.format("%d side switches, %s", logc.side_switches,
                      logc.t_align and string.format("lined up after %.2f s", logc.t_align)
                      or string.format("never held, %.2f deg at the closest", logc.eps_min)))

  pv.PTCH_RATE_FF, pv.PTCH_RATE_P = saved_ff, saved_p
  stubs.sim.quat = nil
  stubs.feed(pack_gain(GAIN_IDX.steering_law, C3.STEERING_LAW)); stubs.advance(DT_MS); update()
end)()

print("\n" .. string.rep("=", 74))
if #fails > 0 then
  print(string.format("FC SHELL FAILED: %d of %d - %s", #fails, n,
                      table.concat(fails, ", ")))
  os.exit(1)
end
print(string.format("FC SHELL PASSED - %d checks", n))
print(string.rep("=", 74))
