-- Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
--
-- balloon_chaser_fc.lua - ArduPilot Lua script: the FC-side shell.
--
-- Wires the steering core (geometry, estimator, suppression, the roll and
-- pitch cascades, track, cascade) to real ArduPilot bindings.  The core is
-- pure and runs on the desktop under the stubs; this file is the part that
-- needs the real bindings, so it is kept as thin as possible.
--
-- ============================================================================
-- OPERATING ASSUMPTIONS - READ BEFORE FLYING HARDWARE
-- ============================================================================
-- This script is configured for SIMULATION.  Read both points before flying
-- a real airframe:
--
--   1. COMPANION STALENESS.  ArduPilot's own nav_scripting timeout watches
--      whether the script is calling, not whether its inputs are fresh, so it
--      never fires while this script runs.  The script therefore checks GUIDE
--      itself: with no NEW frame (a seq it has not seen) for GUIDE_STALE_MS it
--      releases control (gate 7).  After the release ACRO flies the sticks -
--      the transmitter's, or neutral with none - until something changes mode.
--      A Lua error, or an input the law was flying on disappearing, releases
--      control the same way and latches a fault the companion halts on.
--
--   2. RECOVERY DEPENDS ENTIRELY ON A MODE CHANGE.  Mode::enter() clears
--      nav_scripting.enabled, and this script only ever re-claims in ACRO - so
--      ANY other mode, whether selected by the operator or forced by a
--      fence/battery/GCS failsafe, takes the surfaces back and keeps them.
--      (Claiming with the live mode instead would hijack LOITER, which
--      nav_scripting_enable also accepts - and LOITER is what this project
--      switches to when a test aborts.)  But with no transmitter and no safety
--      pilot, nothing initiates that mode change unless a failsafe fires.
--
-- Before any hardware flight: set FENCE_ENABLE with an altitude ceiling and a
-- radius.
--
-- ============================================================================
-- THE SPEED SCALER IS OBSERVED, NEVER OVERWRITTEN
-- ============================================================================
-- ArduPlane scales the rate PID by the ratio SCALING_SPEED / airspeed, so the
-- inner-loop gain moves as the aircraft speeds up and slows down.  That is not
-- interference to be removed: control surfaces genuinely bite harder at speed,
-- and the compensation is part of the plant this steering commands.
--
-- Never pin it by writing SCALING_SPEED = airspeed every tick.  That forces the
-- ratio to 1 and yields stable numbers for an aircraft that does not exist:
-- gains identified that way are wrong the moment the scaler works again, and
-- the low-speed compensation disappears exactly where it is needed.  A
-- parameter cannot be borrowed as a control input.
--
-- Both terms of the ratio are readable, so the scaler is COMPUTED and reported
-- in the STATE frame as lua_scaler.  Identification runs record the gain the FC
-- actually applied on each tick and account for it; they do not suppress it.
--
-- ============================================================================
-- ALLOCATION
-- ============================================================================
-- The control law allocates nothing per tick: its tables are built at load and
-- filled in place.  Garbage collection in the tick surfaces as "exceeded time
-- limit", a symptom with no visible connection to its cause.  What does
-- allocate is at the edges: each binding call that returns a Vector3f or
-- Quaternion (ahrs:get_*) makes a new one, and reading the port
-- (rx_pump) and packing STATE and STATE2 (string.pack) each make a string.
--
-- ArduPilot's sandbox has no collectgarbage, so nothing on the FC can see an
-- allocation.  test_balloon_chaser_fc.lua measures the geometry and the law on the
-- desktop with the collector stopped and fails if they allocate.

local proto  = require("protocol")
local C      = require("constants")
local T      = require("track")
local K      = require("cascade")
local S      = require("cascade_state")
local Est    = require("estimator")
local G      = require("geometry")
local E      = require("excite")
local CP     = require("control_pitch")
local CF     = require("curve_flyer")
local PA     = require("path_attitude")

-- ---------------------------------------------------------------------------
-- configuration
-- ---------------------------------------------------------------------------
local LOOP_HZ        = 50           -- steering rate; see ZOH note in the plan
local LOOP_MS        = math.floor(1000 / LOOP_HZ)
local SERIAL_INSTANCE = 0           -- SERIALx_PROTOCOL = 28 (Scripting)
-- Send STATE EVERY tick.  The companion writes a CSV row per tick, so any
-- divisor > 1 makes every lua_* column a repeated value.
local STATE_DIVISOR  = 1
local RX_BUF_MAX     = 512

-- ---------------------------------------------------------------------------
-- preallocated state
-- ---------------------------------------------------------------------------
local s = {
  vel_marker_rad = 0.0, lat_marker_rad = 0.0, lat_marker_valid = false,
  path_from_vel = false, path_deg = 0.0,
  balloon_path_x = 1.0, balloon_path_y = 0.0, balloon_path_z = 0.0,
  requested_roll_rad = 0.0, requested_pitch_rad = 0.0,
  log_upright = 1.0, chase_key_prev = -1,
  log_pitch_turn_dps = 0.0,
  log_pitch_trim_elev = 0.0, log_pitch_trim_dps = 0.0,
  yaw_point_i_dps = 0.0,
  yaw_coord_dps = 0.0, coord_valid = false,
  accel_valid = false,
  log_rud_ovr = 0,
  -- the path frame in body axes, its speed and world down in body axes, as
  -- track.lua leaves them for Lift Vector Pointing
  path_bx = 1.0, path_by = 0.0, path_bz = 0.0, path_yx = 0.0, path_yy = 1.0,
  path_zx = 0.0, path_zy = 0.0, path_zz = 1.0, path_speed = 0.0,
  down_bx = 0.0, down_by = 0.0, down_bz = 1.0,
}
local cs   = S.new_cascade()
local est  = Est.new()
local exc  = E.new()
local g    = {}          -- ControlGains, filled from constants then SET_GAIN
local cfg  = {}          -- per-tick inputs
local out  = {}          -- per-tick diagnostics
local cf   = CF.new()    -- the Curve Flyer and the PLAN it flies
local pa   = PA.new()    -- Lift Vector Pointing (steering_law 1)
local guide, set_gain, excite_msg = {}, {}, {}
local st_msg = {}
-- STATE2: the rest of the mirror, sent right after STATE (see protocol.py).
local st2 = {}
local NAN = 0/0

local scratch_world = Vector3f()
local scratch_body  = Vector3f()
local quat          = Quaternion()

local port = nil
local rx = ""            -- NOTE: the one unavoidable allocation; see rx_pump
local last_ms = 0
local tick = 0
local have_guide = false
-- The link's health, in one table (the 100-local limit, as GATE):
--   guide_seq, guide_ms  the newest GUIDE seq and the FC time it arrived; the
--                        staleness check stands down when no frame with a NEW
--                        seq has come for GUIDE_STALE_MS
--   rejected             set once a frame from the companion is rejected for
--                        an id or a length this build does not expect, and
--                        kept for the session.  STATE carries it (flags bit 4)
--                        and the companion halts on it: the two ends run
--                        different builds of the protocol
--   fault                what took control away mid-chase, latched in STATE
--                        flags bits 5-7 until GUIDE says tracking has stopped;
--                        the script claims nothing while it is set.  Both
--                        latches clear when a new session's first GUIDE
--                        arrives after a stale gap (rx_pump).  0 none,
--                        proto.FAULT_SCRIPT_ERROR a Lua error, otherwise the
--                        gate (GATE.MOUNT, GATE.QUAT, GATE.EST) whose input
--                        disappeared.  protocol.FAULT_NAMES reads it.
local link = { guide_seq = nil, guide_ms = 0, rejected = false, fault = 0 }
-- The messages this script accepts from the companion.
local INBOUND = { [proto.MSG_GUIDE] = true, [proto.MSG_SET_GAIN] = true,
                  [proto.MSG_EXCITE] = true, [proto.MSG_PLAN] = true }
local scaling_param = nil
local aspd_min_param, aspd_max_param = nil, nil
-- ArduPlane floors AIRSPEED_MIN at this before sizing the scaler's ceiling
-- (defines.h MIN_AIRSPEED_MIN; Attitude.cpp calc_speed_scaler).  It is not a
-- parameter, so it is copied here.
local MIN_AIRSPEED_MIN = 5.0
-- ArduPlane's mode number for ACRO, the only mode the script claims control in.
local MODE_ACRO = 4
-- The smallest rate-loop conversion (elevator units per deg/s) the pitch trim
-- is sent through.  Below it there is no FF, P or airspeed to carry a trim,
-- and dividing by it would turn the trim into an unbounded rate request.
local TRIM_CONV_MIN = 1e-6
local nav_owned = false          -- does the script currently own the surfaces?
local nav_warned = false

-- Every early return in steering() is a REASON the aircraft is not being
-- flown, and without a name for it the symptom for all of them is identical:
-- the script appears healthy and ACRO flies with neutral sticks.  The reason
-- goes out in every STATE frame as `gate` (protocol.GATE_NAMES).
-- One table, not a local per gate: ArduPilot's Lua allows 100 locals in a
-- function, the main chunk included (see LUA_MAX_LOCALS in test_no_globals.sh).
local GATE = { OK = 0, GUIDE = 1, MOUNT = 2, QUAT = 3, EST = 4, TRACK = 5,
               NAV = 6, STALE = 7, FAULT = 8 }
local last_gate = GATE.GUIDE
-- True once the law has run this tick (gates 0 and 6).  Its internals in STATE
-- are NaN on every other tick, never the value an earlier tick left behind.
local law_ran = false
local last_report_ms = 0
local last_param_ms = 0
local cached_acro_roll, cached_acro_pitch = 180.0, 90.0
-- Seeded with the flown value, like its roll and pitch siblings above: 0 is
-- not a neutral default here, it is the value that clamps every yaw command to
-- zero.
local cached_acro_yaw = 45.0
-- Floor for the 1/V in the coordination feedforward, m/s.  Seeded from the
-- flown AIRSPEED_MIN so a param read that fails at boot still floors at a sane
-- speed rather than at zero, which would divide.
local cached_aspd_min = 12.0
-- The pitch rate loop's FF and P, read EVERY tick for the pitch trim's rate
-- conversion.  The companion writes a different rate-loop set on each mode
-- switch, so a value cached across ticks would convert the trim with the
-- previous mode's gains for as long as the cache lives.  A bound Parameter's
-- get() is a pointer read, not a name lookup.  Unbound or unreadable, the
-- conversion is 0 and no trim is sent, rather than a trim sized for gains the
-- FC may not have.
local ptch_ff_param, ptch_p_param = nil, nil
-- The airspeed-hold throttle loop's state (control_pitch.compute_throttle_command).
-- Cleared by step() on every tick its throttle is not flown, so each
-- engagement re-seeds from the operator's throttle.
local thr_state = S.new_throttle()

-- Sensor freshness.  Counts how often each value actually CHANGED, so the
-- effective rate reaching the control law is measured, not assumed.
local fresh = { quat=0, gyro=0, vel=0, asp=0 }
local hz    = { quat=0, gyro=0, vel=0, asp=0 }
local prev_qw, prev_gx, prev_vn, prev_asp = 0/0, 0/0, 0/0, 0/0
-- This tick's clock, published for step().  steering() keeps it local for
-- speed; step() needs it to stamp STATE and cannot see a local.
local now_ms_cur = 0

-- Gimbal-pose clock offset.  The companion sends a Gazebo sim-time stamp; this
-- script has ArduPilot's millis().  Both advance with simulated time but start
-- from different epochs (SITL accumulates from deltas, SIM_JSON.cpp:514), so
-- the difference contains a constant unknown.
--
-- The running MINIMUM of (millis() - stamp) is that constant: the smallest
-- difference seen corresponds to the freshest pose observed, so anything above
-- it is genuine age.  Standard one-way clock-offset estimation, and it needs no
-- calibration step.
local pose_offset_ms = nil
local gimbal_sim_age_ms = 0

-- ---------------------------------------------------------------------------
-- manual surface override (debug / plant ID)
-- ---------------------------------------------------------------------------
-- Resolved lazily and cached: SRV_Channels:find_channel() maps a SERVOn_FUNCTION
-- to the channel carrying it, so nothing here hardcodes which output a surface
-- is wired to.  All three lookups can legitimately fail - a conventional tail
-- has no v-tail functions, this airframe has no elevator or rudder function -
-- so a missing channel is skipped rather than treated as an error.
local K_AILERON, K_VTAIL_L, K_VTAIL_R = 4, 79, 80
local K_ELEVATOR, K_RUDDER, K_THROTTLE = 19, 21, 70
local ovr_ready, ovr_ch, ovr_mix = nil, {}, 0.5
local ovr_tail_hold = nil   -- captured on the start edge, see ovr_write_surfaces

local function ovr_chan(fn)
  -- channel index (0-based, as set_output_pwm_chan_timeout wants) plus that
  -- channel's own trim/min/max, so percent maps through the SAME endpoints
  -- ArduPlane uses rather than an assumed 1000/1500/2000.
  if not SRV_Channels.find_channel then return nil end
  local c = SRV_Channels:find_channel(fn)
  if not c then return nil end
  local n = c + 1
  return { c = c, fn = fn,
           trim = param:get('SERVO' .. n .. '_TRIM') or 1500,
           lo   = param:get('SERVO' .. n .. '_MIN')  or 1000,
           hi   = param:get('SERVO' .. n .. '_MAX')  or 2000 }
end

local function ovr_resolve()
  if ovr_ready ~= nil then return ovr_ready end
  if SRV_Channels == nil then
    gcs:send_text(3, "BALLOON CHASER: no SRV_Channels binding - surface override unavailable")
    ovr_ready = false
    return false
  end
  ovr_ch.ail = ovr_chan(K_AILERON)
  ovr_ch.vl  = ovr_chan(K_VTAIL_L)
  ovr_ch.vr  = ovr_chan(K_VTAIL_R)
  ovr_mix    = param:get('MIXING_GAIN') or 0.5
  if ovr_mix < 0.05 then ovr_mix = 0.05 end       -- it is a divisor below
  ovr_ready  = (ovr_ch.ail ~= nil) or (ovr_ch.vl ~= nil and ovr_ch.vr ~= nil)
  gcs:send_text(ovr_ready and 6 or 3, string.format(
    "BALLOON CHASER: surface override ail=%s vtailL=%s vtailR=%s mix=%.2f",
    ovr_ch.ail and ovr_ch.ail.c or "none",
    ovr_ch.vl  and ovr_ch.vl.c  or "none",
    ovr_ch.vr  and ovr_ch.vr.c  or "none", ovr_mix))
  return ovr_ready
end

local function ovr_pct(ch, pwm)   -- pwm -> percent, inverse of ovr_set
  if pwm >= ch.trim then
    local span = ch.hi - ch.trim
    return (span > 1) and ((pwm - ch.trim) * 100.0 / span) or 0.0
  end
  local span = ch.trim - ch.lo
  return (span > 1) and ((pwm - ch.trim) * 100.0 / span) or 0.0
end

local function ovr_set(ch, pct)
  if ch == nil then return end
  if pct > 100.0 then pct = 100.0 elseif pct < -100.0 then pct = -100.0 end
  -- clamped to the channel's OWN endpoints: past them the servo cannot go and
  -- the extra command would only make the log disagree with the surface.
  local pwm = (pct >= 0.0) and (ch.trim + (ch.hi - ch.trim) * pct / 100.0)
                            or (ch.trim + (ch.trim - ch.lo) * pct / 100.0)
  -- 200 ms, refreshed every tick.  If this script stops the surfaces go back to
  -- ArduPlane on their own within 200 ms - that is the failsafe.
  SRV_Channels:set_output_pwm_chan_timeout(ch.c, math.floor(pwm + 0.5), 200)
end

-- PER-SURFACE OVERRIDE.  Each surface has its own switch, and a surface that is
-- not overridden is not written at all.
--
-- The aileron is clean: its own channel, so overriding it leaves pitch and yaw
-- entirely with ArduPlane.
--
-- The tail is not.  A V-tail has no elevator channel and no rudder channel -
-- both are mixed onto the two ruddervators - so overriding EITHER one means
-- this script must write BOTH channels, and it therefore has to supply a value
-- for the axis that is not being overridden.
--
-- It takes that value from the aircraft, once, on the tick the tail override
-- switches on: read both ruddervator PWMs BEFORE anything is written and invert the
-- mix to recover ArduPlane's own elevator and rudder demand.  Reading before
-- writing is the whole point - once set_output_pwm_chan_timeout is active,
-- get_output_pwm reports the override back, so a live read would latch onto our
-- own output and drift.
--
-- So the un-overridden tail axis is HELD at the value it had when you flipped
-- the switch, not tracked live.  For elevator that means the pitch trim you
-- were flying at, which is a sane place to sit for a short test - but it is a
-- hold, not stabilisation, and pitch will wander as speed changes.
local function ovr_write_surfaces(ail_on, ail_pct, elev_on, elev_pct,
                                  rud_on, rud_pct)
  if not ovr_resolve() then return end
  if ail_on then ovr_set(ovr_ch.ail, ail_pct) end

  if not (elev_on or rud_on) then
    ovr_tail_hold = nil      -- re-capture next time the tail is taken
    return
  end
  if ovr_ch.vl == nil or ovr_ch.vr == nil then return end

  if ovr_tail_hold == nil then
    local lp, rp = 0.0, 0.0
    if SRV_Channels.get_output_pwm then
      local l = SRV_Channels:get_output_pwm(K_VTAIL_L)
      local r = SRV_Channels:get_output_pwm(K_VTAIL_R)
      if l then lp = ovr_pct(ovr_ch.vl, l) end
      if r then rp = ovr_pct(ovr_ch.vr, r) end
    end
    -- invert  vr = (e-u)*mix,  vl = (e+u)*mix
    ovr_tail_hold = { elev = (rp + lp) / (2.0 * ovr_mix),
                      rud  = (lp - rp) / (2.0 * ovr_mix) }
    gcs:send_text(6, string.format(
      "BALLOON CHASER: tail override active, holding elev %.1f%% rud %.1f%%",
      ovr_tail_hold.elev, ovr_tail_hold.rud))
  end

  local e = elev_on and elev_pct or ovr_tail_hold.elev
  local u = rud_on  and rud_pct  or ovr_tail_hold.rud
  -- V-TAIL MIX, matching ArduPlane servos.cpp channel_function_mixer(rudder,
  -- elevator, vtail_right, vtail_left).  Applied here because writing the
  -- channels bypasses that mixer.  Keeping MIXING_GAIN in means a given rudder
  -- percent produces the same surface it did under the law, so every deflection
  -- measured so far stays comparable.
  ovr_set(ovr_ch.vr, (e - u) * ovr_mix)
  ovr_set(ovr_ch.vl, (e + u) * ovr_mix)
end

-- ---------------------------------------------------------------------------
-- gains: constants first, SET_GAIN overrides live
-- ---------------------------------------------------------------------------
local GAIN_ORDER = proto.GAIN_NAMES     -- generated; index -> field name

local function load_default_gains()
  for i = 0, 200 do
    local name = GAIN_ORDER[i]
    if name then
      local up = string.upper(name)
      if C[up] ~= nil then g[name] = C[up] end
    end
  end
end

-- ---------------------------------------------------------------------------
-- serial ingest
-- ---------------------------------------------------------------------------
local function rx_pump(now_ms)
  if not port then return end
  -- available() is declared `uint32_t` in bindings.desc, and ArduPilot returns
  -- uint32_t as a USERDATA object, not a Lua number - because a Lua 5.3 integer
  -- cannot represent the full unsigned range.  It has to be converted before it
  -- can be compared or passed on:
  --     bad argument #1 to 'readstring' (number expected, got uint32_t)
  local n = port:available()
  if n == nil then return end
  n = n:toint()
  if n <= 0 then return end
  -- Reading bytes into a Lua string allocates; unavoidable without a ring
  -- buffer binding.  ONE binding call per tick, of at most RX_BUF_MAX bytes:
  -- a call per byte cannot keep up with the frames at 50 Hz (no GUIDE ever
  -- completes, and every tick reports noguide), and a backlog stays in the
  -- port for the next tick rather than growing this buffer.
  --
  -- NEVER trim `rx` before parsing it.  What the last tick left in it is the
  -- head of a frame whose tail is at the front of this chunk; with a backlog
  -- the chunk is a full RX_BUF_MAX, and cutting the buffer back to that size
  -- drops exactly that head, so the frame - a gain edit as easily as a GUIDE -
  -- is lost without a trace.  The buffer is bounded without trimming: the
  -- parser below consumes every complete frame and skips garbage byte by byte,
  -- so what it leaves is less than one frame (4 + 255 + 2 bytes).
  if n > RX_BUF_MAX then n = RX_BUF_MAX end
  local chunk = port:readstring(n)
  if chunk == nil or #chunk == 0 then return end
  rx = rx .. chunk

  -- frame: magic(2) msgid(1) len(1) payload crc(2)
  local p = 1
  while #rx - p + 1 >= 6 do
    local magic = string.byte(rx, p) | (string.byte(rx, p + 1) << 8)
    if magic ~= proto.MAGIC then
      p = p + 1                       -- resync one byte at a time
    else
      local msgid = string.byte(rx, p + 2)
      local plen  = string.byte(rx, p + 3)
      local total = 4 + plen + 2
      if #rx - p + 1 < total then break end
      local want = proto.crc16(rx, p, p + 4 + plen - 1)
      local got  = string.byte(rx, p + 4 + plen)
                 | (string.byte(rx, p + 4 + plen + 1) << 8)
      if want ~= got then
        p = p + 1
      elseif not INBOUND[msgid] or plen ~= proto.PAYLOAD_SIZE[msgid] then
        -- Whole and checksummed, but not a frame this build reads: another
        -- build of the protocol wrote it.  Decoding it would read every field
        -- at the wrong offset, so it is stepped over and reported.
        if not link.rejected then
          gcs:send_text(3, string.format(
            "balloon chaser: rejected msg %d of %d bytes - companion and FC "
            .. "run different protocol builds", msgid, plen))
        end
        link.rejected = true
        p = p + total
      else
        local q = p + 4
        if msgid == proto.MSG_GUIDE then
          proto.parse_guide(rx, q, guide)
          have_guide = true
          if guide.seq ~= link.guide_seq then
            -- The first new frame after a stale gap opens a new session (the
            -- script outlives the program): the latches were the previous
            -- session's to halt on, and this one starts clean.
            if now_ms - link.guide_ms > C.GUIDE_STALE_MS then
              link.fault = 0
              link.rejected = false
            end
            link.guide_seq = guide.seq
            link.guide_ms = now_ms
          end
        elseif msgid == proto.MSG_SET_GAIN then
          proto.parse_set_gain(rx, q, set_gain)
          local name = GAIN_ORDER[set_gain.index]
          if name then g[name] = set_gain.value end
        elseif msgid == proto.MSG_EXCITE then
          proto.parse_excite(rx, q, excite_msg)
          E.start(exc, excite_msg.kind, excite_msg.axis,
                  excite_msg.replace ~= 0, excite_msg.amp_dps,
                  excite_msg.f0_hz, excite_msg.f1_hz, excite_msg.dur_ms, now_ms)
        elseif msgid == proto.MSG_PLAN then
          proto.parse_plan(rx, q, cf.msg)
          CF.accept(cf, ((g.steering_law or 0) >= 0.5) and 1 or 0)
        end
        p = p + total
      end
    end
  end
  if p > 1 then rx = string.sub(rx, p) end
end

-- ---------------------------------------------------------------------------
-- the tick
-- ---------------------------------------------------------------------------
-- THE COMMAND FIELDS, CLEARED AT ENTRY.
--
-- st_msg persists between ticks, so any field the commanded path writes keeps
-- its last value when steering() returns early at a gate.  Uncleared, a gated
-- tick reports the previous tick's yaw command, and it reads as a live
-- measurement: lua_gate says otherwise, but only to a reader who thinks to
-- check it.
--
-- These are the fields only a COMMANDED tick (gate 0) writes.  Cleared to NaN
-- at entry, a gated tick reports them blank, and the commanded path overwrites
-- them further down.  asp_flags is an integer field and cannot carry NaN; the
-- companion blanks it on any tick whose gate is not 0.  The law's own
-- internals are published in step() against law_ran.  Sensor fields are NOT
-- cleared: they come from sample_sensors_and_report_scaler, which runs before
-- this and does not gate, so they are live on every tick.
local function clear_command_fields()
  law_ran = false
  s.log_rud_ovr = 0
  st_msg.pan_err_deg       = NAN
  st_msg.yaw_rate_cmd_dps  = NAN
  st_msg.yaw_point_dps     = NAN
  st_msg.yaw_ff_ari_dps    = NAN
  st_msg.yaw_coordination_dps = NAN
  st_msg.pitch_turn_dps    = NAN
  st_msg.pitch_trim_elev   = NAN
  st_msg.pitch_trim_dps    = NAN
  st_msg.pitch_trim_lift_term = NAN
  st_msg.omega_x_cmd_dps   = NAN
  st_msg.omega_y_cmd_dps   = NAN
  st_msg.throttle_out      = NAN
  st_msg.asp_flags         = 0
  st_msg.asp_err_mps       = NAN
  st_msg.asp_i_term        = NAN
  st_msg.world_est_trust   = NAN
  st_msg.excite_value      = NAN
  -- STATE2's law internals: NaN, written blank, until the law runs this tick.
  st2.balloon_body_x, st2.balloon_body_y, st2.balloon_body_z = NAN, NAN, NAN
  st2.los_elev_deg = NAN
  st2.bank_rate_cmd_dps, st2.bank_rate_coupling_dps = NAN, NAN
  st2.sideslip_deg, st2.aoa_deg = NAN, NAN
  st2.yaw_point_i_dps = NAN
  st2.plane_off_nose_deg, st2.plane_err_deg = NAN, NAN
  st2.plane_side, st2.plane_axis_up = NAN, NAN
  st2.plane_drift_dps, st2.plane_ff_dps = NAN, NAN
  st2.plane_err_rate_dps, st2.plane_d_dps = NAN, NAN
  st2.plane_path_share, st2.plane_i_dps, st2.plane_i_frozen = NAN, NAN, NAN
  st2.plane_path_deg, st2.plane_path_from_vel = NAN, NAN
  st2.path_turn_up_dps, st2.path_turn_right_dps = NAN, NAN
end

--- The law ran but its command is not flown: every integrator is inactive and
--- is cleared, and the integrator columns say so this tick.
local function clear_unflown_integrators()
  S.reset_integrators(cs)
  s.yaw_point_i_dps = 0.0
  if pa.ran then
    PA.clear_integrators(pa)
  else
    st2.plane_i_dps = 0.0
  end
end

--- An input the law was flying on is gone: hand the aircraft back NOW.
--- Returning without the release leaves ArduPlane flying the last rates and
--- throttle until its own 1000 ms nav_scripting timeout.  When the script
--- owned the aircraft this is a fault, not a pause: the gate is latched in
--- link.fault and the companion halts the program on it.  `expected` is a
--- stop that legitimately follows a fresh chase (the estimator's first fix):
--- it releases without the fault.
local function drop_control(gate, why, expected)
  if nav_owned then
    vehicle:nav_scripting_enable(255)
    nav_owned = false
    if expected then
      gcs:send_text(4, "balloon chaser: released control - " .. why)
    else
      if link.fault == 0 then link.fault = gate end
      gcs:send_text(3, "balloon chaser: FAULT, released control - " .. why)
    end
  end
  last_gate = gate
end

--- A Lua error is a bug: release the aircraft, latch proto.FAULT_SCRIPT_ERROR for
--- the companion to halt on, and print the error (file and line) once.
local function script_fault(err)
  if nav_owned then
    vehicle:nav_scripting_enable(255)
    nav_owned = false
  end
  if link.fault == 0 then
    link.fault = proto.FAULT_SCRIPT_ERROR
    gcs:send_text(3, "balloon chaser: FAULT, script error - " .. tostring(err))
  end
  law_ran = false
  PA.clear_log(pa)
end

local function steering()
  local now_ms = millis():toint()
  -- Whether the law ran on the previous tick, read before the entry clear.
  local law_ran_before = law_ran
  clear_command_fields()
  -- The Curve Flyer runs a PLAN this tick only if it says so below; a gated
  -- tick leaves its internals blank.
  cf.active, cf.plan = false, false
  CF.clear_log(cf)
  PA.clear_log(pa)
  now_ms_cur = now_ms
  -- Measured dt, never the nominal.  The scripting scheduler is cooperative and
  -- explicitly not guaranteed to hold a period.
  local gap_ms = (last_ms == 0) and LOOP_MS or (now_ms - last_ms)
  local dt = gap_ms * 0.001
  if dt <= 0.0 then dt = LOOP_MS * 0.001 end
  if dt > 0.5 then dt = 0.5 end          -- a long stall must not spike the D term
  last_ms = now_ms
  -- The interval as measured, before the clamp above: a stall shows at its
  -- real length.
  st_msg.dt_ms = math.max(0, math.min(65535, gap_ms))
  tick = tick + 1

  -- NOTHING ROUTINE IS PRINTED.  Both the gate outcome and the sensor rates are
  -- already in the flight record, per tick and at full resolution:
  --
  --   lua_gate    which gate stopped this tick, or 0 for commanded.  A per-tick
  --               value at full resolution, not a once-a-second tally.
  --   hz_quat/gyro/vel/asp   how often each reading actually CHANGED in the
  --               last second - the only way to tell a live feed from the same
  --               stale value read fifty times.
  --
  -- A console line repeating once a second is read for a minute and then
  -- ignored; the CSV is read when something has gone wrong.  Faults still print
  -- (severity 3), and those are not routine.
  if now_ms - last_report_ms >= 1000 then
    last_report_ms = now_ms
    hz.quat, hz.gyro = math.min(255, fresh.quat), math.min(255, fresh.gyro)
    hz.vel,  hz.asp  = math.min(255, fresh.vel),  math.min(255, fresh.asp)
    fresh.quat, fresh.gyro, fresh.vel, fresh.asp = 0, 0, 0, 0
  end

  rx_pump(now_ms)
  if not have_guide then last_gate = GATE.GUIDE return end

  -- ---- GUIDE staleness ------------------------------------------------------
  -- The companion sends GUIDE every tick, so a gap this long means it has
  -- stopped: crashed, stalled or cut off.  Everything in the last frame is
  -- then frozen - the gimbal angles, the detection, the tracking flag - and
  -- flying on it chases a direction fixed to the aircraft's own body.  Stand
  -- down explicitly: ArduPilot's nav_scripting timeout never fires while this
  -- script keeps calling, so the release has to come from here.  The balloon
  -- estimate goes too; the next chase starts from its own measurement.
  if now_ms - link.guide_ms > C.GUIDE_STALE_MS then
    if nav_owned then
      vehicle:nav_scripting_enable(255)
      nav_owned = false
      gcs:send_text(3, "balloon chaser: released control - GUIDE stale")
    end
    est:reset()
    last_gate = GATE.STALE
    return
  end

  -- ---- gimbal: read the encoders -----------------------------------------
  -- C-02: the body-frame balloon direction comes from ENCODERS ONLY.  The
  -- camera's pixel offset drives the gimbal on the companion and never enters
  -- the geometry.
  --
  -- NOT DRIVEN FROM HERE.  The gimbal is a Gazebo model commanded by the
  -- companion over gz-transport (/gimbal/{pan,tilt}/cmd_pos), and ArduPilot has
  -- no mount driver on this airframe: mount:set_rate_target() would go to a
  -- device that does not exist, and on one that did it would be a SECOND
  -- controller fighting the companion's.  Pointing the gimbal is the camera's
  -- servo loop and belongs with the vision pipeline that produces the pixel
  -- error; the FC consumes the resulting ENCODER ANGLES and computes every
  -- steering quantity from them.
  local m_roll, m_pitch, m_yaw = mount:get_attitude_euler(0)
  if m_pitch == nil then
    -- No ArduPilot mount.  In this project the gimbal is simulated in GAZEBO and
    -- driven by the COMPANION over gz-transport, so the flight controller has no
    -- mount driver and no encoders to read.  Fall back to the encoder angles the
    -- companion sends in the GUIDE frame.
    if guide.gimbal_pan == nil then
      drop_control(GATE.MOUNT, "no gimbal angles") ; return
    end
    m_pitch, m_yaw = guide.gimbal_tilt, guide.gimbal_pan
  end

  -- encoder angles -> body-frame unit vector
  -- Age of those angles in SIMULATED milliseconds - the quantity that actually
  -- multiplies body rate to give the C-02 pairing error.
  if guide.gimbal_stamp_ms and guide.gimbal_stamp_ms > 0 then
    local d = now_ms - guide.gimbal_stamp_ms
    if pose_offset_ms == nil or d < pose_offset_ms then pose_offset_ms = d end
    gimbal_sim_age_ms = d - pose_offset_ms
  else
    gimbal_sim_age_ms = guide.gimbal_age_ms or 0   -- wall fallback
  end

  local bx, by, bz = G.pan_tilt_to_vector(m_yaw, m_pitch)

  -- ---- aircraft state ------------------------------------------------------
  --
  -- The gimbal angles and the attitude below are NOT sampled at the same time.
  --
  -- ArduPilot has no mount driver here, so mount:get_attitude_euler() returns
  -- nil and the gimbal angles above came from guide.gimbal_pan/tilt - packed by
  -- the companion on ITS wall clock and shipped over TCP.  The quaternion below
  -- is read on-chip, this tick, in simulated time.
  --
  -- So the body-frame balloon direction and the attitude used to rotate it into
  -- the world are sampled at DIFFERENT times.  That is precisely the
  -- attitude-pairing error C-02 exists to prevent: C-02 is satisfied in form
  -- (pixels are excluded) while the substitute travels the same unsynchronised
  -- path the pixels do.
  --
  -- The error is (gimbal-angle age) x (body rate).  guide.gimbal_age_ms now
  -- carries that age so it can be measured rather than argued about; a real
  -- ArduPilot mount would remove it entirely.
  local q = ahrs:get_quaternion()
  if q == nil then drop_control(GATE.QUAT, "no attitude from the AHRS") ; return end
  local roll, pitch = q:get_euler_roll(), q:get_euler_pitch()
  local gyro = ahrs:get_gyro()
  local vel = ahrs:get_velocity_NED()

  -- ---- trust: readback is available on the FC, so alpha_C = 1.0 -----------
  local trust = Est.trust(guide.det_norm_x, guide.det_norm_y, 0.0, 0.0, true)
  if (guide.flags & 0x01) == 0 then trust = 0.0 end

  -- Body measurement -> world, using THIS tick's attitude.
  --
  -- Uses G.quat_to_rotation_matrix TRANSPOSED, deliberately - NOT ArduPilot's
  -- Quaternion:earth_to_body with a conjugate.
  --
  -- The estimator works in the world frame, so every tick does a round trip:
  -- encoders -> world (here) -> body (in track.lua).  track.lua uses
  -- G.quat_to_rotation_matrix.  If the outbound leg used a DIFFERENT
  -- implementation and the two disagreed by a transpose, the round trip would
  -- apply the rotation twice instead of cancelling - an error that is zero at
  -- level flight and grows with attitude, so the two paths agree while level
  -- and diverge as a dive develops.
  --
  -- Desktop tests cannot catch that: ap_shim's earth_to_body is a
  -- transcription, self-consistent with itself whether or not it matches
  -- ArduPilot.  Using ONE implementation for both legs removes the possibility
  -- rather than testing for it.
  local r00,r01,r02,r10,r11,r12,r20,r21,r22 =
    G.quat_to_rotation_matrix(q:q1(), q:q2(), q:q3(), q:q4())
  -- world = R^T * body  (R is world->body, so its transpose is body->world)
  local mwx = r00 * bx + r10 * by + r20 * bz
  local mwy = r01 * bx + r11 * by + r21 * bz
  local mwz = r02 * bx + r12 * by + r22 * bz
  est:update(mwx, mwy, mwz, trust, dt)

  -- ---- mode dispatch ------------------------------------------------------
  -- _update_control does NOT always use the estimator.  Straight-and-level and
  -- manual SYNTHESISE a world balloon from a heading, and only balloon chaser tracks a
  -- real one.  Without this the script demanded an estimator fix in every mode,
  -- so pressing 's' produced noest on every tick even though the link, the
  -- gimbal and the attitude were all fine.
  --
  --   flags bit1 tracking, bit3 straight_level; mode 2 = manual
  local tracking       = (guide.flags & 0x02) ~= 0
  local straight_level = (guide.flags & 0x08) ~= 0
  local wx, wy, wz

  -- ---- FRESH CHASE -> RESET THE LAW ---------------------------------
  -- A mode entry is not a continuation of anything.  Every integrator is a
  -- memory of past error, and every derivative holds a sample from the last
  -- tick the law ran; carried across a mode change both describe a situation
  -- that no longer exists.
  --
  -- The integrator commands bank on the FIRST tick from error accumulated
  -- while the law was not flying.  The derivative is worse: it computes
  -- (e - e_prev)/dt against a pre-gap sample, a large difference over one
  -- small dt, and spikes into the roll axis at the exact moment of
  -- chase.  Fine in steady flight, violent on the transition.
  --
  -- Keyed on tracking and mode together: losing the balloon and regaining it
  -- is as much a fresh start as changing mode, and both must clear the yaw
  -- law's own integrator and derivative alongside the cascade's.  The FC's
  -- own mode is in the key too: the law runs whatever mode the FC is in, so
  -- while a failsafe, the transmitter or MAVProxy flies another mode its
  -- integrators wind against errors nobody is flying, and the return to ACRO
  -- is a fresh start.  guide.mode is one byte, so 4 * mode stays below 1024.
  local mode_now = vehicle:get_mode()
  -- Which law steers, every mode at once (C-03): 0 the cascade, 1 Lift
  -- Vector Pointing.  A change of law is a fresh chase, so neither carries a
  -- memory of the other's.
  local use_att = (g.steering_law or 0) >= 0.5
  local chase_key = (tracking and 1 or 0)
                   + 2 * (straight_level and 1 or 0)
                   + 4 * (guide.mode or 0)
                   + 1024 * ((mode_now == MODE_ACRO) and 1 or 0)
                   + 2048 * (use_att and 1 or 0)
  local fresh_chase = chase_key ~= s.chase_key_prev
  if fresh_chase then
    s.chase_key_prev = chase_key
    S.reset_mode_entry(cs)
    s.yaw_point_i_dps = 0.0
    PA.reset(pa)
    -- Standing down ends the balloon chaser, as the companion's own estimator reset
    -- does: the next chase starts from its own first measurement, never
    -- from the previous balloon's bearing.
    if not tracking then est:reset() end
    -- The Curve Flyer flies the turn of a p or x chase.  Straight and manual
    -- fly the selected law from their first tick, and a measurement or tuning
    -- run (autotune) flies the law it measures, its excitation included.  Any
    -- change of mode or tracking is a fresh chase, so leaving p or x hands
    -- the aircraft to the law without a PLAN on that tick, whatever Transfer
    -- Control When Done says.  A fresh chase waits for a PLAN made for it; a PLAN and
    -- a held line from an earlier one are dropped.
    CF.start_chase(cf, now_ms, tracking and CF.flies_mode(guide.mode)
                               and (g.curve_flyer_enable or 0) >= 0.5)
  end

  if not tracking then
    -- GUIDE arrives every tick, so "not tracking" is the companion's live
    -- instruction, not a gap in the feed.  Stand down NOW: merely skipping the
    -- call leaves ArduPlane flying the last command until its own 1000 ms
    -- nav_scripting timeout, and a frozen roll rate for a second is a manoeuvre
    -- nobody asked for.
    if nav_owned then
      vehicle:nav_scripting_enable(255)
      nav_owned = false
      gcs:send_text(6, "balloon chaser: released control - not tracking")
    end
    -- The companion has stood down: the fault it halted on is acknowledged.
    link.fault = 0
    last_gate = GATE.TRACK ; return
  elseif straight_level then
    -- level flight on the commanded heading: elevation 0
    wx, wy, wz = G.yaw_pitch_to_world_vec(math.rad(guide.straight_yaw), 0.0)
  elseif guide.mode == 2 then
    wx, wy, wz = G.yaw_pitch_to_world_vec(math.rad(guide.manual_yaw),
                                          math.rad(guide.manual_pitch))
  elseif (guide.flags & 0x04) ~= 0 then
    -- Oracle (ptest): an exact injected bearing.  _balloon_world_dir returns it
    -- directly and never consults the estimator - there is no measurement to
    -- fold, so waiting for an estimator fix here would wait forever.
    wx, wy, wz = guide.oracle_x, guide.oracle_y, guide.oracle_z
    if (wx * wx + wy * wy + wz * wz) < 1e-6 then
      drop_control(GATE.EST, "no oracle bearing", fresh_chase) ; return
    end
  else
    -- Waiting for the first fix after a fresh chase is expected; losing the
    -- estimate the law was flying on is not.
    if not est.valid then
      drop_control(GATE.EST, "no balloon estimate", fresh_chase) ; return
    end
    local e = est:get()
    wx, wy, wz = e:x(), e:y(), e:z()
  end

  -- ---- per-tick inputs ----------------------------------------------------
  cfg.qw, cfg.qx, cfg.qy, cfg.qz = q:q1(), q:q2(), q:q3(), q:q4()
  cfg.vel_valid = (vel ~= nil)
  if vel then cfg.vel_n, cfg.vel_e, cfg.vel_d = vel:x(), vel:y(), vel:z() end
  cfg.mode = guide.mode
  cfg.det_norm_x, cfg.det_norm_y = guide.det_norm_x, guide.det_norm_y
  cfg.rate_p, cfg.rate_q, cfg.rate_r = gyro:x(), gyro:y(), gyro:z()
  -- Cached: name-keyed param:get() lookups, for values that change only when
  -- the operator edits them.  Refreshed once a second so a mid-flight change
  -- is still picked up.
  if now_ms - last_param_ms >= 1000 then
    last_param_ms = now_ms
    cached_acro_roll  = param:get('ACRO_ROLL_RATE') or 180.0
    cached_acro_pitch = param:get('ACRO_PITCH_RATE') or 90.0
    -- THE CEILING ON THE WHOLE YAW COMMAND, and the law's own bound: the FC
    -- constrains the yaw argument of set_target_throttle_rate_rpy to
    -- +-ACRO_YAW_RATE (commands_logic.cpp:1289), so clamping here is not a
    -- second opinion, it is the same limit made visible.  Clamping silently
    -- inside the FC would leave the pointing integrator winding against a stop
    -- the log cannot see.
    --
    -- 0 is not a safe fallback here - it is the value that deletes every yaw
    -- command - so a failed read keeps the last good value and init() checks it
    -- once at boot.
    cached_acro_yaw   = param:get('ACRO_YAW_RATE') or cached_acro_yaw
    cached_aspd_min   = param:get('AIRSPEED_MIN')  or cached_aspd_min
  end
  -- The rates the law may command: ArduPlane's ACRO rates.  The cascade clips
  -- at these and its anti-windup reads them, so no integrator winds against a
  -- ceiling it cannot see.
  cfg.acro_roll_rate_dps  = cached_acro_roll
  cfg.acro_pitch_rate_dps = cached_acro_pitch
  cfg.acro_yaw_rate_dps   = cached_acro_yaw

  -- Excitation is generated HERE, against the FC clock, on the axis EXCITE
  -- names: replace=0 adds to that axis's rate command (the loop stays closed),
  -- replace=1 replaces it (open loop on that axis).  An axis this script does
  -- not know excites nothing.
  local exc_v = E.value(exc, now_ms)
  local exc_roll  = exc.running and exc.axis == E.AXIS_ROLL
  local exc_pitch = exc.running and exc.axis == E.AXIS_PITCH
  cfg.freqid_inject_dps       = (exc_roll  and not exc.replace) and exc_v or 0.0
  cfg.freqid_inject_pitch_dps = (exc_pitch and not exc.replace) and exc_v or 0.0
  cfg.plantid_override_valid       = exc_roll and exc.replace
  cfg.plantid_roll_override_dps    = exc_v
  cfg.plantid_pitch_override_valid = exc_pitch and exc.replace
  cfg.plantid_pitch_override_dps   = exc_v

  -- ---- steering -----------------------------------------------------------
  -- After a tick the law did not run, it starts afresh: its integrators were
  -- inactive across the gap, and its derivatives hold a sample from before it.
  if not law_ran_before then
    S.reset_mode_entry(cs)
    s.yaw_point_i_dps = 0.0
    PA.clear_integrators(pa)
  end
  -- The path's own turn is read against the previous tick's path, which is
  -- this tick's predecessor only when the law ran on it, over the interval
  -- as measured (not the clamped dt the D terms use).
  s.path_prev_valid = law_ran_before
  cfg.tick_s = gap_ms * 0.001
  -- The line the Curve Flyer aims the law at - a run-in's, from this tick's
  -- balloon, or a hold's - read in this tick's path frame too.
  s.hold_valid = CF.line(cf, wx, wy)
  s.hold_n, s.hold_e, s.hold_d = cf.hold_n, cf.hold_e, cf.hold_d
  local pan_err, e_pitch, tbx, tby, tbz =
    T.track_world_balloon(wx, wy, wz, roll, pitch, cfg.qw, cfg.qx, cfg.qy, cfg.qz,
                         s, cfg, out)

  -- ---- the Curve Flyer ------------------------------------------------------
  -- Runs before the law.  While it waits for its PLAN the rates are held at
  -- zero (below); while it runs one - the turn, a run-in, its dive, a hold -
  -- the selected law flies its PLAN MODE: aimed at the Curve Flyer's aim point,
  -- at the PLAN's bank, with its own roll, pitch, rudder and bank limit.
  -- The aim point is read in the path frame and turned into body axes here, and
  -- its pitch error is track.lua's formula on it.  When the aim point changes
  -- between the balloon and a line the cascade's pitch derivative re-seeds,
  -- rather than differentiating the jump.
  CF.step(cf, now_ms, s, g)
  local plan_bank = nil
  local abx, aby, abz, a_pitch = tbx, tby, tbz, e_pitch
  if cf.plan then
    plan_bank = cf.bank_ref_deg
    abx = cf.tx * s.path_bx + cf.ty * s.path_yx + cf.tz * s.path_zx
    aby = cf.tx * s.path_by + cf.ty * s.path_yy + cf.tz * s.path_zy
    abz = cf.tx * s.path_bz + cf.tz * s.path_zz
    a_pitch = math.atan(-cf.tz, math.sqrt(cf.tx * cf.tx + cf.ty * cf.ty))
  end
  if cf.retarget then cs.pitch_err_deriv_init = false end

  local omega_x, omega_y
  if use_att then
    -- LIFT VECTOR POINTING reads the same path frame, then the excitation
    -- goes on as it goes on the cascade's output: freq-ID's ADDS to the
    -- axis it names, plant-ID's REPLACES it (open loop on that axis, whose
    -- integrator then reaches nothing and is cleared).
    omega_x, omega_y = PA.step(pa, s, cfg, g, abx, aby, abz, dt, plan_bank)
    omega_x = omega_x + (cfg.freqid_inject_dps or 0.0)
    omega_y = omega_y + (cfg.freqid_inject_pitch_dps or 0.0)
    if cfg.plantid_override_valid then
      omega_x = cfg.plantid_roll_override_dps
      pa.roll_i, pa.log_roll_i_dps = 0.0, 0.0
    end
    if cfg.plantid_pitch_override_valid then
      omega_y = cfg.plantid_pitch_override_dps
      pa.pitch_i, pa.log_pitch_i_dps = 0.0, 0.0
    end
    -- The cascade does not run: it starts fresh if it is selected again.
    S.reset_mode_entry(cs)
    s.yaw_point_i_dps = 0.0
  else
    -- THE CASCADE'S CONVENTION IS "HOW FAR TO TURN RIGHT", which is the
    -- negative of pan_err's "where the path is relative to the balloon".
    -- Negated here, once, at the boundary - NOT inside control_roll.lua, whose
    -- four anti-windup direction tests each compare the error against the
    -- bank or rate it produces and would every one need inverting.  This is a
    -- control step, not a display flip: pan_err is logged exactly as it
    -- stands above.
    omega_x, omega_y = K.apply_control_law(-pan_err, a_pitch, dt, roll, pitch,
                                           abx, aby, abz, s, cs, g, cfg, out, plan_bank)
  end
  -- Waiting for the PLAN: the rates are held at zero.  The law still runs, so
  -- its internals are logged as what it would have flown; its output is not
  -- flown, so its integrators and derivative memory are cleared every such
  -- tick and it starts fresh when the PLAN arrives.
  local holding = cf.active and not cf.plan
  if holding then
    omega_x, omega_y = 0.0, 0.0
    S.reset_mode_entry(cs)
    s.yaw_point_i_dps = 0.0
    PA.clear_integrators(pa)
  end

  -- The law ran: publish what it saw and worked out.  Set before the claim, so
  -- a tick the FC refuses to fly (lua_gate 6) still shows what it would have.
  law_ran = true
  st2.balloon_body_x, st2.balloon_body_y, st2.balloon_body_z = tbx, tby, tbz
  st2.los_elev_deg = out.log_los_elev_deg or NAN
  -- Each law's own internals only from the law that ran: `out` keeps the
  -- cascade's from the last tick it ran, which would read as this tick's.
  if use_att then
    st2.sideslip_deg, st2.aoa_deg = pa.log_sideslip_deg, pa.log_aoa_deg
  else
    st2.bank_rate_cmd_dps = out.log_bank_rate_cmd_dps or NAN
    st2.bank_rate_coupling_dps = out.log_bank_rate_coupling_dps or NAN
    st2.sideslip_deg = out.log_sideslip_deg or NAN
    st2.aoa_deg = out.log_aoa_deg or NAN
    st2.plane_off_nose_deg = out.log_plane_off_nose_deg or NAN
    st2.plane_err_deg = out.log_plane_err_deg or NAN
    st2.plane_axis_up = out.log_plane_axis_up or NAN
    st2.plane_drift_dps = out.log_plane_drift_dps or NAN
    st2.plane_ff_dps = out.log_plane_ff_dps or NAN
    st2.plane_err_rate_dps = out.log_plane_err_rate_dps or NAN
    st2.plane_d_dps = out.log_plane_d_dps or NAN
    st2.plane_path_share = out.log_plane_path_share or NAN
    st2.plane_path_deg = out.log_plane_path_deg or NAN
    st2.plane_path_from_vel = out.log_plane_path_from_vel or NAN
    st2.plane_i_dps = out.log_plane_i_dps or NAN
    st2.plane_i_frozen = out.log_plane_i_frozen or NAN
    st2.plane_side = out.log_plane_side or NAN
  end
  st2.path_turn_up_dps, st2.path_turn_right_dps = s.path_turn_up_dps, s.path_turn_right_dps

  -- ---- claim control ------------------------------------------------------
  -- ONLY IN ACRO, AND ONLY WHILE TRACKING (the tracking gate above releases).
  --
  -- nav_scripting_enable() accepts CIRCLE, STABILIZE, ACRO, FBWA, FBWB, CRUISE
  -- and LOITER.  Claiming with whatever mode happens to be live therefore
  -- hijacks LOITER too - and LOITER is precisely what this project switches to
  -- when a test aborts or the operator stops tracking.  The aircraft "returned
  -- to loiter", the script re-claimed it on the very next tick, and it kept
  -- flying the last steering command into the ground.
  --
  -- ACRO is the only mode the steering is designed for (it is what the
  -- companion enters to fly), so any switch AWAY from ACRO must mean the script
  -- stands down - immediately and explicitly, not after the 1000 ms timeout.
  if mode_now ~= MODE_ACRO then
    if nav_owned then
      -- Release now.  nav_scripting_enable() with a mode that is not the
      -- current one sets enabled = false, which hands the surfaces straight
      -- back to whatever mode the operator selected.
      vehicle:nav_scripting_enable(255)
      nav_owned = false
      gcs:send_text(6, string.format(
        "balloon chaser: released control - mode is %d, not ACRO", mode_now))
    end
    clear_unflown_integrators()
    last_gate = GATE.NAV
    return
  end

  if link.fault ~= 0 then
    clear_unflown_integrators()
    last_gate = GATE.FAULT
    return
  end
  nav_owned = vehicle:nav_scripting_enable(MODE_ACRO)
  if not nav_owned then
    if not nav_warned then
      gcs:send_text(4, "balloon chaser: cannot take control in ACRO")
      nav_warned = true
    end
    clear_unflown_integrators()
    last_gate = GATE.NAV
    return
  end
  nav_warned = false

  -- ---- actuate ------------------------------------------------------------
  -- Rates in deg/s straight into ArduPlane's own rate controllers - the SAME
  -- rollController / pitchController get_rate_out() that ACRO uses.  NaN is
  -- replaced here: the binding's clamp would let it through.
  if omega_x ~= omega_x then omega_x = 0.0 end
  if omega_y ~= omega_y then omega_y = 0.0 end

  -- THE TURN'S OWN BODY PITCH RATE, added after the law's own clip.
  --
  -- A banked aeroplane pitches in body axes because it is turning.  Nothing
  -- commands it and nothing can stop it, so the pointing law's demand has to sit
  -- ON TOP of it - ask for zero in a bank and the rate loop spends elevator
  -- resisting the turn's own pitch rate, which the pointing law can only make up
  -- by sitting off the balloon.
  --
  -- q = r * tan(bank), with r the coordinated-turn yaw rate already computed in
  -- sample_sensors_and_report_scaler.  Both are one heading rate resolved into body
  -- axes, so this reuses that term's airspeed floor and validity rather than
  -- repeating them.
  --
  -- NOT APPLIED PAST PITCH_TURN_FF_BANK_LIM_DEG: the identity describes a LEVEL
  -- coordinated turn, which past 60 deg needs more than 2 g, and tan() is
  -- climbing hard by then - 1.73 at 60 deg, 5.67 at 80.  Beyond that the
  -- aircraft is not flying the geometry this term describes.
  local pitch_turn_dps = 0.0
  if s.coord_valid and math.abs(math.deg(roll)) <= C.PITCH_TURN_FF_BANK_LIM_DEG then
    pitch_turn_dps = s.yaw_coord_dps * math.tan(roll)
  end
  s.log_pitch_turn_dps = pitch_turn_dps
  -- A pitch excitation that REPLACES the command is the whole command: open
  -- loop on that axis means nothing else is added to it, here or below.
  local pitch_open = cfg.plantid_pitch_override_valid
  -- Nothing is added to Lift Vector Pointing's pitch rate, which is the path
  -- frame's whole turn in body axes, nor to the rates held at zero while the
  -- Curve Flyer waits for its PLAN.
  if use_att then
    s.log_pitch_turn_dps = NAN
  elseif not pitch_open and not holding then
    omega_y = omega_y + (g.pitch_turn_ff or 0.0) * pitch_turn_dps
  end

  -- PITCH TRIM: the elevator the airframe needs to hold its nose still,
  --
  --     trim_elev = offset + (VREF / V)^2 * (lift     * lift_term
  --                                         + attitude * sin(pitch)
  --                                         + thrust   * throttle / EAS2TAS^2)
  --
  -- in ArduPlane's elevator units (+-C.ELEV_FULL_TRAVEL at full travel, the
  -- units PID_TUNING logs), with V the EAS (floored at the airspeed minimum the
  -- speed scaler uses), VREF = C.PITCH_TRIM_VREF_MPS, throttle ArduPlane's
  -- throttle output as a fraction of C.THROTTLE_OUTPUT_FULL, and lift_term
  -- cos(pitch) or the load the steering law asks for (below).  The terms are
  -- what the trim map ('e') measured: the wing's lift need, which grows as the
  -- airspeed falls; a nose-attitude moment; and the motor's moment - each held
  -- by a tail whose authority grows with V^2, hence the one (VREF/V)^2.  sin
  -- and cos of the Euler pitch stay correct through the vertical, where the
  -- angle itself folds.
  --
  -- 1 / EAS2TAS^2 is the air density as a fraction of sea level's.  The wing
  -- terms need no such factor: V is EAS, so (VREF/V)^2 already follows the
  -- density.  The motor's push does not: at one throttle the propeller turns
  -- at about one speed at any height, so its push scales with the density, and
  -- without the factor a thrust gain fitted at one height is wrong at another.
  -- The thrust gain is therefore the motor's moment at sea-level density.
  --
  -- The rate loop has no integrator, so without this its only source of trim
  -- elevator is a standing rate error, which the pointing law can only produce
  -- by sitting off the balloon.
  --
  -- This script cannot write the elevator: in the nav_scripting branch
  -- ArduPlane sets it from pitchController.get_rate_out() and nothing else.  So
  -- the trim goes in as the extra pitch-rate request that the rate loop turns
  -- into exactly that elevator.  At a steady rate, one deg/s of request buys
  --
  --     FF * scaler / EAS2TAS  +  P * scaler^2     (AP_FW_Controller::_get_rate_out)
  --
  -- of elevator, so the request is trim_elev divided by that.  It is never
  -- flown as a rotation: the aircraft holds its attitude and the rate loop
  -- reports "asked for trim_dps, got 0", which is why trim_dps has its own
  -- column.
  --
  -- Added after the law's own clip, which bounds rates the aircraft flies:
  -- this is elevator expressed in rate units.  ArduPlane's ACRO_PITCH_RATE
  -- ceiling, applied at the end, still bounds the sum.  With no airspeed
  -- estimate the trim is NaN (blank in the log) and, the scaler being 0, no
  -- trim is sent; with no FF and P the conversion is 0 and no trim is sent
  -- either.
  local sc, e2t = cfg.speed_scaler or 0.0, cfg.eas2tas or 0.0
  local v = cfg.airspeed_eas
  local q_ratio = NAN
  if v and v == v then
    local vf = math.max(v, cfg.airspeed_floor or MIN_AIRSPEED_MIN)
    q_ratio = (C.PITCH_TRIM_VREF_MPS / vf) ^ 2
  end
  local thr = (cfg.throttle_out or NAN) / C.THROTTLE_OUTPUT_FULL
  local density_ratio = (e2t > 0.0) and 1.0 / (e2t * e2t) or NAN
  -- THE LIFT TERM reads cos(pitch) under the cascade.  Under Lift Vector
  -- Pointing it reads the load the law asks of the wing: the force
  -- line's, or on a tick the Curve Flyer owns the chase the load of the rates
  -- flown, since its PLAN's bank, or the rates held at zero, do not face the
  -- force line.  1/cos(bank) in a level turn, negative in a push, zero
  -- straight up, where cos(pitch) is right only in straight, upright flight.
  -- The trim map fits against what the trim read (pitch_trim_lift_term), so
  -- its lift gain means the same either way.
  local lift_term = math.cos(pitch)
  if use_att then
    lift_term = cf.active and PA.load_of(s, omega_x, omega_y) or pa.n_lift
  end
  st_msg.pitch_trim_lift_term = lift_term
  local trim_elev = (g.pitch_trim_offset or 0.0)
                  + q_ratio * ((g.pitch_trim_lift or 0.0) * lift_term
                               + (g.pitch_trim_attitude or 0.0) * math.sin(pitch)
                               + (g.pitch_trim_thrust or 0.0) * thr * density_ratio)
  local ptch_ff = ptch_ff_param and ptch_ff_param:get() or 0.0
  local ptch_p  = ptch_p_param  and ptch_p_param:get()  or 0.0
  local elev_per_dps = 0.0
  if sc > 0.0 and e2t > 0.0 then
    elev_per_dps = ptch_ff * sc / e2t + ptch_p * sc * sc
  end
  local trim_dps = 0.0
  if elev_per_dps > TRIM_CONV_MIN and trim_elev == trim_elev then
    trim_dps = trim_elev / elev_per_dps
  end
  s.log_pitch_trim_elev = trim_elev
  s.log_pitch_trim_dps = trim_dps
  if not pitch_open then omega_y = omega_y + trim_dps end
  -- ---- YAW: COORDINATION AND POINTING -------------------------------------
  --   r_cmd = clamp(coordination + point, +-ACRO_YAW_RATE)
  --   coordination = yaw_coordination_ff * (turning yaw + rolling yaw)
  --   point        = clamp(-(kp*pan_err + INT(ki*pan_err)), +-yaw_point_lim)
  --
  -- The yaw rate this sends is the WHOLE body yaw rate ArduPlane's rate loop
  -- then holds, not an addition to one the aircraft would have anyway.
  -- COORDINATION is the yaw the aircraft needs to fly clean: the turn's own
  -- yaw rate, and the yaw that makes a roll turn about the flight path rather
  -- than the nose.  POINTING steers, on the same path-to-balloon error the roll
  -- axis closes, as a bounded assist to bank-to-turn.
  --
  -- ROLLING YAW.  A roll about the nose swings the path round the nose: with the
  -- path alpha below the nose, a roll of phi turns alpha*sin(phi) of it into
  -- sideslip, and every balloon error the laws read from the path moves with
  -- it.  Rolling about the path instead needs r = p * tan(alpha), alpha the
  -- path's angle below the nose in the body pitch plane, from the path
  -- track.lua took this tick (the nose, and so zero, at or below
  -- FLIGHT_PATH_MIN_SPEED_MPS).  The vertical marker is atan2(-v_z, v_x),
  -- positive with the path ABOVE the nose, so alpha is its NEGATIVE: without
  -- the sign this term adds the slip it exists to remove.  p is the roll rate this
  -- tick commands, so the yaw loop moves with the roll loop instead of
  -- following the slip the roll has already built.  It grows without bound as
  -- the path nears 90 deg off the nose, where no roll about the nose turns
  -- about the path; the ACRO_YAW_RATE ceiling below bounds the command.
  local rolling_yaw = omega_x * math.tan(-s.vel_marker_rad)
  local coordination = 0.0
  local yaw_rate_cmd, yaw_point = 0.0, 0.0

  local ail_on  = (g.aileron_override  or 0.0) >= 0.5
  local elev_on = (g.elevator_override or 0.0) >= 0.5
  local rud_on  = (g.rudder_override   or 0.0) >= 0.5
  local any_ovr = ail_on or elev_on or rud_on
  s.log_rud_ovr = any_ovr and 1 or 0
  if any_ovr then
    -- MANUAL SURFACE OVERRIDE, per surface.  Each switch is independent, and a
    -- surface whose switch is off is not written at all.  Whenever ANY of them
    -- is on the yaw law is skipped entirely, along with every bound in it - a
    -- plant-ID step that a clamp quietly reshapes identifies the clamp, not the
    -- airframe.
    --
    -- WHY NOT set_rudder_offset.  That binding exists only for the rudder and
    -- routes through ArduPlane's mixer.  There is no equivalent for aileron or
    -- elevator: those axes leave this script as RATES, through
    -- set_target_throttle_rate_rpy, which is a closed loop and not a surface
    -- command.  Driving the servo channels is the only way to command a
    -- deflection on all three.
    --
    -- CONSEQUENCE, and it is not small: writing a servo channel bypasses
    -- ArduPlane's stabilisation on that axis.  Overriding the aileron leaves
    -- pitch and yaw alone; overriding either tail surface takes BOTH
    -- ruddervators, because a V-tail has no separate elevator or rudder
    -- channel, and pitch stabilisation goes with them.
    --
    -- Throttle is untouched (a different channel) and set_target_throttle_rate_rpy
    -- keeps being called - roll and pitch still carry the law's rates, yaw
    -- carries 0 - so nav_scripting stays active and the motor keeps running.
    --
    -- The override carries a 200 ms timeout refreshed every tick: if this script
    -- stops, dies, or is removed, the surfaces return to ArduPlane within 200 ms
    -- on their own.  That is the failsafe, and it is why the timeout is short.
    --
    -- The pointing integrator is CLEARED: switching the override off starts from
    -- a known zero instead of dumping whatever the integrator drifted to while
    -- nobody was watching.
    yaw_rate_cmd = 0.0
    ovr_write_surfaces(ail_on,  g.aileron_deflection_pct  or 0.0,
                       elev_on, g.elevator_deflection_pct or 0.0,
                       rud_on,  g.rudder_deflection_pct   or 0.0)
    s.yaw_point_i_dps = 0.0
    -- An overridden surface does not move with the law, so the integrators
    -- on its axis are inactive and are cleared.  Either tail surface takes
    -- both ruddervators, and pitch with them.
    if ail_on then
      cs.pan_integrator = 0.0
      cs.plane_i_dps = 0.0
      if use_att then
        pa.roll_i, pa.log_roll_i_dps = 0.0, 0.0
      else
        st2.plane_i_dps = 0.0
      end
    end
    if elev_on or rud_on then
      cs.pitch_integrator = 0.0
      if use_att then pa.pitch_i, pa.log_pitch_i_dps = 0.0, 0.0 end
    end
  elseif use_att then
    -- LIFT VECTOR POINTING, VERSION B, flies no yaw: the rudder is held at
    -- zero below (set_rudder_offset(0, false)) and the fin keeps the aircraft
    -- straight.  A roll about the nose swings the path round it, which the law
    -- reads as path error like any other.  The yaw law's integrator reaches
    -- nothing and is cleared.
    yaw_rate_cmd = 0.0
    s.yaw_point_i_dps = 0.0
  else
    -- ===================================================================
    -- COORDINATION: the turning yaw (yaw_coord_dps, from this tick's sensor
    -- pass, zero with no airspeed estimate) plus the rolling yaw above, scaled
    -- by yaw_coordination_ff - 1 is the geometry.  A feedforward: it costs no
    -- phase, and a wrong value only slips the aircraft.
    --
    -- POINTING: a bounded steering assist on pan_err, the path-to-balloon error
    -- the roll axis closes too.
    --
    -- REQUIRES YAW_RATE_P/I/D/FF at their tuned values.  With them zeroed the
    -- rate controller outputs nothing and the rudder goes dead.  init() checks.
    -- ===================================================================
    -- pan_err is path - balloon, the error roll closes too; this is the rudder's
    -- bounded share of it.  Negated because a path RIGHT of the balloon wants a
    -- nose-LEFT rate.
    local p_kp  = g.yaw_point_kp  or 0.0
    local p_ki  = g.yaw_point_ki  or 0.0
    local p_lim = g.yaw_point_lim or 0.0
    local p_wu  = g.yaw_point_windup or 1.0
    local pe    = pan_err or 0.0
    -- The steep schedule: the pointing output times the roll blend's steep
    -- share, the path's angle to the horizon between Roll-to-Plane Steep Start
    -- and Full - one number for "banking is losing its grip" on both axes.
    -- The integrator grows by the same share, so it does not wind up while the
    -- schedule holds the output at 0 on a shallow path.
    local sched = ((g.yaw_point_pitch_sched or 0) ~= 0) and cs.steep_share or 1.0

    -- The integrator is clamped to the SAME bound as the whole term, so a wound
    -- integral cannot be hidden behind a clamp on the sum and then reappear when
    -- the proportional part changes sign.
    if p_ki ~= 0.0 and s.lat_marker_valid and sched > 0.0 then
      s.yaw_point_i_dps = s.yaw_point_i_dps + p_ki * pe * sched * dt
      -- Anti-windup: the integrator owns only its share (p_wu) of the output
      -- limit, leaving the rest of p_lim for the Kp part.  Applied at every
      -- non-negative limit, so a limit of 0 pins the integrator at 0.
      local p_iceil = p_lim * p_wu
      if s.yaw_point_i_dps >  p_iceil then s.yaw_point_i_dps =  p_iceil
      elseif s.yaw_point_i_dps < -p_iceil then s.yaw_point_i_dps = -p_iceil end
    else
      -- Switching the term off, or a schedule that gives it no share, REMOVES
      -- it rather than leaving the aircraft flying whatever the integral last
      -- reached.
      s.yaw_point_i_dps = 0.0
    end
    yaw_point = -(p_kp * pe + s.yaw_point_i_dps)
    -- yaw_point_lim bounds the term OUTPUT; 0 means 0 (no pointing authority),
    -- not "no ceiling".
    if yaw_point >  p_lim then yaw_point =  p_lim
    elseif yaw_point < -p_lim then yaw_point = -p_lim end

    yaw_point = yaw_point * sched
    -- Pointing steers on bank-to-turn's error; while the Curve Flyer owns
    -- the chase it is off and its integrator cleared.  Coordination stays.
    if cf.active then
      yaw_point = 0.0
      s.yaw_point_i_dps = 0.0
    end

    local ceil = cached_acro_yaw
    coordination = (g.yaw_coordination_ff or 0.0) * (s.yaw_coord_dps + rolling_yaw)
    yaw_rate_cmd = coordination + yaw_point

    -- The same ceiling the FC applies, made visible: it constrains the command
    -- to +-ACRO_YAW_RATE regardless, so mirroring it here keeps the logged
    -- command equal to what the rudder actually received.
    if ceil > 0.0 then
      if yaw_rate_cmd >  ceil then yaw_rate_cmd =  ceil end
      if yaw_rate_cmd < -ceil then yaw_rate_cmd = -ceil end
    end
  end

  -- IS THE YAW LAW SAYING ANYTHING AT ALL?
  --
  -- With every yaw gain at zero the law commands 0.0 deg/s - and a commanded
  -- ZERO IS NOT NO COMMAND.  Past the three gates it is an active instruction to
  -- hold the body yaw rate at zero, which ArduPlane's rate controller chases
  -- with real rudder.  A banked aeroplane is already rotating: at 26 deg of bank
  -- and 20 m/s that is about 12 deg/s of yaw the aircraft HAS, and telling the rate
  -- loop to stop it costs 46% of full rudder held, peaking at 67%, and builds
  -- 5 deg of sideslip - with every gain at zero and the log correctly reporting
  -- the law as off.
  --
  -- So zeroing the gains must ALSO take the rate controller out of the loop.
  -- Passing false here makes the FC write rudder = 0 and call
  -- yawController.reset_I(), which is genuinely "no rudder from this script"
  -- rather than "hold still".  Coordination alone is enough to run it: it is
  -- the yaw the aircraft HAS, so with it on the rate loop holds the turn's yaw
  -- instead of fighting it.
  --
  -- Lift Vector Pointing, version B, holds the rudder at zero the same way:
  -- false here, whatever the yaw gains say.
  local yaw_active = not use_att
                 and ((g.yaw_point_kp or 0.0) ~= 0.0
                      or (g.yaw_point_ki or 0.0) ~= 0.0
                      or (g.yaw_coordination_ff or 0.0) ~= 0.0)

  -- The second argument DECIDES WHETHER THE YAW RATE CONTROLLER RUNS AT ALL.
  -- All three axes leave this script as rates and ArduPlane owns every surface.  The 0.0 is the rudder
  -- DEFLECTION offset, which this law does not use: the FC adds it straight to
  -- the surface on top of the rate controller's output (Attitude.cpp,
  -- nav_scripting branch), so a non-zero value here would be a second input to
  -- one degree of freedom - a trim in deflection arguing with a trim in rate.
  -- One knob, one owner.  The pair (0, true) is what routes the rate demand to
  -- the rudder at all.
  --
  -- Under a surface override yaw_rate_cmd is 0 and the rate controller is
  -- still called, so nav_scripting stays active and throttle keeps flowing.
  -- omega_x/omega_y keep flying the law; on an overridden surface the servo
  -- write above is what reaches it.
  vehicle:set_rudder_offset(0.0, yaw_active)

  -- THROTTLE.  GUIDE carries the OPERATOR's throttle; the airspeed-hold loop
  -- runs HERE, and with airspeed_cmd_enable = 0 it hands that throttle back
  -- unchanged.  Its only actuator is this value - it has no route into the
  -- rate commands above.
  local throttle = CP.compute_throttle_command(cfg.airspeed_eas, dt, thr_state,
                                               g, guide.throttle_pct)
  thr_state.flown = true
  -- NaN guard on the rates, at the last step: a NaN passes every clamp,
  -- because every comparison with it is false, and the earlier guard runs
  -- before the turn's pitch rate, the ceilings and the yaw command.
  if omega_x ~= omega_x then omega_x = 0.0 end
  if omega_y ~= omega_y then omega_y = 0.0 end
  if yaw_rate_cmd ~= yaw_rate_cmd then yaw_rate_cmd = 0.0 end
  -- ArduPlane's own ceilings on the roll and pitch rates, made visible: it
  -- constrains them to +-ACRO_ROLL_RATE and +-ACRO_PITCH_RATE on receipt
  -- (commands_logic.cpp, set_target_throttle_rate_rpy), so clamping here
  -- changes nothing it flies and keeps the logged command the flown one.  The
  -- pitch trim and the turn's pitch rate are added after the law's own clip,
  -- so at full pull their sum passes ACRO_PITCH_RATE and this is where it is
  -- cut.
  omega_x = math.max(-cached_acro_roll, math.min(cached_acro_roll, omega_x))
  omega_y = math.max(-cached_acro_pitch, math.min(cached_acro_pitch, omega_y))
  vehicle:set_target_throttle_rate_rpy(throttle, omega_x, omega_y,
                                       yaw_rate_cmd)
  st_msg.pan_err_deg  = pan_err or 0.0
  if use_att then
    -- No yaw law ran and no yaw rate was commanded: the yaw columns are blank.
    st_msg.yaw_rate_cmd_dps = NAN
    st_msg.yaw_point_dps = NAN
    st_msg.yaw_ff_ari_dps = NAN
    st_msg.yaw_coordination_dps = NAN
    st2.yaw_point_i_dps = NAN
  else
    st_msg.yaw_rate_cmd_dps = yaw_rate_cmd
    st_msg.yaw_point_dps = yaw_point
    st_msg.yaw_ff_ari_dps = rolling_yaw
    st_msg.yaw_coordination_dps = coordination
    st2.yaw_point_i_dps = s.yaw_point_i_dps
  end
  st_msg.pitch_turn_dps = s.log_pitch_turn_dps
  st_msg.pitch_trim_elev = s.log_pitch_trim_elev
  st_msg.pitch_trim_dps = s.log_pitch_trim_dps
  last_gate = GATE.OK

  -- ---- mirrored state back to the companion -------------------------------
  -- Record what was ACTUALLY commanded, at the point of actuation and from the
  -- same locals passed to set_target_throttle_rate_rpy. Anything sampled
  -- earlier misses ArduPlane's ceilings applied a few lines above, and anything
  -- recomputed is a second implementation that can drift from the first.
  --
  -- These are the fields the HUD's Roll/Pitch Cmd lines and the CSV's
  -- lua_omega_x/y read. Leave them unassigned and every consumer reports a
  -- confident zero: the protocol carries the field, the companion decodes it,
  -- the column exists - and the number is simply never written.
  st_msg.omega_x_cmd_dps = omega_x
  st_msg.omega_y_cmd_dps = omega_y
  -- The throttle that reached set_target_throttle_rate_rpy, and the airspeed
  -- loop behind it: bit 0 of asp_flags = loop active, bit 1 = output clipped.
  st_msg.throttle_out    = throttle
  st_msg.asp_flags       = (thr_state.log_en ~= 0 and 1 or 0)
                         | (thr_state.log_sat ~= 0 and 2 or 0)
  st_msg.asp_err_mps     = thr_state.log_err_mps
  st_msg.asp_i_term      = thr_state.log_i_term
  -- Locals that only exist inside steering(): stash them for the STATE block,
  -- which runs on a divisor and cannot see this scope.
  st_msg.world_est_trust = trust
  st_msg.excite_value    = exc_v
end

-- ---------------------------------------------------------------------------
-- lifecycle
-- ---------------------------------------------------------------------------
--- Run the steering, then ALWAYS report - including when a gate stopped it.
--- A STATE frame that only appears on success cannot explain a failure.
local function step()
  thr_state.flown = false
  local ok, err = pcall(steering)
  if not ok then script_fault(err) end
  -- The airspeed hold's integrator is inactive on any tick its throttle was
  -- not flown - a gate, a release, an error - and is
  -- cleared, never carried to the next engagement.
  if not thr_state.flown then S.reset_throttle(thr_state) end
  if port and (tick % STATE_DIVISOR == 0) then
    st_msg.gate = last_gate
    -- The seq of the GUIDE frame this tick was computed from, so the
    -- companion can tell which of its frames an answer belongs to.
    st_msg.seq              = (guide.seq or 0) & 0xFFFF
    st_msg.flags            = (nav_owned and 0x01 or 0)
                            | (est.valid and 0x02 or 0)
                            | (exc.running and 0x04 or 0)
                            | (law_ran and 0x08 or 0)
                            | (link.rejected and 0x10 or 0)
                            | (link.fault << 5)
    -- EVERYTHING THAT PARTICIPATES IN THE CONTROL LAW IS RECORDED.
    -- A protocol field the FC never assigns reports a confident zero in the CSV
    -- and the HUD - indistinguishable from a real measurement of zero.  These
    -- are read from the long-lived state tables, which the STATE block can see;
    -- values local to steering() are stashed there instead.
    --
    -- Those tables keep their values across ticks, so on a tick the law did not
    -- run they hold an earlier tick's.  Such a tick sends NaN (float fields)
    -- and 0 with flags bit 3 clear (cs_flags), never the stale value.
    -- The track's readings stand whichever law ran; the cascade's internals
    -- only on a tick the cascade ran.  On a tick Lift Vector Pointing flew,
    -- cs_flags bit 6 says so and the cascade's bits mean nothing; on a tick
    -- a PLAN set the bank, under either law, bit 5 does.
    if law_ran then
      -- The STEERING ERROR track.lua computes, not the encoder angles GUIDE
      -- carries inward: the two are different quantities under similar names.
      st_msg.steering_pan     = out.steering_pan_deg or NAN
      st_msg.steering_tilt    = out.steering_tilt_deg or NAN
      st_msg.vel_marker_deg   = math.deg(s.vel_marker_rad)
      st_msg.lat_marker_deg   = math.deg(s.lat_marker_rad)
    else
      st_msg.steering_pan, st_msg.steering_tilt = NAN, NAN
      st_msg.vel_marker_deg, st_msg.lat_marker_deg = NAN, NAN
    end
    if law_ran and pa.ran then
      st_msg.cs_flags = 64 | (pa.w_plan and 32 or 0) | ((s.log_rud_ovr or 0) ~= 0 and 128 or 0)
      st_msg.p_term_roll, st_msg.d_term_roll = NAN, NAN
      st_msg.d_term_pitch = NAN
      -- the bank its band holds, measured as the cascade's feedback is
      st_msg.bank_proxy_deg = pa.log_proxy_deg
      st_msg.theta_ip_deg = NAN
      st_msg.pan_integrator, st_msg.pitch_integrator = NAN, NAN
      st_msg.roll_cmd_presat = NAN
      st_msg.requested_roll, st_msg.requested_pitch = NAN, NAN
      st_msg.omega_x_presat_dps, st_msg.omega_y_presat_dps = NAN, NAN
      st_msg.roll_err_deg = NAN
      st_msg.upright = NAN
    elseif law_ran then
      -- bit 5: a PLAN set the bank, so bank-to-turn did not run and bits 0-2
      -- mean nothing.
      local bank_bits = 32
      if not cs.plan_set_bank then
        bank_bits = (cs.log_roll_sat ~= 0 and 1 or 0)
                  | (cs.log_pan_i_frozen ~= 0 and 2 or 0)
                  | (cs.log_pan_i_clamped ~= 0 and 4 or 0)
      end
      st_msg.cs_flags = bank_bits
                      | (cs.log_pitch_i_frozen ~= 0 and 8 or 0)
                      | (cs.log_pitch_i_clamped ~= 0 and 16 or 0)
                      -- bit 7: a surface was under MANUAL OVERRIDE, not the
                      -- law.  Without it the CSV cannot tell an override
                      -- deflection from one the law produced.
                      | ((s.log_rud_ovr or 0) ~= 0 and 128 or 0)
      st_msg.p_term_roll = cs.log_p_term_roll
      st_msg.d_term_roll = cs.log_d_term_roll
      st_msg.d_term_pitch = cs.log_d_term_pitch
      st_msg.bank_proxy_deg = out.log_bank_proxy_deg or NAN
      st_msg.theta_ip_deg     = out.log_theta_ip_deg or NAN
      st_msg.pan_integrator   = cs.pan_integrator
      st_msg.pitch_integrator = cs.pitch_integrator
      -- The bank chain, both sides of the bank-limit clip:
      --   roll_cmd_presat  what the law asked for, before the limit
      --   requested_roll   what survived it, and what the rate loop tracks
      -- Unlike the ACRO rate clamp, this one fires in ordinary balloon chaser, and
      -- when it does, no gain change helps.
      st_msg.roll_cmd_presat = cs.log_roll_cmd_presat
      st_msg.requested_roll  = s.requested_roll_rad
      st_msg.requested_pitch = s.requested_pitch_rad
      st_msg.omega_x_presat_dps = cs.log_omega_x_presat
      st_msg.omega_y_presat_dps = cs.log_omega_y_presat
      st_msg.roll_err_deg = cs.log_roll_err_deg
      st_msg.upright = s.log_upright
    else
      st_msg.cs_flags = 0
      st_msg.p_term_roll, st_msg.d_term_roll = NAN, NAN
      st_msg.d_term_pitch = NAN
      st_msg.bank_proxy_deg = NAN
      st_msg.theta_ip_deg = NAN
      st_msg.pan_integrator, st_msg.pitch_integrator = NAN, NAN
      st_msg.roll_cmd_presat = NAN
      st_msg.requested_roll, st_msg.requested_pitch = NAN, NAN
      st_msg.omega_x_presat_dps, st_msg.omega_y_presat_dps = NAN, NAN
      st_msg.roll_err_deg = NAN
      st_msg.upright = NAN
    end
    -- Lift Vector Pointing's internals: PA.clear_log blanks them at the
    -- tick's entry, so a tick it did not run sends NaN.
    st_msg.att_roll_i_dps, st_msg.att_pitch_i_dps = pa.log_roll_i_dps, pa.log_pitch_i_dps
    st_msg.att_wx_dps, st_msg.att_wy_dps, st_msg.att_wz_dps =
      pa.log_wx_dps, pa.log_wy_dps, pa.log_wz_dps
    st_msg.att_bank_deg = pa.log_bank_deg
    st_msg.att_band_lo_deg, st_msg.att_band_hi_deg = pa.log_band_lo_deg, pa.log_band_hi_deg
    st_msg.att_room_lo_deg, st_msg.att_room_hi_deg = pa.log_room_lo_deg, pa.log_room_hi_deg
    st_msg.att_drift_dps, st_msg.att_theta_now_deg = pa.log_drift_dps, pa.log_theta_now_deg
    st_msg.att_p_dps, st_msg.att_q_dps = pa.log_p_dps, pa.log_q_dps
    st_msg.att_w_f = pa.log_w_f
    st_msg.att_n_lift = pa.ran and pa.n_lift or NAN
    st_msg.hz_quat, st_msg.hz_gyro = hz.quat, hz.gyro
    st_msg.hz_vel,  st_msg.hz_asp  = hz.vel,  hz.asp
    st_msg.fc_millis = now_ms_cur & 0xFFFFFFFF
    st_msg.gimbal_sim_age_ms = math.min(65535, math.max(0, gimbal_sim_age_ms))
    port:writestring(proto.pack_state(st_msg))
    st2.seq   = st_msg.seq
    st2.flags = (s.coord_valid and 0x01 or 0)
              | (s.accel_valid and 0x02 or 0)
              | (s.lat_marker_valid and 0x04 or 0)
    -- The Curve Flyer: its state and accepted plan every tick, what it hands
    -- the law NaN on a tick the law flew no PLAN (CF.clear_log at steering's
    -- entry).
    st2.cf_state, st2._cf_pad, st2.cf_plan_id = cf.state, 0, cf.plan_id
    st2.cf_progress, st2.cf_eps_deg = cf.progress, cf.eps_deg
    st2.cf_bank_ref_deg = cf.bank_ref_deg
    st2.cf_hold_n, st2.cf_hold_e, st2.cf_hold_d = cf.hold_n, cf.hold_e, cf.hold_d
    st2.att_theta_deg, st2.att_u_y, st2.att_u_z = pa.log_theta_deg, pa.log_u_y, pa.log_u_z
    st2.att_omega_dps = pa.log_omega_dps
    st2.att_f_y_mps2, st2.att_f_z_mps2 = pa.log_f_y, pa.log_f_z
    st2.att_mu_deg, st2.att_side = pa.log_mu_deg, pa.log_side
    port:writestring(proto.pack_state2(st2))
  end
end


-- ---- sensors and the speed scaler ------------------------------------------
-- UNCONDITIONAL, and deliberately OUTSIDE step().
--
-- ArduPlane scales rate-loop output by SCALING_SPEED/airspeed, so the FC's gains
-- move with speed.  That movement is REAL and is reported, not suppressed.
--
-- Those gains act on the aircraft whether or not steering has a balloon, so the
-- report must not sit behind the steering gates: below the gimbal, quaternion
-- and ACRO-ownership checks it would run only on ticks where steering had a
-- complete picture, and never in MANUAL or LOITER, which is most of a session.
--
-- ahrs:airspeed_EAS() is the ONLY airspeed binding ArduPilot exposes -
-- bindings.desc declares airspeed_estimate as an ALIAS of it, not a second
-- source.  There is no fallback to try, and groundspeed is not a substitute:
-- a scaler computed from groundspeed would scale the rate loop by the wind.
--
-- A refusal here is visible as hz_asp = 0 and a blank lua_scaler in the log.
local function sample_sensors_and_report_scaler()
  -- SENSOR FRESHNESS.  These count how often each reading actually CHANGED,
  -- which is the only way to tell a live 50 Hz feed from the same stale number
  -- read 50 times.  They belong here, not in step(): counted below the steering
  -- gates they report 0 whenever the companion is not commanding, which reads
  -- as "the FC has no sensors" when the truth is "steering is idle".  A
  -- diagnostic that goes blind exactly when you are investigating why nothing
  -- is happening is worse than none.
  local q_now  = ahrs:get_quaternion()
  local gyro_n = ahrs:get_gyro()
  local vel_n  = ahrs:get_velocity_NED()
  if q_now  and q_now:q1() ~= prev_qw then fresh.quat = fresh.quat + 1 ; prev_qw = q_now:q1() end
  if gyro_n and gyro_n:x()  ~= prev_gx then fresh.gyro = fresh.gyro + 1 ; prev_gx = gyro_n:x() end
  if vel_n  and vel_n:x()   ~= prev_vn then fresh.vel  = fresh.vel  + 1 ; prev_vn = vel_n:x() end

  -- SINGLE return value, not a (bool, value) pair.  bindings.desc declares
  -- "boolean float'Null", the same form as get_quaternion and get_velocity_NED,
  -- and ArduPilot returns the number or nil for all three.  Unpacking it as a
  -- pair puts the airspeed in the first variable and nil in the second, so every
  -- test against the second silently fails and the scaler is never computed.
  local eas = ahrs:airspeed_EAS()
  if eas and eas ~= prev_asp then fresh.asp = fresh.asp + 1 ; prev_asp = eas end

  -- STATE2's sensor group: what the law reads, on the FC's clock, every tick
  -- whatever the gates decide.  NaN where a reading is absent, never a zero
  -- that would read as a measurement.
  if q_now then
    st2.q_w, st2.q_x, st2.q_y, st2.q_z = q_now:q1(), q_now:q2(), q_now:q3(), q_now:q4()
  else
    st2.q_w, st2.q_x, st2.q_y, st2.q_z = NAN, NAN, NAN, NAN
  end
  if gyro_n then
    st2.gyro_x_rads, st2.gyro_y_rads, st2.gyro_z_rads = gyro_n:x(), gyro_n:y(), gyro_n:z()
  else
    st2.gyro_x_rads, st2.gyro_y_rads, st2.gyro_z_rads = NAN, NAN, NAN
  end
  if vel_n then
    st2.vel_n_mps, st2.vel_e_mps, st2.vel_d_mps = vel_n:x(), vel_n:y(), vel_n:z()
  else
    st2.vel_n_mps, st2.vel_e_mps, st2.vel_d_mps = NAN, NAN, NAN
  end
  st2.eas_mps = eas or NAN
  st2.fc_mode = vehicle:get_mode()
  -- What ArduPlane asked each surface for, and the servo signal after the
  -- V-tail mix.  A function no channel carries reads PWM 0.
  st2.out_aileron  = SRV_Channels:get_output_scaled(K_AILERON)
  st2.out_elevator = SRV_Channels:get_output_scaled(K_ELEVATOR)
  st2.out_rudder   = SRV_Channels:get_output_scaled(K_RUDDER)
  st2.out_throttle = SRV_Channels:get_output_scaled(K_THROTTLE)
  st2.pwm_aileron  = SRV_Channels:get_output_pwm(K_AILERON) or 0
  st2.pwm_vtail_l  = SRV_Channels:get_output_pwm(K_VTAIL_L) or 0
  st2.pwm_vtail_r  = SRV_Channels:get_output_pwm(K_VTAIL_R) or 0
  -- Publish it.  `eas` is a LOCAL of this function and the actuation block is
  -- a different one, so reading `eas` there resolves to a global, yields nil,
  -- and any term built on it is silently zero - the same shape of bug as
  -- reading a gain from cfg instead of g.  cfg is the per-tick input table
  -- and airspeed is a per-tick input, so it goes there.
  cfg.airspeed_eas = eas
  -- The pitch trim's other per-tick inputs: ArduPlane's throttle output
  -- (percent) and the airspeed below which the trim stops growing - the floor
  -- the speed scaler below uses, MAX(AIRSPEED_MIN, MIN_AIRSPEED_MIN).
  cfg.throttle_out = st2.out_throttle
  local vmin_p = aspd_min_param and aspd_min_param:get()
  cfg.airspeed_floor = math.max(vmin_p or MIN_AIRSPEED_MIN, MIN_AIRSPEED_MIN)
  -- Replicate ArduPlane/Attitude.cpp calc_speed_scaler(). READ ONLY.
  --
  --     airspeed_min = MAX(AIRSPEED_MIN, MIN_AIRSPEED_MIN)
  --     scale_min    = MIN(0.5, SCALING_SPEED / (2.0 * AIRSPEED_MAX))
  --     scale_max    = MAX(2.0, SCALING_SPEED / (0.7 * airspeed_min))
  --     speed_scaler = SCALING_SPEED / airspeed_EAS   (scale_max at ~0 airspeed)
  --     speed_scaler = constrain(speed_scaler, scale_min, scale_max)
  --
  -- The FC reads the SAME ahrs:airspeed_EAS() this script reads, so the value
  -- below matches what it applied, to within the sampling offset within a tick.
  -- With no airspeed estimate ArduPlane scales from throttle instead, which this
  -- does not reproduce: the scaler is then NaN, blank in the log, never a guess.
  --
  -- THE GAIN MULTIPLIER IS THE SQUARE OF THIS. AP_FW_Controller.cpp feeds the
  -- rate PID `desired * scaler * scaler` and `measured * scaler * scaler`, so a
  -- scaler of 1.25 is 1.56x the gain.  Identification must square lua_scaler
  -- before using it.
  local scaler = NAN
  if scaling_param and aspd_min_param and aspd_max_param and eas then
    local ref  = scaling_param:get()
    local vmin = aspd_min_param:get()
    local vmax = aspd_max_param:get()
    if ref and ref > 0.0 and vmin and vmax and vmax > 0.0 then
      local amin = math.max(vmin, MIN_AIRSPEED_MIN)
      local lo = math.min(0.5, ref / (2.0 * vmax))
      local hi = math.max(2.0, ref / (0.7 * amin))
      scaler = (eas > 0.0001) and (ref / eas) or hi
      if scaler < lo then scaler = lo elseif scaler > hi then scaler = hi end
    end
  end
  -- Logged as lua_scaler: the multiplier ArduPlane applied to the rate loop on
  -- THIS tick.  Identification reads it to separate the aircraft's response
  -- from the FC's speed compensation.
  st_msg.speed_scaler = scaler
  -- Per-tick inputs to the pitch trim's rate conversion (see the actuation
  -- block): the scaler above, 0 when absent so no trim is sent, and the
  -- EAS-to-TAS ratio ArduPlane divides the rate loop's FF by.
  cfg.speed_scaler = (scaler == scaler) and scaler or 0.0
  cfg.eas2tas = ahrs:get_EAS2TAS()
  st2.eas2tas = cfg.eas2tas or NAN

  -- ---- YAW / COORDINATION INSTRUMENTATION --------------------------------
  -- Assigned HERE rather than in step() because eas and gyro_n already exist
  -- in this scope, and because these must be written on EVERY tick: a STATE
  -- field the FC skips reports a confident zero in the CSV, indistinguishable
  -- from a real measurement of zero.  A reading the FC could not take goes out
  -- as NaN.
  --
  -- get_accel() is the primary IMU in BODY axes and returns SPECIFIC FORCE,
  -- so level flight reads about (0, 0, -9.81).  accel_y is the coordination
  -- signal - zero in a coordinated turn at ANY bank, and immune to wind, which
  -- the velocity-derived sideslip is not.  The cascade only logs it.  Lift
  -- Vector Pointing reads all three axes: the specific force plus gravity is the
  -- path's acceleration, which its lead projects ahead; with no reading it
  -- flies without the lead.
  local acc = ahrs:get_accel()
  s.accel_valid = (acc ~= nil)
  if acc then
    st_msg.accel_x_mps2 = acc:x()
    st_msg.accel_y_mps2, st_msg.accel_z_mps2 = acc:y(), acc:z()
    cfg.acc_x, cfg.acc_y, cfg.acc_z = acc:x(), acc:y(), acc:z()
  else
    st_msg.accel_x_mps2 = NAN
    st_msg.accel_y_mps2, st_msg.accel_z_mps2 = NAN, NAN
    cfg.acc_x, cfg.acc_y, cfg.acc_z = NAN, NAN, NAN
  end

  -- THE COORDINATED TURN'S OWN YAW RATE at this bank and speed.  The yaw law
  -- reads it as the turning half of its coordination, and the pitch axis as the
  -- turn's body pitch rate (pitch_turn_ff, in the actuation block); both reuse
  -- its airspeed floor and validity.
  --
  -- r = g*sin(roll)*cos(pitch)/V, with V the TRUE airspeed: the turn is
  -- kinematics, and the EAS reads low by EAS2TAS wherever the air is thinner
  -- than at sea level, which would overstate the rate by the same factor.
  --
  -- FLOORED, NOT GATED, on airspeed.  The division makes a small V an
  -- enormous rate: 1 m/s and 45 deg of bank gives 397 deg/s.  ArduPlane floors
  -- the EAS in the same division at AIRSPEED_MIN
  -- (AP_YawController::get_servo_out uses MAX(aspeed, aspd_min)) and this does
  -- the same before converting to true airspeed, so a degraded reading gives a
  -- CONSERVATIVE rate instead of a spike, with no discontinuity at the
  -- threshold.  With no EAS2TAS reading the floored EAS stands in.
  --
  -- airspeed_EAS returns nil (not a pair - see above) when there is no estimate
  -- at all.  That is a different thing from a low reading: it clears
  -- s.coord_valid, which removes the pitch turn term.
  local roll_r  = ahrs:get_roll_rad()
  local pitch_r = ahrs:get_pitch_rad()
  if eas and roll_r and pitch_r then
    -- Floored as ArduPlane floors its own: AIRSPEED_MIN, but never below
    -- MIN_AIRSPEED_MIN, so an AIRSPEED_MIN of 0 cannot divide by zero.
    local v = eas
    local v_floor = math.max(cached_aspd_min, MIN_AIRSPEED_MIN)
    if v < v_floor then v = v_floor end
    local e2t = cfg.eas2tas
    if e2t and e2t > 0.0 then v = v * e2t end
    s.yaw_coord_dps =
      math.deg(9.80665 * math.sin(roll_r) * math.cos(pitch_r) / v)
    s.coord_valid = true
  else
    s.yaw_coord_dps = 0.0
    s.coord_valid = false
  end
  st_msg.yaw_ff_coord_dps = s.coord_valid and s.yaw_coord_dps or NAN

  -- The rolling half of the coordination, p * tan(alpha), is worked out in
  -- the yaw block from the roll rate that tick commands and the path track.lua
  -- took, and is a commanded-tick field (yaw_ff_ari_dps).
end

function update()
  -- Sensors FIRST and in their own pcall: they must not be skipped by a step()
  -- failure, and a fault here must not remove the script either.
  local ok_sens, err_sens = pcall(sample_sensors_and_report_scaler)
  if not ok_sens then script_fault("sensor/scaler pass - " .. tostring(err_sens)) end
  -- pcall so an internal error is reported rather than removing the script.
  -- ArduPilot does NOT retry a failed script, and a removed script sends no
  -- STATE to say why.  step() runs the steering in a pcall of its own, so the
  -- STATE carrying the fault still goes out on the tick of the error.
  local ok, err = pcall(step)
  if not ok then script_fault(tostring(err)) end
  return update, LOOP_MS
end

local function init()
  load_default_gains()
  port = serial:find_serial(SERIAL_INSTANCE)
  if not port then
    gcs:send_text(3, "balloon chaser: no scripting serial port (SERIALx_PROTOCOL=28)")
    return
  end
  -- 921600, matching the pyserial baud in main.py.  BOTH SIDES MUST AGREE.
  -- At 115200 the 50 Hz frame ceiling is 230 B and STATE was already 172, so
  -- there was no room to report FC-side values instead of recomputing them on
  -- the companion.  SERIALx_BAUD must match on the FC.
  port:begin(921600)
  port:set_flow_control(0)

  -- Bound READ-ONLY so lua_scaler can report the gain the FC is applying.
  -- AIRSPEED_MIN/MAX are needed too: ArduPlane's clamp is computed from them,
  -- not fixed (ArduPlane/Attitude.cpp, calc_speed_scaler).  A name this
  -- firmware does not have fails init() and is reported, never skipped
  -- silently.
  scaling_param = Parameter()
  if not scaling_param:init('SCALING_SPEED') then scaling_param = nil end
  aspd_min_param = Parameter()
  if not aspd_min_param:init('AIRSPEED_MIN') then aspd_min_param = nil end
  aspd_max_param = Parameter()
  if not aspd_max_param:init('AIRSPEED_MAX') then aspd_max_param = nil end
  if not (scaling_param and aspd_min_param and aspd_max_param) then
    gcs:send_text(3, "balloon chaser: SCALING_SPEED/AIRSPEED_MIN/AIRSPEED_MAX not "
                  .. "readable - lua_scaler will be blank, no pitch trim")
  end
  -- The pitch rate loop's FF and P, for the pitch trim's rate conversion.
  ptch_ff_param = Parameter()
  if not ptch_ff_param:init('PTCH_RATE_FF') then ptch_ff_param = nil end
  ptch_p_param = Parameter()
  if not ptch_p_param:init('PTCH_RATE_P') then ptch_p_param = nil end
  if not (ptch_ff_param and ptch_p_param) then
    gcs:send_text(3, "balloon chaser: PTCH_RATE_FF/P not readable - no pitch trim")
  end

  -- THE YAW LAW'S THREE PRECONDITIONS, CHECKED ONCE.  All three fail SILENTLY:
  -- the rudder simply does not move, and every other axis keeps flying, so the
  -- symptom is a pointing law that appears to run and does nothing.
  --
  --   YAW_RATE_ENABLE  selects the rate controller over ArduPlane's own
  --                    coordinated-yaw path.  At 0 the nav_scripting branch
  --                    writes rudder = 0 outright, offset included.
  --   ACRO_YAW_RATE    is the ceiling the FC puts on the yaw argument of
  --                    set_target_throttle_rate_rpy.  At 0 every command is
  --                    constrained to 0.  It is also what this law clamps its
  --                    command to.
  --   YAW_RATE_FF      stands in for the tuned P/I/D/FF set.  With them zeroed
  --                    the rate controller outputs nothing.
  --
  -- Reported, not enforced: this script does not own the FC's tune, and a
  -- startup that silently rewrote it would hide the same fault one level down.
  local yre = param:get('YAW_RATE_ENABLE')
  local ayr = param:get('ACRO_YAW_RATE')
  local yff = param:get('YAW_RATE_FF')
  if yre == nil or yre < 0.5 then
    gcs:send_text(3, "balloon chaser: YAW_RATE_ENABLE is not 1 - rudder will not move")
  elseif ayr == nil or ayr <= 0.0 then
    gcs:send_text(3, "balloon chaser: ACRO_YAW_RATE is 0 - every yaw command clamps to 0")
  elseif yff == nil or yff == 0.0 then
    gcs:send_text(3, "balloon chaser: YAW_RATE_* untuned - rate controller outputs nothing")
  end

  gcs:send_text(6, string.format("balloon chaser: v%d, %d Hz, SIM CONFIG (GUIDE stale after %d ms)",
                                 proto.PROTO_VERSION, LOOP_HZ, C.GUIDE_STALE_MS))
end

init()
return update, LOOP_MS
