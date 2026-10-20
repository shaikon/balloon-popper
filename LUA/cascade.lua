-- Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
--
-- cascade.lua - the balloon chaser control law's dispatch layer.
--
-- Runs the roll law and the pitch law, applies the rate clips, and converts
-- everything into the two body-rate commands that leave the aircraft.  It is
-- wiring, but it is the widest branch surface in the system and the branches
-- are where behaviour actually lives.
--
-- ============================================================================
-- ROLL: THE BANK FED BACK IS THE PROXY
-- ============================================================================
-- The roll law feeds back the bank proxy asin(R[1][2]), the lift vector's
-- sideways tilt from the vertical: gravity-referenced and analytic through
-- vertical, where the Euler bank has a pole, and fading to 0 as |pitch| -> 90,
-- where "bank" stops having lateral meaning.  The proxy is the feedback at BOTH
-- points - the bank handed to compute_roll_command and the setpoint error
-- turned into a rate - so the inner anti-windup rate predictor agrees with the
-- setpoint->rate conversion further down.  PITCH is one unified law at every
-- attitude.
--
-- PLAN MODE.  While the Curve Flyer runs a PLAN (curve_flyer.lua) it hands
-- the law the balloon to turn the path toward and the bank to hold, in the
-- proxy's terms.  The PLAN's bank, within the bank limit, is the roll
-- setpoint: bank-to-turn does not run and roll-to-plane has no share.  The
-- pitch law closes on the balloon's error, and the roll setpoint becomes a
-- rate as it always does.  The Curve Planner flies a model of this mode
-- (curve_planner.py), so a change here is a change there in the same commit.

local G  = require("geometry")
local T  = require("track")
local CR = require("control_roll")
local CP = require("control_pitch")

local M = {}

local abs, deg, rad = math.abs, math.deg, math.rad
local sin, cos, asin, atan, sqrt = math.sin, math.cos, math.asin, math.atan, math.sqrt
local max = math.max
local NAN = 0 / 0

local function clip(v, lo, hi)
  if v < lo then return lo elseif v > hi then return hi end
  return v
end

-- 0 at or below `start`, 1 at `full`, a straight line between; with `full`
-- at or below `start`, a step at `start`.
local function ramp_share(x, start, full)
  local span = full - start
  if span > 0.0 then return clip((x - start) / span, 0.0, 1.0) end
  return (x >= start) and 1.0 or 0.0
end

local function copysign(mag, x)
  if x < 0.0 or (x == 0.0 and 1.0 / x < 0.0) then return -abs(mag) end
  return abs(mag)
end

-- Below this sine of the balloon's angle off the nose the balloon is dead ahead
-- or dead astern, and the turn axis (nose x balloon) has no direction.
local TURN_AXIS_MIN_SIN = 1e-6
-- Below this a tick has no duration to differentiate the clock angle over.
local DT_MIN_S = 1e-6

-- scratch table, allocated once at load and filled in place every tick
local _pg  = {}        -- the pitch law's gains, in compute_bodyplane_pitch_rate's names

-- ---------------------------------------------------------------------------
--- Run the cascade for one tick.
---
--- `s`   track state   `cs` CascadeState   `g` ControlGains
--- `cfg` per-tick inputs     `out` diagnostics, filled not allocated
---
--- `pan_err_deg` and `e_pitch_rad` are track.lua's two errors (the first in
--- the cascade's turn-right sense); `bx, by, bz` the balloon in body axes.
--- On a tick the Curve Flyer runs a PLAN, `plan_bank_deg` is its bank in the
--- proxy's terms, and `e_pitch_rad` and `bx, by, bz` are its aim point's (PLAN
--- MODE); else it is nil.
---
--- Returns omega_x_cmd_dps, omega_y_cmd_dps.
function M.apply_control_law(pan_err_deg, e_pitch_rad, dt, roll, pitch,
                             bx, by, bz, s, cs, g, cfg, out, plan_bank_deg)
  local acro_roll_rate  = cfg.acro_roll_rate_dps
  local acro_pitch_rate = cfg.acro_pitch_rate_dps

  -- ---- bank proxy + rate decomposition ----------------------------------
  -- down_body = R_wb * [0,0,1]; down_body[1] = sin(phi)cos(theta).  asin of it
  -- is the lift-vector lateral tilt - the quantity that actually generates a
  -- bank-to-turn, and the roll law's feedback, logged as bank_proxy_deg.
  local r00,r01,r02,r10,r11,r12,r20,r21,r22 =
    G.quat_to_rotation_matrix(cfg.qw, cfg.qx, cfg.qy, cfg.qz)
  local bank_proxy_rad = asin(clip(r12, -1.0, 1.0))     -- R_wb[1][2], 0-based
  out.log_bank_proxy_deg = deg(bank_proxy_rad)

  -- Exact two-term split of the proxy bank rate:
  --   bank_rate = ( cos(roll)cos(pitch)*p + sin(pitch)*r ) / cos(bank)
  -- The first term is what the roll command buys; the second is driven by yaw
  -- rate alone and is commanded by nothing.  Since r carries the sign of the
  -- bank in a turn, sin(pitch) decides whether it drives the bank back toward
  -- level (nose down) or away from it (nose up) - so the roll loop is damped in
  -- a descent and eroded in a climb at the SAME gain.
  local _cb = cos(bank_proxy_rad)
  if abs(_cb) <= 1e-3 then
    _cb = copysign(1e-3, (_cb ~= 0.0) and _cb or 1.0)
  end
  out.log_bank_rate_cmd_dps = deg(cos(roll) * cos(pitch) * cfg.rate_p / _cb)
  out.log_bank_rate_coupling_dps = deg(sin(pitch) * cfg.rate_r / _cb)

  -- cos(roll)cos(pitch): "down-ness" of body z.  Negative means inverted.  The
  -- proxy alone cannot see inversion - sin(roll) is symmetric about 90 deg, so
  -- belly-up reads as a small bank - and it passes through zero at vertical,
  -- where inversion is undefined.
  s.log_upright = cos(roll) * cos(pitch)
  out.log_upright = s.log_upright

  out.log_sideslip_deg, out.log_aoa_deg =
    T.flow_angles(r00,r01,r02,r10,r11,r12,r20,r21,r22,
                  cfg.vel_valid and cfg.vel_n or nil, cfg.vel_e, cfg.vel_d)

  -- ---- ROLL-TO-PLANE: where the balloon sits around the nose ---------------
  -- Looking along the nose, the balloon sits at a clock position,
  -- atan2(b_y, -b_z) in body axes: 0 over the canopy (12 o'clock), +90 to the
  -- right, +-180 under the belly.  At 12 or 6 the balloon lies in the plane the
  -- pitch rate swings the nose in, and the clock angle is the roll still to go,
  -- with no bank setpoint, no bank measurement, nothing measured against the
  -- horizon.  The rate loop rolls the aircraft about the nose, so a roll moves
  -- the balloon round this clock by exactly the angle rolled.
  --
  -- The flight path is not the nose: it sits off it by the angle of attack and
  -- the sideslip, and a pull swings the PATH in the plane through the path and
  -- the body z axis.  Wherever the two part, a balloon at 12 or 6 round the
  -- nose can sit off to the side of where a pull takes the path, and the pitch
  -- law, which reads the path (track.lua), closes only the part of the miss
  -- in that plane.  Bank-to-turn reads the path too, so the two roll laws share
  -- one zero only while the path lies in the body's plane of symmetry.  Near
  -- the nose the clock swings for small moves (5 deg right of the nose at its
  -- height reads 3 o'clock, 90 deg still to roll), and roll-to-plane rolls for
  -- every swing.
  --
  -- Reading this clock round the path, like every other balloon error, is the
  -- tempting change and the wrong one.  For a balloon between the nose and the
  -- path the two clocks ask for opposite rolls: on a steep path the wing lifts
  -- with the nose below the path, so that balloon is above the nose and below
  -- the path.  The wing is holding a pull there, against gravity across the
  -- path and the propeller's push the low nose aims at the belly, and a roll
  -- tilts it with the wings.  The nose's roll tilts it toward the balloon.  The
  -- path's tilts it away: the path slides off before the pitch law can unload
  -- the pull to push, the balloon reads further out round the path, and that
  -- clock asks for more of the same roll.  A pull hard enough to lift the nose
  -- above the path swaps the two (ARCHITECTURE.md, "Roll-to-plane").
  --
  -- On a steep flight path, banking toward the balloon is the wrong law: its
  -- sideways error cannot tell right-and-above from right-and-below, and a
  -- balloon under the belly makes it run away, over the top.
  --
  -- Which law gravity calls for is set by the share of gravity acting ACROSS
  -- the flight path, the cosine of the path's angle above or below the
  -- horizon.  Moving level it is all of it: the lift must hold the aircraft up
  -- while it turns, which banking toward the balloon does and roll-to-plane,
  -- knowing nothing of gravity, does not.  Moving vertically it is none:
  -- gravity only changes the speed, and the turn is pure geometry.  The bank
  -- does not enter it - knife-edge on a level path is all gravity across the
  -- path, and banking toward the balloon is what rolls the lift back up - and
  -- neither does the nose, which sits off the path by the angle of attack and
  -- the sideslip.
  --
  -- So the blend is one straight line, the path's angle above or below the
  -- horizon: roll-to-plane's share w is 0 at plane_steep_start_deg and 1 at
  -- plane_steep_full_deg, and bank-to-turn has the rest.  With the full at or
  -- below the start the line is a step at the start.  The angle is the one
  -- path track.lua took from this tick's velocity, where the nose stands in
  -- below FLIGHT_PATH_MIN_SPEED_MPS; the clock reads the nose.  path_share is
  -- kept in cs.steep_share for the yaw block, whose pointing schedule reads
  -- the same number.
  --
  -- axis_up, the world-vertical part of the unit turn axis nose x balloon (1 a
  -- heading change, 0 a climb or dive), is logged, not flown: it says which
  -- job a turn was.
  --
  -- The balloon is brought to whichever of 12 and 6 is nearer.  The pitch law
  -- already pulls toward a balloon above the path and pushes toward one below,
  -- so it is unchanged.  The choice holds until the other end is nearer by
  -- plane_side_hold_deg: at 3 and 9 o'clock both ends are equally far, and a
  -- free choice there swings the command from full one way to full the other.
  local path_deg, path_from_vel = s.path_deg, s.path_from_vel
  -- The angle between two directions is atan2(|a x b|, a . b): accurate at
  -- every angle, front to back, where asin cannot pass 90 and acos loses
  -- precision near 0 and 180.  The nose is (1, 0, 0) in body axes.
  local off_nose_deg = deg(atan(sqrt(by * by + bz * bz), bx))
  local clock_deg = deg(atan(by, -bz))
  if cs.plane_side == 0 then
    cs.plane_side = (abs(clock_deg) <= 90.0) and 1 or -1
  elseif cs.plane_side == 1 and abs(clock_deg) > 90.0 + g.plane_side_hold_deg then
    cs.plane_side = -1
  elseif cs.plane_side == -1 and abs(clock_deg) < 90.0 - g.plane_side_hold_deg then
    cs.plane_side = 1
  end
  local plane_err_deg = clock_deg
  if cs.plane_side == -1 then
    plane_err_deg = (clock_deg > 0.0) and (clock_deg - 180.0) or (clock_deg + 180.0)
  end
  -- Under a PLAN roll-to-plane has no share: the PLAN sets the bank.
  local path_share = 0.0
  if plan_bank_deg == nil then
    path_share = ramp_share(abs(path_deg), g.plane_steep_start_deg,
                            g.plane_steep_full_deg)
  end
  local w = path_share
  cs.steep_share = path_share
  -- nose x balloon in body axes is (0, -bz, by); its world-vertical part is
  -- row 2 of the body->world rotation (column 2 of R) applied to it.
  local axis_sin = sqrt(by * by + bz * bz)
  local axis_up = 1.0
  if axis_sin > TURN_AXIS_MIN_SIN then
    axis_up = clip(abs(r22 * by - r12 * bz) / axis_sin, 0.0, 1.0)
  end

  -- Roll-to-plane's feedforward: the balloon's own drift round the clock - the
  -- clock angle's rate plus the aircraft's roll rate, since a roll right turns
  -- it back by the angle rolled - low-passed at plane_ff_lpf_rate.
  -- Without it the proportional law trails a drifting balloon by drift/gain.
  --
  -- Its derivative term: the clock error's own rate, low-passed at
  -- plane_kd_rate, times plane_kd.  Unlike the drift it keeps the roll's share,
  -- so it takes rate off a roll that is already carrying the balloon to its
  -- end.  It reads the clock's rate, which is the error's except at a 12 <-> 6
  -- change, where the error steps by 180 and the clock does not.
  --
  -- Both filters are held at zero, still re-seeding, while roll-to-plane has no
  -- share: a filter that ran there would carry what it read into the first tick
  -- roll-to-plane flies.  Near the nose the clock swings for tiny moves, and so
  -- do both rates.
  if w > 0.0 and cs.plane_clock_prev ~= nil and dt > DT_MIN_S then
    local d_clock = (clock_deg - cs.plane_clock_prev + 180.0) % 360.0 - 180.0
    local clock_rate_dps = d_clock / dt
    local drift_dps = clock_rate_dps + deg(cfg.rate_p)
    local a = g.plane_ff_lpf_rate * dt / (1.0 + g.plane_ff_lpf_rate * dt)
    cs.plane_drift_lpf = cs.plane_drift_lpf + a * (drift_dps - cs.plane_drift_lpf)
    local ad = g.plane_kd_rate * dt / (1.0 + g.plane_kd_rate * dt)
    cs.plane_err_rate_lpf = cs.plane_err_rate_lpf
                          + ad * (clock_rate_dps - cs.plane_err_rate_lpf)
  else
    cs.plane_drift_lpf = 0.0
    cs.plane_err_rate_lpf = 0.0
  end
  cs.plane_clock_prev = clock_deg
  local plane_ff_dps = g.plane_ff_gain * cs.plane_drift_lpf
  local plane_d_dps = g.plane_kd * cs.plane_err_rate_lpf

  out.log_plane_off_nose_deg = off_nose_deg
  out.log_plane_err_deg = plane_err_deg
  out.log_plane_axis_up = axis_up
  out.log_plane_path_share = path_share
  out.log_plane_path_deg = path_deg
  out.log_plane_path_from_vel = path_from_vel and 1 or 0
  out.log_plane_side = cs.plane_side
  out.log_plane_drift_dps = cs.plane_drift_lpf
  out.log_plane_ff_dps = plane_ff_dps
  out.log_plane_err_rate_dps = cs.plane_err_rate_lpf
  out.log_plane_d_dps = plane_d_dps

  -- ---- ROLL --------------------------------------------------------------
  -- bank_lim_deg goes in unchanged, and at steep pitch it is out of the
  -- proxy's reach (the proxy cannot exceed asin(cos(pitch))).  That is how the
  -- law steers near vertical: the roll goes on until the sideways
  -- error is gone, lining the balloon up with the pull.  Translating the limit
  -- into the proxy's terms so it caps the real bank stops the steep roll-over
  -- and the dive steering with it.
  --
  -- The integrator is held while roll-to-plane has any share: the sideways
  -- error it integrates is not what that share is closing, and what it winds
  -- up there it would bank the moment the hand-over ends.
  --
  -- Under a PLAN the setpoint is the PLAN's bank within the limit, and
  -- bank-to-turn does not run: its integrator is inactive and is cleared,
  -- and its derivative re-seeds on the first tick it flies again.
  cs.plan_set_bank = plan_bank_deg ~= nil
  if cs.plan_set_bank then
    cs.pan_integrator = 0.0
    cs.pan_err_deriv_init = false
    cs.log_p_term_roll, cs.log_d_term_roll = NAN, NAN
    cs.log_roll_cmd_presat = plan_bank_deg
    s.requested_roll_rad = rad(clip(plan_bank_deg, -g.bank_lim_deg, g.bank_lim_deg))
  else
    s.requested_roll_rad = CR.compute_roll_command(
      pan_err_deg, dt, deg(bank_proxy_rad),
      cs, g, acro_roll_rate, g.bank_lim_deg, w > 0.0)
  end
  -- With no share of the roll command left to the branch, or no roll gain to
  -- turn its bank into a rate, bank-to-turn reaches nothing: its integrator
  -- is inactive and is cleared rather than held.
  if (1.0 - w) <= 0.0 or g.roll_gain <= 0.0 then
    cs.pan_integrator = 0.0
  end

  -- ---- PITCH: one unified law, both regimes ------------------------------
  _pg.gain = g.pitch_gain
  _pg.ki = g.pitch_ki
  _pg.windup_limit = g.windup_limit_pitch
  _pg.deadband_deg = g.pitch_deadband_deg
  _pg.kd = g.pitch_kd
  _pg.kd_rate = g.pitch_kd_rate

  local omega_y_dps = CP.compute_bodyplane_pitch_rate(
    e_pitch_rad, dt, cs, _pg, acro_pitch_rate)

  out.log_theta_ip_deg = deg(e_pitch_rad)
  s.requested_pitch_rad = pitch + e_pitch_rad    -- continued-Euler, may pass -90

  -- ---- roll setpoint -> body rate ---------------------------------------
  local roll_angle_err_deg = deg(s.requested_roll_rad - bank_proxy_rad)
  local omega_x_dps = roll_angle_err_deg * max(0.0, g.roll_gain)
  cs.log_roll_err_deg = roll_angle_err_deg

  -- ---- the blend ---------------------------------------------------------
  -- 1 - w of the branch's rate and w of roll-to-plane's - its own gain on the
  -- degrees still to roll, its integrator, its derivative term and its
  -- feedforward.  At w = 0 this is the branch's command exactly, at w = 1
  -- roll-to-plane's.
  --
  -- The integrator is in deg/s of roll, so a Ki change moves its rate of
  -- growth, not its value.  It holds while the command it would join is
  -- already at the ACRO roll rate its way, and owns at most plane_windup of
  -- that rate; it reaches the command as w of that, with the rest of
  -- roll-to-plane's rate.  It is cleared while roll-to-plane has no share, for
  -- the feedforward's reason, and at Ki 0, so switching it off removes it
  -- rather than leaving the aircraft flying whatever it last reached.  A
  -- 12 <-> 6 change keeps it: the clock error's rate is the same at either end.
  local plane_p_dps = plane_err_deg * max(0.0, g.plane_gain)
  local plane_i_frozen = false
  if w > 0.0 and g.plane_ki ~= 0.0 then
    local tentative = (1.0 - w) * omega_x_dps
                    + w * (plane_p_dps + cs.plane_i_dps + plane_d_dps + plane_ff_dps)
    plane_i_frozen = abs(tentative) >= acro_roll_rate
                     and (tentative > 0.0) == (plane_err_deg > 0.0)
    if not plane_i_frozen then
      cs.plane_i_dps = cs.plane_i_dps + g.plane_ki * plane_err_deg * dt
    end
    local i_ceil = acro_roll_rate * g.plane_windup
    cs.plane_i_dps = clip(cs.plane_i_dps, -i_ceil, i_ceil)
  else
    cs.plane_i_dps = 0.0
  end
  out.log_plane_i_dps = cs.plane_i_dps
  out.log_plane_i_frozen = plane_i_frozen and 1 or 0
  omega_x_dps = (1.0 - w) * omega_x_dps
              + w * (plane_p_dps + cs.plane_i_dps + plane_d_dps + plane_ff_dps)

  -- Frequency-ID excitation ADDS to the rate on the axis EXCITE names, after
  -- the blend - it is a closed-loop probe, and living OUTSIDE the laws is
  -- what stops one of them from silently dropping it.  plant-ID's
  -- REPLACE override is applied after the clip below.
  omega_x_dps = omega_x_dps + (cfg.freqid_inject_dps or 0.0)
  omega_y_dps = omega_y_dps + (cfg.freqid_inject_pitch_dps or 0.0)

  -- Capture the demand BEFORE the envelope clip.  Reported as
  -- omega_*_presat_dps: alongside the post-clip command it says whether the
  -- ACRO envelope is cutting the law off.  Equal means the law got what it
  -- asked for; different means it is clipped and no gain change will help.
  cs.log_omega_x_presat = omega_x_dps
  cs.log_omega_y_presat = omega_y_dps

  omega_x_dps = clip(omega_x_dps, -acro_roll_rate, acro_roll_rate)
  omega_y_dps = clip(omega_y_dps, -acro_pitch_rate, acro_pitch_rate)

  -- plant-ID REPLACES the named axis's command outright (open loop on that
  -- axis); freqid, applied above, ADDS to it (loop stays closed).
  -- The law's own command on that axis then reaches nothing, so the axis's
  -- integrators are inactive and are cleared rather than held.
  if cfg.plantid_override_valid then
    omega_x_dps = cfg.plantid_roll_override_dps
    cs.pan_integrator = 0.0
    cs.plane_i_dps = 0.0
    out.log_plane_i_dps = 0.0
  end
  if cfg.plantid_pitch_override_valid then
    omega_y_dps = cfg.plantid_pitch_override_dps
    cs.pitch_integrator = 0.0
  end

  return omega_x_dps, omega_y_dps
end

return M
