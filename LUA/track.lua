-- Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
--
-- track.lua - geometry layer of the steering core.
--
-- The per-tick geometry and its helpers:
--   rear_hemisphere_pan_err   C-04 continuation past +-90 deg
--   flow_angles               sideslip / in-plane flow from velocity
--   track_world_balloon        the tick: world balloon -> cascade inputs
--
-- On the FC this layer builds its rotation from ahrs:get_quaternion(), read in
-- the same tick as the gimbal encoders, rather than from telemetry.  The
-- quaternion has no pole at +-90 deg pitch, which Euler angles do.
--
-- ============================================================================
-- WHY THE PITCH ERROR IS NOT AN EULER ANGLE
-- ============================================================================
--   e_pitch = atan2(-t'_z, hypot(t'_x, t'_y))      (balloon in the path frame)
--
-- e_pitch is the balloon's ELEVATION above the path frame's horizontal plane
-- (see THE PATH FRAME in track_world_balloon).  It stays well-conditioned
-- through the vertical - the entire reason the aircraft can fly the full
-- sphere.  The Euler pitch DOES degenerate at +-90; no error the laws close
-- uses it.  e_pitch takes the FULL horizontal leg, not t'_x alone: the lateral
-- offset is the ROLL axis's to null, and a t'_x-only leg would let a balloon
-- abeam inflate the pitch demand toward 90 deg while the turn is still
-- bringing it forward.  The non-negative leg also folds the rear hemisphere
-- onto the front with no seam at t'_x = 0.
--
-- ============================================================================
-- REAR-HEMISPHERE CONTINUATION (C-04)
-- ============================================================================
-- The pan error is asin(t'_y), the balloon's angle to the side of the path.  It
-- spans only +-90 deg, so on its own it FOLDS the rear hemisphere onto the
-- front and reads a balloon behind the path as converged - the aircraft would
-- sit happily flying 180 deg away.  rear_hemisphere_pan_err continues the
-- error past 90 deg, blending by w = min(1, -tx), with (tx, ty) the balloon in
-- the path frame.
--
-- The tie-break matters: with the balloon near-exactly behind the path, ty ~ 0
-- gives no side information, so the roll sign is used instead, and failing
-- that +1.
-- Without it the sign chatters and the aircraft cannot decide which way to turn.

local C  = require("constants")
local G  = require("geometry")

local M = {}

local NAN = 0 / 0
local abs, deg = math.abs, math.deg
local asin, atan, sqrt = math.asin, math.atan, math.sqrt
local min = math.min

local function clip(v, lo, hi)
  if v < lo then return lo elseif v > hi then return hi end
  return v
end

local function copysign(mag, x)
  if x < 0.0 or (x == 0.0 and 1.0 / x < 0.0) then return -abs(mag) end
  return abs(mag)
end

-- Below this sine of the flight path's angle to the body z axis the path runs
-- along the belly-canopy axis and the path frame has no sideways direction.
local PATH_FRAME_MIN_SIN = 1e-3

-- ---------------------------------------------------------------------------
--- Continue the pan error into the rear hemisphere.
function M.rear_hemisphere_pan_err(lateral_err_deg, tx, ty, att_roll_rad)
  local sign
  if abs(ty) > C.REAR_AZIMUTH_TIE_TY then
    sign = copysign(1.0, ty)
  elseif abs(att_roll_rad) > C.REAR_AZIMUTH_TIE_ROLL_RAD then
    sign = copysign(1.0, att_roll_rad)
  else
    sign = 1.0
  end
  local w = min(1.0, -tx)
  return lateral_err_deg + w * (sign * 90.0 - lateral_err_deg)
end

-- ---------------------------------------------------------------------------
--- Sideslip and in-plane flow angles from the velocity vector, degrees.
--- Returns (nil, nil) when velocity is unavailable or the aircraft is
--- essentially stationary - below 1 m/s the angles are pure noise.
function M.flow_angles(r00,r01,r02,r10,r11,r12,r20,r21,r22, vn, ve, vd)
  if vn == nil then return nil, nil end
  local sp = sqrt(vn * vn + ve * ve + vd * vd)
  if sp < 1.0 then return nil, nil end
  local bx, by, bz = G.rotate(r00,r01,r02,r10,r11,r12,r20,r21,r22, vn, ve, vd)
  return deg(asin(clip(by / sp, -1.0, 1.0))), deg(atan(bz, bx))
end

-- ---------------------------------------------------------------------------
--- One geometry tick.
---
--- `roll`, `pitch` the Euler attitude; `qw..qz` the same attitude as
--- ArduPilot's quaternion, which the rotation is built from.
--- `s`   carried state (balloon_chaser_fc.lua's `s`)
--- `out` caller-supplied table for diagnostics; filled, never allocated
---
--- Returns pan_err_deg, e_pitch_rad, body_x, body_y, body_z - exactly the
--- inputs the cascade needs - and leaves this tick's flight path in `s` for
--- it: path_from_vel, path_deg (the path's angle to the horizon),
--- path_n/e/d (the path's world direction), path_bx..path_zz (the path frame
--- in body axes), path_speed, down_bx/by/bz (world down in body axes),
--- path_turn_up_dps and
--- path_turn_right_dps (the path's own turn since the previous tick),
--- balloon_path_x/y/z (the balloon in the path frame), balloon_elev_deg (its
--- elevation above the horizon) and, while s.hold_valid, hold_path_x/y/z (the
--- line the Curve Flyer flies toward, in the path frame).  The
--- caller then invokes the control law; splitting it this way keeps the
--- geometry independently testable.
function M.track_world_balloon(wx, wy, wz, roll, pitch, qw, qx, qy, qz,
                              s, cfg, out)
  local r00,r01,r02,r10,r11,r12,r20,r21,r22 =
    G.quat_to_rotation_matrix(qw, qx, qy, qz)

  local bx, by, bz = G.rotate(r00,r01,r02,r10,r11,r12,r20,r21,r22, wx, wy, wz)

  local _, tilt_deg = G.vector_to_pan_tilt(bx, by, bz)

  -- The balloon's elevation above the horizon, logged as los_elev_deg; a
  -- run-in ends on it (curve_flyer.lua).
  s.balloon_elev_deg = deg(atan(-wz, sqrt(wx * wx + wy * wy)))
  out.log_los_elev_deg = s.balloon_elev_deg

  -- ---- THE FLIGHT PATH, once per tick, before anything reads it -----------
  -- ALWAYS ON.  Where the aircraft is actually GOING: the FC's velocity, this
  -- tick's, as read.  Every balloon error below is measured from it, and so are
  -- the blend's two shares in cascade.lua, which read what this block leaves
  -- in `s`.  There is no meaningful configuration in which the aircraft's own
  -- direction of travel should be replaced by an assumed zero.
  --
  -- NO MODE TERM.  The path is a MEASUREMENT of how the aircraft is moving and
  -- does not depend on where the balloon vector came from.  A mode that forced
  -- it onto the nose would aim the NOSE while every other mode aims the FLIGHT
  -- PATH: a per-mode control law, which C-03 forbids.
  --
  -- Taken as read: no filter, no glitch guard, no clip.  The attitude and
  -- velocity are ArduPilot's own AHRS estimates, already smoothed at the
  -- source (EKF3's output filter, EK3_TAU_OUTPUT), so a filter here only adds
  -- lag, and the laws aim the ESTIMATED path, so a lagging, held or clipped
  -- estimate is flown as flight-path error.  The angle between nose and path
  -- is not a slow trim: a hard pull swings it by several degrees within a
  -- second.
  --
  -- GROUND-RELATIVE.  The balloon is treated as fixed to the ground, so the
  -- path that has to meet it is the path over the ground, and that is the path
  -- the p-test verdict scores.  In a crosswind the path includes the crab
  -- angle, and the laws steer the ground track onto the balloon.
  --
  -- Never subtract ahrs:wind_estimate() here.  EKF3 fuses a synthetic
  -- zero-sideslip measurement whenever the aircraft flies forward, so it books
  -- any steady sideslip as wind.  A path taken against that wind converges to
  -- the nose direction and hides the airframe's steady sideslip from the laws.
  -- No EKF3 parameter stops the wind states absorbing it: EK3_BETA_MASK
  -- governs only the non-wind states.
  --
  -- Below FLIGHT_PATH_MIN_SPEED_MPS, or with no velocity, the direction is
  -- noise and the nose stands in: the path is (1, 0, 0) in body axes and its
  -- angle to the horizon is the Euler pitch.
  local px, py, pz = 1.0, 0.0, 0.0
  s.path_from_vel = false
  s.path_deg = deg(pitch)
  s.path_n, s.path_e, s.path_d = r00, r01, r02
  s.vel_marker_rad = 0.0
  s.lat_marker_rad = 0.0
  s.lat_marker_valid = false
  local vn, ve, vd, v_horiz, v_speed = 0.0, 0.0, 0.0, 0.0, 0.0
  if cfg.vel_valid then
    vn, ve, vd = cfg.vel_n, cfg.vel_e, cfg.vel_d
    v_horiz = sqrt(vn * vn + ve * ve)
    v_speed = sqrt(v_horiz * v_horiz + vd * vd)
  end
  if v_speed > C.FLIGHT_PATH_MIN_SPEED_MPS then
    local gamma = atan(-vd, v_horiz)
    s.path_deg = deg(gamma)
    s.path_from_vel = true
    s.path_n, s.path_e, s.path_d = vn / v_speed, ve / v_speed, vd / v_speed
    local vbx, vby, vbz = G.rotate(r00,r01,r02,r10,r11,r12,r20,r21,r22, vn, ve, vd)
    px, py, pz = vbx / v_speed, vby / v_speed, vbz / v_speed

    -- The path's two angles off the nose, logged as the markers: vertical in
    -- the body pitch plane, lateral in the body's horizontal plane.  The laws
    -- read the path frame below, not these.
    s.vel_marker_rad = atan(-vbz, vbx)
    s.lat_marker_rad = atan(vby, vbx)
    s.lat_marker_valid = true
  end

  -- ---- THE PATH FRAME ------------------------------------------------------
  -- The balloon as seen looking along the path: x' along the path; y' along
  -- z_body x x', square to both the path and the belly-canopy axis; z' =
  -- x' x y', toward the belly.  The x'-z' plane therefore holds the path and
  -- the body z axis - the plane a pull swings the path in - and every error read
  -- here uses the formulas the nose frame would use: to the side asin(t'_y), up
  -- or down atan2(-t'_z, hypot(t'_x, t'_y)), off the path
  -- atan2(hypot(t'_y, t'_z), t'_x).  With the path on the nose it IS the body
  -- frame, so below FLIGHT_PATH_MIN_SPEED_MPS these are the nose's.
  -- Roll-to-plane's clock is the one balloon error read in body axes instead,
  -- round the nose (cascade.lua).
  --
  -- Yawing the nose turns the balloon and the path together about the body z
  -- axis the frame is built on, so no error read here moves with the rudder,
  -- and the two axes cannot argue through it.  Roll-to-plane's clock, read
  -- round the nose, does: a yaw turns the balloon round the nose.
  --
  -- A path along the body z axis (falling flat, belly or canopy first) leaves
  -- y' without a direction; the nose stands in for the frame there, while the
  -- path's angle to the horizon still reads the velocity.
  local h = sqrt(px * px + py * py)
  local nose_frame = h < PATH_FRAME_MIN_SIN
  if nose_frame then
    px, py, pz, h = 1.0, 0.0, 0.0, 1.0
  end
  local yx, yy = -py / h, px / h
  local zx, zy, zz = -pz * px / h, -pz * py / h, h
  -- The frame itself in body axes, the speed it was built from, and world
  -- down in body axes, for Lift Vector Pointing (path_attitude.lua), which
  -- reads this frame rather than building a second one.
  s.path_bx, s.path_by, s.path_bz = px, py, pz
  s.path_yx, s.path_yy = yx, yy
  s.path_zx, s.path_zy, s.path_zz = zx, zy, zz
  s.path_speed = v_speed
  s.down_bx, s.down_by, s.down_bz = r02, r12, r22
  local tpx = bx * px + by * py + bz * pz
  local tpy = bx * yx + by * yy
  local tpz = bx * zx + by * zy + bz * zz
  s.balloon_path_x, s.balloon_path_y, s.balloon_path_z = tpx, tpy, tpz
  -- World up (NED (0, 0, -1)) in the same frame, across the path: the bank
  -- against the horizon is the canopy's (-z') angle round the path from it,
  -- atan2(-up'_y, -up'_z), and its length is how far the path is from vertical.
  local ubx, uby, ubz = -r02, -r12, -r22
  s.up_path_y = ubx * yx + uby * yy
  s.up_path_z = ubx * zx + uby * zy + ubz * zz
  -- The line the Curve Flyer flies toward (s.hold_valid, world
  -- s.hold_n/e/d), read in the same frame as the balloon.
  if s.hold_valid then
    local hx, hy, hz = G.rotate(r00,r01,r02,r10,r11,r12,r20,r21,r22,
                                s.hold_n, s.hold_e, s.hold_d)
    s.hold_path_x = hx * px + hy * py + hz * pz
    s.hold_path_y = hx * yx + hy * yy
    s.hold_path_z = hx * zx + hy * zy + hz * zz
  end
  -- THE PATH'S OWN TURN since the previous tick (s.path_prev_valid, over
  -- cfg.tick_s): the previous tick's path read in this tick's path frame lies
  -- behind the path by the angle it has turned, so its parts across the path
  -- give the turn rates toward the canopy (-z') and the right wing (+y').
  -- These are the path frame's own rates.  The gyro's are the nose's, which
  -- differ from them by every change of angle of attack and sideslip: a nose
  -- yawing back and forth in a sideslip the path does not follow reads as a
  -- turn it is not.  Unknown (NaN) on the first tick after a gap, across a
  -- change between the velocity's path and the nose's, and while the nose
  -- stands in for the frame.
  s.path_turn_up_dps, s.path_turn_right_dps = NAN, NAN
  if s.path_prev_valid and s.path_prev_from_vel == s.path_from_vel
     and not nose_frame and (cfg.tick_s or 0.0) > 0.0 then
    local ox, oy, oz = G.rotate(r00,r01,r02,r10,r11,r12,r20,r21,r22,
                                s.path_prev_n, s.path_prev_e, s.path_prev_d)
    local cx = ox * px + oy * py + oz * pz
    local cy = ox * yx + oy * yy
    local cz = ox * zx + oy * zy + oz * zz
    s.path_turn_up_dps = deg(atan(cz, cx)) / cfg.tick_s
    s.path_turn_right_dps = deg(atan(-cy, cx)) / cfg.tick_s
  end
  s.path_prev_n, s.path_prev_e, s.path_prev_d = s.path_n, s.path_e, s.path_d
  s.path_prev_from_vel = s.path_from_vel

  -- ---- THE ROLL AXIS AIMS THE FLIGHT PATH ---------------------------------
  -- pan_err is the balloon's angle to the side of the path, asin(t'_y): zero
  -- exactly when the balloon lies in the plane a pull swings the path in.
  -- Roll-to-plane's clock, read round the nose, reads 12 or 6 there only while
  -- the path lies in the body's plane of symmetry.  Rolling on nose - balloon
  -- aims the wrong thing whenever the aircraft is sideslipping: with the path
  -- right of the balloon and the balloon right of the nose, a nose-aiming roll
  -- turns RIGHT while the flight path needs to go LEFT, and the rudder ends up
  -- fighting it.
  --
  -- Continued past +-90 deg behind the PATH by the rear-hemisphere
  -- continuation, which is written in the balloon-relative sense, and negated
  -- after it to publish the HUD's convention.
  local tpy_clamped = clip(tpy, -1.0, 1.0)
  local pan_err_deg = deg(asin(tpy_clamped))
  if tpx < 0.0 then
    pan_err_deg = M.rear_hemisphere_pan_err(pan_err_deg, tpx, tpy_clamped, roll)
  end
  pan_err_deg = -pan_err_deg

  out.steering_pan_deg  = pan_err_deg
  out.steering_tilt_deg = tilt_deg

  -- ---- ACTED pitch error --------------------------------------------------
  -- The balloon's ELEVATION above the path's own horizontal plane (x'-y'): the
  -- up-down part -t'_z over the FULL horizontal leg hypot(t'_x, t'_y).
  -- Dividing by t'_x alone would measure -t'_z against the forward part only,
  -- so a balloon off to the side - large t'_y, small t'_x - reads a pitch
  -- demand climbing toward 90 deg though its true elevation is modest.  That
  -- lateral offset is the roll axis's to null, not the pitch axis's, and
  -- feeding it into pitch drives the path past the balloon's elevation until
  -- the turn removes the offset - the abeam-balloon overshoot.  The leg is
  -- non-negative, so it also folds a balloon behind the path onto its true
  -- elevation continuously (no seam at t'_x = 0): pitch flies the elevation it
  -- can honestly reach and the roll axis banks the heading round.
  local e_pitch = atan(-tpz, sqrt(tpx * tpx + tpy * tpy))

  return pan_err_deg, e_pitch, bx, by, bz
end

return M
