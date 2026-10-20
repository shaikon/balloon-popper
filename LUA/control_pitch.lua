-- Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
--
-- control_pitch.lua - pitch path and throttle loop of the balloon chaser control law
--
--   update_bodyplane_pitch_integrator   conditional integration + clamp
--   compute_bodyplane_pitch_rate        pitch error theta_ip -> pitch rate
--   compute_throttle_command            airspeed hold with operator handback
--
-- The roll path lives in control_roll.lua.
--
-- ============================================================================
-- INT-VALUED FLAG
--   airspeed_cmd_enable is 0/1, and Lua `if x then` is TRUE at 0.  Tested
--   `~= 0` below.
--
-- NaN
--   Airspeed is guarded with `v == v`, an explicit NaN test: NaN ~= NaN.
-- ============================================================================

local C = require("constants")

local M = {}

local abs, deg = math.abs, math.deg
local max = math.max

local function clip(v, lo, hi)
  if v < lo then return lo elseif v > hi then return hi end
  return v
end

-- ---------------------------------------------------------------------------
--- Conditional integration for the body-plane pitch loop.
--- The state is the pointing error integrated in deg x s, so ki is in 1/s, as
--- the roll, roll-to-plane and yaw-pointing integrators' are.  Returns the
--- integral's pitch-RATE contribution in deg/s (ki * state), added directly to
--- the rate demand (parallel form: NOT scaled by the P gain).
---
--- `pg` fields: gain ki windup_limit deadband_deg kd kd_rate
function M.update_bodyplane_pitch_integrator(theta_ip_rad, dt, state, pg,
                                             acro_pitch_rate)
  state.log_pitch_i_frozen = 0
  state.log_pitch_i_clamped = 0
  -- With the gain off the integrator is inactive, so it is cleared rather
  -- than held: held, the log keeps showing its last value, and turning the
  -- gain back on adds that stale integral to the command at once.
  if abs(pg.ki) <= 1e-6 then
    state.pitch_integrator = 0.0
    return 0.0
  end

  local err_deg = deg(theta_ip_rad)

  -- pitch-rate ceiling the inner loop can actually deliver: the FC ACRO rate
  local rate_ceiling = acro_pitch_rate

  -- P-path gain, floored at 0; used for the parallel rate prediction (the
  -- freeze gate) only - the windup ceiling below is tied to the achievable rate
  local g_eff = max(0.0, pg.gain)

  -- deadband: stops the integrator chasing zero-crossing noise
  if abs(err_deg) > pg.deadband_deg then
    local freeze = false
    if rate_ceiling > 0 then
      local tentative_integrator   = state.pitch_integrator + err_deg * dt
      local tentative_integral_dps = pg.ki * tentative_integrator
      local predicted_rate_dps     = err_deg * g_eff + tentative_integral_dps
      local inner_sat = abs(predicted_rate_dps) >= rate_ceiling
      local inner_same_dir = (err_deg > 0 and predicted_rate_dps > 0)
                          or (err_deg < 0 and predicted_rate_dps < 0)
      freeze = inner_sat and inner_same_dir
    end
    if not freeze then
      state.pitch_integrator = state.pitch_integrator + err_deg * dt
    end
    state.log_pitch_i_frozen = freeze and 1 or 0
  end

  -- hard windup clip: bound the state so the integral's rate contribution
  -- ki*state stays within windup_limit of the ACHIEVABLE pitch rate (the same
  -- ceiling the freeze gate uses).  ki cancels; the reference is the achievable
  -- rate, not gain*angle, so the ceiling is independent of the P gain (a
  -- ceiling of 0 pins it)
  local max_pi = (pg.windup_limit * max(0.0, rate_ceiling)) / pg.ki
  state.pitch_integrator = clip(state.pitch_integrator, -max_pi, max_pi)
  state.log_pitch_i_clamped =
    (abs(state.pitch_integrator) >= max_pi - 1e-12) and 1 or 0

  return pg.ki * state.pitch_integrator
end

-- ---------------------------------------------------------------------------
-- ---------------------------------------------------------------------------
--- Body-plane pitch rate from the pitch error.
--- Returns omega_y_dps.
---
--- theta_ip is the balloon's elevation above the path frame's horizontal plane
--- (track.lua): an in-plane angle, not an Euler angle, so it is well-defined
--- through the vertical.  Stack: P + I + D.  Returns the RAW rate demand: no
--- shaper and no saturation here.  The caller clips to the available pitch
--- authority and the FC clamps to ACRO, so clipping here would be redundant and
--- would corrupt the presat diagnostic captured before that clip.
--- acro_pitch_rate is passed for the integrator anti-windup: both the freeze
--- gate and the windup clamp use it as the achievable ceiling.
function M.compute_bodyplane_pitch_rate(theta_ip, dt, state, pg,
                                        acro_pitch_rate)
  local integral_dps = M.update_bodyplane_pitch_integrator(
    theta_ip, dt, state, pg, acro_pitch_rate)

  -- Parallel form: P scales the pointing error, the integral adds its own rate
  -- DIRECTLY (not through the P gain), so pitch_ki is independent of pitch_gain.
  local omega_raw = deg(theta_ip) * max(0.0, pg.gain) + integral_dps

  -- Derivative (damping) on the RAW pointing error theta_ip - not err_eff, so
  -- the integrator's own motion is not differentiated back into the command.
  -- d(theta_ip)/dt is low-pass filtered (kd_rate); adding kd*deriv reduces the
  -- commanded rate as the error closes, so the aircraft arrives with less rate
  -- and overshoots less.
  local err_deg = deg(theta_ip)
  local raw_deriv = 0.0
  if dt > 1e-6 then raw_deriv = (err_deg - state.pitch_err_prev) / dt end
  if not state.pitch_err_deriv_init then
    state.pitch_err_deriv_filt = 0.0      -- seed, else a spike from prev = 0
    state.pitch_err_deriv_init = true
  else
    local rate = pg.kd_rate or 0.0
    local alpha = rate * dt / (1.0 + rate * dt)
    state.pitch_err_deriv_filt = state.pitch_err_deriv_filt
                               + alpha * (raw_deriv - state.pitch_err_deriv_filt)
  end
  state.pitch_err_prev = err_deg

  local d_term_dps = 0.0
  if abs(pg.kd or 0.0) > 1e-9 then
    d_term_dps = pg.kd * state.pitch_err_deriv_filt
    omega_raw = omega_raw + d_term_dps
  end
  state.log_d_term_pitch = d_term_dps

  -- Return the RAW demand: the caller (cascade.lua) clips to authority and the
  -- FC clamps to ACRO.  Clipping here is redundant and would hide saturation
  -- from the presat log.
  return omega_raw
end

-- ---------------------------------------------------------------------------
--- Airspeed-hold throttle loop, percent.
--- Hands straight back to the operator whenever the loop is disengaged or the
--- airspeed is not worth believing, rather than integrating against a dead
--- sensor.  State is dropped on handback so the next start re-seeds from the
--- operator's throttle - which is what makes restart bumpless.
function M.compute_throttle_command(airspeed_mps, dt, state, g,
                                    operator_throttle_pct)
  local lo, hi = 0.0, 100.0
  local v = airspeed_mps

  -- `v == v` is the NaN test: NaN ~= NaN.
  local live = ((g.airspeed_cmd_enable or 0) ~= 0)
           and v ~= nil and v == v
           and v >= g.airspeed_cmd_min_valid

  if not live then
    state.i_term = 0.0
    state.active = false
    state.log_en = 0
    state.log_err_mps = 0.0
    state.log_i_term = 0.0
    state.log_cmd_pct = operator_throttle_pct
    state.log_sat = 0
    return operator_throttle_pct
  end

  if not state.active then
    state.i_term = clip(operator_throttle_pct, lo, hi)
    state.active = true
  end

  local err = g.airspeed_cmd_mps - v
  local p_term = g.airspeed_cmd_kp * err

  -- Anti-windup: integrate only when doing so would not push further into a
  -- clamp the output is already sitting on.
  local tentative = p_term + state.i_term
  if not ((tentative >= hi and err > 0.0) or (tentative <= lo and err < 0.0)) then
    state.i_term = clip(state.i_term + g.airspeed_cmd_ki * err * dt, lo, hi)
  end

  local cmd = p_term + state.i_term
  local sat = (cmd > hi or cmd < lo) and 1 or 0
  cmd = clip(cmd, lo, hi)

  state.log_en = 1
  state.log_err_mps = err
  state.log_i_term = state.i_term
  state.log_cmd_pct = cmd
  state.log_sat = sat
  return cmd
end

return M
