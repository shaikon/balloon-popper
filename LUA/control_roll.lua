-- Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
--
-- control_roll.lua - roll path of the balloon chaser control law
--
--   compute_roll_command       pan error -> bank angle (rad)
--
-- The pitch path and the throttle loop are in control_pitch.lua.

local C = require("constants")

local M = {}

local abs, rad = math.abs, math.rad
local sin, cos, asin, atan, sqrt = math.sin, math.cos, math.asin, math.atan, math.sqrt
local floor, max, min, huge = math.floor, math.max, math.min, math.huge


local function clip(v, lo, hi)
  if v < lo then return lo elseif v > hi then return hi end
  return v
end

local function copysign1(x)
  -- copysign(1.0, x): -0.0 counts as negative, because 1/-0.0 is -inf.
  if x < 0.0 or (x == 0.0 and 1.0 / x < 0.0) then return -1.0 end
  return 1.0
end

-- ---------------------------------------------------------------------------
--- Roll law: pan error -> commanded bank angle, RADIANS.
--- `hold_i` true holds the integrator where it is this tick (the caller has
--- handed some of the roll command to another law, whose error this one's
--- integrator must not accumulate).
function M.compute_roll_command(pan_err_deg, dt, act_roll_deg,
                                state, g, acro_roll_rate,
                                bank_lim_deg, hold_i)
  if bank_lim_deg == nil then bank_lim_deg = g.bank_lim_deg end

  local p_gain       = g.pan_p_gain
  local pan_ki       = g.pan_ki
  local windup_limit = g.windup_limit_pan
  local roll_gain    = g.roll_gain

  -- proportional
  local bank_deg = p_gain * pan_err_deg
  state.log_p_term_roll = bank_deg

  -- integrator.  Stamp the inert state FIRST: with the gain off the block
  -- below is skipped entirely, and without this the CSV would keep showing the
  -- last live integrator value for the rest of the run - exactly the columns
  -- an operator reads while sweeping pan_ki through zero.
  if abs(pan_ki) <= 1e-6 then
    state.pan_integrator = 0.0
    state.log_pan_i_frozen = 0
    state.log_pan_i_clamped = 0
  end
  if abs(pan_ki) > 1e-6 then
    local max_ri = (bank_lim_deg * windup_limit) / pan_ki

    local tentative_bank = bank_deg + pan_ki * state.pan_integrator

    local outer_sat = abs(tentative_bank) > bank_lim_deg
    local outer_same_dir = (pan_err_deg > 0 and tentative_bank > 0)
                        or (pan_err_deg < 0 and tentative_bank < 0)

    local inner_sat, inner_same_dir = false, false
    if acro_roll_rate > 0 then
      local predicted_rate_dps = (tentative_bank - act_roll_deg) * roll_gain
      inner_sat = abs(predicted_rate_dps) >= acro_roll_rate
      inner_same_dir = (pan_err_deg > 0 and predicted_rate_dps > 0)
                    or (pan_err_deg < 0 and predicted_rate_dps < 0)
    end

    local freeze = hold_i or (outer_sat and outer_same_dir)
                   or (inner_sat and inner_same_dir)
    state.log_pan_i_frozen = freeze and 1 or 0

    if not freeze then
      state.pan_integrator = state.pan_integrator + pan_err_deg * dt
    end

    state.pan_integrator = clip(state.pan_integrator, -max_ri, max_ri)
    state.log_pan_i_clamped = (abs(state.pan_integrator) >= max_ri - 1e-12) and 1 or 0
    bank_deg = bank_deg + pan_ki * state.pan_integrator
  end

  -- lateral-error derivative (lead): roll out of the bank as the pointing
  -- error closes, so the aircraft reaches ty=0 already wings-level rather than
  -- still banked - which would overshoot the null and start the oscillation.
  local raw_deriv = 0.0
  if dt > 1e-6 then
    raw_deriv = (pan_err_deg - state.pan_err_prev) / dt
  end
  if not state.pan_err_deriv_init then
    state.pan_err_deriv_filt = 0.0      -- seed, else a spike from prev = 0
    state.pan_err_deriv_init = true
  else
    local rate = g.pan_kd_rate
    local alpha = rate * dt / (1.0 + rate * dt)
    state.pan_err_deriv_filt = state.pan_err_deriv_filt
                             + alpha * (raw_deriv - state.pan_err_deriv_filt)
  end
  state.pan_err_prev = pan_err_deg

  local d_term_bank = 0.0
  if abs(g.pan_kd) > 1e-9 then
    d_term_bank = g.pan_kd * state.pan_err_deriv_filt
    bank_deg = bank_deg + d_term_bank
  end
  state.log_d_term_roll = d_term_bank

  -- output clip.  The setpoint is in the fed-back quantity's terms, so this
  -- caps the bank PROXY, not the real bank.  Translating bank_lim_deg into
  -- proxy terms to cap the real bank at every pitch is the tempting fix for
  -- the roll-over near vertical, and it removes the steering with it: the roll
  -- that runs on past the limit is what closes a sideways miss there
  -- (ARCHITECTURE.md, "Bank-to-turn").
  state.log_roll_cmd_presat = bank_deg
  state.log_roll_sat = (abs(bank_deg) > bank_lim_deg) and 1 or 0
  return rad(clip(bank_deg, -bank_lim_deg, bank_lim_deg))
end

return M
