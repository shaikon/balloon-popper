-- Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
--
-- cascade_state.lua - CascadeState / ThrottleState and the mode-entry reset.
--
-- The reset is small and easy to treat as bookkeeping; it is not.  It clears a
-- SPECIFIC set and deliberately leaves the rest alone, and the omissions carry
-- as much meaning as the assignments.
--
--   Integrators earn their value against one chase's signal geometry and
--   are invalid across a mode change, so all three are cleared (and, through
--   reset_integrators, whenever their output stops reaching the command).
--
--   pan_err_deriv_init is cleared, NOT merely the filter value.  This resets
--   compute_roll_command's own first-call SEED.  compute_roll_command runs
--   only while the law is running, so across a gap pan_err_prev freezes at
--   the last live tick; without resetting the seed the first tick back would
--   differentiate across the entire gap and produce a derivative kick.  The
--   pitch derivative is reset the same way.
--
--   So is roll-to-plane's 12-or-6 choice: it was made against the previous
--   chase's balloon, and holding it would bring a new balloon to the far
--   end of the belly-canopy line.  The clock history and the filtered drift
--   and error rate its feedforward and derivative read go with it, for the
--   reason the derivative seeds do.

local M = {}

-- ---------------------------------------------------------------------------
--- Fresh CascadeState, including the log_* diagnostics (they are OUTPUTS,
--- not carried state, but they must exist so the writer never touches a nil).
function M.new_cascade()
  return {
    -- carried state
    pan_integrator = 0.0,
    pan_err_prev = 0.0,
    pan_err_deriv_filt = 0.0,
    pan_err_deriv_init = false,
    pitch_integrator = 0.0,
    pitch_err_prev = 0.0,
    pitch_err_deriv_filt = 0.0,
    pitch_err_deriv_init = false,
    plane_side = 0,               -- roll-to-plane: 1 = 12 o'clock, -1 = 6, 0 = not chosen
    plane_clock_prev = nil,       -- Optional[float]: last tick's clock angle, deg
    plane_drift_lpf = 0.0,        -- filtered drift round the clock, deg/s
    plane_err_rate_lpf = 0.0,     -- filtered rate of the clock error, deg/s
    plane_i_dps = 0.0,            -- roll-to-plane integrator, deg/s of roll
    steep_share = 0.0,            -- this tick's path share, for the yaw pointing schedule
    plan_set_bank = false,        -- this tick's roll setpoint is a PLAN's bank (cascade.lua)
    -- diagnostics
    log_p_term_roll = 0.0,
    log_roll_cmd_presat = 0.0,
    log_omega_x_presat = 0.0, log_omega_y_presat = 0.0,    log_roll_sat = 0,
    log_d_term_roll = 0.0,        log_roll_err_deg = 0.0,
    log_d_term_pitch = 0.0,
    log_pan_i_frozen = 0,         log_pan_i_clamped = 0,
    log_pitch_i_frozen = 0,       log_pitch_i_clamped = 0,
  }
end

--- Fresh ThrottleState.
function M.new_throttle()
  return {
    i_term = 0.0, active = false,
    flown = false,      -- did this tick's throttle reach ArduPlane? (step())
    log_en = 0, log_err_mps = 0.0, log_i_term = 0.0,
    log_cmd_pct = 0.0, log_sat = 0,
  }
end

--- The airspeed hold's integrator, cleared on every tick its throttle is not
--- flown.  `active` goes with it, so the next activation re-seeds the
--- integrator from the operator's throttle: a bare i_term = 0 would restart
--- the loop from zero throttle.
function M.reset_throttle(t)
  t.i_term = 0.0
  t.active = false
end

-- ---------------------------------------------------------------------------
--- The law's integrators, cleared whenever their output stops reaching the
--- command: a tick the law does not run, a tick it runs but nobody flies it,
--- an axis taken over by an override.  An inactive integrator is cleared, not
--- held - held, it adds a stale integral the moment it becomes active again.
--- The yaw pointing integrator lives in the track state and is cleared beside
--- each call.
function M.reset_integrators(s)
  s.pan_integrator = 0.0
  s.pitch_integrator = 0.0
  s.plane_i_dps = 0.0
end

-- ---------------------------------------------------------------------------
--- Mode entry: a fresh start.  A mode entry is not a continuation of
--- anything.
function M.reset_mode_entry(s)
  s.pan_integrator = 0.0
  s.pitch_integrator = 0.0
  s.pan_err_prev = 0.0
  s.pan_err_deriv_filt = 0.0
  s.pan_err_deriv_init = false
  s.pitch_err_prev = 0.0
  s.pitch_err_deriv_filt = 0.0
  s.pitch_err_deriv_init = false
  s.plane_side = 0
  s.plane_clock_prev = nil
  s.plane_drift_lpf = 0.0
  s.plane_err_rate_lpf = 0.0
  s.plane_i_dps = 0.0
end

return M
