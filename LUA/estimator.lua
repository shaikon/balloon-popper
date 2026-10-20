-- Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
--
-- estimator.lua - port of Companion/estimator.py
--
-- Complementary filter on the unit sphere, plus the measurement trust weight.
--
-- PORT RULES OBSERVED HERE (they apply to every module that follows)
--
--   1. ZERO ALLOCATION after construction.  The estimate lives in ONE
--      preallocated Vector3f that is mutated in place; nothing inside update()
--      calls a constructor, :copy(), :scale(), :cross() or an arithmetic
--      operator.  Verified by ap_shim's allocation counter, not by inspection.
--
--   2. Optional[] becomes an explicit VALID FLAG.  Python returns None for an
--      uninitialised estimate; Lua carries `self.valid`.  Never use nil to mean
--      "uninitialised" - a nil that leaks into arithmetic is a runtime error at
--      50 Hz, and a runtime error removes the script (ArduPilot does not retry
--      a failed script), which then holds the last commanded rate for a full
--      second.  Flags are cheap; nils are not.
--
--   3. NO numpy.  Every vector operation is written out on the three
--      components.  This is faithful to the Python because the Python only ever
--      uses norm / scale / add on 3-vectors.
--
--   4. OPERATION ORDER IS PRESERVED EXACTLY.  `(1-w)*est + w*z` then normalise -
--      not an algebraically equivalent rearrangement.  Float addition is not
--      associative; reordering changes the last bits and the differ will (
--      correctly) call it a divergence.
--
-- Reference: estimator.py, WORLD_EST_ALPHA_NOM = 0.5

local M = {}

local WORLD_EST_ALPHA_NOM = 0.5
local EPS = 1e-9

-- ---------------------------------------------------------------------------
-- WorldBalloonEstimator
-- ---------------------------------------------------------------------------

local Est = {}
Est.__index = Est

--- Construct.  The single Vector3f allocated here is the ONLY allocation the
--- estimator ever performs.
function M.new()
  return setmetatable({
    _e = Vector3f(),      -- the estimate, mutated in place forever after
    valid = false,
  }, Est)
end

function Est:reset()
  self.valid = false
  self._e:x(0.0); self._e:y(0.0); self._e:z(0.0)
end

--- Current estimate.  Returns the INTERNAL vector - callers must not retain or
--- mutate it.  Matches Python's `get()` returning the live array.
function Est:get()
  if not self.valid then return nil end
  return self._e
end

--- Fold a world-frame measurement into the estimate.
---
--- @param mx,my,mz  measurement components, or mx==nil for "no measurement"
--- @param trust     [0,1]
--- @param dt        seconds
--- @param alpha_nom nominal filter bandwidth (1/s); defaults to 0.5
--- @return true if an estimate is available afterwards
function Est:update(mx, my, mz, trust, dt, alpha_nom)
  alpha_nom = alpha_nom or WORLD_EST_ALPHA_NOM

  -- no measurement, or zero trust: hold state (Python returns self._estimate)
  if mx == nil or trust <= 0.0 then return self.valid end

  local n = math.sqrt(mx * mx + my * my + mz * mz)
  if n < EPS then return self.valid end
  local zx, zy, zz = mx / n, my / n, mz / n

  -- cold start: snap to first valid measurement
  if not self.valid then
    self._e:x(zx); self._e:y(zy); self._e:z(zz)
    self.valid = true
    return true
  end

  -- complementary filter: lerp then renormalise.
  -- w = min(1, max(0, alpha_nom * trust * dt))
  local w = alpha_nom * trust * dt
  if w < 0.0 then w = 0.0 elseif w > 1.0 then w = 1.0 end
  local iw = 1.0 - w

  -- EXACT Python order: (1-w)*estimate + w*z, componentwise.
  local bx = iw * self._e:x() + w * zx
  local by = iw * self._e:y() + w * zy
  local bz = iw * self._e:z() + w * zz

  local bn = math.sqrt(bx * bx + by * by + bz * bz)
  if bn < EPS then return self.valid end     -- degenerate: keep old

  self._e:x(bx / bn); self._e:y(by / bn); self._e:z(bz / bn)
  return true
end

-- ---------------------------------------------------------------------------
-- Trust weight
-- ---------------------------------------------------------------------------

--- alpha_A * alpha_C, per estimator.world_est_trust.
---
---   alpha_A = (1 - min(1, |det_norm|))^2   gimbal tracking quality
---   alpha_C = max(0, 1 - |body_rate| / gimbal_slew)   or 1.0 with readback
---
--- NOTE for the real airframe: with encoder readback the C-factor collapses to
--- 1.0, because the read-back angle IS the physical pointing regardless of how
--- fast the airframe rotated.  Whether the FC gimbal driver reports readback is
--- open item 4 in the migration plan - it changes trust, and trust changes the
--- filter bandwidth, so it must be settled before flight comparison.
function M.trust(det_norm_x, det_norm_y, body_rate_abs_dps,
                 gimbal_slew_rate_dps, gimbal_has_readback)
  local off = math.sqrt(det_norm_x * det_norm_x + det_norm_y * det_norm_y)
  if off > 1.0 then off = 1.0 end
  local a = (1.0 - off)
  local alpha_a = a * a

  local alpha_c
  if gimbal_has_readback then
    alpha_c = 1.0
  elseif gimbal_slew_rate_dps <= 0 then
    alpha_c = 0.0
  else
    alpha_c = 1.0 - body_rate_abs_dps / gimbal_slew_rate_dps
    if alpha_c < 0.0 then alpha_c = 0.0 end
  end

  return alpha_a * alpha_c
end

M.WORLD_EST_ALPHA_NOM = WORLD_EST_ALPHA_NOM
return M
