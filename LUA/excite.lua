-- Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
--
-- excite.lua - identification excitation, generated ON THE FC.
--
-- ============================================================================
-- WHY THIS IS HERE AND NOT ON THE COMPANION
-- ============================================================================
-- A waveform streamed from the companion a value per tick would leave every
-- identification run measuring plant PLUS transport, stamped with the
-- companion's scheduling jitter.
--
-- So the companion sends a DESCRIPTION once (EXCITE: kind, axis, replace,
-- amplitude, frequency, duration) and this module generates the waveform
-- locally against millis().  The excitation is then as clean as the FC clock,
-- and - critically - the applied value is timestamped in the SAME tick as the
-- gyro sample that responds to it.  A frequency fit is only as good as the
-- phase relationship between its input and output records.
--
--   SQUARE   +amp for the first half of each period, -amp after.  Any
--            amplitude scaling with period is the companion's scheduling
--            decision; amp_dps arrives already scaled.
--   STEP     a signed constant rate, held; amp_dps arrives signed.
--   DOUBLET  +amp for the first half of the duration, -amp for the second.
--   SINE     amp * sin(2 pi f0 t).
--   CHIRP    linear sweep f0 -> f1 over the duration.
--
-- Nothing in the companion sends EXCITE yet; verify any fit written against
-- one of these against a recorded run first.
--
-- ============================================================================
-- APPLICATION MODE - the distinction that matters
-- ============================================================================
--   replace = false  ADDS to the command on `axis`.  Loop stays CLOSED; the
--                    excitation is external to it.  (freqid)
--   replace = true   REPLACES the command on `axis`.  That axis is OPEN LOOP;
--                    the other keeps flying the aim.  (plantid)
-- Getting these the wrong way round does not error - it silently identifies
-- the wrong system.  The add and the replace are applied in cascade.lua, on
-- either side of the rate-envelope clip.

local M = {}

local sin, floor, abs, pi = math.sin, math.floor, math.abs, math.pi

M.NONE, M.DOUBLET, M.CHIRP, M.SINE, M.STEP, M.SQUARE = 0, 1, 2, 3, 4, 5

M.AXIS_ROLL, M.AXIS_PITCH = 0, 1

-- ---------------------------------------------------------------------------
--- Construct.  One table, allocated once; start()/value() never allocate.
function M.new()
  return {
    kind = M.NONE,
    axis = M.AXIS_ROLL,
    replace = false,
    amp_dps = 0.0,
    f0_hz = 0.0,
    f1_hz = 0.0,
    dur_ms = 0,
    t0_ms = 0,
    running = false,
    last_value = 0.0,
  }
end

--- Begin a waveform.  `now_ms` is millis() at the tick that received the
--- command, NOT at parse time - so the phase reference is a control tick.
--- dur_ms == 0 cancels any running excitation (protocol contract).
function M.start(e, kind, axis, replace, amp_dps, f0_hz, f1_hz, dur_ms, now_ms)
  if kind == M.NONE or dur_ms == nil or dur_ms <= 0 then
    M.stop(e)
    return false
  end
  e.kind = kind
  e.axis = axis or M.AXIS_ROLL
  e.replace = replace and true or false
  e.amp_dps = amp_dps or 0.0
  e.f0_hz = f0_hz or 0.0
  e.f1_hz = f1_hz or 0.0
  e.dur_ms = dur_ms
  e.t0_ms = now_ms
  e.running = true
  e.last_value = 0.0
  return true
end

function M.stop(e)
  e.kind = M.NONE
  e.running = false
  e.last_value = 0.0
end

--- Current excitation value, deg/s.  Returns 0.0 when idle or expired.
--- Call ONCE per tick: it also retires the waveform when the duration elapses.
function M.value(e, now_ms)
  if not e.running then return 0.0 end

  local el_ms = now_ms - e.t0_ms
  if el_ms < 0 then el_ms = 0 end          -- clock stepped; treat as t0
  if el_ms >= e.dur_ms then
    M.stop(e)
    return 0.0
  end

  local t = el_ms * 0.001                  -- seconds since start
  local A = e.amp_dps
  local v = 0.0

  if e.kind == M.SQUARE then
    -- plant_freq_id: + for the first half of each period, - for the second.
    -- period from f0; guarded so a zero frequency cannot divide by zero.
    local T = (e.f0_hz > 1e-6) and (1.0 / e.f0_hz) or 0.0
    if T <= 0.0 then
      v = A
    else
      v = ((t % T) < 0.5 * T) and A or -A
    end

  elseif e.kind == M.STEP then
    v = A                                  -- signed constant, held

  elseif e.kind == M.DOUBLET then
    -- +A for the first half of the DURATION, -A for the second
    local half = e.dur_ms * 0.0005
    v = (t < half) and A or -A

  elseif e.kind == M.SINE then
    v = A * sin(2.0 * pi * e.f0_hz * t)

  elseif e.kind == M.CHIRP then
    -- linear frequency sweep f0 -> f1 over the duration.
    -- Phase is the INTEGRAL of frequency: 2*pi*(f0*t + 0.5*k*t^2), k = df/dur.
    -- Using 2*pi*f(t)*t instead is the classic chirp bug - it produces an
    -- instantaneous frequency of f0 + 2*k*t, i.e. twice the intended sweep
    -- rate, and every fitted frequency comes back wrong.
    local dur_s = e.dur_ms * 0.001
    local k = (dur_s > 1e-9) and ((e.f1_hz - e.f0_hz) / dur_s) or 0.0
    v = A * sin(2.0 * pi * (e.f0_hz * t + 0.5 * k * t * t))
  end

  e.last_value = v
  return v
end

return M
