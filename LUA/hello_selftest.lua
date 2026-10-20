-- Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
--
-- hello_selftest.lua - does ArduPilot's scripting engine run AT ALL?
--
-- Deliberately trivial: no require(), no bindings beyond gcs:send_text, no
-- modules directory, nothing that can fail for any reason except "the engine
-- is not running".  This is a bisection tool, not part of the steering system.
--
--   PRINTS   -> scripting works; the fault is in balloon_chaser_fc.lua or its modules
--   SILENT   -> the engine never scanned the directory; nothing in the Lua
--               steering path can possibly work until that is fixed
--
-- Usage:
--   cp hello_selftest.lua ~/ardupilot/ArduPlane/scripts/
--   restart sim_vehicle.py      <- scripts are read ONCE at boot
--   watch the MAVProxy console

local count = 0

function update()
  count = count + 1
  if count == 1 then
    gcs:send_text(0, "SELFTEST: ArduPilot Lua scripting IS RUNNING")
  end
  if count % 50 == 0 then
    gcs:send_text(6, string.format("SELFTEST: alive, %d ticks", count))
  end
  return update, 20
end

gcs:send_text(0, "SELFTEST: hello_selftest.lua loaded")
return update, 20
