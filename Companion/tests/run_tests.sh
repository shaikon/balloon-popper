#!/bin/sh
# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
# Run every desktop check.  Nothing here needs SITL, a flight controller, or a
# simulator - it all runs on a laptop.
set -e
cd "$(dirname "$0")/../.."      # repo root
echo "=== Python: contract checks ==="
for t in test_lua_bindings test_no_micromanagement test_link_contract \
         test_tuner_registration test_export_gazebo test_hud_contract \
         test_lua_deploy test_no_dead_code test_doc_accuracy \
         test_logging_visible test_csv_alignment test_control_path \
         test_trim_map; do
  python3 "Companion/tests/$t.py"
done
echo "=== Lua: shim + FC shell ==="
./LUA/test_no_globals.sh
lua5.4 LUA/test_ap_shim.lua
lua5.4 LUA/test_balloon_chaser_fc.lua
lua5.4 LUA/test_yaw_coord.lua
echo
echo "ALL SUITES PASSED"
