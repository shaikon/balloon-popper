#!/bin/sh
# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
#
# The FC scripts must not read undeclared globals.
#
# A local used outside its scope silently becomes a GLOBAL READ returning nil.
# Lua does not complain until the line RUNS, so a scope error can sit in a
# branch the desktop tests never reach and surface only in flight:
#
#   balloon_chaser_fc.lua:619: attempt to perform bitwise operation on a nil value
#                       (global 'now_ms')
#
# now_ms was local to steering(); step() referenced it.
#
# TEXT MATCHING CANNOT FIND THIS - a scan for "is the name declared somewhere in
# the file" sees `local now_ms` inside steering() and passes.  Two attempts at
# that failed with the bug deliberately reintroduced.
#
# The COMPILER already knows.  `luac -l` emits GETTABUP _ENV "name" for every
# global read; anything not an ArduPilot binding or stdlib is a scope error.
#
# The same pass checks the LOCALS limit.  ArduPilot builds its Lua with at most
# LUA_MAX_LOCALS locals in one function, the main chunk included; the desktop
# lua5.4 allows 200, so every desktop test passes on a script the FC refuses
# to load ("too many local variables (limit is 100) in main function").
# `luac -l -l` lists each function's locals; their count, scopes included, is
# never below the most alive at once, so passing here passes on the FC.
set -e
cd "$(dirname "$0")/.."
LUAC="${LUAC:-luac5.4}"

ALLOWED="ahrs mount vehicle serial param gcs millis Parameter Vector3f Quaternion
Location mavlink logger arming rc gps SRV_Channels battery baro ins terrain
notify uart i2c math string table io os tostring tonumber type pairs ipairs
print pcall error assert require select setmetatable getmetatable rawget rawset
next unpack collectgarbage load loadstring dofile loadfile xpcall rawequal
rawlen arg coroutine debug update"

LUA_MAX_LOCALS=100

# Every script that flies: the entry script and the modules lua_deploy ships.
FILES="balloon_chaser_fc protocol constants cascade_state geometry estimator curve_flyer
control_roll control_pitch track cascade excite path_attitude"

echo ""
echo "=========================================================================="
echo "UNDECLARED GLOBALS AND THE LOCALS LIMIT IN THE FC SCRIPTS"
echo "=========================================================================="
echo ""
rc=0
n=0
for f in $FILES; do
    n=$((n + 1))
    # A listed file that does not exist is a failure, not a skip: a stale list
    # would otherwise pass while checking nothing.
    if [ ! -f "LUA/$f.lua" ]; then
        echo "  [FAIL] $f.lua  listed but not found"
        rc=1
        continue
    fi
    got=$($LUAC -l -p "LUA/$f.lua" | grep GETTABUP \
          | sed -n 's/.*_ENV "\([A-Za-z_][A-Za-z0-9_]*\)".*/\1/p' | sort -u)
    bad=""
    for g in $got; do
        case " $(echo $ALLOWED) " in
            *" $g "*) ;;
            *) bad="$bad $g" ;;
        esac
    done
    most=$($LUAC -l -l -p "LUA/$f.lua" \
           | sed -n 's/^locals (\([0-9]*\)).*/\1/p' | sort -n | tail -1)
    if [ -n "$bad" ]; then
        echo "  [FAIL] $f.lua  undeclared global(s):$bad"
        rc=1
    elif [ "${most:-0}" -gt "$LUA_MAX_LOCALS" ]; then
        echo "  [FAIL] $f.lua  $most locals in one function (ArduPilot allows $LUA_MAX_LOCALS)"
        rc=1
    else
        echo "  [PASS] $f.lua  (most locals in a function: ${most:-0} of $LUA_MAX_LOCALS)"
    fi
done
echo ""
echo "=========================================================================="
[ $rc -eq 0 ] && echo "PASSED - $n files" || echo "FAILED"
echo "=========================================================================="
exit $rc
