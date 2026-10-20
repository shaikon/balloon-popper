# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
"""Automatic deployment of the FC scripts.

The point of automating this is NOT to save typing - it is that a hand copy has
no verification step.  So the tests are about the failure modes, not the happy
path: a corrupted script must be detected and repaired, a stale ap_shim.lua must
be removed, and an unchanged deploy must NOT report a change (which would force
a pointless reload on every launch).
"""
import hashlib, os, sys, tempfile
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from Companion.adapters.lua_deploy import (deploy, find_scripts_dir,
                                                FORBIDDEN,
                                                ENTRY_SCRIPTS, MODULES,
                                                RETIRED_ENTRY_SCRIPTS)

fails = []
def check(n, c, d=""):
    print(f"  [{'PASS' if c else 'FAIL'}] {n}" + (f"   {d}" if d else ""))
    if not c: fails.append(n)

print("\n" + "=" * 70); print("FC SCRIPT DEPLOYMENT"); print("=" * 70)
d = tempfile.mkdtemp()

print("\n1. FIRST DEPLOY")
ok, changed, dest = deploy(dest_dir=d)
check("succeeded", ok)
check("reports a change (reload needed)", changed)
# ArduPilot EXECUTES everything in scripts/ and require() looks ONLY in
# scripts/modules/.  The split is not cosmetic: a module in scripts/ is run as a
# script (it returns a table, not an update function) AND is invisible to
# require, so nothing loads and the only symptom is silence.
top = sorted(f for f in os.listdir(dest) if f.endswith('.lua'))
mods = sorted(os.listdir(os.path.join(dest, 'modules')))
check("entry point alone in scripts/", top == sorted(ENTRY_SCRIPTS), str(top))
check(f"all {len(MODULES)} modules in scripts/modules/",
      mods == sorted(MODULES), f"{len(mods)} modules")

print("\n2. IDEMPOTENT")
ok2, changed2, _ = deploy(dest_dir=d)
check("succeeded", ok2)
check("reports NO change - no pointless reload", not changed2)

print("\n3. CORRUPTED SCRIPT IS DETECTED AND REPAIRED")
victim = os.path.join(d, 'modules', 'geometry.lua')
good = hashlib.sha256(open(victim, 'rb').read()).hexdigest()
open(victim, 'w').write('-- truncated mid-function\n')
ok3, changed3, _ = deploy(dest_dir=d)
now = hashlib.sha256(open(victim, 'rb').read()).hexdigest()
check("repaired", ok3 and now == good)
check("forces a reload", changed3)

print("\n4. FORBIDDEN FILES ARE REMOVED")
for bad in FORBIDDEN:
    open(os.path.join(d, bad), 'w').write('-- shadows the real bindings\n')
ok4, changed4, _ = deploy(dest_dir=d)
check("ap_shim/ap_stubs removed",
      all(not os.path.exists(os.path.join(d, b)) for b in FORBIDDEN),
      "they would shadow ArduPilot's own Vector3f/Quaternion")
check("forces a reload", changed4)

print("\n5. A MODULE LEFT IN scripts/ BY AN OLD LAYOUT IS REMOVED")
open(os.path.join(d, 'geometry.lua'), 'w').write('-- stray\n')
ok5, changed5, _ = deploy(dest_dir=d)
check("stray module removed from scripts/",
      not os.path.exists(os.path.join(d, 'geometry.lua')),
      "ArduPilot would execute it every boot")
check("still present in modules/",
      os.path.isfile(os.path.join(d, 'modules', 'geometry.lua')))

print("\n5b. A RETIRED ENTRY SCRIPT IS REMOVED")
for _old in RETIRED_ENTRY_SCRIPTS:
    open(os.path.join(d, _old), 'w').write('-- retired\n')
deploy(dest_dir=d)
check("no retired entry script left in scripts/",
      not any(os.path.exists(os.path.join(d, _o)) for _o in RETIRED_ENTRY_SCRIPTS),
      "ArduPilot would run it beside the current one")
check("no retired name is a current entry script",
      not set(RETIRED_ENTRY_SCRIPTS) & set(ENTRY_SCRIPTS))

print("\n6. DESKTOP-ONLY FILES NEVER SHIP")
listing = set(os.listdir(d)) | set(os.listdir(os.path.join(d, 'modules')))
check("no test_/xcheck_/replay_/e2e_ files",
      not any(f.startswith(('test_', 'xcheck_', 'replay_', 'e2e_')) for f in listing))
check("balloon_chaser_fc.lua is the entry point and is present",
      'balloon_chaser_fc.lua' in listing)

print("\n7. EXPLICIT PATH IS CREATED, AUTO-DETECT DOES NOT INVENT ONE")
made = find_scripts_dir(os.path.join(d, 'nested', 'scripts'))
check("explicit path created", made is not None and os.path.isdir(made))

print("\n8. THE GENERATED LUA MATCHES ITS GENERATORS")
# LUA/protocol.lua and LUA/constants.lua are generated from protocol.py and
# constants.py.  A generator run that was forgotten deploys a Lua that
# disagrees with the companion - a frame layout or a constant flown that nothing
# on this side describes - and nothing else notices.
from Companion import protocol as _P, gen_constants as _GC
from Companion.control_laws import ControlGains as _CG
_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_proto_lua = open(os.path.join(_root, 'LUA', 'protocol.lua')).read()
check("LUA/protocol.lua is protocol.generate_lua()",
      _proto_lua == _P.generate_lua(_CG), "run: python3 -m Companion.protocol")
_vals, _ = _GC.collect()
_const_lua = open(os.path.join(_root, 'LUA', 'constants.lua')).read()
check("LUA/constants.lua is gen_constants.render()",
      _const_lua == _GC.render(_vals), "run: python3 -m Companion.gen_constants")

print("\n9. A DIRECTORY THE AUTOPILOT DOES NOT READ GETS NOTHING")
# Checked before the copy: a refused deploy leaves no second set of scripts
# behind in a directory nothing loads.
import Companion.adapters.lua_deploy as _ld
_real_probe = _ld._sitl_cwd_from_proc
_elsewhere = tempfile.mkdtemp()
_wrong = tempfile.mkdtemp()
_ld._sitl_cwd_from_proc = lambda: os.path.join(_elsewhere, "scripts")
try:
    ok9, changed9, _ = deploy(dest_dir=_wrong)
finally:
    _ld._sitl_cwd_from_proc = _real_probe
check("refused", not ok9 and not changed9)
check("and nothing was copied", os.listdir(_wrong) == [], str(os.listdir(_wrong)))

print("\nSTARTUP ORDER: check, then write, then reload")
# Written before a halt the operator stops at, new scripts sit on disk unloaded
# and the next launch finds them up to date: the FC keeps the old script.
_main = open(os.path.join(_root, 'main.py')).read()
_i_check = _main.find('ensure_scripting(link')
_i_write = _main.find('_deploy_lua(dest_dir')
_i_reload = _main.find('restart_scripting(link)')
check("main.py checks the parameters before it writes the scripts",
      0 <= _i_check < _i_write, f"check at {_i_check}, write at {_i_write}")
check("and reloads the VM after writing them",
      _i_write < _i_reload, f"write at {_i_write}, reload at {_i_reload}")

print("\n" + "=" * 70)
print("PASSED" if not fails else f"FAILED: {fails}"); print("=" * 70)
sys.exit(1 if fails else 0)
