# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
"""Every ArduPilot binding the FC scripts call must actually exist.

The desktop stubs are hand-written, so a method I INVENT gets stubbed under the
invented name and every test passes - against an API that does not exist on the
flight controller.  That happened twice:

    port:write_string(...)   real name: writestring   (no underscore)
    port:read()  per byte    real name: readstring    (bulk)

Both only surfaced in flight, as
    "attempt to call a nil value (method 'write_string')"
at 50 Hz.

This checks the calls in the FC scripts against ArduPilot's own bindings.desc,
which is the generator input and therefore the authority.
"""
import os, re, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
HERE = os.path.dirname(os.path.abspath(__file__))
LUA = os.path.join(os.path.dirname(os.path.dirname(HERE)), 'LUA')

desc = open(os.path.join(HERE, 'bindings.desc')).read()

# name -> set of method names, for every singleton/userdata in bindings.desc
known = {}
for line in desc.splitlines():
    m = re.match(r'\s*(singleton|userdata|ap_object)\s+(\S+)\s+'
                 r'(?:method|manual|field)\s+(\w+)', line)
    if m:
        known.setdefault(m.group(2), set()).add(m.group(3))
    r = re.match(r'\s*(?:singleton|userdata|ap_object)\s+(\S+)\s+rename\s+(\w+)', line)
    if r:
        known[r.group(2)] = known.get(r.group(1), set())
ALL_METHODS = set().union(*known.values()) if known else set()

fails = []
def check(n, c, d=""):
    print(f"  [{'PASS' if c else 'FAIL'}] {n}" + (f"   {d}" if d else ""))
    if not c: fails.append(n)

print("\n" + "=" * 74); print("ArduPilot BINDING EXISTENCE"); print("=" * 74)
check("bindings.desc parsed", len(ALL_METHODS) > 200,
      f"{len(known)} types, {len(ALL_METHODS)} distinct methods")

# Objects obtained from bindings whose methods must be real.  Locals like
# `port`, `q`, `est` are checked by name where we know their origin.
SUSPECT = ('port', 'q', 'qi', 'quat', 'gyro', 'vel', 'scaling_param')
CALLS = re.compile(r'\b(' + '|'.join(SUSPECT) + r')\s*:\s*(\w+)\s*\(')
# plus every singleton call: name:method(
SINGLETON = re.compile(r'\b(ahrs|mount|vehicle|serial|param|gcs|mavlink|rc|'
                       r'SRV_Channels|logger|arming|battery)\s*:\s*(\w+)\s*\(')

bad = []
for fn in ('balloon_chaser_fc.lua',):
    src = open(os.path.join(LUA, fn)).read()
    for obj, meth in set(CALLS.findall(src)) | set(SINGLETON.findall(src)):
        if meth not in ALL_METHODS:
            bad.append(f"{fn}: {obj}:{meth}()")

check("every method called in balloon_chaser_fc.lua exists in bindings.desc",
      not bad, "; ".join(sorted(bad)) if bad else "")

# ---- ARGUMENT COUNTS -----------------------------------------------------
# Checking that a method NAME exists is not enough: set_rate_target exists and
# takes FIVE arguments, and calling it with four shifted every parameter left -
# pan was applied to pitch, and `false` landed where yaw belonged.  The name
# check passed the whole time.
print("\n  argument counts:")

def _arity(typename, method):
    """Declared argument count from bindings.desc (excluding the return type)."""
    for line in desc.splitlines():
        m = re.match(r'\s*(?:singleton|userdata|ap_object)\s+' + re.escape(typename)
                     + r'\s+method\s+' + re.escape(method) + r'\s+(\S+)\s*(.*)', line)
        if m:
            rest = m.group(2).strip()
            if not rest:
                return 0
            # each argument is one whitespace-separated token, but range-checked
            # numerics carry two extra bound tokens: "float 0 100"
            toks = rest.split()
            n, i = 0, 0
            while i < len(toks):
                n += 1
                i += 1
                while i < len(toks) and re.match(r'^-?[\d.]+$|^[A-Z_]+$|^-?\w+::\w+', toks[i] or ''):
                    i += 1
            return n
    return None

ARITY_CHECKS = [
    ("AP_Mount", "set_rate_target", 5,
     "instance, roll, pitch, yaw, earth_frame"),
    ("AP_Mount", "set_angle_target", 5,
     "instance, roll, pitch, yaw, earth_frame"),
]
for typ, meth, want, sig in ARITY_CHECKS:
    got = _arity(typ, meth)
    check(f"    {typ}:{meth} takes {want} args", got == want,
          f"declared {got} ({sig})")

# Strip comments first.  balloon_chaser_fc.lua no longer CALLS set_rate_target - the
# gimbal is driven by the companion over gz-transport - but the header explains
# why, and a naive regex matches the explanation.
src_fc = open(os.path.join(LUA, 'balloon_chaser_fc.lua')).read()
_code = "\n".join(l for l in src_fc.splitlines() if not l.lstrip().startswith('--'))
m = re.search(r'mount:set_rate_target\(([^)]*)\)', _code, re.S)
if m is None:
    check("    set_rate_target is not called (gimbal is companion-owned)", True,
          "nothing to check arity against")
else:
    n_passed = len(m.group(1).split(','))
    check("    balloon_chaser_fc.lua passes 5 args to set_rate_target", n_passed == 5,
          f"passes {n_passed}")

# ---- uint32_t RETURNS ----------------------------------------------------
# A method declared uint32_t returns a USERDATA object, not a Lua number - a
# Lua 5.3 integer cannot hold the full unsigned range.  It must be :toint()ed
# before being compared or passed on.
print("\n  uint32_t returns must be converted:")
u32 = set()
for line in desc.splitlines():
    m2 = re.match(r'\s*(?:singleton|userdata|ap_object)\s+\S+\s+method\s+(\w+)\s+uint32_t\b', line)
    if m2:
        u32.add(m2.group(1))
check("    available() is uint32_t", 'available' in u32,
      "returns userdata; readstring wants a number")
bad_u32 = []
for meth in sorted(u32):
    for mm in re.finditer(r'\b\w+:' + meth + r'\(\)', src_fc):
        line_start = src_fc.rfind('\n', 0, mm.start()) + 1
        line_end = src_fc.find('\n', mm.end())
        line = src_fc[line_start:line_end]
        nxt = src_fc[mm.end():mm.end() + 220]
        if ':toint()' not in line and ':toint()' not in nxt:
            bad_u32.append(f"{meth} at '{line.strip()[:50]}'")
check("    every uint32_t result is :toint()ed", not bad_u32,
      "; ".join(bad_u32) if bad_u32 else "")

# The two that bit us, asserted by name so a regression is unmistakable
print("\n  the two that were wrong in flight:")
check("writestring is real", 'writestring' in ALL_METHODS)
check("readstring is real", 'readstring' in ALL_METHODS)
check("write_string is NOT real", 'write_string' not in ALL_METHODS,
      "this is the name that was called for six rounds")
src = open(os.path.join(LUA, 'balloon_chaser_fc.lua')).read()
check("balloon_chaser_fc.lua no longer calls write_string", 'write_string' not in src)

# stubs must mirror the real names, or desktop tests pass against fiction
stubs = open(os.path.join(LUA, 'ap_stubs.lua')).read()
check("ap_stubs defines writestring (not write_string)",
      'function port:writestring' in stubs and 'write_string' not in stubs)
check("ap_stubs defines readstring", 'function port:readstring' in stubs)


# --- return ARITY -----------------------------------------------------------
# bindings.desc "boolean X'Null" means ArduPilot returns the VALUE or nil - ONE
# result, not a (ok, value) pair. Unpacking such a call as a pair puts the value
# in the first variable and nil in the second, so every test against the second
# fails silently and the feature is simply dead. Existence checks do not catch
# this: the method is real, it is called correctly, and the call returns.
print("\nsingle-return methods are not unpacked as pairs:")
_single = set(re.findall(r"method (\w+) boolean \w+'Null", desc))
for _m in re.finditer(r"local\s+\w+\s*,\s*\w+\s*=\s*\w+:(\w+)\(", src):
    _name = _m.group(1)
    check(f"    {_name}()", _name not in _single,
          "declared boolean X'Null - returns ONE value or nil, never a pair")
if not any(_m.group(1) in _single
           for _m in re.finditer(r"local\s+\w+\s*,\s*\w+\s*=\s*\w+:(\w+)\(", src)):
    check("    none unpacked as pairs", True)


# --- STATE fields must actually be WRITTEN ----------------------------------
# A field in the protocol that balloon_chaser_fc.lua never assigns is worse than a
# missing one: the wire carries it, the companion decodes it, the CSV has a
# column for it and the HUD prints it - as a confident zero. omega_x_cmd_dps
# shipped that way and read 0.0 on every flight while the aircraft was visibly
# rolling.
#
# Fields listed in _STATE_UNWRITTEN are known gaps, not accidents. Removing a
# name from that list is how you close one; adding a name needs a reason.
print("\nevery STATE field is written by balloon_chaser_fc.lua:")
_fc = open(os.path.join(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))), "LUA", "balloon_chaser_fc.lua")).read()
_written = set(re.findall(r"st_msg\.(\w+)", _fc))
_STATE_UNWRITTEN = set()   # no known gaps: every STATE field is written
from Companion import protocol as _proto
for _f in _proto.STATE:
    if _f.name.startswith("_"):
        continue
    if _f.name in _STATE_UNWRITTEN:
        continue
    check(f"    {_f.name}", _f.name in _written,
          "in the protocol but never assigned - reports 0 everywhere")

print("\nevery STATE2 field is written by balloon_chaser_fc.lua:")
_written2 = set(re.findall(r"\bst2\.(\w+)", _fc))
for _f in _proto.STATE2:
    check(f"    {_f.name}", _f.name in _written2,
          "in the protocol but never assigned - reports 0 everywhere")

print("\n" + "=" * 74)
print("PASSED" if not fails else f"FAILED: {fails}"); print("=" * 74)
sys.exit(1 if fails else 0)
