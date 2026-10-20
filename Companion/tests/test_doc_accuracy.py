# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
"""The project docs must not contradict the code.

"The docs" are CLAUDE.md (rules) and ARCHITECTURE.md (how it works / why); this
check reads both, because a stale claim can live in either and either outranks a
code comment. The doc outranks implementation notes, so a stale claim in it is
worse than a stale comment: it is the thing a future reader trusts. Two failure
modes have actually occurred:

  1. A referenced identifier no longer exists  (SHALLOW_PAN_LEAK_RATE,
     SHALLOW_BANK_LIM_HARD_CAP_DEG, AIRSPEED_MIN, _pan_sched_multiplier ...).
  2. A stated VALUE no longer matches         (a section HEADING asserted
     BANK_LIM_DEG was "FIXED at 90"; the constant said otherwise).

The second is the dangerous one - the name still resolves, so nothing notices.
This checks both, and is deliberately tolerant: it reports rather than fails on
identifiers, because prose legitimately mentions historical names. It FAILS only
on a heading that asserts a numeric value the code contradicts.
"""
import os, re, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Companion import constants as C

fails = []
def check(n, c, d=""):
    print(f"  [{'PASS' if c else 'FAIL'}] {n}" + (f"   {d}" if d else ""))
    if not c: fails.append(n)

print("\n" + "=" * 74); print("PROJECT DOCS vs THE CODE"); print("=" * 74)
# The docs are split: CLAUDE.md holds the rules, ARCHITECTURE.md the mechanism
# and rationale.  Both are authoritative, so both are checked as one corpus.
doc = ""
for _docname in ('CLAUDE.md', 'ARCHITECTURE.md'):
    _p = os.path.join(ROOT, _docname)
    if os.path.exists(_p):
        doc += open(_p).read() + "\n"

print("\n1. SECTION HEADINGS THAT ASSERT A VALUE")
bad = []
for line in doc.splitlines():
    if not line.startswith('#'):
        continue
    m = re.search(r'\b([A-Z][A-Z0-9_]{4,})\b.*?\b(-?\d+(?:\.\d+)?)\b', line)
    if not m:
        continue
    name, claimed = m.group(1), float(m.group(2))
    actual = getattr(C, name, None)
    if isinstance(actual, (int, float)) and abs(float(actual) - claimed) > 1e-6:
        bad.append(f"{name}: heading says {claimed:g}, code says {actual:g}")
check("no heading contradicts a constant", not bad, "; ".join(bad))

print("\n2. IDENTIFIERS THE CODE NO LONGER DEFINES (reported, not fatal)")
corpus = ""
for base, _d, files in os.walk(ROOT):
    if '__pycache__' in base or '/.git' in base:
        continue
    for f in files:
        # EXCLUDE THIS FILE.  Its docstring names the stale constants as
        # examples, which put them in the corpus and made the check report
        # "0 stale" - the test validating itself instead of the code.
        if f == os.path.basename(__file__):
            continue
        if f.endswith(('.py', '.lua')):
            corpus += open(os.path.join(base, f), errors='ignore').read()
# ArduPilot PARAMETERS are not project constants.  They legitimately appear in
# the doc and mostly do not appear in the code as identifiers, so flagging them
# is noise - the check is about constants.py drifting away from CLAUDE.md.
FC_PARAM = ('ARSPD_', 'SCR_', 'SERIAL', 'RC', 'PTCH_', 'RLL_', 'ACRO_', 'MNT',
            'SERVO', 'FS_', 'GCS_', 'STAB_', 'KFF_', 'MAV', 'SIM_', 'AHRS_',
            'EK3_', 'THR_', 'TRIM_', 'WP_', 'Q_', 'LOG_', 'BRD_', 'INS_')
gone = sorted({n for n in re.findall(r'`([A-Z][A-Z0-9_]{4,})`', doc)
               if n not in corpus and not n.startswith(FC_PARAM)})
print(f"       {len(gone)} stale: {', '.join(gone) if gone else 'none'}")
check("fewer than 8 stale constant references", len(gone) < 8, f"{len(gone)}")

print("\n3. THE LUA ARCHITECTURE IS DOCUMENTED")
for needle, why in (
        ("Lua steering offload", "the section exists"),
        ("nav_scripting", "the control-claim rules"),
        ("scripts/modules/", "the deployment layout"),
        ("ARSPD_TYPE", "the airspeed finding"),
        ("C-01 is met", "how the Lua path meets C-01")):
    check(f"    {why}", needle in doc)

print("\n" + "=" * 74)
print("PASSED" if not fails else f"FAILED: {fails}"); print("=" * 74)
sys.exit(1 if fails else 0)
