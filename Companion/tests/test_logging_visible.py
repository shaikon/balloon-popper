# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
"""Package log output must actually reach the operator.

main.py configures the "balloon_chaser" logger only.  Companion.* module loggers
are NOT its children, so every log.info() inside the package went to a logger
with no handler and was discarded - the whole Lua deploy and scripting-setup
diagnostic trail included.  The startup looked like nothing had happened, which
turned a one-line configuration problem into several rounds of guessing.
"""
import ast, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

fails = []
def check(n, c, d=""):
    print(f"  [{'PASS' if c else 'FAIL'}] {n}" + (f"   {d}" if d else ""))
    if not c: fails.append(n)

print("\n" + "=" * 70); print("PACKAGE LOG VISIBILITY"); print("=" * 70)
src = open(os.path.join(ROOT, 'main.py')).read()
ast.parse(src)

check("main.py configures the companion logger",
      'getLogger("Companion")' in src)
check("at INFO or lower", '_pkg_log.setLevel(logging.INFO)' in src)
check("attached to the console handler", '_pkg_log.addHandler(_handler)' in src)
check("attached to the CSV-head buffer",
      '_pkg_log.addHandler(_startup_buffer)' in src,
      "these ARE the 'configuration verifications' that buffer exists for")

# Every module that logs setup detail must use a logger under Companion.*
print("\n  modules emitting operator-facing diagnostics:")
for rel in ('adapters/lua_deploy.py', 'adapters/lua_setup.py',
            'adapters/lua_offload.py'):
    p = os.path.join(ROOT, 'Companion', rel)
    t = open(p).read()
    ok = 'logging.getLogger(__name__)' in t
    check(f"    {rel} uses __name__", ok,
          "" if ok else "a hardcoded logger name would escape the package config")

print("\n" + "=" * 70)
print("PASSED" if not fails else f"FAILED: {fails}"); print("=" * 70)
sys.exit(1 if fails else 0)
