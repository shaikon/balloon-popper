# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
"""No statements after a return, and every __init__ is intact.

A textual edit that inserts a `def` in the middle of another function does not
break parsing - the remaining statements simply become unreachable code after
the new function's `return`.  ast.parse() is happy, the import succeeds, and the
failure surfaces only when something reads an attribute that was never set:

    'MavlinkAutopilotLink' object has no attribute '_telem_thread'

That happened when a @property was inserted into __init__, which silently
truncated it and moved 6 attribute assignments into the property's dead tail.
"""
import ast, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

fails = []
def check(n, c, d=""):
    print(f"  [{'PASS' if c else 'FAIL'}] {n}" + (f"   {d}" if d else ""))
    if not c: fails.append(n)

print("\n" + "=" * 74); print("UNREACHABLE CODE / TRUNCATED FUNCTIONS"); print("=" * 74)

dead = []
for base, _, files in os.walk(os.path.join(ROOT, 'Companion')):
    if '__pycache__' in base:
        continue
    for fn in files:
        if not fn.endswith('.py'):
            continue
        path = os.path.join(base, fn)
        try:
            tree = ast.parse(open(path).read())
        except SyntaxError:
            continue
        for node in ast.walk(tree):
            if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            body = node.body
            for i, stmt in enumerate(body[:-1]):
                if isinstance(stmt, (ast.Return, ast.Raise)):
                    nxt = body[i + 1]
                    dead.append(f"{os.path.relpath(path, ROOT)}:{nxt.lineno} "
                                f"in {node.name}() after {type(stmt).__name__.lower()}")
check("no statements after return/raise in any function body",
      not dead, "; ".join(dead[:4]) if dead else "")

print("\n  __init__ completeness (attributes other code depends on):")
# Parse, do not import: pymavlink is a runtime dependency and may be absent on
# a machine that only runs the verification suite.
src = open(os.path.join(ROOT, 'Companion', 'adapters', 'mavlink_link.py')).read()
cls = next(n for n in ast.parse(src).body
           if isinstance(n, ast.ClassDef) and n.name == 'MavlinkAutopilotLink')
init = next(m for m in cls.body
            if isinstance(m, ast.FunctionDef) and m.name == '__init__')
assigned = {t.attr for n in ast.walk(init) if isinstance(n, ast.Assign)
            for t in n.targets if isinstance(t, ast.Attribute)}
for attr in ('_telem_thread', '_mav_lock', '_servo1_function', '_servo2_function',
             '_fs_gcs_enabl'):
    check(f"    {attr}", attr in assigned)

print("\n" + "=" * 74)
print("PASSED" if not fails else f"FAILED: {fails[:3]}"); print("=" * 74)
sys.exit(1 if fails else 0)
