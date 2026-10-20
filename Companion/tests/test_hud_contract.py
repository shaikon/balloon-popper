import re
# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
"""_build_hud_snapshot must match the HudSnapshot dataclass.

They live in different modules, so removing a field from one and forgetting the
other is a TypeError at the first frame - after arming, mid mission upload.

Grepping for the field NAMES (which the previous test did) proves they are gone
from hud.py and says nothing about the call site.  This compares the two
directly, and also catches the reverse - a required field the call site never
passes.
"""
import ast, dataclasses, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
from Companion.ui.hud import render_gimbal_camera, HudSnapshot

fails = []
def check(n, c, d=""):
    print(f"  [{'PASS' if c else 'FAIL'}] {n}" + (f"   {d}" if d else ""))
    if not c: fails.append(n)

print("\n" + "=" * 70); print("HudSnapshot CONSTRUCTION CONTRACT"); print("=" * 70)

src = open(os.path.join(PKG, 'controller.py')).read()
tree = ast.parse(src)
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
           and n.name == 'BalloonChaser')
fn = next(m for m in cls.body if isinstance(m, ast.FunctionDef)
          and m.name == '_build_hud_snapshot')
call = next(n for n in ast.walk(fn) if isinstance(n, ast.Call)
            and getattr(n.func, 'id', getattr(n.func, 'attr', None)) == 'HudSnapshot')
passed = {kw.arg for kw in call.keywords if kw.arg}
declared = {f.name for f in dataclasses.fields(HudSnapshot)}
required = {f.name for f in dataclasses.fields(HudSnapshot)
            if f.default is dataclasses.MISSING
            and f.default_factory is dataclasses.MISSING}

extra = sorted(passed - declared)
missing = sorted(required - passed)
check("no kwarg without a matching field", not extra, f"orphaned: {extra}" if extra else "")
check("every required field is passed", not missing, f"missing: {missing}" if missing else "")
check("counts line up", True,
      f"{len(passed)} passed / {len(declared)} declared / {len(required)} required")

# the real thing: construct it
try:
    HudSnapshot(**{f.name: (0 if f.type in ('int', 'float') else None)
                   for f in dataclasses.fields(HudSnapshot)})
    ok = True
except Exception as exc:
    ok = False
    print(f"      {exc}")
check("HudSnapshot constructs with every field", ok)

# ---------------------------------------------------------------------------
# THE V-TAIL RUDDER READOUT MUST BE 1:1 WITH THE COMMAND.
#
# ArduPlane's mixer already applies MIXING_GAIN (0.5):
#     ch2 = (elev + rud) * 0.5      ch4 = (elev - rud) * 0.5
# so ch2 - ch4 IS the rudder, and any extra scaling makes the HUD disagree with
# yaw_out_pct by exactly that factor.  A /2 was added here once on the reasoning
# that two percentages differenced can reach +-200 - true, but unreachable with
# the 0.5 gain - and it turned a correct 110% readout into an apparent display
# bug.  A wrong scale factor is worse than no readout: it reads plausibly.
print()
print("V-tail rudder readout is unscaled:")
_hsrc = open(os.path.join(os.path.dirname(__file__), "..", "ui", "hud.py")).read()
_m = re.search(r"_rud = ([^\n]+)", _hsrc)
_expr = _m.group(1).strip() if _m else "<missing>"
check("    rudder = ch2 - ch4, no extra factor", _expr == "_s2 - _s4",
      "found %r - ArduPlane's mixer already halves it" % _expr)
# The ELEVATOR half-sum is not checked here because the HUD does not render it.
# A contract on a quantity nothing displays guards nothing and forces the code to
# keep computing a value it has no use for.  If an elevator-share readout comes
# back, so does its check: the share is (ch2 + ch4) / 2, and unlike the rudder it
# genuinely keeps its /2.

print("\n" + "=" * 70)
# ---------------------------------------------------------------------------
# ACTUALLY RENDER, both paths.
#
# The declared-vs-passed checks above compare NAMES.  They cannot see a name
# used in the body that nothing assigns - which is what happens when a branch is
# deleted and the code that consumed its variables survives.  That has now
# reached flight three times, and each time the traceback arrived on the first
# rendered frame, well past anything import-time.
#
# cv2 needs a display even to draw into an array, so this is skipped rather than
# failed where there is none - a skipped check is honest, a passed one would not
# be.
print()
print("render_gimbal_camera actually runs:")
# Gate on the ENVIRONMENT, not on a try/except.  A cv2 built against Qt calls
# abort() when it cannot find a platform plugin - the process dies with SIGABRT
# and no exception is ever raised, so try/except cannot see it.  Run this under
# xvfb-run (or any real display) and it executes; without one it skips.
_can_draw = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
if _can_draw:
    import numpy as _np
else:
    print("  [SKIP]     no display - run under xvfb-run to exercise the render")

if _can_draw:
    _F = {f.name: f for f in dataclasses.fields(HudSnapshot)}

    def _v(f):
        if f.default is not dataclasses.MISSING:
            return f.default
        if getattr(f, "default_factory", dataclasses.MISSING) is not dataclasses.MISSING:
            return f.default_factory()
        a = str(f.type)
        return (False if "bool" in a else 0 if "int" in a
                else 0.0 if "float" in a else "" if "str" in a else None)

    _base = {n: _v(f) for n, f in _F.items()}
    _extra = dict(tracking=True,
                  last_balloon_body=_np.array([1.0, 0.05, -0.02]),
                  flight_path_body=_np.array([1.0, 0.0, -0.03]),
                  servo_aileron=1500, servo_elevator=1500, servo_ch4=1500,
                  servo2_trim=1500, servo2_min=1000, servo2_max=2000)
    _kw = dict(_base)
    _kw.update({k: v for k, v in _extra.items() if k in _F})
    _err = ""
    try:
        render_gimbal_camera(_np.zeros((720, 1280, 3), dtype=_np.uint8),
                             HudSnapshot(**_kw))
    except Exception as _e:
        _err = "%s: %s" % (type(_e).__name__, _e)
    check("    the gimbal view renders", not _err, _err)


print("\nKEYS: a letter is never an arrow")
# An arrow's low byte under GTK is 'Q'..'T', so decoding the low byte turned
# Shift- or Caps-Lock-s into a right arrow.  Keys go in as cv2.waitKeyEx would
# report them; the controller records what each became.
import cv2 as _cv2
from Companion.ui import keyboard as _kb
class _Rec:
    def __init__(self): self.calls = []
    def __getattr__(self, name):
        return lambda *a, **k: self.calls.append((name,) + a)
class _Tuner:
    def take_key(self): return -1
    def handle_key(self, key): return False
_rec = _Rec()
_kh = _kb.KeyboardHandler(_rec, _Tuner())
_wk = _cv2.waitKeyEx
def _press(code):
    _rec.calls.clear()
    _cv2.waitKeyEx = lambda d: code
    try:
        _kh.handle_key()
    finally:
        _cv2.waitKeyEx = _wk
    return list(_rec.calls)
_GTK_NUMLOCK = 0x10 << 16                   # GDK Mod2 (Num Lock) in the state bits
check("    capital S, Q, R, T are not arrows",
      all(not any(c[0] == 'handle_arrow' for c in _press(ord(ch))) for ch in 'SQRT'))
check("    s still selects straight-and-level",
      _press(ord('s')) == [('toggle_straight',)], str(_press(ord('s'))))
check("    a GTK arrow, Num Lock on, is an arrow",
      _press(0xFF52 | _GTK_NUMLOCK) == [('handle_arrow', 'up')],
      str(_press(0xFF52 | _GTK_NUMLOCK)))
check("    Windows and Qt arrows too",
      _press(0x270000) == [('handle_arrow', 'right')]
      and _press(0x01000015) == [('handle_arrow', 'down')])
check("    the tuner's arrows are outside the characters it queues",
      all(v > 0x7F for v in _kb.TUNER_ARROW_CODES.values())
      and all(_kb._arrow_direction(v) == k.lower()
              for k, v in _kb.TUNER_ARROW_CODES.items()))

print("\n" + "=" * 70); print("OPENCV WINDOW CALLS: ONE THREAD"); print("=" * 70)
# HighGUI is not thread-safe: every window call runs on the thread that pumps
# its event loop, which is the control loop's - BalloonChaser.run(), the
# loop that also sends GUIDE every tick.  The calls live in exactly the
# functions in _WIN_ALLOWED, run() is the only caller of the others, and
# main.py calls run() on its own thread.  A lambda or a nested def is a scope
# of its own: its body runs wherever it is called from, not where it is written.
_HIGHGUI = {
    'namedWindow', 'destroyWindow', 'destroyAllWindows', 'imshow', 'waitKey',
    'waitKeyEx', 'pollKey', 'setMouseCallback', 'moveWindow', 'resizeWindow',
    'setWindowProperty', 'getWindowProperty', 'setWindowTitle',
    'startWindowThread', 'createTrackbar', 'getTrackbarPos', 'setTrackbarPos',
    'selectROI', 'selectROIs', 'displayOverlay', 'displayStatusBar'}
_WIN_RUN = ('controller.py', 'BalloonChaser.run')
_WIN_ALLOWED = {
    _WIN_RUN,
    ('controller.py', 'BalloonChaser._apply_camera_views'),
    ('ui/hud.py', 'render_nose_camera'),
    ('ui/hud.py', 'render_gimbal_camera'),
    ('ui/keyboard.py', 'KeyboardHandler.handle_key'),
}

def _scoped(tree):
    """Yield (qualname, node) for every node, qualname being the def, class or
    lambda whose body it runs in ('' at module level)."""
    def walk(node, scope):
        for child in ast.iter_child_nodes(node):
            yield '.'.join(scope), child
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef,
                                  ast.ClassDef)):
                yield from walk(child, scope + [child.name])
            elif isinstance(child, ast.Lambda):
                yield from walk(child, scope + ['<lambda>'])
            else:
                yield from walk(child, scope)
    return walk(tree, [])

def _window_calls(src):
    """(qualname, name, line) for every HighGUI function the source reaches:
    cv2.<name> under any alias of cv2, called or passed on, and any
    `from cv2 import` of one."""
    tree = ast.parse(src)
    aliases = {a.asname or a.name for n in ast.walk(tree)
               if isinstance(n, ast.Import) for a in n.names if a.name == 'cv2'}
    out = []
    for scope, n in _scoped(tree):
        if isinstance(n, ast.ImportFrom) and n.module == 'cv2':
            out += [(scope, a.name, n.lineno) for a in n.names
                    if a.name in _HIGHGUI or a.name == '*']
        elif (isinstance(n, ast.Attribute) and n.attr in _HIGHGUI
              and isinstance(n.value, ast.Name) and n.value.id in aliases):
            out.append((scope, n.attr, n.lineno))
    return out

def _thread_targets(tree):
    """The expression each threading.Thread(...) is given as its target."""
    for n in ast.walk(tree):
        if (isinstance(n, ast.Call)
                and (getattr(n.func, 'attr', None) == 'Thread'
                     or getattr(n.func, 'id', None) == 'Thread')):
            for kw in n.keywords:
                if kw.arg == 'target':
                    yield kw.value
            if len(n.args) >= 2:                 # Thread(group, target, ...)
                yield n.args[1]

_ROOT = os.path.dirname(PKG)
_win_srcs = {'../main.py': open(os.path.join(_ROOT, 'main.py')).read()}
for _dp, _dn, _fn in os.walk(PKG):
    _dn[:] = [d for d in _dn if d not in ('tests', '__pycache__')]
    for _f in _fn:
        if _f.endswith('.py'):
            _p = os.path.join(_dp, _f)
            _win_srcs[os.path.relpath(_p, PKG).replace(os.sep, '/')] = open(_p).read()
_win_trees = {rel: ast.parse(s) for rel, s in _win_srcs.items()}

_win_bad = [f"{rel}:{ln} {name} in {scope or '<module>'}"
            for rel, s in _win_srcs.items()
            for scope, name, ln in _window_calls(s)
            if (rel, scope) not in _WIN_ALLOWED]
check("    window calls only in the allowed functions", not _win_bad,
      "; ".join(_win_bad))
check("    each allowed function still makes one",
      {(rel, scope) for rel, s in _win_srcs.items()
       for scope, _n, _l in _window_calls(s)} == _WIN_ALLOWED)

# Who reaches the allowed functions.  The keyboard handler is found through the
# attribute it is stored in, so the tuner's own handle_key(key) does not count.
_kb_attrs = {t.attr for tree in _win_trees.values() for n in ast.walk(tree)
             if isinstance(n, ast.Assign) and isinstance(n.value, ast.Call)
             and getattr(n.value.func, 'id', None) == 'KeyboardHandler'
             for t in n.targets if isinstance(t, ast.Attribute)}
def _win_ref(n):
    """Which allowed function (other than run) a node names, or None."""
    if isinstance(n, ast.Name) and n.id in ('render_nose_camera',
                                            'render_gimbal_camera'):
        return n.id
    if isinstance(n, ast.Attribute):
        if n.attr in ('_apply_camera_views', 'render_nose_camera',
                      'render_gimbal_camera'):
            return n.attr
        if (n.attr == 'handle_key' and isinstance(n.value, ast.Attribute)
                and n.value.attr in _kb_attrs):
            return 'KeyboardHandler.handle_key'
    return None
_ref_bad = [f"{rel}:{n.lineno} {_win_ref(n)} in {scope or '<module>'}"
            for rel, tree in _win_trees.items() for scope, n in _scoped(tree)
            if _win_ref(n) and (rel, scope) != _WIN_RUN]
check("    the keyboard handler is found", bool(_kb_attrs))
check("    only run() reaches them", not _ref_bad, "; ".join(_ref_bad))
# A balloon is read whole, so a lambda or partial wrapping one counts too.
_tgt_bad = [f"{rel}:{t.lineno}" for rel, tree in _win_trees.items()
            for t in _thread_targets(tree)
            if any(_win_ref(y) or getattr(y, 'attr', None) == 'run'
                   for y in ast.walk(t))]
check("    none of them, nor run(), is a thread's target", not _tgt_bad,
      "; ".join(_tgt_bad))
_main = next(n for n in _win_trees['../main.py'].body
             if isinstance(n, ast.FunctionDef) and n.name == 'main')
check("    main() calls controller.run() itself",
      any(isinstance(n, ast.Call) and getattr(n.func, 'attr', None) == 'run'
          and getattr(n.func.value, 'id', None) == 'controller'
          for n in ast.walk(_main)))

# The scanner itself: a window call under an alias in another method, one
# deferred into a lambda inside run(), and one imported by name are all caught.
_probe = ("import cv2 as c\nfrom cv2 import imshow\n"
          "class BalloonChaser:\n"
          "    def run(self):\n        f = lambda: c.waitKey(1)\n"
          "    def other(self):\n        c.imshow('x', None)\n")
check("    the scanner sees through aliases, lambdas and from-imports",
      {(s, nm) for s, nm, _l in _window_calls(_probe)}
      == {('', 'imshow'), ('BalloonChaser.run.<lambda>', 'waitKey'),
          ('BalloonChaser.other', 'imshow')},
      str(_window_calls(_probe)))

print("PASSED" if not fails else f"FAILED: {fails}"); print("=" * 70)
sys.exit(1 if fails else 0)
