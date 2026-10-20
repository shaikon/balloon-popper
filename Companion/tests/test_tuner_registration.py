import re
# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
"""A tuner switch must be registered in EVERY registry, not just some.

Adding a discrete switch touches four independent places:

    DISCRETE_PARAMS    the value set            -> the switch exists
    DISCRETE_CAPTIONS  the labels               -> it has readable positions
    _RUNTIME / gains   where the value lives    -> get()/set() resolve it
    DEFAULT_CONST      the constant to restore  -> the reset button works

Miss one and the failure is partial and confusing rather than loud.  A switch
registered in only two of the four draws with a BLANK value and no info button,
and clicking it raises

    AttributeError: 'ControlGains' object has no attribute '<name>'

because get() falls through to the gains object for anything not in _RUNTIME.

This checks every discrete param, so the next switch cannot ship half-wired.
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
import dataclasses
from Companion import controller_autotune_store as store
from Companion.ui import controller_autotune as ca
from Companion import constants as C
from Companion.control_laws import ControlGains

Store = next(v for v in vars(store).values()
             if isinstance(v, type) and hasattr(v, '_RUNTIME'))
GAIN_FIELDS = {f.name for f in dataclasses.fields(ControlGains)}

fails = []
def check(n, c, d=""):
    print(f"  [{'PASS' if c else 'FAIL'}] {n}" + (f"   {d}" if d else ""))
    if not c: fails.append(n)

print("\n" + "=" * 74)
print("TUNER SWITCH REGISTRATION")
print("=" * 74)
print(f"  {len(ca.DISCRETE_PARAMS)} discrete params declared\n")

for attr, values in sorted(ca.DISCRETE_PARAMS.items()):
    routed = attr in Store._RUNTIME or attr in Store._GIMBAL or attr in GAIN_FIELDS
    const = store.DEFAULT_CONST.get(attr)
    problems = []
    # Captions are OPTIONAL by design: _discrete_caption falls back to the bare
    # number, and controller_autotune.py already validates count consistency at
    # import (line ~803).
    # Flagging them here would be this test inventing a rule the code does not
    # have.  What is NOT optional is routing and the reset default.
    if not routed:
        problems.append("not in _RUNTIME and not a ControlGains field "
                        "-> get() will raise")
    if const is None:
        problems.append("no DEFAULT_CONST -> reset button does nothing")
    elif not hasattr(C, const):
        problems.append(f"DEFAULT_CONST points at missing constant {const}")
    check(f"{attr:<28}", not problems, "; ".join(problems))

print("\n  round-trip through the real store:")
class _FakeController:
    def operator_gains(self): return None
fc = _FakeController()
for attr in sorted(a for a in ca.DISCRETE_PARAMS if a in Store._RUNTIME):
    setattr(fc, attr, ca.DISCRETE_PARAMS[attr][0])
s = Store(fc)
for attr, values in sorted(ca.DISCRETE_PARAMS.items()):
    if attr not in Store._RUNTIME:
        continue
    try:
        s.set(attr, values[-1])
        hi = s.get(attr)
        s.reset_one(attr)
        lo = s.get(attr)
        ok = abs(hi - float(values[-1])) < 1e-9 and abs(lo - float(getattr(C, store.DEFAULT_CONST[attr]))) < 1e-9
        check(f"    {attr:<26} set/get/reset", ok, f"set->{hi:g} reset->{lo:g}")
    except Exception as exc:
        check(f"    {attr:<26} set/get/reset", False, repr(exc)[:60])

# ---------------------------------------------------------------------------
# EVERY ROW MUST RESOLVE TO SOMETHING THE STORE CAN READ AND WRITE.
#
# ControllerAutotuneStore.get() dispatches three ways: _GIMBAL, _RUNTIME (an
# attribute on the CONTROLLER), and otherwise a field on ControlGains.  A row
# that is in none of them falls through to the gains object, which does not have
# it - and the tuner shows 0 while the real value lives on the controller,
# untouched by anything typed into the GUI.
#
# That is exactly what happened to point_lon_deg / point_lat_deg: the only two
# of the seven point_* rows left out of _RUNTIME.  The tuner read 0, the p-test
# injected the controller's real 75 / -40, and editing the field did nothing.
# The checks above all passed throughout - they verify a row HAS an info entry
# and a reset mapping, not that it points at a real variable.
print()
print("every tuner row resolves to a real variable:")
_ssrc = open(os.path.join(os.path.dirname(__file__), "..",
                          "controller_autotune_store.py")).read()
_rt = set(re.findall(r"'([a-z0-9_]+)'",
                     re.search(r"_RUNTIME\s*=\s*\{(.*?)\}", _ssrc, re.S).group(1)))
_gm = set(re.findall(r"'([a-z0-9_]+)'",
                     re.search(r"_GIMBAL\s*=\s*\{(.*?)\}", _ssrc, re.S).group(1)))
_gf = {f.name for f in dataclasses.fields(ControlGains)}
from Companion.ui.controller_autotune import TUNER_GROUPS as _TG
_orphan = sorted(a for _, _grp in _TG for _, a in _grp
                 if a not in _rt and a not in _gm and a not in _gf)
check("    no row falls through to nothing", not _orphan,
      "%s - tuner shows 0 and edits are discarded" % _orphan)

# A typed value that the link cannot carry does not fail at the tuner: it is
# accepted, and the control loop dies on the next tick when struct refuses to
# pack it.  Every value the store will accept must fit the field that carries
# it to the FC - GUIDE for settings sent every tick, SET_GAIN (a 32-bit float)
# for gains.
print()
print("no value the tuner accepts can crash the link:")
from Companion import protocol as _P
_TUNER = {a for _, _grp in _TG for _, a in _grp}
_GUIDE = {f.name for f in _P.GUIDE}
for _a in sorted(_TUNER & _GUIDE):
    _flo, _fhi = store.guide_field_range(_a)
    if _a in ca.DISCRETE_PARAMS:
        _vals = ca.DISCRETE_PARAMS[_a]
        check(f"    {_a}: every position fits its GUIDE field",
              all(_flo <= v <= _fhi for v in _vals), f"{_vals} vs {_flo}..{_fhi}")
    else:
        _lo, _hi = store.limits_for(_a)
        check(f"    {_a}: the tuner range fits its GUIDE field",
              _lo is not None and _hi is not None and _flo <= _lo and _hi <= _fhi,
              f"tuner {_lo}..{_hi}, field {_flo}..{_fhi}")
_gidx = _P.gain_index_map(ControlGains)
_crash = []
for _a in sorted(_TUNER - set(ca.DISCRETE_PARAMS)):
    for _typed in (1e300, -1e300, 1e39, -1e39, 3.5e38, 1e30):
        _v, _st = store.clamp_param(_a, _typed)
        if _st == 'rejected':
            continue
        try:
            if _a in _GUIDE:
                _P.pack(_P.MSG_GUIDE, {_a: _v})
            if _a in _gidx:
                _P.pack(_P.MSG_SET_GAIN, dict(index=_gidx[_a], value=_v))
        except Exception as _e:                       # noqa: BLE001
            _crash.append(f"{_a}={_typed:g} -> {type(_e).__name__}")
check("    typing a huge number into any row cannot crash the link", not _crash,
      ", ".join(_crash[:4]))

# Every saved ArduPlane parameter row carries an info button like every other
# row; its text is built from the name, so an unknown suffix is a KeyError the
# moment the window opens.
print()
print("every saved ArduPlane parameter row has info text:")
from Companion.constants import ARDUPLANE_AUTOTUNE_PARAMS as _AP
_noinfo = []
for _n in _AP:
    try:
        if not ca.native_param_info(_n).strip():
            _noinfo.append(_n)
    except Exception as _e:                            # noqa: BLE001
        _noinfo.append(f"{_n} ({type(_e).__name__})")
check(f"    all {len(_AP)} have one", not _noinfo, str(_noinfo[:4]))

# The GUI Live Values copy is a drop-in constants.py: it must load as one, with
# every tunable at the value the tuner holds - infinity included, which
# repr() spells as a bare `inf` that is no Python name.
print("\nthe GUI Live Values copy loads as a constants.py, every tunable included:")
import tempfile, math as _m
_live = {a: float(getattr(C, c)) for a, c in store.DEFAULT_CONST.items()}
_live['pitch_gain'] = 4.25                            # an edited value
_st = object.__new__(store.ControllerAutotuneStore)
_st.get = lambda attr: _live[attr]
_cwd = os.getcwd()
with tempfile.TemporaryDirectory() as _tmp:
    os.chdir(_tmp)
    try:
        _path = _st.write_current_constants('gui_live_values_test.py')
        _ns, _err = {}, None
        try:
            exec(compile(open(_path).read(), _path, 'exec'), _ns)
        except Exception as _e:                        # noqa: BLE001
            _err = f"{type(_e).__name__}: {_e}"
    finally:
        os.chdir(_cwd)
check("    the copy runs as Python", _err is None, _err or "")
_wrong = [c for a, c in store.DEFAULT_CONST.items()
          if _err is None and _ns.get(c) != _live[a]]
check("    every tunable carries the live value, an infinite distance and an edit included",
      _err is None and not _wrong and _m.isinf(_ns.get('POINT_BALLOON_RANGE', 0.0))
      and _ns.get('PITCH_GAIN') == 4.25, str(_wrong[:4]))

# The ArduPlane parameters: the operator's ACRO values (the in-memory set, not
# what the FC holds in LOITER) become the stored Acro defaults; the attitude
# settings, which have none, are listed in the banner.  An FC with
# YAW_RATE_ENABLE=0 reports no YAW_RATE_*: those stay as they were.
print("\nthe copy carries the ACRO values of the ArduPlane parameters:")
from Companion import constants_suggest as _cs
check("    every stored Acro default has a place to be written, and only those",
      set(store.ACRO_DEFAULT_ENTRIES) == set(C.ACRO_DEFAULT_PARAMS)
      and all(getattr(C, w) == C.ACRO_DEFAULT_PARAMS[n]
              for n, (w, k) in store.ACRO_DEFAULT_ENTRIES.items() if k is None),
      str(sorted(set(store.ACRO_DEFAULT_ENTRIES) ^ set(C.ACRO_DEFAULT_PARAMS))))
_acro = {n: 0.125 * (i + 1) for i, n in enumerate(C.ARDUPLANE_AUTOTUNE_PARAMS)
         if not n.startswith('YAW_')}
_st._c = type('Ctl', (), {})()
_st._c.link = type('Link', (), {})()
_st._c.link.native_params_operator = _acro
_src_fn = _cs._live_constants_source
with tempfile.TemporaryDirectory() as _tmp:
    os.chdir(_tmp)
    try:
        _path = _st.write_current_constants('gui_live_values_test.py')
        _text = open(_path).read()
        _ns = {}
        exec(compile(_text, _path, 'exec'), _ns)
        # Installed as constants.py, the next copy carries one banner, not two.
        _cs._live_constants_source = lambda: (_path, _text)
        _text2 = open(_st.write_current_constants('gui_live_values_test2.py')).read()
    finally:
        _cs._live_constants_source = _src_fn
        os.chdir(_cwd)
_exp, _nat = _ns['EXPECTED_RATE_PIDS'], _ns['NATIVE_MODE_RATE_PIDS']
_bad = [n for n in C.ACRO_DEFAULT_PARAMS if n in _acro
        and _ns['ACRO_DEFAULT_PARAMS'][n] != _acro[n]]
check("    each ACRO value is the stored Acro default in the copy, the _FLTE expected values included",
      not _bad and _ns['PTCH_RATE_FLTE_EXPECTED'] == _acro['PTCH_RATE_FLTE']
      and _ns['RLL_RATE_FLTE_EXPECTED'] == _acro['RLL_RATE_FLTE'], str(_bad[:4]))
check("    the Autopilot set and a value the FC did not report are left as they were",
      _nat == C.NATIVE_MODE_RATE_PIDS
      and all(_exp[n] == C.EXPECTED_RATE_PIDS[n] for n in _exp if n.startswith('YAW_'))
      and 'not reported by the FC' in _text and 'YAW_RATE_P' in _text)
check("    the attitude settings are listed in the banner with their ACRO values",
      all(('#   - %s = %s' % (n, _acro[n])) in _text for n in C.ARDUPLANE_AUTOTUNE_PARAMS
          if n not in C.ACRO_DEFAULT_PARAMS))
check("    a copy taken from an installed copy carries one banner",
      _text2.count('# Copied from GUI live values') == 1)

print("\nthe controller starts every gain at its constants.py value:")
# A ControlGains field the controller leaves out takes the dataclass default,
# and the FC flies that default from the first gain push while the tuner's
# reset, the log and constants.py all say otherwise.
import ast as _ast
_ctl = open(os.path.join(os.path.dirname(store.__file__), 'controller.py')).read()
_kw = {}
for _n in _ast.walk(_ast.parse(_ctl)):
    if isinstance(_n, _ast.Call) and getattr(_n.func, 'id', None) == 'ControlGains':
        _kw.update({k.arg: k.value for k in _n.keywords})
_wrong = []
for _f in dataclasses.fields(ControlGains):
    _const = store.DEFAULT_CONST.get(_f.name)
    if _const is None:
        continue
    if _f.name in _kw:
        _v = _kw[_f.name]
        if not (isinstance(_v, _ast.Attribute) and _v.attr == _const):
            _wrong.append(f"{_f.name} is not built from C.{_const}")
    elif _f.default != getattr(C, _const):
        _wrong.append(f"{_f.name} left at {_f.default!r}, C.{_const} = {getattr(C, _const)!r}")
check("    every gain with a stored default is built from it", not _wrong,
      "; ".join(_wrong[:4]))

print("\n" + "=" * 74)
print("PASSED" if not fails else f"FAILED: {len(fails)} - {fails[:4]}")
print("=" * 74)
sys.exit(1 if fails else 0)
