import re
# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
"""CSV header and row must have the same length.

They are two separate literals in csv_logger.py. Adding a column to one and not
the other shifts every value after the insertion point - silently. The file
still parses, the plots still draw, and every number past that column is
attributed to the wrong name.
"""
import ast, dataclasses, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from Companion.csv_logger import CsvLogger, TickSnapshot

fails = []
def check(n, c, d=""):
    print(f"  [{'PASS' if c else 'FAIL'}] {n}" + (f"   {d}" if d else ""))
    if not c: fails.append(n)

print("\n" + "=" * 70); print("CSV HEADER / ROW ALIGNMENT"); print("=" * 70)

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src = open(os.path.join(PKG, 'csv_logger.py')).read()
cls = next(n for n in ast.parse(src).body
           if isinstance(n, ast.ClassDef) and n.name == 'CsvLogger')
fn = next(m for m in cls.body
          if isinstance(m, ast.FunctionDef) and m.name == 'write_tick')
row = max((n for n in ast.walk(fn) if isinstance(n, (ast.List, ast.Tuple))),
          key=lambda n: len(n.elts))
hdr = CsvLogger.HEADER if isinstance(CsvLogger.HEADER, (list, tuple)) \
      else CsvLogger.HEADER.split(',')

check("header length == row length", len(row.elts) == len(hdr),
      f"header {len(hdr)}, row {len(row.elts)}")

# defaulted fields must come last, or TickSnapshot raises at import
flds = dataclasses.fields(TickSnapshot)
seen, bad = False, []
for f in flds:
    if f.default is not dataclasses.MISSING:
        seen = True
    elif seen:
        bad.append(f.name)
check("TickSnapshot field order is legal", not bad, f"{bad[:4]}" if bad else
      f"{len(flds)} fields")

print("\n  Lua-path mirror columns present:")
for name in ('lua_omega_x', 'lua_omega_y', 'lua_gate',
             'lua_trust', 'lua_dt_ms', 'lua_frames'):
    check(f"    {name}", name in hdr,
          "" if name in hdr else "the FC's own view is invisible without it")



# ---------------------------------------------------------------------------
# SNAPSHOT CONSTRUCTION must match the dataclass, and the HUD must not read a
# field that does not exist.
#
# Both are AttributeError/TypeError at RUNTIME, in the render path or the first
# logged tick - after takeoff, not at import. A field added to one side and not
# the other passes every import-time check and crashes the main loop.
# ---------------------------------------------------------------------------
import dataclasses as _dc
from Companion.ui.hud import HudSnapshot as _Hud
from Companion.csv_logger import TickSnapshot as _Tick

_ctl = open(os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "controller.py")).read()
_hud_src = open(os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "ui", "hud.py")).read()

print("\nsnapshot constructions match their dataclass:")
# PARSED, NOT PATTERN-MATCHED.  A regex anchored at start-of-line captures only
# the FIRST keyword on each line - harmless for spotting a kwarg with no field,
# since that is a subset test, but it silently under-counts and makes the
# required-fields check below meaningless.  ast reads every keyword wherever it
# sits.
_tree = ast.parse(_ctl)
for _name, _cls in (("HudSnapshot", _Hud), ("TickSnapshot", _Tick)):
    _have = {f.name for f in _dc.fields(_cls)}
    _req  = {f.name for f in _dc.fields(_cls)
             if f.default is _dc.MISSING and f.default_factory is _dc.MISSING}
    for _n in ast.walk(_tree):
        if not (isinstance(_n, ast.Call) and getattr(_n.func, "id", None) == _name):
            continue
        _kw   = {k.arg for k in _n.keywords if k.arg is not None}
        _star = any(k.arg is None for k in _n.keywords)   # **something
        _bad = sorted(_kw - _have)
        check(f"    {_name}({len(_kw)} kwargs)", not _bad,
              f"unknown: {_bad} - TypeError on the first tick")
        # AND THE OTHER DIRECTION.  The check above only catches a kwarg with no
        # field.  The reverse - a REQUIRED field that nobody passes - is the same
        # TypeError on the same first tick, and it is what happens when a column
        # is dropped from the constructor while its field declaration survives.
        # A one-directional contract is half a contract.
        if not _star and not _n.args:
            _missing = sorted(_req - _kw)
            check(f"    {_name}({len(_req)} required fields all passed)", not _missing,
                  f"never passed: {_missing} - TypeError on the first tick")

# ---------------------------------------------------------------------------
# NO COLUMN MAY CARRY ANOTHER COLUMN'S VALUE.
#
# The length check above passes happily on a SHUFFLE.  Four yaw columns were
# swapped for a week - lua_yaw_p_pct held the integrator, lua_yaw_i_pct held the
# proportional term - and the flight data read as "the integrator never ran"
# when it had in fact been saturated at its clamp the whole time.  A whole
# analysis was built on it.
#
# It happens when entries are inserted after a fixed anchor: each new one lands
# in front of the last, so the row ends up reversed against the header while
# both still have the same length.
#
# Only exact swaps are flagged.  Shortened names (time <- s.t, req_roll <-
# s.requested_roll) are deliberate and must not fail.
print()
print("no column carries another column's value:")
_hdr = None
for _n in ast.walk(ast.parse(src)):
    if isinstance(_n, ast.Assign):
        for _t in _n.targets:
            if (getattr(_t, "id", None) or getattr(_t, "attr", None)) == "HEADER":
                _hdr = [e.value for e in _n.value.elts]
_rowsrc = [ast.get_source_segment(src, e) for e in row.elts]
_var = [(re.findall(r"s\.([a-z0-9_]+)", r or "") or [None])[0] for r in _rowsrc]
_at = {v: k for k, v in enumerate(_var) if v}
_swap = ["col %d '%s' writes s.%s" % (k, h, _var[k])
         for k, h in enumerate(_hdr) if h in _at and _at[h] != k]
check("    header name matches the value written", not _swap, str(_swap[:6]))

print("\nthe HUD only reads fields HudSnapshot defines:")
_have = {f.name for f in _dc.fields(_Hud)}
# Parse it, do not grep it: a mention inside a COMMENT is not a read, and
# flagging one sends the next reader hunting for a bug that is not there.
_used = {_n.attr for _n in ast.walk(ast.parse(_hud_src))
         if isinstance(_n, ast.Attribute) and getattr(_n.value, "id", "") == "s"}
_miss = sorted(u for u in _used if u not in _have)
check("    every s.<field> exists", not _miss,
      f"{_miss} - AttributeError in the render path")

# ---------------------------------------------------------------------------
# EVERY GAIN IS LOGGED, FROM THE SET FLYING NOW.
#
# A gain with no column cannot be read back from a flight, so a comparison
# between two flights silently compares unknown settings.  A column fed from
# anything but the live gains - a copy taken at startup, the operator's set
# while a measurement owns the gains, a constant - is worse: it reads as a
# record and is not one.  So each ControlGains field must reach a TickSnapshot
# kwarg whose value is read from self._gains (or _pg, the pitch set resolved
# from it the same tick).  Each such kwarg must then actually be written by
# write_tick.
# ---------------------------------------------------------------------------
print("\nevery gain is logged from the set flying now:")
from Companion.control_laws import ControlGains as _CG
_PITCH = {'gain': 'pitch_gain', 'ki': 'pitch_ki', 'windup_limit': 'windup_limit_pitch',
          'deadband_deg': 'pitch_deadband_deg', 'kd': 'pitch_kd', 'kd_rate': 'pitch_kd_rate'}
_tick = next(_n for _n in ast.walk(_tree) if isinstance(_n, ast.Call)
             and getattr(_n.func, "id", None) == "TickSnapshot")
_fn = next(_f for _f in ast.walk(_tree) if isinstance(_f, ast.FunctionDef)
           and any(_c is _tick for _c in ast.walk(_f)))
_pg_live = any(isinstance(_a, ast.Assign) and ast.unparse(_a.value) == "self._gains.pitch_gains"
               and any(getattr(_t, "id", None) == "_pg" for _t in _a.targets)
               for _a in ast.walk(_fn))
_wt_src = ast.get_source_segment(src, fn)
_logged = {}
for _k in _tick.keywords:
    _v = ast.unparse(_k.value)
    for _m in re.finditer(r"self\._gains\.([a-z0-9_]+)|getattr\(self\._gains, '([a-z0-9_]+)'", _v):
        _logged.setdefault(_m.group(1) or _m.group(2), _k.arg)
    for _m in re.finditer(r"\b_pg\.([a-z0-9_]+)", _v):
        if _pg_live and _m.group(1) in _PITCH:
            _logged.setdefault(_PITCH[_m.group(1)], _k.arg)
_unlogged = [f.name for f in _dc.fields(_CG) if f.name not in _logged]
check("    every ControlGains field has a column fed from the live gains", not _unlogged,
      str(_unlogged[:6]))
_unwritten = sorted(k for k in set(_logged.values())
                    if not re.search(r"\bs\.%s\b" % k, _wt_src))
check("    and write_tick writes every one of them", not _unwritten, str(_unwritten[:6]))
_kw_src = {_k.arg: ast.unparse(_k.value) for _k in _tick.keywords}

# The saved ArduPlane parameters: one column each, named exactly as on the FC,
# fed from the values the Gain Tuner holds (link.native_params - what the FC last
# confirmed).  Building a row must never read the FC: a MAVLink round trip per
# parameter per tick would stall the 50 Hz loop, and the tuner's Write and Read
# are the only paths that change these values.
print("\nthe saved ArduPlane parameters are logged as the tuner holds them:")
from Companion.constants import ARDUPLANE_AUTOTUNE_PARAMS as _AP
_nohdr = [n for n in _AP if n not in _hdr]
check("    every ARDUPLANE_AUTOTUNE_PARAMS name is a column", not _nohdr, str(_nohdr[:6]))
_np_src = any(isinstance(_a, ast.Assign) and "native_params" in ast.unparse(_a.value)
              and "self.link" in ast.unparse(_a.value)
              and any(getattr(_t, "id", None) == "_np" for _t in _a.targets)
              for _a in ast.walk(_fn))
_fed = {m.group(1) for v in _kw_src.values() for m in re.finditer(r"_np\.get\('([A-Z0-9_]+)'\)", v)}
check("    each is fed from link.native_params", _np_src and set(_AP) <= _fed,
      str(sorted(set(_AP) - _fed)[:6]))
_reads = sorted({getattr(_c.func, "attr", "") for _c in ast.walk(_fn) if isinstance(_c, ast.Call)}
                & {"read_native_param", "_fetch_param", "param_request_read_send",
                   "_read_native_params"})
check("    building a row reads nothing from the FC", not _reads, str(_reads))

# The flight path's angle off the nose is the FC's vertical marker (STATE
# vel_marker_deg), as read.
print("\nthe flight path's angle off the nose is logged from the FC:")
check("    lua_vel_marker is a column fed from the FC's marker",
      'lua_vel_marker' in _hdr
      and "lua_vel_marker_deg" in _kw_src.get('lua_vel_marker', ''),
      _kw_src.get('lua_vel_marker', 'not built'))

# The head of the log: every line before the column header is a comment, even
# when a console message or a note spans several lines.  An unmarked
# continuation line reads as a data row before the header, and a reader that
# skips comments (pandas, a spreadsheet import) stops or shifts there.
print("\nthe log head is comment lines only, a multi-line message included:")
import logging as _logging, tempfile as _tempfile
from Companion.csv_logger import StartupBufferHandler
_buf = StartupBufferHandler()
_buf.emit(_logging.LogRecord('balloon_chaser', _logging.INFO, __file__, 0,
                             'Subscribed to Gazebo camera topic:\n  /world/cam/image',
                             None, None))
_buf.emit(_logging.LogRecord('balloon_chaser', _logging.INFO, __file__, 0, '', None, None))
_cwd = os.getcwd()
with _tempfile.TemporaryDirectory() as _tmp:
    os.chdir(_tmp)                         # start() clears other flight_log_*.csv here
    try:
        _lg = CsvLogger(os.path.join(_tmp, 'flight_log_test.csv'), startup_buffer=_buf)
        _lg.header_notes = ['two-line\nnote']
        _lg.start(); _lg.stop()
        _lines = open(os.path.join(_tmp, 'flight_log_test.csv')).read().splitlines()
    finally:
        os.chdir(_cwd)
_hi = next((i for i, l in enumerate(_lines) if l.startswith('tick,')), None)
_bare = [l for l in _lines[:_hi or 0] if not l.startswith('#')]
check("    every line before the column header starts with '#'",
      _hi is not None and not _bare, str(_bare[:3]))
check("    and the continuation lines are kept, marked",
      '# log:   /world/cam/image' in _lines and '# note: note' in _lines,
      str(_lines[:_hi or 0][-6:]))

print("\nthe FC's PID_TUNING and AoA columns are blank until they arrive:")
# A reading that has not arrived written as 0.0 reads as a rate loop measuring
# zero desired and zero achieved, or an aircraft flying straight.
import types as _types
from Companion.adapters.mavlink_link import MavlinkAutopilotLink as _MAL
_none = _MAL.pid_tuning_field(_types.SimpleNamespace(_pid_tuning={}), 1, 'desired')
check("    pid_tuning_field gives None for an axis not streamed yet", _none is None, repr(_none))
_fc = [f for f in dataclasses.fields(TickSnapshot)
       if re.fullmatch(r'fc_(roll|pitch|yaw)_(des|ach|ff|p|i|d)|fc_aoa|fc_ssa', f.name)]
check("    their snapshot fields default to None", len(_fc) == 20
      and all(f.default is None for f in _fc), str([f.name for f in _fc if f.default is not None]))
_wt = ast.get_source_segment(src, fn)
_bare_fc = [f.name for f in _fc if ('_opt(s.%s,' % f.name) not in _wt]
check("    and write_tick writes each through _opt", not _bare_fc, str(_bare_fc))

print("\n" + "=" * 70)
print("PASSED" if not fails else f"FAILED: {fails}")
print("=" * 70)
sys.exit(1 if fails else 0)
