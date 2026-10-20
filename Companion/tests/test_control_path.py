"""_update_control must RUN, in every mode.
import os

The suite compared names for a long time and never executed the control path.
Five defects reached flight that way in a single week - a missing @property, a
deleted list initialiser, a call that lost an argument, a constant removed by
prefix match, a tuner row wired to nothing.  Every one surfaced on the first tick
after the operator pressed a key, because that is the first time this code runs.

The collaborators are stubs: the point is not to check the OUTPUT (there is no
aircraft here to be right about), it is to check that the path executes at all.
A stub that is too permissive is the hazard - __getattr__ returning a callable
for everything would hide exactly the AttributeError this exists to catch - so
every value the path reads is declared explicitly below and __getattr__ raises
on anything private.
"""

import sys,math,os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
import numpy as np
from Companion.controller import BalloonChaser

class StubLink:
    attitude=(0.0,0.0,0.0); vel_ned=np.array([15.0,0.0,0.0]); airspeed=15.0
    quat_age_s=0.0; attitude_age_s=0.0; pos_time_boot_s=0.0; alt_age_s=0.0
    attitude_quat=None; attitude_quat_recv_t=0.0; attitude_recv_t=0.0
    roll_rate=0.0; pitch_rate=0.0; yaw_rate=0.0; groundspeed=15.0
    rates=(0.0,0.0,0.0)
    acro_roll_rate=180.0; acro_pitch_rate=90.0; relative_alt=600.0
    position=(-35.3633,149.1652,600.0); quat=None; quat_fresh=False
    servo_out={}; elevon_mixing=False
    def aircraft_world_pos_ned(self): return np.zeros(3)
    def __getattr__(self,n):
        if n.startswith('_'): raise AttributeError(n)
        if n.startswith('lua_'): return 0.0
        # anything else could be a method or a value; a callable that also reads
        # as 0.0 is not possible, so default to a no-op callable and let the
        # data attributes above win.
        return lambda *a,**k: None
class StubSensor:
    def detection_norm(self): return (0.0,0.0)
    detection_valid=False
    def __getattr__(self,n):
        if n.startswith('_'): raise AttributeError(n)
        return lambda *a,**k: None
class StubGimbal:
    slew_rate_dps=200.0; measured_pan_deg=0.0; measured_tilt_deg=0.0
    def __getattr__(self,n): return (lambda *a,**k: None) if not n.startswith('_') else 0.0
class StubCam:
    width=1280; height=720; fx=276.2; fy=276.2; cx=160.0; cy=120.0
    def __getattr__(self,n):
        if n.startswith('_'): raise AttributeError(n)
        return (lambda *a,**k: None)

print("\n" + "="*70)
print("CONTROL PATH EXECUTES")
print("="*70)
c=BalloonChaser(StubLink(),StubGimbal(),StubSensor(),StubCam(),StubCam(),takeoff_alt=600.0,waypoint_lat=-35.3633,waypoint_lon=149.1652)
ok=True
for mode in ('idle','straight','balloon_chaser','manual','ptest'):
    try:
        c._mode=mode
        c._straight_level = (mode=='straight')
        c.tracking = (mode!='idle')
        c._update_control()
        print("  [PASS]     mode=%s"%mode)
    except Exception as e:
        ok=False; import traceback
        print("  [FAIL]     mode=%s -> %s: %s"%(mode,type(e).__name__,e))
        traceback.print_exc(limit=9)
# ---------------------------------------------------------------------------
# EVERY self.<attr> THE CONTROLLER READS IS ONE IT SETS.
#
# An attribute that is read but never assigned is an AttributeError on the tick
# that first reaches that line - which may be minutes into a flight, on a branch
# the tests never walk.  It has happened twice: a removed operator switch left
# three reads behind, and the main loop died after takeoff both times.
#
# getattr(self, 'x', default) is deliberately NOT counted as a read: that form
# says "may be absent" and handles it.  Only bare self.x is checked.
# ---------------------------------------------------------------------------
import ast as _ast
print()
print("every self.<attr> the controller reads is one it sets:")
_csrc = open(os.path.join(os.path.dirname(__file__), "..", "controller.py")).read()
_tree = _ast.parse(_csrc)
_cls = next(n for n in _ast.walk(_tree)
            if isinstance(n, _ast.ClassDef) and n.name == "BalloonChaser")
_set, _read = set(), {}
for n in _ast.walk(_cls):
    if isinstance(n, (_ast.FunctionDef, _ast.AsyncFunctionDef)) and n in _cls.body:
        _set.add(n.name)                       # methods and properties
    if isinstance(n, _ast.Attribute) and isinstance(n.value, _ast.Name) and n.value.id == "self":
        if isinstance(n.ctx, (_ast.Store, _ast.Del)):
            _set.add(n.attr)
        else:
            _read.setdefault(n.attr, n.lineno)
# getattr(self, 'x', ...) guards its own absence
for n in _ast.walk(_cls):
    if (isinstance(n, _ast.Call) and isinstance(n.func, _ast.Name) and n.func.id == "getattr"
            and len(n.args) >= 2 and isinstance(n.args[0], _ast.Name) and n.args[0].id == "self"
            and isinstance(n.args[1], _ast.Constant)):
        _set.add(n.args[1].value)
_orphans = sorted(a for a in _read if a not in _set)
if _orphans:
    ok = False
    print("  [FAIL]     no attribute is read without being set   %s" %
          [(a, _read[a]) for a in _orphans[:4]])
else:
    print("  [PASS]     no attribute is read without being set")


# ---------------------------------------------------------------------------
# A TUNER EDIT MADE MID-TEST REACHES THE FC AND CHANGES WHAT IT FLIES.
#
# End to end through the real code: the tuner store's set() (its only way in) ->
# the controller's per-tick _push_gains_if_changed -> LuaOffloadLink packing ->
# the bytes fed to balloon_chaser_fc.lua under the desktop stubs -> the pitch-rate
# command the script hands ArduPlane.  Nothing in between is stubbed except the
# serial port, which records bytes.  A gain that is set but never sent, sent
# but never applied, or applied a tick late fails here.
# ---------------------------------------------------------------------------
import re as _re, shutil as _sh, struct as _st, subprocess as _sp, tempfile as _tf
from Companion import protocol as _P
from Companion.adapters.lua_offload import LuaOffloadLink
from Companion.controller_autotune_store import ControllerAutotuneStore as _Store
print()
print("a tuner edit made mid-test reaches the FC and changes what it flies:")

def _chk(name, cond, detail=""):
    global ok
    ok = ok and bool(cond)
    print("  [%s]     %s   %s" % ("PASS" if cond else "FAIL", name, detail))

class _Port:
    in_waiting = 0
    def __init__(self): self.buf = bytearray()
    def write(self, b): self.buf += b; return len(b)
    def read(self, n): return b""

def _frames(chunk):
    """Split a tick's bytes into decoded (msgid, fields) frames."""
    out, p = [], 0
    while p + _P.HEADER_SIZE <= len(chunk):
        _, _, ln = _st.unpack_from(_P.HEADER_FMT, chunk, p)
        end = p + _P.HEADER_SIZE + ln + _P.CRC_SIZE
        out.append(_P.unpack(bytes(chunk[p:end])))
        p = end
    return out

_port = _Port()
c.link = LuaOffloadLink(StubLink(), _port)
c._gains_pushed_sig = None                 # a fresh link has been sent nothing
_store = _Store(c)
# pitch_ki 0 and the pitch trim off isolate the proportional path: the command
# is pitch_gain x error.  The trim's default is not zero, and it adds a part of
# the command that does not scale with the gain.
_store.set('pitch_ki', 0.0)
for _t in ('pitch_trim_offset', 'pitch_trim_lift', 'pitch_trim_attitude',
           'pitch_trim_thrust'):
    _store.set(_t, 0.0)
_store.set('pitch_gain', 3.0)
_EL = math.radians(5.0)                     # oracle balloon 5 deg above, dead ahead

def _tick():
    """One control tick of the Lua path, in the order run() makes the calls."""
    start = len(_port.buf)
    c.link.set_mode(4, True, oracle=True)
    c.link.set_oracle_dir(math.cos(_EL), 0.0, -math.sin(_EL))
    c._push_gains_if_changed()
    c.link.send_guide(0.85)
    return bytes(_port.buf[start:])

_chunks = [_tick() for _ in range(10)]
_store.set('pitch_gain', 6.0)               # the operator's edit, mid-test
_edit = len(_chunks)
_chunks += [_tick() for _ in range(10)]

_gi = _P.gain_index_map(type(c._gains))['pitch_gain']
_sent = [(v['index'], round(v['value'], 4)) for m, v in _frames(_chunks[_edit])
         if m == _P.MSG_SET_GAIN]
_chk("the edit is sent on the very next tick", (_gi, 6.0) in _sent,
     "%d SET_GAIN frames, pitch_gain=%s" % (len(_sent), dict(_sent).get(_gi)))
_chk("nothing is re-sent while nothing changes",
     not any(m == _P.MSG_SET_GAIN for ch in _chunks[_edit + 1:] for m, _ in _frames(ch)))

_lua = _sh.which('lua5.4')
_chk("lua5.4 is available to run the FC script", _lua is not None)
if _lua:
    _root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
    _drv = (
        'package.path = "LUA/?.lua;" .. package.path\n'
        'require("ap_shim").install()\n'
        'local stubs = require("ap_stubs")\n'
        'dofile("LUA/balloon_chaser_fc.lua")\n'
        'local f = io.open(arg[1], "rb"); local d = f:read("a"); f:close()\n'
        'local p = 1\n'
        'while p <= #d do\n'
        '  local n = string.unpack("<I4", d, p); p = p + 4\n'
        '  stubs.feed(string.sub(d, p, p + n - 1)); p = p + n\n'
        '  stubs.advance(20); update()\n'
        '  local c = stubs.log.rate_cmds[#stubs.log.rate_cmds]\n'
        '  print(c and string.format("%.6f", c[3]) or "nan")\n'
        'end\n')
    with _tf.TemporaryDirectory() as _td:
        _bin, _src = os.path.join(_td, 'ticks.bin'), os.path.join(_td, 'drive.lua')
        with open(_bin, 'wb') as fh:
            for ch in _chunks:
                fh.write(_st.pack('<I', len(ch)) + ch)
        with open(_src, 'w') as fh:
            fh.write(_drv)
        _r = _sp.run([_lua, _src, _bin], cwd=_root, capture_output=True, text=True)
    _q = [float(x) for x in _r.stdout.split()] if _r.returncode == 0 else []
    _chk("the FC script ran every tick", len(_q) == len(_chunks),
         _r.stderr.strip()[-200:] if _r.returncode else "%d ticks" % len(_q))
    if len(_q) == len(_chunks):
        _before, _after = _q[_edit - 1], _q[_edit]
        _chk("the FC's pitch command doubles on the tick the edit lands",
             _before > 1.0 and abs(_after / _before - 2.0) < 0.02,
             "gain 3 -> %.3f dps, gain 6 -> %.3f dps" % (_before, _after))

# A GAIN PUSH THAT DOES NOT LEAVE WHOLE IS SENT AGAIN.
#
# The SITL link is a non-blocking socket: under back-pressure its write drops a
# frame's tail and returns how much left.  The FC confirms nothing, so a push
# that counted a short frame as sent would leave the FC on the old gain for good.
print()
print("a gain push that does not leave whole is sent again:")
_short = {'n': 1}
_w0 = _port.write
def _short_write(b):
    if _short['n'] and b[2] == _P.MSG_SET_GAIN:
        _short['n'] -= 1
        return len(b) - 1               # nothing recorded: the frame never arrived
    return _w0(b)
_port.write = _short_write
_gi5 = _P.gain_index_map(type(c._gains))['pitch_gain']
_store.set('pitch_gain', 5.0)
_s1 = [v for m, v in _frames(_tick()) if m == _P.MSG_SET_GAIN]
_s2 = {v['index']: round(v['value'], 4) for m, v in _frames(_tick())
       if m == _P.MSG_SET_GAIN}
_port.write = _w0
_chk("the push with a short frame is not recorded as sent",
     not _short['n'] and len(_s1) > 0)
_chk("and the whole set goes again on the next tick", _s2.get(_gi5) == 5.0,
     "pitch_gain=%s, %d frames" % (_s2.get(_gi5), len(_s2)))

# A MEASUREMENT'S PRIVATE GAINS REACH THE FC; THE OPERATOR'S EDITS WAIT.
#
# begin_measurement_gains installs a private set the law must fly.  A push that
# skips while a measurement owns the gains leaves the FC on the operator's set
# for the whole run while the log records the measurement's.
print()
print("a measurement's private gains reach the FC; the operator's edits wait:")
_ai = _P.gain_index_map(type(c._gains))['airspeed_cmd_mps']
_pi = _P.gain_index_map(type(c._gains))['pitch_gain']
def _gains_sent(ch):
    return {v['index']: round(v['value'], 4) for m, v in _frames(ch)
            if m == _P.MSG_SET_GAIN}
_op_asp = round(float(c._gains.airspeed_cmd_mps), 4)
c.begin_measurement_gains('test', airspeed_cmd_mps=17.0)
_m1 = _gains_sent(_tick())
_chk("the private set is sent on the next tick", _m1.get(_ai) == 17.0,
     "airspeed_cmd_mps=%s" % _m1.get(_ai))
c.update_measurement_gains(airspeed_cmd_mps=19.0)
_m2 = _gains_sent(_tick())
_chk("a change to the private set is sent on the next tick", _m2.get(_ai) == 19.0,
     "airspeed_cmd_mps=%s" % _m2.get(_ai))
_store.set('pitch_gain', 4.0)               # the operator edits mid-measurement
_m3 = _gains_sent(_tick())
_chk("an operator edit is not sent while the measurement owns the gains",
     not _m3, "%d SET_GAIN frames" % len(_m3))
c.end_measurement_gains()
_m4 = _gains_sent(_tick())
_chk("giving the gains back sends the operator's set, edit included",
     _m4.get(_pi) == 4.0 and _m4.get(_ai) == _op_asp,
     "pitch_gain=%s airspeed_cmd_mps=%s" % (_m4.get(_pi), _m4.get(_ai)))
_raised = False
try:
    c.update_measurement_gains(airspeed_cmd_mps=15.0)
except RuntimeError:
    _raised = True
_chk("changing a private set nobody owns is refused", _raised)

# The FC-shell test builds its GUIDE frames by hand; a layout that drifts from
# protocol.py decodes as silently wrong settings rather than failing.
_lua_fmt = {'H': 'I2', 'h': 'i2', 'B': 'B', 'b': 'b', 'I': 'I4', 'i': 'i4', 'f': 'f'}
_want = '<' + ''.join(_lua_fmt[f.fmt] for f in _P.GUIDE)
_shell = open(os.path.join(os.path.dirname(__file__), '..', '..', 'LUA',
                           'test_balloon_chaser_fc.lua')).read()
_have = set(_re.findall(r"string\.pack\('(<I2BBi2[^']*)'", _shell))
_chk("the FC-shell test's GUIDE frames match protocol.py", _have == {_want},
     "want %s, have %s" % (_want, sorted(_have)))

# MODE KEYS: THE SAME KEY AGAIN HANDS BACK; ANOTHER GOES STRAIGHT THERE.
#
# From a run in the 'autotune' regime (the sweep, plant-ID, the trim map) a
# different mode key stops the run - checkpoint kept - and its teardown hands
# the aircraft over in ACRO (sticks centred, ACRO confirmed, sticks released);
# the control loop presses the key again once the run's thread has ended.  A
# mode started before then is a second driver beside the teardown.  Threads
# are recorded, not started, and run here one at a time; _service_switch() is
# the control loop's tick.
print()
print("mode keys: the same key again hands back, another goes straight there:")
import logging as _lg, os as _os, tempfile as _tf, threading as _th
import Companion.controller as _CM, Companion.trim_map as _TMm
_threads, _said, _calls = [], [], []
class _FakeThread:
    def __init__(self, target=None, args=(), daemon=None, name=None):
        self.target, self.args, self.name, self.alive = target, args, name, True
    def start(self): _threads.append(self)
    def join(self, timeout=None): pass
    def is_alive(self): return self.alive
    def run_now(self):
        try:
            self.target(*self.args)
        finally:
            self.alive = False
def _named(n): return [t for t in _threads if t.name == n]
class _Said(_lg.Handler):
    def emit(self, r): _said.append(r.getMessage())
def _fake_run(ctrl, abort, *a):
    """A run that has seen its abort.  Its teardown calls autotune_finish
    twice, as the sweep's abort branch and finally do."""
    ctrl.autotune_finish()
    ctrl.autotune_finish()
def _acro(hold=None):
    _calls.append('ACRO')
    if hold is not None:
        hold()
_h = _Said(); _plog = _lg.getLogger("balloon_chaser")
_plog_level = _plog.level
_plog.addHandler(_h); _plog.setLevel(_lg.INFO)
_saved = (_th.Thread, _CM.run_controller_autotune_sweep, _TMm.run_trim_map)
_th.Thread = _FakeThread
_CM.run_controller_autotune_sweep = _fake_run
_TMm.run_trim_map = _fake_run
c.link.set_mode_loiter = lambda **k: _calls.append('LOITER')
c.link.set_mode_acro = _acro
c.link.hold_level_sticks = lambda thr: _calls.append('centre')
c.link.release_rc_overrides = lambda: _calls.append('release')
_active_calls = []
c.sensor.set_active = lambda *a, **k: _active_calls.append(a)
def _at_rest():
    c._mode, c._tracking_transition, c.flight_complete = 'loiter', False, True
    c._run_key = c._run_abort = c._run_thread = c._switch = None
    for _l in (_threads, _said, _calls, _active_calls):
        del _l[:]
def _start(key):
    """Start run `key` from rest; returns its (unstarted) thread."""
    _at_rest()
    {'t': c.toggle_autotune, 'e': c.toggle_trim_map}[key]()
    c._tracking_transition = False           # the run clears it once underway
    del _calls[:]
    return _threads[-1]
_cwd = _os.getcwd()
_td = _tf.TemporaryDirectory()
_os.chdir(_td.name)
try:
    _at_rest()
    c.toggle_ptest()
    _chk("from rest, 'p' starts its test", _named("point-balloon-chaser-test") and c._mode == 'ptest')
    c._tracking_transition = False
    c.toggle_ptest()
    _chk("'p' again hands back to LOITER", _named("go-loiter") and c._mode == 'loiter')

    _run = _start('e')
    _chk("from rest, 'e' starts the trim map", _run.name == "trim-map" and c._run_key == 'e')
    c.toggle_ptest()
    _chk("'p' during the trim map stops it and waits: nothing new yet",
         c._run_abort.is_set() and c._switch is not None
         and not _named("point-balloon-chaser-test") and c._mode == 'autotune')
    _chk("the switch key is in the flight log's events",
         c._pending_event == 'KEY_P_SWITCH_FROM_E', c._pending_event)
    c.toggle_balloon_chaser()
    _chk("another key while the switch waits is refused",
         any(m.startswith("Switching to the point-balloon-chaser test") for m in _said)
         and not _named("tracking-start"))
    c._service_switch()
    _chk("the control loop waits while the run's thread is up",
         not _named("point-balloon-chaser-test") and c._switch is not None)
    _run.run_now()
    _chk("the teardown hands over in ACRO: sticks centred, ACRO, then released",
         _calls[:3] == ['centre', 'ACRO', 'centre'] and _calls[-1] == 'release'
         and 'LOITER' not in _calls and c.tracking is False, str(_calls))
    _chk("a second teardown call leaves the hand-over alone", 'LOITER' not in _calls)
    c._service_switch()
    _chk("then the control loop starts 'p', straight from the run",
         _named("point-balloon-chaser-test") and c._mode == 'ptest' and c._switch is None
         and 'LOITER' not in _calls)

    _run = _start('e')
    c.toggle_trim_map()
    _run.run_now()
    _chk("'e' again stops the trim map and hands back to LOITER",
         c._switch is None and 'LOITER' in _calls and 'ACRO' not in _calls
         and c._run_key is None)

    _run = _start('t')
    c.handle_arrow('up')
    _run.run_now(); c._service_switch()
    _chk("an arrow during the sweep switches straight into manual, nudged up",
         c._mode == 'manual' and _named("tracking-start") and 'LOITER' not in _calls
         and c._manual_desired_pitch > c.link.attitude[1])

    _run = _start('e')
    c.toggle_autotune()
    _run.run_now(); c._service_switch()
    _chk("'t' during the trim map starts the sweep straight after it",
         _named("controller-autotune") and c._run_key == 't' and 'LOITER' not in _calls)
    c._tracking_transition = False
    c.toggle_cell_autotune()
    _chk("'y' during 't' is a switch, not a stop",
         c._switch is not None and c._run_abort.is_set())

    _start('e')
    c.toggle_vision_tracking()
    _chk("'l' during a run is refused, the run's sensor untouched",
         any(m.startswith("The trim map is flying") for m in _said)
         and not [a for a in _active_calls if a and a[0]])

    _at_rest()
    _run = _FakeThread(name="plant-id")
    c._run_key, c._run_abort, c._run_thread = 'i', _th.Event(), _run
    c.toggle_ptest()                          # between plant-ID stages, in LOITER
    _chk("between plant-ID stages 'p' waits for plant-ID to end",
         c._switch is not None and not _named("point-balloon-chaser-test"))
    _run.alive, c._run_key = False, None      # plant-ID ends without a teardown
    c._service_switch()
    _chk("then starts from LOITER", _named("point-balloon-chaser-test") and 'ACRO' not in _calls)

    _run = _start('e')
    c.toggle_ptest()
    c._switch['t0'] -= _CM._SWITCH_WAIT_S + 1
    c._service_switch()
    _chk("a run that does not stop in time: the switch is dropped, nothing starts",
         c._switch is None and not _named("point-balloon-chaser-test"))
    _run.run_now()
    _chk("and its teardown hands back to LOITER", 'LOITER' in _calls and 'ACRO' not in _calls)

    _run = _start('e')
    c.toggle_ptest()
    _sw = c._switch
    c.autotune_finish()                       # handed over; the thread still runs
    _sw['t0'] -= _CM._SWITCH_WAIT_S + 1
    c._service_switch()
    _gl = _named("go-loiter")
    _chk("handed over but the run's thread outlives the wait: handed back",
         _gl and c._switch is None and not _named("point-balloon-chaser-test"))
    _gl[0].run_now()
    _chk("  to LOITER", 'LOITER' in _calls)

    _run = _start('e')
    c.toggle_ptest()
    def _refused(hold=None):
        _calls.append('ACRO refused')
        raise TimeoutError("refused")
    c.link.set_mode_acro = _refused
    _run.run_now()
    c.link.set_mode_acro = _acro
    c._service_switch()
    _chk("ACRO refused at the hand-over: LOITER, and the switch is cancelled",
         'LOITER' in _calls and not _named("point-balloon-chaser-test") and c._switch is None)

    _run = _start('e')
    c.toggle_balloon_chaser()
    _run.run_now()
    c.flight_complete = False                 # 'x' will refuse
    c._service_switch()
    _gl = _named("go-loiter")
    _chk("a mode that does not start after the switch is handed back",
         _gl and not _named("tracking-start"))
    _gl[0].run_now()
    _chk("  to LOITER", 'LOITER' in _calls and c._mode == 'loiter')

    _run = _start('e')
    c.toggle_ptest()
    c._cancel_switch()                        # Ctrl+C: run()'s finally
    _run.run_now(); c._service_switch()
    _chk("shutdown cancels a pending switch: the teardown hands back, nothing starts",
         'LOITER' in _calls and 'ACRO' not in _calls and not _named("point-balloon-chaser-test"))

    _at_rest()
    c._mode = 'straight'
    c.link.set_mode_acro = _refused
    c._begin_tracking()
    c.link.set_mode_acro = _acro
    _chk("a failed ACRO entry for s, x or an arrow hands back to LOITER",
         'LOITER' in _calls and c._mode == 'loiter' and not c._tracking_transition)

    _at_rest()
    for _ax in ('plantid_do_pitch', 'plantid_do_yaw', 'plantid_do_roll'):
        setattr(c, _ax, 0)
    c.toggle_plant_id()
    _pi = _named("plant-id")[0]
    _pi.run_now()
    _chk("plant-ID with nothing to fly still hands back",
         'LOITER' in _calls and c._mode == 'loiter' and not c._tracking_transition
         and c._run_key is None)
    for _ax in ('plantid_do_pitch', 'plantid_do_yaw', 'plantid_do_roll'):
        setattr(c, _ax, 1)
finally:
    _os.chdir(_cwd)
    _td.cleanup()
    _th.Thread, _CM.run_controller_autotune_sweep, _TMm.run_trim_map = _saved
    for _n in ('set_mode_loiter', 'set_mode_acro', 'hold_level_sticks',
               'release_rc_overrides'):
        c.link.__dict__.pop(_n, None)
    c.sensor.__dict__.pop('set_active', None)
    _plog.removeHandler(_h); _plog.setLevel(_plog_level)

# PLANT-ID'S UNDERSPEED CHECK READS THE FC'S AIRSPEED_MIN.
#
# Read from an attribute nothing set, it checked every axis against a typed-in
# fallback and logged that as the FC's AIRSPEED_MIN.
print()
print("plant-ID's underspeed check uses the FC's AIRSPEED_MIN:")
from Companion import plant_id as _PID
class _AspLink:
    def __init__(self, asp, amin): self.airspeed, self._amin = asp, amin
    def read_param(self, name): return self._amin if name == 'AIRSPEED_MIN' else None
def _clear(asp, amin):
    _l = _AspLink(asp, amin)
    return _PID._airspeed_clear(_l, _PID._airspeed_min(_l))
_chk("an airspeed above the FC's AIRSPEED_MIN is clear", _clear(11.0, 9.0))
_chk("one below it is refused", not _clear(13.0, 15.0))
_chk("AIRSPEED_MIN 0 is floored as ArduPlane floors it",
     _PID._airspeed_min(_AspLink(0.0, 0.0)) == _PID.MIN_AIRSPEED_MIN_MPS
     and not _clear(4.0, 0.0))
_chk("an unread AIRSPEED_MIN refuses rather than guesses", not _clear(30.0, None))

# THE p TEST TIMES CONVERGENCE FROM THE INJECTION WHEN THE FC NEVER ANSWERS.
#
# The clock restarts at the FC's first commanded tick.  When that tick never
# comes inside the warm-up, restarting it anyway would leave the wait out of
# the time the test reports.  A fake clock runs the test at once.
print()
print("the p test times convergence from the injection when the FC never answers:")
import logging as _lg2, threading as _thr2
from Companion.testing import point_balloon_chaser as _PP
class _Clock:
    t = 100.0
    def monotonic(self): return self.t
    def sleep(self, d): self.t += d
_clk = _Clock()
class _Inj:
    def __init__(self, *a, **k): self.lon, self.lat = 0.0, 0.0
    def start(self): return True
class _PCtl:
    class link:
        attitude, rates, airspeed = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 18.0
    sensor = None
    point_lon_deg = point_lat_deg = 0.0
    t_begin = None
    def set_point_injector(self, inj): pass
    def ptest_enter_acro(self): return True
    def ptest_begin(self, lon, lat): self.t_begin = _clk.t
    def ptest_finish(self): pass
    def cascade_has_run_since_inject(self): return False      # never answers
    def alignment_errors(self):                               # aligned 4.4 s in
        return (0.0, 0.0) if _clk.t - self.t_begin >= 4.4 else (10.0, 0.0)
    @property
    def mode(self): return 'ptest' if _clk.t - (self.t_begin or _clk.t) < 20.0 else 'loiter'
class _Grab(_lg2.Handler):
    def __init__(self): super().__init__(); self.lines = []
    def emit(self, r): self.lines.append(r.getMessage())
_grab = _Grab()
_plog2 = _lg2.getLogger("balloon_chaser"); _plog2.addHandler(_grab)
_lvl2 = _plog2.level; _plog2.setLevel(_lg2.INFO)
_saved_pp = (_PP.time, _PP.SyntheticBalloonInjector)
_PP.time, _PP.SyntheticBalloonInjector = _clk, _Inj
try:
    _PP.run_point_balloon_chaser_test(_PCtl(), _thr2.Event())
finally:
    _PP.time, _PP.SyntheticBalloonInjector = _saved_pp
    _plog2.removeHandler(_grab); _plog2.setLevel(_lvl2)
_chk("a balloon first held 4.4 s after injection is aligned at 4.4 s, no deadline",
     any(l.endswith("ALIGNED") and not l.endswith("NOT ALIGNED") for l in _grab.lines)
     and any("Aligned at:  4.4" in l for l in _grab.lines),
     [l for l in _grab.lines if "Aligned at" in l or l.endswith("ALIGNED")])

sys.exit(0 if ok else 1)
print("\n"+"="*70)
print("PASSED" if ok else "FAILED")
print("="*70)

