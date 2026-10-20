import dataclasses
import re
# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
"""LuaOffloadLink must accept every call MavlinkAutopilotLink accepts.

It is a DECORATOR.  __getattr__ forwards everything it does not define, so an
attribute drift is caught immediately - but an OVERRIDDEN method with a narrower
signature is silent until the moment it is called, which is after arming, in
ACRO, on the first tracking tick.  Swallowing the extras as **kwargs is worse:
the call succeeds and the arguments are dropped on the floor.
"""
import ast, inspect, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from Companion import protocol as P
from Companion.adapters.lua_offload import LuaOffloadLink
from Companion.adapters import lua_offload as _LO_mod

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
fails = []
def check(n, c, d=""):
    print(f"  [{'PASS' if c else 'FAIL'}] {n}" + (f"   {d}" if d else ""))
    if not c: fails.append(n)

print("\n" + "=" * 74); print("LuaOffloadLink <-> MavlinkAutopilotLink SIGNATURE CONTRACT")
print("=" * 74)

# Parse the real link rather than import it: pymavlink may not be installed on
# a machine that only runs the verification suite.
src = open(os.path.join(PKG, 'adapters', 'mavlink_link.py')).read()
real = {m.name: m for m in ast.walk(ast.parse(src))
        if isinstance(m, ast.FunctionDef)}

overridden = [n for n, v in vars(LuaOffloadLink).items()
              if callable(v) and not n.startswith('__') and n in real]
print(f"  {len(overridden)} method(s) overridden: {', '.join(sorted(overridden))}\n")

for name in sorted(overridden):
    r = real[name]
    r_args = [a.arg for a in r.args.args if a.arg != 'self']
    r_kwonly = [a.arg for a in r.args.kwonlyargs]
    sig = inspect.signature(getattr(LuaOffloadLink, name))
    mine = [p for p in sig.parameters if p != 'self']
    has_varkw = any(p.kind is inspect.Parameter.VAR_KEYWORD
                    for p in sig.parameters.values())
    missing = [a for a in (r_args + r_kwonly) if a not in mine]
    check(f"{name}: accepts every real parameter", not missing,
          f"missing {missing}" if missing else f"{len(mine)} params")
    # **kwargs would make the check pass while silently discarding arguments
    check(f"{name}: does not hide drift behind **kwargs", not has_varkw)


# ---------------------------------------------------------------------------
print("\n  the link's setters must actually be CALLED")
# Writing set_detection/set_mode/set_manual_setpoints and never calling them
# meant every GUIDE frame carried det_valid=false, mode=0 and zeroed setpoints.
# The link was healthy, the frames arrived, and the FC still had nothing to fly.
ctl = open(os.path.join(PKG, 'controller.py')).read()
for setter in ('set_detection', 'set_mode', 'set_manual_setpoints', 'set_gimbal'):
    check(f"controller.py calls {setter}", f'.{setter}(' in ctl or
          f'"{setter}"' in ctl,
          "an uncalled setter leaves its field at the default, silently")

# GUIDE goes out EVERY tick.  It is the FC's evidence that the companion is
# alive - a send guarded by `if self.tracking:` would trip the FC's staleness
# release whenever tracking is off - and with the frame always flowing its
# tracking flag is the live instruction.
_run = next(n for n in ast.walk(ast.parse(ctl))
            if isinstance(n, ast.FunctionDef) and n.name == 'run')
_guarded, _sends = [], []
def _walk(node, conds):
    for ch in ast.iter_child_nodes(node):
        if (isinstance(ch, ast.Call) and isinstance(ch.func, ast.Attribute)
                and ch.func.attr == 'send_guide'):
            _sends.append(ch)
            if any('tracking' in c for c in conds):
                _guarded.append(ch.lineno)
        _walk(ch, conds + ([ast.unparse(ch.test)] if isinstance(ch, ast.If) else []))
_walk(_run, [])
check("run() sends GUIDE every tick, not only while tracking",
      bool(_sends) and not _guarded,
      ("guarded by tracking at line(s) %s" % _guarded) if _guarded else
      ("%d call(s)" % len(_sends) if _sends else "send_guide is never called"))

print("\n  the FC script must handle every flight mode")
fc2 = open(os.path.join(os.path.dirname(PKG), 'LUA', 'balloon_chaser_fc.lua')).read()
check("straight-and-level synthesises a heading target",
      'straight_yaw' in fc2 and 'yaw_pitch_to_world_vec' in fc2,
      "_update_control does not use the estimator in this mode")
check("manual synthesises from yaw+pitch", 'manual_pitch' in fc2)
check("only balloon chaser requires an estimator fix",
      fc2.count('drop_control(GATE.EST') >= 1 and 'est.valid' in fc2)

# ---------------------------------------------------------------------------
# Every STATE field the FC sends must be READ on this side.
#
# The silent half of the four-step rule.  A field can be computed on the FC,
# packed into the frame, given a CSV column and a HUD label, and still read a
# flat default forever because _apply_state never assigned it.  Nothing fails
# and nothing warns.  A loop over P.STATE that decodes every float field counts
# for all of them.
print()
print("every frame to the FC goes out whole, through one locked writer:")
# GUIDE from the control loop, the gain push from the tracking start's thread,
# single gains from the tuner and the sweep: two unserialised writes that
# overlap interleave their bytes, both frames fail the FC's CRC, and a lost
# SET_GAIN is never re-sent.
_lo_src = inspect.getsource(_LO_mod)
_writes = [m.start() for m in re.finditer(r'_port\.write\(', _lo_src)]
_send_src = inspect.getsource(LuaOffloadLink._send)
check("    _port.write is called in exactly one place, _send, under _tx_lock",
      len(_writes) == 1 and '_port.write(' in _send_src
      and 'with self._tx_lock' in _send_src, "%d call sites" % len(_writes))
print()
print("plant-ID measures from the FC's own tick, paired STATE+STATE2:")
# The gyro, the servo signals and the FC clock of one tick, stored as one tuple
# only when STATE2 follows its own STATE: the one just before it, same seq.  Plant-ID reads that tuple and
# nothing the simulator or MAVLink supplies, so the same run works on hardware.
class _NoLink:
    def __getattr__(self, n): raise AttributeError(n)
_LL = LuaOffloadLink(_NoLink(), _NoLink())
def _state(seq, ms):
    v = {f.name: 0 for f in P.STATE}; v.update(seq=seq, fc_millis=ms, gate=5)
    return P.unpack(P.pack(P.MSG_STATE, v))[1]
def _state2(seq, gy):
    v = {f.name: 0 for f in P.STATE2}
    v.update(seq=seq, gyro_x_rads=0.1, gyro_y_rads=gy, gyro_z_rads=0.3,
             pwm_aileron=1600, pwm_vtail_l=1575, pwm_vtail_r=1525)
    return P.unpack(P.pack(P.MSG_STATE2, v))[1]
_LL._apply_state(_state(7, 5000)); _LL._apply_state2(_state2(7, -0.2))
_s1 = _LL.lua_tick_sample
_LL._apply_state(_state(8, 5020)); _LL._apply_state2(_state2(9, -0.5))
check("    a paired tick is one tuple (FC ms, gyro x/y/z, aileron, V-tail L/R)",
      _s1 is not None and _s1[0] == 5000 and abs(_s1[2] + 0.2) < 1e-6
      and tuple(_s1[4:]) == (1600, 1575, 1525), str(_s1))
check("    frames from two different ticks are never paired",
      _LL.lua_tick_sample == _s1, str(_LL.lua_tick_sample))
# Two FC ticks can echo one GUIDE seq.  Tick A's STATE2 lost, tick B's STATE
# lost: A's STATE and B's STATE2 share a seq but not a tick.
_LL._apply_state(_state(10, 6000)); _LL._apply_state2(_state2(10, -0.7))
_s2 = _LL.lua_tick_sample
_LL._apply_state2(_state2(10, -0.9))
check("    a STATE2 pairs only with the STATE just before it, never one used",
      _LL.lua_tick_sample == _s2 and _s2[0] == 6000, str(_LL.lua_tick_sample))
_pid_src = open(os.path.join(PKG, 'plant_id.py')).read()
_simlike = [w for w in ('sim_rates', 'attitude_time_boot_s', 'servo_elevator',
                        'servo_ch4', 'servo_aileron') if w in _pid_src]
check("    plant-ID reads no SIMSTATE rate, MAVLink servo or MAVLink clock",
      not _simlike and 'lua_tick_sample' in _pid_src, str(_simlike))

print()
print("every STATE field is decoded by LuaOffloadLink:")
_src = inspect.getsource(LuaOffloadLink._apply_state)
_dec = set(re.findall(r'v\["([a-z0-9_]+)"\]', _src))
_dec |= set(re.findall(r'v\.get\("([a-z0-9_]+)"', _src))
for _m in re.finditer(r'for _n in \(([^)]*)\):', _src):
    _dec |= set(re.findall(r'"([a-z0-9_]+)"', _m.group(1)))
if re.search(r'for _f in P\.STATE:\s*\n\s*if _f\.fmt == "f":', _src):
    _dec |= {f.name for f in P.STATE if f.fmt == "f"}
_wire = [f.name for f in P.STATE if not f.name.startswith("_pad")]
_missing = [f for f in _wire if f not in _dec]
check("    no STATE field is silently dropped", not _missing,
      "%s - these would read 0.0 forever" % _missing)



# ---------------------------------------------------------------------------
# Every self.link.<method>(...) call in the controller must match the
# LuaOffloadLink signature it will hit.
#
# A call and its signature can drift apart - a parameter dropped from the CALL
# but kept in the SIGNATURE, or the reverse - and Python only notices at call
# time, inside run(), which no test drives, so the crash lands in main-loop
# startup well past anything a unit test exercises.  This contract is the only
# guard.
print()
print("controller -> LuaOffloadLink call signatures agree:")
_src = open(os.path.join(os.path.dirname(__file__), "..", "controller.py")).read()
_bad = []
for _n in ast.walk(ast.parse(_src)):
    if not (isinstance(_n, ast.Call)
            and isinstance(_n.func, ast.Attribute)
            and isinstance(_n.func.value, ast.Attribute)
            and _n.func.value.attr == "link"):
        continue
    _m = getattr(LuaOffloadLink, _n.func.attr, None)
    if _m is None or not callable(_m):
        continue
    _sig = inspect.signature(_m)
    _pos = [q for q, v in _sig.parameters.items()
            if q != "self" and v.kind in (v.POSITIONAL_ONLY, v.POSITIONAL_OR_KEYWORD)]
    _req = [q for q in _pos if _sig.parameters[q].default is inspect.Parameter.empty]
    _given = len(_n.args) + len([k for k in _n.keywords if k.arg])
    if _given < len(_req) or (len(_n.args) > len(_pos)
                              and not any(v.kind == v.VAR_POSITIONAL
                                          for v in _sig.parameters.values())):
        _bad.append("%s: passes %d, needs %d" % (_n.func.attr, _given, len(_req)))
check("    every link call matches its signature", not _bad,
      "%s - TypeError at run() time, not at import" % _bad)

# ---------------------------------------------------------------------------
# ATTRIBUTE-STYLE ACCESSORS MUST BE PROPERTIES.
#
# Every `self._gains.<name>` read without parentheses either hits a dataclass
# FIELD or a @property.  A plain method there returns the function object, and
# the failure surfaces one frame later somewhere else entirely - the real one
# read "'function' object has no attribute 'ki'" from inside the pitch
# integrator, three call levels below the missing decorator.
#
# It goes missing when the method ABOVE it is deleted and the regex takes the
# decorator with it.  That has now happened once and would not have been caught
# by anything: the accessor is only read inside _update_control, which no test
# drives.
print()
print("ControlGains attribute reads resolve to fields or properties:")
_gsrc = open(os.path.join(os.path.dirname(__file__), "..", "control_laws.py")).read()
_cls = [n for n in ast.walk(ast.parse(_gsrc))
        if isinstance(n, ast.ClassDef) and n.name == "ControlGains"][0]
_fields = {n.target.id for n in _cls.body if isinstance(n, ast.AnnAssign)}
_props = {n.name for n in _cls.body if isinstance(n, ast.FunctionDef)
          and any(getattr(d, "id", "") == "property" for d in n.decorator_list)}
_plain = {n.name for n in _cls.body if isinstance(n, ast.FunctionDef)} - _props
_csrc = open(os.path.join(os.path.dirname(__file__), "..", "controller.py")).read()
_read = set(re.findall(r"_gains\.([a-z_][a-z0-9_]*)(?!\s*\()", _csrc))
_broken = sorted(_read & _plain)
check("    no attribute read hits a plain method", not _broken,
      "%s - returns the function object, fails later and elsewhere" % _broken)

# ---------------------------------------------------------------------------
# THE PITCH GAIN SET RESOLVES.
#
# The CSV snapshot reads self._gains.pitch_gains every tick, so a broken
# accessor fails on the first logged row, far from its cause.  The law itself
# runs on the FC and is exercised by LUA/test_balloon_chaser_fc.lua.
print()
print("ControlGains.pitch_gains resolves on default gains:")
_lawerr = ""
try:
    from Companion.control_laws import ControlGains as _CG
    _f = {q.name: q for q in dataclasses.fields(_CG)}
    _g = _CG(**{n: (q.default if q.default is not dataclasses.MISSING else 0.0)
                for n, q in _f.items()})
    assert hasattr(_g.pitch_gains, "ki"), "pitch_gains did not resolve to a gain set"
except Exception as _e:
    _lawerr = "%s: %s" % (type(_e).__name__, _e)
check("    pitch_gains resolves", not _lawerr, _lawerr)

print()
print("every st_msg field the FC writes is carried by the STATE schema:")
# A field assigned into st_msg but absent from STATE is packed NOWHERE, and the
# companion's `v.get(name, 0.0)` then records a confident zero forever: the
# column exists, the HUD reads it, and the number is never written. That is
# indistinguishable from a real measurement of zero, so it survives exactly as
# long as nobody needs the value - and lat_marker_deg survived a full flight
# campaign that way while the yaw diagnosis was argued over its zero.
#
# Assignment order in balloon_chaser_fc.lua is irrelevant (the packer reads by name),
# so this only checks membership, not position.
_fc = open(os.path.join(os.path.dirname(PKG), 'LUA', 'balloon_chaser_fc.lua')).read()
_written = set(re.findall(r'\bst_msg\.([A-Za-z_][A-Za-z_0-9]*)\s*=', _fc))
_carried = {f.name for f in P.STATE}
_orphans = sorted(_written - _carried)
check("    no orphaned st_msg fields", not _orphans,
      ("packed nowhere: " + ", ".join(_orphans)) if _orphans else
      f"{len(_written)} fields, all carried")
_written2 = set(re.findall(r'\bst2\.([A-Za-z_][A-Za-z_0-9]*)', _fc))
_orphans2 = sorted(_written2 - {f.name for f in P.STATE2})
check("    no orphaned st2 fields", not _orphans2,
      ("packed nowhere: " + ", ".join(_orphans2)) if _orphans2 else
      f"{len(_written2)} fields, all carried")

print()
print("every STATE2 field reaches a lua_* attribute, NaN as None:")
# Decoded by packing a frame with a distinct value in every field and reading
# the attributes back: a field the decoder skips keeps its None.
class _NoInner:
    pass
class _NoPort:
    in_waiting = 0
    def write(self, b): return len(b)
    def read(self, n): return b""
_L2 = LuaOffloadLink(_NoInner(), _NoPort())
_v2 = {f.name: (float(i) + 0.5 if f.fmt == "f" else i + 1)
       for i, f in enumerate(P.STATE2)}
_v2["flags"] = 0x05
_v2["eas_mps"] = float("nan")
_L2._rx += P.pack(P.MSG_STATE2, _v2)
_L2._drain()
_miss2 = [f.name for f in P.STATE2
          if f.name not in ("seq", "flags", "eas_mps")
          and getattr(_L2, "lua_" + f.name, None) != _v2[f.name]]
check("    every field decoded", not _miss2 and _L2.lua_state2_seq == _v2["seq"],
      "not decoded: %s" % _miss2)
check("    flags decoded, NaN read as None",
      _L2.lua_coord_valid is True and _L2.lua_accel_valid is False
      and _L2.lua_lat_marker_valid is True and _L2.lua_eas_mps is None)

print()
print("a whole frame from another build is stepped over, not stuck on:")
# Checksum good, size or id unknown to this build: the FC runs a different
# balloon_chaser_fc.lua.  Decoding it raises struct.error / KeyError, not ValueError.
import struct as _st
def _foreign(msgid, n):
    body = bytes(n)
    hdr = _st.pack(P.HEADER_FMT, P.MAGIC, msgid, n)
    return hdr + body + _st.pack("<H", P.crc16(hdr + body))
_L3 = LuaOffloadLink(_NoInner(), _NoPort())
_L3._rx += _foreign(P.MSG_STATE2, P.payload_size(P.STATE2) + 8)
_L3._rx += _foreign(0x7E, 4)
_L3._rx += P.pack(P.MSG_STATE2, _v2)
_L3._drain()
check("    the good frame behind them decodes",
      _L3.lua_state2_seq == _v2["seq"], "seq %r" % _L3.lua_state2_seq)
check("    both are counted bad and the buffer is consumed",
      _L3.lua_frames_bad >= 2 and len(_L3._rx) < P.HEADER_SIZE + P.CRC_SIZE,
      "bad %d, %d bytes left" % (_L3.lua_frames_bad, len(_L3._rx)))

check("    and the link reports a fault, for the control loop to halt on",
      bool(_L3.link_fault()) and "different version" in (_L3.link_fault() or ""),
      repr(_L3.link_fault()))

print()
print("the control loop halts on an untrustworthy FC link:")
# A healthy link reports nothing; a silent FC, a foreign frame (above) or the
# FC's own rejection flag each report why, and run() stops on that answer.
from Companion.constants import FC_LINK_TIMEOUT_S as _FC_TIMEOUT
import time as _time
_L4 = LuaOffloadLink(_NoInner(), _NoPort())
_v4 = {f.name: 0 for f in P.STATE}
_L4._rx += P.pack(P.MSG_STATE, _v4)
_L4._drain()
check("    a fresh link reports no fault", _L4.link_fault() is None,
      repr(_L4.link_fault()))
_L4.lua_last_rx_mono = _time.monotonic() - (_FC_TIMEOUT + 0.5)
check("    no STATE for FC_LINK_TIMEOUT_S is a fault",
      "No report from the flight controller" in (_L4.link_fault() or ""),
      repr(_L4.link_fault()))
def _sent_one(link):
    """As if this session had sent its first GUIDE (seq 1)."""
    link._seq, link._guide_sent_count = 1, 1
_L5 = LuaOffloadLink(_NoInner(), _NoPort())
_sent_one(_L5)
_L5._rx += P.pack(P.MSG_STATE, dict(_v4, seq=1, flags=0x10))
_L5._drain()
check("    the FC's rejection flag (STATE bit 4) is a fault",
      "rejected a message" in (_L5.link_fault() or ""), repr(_L5.link_fault()))
for _code, _name in P.FAULT_NAMES.items():
    _L6 = LuaOffloadLink(_NoInner(), _NoPort())
    _sent_one(_L6)
    _L6._rx += P.pack(P.MSG_STATE, dict(_v4, seq=1, flags=_code << 5))
    _L6._drain()
    check("    FC fault %d in STATE bits 5-7 is a fault naming it" % _code,
          _name in (_L6.link_fault() or ""), repr(_L6.link_fault()))
print("every controller mode has a GUIDE mode code:")
# run() looks the mode up directly, so a mode string missing from MODE_CODE
# stops the loop; this finds one before a flight does.
import re as _re_mode
from Companion.controller import MODE_CODE as _MODE_CODE
_modes_set = set()
for _dp, _dn, _fn in os.walk(PKG):
    if os.sep + 'tests' in _dp or '__pycache__' in _dp:
        continue
    for _f in _fn:
        if _f.endswith('.py'):
            _modes_set |= set(_re_mode.findall(r"\._mode = '([a-z_]+)'",
                                               open(os.path.join(_dp, _f)).read()))
check("    every mode the code sets is in MODE_CODE",
      _modes_set and _modes_set <= set(_MODE_CODE),
      "missing: %s" % sorted(_modes_set - set(_MODE_CODE)))
check("    run() looks the mode up directly, never with a fallback",
      "MODE_CODE[self._mode]" in ctl and "MODE_CODE.get(" not in ctl)
# The script outlives the program: a flag latched at the end of the previous
# session arrives in the first STATE frames of the next, which echo that
# session's seq, not one this one sent.
_L7 = LuaOffloadLink(_NoInner(), _NoPort())
_L7._rx += P.pack(P.MSG_STATE, dict(_v4, seq=4321, flags=0x10 | (3 << 5)))
_L7._drain()
check("    a flag latched in a previous session is not this session's fault",
      _L7.link_fault() is None, repr(_L7.link_fault()))
_sent_one(_L7)
_L7._rx += P.pack(P.MSG_STATE, dict(_v4, seq=4321, flags=3 << 5))
_L7._drain()
check("    nor once this session sends, on a frame that echoes another seq",
      _L7.link_fault() is None, repr(_L7.link_fault()))
_run_src = ast.get_source_segment(ctl, _run)
check("    run() asks link_fault() every tick and stops on LinkFault",
      "link_fault" in _run_src and "raise LinkFault" in _run_src
      and "_halt_on_console(halt_reason)" in _run_src)
check("    the halt comes after the hand-back to LOITER",
      _run_src.index("go_to_loiter('shutdown'") < _run_src.index("_halt_on_console(halt_reason)"))

print()
print("STATE and STATE2 fit the serial budget:")
# The budget comment beside STATE2 quotes both payload sizes, the bytes per
# tick and a percentage of the line.  They rot silently as fields are appended,
# so they are recomputed here and the test fails naming the drift.  The line
# rate is read from the two ends of the link, which must agree.
import struct as _struct
_baud_fc = re.search(r'port:begin\((\d+)\)', _fc)
_main = open(os.path.join(os.path.dirname(PKG), 'main.py')).read()
_baud_py = re.search(r'Serial\(args\.lua_path,\s*(\d+)', _main)
check("    balloon_chaser_fc.lua and main.py open the link at the same baud",
      bool(_baud_fc) and bool(_baud_py) and _baud_fc.group(1) == _baud_py.group(1),
      "%s / %s" % (_baud_fc and _baud_fc.group(1), _baud_py and _baud_py.group(1)))
_baud = int(_baud_fc.group(1)) if _baud_fc else 115200
_b1 = _struct.calcsize("<" + "".join(f.fmt for f in P.STATE))
_b2 = _struct.calcsize("<" + "".join(f.fmt for f in P.STATE2))
_tick = (_b1 + 6) + (_b2 + 6)                 # 4 header + 2 CRC each
_pct = _tick * 10 * 50 / _baud * 100          # 10 bits/byte, 50 Hz
_quoted = re.search(r'STATE (\d+) B \+ STATE2 (\d+) B payload -> (\d+) B per tick'
                    r' -> \d+ kbps -> (\d+)% of the line\s+<- here',
                    inspect.getsource(P))
check("    budget comment matches the structs",
      bool(_quoted) and (int(_quoted.group(1)), int(_quoted.group(2)),
                         int(_quoted.group(3)), int(_quoted.group(4)))
      == (_b1, _b2, _tick, round(_pct)),
      f"structs {_b1} + {_b2} B -> {_tick} B/tick -> {_pct:.0f}%; comment says "
      f"{_quoted.groups() if _quoted else '(no marker found)'}")
check("    each payload fits its one-byte length field", _b1 <= 255 and _b2 <= 255,
      f"STATE {_b1} B, STATE2 {_b2} B")
check("    both frames under 90% of the line", _pct < 90.0,
      f"{_tick} B per tick -> {_pct:.0f}% of {_baud}")

print()
print("only the autopilot's own HEARTBEAT sets the FC mode and armed state:")
# A GCS (MAVProxy), gimbal or second companion on the link also sends HEARTBEAT,
# with custom_mode 0 and the armed bit clear.  Taking theirs flips current_mode
# to 0 (MANUAL) between the FC's beats, which confirms _set_mode(MANUAL) before
# the FC has switched and reads a flying aircraft as disarmed.
# The predicate is exec'd from the real source against the MAVLink enum values,
# so this runs without pymavlink installed.
class _MavEnums:
    MAV_COMP_ID_AUTOPILOT1 = 1
    MAV_AUTOPILOT_ARDUPILOTMEGA = 3
class _Mavutil:
    mavlink = _MavEnums
_hb_fn = next(f for f in ast.parse(src).body
              if isinstance(f, ast.FunctionDef) and f.name == 'is_autopilot_heartbeat')
_ns = {'mavutil': _Mavutil}
exec(compile(ast.Module(body=[_hb_fn], type_ignores=[]), 'mavlink_link.py', 'exec'), _ns)
_is_ap_hb = _ns['is_autopilot_heartbeat']

class _Hb:
    def __init__(self, sysid, compid, autopilot):
        self._s, self._c, self.autopilot = sysid, compid, autopilot
    def get_srcSystem(self): return self._s
    def get_srcComponent(self): return self._c
MAV_AUTOPILOT_INVALID = 8
check("    the autopilot's heartbeat is taken", _is_ap_hb(_Hb(1, 1, 3), 1))
check("    a GCS heartbeat (MAVProxy) is ignored",
      not _is_ap_hb(_Hb(255, 190, MAV_AUTOPILOT_INVALID), 1))
check("    another component on the vehicle is ignored",
      not _is_ap_hb(_Hb(1, 154, MAV_AUTOPILOT_INVALID), 1))
check("    another vehicle's autopilot is ignored", not _is_ap_hb(_Hb(2, 1, 3), 1))

# Every write of current_mode / armed outside __init__ sits under the predicate.
_link_cls = next(c for c in ast.parse(src).body
                 if isinstance(c, ast.ClassDef) and c.name == 'MavlinkAutopilotLink')
_guarded, _all_writes = set(), []
for _fn in _link_cls.body:
    if not isinstance(_fn, ast.FunctionDef) or _fn.name == '__init__':
        continue
    for _n in ast.walk(_fn):
        if (isinstance(_n, ast.If) and isinstance(_n.test, ast.Call)
                and getattr(_n.test.func, 'id', None) == 'is_autopilot_heartbeat'):
            _guarded |= {id(a) for s in _n.body for a in ast.walk(s)}
        if isinstance(_n, ast.Assign):
            for _t in _n.targets:
                if (isinstance(_t, ast.Attribute) and _t.attr in ('current_mode', 'armed')
                        and getattr(_t.value, 'id', None) == 'self'):
                    _all_writes.append((_fn.name, _t.attr, _n))
_unguarded = sorted({f"{fn}: self.{attr}" for fn, attr, n in _all_writes
                     if id(n) not in _guarded})
check("    every mode/armed write is guarded", bool(_all_writes) and not _unguarded,
      ("unguarded: " + ", ".join(_unguarded)) if _unguarded else
      f"{len(_all_writes)} write(s), all behind is_autopilot_heartbeat")

print()
print("rate-loop gains are set by hand; startup only reads and compares them:")
# The operator sets the <AXIS>_RATE_* values in MAVProxy.  A startup path that
# writes them silently undoes that setting on the next boot - the comparison
# flight flies the old value while the operator believes the new one is in.
from Companion import constants as _C
_rp_re = re.compile(r'^(PTCH|RLL|YAW)_RATE_(FF|P|I|D|IMAX|FLTT|FLTE|FLTD|SMAX)$')
_rp_badname = [k for k in _C.EXPECTED_RATE_PIDS if not _rp_re.match(k)]
check("    EXPECTED_RATE_PIDS names are rate-loop settings plant-ID covers", not _rp_badname,
      ("not a plant-ID rate-loop setting: " + ", ".join(_rp_badname)) if _rp_badname else
      f"{len(_C.EXPECTED_RATE_PIDS)} entries")
# PTCH/RLL _FLTE have their own check against a balloon that differs from the
# plant-ID report; listing them here too would compare one value against two.
_rp_dup = sorted({'PTCH_RATE_FLTE', 'RLL_RATE_FLTE'} & set(_C.EXPECTED_RATE_PIDS))
check("    PTCH/RLL _FLTE stay with their own check, not EXPECTED_RATE_PIDS", not _rp_dup,
      ", ".join(_rp_dup))
import math as _m
_rp_badval = [k for k, v in _C.EXPECTED_RATE_PIDS.items()
              if not isinstance(v, (int, float)) or not _m.isfinite(v) or v < 0]
check("    every expected value is a finite number >= 0", not _rp_badval,
      ", ".join(_rp_badval))
_crp = real.get('_check_rate_pids')
_crp_calls = {getattr(n.func, 'attr', getattr(n.func, 'id', None))
              for n in ast.walk(_crp) if isinstance(n, ast.Call)} if _crp else set()
_writers = sorted(_crp_calls & {'_enforce_param', 'param_set_send', '_set_param'})
check("    _check_rate_pids never writes a parameter", bool(_crp) and not _writers,
      ("calls " + ", ".join(_writers)) if _writers else "")
_rfc = real.get('read_fc_config')
check("    read_fc_config runs the check",
      bool(_rfc) and any(isinstance(n, ast.Call) and getattr(n.func, 'attr', None) == '_check_rate_pids'
                         for n in ast.walk(_rfc)))

print()
print("startup writes ONLY what does not survive between runs:")
# Saved FC parameters are set by hand and only CHECKED at startup; a write would
# silently replace the operator's value on the next boot.  The one exception is
# the MAVx stream-group rates: a ground station's stream request overwrites and
# saves them every session.  The other writers are the operator's Write and the
# mode switch's rate-loop set, which a later section pins.  --arduplane-autotune
# is a separate maintenance mode whose job is to tune and save gains, so it is
# outside this rule.
_writes = {}
for _base, _d, _files in os.walk(PKG):
    if 'tests' in _base.split(os.sep) or '__pycache__' in _base:
        continue
    for _fn in _files:
        if not _fn.endswith('.py') or _fn == 'arduplane_autotune.py':
            continue
        _t = ast.parse(open(os.path.join(_base, _fn)).read())
        for _f in ast.walk(_t):
            if not isinstance(_f, ast.FunctionDef):
                continue
            _src_f = ast.unparse(_f)
            for _w in ('param_set_send', 'MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN', 'set_and_save'):
                if _w in _src_f and not any(
                        isinstance(_g, ast.FunctionDef) and _g is not _f and _w in ast.unparse(_g)
                        for _g in ast.walk(_f)):
                    _writes.setdefault(f"{_fn}:{_f.name}", set()).add(_w)
check("    the only parameter writes are the stream rates, the operator's Write and "
      "the mode switch's rate-loop set",
      _writes == {'mavlink_link.py:log_telemetry_rate_config': {'param_set_send'},
                  'mavlink_link.py:write_native_param': {'param_set_send'},
                  'mavlink_link.py:_write_param_set': {'param_set_send'}},
      ", ".join(f"{k} ({', '.join(sorted(v))})" for k, v in sorted(_writes.items())))
# write_native_param is the operator's: reachable only from the Gain Tuner's
# Write button (ui -> store.write_native -> link.write_native_param), never from
# startup, shutdown, main.py or any automatic path.
_callers = {}
_roots = [PKG, os.path.join(PKG, '..', 'main.py')]
for _p in [os.path.join(_b, _f) for _b, _d, _fs in os.walk(PKG)
           if 'tests' not in _b.split(os.sep) and '__pycache__' not in _b
           for _f in _fs if _f.endswith('.py')] + [_roots[1]]:
    _t = ast.parse(open(_p).read())
    for _f in ast.walk(_t):
        if not isinstance(_f, ast.FunctionDef):
            continue
        for _c in ast.walk(_f):
            _nm = getattr(getattr(_c, 'func', None), 'attr', None) if isinstance(_c, ast.Call) else None
            if _nm in ('write_native_param', 'write_native'):
                _callers.setdefault(_nm, set()).add(f"{os.path.basename(_p)}:{_f.name}")
check("    link.write_native_param is called only by the tuner store's write_native",
      _callers.get('write_native_param') == {'controller_autotune_store.py:write_native'},
      str(sorted(_callers.get('write_native_param', []))))
check("    and store.write_native only by the Gain Tuner window",
      bool(_callers.get('write_native'))
      and all(c.startswith('controller_autotune.py:') for c in _callers['write_native']),
      str(sorted(_callers.get('write_native', []))))

# The Write path, against a fake FC: the value goes out as PARAM_SET, the FC is
# then asked for it, and only what the FC answers becomes the confirmed value
# the tuner shows and the log records.  A value the store refuses sends nothing.
print()
print("the Gain Tuner's Write and Read go through the FC and back:")
import struct as _struct, threading as _threading
from Companion.adapters import mavlink_link as _ML
from Companion.controller_autotune_store import ControllerAutotuneStore as _Store

class _FakeMav:
    def __init__(self, fc, mode, link=None):
        self.fc, self.mode, self.sets, self.link = fc, mode, [], link
    def param_set_send(self, sysid, compid, name, value, ptype):
        self.sets.append((name.decode(), value, ptype))
        if self.mode == 'accept':               # the FC stores it as a 32-bit float
            self.fc[name.decode()] = _struct.unpack('<f', _struct.pack('<f', value))[0]
    def set_mode_send(self, sysid, flag, mode_id):
        self.link.current_mode = mode_id        # the FC confirms at once

def _fake_link(fc, mode='accept'):
    L = object.__new__(_ML.MavlinkAutopilotLink)
    L.master = type('M', (), {})()
    L.master.mav = _FakeMav(fc, mode, L)
    L.master.target_system = L.master.target_component = 1
    L._mav_lock = _threading.Lock()
    L._subscribe = lambda *a, **k: 'q'
    L._unsubscribe = lambda *a, **k: None
    L._fetch_param = (lambda q, n, attempts=3, per_attempt_s=2.0:
                      None if mode == 'silent' else fc.get(n))
    L._fetch_params = (lambda q, names, attempts=3, per_attempt_s=2.0, tick=None:
                       {} if mode == 'silent' else {n: fc[n] for n in names if n in fc})
    L.native_params = {}
    L.native_params_operator = {}
    L._mode_param_sets = False
    L._fc_param_names = None
    L.current_mode = None
    return L

_L = _fake_link({'PTCH_RATE_P': 1.3634})
_got = _L.write_native_param('PTCH_RATE_P', 2.0)
check("    a write is sent once, as REAL32, and the readback is recorded",
      _got == 2.0 and _L.native_params.get('PTCH_RATE_P') == 2.0
      and _L.master.mav.sets == [('PTCH_RATE_P', 2.0,
                                  _ML.mavutil.mavlink.MAV_PARAM_TYPE_REAL32)],
      "%s %s" % (_got, _L.master.mav.sets))
_L = _fake_link({'PTCH_RATE_P': 1.3634}, mode='deny')
_got = _L.write_native_param('PTCH_RATE_P', 2.0)
check("    an FC that keeps its value is recorded with THAT value",
      _got == 1.3634 and _L.native_params.get('PTCH_RATE_P') == 1.3634, str(_got))
_L = _fake_link({'PTCH_RATE_P': 1.3634}, mode='silent')
_L.native_params = {'PTCH_RATE_P': 1.3634}      # confirmed before the write
_got = _L.write_native_param('PTCH_RATE_P', 2.0)
check("    no readback leaves no confirmed value - not the one held before (the log shows none)",
      _got is None and 'PTCH_RATE_P' not in _L.native_params, str(_L.native_params))
_L = _fake_link({'RLL_RATE_P': 0.0973})
check("    Read records the FC's value",
      _L.read_native_param('RLL_RATE_P') == 0.0973
      and _L.native_params == {'RLL_RATE_P': 0.0973} and not _L.master.mav.sets)
_L = _fake_link({'PTCH_RATE_P': 1.3634})
_ctl = type('C', (), {})(); _ctl.link = _L
_store = object.__new__(_Store); _store._c = _ctl
for _bad in ('-1', 'abc', 'inf', 'nan', '1e39', ''):
    _store.write_native('PTCH_RATE_P', _bad)
check("    the store sends nothing for a negative, non-numeric or non-finite value",
      not _L.master.mav.sets, str(_L.master.mav.sets))
_fc_all = {n: 0.5 for n in _C.ARDUPLANE_AUTOTUNE_PARAMS if not n.startswith('YAW_')}
_L = _fake_link(dict(_fc_all))
_L._fc_param_names = set(_fc_all)          # an FC with YAW_RATE_ENABLE=0
_L._param_mismatches = []
_L._check_rate_pids()
_L._read_native_params()
check("    startup reads every parameter the FC has, and writes none",
      _L.native_params == _fc_all and not _L.master.mav.sets,
      "%d of %d, %d writes" % (len(_L.native_params), len(_fc_all), len(_L.master.mav.sets)))
_L.write_native_param('PTCH_RATE_P', 2.0)
check("    the startup values are kept apart: a later write does not move them",
      _L.native_params_at_startup == _fc_all and _L.native_params['PTCH_RATE_P'] == 2.0,
      "startup PTCH_RATE_P %s" % _L.native_params_at_startup.get('PTCH_RATE_P'))
# The reset icon writes the row's default exactly, and the console prints it
# exactly.  0.12345678 needs eight figures as a 32-bit float, so six-figure text
# lands on a neighbour when written and hides the difference when printed.
# PTCH2SRV_TCONST has no expected value, so its default is the startup value.
import logging as _logging
from Companion.ui import controller_autotune as _UI
from Companion.controller_autotune_store import (float32_text as _f32text,
                                                  acro_default_value as _expected)
_lines = []
_cap = _logging.Handler(); _cap.emit = lambda rec: _lines.append(rec.getMessage())
_plog = _logging.getLogger("balloon_chaser"); _plevel = _plog.level
_plog.addHandler(_cap); _plog.setLevel(_logging.INFO)

def _reset_through_tuner(name, fc, startup):
    """Press a row's reset icon against a fake FC; return the link."""
    L = _fake_link(dict(fc))
    L.native_params_at_startup = dict(startup)
    ctl = type('C', (), {})(); ctl.link = L
    st = object.__new__(_Store); st._c = ctl
    box = type('V', (), {'set': lambda self, t: setattr(self, 't', t),
                         'get': lambda self: self.t})()
    tuner = object.__new__(_UI.ControllerAutotune)
    tuner._store = st
    tuner._native_rows = {name: {'var': box}}
    tuner._on_native_reset(name)
    for t in _threading.enumerate():
        if t.name == 'fc-param-reset-' + name:
            t.join(5)
    L.read_native_param(name)
    return L

_v = _struct.unpack('<f', _struct.pack('<f', 0.12345678))[0]
check("    PTCH2SRV_TCONST has no stored Acro default (the startup-value case below)",
      _expected('PTCH2SRV_TCONST') is None)
_L = _reset_through_tuner('PTCH2SRV_TCONST', {'PTCH2SRV_TCONST': 0.5},
                          {'PTCH2SRV_TCONST': _v})
check("    the reset icon writes the startup value exactly, not the box's rounded text",
      _L.native_params.get('PTCH2SRV_TCONST') == _v
      and _f32text(_v) == '0.12345678'
      and _struct.pack('<f', float(_UI._fmt_native(_v))) != _struct.pack('<f', _v),
      "startup %r, sent %s, FC now %r" % (_v, [s[1] for s in _L.master.mav.sets],
                                          _L.native_params.get('PTCH2SRV_TCONST')))
check("    the console prints the written, read-back and read values exactly",
      'write PTCH2SRV_TCONST = 0.12345678 -> readback 0.12345678 ✓' in ' | '.join(_lines)
      and 'read PTCH2SRV_TCONST = 0.12345678' in ' | '.join(_lines),
      ' | '.join(l for l in _lines if 'PTCH2SRV_TCONST' in l))
# Where a row has a stored Acro default, the reset writes THAT, whatever the FC
# held at startup: a wrong value saved on the FC is otherwise what reset restores.
_bad = {n: 1.3634 for n in ('RLL_RATE_P', 'RLL_RATE_I', 'RLL_RATE_IMAX', 'RLL_RATE_FLTE')}
_reset_bad = {}
for _n in _bad:
    _L = _reset_through_tuner(_n, {_n: _bad[_n]}, {_n: _bad[_n]})
    _want = _struct.unpack('<f', _struct.pack('<f', _expected(_n)))[0]
    if _L.native_params.get(_n) != _want:
        _reset_bad[_n] = (_expected(_n), _L.native_params.get(_n))
check("    a row with a stored Acro default resets to it, not to the startup value",
      not _reset_bad and _expected('RLL_RATE_FLTE') == _C.RLL_RATE_FLTE_EXPECTED,
      str(_reset_bad))
check("    the roll rate loop's stored I and IMAX are 0",
      (_expected('RLL_RATE_I'), _expected('RLL_RATE_IMAX')) == (0.0, 0.0))
_L = _reset_through_tuner('PTCH2SRV_TCONST', {'PTCH2SRV_TCONST': 0.5}, {})
check("    no stored default and no startup value: the reset sends nothing",
      not _L.master.mav.sets, str(_L.master.mav.sets))
_plog.removeHandler(_cap); _plog.setLevel(_plevel)
# A mode switch changes the confirmed value.  A box holding a typed, unwritten
# value keeps it - replaced, the next Write would store the switch's value as
# the operator's - and a box that was showing the FC's value follows it.
class _Box:
    def __init__(self, t): self.t = t
    def set(self, t): self.t = t
    def get(self): return self.t
_ent = type('E', (), {'configure': lambda self, **k: None})()
_L = _fake_link({})
_L.native_params = {'PTCH_RATE_P': 1.0, 'PTCH_RATE_I': 1.0}
_ctl = type('C', (), {})(); _ctl.link = _L
_store = object.__new__(_Store); _store._c = _ctl
_tuner = object.__new__(_UI.ControllerAutotune)
_tuner._store = _store
_tuner._native_rows = {
    'PTCH_RATE_P': {'var': _Box('1.5'), 'shown': 1.0, 'ent': _ent, 'fg': 'w'},
    'PTCH_RATE_I': {'var': _Box('1'), 'shown': 1.0, 'ent': _ent, 'fg': 'w'}}
_L.native_params = {'PTCH_RATE_P': 1.3634, 'PTCH_RATE_I': 1.0225}   # the switch
_tuner._refresh_native_rows()
_rows = _tuner._native_rows
check("    a mode switch keeps a typed, unwritten value and updates a shown one",
      _rows['PTCH_RATE_P']['var'].get() == '1.5' and _rows['PTCH_RATE_P']['shown'] == 1.3634
      and _rows['PTCH_RATE_I']['var'].get() == '1.0225',
      "P box %r, I box %r" % (_rows['PTCH_RATE_P']['var'].get(), _rows['PTCH_RATE_I']['var'].get()))
# Read All: every parameter the FC has, in one batch, and no write.  A name the
# FC does not answer keeps the value it showed, as a single Read does.
_fc_all = {n: 0.25 for n in _C.ARDUPLANE_AUTOTUNE_PARAMS if not n.startswith('YAW_')}
_fc_ans = {n: v for n, v in _fc_all.items() if n != 'RLL_RATE_P'}   # one stays silent
_L = _fake_link(dict(_fc_ans))
_L._fc_param_names = set(_fc_all)          # an FC with YAW_RATE_ENABLE=0
_L.native_params = {'RLL_RATE_P': 1.3634, 'YAW_RATE_P': 2.0}
_got = _L.read_all_native_params()
check("    Read All reads every parameter the FC has, and writes none",
      _got == _fc_ans and not _L.master.mav.sets
      and all(_L.native_params[n] == 0.25 for n in _fc_ans),
      "%d of %d answered, %d writes" % (len(_got), len(_fc_ans), len(_L.master.mav.sets)))
check("    and one the FC does not answer keeps the value it showed",
      _L.native_params['RLL_RATE_P'] == 1.3634 and _L.native_params['YAW_RATE_P'] == 2.0,
      str({n: _L.native_params.get(n) for n in ('RLL_RATE_P', 'YAW_RATE_P')}))
# The button drops typed, unwritten text in EVERY box, then the poll shows what
# the FC reported - the same as pressing each row's Read.
_L = _fake_link({'PTCH_RATE_P': 1.3634, 'RLL_RATE_P': 0.0973})
_L._fc_param_names = {'PTCH_RATE_P', 'RLL_RATE_P'}
_L.native_params = {'PTCH_RATE_P': 1.0, 'RLL_RATE_P': 1.3634}
_ctl = type('C', (), {})(); _ctl.link = _L
_store = object.__new__(_Store); _store._c = _ctl
_tuner = object.__new__(_UI.ControllerAutotune)
_tuner._store = _store
_tuner._native_rows = {
    'PTCH_RATE_P': {'var': _Box('1.5'), 'shown': 1.0, 'ent': _ent, 'fg': 'w'},
    'RLL_RATE_P': {'var': _Box('1.3634'), 'shown': 1.3634, 'ent': _ent, 'fg': 'w'}}
_tuner._on_native_read_all()
for _t in _threading.enumerate():
    if _t.name == 'fc-param-read-all':
        _t.join(5)
_tuner._refresh_native_rows()
_rows = _tuner._native_rows
check("    the Read All button puts the FC's value in every box, typed text included",
      _rows['PTCH_RATE_P']['var'].get() == '1.3634'
      and _rows['RLL_RATE_P']['var'].get() == '0.0973' and not _L.master.mav.sets,
      "P box %r, RLL P box %r" % (_rows['PTCH_RATE_P']['var'].get(),
                                  _rows['RLL_RATE_P']['var'].get()))
# The script flies by the AHRS estimates.  At AHRS_EKF_TYPE 10 SITL substitutes
# the simulator's truth, and nothing in the flight shows it - so startup checks
# the estimator like any other value the flight code depends on, and halts.
_checked = {}
for _n in ast.walk(real['read_fc_config']):
    if (isinstance(_n, ast.Call) and getattr(_n.func, 'attr', None) == '_check_param'
            and len(_n.args) >= 2 and isinstance(_n.args[0], ast.Constant)):
        _checked[_n.args[0].value] = ast.unparse(_n.args[1])
check("    startup checks EK3_ENABLE and AHRS_EKF_TYPE against their expected values",
      _checked.get('EK3_ENABLE') == 'EK3_ENABLE_EXPECTED'
      and _checked.get('AHRS_EKF_TYPE') == 'AHRS_EKF_TYPE_EXPECTED',
      str({k: _checked.get(k) for k in ('EK3_ENABLE', 'AHRS_EKF_TYPE')}))
check("    the values are EKF3 running (1) and selected (3)",
      (_C.EK3_ENABLE_EXPECTED, _C.AHRS_EKF_TYPE_EXPECTED) == (1, 3))
_L = _fake_link({'AHRS_EKF_TYPE': 10.0, 'EK3_ENABLE': 1.0})
_L._param_mismatches = []
_L._check_param('EK3_ENABLE', _C.EK3_ENABLE_EXPECTED, int_compare=True)
_L._check_param('AHRS_EKF_TYPE', _C.AHRS_EKF_TYPE_EXPECTED, int_compare=True)
check("    simulator truth (10) is a difference to halt on, and nothing is written",
      [m[0] for m in _L._param_mismatches] == ['AHRS_EKF_TYPE'] and not _L.master.mav.sets,
      str(_L._param_mismatches))
_L = _fake_link({'PTCH_RATE_P': 1.3634})
_ctl = type('C', (), {})(); _ctl.link = _L
_store = object.__new__(_Store); _store._c = _ctl
_store.write_native('PTCH_RATE_P', ' 1.5 ')
check("    and sends exactly the typed number otherwise",
      [s[:2] for s in _L.master.mav.sets] == [('PTCH_RATE_P', 1.5)]
      and _store.native_value('PTCH_RATE_P') == 1.5, str(_L.master.mav.sets))
_ltrc = real.get('log_telemetry_rate_config')
_names = set(re.findall(r"'(MAV\d_[A-Z_0-9]+)'", ast.unparse(_ltrc))) if _ltrc else set()
check("    and it writes only MAVx_EXTRA1 / MAVx_RC_CHAN",
      bool(_names) and all(re.fullmatch(r'MAV\d_(EXTRA1|RC_CHAN)', n) for n in _names),
      ", ".join(sorted(_names)))
_shut = real.get('shutdown')
check("    shutdown changes no parameter",
      bool(_shut) and not any(isinstance(n, ast.Call) for n in ast.walk(_shut)))

# Each mode flies its own rate loop: ArduPlane's modes AUTOTUNE's values
# (NATIVE_MODE_RATE_PIDS), ACRO the operator's.  A switch writes the new mode's
# set, and only what the FC does not already hold - measured against the
# startup read plus every write since, never by asking the FC first.
print()
print("each mode switch writes the rate loop that mode flies, and only what differs:")
_nm = _C.NATIVE_MODE_RATE_PIDS
check("    NATIVE_MODE_RATE_PIDS names are parameters the Gain Tuner holds",
      bool(_nm) and set(_nm) <= set(_C.ARDUPLANE_AUTOTUNE_PARAMS),
      ", ".join(sorted(set(_nm) - set(_C.ARDUPLANE_AUTOTUNE_PARAMS))))
check("    and every value is a finite number >= 0",
      all(isinstance(v, (int, float)) and _m.isfinite(v) and v >= 0 for v in _nm.values()))
_L = _fake_link({})
_L.native_params_operator = {'X': 1.0}
check("    ACRO flies the operator's values, MANUAL and AUTOTUNE none, every other "
      "mode NATIVE_MODE_RATE_PIDS",
      _L._mode_param_set(4) is _L.native_params_operator
      and _L._mode_param_set(0) is None and _L._mode_param_set(8) is None
      and all(_L._mode_param_set(m) is _nm for m in (5, 6, 7, 10, 11, 12, 13, 15)))
# Who may switch it on: the controller, after its startup read.  The ArduPlane
# AUTOTUNE maintenance mode switches modes through the same _set_mode and must
# never write - it would replace the gains AUTOTUNE is tuning.
_en, _wps = {}, {}
for _p in [os.path.join(_b, _f) for _b, _d, _fs in os.walk(PKG)
           if 'tests' not in _b.split(os.sep) and '__pycache__' not in _b
           for _f in _fs if _f.endswith('.py')] + [_roots[1]]:
    for _f in ast.walk(ast.parse(open(_p).read())):
        if not isinstance(_f, ast.FunctionDef):
            continue
        for _c in ast.walk(_f):
            _a = getattr(getattr(_c, 'func', None), 'attr', None) if isinstance(_c, ast.Call) else None
            if _a == 'enable_mode_param_sets':
                _en.setdefault(f"{os.path.basename(_p)}:{_f.name}", _f)
            elif _a in ('_write_param_set', 'write_rate_loop_set', 'write_param_set'):
                _wps.setdefault(_a, set()).add(f"{os.path.basename(_p)}:{_f.name}")
check("    a rate-loop set is written only by a mode switch or the tuner's set Write",
      _wps.get('_write_param_set') == {'mavlink_link.py:_set_mode',
                                       'mavlink_link.py:write_rate_loop_set'},
      str(sorted(_wps.get('_write_param_set', []))))
check("    and the set Write is reached only from the Gain Tuner window",
      _wps.get('write_rate_loop_set') == {'controller_autotune_store.py:write_param_set'}
      and bool(_wps.get('write_param_set'))
      and all(c.startswith('controller_autotune.py:') for c in _wps['write_param_set']),
      str({k: sorted(v) for k, v in _wps.items() if k != '_write_param_set'}))
_body = next(iter(_en.values()), None)
def _first_line(fn, attr):
    return min((n.lineno for n in ast.walk(fn) if isinstance(n, ast.Call)
                and getattr(n.func, 'attr', None) == attr), default=None)
check("    switched on only by the controller, after its startup read",
      len(_en) == 1 and next(iter(_en)).startswith('controller.py:')
      and _first_line(_body, 'read_fc_config') is not None
      and _first_line(_body, 'read_fc_config') < _first_line(_body, 'enable_mode_param_sets'),
      str(sorted(_en)))
_csrc = open(os.path.join(PKG, 'controller.py')).read()
check("    the exit hand-back leaves the FC holding the operator's values",
      "go_to_loiter('shutdown', keep_operator_params=True)" in _csrc)

_fc0 = {n: 0.5 for n in _C.ARDUPLANE_AUTOTUNE_PARAMS if not n.startswith('YAW_')}
_fc0.update({'PTCH_RATE_IMAX': 0.0, 'PTCH_RATE_I': 0.0,
             'RLL_RATE_IMAX': _nm['RLL_RATE_IMAX']})     # one already matches
_L = _fake_link(dict(_fc0))
_L._fc_param_names = set(_fc0)             # YAW_RATE_ENABLE=0: no YAW_RATE_*
_L._param_mismatches = []
_L._check_rate_pids()
_L._read_native_params()
_sets = _L.master.mav.sets
check("    the startup read is the operator's values",
      _L.native_params_operator == _fc0 and not _sets)
_L._set_mode(12, 'LOITER')
check("    nothing is written until the controller switches it on", not _sets)
_L.enable_mode_param_sets()
_L._set_mode(12, 'LOITER')
_want = {n: v for n, v in _nm.items() if n in _fc0 and n != 'RLL_RATE_IMAX'}
_f32 = lambda v: _struct.unpack('<f', _struct.pack('<f', v))[0]
check("    LOITER writes each AUTOTUNE value the FC lacks, once, as REAL32",
      sorted(s[0] for s in _sets) == sorted(_want)
      and all(s[2] == _ML.mavutil.mavlink.MAV_PARAM_TYPE_REAL32 for s in _sets)
      and all(_L.native_params[n] == _f32(v) for n, v in _want.items()),
      "%d written: %s" % (len(_sets), sorted(s[0] for s in _sets)))
check("    not a value already in place, nor one the FC does not have",
      'RLL_RATE_IMAX' not in [s[0] for s in _sets]
      and not any(s[0].startswith('YAW_') for s in _sets))
_n = len(_sets)
_L._set_mode(12, 'LOITER'); _L._set_mode(5, 'FBWA'); _L._set_mode(0, 'MANUAL')
check("    a second LOITER, FBWA over it, and MANUAL write nothing", len(_sets) == _n,
      str(_sets[_n:]))
_L._set_mode(4, 'ACRO')
check("    ACRO writes the operator's values back, and only those LOITER changed",
      sorted(s[0] for s in _sets[_n:]) == sorted(_want)
      and all(_L.native_params[n] == _fc0[n] for n in _fc0),
      str([s[:2] for s in _sets[_n:]]))
_L.write_native_param('PTCH_RATE_P', 2.0)
_n = len(_sets)
_L._set_mode(12, 'LOITER'); _L._set_mode(4, 'ACRO')
check("    a Gain Tuner Write becomes the operator's value ACRO restores",
      _L.native_params_operator['PTCH_RATE_P'] == 2.0
      and ('PTCH_RATE_P', 2.0) in [s[:2] for s in _sets[_n:]]
      and _L.native_params['PTCH_RATE_P'] == 2.0,
      str([s[:2] for s in _sets[_n:]]))
_L._set_mode(12, 'LOITER')
_n = len(_sets)
_L.set_mode_loiter(keep_operator_params=True)
check("    the exit hand-back writes the operator's values over AUTOTUNE's",
      _L.current_mode == 12
      and all(_L.native_params[n] == _L.native_params_operator[n] for n in _fc0)
      and len(_sets) - _n == len(_want),
      str([s[:2] for s in _sets[_n:]]))
_L = _fake_link(dict(_fc0), mode='silent')
_L._fc_param_names = set(_fc0)
_L.native_params = dict(_fc0); _L.native_params_operator = dict(_fc0)
_L.enable_mode_param_sets()
_L._set_mode(12, 'LOITER')
_n = len(_L.master.mav.sets)
_L._set_mode(12, 'LOITER')
check("    a value that did not answer is unknown: blank in the log, written again",
      not any(n in _L.native_params for n in _nm if n in _fc0 and n != 'RLL_RATE_IMAX')
      and len(_L.master.mav.sets) == 2 * _n, "%d then %d" % (_n, len(_L.master.mav.sets)))

# Startup's rate-gain check passes either whole set the FC can hold - the Acro
# set, or the Autopilot set a switch out of ACRO leaves - and halts on anything
# else.  On the Autopilot set ACRO takes the stored Acro defaults, never the
# FC's: adopting them flies the balloon chaser on the autopilot's gains.
_acro_fc = {n: 0.5 for n in _C.ARDUPLANE_AUTOTUNE_PARAMS}
_acro_fc.update({n: float(v) for n, v in _C.ACRO_DEFAULT_PARAMS.items()})
_auto_fc = dict(_acro_fc)
_auto_fc.update({n: float(v) for n, v in _nm.items()})
def _startup(fc):
    L = _fake_link(dict(fc))
    L._fc_param_names = set(fc)
    L._param_mismatches = []
    L._fc_holds_autopilot_set = False
    L._check_rate_pids()
    L._read_native_params()
    return L
_L = _startup(_acro_fc)
check("    the Acro set whole passes, and is the operator's values",
      not _L._param_mismatches and not _L._fc_holds_autopilot_set
      and _L.native_params_operator == _acro_fc and not _L.master.mav.sets,
      str(_L._param_mismatches))
_L = _startup(_auto_fc)
_acro_d = {n: float(_C.ACRO_DEFAULT_PARAMS[n]) for n in _nm}
check("    the Autopilot set whole passes, and nothing is written at startup",
      not _L._param_mismatches and _L._fc_holds_autopilot_set
      and not _L.master.mav.sets, str(_L._param_mismatches))
check("    ACRO takes the stored Acro defaults for those, the FC's value for the rest",
      all(_L.native_params_operator[n] == v for n, v in _acro_d.items())
      and _L.native_params_operator['PTCH2SRV_TCONST'] == _auto_fc['PTCH2SRV_TCONST']
      and all(_L.native_params[n] == _auto_fc[n] for n in _nm),
      str({n: _L.native_params_operator[n] for n in ('PTCH_RATE_I', 'PTCH2SRV_TCONST')}))
_L.enable_mode_param_sets()
_L._set_mode(12, 'LOITER')
check("    LOITER then writes nothing: the FC already holds its set", not _L.master.mav.sets,
      str(_L.master.mav.sets))
_L._set_mode(4, 'ACRO')
_want_acro = sorted(n for n, v in _acro_d.items() if not _ML._same_float32(_nm[n], v))
check("    the first switch into ACRO writes the Acro defaults that differ",
      sorted(x[0] for x in _L.master.mav.sets) == _want_acro
      and all(_ML._same_float32(_L.native_params[n], v) for n, v in _acro_d.items()),
      str(sorted(x[0] for x in _L.master.mav.sets)))
_L = _startup({n: v for n, v in _auto_fc.items() if not n.startswith('YAW_')})
check("    an FC without the yaw rate gains (YAW_RATE_ENABLE=0) is recognised the same",
      not _L._param_mismatches and _L._fc_holds_autopilot_set
      and 'YAW_RATE_P' not in _L.native_params_operator)
_part = dict(_auto_fc); _part['PTCH_RATE_I'] = 0.5
_L = _startup(_part)
_exp_bad = sorted(n for n, v in _C.EXPECTED_RATE_PIDS.items()
                  if not _ML._rate_pid_matches(_part[n], v))
check("    the Autopilot set with one value off halts, listed against the Acro set",
      sorted(m[0] for m in _L._param_mismatches) == _exp_bad
      and not _L._fc_holds_autopilot_set and _L.native_params_operator == _part,
      str(sorted(m[0] for m in _L._param_mismatches)))
_part = dict(_acro_fc); _part['RLL_RATE_P'] = 0.5
_L = _startup(_part)
check("    the Acro set with one value off halts on that one",
      [m[0] for m in _L._param_mismatches] == ['RLL_RATE_P']
      and not _L._fc_holds_autopilot_set, str(_L._param_mismatches))

# A caller flying the new mode on RC overrides (the protection climb, the
# level-out) has its sticks on the moment the mode is confirmed - before the
# rate-loop write - and refreshed while the write waits on readbacks.
_L = _fake_link(dict(_fc0))
_L._fc_param_names = set(_fc0)
_L.native_params = dict(_fc0); _L.native_params_operator = dict(_fc0)
_L.enable_mode_param_sets()
_events = []
_ps = _L.master.mav.param_set_send
_L.master.mav.param_set_send = lambda *a: (_events.append('set'), _ps(*a))
_L._set_mode(5, 'FBWA', hold=lambda: _events.append('hold'))
check("    the caller's override goes on before the mode's rate-loop write",
      _events[:1] == ['hold'] and 'set' in _events, str(_events[:4]))
_src_ml = open(os.path.join(PKG, 'adapters', 'mavlink_link.py')).read()
check("    the protection climb and the level-out pass their sticks as that hold",
      "self._set_mode(5, 'FBWA', hold=_climb_sticks)" in _src_ml
      and "self._set_mode(5, 'FBWA', hold=_level_sticks)" in _src_ml)
import queue as _queue, time as _time
_L = _fake_link({})
_q = _queue.Queue()
_ticks = []
_L.master.mav.param_request_read_send = lambda *a: None       # the FC never answers
_L._wait_msg = lambda q, t: (_time.sleep(t), None)[1]
_t0 = _time.monotonic()
_ML.MavlinkAutopilotLink._fetch_params(_L, _q, ['A'], attempts=1, per_attempt_s=1.0,
                                       tick=lambda: _ticks.append(_time.monotonic()))
_gaps = [b - a for a, b in zip([_t0] + _ticks, _ticks)]
check("    and refreshed at least every HOLD_REFRESH_S while the readbacks are awaited",
      len(_ticks) >= 3 and max(_gaps) <= _ML.HOLD_REFRESH_S + 0.1,
      "%d refreshes, longest gap %.2f s" % (len(_ticks), max(_gaps) if _gaps else -1))

# The tuner's set Writes send a STORED set from constants.py, never the boxes,
# every value whether or not the FC already holds it.  The Default Acro Write
# also makes the defaults the operator's values; the Default Autopilot Write
# leaves them alone.  Walked through as the operator does it: write a
# PTCH_RATE_FF other than its default from its row, then press Default Acro
# Parameters.
_ad = _C.ACRO_DEFAULT_PARAMS
check("    the stored Acro defaults are EXPECTED_RATE_PIDS plus the PTCH/RLL _FLTE expected values",
      _ad == {**_C.EXPECTED_RATE_PIDS, 'PTCH_RATE_FLTE': _C.PTCH_RATE_FLTE_EXPECTED,
              'RLL_RATE_FLTE': _C.RLL_RATE_FLTE_EXPECTED}
      and set(_ad) <= set(_C.ARDUPLANE_AUTOTUNE_PARAMS),
      ", ".join(sorted(set(_ad) - set(_C.ARDUPLANE_AUTOTUNE_PARAMS))))
_L = _fake_link(dict(_fc0))
_L._fc_param_names = set(_fc0)
_L.native_params = dict(_fc0); _L.native_params_operator = dict(_fc0)
_L.enable_mode_param_sets(); _L.current_mode = 4
_edit = _f32(_ad['PTCH_RATE_FF'] + 0.5)              # never the stored default
_L.write_native_param('PTCH_RATE_FF', _edit)
_mid = (_L.read_native_param('PTCH_RATE_FF'), _L.native_params_operator['PTCH_RATE_FF'])
_n = len(_L.master.mav.sets)
_L.write_rate_loop_set('operator')
_sets = _L.master.mav.sets
_acro = {n: v for n, v in _ad.items() if n in _fc0}
check("    a row's Write puts its value on the FC; the Acro set Write then sends every stored default the FC has",
      _mid == (_edit, _edit)
      and sorted(s[0] for s in _sets[_n:]) == sorted(_acro),
      "after the row Write %s; %d sent" % (_mid, len(_sets) - _n))
check("    and a Read, or Read All, shows the stored default, not the value written before",
      _L.read_native_param('PTCH_RATE_FF') == _f32(_ad['PTCH_RATE_FF'])
      and _L.read_all_native_params().get('PTCH_RATE_FF') == _f32(_ad['PTCH_RATE_FF'])
      and _L.native_params['PTCH_RATE_FF'] == _f32(_ad['PTCH_RATE_FF']),
      str(_L.native_params.get('PTCH_RATE_FF')))
check("    the defaults become the operator's values; a parameter with no default keeps its own",
      all(_L.native_params_operator[n] == _f32(v) for n, v in _acro.items())
      and _L.native_params_operator['PTCH2SRV_TCONST'] == _fc0['PTCH2SRV_TCONST'],
      str({n: _L.native_params_operator[n] for n in ('PTCH_RATE_FF', 'PTCH2SRV_TCONST')}))
_n = len(_sets)
_L._set_mode(12, 'LOITER'); _L._set_mode(4, 'ACRO')
check("    so the next switch into ACRO writes the default back, not the value written before",
      _L.native_params['PTCH_RATE_FF'] == _f32(_ad['PTCH_RATE_FF'])
      and ('PTCH_RATE_FF', _edit) not in [s[:2] for s in _sets[_n:]],
      str([s[:2] for s in _sets[_n:] if s[0] == 'PTCH_RATE_FF']))
_op = dict(_L.native_params_operator)
_n = len(_sets)
_L.write_rate_loop_set('native')
check("    the Autopilot set Write sends every stored AUTOTUNE value the FC has, held or not",
      sorted(s[0] for s in _sets[_n:]) == sorted(n for n in _nm if n in _fc0)
      and all(_L.native_params[n] == _f32(v) for n, v in _nm.items() if n in _fc0),
      "%d sent" % (len(_sets) - _n))
check("    and leaves the operator's values alone", _L.native_params_operator == _op)
_L = _fake_link({'PTCH_RATE_FF': 0.0}, mode='silent')
_L._fc_param_names = {'PTCH_RATE_FF'}
_L.native_params_operator = {'PTCH_RATE_FF': 0.0}
_L.write_rate_loop_set('operator')
check("    an Acro default the FC does not confirm is still the operator's value, written again at the next ACRO",
      _L.native_params_operator == {'PTCH_RATE_FF': float(_ad['PTCH_RATE_FF'])}
      and 'PTCH_RATE_FF' not in _L.native_params, str(_L.native_params_operator))
try:
    _L.write_rate_loop_set('both'); _bad = False
except ValueError:
    _bad = True
check("    an unknown set name is refused", _bad)
check("    the rows read Default Autopilot Parameters and Default Acro Parameters",
      _UI.PARAM_SET_ROWS == (('native', 'Default Autopilot Parameters'),
                             ('operator', 'Default Acro Parameters')))

# _fetch_params: one request per name, replies taken in any order, a name that
# does not answer asked again and then given up on.
import queue as _queue
_L = _fake_link({})
_q = _queue.Queue()
_asked = []
class _PV:
    def __init__(self, n, v): self.param_id, self.param_value = n + '\x00', v
def _prr(sysid, compid, name, idx):
    _asked.append(name.decode())
    if name.decode() == 'B':
        _q.put(_PV('B', 2.0)); _q.put(_PV('OTHER', 9.0))
    if name.decode() == 'A' and _asked.count('A') == 2:
        _q.put(_PV('A', 1.0))
_L.master.mav.param_request_read_send = _prr
_L._wait_msg = lambda q, t: q.get(timeout=t) if not q.empty() else None
_got = _ML.MavlinkAutopilotLink._fetch_params(_L, _q, ['A', 'B', 'C'],
                                              attempts=2, per_attempt_s=0.05)
check("    _fetch_params gathers the replies, retries the silent, gives up after the last try",
      _got == {'A': 1.0, 'B': 2.0} and _asked == ['A', 'B', 'C', 'A', 'C'],
      "%s asked %s" % (_got, _asked))
# ArduPlane drops read requests past its queue, so no more than a batch is ever
# waiting on an answer.
_L = _fake_link({})
_q = _queue.Queue()
_out = {'now': 0, 'max': 0}
def _prr(sysid, compid, name, idx):
    _out['now'] += 1; _out['max'] = max(_out['max'], _out['now'])
    _q.put(_PV(name.decode(), 1.0))
def _wm(q, t):
    if q.empty():
        return None
    _out['now'] -= 1
    return q.get()
_L.master.mav.param_request_read_send = _prr
_L._wait_msg = _wm
_names = ['P%02d' % i for i in range(25)]
_got = _ML.MavlinkAutopilotLink._fetch_params(_L, _q, _names, per_attempt_s=0.05)
check("    and keeps no more than PARAM_READ_BATCH reads waiting at once",
      set(_got) == set(_names) and _out['max'] <= _ML.PARAM_READ_BATCH < 20,
      "%d answered, at most %d waiting" % (len(_got), _out['max']))

print()
print("while startup waits for Enter, the prompt stays the last thing on screen:")
# Other threads (telemetry rates, FC status text) keep logging during the wait;
# printed, they push the prompt off screen and the operator cannot tell startup is
# waiting.  They are held and printed once the prompt returns, and the console-log
# file keeps recording them as they happen.
import io as _io, logging as _logging
_fns = [f for f in ast.parse(src).body
        if isinstance(f, (ast.FunctionDef, ast.ClassDef))
        and f.name in ('_screen_log_handlers', '_HeldScreenOutput', 'halt_on_param_mismatches')]
_screen = _io.StringIO()
class _FakeSys:
    stdout = _screen
    stderr = _screen
_hlog = _logging.getLogger('test_link_contract.hold')
_hlog.propagate = False
_hlog.setLevel(_logging.INFO)
_screen_h = _logging.StreamHandler(_screen)
_hlog.addHandler(_screen_h)
_file_records = []
class _FileSink(_logging.Handler):
    def emit(self, r): _file_records.append(r.getMessage())
_hlog.addHandler(_FileSink())
def _run_hold(answer):
    _screen.seek(0); _screen.truncate(); _file_records.clear()
    seen = {}
    def _input(prompt=''):
        _screen.write(prompt)
        seen['at_prompt'] = _screen.getvalue()
        for k in range(3):
            _hlog.info('[TELEM RATES] tick %d', k)        # as another thread would
        seen['at_enter'] = _screen.getvalue()
        seen['file'] = list(_file_records)
        if answer == 'ctrl-c':
            raise KeyboardInterrupt
        return ''
    ns = {'logging': _logging, 'sys': _FakeSys, 'log': _hlog, 'input': _input}
    exec(compile(ast.Module(body=_fns, type_ignores=[]), 'mavlink_link.py', 'exec'), ns)
    stopped = False
    try:
        ns['halt_on_param_mismatches']([('PTCH_RATE_I', '1.0225', '0.1744', '')])
    except KeyboardInterrupt:
        stopped = True
    return seen, _screen.getvalue(), stopped
_seen, _after, _ = _run_hold('enter')
check("    nothing is printed while the prompt waits", _seen['at_enter'] == _seen['at_prompt'])
check("    the console-log file keeps recording during the wait",
      sum('TELEM' in m for m in _seen['file']) == 3)
_tail = _after[len(_seen['at_prompt']):]
check("    held messages are printed, in order, once Enter is pressed",
      _tail.find('tick 0') < _tail.find('tick 1') < _tail.find('tick 2') != -1
      and 'held while waiting' in _tail)
check("    the screen handler is released afterwards", not _screen_h.filters)
_seen, _after, _stopped = _run_hold('ctrl-c')
check("    Ctrl+C stops, and still prints what was held",
      _stopped and 'tick 2' in _after[len(_seen['at_prompt']):] and not _screen_h.filters)

print()
print("every gimbal-driver attribute the controller reads exists on the driver:")
# The controller reads the driver through getattr with a default, so a name the
# driver does not have reads the default forever and nothing fails: the
# reading time read as "now" made every gimbal age 0.
import ast as _ast2
_root2 = os.path.dirname(PKG)
_ctl2 = open(os.path.join(PKG, 'controller.py')).read()
_wanted = {n.args[1].value for n in _ast2.walk(_ast2.parse(_ctl2))
           if isinstance(n, _ast2.Call) and getattr(n.func, 'id', None) == 'getattr'
           and len(n.args) >= 2 and isinstance(n.args[0], _ast2.Name)
           and n.args[0].id == '_gd' and isinstance(n.args[1], _ast2.Constant)}
_sg = _ast2.parse(open(os.path.join(PKG, 'adapters', 'sim_gazebo.py')).read())
_drv = next(n for n in _sg.body if isinstance(n, _ast2.ClassDef) and n.name == 'GzGimbalDriver')
_has = {f.name for f in _drv.body if isinstance(f, _ast2.FunctionDef)}
_has |= {t.attr for n in _ast2.walk(_drv) if isinstance(n, _ast2.Assign)
         for t in n.targets if isinstance(t, _ast2.Attribute)}
check("    GzGimbalDriver has every one", _wanted and _wanted <= _has,
      "missing: %s" % sorted(_wanted - _has))

print("\n" + "=" * 74)
print("PASSED" if not fails else f"FAILED: {fails}"); print("=" * 74)
sys.exit(1 if fails else 0)
