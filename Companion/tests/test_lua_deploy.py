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

_i_ftp = _main.find('deploy_ftp(link)')
check("with no local scripts directory it uploads over MAVLink FTP, before the reload",
      0 <= _i_check < _i_ftp < _i_reload, f"ftp at {_i_ftp}, reload at {_i_reload}")

print("\n11. MAVLINK FTP: THE REAL AIRCRAFT'S SD CARD")
# A stand-in for ArduPilot's FTP server, following GCS_FTP.cpp at the
# firmware this project runs: the reply's seq is the request's + 1, a repeated
# request is answered by resending the last reply, a NAK carries its error in
# data[0], and FATFS will not rename onto an existing name.  The files live in
# a temporary directory, "/" stripped as SITL strips it.
import queue as _queue, struct as _struct, threading as _threading
from types import SimpleNamespace as _NS
from pymavlink import mavutil as _mu
from Companion.adapters.lua_deploy import (deploy_ftp, ap_crc32, FTP_DATA_MAX,
                                           FTP_PAYLOAD_LEN)
_M = _mu.mavlink
_HDR = _struct.Struct("<HBBBBBBI")

class _FakeFtp:
    def __init__(self, root):
        self.root = root
        self.fd = None
        self.last = None              # (request seq, reply payload)
        self.drop_every = 0           # drop every Nth reply: a lossy radio
        self.corrupt = None           # file name whose writes land damaged
        self.n = 0
        self.writes = 0
    def _p(self, data, size):
        return os.path.join(self.root, data[:size].split(b"\0")[0].decode().lstrip("/"))
    def handle(self, raw):
        seq, sess, op, size, _r, _b, _pad, off = _HDR.unpack_from(raw)
        data = raw[_HDR.size:_HDR.size + size]
        if self.last and self.last[0] == seq:
            return self.last[1]
        ack, body = True, b""
        def nak(code):
            return False, bytes([code])
        try:
            if op == _M.MAV_FTP_OPCODE_RESETSESSION or op == _M.MAV_FTP_OPCODE_TERMINATESESSION:
                if self.fd: self.fd.close(); self.fd = None
            elif op == _M.MAV_FTP_OPCODE_LISTDIRECTORY:
                p = self._p(data, size)
                if not os.path.isdir(p): ack, body = nak(_M.MAV_FTP_ERR_FILENOTFOUND)
                elif not os.listdir(p): ack, body = nak(_M.MAV_FTP_ERR_EOF)
            elif op == _M.MAV_FTP_OPCODE_CALCFILECRC:
                p = self._p(data, size)
                if not os.path.isfile(p): ack, body = nak(_M.MAV_FTP_ERR_FILENOTFOUND)
                else: body = _struct.pack("<I", ap_crc32(open(p, "rb").read()))
            elif op == _M.MAV_FTP_OPCODE_CREATEFILE:
                p = self._p(data, size)
                self.name = os.path.basename(p)
                self.fd = open(p, "wb")
            elif op == _M.MAV_FTP_OPCODE_WRITEFILE:
                self.writes += 1
                if self.corrupt and self.name.startswith(self.corrupt):
                    data = bytes([data[0] ^ 0xFF]) + data[1:]
                self.fd.seek(off); self.fd.write(data)
            elif op == _M.MAV_FTP_OPCODE_CREATEDIRECTORY:
                p = self._p(data, size)
                if os.path.exists(p): ack, body = nak(_M.MAV_FTP_ERR_FILEEXISTS)
                else: os.mkdir(p)
            elif op == _M.MAV_FTP_OPCODE_REMOVEFILE:
                p = self._p(data, size)
                if not os.path.exists(p): ack, body = nak(_M.MAV_FTP_ERR_FILENOTFOUND)
                else: os.remove(p)
            elif op == _M.MAV_FTP_OPCODE_RENAME:
                a, b = data[:size].split(b"\0")[:2]
                pa = os.path.join(self.root, a.decode().lstrip("/"))
                pb = os.path.join(self.root, b.decode().lstrip("/"))
                if os.path.exists(pb): ack, body = nak(_M.MAV_FTP_ERR_FILEEXISTS)
                else: os.rename(pa, pb)
            else:
                ack, body = nak(_M.MAV_FTP_ERR_UNKNOWNCOMMAND)
        except OSError:
            ack, body = nak(_M.MAV_FTP_ERR_FAIL)
        reply = (_HDR.pack((seq + 1) & 0xFFFF, sess,
                           _M.MAV_FTP_OPCODE_ACK if ack else _M.MAV_FTP_OPCODE_NAK,
                           len(body), op, 0, 0, off) + body).ljust(FTP_PAYLOAD_LEN, b"\0")
        self.last = (seq, reply)
        return reply

class _FakeLink:
    """The parts of MavlinkAutopilotLink deploy_ftp uses: the subscriber queues
    the telemetry thread feeds, the mav lock, the connection, armed."""
    def __init__(self, server):
        self.server, self.armed, self._qs = server, False, []
        self._mav_lock = _threading.Lock()
        self.master = _NS(source_system=255, source_component=190,
                          target_system=1, target_component=1,
                          mav=_NS(file_transfer_protocol_send=self._send))
    def _send(self, net, sysid, compid, payload):
        assert len(payload) == FTP_PAYLOAD_LEN
        reply = self.server.handle(bytes(payload))
        self.server.n += 1
        if self.server.drop_every and self.server.n % self.server.drop_every == 0:
            return
        # A ground station's FTP reply on the same link, for another system:
        # never taken for ours.
        for q in self._qs:
            q.put(_NS(target_system=200, target_component=190, payload=list(reply)))
            q.put(_NS(target_system=255, target_component=190, payload=list(reply)))
    def _subscribe(self, *types, maxsize=50):
        q = _queue.Queue(); self._qs.append(q); return q
    def _unsubscribe(self, q, *types):
        self._qs.remove(q)
    def _wait_msg(self, q, timeout):
        try: return q.get(timeout=timeout)
        except _queue.Empty: return None

import Companion.adapters.lua_deploy as _ldm
_ldm.FTP_REPLY_TIMEOUT_S = 0.05
_src = os.path.join(_root, "LUA")
def _remote(fs, name):
    return os.path.join(fs, "APM", "scripts",
                        name if name in ENTRY_SCRIPTS else os.path.join("modules", name))
def _same(fs, name):
    return open(_remote(fs, name), "rb").read() == open(os.path.join(_src, name), "rb").read()

_fs = tempfile.mkdtemp()
os.makedirs(os.path.join(_fs, "APM", "scripts"))       # made at boot (AP_Scripting::init)
for bad in ("ap_shim.lua", RETIRED_ENTRY_SCRIPTS[0], MODULES[0]):
    open(os.path.join(_fs, "APM", "scripts", bad), "w").write("-- stale\n")
_srv = _FakeFtp(_fs); _lk = _FakeLink(_srv)
_srv.drop_every = 7
ok, changed, dest = deploy_ftp(_lk)
check("first upload succeeds through a radio that loses every 7th reply", ok and changed, dest)
check("into the board's /APM/scripts: entry script there, modules in modules/",
      all(_same(_fs, n) for n in ENTRY_SCRIPTS + MODULES))
check("the strays, a stale ap_shim.lua and the retired entry script are removed",
      sorted(os.listdir(os.path.join(_fs, "APM", "scripts"))) == sorted(list(ENTRY_SCRIPTS) + ["modules"]),
      str(os.listdir(os.path.join(_fs, "APM", "scripts"))))
check("no .part file is left behind",
      not any(f.endswith(".part") for f in os.listdir(os.path.join(_fs, "APM", "scripts", "modules"))))
_srv.drop_every = 0

_srv.writes = 0
ok, changed, _ = deploy_ftp(_lk)
check("an unchanged aircraft: no change reported, nothing written",
      ok and not changed and _srv.writes == 0, f"{_srv.writes} writes")

_target = _remote(_fs, "path_attitude.lua")
open(_target, "ab").write(b"-- edited on the card\n")
_srv.writes = 0
ok, changed, _ = deploy_ftp(_lk)
_size = os.path.getsize(os.path.join(_src, "path_attitude.lua"))
check("one file that differs is the only one sent, and it is repaired",
      ok and changed and _same(_fs, "path_attitude.lua")
      and _srv.writes == -(-_size // FTP_DATA_MAX), f"{_srv.writes} writes")

open(_target, "ab").write(b"-- edited again\n")
_lk.armed = True
_before = open(_target, "rb").read()
ok, changed, _ = deploy_ftp(_lk)
check("ARMED with a file that differs: refused and nothing changed",
      not ok and not changed and open(_target, "rb").read() == _before)
_lk.armed = False
os.remove(_target)
_lk.armed = True
ok, changed, _ = deploy_ftp(_lk)
check("armed with a file missing from the card: refused too", not ok and not changed)
_lk.armed = False
ok, _, _ = deploy_ftp(_lk)
_lk.armed = True
ok2, changed2, _ = deploy_ftp(_lk)
check("armed with every file current: passes, nothing to change", ok and ok2 and not changed2)
_lk.armed = False

open(_target, "ab").write(b"-- edited\n")
_srv.corrupt = "path_attitude.lua"
ok, changed, _ = deploy_ftp(_lk)
_srv.corrupt = None
check("a file damaged on the way fails verification and is never renamed into place",
      not ok and changed and open(_target, "rb").read().endswith(b"-- edited\n")
      and not os.path.exists(_target + ".part"))

_fs2 = tempfile.mkdtemp()
os.makedirs(os.path.join(_fs2, "scripts"))           # SITL: ./scripts
ok, changed, dest = deploy_ftp(_FakeLink(_FakeFtp(_fs2)))
check("SITL's ./scripts is found the same way",
      ok and changed and os.path.isfile(os.path.join(_fs2, "scripts", ENTRY_SCRIPTS[0])), dest)
os.makedirs(os.path.join(_fs2, "APM", "scripts"))
ok, changed, _ = deploy_ftp(_FakeLink(_FakeFtp(_fs2)))
check("both directories present: refused, it cannot tell which is read", not ok and not changed)
ok, changed, _ = deploy_ftp(_FakeLink(_FakeFtp(tempfile.mkdtemp())))
check("no scripts directory (scripting off at boot): refused", not ok and not changed)

class _Silent(_FakeLink):
    def _send(self, *a):
        pass
ok, changed, _ = deploy_ftp(_Silent(_FakeFtp(_fs)))
check("an autopilot that never answers: fails, nothing changed", not ok and not changed)

print("\n" + "=" * 70)
print("PASSED" if not fails else f"FAILED: {fails}"); print("=" * 70)
sys.exit(1 if fails else 0)
