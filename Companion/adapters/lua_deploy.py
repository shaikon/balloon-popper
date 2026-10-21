# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
"""Deploy the Lua steering scripts to the flight controller, automatically.

Copies the FC scripts (FC_SCRIPTS) to ArduPilot's scripts directory and
VERIFIES every one by reading it back and comparing SHA-256.

WHY VERIFY RATHER THAN JUST COPY
  A truncated or half-written .lua is not an inert file - ArduPilot loads and
  runs whatever is there.  A script cut off mid-function fails at load and
  nothing flies the balloon chaser; cut off mid-table and it may load with a constant
  missing, which is worse because it flies.
  Hashing the file as it exists ON DISK afterwards is what makes the copy an
  observation rather than an assumption.

TWO ROUTES: A LOCAL COPY AND MAVLINK FTP
  deploy() copies with shutil when the scripts directory is a local path: an
  explicit --lua-scripts, or SITL running on this machine, which shares its
  filesystem.  deploy_ftp() uploads over the MAVLink connection when it is not:
  the real aircraft's SD card, or SITL on another machine.  It asks the
  autopilot for each file's CRC32 first and sends only what differs, so an
  aircraft already holding this build costs one round trip per file.  Each
  upload goes to a `.part` name and is renamed into place once its CRC matches;
  ArduPilot loads only names ending in `.lua`, so a half-sent file is never
  run.  It refuses to change anything while the aircraft is armed: the reload
  that makes new scripts run drops the script's control of the aircraft.

WHAT IS DELIBERATELY NOT COPIED
  ap_shim.lua and ap_stubs.lua.  ap_shim defines Vector3f and Quaternion, which
  ArduPilot already provides as real userdata; loading it on the FC would shadow
  the genuine bindings with desktop stand-ins and every rotation would silently
  run on my reimplementation instead of the autopilot's.  Same for the test
  drivers, which pull in ap_shim transitively.
"""

from __future__ import annotations

import hashlib
import logging
import os
import shutil
import struct
import time
import zlib

from pymavlink import mavutil

log = logging.getLogger(__name__)

# The scripts that RUN on the flight controller.  Everything else in LUA/ is
# desktop-only; see the module docstring for why that separation is load-bearing.
# ArduPilot EXECUTES every .lua in the scripts directory as a standalone
# script, and require() looks ONLY in scripts/modules/ (LUA_LDIR =
# SCRIPTING_DIRECTORY "/modules/").  So the entry point and the library modules
# must NOT live together:
#
#   scripts/balloon_chaser_fc.lua          <- run by ArduPilot, returns update()
#   scripts/modules/geometry.lua    <- found by require("geometry")
#
# Putting the modules in scripts/ fails twice over: require cannot find them, and
# ArduPilot tries to run each one as a script - they return a table, not an
# update function.  Nothing loads, and the only symptom is silence.
ENTRY_SCRIPTS = (
    "balloon_chaser_fc.lua",
)

# Entry scripts that earlier builds shipped under another name.  ArduPilot runs
# every .lua in scripts/, so a retired one left there runs BESIDE the current
# script: two scripts on the one serial port, each claiming the aircraft with
# its own law.  deploy() removes them.
RETIRED_ENTRY_SCRIPTS = (
    "pursuit_fc.lua",
)

MODULES = (
    "protocol.lua",
    "constants.lua",
    "cascade_state.lua",
    "geometry.lua",
    "estimator.lua",
    "control_roll.lua",
    "control_pitch.lua",
    "track.lua",
    "cascade.lua",
    "excite.lua",
    "curve_flyer.lua",
    "path_attitude.lua",
)

FC_SCRIPTS = ENTRY_SCRIPTS + MODULES      # everything that ships

# Never upload these, even if asked.
FORBIDDEN = ("ap_shim.lua", "ap_stubs.lua")

# Where ArduPilot looks.  SITL reads scripts from `scripts/` relative to its
# WORKING directory - the same place it writes eeprom.bin - so finding
# eeprom.bin is the only reliable way to locate it; the fixed paths below are
# fallbacks for layouts that do not match.
_EEPROM_MARKERS = (
    "~/ardupilot/ArduPlane",
    "~/ardupilot",
    ".",
)
_CANDIDATES = (
    "~/ardupilot/ArduPlane/scripts",
    "~/ardupilot/build/sitl/scripts",
    "./scripts",
    "~/.config/ardupilot/scripts",
)


# The cwd probe runs more than once per startup (path resolution, then the
# deploy step), and the value never changes between calls - so remember the last
# cwd we announced and stay quiet when it repeats, rather than printing the same
# "running in:" line twice.
_LAST_LOGGED_PROC_CWD: str | None = None


def _sitl_cwd_from_proc() -> str | None:
    """Read the autopilot's ACTUAL working directory from /proc.

    SCRIPTING_DIRECTORY is "./scripts" - relative to the process's cwd, not to
    the source tree, not to the build directory, and NOT to wherever eeprom.bin
    happens to sit.  sim_vehicle.py launches the binary from the directory the
    OPERATOR ran it in, so on this setup the autopilot was looking in
    "/mnt/c/Users/Shai Kon/scripts" while every file was deployed to
    "~/ardupilot/ArduPlane/scripts".

    /proc/<pid>/cwd is the authoritative answer and costs one readlink.  The
    eeprom.bin heuristic below is kept only as a fallback for platforms without
    /proc, and it is a GUESS - which is why it is second.
    """
    import glob
    for pat in ("/proc/[0-9]*/cmdline",):
        for cmdline in glob.glob(pat):
            try:
                with open(cmdline, "rb") as fh:
                    argv = fh.read().split(b"\0")
                if not argv or b"arduplane" not in argv[0].lower():
                    continue
                pid = cmdline.split("/")[2]
                cwd = os.readlink(f"/proc/{pid}/cwd")
                global _LAST_LOGGED_PROC_CWD
                if cwd != _LAST_LOGGED_PROC_CWD:
                    log.info("[lua] autopilot pid %s is running in: %s", pid, cwd)
                    _LAST_LOGGED_PROC_CWD = cwd
                return os.path.join(cwd, "scripts")
            except (OSError, IndexError, PermissionError):
                continue
    return None


def _sitl_cwd_from_eeprom() -> str | None:
    """Find SITL's working directory by locating eeprom.bin.

    sim_vehicle.py runs the binary from the vehicle directory and SITL writes
    eeprom.bin there, so `<that dir>/scripts` is where ArduPilot will look.
    Deploying to a plausible-looking directory that SITL does not read is the
    failure this avoids - the files land, nothing loads them, and the only
    symptom is that the script never starts.
    """
    for base in _EEPROM_MARKERS:
        b = os.path.abspath(os.path.expanduser(base))
        if os.path.isfile(os.path.join(b, "eeprom.bin")):
            log.info("[lua] located SITL working directory via eeprom.bin: %s", b)
            return os.path.join(b, "scripts")
    return None


def _sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def find_scripts_dir(explicit: str | None = None) -> str | None:
    """Locate ArduPilot's scripts directory.

    An explicit path is created if missing - the operator naming a directory is
    an instruction, not a guess.  Auto-detection only ever RETURNS a directory
    that already exists, because creating `~/ardupilot/...` on a machine that
    has no ArduPilot there would silently deploy into a path nothing reads.
    """
    if explicit:
        p = os.path.abspath(os.path.expanduser(explicit))
        os.makedirs(p, exist_ok=True)
        return p
    # Ask the running process first - it cannot be wrong.
    found = _sitl_cwd_from_proc() or _sitl_cwd_from_eeprom()
    if found:
        os.makedirs(found, exist_ok=True)
        return found
    for cand in _CANDIDATES:
        p = os.path.abspath(os.path.expanduser(cand))
        if os.path.isdir(p):
            log.info("[lua] found ArduPilot scripts directory: %s", p)
            return p
        parent = os.path.dirname(p)
        if os.path.isdir(parent):
            os.makedirs(p, exist_ok=True)
            log.info("[lua] created ArduPilot scripts directory: %s", p)
            return p
    return None


def deploy(src_dir: str | None = None, dest_dir: str | None = None
           ) -> tuple[bool, bool, str | None]:
    """Copy and verify the FC scripts.

    Returns (ok, changed, dest).  `changed` is True when any file's content
    differed from what was already there - the caller needs it because ArduPilot
    reads scripts only when its scripting engine starts, so a content change
    requires a reload (MAV_CMD_SCRIPTING, never a reboot).
    """
    if src_dir is None:
        here = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))))
        src_dir = os.path.join(here, "LUA")
    dest = find_scripts_dir(dest_dir)
    if dest is None:
        log.error("[lua] could not find an ArduPilot scripts directory. Pass "
                  "--lua-scripts <path>. On real hardware the SD card is not "
                  "reachable from here - copy the files to APM/scripts/ or use "
                  "MAVLink FTP.")
        return False, False, None

    # Cross-check against the RUNNING autopilot BEFORE anything is written.  A
    # deploy into a directory the process does not read is silent - the files
    # are correct, the hashes verify, and nothing ever loads - and copying first
    # would also leave a second set of scripts behind in the wrong place.
    live = _sitl_cwd_from_proc()
    if live and os.path.abspath(live) != os.path.abspath(dest):
        log.error("[lua] REFUSING TO DEPLOY: wrong directory, nothing copied.")
        log.error("      would deploy to : %s", dest)
        log.error("      autopilot reads : %s  (its cwd + /scripts)", live)
        log.error("      ArduPilot resolves SCRIPTING_DIRECTORY ('./scripts') "
                  "against its OWN working directory. Re-run with "
                  "--lua-scripts '%s', or launch sim_vehicle.py from the "
                  "directory you want to deploy into.", live)
        return False, False, dest

    missing = [f for f in FC_SCRIPTS
               if not os.path.isfile(os.path.join(src_dir, f))]
    if missing:
        log.error("[lua] missing from %s: %s", src_dir, missing)
        return False, False, dest

    mod_dir = os.path.join(dest, "modules")
    os.makedirs(mod_dir, exist_ok=True)

    changed = 0
    for name in FC_SCRIPTS:
        s = os.path.join(src_dir, name)
        # entry point in scripts/, everything else in scripts/modules/
        d = os.path.join(dest if name in ENTRY_SCRIPTS else mod_dir, name)
        want = _sha256(s)
        if os.path.isfile(d) and _sha256(d) == want:
            continue
        # Write to a temp name in the SAME directory, then rename.  os.replace
        # is atomic on POSIX, so ArduPilot can never observe a partial file even
        # if this process dies mid-copy.
        tmp = d + ".part"
        shutil.copyfile(s, tmp)
        os.replace(tmp, d)
        got = _sha256(d)
        if got != want:
            log.error("[lua] %s FAILED verification after copy (%s != %s) - "
                      "refusing to continue; ArduPilot would run a corrupt "
                      "script", name, got[:12], want[:12])
            return False, True, dest
        changed += 1

    # Anything dangerous already sitting in the destination is removed rather
    # than left: a stale ap_shim.lua would shadow ArduPilot's own userdata.
    # A module left in scripts/ by an earlier deployment is still EXECUTED by
    # ArduPilot every boot, and errors out.  Remove strays as well as forbidden
    # files.
    strays = list(FORBIDDEN) + [m for m in MODULES]
    for bad in strays:
        p = os.path.join(dest, bad)
        if os.path.isfile(p):
            os.remove(p)
            log.warning("[lua] removed stray %s from scripts/ - ArduPilot "
                        "executes everything there; modules belong in "
                        "scripts/modules/", bad)
            changed += 1
    for old in RETIRED_ENTRY_SCRIPTS:
        p = os.path.join(dest, old)
        if os.path.isfile(p):
            os.remove(p)
            log.warning("[lua] removed retired entry script %s from scripts/ - "
                        "ArduPilot would run it beside %s", old,
                        ", ".join(ENTRY_SCRIPTS))
            changed += 1

    if changed:
        log.info("[lua] scripts changed - the scripting engine will be reloaded "
                 "in place (no reboot, no sim_vehicle.py restart).")
        log.info("[lua] deployed and verified %d file(s): %s -> %s, "
                 "%d module(s) -> %s  (%d changed - reloaded in place, no reboot)",
                 len(FC_SCRIPTS), ", ".join(ENTRY_SCRIPTS), dest,
                 len(MODULES), mod_dir, changed)
    else:
        log.info("[lua] all %d scripts already up to date in %s",
                 len(FC_SCRIPTS), dest)
    return True, changed > 0, dest


# ---------------------------------------------------------------------------
# MAVLink FTP: the route to a scripts directory that is not a local path.

# SCRIPTING_DIRECTORY at the firmware this project runs (lua_common_defs.h):
# "/APM/scripts" on a board with an SD card or flash filesystem
# (HAL_OS_FATFS_IO / HAL_OS_LITTLEFS_IO), "./scripts" on SITL, whose FTP
# server strips a leading "/" and resolves the rest against its own working
# directory (AP_Filesystem_posix.cpp map_filename).  AP_Scripting::init creates
# it at boot while scripting is enabled, so the one that exists is the one the
# autopilot reads; deploy_ftp() refuses when neither or both do.
FTP_SCRIPT_DIRS = ("/APM/scripts", "scripts")

# FILE_TRANSFER_PROTOCOL's payload as GCS_FTP.cpp packs it: seq u16, session
# u8, opcode u8, size u8, req_opcode u8, burst_complete u8, one pad byte,
# offset u32, then the data.
_FTP_HDR = struct.Struct("<HBBBBBBI")
FTP_PAYLOAD_LEN = 251
FTP_DATA_MAX = FTP_PAYLOAD_LEN - _FTP_HDR.size

# The server keys a session on (sysid, compid, channel, session id), so a
# ground station's own sessions never collide with this one.
FTP_SESSION = 0
# One reply wait, and how many times a request is sent before giving up.  A
# repeated request carries the same seq, and the server answers it by resending
# its last reply rather than running it twice (GCS_FTP::worker).
FTP_REPLY_TIMEOUT_S = 2.0
FTP_ATTEMPTS = 5

_M = mavutil.mavlink


def ap_crc32(data: bytes) -> int:
    """The CRC32 the autopilot reports for a file: crc_crc32() over its bytes
    from 0 (AP_Filesystem::crc32), the standard reflected table with no initial
    or final inversion.  zlib's CRC-32 inverts at both ends, so starting it at
    the inverted 0 and inverting its result gives the same number."""
    return zlib.crc32(data, 0xFFFFFFFF) ^ 0xFFFFFFFF


class FtpError(Exception):
    """A MAVLink FTP request the autopilot refused (NAK) or never answered."""

    def __init__(self, what: str, code: int | None = None,
                 errno_: int | None = None):
        self.code = code
        detail = ("no reply" if code is None else
                  "error %d%s" % (code, "" if errno_ is None else
                                  ", errno %d" % errno_))
        super().__init__("%s: %s" % (what, detail))


class _FtpClient:
    """One FTP session over the link's MAVLink connection.

    The telemetry thread is the connection's only reader, so replies come
    through a subscriber queue (link._subscribe), never a recv of this
    client's own: two readers would each take messages the other needs."""

    def __init__(self, link):
        self._link = link
        self._master = link.master
        self._seq = 0
        self._q = None

    def __enter__(self):
        self._q = self._link._subscribe('FILE_TRANSFER_PROTOCOL', maxsize=0)
        return self

    def __exit__(self, *_exc):
        self._link._unsubscribe(self._q, 'FILE_TRANSFER_PROTOCOL')

    def _request(self, what: str, opcode: int, data: bytes = b"",
                 offset: int = 0) -> bytes:
        """Send one request and return the data of its ACK.  Raises FtpError
        on a NAK or when no reply comes after FTP_ATTEMPTS sends."""
        self._seq = (self._seq + 1) & 0xFFFF
        seq = self._seq
        payload = (_FTP_HDR.pack(seq, FTP_SESSION, opcode, len(data), 0, 0, 0,
                                 offset) + data).ljust(FTP_PAYLOAD_LEN, b"\0")
        m = self._master
        for _ in range(FTP_ATTEMPTS):
            with self._link._mav_lock:
                m.mav.file_transfer_protocol_send(
                    0, m.target_system, m.target_component, list(payload))
            deadline = time.monotonic() + FTP_REPLY_TIMEOUT_S
            while True:
                left = deadline - time.monotonic()
                if left <= 0:
                    break
                msg = self._link._wait_msg(self._q, left)
                if msg is None:
                    break
                if (msg.target_system != m.source_system
                        or msg.target_component != m.source_component):
                    continue
                raw = bytes(msg.payload)
                (r_seq, r_session, r_op, r_size, r_req, _b, _p,
                 _off) = _FTP_HDR.unpack_from(raw)
                if (r_session != FTP_SESSION or r_req != opcode
                        or r_seq != (seq + 1) & 0xFFFF):
                    continue
                body = raw[_FTP_HDR.size:_FTP_HDR.size + r_size]
                if r_op == _M.MAV_FTP_OPCODE_ACK:
                    return body
                code = body[0] if body else None
                errno_ = body[1] if len(body) > 1 else None
                raise FtpError(what, code, errno_)
        raise FtpError(what)

    @staticmethod
    def _path(path: str) -> bytes:
        return path.encode("utf-8")

    def reset(self) -> None:
        """Close any session this connection left open."""
        self._request("reset sessions", _M.MAV_FTP_OPCODE_RESETSESSION)

    def dir_exists(self, path: str) -> bool:
        """True when `path` is a directory the autopilot can list.  An empty
        directory answers end-of-file, which still means it is there."""
        try:
            self._request("list " + path, _M.MAV_FTP_OPCODE_LISTDIRECTORY,
                          self._path(path))
        except FtpError as e:
            if e.code == _M.MAV_FTP_ERR_EOF:
                return True
            if e.code in (_M.MAV_FTP_ERR_FILENOTFOUND, _M.MAV_FTP_ERR_FAILERRNO):
                return False
            raise
        return True

    def crc(self, path: str) -> int | None:
        """The file's CRC32 as the autopilot computes it (ap_crc32), or None
        when there is no such file."""
        try:
            body = self._request("CRC " + path, _M.MAV_FTP_OPCODE_CALCFILECRC,
                                 self._path(path))
        except FtpError as e:
            if e.code == _M.MAV_FTP_ERR_FILENOTFOUND:
                return None
            raise
        return struct.unpack_from("<I", body)[0]

    def mkdir(self, path: str) -> None:
        """Create `path`; one already there is fine."""
        try:
            self._request("mkdir " + path, _M.MAV_FTP_OPCODE_CREATEDIRECTORY,
                          self._path(path))
        except FtpError as e:
            if e.code != _M.MAV_FTP_ERR_FILEEXISTS:
                raise

    def remove(self, path: str) -> bool:
        """Delete the file at `path`.  Returns False when there was none."""
        try:
            self._request("remove " + path, _M.MAV_FTP_OPCODE_REMOVEFILE,
                          self._path(path))
        except FtpError as e:
            if e.code == _M.MAV_FTP_ERR_FILENOTFOUND:
                return False
            raise
        return True

    def rename(self, src: str, dst: str) -> None:
        self._request("rename %s -> %s" % (src, dst), _M.MAV_FTP_OPCODE_RENAME,
                      self._path(src) + b"\0" + self._path(dst))

    def upload(self, path: str, data: bytes) -> None:
        """Write `data` to `path`, created or truncated.  One write in flight
        at a time: each waits for its ACK before the next goes."""
        self._request("create " + path, _M.MAV_FTP_OPCODE_CREATEFILE,
                      self._path(path))
        try:
            for off in range(0, len(data), FTP_DATA_MAX):
                self._request("write %s @%d" % (path, off),
                              _M.MAV_FTP_OPCODE_WRITEFILE,
                              data[off:off + FTP_DATA_MAX], offset=off)
        finally:
            self._request("close " + path, _M.MAV_FTP_OPCODE_TERMINATESESSION)


def deploy_ftp(link, src_dir: str | None = None
               ) -> tuple[bool, bool, str | None]:
    """Upload and verify the FC scripts over MAVLink FTP.

    The same contract as deploy(): returns (ok, changed, dest), the entry
    script in the scripts directory and the modules in its modules/, each
    verified by the CRC32 the autopilot computes, the strays and retired entry
    scripts removed.  Only files whose CRC differs are sent.  When anything
    would change and the aircraft is armed, nothing is changed and it fails:
    the reload new scripts need drops the script's control of the aircraft.

    The telemetry thread must be running: the replies arrive through it."""
    if src_dir is None:
        here = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))))
        src_dir = os.path.join(here, "LUA")
    missing = [f for f in FC_SCRIPTS
               if not os.path.isfile(os.path.join(src_dir, f))]
    if missing:
        log.error("[lua] missing from %s: %s", src_dir, missing)
        return False, False, None
    local = {}
    for name in FC_SCRIPTS:
        with open(os.path.join(src_dir, name), "rb") as fh:
            local[name] = fh.read()

    changed = False
    dest = None
    try:
        with _FtpClient(link) as ftp:
            ftp.reset()
            found = [d for d in FTP_SCRIPT_DIRS if ftp.dir_exists(d)]
            if len(found) != 1:
                if found:
                    log.error("[lua] REFUSING TO DEPLOY over MAVLink FTP: the "
                              "autopilot has both %s - cannot tell which one it "
                              "reads. Nothing changed.", " and ".join(found))
                else:
                    log.error("[lua] the autopilot has no scripts directory (%s). "
                              "It creates one at boot while SCR_ENABLE = 1: set "
                              "it, reboot the FC, and launch again.",
                              " or ".join(FTP_SCRIPT_DIRS))
                return False, False, None
            root = found[0]
            mod_dir = root + "/modules"
            dest = "%s on the autopilot (MAVLink FTP)" % root

            def remote(name):
                return (root if name in ENTRY_SCRIPTS else mod_dir) + "/" + name

            log.info("[lua] comparing the %d scripts with %s",
                     len(FC_SCRIPTS), dest)
            uploads = [n for n in FC_SCRIPTS
                       if ftp.crc(remote(n)) != ap_crc32(local[n])]
            # A module left in scripts/ is executed every boot and errors out; a
            # stale ap_shim.lua shadows the autopilot's own userdata; a retired
            # entry script runs beside the current one.
            strays = [b for b in FORBIDDEN + MODULES + RETIRED_ENTRY_SCRIPTS
                      if ftp.crc(root + "/" + b) is not None]
            if not uploads and not strays:
                log.info("[lua] all %d scripts already up to date in %s",
                         len(FC_SCRIPTS), dest)
                return True, False, dest
            if getattr(link, "armed", False):
                log.error("[lua] REFUSING TO DEPLOY: the aircraft is ARMED and "
                          "the FC's scripts are not this build's (%s); nothing "
                          "changed. Disarm and launch again - the reload that "
                          "makes new scripts run drops the script's control of "
                          "the aircraft.", ", ".join(uploads + strays))
                return False, False, dest

            if any(n in MODULES for n in uploads):
                ftp.mkdir(mod_dir)
            for name in uploads:
                path, data = remote(name), local[name]
                want = ap_crc32(data)
                part = path + ".part"
                t0 = time.monotonic()
                ftp.remove(part)
                ftp.upload(part, data)
                changed = True
                got = ftp.crc(part)
                if got != want:
                    log.error("[lua] %s FAILED verification after upload "
                              "(CRC %s != %08x) - refusing to continue; "
                              "ArduPilot would run a corrupt script", name,
                              "none" if got is None else "%08x" % got, want)
                    ftp.remove(part)
                    return False, True, dest
                # FATFS will not rename onto an existing name.
                ftp.remove(path)
                ftp.rename(part, path)
                got = ftp.crc(path)
                if got != want:
                    log.error("[lua] %s FAILED verification after its rename "
                              "(CRC %s != %08x)", name,
                              "none" if got is None else "%08x" % got, want)
                    return False, True, dest
                log.info("[lua] uploaded and verified %s (%d bytes, %.1f s)",
                         path, len(data), time.monotonic() - t0)
            for bad in strays:
                ftp.remove(root + "/" + bad)
                changed = True
                log.warning("[lua] removed %s from %s - ArduPilot executes "
                            "everything there", bad, root)
    except FtpError as e:
        log.error("[lua] MAVLink FTP failed: %s. %s", e,
                  "Some files changed before it stopped; launch again to "
                  "finish." if changed else "Nothing changed.")
        return False, changed, dest
    log.info("[lua] deployed and verified %d file(s) to %s (%d removed) - the "
             "scripting engine will be reloaded in place",
             len(uploads), dest, len(strays))
    return True, True, dest
