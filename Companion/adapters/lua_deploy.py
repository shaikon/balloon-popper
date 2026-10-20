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

WHY A PLAIN FILE COPY AND NOT MAVLINK FTP
  In SITL the autopilot and the companion share a filesystem, so the scripts
  directory is a local path and shutil is both simpler and more verifiable than
  an FTP transfer.  On real hardware the SD card is not reachable this way and
  MAVLink FTP (or pulling the card) is required - deploy() says so explicitly
  rather than silently appearing to succeed.

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
