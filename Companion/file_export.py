# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""file_export - copy generated run artefacts from WSL to a Windows folder.

One entry per FILE TYPE the system produces; the Gain Tuner renders one button
per entry (see ui/controller_autotune.py, the "Export to Windows" group).  No
Tk import here - this module is pure discovery + copy so it can be tested and
called headless.

DISCOVERY
    Everything is found relative to the CURRENT WORKING DIRECTORY, because that
    is where every writer puts its output (plant_id.py, plant_freq_id.py,
    constants_suggest.py and controller_autotune_sweep.py all join onto
    os.getcwd()).  There is no source path to configure and therefore none to drift.

    Entries whose names carry a timestamp (plant_id_report_<ts>.txt,
    controller_autotune_<ts>.txt) resolve to the NEWEST match by that stamp.
    The per-cell freq-ID CSVs are a SET written in one run
    (freq_id_cell_<cell>_<ts>.csv), so that one entry copies the newest run's
    whole set - the <cell> tag is not the stamp, the trailing <ts> is.  Whether
    an entry takes the newest file, the newest run's set, or every match is
    declared HERE, per entry - never inferred from the filename.

RETENTION
    A completed run's outputs are KEPT, not overwritten: each id/autotune
    procedure writes a timestamped report (and its raw signals) and does NOT
    claim away the previous run's, so runs accumulate on disk until the operator
    clears them with Delete.  Only the resume CHECKPOINT is claimed away, so at
    most one is ever live.  See CLAUDE.md "Completed-run outputs are kept; only
    the checkpoint is cleaned".

OVERWRITE POLICY
    The destination name is the source name, so re-exporting replaces the
    previous copy.  Note what that means for flight_log.csv specifically:
    CsvLogger.start() opens the log with mode 'w', so each 'q' press already
    truncates the WSL copy, and an export after a new run overwrites the
    Windows copy too.  If a log matters, move it out of the destination folder
    before the next run.

SAFETY
    export() never raises.  It returns (ok, message) for the GUI to display, so
    a mid-flight button press cannot take down the tuner thread or the control
    loop.  Copies happen on the caller's thread - the GUI calls this from a
    worker so a large CSV over DrvFs cannot stall the window.
"""
from __future__ import annotations

import glob
import logging
import os
import re
import shutil
import time

try:
    from .export_paths import EXPORT_DEST_WINDOWS
except ImportError:                                   # pragma: no cover
    EXPORT_DEST_WINDOWS = None

log = logging.getLogger("balloon_chaser")

# Set at launch by main.py to a "Downloaded Files" folder beside main.py, so the
# export destination follows the code.  A hand-set EXPORT_DEST_WINDOWS override
# in export_paths.py wins over it; see destination().
_launch_dest = None

# key, button label, glob pattern, pick, written-by, what the file is
#   pick 'one'    - a single fixed name
#   pick 'latest' - newest match by the stamp in the name (timestamped names)
#   pick 'run'    - the newest RUN's whole set: every match sharing the newest
#                   in-name stamp (a set written in one run that is KEPT, not
#                   overwritten, so several runs' sets can coexist on disk)
#   pick 'all'    - every match on disk, regardless of run
EXPORTS = [
    ("flight_csv", "Flight log CSV", "flight_log_*.csv", "latest",
     "flight logging - the 'q' key",
     "Per-tick flight record: the full control-loop state, one row per control "
     "tick, including the FC's own mirrored internals (the lua_*/fc_* "
     "columns), with the code_sha256 fingerprint and the startup log in the "
     "header. "
     "This is the file every post-flight diagnosis is built from. "
     "NOTE: starting a new recording REPLACES this file rather than appending "
     "to it, so each 'q' press ERASES the previous run - export it before "
     "flying again."),

    ("console_log", "Console log", "balloon_chaser_console_*.log", "latest",
     "the console - saved automatically from launch",
     "Everything the program prints to the console this session: the startup "
     "configuration, parameter readbacks, Lua deploy and scripting-setup "
     "diagnostics, mode changes and any faults. Saved to a file as it is "
     "printed, so the messages that scroll off the terminal can be kept and "
     "shared. One file per launch; the previous run's is cleared at startup. "
     "Delete removes it, and if the program prints anything afterwards a fresh "
     "one begins."),

    ("freqid_ckpt", "Freq-ID checkpoint", "incomplete_freq_id*.json", "latest",
     "frequency-response ID - the 'k' key",
     "Present only while a sweep is INCOMPLETE. Holds the cells already flown "
     "so pressing 'k' again resumes rather than restarting; deleted on a clean "
     "finish. A resumed run refuses to pool cells flown under different "
     "excitation, roll gain or airspeed command, and says which changed."),

    ("freqid_cells", "Freq-ID cell CSVs", "freq_id_cell_*.csv", "run",
     "frequency-response ID - the 'k' key",
     "Raw swept-frequency roll excitation, one CSV per pitch cell ('m' prefix "
     "means negative). Carries the commanded rate, the gyro, the bank proxy "
     "and the yaw rate per sample, which is everything the gain/phase fit "
     "reads. Every 'k' run's set is kept; Copy takes the newest run's whole "
     "set, never a mix, and Delete clears every run on disk."),

    ("freqid_report", "Freq-ID report", "freq_id_report_*.txt", "latest",
     "frequency-response ID - the 'k' key",
     "Human-readable gain and phase for the two measured roll-axis links "
     "against frequency, per cell, plus the climb-vs-dive comparison at "
     "matched frequency."),

    ("freqid_json", "Freq-ID results", "freq_id_results_*.json", "latest",
     "frequency-response ID - the 'k' key",
     "Machine-readable form of the same: per-cell, per-frequency gain, phase, "
     "coherence and the bank-rate identity residual."),

    ("plantid_report", "Plant-ID report", "plant_id_report_*.txt", "latest",
     "plant-ID - the 'i' key",
     "The single human-readable report from an 'i' run. A header naming which axes "
     "ran; then a paste-ready block of the suggested ArduPlane feedforwards "
     "(PTCH_RATE_FF and RLL_RATE_FF - REVIEW before loading, nothing is applied, "
     "and the braking rate-P is left to a later step); then the full run "
     "transcript, where each axis's control power, damping, overshoot floor and "
     "any warnings are recorded. Its content reflects which axes were enabled. "
     "Timestamped, and every completed run is kept - Copy takes the newest, "
     "Delete clears them."),

    ("plantid_csv", "Plant-ID signals", "plant_id_signals_*.csv", "latest",
     "plant-ID - the 'i' key",
     "The single raw-signal CSV from an 'i' run: one row per sample, tagged with "
     "the axis (pitch, yaw or roll), carrying that axis's surface fraction and the "
     "measured body rate for every doublet the run flew. Only the enabled axes "
     "appear. This is what each axis's short-period fit is computed from - re-fit "
     "it offline with fit_pitch_plant. Timestamped, and every completed run is "
     "kept - Copy takes the newest, Delete clears them."),

    ("plantid_run_ckpt", "Plant-ID checkpoint", "incomplete_plant_id_run_*.json", "latest",
     "unified plant-ID ('i'), saved after each completed axis stage",
     "Resume state for an unfinished plant-ID run: which axis stages "
     "(pitch/yaw/roll) are already done. Present only while a run is incomplete - "
     "finishing deletes it. Delete this to make the next 'i' start from the first "
     "enabled axis instead of resuming."),

    ("trimmap_report", "Trim-map report", "trim_map_report_*.txt", "latest",
     "trim map - the 'e' key",
     "The single human-readable report from an 'e' run: the elevator ArduPlane "
     "needs to hold each climb angle at each airspeed (45 = full travel), with "
     "the pitch, airspeed, angle of attack, flight path and throttle actually "
     "flown; the pitch-trim formula fitted to every steady cell (suggested "
     "Pitch Trim Offset, Lift, Attitude and Thrust - REVIEW, nothing is "
     "applied) and what it misses per cell; a paste-ready line per cell for a "
     "fit or a lookup table; then the run transcript. Cells marked ~ were "
     "steady at another airspeed than asked, * not steady, - skipped as out "
     "of reach. Timestamped, and every completed run is kept - Copy takes the "
     "newest, Delete clears them."),

    ("trimmap_log", "Trim-map log", "trim_map_log_*.csv", "latest",
     "trim map - the 'e' key",
     "Every FC tick the 'e' run sampled, tagged with its cell: pitch, roll, "
     "pitch rate, airspeed, angle of attack, flight path, elevator, throttle, "
     "the trim and the pitch-rate command, with in_window marking the samples "
     "a cell's numbers were averaged over. A resumed run appends to the same "
     "file. Timestamped, and every completed run is kept."),

    ("trimmap_ckpt", "Trim-map checkpoint", "incomplete_trim_map_run_*.json", "latest",
     "trim map ('e'), saved after each flown cell",
     "Resume state for an unfinished trim map: every flown cell's result. "
     "Present only while a run is incomplete - finishing deletes it. It is "
     "ignored when the code or the grid has changed since. Delete this to make "
     "the next 'e' start from the first cell instead of resuming."),

    ("autotune_report", "Autotune report", "controller_autotune_*.txt", "latest",
     "controller autotune sweep - the 't' key",
     "Sweep results report plus the paste-ready WINNING SET block. "
     "Timestamped, so the newest sweep is exported."),

    ("suggested_t", "Suggested consts (t)", "suggested_constants_t_*.py", "latest",
     "controller autotune sweep - the 't' key",
     "A copy of the live constants.py with ONLY the winning scalar gains "
     "replaced. Review and copy over constants.py by hand to adopt it."),

    ("autotune_ckpt", "Autotune checkpoint", "incomplete_controller_autotune_*.json",
     "latest", "controller autotune sweep - the 't' key",
     "Resume checkpoint for an IN-PROGRESS sweep, rewritten as it advances and "
     "deleted on completion. Carries the grid signature, so it is abandoned if "
     "the gains change between sessions. Export it to keep a "
     "record of an interrupted sweep."),

    ("cell_report", "Cell autotune report", "cell_autotune_*.txt", "latest",
     "single-cell autotune - the 'y' key",
     "Results report for a single-cell tune plus its paste-ready WINNING SET "
     "block. Names the cell it was flown at and the bracket anchors it "
     "searched, both of which the 'g' copy cannot tell you. Timestamped, so "
     "the newest run is exported."),

    ("suggested_y", "Suggested consts (y)", "suggested_constants_y_*.py", "latest",
     "single-cell autotune - the 'y' key",
     "A copy of the live constants.py with ONLY the winning scalar gains "
     "replaced. These are the best gains AT ONE BEARING and carry no envelope "
     "guarantee - a diagnostic, not a shippable set."),

    ("cell_ckpt", "Cell autotune checkpoint", "incomplete_cell_autotune_*.json",
     "latest", "single-cell autotune - the 'y' key",
     "Resume checkpoint for an IN-PROGRESS single-cell tune. Separate from the "
     "sweep's so the two can never resume each other. Carries the bracket "
     "anchors, which with 'Use Constants' OFF exist nowhere else."),

    ("gui_live", "GUI Live Values", "gui_live_values_*.py", "latest",
     "the Gain Tuner - this Copy button",
     "A copy of the live constants.py with EVERY Gain-Tuner value set to its "
     "CURRENT live setting, and your ACRO values of the ArduPlane parameters "
     "written as the stored Acro defaults (what startup checks, the reset icons "
     "write and Default Acro Parameters sends); the attitude settings, which have "
     "no stored default, are listed in its header. Written fresh each time you press Copy (from the "
     "tuner's current state), then copied out. The timestamp is the moment you "
     "pressed Copy, which for a snapshot is the only moment there is. Review "
     "it, then copy it over constants.py by hand to adopt it."),

    ("gz_world", "Gazebo world (talon_runway)",
     "~/talon_sim/worlds/talon_runway.sdf", "latest",
     "hand-authored - NOT written by this code",
     "The WORLD the aircraft flies in: terrain, runway, lighting, and the "
     "gz-transport topics the companion publishes gimbal commands on. It also "
     "holds the gimbal joint controller's own gains - the servo loop that "
     "physically moves the camera, one layer BELOW anything this codebase "
     "tunes, so a tracking problem that survives every steering gain change "
     "may live here instead. Hand-authored simulator input, not a product of "
     "any run."),

    ("gz_model", "Gazebo model (talon_balloon_chaser)",
     "~/talon_sim/models/talon_balloon_chaser/model.sdf", "latest",
     "hand-authored - NOT written by this code",
     "The aircraft. ONE flat model file carrying everything: masses and "
     "inertias, the lift-drag elements that are the aerodynamics, the aileron, "
     "ruddervator and propeller joints, the gimbal pan/tilt joints whose "
     "ENCODER angles the steering reads (C-02), the vision turret, and the "
     "ArduPilot plugin that carries servo commands out and sensor state back "
     "to SITL. The servo function assignments here decide whether the first "
     "two surfaces are elevons or a separate aileron and elevator, which is "
     "what the HUD's surface demix keys off. "
     "There is no separate airframe file and no wrapper: the world spawns this "
     "model directly, and it is a copy of upstream's mini_talon_vtail, which "
     "is also a single flat model. Geometry, mass, aerodynamics and ArduPilot "
     "plumbing all live in this one file - do not go looking for a second one. "
     "Hand-authored simulator input, not a product of any run."),
]

_BY_KEY = {e[0]: e for e in EXPORTS}


def set_destination(path: str) -> None:
    """Record the folder the export buttons copy into.

    main.py calls this at launch with a "Downloaded Files" folder beside main.py
    (recorded from __file__, so it is where main.py lives regardless of the
    working directory the program was started from).  The folder itself is NOT
    created here: export() makes it on the first Copy that has a file to write,
    so a run that never exports leaves no empty folder behind.
    """
    global _launch_dest
    _launch_dest = path


def destination() -> str | None:
    """The folder the export buttons copy into.

    A hand-set EXPORT_DEST_WINDOWS in export_paths.py wins - a developer forcing
    a fixed path.  Otherwise it is the launch-relative "Downloaded Files" folder
    main.py recorded via set_destination().  None only when neither is set, e.g.
    file_export used without main.py (the tests)."""
    return EXPORT_DEST_WINDOWS or _launch_dest


_PICK_WORDS = {
    "one": "the file of that name",
    "latest": "the NEWEST match (names carry a timestamp)",
    "run": "the NEWEST run's whole set (every match sharing its timestamp)",
    "all": "EVERY match on disk (all runs)",
}


def info_text(key, src_dir=None) -> str:
    """Description for the GUI info button: what the file is, where it lives on
    the WSL side, and where the button copies it to."""
    entry = _BY_KEY.get(key)
    if entry is None:
        return "(no description available)"
    _k, _label, pattern, pick, origin, purpose = entry
    src_dir = src_dir or os.getcwd()
    dest = destination() or "NOT SET - see export_paths.py"
    return (
        "%s\n\n"
        "WRITTEN BY: %s\n\n"
        "LIVES IN: %s\n"
        "  pattern %s - copies %s.\n"
        "  This is the working directory the program was started from; every "
        "generated file is written there, so the button follows it "
        "automatically.\n\n"
        "COPIES TO: %s\n"
        "  Same filename, overwriting any previous copy.\n\n"
        "DELETE removes EVERY working-directory file matching the pattern - not "
        "just the one Copy would take, and not the copy already exported. For a "
        "timestamped entry that is every run still on disk, so the row is empty "
        "afterwards. Two presses, because it cannot be undone.%s%s\n\n"
        "Result is printed to the console, not shown in this window."
        % (purpose, origin, src_dir, pattern, _PICK_WORDS.get(pick, pick), dest,
           ("\n  " + _DELETE_CONSEQUENCE[key][0].upper()
            + _DELETE_CONSEQUENCE[key][1:] + ".") if key in _DELETE_CONSEQUENCE
           else "",
           ("\n  NOTE: Copy WRITES this file fresh before copying it, so pressing "
            "Copy after Delete recreates it. Delete clears what is on disk now; "
            "it does not disable the row.") if key in REGENERATED_ON_COPY else "")
    )


def matches(key, src_dir=None, every=False):
    """Files for the given entry, newest first.  [] = nothing yet.

    Two different questions, and they are NOT the same set for a 'latest' or
    'run' entry:

      every=False  what COPY would take - one file for 'one' and 'latest', the
                   newest run's set for 'run', all of them for 'all'.
      every=True   every file the pattern owns, which is what DELETE clears.

    Deleting only what Copy would take leaves the older matches in place and
    simply PROMOTES the next one to newest, so the row still copies a file
    afterwards - the opposite of what pressing Delete means."""
    entry = _BY_KEY.get(key)
    if entry is None:
        return []
    _k, _label, pattern, pick, _origin, _purpose = entry
    src_dir = src_dir or os.getcwd()
    found = [p for p in glob.glob(os.path.join(src_dir, os.path.expanduser(pattern)))
             if os.path.isfile(p)]
    # Newest by the STAMP IN THE NAME, not by mtime.  The stamp is written once
    # and never changes; mtime is incidental metadata that an editor, an rsync
    # or a cp -p will happily rewrite, which would silently make an older run's
    # file the one Copy takes.  Unstamped files sort by mtime, below every
    # stamped one, so they are only ever picked when nothing stamped exists.
    found.sort(key=lambda p: (stamp_from_name(p) or '', os.path.getmtime(p)),
               reverse=True)
    if every or pick == "all":
        return found
    if pick == "run":
        # A set written in ONE run (the freq-ID cells), identified by the run
        # stamp each name ends with.  Completed runs are KEPT rather than claimed
        # away, so several runs' sets coexist; Copy takes the NEWEST run's whole
        # set - every file sharing the newest stamp - never a mix of two runs.
        # An unstamped file (should not occur for a 'run' entry) falls back to
        # the single newest, matching 'latest'.
        newest = stamp_from_name(found[0]) if found else None
        if newest is None:
            return found[:1]
        return [p for p in found if stamp_from_name(p) == newest]
    return found[:1]


# EVERY entry may be DELETED from the working directory.  Copy puts a file on
# the host side; Delete removes the working-directory original.  That is not
# only housekeeping: 'latest' entries pick the newest match by mtime, so a stale
# report keeps being the one exported until it is removed, and a checkpoint
# keeps being resumed until it is.  Deleting after a Copy is the normal way to
# start a clean run.
# COPY-ONLY.  These are SIMULATOR INPUTS, not outputs of a run: they are hand-
# authored, they live outside the working directory, and deleting one breaks the
# simulation rather than starting a clean run.  Offering Delete on a file this
# code did not write would be offering to destroy the operator's own work.
COPY_ONLY = {"gz_world", "gz_model"}

DELETABLE = {e[0]: e[4] for e in EXPORTS if e[0] not in COPY_ONLY}

# Deleting these changes what the NEXT run DOES, not just what is on disk, so
# the console says so rather than reporting a bare filename.
_DELETE_CONSEQUENCE = {
    "autotune_ckpt": "the next sweep starts FRESH (swept gains reset to "
                     "constants.py) instead of resuming",
    "cell_ckpt":     "the next single-cell tune starts FRESH instead of resuming",
    "plantid_run_ckpt": "the next plant-ID run starts from the first enabled axis "
                        "instead of resuming at the next unfinished one",
}

# WHEN a delete would achieve nothing, or worse.  Checked by the caller before
# it offers to confirm - see the Gain Tuner's Delete button.
#   BUSY_WHILE_RUN      't', 'y' and 'i' all run as mode 'autotune' and rewrite
#                       their outputs as they advance, so the file reappears.
#   BUSY_WHILE_LOGGING  the CSV logger holds flight_log.csv OPEN.  Unlinking an
#                       open file on Linux does not stop the writer - it keeps
#                       filling an inode that no longer has a name, so the log
#                       looks deleted and the run's data is lost with it.
BUSY_WHILE_RUN = {
    "plantid_report", "plantid_csv",
    "plantid_run_ckpt",
    "autotune_report", "suggested_t", "autotune_ckpt",
    "cell_report", "suggested_y", "cell_ckpt",
}
BUSY_WHILE_LOGGING = {"flight_csv"}

# Entries whose Copy button WRITES the file before copying it, so Delete cannot
# leave the row empty - the next Copy recreates it from live state.  Not a bug:
# gui_live_values_<ts>.py is a snapshot of the tuner's CURRENT values and only
# ever meaningful at the instant it is taken.  Called out in the info text so a
# file reappearing after a delete is not a surprise - though each press writes a
# NEW stamp and clears the last, so the row still holds exactly one file.
REGENERATED_ON_COPY = {"gui_live"}


def delete(key, src_dir=None):
    """Delete the working-directory file(s) for one entry.

    Returns (ok, message).  Irreversible, and a checkpoint can represent hours of
    flying, so the caller is expected to confirm first - see the Gain Tuner's
    two-press Delete button."""
    if key not in DELETABLE:
        return _fail("%s is not a known export entry" % key)
    label = _BY_KEY[key][1]
    # every=True: Delete clears the row, not just the one file Copy would take.
    found = matches(key, src_dir, every=True)
    if not found:
        log.info("[EXPORT] no %s file to delete - nothing there.", label)
        return True, "nothing to delete"
    gone = []
    for p in found:
        try:
            os.remove(p)
            gone.append(os.path.basename(p))
        except OSError as exc:
            return _fail("could not delete %s: %s" % (p, exc))
    tail = _DELETE_CONSEQUENCE.get(key)
    # Name every file.  A timestamped entry can hold several runs' worth, and an
    # operator who expected to remove one should see that they removed four.
    log.warning("[EXPORT] DELETED %s - %d file(s): %s.%s", label, len(gone),
                ", ".join(gone), ("  Consequence: %s." % tail) if tail else "")
    return True, "deleted " + ", ".join(gone)


ALL_KEY = "__all__"          # not a real entry; the Delete All row's handle


def run_stamp():
    """The timestamp a procedure brands its output files with.

    Taken ONCE when the procedure starts and carried to every file it writes,
    so a plant-ID run that takes twenty minutes names its report, its JSON, its
    cell CSVs and its suggested-constants file with the SAME moment - the moment
    the flying began.  Stamping at write time instead would give one run several
    different timestamps and would date the report by when it finished, which is
    not the question anyone asks of a filename."""
    return time.strftime('%Y%m%d_%H%M%S')


def stamp_display(ts):
    """'20260811_022200' -> '2026-08-11 02:22:00', for headers inside files.

    The compact form is for filenames, this one is for reading.  Both live here
    so the layout is known in one place and they cannot drift apart."""
    try:
        return time.strftime('%Y-%m-%d %H:%M:%S',
                             time.strptime(str(ts), '%Y%m%d_%H%M%S'))
    except (ValueError, TypeError):
        return str(ts)


def stamp_from_name(name):
    """Pull the run stamp back out of a stamped filename, or None.

    Lets a file's HEADER take its timestamp from its own NAME rather than from a
    second clock reading, so the two can never disagree - and no caller has to
    pass the stamp twice."""
    m = re.search(r'_(\d{8}_\d{6})\.[A-Za-z0-9]+$', str(name))
    return m.group(1) if m else None


def stamped(base, ext, ts):
    """'plant_id_results', 'json', '20260811_022200' -> the stamped filename."""
    return "%s_%s.%s" % (base, ts, ext.lstrip("."))


def claim(pattern, keep, src_dir=None):
    """Delete every file matching `pattern` except the ones in `keep`.

    A cleanup tool for the outputs that keep only their LATEST copy - the resume
    checkpoints, the per-launch console log, the 'q' flight log.  Called right
    after the new file is written, so the replacement is already in place when
    its predecessor goes; a crash between the two leaves a duplicate, never a
    gap.

    COMPLETED-RUN outputs (the id/autotune reports and their raw signals)
    deliberately do NOT call this: they are timestamped and KEPT so several runs
    accumulate for the operator to compare or export, cleared only by Delete.
    See CLAUDE.md "Completed-run outputs are kept; only the checkpoint is
    cleaned".

    Returns the list of paths removed, for the caller to log."""
    keep = {os.path.abspath(k) for k in ([keep] if isinstance(keep, str) else keep)}
    removed = []
    for p in glob.glob(os.path.join(src_dir or os.getcwd(), os.path.expanduser(pattern))):
        if not os.path.isfile(p) or os.path.abspath(p) in keep:
            continue
        try:
            os.remove(p)
            removed.append(os.path.basename(p))
        except OSError as exc:
            log.warning("[EXPORT] could not clear the previous %s: %s",
                        os.path.basename(p), exc)
    return removed


def start_console_log(src_dir=None):
    """Open this run's console-log file and return (handler, path).

    main.py attaches the handler to the package loggers at launch, so every line
    the program prints to the console is also saved to balloon_chaser_console_<stamp>.log
    in the working directory - exportable and deletable from the Gain Tuner like
    any other output.

    WatchedFileHandler, not FileHandler: if Delete removes the file mid-session
    the handler recreates it on the next record rather than going on filling an
    unlinked inode that has no name - the open-file trap the CSV logger avoids by
    refusing Delete while it holds flight_log.csv open.  One file per launch; the
    previous run's is claimed away so they do not pile up.
    """
    import logging.handlers
    src_dir = src_dir or os.getcwd()
    path = os.path.join(src_dir, stamped('balloon_chaser_console', 'log', run_stamp()))
    for old in claim('balloon_chaser_console_*.log', path, src_dir):
        log.info("[LOG] replaced the previous console log %s", old)
    handler = logging.handlers.WatchedFileHandler(path, mode='a', encoding='utf-8')
    return handler, path


def delete_all(src_dir=None):
    """Delete the working-directory files for EVERY export entry.

    ONE console line, not twelve.  The operator pressed one button and wants one
    answer, so entries that had nothing are not mentioned - the line is a
    receipt of what actually went, not a directory listing.

    All-or-nothing is the CALLER's job: this removes whatever it can and reports
    per-file failures, because a partial result the operator can read beats a
    refusal that leaves them guessing which half is gone.  The Gain Tuner checks
    the busy conditions BEFORE offering to confirm, so by the time this runs
    nothing should be holding a file open."""
    gone_by_label, failed, n = [], [], 0
    for key, label, _pattern, _pick, _origin, _purpose in EXPORTS:
        # Honour the same guard the per-row delete() does: COPY_ONLY entries are
        # hand-authored simulator INPUTS (the world and the model), not a run's
        # output. Their patterns are absolute (~/talon_sim/...), so deleting them
        # reaches OUTSIDE the working directory and breaks the simulation - the one
        # button that clears everything must not destroy what the per-row Delete
        # refuses to touch.
        if key not in DELETABLE:
            continue
        found = matches(key, src_dir, every=True)
        if not found:
            continue
        gone = []
        for p in found:
            try:
                os.remove(p)
                gone.append(os.path.basename(p))
            except OSError as exc:
                failed.append("%s (%s)" % (os.path.basename(p), exc))
        if gone:
            n += len(gone)
            gone_by_label.append("%s x%d" % (label, len(gone)))
    if not n and not failed:
        log.info("[EXPORT] DELETE ALL: nothing to delete - the working directory "
                 "is already clear.")
        return True, "nothing to delete"
    # Name the checkpoints separately: they are the only entries whose removal
    # changes what the NEXT run does rather than just what is on disk.
    tail = "".join(
        "  Consequence: %s." % _DELETE_CONSEQUENCE[k]
        for k in _DELETE_CONSEQUENCE
        if any(_BY_KEY[k][1] in g for g in gone_by_label))
    log.warning("[EXPORT] DELETE ALL removed %d file(s): %s.%s",
                n, "; ".join(gone_by_label), tail)
    if failed:
        return _fail("DELETE ALL could not remove: " + ", ".join(failed))
    return True, "deleted %d file(s)" % n


def _copy_one(src, dest_dir):
    """copy2 first so the mtime survives; fall back to a contents-only copy
    because copystat can raise on some DrvFs mounts."""
    dest = os.path.join(dest_dir, os.path.basename(src))
    try:
        shutil.copy2(src, dest)
    except (PermissionError, OSError):
        shutil.copyfile(src, dest)
    return dest


def _fail(msg):
    """Every failure path logs to the CONSOLE and returns the reason.  The GUI
    deliberately shows nothing - see the Gain Tuner GUI conventions in
    CLAUDE.md - so this log line is the only report the operator gets."""
    log.warning("[EXPORT] %s", msg)
    return False, msg


def export(key, src_dir=None, dest_dir=None, flush_hook=None):
    """Copy one entry's file(s) to the host-OS folder.

    flush_hook: optional zero-arg callable invoked before copying, so a live
    flight_log.csv is flushed and the copy is not missing its tail.

    Returns (ok, message) and logs the outcome to the console.  Never raises.
    """
    entry = _BY_KEY.get(key)
    if entry is None:
        return _fail("unknown export '%s'" % key)
    _k, label, pattern, _pick, _origin, _purpose = entry

    dest_dir = dest_dir or destination()
    if not dest_dir:
        return _fail("%s: no destination set (see export_paths.py)" % label)

    if flush_hook is not None:
        try:
            flush_hook()
        except Exception:                             # noqa: BLE001
            log.exception("[EXPORT] flush hook failed (copying anyway)")

    src_dir = src_dir or os.getcwd()
    files = matches(key, src_dir)
    if not files:
        return _fail("%s: nothing to copy (no %s yet)" % (label, pattern))

    try:
        os.makedirs(dest_dir, exist_ok=True)
    except OSError as e:
        return _fail("%s: destination unavailable (%s)" % (label, e))

    copied = []
    for src in files:
        try:
            copied.append(os.path.basename(_copy_one(src, dest_dir)))
        except OSError as e:
            return _fail("%s: copy failed after %d file(s) (%s)" % (
                label, len(copied), e))
        except Exception as e:                        # noqa: BLE001
            return _fail("%s: copy failed (%s)" % (label, e))

    log.info("[EXPORT] %s -> %s (%d file(s): %s)",
             label, dest_dir, len(copied), ", ".join(copied))
    if len(copied) == 1:
        return True, "copied %s" % copied[0]
    return True, "copied %d files" % len(copied)
