# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
"""The file_export contract: the Gazebo sources, completed-run retention, and
the simulator's air.

Four properties that are easy to get wrong:

  1. The Gazebo sources live OUTSIDE the working directory, so the pattern is
     home-relative.  os.path.join does NOT treat "~" as absolute - it prepends
     the cwd, and a later expanduser() then operates on an already-wrong path.
     Expansion must happen on the PATTERN, before the join.

  2. DELETABLE was built from every EXPORTS row, so adding a row silently added
     a Delete button.  These are simulator INPUTS the operator hand-authored;
     offering to delete one is offering to break the simulation.

  3. A completed id/autotune run is KEPT, not overwritten.  A new run may clean
     only its resume CHECKPOINT, never a previous run's report or raw signals -
     so the completed-output writers must not claim() their output away, and the
     export must copy the newest RUN's set without ever mixing two runs.

  4. The simulated wing and propeller fly in the air ArduPilot's airspeed
     reading assumes.  Stock LiftDrag holds one fixed <air_density> at every
     height while SITL's reading thins with height, so a fit made at one height
     is wrong at another.  Every lift-drag element must be AltitudeLiftDrag,
     whose atmosphere is ArduPilot's own table, with no fixed density, in a
     world that states its elevation.
"""
import glob, os, sys, tempfile
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from Companion import file_export as fe

fails = []
def check(n, c, d=""):
    print(f"  [{'PASS' if c else 'FAIL'}] {n}" + (f"   {d}" if d else ""))
    if not c: fails.append(n)

print("\n" + "=" * 70); print("GAZEBO SOURCE EXPORT"); print("=" * 70)

keys = {e[0] for e in fe.EXPORTS}
for k, label in (('gz_world', 'Gazebo world (talon_runway)'),
                 ('gz_model', 'Gazebo model (talon_balloon_chaser)')):
    e = next((x for x in fe.EXPORTS if x[0] == k), None)
    check(f"{k} registered", e is not None)
    if e is None:
        continue
    check(f"    labelled '{label}'", e[1] == label, e[1])
    check("    COPY ONLY - no Delete", k not in fe.DELETABLE,
          "these are hand-authored simulator inputs")
    check("    has info text", bool(fe.info_text(k)))
    check("    pattern is home-relative", e[2].startswith('~/'), e[2])

print("\n  resolution against a fake home:")
home = tempfile.mkdtemp()
os.makedirs(os.path.join(home, 'talon_sim', 'worlds'))
os.makedirs(os.path.join(home, 'talon_sim', 'models', 'talon_balloon_chaser'))
open(os.path.join(home, 'talon_sim', 'worlds', 'talon_runway.sdf'), 'w').write('w')
open(os.path.join(home, 'talon_sim', 'models', 'talon_balloon_chaser', 'model.sdf'), 'w').write('m')
_old = os.environ.get('HOME')
os.environ['HOME'] = home
try:
    for k in ('gz_world', 'gz_model'):
        pat = next(x[2] for x in fe.EXPORTS if x[0] == k)
        # exactly what the exporter does
        hits = glob.glob(os.path.join(os.getcwd(), os.path.expanduser(pat)))
        check(f"    {k} resolves", bool(hits),
              os.path.basename(hits[0]) if hits else
              "expanduser must run on the PATTERN, before the join")
    # DELETE ALL must not reach these. Their patterns are absolute, so the working
    # directory it is given cannot shield them - only the COPY_ONLY guard can. This
    # is the button that pressing once removes the hand-authored world and model,
    # not just the per-row Delete that already refuses them.
    fe.delete_all(src_dir=home)
    world = os.path.join(home, 'talon_sim', 'worlds', 'talon_runway.sdf')
    model = os.path.join(home, 'talon_sim', 'models', 'talon_balloon_chaser', 'model.sdf')
    check("    DELETE ALL leaves the world in place", os.path.exists(world),
          "delete_all must skip COPY_ONLY or it destroys simulator input")
    check("    DELETE ALL leaves the model in place", os.path.exists(model),
          "delete_all must skip COPY_ONLY or it destroys simulator input")
finally:
    if _old is not None:
        os.environ['HOME'] = _old

print("\n  every OTHER export still deletable:")
n_del = sum(1 for e in fe.EXPORTS if e[0] in fe.DELETABLE)
check(f"    {n_del} of {len(fe.EXPORTS)} rows have Delete",
      n_del == len(fe.EXPORTS) - len(fe.COPY_ONLY),
      f"COPY_ONLY = {sorted(fe.COPY_ONLY)}")

print("\n  ONE model row, and it says the airframe is FLAT:")
# There must be exactly one aircraft SDF row. A duplicate row is not harmless:
# it tells a reader a second model file exists, so when something in the model
# looks wrong the natural conclusion is "I must be editing the wrong file" -
# when in fact everything lives in the one flat talon_balloon_chaser model.
_model = [e for e in fe.EXPORTS if e[2].endswith('models/talon_balloon_chaser/model.sdf')]
check("    exactly one row for the aircraft model", len(_model) == 1,
      f"rows pointing there: {[e[0] for e in _model]}")
_bare = fe.info_text('gz_model')
check("    flat structure is stated", 'flat' in _bare.lower(),
      "a reader must not go looking for a wrapper model that does not exist")
check("    it names the model", 'talon_balloon_chaser' in _bare)

print("\n  no info text claims a file wraps or includes another:")
# The stale text passed a substring check for 'flat' while the next sentence
# said "this is the wrapper. The OTHER one ... is the bare airframe it
# includes". Assert on the contradiction directly, not on a keyword.
for _e in fe.EXPORTS:
    _t = fe.info_text(_e[0]).lower()
    _bad = [w for w in ('wraps this one', 'it includes', 'is the wrapper')
            if w in _t]
    check(f"    {_e[0]}", not _bad, f"claims a second file: {_bad}")

print("\n  no two rows copy the same file:")
import collections as _c
_p = _c.defaultdict(list)
for _e in fe.EXPORTS:
    if _e[2].startswith('~') or '/' in _e[2]:
        _p[_e[2]].append(_e[0])
_dups = {k: v for k, v in _p.items() if len(v) > 1}
check("    every literal source path is unique", not _dups, f"{_dups}")

print("\n  INFO TEXT HYGIENE (all rows):")
import re as _re
# A value quoted in prose goes stale silently, and most of these are TUNABLE -
# the info button must explain what a file is FOR, not assert a number.
_NUM = _re.compile(r'(?<![\w.])(\d+(?:\.\d+)?)\s*(Hz|deg|m/s|ms|%)(?![\w])')
_vals = [k for k, _l, _p, _pk, _o, i in fe.EXPORTS if _NUM.search(i)]
check("    no info text quotes a value", not _vals, str(_vals))
# Only rows that HAVE a Delete button may talk about deleting.
_del = [k for k, _l, _p, _pk, _o, i in fe.EXPORTS
        if k in fe.COPY_ONLY and 'elet' in i]
check("    copy-only rows never mention Delete", not _del, str(_del))
# Operator-facing text should not use implementation jargon.
_jarg = [k for k, _l, _p, _pk, _o, i in fe.EXPORTS
         if "'w' mode" in i or 'w mode' in i or 'glob' in i]
check("    no file-mode / implementation jargon", not _jarg, str(_jarg))

print("\n" + "=" * 70); print("COMPLETED-RUN RETENTION"); print("=" * 70)
# The contract: a new id/autotune run removes only its resume CHECKPOINT, never
# a completed run's report or raw signals.  Guarded two ways - the export copies
# the newest RUN's set (never a mix), and the writers do not claim their output.

print("\n  the export keeps every run and copies the newest cleanly:")
ret_home = tempfile.mkdtemp()
runs = ['20260101_000000', '20260101_010000']          # oldest, newest
for ts in runs:
    # A freq-ID run is a SET of per-cell CSVs, all stamped with the run ts.
    for tag in ('25', 'm10', '0'):
        open(os.path.join(ret_home, f'freq_id_cell_{tag}_{ts}.csv'), 'w').write('x')
    # plant-ID and sweep each leave one timestamped report per run.
    open(os.path.join(ret_home, f'plant_id_report_{ts}.txt'), 'w').write('r')
newest = runs[-1]
copy_cells = sorted(os.path.basename(p) for p in fe.matches('freqid_cells', src_dir=ret_home))
want_cells = sorted(f'freq_id_cell_{t}_{newest}.csv' for t in ('25', 'm10', '0'))
check("    Copy takes the newest run's WHOLE cell set", copy_cells == want_cells, str(copy_cells))
check("    ...and only that run - no mixing", all(newest in b for b in copy_cells))
check("    Delete (every=True) still clears every run's cells",
      len(fe.matches('freqid_cells', src_dir=ret_home, every=True)) == 6)
copy_rep = [os.path.basename(p) for p in fe.matches('plantid_report', src_dir=ret_home)]
check("    Copy takes the newest report", copy_rep == [f'plant_id_report_{newest}.txt'], str(copy_rep))
check("    Delete clears all reports",
      len(fe.matches('plantid_report', src_dir=ret_home, every=True)) == 2)

print("\n  the writers do NOT claim a completed output away:")
_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
def _src(mod):
    return open(os.path.join(_root, 'Companion', mod + '.py')).read()
# Each entry: module, the claim() calls that MUST be gone (a completed output).
removed = {
    'plant_id':                 ["claim('plant_id_signals_*.csv'", "claim('plant_id_report_*.txt'"],
    'plant_freq_id':            ["claim('freq_id_cell", "claim('freq_id_report_*.txt'",
                                 "claim('freq_id_results_*.json'"],
    'constants_suggest':        ["_fx.claim("],
    'controller_autotune_sweep':["file_export.claim("],
    'trim_map':                 ["claim('trim_map_log", "claim('trim_map_report",
                                 "claim(\"trim_map_log", "claim(\"trim_map_report"],
}
for mod, gone in removed.items():
    s = _src(mod)
    live = [c for c in gone if c in s]
    check(f"    {mod}: no completed-output claim()", not live, str(live))
# The ONE claim that must SURVIVE: the plant-ID run checkpoint is the thing a new
# run is allowed to clean.  (freq-ID and the sweep clean theirs by fixed-name
# overwrite + clear, so they have no claim() to keep.)
check("    plant_id: the run CHECKPOINT is still claimed",
      "claim(_RUN_CKPT_GLOB" in _src('plant_id'),
      "only the checkpoint may be cleaned on a new run")
check("    trim_map: the run CHECKPOINT is still claimed",
      "claim(_CKPT_GLOB" in _src('trim_map'),
      "only the checkpoint may be cleaned on a new run")

print("\n" + "=" * 70); print("SIMULATOR AIR"); print("=" * 70)
import xml.etree.ElementTree as _ET
_sdf_dir = os.path.join(_root, 'Arduplane', 'SDF')
_ald_dir = os.path.join(_root, 'Arduplane', 'AltitudeLiftDrag')
_model = _ET.parse(os.path.join(_sdf_dir, 'model.sdf')).getroot()
_plugins = list(_model.iter('plugin'))
_aero = [pl for pl in _plugins if 'LiftDrag' in (pl.get('name') or '')]

print("\n  every lift-drag element flies in air that thins with height:")
_stock = [pl.get('name') for pl in _aero
          if (pl.get('filename'), pl.get('name'))
          != ('AltitudeLiftDrag', 'talon_sim::AltitudeLiftDrag')]
check("    no stock LiftDrag left in model.sdf", not _stock, str(_stock))
check("    all seven elements (propeller x2, wings, V-tail, fuselage side)",
      len(_aero) == 7, str(len(_aero)))
_fixed = [pl.findtext('link_name') for pl in _plugins
          if pl.find('air_density') is not None]
check("    no element carries a fixed <air_density>", not _fixed, str(_fixed))
_world = _ET.parse(os.path.join(_sdf_dir, 'talon_runway.sdf')).getroot()
_elev = _world.findtext('./world/spherical_coordinates/elevation')
try:
    float(_elev)
    _elev_ok = True
except (TypeError, ValueError):
    _elev_ok = False
check("    the world states its <spherical_coordinates><elevation>",
      _elev_ok, str(_elev))

print("\n  the plugin is the one the SDF names, in ArduPilot's atmosphere:")
_cc = open(os.path.join(_ald_dir, 'AltitudeLiftDrag.cc')).read()
_cm = open(os.path.join(_ald_dir, 'CMakeLists.txt')).read()
check("    CMake builds libAltitudeLiftDrag (the SDF's filename)",
      'add_library(AltitudeLiftDrag SHARED' in _cm)
check("    the library registers the SDF's plugin name",
      '"talon_sim::AltitudeLiftDrag"' in _cc)
# AP_Baro_atmosphere.cpp at the firmware this project runs (2d99a5ea): the
# 1976 table as (base height m, temperature K, density kg/m^3, lapse K/m) and
# the three constants get_air_density_for_alt_amsl() uses.
_AP_1976 = [(-5000, 320.650, 1.930467, -6.5e-3), (11000, 216.650, 0.363918, 0),
            (20000, 216.650, 8.80349e-2, 1e-3), (32000, 228.650, 1.32250e-2, 2.8e-3),
            (47000, 270.650, 1.42753e-3, 0), (51000, 270.650, 8.61606e-4, -2.8e-3),
            (71000, 214.650, 6.42110e-5, -2.0e-3), (84852, 186.946, 6.95788e-6, 0)]
_AP_CONST = {'kEarthRadiusM': 6356.766e3, 'kAirGasConstant': 287.053072,
             'kStandardGravity': 9.80665}
_tbl = _re.search(r'kAtmosphere1976\[\] = \{(.*?)\};', _cc, _re.S)
_rows = [tuple(float(v) for v in r.split(','))
         for r in _re.findall(r'\{([^{}]*)\}', _tbl.group(1))] if _tbl else []
check("    the 1976 table is ArduPilot's, row for row",
      _rows == [tuple(float(v) for v in r) for r in _AP_1976], str(_rows))
for _name, _want in _AP_CONST.items():
    _m = _re.search(r'constexpr double ' + _name + r' = ([^;]+);', _cc)
    check(f"    {_name} is ArduPilot's",
          bool(_m) and float(_m.group(1)) == _want,
          _m.group(1) if _m else 'missing')

print("\n" + "=" * 70)
print("PASSED" if not fails else f"FAILED: {fails}"); print("=" * 70)
sys.exit(1 if fails else 0)
