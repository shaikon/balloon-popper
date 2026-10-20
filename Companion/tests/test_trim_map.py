# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
"""The trim map ('e'): grid, one-tick sampling, the steady test, the fit, the
report, the checkpoint, and the whole run flown against a fake aircraft.

The fake aircraft holds whatever airspeed the run's PRIVATE gains command and
needs an elevator of TRUE_LEVEL + TRUE_SLOPE * sin(pitch), so a run that reads
the wrong tick, sets the wrong gains or fits the wrong quantity cannot
recover those two numbers.
"""
import math, os, sys, tempfile, threading, types
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from Companion import protocol as P
from Companion import trim_map as T
from Companion import constants as C
import Companion.testing.synthetic_injector as _inj_mod

fails = []
def check(n, c, d=""):
    print(f"  [{'PASS' if c else 'FAIL'}] {n}" + (f"   {d}" if d else ""))
    if not c: fails.append(n)

print("\n" + "=" * 70); print("TRIM MAP ('e')"); print("=" * 70)

# ---------------------------------------------------------------------------
print("\n  the grid:")
climbs = T._axis(C.TRIM_MAP_CLIMB_MIN_DEG, C.TRIM_MAP_CLIMB_MAX_DEG, C.TRIM_MAP_CLIMB_STEP_DEG)
speeds = T._axis(C.TRIM_MAP_SPEED_MIN_MPS, C.TRIM_MAP_SPEED_MAX_MPS, C.TRIM_MAP_SPEED_STEP_MPS)
cells = T.cells_of(climbs, speeds)
check("    the default grid is 8 climb angles x 4 airspeeds", len(cells) == 32,
      "%d x %d" % (len(climbs), len(speeds)))
check("    both ends are included", climbs[0] == -45.0 and climbs[-1] == 60.0
      and speeds[0] == 15.0 and speeds[-1] == 21.0, "%s %s" % (climbs, speeds))
check("    every cell is flown exactly once", len(set(cells)) == 32)
check("    climbs and dives alternate, level last",
      T.fly_order([-45, -30, -15, 0, 15, 30, 45, 60]) == [60, -45, 45, -30, 30, -15, 15, 0],
      str(T.fly_order([-45, -30, -15, 0, 15, 30, 45, 60])))
check("    a step of 0 flies the minimum alone", T._axis(14, 20, 0) == [14.0])

# ---------------------------------------------------------------------------
def frames(pitch_deg=0.0, roll_deg=0.0, eas=16.0, elev=10.0, q_dps=0.0, t_ms=1000,
           gate=0, asp_sat=False, seq=1, throttle=60.0, eas2tas=1.0, lift_term=None):
    """A (STATE, STATE2) pair as the link decodes it, for the given aircraft.
    The trim's lift term is the cascade's, cos(pitch), unless one is given."""
    st = {f.name: (0.0 if f.fmt == "f" else 0) for f in P.STATE}
    st2 = {f.name: (0.0 if f.fmt == "f" else 0) for f in P.STATE2}
    th, ph = math.radians(pitch_deg) / 2.0, math.radians(roll_deg) / 2.0
    # roll then pitch, no yaw
    st2.update(q_w=math.cos(ph) * math.cos(th), q_x=math.sin(ph) * math.cos(th),
               q_y=math.cos(ph) * math.sin(th), q_z=-math.sin(ph) * math.sin(th),
               gyro_y_rads=math.radians(q_dps), eas_mps=eas, out_elevator=elev * 100.0,
               out_throttle=throttle, eas2tas=eas2tas, aoa_deg=-4.0,
               vel_n_mps=eas * math.cos(math.radians(pitch_deg)),
               vel_d_mps=-eas * math.sin(math.radians(pitch_deg)), seq=seq)
    st.update(seq=seq, fc_millis=t_ms, gate=gate, asp_flags=(2 if asp_sat else 0) | 1,
              pitch_trim_elev=5.0, omega_y_cmd_dps=1.5,
              pitch_trim_lift_term=(math.cos(math.radians(pitch_deg)) if lift_term is None
                                    else lift_term))
    return st, st2

print("\n  one FC tick:")
s = T.sample_from_frames(frames(pitch_deg=30.0, roll_deg=10.0, elev=13.8, t_ms=2500))
check("    pitch and roll come from the tick's quaternion",
      abs(s["pitch_deg"] - 30.0) < 1e-6 and abs(s["roll_deg"] - 10.0) < 1e-6,
      "pitch %.4f roll %.4f" % (s["pitch_deg"], s["roll_deg"]))
check("    the elevator is in the trim formula's units (45 = full)", abs(s["elevator"] - 13.8) < 1e-9)
check("    the tick is stamped with the FC clock", s["t_s"] == 2.5)
check("    the tick carries the FC's EAS2TAS",
      T.sample_from_frames(frames(eas2tas=1.15))["eas2tas"] == 1.15)
_f = frames(); _f[1]["eas_mps"] = float("nan")
check("    a tick with no airspeed is not a sample", T.sample_from_frames(_f) is None)
check("    the throttle-saturation bit means nothing off gate 0",
      T.sample_from_frames(frames(gate=5, asp_sat=True))["asp_sat"] is None)

# ---------------------------------------------------------------------------
print("\n  what counts as steady:")
def window(n=160, **kw):
    return [T.sample_from_frames(frames(t_ms=1000 + 20 * i, **{k: (v(i) if callable(v) else v)
                                                             for k, v in kw.items()}))
            for i in range(n)]
check("    a steady window passes", T.steady_why_not(window(), 16.0) is None)
check("    a window shorter than STEADY_S does not", T.steady_why_not(window(n=50), 16.0)
      == "window too short")
check("    a rotating nose does not", T.steady_why_not(window(q_dps=2.0), 16.0) == "nose rotating")
check("    a drifting pitch does not",
      T.steady_why_not(window(pitch_deg=lambda i: 0.01 * i), 16.0) == "pitch drifting")
check("    a steady bank inside ROLL_TOL_DEG passes (1.5% more lift at 10 deg)",
      T.steady_why_not(window(roll_deg=8.0), 16.0) is None)
check("    banked wings past it do not", T.steady_why_not(window(roll_deg=11.0), 16.0)
      == "wings not level")
check("    an airspeed off goal does not", T.steady_why_not(window(eas=17.0), 16.0)
      == "airspeed off goal")
check("    a drifting airspeed does not",
      T.steady_why_not(window(eas=lambda i: 15.7 + 0.004 * i), 16.0) == "airspeed drifting")
check("    an elevator on its stop does not",
      T.steady_why_not(window(elev=44.5), 16.0) == "elevator on its stop")
check("    steady at another airspeed is still steady (a trim measurement)",
      T.steady_why_not(window(eas=17.0)) is None)
check("    but not while that airspeed drifts",
      T.steady_why_not(window(eas=lambda i: 15.7 + 0.004 * i)) == "airspeed drifting")

print("\n  when a cell stops on airspeed:")
_acc = window(n=100, eas=lambda i: 15.5 + 0.02 * i, asp_sat=True)      # 15.5 -> 17.5, rising
check("    accelerating toward a far goal, throttle pinned, does not stop it",
      T.speed_stop(_acc, 21.0, sat_s=10.0) is None)
_dec = window(n=100, eas=lambda i: 18.0 - 0.02 * i, asp_sat=True)      # 18 -> 16, falling
check("    below the goal and falling stops it (the stall guard)",
      (T.speed_stop(_dec, 21.0, sat_s=0.0) or "").startswith("underspeed"))
_run = window(n=100, eas=lambda i: 20.0 + 0.05 * i)                    # 20 -> 25, rising
check("    above the goal and rising stops it (a dive running away)",
      (T.speed_stop(_run, 13.0, sat_s=0.0) or "").startswith("overspeed"))
_flat = window(n=100, eas=17.5, asp_sat=True)
check("    pinned throttle with the gap not closing stops it after SAT_GIVEUP_S",
      T.speed_stop(_flat, 19.0, sat_s=T.SAT_GIVEUP_S - 0.1) is None
      and (T.speed_stop(_flat, 19.0, sat_s=T.SAT_GIVEUP_S) or "").startswith("airspeed unreachable"))

# ---------------------------------------------------------------------------
TRUE = {"pitch_trim_offset": 2.0, "pitch_trim_lift": 16.0,
        "pitch_trim_attitude": -18.0, "pitch_trim_thrust": -7.0}
def elev_needed(pitch_deg, eas, throttle, e2t, lift=None):
    return sum(TRUE[g] * x for g, x in zip(T.TRIM_GAINS,
                                           T.trim_terms(pitch_deg, eas, throttle, e2t, lift)))

# The cells are flown at different heights, as a run that climbs back to the
# protection's climb-to and dives many times is: each has its own EAS2TAS.
def cell(climb, speed, pitch, eas, thr, status="held", steady=True, elev=None, e2t=1.0,
         lift=None):
    r = T.summarize(window(pitch_deg=pitch, eas=eas, throttle=thr, eas2tas=e2t, lift_term=lift,
                           elev=elev_needed(pitch, eas, thr, e2t, lift) if elev is None else elev),
                    climb, speed, status)
    r["steady"] = steady
    return r

print("\n  the trim formula, fitted:")
t = T.trim_terms(30.0, T.C.PITCH_TRIM_VREF_MPS, 50.0, 1.0)
check("    at VREF and sea-level air the terms are 1, cos, sin and the throttle fraction",
      abs(t[1] - math.cos(math.radians(30))) < 1e-12
      and abs(t[2] - math.sin(math.radians(30))) < 1e-12 and abs(t[3] - 0.5) < 1e-12)
t_thin = T.trim_terms(30.0, T.C.PITCH_TRIM_VREF_MPS, 50.0, 1.2)
check("    thinner air divides the throttle term by EAS2TAS^2 and nothing else",
      t_thin[:3] == t[:3] and abs(t_thin[3] - 0.5 / 1.44) < 1e-12, str(t_thin))
res = [cell(0, 14.0, -3.0, 14.0, 60.0, e2t=1.03), cell(30, 14.0, 26.0, 14.0, 75.0, e2t=1.10),
       cell(60, 14.0, 55.0, 14.0, 90.0, e2t=1.19), cell(-30, 14.0, -34.0, 14.0, 45.0, e2t=1.05),
       cell(0, 18.0, -5.0, 18.0, 92.0, e2t=1.17), cell(30, 18.0, 25.0, 17.2, 100.0,
                                                        status="not held: airspeed unreachable",
                                                        e2t=1.08),
       cell(-30, 18.0, -35.0, 18.0, 80.0, e2t=1.14),
       cell(60, 18.0, 45.0, 16.0, 100.0, status="not held: underspeed", steady=False,
            elev=9.0),
       {"climb_deg": 60.0, "speed_goal_mps": 20.0, "n": 0,
        "status": "skipped: the motor holds at most 16.0 m/s at climb +60"}]
fit = T.fit_trim(res)
check("    the four gains are recovered from the steady cells",
      fit is not None and all(abs(fit[0][g] - TRUE[g]) < 1e-6 for g in T.TRIM_GAINS),
      str({g: round(v, 4) for g, v in fit[0].items()}) if fit else "no fit")
check("    a cell steady at another airspeed is in the fit; one not steady is not",
      fit is not None and len(fit[3]) == 7 and T.cell_key(30, 18.0) in fit[3]
      and T.cell_key(60, 18.0) not in fit[3])
check("    fewer than five steady cells is not enough to fit", T.fit_trim(res[:4]) is None)
_no_e2t = [dict(r, eas2tas=None) if i == 0 else r for i, r in enumerate(res)]
_fit_no = T.fit_trim(_no_e2t)
check("    a cell without EAS2TAS is left out of the fit",
      _fit_no is not None and T.cell_key(0, 14.0) not in _fit_no[3]
      and len(_fit_no[3]) == 6)
check("    the lift regressor is the lift term the trim read, when there is one",
      T.trim_terms(30.0, T.C.PITCH_TRIM_VREF_MPS, 50.0, 1.0, 0.7)[1] == 0.7)
# Under Lift Vector Pointing the trim reads the lift load the law asks for,
# which differs from cos(pitch): each cell's own term is what the fit regresses on.
_att = [cell(c, sp, p, v, thr, e2t=e, lift=1.2 * math.cos(math.radians(p)) + 0.15)
        for c, sp, p, v, thr, e in ((0, 14.0, -3.0, 14.0, 60.0, 1.03),
                                    (30, 14.0, 26.0, 14.0, 75.0, 1.10),
                                    (60, 14.0, 55.0, 14.0, 90.0, 1.19),
                                    (-30, 14.0, -34.0, 14.0, 45.0, 1.05),
                                    (0, 18.0, -5.0, 18.0, 92.0, 1.17),
                                    (-30, 18.0, -35.0, 18.0, 80.0, 1.14))]
_fit_att = T.fit_trim(_att)
check("    the four gains are recovered from cells flown on the lift load",
      _fit_att is not None
      and all(abs(_fit_att[0][g] - TRUE[g]) < 1e-6 for g in T.TRIM_GAINS),
      str({g: round(v, 4) for g, v in _fit_att[0].items()}) if _fit_att else "no fit")

print("\n  out of reach:")
lim = dict(res[5]); lim["limit"] = T.limit_of(lim, "airspeed unreachable")
check("    full throttle marks the motor's limit", lim["limit"] == "thrust")
check("    a faster cell at that climb or steeper is skipped",
      (T.out_of_reach([lim], 30.0, 20.0) or "").startswith("skipped: the motor holds at most")
      and (T.out_of_reach([lim], 60.0, 20.0) or "").startswith("skipped"))
check("    a shallower climb, or a speed it reached, is flown",
      T.out_of_reach([lim], 15.0, 20.0) is None and T.out_of_reach([lim], 30.0, 17.5) is None)
idle = dict(cell(-45, 13.0, -49.0, 14.5, 0.0, status="not held: airspeed unreachable"))
idle["limit"] = T.limit_of(idle, "airspeed unreachable")
check("    idle throttle marks the dive's limit; a slower, steeper dive is skipped",
      idle["limit"] == "idle"
      and (T.out_of_reach([idle], -60.0, 12.0) or "").startswith("skipped: idle throttle")
      and T.out_of_reach([idle], -30.0, 12.0) is None)
capped = cell(30, 18.0, 25.0, 16.5, 80.0, status="not held: airspeed unreachable")
check("    a pinned airspeed loop marks the limit below a throttle ceiling",
      T.limit_of(capped, "airspeed unreachable: throttle held at 80% with 16.5 m/s") == "thrust"
      and T.limit_of(dict(capped, eas_mps=19.5, throttle_pct=20.0),
                     "airspeed unreachable: throttle held at 20% with 19.5 m/s") == "idle")
check("    a held cell off the throttle stops marks no limit",
      T.limit_of(cell(30, 18.0, 25.0, 18.0, 80.0), None) is None)
# A slow climb that runs out of LIFT: the elevator on its stop, the aircraft
# mushing, the airspeed falling with the throttle low.  Not the motor's limit,
# so the faster cells at that climb and above stay in the run.
mush = cell(15, 13.0, -13.0, 13.1, 9.0, status="not held: underspeed", steady=False,
            elev=44.5)
mush_why = "underspeed (10.2 m/s and falling, throttle 9%)"
check("    an underspeed stop with the elevator on its stop marks no limit",
      T.limit_of(mush, mush_why) is None)
mush["limit"] = T.limit_of(mush, mush_why)
check("    ...so no faster or steeper cell is skipped because of it",
      T.out_of_reach([mush], 15.0, 17.0) is None and T.out_of_reach([mush], 45.0, 19.0) is None)
check("    an underspeed stop with the throttle low marks no limit",
      T.limit_of(cell(30, 17.0, 20.0, 14.0, 40.0, steady=False, elev=20.0),
                 "underspeed (14.0 m/s and falling, throttle 40%)") is None)
check("    an underspeed stop at full throttle is the motor's limit",
      T.limit_of(cell(30, 17.0, 20.0, 14.0, 100.0, steady=False, elev=20.0),
                 "underspeed (14.0 m/s and falling, throttle 100%)") == "thrust")
check("    an overspeed stop with the throttle up marks no limit",
      T.limit_of(cell(-45, 15.0, -50.0, 24.0, 40.0, steady=False, elev=20.0),
                 "overspeed (24.0 m/s and rising, throttle 40%)") is None)
check("    a pinned loop with the elevator on its stop marks no limit",
      T.limit_of(cell(0, 13.0, -35.0, 13.8, 0.0, steady=False, elev=45.0),
                 "airspeed unreachable: throttle held at 0% with 13.8 m/s") is None)
_st = T.speed_stop(window(n=100, eas=lambda i: 12.0 - 0.02 * i, throttle=9.0), 15.0, sat_s=0.0)
check("    the underspeed stop says the throttle, not a cause",
      _st == "underspeed (10.0 m/s and falling, throttle 9%)", str(_st))

print("\n  the report:")
L = T.report_lines(res, [-30, 0, 30, 60], [14.0, 18.0, 20.0], "20260927_120000",
                   "trim_map_log_20260927_120000.csv", dict(TRUE), ["12:00:00  hello"], False)
txt = "\n".join(L)
check("    it states what was flown and skipped",
      "8 cells: 6 held, 1 steady at another airspeed, 1 not steady" in txt
      and "1 cells out of reach" in txt, [l for l in L if l.startswith(("flown", "skipped"))])
check("    it names the log", "log         trim_map_log_20260927_120000.csv" in txt)
check("    it tabulates the elevator", "ELEVATOR TO HOLD THE ATTITUDE" in txt)
check("    steady at another airspeed is marked ~, not steady *",
      "%.1f~" % res[5]["elevator"] in txt and "9.0*" in txt)
check("    a skipped cell is marked -", any("-" in l.split()[-1] for l in L if "+60 deg" in l))
check("    it suggests the four gains",
      all(("%-28s = %.2f" % (row, TRUE[g])) in txt for row, g in zip(T.TRIM_ROWS, T.TRIM_GAINS)),
      [l for l in L if "Pitch Trim" in l])
_cols = ("climb_deg", "speed_goal_mps", "status", "steady", "n", "pitch_deg", "pitch_sd",
         "eas_mps", "eas_sd", "elevator", "elevator_sd", "aoa_deg", "path_deg",
         "throttle_pct", "eas2tas", "q_dps", "trim_elev")
_data = L[L.index(",".join(_cols)) + 1:]
check("    one data line per cell, with its steady flag",
      sum(1 for l in _data[:9] if l.count(",") == len(_cols) - 1) == 9
      and any(",1," in l and "airspeed unreachable" in l for l in _data[:9]))
check("    the transcript is included", "12:00:00  hello" in txt)

# ---------------------------------------------------------------------------
print("\n  the checkpoint:")
with tempfile.TemporaryDirectory() as td:
    sig = T.grid_signature([0, 30], [16.0])
    T.ckpt_save("20260101_000000", sig, res[:1], cwd=td)
    T.ckpt_save("20260101_000001", sig, res[:2], cwd=td)
    kept = sorted(f for f in os.listdir(td) if f.startswith("incomplete_trim_map_run_"))
    check("    a new checkpoint clears the previous one", kept ==
          ["incomplete_trim_map_run_20260101_000001.json"], str(kept))
    ts, got = T.ckpt_load(sig, cwd=td)
    check("    it resumes the same grid under the same code",
          ts == "20260101_000001" and len(got) == 2)
    check("    a different grid starts afresh", T.ckpt_load(T.grid_signature([0], [16.0]), cwd=td)
          == (None, []))
    _orig = T.PACKAGE_SHA256
    T.PACKAGE_SHA256 = "different"
    check("    different code starts afresh", T.ckpt_load(sig, cwd=td) == (None, []))
    T.PACKAGE_SHA256 = _orig
    T.ckpt_clear(cwd=td)
    check("    clearing removes it", not any(f.startswith("incomplete_trim_map_run_")
                                          for f in os.listdir(td)))

# ---------------------------------------------------------------------------
# The whole run, flown against a fake aircraft.
# ---------------------------------------------------------------------------
class _FakeInjector:
    def __init__(self, link, sensor, lon, lat, force_direction=False):
        self.lat, self.force = lat, force_direction
        link.balloon_lat = lat
    def start(self): return True
    def stop(self): pass

class _Link:
    """The aircraft: steady on the injected climb angle at the airspeed the
    FLOWN gains command, needing elev_needed(); its throttle varies with climb
    and airspeed below full.  unreachable = {climb: airspeed it tops out at, at
    full throttle}; floor_ticks = altitude reads below the floor; floor_at /
    abort_at = the FC tick from which it is below the floor until the next
    climb / at which the operator presses 'e'."""
    def __init__(self, ctrl, unreachable=None, floor_ticks=0, floor_at=None,
                 abort_at=None):
        self.c, self.t_ms, self.seq = ctrl, 0, 0
        self.balloon_lat = 0.0
        self.unreachable = unreachable or {}
        self.floor_ticks = floor_ticks
        self.floor_at, self.abort_at = floor_at, abort_at
        self.climbs = 0
    @property
    def relative_alt(self):
        if self.floor_ticks > 0:
            self.floor_ticks -= 1
            return 50.0
        if self.floor_at is not None and self.seq >= self.floor_at:
            return 50.0
        return 500.0
    @property
    def lua_tick_frames(self):
        self.t_ms += 20
        self.seq += 1
        if self.seq == self.abort_at:
            self.c.abort.set()
        g = self.c._gains
        v = float(g.airspeed_cmd_mps) if int(g.airspeed_cmd_enable) else 12.0
        cap = self.unreachable.get(self.balloon_lat)
        sat = cap is not None and v > cap
        v = min(v, cap) if cap is not None else v
        pitch = self.balloon_lat - 4.0
        thr = 100.0 if sat else min(95.0, 30.0 + 0.5 * self.balloon_lat + 3.0 * (v - 14.0))
        # Each climb angle is flown at its own height, so its own EAS2TAS.
        e2t = 1.06 + 0.001 * self.balloon_lat
        return frames(pitch_deg=pitch, eas=v, elev=elev_needed(pitch, v, thr, e2t),
                      t_ms=self.t_ms, seq=self.seq, asp_sat=sat, throttle=thr,
                      eas2tas=e2t)
    def level_and_hold(self, *a, **k): return True
    def climb_to(self, alt, abort=None):
        self.climbs += 1
        self.floor_at = None

class _Ctrl:
    def __init__(self, climbs, speeds, stop_after=None):
        self._gains = types.SimpleNamespace(airspeed_cmd_enable=0, airspeed_cmd_mps=15.0,
                                            **TRUE)
        self._op = self._gains
        self.trim_map_climb_min_deg, self.trim_map_climb_max_deg = min(climbs), max(climbs)
        self.trim_map_climb_step_deg = (max(climbs) - min(climbs)) / max(1, len(climbs) - 1)
        self.trim_map_speed_min_mps, self.trim_map_speed_max_mps = min(speeds), max(speeds)
        self.trim_map_speed_step_mps = (max(speeds) - min(speeds)) / max(1, len(speeds) - 1)
        self.trim_map_protect_floor_m, self.trim_map_protect_climb_to_m = 100.0, 300.0
        self.throttle_pct = 70.0
        self.autotune_status = self.straighten_start_t = self.autotune_start_t = None
        self.autotune_test_t0 = self.autotune_test_done_s = None
        self._tracking_transition = True
        self.sensor = types.SimpleNamespace(set_gimbal_locked=lambda on: None)
        self.injected, self.gain_log, self.finished = [], [], 0
        self.stop_after, self.abort = stop_after, threading.Event()
        self._ts = {}
    def run_stamp_for(self, k): return self._ts.setdefault(k, "20260927_120000")
    def adopt_run_stamp(self, k, ts): self._ts[k] = ts; return ts
    def operator_gains(self): return self._op
    def begin_measurement_gains(self, reason, **o):
        self.gain_log.append(("begin", o))
        self._gains = types.SimpleNamespace(**{**vars(self._op), **o})
    def update_measurement_gains(self, **o):
        self.gain_log.append(("update", o))
        self._gains = types.SimpleNamespace(**{**vars(self._gains), **o})
    def end_measurement_gains(self):
        self.gain_log.append(("end", {}))
        self._gains = self._op
    def set_point_injector(self, inj): pass
    def ptest_begin(self, lon, lat):
        self.injected.append(lat)
        if self.stop_after is not None and len(self.injected) > self.stop_after:
            self.abort.set()
    def cascade_has_run_since_inject(self): return True
    def autotune_pause_tracking(self): pass
    def ptest_enter_acro(self):
        self._tracking_transition = False
        return True
    def autotune_finish(self): self.finished += 1

_saved = (_inj_mod.SyntheticBalloonInjector, T.MIN_FLY_S, T.SAT_GIVEUP_S)
_inj_mod.SyntheticBalloonInjector = _FakeInjector
# SAT_GIVEUP_S is FC time, which the fake runs through at no wall cost; a
# STEADY_S window before the give-up makes a capped cell steady at its cap.
T.MIN_FLY_S, T.SAT_GIVEUP_S = 0.0, T.STEADY_S + 0.5
_cwd = os.getcwd()
try:
    with tempfile.TemporaryDirectory() as td:
        os.chdir(td)
        print("\n  a whole run:")
        c = _Ctrl([0, 30, 60], [14.0, 16.0])
        c.link = _Link(c, unreachable={60.0: 15.0})
        T.run_trim_map(c, c.abort)
        rep = open("trim_map_report_20260927_120000.txt").read()
        check("    every cell is flown", len(c.injected) == 6, str(c.injected))
        check("    the private gains switch the airspeed hold on",
              c.gain_log[0] == ("begin", {"airspeed_cmd_enable": 1, "airspeed_cmd_mps": 14.0}),
              str(c.gain_log[0]))
        check("    each cell flies its own airspeed",
              [o["airspeed_cmd_mps"] for k, o in c.gain_log if k == "update"]
              == [14.0, 14.0, 14.0, 16.0, 16.0, 16.0])
        check("    the gains are given back once, at the end",
              [k for k, _ in c.gain_log].count("end") == 1 and c.gain_log[-1][0] == "end")
        check("    the aircraft is handed back", c.finished == 1)
        check("    the start latch is cleared so a second 'e' can stop it",
              c._tracking_transition is False)
        check("    a cell the motor cannot hold is recorded steady at its cap",
              "not held: airspeed unreachable" in rep and "5 held, 1 steady" in rep)
        check("    the steady cells recover the true formula",
              all(("%-28s = %.2f" % (row, TRUE[g])) in rep
                  for row, g in zip(T.TRIM_ROWS, T.TRIM_GAINS)),
              [l for l in rep.split("\n") if "Pitch Trim" in l])
        check("    a finished run clears its checkpoint",
              not any(f.startswith("incomplete_trim_map_run_") for f in os.listdir(td)))
        rows = [l for l in open("trim_map_log_20260927_120000.csv") if l[0] not in "#c"]
        # six cells, each at least a full STEADY_S window of 50 Hz ticks
        check("    every sampled tick is in the log, the window marked",
              len(rows) >= 5 * int(T.STEADY_S * 50 - 2)
              and sum(l.rstrip().endswith(",1") for l in rows) >= 5 * int(T.STEADY_S * 50 - 2),
              "%d rows" % len(rows))

        print("\n  skipping what a flown cell showed is out of reach:")
        c = _Ctrl([0, 60], [14.0, 16.0, 18.0])
        c.run_stamp_for = lambda k: "20260927_125000"
        c.link = _Link(c, unreachable={60.0: 15.0})
        T.run_trim_map(c, c.abort)
        rep = open("trim_map_report_20260927_125000.txt").read()
        check("    +60 at 18 m/s is skipped once +60 topped out at 15",
              c.injected == [60.0, 0.0, 60.0, 0.0, 0.0]
              and "skipped: the motor holds at most 15.0 m/s at climb +60" in rep,
              str(c.injected))

        print("\n  the altitude floor:")
        os.remove("trim_map_report_20260927_120000.txt")
        c = _Ctrl([0], [16.0])
        c._ts = {}
        c.link = _Link(c, floor_ticks=5)
        c.run_stamp_for = lambda k: "20260927_130000"
        T.run_trim_map(c, c.abort)
        rep = open("trim_map_report_20260927_130000.txt").read()
        check("    below the floor it climbs, then re-flies the cell",
              c.link.climbs >= 1 and len(c.injected) == 2 and "1 held" in rep,
              "%d climbs, %d flights" % (c.link.climbs, len(c.injected)))

        def cell_rows(ts):
            """The log's rows as (t_s, in_window)."""
            rows = [l.rstrip().split(",") for l in open("trim_map_log_%s.csv" % ts)
                    if l[0] not in "#c"]
            return [(float(r[3]), r[-1] == "1") for r in rows]

        c = _Ctrl([0], [16.0])
        c.link = _Link(c, floor_at=40)
        c.run_stamp_for = lambda k: "20260927_133000"
        T.run_trim_map(c, c.abort)
        rows = cell_rows("20260927_133000")
        first = [r for r in rows if r[0] <= 40 * 0.02]
        check("    a cell the floor interrupts keeps that attempt's ticks in the log",
              len(c.injected) == 2 and len(first) >= 35 and not any(w for _, w in first)
              and any(w for _, w in rows),
              "%d flights, %d ticks before the floor, %d rows" % (len(c.injected),
                                                                 len(first), len(rows)))

        print("\n  a run that cannot start:")
        c = _Ctrl([0], [16.0])
        c.link = _Link(c)
        c.trim_map_climb_min_deg = "not a number"
        T.run_trim_map(c, c.abort)
        check("    still hands the aircraft back",
              c.finished == 1 and not c.injected)
        with open("incomplete_trim_map_run_20260927_000000.json", "w") as f:
            f.write("[]")
        check("    a checkpoint of the wrong shape is ignored",
              T.ckpt_load(T.grid_signature([0.0], [16.0])) == (None, []))
        os.remove("incomplete_trim_map_run_20260927_000000.json")

        print("\n  stop and resume:")
        c = _Ctrl([0], [16.0])
        c.link = _Link(c, abort_at=30)
        c.run_stamp_for = lambda k: "20260927_135000"
        T.run_trim_map(c, c.abort)
        rows = cell_rows("20260927_135000")
        check("    'e' mid-cell keeps the ticks flown so far in the log",
              len(rows) >= 25 and not any(w for _, w in rows), "%d rows" % len(rows))
        for f in os.listdir(td):
            if f.startswith("incomplete_trim_map_run_"):
                os.remove(f)

        c = _Ctrl([0, 30], [16.0], stop_after=1)
        c.run_stamp_for = lambda k: "20260927_140000"
        c.link = _Link(c)
        T.run_trim_map(c, c.abort)
        ck = [f for f in os.listdir(td) if f.startswith("incomplete_trim_map_run_")]
        check("    'e' mid-run keeps the checkpoint", ck == ["incomplete_trim_map_run_20260927_140000.json"],
              str(ck))
        check("    and the report of what was flown", "1 held" in
              open("trim_map_report_20260927_140000.txt").read())
        c2 = _Ctrl([0, 30], [16.0])
        c2.run_stamp_for = lambda k: "20260927_150000"      # a new launch's stamp
        c2.link = _Link(c2)
        T.run_trim_map(c2, c2.abort)
        check("    the next 'e' flies only the unflown cell", c2.injected == [0.0], str(c2.injected))
        rep = open("trim_map_report_20260927_140000.txt").read()
        check("    under the first launch's stamp, with every cell in the report",
              "2 held" in rep and not os.path.exists("trim_map_report_20260927_150000.txt"))
        hdr = [l for l in open("trim_map_log_20260927_140000.csv") if l.startswith("cell,")]
        check("    the resumed run appends to the same log", len(hdr) == 1)
        check("    and clears the checkpoint when done",
              not any(f.startswith("incomplete_trim_map_run_") for f in os.listdir(td)))
finally:
    os.chdir(_cwd)
    _inj_mod.SyntheticBalloonInjector, T.MIN_FLY_S, T.SAT_GIVEUP_S = _saved

print("\n" + "=" * 70)
print("PASSED" if not fails else f"FAILED: {fails}"); print("=" * 70)
sys.exit(1 if fails else 0)
