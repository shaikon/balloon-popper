# TODO — tests for the operator to fly

A record of the flight tests waiting to be flown on the current build, and what
each log is read for. Per `CLAUDE.md`, nothing here is started by Claude without
explicit approval; these are the operator's to fly.

Record ONE continuous flight log per session: press `q` once before the first
test and once after the last. A new `q` recording overwrites `flight_log.csv`,
so copy it out first if a session needs two.

---

## 1. New-build checks (first SITL session on this build)

The build adds per-mode rate-loop writes, the two set Write rows and STATE2.
Watch the console and the log for each of these; none needs its own flight.

- [ ] **Startup.** No halt listing rate-loop differences (the FC should hold the
  operator's values from the previous exit). If it halts, the list says which
  values the FC is holding instead.
- [ ] **Launch (FBWA).** The console prints `[FC PARAM] FBWA: wrote …` with the
  `NATIVE_MODE_RATE_PIDS` values that differed from the FC.
- [ ] **Entering a `p` test (ACRO).** `[FC PARAM] ACRO: wrote …` with the
  operator's values back.
- [ ] **End of the test (LOITER).** `[FC PARAM] LOITER: wrote …` with the native
  values again. In MAVProxy, `param show PTCH_RATE_IMAX` reads 0.666.
- [ ] **In the log.** The `PTCH_RATE_IMAX` column reads 0 on the `p`-test rows and
  0.666 on the loiter rows.
- [ ] **Set Write rows.** In LOITER, press "Default Autopilot Parameters" Write: the
  console lists all 15 values written and read back. Press "Default Acro
  Parameters" Write: every stored Acro default written and read back, and the
  rate-loop boxes change to match. Leave "Default Autopilot Parameters" as the
  last one pressed before the next test, so the next ACRO entry shows the restore.
- [ ] **Exit.** Stop the program with Ctrl+C. In MAVProxy,
  `param show PTCH_RATE_IMAX` reads 0 (the operator's). Restart the program: no
  rate-loop halt.
- [ ] **STATE2 columns.** `lua_q_*`, `lua_gyro_*_rads`, `lua_vel_*_mps`,
  `lua_out_*` and `lua_pwm_*` carry values; `lua_state2_seq` equals `lua_seq`;
  `lua_dt_ms` sits near 20.
- [ ] **Hand-back.** After each `p` test ends, the aircraft loiters normally and
  does not chase the old balloon.

### Added by the review-fix build

- [ ] **Blank when the law did not run.** On loiter rows (`lua_gate` not 0 or 6)
  the law-internal columns (`pan_integrator`, `pitch_integrator`, `steering_*`,
  `roll_sat`, `pan_i_frozen` …) are BLANK, not 0; on `p`-test rows they carry
  values. The rate and throttle command columns are blank whenever `lua_gate` is
  not 0.
- [ ] **Speed scaler.** `lua_scaler` stays inside the range ArduPlane allows for
  `AIRSPEED_MIN`/`AIRSPEED_MAX`, and matches `SCALING_SPEED / lua_eas_mps` away
  from the ends. At startup the console shows no warning that `AIRSPEED_MIN`,
  `AIRSPEED_MAX`, `PTCH_RATE_FF` or `PTCH_RATE_P` could not be bound.
- [ ] **Alignment measure.** The `p` verdict, the HUD "Path Aligned" timer and
  the `in_tol` column all agree, tick for tick: `in_tol` is 1 exactly where
  `hypot(align_pan_deg, align_tilt_deg)` is under `lock_tol_deg`. A `p` PASS now
  means the FLIGHT PATH held on the balloon, not the nose.
- [ ] **Protection climb and level-out.** When a sweep or `i` run drops below the
  floor, the climb's stick overrides start the moment FBWA is confirmed — no
  second or two of the aircraft on centred real sticks while the rate-loop
  values are written. Same for each level-out between trials.
- [ ] **Typed tuner value survives a mode switch.** Type a new value into a saved
  ArduPlane parameter row (e.g. `PTCH_RATE_P`) without pressing Write, then
  change mode. The box keeps the typed number, drawn in the pending colour.
  Pressing Read puts the FC's value back.
- [ ] **Servo calibration halt.** With a servo `_MIN`, `_MAX` or `_TRIM` changed
  in MAVProxy, startup HALTS listing it with the expected value (it no longer
  aborts, and no longer only prints an advisory). Put it back afterwards.
- [ ] **Plant-ID from the FC's own tick.** Run `i` on one axis. The console shows
  the doublets being recorded and a fit with a sensible R²; in the raw-signals
  CSV the `t_s` column (FC time) steps by about 0.02 s, one sample per FC tick.
  The same run is now possible on the real aircraft (no SIMSTATE needed).
- [ ] **Code hash.** The `# code_sha256:` at the head of the log matches the one
  given with this build (it now includes `main.py`).

### Added by the Gain Tuner defaults build

- [ ] **Roll rate loop to its defaults.** Press the reset icon on `RLL_RATE_P`,
  `RLL_RATE_I` and `RLL_RATE_IMAX`: the console prints each reset "to its stored
  Acro default" and the readback ✓ (P as `EXPECTED_RATE_PIDS` holds it, I and
  IMAX 0). The next startup does not halt
  on them.
- [ ] **Read All.** Press Read All on the All Parameters row: the console lists
  every parameter read, and each box shows the FC's value; a typed, unwritten
  value in any box is cleared.
- [ ] **Default Acro Parameters restores the defaults.** In ACRO, type 0 into
  `PTCH_RATE_FF` and press its Write; Read shows 0. Press "Default Acro
  Parameters" Write; Read (or Read All) shows the stored default. Switch to
  LOITER and back to ACRO: `PTCH_RATE_FF` is still the default, not 0.
- [ ] **Roll with no rate integrator.** In the next `p` test, watch for a steady
  bank offset or a slow roll-off: with `RLL_RATE_I` 0, the aileron that holds
  wings level against propeller torque comes from the roll law's pan integrator,
  which is capped by its windup limit. Compare `pan_integrator` and
  `pan_i_clamped` in the log against `fc_roll_i` of an earlier flight.

### Added by the path-frame and yaw build

With every new setting at 0 (Yaw Coordination FF, Yaw Point Steep Schedule
off) the laws fly as before, except that every balloon error but roll-to-plane's
clock is read from the flight path (§ 7 items 12-13 and 17).

- [ ] **The link.** `lua_dt_ms` sits near 20 and `lua_state2_seq` equals
  `lua_seq` on every row, at the STATE and STATE2 sizes
  `python3 -m Companion.protocol` prints.
- [ ] **The markers as read.** `lua_vel_marker` and `lua_lat_marker` follow the
  path with no flat tops at 45 deg, including through a stall or a tail-slide.
- [ ] **The new rows.** Yaw Coordination FF is in the Gain Tuner with an info
  and a reset button, and Yaw Point Pitch Sched reads Yaw Point Steep Schedule.
  A change reaches `g_yaw_coordination_ff` on the next row.
- [ ] **Coordination alone.** Yaw Coordination FF 1, Yaw Point Kp and Ki 0, a
  `p` test with turns: `lua_yaw_rate_cmd_dps` equals `lua_yaw_coordination_dps`,
  which is `lua_yaw_ff_coord + lua_yaw_ff_ari`; `lua_out_rudder` moves with the
  turns; `lua_lat_marker` stays small through rolls. At 0 the rudder is
  released (`lua_out_rudder` 0 with pointing off).
- [ ] **No schedules.** The Gain Tuner has no Gain scheduling group, and the
  log has no `sched_*` or `g_*_interp_sched_*` columns. A script that reads a
  log by column position needs the new header; reading by name is unaffected.

### Added by the bug-fix build

- [ ] **The turn's pitch rate is off.** `g_pitch_turn_ff` reads 0 from the
  first row, as `PITCH_TURN_FF` says; `lua_pitch_turn_dps` is still logged, but
  the pitch command in a turn no longer carries it. Logs from earlier builds flew
  it at 1, so compare turns with that in mind.
- [ ] **Caps Lock.** With Caps Lock on, `s`, `q`, `r` and `t` do nothing (no
  arrow, no mode change); the arrow keys still steer in manual mode.
- [ ] **The gimbal's age.** `gimbal_age_ms` moves with the gimbal's pose
  stream instead of sitting near 0.
- [ ] **Blank until it arrives.** At startup the `fc_*` PID_TUNING columns and
  `fc_aoa` / `fc_ssa` are blank until their messages arrive, then filled.
- [ ] **A fresh balloon chaser.** Stop an `x` balloon chaser, then start one on a balloon the
  detector has not found yet: `lua_gate` reads 4 (no estimate) until it does,
  instead of steering toward the previous balloon.
- [ ] **Plant-ID's airspeed check.** The console prints each axis's gate line
  with the FC's own `AIRSPEED_MIN` (floored at `MIN_AIRSPEED_MIN_MPS`).
- [ ] **Stopped at the parameter halt.** With changed scripts, press Ctrl+C at
  the halt, then relaunch: the scripts are written and the VM reloaded on the
  relaunch, and the FC answers.

### Added by the audit build

- [ ] **The banner.** The FC console's startup line ends `(GUIDE stale after
  N ms)`, N being `GUIDE_STALE_MS` in `constants.py`.
- [ ] **A stale GUIDE releases.** During an `x` chase, pause the companion
  (Ctrl+Z in its terminal) for a few seconds: the FC console prints `released
  control - GUIDE stale` and the aircraft flies ACRO on
  the sticks. Resumed (`fg`), the companion either carries on and the next
  GUIDE takes control back, or, if it reads the link's silence first, halts
  with `No report from the flight controller` and hands back to LOITER.
- [ ] **The halt box.** Any halt in flight ends with the framed `THE PROGRAM HAS
  STOPPED` block and waits for Enter; the flight log is complete up to it.
- [ ] **Integrators outside ACRO.** In LOITER and FBWA rows, `pan_integrator`,
  `pitch_integrator`, `lua_plane_i_dps` and `lua_yaw_point_i_dps` are blank or
  0, and each chase's integrators start from 0.
- [ ] **The airspeed hold restarts from the operator's throttle.** With Airspeed
  Hold on, stop and restart a chase: `lua_asp_i_term` on the first commanded
  row equals the operator's throttle, not the value it held before the stop.
- [ ] **A wrong scripts directory copies nothing.** Launched with
  `--lua-scripts` pointing elsewhere than SITL's cwd, startup refuses with
  `REFUSING TO DEPLOY: wrong directory, nothing copied` and the named directory
  holds no new scripts.

### Added by the roll-to-plane build

The blend reads the path's steepness alone, Rudder Near-Path Share is gone,
roll-to-plane's clock is read round the nose, roll-to-plane has a derivative
term (off by default), and the defaults are Steep 65 / 80 and Roll-to-Plane
Gain 9 (§ 7 items 16-19).

- [ ] **The rows.** Pan Law Blending holds Roll-to-Plane Steep Start and Steep
  Full only, and ROLL - roll-to-plane starts at Roll-to-Plane Hold; there is no
  Rudder Near-Path Share row, and the log has no `g_rudder_near_path_share` or
  `lua_rudder_share` column. A reset puts Steep Start / Full at 65 / 80 and
  Roll-to-Plane Gain at 9.
- [ ] **The blend.** On a path between the steep angles, the roll command
  (`lua_omega_x_presat`) is `(1 - lua_plane_path_share)` of bank-to-turn's plus
  `lua_plane_path_share` of roll-to-plane's.
- [ ] **Steep near the balloon.** On a path past Steep Full with the balloon a
  degree or two off the path, the roll command is roll-to-plane's whole rate,
  `lua_plane_i_dps` and `lua_plane_drift_dps` run, and the miss keeps closing.
- [ ] **The nose clock.** `lua_plane_err_deg` is
  `atan2(lua_balloon_body_y, -lua_balloon_body_z)`, taken to the chosen end
  (`lua_plane_side`). Re-fly the 85 deg manual climb of
  `flight_log_20261007_222608`: roll-to-plane rolls the other way from that
  log (right, to bring the balloon to 12 round the nose); compare whether the
  miss closes or the path keeps circling the target (`lua_plane_drift_dps`
  against `lua_gyro_x_rads`).
- [ ] **The derivative term.** At Roll-to-Plane Kd 0, `lua_plane_d_dps` reads 0
  wherever roll-to-plane has a share, and the roll command is as before. Raised
  (from about 0.2), `lua_plane_d_dps` is `g_plane_kd` x
  `lua_plane_err_rate_dps`, opposite in sign to `lua_plane_err_deg` while a roll
  carries the balloon toward its end. Against Kd 0, compare how far
  `lua_plane_err_deg` swings past 0 and back, and the roll's swings
  (`lua_gyro_x_rads`); with a small `lua_plane_off_nose_deg`, where the clock's
  rate swings, watch for kicks in `lua_plane_d_dps`.

### Added by the Lift Vector Pointing build

A second steering law, Lift Vector Pointing (`LUA/path_attitude.lua`), chosen
by the Steering Law row in the Gain Tuner's BOTH LAWS section (0 the cascade,
the default; 1 Lift Vector Pointing). It holds Bank Lim on the cascade's bank
proxy in every chase and keeps the rudder at zero (ARCHITECTURE.md § "Lift
Vector Pointing: the second steering law").

- [ ] **The switch.** With Steering Law 1, a `p` test's rows read
  `g_steering_law` 1 and `lua_cs_flags` with bit 6 (64) set, the `lua_att_*`
  columns carry values, and the cascade's (`lua_plane_*`,
  `bank_rate_cmd_dps`) are blank. Back at 0, the reverse.
- [ ] **Rudder at zero.** Under Steering Law 1, `lua_out_rudder` reads 0 on
  every chase row.
- [ ] **The bank limit.** On a chase that starts inside Bank Lim,
  `bank_proxy_deg` stays within `g_bank_lim_deg`, or a few degrees past it while
  the roll settles; `lua_att_band_lo_deg` / `lua_att_band_hi_deg` read minus
  and plus Bank Lim.
- [ ] **A change of law is a fresh chase.** Switching Steering Law during a
  chase starts a new one on the next tick: the other law's integrators start
  from zero, and with the Curve Flyer on the chase waits for a new PLAN
  (`lua_cf_state` 1).
- [ ] **The pitch trim's lift term.** Under Steering Law 1,
  `lua_pitch_trim_lift_term` reads about 1 in straight, level flight and about
  1/cos(bank) in a level turn; under 0 it reads cos(pitch).
- [ ] **The desktop model's weak spots, in SITL.** The FC test's sphere chases
  (§ 18) never hold a balloon high and behind lined up - within a quarter of a
  degree, never inside half a degree for half a second - and fly a Suboptimal
  Curve in about three chases in five that the rule covers: most often a tilt
  that comes back, then a bank a couple of degrees past the band, then a
  heading that moves after the turn. Fly a few of each under Steering Law 1:
  a `p` balloon 75 deg up and 150-170 deg round from the nose, and one 60-75
  deg below. Compare `lua_att_theta_deg` and the Suboptimal Curve checks with
  the cascade's on the same balloons.

### Added by the Curve Flyer plan-mode build

The Curve Flyer no longer flies its own roll and pitch: the selected law flies
the PLAN in its plan mode, the PLAN names the law it was made for, and no clock
ends a turn, a run-in or its dive (ARCHITECTURE.md § "The Curve Planner and the
Curve Flyer").

- [ ] **Each law flies its PLAN.** With the Curve Flyer on, fly the default `p`
  balloon under Steering Law 0 and again under 1: `lua_cf_state` goes 1, 2, 3;
  `cf_plan_law` equals `g_steering_law`; `lua_cs_flags` bit 5 (32) is set on
  the rows in state 2; the console prints one `[curve] plan N: ... planned in
  X s` line per chase.
- [ ] **No time limit.** No row in any log has `lua_cf_state` 5; a turn still
  turning past 8 s stays in state 2 until the path points at the balloon.
- [ ] **Transfer off.** With Transfer Control When Done off, state 6 follows the
  turn, `lua_cf_hold_n/e/d` is the path's direction at the turn's end, and the
  path stays within a few degrees of that line.
- [ ] **The log's head** has a `# system id ...` line (on SITL the computer's
  machine ID) and no `lua bindings` line.

## 2. Pitch overshoot: trim off vs trim on

Same setup as the Ki 50 run (`flight_log_20260925_213422.csv`), so the trim is
the only difference:

- the same `p` balloon and the same 85% throttle, set the same way;
- the same gains: only the small Pitch Gain non-zero, Pitch Ki and every other
  gain 0;
- the flight-path markers unfiltered (built in: there is no setting for it);
- `PTCH_RATE_IMAX` = 0 in the operator's values (startup checks it).

Six `p` tests in one recording, alternating, about 10 s of settling between
tests, each left to finish on its own:

| Test | Pitch Trim Level elev deg | Pitch Trim Slope elev deg |
|------|---------------------------|---------------------------|
| 1    | 0                         | 0                         |
| 2    | 10.9                      | -23.7                     |
| 3    | 0                         | 0                         |
| 4    | 10.9                      | -23.7                     |
| 5    | 0                         | 0                         |
| 6    | 10.9                      | -23.7                     |

Flown (test_1..6, throttle_70, throttle_100): the trim halved the climb's
standing miss and cut level flight's from 6.6 to 0.5 deg, but only at the
throttle it was measured at - which led to the trim map (section 3) and the
airspeed-aware formula that replaced Level and Slope (section 5).

- [x] Fly tests 1-6 and send the log.
- [x] Optional: tests 7 (off) and 8 (on) at a clearly lower throttle, e.g. 60%,
  to see whether a trim measured at 85% still fits.

Read from the log:

- overshoot: how far `dbg_theta_ip_deg` swings past zero after first reaching
  the balloon, and how long it takes to settle.  That is the pitch law's own
  error (balloon elevation minus the flight path).  Beside it: `align_tilt_deg`
  (the flight-path error the `p` verdict, the HUD "Path Aligned" timer and the
  sweep use) and `steering_tilt_deg` (the NOSE's tilt to the balloon, as the FC
  computed it);
- the standing miss: where each of those settles (about 1.2 deg with trim off;
  toward 0 with trim on);
- the trim itself: `lua_pitch_trim_elev`, `lua_pitch_trim_dps` across the
  attitudes flown;
- rate tracking: `lua_omega_y` (asked) against `lua_gyro_y_rads` (flown), and
  `lua_out_elevator`;
- `fc_pitch_i` flat at 0;
- `fc_roll_i` at the moment each test starts (what the roll integrator carried
  out of LOITER);
- link health: `lua_seq_lag`, `lua_dt_ms`, `lua_state2_seq` = `lua_seq`.

Decision:

| Log shows | Next |
|-----------|------|
| Trim on removes the standing miss; overshoot same or smaller | Keep the trim; tune the remaining overshoot |
| Trim on removes the standing miss; overshoot grows | Refit the trim from these logs |
| Flown pitch rate lags well behind the asked rate | Look at the rate-loop lag before the pitch gains |

## 3. Trim map ('e') - first run

Measures the elevator the airframe needs to hold each climb angle at each
airspeed, for a pitch-trim formula (or table) that accounts for airspeed.
About 25 minutes for the default 40 cells; it can be stopped with `e` and
resumed with `e`.

Flown (trim_map_report_20260927_030050): 40 cells in 18 minutes, 34 steady; the
fitted formula is now the default trim (section 5).

- [x] Before starting: the Trim Map group's Alt Protect Floor / Climb-To set
  with room for a 45-degree dive, the grid rows at their defaults (climb -45 to
  +60 by 15, airspeed 13 to 21 by 2). Record a log with `q` as usual.
- [x] Press `e`. The HUD reads "Mode: Trim Map" bottom-left and
  "Trim Map MM:SS cell n/40: climb ... @ ... m/s - <phase>" top-left (level-out,
  pulling onto the bearing, settling, ...). No Balloon Distance line.
- [x] Watch one fast cell (21 m/s) start from the slower level-out speed: it
  must accelerate and settle, not stop as "underspeed".
- [ ] Stop it once with `e` part-way, check the "Trim-map checkpoint" row has a
  file, press `e` again and see it skip the cells already flown.
- [x] At the end, export "Trim-map report" and "Trim-map log" and send
  both, with the flight log.

Read from the report: the elevator table (how it changes along each airspeed
column and each climb row), the cells marked `*` and why, and the residual
table of the fitted current formula.

## 4. The airspeed-aware pitch trim (this build)

The trim is now `offset + (VREF / airspeed)^2 x (lift x L + attitude x
sin(pitch) + thrust x throttle / EAS2TAS^2)`, VREF being `PITCH_TRIM_VREF_MPS`
and L the lift term the FC logs as `lua_pitch_trim_lift_term` (cos(pitch) under
the cascade, the load Lift Vector Pointing asks of the wing under it), and the four
gains default to the trim map's fit (`PITCH_TRIM_OFFSET`, `_LIFT`, `_ATTITUDE`,
`_THRUST` in `constants.py` - the four Pitch Trim rows); the Gain Tuner starts
from them at launch.

- [ ] The same `p` balloon as section 2, at 70%, 85% and 100% throttle, one
  test each, Pitch Ki 0.  Read `dbg_theta_ip_deg` and `align_tilt_deg` where
  each settles, and the level stretches before and after.  Expected: the
  standing miss near zero at ALL three throttles (the old trim was only right
  at 85%), in the climb and in level flight.
- [ ] `lua_pitch_trim_elev` beside `lua_out_elevator` / 100 on the settled rows
  (the output column is ArduPlane's scaled output, 4500 = full travel; the trim
  is in the same units divided by 100, 45 = full): the trim should supply nearly
  all of the elevator, the pitch law very little.
- [ ] Re-run `e` once: cells above the motor's reach at a climb are now
  skipped (the 21 m/s climbs, +60 above 17 m/s), a steady 6-7 deg bank no
  longer times a cell out, and the report fits the four gains over every
  steady cell (~ and blank).  The suggested gains should land near the
  defaults.


## 5. Switching modes without LOITER (this build)

The same key again still hands back to LOITER.  Another mode key during a long
run (`t`, `y`, `i`, `e`) stops the run, checkpoint kept; the run hands the
aircraft over in ACRO (sticks centred, ACRO, sticks released) and the new mode
starts once the run has ended.  The arrows switch into manual.

- [ ] During `e`, mid-cell (in ACRO): press `p`.  The console says "stopping
  The trim map ... no LOITER in between", then "handed over in ACRO", then the
  `p` test starts.  `lua_fc_mode` never shows LOITER; the event column shows
  `KEY_P_SWITCH_FROM_E`; `e` resumes at the interrupted cell next time.
- [ ] During `e`, in a level-out (FBWA): press `x`.  FBWA -> ACRO with the
  sticks centred, then balloon chaser; no override left behind.
- [ ] During `i`, in a doublet (MANUAL): press `s`.  MANUAL -> ACRO with the
  sticks centred at your throttle (no doublet deflection carried over), then
  straight & level.
- [ ] During `t`: press an arrow.  Manual starts, nudged once; a second arrow
  nudges again and does NOT hand back.
- [ ] While a switch waits (the second or so after the key), press another
  mode key: refused with "Switching to ...".
- [ ] `i` with Do Pitch/Yaw/Roll all off: it ends at once and hands back to
  LOITER; the keys work afterwards (no "Transition in progress").

## 6. Simulator air: the trim fitted high misses low

### Findings

The same `p` manoeuvre, same throttle, flown twice
(`flight_log_20260927_154219.csv` low, `flight_log_20260927_161313.csv`
high), settled rows:

|                           | Low (108 m) | High (3012 m) |
|---------------------------|-------------|---------------|
| Airspeed reading (EAS)    | 19.21       | 16.55         |
| Speed through the air     | 19.86       | 19.85         |
| Ratio of the two          | 1.034       | 1.199         |
| Angle of attack           | -3.86       | -3.86         |
| Elevator flown            | 13.69       | 13.67         |
| Trim supplied             | 10.91       | 13.92         |
| Trim short by             | +2.78       | -0.25         |

The aircraft flies identically at both heights (same true speed, same angle of
attack, same elevator), but the airspeed READING drops 14% up high, so the
trim formula, which divides by the reading squared, gives a different answer.

- **Root cause: the simulator's two halves disagree about the air.** Gazebo's
  wing and propeller model (`gz::sim::systems::LiftDrag`, seven entries in
  `Arduplane/SDF/model.sdf`, the propeller included) uses one fixed
  `air_density` at every height. ArduPilot's airspeed reading assumes air
  that thins with height: the JSON backend receives no airspeed from the
  plugin, so SITL makes one from the true speed divided by the standard
  atmosphere's `EAS2TAS` at the aircraft's height (`SIM_JSON.cpp`,
  `SIM_Aircraft.cpp`). The ratio in the table matches the standard atmosphere
  at the world's 584 m elevation plus the flying height.
- **The trim map was flown at about 3000 m** (the Trim Map Climb-To,
  `TRIM_MAP_PROTECT_CLIMB_TO_M`), so its fit learned the high-altitude
  reading. At 100 m the `(VREF / airspeed)^2` factor comes out about a quarter
  smaller for the same flight.
- **Pitch Ki and Pitch Gain do not enter the trim** (checked in the flight
  code): the trim is added after the ceilings, independent of both. Taking Ki
  to 0 removed what was covering the miss; it did not cause it.
- **Gazebo's fixed air equals standard air at about 180 m above sea level.**
  The world sits at 584 m, so even at the field the simulated wing has a few
  per cent more lift than air at that height would give.
- **The real aircraft does not have this problem.** A pitot tube feels the
  same air as the wing, so a trim built on the airspeed reading is right at
  any height and in any weather. ArduPilot's airspeed calibration removes the
  height effect before comparing with GPS speed (`Airspeed_Calibration.cpp`
  at 2d99a5ea), so `ARSPD_RATIO` is a constant of the sensor, not of the
  weather.

### What others found (online search)

- Nobody reports this exact symptom. The same split is built into the
  standard tools: Gazebo's own airspeed sensor (`AirSpeedSensor.cc`) thins the
  air with height while its `LiftDrag` and `AdvancedLiftDrag` use a fixed
  density; PX4's Gazebo setup is the same (airspeed plugin from
  `PX4_HOME_ALT`, lift-drag fixed). Most people fly low, near where the two
  agree.
- ArduPilot's own built-in simulator computes the density from height every
  step, so its wing and its airspeed reading always agree (ArduPilot PR
  #17984, atmosphere tables). That fix does not reach Gazebo, which computes
  its own wing forces.
- Known workarounds: set `air_density` for the height flown (exact at one
  height only); replace the wing model with one that thins the air (a copy
  can be built on its own and dropped into `~/.gz/sim/plugins`, no Gazebo
  rebuild); or use JSBSim, which models the standard atmosphere.

### Plan

1. [ ] **Back up the WSL environment first** (`wsl --shutdown`, then
   `wsl --export <distro> <file>.tar`).
2. [x] **Make the simulator's air thin with height** for the wings and the
   propeller (this build): `Arduplane/AltitudeLiftDrag` is Gazebo's
   `LiftDrag` with the density of ArduPilot's own 1976 standard atmosphere at
   the world's `<elevation>` plus each surface's height, and the seven
   `model.sdf` entries point at it. Checked here against Gazebo Harmonic
   8.10: its force equals stock `LiftDrag`'s times the density ratio, and the
   density matches ArduPilot's formula, at -600, 0, 3000 and 20000 m.
   - [x] Build and install it on WSL, in the shell Gazebo runs from (with the
     same `GZ_VERSION` ardupilot_gazebo was built with, if one was set):
     `cd Arduplane/AltitudeLiftDrag`, then
     `cmake -S . -B ~/build/AltitudeLiftDrag`,
     `cmake --build ~/build/AltitudeLiftDrag`,
     `cmake --install ~/build/AltitudeLiftDrag`. The library lands in
     `~/.gz/sim/plugins`; the build folder stays out of the project.
   - [x] Copy `Arduplane/SDF/model.sdf` over
     `~/talon_sim/models/talon_balloon_chaser/model.sdf`.
   - [x] Start Gazebo with `-v 3` or higher: seven lines
     `AltitudeLiftDrag on [...]: world elevation 584 m, air density there ...`
     (two on `motor_link`, five on `base_link`). At any verbosity, no
     `Failed to load system plugin [AltitudeLiftDrag]` and no
     `AltitudeLiftDrag ... will not generate forces`.
   - [x] ArduPilot's home altitude is the world's elevation: with no `-L` or
     `-l` on the `sim_vehicle.py` line SITL starts at `SIM_OPOS_ALT` (default
     584 m); with `-L <name>` it is the third number of that name's line in
     `Tools/autotest/locations.txt`; with `-l lat,lon,alt,hdg` it is `alt`.
   - [x] Take-off and a circuit fly normally. At the field the air is now a
     few per cent thinner than the old fixed value, so the aircraft needs a
     little more speed for the same lift.
   - [x] Superseded by the log of 2026-09-28 13:23 and the two trim maps:
     with the new air the trim formula's shortfall in that log is predicted
     to within 0.1 elevator units (1.4 off with the old air), and the second
     map's gains predict the first map's higher cells to rms 0.61. The test
     as first written:
     straight and level (`s`) holding the SAME
     airspeed reading (Airspeed Cmd on, e.g. 16 m/s) at about 100 m and about
     3000 m, 30 s each, one recording. The angle of attack (`aoa_deg`) and
     the elevator (`lua_out_elevator`) match at both heights; the throttle is
     higher up high and the speed through the air (`lua_vel_*_mps`) about a
     sixth higher. In the old fixed air the angle of attack was clearly
     lower up high at the same reading. Same THROTTLE is not the test: the
     motor holds its speed, so at one throttle the true speed stays about
     the same and the reading falls with height in the new air too.
3. [x] **Turn rate from the true speed, not the reading.** The turn rate
   `g * sin(roll) * cos(pitch) / V` in `balloon_chaser_fc.lua` (`s.yaw_coord_dps`,
   used by the pitch turn term) divides by the airspeed reading; the aircraft
   turns at the rate its TRUE speed gives. ArduPlane's pitch controller uses
   the true speed here (`AP_PitchController.cpp`, `aspeed * get_EAS2TAS()`);
   its yaw controller uses the reading. Wrong on the real aircraft too, by
   the height of the field. Done: § 9 bug 7, the floored EAS times EAS2TAS.
4. [x] **Log `EAS2TAS` every tick** (STATE2, the four-step rule), so a height
   effect shows in the log at once. Done in this build: the `lua_eas2tas`
   column, and `eas2tas` in the trim map's log and cell data.
   - [x] The FC's value arrives: the second trim map's cells carry 1.06 to
     1.08, flown about 600-900 m above the field.
   - [ ] In the next flight log `lua_eas2tas` has a value on every row, about
     1.03 near 100 m above the field.
5. [ ] **Height and `EAS2TAS` per trim-map cell and in the report header.**
   Awaits approval.
6. [ ] **Startup check that Gazebo's world `<elevation>` and ArduPilot's home
   altitude agree.** Both are 584 m today; if they drift apart the air
   disagrees again. Awaits approval.
7. [ ] **Fly again after item 2:** the trim map (`e`), plant-ID (`i`) and the
   sweep (`t`), all measured in the mismatched air.
   - [x] Trim map, first run in the new air
     (`trim_map_report_20260928_141413`): 17 steady cells, fit rms 0.51
     (offset -1.83, lift 19.93, attitude -26.49, thrust -0.51). At 13 m/s the
     elevator reaches its stop before the wing holds level flight, and those
     stalls were taken for thrust limits, skipping 16 climb cells; with one
     climb cell left the thrust gain is not pinned down (+-3.4).
   - [x] Trim map, second run (`trim_map_report_20260928_153401`): lowest
     airspeed 15 m/s, climb-to 1000 m, and a stall no longer taken for a
     thrust limit. 31 of 32 cells flown, 30 steady; only +60 at 21 m/s
     skipped, after +60 at 19 m/s needed full throttle. Fit rms 0.42 over
     30 cells (worst 1.00): offset -3.16, lift 20.71, attitude -27.22,
     thrust 0.51 - now the defaults, so the four Pitch Trim rows start from
     them. The same fit predicts the first run's cells, flown higher, to rms
     0.61. The thrust gain is still +-2.3: each cell's throttle is whatever
     holds its airspeed, so it moves with the climb angle and the speed and
     the fit cannot separate it; a fit without it is as good (rms 0.42) -
     the motor barely pitches this airframe.
   - [ ] The trim on its own: straight and level (`s`) with Pitch Ki 0 and
     the pitch wind-up limit 0, at about 100 m and about 900 m above the
     field. Expected: level flight at both, `lua_pitch_trim_elev` within
     about half a unit of `lua_out_elevator` / 100 - not the steady 6 deg
     descent of the 13:23 log.
   - [ ] Plant-ID (`i`), then the sweep (`t`).
8. [ ] **Real aircraft:** log the battery voltage, then decide whether the
   propeller's weakening push as the battery drains belongs in the trim's
   thrust term; keep a small Pitch Ki to absorb what the formula misses.
9. [ ] **The trim's thrust term and air density.** The ArduPilot plugin
   holds the motor at a speed set by the throttle (`<type>VELOCITY</type>`),
   so at one throttle the propeller's push scales with the air density, as
   a real propeller's roughly does. The trim's bracket is divided by the
   airspeed reading squared, which carries density for the wing terms but
   not for thrust, so `thrust x throttle x (VREF / airspeed)^2` holds only at
   the density it was fitted at. Fitted about 3000 m above the field and
   used at 100 m, the real thrust effect is about a third larger than the
   term, about 2 deg of elevator at 85% throttle near VREF with the current
   gain. Done in this build: the FC scales the thrust term by the density
   ratio `1 / EAS2TAS^2`, and `trim_map.py` fits it the same way from each
   cell's `eas2tas`; the thrust gain is now the motor's share at sea-level
   air. Within a few hundred metres of a real field the difference is a few
   per cent.
   - [x] Re-flown (item 7); the report shows the formula with
     `/ EAS2TAS^2` and an `eas2tas` value on every cell line.
   - [x] The old gains left the trim 4.5 elevator units short in the new
     air (the log of 2026-09-28 13:23); the second run's fit replaces them as
     the defaults.

Open question: does the real aircraft have a pitot tube? Without one ArduPilot
estimates airspeed from GPS and wind, and the trim's airspeed input needs a
closer look before real flight.

## 7. Barrel roll on `s` from a steep climb

### Findings

`flight_log_20260930_125639.csv`: `s` pressed during a `p` climb (nose 74 deg
up, 14 m/s, 7.5 deg right bank). The aircraft rolled right through a full
360 deg in about 4 s (7 -> 180 deg at 2.9 s -> 0 at 4.9 s), dived to -23 deg
pitch, then settled on the straight heading. Nothing here is special to `s`:
every mode flies the same law (C-03), so any balloon well below the nose in a
steep climb does the same, and C-04 requires such balloons.

Three causes, in order:

- **A balloon under the belly makes bank-to-turn run away.** The straight
  target (level, on the heading at the key press) sat 72 deg below the nose,
  on the belly side. The 7.5 deg bank made it read 7 deg right
  (`steering_pan_deg` -5.9), so the roll law banked right; banking right swings
  a belly-side balloon further right, so the sideways error grew from 7 to
  73 deg and the request reached the bank limit within 0.15 s. The roll law
  reads only the sideways part (`asin(ty)` in `track.lua`), so it cannot tell
  right-and-above from right-and-below, which need opposite rolls. Meanwhile
  the pitch law pushed full nose-down elevator (`lua_out_elevator` -4500) and
  got only about 40 deg/s at 14 m/s, while the roll ran at about 150 deg/s.
  The leftover bank set the direction: a left bank gives a left roll.
- **The bank proxy cannot reach the bank limit at steep pitch.** The roll law
  feeds back `asin(sin(roll) cos(pitch))`, which can never exceed
  `asin(cos(pitch))`: 16 deg at 74 deg pitch. A request above that has no
  stopping point, so the loop kept commanding 95-170 deg/s of roll
  (`lua_omega_x`) through the whole first half-roll; `bank_proxy_deg`
  never read more than about 22 deg while the real bank went past 90.
- **Inverted, the proxy loop runs backwards.** The proxy reads about 0 upside
  down and moves the wrong way as the aircraft rolls, so its only resting
  point is upright. From inverted it follows whatever error is left (+7 deg,
  right) all the way round to upright, completing the full roll.

### Tried and reverted: limiting the real bank

Limiting the roll law's bank request at `asin(sin(bank_lim_deg) x cos(pitch))` - the
proxy reading of a real bank of exactly the bank limit - stops the flip: in the
log's case the roll halts at the bank limit. It also stops the aircraft
steering at steep pitch, in dives and climbs alike, and was reverted.

At steep pitch the roll law steers BECAUSE its bank request is out of the proxy's
reach: the roll goes on until the sideways error is gone, which lines the
balloon up with the pull. Near vertical, bank measured from the horizon means
little; rolling about the nose is what chooses the direction the pull moves
it. With the roll capped at the bank limit the balloon stays off to one side,
where the pull cannot reach it.

Roll-only closed loop through `balloon_chaser_fc.lua` on the desktop stubs, pitch
held, 5 s, balloon 25 deg off the nose at the right wingtip:

| Pitch   | Capped: roll, sideways miss | As flown: roll, sideways miss       |
|---------|-----------------------------|-------------------------------------|
| -60 deg | 45 deg, 17.4 deg            | 76 deg, 5.7 deg                     |
| -75 deg | 44 deg, 17.6 deg            | 86 deg, 1.8 deg                     |
| -85 deg | 34 deg, 20.4 deg            | 89 deg, 0.3 deg                     |
| +75 deg | 44 deg, 17.6 deg            | 86 deg, 1.8 deg                     |
| 0 deg   | 45 deg, 17.4 deg            | 45 deg, 17.4 deg                    |


### Implemented: roll-to-plane, blended with bank-to-turn

Plan items 17 and 19 below change parts of what follows: the clock is read
round the nose (item 17), and roll-to-plane has a derivative term (item 19).

On a steep flight path the roll axis rolls the balloon into the plane a pull
swings the path in instead of banking toward it: looking along the path as at
a clock face (12 = over the canopy, 3 = to the right, 6 = under the belly), to
12 or 6 o'clock, whichever is nearer, and the pitch law (unchanged) pulls
toward a balloon above the path or pushes toward one below. Its error is the clock angle `atan2(t'_y, -t'_z)` in the
path frame (`track.lua`), which a roll about the path changes by exactly the
angle rolled and a roll about the nose to within 1 - cos of the nose-to-path
angle. `LUA/cascade.lua`.

- **The share** is the path share, a straight line, and bank-to-turn has the
  rest. It is the master switch: 0 below Roll-to-Plane Steep Start, 1 at
  Steep Full, in |flight-path angle| - the direction the aircraft is moving
  against the horizon, from the FC's velocity, the nose's pitch standing in
  below `FLIGHT_PATH_MIN_SPEED_MPS` (`plane_steep_start_deg`,
  `plane_steep_full_deg`; defaults `PLANE_STEEP_START_DEG` /
  `PLANE_STEEP_FULL_DEG`; 90 / 90 switches roll-to-plane off). Its cosine is
  the share of gravity acting across the path; the bank does not enter it, so
  knife-edge on a level path stays with bank-to-turn, which rolls the lift
  back up; the nose does not either, since it sits off the path by the angle
  of attack and the sideslip. Below Steep Start the roll command is
  bank-to-turn's exactly. Shallow flight stays with bank-to-turn because a
  turn there needs the lift partly up against gravity, which roll-to-plane
  does not know about. `lua_plane_axis_up` (the world-vertical part of the
  turn axis, nose x balloon) is still logged, not flown: it says whether a
  turn was a heading change or a climb or dive.
- **12 or 6** holds until the other end is nearer by the hold band (Gain
  Tuner row `plane_side_hold_deg`, default `PLANE_SIDE_HOLD_DEG`), and is
  chosen afresh on each chase.
- **Its own gain, integrator and feedforward.** Roll-to-plane's command is
  `plane_gain` x the clock error + an integrator (`plane_ki`, in deg/s of
  roll, at most `plane_windup` of the ACRO roll rate, held while the command
  is saturated its way, cleared with no share, on a new chase and at Ki
  0, kept across a 12 <-> 6 change; default Ki 0) + `plane_ff_gain` x the
  balloon's drift round the clock (the clock angle's rate plus the aircraft's roll rate, low-passed
  at `plane_ff_lpf_rate`). The drift filter is held at zero while
  roll-to-plane has no share and restarts on each chase. Defaults:
  `PLANE_GAIN`, `PLANE_FF_GAIN` = 0, `PLANE_FF_LPF_RATE` = `FF_LPF_RATE`.
  Bank-to-turn's Roll Gain does not reach roll-to-plane's share.
- **The settings** are the first two groups in the Gain Tuner after BOTH LAWS
  (the settings both steering laws read): Pan Law Blending (Roll-to-Plane Steep
  Start / Steep Full, the
  settings that decide the share), then ROLL - roll-to-plane (Hold, Gain, Ki,
  Windup (0-1), FF, FF LPF Rate). Each has Info and reset, each logged as a
  `g_plane_*` column.
- **The pan integrator is held** while roll-to-plane has any share.
- **Logged** (STATE2): `lua_plane_err_deg`, `lua_plane_side`,
  `lua_plane_axis_up`, `lua_plane_drift_dps`, `lua_plane_ff_dps`,
  `lua_plane_path_share`, `lua_plane_i_dps`, `lua_plane_i_frozen`,
  `lua_plane_path_deg` (the angle the path share read),
  `lua_plane_path_from_vel` (0 when the nose's pitch stood in), and
  `lua_plane_off_nose_deg`, the balloon's angle off the nose, logged, not
  flown.
- **Tests:** `test_balloon_chaser_fc.lua` section 10 - shallow flight gets no share
  (a balloon 60 deg above ahead, a balloon 100 deg right at the same height:
  bank-to-turn's command exactly); steep flight gets all of it, a balloon a
  degree off the nose included, built with roll-to-plane's own gain (set
  unlike Roll Gain); the integrator hold; the path line (half of each law
  halfway between the steep angles, either sign, none below Steep Start, none
  at 90 / 90); the flight path, not the nose or the bank (knife-edge on a level
  path: none; nose 75 deg down on a 50 deg-down path: none; nose level on a
  75 deg-down path: all; below `FLIGHT_PATH_MIN_SPEED_MPS` the nose stands in
  for the path); the 12/6 hold band both ways and reaching the FC over
  SET_GAIN; a fresh choice on a new chase; the clock read round the path (56
  deg round it where the nose's reads 8, in a near-vertical climb with the
  path 7 deg off the nose), a roll about the nose turning it by the angle rolled to within 1 -
  cos of the nose-to-path angle, and a balloon over a sideslipping path zero
  for both laws; the feedforward (a balloon drifting round the clock:
  the filter settles on its rate and the command carries the share; a
  world-fixed balloon with the aircraft rolling: no drift; no share: held at
  zero); the integrator (from zero on a new chase, grows at Ki x
  error and reaches the command, stops at its windup share, held when
  saturated its way, removed at Ki 0 and with no share), and the log's case
  flown closed loop on roll (pitch held): roll-to-plane levels the wings and
  the pitch law pushes.

Known limits:

- **Shallow flight is bank-to-turn's alone,** a balloon far above or below
  included: on a path under Steep Start, a balloon 60 deg above ahead is
  flown by banking toward it.
- **Between the steep angles the share is a mix** of the two laws - a
  reasonable middle, not the exact answer, which would roll the LIFT to 12
  (the balloon direction plus enough upward to cancel gravity) and needs a
  turn-strength input.
- **Near dead astern** the clock angle is undefined and the pitch law's demand
  small (it folds a balloon behind onto its elevation); bank-to-turn's
  rear-hemisphere tie-break flies only its share there.

### Plan

1. [x] **Roll-to-plane blend** (above).
2. [x] **Pin the cases in `test_balloon_chaser_fc.lua`:** section 10 (above).
3. [ ] **Re-fly in SITL:** the log's case (a `p` climb, then `s` with a small
   bank, once each side); a balloon level off the wingtip (expect an ordinary
   level turn, `lua_plane_path_share` near 0); a balloon well below ahead
   (expect a push, wings level); and the sweep's steep cells, whose results
   were measured without roll-to-plane. Then tune Roll-to-Plane Gain
   and FF from `lua_plane_err_deg` and `lua_plane_drift_dps`: a steady
   `lua_plane_err_deg` while the balloon drifts is what the feedforward
   removes, and one that stays with no drift is the integrator's. Read the
   log's case path angle from `lua_plane_path_deg` (the nose read 74 deg): its
   share depends on it (item 10). With Steep Start low, roll-to-plane takes
   part on moderate climbs and dives, where most of gravity still acts across
   the path: watch level-ish turns there. Near the nose small sideways misses
   become big rolls: watch `lua_plane_err_deg` there.
4. [ ] **Only if item 3 shows the hand-back is too abrupt:** bank-to-turn's
   run-away with a balloon below the wing plane (cause 1) inside the blend -
   bank the other way there, so the balloon stays at 6. Awaits a decision.
5. [x] **Docs:** `ARCHITECTURE.md` § "The roll axis: bank-to-turn and
   roll-to-plane" (bank-to-turn's weak spots, why the bank limit stays a proxy
   limit at steep pitch, roll-to-plane, the blend), the `lua_plane_*` columns in
   § "The flight log format", the STATE + STATE2 size pointed at the
   `protocol.py` comment instead of quoted, and the project tree; the Bank Lim
   Info text; the trap beside the limit in `cascade.lua`.
6. [x] **Stale references:** removed `steep_atan2_roll_law` from the tuner's
   `DISCRETE_CAPTIONS` comment (`ui/controller_autotune.py`) and `law_mode` /
   `steep_entry` from `ARCHITECTURE.md`'s GUIDE table; nothing implemented
   them. The table now matches the GUIDE fields in `protocol.py`.
7. [x] **Roll-to-plane's own gain and feedforward, and every blend setting a
   Gain Tuner row** in the groups right after BOTH LAWS (above).
8. [x] **Docs for items 7 and 9-11:** `ARCHITECTURE.md` § "The roll axis:
   bank-to-turn and roll-to-plane" (roll-to-plane's own gain, integrator and
   feedforward; the blend; the Gain Tuner groups; the log columns), the law-internals row of § "The flight log
   format", and § "FC configuration and the manifest gate" with `CLAUDE.md` §
   "FC configuration is set by hand and only CHECKED at startup" for the
   startup check that passes the Autopilot set whole.
9. [x] **The nose's pitch as roll-to-plane's master switch, and
   roll-to-plane's integrator with its windup** (above).
10. [x] **Defaults: Steep 30 / 70** (the operator's choice).
    The log's case, flown closed loop on roll alone with pitch held
    (`test_balloon_chaser_fc.lua`, at the default gains), nose at 74 deg, for a
    range of path angles at the moment `s` is pressed:

    | Path angle | 30 / 70 (defaults) | 60 / 80 | 50 / 70 |
    |---|---|---|---|
    | 74 | 1.00, level | 0.70, holds 17 deg | 1.00, level |
    | 70 | 1.00, level | 0.50, holds 36 deg | 1.00, level |
    | 66 | 0.90, level | 0.30, holds 69 deg | 0.80, holds 2 deg |
    | 62 | 0.80, level | 0.10, rolls over | 0.60, holds 25 deg |
    | 58 | 0.70, holds 17 deg | - | 0.40, holds 48 deg |

    (roll-to-plane's share, then what the roll does; flown with the clock
    read round the path, item 12.) The real path at `s` in the log is not
    known (the CSV predates `lua_plane_path_deg`); the angle of attack puts
    it below the nose.
11. [x] **The master switch reads the flight path, not the nose** (above).
    `FLIGHT_PATH_MIN_SPEED_MPS` names the speed below which the velocity's
    direction is noise, for the flight-path markers in `track.lua` too.
12. [x] **One path frame for every balloon error.** `track.lua` takes the
    flight path once per tick, before anything reads it, from that tick's
    velocity as read (no filter, glitch guard or clip; the nose at or below
    `FLIGHT_PATH_MIN_SPEED_MPS`), and expresses the balloon in the frame along
    it. Bank-to-turn's and yaw pointing's `pan_err` (`asin(t'_y)`), the pitch
    error (`atan2(-t'_z, hypot(t'_x, t'_y))`) and roll-to-plane's clock
    (`atan2(t'_y, -t'_z)`) all read it, so bank-to-turn and roll-to-plane
    share one zero and the rudder moves none of them. The flight-path markers stay as logged diagnostics. The companion's path
    reads `FLIGHT_PATH_MIN_SPEED_MPS` instead of a bare number.
    `test_balloon_chaser_fc.lua` 5h (markers as read, the pan and pitch errors from
    this tick's path frame, yawing the nose moves no error, a balloon behind
    the path the short way) and 10 (the clock round the path; the shared
    zero). Docs: `ARCHITECTURE.md` § "The flight path is taken once per
    tick, as read", § "The roll axis", § "What each error is measured from";
    `CLAUDE.md` § "Lua steering offload". Item 17 moves roll-to-plane's clock
    to the nose.
13. [x] **No filtered copy of the path left on the FC.** `track.lua` computes
    no angle-of-attack estimate (it was held through a gate and clipped, and
    read as the vertical marker with the sign flipped) and no trimmed
    diagnostic aim (computed every tick, sent nowhere). STATE drops
    `vert_comp_deg`, and the `fpa_gap` column is deleted; `balloon_elev` and
    the `dbg_aim_body_*` columns are written blank. The path's angle off the
    nose is `lua_vel_marker`, as read. Docs: `ARCHITECTURE.md` § "The flight
    log format".
14. [x] **Yaw Coordination FF.** The yaw command is the whole body yaw rate
    ArduPlane's rate loop holds, so with pointing on and nothing else it held
    the yaw near the pointing term's value: in the log's near-vertical climb
    the rudder reached 74% of full against a demand under 2 deg/s, and each
    roll about the nose turned the 7 deg between nose and path into sideslip
    (correlation -0.95 with alpha x sin(roll change)). The command is now
    `yaw_coordination_ff` x (turning yaw `g sin(roll) cos(pitch) / V` +
    rolling yaw `p tan(alpha)`, `p` the commanded roll rate and `alpha` the
    path's angle below the nose from the shared path) + pointing, inside
    `ACRO_YAW_RATE`. Gain Tuner row Yaw Coordination FF (1 = the geometry,
    default 0); any value but 0 runs the rudder loop. `lua_yaw_ff_ari` now
    carries the flown rolling yaw (blank on a tick the FC did not command);
    STATE gains `yaw_coordination_dps` (`lua_yaw_coordination_dps`), with
    `g_yaw_coordination_ff`. `test_yaw_coord.lua` section 7. Docs:
    `ARCHITECTURE.md` § "The yaw axis: coordination and pointing", the yaw
    gotcha, the measured-from table. Not yet flown: the log's climb at 0 and
    at 1, comparing `lua_lat_marker` through the rolls, `lua_out_rudder` and
    the elevator left (`lua_out_elevator`).
15. [x] **The rudder's share near the path, and the pointing schedule on the
    steep share.** Steep and close to the path neither roll law is the tool
    (bank-to-turn has almost no bank to feel, roll-to-plane's clock is jumpy).
    Rudder Near-Path Share (0-1, `rudder_near_path_share`, default 0) hands
    that fraction of the roll command steep and close to the path from
    bank-to-turn to the rudder: the roll command carries nothing for it and
    yaw pointing takes the sideways miss (pointing must be on). Bank-to-turn
    has `1 - w - u`; the pan integrator holds while `w + u > 0`. Logged as
    `lua_rudder_share` and `g_rudder_near_path_share`, deleted with the share
    (item 16). Yaw
    Point Steep Schedule (the `yaw_point_pitch_sched` switch, relabelled)
    multiplies pointing by the roll blend's `path_share` instead of
    `1 - abs(cos(pitch))`. `test_balloon_chaser_fc.lua` section 10 (0, 1 and
    0.5 near the path, nothing on a shallow path, the integrator held) and
    `test_yaw_coord.lua` 4b (level 0, halfway half, past Steep Full all).
    Docs: `ARCHITECTURE.md` § "The blend", § "The rudder's share near the
    path", § "The yaw axis", the measured-from table. Not yet flown: the
    log's near-vertical climb with Yaw Point Kp on, Yaw Coordination FF 1 and
    the share at 1, against bank-to-turn alone. Item 16 removes the share.
16. [x] **The blend reads the path's steepness alone, and Rudder Near-Path
    Share is removed** (the operator's call). Roll-to-plane's share is
    `path_share` and bank-to-turn has `1 - path_share`: the command is
    `(1 - w) x bank-to-turn + w x roll-to-plane`, and roll-to-plane's
    integrator and feedforward run only while `w` is above 0. Removed with the
    setting: its Gain Tuner row, constant, `ControlGains` field and SET_GAIN
    slot (the Curve Flyer's gains move down one index) and STATE2's
    `rudder_share`; the `g_rudder_near_path_share` and `lua_rudder_share`
    columns are deleted. `test_balloon_chaser_fc.lua` section 10 (on a steep path
    roll-to-plane's rate is the whole command and the pan integrator is
    cleared; halfway between the steep angles half of each law). Docs:
    `ARCHITECTURE.md` § "The blend", § "The roll axis", the measured-from
    tables, § "The flight log format".
17. [x] **Roll-to-plane's clock is read round the nose** (the operator's
    call): `atan2(b_y, -b_z)` with `b` the balloon in body axes, in place of
    `atan2(t'_y, -t'_z)` round the path. A roll about the nose turns it by
    exactly the angle rolled; the 12/6 choice and the feedforward's drift read
    it. The steep share, bank-to-turn, yaw pointing and the pitch law still
    read the path, so the two roll laws share one zero only without sideslip,
    and near the nose, where its clock swings, roll-to-plane rolls for every
    swing. In the 85 deg climb of `flight_log_20261007_222608` the nose clock
    read the opposite roll from the path clock on every tick (right to 12
    against left to 6).
    `test_balloon_chaser_fc.lua` section 10 (the clock read round the nose, a
    roll about the nose turning it by exactly the angle rolled, a balloon over
    a sideslipping path reading off 12). Section 15 flies with the nose on the
    path, so its results do not move. Docs: `CLAUDE.md` § "Lua steering
    offload", `ARCHITECTURE.md` § "Roll-to-plane", § "What each error is
    measured from", § "The yaw axis". Why the nose: for a balloon between the
    nose and the path the path's clock asks for the opposite roll, which tilts
    the pull the wing is holding away from the balloon (`ARCHITECTURE.md` §
    "Roll-to-plane"); section 10 pins that case (12 round the nose, the roll
    toward the balloon's side).
18. [x] **Defaults: Steep 65 / 80, Roll-to-Plane Gain 9** (the operator's
    choice). `test_balloon_chaser_fc.lua` at these defaults: section 15's
    three-rules flights roll in 125 of the 230 rule-1 chases and 58 of the 225
    rule-2 chases, and the default `p` balloon rolls 73 deg and ends 17 deg
    from level, 8.4 deg off it; section 10's log's case, on a path at the
    nose's 74 deg (share 0.6), banks out to 11.6 deg from its starting 7.5 deg
    before levelling the wings, which its check fails.
19. [x] **Roll-to-plane's derivative term** (the operator's call):
    `plane_kd` (Roll-to-Plane Kd, deg/s of roll per deg/s, default `PLANE_KD`
    = 0) x the clock error's rate, low-passed at `plane_kd_rate`
    (Roll-to-Plane Kd LPF Rate, default `PLANE_KD_RATE`), inside
    roll-to-plane's share and its integrator's saturation test. The rate is
    the clock angle's, so a 12 <-> 6 change does not reach it; it is held at
    zero with no share and starts afresh on each chase. STATE2 carries
    `plane_err_rate_dps` and `plane_d_dps`; the log appends `g_plane_kd`,
    `g_plane_kd_rate`, `lua_plane_err_rate_dps` and `lua_plane_d_dps` at the
    end of the row. `test_balloon_chaser_fc.lua` 5e (NaN on a tick the law did
    not run) and section 10 (a drifting balloon: the rate settles on the drift
    and the command carries Kd x it; a world-fixed balloon with the aircraft
    rolling toward it: the rate is minus the roll rate and the term rolls
    against the roll; a 12 <-> 6 change leaves the rate the roll's; no share:
    held at zero). Docs: `ARCHITECTURE.md` § "Roll-to-plane", § "The blend",
    § "What each error is measured from", § "The flight log format". Not yet
    flown.

## 8. Run outputs: the sweep, the cell tune and `k`

Not started; work, not a flight test, and like everything here it waits for
approval. Plant-ID (`i`) and the trim map (`e`) write one report, one checkpoint
and one signals CSV (`CLAUDE.md` § "Measurement and tuning runs follow one
pattern"). The controller-autotune sweep (`t`), the single-cell tune (`y`) and
the frequency-ID sweep (`k`) still write their own sets: `t` and `y` a report,
a suggested-constants file and a checkpoint, with no signals CSV; `k` a report,
a results JSON, one CSV per cell and a checkpoint. Bringing them to the
three-file shape changes their export rows with them.

## 9. Control-law audit: bugs and simplification

Work, not a flight test, and like everything here it waits for approval. The
bugs are settled below; of the simplifications, 1, 3, 4, 5, 6 and 7 are done. A
review of the FC laws: `track.lua`, `cascade.lua`, `control_roll.lua`,
`control_pitch.lua`, `pitch_suppression.lua`, `estimator.lua` and the yaw and
trim blocks of `balloon_chaser_fc.lua`. Each bug was reproduced by driving the real Lua
modules on the desk.

### Bugs

1. [x] **The bearing-rate feedforward has the wrong sign.** Removed with Roll
   FF (Simplification 3). `track.lua` builds `ff_bank_deg` from the published
   `pan_err` (path minus balloon); `balloon_chaser_fc.lua` negates the error into the
   cascade's turn-right sense but passes the feedforward through as it is. A
   balloon drifting right (turn-right error 10 -> 12 deg) gives a LEFT bank
   feedforward. It is `-pan_kd` on the same derivative, so raising Roll FF
   removes damping. Inert at the shipped `ROLL_FF_GAIN` = 0, but the `t` sweep
   raises it.
2. [x] **A balloon behind and above or below makes bank-to-turn bang between its
   limits.** Decided: left as it is. `rear_hemisphere_pan_err` blends on
   `-t'_x` alone, so a balloon behind in the pull plane (`t'_y` = 0) - one a
   pull alone reaches, and which roll-to-plane reads as 12 or 6 o'clock - gets
   a 45-78 deg turn error whose sign follows `t'_y` and flips the moment the
   aircraft rolls either way. A kinematic loop on the real modules (no gravity,
   first-order rate response), balloon 45 deg above dead astern: the roll
   command swung between +-ACRO_ROLL_RATE 13-16 times before the pull brought
   the balloon ahead. In the same function the error is not monotonic (90 deg
   abeam, 75 at 120, 90 dead astern), the `REAR_AZIMUTH_TIE_TY` band is
   narrower than noise, and the roll-sign tie-break reads the Euler roll, whose
   sign means the opposite when inverted. A C-04 case; the fix is a design
   decision. The "Near dead astern" limit in § 7 is the related, known one.
3. [x] **At full pull the pitch trim is dropped, and the log does not show
   it.** Logging fixed: the script makes ArduPlane's +-ACRO_ROLL_RATE /
   +-ACRO_PITCH_RATE cut itself, so `lua_omega_x/y_cmd` log the command flown.
   Decided: ACRO_PITCH_RATE stays at its shipped value, and the pitch
   integrator's anti-windup is not told about the cut. `balloon_chaser_fc.lua` adds
   the turn term and the trim after the cascade's pitch clip; ArduPlane then
   constrains the total to ACRO_PITCH_RATE (`commands_logic.cpp`,
   `set_target_throttle_rate_rpy`, at this firmware). With the law at the
   envelope the trim request is cut - about 17 deg/s level at 13-17 m/s and 30
   deg/s at 45 deg nose-down, with ArduPlane's default SCALING_SPEED - and
   `lua_omega_y_cmd` logs the unclipped sum. The yaw command mirrors the FC's
   ceiling; pitch does not, and the pitch integrator's anti-windup does not see
   the cut either.
4. [x] **A hidden pitch-rate cap.** Fixed: the envelope is ACRO_PITCH_RATE.
   `cascade.lua` sizes the pitch envelope as
   `min(ACRO_PITCH_RATE, ACRO_PITCH_RATE_DEFAULT)`: a fallback used as a
   ceiling. Raising ACRO_PITCH_RATE (headroom for item 3) changes nothing,
   contrary to the startup check's "the FC's own value is what the law then
   uses".
5. [x] **`lua_pan_dot_lpf` is always blank.** Moot: the filter went with Roll
   FF, and the column is deleted with its other columns. `step()` reads
   `cs.log_pan_dot_lpf`, which nothing writes; the filter is `cs.pan_dot_lpf`.
6. [x] **The yaw-pointing integrator winds while the steep schedule zeroes its
   output** (level flight, Yaw Point Steep Schedule on, Ki above 0) and
   releases up to Yaw Point Lim x Windup as the path steepens. Fixed: it grows
   by the same share.
7. [x] **The coordination yaw rate divides by EAS;** the turn's kinematics need
   true airspeed. It over-states by EAS2TAS - about 3% at the world's home
   altitude, about 8% 1000 m above it - so Yaw Coordination FF = 1 is not quite
   the geometry. The pitch turn term reuses the number. Fixed: the floored EAS
   times EAS2TAS.
8. [x] **Comment mismatch:** the yaw-decoupling feedforward claims "one plant
   at every attitude" and acts only in climbs (`sin(pitch) > 0`). Fixed: the
   comment says why it is climb-only.

Checked and correct: roll-to-plane's P, feedforward and 12/6 signs; the
bank-proxy rate split; the coordinated-turn formulas; the NaN guards.

### Simplification

Of the 46 SET_GAIN settings the audit found, 10 law terms shipped at 0 and 7
more acted only when one of those was on.

1. [x] **Pitch suppression** (18 modes, `pitch_suppression.lua`, its context in
   `cascade.lua`, the `PSUP_*` constants, GUIDE's `psup_mode`, the `j` key):
   off by default. Removed whole: the module, the cascade's envelope scaling
   and the previous-command carry it read, GUIDE's `psup_mode`, STATE's
   `psup_scale`, STATE2's `psup_raw`, the `j` key and the HUD label. Its log
   columns (`psup_mode`, `psup_scale`, `psup_raw`) are deleted.
2. [x] **Roll Law 0 and Vertical Fade:** the legacy Euler law. Removed whole:
   the roll-law selector and the vertical fade on the FC (the roll law feeds
   back the bank proxy only), GUIDE's `roll_law` and `cfg_flags` (GUIDE is 2
   bytes shorter; `PROTO_VERSION` 4), `set_law_config`, `ROLL_LAW` and
   `VERT_FADE_ENABLE`, the two Gain Tuner rows, the sweep and freq-ID
   checkpoint keys, and its log columns `g_roll_law` and `g_vert_fade_enable`
   (deleted, so every later column moved two places left).
3. [x] **Bearing-rate feedforward** (`roll_ff_gain`, `ff_lpf_rate`): the same
   derivative as `pan_kd` (bug 1). Removed: the FC law, its three STATE fields,
   the two gains, Gain Tuner rows and sweep steps. Its five log columns
   (`ff_bank_deg`, `pan_dot_raw`, `pan_dot_lpf`, `g_shallow_roll_ff`,
   `g_ff_lpf_rate`) are deleted.
4. [x] **Yaw-decoupling feedforward** (`roll_decouple`, `FF_UPRIGHT_*`, the
   inline floor on `AIRFRAME_ROLL_RESP_GAIN`): off. Removed: the FC term,
   STATE2's `roll_ff_decouple_dps`, the gain and its Gain Tuner row, and
   freq-ID's forcing it to 0. Its log columns (`roll_ff_decouple_dps`,
   `g_shallow_roll_decouple`) are deleted.
   `AIRFRAME_ROLL_RESP_GAIN` stays, as the measured roll response.
5. [x] **BTT rate ceilings** (`btt_rmax_roll`, `btt_rmax_pitch`): 0 = none,
   lowering ACRO_ROLL_RATE / ACRO_PITCH_RATE does the same, and they are
   applied in three places. They have no Gain Tuner row: `constants.py` is the
   only way to set them. Removed: the law's envelope and ArduPlane's final cut
   are the ACRO rates alone. Their log columns (`g_btt_rmax_roll`,
   `g_btt_rmax_pitch`) are deleted.
6. [x] **Identities and dead paths:** the pitch aim vector (`theta_ip` equals
   `e_pitch` to 1e-16 over the sphere; pass the angle); `body_from_world_R`'s
   Euler fallback and `quat_fresh` (always true on the FC); the tilt
   clip-and-rescale in `track.lua`; `update_bodyplane_pitch_integrator`'s
   `enabled`; the estimator's no-readback branches. Removed all but the last,
   plus the estimator's uncalled `get_into` and `snap_to`. The tilt clip
   changed nothing at GIMBAL_TILT_MAX = 90 and would only have clipped a balloon
   angle that is not a gimbal angle. Kept: the no-readback trust branch, the
   provision for a gimbal that reports its commanded angle rather than a
   measured one.
7. [x] **Unused outputs:** `out.last_target_body_*`, `out.pitch_demand_gap_rad`,
   the local `pitch_deg` in `apply_control_law`, `cs.log_req_pitch_raw` written
   twice. `pitch_weight` logged |cos(roll)| for a weighting the law does not
   have. Removed, with track.lua's unread `log_det_norm_x/y`; `pitch_weight`
   leaves STATE and its column is deleted.
8. [ ] **The lateral axis** has three laws, three integrators and three
   feedforwards behind two ramps and a hand-off. The yaw-pointing set (Kp, Ki,
   Lim, Windup, Steep Schedule) ships off: fly it
   (§ 7), then keep or cut.

### Question

- **The world-balloon estimator lags by about 1 / WORLD_EST_ALPHA_NOM** at full
  trust. A balloon crossing at 5 m/s at 100 m leaves about 6 deg of lag against
  ALIGN_TOL_DEG. Measure in `x` before trusting `x` on a moving balloon.
