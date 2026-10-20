# ARCHITECTURE.md — Companion

**This file is documentation, not instructions.** It explains how the system
works and why the design is the way it is: subsystem mechanisms, design
rationale, worked examples, measured evidence, and reference tables for reading
what the system produces. The RULES — what must be true, what must never be
done, the directives and conventions Claude follows — live in `CLAUDE.md`,
together with the technical knowledge Claude acts on: the procedures it carries
out on every change and the ArduPilot traps a change has to avoid. Where a rule here appears to conflict with `CLAUDE.md`, `CLAUDE.md`
wins and this file is the stale one; each section below points back to the rule
it explains.

This file is still NOT a lab notebook. Per-run investigation notes belong in a
comment beside the code they describe, never here — see the header of
`CLAUDE.md`. What lives here is standing explanation of the current design, not a
record of how it came to be.

Contents:

- [The `p` / `x` tests](#the-p--x-tests)
- [Tuner edits are live in flight](#tuner-edits-are-live-in-flight)
- [The FC owns every value — why](#the-fc-owns-every-value--why)
- [The flight log format](#the-flight-log-format)
- [FC configuration and the manifest gate](#fc-configuration-and-the-manifest-gate)
- [plant-ID design](#plant-id-design)
- [Tuning-run outputs: one report, one checkpoint, one signals CSV](#tuning-run-outputs-one-report-one-checkpoint-one-signals-csv)
- [Measurement runs: the shared pattern](#measurement-runs-the-shared-pattern)
- [Trim map design (`e`)](#trim-map-design-e)
- [Telemetry rates: attitude, position, the oracle pull](#telemetry-rates-attitude-position-the-oracle-pull)
- [PID_TUNING: FC rate-loop internals](#pid_tuning-fc-rate-loop-internals)
- [Project structure](#project-structure)
- [Lua steering offload](#lua-steering-offload)
- [The roll axis: bank-to-turn and roll-to-plane](#the-roll-axis-bank-to-turn-and-roll-to-plane)
- [The yaw axis: coordination and pointing](#the-yaw-axis-coordination-and-pointing)
- [What each error is measured from: path, nose or gravity](#what-each-error-is-measured-from-path-nose-or-gravity)
- [Lift Vector Pointing: the second steering law](#lift-vector-pointing-the-second-steering-law)
- [The Curve Planner and the Curve Flyer](#the-curve-planner-and-the-curve-flyer)
- [The two autotune subsystems](#the-two-autotune-subsystems)
- [Controller-Autotune stop/resume and HUD timers](#controller-autotune-stopresume-and-hud-timers)
- [Sweep / plant-ID operator display](#sweep--plant-id-operator-display)
- [Sweep report: cells that never converged](#sweep-report-cells-that-never-converged)
- [Camera view switches](#camera-view-switches)
- [FOV measurement (diagnostic)](#fov-measurement-diagnostic)
- [Simulator air: AltitudeLiftDrag](#simulator-air-altitudeliftdrag)
- [SITL from conda-forge, where Gazebo's package host is unreachable](#sitl-from-conda-forge-where-gazebos-package-host-is-unreachable)
- [Deliberate stops: where the program halts on purpose](#deliberate-stops-where-the-program-halts-on-purpose)
- [How the code hash (`code_sha256`) is calculated](#how-the-code-hash-code_sha256-is-calculated)

---

## The `p` / `x` tests

*Rules for these tests live in `CLAUDE.md` § "What the `p` and `x` tests are
for". This is the mechanism.*

The point-balloon-chaser test (`p` — `testing/point_balloon_chaser.py` + `testing/synthetic_injector.py`)
exists to **simplify development by isolating the vision path**. It hands the
controller ground truth directly: `SyntheticBalloonInjector` anchors a world-space
balloon and feeds it to the sensor's ORACLE interface (`set_oracle_world_position` /
`set_oracle_world_direction`), so no camera, no detector, and no gimbal encoder sits
in the loop. What remains under test is only the **control law and the navigation** —
steering error → the selected steering law (the cascade or Lift Vector Pointing) →
roll/pitch rate.

`x` is the same test with the vision path connected. That is the difference, and per
C-03 it must remain the only one: both call the same shared control function and
differ solely in the source of the balloon world vector (oracle vs. sensor).

Consequences:

- **Balloon POSITION is longitude/latitude; pan/tilt means motion.** Where the
  injected balloon sits is `POINT_LON_DEG` / `POINT_LAT_DEG` — longitude around the
  aircraft (0 ahead, +90 right, 180 astern, -90 left, referenced to the heading at
  injection) and latitude up/down (+90 zenith, 0 level, -90 nadir, referenced to
  the world horizon). This is a bearing on a sphere centred on the aircraft, **not
  WGS84** — the aircraft's real geographic position is `link.lat` / `link.lon` and
  is unrelated. `pan` and `tilt` are reserved for MOTION or DESIRED motion: the
  nose-to-balloon steering error the cascade closes, the gimbal's aim, the control
  axis a sweep cell is stressing, and the moving balloon's *direction of travel*
  (`POINT_BALLOON_VEL_PAN` / `VEL_TILT`, which stay pan/tilt for exactly that
  reason). Same spherical frame throughout; only the thing being described
  changes. The autotune grid keeps pan/tilt for its cells because a cell names
  which control loop it stresses — `_inject_arm_warmup` is the boundary where a
  grid cell becomes a balloon position, and the vocabulary switches there.

- A `p` failure is a control/navigation failure. Something that passes `p` and fails
  `x` is a SENSING failure — detector, gimbal tracking, encoder calibration, or
  estimator trust — not a gains problem. Bisect in that order.
- C-02's "p-test truth may not use FOV" is satisfied by construction, not by care:
  the oracle path never touches a pixel. The encoder-only rule for the body-frame
  direction is an `x`-path obligation.
- Because `sensor.oracle_active` is True for the whole run, every branch that would
  swap in a detection-derived vector is short-circuited (`_refresh_vision_display_balloon`,
  and the inline override at the end of `_control_balloon_chaser`). So `_last_balloon_body` —
  and therefore `flight_path_errors()` / `alignment_errors()` — is oracle-derived and
  FOV-free under `p`, but detection-derived under `x`. The scoring accessors are NOT
  interchangeable between the two modes; under C-02, check which mode you are in
  before trusting them.

- The alignment criterion is ONE angle, not two. Aligned is
  `hypot(pan_err, tilt_err) < ALIGN_TOL_DEG` on `alignment_errors()` — the
  combined miss angle between the FLIGHT PATH and the balloon — held 0.5 s,
  with no deadline: the competition asks for best effort, a missed balloon the
  worst result, so the `p` test reports when the path aligned, or how close it
  came if stopped first. The flight path, not the nose: an aircraft flies a path
  shallower than its nose, and the path is what reaches the balloon. It is
  deliberately NOT "each axis under the tolerance", which would be looser (pan 0.4
  with tilt 0.4 leaves both axes inside 0.5 while the path is 0.57 deg off) and
  would make the threshold depend on how the error happened to split between the
  axes. The same test — `controller.aligned_now()` — drives the align-seconds
  counter, `_resolve_aligned_seconds()`, the HUD's "Path Aligned" indicator and timer,
  the log's `in_tol` column and the sweep's `ALIGN_TOL_DEG` gate, so a `p`
  ALIGNED means the same thing the operator watched on screen. Single source of truth:
  `constants.ALIGN_TOL_DEG` and `controller.alignment_errors()`. The flight path
  there is the FC's own 50 Hz velocity (STATE2), rotated with the same attitude
  as the balloon geometry; the MAVLink velocity is a 5 Hz staircase. The HUD's
  "Nose Aligned" lamp is a separate readout of the nose, not an alignment test.

- Balloon motion is controlled LIVE from the Gain Tuner: both
  `POINT_BALLOON_MOVE_ENABLE` and `POINT_BALLOON_SPEED` are polled every injector
  tick, and each change starts a new motion leg from the balloon's CURRENT
  position, so neither can teleport it. Pausing is itself a speed change (to
  zero), which is why the two go through one code path. What stays frozen at
  `start()` is the DIRECTION of travel — every leg of a run lies on the one
  world-frame line chosen at injection, whatever the speed, which is what keeps
  a run repeatable. The sweep and plant-ID pass no live callbacks at all, so
  their balloons are stationary and bit-repeatable by construction: nothing the
  operator does to the tuner can perturb a calibration run.

---

## Tuner edits are live in flight

*The rule — new modes that select their own gains must go through
`begin_measurement_gains` / `end_measurement_gains` — lives in `CLAUDE.md`
§ "Tuner edits are live in flight". This is the mechanism.*

A gain changed in the Gain Tuner reaches the FC on the NEXT TICK. The operator
turns a knob and the aircraft responds; there is no stop-and-restart step and no
"apply" button. The Steering Law switch is the one edit that does more: the law
is part of the FC's chase key, so the tick it lands starts a fresh chase - both
laws reset, any PLAN and held line dropped on the FC and on the companion, and
with the Curve Flyer on a `p` or `x` chase waits again for a PLAN made for the
new law.

`_push_gains_if_changed()` runs once per tick, compares the gains BY VALUE, and
calls `sync_gains` only when they differ - so a slider dragged back where it
started costs nothing, and an unchanged set costs one tuple comparison. A failed
push does NOT record the signature, so it retries next tick rather than leaving
the FC on stale gains while believing it is current.

**While a measurement owns the gains, the law flies its private set.** A
closed-loop measurement (the trim map `e`, freq-ID `k`) calls
`begin_measurement_gains()` to install a private copy for the duration of the
run, and `update_measurement_gains()` to change it between cells.
`_push_gains_if_changed()` always pushes `self._gains` - the set the law flies,
which is that private copy during a run - so the FC receives it on the next tick.
The tuner writes the operator's set (`operator_gains()`), not `self._gains`, so an
edit made during a run changes nothing the push sees: it stays live and editable,
and lands when the run gives the gains back, which `end_measurement_gains()`
forces by clearing the signature. Skipping the push while a measurement owns the
gains is the trap: the FC then flies the operator's set for the whole run while
the log records the measurement's. `test_control_path.py` sends a private set, a
change to it and an operator edit through the real push and link packing.

### Delivery: how an edit reaches the law

There are two routes, and both land on the next tick:

- **Gains** go out as `SET_GAIN` frames, the whole set at once (one frame per
  gain) on the tick they change, ahead of that tick's GUIDE. The
  script writes each into its gain table as it parses it, and the law reads the
  table every tick, so the new value is flown on the tick it arrives.
- **What rides in GUIDE** (the throttle, the straight and manual setpoints) is
  packed into every GUIDE frame and copied into the script's inputs every tick;
  it needs no push.

Frames reach the port from several threads — GUIDE from the control loop, the
full gain push from the tracking start's thread, single gains from the tuner and
the sweep — so `LuaOffloadLink._send()` is the one place that writes it, a whole
frame at a time under `_tx_lock`. Two writes that overlap under back-pressure
would otherwise interleave their bytes, both frames would fail the FC's CRC, and
a lost `SET_GAIN` is not re-sent.

On the FC, `rx_pump` makes one read of at most `RX_BUF_MAX` bytes per tick, so a
full gain push can straddle two ticks; the rest waits in the port. The buffer is
parsed as read, never trimmed first. What a tick leaves behind is the head of a
frame whose tail arrives next, and a trim on a full read — which is exactly what
a backlog produces — cuts that head off and loses the frame without a trace. The
buffer stays bounded without a trim: the parser consumes every complete frame and
skips garbage byte by byte, so it never leaves more than one frame (4 + 255 + 2
bytes) behind.

Two tests pin the route end to end. `test_control_path.py` makes an edit through
the real tuner store, runs the controller's per-tick push and the real link
packing, feeds the bytes to `balloon_chaser_fc.lua` under the desktop stubs, and checks
that the pitch-rate command doubles on the tick a doubled `pitch_gain` lands, that
nothing is re-sent while nothing changes, and that a GUIDE-carried setting is in
the next GUIDE. `test_balloon_chaser_fc.lua` splits a gain frame across two reads with a
full read behind it and checks the gain still takes effect.

What is NOT confirmed: the FC does not report its gains back. The companion
records a push as done once it is written, and a frame the FC discards for a bad
CRC — possible on a real serial line, not on SITL's — is not re-sent until the
next edit.

### Measurement gains are PRIVATE to the run

`begin_measurement_gains(reason, **overrides)` / `end_measurement_gains()`.
The control law flies a private copy; the Gain Tuner keeps reading and editing
the OPERATOR's own set. Nothing appears to move on screen and the override is
stated in the LOG, which is where a transient procedural fact belongs.

Forcing the values in place instead is wrong for a reason worth stating: the
tuner reads and writes the same object, so sharing it with a run flips the
switches to the measurement values in front of the operator,
invites a "correction" mid-run, and then silently overwrites that correction at
the restore. Two writers, one variable, no way to tell whose value won. An edit
made during a run must survive the restore and take effect when the run
finishes.

Not re-entrant: nested runs would make "the operator's set" ambiguous.

---

## The FC owns every value — why

*The rule lives in `CLAUDE.md` § "The FC owns every value it computes." This is
the reasoning behind it.*

The flight controller is the only authority on steering. If the companion needs one of its values — for the HUD, the CSV, the
tuner, plant-ID, anything — that value is put on the STATE or STATE2 frame and
sent. It is never recalculated on the companion side from telemetry.

The failure this prevents is silent and total. A recomputed value is plausible,
smooth, and correlated with the real one, so nothing looks wrong: the HUD shows a
number, the CSV has a column, the tuner responds. What it does not show is the
FC. Anyone reading that log is tuning a simulation of the aircraft against
telemetry the aircraft never acted on.

Identification is where it bites hardest. A fit against a rate the aircraft never
received is not a measurement of the aircraft, and nothing in the result says so:
the curve is smooth, the residuals are small, and the number is wrong. Any tool
that fits, differentiates or integrates a control quantity reads the `lua_*`
mirror, never the companion's own.

Bandwidth is not a reason to break the rule. STATE and STATE2 are raw serial
frames, so the cap is the line rate and the line rate is a parameter — the port runs at 921600
and `port:begin()` in `balloon_chaser_fc.lua` must match the pyserial baud in `main.py`
exactly. Send the value and raise the baud; do not recompute it to save bytes.

The companion keeps only what has no FC equivalent: the camera, the detector, the
gimbal, the operator's inputs, and the estimator inputs it feeds forward in
`GUIDE`. Everything downstream of the steering law belongs to the FC. The one
exception is the Curve Planner: it computes the PLAN, and its model of each law's
plan mode is the one copy of a flying law in Python. It flies candidate plans the
FC has not flown, to choose one, and never stands in for a value the FC computed
(§ "The Curve Planner and the Curve Flyer").

The companion does keep its own view of the balloon geometry, for the HUD markers
and the `p`-test scoring (`dbg_balloon_body_*`, `dbg_control_body_*`,
`align_pan/tilt_deg`, `roll_align_err`): built from its estimator and the MAVLink
attitude, it is an independent check of the outcome, not a second law. The FC's
view of the same geometry is logged beside it (`lua_balloon_body_*`, `los_elev_deg`),
and the gap between the two is the timing and attitude difference between the
two ends of the link.

### Log the signal, never a presentation of it — the two lookalikes

*The rule lives in `CLAUDE.md` § "Log the signal, never a presentation of it".
These two consequences look like violations of it and are not.*

- **A control step is not a presentation step.** `steering()` in
  `balloon_chaser_fc.lua` passes `-pan_err` into the cascade when the cascade
  is the law selected (Lift Vector Pointing reads the balloon in body axes
  instead), once, at the
  boundary: the cascade's convention is "how far to turn right", while
  `pan_err` means "where the flight path is relative to the balloon". `pan_err`
  is logged as it stands (`lua_pan_err_deg`, `steering_pan_deg`). Turning an
  error into a correction is arithmetic the law does; it is not formatting.
- **Two columns for nearly the same angle may legitimately disagree.**
  `steering_tilt_deg` is the balloon's tilt from the NOSE and settles at -AoA by
  design; `align_tilt_deg` is its tilt from the FLIGHT PATH and settles at zero.
  They are different signals and each is logged as it stands. Document the pair;
  do not reconcile them in the writer.

---

## The flight log format

*The rules — append columns, never insert; analyse from the mirror columns; an
absent value is blank; the FC's condition goes in the log, not on its console —
live in `CLAUDE.md` §§ "The flight log", "Everything that participates in the
control law is recorded" and "The FC console is for faults, not for status".
This is the format.*

A `q` press writes exactly ONE file, `flight_log_*.csv`: the human-readable
flight record. The head carries `# code_sha256:`, the drained startup log and any
`# note:` lines (all prefixed `# ` so `csv.reader` / `pandas(comment='#')` skip
them); every data row is `HEADER` in column order. A message that spans several
lines gets the prefix on each one (`_comment_lines`): an unmarked continuation
line is read as a data row before the header, where pandas stops and a
spreadsheet import shifts every column.

The row is the steering story as well as the outcome, because the FC mirrors its
own internals back in the `STATE` and `STATE2` frames and those land in the
`lua_*` columns, beside the rate loop's own record in the `fc_*` columns
(`PID_TUNING`). So the log says both what happened and what the steering saw
and decided to make it happen — the mirror columns are the record of the flying
law, not just its result. A measurement's excitation is there too:
`lua_excite_value` is what `EXCITE` adds to its axis's rate command (freq-ID
`k`) or replaces it with (plant-ID `i`).

An absent value is written BLANK, never zero, so it cannot be read as a
measurement:

- **Columns with no source.** `pwm_ch1/2` and `rate_cmd_pitch/roll` (nothing
  sends stick PWM or computes a rate on the companion) and `aoa_lat` (always 0 in
  the law). `req_pitch_raw` would only repeat `lua_requested_pitch`.
  `balloon_elev`, `dbg_aim_body_x/y/z` and `dbg_theta_ip_body_deg` name an
  angle-of-attack-trimmed aim and estimate the FC does not compute: the pitch
  law's error is `dbg_theta_ip_deg`, the balloon's elevation is
  `los_elev_deg`, and the flight path's angle off the nose, as read, is
  `lua_vel_marker`. A column whose function is removed completely is deleted
  with it (CLAUDE.md "The flight log"), so the columns after it move up one
  place and a log is read by its header names, never by position.
- **STATE's law internals and commands.** STATE's `flags` bit 3 (`law_ran`)
  says whether the law ran this tick (gates 0, 6 and 8). With it clear, every
  law-internal float on STATE is NaN; the rate and throttle commands, and the
  yaw command's parts (`lua_yaw_point_dps`, `lua_yaw_ff_ari`,
  `lua_yaw_coordination_dps`), are NaN on any tick the FC did not command. Only
  the law that ran fills its own internals: on a tick Lift Vector Pointing flew,
  the cascade's STATE floats, the yaw columns and `lua_pitch_turn_dps` are NaN,
  and on a tick the cascade flew, the `lua_att_*` fields are; `bank_proxy_deg`
  is filled under both. Flag bytes cannot carry NaN, so the companion
  decodes them to None instead: `cs_flags` and its bits when the law did not run,
  the cascade's bits (`roll_sat`, `pan_i_frozen`, `pan_i_clamped`,
  `pitch_i_frozen`, `pitch_i_clamped`) when bit 6 says Lift Vector Pointing
  flew, bank-to-turn's three when bit 5 says a PLAN set the bank, and
  `asp_flags` (`asp_en`, `asp_sat`) on any tick but gate 0.
  `lua_scaler` is NaN with no airspeed estimate.
- **STATE2's law internals** (`lua_balloon_body_*`, `los_elev_deg`,
  `bank_rate_cmd_dps`, `bank_rate_coupling_dps`, `sideslip_deg`, `aoa_deg`,
  `lua_yaw_point_i_dps`, the `lua_plane_*` columns) on a tick the law did not run - `lua_gate` says why - and `sideslip_deg` / `aoa_deg` below 1 m/s.
  The cascade's (the bank-rate split, `lua_yaw_point_i_dps`, `lua_plane_*`) are
  NaN too on a tick Lift Vector Pointing flew; `sideslip_deg` and `aoa_deg` then
  come from Lift Vector Pointing and are blank at or below
  `FLIGHT_PATH_MIN_SPEED_MPS`. The FC sends NaN; the companion decodes it as None.
- **STATE2's sensor columns** where the FC had no reading (`lua_eas_mps` with no
  airspeed estimate, for example).
- **The Curve Flyer's internals** (`lua_cf_progress`, `lua_cf_eps_deg`,
  `lua_cf_bank_ref_deg`) on any tick the law flew no PLAN, and the line
  (`lua_cf_hold_n/e/d`) except in a hold and in a run-in's line-up and RUNIN;
  `lua_cf_state` and `lua_cf_plan_id` are meaningful on every tick. The plan's
  own columns (`cf_plan_*`) are blank until the Curve Planner has a plan ready,
  and its run-in's two (`cf_plan_runin_climb_deg`, `cf_plan_runin_dep_deg`)
  for a plan without one.
- **Lift Vector Pointing's internals** (the `lua_att_*` columns) on any tick it
  did not run, and the cascade's on any tick Lift Vector Pointing flew instead
  (§ "Lift Vector Pointing: the second steering law").

STATE2's columns, in the units the FC holds:

| group | columns |
|---|---|
| pairing | `lua_state2_seq` - equal to `lua_seq` when the row's two frames are from the same tick |
| the FC's sensors, this tick | `lua_fc_mode`; `lua_q_w/x/y/z` (world->body); `lua_gyro_x/y/z_rads`; `lua_vel_n/e/d_mps`; `lua_eas_mps`; `lua_eas2tas` (true airspeed per EAS, sqrt(sea-level / local density); the pitch trim's density ratio is its inverse square) |
| law internals | `lua_balloon_body_x/y/z` (the balloon direction, body frame: what the law flew toward unless a PLAN aimed it at a line); `los_elev_deg`; the bank-rate split; `sideslip_deg`, `aoa_deg`; `lua_yaw_point_i_dps`; `lua_coord_valid`, `lua_accel_valid`, `lua_lat_marker_valid`; roll-to-plane's `lua_plane_path_share`, `lua_plane_path_deg`, `lua_plane_path_from_vel`, `lua_plane_err_deg`, `lua_plane_side`, `lua_plane_i_dps`, `lua_plane_i_frozen`, `lua_plane_err_rate_dps`, `lua_plane_d_dps`, `lua_plane_drift_dps`, `lua_plane_ff_dps`, and the unflown `lua_plane_off_nose_deg`, `lua_plane_axis_up` (§ "The roll axis"); the path's own turn, `lua_path_turn_up_dps`, `lua_path_turn_right_dps`; Lift Vector Pointing's `lua_att_theta_deg`, `lua_att_u_y`, `lua_att_u_z`, `lua_att_omega_dps`, `lua_att_f_y_mps2`, `lua_att_f_z_mps2`, `lua_att_mu_deg`, `lua_att_side` (§ "Lift Vector Pointing: the second steering law") |
| the surfaces | `lua_out_aileron/elevator/rudder` (what ArduPlane asked of each, before the V-tail mix, +-4500 = full travel); `lua_out_throttle` (percent); `lua_pwm_aileron`, `lua_pwm_vtail_l/r` (the servo signal after the mix; 0 = no channel carries it) |
| the Curve Flyer | `lua_cf_state` (0 off, 1 waiting for the PLAN, 2 flying the turn, 3 turn complete, 4 no PLAN in time, 6 the turn's line held, 7 a run-in holding its line, 8 the dive after it: `CF_STATE_*`; 5 is unused); `lua_cf_plan_id` (the PLAN taken this chase, 0 for none); `lua_cf_progress` (0-1 through the turn or the dive; 1 while a line is held); `lua_cf_eps_deg` (the path's angle off the law's target: the balloon, or the line it holds or turns onto); `lua_cf_bank_ref_deg` (the bank the law was handed, in its own measure: the schedule's, or the bank toward the target); `lua_cf_hold_n/e/d` (the line, world unit vector: a hold's, or a run-in's) (§ "The Curve Planner and the Curve Flyer") |

STATE carries the rest of Lift Vector Pointing's internals (`lua_att_*`), the
pitch trim's lift term (`lua_pitch_trim_lift_term`) and `lua_accel_x`; the law's
gains are the `g_att_*` columns and the switch `g_steering_law` (§ "Lift Vector
Pointing: the second steering law" → "The log").

The Curve Planner's side of a chase is logged from the companion, beside the
FC's: `cf_plan_id`, `cf_plan_eps0_deg`, `cf_plan_knot_1_deg` …
`cf_plan_knot_5_deg`, `cf_plan_runin_climb_deg`, `cf_plan_runin_dep_deg` and
`cf_plan_law` (the PLAN as sent) and `cf_plan_s` (the seconds it took to
make). `g_cf_transfer_when_done` is the Transfer Control When Done switch the
chase flew with, `g_cf_plan_wait_s` the Plan Wait and `g_steering_law` the law
that flew it.

The FC's own condition is in the record, per tick, rather than on its console:

| column | what it says |
|---|---|
| `lua_gate` | which gate stopped the tick: 0 commanded, otherwise the reason in `protocol.GATE_NAMES`. A per-tick value, not a once-a-second tally |
| `hz_quat`, `hz_gyro`, `hz_vel`, `hz_asp` | how often each reading actually CHANGED in the last second: the only way to tell a live feed from the same stale value read fifty times (§ "Measured, not assumed") |
| `lua_scaler` | ArduPlane's speed scaler on this tick, blank with no airspeed estimate. The rate loop's gain multiplier is its SQUARE, so identification squares it before use |
| `lua_dt_ms` | the measured interval since the previous script tick |

A console line that repeats once a second is read for a minute and then ignored,
and it crowds out the messages that matter: ArduPilot's own STATUSTEXT shares
that budget. The CSV is read when something has gone wrong, which is when the
detail is wanted.

A flight-log column is appended to `HEADER` and to the `writerow` list in
`csv_logger.py`, both in the same order. `test_csv_alignment` checks the two stay
the same length and that no column carries another's value.

The configuration is in every row, not only the outcome. Each `ControlGains`
field has a `g_*` column — most as `g_<field>`, a few under older names such as
`g_shallow_pan_p_gain`. All are read from the set flying that tick, so a tuner
edit appears on the next row and a measurement's private set is what a
measurement's rows show. `test_csv_alignment` traces each gain from its source
to `write_tick`.

The saved ArduPlane parameters follow, one column each, named exactly as on the
FC (`PTCH_RATE_P`, `RLL2SRV_TCONST`, …): the value the FC last confirmed, as the
Gain Tuner holds it (§ "The Gain Tuner's Write, reset, Read and Read All"). Nothing
re-reads the FC when logging starts or on any tick. The rate-loop columns change
at each mode switch, because each mode flies its own set (§ "Each mode flies its
own rate loop"), so a row says what the FC held on that tick.

Any analysis of a run reads the flight log, and reads the FC's mirrored internals
in it — not only the outcome columns. A diagnosis from the outcome alone is
guesswork about the loop that produced it: an oscillation visible in a rate
channel can be attributed to "the controller" but not to a particular loop
without the mirror columns that show what each stage commanded.

---

## FC configuration and the manifest gate

*The rules — FC parameters are set by hand and only checked at startup; identity
checks read `_FUNCTION` never `_TRIM`; don't silence a Stage-1 failure by deleting
names — live in `CLAUDE.md` § "FC configuration is set by hand and only CHECKED at
startup". This is how the checks and the gate work, and why.*

Every FC parameter the system depends on is saved on the FC and set by hand
(MAVProxy `param set`, or the Gain Tuner's Write or reset for the parameters
ArduPlane AUTOTUNE tunes). Startup reads and compares; it writes none of them,
never reboots the FC, and `shutdown()` puts nothing back. The one automatic write at
startup is the stream-group rates, which do not survive between runs (§ "Why
startup writes nothing" below); the tuner's Write and reset are the operator's own
(§ "The Gain Tuner's Write, reset, Read and Read All" below); and each mode switch writes the
rate loop the new mode flies, leaving the operator's values on the FC at exit (§
"Each mode flies its own rate loop" below).

### How a check runs

`read_fc_config` reads each parameter with `_check_param(name, expected, …)`: a
match logs ✓; a difference, or an absent parameter, is logged ✗ with the expected
value and recorded by `_record_mismatch` — never corrected. `_check_param` returns
the FC's own value, so code that uses the number (`acro_pitch_rate`, the pitch
limits) runs on what the aircraft actually has. Pitch limits are bounds rather
than exact values: `PTCH_LIM_MIN_DEG` must be at most `PTCH_LIM_MIN_EXPECTED_DEG`,
`PTCH_LIM_MAX_DEG` at least `PTCH_LIM_MAX_EXPECTED_DEG`.

At the end of `read_fc_config`, `halt_on_param_mismatches` prints every recorded
difference in one list and HALTS on the console until the operator presses Enter
(continue with the FC's values) or Ctrl+C (stop); with no console to confirm on it
refuses to start. It halts rather than aborts because a difference can be
intended — a comparison flight — and the operator is the one who knows.
`lua_setup.ensure_scripting` checks the Lua-path parameters (`SCR_*`,
`SERIALn_*`, `ARSPD_*`, `ACRO_YAW_RATE`, `YAW_RATE_ENABLE`) the same way, earlier
in startup, with its own list and halt; when the deployed scripts changed it
still reloads the Lua VM in place (`MAV_CMD_SCRIPTING`), which is a command, not
a parameter.

### The rate-loop settings: the Acro set or the Autopilot set, whole

`<AXIS>_RATE_{FF,P,I,D,IMAX,FLTT,FLTE,FLTD,SMAX}` for pitch, roll and yaw are the
inner-loop tune — every value the plant-ID report covers.
`_check_rate_pids()` fetches each one, then compares the whole read with the two
sets the FC can hold, each value to `RATE_PID_MATCH_REL_TOL` (with
`RATE_PID_MATCH_ABS_TOL` as the floor for values at or near zero):

- **The Acro set,** `EXPECTED_RATE_PIDS` in `constants.py`, matched in full:
  passes.
- **The Autopilot set,** `NATIVE_MODE_RATE_PIDS` over the Acro set's filters,
  matched in full: passes too. It is what a switch out of ACRO leaves (§ "Each
  mode flies its own rate loop"), and what the FC keeps when the program stops
  outside ACRO without its exit hand-back - a killed process, a closed
  terminal, a link lost at exit. The operator's values for those gains are then
  `ACRO_DEFAULT_PARAMS`, not the FC's, and the first switch into ACRO writes
  them; startup itself writes nothing. Adopting the FC's would fly the balloon chaser
  on the autopilot's gains, rate-loop integrators included, and the exit
  hand-back would save them back, so every later startup would find them again.
- **Anything else** - a set matched only in part: each value that differs from
  `EXPECTED_RATE_PIDS` joins the same list and startup halts.

A param the FC does not register (`YAW_RATE_*` with `YAW_RATE_ENABLE=0`) is
skipped in both sets, using the name set the manifest gate captures. A Gain
Tuner edit the operator never copied into `EXPECTED_RATE_PIDS` does not survive
the Autopilot case: the FC overwrote it when it left ACRO.

`PTCH_RATE_FLTE` and `RLL_RATE_FLTE` stay out of the dict. `read_fc_config`
checks them against `PTCH_RATE_FLTE_EXPECTED` / `RLL_RATE_FLTE_EXPECTED` (5 Hz),
which differ from the report's 0 on purpose; listed in both places, one value
would be compared against two balloons. `test_link_contract.py` fails if either
appears in the dict.

**The pitch integrator is off.** `PTCH_RATE_IMAX = 0` clamps the rate loop's
integrator to zero on every step (`AC_PID::update_i` constrains it to
`±IMAX`), so `PTCH_RATE_I` has no effect and is 0 to say so. The rate loop's
integrator is where the elevator trim would otherwise live, and in a balloon chaser
pull-up that trim is wrong at the end: it is the trim of the attitude the pull
started from, grown further while the nose rotated, and it must be worked off
before the nose can stop. With it at zero the rate loop answers with P and FF
alone, the steady error that leaves is visible as a standing rate demand, and
the pointing law's own integrator holds the trim instead. The report still
suggests AUTOTUNE's I and IMAX for pitch; the dict records the operator's choice.
That choice is what ACRO flies. ArduPlane's own modes fly
`NATIVE_MODE_RATE_PIDS`, whose pitch integrator is on (§ "Each mode flies its own
rate loop").

Where the FC still carries ArduPlane's shipped filters (`FLTT` 3 Hz, `FLTD`
12 Hz), the first startup lists them against the report's `FLTT = 10/(2π·TCONST)`
and `FLTD = 0.5·gyro_filter_hz` until one side is changed to match.

The plant-ID report only SUGGESTS these values. Adopting them is two manual
steps: set them on the FC (MAVProxy `param set`, or the Gain Tuner's Write), and
copy the same numbers into `EXPECTED_RATE_PIDS`.
`EXPECTED_RATE_PIDS` is a dict, so `gen_constants` does not export it — it is a
startup check, not steering, and never reaches `LUA/constants.lua`.

### The manifest gate runs first

Before any per-parameter work, `verify_param_manifest()` fetches the FC's ENTIRE
parameter table and gates on it. The inventory lives in
`Companion/fc_param_manifest.py`.

  **Stage 1 — name set equality.** Every name in `FLIGHT_PARAMS` must be present
  on the FC, and the FC must report no flight parameter absent from it. Any
  difference either way prints the offending names and RAISES. Names only; no
  value is examined at this stage. `SIM_*` is excluded in both directions so one
  manifest serves SITL and real hardware.

  **Stage 2 — airframe identity.** `BUCKET_1_IDENTITY` — every output's
  `SERVO*_FUNCTION` and `SERVO*_REVERSED` — answers "which aircraft is this?".
  Function alone cannot: with the functions right and one reversal wrong, roll
  input rolls the wrong way. Mismatch prints expected against found, names the
  likely cause (a stale `eeprom.bin`), gives the fix (`-w`), and RAISES.

  **Output calibration.** `BUCKET_1B_OUTPUT_CAL` — each servo's `MIN`, `MAX` and
  `TRIM` — is checked after identity and read-only. It is calibration, not
  identity (a trim of 1500 reads the same on any airframe), so a difference is
  recorded with `_record_mismatch` and HALTS for the operator with the others at
  the end of `read_fc_config`; it does not abort.

  **Stage 3 — contract values.** `BUCKET_2_ENFORCED` and `BUCKET_2_VERIFY_ONLY`,
  whose values `read_fc_config` / `lua_setup` check read-only (§ "How a check
  runs").

  **Bucket 3 — everything else.** `BUCKET_3_UNCHECKED`, recorded in the manifest
  and never value-checked. Calibration and tuning numbers are *supposed* to drift.
  Alarming on them trains the operator to click past the alarm that matters.
  The rate-loop settings sit in this bucket as far as the gate is concerned; their
  comparison is `_check_rate_pids`, described above.

### Why a gate and not another advisory

An `eeprom.bin` carried into a cloned environment brings another airframe's
output mapping with it — elevons (`SERVO1/2_FUNCTION` 77/78) under the V-tail
airframe — because a parameter file supplies defaults only and never overrides
a stored value. Startup reads that mapping and prints it like any other
parameter: one line among dozens, and nothing stops, so an AUTOTUNE can run its
full duration against an aircraft that cannot fly. **A check without the
authority to refuse does not protect.** A per-parameter advisory cannot express
"this is the wrong aircraft" — only a gate can. When adding a parameter check,
decide which bucket it belongs to: one that identifies the airframe belongs in the
gate and aborts; one the flight code depends on is a `_check_param`, which halts
for the operator instead of scrolling past.

### Trims cannot identify an aircraft

A rudder on SERVO4 (`SERVO4_FUNCTION` = 21) and a V-tail half both trim at
1500, so a `SERVO4_TRIM == 1500` check PASSES on the wrong airframe. Identity
checks read `_FUNCTION` and `_REVERSED`, never `_TRIM`.

### Regenerating the manifest

Firmware upgrades add and remove parameters, so Stage 1 will fail after one. That
is intended: a changed parameter set is a real change worth a human look. Dump
`param show *` from a known-good FC and rebuild — see the REGENERATING note in
`fc_param_manifest.py`. Do not silence a Stage 1 failure by deleting names.

### Why startup writes nothing

Every value a startup write sets is SAVED on the FC and survives a reboot and a
simulator restart (`GCS_Param.cpp`, `handle_param_set`: `set_float` then
`save`), so the operator's own setting is what the next boot finds. A startup
that wrote would replace it silently: a comparison flight flies the old value
while the operator believes the new one is in, and a real aircraft is
reconfigured on the runway instead of the difference being noticed. So startup
reports and lets the operator decide.

The exception is the `MAVx_EXTRA1` / `MAVx_RC_CHAN` stream-group rates, written
every session in `log_telemetry_rate_config`. They do not survive between runs:
ArduPlane saves every `REQUEST_DATA_STREAM` (`persist_streamrates()` is true for
Plane; `GCS_Param.cpp`, `handle_request_data_stream`: `set_and_save_ifchanged`),
and MAVProxy requests 4 Hz each time it connects, so a checked-only value would
halt every startup.

`test_link_contract.py` pins this: outside `--arduplane-autotune`, the only
functions that send a parameter write are `log_telemetry_rate_config`, which
writes only `MAVx_EXTRA1` / `MAVx_RC_CHAN`; `write_native_param`, which only the
Gain Tuner's Write and reset reach; and `_write_param_set`, which only a mode
switch (`_set_mode`) and the Gain Tuner's set Writes reach. Nothing sends
`MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`, and `shutdown()` makes no call.

### The Gain Tuner's Write, reset, Read and Read All

*The rules — the Write and reset are the operator's hand-setting and nothing
automatic may reach them; the log records the FC-confirmed value and never reads
the FC — live in `CLAUDE.md` § "FC configuration is set by hand and only CHECKED
at startup" and § "Everything that participates in the control law is
recorded". This is the mechanism.*

**Which parameters.** `constants.ARDUPLANE_AUTOTUNE_PARAMS` is the set ArduPlane
AUTOTUNE writes: `AP_AutoTune::save_gains` saves each axis's rate-PID FF, P, I,
D, IMAX and FLTT/FLTE/FLTD plus the attitude-loop `tau` and `rmax`, and
`AP_AutoTune::start` installs `SMAX` = 150 when it is unset. For pitch, `tau` and
`rmax` are `PTCH2SRV_TCONST` and `PTCH2SRV_RMAX_UP` / `_DN`; for roll,
`RLL2SRV_TCONST` and `RLL2SRV_RMAX`; yaw's are fixed inside AUTOTUNE, not
parameters. That is 12 pitch, 11 roll and 9 yaw names; the `YAW_RATE_*` ones
exist only with `YAW_RATE_ENABLE=1`. TCONST and RMAX act in ArduPlane's angle
modes (FBWA, including the level-outs between runs): the balloon chaser law's rate
demands go straight to the rate loop and do not pass through them.

**The confirmed value.** The link keeps `native_params`, name → the value the FC
last reported. Startup fills it read-only: `_check_rate_pids` records what it
fetches, and `_read_native_params` reads the rest, then keeps a copy of the
whole map as `native_params_at_startup` — also when the read stops partway, so
the copy holds whatever the FC had answered — and a second copy as
`native_params_operator`, the operator's values. A Read, a Read All, a Write's
readback, or the readback of a mode switch or set Write updates `native_params`;
a Write, a reset or the Default Acro Parameters Write also updates
`native_params_operator`; nothing changes the startup copy.
The tuner shows
`native_params`, and the flight log's per-parameter columns (spelled as on the
FC) carry it; a name is absent — an empty box, an empty cell — until the FC has
answered.

**A Write.** `ControllerAutotuneStore.write_native` refuses, with nothing sent,
anything but a finite number from 0 to `FLOAT32_MAX`. Every one of these is a
gain, a filter frequency, a time constant or a rate limit, none meaningful below
zero, and a negative filter frequency is an internal error in ArduPilot
(`calc_lowpass_alpha_dt`). It never clamps, so what reaches the FC is exactly
what was typed. `write_native_param` then sends PARAM_SET as REAL32. ArduPlane
saves it (`handle_param_set` → `save`), so the value survives a reboot; the
handler sends no reply, and the PARAM_VALUE the save broadcasts comes later from
the storage thread's save queue. So the value is confirmed by an explicit read:
a readback matching at 32-bit precision logs ✓, a different one logs ✗ and is
recorded as what the FC holds, and no reply records nothing and says to press
Read. The console prints the value sent, the readback and a Read's value with
`float32_text` — the exact 32-bit value, never `%g`'s six figures, which can
show two different values as the same number.

**A reset.** The row's reset icon (`_on_native_reset`) takes the row's stored
Acro default, `acro_default_value` — its entry in `constants.ACRO_DEFAULT_PARAMS`,
which is `EXPECTED_RATE_PIDS` plus the `PTCH_RATE_FLTE` / `RLL_RATE_FLTE`
targets — puts it in the box as the box shows every value (`_fmt_native`, six
significant figures) and writes it through `write_native`: the same refusal,
PARAM_SET and readback as a Write. The write carries `float32_text` of the
value, not the box's text: the FC stores a 32-bit float, which can need nine
significant figures to reproduce, and six-figure text lands on a neighbouring
value. So the reset lands on the default to the bit.

It writes the default rather than the value the FC held at startup because a
reset is how a wrong value saved on the FC gets undone, and the startup value IS
that wrong value: an `RLL_RATE_P` typed into the wrong row survives every
reboot, and a reset to startup would put it straight back. It is still the
operator's command, not a startup write, and the startup check compares the FC
against the same number. The `PTCH2SRV_` / `RLL2SRV_` rows have no stored
default; their reset writes the startup copy (`native_startup_value`), and a
parameter with neither logs an error and sends nothing.

**Read All.** The All Parameters row's Read All (`_on_native_read_all`) first
puts every box back to its confirmed value, dropping typed text as a row's Read
does, then `read_all_native_params` asks for every parameter the FC has in one
`_fetch_params` round trip rather than one per row. Each answer goes into
`native_params`, where the poll shows it; a parameter that does not answer keeps
the value it had. Like Read it changes nothing ACRO flies: a value set outside
this program shows in its box and is written over at the next switch into ACRO
unless the operator Writes it.

**On screen.** Write, reset, Read and Read All run on worker threads that touch
no Tk object; `_poll_values` puts a newly confirmed value into the box and otherwise
leaves the operator's typing alone, drawing a box whose number differs from the
confirmed value in `NATIVE_PENDING_FG`. Outcomes go to the console. The rows use
the same label and value cells as every tunable row, and `_measure_columns`
sizes the window for the value box plus both buttons, the widest value column in
the window, and for the info and reset icons at their drawn size (the icon plus
its border and aura on each side).

**Afterwards.** A written value is still checked at the next startup. Where
`EXPECTED_RATE_PIDS` lists the parameter and the FC now differs, startup halts
for Enter until the operator copies the value into `constants.py` or accepts the
difference.

### Each mode flies its own rate loop

*The rules — which set each mode gets, only what differs, the mode first, only
the controller switches it on, the exit leaves the operator's values — live in
`CLAUDE.md` § "FC configuration is set by hand and only CHECKED at startup". This
is the mechanism.*

**Why two sets.** ArduPlane has one rate controller per axis, and every mode
that flies itself ends in it. In ACRO under the script, `Plane::stabilize()`
hands the script's rate demand straight to `rollController` /
`pitchController.get_rate_out()`. In LOITER, FBWA or AUTO, `Mode::run()` calls
`stabilize_roll` / `stabilize_pitch`, whose `get_servo_out` turns the attitude
error into a rate demand (`2SRV_TCONST`, `2SRV_RMAX`) and passes it to the same
`_get_rate_out`, with the same `<AXIS>_RATE_*` gains. A balloon chaser tune such as
`PTCH_RATE_IMAX = 0` would therefore fly the loiter as well. MANUAL has no rate
loop: `ModeManual::update` drives the surfaces from the sticks. The rudder
differs: outside ACRO, `stabilize_yaw` steers it with the yaw damper
(`YAW2SRV_*`), so `YAW_RATE_*` act only in ACRO and AUTOTUNE, and are written
with the rest of each set.

**The sets.** `constants.NATIVE_MODE_RATE_PIDS` holds AUTOTUNE's result for this
airframe: FF, P, I and D for each axis, the I values following AUTOTUNE's own
rule (pitch and yaw `max(0.75·P, FF)`, roll `min(P, FF)`), and IMAX at
ArduPlane's default, which AUTOTUNE keeps as long as it lies within 0.4–0.9. The
operator's values, `native_params_operator`, start as the whole startup read -
with `ACRO_DEFAULT_PARAMS` for the gains `NATIVE_MODE_RATE_PIDS` covers when
startup found the FC holding the Autopilot set (§ "The rate-loop settings"
above) - and change only when a row's Write or reset changes one or the Default
Acro Parameters Write replaces them with `ACRO_DEFAULT_PARAMS`.

**When.** `_set_mode` commands the mode and waits for HEARTBEAT to confirm it,
then asks `_mode_param_set` which set the mode flies: the operator's for ACRO
(4), none for MANUAL (0) and AUTOTUNE (8), `NATIVE_MODE_RATE_PIDS` for every
other mode. Doing the mode first means a hand-back reaches LOITER without
waiting on parameter traffic, and the balloon chaser does not start until
`enter_acro` returns, which is after the operator's values are back. A write
that fails is logged and leaves the mode change standing.

A caller that flies the new mode on RC overrides passes them as `hold`:
`_set_mode` calls it the moment the mode is confirmed and again at least every
`HOLD_REFRESH_S` while the write waits for its readbacks. The protection climb
(`climb_to`) and the level-out (`level_and_hold`) do. Without it the aircraft
flies the new mode on the real sticks for as long as the round trip takes, which
is seconds when the FC is slow to answer.

**Only what differs.** `_write_param_set` compares each value with
`native_params` at 32-bit precision, the precision PARAM_VALUE carries, and
sends only those that differ. The comparison never asks the FC: `native_params`
is the startup read plus every write since, so a switch that changes nothing
costs nothing. The PARAM_SETs go out together, and `_fetch_params` reads them
back `PARAM_READ_BATCH` at a time. ArduPlane queues 20 read requests across
every link (`GCS_MAVLINK::param_requests`) and drops the rest, so a batch leaves
room for a ground station reading at the same time. A readback that differs from
what was sent is recorded as the FC's value. A parameter that does not answer
leaves `native_params`, because its value is unknown: the log shows it blank,
and the next switch writes it again.

**Who switches it on.** The controller calls `enable_mode_param_sets()` right
after `read_fc_config`, because "only what differs" is measured against that
read. The `--arduplane-autotune` maintenance mode runs the same startup read and
switches modes through the same `_set_mode`, but never enables it: a write there
would replace the gains AUTOTUNE is tuning.

**At exit.** The controller's last hand-back is
`go_to_loiter('shutdown', keep_operator_params=True)`: LOITER, with the
operator's values written instead of `NATIVE_MODE_RATE_PIDS`. The FC saves every
write, so the next startup reads the operator's values back as the operator's,
and its check against `EXPECTED_RATE_PIDS` sees what the operator set. A program
terminated before that hand-back leaves whichever set the FC held. Outside ACRO
that is the Autopilot set, which the next startup passes as a whole set and
replaces in ACRO with the stored Acro defaults; in ACRO it is the operator's
values, checked as usual.

**The set Writes.** Below the parameter rows and Read All, `PARAM_SET_ROWS`
gives one Write for each STORED set: Default Autopilot Parameters sends
`NATIVE_MODE_RATE_PIDS`, Default Acro Parameters sends `ACRO_DEFAULT_PARAMS`.
Neither reads the tuner's boxes. `write_rate_loop_set` sends every value in the
set, whether or not `native_params` says the FC holds it, and reads each back,
which also covers what the record cannot see: a value changed outside this
program, or a mode entered outside it (MAVProxy, the transmitter, a failsafe),
which writes nothing.

The Acro Write also makes the defaults the operator's values, each the value the
FC confirms or, with no reply, the default itself. Without that the next switch
into ACRO would write the replaced values back: write `PTCH_RATE_FF` 0 from its
row, press Default Acro Parameters, and a Read shows the default, and it stays
through every later switch until a row's Write or reset changes it. The Autopilot
Write leaves the operator's values alone; the next switch into ACRO writes them
back.

**On screen.** The rate-loop boxes follow `native_params`, so they change at each
switch to show what the FC now holds. A row's Write made outside ACRO takes
effect at once, in the mode being flown, until the next switch; it is also the
operator's value from then on, so ACRO keeps it.

---

## plant-ID design

*The one rule — measurement gains are private to the run — lives in `CLAUDE.md`
(§ "plant-ID (`i`)"), and belongs to the CLOSED-loop frequency sweep `k`, not to
this open-loop measurement, which has no controller gains in the loop to protect.
Everything below is the design of the plant-ID measurement (`i` key,
`plant_id.py`) and the reasoning behind its choices.*

`i` identifies the airframe's RATE plants — one per axis, pitch then yaw then
roll — by exciting each control surface directly in MANUAL, with the FC's rate
loop, integrator, scaler and the steering law all OUT of the loop, and fitting the
short-period response, then maps each fit to the FULL ArduPlane rate PID for that
axis. It answers the AUTOTUNE mismatch report
(`Notes/autotune_mismatch_report.md`): ArduPlane AUTOTUNE finds the feedforward
from the airframe but P and D from a raise-until-ring doublet search that drove
`PTCH_RATE_P` the wrong way for the sustained balloon chaser slew, so `i` measures the
bare airframe and COMPUTES the rate PID from it instead — the same variables
AUTOTUNE tunes, with the P AUTOTUNE's search cannot find. It changes no stored
gains, applies nothing (real-hardware config is read-only by policy), and hands
each axis back to LOITER through the controller's canonical teardown.

### One method, three axes

Pitch (elevator), yaw (rudder) and roll (aileron) are the SAME measurement,
sequenced by `run_plant_id()` and gated by the Gain-Tuner Do Pitch / Do Yaw / Do
Roll switches. Each stage is self-contained — its own flight, altitude protection
and hand-back — and a top-level checkpoint (`incomplete_plant_id_run_*.json`)
records which axes are done, so a second `i` or a crash resumes at the next
unfinished axis rather than re-flying finished ones.

The axis-specific parts are only three: the RC channel driven, the surface
fraction read back, and the body-rate index. Pitch and yaw share the V-tail — the
elevator is the two halves in COMMON (`(servo2−trim)/500`), the rudder their
DIFFERENTIAL (`((servo2−t2)−(servo4−t4))/1000`) — while roll is a single separate
aileron (CH1/SERVO1, `(servo1−trim)/500`). The body rate is the FC's own gyro
(`lua_gyro_*_rads`, indices 0/1/2) and the servo outputs are the FC's own, both
read on the same FC tick and carried in STATE2, stamped with that tick's
`fc_millis` (`lua_offload.lua_tick_sample`). Nothing needs a simulator, so the
same run identifies the real aircraft. Everything downstream — the estimator,
the wings-level settle, the altitude protection, the checkpoint — is shared.

### The model: short period about its operating point

The fit is a grey-box short period on the perturbation `pr = rate − rate_ss`:

    alpha_dot = pr − Za·alpha
    pr_dot    = M_delta·delta + M_q·pr + M_alpha·alpha
    rate(t)   = rate_ss + pr(t)

`rate_ss` is an OUTPUT offset, not a moment bias, so damping acts on the
perturbation and the airframe's background rate is not charged to `M_q` or
`M_delta`. A doublet that settles to a steady non-zero rate is then fitted about
that point instead of being forced through zero — forcing zero drives R² negative
and rails the control power, which is exactly what an untrimmed entry does to the
fit. `rate_ss` past `SETTLE_MAX_Q_DPS` flags a run that was not flown from
trimmed/level flight; the plant is still identified about that point, with a
warning to re-fly cleaner. The same estimator serves all three axes (the roll and
yaw axes are first-order in practice, which the second-order model represents with
its `alpha` branch inert).

### Output-error fit, multi-started

`fit_pitch_plant` simulates `rate(t)` FORWARD from `delta(t)` and minimises the
mismatch to the measured rate — output error, not a `q_dot` regression, because
differentiating the rate is biased low under gyro noise. A single short doublet does not separate `M_q` from
`M_alpha`, so Levenberg–Marquardt runs from several seeds (a damping-dominated
one, a stability-dominated one, balanced ones) and the best-residual solution
wins. `selftest()` recovers a known plant to ~1–3% control power over 20 noise
seeds, from both a trimmed and an offset entry, pins the feedforward-map inversion
and the rate-P calibration for both pitch and roll, and pins that a high-authority
roll plant is recovered by escalation rather than clipped; it runs before every
flight so a regression fails loudly rather than fitting garbage to real data.

Roll authority varies widely between airframes and can exceed the fixed fit
ceiling that suits pitch, so `_fit_roll` ESCALATES: it starts the fit at
`ROLL_MD_MAX` and doubles the ceiling whenever the optimiser rails against it, up
to `ROLL_MD_CAP`, so a powerful roll axis is identified rather than reported as the
ceiling. A fit that still rails at the cap is genuinely non-linear — the roll rate
saturated within the hold, no finite control power fits — and is excluded from the
airframe estimate when enough clean doublets remain, flagged otherwise. The same
happens on any axis whose largest-amplitude doublets leave the linear short period.

### From plant to the rate PID — all three axes

All three fixed-wing axes run the SAME `AC_PID` rate loop (verified at V4.7.0-dev):
`AP_FW_Controller::_get_rate_out` for pitch and roll, and
`AP_YawController::get_rate_out` — byte-for-byte the same rate math — for yaw, used
when `YAW_RATE_ENABLE=1`. `AP_AutoTune` tunes the SAME variable set on each. So one
derivation (`suggest_rate_pid`) serves all three; the axis only picks the parameter
prefix and, for the integral, AUTOTUNE's per-axis I formula.

`i` reproduces the variables AUTOTUNE gets right and replaces the two it gets wrong.
The loop runs `AC_PID` in radians on `desired·scaler²`, scales P/I/D back by
`degrees(1)` and FF by `1/(scaler·eas2tas)`, and returns `out·100` clamped to
±4500 centidegrees, where ±4500 IS full throw. So for a throw-fraction gain
`G` [throw per (deg/s)]:

    kFF = G_FF · 45 · eas2tas / scaler      (FF sees scaler¹)
    kP  = G_P  · 45 / scaler²               (P and D see scaler²)
    kD  = G_D  · 45 / scaler² .

**FF — the measured feedforward** (equals AUTOTUNE's). The surface to HOLD a steady
rate is `−M_q/M_delta` [throw per deg/s], so
`<AXIS>_RATE_FF = (−M_q/M_delta)·45·eas2tas/scaler`. `M_q ~ V` and `M_delta ~ V²`,
so this one constant holds across the speed envelope — ArduPlane's own scaler
carries the speed law.

**P — closed-loop rate-bandwidth placement.** With FF making the DC gain 1, the
first-order rate plant closes under P to a single pole at
`omega_cl = M_delta·G_P − M_q`, so `G_P = (omega_cl + M_q)/M_delta`. The BANDWIDTH,
not the raw gain, is the transferable goal: because `M_delta·scaler²` is
airspeed-invariant, one `kP` holds over the speed envelope and one `omega_cl`
transfers between airframes. `omega_cl = aggressiveness · RATE_BW_REF_RADPS`, where
`RATE_BW_REF_RADPS` (34.5 rad/s) is the pole `PTCH_RATE_P=1.0` places on the
mismatch-report airframe — so aggressiveness 1.0 reproduces that flight-validated
operating point. This is exactly the gain AUTOTUNE's raise-until-ring search cannot
find: on a short doublet the best P is the one that does not ring, ~30× below what
the sustained slew needs; a bandwidth goal reads the maneuver we actually fly.

**D — critical damping of the plant + derivative-filter pole.** A PD controller on a
pure first-order plant stays first-order (D only rescales the one pole), so D is
defined against the one extra pole the loop really has: the derivative low-pass at
`tau_a = 1/(2π·FLTD)`. Placing that second-order loop at `(omega_cl, ζ=1)` gives
`G_D = (2·ζ·omega_cl·tau_a − 1 + M_q·tau_a)/M_delta`, clamped ≥ 0 so D can never
invert the loop. Near the validated point this is ~0 — which is why the hand tune
left D at ArduPlane's default — and it grows only as aggressiveness raises the
bandwidth. D is a damping trim; P is the lever.

**I, IMAX, filters — AUTOTUNE's formulas, verbatim.** `I = max(0.75·P, FF/1.0)` for
the naturally-damped pitch and yaw axes, `I = min(P, FF/1.0)` for roll; `IMAX`
clamped into `[0.4, 0.9]`; `FLTT = 10/(2π·TCONST)`, `FLTE = 0`,
`FLTD = 0.5·gyro_filter_hz`; `SMAX` the 150 AUTOTUNE installs. The attitude-loop
`2SRV_TCONST`/`RMAX` are design defaults (TCONST 0.5, RMAX 0 = no limit), reported
but not plant-derived — yaw has no such params at all.

**The pitch/yaw integral is not sized from FF alone.** Besides closing small
standing rate errors, it carries the elevator trim, and that trim moves by a large
share of full throw between level flight and a steep climb — the balloon chaser pull-up
starts at one and ends at the other. The doublets measure the plant about level
trim and never see that shift, so an integral set from their FF alone re-trims
several times too slowly. The integral does not set the overshoot; its speed sets
how quickly the nose comes back after one. With P bandwidth-placed well above FF,
AUTOTUNE's `0.75·P` branch is the one in effect. On roll, a strongly self-damped
axis can have P placed below FF, and `min(P, FF)` then makes the roll I equal P,
so it follows the aggressiveness knob.

The full per-axis set is written as paste-ready lines in the run's single report
(see "Tuning-run outputs" below) for the operator to REVIEW; nothing is applied,
and real-hardware config is read-only by policy. Adopting any of them means
setting it on the FC by hand and copying it into `EXPECTED_RATE_PIDS` (see § "FC
configuration and the manifest gate"). `YAW_RATE_*` apply only with
`YAW_RATE_ENABLE=1` (ArduPlane's direct-rate controller); under the default
sideslip damper they are inert. One operator knob, `plantid_rate_agg` (a Gain-Tuner
row, default 1.0), scales the goal bandwidth; the report also prints a
sensitivity table around the chosen value. `RATE_BW_REF_RADPS` encodes the one
validated operating point we have — the estimator selftest pins it, so a drift
fails loudly rather than silently re-deciding every future suggestion.

### The overshoot floor

`a_max = M_delta · full_throw` is the maximum angular deceleration, so a rate R
cannot be arrested in less than `R²/(2·a_max)` — a physical lower bound no
rate-loop tune can beat, and the arrival overshoot no `P` can drive below. Every
axis's report carries it at representative arrival rates, so an aggressiveness
choice can be sanity-checked against what the airframe can physically do rather
than chased past it.

### Altitude protection and the wings-level settle

The nose-down doublets bleed height, so each axis is flown with the same altitude
protection as the sweep and `k`: below `autotune_protect_floor_m` (a live
Gain-Tuner knob), climb to `autotune_protect_climb_to_m` before the next doublet —
never pre-emptively. Between doublets `_relevel` uses FBWA to self-level and
VERIFIES straight-and-level from telemetry (roll angle, yaw rate, pitch angle and
pitch rate all inside tolerance for `STRAIGHT_HOLD_S`, imported from the sweep so
the two cannot drift), because a residual bank or dive left by the previous
doublet is exactly what the next open-loop excitation would ride, surfacing as a
non-zero `rate_ss`.

### Pitch entry: trim the bunt before the doublet

The wings-level settle levels the aircraft in FBWA, but the doublet flies MANUAL,
and the two do not agree on the elevator. FBWA holds level with a NON-ZERO
elevator (the deflection that trims for cruise); MANUAL's mechanical neutral is a
different angle. So a pitch doublet centred on neutral rides a steady nose-drop of
tens of deg/s — the bunt — for its whole window.

That bunt is fatal to the feedforward, though not to the control power. It is not
a constant offset the constant-`q_ss` model can absorb: it eases as the aircraft
settles toward its neutral-trim attitude, so the fit cannot separate it from the
damping. `M_q` rails at 0 and `FF = -M_q/M_delta` collapses to 0, while `M_delta`
survives because it is read from the perturbation slope. A bunting run therefore
yields a usable control power and overshoot floor but a useless feedforward.

`_trim_pitch_manual` removes the bunt before each pitch doublet by nulling the
MEASURED body pitch rate in MANUAL — a short proportional search that moves the
elevator until the rate holds near zero, i.e. onto the level-holding elevator. It
uses the measured rate rather than reproducing FBWA's elevator through the
RC→servo→V-tail-mix calibration, whose reversals make an open-loop servo match
fragile to get right and impossible to verify without flying. The feedback sign is
fixed by the doublet's own convention (`PWM_MID + amp` is nose-up, so higher
elevator PWM raises the rate): to null a rate `q`, move the PWM by `-Kp*q`. The
correction is clamped to `±TRIM_MAX_OFFSET_PWM` so a wrong response cannot command
a runaway, and a search that never nulls returns the steadiest PWM it saw, so the
worst case is the old bunting entry, not a departure.

The trim search flies MANUAL open-loop for up to a few seconds with the ailerons
uncommanded, so the wings can drift off the level the settle verified. The driver
therefore RE-LEVELS in FBWA after the trim and before the doublet, so the maneuver
STARTS wings-level, then returns to MANUAL and commands the trimmed elevator (still
valid — the brief level-out holds airspeed). The re-level is not fooled by the
trim's latched elevator override: `level_and_hold` sends its own centred
roll/pitch sticks every loop, so FBWA levels regardless of what the search left on
the wire. Roll trims the same way and for the reason below; yaw does not — it
spends only a fraction of a second in MANUAL before its doublet, too little to
drift, and rudder-neutral is already coordinated (`r_ss ≈ 0`).

The doublet then records its elevator as the DEVIATION from that trim, not from
mechanical neutral. This is not cosmetic: the model's only equilibrium is
`q = q_ss` at `delta = 0`, so centring the doublet on a non-zero trim deflection
while recording ABSOLUTE deflection asks the model to hold a steady rate at
`delta ≠ 0`, which it structurally cannot — and the misfit corrupts the very
damping the trim exists to expose. Referencing to the trim puts the operating
point back at `delta = 0`.

Roll needs the same trim, for a different reason. Aileron-neutral does NOT hold a
steady zero roll rate: prop-torque reaction (and any aileron mis-trim) leaves a
rolling moment that FBWA quietly holds out and MANUAL-at-neutral exposes. Roll
control power is several times pitch's, so that small moment becomes a steady roll
well past the `SETTLE_MAX_Q_DPS` flag; recorded as absolute aileron it forces the
model to hold a steady rate at `delta ≠ 0`, and the misfit scatters `L_p`
doublet-to-doublet (`L_delta` survives, read from the perturbation slope).
`_trim_roll_manual` nulls the measured roll rate as pitch does, with
`ROLL_TRIM_KP_PWM_PER_DPS` set below the deadbeat value because roll answers so
much harder per PWM. Only yaw still centres on mechanical neutral. `selftest`
recovers `M_q` from a clean (steady) doublet and fails if that breaks; a drifting
entry rails it at 0, which is the trap this trim removes.

### The directness gate

Before spending air time, each axis proves its command reaches the surface and its
measurement reflects the airframe, or refuses to run: a small stick step must move
the right surface (and, on pitch, move both V-tail halves TOGETHER — a large
asymmetry means rudder mix is contaminating the pitch measurement), the body rate
must respond, and airspeed must be clear of the underspeed lock. A distorted plant
is worse than no measurement, so the gate FAILS the stage rather than fit through
the distortion.

---

## Tuning-run outputs: one report, one checkpoint, one signals CSV

*The rule lives in `CLAUDE.md` § "Measurement and tuning runs follow one
pattern". This is what each file is, and why.*

**A tuning run emits exactly three KINDS of file, whatever sub-parts it ran, and
the content of each reflects which sub-parts were enabled — not one file per
sub-part.** Completed runs are KEPT, so a rerun writes a new set beside the old
rather than replacing it (see "Completed-run retention" below). The three kinds
are:

- **one report** (`*_report_*.txt`) — the human-readable result: a header naming
  what ran, a paste-ready block of any suggested FC parameters (REVIEW before
  loading; nothing is applied), then the full run transcript so a result missing
  from the block can be explained from the file alone;
- **one checkpoint** (`incomplete_*_run_*.json`) — resume state, present only
  while a run is unfinished and deleted on completion;
- **one signals CSV** (`*_signals_*.csv`; the trim map calls its own the log,
  `trim_map_log_*.csv`) — the raw measured samples, tagged with
  the sub-part they came from (the axis, the cell, …) so one file holds them all.

Why: a run with N sub-parts otherwise scatters 2N or
more files (a CSV and a params file per axis, plus a report and a checkpoint), the
operator has to hunt for the newest of each, and the export buttons drift out of
step with the filenames — the button "Pitch Feedforward (i)" copying a file called
`suggested_pitch_rate_params_*` is exactly the confusion this removes. Folding the
suggested parameters into the report (rather than separate `.param` files) is part
of it: the report is the one thing to read, and the button name and file name then
agree.

**Implemented for Plant-ID (`i`) and the trim map (`e`).** `_RunOutputs` in
`plant_id.py` owns the report (`plant_id_report_*.txt`) and the signals CSV
(`plant_id_signals_*.csv`), shared across the pitch/yaw/roll stages; the
`_run_ckpt_*` helpers own the checkpoint (`incomplete_plant_id_run_*.json`).
`_Outputs` and the `ckpt_*` helpers in `trim_map.py` do the same for
`trim_map_report_*.txt`, the log `trim_map_log_*.csv` and
`incomplete_trim_map_run_*.json`. Each file has a Gain-Tuner export button named
after it ("Plant-ID report", "Trim-map log", …). The controller-autotune
sweep (`t`), the single-cell tune (`y`) and the frequency-ID sweep (`k`) write
their own multi-file sets (`TODO.md` § 8).

### Completed-run retention

A completed run's outputs are KEPT. Each is stamped with the run time
(`file_export.stamped`), so a rerun writes a new file beside the previous one
instead of overwriting it, and several runs accumulate on disk for the operator
to compare or export. The only file a new run removes is its own resume
CHECKPOINT. The rule and its scope are in `CLAUDE.md` § "Completed-run outputs
are kept; only the checkpoint is cleaned"; this is the mechanism.

It is enforced at two layers.

**The writers.** `_RunOutputs.finish` (plant-ID), `_dump_rows`/`_report` and the
results-JSON write (freq-ID), `constants_suggest` and the sweep's report write
each write their timestamped file and STOP — none calls `file_export.claim()` on
a completed output. `claim()` survives only where a single live file is correct:
the resume checkpoints, the per-launch console log, and the `q` flight log
(truncated on each new recording by design). Plant-ID clears its previous
checkpoint through `claim(_RUN_CKPT_GLOB…)` in `_run_ckpt_save`; freq-ID and the
sweep keep one checkpoint by overwriting a fixed run-stamped name and deleting it
on a clean finish.

**The export.** With runs accumulating, `file_export.matches()` must never hand
Copy a mix of two runs. A single-file entry uses `pick='latest'` — the newest by
the stamp in the name. The one entry that is a SET written across a run —
freq-ID's per-cell CSVs, `freq_id_cell_<cell>_<ts>.csv` — uses `pick='run'`,
which copies every file sharing the newest run stamp and nothing older. Delete
(`every=True`) still clears every run on disk, because Delete means "empty this
row", not "drop one run".

Why keep them: the reason to run `i` or `k` twice is almost always to COMPARE — a
re-fly after a config change, a level pass against a non-level one. A `claim()`
at a completed-output write deletes the predecessor the moment the second run
begins: exactly the file the comparison needs, before the operator can copy it
out. Retention costs disk the operator reclaims with one Delete; deletion costs
a measurement that can take twenty minutes to re-fly. `test_export_gazebo.py` pins
the contract: no writer claims a completed output away, the checkpoint claim
survives, and `pick='run'` copies the newest run's whole set without mixing.

---

## Measurement runs: the shared pattern

*Plant-ID (`i`) and the trim map (`e`) follow this pattern whole; freq-ID (`k`)
and the sweep (`t`, `y`) follow most of it. The rule that a new run follows all
of it, and the rules it rests on, are in `CLAUDE.md` (§§ "Measurement and tuning
runs follow one pattern", "Tuner edits are live in flight", "Completed-run
outputs are kept; only the checkpoint is cleaned", "Sweep / plant-ID operator
display").*

**One key starts it; the same key stops it, another switches away.** The
controller's `toggle_*` method starts the run on its own daemon thread
(`_launch_run`, which sets `_run_key`, `_run_abort` and `_run_thread`; the key
is cleared when the thread has ended). A second press of the same key sets the
run's abort `Event`, which every phase checks (the protection climb, the
level-out, each step), and the run hands back to LOITER. It is checked before
the "transition in progress" guard, so it reaches the abort even while the run
is starting (`_run_key_pressed`, the shared start of every run key).

Any other mode key — `p`, `x`, `s`, an arrow, or another run's key — goes
straight to its own mode with no LOITER in between (`_switch_from_run`). It
records the switch (`_switch`, with a `KEY_<k>_SWITCH_FROM_<run>` event in the
flight log) and stops the run with the run's own abort, checkpoint kept. The
switch is then settled in two places, once each, and its state changes only
under `_switch_lock`:

- the run's teardown (`autotune_finish`) claims it and hands the aircraft over
  in ACRO instead of LOITER (`_hand_over`), in `go_to_loiter`'s order: tracking
  off, the sticks centred at the operator's throttle (`link.hold_level_sticks`,
  so a level-out's or a doublet's override cannot fly on as an ACRO rate
  command), ACRO commanded and confirmed with the sticks held, then the sticks
  released. ACRO refused: LOITER instead, and the switch is cancelled. A
  teardown that calls `autotune_finish` twice (the sweep does) sees the switch
  already handed over and leaves the aircraft alone;
- the control loop (`_service_switch`, once per tick, on the thread that
  handles the keys) presses the key again once the run's thread has ended,
  which finds no run and starts its mode.

The wait for the thread is the point: a mode started while the run is still
tearing down is a second driver. Starting it from the control loop, not a
helper thread, means it cannot race a key or reset state mid-tick. A key pressed
while a switch waits is refused. A run still up after `_SWITCH_WAIT_S` has its
switch dropped (its teardown then hands back to LOITER) or, if it already handed
over, the aircraft is handed back. A mode that fails to start after a switch is
handed back, as is a failed ACRO entry for `s`, `x` or an arrow
(`_begin_tracking`). Shutdown cancels a pending switch first
(`_cancel_switch`), so the exit hand-back is the last mode command. `l` stays
refused during a run, as it is in `p` and `x`: the run owns the sensor.

**It flies the `'autotune'` mode regime under its own HUD label.**
`_mode = 'autotune'` is what drives control and must not change; `_mode_label`
changes only what the HUD prints ("Mode: Plant ID", "Mode: Trim Map"). The
magenta column shows `autotune_status` — which step is running and its phase —
beside the run timer, the `Straight:` timer during a level-out and the `Test:`
timer during a step. The status line is drawn only while the run timer runs
(`autotune_start_t` set): the sweep and the trim map set it, plant-ID does not,
so plant-ID's status never reaches the HUD.

**Every step is self-contained: protection, level-out, step.**
- *Altitude protection.* Below the floor the run climbs to the climb-to
  altitude with `link.climb_to` in FBWA — only when below, never
  pre-emptively. Floor and climb-to are live Gain-Tuner rows, read at each
  check, in the run's own tuner group: `t`, `i` and `k` share one pair
  (`autotune_protect_floor_m` / `_climb_to_m`, in the Plant ID group); the trim
  map has its own (`trim_map_protect_floor_m` / `_climb_to_m`, in the Trim Map
  group). The HUD status gains `| Alt protect: climbing to Nm` for the
  climb (split on ` | ` first, so repeats do not stack). A step that breaches
  the floor part-way is climbed back and flown again (the trim map once, the
  sweep up to `PROTECT_MAX_RETRIES`); one that breaches every time is recorded
  as such rather than retried forever.
- *Level-out.* `link.level_and_hold` in FBWA, with the sweep's `STRAIGHT_*`
  tolerances imported rather than copied: roll angle, yaw rate, pitch angle and
  pitch rate all held for `STRAIGHT_HOLD_S`, at the operator's throttle. A
  timeout logs and proceeds. Then ACRO (`ptest_enter_acro`) for a closed-loop
  step, or MANUAL for plant-ID's open-loop doublets.
- *The step*, then tracking paused (`autotune_pause_tracking`) before the next.

**Samples are the FC's own ticks.** Both frames of one tick - a STATE2 paired
only with the STATE that arrived immediately before it, with the same seq, since
several FC ticks can echo one `GUIDE` seq - stamped with `fc_millis` — `lua_tick_sample` for plant-ID, `lua_tick_frames` for
the trim map — never the wall clock and never MAVLink telemetry, so a value is
always paired with the others of its tick and the run works on the real
aircraft.

**Gains.** An open-loop run (plant-ID) has none in the loop. A closed-loop run
installs a private set for its duration (`begin_measurement_gains`,
`update_measurement_gains` between steps, `end_measurement_gains` in its
`finally`), which the FC flies while the tuner keeps editing the operator's set
(§ "Tuner edits are live in flight").

**A checkpoint after every finished step.** `incomplete_<name>_run_<stamp>.json`
holds a version, the code fingerprint (`PACKAGE_SHA256`), the run stamp and what
is done (plant-ID: the finished axes; the trim map: every flown cell's result
and the grid it belongs to). Each save removes any older checkpoint with
`file_export.claim()`. The next start loads it: a different version or
fingerprint — the code changed since — or, for the trim map, a different grid
starts afresh (the trim map says why on the console; plant-ID starts afresh
silently); otherwise the run resumes at the next unfinished step
under the ORIGINAL run stamp (`adopt_run_stamp`), so every file of one run keeps
one stamp across relaunches. A crash, Ctrl+C or a second key press keeps the
checkpoint; only completion deletes it. There is no separate save on the stop
path.

**Three files, written whatever happened.** One report, one signals CSV, one
checkpoint (§ "Tuning-run outputs" above). The report is written in the run's
`finally` — on completion, on a stop and on a failure — with the session's
console output captured by a log handler (`plant_id._LogCapture`) so a result
can be explained from the file alone. Suggested values go in the report for the
operator to REVIEW; nothing is applied. On a resume the trim map appends to the
first session's log and rebuilds its report from every cell in the
checkpoint. Plant-ID reopens both for writing under the same stamp, so after a
resume they hold only the axes flown in the last session.

**Export rows.** Each file kind has a Gain-Tuner export row
(`file_export.EXPORTS`): Copy takes the newest by the stamp in the name, Delete
clears every run on disk. Deleting the checkpoint is how the operator makes the
next start begin afresh instead of resuming.

**The hand-back.** The `finally` releases any measurement gains, then
`autotune_finish()` hands the aircraft to LOITER through the canonical
`go_to_loiter` (tracking off first, LOITER confirmed, overrides released) and
clears the HUD status and timers. When another mode key stopped the run, it
hands over in ACRO instead (`_hand_over`, above) and the next mode starts from
there. Plant-ID hands back as each stage ends, and once more at the end when no
stage flew, so a run with nothing to fly still clears the mode and the start-up
transition.

---

## Trim map design (`e`)

*`trim_map.py`; the run pattern above; the tuner rows are the Trim Map group.*

**What it is for.** The pitch trim (`balloon_chaser_fc.lua`, "PITCH TRIM") supplies the
elevator the rate loop cannot find by itself — ACRO flies it with no integrator.
The elevator needed to hold an attitude depends on the attitude, the airspeed and
the throttle, so the trim map measures it across climb angle and airspeed, and the
trim formula is fitted to what it measures rather than guessed.

**The formula it fits,** which `balloon_chaser_fc.lua` flies:

    trim_elev = offset + (VREF / V)^2 * (lift     * L
                                        + attitude * sin(pitch)
                                        + thrust   * throttle / EAS2TAS^2)

with V the EAS (floored at the speed scaler's airspeed minimum), VREF
`constants.PITCH_TRIM_VREF_MPS`, throttle ArduPlane's throttle output as a
fraction, and L the lift term: cos(pitch) under the cascade, the load Lift Vector
Pointing asks of the wing under it (§ "Lift Vector Pointing: the second steering
law" → "The pitch trim under this law"), logged every tick as
`lua_pitch_trim_lift_term`. Each term is a moment the tail must balance, divided
by the tail's authority, which grows with V²: `lift` is the wing's lift need (the
weight times L, more nose-up the slower it flies), `thrust` the motor's pitching
moment, `attitude` a nose-attitude moment.

`1 / EAS2TAS^2` is the air density as a fraction of sea level's. The wing terms
need no such factor, because V is EAS and already follows the density; the
motor's push does not. At one throttle the propeller turns at about one speed at
any height, so its push scales with the density, and a thrust gain fitted at one
height would be wrong at another. With the factor, `thrust` is the motor's
moment in sea-level air, and one set of gains holds at every height. The FC
reads `ahrs:get_EAS2TAS()` for it and logs it every tick (`lua_eas2tas`); the
trim map records it per tick and per cell and fits the same term.

**Where the trim meets the pitch limit.** The trim goes in as an extra
pitch-rate request (`lua_pitch_trim_dps`), added after the law's own clip at
its envelope, and ArduPlane then holds the whole request at ±ACRO_PITCH_RATE
(`set_target_throttle_rate_rpy`). With the law already at the envelope, a
nose-up trim is cut there: at full pull the rate loop gets no trim, and the
pull falls short of the law's by the trim's share. The script makes the same
cut itself before sending, so `lua_omega_y` is the request ArduPlane flew,
not the sum asked for. The way to give the trim room at full pull is a higher
ACRO_PITCH_RATE, which the law's envelope follows.

`constants.py` carries the four gains the trim map fitted on the SITL airframe,
with how well they fit beside them. Two need care. The thrust gain is not
separable from the other terms in a trim map: each cell's throttle is whatever
holds its airspeed, so it rises and falls with the climb angle and the speed,
and a fit without it does as well - the motor barely pitches this airframe. The
attitude term is large and consistent in SITL, but a rigid aircraft has no
gravity moment about its centre of gravity, so it may belong to the simulated
model: map the real aircraft before trusting it there.

**What it measures.** For each cell of a climb-angle × airspeed grid it flies the
aircraft steady and records the elevator ArduPlane is flying. With the nose not
rotating and no integrator in the rate loop, the elevator is the loop's gain
times the whole pitch-rate request, trim included; when `trim_elev` equals it, the
law's own share is zero and there is no standing miss. So the steady elevator IS
the value `trim_elev` must produce at that pitch and airspeed, whatever the gains
or the trim setting were — they decide how fast the aircraft settles, not where.
It is `out_elevator / 100` from STATE2 (ArduPlane's scaled output before the
V-tail mix, 45 = full travel, the formula's units), the pitch is the Euler
pitch of the FC's quaternion on the same tick, the angle the formula takes the
sine of, and the lift term is the one the FC's trim read that tick
(`pitch_trim_lift_term`), which the fit regresses on.

**How a cell is flown.** Wings level, closed loop, in ACRO. The injector anchors
a fixed direction dead ahead at the cell's climb angle (`force_direction`, so a
dive's direction is never capped at the ground), and the balloon chaser law flies the
flight path onto it. The FC's airspeed hold holds the cell's airspeed through the
throttle alone, switched on with the run's private gains
(`airspeed_cmd_enable`, `airspeed_cmd_mps`); everything else, the operator's
trim included, is flown as the operator set it. Cells are flown one airspeed at a
time, the climb angles alternating steepest climb and steepest dive
(`fly_order`) so the height one gains is spent by the next. The lowest default
airspeed is `TRIM_MAP_SPEED_MIN_MPS`: below it the SITL airframe's elevator
reaches its stop before the wing holds level flight, so a slower cell only
mushes and sinks. The protection's climb-to is kept low
(`TRIM_MAP_PROTECT_CLIMB_TO_M`): the simulated air thins with height as real air
does ("Simulator air: AltitudeLiftDrag"), so the motor's reach up high is not
the reach near the field the aircraft competes over.

**When a cell is held.** After `MIN_FLY_S` of pull onto the bearing, the cell
is held once a trailing `STEADY_S` window of FC ticks has: mean |pitch rate|
under `Q_TOL_DPS` and pitch drift under `PITCH_DRIFT_DEG` (the nose is not
rotating), |roll| under `ROLL_TOL_DEG` (a steady bank at the limit asks 1.5% more
lift, well inside the fit's residual), the mean airspeed within `SPEED_TOL_MPS`
of the goal and drifting less than `SPEED_DRIFT_MPS`, and the elevator off its
stop. The cell's numbers are the means over that window.

**When a cell stops without holding, and the trap in it.** Every cell starts at
the level-out's airspeed, not its own, so being far from the goal is normal
while the airspeed hold closes the gap — an underspeed stop keyed on distance
alone ends every fast cell during its acceleration. Each stop therefore also
needs the airspeed going the WRONG way over `TREND_S`: more than
`UNDERSPEED_MPS` below the goal and falling (out of thrust or lift for the
climb — the stall guard), more than `OVERSPEED_MPS` above and rising (a dive
running away), or the throttle pinned for `SAT_GIVEUP_S` with the gap not
closing by `CLOSING_MPS`. The stop names the throttle at that moment, not a
cause. The altitude floor on every attempt, or
`CELL_TIMEOUT_S`, also end it. A cell that stops is recorded NOT HELD with its
last window's means. When that window passes the steady test without the
airspeed goal — the aircraft steady at the airspeed it could reach — it is
still a trim measurement: marked `~` in the report and used in the fit. Only a
window that is not steady (`*`) is left out.

**Out of reach, skipped.** A climb at full throttle (`THR_FULL_PCT`) over its
window, or one that ends with the airspeed loop pinned at the top, shows the
fastest airspeed the motor holds there, and no steeper climb can be held faster;
a dive at idle (`THR_IDLE_PCT`), or with the loop pinned at the bottom, shows
the slowest, and no steeper dive can be held slower. `out_of_reach` skips such a
cell before it is flown and records why, so a run does not spend a level-out and
a timeout on a cell a flown one already answered.

`limit_of` decides this from the throttle, never from the stop alone. A slow
climb whose elevator reaches its stop has run out of lift, not thrust: the
aircraft mushes and sinks and the airspeed falls with the throttle anywhere, so
the window says nothing about the motor and marks no limit. Taking such a stall
for a thrust limit skips every faster and steeper climb, cells well within the
motor's reach, and leaves the fit without the climbs that separate the thrust
term.

**The report.** Climb × airspeed tables of the elevator needed, with the pitch,
airspeed, angle of attack, flight path and throttle actually flown; the formula
above least-squares fitted to every steady cell (the four suggested Pitch Trim
rows, the rms and the worst residual) and its residual per cell — a pattern
across a row or a column is a dependence the formula lacks; one paste-ready line
per cell, with its steady flag, for another fit or a lookup table; and the
transcript. The log (`trim_map_log_*.csv`) carries every sampled tick with its
cell and marks the averaged window (`in_window`), so any other fit can be made
offline. Neither carries the lift term, so a refit from them alone holds for a
map flown under the cascade, whose term is cos(pitch); under Lift Vector Pointing
the term is in the flight log (`lua_pitch_trim_lift_term`).

SIM only until validated: a steep climb at a low airspeed can run out of thrust,
and a slow cell out of elevator; the underspeed stop and the altitude floor are
its guards.

---

## Telemetry rates: attitude, position, the oracle pull

*The rules — never feed raw rate telemetry unfiltered; sample position-derived
quantities per FIX not per tick; the oracle balloon must be pulled not pushed —
live in `CLAUDE.md` §§ "Raw rate telemetry…", "POSITION is 5 Hz…", "The oracle
balloon is PULLED…". This is the mechanism and the measured evidence.*

### Raw rate telemetry is hold-and-jump noisy

MAVLink rate telemetry arrives slower than it is requested and repeats unchanged
between arrivals, so it is a staircase rather than a signal. The steering loop
does not consume it — the FC reads its own quaternion, gyro, velocity and airspeed
on-chip, and `hz_quat`/`hz_gyro`/`hz_vel` in `STATE` report 49 against a 50 Hz
loop. Everything the companion still reads over the wire is subject to it: the
HUD, the CSV, and any analysis run against them.

The gimbal ENCODER angles cross the wire in `GUIDE`, beside the detection and the
`p` test's oracle bearing, because the gimbal is a Gazebo model the companion
drives. Attitude is
fresh; the encoder is not. See "Lua steering offload".

Differentiated rate telemetry carries far more high-frequency energy than the
angle it came from, and the excess is the staircase, not the airframe. An
apparent oscillation of a few Hz in a raw rate channel is this artifact; a real
limit cycle appears identically in the ANGLE, the COMMAND and the servo output.
The measured numbers behind this are recorded beside the stream setup in
`mavlink_link`.

`quat_fresh` records PER-TICK freshness (1 = a NEW quaternion arrived since the
previous control tick), so `mean(quat_fresh)` measures the effective attitude rate
per log. A liveness window rather than a per-tick flag reads a constant 1 and
measures nothing. Startup measures the actual `ATTITUDE_QUATERNION` arrival rate
for 2 s and warns loudly below 45 Hz. Requesting a rate does not set it, so the
assumption is checked rather than trusted.

### POSITION is 5 Hz — sample per fix, not per tick

Position is a separate and much worse case than the attitude stream.

The Lua offload does NOT fix this: the FC reads its own attitude and velocity, but
`aircraft_world_pos_ned()` remains a companion-side quantity built from
`GLOBAL_POSITION_INT`, and it is what the ORACLE bearing is computed from before
being sent in `GUIDE`. So ptest still inherits the staircase. `hz_vel` measures 49
(EKF velocity is propagated at IMU rate), but that is VELOCITY, not POSITION — do
not read the 49 Hz figure as covering this.

`aircraft_world_pos_ned()` is built entirely from `GLOBAL_POSITION_INT`, which
`request_high_rate_streams` asks for at **5 Hz**. The control loop reads it at
50 Hz. So the aircraft position is not merely stale — it is a STAIRCASE:
bit-identical for ~10 consecutive ticks, then a step. Nothing above it smooths
that, because there is nothing to smooth; between fixes there is genuinely no new
information.

A fit that samples this staircase per tick satisfies every bracket guard by
construction and latches a false result on the first step of every run; the
trap is written out beside `_update_balloon_range`, which owns it.

How the fit avoids it:

1. **Sample per FIX, not per tick.** Anything that differentiates, fits, or
   integrates a position-derived quantity must gate on the data actually being
   new. Re-reading an unchanged value is not a second observation.
2. **Use `pos_time_boot_s`, not the wall clock.** `mavlink_link` records
   `GLOBAL_POSITION_INT.time_boot_ms` as `pos_time_boot_s` (seconds, `None`
   before the first fix; declared on the `AutopilotLink` protocol). That is the
   FC's own stamp for when the fix was TRUE, so it carries neither link jitter
   nor the 0–20 ms the loop took to notice. It doubles as the freshness key: a
   changed stamp means a new fix. Timing a fit by arrival instead smears the
   fixes' true uniform spacing by up to 10% straight into the result.
3. **Read the stamp BEFORE the position it labels.** The telemetry thread writes
   the stamp last, so this ordering means a fix landing mid-read can only pair a
   NEW position with the PREVIOUS stamp — read as "no new fix" and skipped,
   picked up correctly one tick later. The opposite order silently mistimes a
   sample the fit then trusts.
4. **Fit in real time, not in sample index.** Fixes do not land on a perfect
   200 ms grid, so the equally-spaced closed form does not apply. The fit uses
   the Newton form through three non-uniform `(t, range²)` samples; it is
   algebraically identical to the equally-spaced formula when the spacing does
   happen to be uniform, so nothing is traded away.
5. **Display values are exempt.** `_balloon_range_m` (the HUD "Balloon Distance"
   readout and its CSV column) is still refreshed every tick — a moving balloon
   changes it between fixes, and showing the operator a value that ticks is
   correct. Only the FIT is gated.

Residual accuracy after the fix: ~1.5 cm for a stationary balloon, set by
MAVLink's 1 ms `time_boot_ms` resolution. A MOVING balloon carries a further
`balloon_speed × ~10 ms` — see the oracle-pull note below for what that is and why
it is not staleness.

### The oracle balloon is PULLED at the tick, not pushed from the injector thread

`SyntheticBalloonInjector` must NOT push a position snapshot to the sensor from
its own 50 Hz thread. The control loop also runs at 50 Hz, and two independent
free-running timers beat: a pushed position reaches the controller **unchanged**
on some ticks and double-stepped on others, off by tens of centimetres at a
moving balloon's speed.

The injector holds an exact closed form (`pos0 + vel*(t - t0)`), so it hands the
sensor a callable (`set_oracle_position_fn`) instead, which the sensor evaluates
**once per control tick** at the top of `update_per_tick` and caches.

Three things this buys, in order of importance:

1. **The law sees the right balloon.** `oracle_world_dir()` feeds the
   estimator snap and the control law, whichever is selected. A pushed position jitters at the beat
   frequency — a small artifact, but one injected into the
   control path during exactly the moving-balloon tests meant to measure it.
2. **Intra-tick consistency.** Every consumer in a tick (the law, range fit, HUD
   reprojection) sees the SAME position. A push could land between
   `update_per_tick` and `_update_balloon_range`, so they would disagree.
3. **One less free-running timer to reason about.**

Two constraints come with it, both handled:

- **The leg must be swapped atomically.** `position_now()` is called from
  the CONTROL thread while the injector thread starts new legs. Four separate
  attribute stores are not atomic together, so a reader could pair the NEW
  `pos0` with the OLD `t0` and compute a position tens of metres out for one
  tick. The leg is therefore ONE tuple `(pos0, vel_world, vel_ground, t0)`,
  swapped in a single store; `_position_at` snapshots it into locals before any
  arithmetic. `_pos0` / `_vel_world` / `_vel_ground` / `_t0` are read-only
  properties onto it, so nothing can reintroduce a partial write by accident.
- **A sensor without the hook still works.** `_publish_position` falls back to
  pushing, and `_loop` keeps refreshing it. Any `BalloonSensor` implementation and
  every test fake is unaffected.

**The time skew it leaves.** Pulling makes the balloon exact at the control
tick, which is not the instant the aircraft's position was true. The controller
samples on a position fix valid at `t_fix`, but evaluates the balloon at the
control tick `t_tick`, and `t_tick - t_fix` is the loop latency: uniform in
[0, 20 ms), mean 10 ms. So a moving balloon carries a residual of
`balloon_speed × 10 ms` — about 11 cm at 11 m/s, 25 cm at 25 m/s. Closing it needs
the balloon evaluated at the FIX's instant, i.e. a clock shared between the FC's
`time_boot_ms` and the injector's `monotonic()`: real work for centimetres on a
synthetic-balloon-only metric, and the stationary competition balloon has no such
term (speed zero). Timing the sample by the tick instead would mismatch the
AIRCRAFT by the same latency, at aircraft speed, which is larger.

---

## PID_TUNING: FC rate-loop internals

*The rule — `fc_*_des` vs `fc_*_ach` is the last stream to drop, not the first —
lives in `CLAUDE.md` § "PID_TUNING". This is what it is and a worked example.*

Startup checks `GCS_PID_MASK = 7` (roll bit0 + pitch bit1 + yaw bit2) and requests
`PID_TUNING` at 25 Hz/axis, so the FC streams its rate loop's OWN desired rate, achieved
rate, and P/I/D/FF terms for all three axes with no manual setup. The link captures these
(`_pid_tuning` per axis, `pid_tuning_field()` accessor) and the controller snapshots
roll, pitch and yaw into 18 CSV columns per row: `fc_{roll,pitch,yaw}_{des,ach,ff,p,i,d}`.

This is the authoritative inner-loop view. Unlike the raw `roll_rate`/`pitch_rate`
passthrough, these are the FC's internal signals: `fc_pitch_des` vs `fc_pitch_ach`
is the rate loop tracking itself, and `fc_pitch_i` is its integrator - so a
SETTLED hold (where the outer command is near zero and the external correlation is
uninformative) can still be judged directly from the FC's own desired/achieved/I-term.
These columns are blank until the FC streams that axis (before the mask takes
effect, for example); `fc_mode` and the startup log confirm the stream came up.

`fc_*_des` vs `fc_*_ach` is the rate loop's own record of its tracking, with the
P/I/D/FF terms that produced it — it is what distinguishes "steering is wrong"
from "steering is right and the airframe cannot follow". STATE and STATE2 add
the same-tick view at 50 Hz: `lua_omega_x/y` (commanded, STATE) against
`lua_gyro_x/y_rads` (achieved, the FC's gyro, STATE2) and `lua_out_*` (what the
rate loop put on each surface, STATE2). The two views check each other; neither replaces the other.

---

## Project structure

*The rules — keep the exact package structure when moving files; the import-depth
rules — live in `CLAUDE.md` § "Project structure". This is the tree.*

`main.py` lives at the **repo root**, *outside* the package (it imports
`Companion.*` by name). Everything else is inside the `Companion/`
package, split into subpackages: `ui/`, `adapters/`, `testing/`, `tests/`.

`LUA/` is also at the repo root and is NOT a Python package. It holds the
steering laws that run ON THE FLIGHT CONTROLLER, the cascade and Lift Vector
Pointing. It is source, not a build
artifact, and it is covered by `package_sha256()` — see "Lua steering offload".
`main.py` is covered too: it is the entry point that wires the flight.

`Planning/` is at the repo root too: `sweep_plans.py`, which plans a fixed set of
rule chases with the Curve Planner offline for one steering law (its first
argument, `STEERING_LAW` when none is given) and checks each plan through that
law's plan-mode model, and the `plans_*.json` each run writes. It is a desk tool; nothing in it flies, and it is
not covered by `package_sha256()`.

`Arduplane/` is at the repo root as well. It is simulator INPUT: the Gazebo world
and aircraft model, and the C++ system plugin the model's aerodynamics use — see
"Simulator air: AltitudeLiftDrag". `LUA/test_plant.lua` reads the two SDFs at load
for its airframe. Nothing in it flies on the aircraft, and its sources are not
covered by `package_sha256()`.

The documents are at the repo root - `CLAUDE.md` (the rules), `ARCHITECTURE.md`
(this file), `TODO.md` (the flight tests waiting to be flown) and
`SUGGESTIONS.md` (open proposals) - with investigation reports in `Notes/`. They
fly nothing, and every `.md` is covered by `package_sha256()`.

```
.
├── main.py                          # entry point — repo root, OUTSIDE the package
├── CLAUDE.md · ARCHITECTURE.md · TODO.md · SUGGESTIONS.md   # the documents (hashed)
├── Notes/                           # investigation reports (hashed)
├── .gitignore
├── Planning/                        # offline Curve Planner runs — a desk tool, not hashed
│   ├── sweep_plans.py               # plans the rule chases for one law and checks each plan
│   └── plans_*.json                 # each run's results
├── Arduplane/                       # simulator input — NOT flight code, not hashed
│   ├── SDF/
│   │   ├── talon_runway.sdf         # the world: installed as ~/talon_sim/worlds/talon_runway.sdf
│   │   └── model.sdf                # the aircraft: installed as ~/talon_sim/models/talon_balloon_chaser/model.sdf
│   └── AltitudeLiftDrag/            # gz-sim LiftDrag in air that thins with height
│       ├── AltitudeLiftDrag.cc
│       └── CMakeLists.txt           # builds and installs libAltitudeLiftDrag.so to ~/.gz/sim/plugins
├── LUA/                             # runs ON THE FC — see "Lua steering offload"
│   ├── balloon_chaser_fc.lua               # entry point: the ArduPilot script
│   ├── modules deployed to scripts/modules/ (require() looks ONLY there):
│   │   protocol · constants · geometry · estimator
│   │   control_roll · control_pitch
│   │   cascade_state · track · cascade · excite · curve_flyer
│   │   path_attitude                # Lift Vector Pointing, the second steering law
│   └── desktop-only, NEVER deployed: ap_shim · ap_stubs · hello_selftest · test_*
│       (test_plant: the desktop rigid-body model of the simulated Mini Talon)
└── Companion/
    ├── __init__.py                  # public surface (BalloonChaser, interfaces, ControlGains)
    ├── constants.py
    ├── controller.py                # top-level orchestration; imports .ui.*, .testing.*, .adapters via main
    ├── control_laws.py              # ControlGains / PitchGains only; the laws run in LUA/
    ├── csv_logger.py                # defines package_sha256() + writes the CSV fingerprint
    ├── estimator.py
    ├── export_paths.py              # MACHINE-SPECIFIC: host-OS folder the export buttons copy to
    ├── file_export.py               # export table + WSL->host-OS copy logic (no Tk; testable headless)
    ├── geometry.py
    ├── gimbal_calibration.py
    ├── interfaces.py
    ├── controller_autotune_store.py # Controller Autotune: param store backing the gain tuner
    ├── controller_autotune_sweep.py # Controller Autotune: automatic gain sweep engine (the 't' key)
    ├── plant_id.py                  # plant-ID: open-loop rate plants, pitch/yaw/roll (the 'i' key)
    ├── plant_freq_id.py             # frequency-response ID of the roll axis (the 'k' key; refused)
    ├── plant_id_rescale.py          # offline sim->real rescale of the outer pan gains
    ├── constants_suggest.py         # writes the constants.py copies: 't', 'y' and the Gain Tuner's Copy
    ├── trim_map.py                  # trim map: elevator to hold attitude vs climb and airspeed (the 'e' key)
    ├── arduplane_autotune.py        # Arduplane Autotune: drives ArduPlane's built-in AUTOTUNE (--arduplane-autotune)
    ├── protocol.py                  # companion<->FC wire protocol (pack/unpack) + LUA/protocol.lua generator
    ├── gen_constants.py             # generates LUA/constants.lua from constants.py
    ├── curve_planner.py             # the Curve Planner: plans a chase's turn for the selected law, through a model of its plan mode
    ├── fc_param_manifest.py         # the FC parameter manifest the startup gate checks
    ├── adapters/                    # hardware adapters (sim + real)
    │   ├── __init__.py
    │   ├── mavlink_link.py
    │   ├── real_hardware.py
    │   ├── sim_gazebo.py
    │   ├── lua_offload.py           # AutopilotLink DECORATOR: offloads steering to the FC
    │   ├── lua_deploy.py            # verified script deployment (SHA-256, /proc cwd)
    │   └── lua_setup.py             # SCR_*/SERIALn_*/ARSPD_* check (read-only) + VM reload
    ├── tests/                       # desktop contract checks — DEV ONLY, nothing flies
    │   ├── __init__.py
    │   ├── bindings.desc            # the pinned firmware's own, byte for byte, for test_lua_bindings
    │   └── test_*.py run_tests.sh   # link/HUD/CSV/doc/deploy/no-micromanagement guards
    ├── testing/                     # test/eval harness + synthetic balloon injection
    │   ├── __init__.py
    │   ├── point_balloon_chaser.py
    │   └── synthetic_injector.py
    └── ui/                          # HUD overlays, Controller Autotune (gain tuner), keyboard handling
        ├── __init__.py
        ├── hud.py
        ├── keyboard.py
        └── controller_autotune.py   # Controller Autotune: interactive gain-tuner window
```

---

## Lua steering offload

*The contract rules — what `GUIDE` may and may not carry; claim control only in
ACRO; deployment; the fingerprint covers Lua — live in `CLAUDE.md` § "Lua
steering offload". This is the mechanism, the C-01/C-02 analysis, and the measured
evidence.*

The steering laws - the cascade and Lift Vector Pointing, one selected for every
mode by `steering_law` - run on the flight controller as an ArduPilot Lua script;
`--lua-path` carries the link and is required (only `--arduplane-autotune` runs
without it). The companion keeps computer vision, the gimbal, mode logic and
logging, and keeps no copy of either law: the HUD, the CSV, plant-ID and the
autotune sweep read the FC's own values from `STATE` and `STATE2`. The one piece
of steering the companion computes is a chase's PLAN, which the Curve Flyer runs
on the FC and the selected law flies in its plan mode; the Curve Planner's model
of each law's plan mode, which it flies candidate plans through, is the one copy
of a flying law in Python (§ "The Curve Planner and the Curve Flyer").

### What crosses the wire

`GUIDE` (companion → FC, every 50 Hz tick, tracking or not) carries two kinds
of thing, and the companion's configuration travels in two messages of its
own:

| kind | fields |
|---|---|
| SENSOR | `det_norm_x/y`, gimbal encoders, oracle bearing |
| OPERATOR INTENT | mode, flags, throttle, straight/manual setpoints |
| CONFIGURATION (not in GUIDE) | gains via `SET_GAIN`, the steering law among them; excitation DESCRIPTION via `EXCITE` |
| THE PLAN (not in GUIDE) | the Curve Planner's turn via `PLAN`: a bank schedule by progress, in the measure of the law it was made for, which it names, and a run-in's climb and depression; re-sent until STATE2 echoes its `plan_id` |

GUIDE's `flags` byte (layout in `protocol.py`) carries the detection's validity
and three bits of operator intent: bit 0 a detection is valid, bit 1
tracking, bit 2 the `p` test's oracle bearing is in use, bit 3
straight-and-level. Bits 4-7 are unused.

**Every value must fit its field.** Each GUIDE field is a fixed-size integer at
a fixed scale, and `SET_GAIN` carries a gain as a 32-bit float; struct refuses a
value the field cannot hold, and the refusal breaks the control loop on the tick
it is sent. So the tuner never accepts one. A setting whose only ceiling is
its field takes its tuner range from the protocol through `guide_field_range()`,
so the range moves with the field. A gain past the 32-bit
float range (`FLOAT32_MAX`) is refused like infinity, which is all a 32-bit
float can make of it. `test_tuner_registration.py` checks every GUIDE-carried
row against its field and types huge numbers into every row.

`STATE` and `STATE2` (FC → companion, every tick) carry the mirror back. A
payload's length travels in one byte, so a frame holds at most 255 payload bytes;
both are near that, STATE2 carrying what does not fit in STATE, sent right after
it. Both echo the seq of the GUIDE frame they were computed from. Together they
take a little over a quarter of the 921600 line. The exact bytes per tick and share are in the comment beside
`STATE2` in `protocol.py`; `test_link_contract` recomputes them from the structs
and from the baud read at both ends of the link, and fails when the comment
drifts.

The mirror of the no-control-output rule matters just as much: the FC must not
INVENT configuration the operator owns. A setting hardcoded on the FC makes it fly
a different configuration from the one the companion shows and logs, so the two
run different controllers while appearing to run one. Every gain the law reads
travels in `SET_GAIN`, and a new operator setting travels from the companion
the same way.

### C-01 and C-02 under this architecture

- **C-01 is met.** The script commands through
  `vehicle:set_target_throttle_rate_rpy()` and claims control only while ArduPlane
  is in ACRO. Rates go in as deg/s: no PWM quantisation, no trim map, no `RC*_DZ`
  coupling. While `nav_scripting` is active, `Plane::stabilize()` hands those
  rates straight to `rollController` / `pitchController.get_rate_out()` instead of
  calling ACRO's `Mode::run()` — and with `ACRO_LOCKING` = 0 (checked at startup)
  `Mode::run()` makes exactly those calls with the stick rates. So the aircraft is
  in ACRO and ACRO's own rate controllers close the inner loop on-chip against
  fresh gyro data; the only difference is that the rate demand comes from the
  script rather than the sticks. Under Lift Vector Pointing only the roll and
  pitch rates are commanded; the yaw rate controller is kept off
  (`set_rudder_offset(0, false)`), so the rudder stays at zero.
- **C-02** — the FC derives the body-frame balloon from gimbal ENCODER angles, never
  from pixels. On this airframe the gimbal is a Gazebo model driven by the companion
  over gz-transport, so ArduPilot has no mount driver and the encoders arrive in
  `GUIDE` instead of being read locally. **This is a partial win by construction**:
  attitude is on-chip and fresh, the encoder angle is not. A hardware mount on the
  autopilot would close that gap; nothing in software can.

### The script must CLAIM control, and must claim it narrowly

`set_target_throttle_rate_rpy()` only STORES the demand. ArduPlane uses it solely
while `nav_scripting_active()`, which only `nav_scripting_enable()` sets. Omit the
claim and the script computes correctly while ACRO flies with neutral sticks — a
slight nose drop and a shallow descent, with everything looking healthy.

Claim ONLY in ACRO, and re-claim every tick because `Mode::enter()` clears the flag.
`nav_scripting_enable()` also accepts CIRCLE/STABILIZE/FBWA/FBWB/CRUISE/**LOITER**,
so claiming with whatever mode is live hijacks LOITER — the mode this project
switches to when a test aborts — and the script flies on while the log reads
"returning to loiter". Any mode that is not ACRO must trigger an immediate
explicit release.

Claim ONLY while `GUIDE` says tracking, too. The companion sends a frame EVERY
tick and its tracking flag is the live instruction. A frame that stopped when
tracking stopped would leave the FC flying the previous balloon the next time
ACRO is entered - between sweep trials, for example, from the moment ACRO is
confirmed until the next injection. When the flag clears, the script releases on
that tick with `nav_scripting_enable(255)` (a mode that is never current, so
ArduPlane sets `enabled = false`). Merely not calling is not a release: ArduPlane
keeps flying the last stored command until its own 1000 ms `nav_scripting`
timeout. With the script released in ACRO, ArduPlane flies ACRO on the pilot's
sticks - centred sticks hold zero rates, and the throttle is the RC throttle
input - until the next claim.

ArduPlane's timeout watches whether the script CALLS, not whether its inputs are
fresh, so it never fires while the script runs. The script therefore checks
`GUIDE` itself: with no frame carrying a NEW seq for `GUIDE_STALE_MS`, it
releases (gate 7) and drops its balloon estimate. A companion stuck resending one
frame is no fresher than one that sends nothing. The next new frame re-claims.

Every other stop releases too. When an input the law was flying on disappears
(the gimbal angles, the attitude, the balloon estimate or the oracle bearing:
gates 2, 3 and 4), or a Lua error interrupts the tick, the script releases on
that tick and latches the cause as a FAULT in STATE's flags bits 5-7
(`protocol.FAULT_NAMES`). While a fault is latched it claims nothing (gate 8);
the companion halts on the flag (§ "Deliberate stops"), its hand-back clears
tracking, and that clears the latch. One stop is not a fault: a fresh chase
waiting for its first estimate releases with a severity-4 line and no latch.

The link is watched from both ends: the FC releases on a stale `GUIDE`, and the
companion halts when STATE stops arriving for `FC_LINK_TIMEOUT_S` (a stopped
script, a reloading scripting engine, a broken link). Both constants are in
`constants.py`.

### Deployment

ArduPilot EXECUTES every `.lua` in the scripts directory, and `require()` searches
ONLY `scripts/modules/` (`LUA_LDIR = SCRIPTING_DIRECTORY "/modules/"`). Modules in
`scripts/` fail twice over: invisible to `require`, and run as scripts that return a
table instead of an update function. Nothing loads and the only symptom is silence.

`SCRIPTING_DIRECTORY` is `./scripts` — relative to the AUTOPILOT PROCESS's working
directory, which for `sim_vehicle.py` is wherever the operator launched it, NOT the
source tree and NOT where `eeprom.bin` happens to sit. `lua_deploy` reads
`/proc/<pid>/cwd` and refuses to deploy anywhere else. It checks before it copies
anything, so a refused deploy leaves no second set of scripts behind.

Because ArduPilot runs everything in `scripts/`, renaming the entry script does
not retire the old file: the copy an earlier deploy left there keeps running
beside the new one, on the same serial port, each script claiming the aircraft
with its own law. `RETIRED_ENTRY_SCRIPTS` in `lua_deploy.py` lists every former
entry-script name (`pursuit_fc.lua`), and the deploy deletes any it finds. The
deploy reaches only SITL's directory; on the real aircraft the old file is
deleted from the SD card by hand.

Scripts are read when the scripting engine starts. `MAV_CMD_SCRIPTING` (42701) with
`SCRIPTING_CMD_STOP_AND_RESTART` (3) reloads the VM in place — send it **once**: a
second STOP arriving while the VM is rebuilding parks it in the 1 Hz wait loop
indefinitely, and both scripts go silent.

### The source fingerprint covers Lua

`package_sha256()` covers `.lua` as well as `.py`, and every `.md`. With the
steering laws on the FC, a `.py`-only hash gives two runs flown by completely
different steering code IDENTICAL fingerprints — the one thing a fingerprint
exists to make impossible. It includes every `.lua` file that flies,
`path_attitude.lua` among them, and deliberately excludes `ap_shim`, `ap_stubs`
and the files named `test_*`, `xcheck_*`, `e2e_*`, `replay_*` and `hello_*` -
the test drivers and `test_plant.lua` - which never reach the aircraft.

### Desktop stubs validate assumptions, not behaviour

`ap_shim.lua` and `ap_stubs.lua` are hand transcriptions of ArduPilot's
bindings. Where they only record calls, a desktop pass shows the script calls
what it means to; where they REIMPLEMENT ArduPilot behaviour, it proves
nothing, because the stub agrees with whatever the script assumed. The traps
this has hidden, and the rotation choice that removes one of them, are in
`CLAUDE.md` § "ArduPilot binding gotchas". `Companion/tests/test_lua_bindings.py`
checks every call against ArduPilot's own `bindings.desc`, including argument
counts and `uint32_t` conversion.

`test_plant.lua` goes further than the stand-ins: it REIMPLEMENTS the airframe and
ArduPlane's rate loop (AC_PID, the FLTT, FLTE and FLTD filters, the SMAX slew
limiter), with two numbers fitted to SITL (`MIXING_GAIN`, `AIL_GAIN`). A pass on
it - § 18 of `test_balloon_chaser_fc.lua` - is evidence about that model, not
about SITL or the aircraft; so is a pass on the point-mass aircraft of §§ 15 and
16.

### Verification

Correctness is argued from the desktop contract checks in `Companion/tests/`
— among them `test_no_micromanagement` (the wire carries no control output but
the PLAN),
`test_link_contract`, `test_lua_bindings` and the CSV/HUD/deploy guards — plus the
Lua desktop drivers in `LUA/` that exercise each module against `ap_shim`, the
closed-loop sections of `test_balloon_chaser_fc.lua` - § 15 the cascade under
the rules of a chase, § 16 the Curve Flyer under each law, § 18 Lift Vector
Pointing over the whole sphere - which fly the whole script on desktop aircraft
(point masses, and `test_plant.lua`'s rigid body), and
analysis of the flight log after a run. There is no offline replay of either
law. The script also loads and runs on a SITL build of the pinned firmware
(§ "SITL from conda-forge, where Gazebo's package host is unreachable"), which
checks it against the real bindings rather than the stand-ins.

There is NO in-flight A/B and no companion copy of either law to make one with:
the Curve Planner's model of each law's plan mode flies only candidate plans,
on its own point-mass aircraft. Do not reintroduce one for comparison: the FC
and the companion see different inputs a transport delay apart, so their gap
measures the link, not the law.

Roll-to-plane, Lift Vector Pointing, both laws' plan modes, the rear hemisphere
and |pitch| > 85° have never been entered in a real flight and are UNVERIFIED
against flight data.

### Measured, not assumed

Reading a sensor at 50 Hz does not make it 50 Hz. The FC counts how often each value
actually CHANGED and reports it in `STATE` (`hz_quat`, `hz_gyro`, `hz_vel`, `hz_asp`).
A healthy aircraft reads quat 49, gyro 49, vel 49, airspeed 10. Airspeed is
lower because ArduPilot updates it at ~10 Hz, not because the feed is stale.

`lua_dt_ms` is the measured interval since the previous script tick, before the
0.5 s clamp the law applies to its own `dt`, so a stall shows at its real length.

`ARSPD_TYPE` is checked at startup (`lua_setup.py`) because without it the FC has
no airspeed estimate at all: `ahrs:airspeed_EAS()` refuses without one. It is a
reboot-required parameter: set it by hand, then reboot the autopilot. How the
binding returns is in `CLAUDE.md` § "ArduPilot binding gotchas".

`lua_seq_lag` measures the round trip directly — `STATE` echoes the `GUIDE` seq it was
computed from (as `STATE2` does, so `lua_state2_seq` pairs a row's two frames), so it
needs no clock synchronisation. The `p` test's start gate
(`cascade_has_run_since_inject`) uses the same echo: it waits for a commanded
`STATE` answering a `GUIDE` frame sent after the injection, so the steering error
it reads belongs to the new balloon. Read the lag from the flight log rather than
quoting a figure here.

### The flight path is taken once per tick, as read

Every error the laws close is measured from the FLIGHT PATH, not the nose
(§ "What each error is measured from"). `track_world_balloon` works the path
out once, at the start of the tick and before anything reads it, from that
tick's `ahrs:get_quaternion()` and `ahrs:get_velocity_NED()`, and leaves it in
the track state for whichever law runs and for the Curve Flyer: the path's
angle to the horizon, the balloon in the path frame, and for Lift Vector
Pointing the path frame in body axes, the path's speed and world down in body
axes; for the Curve Flyer the balloon's elevation, the path's world direction
and any line in the path frame. It is used as read: no filter, no glitch guard,
no clip. Lift Vector Pointing's lead is part of its guidance, not a filter: it
reads its angle off the balloon from the path `att_lead_s` ahead, projected from
the accelerometer, and logs the angle as read beside it (`lua_att_theta_now_deg`). The one fallback is speed: at or below `FLIGHT_PATH_MIN_SPEED_MPS` the
velocity's direction is noise and the nose stands in.

The vertical and lateral flight-path markers, `vel_marker_rad = atan2(-v_z,
v_x)` and `lat_marker_rad = atan2(v_y, v_x)`, are the path's two angles off
the nose. They are logged (`lua_vel_marker`, `lua_lat_marker`); the laws read
the path frame instead.

A filter, a held reading or a clip only misplaces the path, and the error is
flown. The angle between nose and path is not a slow trim: a hard pull swings
it by several degrees within a second, and the climb that follows holds a
different value again. The law puts the ESTIMATED path on the balloon, so
whatever the estimate trails or misreads by is flight-path error, one for one.
The path itself trails the nose only briefly on this airframe, so steering by
the unfiltered path stays well damped at the pitch gains flown.

Smoothing, where it is needed, happens at the source. The AHRS values are
ArduPilot's estimates: EKF3 builds the velocity from the IMU and blends each GPS
correction in over `EK3_TAU_OUTPUT` (`NavEKF3_core::calcOutputStates`), so a
fix arriving does not step it. That parameter is the place to change it; a
second filter in the script is not.

**What SITL feeds the script depends on `AHRS_EKF_TYPE`.** At 3 it runs EKF3,
the estimator the aircraft flies. At 10 (`AP_AHRS_SIM`) attitude, velocity and
wind are the simulator's truth, while the gyro and the airspeed (the airspeed
sensor comes first in `AP_AHRS::_airspeed_EAS`) are still simulated sensor
readings. A result that depends on estimator behaviour - noise, lag, a wind
estimate - has to be flown at 3 to say anything about the aircraft.

### Inactive integrators are cleared

An integrator is a memory of error that its output was closing. The moment
that output stops reaching the aircraft, the memory describes nothing being
flown, and held, it would add a stale push the moment it is active again. So
every integrator is CLEARED, never held, when it stops being used, for any
reason:

- **A tick the law does not fly.** On a tick the law runs but nobody flies - a
  mode other than ACRO, a refused claim, a latched fault -
  `clear_unflown_integrators()` zeroes the pan, pitch, roll-to-plane and
  yaw-pointing integrators (`cascade_state.reset_integrators`), and Lift Vector
  Pointing's roll and pitch integrators when it ran (`path_attitude.clear_integrators`).
  After a tick the law does not run at all (a gate before it, a stand-down), the
  first tick it runs again is a fresh start (`reset_mode_entry`, and
  `path_attitude.clear_integrators`): integrators cleared, derivatives re-seeded.
  A fresh chase resets both laws the same way.
- **Waiting for a PLAN.** While the Curve Flyer holds the rates at zero for a
  PLAN, the law runs and is logged but not flown, so every integrator and
  derivative memory is cleared each such tick.
- **A gain of 0.** Ki 0 on the pitch, pan, roll-to-plane or yaw-pointing law,
  or on Lift Vector Pointing's roll or pitch (`att_roll_ki`, `att_pitch_ki`),
  clears that integrator rather than leaving its last value in the command.
  Lift Vector Pointing's two are cleared outside `att_i_zone_deg` too.
- **No share.** The pan integrator when bank-to-turn has no share of the roll
  command (or the roll gain is 0), and on every tick a PLAN sets the cascade's
  bank; the roll-to-plane integrator while roll-to-plane has no share; the
  yaw-pointing integrator when its steep schedule gives it no share or the
  lateral marker is invalid, while the Curve Flyer owns the chase, and on every
  tick Lift Vector Pointing flies, when the whole cascade, not run, is reset
  every tick.
- **An override.** A surface plant-ID drives open loop clears the integrators
  of the laws that surface would have carried: aileron the pan and
  roll-to-plane integrators or Lift Vector Pointing's roll integrator, elevator
  or rudder the pitch integrator or Lift Vector Pointing's; plant-ID's rate
  replacement does the same for the axis it replaces.
- **The airspeed hold.** On any tick its throttle is not flown (a gate, a
  release, a script error) `step()` clears it
  with `reset_throttle`, which also disengages the loop: the next engagement
  re-seeds the integrator from the operator's throttle, so it restarts from
  the throttle already flying, not from zero.

Held is different from cleared. An integrator whose output is still flown but
whose law should not accumulate - the pan integrator while roll-to-plane has
part of the command, any integrator while its output sits at a limit in the
same direction - is frozen at its value, because that value is still part of
what the aircraft flies. Lift Vector Pointing's two are clipped to their ACRO
rates instead.

## The roll axis: bank-to-turn and roll-to-plane

*The rules this serves are C-03 and C-04 in `CLAUDE.md`: one law for every mode,
and balloons anywhere around the aircraft. This is the cascade's roll axis
(Steering Law 0); Lift Vector Pointing has neither law below (§ "Lift Vector
Pointing: the second steering law"). The code is `LUA/cascade.lua` (the blend
and roll-to-plane) and `LUA/control_roll.lua` (bank-to-turn);
`LUA/test_balloon_chaser_fc.lua` section 10 pins the behaviour below.*

The roll command mixes two laws, each a roll rate: bank-to-turn's, from its bank
setpoint and `roll_gain`, and roll-to-plane's, from its own gain, integrator,
derivative term and feedforward. The mix is a weighted sum of the two rates, its
weights set by how steep the flight path is. Under a PLAN neither runs: the
PLAN's bank, clipped to the bank limit, is the setpoint, and roll-to-plane's
share is 0 (§ "Each law's plan mode").

### Bank-to-turn

`compute_roll_command` turns the sideways pointing error - `asin(t'_y)`, the
balloon's angle to the side of the flight path in the path frame (§ "What each
error is measured from"), continued past ±90° into the rear hemisphere - into
a bank setpoint (P + I + D, clipped at `bank_lim_deg`), and the roll rate closes the gap between that setpoint and the
bank fed back: the bank proxy `asin(sin(roll) cos(pitch))`, the lift's sideways
tilt from the vertical, which has no pole at vertical.

Its setpoint is measured from the horizon, so it finishes wings level, and a
turn holds the aircraft up: on a shallow path, where most of gravity acts
across the path, that is what a turn needs.

It has three weak spots:

- **It reads only the sideways part.** A balloon right-and-above and one
  right-and-below read the same, though they need opposite rolls. With the
  balloon below the wing plane, banking toward it swings it further the same
  way: the error grows and the request saturates.
- **The bank limit limits the proxy, not the bank.** The proxy
  cannot read more than `asin(cos(pitch))` (16° at 74° nose-up), so at steep
  pitch a request near `bank_lim_deg` is out of its reach and the roll goes on.
  With the balloon off to the side that is how the law steers near vertical: the
  roll continues until the sideways error is gone, which lines the balloon up
  with the pull. With the balloon under the belly in a steep climb it carries
  the aircraft over the top.
- **Inverted, the proxy loop runs backwards.** The proxy reads about 0 upside
  down and moves the wrong way as the aircraft rolls, so its only resting point
  is upright.

Translating `bank_lim_deg` into the proxy's terms, so that it limits the real
bank at every pitch, stops the roll-over and stops the steering with it: in a
steep dive with the balloon off at the wingtip, the capped law halts at the bank
limit with the sideways miss still open, where the proxy law rolls on until the
miss closes. Lift Vector Pointing holds Bank Lim on the same proxy, by a
different route (§ "Lift Vector Pointing: the second steering law" → "The bank
limit, on the cascade's bank proxy").

### Roll-to-plane

Looking along the nose as at a clock face, the balloon sits at the clock angle
`atan2(b_y, -b_z)`, `b` the balloon in body axes: 0 over the canopy (12
o'clock), +90° to the right, ±180° under the belly. At 12 or 6 the balloon lies
in the body's plane of symmetry, the plane the pitch rate swings the nose in,
and the clock angle is itself the roll error - no bank setpoint, no bank
measurement, nothing measured against the horizon. ACRO's rate loop rolls the
aircraft about the nose, so a roll moves the balloon round this clock by
exactly the angle rolled (`test_balloon_chaser_fc.lua` section 10 pins it).

The flight path is not the nose: it sits off it by the angle of attack and the
sideslip, and a pull swings the PATH, in the plane through the path and the body
z axis. That plane is the body's plane of symmetry only while the path lies in
it, with no sideslip, and a roll about the nose turns any angle of attack into
sideslip as it goes (§ "The yaw axis: coordination and pointing"). So a roll
that centres the balloon over the nose can leave it to the side of where a pull
takes the path, and the pitch law, which reads the path, closes only the part
of the miss in that plane. In a near-vertical climb with the path 7° toward the
canopy from the nose and the balloon 1° to the side of the path and 0.7° beyond
it, the clock round the nose reads about 8° and the clock round the path about
56°. Rolling the 8° leaves the balloon still about 0.9° to the side of where a
pull takes the path; rolling the 56° would leave it on that plane.

**Why the nose and not the path.** Every other balloon error is read round the
path, so a clock round the path looks like the consistent choice. The two
clocks choose the same end and roll the same way except for a balloon between
the nose and the path, and there they ask for opposite rolls. Take the same
climb, the path 7° toward the canopy from the nose, with the balloon halfway
between the two and 1° to the right. Round the nose it is above and to the
right, at about 16°: roll-to-plane rolls 16° right, to 12. Round the path it is
below and to the right, at about 164°: 6 is nearer, so a clock round the path
would roll 16° left and leave the pitch law to push.

The pull the wing is already holding decides between them. On a steep path it
is not zero: it holds the path against gravity's share across it (cos 79° ≈
0.19 g in a 79° climb) and against the part of the propeller's push that the
nose, sitting below the path, aims toward the belly. A roll about the nose tilts
that pull with the wings, and the path starts to slide the way it tilts. Near
the end of a chase the miss is small, so the held pull is the biggest force
across the path, and the way a roll tilts it decides which way the path moves
first.

- **Rolling right**, as the nose's clock asks, tilts the held pull toward the
  balloon, and the path moves toward it at once, as banking toward a balloon
  does in a level turn.
- **Rolling left**, as the path's clock asks, tilts it away, and the path slides
  left, away from the balloon. That roll's plan is to push once the balloon is
  at 6, and to push, the pitch law must first unload the pull it is holding.
  Until it has, the path keeps sliding, the balloon reads further right round
  the path, and the path's clock asks for more left roll: a rolling fall that
  feeds itself.

The pitch law reads the path either way, so only the roll differs.

This rests on the path sitting toward the canopy from the nose. The simulated
wing is set at a built-in angle of attack (`a0` = 0.13 rad in
`Arduplane/SDF/model.sdf`), so, counting the wing alone, it lifts with the nose
on the path at about 1 g at 14 m/s and 2 g at 20 m/s, and with the nose below
the path under any lighter load: the load of a steep path near the end of a
chase. A pull heavy enough to lift the nose above the path swaps the two
clocks' roles in the band between them, and there the nose's clock is the one
that tilts the pull away. `lua_vel_marker`, positive with the path toward the
canopy from the nose, says which side the path was on.

Roll-to-plane rolls the balloon to whichever of 12 and 6 o'clock is nearer, and
the pitch law, unchanged, pulls toward a balloon above the path or pushes
toward one below. The choice holds until the other end is nearer by
`plane_side_hold_deg` (Gain Tuner row Roll-to-Plane Hold, default
`PLANE_SIDE_HOLD_DEG`): at 3 and 9 o'clock both ends are equally far, and a free
choice there swings the command from full one way to full the other. It is made
afresh on each chase (`reset_mode_entry`).

Its rate is

    plane_gain x err  +  integrator  +  plane_kd x err rate  +  plane_ff_gain x drift

- **`plane_gain`** (Roll-to-Plane Gain, default `PLANE_GAIN`): deg/s of roll
  per degree still to roll. A roll about the nose
  moves the balloon round the clock by exactly the angle rolled, so this alone
  closes the error with a time constant of 1/gain.
- **The integrator** (Roll-to-Plane Ki, default `PLANE_KI`): deg/s of roll,
  growing at Ki x error, so a Ki change moves its rate of growth, not its
  value. It owns at most `plane_windup` (Roll-to-Plane Windup, a fraction of the
  ACRO roll rate), holds while the command it joins is already at the ACRO roll
  rate its way, and is cleared while roll-to-plane has no share, on a new chase
  and at Ki 0. A 12 <-> 6 change keeps it: the clock error's rate is the same at
  either end.
- **The derivative term** (Roll-to-Plane Kd, default `PLANE_KD`, filtered at
  Roll-to-Plane Kd LPF Rate, default `PLANE_KD_RATE`): `plane_kd` x the clock
  error's rate, deg/s of roll per deg/s, dimensionless like `pitch_kd`. The
  rate is the clock angle's own, so it is the balloon's whole motion round the
  clock - the roll carrying it, the nose's yaw and pitch swinging it, and its
  own drift - where the feedforward takes the drift alone. While a roll carries
  the balloon toward its end the rate opposes the error, so the term takes rate
  off the roll before the balloon arrives, and a balloon swinging past the end
  meets a roll back against the swing. A 12 <-> 6 change steps the error by
  180° and leaves the clock where it is, so it does not reach the rate. It is
  held at zero, its history re-seeding, while roll-to-plane has no share, and
  starts afresh on each chase.
- **The feedforward** (Roll-to-Plane FF, default `PLANE_FF_GAIN`, filtered at
  Roll-to-Plane FF LPF Rate): the balloon's own drift round the clock - the clock
  angle's rate plus the aircraft's roll rate, since a roll turns the balloon back
  by the angle rolled. A world-fixed balloon with the aircraft rolling has
  none; a path swinging round the balloon has plenty. Without it the
  proportional law trails a drifting balloon by drift/gain. It is held at zero,
  its history re-seeding, while roll-to-plane has no share.

Near the nose the clock is meaningless: a balloon 5° right of the nose at its
height reads 3 o'clock, a 90° roll to fix 5°, and roll-to-plane rolls for it at
its full gain. Its rate swings with it: a balloon 2° off the nose moving 1°/s
across it goes round the clock at about 29°/s, and the feedforward and the
derivative term read that. `lua_plane_err_deg` with a small
`lua_plane_off_nose_deg` shows it. And roll-to-plane knows nothing of gravity.

### The blend

Roll-to-plane's share of the roll command is one straight line in the flight
path's steepness, and bank-to-turn has the rest:

    w            = path_share
    roll command = (1 - w) x bank-to-turn's rate  +  w x roll-to-plane's rate

The line is 0 below its start and 1 at and beyond its full, a straight line
between; with full at or below start it is a step at start. At `w` = 0 the roll
command is bank-to-turn's exactly, at `w` = 1 roll-to-plane's: its gain on the
degrees still to roll, its integrator, its derivative term and its feedforward.

- **`path_share`, the master switch: how much of gravity acts across the flight
  path.** It reads |flight-path angle| - the direction the aircraft is MOVING
  against the horizon, from the path `track_world_balloon` took from this
  tick's velocity, ground-relative - with the nose's pitch standing in below
  `FLIGHT_PATH_MIN_SPEED_MPS`, where the velocity's direction is noise (Gain
  Tuner rows Roll-to-Plane Steep Start / Steep Full, `plane_steep_start_deg` /
  `plane_steep_full_deg`). The cosine of that angle is the share of gravity
  acting across the path. Moving level it is all of it: the lift must hold the
  aircraft up while it turns, which bank-to-turn's banked pull does and
  roll-to-plane does not. Moving vertically it is none: gravity only changes the
  speed, the turn is pure geometry, and "bank" means nothing. The bank does not
  enter it - knife-edge on a level path is all gravity across the path, and
  bank-to-turn is the law that rolls the lift back up - and neither does the
  nose, which sits off the path by the angle of attack and the sideslip, most
  when slow and pulling hard. The angle cannot pass 90°, so 90 / 90 switches
  roll-to-plane off.

With the defaults (`PLANE_STEEP_START_DEG` / `PLANE_STEEP_FULL_DEG` 65 / 80) and
the path along the nose:

| Case | Path angle | `w` |
|---|---|---|
| Level, balloon anywhere | 0° | 0 |
| Knife-edge on a level path, balloon 80° off | 0° | 0 |
| 50° climb | 50° | 0 |
| 70° dive | 70° | 0.33 |
| 74° climb, balloon under the belly (the roll-over) | 74° | 0.60 |
| 85° dive | 85° | 1 |

A ramp - full above start - hands over gradually. With a single switch angle, a
balloon wobbling across it flips the command between the two laws on every
crossing, and they can ask for opposite rolls (a balloon below and to the right:
bank-to-turn rolls right, roll-to-plane rolls left to bring it to 6). The cost
of a ramp is its middle: where the two laws disagree in direction the mix
partly cancels, and the aircraft holds a bank short of where either law would
take it. `test_balloon_chaser_fc.lua` section 10 flies that climb on roll alone with
pitch held, at the defaults with the path at the nose's 74°, and checks that it
levels the wings without first banking further out.

The pan integrator is held while roll-to-plane has any share, so it neither
accumulates an error that share is closing nor banks it at the hand-over. When
bank-to-turn has no share at all, and on every tick a PLAN sets the bank, it is
cleared (§ "Inactive integrators are cleared"). Roll-to-plane's integrator, derivative term and feedforward run only
while it has a share, `w` above 0.

**In the Gain Tuner** the settings that decide the share come first among the
cascade's groups, after BOTH LAWS, in Pan Law Blending: Roll-to-Plane Steep
Start, Steep Full.
Roll-to-plane's own law follows in ROLL - roll-to-plane: Hold, Gain, Ki,
Windup (0-1), Kd, Kd LPF Rate, FF, FF LPF Rate. Each row has an Info text and a
reset, and each setting is logged as a `g_plane_*` column.

**The log carries every part:** the share as `lua_plane_path_share`
(bank-to-turn has 1 minus it); the angle it reads as `lua_plane_path_deg` (with
`lua_plane_path_from_vel`, 0 when the nose stood in); `lua_plane_err_deg`
(degrees still to roll to the chosen end, round the nose) and `lua_plane_side`
(1 = 12 o'clock, -1 = 6); the integrator as `lua_plane_i_dps` and
`lua_plane_i_frozen`; the derivative term as `lua_plane_err_rate_dps` (the clock
error's filtered rate) and `lua_plane_d_dps`; the feedforward as
`lua_plane_drift_dps` and `lua_plane_ff_dps`. `lua_plane_off_nose_deg` and `lua_plane_axis_up` (the
world-vertical part of the turn axis, nose x balloon: 1 a heading change, 0 a
climb or dive) are logged, not flown. Every `lua_plane_*` column is blank on a
tick Lift Vector Pointing flies, and `lua_plane_path_share` reads 0 under a PLAN.

---

## The yaw axis: coordination and pointing

*The rules this serves are in `CLAUDE.md` § "Flight-control invariants" (the
actuation surface is roll rate, pitch rate, a yaw rate and throttle; under the
cascade the yaw rate is coordination plus pointing) and § "ArduPilot binding
gotchas" (the yaw path's gates). This is the cascade's yaw axis: Lift Vector
Pointing commands no yaw (§ "Lift Vector Pointing: the second steering law" →
"Into body axes, and the rudder"). The code is the yaw block of
`LUA/balloon_chaser_fc.lua`; `LUA/test_yaw_coord.lua` pins it.*

The yaw command is

    r_cmd        = clamp(coordination + pointing, +-ACRO_YAW_RATE)
    coordination = yaw_coordination_ff x (turning yaw + rolling yaw)

### Why the command carries coordination

Past the three gates (`CLAUDE.md` § "ArduPilot binding gotchas") ArduPlane's yaw rate loop
holds the body yaw rate at exactly the command. It is the whole yaw rate, not an
addition to the yaw the aircraft has, so a clean aircraft's own yaw has to be
asked for:

- **Turning yaw**, `g sin(roll) cos(pitch) / V` (`lua_yaw_ff_coord`): a banked
  turn swings the nose round with the heading, about 12°/s at 26° of bank and
  20 m/s. The turn is kinematics, so V is the true airspeed: the EAS, floored at
  `AIRSPEED_MIN` as ArduPlane floors its own, times EAS2TAS. On the EAS alone
  the rate would read high by EAS2TAS wherever the air is thinner than at sea
  level.
- **Rolling yaw**, `p tan(alpha)` (`lua_yaw_ff_ari`): with the path `alpha`
  below the nose, a roll about the nose swings the path round the nose and turns
  `alpha sin(phi)` of it into sideslip; a roll about the path needs
  `r = p tan(alpha)`. `p` is the roll rate the tick commands, so the yaw loop
  moves with the roll loop instead of following the slip the roll has already
  built. `alpha` is the path's angle below the nose in the body pitch plane, the
  negative of the vertical marker from the path `track_world_balloon` took this
  tick: zero, the nose, at or below `FLIGHT_PATH_MIN_SPEED_MPS`. It grows without
  bound as the path nears 90° off the nose, where no roll about the nose turns
  about the path; the `ACRO_YAW_RATE` ceiling bounds the command.

Without them the rate loop holds the yaw at the pointing term's value. It
fights every turn and every roll with rudder, and on the V-tail the elevator
shares that deflection. A roll about the nose also turns the angle between nose
and path into sideslip, which moves every error read in the path frame and
takes the path out of the body's plane of symmetry, the plane roll-to-plane's
clock, read round the nose, puts the balloon in. In a
near-vertical climb with the path 7° off the nose, rolls of ±60° at the ACRO
roll rate with pointing on and no coordination turn that angle into up to 7° of
sideslip, following `alpha sin(roll change)` closely, while the rudder holds up
to three quarters of full deflection against a demand under 2°/s.

**Yaw Coordination FF** (`yaw_coordination_ff`, default `YAW_COORDINATION_FF`):
1 is the geometry, 0 none; below 1 under-coordinates, above over-drives. It is a
feedforward, so it costs no phase and a wrong value only slips the aircraft. Any
value but 0 runs the yaw rate loop, with pointing on or off; with every yaw
setting at 0 the rudder is released (`set_rudder_offset(0, false)`), not held.

### Pointing

`yaw_point_kp` and `yaw_point_ki` steer on `pan_err`, the path-frame sideways
error bank-to-turn closes, so the rudder agrees with bank-to-turn on where done
is, and with roll-to-plane while the path lies in the body's plane of symmetry.
It is a bounded assist: the rudder moves the path only through sideslip.
Yawing the nose moves no error read in the path frame, but it turns the
balloon round the nose, so it moves roll-to-plane's clock. The output is
bounded by `yaw_point_lim`, the integrator by `yaw_point_windup` of it.

Yaw Point Steep Schedule (`yaw_point_pitch_sched`, off by default) multiplies
the output by the roll blend's `path_share`: none on a path shallower than
Roll-to-Plane Steep Start, all of it past Steep Full. It is the same number
that hands the roll command from banking to roll-to-plane, so one reading of
"banking is losing its grip" moves both axes. With Steep 90 / 90 it is 0 except
moving straight up or down. The pointing integrator grows by the same share, so
a shallow path that gives the output no share clears the integrator, rather
than winding it up to release when the path steepens.

While the Curve Flyer owns the chase - waiting for its PLAN, flying it, holding
a line, a run-in and its dive - pointing is off and its integrator cleared:
it steers on bank-to-turn's error, and bank-to-turn does not run under a PLAN.
The coordination stays, so the roll still turns about the path.

### The log

`lua_yaw_ff_coord` (turning yaw) and `lua_yaw_ff_ari` (rolling yaw), both
unscaled; `g_yaw_coordination_ff`; `lua_yaw_coordination_dps`, the coordination
the command carried before the ceiling; `lua_yaw_point_dps` and
`lua_yaw_point_i_dps` (pointing); `lua_yaw_rate_cmd_dps`, the command after the
ceiling; `lua_out_rudder`, what ArduPlane asked of the rudder. The rolling yaw,
the coordination and the command are blank on a tick the FC did not command, and
with pointing on every tick Lift Vector Pointing flies.

---

## What each error is measured from: path, nose or gravity

*The rules this serves are in `CLAUDE.md`: the alignment criterion measures the
flight path (§ "What the `p` and `x` tests are for"), and the flight path is
used as read (§ "Lua steering offload"). The code is `LUA/track.lua` (the path,
the path frame, `pan_err`, the pitch error), `LUA/control_roll.lua`,
`LUA/control_pitch.lua`, `LUA/cascade.lua` (roll-to-plane and the blend) and the
yaw block of `LUA/balloon_chaser_fc.lua`. This section is the cascade's; Lift
Vector Pointing reads the same path frame, its angle off the balloon from the
path led `att_lead_s` ahead (§ "Lift Vector Pointing: the second steering
law").*

Every error the laws close but one is an angle between the balloon and the
flight path, read in one frame with one formula per kind of error; the one is
roll-to-plane's clock, read round the nose. A few terms that close no balloon
error read the nose or gravity instead.

- **Path** - the direction the aircraft is moving: the FC's velocity
  (`ahrs:get_velocity_NED()`), ground-relative, rotated into body axes, taken
  once at the start of the tick (§ "The flight path is taken once per tick, as
  read").
- **Nose** - the body x axis, where the aircraft points. It sits off the path
  by the angle of attack vertically and the sideslip sideways, most when slow
  and pulling hard.
- **Gravity** - the world vertical. Not a balloon error: the bank the roll loop
  feeds back.

### The path frame

`track_world_balloon` expresses the unit balloon vector `t` in a frame that looks
along the path:

- `x'` along the path;
- `y'` along `z_body x x'`, square to both the path and the body z axis;
- `z'` = `x' x y'`, toward the belly.

The `x'`-`z'` plane holds the path and the body z axis: it is the plane a pull
swings the path in. With the path on the nose the frame IS the body frame, so
at or below `FLIGHT_PATH_MIN_SPEED_MPS`, where the nose stands in, every error
reads exactly as measured from the nose. A path along the body z axis (falling
flat) leaves `y'` without a direction, and the nose stands in for the frame
there too.

With `t' = (t'_x, t'_y, t'_z)` the balloon in this frame, the path-frame errors
are two readings of it:

| Reading | Formula | Used by |
|---|---|---|
| To the side | `asin(t'_y)`, continued past ±90° behind the path | bank-to-turn, yaw pointing; the Curve Flyer's bank toward the balloon (`b_toward`) |
| Up or down | `atan2(-t'_z, hypot(t'_x, t'_y))` | the pitch law (under a PLAN, of the Curve Flyer's aim point) |
| Off the path | `atan2(hypot(t'_y, t'_z), t'_x)` | the Curve Flyer's progress and the turn's completion |

Roll-to-plane's clock reads the balloon in body axes instead, `b` with the nose
along `x`: `atan2(b_y, -b_z)`, round the nose. For a balloon between the nose
and the path a clock round the path asks for the opposite roll, which tilts the
pull the wing is holding away from the balloon (§ "Roll-to-plane"). Two things
follow.

- **Bank-to-turn and roll-to-plane share one zero only without sideslip.** The
  sideways error is zero exactly when the balloon lies in the `x'`-`z'` plane;
  the clock reads 12 or 6 when it lies in the body's plane of symmetry. The two
  planes are one while the path lies in the body's plane of symmetry - an angle
  of attack alone keeps it there - and a sideslip parts them, leaving the blend
  holding the aircraft between two aims.
- **The rudder moves no path-frame error, but it moves the clock.** Yawing the
  nose turns the balloon and the path together about the body z axis the frame
  is built on, so neither reading changes; it turns the balloon round the nose.

### Roll: bank-to-turn

| Term | Measured from | Quantity |
|---|---|---|
| Pan error, P (`pan_p_gain`) | Path | `pan_err = -asin(t'_y)`, continued past ±90° by `rear_hemisphere_pan_err`: the sideways part only |
| Pan integrator (`pan_ki`) | Path | `pan_err` integrated |
| Pan derivative (`pan_kd`) | Path | `pan_err`'s rate, low-passed at `pan_kd_rate` |
| Roll-angle loop (`roll_gain`) | Gravity | the bank setpoint minus the bank fed back, the proxy `asin(sin(roll) cos(pitch))`: the lift's sideways tilt from the vertical |

### Roll: roll-to-plane

| Term | Measured from | Quantity |
|---|---|---|
| Error, P (`plane_gain`) | Nose | the clock angle `atan2(b_y, -b_z)` round the nose, `b` the balloon in body axes, less the chosen end (12 or 6 o'clock) |
| Integrator (`plane_ki`) | Nose | the clock error integrated |
| Derivative (`plane_kd`) | Nose | the clock angle's rate, low-passed at `plane_kd_rate` |
| Feedforward (`plane_ff_gain`) | Nose | the clock angle's rate plus the roll rate, low-passed at `plane_ff_lpf_rate` |
| 12-or-6 choice (`plane_side_hold_deg`) | Nose | the clock angle |

### The blend

| Share | Measured from | Quantity |
|---|---|---|
| `path_share` (Steep Start / Full) | Path, against gravity | the size of the flight-path angle, `atan2(-v_d, v_horiz)` in world axes; the Euler pitch at or below `FLIGHT_PATH_MIN_SPEED_MPS`; 0 under a PLAN |

### Pitch

| Term | Measured from | Quantity |
|---|---|---|
| Pitch error, P (`pitch_gain`) | Path | `e_pitch = atan2(-t'_z, hypot(t'_x, t'_y))`, the balloon's elevation above the path frame's horizontal plane: the up-down part only; under a PLAN, the Curve Flyer's target's |
| Pitch integrator (`pitch_ki`) | Path | `e_pitch` integrated |
| Pitch derivative (`pitch_kd`) | Path | `e_pitch`'s rate, low-passed at `pitch_kd_rate` |

Two pitch terms close no error and read the attitude: the pitch trim (the Euler
pitch, the EAS and the throttle) and the turn's own pitch rate (`pitch_turn_ff`,
off by default: `r tan(roll)` with `r` the coordinated-turn yaw rate from the
Euler roll and pitch).

### Yaw

| Term | Measured from | Quantity |
|---|---|---|
| Coordination (`yaw_coordination_ff`; off by default) | Path and attitude | not an error: the turning yaw from the Euler roll and pitch and the true airspeed; the rolling yaw from the commanded roll rate and the path's angle below the nose |
| Pointing P and integrator (`yaw_point_kp`, `yaw_point_ki`; off by default) | Path | `pan_err`, the error bank-to-turn closes; the integrator runs only while the path is known; off while the Curve Flyer owns the chase |
| Pointing steep schedule (`yaw_point_pitch_sched`; off by default) | Path, against gravity | the roll blend's `path_share` |

### Scoring and display (companion)

| Check | Measured from | Quantity |
|---|---|---|
| `alignment_errors()`: the `p` verdict, `in_tol`, the align-seconds counter, HUD Path Aligned, the sweep's align gate | Path | the companion's own balloon geometry against the FC's velocity, undefined at or below `FLIGHT_PATH_MIN_SPEED_MPS` |
| HUD Nose Aligned | Nose | the balloon's bearing from the boresight |

Logged, not flown, from the nose: `lua_plane_off_nose_deg`, `lua_plane_axis_up`,
and the flight-path markers `lua_vel_marker` / `lua_lat_marker` (the path's
angles off the nose).

### A worked example: where path and nose part

A near-vertical climb: the path at 89°, the nose 7° below it at 82° (so the
path sits 7° toward the canopy from the nose), and the balloon 1° to the side of
the path and 0.7° beyond it toward the canopy.

- `path_share` is 1: the path is past Steep Full, so roll-to-plane has the
  whole share and bank-to-turn none.
- Round the nose the clock reads about 8°: the roll roll-to-plane flies,
  centring the balloon over the nose. Round the path it would read about 56°,
  the roll that brings the balloon over the path. Rolling the 8° about the nose
  leaves the balloon still about 0.9° to the side of where a pull takes the
  path, because the roll turns the 7° between nose and path into sideslip.
- `pan_err` reads about 1°. On a path under Steep Full bank-to-turn keeps
  `1 - path_share` of the command, and its bank proxy cannot exceed
  `asin(cos(pitch))`, 8° at 82° nose-up, so a bank request past that rolls
  until `pan_err` is gone (§ "Bank-to-turn").
- Halfway between the nose and the path instead, 1° to the right, the two
  clocks part in direction: about 16° round the nose, a roll right toward the
  balloon's side, and about 164° round the path, nearer 6, a roll left
  (§ "Roll-to-plane": why the nose's is the one flown).

The columns that show it side by side are `lua_pan_err_deg` against
`lua_plane_err_deg`.

---

## Lift Vector Pointing: the second steering law

*The rules - one law for every mode (C-03), the FC owns it, every value it
computes logged - are `CLAUDE.md`'s. This is the mechanism, in
`LUA/path_attitude.lua`. The name says what it does: it points the wing's lift
vector at the push the flight path's turn needs.*

### One switch for every mode

`steering_law` (the Gain Tuner's Steering Law, default `STEERING_LAW`) picks the
law that steers every mode at once: 0 the cascade (the roll, yaw and pitch laws
of the sections above), 1 Lift Vector Pointing. Modes still differ only in the
world vector they steer toward. The switch is part of the FC's chase key, so a
change of law is a fresh chase: each law starts from its own reset, the Curve
Flyer drops any PLAN, and neither law carries a memory of the other. Only the
selected law runs. Under Lift Vector Pointing the cascade is reset every tick
and its internals go out blank; under the cascade, Lift Vector Pointing's are
blank. STATE's `cs_flags` bit 6 says Lift Vector Pointing flew; with it set the
cascade's flag bits mean nothing.

The settings both laws read sit in the Gain Tuner's first section, BOTH LAWS:
the switch, Bank Lim (`bank_lim_deg`) and the four pitch trim terms. Every row in
a law's own section is read by that law alone.

The measurement runs excite either law's output alike: freq-ID's excitation
adds to the axis it names, plant-ID's replaces it, and the replaced axis's
integrator, which then reaches nothing, is cleared.

### Three questions, in the flight path's frame

The law steers the frame `track.lua` builds along the flight path every tick
(§ "The path frame"): x' along the velocity, y' the right wing kept square to the
belly-canopy axis, z' the belly. Every quantity it closes is a component in that
frame, so a balloon overhead, underneath or behind is no special case and no
Euler angle enters the law. Each tick answers three questions in order.

1. **Guidance: where should the path go, and how fast should it turn?** theta
   is the path's angle off the balloon, read from where the path will point
   `att_lead_s` from now. The path's acceleration across itself - the specific
   force plus gravity - over the speed is the rate it is turning at, and the
   accelerometer sees it as soon as the wing loads, before the velocity has
   turned; with no accelerometer reading the path is read as it is. u is the
   direction across the path toward the balloon. The path is asked to turn
   toward u at

       Omega = min(att_k_path * theta, att_path_rate_max_dps)

2. **The force line: what push turns it that way?**

       f = V * Omega * u - g_perp          (per unit mass, across the path)

   The turn the path needs, less what gravity across the path already does. Its
   direction is the lift line to aim at, its size the load.

3. **The attitude: how must the aircraft rotate to make that push?** The wing
   pushes only along its lift line, either way: out of the canopy (a pull) or
   out of the belly (a push). The roll about the path turns the end of the lift
   line on f's side of the wing line (§ "The lift side") to face f, at
   `att_k_roll` per radian still to go; the pitch turns the path at Omega's
   share in the lift plane, which the wing makes by pulling or pushing; and the
   path frame turns about z' with gravity's sideways part, g_y / V.

On a shallow path gravity is most of the push, so the lift line stays near
canopy-up and a modest bank turns the aircraft. Near vertical gravity does almost
nothing across the path, and the lift line rolls round to face the balloon. One
formula covers both.

A balloon dead astern has no direction across the path, and the turn goes toward
the canopy: up and over.

### The lift side

The wing pushes along its lift line either way, so the side is the one of the
wing line f lies on: f on the canopy's side is a pull, f on the belly's side a
push. A push the room (below: the band and upside down) stops from facing f is
a pull instead, held to the room like any aim: a push it cut short would turn
the path off the turn.

The side changes only where f crosses the wing line, or where the belly's
facing roll enters or leaves the room as f moves, and the law's own roll brings
neither about: rolling the lift line toward f turns the wing line away from it,
and which banks the room allows is set by the path, not by the roll.

Choosing instead the side whose push fits f better once the room has cut each
side's roll makes a tie the law steers itself onto. Between the room's two ends
lies a range of f no roll faces; there each cut push bends the path so that f
moves toward the middle of that range, where the two sides fit equally, and the
side switches on every tick it can.

### The bank limit, on the cascade's bank proxy

The bank limit holds in every chase, on the bank as the cascade measures it: the
bank proxy asin(sin(roll) cos(pitch)), the wing tip's angle below the horizon
(world down along the body's wing axis, `track.lua`'s `down_by`). Level it is
the bank. The steeper the path, the more roll it takes to dip a wing tip that
far, and with the path no further than the limit from vertical no roll about the
path does, so there the roll is free. Upside down with the wings level reads as
no bank. One measure under both laws makes Bank Lim mean one thing whichever law
flies, and lets `bank_proxy_deg` judge either.

On the chase's first tick the band is

    [min(-bank_lim_deg, p0), max(bank_lim_deg, p0)]

with p0 the proxy the chase starts at, so a start banked beyond the limit may
roll back toward it but never further out.

A roll about the path that takes the bank round the path to t leaves the proxy's
sine at c + k sin(t + d): c from the wing axis's part along the path, which no
such roll changes, and k and d from its part across. The band therefore forbids
at most two arcs of bank, one round the right wing straight down and one round
the left. `band_room()` works out how far the roll may go either way before the
wing tip enters one; `bank_room()` cuts that room further to upside down (below),
and the roll still to go is held to the result. A turn that wants more bank than
the limit aims at the limit itself, and cannot roll on through the wing tip's
lowest point to the far side; banked inside an arc already, the roll may only
leave it, by its nearer end. The law aims at the limit, not inside it: the roll
can carry a few degrees past before it settles, and an operator who wants less
lowers Bank Lim.

While the room holds the bank - the roll still to go cut to it - the roll also
cancels the bank's drift. The
bank round the path moves as the path turns, with no roll about it at all:

    d(bank)/dt = wx - up_x * (up_y * wy + up_z * wz) / um^2

with world up in the path frame (up_x along the path, um its length across) and
up_x / um, the tangent of the path's angle to the horizon, capped at
tan(90 - `CURVE_POLE_FADE_DEG`). The steeper the path, the larger that drift's
share of the roll.

### Upside down

The room is cut further to upside down, as the rules of a chase define it
(CLAUDE.md): the law never rolls the aircraft upside down, and one that is may
only roll back. The rule is read on the aircraft's own belly-canopy axis, not
on the bank. With pz the path's part along the body's belly axis and zz that
axis's part along z', the canopy axis (-z of the body) is -pz x' - zz z' in the
path frame (y' is square to it). After a roll that takes the bank round the path
to t, world down along it is

    pz * up_x - zz * um * cos(t)

and only its part across the path turns with the roll. Where that is positive
the axis meets the ground on the canopy side, so the rule leaves the banks with
cos(t) >= q, q = pz * up_x / (zz * um): within h = acos(q) of up either way,
every bank for q <= -1, and up alone for q >= 1. A bank already past h may only
roll back toward up. With the nose on the path h is 90 deg. Pushing in a dive
the nose sits below the path and h is less: with the wings square to up across
the path, the canopy axis tips toward the ground. Within `CURVE_POLE_FADE_DEG`
of vertical the rule cuts nothing: there the axis is near level and the bank
has no meaning.

The rules of a chase grade by this item, and the law also steers by it, in
every chase, with or without a PLAN. `upside_room()` makes the cut.

### The sideways weighting

The turn's sideways part, along the horizontal across the path, weighs
`att_k_side` times its share, so the heading turns while the path is still
shallow and a bank still turns it. 1 is no weighting. It does not act within
`CURVE_POLE_FADE_DEG` of vertical, where the horizontal across the path has no
direction.

### The integrators

A roll integrator on the roll still to go and a pitch integrator on the path's
error in the lift plane (`att_roll_ki`, `att_pitch_ki`, 1/s) run only within
`att_i_zone_deg` of the target, where what is left is trim and the propeller's
torque; wound through the turn they overshoot. Each is held within its ACRO
rate, past which nothing it stores is flown. Outside the zone, at a gain of 0,
on a fresh chase and on any tick the law's output is not flown, they are
cleared (§ "Inactive integrators are cleared").

### Into body axes, and the rudder

The three path-frame rates are rotated into body roll and pitch rates, then cut
at `ACRO_ROLL_RATE` and `ACRO_PITCH_RATE`, as ArduPlane cuts them. Each command
goes out as the law works it out, with no ramp of its own between it and the
rate loop; a jumping command meets the rate loop's own slew limiter
(`PTCH_RATE_SMAX`, `RLL_RATE_SMAX`), which cuts the loop's P and D while it
acts. Nothing is added to the pitch rate after the law: it is the path frame's
whole turn in body axes, so the cascade's turn pitch rate (`pitch_turn_ff`) is
not added and `lua_pitch_turn_dps` is blank.

Version B commands no yaw. The FC calls `set_rudder_offset(0, false)`, which
keeps ArduPlane's yaw rate controller off and the rudder at zero (CLAUDE.md
"ArduPilot binding gotchas", gate 3). The fin keeps the aircraft straight, and a
roll about the nose swings the path round it, which the law reads as path error
and steers out like any other. The yaw law's integrator reaches nothing and is
cleared.

### The pitch trim under this law

The pitch trim adds the same four terms under either law; only its lift term
differs. Under the cascade it is cos(pitch). Under Lift Vector Pointing it is the
load the law asks of the wing: the force line's, -f_z / g, or on a tick the
Curve Flyer owns the chase the load of the rates actually flown
(`path_attitude.load_of`), since the PLAN's bank, or rates held at zero, do not
face the force line. That is 1/cos(bank) in a level turn, negative in a push and
zero straight up, where cos(pitch) is right only in straight, upright flight.
The FC logs the term it used (`lua_pitch_trim_lift_term`) and the trim map fits
against the logged term, so the lift gain means the same either way.

### Its plan mode

While the Curve Flyer runs a PLAN it hands the law the target to turn the path
toward and the bank to hold, measured round the path as this law measures it
(§ "The Curve Planner and the Curve Flyer"). The guidance and the pitch are the
law's own. The roll goes to the PLAN's bank instead of facing the force line,
held to the band and to upside down as any target is, so no lift side is chosen
(`lua_att_side` blank). The sideways weighting is off, since it serves the
law's own choice of bank. The roll cancels the bank's drift on every tick, and the roll, with the
roll integrator's input, fades over the last `CURVE_POLE_FADE_DEG` before
vertical to nothing there, where the bank cannot be measured.

### The log

The law's internals travel on STATE and STATE2 into the `lua_att_*` columns, its
gains into the `g_att_*` columns and the switch into `g_steering_law`. Every
internal is NaN, and so blank, on a tick the law did not run.

| Column | What it carries |
|---|---|
| `lua_att_theta_deg`, `lua_att_theta_now_deg` | the path's angle off the target: from the led path, and as read |
| `lua_att_u_y`, `lua_att_u_z` | u, the turn's direction across the path |
| `lua_att_omega_dps` | Omega, the turn rate asked of the path |
| `lua_att_f_y_mps2`, `lua_att_f_z_mps2` | the force line f |
| `lua_att_mu_deg` | the roll still to go for the lift side's end of the lift line to face f, held to the room; under a PLAN, to the PLAN's bank |
| `lua_att_side` | the lift side: 1 canopy, -1 belly; blank under a PLAN |
| `lua_att_w_f` | the roll's weight under a PLAN, its fade near vertical; blank without one |
| `lua_att_bank_deg` | the bank round the path |
| `lua_att_band_lo_deg`, `lua_att_band_hi_deg` | the chase's band, on the bank proxy |
| `lua_att_room_lo_deg`, `lua_att_room_hi_deg` | how far the roll may go each way before the proxy leaves the band or the aircraft is upside down; -180 and 180 where nothing limits it |
| `lua_att_drift_dps` | the bank drift the roll cancelled, while the room held the bank and under a PLAN; blank where it did not |
| `lua_att_wx_dps`, `lua_att_wy_dps`, `lua_att_wz_dps` | the three path-frame rates |
| `lua_att_p_dps`, `lua_att_q_dps` | the body rates before ArduPlane's cut |
| `lua_att_roll_i_dps`, `lua_att_pitch_i_dps` | the two integrators |
| `lua_att_n_lift` | the load the force line asks of the wing, g |

`bank_proxy_deg` is filled under both laws, so the bank limit is read from one
column whichever flew; `sideslip_deg` and `aoa_deg` come from whichever law ran;
the lead reads `lua_accel_x`, `lua_accel_y` and `lua_accel_z`.

### Checking it

- **`LUA/test_plant.lua`** is a six-degree-of-freedom model of the simulated
  Mini Talon for flying `balloon_chaser_fc.lua` closed loop on the desktop. The
  airframe is `Arduplane/SDF/model.sdf`'s, read at load: every lift-drag element
  on `base_link`, the masses and inertia, and the world's elevation for the air.
  Each element's force is `AltitudeLiftDrag`'s with the stall left out, the
  propeller is a thrust line through SITL's climb and dive speeds, and the rate
  loop is ArduPlane's - AC_PID on the rate error times the speed scaler squared,
  the FLTT, FLTE and FLTD filters and the SMAX slew limiter, on the stored Acro
  set - feeding the V-tail mixer with the rudder at zero. Two of its numbers are
  fitted to SITL rather than read from the SDF: `MIXING_GAIN`, the V-tail
  mixer's gain, so the model's trim elevator matches the SITL trim map's, and
  `AIL_GAIN`, the aileron's effectiveness, so its roll-rate feedforward matches
  the `RLL_RATE_FF` ArduPlane AUTOTUNE finds in SITL. It is not SITL: a pass
  there is evidence about this model.
- **`test_balloon_chaser_fc.lua` § 17** checks the law's pieces: selection,
  straight flying the same law, the logging, no yaw sent and the rudder held at
  zero, the pitch trim's lift term under each law, the ACRO limits, the
  integrators, the lead, no allocation per tick, and the band and its room on
  the proxy - level, nose 30 deg up, nose 50 deg up and banked past the band
  mid-chase. It checks the lift side - wings level, a pull with f on the
  canopy's side of the wing line and a push on the belly's side where the belly
  faces it, a pull instead of a push the room stops, and a pull when banked to
  the limit with f below the horizon on the canopy's side - the turn the room
  allows, and upside down: in a dive the canopy axis meets the ground before
  90 deg of bank pushing and after it pulling, and banked 120 deg on a steep
  path the roll may only come back toward up.
- **§ 18** flies the law closed loop on `test_plant.lua`'s airframe over the
  whole sphere of balloon directions, wings level and banked, and judges every
  chase by the rules of a chase. It is strict and reports each failing chase by
  where its balloon started. It also flies a balloon right and below whose force
  line, from level, lies between the band's two ends, and checks that it keeps
  its lift side (at most one switch) and lines up.

---

## The Curve Planner and the Curve Flyer

*The rules — the PLAN is the one control-law output the wire carries; a fresh
chase holds for it; each law's plan mode is one law in two languages — live in
`CLAUDE.md` § "Lua steering offload", and the rules of the curve itself in
§ "The rules of a chase". This is the mechanism.*

A chase is a turn and then a straight line to the balloon. Either steering law
steers on the error it sees each tick, which keeps no view of the shape of the
whole turn, and the rules of a chase are about exactly that shape: how far it
banks, whether its tilt comes back, whether its heading wanders once it has
turned. So the turn is PLANNED. The Curve Planner (`Companion/curve_planner.py`)
works out the turn on the companion as a bank schedule; the Curve Flyer
(`LUA/curve_flyer.lua`) runs the schedule on the FC every tick and hands the
selected law what to fly; and the law flies it closed loop in its PLAN MODE,
with its own roll, pitch, rudder and bank limit. At the end of the turn the law
flies on without the PLAN, or, with Transfer Control When Done off, the Curve
Flyer aims it at the line the turn ended on. They fly `p` and `x` chases only:
straight and manual fly the selected law from their first tick. The plan is a
numeric plan of the turn, not a control law, so C-03 binds the law that flies
it, which straight shares.

### What a PLAN is

Not a route and not a stick schedule: a BANK SCHEDULE. `CURVE_N_KNOTS` bank
angles, knot k at PROGRESS (k-1)/(`CURVE_N_KNOTS`-1), where

    progress s = 1 - eps / eps0        (clipped to 0..1)

with eps the flight path's angle off the turn's target now and eps0 the PLAN's,
its angle at the start; between knots the bank is linear in progress. The banks
are in the measure of the law the PLAN was made for: the bank proxy under the
cascade, the bank round the path under Lift Vector Pointing. The PLAN frame
(`protocol.PLAN`) carries the knots, eps0, the law, a `plan_id`, and for a chase
with a run-in the run-in's climb and depression (NaN without one), nothing else;
`test_no_micromanagement.py` pins its fields. The FC takes a PLAN only while a
fresh chase waits for one, and only one made for the law it has selected: the
other law's banks are in another measure.

Progress, not time, indexes the schedule. An aircraft that turns slower than the
planner expected reaches each knot later, and still meets it where the turn has
got to - a time-indexed schedule would hand it the bank meant for a point it has
not reached. The schedule is therefore robust to the one thing the model gets
most wrong: how fast the real aircraft turns.

The FC works out no rule of a chase. Each law holds the bank limit in every
chase on its own - the cascade clips its roll setpoint to `bank_lim_deg` on the
proxy, Lift Vector Pointing holds its band - and the Curve Planner decides which
rules bind and plans within them.

### The Curve Flyer: running the PLAN

The Curve Flyer runs before the law each tick and commands no rate of its own.
It reads the path frame `track.lua` built this tick - the balloon
`(t'x, t'y, t'z)`, its elevation, the path's world direction, and any line it
aims at (`hold_path_x/y/z`) - and leaves the law two things: the TARGET to turn
the path toward, which the FC turns into body axes, and the BANK to hold.

| State (`cf_state`) | What the law is handed |
|---|---|
| OFF (0) | nothing: the Curve Flyer is off, or the chase is not `p` or `x` |
| WAIT (1) | nothing; the rates are held at zero |
| FLY (2) | the turn's target - the balloon, or a run-in's line - and the schedule's bank at this tick's progress |
| DONE (3) | nothing: the turn is complete, and the law flies as without a PLAN |
| TIMEOUT (4) | nothing: no PLAN arrived within the Plan Wait |
| HOLD (6) | the line the path had when the turn completed, and the bank toward it |
| RUNIN (7) | the run-in line, and the bank toward it |
| DIVE (8) | the balloon, and the bank toward it |

The numbers are the ones STATE2 carries; 5 is unused, so each value means the
same in every log. One tick may pass from the turn to the run-in to the dive, as
the conditions for each hold.

The bank toward a target lying `ty` across the wings (y', + right) is

    b_toward = clamp(CF_HOLD_BANK_PER_DEG * asin(ty), +-bank_lim_deg)

level on the target and banked toward it off it, so a drift across the wings -
which wings-level flight could not correct - is turned back rather than kept.

On the tick the target changes between the balloon and a line, the Curve Flyer
sets `retarget`, and the cascade's pitch derivative re-seeds instead of
differentiating across the jump; the planner's model does the same.

No clock ends the turn, a run-in or its dive: each runs until its own condition
ends it, however long that takes. The one timed state is WAIT.

### Each law's plan mode

Both laws take the Curve Flyer's target and bank and fly them with the gains
they fly without a PLAN:

- **The cascade.** The PLAN's bank, clipped to `bank_lim_deg` either way, is the
  roll setpoint against the bank proxy, turned into a rate by `roll_gain` as
  always. Bank-to-turn does not run - its integrator is cleared and its
  derivative re-seeds when it flies again - and roll-to-plane has no share. The
  pitch law closes on the target's pitch error with its own P, I and D, and the
  turn's own body pitch rate (`pitch_turn_ff`) is added after it. Yaw pointing,
  which steers on bank-to-turn's error, is off and its integrator cleared; the
  yaw coordination stays, so the roll turns about the path.
- **Lift Vector Pointing.** § "Lift Vector Pointing" → "Its plan mode": the
  guidance and pitch its own, the roll to the PLAN's bank round the path held
  to the band and to upside down, no lift side and no sideways
  weighting, the bank's drift cancelled every tick, and the roll faded near
  vertical.

`cs_flags` bit 5 says a PLAN set the bank, under either law. The pitch trim and
ArduPlane's ceilings apply after the law, as they do without a PLAN.

### A chase, tick by tick

1. **Hold.** With `curve_flyer_enable` on, a fresh `p` or `x` chase enters WAIT:
   roll and pitch rates held at zero, so the aircraft flies on in the state the
   plan is being made from. The law still runs, and is logged as what it would
   have flown; its output is not flown, so its integrators and derivative memory
   are cleared every such tick and it starts fresh when the PLAN arrives.
2. **Plan.** The companion sees `lua_cf_state` = WAIT and has the planner
   process (below) plan for the selected law, with the gains that law flies,
   from STATE2's attitude and velocity, the balloon direction this tick's GUIDE
   carries (with its range under `p`), the relative altitude and the operator's
   settings. It sends the PLAN every tick until STATE2's `cf_plan_id` echoes it:
   a lost frame is not re-sent by anything else.
3. **Fly.** The FC takes the PLAN (FLY) and the law flies the turn until the
   path points within `CURVE_TURN_DONE_DEG` of the turn's target. A PLAN with a
   run-in goes on to RUNIN and DIVE (below); one without completes there.
4. **Hand over, or hold the line.** With Transfer Control When Done
   (`cf_transfer_when_done`) on, the law flies the chase from there as it flies
   one without a PLAN (DONE). Off, the Curve Flyer enters HOLD (below) until the
   chase ends, or until the switch goes on, which hands over on the next tick;
   turning it off after the hand-over takes nothing back.

With no PLAN within the Plan Wait (`cf_plan_wait_s`, a Gain Tuner setting; the
FC does not wait at all until it has received one) the law flies without one
(TIMEOUT). Tracking off, or any fresh chase - a change of mode, of FC mode or
of steering law - ends the chase and drops its PLAN and any held line on that
tick, whatever Transfer Control When Done says: tracking off releases control,
a fresh `p` or `x` chase waits for a new PLAN, and any other flies the law
without one.
The companion drops its plan on the same change, so the next chase - `p` to `x`
included - waits for and plans its own turn from scratch. Straight, manual and
the measurement and tuning runs never enter WAIT.

### The run-in and the dive

The Approach Angle (`cf_approach_deg`, default `CF_APPROACH_DEG`) is the dive
the turn of a chase ends on, for a balloon below the horizon. At its default of
0 no balloon is shallower than it, so no chase runs in. One that starts
below the horizon but less far below than the Approach Angle is flown with a
RUN-IN, unless the planner's rule decision puts it under rule 2:

- the scheduled turn (the LINE-UP) aims at the RUN-IN LINE instead of the
  balloon: the PLAN's run-in climb above the horizon, toward the balloon's
  compass direction, worked out afresh every tick (`CF.line`), so it follows
  the balloon as the aircraft moves;
- with the path within `CURVE_TURN_DONE_DEG` of the line, RUNIN holds it, as a
  hold holds its line, until the balloon sits the PLAN's run-in depression below
  the horizon;
- DIVE then aims the law at the balloon, banked toward it; within
  `CURVE_TURN_DONE_DEG` of the balloon the turn is complete, and the chase is
  handed over or held as any other.

The run-in ends on the balloon's angle below the horizon, not on time, so a
slower aircraft or a nearer balloon than planned still dives where the line to
the balloon is as steep as planned. The run-in climb is level, or where rule 1
binds the starting climb if that is a dive: levelling it would bring the tilt
back. The planner needs the balloon's range for it; with none known (`x`) it
plans with the balloon on the ground along its line of sight, the farthest it
can be, and a nearer balloon ends steeper.

### Holding the line

With Transfer Control When Done off, the turn's completion captures the flight
path's world direction at that tick (`track.lua`'s `path_n/e/d`) as the line to
hold, and the law flies its plan mode aimed at it: `track.lua` reads the line in
the path frame each tick (`hold_path_x/y/z`) as it reads the balloon, the law's
pitch turns the path toward it, and the bank is `b_toward` of the line's
component across the wings. On the line the wings are level, so the path does
not turn; a drift off it across the wings is banked back toward. The balloon is
no longer steered toward. The turn's own end still carries the path off the line
for a moment, since its bank and pitch rate do not stop on one tick, and the
hold then brings it back.

The Curve Planner's `SETTLE_S` window flies the same hold when the switch is
off, so its after-turn check judges the flight the FC will make.

### How the Curve Planner chooses

The planner flies every candidate schedule through `fly()`: the Curve Flyer's
states and a model of the selected law's plan mode (`PlanMode`, the cascade's or
Lift Vector Pointing's, with the gains the law flies: `PLAN_GAINS`), on a
point-mass aircraft. Its nose is on its flight path; the roll rate about the
path and the pitch rate turning the path toward the canopy follow their
commands after first-order lags; gravity across the wings turns the path
sideways as it does in any banked flight; and the speed follows a fit to SITL
(a climb costs speed, a dive gains it). Each plan mode is the same law as the
Lua, from the same constants and gains (CLAUDE.md: one law in two languages).

The schedule chosen completes the turn soonest, subject to, as penalties in the
cost (`PENALTY_PER_UNIT` seconds per degree, metre or m/s broken):

- above `ALT_MIN_M` and faster than `SPEED_MIN_MPS` throughout;
- for `SETTLE_S` after the turn, the path still within `CURVE_TURN_DONE_DEG` of
  the balloon - the turn arrives, it does not swing through;
- where rule 1 or 2 binds (`rule_binds()`, below), no Suboptimal Curve: the
  bank proxy within the band (the bank limit either way, or the starting proxy
  where that is beyond it), the canopy never further below the horizon than at
  the start, the tilt never coming back by more than the Tilt Back Travel, the
  heading within `CURVE_TURN_DONE_DEG` of the turn end's for `SETTLE_S` after
  it (weighed by the path's distance from vertical), and the path no nearer
  vertical than `CURVE_POLE_FADE_DEG`, where the bank cannot be held. The proxy
  reads level wings upside down as no bank, so being upside down is a penalty
  of its own. A start upside down rolls the right way up through whatever bank
  that takes: its band holds from its first tick the right way up, widened to
  the proxy there, as a banked start's is;
- with a run-in, `APPROACH_PENALTY_PER_DEG` per degree the line from the
  aircraft to the balloon ends off the Approach Angle: more than any turn
  takes, so the plan meets it wherever it can, and less than a rule's penalty,
  so where it cannot meet it without breaking a rule the rule wins.

Among turns equally soon, the one that rolls least wins
(`ROLL_TRAVEL_S_PER_DEG`, a tie-breaker worth a hundredth of a second per
100 deg of roll). A candidate is flown at most `TURN_HORIZON_S` ahead (a run-in
across the whole distance to the balloon besides): a turn not complete by then
is scored as one that never completes: its cost is the flight's cap (the
horizon, or lower once a batch has a finished plan) plus a tenth of a second per
degree of its closest miss, with a run-in the Approach Angle penalty for
`APPROACH_MISS_MAX_DEG` besides, and `NEVER_DONE_S` on top, so it costs more
than any turn that completes, whatever rule that turn breaks: a missed balloon
is the worst result. That bounds only the search; the FC flies a PLAN until its turn completes.

**The rule decision is the planner's, and is narrower than CLAUDE.md's rule.**
`rule_binds()` puts a chase under rule 1 when the balloon's direction across the
path lies within `RULE_PLANE_DEG`, round the path, of world down's, and under
rule 2 within `RULE_PLANE_DEG` of world up's; in either case less than 90 deg
off the path, and not through the pole (the turn's great circle passing within
`RULE_POLE_DEG` of straight down or up before it reaches the balloon). Every
other chase is planned for the soonest turn without the Suboptimal Curve
penalties. CLAUDE.md's rule covers
every chase but one whose balloon starts behind, so outside those two slices a
plan is not held to it. The same decision sets the run-in climb and keeps rule 2
chases from a run-in.

Only the turn is planned. After it the rules still bind - the heading, the tilt
- but they bind whatever flies the chase. With Transfer Control When Done on,
the planner's `SETTLE_S` window keeps the law's plan mode aimed at the balloon,
as a stand-in for the law without a PLAN; off, it flies the hold the FC flies.

### The search

The cost of a turn is not convex: it has a few separate valleys - turning left
or right, rolling far over, and under a bank limit rolling one way and then the
other - and a search that only runs downhill keeps whichever it starts in. But
inside a valley it is nearly one-dimensional: the best schedule is close to one
steady bank, with the rest of the schedule shaping it. `plan_turn()` searches
the knots in two stages to match, within `knot_range()`: the bank limit either
way for the cascade, which clips its setpoint there, and every bank round the
path for Lift Vector Pointing, whose band holds the limit on the proxy wherever
the path is.

1. **Find the valleys.** One batch flies every bank in the range,
   `PLAN_SCAN_STEP_DEG` apart, both held from the start and reached by a ramp
   from the starting bank. Where a rule binds it also flies every reversal: one
   bank to the turn's midpoint and another after it, `PLAN_SWITCH_STEP_DEG`
   apart. The scan only ranks valleys, so it flies at the coarser step
   `PLAN_SCAN_DT_S`. The best plan of each of the `PLAN_VALLEYS` best valleys,
   at least `PLAN_VALLEY_SEP_DEG` of bank apart, goes on.
2. **Shape the schedule** (`polish()`). `PLAN_REFINE_ROUNDS` batches at the
   planner's own step: each knot of every plan stepped up and down, and
   `PLAN_REFINE_RANDOM` random nudges of the same size; a plan takes the best
   that beats it, and its steps halve when none does.

For a chase with a run-in, `plan_dive()` then searches the run-in's depression
on the line-up `plan_turn()` chose: every depression from the balloon's starting
one to the Approach Angle, `PLAN_DEP_STEP_DEG` apart, polished as a turn is. The
line-up and the run-in are the same for every such plan, so they are flown once
and each dive is flown on from where its run-in ends (`dive_starts()`); while
every plan in a batch is settled on its run-in line, the model takes the longer
step `DT_RUNIN_S`.

A batch costs about the same however many plans it flies, because the time goes
to the Python step loop rather than the arithmetic, so the search is a few wide
batches rather than many narrow ones. Each batch also stops early: once a plan's
turn and its `SETTLE_S` after it are flown its cost is known, and a plan still
turning past that cost cannot beat it, since a plan costs at least its turn
time (`fly()`, every `CAP_EVERY_STEPS` steps). Across batches the same holds for
the best plan so far, by its cost and never by the soonest turn's time: where a
rule binds, the soonest turn can carry a penalty, and a plan still turning then
can yet complete within the rules at a lower cost. A batch flies a step past
its cap and `SETTLE_S`, so a plan whose turn completes at the cap has its
`SETTLE_S` after the turn checked whole.

### The planner process

The planner runs in a process of its own, `curve_planner.PlannerProcess`:
`python -m Companion.curve_planner --serve`, started the first time the Curve
Flyer is on, taking pickled requests on its stdin and answering on its stdout,
one request at a time. It ends when its stdin closes, which the companion's exit
does by itself. It is a separate process rather than a thread because the search
holds the interpreter lock almost throughout: on a thread it slows the 50 Hz
control loop to a fraction of its rate for as long as a plan takes, and the plan
itself by the same contention. It is started with `-m` rather than through
`multiprocessing` because `multiprocessing` re-imports `main.py`, whose
top-level code opens a new console log.

The PLAN goes out the moment it is made. The finer check of its turn time
(`check()`, at `DT_CHECK_S`) runs afterwards, for the console's
`[curve] plan N: ... planned in X s` line only. A planner that fails logs why and
sends nothing; the FC flies the law without a PLAN when the Plan Wait runs out.

### Checking it

- **`test_balloon_chaser_fc.lua` § 16** flies the Curve Planner's PLANs through
  the real FC script under each law, on two point-mass aircraft. On the
  planner's own aircraft - its pitch-rate lag and SITL speed fit - every turn
  takes what the planner predicts, within `TURN_AGREE_S`: a law that drifts from
  the planner's model of its plan mode fails there. On section 15's, which lags
  the load and holds its speed, each turn still completes closed loop. The
  other checks run under both laws too: the wait, the echo, the law saying it
  flies its plan mode (`cs_flags` bit 5) and flying on without the PLAN after
  the turn, a PLAN made for the other law not taken, the bank held to the limit
  when the schedule asks for more, the timeout and tracking off; with Transfer
  Control When Done off, the line held and the path back on it after the turn's
  end, a slower aircraft's hold settling onto the line, the switch turned on
  mid-hold handing over, a change to straight handing over at once and dropping
  the plan, straight never using the Curve Flyer, and `p` to `x` starting a
  fresh wait; a nose wobbling in a sideslip the path does not follow, and
  `track.lua`'s turn rate against the turn flown; the two run-ins, and a run-in
  with Transfer Control When Done off; and a turn that can never complete still
  flown at twice a chase's length, since no clock ends it. The Curve Flyer off
  is checked once, under the default law. The pitch trim
  is off there: those aircraft turn at exactly the rate commanded and need no
  elevator to hold their nose, so a trim would be a rate error.
- **`Planning/sweep_plans.py`** plans the default `p` balloon and the rule chases
  from level and from 60 deg of climb or dive, wings level or banked 30 deg right
  or 60 deg left, under the law it is given, and checks each plan at a finer
  step than it was planned at.

### What it costs

The hold is time the aircraft does not turn: as long as the Curve Planner takes,
about half a second for a chase on a desktop x86 under either law (the example
chase `python -m Companion.curve_planner` times under each), and several times
that on an embedded ARM companion. The competition judges how soon the
path aligns, so the planning time is the Curve Flyer's main cost. Nearly all of
it is the Python step loop; the arithmetic of a plan, compiled, would take a few
tens of milliseconds.

---

## The two autotune subsystems

*The rule — never conflate them, use these names, and do not rename ArduPlane's
own identifiers — lives in `CLAUDE.md` § "Two autotune subsystems". This is what
each one is.*

There are two unrelated "autotune" concepts in this repo, plus a third
measurement-only subsystem (Plant-ID).

**Plant-ID** (`i` key, `plant_id.py`) tunes nothing. It identifies the airframe's
RATE plants one axis at a time — pitch (elevator), yaw (rudder), roll (aileron) —
by exciting each surface with an open-loop MANUAL doublet, the FC rate loop out of
the way, and fitting the short-period response (see § "plant-ID design"). All three
axes share ArduPlane's `AC_PID` rate loop (`AP_FW_Controller` for pitch/roll, the
math-identical `AP_YawController::get_rate_out` for yaw), so each maps its fit to
the full rate PID (`<AXIS>_RATE_{FF,P,I,D,IMAX,FLTT,FLTE,FLTD,SMAX}`) plus the
physical overshoot floor, written to the run's single report: FF/I/IMAX/filters
from AUTOTUNE's formulas, P and D computed from the plant via one aggressiveness knob (`YAW_RATE_*` apply only with `YAW_RATE_ENABLE=1`). A human reviews and loads
them; nothing is auto-applied and no stored gain is touched. The three axes are gated by the Do Pitch / Do Yaw / Do Roll switches and
sequenced with a top-level checkpoint (`incomplete_plant_id_run_*.json`) so a
second `i` resumes at the next unfinished axis. The whole run emits just three
files — one report, one signals CSV, one checkpoint — see "Tuning-run outputs". Altitude protection is symmetric with the sweep's: climb
to `AUTOTUNE_PROTECT_CLIMB_TO_M` ONLY when AGL falls below `AUTOTUNE_PROTECT_FLOOR_M`,
and between doublets a verified FBWA wings-level settle removes any residual bank
the next open-loop excitation would otherwise ride. `i` does not measure the
pan authority (the turn a degree of bank buys): that is the `k` frequency sweep,
which also recovers phase. SIM ONLY until validated on hardware.

- **Arduplane Autotune** — commands the **autopilot's** built-in AUTOTUNE
  (ArduPlane's RATE-PID tuning) over MAVLink. File: `arduplane_autotune.py`
  (entry point `run_arduplane_autotune`), launched as a maintenance mode from
  `main.py` via `--arduplane-autotune` (`--autotune` is an alias). Identifiers like `AUTOTUNE_LEVEL`, `AUTOTUNE_AXES`, and mode
  `AUTOTUNE` inside that file are ArduPlane's real param/mode names.
- **Controller Autotune** — *our* controller-gain tuning, in two halves:
  - *Manual* — the interactive gain-tuner window: `ui/controller_autotune.py`
    (`class ControllerAutotune`) + `controller_autotune_store.py`
    (`class ControllerAutotuneStore`). Reached through the tuner window/keyboard.
  - *Automatic* — the gain sweep: `controller_autotune_sweep.py`
    (`run_controller_autotune_sweep`), started with the `'t'` key during a sim
    run. `SWEEP_PLAN` holds the cascade's gains, so the sweep tunes the
    cascade and means something only with Steering Law 0; Lift Vector
    Pointing's `att_*` gains have no sweep. For each entry in `SWEEP_PLAN`
    (every cascade knob is swept, by operator direction) it flies a fixed multiplicative candidate bracket
    (x0.5…x2 around the `constants.py` default) over a pan x tilt maneuver
    grid — pan-axis entries capped at tilt >= `GRID_TILT_CAP_DEG` (set to cover
    the sphere), pitch entries on the full grid — and scores each
    candidate by its WORST cell (damping-weighted cost; no-locks score
    1000 + closest approach). Decisions then pass through gates: hysteresis
    (10%+1 to displace the baseline), NEAR-MISS confirmation (a
    strictly-better-but-short challenger triggers median-of-3 re-flies of both
    deciders at their worst cells, adopted at 3%+0.5), a confirmation re-fly on
    full-margin switches (winner's-curse guard), a closest-approach tie-break
    for no-lock pairs, and a headroom gate on destabilizing-up entries. Passes
    repeat over still-moving params until converged (bounded by `MAX_PASSES`),
    then a combined VALIDATION flies the full set at the hardest global cell.
    It reuses the point-balloon-chaser injection hooks (`SyntheticBalloonInjector`,
    `ptest_enter_acro`/`ptest_begin`/`steering_errors`) plus the controller's
    `autotune_pause_tracking()` / `autotune_finish()`, with floor-triggered
    altitude protection and trial re-flies. Winners are applied live and written
    to `controller_autotune_<ts>.txt` (opens with a paste-ready WINNING SET
    block) plus `suggested_constants_t.py` (the CHANGED winners substituted into
    a copy of the live `constants.py`); nothing is auto-persisted — review and
    copy over `constants.py` to keep. SIM only (it drives gains to the edge on
    purpose).

### When to run `i` (Plant-ID) vs `t` (Controller-Autotune sweep)

*This operator workflow guideline also appears in `CLAUDE.md`; the summary is
kept there and the full reasoning here.*

**`i` measures the airplane; `t` tunes the software.** Plant-ID characterizes the
*physics* — the airframe RATE plants per axis (control power, damping, the honest
feedforward and the overshoot floor) — via open-loop MANUAL doublets, so its output
is gain-independent: changing `constants.py` gains never requires re-running `i`.
The pan authority (the turn a degree of bank buys) is a separate physics
measurement, made by the `k` frequency sweep (bank→yaw, with phase). The sweep `t`
optimizes controller gains *against whatever airplane is live*, so its winning
set is only valid for the configuration it flew with. That is a one-way dependency:
**physics upstream, gains downstream.**

- **Run `i` when an airframe rate plant changed (or was never measured):** first
  run on a new airframe or sim model; prop/thrust, weight/CG, or cruise-throttle
  changes; and after ArduPlane AUTOTUNE (the FC's inner rate loops are part of the
  plant as this controller sees it). It yields the per-axis feedforward
  (`PTCH_RATE_FF`, `RLL_RATE_FF`) and the overshoot floor, not the pan authority.
- **Run `k` when the pan authority (bank→yaw) changed:** its numbers are what
  `plant_id_rescale` carries the pan gains from sim to the real aircraft with.
  Same airframe/airspeed triggers as `i`, since the pan authority is a physics
  quantity too. `k` currently REFUSES to start: its
  excitation is a roll-rate perturbation added to the roll law's output, and the
  roll law runs on the FC, so nothing on the companion can add to it. It runs
  again once the excitation is carried to the FC (`EXCITE`, which
  `balloon_chaser_fc.lua` already generates against its own clock).
- **Run `t` when the controller's world changed:** after code changes to the
  control laws, cost function, or sweep decision rules; after hand-editing gains,
  a rescale from `k`'s numbers included (the bracket re-centers on the new
  baseline); or as periodic verification. The workhorse — hours, usually overnight.
  It tunes the cascade's gains, so it is flown with Steering Law 0.
- **Measurement before tuning, always.** A sweep flown before fresh measurement
  tunes an airplane you no longer have and its winning set is dead on arrival.
  Running `i` or `k` after a `t` does not by itself invalidate the tune — a
  measurement changes no gain — but if it shows the airplane changed, re-run `t`
  to re-anchor.
- **Real aircraft ordering:** ArduPlane AUTOTUNE → real `i` (rate feedforwards) and
  `k` (pan-authority) → `plant_id_rescale` on the `k` numbers → bake derated values
  → hand-polish in the manual tuner. The sweep itself stays SIM-only until validated
  for real flight.

---

## Controller-Autotune stop/resume and HUD timers

*The rules — no separate save step on the abort path; no HUD text is ALL-CAPS —
live in `CLAUDE.md` (§§ "Controller-Autotune stop/resume" and "HUD text
conventions"). This is the mechanism.*

### Graceful stop & resume (checkpointing)

The Controller-Autotune sweep can be stopped and resumed without losing progress:

- The checkpoint `incomplete_controller_autotune.json` is rewritten **after every test
  (every flown grid cell)**, not just on a deliberate stop. It holds the full state:
  every swept parameter's current value, the frozen (`settled`) set, the in-progress
  pass index and which params were already tuned in that pass, the accumulated
  per-parameter results, the trial count, and — when a parameter is mid-flight — that
  parameter's **partial `(candidate x cell)` cost matrix** (stored sparsely as
  `{attr, cands, baseline, filled:[[ci,gi,cost],...]}`).
- Because the sweep runs in a **daemon thread** (terminated outright on Ctrl+C) and the
  checkpoint is only ever cleared on NORMAL completion, whatever the last per-test write
  left on disk is exactly what a relaunch resumes from. So a **crash OR a Ctrl+C OR a
  't' abort** all resume automatically. There must be NO separate save step on the
  abort path: an explicit abort-time `_save_checkpoint` clobbers the live
  in-progress matrix with `None`.
- **At sweep start** exactly one tuning (checkpoint) file should exist. If a relaunch
  finds MORE than one match for `CHECKPOINT_GLOB` (stale files from an older/!=version
  run), the resume balloon is ambiguous, so they are ALL deleted and the sweep starts
  fresh rather than guessing. (Timestamped `controller_autotune_<ts>.txt` reports are a
  separate output and are NOT touched by this cleanup.)
- On the **next launch**, if a single checkpoint exists *and the config still matches*
  (same `SWEEP_PLAN` attrs, same maneuver grid, same maneuver mode), it RESUMES;
  otherwise it starts fresh and resets the swept params to their defaults. A config
  mismatch is logged and the stale checkpoint ignored.
- A **completed/converged** sweep deletes the checkpoint and writes the normal
  `controller_autotune_<ts>.txt` report. A **crash** keeps the checkpoint (so progress
  isn't lost) and writes a partial report.
- **Resume granularity is sub-parameter.** A parameter interrupted mid-tuning re-flies
  ONLY the grid cells it had not flown yet: `_tune_one` re-seeds the cost matrix from the
  checkpoint, SKIPS already-flown cells, and re-flies the rest. The kept value is a pure
  function of the full matrix (lowest worst-case over cells), so it is **identical**
  whether the parameter was flown in one go or resumed after a crash. The saved
  `baseline` is authoritative on resume (the store may hold the last in-flight candidate,
  not the pass-start value), keeping the convergence/"changed" test correct. Helpers:
  `_save_checkpoint` / `_load_checkpoint` / `_clear_checkpoint`, the per-trial
  `_save_progress` closure, and `_tune_one`'s `resume_state`/`save_cb` params in
  `controller_autotune_sweep.py`.

### HUD timers (upper-left)

Every writing in the gimbal-view HUD's top-left column is **MAGENTA — the same colour as
the bottom-left `Mode` line**: the gimbal angle readout (y=25), the Nose to Path, Nose to
Balloon and Path to Balloon error readouts (y=50, 75, 100), the shared Straight/Aligned
counter (y=125), then the slot at y=150 — Balloon Distance in `p` and `x` only, the
per-test seconds counter during a run — and the run banner below it (y=175, or
y=150 while no test counter is shown).
(The nose-camera frame's own "Nose Camera" title, also top-left, is magenta too.) The
top-RIGHT status/aileron/elevator blocks stay yellow. The mode is named once, on the
bottom-left line; the top-right block starts with the roll error. The counters render ONLY while
actively counting (no idle/grey/zero text), driven by controller state via `HudSnapshot`.

**No HUD text is ALL-CAPS.** Labels are title-case — `Straight`, `Path Aligned`/`Nose
Aligned`, `Autotune` — and the values are title-case too:
the `Mode:` value, plus the sweep's `Alt protect:` status string. Standard abbreviations/units
(`N/A`, `m/s`, `deg`, `AoA`) are deliberately left as-is.

- **Shared counter at y=125** — the Straight timer and the Aligned timer share this one
  slot because they are mutually exclusive (tracking is paused during the level-out, so
  only one can be counting). Both show a **colon before the value**:
  - **`Straight: X.Xs`**: the HELD-straight-and-level time of the current inter-test FBWA
    level-out, counting 0 → `STRAIGHT_HOLD_S` only once the belly is actually parallel to
    the ground — i.e. wings level (`STRAIGHT_ROLL_TOL_DEG`), not turning
    (`STRAIGHT_YAWRATE_TOL_DPS`), nose near level (`STRAIGHT_PITCH_TOL_DEG`) AND nose
    settled (`STRAIGHT_PITCHRATE_TOL_DPS`, so it is not still pulling out of a dive). It
    stays hidden while the aircraft is still rolling toward level OR still pitching up.
    Source of truth is `link._straighten_level_since` (published every loop by
    `level_and_hold`, which gates on pitch angle + pitch rate as well as roll + yaw:
    the instant the aircraft became and stayed straight-and-level, or `None`); the
    controller gates on `straighten_start_t` (set/cleared by the sweep's `_straighten`)
    and exposes the held elapsed via `_resolve_straighten_seconds()`. NOTE the settled
    FBWA cruise pitch is the trim attitude (a few degrees off 0, airspeed-dependent), not
    exactly level, so `STRAIGHT_PITCH_TOL_DEG` is deliberately a little loose; if the nose
    never settles within tolerance the level-out times out (`STRAIGHT_TIMEOUT_S`) and
    proceeds.
  - **`Path Aligned: X.Xs`**: how long the flight path has held within
    `ALIGN_TOL_DEG` of the balloon (`aligned_now()`, on `alignment_errors()`). Driven by
    `_resolve_aligned_seconds()` (resets the instant it leaves tolerance, shown only
    while > 0). Climbs toward `ALIGN_DWELL_S` during a trial.
- **`Test: X.Xs`** (y=150, magenta): the per-test SECONDS counter, on its own line. It
  times the current test from the moment the inter-test level-out reached straight (the
  point the test actually begins), counting up toward the no-lock timeout
  (`ALIGN_TIMEOUT_S`) and freezing at the elapsed value the test ended with — the
  time-to-align on a pass, the timeout on a no-align skip. The trim map times each cell the
  same way, from the moment its balloon is live toward `CELL_TIMEOUT_S`, freezing when the
  cell holds or stops. Wiring: the sweep stamps `ctrl.autotune_test_t0 = monotonic()` right
  after `_straighten` returns and clears `autotune_test_done_s`; when the trial returns it
  sets `autotune_test_done_s = now - t0` to freeze (a re-fly on an altitude breach resets
  it at the next level-out). The controller resolver `_resolve_autotune_test_seconds()`
  returns the frozen value if set, else `now - t0` (live), else None; both fields are
  cleared by `autotune_finish()`. Hidden when no test is in flight. NB: measured from
  straight-achieved, so it includes the ~0.5s inject/arm warmup before the line-up loop's
  own `ALIGN_TIMEOUT_S` window - i.e. a skip reads a touch over it.
- **`<run>  MM:SS  <status>`** (y=175, magenta): the run timer, driven by
  `autotune_start_t`. `<run>` names the run — its HUD label when it sets one ("Trim
  Map"), else "Autotune" for the sweep — the same name as the bottom-left `Mode:` line.
  `<status>` is the live "test x/y  param @(pan,tilt)" trial line during tuning, the
  trim map's "cell n/N: climb ... @ ... m/s - <phase>", the validation message, or the
  alt-protect climb message.

The "test x/y" counter survives pause/resume: the checkpoint stores `in_pass`, and on
resume the runner sets `progress.in_pass` to the saved value and re-sizes `pass_total`
to (already-done + work-remaining) so x continues from where it paused (instead of
restarting at 0) and still ends at y. With sub-parameter resume the interrupted
parameter re-flies only its UNflown cells, so `pass_total` also subtracts that
parameter's already-flown cells (`len(in_progress["filled"])`) to stay exact.

---

## Sweep / plant-ID operator display

*The rule — the HUD mode line is display-only; plant-ID's `_mode = 'autotune'`
drives control and must not change — lives in `CLAUDE.md` § "Sweep / plant-ID
operator display". This is the detail.*

**The HUD mode line is display-only.** plant-ID deliberately runs as
`_mode = 'autotune'` ("reuse the sweep's proven mode regime"), so that string
drives CONTROL behaviour and must not change. `_mode_label` overrides only what
the HUD prints, so `i` reads "Mode: Plant ID" while everything downstream still
sees `autotune`. Cleared when the run ends.

**Altitude protection APPENDS to the status, it does not replace it.** The test
line reads `test 12/36 pan_p_gain @(+40,-40) | Alt protect: climbing
to 250m` and reverts when the climb finishes, so the operator does not lose
track of which test is running during a long climb. Split on ' | ' first so
repeated protections cannot stack suffixes.

**The trial denominator is per-PASS, and says so when it is unknown.** The grand
total across all passes is NOT knowable: later passes re-tune only the gains
whose choice is still moving, so the horizon is data-dependent. The counter
shows `P2 trial 14/36` per pass plus a cumulative count, and
`0/Total not yet known` before a pass has been planned - better than a number
that is wrong. The NOISE-FLOOR pass does know its horizon exactly
(`reps x cells`) and sets it as pass 0. Reading `0/0` there means
`start_pass()` is only called inside the tuning loop that runs after it.

---

## Sweep report: cells that never converged

`_CELL_LOCKS` tallies `(pan, tilt) -> locked/attempts` inside
`_fly_with_protection`, so EVERY trial is counted including the near-miss,
confirm and headroom re-flies. The report then lists cells that never locked
under any gain the sweep tried.

Why it matters: the cost is a MAX over cells, so an unwinnable cell pins every
candidate at the same no-lock penalty. It cannot separate good gains from bad,
it silently caps what the sweep can conclude, and every trial spent there is
wasted. `worst[]` aggregates across cells and discards which one failed, so
without it the only way to find such a cell is to read the logs by hand.

---

## Camera view switches

*The rules — neither camera-view switch may affect control; OpenCV GUI calls run
only on the thread that pumps HighGUI's event loop; both-views-off must keep the
keyboard alive — live in
`CLAUDE.md` § "Camera view switches". This is the mechanism.*

Two Gain Tuner rows control the OpenCV windows: `Balloon Chaser Cam Window`
(`nose_cam_display`, DEFAULT 0 = off) and `Gimbal Cam Window`
(`gimbal_cam_display`, DEFAULT 1 = on).

Detection runs off the GIMBAL camera via `HsvDetectionSensor`; the nose camera is
display-only in the strict sense (audited: its only uses are the HUD projection
scale, two startup log lines, the startup wait and the render call). The gimbal
CAMERA always keeps decoding whatever its window is doing - only its WINDOW is
optional.

Both views off is a legitimate configuration, and with the shipped default
(nose off) it is the ONLY way to close the gimbal view at all. What needs
protecting is the KEYBOARD: `cv2.waitKeyEx` can only deliver keys to an OpenCV
window, and it is the program's only key path.

The Gain Tuner hosts the keyboard instead, so nothing is left on screen. It
binds `<Key>`, queues the keystroke in a bounded `deque` (append/popleft are
atomic under the GIL, so no lock is needed between the Tk thread and the
control thread), and `KeyboardHandler.handle_key()` drains it whenever
`cv2.waitKeyEx` returns -1. `waitKeyEx` is still called unconditionally, because
it is what pumps the HighGUI event loop for any camera window that IS open. It
returns the key's full code, not its low byte, because an arrow's low byte under
GTK is a capital letter's (`keyboard._arrow_direction`). Keys
typed into a value box are swallowed by the box - guarded twice, by the row's
`editing` flag and by `focus_get()`, because `focus_get()` returns None when
the toplevel lacks window-manager focus and cannot be relied on alone.

Two tempting shortcuts do not work. Refusing to close the last window (and
reopening the gimbal view) makes the Gimbal Cam Window switch do nothing in the
default configuration. Opening a small stand-in window for the keyboard only
replaces one window on screen with another, which is not what "close the window"
means.

**Threading.** `cv2.namedWindow` / `destroyWindow` / `imshow` / `setMouseCallback`
/ `waitKey` / `waitKeyEx` are not thread-safe and must run on the thread that
pumps HighGUI's event loop.
The Tk tuner thread therefore only ever writes two ints; `_apply_camera_views()`
runs once per control tick and is the sole owner of the window lifecycle. The
window calls live in five functions - `run()`, `_apply_camera_views()`, the two
`render_*_camera()` and `KeyboardHandler.handle_key()` - and `run()` is the only
caller of the other four, so all five run on the thread `main()` calls `run()`
on. `test_hud_contract.py` reads the source to hold that: it lists every HighGUI
function the code reaches (under any alias of `cv2`, called or passed on), and
fails on one outside the five, on another caller of the four, or on any of them
as a thread's target. A lambda or nested def counts as a function of its own,
because its body runs wherever it is called from. A call from the wrong thread
can work on the desk and still freeze or crash the program another day, and with
it go the GUIDE frames the FC flies by. The mouse callback belongs to the window, so it
is re-attached whenever the gimbal window is recreated.

The nose camera's decode gate (`GazeboCamera.decode_enabled`) is a plain bool
read on the gz transport callback thread and written on the control thread -
atomic under the GIL, and deliberately NOT lock-protected, because taking the
frame lock there would put the transport thread in contention with the renderer
every frame. `_on_image` records arrival and the native geometry BEFORE the
gate, so a camera that starts disabled cannot hang the boot on
`received_first_frame` and the HUD still has its aspect correction if the view
is switched on later.

Measured cost of the nose pipeline at 320x240 -> 1280x720: colour convert
0.033 ms + resize 0.98 ms per received frame, plus a 0.21 ms frame copy and
~0.16 ms of overlay per tick at 50 Hz - about 5% of a core at 30 fps BEFORE
cv2.imshow, which under WSLg goes through the Wayland compositor and is likely
the larger cost again (not measurable from the dev container).

---

## FOV measurement (diagnostic)

*The rule — the FOV estimator is DIAGNOSTIC ONLY and its output must never reach
the control path (that boundary IS C-02) — lives in `CLAUDE.md` § "FOV
measurement". This is how it works.*

A background estimator measures the gimbal camera's horizontal field of view
from motion that is ALREADY happening, so a lens swap or an in-flight zoom is
noticed rather than assumed. It is enabled by the Gain Tuner switch
`FOV_EST_ENABLE` (default 0). `FovEstimator` in `controller.py` owns it;
`SdfManifest` in `adapters/sim_gazebo.py` supplies the ground truth.

Its output goes to exactly five CSV columns — `fov_est_deg`, `fov_est_m1_deg`
(aircraft observer), `fov_est_m2_deg` (gimbal observer), `fov_truth_deg`,
`fov_est_n` — and NOWHERE else. The reason is the fusion identity `T = g + p*k`:
recovering the true bearing needs the pixel-to-angle scale `k` (the FOV), which
the lens can change in flight, so C-02 forbids FOV from steering and this
estimator only WATCHES it — measuring FOV to detect a lens/zoom change is not the
same as steering on it.

**It never commands motion.** No maneuver, no gimbal command: it reads the
detection pixel, the gimbal encoders and the attitude the aircraft and the
tracker already produce. If nothing is moving it harvests nothing and the last
estimate stands.

**How it measures.** Over a short window the balloon's true bearing sweeps by the
camera's rotation (from attitude and encoder ENDPOINTS, never differentiated
rate telemetry), while its normalised pixel shifts by that angle divided by
`tan(FOV/2)`; so FOV is `2*atan(slope)` of true-angle against pixel-shift, fit
through the origin with a `cos^2` correction so it is unbiased off-centre and
with exponential forgetting so a zoom is tracked within seconds. Two observers
keep it honest: Method 1 uses windows where the aircraft's own rotation
dominates (fresh on-chip attitude), Method 2 where the gimbal slew dominates;
they fuse into one estimate and are also kept apart so a post-run comparison
sees which did better. The gimbal camera renders slower than the control loop,
so a sample is taken only when the pixel actually CHANGED — never a held pixel
paired with a newer attitude.

**Ground truth is read from the active SDF at launch, comparison ONLY.**
`SdfManifest` is the sim-side analogue of the ArduPlane parameter gate: at launch
it resolves the active model SDF the way Gazebo resolves `model://` (the
`VP_MODEL_SDF` override, then the gz resource-path variables, then the default
install path), parses every camera and gimbal-joint parameter, checks the model
identity, and logs the whole manifest beside the FC parameter gate. The gimbal
camera's `horizontal_fov` becomes `fov_truth_deg`. Like the FC identity values it
describes what is flying and is NEVER fed into a calculation — it exists so the
estimate can be scored afterwards. A missing or wrong SDF WARNS and never raises:
this is a diagnostic and must not ground the aircraft; only the FC gate, which
guards the airframe, refuses to start.

---

## Simulator air: AltitudeLiftDrag

*The rules — every lift-drag element is `AltitudeLiftDrag`, none carries
`<air_density>`, its atmosphere is ArduPilot's, the world's elevation equals
ArduPilot's home altitude — live in `CLAUDE.md` § "The simulator flies in
ArduPilot's air". This is the mechanism.*

### Two halves that must describe the same air

In SITL the aircraft's forces come from Gazebo and its airspeed reading from
ArduPilot. The ardupilot_gazebo plugin sends no airspeed in its JSON frame, so
SITL makes one: the air-relative true speed divided by `AP_AHRS::get_EAS2TAS()`,
which SITL evaluates from `AP_Baro`'s 1976 standard atmosphere at the aircraft's
altitude (`SIM_JSON.cpp`, `SIM_Aircraft.cpp`, `AP_Baro_atmosphere.cpp`). The
reading is therefore equivalent airspeed: the speed at sea-level density that
gives the same dynamic pressure.

Stock gz-sim `LiftDrag` computes dynamic pressure as `0.5 * rho * v^2` with one
fixed `<air_density>`. At a given true speed the wing then feels the same
pressure at every height while the reading falls with height, so the reading
stops measuring what the wing feels. Anything fitted against it — the pitch
trim's `(VREF / airspeed)^2`, a rate feedforward, a plant from `i` — holds only
at the height it was fitted at.

For scale: with the world at 584 m, stock `LiftDrag`'s default density is
standard air at about 180 m. At 100 m above the field SITL's `EAS2TAS` is about
1.03, at 3000 m above it about 1.20. The same flight, with the same wing forces,
reads 14% slower up high, and a `(VREF / airspeed)^2` term comes out about a
third larger there than at 100 m.

On a real aircraft the two cannot disagree: a pitot tube feels the same air as
the wing, and its reading is equivalent airspeed at any height and in any
weather. The simulator is made to behave the same way, not the flight code to
compensate for the simulator.

### The mechanism

`Arduplane/AltitudeLiftDrag/AltitudeLiftDrag.cc` is gz-sim's `LiftDrag` with one
change: every update computes the density at the centre of pressure's height
above sea level, the world's `<spherical_coordinates><elevation>` plus the
centre of pressure's world Z, through the same 1976 table, constants and
geometric-to-geopotential conversion as `AP_Baro::get_air_density_for_alt_amsl`.
Dynamic pressure is then `0.5 * rho(h) * v^2 = 0.5 * rho_sea_level * EAS^2`, so
lift, drag and moment depend on the reading alone, as they do on the real
aircraft.

All seven lift-drag elements use it: the two propeller blades on `motor_link`,
and the wings, the V-tail halves and the fuselage side force on `base_link`. The
propeller being one of them matters: at a given motor speed its thrust falls
with density, as a real propeller's does, which is why the pitch trim's thrust
term carries the density ratio ("Trim map design").

The height is a flat-earth sum, the same one ArduPilot makes from its home
altitude and the NED position the ArduPilot plugin sends, so the two agree only
when the world's elevation and ArduPilot's home altitude are the same number.
Both are 584 m: the world's `<elevation>` and ArduPilot's default SITL location.

### Why a copy of LiftDrag rather than a new model

The airframe's coefficients were written for `LiftDrag`'s arithmetic, and a
different aerodynamic model would change what each of them means. The copy
follows the upstream source line for line, upstream comments included, so a
diff against it shows only the parts marked `talon_sim`: the density, the class
and namespace names, the configuration checks and the startup line. gz-sim 7.9,
8.3 to 8.10, 9.0 and Jetty share the arithmetic; 8.0 to 8.2 predate its wind and
reverse-flow handling.

### Refusals

A world without `<spherical_coordinates>`, or an `<air_density>` left on an
element, is logged as an error and that element generates no forces. An
aircraft that cannot fly is noticed at once; one flying on a density nobody
chose is not.

### Build, install and the startup line

`CMakeLists.txt` builds `libAltitudeLiftDrag.so` against the gz-sim release named
by `GZ_VERSION` (Harmonic when unset), the same selection ardupilot_gazebo makes,
and `cmake --install` puts it in `~/.gz/sim/plugins`, which gz-sim searches after
`GZ_SIM_SYSTEM_PLUGIN_PATH`. The model file refers to it as
`filename="AltitudeLiftDrag"`, `name="talon_sim::AltitudeLiftDrag"`. With Gazebo
at verbosity 3 or higher each element prints one line at load: the link, the
world elevation, and the density there.

`test_export_gazebo.py` pins the switch from the repo side: no stock element, no
fixed density, a world with an elevation, and the table and constants equal to
ArduPilot's.

---

## SITL from conda-forge, where Gazebo's package host is unreachable

*An environment whose network policy blocks `packages.osrfoundation.org`,
Gazebo's own package host, can still build and run the simulator's parts. What
is verified, and what is not yet.*

**The firmware.** ArduPlane SITL builds from the pinned commit (CLAUDE.md
"Verify against the exact ArduPilot version") with the submodules `modules/waf`,
`modules/mavlink`, `modules/DroneCAN/*`, `modules/littlefs` and `modules/lwip`
initialised, and the Python packages `empy==3.3.4 future lxml pexpect
ptyprocess pymavlink setuptools`:

    ./waf configure --board sitl
    ./waf plane                       # -> build/sitl/bin/arduplane

Started with ArduPilot's built-in V-tail physics (`--model plane-vtail`, with
`Tools/autotest/models/plane.parm` and `default_params/plane-vtail.parm`), the
scripting parameters `lua_setup.ensure_scripting` checks, `--serial5 tcp:5765`
and the FC scripts in `scripts/` and `scripts/modules/` of the directory it runs
in, it boots, loads `balloon_chaser_fc.lua` and the script prints its startup
line. That model is not the Mini Talon: its V-tail outputs are SERVO2 = 80 and
SERVO4 = 79, the reverse of this airframe's, so the manifest gate cannot pass
it: whatever its parameter-name check finds, its airframe check refuses those
outputs (§ "FC configuration and the manifest gate"). It serves for
checking the script against the real firmware's bindings, not for flying chases.

**Gazebo.** conda-forge carries Gazebo Harmonic: `gz-sim8`, and the
`gz.transport13` / `gz.msgs10` Python bindings `adapters/sim_gazebo.py` imports
(`gz-transport13-python`, `gz-msgs10`). With micromamba, itself the conda-forge
package `micromamba`:

    micromamba create -p <env> -c conda-forge --override-channels \
        python=3.12 gz-sim8 gz-transport13-python gz-msgs10 gz-tools2 \
        cmake make cxx-compiler pkg-config

Its gz-sim is 8.10.0, the release `AltitudeLiftDrag.cc` follows. The server runs
headless (`gz sim -s -r <world>`), and camera sensors render with software
OpenGL on a virtual screen (`xvfb-run -a -s "-screen 0 1280x1024x24" gz sim -s -r
<world>`): a 640x480 camera asked for 30 Hz delivers about 29 frames a second on
a four-core machine with no GPU. The environment's Python carries Tk, which the
Gain Tuner needs.

**The model.** The meshes `model.sdf` names (`model://talon_balloon_chaser/meshes/`)
are the Mini Talon's in ArduPilot/SITL_Models, `Gazebo/models/mini_talon_vtail/meshes/`,
beside its parameter file `Gazebo/config/mini_talon_vtail.param`;
`talon_runway.sdf`'s `model://runway` is ArduPilot/ardupilot_gazebo's
`models/runway`.

**Not yet verified:** building ardupilot_gazebo's `ArduPilotPlugin` and
`AltitudeLiftDrag` against that environment, the FC parameters that pass the
manifest gate on that airframe, and a `p` chase flown through the whole chain -
Gazebo, SITL on the JSON backend and the companion - or whether a machine with
no GPU keeps that chain at real time, which the companion's wall-clock timing
needs.

---

## Deliberate stops: where the program halts on purpose

*The rule — every one of these stops loudly, and a new one joins this list —
is in `CLAUDE.md` § "Deliberate stops". This is the list. When the program
stops, the console's last lines say which one fired; start here.*

The program stops on purpose wherever carrying on would fly, or record, an
aircraft it can no longer vouch for. Each stop prints why, in words, on the
console. None is silent and none is retried.

### At startup: before anything flies

| stop | trigger | what the console shows |
|---|---|---|
| no autopilot | no MAVLink HEARTBEAT within the connect timeout (`_wait_for_heartbeat`) | `No MAVLink heartbeat after Ns.` and the commands to check the autopilot and its port |
| manifest gate | the FC returns no parameters, a different parameter name set, or a different airframe (`verify_param_manifest`, § "The manifest gate runs first") | `Parameter manifest gate: ...` with the missing, unexpected or wrong names |
| parameter differences | a checked parameter differs from what the code expects (`halt_on_param_mismatches`) | the list of differences, then a prompt: Enter continues with the FC's values, Ctrl+C stops. With no console to answer, startup refuses |
| script deployment | the autopilot reads a different scripts directory, a source file is missing, or a copied file fails its hash check (`lua_deploy.deploy`) | `[lua] script deployment failed` after the reason (`REFUSING TO DEPLOY: wrong directory`, `missing from`, `FAILED verification`) |
| scripting reload | the scripts changed and `MAV_CMD_SCRIPTING` could not reload the engine | `[lua] the scripting engine could not be reloaded` |
| FC script silent | no STATE from the FC script after the startup waits and one reload | `[lua] the flight controller script is not responding` |
| Gazebo camera | the camera topic cannot be subscribed | `Failed to subscribe to Gazebo topic` |

### In flight: the control loop stops

Every stop in the control loop ends the same way. `run()` leaves its loop,
hands the aircraft back to LOITER (`go_to_loiter('shutdown',
keep_operator_params=True)`), shuts the link, closes the flight log, the Gain
Tuner and the camera windows, and only then calls `_halt_on_console`: a framed
`THE PROGRAM HAS STOPPED` block with the reason, which waits for Enter so the
reason cannot scroll away. The flight log is complete up to the stop.

| stop | trigger | the reason printed |
|---|---|---|
| a tick raises | any exception inside the control loop | `The control loop crashed: <type>: <message>`; the traceback is just above it. Deliberate: an error that ends the program is found, one that is caught is not |
| FC silent | no STATE for `FC_LINK_TIMEOUT_S` (`LuaOffloadLink.link_fault`) | `No report from the flight controller for N s`, with the causes: the script stopped (a Lua error stops it; the FC console shows the file and line), the scripting engine restarted, or the link broke |
| FC speaks another protocol | a whole, checksummed frame from the FC whose id or size this build does not know | `The flight controller sent a message this program cannot read` |
| FC refused a frame | STATE flags bit 4: the FC rejected a frame from this program for its id or size | `The flight controller rejected a message from this program` |
| FC fault | STATE flags bits 5-7 carry a fault (`protocol.FAULT_NAMES`): the script dropped control mid-chase | `The flight controller script stopped flying the aircraft and handed control back: <fault>` |

The last three mean the companion and the FC run different builds, or the
script is in trouble; in every case the FC has already released control on its
own (§ "The script must CLAIM control, and must claim it narrowly").

### On the flight controller: what the script stands down for

The script never stops itself; it releases control and says why on the FC
console (MAVProxy, or the ground station's messages). These lines are the FC's
half of the stops above.

| console line | severity | what happened | companion |
|---|---|---|---|
| `balloon chaser: FAULT, released control - no gimbal angles` / `no attitude from the AHRS` / `no balloon estimate` / `no oracle bearing` | 3 | an input the law was flying on disappeared (gate 2, 3 or 4); the fault is latched | halts (FC fault) |
| `balloon chaser: FAULT, script error - <file:line: message>` | 3 | a Lua error in a tick; the tick's STATE still goes out carrying the fault | halts (FC fault) |
| `balloon chaser: rejected msg N of M bytes - companion and FC run different protocol builds` | 3 | a frame from the companion with an unknown id or the wrong size, stepped over undecoded; printed once | halts (FC refused a frame) |
| `balloon chaser: released control - GUIDE stale` | 3 | no new `GUIDE` for `GUIDE_STALE_MS` (gate 7) | the companion is the side that stopped |
| `balloon chaser: released control - no balloon estimate` (or `- no oracle bearing` in `p`) | 4 | a fresh chase waiting for its first estimate; not a fault | carries on |
| `balloon chaser: cannot take control in ACRO` | 4 | `nav_scripting_enable` refused (gate 6), printed once until a claim succeeds | carries on |
| `balloon chaser: no scripting serial port (SERIALx_PROTOCOL=28)` | 3 | at script start; the script does not run | halts at startup (FC script silent) |
| `SCALING_SPEED/AIRSPEED_MIN/AIRSPEED_MAX not ...`, `PTCH_RATE_FF/P not readable`, `YAW_RATE_ENABLE is not 1`, `ACRO_YAW_RATE is 0`, `YAW_RATE_* untuned`, `no SRV_Channels binding` | 3 | at script start, a parameter or binding a feature depends on is missing; that feature is off | carries on |

---

## How the code hash (`code_sha256`) is calculated

*What it covers, how it is computed and how to print it are in `CLAUDE.md`
§ "When modifying code", which Claude follows on every change. This is why it
covers what it does.*

Canonical implementation: `package_sha256()` in `Companion/csv_logger.py`.
It is computed once at import into the module constant `PACKAGE_SHA256` and
written to the head of every log as `# code_sha256: <hex>` (prefixed `# ` so
`csv.reader` / `pandas(comment='#')` skip it).

The digest covers the flying `.lua` sources and the repo-root `main.py` as well
as the package's `.py`. With the steering laws on the flight controller, a `.py`-only
digest would be IDENTICAL for two runs flown by completely different steering
code — the one thing a fingerprint exists to make impossible — and `main.py` sets
the serial link and the startup order from outside the package. `ap_shim.lua`,
`ap_stubs.lua` and the `test_`/`xcheck_`/`e2e_`/`replay_`/`hello_` drivers are
excluded: they never reach the aircraft, and letting them move the digest would
make it noisy. Paths are taken relative to the REPO ROOT so `LUA/` and
`Companion/` cannot collide, and each path is hashed before its file's bytes,
so a rename changes the digest. `Arduplane/` (the Gazebo world, model and
`AltitudeLiftDrag`) is not covered: it is simulator input, so two logs with the
same digest can still have been flown in different simulated aerodynamics.

Raw bytes are hashed deliberately: **line endings, BOM, and trailing whitespace
all affect the digest** — the digest fingerprints exactly the files on disk.

The digest also covers every `.md` document in the repo (`CLAUDE.md`,
`ARCHITECTURE.md`, `TODO.md`, `SUGGESTIONS.md`, `Notes/`), found anywhere outside
hidden directories and `__pycache__/`. They are the rules and the design the code
is written against, so a log's digest names the documents as well as the code it
was flown with. The cost is that a docs-only edit moves the digest, and the resume
checkpoints that compare it (plant-ID, the trim map) start afresh after one, as
they do after a code edit; the sweep and freq-ID record the digest without
comparing it.

`code_sha256` covers the flying source and the documents, nothing else. The
archive's own checksum (the SHA-256 of the `.zip`) is a separate value; record it
alongside as `archive_sha256` if you track the archive file itself.
