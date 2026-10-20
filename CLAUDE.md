# CLAUDE.md — Companion

**This file is instructions, not a diary.** It states what must be true, what
must never be done, the directives, conventions and prohibitions for working on
this codebase, and the technical knowledge needed to follow them. Present tense,
about the code as it stands.

**What Claude acts on belongs here, at any level of detail.** A procedure Claude
carries out (calculating `code_sha256`, regenerating the generated Lua, running
the desktop checks, building the archive), a trap in ArduPilot or the toolchain
that a change has to avoid, and every fact a rule depends on. How low-level it
is does not decide where it goes; whether Claude needs it to do the work right
does.

**Documentation lives in `ARCHITECTURE.md`.** How each subsystem works, why the
design is the way it is, worked examples, measured evidence, and reference
tables for reading what the system produces (the log's columns, the wire
layout) are there, not here. Many rules below carry a `→ ARCHITECTURE.md §`
pointer to the explanation behind them. When a rule and its explanation
disagree, the rule here wins and the explanation is the stale one.

It is NOT a record of investigations. No bug post-mortems, no "what the run of
<date> showed", no changelog of what a value used to be. That material is real
and worth keeping — it belongs in a comment beside the code it describes, so
that changing the behaviour puts the note in front of you. A reader of this file
should be able to follow every rule without knowing any of that history.

The test: if a section reads like a lab notebook, it is in the wrong file — it
goes next to the code it constrains. If it reads like a standing order, a
procedure Claude follows or a trap a change must avoid, it belongs here. If it
reads like an explanation of how something works, or a reference for reading
what the system produces, it belongs in `ARCHITECTURE.md`.

Investigation notes live in the SOURCE file that owns the behaviour, never in a
separate document. A note in its own file drifts the moment the code moves; a
note above the constant it explains cannot.


Instructions for working on this codebase.

> **Do not modify this file (CLAUDE.md) automatically.** Claude must never edit
> CLAUDE.md as an automatic side-effect of a code change. Claude may — and should —
> *suggest* updates to this doc when a change has made something here inaccurate or
> worth recording, after weighing whether the update genuinely belongs in CLAUDE.md;
> but it applies those edits only when the user has explicitly asked for them. (The
> notes in this turn were added at the user's explicit request.) The same holds for
> `ARCHITECTURE.md`: suggest, do not auto-edit.

> **Do not add files or change the file structure automatically.** Claude must
> never create a new file, delete a file, rename a file, or move anything between
> directories as an automatic side-effect of a code change. Claude may — and
> should — *suggest* a new file or a structural change when the work genuinely
> warrants one, after weighing whether it belongs in this project at all; but it
> makes those changes only when the user has explicitly asked for them. Editing
> the contents of files that already exist is the normal way to do the work and
> needs no permission. If a requested change seems to require a new file, Claude
> says so and waits for a decision rather than creating it. (This note was added
> at the user's explicit request.)

> **Explain in plain English.** When explaining anything to the user — a
> diagnosis, a design decision, a trade-off, why something broke — Claude writes
> the way it would to a smart person who does not work on this codebase. No
> jargon where an ordinary word will do; where a technical term is genuinely
> unavoidable, say what it means in the same sentence. Reach for a short,
> concrete, everyday example rather than an abstract description. This applies to
> chat, not to code comments and docs, which keep the precise vocabulary the
> codebase uses. A good test: would this make sense read aloud to a teammate who
> joined last week?

> **TODO.md is a record, not a mandate.** Any work on an item listed in
> `TODO.md` requires EXPLICIT user approval first. Claude must not start,
> partially implement, prototype, or opportunistically "fold in" a TODO item
> because it is listed, because it is marked `[NEXT]`, because it looks cheap,
> or because it is a prerequisite for something the user did ask for. If a
> listed item is genuinely blocking the requested work, Claude says so and
> waits for a decision rather than proceeding. Reading, citing, and proposing
> updates to TODO.md is always fine.

## When modifying code

1. **Output the whole project archive**, not just the changed file(s). Keep the
   exact package structure documented in `ARCHITECTURE.md` § "Project structure".
   Ship sources only — do **not** include `__pycache__/` or `.pyc` files (stale
   bytecode for an edited module is a footgun, and it doesn't affect the hash).
   Build it from the commit; `git archive` carries every tracked file and
   nothing else, so no bytecode:

   ```sh
   git archive --format=zip -o talon_balloon_popper_$(git rev-parse --short HEAD).zip HEAD
   ```

2. **Recalculate `code_sha256`** (the whole-package fingerprint) and report it.
   No separate manifest file is shipped; the runtime writes the fingerprint into
   the head of `flight_log.csv` as `# code_sha256:` on its own via
   `package_sha256()` (`Companion/csv_logger.py`), so there is nothing to update
   by hand.
   - **What it covers:** every `.py` under `Companion/`, recursively; the
     repo-root `main.py`; every `.lua` directly in `LUA/` except `ap_shim.lua`,
     `ap_stubs.lua` and names starting `test_`, `xcheck_`, `e2e_`, `replay_` or
     `hello_`; every `.md` anywhere in the repo outside hidden directories and
     `__pycache__/` (`CLAUDE.md`, `ARCHITECTURE.md`, `TODO.md`,
     `SUGGESTIONS.md`, `Notes/`). Nothing else: not `Arduplane/`'s sources. A
     change to a comment in a covered file changes it, and so does a change to
     the docs alone - which also discards the plant-ID and trim-map resume
     checkpoints, as a code change does.
   - **How:** one SHA-256 over the covered files in sorted order, each its
     repo-relative path and then its raw bytes, so a rename, a line ending or
     trailing whitespace changes it.
   - **Printing it,** from the repo root; the import leaves `__pycache__/`
     directories behind, so delete them after:

     ```sh
     python3 -c "from Companion.csv_logger import PACKAGE_SHA256; print(PACKAGE_SHA256)"
     find . -name __pycache__ -type d -prune -exec rm -rf {} +
     ```

   - When files move or the set of covered files changes, update this list in
     the same change as `package_sha256()`. Why each part is covered: →
     ARCHITECTURE.md § "How the code hash (`code_sha256`) is calculated".
3. **Regenerate the generated Lua; never edit it by hand.** From the repo root:

   ```sh
   python3 -m Companion.protocol        # after protocol.py: rewrites LUA/protocol.lua, prints each frame's size
   python3 -m Companion.gen_constants   # after constants.py: rewrites LUA/constants.lua
   ```

   When a frame's size changes, update the serial-budget comment beside
   `STATE2` in `protocol.py` to match; `test_link_contract.py` fails until it
   does.
4. **Run the desktop checks before every push.** From the repo root,
   `run_tests.sh` runs every Python contract check and then the Lua drivers;
   none needs SITL or a flight controller. `LUA/test_no_globals.sh` is stored
   without its execute bit, so in a fresh clone the script stops at its Lua
   step with "Permission denied", after every Python check has passed. Run the
   Lua step by hand then:

   ```sh
   sh Companion/tests/run_tests.sh
   sh LUA/test_no_globals.sh
   lua5.4 LUA/test_ap_shim.lua && lua5.4 LUA/test_balloon_chaser_fc.lua && lua5.4 LUA/test_yaw_coord.lua
   ```

5. **Anything that touches the Gain Tuner window follows the GUI conventions
   below** (console not GUI for transient messages; new controls line up with
   existing rows; labels must fit, widening the window if needed; info button
   on every row, reset button on every resettable row).
6. **Check the change against the mandatory competition constraints below.**
   They are pass/fail, they outrank every other note in this doc, and a
   diagnosis that is correct about the code can still be wrong about the spec
   (see C-04, which retired the front-half-sphere assumption a lot of this
   codebase was built on).
7. **Write comments in the present tense, about the design — not its history.**
   See *Comment style* below. This applies to comments Claude edits as well as
   ones it writes: if a rewrite touches a block carrying old narration, the
   narration goes.

## Mandatory competition constraints

The governing requirements for the competition build, reproduced from the spec.
They outrank everything else in this doc: where a constraint and an implementation
note disagree, the constraint wins and the note is the bug.

IDs are a local, CONTIGUOUS numbering: C-01 through C-04, no gaps. They do not
track the competition spec sheet's own numbers one-for-one, so cite a requirement
by what it SAYS, not by assuming its number matches the sheet.

NEVER renumber locally. The IDs are cited by name throughout the Python, the Lua
and this document; shifting them without rewriting every citation in the same
pass silently repoints each one at a different constraint.

**What the competition judges: best effort, a missed balloon the worst
result.** It sets no time limit. Never add a time-to-align deadline to a
verdict or a test, and never judge a change against one. A chase is
judged by whether the flight path aligns with the balloon (the alignment
criterion under "What the `p` and `x` tests are for") and how soon; one that
never aligns, by how far off it stays.

- **C-01 — ArduPlane ACRO-only balloon chaser actuation.** All tracking modes (p, x,
  straight, balloon chaser) command via ACRO. FBWA may be used for the catapult launch
  phase only. *Rationale: inner-loop closure happens on-chip in ArduPlane against
  fresh gyro data.*
  **C-01 is met.** The script claims control only while ArduPlane is in ACRO, and
  its roll and pitch rate demands (deg/s, via `set_target_throttle_rate_rpy()`) go
  to the same `rollController` / `pitchController.get_rate_out()` that ACRO's own
  `Mode::run()` calls with `ACRO_LOCKING` = 0 (checked at startup). The inner loop
  therefore closes on-chip against fresh gyro data, which is the rationale, and
  the outer loop is on-chip too. Only the SOURCE of the rate demand differs: while
  `nav_scripting` is active, `Plane::stabilize()` takes it from the script instead
  of from the sticks. Keep it that way — claim only in ACRO, and keep
  `ACRO_LOCKING` = 0 so ACRO's own path is the same rate controllers.
- **C-02 — No FOV-derived balloon location.** Balloon bearing, body vector, world
  vector, estimator update, steering error, p-test truth, x-test truth, and
  straight-mode direction may not use FOV, focal length, pixel-to-angle conversion, or
  pinhole projection. *Rationale: the lens may be swapped or zoomed during flight
  without recalibration.*
- **C-03 — One shared control law.** p, x, and straight all call the same shared
  control function. No per-mode control law, no per-mode gains, and no
  straight-mode level-wing special controller. Modes differ only in the source
  of the world vector they steer toward.
  **The Curve Planner and Curve Flyer are not a control law** but a numeric
  plan of a chase's turn and the module that runs it, in `p` and `x` only.
  The selected law flies the plan in its plan mode - its own gains, pitch,
  rudder and bank limit, the bank taken from the plan instead of its own
  choice - so C-03 binds that law, which is the one straight flies from its
  first tick.
- **C-04 — Coverage of the full sphere.** Balloons may appear anywhere around the
  aircraft at the moment of test initiation, including the rear hemisphere,
  zenith, and nadir directions. *New in this revision; replaces the previous
  front-half-sphere restriction.*

### FOV-free sensing (elaborates C-02)

- The camera image is allowed to answer only one question: which direction should
  the gimbal move to center the balloon?
- FOV may exist only in rendering, debug visualization, detector ROI heuristics,
  or display overlays. It must not feed balloon geometry, scoring, or any signal
  that propagates into the control law.
- The body-frame balloon direction shall be derived from gimbal joint encoders
  only (the gimbal's pan and tilt angles), without reference to camera intrinsics.
- **Lua path:** still encoder-only, but on THIS airframe the gimbal is a Gazebo
  model driven by the companion, so ArduPilot has no mount driver and the encoder
  angles are sent in `GUIDE` rather than read locally. C-02 is satisfied; the
  freshness benefit is only partial. `mount:get_attitude_euler()` is still preferred
  when a real mount exists — the script falls back to the wire only when it returns
  nil.

Rationale: the real lens can be changed, and zoom can be used during flight. Pixel
offset is therefore not a stable angular measurement. The centerline of the optical
axis remains meaningful; off-center pixel-to-angle conversion does not. Routing the
body-frame measurement through gimbal encoders gives the controller a measurement
source whose calibration is mechanical, not optical.

## Throttle changes are safety-critical

The spec: **any throttle-related change must be done under careful review, as
the propeller is rotating at high speed and safety is the highest priority.**
It is a safety requirement, not a competition criterion, so it carries no C
number.

- **Claude never changes what the motor does on its own initiative or as a
  side-effect of other work.** A change that can alter the throttle - the
  operator throttle's path to the FC (`throttle_pct`, `_effective_throttle_pct`,
  GUIDE `throttle_pct`), the FC's airspeed hold (`compute_throttle_command`),
  any RC override of CH3, or a throttle limit - is
  made only when the user has asked for that specific change.
- **A throttle-related change is named as one** - in the proposal, the commit
  message and the summary - with the throttle path it touches, so the review the
  spec asks for happens. The design such a change must respect is in
  "Flight-control invariants".

## Comment style

Comments explain **what the design does and why**, in the present tense. They are
not a changelog. Git holds the history; a comment that narrates it makes the
reader reconstruct a past state of the code before they can understand the
current one.

**Do not write:**

- What the code used to do — *"this used to overwrite X"*, *"previously a
  WARN-ONLY check"*, *"the old dot-joining path"*, *"no longer aborts"*.
- Alternatives that were tried and rejected, framed as a decision narrative —
  *"it was originally done first, on the reasoning that…, but"*.
- Dated run evidence as the justification — *"the 2026-07-26 telemetry proved
  it"*, *"confirmed across two diagnostic builds"*, *"cell +40 of the
  2026-07-27 run"*.
- Bug post-mortems — *"which is why the steep cells kept coming back empty"*.
- Provenance — *"lifted verbatim from vision_balloon_chaser_gazebo.py lines 596–630"*.
- Requests — *"the user asked for"*, *"added at request"*.

**Keep** the constraint itself. Most of the above is guarding something real; the
fix is almost never to delete the comment, it is to state the trap instead of its
discovery:

> *Before:* "This MUST come last. It was originally done first, on the reasoning
> that the wanted requests should have the final word — but a stream-GROUP
> request enables every member of that group… The 2026-07-26 telemetry proved
> it: all three were still arriving at the full group rate."
>
> *After:* "This MUST come last: a stream-GROUP request enables every member of
> that group, so `request_data_stream(EXTRA1)` re-enables SIMSTATE and AHRS2.
> Doing this first leaves all three arriving at the full group rate."

Same warning, no archaeology. If deleting the history leaves nothing behind, the
comment was scar tissue and should go entirely.

**The test:** would this sentence still make sense to someone who had never seen
any earlier version of the file? If it only makes sense as a correction to
something they cannot see, rewrite it.

**Where the finding goes instead:** into the same comment, as the TRAP rather
than its discovery. A bug found in a run becomes a statement of what must not be
done and why it is tempting; the run that revealed it is not part of the rule.
If restating it that way leaves nothing, it was scar tissue.

Nothing goes in CLAUDE.md. That file states what must be true, not how it came
to be known — see the header.

### Length

One clause where one clause will do. Long blocks are justified for genuine
derivations (the closest-approach parabola fit, the rear-hemisphere continuation,
mode 17's coupling budget) — not for restating what the next three lines of code
plainly say.

### Numbers in comments

A comment that quotes a constant, a default, or a measured figure is a claim that
silently rots. Two rules:

- **Name the constant, don't inline its value**, unless the value is the point.
- **If a measured claim matters enough to write down, pin it with a test.** See
  `test_link_contract.py`, which recomputes every figure the serial-budget
  comment beside `STATE2` in `protocol.py` quotes and fails naming the one that
  drifted. A quoted figure can be wrong by a factor of three and go unnoticed:
  when the output it describes is clipped at a limit, the clipped value reads
  the same at either gain — the clip erases the evidence.

## No magic numbers

A number that stands for something — a reference value, a scale, an ArduPlane
convention, a threshold, a margin, a fitted coefficient — is a NAMED constant,
defined once beside a comment saying what it is and where it comes from, and
every use reads the name. A bare `17`, `100.0` or `1e-6` in a formula leaves the
reader to guess what it means, and the day the value changes, every copy typed
out by hand disagrees with the one that was edited.

- **In code** the literal appears only where the constant is defined. A value
  both sides use lives in `constants.py` and reaches the Lua through
  `gen_constants`; a value private to one module is a module-level constant
  there.
- **In a formula a person reads**, write the NAMES. Where the text is generated
  (a report, an Info text, a console line), give the value beside the name,
  formatted from the constant: `(VREF / airspeed)^2 with VREF = 17 m/s`, never
  `(17 / airspeed)^2`. Where it is static (a doc, a comment, a TODO check), the
  name alone — see *Numbers in comments*.
- **Not magic:** 0 and 1 as identities; the power or factor the maths itself
  states (a square, a half); universal unit definitions (per cent, ms per s,
  degrees per turn); and values in tests, which pin numbers on purpose.

## What the `p` and `x` tests are for

*Mechanism and consequences: → ARCHITECTURE.md § "The `p` / `x` tests".*

- `p` (point-balloon-chaser) isolates the control law and navigation by feeding the
  controller ground truth through the sensor's ORACLE interface — no camera,
  detector or gimbal encoder in the loop. `x` is the same test with the vision
  path connected. **Per C-03 that must remain the ONLY difference:** both call the
  same shared control function and differ solely in the source of the balloon
  world vector (oracle vs. sensor).
- **Bisect in this order:** a `p` failure is a control/navigation failure.
  Something that passes `p` and fails `x` is a SENSING failure — detector, gimbal
  tracking, encoder calibration, or estimator trust — not a gains problem.
- The scoring accessors (`flight_path_errors()` / `alignment_errors()`) are
  oracle-derived and FOV-free under `p` but detection-derived under `x`. They are
  NOT interchangeable between the two modes; under C-02, check which mode you are
  in before trusting them.
- **The alignment criterion is ONE angle, not two:** `hypot(pan_err, tilt_err)
  < ALIGN_TOL_DEG` on `alignment_errors()` — the FLIGHT PATH against the
  balloon — held 0.5 s, with no deadline: the `p` test reports when it aligned,
  or how close it came; NOT each axis under the tolerance, and NOT the nose. Single source of truth: `constants.ALIGN_TOL_DEG` and
  `controller.aligned_now()`, which every "aligned?" check reads — the `p`
  verdict, the align-seconds counter, `_resolve_aligned_seconds()`, the HUD's Path Aligned
  indicator and timer, the log's `in_tol` and the sweep's `ALIGN_TOL_DEG`. Never
  add a second definition. The flight path there is the FC's 50 Hz velocity from
  STATE2, never the 5 Hz MAVLink velocity.

## The rules of a chase

A chase flies a curve: a turn, then a straight line to the balloon. A
Suboptimal Curve is a failure. The rule forbids it, and an exception says
where the rule does not apply; they rank above every other rule of the
control law:

**The rule: every chase is flown without a Suboptimal Curve and ends lined up
with the balloon.**

**The exception: a balloon behind.** When the balloon starts behind the
aircraft, the rule does not apply: the chase may fly a Suboptimal Curve and
is not marked down for it. It still has to end lined up with the balloon, as
every chase does. "Behind" is more than a right angle (90 deg) away from the
direction the aircraft is moving at the start, measured from where it is
going, not from north. From level flight a balloon straight above or straight
below is exactly a right angle away, so neither is behind; a little past
either, toward the tail, is.

The exception exists because facing a balloon behind means turning the whole
direction of travel round. With the bank kept inside the limit, the quick ways
round are up and over the top - the top half of a loop, which comes out
heading back the way it came, upside down - or down and under, which ends
upside down the same way. Each is a Suboptimal Curve twice over: it ends
upside down, and its tilt goes to straight up or straight down and comes all
the way back to level. So without the exception every such chase would fail.
It is "may", not "must": a clean curve to a balloon behind still passes.

- **A Suboptimal Curve is one that needs any of:**
  1. a bank beyond the Bank Angle Limit either way - the Gain Tuner's
     `bank_lim_deg`, default `BANK_LIM_DEG` - measured as the wing tip's
     angle below the horizon; or
  2. a change of more than 5 deg in heading - the compass direction of the
     flight path - after the turn is complete; or
  3. the tilt - the flight path's angle above or below the horizon - coming
     back by more than the Tilt Back Travel (an operator setting, default
     2 deg) from the furthest it has reached in the direction it is tilting,
     at any point in the chase - tilted from level to 40 deg up, the path
     never again goes below 38 deg up; or
  4. the aircraft upside down, at any point in the chase: the belly-canopy
     axis meets the ground on the canopy side, so the canopy faces the
     ground. On a knife-edge, or flying straight up or straight down, the
     axis is level and meets the ground on neither side: not upside down.
     This item is not only for grading: Lift Vector Pointing steers by it
     in every chase and never rolls the aircraft upside down.

  The rule makes the curve a straight line to the balloon as soon as it can
  be one without exceeding the Bank Angle Limit.
- **The turn is complete** when the flight path first points within 5 deg of
  the balloon. From then on the heading stays within 5 deg of where it
  pointed at that moment, and the chase still ends lined up (the alignment
  criterion under "What the `p` and `x` tests are for").
- **Heading is undefined straight up or down.** Near vertical a tiny change of
  path swings the compass direction round, so a heading change is weighed by
  how far the path is from vertical, never read as a raw compass angle there.
- **Load is not limited, positive or negative.** The rules constrain the
  curve's bank and its shape, never how hard it pulls or pushes.
- **A start banked beyond the limit** is not a Suboptimal Curve for rolling
  back toward it; banking further than the starting bank is.
- **A start upside down** is not a Suboptimal Curve for rolling the right way
  up, through whatever bank that takes; turning further upside down than at
  the start is.
- **The bank is against the horizon**: never measure it as a rotation about
  the flight path, which reads a turn arriving on its wing tip as no bank at
  all. The wing tip's angle below the horizon reads level wings upside down as
  no bank, which is why being upside down is an item of its own.
- **Where the balloon ends is not judged.** A chase that starts with it above
  the canopy and ends with it below has overshot; that alone does not make
  its curve Suboptimal.
- **Every test of a control law keeps the rules of a chase**, wings level
  and banked up to 60 deg either way, at every angle off the path, and for
  the default `p` balloon from level.

## Verify against the exact ArduPilot version — never from memory

**This project runs ArduPlane V4.7.0-dev (2d99a5ea), commit
2d99a5ea648b651beedc8ee87033a0f56c6d04f4 of ArduPilot/ardupilot.** The
firmware banner's first line prints the version and short commit at startup;
read it rather than assuming, because it moves. The line after it is the
system ID (`send_banner()` in `GCS_Common.cpp`): on SITL the computer's
`/etc/machine-id`, 32 hex digits; on a board its name and chip serial number.
It identifies the machine, never the firmware or its Lua bindings, and the
flight log's head records it as `system id`.

If a change touches an ArduPilot internal in any way — a parameter's meaning, a
Lua binding's return shape, a controller's arithmetic, a mode's behaviour — open
that version's source and read it. Not the wiki, not a forum post, not a memory
of how it worked. `bindings.desc` for the Lua API, and that commit of
ArduPilot/ardupilot for everything else.

The wiki lags the code and forum answers describe whichever release the author
was on. Reasoning about what ArduPilot "probably" does is how a false answer
acquires the tone of a verified one.

### ArduPilot binding gotchas

Things that have actually differed from a confident guess. Read them before
touching the yaw-rate path, the speed scaler, `nav_scripting`, the airspeed, the
serial port, the desktop stand-ins, or any `boolean X'Null` binding.

- `boolean X'Null` in bindings.desc returns ONE value or nil, never a
  `(ok, value)` pair. `get_quaternion` and `get_velocity_NED` are the same
  shape; unpacking any of them as a pair silently yields nil.
- The speed scaler is applied SQUARED. `calc_speed_scaler()` in
  ArduPlane/Attitude.cpp returns `SCALING_SPEED / airspeed_EAS`, and
  AP_FW_Controller.cpp:55 feeds the rate PID `desired * scaler * scaler`.
- That scaler's clamp bounds are COMPUTED from `AIRSPEED_MIN`/`AIRSPEED_MAX` (this
  firmware's names; older releases called them `ARSPD_FBW_MIN/MAX`), with
  `AIRSPEED_MIN` floored at `MIN_AIRSPEED_MIN` = 5 m/s (defines.h) — not the
  fixed [0.5, 2.0] they resemble.  A `Parameter:init()` on a name the firmware
  does not have simply fails, so a renamed parameter is silently unbound unless
  the script checks and says so.
- On the `nav_scripting` path ArduPlane forces rudder to zero unless
  `yawController.rate_control_enabled()`. `KFF_RDDRMIX` and `YAW2SRV_*` feed
  `calc_nav_yaw_coordinated()`, which that path never calls.
- **A yaw rate reaches the surfaces only past THREE gates, and all three fail
  silently.** The `yaw_rate_dps` argument to `set_target_throttle_rate_rpy()` is
  discarded unless every one holds. There is no warning on any of them:
  1. `ACRO_YAW_RATE` — `commands_logic.cpp` applies
     `constrain_float(yaw_rate_dps, -g.acro_yaw_rate, g.acro_yaw_rate)`. The
     default is 0, and constraining to ±0 yields 0.
  2. `YAW_RATE_ENABLE` — `rate_control_enabled()` is `_rate_enable != 0`
     (AP_YawController.h). At 0 the rudder branch in Attitude.cpp is skipped and
     rudder is written as 0 every tick.
  3. `nav_scripting.run_yaw_rate_controller` — past gate 2 the rate controller
     runs only when this is true, else `yawController.reset_I()` runs instead.
     `nav_scripting_enable()` does NOT set it; only the mission-command timeout
     path and `set_rudder_offset()` do. A script claiming control through
     `nav_scripting_enable(MODE_ACRO)` must call
     `vehicle:set_rudder_offset(0.0, true)` itself to route a yaw rate at all,
     and again after any re-claim. Lift Vector Pointing passes `false` on
     purpose: no yaw rate, the rudder held at zero.
- `set_target_throttle_rate_rpy()` constrains the roll and pitch rates too, to
  ±`ACRO_ROLL_RATE` and ±`ACRO_PITCH_RATE` (`commands_logic.cpp`). A term added
  after the law's own clip - the pitch trim, the turn's pitch rate - is cut there
  whenever the law is at its envelope. The script makes the same cut before
  sending, so `lua_omega_x/y_cmd` log what ArduPlane flew; a new term on either
  axis goes in before that cut, never after it.
- `YAW_RATE_ENABLE` carries `AP_PARAM_FLAG_ENABLE`, so AP_Param SKIPS the whole
  `YAW_RATE_*` subtree during enumeration while it reads 0 (`AP_Param.cpp`,
  `_hide_disabled_groups`). Those 13 PID parameters do not exist on an FC that has
  never had it set, and all 13 appear at once when it does — a changed parameter
  set as far as the manifest gate is concerned. See
  `CONDITIONAL_GROUPS['YAW_RATE_ENABLE']`.
- A commanded yaw rate of 0.0 is NOT the same as zero rudder. Past all three gates
  it is an active instruction to hold zero yaw rate, which the rate PID chases with
  real rudder and which fights the natural yaw of a banked turn. It is a no-op only
  while a gate is shut.
- The yaw demand is the WHOLE body yaw rate the rate loop holds, not an addition
  to the yaw the aircraft has. Any demand short of the turn's own yaw rate, and of
  the yaw a roll needs to turn about the flight path rather than the nose, is
  fought with rudder, on the V-tail with deflection the elevator shares.
  `yaw_coordination_ff` supplies both (→ ARCHITECTURE.md § "The yaw axis:
  coordination and pointing").
- `airspeed_EAS` and `airspeed_estimate` are the SAME function; bindings.desc
  declares the second an alias. There is no second airspeed source to fall
  back on. It is a SINGLE-return binding: `local eas = ahrs:airspeed_EAS()`.
  Unpacking it as an `(ok, value)` pair puts the airspeed in the first variable
  and nil in the second, and every test against the second fails silently. With
  `ARSPD_TYPE` unset (a reboot-required parameter) the FC has no airspeed
  estimate and it returns nil.
- The serial port's write method is `writestring`, and `available()` returns a
  `uint32_t`, which arrives as userdata needing `:toint()`.
- `mount:set_rate_target` takes FIVE arguments (instance, roll, pitch, yaw,
  earth_frame); passing four shifts every argument left.
- Rotation: the FC path uses `G.quat_to_rotation_matrix` for BOTH directions
  (transposed for body->world), never `Quaternion:earth_to_body`. `ap_shim`'s
  `earth_to_body` is self-consistent, so a transpose mismatch against the real
  binding passes every desktop test; using one function both ways removes the
  possibility instead of testing for it.
- The desktop stand-ins `ap_shim.lua` and `ap_stubs.lua` are hand
  transcriptions. Where they REIMPLEMENT ArduPilot behaviour rather than record
  calls, a desktop pass proves nothing, and a stub under a name the firmware
  does not have lets every test pass against a method that does not exist.
- ArduPilot's Lua allows 100 locals in one function, the main chunk included;
  the desktop `lua5.4` allows 200. A script over 100 passes every desktop test
  and fails to load on the FC ("too many local variables (limit is 100)"), so
  nothing steers. Group related state into a table rather than adding a
  top-level local per value. `test_no_globals.sh` fails any FC script past
  `LUA_MAX_LOCALS`.

`test_lua_bindings.py` checks every method called against `bindings.desc`,
including return arity, argument counts and `uint32_t` conversion. Keep
`bindings.desc` in step with the firmware: `Companion/tests/bindings.desc` is
`libraries/AP_Scripting/generator/description/bindings.desc` at the commit
above, byte for byte. When the firmware moves, copy that commit's file over
it; never edit it by hand or take it from another release. A copy from a newer
release lists bindings this firmware lacks, and a call to one passes every
desktop check and is nil on the FC.

## Tuner edits are live in flight

A gain changed in the Gain Tuner reaches the FC on the NEXT TICK. There is no
stop-and-restart step and no "apply" button. → ARCHITECTURE.md § "Tuner edits are
live in flight" for how the push works, how it is delivered, and the measurement
exclusion.

**Never trim the FC's receive buffer before parsing it.** `rx_pump` in
`balloon_chaser_fc.lua` keeps what the previous tick left - the head of a frame whose
tail arrives next - and a trim to `RX_BUF_MAX` on a full read cuts that head off,
losing the frame: a gain edit as easily as a GUIDE, with nothing reporting it.
`test_control_path.py` sends an edit through the real tuner store, controller
push, link packing and `balloon_chaser_fc.lua`, and fails unless the FC's command
changes on the tick the edit lands; `test_balloon_chaser_fc.lua` fails if a frame split
across two reads is lost.

**Every frame to the FC goes through `LuaOffloadLink._send()`.** It writes a whole
frame under `_tx_lock`. GUIDE, the gain push and single gains come from different
threads; two unlocked writes that overlap interleave their bytes, both frames fail
the FC's CRC, and a lost `SET_GAIN` is never re-sent. Never write the port from
anywhere else.

**Any new mode that selects its own gains must go through `begin_measurement_gains`
/ `end_measurement_gains`.** That pair installs a private set for the duration of
a run so the live tuner cannot corrupt the measurement; reaching around it (forcing
values in place, sharing the operator's object) re-creates a silent two-writer bug.
Not re-entrant.

## The FC console is for faults, not for status

`gcs:send_text` from `balloon_chaser_fc.lua` is reserved for things that need a human
NOW: a fault (severity 3), a refusal to take control (4), an edge-triggered
change of ownership, and the startup banner. Nothing periodic: a repeating line
is soon ignored, and it crowds out ArduPilot's own STATUSTEXT, which shares the
budget.

Per-tick condition goes in the flight record instead, at full resolution. If a
periodic print seems necessary, the column it duplicates is missing: add the
column. *The columns that carry the FC's condition: → ARCHITECTURE.md § "The
flight log format".*

## Deliberate stops

The program halts on purpose in a fixed set of places, at startup and in
flight, each with a console message saying why. They are listed, with the
messages they print, in ARCHITECTURE.md § "Deliberate stops: where the program
halts on purpose" - the first place to look when the program has stopped.

- **A stop is loud and final.** An error in the control loop ends the program;
  never catch one to keep the loop running. Every in-flight stop goes through
  `run()`'s exit: hand-back to LOITER, everything closed, then
  `_halt_on_console` with the reason, waiting for Enter.
- **A new stop joins that list** in the same change, with the message it prints
  and what it means.

## Inactive integrators are cleared

Every integrator - the pan, pitch, roll-to-plane and yaw-pointing integrators,
Lift Vector Pointing's roll and pitch integrators, and the airspeed hold's - is
CLEARED, never held, on any tick its output does
not reach the aircraft: a gate, a release, a mode other than ACRO, a fault, a
gain of 0, a share of 0, a surface override.
Held, it adds a stale push the moment it is active again. A new integrator
follows the same rule. Freezing an integrator whose output IS still flown (at a
limit, or beside another law's share) is different and stays. → ARCHITECTURE.md
§ "Inactive integrators are cleared".

## Everything that participates in the control law is recorded

If a quantity feeds a control decision, it has a CSV column. Not the HUD - the
HUD is for the few numbers a human watches in flight - but the log, always.

A STATE or STATE2 field takes four steps, all of them or none:

1. write it in `balloon_chaser_fc.lua` (`st_msg` for STATE, `st2` for STATE2)
2. decode it in `lua_offload.py`
3. pass it in the CSV snapshot in `controller.py`
4. add the column and the row entry in `csv_logger.py`

Three of four yields a column of zeros that is indistinguishable from a real
measurement of zero. `test_lua_bindings` fails if any STATE or STATE2 field is
unassigned in `balloon_chaser_fc.lua`.

**An absent value is BLANK, never zero.** A reading the FC could not take, or a
law internal on a tick the law did not run, goes out as NaN, is decoded as None
and is written blank. A zero there reads as a measurement. A new law-internal
field goes out NaN on a tick the law did not run, and a new command field NaN on
any tick the FC did not command. An integer flag byte cannot carry NaN, so its
bits decode to None on a tick where they mean nothing — never to a confident 0.
*Which flag says the law ran, and when each flag byte decodes to None: →
ARCHITECTURE.md § "The flight log format".*

Gains and limits are constant within a flight and that is correct - they are
context. A PER-TICK quantity that never changes is an incomplete field.

**Every gain has a column, read from the set flying that tick.** Each
`ControlGains` field reaches a `g_*` column from `self._gains` (or `_pg`, the
pitch set resolved from it the same tick). Never from a copy taken at startup,
the operator's set while a measurement owns the gains, or a constant.
`test_csv_alignment.py` fails on a gain without such a column, so a new gain is
not finished until it is logged.

**The saved ArduPlane parameters are logged as the Gain Tuner holds them.** One
column per `ARDUPLANE_AUTOTUNE_PARAMS` name, spelled as on the FC, carrying the
value the FC last CONFIRMED (startup read, Read, Read All, or the readback after
a Write, a reset, a set Write or a mode switch) — empty until it answers or after
a write it did not answer, never a typed-but-unwritten value. Building a row must
never read the FC: a MAVLink round trip per parameter per tick stalls the 50 Hz
loop, and the tuner's Write, reset, Read, Read All and set Writes and each mode
switch are the only paths that change these values.

## The FC owns every value it computes. The companion never recomputes one.

*Why this matters, in full: → ARCHITECTURE.md § "The FC owns every value — why".*

The flight controller is the only authority on steering. If the companion needs
one of its values — for the HUD, the CSV, the tuner, plant-ID, anything — **that
value is put on the STATE or STATE2 frame and sent.** It is never recalculated on
the companion side from telemetry.

There must be NO Python code whose only job is to reproduce something the Lua
already computes. Not for the log, not for the HUD, not as a reference, not as a
cross-check. A second implementation of a flying law is not a diagnostic; it is a
second thing to keep correct, and the moment the two drift the log describes an
aircraft that does not exist. Any tool that fits, differentiates or integrates a
control quantity reads the `lua_*` mirror, never the companion's own. The Curve
Planner's model of each law's plan mode is the one copy of a flying law in
Python: it flies candidate plans the FC has not flown, to choose one, and never
stands in for a value the FC computed ("Lua steering offload").

Bandwidth is not a reason to break the rule. Send the value and raise the baud;
do not recompute it to save bytes. `port:begin()` in `balloon_chaser_fc.lua` must match
the pyserial baud in `main.py` exactly (both 921600).

The companion keeps only what has no FC equivalent: the camera, the detector, the
gimbal, the operator's inputs, and the estimator inputs it feeds forward in
`GUIDE`. Everything downstream of the steering law belongs to the FC. Its own
balloon geometry, for the HUD markers and the `p`-test scoring, is an independent
view of the outcome, logged beside the FC's (`lua_balloon_body_*`) and never standing
in for it.

## Every tuner row carries an Info button, and every bound is derived

**A row without an Info entry is not finished.** Anything that appears in
`TUNER_GROUPS` has an entry in `PARAM_INFO` saying what the quantity is, what
units it is in, and which direction makes it worse. A label alone tells an
operator holding a live aircraft nothing: `Yaw Coordination FF` does not say
whether it is a fraction or a rate, whether up is more coordination or less, or
what 0 does. Where the value has a meaningful default it also needs a reset —
`DEFAULT_CONST` maps the attr to its `constants.py` name, and without that entry
the reset button silently does nothing.

**Bounds are derived, never invented.** `PARAM_LIMITS` clamps every `set()`, so a
bound is a hard statement that no value past it can mean anything. Legitimate
grounds are exactly four:

- **a physical limit** — full rudder is 100%, because ArduPlane computes
  `rudder_offset_pct * 45` into a ±4500 output and there is no surface past it;
- **what the maths allows** — the sign it requires (a negative time constant is
  not a filter, a negative feedback gain inverts the loop), or the point past
  which it cannot mean more;
- **a documented fraction** — an anti-windup limit expressed as a share of the
  bank limit cannot exceed 1;
- **the field that carries it to the FC** — a value its GUIDE field cannot hold
  never reaches the aircraft, and struct's refusal to pack it breaks the control
  loop on that tick. Such a bound is read from the protocol through
  `guide_field_range()`, never typed as a number, so it moves with the field.
  Where a value is carried too narrowly for what it means, widen the field
  instead.

"Too high rings", "nobody would want more than that" and "the default is 1.0" are
NOT grounds. They are tuning results and defaults, and a ceiling built from one
of them removes the value the operator needs on the day the airframe disagrees
with the estimate. Where no real bound exists, write `None` and let the entry be
open in that direction. A feedforward trim whose correct size is 1.0 still has no
ceiling: over-driving it is a choice, not an impossibility.

An invented bound is worse than none, because it fails silently — the value is
clamped, the aircraft flies the clamp, and the tuner shows what was asked for.

**A row label carries UNITS, never a range.** `Yaw Point Limit deg/s` is a
label; `Yaw Coordination FF 0-1` would be a claim, and the label lies the moment
the bound moves. The only labels that may name a range are the ones where the range IS the
unit — an anti-windup limit expressed as a fraction of the bank limit is `0-1`
because that is what the number means, not because something clamps it there.
The Info text is where a recommended operating range belongs, said as a
recommendation.

**Choose the unit the quantity naturally has.** A sensor zero is a fraction of
gravity, so it is carried in g and scaled to m/s² once, inside the Lua. Carried
in m/s², its honest bound is ±9.807, which reads like an arbitrary number, and a
typed digit is worth 0.01 m/s² instead of 0.001 g.
Where a unit conversion is needed, do it at ONE place in the flight code, never
in the tuner or the display.

**Zero is a number, not a switch — and a lead time is not a `tau`.** A
parameter's value of 0 is interpreted by the maths that consumes it, never as a
hidden "disabled" flag. For a first-order low-pass with `alpha = dt/(tau + dt)`,
`tau = 0` gives `alpha = 1` — the filter passes its input through UNCHANGED, i.e.
INFINITELY FAST, not off. Writing "0 = disabled" next to a filter time constant
is therefore wrong, and a `if tau > 0` special case that skips the filter at 0 is
redundant at best and a lie about the semantics at worst: let the formula produce
the answer 0 already gives. The one place 0 legitimately means "no effect" is a
term of the form `x + rate*T` (a lead/prediction horizon), where `T = 0` really
is zero lead — but that quantity is a TIME added in the numerator, not a filter
denominator constant, so it must NOT be named `tau`. Reserve `tau`/`_TAU` for a
genuine time constant in a `1/(tau*s + 1)` denominator; name a lead or prediction
horizon `_S` (seconds) and describe 0 as "no lead", never "disabled". General
rule: prefer an explicit seconds name (`_S`) over `tau` wherever the quantity is
not literally a filter/plant denominator constant.

## Log the signal, never a presentation of it

A column carries the variable **as the code holds it**, at the point the code
names it. Never negate, rescale or re-reference a value on the way into the CSV
to make it read the way someone expects.

The reason is that the log is evidence about the code. A column that has been
adjusted for presentation is evidence about the adjustment, and the moment it
disagrees with a screen or a second column nobody can tell which of the two was
massaged. Every hour lost in this project to a signal that "looked wrong" was
lost to exactly that ambiguity.

So when a logged number reads with the wrong sign or in the wrong frame, the fix
is upstream, in the definition of the signal, and every consumer moves with it.
That is a real refactor with real risk and it is the only honest option: if the
convention is wrong for the log it is wrong for the law, and hiding it at the
logging step leaves the law wrong and the log lying.

Two things look like violations of this and are NOT — a control step is not a
presentation step, and two columns for nearly the same angle may legitimately
disagree. → ARCHITECTURE.md § "Log the signal … the two lookalikes".

## The flight log

*Format of the file: → ARCHITECTURE.md § "The flight log format".*

- **Adding a column:** append to `HEADER` and to the `writerow` list in
  `csv_logger.py`, both in the same order. `test_csv_alignment` checks the two
  stay the same length and that no column carries another's value. Append; never
  insert, so every existing column keeps its meaning.
- **Removing a column's source:** when the function that fills a column is
  removed completely, the log is adjusted in the same change: the column is
  deleted from `HEADER` and from the `writerow` list together, and the deletion
  is named in the commit and the summary. Every later column moves up one
  place, so a log is read by its header names, never by position.
- **Every line before the header is a comment.** The head's startup log and
  notes go through `_comment_lines`, which puts `# ` on each line of a
  multi-line message; an unmarked line there reads as a data row, and pandas or
  a spreadsheet import breaks on it. `test_csv_alignment` writes one and checks.
- **Analysing a flight:** read the flight log, and read the FC's mirrored
  internals (`lua_*`, `fc_*`) in it — not only the outcome columns. A
  diagnosis from the outcome alone is guesswork about the loop that produced it.

## FC configuration is set by hand and only CHECKED at startup

Every flight-controller parameter the system depends on is SAVED on the FC and
set BY HAND — MAVProxy `param set`, or, for the parameters ArduPlane AUTOTUNE
tunes, the Gain Tuner's Write button or reset icon. Startup never writes one and
never reboots the FC, and `shutdown()` makes no call — on SITL and real hardware
alike. A startup write would silently replace the operator's value on the next
boot, so a comparison flight would fly the old value while the operator believes
the new one is in. The one automatic write of a saved parameter is the mode
switch's rate-loop set (below), and it leaves the operator's values on the FC
when the program exits.

Startup CHECKS them instead. `read_fc_config` (`adapters/mavlink_link.py`) reads
each one with `_check_param` / `_check_rate_pids`, and `lua_setup.ensure_scripting`
does the same on the Lua path. A difference is recorded, never corrected;
`halt_on_param_mismatches` then lists every difference with its expected value
and HALTS until the operator presses Enter on the console (continue with the FC's
values) or Ctrl+C (stop). With no console to confirm on, startup refuses.

**The one write is what does not survive between runs:** the `MAVx_EXTRA1` /
`MAVx_RC_CHAN` stream-group rates, set every session in
`log_telemetry_rate_config`, because ArduPlane saves every `REQUEST_DATA_STREAM`
and a ground station overwrites them each time it connects. Reloading the Lua VM
when the scripts changed (`MAV_CMD_SCRIPTING`) is a command, not a parameter.
`--arduplane-autotune` is a separate maintenance mode whose job is to tune and
save gains, and is outside this rule.

**The Gain Tuner's Write and reset are the operator setting a value by hand, not
an automatic write.** `write_native_param` (`adapters/mavlink_link.py`) is
reachable only from the Write button or the reset icon on a
`constants.ARDUPLANE_AUTOTUNE_PARAMS` row
(ui → `ControllerAutotuneStore.write_native` → link): Write sends exactly the
typed number, reset the row's stored Acro default (`acro_default_value`, from
`constants.ACRO_DEFAULT_PARAMS`) or, for a parameter without one, the value the
FC reported at this program's startup (`native_params_at_startup`), or, refused
on the console, nothing — never a clamped value — and both confirm by reading the
parameter back. It, the Default Acro Parameters Write and startup finding the
Autopilot set (below) are the only places the operator's own values
(`native_params_operator`) change. No startup, shutdown, logging, measurement or
mode-switch path may call it.
`test_link_contract.py` fails if anything else writes a parameter, if anything
but the tuner reaches `write_native_param`, if anything reboots the FC, or if
shutdown changes a parameter.

**Each mode switch writes the rate loop that mode flies.** ArduPlane's own modes
run the same rate PIDs as ACRO, so the balloon chaser's tune would otherwise fly LOITER
and FBWA too. `_set_mode` (`adapters/mavlink_link.py`), which every mode change
in this code goes through, writes after the FC confirms the mode:

- ACRO → the operator's values: the startup read (the stored Acro defaults
  where startup found the Autopilot set), then each row's Write or reset, or the
  Default Acro Parameters Write.
- MANUAL (no rate loop) and AUTOTUNE (tunes these itself) → nothing.
- Every other mode → `constants.NATIVE_MODE_RATE_PIDS`, AUTOTUNE's result.

Rules that bind:

- **Only what differs is written,** measured against `native_params` — the
  startup read plus every write since — never by asking the FC first. A value
  the FC did not answer is unknown: it leaves `native_params` and is written
  again at the next switch.
- **The mode comes first.** A hand-back must reach LOITER without waiting on
  parameter round trips, and a failed write never undoes the mode change.
- **A caller flying the new mode on RC overrides passes them as `hold`.**
  `_set_mode` sends them the moment the mode is confirmed and at least every
  `HOLD_REFRESH_S` through the write; setting them only after `_set_mode`
  returns leaves the aircraft on the real sticks for the whole round trip.
- **Only the controller switches it on** (`enable_mode_param_sets`), after its
  startup read. The `--arduplane-autotune` maintenance mode switches modes through
  the same `_set_mode` and must never enable it: a write there replaces the gains
  AUTOTUNE is tuning.
- **The exit hand-back leaves the operator's values**
  (`go_to_loiter('shutdown', keep_operator_params=True)`). The FC saves every
  write, so whatever it holds at exit is what the next startup reads as the
  operator's values.
- **The Gain Tuner's two set Writes** (`PARAM_SET_ROWS`) write one whole STORED
  set from `constants.py`, never the tuner's boxes, every value, whether or not
  the FC is thought to hold it. Default Autopilot Parameters writes
  `NATIVE_MODE_RATE_PIDS` and leaves the operator's values alone. Default Acro
  Parameters writes `ACRO_DEFAULT_PARAMS` and makes it the operator's values,
  each the value the FC confirms or, with no reply, the default: left as they
  were, the next switch into ACRO would write back the very values it replaced.
  Both are the operator's command, reached only from the window
  (ui → `ControllerAutotuneStore.write_param_set` → `write_rate_loop_set`).
- Mode changes made outside this code (MAVProxy, the transmitter, ArduPlane's
  failsafes) write nothing.

`test_link_contract.py` fails if the set is written from anywhere but `_set_mode`
and the set Writes, if anything but the controller enables it or does so before
its startup read, if a switch writes a value the FC already holds, or if the exit
hand-back does not keep the operator's values.

A value written from the tuner is still CHECKED at the next startup: where
`EXPECTED_RATE_PIDS` lists the parameter and the two differ, startup halts for
Enter as for any other difference. Copying the value into `constants.py` is the
operator's decision.

**One stored Acro default per parameter.** `ACRO_DEFAULT_PARAMS` is
`EXPECTED_RATE_PIDS` plus the `PTCH_RATE_FLTE` / `RLL_RATE_FLTE` expected values: the
value startup checks, the value a row's reset writes and the value the Default
Acro Parameters Write sends are the same number. Never give any of the three its
own copy. The `PTCH2SRV_` / `RLL2SRV_` rows have no stored default; their reset
writes the startup value and the Acro set Write leaves them alone.

**The rate-loop settings** — every value plant-ID covers,
`<AXIS>_RATE_{FF,P,I,D,IMAX,FLTT,FLTE,FLTD,SMAX}` for pitch, roll and yaw — are
checked against `EXPECTED_RATE_PIDS` in `constants.py`, or, whole, the Autopilot
set (below). `PTCH_RATE_FLTE` and
`RLL_RATE_FLTE` are the exception: `read_fc_config` checks them against their own
5 Hz expected values, which differ from the report, and listing them twice would compare
one value against two. The plant-ID report only suggests values; setting them on
the FC and copying them into `EXPECTED_RATE_PIDS` is the operator's decision, and
no code reads the report into either.

**The FC holds one of two whole rate-loop sets, and startup passes either.**
`_check_rate_pids` passes the Acro set (`EXPECTED_RATE_PIDS`) matched in full,
and the Autopilot set matched in full: `NATIVE_MODE_RATE_PIDS` over the Acro
set's filters, which a switch out of ACRO leaves and the FC keeps when the
program stops without its exit hand-back. On the Autopilot set the operator's
values for those gains are `ACRO_DEFAULT_PARAMS`, never the FC's: adopting the
FC's flies the balloon chaser on the autopilot's gains, rate-loop integrators included,
and the exit hand-back saves them back for every later startup. Startup still
writes nothing; the first switch into ACRO does. A set matched only in part is a
difference like any other and HALTS. `test_link_contract.py` fails if either
whole set halts, if a partial one does not, or if the Autopilot set's gains
become the operator's values.

`PTCH_RATE_IMAX` and `RLL_RATE_IMAX` are 0 in the stored Acro defaults, which
ACRO flies, and `PTCH_RATE_I` and `RLL_RATE_I` 0 with them (an integrator clamped
to zero makes I inert): in the balloon chaser, pitch trim and any steady
aileron are held by the selected law's integrators - the cascade's pitch and
pan integrators, or Lift Vector Pointing's pitch and roll - not by the rate
loop's. ArduPlane's own modes fly `NATIVE_MODE_RATE_PIDS`, integrators
included.

**The manifest gate runs first.** Before any per-parameter work,
`verify_param_manifest()` fetches the FC's ENTIRE parameter table and gates on it
(name-set equality, then airframe identity — every output's `_FUNCTION` and
`_REVERSED`), raising on any mismatch. The servo output calibration
(`BUCKET_1B_OUTPUT_CAL`: `_MIN`, `_MAX`, `_TRIM`) and the contract values are then
checked as above: a difference HALTS for the operator, it does not abort. → ARCHITECTURE.md § "FC configuration and the manifest gate" for the
stages, the bucketing and why a gate rather than an advisory.

Rules that bind when touching this path:

- **When adding a parameter check, decide which bucket it belongs to.** If it
  identifies the airframe, it belongs in the manifest gate and ABORTS — a
  per-parameter check cannot express "this is the wrong aircraft". If the flight
  code depends on it, it is a `_check_param` that HALTS for the operator — never
  an advisory line that scrolls past, and never a write that makes it pass.
- **Identity checks read `_FUNCTION` and `_REVERSED`, never `_TRIM`.** A rudder
  trim of 1500 and a V-tail trim of 1500 are indistinguishable, so a `_TRIM` check
  passes on the wrong airframe; a reversed surface with the right function rolls
  the wrong way.
- **Do not silence a Stage 1 failure by deleting names.** A changed parameter set
  after a firmware upgrade is a real change worth a human look; rebuild the
  manifest from a known-good FC instead (see `fc_param_manifest.py`).

## Flight-control invariants

- **The actuation surface is roll rate, pitch rate, a yaw rate and
  throttle.** The FC script hands all four to ArduPlane through
  `set_target_throttle_rate_rpy()`, in deg/s and percent. Under the cascade the
  yaw rate is coordination plus pointing (pointing off while the Curve Flyer
  owns the chase); Lift Vector Pointing sends none and
  keeps the yaw rate controller off (`set_rudder_offset(0, false)`), so its
  rudder stays at zero. Nothing on the companion commands a surface. The V-tail mixes
  elevator and rudder onto SERVO2 (79, VTailLeft) and SERVO4 (80, VTailRight), so
  elevator and rudder demand share the same two surfaces and compete for the same
  deflection range. Most "yaw" in the code is a DESIRED bearing/direction setpoint
  (where to point the aircraft or the gimbal) rather than an actuation quantity;
  read the surrounding context before assuming which one a variable is.
- **Throttle is the operator's, never the pointing law's.** The operator
  sets `throttle_pct` in the GUI. With `AIRSPEED_CMD_ENABLE` = 0 the FC flies it
  verbatim; with 1 the FC's airspeed-hold loop (`compute_throttle_command` in
  `LUA/control_pitch.lua`) moves the throttle, and only the throttle, to hold
  `AIRSPEED_CMD_MPS`. What must NEVER happen: the pointing law computing or
  modulating throttle to meet a manoeuvre demand. Nor does the airspeed hold
  steer: it may not use pitch, roll, the steering error, the bank limit or any
  acceptance window to gain or shed speed, though it may monitor airspeed and
  altitude and adjust acceptance windows to the energy available - the pointing
  law owns the airframe and the speed loop owns the motor. Any change here is a
  throttle change ("Throttle changes are safety-critical"). `throttle_pct` flows: GUI cell →
  `self.throttle_pct` (or `SPECIAL_THROTTLE_PCT` during `p`, via
  `_effective_throttle_pct`) → `link.send_guide(throttle_frac)` → GUIDE
  `throttle_pct` → the FC.
- The Controller-Autotune inter-trial level-out (`link.level_and_hold`, called from
  `_straighten`) uses **FBWA** so the flight controller self-levels the wings, and
  VERIFIES straight-and-level from telemetry (roll angle + body yaw rate held within
  tolerance for `STRAIGHT_HOLD_S`). It commands the operator throttle (`throttle_pct`)
  on CH3 by RC override so airspeed is maintained, releases the override when the
  hold ends, then restores ACRO for the trial.

## plant-ID (`i`)

*plant-ID is a MEASUREMENT-ONLY subsystem (`i` key, `plant_id.py`); its full
design — the doublet excitation, the short-period estimator, the per-axis
rate-PID derivation and the altitude protection — is in `ARCHITECTURE.md`
§ "plant-ID design".*

`i` runs the SAME open-loop MANUAL doublet measurement on each of three axes in
turn — pitch (elevator), yaw (rudder), roll (aileron) — gated by the Gain-Tuner
Do Pitch / Do Yaw / Do Roll switches and sequenced with a top-level checkpoint so
a second `i` or a crash resumes at the next unfinished axis. Each axis is flown
in MANUAL with the FC rate loop OUT of the way, the short-period plant is fitted,
and the fit is mapped to the FULL ArduPlane rate PID for that axis
(`<AXIS>_RATE_{FF,P,I,D,IMAX,FLTT,FLTE,FLTD,SMAX}`) for the operator to review.
All three axes share ArduPlane's AC_PID rate loop — `AP_FW_Controller::_get_rate_out`
for pitch/roll, the math-identical `AP_YawController::get_rate_out` (used with
`YAW_RATE_ENABLE=1`) for yaw — and AUTOTUNE tunes the same variables on each, so
yaw emits `YAW_RATE_*` too, not facts only. FF, I, IMAX and the filters reproduce
AUTOTUNE's own formulas — never size the pitch/yaw I from FF alone: it also
re-trims the airframe between attitudes, which the doublets do not measure; P and
D are COMPUTED from the plant (bandwidth placement
plus critical damping), NEVER from AUTOTUNE's raise-until-ring doublet search,
which drives P the wrong way for the sustained balloon chaser slew. One operator
aggressiveness knob (`plantid_rate_agg`, default reproduces the flight-validated
`PTCH_RATE_P=1.0`) scales the goal bandwidth. It changes NO stored gains, and
applies nothing — FC parameters are set by hand (see "FC configuration is set by
hand and only CHECKED at startup").

**plant-ID reads the FC's own tick, never a simulator stream.** Each sample is the
FC's gyro and servo outputs from ONE STATE2 frame, stamped with that tick's
`fc_millis` (`lua_offload.lua_tick_sample`), so a surface position is always paired
with the rate measured on the same FC tick and the run works on the real aircraft.
Never mix in SIMSTATE or MAVLink `SERVO_OUTPUT_RAW`: they arrive at their own rates
on their own clocks.

Because it flies OPEN LOOP, no controller gains are in the loop to protect, so —
unlike the frequency sweep `k` — `i` does NOT install measurement gains. The
standing rule below is `k`'s (and any future closed-loop measurement's):

**measurement gains are PRIVATE to the run.** `k` installs its gains through
`begin_measurement_gains` / `end_measurement_gains` (see "Tuner edits are live in
flight"); the Gain Tuner keeps reading and editing the operator's own set
throughout, and the override is stated in the LOG, not forced onto the tuner in
front of the operator.

## Raw rate telemetry is hold-and-jump noisy — do not trust it

*Why, and the measured evidence: → ARCHITECTURE.md § "Telemetry rates".*

MAVLink rate telemetry arrives slower than it is requested and repeats unchanged
between arrivals, so it is a staircase rather than a signal. The steering loop does
not consume it; everything the companion reads over MAVLink (the HUD, the MAVLink
columns of the CSV) is subject to it. The FC's own readings, STATE2's sensor
columns, are taken on the tick the command was computed and are not.

Rules:

1. Never feed raw rate telemetry into a control term without low-pass filtering.
2. Characterize oscillations from the ANGLE, the COMMAND, or the FC's own gyro
   (`lua_gyro_*_rads`) - never from the MAVLink `roll_rate` / `pitch_rate` /
   `yaw_rate` columns.
3. `quat_fresh` is a PER-TICK flag (a new quaternion since the previous control
   tick), never a liveness window: a window reads a constant 1 and measures
   nothing.
4. The attitude arrival rate is measured at startup, never assumed: requesting a
   rate does not set it.

## POSITION is 5 Hz — anything derived from it must sample per FIX, not per tick

*The staircase and the fix, in full: → ARCHITECTURE.md § "Telemetry rates".*

`aircraft_world_pos_ned()` is built entirely from `GLOBAL_POSITION_INT`, requested
at 5 Hz and read by the control loop at 50 Hz — so it is bit-identical for ~10
consecutive ticks, then a step. The Lua offload does NOT fix this (ptest's oracle
bearing is still computed from it companion-side).

Rules:

1. **Sample per FIX, not per tick.** Anything that differentiates, fits, or
   integrates a position-derived quantity must gate on the data actually being
   new. Re-reading an unchanged value is not a second observation.
2. **Use `pos_time_boot_s`, not the wall clock.** It is the FC's own stamp for
   when the fix was TRUE, and doubles as the freshness key: a changed stamp means
   a new fix. Timing a fit by arrival smears the fixes' true spacing into the
   result.
3. **Read the stamp BEFORE the position it labels.** The telemetry thread writes
   the stamp last, so this ordering makes a fix landing mid-read pair a NEW
   position with the PREVIOUS stamp — read as "no new fix" and skipped, picked up
   correctly one tick later. The opposite order silently mistimes a sample.
4. **Fit in real time, not in sample index.** Fixes do not land on a perfect
   200 ms grid; the fit uses the Newton form through three non-uniform
   `(t, range²)` samples.
5. **Display values are exempt.** `_balloon_range_m` (the HUD "Balloon Distance" readout
   and its CSV column) is still refreshed every tick. Only the FIT is gated.

## The oracle balloon is PULLED at the tick, not pushed from the injector thread

*Rationale and measurements: → ARCHITECTURE.md § "Telemetry rates" (the oracle
pull).*

`SyntheticBalloonInjector` must NOT push a position snapshot to the sensor from its
own 50 Hz thread — two free-running 50 Hz timers beat and the controller reads a
stale or double-stepped position. It hands the sensor a callable
(`set_oracle_position_fn`) evaluated ONCE per control tick and cached, so every
consumer in a tick sees the same position. The leg is swapped as ONE tuple so a
reader can never pair a new `pos0` with an old `t0`; `_publish_position` remains
as a fallback for sensors without the hook.

## PID_TUNING: FC rate-loop internals

*What it is and a worked example: → ARCHITECTURE.md § "PID_TUNING".*

Startup checks `GCS_PID_MASK = 7` and requests `PID_TUNING`, whose `fc_*`
columns are the rate loop's own record of whether it tracks what steering asked
for. **PID_TUNING is the LAST stream to drop, not the first** — do not treat it
as redundant CSV filler.

## Project structure

*The directory tree: → ARCHITECTURE.md § "Project structure".* `main.py` lives at
the repo root, outside the `Companion/` package, and is covered by
`package_sha256()`. `LUA/` is also at the repo root, is NOT a Python package, and
is covered by `package_sha256()`. `Arduplane/` is at the repo root too: the
Gazebo world and model (`SDF/`) and the `AltitudeLiftDrag` plugin. It is
simulator input, not flight code, and its sources are NOT covered by
`package_sha256()`.

Import-depth rules that the structure must satisfy:

- Top-level package modules use single-dot imports (`from .constants import ...`,
  `from .geometry import ...`).
- Subpackage modules (anything under `ui/`, `adapters/`, `testing/`) reach the
  package top with two dots (`from ..constants import ...`, `from ..geometry import ...`).
- `controller.py` is the only top-level module that pulls in `ui/`
  (`from .ui.hud import ...`, `from .ui.controller_autotune import ...`). It and
  the runs that fly an injected balloon (`controller_autotune_sweep.py`,
  `plant_freq_id.py`, `trim_map.py`) pull in `testing/`
  (`from .testing.point_balloon_chaser import ...`,
  `from .testing.synthetic_injector import ...`). No top-level module imports
  `adapters/`; `main.py` builds the link and hands it in.

## Lua steering offload — the FC owns the control law

The steering laws - the cascade and Lift Vector Pointing, one selected for
every mode by `steering_law` - run on the flight controller as an ArduPilot
Lua script; `--lua-path` carries the link and is required (only
`--arduplane-autotune` runs without it). There is no companion copy of either
law.
The one piece of steering the companion computes is the PLAN: the Curve
Planner (companion) plans the turn of a chase under the rules of a chase and
sends it to the FC, where the Curve Flyer runs it and the selected steering
law flies it, closed loop.
The companion keeps the Curve Planner, computer vision, the gimbal, mode logic
and logging. This section is
the contract; where code and this section disagree,
one of them is a bug. *Mechanism, C-01/C-02 analysis, deployment detail, desktop
stubs, verification and measured rates: → ARCHITECTURE.md § "Lua steering
offload".*

- **What the companion may send the FC:** in `GUIDE`, SENSOR readings
  (`det_norm_x/y`, gimbal encoders, oracle bearing) and OPERATOR INTENT (mode,
  flags, throttle, straight/manual setpoints); CONFIGURATION in its own
  messages (gains via `SET_GAIN`, an excitation's DESCRIPTION via `EXCITE`);
  and the PLAN in its own message.
- **Outside the PLAN it must NEVER carry a control-law OUTPUT** — no rates,
  bank angles, pointing errors, integrators or phase. If the companion
  computed it and shipped it, the migration bought nothing.
  `Companion/tests/test_no_micromanagement.py` enforces this and fails the
  build; the PLAN message is the one exception it allows.
- **The PLAN describes the TURN only, as a bank schedule, never a stick
  schedule.** It carries bank angles, each tied to the turn's PROGRESS - how
  far the flight path has come from its angle off the balloon at the start
  toward pointing at it - in the measure of the steering law it was made for
  (the bank proxy under the cascade, the bank round the path under Lift
  Vector Pointing), and which law that is, under a plan number. The FC takes
  a PLAN only for the law it has selected. The Curve Flyer measures progress
  every tick from its own AHRS and the balloon direction, and hands the law
  the scheduled bank for that progress and the direction to turn the path
  toward; the law flies them in its PLAN MODE, with its own roll, pitch,
  rudder and bank limit. The FC works out no rule of a chase: the Curve
  Planner decides which chases it holds to the rule (`rule_binds()`, which is
  narrower than the rule: → ARCHITECTURE.md § "How the Curve Planner
  chooses"), and the law holds its bank limit in every chase. Progress, not time, indexes the schedule: an aircraft that turns
  slower than planned still meets each bank where the Curve Planner placed
  it. No roll or pitch rate travels in the PLAN, and the Curve Flyer works
  none out: the law does, every tick.
- **The turn ends when the flight path points within 5 deg of the balloon**
  (the turn's completion in "The rules of a chase"). There the law flies
  the chase on without the PLAN, or, with Transfer Control When Done
  (`cf_transfer_when_done`) off, the Curve Flyer aims it at the line the
  flight path had at that moment until the chase ends or the switch goes on;
  the Curve Planner plans nothing after it.
- **One Curve Planner, one Curve Flyer, for `p` and `x` only.** They hand
  the Curve Planner the balloon world vector from the oracle or the sensor,
  and the Curve Flyer reads the balloon direction the same way the law
  does; nothing else differs. Every PLAN is flown by the law selected, with
  the gains it flies without one. Straight and manual fly the law without a
  PLAN from their first tick, and a measurement or tuning run (`autotune`)
  flies the law it measures: the Curve Flyer never runs any of them. Neither
  reads FOV (C-02).
- **Leaving `p` or `x` hands over at once.** Any change of mode or of
  tracking is a fresh chase: the FC drops the PLAN and any held line on that
  tick and the law flies without it, whatever Transfer Control When Done
  says; the companion drops its plan too, so the next chase plans from
  scratch. So is a change of steering law.
- **A fresh chase HOLDS for its PLAN.** With the Curve Flyer on
  (`curve_flyer_enable`), a fresh chase flies straight - the law's roll and
  pitch rates held at zero, the pitch trim and the cascade's yaw coordination
  still applied - until a PLAN arrives, so the aircraft stays in the state the
  plan was made from. The Curve Planner plans from the FC's own state
  (STATE2) whenever the FC reports it waiting, and re-sends the PLAN until
  STATE2 echoes its plan number. With no PLAN within the Plan Wait
  (`cf_plan_wait_s`, an operator setting sent in `SET_GAIN`; none received is
  no wait) the FC flies the law without one; its integrators, cleared every
  tick its output was not flown, start fresh. Once the turn has ended the law
  flies on without the PLAN, or, with Transfer Control When Done off, holds
  the turn's line. No clock ends a turn, a run-in or its dive: each
  runs until its own condition ends it, however long that takes, and the Plan
  Wait is the only time limit the Curve Flyer has. A PLAN
  never outlives the chase it was made for: tracking off, or any fresh
  chase, discards it, and a PLAN arriving while the FC is not waiting, or
  made for the other law, is not taken.
- **The Curve Planner runs in a process of its own**
  (`curve_planner.PlannerProcess`), never on a thread of the companion. Its
  search is pure Python stepping small arrays and holds the interpreter lock
  almost throughout: on a thread it slows the 50 Hz control loop - GUIDE, the
  gimbal, the log - to a fraction of its rate for as long as a plan takes, and
  the plan by the same contention. The PLAN goes out the moment it is made;
  anything else worked out about it (the finer check of its turn time) comes
  after.
- **The Curve Planner flies every candidate through a model of each law's
  plan mode and of the Curve Flyer.** `PlanMode` and `fly()` in
  `curve_planner.py`, and the plan modes of `LUA/cascade.lua` and
  `LUA/path_attitude.lua` with `LUA/curve_flyer.lua`, are the same laws in
  two languages, from the same constants (`constants.py`) and gains: a
  change to either side is a change to both, in the same commit, or the FC
  flies turns the planner never made. `test_balloon_chaser_fc.lua` section 16
  flies the planner's PLANs through the real FC script under each law, on
  the planner's own aircraft, and fails when a turn's time drifts from the
  planner's prediction.
- **The FC must not INVENT configuration the operator owns.** Every gain the
  law reads travels from the companion in `SET_GAIN`; hardcoding one on the FC
  makes it fly a different controller than the companion shows. A new operator
  setting travels from the companion the same way.
- **GUIDE goes out EVERY tick, tracking or not.** The FC releases control when no
  GUIDE with a new `seq` has arrived for `GUIDE_STALE_MS` (gate 7), so a frame
  that stops when tracking stops hands the aircraft back mid-chase, and one that
  stops between chases leaves it the previous balloon on the next ACRO entry.
  With the frame always flowing, its tracking flag is the live instruction.
  `test_link_contract.py` fails if `run()` guards `send_guide` with tracking.
- **The flight path is taken once per tick, as read.** `track_world_balloon`
  works it out from that tick's AHRS reading before anything reads it, with no
  filter, glitch guard or clip of the script's own; the only fallback is the
  nose at or below `FLIGHT_PATH_MIN_SPEED_MPS`. The laws aim the ESTIMATED path,
  so any lag, held reading or clip is flown as flight-path error. Smoothing of
  attitude and velocity belongs to ArduPilot's estimator (`EK3_TAU_OUTPUT`). A
  SITL result that depends on estimator behaviour counts only with
  `AHRS_EKF_TYPE` = 3; at 10 the script reads the simulator's truth.
  → ARCHITECTURE.md § "The flight path is taken once per tick, as read".
- **Every balloon error but roll-to-plane's clock is read in the path frame.**
  The balloon is expressed once per tick in the frame along the path
  (`track.lua`), and bank-to-turn's and yaw pointing's `pan_err` and the pitch
  error are each one reading of it. Never measure one of them from the nose,
  from a second copy of the path, or with a formula of its own. Roll-to-plane's
  clock is the one exception: it is read round the nose, in body axes, so the
  two roll laws share a zero only while the path lies in the body's plane of
  symmetry, and near the nose, where the clock swings for small moves,
  roll-to-plane rolls for every swing. Lift Vector Pointing reads the same
  frame, its angle off the balloon from the path led `att_lead_s` ahead (at
  0, the path as read): the lead is part of its guidance, not a second copy
  of the path. Reading roll-to-plane's clock round the path, like the
  other errors, is the tempting change: for a balloon between the nose and the
  path that clock asks for the opposite roll, which tilts the pull the wing is
  holding away from the balloon (→ ARCHITECTURE.md § "Roll-to-plane").
  `test_balloon_chaser_fc.lua` sections 5h and 10 pin it. → ARCHITECTURE.md §
  "What each error is measured from".
- **The script must CLAIM control, and narrowly.** `set_target_throttle_rate_rpy()`
  only stores the demand; ArduPlane uses it only while `nav_scripting_active()`.
  Claim ONLY in ACRO and ONLY while GUIDE says tracking, and re-claim every tick
  (`Mode::enter()` clears the flag). Any mode that is not ACRO, and any GUIDE with
  tracking clear, must trigger an immediate explicit release
  (`nav_scripting_enable(255)`) — `nav_scripting_enable()` also accepts LOITER, and
  claiming it hijacks the mode this project aborts into; merely not calling leaves
  ArduPlane flying the last command until its own 1000 ms timeout.
  `test_balloon_chaser_fc.lua` fails if the script keeps control in either case.
- **Every stop of the law releases control on that tick.** A gate, a stale
  GUIDE, a Lua error: no path returns from `steering()` while the script still
  owns the aircraft. Losing an input the law was flying on, and any Lua error,
  is a FAULT: latched in STATE's flags bits 5-7 (`protocol.FAULT_NAMES`), no
  claim while latched, cleared only by a GUIDE with tracking off. A new gate
  goes through `drop_control()`, which does both. → ARCHITECTURE.md § "The
  script must CLAIM control, and must claim it narrowly".
- **STATE's and STATE2's `seq` echo the GUIDE `seq` they were computed from**,
  never an FC counter: the `p` test's start gate, the round-trip lag and the
  pairing of a row's two frames all read the echo (→ ARCHITECTURE.md § "Measured,
  not assumed").
- **A frame's payload is capped at 255 bytes** by its one-byte length field.
  STATE is near the cap; a field that does not fit goes in STATE2, sent right
  after STATE every tick - never squeezed in by dropping, quantising or
  decimating another. `test_link_contract.py` checks both payloads against the
  cap and both frames together against the line rate read from both ends of the
  link.
- **Deployment:** deploy modules to `scripts/modules/` only (`require()` searches
  nowhere else); `lua_deploy` reads `/proc/<pid>/cwd` and refuses to deploy
  elsewhere, checking before it copies anything; send `MAV_CMD_SCRIPTING`
  STOP_AND_RESTART exactly ONCE (a second STOP mid-rebuild parks the VM
  forever). An entry script renamed away stays in
  `RETIRED_ENTRY_SCRIPTS` (`lua_deploy.py`), which the deploy deletes from
  `scripts/`: left there, ArduPilot runs it beside the current one, two scripts
  on one serial port each claiming the aircraft.
- **The source fingerprint covers `.lua`** (excluding the desktop-only stubs
  and test drivers that never reach the aircraft), the repo-root `main.py`
  and the `.md` documents.

## Two autotune subsystems — keep them distinct

There are two unrelated "autotune" concepts in this repo (plus the
measurement-only Plant-ID). **Never conflate them; use these names in code,
comments, and docs.** *What each one does in detail: → ARCHITECTURE.md § "The two
autotune subsystems".*

- **Arduplane Autotune** — commands the AUTOPILOT's built-in AUTOTUNE (ArduPlane's
  RATE-PID tuning) over MAVLink. File: `arduplane_autotune.py`, launched via
  `--arduplane-autotune`. Identifiers like `AUTOTUNE_LEVEL`, `AUTOTUNE_AXES` and
  mode `AUTOTUNE` inside that file are ArduPlane's real param/mode names — **do
  not rename them.**
- **Controller Autotune** — *our* controller-gain tuning: the interactive gain
  tuner (`ui/controller_autotune.py` + `controller_autotune_store.py`) and the
  automatic sweep (`controller_autotune_sweep.py`, the `'t'` key). SIM only.

### When to run `i` (Plant-ID) vs `t` (Controller-Autotune sweep)

**`i` measures the airplane; `t` tunes the software.** The dependency is one-way —
physics upstream, gains downstream. *Full reasoning: → ARCHITECTURE.md § "When to
run `i` vs `t`".*

- **Run `i`** when an airframe RATE plant changed or was never measured (new
  airframe/model; prop/thrust, weight/CG or cruise-throttle change; after
  ArduPlane AUTOTUNE). It yields the per-axis feedforward (`PTCH_RATE_FF`,
  `RLL_RATE_FF`) and the overshoot floor — not the pan authority.
- **Run `k`** (frequency sweep) when the pan authority (bank→yaw) changed: its
  numbers are what `plant_id_rescale` carries the pan gains from sim to the real
  aircraft with. **`k` refuses to start** until its excitation is carried to the
  FC (`EXCITE`): it adds to the roll law's output, and the roll law runs on the
  FC.
- **Run `t`** when the controller's world changed (code changes to the
  laws/cost/decision rules; hand-edited gains, a rescale from `k` included;
  periodic checks).
- **Measurement before tuning, always** — a sweep flown before fresh measurement
  tunes an airplane you no longer have.
- **Real aircraft ordering:** ArduPlane AUTOTUNE → real `i` (rate feedforwards)
  and `k` (pan-authority) → `plant_id_rescale` on the `k` numbers → bake derated
  values → hand-polish in the manual tuner. The sweep stays SIM-only until
  validated for real flight.

### Controller-Autotune stop/resume

*Mechanism and HUD timers: → ARCHITECTURE.md § "Controller-Autotune stop/resume
and HUD timers".* The sweep checkpoints after every flown grid cell and resumes
automatically after a crash, Ctrl+C or `'t'` abort. **There must be NO separate
save step on the abort path:** an explicit abort-time `_save_checkpoint` clobbers
the live in-progress matrix with `None`.

## Mode transfer: two canonical paths, nothing else

Every change to the FC's flight mode goes through exactly one of two controller
methods. Nothing calls `link.set_mode_acro()` / `set_mode_loiter()` directly
except those two methods.

> **The Lua script is a THIRD actor that must not become a fourth path.** It
> calls `vehicle:nav_scripting_enable(ACRO)` every tick to hold control, and
> `nav_scripting_enable(255)` to release. That is NOT a mode change — it never
> calls `set_mode` — but it decides who owns the surfaces, so it obeys the same
> discipline: claim only in ACRO while GUIDE says tracking, release immediately
> otherwise. The script must never issue a mode change of its own; recovery
> belongs to the operator and the failsafes.

- **`enter_acro(reason)`** - "the controller is flying it". Commands and
  confirms, then writes back the operator's rate-loop values the FC does not
  hold; the exception PROPAGATES, because a persistent refusal must stop
  the caller loudly rather than let it track in a mode the script will not
  claim - the aircraft would fly that mode, not the balloon chaser.
- **`go_to_loiter(reason)`** - "hand the aircraft back". tracking off first (the
  next GUIDE says so and the script releases on that tick), LOITER commanded and
  confirmed and `NATIVE_MODE_RATE_PIDS` written (the operator's values instead on
  the exit hand-back, `keep_operator_params=True`), then **RC overrides released
  in a `finally`** so they are released even when the mode change times out. The
  overrides are the level-out's; a latched one with no mode
  change leaves the FC honouring the last stick values with nothing commanding
  it.

Both BLOCK (up to 30 s, waiting on HEARTBEAT, plus the parameter round trip), so
neither may be called from the 50 Hz control thread. The ptest / plant-ID / sweep teardowns already run on
worker threads; the keyboard path uses the `_go_to_loiter()` wrapper, which is
spawned on its own thread and clears `_tracking_transition` afterwards.

Setting `self._mode` is NOT finishing: it is the HUD label and nothing else. A
finisher that only sets it leaves tracking set, so GUIDE keeps telling the script
to fly the balloon chaser, with the HUD reading "Mode: Loiter" throughout. The trap is written out at
`ptest_finish`, which owns it.

**Link-layer symmetry.** `set_mode_acro()` must NOT skip the command when the
cached `current_mode` already read 4. That cache is fed by HEARTBEAT at ~1 Hz,
so it can be stale by up to a second - long enough to skip a command the
aircraft needed after a mode change it had not yet reported. It commands
unconditionally, like `set_mode_loiter()`; the cost is one MAVLink message,
because `_set_mode`'s confirm loop exits on its first check when the FC is
already there.

**Sensor housekeeping is part of every transition — do not skip it.** Leaving a
mode must leave `sensor.active` False unless `'l'` vision tracking is genuinely on. The
HUD derives `vision_tracking` from `sensor.active` and draws the red detection
circle and its ray from that, so a transition that leaves the sensor on paints a
vision balloon onto a mode that has none.

The trap is that `sensor.active` is **not** evidence of `'l'` vision tracking. `'p'` and
`'x'` switch the sensor on themselves, and `toggle_vision_tracking` refuses to run in
those modes — so inside `'p'` and `'x'` `'l'` vision tracking cannot be on, and an active
sensor there is always mode state. Any path that preserves tracking state across a
transition must therefore test both:

```python
vision_was_active = (self.sensor.active
                   and self._mode not in ('balloon_chaser', 'ptest'))
```

`handle_arrow` and `toggle_balloon_chaser` both snapshot and restore the vision tracking's
state (filtered estimate, gimbal-lock flag) so `'l'`
vision tracking survives an arrow-key turn without a re-converge transient. That
restore is correct only under the test above; with the bare `sensor.active` it
also fires when leaving `'p'` or `'x'`, undoing the teardown that had just run.

This correctness lives in each exit path rather than in one place, which is the
accepted cost of having a single sensor-active fact instead of a second
lock-requested flag that could drift out of step with it. **A new mode, or a new
way of leaving one, has to honour this itself.** A new `_mode` string also needs
its GUIDE code in `controller.MODE_CODE`, read without a fallback;
`test_link_contract.py` fails on a mode the code sets that is missing there.

A mode key pressed during a run in the `'autotune'` regime switches straight to
its own mode: the run stops with its own abort and its `autotune_finish` hands
over in ACRO (`_hand_over`) instead of LOITER; the control loop starts the new
mode (`_service_switch`) only after the run's thread has ended.

## Fixed values no sweep may touch

The bank limit (`bank_lim_deg`, default `BANK_LIM_DEG`) and the sweep cost
weights are FIXED as far as any sweep is concerned. The operator sets the bank
limit by hand in the Gain Tuner; no sweep may search over it or the weights.

The bank limit sets control AUTHORITY, not safety. A sweep free to lower
it will, because a smaller limit produces less overshoot — buying score by
spending the authority the aircraft needs. Load-factor protection is
`apply_load_factor_roll_limits()` in ArduPlane and is not this value's job.

`COST_W_TLOCK = 3`, `COST_W_OVERSHOOT = 8`. That ratio decides which gains every
sweep selects, so changing it silently re-decides past results. Time to line up
is a flight cost — height spent lining up is height unavailable for the dive —
and overshoot overlaps with ring, which counts oscillations directly.

Evidence for both sits with the values: `constants.py` and
`controller_autotune_sweep.py`.

## Tunable bounds

`PARAM_LIMITS` in `controller_autotune_store.py` bounds every continuous
tunable, enforced in `ParamStore.set()` - the single choke point every setter
goes through (tuner, per-row reset, reset-all, the sweep). Bounding there bounds
all of them. Discrete params are NOT listed: they cycle an explicit allowed
list and cannot leave it.

Bounds are DERIVED, never invented. Each is a documented fraction
(`windup_limit` is the fraction of the authority envelope the integrator may
own, so >1 lets the integrator alone exceed the full envelope), a physical limit
(bank beyond 90 deg is inverted flight), what the maths allows, or the capacity
of the field that carries the value to the FC (see "Every tuner row carries an
Info button, and every bound is derived"). Where no real bound exists the
default is `(0.0, None)` - non-negative only, because every gain, time constant
and rate limit here inverts or destabilises its loop if negative, and a NEGATIVE
leak GROWS the integrator without bound.

**Two failures, two answers.** FINITE but out of range is CLAMPED to the
nearest bound and logged at WARNING - the operator meant a real number and
overshot, and refusing mid-flight leaves them unsure what the parameter even
is. NON-FINITE or unparseable (NaN, inf, `1e999`, text) is REFUSED at ERROR,
keeping the current value: there is no "nearest bound" to a NaN, and both
tempting fallbacks are dangerous - clamping +inf down to `lo` turns a runaway
gain into ZERO (a `bank_lim` of 0 is no roll authority at all), while letting
it through poisons every downstream state unrecoverably. A finite value past
`FLOAT32_MAX` is refused the same way: `SET_GAIN` carries every gain as a 32-bit
float, which can only hold it as infinity - and struct refuses to pack it,
breaking the control loop on that tick.

**No value the store accepts may fail to reach the FC.** Every accepted value
must fit the field that carries it - its GUIDE field, or `SET_GAIN`'s 32-bit
float. `test_tuner_registration.py` checks each GUIDE-carried row's range
against its field and types huge numbers into every row; a new tunable that
travels to the FC is covered the moment it is added.

The info popup appends the enforced range to every continuous tunable's text,
so the bound is visible BEFORE typing rather than discovered afterwards.

No parameter gets an inline clamp in `_on_entry_commit` - that protects one
parameter and leaves every other one unguarded. There is one mechanism.

## Sweep / plant-ID operator display

*Detail: → ARCHITECTURE.md § "Sweep / plant-ID operator display" and
§ "Sweep report: cells that never converged".*

**The HUD mode line is display-only.** plant-ID deliberately runs as
`_mode = 'autotune'` so that string drives CONTROL behaviour and **must not
change**. `_mode_label` overrides only what the HUD prints, so `i` reads "Mode:
Plant ID" while everything downstream still sees `autotune`. Cleared when the run
ends.

## Measurement and tuning runs follow one pattern

*The pattern, piece by piece: → ARCHITECTURE.md § "Measurement runs: the shared
pattern" and § "Tuning-run outputs".*

- **A new measurement or tuning run follows the shared pattern whole.** One key
  starts and stops it; each step is self-contained (altitude protection,
  level-out, the step); samples are the FC's own ticks; a closed-loop run flies
  a private gain set (`begin_measurement_gains`); a checkpoint is written after
  every finished step; the hand-back goes through `autotune_finish`. Plant-ID
  (`i`) and the trim map (`e`) are the reference.
- **A run emits exactly three kinds of file, whatever sub-parts it ran:** one
  report (`*_report_*.txt`, any suggested parameters folded in as a paste-ready
  block), one checkpoint (`incomplete_*_run_*.json`), and one signals CSV with
  every sample tagged by the sub-part it came from (the trim map's is its log)
  — never a file per sub-part. Each has a Gain-Tuner export row named after it,
  and the report is written whatever happened: on completion, a stop or a
  failure. The sweep (`t`), the single-cell tune (`y`) and `k` still write
  their own sets (`TODO.md` § 8).

## Completed-run outputs are kept; only the checkpoint is cleaned

*Mechanism: → ARCHITECTURE.md § "Completed-run retention".*

Every measurement/tuning procedure — plant-ID (`i`), frequency-response ID
(`k`), the Controller-Autotune sweep (`t`) and the single-cell tune (`y`) —
writes its results to TIMESTAMPED files and KEEPS them. Starting a new run must
NEVER delete a previous run's completed output. The one thing a new run may
clean is its own resume CHECKPOINT.

- **A completed output is timestamped and kept.** The report, the raw-signals
  CSV, the results JSON, the per-cell CSVs and the suggested-constants file all
  carry the run stamp (`file_export.stamped`), so a rerun writes a NEW file
  beside the old one instead of colliding. The writers do NOT call
  `file_export.claim()` on these; the files accumulate until the operator clears
  them with the Gain Tuner's Delete (or Delete All).
- **Only the checkpoint is claimed.** Plant-ID clears the previous checkpoint
  with `claim()` in `_run_ckpt_save`; freq-ID and the sweep keep one checkpoint
  by overwriting a fixed run-stamped name and deleting it on a clean finish.
  That is resume state, not a result — the one file a new run is allowed to
  remove.
- **The export copies one run, never a mix.** A single-file output exports its
  newest (`pick='latest'`). A run that emits a SET — freq-ID's per-cell CSVs —
  exports with `pick='run'`: every file sharing the newest run stamp and no
  others. Delete still clears every run on disk.

The trap this guards: a `claim()` at a completed-output write silently deletes
the previous run the instant the next one starts, so a comparison the operator
meant to keep is gone before they notice. Not the flight log: `flight_log`
(`q`) is truncated on each new recording by design and is out of this scope.
`test_export_gazebo.py` pins both halves — no writer claims a completed output,
and the plant-ID checkpoint claim survives.

## Camera view switches (display only)

*Mechanism and measured costs: → ARCHITECTURE.md § "Camera view switches".*

Two Gain Tuner rows control the OpenCV windows: `Balloon Chaser Cam Window`
(`nose_cam_display`, DEFAULT 0 = off) and `Gimbal Cam Window`
(`gimbal_cam_display`, DEFAULT 1 = on).

- **Neither can affect control.** Detection runs off the GIMBAL camera; the nose
  camera is display-only, and the gimbal camera keeps decoding regardless of its
  window.
- **Both views off is a legitimate configuration**, so the KEYBOARD must survive
  it: the Gain Tuner hosts the key path (a bounded `deque` drained when
  `cv2.waitKeyEx` returns -1), and `waitKeyEx` is still called unconditionally to
  pump any open window's event loop. Arrow keys are matched on its full code,
  never the low byte: under GTK an arrow's low byte is capital `Q`..`T`.
- **Threading.** OpenCV's window calls (`namedWindow`, `destroyWindow`,
  `imshow`, `setMouseCallback`, `waitKey`, `waitKeyEx` and the rest of HighGUI)
  are not thread-safe and run only on the control loop's thread, the one that
  pumps HighGUI's event loop and sends GUIDE. They live in exactly five
  functions: `BalloonChaser.run()`, `_apply_camera_views()` (the SOLE owner
  of the window lifecycle), `render_nose_camera()`, `render_gimbal_camera()` and
  `KeyboardHandler.handle_key()`. `run()` is the only caller of the other four,
  never through a thread, a lambda or a nested def, and `main()` calls `run()`
  itself. The Tk tuner thread only ever writes two ints. A new window call goes
  in one of the five; adding a sixth function to the list in
  `test_hud_contract.py` needs the same guarantee. That test fails on a window
  call anywhere else, on any other caller of the four, or on any of them (or
  `run()`) as a thread's target.

## The simulator flies in ArduPilot's air

*Why, and how the plugin works: → ARCHITECTURE.md § "Simulator air:
AltitudeLiftDrag".*

In SITL the forces come from Gazebo and the airspeed reading from ArduPilot, and
the two must describe the same air. ArduPilot's reading assumes air that thins
with height; a wing flown in one fixed density disagrees with it by more the
higher the aircraft flies, and anything fitted against the reading (the pitch
trim, a feedforward, a plant) then holds only at the height it was fitted at.

- **Every lift-drag element in `Arduplane/SDF/model.sdf` is `AltitudeLiftDrag`**
  (`Arduplane/AltitudeLiftDrag/`), never stock `LiftDrag`, and none carries an
  `<air_density>`. A new lift-drag element uses it too.
- **Its atmosphere is ArduPilot's own**: the 1976 table and formula of
  `AP_Baro::get_air_density_for_alt_amsl` at the firmware this project runs.
  Never a different model, not even a more accurate one: any difference reopens
  the gap. When the firmware moves, re-read that function and bring the table
  with it.
- **The world's `<spherical_coordinates><elevation>` equals ArduPilot's home
  altitude** (the SITL location, `-L`/`--home`). A difference between the two is
  a height error in the air.
- **A term for the propeller's push carries the density ratio**
  `1 / EAS2TAS^2`, on the FC and in any fit of it. At one throttle the motor
  holds about one speed, so its push follows the air density; the wing's terms
  need no factor, because EAS already carries it. Without it a gain fitted at
  one height is wrong at another - in SITL and on the real aircraft alike.
- **Its aerodynamics stay upstream `LiftDrag`'s.** The file follows the upstream
  source line for line, upstream comments included, so a diff against it shows
  only the parts marked `talon_sim`. The comment-style rules above apply to
  those parts, not to the upstream text.

`test_export_gazebo.py` fails on a stock element, a fixed density, a world
without an elevation, or a table or constant that differs from ArduPilot's.

## Gain Tuner GUI conventions

The Tk tuner (`ui/controller_autotune.py`) is flown with, so it is read at a
glance under load. These rules are pass/fail for anything new added to it.

1. **The GUI is not a log.** Never render transient per-action results (copied
   / saved / done / failed messages, progress text, counters that exist only to
   confirm something happened) into the window. Print them to the CONSOLE with
   the normal `log` calls. Persistent STATE — a value, a switch position, a
   live timer — belongs on screen; a report about a thing that just happened
   does not. The file-export buttons are the reference case: every outcome,
   success and failure alike, goes out through `file_export`'s own logging and
   the window shows nothing.

2. **Anything new must line up with what is already there.** Rows put their
   label in a fixed-PIXEL cell (`_label_cell`), so every value widget — entry,
   switch, action button — starts at the same x. Do not hand-place a new
   control, and do not size a label in characters: `TkDefaultFont` is
   proportional, so a character count clips wide labels (that is what covered
   the right edge of "Autotune Maneuver (0 step / 1 doublet)"). Action rows use
   the same `_label_cell` as tunable rows for exactly this reason.

3. **A new tunable must fit, and the window grows if it does not.**
   `_measure_columns()` measures every label in `TUNER_GROUPS` plus the export
   labels in the real font, sizes the label column to the widest, adds the
   widest value widget and the icon columns, and sets the window width from the
   result. Adding a longer label than any existing one therefore widens the
   column and the window automatically — nothing to tune by hand. If a change
   ever needs a wider window, widening it is APPROVED; a clipped or overlapped
   control is not an acceptable trade.

4. **Every row carries an info button, and every resettable row carries a reset
   button.** No exceptions for "obvious" parameters. Tunables take their text
   from `PARAM_INFO[attr]`; rows that are not tunables pass explicit text to
   `_make_info_button(..., text=...)` — the export rows compose theirs with
   `file_export.info_text()`, which states what the file is, where it lives on
   the WSL side, and where the button copies it to. The per-row reset icon is
   added by `_make_reset_button` for tunables. ACTION rows (buttons that do a
   thing rather than hold a value) get an info button but NO reset, because an
   action has no value to restore. The saved ArduPlane parameter rows carry
   Write, Read, an info button (`native_param_info`) and a reset icon. Their
   value lives on the FC, so the reset WRITES the row's stored Acro default
   (`acro_default_value`) to the FC, or, for a parameter without one, the value
   the FC held when this program started (`native_startup_value`), and sends
   nothing when there is neither. Read is how the box returns to the FC's
   current value. The rows below them are ACTION rows, each one button and an
   info button, no reset: All Parameters (Read All, `READ_ALL_INFO`), then the
   two set Writes, Default Autopilot Parameters and Default Acro Parameters
   (`PARAM_SET_ROWS`, `param_set_info`).

5. **A round trip to the FC never runs on the Tk thread.** Write, reset, Read,
   Read All and the set Writes are MAVLink round trips of up to seconds; they run on worker threads
   that touch no Tk object, and `_poll_values` shows the result. A box whose
   number is not the FC's confirmed value is drawn in `NATIVE_PENDING_FG` —
   pending state, on screen; the outcome of the write itself goes to the
   console.

## HUD text conventions

*The full magenta-column / timer layout is in `ARCHITECTURE.md` §
"Controller-Autotune stop/resume and HUD timers".*

**No HUD text is ALL-CAPS.** Labels and values are title-case (`Straight`,
`Path Aligned`/`Nose Aligned`, `Autotune`, the `Mode:` value, the
`Alt protect:` status). Standard abbreviations/units (`N/A`, `m/s`, `deg`, `AoA`)
are deliberately left as-is.

## FOV measurement (diagnostic, default OFF)

*How it works: → ARCHITECTURE.md § "FOV measurement (diagnostic)".*

A background estimator (`FovEstimator`, activated by the Gain Tuner switch
`FOV_EST_ENABLE`, default 0) measures the gimbal camera's horizontal field of view
from motion already happening.

**It is DIAGNOSTIC ONLY, and that boundary IS C-02.** Its output goes to exactly
five CSV columns — `fov_est_deg`, `fov_est_m1_deg`, `fov_est_m2_deg`,
`fov_truth_deg`, `fov_est_n` — and NOWHERE else. It must NEVER reach the `GUIDE`
frame, the world-balloon estimator, the steering error, the balloon vector, or any
scoring; recovering a bearing from pixels needs the FOV, and the lens can change
in flight, so C-02 forbids FOV from steering. It never commands motion. Anything
added here holds the same line; `Companion/tests/test_no_micromanagement.py`
guards the wire. Ground truth (`SdfManifest`) is read from the active SDF at
launch for comparison ONLY, WARNS and never raises.
