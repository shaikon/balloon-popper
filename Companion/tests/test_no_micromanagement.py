# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition.
"""The FC must own the control law outright.

The entire point of the port is that steering closes on the flight controller,
against attitude sampled in the same tick as the gimbal encoders. If the
companion computes any part of the cascade and ships the answer, the migration
buys nothing and the A/B measures nothing - the numbers would agree because one
side is copying the other.

So the wire is only allowed to carry three kinds of thing:

    SENSOR          det_norm, gimbal encoders, oracle bearing
    OPERATOR INTENT mode, flags, throttle, heading setpoints
    CONFIGURATION   gains (SET_GAIN), excitation description (EXCITE)

and one exception, the PLAN: the Curve Planner's bank schedule for the turn
(CLAUDE.md "Lua steering offload"), which carries the schedule and nothing
else.

Never: rates, bank angles, pointing errors, integrators, phase, or any other
quantity the control law produces.
"""
import os, re, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from Companion import protocol as P

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LUA = os.path.join(ROOT, 'LUA')
fails = []
def check(n, c, d=""):
    print(f"  [{'PASS' if c else 'FAIL'}] {n}" + (f"   {d}" if d else ""))
    if not c: fails.append(n)

print("\n" + "=" * 74)
print("THE FC OWNS THE CONTROL LAW"); print("=" * 74)

print("\n1. THE WIRE CARRIES NO CONTROL-LAW OUTPUT")
BANNED = ('omega', 'rate_cmd', 'bank', 'roll_cmd', 'pitch_cmd', 'req_roll',
          'req_pitch', 'theta_ip', 'pan_err', 'tilt_err', 'integrator',
          'ff_bank', 'aim_body')
for msgid, (name, flds) in sorted(P.MESSAGES.items()):
    if msgid in (P.MSG_STATE, P.MSG_STATE2):
        continue                      # FC -> companion: telemetry only
    if msgid == P.MSG_PLAN:
        continue                      # the one exception, checked below
    bad = [f.name for f in flds
           if any(b in f.name.lower() for b in BANNED)]
    check(f"{name} carries no computed control quantity", not bad, str(bad))

# The PLAN carries the bank schedule, the steering law it was made for, its
# progress reference and the run-in - its line's climb and the balloon's
# depression that ends it - under a plan number, and nothing else: no rate,
# no time, no route.
_plan_fields = [f.name for f in P.PLAN if not f.name.startswith('_')]
_plan_allowed = (['plan_id', 'law', 'eps0_deg']
                 + [f"knot_{k + 1}_deg" for k in range(P.CURVE_N_KNOTS)]
                 + ['runin_climb_deg', 'runin_dep_deg'])
check("PLAN carries the bank schedule and its run-in, nothing else",
      _plan_fields == _plan_allowed, str(_plan_fields))

print("\n2. THE FC COMPUTES THE WHOLE CASCADE ITSELF")
fc = open(os.path.join(LUA, 'balloon_chaser_fc.lua')).read()
for what, token in (("reads its own attitude", "ahrs:get_quaternion"),
                    ("reads its own gyro", "ahrs:get_gyro"),
                    ("reads its own velocity", "ahrs:get_velocity_NED"),
                    ("reads its own airspeed", "ahrs:airspeed_EAS"),
                    ("runs the estimator", "est:update"),
                    ("computes trust", "Est.trust"),
                    ("runs the geometry layer", "track_world_balloon"),
                    ("runs the cascade", "apply_control_law"),
                    ("actuates directly", "set_target_throttle_rate_rpy")):
    check(f"    {what}", token in fc)

print("\n3. THE FC DOES NOT DUPLICATE COMPANION-OWNED ACTUATORS")
# The gimbal is a Gazebo model driven by the companion over gz-transport.  A
# mount:set_rate_target() here is either a no-op (no mount driver) or a second
# controller fighting the first.
# Match the CALL, not the word: the header comment explains why the call was
# removed, and that explanation must not itself trip the check.
_calls = [l for l in fc.splitlines()
          if 'mount:set_rate_target' in l and not l.lstrip().startswith('--')]
check("    does not command the gimbal", not _calls,
      "gimbal pointing is tier-1 sensor work, owned by the vision pipeline")
check("    consumes gimbal ENCODERS instead",
      'guide.gimbal_pan' in fc and 'guide.gimbal_tilt' in fc)

print("\n4. ONE ROTATION IMPLEMENTATION, NOT TWO")
# The estimator works in the world frame, so every tick round-trips
# encoders -> world -> body.  If the two legs use DIFFERENT implementations of
# the same rotation and they disagree by a transpose, the round trip applies the
# rotation twice instead of cancelling: zero error at level flight, growing with
# attitude.  Desktop tests cannot catch it, because ap_shim's earth_to_body is a
# hand transcription that is self-consistent with itself.
#
# So the FC path must use G.quat_to_rotation_matrix for BOTH directions - the
# possibility is removed rather than tested for.
for f in ('balloon_chaser_fc.lua', 'track.lua', 'cascade.lua'):
    body = open(os.path.join(LUA, f)).read()
    code = "\n".join(l for l in body.splitlines() if not l.lstrip().startswith('--'))
    check(f"    {f} does not call earth_to_body",
          'earth_to_body' not in code,
          "use G.quat_to_rotation_matrix (transposed for body->world)")
check("    balloon_chaser_fc.lua transposes the matrix for body->world",
      'r00 * bx + r10 * by + r20 * bz' in open(os.path.join(LUA,'balloon_chaser_fc.lua')).read())

print("\n5. THERE IS ONE CONTROL LAW, AND IT IS ON THE FC")
# The companion sends inputs and reads the FC's values back.  A second copy of
# the law on this side is not a diagnostic: the moment the two drift, the log
# describes an aircraft that does not exist.
link = open(os.path.join(ROOT, 'Companion', 'adapters', 'lua_offload.py')).read()
ml = open(os.path.join(ROOT, 'Companion', 'adapters', 'mavlink_link.py')).read()
ctl = open(os.path.join(ROOT, 'Companion', 'controller.py')).read()
cl = open(os.path.join(ROOT, 'Companion', 'control_laws.py')).read()
m = re.search(r'def send_guide\((.*?)\).*?\n(.*?)\n    def ', link, re.S)
check("    the per-tick call sends GUIDE and takes only the throttle",
      m is not None and m.group(1).replace(' ', '') == 'self,throttle_frac:float'
      and '_send_guide()' in m.group(2),
      m.group(1) if m else "send_guide not found")
for where, src in (('lua_offload.py', link), ('mavlink_link.py', ml),
                   ('controller.py', ctl)):
    check(f"    no rate command is encoded or sent from {where}",
          'def send_rate_command' not in src and 'send_rate_command(' not in src)
_law_defs = [n for n in ('def compute_roll_command', 'def compute_bodyplane_pitch_rate',
                         'def update_bodyplane_pitch_integrator',
                         'def compute_throttle_command', 'class CascadeState')
             if n in cl]
check("    control_laws.py holds gains only, no law", not _law_defs,
      ", ".join(_law_defs))
check("    the controller runs no law",
      '_apply_control_law' not in ctl and 'compute_' not in ctl)

print("\n6. RC OVERRIDE CARRIES ONLY THE LEVEL-OUT")
# Steering never overrides the sticks.  The override remains for the FBWA
# level-out, so RC_OPTIONS bit0 (IGNORE_OVERRIDES) is still checked at startup.
check("    the level-out is held by RC override", 'def hold_level_sticks' in ml
      and 'rc_channels_override_send' in ml.split('def hold_level_sticks')[1][:3000])
check("    RC_OPTIONS bit0 is still verified", "'RC_OPTIONS'" in ml)
check("    no stick-encoding parameters are read",
      not any(f"'{p_}'" in ml for p_ in ('RC1_TRIM', 'RC2_TRIM', 'RC1_DZ',
                                         'RC2_DZ', 'RC1_REVERSED')))
# SERVO*_TRIM/MIN/MAX look like RC plumbing but the HUD converts
# SERVO_OUTPUT_RAW to a deflection percentage with them - the only view of the
# surfaces.
for p_ in ('SERVO1_TRIM', 'SERVO1_MIN', 'SERVO1_MAX',
           'SERVO2_TRIM', 'SERVO2_MIN', 'SERVO2_MAX'):
    check(f"    {p_} is read (HUD scales with it)", f"'{p_}'" in ml)

print("\n6b. SURFACE MIXING IS REPORTED CORRECTLY")
# The Zephyr is a flying wing: servos 1/2 are ELEVONS, and both carry pitch and
# roll.  Reporting them as "Aileron"/"Elevator" showed two identical numbers in
# level flight, reading as roll AND pitch held when the aircraft held neither.
check("    SERVOn_FUNCTION is read", "'SERVO1_FUNCTION'" in ml)
check("    elevon_mixing is exposed", 'def elevon_mixing' in ml)
_hud2 = open(os.path.join(ROOT, 'Companion', 'ui', 'hud.py')).read()
check("    HUD demixes to roll/pitch when elevons",
      's.elevon_mixing' in _hud2 and '(_p1 - _p2) / 2.0' in _hud2,
      "roll = differential, pitch = common mode")

print("\n7. THE DIAGNOSTIC STREAMS THAT STILL MATTER ARE KEPT")
# drop_telemetry_the_fc_does_not_need is a documented no-op: every remaining stream is
# either group-rate-locked to ATTITUDE or is the only source of something.
_dropblock = link[link.index('def drop_telemetry_the_fc_does_not_need'):][:1600]
# PID_TUNING carries the FC's own desired-vs-achieved rate.  With the whole
# cascade on the FC, it is the only remaining window into whether the inner
# loop is tracking what the steering asked for.
check("    nothing is actually dropped", 'log.info' in _dropblock
      and 'command_long_send' not in _dropblock,
      "SIMSTATE is group-rate-locked to ATTITUDE; the rest are load-bearing")
check("    the reason is written down, not just the behaviour",
      'unavoidable bandwidth cost' in _dropblock and 'PID_TUNING' in _dropblock)
# It feeds the HUD's "Real Aileron/Elevator" deflection - the only remaining
# view of what the surfaces are doing now the companion does not command them.
check("    SERVO_OUTPUT_RAW's role is documented",
      'SERVO_OUTPUT_RAW' in _dropblock)
hud = open(os.path.join(ROOT, 'Companion', 'ui', 'hud.py')).read()
# There is no requested surface PWM - the FC is commanded in deg/s - so the
# HUD has no "Requested Aileron/Elevator" line, and the demand lines show the
# FC's own rates.
_hud_block = hud[hud.index('# ---- Aileron / elevator blocks'):]
_hud_block = _hud_block[:_hud_block.index('# Controller error readouts')]
check("    no requested-PWM lines",
      'last_pwm' not in _hud_block and 'lua_path' not in _hud_block)
check("    the demand lines use the FC's rates",
      's.lua_omega_x' in _hud_block and 's.lua_omega_y' in _hud_block)
check("    layout reflows instead of leaving a gap",
      '_y = 25 + (len(_status_lines) + 1) * 25' in _hud_block and '_y += 25' in _hud_block,
      "running cursor from one row below the status block, not hardcoded offsets")

print("\n8. THE SOURCE FINGERPRINT COVERS WHAT ACTUALLY FLIES, AND THE DOCS")
# With the cascade on the flight controller, a fingerprint over the package's
# .py alone would be IDENTICAL for two runs flown by completely different
# steering code - the one thing a fingerprint exists to make impossible.
csvsrc = open(os.path.join(ROOT, 'Companion', 'csv_logger.py')).read()
_fp_src = csvsrc.split('def package_sha256')[1].split('\nPACKAGE_SHA256 =')[0]
check("    the .lua sources are hashed", '.lua' in _fp_src)
check("    desktop-only Lua is excluded",
      'ap_shim.lua' in _fp_src,
      "the shim and test drivers never reach the aircraft")
check("    main.py is hashed (it sets the link and the startup order)",
      '"main.py"' in _fp_src)
check("    the .md documents are hashed (the rules and design the code follows)",
      '".md"' in _fp_src)
from Companion.csv_logger import package_sha256
_d = package_sha256()
check("    digest computes", len(_d) == 64, _d[:16] + "...")

print("\n" + "=" * 74)
print("PASSED" if not fails else f"FAILED: {fails}"); print("=" * 74)
sys.exit(1 if fails else 0)
