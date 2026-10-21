# SUGGESTIONS.md

Proposals under discussion. Adopting any of them is an explicit decision, made
the way a `TODO.md` item is; a proposal that has been adopted says so, and
where it now lives.

## Lift Vector Pointing: aim the roll at the force the wing must make

**Adopted, both versions.** Lift Vector Pointing P is Steering Law 2 and N is
Steering Law 3 (`LUA/path_attitude.lua`), beside the cascade (0) and Lift
Vector Pointing itself (1). The numbers below are the implemented laws'.

### The problem

In a near-vertical climb at high throttle, Lift Vector Pointing rocks the
aircraft from side to side at about 2 Hz: the roll command pinned at
`ACRO_ROLL_RATE`, the lift side swapping between pull and push, and the bank
winding round the path. `test_balloon_chaser_fc.lua` § 19 reproduces it on
`test_plant.lua`: a manual climb toward a target 85° up at the default throttle,
with the steady side force SITL's Talon carries in that climb, 0.6 m/s² along
the wing at a skid of about 1°. Without the side force the same climb is calm.

### The cause: the force line is not the wing's

The law asks the wing for the force line, `f = V·Ω·u − g⊥`: the push across the
path that the whole airframe must make. The wing is not the only thing pushing
across the path. With the nose off the path, the propeller's push has a part
across it as well - `T·sin α` toward the belly while the nose sits below the
path, `−T·sin β` along the wing in a skid. In level flight that part is small.
In the model's 85° climb at the default throttle the thrust is about 1 g and
the nose sits 6.7° below the path, so it is about 1.15 m/s²: more than twice
the force line of the reproduced climb, about 0.5 m/s².

The law aims the lift line at `f` as if the wing had to make all of it, and its
roll error - the angle from the lift line to `f` - moves by `1/|f|` for any
change in `f`. So it aims at the wrong force, and reacts over three times too
hard to everything, the skid its own rolling makes included: a roll about the
nose turns the angle of attack into sideslip, and the propeller's push swings
across the path with it.

### The change

Aim the lift line at the force the wing must make - the force line less the
propeller's part across the path:

    f_wing = f − T⊥,    T⊥ = T · (x_b·y', x_b·z')

`x_b` the body x axis, `(y', z')` the path frame across the path, `T` the
propeller's push per unit mass. The guidance, the pitch, the lift side rule,
the band and its room, upside down and plan mode are otherwise unchanged.

`T` is not measured on the FC. P and N estimate it from the throttle and the
airspeed by a straight propeller line, two operator settings carried in
`SET_GAIN` (the Gain Tuner's Prop Pitch Speed and Prop Slope, logged as
`g_att_prop_speed_mps` and `g_att_prop_slope`):

    T = att_prop_slope / EAS2TAS² · τ · (att_prop_speed_mps · τ − u)

`τ` the throttle output as a fraction of full and `u` the true airspeed along
the nose, the EAS floored where the pitch trim floors it. With no airspeed
estimate there is no `T`, and P and N aim at the force line. The defaults are
`test_plant.lua`'s propeller line. Each version's pitch trim lift term reads
the load its own aim asks of the wing.

### Two versions: the roll error round the path, or round the nose

The law reads its roll error round the path, as every balloon error but one is
read. Roll-to-plane reads its clock round the nose, for a reason found in the
same kind of climb (ARCHITECTURE.md § "Roll-to-plane": for a balloon between
the nose and the path, a clock round the path asks for the opposite roll, which
tilts the pull the wing holds away from the balloon). So the change has two
versions.

- **P - round the path, the law's frame.** The roll error is the angle round
  the path from the lift line to `f_wing`: `atan2(sd·f_wing,y', −sd·f_wing,z')`.
  Nothing else moves.
- **N - round the nose, roll-to-plane's frame.** The roll is about the body x
  axis, which turns the lift and the side force but not the propeller's push
  along `x_b` nor the drag along the path. The force it must bring the lift
  line onto is the needed specific force less those two, in the body's y-z
  plane: `(f + T·(x_b·x')·x')` resolved on `y_b` and `z_b`. The roll error is
  the angle round `x_b` from the lift line to it, and the roll goes out as the
  body roll rate. Its band and upside down are worked out for a roll about the
  nose: after a roll `t` about it, the wing tip's angle below the horizon has
  sine `R·sin(φ + t)`, `φ` the bank round the nose (the Euler roll) and `R` the
  cosine of the nose's pitch, and the canopy faces the ground past
  `|φ + t| = 90°`. The drift it cancels while the room holds the bank is the
  Euler roll's rate.

Both fly Lift Vector Pointing's plan mode, round the path, so the Curve Planner
plans their turns as it plans law 1's.

### What the desktop model shows

The FC shell test, run whole - § 16 (the Curve Flyer) under all four laws,
§ 17 to § 19 under 1, 2 and 3 - and the 85° climb flown 20 s against side
forces from 0 to 0.9 m/s². `T` is estimated with the default line, which is
the model's own push.

| | Lift Vector Pointing (1) | P (2) | N (3) |
|---|---|---|---|
| § 19 against 0.6 m/s²: roll command at the limit, lift side changes | 240 of 506 ticks, 23 | none, none | none, none |
| § 19 against 0.6 m/s²: lines up (within `ALIGN_TOL_DEG`) | no: 0.16° at the closest | no: 0.59° | no: 0.57° |
| § 18: sphere chases that line up | 274 of 276 | 276 of 276 | 276 of 276 |
| § 18: chases with a Suboptimal Curve, of 169 | 122 | 125 | 124 |
| default `p` balloon lines up after | 4.24 s | 4.74 s | 4.60 s |
| the climb at 0 / 0.3 / 0.6 / 0.9 m/s²: roll rate rms after 6 s | 0.8 / 10.6 / 84.1 / 96.5 °/s | 0.0 / 0.4 / 1.5 / 95.3 °/s | 0.0 / 0.3 / 1.5 / 91.2 °/s |

§ 16 passes under P and N as under law 1. Of the FC shell test's checks, P and
N fail only the two law 1 fails in § 18 and § 19 that they do not fix: the
Suboptimal Curves, and lining up against the side force.

With `T` estimated at half and at 1.5 times the model's push: every sphere
chase still lines up under both versions; P has 124 and 124 Suboptimal Curves
and N 120 and 124; the default `p` balloon lines up after 4.24 and 4.50 s (P)
and 4.62 and 4.64 s (N); and § 19's roll stays off its limit with one lift
side.

### The propeller line in SITL

The defaults are the desktop model's line, and SITL pushes harder. In the
85° climb of `flight_log_20261020_231238.csv` - 85 % throttle, about 16 m/s
EAS, the nose about 7° off the path - the accelerometer along the flight path
reads 9.8 m/s². That is the push's share along the path less the drag, so the
push is at least 9.9 m/s², about 1 g. The default line gives 3.7 m/s² there. So in SITL the defaults leave P and N with under 40 % of the
push, past the half tested above; the line needs fitting to SITL before P
and N are judged there, and to the aircraft before they fly.

### What neither version fixes

- **The steady miss against a side force.** § 19's climb holds 0.6° off the
  target, outside `ALIGN_TOL_DEG`. The guidance is proportional, so it holds
  off a steady sideways push only by sitting off the target, by about
  `d / (V·k_path)`. A separate change.
- **A side force larger than gravity's part across the path.** With the rudder
  at zero, the only steady lever against a force along the wing is gravity's
  part across the path, `g·cos γ`, through the bank - about 0.8 m/s² in the
  85° climb. Past it no bank holds the line, and both versions rock at
  0.9 m/s². Holding it needs the rudder as a side force channel, a departure
  from version B's rudder held at zero.

### Open questions

- **Which version to keep.** In the model they differ little; flying both in
  SITL with a fitted propeller line decides it.
- **The trim map** fitted under one law is not another's: each reads its own
  lift term, so it is re-fitted under the law that flies.
