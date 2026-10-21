# SUGGESTIONS.md

Proposals under discussion. Nothing in this file is implemented; adopting any
of it is an explicit decision, made the way a `TODO.md` item is.

## Lift Vector Pointing: aim the roll at the force the wing must make

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

`T` is not measured on the FC. It needs an estimate from the throttle and the
airspeed: a propeller model whose two numbers, a pitch speed and a slope, come
from measurement. `test_plant.lua`'s propeller line is fitted to SITL's climb
and dive speeds at 85 % throttle; the aircraft needs the same measurement. As
operator settings they travel in `SET_GAIN`, each with a Gain Tuner row, an
Info entry and a `g_*` column (CLAUDE.md).

### Two versions: the roll error round the path, or round the nose

The law reads its roll error round the path, as every balloon error but one is
read. Roll-to-plane reads its clock round the nose, for a reason found in the
same kind of climb (ARCHITECTURE.md § "Roll-to-plane": for a balloon between
the nose and the path, a clock round the path asks for the opposite roll, which
tilts the pull the wing holds away from the balloon). So the change has two
versions.

- **Version P - round the path, the law's frame now.** The roll error is the
  angle round the path from the lift line to `f_wing`:
  `atan2(sd·f_wing,y', −sd·f_wing,z')`. Nothing else moves.
- **Version N - round the nose, roll-to-plane's frame.** The roll is about the
  body x axis, which turns the lift and the side force but not the propeller's
  push along `x_b` nor the drag along the path. The force it must bring the
  lift line onto is the needed specific force less those two, in the body's
  y-z plane: `(f + T·(x_b·x')·x')` resolved on `y_b` and `z_b`. The roll
  error is the angle round `x_b` from the lift line to it, and the roll goes
  out as the body roll rate.

### What the desktop model shows

The FC shell test, run whole, and § 19's climb flown 20 s against side forces
from 0 to 0.9 m/s². `T` is the model's own push, not an estimate, except where
the table says otherwise.

| | Now | Version P | Version N |
|---|---|---|---|
| § 19: roll command at the limit, lift side changes | 247 of 506 ticks, 30 | none, none | none, none |
| § 19: lines up (within `ALIGN_TOL_DEG`) | no | no: holds 0.66° off | no: holds 0.64° off |
| § 18: sphere chases that line up | 274 of 276 | 276 of 276 | 240 of 276 |
| § 18: chases with a Suboptimal Curve, of 169 | 122 | 121 | 127 |
| default `p` balloon lines up after | 4.24 s | 4.74 s | 5.66 s |
| FC shell test: failing checks, of 275 | 8 | 6 | 7 |
| the climb at 0 / 0.3 / 0.6 / 0.9 m/s²: roll rate rms | 0.8 / 10 / 91 / 95 °/s | 0.0 / 0.3 / 1.3 / 92 °/s | 0.0 / 0.3 / 1.3 / 91 °/s |

Version P with `T` estimated 30 % low and 30 % high: 276 of 276 chases line
up, 120 and 121 Suboptimal Curves, the default `p` balloon after 4.98 s and
4.86 s, and § 19's roll stays off its limit with one lift side.

Version N's 36 misses are 27 balloons behind, and 9 ahead of or square to the
wing line, all from a start banked 60°, six of which the path never came
within 45° of. It was flown with the path roll's band and room - the limits a
roll about the nose may go to were not worked out for it - and misses from
banked starts point at those borrowed limits more than at the error itself;
that is untested.

### What neither version fixes

- **The steady miss against a side force.** § 19's climb holds 0.6-0.7° off
  the target, outside `ALIGN_TOL_DEG`. The guidance is proportional, so it holds
  off a steady sideways push only by sitting off the target, by about
  `d / (V·k_path)`. A separate change.
- **A side force larger than gravity's part across the path.** With the rudder
  at zero, the only steady lever against a force along the wing is gravity's
  part across the path, `g·cos γ`, through the bank - about 0.8 m/s² in the
  85° climb. Past it no bank holds the line, and both versions rock at
  0.9 m/s². Holding it needs the rudder as a side force channel, a departure
  from version B's rudder held at zero.

### Open questions

- **Which version.** The model favours version P; version N's result may
  change with its own band and room.
- **The pitch trim's lift term.** It reads `−f_z/g`, the load the law asks; the
  wing's own load is now `−f_wing,z'/g`. The tests kept `−f_z/g`, and changing
  it changes what the trim map fits.
- **Plan mode** is untouched: its roll goes to the PLAN's bank, so the Curve
  Planner's model of it needs no change.
