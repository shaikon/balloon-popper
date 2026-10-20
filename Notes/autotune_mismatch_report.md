# The AUTOTUNE mismatch

*ArduPlane's built-in AUTOTUNE moved our pitch gain the wrong way by a factor
of thirty. Not because it broke — because it was optimising a different
maneuver from the one we fly.*

**Date** 2026-09-22 &nbsp;·&nbsp; **Airframe** X-UAV Mini Talon (V-tail) &nbsp;·&nbsp;
**Firmware** ArduPlane V4.7.0-dev (2d99a5ea) &nbsp;·&nbsp; **Scope** pitch rate loop only

---

## 1 · The starting problem

Persistent pitch overshoot at the end of every balloon chaser run.

Every balloon chaser slew showed the same failure: as the nose reached the balloon the
aircraft kept pitching, overshot by several degrees, and then settled back.
We first found and fixed a mathematical bug in the pitch-error formula — the
elevation demand was being inflated by the balloon's lateral offset, so a
balloon far off to one side produced a much larger pitch demand than the
geometry actually called for. That fix landed and the overshoot shrank. It
did not disappear.

The residual overshoot had a specific signature: nose-up authority looked
adequate on the way in, and the surface reversed briskly at the top; the
aircraft still kept pitching for a fraction of a second after the reversal.
That is what a wound-up integrator looks like when the loop cannot unwind it
fast enough at the stop.

## 2 · Reasonable expectation from AUTOTUNE

More proportional gain would give the loop more braking authority.

A rate loop that fights a wound-up integrator at the stop has one useful
lever: the proportional term, `PTCH_RATE_P`. Larger P means the loop reacts
faster to the reversal of the demanded pitch rate; smaller P means the
integrator sits there for longer, adding elevator the wrong way. So we ran
ArduPlane's built-in AUTOTUNE and expected it to raise P.

## 3 · What AUTOTUNE actually did

It reduced `PTCH_RATE_P` by a factor of fifteen. Overshoot got worse.

### PTCH_RATE_P across three flights

| Condition                     | PTCH_RATE_P | vs. pre-tune | Effect on overshoot          |
| ----------------------------- | ----------: | -----------: | ---------------------------- |
| Before AUTOTUNE               |       0.48  |     baseline | baseline overshoot (~7°)     |
| After AUTOTUNE                |       0.033 |      0.07 ×  | **worse** (~15°)             |
| Manual, `PTCH_RATE_P = 1.0`   |       1.00  |      2.1 ×   | **better** (~7° → < baseline)|

Two facts, one conclusion. AUTOTUNE moved the gain the opposite direction from
the one that helps our maneuver — *by a factor of thirty against the gain we
later found reduced overshoot by hand*. That is not a small effect and not a
matter of noise: it is a categorical disagreement between what the tool
prefers and what the aircraft prefers on this task.

## 4 · Why AUTOTUNE moved P the wrong way

It optimises small, fast doublets. We fly large, sustained slews.

Read from the ArduPilot source (`AP_AutoTune.cpp`), AUTOTUNE's method is:
inject a short doublet (~0.7 s each way), search upward for the largest P at
which the response does not ring, and stop there. It is optimising for
buzz-free tracking of small, fast disturbances — exactly what a
general-purpose acro tune is supposed to do.

The balloon chaser maneuver looks nothing like that. A balloon chaser run is a large,
sustained pitch-rate demand, held for seconds, slammed to a stop. The two
signals put the plant at two different operating points, and the gain that is
best at one is not the gain that is best at the other:

**AUTOTUNE's operating point — small, fast, buzz-limited.**
P is chosen so the response does not ring. Small amplitude, no integrator
windup, no aerodynamic non-linearity. Higher P just adds noise sensitivity,
so AUTOTUNE (correctly) prefers a lower P.

**Our operating point — large, sustained, brake-limited.**
During the slew the integrator quietly accumulates. At the stop, only P is
fast enough to unwind it before the nose carries past the balloon. Lower P
means more overshoot; larger P means a cleaner arrest.

> AUTOTUNE didn't fail. It correctly optimised what it was asked to optimise.
> It's simply the wrong question for this task.

## 5 · What follows

The rate loop needs a task-shaped cost, and the airframe measured cleanly.

Two consequences, in order.

**The gain we want cannot be found by small doublets.** Any procedure whose
cost function is "no ringing on a 0.7 s doublet" will converge somewhere near
AUTOTUNE's answer, for the same reasons AUTOTUNE did. We need a cost that
reads the actual failure mode — overshoot at the arrival, time-to-arrest,
ringing on the stop — and iterates the rate PID against that.

**We can't do that safely without measuring the plant first.** A gain search
bounded only by pass/fail on real flights is expensive and risky. Bounding it
against a measured plant turns "how much P is too much" into arithmetic:
control power sets the physical brake authority, damping and static stability
set what the loop is fighting, and the two together give an overshoot floor —
a lower bound no rate-loop tune can beat. A search that doesn't know that
floor spends flights chasing a balloon that is not physically reachable.

### Stage A — open-loop plant identification (`i` key)

Excite the elevator directly in MANUAL, with the FC's rate loop out of the
way, and fit the short-period model from the response. Yields control power,
damping, the honest feedforward, and the overshoot floor. Changes no stored
gains.

### Stage B — balloon chaser rate autotune (deferred)

Iterate the rate PID against a balloon-chaser-shaped cost, using Stage A's plant to
bound the search. Not built yet: it exists as a plan, gated on Stage A
validating in flight.

---

## Appendix · the three flights that decided this

What each flight told us, and what it left open.

**Pre-AUTOTUNE baseline.** `PTCH_RATE_P = 0.48`. Balloon Chaser runs finished with
roughly seven degrees of overshoot, consistent across seeds. Pitch demand
already carried the geometry fix, so the residual overshoot was not the
pitch-formula bug.

**Post-AUTOTUNE.** AUTOTUNE ran to completion on both axes and settled at
`PTCH_RATE_P = 0.033`. The following balloon chaser runs overshot by roughly fifteen
degrees. Nothing else changed between the two flights.

**Manual P = 1.0.** Set by hand, roughly twice the pre-AUTOTUNE value.
Overshoot returned to seven degrees and reduced further on the well-flown
runs. This is the flight that settled the direction: the gain AUTOTUNE
recommended and the gain that helps our maneuver sit on opposite sides of the
pre-tune value, not close together with noise between them.

What the three flights *don't* tell us is where the pitch loop's true optimum
sits on this task, or how the same argument plays out on the roll axis. Both
are Stage B's job; both are unsafe to search without Stage A first.

---

*Prepared for the balloon chaser tuning file. All numbers cited above are from the
flight logs in the project's `flight_log` archive.*
