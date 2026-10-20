# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""
Controller Autotune (gain tuner) window - click a row, type a value, Enter to apply; backtick resets.

Source: _TUNER_PARAMS 4501-4542, _create_pid_tuner 4545-4551,
        _tuner_mouse_callback 4553-4562, _DEFAULTS 4564,
        _reset_tuner_to_defaults 4566-4609, _handle_tuner_key 4611-4644,
        _render_pid_tuner 4646-4686.

REFACTOR: the source mutated controller attributes directly (setattr(self, attr,
val)) and read module globals() for defaults.  Post-split the tunables live in
three different holders:
    - most are ControlGains fields
    - gimbal_slew_rate is a GimbalDriver property
    - throttle_pct / point_* / arrow_* are runtime knobs
So the tuner talks to a ParamStore (duck-typed: get(attr)->float, set(attr,val),
reset_all()).  The controller implements ParamStore and routes each attr to the
right holder.  There is ONE row list, with each row's default-constant name
folded in as a third tuple element, not two parallel name lists to keep in step.
"""
from __future__ import annotations
import collections as _collections
import logging
import math
import threading
from typing import Optional, Protocol

# File-export buttons (WSL -> Windows).  Imported tolerantly: if the module or
# its destination config is missing the tuner still opens, just without the
# export group.
try:
    from .. import file_export
except ImportError:                                   # pragma: no cover
    file_export = None

from ..constants import (ARDUPLANE_AUTOTUNE_PARAMS, EXPECTED_RATE_PIDS,
                         ACRO_DEFAULT_PARAMS,
                         NATIVE_MODE_RATE_PIDS,
                         PTCH_RATE_FLTE_EXPECTED, RLL_RATE_FLTE_EXPECTED,
                         PITCH_TRIM_VREF_MPS, ELEV_FULL_TRAVEL,
                         PLANE_SIDE_HOLD_DEG, PLANE_GAIN, PLANE_FF_GAIN,
                         PLANE_FF_LPF_RATE, PLANE_STEEP_START_DEG,
                         PLANE_STEEP_FULL_DEG, PLANE_KI, PLANE_WINDUP,
                         PLANE_KD, PLANE_KD_RATE,
                         FLIGHT_PATH_MIN_SPEED_MPS, YAW_COORDINATION_FF,
                         CURVE_TURN_DONE_DEG, CURVE_POLE_FADE_DEG)
from ..controller_autotune_store import acro_default_value, float32_text


log = logging.getLogger("balloon_chaser")

# How long a Delete button stays primed after the first press.  Long enough to be
# a deliberate second click, short enough that a primed button cannot be left
# sitting there to be hit by accident later.
DELETE_CONFIRM_MS = 4000

# Shared widget styling.  Every pressable thing gets the same treatment so the
# tuner reads as one panel rather than a mix of flat labels and live controls.
#
# BTN_RELIEF/BTN_BD: Tk draws a RAISED button as SUNKEN for as long as it is held
# down, which is the physical press feedback.  A flat button gives none - it is
# indistinguishable from a coloured label until something happens elsewhere.
# BTN_OVERRELIEF lifts it on hover, so it also reads as pressable before you
# commit to clicking.
# The small drawn icon controls (info dot, per-row reset).
#
# ICON_SIZE (the canvas each glyph is drawn in) is larger than the glyph's own
# 16px circle because the reset arrow's head is placed off the end of its arc
# and overhangs it: at 20 or 22 the head runs past the top edge and is clipped,
# and at 23 - after the 2px ICON_INSET shift spends 2px of the bottom-right
# margin - it still overran by 0.2px.  24 is the smallest canvas the glyph fits
# in whole; ICON_SIZE is 25, a pixel of headroom above that.
# ROW_WIDGET_H is the one height every control in the value column takes.  The
# info and reset icon canvases are BUILT at this height (ICON_SIZE + border +
# aura); the entry fields and the discrete switch reach it by FILLING a
# fixed-height value cell (see _value_cell), the same way labels fill
# _label_cell.  The cell owns the size, so a widget's own height request does
# not have to be tuned to match the icons - which is why the entry and the
# switch carry no ipady/pady of their own.  Changing ICON_SIZE moves the value
# column together.
ICON_SIZE = 25
ROW_WIDGET_H = ICON_SIZE + 2 * (1 + 1)      # canvas + bd + aura = 29
EXPORT_PADY = 3                             # Copy / Delete / Reset-all / Close
ICON_R = 8.0            # circle radius the info dot is built on
ICON_RESET_R = 6.29     # the reset arrow, 15% smaller again (7.4 x 0.85).
                        # The arrowhead offsets scale with ICON_RESET_R /
                        # ICON_R, so the whole glyph shrinks together and the
                        # head cannot grow proportionally dominant.
ICON_W = 2              # stroke width, shared
ICON_BD = 1

# The drawing origin of these Canvases sits at the widget's OUTER edge, so the
# raised bevel and the focus ring are painted OVER the top-left of the drawing
# area rather than around it.  Measured on screen, that puts both glyphs 2px up
# and 2px left of the visible interior - 4px more clearance on the right than the
# left, on both icons.  Shifting the drawing by the same inset centres them on
# what the eye actually sees.  Item coordinates cannot detect this: in canvas
# coordinates the glyphs are already centred.
ICON_INSET = 2          # = ICON_BD + AURA_PX

# How the drawn icons show a press.  A 1px bevel flipping raised/sunken is not
# visible on this panel, so the glyph also dips and the face lightens.
ICON_PRESS_DIP = 1      # px the glyph moves down-right while held
ICON_PRESS_BG = "#26313b"

def _bind_aura_hover(w):
    """Brighten the focus ring while the pointer is over `w`.

    Applied to every control whatever its widget class, so the hover cue is ONE
    behaviour instead of three: Button had it natively via -activebackground,
    Canvas and Entry had nothing.  add="+" so it never displaces a handler the
    control already carries.
    """
    w.bind("<Enter>", lambda _e: w.configure(highlightbackground=AURA_HOVER), add="+")
    w.bind("<Leave>", lambda _e: w.configure(highlightbackground=AURA_IDLE), add="+")


def _style_entry_hover(ent):
    """Hover cue for an Entry: the aura, and nothing else.

    Entry has no -activebackground.  Filling it pale would mean swapping the
    foreground and the caret with it - these fields carry light text on
    near-black - so the ring says the same thing without touching anything the
    eye is trying to read, and it is the cue every other control uses too.
    """
    _bind_aura_hover(ent)

# The icons are nudged within the ROW rather than drawn off-centre in their own
# canvases - shifting the drawing would push it past the edge and clip it.
#
# ICON_DY applies to BOTH icons so they stay on the same baseline.  Give it to
# only one and pack centres the other in the row's new height, leaving the two
# about half the nudge apart: with pady=(5,0) on the info alone they sat at y=5
# and y=2.  INFO_DX is the info's horizontal gap from the entry field and is its
# own business, so it stays info-only.
INFO_DX = 5
# 0 now that every control in the value column is ROW_WIDGET_H tall: with equal
# heights pack aligns them by itself, and any nudge here pushes the icons BELOW
# the entry and the switch.  Any non-zero value here compensates for an icon /
# entry height mismatch that must be fixed at the source instead.
ICON_DY = 0

BTN_RELIEF = "raised"
BTN_OVERRELIEF = "ridge"
BTN_BD = 1              # one border for every control: 1px, raised

# The "aura": a 1px focus ring around every control, in the panel accent when
# idle and brighter when it holds keyboard focus.  Entries already carried one
# from Tk's defaults; this gives buttons and the info dots the same outline so
# the boundary of a control is always visible against the dark background.
AURA_PX = 1
# Chosen for CONTRAST against the panel, not for taste: the backgrounds here run
# #0a0a0a..#202830, and a ring darker than about #40 has under 20 points of
# luminance separation from the row it sits on - present in the widget config,
# invisible on screen.  #46586a clears 46.
# The aura IS the hover cue - nothing fills pale under the pointer any more, so
# the ring is what says "this is live".  Buttons get it by BINDING rather than by
# -activebackground, which is pinned to each control's own face precisely so Tk
# cannot repaint it.
AURA_IDLE = "#46586a"       # at rest
AURA_HOVER = "#8fc4dc"      # under the pointer
AURA_FOCUS = "#5fb0d0"

# A discrete switch shows which value is live by COLOUR, not by brackets: the
# active option is drawn in the row's accent, the others dimmed.  Colour is what
# lets every option be the SAME width, so the options can sit at equal spacing
# across the strip (a bracketed "[1]" is wider than "0" and would break it).
DISCRETE_IDLE_FG = "#6f7c88"   # an option that is NOT selected

# The one background every VALUE widget sits on - the numeric entry and the
# discrete switch alike - so the value column reads as one family.  Kept in a
# single place so the two cannot drift to different blacks.
VALUE_BG = "#181818"

# Saved ArduPlane parameter rows (see _build_native_group).  Write changes the
# aircraft, so it is warm where Read and Copy are the cool accent; a box whose
# text is not the value the FC last confirmed is drawn in the pending colour.
NATIVE_WRITE_BG = "#2e2a1c"
NATIVE_WRITE_FG = "#e0c070"
NATIVE_PENDING_FG = "#e0a040"


def _is_infinity_word(text):
    """True if `text` SPELLS infinity rather than overflowing to it.

    'inf' and '1e999' parse to the same float, so only the raw text separates an
    intended infinity from a mistyped exponent.  Accepts what float() accepts,
    plus the symbol."""
    return text.strip().lstrip('+').lower() in ('inf', 'infinity', '\u221e')

# Tuner row height, sized for the tuner's font.
_TUNER_ROW_H = 40

# Tuner window width and fixed visible (viewport) height.  The full content is
# taller than _TUNER_VIEW_H; the renderer shows a scrollable slice.
_TUNER_WIN_W = 560
_TUNER_VIEW_H = 760

# Font scales (one step larger than the original 0.45/0.33/0.55/0.4 set).
_FONT_MAIN = 0.6     # param label/value rows
_FONT_SMALL = 0.45   # hints ([click to switch], [type value...])
_FONT_TITLE = 0.72   # group titles
_FONT_HINT = 0.55    # bottom reset hint



# Per-parameter info text shown by the GUI's info ("i") buttons.  Keyed by the
# tuner attr name.  Plain-language explanation of what each parameter does.
PARAM_INFO = {
    # --- CONTROL OVERRIDE ---
    'aileron_override':
        "Takes the AILERON away from the law and gives it to you. OFF (0) is "
        "normal flight. ON (1) writes Aileron Deflection straight to whichever "
        "channel carries SERVOn_FUNCTION 4, found at run time. The aileron has "
        "its own channel, so this leaves pitch and yaw entirely with "
        "ArduPlane - but roll is unstabilised while it is on. For bench checks "
        "and open-loop plant identification, where a clamp reshaping a step "
        "would identify the clamp instead of the aircraft. The override "
        "carries a 200 ms timeout refreshed every tick, so if the script stops "
        "the surface returns to ArduPlane within 200 ms by itself. Leave it "
        "OFF for scored runs; cs_flags bit 7 records when any override was on.",
    'aileron_deflection_pct':
        "Aileron surface percent while Aileron Override is ON, -100 to +100, "
        "mapped through that channel's own TRIM/MIN/MAX. Ignored when the "
        "override is off.",
    'elevator_override':
        "Takes the ELEVATOR away from the law. On this V-tail there is no "
        "elevator channel - elevator and rudder are mixed onto the two "
        "ruddervators - so switching this on takes BOTH tail channels and "
        "pitch stabilisation goes with them. If Rudder Override is off at the "
        "same time, the rudder is HELD at the value it had when you flipped "
        "this switch, recovered by inverting the mix from the live ruddervator "
        "outputs on that tick. That is a hold, not stabilisation.",
    'elevator_deflection_pct':
        "Elevator command while Elevator Override is ON, -100 to +100. Mixed "
        "with the rudder onto the two ruddervators as ArduPlane would, "
        "vtail_right = (elevator - rudder) * MIXING_GAIN and vtail_left = "
        "(elevator + rudder) * MIXING_GAIN, so a non-zero value moves both "
        "tail surfaces together. Ignored when the override is off.",
    'rudder_override':
        "Takes the RUDDER away from the law. On this V-tail there is no rudder "
        "channel - elevator and rudder are mixed onto the two ruddervators - "
        "so switching this on takes BOTH tail channels and pitch stabilisation "
        "goes with them. If Elevator Override is off at the same time, the "
        "elevator is HELD at the pitch trim it had when you flipped this "
        "switch, recovered by inverting the mix from the live ruddervator "
        "outputs on that tick. That is a hold, not stabilisation: pitch will "
        "wander as speed changes, so keep the test short.",
    'rudder_deflection_pct':
        "Rudder command while Rudder Override is ON, -100 to +100. Mixed with "
        "the elevator onto the two ruddervators, so it moves them "
        "differentially. MIXING_GAIN is applied exactly as ArduPlane applies "
        "it, which means a given percent produces the same surface deflection "
        "it did under the law and every measurement taken so far stays "
        "comparable. Positive yaws the nose LEFT on this airframe and adds "
        "about +0.09 deg of sideslip per percent, measured over several "
        "flights. Ignored when the override is off.",
    # --- YAW: coordination and pointing ---
    'yaw_coordination_ff':
        "Yaw Coordination FF (dimensionless; 1 = the geometry). The yaw command "
        "is the WHOLE body yaw rate ArduPlane's rudder loop holds, so the yaw the "
        "aircraft needs to fly clean has to be asked for. This adds it: the "
        "turn's own yaw rate, g*sin(roll)*cos(pitch)/V (V floored at "
        "AIRSPEED_MIN), and the yaw that makes a roll turn about the flight "
        "path instead of the nose, roll rate x tan(the path's angle below the "
        "nose). At 0 nothing asks for them: with pointing on, the rudder loop "
        "holds the yaw at the pointing term's value and fights every turn and "
        "every roll, and each roll about the nose turns the angle between nose "
        "and path into sideslip, which moves every error the laws read from the "
        "path. Lower than 1 under-coordinates (the turn and the roll still slip "
        "the same way, less); higher over-drives (the slip reverses). Any value "
        "but 0 runs the rudder loop, with pointing on or off. The whole yaw "
        "command stays inside ACRO_YAW_RATE. Recommended: 1. "
        f"Default {YAW_COORDINATION_FF:g}.",
    'yaw_point_kp':
        "Proportional gain of the rudder POINTING term, deg/s of yaw per degree "
        "of path-to-balloon error - the same error the roll axis closes. Bank to "
        "turn is the sword; this is the scalpel, a bounded assist that buys "
        "quickness on the transient. It cannot be the primary steering: the "
        "rudder moves the flight path only through sideslip, runs out of it at "
        "about 9 degrees, and its fast effect - rotating the nose - cancels out "
        "of path-minus-balloon entirely, since the nose is in both halves of that "
        "difference. Default 0, dialled in from here.",
    'yaw_point_ki':
        "Integral gain of the rudder pointing term, 1/s. This puts a SECOND "
        "integrator on an error the roll axis is already integrating. The "
        "equilibrium is still unique - both reach zero - but the split between "
        "them is decided by saturation rather than by design: the rudder's "
        "integral fills Yaw Point Limit first and roll takes the remainder. That "
        "is wanted here, because roll's own integrator is clipped and cannot "
        "always finish; it is also why the limit matters more than the gain.",
    'yaw_point_lim':
        "The rudder's share of the airframe's sideslip budget, deg/s, and the "
        "thing that actually decides how much of the steering it takes. The "
        "airframe has about 9 degrees of sideslip at full rudder, so a useful "
        "value is only a slice of it. Past roughly 7.4 deg/s the airframe has no "
        "more slip to give and the extra buys drag, not turn. A value of 0 "
        "disables the term (zero pointing authority), not 'no ceiling'.",
    'yaw_point_windup':
        "Anti-windup limit (0-1) for the POINTING integrator, as a fraction of "
        "Yaw Point Limit. 1 lets the integral fill the whole limit; lower "
        "reserves that share for the Kp part instead. Down caps how much steady "
        "steering the integral alone can hold.",
    'curve_flyer_enable':
        "Curve Flyer. On: a fresh p or x chase holds its roll and pitch "
        "rates at zero while the Curve Planner works out the turn for the "
        "selected Steering Law, then that law flies the turn on the FC - its "
        "own roll, pitch, rudder and Bank Lim, the bank set by the plan - until "
        f"the flight path points within {CURVE_TURN_DONE_DEG:g} deg of the "
        "balloon, however long that takes; Transfer Control When Done decides "
        "what flies from there. If no plan arrives within Plan Wait, the law "
        "flies the chase without one. Leaving p or x, for any mode, hands over "
        "at once and drops the plan. Straight and manual never use it. Off: "
        "the law flies every chase without a plan from its first tick.",
    'cf_transfer_when_done':
        "Transfer Control When Done. On: when the planned turn is complete "
        f"(the flight path within {CURVE_TURN_DONE_DEG:g} deg of the balloon) "
        "the steering law flies the rest of the chase without the plan. Off: "
        "the Curve Flyer keeps the law aimed at the line the flight path had "
        "at that moment - wings level on it, banked back toward it when the "
        "path drifts off - until the chase ends; the balloon is no longer "
        "steered toward. Turning it On while the line is held hands over on "
        "the next tick; turning it Off after the hand-over does not take "
        "control back. Changing mode hands over whatever this switch says.",
    'cf_plan_wait_s':
        "Plan Wait (s). How long a fresh p or x chase holds straight - roll "
        "and pitch rates at zero - waiting for the Curve Planner's plan. If "
        "none arrives in that time the steering law flies the chase without "
        "one, and a plan arriving later is not taken. Longer gives a slow "
        "planner time to finish but flies straight, not turning, for as long "
        "as it waits; the console's '[curve] plan N: ... planned in X s' line "
        "says how long a plan took. 0 never waits: every chase is flown "
        "without a plan.",
    'cf_approach_deg':
        "Approach Angle (deg below the horizon). The dive the Curve Planner "
        "ends a p or x chase's turn on. A balloon that starts below the horizon "
        "but less far below than this is flown with a run-in: the steering law "
        "turns the flight path level toward the balloon's compass direction, "
        "holds that line while the balloon sinks lower as the aircraft closes, "
        "then dives onto it, so the line from the aircraft to the balloon ends "
        "this far below the horizon. A balloon this far below or further, at or "
        "above the horizon, or above the canopy (rule 2) is flown as without it. "
        "Higher dives steeper and runs in longer first; 0 is shallower than "
        "every balloon below, so no chase runs in. In x the balloon's distance "
        "is not known and the run-in is planned as if it sat on the ground "
        "along its line of sight: a nearer balloon ends steeper. Read by the "
        "planner only; the FC flies the plan it makes.",
    'tilt_back_travel_deg':
        "Tilt Back Travel (deg). The Curve Planner's limit on how far the "
        "flight path's angle above or below the horizon may come back from the "
        "furthest it has reached in the direction it is tilting (CLAUDE.md, the "
        "rules of a chase): tilted from level to 40 deg up, the path never "
        "again goes below 40 deg minus this. Lower forbids more turns that "
        "climb or dive past the balloon and come back; 0 forbids any coming "
        "back. Read by the planner only; the FC flies the plan it makes.",
    # --- THE STEERING LAW, AND LIFT VECTOR POINTING ---
    'steering_law':
        "Which law steers every mode at once. 0: the cascade - bank-to-turn "
        "and roll-to-plane on the roll axis, the pitch pointing law, and yaw "
        "coordination and pointing on the rudder. 1: Lift Vector Pointing, "
        "version B - one law that rolls the wing's lift to face the push the "
        "turn needs, read in the flight path's own frame, rudder held at zero "
        "(its own section below). It holds the bank to Bank Lim in every chase, "
        "measured as the cascade measures it. A change is a fresh chase: both laws start "
        "from scratch, and any plan in force is dropped: with the Curve Flyer "
        "on, the law selected flies the p and x turns the Curve Planner plans "
        "for it, the banks in its own measure. Under 1 the pitch trim's lift "
        "term reads the load the law asks for instead of cos(pitch), so re-fit "
        "the trim map ('e') with this set to 1.",
    'att_k_path':
        "Path Turn Gain (1/s: rad/s of flight-path turn per rad the path is "
        "off the balloon). How hard the guidance asks the path to turn toward "
        "the balloon, up to Path Rate Limit. Higher closes faster but overshoots "
        "and, with the rudder at zero, sets off more sideslip; lower is gentler "
        "and slower. 0 asks for no turn: the aircraft only holds its path "
        "against gravity.",
    'att_path_rate_max_dps':
        "Path Rate Limit (deg/s). The fastest turn the guidance asks of the "
        "flight path, however far off the balloon is. The pull or push the wing "
        "makes grows with it (about speed x rate), and ACRO_PITCH_RATE still "
        "caps what is flown. Lower turns more gently and takes longer; 0 asks "
        "for no turn at all.",
    'att_lead_s':
        "Path Lead (s). The guidance reads the balloon's angle off the path "
        "from where the path will point this far ahead, projected from the "
        "accelerometer, which sees a pull before the velocity has turned with "
        "it. It is the guidance's damping: 0 reads the path as it is and the "
        "turn overshoots more; too much eases off early and arrives slowly. "
        "With no accelerometer reading the FC reads the path as it is.",
    'att_k_roll':
        "Lift Line Roll Gain (1/s: rad/s of roll about the flight path per rad "
        "the lift line is off the force line). How hard the roll axis turns the "
        "wing's lift - out of the canopy, or out of the belly when pushing - to "
        "face the push the turn needs. Higher rolls sharper, and without rudder "
        "a fast roll swings the path round the nose (sideslip); lower lags the "
        "turn. 0 never rolls.",
    'att_k_side':
        "Sideways Turn Weight (times). The part of the turn along the "
        "horizontal across the flight path - the part that swings the heading "
        "- counts this many times its share, so the heading turns while the "
        "path is still shallow, where a bank turns it, and the climb or dive "
        "follows. 1 is no weighting; higher swings the heading round first and "
        "turns the path up or down later. Not applied within "
        f"{CURVE_POLE_FADE_DEG:g} deg of straight up or down, where the "
        "horizontal across the path has no direction.",
    'att_roll_ki':
        "Path Roll Ki (1/s). Integrator on the lift line's error, running only "
        "within Integrator Zone of the balloon, where what is left is trim and "
        "the propeller's torque (ACRO's roll loop has no integrator of its "
        "own). Higher removes a steady roll error sooner and overshoots more; "
        "0 removes the term and clears the integrator.",
    'att_pitch_ki':
        "Path Pitch Ki (1/s). Integrator on the path's error in the lift plane, "
        "within Integrator Zone of the balloon: it holds the last of the pitch "
        "trim the trim map does not. Higher removes it sooner and overshoots "
        "more; 0 removes the term and clears the integrator.",
    'att_i_zone_deg':
        "Integrator Zone (deg off the balloon). The two path integrators run "
        "only inside this angle and are cleared outside it: through the turn "
        "they would wind up and overshoot. Larger runs them earlier in the "
        "approach; 0 never runs them.",
    'yaw_point_pitch_sched':
        "Steep schedule on the pointing output. Off = the rudder pointing term "
        "acts at full strength on any path. On = the output is multiplied by the "
        "roll blend's steep share: none on a path shallower than Roll-to-Plane "
        "Steep Start, all of it past Steep Full, a straight line between - the "
        "same number that hands the roll command from banking to roll-to-plane, "
        "read from the flight path's angle to the horizon (the nose's pitch "
        f"below {FLIGHT_PATH_MIN_SPEED_MPS:g} m/s). Steep, banking has little "
        "gravity across the path to work with and the rudder steers at any "
        "pitch. With Steep 90 / 90 it is 0 except moving straight up or down.",
    'bank_lim_deg':
        "Maximum bank angle (deg), as both steering laws measure it: the wing "
        "tip's angle below the horizon, the bank proxy "
        "asin(sin(roll)cos(pitch)) - the real bank when level; at steep pitch "
        "the same proxy takes more roll, by design. The proxy cannot exceed "
        "asin(cos(pitch)), so at steep pitch the limit is out of its reach and "
        "the roll goes on as far as the turn needs: that is how bank-to-turn "
        "steers near vertical, and how Lift Vector Pointing (Steering Law 1) "
        "rolls the lift round to the balloon there. Upside down with the wings "
        "level reads as no bank. It sets the laws' AUTHORITY, not safety: a "
        "smaller limit overshoots less by turning less, and load-factor "
        "protection is ArduPlane's own job, not this value's. Under Lift "
        "Vector Pointing a turn that wants more aims at the limit itself, and "
        "the roll can carry a few degrees past it before it settles; a chase "
        "that starts banked beyond it may roll back toward it but never "
        "further out. Both laws hold it through a Curve Flyer turn too: a "
        "planned bank past it is flown at it.",
    'pan_p_gain':
        "Proportional bank gain (deg of bank per deg of pan error). "
        "Higher = harder turn-in for a given pointing error.",
    'pan_ki':
        "Integral gain on the pan error (deg of bank per deg*s of accumulated "
        "error, i.e. 1/s). Nulls steady-state pointing offset over time. Too "
        "high causes windup/overshoot.",
    'windup_limit_pan':
        "Anti-windup limit (0-1) for the pan integrator, as a fraction of the "
        "bank limit. Caps how much authority the integrator can accumulate.",
    # --- Roll inner ---
    'roll_gain':
        "Inner-loop roll-rate gain: body-roll-rate demand per degree of bank "
        "error. Higher = snappier roll tracking of the commanded bank. "
        "Bank-to-turn only: roll-to-plane has its own (Roll-to-Plane Gain).",
    'pan_kd':
        "Lateral-error derivative (lead) gain on the bank command. Subtracts a "
        "term proportional to how fast the pointing error is closing, so the "
        "aircraft rolls wings-level BEFORE the error reaches zero - arriving at "
        "ty=0 already level instead of still banked. Suppresses the turn-overshoot "
        "oscillation. 0 = off; increase until the overshoot at the balloon "
        "disappears without making the roll sluggish.",
    'pan_kd_rate':
        "Low-pass RATE (1/s) for the lateral-error derivative. Bigger = snappier "
        "(less smoothing) but more noise in the lead term; smaller = smoother "
        "with more lag; 0 = frozen. It is 1/time-constant, so it grows in the "
        "direction of responsiveness. ~10 (1/s) is a reasonable start.",
    # --- PITCH stack: one body-plane law at every attitude ---
    'pitch_gain':
        "Proportional pitch gain: deg/s of body pitch-rate demand per degree of "
        "in-plane pointing error (theta_ip).",
    'pitch_ki':
        "Integral gain on the in-plane pitch pointing error, 1/s: deg/s of "
        "pitch-rate demand added per second a 1 deg error is held, as Pan Ki, "
        "Yaw Point Ki and Roll-to-Plane Ki. Nulls a steady pitch offset onto the "
        "aim, such as the trim a speed or attitude change leaves. PARALLEL - "
        "added to the rate demand directly, so it is independent of Pitch Gain "
        "(raising P does not amplify it). A big error winds it in proportion, so "
        "Pitch Windup's ceiling and the freeze at the pitch-rate limit are what "
        "stop a large manoeuvre from winding it up.",
    'windup_limit_pitch':
        "Anti-windup limit (0-1) for the pitch integrator, as a fraction of the "
        "pitch rate the aircraft can deliver. Caps how much of that rate the "
        "integrator alone may command.",
    'pitch_kd':
        "Pitch-error derivative (damping) gain, DIMENSIONLESS: deg/s of "
        "pitch-rate command per deg/s of how fast the pointing error is "
        "closing. Subtracts a term as the error closes, so the aircraft arrives "
        "at the aim carrying less pitch rate and overshoots less on a steep "
        "capture. The sibling of Pan Kd, but dimensionless because pitch's "
        "output is a RATE, not an angle. 0 = off; raise until the "
        "steep-capture overshoot disappears without making the pull sluggish.",
    'pitch_kd_rate':
        "Low-pass RATE (1/s) for the pitch-error derivative. Bigger = snappier "
        "(less smoothing) but more noise in the damping term; smaller = smoother "
        "with more lag; 0 = frozen. It is 1/time-constant, so it grows in the "
        "direction of responsiveness. ~10 (1/s) is a reasonable start; in the "
        "p-test the aim bearing is only 5 Hz, so do not push it high there.",
    'pitch_trim_offset':
        "Pitch trim, constant part: elevator added at every airspeed and attitude, "
        "in ArduPlane's elevator units (%g = full travel, positive = nose-up, the "
        "units of the fc_pitch_* log columns). The whole trim is\n"
        "  Offset + (VREF / airspeed)^2 x (Lift x lift term + Attitude x "
        "sin(pitch) + Thrust x throttle x density ratio)\n"
        "with VREF = %g m/s (PITCH_TRIM_VREF_MPS), " % (ELEV_FULL_TRAVEL,
                                                       PITCH_TRIM_VREF_MPS) +
        "the lift term cos(pitch) under the cascade and, under Lift Vector "
        "Pointing, the load the law asks of the wing (the "
        "pitch_trim_lift_term log column), "
        "airspeed in m/s EAS, throttle as a fraction, and the density ratio "
        "the air's density as a fraction of sea level's (1 / EAS2TAS^2, the "
        "lua_eas2tas log column); it is supplied up "
        "front so the pointing law does not have to sit off the balloon to "
        "produce it - with PTCH_RATE_IMAX at 0 the rate loop has no other source "
        "of trim. Too little trim parks the aim below the balloon; too much, "
        "above. The trim map ('e') measures what it should be; its report fits "
        "all four numbers. All four 0 is no trim.",
    'pitch_trim_lift':
        "Pitch trim, the wing's share: elevator units x the lift term at VREF "
        "(%g m/s), growing as (VREF / airspeed)^2 below it and shrinking above. "
        "It is the " % PITCH_TRIM_VREF_MPS +
        "up-elevator the wing needs to make lift equal to the weight, which takes "
        "more nose-up the slower the aircraft flies. The lift term is cos(pitch) "
        "under the cascade and, under Lift Vector Pointing, the load the law asks "
        "of the wing. Too small shows as the aim "
        "sinking below the balloon at LOW airspeed while fast flight is fine. See "
        "Pitch Trim Offset for the whole formula; the trim map's report fits it.",
    'pitch_trim_attitude':
        "Pitch trim, the attitude share: elevator units x sin(pitch) at VREF "
        "(%g m/s), scaled by (VREF / airspeed)^2. " % PITCH_TRIM_VREF_MPS +
        "Negative means more up-elevator nose-down "
        "and less nose-up, which this airframe needs. Wrong here shows as the aim "
        "parked off the balloon in steep climbs or dives while level flight is "
        "fine. See Pitch Trim Offset for the whole formula; the trim map's report "
        "fits it.",
    'pitch_trim_thrust':
        "Pitch trim, the motor's share: elevator units x throttle (0 to 1) at "
        "VREF (%g m/s) and sea-level air, scaled by (VREF / airspeed)^2 and by "
        "the air's density as a fraction of sea level's. " % PITCH_TRIM_VREF_MPS +
        "At one throttle the propeller pushes less in thinner air, so the same "
        "throttle needs less of this share higher up. "
        "Negative means the motor pitches "
        "the nose up, so more throttle needs less up-elevator. Wrong here shows "
        "as the aim moving off the balloon when the throttle changes at the same "
        "airspeed and attitude. See Pitch Trim Offset for the whole formula; the "
        "trim map's report fits it.",
    'pitch_turn_ff':
        "Scale on the body pitch rate a TURN produces by itself. A banked "
        "aeroplane rotates nose-up in its own axes purely because it is turning "
        "- nothing commands it and nothing can stop it - so the pointing law's "
        "demand has to sit on top of it. Ask for zero in a bank and the rate "
        "loop spends elevator resisting the turn's own pitch rate, which the "
        "pointing law can only make up by sitting off the balloon. It is the "
        "coordinated-turn YAW rate times the tangent of the bank angle, because "
        "both are one heading rate resolved into body axes. 1.0 is the geometry; "
        "more over-drives it, which is a choice, not an impossibility. Being a "
        "feedforward it costs no phase and cannot destabilise the loop; a wrong "
        "value biases the aim and nothing more. Not applied past "
        "PITCH_TURN_FF_BANK_LIM_DEG of bank, where a level turn needs more than "
        "2 g and the aircraft is no longer flying the geometry this describes.",
    'nose_cam_display':
        "'Balloon Chaser Camera' (nose view) WINDOW. 0 = OFF (DEFAULT), 1 = ON. Display "
        "only - detection runs off the GIMBAL camera, so this cannot affect "
        "control at any setting. Switching it off also stops the nose camera "
        "DECODING: the per-frame colour convert and resize to display size cost "
        "~1.0 ms each (the resize is 0.98 of that), plus a 0.21 ms frame copy and "
        "~0.16 ms of overlay drawing every tick at 50 Hz - about 5% of a core at "
        "30 fps before cv2.imshow, whose cost under WSLg is larger again. The "
        "subscription stays up, so switching back on is immediate and the boot "
        "never hangs waiting for a frame.",
    'gimbal_cam_display':
        "'Gimbal Camera' WINDOW. 0 = OFF, 1 = ON (DEFAULT). This switches the "
        "WINDOW only - the gimbal camera itself keeps running and decoding "
        "because HsvDetectionSensor reads it and that IS the control path. "
        "Saves the per-tick overlay draw and the imshow/compositor cost, not the "
        "decode. Switching BOTH camera views off is allowed and is the way to "
        "reclaim the most: the keyboard keeps working, because this Gain Tuner "
        "window takes the keys when no camera window is open.",
    # --- Autotune maneuver grid ---
    'autotune_bracket_wide':
        "Whether each gain is searched on a WIDE net, for both 't' and 'y'.\n\n"
        "OFF (default): the narrow bracket - 5 candidates at x0.5 .. x2.0 of "
        "the anchor. A gain more than 2x from its best cannot be found in one "
        "run.\n\n"
        "ON: the wide bracket - 9 candidates at x0.2 .. x5.0 - reach x5 - for "
        "1.8x the flight time ('y' 60 -> 108 trials per pass, 't' 540 -> 972). "
        "Turn it on when a gain may be badly off; not for routine tuning.\n\n"
        "9 points rather than 5 spread wider: the headroom gate reads the next "
        "candidate UP, so the step ratio IS the gate. 9 points keeps it at "
        "1.50 (today's is 1.40); 5 would make it 2.24 and start rejecting "
        "good gains.",
    'autotune_use_constants':
        "Where the 't' grid sweep starts.\n\n"
        "ON (default): reset the 12 swept gains to their constants.py values "
        "first, and anchor every candidate bracket to those values. The run is "
        "reproducible and does not depend on what the Gain Tuner was showing.\n\n"
        "OFF: keep the Gain Tuner's CURRENT values and anchor every bracket to "
        "them instead. constants.py is not consulted for the swept gains, so its "
        "values are NOT guaranteed to be among the candidates - the search has to "
        "find them. Use this to check whether the mechanism can recover a known-"
        "good set from a deliberately spoiled one.\n\n"
        "The anchors are written into the checkpoint, so an interrupted run "
        "resumes on the same brackets instead of falling back to constants.",
    'cell_autotune_use_constants':
        "The same switch for the 'y' single-cell autotune, kept separate so the "
        "two mechanisms can be configured independently.\n\n"
        "Plant-ID ('i') has no equivalent: it measures the plant rather than "
        "searching gains, so it has no starting gain set to choose.",
    'autotune_cell_lon_deg':
        "Single-cell autotune ('y'): the lateral (longitude) bearing it tunes at, "
        "deg. Same spherical frame as the 'p' test - heading-relative azimuth.\n\n"
        "'y' runs the same engine as the 't' sweep - same gains, same coordinate "
        "descent, same cost, same climb protection, same checkpoint/resume, and "
        "winners applied live - but flies ONE cell instead of the grid. The answer "
        "is the best achievable AT THIS BEARING; it carries no envelope guarantee, "
        "so it diagnoses a bad cell rather than producing a shippable gain. Writes "
        "suggested_constants_y.py and its own checkpoint, separate from 't'.",
    'autotune_cell_lat_deg':
        "Single-cell autotune ('y'): the elevation (latitude) bearing it tunes at, "
        "deg, NEGATIVE = nose-down - the same sign convention as the 'p' test and "
        "the 't' grid's tilt.\n\n"
        "'y' refuses to tune a cell that spends more than "
        "AUTOTUNE_CELL_MAX_SAT_FRAC of its scored window at the bank limit: a "
        "saturated trial is open-loop, so every candidate scores the same and "
        "there is no best to find. Checked at the entry gains before tuning "
        "starts.",
    'autotune_pan_min':
        "Controller-Autotune grid: smallest lateral (pan) bearing tested (deg). The "
        "sweep tests every gain across the full pan x tilt cross-product.",
    'autotune_pan_max':
        "Controller-Autotune grid: largest lateral (pan) bearing tested (deg). "
        "Larger = more aggressive lateral demand.",
    'autotune_pan_step':
        "Controller-Autotune grid: pan increment (deg) from min to max. Smaller = "
        "more grid cells = longer sweep. <=0 or min==max gives a single pan value.",
    'autotune_tilt_min':
        "Controller-Autotune grid: smallest dive (tilt) bearing tested (deg).",
    'autotune_tilt_max':
        "Controller-Autotune grid: largest dive (tilt) bearing tested (deg).",
    'autotune_tilt_step':
        "Controller-Autotune grid: tilt increment (deg) from min to max. Smaller = "
        "more grid cells = longer sweep. <=0 or min==max gives a single tilt value.",
    # --- Plant-ID and altitude protection (shared by 't' and 'i') ---
    'plantid_do_pitch':
        "Plant-ID: include the PITCH axis in the 'i' run. On = fly open-loop "
        "elevator doublets and identify the pitch short-period (control power, "
        "damping, feedforward, overshoot floor). Off = skip the pitch stage.",
    'plantid_do_yaw':
        "Plant-ID: include the YAW axis in the 'i' run. On = fly open-loop rudder "
        "doublets and identify the yaw plant (control power, damping, overshoot "
        "floor). Off = skip the yaw stage. Newest / least validated axis.",
    'plantid_do_roll':
        "Plant-ID: include the ROLL axis in the 'i' run. On = fly open-loop aileron "
        "doublets and identify the roll plant (control power, damping, feedforward, "
        "overshoot floor). Off = skip the roll stage.",
    'plantid_rate_agg':
        "Plant-ID rate-PID aggressiveness (dimensionless multiple of the reference "
        "rate-loop bandwidth). It maps the identified plant to the suggested "
        "<AXIS>_RATE_P and _D in the report: higher = a faster, harder rate loop "
        "(more P, more damping D); lower = softer. 1.0 reproduces the flight-"
        "validated PTCH_RATE_P=1.0; the report also prints a sensitivity table "
        "around your setting. Read only when the report is generated - it changes "
        "no flight behaviour and no stored gain, and applies nothing. Recommended "
        "0.7-2.0; leave at 1.0 unless a run shows the loop too soft or ringing.",
    # --- Trim map ('e') ---
    'trim_map_climb_min_deg':
        "Trim map ('e'): the lowest climb angle flown, deg above the horizon - "
        "NEGATIVE is a dive. Each cell holds the flight path on a fixed bearing "
        "dead ahead at this angle and records the elevator ArduPlane needs to "
        "hold it. A steeper dive measures more of the envelope but loses height "
        "fast and may outrun the throttle (the cell then ends 'overspeed', not "
        "held). Read when a run starts.",
    'trim_map_climb_max_deg':
        "Trim map ('e'): the highest climb angle flown, deg above the horizon. A "
        "steeper climb measures more of the envelope, but at a low airspeed the "
        "aircraft may run out of thrust (the cell then ends 'underspeed', not "
        "held). Read when a run starts.",
    'trim_map_climb_step_deg':
        "Trim map ('e'): climb-angle increment, deg, from min to max. Smaller = "
        "more cells = a longer run and a finer map. 0 flies the minimum alone.",
    'trim_map_speed_min_mps':
        "Trim map ('e'): the lowest airspeed flown, m/s EAS, held by the FC's "
        "airspeed hold (throttle only) for the run. Lower reaches nearer the "
        "stall, where the elevator needed changes fastest, and a steep climb there "
        "may not hold. Read when a run starts.",
    'trim_map_speed_max_mps':
        "Trim map ('e'): the highest airspeed flown, m/s EAS. Higher needs more "
        "throttle; a climb the motor cannot hold at this speed ends 'airspeed "
        "unreachable', not held. Read when a run starts.",
    'trim_map_speed_step_mps':
        "Trim map ('e'): airspeed increment, m/s, from min to max. Smaller = more "
        "cells = a longer run. 0 flies the minimum alone.",
    'trim_map_protect_floor_m':
        "Trim map ('e') altitude-protection floor, metres AGL. When height falls "
        "below this - before a cell or during one - the run climbs back to the "
        "Climb-To altitude first, and re-flies a cell the floor interrupted. Lower "
        "means fewer climbs but less margin under a dive cell, which loses height "
        "fast; too low risks the ground. Must be below Climb-To; 0 means the floor "
        "never trips (protection off). Read at every check.",
    'trim_map_protect_climb_to_m':
        "Trim map ('e') altitude-protection Climb-To altitude, metres AGL. When the "
        "floor trips, the run climbs to this in FBWA at full throttle, then carries "
        "on. Leave enough gap above the floor for the steepest dive cell's height "
        "loss - too close and it re-trips every dive. Higher means longer climbs.",
    'autotune_protect_floor_m':
        "Altitude-protection floor, metres AGL, shared by the autotune sweep ('t') "
        "and the plant-ID ('i'). When height falls below this - between or during "
        "manoeuvres - the run pauses, climbs back to the Climb-To altitude, and "
        "carries on. Lower means fewer climb interruptions but less margin if a "
        "manoeuvre drops the nose; too low risks the ground. Set it well below your "
        "working altitude with room for a dive to overshoot it. Must be below "
        "Climb-To; 0 means the floor never trips (protection off).",
    'autotune_protect_climb_to_m':
        "Altitude-protection Climb-To (resume) altitude, metres AGL. When the "
        "floor trips, the run climbs to this in FBWA at full throttle and then "
        "resumes. Set it above the floor with enough gap that one climb clears a "
        "manoeuvre's height loss - too close to the floor and it re-trips every "
        "manoeuvre. Higher means longer climbs and more time between measurements.",
    # --- Sensor / shared ---
    'gimbal_slew_rate':
        "Maximum gimbal slew rate (deg/s) for pan/tilt tracking of the balloon.",
    'auto_gimbal':
        "Auto Gimbal. On (default) = the controller may automatically slew the "
        "gimbal to track the balloon while vision-tracking (l or x). Off = it "
        "must NOT move the gimbal in l/x. The 'g' freeze and operator mouse "
        "aiming are unaffected either way.",
    'fov_est_enable':
        "FOV Estimator (DIAGNOSTIC ONLY). On = a passive background observer "
        "reads the camera's horizontal FOV out of motion that is already "
        "happening - aircraft rotation and gimbal slew vs. the pixel shift - and "
        "writes fov_est_deg / fov_est_m1_deg (aircraft) / fov_est_m2_deg (gimbal) "
        "and fov_truth_deg to the q CSV log for post-run comparison. It NEVER "
        "moves anything and its output NEVER feeds the control law. Off (default) "
        "= not running (columns stay 0).",
    # --- Manual / operator ---
    'special_throttle_enable':
        "Fly 'p' tests at Special Throttle instead of the normal Throttle %. "
        "Off by default. Reverts by itself when the test ends - the normal "
        "throttle is never overwritten, so an aborted test cannot leave a "
        "setting behind.",
    'special_throttle_pct':
        "Throttle % a 'p' test flies at when Enable Special Throttle is on. "
        "Useful because a climb and a dive at the SAME throttle settle at very "
        "different airspeeds, and pan authority goes as 1/airspeed - so the two "
        "halves of a comparison end up at different loop gains for reasons that "
        "have nothing to do with the gains. Ignored when the switch is off.",
    'throttle_pct':
        "Operator throttle percentage (0-100) applied to the throttle channel. "
        "With Airspeed Command OFF this goes to the throttle channel unmodified "
        "and nothing computes throttle. With it ON this becomes the loop's "
        "output and the operator commands airspeed instead.",
    'airspeed_cmd_enable':
        "WHICH QUANTITY YOU COMMAND. Off = throttle (Throttle % goes straight to "
        "the throttle channel). On = airspeed: you set Airspeed Cmd and a PI loop "
        "drives THROTTLE to hold it. The loop's only actuator is the throttle "
        "channel - it never touches pitch, roll or the steering error, so it "
        "cannot buy airspeed by steering. If the speed is unreachable at the "
        "attitude the balloon chaser law is holding, the throttle saturates and the "
        "speed is missed.",
    'airspeed_cmd_mps':
        "Commanded airspeed (m/s) when Airspeed Command is On. Ignored when Off.",
    'airspeed_cmd_kp':
        "Airspeed loop proportional gain: throttle % per m/s of error.",
    'airspeed_cmd_ki':
        "Airspeed loop integral gain: throttle % per m/s per second. Removes the "
        "standing error a fixed drag/weight state leaves.",
    'point_lon_deg':
        "WHERE the 'p'-test balloon sits, around the aircraft: LONGITUDE in deg. "
        "0 = straight ahead, +90 = right, 180 = dead astern, -90 = left. "
        "Referenced to the heading at injection, so 0 is 'ahead' on any course. "
        "NOT a WGS84 coordinate - it is a bearing on a sphere centred on the "
        "aircraft. Any value works: this is a world-space bearing for the oracle, "
        "NOT a gimbal command, so the gimbal's travel does not bound it.",
    'point_lat_deg':
        "WHERE the 'p'-test balloon sits, up or down: LATITUDE in deg. "
        "+90 = straight up, 0 = level with the horizon, -90 = straight down. "
        "Referenced to the world horizon, so it means the same at any attitude. "
        "NOT a WGS84 coordinate. The two angles cover the full sphere: lat=180 "
        "reaches astern over the top (same direction as lon=180).",
    'point_balloon_move_enable':
        "Master ON/OFF for a MOVING 'p'-test balloon (0 = stationary, the default; "
        "1 = moving).  When OFF the balloon sits at its anchor whatever the speed is; "
        "turn it ON to make the balloon translate at 'Movement Speed' along the "
        "'Movement Bearing'/'Movement Elevation' direction.  Off by default so a set "
        "speed never moves the balloon unexpectedly.",
    'point_balloon_speed':
        "'p'-test balloon speed (m/s) - the MAGNITUDE of the velocity; 'Movement "
        "Bearing' and 'Movement Elevation' give its direction, and the three "
        "together are one 3-D velocity in polar form.  0 = stationary.  Motion "
        "only: it does NOT affect where the balloon starts - 'Distance m' alone "
        "decides that.  IGNORED while the range is "
        "inf, because a balloon at infinity has no position to move; the value is "
        "kept and applies as soon as a finite range is set.",
    'point_balloon_range':
        "How far along the (Longitude, Latitude) bearing the 'p'-test "
        "balloon sits - the ONE knob that decides distance, moving or stationary.  "
        "Accepts a number in metres, or the word 'inf'.\n\n"
        "The rule is:  distance = min(this, distance to the ground).  A balloon "
        "cannot start underground, so the ground bounds every range.\n\n"
        "  * A NUMBER puts the balloon exactly there.  If the ground is farther "
        "away it simply floats in the air, which is a fine airborne-balloon test "
        "and is MORE repeatable across altitudes than a ground point, since the "
        "geometry relative to the aircraft is identical every run.  A closer "
        "balloon also induces a higher bearing rate for the same speed, so this "
        "scales how hard a moving balloon is to track.\n"
        "  * A number BIGGER than the height allows is capped onto the ground "
        "(height / sin of the depression) and says so in the log.\n"
        "  * 'inf' is not an exception to the rule, it is the largest possible "
        "range.  On a DOWNWARD bearing the cap lands it on the ground - "
        "altitude-adaptive, no number to type.  On a LEVEL or UPWARD bearing "
        "nothing caps it, so it stays a fixed world BEARING the aircraft can "
        "never reach and holds a steady constant-angle lock instead of "
        "overflying anything.\n\n"
        "Any finite distance needs GPS lock + above-home; an infinite one needs "
        "neither.  Balloon motion is ignored while this reads inf.",
    'point_balloon_vel_pan':
        "WHICH WAY a moving balloon travels, in azimuth (deg, +right / -left) - the "
        "direction half of the velocity, with 'Movement Speed m/s (0-50)' as its magnitude. "
        "Same heading-relative frame as Longitude, frozen at injection.  90 = "
        "crossing right, 180 = receding straight back, 0 = closing head-on.",
    'point_balloon_vel_tilt':
        "WHICH WAY a moving balloon travels, in elevation (deg, -down / +up), "
        "referenced to the world horizon.  0 = level drift (the balloon-in-wind "
        "case); + climbs, - descends.",
    'arrow_left_deg':
        "Heading/step change (deg) applied by the Left arrow in manual mode.",
    'arrow_right_deg':
        "Heading/step change (deg) applied by the Right arrow in manual mode.",
    'arrow_up_deg':
        "Pitch/step change (deg) applied by the Up arrow in manual mode.",
    'arrow_down_deg':
        "Pitch/step change (deg) applied by the Down arrow in manual mode.",
    'draw_roll_error':
        "HUD toggle: draw the roll-error indicator overlay (0 = off, 1 = on).",
    'plane_side_hold_deg':
        "Roll-to-plane 12/6 hold band (deg). Roll-to-plane rolls the balloon to "
        "whichever of 12 o'clock (over the canopy) and 6 o'clock (under the "
        "belly) is nearer; once chosen, the choice changes only when the other "
        "end is nearer by this many degrees. At 3 and 9 o'clock both ends are "
        "equally far, and with no band a balloon drifting there swings the roll "
        "command from full one way to full the other. Bigger = steadier choice, "
        "but a balloon can sit further round the clock on the far side before "
        "the law gives up on its end. At 90 the first choice is kept for the "
        f"whole chase. Default {PLANE_SIDE_HOLD_DEG:g}.",
    'plane_gain':
        "Roll-to-plane gain (deg/s of roll per deg still to roll to 12 or 6 "
        "round the nose). Roll-to-plane rolls the balloon to 12 o'clock round "
        "the NOSE (over the canopy; the pitch law then pulls) or 6 o'clock "
        "(under the belly; it pushes), whichever is nearer. This is its whole "
        "proportional law: a roll about the nose moves the balloon round its "
        "clock by exactly the angle rolled, so this sets how fast it closes "
        "(its time constant is 1/gain seconds). Higher = brisker roll onto the "
        "line; the roll rate is still capped by ACRO_ROLL_RATE. The clock "
        "swings for small moves near the nose (a balloon 5 deg right of the "
        "nose reads 3 o'clock, 90 deg still to roll), and this gain acts on "
        "every swing. Separate from Roll Gain, which is bank-to-turn's. "
        f"Default {PLANE_GAIN:g}.",
    'plane_ff_gain':
        "Roll-to-plane feedforward (deg/s of roll per deg/s of drift; 1 = match "
        "the drift). Adds the rate at which the balloon is sliding round the "
        "clock by itself - its own motion, or the pull moving it when it is not "
        "exactly on the line - measured as the clock angle's rate plus the "
        "aircraft's own roll rate, low-passed at Roll-to-Plane FF LPF Rate. "
        "Without it the proportional law trails a drifting balloon by "
        "drift/gain degrees. 0 = no feedforward; above 1 over-drives. Held at "
        "zero while roll-to-plane has no share of the roll command. Near the "
        "nose the clock swings for tiny moves, and so does the drift it reads. "
        f"Default {PLANE_FF_GAIN:g}.",
    'plane_ff_lpf_rate':
        "Low-pass RATE (1/s) on the drift roll-to-plane's feedforward reads. "
        "Bigger = snappier (less smoothing, more noise); smaller = smoother but "
        "later; 0 = frozen: the filtered drift holds its last value, which is "
        "zero after any tick roll-to-plane has no share. It is "
        "1/time-constant, so it grows in the direction of responsiveness. "
        f"Default {PLANE_FF_LPF_RATE:g}, the same as FF LPF Rate's default.",
    'plane_steep_start_deg':
        "Roll-to-plane steep start (deg of FLIGHT-PATH angle above or below the "
        "horizon: the direction the aircraft is moving, from the FC's velocity, "
        "not where the nose points). The master switch: on a path shallower "
        "than this, banking toward the balloon flies alone. Past it "
        "roll-to-plane's share of the roll command grows in a straight line to "
        "1 at Roll-to-Plane Steep Full, and bank-to-turn has the rest. The "
        "cosine of this angle is how much of gravity acts across the path: "
        "moving level "
        "the lift must hold the aircraft up while it turns, which banking toward "
        "the balloon does and roll-to-plane does not; moving vertically gravity "
        "only changes the speed. The bank does not enter it: knife-edge on a "
        "level path stays with banking toward the balloon. Below "
        f"{FLIGHT_PATH_MIN_SPEED_MPS:g} m/s (FLIGHT_PATH_MIN_SPEED_MPS) the "
        "nose's pitch stands in. "
        f"Default {PLANE_STEEP_START_DEG:g}. The angle cannot pass 90, so "
        "90 / 90 switches roll-to-plane off.",
    'plane_steep_full_deg':
        "Roll-to-plane steep full (deg of flight-path angle above or below the "
        "horizon). At and beyond this angle roll-to-plane has the whole share "
        "and bank-to-turn none. At or below "
        "Roll-to-Plane Steep Start the hand-over is a step at Steep Start. "
        f"Default {PLANE_STEEP_FULL_DEG:g}.",
    'plane_ki':
        "Roll-to-plane integral gain, 1/s: deg/s of roll added per second a "
        "1 deg error to 12 or 6 is held. PARALLEL - added to roll-to-plane's "
        "rate directly, independent of Roll-to-Plane Gain. Nulls a steady error "
        "the proportional law leaves, such as a roll trim or a steady drift the "
        "feedforward does not cover. Integrates while roll-to-plane has a share "
        "of the roll command, is held while the roll command is saturated its "
        "way, and is cleared while it has none, on a new chase, and at 0. It "
        "reaches the command times roll-to-plane's share, like the rest of its "
        f"rate. Default {PLANE_KI:g}.",
    'plane_windup':
        "Anti-windup limit (0-1) for the roll-to-plane integrator, as a fraction "
        "of the ACRO roll rate (ACRO_ROLL_RATE). Caps how much of that rate the "
        "integrator alone may command; 1 lets it own the whole rate. The cap is "
        "on the integrator itself: what reaches the command is that times "
        "roll-to-plane's share. Default "
        f"{PLANE_WINDUP:g}.",
    'plane_kd':
        "Roll-to-plane derivative (damping) gain, DIMENSIONLESS: deg/s of roll "
        "per deg/s of how fast the error to 12 or 6 is changing. The rate is "
        "the clock angle's own, low-passed at Roll-to-Plane Kd LPF Rate: the "
        "balloon's whole motion round the nose's clock - the roll carrying it, "
        "the nose's yaw and pitch swinging it, and its own drift. While a roll "
        "carries the balloon toward 12 or 6 the rate opposes the error, so this "
        "takes rate off the roll before the balloon arrives; a balloon swinging "
        "past the line meets a roll back against the swing. At 1 a swing of 10 "
        "deg/s round the clock moves the command 10 deg/s against it. Too low "
        "lets the roll carry the balloon past the line and swing back; too high "
        "makes the roll onto the line sluggish. Held at zero while "
        "roll-to-plane has no share of the roll command, and it reaches the "
        "command times that share, like the rest of its rate. Near the nose the "
        "clock swings for tiny moves, and so does the rate this reads. 0 = no "
        f"derivative term. Default {PLANE_KD:g}.",
    'plane_kd_rate':
        "Low-pass RATE (1/s) on the clock error's rate Roll-to-Plane Kd reads. "
        "Bigger = snappier (less smoothing) but more noise in the damping term; "
        "smaller = smoother with more lag; 0 = frozen: the filtered rate holds "
        "its last value, which is zero after any tick roll-to-plane has no "
        "share. It is 1/time-constant, so it grows in the direction of "
        f"responsiveness. Default {PLANE_KD_RATE:g}.",
}


def _detect_screen_size():
    """Best-effort (width, height) in px; falls back to (1920, 1080).  Mirrors
    the controller's detector so the tuner can size its viewport to the screen."""
    try:
        import tkinter as _tk
        r = _tk.Tk(); r.withdraw()
        w, h = r.winfo_screenwidth(), r.winfo_screenheight()
        r.destroy()
        if w > 0 and h > 0:
            return int(w), int(h)
    except Exception:
        pass
    try:
        from screeninfo import get_monitors
        m = get_monitors()[0]
        if m.width > 0 and m.height > 0:
            return int(m.width), int(m.height)
    except Exception:
        pass
    try:
        import subprocess, re
        out = subprocess.run(['xrandr'], capture_output=True, text=True,
                             timeout=2.0).stdout
        m = re.search(r'current\s+(\d+)\s*x\s*(\d+)', out)
        if m:
            return int(m.group(1)), int(m.group(2))
    except Exception:
        pass
    return 1920, 1080


# Info text for the saved ArduPlane parameter rows, by name suffix: what ArduPlane
# V4.7.0-dev's own parameter documentation says each one is (AP_PitchController /
# AP_RollController / AP_YawController var_info, AC_PID), and which way makes it
# worse.  A filter frequency of 0 is calc_lowpass_alpha_dt's alpha = 1: the
# signal passes through unfiltered.
_NATIVE_SUFFIX_INFO = {
    'RATE_FF':
        "Rate feedforward: surface moved in proportion to the DEMANDED rate, "
        "before any error exists (the plant-ID report derives it as "
        "-M_q/M_delta). Too low and the rate lags every demand while P and I "
        "make up the difference; too high and it overshoots the demand.",
    'RATE_P':
        "Proportional gain on the rate error (demanded minus achieved rate). "
        "Higher tracks the demand more stiffly and brakes harder; too high "
        "oscillates at the surfaces.",
    'RATE_I':
        "Integral gain on the rate error. It builds up the surface needed to "
        "remove a lasting error, which makes it the part that holds trim. Higher "
        "re-trims faster; too high overshoots and hunts slowly. With IMAX 0 it "
        "has no effect.",
    'RATE_D':
        "Derivative gain: acts on how fast the rate error is changing, damping "
        "the response. Too high turns gyro noise into surface jitter.",
    'RATE_IMAX':
        "The most surface the I term may hold. Its output is IMAX x 57.3 on the "
        "+-45 full-throw scale, so 0.785 is full throw and 0.666 about 85% of "
        "it; 0 holds the integrator at zero. Too small and it cannot hold the "
        "trim a flight condition needs; larger lets it store more trim, which is "
        "wrong once a manoeuvre ends.",
    'RATE_FLTT':
        "Low-pass on the DEMANDED rate, in Hz. The balloon chaser law's rate demand "
        "passes through it. Lower smooths the demand and delays it; 0 passes it "
        "through unfiltered.",
    'RATE_FLTE':
        "Low-pass on the rate ERROR, in Hz. Lower means less surface jitter and a "
        "later correction; 0 passes it through unfiltered.",
    'RATE_FLTD':
        "Low-pass on the D term's input, in Hz. Lower means less noise and later "
        "damping; 0 passes it through unfiltered.",
    'RATE_SMAX':
        "Slew-rate limit on the combined P and D action, in deg/s of surface. "
        "When P+D moves the surface faster than this, ArduPlane lowers the P and "
        "D gains until it does not, to suppress oscillation. 0 disables it.",
    '2SRV_TCONST':
        "Seconds from a demanded to an achieved attitude in ArduPlane's ANGLE "
        "modes (FBWA, and the FBWA level-outs between runs). Lower is quicker, "
        "but no lower than the airframe can follow. The balloon chaser law's rate "
        "demands go straight to the rate loop and do not pass through it.",
    '2SRV_RMAX':
        "The highest roll rate ArduPlane's attitude controller demands in its "
        "ANGLE modes, in deg/s; 0 = no limit. The balloon chaser law's rate demands do "
        "not pass through it.",
    '2SRV_RMAX_UP':
        "The highest nose-up pitch rate ArduPlane's attitude controller demands "
        "in its ANGLE modes, in deg/s; 0 = no limit. The balloon chaser law's rate "
        "demands do not pass through it.",
    '2SRV_RMAX_DN':
        "The highest nose-down pitch rate ArduPlane's attitude controller "
        "demands in its ANGLE modes, in deg/s; 0 = no limit. The balloon chaser law's "
        "rate demands do not pass through it.",
}


def native_param_info(name):
    """Info-button text for one saved ArduPlane parameter row."""
    axis, _, suffix = name.partition('_')
    if axis.endswith('2SRV'):                 # PTCH2SRV_TCONST -> 2SRV_TCONST
        axis, suffix = axis[:-4], '2SRV_' + suffix
    axis_word = {'PTCH': 'Pitch', 'RLL': 'Roll', 'YAW': 'Yaw'}[axis]
    parts = [f"{axis_word}: {_NATIVE_SUFFIX_INFO[suffix]}"]
    if axis == 'YAW':
        parts.append("Exists on the FC only with YAW_RATE_ENABLE=1.")
    default = acro_default_value(name)
    reset = (f"The reset icon writes {float32_text(default)}, its stored Acro "
             "default (constants.ACRO_DEFAULT_PARAMS)." if default is not None else
             "The reset icon writes back the value the FC held when this program "
             "started: it has no stored Acro default.")
    parts.append(
        "This value lives on the flight controller. Write sends the box's value, "
        "which the FC saves (it survives a reboot), then reads it back; Read "
        f"fetches the FC's value into the box. {reset} All three report on the "
        "console, not here. Amber text means the box holds a value that is not "
        "written yet. "
        "The flight log records the value the FC last confirmed.")
    if name in NATIVE_MODE_RATE_PIDS:
        parts.append(
            "ArduPlane's own modes (LOITER, FBWA, ...) fly "
            f"{NATIVE_MODE_RATE_PIDS[name]:g} (constants.NATIVE_MODE_RATE_PIDS): "
            "every switch into one writes it, and every switch into ACRO writes "
            "your value back - the last one a Write, a reset or the Default Acro "
            "Parameters Write put there, or the startup value. The box follows "
            "whichever the FC holds. A Write made outside ACRO flies at once, "
            "until the next mode switch.")
    if name in EXPECTED_RATE_PIDS:
        parts.append(
            "Startup compares the FC's value with constants.EXPECTED_RATE_PIDS "
            f"({EXPECTED_RATE_PIDS[name]:g}) and waits for Enter if they differ.")
    elif name in ('PTCH_RATE_FLTE', 'RLL_RATE_FLTE'):
        expected = PTCH_RATE_FLTE_EXPECTED if axis == 'PTCH' else RLL_RATE_FLTE_EXPECTED
        parts.append(f"Startup checks it against {expected:g} Hz "
                     f"(constants.{axis}_RATE_FLTE_EXPECTED) and waits for Enter "
                     "if it differs.")
    return "\n\n".join(parts)



# The Read All row at the foot of the ArduPlane parameter group, above the set
# Writes: its label and its info-button text.
READ_ALL_LABEL = 'All Parameters'
READ_ALL_INFO = (
    "Reads every parameter in this group from the FC now and puts each value "
    "in its box. The requests go out together, so it takes about as long as "
    "one Read.\n\n"
    "A box holding a typed value you have not written is cleared back to the "
    "FC's value, as a row's Read does. A parameter the FC does not answer "
    "keeps the value it showed.\n\n"
    "Reading changes nothing the aircraft flies: ACRO still flies your values "
    "(the startup read, then each row's Write or reset, or the Default Acro "
    "Parameters Write), so a value set outside this program shows here but is "
    "written back over at the next switch into ACRO unless you Write it. The "
    "result is printed to the console, not shown here.")

# The two set Write rows at the foot of the ArduPlane parameter group, as
# (which, label); `which` is what ControllerAutotuneStore.write_param_set takes.
# Each writes a stored set from constants.py, never the boxes.
PARAM_SET_ROWS = (('native', 'Default Autopilot Parameters'),
                  ('operator', 'Default Acro Parameters'))


def param_set_info(which):
    """Info-button text for one set Write row."""
    if which == 'native':
        values = ", ".join(f"{n} {v:g}" for n, v in NATIVE_MODE_RATE_PIDS.items())
        return (
            "Writes the stored rate loop ArduPlane's own modes fly (LOITER, FBWA, "
            "AUTO, ...) to the FC now: constants.NATIVE_MODE_RATE_PIDS, AUTOTUNE's "
            f"result for this airframe - {values}. The boxes above play no part.\n\n"
            "Every value goes out, whether or not the FC already holds it, and "
            "each is read back. Every switch into one of those modes writes the "
            "same set by itself, but only what differs; this button is for when "
            "the FC may not hold what this program thinks, or after a mode change "
            "made outside this program (MAVProxy, the transmitter, a failsafe), "
            "which writes nothing.\n\n"
            "Your values do not change: the next switch into ACRO writes them "
            "back. The boxes above show what the FC then holds. The result is "
            "printed to the console, not shown here.")
    values = ", ".join(f"{n} {v:g}" for n, v in ACRO_DEFAULT_PARAMS.items())
    return (
        "Writes the stored Acro defaults to the FC now and makes them your "
        "values: constants.ACRO_DEFAULT_PARAMS - "
        f"{values}. The boxes above play no part: a value you typed or wrote "
        "with a row's Write is replaced by its default.\n\n"
        "Every value goes out, whether or not the FC already holds it, and each "
        "is read back. From then on every switch into ACRO, the balloon chaser's mode, "
        "writes these, until a row's Write or reset changes one. The "
        "PTCH2SRV_ and RLL2SRV_ rows have no stored default and are left as "
        "they are.\n\n"
        "The next switch into LOITER, FBWA or another ArduPlane mode writes the "
        "Default Autopilot Parameters back. The boxes above show what the FC "
        "then holds. The result is printed to the console, not shown here.")


def _fmt_native(value):
    """A saved ArduPlane parameter as the box shows it; '' until the FC answers."""
    return "" if value is None else f"{value:.6g}"


def _native_pending(text, confirmed):
    """True when the box does not hold the value the FC confirmed - typed, not
    written.  Compared as numbers, so '2.0' against a confirmed 2 is not pending."""
    if confirmed is None:
        return text.strip() != ""
    try:
        return abs(float(text) - confirmed) > 1e-6 * max(1.0, abs(confirmed))
    except ValueError:
        return True


def _native_work(what, call):
    """Worker-thread body for Write/Read: touches no Tk object; outcomes are
    logged by the store and the link."""
    try:
        call()
    except Exception:                                  # noqa: BLE001
        log.exception("[FC PARAM] %s failed unexpectedly", what)

# Single source of truth: tunables grouped BY AXIS, then by LAYER within each
# axis.  Each group is (group_title, [(label, attr), ...]).  The settings both
# steering laws read come first, above everything: the switch between them,
# the bank limit and the pitch trim.  A setting one law alone reads lives in
# that law's groups.  Then the cascade's: Pan Law Blending, the flight-path
# pair that decides the share between bank-to-turn and roll-to-plane, and
# roll-to-plane's own law.  The rest follow the FC's law
# (LUA/control_roll.lua, LUA/control_pitch.lua):
#   - ROLL outer  = compute_roll_command's steering gains (linear P + I).
#   - ROLL inner  = the rate constant it predicts with, plus the FF path.
#   - PITCH       = compute_bodyplane_pitch_rate, the one body-plane law at
#                   every attitude, and the turn's own pitch rate added to it.
# The renderer draws a separator line (and the title) between groups.  The flat
# TUNER_PARAMS list below is DERIVED from this, so the ParamStore, discrete-switch
# logic, and attr-based selection are unchanged.
TUNER_GROUPS = [
    ('BOTH LAWS - shared by the cascade and Lift Vector Pointing', [
        ('Steering Law (0 cascade / 1 Lift Vector Pointing)', 'steering_law'),
        ('Bank Lim deg',                  'bank_lim_deg'),
        ('Pitch Trim Offset elev deg',    'pitch_trim_offset'),
        ('Pitch Trim Lift elev deg',      'pitch_trim_lift'),
        ('Pitch Trim Attitude elev deg',  'pitch_trim_attitude'),
        ('Pitch Trim Thrust elev deg',    'pitch_trim_thrust'),
    ]),
    ('Pan Law Blending', [
        ('Roll-to-Plane Steep Start deg',        'plane_steep_start_deg'),
        ('Roll-to-Plane Steep Full deg',         'plane_steep_full_deg'),
    ]),
    ('ROLL - roll-to-plane', [
        ('Roll-to-Plane Hold deg',               'plane_side_hold_deg'),
        ('Roll-to-Plane Gain dps per deg',       'plane_gain'),
        ('Roll-to-Plane Ki 1/s',                 'plane_ki'),
        ('Roll-to-Plane Windup (0-1)',           'plane_windup'),
        ('Roll-to-Plane Kd dps per dps',         'plane_kd'),
        ('Roll-to-Plane Kd LPF Rate 1/s',        'plane_kd_rate'),
        ('Roll-to-Plane FF (1=full)',            'plane_ff_gain'),
        ('Roll-to-Plane FF LPF Rate 1/s',        'plane_ff_lpf_rate'),
    ]),
    ('PITCH stack', [
        ('Pitch Gain dps per deg',  'pitch_gain'),
        ('Pitch Ki 1/s',           'pitch_ki'),
        ('Pitch Windup (0-1)',     'windup_limit_pitch'),
        ('Pitch Kd dps per dps',          'pitch_kd'),
        ('Pitch Kd LPF Rate 1/s',         'pitch_kd_rate'),
        ('Pitch Turn FF (1=full)',        'pitch_turn_ff'),
    ]),
    ('ROLL - outer (steering)', [
        ('Pan P Gain deg per deg', 'pan_p_gain'),
        ('Pan Ki 1/s',           'pan_ki'),
        ('Pan Windup (0-1)',       'windup_limit_pan'),
    ]),
    ('ROLL - inner (rate) + lead', [
        ('Roll Gain dps per deg', 'roll_gain'),
        ('Pan Kd (lead) s',     'pan_kd'),
        ('Pan Kd LPF Rate 1/s', 'pan_kd_rate'),
    ]),
    ('CURVE FLYER - the planned turn', [
        ('Curve Flyer',                      'curve_flyer_enable'),
        ('Tilt Back Travel deg',             'tilt_back_travel_deg'),
        ('Transfer Control When Done',       'cf_transfer_when_done'),
        ('Plan Wait s',                      'cf_plan_wait_s'),
        ('Approach Angle deg',               'cf_approach_deg'),
    ]),
    ('YAW - coordination and pointing', [
        ('Yaw Coordination FF',              'yaw_coordination_ff'),
        ('Yaw Point Kp dps per deg',         'yaw_point_kp'),
        ('Yaw Point Ki 1/s',                 'yaw_point_ki'),
        ('Yaw Point Windup (0-1)',          'yaw_point_windup'),
        ('Yaw Point Limit deg/s',           'yaw_point_lim'),
        ('Yaw Point Steep Schedule',        'yaw_point_pitch_sched'),
    ]),
    ('LIFT VECTOR POINTING - the steering law, version B (rudder at zero)', [
        ('Path Turn Gain 1/s',               'att_k_path'),
        ('Path Rate Limit deg/s',            'att_path_rate_max_dps'),
        ('Path Lead s',                      'att_lead_s'),
        ('Lift Line Roll Gain 1/s',          'att_k_roll'),
        ('Sideways Turn Weight',             'att_k_side'),
        ('Path Roll Ki 1/s',                 'att_roll_ki'),
        ('Path Pitch Ki 1/s',                'att_pitch_ki'),
        ('Integrator Zone deg',              'att_i_zone_deg'),
    ]),
    ('Autotune', [
        ('Autotune Wide Bracket',       'autotune_bracket_wide'),
        ('Autotune Use Constants',      'autotune_use_constants'),
        ('Cell Autotune Use Constants',  'cell_autotune_use_constants'),
        ('Cell Longitude deg',          'autotune_cell_lon_deg'),
        ('Cell Latitude deg',           'autotune_cell_lat_deg'),
        ('AT Pan Min deg',  'autotune_pan_min'),
        ('AT Pan Max deg',  'autotune_pan_max'),
        ('AT Pan Step deg', 'autotune_pan_step'),
        ('AT Tilt Min deg',  'autotune_tilt_min'),
        ('AT Tilt Max deg',  'autotune_tilt_max'),
        ('AT Tilt Step deg', 'autotune_tilt_step'),
    ]),
    ('Plant ID', [
        ('Do Pitch',                'plantid_do_pitch'),
        ('Do Yaw',                  'plantid_do_yaw'),
        ('Do Roll',                 'plantid_do_roll'),
        ('Rate Tune Aggressiveness', 'plantid_rate_agg'),
        ('Alt Protect Floor m',     'autotune_protect_floor_m'),
        ('Alt Protect Climb-To m',  'autotune_protect_climb_to_m'),
    ]),
    ('Trim Map', [
        ('Climb Min deg',           'trim_map_climb_min_deg'),
        ('Climb Max deg',           'trim_map_climb_max_deg'),
        ('Climb Step deg',          'trim_map_climb_step_deg'),
        ('Airspeed Min m/s',        'trim_map_speed_min_mps'),
        ('Airspeed Max m/s',        'trim_map_speed_max_mps'),
        ('Airspeed Step m/s',       'trim_map_speed_step_mps'),
        ('Alt Protect Floor m',     'trim_map_protect_floor_m'),
        ('Alt Protect Climb-To m',  'trim_map_protect_climb_to_m'),
    ]),
    ('Sensor / Shared', [
        ('Gimbal Slew deg/s',       'gimbal_slew_rate'),
        ('Auto Gimbal',             'auto_gimbal'),
        ('FOV Estimator',           'fov_est_enable'),
    ]),
    ('Throttle & Airspeed', [
        ('Throttle %',                      'throttle_pct'),
        ('Enable Special Throttle',         'special_throttle_enable'),
        ('Special Throttle %',              'special_throttle_pct'),
        ('Airspeed Cmd (Off Thr / On Spd)', 'airspeed_cmd_enable'),
        ('Airspeed Cmd m/s',              'airspeed_cmd_mps'),
        ('Airspeed Kp % per m/s',          'airspeed_cmd_kp'),
        ('Airspeed Ki % per m',            'airspeed_cmd_ki'),
    ]),
    ('Balloon Injection', [
        ('Longitude deg',        'point_lon_deg'),
        ('Latitude deg',         'point_lat_deg'),
        ('Enable Movement',      'point_balloon_move_enable'),
        ('Movement Speed m/s',   'point_balloon_speed'),
        ('Movement Bearing deg', 'point_balloon_vel_pan'),
        ('Movement Elevation deg', 'point_balloon_vel_tilt'),
        ('Distance m',           'point_balloon_range'),
    ]),
    ('Manual Movement', [
        ('Arrow Left deg',    'arrow_left_deg'),
        ('Arrow Right deg',   'arrow_right_deg'),
        ('Arrow Up deg',      'arrow_up_deg'),
        ('Arrow Down deg',    'arrow_down_deg'),
    ]),
    ('CONTROL OVERRIDE - debug, bypasses the law', [
        ('Aileron Override (0 law / 1 manual)',  'aileron_override'),
        ('Aileron Deflection %',       'aileron_deflection_pct'),
        ('Elevator Override (0 law / 1 manual)', 'elevator_override'),
        ('Elevator Deflection %',      'elevator_deflection_pct'),
        ('Rudder Override (0 law / 1 manual)',   'rudder_override'),
        ('Rudder Deflection %',        'rudder_deflection_pct'),
    ]),
    ('Display', [
        ('Draw Debug', 'draw_roll_error'),
        ('Balloon Chaser Cam Window', 'nose_cam_display'),
        ('Gimbal Cam Window', 'gimbal_cam_display'),
    ]),
]

# Flat (label, attr) list derived from the groups - preserves the existing
# attr-indexed selection model and the ParamStore contract.
TUNER_PARAMS = [row for _title, rows in TUNER_GROUPS for row in rows]

# Map each flat param index -> the number of group boundaries at or before it,
# i.e. how many separators precede that row.  Used by the layout so the renderer
# and the mouse hit-test derive row pixel positions from ONE shared calculation
# (they cannot drift).  Index 0 of a group (except the first) sits below a
# separator.
_GROUP_START_INDICES = []
_acc = 0
for _title, _rows in TUNER_GROUPS:
    _GROUP_START_INDICES.append(_acc)
    _acc += len(_rows)


# Tunables that accept only discrete values.  These render as clickable switches
# (click cycles to the next value and applies immediately) instead of accepting
# typed keyboard input.  To add a new discrete tunable, register it here with
# its allowed values, in order; the UI handles the rest automatically.
#   attr -> ordered list of allowed values
DISCRETE_PARAMS = {
    'draw_roll_error': [0, 1],        # 0 = off, 1 = draw roll-error rays on gimbal view
    'aileron_override': [0, 1],    # 0 = law drives the surface, 1 = manual
    'elevator_override': [0, 1],
    'rudder_override': [0, 1],
    'yaw_point_pitch_sched': [0, 1],         # 0 = steep schedule OFF (default), 1 = ON
    'curve_flyer_enable': [0, 1],            # 0 = no plan (default), 1 = the law flies the planned turn
    'cf_transfer_when_done': [0, 1],         # 0 = the law holds the turn's line, 1 = the law flies on without the plan (default)
    'steering_law': [0, 1],                  # 0 = the cascade (default), 1 = Lift Vector Pointing
    'airspeed_cmd_enable': [0, 1],          # 0 = operator commands THROTTLE (default), 1 = commands AIRSPEED
    'autotune_bracket_wide': [0, 1],
    'autotune_use_constants': [0, 1],
    'cell_autotune_use_constants': [0, 1],
    'nose_cam_display': [0, 1],            # 'Balloon Chaser Camera' window: 0 = OFF (default)
    'gimbal_cam_display': [0, 1],          # 'Gimbal Camera' window: 1 = ON (default)
    'auto_gimbal': [0, 1],                 # 1 = auto gimbal tracking in l/x ON (default)
    'fov_est_enable': [0, 1],              # 1 = background FOV estimator ON (default OFF)
    'point_balloon_move_enable': [0, 1],    # 0 = ptest balloon stationary (default), 1 = moving
    'special_throttle_enable': [0, 1],
    'plantid_do_pitch': [0, 1],            # include the pitch axis in the 'i' run
    'plantid_do_yaw': [0, 1],              # include the yaw axis in the 'i' run
    'plantid_do_roll': [0, 1],             # include the roll axis in the 'i' run
}


def _is_discrete(attr):
    return attr in DISCRETE_PARAMS


# How a discrete value is LABELLED in the tuner.  Display only: the stored value
# is still the number in DISCRETE_PARAMS.
#
# Only genuine on/off switches belong here: a selector whose 0 picks another
# option rather than disabling anything would read "Off" falsely.
DISCRETE_CAPTIONS = {
    'draw_roll_error':          ['Off', 'On'],
    'yaw_point_pitch_sched':    ['Off', 'On'],
    'curve_flyer_enable':       ['Off', 'On'],
    'cf_transfer_when_done':    ['Off', 'On'],
    'airspeed_cmd_enable':      ['Off', 'On'],
    'autotune_bracket_wide':       ['Off', 'On'],
    'autotune_use_constants':      ['Off', 'On'],
    'cell_autotune_use_constants': ['Off', 'On'],
    'nose_cam_display':         ['Off', 'On'],
    'gimbal_cam_display':       ['Off', 'On'],
    'auto_gimbal':              ['Off', 'On'],
    'fov_est_enable':           ['Off', 'On'],
    'point_balloon_move_enable': ['Off', 'On'],
    'special_throttle_enable':  ['Off', 'On'],
    'plantid_do_pitch':         ['Off', 'On'],
    'plantid_do_yaw':           ['Off', 'On'],
    'plantid_do_roll':          ['Off', 'On'],
}

# A caption list out of step with its value list would mislabel a switch
# silently.  Caught at import - startup, not mid-flight.
for _a, _caps in DISCRETE_CAPTIONS.items():
    if _a not in DISCRETE_PARAMS:
        raise AssertionError(
            f"DISCRETE_CAPTIONS has '{_a}', which is not a discrete param")
    if len(_caps) != len(DISCRETE_PARAMS[_a]):
        raise AssertionError(
            f"DISCRETE_CAPTIONS['{_a}'] has {len(_caps)} captions for "
            f"{len(DISCRETE_PARAMS[_a])} values")
del _a, _caps


def _discrete_caption(attr, idx):
    """Text shown for the idx-th allowed value; the number if uncaptioned."""
    caps = DISCRETE_CAPTIONS.get(attr)
    if caps is not None:
        return caps[idx]
    return str(DISCRETE_PARAMS[attr][idx])


# ---------------------------------------------------------------------------
# Layout geometry.  ONE place computes where each param row sits and how to map
# a click-y back to a row.  Both render() and the mouse callback call these, so
# the two can never disagree about coordinates.
# ---------------------------------------------------------------------------
_LAYOUT_TOP = 40          # y of the first param row's text baseline
_SEPARATOR_H = 42         # extra vertical space a group's separator+title takes,
                          # added before every group except the first


def _group_index_for_param(param_idx):
    """Which group a flat param index belongs to (0-based)."""
    g = 0
    for gi, start in enumerate(_GROUP_START_INDICES):
        if param_idx >= start:
            g = gi
        else:
            break
    return g


def _param_row_y(param_idx):
    """Baseline y for a param row, accounting for the separators of all groups
    that start at or before this row."""
    n_separators_before = _group_index_for_param(param_idx)  # groups 1..g each add one
    return _LAYOUT_TOP + param_idx * _TUNER_ROW_H + n_separators_before * _SEPARATOR_H


def _separator_y(group_idx):
    """Y of the separator line drawn ABOVE the first row of group `group_idx`
    (group 0 has none).  Sits midway in the gap that _param_row_y opened up."""
    first = _GROUP_START_INDICES[group_idx]
    return _param_row_y(first) - _TUNER_ROW_H - (_SEPARATOR_H // 2) + 6


def _total_layout_height():
    last = len(TUNER_PARAMS) - 1
    return _param_row_y(last) + _TUNER_ROW_H


def _param_index_for_click(click_y):
    """Inverse of _param_row_y: the param index whose row contains click_y, or
    None if the click landed in a separator gap or outside any row."""
    for i in range(len(TUNER_PARAMS)):
        y = _param_row_y(i)
        if y - 28 <= click_y <= y + 10:
            return i
    return None


def _next_discrete_value(attr, current):
    """Return the next allowed value after `current`, wrapping.  Matches by
    nearest allowed value so float drift (1.0 vs 1) resolves cleanly."""
    allowed = DISCRETE_PARAMS[attr]
    # find index of the value closest to current
    idx = min(range(len(allowed)), key=lambda i: abs(allowed[i] - current))
    return allowed[(idx + 1) % len(allowed)]


class ParamStore(Protocol):
    """Routes a controller-autotune attr name to its holder (ControlGains / gimbal / runtime)."""
    def get(self, attr: str) -> float: ...
    def set(self, attr: str, value: float) -> None: ...
    def reset_all(self) -> None: ...
    def reset_one(self, attr: str) -> None: ...
    # Saved ArduPlane parameters (optional: without native_value the group is
    # not drawn).  read_native / write_native BLOCK for a MAVLink round trip.
    def native_value(self, name: str) -> Optional[float]: ...
    def read_native(self, name: str) -> Optional[float]: ...
    def read_all_native(self) -> Optional[dict]: ...
    def write_native(self, name: str, text: str) -> Optional[float]: ...
    def write_param_set(self, which: str) -> Optional[list]: ...



class ControllerAutotune:
    """Controller Autotune (gain tuner) implemented as a native Tk window (replacing the OpenCV
    HighGUI version).  Tk runs on its own daemon thread with its own event
    loop, so it gets real scrolling (scrollbar + mouse wheel), a resizable
    window, and proper text-entry fields for free.  All mutation goes through
    the same ParamStore the rest of the app uses, so the control loop is
    unaffected.

    Public API is unchanged so the controller/keyboard need no edits:
      - create_window():       start the Tk thread + build the window.
      - render():              no-op (Tk redraws itself); kept for call-site
                               compatibility.  A periodic Tk `after` poll keeps
                               displayed values in sync with external changes.
      - handle_key(key)->bool: returns False (Tk owns its own keyboard now).
      - reset_to_defaults():   reset all params via the store (also reachable
                               from the OpenCV backtick key).

    If Tkinter is unavailable, the controller autotune degrades to inert (logs a warning);
    the rest of the application runs normally.
    """

    def __init__(self, store: ParamStore):
        self._store = store
        self._tk = None            # tkinter module (imported on the Tk thread)
        self._root = None          # Tk root window
        self._delete_primed = {}    # export key -> primed for a confirming 2nd press
        # Keys typed at the tuner window, drained by the CONTROL thread.
        # cv2.waitKey only delivers keys to an OpenCV window, so with the camera
        # views switched off the operator would have no key path at all.  The
        # tuner is already open and costs nothing extra, so it hosts them too.
        # deque(maxlen=...) is atomic under the GIL for append/popleft, so no
        # lock is needed between the Tk thread and the control thread, and a
        # bounded deque cannot grow without limit if nothing drains it.
        self._key_queue = _collections.deque(maxlen=32)
        self._thread = None
        self._ready = False
        # attr -> dict of widget refs / state, populated on the Tk thread
        self._rows = {}
        # saved ArduPlane parameter name -> its row's widgets and the value the
        # FC last confirmed, as shown (see _build_native_group)
        self._native_rows = {}
        self._reset_requested = False
        self._info_windows = {}     # attr -> its open info Toplevel (one each)
        self._keydiag = None        # on-screen last-key diagnostic label

    # ----- public API (unchanged signatures) -----

    def create_window(self):
        """Start the Tk event-loop thread and build the window."""
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._tk_main, name="ControllerAutotune",
                                        daemon=True)
        self._thread.start()

    def render(self):
        """No-op: the Tk thread redraws itself.  Kept so the controller's
        per-frame call site is unchanged."""
        return

    def handle_key(self, key) -> bool:
        """Tk owns its own keyboard (Entry widgets handle text).  Return False
        so the OpenCV keyboard handler treats the key as not-consumed."""
        return False

    def reset_to_defaults(self):
        """Reset all params to constant defaults via the store, then refresh the
        GUI IMMEDIATELY (don't wait for the periodic poll)."""
        try:
            self._store.reset_all()
        except Exception:
            log.exception("Tuner reset_to_defaults failed")
        # Refresh now, on the Tk thread, so the new values show instantly.
        root = self._root
        if root is not None:
            try:
                root.after(0, self._refresh_all_now)
            except Exception:
                self._reset_requested = True   # fallback: let the poll catch it
        else:
            self._reset_requested = True

    def _refresh_all_now(self):
        """Force every displayed value to match the store right now, clearing
        any stale 'editing' flags (used after a reset)."""
        for attr, r in self._rows.items():
            try:
                if r.get("kind") == "cont":
                    r["editing"] = False
                    r["var"].set(self._fmt(attr, self._store.get(attr)))
                else:
                    self._refresh_discrete(attr)
            except Exception:
                pass

    def close(self):
        """Tear down the Tk window/thread cleanly.  Called on app shutdown
        (incl. Ctrl+C) so the Tcl interpreter and its display connection are
        released - otherwise a daemon Tk thread can leave them half-open and the
        next launch shows no window.  Thread-safe: schedules the quit onto the
        Tk thread via after()."""
        root = self._root
        if root is None:
            return
        try:
            root.after(0, root.quit)       # ask the Tk loop to exit
        except Exception:
            pass
        t = self._thread
        if t is not None:
            t.join(timeout=2.0)            # wait for _tk_main's finally to run

    # ----- Tk thread internals -----

    def _tk_main(self):
        try:
            import tkinter as tk
            from tkinter import ttk
        except Exception as e:
            log.warning("Gain Tuner: Tkinter unavailable (%s); tuner disabled", e)
            return
        self._tk = tk
        try:
            self._root = tk.Tk()
            # Route keystrokes to the control thread.  Entry widgets get first
            # refusal: while a value is being typed, the keys belong to the box,
            # not to the mode shortcuts.
            self._root.bind_all("<Key>", self._on_tk_key)
            self._root.title("Gain Tuner")
            self._root.geometry("540x900")
            # Force the interpreter to finish initializing (and register this as
            # the default root) BEFORE creating any widgets or StringVars - on a
            # worker thread the default-root global is otherwise set racily,
            # which makes StringVar() intermittently raise "Too early to create
            # variable: no default root window".
            self._root.update_idletasks()
            self._root.protocol("WM_DELETE_WINDOW", self._on_close)
            self._build_ui(tk, ttk)
            self._ready = True
            self._root.after(150, self._poll_values)
            self._root.mainloop()
        except Exception:
            log.exception("Gain Tuner Tk thread crashed")
        finally:
            # Ensure the Tcl interpreter and its display connection are released
            # so a subsequent launch can open a fresh window (a daemon thread
            # terminated by Ctrl+C can otherwise leave it half-open).
            try:
                if self._root is not None:
                    self._root.quit()
                    self._root.destroy()
            except Exception:
                pass
            self._root = None
            self._ready = False

    def _on_close(self):
        """Window close button: tear down the Tk root cleanly."""
        try:
            if self._root is not None:
                self._root.quit()
        except Exception:
            pass

    def _build_ui(self, tk, ttk):
        # Dark-ish theme.
        bg = "#101010"; fg = "#dcdcdc"; acc = "#78dcff"; grp = "#82c882"
        self._root.configure(bg=bg)

        # Top bar: a Reset button (always visible; same as the ` key).
        topbar = tk.Frame(self._root, bg="#0a0a0a")
        topbar.pack(fill="x", side="top")
        _b = tk.Button(topbar, text="Reset all to defaults (`)",
                  command=self._on_reset_button,
                  relief=BTN_RELIEF, overrelief=BTN_OVERRELIEF, bd=BTN_BD,
                  highlightthickness=AURA_PX, highlightbackground=AURA_IDLE,
                  highlightcolor=AURA_FOCUS, bg="#205020", fg="#d0ffd0",
                  activebackground="#205020", activeforeground="#d0ffd0",
                  pady=EXPORT_PADY, font=("TkDefaultFont", 11))
        _b.pack(side="left", padx=6, pady=4)
        _bind_aura_hover(_b)

        # Scrollable area: Canvas + inner Frame + vertical Scrollbar.
        outer = tk.Frame(self._root, bg=bg)
        outer.pack(fill="both", expand=True)
        canvas = tk.Canvas(outer, bg=bg, highlightthickness=0)
        vbar = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vbar.set)
        vbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        inner = tk.Frame(canvas, bg=bg)
        win_id = canvas.create_window((0, 0), window=inner, anchor="nw")

        def _on_inner_config(_e):
            canvas.configure(scrollregion=canvas.bbox("all"))
        inner.bind("<Configure>", _on_inner_config)

        def _on_canvas_config(e):
            canvas.itemconfig(win_id, width=e.width)   # inner fills width
        canvas.bind("<Configure>", _on_canvas_config)

        # Mouse-wheel scrolling (Tk delivers these reliably, unlike OpenCV).
        def _wheel(e):
            delta = e.delta
            if delta == 0 and hasattr(e, 'num'):
                delta = 120 if e.num == 4 else -120   # X11 buttons 4/5
            canvas.yview_scroll(int(-delta / 120), "units")
        canvas.bind_all("<MouseWheel>", _wheel)        # Windows / WSLg
        canvas.bind_all("<Button-4>", _wheel)          # X11 up
        canvas.bind_all("<Button-5>", _wheel)          # X11 down

        # Backtick resets all to defaults (matches the OpenCV-era ` key).
        def _maybe_reset(e):
            ch = getattr(e, 'char', '')
            ks = getattr(e, 'keysym', '')
            if ch == '`' or ks in ('grave', 'quoteleft', 'dead_grave', 'asciitilde'):
                self.reset_to_defaults()
                return "break"
            return None
        self._root.bind_all("<KeyPress>", _maybe_reset, add="+")

        # Click on any empty (non-Entry) area commits the active field and drops
        # focus, so a click anywhere applies the change (like Enter) and stops
        # further typing from editing that box.  Bind at the TOPLEVEL so clicks
        # on the many background children (row frames, group headers, separator
        # lines) are caught too - those bubble up to the toplevel.  The
        # isinstance(Entry) guard skips clicks that land on a value box, so
        # clicking into a field still lets it take focus.
        def _bg_click(e):
            w = getattr(e, 'widget', None)
            if isinstance(w, (tk.Entry, tk.Canvas)):
                return        # field click -> keep focus; info-circle Canvas -> its own handler
            try:
                self._commit_active()
                self._root.focus_set()
            except Exception:
                pass
        self._root.bind("<Button-1>", _bg_click, add="+")

        big = ("TkDefaultFont", 12)
        big_b = ("TkDefaultFont", 13, "bold")

        # Column geometry.  Every row - tunables AND action rows like the
        # export buttons - puts its label in a fixed-PIXEL cell, so the value
        # column starts at the same x on every row and nothing overlaps a long
        # label.  Measured from the real font rather than assumed in
        # characters: TkDefaultFont is proportional, so a width=N character
        # count clips a wide label (this is what covered the right edge of
        # "Autotune Maneuver (0 step / 1 doublet)").  Adding a longer label
        # than any existing one therefore widens the column automatically -
        # see the Gain Tuner GUI conventions in CLAUDE.md.
        self._label_px, self._label_h, want_w = self._measure_columns(tk, big)
        try:
            self._root.geometry("%dx900" % want_w)
        except Exception:
            pass

        for title, rows in TUNER_GROUPS:
            hdr = tk.Label(inner, text=title, bg=bg, fg=grp, font=big_b,
                           anchor="w", pady=6)
            hdr.pack(fill="x", padx=8, pady=(10, 2))
            tk.Frame(inner, bg="#444444", height=1).pack(fill="x", padx=4)
            for label, attr in rows:
                self._build_row(tk, inner, label, attr, bg, fg, acc, big)

        # Saved ArduPlane parameters: a value box with Write and Read per row.
        # Not in TUNER_GROUPS: the values live on the FC, not in the ParamStore,
        # and their reset writes to the FC instead of restoring a DEFAULT_CONST.
        self._build_native_group(tk, inner, bg, fg, acc, grp, big, big_b)

        # Export group: one button per generated file type.  Rendered here
        # rather than as a TUNER_GROUPS entry because these are ACTIONS, not
        # tunables - they have no attr, no ParamStore route and no default, so
        # putting them in TUNER_GROUPS would break the (label, attr) contract
        # that TUNER_PARAMS, the ParamStore and DEFAULT_CONST all rely on.
        self._build_export_group(tk, inner, bg, fg, acc, grp, big, big_b)

    def _build_native_group(self, tk, inner, bg, fg, acc, grp, font, font_b):
        """The saved ArduPlane parameters AUTOTUNE tunes, one row each: the value
        the FC last confirmed in the box, Write, Read, info, reset.  The same
        _label_cell and _value_cell as every tunable row, so the boxes line up
        with the entries above.  Below them, a Read All for every row, then
        PARAM_SET_ROWS: one Write each for the two stored sets, the Autopilot
        defaults and the Acro defaults, whole, from constants.py.

        Write, Read, Read All and reset are MAVLink round trips, so they run on
        worker threads that touch no Tk object; the outcome goes to the CONSOLE
        (GUI conventions), and _poll_values puts the value the FC confirms into
        the box.  A box whose number is not the FC's confirmed value is drawn in
        NATIVE_PENDING_FG - typed, not written.  The reset icon WRITES the row's
        stored Acro default (acro_default_value), or, where it has none, the
        value the FC held when this program started (native_startup_value)."""
        if not callable(getattr(self._store, 'native_value', None)):
            return
        hdr = tk.Label(inner, text="ArduPlane parameters (on the FC)", bg=bg,
                       fg=grp, font=font_b, anchor="w", pady=6)
        hdr.pack(fill="x", padx=8, pady=(10, 2))
        tk.Frame(inner, bg="#444444", height=1).pack(fill="x", padx=4)
        for name in ARDUPLANE_AUTOTUNE_PARAMS:
            row = tk.Frame(inner, bg=bg)
            row.pack(fill="x", padx=8, pady=2)
            self._label_cell(tk, row, name, bg, fg, font)
            shown = self._store.native_value(name)
            var = self._tk.StringVar(master=self._root, value=_fmt_native(shown))
            cell = self._value_cell(tk, row, bg)
            ent = tk.Entry(cell, textvariable=var, font=font,
                           highlightthickness=AURA_PX,
                           highlightbackground=AURA_IDLE,
                           highlightcolor=AURA_FOCUS,
                           bg=VALUE_BG, fg=fg, insertbackground=fg,
                           relief=BTN_RELIEF, bd=BTN_BD)
            ent.pack(side="left", fill="both", expand=True)
            _style_entry_hover(ent)
            for text, face, colour, handler in (
                    ("Write", NATIVE_WRITE_BG, NATIVE_WRITE_FG, self._on_native_write),
                    ("Read", "#202830", acc, self._on_native_read)):
                b = tk.Button(row, text=text, font=font, relief=BTN_RELIEF,
                              overrelief=BTN_OVERRELIEF, bd=BTN_BD,
                              highlightthickness=AURA_PX,
                              highlightbackground=AURA_IDLE,
                              highlightcolor=AURA_FOCUS,
                              bg=face, fg=colour, padx=10, pady=EXPORT_PADY,
                              activebackground=face, activeforeground=colour,
                              command=lambda n=name, h=handler: h(n))
                b.pack(side="left", padx=(4, 0))
                _bind_aura_hover(b)
            self._make_info_button(tk, row, "native:" + name, name, bg,
                                   text=native_param_info(name))
            self._make_reset_button(tk, row, "native:" + name, bg,
                                    on_click=lambda n=name: self._on_native_reset(n))
            self._native_rows[name] = {"var": var, "ent": ent, "shown": shown,
                                       "fg": fg}
        # Read All, then the two set Writes, at the foot of the group after
        # every row they act on.  Action rows: the same _label_cell, one button,
        # an info button and no reset, because an action has no value to restore.
        row = tk.Frame(inner, bg=bg)
        row.pack(fill="x", padx=8, pady=(6, 2))
        self._label_cell(tk, row, READ_ALL_LABEL, bg, fg, font)
        b = tk.Button(row, text="Read All", font=font, relief=BTN_RELIEF,
                      overrelief=BTN_OVERRELIEF, bd=BTN_BD,
                      highlightthickness=AURA_PX,
                      highlightbackground=AURA_IDLE,
                      highlightcolor=AURA_FOCUS,
                      bg="#202830", fg=acc, padx=10, pady=EXPORT_PADY,
                      activebackground="#202830", activeforeground=acc,
                      command=self._on_native_read_all)
        b.pack(side="left")
        _bind_aura_hover(b)
        self._make_info_button(tk, row, "native-read-all", READ_ALL_LABEL, bg,
                               text=READ_ALL_INFO)
        for which, label in PARAM_SET_ROWS:
            row = tk.Frame(inner, bg=bg)
            row.pack(fill="x", padx=8, pady=2)
            self._label_cell(tk, row, label, bg, fg, font)
            b = tk.Button(row, text="Write", font=font, relief=BTN_RELIEF,
                          overrelief=BTN_OVERRELIEF, bd=BTN_BD,
                          highlightthickness=AURA_PX,
                          highlightbackground=AURA_IDLE,
                          highlightcolor=AURA_FOCUS,
                          bg=NATIVE_WRITE_BG, fg=NATIVE_WRITE_FG, padx=10,
                          pady=EXPORT_PADY, activebackground=NATIVE_WRITE_BG,
                          activeforeground=NATIVE_WRITE_FG,
                          command=lambda w=which: self._on_param_set_write(w))
            b.pack(side="left")
            _bind_aura_hover(b)
            self._make_info_button(tk, row, "native-set:" + which, label, bg,
                                   text=param_set_info(which))

    def _on_native_write(self, name):
        """Write button (Tk thread): take the box's text now, write on a worker."""
        r = self._native_rows.get(name)
        if r is None:
            return
        text = r["var"].get()
        threading.Thread(target=_native_work,
                         args=("write " + name, lambda: self._store.write_native(name, text)),
                         name="fc-param-write-%s" % name, daemon=True).start()

    def _on_param_set_write(self, which):
        """A set Write button (Tk thread): write the whole set on a worker."""
        threading.Thread(target=_native_work,
                         args=("%s set Write" % which,
                               lambda: self._store.write_param_set(which)),
                         name="fc-param-set-%s" % which, daemon=True).start()

    def _on_native_reset(self, name):
        """Reset icon (Tk thread): put the row's default in the box and write it
        on a worker, through the same refusal and readback as Write.  The
        default is the stored Acro default (acro_default_value); for a
        parameter without one, the value the FC held when this program started.
        The write carries the exact value (float32_text), not the box's rounded
        text."""
        r = self._native_rows.get(name)
        if r is None:
            return
        want, source = acro_default_value(name), "its stored Acro default"
        if want is None:
            want, source = self._store.native_startup_value(name), "its startup value"
        if want is None:
            log.error("[FC PARAM] reset %s: it has no stored Acro default and the FC "
                      "did not report it at startup - nothing sent", name)
            return
        text = float32_text(want)
        r["var"].set(_fmt_native(want))
        log.info("[FC PARAM] reset %s to %s %s", name, source, text)
        threading.Thread(target=_native_work,
                         args=("reset " + name, lambda: self._store.write_native(name, text)),
                         name="fc-param-reset-%s" % name, daemon=True).start()

    def _on_native_read(self, name):
        """Read button (Tk thread): drop any typed, unwritten text now, read on a
        worker; the poll shows the value the FC reports."""
        r = self._native_rows.get(name)
        if r is not None:
            r["var"].set(_fmt_native(r["shown"]))
        threading.Thread(target=_native_work,
                         args=("read " + name, lambda: self._store.read_native(name)),
                         name="fc-param-read-%s" % name, daemon=True).start()

    def _on_native_read_all(self):
        """Read All button (Tk thread): drop every row's typed, unwritten text
        now, read every parameter on one worker; the poll shows the values the
        FC reports."""
        for r in self._native_rows.values():
            r["var"].set(_fmt_native(r["shown"]))
        threading.Thread(target=_native_work,
                         args=("Read All", self._store.read_all_native),
                         name="fc-param-read-all", daemon=True).start()

    def _refresh_native_rows(self):
        """Tk thread, from _poll_values.  A value the FC newly confirmed replaces
        the box's text - unless the box holds a value typed and not yet written.
        Mode switches change the confirmed value often, and replacing a typed
        value there would make the next Write store the switch's value as the
        operator's.  The colour says whether the box matches the FC."""
        for name, r in self._native_rows.items():
            try:
                conf = self._store.native_value(name)
                if conf != r["shown"]:
                    typed = _native_pending(r["var"].get(), r["shown"])
                    r["shown"] = conf
                    if not typed:
                        r["var"].set(_fmt_native(conf))
                pending = _native_pending(r["var"].get(), r["shown"])
                r["ent"].configure(fg=NATIVE_PENDING_FG if pending else r["fg"])
            except Exception:                          # noqa: BLE001
                pass

    def _build_export_group(self, tk, inner, bg, fg, acc, grp, font, font_b):
        """WSL -> host-OS file export.  Each row copies one file type from the
        working directory (where every writer puts its output) to the folder in
        export_paths.py.  Copies run on a worker thread so a large CSV over
        DrvFs cannot freeze the tuner window.

        Rows are built with the same _label_cell as every tunable row, so the
        Copy buttons line up with the entry fields and switches above them.
        Each row carries an info button describing what the file is, where it
        lives and where it is copied to; there is no reset button because an
        action has nothing to reset.  Results are printed to the CONSOLE only -
        see the Gain Tuner GUI conventions in CLAUDE.md.
        """
        if file_export is None:
            return
        hdr = tk.Label(inner, text="Export to OS", bg=bg, fg=grp,
                       font=font_b, anchor="w", pady=6)
        hdr.pack(fill="x", padx=8, pady=(10, 2))
        tk.Frame(inner, bg="#444444", height=1).pack(fill="x", padx=4)

        for key, label, _pattern, _pick, _origin, _purpose in file_export.EXPORTS:
            row = tk.Frame(inner, bg=bg)
            row.pack(fill="x", padx=8, pady=2)
            self._label_cell(tk, row, label, bg, fg, font)
            _b = tk.Button(row, text="Copy", font=font, relief=BTN_RELIEF,
                      overrelief=BTN_OVERRELIEF, bd=BTN_BD,
                      highlightthickness=AURA_PX, highlightbackground=AURA_IDLE,
                      highlightcolor=AURA_FOCUS,
                      bg="#202830", fg=acc, padx=10, pady=EXPORT_PADY,
                      activebackground="#202830", activeforeground=acc,
                      command=lambda k=key: self._on_export(k))
            _b.pack(side="left")
            _bind_aura_hover(_b)
            # Every export gets a Delete alongside Copy.  Copy puts the file on
            # the host side; Delete clears the working-directory original, which
            # is how a stale 'latest' report stops being the one exported and how
            # a checkpoint is made to start fresh.  Two presses, because it is
            # irreversible and some of these are hours of flying.
            if key in getattr(file_export, "DELETABLE", {}):
                btn = tk.Button(row, text="Delete", font=font, relief=BTN_RELIEF,
                                overrelief=BTN_OVERRELIEF, bd=BTN_BD,
                                highlightthickness=AURA_PX,
                                highlightbackground=AURA_IDLE,
                                highlightcolor=AURA_FOCUS,
                                bg="#302020", fg="#d08080", padx=10,
                                pady=EXPORT_PADY,
                                activebackground="#302020",
                                activeforeground="#d08080")
                btn.configure(command=lambda k=key, b=btn: self._on_delete(k, b))
                btn.pack(side="left", padx=(4, 0))
                _bind_aura_hover(btn)
                self._delete_primed[key] = None
            self._make_info_button(tk, row, "export:" + key, label, bg,
                                   text=file_export.info_text(key))

        # The "All generated files" row - the bottom of the group, after
        # everything it acts on.  Deliberately identical to the per-file rows:
        # same label cell, same "Delete" button, same "Confirm?" on arming.  It
        # is the same action over a wider set, and a differently-worded button
        # would imply a different KIND of action.
        row = tk.Frame(inner, bg=bg)
        row.pack(fill="x", padx=8, pady=(6, 2))
        self._label_cell(tk, row, "All generated files", bg, fg, font)
        allb = tk.Button(row, text="Delete", font=font, relief=BTN_RELIEF,
                         overrelief=BTN_OVERRELIEF, bd=BTN_BD,
                         highlightthickness=AURA_PX,
                         highlightbackground=AURA_IDLE,
                         highlightcolor=AURA_FOCUS,
                         bg="#302020", fg="#d08080", padx=10, pady=EXPORT_PADY,
                         activebackground="#302020", activeforeground="#d08080")
        allb.configure(command=lambda b=allb: self._on_delete_all(b))
        allb.pack(side="left")
        _bind_aura_hover(allb)
        self._delete_primed[file_export.ALL_KEY] = None
        self._make_info_button(
            tk, row, "export:all", "All generated files", bg,
            text=("Deletes the working-directory files for EVERY row above, in "
                  "one press-and-confirm.\n\n"
                  "This is the same Delete as each row, applied to all of them: "
                  "reports, logs, cell CSVs, suggested-constants files and both "
                  "resume checkpoints. Copies already exported to the host side "
                  "are NOT touched.\n\n"
                  "Removing the checkpoints means the next sweep and the next "
                  "single-cell tune start FRESH instead of resuming.\n\n"
                  "Refused while a tuning or plant-ID run is up, or while flight "
                  "logging is running - those files are being written, and the "
                  "console says which.\n\n"
                  "Two presses, because it cannot be undone. Result is printed "
                  "to the console, not shown in this window."))

    def _on_delete_all(self, btn):
        """Two-press confirm, then delete every entry on a worker thread.

        Refused OUTRIGHT if anything is busy, rather than clearing what it can.
        A half-completed Delete All is the exact failure the per-row Delete had -
        the operator presses it, some files survive, and nothing on screen says
        which.  One blocked entry is one keypress away from being unblocked."""
        blockers = []
        for key, _label, *_rest in file_export.EXPORTS:
            why = self._delete_blocked(key)
            if why:
                blockers.append(why)
        if blockers:
            log.warning("[EXPORT] DELETE ALL refused - %s", blockers[0])
            self._disarm_delete(file_export.ALL_KEY, btn)
            return
        if not self._delete_primed.get(file_export.ALL_KEY):
            self._delete_primed[file_export.ALL_KEY] = True
            btn.configure(text="Confirm?", fg="#ffb060")
            self._root.after(DELETE_CONFIRM_MS,
                             lambda: self._disarm_delete(file_export.ALL_KEY, btn))
            return
        self._disarm_delete(file_export.ALL_KEY, btn)

        def _work():
            try:
                file_export.delete_all()
            except Exception:                                   # noqa: BLE001
                log.exception("[EXPORT] delete-all failed")
        threading.Thread(target=_work, daemon=True,
                         name="export-delete-all").start()

    def _on_delete(self, key, btn):
        """Two-press confirm, then delete on a worker thread.

        First press arms the button and relabels it; a second press within
        DELETE_CONFIRM_MS deletes, and anything slower disarms.  No modal dialog:
        the tuner refreshes on a timer and a modal would block that thread.

        Refused while a run is up, because the running sweep rewrites its
        checkpoint after every trial - the file would simply reappear, and the
        operator would be left thinking it had been cleared."""
        blocked = self._delete_blocked(key)
        if blocked:
            log.warning("[EXPORT] %s", blocked)
            self._disarm_delete(key, btn)
            return
        if not self._delete_primed.get(key):
            self._delete_primed[key] = True
            btn.configure(text="Confirm?", fg="#ffb060")
            self._root.after(DELETE_CONFIRM_MS,
                             lambda: self._disarm_delete(key, btn))
            return
        self._disarm_delete(key, btn)

        def _work():
            try:
                file_export.delete(key)
            except Exception:                                   # noqa: BLE001
                log.exception("[EXPORT] delete failed for %s", key)
        threading.Thread(target=_work, daemon=True,
                         name="export-delete-%s" % key).start()

    def _delete_blocked(self, key):
        """Why deleting this entry right now would be pointless or harmful, or
        None.  Checked per KEY rather than as one blanket rule: a sweep in
        progress rewrites its own outputs, but it has nothing to do with the
        flight CSV, and the CSV logger's hold on flight_log.csv has nothing to
        do with a sweep."""
        ctrl = getattr(self._store, "_c", None)
        if (key in getattr(file_export, "BUSY_WHILE_RUN", ())
                and getattr(ctrl, "mode", None) == 'autotune'):
            return ("a tuning or plant-ID run is in progress and rewrites this "
                    "file as it advances, so deleting it now would achieve "
                    "nothing.  Stop the run first.")
        if key in getattr(file_export, "BUSY_WHILE_LOGGING", ()):
            csv = getattr(ctrl, "_csv", None)
            if getattr(csv, "active", False):
                return ("flight logging is running and holds this file open.  "
                        "Deleting it now would not stop the logger - it would "
                        "keep writing to a file with no name, and the run's data "
                        "would be lost.  Press 'q' to stop logging first.")
        return None

    def _disarm_delete(self, key, btn):
        self._delete_primed[key] = None
        try:
            btn.configure(text="Delete", fg="#d08080")
        except Exception:                                       # noqa: BLE001
            pass                        # widget destroyed with the window

    def _on_export(self, key):
        """Button press (Tk thread): copy on a worker thread.

        The worker touches NO Tk object and the GUI shows no result - the
        outcome goes to the console via file_export's own logging.  (Calling
        root.after() from a worker thread is not safe under Tcl and was
        observed to silently drop updates.)

        The pre-copy hook depends on the entry: flight_csv flushes the live
        CSV so the copy is not missing its tail; gui_live (GUI Live Values)
        writes a fresh gui_live_values_<ts>.py from the tuner's CURRENT state so
        the copied file always reflects the live configuration, not a stale one
        left in the working directory.
        """
        if key == "flight_csv":
            hook = getattr(self._store, "flush_csv", None)
        elif key == "gui_live":
            hook = getattr(self._store, "write_current_constants", None)
        else:
            hook = None

        # C6: Copy WARNS but proceeds.  Copying a half-written file is
        # recoverable - press Copy again when the run ends - whereas refusing
        # would block the legitimate case of grabbing a log mid-session.  Delete
        # still refuses, because deleting is not recoverable.
        busy = self._delete_blocked(key)
        if busy:
            log.warning("[EXPORT] copying while busy: %s  The copy is taken from "
                        "the file as it stands right now and may be incomplete.",
                        busy)

        def _work():
            try:
                file_export.export(key, flush_hook=hook)
            except Exception:                          # noqa: BLE001
                log.exception("[EXPORT] %s failed unexpectedly", key)

        threading.Thread(target=_work, name="export-%s" % key,
                         daemon=True).start()

    def _measure_columns(self, tk, font):
        """Return (label_px, row_h, window_px) sized to the WIDEST label and
        the widest value widget actually present, so no row is clipped."""
        try:
            import tkinter.font as tkfont
            m = tkfont.Font(root=self._root, font=font)
            measure, linespace = m.measure, m.metrics("linespace")
        except Exception:
            measure, linespace = (lambda s: 8 * len(s)), 20

        labels = [lbl for _t, rows in TUNER_GROUPS for lbl, _a in rows]
        labels += list(ARDUPLANE_AUTOTUNE_PARAMS)
        labels += [READ_ALL_LABEL] + [lbl for _w, lbl in PARAM_SET_ROWS]
        if file_export is not None:
            labels += [e[1] for e in file_export.EXPORTS]
            labels.append("All generated files")        # the Delete All row
        label_px = max([measure(s) for s in labels] or [240]) + 12

        # ONE width for every value widget - numeric entry AND discrete switch.
        # Sized to the Off/On switch caption, but never below the widest numeric
        # value so a number cannot clip.  Every value widget FILLS a fixed cell
        # of this width (see _value_cell), the way labels fill _label_cell, so
        # the entries, the Off/On switches and the 0/1 selectors all render one
        # width and the info/reset icons line up across every row.  Discrete
        # captions must fit it; today's ("Off/On", "0/1") do.
        self._value_px = max(measure("[Off]   On") + 2 * 8,   # caption + button padx
                             measure("0" * 10)) + 8           # or a 10-char value
        # Export rows carry Copy AND Delete side by side, and Delete relabels to
        # the wider "Confirm?" when primed - so the WINDOW is sized for that even
        # though the tunable value cells are narrower.
        value_px = max(self._value_px,
                       measure("Copy") + 28 + 4 + measure("Confirm?") + 28)
        # The saved ArduPlane parameter rows carry a value box AND Write and Read
        # buttons after it, so they are the widest value column of all.
        value_px = max(value_px, self._value_px + 4 + measure("Write") + 28
                       + 4 + measure("Read") + 28)

        # Info + reset canvases as they actually lay out: ICON_SIZE plus a border
        # and an aura each side, after the left pads _make_info_button and
        # _make_reset_button give them.
        icon_w = ICON_SIZE + 2 * (ICON_BD + AURA_PX)
        icons_px = (6 + INFO_DX + icon_w) + (4 + icon_w)
        chrome_px = 8 * 2 + 18 + 12      # row padding + scrollbar + margin
        return label_px, linespace + 4, label_px + value_px + icons_px + chrome_px

    def _label_cell(self, tk, row, text, bg, fg, font):
        """Fixed-width label cell - the reason every value column lines up."""
        cell = tk.Frame(row, bg=bg, width=self._label_px, height=self._label_h)
        cell.pack(side="left")
        cell.pack_propagate(False)
        tk.Label(cell, text=text, bg=bg, fg=fg, font=font,
                 anchor="w").pack(side="left", fill="both", expand=True)

    def _value_cell(self, tk, row, bg):
        """Fixed-width cell every value widget FILLS, so a numeric entry, an
        Off/On switch and a 0/1 selector all render one width and the info /
        reset icons after them line up across every row.  Mirrors _label_cell.
        Height is ROW_WIDGET_H (not _label_h) because the whole value column is
        that tall - it is what keeps ICON_DY at 0."""
        cell = tk.Frame(row, bg=bg, width=self._value_px, height=ROW_WIDGET_H)
        cell.pack(side="left")
        cell.pack_propagate(False)
        return cell

    def _build_row(self, tk, parent, label, attr, bg, fg, acc, font):
        row = tk.Frame(parent, bg=bg)
        row.pack(fill="x", padx=8, pady=2)
        self._label_cell(tk, row, label, bg, fg, font)
        try:
            val = self._store.get(attr)
        except Exception:
            # A FAILED READ IS NOT A ZERO.  This fell through to 0.0, which
            # renders identically to a parameter that genuinely is zero - so a
            # gain the store cannot see AT ALL looks like one the operator
            # turned off, and the next edit to that row writes the fake zero
            # back into the live gains.  Three yaw rows showed 0.0000 that way
            # while the store held 1.0 and 7.0.
            #
            # Show the constants.py default instead, so the number on screen has
            # a documented source, and say so loudly.  NaN when even that fails:
            # it renders as 'nan', which nobody mistakes for a setting.
            log.exception("Gain Tuner: cannot read %s from the store - the row is "
                          "NOT live and is showing a default, not the flown value", attr)
            try:
                val = self._store.default_for(attr)
            except Exception:                          # noqa: BLE001
                val = float("nan")

        if _is_discrete(attr):
            # A cycling switch whose options are spread EVENLY across the value
            # cell: with N options there are N+1 equal gaps - left edge, between
            # each pair, and right edge - so option i is centred at (i+1)/(N+1)
            # of the width.  Drawn on ONE Canvas so it presses and fires exactly
            # like the info/reset icons through _style_canvas_button - recess on
            # press, action on RELEASE, and a press dragged off before release is
            # abandoned.  The active option is coloured, not bracketed, so all
            # options stay one width and the even spacing holds (see
            # DISCRETE_IDLE_FG).  Text is positioned on <Configure> because the
            # canvas has no real size until the cell has laid it out.
            cell = self._value_cell(tk, row, bg)
            c = tk.Canvas(cell, bg=VALUE_BG, cursor="hand2",
                          highlightthickness=AURA_PX, bd=BTN_BD)
            c.pack(side="left", fill="both", expand=True)
            n = len(DISCRETE_PARAMS[attr])
            items = [c.create_text(0, 0, text="", font=font, anchor="center")
                     for _ in range(n)]

            def _layout(e, cv=c, ids=items, nn=n):
                for i, tid in enumerate(ids):
                    cv.coords(tid, e.width * (i + 1) / (nn + 1), e.height / 2)
            c.bind("<Configure>", _layout)
            self._rows[attr] = {"kind": "discrete", "canvas": c,
                                "items": items, "acc": acc}
            self._style_canvas_button(tk, c, lambda a=attr: self._on_cycle(a))
            self._refresh_discrete(attr)
        else:
            var = self._tk.StringVar(master=self._root,
                                     value=self._fmt(attr, val))
            # Fills the same fixed value cell as the discrete switch, so a
            # numeric field and an Off/On switch are exactly one width.  No
            # width= or ipady: the cell sets both dimensions (ROW_WIDGET_H tall).
            cell = self._value_cell(tk, row, bg)
            ent = tk.Entry(cell, textvariable=var, font=font,
                           highlightthickness=AURA_PX,
                           highlightbackground=AURA_IDLE,
                           highlightcolor=AURA_FOCUS,
                           bg=VALUE_BG, fg=fg, insertbackground=fg,
                           # One border for every control: 1px, raised.
                           relief=BTN_RELIEF, bd=BTN_BD)
            ent.pack(side="left", fill="both", expand=True)
            _style_entry_hover(ent)
            self._rows[attr] = {"kind": "cont", "var": var, "ent": ent,
                                "editing": False}
            # Commit on Enter AND on losing focus (clicking away / Tab).
            ent.bind("<Return>", lambda _e, a=attr: self._on_entry_enter(a))
            ent.bind("<FocusOut>", lambda _e, a=attr: self._on_entry_commit(a))
            # Clicking into the field (re)starts an edit; typing alone does not
            # restart it once committed (focus has moved away after Enter).
            ent.bind("<Button-1>", lambda _e, a=attr: self._mark_editing(a))
            ent.bind("<FocusIn>", lambda _e, a=attr: self._mark_editing(a))

        # Info button: a small drawn circle with an "i" (Canvas, so it renders
        # regardless of font glyph support).  Click opens a NEW info window.
        if attr in PARAM_INFO:
            self._make_info_button(tk, row, attr, label, bg)

        # Per-row reset icon (circular refresh arrow): resets just THIS param to
        # its default.  The store's reset_one no-ops if no default exists.
        self._make_reset_button(tk, row, attr, bg)

    @staticmethod
    def _style_canvas_button(tk, c, on_click):
        """Make a Canvas behave like a real button.

        The info and per-row reset controls are Canvases because their glyphs are
        drawn (a circled 'i', a circular arrow) rather than text, and a Canvas has
        no press behaviour of its own.

        Relief alone is not enough here.  These carry ICON_BD=1 so that
        ICON_SIZE + 2*(bd + aura) lands on ROW_WIDGET_H, and a one-pixel bevel
        swapping between raised and sunken is invisible on a dark panel.  So the
        press is shown three ways at once, the way a real button does it:

          * the GLYPH DIPS a pixel down-right, which is what actually reads as a
            button being pushed in;
          * the face LIGHTENS, so the control looks live under the finger;
          * the relief goes sunken, which is the part that survives if a theme
            ever gives these a thicker border.

        The action fires on RELEASE and only if the pointer is still over the
        control, so a click begun by mistake can be dragged off and abandoned.
        `down` guards the dip: press and release must move the glyph the same
        number of times, or a dragged-away click would leave it displaced.
        """
        c.configure(relief=BTN_RELIEF, bd=ICON_BD,
                    highlightthickness=AURA_PX, highlightbackground=AURA_IDLE,
                    highlightcolor=AURA_FOCUS)
        face = c.cget("bg")
        state = {"in": False, "down": False}

        def _up():
            if state["down"]:
                c.move("all", -ICON_PRESS_DIP, -ICON_PRESS_DIP)
                state["down"] = False
            c.configure(bg=face)

        def _press(_e):
            if not state["down"]:
                c.move("all", ICON_PRESS_DIP, ICON_PRESS_DIP)
                state["down"] = True
            c.configure(relief="sunken", bg=ICON_PRESS_BG)

        def _release(e):
            _up()
            c.configure(relief=BTN_OVERRELIEF if state["in"] else BTN_RELIEF)
            if 0 <= e.x < c.winfo_width() and 0 <= e.y < c.winfo_height():
                on_click()

        def _enter(_e):
            state["in"] = True
            c.configure(relief=BTN_OVERRELIEF)

        def _leave(_e):
            state["in"] = False
            _up()                            # dragged off mid-press: undo the dip
            c.configure(relief=BTN_RELIEF)

        c.bind("<Enter>", _enter)
        c.bind("<Leave>", _leave)
        c.bind("<ButtonPress-1>", _press)
        c.bind("<ButtonRelease-1>", _release)
        _bind_aura_hover(c)

    def _make_reset_button(self, tk, row, attr, bg, on_click=None):
        size = ICON_SIZE
        c = tk.Canvas(row, width=size, height=size, bg=bg, cursor="hand2")
        c.pack(side="left", padx=(4, 0), pady=(ICON_DY, 0))
        col = "#9ad29a"
        import math as _m
        r = ICON_RESET_R
        k = ICON_RESET_R / ICON_R        # head scales with the arc, not fixed
        # The arrowhead overhangs the TOP of the arc, so the ink is top-heavy and
        # centring the circle leaves the glyph looking high.  Drop it by half the
        # overhang so the drawn extent, not the construction circle, is what sits
        # centred - computed from r so it stays right if the size is changed.
        _over = (r * _m.sin(_m.radians(80)) + 4 * k) - (r + ICON_W / 2.0)
        cx = size / 2.0 + ICON_INSET
        cy = size / 2.0 + ICON_INSET + _over / 2.0
        # Circular-arrow (refresh) glyph: an arc with an arrowhead at the open
        # end, pointing clockwise.  The head is offset OFF the arc, so the glyph
        # is taller than the circle it is built on - see ICON_SIZE.
        c.create_arc(cx - r, cy - r, cx + r, cy + r, start=80, extent=300,
                     style="arc", outline=col, width=ICON_W)
        ang = _m.radians(80)                       # arc start angle
        ex = cx + r * _m.cos(ang)
        ey = cy - r * _m.sin(ang)
        c.create_polygon(ex, ey - 4 * k, ex + 6 * k, ey + 1 * k,
                         ex - 1 * k, ey + 5 * k, fill=col, outline=col)
        self._style_canvas_button(
            tk, c, on_click or (lambda a=attr: self._on_reset_one(a)))

    def _on_reset_one(self, attr):
        try:
            self._store.reset_one(attr)
        except Exception:
            log.exception("Tuner reset_one failed for %s", attr)
        # Refresh just this row's display immediately.
        r = self._rows.get(attr)
        if not r:
            return
        try:
            if r.get("kind") == "cont":
                r["editing"] = False
                r["var"].set(self._fmt(attr, self._store.get(attr)))
            else:
                self._refresh_discrete(attr)
        except Exception:
            pass

    def _make_info_button(self, tk, row, attr, label, bg, text=None):
        size = ICON_SIZE
        c = tk.Canvas(row, width=size, height=size, bg=bg, cursor="hand2")
        c.pack(side="left", padx=(6 + INFO_DX, 0), pady=(ICON_DY, 0))
        cx = cy = size / 2.0 + ICON_INSET
        r = ICON_R - ICON_W / 2.0        # stroke centreline; its outer edge = ICON_R
        c.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#7aa6c2",
                      width=ICON_W, fill="#1c2630")
        c.create_text(cx, cy, text="i", fill="#bfe0f2",
                      font=("TkDefaultFont", 10, "bold"))
        self._style_canvas_button(
            tk, c, lambda a=attr, lbl=label, x=text: self._show_info(a, lbl, x))

    # ----- value formatting + sync -----

    @staticmethod
    def _fmt(attr, val):
        # Infinity is legal for some cells (point_balloon_range).  Spelled
        # explicitly rather than leaning on f"{inf:.4f}" happening to give
        # 'inf'.  -inf/NaN cannot be stored, so this is always a real +inf.
        if not math.isfinite(val):
            return "inf"
        if attr == 'throttle_pct':
            return f"{val:.1f}"
        if attr == 'point_balloon_range':
            return f"{val:.1f}"          # metres; 4 decimals of a distance is noise
        return f"{val:.4f}"

    def _refresh_discrete(self, attr):
        r = self._rows.get(attr)
        if not r:
            return
        try:
            val = self._store.get(attr)
        except Exception:
            return
        allowed = DISCRETE_PARAMS[attr]
        idx = min(range(len(allowed)), key=lambda k: abs(allowed[k] - val))
        c = r["canvas"]
        for k, tid in enumerate(r["items"]):
            c.itemconfigure(tid, text=_discrete_caption(attr, k),
                            fill=(r["acc"] if k == idx else DISCRETE_IDLE_FG))

    def _on_tk_key(self, event):
        """Tk THREAD.  Queue a keystroke for the control thread to act on.

        Ignored while the focus is in an entry box - those keys are the value
        being typed.  Only the plain characters the balloon chaser keyboard uses are
        forwarded; arrows are sent as the same codes cv2 reports for them, so
        the control thread needs no second decoding path."""
        # A key typed into a value box belongs to the box, not to the mode
        # shortcuts - otherwise entering "0.08" would fire whatever '0' and '8'
        # are bound to.  Two independent checks, because neither is sufficient
        # alone: the row's own editing flag (set on focus-in, cleared on
        # commit) is authoritative but only covers rows, while focus_get()
        # catches any other Entry - and returns None when the toplevel has no
        # window-manager focus, so it cannot be relied on by itself.
        try:
            if any(r.get("editing") for r in self._rows.values()):
                return
        except Exception:                                   # noqa: BLE001
            pass
        try:
            w = self._root.focus_get()
            if w is not None and w.winfo_class() == 'Entry':
                return
        except Exception:                                   # noqa: BLE001
            pass
        from .keyboard import TUNER_ARROW_CODES   # keyboard imports this module
        if event.keysym in TUNER_ARROW_CODES:
            self._key_queue.append(TUNER_ARROW_CODES[event.keysym])
        elif event.char and len(event.char) == 1 and 32 <= ord(event.char) < 127:
            self._key_queue.append(ord(event.char))

    def take_key(self):
        """CONTROL THREAD.  Next queued keystroke, or -1.  Mirrors cv2.waitKey's
        return convention so the caller can treat both sources identically."""
        try:
            return self._key_queue.popleft()
        except IndexError:
            return -1

    def _poll_values(self):
        """Periodically refresh displayed values so external changes (reset,
        other code paths) show up.  Entry fields being actively edited are left
        alone so typing isn't clobbered."""
        for attr, r in self._rows.items():
            try:
                if r["kind"] == "discrete":
                    self._refresh_discrete(attr)
                elif not r.get("editing"):
                    r["var"].set(self._fmt(attr, self._store.get(attr)))
            except Exception:
                pass
        self._refresh_native_rows()
        if self._reset_requested:
            self._reset_requested = False
            for attr, r in self._rows.items():
                if r["kind"] == "cont":
                    r["editing"] = False
        if self._root is not None:
            self._root.after(150, self._poll_values)

    # ----- event handlers (Tk thread) -----

    def _mark_editing(self, attr):
        r = self._rows.get(attr)
        if r:
            r["editing"] = True

    def _commit_active(self):
        """Commit every continuous field currently flagged as being edited.
        Called when the user clicks empty space, so a click anywhere applies
        the in-progress edit."""
        for attr, r in list(self._rows.items()):
            if r.get("kind") == "cont" and r.get("editing"):
                self._on_entry_commit(attr)

    def _on_entry_enter(self, attr):
        """Enter key: commit, then move focus OFF the entry so that subsequent
        typing does NOT keep editing the field (it can only be re-edited by
        clicking into it again)."""
        self._on_entry_commit(attr)
        try:
            self._root.focus_set()      # drop focus from the entry
        except Exception:
            pass

    def _on_entry_commit(self, attr):
        r = self._rows.get(attr)
        if not r:
            return
        text = r["var"].get().strip()
        try:
            # Spelled-out infinity is intent.  Parsed here because float()
            # refuses some spellings a person will type (the symbol).  Whether
            # the parameter may HOLD it is still the store's call.
            v = float('inf') if _is_infinity_word(text) else float(text)
            # No per-parameter special cases: ControllerAutotuneStore.set()
            # clamps every tunable against PARAM_LIMITS and logs when it does.
            #
            # The widget's one job here is separating a typed INFINITY from a
            # numeric OVERFLOW, which only it can do - float() maps 'inf' and
            # '1e999' to the same value, and by the store they are one float.
            # An overflow is refused; cells that do not accept infinity are
            # still refused downstream by clamp_param.
            if math.isinf(v) and not _is_infinity_word(text):
                log.error("Gain Tuner: %s = %r is not a finite value - REFUSED, "
                          "the value is unchanged. (Type 'inf' if you mean "
                          "infinity.)", attr, text)
            else:
                self._store.set(attr, v)
        except ValueError:
            # Unparseable text is refused like any other garbage: loudly, with
            # the value kept - a silent revert reads as if it had been taken.
            log.error("Gain Tuner: %s = %r is not a number - REFUSED, the value "
                      "is unchanged.", attr, text)
        r["editing"] = False
        # Reflect the stored (possibly clamped) value.
        try:
            r["var"].set(self._fmt(attr, self._store.get(attr)))
        except Exception:
            pass

    def _on_cycle(self, attr):
        try:
            cur = self._store.get(attr)
            nxt = _next_discrete_value(attr, cur)
            self._store.set(attr, nxt)
            log.info("Gain Tuner: %s = %s (switch)", attr, nxt)
        except Exception:
            log.exception("Tuner cycle failed for %s", attr)
        self._refresh_discrete(attr)

    def _on_reset_button(self):
        self.reset_to_defaults()

    def _show_info(self, attr, label, text=None):
        """Open a popup window describing `attr`.  At most ONE window per info
        button: pressing the same button again raises the window it already
        opened instead of stacking duplicate copies of the same text.

        `text` overrides the PARAM_INFO lookup, for rows that are not tunables
        (the export buttons compose theirs from file_export)."""
        tk = self._tk
        if tk is None:
            return
        existing = self._info_windows.get(attr)
        if existing is not None:
            try:
                if existing.winfo_exists():
                    existing.deiconify()               # in case it was minimised
                    existing.lift()
                    existing.focus_force()
                    return
            except Exception:                          # noqa: BLE001
                pass
            self._info_windows.pop(attr, None)         # stale handle - recreate
        if text is None:
            text = PARAM_INFO.get(attr, "(no description available)")
            # Append the enforced range so the bound is visible BEFORE typing,
            # not discovered by having a value silently clamped afterwards.
            try:
                from ..controller_autotune_store import limits_for
                if attr not in DISCRETE_PARAMS:
                    lo, hi = limits_for(attr)
                    text += ("\n\nALLOWED RANGE: %s to %s (values outside are "
                             "clamped to the nearest bound and logged)."
                             % ("-inf" if lo is None else ("%g" % lo),
                                "+inf" if hi is None else ("%g" % hi)))
            except Exception:
                pass
        body = f"{label}\n\n{text}"

        win = tk.Toplevel(self._root)
        win.title(label)
        win.configure(bg="#181818")
        # Size to the text: export descriptions run far longer than a one-line
        # gain note, and a fixed height clips them.
        win.geometry("380x%d" % max(200, min(560, 90 + 15 * (len(body) // 42 + body.count("\n")))))
        lbl = tk.Label(win, text=body, bg="#181818", fg="#dcdcdc",
                       font=("TkDefaultFont", 12), justify="left",
                       anchor="nw", wraplength=330, padx=12, pady=12)
        lbl.pack(fill="both", expand=True)

        def _close(a=attr, w=win):
            if self._info_windows.get(a) is w:
                self._info_windows.pop(a, None)
            try:
                w.destroy()
            except Exception:
                pass
        tk.Button(win, text="Close", command=_close, relief=BTN_RELIEF,
                  overrelief=BTN_OVERRELIEF, bd=BTN_BD,
                  highlightthickness=AURA_PX, highlightbackground=AURA_IDLE,
                  highlightcolor=AURA_FOCUS,
                  bg="#205020", fg="#d0ffd0", pady=EXPORT_PADY,
                  activebackground="#205020", activeforeground="#d0ffd0",
                  font=("TkDefaultFont", 11)).pack(pady=(0, 10))
        win.protocol("WM_DELETE_WINDOW", _close)
        # Offset each new window slightly so they don't stack exactly.
        n = len(self._info_windows)
        try:
            win.geometry(f"+{120 + 28 * (n % 8)}+{120 + 28 * (n % 8)}")
        except Exception:
            pass
        self._info_windows[attr] = win
