# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""plant_id_rescale - offline sim->real bridge for the OUTER pan gains.

WHY THIS EXISTS
    The competition airframe is NOT the SITL model, so gain values tuned by the
    autotune sweep in sim do not transfer numerically.  But the outer pan loop's
    plant is mostly KINEMATICS, not airframe: in a coordinated turn the heading
    rate is  psi_dot = g * tan(bank) / V,  so the pan authority per degree of
    bank depends on trim airspeed and gravity - not on mass or inertia.  That
    authority is EXACTLY the bank->yaw response the 'k' frequency-response sweep
    (plant_freq_id.py) measures:

        k_p = d(pan_err)/dt / bank      [ (deg/s) / deg ]

    Take the per-attitude k_p from the 'k' sweep's report and assemble the
    sim/real JSONs this tool reads by hand; the ratio below needs only k_p at a
    matched shallow attitude for each.

    So the principled transfer is a RATIO.  The pan loop gain is
    L ~ k_p * K_controller; to keep the same loop crossover on the real
    aircraft,

        K_real = K_sim * (k_p_sim / k_p_real)

    e.g. if the Talon flies slower than the SITL model, each degree of bank
    buys MORE turn rate (k_p_real > k_p_sim), the loop gain would rise, and the
    controller gains must come DOWN by the same factor.  One short plant-ID
    flight on the real aircraft replaces re-flying the whole sweep.

WHAT IS (and is NOT) RESCALED
    Scaled by k_p_sim / k_p_real - the outer pan-loop controller gains:
        PAN_P_GAIN   (THE small-signal pan gain: the linear P gain,
                                  deg of bank per deg of pan error)
        PAN_KI
        PAN_KD
    Deliberately NOT scaled:
        BANK_LIM_DEG - a kinematic AUTHORITY limit (max bank), not
                                   a loop gain
        inner roll loop (ROLL_GAIN / _FF_GAIN) - its plant is the
                                   airframe roll dynamics + ArduPlane rate loop;
                                   handled by running ArduPlane AUTOTUNE on the
                                   real aircraft, then the sweep's inner group
        LPF rates                - governed by the REAL sensor noise; swept in
                                   sim (operator direction) - re-verify on the
                                   aircraft
        pitch gains              - the pitch plant is not measured by this ID;
                                   re-confirm on hardware by hand

WORKFLOW
    1. sweep in a SITL model matched to the real aircraft; bake results into
       constants.py
    2. 'k' frequency sweep in SIM    -> assemble k_p into plant_id_results_sim.json
    3. real aircraft: ArduPlane AUTOTUNE first (rate loops), then the 'k' sweep
                                     -> assemble k_p into plant_id_results_real.json
    4.   python -m Companion.plant_id_rescale sim.json real.json
       paste the DERATED column into constants.py for the first real flights
       (default 0.7x: deliberately under-gained - buy phase margin against
       everything the ratio does not capture, then walk up with the manual
       tuner).

    A cross-check is printed: kinematics say k_p ~ g/V, so k_sim/k_real should
    roughly equal V_real/V_sim (from the recorded airspeed medians).  A large
    disagreement means non-kinematic effects (sideslip, sensor scaling, a bad
    fit) - investigate before trusting the scale.

Nothing is applied automatically; this prints suggestions for a HUMAN to review
and paste into constants.py, exactly like the other suggested-constants outputs.
"""
from __future__ import annotations
import argparse
import json
import sys

# Current constants are read only to print the "current -> scaled" table; the
# SCALE itself needs no constants.  Standalone execution (not -m) or an import
# failure degrades to printing the scale factor alone.
try:                                    # package execution: python -m Companion.plant_id_rescale
    from . import constants as C        # type: ignore
except ImportError:
    try:                                # script execution from inside the package dir
        import constants as C           # type: ignore
    except ImportError:
        C = None

# (constant name, why it scales) - the outer pan-loop controller gains.
SCALED_GAINS = (
    ("PAN_P_GAIN", "small-signal pan gain (linear P gain, deg bank per deg error)"),
    ("PAN_KI",     "pan integrator gain"),
    ("PAN_KD",         "pan lead/derivative gain"),
)


def _load(path):
    with open(path) as f:
        return json.load(f)


def reference_kp(data, label):
    """|k_p| at the SHALLOWEST measured attitude - the reference pan authority.
    Prefers the folded |pitch| groups (worst-of-signs); falls back to the
    shallowest usable raw cell."""
    groups = data.get("groups") or []
    if groups:
        g = min(groups, key=lambda g: float(g["abs_pitch_deg"]))
        return (abs(float(g["k_p_worst"])),
                "|pitch|=%.0f deg group, worst-of-signs" % float(g["abs_pitch_deg"]))
    cells = [c for c in (data.get("cells") or []) if abs(float(c.get("k_p", 0.0))) > 1e-4]
    if not cells:
        raise SystemExit("%s: no usable k_p (no groups, no cells with |k_p|>1e-4) - "
                         "supply k_p from more of the 'k' sweep's cells." % label)
    c = min(cells, key=lambda c: abs(float(c["pitch_cell_deg"])))
    return (abs(float(c["k_p"])),
            "single cell at %+.0f deg (no folded groups)" % float(c["pitch_cell_deg"]))


def reference_airspeed(data):
    """Median airspeed recorded at the shallowest usable cell (for the kinematic
    cross-check); None when the file has no usable cells / speeds."""
    cells = [c for c in (data.get("cells") or [])
             if abs(float(c.get("k_p", 0.0))) > 1e-4 and float(c.get("airspeed_med", 0.0)) > 0.5]
    if not cells:
        return None
    c = min(cells, key=lambda c: abs(float(c["pitch_cell_deg"])))
    return float(c["airspeed_med"])


def rescale(sim_path, real_path, derate=0.7):
    sim, real = _load(sim_path), _load(real_path)
    k_sim, sim_src = reference_kp(sim, sim_path)
    k_real, real_src = reference_kp(real, real_path)
    scale = k_sim / k_real

    print("plant-ID pan-authority rescale  (sim -> real)")
    print("  sim :  |k_p| = %.4f (deg/s)/deg   [%s]" % (k_sim, sim_src))
    print("  real:  |k_p| = %.4f (deg/s)/deg   [%s]" % (k_real, real_src))
    print("  gain scale = k_sim / k_real = %.3f   (derate x%.2f for first flights)"
          % (scale, derate))

    # Kinematic cross-check: k_p ~ g/V  =>  k_sim/k_real ~ V_real/V_sim.
    v_sim, v_real = reference_airspeed(sim), reference_airspeed(real)
    if v_sim and v_real:
        v_ratio = v_real / v_sim
        print("  cross-check: V_sim=%.1f  V_real=%.1f  ->  V_real/V_sim = %.3f "
              "(kinematics predict ~= gain scale)" % (v_sim, v_real, v_ratio))
        if scale > 0 and (max(scale, v_ratio) / max(1e-9, min(scale, v_ratio))) > 1.35:
            print("  *** WARNING: measured scale and airspeed ratio disagree by >35%% -")
            print("  *** non-kinematic effects or a bad fit; investigate before trusting.")
    else:
        print("  (no airspeed medians recorded - kinematic cross-check skipped)")

    print()
    print("  %-26s %12s %14s %18s" % ("constant", "current", "x scale",
                                      "x scale x %.2f" % derate))
    for name, why in SCALED_GAINS:
        cur = getattr(C, name, None) if C is not None else None
        if cur is None:
            print("  %-26s %12s %14s %18s   # %s" % (name, "?", "x%.3f" % scale,
                                                     "x%.3f" % (scale * derate), why))
        else:
            cur = float(cur)
            print("  %-26s %12.5g %14.5g %18.5g   # %s"
                  % (name, cur, cur * scale, cur * scale * derate, why))
    print()
    print("  NOT scaled: BANK_LIM_DEG (kinematic authority limit, not a loop")
    print("  gain), the inner roll loop (re-run ArduPlane AUTOTUNE + the sweep's inner")
    print("  group on the real plane), LPF rates (swept in sim but governed by REAL")
    print("  sensor noise - re-verify on the aircraft), and the pitch gains (not")
    print("  measured by this ID - re-confirm on hardware by hand).")
    print()
    print("  Paste the DERATED column into constants.py for the first real flights,")
    print("  then walk gains up with the manual tuner.")
    return scale


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Rescale the outer pan gains from a sim vs real plant-ID pair "
                    "(see the module docstring for the theory and workflow).")
    ap.add_argument("sim_json", help="plant_id_results.json from the SITL run")
    ap.add_argument("real_json", help="plant_id_results.json from the real aircraft")
    ap.add_argument("--derate", type=float, default=0.7,
                    help="extra first-flight safety factor on the scaled gains "
                         "(default 0.7 = 30%% under-gained)")
    args = ap.parse_args(argv)
    rescale(args.sim_json, args.real_json, derate=args.derate)


if __name__ == "__main__":
    main(sys.argv[1:])
