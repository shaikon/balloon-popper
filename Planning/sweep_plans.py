"""Plans the rule chases with the Curve Planner (Companion/curve_planner.py) for one steering law and
checks each plan flown by the model of that law's plan mode at a finer step.  Prints one line per chase
and a summary; writes plans_<stamp>.json beside this file.  Desktop prototype.

    python3 Planning/sweep_plans.py            # the default subset, the default steering law
    python3 Planning/sweep_plans.py 1          # ... flown by Lift Vector Pointing
"""

import json, math, os, sys, time
from multiprocessing import Pool
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from Companion import curve_planner as cp
from Companion import constants as C

CLIMBS = (-60, 0, 60)
BANKS = (0, 30, -60)
OFFS = (15, 45, 75)
CLOCKS = (0, 15)
LAW = int(sys.argv[1]) if len(sys.argv) > 1 else C.STEERING_LAW


def chase_list():
    out = [dict(name="p balloon (-70, 75) from level", climb=0, bank=0,
                el=-70.0, az=75.0, rule=None)]
    for rule in (1, 2):
        for climb in CLIMBS:
            for bank in BANKS:
                for off in OFFS:
                    for clk in CLOCKS:
                        v, _ = cp.start_state(climb, 0)
                        cup = cp.unit(cp.across(cp.UP, v))
                        y = np.cross(v, cup)
                        a = math.radians((180 if rule == 1 else 0) + clk)
                        u = cup * math.cos(a) + y * math.sin(a)
                        e = math.radians(off)
                        b = v * math.cos(e) + u * math.sin(e)
                        if cp.rule_binds(v, cp.start_state(climb, bank)[1], b) != rule:
                            continue
                        out.append(dict(name=f"rule {rule} climb {climb} bank {bank} off {off} clock {clk}",
                                        climb=climb, bank=bank, b=b.tolist(), rule=rule))
    return out


def run(spec):
    if "b" in spec:
        b = spec["b"]
    else:
        el, az = math.radians(spec["el"]), math.radians(spec["az"])
        b = [math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), -math.sin(el)]
    ch = cp.Chase(spec["climb"], spec["bank"], b, rule=spec["rule"], law=LAW)
    t0 = time.time()
    p = cp.plan(ch)
    res = dict(spec, plan_s=time.time() - t0, law=LAW, knots=p["knots"], alt0=ch.alt0)
    res.update(cp.check(ch, p))
    res["ok"] = bool(np.isfinite(res["t_done"]))
    return res


if __name__ == "__main__":
    specs = chase_list()
    print(len(specs), "chases", flush=True)
    with Pool(4) as pool:
        results = []
        for r in pool.imap(run, specs):
            results.append(r)
            if r["ok"]:
                print(f"{r['name']:48s} turn done at {r['t_done']:5.2f} s  heading dev {r['heading_dev']:4.1f}  "
                      f"tilt back {r['tilt_back']:4.1f}  upside down {r['upside_over']:4.1f}  off after {r['eps_after']:4.1f}  "
                      f"closest to vertical {r['pole_min']:4.1f}  "
                      f"lowest {r['alt_min']:5.1f} m  [{r['plan_s']:4.1f} s to plan]", flush=True)
            else:
                print(f"{r['name']:48s} TURN NEVER COMPLETED, closest {r['eps_min']:5.1f} deg  "
                      f"[{r['plan_s']:4.1f} s]", flush=True)
    good = [r for r in results if r["ok"] and (not r["rule"] or (
            r["heading_dev"] <= cp.STRAIGHT_TOL_DEG + 0.5 and r["tilt_back"] <= C.TILT_BACK_TRAVEL_DEG + 0.5
            and r["pole_min"] >= C.CURVE_POLE_FADE_DEG - 0.5
            and r["bank_over"] <= 0.5 and r["upside_over"] <= 0.5))
            and r["eps_after"] <= cp.STRAIGHT_TOL_DEG + 0.5
            and r["alt_min"] >= cp.ALT_MIN_M]
    print(f"\n{len(good)} of {len(results)} turned onto the balloon and stayed within the turn tolerance, with no Suboptimal Curve; "
          f"{sum(not r['ok'] for r in results)} never completed the turn")
    stamp = time.strftime("%Y%m%d_%H%M%S")
    with open(os.path.join(os.path.dirname(__file__), f"plans_{stamp}.json"), "w") as f:
        json.dump(results, f)
