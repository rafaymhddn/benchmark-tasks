#!/usr/bin/env python3
"""G1 checks without the simulator: the reference plan through the IK, how near the two hands come while both work,
and the start's geometry.

    python3 mech/tasks/adjustable_wrench/checks.py      (in the robocurve:dev container)
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def main() -> int:
    import demo
    import demo_kit as kit
    from yam_arena import station, task_file

    m = task_file.load(HERE / "task.py").module
    P = m.P
    out = {"ik": demo.dry()}
    plan = demo.Plan(m, "full")
    moves = plan.nominal()
    arms = {side: kit.Arm(side, station.BASES[side], moves[side]) for side in ("left", "right")}
    end = max(arm.seconds for arm in arms.values())
    closest = (1e9, None)
    for i in range(int(end * 20) + 1):
        t = i / 20
        pl, pr = (arms[s].target(t)[0] for s in ("left", "right"))
        d = float(np.linalg.norm(np.asarray(pl) - np.asarray(pr)))
        if d < closest[0]:
            closest = (d, f"{t:.2f} s: left {arms['left'].target(t)[3]}, right {arms['right'].target(t)[3]}")
    out["hands_closest_mm"] = [round(1000 * closest[0], 1), closest[1]]
    o, rw = m.START["wrench"]
    knurl = o + rw @ np.array([*P.KNURL_IN_W, 0.0])
    grasp, rot, h, radius = plan.pinch()
    turn = kit.about((0, 0, 1), math.radians(demo.TURN_DEG))
    after = plan.nut + turn @ (grasp - plan.nut)
    out["start"] = {
        "wrench_tilt_deg": round(math.degrees(math.acos(rw[2, 2])), 2),
        "knurl_xy_m": [round(float(v), 4) for v in knurl[:2]],
        "knurl_from_nut_axis_mm": round(1000 * math.hypot(*(knurl[:2] - np.array(P.NUT_XY))), 1),
        "knurl_bottom_over_plate_mm": round(1000 * (knurl[2] - P.KNURL_D / 2 - m.START["plate_top"]), 2),
        "pinch_xy_m": [round(float(v), 4) for v in grasp[:2]],
        "pinch_after_turn_xy_m": [round(float(v), 4) for v in after[:2]],
        "pinch_radius_mm": round(1000 * radius, 1),
        "pull_for_prevailing_torque_N": round(P.PREVAILING_TORQUE / radius, 2),
        "free_swing_deg_at_17_5_mm": round(free_swing(P, 17.5), 1),
        "free_swing_deg_at_18_4_mm": round(free_swing(P, 18.4), 1),
    }
    print(json.dumps(out, indent=2, default=str))
    ok = all(v["worst_ik_mm"] <= 0.5 and v["smallest_margin_deg"] >= 6.0 for case in out["ik"].values() for k, v in case.items() if k in ("left", "right"))
    print("G1 IK", "PASS" if ok else "FAIL", "| hands closest", out["hands_closest_mm"][0], "mm")
    return 0 if ok else 1


def free_swing(P, jaw_mm: float) -> float:
    """How far (deg) the chamfered hex turns between parallel jaws ``jaw_mm`` apart before it jams (from flats
    parallel), by sampling its width across the jaws."""
    af, e = 1000 * P.NUT_AF, 1000 * P.NUT_E
    pts = []
    # the 12-gon's corners: intersections of flat lines (distance af/2, normals 60k) and corner facets (e/2, 60k+30)
    def corner(n1, d1, n2, d2):
        a = np.array([[math.cos(n1), math.sin(n1)], [math.cos(n2), math.sin(n2)]])
        return np.linalg.solve(a, [d1, d2])
    for k in range(6):
        f, c = math.radians(60 * k), math.radians(60 * k + 30)
        pts.append(corner(f, af / 2, c, e / 2))
        pts.append(corner(c, e / 2, f + math.radians(60), af / 2))
    pts = np.array(pts)
    for step in range(0, 3001):
        phi = math.radians(step * 0.01)
        r = np.array([[math.cos(phi), -math.sin(phi)], [math.sin(phi), math.cos(phi)]])
        width = float(np.ptp((pts @ r.T)[:, 0]))  # across the flats whose normal is x at phi = 0
        if width > jaw_mm:
            return step * 0.01
    return 30.0


if __name__ == "__main__":
    raise SystemExit(main())
