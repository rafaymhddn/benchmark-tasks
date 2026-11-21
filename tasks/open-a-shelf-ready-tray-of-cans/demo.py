#!/usr/bin/env python3
"""Scripted bimanual reference run for mech/tasks/shelf_ready_tray:

    scripts/rta demo <id> mech/tasks/shelf_ready_tray [--check] [--quick]

Flags:
- ``--check``: run in Isaac Sim with ``record=False`` (no MP4 videos written; saves setup.png, end_top.png,
  end_top_policy.png, contact sheets, and result.json with force-vs-travel readings).
- ``--quick``: offline IK and joint-margin check of the full choreography without starting the simulator.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as G  # noqa: E402
import demo_kit as kit  # noqa: E402


def run_quick_ik_check() -> dict:
    offline = G.run_offline_checks()
    over_pos, over_rot = G.right_drop_pose(lift_m=0.105)
    lay_pos, lay_rot = G.right_drop_pose(lift_m=0.0)
    q_over, mm1, deg1, m1 = G.solve_ik("right", over_pos, over_rot)
    _, mm2, deg2, m2 = G.solve_ik("right", lay_pos, lay_rot, start=q_over)
    offline["drop_waypoints"] = {
        "over_zone": {"pos": [round(float(v), 4) for v in over_pos], "mm_err": round(mm1, 3), "deg_err": round(deg1, 3), "margin_deg": round(m1, 2)},
        "lay_flat": {"pos": [round(float(v), 4) for v in lay_pos], "mm_err": round(mm2, 3), "deg_err": round(deg2, 3), "margin_deg": round(m2, 2)},
    }
    offline["worst_ik_mm"] = round(max(offline["worst_ik_mm"], mm1, mm2), 3)
    offline["min_joint_margin_deg"] = round(min(offline["min_joint_margin_deg"], m1, m2), 2)
    offline["pass"] = bool(offline["pass"] and max(mm1, mm2) <= 0.5 and min(m1, m2) >= 5.0)
    return offline


def main() -> int:
    if "--quick" in sys.argv:
        res = run_quick_ik_check()
        print("DEMO_QUICK " + json.dumps(res, indent=2), flush=True)
        return 0 if res["pass"] else 1

    import cv2
    from yam_arena import ik, station
    from yam_arena.constants import READY

    record = "--check" not in sys.argv and "--norec" not in sys.argv
    args = [a for a in sys.argv[1:] if a not in ("--check", "--norec")]
    out_dir = Path(args[0])
    case_dir = out_dir / "demo"
    case_dir.mkdir(parents=True, exist_ok=True)

    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    mk = m.mk
    ready_l = ik.tcp_pose(np.array(READY, dtype=float), station.BASES["left"])
    ready_r = ik.tcp_pose(np.array(READY, dtype=float), station.BASES["right"])

    brace_over, brace_rot = G.left_brace_pose(lift_m=0.065)
    brace_pos, _ = G.left_brace_pose(lift_m=0.0)
    brace_hold = brace_pos + np.array([0.008, -0.003, -0.006])
    brace_hold_rot = G.rz(math.radians(4.0)) @ brace_rot

    peel0_pos, peel0_rot = G.peel_arc_pose(0.0, skew_deg=0.0)
    peel_over = peel0_pos + np.array([0.0, 0.0, 0.065])
    skew_init_pos, skew_init_rot = G.peel_arc_pose(3.0, skew_deg=18.0, pull_extra_m=0.0)

    left_init = [
        kit.Move(1.4, brace_over, brace_rot, 0.45, "over_wall"),
        kit.Move(1.0, brace_pos, brace_rot, 0.45, "down_wall"),
        kit.Move(0.6, brace_hold, brace_hold_rot, 0.0, "brace_pinch"),
        kit.Move(0.8, brace_hold, brace_hold_rot, 0.0, "brace_steady"),
    ]
    right_init = [
        kit.Move(1.6, peel_over, peel0_rot, 0.50, "over_corner"),
        kit.Move(0.8, peel0_pos, peel0_rot, 0.50, "down_corner"),
        kit.Move(0.6, peel0_pos, peel0_rot, 0.0, "pinch_corner"),
        kit.Move(0.8, skew_init_pos, skew_init_rot, 0.0, "skew_corner"),
    ]

    peel_targets = [14.0, 26.0, 38.0, 50.0, 62.0, 70.0]
    state = {"idx": 0, "post_peel_scheduled": False, "last_images": None, "last_peel_rot": skew_init_rot}

    def director(t, arms, images):
        state["last_images"] = images
        if state["post_peel_scheduled"] or t < arms["right"].seconds - 1e-6:
            return

        if m.L.detached() or state["idx"] >= len(peel_targets):
            state["post_peel_scheduled"] = True
            # 1. Pull outward (-x) and right (-y) high in front of tray_base so panel clears tray by >45 mm!
            lift_high_pos = np.array([0.135, -0.230, G.TOP + 0.180], dtype=float)
            carry_pos, carry_rot = G.right_carry_pose()
            drop_over_pos, drop_rot = G.right_drop_pose(lift_m=0.105)
            drop_lay_pos, _ = G.right_drop_pose(lift_m=0.0)
            retreat_up = drop_lay_pos + np.array([-0.015, 0.0, 0.095])

            arms["right"].moves.extend(
                [
                    kit.Move(1.0, lift_high_pos, state["last_peel_rot"], 0.0, "lift_clear"),
                    kit.Move(1.1, carry_pos, carry_rot, 0.0, "carry_high"),
                    kit.Move(1.0, drop_over_pos, drop_rot, 0.0, "over_zone"),
                    kit.Move(1.1, drop_lay_pos, drop_rot, 0.0, "lay_flat"),
                    kit.Move(0.6, drop_lay_pos, drop_rot, 1.0, "release_panel"),
                    kit.Move(0.9, retreat_up, drop_rot, 1.0, "up_zone"),
                    kit.Move(1.5, ready_r[0], ready_r[1], 1.0, "ready"),
                    kit.Move(6.5, ready_r[0], ready_r[1], 1.0, "park_ready"),
                ]
            )
            arms["left"].moves.extend(
                [
                    kit.Move(2.1, brace_hold, brace_hold_rot, 0.0, "hold_while_lifting"),
                    kit.Move(0.5, brace_pos, brace_rot, 0.50, "open_brace"),
                    kit.Move(0.9, brace_over, brace_rot, 0.50, "up_wall"),
                    kit.Move(1.5, ready_l[0], ready_l[1], 1.0, "ready"),
                    kit.Move(8.7, ready_l[0], ready_l[1], 1.0, "park_ready"),
                ]
            )
            return

        deg = peel_targets[state["idx"]]
        state["idx"] += 1
        p_t, r_t = G.peel_arc_pose(deg, skew_deg=15.0, pull_extra_m=0.0)
        state["last_peel_rot"] = r_t
        arms["right"].moves.append(kit.Move(1.0, p_t, r_t, 0.0, f"peel_{int(deg)}"))
        arms["left"].moves.append(kit.Move(1.0, brace_hold, brace_hold_rot, 0.0, "brace"))

    def extra():
        return {
            "pitch_deg": round(math.degrees(float(mk.joint("peel_pitch"))), 2),
            "skew_deg": round(math.degrees(float(mk.joint("peel_skew"))), 2),
            "arc_mm": round(m.L.arc_travel_mm(), 1),
            "peel_n": round(m.L.peel_force_n(), 2),
            "ties_torn": m.L.ties_torn(),
            "detached": m.L.detached(),
            "tray_shift_mm": round(1000.0 * m.tray_max_shift_m(), 2),
        }

    result = kit.run(
        emb,
        {"left": left_init, "right": right_init},
        case_dir,
        seed=0,
        director=director,
        extra=extra,
        limit_s=m.RUN_LIMIT_STEPS / 20.0,
        tail_s=1.0,
        sheet_frames=12,
        record=record,
    )

    if state["last_images"] is not None and "top_cam" in state["last_images"]:
        end_bgr = cv2.cvtColor(state["last_images"]["top_cam"], cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(case_dir / "end_top.png"), end_bgr)
        cv2.imwrite(str(case_dir / "end_top_policy.png"), cv2.resize(end_bgr, (320, 176), interpolation=cv2.INTER_AREA))

    final_score = float(result["info"].get("score", 0.0))
    rep = m.report()
    demo_summary = {
        "score": final_score,
        "terminated": result["terminated"],
        "termination_reason": result["termination_reason"],
        "steps": result["steps"],
        "seconds": result["seconds"],
        "run_limit_steps": m.RUN_LIMIT_STEPS,
        "worst_ik": result["worst_ik"],
        "report": rep,
    }
    (case_dir / "demo_summary.json").write_text(json.dumps(demo_summary, indent=2))
    print("DEMO_SUMMARY " + json.dumps(demo_summary, indent=2), flush=True)
    return 0 if final_score == 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
