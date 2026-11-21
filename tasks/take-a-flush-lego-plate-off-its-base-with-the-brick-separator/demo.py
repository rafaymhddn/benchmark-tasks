#!/usr/bin/env python3
"""Scripted reference run of ``mech/tasks/lego_separator`` (both arms; ``plan.py`` holds the motions):

    scripts/rta demo <id> mech/tasks/lego_separator --check    # in the simulator, no videos
    scripts/rta demo <id> mech/tasks/lego_separator            # the same with the three cameras' videos
    python3 mech/tasks/lego_separator/demo.py --quick           # no simulator: G1 checks and the pick through the IK

Writes into ``outputs/<id>/demo/``: ``result.json`` (every step: joints' travel, the arms' estimated forces, the press
gauge against the separator-plate contact sensor), ``demo_summary.json`` (score, the director's readings, the
force-against-travel logs of ``peel`` and ``pull``), ``setup.png``, ``end_top.png``, ``end_top_policy.png`` and a
contact sheet per camera.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import demo_kit as kit  # noqa: E402


def quick() -> int:
    import checks as C
    import plan
    from yam_arena import station
    from yam_arena.task_file import TABLE_TOP

    res = C.main()
    dry = kit.dry_run({"right": plan.pick_moves(TABLE_TOP["station"])}, station.BASES)
    res["pick_dry_run"] = dry
    print("DEMO_QUICK " + json.dumps(res, indent=1, default=str), flush=True)
    return 0 if dry["right"]["worst_ik_mm"] <= 0.5 and dry["right"]["smallest_margin_deg"] >= 5.0 else 1


def main() -> int:
    if "--quick" in sys.argv:
        return quick()
    import cv2
    import plan
    import trials

    record = "--check" not in sys.argv
    out = Path([a for a in sys.argv[1:] if not a.startswith("--")][0]) / "demo"
    out.mkdir(parents=True, exist_ok=True)
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    director = plan.Director(m, "demo")
    last: dict = {}

    def watch(t, arms, pictures):
        last["pictures"] = pictures
        director(t, arms, pictures)

    hz = float(embodiment.info.control_hz)
    result = kit.run(embodiment, {"left": [], "right": plan.pick_moves(m.TOP)}, out, seed=0, director=watch, tail_s=1.0,
                     limit_s=m.RUN_LIMIT_STEPS / hz, sheet_frames=16, record=record, extra=trials.extra_reader(m))  # fmt: skip
    pictures = last.get("pictures") or {}
    if "top_cam" in pictures:
        end = cv2.cvtColor(pictures["top_cam"], cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(out / "end_top.png"), end)
        cv2.imwrite(str(out / "end_top_policy.png"), cv2.cvtColor(kit.policy_picture(pictures["top_cam"]), cv2.COLOR_RGB2BGR))
    rows = result["rows"]
    summary = {
        "score": float(result["info"].get("score", 0.0)),
        "terminated": result["terminated"],
        "termination_reason": result["termination_reason"],
        "steps": result["steps"],
        "seconds": result["seconds"],
        "run_limit_steps": m.RUN_LIMIT_STEPS,
        "worst_ik": result["worst_ik"],
        "median_step_ms": result["median_step_ms"],
        "director": director.log,
        "report": result["report"],
        "press_vs_contact": [{"t": r["t"], "press_n": r["extra"]["press_n"], "contact_n": r["extra"]["contact_sep_target_n"]}
                             for r in rows if r["extra"]["press_n"] > 0.5 and not r["extra"]["off"] and not r["extra"]["seated"]],  # fmt: skip
        "arm_force_while_pulling": [{"t": r["t"], "pull_mm": r["extra"]["pull_mm"], "left_n": r["force"]["left"]}
                                    for r in rows if r["left"] == "pull along studs"],  # fmt: skip
    }
    (out / "demo_summary.json").write_text(json.dumps(summary, indent=1, default=str))
    print("DEMO_SUMMARY " + json.dumps({k: v for k, v in summary.items() if k not in ("press_vs_contact", "arm_force_while_pulling")}, default=str)[:4000], flush=True)
    return 0 if summary["score"] == 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
