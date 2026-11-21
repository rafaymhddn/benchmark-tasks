#!/usr/bin/env python3
"""PhysX trials verifying each zero rule, partial score level, and bimanual bracing requirement (Gate 2).

    scripts/rta simpy <id> mech/tasks/curing_hood/trials.py /workspace/robocurve/outputs/<id>

Trials run in one simulator session:
- ``unshielded_flash_zero``: pressing the wand's power button in the cradle without seating the orange hood triggers
                             the whole-trial optical-safety zero rule (score 0.0).
- ``off_bench_zero``:        dropping the hood off the bench latches the off-bench zero rule (score 0.0).
- ``unbraced_wand_push``:    threading and pushing the hood with one hand while the right arm stays at ready shoves the
                             unbraced 145 g wand out of its open cradle under the 6.0 N grommet force (score 0.0).
- ``hood_seated_half``:      bimanual threading past the 55 deg bend and 22 mm seating without curing scores 0.5.
- ``cured_out_of_cradle``:   shielded curing of molar #46 without returning the wand to its cradle scores 0.75.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as K  # noqa: E402
import demo as D  # noqa: E402
import parts as P  # noqa: E402


def unshielded_flash_zero(emb, _m, out_dir: Path) -> dict:
    """Skip fitting the orange hood: Right arm lifts the bare wand to molar #46 and Left arm presses the power button
    (same verified cure poses as demo.py) -> unshielded_flash latches permanent 0.0."""
    import demo_kit as kit

    Move = kit.Move
    full = D.plan()
    pw0, rw0 = P.wand_start_pose(K.TOP)
    pr_c, rr_c = K.right_on_wand(pw0, rw0)
    l_ready_p, l_ready_r = D.ready("left")
    # Right arm: grasp bare wand in cradle, lift and aim at molar #46, brace press
    lift_idx = next(i for i, mv in enumerate(full["right"]) if mv.label == "lift shielded wand")
    brace_idx = next(i for i, mv in enumerate(full["right"]) if mv.label == "brace press")
    right = [
        Move(1.4, pos=pr_c + np.array([0.0, 0.0, 0.055]), rot=rr_c, grip=D.OPEN, label="above wand"),
        Move(0.8, pos=pr_c, label="on wand barrel"),
        Move(0.6, grip=D.WAND_GRIP, label="grasp bare wand"),
        *full["right"][lift_idx:brace_idx],
    ]
    t_ready_for_press = sum(mv.seconds for mv in right)
    btn_idx = next(i for i, mv in enumerate(full["left"]) if mv.label == "above button")
    left = [
        Move(t_ready_for_press, pos=l_ready_p, rot=l_ready_r, grip=D.CLOSED, label="wait aim"),
        *full["left"][btn_idx:],
    ]
    right.append(Move(2.8, pos=full["right"][brace_idx].pos, push=(0.0, 0.0, 2.2), label="brace press"))
    res = kit.run(emb, {"left": left, "right": right}, out_dir / "unshielded_flash_zero", seed=0, tail_s=0.6, record=False)
    rep = res["report"]
    ok = res["info"].get("score", 0.0) == 0.0 and rep["zero"] is not None and "without orange light hood" in rep["zero"]
    return {"pass": ok, "score": res["info"].get("score"), "report": rep}


def off_bench_zero(emb, m, out_dir: Path) -> dict:
    """Drop hood below the bench -> permanent zero rule -> 0.0."""
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    emb.reset(Scene(id="drop", instruction=m.INSTRUCTION), seed=0)
    m.mk.write_pose("hood", (0.25, 0.42, K.TOP - 0.030), (0.0, 0.0, 0.0, 1.0))
    hold = Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))
    step_res = None
    for _ in range(10):
        step_res = emb.step(hold)
    assert step_res is not None
    rep = m.report()
    sc = float(step_res.info.get("score", 0.0))
    ok = sc == 0.0 and rep["zero"] == "hood left the bench"
    out = {"pass": ok, "score": sc, "report": rep}
    (out_dir / "off_bench_zero.json").write_text(json.dumps(out, indent=1, default=str))
    return out


def unbraced_wand_push(emb, m, out_dir: Path) -> dict:
    """Left arm executes the exact Phase-1 hood threading & 6.0 N push while Right arm stays at READY (no brace):
    the 145 g wand slides out of its open cradle and the hood cannot reach >= 19 mm seated -> 0.0."""
    import demo_kit as kit

    full = D.plan()
    # Left arm executes Phase-1 hood threading (first 14 moves) while Right arm stays at READY (no brace)
    moves = {"left": full["left"][:14], "right": []}
    pw_start = P.wand_start_pose(K.TOP)[0]
    res = kit.run(emb, moves, out_dir / "unbraced_wand_push", seed=0, tail_s=0.8, record=False)
    pw_end = m.mk.pose("wand")[0]
    wand_slid_mm = round(1000.0 * float(np.linalg.norm(pw_end - pw_start)), 1)
    rep = res["report"]
    sc = float(res["info"].get("score", 0.0))
    ok = wand_slid_mm >= 12.0 and sc == 0.0
    return {"pass": ok, "wand_slid_out_mm": wand_slid_mm, "score": sc, "report": rep}


def hood_seated_half(emb, m, out_dir: Path) -> dict:
    """Phase 1 only (Right braces wand, Left threads 55 deg bend and pushes 22 mm home, both return to ready): 0.5."""
    import demo_kit as kit

    Move = kit.Move
    full = D.plan()
    pw0, rw0 = P.wand_start_pose(K.TOP)
    pr_c, _ = K.right_on_wand(pw0, rw0)
    r_ready_p, r_ready_r = D.ready("right")
    l_ready_p, l_ready_r = D.ready("left")
    left = full["left"][:14] + [Move(1.8, pos=l_ready_p, rot=l_ready_r, grip=D.OPEN, label="left ready")]
    right = full["right"][:6] + [
        Move(0.6, grip=D.OPEN, push=(0.0, 0.0, 0.0), label="release wand"),
        Move(1.0, pos=pr_c + np.array([0.0, 0.0, 0.055]), label="clear cradle"),
        Move(1.8, pos=r_ready_p, rot=r_ready_r, label="right ready"),
    ]
    res = kit.run(emb, {"left": left, "right": right}, out_dir / "hood_seated_half", seed=0, tail_s=1.5, record=False)
    rep = res["report"]
    sc = float(res["info"].get("score", 0.0))
    ok = sc == 0.5 and rep["hood_seated"] and rep["zero"] is None
    return {"pass": ok, "score": sc, "report": rep}


def cured_out_of_cradle(emb, m, out_dir: Path) -> dict:
    """Phases 1 + 2 (hood seated and molar #46 cured for >= 2.5 s), stopping with wand held outside cradle: 0.75."""
    import demo_kit as kit

    full = D.plan()
    dwell_idx = next(i for i, mv in enumerate(full["right"]) if mv.label.startswith("dwell cure"))
    moves = {"left": full["left"], "right": full["right"][: dwell_idx + 1]}
    res = kit.run(emb, moves, out_dir / "cured_out_of_cradle", seed=0, tail_s=1.0, record=False)
    rep = res["report"]
    sc = float(res["info"].get("score", 0.0))
    ok = sc == 0.75 and rep["cured"] and rep["zero"] is None
    return {"pass": ok, "score": sc, "report": rep}


TRIALS = {
    "unshielded_flash_zero": unshielded_flash_zero,
    "off_bench_zero": off_bench_zero,
    "unbraced_wand_push": unbraced_wand_push,
    "hood_seated_half": hood_seated_half,
    "cured_out_of_cradle": cured_out_of_cradle,
}


def main() -> int:
    import demo_kit as kit

    out_dir = Path(sys.argv[1])
    out_dir.mkdir(parents=True, exist_ok=True)
    names = sys.argv[2].split(",") if len(sys.argv) > 2 else list(TRIALS)
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    all_ok = True
    summary = {}
    for name in names:
        res = TRIALS[name](emb, m, out_dir)
        summary[name] = res
        print(f"TRIAL {name} " + json.dumps(res, default=str), flush=True)
        all_ok &= bool(res["pass"])
    (out_dir / "trials_summary.json").write_text(json.dumps(summary, indent=1, default=str))
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
