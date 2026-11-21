#!/usr/bin/env python3
"""Simulator trials verifying each partial/zero score rule and the one-handed slip-on failure mode.

    scripts/rta simpy <id> mech/tasks/endcap_pack/trials.py /workspace/robocurve/outputs/<id>

- ``off_bench_zero``: roller rolls/drops off the bench edge -> zero rule latches, score 0.0.
- ``one_cap_jam_zero``: cap_l left inside carton, roller seated 21 mm in cap_l (leaving 29 mm to the -y wall, 16 mm
  past the 8 mm spindle), cap_r lowered onto the -y end -> physically jams above the carton rim, score 0.0.
- ``bench_mated_half``: both arms extract both caps and slide them onto the roller on the bench, then return to Ready
  -> coupled unit rests on the bench cradle with >= 18 mm engagement each end, score 0.5.
- ``one_hand_lift_quarter``: after both arms mate the two slip-on caps onto the roller on the bench (reaching 0.5),
  the left arm alone lifts cap_l while the right arm stays at Ready -> the unheld slip-on assembly tips and separates
  under gravity, ending at 0.25.
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


def _hold_action():
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))


def off_bench_zero(embodiment, m, out_dir: Path) -> dict:
    from inspect_robots import Scene

    embodiment.reset(Scene(id="off_bench_zero", instruction=m.INSTRUCTION), seed=0)
    m.mk.write_pose("roller", (P.X_ROLLER, -0.68, m.TOP + 0.025), (0.0, 0.0, 0.0, 1.0))
    res = None
    for _ in range(30):
        res = embodiment.step(_hold_action())
    assert res is not None
    score = float(res.info.get("score", 0.0))
    rep = m.report()
    ok = bool(score == 0.0 and rep["zero"] == "roller fell off the bench")
    return {"pass": ok, "score": score, "report": rep}


def one_cap_jam_zero(embodiment, m, out_dir: Path) -> dict:
    from inspect_robots import Scene

    embodiment.reset(Scene(id="one_cap_jam_zero", instruction=m.INSTRUCTION), seed=0)
    # cap_l pushed flush to +y inner wall (+127 mm -> centre at +104 mm); roller plugged 21 mm into cap_l
    y_cap_l = P.BOX_L / 2.0 - P.CAP_T / 2.0  # +0.1040 m
    y_roller = (y_cap_l - P.CAP_T / 2.0 + P.SOCK_DEPTH) - P.ROLLER_L / 2.0  # +0.0020 m (tube -y end at -0.0980 m: 29 mm gap)
    m.mk.write_pose("cap_l", (P.X_CARTON, y_cap_l, m.TOP + P.Z_FLOOR + 0.0005), (0.0, 0.0, 0.0, 1.0))
    m.mk.write_pose("roller", (P.X_CARTON, y_roller, m.TOP + P.Z_FLOOR + (P.SOCK_Z - P.SOCK_R) + P.ROLLER_R), (0.0, 0.0, 0.0, 1.0))
    # Place cap_r (46 mm wide) just above the remaining 29 mm gap at the -y end and let gravity pull it down
    m.mk.write_pose("cap_r", (P.X_CARTON, -0.112, m.TOP + P.Z_RIM + 0.012), (0.0, 0.0, 0.0, 1.0))
    res = None
    for _ in range(40):
        res = embodiment.step(_hold_action())
    assert res is not None
    score = float(res.info.get("score", 0.0))
    lo_r, hi_r = m.cap_world_bounds("r")
    cap_r_top_over_rim_mm = round(1000 * float(hi_r[2] - (m.TOP + P.Z_RIM)), 1)
    rep = m.report()
    ok = bool(score == 0.0 and not m.cap_seated_in_carton("r") and (cap_r_top_over_rim_mm > 5.0 or m.cap_tilt_deg("r") > 30.0))
    return {"pass": ok, "score": score, "cap_r_top_over_rim_mm": cap_r_top_over_rim_mm, "report": rep}


def bench_mated_half(embodiment, m, out_dir: Path) -> dict:
    import demo_kit as kit

    Move = kit.Move
    moves = {}
    for side in ("left", "right"):
        s = +1.0 if side == "left" else -1.0
        r_pos, r_rot = D.ready(side)
        # Extract both caps, stage outside roller, slide onto roller ends (first 7 moves of demo.py), then release & lift straight up
        base = D.arm_plan(side)[:7]
        moves[side] = base + [
            Move(0.8, grip=D.OPEN, push=(0.0, 0.0, 0.0), label="release on bench"),
            Move(1.2, pos=(P.X_ROLLER, s * K.Y_GRIP_COUPLED, K.Z_HOVER_BOX), label="up off bench"),
            Move(2.0, pos=r_pos, rot=r_rot, grip=1.0, label="ready"),
        ]
    res = kit.run(embodiment, moves, out_dir / "bench_mated_half", seed=0, tail_s=1.0, limit_s=30.0, record=False)
    score = float(res["info"].get("score", 0.0))
    rep = res["report"]
    ok = bool(score == 0.5 and rep["coupled_0.5"] is True and rep["zero"] is None)
    return {"pass": ok, "score": score, "seconds": res["seconds"], "report": rep}


def one_hand_lift_quarter(embodiment, m, out_dir: Path) -> dict:
    import demo_kit as kit

    Move = kit.Move
    r_pos_l, r_rot_l = D.ready("left")
    r_pos_r, r_rot_r = D.ready("right")
    # Both hands extract and mate both caps onto roller on the bench (reaching 0.5), then right hand releases & leaves
    # while left hand alone lifts cap_l straight up to Z_HOVER_BOX without bilateral clamping.
    left = D.arm_plan("left")[:7] + [
        Move(1.0, push=(0.0, 0.0, 0.0), label="wait right release"),
        Move(1.6, pos=(P.X_ROLLER, +0.101, K.Z_HOVER_BOX), label="one-hand lift"),
        Move(1.6, pos=(P.X_CARTON, +0.101, K.Z_HOVER_BOX), label="one-hand carry"),
        Move(0.8, grip=D.OPEN, label="release"),
        Move(1.2, pos=(P.X_CARTON, +0.101, K.Z_HOVER_BOX), label="clear"),
        Move(1.8, pos=r_pos_l, rot=r_rot_l, grip=1.0, label="ready"),
    ]
    right = D.arm_plan("right")[:7] + [
        Move(0.8, grip=D.OPEN, push=(0.0, 0.0, 0.0), label="right release"),
        Move(1.2, pos=(P.X_ROLLER, -0.101, K.Z_HOVER_BOX), label="right clear"),
        Move(2.0, pos=r_pos_r, rot=r_rot_r, grip=1.0, label="ready"),
    ]
    res = kit.run(embodiment, {"left": left, "right": right}, out_dir / "one_hand_lift_quarter", seed=0, tail_s=1.5, limit_s=35.0, record=False)
    saw_half = any(r["info"].get("score", 0.0) == 0.5 for r in res["rows"])
    end_score = float(res["info"].get("score", 0.0))
    rep = res["report"]
    ok = bool(saw_half and end_score == 0.25 and rep["coupled_0.5"] is False)
    return {"pass": ok, "saw_0.5_on_bench": saw_half, "score": end_score, "seconds": res["seconds"], "report": rep}


TRIALS = {
    "off_bench_zero": off_bench_zero,
    "one_cap_jam_zero": one_cap_jam_zero,
    "bench_mated_half": bench_mated_half,
    "one_hand_lift_quarter": one_hand_lift_quarter,
}


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    names = sys.argv[2].split(",") if len(sys.argv) > 2 else list(TRIALS)
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    all_ok = True
    summary = {}
    for name in names:
        r = TRIALS[name](embodiment, m, out)
        (out / f"{name}.json").write_text(json.dumps(r, indent=1, default=str))
        print(f"TRIAL {name} " + json.dumps(r, default=str), flush=True)
        summary[name] = {"pass": r["pass"], "score": r["score"]}
        all_ok = bool(all_ok and r["pass"])
    (out / "summary.json").write_text(json.dumps({"pass": all_ok, "trials": summary}, indent=1))
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
