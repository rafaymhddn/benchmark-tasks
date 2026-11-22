#!/usr/bin/env python3
"""PhysX mechanics and scoring trials for ``mech/tasks/baby_scale_convert``.

    scripts/rta simpy <id> mech/tasks/baby_scale_convert/trials.py /workspace/robocurve/outputs/<id> [names...]

- ``slide_without_button``  left arm pinches the head wing and pushes +x without pressing ``release_btn``: the pawl in
                            the front step stops the tray's tongue (< 8 mm << 20 mm), score 0.0.
- ``yank_locked_base``      both arms pinch the head and foot wings and lift straight up without unlocking: the two
                            keyhole studs catch under their lips and lift ``scale_base`` > 8.0 mm off its marks (0.0).
- ``tray_bench_fall``       tray released 20 cm above bare bench in PhysX free fall: bench-fall zero rule (0.0).
- ``unlocked_half``         right arm holds ``release_btn`` down while left arm slides ``weighing_tray`` 23 mm back (both
                            studs in their keyhole mouths), then both arms retract to READY without the mat: score 0.5.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as K  # noqa: E402
import demo  # noqa: E402
import demo_kit as kit  # noqa: E402
import parts as P  # noqa: E402


def _run(embodiment, m, out_dir: Path, name: str, moves: dict, tail_s: float = 1.5) -> dict:
    hz = float(embodiment.info.control_hz)
    res = kit.run(embodiment, moves, out_dir / name, seed=0, extra=demo.make_extra(m),
                  limit_s=float(m.RUN_LIMIT_STEPS) / hz, tail_s=tail_s, sheet_frames=8, record=False)  # fmt: skip
    rows = res["rows"]
    return {
        "score": float(rows[-1]["info"].get("score", 0.0)) if rows else 0.0,
        "steps": len(rows),
        "seconds": rows[-1]["t"] if rows else 0.0,
        "max_tray_slide_mm": max((r["extra"]["tray_slide_mm"] for r in rows), default=0.0),
        "peak_tray_btn_n": max((r["extra"]["tray_btn_n"] for r in rows), default=0.0),
        "peak_tray_base_n": max((r["extra"]["tray_base_n"] for r in rows), default=0.0),
        "report": res["report"],
        "worst_ik": res["worst_ik"],
    }


def slide_without_button(embodiment, m, out_dir: Path) -> dict:
    Move = kit.Move
    sx0, z_deck = P.TRAY_X, P.BASE_H
    l_rdy = demo.ready("left")
    moves = {"right": [], "left": [
        Move(1.6, pos=K.wing_tcp("left", sx0, z_deck, out=0.065), rot=K.WING_ROT["left"], grip=0.60, label="outside wing"),
        Move(1.0, pos=K.wing_tcp("left", sx0, z_deck), rot=K.WING_ROT["left"], grip=0.60, label="straddle wing"),
        Move(0.8, grip=demo.WING_HOLD, label="pinch wing"),
        Move(1.5, pos=K.wing_tcp("left", sx0 + 0.008, z_deck), push=(7.0, 0.0, 0.0), label="push locked"),
        Move(0.4, push=(0.0, 0.0, 0.0), label="relax"),
        Move(0.6, grip=0.65, label="release"),
        Move(1.0, pos=K.wing_tcp("left", sx0, z_deck, out=0.065), grip=demo.OPEN, label="retract"),
        Move(1.6, pos=l_rdy[0], rot=l_rdy[1], grip=demo.OPEN, label="ready"),
    ]}
    out = _run(embodiment, m, out_dir, "slide_without_button", moves, tail_s=0.5)
    out["pass"] = bool(out["score"] == 0.0 and out["max_tray_slide_mm"] < 8.0
                       and out["peak_tray_btn_n"] >= 2.0 and not out["report"]["separated"])
    return out


def yank_locked_base(embodiment, m, out_dir: Path) -> dict:
    Move = kit.Move
    sx0, z_deck = P.TRAY_X, P.BASE_H
    moves = {}
    for side in ("left", "right"):
        rdy = demo.ready(side)
        moves[side] = [
            Move(1.6, pos=K.wing_tcp(side, sx0, z_deck, out=0.065), rot=K.WING_ROT[side], grip=0.60, label="outside wing"),
            Move(1.0, pos=K.wing_tcp(side, sx0, z_deck), rot=K.WING_ROT[side], grip=0.60, label="straddle wing"),
            Move(0.8, grip=demo.WING_HOLD, label="pinch wing"),
            Move(1.4, pos=K.wing_tcp(side, sx0, z_deck + 0.040), push=(0.0, 0.0, 18.0), label="yank locked"),
            Move(0.6, pos=K.wing_tcp(side, sx0, z_deck), push=(0.0, 0.0, 0.0), label="set back"),
            Move(0.6, grip=0.65, label="release"),
            Move(1.0, pos=K.wing_tcp(side, sx0, z_deck, out=0.065), grip=demo.OPEN, label="retract"),
            Move(1.5, pos=rdy[0], rot=rdy[1], grip=demo.OPEN, label="ready"),
        ]
    out = _run(embodiment, m, out_dir, "yank_locked_base", moves, tail_s=0.5)
    out["pass"] = bool(out["score"] == 0.0 and out["report"]["worst_base_lift_mm"] > 8.0
                       and out["report"]["zero"] == "the scale base was lifted off its marks")
    return out


def tray_bench_fall(embodiment, m, out_dir: Path) -> dict:
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    (out_dir / "tray_bench_fall").mkdir(parents=True, exist_ok=True)
    embodiment.reset(Scene(id="tray_bench_fall", instruction=m.INSTRUCTION), seed=0)
    m.mk.write_pose("weighing_tray", (0.50, 0.40, m.TOP + 0.20), (0.0, 0.0, 0.0, 1.0))  # over bare bench, clear of base
    hold = Action(data=pack(list(READY) + [1.0], list(READY) + [1.0]))
    res = None
    for _ in range(25):
        res = embodiment.step(hold)
    assert res is not None
    rep = m.report()
    out = {"score": float(res.info.get("score", 0.0)), "steps": 25, "seconds": 1.25, "report": rep}
    out["pass"] = bool(out["score"] == 0.0
                       and rep["zero"] == "the weighing tray fell onto or off the bench")
    return out


def unlocked_half(embodiment, m, out_dir: Path) -> dict:
    Move = kit.Move
    z_deck = P.BASE_H
    l_rdy, r_rdy = demo.ready("left"), demo.ready("right")
    u = demo.unlock_moves()
    right = u["right"] + [
        Move(2.0, pos=r_rdy[0], rot=r_rdy[1], grip=demo.OPEN, label="ready"),
        Move(1.4, label="done"),
    ]
    left = u["left"] + [
        Move(0.6, grip=0.65, label="release wing"),
        Move(1.0, pos=K.wing_tcp("left", P.TRAY_X + P.SLIDE_HAND, z_deck, out=0.065), grip=demo.OPEN, label="retract"),
        Move(1.8, pos=l_rdy[0], rot=l_rdy[1], grip=demo.OPEN, label="ready"),
    ]
    out = _run(embodiment, m, out_dir, "unlocked_half", {"left": left, "right": right}, tail_s=1.0)
    out["pass"] = bool(out["score"] == 0.5 and out["report"]["zero"] is None
                       and out["report"]["separated"] and out["report"]["tray_slide_mm"] >= 20.0)
    return out


TRIALS = {
    "slide_without_button": slide_without_button,
    "yank_locked_base": yank_locked_base,
    "tray_bench_fall": tray_bench_fall,
    "unlocked_half": unlocked_half,
}


def main() -> int:
    out_dir = Path(sys.argv[1])
    out_dir.mkdir(parents=True, exist_ok=True)
    names = sys.argv[2].split(",") if len(sys.argv) > 2 else list(TRIALS)
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    summary = {}
    ok = True
    for name in names:
        res = TRIALS[name](embodiment, m, out_dir)
        (out_dir / f"{name}.json").write_text(json.dumps(res, indent=1, default=str))
        print(f"TRIAL {name} " + json.dumps(res, default=str), flush=True)
        summary[name] = res
        ok &= bool(res["pass"])
    (out_dir / "trials_summary.json").write_text(json.dumps({"pass": ok, "trials": summary}, indent=1, default=str))
    print(f"TRIALS_ALL pass={ok}", flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
