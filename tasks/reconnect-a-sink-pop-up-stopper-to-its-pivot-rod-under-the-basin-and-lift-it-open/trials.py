#!/usr/bin/env python3
"""G2 PhysX trials for sink_stopper: verifies blocked shortcut, both zero rules, 0.5 seated, and 1.0 held open.

    scripts/rta simpy <id> mech/tasks/sink_stopper/trials.py /workspace/robocurve/outputs/<id>
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as K  # noqa: E402
import parts as P  # noqa: E402


def _ready_action():
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))


def _quat(m, rot) -> tuple[float, float, float, float]:
    q = m.mk.matrix_to_quat(rot)
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def _save_start_views(obs, out_dir: Path) -> None:
    import cv2

    out_dir.mkdir(parents=True, exist_ok=True)
    for cam, stem in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        bgr = cv2.cvtColor(np.asarray(obs.images[cam]), cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(out_dir / f"{stem}.png"), bgr)
        cv2.imwrite(str(out_dir / f"{stem}_policy.png"), cv2.resize(bgr, (320, 176), interpolation=cv2.INTER_AREA))


def _step_idle(embodiment, m, seconds: float) -> dict:
    hz = float(embodiment.info.control_hz)
    act = _ready_action()
    samples = []
    res = None
    for i in range(int(round(seconds * hz))):
        res = embodiment.step(act)
        if (i + 1) % 5 == 0:
            samples.append({"t": round((i + 1) / hz, 2), **m.report()})
    assert res is not None
    return {"end_score": float(res.info.get("score", 0.0)), "report": m.report(), "samples": samples}


def trial_blocked_closed(embodiment, m, out_dir: Path) -> dict:
    """Right arm pushes the rod forward while the stopper rests closed & unrotated: stem blocks tip >= 12 mm short (0.0)."""
    import demo_kit as kit

    Move = kit.Move
    w = K.poses()
    gc = K.RIGHT_GRAV_COMP
    r_on = w["right: on retracted rod"][1]
    r_push = K.rod_grip_world(P.X_SEAT - 0.020, 0.0) + gc
    r_pre = w["right: pre-grasp rod"][1]
    moves = {
        "right": [
            Move(1.2, pos=w["right: front pull-back"][1], rot=K.RIGHT_FRONT_ROT, grip=K.RIGHT_OPEN_JAW / K.MAX_JAW, label="front"),
            Move(1.3, pos=w["right: outside doorway"][1], rot=K.RIGHT_DOOR_ROT, label="door"),
            Move(1.3, pos=w["right: bypass pipe"][1], rot=K.RIGHT_BYPASS_ROT, label="bypass"),
            Move(1.2, pos=r_pre, rot=K.RIGHT_ROT, label="pre"),
            Move(1.1, pos=r_on, label="on_rod"),
            Move(0.7, grip=0.004 / K.MAX_JAW, label="pinch"),
            Move(1.8, pos=r_push, push=(-25.0, 0.0, 0.0), label="push_blocked"),
            Move(0.6, grip=K.RIGHT_OPEN_JAW / K.MAX_JAW, push=(0.0, 0.0, 0.0), label="release"),
            Move(1.0, pos=r_pre, label="back"),
            Move(1.2, pos=w["right: bypass pipe"][1], rot=K.RIGHT_BYPASS_ROT, label="bypass_out"),
        ]
    }
    res = kit.run(
        embodiment,
        moves,
        out_dir / "blocked_closed",
        seed=0,
        tail_s=1.0,
        limit_s=15.0,
        record=False,
        extra=lambda: {
            "mark_mm": round(1000.0 * m.collar_mark_gap(), 2),
            "f_rod_stop": round(m.mk.contact_force("pivot_rod", "stopper"), 2),
        },
    )
    min_mark = min(r["extra"]["mark_mm"] for r in res["rows"])
    peak_contact = max(r["extra"]["f_rod_stop"] for r in res["rows"])
    end_score = float(res["rows"][-1]["info"].get("score", 0.0))
    ok = min_mark >= 10.0 and peak_contact >= 0.5 and end_score == 0.0
    return {"pass": ok, "min_collar_mark_mm": min_mark, "peak_rod_stopper_force_N": peak_contact, "end_score": end_score, "report": res.get("report")}


def trial_lift_inspect_recover(embodiment, _m) -> dict:
    """Lifting stopper 64 mm up to inspect eye scores 0.0 while out, does NOT latch a foul, and recovers to 1.0 when coupled."""
    from inspect_robots import Scene

    m = embodiment.task.module
    embodiment.reset(Scene(id="inspect_recover", instruction=m.INSTRUCTION), seed=0)
    m.mk.write_pose("stopper", (P.X_DRAIN, P.Y_DRAIN, m.TOP + P.FLANGE_Z + 0.064), (0.0, 0.0, 0.0, 1.0))
    r_while_out = embodiment.step(_ready_action())
    score_while_out = float(r_while_out.info.get("score", 0.0))
    # Reinsert onto seated 17.5 deg tilted rod: since pull-out is unlatched, end state recovers to 1.0
    th = math.radians(17.5)
    m.mk.write_pose("pivot_rod", (P.X_SEAT, P.Y_DRAIN, m.TOP + P.PORT_Z), _quat(m, m.mk.axis_angle((0.0, 1.0, 0.0), th)))
    m.mk.write_pose("stopper", (P.X_DRAIN, P.Y_DRAIN, m.TOP + P.FLANGE_Z + 0.0115), (0.0, 0.0, 0.0, 1.0))
    out = _step_idle(embodiment, m, 2.5)
    out["score_while_lifted_out"] = score_while_out
    out["pass"] = score_while_out == 0.0 and out["end_score"] == 1.0 and out["report"]["zero"] is None
    return out


def trial_foul_rod_drop(embodiment, _m) -> dict:
    from inspect_robots import Scene

    m = embodiment.task.module
    embodiment.reset(Scene(id="foul_rod", instruction=m.INSTRUCTION), seed=0)
    m.mk.write_pose("pivot_rod", (0.440, P.Y_DRAIN - 0.035, m.TOP + 0.012), (0.0, 0.0, 0.0, 1.0))
    out = _step_idle(embodiment, m, 1.2)
    out["pass"] = out["end_score"] == 0.0 and out["report"]["zero"] == "pivot rod dropped onto cabinet floor"
    return out


def trial_seated_half(embodiment, _m, start_img_dir: Path | None = None) -> dict:
    """Ball seated horizontally through aligned eye without downward press -> settles at ~3.5 mm (< 8 mm) -> 0.5."""
    from inspect_robots import Scene

    m = embodiment.task.module
    obs = embodiment.reset(Scene(id="half", instruction=m.INSTRUCTION), seed=0)
    if start_img_dir is not None:
        _save_start_views(obs, start_img_dir)
    m.mk.write_pose("stopper", (P.X_DRAIN, P.Y_DRAIN, m.TOP + P.FLANGE_Z + 0.0055), (0.0, 0.0, 0.0, 1.0))
    m.mk.write_pose("pivot_rod", (P.X_SEAT, P.Y_DRAIN, m.TOP + P.PORT_Z), (0.0, 0.0, 0.0, 1.0))
    out = _step_idle(embodiment, m, 2.0)
    gap_mm = out["report"]["stopper gap (mm)"]
    f_rd = out["report"]["forces rod-drain, rod-stopper, stopper-drain (N)"][0]
    out["pass"] = out["end_score"] == 0.5 and 1.5 <= gap_mm < 8.0 and f_rd >= 3.0
    return out


def trial_pressed_open_full(embodiment, _m) -> dict:
    """Ball seated and rear rod tilted down 17.5 deg: pure 240 Hz PhysX cone-washer friction holds stopper open -> 1.0."""
    from inspect_robots import Scene

    m = embodiment.task.module
    embodiment.reset(Scene(id="full", instruction=m.INSTRUCTION), seed=0)
    th = math.radians(17.5)
    m.mk.write_pose("pivot_rod", (P.X_SEAT, P.Y_DRAIN, m.TOP + P.PORT_Z), _quat(m, m.mk.axis_angle((0.0, 1.0, 0.0), th)))
    m.mk.write_pose("stopper", (P.X_DRAIN, P.Y_DRAIN, m.TOP + P.FLANGE_Z + 0.0115), (0.0, 0.0, 0.0, 1.0))
    out = _step_idle(embodiment, m, 2.5)
    gap_mm = out["report"]["stopper gap (mm)"]
    forces = out["report"]["forces rod-drain, rod-stopper, stopper-drain (N)"]
    out["pass"] = out["end_score"] == 1.0 and 8.0 <= gap_mm <= 22.0 and forces[0] >= 1.5 and forces[1] >= 0.2
    return out


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module

    results = {
        "seated_half": trial_seated_half(emb, m, start_img_dir=out / "start_views"),
        "pressed_open_full": trial_pressed_open_full(emb, m),
        "lift_inspect_recover": trial_lift_inspect_recover(emb, m),
        "foul_rod_drop": trial_foul_rod_drop(emb, m),
        "blocked_closed": trial_blocked_closed(emb, m, out),
    }
    ok = True
    for name, res in results.items():
        (out / f"{name}.json").write_text(json.dumps(res, indent=1, default=str))
        brief = {k: v for k, v in res.items() if k != "samples"}
        print(f"TRIAL {name} " + json.dumps(brief, default=str), flush=True)
        ok &= bool(res["pass"])
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
