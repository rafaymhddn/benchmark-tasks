#!/usr/bin/env python3
"""The scripted reference run of the specimen-pan task: open loop, every pose from ``checks.py`` (reach, margins and
clearances checked there); it reads nothing from the simulator for its plan and places nothing.

    scripts/rta demo <id> mech/tasks/specimen_hat [--check | --no-video] [--quick]

The seat stays up on its check hinges by itself, so one hand does the work and the other stays at its ready pose:
the right hand pinches the pan's front wall (10 N a finger), lifts the pan 12 mm over the rim's height, turns it
90 degrees about the grasp, carries it over the opening, lowers it until both wings rest on the rim, lets go and goes
back to its ready pose. The raised seat is never touched.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as K  # noqa: E402

PAN_OPEN = 0.040 / K.MAX_JAW  # over the wall before the pinch
PAN_LET_GO = K.PAN_LET_GO / K.MAX_JAW  # round the wall when letting go
PAN_PINCH = 0.0  # sent shut on the 4 mm wall: 2 mm a finger x 5,000 N/m = 10 N a finger


def plan() -> dict[str, list]:
    import demo_kit as kit

    Move = kit.Move
    start, y0 = K.pan_origin(K.P.PAN_START, 0.0), K.P.PAN_START_YAW_DEG
    path = {name: (origin, yaw) for name, origin, yaw in K.pan_path()}

    def on_pan(name: str, up: float = 0.0):
        return K.right_on_pan(*path[name], up=up)

    up = on_pan("wings on the rim", up=0.050)
    right = [
        Move(1.6, *K.right_on_pan(start, y0, up=0.040), grip=PAN_OPEN, label="over the wall"),
        Move(1.0, *K.right_on_pan(start, y0), label="on the wall"),
        Move(0.6, grip=PAN_PINCH, label="pinch the wall"),
        Move(1.2, *on_pan("lifted"), label="lift the pan"),
        Move(1.4, *on_pan("turning"), label="turn 45"),
        Move(1.4, *on_pan("turned"), label="turn 90"),
        Move(1.8, *on_pan("over the opening"), label="over the opening"),
        Move(1.6, *on_pan("wings on the rim"), label="wings on the rim"),
        Move(0.6, grip=PAN_LET_GO, label="let go"),
        Move(1.0, *up, label="up"),
        Move(1.0, up[0] + K.BACK, up[1], label="back"),
        Move(1.6, *K.ready_pose("right"), label="ready"),
    ]
    return {"left": [], "right": right}


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    moves = plan()
    if "--quick" in sys.argv[2:]:
        from yam_arena import station

        print("QUICK " + json.dumps(kit.dry_run(moves, station.BASES)), flush=True)
        return 0
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk

    def on(body: str) -> dict[str, float]:
        totals: dict[str, float] = {}
        for c in mk.contacts(body):
            if c.other.endswith("robot"):
                totals[c.other] = round(totals.get(c.other, 0.0) + abs(float(c.force)), 2)
        return totals

    def extra() -> dict:
        rep = m.report()
        return {k: rep.get(k) for k in ("seat_deg", "pan_xyz_m", "pan_tilt_deg", "pan_across_deg", "wing_tips_over_rim_mm",
                                        "pan_lowest_over_water_mm", "pan_on_bowl_N", "seat_on_pan_N", "pan_bridged", "seat_up",
                                        "hands_clear_mm")} | {"seat_hands_N": on("toilet_seat"), "pan_hands_N": on("specimen_pan")}  # fmt: skip

    folder = out / "full"
    record = "--no-video" not in sys.argv[2:] and "--check" not in sys.argv[2:]
    latest: dict = {}

    def director(_t, _arms, pictures) -> None:
        if pictures is not None:
            latest.update(pictures)

    result = kit.run(embodiment, moves, folder, seed=0, director=director, extra=extra, limit_s=m.RUN_LIMIT_STEPS / 20.0,
                     tail_s=4.0, sheet_frames=16, record=record)  # fmt: skip
    rows = result["rows"]
    if "top_cam" in latest:
        import cv2

        frame = np.asarray(latest["top_cam"])
        cv2.imwrite(str(folder / "end_top.png"), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(folder / "end_top_policy.png"), cv2.cvtColor(kit.policy_picture(frame), cv2.COLOR_RGB2BGR))
    scores = [r["info"].get("score", 0.0) for r in rows]

    def peak(key: str) -> float:
        return max((max(r["extra"][key].values(), default=0.0) for r in rows), default=0.0)

    readings = {
        "score_end": scores[-1] if scores else 0.0,
        "steps": len(rows),
        "seconds": rows[-1]["t"] if rows else 0.0,
        "first_0.5_s": next((r["t"] for r, s in zip(rows, scores) if s >= 0.5), None),
        "first_1.0_s": next((r["t"] for r, s in zip(rows, scores) if s == 1.0), None),
        "min_seat_deg": min((r["extra"]["seat_deg"] for r in rows), default=None),
        "seat_hand_peak_N": peak("seat_hands_N"),
        "pan_hand_peak_N": peak("pan_hands_N"),
        "pan_per_finger_peak_N": round(0.5 * peak("pan_hands_N"), 1),
        "end_state": rows[-1]["extra"] if rows else {},
        "report": result.get("report"),
        "worst_ik": result.get("worst_ik"),
    }
    (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
    print("READINGS " + json.dumps(readings, default=str), flush=True)
    return 0 if readings["score_end"] == 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
