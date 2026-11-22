#!/usr/bin/env python3
"""The scripted reference run of the specimen-pan task: open loop, every pose from ``checks.py`` (reach, margins and
clearances checked there); it reads nothing from the simulator for its plan and places nothing.

    scripts/rta demo <id> mech/tasks/specimen_hat [--check | --no-video] [--quick]

1. Left hand: closes across the near side arm of the seat (83.4 mm, 33 N a finger), raises the seat to 42 degrees
   and holds it there against its 19-24 N weight at the grasp.
2. Right hand: pinches the middle of the pan's front wall (10 N a finger), lifts the pan 127 mm, turns it 90 degrees
   beside the deck, carries it under the raised seat over the opening, sets both wings on the rim and lets go.
3. Left hand: lowers the seat onto the wings, lets go and both hands go back to their ready pose.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as K  # noqa: E402

SEAT_OPEN = (K.SEAT_ARM_W + 0.009) / K.MAX_JAW  # 92.4 mm round the 83.4 mm arm
SEAT_PINCH = (K.SEAT_ARM_W - 2 * 0.0066) / K.MAX_JAW  # sent to 70.2 mm: 6.6 mm a finger x 5,000 N/m = 33 N a finger
PAN_OPEN = 0.040 / K.MAX_JAW  # over the wall before the pinch
PAN_LET_GO = K.PAN_LET_GO / K.MAX_JAW  # round the wall when letting go: the outer finger stays off the rim
PAN_PINCH = 0.0  # sent shut on the 4 mm wall: 2 mm a finger x 5,000 N/m = 10 N a finger


def ready(side: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, dtype=float), K.BASES[side])


def hold(deg: float, share: float = 1.0) -> np.ndarray:
    """The push (N, world) that carries the seat's weight at the grasp, raised by ``deg``."""
    return share * K.seat_lift(deg) * K.seat_normal(deg)


def plan() -> dict[str, list]:
    import demo_kit as kit

    Move = kit.Move
    start = K.pan_origin(K.P.PAN_START, 0.0)
    path = {name: (origin, yaw) for name, origin, yaw in K.pan_path()}

    def on_pan(name: str, up: float = 0.0):
        return K.right_on_pan(*path[name], up=up)

    right = [
        Move(1.6, *K.right_on_pan(start, K.P.PAN_START_YAW_DEG, up=0.040), grip=PAN_OPEN, label="over the wall"),
        Move(1.0, *K.right_on_pan(start, K.P.PAN_START_YAW_DEG), label="on the wall"),
        Move(0.6, grip=PAN_PINCH, label="pinch the wall"),
        Move(1.2, *on_pan("lifted"), label="lift the pan"),
        Move(1.4, *on_pan("turning"), label="turn 45"),
        Move(1.4, *on_pan("turned"), label="turn 90"),
    ]
    t_turned = sum(m.seconds for m in right)
    left = [
        Move(1.6, *K.left_on_seat(0.0, up=0.040), grip=SEAT_OPEN, label="over the seat"),
        Move(1.0, *K.left_on_seat(0.0), label="round the arm"),
        Move(0.6, grip=SEAT_PINCH, label="pinch the arm"),
        Move(2.0, *K.left_on_seat(20.0), push=hold(20.0), label="raise 20"),
        Move(2.0, *K.left_on_seat(K.SEAT_HOLD_DEG), push=hold(K.SEAT_HOLD_DEG), label="raise 42"),
    ]
    t_up = sum(m.seconds for m in left)
    if t_turned < t_up + 0.3:
        right.append(Move(t_up + 0.3 - t_turned, label="wait for the seat"))
    right += [
        Move(1.8, *on_pan("over the opening"), label="under the seat"),
        Move(1.6, *on_pan("wings on the rim"), label="wings on the rim"),
        Move(0.6, grip=PAN_LET_GO, label="let go"),
        Move(1.0, *on_pan("wings on the rim", up=0.050), label="up"),
    ]
    t_clear = sum(m.seconds for m in right)
    r_ready = ready("right")
    right += [Move(1.0, pos=right[-1].pos + np.array([0.0, -0.080, 0.030]), label="back"), Move(1.6, *r_ready, label="ready")]
    left += [
        Move(t_clear - t_up, *K.left_on_seat(K.SEAT_HOLD_DEG), push=hold(K.SEAT_HOLD_DEG), label="hold up"),
        Move(1.4, *K.left_on_seat(20.0), push=hold(20.0), label="lower 20"),
        Move(1.6, *K.left_on_seat(K.SEAT_REST_DEG + 0.4), push=hold(K.SEAT_REST_DEG, 0.0), label="onto the wings"),
        Move(0.6, grip=SEAT_OPEN, label="let go"),
        Move(1.0, *K.left_on_seat(K.SEAT_REST_DEG, up=0.040), label="up"),
        Move(1.6, *ready("left"), label="ready"),
    ]
    return {"left": left, "right": right}


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
        return {k: rep.get(k) for k in ("seat_deg", "pan_xyz_m", "pan_tilt_deg", "pan_across_deg", "wing_tips_over_rim_mm", "pan_lowest_mm",
                                        "pan_on_deck_N", "seat_on_pan_N", "pan_bridged", "seat_down", "hands_clear_mm")} | {
            "seat_hands_N": on("toilet_seat"), "pan_hands_N": on("specimen_pan"), "hand_seat_mm": round(1000.0 * m.hand_to_seat(), 1)}  # fmt: skip

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
        "peak_seat_deg": max((r["extra"]["seat_deg"] for r in rows), default=0.0),
        "seat_hand_peak_N": peak("seat_hands_N"),
        "seat_per_finger_peak_N": round(0.5 * peak("seat_hands_N"), 1),
        "pan_hand_peak_N": peak("pan_hands_N"),
        "pan_per_finger_peak_N": round(0.5 * peak("pan_hands_N"), 1),
        "seat_on_pan_end_N": rows[-1]["extra"]["seat_on_pan_N"] if rows else None,
        "end_state": rows[-1]["extra"] if rows else {},
        "report": result.get("report"),
        "worst_ik": result.get("worst_ik"),
    }
    (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
    print("READINGS " + json.dumps(readings, default=str), flush=True)
    return 0 if readings["score_end"] == 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
