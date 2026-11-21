#!/usr/bin/env python3
"""Scripted bimanual full-score reference run for sink_stopper, plus offline --quick trajectory check.

    scripts/rta demo <id> mech/tasks/sink_stopper [--no-video] [--quick]

Open-loop plan (reads zero simulator state):
1. Left hand enters the basin from above, grasps the 35.6 mm chrome stopper cap, lifts it 7.5 mm, and turns it -90 deg
   so its hidden 8.5 x 10.0 mm clevis eye aligns with the rear +x drain port; meanwhile Right hand pulls back in front
   of the vanity doorway, descends into the open cabinet, and curves around the -y side of the 38 mm drain pipe to the
   clevis connector on the rear 5.5 mm rod.
2. Right hand pinches the rear rod and slides it 32 mm forward (-x), pushing the 18.3 mm ball through the 4-slot split
   snap-collar (~8-10 N click) into the compliant cone-washer seat while threading the 4.5 mm inner tip 10.65 mm past
   the stopper axis through the aligned eye.
3. Left hand releases the stopper cap and returns to READY; Right hand presses the rear rod down ~17.5 deg, lifting the
   stopper head ~11.5 mm open above the flange, releases the rod (held open by 240 Hz PhysX cone-washer friction), and
   withdraws cleanly through the front doorway back to READY while the 1.0 s hold completes.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as K  # noqa: E402
import parts as P  # noqa: E402

CAP_SQUEEZE_GRIP = 0.021 / K.MAX_JAW  # squeezes 35.6 mm cap (~38 N/finger, under 50 N/pad) with zero slip
ROD_SQUEEZE_GRIP = 0.004 / K.MAX_JAW  # squeezes 16 mm brass clevis block (~35 N/finger, under 50 N/pad)
LEFT_CAP_Z_G3 = K.TOP + P.FLANGE_Z + 0.0155
LEFT_OPEN_GRIP = 0.054 / K.MAX_JAW
RIGHT_OPEN_GRIP = K.RIGHT_OPEN_JAW / K.MAX_JAW
LEFT_LIFT_PUSH = (0.0, 0.0, 2.2)  # N upward feedforward: lifts stopper ~6.2 mm so eye aligns dead-centre with port
SEAT_PUSH = (-30.0, 0.0, 0.0)  # N along -x to snap 18.3 mm ball past the 4-slot collar lip flush into X_SEAT
PRESS_DOWN_PUSH = (-10.0, 0.0, -13.0)  # N forward + downward: keeps ball flush in seat while tilting rear rod ~15 deg


def ready(side: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, dtype=float), K.BASES[side])


def plan() -> dict[str, list]:
    import demo_kit as kit

    Move = kit.Move
    w = K.poses()
    gc = K.RIGHT_GRAV_COMP
    lc = np.array([-0.006, -0.005, -0.003], dtype=float)  # centres actual PhysX Left TCP onto (0.375, 0.035)
    l_on = np.array([P.X_DRAIN, P.Y_DRAIN, LEFT_CAP_Z_G3], dtype=float) + lc
    l_lift = l_on + np.array([0.0, 0.0, 0.0075])
    r_pre = w["right: pre-grasp rod"][1]
    r_on = w["right: on retracted rod"][1]
    r_seat_over = K.rod_grip_world(P.X_SEAT - 0.022, 0.0) + gc
    r_seated_hold = K.rod_grip_world(P.X_SEAT - 0.015, 0.0) + gc
    r_pressed = K.rod_grip_world(P.X_SEAT - 0.015, 16.0, extra_z=-0.002) + gc
    r_off = K.rod_grip_world(P.X_SEAT, K.RIGHT_PRESS_PITCH_DEG, extra_z=0.008) - 0.026 * K.RIGHT_PRESS_ROT[:, 2] + gc

    left = [
        Move(1.6, pos=w["left: hover over stopper"][1] + lc, rot=w["left: hover over stopper"][2], grip=LEFT_OPEN_GRIP, label="hover"),
        Move(1.0, pos=l_on, label="on_cap"),
        Move(0.7, grip=CAP_SQUEEZE_GRIP, label="grasp_cap"),
        Move(0.9, pos=l_lift, push=LEFT_LIFT_PUSH, label="lift"),
        Move(1.0, rot=K.left_rot(-53.0), push=LEFT_LIFT_PUSH, label="align_eye"),
        Move(4.4, push=LEFT_LIFT_PUSH, label="hold_aligned"),
        Move(0.6, grip=LEFT_OPEN_GRIP, push=(0.0, 0.0, 0.0), label="release_cap"),
        Move(0.8, pos=w["left: release & rise"][1] + lc, label="rise_clear"),
        Move(2.0, pos=ready("left")[0], rot=ready("left")[1], grip=1.0, label="ready"),
    ]

    right = [
        Move(1.2, pos=w["right: front pull-back"][1], rot=K.RIGHT_FRONT_ROT, grip=RIGHT_OPEN_GRIP, label="front_in"),
        Move(1.3, pos=w["right: outside doorway"][1], rot=K.RIGHT_DOOR_ROT, label="door_in"),
        Move(1.3, pos=w["right: bypass pipe"][1], rot=K.RIGHT_BYPASS_ROT, label="bypass_in"),
        Move(1.2, pos=r_pre, rot=K.RIGHT_ROT, label="pre_rod"),
        Move(1.1, pos=r_on, label="on_rod"),
        Move(0.7, grip=ROD_SQUEEZE_GRIP, label="pinch_rod"),
        Move(2.0, pos=r_seat_over, push=SEAT_PUSH, label="snap_seat"),
        Move(0.4, pos=r_seated_hold, push=(-8.0, 0.0, 0.0), label="seated"),
        Move(1.2, push=(-8.0, 0.0, 0.0), label="wait_left_off"),
        Move(1.6, pos=r_pressed, rot=K.RIGHT_PRESS_ROT, push=PRESS_DOWN_PUSH, label="press_down"),
        Move(0.5, push=(-4.0, 0.0, 0.0), label="settle_open"),
        Move(0.7, grip=RIGHT_OPEN_GRIP, label="release_rod"),
        Move(0.9, pos=r_off, rot=K.RIGHT_OFF_ROT, label="back_off"),
        Move(0.9, pos=r_pre, rot=K.RIGHT_ROT, label="clear_rod"),
        Move(1.0, pos=w["right: bypass pipe"][1], rot=K.RIGHT_BYPASS_ROT, label="bypass_out"),
        Move(1.0, pos=w["right: outside doorway"][1], rot=K.RIGHT_DOOR_ROT, label="door_out"),
        Move(1.0, pos=w["right: front pull-back"][1], rot=K.RIGHT_FRONT_ROT, label="front_out"),
        Move(1.4, pos=ready("right")[0], rot=ready("right")[1], grip=1.0, label="ready"),
    ]
    return {"left": left, "right": right}


def save_end_picture(frame, folder: Path) -> None:
    import cv2
    import demo_kit as kit

    cv2.imwrite(str(folder / "end_top.png"), cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(folder / "end_top_policy.png"), cv2.cvtColor(kit.policy_picture(np.asarray(frame)), cv2.COLOR_RGB2BGR))


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    moves = plan()
    if "--quick" in sys.argv[2:]:
        from yam_arena import station

        dry = kit.dry_run(moves, station.BASES)
        print("QUICK " + json.dumps(dry), flush=True)
        return 0

    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk

    def finger_forces(body_name: str) -> dict[str, float]:
        totals: dict[str, float] = {}
        for c in mk.contacts(body_name):
            if c.other.endswith("robot"):
                totals[c.other] = round(totals.get(c.other, 0.0) + abs(float(c.force)), 2)
        return totals

    def extra() -> dict:
        return {
            "gap_mm": round(1000.0 * m.stopper_gap(), 2),
            "mark_mm": round(1000.0 * m.collar_mark_gap(), 2),
            "pitch_deg": round(m.rod_pitch_deg(), 2),
            "f_rod_drain_N": round(mk.contact_force("pivot_rod", "drain"), 2),
            "f_rod_stop_N": round(mk.contact_force("pivot_rod", "stopper"), 2),
            "fingers_on_stop_N": finger_forces("stopper"),
            "fingers_on_rod_N": finger_forces("pivot_rod"),
            "hands_clear_mm": round(1000.0 * m.hands_clear(), 1),
            "hold": m.STATE.get("hold", 0),
        }

    folder = out / "full"
    record = "--no-video" not in sys.argv[2:]
    latest: dict = {}

    def director(_t, _arms, pictures) -> None:
        if pictures is not None:
            latest.update(pictures)

    result = kit.run(
        embodiment,
        moves,
        folder,
        seed=0,
        director=director,
        extra=extra,
        limit_s=float(m.RUN_LIMIT_STEPS) / 20.0,
        tail_s=5.0,
        sheet_frames=16,
        record=record,
    )
    rows = result["rows"]
    save_end_picture(latest["top_cam"], folder)
    first_half = next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.5), None)
    first_full = next((r["t"] for r in rows if r["info"].get("score", 0.0) == 1.0), None)
    seat_trace = [
        {
            "t": r["t"],
            "mark_mm": r["extra"]["mark_mm"],
            "gap_mm": r["extra"]["gap_mm"],
            "f_rod_drain_N": r["extra"]["f_rod_drain_N"],
            "f_rod_stop_N": r["extra"]["f_rod_stop_N"],
            "fingers_on_rod_N": r["extra"]["fingers_on_rod_N"],
        }
        for r in rows[::8]
    ]
    readings = {
        "score_at_end": rows[-1]["info"].get("score"),
        "seconds": rows[-1]["t"],
        "steps": len(rows),
        "first_0.5_at_s": first_half,
        "first_1.0_at_s": first_full,
        "peak_rod_drain_force_N": max(r["extra"]["f_rod_drain_N"] for r in rows),
        "peak_finger_on_stopper_N": max((max(r["extra"]["fingers_on_stop_N"].values(), default=0.0) for r in rows), default=0.0),
        "peak_finger_on_rod_N": max((max(r["extra"]["fingers_on_rod_N"].values(), default=0.0) for r in rows), default=0.0),
        "both_arms_ready_at_end": {side: rows[-1][side] == "done" for side in ("left", "right")},
        "trace_every_8_steps": seat_trace,
        "report": result.get("report"),
        "worst_ik": result.get("worst_ik"),
        "median_step_ms": result.get("median_step_ms"),
    }
    (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
    print("READINGS " + json.dumps(readings, default=str), flush=True)
    return 0 if readings["score_at_end"] == 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
