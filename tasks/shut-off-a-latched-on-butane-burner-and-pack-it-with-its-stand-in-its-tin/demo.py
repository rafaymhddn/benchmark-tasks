#!/usr/bin/env python3
"""Scripted full-score reference run for ``burner_stow``, plus offline IK dry-run check.

    scripts/rta demo <id> mech/tasks/burner_stow [--check|--no-video] [--quick]

Open-loop trajectory built from ``checks.py``'s verified poses (zero write_pose / set_joint):
1. Left hand grasps the 38 mm tank at its ``left_cam``-visible start pose, carries ``burner + stand`` to the central
   workspace point ``(0.300, +0.140)`` m, and braces it upright while the closed right fingertip slides ``lock_slider``
   forward through its 6.0 mm stroke against the 10.0 N detent.
2. Left hand lifts and turns the burner horizontal; right hand grasps the 60 mm round stand and pulls it axially past
   the 12.0 mm socket engagement against 8.0 N resistance until ``w_stand`` parts cleanly.
3. Right hand sets the detached stand flat in the front pocket of ``tin_base`` and clears high above the open lid;
   left hand softens its grip (<= 40 N/finger throughout) to lay the burner gently flat in the main pocket.
4. Right hand tips ``tin_lid`` from 108 deg past 90 deg to 73 deg so gravity closes it flush onto ``tin_base``, and
   both arms return to ``READY`` while the 1.0 s rest hold completes.
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

OPEN = 1.0
CLOSED = 0.0
FINGER_STIFFNESS = 5000.0  # N/m per finger drive in yam_arena
# Brace grip (~29 N/pad) during the 10 N lock push and 8 N axial stand pull; softened (~20 N/pad) while lowering into tin
# so every step stays strictly <= 40 N per finger pad.
TANK_GRIP = (P.TANK_D - 2.0 * (17.2 / FINGER_STIFFNESS)) / K.MAX_JAW
TANK_SOFT_GRIP = (P.TANK_D - 2.0 * (12.0 / FINGER_STIFFNESS)) / K.MAX_JAW
STAND_GRIP = (P.STAND_D - 2.0 * (27.0 / FINGER_STIFFNESS)) / K.MAX_JAW

LOCK_PUSH_N = tuple(float(v) for v in (13.5 * K.XBW + 3.0 * K.YBW))
PULL_STAND_N = (9.5, 0.0, 0.0)
BRACE_PULL_N = (-9.0, 0.0, 0.0)
ZERO_PUSH = (0.0, 0.0, 0.0)


def ready(side: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, dtype=float), K.BASES[side])


def plan() -> dict[str, list]:
    import demo_kit as kit

    Move = kit.Move
    p_lid_pre, r_lid_pre = K.lid_push_pose(P.LID_OPEN_DEG, back_mm=20.0)
    p_lid_108, r_lid_108 = K.lid_push_pose(P.LID_OPEN_DEG, back_mm=-2.0)
    p_lid_88, r_lid_88 = K.lid_push_pose(88.0, back_mm=-2.0)
    p_lid_73, r_lid_73 = K.lid_push_pose(73.0, back_mm=-2.0)
    p_lid_retract, r_lid_retract = K.lid_push_pose(80.0, back_mm=28.0)

    high_outside = np.array([0.365, 0.395, K.TOP + 0.235])
    left = [
        # Phase 1a: arc high outside burner (z = TOP + 235 mm > 183 mm burner top), grasp tank, carry to WORK_ORIGIN
        Move(1.0, pos=high_outside, grip=OPEN, label="over start"),
        Move(1.0, pos=K.P_TANK_START - 0.035 * K.R_TANK_START[:, 2], rot=K.R_TANK_START, label="to start tank"),
        Move(0.8, pos=K.P_TANK_START, label="on start tank"),
        Move(0.6, grip=TANK_GRIP, label="grasp tank"),
        Move(0.8, pos=K.P_TANK_START + np.array([0.0, 0.0, 0.032]), label="lift start"),
        Move(1.4, pos=K.P_TANK_WORK + np.array([0.0, 0.0, 0.030]), rot=K.R_TANK_WORK, label="carry to work"),
        Move(0.8, pos=K.P_TANK_WORK, label="set at work"),
        # Phase 1b: hold upright at WORK_ORIGIN while Right slides lock_slider forward (t = 6.3 .. 10.1 s)
        Move(3.8, label="hold for lock"),
        # Phase 2: lift clear and present horizontal heel for stand pull-off (t = 10.1 .. 17.8 s)
        Move(1.2, pos=K.P_TANK_WORK + np.array([0.0, 0.0, 0.055]), label="lift burner"),
        Move(2.0, pos=K.P_TANK_PULL, rot=K.R_TANK_FLAT, label="present stand"),
        Move(1.9, label="wait stand grip"),
        Move(0.4, push=BRACE_PULL_N, label="brace pull"),
        Move(1.8, label="hold against pull"),
        Move(0.4, push=ZERO_PUSH, label="unbrace pull"),
        # Phase 3: soften grip (<= 40 N/pad) and lay burner gently flat in main pocket (t = 17.8 .. 28.5 s)
        Move(3.4, pos=K.P_TANK_PULL + np.array([0.0, 0.0, 0.020]), grip=TANK_SOFT_GRIP, label="hold clear"),
        Move(1.8, pos=K.P_BURNER_TIN + np.array([0.0, 0.0, 0.095]), rot=K.R_TANK_FLAT, label="burner over tin"),
        Move(1.5, pos=K.P_BURNER_TIN, label="lower burner"),
        Move(0.7, grip=OPEN, label="release burner"),
        Move(1.1, pos=K.P_BURNER_TIN + np.array([0.0, 0.015, 0.110]), label="clear burner"),
        Move(2.2, pos=ready("left")[0], rot=ready("left")[1], label="ready"),
    ]

    right = [
        # Phase 1: wait for Left to set burner at WORK_ORIGIN, then slide lock_slider +6.0 mm forward (t = 0.0 .. 10.5 s)
        Move(2.4, pos=K.P_LOCK_START - 0.035 * K.R_LOCK[:, 2], rot=K.R_LOCK, grip=CLOSED, label="outside lock"),
        Move(4.2, label="wait work set"),
        Move(0.8, pos=K.P_LOCK_START, label="behind lock"),
        Move(2.0, pos=K.P_LOCK_END, push=LOCK_PUSH_N, label="slide lock off"),
        Move(0.4, push=ZERO_PUSH, label="release push"),
        Move(0.8, pos=K.P_LOCK_START - 0.040 * K.R_LOCK[:, 2], label="retract lock"),
        # Phase 2: grasp 60 mm stand and pull axially > 12 mm against 8 N socket friction (t = 10.0 .. 17.3 s)
        Move(1.6, pos=np.array([0.310, -0.055, K.TOP + 0.150]), rot=K.R_STAND_PULL, grip=OPEN, label="clear to stand"),
        Move(1.4, pos=K.P_STAND_GRAB - 0.045 * K.R_STAND_PULL[:, 2], label="stand pre"),
        Move(1.0, pos=K.P_STAND_GRAB, label="on stand"),
        Move(0.7, grip=STAND_GRIP, label="grasp stand"),
        Move(2.0, pos=K.P_STAND_PULLED + np.array([0.008, 0.0, 0.0]), push=PULL_STAND_N, label="pull stand off"),
        Move(0.6, push=ZERO_PUSH, label="settle pull"),
        # Phase 3: place detached stand flat in front pocket, then lift straight up (z = TOP + 245 mm) over open lid
        Move(1.8, pos=K.P_STAND_TIN + np.array([0.0, 0.0, 0.088]), rot=K.R_STAND_TIN, label="stand over tin"),
        Move(1.4, pos=K.P_STAND_TIN, label="lower stand"),
        Move(0.7, grip=OPEN, label="release stand"),
        Move(1.2, pos=np.array([0.200, -0.100, K.TOP + 0.245]), label="clear over lid"),
        # Phase 4: pass through ready, tip tin_lid gently past 90 deg to 73 deg (gravity closes to 0 deg), return to READY
        Move(1.6, pos=ready("right")[0], rot=ready("right")[1], grip=CLOSED, label="through ready"),
        Move(1.8, pos=p_lid_pre, rot=r_lid_pre, label="lid pre"),
        Move(1.2, label="wait burner stow"),
        Move(0.9, pos=p_lid_108, rot=r_lid_108, label="touch lid"),
        Move(1.1, pos=p_lid_88, rot=r_lid_88, label="tip lid past 90"),
        Move(1.0, pos=p_lid_73, rot=r_lid_73, label="tip lid to 73"),
        Move(0.9, pos=p_lid_retract, rot=r_lid_retract, label="retract from lid"),
        Move(1.8, pos=ready("right")[0], rot=ready("right")[1], label="ready"),
    ]
    return {"left": left, "right": right}


def save_end_picture(frame, folder: Path) -> None:
    import cv2
    import demo_kit as kit

    cv2.imwrite(str(folder / "end_top.png"), cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(folder / "end_top_policy.png"), cv2.cvtColor(kit.policy_picture(np.asarray(frame)), cv2.COLOR_RGB2BGR))


def _robot_contact_force(mk, body_name: str, side: str) -> float:
    tag = f"{side} robot"
    return round(sum(abs(float(c.force)) for c in mk.contacts(body_name) if c.other == tag), 2)


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

    def extra() -> dict:
        tank_tot = _robot_contact_force(mk, "burner", "left")
        stand_tot = _robot_contact_force(mk, "stand", "right")
        return {
            "lock_mm": round(1000.0 * mk.joint("lock_slider"), 2),
            "trig_out_mm": round(1000.0 * mk.joint("trigger"), 2),
            "child_mm": round(1000.0 * mk.joint("child_stop"), 2),
            "heel_mm": round(1000.0 * mk.joint("heel_fit"), 2),
            "detached": m.detached(),
            "lid_deg": round(m.lid_deg(), 2),
            "seam_gap_mm": round(m.seam_gap_mm(), 2),
            "lock_contact_N": _robot_contact_force(mk, "lock_slider", "right"),
            "tank_pinch_N": tank_tot,
            "tank_pad_N": round(0.5 * tank_tot, 2),
            "stand_pinch_N": stand_tot,
            "stand_pad_N": round(0.5 * stand_tot, 2),
            "hands_clear_mm": round(1000.0 * m.hands_clear(), 1),
            "hold": m.STATE.get("hold", 0),
        }

    folder = out / "full"
    record = ("--no-video" not in sys.argv[2:]) and ("--check" not in sys.argv[2:])
    latest: dict = {}

    def director(t, arms, pictures) -> None:
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
        tail_s=4.5,
        sheet_frames=16,
        record=record,
    )
    rows = result["rows"]
    if "top_cam" in latest:
        save_end_picture(latest["top_cam"], folder)

    lock_trace = [
        {
            "t": r["t"],
            "lock_mm": r["extra"]["lock_mm"],
            "trig_out_mm": r["extra"]["trig_out_mm"],
            "child_mm": r["extra"]["child_mm"],
            "right_spring_N": round(float(np.linalg.norm(r["force"]["right"])), 2),
            "lock_contact_N": r["extra"]["lock_contact_N"],
        }
        for r in rows
        if r["right"] in ("behind lock", "slide lock off")
    ]
    pull_trace = [
        {
            "t": r["t"],
            "heel_mm": r["extra"]["heel_mm"],
            "detached": r["extra"]["detached"],
            "right_pull_x_N": r["force"]["right"][0],
            "left_brace_x_N": r["force"]["left"][0],
            "stand_pad_N": r["extra"]["stand_pad_N"],
        }
        for r in rows
        if r["right"] in ("grasp stand", "pull stand off")
    ]
    tank_pads = [r["extra"]["tank_pad_N"] for r in rows if r["extra"]["tank_pad_N"] > 1.0]
    readings = {
        "score_at_end": rows[-1]["info"].get("score") if rows else None,
        "seconds": rows[-1]["t"] if rows else 0.0,
        "steps": len(rows),
        "first_0.25_s": next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.25), None),
        "first_0.5_s": next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.5), None),
        "first_1.0_s": next((r["t"] for r in rows if r["info"].get("score", 0.0) == 1.0), None),
        "peak_tank_pad_N": max(tank_pads, default=0.0),
        "median_tank_pad_N": round(float(np.median(tank_pads)), 2) if tank_pads else 0.0,
        "steps_over_40N_per_pad": sum(1 for r in rows if r["extra"]["tank_pad_N"] > 40.0 or r["extra"]["stand_pad_N"] > 40.0),
        "peak_stand_pad_N": max((r["extra"]["stand_pad_N"] for r in rows), default=0.0),
        "peak_lock_contact_N": max((r["extra"]["lock_contact_N"] for r in rows), default=0.0),
        "lock_force_vs_travel": lock_trace[::3],
        "heel_force_vs_travel": pull_trace[::3],
        "both_arms_ready_at_end": {side: rows[-1][side] == "done" for side in ("left", "right")} if rows else {},
        "report": result.get("report"),
        "worst_ik": result.get("worst_ik"),
        "median_step_ms": result.get("median_step_ms"),
    }
    (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
    print("READINGS " + json.dumps(readings, default=str), flush=True)
    return 0 if readings["score_at_end"] == 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
