#!/usr/bin/env python3
"""Scripted bimanual full-score reference demo for holesaw_arbor.

    scripts/rta demo <id> mech/tasks/holesaw_arbor [--no-video] [--quick]
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

OPEN = 1.0
# Finger targets with FINGER_STIFFNESS = 5000 N/m:
# - Ø 42.0 mm collar -> 30 mm target (6.0 mm/finger = ~30 N/pad <= 40 N limit)
# - Ø 63.5 mm hole saw -> 52 mm target (5.75 mm/finger = ~28.8 N/pad <= 40 N limit)
COLLAR_GRIP = 0.030 / K.MAX_JAW
SAW_GRIP = 0.052 / K.MAX_JAW
ZERO_PUSH = (0.0, 0.0, 0.0)


def ready(side: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, dtype=float), K.BASES[side])


def plan() -> dict[str, list]:
    import demo_kit as kit

    Move = kit.Move
    mat_xy = (P.X_MAT, P.Y_MAT)
    arb_xy = (P.X_ARBOR, P.Y_ARBOR)
    z_mat = P.MAT_SIZE[2]
    z_entry = P.Z_ENTRY
    z_stop = P.Z_SHOULDER
    z_align = P.Z_SHOULDER + P.gap_for_yaw_deg(P.ALIGN_DEG)

    # Stroke 2 commands 214 deg CW (+50 -> -55 -> -164 deg) so the cup firmly reaches the +415 deg hard shoulder stop,
    # then +59 deg CCW back-off (-164 -> -105 deg) lands right on +360.0 deg (phase_err < 1.5 deg)!
    yaw_stop_2 = math.radians(-164.0)
    yaw_align_2 = math.radians(-105.0)

    # Phase 1: Right arm pulls the detented collar down (8 -> 0 mm) and parks clear at -y; meanwhile Left arm grips hole_saw on mat
    right = [
        Move(1.8, pos=K.collar_tcp(P.PIN_STROKE, back=0.075)[0], rot=K.COLLAR_ROT, grip=OPEN, label="approach collar"),
        Move(0.9, pos=K.collar_tcp(P.PIN_STROKE)[0], label="around collar up"),
        Move(0.6, grip=COLLAR_GRIP, label="grip collar up"),
        Move(1.2, pos=K.collar_tcp(0.0, dz=-0.001)[0], push=(0.0, 0.0, -14.0), label="pull collar down"),
        Move(0.3, push=ZERO_PUSH, label="settle collar down"),
        Move(0.5, grip=OPEN, label="release collar down"),
        Move(1.1, pos=K.collar_tcp(0.0, back=0.125, dz=0.020)[0], label="park clear"),
    ]
    t_collar_ready = sum(m.seconds for m in right)  # 6.4 s

    left = [
        Move(2.2, pos=K.saw_tcp(mat_xy, z_mat + 0.055, K.SAW_YAW_START)[0], rot=kit.down(K.SAW_YAW_START), grip=OPEN, label="over mat"),
        Move(1.0, pos=K.saw_tcp(mat_xy, z_mat, K.SAW_YAW_START)[0], label="around saw on mat"),
        Move(0.7, grip=SAW_GRIP, label="grip saw"),
        Move(1.3, pos=K.saw_tcp(mat_xy, K.SAW_CARRY_Z, K.SAW_YAW_START)[0], label="lift saw"),
    ]
    # Ensure collar is fully retracted & right hand parked clear before left moves over the arbor
    if sum(m.seconds for m in left) < t_collar_ready:
        left.append(Move(round(t_collar_ready - sum(m.seconds for m in left), 3), label="wait collar clear"))

    # Phase 2: Left arm lowers cup coaxially onto thread entry, threads CW to +415 deg stop, backs off 55 deg CCW to +360 deg, clears
    left += [
        Move(1.5, pos=K.saw_tcp(arb_xy, K.SAW_CARRY_Z, K.SAW_YAW_START)[0], label="over arbor pilot"),
        Move(1.4, pos=K.saw_tcp(arb_xy, z_entry - 0.0006, K.SAW_YAW_START)[0], push=(0.0, 0.0, -3.5), label="seat thread entry"),
        Move(0.3, push=ZERO_PUSH, label="thread caught"),
        # Stroke 1: +210 deg CW (two 105 deg halves so shortest-turn SLERP stays strictly CW)
        Move(1.2, pos=K.saw_tcp(arb_xy, P.Z_SHOULDER + P.gap_for_yaw_deg(105.0), K.SAW_YAW_MID)[0], rot=kit.down(K.SAW_YAW_MID), label="screw CW 1a"),
        Move(1.2, pos=K.saw_tcp(arb_xy, P.Z_SHOULDER + P.gap_for_yaw_deg(210.0), K.SAW_YAW_END)[0], rot=kit.down(K.SAW_YAW_END), label="screw CW 1b"),
        # Single in-place regrasp
        Move(0.5, grip=OPEN, label="open regrasp"),
        Move(0.8, pos=K.saw_tcp(arb_xy, P.Z_SHOULDER + P.gap_for_yaw_deg(210.0) + 0.004, K.SAW_YAW_MID)[0], rot=kit.down(K.SAW_YAW_MID), label="cock CCW 1"),
        Move(0.8, pos=K.saw_tcp(arb_xy, P.Z_SHOULDER + P.gap_for_yaw_deg(210.0), K.SAW_YAW_START)[0], rot=kit.down(K.SAW_YAW_START), label="cock CCW 2"),
        Move(0.6, grip=SAW_GRIP, label="regrip saw"),
        # Stroke 2: +207 deg CW right into the +415.0 deg hard shoulder stop, then 55 deg CCW back-off to +360.0 deg!
        Move(1.2, pos=K.saw_tcp(arb_xy, P.Z_SHOULDER + P.gap_for_yaw_deg(315.0), K.SAW_YAW_MID)[0], rot=kit.down(K.SAW_YAW_MID), label="screw CW 2a"),
        Move(1.3, pos=K.saw_tcp(arb_xy, z_stop, yaw_stop_2)[0], rot=kit.down(yaw_stop_2), label="screw to +415 stop"),
        Move(1.2, pos=K.saw_tcp(arb_xy, z_align, yaw_align_2)[0], rot=kit.down(yaw_align_2), label="back off 55 deg CCW"),
        Move(0.6, grip=OPEN, label="release aligned saw"),
        Move(1.0, pos=K.saw_tcp(arb_xy, K.SAW_CARRY_Z + 0.010, yaw_align_2)[0], label="lift clear of cup"),
    ]
    t_left_clear = sum(m.seconds for m in left)
    left.append(Move(2.2, pos=ready("left")[0], rot=ready("left")[1], label="ready"))

    # Phase 3: Right arm waits until Left hand lifts clear, then grips collar and pushes it UP (+8 mm) locking both pins through the holes
    right.append(Move(round(t_left_clear - sum(m.seconds for m in right), 3), label="wait for back-off"))
    right += [
        Move(1.1, pos=K.collar_tcp(0.0, back=0.055)[0], label="approach retracted collar"),
        Move(0.8, pos=K.collar_tcp(0.0)[0], label="around collar down"),
        Move(0.6, grip=COLLAR_GRIP, label="grip collar down"),
        Move(1.3, pos=K.collar_tcp(P.PIN_STROKE, dz=0.0005)[0], push=(0.0, 0.0, 16.0), label="push collar locked"),
        Move(0.3, push=ZERO_PUSH, label="settle collar locked"),
        Move(0.5, grip=OPEN, label="release locked collar"),
        Move(1.0, pos=K.collar_tcp(P.PIN_STROKE, back=0.095)[0], label="back off collar"),
        Move(2.0, pos=ready("right")[0], rot=ready("right")[1], label="ready"),
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
        ok = all(dry[s]["worst_ik_mm"] <= 0.5 and dry[s]["smallest_margin_deg"] >= 5.0 for s in ("left", "right"))
        return 0 if ok else 1

    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk

    def finger_forces(body_name: str) -> dict[str, float]:
        """Per-pad mean normal force (N) from each robot arm's two opposing finger pads on ``body_name``."""
        totals: dict[str, float] = {}
        for c in mk.contacts(body_name):
            if c.other.endswith("robot"):
                totals[c.other] = round(totals.get(c.other, 0.0) + 0.5 * abs(float(c.force)), 2)
        return totals

    def extra() -> dict:
        return {
            "collar_z_mm": round(1000.0 * m.collar_travel(), 2),
            "thread_engaged": m.thread_engaged(),
            "thread_cw_deg": round(m.thread_cw_deg(), 2),
            "shoulder_gap_mm": round(1000.0 * m.saw_shoulder_gap(), 3),
            "phase_err_deg": round(m.saw_phase_err_deg(), 2),
            "collar_fingers_N": finger_forces("collar"),
            "saw_fingers_N": finger_forces("hole_saw"),
            "pin_contact_N": round(mk.contact_force("collar", "hole_saw"), 2),
            "hands_clear_mm": round(1000.0 * m.hands_clear(), 1),
            "hold": m.STATE.get("hold", 0),
        }

    folder = out / "full"
    record = "--no-video" not in sys.argv[2:] and "--check" not in sys.argv[2:]
    latest: dict = {}

    def director(_t, _arms, pictures) -> None:
        if pictures is not None:
            latest.update(pictures)

    result = kit.run(
        embodiment, moves, folder, seed=0, director=director, extra=extra,
        limit_s=float(m.RUN_LIMIT_STEPS) / 20.0, tail_s=4.5, sheet_frames=16, record=record,
    )
    rows = result["rows"]
    if "top_cam" in latest:
        save_end_picture(latest["top_cam"], folder)

    first_qtr = next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.25), None)
    first_half = next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.5), None)
    first_full = next((r["t"] for r in rows if r["info"].get("score", 0.0) == 1.0), None)

    collar_trace = [
        {"t": r["t"], "label": r["right"], "collar_z_mm": r["extra"]["collar_z_mm"], "fingers_N": r["extra"]["collar_fingers_N"]}
        for r in rows if r["extra"]["collar_fingers_N"]
    ]
    thread_trace = [
        {"t": r["t"], "label": r["left"], "cw_deg": r["extra"]["thread_cw_deg"], "gap_mm": r["extra"]["shoulder_gap_mm"], "phase_deg": r["extra"]["phase_err_deg"], "fingers_N": r["extra"]["saw_fingers_N"]}
        for r in rows if r["extra"]["thread_engaged"] and r["extra"]["saw_fingers_N"]
    ]

    readings = {
        "score_at_end": rows[-1]["info"].get("score"),
        "seconds": rows[-1]["t"],
        "steps": len(rows),
        "first_0p25_s": first_qtr,
        "first_0p50_s": first_half,
        "first_1p00_s": first_full,
        "peak_CW_deg": max(r["extra"]["thread_cw_deg"] for r in rows),
        "end_cw_deg": rows[-1]["extra"]["thread_cw_deg"],
        "end_shoulder_gap_mm": rows[-1]["extra"]["shoulder_gap_mm"],
        "end_phase_err_deg": rows[-1]["extra"]["phase_err_deg"],
        "end_collar_z_mm": rows[-1]["extra"]["collar_z_mm"],
        "end_hands_clear_mm": rows[-1]["extra"]["hands_clear_mm"],
        "both_arms_ready_at_end": {side: rows[-1][side] == "done" for side in ("left", "right")},
        "collar_action_samples": collar_trace[::6],
        "thread_action_samples": thread_trace[::10],
        "report": result.get("report"),
        "worst_ik": result.get("worst_ik"),
        "median_step_ms": result.get("median_step_ms"),
    }
    (folder / "readings.json").write_text(json.dumps(readings, indent=2, default=str))
    print("READINGS " + json.dumps({k: v for k, v in readings.items() if not k.endswith("_samples")}, default=str), flush=True)
    return 0 if readings["score_at_end"] == 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
