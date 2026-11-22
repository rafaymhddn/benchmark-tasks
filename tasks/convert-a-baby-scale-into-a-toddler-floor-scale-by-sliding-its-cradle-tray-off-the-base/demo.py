#!/usr/bin/env python3
"""Scripted bimanual reference run of ``mech/tasks/baby_scale_convert``.

    scripts/rta demo <id> mech/tasks/baby_scale_convert --check    # simulator check, no MP4 encoding
    scripts/rta demo <id> mech/tasks/baby_scale_convert            # full recording (three MP4s + setup.png + sheets)
    python mech/tasks/baby_scale_convert/demo.py --quick           # offline IK & joint-margin check (no simulator)

Open loop: reads nothing from the simulator. The right hand passes in front of the tray and presses the round
``release_btn`` (16 N down; it stops at 5.5 mm where the fingertip meets the apron, pressing ``scale_base`` onto its
rubber feet) while the left hand comes in past the open head end, pinches the 16 mm rolled edge of the wing (81 mm over
the bench, ~25 N a finger) and slides ``weighing_tray`` back: the hand travels 32 mm and the tray's centre about 23 mm
(pushed at one end it yaws about 2 deg), so both keyhole studs reach their mouths and the tongue passes the pawl (0.5).
The right hand lets the button spring back and takes the foot wing's edge the same way (grasps 509 mm apart); both arms
lift the scoop (grasps 111 mm over the bench), carry it onto the grey IKEA mat at (63, 0) cm, set it down level, open,
back off past the open ends, and return to READY (1.0).
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as K  # noqa: E402
import demo_kit as kit  # noqa: E402
import parts as P  # noqa: E402

OPEN = 1.0
# 6 mm opening on the 16 mm rolled edge: each finger stops 5 mm short of its target, 5 mm x 5000 N/m = ~25 N a finger
WING_HOLD = (P.EDGE_D - 0.010) / K.MAX_JAW
LET_GO = 0.30  # 28 mm opening at the set-down pose: 6 mm clear of the edge, lower jaw still over 4 mm above the bench
BTN_PUSH = (0.0, 0.0, -16.0)
SLIDE_PUSH = (4.5, 0.0, 0.0)
CARRY_LIFT_PUSH = (0.0, 0.0, 4.0)


def ready(side: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, dtype=float), K.BASES[side])


def unlock_moves() -> dict[str, list[kit.Move]]:
    """Synchronized two-arm moves (0..8.7 s) that hold ``release_btn`` down and slide ``weighing_tray`` about 23 mm back."""
    Move = kit.Move
    sx0, z_deck = P.TRAY_X, P.BASE_H
    right = [
        Move(1.6, pos=np.array([0.158, -0.180, K.TOP + 0.205]), rot=K.BTN_ROT, grip=0.0, label="front clear"),
        Move(1.2, pos=K.btn_tcp(lift=0.040), rot=K.BTN_ROT, grip=0.0, label="above button"),
        Move(0.9, pos=K.btn_tcp(depress=0.001), rot=K.BTN_ROT, grip=0.0, label="touch button"),
        Move(0.8, pos=K.btn_tcp(depress=0.0058), rot=K.BTN_ROT, grip=0.0, push=BTN_PUSH, push_at=K.BTN_TIP_LOCAL, label="hold button"),
        Move(2.6, label="hold button"),  # until the slide is done and the tongue is past the pawl
        Move(0.4, push=(0.0, 0.0, 0.0), label="relax button"),
        Move(1.2, pos=np.array([0.175, -0.240, K.TOP + 0.175]), rot=K.BTN_ROT, grip=0.60, label="off button"),
    ]
    left = [
        Move(1.6, pos=K.wing_tcp("left", sx0, z_deck, out=0.065), rot=K.WING_ROT["left"], grip=0.60, label="outside left wing"),
        Move(1.2, label="outside left wing"),
        Move(0.9, pos=K.wing_tcp("left", sx0, z_deck), rot=K.WING_ROT["left"], grip=0.60, label="straddle left wing"),
        Move(0.8, grip=WING_HOLD, label="pinch left wing"),
        Move(2.2, pos=K.wing_tcp("left", sx0 + P.SLIDE_HAND, z_deck), push=SLIDE_PUSH, label="slide back"),
        Move(0.4, push=(0.0, 0.0, 0.0), label="unlocked"),
        Move(1.6, label="steady scoop"),
    ]
    return {"left": left, "right": right}


def plan() -> dict[str, list[kit.Move]]:
    Move = kit.Move
    sx1 = P.TRAY_X + P.SLIDE_TARGET
    mx = P.MAT_X
    z_deck, z_lift, z_mat = P.BASE_H, K.Z_LIFT, K.Z_SET
    l_rdy, r_rdy = ready("left"), ready("right")

    u = unlock_moves()
    right = u["right"] + [
        Move(1.4, pos=K.wing_tcp("right", sx1, z_deck, out=0.065), rot=K.WING_ROT["right"], grip=0.60, label="outside right wing"),
        Move(1.0, pos=K.wing_tcp("right", sx1, z_deck), rot=K.WING_ROT["right"], grip=0.60, label="straddle right wing"),
        Move(0.8, grip=WING_HOLD, label="pinch right wing"),
        Move(1.4, pos=K.wing_tcp("right", sx1, z_lift), push=CARRY_LIFT_PUSH, label="lift scoop"),
        Move(2.6, pos=K.wing_tcp("right", mx, z_lift), push=CARRY_LIFT_PUSH, label="carry to mat"),
        Move(1.8, pos=K.wing_tcp("right", mx, z_mat), push=(0.0, 0.0, 0.0), label="lower on mat"),
        Move(0.8, grip=LET_GO, label="release scoop"),
        Move(1.2, pos=K.wing_tcp("right", mx, z_mat, out=0.075), grip=OPEN, label="retract off end"),
        Move(2.2, pos=r_rdy[0], rot=r_rdy[1], grip=OPEN, label="ready"),
    ]
    left = u["left"] + [
        Move(1.4, label="steady scoop"),
        Move(1.0, label="steady scoop"),
        Move(0.8, label="both hold"),
        Move(1.4, pos=K.wing_tcp("left", P.TRAY_X + P.SLIDE_HAND, z_lift), push=CARRY_LIFT_PUSH, label="lift scoop"),
        Move(2.6, pos=K.wing_tcp("left", mx, z_lift), push=CARRY_LIFT_PUSH, label="carry to mat"),
        Move(1.8, pos=K.wing_tcp("left", mx, z_mat), push=(0.0, 0.0, 0.0), label="lower on mat"),
        Move(0.8, grip=LET_GO, label="release scoop"),
        Move(1.2, pos=K.wing_tcp("left", mx, z_mat, out=0.075), grip=OPEN, label="retract off end"),
        Move(2.2, pos=l_rdy[0], rot=l_rdy[1], grip=OPEN, label="ready"),
    ]
    return {"left": left, "right": right}


def make_extra(m):
    mk = m.mk

    def extra() -> dict:
        hand_tray = {}
        for c in mk.contacts("weighing_tray"):
            if "robot" in c.other or "arm" in c.other.lower():
                hand_tray[c.other] = round(hand_tray.get(c.other, 0.0) + abs(float(c.force)), 2)
        t_pos, t_rot = mk.pose("weighing_tray")
        edges = [t_rot @ np.array([0.0, sgn * K.WING_Y, K.WING_LOCAL_Z]) + t_pos for sgn in (1.0, -1.0)]
        return {
            "tray_yaw_deg": round(float(np.degrees(np.arctan2(t_rot[1, 0], t_rot[0, 0]))), 2),
            "edge_x_mm": [round(1000 * float(e[0]), 1) for e in edges],
            "btn_mm": round(1000 * m.button_depression(), 2),
            "btn_spring_n": round(P.BTN_SPRING * max(0.0, m.button_depression()), 2),
            "tray_slide_mm": round(1000 * m.tray_slide_back(), 2),
            "base_lift_mm": round(1000 * m.base_corner_lift(), 2),
            "base_shift_mm": round(1000 * m.base_shift(), 2),
            "tray_tilt_deg": round(m.tilt_deg("weighing_tray"), 2),
            "tray_base_n": round(float(mk.contact_force("weighing_tray", "scale_base")), 2),
            "tray_btn_n": round(float(mk.contact_force("weighing_tray", "release_btn")), 2),
            "tray_mat_n": round(float(mk.contact_force("weighing_tray", "mat")), 2),
            "hand_tray_n": hand_tray,
            "hands_clear_mm": round(1000 * m.hands_clear(), 1),
            "hold": int(m.STATE.get("hold", 0)),
            "zero": m.STATE.get("zero"),
        }

    return extra


def quick() -> int:
    from yam_arena import station

    offline = K.run_offline()
    dry = kit.dry_run(plan(), station.BASES)
    out = {"offline_pass": offline["pass"], "dry_run": dry}
    print("QUICK " + json.dumps(out, indent=1), flush=True)
    ok = (offline["pass"]
          and all(dry[s]["worst_ik_mm"] <= 0.5 and dry[s]["smallest_margin_deg"] >= 5.0 for s in ("left", "right")))
    return 0 if ok else 1


def main() -> int:
    if "--quick" in sys.argv[1:]:
        return quick()
    import cv2

    flags = {a for a in sys.argv[1:] if a.startswith("--")}
    positional = [a for a in sys.argv[1:] if not a.startswith("--")]
    out = Path(positional[0]) / "demo"
    out.mkdir(parents=True, exist_ok=True)
    record = "--check" not in flags and "--no-video" not in flags

    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    moves = plan()
    latest: dict = {}

    def director(_t, _arms, pictures) -> None:
        if pictures is not None:
            latest.update(pictures)

    hz = float(embodiment.info.control_hz)
    result = kit.run(embodiment, moves, out, seed=0, director=director, extra=make_extra(m),
                     limit_s=float(m.RUN_LIMIT_STEPS) / hz, tail_s=6.0, sheet_frames=16, record=record)  # fmt: skip
    if "top_cam" in latest:
        cv2.imwrite(str(out / "end_top.png"), cv2.cvtColor(np.asarray(latest["top_cam"]), cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(out / "end_top_policy.png"), cv2.cvtColor(kit.policy_picture(np.asarray(latest["top_cam"])), cv2.COLOR_RGB2BGR))

    rows = result["rows"]
    first_half = next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.5), None)
    first_full = next((r["t"] for r in rows if r["info"].get("score", 0.0) == 1.0), None)
    slide_trace = [{"t": r["t"], "btn_mm": r["extra"]["btn_mm"], "btn_spring_n": r["extra"]["btn_spring_n"],
                    "tray_slide_mm": r["extra"]["tray_slide_mm"], "left_force_xyz_n": r["force"]["left"]}
                   for r in rows if r["left"] == "slide back"][::4]  # fmt: skip
    readings = {
        "score": float(rows[-1]["info"].get("score", 0.0)) if rows else 0.0,
        "steps": len(rows),
        "seconds": rows[-1]["t"] if rows else 0.0,
        "run_limit_steps": m.RUN_LIMIT_STEPS,
        "terminated": result["terminated"],
        "termination_reason": result["termination_reason"],
        "first_0.5_s": first_half,
        "first_1.0_s": first_full,
        "peak_btn_mm": max((r["extra"]["btn_mm"] for r in rows), default=0.0),
        "peak_btn_spring_n": max((r["extra"]["btn_spring_n"] for r in rows), default=0.0),
        "peak_hand_on_tray_n": {k: max((r["extra"]["hand_tray_n"].get(k, 0.0) for r in rows), default=0.0)
                                for k in {kk for r in rows for kk in r["extra"]["hand_tray_n"]}},
        "slide_force_vs_travel": slide_trace,
        "both_arms_ready_at_end": {s: (rows[-1][s] in ("ready", "done") if rows else False) for s in ("left", "right")},
        "report": result.get("report"),
        "worst_ik": result.get("worst_ik"),
        "median_step_ms": result.get("median_step_ms"),
    }
    (out / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
    print("READINGS " + json.dumps(readings, default=str), flush=True)
    return 0 if readings["score"] == 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
