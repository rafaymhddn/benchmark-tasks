#!/usr/bin/env python3
"""The scripted full-score reference run of the CryoCane sleeving task, and an offline IK check of its plan.

    scripts/rta demo <id> mech/tasks/cryocane_sleeve [--no-video] [--quick]

Pure arm joint actions throughout (zero write_pose, zero set_joint, zero welds):
1. Left hand grasps the 24 mm bent top ID tab of the loaded CryoCane on the prep mat, lifts it level (roll < 2 deg so
   neither loose 12.5 mm Nunc 377267 vial spills from its open C-bay), and yaws 180 deg horizontally (-90 -> 0 -> +90
   deg) so the cane's straight bottom foot points -y toward the sleeve.
2. Right hand grasps the 273 mm opaque frosted-ivory CryoSleeve near its -y tail and lifts its +y mouth in front of the
   cane's straight bottom foot.
3. At pre-entry and across the 3-stage 265 mm coaxial pass (22 mm foot entry -> 145 mm mid-cane -> 264 mm seated up to
   the 268 mm bent top tab), the director reads the cane's and sleeve's poses (allowed for scripted demos per Guide)
   and steers both arms coaxially along the cane's shaft axis against the compliant side-wipers' ~5 N friction.
4. Right hand opens to 94 mm, lifts straight up clear of the sleeve, and returns to ready; left hand carries the
   enclosed cane over the stainless transfer tray, lays it flat inside, opens, lifts clear, and returns to ready (1.0).
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
FINGER_STIFFNESS = 5000.0  # N/m per finger drive (yam_arena constants)
CANE_PINCH_N = 30.0  # N per finger (<= 40 N hard limit)
SLEEVE_PINCH_N = 34.0  # N per finger (<= 40 N hard limit)
CANE_GRIP = max(0.0, (P.TAB_W - 2.0 * CANE_PINCH_N / FINGER_STIFFNESS) / K.MAX_JAW)
SLEEVE_GRIP = max(0.0, (2.0 * P.SLEEVE_R_OUT - 2.0 * SLEEVE_PINCH_N / FINGER_STIFFNESS) / K.MAX_JAW)


def ready(side: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, dtype=float), K.BASES[side])


def _align_rot(u_from: np.ndarray, u_to: np.ndarray) -> np.ndarray:
    """Smallest rotation matrix mapping unit vector ``u_from`` onto ``u_to``."""
    a = u_from / np.linalg.norm(u_from)
    b = u_to / np.linalg.norm(u_to)
    cross = np.cross(a, b)
    s = float(np.linalg.norm(cross))
    if s < 1e-8:
        return np.eye(3)
    c = max(-1.0, min(1.0, float(np.dot(a, b))))
    k_mat = np.array([[0.0, -cross[2], cross[1]], [cross[2], 0.0, -cross[0]], [-cross[1], cross[0], 0.0]]) / s
    ang = math.atan2(s, c)
    return np.eye(3) + math.sin(ang) * k_mat + (1.0 - math.cos(ang)) * (k_mat @ k_mat)


def initial_plan() -> dict[str, list]:
    """Phase 1 (0 .. 11.0 s): pick cane, lift level and yaw 180 deg; pick sleeve and lift near coaxial pre-entry."""
    import demo_kit as kit

    Move = kit.Move
    p = K.poses()
    left = [
        Move(2.2, pos=p["L: above cane tab"][1], rot=K.ROT_START_L, grip=OPEN, label="above cane tab"),
        Move(1.2, pos=p["L: on cane tab"][1], label="on cane tab"),
        Move(0.8, grip=CANE_GRIP, label="pinch cane tab"),
        Move(1.8, pos=p["L: lift cane level"][1], label="lift level"),
        Move(2.2, pos=p["L: yaw 90 deg"][1], rot=K.ROT_MID_L, label="yaw 90 deg"),
        Move(2.2, pos=p["L: yaw 180 deg ready"][1], rot=K.ROT_COAX_L, label="yaw 180 deg"),
        Move(0.6, label="settle coaxial"),
    ]
    right_pick = [
        Move(2.0, label="wait cane pick"),
        Move(2.4, pos=p["R: above sleeve tail"][1], rot=K.ROT_SLV_R, grip=OPEN, label="above sleeve"),
        Move(1.2, pos=p["R: on sleeve tail"][1], label="on sleeve"),
        Move(0.8, grip=SLEEVE_GRIP, label="pinch sleeve"),
        Move(2.5, pos=p["R: lift sleeve coaxial"][1], rot=K.ROT_COAX_R, label="lift coaxial"),
    ]
    dwell = (sum(mv.seconds for mv in left) - sum(mv.seconds for mv in right_pick))
    right = right_pick + [Move(max(0.2, dwell), label="settle coaxial")]
    return {"left": left, "right": right}


def full_nominal_plan() -> dict[str, list]:
    """Full nominal open-loop trajectory used by ``--quick`` IK verification."""
    import demo_kit as kit

    Move = kit.Move
    p = K.poses()
    base = initial_plan()
    base["left"] += [
        Move(1.4, label="align mouth"),
        Move(1.6, label="enter foot"),
        Move(2.6, pos=p["L: coaxial mid"][1], label="thread mid"),
        Move(2.8, pos=p["L: coaxial seated"][1], label="thread seated"),
        Move(0.6, label="seated hold"),
        Move(2.2, label="wait right release"),
        Move(2.4, pos=p["L: over tray"][1], rot=K.ROT_END_L, label="carry over tray"),
        Move(2.0, pos=p["L: lay in tray"][1], label="lay in tray"),
        Move(0.8, grip=OPEN, label="release in tray"),
        Move(1.2, pos=p["L: over tray"][1], label="clear tray"),
        Move(2.6, pos=ready("left")[0], rot=ready("left")[1], label="ready"),
    ]
    base["right"] += [
        Move(1.4, pos=p["R: lift sleeve coaxial"][1], label="align mouth"),
        Move(1.6, pos=p["R: lift sleeve coaxial"][1] + np.array([0.0, 0.026, 0.0]), label="enter foot"),
        Move(2.6, pos=p["R: coaxial mid"][1], label="thread mid"),
        Move(2.8, pos=p["R: coaxial seated"][1], label="thread seated"),
        Move(0.6, label="seated hold"),
        Move(0.9, grip=OPEN, label="release sleeve"),
        Move(1.3, pos=p["R: coaxial seated"][1] + np.array([0.0, 0.0, 0.055]), rot=K.ROT_SLV_R, label="lift off sleeve"),
        Move(2.8, pos=ready("right")[0], rot=ready("right")[1], label="ready"),
    ]
    return base


def save_end_picture(frame, folder: Path) -> None:
    import cv2
    import demo_kit as kit

    cv2.imwrite(str(folder / "end_top.png"), cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(folder / "end_top_policy.png"), cv2.cvtColor(kit.policy_picture(np.asarray(frame)), cv2.COLOR_RGB2BGR))


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    if "--quick" in sys.argv[2:]:
        from yam_arena import station

        dry = kit.dry_run(full_nominal_plan(), station.BASES)
        print("QUICK " + json.dumps(dry), flush=True)
        return 0
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    moves = initial_plan()
    p_poses = K.poses()
    Move = kit.Move
    stage_state = {"phase": 0}

    def _right_coaxial_goal(t_now: float, arms, s_target: float, left_shift: np.ndarray | None = None):
        """Differential closed-loop target for right arm: shifts commanded TCP by exact measured (mouth_goal - mouth_now)
        using actual gripper pose so arm gravity sag cancels to 0.0 mm."""
        p_c, r_c = m.cane_pose()
        u_c = r_c[:, 1].copy()
        u_c[2] = float(np.clip(u_c[2], -0.05, 0.05))
        u_c = u_c / np.linalg.norm(u_c)
        if left_shift is not None:
            p_c = p_c + left_shift
        p_s, r_s = m.mk.pose("sleeve")
        u_s = r_s[:, 1]
        tcp_cmd, rot_cmd = arms["right"].state(t_now)[:2]
        tcp_actual = m.mk.gripper("right")[0]
        mouth_now = p_s + 0.5 * P.SLEEVE_L * u_s
        lever_now = mouth_now - tcp_actual
        r_fix = _align_rot(u_s, u_c)
        rot_goal = r_fix @ rot_cmd
        mouth_after_turn = tcp_actual + r_fix @ lever_now
        mouth_goal = p_c + s_target * u_c
        pos_goal = tcp_cmd + (mouth_goal - mouth_after_turn)
        return pos_goal, rot_goal, u_c

    def director(t: float, arms, pictures) -> None:
        if pictures is not None:
            latest.update(pictures)
        if not m.mk.MECH.poses:
            return
        max_end = max(arms["left"].seconds, arms["right"].seconds)
        if t < max_end - 0.025:
            return
        ph = stage_state["phase"]
        if ph == 0:
            # Stage 1: align sleeve orientation + mouth 6 mm before straight foot, then dwell 0.5 s to settle to <1 mm
            r_pos, r_rot, _ = _right_coaxial_goal(t, arms, -0.006)
            arms["left"].moves += [Move(1.4, label="align mouth"), Move(0.5, label="settle mouth")]
            arms["right"].moves += [Move(1.4, pos=r_pos, rot=r_rot, label="align mouth"), Move(0.5, label="settle mouth")]
            stage_state["phase"] = 1
        elif ph == 1:
            # Stage 2: pure translational trim (keep settled 0.1 deg rotation fixed!) + smooth straight coaxial stroke!
            p_c, r_c = m.cane_pose()
            u_c = r_c[:, 1].copy()
            u_c[2] = float(np.clip(u_c[2], -0.04, 0.04))
            u_c = u_c / np.linalg.norm(u_c)
            p_s, r_s = m.mk.pose("sleeve")
            mouth_now = p_s + 0.5 * P.SLEEVE_L * r_s[:, 1]
            tcp_l = arms["left"].state(t)[0]
            tcp_r = arms["right"].state(t)[0]
            # Exact translational offset from current sleeve mouth to cane foot tip p_c:
            to_foot = p_c - mouth_now
            # Step 2a: telescope +18 mm over bullet foot (1.5 s), keeping rotation fixed
            arms["left"].moves.append(Move(1.5, label="enter foot"))
            arms["right"].moves.append(Move(1.5, pos=tcp_r + to_foot + 0.018 * u_c, label="enter foot"))
            # Step 2b: bimanual 265 mm coaxial stroke along u_c (+8 mm preload overcomes 5.0 N wipe lag to seat at 265 mm!)
            d_left = -0.055 * u_c
            r_seat = tcp_r + to_foot + d_left + 0.273 * u_c
            arms["left"].moves += [
                Move(5.4, pos=tcp_l + d_left, push=(-2.5 * u_c).tolist(), label="thread 265 mm"),
                Move(0.7, push=(0.0, 0.0, 0.0), label="seated hold"),
            ]
            arms["right"].moves += [
                Move(5.4, pos=r_seat, push=(3.2 * u_c).tolist(), label="thread 265 mm"),
                Move(0.7, push=(0.0, 0.0, 0.0), label="seated hold"),
            ]
            stage_state["phase"] = 4
        elif ph == 4:
            # Stage 5: open right hand to 94 mm, lift straight up clear of sleeve, carry enclosed cane into tray (1.0)
            tcp_r = arms["right"].state(t)[0]
            arms["right"].moves += [
                Move(0.9, grip=OPEN, label="release sleeve"),
                Move(1.3, pos=tcp_r + np.array([0.0, 0.0, 0.060]), rot=K.ROT_SLV_R, label="lift off sleeve"),
                Move(2.6, pos=ready("right")[0], rot=ready("right")[1], label="ready"),
            ]
            arms["left"].moves += [
                Move(2.2, label="wait right release"),
                Move(2.4, pos=p_poses["L: over tray"][1], rot=K.ROT_END_L, label="carry over tray"),
                Move(2.0, pos=p_poses["L: lay in tray"][1], label="lay in tray"),
                Move(0.8, grip=OPEN, label="release in tray"),
                Move(1.2, pos=p_poses["L: over tray"][1], label="clear tray"),
                Move(2.6, pos=ready("left")[0], rot=ready("left")[1], label="ready"),
            ]
            stage_state["phase"] = 5

    def extra() -> dict:
        ok, y_lo, y_hi, rad, ang = m.sleeve_span_on_cane()
        fw = float(m.STATE.get("slide_force_n", 0.0))
        return {
            "mated": ok,
            "slide_mm": round(1000.0 * (max(0.0, y_hi) if ok else 0.0), 2),
            "y_lo_mm": round(1000.0 * y_lo, 2),
            "rad_mm": round(1000.0 * rad, 2),
            "ang_deg": round(ang, 2),
            "gap_tab_mm": None if not ok else round(1000.0 * max(0.0, P.TAB_SHOULDER_Y - y_hi), 2),
            "wipe_f_n": round(fw, 2),
            "bend_nm": round(float(m.STATE.get("bend_nm", 0.0)), 3),
            "roll_deg": round(m.cane_roll_deg(), 2),
            "pitch_deg": round(m.cane_pitch_deg(), 2),
            "hands_clear_mm": round(1000.0 * m.hands_clear(), 1),
            "hold": m.STATE.get("hold", 0),
        }

    folder = out / "full"
    record = "--no-video" not in sys.argv[2:]
    latest: dict = {}
    result = kit.run(
        emb,
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
    labels = {side: [r[side] for r in rows] for side in ("left", "right")}
    first_q = next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.25), None)
    first_h = next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.5), None)
    first_f = next((r["t"] for r in rows if r["info"].get("score", 0.0) == 1.0), None)
    wipe_trace = [
        {
            "t": r["t"],
            "slide_mm": r["extra"]["slide_mm"],
            "wipe_f_n": r["extra"]["wipe_f_n"],
            "bend_nm": r["extra"]["bend_nm"],
            "arm_f_y_n": {"left": r["force"]["left"][1], "right": r["force"]["right"][1]},
        }
        for r in rows
        if r["right"] in ("enter foot", "thread 265 mm", "seated hold")
    ][::8]
    pinch_l = max(
        0.5 * FINGER_STIFFNESS * max(0.0, (r["grip"]["left"][3] - CANE_GRIP) * K.MAX_JAW)
        for r in rows
        if "left" in r["grip"] and r["left"] in ("lift level", "yaw 90 deg", "yaw 180 deg", "thread 265 mm")
    )
    pinch_r = max(
        0.5 * FINGER_STIFFNESS * max(0.0, (r["grip"]["right"][3] - SLEEVE_GRIP) * K.MAX_JAW)
        for r in rows
        if "right" in r["grip"] and r["right"] in ("lift coaxial", "enter foot", "thread 265 mm")
    )
    readings = {
        "score at end": rows[-1]["info"].get("score"),
        "seconds": rows[-1]["t"],
        "steps": len(rows),
        "first 0.25 / 0.5 / 1.0 at (s)": [first_q, first_h, first_f],
        "worst abs cane roll / pitch (deg)": [
            round(max(abs(r["extra"]["roll_deg"]) for r in rows), 2),
            round(max(abs(r["extra"]["pitch_deg"]) for r in rows), 2),
        ],
        "max slide (mm)": max(r["extra"]["slide_mm"] for r in rows),
        "final gap to top tab (mm)": rows[-1]["extra"]["gap_tab_mm"],
        "max wipe resistance (N)": max(r["extra"]["wipe_f_n"] for r in rows),
        "peak per-finger pinch on cane / sleeve (N)": [round(pinch_l, 1), round(pinch_r, 1)],
        "force vs 265 mm travel trace (every 8th step)": wipe_trace,
        "both arms ready at end": {side: labels[side][-1] == "done" for side in ("left", "right")},
        "hands clear at end (mm)": rows[-1]["extra"]["hands_clear_mm"],
        "report": result.get("report"),
        "worst IK": result.get("worst_ik"),
        "median step (ms)": result.get("median_step_ms"),
    }
    (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
    print("READINGS " + json.dumps(readings, default=str), flush=True)
    return 0 if readings["score at end"] == 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
