#!/usr/bin/env python3
"""Scripted full-score reference run for the Minifix 15 cam-lock shelf task, plus offline IK dry-run check.

    scripts/rta demo <id> mech/tasks/cam_lock_shelf [--check|--no-video] [--quick] [--with-trials]

Note on ``yam_arena.demo.grip_rotation(yaw)``: because the gripper's +z approach axis points straight down (0, 0, -1),
increasing ``yaw`` rotates clockwise viewed from above (+d(cam_deg)), and decreasing ``yaw`` rotates counter-clockwise:
1. Left hand steadies the shelf's left rear wing while right hand picks the right stubby (+45 deg), seats its PZ2 tip in
   the right cam recess, turns 94 deg CCW (+45 -> -49 deg) to open the right cam, and parks in the right cradle (-45 deg).
2. Right hand steadies the right rear wing while left hand picks the left stubby (+45 deg), seats in the left cam recess,
   turns 94 deg CCW (+45 -> -49 deg) to open the left cam, parks in the left cradle (-45 deg), and holds the left wing.
3. Both hands slide the 280 mm shelf square onto the two Ø 7.0 mm bolts in the Ø 8.0 mm bores (0.50 mm play) -> 0.25.
4. Right hand steadies the seated shelf while left hand picks the left stubby (-45 deg), turns 184 deg CW in one unbroken
   sweep (-135 -> +49 deg) to pull the left seam flush and lock -> 0.50, parks (+45 deg), and steadies the left wing.
5. Right hand picks the right stubby (-45 deg), turns 184 deg CW in one unbroken sweep (-45 -> +139 deg) to pull the right
   seam flush and lock, parks (+45 deg), and both arms return to ready for the 1.0 s hold -> 1.00.
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
DRV_OPEN = 0.052 / K.MAX_JAW
DRV_GRIP = 0.011 / K.MAX_JAW  # ~32 N per finger across 24 mm flats (<= 40 N limit, clean release)
WING_OPEN = 0.052 / K.MAX_JAW
SHELF_HOLD = 0.008 / K.MAX_JAW  # ~22 N per finger clamping the 16 mm shelf wing

TIP_AT = (0.0, 0.0, 0.0)  # pure vertical push at TCP so joint4 wrist pitch stays dead vertical (0 mm tip deflection)
CAM_PRESS = (0.0, 0.0, -4.0)  # N downward axial press inside the 3.5 mm PZ2 recess (>= 1.5 N required)
CAM_X_LEAD = 0.0020  # m radial compensation away from arm base under downward load
CAM_Y_LEAD = 0.0012  # m inward y compensation away from arm base under downward load
SLIDE_PUSH = (5.5, 0.0, 0.0)  # N forward per hand during two-hand alignment slide
STEADY_PUSH = (1.2, 0.0, 0.0)  # N gentle forward hold on the seated shelf while locking
ZERO = (0.0, 0.0, 0.0)

WING_ROT_R = K.SHELF_GRIP @ np.diag([-1.0, -1.0, 1.0])
WING_ROT_L = K.SHELF_GRIP


def wing_rot(c: str) -> np.ndarray:
    return WING_ROT_L if c == "l" else WING_ROT_R


def wing_entry_waypoint(c: str) -> tuple[np.ndarray, np.ndarray]:
    sgn = 1.0 if c == "l" else -1.0
    cz = -1.0 if c == "l" else 1.0
    pos = np.array([0.160, sgn * 0.135, K.TOP + 0.145])
    rot = K.frame((0.0, 0.0, cz), (math.cos(math.radians(45.0)), 0.0, -math.sin(math.radians(45.0))))
    return pos, rot


def ready(side: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, dtype=float), K.BASES[side])


def open_cam_moves(c: str, cam_xy: tuple[float, float]) -> list:
    """Pick stubby ``c`` at -45 deg, press into ``cam_<c>``, turn 93 deg CCW (-45 -> 0 -> +48 deg) to open, and park at +45 deg."""
    import demo_kit as kit

    Move = kit.Move
    y0, y1 = K.OPEN_YAWS[c]  # -45.0, +45.0
    y_mid = 0.5 * (y0 + y1)
    y_over = y1 + 3.0  # +48.0 deg: +3 deg CCW overshoot compensates slot spring twist (~3 deg at 0.05 N m) so cam lands right at ~0 deg open
    cx = float(cam_xy[0]) + CAM_X_LEAD
    cy = float(cam_xy[1]) - math.copysign(CAM_Y_LEAD, float(cam_xy[1]))
    z_hi = K.TOP + K.Z_TRANSIT
    z_lo = K.TOP + K.Z_CAM_SEATED - 0.0003
    p_ch0, r0 = K.cradle_at(c, K.Z_TRANSIT, y0)
    p_cg0, _ = K.cradle_at(c, K.Z_CRADLE_GRASP, y0)
    p_ch1, r1 = K.cradle_at(c, K.Z_TRANSIT, y1)
    p_cg1, _ = K.cradle_at(c, K.Z_CRADLE_GRASP + 0.004, y1)
    return [
        Move(1.2, pos=p_ch0, rot=r0, grip=DRV_OPEN, push=ZERO, label=f"to {c} cradle"),
        Move(0.35, label=f"settle {c} cradle"),
        Move(0.8, pos=p_cg0, label=f"at {c} driver"),
        Move(0.4, grip=DRV_GRIP, label=f"grip {c} driver"),
        Move(0.8, pos=p_ch0, label=f"lift {c} driver"),
        Move(1.1, pos=np.array([cx, cy, z_hi]), rot=r0, label=f"over {c} cam"),
        Move(0.3, label=f"settle {c} cam"),
        Move(0.8, pos=np.array([cx, cy, z_lo]), push=CAM_PRESS, push_at=TIP_AT, label=f"seat {c} slot"),
        Move(0.7, pos=np.array([cx, cy, z_lo]), rot=K.down_yaw(y_mid), push=CAM_PRESS, label=f"open {c} 1/2"),
        Move(0.8, pos=np.array([cx, cy, z_lo]), rot=K.down_yaw(y_over), push=CAM_PRESS, label=f"open {c} 2/2"),
        Move(0.25, push=ZERO, label=f"unload {c}"),
        Move(0.7, pos=np.array([cx, cy, z_hi]), rot=r1, label=f"lift off {c} cam"),
        Move(1.2, pos=p_ch1, rot=r1, label=f"back to {c} cradle"),
        Move(0.5, label=f"align {c} cradle"),
        Move(0.95, pos=p_cg1, label=f"lower {c} driver"),
        Move(0.3, label=f"hold {c} in well"),
        Move(0.4, grip=DRV_OPEN, label=f"park {c} driver"),
        Move(0.7, pos=p_ch1, label=f"clear {c} cradle"),
    ]


def lock_cam_moves(c: str, cam_xy: tuple[float, float]) -> list:
    """Pick stubby ``c`` at +45 deg (where open parked it), turn 183 deg CW in one continuous sweep, and park at -45 deg."""
    import demo_kit as kit

    Move = kit.Move
    y_pick = 45.0
    y_park = -45.0
    y0, y1, y2, y3 = K.LOCK_YAWS[c]
    cx = float(cam_xy[0]) + CAM_X_LEAD
    cy = float(cam_xy[1]) - math.copysign(CAM_Y_LEAD, float(cam_xy[1]))
    z_hi = K.TOP + K.Z_TRANSIT
    z_lo = K.TOP + K.Z_CAM_SEATED - 0.0003
    p_ch0, r_pick = K.cradle_at(c, K.Z_TRANSIT, y_pick)
    p_cg0, _ = K.cradle_at(c, K.Z_CRADLE_GRASP, y_pick)
    p_chp, r_park = K.cradle_at(c, K.Z_TRANSIT, y_park)
    p_cgp, _ = K.cradle_at(c, K.Z_CRADLE_GRASP + 0.004, y_park)
    return [
        Move(1.2, pos=p_ch0, rot=r_pick, grip=DRV_OPEN, push=ZERO, label=f"to {c} cradle"),
        Move(0.35, label=f"settle {c} cradle"),
        Move(0.8, pos=p_cg0, label=f"at {c} driver"),
        Move(0.4, grip=DRV_GRIP, label=f"grip {c} driver"),
        Move(0.8, pos=p_ch0, label=f"lift {c} driver"),
        Move(1.2, pos=np.array([cx, cy, z_hi]), rot=K.down_yaw(y0), label=f"over {c} cam"),
        Move(0.3, label=f"settle {c} cam"),
        Move(0.8, pos=np.array([cx, cy, z_lo]), push=CAM_PRESS, push_at=TIP_AT, label=f"seat {c} lock"),
        Move(1.0, pos=np.array([cx, cy, z_lo]), rot=K.down_yaw(y1), push=CAM_PRESS, label=f"lock {c} 1/3"),
        Move(1.0, pos=np.array([cx, cy, z_lo]), rot=K.down_yaw(y2), push=CAM_PRESS, label=f"lock {c} 2/3"),
        Move(1.1, pos=np.array([cx, cy, z_lo]), rot=K.down_yaw(y3), push=CAM_PRESS, label=f"lock {c} 3/3"),
        Move(0.25, push=ZERO, label=f"unload {c}"),
        Move(0.7, pos=np.array([cx, cy, z_hi]), rot=K.down_yaw(y3 + 3.0), label=f"lift off {c} cam"),
        Move(0.6, pos=np.array([cx, cy, z_hi]), rot=r_park, label=f"unroll {c} wrist"),
        Move(1.2, pos=p_chp, rot=r_park, label=f"back to {c} cradle"),
        Move(0.55, label=f"align {c} cradle"),
        Move(1.0, pos=p_cgp, label=f"lower {c} driver"),
        Move(0.3, label=f"hold {c} in well"),
        Move(0.4, grip=DRV_OPEN, label=f"park {c} driver"),
        Move(0.7, pos=p_chp, label=f"clear {c} cradle"),
    ]


def build_phase1a() -> tuple[dict[str, list], float]:
    """Phase 1A: Left steadies l wing while Right opens cam_r and grasps r wing."""
    import demo_kit as kit

    Move = kit.Move
    wb_l0 = K.shelf_wing("l", P.START_GAP, back=K.SHELF_BACK)[0]
    wh_l0 = K.shelf_wing("l", P.START_GAP)[0]
    wb_r0 = K.shelf_wing("r", P.START_GAP, back=K.SHELF_BACK)[0]
    wh_r0 = K.shelf_wing("r", P.START_GAP)[0]
    we_l, re_l = wing_entry_waypoint("l")
    we_r, re_r = wing_entry_waypoint("r")
    x_cam0 = P.X_SHELF_START + P.CAM_X_IN_SHELF

    right_1a = open_cam_moves("r", (x_cam0, -P.CAM_Y)) + [
        Move(0.9, pos=we_r, rot=re_r, grip=WING_OPEN, push=ZERO, label="approach r wing"),
        Move(0.8, pos=wb_r0, rot=wing_rot("r"), label="behind r wing"),
        Move(0.7, pos=wh_r0, label="on r wing"),
        Move(0.4, grip=SHELF_HOLD, label="steady r wing"),
    ]
    t_1a = sum(m.seconds for m in right_1a)
    left_1a = [
        Move(0.9, pos=we_l, rot=re_l, grip=WING_OPEN, push=ZERO, label="approach l wing"),
        Move(0.9, pos=wb_l0, rot=wing_rot("l"), label="behind l wing"),
        Move(0.7, pos=wh_l0, label="on l wing"),
        Move(0.4, grip=SHELF_HOLD, label="steady l wing"),
    ]
    left_1a.append(Move(round(t_1a - sum(m.seconds for m in left_1a), 4), label="steady l wing"))
    return {"left": left_1a, "right": right_1a}, round(t_1a, 4)


def append_phase1b_and_slide(left_moves: list, right_moves: list, cam_l_xy: tuple[float, float], cur_gap: float) -> float:
    """Append Phase 1B (Right steadies while Left opens cam_l at live ``cam_l_xy``) and Phase 2 (two-hand alignment slide)."""
    import demo_kit as kit

    Move = kit.Move
    wb_l0 = K.shelf_wing("l", cur_gap, back=K.SHELF_BACK)[0]
    wh_l0 = K.shelf_wing("l", cur_gap)[0]
    we_l, re_l = wing_entry_waypoint("l")

    left_1b = (
        [
            Move(0.35, grip=WING_OPEN, push=ZERO, label="release l wing"),
            Move(0.65, pos=wb_l0, label="off l wing"),
            Move(0.8, pos=we_l, rot=re_l, label="lift from l wing"),
        ]
        + open_cam_moves("l", cam_l_xy)
        + [
            Move(0.9, pos=we_l, rot=re_l, grip=WING_OPEN, push=ZERO, label="approach l wing"),
            Move(0.8, pos=wb_l0, rot=wing_rot("l"), label="behind l wing"),
            Move(0.7, pos=wh_l0, label="on l wing"),
            Move(0.4, grip=SHELF_HOLD, label="hold l wing"),
        ]
    )
    t_1b = sum(m.seconds for m in left_1b)
    right_1b = [Move(round(t_1b, 4), label="steady r wing")]

    wh_l1 = K.shelf_wing("l", 0.0002)[0]
    wh_r1 = K.shelf_wing("r", 0.0002)[0]
    slide_l = [
        Move(3.0, pos=wh_l1, push=SLIDE_PUSH, label="align & slide"),
        Move(0.5, push=STEADY_PUSH, label="seated"),
    ]
    slide_r = [
        Move(3.0, pos=wh_r1, push=SLIDE_PUSH, label="align & slide"),
        Move(0.5, push=STEADY_PUSH, label="seated"),
    ]
    left_moves.extend(left_1b + slide_l)
    right_moves.extend(right_1b + slide_r)
    return round(sum(m.seconds for m in left_moves), 4)


def append_phase3a_lock_left(left_moves: list, right_moves: list, cam_l_xy: tuple[float, float], seated_gap: float) -> float:
    """Append Phase 3A (Right steadies r wing while Left locks cam_l 180 deg CW, parks driver_l, and re-grasps l wing)."""
    import demo_kit as kit

    Move = kit.Move
    wb_l = K.shelf_wing("l", seated_gap, back=K.SHELF_BACK)[0]
    wh_l = K.shelf_wing("l", max(0.0, seated_gap))[0]
    we_l, re_l = wing_entry_waypoint("l")

    p3a_l = (
        [
            Move(0.35, grip=WING_OPEN, push=ZERO, label="release l wing"),
            Move(0.65, pos=wb_l, label="off l wing"),
            Move(0.8, pos=we_l, rot=re_l, label="lift from l wing"),
        ]
        + lock_cam_moves("l", cam_l_xy)
        + [
            Move(0.9, pos=we_l, rot=re_l, grip=WING_OPEN, push=ZERO, label="approach l wing"),
            Move(0.8, pos=wb_l, rot=wing_rot("l"), label="behind l wing"),
            Move(0.7, pos=wh_l, label="on l wing"),
            Move(0.4, grip=SHELF_HOLD, push=STEADY_PUSH, label="steady l wing"),
        ]
    )
    t_3a = sum(m.seconds for m in p3a_l)
    p3a_r = [Move(round(t_3a, 4), push=STEADY_PUSH, label="steady r wing")]
    left_moves.extend(p3a_l)
    right_moves.extend(p3a_r)
    return round(sum(m.seconds for m in left_moves), 4)


def append_phase3b_lock_right(left_moves: list, right_moves: list, cam_r_xy: tuple[float, float], seated_gap: float) -> None:
    """Append Phase 3B (Left steadies l wing while Right locks cam_r at live ``cam_r_xy``, parks driver_r, and both go ready)."""
    import demo_kit as kit

    Move = kit.Move
    wb_r = K.shelf_wing("r", max(0.0, seated_gap), back=K.SHELF_BACK)[0]
    we_l, re_l = wing_entry_waypoint("l")
    we_r, re_r = wing_entry_waypoint("r")

    r_lock = lock_cam_moves("r", cam_r_xy)
    p3b_r = (
        [
            Move(0.35, grip=WING_OPEN, push=ZERO, label="release r wing"),
            Move(0.65, pos=wb_r, label="off r wing"),
            Move(0.8, pos=we_r, rot=re_r, label="lift from r wing"),
        ]
        + r_lock
        + [Move(1.8, pos=ready("right")[0], rot=ready("right")[1], grip=OPEN, push=ZERO, label="ready")]
    )
    t_until_r_locked = 1.8 + sum(m.seconds for m in r_lock[:14])
    wb_l_end = K.shelf_wing("l", 0.0, back=K.SHELF_BACK)[0]
    p3b_l = [
        Move(round(t_until_r_locked, 4), push=STEADY_PUSH, label="steady l wing"),
        Move(0.35, grip=WING_OPEN, push=ZERO, label="release l wing"),
        Move(0.65, pos=wb_l_end, label="off l wing"),
        Move(0.8, pos=we_l, rot=re_l, label="lift from l wing"),
        Move(1.8, pos=ready("left")[0], rot=ready("left")[1], grip=OPEN, push=ZERO, label="ready"),
    ]
    left_moves.extend(p3b_l)
    right_moves.extend(p3b_r)


def full_nominal_plan() -> dict[str, list]:
    moves, _ = build_phase1a()
    x_cam0 = P.X_SHELF_START + P.CAM_X_IN_SHELF
    append_phase1b_and_slide(moves["left"], moves["right"], (x_cam0, P.CAM_Y), P.START_GAP)
    x_nom = P.X_SHELF_FLUSH - 0.0005 + P.CAM_X_IN_SHELF
    append_phase3a_lock_left(moves["left"], moves["right"], (x_nom, P.CAM_Y), 0.0005)
    append_phase3b_lock_right(moves["left"], moves["right"], (x_nom, -P.CAM_Y), 0.0002)
    return moves


def save_end_picture(frame, folder: Path) -> None:
    import cv2
    import demo_kit as kit

    cv2.imwrite(str(folder / "end_top.png"), cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(folder / "end_top_policy.png"), cv2.cvtColor(kit.policy_picture(np.asarray(frame)), cv2.COLOR_RGB2BGR))


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    flags = set(sys.argv[2:])
    if "--quick" in flags:
        from yam_arena import station

        dry = kit.dry_run(full_nominal_plan(), station.BASES)
        print("QUICK " + json.dumps(dry), flush=True)
        return 0 if all(v["worst_ik_mm"] <= 0.5 and v["smallest_margin_deg"] >= 6.0 for v in dry.values()) else 1

    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    moves, t_1a_done = build_phase1a()
    stage = 0
    t_slide_done = 1e9
    t_3a_done = 1e9
    latest: dict = {}
    saved_start = False

    def director(t, arms, pictures) -> None:
        nonlocal stage, t_slide_done, t_3a_done, saved_start
        if pictures is not None:
            latest.update(pictures)
            if not saved_start:
                import cv2

                saved_start = True
                folder.mkdir(parents=True, exist_ok=True)
                for cam, stem in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
                    bgr = cv2.cvtColor(np.asarray(pictures[cam]), cv2.COLOR_RGB2BGR)
                    cv2.imwrite(str(folder / f"{stem}.png"), bgr)
                    cv2.imwrite(str(folder / f"{stem}_policy.png"), cv2.resize(bgr, (320, 176), interpolation=cv2.INTER_AREA))
        if stage == 0 and t >= t_1a_done - 0.04:
            stage = 1
            p_cl = m.mk.pose("cam_l")[0]
            gaps = m.seam_gaps()
            cur_gap = max(0.0, 0.5 * (gaps["l"] + gaps["r"]))
            t_slide_done = append_phase1b_and_slide(arms["left"].moves, arms["right"].moves, (p_cl[0], p_cl[1]), cur_gap)
        elif stage == 1 and t >= t_slide_done - 0.04:
            stage = 2
            p_cl = m.mk.pose("cam_l")[0]
            gaps = m.seam_gaps()
            mean_gap = max(0.0, 0.5 * (gaps["l"] + gaps["r"]))
            t_3a_done = append_phase3a_lock_left(arms["left"].moves, arms["right"].moves, (p_cl[0], p_cl[1]), mean_gap)
        elif stage == 2 and t >= t_3a_done - 0.04:
            stage = 3
            p_cr = m.mk.pose("cam_r")[0]
            gaps = m.seam_gaps()
            mean_gap = max(0.0, 0.5 * (gaps["l"] + gaps["r"]))
            append_phase3b_lock_right(arms["left"].moves, arms["right"].moves, (p_cr[0], p_cr[1]), mean_gap)

    def extra() -> dict:
        gaps = m.seam_gaps()
        return {
            "gaps_mm": {c: round(1000.0 * gaps[c], 2) for c in m.CONNS},
            "cam_deg": {c: round(m.cam_deg(c), 1) for c in m.CONNS},
            "cam_clamp_n": {c: round(m.cam_clamp_force(c), 2) for c in m.CONNS},
            "drv_cam_n": {c: round(m.mk.contact_force(f"driver_{c}", f"cam_{c}"), 2) for c in m.CONNS},
            "tau_z_nm": {c: round(-float(m.STATE.get("couplings", {}).get(f"cam_{c}", (0, 0, 0))[2]), 3) for c in m.CONNS},
            "hands_clear_mm": round(1000.0 * m.hands_clear(), 1),
            "hold": m.STATE.get("hold", 0),
        }

    folder = out / "full"
    record = not bool(flags & {"--no-video", "--check"})
    result = kit.run(
        emb,
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
    first_q = next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.25), None)
    first_h = next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.50), None)
    first_f = next((r["t"] for r in rows if r["info"].get("score", 0.0) == 1.00), None)
    torque_trace = [
        {"t": r["t"], "cam_deg": r["extra"]["cam_deg"], "drv_cam_n": r["extra"]["drv_cam_n"], "tau_z_nm": r["extra"]["tau_z_nm"], "gaps_mm": r["extra"]["gaps_mm"], "clamp_n": r["extra"]["cam_clamp_n"]}
        for r in rows[::8]
        if max(abs(v) for v in r["extra"]["tau_z_nm"].values()) > 0.01 or max(r["extra"]["cam_clamp_n"].values()) > 1.0
    ]
    readings = {
        "score_at_end": rows[-1]["info"].get("score") if rows else 0.0,
        "steps": len(rows),
        "seconds": rows[-1]["t"] if rows else 0.0,
        "first_0.25_s": first_q,
        "first_0.50_s": first_h,
        "first_1.00_s": first_f,
        "end_state": rows[-1]["extra"] if rows else None,
        "report": result.get("report"),
        "worst_ik": result.get("worst_ik"),
        "median_step_ms": result.get("median_step_ms"),
        "torque_and_draw_samples": torque_trace[:24],
    }
    (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
    print("READINGS " + json.dumps(readings, default=str), flush=True)
    trials_ok = True
    if "--with-trials" in flags:
        import trials

        trials_ok, _ = trials.run_suite(emb, m, out / "trials")
    return 0 if (readings["score_at_end"] == 1.0 and trials_ok) else 1


if __name__ == "__main__":
    raise SystemExit(main())
