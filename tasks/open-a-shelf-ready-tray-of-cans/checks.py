#!/usr/bin/env python3
"""Gate G1 checks for the shelf-ready tray task:
1. Offline (no simulator): presets, IK accuracy (<= 0.5 mm) and joint margins (>= 5 deg) across all bimanual
   brace, corner-peel, carry, and flat-placement poses; 3D link-hull clearances (hand-to-hand TCP >= 200 mm and
   positive hull gap, fingertip clearance above all cans and bench); pad normal & tangential grip force on
   PANEL_GRIP_T; and top-camera pixel projection at 640x360 and 320x176.
2. Frame-0 + idle settle in simulator (``--frame0 <out_dir>``): renders ``start_top.png``, ``start_top_policy.png``,
   ``start_left_wrist.png``, and ``start_right_wrist.png``, and verifies 2.0 s idle rest stability.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np
from yam_arena import analysis, constants as C, ik, station
from yam_arena.constants import READY
from yam_arena.task_file import TABLE_TOP

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import parts as P  # noqa: E402

TOP = TABLE_TOP["station"]
BASES = {side: np.array(station.BASES[side], dtype=float) for side in ("left", "right")}
HAND_LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")
MAX_JAW = 2.0 * abs(float(C.FINGER_OPEN))  # 0.0939 m
FINGER_STIFFNESS = float(C.GRIPPER["stiffness"])  # 5000.0 N/m per finger

# Canonical bimanual grasp geometry (used identically by demo.py)
BRACE_X = P.TRAY_XY[0] + 0.030  # 0.310 m: middle-rear of left side wall, strong anti-yaw brace
BRACE_Y = P.TRAY_XY[1] + 0.5 * P.TRAY_W - 0.5 * P.PANEL_GRIP_T  # +0.1118 m: center of 6.4 mm 2-ply left brace rim
GRIP_TOP_DZ = P.TRAY_H - 0.0015  # 0.1285 m above tray underside: 14.3 mm pad overlap on header, +2.5..+3.5 mm above cans
PEEL_CORNER_Y = -0.075  # m: right upper corner of front_panel (36 mm inside -111 mm right seam)
PEEL_GRIP_X = P.panel_built_pos(TOP)[0] - 0.5 * (P.PANEL_GRIP_T - P.BOARD_T)  # exact center of 6.4 mm double-ply rim
DROP_YAW_DEG = -65.0  # deg yaw when laying detached panel flat on right-bench drop zone


def frame(closing, approach) -> np.ndarray:
    """TCP rotation matrix whose columns are (closing axis, pad width axis, approach axis)."""
    a = np.asarray(approach, dtype=float)
    a = a / np.linalg.norm(a)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c = c / np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def rz(angle_rad: float) -> np.ndarray:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def left_brace_pose(lift_m: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Left gripper pose bracing tray_base's left wall top rim (closing = -y gives 37 deg joint margin)."""
    pos = np.array([BRACE_X, BRACE_Y, TOP + P.OVER + GRIP_TOP_DZ + lift_m], dtype=float)
    # Slight 12 deg forward lean from vertical matches the arm's natural elbow-up posture atboth low & lifted heights
    rot = frame(closing=(0.0, -1.0, 0.0), approach=(math.sin(math.radians(12.0)), 0.0, -math.cos(math.radians(12.0))))
    return pos, rot


def peel_arc_pose(
    pitch_deg: float, skew_deg: float = 8.0, pull_extra_m: float = 0.0, grip_y: float = PEEL_CORNER_Y
) -> tuple[np.ndarray, np.ndarray]:
    """World TCP pose for the right gripper holding the rim at ``grip_y`` (default: top-right corner) at ``pitch_deg``."""
    anchor = np.array(P.hinge_anchor_world(TOP), dtype=float)
    th = math.radians(pitch_deg)
    # Rotation about hinge axis (0, -1, 0): +z swings into (-sin(th), 0, +cos(th)), i.e. outward (-x) and downward (-z)
    r_pitch = np.array(
        [
            [math.cos(th), 0.0, -math.sin(th)],
            [0.0, 1.0, 0.0],
            [math.sin(th), 0.0, math.cos(th)],
        ],
        dtype=float,
    )
    ax = np.asarray(P.SKEW_AXIS, dtype=float)
    ax = ax / np.linalg.norm(ax)
    ang = math.radians(skew_deg)
    k = np.array([[0.0, -ax[2], ax[1]], [ax[2], 0.0, -ax[0]], [-ax[1], ax[0], 0.0]])
    r_skew = np.eye(3) + math.sin(ang) * k + (1.0 - math.cos(ang)) * (k @ k)
    r_panel = r_pitch @ r_skew

    local_grip = np.array([PEEL_GRIP_X, grip_y, TOP + P.OVER + GRIP_TOP_DZ]) - anchor
    outward = r_panel @ np.array([-1.0, 0.0, 0.0])
    pos = anchor + r_panel @ local_grip + pull_extra_m * outward

    # Hand follows BOTH panel pitch (up to 52 deg) AND corner skew so the pads twist the corner open rather than clamping skew to 0!
    hth = math.radians(min(pitch_deg, 52.0))
    r_pitch_h = np.array(
        [
            [math.cos(hth), 0.0, -math.sin(hth)],
            [0.0, 1.0, 0.0],
            [math.sin(hth), 0.0, math.cos(hth)],
        ],
        dtype=float,
    )
    r_hand = r_pitch_h @ r_skew
    # closing = +r_hand @ [1, 0, 0] puts wrist joint6 at -41..+6 deg (75+ deg clear of +-120 deg limits)
    rot = frame(closing=r_hand @ np.array([1.0, 0.0, 0.0]), approach=r_hand @ np.array([0.0, 0.0, -1.0]))
    return pos, rot


def right_carry_pose() -> tuple[np.ndarray, np.ndarray]:
    """High clear waypoint carrying detached panel completely right of tray_base (y = -0.345) before moving +x."""
    hth = math.radians(48.0)
    r_fold = np.array(
        [
            [math.cos(hth), 0.0, -math.sin(hth)],
            [0.0, 1.0, 0.0],
            [math.sin(hth), 0.0, math.cos(hth)],
        ],
        dtype=float,
    )
    rot = frame(closing=r_fold @ np.array([1.0, 0.0, 0.0]), approach=r_fold @ np.array([0.0, 0.0, -1.0]))
    pos = np.array([0.240, -0.345, TOP + 0.175], dtype=float)
    return pos, rot


def right_drop_pose(lift_m: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Zero-yaw right-bench drop pose at (0.285, -0.370): centers panel in DROP_ZONE_XY >65 mm clear of tray_base."""
    hth = math.radians(48.0)
    r_fold = np.array(
        [
            [math.cos(hth), 0.0, -math.sin(hth)],
            [0.0, 1.0, 0.0],
            [math.sin(hth), 0.0, math.cos(hth)],
        ],
        dtype=float,
    )
    rot = frame(closing=r_fold @ np.array([1.0, 0.0, 0.0]), approach=r_fold @ np.array([0.0, 0.0, -1.0]))
    pos = np.array([0.285, -0.370, TOP + 0.036 + lift_m], dtype=float)
    return pos, rot


def solve_ik(side: str, pos, rot, start=None):
    """Best IK solution: returns (q, pos_err_mm, rot_err_deg, min_joint_margin_deg)."""
    best = None
    seeds = ([np.asarray(start, dtype=float)] if start is not None else []) + [
        np.array(s, dtype=float)
        for s in (
            READY,
            [0.0, 1.2, 1.2, -0.5, 0.0, 0.0],
            [0.0, 1.5, 1.5, -0.4, 0.0, 0.0],
            [0.6, 1.6, 1.2, -1.0, 0.5, 0.0],
            [-0.6, 1.6, 1.2, -1.0, 0.0, -0.6],
        )
    ]
    quat = ik.matrix_to_quat_xyzw(rot)
    for q0 in seeds:
        q, pe, re = ik.solve(np.asarray(pos, dtype=float), quat, q0, base_pos=BASES[side], iters=200)
        margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
        ok = pe < 5e-4 and re < 0.01
        score_key = (ok, margin if ok else -(pe * 1000.0 + math.degrees(re)))
        if best is None or score_key > best[0]:
            best = (score_key, q, float(pe * 1000.0), float(math.degrees(re)), margin)
    assert best is not None
    return best[1], best[2], best[3], best[4]


def link_poses(side: str, q: np.ndarray, opening: float):
    half = -0.5 * max(0.0, min(MAX_JAW, opening))
    joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, q)}, **dict.fromkeys(C.FINGER_JOINTS, half)}
    return analysis.fk_link_poses_world(joints, tuple(BASES[side]))


def hand_hulls(side: str, q: np.ndarray, opening: float):
    poses = link_poses(side, q, opening)
    out = []
    for link in HAND_LINKS:
        for idx, h in enumerate(analysis.link_hulls().get(link, [])):
            r, p = poses[link][:3, :3], poses[link][:3, 3]
            n_w = h.planes[:, :3] @ r.T
            out.append(
                (
                    link,
                    idx,
                    analysis.Hull(
                        vertices=h.vertices @ r.T + p,
                        planes=np.c_[n_w, h.planes[:, 3] + n_w @ p],
                    ),
                )
            )
    return out


def hulls_gap_m(ha, hb) -> float:
    best = math.inf
    for _, _, a in ha:
        for _, _, b in hb:
            lo_a, hi_a = a.vertices.min(0), a.vertices.max(0)
            lo_b, hi_b = b.vertices.min(0), b.vertices.max(0)
            if float(np.maximum(lo_b - hi_a, lo_a - hi_b).max()) > 0.15:
                continue
            best = min(best, float(analysis._signed(a.vertices, b).min()), float(analysis._signed(b.vertices, a).min()))
    return best if math.isfinite(best) else 0.15


def lowest_z(hs) -> float:
    return float(min(h.vertices[:, 2].min() for _, _, h in hs))


def project_top_cam(point_w, width: int = 640, height: int = 360) -> tuple[float, float, float]:
    """Project a 3D world point into the station top camera image (u, v, depth_m)."""
    cam_pos = np.array(C.TOP_CAM_POS, dtype=float)
    fwd = np.array(C.TOP_CAM_FORWARD, dtype=float)
    fwd = fwd / np.linalg.norm(fwd)
    right = np.array([0.0, -1.0, 0.0], dtype=float)
    up = np.cross(right, fwd)
    rel = np.asarray(point_w, dtype=float) - cam_pos
    depth = float(np.dot(rel, fwd))
    fx = width * float(C.D435_FOCAL) / 20.955
    fy = fx
    u = 0.5 * width + fx * float(np.dot(rel, right)) / depth
    v = 0.5 * height - fy * float(np.dot(rel, up)) / depth
    return round(u, 1), round(v, 1), round(depth, 3)


def run_offline_checks() -> dict:
    can_top_z = TOP + P.OVER + P.BOARD_T + P.CAN_H

    brace_over_pos, brace_rot = left_brace_pose(lift_m=0.065)
    brace_pos, _ = left_brace_pose(lift_m=0.0)

    peel0_pos, peel0_rot = peel_arc_pose(0.0, skew_deg=0.0)
    peel_over_pos = peel0_pos + np.array([0.0, 0.0, 0.065])
    peel15_pos, peel15_rot = peel_arc_pose(15.0, skew_deg=11.0, pull_extra_m=0.0)
    peel30_pos, peel30_rot = peel_arc_pose(30.0, skew_deg=11.0, pull_extra_m=0.0)
    peel45_pos, peel45_rot = peel_arc_pose(45.0, skew_deg=11.0, pull_extra_m=0.0)
    peel58_pos, peel58_rot = peel_arc_pose(58.0, skew_deg=11.0, pull_extra_m=0.0)
    carry_pos, carry_rot = right_carry_pose()
    drop_pos, drop_rot = right_drop_pose(lift_m=0.0)

    left_waypoints = [
        ("left_over_wall", brace_over_pos, brace_rot, 0.45),
        ("left_brace_wall", brace_pos, brace_rot, 0.0),
    ]
    right_waypoints = [
        ("right_over_corner", peel_over_pos, peel0_rot, 0.50),
        ("right_pinch_corner", peel0_pos, peel0_rot, 0.0),
        ("right_peel_15deg", peel15_pos, peel15_rot, 0.0),
        ("right_peel_30deg", peel30_pos, peel30_rot, 0.0),
        ("right_peel_45deg", peel45_pos, peel45_rot, 0.0),
        ("right_peel_58deg", peel58_pos, peel58_rot, 0.0),
        ("right_carry_panel", carry_pos, carry_rot, 0.0),
        ("right_lay_flat", drop_pos, drop_rot, 0.0),
    ]

    ik_rows = {}
    q_prev = np.array(READY, dtype=float)
    left_solutions = {}
    for name, pos, rot, op in left_waypoints:
        q, mm_err, deg_err, margin = solve_ik("left", pos, rot, start=q_prev)
        q_prev = q
        hs = hand_hulls("left", q, op * MAX_JAW)
        left_solutions[name] = (q, pos, rot, hs)
        ik_rows[name] = {
            "mm_err": round(mm_err, 3),
            "deg_err": round(deg_err, 3),
            "margin_deg": round(margin, 2),
            "lowest_above_bench_mm": round(1000.0 * (lowest_z(hs) - TOP), 1),
            "lowest_above_cans_mm": round(1000.0 * (lowest_z(hs) - can_top_z), 1),
        }

    q_prev = np.array(READY, dtype=float)
    right_solutions = {}
    for name, pos, rot, op in right_waypoints:
        q, mm_err, deg_err, margin = solve_ik("right", pos, rot, start=q_prev)
        q_prev = q
        hs = hand_hulls("right", q, op * MAX_JAW)
        right_solutions[name] = (q, pos, rot, hs)
        ik_rows[name] = {
            "mm_err": round(mm_err, 3),
            "deg_err": round(deg_err, 3),
            "margin_deg": round(margin, 2),
            "lowest_above_bench_mm": round(1000.0 * (lowest_z(hs) - TOP), 1),
            "lowest_above_cans_mm": round(1000.0 * (lowest_z(hs) - can_top_z), 1),
        }

    _, l_pos, _, l_hulls = left_solutions["left_brace_wall"]
    separations = {}
    for r_name in ("right_pinch_corner", "right_peel_15deg", "right_peel_30deg", "right_peel_45deg", "right_peel_58deg"):
        _, r_pos, _, r_hulls = right_solutions[r_name]
        tcp_mm = 1000.0 * float(np.linalg.norm(l_pos - r_pos))
        hull_mm = 1000.0 * hulls_gap_m(l_hulls, r_hulls)
        separations[r_name] = {"tcp_center_mm": round(tcp_mm, 1), "hull_gap_mm": round(hull_mm, 1)}

    squeeze_per_finger_m = 0.5 * P.PANEL_GRIP_T
    normal_per_finger_n = FINGER_STIFFNESS * squeeze_per_finger_m
    mu_pad = min(C.GRIPPY_FRICTION[0], 1.2)
    tangential_pull_cap_n = 2.0 * mu_pad * normal_per_finger_n

    proj_320 = {
        "tray_front_left": project_top_cam((P.FRONT_X, +0.5 * P.TRAY_W, TOP + P.TRAY_H), 320, 176),
        "tray_front_right": project_top_cam((P.FRONT_X, -0.5 * P.TRAY_W, TOP + P.TRAY_H), 320, 176),
        "tray_rear_center": project_top_cam((P.REAR_X, 0.0, TOP + P.TRAY_H), 320, 176),
        "perforation_mid": project_top_cam((P.FRONT_X, 0.0, TOP + P.LIP_H), 320, 176),
        "drop_zone_center": project_top_cam((P.DROP_ZONE_XY[0], P.DROP_ZONE_XY[1], TOP), 320, 176),
        "drop_zone_outer": project_top_cam((P.DROP_ZONE_XY[0], P.DROP_ZONE_XY[1] - 0.5 * P.DROP_ZONE_SIZE[1], TOP), 320, 176),
    }
    in_frame_320 = all(4.0 <= u <= 316.0 and 4.0 <= v <= 172.0 for u, v, _ in proj_320.values())

    worst_mm = max(r["mm_err"] for r in ik_rows.values())
    min_margin_deg = min(r["margin_deg"] for r in ik_rows.values())
    min_tcp_sep_mm = min(v["tcp_center_mm"] for v in separations.values())
    min_hull_gap_mm = min(v["hull_gap_mm"] for v in separations.values())
    min_bench_clear_mm = min(r["lowest_above_bench_mm"] for r in ik_rows.values())

    verdict = (
        worst_mm <= 0.5
        and min_margin_deg >= 5.0
        and min_tcp_sep_mm >= 200.0
        and min_hull_gap_mm >= 10.0
        and min_bench_clear_mm >= 2.0
        and normal_per_finger_n >= 1.85 * P.PEEL_START_N
        and tangential_pull_cap_n >= 3.5 * P.PEEL_START_N
        and in_frame_320
    )

    return {
        "pass": bool(verdict),
        "preset": P.TRAY_PRESET,
        "worst_ik_mm": round(worst_mm, 3),
        "min_joint_margin_deg": round(min_margin_deg, 2),
        "start_peel_tcp_sep_mm": separations["right_pinch_corner"]["tcp_center_mm"],
        "min_peel_tcp_sep_mm": round(min_tcp_sep_mm, 1),
        "min_peel_hull_gap_mm": round(min_hull_gap_mm, 1),
        "min_bench_clearance_mm": round(min_bench_clear_mm, 1),
        "grasp_fingertip_above_cans_mm": {
            "left_brace": ik_rows["left_brace_wall"]["lowest_above_cans_mm"],
            "right_pinch": ik_rows["right_pinch_corner"]["lowest_above_cans_mm"],
        },
        "pinch_force": {
            "panel_grip_t_mm": round(1000.0 * P.PANEL_GRIP_T, 2),
            "normal_per_finger_n": round(normal_per_finger_n, 2),
            "two_pad_tangential_capacity_n": round(tangential_pull_cap_n, 2),
            "ratio_to_peel_start_n": round(normal_per_finger_n / P.PEEL_START_N, 2),
        },
        "top_cam_320x176_in_frame": in_frame_320,
        "top_cam_320x176_px": proj_320,
        "waypoints": ik_rows,
        "bimanual_separation": separations,
    }


def run_frame0_and_idle(out_dir: Path) -> dict:
    """Render frame-0 start cameras + 320x176 policy crops and verify 2.0 s idle settle in Isaac Sim."""
    import cv2
    import demo_kit as kit
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack

    out_dir.mkdir(parents=True, exist_ok=True)
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    obs = emb.reset(Scene(id="g1_frame0", instruction=m.INSTRUCTION), seed=0)

    saved = {}
    for cam, stem in (
        ("top_cam", "start_top"),
        ("left_cam", "start_left_wrist"),
        ("right_cam", "start_right_wrist"),
    ):
        rgb = obs.images[cam]
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        p640 = out_dir / f"{stem}.png"
        cv2.imwrite(str(p640), bgr)
        p320 = out_dir / f"{stem}_policy.png"
        cv2.imwrite(str(p320), cv2.resize(bgr, (320, 176), interpolation=cv2.INTER_AREA))
        saved[stem] = {"640x360": str(p640), "320x176": str(p320)}

    ready_act = np.array([*READY, 1.0], dtype=float)
    action = Action(data=pack(ready_act, ready_act))
    last_score = 0.0
    for _ in range(40):
        res = emb.step(action)
        last_score = float(res.info.get("score", 0.0))

    rep = m.report()
    idle_summary = {
        "frame0_files": saved,
        "idle_2s_score": last_score,
        "idle_2s_report": rep,
    }
    (out_dir / "g1_frame0_summary.json").write_text(json.dumps(idle_summary, indent=2))
    return idle_summary


def main() -> int:
    if "--parts" in sys.argv:
        print(json.dumps(P.summary(), indent=2))
        return 0

    offline = run_offline_checks()
    print("G1_OFFLINE " + json.dumps(offline, indent=2), flush=True)
    if not offline["pass"]:
        return 1

    if "--frame0" in sys.argv:
        idx = sys.argv.index("--frame0")
        out_dir = Path(sys.argv[idx + 1])
        idle = run_frame0_and_idle(out_dir)
        print("G1_FRAME0 " + json.dumps(idle, indent=2), flush=True)
        if (
            idle["idle_2s_score"] != 0.0
            or idle["idle_2s_report"]["ties_torn"] != 0
            or idle["idle_2s_report"]["zero_reason"] is not None
        ):
            return 2

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
