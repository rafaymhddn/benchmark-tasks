#!/usr/bin/env python3
"""Offline checks (G1) for ``pixi_level`` without the simulator:
1. Key figures & static tipping / flop verification from ``parts.py``;
2. Simultaneous left-hand (phone at ~144-167 mm) and right-hand (button pinch at ~103 mm, ~87 mm apart) IK, joint
   margins and URDF collision-hull clearances across the full 20 deg roll + 10 deg tilt levelling trajectory;
3. Top-camera and right-wrist-camera projections of the phone and the 18 mm red button.

    python3 mech/tasks/pixi_level/checks.py
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np
from yam_arena import analysis, constants as C, ik, station
from yam_arena.task_file import TABLE_TOP
from yam_arena.urdf import quat_xyzw_to_matrix

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import parts as P  # noqa: E402

TOP = TABLE_TOP["station"]
BASES = {side: np.array(station.BASES[side], dtype=float) for side in ("left", "right")}
HAND_LINKS = ["link4", "link5", "gripper", "tip_left", "tip_right"]

# Grasp geometry derived from parts.py
BTN_AXIS = np.array(P.button_slide_axis(), dtype=float)  # inward along button press
BTN_OUTWARD = -BTN_AXIS
BUTTON_TCP = np.array(P.button_built_pos(TOP), dtype=float) + 0.5 * P.BUTTON_PROUD * BTN_OUTWARD

# Right-wrist look pose in front-right of the button so the red button sits squarely in right_cam
LOOK_BUTTON_TCP = np.array([P.TRI_XY[0] - 0.095, P.TRI_XY[1] - 0.095, TOP + P.OVER + P.BUTTON_Z + 0.055], dtype=float)


def frame(closing, approach) -> np.ndarray:
    """TCP rotation matrix whose columns are (closing axis, pad width axis, approach axis)."""
    a = np.asarray(approach, dtype=float)
    a = a / np.linalg.norm(a)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c = c / np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


# Right hand comes in from in front (-x -> +x, orthogonal to BTN_AXIS) with wrist camera pointing down away from the phone
_RIGHT_HORIZ = np.array([BTN_AXIS[1], -BTN_AXIS[0], 0.0], dtype=float)
RIGHT_PINCH_ROT_RAW = frame(-BTN_AXIS, _RIGHT_HORIZ + np.array([0.0, 0.0, -0.28]))
# Left hand in the phone's body frame: fingers close across phone's 7.8 mm thickness (x), coming in from +y with slight -z
LEFT_IN_PHONE_RAW = frame([-1.0, 0.0, 0.0], [0.18, -0.93, -0.32])


def solve(side: str, pos, rot, start=None):
    """Best IK of a TCP pose: returns (q, used_rot, pos_err_mm, rot_err_deg, min_joint_margin_deg)."""
    starts = [np.asarray(start, dtype=float)] if start is not None else []
    starts += [
        np.array(C.READY, dtype=float),
        np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0]),
        np.array([-0.35, 1.30, 0.65, 0.15, 0.85, 0.55]),
        np.array([0.50, 1.70, 0.65, 0.55, -1.00, -0.45]),
        np.array([0.0, 1.2, 1.4, 0.3, 0.0, 1.0]),
        np.array([0.0, 1.2, 1.4, 0.3, 0.0, -1.0]),
    ]
    turns = [rot] if start is not None else [rot, rot @ np.diag([-1.0, -1.0, 1.0])]
    best = None
    for turned in turns:
        for q0 in starts:
            q, e, re = ik.solve(np.asarray(pos, dtype=float), ik.matrix_to_quat_xyzw(turned), q0, base_pos=BASES[side], iters=150)
            margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
            ok = e < 0.001 and re < 0.01
            key = (ok, margin if ok else -(e + 0.05 * re))
            if best is None or key > best[0]:
                best = (key, q, turned, 1000.0 * e, math.degrees(re), margin)
        if start is not None and best is not None and best[0][0]:
            break
    assert best is not None
    return best[1], best[2], best[3], best[4], best[5]


def link_poses(side: str, q, opening: float):
    joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, q)}, **dict.fromkeys(C.FINGER_JOINTS, -opening / 2.0)}
    return analysis.fk_link_poses_world(joints, tuple(BASES[side]))


def hulls(side: str, q, opening: float):
    poses = link_poses(side, q, opening)
    out = []
    for link in HAND_LINKS:
        for i, h in enumerate(analysis.link_hulls().get(link, [])):
            r, p = poses[link][:3, :3], poses[link][:3, 3]
            n_w = h.planes[:, :3] @ r.T
            out.append((link, i, analysis.Hull(vertices=h.vertices @ r.T + p, planes=np.c_[n_w, h.planes[:, 3] + n_w @ p])))
    return out


def hands_apart(ha, hb) -> float:
    best = math.inf
    for _, _, a in ha:
        lo_a, hi_a = a.vertices.min(0), a.vertices.max(0)
        for _, _, b in hb:
            lo_b, hi_b = b.vertices.min(0), b.vertices.max(0)
            if np.maximum(lo_b - hi_a, lo_a - hi_b).max() > 0.08:
                continue
            best = min(best, float(analysis._signed(a.vertices, b).min()), float(analysis._signed(b.vertices, a).min()))
    return best


def lowest(hs) -> float:
    return float(min(h.vertices[:, 2].min() for _, _, h in hs))


def box_points(center, rot, size, step: float = 0.004) -> np.ndarray:
    h = np.asarray(size, dtype=float) / 2.0
    pts = []
    for axis in range(3):
        others = [i for i in range(3) if i != axis]
        u = np.linspace(-h[others[0]], h[others[0]], max(2, int(2 * h[others[0]] / step) + 1))
        v = np.linspace(-h[others[1]], h[others[1]], max(2, int(2 * h[others[1]] / step) + 1))
        uu, vv = np.meshgrid(u, v)
        for sign in (-1.0, 1.0):
            p = np.zeros((uu.size, 3))
            p[:, axis], p[:, others[0]], p[:, others[1]] = sign * h[axis], uu.ravel(), vv.ravel()
            pts.append(p)
    return np.vstack(pts) @ np.asarray(rot, dtype=float).T + np.asarray(center, dtype=float)


def to_box(hs, box_spec) -> float:
    center, rot, size = box_spec
    pts = box_points(center, rot, size)
    return float(
        min(
            min(float(analysis.box_signed_distance(h.vertices, center, rot, size).min()), float(analysis._signed(pts, h).min()))
            for _, _, h in hs
        )
    )


def wrist_camera_world(side: str, q, opening: float = 0.060):
    t = link_poses(side, q, opening)["gripper"]
    cam = station.wrist_camera(side)
    return t[:3, :3] @ quat_xyzw_to_matrix(np.array(cam.rot_xyzw, dtype=float)), t[:3, :3] @ np.array(cam.pos, dtype=float) + t[:3, 3], cam.lens


def project(cam, point_world) -> tuple[np.ndarray, float]:
    r, o, lens = cam
    c = r.T @ (np.asarray(point_world, dtype=float) - o)
    return np.array([lens.cx + lens.fx * c[0] / -c[2], lens.cy - lens.fy * c[1] / -c[2]]), float(-c[2])


def disc_view(cam, centre, radius: float, ax_a, ax_b) -> dict:
    uv, d = project(cam, centre)
    lens = cam[2]
    if d <= 0:
        return {"in_frame": False, "dist_m": round(d, 3)}
    sizes = [
        float(np.linalg.norm(project(cam, centre + radius * np.asarray(ax, float))[0] - project(cam, centre - radius * np.asarray(ax, float))[0]))
        for ax in (ax_a, ax_b)
    ]
    in_frame = bool(0 <= uv[0] <= lens.width and 0 <= uv[1] <= lens.height)
    return {
        "px_640x360": uv.round(1).tolist(),
        "px_320x176": [round(float(uv[0]) * 320.0 / lens.width, 1), round(float(uv[1]) * 176.0 / lens.height, 1)],
        "dist_m": round(d, 3),
        "across_px_640": [round(s, 1) for s in sizes],
        "across_px_320": [round(s * 0.5, 1) for s in sizes],
        "in_frame": in_frame,
    }


def phone_grasp_world(tilt_rad: float, roll_rad: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(left_tcp_world, phone_center_world, phone_rot_world) for head angles ``(tilt_rad, roll_rad)``."""
    ball = np.array(P.ball_anchor_world(TOP), dtype=float)
    rp = P.head_rotation(tilt_rad, roll_rad)
    phone_c = ball + rp @ np.array([0.0, 0.0, P.PHONE_CENTER_Z - P.BALL_Z], dtype=float)
    tcp = phone_c + rp @ np.array(P.PHONE_GRASP_LOCAL, dtype=float)
    return tcp, phone_c, rp


def solve_right_look_pose():
    """Find a right-arm waypoint from which the right wrist camera frames the red button squarely."""
    btn_face = P.button_face_world(TOP)
    tan_dir = np.array([BTN_AXIS[1], -BTN_AXIS[0], 0.0], dtype=float)
    best = None
    for tcp in (
        LOOK_BUTTON_TCP,
        np.array([P.TRI_XY[0] - 0.07, P.TRI_XY[1] - 0.13, TOP + 0.16]),
        np.array([P.TRI_XY[0] - 0.04, P.TRI_XY[1] - 0.15, TOP + 0.15]),
        np.array([P.TRI_XY[0] - 0.10, P.TRI_XY[1] - 0.11, TOP + 0.17]),
    ):
        for roll_deg in (0, 90, -90, 180, 45, -45):
            a = (btn_face - tcp) / np.linalg.norm(btn_face - tcp)
            rot0 = frame(np.cross(a, [0.0, 0.0, 1.0]), a) @ P.rz(math.radians(roll_deg))
            q, r_used, emm, _, marg = solve("right", tcp, rot0)
            if emm > 1.0 or marg < 8.0:
                continue
            cam = wrist_camera_world("right", q, 0.060)
            view = disc_view(cam, btn_face, P.BUTTON_R, tan_dir, [0.0, 0.0, 1.0])
            if not view.get("in_frame"):
                continue
            u, v = view["px_640x360"]
            dist_center = math.hypot(u - 320.0, v - 180.0)
            if 60 <= u <= 580 and 50 <= v <= 310:
                cand = (dist_center - 2.0 * marg, tcp.tolist(), r_used, q, round(emm, 2), round(marg, 1), view)
                if best is None or cand[0] < best[0]:
                    best = cand
    return best


def main() -> dict:
    # 1. Right hand pinching the button at ~103.5 mm (solved sequentially from READY, matching demo_kit.Arm)
    r_pinch = RIGHT_PINCH_ROT_RAW
    qr, er_m, er_rad = ik.solve(BUTTON_TCP, ik.matrix_to_quat_xyzw(r_pinch), np.array(C.READY, dtype=float), base_pos=BASES["right"], iters=150)
    er_mm, er_deg = 1000.0 * er_m, math.degrees(er_rad)
    mr_deg = float(np.degrees(min((qr - ik.LOWER).min(), (ik.UPPER - qr).min())))
    hr = hulls("right", qr, P.HEAD_PINCH_SPAN)

    # 2. Left hand holding the phone at ~144 mm (start) and levelling across 6 steps to ~167 mm (level)
    tcp0, _, rp0 = phone_grasp_world(P.START_TILT_RAD, P.START_ROLL_RAD)
    ql0, r_left0, _, _, _ = solve("left", tcp0, rp0 @ LEFT_IN_PHONE_RAW)
    left_in_phone = rp0.T @ r_left0

    sweep_rows = []
    ql = ql0
    for s in np.linspace(0.0, 1.0, 6):
        tilt = P.START_TILT_RAD * (1.0 - s)
        roll = P.START_ROLL_RAD * (1.0 - s)
        l_tcp, phone_c, rp = phone_grasp_world(tilt, roll)
        l_rot = rp @ left_in_phone
        ql, _, emm, edeg, marg = solve("left", l_tcp, l_rot, start=ql)
        hl = hulls("left", ql, P.PHONE_T + 0.002)
        phone_box = (phone_c, rp, np.array([P.PHONE_T, P.PHONE_L, P.PHONE_W]))
        apart_mm = 1000.0 * hands_apart(hl, hr)
        right_to_phone_mm = 1000.0 * to_box(hr, phone_box)
        tcp_gap_mm = 1000.0 * float(np.linalg.norm(l_tcp - BUTTON_TCP))
        sweep_rows.append(
            {
                "s": round(float(s), 2),
                "tilt_deg": round(math.degrees(tilt), 1),
                "roll_deg": round(math.degrees(roll), 1),
                "left_z_mm": round(1000.0 * (float(l_tcp[2]) - TOP), 1),
                "tcp_apart_mm": round(tcp_gap_mm, 1),
                "left_ik_mm": round(emm, 2),
                "left_ik_deg": round(edeg, 2),
                "left_margin_deg": round(marg, 1),
                "left_low_mm": round(1000.0 * (lowest(hl) - TOP), 1),
                "hands_hull_apart_mm": round(apart_mm, 1),
                "right_hand_to_phone_mm": round(right_to_phone_mm, 1),
            }
        )

    # 3. Camera visibility (top camera + right wrist camera look pose)
    tcam = station.top_camera()
    top_cam_tuple = (quat_xyzw_to_matrix(np.array(tcam.rot_xyzw, dtype=float)), np.array(tcam.pos, dtype=float), tcam.lens)
    btn_face = P.button_face_world(TOP)
    tan_dir = np.array([BTN_AXIS[1], -BTN_AXIS[0], 0.0], dtype=float)
    _, phone_c0, rp_start = phone_grasp_world(P.START_TILT_RAD, P.START_ROLL_RAD)
    top_button = disc_view(top_cam_tuple, btn_face, P.BUTTON_R, tan_dir, [0.0, 0.0, 1.0])
    top_phone_left, _ = project(top_cam_tuple, phone_c0 + rp_start @ np.array([0.0, 0.5 * P.PHONE_L, 0.0]))
    top_phone_right, _ = project(top_cam_tuple, phone_c0 + rp_start @ np.array([0.0, -0.5 * P.PHONE_L, 0.0]))

    look_best = solve_right_look_pose()
    look_summary = None
    if look_best is not None:
        _, l_tcp_r, _, _, l_emm, l_marg, l_view = look_best
        look_summary = {
            "tcp_rel_bench_m": (np.array(l_tcp_r) - np.array([0.0, 0.0, TOP])).round(3).tolist(),
            "ik_mm": l_emm,
            "margin_deg": l_marg,
            "button_in_right_wrist": l_view,
        }

    return {
        "statics": P.static_tipping_summary(),
        "right_button_pinch": {
            "tcp_z_mm": round(1000.0 * (float(BUTTON_TCP[2]) - TOP), 1),
            "pinch_span_mm": round(1000.0 * P.HEAD_PINCH_SPAN, 1),
            "ik_mm": round(er_mm, 2),
            "ik_deg": round(er_deg, 2),
            "margin_deg": round(mr_deg, 1),
            "low_mm": round(1000.0 * (lowest(hr) - TOP), 1),
            "joints_deg": np.degrees(qr).round(1).tolist(),
        },
        "levelling_sweep": sweep_rows,
        "summary": {
            "worst_left_ik_mm": max(r["left_ik_mm"] for r in sweep_rows),
            "worst_right_ik_mm": round(er_mm, 2),
            "min_left_margin_deg": min(r["left_margin_deg"] for r in sweep_rows),
            "min_right_margin_deg": round(mr_deg, 1),
            "min_hands_hull_apart_mm": min(r["hands_hull_apart_mm"] for r in sweep_rows),
            "min_right_hand_to_phone_mm": min(r["right_hand_to_phone_mm"] for r in sweep_rows),
            "tcp_distance_mm_range": [min(r["tcp_apart_mm"] for r in sweep_rows), max(r["tcp_apart_mm"] for r in sweep_rows)],
        },
        "cameras": {
            "top_cam_button": top_button,
            "top_cam_phone_ends_px_640": [top_phone_left.round(1).tolist(), top_phone_right.round(1).tolist()],
            "top_cam_phone_vertical_slant_px_320": round(0.5 * abs(float(top_phone_left[1] - top_phone_right[1])), 1),
            "right_wrist_look": look_summary,
        },
    }


if __name__ == "__main__":
    print(json.dumps(main(), indent=2))
