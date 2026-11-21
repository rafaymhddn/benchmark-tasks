#!/usr/bin/env python3
"""Gate G1 checks for the child's bike helmet fit task:
1. Offline (no simulator):
   - Left hand IK (<= 1.0 mm, <= 1.0 deg) and joint margins (>= 5 deg) at the front edge both tipped back
     (TILT_START_DEG, front edge 148 mm over the bench, rim top 159 mm) and level (front edge 122 mm), for both a
     closed-finger top press and a front-visor pinch; plus the left wrist camera's view of the front edge and the brow band.
   - Right hand IK and joint margins pinching the 28 mm rear dial (91 mm over the bench level) with the gripper
     level, turning through a full 90 deg regrasp stroke (-45 deg to +45 deg) without branch jumps.
   - Bimanual grasp-point separation, hand-to-hand hull separation, bench and neck clearance, and >= 25 mm
     clearance of every arm link hull from the head's face (eyes, nose).
   - Top-camera framing at 640 x 360 and 320 x 176 policy resolution.
2. Simulator frame-0 + 2.0 s idle settle (``--frame0 <out_dir>``):
   - Saves ``start_top.png``, ``start_top_policy.png`` (320 x 176), ``start_left_wrist.png``,
     ``start_left_wrist_policy.png``, ``start_right_wrist.png``, ``start_right_wrist_policy.png``, and
     verifies 2.0 s idle stability at the start tilt, dial = 0 deg, score = 0.0.
"""

from __future__ import annotations

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

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
CHECK_LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")
MAX_JAW = 2.0 * abs(float(C.FINGER_OPEN))  # 0.0939 m
FINGER_STIFFNESS = float(C.GRIPPER["stiffness"])  # 5000.0 N/m per finger


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


def qm(q) -> np.ndarray:
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def box_points(center, rot, size, step: float = 0.008) -> np.ndarray:
    h = 0.5 * np.asarray(size, dtype=float)
    pts = []
    for axis in range(3):
        others = [i for i in range(3) if i != axis]
        u = np.linspace(-h[others[0]], h[others[0]], max(2, int(2 * h[others[0]] / step) + 1))
        v = np.linspace(-h[others[1]], h[others[1]], max(2, int(2 * h[others[1]] / step) + 1))
        uu, vv = np.meshgrid(u, v)
        for sign in (-1.0, 1.0):
            p = np.zeros((uu.size, 3), dtype=float)
            p[:, axis], p[:, others[0]], p[:, others[1]] = sign * h[axis], uu.ravel(), vv.ravel()
            pts.append(p)
    return np.vstack(pts) @ np.asarray(rot, dtype=float).T + np.asarray(center, dtype=float)


def solve(side: str, pos, rot, start=None, allow_flip: bool = True):
    """Best IK solution: (q, pos_err_mm, rot_err_deg, min_joint_margin_deg, rot_used)."""
    best = None
    starts = [np.asarray(start, dtype=float)] if start is not None else []
    starts += [
        np.array(s, dtype=float)
        for s in (
            [-0.50, 1.38, 1.00, -0.65, 0.22, 0.88],  # Left elbow-up front-edge branch (~55 deg margin)
            [0.12, 2.22, 0.65, 0.65, -1.35, 0.00],  # Right wrist-up level +y dial branch
            [0.25, 2.30, 0.80, 1.20, -1.20, 0.00],
            [0.0, 1.0, 1.0, -0.6, 0.0, 0.0],
        )
    ]
    turns = [rot] if (start is not None or not allow_flip) else [rot, rot @ np.diag([-1.0, -1.0, 1.0])]
    for turned in turns:
        for q0 in starts:
            q, e, re = ik.solve(
                np.asarray(pos, dtype=float),
                ik.matrix_to_quat_xyzw(turned),
                q0,
                base_pos=BASES[side],
                iters=120,
            )
            margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
            ok = e < 0.001 and re < 0.015
            jump_pen = 0.0 if start is None else 0.15 * float(np.degrees(np.abs(q - start).max()))
            key = (ok, (margin - jump_pen) if ok else -(e + 0.05 * re))
            if best is None or key > best[0]:
                best = (key, q, 1000.0 * float(e), math.degrees(float(re)), margin, turned)
        if start is not None and best[0][0] and best[4] >= 4.0:
            break
    return best[1], best[2], best[3], best[4], best[5]


def link_poses(side: str, q, opening: float):
    joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, q)}, **dict.fromkeys(C.FINGER_JOINTS, -0.5 * opening)}
    return analysis.fk_link_poses_world(joints, tuple(BASES[side]))


def hulls(side: str, q, opening: float):
    poses = link_poses(side, q, opening)
    out = []
    for link in CHECK_LINKS:
        for i, h in enumerate(analysis.link_hulls().get(link, [])):
            r, p = poses[link][:3, :3], poses[link][:3, 3]
            n_w = h.planes[:, :3] @ r.T
            out.append((link, i, analysis.Hull(vertices=h.vertices @ r.T + p, planes=np.c_[n_w, h.planes[:, 3] + n_w @ p])))
    return out


def to_box(hs, box_spec) -> float:
    center, rot, size = box_spec
    pts = box_points(center, rot, size)
    return float(
        min(
            min(analysis.box_signed_distance(h.vertices, center, rot, size).min(), analysis._signed(pts, h).min())
            for _, _, h in hs
        )
    )


def to_sphere(hs, center, radius: float) -> float:
    """Least clearance of the hulls from a sphere: the centre's largest plane distance to each hull less the radius,
    a lower bound (outside a convex hull that plane distance never exceeds the true one)."""
    c = np.asarray(center, dtype=float)[None, :]
    return float(min(analysis._signed(c, h)[0] for _, _, h in hs) - radius)


def hands_apart(ha, hb) -> float:
    best = math.inf
    aabb_best = math.inf
    for _, _, a in ha:
        lo_a, hi_a = a.vertices.min(0), a.vertices.max(0)
        for _, _, b in hb:
            lo_b, hi_b = b.vertices.min(0), b.vertices.max(0)
            gap_vec = np.maximum(0.0, np.maximum(lo_b - hi_a, lo_a - hi_b))
            aabb_gap = float(np.linalg.norm(gap_vec))
            aabb_best = min(aabb_best, aabb_gap)
            if gap_vec.max() > 0.06:
                continue
            best = min(best, float(analysis._signed(a.vertices, b).min()), float(analysis._signed(b.vertices, a).min()))
    return float(aabb_best if math.isinf(best) else best)


def lowest_hull_z(hs) -> float:
    return float(min(h.vertices[:, 2].min() for _, _, h in hs))


def wrist_camera(side: str, q, opening: float = MAX_JAW):
    t = link_poses(side, q, opening)["gripper"]
    cam = station.wrist_camera(side)
    return t[:3, :3] @ qm(cam.rot_xyzw), t[:3, :3] @ np.array(cam.pos, dtype=float) + t[:3, 3], cam.lens


def top_camera():
    cam = station.top_camera()
    return qm(cam.rot_xyzw), np.array(cam.pos, dtype=float), cam.lens


def project(cam, pt_world) -> tuple[np.ndarray, float, bool]:
    r, o, lens = cam
    c = r.T @ (np.asarray(pt_world, dtype=float) - o)
    depth = float(-c[2])
    if depth <= 1e-4:
        return np.array([-1.0, -1.0]), depth, False
    uv = np.array([lens.cx + lens.fx * c[0] / depth, lens.cy - lens.fy * c[1] / depth], dtype=float)
    inside = bool(8.0 <= uv[0] <= lens.width - 8.0 and 8.0 <= uv[1] <= lens.height - 8.0)
    return uv, depth, inside


# --- Canonical bimanual grasp poses (shared by demo.py) ---


def left_press_pose(tilt_deg: float, lift_m: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Left hand closed-finger top press on the helmet front rim (grip = 0.0, closing along -x, ~48 deg joint margin).
    Diagonal +y/+z approach lets left_cam see the white forehead, 8 mm brow band, and front edge simultaneously."""
    ax = math.sin(math.radians(14.0))
    ay = -math.sin(math.radians(44.0))
    az = -math.sqrt(1.0 - ax * ax - ay * ay)
    rot = frame(closing=(-1.0, 0.0, 0.0), approach=(ax, ay, az))
    import demo_kit as kit

    low_in_g = kit.lowest_point(rot)
    touch_world = P.front_top_world(TOP, tilt_deg) + np.array([0.0, 0.0, lift_m], dtype=float)
    pos = touch_world - rot @ low_in_g
    return pos, rot


def left_pinch_pose(tilt_deg: float, lift_m: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Left hand pinch across the 44 mm front visor tab (closing along -x, ~55 deg joint margin, >= 25 mm above face)."""
    ax = math.sin(math.radians(18.0))
    ay = -math.sin(math.radians(22.0))
    az = -math.sqrt(1.0 - ax * ax - ay * ay)
    rot = frame(closing=(-1.0, 0.0, 0.0), approach=(ax, ay, az))
    pos = P.front_top_world(TOP, tilt_deg, inset_y=0.008) + np.array([0.0, 0.0, +0.006 + lift_m], dtype=float)
    return pos, rot


def right_dial_pose(roll_deg: float = 0.0, retreat_m: float = 0.0, tilt_deg: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Right hand pinching the 28 mm rear dial at z = 68 mm coaxial with hinge('dial') (6 deg from horizontal = 99.5% level).
    ``roll_deg`` in [-45, +45] sweeps wrist roll j6 symmetrically across [-46, +44] deg (74 deg j6 margin!)."""
    from yam_arena.urdf import axis_angle_matrix

    center = P.dial_center_world(TOP, tilt_deg)
    app = P.dial_axis_world(tilt_deg)
    pos = center - retreat_m * app
    base_rot = frame(closing=(1.0, 0.0, 0.0), approach=app)
    rot = axis_angle_matrix(app, math.radians(roll_deg + 45.0)) @ base_rot
    return pos, rot


def run_offline_checks() -> dict:
    face_box = P.face_bounds_world(TOP)
    neck_box = (  # the neck stub under the jaw, standing on the bench (parts.head_shapes shape 0)
        np.array(P.head_start_pos(TOP)) + np.array([0.0, -0.012, 0.030]),
        np.eye(3),
        np.array([0.096, 0.096, 0.060]),
    )

    nape_c = np.array(P.head_start_pos(TOP)) + np.array(P.NAPE_CENTER_LOCAL)  # the nape (parts.head_shapes, last)

    worst_ik_mm = 0.0
    worst_ik_deg = 0.0
    min_margin_deg = math.inf
    min_face_clear_mm = math.inf  # the press and the dial poses the demo uses: at least MIN_FACE_ARM_CLEAR_M
    min_pinch_face_clear_mm = math.inf  # the front-visor pinch, the other option: at least 20 mm (spec G1)
    min_bench_clear_mm = math.inf

    # 1. Left hand closed-finger press arc from tipped back (the start tilt) to level (0 deg)
    left_press_rows = []
    q_prev = None
    q_left_level = None
    h_left_level = None
    for tilt in [P.TILT_START_DEG * f for f in (1.0, 0.75, 0.5, 0.25, 0.0)]:
        pos, rot = left_press_pose(tilt)
        q, e_mm, re_deg, m_deg, _ = solve("left", pos, rot, start=q_prev, allow_flip=False)
        q_prev = q
        hs = hulls("left", q, 0.0)
        face_mm = 1000.0 * to_box(hs, face_box)
        bench_mm = 1000.0 * (lowest_hull_z(hs) - TOP)
        edge_z_mm = 1000.0 * (float(P.front_edge_world(TOP, tilt)[2]) - (TOP + P.OVER))
        worst_ik_mm = max(worst_ik_mm, e_mm)
        worst_ik_deg = max(worst_ik_deg, re_deg)
        min_margin_deg = min(min_margin_deg, m_deg)
        min_face_clear_mm = min(min_face_clear_mm, face_mm)
        min_bench_clear_mm = min(min_bench_clear_mm, bench_mm)
        left_press_rows.append(
            {
                "tilt_deg": tilt,
                "front_edge_z_mm": round(edge_z_mm, 1),
                "ik_mm": round(e_mm, 3),
                "ik_deg": round(re_deg, 3),
                "margin_deg": round(m_deg, 1),
                "face_clear_mm": round(face_mm, 1),
                "bench_clear_mm": round(bench_mm, 1),
            }
        )
        if tilt == 0.0:
            q_left_level = q
            h_left_level = hs

    # 2. Left hand front-visor pinch tipped back and level (both options in the G1 spec)
    left_pinch_rows = []
    q_pinch_prev = None
    for tilt in (P.TILT_START_DEG, 0.0):
        pos, rot = left_pinch_pose(tilt)
        q, e_mm, re_deg, m_deg, _ = solve("left", pos, rot, start=q_pinch_prev, allow_flip=False)
        q_pinch_prev = q
        hs = hulls("left", q, P.BRIM_TAB_W)
        face_mm = 1000.0 * to_box(hs, face_box)
        bench_mm = 1000.0 * (lowest_hull_z(hs) - TOP)
        worst_ik_mm = max(worst_ik_mm, e_mm)
        worst_ik_deg = max(worst_ik_deg, re_deg)
        min_margin_deg = min(min_margin_deg, m_deg)
        min_pinch_face_clear_mm = min(min_pinch_face_clear_mm, face_mm)
        min_bench_clear_mm = min(min_bench_clear_mm, bench_mm)
        left_pinch_rows.append(
            {
                "tilt_deg": tilt,
                "ik_mm": round(e_mm, 3),
                "ik_deg": round(re_deg, 3),
                "margin_deg": round(m_deg, 1),
                "face_clear_mm": round(face_mm, 1),
                "bench_clear_mm": round(bench_mm, 1),
            }
        )

    # 3. Left wrist camera check (at pre-contact approach +95 mm in +y and +15 mm in +z looking diagonally into forehead)
    pos_look, rot_look = left_press_pose(P.TILT_START_DEG, lift_m=0.015)
    pos_look = pos_look + np.array([0.0, 0.095, 0.0], dtype=float)
    q_look, _, _, _, _ = solve("left", pos_look, rot_look, allow_flip=False)
    cam_l_ready = wrist_camera("left", np.array(READY, dtype=float))
    cam_l_approach = wrist_camera("left", q_look, 0.0)
    brow_pt = np.array(P.head_start_pos(TOP)) + np.array([0.0, P.HEAD_RY, P.BROW_Z])
    edge_start_pt = P.front_edge_world(TOP, P.TILT_START_DEG)
    edge_level_pt = P.front_edge_world(TOP, 0.0)
    uv_b_r, _, in_b_r = project(cam_l_ready, brow_pt)
    uv_e_r, _, in_e_r = project(cam_l_ready, edge_start_pt)
    uv_b_a, _, in_b_a = project(cam_l_approach, brow_pt)
    uv_e_a, _, in_e_a = project(cam_l_approach, edge_start_pt)
    left_cam_check = {
        "ready_pose": {
            "brow_px": uv_b_r.round(1).tolist(),
            "front_edge_start_px": uv_e_r.round(1).tolist(),
            "both_in_frame": bool(in_b_r and in_e_r),
            "pixel_gap_px": round(float(np.linalg.norm(uv_e_r - uv_b_r)), 1),
        },
        "pre_grasp_pose": {
            "brow_px": uv_b_a.round(1).tolist(),
            "front_edge_start_px": uv_e_a.round(1).tolist(),
            "both_in_frame": bool(in_b_a and in_e_a),
            "pixel_gap_px": round(float(np.linalg.norm(uv_e_a - uv_b_a)), 1),
        },
    }

    # 4. Right hand pinching the 28 mm rear dial at z = 68 mm with gripper level, turning 90 deg (-45 -> +45 deg)
    right_dial_rows = []
    q_r_prev = None
    max_step_jump_deg = 0.0
    min_hands_hull_mm = math.inf
    min_neck_clear_mm = math.inf
    min_nape_clear_mm = math.inf
    for roll_deg in (-45.0, -30.0, -15.0, 0.0, +15.0, +30.0, +45.0):
        pos_r, rot_r = right_dial_pose(roll_deg=roll_deg, retreat_m=0.0, tilt_deg=0.0)
        qr, er_mm, rer_deg, mr_deg, _ = solve("right", pos_r, rot_r, start=q_r_prev, allow_flip=False)
        if q_r_prev is not None:
            max_step_jump_deg = max(max_step_jump_deg, float(np.degrees(np.abs(qr - q_r_prev)).max()))
        q_r_prev = qr
        hs_r = hulls("right", qr, P.DIAL_D)
        face_r_mm = 1000.0 * to_box(hs_r, face_box)
        bench_r_mm = 1000.0 * (lowest_hull_z(hs_r) - TOP)
        neck_r_mm = 1000.0 * to_box(hs_r, neck_box)
        nape_r_mm = 1000.0 * to_sphere(hs_r, nape_c, P.NAPE_SPHERE_R)
        hh_mm = 1000.0 * hands_apart(h_left_level, hs_r)
        worst_ik_mm = max(worst_ik_mm, er_mm)
        worst_ik_deg = max(worst_ik_deg, rer_deg)
        min_margin_deg = min(min_margin_deg, mr_deg)
        min_face_clear_mm = min(min_face_clear_mm, face_r_mm)
        min_bench_clear_mm = min(min_bench_clear_mm, bench_r_mm)
        min_neck_clear_mm = min(min_neck_clear_mm, neck_r_mm)
        min_nape_clear_mm = min(min_nape_clear_mm, nape_r_mm)
        min_hands_hull_mm = min(min_hands_hull_mm, hh_mm)
        right_dial_rows.append(
            {
                "roll_deg": roll_deg,
                "dial_z_mm": round(1000.0 * (float(pos_r[2]) - (TOP + P.OVER)), 1),
                "approach_z": round(float(rot_r[2, 2]), 4),
                "ik_mm": round(er_mm, 3),
                "ik_deg": round(rer_deg, 3),
                "margin_deg": round(mr_deg, 1),
                "j6_deg": round(float(math.degrees(qr[5])), 1),
                "face_clear_mm": round(face_r_mm, 1),
                "neck_clear_mm": round(neck_r_mm, 1),
                "nape_clear_mm": round(nape_r_mm, 1),
                "bench_clear_mm": round(bench_r_mm, 1),
                "hands_hull_mm": round(hh_mm, 1),
            }
        )

    # Also verify Right open-jaw retreat pose (-35 mm along -approach) for regrasp transitions
    pos_ret, rot_ret = right_dial_pose(roll_deg=+45.0, retreat_m=0.035, tilt_deg=0.0)
    qr_ret, er_ret, rer_ret, mr_ret, _ = solve("right", pos_ret, rot_ret, start=q_r_prev, allow_flip=False)
    hs_ret = hulls("right", qr_ret, P.DIAL_D + 0.024)
    retreat_bench_mm = 1000.0 * (lowest_hull_z(hs_ret) - TOP)
    worst_ik_mm = max(worst_ik_mm, er_ret)
    min_margin_deg = min(min_margin_deg, mr_ret)
    min_bench_clear_mm = min(min_bench_clear_mm, retreat_bench_mm)
    min_nape_clear_mm = min(min_nape_clear_mm, 1000.0 * to_sphere(hs_ret, nape_c, P.NAPE_SPHERE_R))

    # 5. Grasp point distance & pinch torque capacity on 28 mm dial
    grasp_front = P.front_edge_world(TOP, 0.0)
    grasp_dial = P.dial_center_world(TOP, 0.0)
    grasp_sep_mm = 1000.0 * float(np.linalg.norm(grasp_front - grasp_dial))

    squeeze_m = 0.004  # command jaw 4 mm tighter than 28 mm dial -> 2 mm per finger = 10 N normal per pad
    normal_per_pad_n = FINGER_STIFFNESS * (0.5 * squeeze_m)
    max_dial_torque_cap_nm = 2.0 * 0.85 * normal_per_pad_n * P.DIAL_R
    req_dial_torque_nm = P.DIAL_TURN_NM + 30.0 * P.BAND_LEAD

    # 6. Top camera projection at 640x360 and 320x176 policy resolution
    tcam = top_camera()
    sx = 320.0 / tcam[2].width
    sy = 176.0 / tcam[2].height
    proj_320 = {}
    all_in_top = True
    for label, pt in (
        ("head_crown", np.array(P.head_start_pos(TOP)) + np.array([0.0, 0.0, P.HEAD_CROWN_Z])),
        ("brow_band", brow_pt),
        ("front_edge_start", edge_start_pt),
        ("front_edge_level", edge_level_pt),
        ("dial_start", P.dial_center_world(TOP, P.TILT_START_DEG)),
        ("dial_level", P.dial_center_world(TOP, 0.0)),
    ):
        uv, _, ok = project(tcam, pt)
        all_in_top = all_in_top and ok
        proj_320[label] = [round(float(uv[0] * sx), 1), round(float(uv[1] * sy), 1)]

    verdict = (
        worst_ik_mm <= 1.0
        and worst_ik_deg <= 1.0
        and min_margin_deg >= 5.0
        and max_step_jump_deg <= 25.0
        and grasp_sep_mm >= 200.0
        and min_hands_hull_mm >= 50.0
        and min_face_clear_mm >= 1000.0 * P.MIN_FACE_ARM_CLEAR_M - 1e-9
        and min_pinch_face_clear_mm >= 20.0
        and min_bench_clear_mm >= 2.0
        and min_neck_clear_mm >= 2.0
        and min_nape_clear_mm >= 2.0
        and max_dial_torque_cap_nm >= 2.0 * req_dial_torque_nm
        and all_in_top
        and (left_cam_check["ready_pose"]["both_in_frame"] or left_cam_check["pre_grasp_pose"]["both_in_frame"])
    )

    return {
        "pass": bool(verdict),
        "worst_ik_mm": round(worst_ik_mm, 3),
        "worst_ik_deg": round(worst_ik_deg, 3),
        "min_joint_margin_deg": round(min_margin_deg, 1),
        "max_90deg_turn_step_jump_deg": round(max_step_jump_deg, 1),
        "grasp_points_sep_mm": round(grasp_sep_mm, 1),
        "min_hands_hull_sep_mm": round(min_hands_hull_mm, 1),
        "min_face_clearance_mm": round(min_face_clear_mm, 1),
        "pinch_face_clearance_mm": round(min_pinch_face_clear_mm, 1),
        "min_neck_clearance_mm": round(min_neck_clear_mm, 1),
        "min_nape_clearance_mm": round(min_nape_clear_mm, 1),
        "min_bench_clearance_mm": round(min_bench_clear_mm, 1),
        "dial_pinch_capacity": {
            "normal_per_pad_n": round(normal_per_pad_n, 1),
            "torque_capacity_nm": round(max_dial_torque_cap_nm, 3),
            "required_torque_nm": round(req_dial_torque_nm, 3),
        },
        "top_cam_320x176_in_frame": all_in_top,
        "top_cam_320x176_px": proj_320,
        "left_wrist_cam_readability": left_cam_check,
        "left_press_arc": left_press_rows,
        "left_pinch_front_edge": left_pinch_rows,
        "right_dial_90deg_turn": right_dial_rows,
    }


def run_frame0_and_idle(out_dir: Path) -> dict:
    """Render frame-0 start cameras + 320 x 176 policy crops and verify 2.0 s idle settle in Isaac Sim."""
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

    # Also step Left arm to non-contact pre-grasp view (+95 mm in +y, +15 mm in +z) to verify left_cam readability in rendered pixels
    pos_look, rot_look = left_press_pose(P.TILT_START_DEG, lift_m=0.015)
    pos_look = pos_look + np.array([0.0, 0.095, 0.0], dtype=float)
    q_look, _, _, _, _ = solve("left", pos_look, rot_look, allow_flip=False)
    q0 = np.array(READY, dtype=float)
    obs_look = obs
    for step_idx in range(1, 25):
        s = step_idx / 24.0
        s_smooth = s * s * (3.0 - 2.0 * s)
        q_cmd = (1.0 - s_smooth) * q0 + s_smooth * q_look
        act_look = Action(data=pack(np.array([*q_cmd, 0.0], dtype=float), ready_act))
        obs_look = emb.step(act_look).observation
    bgr_look = cv2.cvtColor(obs_look.images["left_cam"], cv2.COLOR_RGB2BGR)
    p_look640 = out_dir / "left_wrist_readability.png"
    p_look320 = out_dir / "left_wrist_readability_policy.png"
    cv2.imwrite(str(p_look640), bgr_look)
    cv2.imwrite(str(p_look320), cv2.resize(bgr_look, (320, 176), interpolation=cv2.INTER_AREA))
    saved["left_wrist_readability"] = {"640x360": str(p_look640), "320x176": str(p_look320)}

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
        rep = idle["idle_2s_report"]
        if (
            idle["idle_2s_score"] != 0.0
            or abs(float(rep["tilt_deg"]) - P.TILT_START_DEG) > 0.5
            or float(rep["dial_deg"]) > 1.0
            or rep["zero_reason"] is not None
        ):
            return 2

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
