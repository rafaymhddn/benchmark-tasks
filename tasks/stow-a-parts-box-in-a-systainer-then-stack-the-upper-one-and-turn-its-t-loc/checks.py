#!/usr/bin/env python3
"""Gate-1 kinematics, collision-clearance, and frame-0 start checks for ``mech/tasks/systainer_tloc``.

1. Offline (no simulator): solves every waypoint of the full plan through ``yam_arena.ik``, checking:
   - IK error <= 0.5 mm and <= 0.5 deg;
   - every arm joint >= 5.0 deg inside its URDF limits;
   - every grasp point <= 160 mm above the bench and every jaw opening <= 93.9 mm;
   - hand collision hulls clear (> 0 mm) of the bench and non-contacted bodies;
   - simultaneous two-arm poses >= 200 mm apart at TCP and >= 20 mm apart between collision hulls.
2. ``--frame0 <dir>`` in the simulator: writes ``start_top.png``, ``start_top_policy.png`` (320 x 176),
   ``start_left_wrist.png``, ``start_right_wrist.png``, and verifies 2.0 s idle settle (score 0.0, zero None).
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
POLICY_W, POLICY_H = 320, 176

IK_MM_MAX = 0.5
IK_DEG_MAX = 0.5
MARGIN_DEG_MIN = 5.0
GRASP_Z_MAX = 0.160
TCP_SEP_MIN = 0.200
HULL_SEP_MIN = 0.020


def frame(closing, approach) -> np.ndarray:
    """Gripper 3x3 rotation: columns are closing axis, pad-width axis, approach axis."""
    a = np.asarray(approach, dtype=float)
    a = a / np.linalg.norm(a)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c = c / np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def rx(angle_rad: float) -> np.ndarray:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


# --- Shared grasp orientations and contact-point helpers (used by demo.py) --------------------------------------------
R_BOX = frame([0.0, -1.0, 0.0], [0.0, 0.0, -1.0])  # straight down (yam_arena grip_rotation(0)), closing along y

_LID_TILT = math.radians(35.0)
LID_APP = np.array([0.0, -math.sin(_LID_TILT), -math.cos(_LID_TILT)], dtype=float)
R_LID = frame(np.cross(LID_APP, np.array([1.0, 0.0, 0.0])), LID_APP)  # closed-hand sweep with wrist roll near 0 deg

_SIDE_TILT = math.radians(60.0)


def side_app(side: str) -> np.ndarray:
    sgn = 1.0 if side == "left" else -1.0
    return np.array([0.0, -sgn * math.sin(_SIDE_TILT), -math.cos(_SIDE_TILT)], dtype=float)


SIDE_CRADLE_OPEN_M = 0.076  # m: 76 mm jaw spread gives an 83.7 mm 4-point anti-pitch contact rectangle on smooth walls


def side_rot(side: str) -> np.ndarray:
    """Mirror-symmetric side-wall cradle orientation (closing along -sgn*x; stiff shoulder/elbow squeeze, j6 = +/-86 deg)."""
    sgn = 1.0 if side == "left" else -1.0
    return frame([-sgn, 0.0, 0.0], side_app(side))


def lid_rot() -> np.ndarray:
    """Closed-hand orientation for sweeping lid_low shut with wrist roll near 0 deg."""
    app = side_app("left")
    return frame(np.cross(app, np.array([1.0, 0.0, 0.0])), app)


_TLOC_PITCH = math.radians(10.0)
_TLOC_YAW = math.radians(12.0)
TLOC_APP = np.array(
    [math.cos(_TLOC_PITCH) * math.cos(_TLOC_YAW), math.cos(_TLOC_PITCH) * math.sin(_TLOC_YAW), -math.sin(_TLOC_PITCH)],
    dtype=float,
)


def tloc_rot_stem(deg: float) -> np.ndarray:
    """Right gripper pinching across the -y T-Loc wing (16 mm bar thickness) during Stroke 1 (-55 -> 0 deg)."""
    return frame(rx(math.radians(deg)) @ np.array([0.0, 0.0, -1.0]), TLOC_APP)


def tloc_rot_bar(deg: float) -> np.ndarray:
    """Right gripper pinching across the +y T-Loc wing (16 mm bar thickness) during Stroke 2 (0 -> +90 deg)."""
    return frame(rx(math.radians(deg)) @ np.array([0.0, 0.0, 1.0]), TLOC_APP)


_TIP_CORNERS_CACHE: dict[float, np.ndarray] = {}


def _tip_corners(opening_m: float = 0.0) -> np.ndarray:
    """Distal fingertip corners (z_local >= 0 ahead of TCP on tip_left and tip_right) at jaw ``opening_m``."""
    key = round(float(opening_m), 4)
    if key not in _TIP_CORNERS_CACHE:
        half = -0.5 * max(0.0, min(MAX_JAW, key))
        joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, READY)}, **dict.fromkeys(C.FINGER_JOINTS, half)}
        poses_w = analysis.fk_link_poses_world(joints, (0.0, 0.0, 0.0))
        g = poses_w["gripper"]
        verts = np.vstack([h.vertices for _, h in analysis.world_hulls(poses_w, ["tip_left", "tip_right"])])
        local = (verts - g[:3, 3]) @ g[:3, :3] - np.array(C.TCP_OFFSET)
        _TIP_CORNERS_CACHE[key] = local[local[:, 2] >= 0.0]
    return _TIP_CORNERS_CACHE[key]


def tip_offset_along(rot: np.ndarray, direction_world, opening_m: float = 0.0) -> np.ndarray:
    """Centroid of distal fingertip contact face furthest along ``direction_world`` (in gripper frame, m)."""
    d = np.asarray(direction_world, dtype=float)
    d = d / np.linalg.norm(d)
    corners = _tip_corners(opening_m)
    proj = (corners @ rot.T) @ d
    return corners[proj > proj.max() - 0.0015].mean(axis=0)


def side_wall_tcp(
    side: str,
    x: float,
    z_above_bench: float,
    out_m: float = 0.0,
    opening_m: float = SIDE_CRADLE_OPEN_M,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(tcp_world, rot, local_contact_pt) for 4-point flat-pad cradle on smooth side wall y = +/-CASE_W/2."""
    sgn = 1.0 if side == "left" else -1.0
    rot = side_rot(side)
    inward = np.array([0.0, -sgn, 0.0], dtype=float)
    at_local = tip_offset_along(rot, inward, opening_m=opening_m)
    wall_pt = np.array([x, sgn * (P.CASE_W / 2.0 + out_m), TOP + z_above_bench], dtype=float)
    return wall_pt - rot @ at_local, rot, at_local


def top_brace_tcp(x: float, y: float, z_above_bench: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Closed Left hand pressing down (-z) in pure xz-plane (app_y = 0, zero lateral -y wipe, TCP <= 156 mm)."""
    p = math.radians(36.0)
    app = np.array([math.cos(p), 0.0, -math.sin(p)], dtype=float)
    rot = frame(np.array([0.0, -1.0, 0.0], dtype=float), app)
    at_local = tip_offset_along(rot, np.array([0.0, 0.0, -1.0]), opening_m=0.0)
    pt = np.array([x, y, TOP + z_above_bench], dtype=float)
    return pt - rot @ at_local, rot, at_local


def lid_push_tcp(x: float, y: float, z_above_bench: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rot = lid_rot()
    at_local = tip_offset_along(rot, np.array([-1.0, 0.0, -1.0]), opening_m=0.0)
    pt = np.array([x, y, TOP + z_above_bench], dtype=float)
    return pt - rot @ at_local, rot, at_local


def tloc_wing_tcp(
    deg: float,
    back_m: float = 0.0,
    low_pos=None,
    low_rot=None,
) -> tuple[np.ndarray, np.ndarray]:
    """TCP on the -19.5 mm T-Loc wing (upper arc z = 61..45 mm, clear of bench & case_top) during Stroke 1 (-55 -> 0 deg)."""
    p_low = np.array([P.CASE_LOW_XY[0], P.CASE_LOW_XY[1], TOP], dtype=float) if low_pos is None else np.asarray(low_pos, dtype=float)
    r_low = np.eye(3) if low_rot is None else np.asarray(low_rot, dtype=float)
    r_tloc = rx(math.radians(deg))
    app_w = r_low @ TLOC_APP
    close_w = r_low @ (r_tloc @ np.array([0.0, 0.0, -1.0]))
    rot = frame(close_w, app_w)
    wing_local = np.array(P.TLOC_PIVOT_LOCAL, dtype=float) + r_tloc @ np.array([-0.01175, -0.0195, 0.0], dtype=float)
    wing_w = p_low + r_low @ wing_local
    return wing_w - (0.0117 + back_m) * app_w, rot


def tloc_bar_tcp(
    deg: float,
    back_m: float = 0.0,
    low_pos=None,
    low_rot=None,
) -> tuple[np.ndarray, np.ndarray]:
    """TCP on the +19.5 mm T-Loc wing (upper arc z = 45..64.5 mm, clear of bench & case_top) during Stroke 2 (0 -> +90 deg)."""
    p_low = np.array([P.CASE_LOW_XY[0], P.CASE_LOW_XY[1], TOP], dtype=float) if low_pos is None else np.asarray(low_pos, dtype=float)
    r_low = np.eye(3) if low_rot is None else np.asarray(low_rot, dtype=float)
    r_tloc = rx(math.radians(deg))
    app_w = r_low @ TLOC_APP
    close_w = r_low @ (r_tloc @ np.array([0.0, 0.0, 1.0]))
    rot = frame(close_w, app_w)
    wing_local = np.array(P.TLOC_PIVOT_LOCAL, dtype=float) + r_tloc @ np.array([-0.01175, +0.0195, 0.0], dtype=float)
    wing_w = p_low + r_low @ wing_local
    return wing_w - (0.0117 + back_m) * app_w, rot


# --- Waypoint dictionary & collision checks ---------------------------------------------------------------------------
def all_poses() -> dict[str, tuple[str, np.ndarray, np.ndarray, float, str]]:
    """name -> (side, tcp, rot, jaw_opening_m, stage)."""
    out = {}
    bx, by = P.BOX_START_XY
    cx = P.CASE_LOW_XY[0] + P.CELL_LOCAL_XY[0]
    cy = P.CASE_LOW_XY[1] + P.CELL_LOCAL_XY[1]

    # Stage 1: Left picks red parts_box near top 8 mm (z_tcp = 50 mm) and sets it onto floor (z_tcp = 58 mm, 1 mm drop!)
    cy_place = cy - 0.003
    out["box_hover"] = ("left", np.array([bx, by, TOP + 0.098]), R_BOX, MAX_JAW, "s1_pick")
    out["box_grasp"] = ("left", np.array([bx, by, TOP + 0.050]), R_BOX, P.BOX_W, "s1_pick")
    out["box_lift"] = ("left", np.array([bx, by, TOP + 0.125]), R_BOX, P.BOX_W, "s1_carry")
    out["box_over_cell"] = ("left", np.array([cx, cy_place, TOP + 0.128]), R_BOX, P.BOX_W, "s1_place")
    out["box_release"] = ("left", np.array([cx, cy_place, TOP + 0.075]), R_BOX, 0.072, "s1_place")

    # Stage 2: Right hovers clear beside case_top while Left routes via y = +0.195 behind open lid and sweeps shut gently
    op_c = SIDE_CRADLE_OPEN_M
    out["right_lid_brace"] = ("right", *side_wall_tcp("right", P.CASE_TOP_XY[0], 0.055, out_m=0.032, opening_m=op_c)[:2], op_c, "s2_lid")
    for tag, (lx, ly, lz) in (
        ("lid_out_front", (0.320, 0.195, 0.135)),
        ("lid_around", (0.412, 0.195, 0.138)),
        ("lid_start", (0.412, 0.090, 0.138)),
        ("lid_mid", (0.355, 0.090, 0.122)),
        ("lid_shut", (0.315, 0.090, 0.075)),
    ):
        tcp, rot, _ = lid_push_tcp(lx, ly, lz)
        out[tag] = ("left", tcp, rot, 0.0, "s2_lid")

    # Stage 3: Two-arm 4-point side-wall flat-pad cradle (76 mm jaw spread = 83.7 mm x-span preventing pitch)
    x0 = P.CASE_TOP_XY[0]
    x_land = P.CASE_LOW_XY[0] - P.CLEAT_SLIDE
    x_coupled = P.CASE_LOW_XY[0]
    for side in ("left", "right"):
        out[f"{side}_side_approach"] = (side, *side_wall_tcp(side, x0, 0.046, out_m=0.025)[:2], op_c, "s3_start")
        out[f"{side}_side_squeeze"] = (side, *side_wall_tcp(side, x0, 0.046, out_m=0.002)[:2], op_c, "s3_start")
        out[f"{side}_side_lift"] = (side, *side_wall_tcp(side, x0, 0.148, out_m=0.002)[:2], op_c, "s3_air")
        out[f"{side}_side_carry"] = (side, *side_wall_tcp(side, x_land, 0.148, out_m=0.002)[:2], op_c, "s3_air")
        out[f"{side}_side_land"] = (side, *side_wall_tcp(side, x_land, 0.120, out_m=0.002)[:2], op_c, "s3_land")
        out[f"{side}_side_slid"] = (side, *side_wall_tcp(side, x_coupled, 0.116, out_m=0.002)[:2], op_c, "s3_stacked")
        out[f"{side}_side_off"] = (side, *side_wall_tcp(side, x_coupled, 0.118, out_m=0.028)[:2], op_c, "s3_stacked")

    # Stage 4: Left presses down (-z) on top of stack at (0.392, +0.124, 0.1475) while Right turns T-Loc
    out["left_brace"] = ("left", *top_brace_tcp(0.392, 0.124, 0.1475)[:2], 0.0, "s4_tloc")
    out["tloc_s1_approach"] = ("right", *tloc_wing_tcp(P.TLOC_OPEN_DEG, back_m=0.040), 0.038, "s4_tloc")
    out["tloc_s1_open"] = ("right", *tloc_wing_tcp(P.TLOC_OPEN_DEG), P.TLOC_BAR_T, "s4_tloc")
    out["tloc_s1_mid"] = ("right", *tloc_wing_tcp(-27.5), P.TLOC_BAR_T, "s4_tloc")
    out["tloc_s1_close"] = ("right", *tloc_wing_tcp(2.0), P.TLOC_BAR_T, "s4_tloc")
    out["tloc_s2_approach"] = ("right", *tloc_bar_tcp(0.0, back_m=0.040), 0.038, "s4_tloc")
    out["tloc_s2_close"] = ("right", *tloc_bar_tcp(0.0), P.TLOC_BAR_T, "s4_tloc")
    out["tloc_s2_mid"] = ("right", *tloc_bar_tcp(45.0), P.TLOC_BAR_T, "s4_tloc")
    out["tloc_s2_connect"] = ("right", *tloc_bar_tcp(91.0), P.TLOC_BAR_T, "s4_tloc")
    return out


TOGETHER = (
    ("lid_out_front", "right_lid_brace"),
    ("lid_around", "right_lid_brace"),
    ("lid_start", "right_lid_brace"),
    ("lid_mid", "right_lid_brace"),
    ("lid_shut", "right_lid_brace"),
    ("left_side_approach", "right_side_approach"),
    ("left_side_squeeze", "right_side_squeeze"),
    ("left_side_lift", "right_side_lift"),
    ("left_side_carry", "right_side_carry"),
    ("left_side_land", "right_side_land"),
    ("left_side_slid", "right_side_slid"),
    ("left_side_off", "right_side_off"),
    ("left_brace", "tloc_s1_approach"),
    ("left_brace", "tloc_s1_open"),
    ("left_brace", "tloc_s1_mid"),
    ("left_brace", "tloc_s1_close"),
    ("left_brace", "tloc_s2_approach"),
    ("left_brace", "tloc_s2_close"),
    ("left_brace", "tloc_s2_mid"),
    ("left_brace", "tloc_s2_connect"),
)


def solve_ik(side: str, pos, rot, q_seed=None) -> tuple[np.ndarray, float, float, float]:
    seeds = ([np.asarray(q_seed, dtype=float)] if q_seed is not None else []) + [
        np.array(s, dtype=float)
        for s in (
            READY,
            [0.2, 1.1, 1.2, -0.4, 0.0, 0.0],
            [-0.2, 1.1, 1.2, -0.4, 0.0, 0.0],
            [0.4, 1.3, 1.3, -0.3, 0.2, 0.0],
            [-0.4, 1.3, 1.3, -0.3, -0.2, 0.0],
        )
    ]
    quat = ik.matrix_to_quat_xyzw(np.asarray(rot, dtype=float))
    best: tuple | None = None
    for q0 in seeds:
        q, ep, er = ik.solve(np.asarray(pos, dtype=float), quat, q0, base_pos=BASES[side], iters=120)
        margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
        ok = ep * 1000.0 <= IK_MM_MAX and math.degrees(er) <= IK_DEG_MAX
        key = (ok, margin if ok else -(ep * 1000.0 + math.degrees(er)))
        if best is None or key > best[0]:
            best = (key, q, float(ep * 1000.0), float(math.degrees(er)), margin)
        if ok and margin >= 10.0:
            break
    assert best is not None
    return best[1], best[2], best[3], best[4]


def hand_hulls(side: str, q: np.ndarray, opening_m: float) -> list:
    half = -0.5 * max(0.0, min(MAX_JAW, opening_m))
    joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, q)}, **dict.fromkeys(C.FINGER_JOINTS, half)}
    pw = analysis.fk_link_poses_world(joints, tuple(BASES[side]))
    out = []
    for lk in HAND_LINKS:
        for h in analysis.link_hulls().get(lk, []):
            r, p = pw[lk][:3, :3], pw[lk][:3, 3]
            nw = h.planes[:, :3] @ r.T
            verts = h.vertices @ r.T + p
            out.append(
                (
                    verts.min(axis=0),
                    verts.max(axis=0),
                    analysis.Hull(vertices=verts, planes=np.c_[nw, h.planes[:, 3] + nw @ p]),
                )
            )
    return out


def box_hull(centre, size) -> tuple[np.ndarray, np.ndarray, analysis.Hull]:
    c, h = np.asarray(centre, dtype=float), np.asarray(size, dtype=float) / 2.0
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h + c
    planes = []
    for i in range(3):
        for s in (1.0, -1.0):
            n = np.zeros(3)
            n[i] = s
            planes.append([*n, float(n @ c + h[i])])
    return c - h, c + h, analysis.Hull(vertices=corners, planes=np.array(planes))


def hull_gap(ha: list, hb: list) -> float:
    best = math.inf
    for lo_a, hi_a, a in ha:
        for lo_b, hi_b, b in hb:
            aabb = float(np.linalg.norm(np.maximum(0.0, np.maximum(lo_a - hi_b, lo_b - hi_a))))
            if aabb >= best:
                continue
            best = min(best, float(analysis._signed(a.vertices, b).min()), float(analysis._signed(b.vertices, a).min()))
    return best


def open_lid_hull() -> tuple[np.ndarray, np.ndarray, analysis.Hull]:
    rad = P.LID_OPEN_RAD
    hinge = np.array([P.CASE_LOW_XY[0] + P.LID_HINGE_LOCAL[0], 0.0, TOP + P.LID_HINGE_LOCAL[2]], dtype=float)
    ry = np.array([[math.cos(rad), 0.0, math.sin(rad)], [0.0, 1.0, 0.0], [-math.sin(rad), 0.0, math.cos(rad)]])
    half = np.array([0.084, P.CASE_W / 2.0, 0.005])
    c = hinge + ry @ np.array([-0.082, 0.0, 0.002])
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
    verts = corners @ ry.T + c
    planes = [[*(s * ry[:, i]), float((s * ry[:, i]) @ c + half[i])] for i in range(3) for s in (1.0, -1.0)]
    return verts.min(axis=0), verts.max(axis=0), analysis.Hull(vertices=verts, planes=np.array(planes))


def stage_obstacles(stage: str, pose_name: str) -> dict[str, list]:
    obs = {"bench": [box_hull((0.26, 0.0, TOP - 0.010), (0.56, 0.68, 0.020))]}
    lx, ly = P.CASE_LOW_XY
    x_long = -P.INNER_D / 2.0 + P.CELL_D + P.DIVIDER_T / 2.0
    y_short = P.INNER_W / 2.0 - P.CELL_W - P.DIVIDER_T / 2.0
    low_tub = [
        box_hull((lx, ly, TOP + 0.0035), (P.CASE_D, P.CASE_W, 0.007)),
        box_hull((lx, ly + (P.CASE_W - P.WALL_T) / 2.0, TOP + 0.034), (P.CASE_D, P.WALL_T, 0.068)),
        box_hull((lx, ly - (P.CASE_W - P.WALL_T) / 2.0, TOP + 0.034), (P.CASE_D, P.WALL_T, 0.068)),
        box_hull((lx + (P.CASE_D - P.WALL_T) / 2.0, ly, TOP + 0.033), (P.WALL_T, P.CASE_W, 0.066)),
        box_hull((lx - (P.CASE_D - P.WALL_T) / 2.0, ly, TOP + 0.034), (P.WALL_T, P.CASE_W, 0.068)),
        box_hull((lx + x_long, ly, TOP + P.FLOOR_TOP_Z + P.DIVIDER_H / 2.0), (P.DIVIDER_T, P.INNER_W, P.DIVIDER_H)),
        box_hull((lx + P.CELL_LOCAL_XY[0], ly + y_short, TOP + P.FLOOR_TOP_Z + P.DIVIDER_H / 2.0), (P.CELL_D, P.DIVIDER_T, P.DIVIDER_H)),
    ]
    if stage in ("s1_pick", "s1_carry", "s1_place"):
        obs["case_low"] = low_tub + [open_lid_hull()]
        obs["case_top"] = [box_hull((P.CASE_TOP_XY[0], 0.0, TOP + P.CASE_H / 2.0), (P.CASE_D, P.CASE_W, P.CASE_H))]
    elif stage == "s2_lid" and pose_name in ("lid_around", "lid_start"):
        obs["case_low"] = low_tub + [open_lid_hull()]
    elif stage == "s4_tloc" and pose_name.startswith("tloc_"):
        wing_w = P.CASE_W / 2.0 - P.RECESS_HALF_Y
        obs["stack"] = [
            box_hull((lx, sy * (P.RECESS_HALF_Y + wing_w / 2.0), TOP + 0.0735), (P.CASE_D, wing_w, 0.147))
            for sy in (-1.0, 1.0)
        ] + [box_hull((lx + 0.008, 0.0, TOP + 0.0735), (0.155, 2.0 * P.RECESS_HALF_Y, 0.147))]
    return obs


def run_offline() -> dict:
    poses = all_poses()
    solved, rows = {}, {}
    ok = True
    worst_ik_mm, min_margin_deg, max_z_mm = 0.0, 999.0, 0.0
    prev_q: dict[str, np.ndarray | None] = {"left": None, "right": None}
    for name, (side, tcp, rot, opening, stage) in poses.items():
        q, mm, deg, margin = solve_ik(side, tcp, rot, prev_q[side])
        prev_q[side] = q
        solved[name] = (side, q, opening, tcp)
        hulls = hand_hulls(side, q, opening)
        clear = {k: round(1000.0 * hull_gap(hulls, v), 1) for k, v in stage_obstacles(stage, name).items()}
        z_over = float(tcp[2] - TOP)
        good = (
            mm <= IK_MM_MAX
            and deg <= IK_DEG_MAX
            and margin >= MARGIN_DEG_MIN
            and z_over <= GRASP_Z_MAX
            and opening <= MAX_JAW
            and min(clear.values()) > 0.0
        )
        ok = ok and good
        worst_ik_mm = max(worst_ik_mm, mm)
        min_margin_deg = min(min_margin_deg, margin)
        max_z_mm = max(max_z_mm, 1000.0 * z_over)
        rows[name] = {
            "side": side,
            "tcp_m": [round(float(v), 4) for v in tcp],
            "z_over_bench_mm": round(1000.0 * z_over, 1),
            "ik_mm_deg": [round(mm, 3), round(deg, 3)],
            "margin_deg": round(margin, 1),
            "clear_mm": clear,
            "ok": good,
        }
    pairs = {}
    min_hull_sep_mm, min_tcp_sep_mm = 999.0, 999.0
    for a, b in TOGETHER:
        sa, qa, oa, tcpa = solved[a]
        sb, qb, ob, tcpb = solved[b]
        tcp_d = float(np.linalg.norm(tcpa - tcpb))
        h_d = hull_gap(hand_hulls(sa, qa, oa), hand_hulls(sb, qb, ob))
        good = tcp_d >= TCP_SEP_MIN and h_d >= HULL_SEP_MIN
        ok = ok and good
        min_hull_sep_mm = min(min_hull_sep_mm, 1000.0 * h_d)
        min_tcp_sep_mm = min(min_tcp_sep_mm, 1000.0 * tcp_d)
        pairs[f"{a} + {b}"] = {
            "tcp_apart_mm": round(1000.0 * tcp_d, 1),
            "hulls_apart_mm": round(1000.0 * h_d, 1),
            "ok": good,
        }
    return {
        "pass": bool(ok),
        "n_poses": len(rows),
        "worst_ik_mm": round(worst_ik_mm, 3),
        "min_joint_margin_deg": round(min_margin_deg, 1),
        "max_grasp_z_mm": round(max_z_mm, 1),
        "min_tcp_apart_mm": round(min_tcp_sep_mm, 1),
        "min_hulls_apart_mm": round(min_hull_sep_mm, 1),
        "poses": rows,
        "simultaneous_pairs": pairs,
    }


def run_frame0(out_dir: Path) -> dict:
    import cv2
    import demo_kit as kit
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack

    out_dir.mkdir(parents=True, exist_ok=True)
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    obs = emb.reset(Scene(id="g1", instruction=m.INSTRUCTION), seed=0)
    for cam, stem in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        bgr = cv2.cvtColor(np.asarray(obs.images[cam]), cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(out_dir / f"{stem}.png"), bgr)
        cv2.imwrite(str(out_dir / f"{stem}_policy.png"), cv2.resize(bgr, (POLICY_W, POLICY_H), interpolation=cv2.INTER_AREA))
    ready = np.array([*READY, 1.0], dtype=float)
    act = Action(data=pack(ready, ready))
    res = None
    for _ in range(40):
        res = emb.step(act)
    moved = {
        name: round(1000.0 * float(np.linalg.norm(m.mk.pose(name)[0] - np.asarray(m.mk.built_pose(name)[0]))), 2)
        for name in ("case_low", "case_top", "parts_box")
    }
    out = {
        "score_after_2s": float(res.info.get("score", 0.0)),
        "moved_from_built_mm": moved,
        "lid_deg": round(m.lid_deg(), 2),
        "tloc_deg": round(m.tloc_deg(), 2),
        "zero": m.STATE.get("zero"),
        "report": m.report(),
    }
    out["pass"] = (
        out["score_after_2s"] == 0.0
        and out["zero"] is None
        and max(moved.values()) < 3.0
        and abs(out["lid_deg"] - P.LID_OPEN_DEG) < 6.0
        and abs(out["tloc_deg"] - P.TLOC_OPEN_DEG) < 4.0
    )
    (out_dir / "g1_frame0.json").write_text(json.dumps(out, indent=2, default=str))
    return out


def main() -> int:
    if "--parts" in sys.argv:
        print(json.dumps(P.summary(), indent=2))
        return 0
    offline = run_offline()
    brief = {k: v for k, v in offline.items() if k not in ("poses", "simultaneous_pairs")}
    print("G1_OFFLINE " + json.dumps(brief), flush=True)
    if not offline["pass"]:
        bad = {k: v for k, v in offline["poses"].items() if not v["ok"]}
        bad_p = {k: v for k, v in offline["simultaneous_pairs"].items() if not v["ok"]}
        print("FAILED_POSES " + json.dumps({"poses": bad, "pairs": bad_p}, indent=2), flush=True)
        return 1
    if "--frame0" in sys.argv:
        out = run_frame0(Path(sys.argv[sys.argv.index("--frame0") + 1]))
        print("G1_FRAME0 " + json.dumps(out, default=str), flush=True)
        return 0 if out["pass"] else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
