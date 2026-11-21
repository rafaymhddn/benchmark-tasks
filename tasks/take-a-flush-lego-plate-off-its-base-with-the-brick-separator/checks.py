#!/usr/bin/env python3
"""Gate G1 offline geometry, IK, URDF collision-hull, and camera checks for ``mech/tasks/lego_separator`` (no simulator).

    python3 mech/tasks/lego_separator/checks.py    (with yam_arena importable)

Verifies:
1. Right hand picking ``separator`` across its 16.0 mm width at ``SEP_START_XY``.
2. Right hand seating ``separator`` squarely over ``target``'s near 2-stud row at ``(0.260, -0.080)`` (separator picked from
   ``SEP_START_XY``, in front of the base).
3. Right hand rocking ``separator`` 0 -> 15 -> 20 -> 25 deg over its lower edge on the base's studs (the socket rises
   past the 5.8 mm at which ``target`` reaches ``PEEL_RELEASE_DEG = 12.0 deg``), hand hulls above the bench.
4. Hand-over pose (~75-80 mm TCP spacing) with ``target`` stuck in ``separator``: both a strictly horizontal (level, 0 deg)
   left-hand approach and a 20-deg plate-coaxial approach across ``target``'s 15.8 mm width, plus the stud pull-out stroke,
   confirming > 40 mm arm-to-arm convex-hull clearance and clear tool/plate separation.
5. Left hand placing ``target`` into ``dish`` at ``(0.24, +0.17)`` and right hand laying ``separator`` back on the bench.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np
from yam_arena import analysis, constants as C, ik, station
from yam_arena.task_file import TABLE_TOP

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import parts as P  # noqa: E402

TOP = TABLE_TOP["station"]
BASES = {side: np.array(station.BASES[side], dtype=float) for side in ("left", "right")}
HAND_LINKS = ["link3", "link4", "link5", "gripper", "tip_left", "tip_right"]
PAD_HULLS = {"tip_left": {14, 15}, "tip_right": {5, 8}}

GRASP_ALONG_HANDLE = 0.022  # m along sloped handle from (EDGE_X, -EDGE_DZ) -> ~38 mm behind socket centre
GRASP_BACKOFF = 0.0104  # m along -approach: on settled bench (over=0), fingertips stay +1.6 mm above bench & grip 6.4 mm of handle
HANDOVER_SOCKET_POS = np.array([0.250, 0.000, TOP + 0.130], dtype=float)
HANDOVER_ROCK_DEG = 30.0  # the handle 5 deg over level in the hand-over (35 deg handle)
LEVER_ROCK_DEG = 36.0  # the trials and the demo rock at most this far (the handle's tail 3.9 mm over the bench)
TARGET_GRASP_X_IN_S = -0.0200  # m in S: 16 mm ahead of the -4 mm socket nose, centred on target's free 24 mm end


def rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


def frame(closing, approach) -> np.ndarray:
    a = np.asarray(approach, dtype=float) / np.linalg.norm(approach)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def solve(side: str, pos, rot, start=None):
    """Best IK of a TCP pose: (rot_used, q, mm_err, deg_err, smallest_joint_margin_deg)."""
    starts = ([np.asarray(start, dtype=float)] if start is not None else []) + [
        np.array(s, dtype=float)
        for s in (
            C.READY,
            [0.0, 1.0, 1.0, -0.6, 0.0, 0.0],
            [0.0, 1.2, 1.3, -0.4, 0.0, 1.4],
            [0.0, 1.2, 1.3, -0.4, 0.0, -1.4],
            [0.3, 0.9, 1.0, 0.2, 0.4, 0.0],
            [-0.3, 0.9, 1.0, 0.2, -0.4, 0.0],
        )
    ]
    turns = [rot] if start is not None else [rot, rot @ np.diag([-1.0, -1.0, 1.0])]
    best = None
    for turned in turns:
        for q0 in starts:
            q, pe, re = ik.solve(np.asarray(pos, dtype=float), ik.matrix_to_quat_xyzw(turned), q0, base_pos=BASES[side], iters=100)
            margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
            ok = pe < 0.001 and re < 0.01
            key = (ok, margin if ok else -(pe + 0.05 * re))
            if best is None or key > best[0]:
                best = (key, turned, q, 1000.0 * pe, math.degrees(re), margin)
        if start is not None and best[0][0]:
            break
    return best[1], best[2], best[3], best[4], best[5]


def hulls(side: str, q, opening_m: float):
    fj = max(C.FINGER_OPEN, min(0.0, -opening_m / 2.0))
    joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, q)}, **dict.fromkeys(C.FINGER_JOINTS, fj)}
    poses = analysis.fk_link_poses_world(joints, tuple(BASES[side]))
    out = []
    for link in HAND_LINKS:
        for i, h in enumerate(analysis.link_hulls().get(link, [])):
            r, p = poses[link][:3, :3], poses[link][:3, 3]
            nw = h.planes[:, :3] @ r.T
            out.append((link, i, analysis.Hull(vertices=h.vertices @ r.T + p, planes=np.c_[nw, h.planes[:, 3] + nw @ p])))
    return out


def box_points(center, rot, size, step: float = 0.002) -> np.ndarray:
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


def to_box_mm(hs, box_spec, *, skip_pads: bool = False) -> tuple[float, str]:
    center, rot, size = box_spec
    pts = box_points(center, rot, size)
    best, who = math.inf, ""
    for link, i, h in hs:
        if skip_pads and i in PAD_HULLS.get(link, ()):
            continue
        d = min(
            float(analysis.box_signed_distance(h.vertices, center, rot, size).min()),
            float(analysis._signed(pts, h).min()),
        )
        if d < best:
            best, who = d, f"{link}#{i}"
    return 1000.0 * best, who


def hands_apart_mm(ha, hb) -> tuple[float, str]:
    best, who = math.inf, ""
    for la, ia, a in ha:
        lo_a, hi_a = a.vertices.min(0), a.vertices.max(0)
        for lb, ib, b in hb:
            lo_b, hi_b = b.vertices.min(0), b.vertices.max(0)
            if np.maximum(lo_b - hi_a, lo_a - hi_b).max() > 0.06:
                continue
            d = min(float(analysis._signed(a.vertices, b).min()), float(analysis._signed(b.vertices, a).min()))
            if d < best:
                best, who = d, f"{la}#{ia}/{lb}#{ib}"
    return 1000.0 * best, who


def min_z_above_bench_mm(hs) -> float:
    return 1000.0 * (min(float(h.vertices[:, 2].min()) for _, _, h in hs) - TOP)


def right_grasp_in_sep() -> tuple[np.ndarray, np.ndarray]:
    """(grasp_point_in_S, rot_in_S) on the separator's handle across its 16.0 mm width, with finger order locked cleanly
    for the right arm from bench pick through levering and hand-over."""
    u = np.array([math.cos(P.HANDLE_RAD), 0.0, math.sin(P.HANDLE_RAD)], dtype=float)
    n = np.array([-math.sin(P.HANDLE_RAD), 0.0, math.cos(P.HANDLE_RAD)], dtype=float)
    mid = np.array([P.EDGE_X, 0.0, -P.EDGE_DZ], dtype=float) + GRASP_ALONG_HANDLE * u + 0.5 * P.HANDLE_BEAM_T * n
    approach = -n
    g_s = mid - GRASP_BACKOFF * approach
    r_s = frame([0.0, 1.0, 0.0], approach)
    # Lock finger order against settled bench pick pose
    bench_pos, _ = P.separator_bench_pose(TOP, over=0.0)
    r_bench = rx(+P.HANDLE_RAD) @ P.R_SEP_SEATED
    used_rot, _, _, _, _ = solve("right", np.array(bench_pos) + r_bench @ g_s, r_bench @ r_s)
    if np.allclose(used_rot, (r_bench @ r_s) @ np.diag([-1.0, -1.0, 1.0])):
        r_s = r_s @ np.diag([-1.0, -1.0, 1.0])
    return g_s, r_s


def sep_grasp_world(sep_pos: np.ndarray, sep_rot: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    g_s, r_s = right_grasp_in_sep()
    return np.asarray(sep_pos, dtype=float) + sep_rot @ g_s, sep_rot @ r_s


def handover_poses(left_pitch_deg: float = 0.0) -> dict:
    """World poses for the ~75-80 mm hand-over in clear air: right hand holds separator at HANDOVER_ROCK_DEG, left hand
    enters at ``left_pitch_deg`` below horizontal (0.0 = strictly level; 20.0 = coaxial with the target plate) to pinch
    target's free 24 mm end across its 15.8 mm width and pull along the stud axis."""
    r_sep = rx(math.radians(HANDOVER_ROCK_DEG)) @ P.R_SEP_SEATED
    p_sep = HANDOVER_SOCKET_POS.copy()
    g_r, rot_r = sep_grasp_world(p_sep, r_sep)

    # Target stuck in separator socket: x_s in [-27.9, +3.9] mm, z_s in [-3.2, 0.0] mm, width along y_s
    g_l = p_sep + r_sep @ np.array([TARGET_GRASP_X_IN_S, 0.0, -0.5 * P.PLATE_T], dtype=float)
    pitch_rad = math.radians(left_pitch_deg)
    # Approach from +y towards -y (into target's free end along +x_s, which points along -y_world in horizontal projection)
    app_l = np.array([0.0, -math.cos(pitch_rad), -math.sin(pitch_rad)], dtype=float)
    cls_l = r_sep[:, 1]  # across target's 15.8 mm width (+x_world)
    rot_l0 = frame(cls_l, app_l)
    rot_l, q_l, pe_l, re_l, m_l = solve("left", g_l, rot_l0)

    # Pull stroke: 14 mm along -z_s (down along the studs, > 8x the 1.7 mm STUD_H) plus 2 mm along -x_s away from socket nose
    pull_delta = r_sep @ np.array([-0.002, 0.0, -0.014], dtype=float)
    g_l_pull = g_l + pull_delta
    return {
        "p_sep": p_sep,
        "r_sep": r_sep,
        "right": (g_r, rot_r),
        "left_pinch": (g_l, rot_l, q_l, pe_l, re_l, m_l),
        "left_pull_pos": g_l_pull,
    }


def main() -> dict:
    out: dict = {"parts": P.summary()}

    # 1. Right hand picking separator on the settled bench
    bench_pos, _ = P.separator_bench_pose(TOP, over=0.0)
    r_bench = rx(+P.HANDLE_RAD) @ P.R_SEP_SEATED
    g_pick, rot_pick = sep_grasp_world(np.array(bench_pos), r_bench)
    _, q_pick, pe, re, margin = solve("right", g_pick, rot_pick)
    h_pick = hulls("right", q_pick, P.SEP_W + 0.018)
    base_box = (np.array([P.BASE_XY[0], P.BASE_XY[1], TOP + 0.5 * P.BASE_T]), np.eye(3), np.array([P.BASE_W, P.BASE_L, P.BASE_T]))
    to_base, who_base = to_box_mm(h_pick, base_box)
    out["1_pick_separator"] = {
        "tcp_m": np.round(g_pick, 4).tolist(),
        "ik_mm": round(pe, 3),
        "ik_deg": round(re, 3),
        "margin_deg": round(margin, 1),
        "min_hull_above_bench_mm": round(min_z_above_bench_mm(h_pick), 1),
        "hull_to_base_mm": round(to_base, 1),
        "nearest_base_hull": who_base,
    }

    # 2 & 3. Seating (0 deg) and rocking (15, 22, and 27 deg) over the lower fulcrum edge on base's free studs
    lever_rows = []
    q_cur = q_pick
    for deg in (0.0, 15.0, 25.0, LEVER_ROCK_DEG):
        rad = math.radians(deg)
        p_s, r_s = P.separator_rocked_pose(TOP, rad, over=0.0)
        g_w, rot_w = sep_grasp_world(p_s, r_s)
        _, q_cur, pe_r, re_r, m_r = solve("right", g_w, rot_w, start=q_cur)
        h_r = hulls("right", q_cur, P.SEP_W)
        # Kinematic socket lift above settled seated height, and corresponding rigid peel angle of target about far edge:
        seat_z = P.near_row_top_world(TOP, over=0.0)[2]
        sock_lift_mm = 1000.0 * float(p_s[2] - seat_z)
        peel_kin_deg = math.degrees(math.asin(min(1.0, max(0.0, (p_s[2] - seat_z) / P.ROW_LEVER))))
        # Tail lowest point above bench:
        u = np.array([math.cos(P.HANDLE_RAD), 0.0, math.sin(P.HANDLE_RAD)])
        tail_w = p_s + r_s @ (np.array([P.EDGE_X, 0.0, -P.EDGE_DZ]) + P.HANDLE_BEAM_L * u)
        lever_rows.append(
            {
                "rock_deg": deg,
                "socket_lift_mm": round(sock_lift_mm, 2),
                "target_peel_deg": round(peel_kin_deg, 2),
                "tail_above_bench_mm": round(1000.0 * float(tail_w[2] - TOP), 1),
                "ik_mm": round(pe_r, 3),
                "margin_deg": round(m_r, 1),
                "min_hull_above_bench_mm": round(min_z_above_bench_mm(h_r), 1),
            }
        )
    out["2_seat_and_lever"] = lever_rows

    # 4. Hand-over in clear air (~75 mm between TCPs): both strictly level (0 deg) and coaxial (20 deg)
    ho_rows = []
    for l_pitch in (0.0, 20.0):
        ho = handover_poses(l_pitch)
        p_sep, r_sep = ho["p_sep"], ho["r_sep"]
        g_r, rot_r = ho["right"]
        _, q_r, pe_r, re_r, m_r = solve("right", g_r, rot_r, start=q_cur)
        g_l, rot_l, q_l, pe_l, re_l, m_l = ho["left_pinch"]
        _, q_lp, pe_lp, re_lp, m_lp = solve("left", ho["left_pull_pos"], rot_l, start=q_l)

        h_r = hulls("right", q_r, P.SEP_W)
        h_l_open = hulls("left", q_l, P.PLATE_W + 0.016)
        h_l_pull = hulls("left", q_lp, P.PLATE_W)
        apart_pinch, pair_pinch = hands_apart_mm(h_l_open, h_r)
        apart_pull, pair_pull = hands_apart_mm(h_l_pull, h_r)

        # Check left-hand clearance from separator front socket box, and right-hand clearance from target plate box
        sep_front_box = (p_sep + r_sep @ np.array([0.007, 0.0, 0.0025]), r_sep, np.array([0.022, P.SEP_W, 0.008]))
        target_in_ho = (p_sep + r_sep @ np.array([-1.5 * P.PITCH, 0.0, -0.5 * P.PLATE_T]), r_sep @ P.R_SEP_SEATED.T, np.array([P.PLATE_W, P.PLATE_L, P.PLATE_T]))
        l_to_sep, who_ls = to_box_mm(h_l_open, sep_front_box)
        r_to_target, who_rb = to_box_mm(h_r, target_in_ho)

        ho_rows.append(
            {
                "left_pitch_deg": l_pitch,
                "tcp_distance_mm": round(1000.0 * float(np.linalg.norm(g_r - g_l)), 1),
                "right_ik_mm": round(pe_r, 3),
                "right_margin_deg": round(m_r, 1),
                "left_pinch_ik_mm": round(pe_l, 3),
                "left_pinch_margin_deg": round(m_l, 1),
                "left_pull_ik_mm": round(pe_lp, 3),
                "left_pull_margin_deg": round(m_lp, 1),
                "hands_apart_pinch_mm": round(apart_pinch, 1),
                "hands_apart_pull_mm": round(apart_pull, 1),
                "closest_pair": pair_pinch,
                "left_hull_to_separator_mm": round(l_to_sep, 1),
                "right_hull_to_target_mm": round(r_to_target, 1),
            }
        )
    out["3_handover"] = ho_rows

    # 5. Dish drop (left arm) & separator bench placement (right arm)
    dish_drop = np.array([P.DISH_XY[0], P.DISH_XY[1], TOP + P.DISH_H + 0.015], dtype=float)
    rot_drop = frame([1.0, 0.0, 0.0], [0.0, 0.0, -1.0])
    _, q_d, pe_d, re_d, m_d = solve("left", dish_drop, rot_drop)
    h_d = hulls("left", q_d, P.PLATE_W + 0.024)
    out["4_dish_drop"] = {
        "tcp_m": np.round(dish_drop, 4).tolist(),
        "ik_mm": round(pe_d, 3),
        "margin_deg": round(m_d, 1),
        "min_hull_above_bench_mm": round(min_z_above_bench_mm(h_d), 1),
    }

    return out


if __name__ == "__main__":
    res = main()
    print(json.dumps(res, indent=2))
