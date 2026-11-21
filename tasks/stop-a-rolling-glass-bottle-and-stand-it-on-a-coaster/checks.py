#!/usr/bin/env python3
"""Gate-1 dry checks and frame-0 camera verification for ``mech/tasks/rolling_bottle``.

Offline (CPU container, no simulator):
    python mech/tasks/rolling_bottle/checks.py

Frame-0 simulator render (writes start_top.png, start_top_policy.png, start_left_wrist.png, start_right_wrist.png):
    scripts/rta simpy g2b0 mech/tasks/rolling_bottle/checks.py --render-start outputs/g2b0
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
from yam_arena import analysis, constants as C, ik, station
from yam_arena.constants import READY

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import parts as P  # noqa: E402

BASES = {side: np.array(station.BASES[side], dtype=float) for side in ("left", "right")}
HAND_LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")
POLICY_W, POLICY_H = 320, 176

# Shared nominal grasp & interception geometry (used by demo.py as well)
INTERCEPT_Y = -0.075  # m: 155 mm inside the y = -0.230 open cliff edge
ABOVE_COM_M = 0.002  # m: grasp 2.0 mm toward the cap above the COM: the most the 160 mm grasp height allows (TCP 159.3 mm on the coaster)
COASTER_APPROACH_YAW_DEG = 6.0  # deg: subtle inward yaw giving >= 13.7 deg joint margin at coaster placement
HOVER_CLEAR_OVER_BOTTLE_M = 0.008  # m: fingertips 8 mm above the 54 mm tall rolling bottle while hovering
CAGE_TIP_OVER_WORKTOP_M = 0.0145  # m: fingertips 14.5 mm above worktop when TCP is at bottle equator
LIFT_TCP_OVER_BENCH_M = 0.155  # m: initial horizontal lift height (< 160 mm)
PITCH_APEX_OVER_BENCH_M = 0.190  # m: mid-air carry apex at 90 deg pitch (>= 34 mm bottle clearance over worktop)
PLACE_CLEAR_OVER_FLOOR_M = 0.0008  # m: bottle base 0.8 mm above cork coaster floor before gentle release
RETRACT_BACK_M = 0.055
CAP_CLEAR_ON_RETRACT_M = 0.050  # m: open jaws' grasp point this far above the standing bottle's cap before turning home

MAX_JAW_M = -2.0 * float(C.FINGER_OPEN)  # 0.0939 m
GRIP_OPEN = round(min(1.0, 0.088 / MAX_JAW_M), 3)  # 88 mm open jaw (> 54 mm bottle)
GRIP_PINCH = round(max(0.05, (P.BOTTLE_D - 2.0 * 0.0070) / MAX_JAW_M), 3)  # ~35 N per finger at 5000 N/m (< 40 N limit)


def rz(a_rad: float) -> np.ndarray:
    c, s = math.cos(a_rad), math.sin(a_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def ry(a_rad: float) -> np.ndarray:
    c, s = math.cos(a_rad), math.sin(a_rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def frame(closing, approach) -> np.ndarray:
    """Gripper TCP 3x3 rotation matrix from closing axis and approach axis (columns: closing, pad_width, approach)."""
    a = np.asarray(approach, dtype=float)
    a = a / np.linalg.norm(a)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c = c / np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


R_PINCH_DOWN = frame([0.0, -1.0, 0.0], [0.0, 0.0, -1.0])


def pitch_down_to_upright(s_blend: float, yaw_deg: float = COASTER_APPROACH_YAW_DEG) -> np.ndarray:
    """Smooth SO(3) rotation Rz(s * yaw) @ Ry(-s * pi/2) @ R_PINCH_DOWN for s in [0, 1].
    At s=0: top-down pinch across y. At s=1: horizontal approach along yaw_deg, swinging bottle +x (cap) straight up to +z."""
    return rz(s_blend * math.radians(yaw_deg)) @ ry(-s_blend * 0.5 * math.pi) @ R_PINCH_DOWN


def quat_xyzw_to_mat(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def solve_chained(side: str, pos, rot, q_prev: np.ndarray | None = None) -> tuple[np.ndarray, float, float, float, float]:
    """Solve IK preferring smooth continuation from ``q_prev`` with wide joint margin.

    Returns (q, pos_err_mm, rot_err_deg, min_joint_margin_deg, step_jump_deg).
    """
    seeds = []
    if q_prev is not None:
        seeds.append(np.asarray(q_prev, dtype=float))
    seeds.extend(
        np.array(s, dtype=float)
        for s in (
            READY,
            [0.3, 1.1, 1.1, -0.6, 0.0, 0.0],
            [0.5, 1.3, 1.3, -0.2, 0.0, 0.0],
            [0.4, 0.9, 0.9, 0.1, 0.0, 0.0],
            [-0.2, 1.2, 1.4, -0.4, 0.0, 0.5],
        )
    )
    best = None
    for q0 in seeds:
        q, e_pos, e_rot = ik.solve(
            np.asarray(pos, dtype=float),
            ik.matrix_to_quat_xyzw(np.asarray(rot, dtype=float)),
            q0,
            base_pos=BASES[side],
            iters=180,
        )
        margin_deg = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
        jump_deg = 0.0 if q_prev is None else float(np.degrees(np.max(np.abs(q - q_prev))))
        ok = e_pos < 0.0005 and math.degrees(e_rot) < 0.1
        # Strongly prefer continuous branch (small jump from q_prev) when q_prev is given, then wide margin
        score_key = (
            ok,
            margin_deg >= 5.0,
            -(0.02 * jump_deg) + margin_deg if q_prev is not None else margin_deg,
        )
        if best is None or score_key > best[0]:
            best = (score_key, q, 1000.0 * float(e_pos), math.degrees(float(e_rot)), margin_deg, jump_deg)
        if q_prev is not None and ok and margin_deg >= 6.0 and jump_deg <= 35.0:
            break
    assert best is not None
    return best[1], best[2], best[3], best[4], best[5]


def hand_hulls(side: str, q: np.ndarray, jaw_open_m: float) -> list:
    joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, q)}, **dict.fromkeys(C.FINGER_JOINTS, -0.5 * jaw_open_m)}
    poses = analysis.fk_link_poses_world(joints, tuple(BASES[side]))
    out = []
    for link in HAND_LINKS:
        for h in analysis.link_hulls().get(link, []):
            r, p = poses[link][:3, :3], poses[link][:3, 3]
            n_w = h.planes[:, :3] @ r.T
            out.append((link, analysis.Hull(vertices=h.vertices @ r.T + p, planes=np.c_[n_w, h.planes[:, 3] + n_w @ p])))
    return out


def hull_lowest_z(hs: list) -> float:
    return float(min(h.vertices[:, 2].min() for _, h in hs))


def hulls_min_dist(ha: list, hb: list) -> float:
    best = math.inf
    for _, a in ha:
        lo_a, hi_a = a.vertices.min(0), a.vertices.max(0)
        for _, b in hb:
            lo_b, hi_b = b.vertices.min(0), b.vertices.max(0)
            if float(np.maximum(lo_b - hi_a, lo_a - hi_b).max()) > best:
                continue
            best = min(best, float(analysis._signed(a.vertices, b).min()), float(analysis._signed(b.vertices, a).min()))
    return best


def base_to_tcp_m(spec: P.BottleSpec = P.ACTIVE_BOTTLE) -> float:
    """Distance (m) from bottle base (-z_B) to grasp TCP: 2 mm above bottle COM (116.5 mm on 229 mm bottle)."""
    return 0.5 * spec.height + ABOVE_COM_M


def retract_tcp_over_bench_m(spec: P.BottleSpec = P.ACTIVE_BOTTLE) -> float:
    """Grasp-point height (m over the bench) of the straight-up retract after release: the standing bottle's cap top
    plus CAP_CLEAR_ON_RETRACT_M (0.321 m for the 229 mm bottle), never below the 0.275 m of the shorter preset."""
    return max(0.275, P.WORKTOP_H + P.COASTER_FLOOR_H + spec.height + CAP_CLEAR_ON_RETRACT_M)


def held_bottle_lowest_z(tcp_pos: np.ndarray, alpha_rad: float, spec: P.BottleSpec = P.ACTIVE_BOTTLE) -> float:
    """World z (m) of the held bottle's lowest point at pitch angle ``alpha_rad`` in [0, pi/2]."""
    r = 0.5 * spec.diameter
    ca, sa = math.cos(alpha_rad), math.sin(alpha_rad)
    return float(tcp_pos[2] - (base_to_tcp_m(spec) * sa + r * ca))


def grasp_x_for_bottle(bottle_center_x: float, spec: P.BottleSpec = P.ACTIVE_BOTTLE) -> float:
    """World x of the pinch TCP on a horizontal bottle (+z_B cap along +x, -z_B base along -x): ABOVE_COM_M toward +x."""
    return bottle_center_x + ABOVE_COM_M


def nominal_waypoints(spec: P.BottleSpec = P.ACTIVE_BOTTLE, y_stop: float = -0.085) -> list[tuple[str, np.ndarray, np.ndarray, float, float | None]]:
    """Keyframes (name, tcp_pos, tcp_rot, jaw_open_m, held_pitch_rad_or_None) for the right arm."""
    r = 0.5 * spec.diameter
    b2tcp = base_to_tcp_m(spec)
    rot_down = pitch_down_to_upright(0.0)
    rot_up = pitch_down_to_upright(1.0)

    z_hover = P.WORKTOP_TOP_Z + spec.diameter + HOVER_CLEAR_OVER_BOTTLE_M + float(C.PAD_TIP_PAST_TCP)
    z_cage = P.WORKTOP_TOP_Z + r  # fingertips at r - 0.013 = 14.5 mm above worktop
    x_grasp = grasp_x_for_bottle(P.ROLL_LANE_X, spec)
    z_lift_low = P.TOP + 0.098  # 32.5 mm above worktop with 0 deg pitch

    z_coaster_floor = P.WORKTOP_TOP_Z + P.COASTER_FLOOR_H
    z_place = z_coaster_floor + PLACE_CLEAR_OVER_FLOOR_M + b2tcp  # TOP + 0.1593 m (< 0.160 m Check-24 limit!)
    z_over_coaster = P.TOP + PITCH_APEX_OVER_BENCH_M  # TOP + 0.190 m carry over coaster

    ready_tcp, ready_rot = ik.tcp_pose(np.array(READY, dtype=float), station.BASES["right"])
    ready_rot = np.asarray(ready_rot, dtype=float)

    pts: list[tuple[str, np.ndarray, np.ndarray, float, float | None]] = [
        ("ready_start", np.asarray(ready_tcp, dtype=float), ready_rot, 0.088, None),
        ("hover_gate", np.array([P.ROLL_LANE_X, INTERCEPT_Y, z_hover]), rot_down, 0.088, None),
        ("cage_catch", np.array([P.ROLL_LANE_X, INTERCEPT_Y, z_cage]), rot_down, 0.088, None),
        ("recenter_over_stopped", np.array([x_grasp, y_stop, z_hover]), rot_down, 0.088, None),
        ("pinch_stopped", np.array([x_grasp, y_stop, z_cage]), rot_down, spec.diameter, 0.0),
        ("lift_horizontal", np.array([x_grasp, y_stop, z_lift_low]), rot_down, spec.diameter, 0.0),
    ]
    for deg in (18, 36, 54, 72, 90):
        s_blend = deg / 90.0
        a = math.radians(deg)
        xy = (1.0 - s_blend) * np.array([x_grasp, y_stop]) + s_blend * np.array([P.COASTER_X, P.COASTER_Y])
        z_arc = (1.0 - s_blend) * z_lift_low + s_blend * (P.TOP + PITCH_APEX_OVER_BENCH_M) + 0.032 * math.sin(math.pi * s_blend)
        pts.append((f"pitch_{deg}deg", np.array([xy[0], xy[1], z_arc]), pitch_down_to_upright(s_blend), spec.diameter, a))
    pts.extend(
        [
            ("over_coaster", np.array([P.COASTER_X, P.COASTER_Y, z_over_coaster]), rot_up, spec.diameter, 0.5 * math.pi),
            ("place_on_coaster", np.array([P.COASTER_X, P.COASTER_Y, z_place]), rot_up, spec.diameter, 0.5 * math.pi),
            ("release_open", np.array([P.COASTER_X, P.COASTER_Y, z_place]), rot_up, 0.088, None),
            ("retract_straight_up", np.array([P.COASTER_X, P.COASTER_Y, P.TOP + retract_tcp_over_bench_m(spec)]), rot_up, 0.088, None),
            ("ready_end", np.asarray(ready_tcp, dtype=float), ready_rot, 0.088, None),
        ]
    )
    return pts


def project_top_cam(points_xyz: dict[str, np.ndarray]) -> dict[str, dict]:
    """Project scene landmarks into ``station.top_camera('calibrated')`` at 640x360 and 320x176."""
    cam = station.top_camera("calibrated")
    r_cam = quat_xyzw_to_mat(cam.rot_xyzw)
    o_cam = np.asarray(cam.pos, dtype=float)
    lens = cam.lens
    sx, sy = POLICY_W / lens.width, POLICY_H / lens.height

    out: dict[str, dict] = {}
    for label, pt in points_xyz.items():
        c = r_cam.T @ (np.asarray(pt, dtype=float) - o_cam)
        depth = float(-c[2])
        u = float(lens.cx + lens.fx * c[0] / depth)
        v = float(lens.cy - lens.fy * c[1] / depth)
        u_pol, v_pol = u * sx, v * sy
        in_frame = 4.0 <= u_pol <= (POLICY_W - 4.0) and 4.0 <= v_pol <= (POLICY_H - 4.0)
        out[label] = {
            "px_640x360": [round(u, 1), round(v, 1)],
            "px_320x176": [round(u_pol, 1), round(v_pol, 1)],
            "depth_m": round(depth, 3),
            "in_frame_320x176": bool(in_frame),
        }
    return out


def run_offline_checks() -> dict:
    presets_report = P.check_all_presets()
    left_ready_hulls = hand_hulls("left", np.array(READY, dtype=float), 0.088)
    right_ready_hulls = hand_hulls("right", np.array(READY, dtype=float), 0.088)

    # Sweep across plausible stop positions y_stop in [-0.110, -0.030] and both bottle presets
    plan_checks: dict[str, dict] = {}
    for preset_name, spec in P.BOTTLE_PRESETS.items():
        worst_ik_mm = 0.0
        worst_ik_deg = 0.0
        min_margin_deg = math.inf
        max_step_jump_deg = 0.0
        min_hand_over_worktop_mm = math.inf
        min_bottle_over_worktop_mm = math.inf
        min_bottle_over_coaster_rim_mm = math.inf
        min_arms_apart_mm = math.inf
        max_grasp_height_over_bench_mm = 0.0
        stage_rows = []

        for y_stop in (-0.115, -0.085, -0.060):
            q_prev = np.array(READY, dtype=float)
            for name, pos, rot, jaw_m, held_alpha in nominal_waypoints(spec, y_stop=y_stop):
                q, e_mm, e_deg, margin_deg, jump_deg = solve_chained("right", pos, rot, q_prev)
                q_prev = q
                worst_ik_mm = max(worst_ik_mm, e_mm)
                worst_ik_deg = max(worst_ik_deg, e_deg)
                min_margin_deg = min(min_margin_deg, margin_deg)
                if name != "ready_start":
                    max_step_jump_deg = max(max_step_jump_deg, jump_deg)

                hs_r = hand_hulls("right", q, jaw_m)
                hand_clear_mm = 1000.0 * (hull_lowest_z(hs_r) - P.WORKTOP_TOP_Z)
                arms_mm = 1000.0 * hulls_min_dist(left_ready_hulls, hs_r)
                min_hand_over_worktop_mm = min(min_hand_over_worktop_mm, hand_clear_mm)
                min_arms_apart_mm = min(min_arms_apart_mm, arms_mm)

                if name in ("pinch_stopped", "place_on_coaster"):
                    max_grasp_height_over_bench_mm = max(max_grasp_height_over_bench_mm, 1000.0 * (float(pos[2]) - P.TOP))

                if held_alpha is not None and name not in ("pinch_stopped", "place_on_coaster"):
                    bz = held_bottle_lowest_z(pos, held_alpha, spec)
                    min_bottle_over_worktop_mm = min(min_bottle_over_worktop_mm, 1000.0 * (bz - P.WORKTOP_TOP_Z))
                    if name.startswith("pitch_") or name == "over_coaster":
                        min_bottle_over_coaster_rim_mm = min(
                            min_bottle_over_coaster_rim_mm,
                            1000.0 * (bz - (P.WORKTOP_TOP_Z + P.COASTER_TOTAL_H)),
                        )
                if y_stop == -0.070:
                    stage_rows.append(
                        {
                            "stage": name,
                            "tcp_mm_over_bench": round(1000.0 * (float(pos[2]) - P.TOP), 1),
                            "ik_mm": round(e_mm, 3),
                            "ik_deg": round(e_deg, 3),
                            "margin_deg": round(margin_deg, 2),
                            "hand_over_worktop_mm": round(hand_clear_mm, 1),
                        }
                    )

        assert worst_ik_mm <= 0.5, f"{preset_name}: IK position error {worst_ik_mm:.3f} mm > 0.5 mm"
        assert min_margin_deg >= 5.0, f"{preset_name}: min joint margin {min_margin_deg:.2f} deg < 5.0 deg"
        assert min_hand_over_worktop_mm >= 8.0, f"{preset_name}: hand hull {min_hand_over_worktop_mm:.1f} mm over worktop"
        assert min_bottle_over_worktop_mm >= 15.0, f"{preset_name}: pitched bottle {min_bottle_over_worktop_mm:.1f} mm over worktop"
        assert max_grasp_height_over_bench_mm <= 160.0, f"{preset_name}: grasp TCP {max_grasp_height_over_bench_mm:.1f} mm > 160 mm"
        assert P.idle_coaster_clearance_m(spec) >= 0.005, f"{preset_name}: coaster too close to roll lane"

        plan_checks[preset_name] = {
            "worst_ik_mm": round(worst_ik_mm, 3),
            "worst_ik_deg": round(worst_ik_deg, 3),
            "min_joint_margin_deg": round(min_margin_deg, 2),
            "max_step_jump_deg": round(max_step_jump_deg, 1),
            "min_hand_over_worktop_mm": round(min_hand_over_worktop_mm, 1),
            "min_pitched_bottle_over_worktop_mm": round(min_bottle_over_worktop_mm, 1),
            "min_pitched_bottle_over_coaster_rim_mm": round(min_bottle_over_coaster_rim_mm, 1),
            "min_arms_apart_mm": round(min_arms_apart_mm, 1),
            "max_grasp_tcp_over_bench_mm": round(max_grasp_height_over_bench_mm, 1),
            "nominal_stages_y_minus_70mm": stage_rows,
        }

    b_pos0, b_rot0 = P.bottle_start_pose()
    landmarks = {
        "bottle_center": b_pos0,
        "bottle_cap_end": b_pos0 + 0.5 * P.BOTTLE_H * b_rot0[:, 2],
        "bottle_base_end": b_pos0 - 0.5 * P.BOTTLE_H * b_rot0[:, 2],
        "drainboard_foot": np.array([P.ROLL_LANE_X, P.DRAIN_FOOT_Y, P.WORKTOP_TOP_Z]),
        "coaster_center": np.array([P.COASTER_X, P.COASTER_Y, P.WORKTOP_TOP_Z + P.COASTER_FLOOR_H]),
        "cliff_stripe_mid": np.array([P.ROLL_LANE_X, P.WORKTOP_MIN_Y + 0.006, P.WORKTOP_TOP_Z]),
        "catch_mat_mid": np.array([P.CATCH_MAT_X, P.CATCH_MAT_Y, P.TOP + P.CATCH_MAT_H]),
    }
    cam_proj = project_top_cam(landmarks)
    assert all(v["in_frame_320x176"] for v in cam_proj.values()), f"landmark out of 320x176 frame: {cam_proj}"

    # Verify ready arms stay well outside the central counter view (left y >= +0.26, right y <= -0.26 in base region)
    left_min_y = float(min(h.vertices[:, 1].min() for _, h in left_ready_hulls))
    right_max_y = float(max(h.vertices[:, 1].max() for _, h in right_ready_hulls))

    return {
        "presets_kinematics": presets_report,
        "ik_and_hull_checks": plan_checks,
        "top_cam_projection": cam_proj,
        "ready_arm_y_bounds_m": {"left_arm_min_y": round(left_min_y, 3), "right_arm_max_y": round(right_max_y, 3)},
    }


def render_start_frames(out_dir: Path) -> dict:
    """Reset the simulator scene at seed 0 and write start_top.png, start_top_policy.png, and wrist start PNGs."""
    import cv2
    import demo_kit as kit
    from inspect_robots import Scene

    out_dir.mkdir(parents=True, exist_ok=True)
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    obs = emb.reset(Scene(id="g1_start", instruction=m.INSTRUCTION), seed=0)

    # Read live simulator state right after reset settle to confirm zero start roll drift
    m.mk._read_bodies(m.mk.MECH.environment)
    p_b, _ = m.mk.pose("bottle")
    lin_v, ang_v = m.mk.velocity("bottle")
    expected_b, _ = P.bottle_start_pose()
    drift_mm = 1000.0 * float(np.linalg.norm(p_b[:2] - expected_b[:2]))

    top_rgb = obs.images["top_cam"]
    top_pol = cv2.resize(top_rgb, (POLICY_W, POLICY_H), interpolation=cv2.INTER_AREA)

    files = {
        "start_top.png": top_rgb,
        "start_top_policy.png": top_pol,
        "start_left_wrist.png": obs.images["left_cam"],
        "start_right_wrist.png": obs.images["right_cam"],
    }
    for fname, rgb in files.items():
        cv2.imwrite(str(out_dir / fname), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(HERE / fname), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

    proj = project_top_cam(
        {
            "bottle_center": p_b,
            "bottle_cap_end": p_b + 0.5 * P.BOTTLE_H * m.mk.pose("bottle")[1][:, 2],
            "coaster_center": np.array([P.COASTER_X, P.COASTER_Y, P.WORKTOP_TOP_Z + P.COASTER_FLOOR_H]),
            "cliff_stripe_mid": np.array([P.ROLL_LANE_X, P.WORKTOP_MIN_Y + 0.006, P.WORKTOP_TOP_Z]),
            "worktop_clear": np.array([0.310, 0.030, P.WORKTOP_TOP_Z]),
        }
    )
    pixel_luma = {}
    for k, info in proj.items():
        u, v = (int(round(c)) for c in info["px_320x176"])
        u = max(0, min(POLICY_W - 1, u))
        v = max(0, min(POLICY_H - 1, v))
        rgb_px = [int(x) for x in top_pol[v, u]]
        luma = round(0.299 * rgb_px[0] + 0.587 * rgb_px[1] + 0.114 * rgb_px[2], 1)
        pixel_luma[k] = {"uv_320x176": [u, v], "rgb": rgb_px, "luma": luma}

    summary = {
        "frame0_bottle_pos_m": [round(float(x), 4) for x in p_b],
        "frame0_drift_from_mark_mm": round(drift_mm, 3),
        "frame0_lin_speed_mps": round(float(np.linalg.norm(lin_v)), 5),
        "frame0_ang_speed_radps": round(float(np.linalg.norm(ang_v)), 5),
        "policy_320x176_samples": pixel_luma,
        "written": list(files),
    }
    (out_dir / "g1_start_check.json").write_text(json.dumps(summary, indent=2))
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--render-start", type=Path, default=None, help="Render frame-0 start PNGs in simulator")
    args = parser.parse_args()

    offline = run_offline_checks()
    if args.render_start is not None:
        offline["render_start"] = render_start_frames(args.render_start)
    print(json.dumps(offline, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
