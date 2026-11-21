#!/usr/bin/env python3
"""Offline reach, joint-margin, collision-clearance, and top-camera visibility checks for holesaw_arbor,
plus ``--frame0 <dir>`` in the simulator for the three start pictures and a 2 s idle stability check.

    python mech/tasks/holesaw_arbor/checks.py
    scripts/rta simpy <id> mech/tasks/holesaw_arbor/checks.py --frame0 /workspace/robocurve/outputs/<id>
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
from yam_arena import analysis, constants as C, demo as yam_demo, ik, station
from yam_arena.constants import READY
from yam_arena.task_file import TABLE_TOP

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import parts as P  # noqa: E402

TOP = TABLE_TOP["station"]
BASES = {side: np.array(station.BASES[side], dtype=float) for side in ("left", "right")}
HAND_LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")
MAX_JAW = 2.0 * abs(float(C.FINGER_OPEN))  # 0.0939 m
IK_MM, IK_DEG, MARGIN_DEG, SEPARATION_M = 0.5, 0.6, 5.0, 0.020

# Grasp geometry shared with demo.py
SAW_GRIP_FROM_BOT = 0.0256  # m above hole_saw underside (22.0 mm below guarded rim, fingertips 12.8 mm above underside)
SAW_CARRY_Z = 0.1315  # m cup underside height when carried over the 126.6 mm pilot nose (TCP at 157.1 mm <= 160 mm!)
COLLAR_PITCH_RAD = math.radians(14.0)  # right hand approaches from -y pitched 14 deg down
COLLAR_GRIP_Z_OFFSET = -0.0065  # m: upper 14 mm of the 40 mm-tall finger pads grip collar cleanly while clearing seated hole_saw!
COLLAR_HOLD_BACK = 0.0150  # m along -approach so knuckle stays outside hole_saw's radius while pads span collar


def frame(closing, approach) -> np.ndarray:
    a = np.asarray(approach, dtype=float)
    a = a / np.linalg.norm(a)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c = c / np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


COLLAR_ROT = frame((1.0, 0.0, 0.0), (0.0, math.cos(COLLAR_PITCH_RAD), -math.sin(COLLAR_PITCH_RAD)))


def collar_tcp(collar_z: float, back: float = 0.0, dz: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Right-hand TCP on the Ø 42 mm collar at travel ``collar_z`` (0..0.008 m)."""
    mid = np.array([P.X_ARBOR, P.Y_ARBOR, TOP + P.Z_COLLAR_LOW + collar_z + COLLAR_GRIP_Z_OFFSET + dz], dtype=float)
    return mid - (COLLAR_HOLD_BACK + back) * COLLAR_ROT[:, 2], COLLAR_ROT


# Left-arm wrist-roll window: over the arbor j1 = -54.9 deg and j6 = j1 - yaw (inside [-120, +120] deg).
# Using yaw in [-160, +50] deg keeps j6 inside [-104.9, +105.1] deg (>= 14.9 deg margin on every joint!).
# Decreasing yaw rotates the gripper CLOCKWISE seen from above (+z); each 210 deg spin splits into two 105 deg moves.
SAW_YAW_START = math.radians(50.0)     # j6 = -81.9 deg on mat, -104.9 deg over arbor
SAW_YAW_MID = math.radians(-55.0)      # midpoint (105 deg CW) so SLERP shortest-turn stays strictly CW
SAW_YAW_END = math.radians(-160.0)     # 210 deg CW per stroke (stroke 2 reaches 420 deg -> seats gently on +415 deg stop)
SAW_YAW_ALIGN = math.radians(-100.0)   # 55 deg CCW back-off from the -155 deg (+415 deg) stop -> +360.0 deg aligned!


def saw_tcp(xy: tuple[float, float], z_bot_over_top: float, yaw: float) -> tuple[np.ndarray, np.ndarray]:
    pos = np.array([xy[0], xy[1], TOP + z_bot_over_top + SAW_GRIP_FROM_BOT], dtype=float)
    return pos, yam_demo.grip_rotation(yaw)


def poses() -> dict[str, tuple[str, np.ndarray, np.ndarray, float]]:
    """Every key pose of demo.py: name -> (side, tcp_pos, tcp_rot, jaw_opening_m)."""
    mat_xy = (P.X_MAT, P.Y_MAT)
    arb_xy = (P.X_ARBOR, P.Y_ARBOR)
    z_mat = P.MAT_SIZE[2]
    z_entry = P.Z_ENTRY
    z_stop = P.Z_SHOULDER
    z_align = P.Z_SHOULDER + P.gap_for_yaw_deg(P.ALIGN_DEG)
    return {
        "r: approach collar up": ("right", *collar_tcp(P.PIN_STROKE, back=0.075), MAX_JAW),
        "r: around collar up": ("right", *collar_tcp(P.PIN_STROKE), MAX_JAW),
        "r: grip collar up": ("right", *collar_tcp(P.PIN_STROKE), P.COLLAR_D),
        "r: pull collar down": ("right", *collar_tcp(0.0, dz=-0.001), P.COLLAR_D),
        "r: park clear": ("right", *collar_tcp(0.0, back=0.125, dz=0.020), MAX_JAW),
        "r: around collar down": ("right", *collar_tcp(0.0), MAX_JAW),
        "r: grip collar down": ("right", *collar_tcp(0.0), P.COLLAR_D),
        "r: push collar locked": ("right", *collar_tcp(P.PIN_STROKE, dz=0.0005), P.COLLAR_D),
        "r: release locked": ("right", *collar_tcp(P.PIN_STROKE, back=0.095), MAX_JAW),
        "l: over mat": ("left", *saw_tcp(mat_xy, z_mat + 0.055, SAW_YAW_START), MAX_JAW),
        "l: around saw on mat": ("left", *saw_tcp(mat_xy, z_mat, SAW_YAW_START), MAX_JAW),
        "l: grip saw on mat": ("left", *saw_tcp(mat_xy, z_mat, SAW_YAW_START), P.SAW_OD),
        "l: lift saw carry": ("left", *saw_tcp(mat_xy, SAW_CARRY_Z, SAW_YAW_START), P.SAW_OD),
        "l: over arbor pilot": ("left", *saw_tcp(arb_xy, SAW_CARRY_Z, SAW_YAW_START), P.SAW_OD),
        "l: place thread entry": ("left", *saw_tcp(arb_xy, z_entry - 0.0005, SAW_YAW_START), P.SAW_OD),
        "l: turn 1 mid (+105 deg CW)": ("left", *saw_tcp(arb_xy, P.Z_SHOULDER + P.gap_for_yaw_deg(105.0), SAW_YAW_MID), P.SAW_OD),
        "l: turn 1 end (+210 deg CW)": ("left", *saw_tcp(arb_xy, P.Z_SHOULDER + P.gap_for_yaw_deg(210.0), SAW_YAW_END), P.SAW_OD),
        "l: regrasp open": ("left", *saw_tcp(arb_xy, P.Z_SHOULDER + P.gap_for_yaw_deg(210.0) + 0.004, SAW_YAW_END), MAX_JAW),
        "l: regrasp mid": ("left", *saw_tcp(arb_xy, P.Z_SHOULDER + P.gap_for_yaw_deg(210.0) + 0.004, SAW_YAW_MID), MAX_JAW),
        "l: regrasp cocked": ("left", *saw_tcp(arb_xy, P.Z_SHOULDER + P.gap_for_yaw_deg(210.0), SAW_YAW_START), MAX_JAW),
        "l: turn 2 mid (+315 deg CW)": ("left", *saw_tcp(arb_xy, P.Z_SHOULDER + P.gap_for_yaw_deg(315.0), SAW_YAW_MID), P.SAW_OD),
        "l: turn 2 shoulder stop (+415 deg)": ("left", *saw_tcp(arb_xy, z_stop, SAW_YAW_END), P.SAW_OD),
        "l: back off 55 deg (+360 deg)": ("left", *saw_tcp(arb_xy, z_align, SAW_YAW_ALIGN), P.SAW_OD),
        "l: open after align": ("left", *saw_tcp(arb_xy, z_align + 0.032, SAW_YAW_ALIGN), MAX_JAW),
    }


# Simultaneous hand pairs during the plan (while left works on arbor, right stays at park clear; vice versa at ready)
TOGETHER = tuple(
    (l_step, "r: park clear")
    for l_step in (
        "l: over mat",
        "l: around saw on mat",
        "l: lift saw carry",
        "l: over arbor pilot",
        "l: place thread entry",
        "l: turn 1 mid (+105 deg CW)",
        "l: turn 1 end (+210 deg CW)",
        "l: regrasp open",
        "l: regrasp mid",
        "l: regrasp cocked",
        "l: turn 2 mid (+315 deg CW)",
        "l: turn 2 shoulder stop (+415 deg)",
        "l: back off 55 deg (+360 deg)",
        "l: open after align",
    )
)


def solve_chained(side: str, seq: list[tuple[str, np.ndarray, np.ndarray]]) -> dict[str, tuple]:
    """Solve a sequence of poses for ``side`` starting from READY, chaining ``q`` just as demo_kit.Arm does."""
    q_prev = np.array(READY, dtype=float)
    out = {}
    for name, pos, rot in seq:
        quat = ik.matrix_to_quat_xyzw(rot)
        q, pe, re = ik.solve(pos, quat, q_prev, base_pos=BASES[side], iters=200)
        margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
        out[name] = (q, float(pe * 1000.0), float(math.degrees(re)), margin)
        q_prev = q
    return out


def hand_hulls(side: str, q: np.ndarray, opening: float) -> list:
    half = -0.5 * max(0.0, min(MAX_JAW, opening))
    joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, q)}, **dict.fromkeys(C.FINGER_JOINTS, half)}
    poses_w = analysis.fk_link_poses_world(joints, tuple(BASES[side]))
    out = []
    for link in HAND_LINKS:
        for h in analysis.link_hulls().get(link, []):
            r, p = poses_w[link][:3, :3], poses_w[link][:3, 3]
            n_w = h.planes[:, :3] @ r.T
            out.append((link, analysis.Hull(vertices=h.vertices @ r.T + p, planes=np.c_[n_w, h.planes[:, 3] + n_w @ p])))
    return out


def box_hull(centre, rotation, size) -> analysis.Hull:
    centre, rotation, half = np.asarray(centre, float), np.asarray(rotation, float), np.asarray(size, float) / 2
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
    vertices = corners @ rotation.T + centre
    planes = []
    for i in range(3):
        for s in (1.0, -1.0):
            n = s * rotation[:, i]
            planes.append([*n, float(n @ centre + half[i])])
    return analysis.Hull(vertices=vertices, planes=np.array(planes))


def gap(hulls_a, hulls_b) -> float:
    """Exact minimum separating-plane clearance (m) between two convex-hull lists with AABB pruning."""
    boxes_a = [(a, a.vertices.min(0), a.vertices.max(0)) for _, a in hulls_a]
    boxes_b = [(b, b.vertices.min(0), b.vertices.max(0)) for _, b in hulls_b]
    best = math.inf
    for a, lo_a, hi_a in boxes_a:
        for b, lo_b, hi_b in boxes_b:
            lb = float(np.linalg.norm(np.maximum(np.maximum(lo_a - hi_b, lo_b - hi_a), 0.0)))
            if lb >= best:
                continue
            sep_ab = float((a.vertices @ b.planes[:, :3].T - b.planes[:, 3]).min(0).max())
            sep_ba = float((b.vertices @ a.planes[:, :3].T - a.planes[:, 3]).min(0).max())
            d = max(sep_ab, sep_ba)
            if d < best:
                best = d
    return best


def vise_and_bench_hulls() -> dict[str, list]:
    vise_box = box_hull(
        (P.X_ARBOR + 0.016, P.Y_ARBOR, TOP + P.VISE_TOP / 2),
        np.eye(3),
        (P.VISE_BASE_L, P.VISE_JAW_W, P.VISE_TOP),
    )
    bench = box_hull((0.25, 0.0, TOP - 0.030), np.eye(3), (1.0, 1.2, 0.060))
    return {"vise": [("vise", vise_box)], "bench": [("bench", bench)]}


def top_cam_pin_visibility() -> dict:
    """Verify geometrically that station's calibrated top_cam sees both (0, +/-12.7 mm) pin holes clear of the cup rim."""
    cam = station.top_camera("calibrated")
    cam_pos = np.array(cam.pos, dtype=float)
    z_floor = TOP + P.Z_SHOULDER + P.gap_for_yaw_deg(P.ALIGN_DEG) + P.PLATE_T
    z_rim = z_floor + (P.SAW_H - P.PLATE_T)
    r_guard_in = P.SAW_OD / 2 - P.SAW_WALL_T - 0.0006  # inner lip of PVC rim guard
    holes = {}
    for label, sy in (("+y_hole", 1.0), ("-y_hole", -1.0)):
        # Check the near-camera edge (-x edge) of the 7.8 mm pin hole: (X_ARBOR - 3.9 mm, Y_ARBOR + sy * 12.7 mm, z_floor)
        pt = np.array([P.X_ARBOR - P.HOLE_D / 2, P.Y_ARBOR + sy * (P.PIN_BCD / 2), z_floor])
        # Where the ray from pt to cam_pos pierces the rim plane z = z_rim
        u = (z_rim - pt[2]) / (cam_pos[2] - pt[2])
        pierce = pt + u * (cam_pos - pt)
        radial = math.hypot(pierce[0] - P.X_ARBOR, pierce[1] - P.Y_ARBOR)
        clear_mm = round(1000.0 * (r_guard_in - radial), 2)
        holes[label] = {"ray_radius_at_rim_mm": round(1000.0 * radial, 2), "rim_inner_radius_mm": round(1000.0 * r_guard_in, 2), "clear_of_rim_mm": clear_mm}
    return holes


def run_offline() -> dict:
    all_p = poses()
    r_seq = [(k, v[1], v[2]) for k, v in all_p.items() if v[0] == "right"]
    l_seq = [(k, v[1], v[2]) for k, v in all_p.items() if v[0] == "left"]
    solved = {**solve_chained("right", r_seq), **solve_chained("left", l_seq)}

    obs = vise_and_bench_hulls()
    hulls_by_name = {}
    rows = {}
    worst_mm, worst_deg, min_margin, max_grasp_h_mm = 0.0, 0.0, math.inf, 0.0
    min_vise_mm, min_bench_mm = math.inf, math.inf
    ok = True

    for name, (side, pos, _rot, opening) in all_p.items():
        q, mm, deg, margin = solved[name]
        hh = hand_hulls(side, q, opening)
        hulls_by_name[name] = hh
        c_vise = 1000.0 * gap(hh, obs["vise"])
        c_bench = 1000.0 * gap(hh, obs["bench"])
        grasp_h_mm = 1000.0 * (float(pos[2]) - TOP)
        worst_mm = max(worst_mm, mm)
        worst_deg = max(worst_deg, deg)
        min_margin = min(min_margin, margin)
        if opening < MAX_JAW - 1e-4:
            max_grasp_h_mm = max(max_grasp_h_mm, grasp_h_mm)
        min_vise_mm = min(min_vise_mm, c_vise)
        min_bench_mm = min(min_bench_mm, c_bench)
        pose_ok = mm <= IK_MM and deg <= IK_DEG and margin >= MARGIN_DEG and c_vise > 3.0 and c_bench > 3.0 and (opening >= MAX_JAW - 1e-4 or grasp_h_mm <= 160.0)
        ok &= pose_ok
        rows[name] = {
            "ik_mm": round(mm, 3),
            "ik_deg": round(deg, 3),
            "margin_deg": round(margin, 2),
            "grasp_z_mm": round(grasp_h_mm, 1),
            "vise_clear_mm": round(c_vise, 1),
            "bench_clear_mm": round(c_bench, 1),
            "ok": pose_ok,
        }

    min_sep_mm = math.inf
    for l_name, r_name in TOGETHER:
        sep = 1000.0 * gap(hulls_by_name[l_name], hulls_by_name[r_name])
        min_sep_mm = min(min_sep_mm, sep)
    ready_r = hand_hulls("right", np.array(READY, dtype=float), MAX_JAW)
    for l_name, _ in TOGETHER:
        min_sep_mm = min(min_sep_mm, 1000.0 * gap(hulls_by_name[l_name], ready_r))
    ok &= min_sep_mm >= 1000.0 * SEPARATION_M

    vis = top_cam_pin_visibility()
    ok &= all(v["clear_of_rim_mm"] > 1.5 for v in vis.values())

    return {
        "pass": ok,
        "pose_count": len(rows),
        "worst_ik_mm": round(worst_mm, 3),
        "worst_ik_deg": round(worst_deg, 3),
        "smallest_joint_margin_deg": round(min_margin, 2),
        "max_grasp_height_mm": round(max_grasp_h_mm, 1),
        "min_vise_clear_mm": round(min_vise_mm, 1),
        "min_bench_clear_mm": round(min_bench_mm, 1),
        "min_hand_separation_mm": round(min_sep_mm, 1),
        "top_cam_pin_visibility": vis,
        "poses": rows,
    }


def run_frame0(out_dir: Path) -> dict:
    import cv2
    import demo_kit as kit
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack

    out_dir.mkdir(parents=True, exist_ok=True)
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    obs = embodiment.reset(Scene(id="frame0", instruction=m.INSTRUCTION), seed=0)

    imgs = obs.images
    for src, dst in (("top_cam", "start_top.png"), ("left_cam", "start_left_wrist.png"), ("right_cam", "start_right_wrist.png")):
        cv2.imwrite(str(out_dir / dst), cv2.cvtColor(np.asarray(imgs[src]), cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(out_dir / "start_top_policy.png"), cv2.cvtColor(kit.policy_picture(np.asarray(imgs["top_cam"])), cv2.COLOR_RGB2BGR))

    hold_act = Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))
    p0_saw, p0_col = None, None
    max_saw_drift, max_col_drift = 0.0, 0.0
    res = None
    for _ in range(40):  # 2.0 s idle at 20 Hz
        res = embodiment.step(hold_act)
        ps = m.saw_pose()[0].copy()
        pc = m.mk.pose("collar")[0].copy()
        if p0_saw is None:
            p0_saw, p0_col = ps, pc
        max_saw_drift = max(max_saw_drift, float(np.linalg.norm(ps - p0_saw)))
        max_col_drift = max(max_col_drift, float(np.linalg.norm(pc - p0_col)))

    assert res is not None
    rep = m.report()
    idle_ok = (
        float(res.info.get("score", -1.0)) == 0.0
        and rep["zero"] is None
        and not rep["ever_retracted"]
        and rep["collar_z_mm"] >= 7.5
        and max_saw_drift < 0.001
        and max_col_drift < 0.001
    )
    summary = {
        "idle_pass": idle_ok,
        "idle_score": float(res.info.get("score", -1.0)),
        "saw_drift_mm": round(1000.0 * max_saw_drift, 3),
        "collar_drift_mm": round(1000.0 * max_col_drift, 3),
        "report": rep,
    }
    (out_dir / "frame0.json").write_text(json.dumps(summary, indent=2, default=str))
    print("FRAME0 " + json.dumps(summary, default=str), flush=True)
    return summary


def main() -> int:
    offline = run_offline()
    print("OFFLINE " + json.dumps({k: v for k, v in offline.items() if k != "poses"}, default=str), flush=True)
    if not offline["pass"]:
        for k, r in offline["poses"].items():
            if not r["ok"]:
                print(f"  FAIL {k}: {r}", flush=True)
        return 1
    if "--frame0" in sys.argv[1:]:
        idx = sys.argv.index("--frame0")
        out_dir = Path(sys.argv[idx + 1])
        offline_path = out_dir / "offline_checks.json"
        out_dir.mkdir(parents=True, exist_ok=True)
        offline_path.write_text(json.dumps(offline, indent=2))
        f0 = run_frame0(out_dir)
        return 0 if f0["idle_pass"] else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
