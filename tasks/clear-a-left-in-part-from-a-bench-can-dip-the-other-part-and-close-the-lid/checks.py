#!/usr/bin/env python3
"""Reach, finger/collar clearance, >=180 mm wrist separation, camera framing, and 2 s idle checks for ``bench_can`` (G1).

    scripts/rta simpy <id> mech/tasks/bench_can/checks.py [--frame0 /workspace/robocurve/outputs/<id>]
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
from yam_arena import analysis, constants as C, ik, station
from yam_arena.constants import READY
from yam_arena.task_file import TABLE_TOP
from yam_arena.urdf import quat_xyzw_to_matrix

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402
import scene  # noqa: E402

TOP = TABLE_TOP["station"]
BASES = {side: np.array(station.BASES[side], dtype=float) for side in ("left", "right")}
ARM_LINKS = ("link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
HAND_LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")
MAX_JAW = 2.0 * abs(float(C.FINGER_OPEN))
IK_MM, IK_DEG, MARGIN_DEG, MIN_WRIST_SEP_M, MIN_HULL_GAP_M = 0.5, 0.6, 6.0, 0.180, 0.020


def frame(closing, approach) -> np.ndarray:
    a = np.asarray(approach, dtype=float) / np.linalg.norm(approach)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


# --- Grasp orientations and helper coordinates (shared with demo.py) -----------------------------------------------------
RIGHT_DOWN = frame((0.0, -1.0, 0.0), (0.0, 0.0, -1.0))  # straight down, closing along y across Ø 62 mm bearing (flip=False branch)
BRG_OPEN_M = 0.084  # m approach/release opening around Ø 62 mm bearing (inside 118 mm collar bore!)
BRG_PINCH_M = 0.052  # m commanded squeeze on Ø 62 mm steel bearing (~25 N per finger < 40 N limit)

_U_D = P.handle_unit_vector()
# Right arm braces front horizontal D-handle strap vertically (one finger over, one under the 14 mm strap)
RIGHT_BRACE_ROT = frame((0.0, 0.0, 1.0), -0.88 * _U_D + np.array([0.0, 0.0, -0.47], dtype=float))
HANDLE_OPEN_M = 0.048
HANDLE_PINCH_M = 0.000  # firm vertical pinch (~32 N < 40 N limit) on 14 mm welded strap

_U_H, _T_H = P.hinge_unit_vectors()
# Step A: pinch Stave 2 (+y/+x skirt, 104 mm lever arm) with gripper rotating along _T_H from 172 deg to 138 deg (z_TCP = 156.5 mm <= 160 mm) and release;
# Step B: push the propped 138 deg lid's outer back from behind (at r = 58 mm, z_TCP <= 148 mm) over 90 deg TDC to ~46 deg;
# Step C: press the lid roof flat to 0 deg from above with the level open hand (no finger ever between lid and rim).
LEFT_STAVE2_ROT0 = frame((-0.677, -0.736, 0.0), np.array([0.25, -0.65, -0.71], dtype=float))
LEFT_PUSH_ROT = frame(_T_H, np.array([0.35, -0.65, -0.67], dtype=float))
LEFT_PRESS_ROT = frame(-_U_H, (0.0, 0.0, -1.0))
LEFT_LID_ROT = frame((-0.905, -0.424, 0.0), np.array([0.25, -0.65, -0.71], dtype=float))  # used by unbraced trial
SKIRT_OPEN_M = 0.036
SKIRT_PINCH_M = 0.0
TIP_BELOW_TCP_M = 0.013


def lid_stave2_pose(lid_deg: float) -> tuple[np.ndarray, np.ndarray]:
    """(world centre, rotated gripper frame) for Stave 2 (z = 104.7 mm at 172 deg, 156.5 mm at 138 deg)."""
    hinge = scene.hinge_world_pos(TOP)
    c_pos = scene.can_start_pos(TOP)
    ang2 = 2.0 * math.pi * 1 / 16
    skirt_r = P.LID_R - 0.0015
    sh2_in_lid = np.array([skirt_r * math.cos(ang2), skirt_r * math.sin(ang2), P.LID_ROOF_T - 0.5 * P.LID_SKIRT_H], dtype=float)
    closed_world = c_pos + np.array([0.0, 0.0, P.CAN_H], dtype=float) + sh2_in_lid
    pos = hinge + mk.axis_angle(_T_H, math.radians(lid_deg)) @ (closed_world - hinge)
    rot = mk.axis_angle(_T_H, math.radians(lid_deg - 172.0)) @ LEFT_STAVE2_ROT0
    return pos, rot


def lid_back_push_tcp(lid_deg: float, radius_from_hinge: float = 0.060, back_clear: float = 0.0) -> np.ndarray:
    """TCP behind the propped lid's outer face at ``radius_from_hinge``, pushing across ``90 deg`` TDC to ``48 deg``."""
    hinge = scene.hinge_world_pos(TOP)
    rot = mk.axis_angle(_T_H, math.radians(lid_deg))
    pt = hinge + rot @ (radius_from_hinge * (-_U_H) + np.array([0.0, 0.0, P.LID_ROOF_T + back_clear]))
    return pt - 0.012 * LEFT_PUSH_ROT[:, 2]


def lid_skirt_point(lid_deg: float) -> np.ndarray:
    """Exact world centre of Stave 3 on the (+x, +y) flank of ``lid``'s rolled skirt (used in unbraced pull trial)."""
    hinge = scene.hinge_world_pos(TOP)
    c_pos = scene.can_start_pos(TOP)
    ang3 = 2.0 * math.pi * 2 / 16
    skirt_r = P.LID_R - 0.0015
    sh3_in_lid = np.array([skirt_r * math.cos(ang3), skirt_r * math.sin(ang3), P.LID_ROOF_T - 0.5 * P.LID_SKIRT_H], dtype=float)
    closed_world = c_pos + np.array([0.0, 0.0, P.CAN_H], dtype=float) + sh3_in_lid
    return hinge + mk.axis_angle(_T_H, math.radians(lid_deg)) @ (closed_world - hinge)


def lid_press_point(z_tip_above_roof: float = 0.0) -> np.ndarray:
    """TCP for level open-hand press on the (+x, +y) half of ``lid``'s roof (both MAX_JAW fingertips land 12 mm inside the
    82.5 mm circular lid roof: hypot(52, 47) = 70.1 mm < 82.5 mm; wrists > 320 mm apart)."""
    c = scene.can_start_pos(TOP)
    z_roof = TOP + P.CAN_H + P.LID_ROOF_T
    return np.array([c[0], c[1], z_roof + TIP_BELOW_TCP_M + z_tip_above_roof], dtype=float) - 0.052 * _U_D


def handle_grip(back: float = 0.0) -> np.ndarray:
    return scene.handle_world_pos(TOP) + back * _U_D


def poses() -> dict[str, tuple[str, np.ndarray, np.ndarray, float]]:
    cx, cy = P.CAN_XY
    mx, my = P.MAT_XY
    b1x, b1y = P.TRAY_SLOT_BACK_XY
    b2x, b2y = P.TRAY_SLOT_FRONT_XY
    z_screen_pick = TOP + P.DASH_REST_Z + 0.5 * P.BRG_W + 0.002  # 0.0928 m above bench
    z_plunge = z_screen_pick - 0.0145  # 14.5 mm downward plunge (>= 12 mm threshold)
    z_mat_pick = TOP + P.MAT_T + 0.5 * P.BRG_W + 0.002
    z_tray_put = TOP + P.TRAY_FLOOR + 0.5 * P.BRG_W + 0.003
    z_carry = TOP + 0.125  # 125 mm <= 160 mm ceiling!

    return {
        "r: hover screen bearing": ("right", np.array([cx, cy, z_carry]), RIGHT_DOWN, BRG_OPEN_M),
        "r: pinch screen bearing": ("right", np.array([cx, cy, z_screen_pick]), RIGHT_DOWN, BRG_PINCH_M),
        "r: hover tray back": ("right", np.array([b1x, b1y, z_carry]), RIGHT_DOWN, BRG_PINCH_M),
        "r: place tray back": ("right", np.array([b1x, b1y, z_tray_put]), RIGHT_DOWN, BRG_OPEN_M),
        "r: hover mat bearing": ("right", np.array([mx, my, z_carry]), RIGHT_DOWN, BRG_OPEN_M),
        "r: pinch mat bearing": ("right", np.array([mx, my, z_mat_pick]), RIGHT_DOWN, BRG_PINCH_M),
        "r: plunge on dasher": ("right", np.array([cx, cy, z_plunge]), RIGHT_DOWN, BRG_PINCH_M),
        "r: hover tray front": ("right", np.array([b2x, b2y, z_carry]), RIGHT_DOWN, BRG_PINCH_M),
        "r: place tray front": ("right", np.array([b2x, b2y, z_tray_put]), RIGHT_DOWN, BRG_OPEN_M),
        "r: approach d_handle": ("right", handle_grip(back=0.055), RIGHT_BRACE_ROT, HANDLE_OPEN_M),
        "r: brace d_handle": ("right", handle_grip(back=0.002), RIGHT_BRACE_ROT, HANDLE_PINCH_M),
        "l: pinch stave2 (172 deg)": ("left", *lid_stave2_pose(172.0), 0.0),
        "l: prop stave2 (141 deg)": ("left", *lid_stave2_pose(141.0), MAX_JAW),
        "l: behind propped lid (146 deg)": ("left", lid_back_push_tcp(146.0, 0.052, 0.060), LEFT_PUSH_ROT, 0.0),
        "l: push lid 118 deg": ("left", lid_back_push_tcp(118.0, 0.052, -0.004), LEFT_PUSH_ROT, 0.0),
        "l: high behind 114 deg": ("left", np.array([scene.hinge_world_pos(TOP)[0] + 0.055 * _U_H[0], scene.hinge_world_pos(TOP)[1] + 0.055 * _U_H[1], TOP + 0.152]), LEFT_PRESS_ROT, MAX_JAW),
        "l: sweep over TDC": ("left", lid_press_point(0.046), LEFT_PRESS_ROT, MAX_JAW),
        "l: press dome shut (open hand)": ("left", lid_press_point(0.000), LEFT_PRESS_ROT, MAX_JAW),
    }


TOGETHER = tuple(
    ("r: brace d_handle", l_step)
    for l_step in (
        "l: pinch stave2 (172 deg)",
        "l: prop stave2 (141 deg)",
        "l: behind propped lid (146 deg)",
        "l: push lid 118 deg",
        "l: high behind 114 deg",
        "l: sweep over TDC",
        "l: press dome shut (open hand)",
    )
)


def solve(side: str, pos, rot):
    best = None
    seeds = [
        np.array(READY, dtype=float),
        np.array([0.3, 1.3, 1.3, -0.6, 0.0, 0.0], dtype=float),
        np.array([-0.3, 1.3, 1.3, -0.6, 0.0, 0.0], dtype=float),
        np.array([0.0, 1.5, 1.5, -0.4, 0.0, 0.0], dtype=float),
    ]
    for flip in (False, True):
        r = np.asarray(rot, dtype=float) @ (np.diag([-1.0, -1.0, 1.0]) if flip else np.eye(3))
        quat = ik.matrix_to_quat_xyzw(r)
        for q0 in seeds:
            q, pe, re = ik.solve(np.asarray(pos, dtype=float), quat, q0, base_pos=BASES[side], iters=120)
            margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
            ok = pe < IK_MM / 1000.0 and math.degrees(re) < IK_DEG
            key = (ok, margin if ok else -(pe * 1000.0 + math.degrees(re)))
            if best is None or key > best[0]:
                best = (key, q, float(pe * 1000.0), float(math.degrees(re)), margin, flip)
    return best[1], best[2], best[3], best[4], best[5]


def arm_hulls_and_wrist(side: str, q: np.ndarray, opening: float, links=ARM_LINKS) -> tuple[np.ndarray, list]:
    half = -0.5 * max(0.0, min(MAX_JAW, opening))
    joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, q)}, **dict.fromkeys(C.FINGER_JOINTS, half)}
    poses_w = analysis.fk_link_poses_world(joints, tuple(BASES[side]))
    out = []
    for link in links:
        for h in analysis.link_hulls().get(link, []):
            r, p = poses_w[link][:3, :3], poses_w[link][:3, 3]
            n_w = h.planes[:, :3] @ r.T
            out.append((link, analysis.Hull(vertices=h.vertices @ r.T + p, planes=np.c_[n_w, h.planes[:, 3] + n_w @ p])))
    return poses_w["gripper"][:3, 3], out


def box_hull(centre, rotation, size) -> analysis.Hull:
    centre, rotation, half = np.asarray(centre, float), np.asarray(rotation, float), np.asarray(size, float) / 2.0
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
    vertices = corners @ rotation.T + centre
    planes = []
    for i in range(3):
        for s in (1.0, -1.0):
            n = s * rotation[:, i]
            planes.append([*n, float(n @ centre + half[i])])
    return analysis.Hull(vertices=vertices, planes=np.array(planes))


def gap(hulls_a, hulls_b) -> float:
    best = math.inf
    for _, a in hulls_a:
        for _, b in hulls_b:
            best = min(best, float(analysis._signed(a.vertices, b).min()), float(analysis._signed(b.vertices, a).min()))
    return best


def collar_and_tray_hulls() -> dict[str, list]:
    """Exact 16-stave 118 mm ID can collar/wall hulls and Bochem 8480 tray rim hulls for finger-back clearance checks."""
    c_pos = scene.can_start_pos(TOP)
    collar = []
    n_staves = 16
    collar_t = P.CAN_R - P.COLLAR_IN_R
    r_collar = P.COLLAR_IN_R + 0.5 * collar_t
    chord_collar = 2.0 * P.CAN_R * math.tan(math.pi / n_staves) + 0.001
    for i in range(n_staves):
        ang = 2.0 * math.pi * i / n_staves
        rot = mk.axis_angle((0.0, 0.0, 1.0), ang)
        centre = c_pos + np.array([r_collar * math.cos(ang), r_collar * math.sin(ang), 0.5 * P.CAN_H])
        collar.append((f"collar_{i}", box_hull(centre, rot, (collar_t, chord_collar, P.CAN_H))))
    t_pos = scene.tray_start_pos(TOP)
    wh = P.TRAY_H - P.TRAY_FLOOR
    wz = P.TRAY_FLOOR + 0.5 * wh
    tray_walls = [
        ("tray_yp", box_hull(t_pos + (0.0, 0.5 * (P.TRAY_W - P.TRAY_WALL), wz), np.eye(3), (P.TRAY_L, P.TRAY_WALL, wh))),
        ("tray_ym", box_hull(t_pos + (0.0, -0.5 * (P.TRAY_W - P.TRAY_WALL), wz), np.eye(3), (P.TRAY_L, P.TRAY_WALL, wh))),
        ("tray_xp", box_hull(t_pos + (0.5 * (P.TRAY_L - P.TRAY_WALL), 0.0, wz), np.eye(3), (P.TRAY_WALL, P.TRAY_W, wh))),
        ("tray_xm", box_hull(t_pos + (-0.5 * (P.TRAY_L - P.TRAY_WALL), 0.0, wz), np.eye(3), (P.TRAY_WALL, P.TRAY_W, wh))),
    ]
    bench = [("bench", box_hull((0.36, 0.0, TOP - 0.030), np.eye(3), (1.10, 1.30, 0.060)))]
    return {"can_collar": collar, "tray_walls": tray_walls, "bench": bench}


def camera_framing() -> dict:
    cam = station.rig("calibrated").top_cam
    lens = cam.lens.resized(640, 360)
    c_pos = np.array(cam.pos, dtype=float)
    c_rot = quat_xyzw_to_matrix(np.array(cam.rot_xyzw, dtype=float))

    def proj(pt_w: np.ndarray) -> tuple[float, float]:
        pc = c_rot.T @ (pt_w - c_pos)
        z = -float(pc[2])
        return float(lens.cx + lens.fx * (pc[0] / z)), float(lens.cy - lens.fy * (pc[1] / z))

    hinge = scene.hinge_world_pos(TOP)
    open_far = hinge + mk.axis_angle(_T_H, math.radians(P.LID_OPEN_DEG)) @ (P.LID_OD * (-_U_H))
    tdc_top = hinge + np.array([0.0, 0.0, P.LID_OD])
    pts_w = {
        "can_rim_left": np.array([P.CAN_XY[0], P.CAN_XY[1] + P.CAN_R, TOP + P.CAN_H]),
        "can_rim_right": np.array([P.CAN_XY[0], P.CAN_XY[1] - P.CAN_R, TOP + P.CAN_H]),
        "lid_open_172_tip": open_far,
        "lid_90_tdc_top": tdc_top,
        "mat_outer_corner": np.array([P.MAT_XY[0] - 0.5 * P.MAT_SIDE, P.MAT_XY[1] - 0.5 * P.MAT_SIDE, TOP]),
        "tray_front_outer": np.array([P.TRAY_XY[0] - 0.5 * P.TRAY_L, P.TRAY_XY[1] - 0.5 * P.TRAY_W, TOP + P.TRAY_H]),
        "tray_back_outer": np.array([P.TRAY_XY[0] + 0.5 * P.TRAY_L, P.TRAY_XY[1] - 0.5 * P.TRAY_W, TOP + P.TRAY_H]),
    }
    uv = {k: proj(p) for k, p in pts_w.items()}
    us, vs = [u for u, _ in uv.values()], [v for _, v in uv.values()]
    ys = [abs(float(p[1])) for p in pts_w.values()]
    inside = 0.0 <= min(us) and max(us) <= 640.0 and 0.0 <= min(vs) and max(vs) <= 360.0 and max(ys) <= 0.35
    return {
        "pass": bool(inside),
        "max_abs_y_m": round(max(ys), 4),
        "u_range_640": [round(min(us), 1), round(max(us), 1)],
        "v_range_360": [round(min(vs), 1), round(max(vs), 1)],
        "points_640x360": {k: [round(u, 1), round(v, 1)] for k, (u, v) in uv.items()},
    }


def run_offline() -> dict:
    obs_hulls = collar_and_tray_hulls()
    found, rows, ok = {}, {}, True
    for name, (side, pos, rot, jaw) in poses().items():
        q, mm, deg, margin, flip = solve(side, pos, rot)
        wrist, hulls = arm_hulls_and_wrist(side, q, jaw, ARM_LINKS)
        found[name] = (side, q, jaw, pos, wrist)
        clear = {"bench": round(1000.0 * gap(hulls, obs_hulls["bench"]), 1),
                 "tray_walls": round(1000.0 * gap(hulls, obs_hulls["tray_walls"]), 1)}
        if not ("d_handle" in name or "dome" in name or "lid" in name or "stave2" in name or "114" in name or "TDC" in name):
            clear["can_collar"] = round(1000.0 * gap(hulls, obs_hulls["can_collar"]), 1)
        z_abv = float(pos[2] - TOP)
        good = (
            mm <= IK_MM
            and deg <= IK_DEG
            and margin >= MARGIN_DEG
            and z_abv <= 0.160 + 1e-6
            and 0.0 <= jaw <= MAX_JAW + 1e-6
            and min(clear.values()) > 0.0
        )
        ok &= good
        rows[name] = {
            "side": side,
            "tcp_m": [round(float(v), 4) for v in pos],
            "z_above_bench_mm": round(1000.0 * z_abv, 1),
            "ik_mm_deg": [round(mm, 3), round(deg, 3)],
            "flip": flip,
            "least_joint_margin_deg": round(margin, 1),
            "clear_mm": clear,
            "ok": good,
        }
    pairs = {}
    for a, b in TOGETHER:
        (sa, qa, oa, pa, wa), (sb, qb, ob, pb, wb) = found[a], found[b]
        tcp_sep = float(np.linalg.norm(pa - pb))
        wrist_sep = float(np.linalg.norm(wa - wb))
        _, ha = arm_hulls_and_wrist(sa, qa, oa, HAND_LINKS)
        _, hb = arm_hulls_and_wrist(sb, qb, ob, HAND_LINKS)
        hull_gap = gap(ha, hb)
        good = wrist_sep >= MIN_WRIST_SEP_M and hull_gap >= MIN_HULL_GAP_M
        ok &= good
        pairs[f"{a} + {b}"] = {
            "tcp_sep_mm": round(1000.0 * tcp_sep, 1),
            "wrist_sep_mm": round(1000.0 * wrist_sep, 1),
            "hand_hulls_apart_mm": round(1000.0 * hull_gap, 1),
            "ok": good,
        }
    framing = camera_framing()
    ok &= framing["pass"]
    return {"pass": bool(ok), "poses": rows, "both_hands": pairs, "top_camera_framing": framing}


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
        cv2.imwrite(str(out_dir / f"{stem}_policy.png"), cv2.resize(bgr, (320, 176), interpolation=cv2.INTER_AREA))
    ready = np.array([*READY, 1.0], dtype=float)
    action = Action(data=pack(ready, ready))
    res = None
    for _ in range(40):
        res = emb.step(action)
    brg_top_above_rim_mm = 1000.0 * ((float(m.mk.pose("brg_screen")[0][2]) + 0.5 * P.BRG_W) - (float(m.mk.pose("can")[0][2]) + P.CAN_H))
    after = {
        "lid_deg": round(m.lid_deg(), 2),
        "dash_mm": round(1000.0 * m.dash_travel(), 2),
        "can_slide_mm": round(1000.0 * m.can_slide(), 2),
        "can_tilt_deg": round(m.can_tilt_deg(), 2),
        "brg_screen_proud_of_rim_mm": round(brg_top_above_rim_mm, 2),
        "score_after_2s": float(res.info.get("score", 0.0)),
        "report": m.report(),
    }
    idle_ok = (
        168.0 <= after["lid_deg"] <= 173.0
        and after["brg_screen_proud_of_rim_mm"] >= 8.0
        and after["can_slide_mm"] <= 2.0
        and after["can_tilt_deg"] <= 2.0
        and after["score_after_2s"] == 0.0
        and after["report"]["zero"] is None
    )
    out = {"pass": bool(idle_ok), "after_2s": after}
    (out_dir / "g1_frame0.json").write_text(json.dumps(out, indent=1, default=str))
    return out


def main() -> int:
    if "--parts" in sys.argv:
        print(json.dumps(P.summary(), indent=1))
        return 0
    offline = run_offline()
    print("G1_OFFLINE " + json.dumps(offline, indent=1), flush=True)
    ok = bool(offline["pass"])
    if "--frame0" in sys.argv:
        out = run_frame0(Path(sys.argv[sys.argv.index("--frame0") + 1]))
        print("G1_FRAME0 " + json.dumps(out, indent=1, default=str), flush=True)
        ok &= bool(out["pass"])
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
