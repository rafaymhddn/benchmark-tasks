#!/usr/bin/env python3
"""G1 reach, whole-wrist URDF hull clearances, grasp-ceiling check, and frame-0 idle test for sink_stopper.

1. Offline (no GPU simulator needed): solves IK sequentially from READY for every Left and Right waypoint and checks:
   - IK error <= 0.5 mm and <= 0.6 deg,
   - every arm joint >= 6.0 deg inside its limits,
   - every grasp / waypoint TCP height <= 160.0 mm over the bench (16.0 cm ceiling),
   - full forearm + wrist + finger collision hulls (``link4``, ``link5``, ``gripper``, ``tip_left``, ``tip_right``)
     clear of the benchtop, vanity cabinet walls/deck/basin/cradle, and the 38 mm drain tailpiece & rear port,
   - Left and Right arm hulls >= 20.0 mm apart on every simultaneous step,
   - open-loop rod-straddle geometry: pre-grasp open fingers clear the 5.5 mm rod before advancing onto it.
2. ``--frame0 <dir>`` in Isaac Sim: writes ``start_top.png``, ``start_left_wrist.png``, ``start_right_wrist.png`` (plus
   ``*_policy.png`` at 320 x 176) and holds READY for 2.0 s verifying rest stability and score == 0.0.
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

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import parts as P  # noqa: E402

TOP = TABLE_TOP["station"]
BASES = {side: np.array(station.BASES[side], dtype=float) for side in ("left", "right")}
HAND_LINKS = ("link3", "link4", "link5", "gripper", "tip_left", "tip_right")
UPPER_LINKS = ("link1", "link2") + HAND_LINKS
MAX_JAW = 2.0 * abs(float(C.FINGER_OPEN))
IK_MM, IK_DEG, MARGIN_DEG, SEPARATION_TCP, SEPARATION_HULL = 0.5, 0.6, 5.0, 0.080, 0.020
GRASP_CEILING = 0.160  # m over bench: hard 16.0 cm grasp ceiling


def frame(closing, approach) -> np.ndarray:
    """Gripper rotation matrix: columns are closing axis (x), pad-width axis (y), approach axis (z)."""
    a = np.asarray(approach, dtype=float)
    a = a / np.linalg.norm(a)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c = c / np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


# --- Left hand: grasps the 35.6 mm chrome stopper cap from above inside the open basin -------------------------------
def left_rot(yaw_deg: float) -> np.ndarray:
    pr = math.radians(8.5)
    app = np.array([math.sin(pr) * 0.80, -math.sin(pr) * 0.60, -math.cos(pr)], dtype=float)
    yr = math.radians(yaw_deg)
    cl = np.array([math.sin(yr), -math.cos(yr), 0.0], dtype=float)
    return frame(cl, app)


LEFT_YAW_START = 45.0
LEFT_YAW_ALIGNED = -45.0  # -90 deg turn aligns the stopper's y-facing eye straight onto the +x rear port
LEFT_CAP_Z = TOP + P.FLANGE_Z + 0.0165  # 152.5 mm over bench: tilted tips clear flange by >2 mm
LEFT_LIFT = 0.0065  # m: lifts closed eye (-6.2 mm) into port window (159.0 mm < 160.0 mm ceiling)
LEFT_HOVER_Z = TOP + 0.1595  # 159.5 mm over bench (< 160.0 mm ceiling)
LEFT_OPEN_JAW = 0.044  # m: 44 mm open around 35.6 mm cap
LEFT_HOLD_JAW = 0.036  # m: pinch contact on 35.6 mm cap


# --- Right hand: roll = +90 deg (closing = [0,0,-1]) lays the wrist-camera tower horizontally into the open -y space ---
def right_rot(yaw_deg: float, pitch_deg: float = -6.0, vertical_jaws: bool = True) -> np.ndarray:
    yr, pr = math.radians(yaw_deg), math.radians(pitch_deg)
    app = np.array([math.cos(pr) * math.cos(yr), math.cos(pr) * math.sin(yr), math.sin(pr)], dtype=float)
    cl = np.array([0.0, 0.0, -1.0], dtype=float) if vertical_jaws else np.array([math.sin(yr), -math.cos(yr), 0.0], dtype=float)
    return frame(cl, app)


RIGHT_YAW_DEG = 56.0
RIGHT_PITCH_DEG = -6.0
RIGHT_ROT = right_rot(RIGHT_YAW_DEG, RIGHT_PITCH_DEG, True)
RIGHT_PRESS_ROT = right_rot(RIGHT_YAW_DEG, -16.0, True)  # pitches fingertips down while keeping wrist housing ~60 mm over bench
_R_APP = RIGHT_ROT[:, 2]
RIGHT_OFF_ROT = right_rot(52.0, -14.0, True)
RIGHT_BYPASS_ROT = right_rot(46.0, -8.0, True)
RIGHT_DOOR_ROT = right_rot(22.0, -30.0, False)
RIGHT_FRONT_ROT = right_rot(10.0, -36.0, False)
RIGHT_GRAV_COMP = np.array([0.004, 0.006, 0.004], dtype=float)

ROD_GRIP_BEHIND_BALL = P.CLEVIS_AT_X  # 0.068 m behind ball centre (x = 0.501 m start, 0.469 m seated under full-depth deck)
ROD_PAD_Z_ABOVE = 0.002
RIGHT_OPEN_JAW = 0.036  # m: 36 mm vertical jaw opening straddling the 16 mm clevis block
RIGHT_PRESS_PITCH_DEG = 16.5  # deg downward tilt of rear rod (lifts stopper to ~11.0 mm open)


def rod_grip_world(ball_x: float, pitch_down_deg: float = 0.0, extra_z: float = ROD_PAD_Z_ABOVE) -> np.ndarray:
    th = math.radians(pitch_down_deg)
    return np.array(
        [
            ball_x + ROD_GRIP_BEHIND_BALL * math.cos(th),
            P.Y_DRAIN,
            TOP + P.PORT_Z - ROD_GRIP_BEHIND_BALL * math.sin(th) + extra_z,
        ],
        dtype=float,
    )


def poses() -> dict[str, tuple[str, np.ndarray, np.ndarray, float]]:
    """Named waypoints: (side, tcp_world, rot_matrix, jaw_opening_m)."""
    r_start = rod_grip_world(P.X_BALL_START, 0.0) + RIGHT_GRAV_COMP
    r_seated = rod_grip_world(P.X_SEAT + 0.010, 0.0) + RIGHT_GRAV_COMP
    r_pressed = rod_grip_world(P.X_SEAT + 0.004, RIGHT_PRESS_PITCH_DEG, extra_z=0.004) + RIGHT_GRAV_COMP
    r_pre = r_start - 0.036 * _R_APP
    r_bypass = np.array([0.460, -0.055, TOP + 0.050], dtype=float)  # sails low (z = 50 mm) under full-depth 195 mm right shelf
    r_door = np.array([0.255, -0.155, TOP + 0.074], dtype=float)
    r_front = np.array([0.230, -0.275, TOP + 0.148], dtype=float)
    r_off = r_pressed - 0.018 * RIGHT_PRESS_ROT[:, 2]

    l_base = np.array([P.X_DRAIN, P.Y_DRAIN, LEFT_CAP_Z], dtype=float)
    l_hover = np.array([P.X_DRAIN, P.Y_DRAIN, LEFT_HOVER_Z], dtype=float)
    l_lifted = l_base + np.array([0.0, 0.0, LEFT_LIFT])

    return {
        "left: hover over stopper": ("left", l_hover, left_rot(LEFT_YAW_START), LEFT_OPEN_JAW),
        "left: on stopper cap": ("left", l_base, left_rot(LEFT_YAW_START), LEFT_HOLD_JAW),
        "left: lifted": ("left", l_lifted, left_rot(LEFT_YAW_START), LEFT_HOLD_JAW),
        "left: lifted & aligned": ("left", l_lifted, left_rot(LEFT_YAW_ALIGNED), LEFT_HOLD_JAW),
        "left: release & rise": ("left", l_hover, left_rot(LEFT_YAW_ALIGNED), LEFT_OPEN_JAW),
        "right: front pull-back": ("right", r_front, RIGHT_FRONT_ROT, RIGHT_OPEN_JAW),
        "right: outside doorway": ("right", r_door, RIGHT_DOOR_ROT, RIGHT_OPEN_JAW),
        "right: bypass pipe": ("right", r_bypass, RIGHT_BYPASS_ROT, RIGHT_OPEN_JAW),
        "right: pre-grasp rod": ("right", r_pre, RIGHT_ROT, RIGHT_OPEN_JAW),
        "right: on retracted rod": ("right", r_start, RIGHT_ROT, P.CLEVIS_BLOCK[2]),
        "right: ball seated": ("right", r_seated, RIGHT_ROT, P.CLEVIS_BLOCK[2]),
        "right: rear rod pressed down": ("right", r_pressed, RIGHT_PRESS_ROT, P.CLEVIS_BLOCK[2]),
        "right: release & back off": ("right", r_off, RIGHT_OFF_ROT, RIGHT_OPEN_JAW),
    }


TOGETHER = (
    ("left: hover over stopper", "right: front pull-back"),
    ("left: on stopper cap", "right: outside doorway"),
    ("left: lifted & aligned", "right: pre-grasp rod"),
    ("left: lifted & aligned", "right: on retracted rod"),
    ("left: lifted & aligned", "right: ball seated"),
    ("left: release & rise", "right: ball seated"),
)


def solve_chain(waypoints: list[tuple[str, str, np.ndarray, np.ndarray, float]]) -> dict:
    """Solve IK sequentially per arm from READY (exactly as demo_kit.Arm runs)."""
    q_prev = {"left": np.array(READY, dtype=float), "right": np.array(READY, dtype=float)}
    out = {}
    for name, side, pos, rot, opening in waypoints:
        quat = ik.matrix_to_quat_xyzw(rot)
        q, pe, re = ik.solve(pos, quat, q_prev[side], base_pos=BASES[side], iters=200)
        q_prev[side] = q
        margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
        out[name] = (side, q, opening, float(pe * 1000.0), float(math.degrees(re)), margin)
    return out


def hand_hulls(side: str, q: np.ndarray, opening: float, links=HAND_LINKS) -> list[tuple[str, analysis.Hull]]:
    half = -0.5 * max(0.0, min(MAX_JAW, opening))
    joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, q)}, **dict.fromkeys(C.FINGER_JOINTS, half)}
    poses_w = analysis.fk_link_poses_world(joints, tuple(BASES[side]))
    out = []
    for link in links:
        for h in analysis.link_hulls().get(link, []):
            r, p = poses_w[link][:3, :3], poses_w[link][:3, 3]
            n_w = h.planes[:, :3] @ r.T
            out.append((link, analysis.Hull(vertices=h.vertices @ r.T + p, planes=np.c_[n_w, h.planes[:, 3] + n_w @ p])))
    if "gripper" in links:  # USD wrist-camera mount on -y of gripper link (Constants: WRIST_CAM_POS = (0, -0.08, 0))
        rg, pg = poses_w["gripper"][:3, :3], poses_w["gripper"][:3, 3]
        cam_c = pg + rg @ np.array([0.0, -0.060, 0.005], dtype=float)
        out.append(("wrist_cam_tower", box_hull(cam_c, rg, (0.040, 0.050, 0.040))))
    return out


def box_hull(centre, rotation, size) -> analysis.Hull:
    centre, rotation, half = np.asarray(centre, float), np.asarray(rotation, float), np.asarray(size, float) / 2.0
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float) * half
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


def scene_obstacle_hulls() -> dict[str, list[tuple[str, analysis.Hull]]]:
    """Exact world-frame convex hulls of benchtop, every solid vanity shape, and outer envelopes of drain + rear port."""
    import task as T

    bench = [("benchtop", box_hull((0.35, 0.0, TOP - 0.020), np.eye(3), (0.90, 1.00, 0.040)))]
    vanity_hulls: list[tuple[str, analysis.Hull]] = []
    vspec = T.mk.MECH.bodies["vanity"]
    vpos, vrot = np.array(vspec.pos, float), vspec.rotation()
    for k, sh in enumerate(vspec.shapes):
        if not sh.solid:
            continue
        srot = vrot @ T.mk.quat_to_matrix(sh.quat)
        spos = vpos + vrot @ np.array(sh.pos, float)
        vanity_hulls.append((f"vanity_{k}", box_hull(spos, srot, 2.0 * sh.half_extents())))

    port_len = P.X_COLLAR - (P.X_DRAIN + 0.0185)
    port_cx = 0.5 * ((P.X_DRAIN + 0.0185) + P.X_COLLAR)
    rail_len = (P.X_BALL_START + 0.006) - P.X_COLLAR
    rail_cx = 0.5 * (P.X_COLLAR + P.X_BALL_START + 0.006)
    drain_hulls = [
        ("pipe", box_hull((P.X_DRAIN, P.Y_DRAIN, TOP + P.BASIN_FLOOR / 2.0), np.eye(3), (P.DRAIN_OD, P.DRAIN_OD, P.BASIN_FLOOR))),
        ("flange", box_hull((P.X_DRAIN, P.Y_DRAIN, TOP + P.FLANGE_Z - 0.0013), np.eye(3), (0.046, 0.046, 0.0026))),
        ("port", box_hull((port_cx, P.Y_DRAIN, TOP + P.PORT_Z), np.eye(3), (port_len, 0.0265, 0.0265))),
        ("rail_l", box_hull((rail_cx, P.Y_DRAIN + 0.0082, TOP + P.PORT_Z - 0.009), np.eye(3), (rail_len, 0.0034, 0.005))),
        ("rail_r", box_hull((rail_cx, P.Y_DRAIN - 0.0082, TOP + P.PORT_Z - 0.009), np.eye(3), (rail_len, 0.0034, 0.005))),
    ]
    return {"bench": bench, "vanity": vanity_hulls, "drain": drain_hulls}


def run_offline() -> dict:
    named = poses()
    seq = [(k, *v) for k, v in named.items()]
    solved = solve_chain(seq)
    obstacles = scene_obstacle_hulls()
    rows = {}
    ok = True
    for name, (side, pos, _rot, opening) in named.items():
        _, q, _, mm, deg, margin = solved[name]
        hulls6 = hand_hulls(side, q, opening, HAND_LINKS)
        hulls8 = hand_hulls(side, q, opening, UPPER_LINKS)
        clear = {
            "bench": round(1000.0 * gap(hulls6, obstacles["bench"]), 1),
            "vanity": round(1000.0 * gap(hulls8, obstacles["vanity"]), 1),
            "drain": round(1000.0 * gap(hulls8, obstacles["drain"]), 1),
        }
        over_bench = float(pos[2] - TOP)
        good = (
            mm <= IK_MM
            and deg <= IK_DEG
            and margin >= MARGIN_DEG
            and over_bench <= GRASP_CEILING
            and min(clear.values()) > 0.2
        )
        ok &= good
        rows[name] = {
            "side": side,
            "tcp_over_bench_mm": round(1000.0 * over_bench, 1),
            "IK (mm, deg)": [round(mm, 3), round(deg, 3)],
            "least_margin_deg": round(margin, 1),
            "hulls_clear_mm": clear,
            "ok": good,
        }

    pairs = {}
    for a, b in TOGETHER:
        (sa, qa, oa, *_), (sb, qb, ob, *_) = solved[a], solved[b]
        tcp_d = float(np.linalg.norm(named[a][1] - named[b][1]))
        hull_d = gap(hand_hulls(sa, qa, oa), hand_hulls(sb, qb, ob))
        good = tcp_d >= SEPARATION_TCP and hull_d >= SEPARATION_HULL
        ok &= good
        pairs[f"{a} + {b}"] = {
            "tcp_apart_mm": round(1000.0 * tcp_d, 1),
            "hulls_apart_mm": round(1000.0 * hull_d, 1),
            "ok": good,
        }

    rod_len = P.ROD_OUTER_L - P.ROD_THREAD_IN
    rod_cx = P.X_BALL_START + P.BALL_R + rod_len / 2.0
    rod_start_hull = [
        ("rod", box_hull((rod_cx, P.Y_DRAIN, TOP + P.PORT_Z), np.eye(3), (rod_len, P.ROD_D, P.ROD_D))),
        ("clevis", box_hull((P.X_BALL_START + P.CLEVIS_AT_X, P.Y_DRAIN, TOP + P.PORT_Z), np.eye(3), P.CLEVIS_BLOCK)),
    ]
    _, q_pre, o_pre, *_ = solved["right: pre-grasp rod"]
    pre_rod_clear_mm = round(1000.0 * gap(hand_hulls("right", q_pre, o_pre), rod_start_hull), 1)
    ok &= pre_rod_clear_mm > 2.0

    # Verify the full-depth clear acrylic rear panel physically blocks a top-down reach to the rear rod
    top_down_probe = np.array([0.500, P.Y_DRAIN, TOP + 0.180], dtype=float)
    q_top, *_ = ik.solve(top_down_probe, ik.matrix_to_quat_xyzw(left_rot(0.0)), np.array(READY, dtype=float), base_pos=BASES["left"], iters=200)
    top_down_blocked_mm = round(1000.0 * gap(hand_hulls("left", q_top, 0.030, HAND_LINKS), obstacles["vanity"]), 1)
    ok &= top_down_blocked_mm < 0.0

    return {
        "pass": bool(ok),
        "parts_summary": P.summary(),
        "right_pregrasp_clear_of_rod_mm": pre_rod_clear_mm,
        "top_down_rear_reach_blocked_mm": top_down_blocked_mm,
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
        cv2.imwrite(str(out_dir / f"{stem}_policy.png"), cv2.resize(bgr, (320, 176), interpolation=cv2.INTER_AREA))
    ready = np.array([*READY, 1.0], dtype=float)
    action = Action(data=pack(ready, ready))
    res = None
    for _ in range(40):
        res = emb.step(action)
    assert res is not None
    moved = {
        name: round(1000.0 * float(np.linalg.norm(m.mk.pose(name)[0] - np.asarray(m.mk.built_pose(name)[0]))), 2)
        for name in ("stopper", "pivot_rod")
    }
    rep = m.report()
    ok = float(res.info.get("score", 0.0)) == 0.0 and rep["zero"] is None and max(moved.values()) <= 2.5
    out = {
        "pass": ok,
        "moved_from_built_mm": moved,
        "score_after_2s": float(res.info.get("score", 0.0)),
        "report": rep,
    }
    (out_dir / "g1_frame0.json").write_text(json.dumps(out, indent=1, default=str))
    return out


def main() -> int:
    offline = run_offline()
    print("G1_OFFLINE " + json.dumps(offline, indent=1), flush=True)
    ok = bool(offline["pass"])
    if "--frame0" in sys.argv:
        f0 = run_frame0(Path(sys.argv[sys.argv.index("--frame0") + 1]))
        print("G1_FRAME0 " + json.dumps(f0, indent=1, default=str), flush=True)
        ok &= bool(f0["pass"])
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
