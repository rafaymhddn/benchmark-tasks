#!/usr/bin/env python3
"""Reach, joint margins and clearances of every pose the demo uses, without the simulator; and the start picture.

1. Offline: each waypoint of the right arm solved with ``yam_arena.ik`` (<= 0.5 mm, <= 0.6 deg, every joint >= 5 deg
   inside its limits); the arm's collision hulls (wrist to fingertips, and the forearm links) against the bench, the
   bowl (rim, inside, stand, wall), the raised seat and the pan; both arms' ready poses against the same; the carried
   pan's boxes against the bowl, the seat and the bench along its whole path; where the raised seat stands from the
   arm bases.
2. ``--frame0 <dir>`` in the simulator: the start pictures (top camera at 320 x 176 as the policy sees it) and 2.0 s
   idle: nothing moves, the score stays 0.

    scripts/rta simpy <id> mech/tasks/specimen_hat/checks.py [--frame0 /workspace/robocurve/outputs/<id>]
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
HAND_LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")
ARM_LINKS = ("link2", "link3") + HAND_LINKS
FINGER_LINKS = ("tip_left", "tip_right")
MAX_JAW = 2.0 * abs(float(C.FINGER_OPEN))  # 0.0939 m
IK_MM, IK_DEG, MARGIN_DEG = 0.5, 0.6, 5.0

# The right hand pinches the pan's front wall from above, 35 mm from its middle, the inner finger inside the bowl.
PAN_GRIP_Z = P.PAN_H - 0.012  # grasp point 12 mm under the rim
PAN_GRIP_ALONG = 0.035  # m along the wall from its middle (left of it once the pan is across)
PAN_LEAN_DEG = 45.0  # the hand leans about its closing axis: fingertips to the bowl's left, wrist to the right arm
PAN_FLIP = True  # the closing axis points into the pan (False: out of it, a half turn of the wrist the arm cannot make)
CARRY_LOW = P.RIM_Z + 0.012  # m: the pan's bottom over the bench while carried (12 mm over the rim)
PAN_LET_GO = 0.016  # m the fingers open to round the 4 mm wall when they let go
BACK = np.array([-0.060, -0.060, 0.040])  # m the hand draws back after letting go, before its ready pose


def frame(closing, approach) -> np.ndarray:
    a = np.asarray(approach, dtype=float)
    a = a / np.linalg.norm(a)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c = c / np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def turn(axis, deg: float) -> np.ndarray:
    """Rotation by ``deg`` about ``axis`` (Rodrigues)."""
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    a = math.radians(deg)
    return np.eye(3) + math.sin(a) * kx + (1.0 - math.cos(a)) * kx @ kx


def pan_origin(xy, bottom: float) -> np.ndarray:
    return np.array([xy[0], xy[1], TOP + bottom])


def grip_local() -> np.ndarray:
    return np.array([-(P.PAN_CHORD / 2.0 - P.PAN_WALL / 2.0), PAN_GRIP_ALONG, PAN_GRIP_Z])


def right_on_pan(origin: np.ndarray, yaw_deg: float, up: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """The right grasp point and rotation on the pan whose frame is at ``origin`` turned ``yaw_deg``: the fingers
    close across the front wall (the pan's -x side), the hand pointing down and leaned about its closing axis; ``up``
    m straight up from there."""
    r = P.rz(yaw_deg)
    rot = frame((1.0 if PAN_FLIP else -1.0) * r[:, 0], (0.0, 0.0, -1.0))
    rot = turn(rot[:, 0], PAN_LEAN_DEG) @ rot
    return np.asarray(origin, float) + r @ grip_local() + np.array([0.0, 0.0, up]), rot


def ready_pose(side: str) -> tuple[np.ndarray, np.ndarray]:
    return ik.tcp_pose(np.array(READY, dtype=float), BASES[side])


PAN_PLACE_XY = (P.BOLT_X - P.PAN_PLACE_D, P.BOWL_Y)
PAN_SEATED_BOTTOM = P.RIM_Z - (P.PAN_H - P.WING_T)  # 0.064 m: the wings on the rim


def pan_path() -> list[tuple[str, np.ndarray, float]]:
    """The carried pan's frame (origin, yaw) at the demo's waypoints: lifted, turned half way about the grasp (the hand
    turns its wrist), turned on while carried towards the bowl, over the opening, lowered until the wings rest on the
    rim. (The arm cannot finish the turn beside the start: there the across grasp is out of its wrist's range.)"""
    y0, y1 = P.PAN_START_YAW_DEG, P.PAN_PLACE_YAW_DEG
    lifted = pan_origin(P.PAN_START, CARRY_LOW)
    grasp = lifted + P.rz(y0) @ grip_local()
    over = pan_origin(PAN_PLACE_XY, CARRY_LOW)
    y_half, y_late = (y0 + y1) / 2.0, y1 + (y0 - y1) / 4.0
    g_over = over + P.rz(y1) @ grip_local()
    g_late = np.array([g_over[0], (grasp[1] + g_over[1]) / 2.0, g_over[2]])  # the turning pan kept clear of the seat
    return [
        ("lifted", lifted, y0),
        ("turning", grasp - P.rz(y_half) @ grip_local(), y_half),
        ("turned", g_late - P.rz(y_late) @ grip_local(), y_late),
        ("over the opening", over, y1),
        ("wings on the rim", pan_origin(PAN_PLACE_XY, PAN_SEATED_BOTTOM), y1),
    ]


def poses() -> dict[str, tuple]:
    """name -> (side, grasp point, rotation, finger opening (m), body held or None, pan origin, pan yaw)."""
    start, y0 = pan_origin(P.PAN_START, 0.0), P.PAN_START_YAW_DEG
    path = pan_path()
    _, seated, ys = path[-1]
    out = {"r: over the wall": ("right", *right_on_pan(start, y0, up=0.040), 0.050, None, start, y0),
           "r: on the wall": ("right", *right_on_pan(start, y0), P.PAN_WALL, "specimen_pan", start, y0)}  # fmt: skip
    for name, origin, yaw in path:
        out[f"r: pan {name}"] = ("right", *right_on_pan(origin, yaw), P.PAN_WALL, "specimen_pan", origin, yaw)
    out["r: let go of the wall"] = ("right", *right_on_pan(seated, ys), PAN_LET_GO, "specimen_pan", seated, ys)
    up, rot = right_on_pan(seated, ys, up=0.050)
    out["r: up from the wall"] = ("right", up, rot, PAN_LET_GO, None, seated, ys)
    out["r: back"] = ("right", up + BACK, rot, PAN_LET_GO, None, seated, ys)
    for side in ("left", "right"):
        out[f"{side[0]}: ready, start"] = (side, *ready_pose(side), MAX_JAW, None, start, y0)
        out[f"{side[0]}: ready, end"] = (side, *ready_pose(side), MAX_JAW, None, seated, ys)
    return out


def solve_pose(side: str, pos: np.ndarray, rot: np.ndarray, seed=None) -> tuple[np.ndarray, float, float, float]:
    best = None
    seeds = [np.array(READY, float), np.array([0.0, 1.2, 1.2, -0.5, 0.0, 0.0]), np.array([0.0, 1.6, 1.4, -0.3, 0.0, 0.0]),
             np.array([0.5, 1.4, 1.2, -0.8, 0.4, 0.0]), np.array([-0.5, 1.4, 1.2, -0.8, -0.4, 0.0])]  # fmt: skip
    if seed is not None:
        seeds.insert(0, np.asarray(seed, float))
    quat = ik.matrix_to_quat_xyzw(rot)
    for q0 in seeds:
        q, pe, re = ik.solve(np.asarray(pos, float), quat, q0, base_pos=BASES[side], iters=300)
        margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
        ok = pe * 1000.0 <= IK_MM and math.degrees(re) <= IK_DEG
        key = (ok, margin if ok else -(pe * 1000.0 + math.degrees(re)))
        if best is None or key > best[0]:
            best = (key, q, float(pe * 1000.0), float(math.degrees(re)), margin)
    return best[1], best[2], best[3], best[4]


def hand_hulls(side: str, q: np.ndarray, opening: float, links=ARM_LINKS) -> list:
    half = -0.5 * max(0.0, min(MAX_JAW, opening))
    joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, q)}, **dict.fromkeys(C.FINGER_JOINTS, half)}
    poses_w = analysis.fk_link_poses_world(joints, tuple(BASES[side]))
    out = []
    for link in links:
        for h in analysis.link_hulls().get(link, []):
            r, p = poses_w[link][:3, :3], poses_w[link][:3, 3]
            n_w = h.planes[:, :3] @ r.T
            out.append((link, analysis.Hull(vertices=h.vertices @ r.T + p, planes=np.c_[n_w, h.planes[:, 3] + n_w @ p])))
    return out


def box_hull(centre, rotation, size) -> analysis.Hull:
    centre, rotation, half = np.asarray(centre, float), np.asarray(rotation, float), np.asarray(size, float) / 2.0
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
    planes = []
    for i in range(3):
        for s in (1.0, -1.0):
            n = s * rotation[:, i]
            planes.append([*n, float(n @ centre + half[i])])
    return analysis.Hull(vertices=corners @ rotation.T + centre, planes=np.array(planes))


def gap(hulls_a, hulls_b) -> float:
    best = math.inf
    for _, a in hulls_a:
        for _, b in hulls_b:
            best = min(best, float(analysis._signed(a.vertices, b).min()), float(analysis._signed(b.vertices, a).min()))
    return best


def scene_hulls(pan_origin_: np.ndarray, pan_yaw: float, seat_deg: float = P.CHECK_HINGE_DEG) -> dict[str, list]:
    import scene as S

    bench = [("bench", box_hull((0.36, 0.0, TOP - 0.03), np.eye(3), (1.1, 1.3, 0.06)))]
    bowl = [(kind, box_hull((c[0], c[1], TOP + c[2]), np.eye(3), s)) for c, s, kind in P.bowl_boxes()]
    rs = P.seat_world_rot(seat_deg)
    seat = [("seat", box_hull(P.pin(TOP) + rs @ np.array(c), rs, s)) for c, s in P.seat_boxes() + P.bumper_boxes()]
    rp = P.rz(pan_yaw)
    pan = [("pan", box_hull(pan_origin_ + rp @ np.array(c), rp, s)) for c, s in S.pan_boxes()]
    return {"bench": bench, "bowl": bowl, "seat": seat, "pan": pan}


def pan_sweep() -> dict:
    """The carried pan's boxes against the bowl, the seat and the bench between the waypoints (the hand moves its
    grasp in a straight line and turns about the vertical; the pan turns with it). The last stretch, lowering onto the
    rim, ends with the wings on it: sampled to 90 %."""
    path = pan_path()
    out = {}
    for (n0, o0, y0), (n1, o1, y1) in zip(path, path[1:]):
        g0, g1 = o0 + P.rz(y0) @ grip_local(), o1 + P.rz(y1) @ grip_local()
        worst = {}
        for t in (0.0, 0.25, 0.5, 0.75, 0.9) if n1 == "wings on the rim" else (0.0, 0.2, 0.4, 0.6, 0.8, 1.0):
            yaw = y0 + t * (y1 - y0)
            f = scene_hulls(g0 + t * (g1 - g0) - P.rz(yaw) @ grip_local(), yaw)
            for k in ("bowl", "seat", "bench"):
                worst[k] = min(worst.get(k, math.inf), round(1000.0 * gap(f["pan"], f[k]), 1))
        out[f"{n0} -> {n1}"] = worst
    return out


def seat_reach() -> dict:
    """Where the raised seat stands: its nearest corner to each arm base (along the bench) and its corners' heights."""
    rs = P.seat_world_rot(P.CHECK_HINGE_DEG)
    corners = np.vstack([box_hull(P.pin(TOP) + rs @ np.array(c), rs, s).vertices for c, s in P.seat_boxes()])
    near = {side: round(float(np.linalg.norm((corners - BASES[side])[:, :2], axis=1).min()), 3) for side in BASES}
    return {"nearest_to_base_m": near, "x_m": [round(float(corners[:, 0].min()), 3), round(float(corners[:, 0].max()), 3)],
            "over_bench_m": [round(float(corners[:, 2].min() - TOP), 3), round(float(corners[:, 2].max() - TOP), 3)],
            "onto_stop_Nm": round(-P.seat_torque(P.CHECK_HINGE_DEG), 3), "balance_deg": round(P.seat_balance_deg(), 1)}  # fmt: skip


def run_offline() -> dict:
    rows, ok = {}, True
    for name, (side, pos, rot, opening, held, o, y) in poses().items():
        q, mm, dg, margin = solve_pose(side, pos, rot)
        fixtures = scene_hulls(o, y)
        hulls = hand_hulls(side, q, opening)
        clear = {}
        for k, v in fixtures.items():
            mine = [(n, h) for n, h in hulls if n in HAND_LINKS] if k == "bench" else hulls
            if k == "pan" and held == "specimen_pan":
                clear["pan (wrist)"] = round(1000.0 * gap([(n, h) for n, h in mine if n not in FINGER_LINKS], v), 1)
            else:
                clear[k] = round(1000.0 * gap(mine, v), 1)
        good = mm <= IK_MM and dg <= IK_DEG and margin >= MARGIN_DEG and min(clear.values()) > 0.0
        rows[name] = {"at_m": [round(float(v), 4) for v in pos], "z_mm": round(1000.0 * (pos[2] - TOP), 1),
                      "ik_mm_deg": [round(mm, 3), round(dg, 3)], "margin_deg": round(margin, 1), "clear_mm": clear, "ok": bool(good)}  # fmt: skip
        ok &= good
    sweep = pan_sweep()
    ok &= min(min(v.values()) for v in sweep.values()) >= 5.0
    return {"pass": bool(ok), "pan_sweep_mm": sweep, "seat": seat_reach(), "poses": rows}


def run_frame0(out_dir: Path) -> dict:
    import cv2
    import demo_kit as kit
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack

    out_dir.mkdir(parents=True, exist_ok=True)
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    obs = emb.reset(Scene(id="frame0", instruction=m.INSTRUCTION), seed=0)
    for cam, stem in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        rgb = np.asarray(obs.images[cam])
        cv2.imwrite(str(out_dir / f"{stem}.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(out_dir / "start_top_policy.png"), cv2.cvtColor(kit.policy_picture(np.asarray(obs.images["top_cam"])), cv2.COLOR_RGB2BGR))
    ready = np.array([*READY, 1.0], dtype=float)
    first, res = None, None
    for _ in range(40):
        res = emb.step(Action(data=pack(ready, ready)))
        if first is None:
            first = (m.mk.pose("specimen_pan")[0].copy(), m.seat_deg())
    drift_mm = round(1000.0 * float(np.linalg.norm(m.mk.pose("specimen_pan")[0] - first[0])), 2)
    drift_deg = round(abs(m.seat_deg() - first[1]), 3)
    score = float(res.info.get("score", 0.0))
    out = {"pass": score == 0.0 and m.STATE["zero"] is None and drift_mm < 1.0 and drift_deg < 0.2 and m.seat_up(),
           "drift_2s": {"pan_mm": drift_mm, "seat_deg": drift_deg}, "score_after_2s": score, "report": m.report()}  # fmt: skip
    (out_dir / "frame0.json").write_text(json.dumps(out, indent=1, default=str))
    return out


def main() -> int:
    if "--parts" in sys.argv:
        print(json.dumps(P.summary(), indent=1))
        return 0
    offline = run_offline()
    print("CHECKS " + json.dumps(offline, indent=1), flush=True)
    ok = bool(offline["pass"])
    if "--frame0" in sys.argv:
        f0 = run_frame0(Path(sys.argv[sys.argv.index("--frame0") + 1]))
        print("FRAME0 " + json.dumps(f0, indent=1, default=str), flush=True)
        ok &= bool(f0["pass"])
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
