#!/usr/bin/env python3
"""Reach, joint margins, clearances and loads of every pose the demo uses, without the simulator; and the start picture.

1. Offline: each waypoint of both arms solved with ``yam_arena.ik`` (<= 0.5 mm, <= 0.6 deg, every joint >= 5 deg inside
   its limits); the hands' collision hulls against the bench, the deck, the seat (at the pose's angle) and the pan;
   the hands apart where they work together; the seat's weight on the holding hand as joint torques (J^T F) against
   the drives' limits; the carried pan's wings under the raised seat and its outer finger inside the opening.
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
FINGER_LINKS = ("tip_left", "tip_right")
MAX_JAW = 2.0 * abs(float(C.FINGER_OPEN))  # 0.0939 m
IK_MM, IK_DEG, MARGIN_DEG = 0.5, 0.6, 5.0

# The left hand holds the seat's near side arm (x < its middle line) from above, the fingers closing across the arm's
# 83.4 mm width (the inner finger in the opening, the outer one beyond the narrow deck); it turns with the seat.
SEAT_GRIP_D = 0.170  # m from the hinge line along the seat
SEAT_LEAN_DEG = 20.0  # the hand leans this far about its closing axis, its wrist towards the hinge
SEAT_FLIP = True  # the closing axis points outwards (the IK reaches it with wide joint margins)
SEAT_HOLD_DEG = 42.0  # the seat is held up at this angle while the pan goes under it
SEAT_REST_DEG = 0.6  # where the seat rests on the pan's 4 mm wings
SEAT_ARM_W = (P.SEAT_W - P.SEAT_IN_W) / 2.0  # 83.4 mm across the side arm

# The right hand pinches the middle of the pan's front wall from above, the inner finger inside the bowl.
PAN_GRIP_Z = P.PAN_H - 0.012  # grasp point 12 mm under the rim
PAN_GRIP_ALONG = -0.035  # m along the wall from its middle: the leaning wrist then rises through the seat's open front
CARRY_LOW = 0.127  # m: the pan's bottom over the bench while carried (12 mm over the rim)
PAN_LEAN_DEG = 30.0  # the right hand leans along the wall (its wrist towards the robot)
PAN_LET_GO = 0.016  # m the fingers open to round the 4 mm wall when they let go (the outer one stays off the rim)


def frame(closing, approach) -> np.ndarray:
    a = np.asarray(approach, dtype=float)
    a = a / np.linalg.norm(a)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c = c / np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def left_on_seat(deg: float, up: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """The left grasp point and rotation on the seat raised by ``deg``; ``up`` m drawn back along the fingers."""
    lean = math.radians(SEAT_LEAN_DEG)
    closing = (-1.0 if SEAT_FLIP else 1.0, 0.0, 0.0)
    approach = (0.0, -math.sin(lean), -math.cos(lean))  # down the seat's normal, leaned towards the hinge (+y_s is back)
    rot = P.seat_rot(deg) @ frame(closing, approach)
    x_mid = -(P.SEAT_IN_W + P.SEAT_W) / 4.0
    pos = P.seat_point(TOP, x_mid, SEAT_GRIP_D, P.RING_MID, deg) - up * rot[:, 2]
    return pos, rot


def seat_normal(deg: float) -> np.ndarray:
    return P.seat_rot(deg) @ np.array([0.0, 0.0, 1.0])


def seat_lift(deg: float) -> float:
    """N the hand lifts with, along the seat's normal at the grasp, to hold the seat at ``deg``."""
    return P.seat_torque(deg) / SEAT_GRIP_D


def pan_origin(xy, bottom: float) -> np.ndarray:
    return np.array([xy[0], xy[1], TOP + bottom])


def turn(axis, deg: float) -> np.ndarray:
    """Rotation by ``deg`` about ``axis`` (Rodrigues)."""
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    a = math.radians(deg)
    return np.eye(3) + math.sin(a) * kx + (1.0 - math.cos(a)) * kx @ kx


def right_on_pan(origin: np.ndarray, yaw_deg: float, up: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """The right grasp point and rotation on the pan whose frame is at ``origin`` turned ``yaw_deg``: the fingers
    close across the front wall (the pan's -x side) 35 mm from its middle, the hand pointing down and leaned 30 deg
    along the wall; ``up`` m straight up from there."""
    r = P.rz(yaw_deg)
    local = np.array([-(P.PAN_CHORD / 2.0 - P.PAN_WALL / 2.0), PAN_GRIP_ALONG, PAN_GRIP_Z])
    rot = frame(-r[:, 0], (0.0, 0.0, -1.0))
    rot = turn(rot[:, 0], PAN_LEAN_DEG) @ rot
    return np.asarray(origin, float) + r @ local + np.array([0.0, 0.0, up]), rot


PAN_PLACE_XY = (P.HOLE_X, P.PINTLE_Y - P.PAN_PLACE_D)
PAN_SEATED_BOTTOM = P.RIM_Z - (P.PAN_H - P.WING_T)  # 0.019 m: the wings on the rim


def pan_path() -> list[tuple[str, np.ndarray, float]]:
    """The carried pan's frame (origin, yaw) at the demo's waypoints."""
    sx, sy = P.PAN_START
    px, py = PAN_PLACE_XY
    return [
        ("lifted", pan_origin((sx, sy), CARRY_LOW), 0.0),
        ("turning", pan_origin((sx + 0.005, sy + 0.020), CARRY_LOW), 45.0),
        ("turned", pan_origin((px, sy + 0.040), CARRY_LOW), 90.0),
        ("over the opening", pan_origin((px, py), CARRY_LOW), 90.0),
        ("wings on the rim", pan_origin((px, py), PAN_SEATED_BOTTOM), 90.0),
    ]


def poses() -> dict[str, tuple]:
    """name -> (side, grasp point, rotation, finger opening (m), seat angle (deg), body held or None, pan origin, pan yaw)."""
    start = pan_origin(P.PAN_START, 0.0)
    _, seated, seated_yaw = pan_path()[-1]
    out = {"l: outside the ring": ("left", *left_on_seat(0.0, up=0.040), MAX_JAW, 0.0, None, start, P.PAN_START_YAW_DEG)}
    for name, deg in (("l: seat at 0 deg", 0.0), ("l: seat at 20 deg", 20.0), ("l: seat held up", SEAT_HOLD_DEG)):
        out[name] = ("left", *left_on_seat(deg), SEAT_ARM_W, deg, "toilet_seat", start, P.PAN_START_YAW_DEG)
    out["l: let go on the wings"] = ("left", *left_on_seat(SEAT_REST_DEG), MAX_JAW, SEAT_REST_DEG, "toilet_seat", seated, seated_yaw)
    out["l: drawn out"] = ("left", *left_on_seat(SEAT_REST_DEG, up=0.040), MAX_JAW, SEAT_REST_DEG, None, seated, seated_yaw)
    out["r: over the wall"] = ("right", *right_on_pan(start, P.PAN_START_YAW_DEG, up=0.040), 0.050, 0.0, None, start, P.PAN_START_YAW_DEG)
    out["r: on the wall"] = ("right", *right_on_pan(start, P.PAN_START_YAW_DEG), P.PAN_WALL, 0.0, "specimen_pan", start, P.PAN_START_YAW_DEG)
    for name, origin, yaw in pan_path():
        out[f"r: pan {name}"] = ("right", *right_on_pan(origin, yaw), P.PAN_WALL, SEAT_HOLD_DEG, "specimen_pan", origin, yaw)
    out["r: let go of the wall"] = ("right", *right_on_pan(seated, seated_yaw), PAN_LET_GO, SEAT_HOLD_DEG, "specimen_pan", seated, seated_yaw)
    out["r: up from the wall"] = ("right", *right_on_pan(seated, seated_yaw, up=0.050), PAN_LET_GO, SEAT_HOLD_DEG, None, seated, seated_yaw)
    return out


TOGETHER = [("l: seat held up", f"r: pan {name}") for name, _, _ in pan_path()] + [("l: seat held up", "r: up from the wall")]


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


def hand_hulls(side: str, q: np.ndarray, opening: float, links=HAND_LINKS) -> list:
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


def scene_hulls(seat_deg: float, pan_origin_: np.ndarray, pan_yaw: float) -> dict[str, list]:
    import scene as S

    bench = [("bench", box_hull((0.36, 0.0, TOP - 0.03), np.eye(3), (1.1, 1.3, 0.06)))]
    deck_o = np.array([P.HOLE_X, P.PINTLE_Y, TOP])
    deck = [("deck", box_hull(deck_o + np.array(c), np.eye(3), s)) for c, s, _ in S.deck_boxes()]
    pin = np.array([P.HOLE_X, P.PINTLE_Y, TOP + P.PINTLE_Z])
    rs = P.seat_rot(seat_deg)
    seat = [("seat", box_hull(pin + rs @ np.array(c), rs, s)) for c, s in P.seat_boxes() + P.bumper_boxes()]
    rp = P.rz(pan_yaw)
    pan = [("pan", box_hull(pan_origin_ + rp @ np.array(c), rp, s)) for c, s in S.pan_boxes()]
    return {"bench": bench, "deck": deck, "seat": seat, "pan": pan}


def run_offline() -> dict:
    import demo_kit as kit

    found, rows, ok = {}, {}, True
    for name, (side, pos, rot, opening, deg, held, o, y) in poses().items():
        q, mm, dg, margin = solve_pose(side, pos, rot)
        found[name] = (side, q, opening, pos)
        fixtures = scene_hulls(deg, o, y)
        hulls = hand_hulls(side, q, opening)
        body_links = [(n, h) for n, h in hulls if n not in FINGER_LINKS]
        clear = {}
        for k, v in fixtures.items():
            if held == {"seat": "toilet_seat", "pan": "specimen_pan"}.get(k):
                clear[k + " (wrist)"] = round(1000.0 * gap(body_links, v), 1)
            else:
                clear[k] = round(1000.0 * gap(hulls, v), 1)
        good = mm <= IK_MM and dg <= IK_DEG and margin >= MARGIN_DEG and min(clear.values()) > 0.0
        row = {"side": side, "at_m": [round(float(v), 4) for v in pos], "z_mm": round(1000.0 * (pos[2] - TOP), 1),
               "ik_mm_deg": [round(mm, 3), round(dg, 3)], "margin_deg": round(margin, 1), "clear_mm": clear}  # fmt: skip
        if held == "toilet_seat" and deg > 1.0:
            force = seat_lift(deg) * seat_normal(deg)
            torque = kit.point_jacobian(q, BASES[side]).T @ force
            row["lift_N"] = round(float(np.linalg.norm(force)), 1)
            row["torque_of_most"] = round(float(np.abs(torque / kit.JOINT_EFFORT).max()), 2)
            good &= row["torque_of_most"] <= 0.8
        row["ok"] = bool(good)
        ok &= good
        rows[name] = row
    pairs = {}
    for a, b in TOGETHER:
        (sa, qa, oa, pa), (sb, qb, ob, pb) = found[a], found[b]
        hull_gap = gap(hand_hulls(sa, qa, oa), hand_hulls(sb, qb, ob))
        good = hull_gap >= 0.020
        ok &= good
        pairs[f"{a} + {b}"] = {"tcp_apart_mm": round(1000.0 * float(np.linalg.norm(pa - pb)), 1), "hulls_apart_mm": round(1000.0 * hull_gap, 1), "ok": good}
    # The carried pan against the raised seat and the deck (its own boxes: the seat must not touch it in passing).
    sweep = {}
    for name, origin, yaw in pan_path()[:-1]:
        f = scene_hulls(SEAT_HOLD_DEG, origin, yaw)
        sweep[name] = {k: round(1000.0 * gap(f["pan"], f[k]), 1) for k in ("seat", "deck", "bench")}
        ok &= min(sweep[name].values()) >= 5.0
    seat_up = {deg: round(seat_lift(deg), 1) for deg in (0, 20, int(SEAT_HOLD_DEG))}
    return {"pass": bool(ok), "seat_lift_N": seat_up, "pan_sweep_mm": sweep, "poses": rows, "both_hands": pairs}


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
    out = {"pass": score == 0.0 and m.STATE["zero"] is None and drift_mm < 1.0 and drift_deg < 0.2,
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
