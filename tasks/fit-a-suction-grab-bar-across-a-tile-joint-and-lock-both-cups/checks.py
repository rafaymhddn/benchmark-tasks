#!/usr/bin/env python3
"""Checks of the grab bar task without the simulator: every hand pose of the demo through the arms' IK with its joint
margins, the hands' collision hulls (the URDF's) against the bar, the bench and each other, the heights of the grasps
over the bench, and the arms' joint torques for the stated forces.

    PYTHONPATH=<yam_arena checkout> python3 mech/tasks/grab_bar/checks.py [--bar-z <m over the bench>]
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np
from yam_arena import analysis, ik, station
from yam_arena import constants as C
from yam_arena.task_file import TABLE_TOP

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import parts as P  # noqa: E402

TOP = TABLE_TOP["station"]
BASES = {side: np.array(station.BASES[side], dtype=float) for side in ("left", "right")}
HAND_LINKS = ["link4", "link5", "gripper", "tip_left", "tip_right"]
STIFFNESS = np.array([C.SHOULDER["stiffness"]] * 3 + [C.WRIST["stiffness"]] * 3)
EFFORT = np.array([C.SHOULDER["effort"]] * 3 + [C.WRIST["effort"]] * 3)
HEIGHT_LIMIT = 0.16  # m over the bench: the highest grasp
TWIST_LIMIT = 1.0  # N m: the most twist in a pinch
PAD_LIMIT = 50.0  # N per pad (the finger drive's effort)


def frame(closing, approach) -> np.ndarray:
    """TCP rotation from the closing axis and the approach axis (columns: closing, pads' width, approach)."""
    a = np.asarray(approach, float) / np.linalg.norm(approach)
    c = np.asarray(closing, float) - np.dot(closing, a) * a
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


HAND_PITCH_DEG = 15.0  # both hands point forward, tipped this far down: level, the low poses ran the joints into their limits (bar at 70 mm: 24 mm off, 0 deg margin)
_A = (math.cos(math.radians(HAND_PITCH_DEG)), 0.0, -math.sin(math.radians(HAND_PITCH_DEG)))  # the approach
FORWARD_PINCH_UPDOWN = frame((math.sin(math.radians(HAND_PITCH_DEG)), 0.0, math.cos(math.radians(HAND_PITCH_DEG))), _A)  # fingers above and below (the grip)


def solve(side: str, pos, rot, start=None, orders: str = "both"):
    """Best IK of a TCP pose: (q, mm off, deg off, smallest joint margin deg); a few starts, both finger orders
    (``orders`` "given" or "flipped": that one only)."""
    best = None
    starts = [start] if start is not None else []
    starts += [np.array(s, float) for s in ([0.0, 1.0, 1.0, -0.6, 0.0, 0.0], [0.0, 1.6, 1.4, -0.4, 0.0, 0.0], [0.0, 0.6, 0.6, -1.2, 0.0, 0.0],
                                            [0.0, 1.2, 1.8, 0.4, 0.0, 0.0], [0.0, 1.0, 1.0, 0.6, 0.0, 1.5], [0.0, 1.0, 1.0, 0.6, 0.0, -1.5])]  # fmt: skip
    turns = {"both": [rot, rot @ np.diag([-1.0, -1.0, 1.0])], "given": [rot], "flipped": [rot @ np.diag([-1.0, -1.0, 1.0])]}[orders]
    for turned in turns:
        for q0 in starts:
            q, e, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(turned), q0, base_pos=BASES[side], iters=150)
            margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
            key = (e < 0.001 and re < 0.01, margin if e < 0.001 and re < 0.01 else -(e + 0.05 * re))
            if best is None or key > best[0]:
                best = (key, q, e * 1000, math.degrees(re), margin)
    return best[1], best[2], best[3], best[4]


def link_poses(side: str, q, opening: float):
    joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, q)}, **dict.fromkeys(C.FINGER_JOINTS, -opening / 2)}
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
    best = np.inf
    for _, _, a in ha:
        for _, _, b in hb:
            lo_a, hi_a, lo_b, hi_b = a.vertices.min(0), a.vertices.max(0), b.vertices.min(0), b.vertices.max(0)
            if np.maximum(lo_b - hi_a, lo_a - hi_b).max() > 0.05:
                continue
            best = min(best, analysis._signed(a.vertices, b).min(), analysis._signed(b.vertices, a).min())
    return float(best)


def lowest(hs) -> float:
    return float(min(h.vertices[:, 2].min() for _, _, h in hs))


def deepest_x(hs) -> float:
    """The hand's furthest reach forward (m): against the tiles' face."""
    return float(max(h.vertices[:, 0].max() for _, _, h in hs))


def jacobian(side: str, q, eps: float = 1e-6) -> np.ndarray:
    """3 x 6 linear Jacobian of the TCP (numerical)."""
    p0 = ik.tcp_pose(np.asarray(q, float), BASES[side])[0]
    out = np.zeros((3, 6))
    for k in range(6):
        dq = np.zeros(6)
        dq[k] = eps
        out[:, k] = (ik.tcp_pose(np.asarray(q, float) + dq, BASES[side])[0] - p0) / eps
    return out


def world(b_point, bar_y: float, bar_z: float) -> np.ndarray:
    """A point of the bar (B) in the world, the bar upright on the wall with its middle at (bar_y, bar_z over the bench)."""
    x, y, z = b_point
    return np.array([P.PANEL_X + x, bar_y + y, TOP + bar_z + z])


HOLD_AT = 0.030  # m along the grip (B y): the right hand holds the grip towards the left cup, so that its press seals that cup when the left lever is locked (at 0.050 the left hand's pinch came within 4-6 mm of it)
PRESS_AT = -0.060  # m: the left hand presses the grip near the right cup while the right lever is locked
HOLD_PRESS = 20.0  # N towards the wall from the hand on the grip while a lever is locked: two thirds of it or more on the nearer cup, over PRESS_N
APPROACH_DEG = {("pinch", "r"): (45.0, 25.0), ("pinch", "l"): (50.0, 25.0), ("push", "r"): (45.0, 35.0), ("push", "l"): (45.0, 35.0)}
# deg (turned towards the wall, tipped down) of a lever hand pointing along the bar towards its middle, per grasp and
# lever, in the finger order that keeps its wrist camera away from the tiles (at 15, 15 the camera stood up to 25 mm
# inside them, recording gb5rec, and the other order ran out of joint margin); picked by IK over 0-90 / 5-45 deg for
# joint margins (dry run 5.8 deg or more) and the hands' and the camera's clearances
PINCH_AHEAD = 0.003  # m: the lever's middle lies this far ahead of the side pinch's grasp point, along the approach (the pads' tips 7 mm past its inner face: 11 mm ahead, the pads closed in front of it, gb5d19)
PUSH_TIP = 0.013  # m: the fingertips lie this far ahead of the grasp point (the URDF's pad tips)
PUSH_TURN = 0.5  # share of the lever's swing the pushing hand turns with (turning all of it, the arm runs out of joint margin at the cam's rest)


def side_rot(side: str, theta: float, kind: str) -> np.ndarray:
    """A lever hand's rotation (B) for ``kind`` ('pinch' or 'push') at lever ``side``, ``theta`` rad from released:
    pointing along the bar towards its middle, turned towards the wall and tipped down by APPROACH_DEG, turned with
    the lever's swing; fingers above and below the lever."""
    s = P.out_sign(side)
    turn, pitch = APPROACH_DEG[(kind, side)]
    yaw = math.radians(-s * (90.0 - turn)) + s * (P.LEVER_RAD - theta)
    p = math.radians(pitch)
    a = np.array([math.cos(p) * math.cos(yaw), math.cos(p) * math.sin(yaw), -math.sin(p)])
    c = np.array([math.sin(p) * math.cos(yaw), math.sin(p) * math.sin(yaw), math.cos(p)])
    return frame(c, a)


def pinch_b(side: str, theta: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The side pinch on lever ``side`` at ``theta`` (B): (grasp point, rotation, the lever's point it moves), the pads
    above and below the lever LEVER_ARM out."""
    rot = side_rot(side, theta, "pinch")
    point = np.array(P.lever_tip_b(side, theta))
    return point - PINCH_AHEAD * rot[:, 2], rot, point


def push_b(side: str, theta: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The closed fingertips on lever ``side``'s outer face at ``theta`` (B): (grasp point, rotation, the face's point
    they push), LEVER_ARM out."""
    rot = side_rot(side, P.LEVER_RAD - PUSH_TURN * (P.LEVER_RAD - theta), "push")
    face = np.array(P.lever_tip_b(side, theta)) + (P.LEVER_T / 2) * np.array(P.lever_out_b(side, theta))
    return face - PUSH_TIP * rot[:, 2], rot, face


LOOK_TCP = (0.22, -0.07, 0.15)  # m (x, y, over the bench): the right hand's grasp point for its look at the right cup at the start
LOOK_ABOVE = 0.03  # m: the look aims this far above the point midway between the right cup's red tab and the joint


def tab(side: str, bar_y: float, bar_z: float) -> np.ndarray:
    """The middle of cup ``side``'s red/green tab's top (world), the bar upright on the wall (shapes.bar)."""
    return world((P.INDICATOR_AT[0], P.CUP_Y[side], P.INDICATOR_AT[1] + 0.004), bar_y, bar_z)


def look_pose(bar_y: float = P.BAR_Y0, bar_z: float = P.BAR_Z) -> tuple[np.ndarray, np.ndarray]:
    """(grasp point, rotation) of the right hand looking at the right cup: the hand points at the joint and the red
    tab, so that its wrist camera has both in view (``look_view``)."""
    w = tab("r", bar_y, bar_z)
    aim = (w + np.array([P.PANEL_X, P.JOINT_Y, w[2]])) / 2 + np.array([0.0, 0.0, LOOK_ABOVE])
    tcp = np.array([LOOK_TCP[0], LOOK_TCP[1], TOP + LOOK_TCP[2]])
    a = (aim - tcp) / np.linalg.norm(aim - tcp)
    return tcp, frame(np.cross(a, [0.0, 0.0, 1.0]), a)


def wrist_camera(side: str, q) -> tuple:
    """(rotation, position, lens) of the wrist camera of arm ``side`` at joints ``q`` (station.wrist_camera on the
    gripper link; OpenGL frame: it looks along its -z)."""
    from yam_arena.urdf import quat_xyzw_to_matrix

    t = link_poses(side, q, 0.094)["gripper"]
    cam = station.wrist_camera(side)
    return t[:3, :3] @ quat_xyzw_to_matrix(np.array(cam.rot_xyzw, dtype=float)), t[:3, :3] @ np.array(cam.pos, dtype=float) + t[:3, 3], cam.lens


def pixel(cam, point) -> tuple[np.ndarray, float]:
    """Where ``point`` (world) falls in the camera's 640 x 360 picture, and how far in front of it (m)."""
    r, o, lens = cam
    c = r.T @ (np.asarray(point, dtype=float) - o)
    return np.array([lens.cx + lens.fx * c[0] / -c[2], lens.cy - lens.fy * c[1] / -c[2]]), float(-c[2])


CAMERA_CLEAR = 0.040  # m: a wrist camera's middle this far or more from the tiles' face (in them its picture went black: recording gb5rec, 5.2-8.8 s), its housing well clear


def camera_clearance(side: str, q, bar_y: float, bar_z: float, thetas: dict) -> tuple[float, float, str]:
    """(m to the tiles' face, m to the bar's nearest part, that part) from arm ``side``'s wrist camera at joints ``q``,
    the bar's levers at ``thetas`` (a hull's planes give at least the distance)."""
    at = wrist_camera(side, q)[1]
    d, part = min((float(analysis._signed(at[None, :], h)[0]), name) for name, h in bar_hulls(bar_y, bar_z, thetas))
    return float(P.PANEL_X - at[0]), d, part


FLIPS = (np.eye(3), np.diag([-1.0, -1.0, 1.0]))  # the finger orders: as given, a half turn about the approach


def finger_order(side: str, poses) -> tuple[np.ndarray, bool, float, float]:
    """The finger order for arm ``side`` through ``poses`` ((grasp point, rotation) pairs, world): IK within 1 mm and
    the wrist camera CAMERA_CLEAR or more off the tiles' face first, then the larger smallest joint margin: (order,
    whether the camera stays clear, IK mm, margin deg), the worst over the poses."""
    best = None
    for flip in FLIPS:
        worst_mm, worst_margin, worst_cam = 0.0, 1e9, 1e9
        for tcp, rot in poses:
            q, e, _, margin = solve(side, tcp, rot @ flip, orders="given")
            worst_mm, worst_margin = max(worst_mm, e), min(worst_margin, margin)
            worst_cam = min(worst_cam, P.PANEL_X - float(wrist_camera(side, q)[1][0]))
        key = (worst_mm <= 1.0 and worst_cam >= CAMERA_CLEAR, worst_mm <= 1.0, worst_margin)
        if best is None or key > best[0]:
            best = (key, flip, worst_mm, worst_margin)
    return best[1], best[0][0], best[2], best[3]


def look_view() -> dict:
    """The look: IK, joint margin, the hand against the bar, and where the right wrist camera sees the red tab and
    the joint (pixels at 320 x 176, the policy's size)."""
    tcp, rot = look_pose()
    q, e, _, mg = solve("right", tcp, rot, orders="given")
    cam = wrist_camera("right", q)
    w = tab("r", P.BAR_Y0, P.BAR_Z)
    seen = {name: pixel(cam, x) for name, x in (("red tab", w), ("joint 45 mm over the tab", (P.PANEL_X, P.JOINT_Y, w[2] + 0.045)),
                                                ("joint 80 mm over the tab", (P.PANEL_X, P.JOINT_Y, w[2] + 0.080)))}  # fmt: skip
    d, part = to_bar(hulls("right", q, 0.094), P.BAR_Y0, P.BAR_Z, {"l": P.LEVER_RAD, "r": P.LEVER_RAD})
    return {"grasp point over the bench mm": round(1000 * LOOK_TCP[2], 1), "IK mm": round(e, 2), "margin deg": round(mg, 1), "to the bar mm": [round(1000 * d, 1), part],
            "seen at 320 x 176 (x, y px)": {k: [round(float(uv[0]) / 2, 1), round(float(uv[1]) * 176 / 360, 1)] for k, (uv, _) in seen.items()},
            "red tab across px (320 wide)": round(P.INDICATOR_D * cam[2].fx / seen["red tab"][1] / 2, 1), "camera to the tab m": round(seen["red tab"][1], 3)}  # fmt: skip


def _box_hull(centre, rot, half) -> analysis.Hull:
    c, r, h = np.asarray(centre, float), np.asarray(rot, float), np.asarray(half, float)
    vertices = np.array([c + r @ (np.array([a, b, d]) * h) for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)])
    planes = [[*(s * r[:, k]), s * r[:, k] @ c + h[k]] for k in range(3) for s in (-1, 1)]
    return analysis.Hull(vertices=vertices, planes=np.array(planes))


def _prism_hull(centre, axis, radius: float, length: float, n: int = 24) -> analysis.Hull:
    """A cylinder as the n-sided prism round it (the hull holds the cylinder)."""
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    u = np.cross(a, [1.0, 0.0, 0.0] if abs(a[0]) < 0.9 else [0.0, 1.0, 0.0])
    u /= np.linalg.norm(u)
    v, c, out = np.cross(a, u), np.asarray(centre, float), radius / math.cos(math.pi / n)
    vertices = [c + s * a * length / 2 + out * (math.cos(2 * math.pi * (k + 0.5) / n) * u + math.sin(2 * math.pi * (k + 0.5) / n) * v) for k in range(n) for s in (-1, 1)]
    planes = [[*nk, nk @ c + radius] for nk in (math.cos(2 * math.pi * k / n) * u + math.sin(2 * math.pi * k / n) * v for k in range(n))]
    planes += [[*(s * a), s * a @ c + length / 2] for s in (-1, 1)]
    return analysis.Hull(vertices=np.array(vertices), planes=np.array(planes))


def bar_hulls(bar_y: float, bar_z: float, thetas: dict, skip=()) -> list:
    """The bar's solid parts as hulls (world), its levers at ``thetas`` (rad from released), less the parts named in
    ``skip`` (what the hand takes)."""
    import shapes as S

    o = world((0.0, 0.0, 0.0), bar_y, bar_z)
    front = -P.CUP_T - P.HOUSING_T
    inner, outer = 0.013, P.LEVER_PIVOT[1] - P.LEVER_T / 2 - 0.001  # the bracket, as shapes.bar draws it
    out = [("grip", _prism_hull(o + [P.GRIP_X, 0.0, 0.0], (0, 1, 0), P.GRIP_D / 2, P.CUP_PITCH))]
    for s in ("l", "r"):
        y, sg = P.CUP_Y[s], P.out_sign(s)
        out += [(f"housing_{s}", _prism_hull(o + [-P.CUP_T - P.HOUSING_T / 2, y, 0.0], (1, 0, 0), P.HOUSING_D / 2, P.HOUSING_T)),
                (f"grip end {s}", _box_hull(o + [(front + P.GRIP_X) / 2, y, 0.0], np.eye(3), np.array([front - P.GRIP_X + 0.004, 0.026, 0.030]) / 2)),
                (f"bracket {s}", _box_hull(o + [P.LEVER_PIVOT[0], y + sg * (inner + outer) / 2, P.LEVER_PIVOT[2]], np.eye(3), np.array([0.012, outer - inner, 0.012]) / 2)),
                (f"cup_{s}", _prism_hull(o + [-P.CUP_T / 2, y, 0.0], (1, 0, 0), P.CUP_R, P.CUP_T))]  # fmt: skip
        if f"lever_{s}" in skip:
            continue
        rot = S.lever_rot(thetas[s], s)
        pivot = o + np.array(P.lever_pivot_b(s))
        out.append((f"lever_{s}", _box_hull(pivot + rot @ [P.LEVER_L / 2, 0.0, 0.0], rot, np.array([P.LEVER_L, P.LEVER_T, P.LEVER_WIDTH]) / 2)))
    return [(name, hull) for name, hull in out if name not in skip]


def to_bar(hand_hulls, bar_y: float, bar_z: float, thetas: dict, skip=()) -> tuple[float, str]:
    """(m, the part): how far a hand keeps from the bar's parts (negative: into them)."""
    d, pair = analysis.hull_set_distance([(n, h) for n, _, h in hand_hulls], bar_hulls(bar_y, bar_z, thetas, skip))
    return float(d), (pair[1] if pair else "")


def demo_grasps(bar_z: float) -> dict:
    """The demo's grasps: the right hand holding the grip near the left cup (bar at the start and moved), the left hand
    pressing it near the right cup (moved), each release's side pinch from locked to let go (at the start) and each
    lock's fingertip push from the cam's rest to locked (moved): the grasp's height, IK, joint margins, the hand
    against the bar's other parts and against the other hand."""
    out: dict = {}
    start, moved = P.BAR_Y0, P.BAR_Y0 - P.MOVE
    rest = {"l": P.CAM_RAD, "r": P.CAM_RAD}
    holder = {}
    for name, y, thetas in (("start", start, {"l": P.LEVER_RAD, "r": P.CAM_RAD}), ("moved", moved, rest)):
        q, e, _, mg = solve("right", world((P.GRIP_X, HOLD_AT, 0.0), y, bar_z), FORWARD_PINCH_UPDOWN)
        holder[name] = hulls("right", q, P.GRIP_D)
        d, part = to_bar(holder[name], y, bar_z, thetas, skip=("grip",))
        out[f"right hand holds the grip near the left cup, bar {name}"] = {"IK mm": round(e, 2), "margin deg": round(mg, 1), "to the bar's other parts mm": [round(1000 * d, 1), part]}
    q, e, _, mg = solve("left", world((P.GRIP_X, PRESS_AT, 0.0), moved, bar_z), FORWARD_PINCH_UPDOWN)
    presser = hulls("left", q, P.GRIP_D)
    d, part = to_bar(presser, moved, bar_z, {"l": P.LEVER_RAD, "r": P.CAM_RAD}, skip=("grip",))
    out["left hand presses the grip near the right cup, bar moved"] = {"IK mm": round(e, 2), "margin deg": round(mg, 1), "to the bar's other parts mm": [round(1000 * d, 1), part]}

    def rows_for(arm, s, y, grasp, degs, others, opening, fixed):
        rows, q = [], None
        order = finger_order(arm, [(world(tcp_b, y, bar_z), rot) for tcp_b, rot, _ in (grasp(s, math.radians(t)) for t in degs)])[0]
        for t in degs:
            tcp_b, rot, _ = grasp(s, math.radians(t))
            q, e, _, mg = solve(arm, world(tcp_b, y, bar_z), rot @ order, start=q, orders="given")
            hs = hulls(arm, q, opening)
            d, part = to_bar(hs, y, bar_z, {**fixed, s: math.radians(t)}, skip=(f"lever_{s}",))
            cw, cb, cpart = camera_clearance(arm, q, y, bar_z, {**fixed, s: math.radians(t)})
            row = {"deg": t, "grasp over the bench mm": round(1000 * (bar_z + tcp_b[2]), 1), "IK mm": round(e, 2), "margin deg": round(mg, 1),
                   "to the bar mm": [round(1000 * d, 1), part], "lowest over the bench mm": round(1000 * (lowest(hs) - TOP), 1),
                   "camera to the tiles, the bar mm": [round(1000 * cw, 1), round(1000 * cb, 1), cpart]}  # fmt: skip
            for name, other in others.items():
                row[f"to the {name} mm"] = round(1000 * hands_apart(other, hs), 1)
            rows.append(row)
        return rows

    release = (P.LEVER_DEG, 100.0, P.LEVER_CENTRE_DEG, 90.0, 80.0)
    lock = (45.0, 55.0, P.LEVER_CAM_DEG, 80.0, 90.0, P.LEVER_CENTRE_DEG, 100.0, P.LEVER_DEG)  # from as far open as a let-go lever stood (demo.LOCK_FROM_DEG)
    out["right hand releases lever r by a side pinch, bar at the start"] = rows_for("right", "r", start, pinch_b, release, {}, P.LEVER_WIDTH, {"l": P.LEVER_RAD})
    out["left hand releases lever l by a side pinch, bar at the start"] = rows_for("left", "l", start, pinch_b, release, {"holder": holder["start"]}, P.LEVER_WIDTH, {"r": P.CAM_RAD})
    out["left hand pushes lever l home, bar moved"] = rows_for("left", "l", moved, push_b, lock, {"holder": holder["moved"]}, 0.0, {"r": P.CAM_RAD})
    out["right hand pushes lever r home, bar moved"] = rows_for("right", "r", moved, push_b, lock, {"presser": presser}, 0.0, {"l": P.LEVER_RAD})
    return out


def forces(bar_z: float) -> dict:
    """Joint torques (share of each joint's effort) for the stated forces: a lever hand's pull or push of 25 N at
    centre, the holder's and the presser's HOLD_PRESS towards the wall."""
    moved, out = P.BAR_Y0 - P.MOVE, {}
    for arm, s, y, grasp, sign, what in (("right", "r", P.BAR_Y0, pinch_b, 1.0, "releases"), ("left", "l", P.BAR_Y0, pinch_b, 1.0, "releases"),
                                         ("left", "l", moved, push_b, -1.0, "pushes home"), ("right", "r", moved, push_b, -1.0, "pushes home")):  # fmt: skip
        tcp_b, rot, _ = grasp(s, P.CENTRE_RAD)
        order = finger_order(arm, [(world(tcp_b, y, bar_z), rot)])[0]
        q = solve(arm, world(tcp_b, y, bar_z), rot @ order, orders="given")[0]
        f = 25.0 * sign * np.array(P.lever_out_b(s, P.CENTRE_RAD))
        out[f"{arm} hand {what} lever {s}, 25 N at centre"] = np.round(np.abs(jacobian(arm, q).T @ f) / EFFORT, 3).tolist()
    for arm, at in (("right", HOLD_AT), ("left", PRESS_AT)):
        q = solve(arm, world((P.GRIP_X, at, 0.0), moved, bar_z), FORWARD_PINCH_UPDOWN)[0]
        out[f"{arm} hand presses the grip {HOLD_PRESS:g} N"] = np.round(np.abs(jacobian(arm, q).T @ np.array([HOLD_PRESS, 0.0, 0.0])) / EFFORT, 3).tolist()
    return out


def main() -> dict:
    bar_z = float(sys.argv[sys.argv.index("--bar-z") + 1]) if "--bar-z" in sys.argv else P.BAR_Z
    out: dict = {"bar middle over the bench mm": round(1000 * bar_z, 1)}
    places = {"start": P.BAR_Y0, "moved": P.BAR_Y0 - P.MOVE}
    out["rims from the joint, the edge mm (left cup, right cup)"] = {name: [[round(1000 * v, 1) for v in P.rim_clearance(y + P.CUP_Y[s], bar_z)] for s in ("l", "r")] for name, y in places.items()}
    out["demo grasps"] = demo_grasps(bar_z)
    out["the right hand's look at the right cup"] = look_view()
    heights = [row["grasp over the bench mm"] for rows in out["demo grasps"].values() if isinstance(rows, list) for row in rows]
    heights += [1000 * bar_z, out["the right hand's look at the right cup"]["grasp point over the bench mm"]]  # the grip's holds; the look
    out["highest grasp over the bench mm (limit)"] = [round(max(heights), 1), round(1000 * HEIGHT_LIMIT, 1)]
    cams = [row["camera to the tiles, the bar mm"] for rows in out["demo grasps"].values() if isinstance(rows, list) for row in rows]
    out["lever hands' cameras nearest the tiles (limit), the bar mm"] = [min(c[0] for c in cams), round(1000 * CAMERA_CLEAR, 1), min(c[1] for c in cams)]
    out["joint torque / effort"] = forces(bar_z)
    out["pinch on the grip"] = {"bar's weight N": round(P.BAR_MASS * 9.81, 2), "lever torque at centre N m": round(P.LEVER_N * P.LEVER_ARM, 3),
                                "pinch twist limit N m": TWIST_LIMIT, "pad force limit N": PAD_LIMIT}  # fmt: skip
    return out


if __name__ == "__main__":
    print(json.dumps(main(), indent=1))
