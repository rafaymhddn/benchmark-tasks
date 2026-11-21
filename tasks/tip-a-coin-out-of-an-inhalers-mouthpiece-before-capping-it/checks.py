#!/usr/bin/env python3
"""Checks of the inhaler task without the simulator: how the coins wedge edge first in the 16 mm mouthpiece, demo.py's
plan through the arms' IK with its joint margins, the hands' collision hulls (the URDF's) against each other over the
whole plan, and how big the wedged coin is in the right wrist camera at the ready pose.

    PYTHONPATH=<yam_arena checkout> python3 mech/tasks/inhaler_coin/checks.py
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np
from yam_arena import analysis, ik, station
from yam_arena import constants as C
from yam_arena.constants import READY
from yam_arena.task_file import TABLE_TOP

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import parts as P  # noqa: E402

TOP = TABLE_TOP["station"]
BASES = {side: np.array(station.BASES[side], dtype=float) for side in ("left", "right")}
HAND_LINKS = ["link4", "link5", "gripper", "tip_left", "tip_right"]

def rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def frame(closing, approach) -> np.ndarray:
    """TCP rotation from the closing axis and the approach axis (columns: closing, pads' width, approach)."""
    a = np.asarray(approach, float) / np.linalg.norm(approach)
    c = np.asarray(closing, float) - np.dot(closing, a) * a
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def box_points(center, rot, size, step=0.002) -> np.ndarray:
    h = np.asarray(size) / 2
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
    return np.vstack(pts) @ np.asarray(rot).T + np.asarray(center)


def solve(side: str, pos, rot, start=None):
    """Best IK of a TCP pose: (q, mm off, deg off, smallest joint margin deg); both finger orders, a few starts."""
    best = None
    starts = [start] if start is not None else []
    starts += [np.array(s, float) for s in ([0.0, 1.0, 1.0, -0.6, 0.0, 0.0], [0.0, 1.6, 1.4, -0.4, 0.0, 0.0], [0.0, 0.6, 0.6, -1.2, 0.0, 0.0],
                                            [0.0, 1.2, 1.8, 0.4, 0.0, 0.0], [0.0, 1.0, 1.0, 0.6, 0.0, 1.5], [0.0, 1.0, 1.0, 0.6, 0.0, -1.5])]  # fmt: skip
    turns = [rot] if start is not None else [rot, rot @ np.diag([-1.0, -1.0, 1.0])]
    for turned in turns:
        for q0 in starts:
            q, e, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(turned), q0, base_pos=BASES[side], iters=150)
            margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
            key = (e < 0.001 and re < 0.01, margin if e < 0.001 and re < 0.01 else -(e + 0.05 * re))
            if best is None or key > best[0]:
                best = (key, q, e * 1000, math.degrees(re), margin)
        if start is not None and best[0][0]:
            break
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


def to_box(hs, box) -> float:
    center, rot, size = box
    pts = box_points(center, rot, size)
    return float(min(min(analysis.box_signed_distance(h.vertices, center, rot, size).min(), analysis._signed(pts, h).min()) for _, _, h in hs))


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


def qm(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])  # fmt: skip


def camera(side: str, q, opening: float = 0.094):
    """The wrist camera's world frame (OpenGL: looks along -z, y up) and its lens."""
    t = link_poses(side, q, opening)["gripper"]
    cam = station.wrist_camera(side)
    return t[:3, :3] @ qm(cam.rot_xyzw), t[:3, :3] @ np.array(cam.pos) + t[:3, 3], cam.lens


def project(cam, p) -> tuple[np.ndarray, float]:
    r, o, lens = cam
    c = r.T @ (np.asarray(p, float) - o)
    return np.array([lens.cx + lens.fx * c[0] / -c[2], lens.cy - lens.fy * c[1] / -c[2]]), float(-c[2])


def coin_view(cam, centre, normal_axis_a, normal_axis_b) -> dict:
    """The coin's middle in the picture and its size across two of its diameters (px)."""
    uv, d = project(cam, centre)
    sizes = [float(np.linalg.norm(project(cam, centre + P.LODGED_R * ax)[0] - project(cam, centre - P.LODGED_R * ax)[0])) for ax in (normal_axis_a, normal_axis_b)]
    inside = bool(0 <= uv[0] <= cam[2].width and 0 <= uv[1] <= cam[2].height)
    return {"px": uv.round(0).tolist(), "dist_m": round(d, 3), "across_px": [round(s, 1) for s in sizes], "in_frame": inside}


PRESSING = ("pushes it home", "holds it against the push", "holds it seated", "eases off")  # the press: forces asked of the arms at the seat's pose


def hands_over_plan(case: str = "full", every: float = 0.25) -> dict:
    """The smallest distance between the two hands' hulls at each step of demo.py's plan (mm), and the IK of the plan.
    The press's steps are left out: there the joints are offset to give a force (J^T F / stiffness) against a cap and an
    inhaler that do not move, so the hands stay at the seat's pose (the slide's last step), not where the offsets point."""
    import demo
    import demo_kit as kit
    from yam_arena import task_file

    m = task_file.load(str(HERE / "task.py")).module
    plan = demo.Plan(m, live=False, case=case)
    moves = plan.whole()
    dry = kit.dry_run(moves, station.BASES, limit_s=m.RUN_LIMIT_STEPS / 20.0)
    arms = {side: kit.Arm(side, station.BASES[side], moves.get(side, [])) for side in demo.SIDES}
    worst: dict = {}
    t, end = 0.0, max(a.seconds for a in arms.values())
    while t <= end:
        hs, labels = {}, []
        for side in demo.SIDES:
            action, label = arms[side].action(t)
            hs[side] = hulls(side, action[:6], max(0.0, min(0.094, float(action[6]) * 0.094)))
            labels.append(label)
        key = " | ".join(labels)
        if any(label in PRESSING for label in labels):
            t += every
            continue
        d = hands_apart(hs["left"], hs["right"])
        if d < worst.get(key, (np.inf,))[0]:
            worst[key] = (d, t)
        t += every
    rows = sorted(((round(1000 * d, 1), round(t, 2), k) for k, (d, t) in worst.items()))
    return {"ik": dry, "closest hands (mm, s, left | right)": rows[:6], "m": m}


def main() -> dict:
    out: dict = {"coins": P.check_all_coins()}
    plan = hands_over_plan()
    m = plan.pop("m")
    out.update(plan)
    cam = camera("right", np.array(READY, float))
    out["coin in the right wrist camera at ready (640 x 360)"] = coin_view(cam, np.asarray(m.COIN_POS, float), m.COIN_ROT[:, 0], m.COIN_ROT[:, 1])
    return out


if __name__ == "__main__":
    print(json.dumps(main(), indent=1, default=str))
