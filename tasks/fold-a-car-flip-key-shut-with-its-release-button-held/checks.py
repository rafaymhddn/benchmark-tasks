#!/usr/bin/env python3
"""Checks (a), (b), (c) of the flip-key spec without the simulator: every pose through the arms' IK, the joint
margins, and the hands' collision hulls (the URDF's) against each other, against the key (fob and blade as boxes)
and against the bench and the tray.

    PYTHONPATH=<yam_arena checkout> python3 mech/tasks/flip_key/checks.py [scan|plan] [out.json]

The key's frame K: origin at the fob's middle, x along the fob towards the pivot end, y across it towards the edge
with the blade's channel, z out of the face with the buttons. The blade turns about K's z through the pivot: fold
angle 0 open (along +x), 180 degrees folded (along -x, in the channel). During the fold the fob lies flat, buttons
up, turned PSI about the vertical.

Hands. B (button hand): pinches the fob across its thickness, grasp point on the fob's middle plane under the
button, pointing level along ALPHA in K (from the hand towards the fob). A (other hand): first holds the fob across
its width pointing down; then pushes the blade with the rear finger of its open hand (the pad's inner face on the
blade's trailing edge, the hand's closing axis along the blade's direction of travel); at 180 degrees it closes
across the fob's width over the folded blade, so that it holds key and blade when B lets go.
"""

from __future__ import annotations

import json
import math
import sys

import numpy as np
from yam_arena import analysis, ik, station
from yam_arena import constants as C

TOP = 0.7475  # bench top of the station scene (yam_arena.task_file.TABLE_TOP["station"])
BASES = {side: np.array(station.BASES[side], dtype=float) for side in ("left", "right")}
HAND_LINKS = ["link4", "link5", "gripper", "tip_left", "tip_right"]

# --- the key (K frame, m); (a) = assumed, to be measured on the bought key ---
FOB = np.array([0.070, 0.034, 0.016])  # (a)
PIVOT = np.array([0.026, 0.0128, 0.0])  # (a): blade's outer edge 0.3 mm proud of the fob's edge when folded
BLADE = np.array([0.060, 0.009, 0.0025])  # (a)
BLADE_ROOT = 0.005  # the pivot sits 5 mm from the blade's root end: 55 mm from pivot to tip (spec: radius about 55)
REACH = BLADE[0] - BLADE_ROOT
BUTTON = np.array([0.024, 0.0])  # on the top face, 10 mm round (a)

# --- the tray (world): a round dish, floor and wall as rings of boxes in the check ---
TRAY_AT = np.array([0.28, 0.0])
TRAY_FLOOR_R, TRAY_RIM_R, TRAY_H, TRAY_FLOOR_T = 0.075, 0.090, 0.020, 0.005

PAD_HULLS = {"tip_left": {14, 15}, "tip_right": {5, 8}}  # the finger pads' hulls (they reach the fingertip, z +12.8 mm)


def rz(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def frame(closing, approach) -> np.ndarray:
    """TCP rotation from the closing axis and the approach axis (columns: closing, pads' width, approach)."""
    a = np.asarray(approach, float) / np.linalg.norm(approach)
    c = np.asarray(closing, float) - np.dot(closing, a) * a
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


class Key:
    def __init__(self, psi: float, at) -> None:
        self.r, self.p = rz(psi), np.asarray(at, float)

    def world(self, local) -> np.ndarray:
        return self.p + self.r @ np.asarray(local, float)

    def vec(self, local) -> np.ndarray:
        return self.r @ np.asarray(local, float)

    def fob_box(self):
        return self.p, self.r, FOB

    def blade_box(self, theta: float):
        e = np.array([math.cos(theta), math.sin(theta), 0.0])
        return self.world(PIVOT + (REACH - BLADE_ROOT) / 2 * e), self.r @ rz(theta), BLADE

    def trailing(self, theta: float, s: float) -> np.ndarray:
        """The point of the blade's trailing edge (the side away from the folding motion) s from the pivot."""
        e, n = np.array([math.cos(theta), math.sin(theta), 0.0]), np.array([-math.sin(theta), math.cos(theta), 0.0])
        return self.world(PIVOT + s * e - BLADE[1] / 2 * n)

    def fold_dir(self, theta: float) -> np.ndarray:
        return self.vec([-math.sin(theta), math.cos(theta), 0.0])


def box_points(center, rot, size, step=0.0015) -> np.ndarray:
    """Points on the surface of a box, about ``step`` apart."""
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


def hulls(side: str, q, opening: float):
    joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, q)}, **dict.fromkeys(C.FINGER_JOINTS, -opening / 2)}
    poses = analysis.fk_link_poses_world(joints, tuple(BASES[side]))
    out = []
    for link in HAND_LINKS:
        for i, h in enumerate(analysis.link_hulls().get(link, [])):
            pose = poses[link]
            r, p = pose[:3, :3], pose[:3, 3]
            n_w = h.planes[:, :3] @ r.T
            out.append((link, i, analysis.Hull(vertices=h.vertices @ r.T + p, planes=np.c_[n_w, h.planes[:, 3] + n_w @ p])))
    return out


def to_box(hs, box, skip_pads=False) -> tuple[float, str]:
    """Smallest signed distance (m) between hulls and a box, both ways (hull corners in the box, box surface in the hulls)."""
    center, rot, size = box
    pts = box_points(center, rot, size)
    best, who = np.inf, ""
    for link, i, h in hs:
        if skip_pads and i in PAD_HULLS.get(link, ()):
            continue
        d1 = analysis.box_signed_distance(h.vertices, center, rot, size).min()
        d2 = analysis._signed(pts, h).min()
        d = min(d1, d2)
        if d < best:
            best, who = float(d), f"{link}#{i}"
    return best, who


def hands_apart(ha, hb) -> tuple[float, str]:
    best, who = np.inf, ""
    for la, ia, a in ha:
        for lb, ib, b in hb:
            lo_a, hi_a, lo_b, hi_b = a.vertices.min(0), a.vertices.max(0), b.vertices.min(0), b.vertices.max(0)
            if np.maximum(lo_b - hi_a, lo_a - hi_b).max() > 0.03:
                continue
            d = min(analysis._signed(a.vertices, b).min(), analysis._signed(b.vertices, a).min())
            if d < best:
                best, who = float(d), f"{la}#{ia}/{lb}#{ib}"
    return best, who


def lowest(hs) -> float:
    return float(min(h.vertices[:, 2].min() for _, _, h in hs))


# --- the poses ---
Z_A = 0.006  # hand A's grasp point over the fob's middle plane: fingertips 1 mm over the fob's underside (to set it down)


def pose_b(key: Key, alpha: float):
    a = key.vec([math.cos(alpha), math.sin(alpha), 0.0])
    return key.world([BUTTON[0], BUTTON[1], 0.0]), frame([0.0, 0.0, 1.0], a), FOB[2]


def pose_a_hold(key: Key, x_a: float, width: float = FOB[1]):
    return key.world([x_a, (width - FOB[1]) / 2, Z_A]), frame(key.vec([0.0, 1.0, 0.0]), [0.0, 0.0, -1.0]), width


def pose_a_push(key: Key, theta: float, s: float, opening: float):
    c = key.fold_dir(theta)
    g = key.trailing(theta, s) + opening / 2 * c
    g[2] = key.p[2] + Z_A
    return g, frame(c, [0.0, 0.0, -1.0]), opening


def check_plan(psi, at, alpha, x_a, s_push, opening, side_a="right", side_b="left", verbose=False) -> dict:
    key = Key(psi, at)
    out = {}
    # B on the button, the blade open; A holding across the width (a)
    gb, rb, wb = pose_b(key, alpha)
    qb, eb, reb, mb = solve(side_b, gb, rb)
    hb = hulls(side_b, qb, wb)
    ga, ra, wa = pose_a_hold(key, x_a)
    qa, ea, rea, ma = solve(side_a, ga, ra)
    ha = hulls(side_a, qa, wa)
    out["a"] = dict(b_ik_mm=round(eb, 2), b_margin=round(mb, 1), a_ik_mm=round(ea, 2), a_margin=round(ma, 1),
                    hands_mm=round(1000 * hands_apart(ha, hb)[0], 1), hands_pair=hands_apart(ha, hb)[1],
                    a_fob_mm=round(1000 * to_box(ha, key.fob_box(), skip_pads=True)[0], 1),
                    b_fob_mm=round(1000 * to_box(hb, key.fob_box(), skip_pads=True)[0], 1),
                    a_blade_mm=round(1000 * to_box(ha, key.blade_box(0.0))[0], 1), b_blade_mm=round(1000 * to_box(hb, key.blade_box(0.0))[0], 1),
                    b_low_mm=round(1000 * (lowest(hb) - TOP), 1), a_low_mm=round(1000 * (lowest(ha) - TOP), 1))  # fmt: skip
    # (b) the push along the arc
    rows, q, worst = [], None, dict(hands=9.9, a_fob=9.9, b_blade=9.9, a_blade_other=9.9, margin=999.0, ik=0.0, step=0.0)
    for deg in range(0, 181, 5):
        th = math.radians(deg)
        g, r, w = pose_a_push(key, th, s_push, opening)
        qn, e, re, m = solve(side_a, g, r, start=q)
        if q is None:
            qn, e, re, m = solve(side_a, g, r)
        step = 0.0 if q is None else float(np.degrees(np.abs(qn - q)).max())
        q = qn
        h = hulls(side_a, q, w)
        apart = hands_apart(h, hb)
        fob = to_box(h, key.fob_box())
        blade_b = to_box(hb, key.blade_box(th))
        # A against the blade, without the rear finger's pad (it touches the blade's trailing edge on purpose)
        a_hulls_other = [(l, i, hh) for l, i, hh in h if not (i in PAD_HULLS.get(l, ()) and np.linalg.norm(hh.vertices.mean(0) - key.trailing(th, s_push)) < 0.03 and _rear(hh, g, r, w))]
        blade_a = to_box(a_hulls_other, key.blade_box(th))
        row = dict(deg=deg, ik_mm=round(e, 2), margin=round(m, 1), j6=round(math.degrees(q[5]), 0), step=round(step, 1),
                   hands_mm=round(1000 * apart[0], 1), a_fob_mm=round(1000 * fob[0], 1), a_blade_mm=round(1000 * blade_a[0], 1),
                   b_blade_mm=round(1000 * blade_b[0], 1), low_mm=round(1000 * (lowest(h) - TOP), 1))  # fmt: skip
        if verbose:
            row.update(hands_pair=apart[1], fob_link=fob[1], blade_link=blade_a[1])
        rows.append(row)
        worst["hands"] = min(worst["hands"], row["hands_mm"])
        worst["a_fob"] = min(worst["a_fob"], row["a_fob_mm"])
        worst["b_blade"] = min(worst["b_blade"], row["b_blade_mm"])
        worst["a_blade_other"] = min(worst["a_blade_other"], row["a_blade_mm"])
        worst["margin"] = min(worst["margin"], row["margin"])
        worst["ik"] = max(worst["ik"], row["ik_mm"])
        worst["step"] = max(worst["step"], row["step"])
    out["b"] = dict(worst=worst, rows=rows if verbose else None)
    # (c) A closes across the width over the folded blade at the push's x; B still on the button
    x_end = PIVOT[0] - s_push
    gc, rc, wc = pose_a_hold(key, x_end, FOB[1] + 0.0003)
    qc, ec, rec, mc = solve(side_a, gc, rc, start=q)
    hc = hulls(side_a, qc, wc)
    out["c"] = dict(x_mm=round(1000 * x_end, 1), ik_mm=round(ec, 2), margin=round(mc, 1), j6=round(math.degrees(qc[5]), 0),
                    step_from_push=round(float(np.degrees(np.abs(qc - q)).max()), 1), hands_mm=round(1000 * hands_apart(hc, hb)[0], 1),
                    a_fob_mm=round(1000 * to_box(hc, key.fob_box(), skip_pads=True)[0], 1))  # fmt: skip
    return out


def _rear(h, g, r, w) -> bool:
    """Whether hull h belongs to the finger on the -closing side (the rear finger that pushes)."""
    return float(np.dot(h.vertices.mean(0) - g, r[:, 0])) < 0.0


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "plan"
    if mode == "plan":
        res = check_plan(math.pi, (0.25, 0.0, TOP + 0.10), math.radians(135), -0.019, 0.045, 0.070, verbose=True)
        if len(sys.argv) > 2:
            with open(sys.argv[2], "w") as f:
                json.dump(res, f, indent=1)
        print("a", res["a"])
        print("b worst", res["b"]["worst"])
        print("c", res["c"])
        for row in res["b"]["rows"]:
            print(" ".join(f"{k}={v}" for k, v in row.items()))
