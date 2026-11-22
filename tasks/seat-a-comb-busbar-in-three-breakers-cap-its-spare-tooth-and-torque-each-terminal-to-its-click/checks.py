#!/usr/bin/env python3
"""Offline reach, joint-margin, bench, body-clearance and hand-separation checks for ``comb_busbar`` (no simulator).

On yam_arena's URDF and collision hulls, for every pose of the demo (``planned()``, which ``demo.py`` drives):
1. Left hand, the comb: pinched end to end on its rack, its fourth tooth into the cap, its other three teeth 6 mm into
   the breakers' slots (where it lets go: the comb slides home by its weight), the closed fingers on the bar's top.
2. Left hand, the driver: its T-handle pinched across its 22 mm in the holster (13 mm off its centre), pulled out; at
   each screw in front of the access hole, in it, and the whole stroke: 178 deg to 2 deg short of the clamp's touch,
   then the last turn (at most 22 deg, to 2.0 N m and the click).
3. Right hand: for that last turn (from roll 140 deg) it presses the T-handle's lower arm 36 mm from the axis with its
   closed fingertips' end, pointing 60 deg from down towards +y (30 N: 1.1 N m of the click's 2.0), turning with it.
4. Each pose against the panel, holster and stand (least distance), the bench, and the other hand.

    python3 checks.py
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
from mechkit import quat_to_matrix  # noqa: E402

TOP = TABLE_TOP["station"]
BASES = {side: np.array(station.BASES[side], dtype=float) for side in ("left", "right")}
HAND_LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")
LINK_HULLS = analysis.link_hulls()
UP = np.array([0.0, 0.0, 1.0])

COMB_SQUEEZE = P.COMB_L - 0.010  # commanded opening on the comb's ends (5 mm a side into them: 25 N a finger at the gripper's 5,000 N/m)
COMB_OPEN = P.COMB_L + 0.016
CAP_IN = 0.0245  # m: tooth tip this far into the cap (snaps at 22)
TEETH_IN = 0.006  # m: the teeth's tips this far into the slots when the hand lets go (held there by the slots' walls)
OVER_SLOTS = 0.0025  # m: the tips this far over the breakers' tops where the comb is first aimed
DRV_SQUEEZE = P.TB_THICK - 0.014  # commanded opening on the T-handle (7 mm a side: 35 N a finger, under the 40 N allowed)
DRV_OPEN = P.TB_THICK + 0.022
FRONT = 0.015  # m: bit tip in front of the hole's floor before going in
NEAR = 0.0105  # m: then this far, the tip 2 mm in front of the face, where it is aimed once more
PULL = 0.048  # m: the driver drawn back out of its holster before it is lifted: the bit's tip 8.7 mm out of the tube
J6_START_DEG = -107.0  # the left wrist's last joint where a screw's stroke starts (its limit -120)
RUN_DEG = 178.0  # the left hand runs the screw down to 2 deg short of the clamp's touch (0.1 N m)
FIN_DRV_DEG = 24.0  # then the driver turns about 5 + 15 deg to 2.0 N m and slips 3 deg (the click): the right hand follows it at most this far
FIN_HAND_DEG = 22.0  # the left hand's last turn, at most (8 steps of 2.75 deg); the hands' couple turns the driver on ahead of it, both stop at the click
SNUG_HAND_DEG = 8.0  # --snug: the left hand alone, the screw closed on the tooth a few deg past the touch (about 0.5 N m), no click
PUSH_N = 30.0  # N: the right hand's push on the T-handle's lower arm, 36 mm from the axis (1.1 N m), and the left hand's equal
# and opposite push through its pads, 13 mm from the axis on the other side (0.4 N m): no net force on the driver
PUSH_R = 0.036  # m: where the right fingertips press the arm, from the axis (the arm ends at 49.5 mm)
_T = math.radians(60.0)
_A = np.array([0.0, math.sin(_T), -math.cos(_T)])  # right hand closed, pointing down and 60 deg towards +y (the arm's way round)
_U = -(P.driver_rot(math.radians(140.0))[:, 2])  # the lower arm at the last turn's start
_U = -_U if _U[2] > 0.0 else _U
_C = _U - float(np.dot(_U, _A)) * _A
PUSH_ROT = np.column_stack([_C / np.linalg.norm(_C), np.cross(_A, _C / np.linalg.norm(_C)), _A])  # closing along the arm: the
# closed fingertips' 7.8 mm along it, their end pressing its face
PUSH_OPEN = 0.0
PUSH_END = 0.0128  # m: the closed fingertips' end past the grasp point along the approach (URDF hulls)
FIN_ALPHA = math.radians(140.0)  # the last turn is made from this roll: the lower arm then points down and to -y, its way
# round 34 deg below level towards +y


def frame(closing, approach) -> np.ndarray:
    a = np.asarray(approach, dtype=float)
    a = a / np.linalg.norm(a)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c = c / np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def solve(side: str, pos, rot, start=None, flips: bool = False) -> tuple[np.ndarray, float, float, float]:
    """Best IK solution: (q, pos_err_mm, rot_err_deg, smallest_margin_deg); ``flips``: also the rotation turned half a
    turn about the approach (the same grasp for a symmetric part). The rotation chosen is left in ``SOLVED_ROT[0]``."""
    best = None
    seeds = ([start] if start is not None else []) + [np.array(s, float) for s in (
        [0.0, 1.0, 1.0, -0.6, 0.0, 0.0], [0.0, 1.6, 1.4, -0.4, 0.0, 0.0], [0.0, 0.6, 0.6, -1.2, 0.0, 0.0],
        [0.0, 1.2, 1.8, 0.4, 0.0, 0.0], [0.0, 1.0, 1.0, 0.6, 0.0, 1.5], [0.0, 1.0, 1.0, 0.6, 0.0, -1.5])]  # fmt: skip
    for r_try in ([rot, rot @ np.diag([-1.0, -1.0, 1.0])] if flips else [rot]):
        for q0 in seeds:
            q, e, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(r_try), q0, base_pos=BASES[side], iters=160)
            margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
            ok = e < 0.0005 and re < 0.01
            key = (ok, margin if ok else -(e + 0.05 * re))
            if best is None or key > best[0]:
                best = (key, q, 1e3 * float(e), math.degrees(float(re)), margin, r_try)
            if start is not None and ok:
                break
    assert best is not None
    SOLVED_ROT[0] = best[5]
    return best[1], best[2], best[3], best[4]


SOLVED_ROT = [None]


def hulls(side: str, q: np.ndarray, opening_m: float) -> list:
    joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, q)}, **dict.fromkeys(C.FINGER_JOINTS, -0.5 * opening_m)}
    poses = analysis.fk_link_poses_world(joints, tuple(BASES[side]))
    out = []
    for link in HAND_LINKS:
        for h in LINK_HULLS.get(link, ()):
            r, p = poses[link][:3, :3], poses[link][:3, 3]
            n_w = h.planes[:, :3] @ r.T
            out.append(analysis.Hull(vertices=h.vertices @ r.T + p, planes=np.c_[n_w, h.planes[:, 3] + n_w @ p]))
    return out


def held_opening(op: float, name: str) -> float:
    """The gripper's opening as the hulls stand: a commanded squeeze stops on the part (the comb's 72 mm length, the
    T-handle's 22 mm)."""
    if op <= 0.0:
        return op
    return max(op, P.COMB_L) if name.startswith("L_") else max(op, P.TB_THICK)


def lowest_mm(hs) -> float:
    return 1e3 * (min(float(h.vertices[:, 2].min()) for h in hs) - TOP)


def apart_mm(ha, hb) -> float:
    best = np.inf
    for a in ha:
        lo_a, hi_a = a.vertices.min(0), a.vertices.max(0)
        for b in hb:
            lo_b, hi_b = b.vertices.min(0), b.vertices.max(0)
            if np.maximum(lo_b - hi_a, lo_a - hi_b).max() > 0.06:
                continue
            best = min(best, float(analysis._signed(a.vertices, b).min()), float(analysis._signed(b.vertices, a).min()))
    return 1e3 * float(best)


def boxes(name: str) -> list[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """World OBBs (centre, rotation, size) of a fixed body's solid boxes."""
    if name == "panel":
        shapes, to_w, rot = P.panel_shapes(), (lambda p: P.pw(p)), P.TILT
    else:
        shapes = {"holster": P.holster_shapes, "stand": P.stand_shapes}[name]()
        to_w, rot = (lambda p: np.asarray(p, float)), np.eye(3)
    return [(to_w(s.pos) + TOP * UP, rot @ quat_to_matrix(s.quat), np.asarray(s.dims, float)) for s in shapes if s.solid and s.kind == "box"]


BOXES: dict = {}


def to_body_mm(hs, name: str) -> float:
    if name not in BOXES:
        BOXES[name] = boxes(name)
    best = np.inf
    for c, r, size in BOXES[name]:
        reach = float(np.linalg.norm(0.5 * size)) + 0.15
        for h in hs:
            if float(np.linalg.norm(h.vertices.mean(0) - c)) > reach:
                continue
            best = min(best, float(analysis.box_signed_distance(h.vertices, c, r, size).min()))
    return 1e3 * best


def obbs(shapes, pos, rot) -> list[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """World OBBs (centre, rotation, size) of a body's solid shapes at ``pos``, ``rot`` (a cylinder as its box)."""
    pos, rot = np.asarray(pos, float), np.asarray(rot, float)
    return [(pos + rot @ np.asarray(s.pos, float), rot @ quat_to_matrix(s.quat), 2.0 * s.half_extents()) for s in shapes if s.solid]


def surface_points(c, r, size, step: float = 0.0015) -> np.ndarray:
    """A grid of points (spacing <= ``step``) on the six faces of an OBB."""
    size = np.asarray(size, float)
    axes = [np.linspace(-0.5 * s, 0.5 * s, max(2, int(math.ceil(s / step)) + 1)) for s in size]
    pts = []
    for f in range(3):
        a, b = [i for i in range(3) if i != f]
        ga, gb = np.meshgrid(axes[a], axes[b], indexing="ij")
        for sign in (-1.0, 1.0):
            loc = np.zeros((ga.size, 3))
            loc[:, a], loc[:, b], loc[:, f] = ga.ravel(), gb.ravel(), sign * 0.5 * size[f]
            pts.append(loc)
    return np.asarray(c, float) + np.vstack(pts) @ np.asarray(r, float).T


def fixed_obbs() -> dict:
    """World OBBs of what the carried comb and the hands must clear: the panel with its clamp plates, the stand, the
    holster, the driver and the cap where they start."""
    lean = P.TILT
    jaws = []
    for i in range(3):
        jaws += obbs(P.jaw_shapes(), P.pw(P.jaw_pos(i, P.GAP0)) + TOP * UP, lean)
    d_pos, d_rot = P.driver_start()
    return {"panel": obbs(P.panel_shapes(), P.pw(np.zeros(3)) + TOP * UP, lean) + jaws,
            "stand": obbs(P.stand_shapes(), TOP * UP, np.eye(3)), "holster": obbs(P.holster_shapes(), TOP * UP, np.eye(3)),
            "driver": obbs(P.driver_shapes(), d_pos + TOP * UP, d_rot),
            "cap": obbs(P.cap_shapes(), P.cap_origin() + (0.5 * P.CAP_SIZE[2]) * P.ZP + TOP * UP, lean)}  # fmt: skip


def comb_obbs(tcp, capped: bool) -> list:
    """The comb (and the cap on its fourth tooth) held at ``tcp`` at the panel's lean, its bar's centre at the grasp."""
    out = obbs(P.comb_shapes(), tcp, P.TILT)
    if capped:  # the cap's top end CAP_IN over the fourth tooth's tip
        local = P.tooth_tip_local(3) + np.array([0.0, 0.0, CAP_IN - 0.5 * P.CAP_SIZE[2]])
        out.append((np.asarray(tcp, float) + P.TILT @ local, P.TILT, np.asarray(P.CAP_SIZE, float)))
    return out


def obb_clearance(a_list, a_pts, b_list, b_pts) -> float:
    """Least signed distance (m) between two sets of OBBs, from each set's surface points to the other set's boxes."""
    best = np.inf
    for (ca, ra, sa), pa in zip(a_list, a_pts):
        reach_a = 0.5 * float(np.linalg.norm(sa))
        for (cb, rb, sb), pb in zip(b_list, b_pts):
            if float(np.linalg.norm(ca - cb)) > reach_a + 0.5 * float(np.linalg.norm(sb)) + 0.02:
                continue
            best = min(best, float(analysis.box_signed_distance(pa, cb, rb, sb).min()), float(analysis.box_signed_distance(pb, ca, ra, sa).min()))
    return best


def segments(pl: dict, names: list[str], step: float = 0.003):
    """Points every ``step`` (m) along the straight lines between the named poses: (from, to, tcp, rot, opening)."""
    for a, b in zip(names[:-1], names[1:]):
        (pa, ra, oa), (pb, rb, ob) = pl[a], pl[b]
        n = max(1, int(math.ceil(float(np.linalg.norm(np.asarray(pb) - np.asarray(pa))) / step)))
        for k in range(n + 1):
            s = k / n
            yield a, b, np.asarray(pa) + s * (np.asarray(pb) - np.asarray(pa)), ik.slerp_matrix(ra, rb, s), oa + s * (ob - oa)


COMB_PATHS = {  # the demo's comb moves (its fourth tooth into the cap, or bare), and which part of each the comb is held in
    "capped": ["L_lift", "L_over_cap", "L_cap", "L_capped", "L_high_cap", "L_high_slots", "L_over_slots", "L_teeth_in"],
    "bare": ["L_lift", "L_high_cap", "L_high_slots", "L_over_slots", "L_teeth_in"],
}
INTO = {("L_over_cap", "L_cap"), ("L_cap", "L_capped"), ("L_over_slots", "L_teeth_in")}  # moves into a socket: listed, not held to the 3 mm
BESIDE = {("L_high_slots", "L_over_slots"): 2.5}  # the capped spare tooth comes down 2.75 mm beside breaker 1 (the 18 mm pitch)


def sweep(pl: dict) -> dict:
    """Carried comb and left hand along the comb's moves, and the left hand along the driver's: least clearance to each
    fixed body per move (mm)."""
    fixed = fixed_obbs()
    pts = {k: [surface_points(*o) for o in v] for k, v in fixed.items()}
    flip = {tag: grasp_flip(pl, tag) for tag in ("L_", "D_")}
    out: dict = {}
    for path, names in COMB_PATHS.items():
        capped_from = names.index("L_cap") if "L_cap" in names else len(names)
        q = None
        for a, b, tcp, rot, op in segments(pl, names):
            capped = names.index(a) >= capped_from
            comb = comb_obbs(tcp, capped)
            comb_pts = [surface_points(*o) for o in comb]
            q, *_ = solve("left", tcp, rot @ flip["L_"], start=q)
            hs = hulls("left", q, held_opening(op, "L_"))
            row = out.setdefault(f"{path}: {a} -> {b}", {})
            for body in ("panel", "stand", "holster", "driver") + (() if capped else ("cap",)):
                if body != "cap":
                    row[f"comb_{body}"] = min(row.get(f"comb_{body}", np.inf), 1e3 * obb_clearance(comb, comb_pts, fixed[body], pts[body]))
                row[f"hand_{body}"] = min(row.get(f"hand_{body}", np.inf), hand_to(hs, fixed[body]))
    q = None
    for a, b, tcp, rot, op in segments(pl, ["D_behind_holster", "D_holster", "D_pulled", "D_lifted"]):
        q, *_ = solve("left", tcp, rot @ flip["D_"], start=q)
        hs = hulls("left", q, held_opening(op, "D_"))
        row = out.setdefault(f"driver: {a} -> {b}", {})
        for body in ("panel", "stand", "cap"):
            row[f"hand_{body}"] = min(row.get(f"hand_{body}", np.inf), hand_to(hs, fixed[body]))
    return {k: {kk: round(float(vv), 1) for kk, vv in v.items()} for k, v in out.items()}


def grasp_flip(pl: dict, tag: str) -> np.ndarray:
    """One grasp for every pose of a part (names starting ``tag``): of the two turns about the approach, the one whose
    least joint margin over those poses is larger (``demo.py`` holds the part so)."""
    best = None
    for f in (np.eye(3), np.diag([-1.0, -1.0, 1.0])):
        worst, q = 1e9, None
        for n in [n for n in pl if n.startswith(tag)]:
            q, e, re, mg = solve("left", pl[n][0], pl[n][1] @ f, start=q)
            worst = min(worst, mg if (e < 0.5 and re < 0.5) else -1e3)
        if best is None or worst > best[0]:
            best = (worst, f)
    return best[1]


def hand_to(hs, boxes_) -> float:
    best = np.inf
    for c, r, size in boxes_:
        reach = 0.5 * float(np.linalg.norm(size)) + 0.15
        for h in hs:
            if float(np.linalg.norm(h.vertices.mean(0) - c)) > reach:
                continue
            best = min(best, float(analysis.box_signed_distance(h.vertices, c, r, size).min()))
    return 1e3 * best


def rot_about(axis, angle: float) -> np.ndarray:
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def drv_rot(alpha: float) -> np.ndarray:
    """Gripper frame holding the driver: approach along the screws' axis, closing across the T-handle at roll ``alpha``
    (the driver's own frame is then ``parts.driver_rot(alpha)``: its thickness along the closing axis)."""
    return frame(math.cos(alpha) * P.YP + math.sin(alpha) * P.ZP, P.XP)


def driver_at_screw(i: int) -> np.ndarray:
    """The driver's origin (its T-handle's centre) with the bit's tip on hole ``i``'s floor."""
    tip = P.pw(P.screw_pos(i) + np.array([P.HOLE_FLOOR - P.HEAD_FACE - 0.0005, 0.0, 0.0])) + TOP * UP
    return tip - P.TIP_X * P.XP


def in_tcp(i: int, alpha: float) -> np.ndarray:
    """The left grasp point with the bit in hole ``i`` and the driver at roll ``alpha`` (the pads off its axis)."""
    return driver_at_screw(i) + P.driver_rot(alpha) @ np.asarray(P.GRASP_OFF)


def front_tcp(i: int, alpha: float) -> np.ndarray:
    return in_tcp(i, alpha) - FRONT * P.XP


def push_contact(dp: np.ndarray, dr: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """For the driver at ``dp``, ``dr``: the point on its lower arm's face the right fingertip presses, the face's
    outward normal there, and the arm's way round (unit) when the screw is tightened."""
    span, thick = dr[:, 2], dr[:, 1]
    u = span if span[2] < 0.0 else -span  # the lower arm
    way = np.cross(P.XP, u)
    way = way / np.linalg.norm(way)
    n = thick if float(np.dot(thick, way)) < 0.0 else -thick  # the face the push goes into
    return dp + PUSH_R * u + 0.5 * P.TB_THICK * n, n, way


def push_tcp(contact: np.ndarray, rot: np.ndarray) -> np.ndarray:
    """The right grasp point with its closed fingertips' end on ``contact``."""
    return contact - PUSH_END * rot[:, 2]


def comb_centre_for_tip(k: int, tip_w: np.ndarray) -> np.ndarray:
    return tip_w - P.TILT @ P.tooth_tip_local(k)


def planned() -> dict:
    """Every fixed pose of the demo: name -> (tcp, rot, opening). L_: the comb; D_: the driver (both left hand)."""
    down_y = frame((0.0, 1.0, 0.0), (0.0, 0.0, -1.0))
    rack = P.rack_origin() + P.TILT @ np.array([0.0, 0.0, 0.5 * P.COMB_BAR[2] + P.OVER]) + TOP * UP
    cap_top = P.cap_origin() + (P.CAP_SIZE[2] + P.OVER) * P.ZP + TOP * UP
    cap_in = comb_centre_for_tip(3, cap_top - CAP_IN * P.ZP)
    seat = P.pw(P.comb_seated_panel(0)) + TOP * UP
    bar_top = seat + 0.5 * P.COMB_BAR[2] * P.ZP
    out = {
        "L_over_rack": (rack + 0.040 * UP, down_y, COMB_OPEN), "L_rack": (rack, down_y, COMB_OPEN),
        "L_lift": (rack + 0.035 * UP, down_y, COMB_SQUEEZE), "L_over_cap": (cap_in + 0.036 * P.ZP, down_y, COMB_SQUEEZE),
        "L_cap": (cap_in, down_y, COMB_SQUEEZE), "L_capped": (cap_in + 0.036 * P.ZP, down_y, COMB_SQUEEZE),
        "L_high_cap": (cap_in + 0.036 * P.ZP + np.array([0.0, 0.0, 0.150 + TOP - (cap_in + 0.036 * P.ZP)[2]]), down_y, COMB_SQUEEZE),
        "L_high_slots": (seat + (P.PIN_L + OVER_SLOTS) * P.ZP + np.array([0.0, 0.0, 0.150 + TOP - (seat + (P.PIN_L + OVER_SLOTS) * P.ZP)[2]]), down_y, COMB_SQUEEZE),
        "L_over_slots": (seat + (P.PIN_L + OVER_SLOTS) * P.ZP, down_y, COMB_SQUEEZE),
        "L_teeth_in": (seat + (P.PIN_L - TEETH_IN) * P.ZP, down_y, COMB_SQUEEZE),
        "L_let_go": (seat + (P.PIN_L - TEETH_IN) * P.ZP, down_y, COMB_OPEN),
        "L_press": (bar_top + (0.0128 + 0.0005) * UP, down_y, 0.0),
    }
    d0 = P.driver_rot(P.HOLSTER_ROLL)
    g = np.asarray(P.HOLSTER_G, float) + TOP * UP + d0 @ np.asarray(P.GRASP_OFF)
    r0 = drv_rot(P.HOLSTER_ROLL)
    out.update({
        "D_behind_holster": (g - 0.045 * P.XP, r0, DRV_OPEN), "D_holster": (g, r0, DRV_OPEN),
        "D_pulled": (g - PULL * P.XP, r0, DRV_SQUEEZE), "D_lifted": (g - PULL * P.XP + 0.018 * UP, r0, DRV_SQUEEZE),
        "D_ready": (front_tcp(0, P.HOLSTER_ROLL), r0, DRV_SQUEEZE),
    })
    return out


def sweep_start(i: int, q_seed=None) -> tuple[float, np.ndarray]:
    """alpha at which the left wrist's last joint stands at J6_START_DEG with the bit in hole ``i`` (j6 follows alpha
    one for one about the approach); the IK solution there."""
    alpha = math.radians(135.0)
    q, *_ = solve("left", in_tcp(i, alpha), drv_rot(alpha), start=q_seed)
    for _ in range(5):
        alpha += math.radians(J6_START_DEG) - float(q[5])
        q, *_ = solve("left", in_tcp(i, alpha), drv_rot(alpha), start=q)
    return alpha, q


def main() -> dict:
    pl = planned()
    out: dict = {"left": {}, "screws": {}}
    q_prev, hands = None, {}
    flip = {tag: grasp_flip(pl, tag) for tag in ("L_", "D_")}
    for name in pl:
        tcp, rot, op = pl[name]
        q, e, re, mg = solve("left", tcp, rot @ flip[name[:2]], start=q_prev)
        q_prev = q
        hs = hulls("left", q, held_opening(op, name))
        hands[name] = hs
        out["left"][name] = {"ik_mm": round(e, 3), "margin_deg": round(mg, 1), "z_mm": round(1e3 * (tcp[2] - TOP), 1), "low_mm": round(lowest_mm(hs), 1),
                             "to_panel_mm": round(to_body_mm(hs, "panel"), 1), "to_stand_mm": round(to_body_mm(hs, "stand"), 1),
                             "to_holster_mm": round(to_body_mm(hs, "holster"), 1)}  # fmt: skip
    from yam_arena.constants import READY

    q_ready = np.array(READY, float)
    right_ready = hulls("right", q_ready, 0.094)
    out["apart_right_ready_mm"] = round(min(apart_mm(hs, right_ready) for hs in hands.values()), 1)
    for i in range(3):
        alpha0, q = sweep_start(i, q_prev)
        rows, q_r = [], None
        plan = [(alpha0 + math.radians(RUN_DEG) * k / 8, None) for k in range(9)]
        plan += [(FIN_ALPHA + math.radians(FIN_HAND_DEG) * k / 4, min(FIN_DRV_DEG, FIN_HAND_DEG * k / 4)) for k in range(5)]
        for al, drv_turn in plan:
            tcp = in_tcp(i, al)
            q, e, re, mg = solve("left", tcp, drv_rot(al), start=q)
            hs = hulls("left", q, P.TB_THICK)
            row = {"deg": round(math.degrees(al), 1), "ik_mm": round(e, 3), "margin_deg": round(mg, 1), "j6_deg": round(math.degrees(q[5]), 1),
                   "z_mm": round(1e3 * (tcp[2] - TOP), 1), "low_mm": round(lowest_mm(hs), 1), "to_panel_mm": round(to_body_mm(hs, "panel"), 1)}  # fmt: skip
            if drv_turn is not None:  # the last turn: the right hand presses the lower arm
                dr = P.driver_rot(FIN_ALPHA + math.radians(drv_turn))
                contact, n, way = push_contact(driver_at_screw(i), dr)
                rot_r = rot_about(P.XP, math.radians(drv_turn)) @ PUSH_ROT  # the hand turns with the arm
                tcp_r = push_tcp(contact, rot_r)
                q_r, er, rer, mr = solve("right", tcp_r, rot_r, start=q_r, flips=True)
                hr_ = hulls("right", q_r, PUSH_OPEN)
                row.update(right_ik_mm=round(er, 3), right_margin=round(mr, 1), right_z_mm=round(1e3 * (tcp_r[2] - TOP), 1),
                           apart_mm=round(apart_mm(hs, hr_), 1), right_to_panel_mm=round(to_body_mm(hr_, "panel"), 1),
                           push_way=[round(float(v), 2) for v in way])  # fmt: skip
            rows.append(row)
        out["screws"][i + 1] = {"alpha_start_deg": round(math.degrees(alpha0), 1), "rows": rows}
        q_prev = q
    out["sweep"] = sweep(pl)
    for name, r in out["left"].items():
        print("left", name, json.dumps(r))
    for name, r in out["sweep"].items():
        print("sweep", name, json.dumps(r))
    for i, s in out["screws"].items():
        print("screw", i, "alpha0", s["alpha_start_deg"], json.dumps(s["rows"]))
    print("apart_right_ready_mm", out["apart_right_ready_mm"])

    for name, r in out["left"].items():
        assert r["ik_mm"] <= 0.5 and r["margin_deg"] >= 5.0 and r["low_mm"] >= 2.0 and r["z_mm"] <= 160.0, (name, r)
    for name in ("L_over_rack", "L_lift", "L_over_cap", "L_cap", "L_capped", "L_over_slots", "L_teeth_in", "D_behind_holster", "D_pulled", "D_lifted"):
        r = out["left"][name]
        assert min(r["to_panel_mm"], r["to_stand_mm"]) >= 1.0, (name, r)
    for i, s in out["screws"].items():
        for r in s["rows"]:
            assert r["ik_mm"] <= 0.5 and r["margin_deg"] >= 5.0 and r["z_mm"] <= 160.0 and r["to_panel_mm"] >= 3.0, (i, r)
            if "right_ik_mm" in r:
                assert r["right_ik_mm"] <= 0.5 and r["right_margin"] >= 5.0 and r["apart_mm"] >= 20.0 and r["right_z_mm"] <= 160.0, (i, r)
    assert out["apart_right_ready_mm"] >= 20.0
    for name, r in out["sweep"].items():
        move = tuple(name.split(": ")[1].split(" -> "))
        if move not in INTO:
            assert min(r.values()) >= BESIDE.get(move, 3.0), (name, r)
    return out


assert abs(float(np.linalg.det(PUSH_ROT)) - 1.0) < 1e-9


if __name__ == "__main__":
    main()
