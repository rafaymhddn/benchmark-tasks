#!/usr/bin/env python3
"""The hands' reach and the start of the MRI tray task.

1. Without the simulator: every pose the job needs — for each steel item a pre-grasp 60 mm over it, the grasp
   itself (down, closing across the named part: the clamp's T-handle, the padlock's body, the shears' raised
   rings), the lift, the carry over the bin and the open-fingered release at the bin's mouth, then the retreat —
   each solved by the arm's inverse kinematics (within 0.5 mm and 0.6 degrees, every joint 6 degrees or more
   inside its limits); the working hand's hulls clear of the tray's walls, the bin and every item it does not
   grasp; the parked hand at READY 200 mm or more from the working one. The shears' ring grasp is the proof the
   flat shears can be picked: the pads dip 12.8 mm below the grasp point, so the rings' 30 mm lift is what makes
   it possible — the printed clearance numbers are the proof.
2. ``--frame0 <dir>`` in the simulator: the first pictures of the three cameras (as rendered and at the size a
   policy gets) and 2 s at rest: every body still, the score 0, no zero rule.

    scripts/rta simpy <id> mech/tasks/mri_cart_clear/checks.py [--frame0 /workspace/robocurve/outputs/<id>]
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
MAX_JAW = 2.0 * abs(float(C.FINGER_OPEN))
PAD_DIP = 0.0128  # m: how far below the grasp point the fingertips reach (the URDF hulls of the closed hand)
IK_MM, IK_DEG, MARGIN_DEG, SEPARATION = 0.5, 0.6, 6.0, 0.200


def frame(closing, approach) -> np.ndarray:
    """Gripper rotation: columns closing axis, pads' width axis, approach axis."""
    a = np.asarray(approach, dtype=float) / np.linalg.norm(approach)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


DOWN_X = frame((1.0, 0.0, 0.0), (0.0, 0.0, -1.0))  # closing across x on the tray: clamp T-handle, padlock, shears rings
DOWN_BIN = frame((0.0, -1.0, 0.0), (0.0, 0.0, -1.0))  # turned -90 deg about z: aligns the 212 mm shears with the bin's 333 mm length
CARRY_BIN = frame((0.0, -1.0, 0.0), (0.5, 0.0, -math.sqrt(3.0) / 2.0))  # 30 deg forward pitch: lifts the 120 mm clamp clear of the 149 mm bin
GRIP = {"clamp": 0.000, "padlock": 0.010, "shears": 0.053}  # m target between pads: ~5 mm squeeze/finger (~25 N) on the part
OPEN = {"clamp": 0.036, "padlock": 0.046, "shears": 0.076}  # m: the fingers open clear around each part before closing
LIFT, PRE = 0.065, 0.060  # m: straight up off the tray, and over the item before descending
BIN_Z = {"padlock": 0.195, "clamp": 0.235, "shears": P.RELEASE_Z}  # m over the bench at release
BIN_ROT = {"padlock": CARRY_BIN, "clamp": CARRY_BIN, "shears": DOWN_BIN}


def poses() -> dict:
    """Every pose the plan holds, by name: (side, position in the world, rotation, finger opening m)."""
    out = {}
    for name in P.STEEL:
        gx, gy, gz = P.GRASPS[name]
        g = np.array([gx, gy, TOP + gz], dtype=float)
        rot = DOWN_X
        out[f"{name}: before it"] = ("right", g + (0.0, 0.0, PRE), rot, 1.0)
        out[f"{name}: around it"] = ("right", g, rot, OPEN[name])
        out[f"{name}: closed on it"] = ("right", g, rot, GRIP[name])
        lift_rot = CARRY_BIN if name == "clamp" else rot
        lift_dz = 0.160 if name == "clamp" else LIFT
        out[f"{name}: lifted"] = ("right", g + (0.0, 0.0, lift_dz), lift_rot, GRIP[name])
        rx, ry = P.RELEASES[name]
        bz, brot = BIN_Z[name], BIN_ROT[name]
        out[f"{name}: over the bin"] = ("right", np.array([rx, ry, TOP + bz]), brot, GRIP[name])
        out[f"{name}: let go"] = ("right", np.array([rx, ry, TOP + bz]), brot, 1.0)
    out["clamp: high over the bin"] = ("right", np.array([P.RELEASES["clamp"][0], P.RELEASES["clamp"][1], TOP + 0.280]), CARRY_BIN, GRIP["clamp"])
    ready = ik.tcp_pose(np.array(READY, dtype=float), BASES["right"])
    out["both: ready"] = ("right", ready[0], ready[1], 1.0)
    out["both: parked"] = ("right", np.array([P.PARK[0], P.PARK[1], TOP + P.PARK[2]]), CARRY_BIN, 1.0)
    return out


def solve(side: str, pos, rot, start=None):
    best = None
    seeds = ([np.asarray(start, dtype=float)] if start is not None else []) + [np.array(s, dtype=float) for s in (
        READY, [0.0, 1.2, 1.2, -0.5, 0.0, 0.0], [0.0, 1.5, 1.5, -0.4, 0.0, 0.0], [0.6, 1.6, 1.2, -1.0, 0.5, 0.0],
        [-0.6, 1.6, 1.2, -1.0, 0.0, -0.6], [0.3, 1.0, 1.4, 0.4, 1.5, 0.0], [-0.3, 1.0, 1.4, 0.4, -1.5, 0.0])]  # fmt: skip
    for flip in (False, True):  # the two fingers are alike: the hand may be turned half round about its approach axis
        r = np.asarray(rot, dtype=float) @ (np.diag([-1.0, -1.0, 1.0]) if flip else np.eye(3))
        quat = ik.matrix_to_quat_xyzw(r)
        for q0 in seeds:
            q, pe, re = ik.solve(np.asarray(pos, dtype=float), quat, q0, base_pos=BASES[side], iters=300)
            margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
            ok = pe < IK_MM / 1000 and math.degrees(re) < IK_DEG
            key = (ok, margin if ok else -(pe * 1000.0 + math.degrees(re)))
            if best is None or key > best[0]:
                best = (key, q, float(pe * 1000.0), float(math.degrees(re)), margin, flip)
    return best[1], best[2], best[3], best[4], best[5]


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
    best = math.inf
    for _, a in hulls_a:
        for _, b in hulls_b:
            best = min(best, float(analysis._signed(a.vertices, b).min()), float(analysis._signed(b.vertices, a).min()))
    return best


def scene_boxes() -> dict:
    """The solid boxes the working hand must keep clear of: the tray's walls, the bin, and every item."""
    z = TOP + P.TRAY_FLOOR
    boxes = {"tray walls": [
        box_hull((P.TRAY_AT[0], P.TRAY_AT[1] + P.TRAY_W / 2, z + P.TRAY_RIM / 2), np.eye(3), (P.TRAY_D, P.TRAY_WALL * 2, P.TRAY_RIM)),
        box_hull((P.TRAY_AT[0], P.TRAY_AT[1] - P.TRAY_W / 2, z + P.TRAY_RIM / 2), np.eye(3), (P.TRAY_D, P.TRAY_WALL * 2, P.TRAY_RIM)),
        box_hull((P.TRAY_AT[0] + P.TRAY_D / 2, P.TRAY_AT[1], z + P.TRAY_RIM / 2), np.eye(3), (P.TRAY_WALL * 2, P.TRAY_W, P.TRAY_RIM)),
        box_hull((P.TRAY_AT[0] - P.TRAY_D / 2, P.TRAY_AT[1], z + P.TRAY_RIM / 2), np.eye(3), (P.TRAY_WALL * 2, P.TRAY_W, P.TRAY_RIM))]}
    boxes["tray walls"] = [("wall", b) for b in boxes["tray walls"]]
    boxes["bin"] = [("bin", box_hull((P.BIN_AT[0], P.BIN_AT[1], TOP + P.BIN_H / 2), np.eye(3), (P.BIN_L, P.BIN_W, P.BIN_H)))]
    sizes = {"clamp": (0.058, 0.034, 0.120), "padlock": (P.LOCK_D, P.LOCK_W, 0.046), "shears": (P.SHEARS_W, P.SHEARS_L, 0.036),
             "bottle": (P.BOTTLE_D, P.BOTTLE_D, P.BOTTLE_H), "dish": (P.DISH_W, P.DISH_L, P.DISH_H), "clipboard": (P.CLIP_W, P.CLIP_L, 0.018)}  # fmt: skip
    at = {"clamp": P.CLAMP_AT, "padlock": P.LOCK_AT, "shears": P.SHEARS_AT, "bottle": P.BOTTLE_AT, "dish": P.DISH_AT, "clipboard": P.CLIP_AT}
    heights = {"clamp": 0.060, "padlock": 0.023, "shears": 0.018, "bottle": P.BOTTLE_H / 2, "dish": P.DISH_H / 2, "clipboard": 0.009}
    boxes["items"] = [(n, box_hull((at[n][0], at[n][1], z + heights[n]), np.eye(3), sizes[n])) for n in sizes]
    return boxes


def run_offline() -> dict:
    found, rows, ok = {}, {}, True
    boxes = scene_boxes()
    for name, (side, pos, rot, opening) in poses().items():
        q, mm, deg, margin, flip = solve(side, pos, rot)
        found[name] = (side, q, opening)
        hulls = hand_hulls(side, q, opening)
        target = name.split(":")[0] if "the bin" not in name and "let go" not in name and "away" not in name else None
        clear = {}
        for what, group in boxes.items():
            keep = group if what != "items" else [(lbl, b) for lbl, b in group if lbl != target]
            clear[what] = round(1000 * gap(hulls, keep), 1)
        good = mm <= IK_MM and deg <= IK_DEG and margin >= MARGIN_DEG and min(clear.values()) > 0.0
        ok &= good
        rows[name] = {"side": side, "at (m)": [round(float(v), 4) for v in pos], "IK (mm, deg)": [round(mm, 3), round(deg, 3)],
                      "turned half round": flip, "least joint margin (deg)": round(margin, 1), "hulls clear of (mm)": clear, "ok": good}  # fmt: skip
    ready_left = hand_hulls("left", np.array(READY, dtype=float), MAX_JAW)
    pairs = {}
    for name, (side, q, opening) in found.items():
        tcp = float(np.linalg.norm(np.asarray(poses()[name][1]) - ik.tcp_pose(np.array(READY, dtype=float), BASES["left"])[0]))
        hull_gap = gap(ready_left, hand_hulls(side, q, opening))
        good = tcp >= SEPARATION and hull_gap > 0.0
        ok &= good
        pairs[f"{name} + parked left"] = {"grasp points apart (mm)": round(1000 * tcp, 1), "hulls apart (mm)": round(1000 * hull_gap, 1), "ok": good}
    return {"pass": bool(ok), "poses": rows, "against the parked hand": pairs,
            "pads dip below the grasp point (mm)": round(1000 * PAD_DIP, 1),
            "shears ring lift (mm)": round(1000 * P.SHEARS_RING_LIFT, 1)}


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
    moved = {name: round(1000 * float(np.linalg.norm(m.mk.pose(name)[0] - np.asarray(m.mk.built_pose(name)[0]))), 2) for name in m.P.BODIES}
    out = {"moved from built (mm)": moved, "score after 2 s": float(res.info.get("score", 0.0)), "report": m.report()}
    (out_dir / "g1_frame0.json").write_text(json.dumps(out, indent=1, default=str))
    return out


def main() -> int:
    if "--parts" in sys.argv:
        print(json.dumps(P.summary(), indent=1))
        return 0
    offline = run_offline()
    print("G1_OFFLINE " + json.dumps(offline, indent=1), flush=True)
    if "--frame0" in sys.argv:
        out = run_frame0(Path(sys.argv[sys.argv.index("--frame0") + 1]))
        print("G1_FRAME0 " + json.dumps(out, indent=1, default=str), flush=True)
    return 0 if offline["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
