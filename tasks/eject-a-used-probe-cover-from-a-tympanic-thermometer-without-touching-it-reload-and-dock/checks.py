#!/usr/bin/env python3
"""Reach, clearance, wrist separation, and start-state checks for the tympanic probe-cover task (Gate G1).

    scripts/rta simpy <id> mech/tasks/tympanic_cover/checks.py [--frame0 /workspace/robocurve/outputs/<id>]
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
from yam_arena import analysis, constants as C, ik, station
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, READY
from yam_arena.task_file import TABLE_TOP

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import parts as P  # noqa: E402

TOP = TABLE_TOP["station"]
BASES = {side: np.array(station.BASES[side], dtype=float) for side in ("left", "right")}
HAND_LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")
MAX_JAW = 2.0 * abs(float(C.FINGER_OPEN))
IK_MM, IK_DEG, MARGIN_DEG, WRIST_SEP_M, HULL_GAP_M = 0.5, 0.6, 6.0, 0.280, 0.020

OPEN_HANDLE = 0.068  # m around the 42 mm body (13 mm a side)
GRIP_HANDLE = 0.032  # m told on the 42 mm body: 5 mm a finger at 5000 N/m = 25 N a finger (<= 40 N)
OPEN_CRADLE = 0.080  # m around the cradle's 56 mm side walls (12 mm a side)
GRIP_CRADLE = 0.046  # m told on the 56 mm walls: 5 mm a finger = 25 N a finger (<= 40 N)
BACK = 0.060  # m: the right hand comes onto the tail along its approach (from behind and above) from this far back


FINGER_END = np.array([0.0, 0.0, 0.0128])  # the closed fingertips' flat end (8 x 15 mm), past the TCP along the approach


def left_press_poses() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(pre-press TCP, contact TCP, press direction): the closed fingertips' flat end 2 mm off eject_btn's face."""
    face_w, press_w = P.eject_button_world(TOP)
    contact_tcp = face_w - P.L_PRESS_ROT @ FINGER_END - 0.002 * press_w
    return contact_tcp - 0.040 * press_w, contact_tcp, press_w


def poses() -> dict[str, tuple[str, np.ndarray, np.ndarray, float]]:
    pick = np.array(P.R_PICK_TCP, dtype=float) + (0.0, 0.0, TOP)
    eject_r = np.array(P.R_EJECT_TCP, dtype=float) + (0.0, 0.0, TOP)
    load_r = np.array(P.R_LOAD_TCP, dtype=float) + (0.0, 0.0, TOP)
    dock_r = np.array(P.R_DOCK_TCP, dtype=float) + (0.0, 0.0, TOP)
    behind = BACK * P.R_LEVEL_ROT[:, 2]
    pre_l, contact_l, press_w = left_press_poses()
    grip_l = np.array(P.L_GRIP_TCP, dtype=float) + (0.0, 0.0, TOP)
    ready_r = ik.tcp_pose(np.array(READY, dtype=float), BASES["right"])
    ready_l = ik.tcp_pose(np.array(READY, dtype=float), BASES["left"])
    lv, ld = P.R_LEVEL_ROT, P.R_LOAD_ROT
    return {
        "right: pre-pick": ("right", pick - behind, lv, OPEN_HANDLE),
        "right: around handle": ("right", pick, lv, OPEN_HANDLE),
        "right: closed handle": ("right", pick, lv, GRIP_HANDLE),
        "right: lifted handle": ("right", pick + (0.0, 0.0, 0.050), lv, GRIP_HANDLE),
        "right: eject hold": ("right", eject_r, lv, GRIP_HANDLE),
        "left: pre-press": ("left", pre_l, P.L_PRESS_ROT, 0.0),
        "left: press eject": ("left", contact_l + 0.0055 * press_w, P.L_PRESS_ROT, 0.0),
        "left: pre-grip": ("left", grip_l - 0.050 * P.L_GRIP_ROT[:, 2], P.L_GRIP_ROT, OPEN_CRADLE),
        "left: grip cradle": ("left", grip_l, P.L_GRIP_ROT, GRIP_CRADLE),
        "right: over well": ("right", load_r + (0.0, 0.0, 0.040), ld, GRIP_HANDLE),
        "right: press well": ("right", load_r, ld, GRIP_HANDLE),
        "right: over dock": ("right", dock_r + (0.0, 0.0, 0.050), lv, GRIP_HANDLE),
        "right: dock": ("right", dock_r, lv, GRIP_HANDLE),
        "right: dock release": ("right", dock_r, lv, OPEN_HANDLE),
        "right: off dock": ("right", dock_r - behind, lv, OPEN_HANDLE),
        "right: ready": ("right", ready_r[0], ready_r[1], MAX_JAW),
        "left: ready": ("left", ready_l[0], ready_l[1], MAX_JAW),
    }


SIMULTANEOUS_PAIRS = (
    ("right: eject hold", "left: pre-press"),
    ("right: eject hold", "left: press eject"),
    ("right: over well", "left: grip cradle"),
    ("right: press well", "left: grip cradle"),
    ("right: over dock", "left: ready"),
    ("right: dock", "left: ready"),
    ("right: around handle", "left: ready"),
)


def solve(side: str, pos: np.ndarray, rot: np.ndarray):
    quat = ik.matrix_to_quat_xyzw(np.asarray(rot, dtype=float))
    best = None
    for s in (READY, [0.0, 1.2, 1.2, -0.5, 0.0, 0.0]):
        q, pe, re = ik.solve(np.asarray(pos, dtype=float), quat, np.array(s, dtype=float), base_pos=BASES[side], iters=120)
        margin = float(np.degrees(np.minimum(q - ik.LOWER, ik.UPPER - q)).min())
        ok = pe < IK_MM / 1000.0 and math.degrees(re) < IK_DEG
        key = (ok, margin if ok else -pe * 1000.0)
        if best is None or key > best[0]:
            best = (key, q, float(pe * 1000.0), float(math.degrees(re)), margin)
    return best[1], best[2], best[3], best[4]


def hand_hulls_and_wrist(side: str, q: np.ndarray, opening: float) -> tuple[list, np.ndarray]:
    half = -0.5 * max(0.0, min(MAX_JAW, opening))
    joints = {**{n: float(v) for n, v in zip(ARM_JOINTS, q)}, **dict.fromkeys(FINGER_JOINTS, half)}
    poses_w = analysis.fk_link_poses_world(joints, tuple(BASES[side]))
    out = []
    for link in HAND_LINKS:
        for h in analysis.link_hulls().get(link, []):
            r, p = poses_w[link][:3, :3], poses_w[link][:3, 3]
            n_w = h.planes[:, :3] @ r.T
            out.append((link, analysis.Hull(vertices=h.vertices @ r.T + p, planes=np.c_[n_w, h.planes[:, 3] + n_w @ p])))
    return out, poses_w["link5"][:3, 3].copy()


def gap(hulls_a: list, hulls_b: list) -> float:
    best = math.inf
    for _, a in hulls_a:
        for _, b in hulls_b:
            best = min(best, float(analysis._signed(a.vertices, b).min()), float(analysis._signed(b.vertices, a).min()))
    return best


def run_offline() -> dict:
    solved, rows, ok = {}, {}, True
    for name, (side, pos, rot, opening) in poses().items():
        q, mm, deg, margin = solve(side, pos, rot)
        hulls, wrist = hand_hulls_and_wrist(side, q, opening)
        solved[name] = (side, pos, q, opening, hulls, wrist)
        z_min_over_bench = min(float(h.vertices[:, 2].min() - TOP) for _, h in hulls)
        own_side = (pos[1] <= -0.020) if side == "right" else (pos[1] >= 0.005)
        height_ok = ("ready" in name) or ("pre-" in name) or ("lifted" in name) or ("over " in name) or ("off " in name) or (pos[2] - TOP <= 0.160)
        good = mm <= IK_MM and deg <= IK_DEG and margin >= MARGIN_DEG and z_min_over_bench > 0.002 and own_side and height_ok
        ok &= good
        rows[name] = {
            "side": side,
            "at (m)": [round(float(v), 4) for v in pos],
            "IK (mm, deg)": [round(mm, 3), round(deg, 3)],
            "least margin (deg)": round(margin, 1),
            "bench clear (mm)": round(1000.0 * z_min_over_bench, 1),
            "own side": bool(own_side),
            "ok": bool(good),
        }
    pairs = {}
    for rn, ln in SIMULTANEOUS_PAIRS:
        _, rpos, _, _, rh, rw = solved[rn]
        _, lpos, _, _, lh, lw = solved[ln]
        w_sep = float(np.linalg.norm(rw - lw))
        t_sep = float(np.linalg.norm(rpos - lpos))
        h_gap = gap(rh, lh)
        good = w_sep >= WRIST_SEP_M and h_gap >= HULL_GAP_M
        ok &= good
        pairs[f"{rn} || {ln}"] = {
            "tcp apart (mm)": round(1000.0 * t_sep, 1),
            "wrists apart (mm)": round(1000.0 * w_sep, 1),
            "hulls apart (mm)": round(1000.0 * h_gap, 1),
            "ok": bool(good),
        }
    return {"pass": bool(ok), "layout_ok": P.layout_ok(), "poses": rows, "simultaneous_pairs": pairs}


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
    moved = {name: round(1000.0 * float(np.linalg.norm(m.mk.pose(name)[0] - np.asarray(m.mk.built_pose(name)[0]))), 2) for name in m.P.BODIES}
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
