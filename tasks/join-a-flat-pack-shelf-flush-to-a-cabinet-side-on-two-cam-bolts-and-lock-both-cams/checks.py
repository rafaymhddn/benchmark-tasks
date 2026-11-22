#!/usr/bin/env python3
"""Offline reach, joint-limit, clearance, and height checks plus simulator start-picture and idle verification (Gate 1).

1. Offline (no simulator): every gripper pose used to pick/park both stubby screwdrivers, turn both cams open (90 deg CCW)
   and tight (180 deg CW in one continuous sweep), steady the loose shelf with the opposite hand, and push the shelf onto
   both 0.50 mm-play bolts with both hands:
   - IK error <= 0.5 mm and <= 0.6 deg;
   - every joint >= 6.0 deg inside its limits;
   - every grasp point <= 160 mm over the bench and finger opening <= 94 mm;
   - collision hulls clear of bench, upright cabinet side, support block, and cradles;
   - simultaneous two-hand pairs >= 200 mm apart with >= 20 mm hull-to-hull separation.
2. ``--frame0 <dir>`` in the simulator: saves ``start_top.png``, ``start_left_wrist.png``, ``start_right_wrist.png`` and
   their ``320 x 176`` policy-size images, steps 2.0 s idle at ready, and checks stability and score == 0.0.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
from yam_arena import analysis, constants as C, ik, station
from yam_arena.constants import READY
from yam_arena.demo import grip_rotation
from yam_arena.task_file import TABLE_TOP

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import parts as P  # noqa: E402

TOP = TABLE_TOP["station"]
BASES = {side: np.array(station.BASES[side], dtype=float) for side in ("left", "right")}
HAND_LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")
MAX_JAW = 2.0 * abs(float(C.FINGER_OPEN))  # ~0.094 m
IK_MM, IK_DEG, MARGIN_DEG, SEPARATION, HEIGHT_MAX = 0.5, 0.6, 6.0, 0.200, 0.160


def frame(closing, approach) -> np.ndarray:
    """Gripper rotation: columns are closing axis, finger-width axis, and approach axis."""
    a = np.asarray(approach, dtype=float) / np.linalg.norm(approach)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


_PITCH = math.radians(18.0)  # 18 deg downward approach onto the 16 mm shelf rear wing: >= 18 deg joint margin everywhere
SHELF_GRIP = frame((0.0, 0.0, -1.0), (math.cos(_PITCH), 0.0, -math.sin(_PITCH)))

SHELF_Y = 0.112  # m: +-112 mm rear-wing grasp lines (224 mm apart, aligned with +-110 mm bolts, overhanging 150 mm spine)
SHELF_IN = 0.003  # m: TCP 3 mm inside the shelf's rear -x edge (13 mm finger pads clamp 16 mm onto the board while palm stays 7 mm clear)
SHELF_BACK = 0.030  # m: pre-grasp / retreat behind the shelf's rear -x edge
Z_CRADLE_GRASP = P.CRADLE_H + P.DRV_BOLSTER_Z  # 0.056 m above bench
Z_CAM_SEATED = P.Z_SHELF_TOP - P.SLOT_RECESS_DEPTH + P.DRV_TIP_Z  # 0.1285 m above bench (tip on recess floor)
Z_TRANSIT = 0.150  # m above bench (<= 0.160 m hard limit; clears parked screwdriver butt and shelf)

# Yaw angles in yam_arena.demo.grip_rotation (increasing yaw = CCW about world +z; decreasing yaw = CW):
# - Open 90 deg CCW (both arms): -45 -> +45 deg (>= 30 deg joint margin at both cradles and both cams)
# - Lock 180 deg CW: left arm (+45 -> -15 -> -75 -> -138 deg); right arm (+135 -> +75 -> +15 -> -48 deg)
OPEN_YAWS = {"l": (-45.0, 45.0), "r": (-45.0, 45.0)}
LOCK_YAWS = {"l": (45.0, -15.0, -75.0, -138.0), "r": (135.0, 75.0, 15.0, -48.0)}
CRADLE_YAWS = (-45.0, 45.0)  # every cradle pickup and park uses +-45 deg (>= 30 deg joint margin at both cradles)


def down_yaw(yaw_deg: float) -> np.ndarray:
    """Gripper pointing straight down at ``yaw_deg`` CCW about world +z (decreasing yaw = clockwise turn)."""
    return grip_rotation(math.radians(yaw_deg))


def cradle_at(c: str, z_over: float, yaw_deg: float) -> tuple[np.ndarray, np.ndarray]:
    y = (1.0 if c == "l" else -1.0) * P.CRADLE_Y
    return np.array([P.CRADLE_X, y, TOP + z_over]), down_yaw(yaw_deg)


def cam_at(c: str, gap: float, z_over: float, yaw_deg: float) -> tuple[np.ndarray, np.ndarray]:
    x_shelf = P.X_SIDE - gap - P.SHELF_D / 2.0
    x_cam = x_shelf + P.CAM_X_IN_SHELF
    y_cam = (1.0 if c == "l" else -1.0) * P.CAM_Y
    return np.array([x_cam, y_cam, TOP + z_over]), down_yaw(yaw_deg)


def shelf_wing(c: str, gap: float, back: float = 0.0, up: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    x_rear = P.X_SIDE - gap - P.SHELF_D
    y = (1.0 if c == "l" else -1.0) * SHELF_Y
    return np.array([x_rear + SHELF_IN - back, y, TOP + P.Z_MID + up]), SHELF_GRIP


def poses() -> dict:
    """Every key pose of the plan: name -> (arm_side, world_pos, rot_3x3, jaw_opening_m, shelf_gap_m)."""
    out = {}
    drv_open = 0.048
    drv_held = P.DRV_GRIP_FLAT
    sh_open = 0.046
    sh_held = P.PANEL_T
    for c, side in (("l", "left"), ("r", "right")):
        y_o0, y_o1 = OPEN_YAWS[c]
        y_l0, y_l1, y_l2, y_l3 = LOCK_YAWS[c]
        for tag, ydeg in (("m45", -45.0), ("p45", 45.0)):
            out[f"{c}: cradle high {tag}"] = (side, *cradle_at(c, Z_TRANSIT, ydeg), drv_open, P.START_GAP)
            out[f"{c}: cradle grasp {tag}"] = (side, *cradle_at(c, Z_CRADLE_GRASP, ydeg), drv_held, P.START_GAP)
        for tag, ydeg in (("open0", y_o0), ("open1", y_o1)):
            out[f"{c}: cam start high {tag}"] = (side, *cam_at(c, P.START_GAP, Z_TRANSIT, ydeg), drv_held, P.START_GAP)
            out[f"{c}: cam start seated {tag}"] = (side, *cam_at(c, P.START_GAP, Z_CAM_SEATED, ydeg), drv_held, P.START_GAP)
        for tag, ydeg in (("lock0", y_l0), ("lock1", y_l1), ("lock2", y_l2), ("lock3", y_l3)):
            out[f"{c}: cam flush high {tag}"] = (side, *cam_at(c, 0.001, Z_TRANSIT, ydeg), drv_held, 0.001)
            out[f"{c}: cam flush seated {tag}"] = (side, *cam_at(c, 0.001, Z_CAM_SEATED, ydeg), drv_held, 0.001)
        out[f"{c}: wing behind start"] = (side, *shelf_wing(c, P.START_GAP, back=SHELF_BACK), sh_open, P.START_GAP)
        out[f"{c}: wing hold start"] = (side, *shelf_wing(c, P.START_GAP), sh_held, P.START_GAP)
        out[f"{c}: wing hold seated"] = (side, *shelf_wing(c, 0.001), sh_held, 0.001)
        out[f"{c}: wing behind seated"] = (side, *shelf_wing(c, 0.000, back=SHELF_BACK), sh_open, 0.000)
    return out


TOGETHER = (
    *(("l: wing hold start", f"r: {step}") for step in ("cradle grasp p45", "cam start high open0", "cam start seated open0", "cam start seated open1", "cradle grasp p45")),
    *((f"l: {step}", "r: wing hold start") for step in ("cradle grasp m45", "cam start high open0", "cam start seated open0", "cam start seated open1", "cradle grasp m45")),
    ("l: wing behind start", "r: wing behind start"),
    ("l: wing hold start", "r: wing hold start"),
    ("l: wing hold seated", "r: wing hold seated"),
    *((f"l: {step}", "r: wing hold seated") for step in ("cradle grasp p45", "cam flush high lock0", "cam flush seated lock0", "cam flush seated lock1", "cam flush seated lock2", "cam flush seated lock3", "cradle grasp m45")),
    *(("l: wing hold seated", f"r: {step}") for step in ("cradle grasp p45", "cam flush high lock0", "cam flush seated lock0", "cam flush seated lock1", "cam flush seated lock2", "cam flush seated lock3", "cradle grasp m45")),
    ("l: wing behind seated", "r: wing behind seated"),
)


def solve_pose(side: str, pos, rot, allow_flip: bool = False):
    best = None
    sgn = -1.0 if side == "left" else 1.0
    seeds = [
        np.array(READY, dtype=float),
        np.array([sgn * 0.4, 1.1, 1.2, -0.7, 0.0, 0.0], dtype=float),
        np.array([sgn * 0.4, 1.1, 1.2, -0.7, 0.0, 1.4], dtype=float),
        np.array([sgn * 0.4, 1.1, 1.2, -0.7, 0.0, -1.4], dtype=float),
        np.array([sgn * 0.3, 1.0, 1.2, 0.4, 0.0, 1.5], dtype=float),
        np.array([sgn * 0.3, 1.0, 1.2, 0.4, 0.0, -1.5], dtype=float),
    ]
    flips = (False, True) if allow_flip else (False,)
    for flip in flips:
        r = np.asarray(rot, dtype=float) @ (np.diag([-1.0, -1.0, 1.0]) if flip else np.eye(3))
        quat = ik.matrix_to_quat_xyzw(r)
        for q0 in seeds:
            q, pe, re = ik.solve(np.asarray(pos, dtype=float), quat, q0, base_pos=BASES[side], iters=100)
            margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
            ok = pe < IK_MM / 1000.0 and math.degrees(re) < IK_DEG
            key = (ok, margin if ok else -(pe * 1000.0 + math.degrees(re)))
            if best is None or key > best[0]:
                best = (key, q, float(pe * 1000.0), float(math.degrees(re)), margin)
            if ok and margin >= 14.0:
                break
    assert best is not None
    return best[1], best[2], best[3], best[4]


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


def fixture_boxes() -> dict:
    fx0, fx1 = P.SUPPORT_FRONT_X
    rx0, rx1 = P.SUPPORT_REAR_X
    c_rot = np.array([[math.cos(math.radians(P.CRADLE_YAW_DEG)), -math.sin(math.radians(P.CRADLE_YAW_DEG)), 0.0],
                      [math.sin(math.radians(P.CRADLE_YAW_DEG)), math.cos(math.radians(P.CRADLE_YAW_DEG)), 0.0],
                      [0.0, 0.0, 1.0]])
    return {
        "bench": [("bench", box_hull((0.25, 0.0, TOP - 0.020), np.eye(3), (0.80, 0.90, 0.040)))],
        "cabinet_side": [("side", box_hull((P.X_SIDE + P.PANEL_T / 2.0, 0.0, TOP + P.SIDE_H / 2.0), np.eye(3), (P.PANEL_T, P.SIDE_W, P.SIDE_H)))],
        "support": [
            ("front", box_hull(((fx0 + fx1) / 2.0, 0.0, TOP + P.SUPPORT_H / 2.0), np.eye(3), (fx1 - fx0, P.SUPPORT_FRONT_W, P.SUPPORT_H))),
            ("rear", box_hull(((rx0 + rx1) / 2.0, 0.0, TOP + P.SUPPORT_H / 2.0), np.eye(3), (rx1 - rx0, P.SUPPORT_REAR_W, P.SUPPORT_H))),
        ],
        "cradles": [
            (f"cradle_{c}", box_hull((P.CRADLE_X, (1.0 if c == "l" else -1.0) * P.CRADLE_Y, TOP + P.CRADLE_H / 2.0), c_rot, (P.CRADLE_SIDE, P.CRADLE_SIDE, P.CRADLE_H)))
            for c in ("l", "r")
        ],
    }


def run_offline() -> dict:
    found, rows, ok = {}, {}, True
    fixtures = fixture_boxes()
    for name, (side, pos, rot, opening, _sh_gap) in poses().items():
        q, mm, deg, margin = solve_pose(side, pos, rot, allow_flip=("wing" in name))
        found[name] = (side, q, opening)
        hulls = hand_hulls(side, q, opening)
        z_over = float(pos[2] - TOP)
        clear = {k: round(1000.0 * gap(hulls, v), 1) for k, v in fixtures.items()}
        good = (
            mm <= IK_MM
            and deg <= IK_DEG
            and margin >= MARGIN_DEG
            and z_over <= HEIGHT_MAX
            and opening <= MAX_JAW
            and min(clear.values()) > 0.0
        )
        ok &= good
        rows[name] = {
            "side": side,
            "z_over_mm": round(1000.0 * z_over, 1),
            "IK_mm_deg": [round(mm, 3), round(deg, 3)],
            "margin_deg": round(margin, 1),
            "clear_mm": clear,
            "ok": good,
        }
    pairs = {}
    for a, b in TOGETHER:
        (sa, qa, oa), (sb, qb, ob) = found[a], found[b]
        tcp = float(np.linalg.norm(poses()[a][1] - poses()[b][1]))
        hull_gap = gap(hand_hulls(sa, qa, oa), hand_hulls(sb, qb, ob))
        good = tcp >= SEPARATION and hull_gap >= 0.020
        ok &= good
        pairs[f"{a} + {b}"] = {
            "tcp_apart_mm": round(1000.0 * tcp, 1),
            "hulls_apart_mm": round(1000.0 * hull_gap, 1),
            "ok": good,
        }
    summary = {
        "pass": bool(ok),
        "pose_count": len(rows),
        "worst_ik_mm": max(r["IK_mm_deg"][0] for r in rows.values()),
        "worst_ik_deg": max(r["IK_mm_deg"][1] for r in rows.values()),
        "least_margin_deg": min(r["margin_deg"] for r in rows.values()),
        "highest_grasp_mm": max(r["z_over_mm"] for r in rows.values()),
        "least_fixture_clear_mm": min(min(r["clear_mm"].values()) for r in rows.values()),
        "least_tcp_sep_mm": min(p["tcp_apart_mm"] for p in pairs.values()),
        "least_hull_sep_mm": min(p["hulls_apart_mm"] for p in pairs.values()),
    }
    return {**summary, "poses": rows, "pairs": pairs}


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
        for name in ("shelf", "cam_l", "cam_r", "driver_l", "driver_r")
    }
    out = {
        "moved_from_built_mm": moved,
        "score_after_2s": float(res.info.get("score", 0.0)),
        "report": m.report(),
    }
    (out_dir / "g1_frame0.json").write_text(json.dumps(out, indent=1, default=str))
    return out


def main() -> int:
    if "--parts" in sys.argv:
        print(json.dumps(P.summary(), indent=1))
        return 0
    offline = run_offline()
    brief = {k: v for k, v in offline.items() if k not in ("poses", "pairs")}
    failed_poses = {k: v for k, v in offline["poses"].items() if not v["ok"]}
    failed_pairs = {k: v for k, v in offline["pairs"].items() if not v["ok"]}
    if failed_poses or failed_pairs:
        brief["failed_poses"] = failed_poses
        brief["failed_pairs"] = failed_pairs
    print("G1_OFFLINE " + json.dumps(brief, indent=1), flush=True)
    if "--frame0" in sys.argv:
        out = run_frame0(Path(sys.argv[sys.argv.index("--frame0") + 1]))
        print("G1_FRAME0 " + json.dumps(out, indent=1, default=str), flush=True)
        if out["score_after_2s"] != 0.0 or out["report"]["zero"] is not None or max(out["moved_from_built_mm"].values()) > 2.5:
            return 1
    return 0 if offline["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
