#!/usr/bin/env python3
"""Reach, joint-limit margins, hull clearances, and G1 start check for the end-cap suspension packing task.

1. Offline (no simulator): every waypoint of both arms solved by inverse kinematics (within 0.5 mm and 0.5 deg, every
   joint >= 6.0 deg inside its limits), both hands >= 200 mm apart and hull-disjoint throughout simultaneous bimanual
   work, and hand hulls clear of the carton walls, bench cradle, and roller at every grasp/release pose.
2. ``--frame0 <dir>`` in the simulator: renders the three cameras (640x360 and 320x176 policy size) and holds Ready
   for 2.0 s at rest: all parts settle under 1.0 mm, idle score is 0.0, no zero rule triggered.

    scripts/rta simpy <id> mech/tasks/endcap_pack/checks.py [--frame0 /workspace/robocurve/outputs/<id>]
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
MAX_JAW = 2.0 * abs(float(C.FINGER_OPEN))  # 0.0939 m
IK_MM, IK_DEG, MARGIN_DEG, SEPARATION = 0.5, 0.5, 6.0, 0.200


def frame(closing, approach) -> np.ndarray:
    """Gripper rotation: columns closing axis, pads' width axis, approach axis."""
    a = np.asarray(approach, dtype=float) / np.linalg.norm(approach)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


# Gripper pointing straight down, closing along world x across the 72 mm foam cap width (mirror-symmetric wrist roll!)
GRIP_ROT_R = frame((1.0, 0.0, 0.0), (0.0, 0.0, -1.0))
GRIP_ROT_L = frame((-1.0, 0.0, 0.0), (0.0, 0.0, -1.0))


def grip_rot(side: str) -> np.ndarray:
    return GRIP_ROT_L if side == "left" else GRIP_ROT_R

OPEN_JAW = 0.086  # m: 86 mm open (7 mm clear per side around the 72 mm cap, 9 mm clear of +/-76.2 mm carton walls)
PINCH_JAW = 0.062  # m: sent 5 mm/side inside the 72 mm cap -> ~25 N/finger squeeze (under 40 N limit)
GRIP_ABOVE_BASE = 0.0500  # m: TCP 50 mm above cap underside (24 mm deep grip below 74 mm top; tips 17.2 mm above 20 mm base)

Y_GRIP_COUPLED = 0.1015  # m: +/-101.5 mm (203.0 mm TCP separation; >=5.4 mm clear of +/-127 mm carton walls)
Y_GRIP_STAGE = 0.1400  # m: +/-140.0 mm on bench outside the +/-113 mm spindle tips before sliding inward
Z_HOVER_BOX = TOP + 0.1480  # m: 148.0 mm over bench (<= 160 mm rule; cap bottom at 98.0 mm, 17.8 mm over 80.2 mm carton rim)
Z_SEAT_BOX = TOP + P.Z_FLOOR + GRIP_ABOVE_BASE  # TOP + 0.0540 m
Z_STAGE_BENCH = TOP + GRIP_ABOVE_BASE + 0.0010  # TOP + 0.0510 m


def poses() -> dict[str, tuple[str, np.ndarray, np.ndarray, float]]:
    """Every simultaneous pose of the bimanual plan: name -> (side, pos, rot, jaw_m)."""
    out = {}
    for side, s, tag in (("left", +1.0, "l"), ("right", -1.0, "r")):
        rot = grip_rot(side)
        out[f"{tag}: hover over nested cap"] = (side, np.array([P.X_CARTON, s * Y_GRIP_COUPLED, Z_HOVER_BOX]), rot, OPEN_JAW)
        out[f"{tag}: around nested cap"] = (side, np.array([P.X_CARTON, s * Y_GRIP_COUPLED, Z_SEAT_BOX]), rot, OPEN_JAW)
        out[f"{tag}: pinched in carton"] = (side, np.array([P.X_CARTON, s * Y_GRIP_COUPLED, Z_SEAT_BOX]), rot, P.CAP_TOP_W)
        out[f"{tag}: lifted clear of carton"] = (side, np.array([P.X_CARTON, s * Y_GRIP_COUPLED, Z_HOVER_BOX]), rot, P.CAP_TOP_W)
        out[f"{tag}: over bench outside roller"] = (side, np.array([P.X_ROLLER, s * Y_GRIP_STAGE, Z_HOVER_BOX]), rot, P.CAP_TOP_W)
        out[f"{tag}: on bench outside roller"] = (side, np.array([P.X_ROLLER, s * Y_GRIP_STAGE, Z_STAGE_BENCH]), rot, P.CAP_TOP_W)
        out[f"{tag}: slid onto roller end"] = (side, np.array([P.X_ROLLER, s * Y_GRIP_COUPLED, Z_STAGE_BENCH]), rot, P.CAP_TOP_W)
        out[f"{tag}: coupled lift over rim"] = (side, np.array([P.X_ROLLER, s * Y_GRIP_COUPLED, Z_HOVER_BOX]), rot, P.CAP_TOP_W)
        out[f"{tag}: coupled over carton"] = (side, np.array([P.X_CARTON, s * Y_GRIP_COUPLED, Z_HOVER_BOX]), rot, P.CAP_TOP_W)
        out[f"{tag}: lowered into carton"] = (side, np.array([P.X_CARTON, s * Y_GRIP_COUPLED, Z_SEAT_BOX]), rot, P.CAP_TOP_W)
        out[f"{tag}: released in carton"] = (side, np.array([P.X_CARTON, s * Y_GRIP_COUPLED, Z_SEAT_BOX]), rot, OPEN_JAW)
    return out


STEPS = (
    "hover over nested cap",
    "around nested cap",
    "pinched in carton",
    "lifted clear of carton",
    "over bench outside roller",
    "on bench outside roller",
    "slid onto roller end",
    "coupled lift over rim",
    "coupled over carton",
    "lowered into carton",
    "released in carton",
)
TOGETHER = tuple((f"l: {st}", f"r: {st}") for st in STEPS)


def solve(side: str, pos, rot, start=None):
    best = None
    seeds = ([np.asarray(start, dtype=float)] if start is not None else []) + [
        np.array(s, dtype=float)
        for s in (
            READY,
            [0.0, 1.2, 1.2, -0.5, 0.0, 0.0],
            [0.0, 1.5, 1.5, -0.4, 0.0, 0.0],
            [0.4, 1.3, 1.3, -0.7, 0.4, 0.0],
            [-0.4, 1.3, 1.3, -0.7, -0.4, 0.0],
        )
    ]
    for flip in (False, True):
        r = np.asarray(rot, dtype=float) @ (np.diag([-1.0, -1.0, 1.0]) if flip else np.eye(3))
        quat = ik.matrix_to_quat_xyzw(r)
        for q0 in seeds:
            q, pe, re = ik.solve(np.asarray(pos, dtype=float), quat, q0, base_pos=BASES[side], iters=300)
            margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
            ok = bool(pe < IK_MM / 1000 and math.degrees(re) < IK_DEG)
            key = (ok, margin if ok else -(pe * 1000.0 + math.degrees(re)))
            if best is None or key > best[0]:
                best = (key, q, float(pe * 1000.0), float(math.degrees(re)), margin, bool(flip))
    assert best is not None
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
    bounds_a = [(a, a.vertices.min(axis=0), a.vertices.max(axis=0)) for _, a in hulls_a]
    bounds_b = [(b, b.vertices.min(axis=0), b.vertices.max(axis=0)) for _, b in hulls_b]
    for a, amin, amax in bounds_a:
        for b, bmin, bmax in bounds_b:
            aabb_lb = float(np.linalg.norm(np.maximum(np.maximum(amin - bmax, bmin - amax), 0.0)))
            if aabb_lb >= best:
                continue
            best = min(best, float(analysis._signed(a.vertices, b).min()), float(analysis._signed(b.vertices, a).min()))
    return best


def fixture_hulls() -> dict[str, list]:
    """Solid world hulls of the fixed carton (floor + 4 walls) and the fixed bench cradle, plus the cap's lower wings."""
    w, l, h, t = P.BOX_W, P.BOX_L, P.BOX_H, P.BOX_WALL
    ox, oy = w + 2 * t, l + 2 * t
    carton = [
        ("floor", box_hull((P.X_CARTON, 0.0, TOP + t / 2), np.eye(3), (ox, oy, t))),
        ("front_wall", box_hull((P.X_CARTON - w / 2 - t / 2, 0.0, TOP + t + h / 2), np.eye(3), (t, oy, h))),
        ("back_wall", box_hull((P.X_CARTON + w / 2 + t / 2, 0.0, TOP + t + h / 2), np.eye(3), (t, oy, h))),
        ("right_wall", box_hull((P.X_CARTON, -l / 2 - t / 2, TOP + t + h / 2), np.eye(3), (w, t, h))),
        ("left_wall", box_hull((P.X_CARTON, +l / 2 + t / 2, TOP + t + h / 2), np.eye(3), (w, t, h))),
    ]
    cradle = [("cradle", box_hull((P.X_ROLLER, 0.0, TOP + P.CRADLE_RAIL_TOP / 2), np.eye(3), (P.CRADLE_W, P.CRADLE_L, P.CRADLE_RAIL_TOP)))]
    return {"carton": carton, "cradle": cradle}


def run_offline() -> dict:
    fixtures = fixture_hulls()
    found, rows, ok = {}, {}, True
    for name, (side, pos, rot, opening) in poses().items():
        q, mm, deg, margin, flip = solve(side, pos, rot)
        found[name] = (side, q, opening)
        hulls = hand_hulls(side, q, opening)
        # Also verify the fingers never touch the cap's 144 mm lower base flange (z_cap = 0..20 mm)
        cap_z0 = pos[2] - GRIP_ABOVE_BASE
        s = +1.0 if side == "left" else -1.0
        cap_y = pos[1] + s * (P.COUPLED_CAP_Y - Y_GRIP_COUPLED)
        base_flange = [("cap_base", box_hull((pos[0], cap_y, cap_z0 + P.CAP_BASE_H / 2), np.eye(3), (P.CAP_BASE_W, P.CAP_T, P.CAP_BASE_H)))]
        clear = {
            "carton": round(1000 * gap(hulls, fixtures["carton"]), 1),
            "cradle": round(1000 * gap(hulls, fixtures["cradle"]), 1),
            "cap_base_flange": round(1000 * gap(hulls, base_flange), 1),
            "height_under_160mm": round(1000 * (0.160 - (pos[2] - TOP)), 1),
        }
        good = bool(mm <= IK_MM and deg <= IK_DEG and margin >= MARGIN_DEG and min(clear.values()) > 0.0)
        ok = bool(ok and good)
        rows[name] = {
            "side": side,
            "at_m": [round(float(v), 4) for v in pos],
            "ik_mm_deg": [round(mm, 3), round(deg, 3)],
            "flipped": bool(flip),
            "least_margin_deg": round(margin, 1),
            "clear_mm": clear,
            "ok": good,
        }
    pairs = {}
    for a, b in TOGETHER:
        (sa, qa, oa), (sb, qb, ob) = found[a], found[b]
        tcp = float(np.linalg.norm(poses()[a][1] - poses()[b][1]))
        hull_gap = gap(hand_hulls(sa, qa, oa), hand_hulls(sb, qb, ob))
        good = bool(tcp >= SEPARATION and hull_gap >= 0.020)
        ok = bool(ok and good)
        pairs[f"{a} + {b}"] = {"tcp_apart_mm": round(1000 * tcp, 1), "hulls_apart_mm": round(1000 * hull_gap, 1), "ok": good}
    return {"pass": bool(ok), "poses": rows, "both_hands_at_once": pairs}


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
    for _ in range(40):
        res = emb.step(action)
    moved = {
        name: round(1000 * float(np.linalg.norm(m.mk.pose(name)[0] - np.asarray(m.mk.built_pose(name)[0]))), 2)
        for name in ("roller", "cap_l", "cap_r")
    }
    out = {
        "moved_from_built_mm": moved,
        "score_after_2s": float(res.info.get("score", 0.0)),
        "report": m.report(),
        "idle_pass": float(res.info.get("score", 0.0)) == 0.0 and m.STATE.get("zero") is None and max(moved.values()) <= 2.0,
    }
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
        return 0 if (offline["pass"] and out["idle_pass"]) else 1
    return 0 if offline["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
