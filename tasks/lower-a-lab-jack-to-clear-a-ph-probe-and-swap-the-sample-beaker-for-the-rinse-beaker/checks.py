#!/usr/bin/env python3
"""Reach, joint-margin, hull-clearance, and first-frame checks for ``lab_jack_probe`` (Gate G1).

1. Offline (no simulator): solves every waypoint of the bimanual plan through ``yam_arena.ik`` (position error <= 0.5 mm,
   orientation error <= 0.5 deg, every joint >= 5.0 deg inside its limits), checks all arm-link collision hulls against
   the bench, fixed stand/probe, and scissor jack, and verifies >= 200 mm grasp-point separation and positive hull
   clearance wherever both arms act together.
2. ``--frame0 <dir>`` (in simulator): saves ``start_top.png``, ``start_top_policy.png`` (320 x 176),
   ``start_left_wrist.png``, and ``start_right_wrist.png``, then steps 2.0 s (40 steps) at READY verifying idle stability
   (< 1.5 mm settle), ``score == 0.0``, and ``zero is None``.

    scripts/rta simpy <id> mech/tasks/lab_jack_probe/checks.py --frame0 /workspace/robocurve/outputs/<id>
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
CHECK_LINKS = ("link3", "link4", "link5", "gripper", "tip_left", "tip_right")
HAND_LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")
MAX_JAW = 2.0 * abs(float(C.FINGER_OPEN))  # 0.0939 m
IK_MM, IK_DEG, MARGIN_DEG, SEPARATION_M = 0.5, 0.5, 5.0, 0.200

# --- Shared grasp constants and pose builders (used directly by demo.py) ---------------------------------------------
WHEEL_ARC_LO_DEG = -71.0
WHEEL_ARC_HI_DEG = -7.0  # 64.0 deg (~58 deg effective) per front-quadrant pinch stroke
WHEEL_R_GRIP = 0.0175  # m from spindle axis (leaves fingertip end 4.5 mm clear of 4.0 mm M8 shaft)
WHEEL_R_CLEAR = 0.036  # m radial clearance arc outside the 20.0 mm wheel radius
WHEEL_OPEN_M = 0.044  # m jaw opening around 18.0 mm wheel (13 mm air gap per face)
WHEEL_PINCH_M = 0.008  # m jaw target on 18.0 mm wheel (~25 N per pad)

BEAKER_GRASP_Z = 0.033  # m above beaker underside (95 mm over bench on 62 mm deck; <= 160 mm cap)
BEAKER_LIFT_CLEAR = 0.0035  # m clear above 62.0 mm deck during horizontal transfer (rim at 125.5 mm < 130.0 mm tip)
BEAKER_BACK_M = 0.095  # m horizontal approach/retreat distance outside deck/mat
BEAKER_OPEN_M = 0.068  # m jaw opening around 48.0 mm OD beaker (10 mm per side)
BEAKER_PINCH_M = 0.040  # m jaw target on 48.0 mm OD beaker (~20 N per pad)
Z_DECK_LOW = 0.062  # m lowered deck height (8.0 mm rim-to-tip air gap)
Z_DECK_TARGET = 0.098  # m final raised deck height (28.0 mm immersion inside 20-40 mm green band)


def frame(closing, approach, *, flip: bool = False) -> np.ndarray:
    """Gripper rotation matrix: columns (closing, pad-width, approach); ``flip`` turns 180 deg about approach."""
    a = np.asarray(approach, dtype=float) / np.linalg.norm(approach)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c /= np.linalg.norm(c)
    rot = np.column_stack([c, np.cross(a, c), a])
    return rot @ np.diag([-1.0, -1.0, 1.0]) if flip else rot


BRACE_ROT = frame((1.0, 0.0, 0.0), (0.0, -math.sqrt(0.5), -math.sqrt(0.5)), flip=True)


def brace_pose(*, hover: bool = False, pre_in: bool = False) -> tuple[np.ndarray, np.ndarray]:
    """Left closed fingertip steadying ``jack_base``'s +y top face via pure vertical touchdown (zero lateral shove)."""
    if hover:
        return np.array([0.245, 0.122, TOP + 0.052]), BRACE_ROT
    if pre_in:
        return np.array([0.245, 0.100, TOP + 0.032]), BRACE_ROT
    return np.array([0.245, 0.100, TOP + 0.0235]), BRACE_ROT


def wheel_pose(
    z_deck: float, ang_deg: float, *, clear: bool = False, wheel_centre: np.ndarray | None = None
) -> tuple[np.ndarray, np.ndarray]:
    """Right gripper radially pinching the -y wheel at ``ang_deg`` from +z toward +x (closing across y)."""
    ar = math.radians(ang_deg)
    r = WHEEL_R_CLEAR if clear else WHEEL_R_GRIP
    if wheel_centre is None:
        cx, cy, cz = P.JACK_XY[0], P.WHEEL_Y, TOP + 0.5 * z_deck
    else:
        cx, cy, cz = float(wheel_centre[0]), float(wheel_centre[1]), float(wheel_centre[2])
    pos = np.array([cx + r * math.sin(ar), cy, cz + r * math.cos(ar)])
    rot = frame((0.0, 1.0, 0.0), (-math.sin(ar), 0.0, -math.cos(ar)), flip=True)
    return pos, rot


def _beaker_rot(side: str) -> tuple[np.ndarray, np.ndarray]:
    yaw = math.radians(-22.0 if side == "left" else 22.0)
    pitch = math.radians(36.0)
    app = np.array([math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), -math.sin(pitch)])
    horiz_in = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    close = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    return frame(close, app, flip=True), horiz_in


LEFT_BEAKER_ROT, LEFT_IN_DIR = _beaker_rot("left")
RIGHT_BEAKER_ROT, RIGHT_IN_DIR = _beaker_rot("right")


def beaker_pose(
    side: str, xy: tuple[float, float], z_bottom: float, *, back: float = 0.0, along_approach: bool = False
) -> tuple[np.ndarray, np.ndarray]:
    """Constant-orientation side grasp on a 50 mL beaker with underside at ``z_bottom`` above bench."""
    rot, in_dir = (LEFT_BEAKER_ROT, LEFT_IN_DIR) if side == "left" else (RIGHT_BEAKER_ROT, RIGHT_IN_DIR)
    retreat = rot[:, 2] if along_approach else in_dir
    pos = np.array([xy[0], xy[1], TOP + z_bottom + BEAKER_GRASP_Z]) - back * retreat
    return pos, rot


def poses() -> dict[str, tuple[str, np.ndarray, np.ndarray, float, float]]:
    """Named waypoints: ``name -> (side, pos, rot, jaw_m, z_deck_context)``."""
    out: dict[str, tuple[str, np.ndarray, np.ndarray, float, float]] = {}
    out["brace: hover"] = ("left", *brace_pose(hover=True), 0.0, P.Z_DECK_START)
    out["brace: pre_in"] = ("left", *brace_pose(pre_in=True), 0.0, P.Z_DECK_START)
    out["brace: press"] = ("left", *brace_pose(), 0.0, P.Z_DECK_START)
    out["brace: press lowered"] = ("left", *brace_pose(), 0.0, Z_DECK_LOW)

    for tag, zd in (("start", P.Z_DECK_START), ("low", Z_DECK_LOW), ("target", Z_DECK_TARGET)):
        for ang_name, ang in (("hi", WHEEL_ARC_HI_DEG), ("mid", -39.0), ("lo", WHEEL_ARC_LO_DEG)):
            out[f"wheel {tag} pinch {ang_name}"] = ("right", *wheel_pose(zd, ang, clear=False), WHEEL_PINCH_M, zd)
            out[f"wheel {tag} clear {ang_name}"] = ("right", *wheel_pose(zd, ang, clear=True), WHEEL_OPEN_M, zd)

    # Left arm: beaker_sample from lowered deck -> blue_mat
    out["sample: outside deck"] = ("left", *beaker_pose("left", P.JACK_XY, Z_DECK_LOW, back=BEAKER_BACK_M), BEAKER_OPEN_M, Z_DECK_LOW)
    out["sample: around on deck"] = ("left", *beaker_pose("left", P.JACK_XY, Z_DECK_LOW), BEAKER_OPEN_M, Z_DECK_LOW)
    out["sample: lifted on deck"] = ("left", *beaker_pose("left", P.JACK_XY, Z_DECK_LOW + BEAKER_LIFT_CLEAR), BEAKER_PINCH_M, Z_DECK_LOW)
    out["sample: extracted"] = ("left", *beaker_pose("left", P.JACK_XY, Z_DECK_LOW + BEAKER_LIFT_CLEAR, back=BEAKER_BACK_M), BEAKER_PINCH_M, Z_DECK_LOW)
    out["sample: above mat"] = ("left", *beaker_pose("left", P.MAT_XY, P.MAT_SIZE[2] + 0.035), BEAKER_PINCH_M, Z_DECK_LOW)
    out["sample: on mat"] = ("left", *beaker_pose("left", P.MAT_XY, P.MAT_SIZE[2]), BEAKER_PINCH_M, Z_DECK_LOW)
    out["sample: backed off mat"] = ("left", *beaker_pose("left", P.MAT_XY, P.MAT_SIZE[2], back=0.065, along_approach=True), BEAKER_OPEN_M, Z_DECK_LOW)

    # Right arm: beaker_rinse from rinse_coaster -> lowered deck
    out["rinse: outside coaster"] = ("right", *beaker_pose("right", P.RINSE_START_XY, P.COASTER_SIZE[2], back=0.065, along_approach=True), BEAKER_OPEN_M, Z_DECK_LOW)
    out["rinse: on coaster"] = ("right", *beaker_pose("right", P.RINSE_START_XY, P.COASTER_SIZE[2]), BEAKER_PINCH_M, Z_DECK_LOW)
    out["rinse: lifted"] = ("right", *beaker_pose("right", P.RINSE_START_XY, Z_DECK_LOW + BEAKER_LIFT_CLEAR), BEAKER_PINCH_M, Z_DECK_LOW)
    out["rinse: outside deck"] = ("right", *beaker_pose("right", P.JACK_XY, Z_DECK_LOW + BEAKER_LIFT_CLEAR, back=BEAKER_BACK_M), BEAKER_PINCH_M, Z_DECK_LOW)
    out["rinse: over deck"] = ("right", *beaker_pose("right", P.JACK_XY, Z_DECK_LOW + BEAKER_LIFT_CLEAR), BEAKER_PINCH_M, Z_DECK_LOW)
    out["rinse: set on deck"] = ("right", *beaker_pose("right", P.JACK_XY, Z_DECK_LOW), BEAKER_OPEN_M, Z_DECK_LOW)
    out["rinse: backed off deck"] = ("right", *beaker_pose("right", P.JACK_XY, Z_DECK_LOW, back=BEAKER_BACK_M), BEAKER_OPEN_M, Z_DECK_LOW)
    return out


TOGETHER = tuple(
    (b_pose, f"wheel {tag} {mode} {ang}")
    for b_pose, tag in (("brace: press", "start"), ("brace: press lowered", "low"), ("brace: press", "target"))
    for mode in ("pinch", "clear")
    for ang in ("hi", "mid", "lo")
)


def solve(side: str, pos: np.ndarray, rot: np.ndarray, start: np.ndarray | None = None):
    seeds = ([np.asarray(start, dtype=float)] if start is not None else []) + [
        np.array(s, dtype=float)
        for s in (
            READY,
            [0.0, 1.2, 1.2, -0.5, 0.0, 0.0],
            [0.0, 1.5, 1.5, -0.4, 0.0, 0.0],
            [0.6, 1.6, 1.2, -1.0, 0.5, 0.0],
            [-0.6, 1.6, 1.2, -1.0, 0.0, -0.6],
            [0.9, 1.4, 0.6, -0.2, 0.7, 0.8],
            [-0.9, 1.4, 0.6, -0.2, -0.7, -0.8],
            [0.3, 1.0, 1.4, 0.4, 1.5, 0.0],
            [-0.3, 1.0, 1.4, 0.4, -1.5, 0.0],
        )
    ]
    quat = ik.matrix_to_quat_xyzw(np.asarray(rot, dtype=float))
    best = None
    for q0 in seeds:
        q, pe, re = ik.solve(np.asarray(pos, dtype=float), quat, q0, base_pos=BASES[side], iters=200)
        margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
        ok = pe < IK_MM / 1000.0 and math.degrees(re) < IK_DEG
        key = (ok, margin if ok else -(pe * 1000.0 + math.degrees(re)))
        if best is None or key > best[0]:
            best = (key, q, float(pe * 1000.0), float(math.degrees(re)), margin)
    assert best is not None
    return best[1], best[2], best[3], best[4]


def link_hulls_world(side: str, q: np.ndarray, opening: float, links=CHECK_LINKS) -> list:
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


def scene_obstacles(z_deck: float) -> dict[str, list]:
    jx, jy = P.JACK_XY
    sx, sy = P.STAND_POST_XY
    probe_len = P.PROBE_OVERALL_L
    probe_box = box_hull((jx, jy, TOP + P.Z_PROBE_TIP + probe_len / 2.0), np.eye(3), (P.PROBE_CAP_D, P.PROBE_CAP_D, probe_len))
    stand_post = box_hull((sx, sy, TOP + 0.155), np.eye(3), (0.024, 0.024, 0.310))
    stand_arm = box_hull((0.5 * (sx + jx), jy, TOP + P.Z_CLAMP_ANCHOR), np.eye(3), (sx - jx, 0.016, 0.016))
    deck_box = box_hull((jx, jy, TOP + z_deck - P.PLATE_T / 2.0), np.eye(3), (P.JACK_W, P.JACK_L, P.PLATE_T))
    base_box = box_hull((jx, jy, TOP + P.BASE_TOP_Z / 2.0), np.eye(3), (P.JACK_W, P.JACK_L, P.BASE_TOP_Z))
    return {"probe_and_stand": [("probe", probe_box), ("post", stand_post), ("arm", stand_arm)], "jack_deck": [("deck", deck_box)], "jack_base": [("base", base_box)]}


def run_offline() -> dict:
    all_poses = poses()
    found, rows, ok = {}, {}, True
    prev_q: dict[str, np.ndarray | None] = {"left": None, "right": None}
    for name, (side, pos, rot, jaw, zd) in all_poses.items():
        q, mm, deg, margin = solve(side, pos, rot, prev_q[side])
        prev_q[side] = q
        found[name] = (side, q, jaw, pos)
        hulls = link_hulls_world(side, q, jaw, CHECK_LINKS)
        bench_mm = min(float((h.vertices[:, 2] - TOP).min()) for _, h in hulls) * 1000.0
        obs = scene_obstacles(zd)
        clear = {
            "bench": round(bench_mm, 1),
            "probe_and_stand": round(1000.0 * gap(hulls, obs["probe_and_stand"]), 1),
            "jack_deck": round(1000.0 * gap(hulls, obs["jack_deck"]), 1),
        }
        if name.startswith("brace"):
            placed_sample = [("sample_on_mat", box_hull((P.MAT_XY[0], P.MAT_XY[1], TOP + P.MAT_SIZE[2] + P.BEAKER_H / 2.0), np.eye(3), (P.BEAKER_OD, P.BEAKER_OD, P.BEAKER_H)))]
            clear["sample_on_mat"] = round(1000.0 * gap(hulls, placed_sample), 1)
        else:
            clear["jack_base"] = round(1000.0 * gap(hulls, obs["jack_base"]), 1)
        grasp_z_mm = round(1000.0 * float(pos[2] - TOP), 1)
        good = mm <= IK_MM and deg <= IK_DEG and margin >= MARGIN_DEG and min(clear.values()) > 0.0 and grasp_z_mm <= 160.0
        ok &= good
        rows[name] = {
            "side": side,
            "grasp_z_mm": grasp_z_mm,
            "IK_mm_deg": [round(mm, 3), round(deg, 3)],
            "margin_deg": round(margin, 1),
            "clear_mm": clear,
            "ok": good,
        }
    pairs = {}
    for a, b in TOGETHER:
        sa, qa, oa, pa = found[a]
        sb, qb, ob, pb = found[b]
        tcp_m = float(np.linalg.norm(pa - pb))
        hull_mm = 1000.0 * gap(link_hulls_world(sa, qa, oa, HAND_LINKS), link_hulls_world(sb, qb, ob, HAND_LINKS))
        good = tcp_m >= SEPARATION_M and hull_mm > 20.0
        ok &= good
        pairs[f"{a} + {b}"] = {"tcp_apart_mm": round(1000.0 * tcp_m, 1), "hulls_apart_mm": round(hull_mm, 1), "ok": good}
    worst_ik_mm = max(r["IK_mm_deg"][0] for r in rows.values())
    min_margin = min(r["margin_deg"] for r in rows.values())
    min_pair_tcp = min(p["tcp_apart_mm"] for p in pairs.values())
    min_pair_hull = min(p["hulls_apart_mm"] for p in pairs.values())
    return {
        "pass": bool(ok),
        "summary": {
            "waypoints": len(rows),
            "worst_ik_mm": worst_ik_mm,
            "min_joint_margin_deg": min_margin,
            "min_two_hand_tcp_mm": min_pair_tcp,
            "min_two_hand_hull_mm": min_pair_hull,
        },
        "poses": rows,
        "both_hands": pairs,
    }


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
    first = None
    res = None
    for _ in range(40):
        res = emb.step(action)
        if first is None:
            first = {"deck_mm": round(1000.0 * m.deck_height_m(), 2), "probe_tip_z_mm": round(1000.0 * float(m.probe_tip()[2] - TOP), 2)}
    moved = {
        name: round(1000.0 * float(np.linalg.norm(m.mk.pose(name)[0] - np.asarray(m.mk.built_pose(name)[0]))), 2)
        for name in ("jack_base", "jack_deck", "knob", "beaker_sample", "beaker_rinse", "probe")
    }
    out = {
        "first_step": first,
        "after_2s_deck_mm": round(1000.0 * m.deck_height_m(), 2),
        "after_2s_probe_tip_z_mm": round(1000.0 * float(m.probe_tip()[2] - TOP), 2),
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
    print("G1_OFFLINE " + json.dumps(offline["summary"], indent=1), flush=True)
    if not offline["pass"]:
        bad = {k: v for k, v in offline["poses"].items() if not v["ok"]}
        bad_p = {k: v for k, v in offline["both_hands"].items() if not v["ok"]}
        print("G1_FAIL " + json.dumps({"poses": bad, "pairs": bad_p}, indent=1), flush=True)
    if "--frame0" in sys.argv:
        out = run_frame0(Path(sys.argv[sys.argv.index("--frame0") + 1]))
        print("G1_FRAME0 " + json.dumps(out, indent=1, default=str), flush=True)
        if out["score_after_2s"] != 0.0 or out["report"]["zero"] is not None:
            return 1
    return 0 if offline["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
