#!/usr/bin/env python3
"""Reach, joint-margin, collision-clearance, and frame-0 checks for ``burner_stow``.

1. Offline (no simulator): verifies every canonical hand pose used by ``demo.py``:
   - IK position error <= 0.5 mm and rotation error <= 0.5 deg;
   - Every arm joint >= 5.0 deg inside its limits;
   - Every grasp/contact point <= 160 mm above the bench and jaw opening <= 94 mm;
   - Both hands >= 20 mm apart (hull-to-hull) wherever both work together;
   - Hand collision hulls clear of the bench and tin walls.
2. ``--frame0 <dir>`` in the simulator: writes ``start_top.png``, ``start_left_wrist.png``, ``start_right_wrist.png``
   plus their ``320 x 176`` policy-size counterparts, steps 2.0 s idle at ``READY``, and checks stability and score 0.
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
IK_MM, IK_DEG, MARGIN_DEG, HULL_SEP_MM, MAX_GRASP_Z = 0.5, 0.5, 5.0, 20.0, 0.160


def frame(closing, approach) -> np.ndarray:
    """Gripper rotation matrix: columns are closing axis, pad-width axis, approach axis."""
    a = np.asarray(approach, dtype=float)
    a = a / np.linalg.norm(a)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c = c / np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def _rz(rad: float) -> np.ndarray:
    return np.array([[math.cos(rad), -math.sin(rad), 0.0], [math.sin(rad), math.cos(rad), 0.0], [0.0, 0.0, 1.0]])


# --- Canonical task frames and poses (shared with demo.py) ----------------------------------------------------------------
ZB0 = np.array([0.0, 0.0, 1.0])
RB_START = _rz(P.BURN_YAW)
STAND_ORIGIN = np.array([P.STAND_XY[0], P.STAND_XY[1], TOP + P.OVER + P.STAND_FLOOR_H])

# Bimanual shutoff pose at WORK_XY = (0.300, +0.140), WORK_YAW = -22 deg.
RB_WORK = _rz(P.WORK_YAW)
XBW, YBW = RB_WORK[:, 0], RB_WORK[:, 1]
WORK_ORIGIN = np.array([P.WORK_XY[0], P.WORK_XY[1], TOP + P.OVER + P.STAND_FLOOR_H])

# Burner flat orientation in tin: z_b -> (-1, 0, 0), x_b -> (0, -1, 0), y_b -> (0, 0, +1) (proper rotation, det = +1).
RB_FLAT = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
WORK_TO_FLAT = RB_FLAT @ RB_WORK.T
START_FROM_WORK = RB_START @ RB_WORK.T

# 1. Left hand on the 38 mm cylindrical tank (z_b = 75 mm -> z_world = TOP + 88 mm), carried from STAND_XY to WORK_XY.
TANK_Z_B = 0.075
R_TANK_WORK = frame(XBW, -0.78 * YBW - 0.626 * ZB0)
P_TANK_WORK = WORK_ORIGIN + TANK_Z_B * ZB0
R_TANK_START = START_FROM_WORK @ R_TANK_WORK
P_TANK_START = STAND_ORIGIN + TANK_Z_B * ZB0
R_TANK_FLAT = WORK_TO_FLAT @ R_TANK_WORK

# 2. Right hand closed fingertip riding along the smooth -13 mm slider rail through lock_slider at WORK_ORIGIN.
R_LOCK = frame(ZB0, 0.92 * YBW - 0.39 * ZB0)
P_LOCK_START = WORK_ORIGIN - 0.028 * XBW - 0.0265 * YBW + 0.1245 * ZB0
P_LOCK_END = P_LOCK_START + 0.032 * XBW + 0.005 * YBW

# 3. Mid-air horizontal pull-off: burner heel (+stand) at HEEL_PULL_AT pointing along +x (z_b = -x).
HEEL_PULL_AT = np.array([0.362, 0.045, TOP + 0.126])
P_TANK_PULL = HEEL_PULL_AT - TANK_Z_B * np.array([1.0, 0.0, 0.0])  # (0.287, 0.045, TOP + 0.126)

# Stand rotation from pull pose (socket opening -x, bottom +x) to tin placement (68 - 50 = 18 deg -> settles flat on 60 mm base).
STAND_TILT = math.radians(68.0)
R_STAND_PULL = frame((0.0, 0.0, 1.0), (math.cos(STAND_TILT), math.sin(STAND_TILT), 0.0))
R_STAND_TIN = frame((1.0, 0.0, 0.0), (0.0, math.sin(math.radians(50.0)), -math.cos(math.radians(50.0))))
P_STAND_GRAB = HEEL_PULL_AT + np.array([0.006, 0.0, 0.0])
P_STAND_PULLED = HEEL_PULL_AT + np.array([0.030, 0.0, 0.0])  # 24 mm axial pull > 12 mm socket engagement

# 4. Placement poses inside the 320 x 172 x 52 mm tin (centered at (0.300, -0.115)).
P_STAND_TIN = np.array([0.200, -0.102, TOP + 0.025])
BURNER_HEEL_TIN = np.array([0.426, -0.106, TOP + 0.038])  # gentle release 3 mm above floor keeps pad force <= 40 N
P_BURNER_TIN = BURNER_HEEL_TIN - TANK_Z_B * np.array([1.0, 0.0, 0.0])  # (0.351, -0.106, TOP + 0.038)

# 5. Lid closing arc from 108 deg open over 90 deg to 73 deg (gravity settles lid to 0 deg).
HINGE_AT = np.array([P.TIN_XY[0], P.TIN_XY[1] - P.TIN_W / 2.0 + P.TIN_WALL / 2.0, TOP + P.OVER + P.TIN_BASE_H])


def lid_push_pose(lid_angle_deg: float, back_mm: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Closed right hand on the outer back face of ``tin_lid`` at radial arm r = 84 mm from ``HINGE_AT``."""
    ang = math.radians(lid_angle_deg)
    r_arm = 0.084
    tangent = np.array([0.0, math.cos(ang), math.sin(ang)])
    outer_n = np.array([0.0, -math.sin(ang), math.cos(ang)])
    surface = HINGE_AT + r_arm * tangent + P.TIN_LID_H * outer_n
    push_dir = np.array([0.0, 0.80, -0.60]) if lid_angle_deg >= 75.0 else np.array([0.0, 0.707, -0.707])
    push_dir /= np.linalg.norm(push_dir)
    rot = frame((-1.0, 0.0, 0.0), push_dir)
    tcp = surface - (0.012 + back_mm / 1000.0) * rot[:, 2]
    return tcp, rot


def poses() -> dict[str, tuple[str, np.ndarray, np.ndarray, float, bool]]:
    """name -> (side, tcp_pos, rot, jaw_opening_m, is_grasp_or_contact)."""
    p_lid_pre, r_lid_pre = lid_push_pose(P.LID_OPEN_DEG, back_mm=20.0)
    p_lid_108, r_lid_108 = lid_push_pose(P.LID_OPEN_DEG, back_mm=-2.0)
    p_lid_88, r_lid_88 = lid_push_pose(88.0, back_mm=-2.0)
    p_lid_73, r_lid_73 = lid_push_pose(73.0, back_mm=-2.0)
    p_lid_retract, r_lid_retract = lid_push_pose(80.0, back_mm=28.0)
    return {
        "L: start tank pre": ("left", P_TANK_START - 0.035 * R_TANK_START[:, 2], R_TANK_START, MAX_JAW, False),
        "L: start tank grasp": ("left", P_TANK_START, R_TANK_START, P.TANK_D, True),
        "L: carry to work": ("left", P_TANK_WORK + np.array([0.0, 0.0, 0.030]), R_TANK_WORK, P.TANK_D, False),
        "L: work tank brace": ("left", P_TANK_WORK, R_TANK_WORK, P.TANK_D, True),
        "R: lock pre": ("right", P_LOCK_START - 0.035 * R_LOCK[:, 2], R_LOCK, 0.0, False),
        "R: lock engage": ("right", P_LOCK_START, R_LOCK, 0.0, True),
        "R: lock pushed": ("right", P_LOCK_END, R_LOCK, 0.0, True),
        "L: carry pull-off": ("left", P_TANK_PULL, R_TANK_FLAT, P.TANK_D, True),
        "R: stand pre": ("right", P_STAND_GRAB - 0.045 * R_STAND_PULL[:, 2], R_STAND_PULL, MAX_JAW, False),
        "R: stand grasp": ("right", P_STAND_GRAB, R_STAND_PULL, P.STAND_D, True),
        "R: stand pulled": ("right", P_STAND_PULLED, R_STAND_PULL, P.STAND_D, True),
        "R: stand above tin": ("right", P_STAND_TIN + np.array([0.0, 0.0, 0.085]), R_STAND_TIN, P.STAND_D, False),
        "R: stand in tin": ("right", P_STAND_TIN, R_STAND_TIN, P.STAND_D, True),
        "R: stand clear high": ("right", np.array([0.200, -0.100, TOP + 0.245]), R_STAND_TIN, MAX_JAW, False),
        "L: burner above tin": ("left", P_BURNER_TIN + np.array([0.0, 0.0, 0.095]), R_TANK_FLAT, P.TANK_D, False),
        "L: burner in tin": ("left", P_BURNER_TIN, R_TANK_FLAT, P.TANK_D, True),
        "L: burner release": ("left", P_BURNER_TIN + np.array([0.0, 0.0, 0.015]), R_TANK_FLAT, MAX_JAW, False),
        "R: lid pre": ("right", p_lid_pre, r_lid_pre, 0.0, False),
        "R: lid 108 deg": ("right", p_lid_108, r_lid_108, 0.0, True),
        "R: lid 88 deg": ("right", p_lid_88, r_lid_88, 0.0, True),
        "R: lid 73 deg": ("right", p_lid_73, r_lid_73, 0.0, True),
        "R: lid retract": ("right", p_lid_retract, r_lid_retract, 0.0, False),
    }


TOGETHER = (
    ("L: work tank brace", "R: lock pre"),
    ("L: work tank brace", "R: lock engage"),
    ("L: work tank brace", "R: lock pushed"),
    ("L: carry pull-off", "R: stand pre"),
    ("L: carry pull-off", "R: stand grasp"),
    ("L: carry pull-off", "R: stand pulled"),
)


def solve(side: str, pos, rot, start=None):
    quat = ik.matrix_to_quat_xyzw(np.asarray(rot, dtype=float))
    q0_first = np.asarray(READY if start is None else start, dtype=float)
    q, pe, re = ik.solve(np.asarray(pos, dtype=float), quat, q0_first, base_pos=BASES[side], iters=100)
    margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
    if pe < IK_MM / 1000.0 and math.degrees(re) < IK_DEG and margin >= MARGIN_DEG:
        return q, float(pe * 1000.0), float(math.degrees(re)), margin
    seeds = [
        np.array(s, dtype=float)
        for s in (
            READY,
            [0.0, 1.2, 1.2, -0.5, 0.0, 0.0],
            [-0.6, 1.4, 1.2, -0.8, -0.5, 0.0],
            [0.6, 1.4, 1.2, -0.8, 0.5, 0.0],
        )
    ]
    best = None
    for q0 in seeds:
        q, pe, re = ik.solve(np.asarray(pos, dtype=float), quat, q0, base_pos=BASES[side], iters=120)
        margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
        ok = pe < IK_MM / 1000.0 and math.degrees(re) < IK_DEG
        key = (ok, margin if ok else -(pe * 1000.0 + math.degrees(re)))
        if best is None or key > best[0]:
            best = (key, q, float(pe * 1000.0), float(math.degrees(re)), margin)
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


def tin_wall_hulls() -> list:
    tx, ty = P.TIN_XY
    z0 = TOP + P.OVER
    wh = P.TIN_BASE_H - P.TIN_FLOOR
    wz = z0 + P.TIN_FLOOR + wh / 2.0
    yw = P.TIN_W / 2.0 - P.TIN_WALL / 2.0
    xw = P.TIN_L / 2.0 - P.TIN_WALL / 2.0
    iw = P.TIN_W - 2.0 * P.TIN_WALL
    return [
        ("tin_floor", box_hull((tx, ty, z0 + P.TIN_FLOOR / 2.0), np.eye(3), (P.TIN_L, P.TIN_W, P.TIN_FLOOR))),
        ("tin_back", box_hull((tx, ty - yw, wz), np.eye(3), (P.TIN_L, P.TIN_WALL, wh))),
        ("tin_front", box_hull((tx, ty + yw, wz), np.eye(3), (P.TIN_L, P.TIN_WALL, wh))),
        ("tin_left_end", box_hull((tx - xw, ty, wz), np.eye(3), (P.TIN_WALL, iw, wh))),
        ("tin_right_end", box_hull((tx + xw, ty, wz), np.eye(3), (P.TIN_WALL, iw, wh))),
    ]


def run_offline() -> dict:
    found, rows, ok = {}, {}, True
    bench = [("bench", box_hull((0.36, 0.0, TOP - 0.03), np.eye(3), (1.1, 1.3, 0.06)))]
    tin_walls = tin_wall_hulls()
    last_q: dict[str, np.ndarray] = {}

    for name, (side, pos, rot, opening, is_contact) in poses().items():
        q, mm, deg, margin = solve(side, pos, rot, start=last_q.get(side))
        last_q[side] = q
        found[name] = (side, q, opening)
        hulls = hand_hulls(side, q, opening)
        bench_mm = round(1000.0 * gap(hulls, bench), 1)
        tin_mm = round(1000.0 * gap(hulls, tin_walls), 1)
        z_over = float(pos[2] - TOP)
        height_ok = (not is_contact) or (z_over <= MAX_GRASP_Z)
        tin_ok = name.startswith("R: lid") or tin_mm > 1.0
        good = (
            mm <= IK_MM
            and deg <= IK_DEG
            and margin >= MARGIN_DEG
            and opening <= 0.094
            and height_ok
            and bench_mm > 2.0
            and tin_ok
        )
        ok &= good
        rows[name] = {
            "side": side,
            "at_m": [round(float(v), 4) for v in pos],
            "z_over_bench_mm": round(1000.0 * z_over, 1),
            "ik_mm_deg": [round(mm, 3), round(deg, 3)],
            "least_margin_deg": round(margin, 1),
            "clear_bench_mm": bench_mm,
            "clear_tin_mm": tin_mm,
            "ok": good,
        }

    pairs = {}
    for a, b in TOGETHER:
        (sa, qa, oa), (sb, qb, ob) = found[a], found[b]
        hull_mm = round(1000.0 * gap(hand_hulls(sa, qa, oa), hand_hulls(sb, qb, ob)), 1)
        tcp_mm = round(1000.0 * float(np.linalg.norm(poses()[a][1] - poses()[b][1])), 1)
        good = hull_mm >= HULL_SEP_MM
        ok &= good
        pairs[f"{a} + {b}"] = {"tcp_apart_mm": tcp_mm, "hulls_apart_mm": hull_mm, "ok": good}

    worst_ik_mm = max(r["ik_mm_deg"][0] for r in rows.values())
    least_margin = min(r["least_margin_deg"] for r in rows.values())
    least_sep = min(p["hulls_apart_mm"] for p in pairs.values())
    max_grasp_z_mm = max(r["z_over_bench_mm"] for n, r in rows.items() if poses()[n][4])
    return {
        "pass": bool(ok),
        "summary": {
            "pose_count": len(rows),
            "worst_ik_mm": worst_ik_mm,
            "least_joint_margin_deg": least_margin,
            "least_hand_hull_sep_mm": least_sep,
            "max_contact_z_mm": max_grasp_z_mm,
        },
        "poses": rows,
        "bimanual_pairs": pairs,
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
    res = None
    for _ in range(40):  # 2.0 s idle at 20 Hz
        res = emb.step(action)

    assert res is not None
    moved = {
        name: round(1000.0 * float(np.linalg.norm(m.mk.pose(name)[0] - np.asarray(m.mk.built_pose(name)[0]))), 2)
        for name in ("burner", "stand", "tin_base", "tin_lid")
    }
    score_2s = float(res.info.get("score", 0.0))
    rep = m.report()
    ok = score_2s == 0.0 and rep["zero"] is None and not rep["burner_off"] and max(moved.values()) < 3.0
    out = {
        "pass": bool(ok),
        "moved_from_built_mm": moved,
        "score_after_2s": score_2s,
        "report": rep,
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
        print("G1_FAIL_DETAIL " + json.dumps(offline, indent=1), flush=True)
    frame0_ok = True
    if "--frame0" in sys.argv:
        out_dir = Path(sys.argv[sys.argv.index("--frame0") + 1])
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "g1_offline.json").write_text(json.dumps(offline, indent=1))
        f0 = run_frame0(out_dir)
        print("G1_FRAME0 " + json.dumps(f0, indent=1, default=str), flush=True)
        frame0_ok = bool(f0["pass"])
    return 0 if (offline["pass"] and frame0_ok) else 1


if __name__ == "__main__":
    raise SystemExit(main())
