#!/usr/bin/env python3
"""The hands' reach, hull clearances, and start state of the CryoCane sleeving task.

1. Without the simulator: every pose in the bimanual plan (picking the cane at its top tab, lifting and yawing 180 deg
   level through +x, picking the sleeve near its rear end, coaxial foot alignment, full 265 mm two-hand threading pass,
   and laying the sleeved cane inside the stainless transfer tray) solved by yam_arena's IK (<= 0.5 mm, <= 0.6 deg,
   every joint >= 6.0 deg inside its limits), hulls clear of the bench, prep mat, tray and opposing part, and both
   hands >= 200 mm apart wherever both work at once.
2. ``--frame0 <dir>`` in the simulator: writes the three cameras' start pictures (640 x 360 and 320 x 176 policy size)
   and runs 2.0 s idle at ready to verify stability, 0.0 score and no zero rule.
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
MAX_JAW = 2.0 * abs(float(C.FINGER_OPEN))
IK_MM, IK_DEG, MARGIN_DEG, SEPARATION = 0.5, 0.6, 6.0, 0.200

# Grasp geometry derived from yam_arena finger hull bounds (lowest fingertip is 12.8 mm below TCP on down()):
CANE_GRASP_YC = 0.283  # m from straight foot on the 23 mm wide top ID tab (y_c = 268..290 mm)
CANE_TCP_ABOVE_AXIS = 0.0170  # m: TCP above cane bore axis -> lowest fingertip at z_c = +4.2 mm (10.8 mm clear above mat)
SLEEVE_GRASP_FROM_REAR = 0.022  # m in from sleeve's -y rear rim (251 mm behind +y leading mouth)
SLEEVE_GRASP_YS = -0.5 * P.SLEEVE_L + SLEEVE_GRASP_FROM_REAR  # -0.1145 m in sleeve frame
SLEEVE_TCP_ABOVE_AXIS = 0.0070  # m: TCP above sleeve axis -> lowest fingertip 4.2 mm clear above prep_mat

WORK_X = 0.250  # m forward during coaxial 265 mm sleeve pass
YAW_X = 0.350  # m forward at 90-deg mid-yaw so 283 mm cane stays >= 67 mm ahead of the arm-base line x = 0
Z_AXIS_WORK = TOP + 0.095  # m: coaxial mating height (TCPs <= 11.2 cm over bench, well <= 16 cm)
LEFT_TCP_Z_WORK = Z_AXIS_WORK + CANE_TCP_ABOVE_AXIS
RIGHT_TCP_Z_WORK = Z_AXIS_WORK + SLEEVE_TCP_ABOVE_AXIS + 0.0005

def _rot_x(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])

# Yawing left hand -pi/2 -> 0 -> +pi/2 keeps wrist roll j6 inside [-35, +110] deg (>= 9 deg margin from +-120 deg limits).
# Tiny cantilever pitch trims (-0.65 deg on left for -y cane foot, +1.5 deg on right for +y sleeve mouth) hold both dead level!
ROT_START_L = grip_rotation(-0.5 * math.pi)
ROT_MID_L = grip_rotation(0.0)
ROT_END_L = grip_rotation(0.5 * math.pi)
ROT_COAX_L = _rot_x(-0.65) @ ROT_END_L
ROT_SLV_R = grip_rotation(0.5 * math.pi)
ROT_COAX_R = _rot_x(1.50) @ ROT_SLV_R

# Coaxial stroke stations along y at x = WORK_X (both hands advance inward: left -55 mm, right +225 mm = 280 mm closure):
LEFT_PRE_Y = 0.295  # cane straight foot at +0.295 - 0.283 = +0.012 m
RIGHT_PRE_Y = -0.247  # sleeve +y mouth at -0.247 + 0.251 = +0.004 m (8 mm coaxial pre-entry gap)
LEFT_SEAT_Y = 0.240  # -55 mm inward (j6 margin >= 14 deg)
RIGHT_SEAT_Y = -0.022  # +225 mm inward -> 280 mm total closure (8 mm gap + 265 mm stroke + 7 mm spring preload)

# Placement inside tray (centre (0.365, +0.035)): sleeved assembly centre at y = +0.035 -> left TCP on top tab at y = +0.170
TRAY_DROP_LEFT_Y = P.TRAY_CY + 0.135
TRAY_PLACE_AXIS_Z = TOP + P.TRAY_WALL + P.SLEEVE_R_OUT + 0.003  # 3 mm above tray floor before release


def cane_pick_tcp() -> np.ndarray:
    cane_axis_z = TOP + P.MAT_T - P.BASE_Z
    return np.array([P.MAT_CX, P.CANE_START_FOOT_Y - CANE_GRASP_YC, cane_axis_z + CANE_TCP_ABOVE_AXIS])


def sleeve_pick_tcp() -> np.ndarray:
    slv_axis_z = TOP + P.MAT_T + P.SLEEVE_R_OUT
    return np.array([P.MAT_CX, P.SLEEVE_START_CY + SLEEVE_GRASP_YS, slv_axis_z + SLEEVE_TCP_ABOVE_AXIS])


def poses() -> dict:
    """Every key pose of the bimanual plan: name -> (side, pos, rot, jaw_opening_m)."""
    cp = cane_pick_tcp()
    sp = sleeve_pick_tcp()
    tray_above = np.array([P.TRAY_CX, TRAY_DROP_LEFT_Y, TOP + 0.115])
    tray_lay = np.array([P.TRAY_CX, TRAY_DROP_LEFT_Y, TRAY_PLACE_AXIS_Z + CANE_TCP_ABOVE_AXIS])
    return {
        "L: above cane tab": ("left", cp + np.array([0.0, 0.0, 0.065]), ROT_START_L, MAX_JAW),
        "L: on cane tab": ("left", cp, ROT_START_L, P.TAB_W),
        "L: lift cane level": ("left", np.array([0.270, 0.120, LEFT_TCP_Z_WORK]), ROT_START_L, P.TAB_W),
        "L: yaw 90 deg": ("left", np.array([YAW_X, 0.240, LEFT_TCP_Z_WORK]), ROT_MID_L, P.TAB_W),
        "L: yaw 180 deg ready": ("left", np.array([WORK_X, LEFT_PRE_Y, LEFT_TCP_Z_WORK]), ROT_COAX_L, P.TAB_W),
        "L: coaxial mid": ("left", np.array([WORK_X, 0.5 * (LEFT_PRE_Y + LEFT_SEAT_Y), LEFT_TCP_Z_WORK]), ROT_COAX_L, P.TAB_W),
        "L: coaxial seated": ("left", np.array([WORK_X, LEFT_SEAT_Y, LEFT_TCP_Z_WORK]), ROT_COAX_L, P.TAB_W),
        "L: over tray": ("left", tray_above, ROT_END_L, P.TAB_W),
        "L: lay in tray": ("left", tray_lay, ROT_END_L, MAX_JAW),
        "R: above sleeve tail": ("right", sp + np.array([0.0, 0.0, 0.065]), ROT_SLV_R, MAX_JAW),
        "R: on sleeve tail": ("right", sp, ROT_SLV_R, 2.0 * P.SLEEVE_R_OUT),
        "R: lift sleeve coaxial": ("right", np.array([WORK_X, RIGHT_PRE_Y, RIGHT_TCP_Z_WORK]), ROT_COAX_R, 2.0 * P.SLEEVE_R_OUT),
        "R: coaxial mid": ("right", np.array([WORK_X, 0.5 * (RIGHT_PRE_Y + RIGHT_SEAT_Y), RIGHT_TCP_Z_WORK]), ROT_COAX_R, 2.0 * P.SLEEVE_R_OUT),
        "R: coaxial seated": ("right", np.array([WORK_X, RIGHT_SEAT_Y, RIGHT_TCP_Z_WORK]), ROT_COAX_R, 2.0 * P.SLEEVE_R_OUT),
        "R: release retract": ("right", np.array([WORK_X, RIGHT_SEAT_Y - 0.065, RIGHT_TCP_Z_WORK + 0.030]), ROT_SLV_R, MAX_JAW),
    }


TOGETHER = (
    ("L: lift cane level", "R: above sleeve tail"),
    ("L: yaw 90 deg", "R: on sleeve tail"),
    ("L: yaw 180 deg ready", "R: lift sleeve coaxial"),
    ("L: coaxial mid", "R: coaxial mid"),
    ("L: coaxial seated", "R: coaxial seated"),
    ("L: coaxial seated", "R: release retract"),
)


def solve(side: str, pos, rot, allow_flip: bool = False):
    best = None
    seeds = [
        np.array(s, dtype=float)
        for s in (
            READY,
            [0.0, 1.2, 1.2, -0.5, 0.0, 0.0],
            [0.0, 1.5, 1.5, -0.4, 0.0, 0.0],
            [0.4, 1.3, 1.3, -0.6, 0.0, 0.8],
            [-0.4, 1.3, 1.3, -0.6, 0.0, -0.8],
        )
    ]
    flips = (False, True) if allow_flip else (False,)
    for flip in flips:
        r = np.asarray(rot, dtype=float) @ (np.diag([-1.0, -1.0, 1.0]) if flip else np.eye(3))
        quat = ik.matrix_to_quat_xyzw(r)
        for q0 in seeds:
            q, pe, re = ik.solve(np.asarray(pos, dtype=float), quat, q0, base_pos=BASES[side], iters=300)
            margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
            ok = pe < IK_MM / 1000.0 and math.degrees(re) < IK_DEG
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


def static_boxes() -> dict:
    bench = [("bench", box_hull((0.30, 0.0, TOP - 0.020), np.eye(3), (0.70, 0.90, 0.040)))]
    mat = [("prep_mat", box_hull((P.MAT_CX, 0.0, TOP + 0.5 * P.MAT_T), np.eye(3), (P.MAT_X, P.MAT_Y, P.MAT_T)))]
    w = P.TRAY_WALL
    tray = [
        ("tray_floor", box_hull((P.TRAY_CX, P.TRAY_CY, TOP + 0.5 * w), np.eye(3), (P.TRAY_X, P.TRAY_Y, w))),
        *(
            (f"tray_x_{s}", box_hull((P.TRAY_CX + s * 0.5 * (P.TRAY_X - w), P.TRAY_CY, TOP + 0.5 * P.TRAY_H), np.eye(3), (w, P.TRAY_Y, P.TRAY_H)))
            for s in (-1.0, 1.0)
        ),
        *(
            (
                f"tray_y_{s}",
                box_hull((P.TRAY_CX, P.TRAY_CY + s * 0.5 * (P.TRAY_Y - w), TOP + 0.5 * P.TRAY_H), np.eye(3), (P.TRAY_X - 2.0 * w, w, P.TRAY_H)),
            )
            for s in (-1.0, 1.0)
        ),
    ]
    # At full 265 mm seating, sleeve spans y = (cane_foot_y - 0.008) .. (cane_foot_y + 0.265)
    foot_seated_y = LEFT_SEAT_Y - CANE_GRASP_YC
    slv_seated_cy = foot_seated_y + P.SLIDE_MAX - 0.5 * P.SLEEVE_L
    seated_sleeve = [
        (
            "seated_sleeve",
            box_hull((WORK_X, slv_seated_cy, Z_AXIS_WORK), np.eye(3), (2.0 * P.SLEEVE_R_OUT, P.SLEEVE_L, 2.0 * P.SLEEVE_R_OUT)),
        )
    ]
    return {"bench": bench, "prep_mat": mat, "tray": tray, "seated_sleeve": seated_sleeve}


def run_offline() -> dict:
    boxes = static_boxes()
    found, rows, ok = {}, {}, True
    for name, (side, pos, rot, opening) in poses().items():
        # Keep exact yaw (no 180-deg flip) on the left arm's continuous yaw chain so slerp never jumps!
        q, mm, deg, margin, flip = solve(side, pos, rot, allow_flip=False)
        found[name] = (side, q, opening)
        hulls = hand_hulls(side, q, opening)
        clear = {
            "bench": round(1000.0 * gap(hulls, boxes["bench"]), 1),
            "prep_mat": round(1000.0 * gap(hulls, boxes["prep_mat"]), 1),
            "tray": round(1000.0 * gap(hulls, boxes["tray"]), 1),
        }
        if name == "L: coaxial seated":
            clear["seated_sleeve"] = round(1000.0 * gap(hulls, boxes["seated_sleeve"]), 1)
        height_ok = float(pos[2] - TOP) <= 0.160
        good = mm <= IK_MM and deg <= IK_DEG and margin >= MARGIN_DEG and min(clear.values()) > 0.0 and height_ok
        ok &= good
        rows[name] = {
            "side": side,
            "at (m)": [round(float(v), 4) for v in pos],
            "height over bench (mm)": round(1000.0 * float(pos[2] - TOP), 1),
            "IK (mm, deg)": [round(mm, 3), round(deg, 3)],
            "least joint margin (deg)": round(margin, 1),
            "hulls clear of (mm)": clear,
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
            "grasp points apart (mm)": round(1000.0 * tcp, 1),
            "hulls apart (mm)": round(1000.0 * hull_gap, 1),
            "ok": good,
        }
    return {"pass": bool(ok), "poses": rows, "both hands at once": pairs}


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
    for _ in range(40):
        res = emb.step(action)
        if first is None:
            first = {
                "cane at (m)": [round(float(v), 5) for v in m.cane_pose()[0]],
                "roll / pitch (deg)": [round(m.cane_roll_deg(), 2), round(m.cane_pitch_deg(), 2)],
            }
    last = {
        "cane at (m)": [round(float(v), 5) for v in m.cane_pose()[0]],
        "roll / pitch (deg)": [round(m.cane_roll_deg(), 2), round(m.cane_pitch_deg(), 2)],
    }
    moved = {
        name: round(1000.0 * float(np.linalg.norm(m.mk.pose(name)[0] - np.asarray(m.mk.built_pose(name)[0]))), 2)
        for name in ("cane", "sleeve", "vial_a", "vial_b", "tray")
    }
    out = {
        "first step": first,
        "after 2 s": last,
        "moved from built (mm)": moved,
        "score after 2 s": float(res.info.get("score", 0.0)),
        "report": m.report(),
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
    return 0 if offline["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
