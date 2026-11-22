#!/usr/bin/env python3
"""Offline reach, joint-margin, collision and two-arm separation checks for ``baby_scale_convert``, plus ``--frame0``.

    scripts/rta simpy <id> mech/tasks/baby_scale_convert/checks.py [--frame0 /workspace/robocurve/outputs/<id>]
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

import demo_kit as kit  # noqa: E402
import parts as P  # noqa: E402
import scene as S  # noqa: E402

TOP = TABLE_TOP["station"]
BASES = {side: np.array(station.BASES[side], dtype=float) for side in ("left", "right")}
HAND_LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")
MAX_JAW = 2.0 * abs(float(C.FINGER_OPEN))
IK_MM, IK_DEG, MARGIN_DEG, SEPARATION = 0.5, 0.6, 5.0, 0.020


def frame(closing, approach) -> np.ndarray:
    a = np.asarray(approach, dtype=float) / np.linalg.norm(approach)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


BTN_ROT = frame((0.0, -1.0, 0.0), (0.18, 0.0, -0.984))
def wing_frame(side: str, azimuth_deg: float = 30.0, tilt_deg: float = 12.0) -> np.ndarray:
    """Pinch of the rolled edge on the open head (+y) or foot (-y) wing: the fingers come in past the wing tip, turned
    ``azimuth_deg`` from the y axis toward +x and ``tilt_deg`` down (a level hand cannot reach), rolled about the
    approach so both pads contain the x axis and close on the rod along a line, not on an edge of the floor."""
    sgn = 1.0 if side == "left" else -1.0
    az, tl = math.radians(azimuth_deg), math.radians(tilt_deg)
    a = np.array([math.sin(az) * math.cos(tl), -sgn * math.cos(az) * math.cos(tl), -math.sin(tl)])
    return frame(np.cross(a, (1.0, 0.0, 0.0)), a)  # closing up on the left, down on the right (mirror of each other)


WING_ROT = {side: wing_frame(side) for side in ("left", "right")}

BTN_TIP_LOCAL = kit.lowest_point(BTN_ROT)
BTN_TIP_WORLD = BTN_ROT @ BTN_TIP_LOCAL

WING_LOCAL_Z = S.EDGE_Z  # rolled-edge axis, 0.016 m above the tray origin: 81 mm over the bench on the scale
WING_Y = S.EDGE_Y  # 0.2545 m: the two grasps 509 mm apart
Z_LIFT = 0.095  # tray origin over the bench while carried: stud heads 11.5 mm over the deck, grasp 111 mm over the bench
Z_ON_MAT = P.MAT_FLOOR_T - S.RIB_Z0  # 0.0271 m: tray origin with its ribs on the mat floor
Z_SET = Z_ON_MAT + 0.003  # lowered to 3 mm above resting, then let go


def btn_tcp(depress: float = 0.0, lift: float = 0.0) -> np.ndarray:
    touch = np.array([P.BTN_X, P.BASE_Y, TOP + P.BTN_TOP_Z - depress + lift])
    return touch - BTN_TIP_WORLD


def wing_tcp(side: str, tray_x: float, tray_z_above_top: float, out: float = 0.0) -> np.ndarray:
    """TCP on (``out = 0``) or retracted along ``-approach`` from (``out > 0``) the open scoop wing."""
    sgn = 1.0 if side == "left" else -1.0
    target = np.array([tray_x, sgn * WING_Y, TOP + tray_z_above_top + WING_LOCAL_Z])
    return target - out * WING_ROT[side][:, 2]


def bimanual_stages() -> list[tuple[str, dict, float, float, set[str]]]:
    sx0 = P.TRAY_X
    sx1 = P.TRAY_X + P.SLIDE_TARGET  # 0.363 m (unlocked)
    mx = P.MAT_X  # 0.630 m
    z_deck = P.BASE_H  # 0.065 m
    z_lift, z_mat = Z_LIFT, Z_SET
    return [
        ("front clear & outside left wing", {
            "right": (np.array([0.158, -0.180, TOP + 0.205]), BTN_ROT, 0.0, False),
            "left": (wing_tcp("left", sx0, z_deck, out=0.065), WING_ROT["left"], 0.056, False),
        }, sx0, z_deck, set()),
        ("hover button & straddle left wing", {
            "right": (btn_tcp(lift=0.040), BTN_ROT, 0.0, False),
            "left": (wing_tcp("left", sx0, z_deck), WING_ROT["left"], 0.056, True),
        }, sx0, z_deck, {"tray"}),
        ("press button & pinch left wing", {
            "right": (btn_tcp(depress=0.0058), BTN_ROT, 0.0, True),
            "left": (wing_tcp("left", sx0, z_deck), WING_ROT["left"], P.EDGE_D, True),
        }, sx0, z_deck, {"button", "tray"}),
        ("slide tray 23 mm back", {
            "right": (btn_tcp(depress=0.0058), BTN_ROT, 0.0, True),
            "left": (wing_tcp("left", sx0 + P.SLIDE_HAND, z_deck), WING_ROT["left"], P.EDGE_D, True),
        }, sx1, z_deck, {"button", "tray"}),
        ("right moves outside right wing", {
            "right": (wing_tcp("right", sx1, z_deck, out=0.065), WING_ROT["right"], 0.056, False),
            "left": (wing_tcp("left", sx1, z_deck), WING_ROT["left"], P.EDGE_D, True),
        }, sx1, z_deck, {"tray"}),
        ("both hands pinch unlocked scoop wings", {
            "right": (wing_tcp("right", sx1, z_deck), WING_ROT["right"], P.EDGE_D, True),
            "left": (wing_tcp("left", sx1, z_deck), WING_ROT["left"], P.EDGE_D, True),
        }, sx1, z_deck, {"tray"}),
        ("lift scoop clear of base", {
            "right": (wing_tcp("right", sx1, z_lift), WING_ROT["right"], P.EDGE_D, True),
            "left": (wing_tcp("left", sx1, z_lift), WING_ROT["left"], P.EDGE_D, True),
        }, sx1, z_lift, {"tray"}),
        ("carry scoop over grey mat", {
            "right": (wing_tcp("right", mx, z_lift), WING_ROT["right"], P.EDGE_D, True),
            "left": (wing_tcp("left", mx, z_lift), WING_ROT["left"], P.EDGE_D, True),
        }, mx, z_lift, {"tray"}),
        ("set scoop upright on grey mat", {
            "right": (wing_tcp("right", mx, z_mat), WING_ROT["right"], P.EDGE_D, True),
            "left": (wing_tcp("left", mx, z_mat), WING_ROT["left"], P.EDGE_D, True),
        }, mx, z_mat, {"tray"}),
        ("open and retract off open scoop ends", {
            "right": (wing_tcp("right", mx, z_mat, out=0.075), WING_ROT["right"], 0.060, False),
            "left": (wing_tcp("left", mx, z_mat, out=0.075), WING_ROT["left"], 0.060, False),
        }, mx, Z_ON_MAT, set()),
    ]


def solve(side: str, pos, rot, start=None):
    q0 = np.array(READY, dtype=float) if start is None else np.asarray(start, dtype=float)
    q, pe, re = ik.solve(np.asarray(pos, dtype=float), ik.matrix_to_quat_xyzw(np.asarray(rot, dtype=float)),
                         q0, base_pos=BASES[side], iters=300)
    margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
    return q, float(pe * 1000.0), float(math.degrees(re)), margin


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
    planes = [[*(s * rotation[:, i]), float((s * rotation[:, i]) @ centre + half[i])] for i in range(3) for s in (1.0, -1.0)]
    return analysis.Hull(vertices=vertices, planes=np.array(planes))


def gap(hulls_a, hulls_b) -> float:
    return min(min(float(analysis._signed(a.vertices, b).min()), float(analysis._signed(b.vertices, a).min()))
               for _, a in hulls_a for _, b in hulls_b)


def scene_boxes(tray_x: float, tray_z: float) -> dict:
    bench = [("bench", box_hull((0.40, 0.0, TOP - 0.030), np.eye(3), (1.10, 1.30, 0.060)))]
    base = [("base", box_hull((P.BASE_X, P.BASE_Y, TOP + P.BASE_H / 2), np.eye(3), (P.BASE_D, P.BASE_W, P.BASE_H)))]
    btn = [("button", box_hull((P.BTN_X, P.BASE_Y, TOP + P.BTN_TOP_Z - 0.004), np.eye(3), (0.028, 0.028, 0.012)))]
    z0 = TOP + tray_z
    tray = [("scoop_floor", box_hull((tray_x, P.TRAY_Y, z0 + 0.5 * sum(S.FLOOR_Z)), np.eye(3), (0.140, 2 * S.EDGE_Y, P.FLOOR_T)))]
    for sgn in (-1.0, 1.0):
        tray.append(("edge", box_hull((tray_x, sgn * S.EDGE_Y, z0 + S.EDGE_Z), np.eye(3), (P.TRAY_D, P.EDGE_D, P.EDGE_D))))
    for y0, y1, rim_top in S.WALL_SEGMENTS:  # each wall segment as its bounding box (covers the slope and its rim)
        for yc, wy in ((0.0, 2 * y1),) if y0 == 0.0 else ((0.5 * (y0 + y1), y1 - y0), (-0.5 * (y0 + y1), y1 - y0)):
            for sgn in (-1.0, 1.0):
                tray.append(("wall", box_hull((tray_x + sgn * 0.0955, yc, z0 + 0.5 * (S.FLOOR_Z[0] + rim_top)), np.eye(3),
                                              (0.059, wy, rim_top - S.FLOOR_Z[0]))))
    for sgn in (-1.0, 1.0):
        tray.append(("rib", box_hull((tray_x, sgn * 0.158, z0 + 0.5 * (S.RIB_Z0 + P.SKID_H)), np.eye(3), (0.180, 0.012, P.SKID_H - S.RIB_Z0))))
    mat = [("mat", box_hull((P.MAT_X, P.MAT_Y, TOP + P.MAT_H / 2), np.eye(3), (P.MAT_D, P.MAT_W, P.MAT_H)))]
    return {"bench": bench, "base": base, "button": btn, "tray": tray, "mat": mat}


def run_offline() -> dict:
    q_prev = {"left": np.array(READY, dtype=float), "right": np.array(READY, dtype=float)}
    stages_out = {}
    ok = True
    max_grasp_z_mm = 0.0
    min_carry_sep_mm = math.inf

    for stage_name, arms_spec, tx, tz, held in bimanual_stages():
        hulls_by_side = {}
        stage_row: dict = {"arms": {}}
        for side in ("left", "right"):
            pos, rot, op, is_grasp = arms_spec[side]
            if is_grasp:
                max_grasp_z_mm = max(max_grasp_z_mm, 1000.0 * float(pos[2] - TOP))
            q, mm, deg, margin = solve(side, pos, rot, start=q_prev[side])
            q_prev[side] = q
            hh = hand_hulls(side, q, op)
            hulls_by_side[side] = hh
            boxes = scene_boxes(tx, tz)
            side_held = set(held)
            if side == "left":
                side_held.discard("button")
            if side == "right" and "button" in held:
                side_held.discard("tray")
                side_held.add("base")
            groups = [g for g in ("bench", "base", "button", "tray", "mat") if g not in side_held]
            clear = {g: round(1000 * gap(hh, boxes[g]), 1) for g in groups}
            z_ok = (not is_grasp) or float(pos[2] - TOP) <= 0.160
            good = bool(mm <= IK_MM and deg <= IK_DEG and margin >= MARGIN_DEG and min(clear.values()) > 0.0 and z_ok)
            ok &= good
            stage_row["arms"][side] = {
                "at_m": [round(float(v), 4) for v in pos],
                "over_bench_mm": round(1000 * float(pos[2] - TOP), 1),
                "is_grasp": is_grasp,
                "ik_mm_deg": [round(mm, 3), round(deg, 3)],
                "margin_deg": round(margin, 1),
                "clear_mm": clear,
                "ok": good,
            }
        tcp_sep = float(np.linalg.norm(np.asarray(arms_spec["left"][0]) - np.asarray(arms_spec["right"][0])))
        hull_sep = gap(hulls_by_side["left"], hulls_by_side["right"])
        if "both hands" in stage_name or "lift" in stage_name or "carry" in stage_name or "set scoop" in stage_name:
            min_carry_sep_mm = min(min_carry_sep_mm, 1000.0 * tcp_sep)
        sep_ok = bool(tcp_sep >= SEPARATION and hull_sep > 0.0)
        ok &= sep_ok
        stage_row["tcp_apart_mm"] = round(1000 * tcp_sep, 1)
        stage_row["hulls_apart_mm"] = round(1000 * hull_sep, 1)
        stage_row["ok"] = bool(sep_ok and stage_row["arms"]["left"]["ok"] and stage_row["arms"]["right"]["ok"])
        stages_out[stage_name] = stage_row

    ok = bool(ok and max_grasp_z_mm <= 160.0 and min_carry_sep_mm >= 280.0)
    return {
        "pass": ok,
        "max_grasp_over_bench_mm": round(max_grasp_z_mm, 1),
        "min_two_arm_carry_tcp_apart_mm": round(min_carry_sep_mm, 1),
        "stages": stages_out,
    }


def run_frame0(out_dir: Path) -> dict:
    import cv2
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
    first, res = None, None
    for _ in range(40):
        res = emb.step(action)
        if first is None:
            first = m.report()
    assert res is not None
    moved = {name: round(1000 * float(np.linalg.norm(m.mk.pose(name)[0] - np.asarray(m.mk.built_pose(name)[0]))), 2)
             for name in ("scale_base", "release_btn", "weighing_tray", "mat")}
    out = {"first_step": first, "after_2s": m.report(), "moved_from_built_mm": moved,
           "score_after_2s": float(res.info.get("score", 0.0))}
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
