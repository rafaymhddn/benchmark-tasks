#!/usr/bin/env python3
"""Where the hands hold the parts, and a check of the demo's poses against the arms' reach, without the simulator.

The grasps (every one with the fingers closing across the part, the pads on the two stiff leads just behind the
T-handle's crossbar: pushing towards the nose, the pads' front edges bear on the crossbar's 48.6 mm wings, a form
lock, so the 44 N push does not rest on the pads' friction):
- left lead grasp: the hand tilted 35 deg from straight down towards the nose; the left hand holds the left
  connector so from the start to the end.
- right lead grasp: the hand behind the connector along its axis, 12 deg down, the pads over the leads' whole
  height (the right housing lies on a 20 mm block, its leads free behind it); the right hand takes the right
  connector so, lifts it, turns it over with the wrist's last joint (the connector then points 24 deg nose-down;
  the left one is brought to it on that axis) and pushes it home.
- cover grasp: from above, tilted 25 deg towards the right arm's side, the pads on the cover's sides (46.5 mm).

``python checks.py`` (in the image): every pose of the demo (and of its unflipped trial) through the arms' IK: error at most 0.5 mm and 0.5 deg,
each joint 5 deg or more inside its limits, the hands' hulls clear of the bench and of each other, and while the
connectors are pushed together the wrists 280 mm apart or more. ``--frame0 <dir>`` (simulator): the start pictures
(full size and the policy's 320 x 176) and a 2 s idle at the ready pose (score 0, nothing moves).
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
FINGER_K = 5000.0  # N/m of each finger's drive (yam_arena constants GRIPPER stiffness)
IK_MM, IK_DEG, MARGIN_DEG, WRIST_SEP = 0.5, 0.5, 5.0, 0.280

# grasp geometry (connector frame, m): the tilt of the hand towards the nose and where the grasp point sits
LEAD_TILT = 35.0  # deg from straight down (steeper, the pads' front corners reach the crossbar's height and close on its ends)
LEAD_AT = (-0.0375, 0.0080)  # (x, z) of the left grasp point: the pads' front corners 3 mm below and behind the crossbar
RIGHT_TILT = 78.0  # along the connector's axis, 12 deg down: the wrist's last joint turns it over about that axis
RIGHT_AT = (-0.0420, 0.0030)  # the pads over the leads' whole height (the block holds them 28 mm up), 1.6 mm behind the crossbar
PINCH_N = 26.0  # N each finger presses with (28 N at most)


def opening(width: float, newtons: float = PINCH_N) -> float:
    """The gripper command (0 closed .. 1 open) that presses ``newtons`` per finger on a part ``width`` wide."""
    return max(0.0, width - 2.0 * newtons / FINGER_K) / MAX_JAW


OPEN_WIDE = 0.075 / MAX_JAW
LEAD_GRIP = opening(P.LEAD_W)  # 22.4 mm leads, 12.0 mm sent: 26 N a finger (28 N at most while the mate is pushed)
TURN_GRIP = opening(P.LEAD_W, 36.0)  # 36 N a finger (the drives read up to 3 N more as the lift starts): the right hand turning its connector, the left carrying the pair
COVER_GRIP = opening(P.CAP_W, 20.0)


def level(side: str) -> np.ndarray:
    """A connector lying level with its nose towards the middle: the left one points to -y, the right one to +y."""
    yaw = -math.pi / 2 if side == "left" else math.pi / 2
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def hand_in_connector(tilt_deg: float, turned_hand: bool = False) -> np.ndarray:
    """The hand's axes (columns: closing, pad width, approach) in a connector's frame: closing across it, the
    approach tilted ``tilt_deg`` from straight down towards the nose; ``turned_hand``: the hand turned half a turn
    about its approach (the pads are alike; it puts the wrist on the other side)."""
    a = math.radians(tilt_deg)
    c2 = np.array([math.sin(a), 0.0, -math.cos(a)])
    c0 = np.array([0.0, -1.0 if turned_hand else 1.0, 0.0])
    return np.column_stack([c0, np.cross(c2, c0), c2])


def lead_grasp(conn_pos, conn_rot) -> tuple[np.ndarray, np.ndarray]:
    """Grasp point and hand rotation (world) of the left lead grasp on a connector at ``conn_pos``/``conn_rot``."""
    x, z = LEAD_AT
    return np.asarray(conn_pos, float) + np.asarray(conn_rot, float) @ np.array([x, 0.0, z]), np.asarray(conn_rot, float) @ hand_in_connector(LEAD_TILT, True)


def right_grasp(conn_pos, conn_rot) -> tuple[np.ndarray, np.ndarray]:
    """The right lead grasp: from behind along the connector's axis, 12 deg down, the wrist above the approach
    while the connector lies '+' face up."""
    x, z = RIGHT_AT
    return np.asarray(conn_pos, float) + np.asarray(conn_rot, float) @ np.array([x, 0.0, z]), np.asarray(conn_rot, float) @ hand_in_connector(RIGHT_TILT, True)


def cover_grasp(cover_pos) -> tuple[np.ndarray, np.ndarray]:
    """Over the cover's middle (3 mm higher), closing along x (across the cover), the hand tilted 25 deg from straight
    down towards its own side, clear of the left hand."""
    from yam_arena.demo import grip_rotation

    a = math.radians(25.0)
    tilt = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])
    return np.asarray(cover_pos, float) + np.array([0.0, 0.0, 0.003]), tilt @ grip_rotation(math.pi / 2)


def spin(rot: np.ndarray, deg: float) -> np.ndarray:
    """``rot`` turned ``deg`` about its own approach axis (the wrist's last joint)."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return rot @ np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def solve(side: str, pos, rot, start=None, flips=(False,)):
    """Best IK solution over a few seeds (and, with ``flips=(False, True)``, the hand's two symmetric turns):
    (joints, mm, deg, margin deg). The demo sends ``rot`` as it is, so the plan is checked unflipped."""
    best = None
    seeds = ([np.asarray(start, float)] if start is not None else []) + [np.array(s, float) for s in (
        READY, [0.0, 1.2, 1.2, -0.5, 0.0, 0.0], [0.0, 1.4, 1.4, -0.4, 0.0, 1.5], [0.0, 1.4, 1.4, -0.4, 0.0, -1.5],
        [0.4, 1.2, 1.3, -0.6, 0.4, 0.0], [-0.4, 1.2, 1.3, -0.6, -0.4, 0.0])]  # fmt: skip
    for flip in flips:
        r = np.asarray(rot, float) @ (np.diag([-1.0, -1.0, 1.0]) if flip else np.eye(3))
        quat = ik.matrix_to_quat_xyzw(r)
        for q0 in seeds:
            q, pe, re = ik.solve(np.asarray(pos, float), quat, q0, base_pos=BASES[side], iters=300)
            margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
            ok = pe < IK_MM / 1000.0 and math.degrees(re) < IK_DEG
            key = (ok, margin if ok else -(pe * 1000.0 + math.degrees(re)))
            if best is None or key > best[0]:
                best = (key, q, float(pe * 1000.0), float(math.degrees(re)), margin)
    return best[1], best[2], best[3], best[4]


def hulls(side: str, q: np.ndarray, width: float) -> tuple[np.ndarray, list]:
    """The wrist's position and the hand's collision hulls (world) for joints ``q`` and fingers ``width`` apart."""
    half = -0.5 * max(0.0, min(MAX_JAW, width))
    joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, q)}, **dict.fromkeys(C.FINGER_JOINTS, half)}
    poses = analysis.fk_link_poses_world(joints, tuple(BASES[side]))
    out = []
    for link in HAND_LINKS:
        for h in analysis.link_hulls().get(link, []):
            r, p = poses[link][:3, :3], poses[link][:3, 3]
            n = h.planes[:, :3] @ r.T
            out.append(analysis.Hull(vertices=h.vertices @ r.T + p, planes=np.c_[n, h.planes[:, 3] + n @ p]))
    return poses["link5"][:3, 3].copy(), out


def gap(a: list, b: list) -> float:
    best = math.inf
    for ha in a:
        for hb in b:
            best = min(best, float(analysis._signed(ha.vertices, hb).min()), float(analysis._signed(hb.vertices, ha).min()))
    return best


def lowest(hs: list) -> float:
    return min(float(h.vertices[:, 2].min()) for h in hs)


def pressed_targets(side: str, q: np.ndarray, force) -> tuple[float, float]:
    """For a push of ``force`` (N, world) at the grasp point: the most any joint's drive is asked for (1 = all it
    gives) and the least margin (deg) of the pushed joint targets to their limits."""
    from demo_kit import JOINT_EFFORT, JOINT_STIFFNESS, point_jacobian

    torque = point_jacobian(q, BASES[side]).T @ np.asarray(force, float)
    pressed = q + torque / JOINT_STIFFNESS
    return float(np.abs(torque / JOINT_EFFORT).max()), float(np.degrees(min((pressed - ik.LOWER).min(), (ik.UPPER - pressed).min())))


def check_plan(poses: dict) -> dict:
    """``poses``: name -> {side: (pos, rot, width)} in the plan's order; every pose through the IK. Pairs (both sides)
    marked ``push`` must keep the wrists WRIST_SEP apart; a pose's ``force`` (side -> N, world) is pushed with there."""
    rows, ok, last = {}, True, {}
    worst_mm, least_margin, least_sep, least_pushed = 0.0, 180.0, math.inf, 180.0
    for name, spec in poses.items():
        row, wrists, hs = {}, {}, {}
        for side in ("left", "right"):
            if side not in spec:
                continue
            pos, rot, width = spec[side]
            q, mm, deg, margin = solve(side, pos, rot, last.get(side))
            last[side] = q
            wrists[side], hs[side] = hulls(side, q, width)
            floor = lowest(hs[side]) - TOP
            held = width < 0.070  # the 16 cm limit is on grasps: an open hand on its way may pass higher
            good = mm <= IK_MM and deg <= IK_DEG and margin >= MARGIN_DEG and floor > 0.001 and (pos[2] - TOP <= 0.160 or not held)
            row[side] = {"ik_mm": round(mm, 3), "margin_deg": round(margin, 1), "hand_over_bench_mm": round(1000 * floor, 1),
                         "grasp_over_bench_cm": round(100 * (pos[2] - TOP), 1)}  # fmt: skip
            if side in spec.get("force", {}):  # the push: the joint targets the kit sends (J^T F / stiffness) stay inside the limits
                effort, pressed = pressed_targets(side, q, spec["force"][side])
                row[side]["push_effort"], row[side]["pushed_margin_deg"] = round(effort, 2), round(pressed, 1)
                good &= effort <= 0.9 and pressed >= 2.0
                least_pushed = min(least_pushed, pressed)
            row[side]["ok"] = bool(good)
            ok &= good
            worst_mm, least_margin = max(worst_mm, mm), min(least_margin, margin)
        if len(hs) == 2:
            sep, hull_gap = float(np.linalg.norm(wrists["left"] - wrists["right"])), gap(hs["left"], hs["right"])
            row["wrists_mm"], row["hands_gap_mm"] = round(1000 * sep, 1), round(1000 * hull_gap, 1)
            good = hull_gap >= 0.020 and (sep >= WRIST_SEP if spec.get("push") else True)
            row["pair_ok"] = bool(good)
            ok &= good
            if spec.get("push"):
                least_sep = min(least_sep, sep)
        rows[name] = row
    return {"pass": bool(ok), "worst_ik_mm": round(worst_mm, 3), "least_margin_deg": round(least_margin, 1),
            "least_wrists_while_pushing_mm": round(1000 * least_sep, 1) if least_sep < math.inf else None,
            "least_margin_pushing_deg": round(least_pushed, 1), "poses": rows}  # fmt: skip


def run_frame0(out_dir: Path) -> dict:
    import cv2
    import demo_kit as kit
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack

    out_dir.mkdir(parents=True, exist_ok=True)
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    obs = emb.reset(Scene(id="frame0", instruction=m.INSTRUCTION), seed=0)
    for cam, stem in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        bgr = cv2.cvtColor(np.asarray(obs.images[cam]), cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(out_dir / f"{stem}.png"), bgr)
        cv2.imwrite(str(out_dir / f"{stem}_policy.png"), cv2.cvtColor(kit.policy_picture(np.asarray(obs.images[cam])), cv2.COLOR_RGB2BGR))
    ready = np.array([*READY, 1.0], dtype=float)
    action = Action(data=pack(ready, ready))
    first, res = None, None
    for _ in range(40):
        res = emb.step(action)
        if first is None:
            first = {n: m.mk.pose(n)[0].copy() for n in m.PARTS}
    moved = {n: round(1000.0 * float(np.linalg.norm(m.mk.pose(n)[0] - first[n])), 2) for n in first}
    out = {"pass": float(res.info.get("score", 0.0)) == 0.0 and max(moved.values()) < 1.0, "score_after_2s": float(res.info.get("score", 0.0)),
           "moved_mm": moved, "report": m.report(), "start_poses": {n: [round(float(v), 4) for v in m.mk.pose(n)[0]] for n in m.PARTS}}  # fmt: skip
    (out_dir / "frame0_check.json").write_text(json.dumps(out, indent=1, default=str))
    return out


def main(argv: list[str]) -> int:
    import demo

    report = {"plan": check_plan(demo.key_poses()), "plan_unflipped": check_plan(demo.key_poses(turned=False))}
    if "--frame0" in argv:
        out_dir = Path(argv[argv.index("--frame0") + 1])
        report["frame0"] = run_frame0(out_dir)
        (out_dir / "checks_report.json").write_text(json.dumps(report, indent=1, default=str))
    print("CHECKS " + json.dumps(report, default=str), flush=True)
    return 0 if report["plan"]["pass"] and report["plan_unflipped"]["pass"] and report.get("frame0", {"pass": True})["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
