#!/usr/bin/env python3
"""The hands' reach and the start of the flip-socket task.

1. Without the simulator: every pose demo.py commands (the left hand on the handle from above and outboard, holding it
   while the right hand pinches each barrel off the shaft onto a stand and threads the red/yellow one back; the left
   holds the tool up while the right turns the green/blue barrel end for end and threads it; the left alone carries
   the tool to the bracket, clocking it; the right takes the green/blue barrel there and both seat it), each solved
   by the arm's inverse kinematics (within 0.5 mm and 0.6 degrees, every joint 6 degrees or more inside its limits),
   the hands' hulls clear of the cradles, the stands, the bracket and the tool's bodies; wherever both hands work at
   once, the grasp points 100 mm or more apart and the hulls clear of each other, and through the barrel work (steps
   1-4) the two wrists' housings 280 mm or more apart.
2. ``--frame0 <dir>`` in the simulator: the first pictures of the three cameras (as rendered and at the size a policy
   gets) and 2 s at rest: the tool still in its start cradle, every barrel on the shaft, the score 0, no zero rule.

    scripts/rta simpy <id> mech/tasks/flip_socket_swap/checks.py [--frame0 /workspace/robocurve/outputs/<id>]
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
import scene  # noqa: E402

TOP = TABLE_TOP["station"]
BASES = {side: np.array(station.BASES[side], dtype=float) for side in ("left", "right")}
HAND_LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")
WRIST_LINKS = ("link4", "link5")
MAX_JAW = 2.0 * abs(float(C.FINGER_OPEN))
IK_MM, IK_DEG, MARGIN_DEG = 0.5, 0.6, 6.0
WRIST_APART = 0.280  # m between the two wrists' housings wherever both hands work at once (the outboard approach)
SEPARATION = 0.100  # m between the two grasp points wherever both hands work at once
OUTBOARD = math.radians(35.0)  # every approach tilts outboard this far off vertical, so the wrists splay apart


def frame(closing, approach) -> np.ndarray:
    """Gripper rotation: columns closing axis, pads' width axis, approach axis."""
    a = np.asarray(approach, dtype=float) / np.linalg.norm(approach)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def outboard(side: str, tilt: float = OUTBOARD) -> np.ndarray:
    """An approach from above, tilted outboard (towards the arm's own side), so the wrist splays away."""
    s = 1.0 if side == "left" else -1.0
    return np.array([0.0, -s * math.sin(tilt), -math.cos(tilt)])


HANDLE_Y = P.HANDLE_MID + 0.020  # m: the left hand holds the handle towards its rear, clear of the right hand's work on the barrels
GRIP_HANDLE = frame((1.0, 0.0, 0.0), outboard("left"))  # closing across the handle's flats, from above and the left
GRIP_HANDLE_ALT = GRIP_HANDLE @ np.diag([-1.0, -1.0, 1.0])  # the same grip, the hand turned half round (its other wrist branch)
CLOCK = math.radians(P.POST_TWIST)  # the roll about the tool's axis that brings the barrels' flats onto the standoff's


def turn(axis, angle: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * k @ k


Y = np.array([0.0, 1.0, 0.0])
PINCH = np.array([math.cos(math.radians(30.0)), 0.0, math.sin(math.radians(30.0))])  # across a flat pair of a level barrel (flats up)
DOWN = np.array([math.sin(math.radians(30.0)), 0.0, -math.cos(math.radians(30.0))])  # the approach, square to the pinch and the barrel
TILT = math.radians(35.0)  # the right hand leans outboard about the pinch near the tool, so the two wrists stay apart


FLIP_AXIS = np.array([-math.sin(math.radians(60.0)), 0.0, math.cos(math.radians(60.0))])  # square to the barrel, 60 degrees off vertical towards the arms
LIFT_H = 0.130  # m: the shaft's axis over the bench while the left hand holds the tool up for the turned barrel's threading
FLIP_AT = np.array([0.220, -0.100, TOP + LIFT_H + 0.020])  # where the hand turns the green/blue barrel end for end, clear of the tool


def flip_grip(share: float) -> np.ndarray:
    """The right hand upright on the barrel (no lean) and turned ``share`` of the half turn about FLIP_AXIS: at 1 the
    barrel lies end for end and the hand is level, pinching the barrel's top and bottom flats from behind."""
    return turn(FLIP_AXIS, -math.pi * share) @ frame(PINCH, DOWN)


def barrel_grip(tilt: float = TILT, flipped: bool = False, clock: float = 0.0) -> np.ndarray:
    """The right hand's turn on a barrel lying along y: pinching the flat pair at 30 degrees, leaning ``tilt``
    outboard; ``flipped``: after the half turn about the approach (the barrel end for end); ``clock``: rolled with the
    tool about its axis."""
    base = frame(PINCH, DOWN)
    if flipped:
        base = turn(DOWN, math.pi) @ base
    return turn(Y, clock) @ turn(PINCH, tilt) @ base


# the barrels' centres on the shaft at the start (m, along y)
ON_SHAFT = {b.key: P.TIP_Y + (span[0] + span[1]) / 2 for b, span in zip(P.BARRELS, scene.START_SPANS)}
AXIS = TOP + P.AXIS_Z
PROUD = 0.0065  # m the blue end stands past the tip once threaded: seated with the tip on the standoff, its face 1 mm short of the bracket
GAP_BEHIND = 0.006  # m the red/yellow barrel stops behind the working seat
F_HOME = P.TIP_Y + P.BARRELS[1].length - PROUD + GAP_BEHIND + P.BARRELS[0].length / 2  # the red/yellow barrel's centre once restacked
IN_SOCKET = 0.005  # m a barrel goes onto the shaft to be aimed again, the tip inside its loose socket
HANDLE_SEND = 0.0229  # m sent to the left fingers on the 38.1 mm grip: about 38 N a finger
LIFT_SAG = 0.016  # m the left arm gives under the tool held up by it alone: commanded that much higher
CARRY_UP = 0.006  # m over the end cradle at the end of the carry
REST_UP = 0.001  # m the grip is held over its rest in the end cradle
BRACKET_GRASP = 0.007  # m the right hand pinches the green/blue barrel inboard of its middle at the bracket


def poses() -> dict:
    """Every pose the plan holds, by name: (side, position, rotation, finger opening m), as demo.py commands them."""
    out = {}
    hold_z = AXIS + 0.0032  # just above the handle's equator: the fingers' tips clear the cradle's wall tops
    front, work = P.BARRELS[0], P.BARRELS[1]
    lifted = TOP + LIFT_H
    out["left: over the handle"] = ("left", np.array([P.DRIVER_X, HANDLE_Y + 0.05, hold_z + 0.05]), GRIP_HANDLE_ALT, MAX_JAW)
    out["left: holding the handle"] = ("left", np.array([P.DRIVER_X, HANDLE_Y, hold_z]), GRIP_HANDLE_ALT, HANDLE_SEND)
    out["left: holding the tool up"] = ("left", np.array([P.DRIVER_X, HANDLE_Y, lifted + LIFT_SAG + (hold_z - AXIS)]), GRIP_HANDLE_ALT, HANDLE_SEND)
    clocked = turn(Y, CLOCK) @ GRIP_HANDLE_ALT
    up = turn(Y, CLOCK) @ np.array([0.0, 0.0, hold_z - AXIS])
    dy_pre = (P.POST_FACE_Y + 0.004) - (P.TIP_Y - PROUD)  # the blue face 4 mm short of the standoff
    out["left: carried down alone, clocked"] = ("left", np.array([P.DRIVER_X, HANDLE_Y + dy_pre, AXIS + CARRY_UP + LIFT_SAG]) + up, clocked, HANDLE_SEND)
    out["left: clocked, over the end cradle"] = ("left", np.array([P.DRIVER_X, HANDLE_Y + dy_pre, AXIS + REST_UP]) + up, clocked, HANDLE_SEND)
    dy = P.POST_FACE_Y - P.TIP_Y
    out["left: tip at the standoff"] = ("left", np.array([P.DRIVER_X, HANDLE_Y + dy, AXIS + REST_UP]) + up, clocked, HANDLE_SEND)
    out["left: let go by the bracket"] = ("left", np.array([P.DRIVER_X, HANDLE_Y + dy + 0.04, AXIS + 0.09]), clocked, MAX_JAW)
    g = barrel_grip()
    for b in (front, work):
        key = b.key
        out[f"{key}: pinch on the shaft"] = ("right", np.array([P.DRIVER_X, ON_SHAFT[key], AXIS]), g, b.od)
        out[f"{key}: slid off the tip"] = ("right", np.array([P.DRIVER_X, P.TIP_Y - b.length / 2 - 0.008, AXIS]), g, b.od)
    out["barrel_14_516: let go on its stand"] = ("right", np.array([P.STAND_X, P.STAND_Y[front.key], AXIS + 0.004]), g, MAX_JAW)
    out["barrel_1132_38: let go on its stand"] = ("right", np.array([P.STAND_X, P.STAND_Y[work.key], AXIS + 0.004]), g, MAX_JAW)
    out["barrel_14_516: at the tip to thread"] = ("right", np.array([P.DRIVER_X, P.TIP_Y - front.length / 2 - 0.010, AXIS]), g, front.od)
    out["barrel_14_516: in its socket"] = ("right", np.array([P.DRIVER_X, P.TIP_Y + IN_SOCKET - front.length / 2, AXIS]), g, front.od)
    out["barrel_14_516: slid home"] = ("right", np.array([P.DRIVER_X, F_HOME, AXIS]), g, front.od)
    out["barrel_14_516: off after restacking"] = ("right", np.array([P.DRIVER_X, F_HOME, AXIS + 0.030]), g, MAX_JAW)
    out["barrel_1132_38: taken upright from its stand"] = ("right", np.array([P.STAND_X, P.STAND_Y[work.key], AXIS]), flip_grip(0.0), work.od)
    out["barrel_1132_38: upright, to be turned"] = ("right", FLIP_AT, flip_grip(0.0), work.od)
    out["barrel_1132_38: half turned"] = ("right", FLIP_AT, flip_grip(0.5), work.od)
    out["barrel_1132_38: end for end"] = ("right", FLIP_AT, flip_grip(1.0), work.od)
    out["barrel_1132_38: turned, at the lifted tip"] = ("right", np.array([P.DRIVER_X, P.TIP_Y - work.length / 2 - 0.010, lifted]), flip_grip(1.0), work.od)
    out["barrel_1132_38: in its socket at the lifted tip"] = ("right", np.array([P.DRIVER_X, P.TIP_Y + IN_SOCKET - work.length / 2, lifted]), flip_grip(1.0), work.od)
    out["barrel_1132_38: threaded at the lifted tip"] = ("right", np.array([P.DRIVER_X, P.TIP_Y - PROUD + work.length / 2, lifted]), flip_grip(1.0), work.od)
    out["right: backed off the lifted tip"] = ("right", np.array([P.DRIVER_X - 0.04, P.TIP_Y - PROUD - 0.030, lifted]), flip_grip(1.0), MAX_JAW)
    out["right: up from the lifted tip"] = ("right", np.array([P.DRIVER_X - 0.04, P.TIP_Y - PROUD - 0.030, lifted + 0.03]), flip_grip(1.0), MAX_JAW)
    out["right: hand turned back, clear of the tool"] = ("right", np.array([P.DRIVER_X - 0.04, P.TIP_Y - PROUD - 0.030, lifted + 0.03]), flip_grip(0.0), MAX_JAW)
    at_bracket = turn(Y, CLOCK) @ barrel_grip(tilt=0.0)  # upright: a lean puts the fingertips into the bracket's wall
    out["barrel_1132_38: taken at the bracket"] = ("right", np.array([P.DRIVER_X, P.POST_FACE_Y + 0.004 + work.length / 2 + BRACKET_GRASP, AXIS]), at_bracket, work.od)
    seat_y = P.POST_FACE_Y - PROUD - 0.0005 + work.length / 2 + BRACKET_GRASP
    out["barrel_1132_38: seated"] = ("right", np.array([P.DRIVER_X, seat_y, AXIS]), at_bracket, work.od)
    out["right: away from the standoff"] = ("right", np.array([P.DRIVER_X, seat_y, AXIS + 0.075]), at_bracket, MAX_JAW)
    return out


# where both hands work at once: the left holds the handle while the right works each barrel on or by the tool (steps 1-4:
# wrist housings 280 mm or more apart), and at the bracket (step 5: grasp points and hulls apart only)
HELD = "left: holding the handle"
UP = "left: holding the tool up"
TOGETHER = tuple((HELD, f"{b.key}: {step}") for b, steps in ((P.BARRELS[0], ("pinch on the shaft", "slid off the tip", "at the tip to thread", "in its socket",
                                                                             "slid home", "off after restacking")),
                                                             (P.BARRELS[1], ("pinch on the shaft", "slid off the tip"))) for step in steps) + tuple(
    (UP, f"barrel_1132_38: {step}") for step in ("turned, at the lifted tip", "in its socket at the lifted tip", "threaded at the lifted tip")) + (
    (UP, "right: backed off the lifted tip"),)
AT_BRACKET = (("left: carried down alone, clocked", "right: hand turned back, clear of the tool"),
              ("left: clocked, over the end cradle", "barrel_1132_38: taken at the bracket"), ("left: tip at the standoff", "barrel_1132_38: seated"))


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


def hand_hulls(side: str, q: np.ndarray, opening: float, links=HAND_LINKS) -> list:
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
    """The fixed parts the hands must stay clear of, and the tool's grip and shaft where they lie at the start."""
    mat_top = TOP + P.MAT[2]
    vertex = TOP + P.AXIS_Z - P.HANDLE_D / 2 * math.sqrt(2.0)
    lean = scene.rot_y(math.pi / 4)
    cradle = []
    for y, vertex_h, length in ((P.HANDLE_MID, P.CRADLE_VERTEX, P.CRADLE_L), (P.CRADLE_END_Y, P.END_VERTEX, P.END_CRADLE_L)):
        cradle.append(("cradle base", box_hull((P.DRIVER_X, y, mat_top + (TOP + vertex_h - mat_top) / 2), np.eye(3), (0.040, length, TOP + vertex_h - mat_top))))
        for s in (1.0, -1.0):  # the two walls of the 120-degree V, as they stand above the base
            a = math.radians(30.0)
            at = np.array([P.DRIVER_X + s * P.CRADLE_RISE * math.cos(a) / 2, y, TOP + vertex_h + P.CRADLE_RISE * math.sin(a) / 2])
            at += np.array([s * math.sin(a), 0.0, -math.cos(a)]) * 0.003
            cradle.append((f"cradle wall {y:+.3f} {s:+.0f}", box_hull(at, scene.rot_y(-s * a), (P.CRADLE_RISE + 0.006, length, 0.006))))
    tray = []
    for b in P.BARRELS[:2]:
        rail_h = AXIS - b.od / 2 - mat_top
        for s in (-1.0, 1.0):
            tray.append((f"stand rail {b.key} {s:+.0f}", box_hull((P.STAND_X, P.STAND_Y[b.key] + s * P.STAND_GAP / 2, mat_top + rail_h / 2), np.eye(3),
                                                                  (P.STAND_RAIL[0], P.STAND_RAIL[1], rail_h))))
    bracket = [("bracket base", box_hull((P.BRACKET_AT[0], P.BRACKET_AT[1], mat_top + 0.003), np.eye(3), (0.090, 0.070, 0.006))),
               ("bracket wall", box_hull((P.BRACKET_AT[0], P.WALL_FACE - 0.003, mat_top + 0.0325), np.eye(3), (0.090, 0.006, 0.053))),
               ("standoff", box_hull((P.DRIVER_X, P.POST_FACE_Y - P.POST_OUT / 2, TOP + P.POST_Z), np.eye(3), (P.POST_AF, P.POST_OUT, P.POST_AF)))]
    tool = [("grip", box_hull((P.DRIVER_X, P.TIP_Y + P.TIP_TO_MID, TOP + P.AXIS_Z), np.eye(3), (P.HANDLE_D, P.HANDLE_L, P.HANDLE_D))),
            ("shaft", box_hull((P.DRIVER_X, P.TIP_Y + P.EXPOSED / 2, TOP + P.AXIS_Z), np.eye(3), (P.SHAFT_AC, P.EXPOSED, P.SHAFT_AC)))]
    return {"cradles": cradle, "stands": tray, "bracket": bracket, "tool": tool}


def run_offline() -> dict:
    found, rows, ok = {}, {}, True
    boxes = scene_boxes()
    for name, (side, pos, rot, opening) in poses().items():
        q, mm, deg, margin, flip = solve(side, pos, rot)
        found[name] = (side, q, opening)
        hulls = hand_hulls(side, q, opening)
        clear = {what: round(1000 * min(gap(hulls, [(f"{n}", b)]) for n, b in boxlist), 1) for what, boxlist in boxes.items()}
        if name.startswith("left:"):
            clear.pop("tool")  # the holding hand may touch what it holds
        good = mm <= IK_MM and deg <= IK_DEG and margin >= MARGIN_DEG and min(clear.values()) > 0.0
        ok &= good
        rows[name] = {"side": side, "at (m)": [round(float(v), 4) for v in pos], "IK (mm, deg)": [round(mm, 3), round(deg, 3)], "turned half round": flip,
                      "least joint margin (deg)": round(margin, 1), "hulls clear of (mm)": clear, "ok": good}  # fmt: skip
    pairs = {}
    for a, b in TOGETHER + AT_BRACKET:
        (sa, qa, oa), (sb, qb, ob) = found[a], found[b]
        tcp = float(np.linalg.norm(poses()[a][1] - poses()[b][1]))
        hull_gap = gap(hand_hulls(sa, qa, oa), hand_hulls(sb, qb, ob))
        wa = np.vstack([h.vertices.mean(axis=0) for l, h in hand_hulls(sa, qa, oa, WRIST_LINKS) if l == "link5"])
        wb = np.vstack([h.vertices.mean(axis=0) for l, h in hand_hulls(sb, qb, ob, WRIST_LINKS) if l == "link5"])
        wrist = float(np.linalg.norm(wa.mean(axis=0) - wb.mean(axis=0)))
        good = tcp >= SEPARATION and hull_gap > 0.0 and (wrist >= WRIST_APART or (a, b) in AT_BRACKET)
        ok &= good
        pairs[f"{a} + {b}"] = {"grasp points apart (mm)": round(1000 * tcp, 1), "wrist housings apart (mm)": round(1000 * wrist, 1),
                               "hulls apart (mm)": round(1000 * hull_gap, 1), "ok": good}  # fmt: skip
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
    for i in range(40):
        res = emb.step(action)
        if first is None:
            first = {"driver at (m)": [round(float(v), 5) for v in m.driver_pose()[0]], "level (deg)": round(m.level_deg(), 3)}
    moved = {name: round(1000 * float(np.linalg.norm(m.mk.pose(name)[0] - np.asarray(m.mk.built_pose(name)[0]))), 2)
             for name in ["driver", *(b.key for b in m.P.BARRELS)]}
    out = {"first step": first,
           "after 2 s": {"driver at (m)": [round(float(v), 5) for v in m.driver_pose()[0]], "level (deg)": round(m.level_deg(), 3)},
           "moved from built (mm)": moved, "score after 2 s": float(res.info.get("score", 0.0)),
           "stored at the start": {b.key: m.on_shaft(b.key) for b in m.P.BARRELS}, "report": m.report()}  # fmt: skip
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
