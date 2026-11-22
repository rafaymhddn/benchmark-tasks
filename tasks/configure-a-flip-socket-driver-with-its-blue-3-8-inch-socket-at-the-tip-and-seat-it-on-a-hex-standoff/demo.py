#!/usr/bin/env python3
"""The scripted full-score run of the flip-socket task, and a check of its plan against the arms' reach.

    scripts/rta demo <id> mech/tasks/flip_socket_swap [--check|--quick] [--until <s>]

The left hand pinches the tool's grip (about 33 N a finger) and holds it in its start cradle. The right hand pinches
each barrel across a pair of its flats (the pinch keeps its hex clocked to the shaft's): it slides the red/yellow
barrel off over the tip onto one stand and the green/blue barrel onto the other. It takes the red/yellow barrel back,
aims it at the shaft from where it really sits in the fingers, slides it 5 mm on (the shaft's tip inside its loose
5/16 socket), aims it again there and slides it on to 6 mm behind the working seat, then lets go. The left hand lifts
the tool 130 mm over the bench; the right hand takes the green/blue barrel upright from where it lies on its stand,
turns it end for end in the open air, threads it on the same way until its blue end stands 6.5 mm proud of the tip,
lets go and turns its hand back. The left hand alone carries the tool to the bracket, rolling it 15 degrees about its
axis to the standoff's flats, and levels it by turning about its grasp until the socket is on the standoff's line;
the right hand takes the green/blue barrel where it lies (upright, 7 mm inboard of its middle); the seat aims turn
the tool about the left grasp, roll it onto the flats and square the socket to the standoff's axis; both hands press
until the shaft's tip meets the standoff, let go and return to ready while the score's hold runs out. Every planned
pose is checks.py's (reach, joint limits, collisions); the aims read the simulator's poses of the barrels, the tool
and the standoff (a scripted reference run).

Results in outputs/<id>/full/: result.json (every step), readings.json (the run's summary), sheets of pictures and
the videos unless --check. --check: no video, and the run must end at 1.0. --quick: the plan through the arms' IK
without the simulator. --until <s>: stop the run there (development).
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as K  # noqa: E402
import parts as P  # noqa: E402

OPEN = 1.0
FINGER_STIFFNESS = 5000.0  # N/m of each finger's drive (yam_arena constants: GRIPPER stiffness)
GRIP_N = 16.0  # N a finger presses on a barrel
HANDLE_SEND = 0.0250 / K.MAX_JAW  # 25 mm sent on the 38.1 mm grip: about 33 N a finger (a lighter pinch let the tool turn in the hand, ok9; 38 N
# held it so stiffly that the green/blue barrel jammed at its bore's mouth instead of the tool giving, ok18)
FRONT, WORK, REAR = P.BARRELS
X, TIP = P.DRIVER_X, P.TIP_Y
AXIS = K.TOP + P.AXIS_Z  # the standoff's axis, and a barrel's centre on its stand
SHAFT_Z = AXIS - 0.0008  # the settled shaft's axis: the cradle seats the tool 0.8 mm under its build height
HOLD_Z = SHAFT_Z + 0.004  # the left hand grasps just above the handle's equator
CLOCK = K.CLOCK  # the roll that brings the barrels' flats onto the standoff's
LIFT = 0.045  # m over a barrel for the approach
F_HOME = K.F_HOME  # the red/yellow barrel's centre once restacked: behind the working seat, short of the rear barrel
PROUD = K.PROUD  # m the blue end stands past the tip once threaded: the seat depth once the tip meets the standoff
BACK_OFF = 0.040  # m the level hand backs off the threaded barrel (50 mm left joint 5 only 5.3 degrees inside its limit)
BRACKET_GRASP = 0.007  # m the right hand pinches the green/blue barrel this far inboard of its middle at the bracket, its fingertips clear of the wall
CARRY_UP = 0.006  # m both hands lift the tool for the carry: the handle clears both cradles
REST_UP = 0.001  # m over its rest in the end cradle the handle is let go
LIFT_SAG = 0.016  # m the left arm sags holding the tool up by its handle (run ok5: the shaft rode 16 mm under the target)
AIM_MM, AIM_DEG = 0.008, 8.0  # the largest move and turn one aim makes: beyond it the arm sags differently and the aim overshoots
SQUARE_DEG = 3.0  # the most one aim squares the socket to the standoff's axis (the barrel's play on the shaft is 2.4 degrees)


turn, barrel_grip, DOWN, Y = K.turn, K.barrel_grip, K.DOWN, K.Y


def grip_send(od: float) -> float:
    """The finger opening sent to press ``GRIP_N`` a finger on a barrel of ``od`` across flats."""
    return (od - 2.0 * GRIP_N / FINGER_STIFFNESS) / K.MAX_JAW


def at(x: float, y: float, z: float) -> np.ndarray:
    return np.array([x, y, z], dtype=float)


def off_tip(b, gap: float) -> float:
    """A barrel's centre with its inboard face ``gap`` outboard of the tip (negative: that far onto the shaft)."""
    return TIP - gap - b.length / 2.0


def carried(p, r, dy: float, dz: float, roll: float):
    """A hand's pose (p, r) after the tool is carried dy along y and dz up and rolled ``roll`` about its axis."""
    axis_pt = np.array([X, p[1], SHAFT_Z])
    q = turn(Y, roll)
    return axis_pt + q @ (np.asarray(p) - axis_pt) + np.array([0.0, dy, dz]), q @ r


IN_SOCKET = 0.005  # m a held barrel's inner face goes onto the shaft to be aimed again: the shaft's tip inside its loose 8 mm socket
ENGAGE = {FRONT.key: FRONT.length + WORK.length - PROUD + 0.006, WORK.key: WORK.length - PROUD}  # m its inner face goes on under
# the hand: red/yellow to its place 6 mm behind the working seat (green/blue threads on touching nothing, and the right hand's
# fingertips, leaning along the shaft, stay clear of it while the seating slides green/blue 4.5 mm back), green/blue to its
# place, 6.5 mm proud of the tip


def plan() -> tuple[dict, dict]:
    """The moves of both hands, and the stations (m along the shaft) the aimed moves keep once the director has
    aimed them: label -> (side, barrel key or 'seat', station)."""
    import demo_kit as kit

    Move = kit.Move
    from yam_arena import ik
    from yam_arena.constants import READY

    def ready(side):
        return ik.tcp_pose(np.array(READY, dtype=float), K.BASES[side])

    g = barrel_grip()
    sf, sw = P.STAND_Y[FRONT.key], P.STAND_Y[WORK.key]
    yf, yw = K.ON_SHAFT[FRONT.key], K.ON_SHAFT[WORK.key]
    up_z = K.TOP + K.LIFT_H  # the shaft's axis while the left hand holds the tool up
    rf = K.flip_grip(1.0)  # the hand level, pinching the turned barrel's top and bottom flats from behind
    aimed: dict = {}
    front_only = "--front-only" in sys.argv  # development: the red/yellow barrel off and back on at its start seat
    f_home = TIP + FRONT.length / 2.0 + 0.002 if front_only else F_HOME
    seg1 = [
        # 1: the red/yellow barrel off over the tip, onto its stand
        Move(2.0, pos=at(X, yf, SHAFT_Z + LIFT), rot=g, grip=OPEN, label="over the red/yellow barrel"),
        Move(1.0, pos=at(X, yf, SHAFT_Z), label="down around it"),
        Move(0.6, grip=grip_send(FRONT.od), label="pinch it"),
        Move(2.5, pos=at(X, off_tip(FRONT, 0.008), SHAFT_Z), label="slide it off over the tip"),
        Move(1.0, pos=at(X, off_tip(FRONT, 0.008), SHAFT_Z + 0.040), label="lift it clear"),
        Move(1.4, pos=at(P.STAND_X, sf, AXIS + 0.030), label="over its stand"),
        Move(0.9, pos=at(P.STAND_X, sf, AXIS + 0.004), label="down onto its stand"),
        Move(0.5, grip=OPEN, label="let go"),
        Move(0.8, pos=at(P.STAND_X, sf, AXIS + LIFT), label="up"),
    ]
    seg2 = [
        # 2: the green/blue barrel off over the tip, onto its stand
        Move(1.6, pos=at(X, yw, SHAFT_Z + LIFT), rot=g, label="over the green/blue barrel"),
        Move(1.0, pos=at(X, yw, SHAFT_Z), label="down around it"),
        Move(0.6, grip=grip_send(WORK.od), label="pinch it"),
        Move(3.0, pos=at(X, off_tip(WORK, 0.008), SHAFT_Z), label="slide it off over the tip"),
        Move(1.0, pos=at(X, off_tip(WORK, 0.008), SHAFT_Z + 0.040), label="lift it clear"),
        Move(1.5, pos=at(P.STAND_X, sw, AXIS + 0.030), label="over its stand"),
        Move(0.9, pos=at(P.STAND_X, sw, AXIS + 0.004), label="down onto its stand"),
        Move(0.5, grip=OPEN, label="let go"),
        Move(0.8, pos=at(P.STAND_X, sw, AXIS + LIFT), label="up"),
    ]
    seg3 = [
        # 3: the red/yellow barrel back on, behind the working seat
        Move(1.2, pos=at(P.STAND_X, sf, AXIS + LIFT), rot=g, label="over the red/yellow barrel"),
        Move(0.9, pos=at(P.STAND_X, sf, AXIS), label="down around it"),
        Move(0.6, grip=grip_send(FRONT.od), label="pinch it"),
        Move(0.9, pos=at(P.STAND_X, sf, AXIS + 0.035), label="lift it"),
        Move(1.5, pos=at(X, off_tip(FRONT, 0.010), SHAFT_Z + 0.035), label="over the tip's line"),
        Move(1.0, pos=at(X, off_tip(FRONT, 0.010), SHAFT_Z), label="down to the shaft's line"),
        Move(0.8, label="aim red/yellow"),
        Move(0.6, label="recentre red/yellow"),
        Move(0.6, label="aim red/yellow again"),
        Move(1.4, pos=at(X, off_tip(FRONT, -IN_SOCKET), SHAFT_Z), label="into its socket red/yellow"),
        Move(0.6, label="aim red/yellow in its socket"),
        Move(0.5, label="recentre red/yellow in its socket"),
        Move(0.6, label="aim red/yellow in its socket again"),
        Move(6.5, pos=at(X, off_tip(FRONT, -ENGAGE[FRONT.key]), SHAFT_Z), label="onto the tip red/yellow"),
        Move(0.5, grip=OPEN, label="let go of red/yellow"),
        Move(0.8, pos=at(X, off_tip(FRONT, -ENGAGE[FRONT.key]), SHAFT_Z + 0.030), label="up off it"),
    ]
    seg4 = [
        # 4: the green/blue barrel from its stand, turned end for end in the open air, threaded on the tool the left hand
        # holds up (blue end out, 6.5 mm proud of the tip), let go
        Move(1.4, pos=at(P.STAND_X, sw, AXIS + LIFT), rot=K.flip_grip(0.0), grip=OPEN, label="over the green/blue barrel on its stand"),
        Move(0.9, pos=at(P.STAND_X, sw, AXIS), label="down around it upright"),
        Move(0.6, grip=grip_send(WORK.od), label="pinch it"),
        Move(0.9, pos=at(P.STAND_X, sw, AXIS + 0.040), label="lift it upright"),
        Move(2.0, pos=K.FLIP_AT, label="up into the open"),  # the hand took it upright: the turn's frames are this grasp's
        Move(1.4, rot=K.flip_grip(1.0 / 3.0), label="turn it end for end"),
        Move(1.4, rot=K.flip_grip(2.0 / 3.0), label="turn it end for end"),
        Move(1.4, rot=rf, label="turn it end for end"),
        Move(1.6, pos=at(X, off_tip(WORK, 0.010), up_z), label="to the lifted tip's line"),
        Move(0.8, label="aim green/blue"),
        Move(0.6, label="recentre green/blue"),
        Move(0.6, label="aim green/blue again"),
        Move(1.4, pos=at(X, off_tip(WORK, -IN_SOCKET), up_z), label="into its socket green/blue"),
        Move(0.6, label="aim green/blue in its socket"),
        Move(0.5, label="recentre green/blue in its socket"),
        Move(0.6, label="aim green/blue in its socket again"),
        Move(2.0, pos=at(X, off_tip(WORK, -ENGAGE[WORK.key]), up_z), label="onto the tip green/blue"),
        Move(0.5, grip=OPEN, label="let go of green/blue"),
        Move(0.8, pos=at(X - BACK_OFF, TIP - PROUD - 0.030, up_z), label="back off"),
        Move(0.6, pos=at(X - BACK_OFF, TIP - PROUD - 0.030, up_z + 0.03), label="up"),
        Move(1.2, rot=K.flip_grip(2.0 / 3.0), label="turn the hand back"),
        Move(1.2, rot=K.flip_grip(1.0 / 3.0), label="turn the hand back"),
        Move(1.2, rot=K.flip_grip(0.0), label="turn the hand back"),
    ]
    right = seg1 + seg3 if front_only else seg1 + seg2 + seg3 + seg4
    for b, name in ((FRONT, "red/yellow"), (WORK, "green/blue")):
        aimed[f"into its socket {name}"] = ("right", b.key, off_tip(b, -IN_SOCKET) - TIP)
        aimed[f"onto the tip {name}"] = ("right", b.key, off_tip(b, -ENGAGE[b.key]) - TIP)
    # where the plan puts the green/blue barrel when the right hand goes for it: the director moves those grasps to where it lies
    aimed["@stand"] = ("right", WORK.key, at(P.STAND_X, sw, AXIS))
    hold_at = at(X, K.HANDLE_Y, HOLD_Z)
    left = [
        Move(2.0, pos=at(X, K.HANDLE_Y + 0.05, HOLD_Z + 0.05), rot=K.GRIP_HANDLE_ALT, grip=OPEN, label="over the handle"),
        Move(0.9, pos=hold_at, label="down to the handle"),
        Move(0.7, grip=HANDLE_SEND, label="hold the tool"),
    ]
    if front_only:
        left.append(Move(sum(m.seconds for m in right) - sum(m.seconds for m in left), label="hold the tool"))
        right += [Move(2.5, pos=ready("right")[0], rot=ready("right")[1], label="ready")]
        left += [Move(0.5, grip=OPEN, label="let go"), Move(2.5, pos=ready("left")[0], rot=ready("left")[1], label="ready")]
        return {"left": left, "right": right}, aimed
    # the left hand lifts the tool once the red/yellow barrel is home and the right hand has gone for the other one
    t_lift = sum(m.seconds for m in seg1 + seg2 + seg3) + 0.5
    left.append(Move(t_lift - sum(m.seconds for m in left), label="hold the tool"))
    p_up, r_up = carried(hold_at, K.GRIP_HANDLE_ALT, 0.0, up_z + LIFT_SAG - SHAFT_Z, 0.0)  # commanded higher by the arm's sag under the tool
    left.append(Move(2.5, pos=p_up, rot=r_up, label="hold the tool up"))
    t_carry = sum(m.seconds for m in right)
    left.append(Move(t_carry - sum(m.seconds for m in left), label="hold the tool up"))
    # 5: the left hand alone carries the tool down to the bracket, rolling it to the standoff's flats (commanded higher by
    # its arm's give under the tool), and lowers the grip to just over the end cradle; the right hand, clear meanwhile, then
    # takes the green/blue barrel where it lies (two hands carrying through a barrel that slides on the oiled shaft fought:
    # ok13), and the two aim the socket at the standoff and seat it
    dy_pre = (P.POST_FACE_Y + 0.004) - (TIP - PROUD)  # the blue face 4 mm short of the standoff's near face
    pw = at(X, TIP - PROUD + WORK.length / 2.0, SHAFT_Z)
    p2, r2 = carried(hold_at, K.GRIP_HANDLE_ALT, dy_pre, CARRY_UP + LIFT_SAG, CLOCK)
    p3, r3 = carried(hold_at, K.GRIP_HANDLE_ALT, dy_pre, K.TOP + P.AXIS_Z + REST_UP - SHAFT_Z, CLOCK)
    left += [Move(7.0, pos=p2, rot=r2, label="carry it down to the bracket, rolling it to the standoff's flats"),
             Move(1.2, pos=p3, rot=r3, label="down to the standoff's line"),
             Move(0.6, label="level the tool"), Move(0.6, label="lift the socket to the line"),
             Move(0.6, label="level the tool again"), Move(0.6, label="lift the socket to the line again")]
    pbc = carried(pw, g, dy_pre, AXIS - SHAFT_Z, CLOCK)[0]  # the green/blue barrel's centre there
    # the pinch rolled with it, upright (a lean puts the fingertips into the bracket's wall) and inboard of the middle
    pb, rb = carried(pw + np.array([0.0, BRACKET_GRASP, 0.0]), K.barrel_grip(tilt=0.0), dy_pre, AXIS - SHAFT_Z, CLOCK)
    right += [Move(10.6, label="wait clear of the tool"),
              Move(1.8, pos=pb + np.array([0.0, 0.0, LIFT]), rot=rb, grip=OPEN, label="over the green/blue barrel at the bracket"),
              Move(1.0, pos=pb, label="down around it at the bracket"),
              Move(0.6, grip=grip_send(WORK.od), label="pinch it at the bracket")]
    left += [Move(3.4, label="hold it there")]
    aimed["@bracket"] = ("right", WORK.key, pbc)
    for hand in (right, left):
        hand += [Move(0.8, label="aim at the standoff"), Move(0.6, label="recentre at the standoff"), Move(0.6, label="aim at the standoff again"),
                 Move(0.6, label="recentre at the standoff again"), Move(0.6, label="aim at the standoff a third time")]
    seat_face = P.POST_FACE_Y - PROUD - 0.0005  # the blue face 0.5 mm past where it stands when the tip meets the standoff
    right += [Move(2.0, pos=at(X, seat_face + WORK.length / 2.0, AXIS), label="press the socket home")]
    tip_end = P.POST_FACE_Y - 0.0015  # the shaft's tip pressed 1.5 mm past the standoff's face: it stops there
    p4, r4 = carried(hold_at, K.GRIP_HANDLE_ALT, tip_end - TIP, K.TOP + P.AXIS_Z + REST_UP - SHAFT_Z, CLOCK)
    left += [Move(2.0, pos=p4, rot=r4, label="follow until the shaft meets the standoff")]
    aimed["press the socket home"] = ("right", "seat", seat_face + WORK.length / 2.0)
    aimed["follow until the shaft meets the standoff"] = ("left", "seat", tip_end)
    up_r = at(X, seat_face + WORK.length / 2.0, AXIS + 0.075)
    right += [Move(0.6, label="settle"), Move(0.5, grip=OPEN, label="let go"), Move(1.5, pos=up_r, label="away"),
              Move(2.5, pos=ready("right")[0], rot=ready("right")[1], label="ready"), Move(8.0, label="ready")]
    left += [Move(1.1, label="settle"), Move(0.5, grip=OPEN, label="let go"), Move(1.5, pos=p4 + np.array([0.0, 0.04, 0.09]), label="away"),
             Move(2.5, pos=ready("left")[0], rot=ready("left")[1], label="ready"), Move(7.5, label="ready")]
    return {"left": left, "right": right}, aimed


def save_end_picture(frame, folder: Path) -> None:
    import cv2
    import demo_kit as kit

    cv2.imwrite(str(folder / "end_top.png"), cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(folder / "end_top_policy.png"), cv2.cvtColor(kit.policy_picture(np.asarray(frame)), cv2.COLOR_RGB2BGR))


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    moves, aimed = plan()
    if "--quick" in sys.argv:
        from yam_arena import station

        dry = kit.dry_run(moves, station.BASES)
        print("QUICK " + json.dumps(dry), flush=True)
        return 0
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk

    def roll_deg(r: np.ndarray) -> float:
        """The barrel's hex roll off the shaft's flats about its own axis, folded into -30..30 degrees."""
        a, n_b, n_d = r[:, 0], r[:, 2], m.driver_pose()[1][:, 2]
        n_d = n_d - np.dot(n_d, a) * a
        ang = math.degrees(math.atan2(float(np.dot(np.cross(n_d, n_b), a)), float(np.dot(n_d, n_b))))
        return round((ang + 30.0) % 60.0 - 30.0, 1)

    def in_hand(p) -> list:
        if "right" not in mk.MECH.grippers:
            return []
        tcp, turn_ = mk.MECH.grippers["right"][0], mk.MECH.gripper_turns["right"]
        return [round(1000 * float(v), 1) for v in turn_.T @ (np.asarray(p) - np.asarray(tcp))]

    def barrel_row(key: str) -> dict:
        p, r = mk.pose(key)
        loc = m.local(p)
        return {"s": round(1000 * float(loc[0]), 1), "yz": [round(1000 * float(loc[1]), 1), round(1000 * float(loc[2]), 1)],
                "tilt": round(m.tilt_deg(r[:, 0], m.driver_pose()[1][:, 0]), 1), "roll": roll_deg(r), "hand": in_hand(p), "on": m.on_shaft(key)}

    def extra() -> dict:
        at_, into = m.mouth()
        tip, rt = m.driver_pose()
        return {"level": round(m.level_deg(), 2), "tip (mm)": [round(1000 * float(v), 1) for v in tip - np.array([0.0, 0.0, m.TOP])],
                "face at tip (mm)": round(1000 * m.face_at_tip(), 1), "seat depth (mm)": round(1000 * float(np.dot(m.POST_AT - at_, into)), 1),
                "barrels": {b.key: barrel_row(b.key) for b in m.P.BARRELS}, "stored at tip": m.stored_at_tip(), "seated": m.seated(),
                "in rest": m.in_rest(), "extracted at (s)": m.STATE.get("extracted"), "hands clear (mm)": round(1000 * m.hands_clear(), 1),
                "bore force (N)": {b.key: round(abs(mk.contact_force(b.key, "driver")), 2) for b in m.P.BARRELS}}  # fmt: skip

    folder = out / "full"
    record = "--check" not in sys.argv[2:]
    until = float(sys.argv[sys.argv.index("--until") + 1]) if "--until" in sys.argv else 1e9
    latest: dict = {}
    done: set = set()
    notes: dict = {}

    def start_of(side_moves, label: str) -> float:
        t0 = 0.0
        for mv in side_moves:
            if mv.label == label:
                return t0
            t0 += mv.seconds
        raise KeyError(label)

    def wanted_barrel(key: str, r_now: np.ndarray) -> np.ndarray:
        """The barrel's turn coaxial with the shaft (red end or blue end out), the hex roll nearest its own."""
        tip, rt = m.driver_pose()
        rel = m.scene.rot_z(math.pi) if key == FRONT.key else np.eye(3)
        best = None
        for k in range(6):
            cand = rt @ rel @ P.rot_x(k * math.pi / 3.0)
            ang = math.acos(max(-1.0, min(1.0, (np.trace(cand @ r_now.T) - 1.0) / 2.0)))
            if best is None or ang < best[0]:
                best = (ang, cand)
        return best[1]

    def clamp_turn(d_r: np.ndarray) -> np.ndarray:
        ang = math.acos(max(-1.0, min(1.0, (np.trace(d_r) - 1.0) / 2.0)))
        if ang <= math.radians(AIM_DEG) or ang < 1e-9:
            return d_r
        w, v = np.linalg.eig(d_r)
        axis = np.real(v[:, int(np.argmin(np.abs(w - 1.0)))])
        k = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
        sgn = 1.0 if np.dot(np.array([d_r[2, 1] - d_r[1, 2], d_r[0, 2] - d_r[2, 0], d_r[1, 0] - d_r[0, 1]]), axis) >= 0 else -1.0
        return turn(axis, sgn * math.radians(AIM_DEG))

    def clamp_move(v: np.ndarray) -> np.ndarray:
        n = float(np.linalg.norm(v))
        return v if n <= AIM_MM else v * (AIM_MM / n)

    def move_of(hand, label: str):
        return next(mv for mv in hand.moves if mv.label == label)

    def aim_threading(t: float, arms, key: str, recentre: str | None, labels: list[str], rigid: tuple = ()) -> dict:
        """Move the right hand so the held barrel's axis lies on the shaft's and its hex lines up with the shaft's:
        the world motion taking the barrel from where it is to there moves the hand's present target (``recentre``,
        the same station) and its next targets (each keeps its station along the shaft); ``rigid`` moves take the
        same world motion as they stand."""
        hand = arms["right"]
        p_cmd, r_cmd = hand.state(t)[:2]
        p_b, r_b = mk.pose(key)
        tip, rt = m.driver_pose()
        s_now = float(m.local(p_b)[0])
        p_want = tip + rt[:, 0] * s_now
        d_r = clamp_turn(wanted_barrel(key, r_b) @ r_b.T)
        p_fix = p_want + d_r @ (np.asarray(p_cmd) - p_b)
        p_fix = np.asarray(p_cmd) + clamp_move(p_fix - np.asarray(p_cmd))
        if recentre is not None:
            mv = move_of(hand, recentre)
            mv.pos, mv.rot = p_fix, d_r @ r_cmd
        for label in labels:
            side, _, station = aimed[label]
            mv = move_of(hand, label)
            mv.pos, mv.rot = p_fix + rt[:, 0] * (station - s_now), d_r @ r_cmd
        for label in rigid:
            mv = move_of(hand, label)
            mv.pos, mv.rot = p_fix + d_r @ (np.asarray(mv.pos) - np.asarray(p_cmd)), d_r @ np.asarray(mv.rot)
        loc = m.local(p_b)
        w, v = np.linalg.eig(d_r)
        axis = np.real(v[:, int(np.argmin(np.abs(w - 1.0)))])
        return {"off axis (mm)": [round(1000 * float(loc[1]), 2), round(1000 * float(loc[2]), 2)], "turn (deg)": round(math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(d_r) - 1.0) / 2.0)))), 2),
                "turn axis": [round(float(c), 2) for c in axis], "cmd-barrel (mm)": [round(1000 * float(c), 1) for c in np.asarray(p_cmd) - p_b],
                "fix-cmd (mm)": [round(1000 * float(c), 1) for c in p_fix - np.asarray(p_cmd)], "roll": roll_deg(r_b)}

    def aim_seat(t: float, arms, recentre: str | None, sides=("right", "left")) -> dict:
        """Put the blue socket on the standoff's line and its hex on the standoff's flats by turning the tool about the
        left hand's grasp (the grip stays over the end cradle; the tool droops a few degrees in the left hand, and moving
        the socket across alone drove the grip into the cradle or left the barrel behind: ok16, ok17): the turn takes the
        socket from where it is onto the line (at most AIM_DEG), then rolls the tool about its axis onto the flats. The
        hands' present targets (``recentre``) and the right hand's press and the left hand's follow (each keeping its
        station along y) take that turn about the left grasp."""
        p_b, r_b = mk.pose(WORK.key)
        p_l = np.asarray(arms["left"].state(t)[0], dtype=float)
        u = p_b - p_l
        w = np.array([X, p_b[1], AXIS]) - p_l
        c = np.cross(u, w)
        swing = math.atan2(float(np.linalg.norm(c)), float(np.dot(u, w)))
        d_r = turn(c, min(swing, math.radians(AIM_DEG))) if np.linalg.norm(c) > 1e-12 else np.eye(3)
        r_n = d_r @ r_b
        a = r_n[:, 0] / np.linalg.norm(r_n[:, 0])  # the socket's axis once swung
        want = None
        for k in range(6):  # the barrel's flats onto the standoff's (a flat normal nearest its own)
            cand = m.scene.rot_y(CLOCK) @ m.scene.rot_z(math.pi / 2.0) @ P.rot_x(k * math.pi / 3.0)
            n_w = cand[:, 2] - np.dot(cand[:, 2], a) * a
            ang = math.atan2(float(np.dot(np.cross(r_n[:, 2], n_w), a)), float(np.dot(r_n[:, 2], n_w)))
            if want is None or abs(ang) < abs(want):
                want = ang
        d_r = turn(a, max(-math.radians(AIM_DEG), min(math.radians(AIM_DEG), want))) @ d_r
        # the right hand also squares the socket to the standoff's axis within the barrel's play on the shaft (a socket
        # 1.7 degrees off bound on the standoff 2.5 mm in: ok20), turning about the socket
        a_n = (d_r @ r_b)[:, 0]
        c2 = np.cross(a_n, m.POST_AXIS)
        square = math.atan2(float(np.linalg.norm(c2)), float(np.dot(a_n, m.POST_AXIS)))
        e_r = turn(c2, min(square, math.radians(SQUARE_DEG))) if np.linalg.norm(c2) > 1e-12 else np.eye(3)
        p_sock = p_l + d_r @ (p_b - p_l)
        for side in sides:
            hand = arms[side]
            p_cmd, r_cmd = hand.state(t)[:2]
            p_fix = p_l + d_r @ (np.asarray(p_cmd) - p_l)
            r_fix = d_r @ r_cmd
            if side == "right":
                p_fix, r_fix = p_sock + e_r @ (p_fix - p_sock), e_r @ r_fix
            if recentre is not None:
                mv = move_of(hand, recentre)
                mv.pos, mv.rot = p_fix, r_fix
            for label, (s, _, station) in aimed.items():
                if s == side and _ == "seat":
                    mv = next(mv for mv in hand.moves if mv.label == label)
                    dy = station - (p_b[1] if side == "right" else m.driver_pose()[0][1])
                    mv.pos, mv.rot = p_fix + np.array([0.0, dy, 0.0]), r_fix
        return {"socket off axis (mm)": [round(1000 * float(p_b[0] - X), 2), round(1000 * float(p_b[2] - AXIS), 2)], "swing (deg)": round(math.degrees(swing), 2),
                "hex off the flats (deg)": round(math.degrees(want), 2), "socket off the standoff's axis (deg)": round(math.degrees(square), 2)}

    def aim_grasp(arms, key: str, nominal, labels, rolled=()) -> dict:
        """Move the right hand's next grasp targets (``labels``) by where the barrel really lies against where the plan
        put it; ``rolled`` targets (the carry, after the tool's roll to the standoff's flats) take that offset rolled too."""
        hand = arms["right"]
        d = np.asarray(mk.pose(key)[0], dtype=float) - np.asarray(nominal, dtype=float)
        for label in labels:
            mv = move_of(hand, label)
            mv.pos = np.asarray(mv.pos, dtype=float) + d
        for label in rolled:
            mv = move_of(hand, label)
            mv.pos = np.asarray(mv.pos, dtype=float) + turn(Y, CLOCK) @ d
        return {"barrel off plan (mm)": [round(1000 * float(c), 1) for c in d]}

    def director(t, arms, pictures) -> None:
        if pictures is not None:
            latest.update(pictures)
        right = arms["right"]
        labels_r = {mv.label for mv in right.moves}
        end_of = lambda label: start_of(right.moves, label) + next(mv for mv in right.moves if mv.label == label).seconds  # noqa: E731
        for tag, at_end, where, labels, rolled in (
                ("over the green/blue barrel on its stand", True, "@stand", ("down around it upright", "lift it upright"), ()),
                ("over the green/blue barrel at the bracket", True, "@bracket", ("down around it at the bracket",), ())):
            if tag in labels_r and f"grasp {tag}" not in done:
                when = end_of(tag) if at_end else start_of(right.moves, tag)
                if t >= when - 1.5 / 20.0:
                    done.add(f"grasp {tag}")
                    notes[f"grasp {tag}"] = aim_grasp(arms, WORK.key, aimed[where][2], labels, rolled)
                    print(f"AIM {t:6.2f} grasp {tag}: {notes[f'grasp {tag}']}", flush=True)
        for key, name in ((FRONT.key, "red/yellow"), (WORK.key, "green/blue")):
            rigid, both, last = (), [f"into its socket {name}", f"onto the tip {name}"], [f"onto the tip {name}"]
            for tag, recentre, labels in ((f"aim {name}", f"recentre {name}", both), (f"aim {name} again", None, both),
                                          (f"aim {name} in its socket", f"recentre {name} in its socket", last),
                                          (f"aim {name} in its socket again", None, last)):
                if tag not in labels_r:
                    continue
                if tag not in done and t >= end_of(tag) - 1.5 / 20.0:
                    done.add(tag)
                    notes[tag] = aim_threading(t, arms, key, recentre, labels, rigid)
                    print(f"AIM {t:6.2f} {tag}: {notes[tag]}", flush=True)
        for tag, recentre, sides in (("level the tool", "lift the socket to the line", ("left",)),
                                     ("level the tool again", "lift the socket to the line again", ("left",)),
                                     ("aim at the standoff", "recentre at the standoff", ("right", "left")),
                                     ("aim at the standoff again", "recentre at the standoff again", ("right", "left")),
                                     ("aim at the standoff a third time", None, ("right", "left"))):
            if tag not in labels_r and tag not in {mv.label for mv in arms["left"].moves}:
                continue
            if tag not in done and t >= start_of(arms["left"].moves, tag) + move_of(arms["left"], tag).seconds - 1.5 / 20.0:
                done.add(tag)
                notes[tag] = aim_seat(t, arms, recentre, sides)
                print(f"AIM {t:6.2f} {tag}: {notes[tag]}", flush=True)
        if mk.MECH.poses and int(round(t * 20)) % 10 == 0:
            ex = extra()
            hands = {}
            for side in ("left", "right"):
                if side in mk.MECH.grippers:
                    point, opening = mk.MECH.grippers[side]
                    hands[side] = {"at": [round(1000 * float(v), 1) for v in np.asarray(point) - np.array([0.0, 0.0, m.TOP])],
                                   "sent": [round(1000 * float(v), 1) for v in np.asarray(arms[side].state(t)[0]) - np.array([0.0, 0.0, m.TOP])],
                                   "open": round(float(opening), 2)}
            print(f"ROW {t:6.2f} L {arms['left'].state(t)[3][:28]:28s} R {right.state(t)[3][:30]:30s} " + json.dumps(
                {**{k: ex[k] for k in ("level", "tip (mm)", "face at tip (mm)", "seat depth (mm)", "barrels", "bore force (N)")}, "hands": hands}), flush=True)

    result = kit.run(embodiment, moves, folder, seed=0, director=director, extra=extra, limit_s=min(float(m.RUN_LIMIT_STEPS) / 20.0, until), tail_s=6.0,
                     sheet_frames=16, record=record)  # fmt: skip
    rows = result["rows"]
    if "top_cam" in latest:
        save_end_picture(latest["top_cam"], folder)
    tiers = {t: next((r["t"] for r in rows if r["info"].get("score", 0.0) >= t), None) for t in (0.25, 0.5, 1.0)}
    readings = {
        "score at the end": rows[-1]["info"].get("score"), "seconds": rows[-1]["t"],
        "first 0.25 at (s)": tiers[0.25], "first 0.5 at (s)": tiers[0.5], "first 1.0 at (s)": tiers[1.0],
        "worst level (deg)": max(r["extra"]["level"] for r in rows), "seat depth at the end (mm)": rows[-1]["extra"]["seat depth (mm)"],
        "face at tip at the end (mm)": rows[-1]["extra"]["face at tip (mm)"], "stored at tip at the end": rows[-1]["extra"]["stored at tip"],
        "hands clear at the end (mm)": rows[-1]["extra"]["hands clear (mm)"], "aims": notes,
        "both arms ready at the end": {side: [r[side] for r in rows][-1] in ("done", "ready") for side in ("left", "right")},
        "report": result.get("report"), "worst IK": result.get("worst_ik"), "median step (ms)": result.get("median_step_ms"),
    }  # fmt: skip
    (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
    print("READINGS " + json.dumps(readings, default=str), flush=True)
    if "--check" in sys.argv[2:]:
        ok = readings["score at the end"] == 1.0
        print("CHECK " + ("PASS" if ok else "FAIL"), flush=True)
        return 0 if ok else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
