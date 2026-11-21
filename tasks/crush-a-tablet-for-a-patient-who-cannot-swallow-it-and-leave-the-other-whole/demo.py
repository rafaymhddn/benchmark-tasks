#!/usr/bin/env python3
"""The pill crusher's scripted full-score run:

    scripts/rta demo <id> mech/tasks/pill_crusher [--quick] [--norec]

1. The left hand tips each paracetamol tablet's cup (cups 1 and 2) over the crusher's well: it takes the cup by its
   rim in a pinch tilted 70 deg from straight down, from the side away from cup 3, closed slowly 1 mm past the rim
   (Stop if the pads close further: they would be in the wall), lifts it, carries it over the base and tips
   it to 118 deg by the wrist's roll until the tablet drops into the well, tips it back and sets it down where it
   stood. Cup 3, on the card, is not touched.
2. The right hand takes the crusher's top from above by its cap and carries it over the base. It lines it up 10 mm
   over the rim from its pose (read from the simulator once it stands still), lowers it in steps, lined up again at
   each, and 1 mm over the rim until within 0.4 of the plunger's play a side (at most ALIGN_TRIES, then Stop); sets it
   into the mouth and holds it there, moved by the residual read until the thread takes it (people set the cap on and
   turn; nothing in the simulator holds a free top there). A top standing on the rim is lifted and lined up again.
3. The left hand holds the base from the arms' side, its pinch tilted 70 deg, HOLD_AT over the bench (20 mm, 5 mm or
   more under the cap's lowest edge with the top fully down).
4. The right turns the top down in half-turns (the pinch on the cap turned 180 deg clockwise, let go, lifted, turned
   back, closed again) until the tablet has given (not on to the thread's end over the grains). Then back up in
   half-turns until it sits on the thread's start; it lifts it off and sets it down where it stood; the left lets go.
   One tablet at a time: cup 1 is tipped and its tablet crushed (2 to 4), then cup 2 and its tablet (0.45 N m each;
   both at once turned the base in the left's hold, run w12s41).
5. The left lets go of the base; the right takes it 40 mm up in a pinch tilted 70 deg, carries it over cup 4 leaning
   25 deg (the pressed powder holds), leans it on to 55 deg there, brings its mouth down to 25 mm over cup 4's rim and
   tips it to 118 deg while the grains pour onto the applesauce; lifts it 3 cm, tips it back and sets it down where it
   stood.
6. Both hands back to READY.
Each stage is planned when the one before ends, from the parts' poses as the simulator has them (a scripted demo may
read the simulator's state); --quick runs the plan through the IK from the parts' built poses, without the simulator,
and measures how near the arms come to each other, to the cup on the card and its card, the left's hold to the other
parts, and the right hand (the base in it) to cup 4.
"""

from __future__ import annotations

import json
import math
import os
import sys

if "--quick" in sys.argv:  # the dry run's hull distances are many small products: one thread does them fastest
    for _var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ.setdefault(_var, "1")
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from clearance import box_hull, cone_hull, plan_apart, plan_clear_of  # noqa: E402

OPEN = 0.0939  # m between the pads, fully open
SQUEEZE_CUP, SQUEEZE_CAP, SQUEEZE_BASE = 0.001, 0.012, 0.008  # m the grip is closed past the part's width (the pads press; the cap's 12 mm so that the top does not swing in the pinch: run w12s7, 5.6 deg)
# A cup's pads (flat, upright) touch it only at its rim, the widest of the cone within their reach: its 1 mm past the
# rim's 45 mm press each pad on with 2.5 N (the drive's 5000 N/m). Run w12s30 closed them 3 mm past the cup's width at
# the grasp point, 7.3 mm past the rim: they went 2.4 mm into the 1 mm wall and the jolting 2 g cup threw both tablets.
SQUEEZE_HOLD, HOLD_CLOSE = 0.004, 2.0  # m and s: the left's hold closes gently (run w12s6: closed 8 mm past in 0.8 s, it jolted the base 0.4 mm up and a tablet jumped out of the well)
CUP_AT = 0.028  # m: the grasp point on a cup over its foot: the rim (33 mm) 5 mm over it, in the pads' middle (they reach from 16 to 54 mm over the foot; the hand's lowest point 15 mm over the bench: dry probe)
HOLD_AT, POUR_AT = 0.020, 0.040  # m: the pinches on the base over its foot: the left's hold (the spec's 20 mm, pads about 10 to 30 mm up; 5 mm or more under the cap's lowest edge with the top fully down), the right's for the pour (at 50 mm it closed to 44.3 mm, not on the base: run w12s35).
# The hold's height against the right's strokes (dry probes, the arms' shells' least distance): 25 mm 2.3 mm, 28 mm 0.8 mm, 30 mm 0.2 to 0.4 mm
CAP_OVER = 0.008  # m: the pinch on the cap this above its middle (the fingertips 4.3 mm over the cap's lower edge)
PINCH, HOLD_TILT = 70.0, 70.0  # deg from straight down: the pinches on a cup or the base for tipping it, the left's hold on the base (at 45 deg its fingers rose into the right's on the cap: dry run)
STROKE = (120.0, 30.0, -60.0)  # deg: the right hand's yaw as it turns the top down (clockwise seen from above), in two quarter turns
APPROACH = 0.06  # m back along the approach before a pinch
HIGH = 0.10  # m over the pinch's start: every move to or from a part passes there (a move low across the row swept through the cup on the card: dry run)
CLEAR_RIM = 0.015  # m: a tipped cup's (or the base's) mouth this over the rim it pours into
LIFT = 0.10  # m a part is lifted before it is carried
SEAT_HIGH = 0.0135  # m over the thread's start the face is first lined up: 10 mm over the rim (run w12s6: lined up 1 mm over it from 7.6 mm off, the plunger's edge struck the rim and the top tilted 18 deg in the hand)
SEAT_OVER = 0.0045  # m over the thread's start it is lined up again: 1 mm over the rim
SEAT_STEPS = (0.0105, 0.0075)  # m over the thread's start: on the way down it is lined up again at each (run w12s7: lowered 9 mm in one move it came 4 mm off and 5.6 deg tilted onto the rim)
ALIGN_SHARE = 0.4  # lined up within this share of the plunger's play a side, it is set in; else lined up again from the residual read
ALIGN_TRIES = {"high": 3, "low": 5}  # lining-up tries at each height; at the last one, still off after these: Stop
SEAT_IN = -0.0003  # m: the face is set this far under the thread's start for the take (run w12s9: set 0.2 mm over it, it stopped 0.73 mm over, past the take's 0.5 mm)
SEAT_TRIES, FIX_TRIES = 3, 4  # set-ins from the last height, and corrections in the mouth after each; then Stop
RIM_IN = 0.001  # m: the face this far under the rim, the plunger is in the mouth; higher, it stands on the rim
STILL_MOVE, STILL_HOLD, STILL_HOLDS = 0.00005, 0.3, 8  # m, s: a read is taken once the top stands still against the base (its face moved less than STILL_MOVE in STILL_HOLD; at most STILL_HOLDS holds) (run w12s15_34: read 0.5 s after a move, the hand went on 0.9 mm with no new command)
SHORT = 0.006  # m a pinch on the base stops short of its axis: the palm is 25 mm behind the grasp point and the base 24 mm round it (run w12s2: with no stop short it pushed the base at 10 N)
HOLD_SHORT = 0.010  # m the left's hold stops this short: its fingers 1.2 mm further from the right's (dry probe; the right's pour pinch at 10 mm ended with 2.3 deg of joint margin)
MAX_STROKES = 9
END_TOL = 0.0002  # m: the face this near the thread's end, the screwing is done (it reads 3.01 to 3.02 mm there: run w12s17 turned five more strokes)
# The screwing stops once both paracetamol tablets have given, the face about 4 mm over the floor: turned on towards
# the thread's end over the grains (2.4 mm balls), the base and top were thrown 4 m (run w12s18, the face 3.7 mm up)
# and the base rose 8 mm in the left's pinch (run w12s17). The dry plan's half-turns down and up, as the run has them:
DRY_DOWN, DRY_UP = 3, 4
CUP_SET = {"cup_a": (0.0, 0.0), "cup_b": (0.0, 0.0)}  # m: where an emptied cup is set down, from where it stood
CAP_OPEN = 0.008  # m the right opens past the cap's width to let it go between strokes (12 mm: its fingertips 3 mm into the left's shells, dry probe)
# deg: the leans of a tip, the mouth's lowest point over the target at each (the grains leave from about 70 deg: run
# w12s22); leaned less, a base with its mouth over cup 4 would stand in the cup
TIP_LEANS = (55.0, 65.0, 75.0, 85.0, 95.0, 105.0, 118.0)
# deg: the leans of the carry, CARRY_OVER over the tip's first pose: under the slide angle (30 deg) on the way, the
# pressed powder lets go there, high over the cup with the base nearly still (run w12s26: let go at 30 deg on the
# carry, 14 of 48 grains spilled on the way, and the leaning base passed 3 mm over cup 4's rim)
PRE_LEANS = (25.0, 35.0, 45.0)
CARRY_OVER = 0.05  # m
# The cups (a whole tablet each): carried 3 cm over their first pour pose leaning at most 55 deg (a tablet stays in its
# cup until the wall slopes down to the rim, about 80 deg), brought down and tipped to 118 deg with the mouth's lowest
# point over the well (run w12s29: carried by a move in joint space straight to the tip, cup 1 swept 11 mm over the
# bench into the base and knocked it over; its tablet had been thrown out by the pinch, see SQUEEZE_CUP)
CUP_LEANS = TIP_LEANS
CUP_PRE_LEANS = ()
CUP_OVER = 0.03  # m
# m: a cup's mouth's lowest point over the well's rim as it tips, and that lowest point over the well's middle (the
# aim, in radii from the mouth's middle): with 15 mm and the middle of the mouth's lower half, cup 1's foot came 1.5 mm
# from the base (dry) and struck it (run w12s29); with these, 13.3 mm
CUP_CLEAR, CUP_AIM = 0.025, 1.0
PART_GAP = 0.003  # m: a held part this far at least from what it pours into, on its carry and tip
CARRY_CLEAR = 0.02  # m: a held part's lowest point on its carry over what it pours into (the crusher's rim, cup 4's)
TIP_STEP_S = 1.0  # s from one lean to the next
BASE_CLEAR = 0.025  # m: the base's mouth's lowest point over cup 4's rim as it pours. The right's finger under the tipped base
# came 2.7 mm into cup 4 with the mouth 15 mm over the rim (dry; runs w12s33, w12s34: the hand pushed cup 4 51 mm aside
# as the base came down, and the powder fell on the bench); 20 mm over: 2.3 mm clear, 25 mm: 7.3 mm (dry probe). Run
# w12s23 (15 mm) put 34 of 38 grains in the applesauce.
# m the base's pour aimed this far back from cup 4's middle, against the way it leans: grains leave its lip at up to
# 0.46 m/s across and, poured from over cup 4's middle, 7 of 48 flew over its rim without touching the applesauce (run
# w12s39, traced: from 25 to 43 mm over the rim, in a fan towards the way it leans); the slow ones land 6 mm on. Dry:
# the hand 8.7 mm from cup 4, the base 24.9 mm.
BASE_BACK = 0.015
# The base is carried at 25 deg, under the slide angle (30 deg, SLIDE_ANGLE), CARRY_OVER over its first pour pose, and
# leans on to 55 deg only on its way down to it, over BASE_LEAN_DOWN s (run w12s42: leaned to 35, 45 and 55 deg 50 mm
# over its pour, 3 grains left the well there, all 3 to the bench; 40 of 48 on the applesauce). Carried lower, the
# base's foot comes into cup 4 (dry: the 35 deg pour pose with the carry's 25 deg)
BASE_PRE_LEANS = (25.0,)
BASE_LEANS = TIP_LEANS
BASE_LEAN_DOWN = 3.0  # s
BACK_UP = 0.03  # m a tipped part is lifted straight up before it is tipped back (fast, 0.3 s a step): away from what it poured into
AWAY = np.array([0.04, -0.03, 0.035])  # m: where the right waits, from its grasp point on the cap, while the left takes its hold (up first: its fingertips 30 mm over the cap's top; 80 mm up was out of reach, dry)
EASE, RELEASE_S, CLOSE_S = 0.001, 1.0, 0.8  # m, s: between half-turns the right lets go of the cap slowly (its squeeze eased off to the cap's width + 1 mm over RELEASE_S) and closes on it slowly (from there over CLOSE_S) (run w12s19: let go in 0.4 s, the top snapped 0.4 mm back onto its axis, or dropped 0.8 mm onto the thread's start, and grains hopped out of the well: 5 of 48)
TURN_LIFT = 0.010  # m the open right hand rises off the cap while it turns back between strokes (with none its fingers swept 2.6 to 6 mm into the left's)


def m_crushed(m) -> bool:
    """Whether a paracetamol tablet has been crushed (its grains are then traced)."""
    return any(t != "tablet_er" for t in m.STATE["crushed"])


def grip(width: float) -> float:
    return min(1.0, max(0.0, width / OPEN))


def down(yaw_deg: float) -> np.ndarray:
    from yam_arena.demo import grip_rotation

    return grip_rotation(math.radians(yaw_deg))


def tilted(psi_deg: float, theta_deg: float, sign: float = 1.0) -> np.ndarray:
    """The hand's approach tilted ``theta`` from straight down towards azimuth ``psi``, closing level across it."""
    psi, th = math.radians(psi_deg), math.radians(theta_deg)
    a = np.array([math.sin(th) * math.cos(psi), math.sin(th) * math.sin(psi), -math.cos(th)])
    c = sign * np.array([-math.sin(psi), math.cos(psi), 0.0])
    return np.column_stack([c, np.cross(a, c), a])


def roll(rot: np.ndarray, angle: float) -> np.ndarray:
    """``rot`` turned by ``angle`` about its own approach axis (its third column)."""
    a = rot[:, 2]
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return (np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * k @ k) @ rot


def rot_y(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])


# The pinches and pours (dry checks, steep.py: IK margins 10 deg): a cup or the base is taken in a pinch 70 deg from
# straight down (from the side away from the cup on the card), carried over its pour point at 71 deg and tipped to
# 118 deg by the wrist's roll alone, the hand level; a pinch 30 deg from straight down tips it no further than 60 deg
# with a level hand, the wrist's roll range (+-120 deg) short of the rest.
CUP_GRASP = {"cup_a": tilted(270.0, PINCH), "cup_b": tilted(15.0, PINCH)}  # cup 2 from 15 deg: the only pinch of 330 to 120 deg whose roll tip the left reaches (dry probe)
BASE_GRASP = tilted(330.0, PINCH)
# The base is tipped by the wrist's roll alone about its pinch's approach: rolled BASE_TIP from the pinch, a base held
# 70 deg from straight down leans 118 deg, and leans steadily on the way (the carry 67 deg at most, dry). A level hand
# for the pour needed the wrist turned the long way: the move there swung the full base to 151 deg (run w12s19)
BASE_TIP = math.acos((math.cos(math.radians(118.0)) - math.cos(math.radians(PINCH)) ** 2) / math.sin(math.radians(PINCH)) ** 2)
BASE_POUR = roll(BASE_GRASP, BASE_TIP)


def lean_roll(lean_deg: float) -> float:
    """The wrist's roll (rad) from the base's pinch that leans the base ``lean_deg`` from upright (its axis 90 + PINCH
    deg from the pinch's approach: cos(lean) = cos^2(PINCH) + sin^2(PINCH) cos(roll))."""
    c = (math.cos(math.radians(lean_deg)) - math.cos(math.radians(PINCH)) ** 2) / math.sin(math.radians(PINCH)) ** 2
    return math.acos(max(-1.0, min(1.0, c)))
HOLD = rot_y(-HOLD_TILT) @ down(0.0)  # the left on the base, from the arms' side, the camera up


_SEEDS: list = []


def seeds() -> list:
    """Starting joints for the IK: READY and 24 others round it (fixed), clipped inside the joints' limits."""
    from yam_arena import ik
    from yam_arena.constants import READY

    if not _SEEDS:
        rng = np.random.default_rng(0)
        ready = np.array(READY, dtype=float)
        _SEEDS.append(ready)
        for _ in range(24):
            _SEEDS.append(np.clip(ready + rng.uniform(-1.2, 1.2, 6) * np.array([1.0, 0.6, 0.6, 1.0, 1.0, 1.0]), ik.LOWER[:6] + 0.05, ik.UPPER[:6] - 0.05))
    return _SEEDS


def margin_of(q) -> float:
    from yam_arena import ik

    return float(np.degrees(np.minimum(q[:6] - ik.LOWER[:6], ik.UPPER[:6] - q[:6])).min())


def best_ik(side: str, pos, rot, near=None) -> np.ndarray:
    """The arm's joints for the grasp point at ``pos`` turned as ``rot``: of the solutions found from many starts, the
    one nearest ``near`` among those within 5 deg of the most joint margin. Stop if none reaches."""
    from yam_arena import ik, station

    found = []
    for s in ([np.asarray(near, dtype=float)] if near is not None else []) + seeds():
        q, pe, re = ik.solve(np.asarray(pos, dtype=float), ik.matrix_to_quat_xyzw(np.asarray(rot, dtype=float)), s, base_pos=station.BASES[side], iters=200)
        if pe <= 0.0003 and re <= math.radians(0.3):
            found.append((margin_of(q), q))
    if not found:
        raise Stop(f"no joints reach {np.round(pos, 3).tolist()} for the {side} arm")
    best = max(f[0] for f in found)
    good = [f for f in found if f[0] >= best - 5.0]
    if near is not None:
        good.sort(key=lambda f: float(np.abs(f[1] - near).max()))
    return good[0][1]


def ik_near(side: str, pos, rot, near, least_margin: float = 3.0) -> np.ndarray:
    """The arm's joints for the grasp point at ``pos`` turned as ``rot`` nearest ``near`` (the largest joint's
    difference) among the solutions found from many starts with ``least_margin`` deg of joint margin or more: kept on
    the branch of ``near``, where ``best_ik`` takes the most margin. Stop if none reaches."""
    from yam_arena import ik, station

    found = []
    for s in [np.asarray(near, dtype=float)] + seeds():
        q, pe, re = ik.solve(np.asarray(pos, dtype=float), ik.matrix_to_quat_xyzw(np.asarray(rot, dtype=float)), s, base_pos=station.BASES[side], iters=200)
        if pe <= 0.0003 and re <= math.radians(0.3) and margin_of(q) >= least_margin:
            found.append(q)
    if not found:
        raise Stop(f"no joints reach {np.round(pos, 3).tolist()} for the {side} arm with {least_margin:.0f} deg of margin")
    return min(found, key=lambda q: float(np.abs(q - np.asarray(near, dtype=float)).max()))


class Stop(Exception):
    """The plan cannot go on as planned (a grip or a part is not where it must be): both arms go back to READY."""


class Plan:
    """The stages, each planned from the parts' poses when it starts (the simulator's, or as built in a dry run)."""

    def __init__(self, m, live: bool, case: str = "full") -> None:
        from demo_kit import Move

        self.m, self.live, self.Move, self.mk, self.case = m, live, Move, m.mk, case
        self.set_up = case == "full"
        # One tablet at a time (TABLET_NM 0.45 N m a tablet: two at once, 0.9 to 1.1 N m, turned the base 34 deg in the
        # left's hold, run w12s41): each tipped into the well, then crushed in a cycle of its own
        cycle = [self.top_over, self.take_top, self.seat_over, self.seat_align_high, self.seat_down, self.seat_align, self.seat_in, self.seat_fix, self.step_away,
                 self.hold_over, self.hold_base, self.back_on_cap, self.screw, self.unscrew, self.top_down, self.release_base]  # fmt: skip
        self.stages = [self.tip_a_over, self.pinch_a, self.tip_a, *cycle, self.tip_b_over, self.pinch_b, self.tip_b, *cycle, self.base_over, self.take_base, self.pour,
                       self.away]  # fmt: skip
        self.i, self.again, self.strokes, self.turned, self.aligned, self.still, self.current = 0, False, 0, False, {}, {}, ""
        self.census = False
        self.gives, self.most, self.ref = 0, {}, None  # the twist readings at the gives (read_twist)
        self.cup_at: dict = {}  # cup: (where it stood, the pinch's grasp point, the arm's miss there)
        self.log: list = []

    # --- the parts' poses ---
    def pose(self, name: str) -> tuple[np.ndarray, np.ndarray]:
        if self.live:
            p, r = self.mk.pose(name)
            return np.array(p, dtype=float), np.array(r, dtype=float)
        return self.dry[name]

    def start_dry(self) -> None:
        bodies = self.m.mk.MECH.bodies
        self.dry = {name: (np.array(bodies[name].pos, dtype=float), bodies[name].rotation()) for name in ("cup_a", "cup_b", "cup_er", "base", "top", "sauce")}

    def hand(self, side: str) -> np.ndarray:
        """Where the plan has the hand's grasp point now (the end of its last move)."""
        return np.array(self.arms[side].state(1e9)[0], dtype=float)

    def hand_rot(self, side: str) -> np.ndarray:
        return np.array(self.arms[side].state(1e9)[1], dtype=float)

    def cart(self, side: str, pos, rot, seconds: float, label: str, grip=None) -> list:
        """A straight move of the grasp point (a short approach): the planned joints follow it through the IK."""
        from yam_arena import ik, station

        rot = self.qrot[side] if rot is None else np.asarray(rot, dtype=float)
        pos = self.qpos[side] if pos is None else np.asarray(pos, dtype=float)
        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.qp[side], base_pos=station.BASES[side], iters=200)
        if pe > 0.0005 or re > math.radians(0.5) or margin_of(q) < 3.0:
            raise Stop(f"{label}: the {side} arm cannot follow a straight move to {np.round(pos, 3).tolist()} ({1000 * pe:.1f} mm, {math.degrees(re):.1f} deg, margin {margin_of(q):.1f} deg)")
        self.qp[side], self.qpos[side], self.qrot[side] = q, pos, rot
        return [self.Move(seconds, pos, rot, grip, label)]

    def transfer(self, side: str, pos, rot, seconds: float, label: str, grip=None, step_deg: float = 6.0, q1=None) -> list:
        """A move in joint space to the best joints for the pose (or to ``q1``, joints for it): waypoints every
        ``step_deg`` of the largest joint's travel, each the grasp point's pose at the joints on the straight line
        between the two sets of joints."""
        from yam_arena import ik, station

        rot = self.qrot[side] if rot is None else np.asarray(rot, dtype=float)
        q0 = self.qp[side]
        q1 = best_ik(side, pos, rot, near=q0) if q1 is None else np.asarray(q1, dtype=float)
        n = max(1, int(math.ceil(float(np.degrees(np.abs(q1 - q0)).max()) / step_deg)))
        moves = []
        for k in range(1, n + 1):
            qk = q0 + (q1 - q0) * k / n
            pk, rk = (np.asarray(pos, dtype=float), rot) if k == n else ik.tcp_pose(qk, station.BASES[side])
            moves.append(self.Move(seconds / n, pk, rk, grip if k == 1 else None, label))
        self.qp[side], self.qpos[side], self.qrot[side] = q1, np.asarray(pos, dtype=float), rot
        return moves

    def begin_stage(self) -> None:
        """The planned joints and pose at a stage's start: where the arms' IK has them now."""
        self.qp = {side: np.array(self.arms[side].q, dtype=float) for side in ("left", "right")}
        self.qpos = {side: self.hand(side) for side in ("left", "right")}
        self.qrot = {side: self.hand_rot(side) for side in ("left", "right")}

    def fitted(self, side: str, target) -> np.ndarray:
        """The command that brings the hand's grasp point to ``target``: its last command moved by what the arm misses
        there (the simulator's grasp point against the command; nothing in a dry run). Read after a settle."""
        target = np.asarray(target, dtype=float)
        if not self.live:
            return target
        return target + self.hand(side) - np.array(self.mk.gripper(side)[0], dtype=float)

    def require_grip(self, side: str, width: float, what: str, slack: float = 0.0015) -> None:
        if not self.live:
            return
        opening = OPEN * float(self.mk.gripper(side)[1])
        if abs(opening - width) > slack:
            raise Stop(f"the {side} hand closed at {1000 * opening:.1f} mm, not on {what} ({1000 * width:.1f} mm)")

    def pour_point(self, name: str, grasp_at, grasp_rot, pour_rot, height: float, rim_r: float, target, rim_z: float, clear: float = None, aim_k: float = 0.5,
                   back: float = 0.0) -> np.ndarray:
        """Where the grasp point goes for the pour: the part held as it was taken, turned to ``pour_rot``, with the
        point ``aim_k`` of the mouth's radius from its middle towards its lowest point (0.5: the middle of its lower
        half) over ``target`` (x, y), moved ``back`` against the way the part leans (its axis across), and its mouth's
        lowest point ``clear`` (default CLEAR_RIM) over ``rim_z``."""
        clear = CLEAR_RIM if clear is None else clear
        p, r = self.pose(name)
        offset = grasp_rot.T @ (p - np.asarray(grasp_at, dtype=float))
        turned = pour_rot @ grasp_rot.T @ r
        mouth = pour_rot @ offset + turned @ np.array([0.0, 0.0, height])
        n = turned[:, 2]
        d = np.array([0.0, 0.0, -1.0]) - (np.array([0.0, 0.0, -1.0]) @ n) * n
        d /= np.linalg.norm(d)
        aim, lowest = mouth + aim_k * rim_r * d, mouth + rim_r * d
        across = np.array([n[0], n[1], 0.0]) / max(math.hypot(float(n[0]), float(n[1])), 1e-9)
        return np.array([target[0] - back * across[0] - aim[0], target[1] - back * across[1] - aim[1], rim_z + clear - lowest[2]])

    # --- the stages: each returns {side: [Move, ...]} ---
    def spill(self, name: str, grasp_at, grasp_rot, moves: list, height: float, rim_r: float, target, start=None, past: float = 90.0) -> dict:
        """Along ``moves`` (the hand holding ``name`` as it was taken): the part's most tilt, and how far the lowest point
        of its mouth strays from ``target`` (x, y) while the part is over 90 deg. With ``start`` (the hand's pose before
        the moves) each move is also sampled on its way, as the arm interpolates it (straight, the turn slerped)."""
        from yam_arena import ik

        p, r = self.pose(name)
        offset = grasp_rot.T @ (p - np.asarray(grasp_at, dtype=float))
        rel = grasp_rot.T @ r
        poses = []
        last = None if start is None else (np.asarray(start[0], dtype=float), np.asarray(start[1], dtype=float))
        for mv in moves:
            if mv.pos is None or mv.rot is None:
                continue
            pos, rot = np.asarray(mv.pos, dtype=float), np.asarray(mv.rot, dtype=float)
            if last is not None:
                poses += [(last[0] + f * (pos - last[0]), ik.slerp_matrix(last[1], rot, f)) for f in np.linspace(0.1, 0.9, 9)]
            poses.append((pos, rot))
            last = (pos, rot)
        most_tilt, stray = 0.0, 0.0
        for pos, rot in poses:
            turned = rot @ rel
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(turned[2, 2])))))
            n = turned[:, 2]
            d = np.array([0.0, 0.0, -1.0]) - (np.array([0.0, 0.0, -1.0]) @ n) * n
            d = d / max(np.linalg.norm(d), 1e-9)
            lowest = pos + rot @ offset + turned @ np.array([0.0, 0.0, height]) + rim_r * d
            most_tilt = max(most_tilt, tilt)
            if tilt > past:
                stray = max(stray, float(np.hypot(lowest[0] - target[0], lowest[1] - target[1])))
        return {"part": name, "most tilt (deg)": round(most_tilt, 1), f"mouth's lowest point off the target past {past:.0f} deg (mm)": round(1000 * stray, 1)}

    def part_gap(self, name: str, grasp_at, grasp_rot, moves: list, height: float, rim_r: float, foot_r: float, avoid, start) -> float:
        """The least gap (m) between the held part's foot and mouth rings and an upright solid cylinder ``avoid`` =
        (axis x, y, bottom z, top z, radius) along ``moves`` from ``start``, each move sampled on its way; negative
        inside it."""
        from yam_arena import ik

        p, r = self.pose(name)
        offset = grasp_rot.T @ (p - np.asarray(grasp_at, dtype=float))
        rel = grasp_rot.T @ r
        ring = [np.array([math.cos(a), math.sin(a), 0.0]) for a in np.linspace(0.0, 2 * math.pi, 24, endpoint=False)]
        cx, cy, z0, z1, radius = avoid
        poses, last = [], (np.asarray(start[0], dtype=float), np.asarray(start[1], dtype=float))
        for mv in moves:
            if mv.pos is None or mv.rot is None:
                continue
            pos, rot = np.asarray(mv.pos, dtype=float), np.asarray(mv.rot, dtype=float)
            poses += [(last[0] + f * (pos - last[0]), ik.slerp_matrix(last[1], rot, f)) for f in np.linspace(0.0, 1.0, 11)]
            last = (pos, rot)
        gap = math.inf
        for pos, rot in poses:
            origin, turn = pos + rot @ offset, rot @ rel
            for u in ring:
                for q in (origin + turn @ (foot_r * u), origin + turn @ (rim_r * u + np.array([0.0, 0.0, height]))):
                    h = math.hypot(float(q[0]) - cx, float(q[1]) - cy) - radius
                    dz = max(z0 - float(q[2]), float(q[2]) - z1, 0.0)
                    gap = min(gap, math.hypot(max(h, 0.0), dz) if (h > 0.0 or dz > 0.0) else max(h, z0 - float(q[2]), float(q[2]) - z1))
        return gap

    def path_low(self, side: str, q1, name: str, grasp_at, grasp_rot, height: float, rim_r: float, foot_r: float) -> float:
        """The held part's lowest point (world z) on a move in joint space from the planned joints to ``q1``, sampled
        every degree of the largest joint's travel: its foot's and its mouth's edges (circles ``foot_r`` and ``rim_r``)."""
        from yam_arena import ik, station

        p, r = self.pose(name)
        offset = grasp_rot.T @ (p - np.asarray(grasp_at, dtype=float))
        rel = grasp_rot.T @ r
        ring = [np.array([math.cos(a), math.sin(a), 0.0]) for a in np.linspace(0.0, 2 * math.pi, 24, endpoint=False)]
        q0 = np.asarray(self.qp[side], dtype=float)
        q1 = np.asarray(q1, dtype=float)
        n = max(1, int(math.ceil(float(np.degrees(np.abs(q1 - q0)).max()))))
        lowest = math.inf
        for k in range(n + 1):
            pk, rk = ik.tcp_pose(q0 + (q1 - q0) * k / n, station.BASES[side])
            pk, rk = np.asarray(pk, dtype=float), np.asarray(rk, dtype=float)
            origin, turn = pk + rk @ offset, rk @ rel
            for u in ring:
                lowest = min(lowest, float((origin + turn @ (foot_r * u))[2]), float((origin + turn @ (rim_r * u + np.array([0.0, 0.0, height])))[2]))
        return lowest

    def path_tilt(self, side: str, q1, rel) -> float:
        """The held part's most lean (deg from upright) on a move in joint space from the planned joints to ``q1``,
        sampled every degree of the largest joint's travel; ``rel``: the part's turn in the hand."""
        from yam_arena import ik, station

        q0 = np.asarray(self.qp[side], dtype=float)
        q1 = np.asarray(q1, dtype=float)
        n = max(1, int(math.ceil(float(np.degrees(np.abs(q1 - q0)).max()))))
        most = 0.0
        for k in range(n + 1):
            _, rk = ik.tcp_pose(q0 + (q1 - q0) * k / n, station.BASES[side])
            most = max(most, math.degrees(math.acos(max(-1.0, min(1.0, float((np.asarray(rk) @ rel)[2, 2]))))))
        return most

    def tip_over(self, cup: str) -> dict:
        """The left hand over the cup, then down to where its pinch starts (back along the approach), settled."""
        p, _ = self.pose(cup)
        g = CUP_GRASP[cup]
        a = g[:, 2]
        at = p + np.array([0.0, 0.0, CUP_AT])
        s = cup[-1].upper()
        moves = self.transfer("left", at + [0, 0, HIGH], g, 2.0, f"L over cup {s}", grip=1.0)  # from READY (tip_in parks it)
        moves += self.cart("left", at - APPROACH * a, None, 1.0, f"L down to cup {s}")
        moves += self.cart("left", None, None, 0.6, "L settles")
        return {"left": moves, "right": []}

    def pinch(self, cup: str) -> dict:
        """The pinch on the cup's rim (its start corrected by what the arm misses there): closed to the rim, then slowly
        past it, and held a moment so that the grip is read at rest."""
        m = self.m
        p, _ = self.pose(cup)
        a = CUP_GRASP[cup][:, 2]
        at = p + np.array([0.0, 0.0, CUP_AT])
        fit = self.fitted("left", at - APPROACH * a) - (at - APPROACH * a)
        self.cup_at[cup] = (p, at, fit)
        s = cup[-1].upper()
        moves = self.cart("left", at + fit, None, 1.0, f"L in cup {s}")
        moves += self.cart("left", None, None, 0.4, f"L closes to cup {s}", grip=grip(m.CUP_TOP_D + EASE))
        moves += self.cart("left", None, None, CLOSE_S, f"L grips cup {s}", grip=grip(m.CUP_TOP_D - SQUEEZE_CUP))
        moves += self.cart("left", None, None, 0.3, f"L holds cup {s}")
        return {"left": moves, "right": []}

    def tip_in(self, cup: str) -> dict:
        """With the pinch on the rim (Stop if the pads closed further), the lift, the carry, the tip by the wrist's roll,
        and back: the cup set down where it stood."""
        m = self.m
        s = cup[-1].upper()
        self.require_grip("left", m.CUP_TOP_D, f"cup {s}'s rim")
        p, at, fit = self.cup_at[cup]
        g = CUP_GRASP[cup]
        a = g[:, 2]
        bp, br = self.pose("base")
        well = bp + br @ np.array([0.0, 0.0, m.BASE_H])
        moves = self.cart("left", at + fit + [0, 0, LIFT], None, 1.2, f"L lifts cup {s}")
        carrying, tipping, back, _ = self.roll_tip("left", cup, at + fit, g, m.CUP_H, m.CUP_TOP_D / 2, well, CUP_CLEAR, at + fit + [0, 0, LIFT], f"L cup {s}", 4.0,
                                                   pre_leans=CUP_PRE_LEANS, over=CUP_OVER, leans=CUP_LEANS, foot_r=m.CUP_FOOT_D / 2,
                                                   aim_k=CUP_AIM, avoid=(float(bp[0]), float(bp[1]), float(bp[2]), float(well[2]), m.BASE_D / 2))
        moves += carrying + tipping + self.cart("left", None, None, 1.5, f"L pours cup {s}") + back
        home = at + np.array([*CUP_SET[cup], 0.0])
        moves += self.transfer("left", home + fit + [0, 0, LIFT], g, 2.0, f"L returns cup {s}")
        moves += self.cart("left", home + fit + [0, 0, 0.002], None, 1.2, f"L sets cup {s}")
        moves += self.cart("left", None, None, 0.6, f"L lets cup {s} go", grip=1.0)
        moves += self.cart("left", home + fit - APPROACH * a + [0, 0, 0.03], None, 1.0, f"L out of cup {s}")
        moves += self.cart("left", home + fit + [0, 0, HIGH], None, 1.0, f"L up from cup {s}")
        pos, rot = self.ready_pose("left")  # parked, high over the row, while the right seats the top: from cup 1 straight to the
        moves += self.transfer("left", pos, rot, 2.4, "L parks")  # hold crossed the card (dry: 0.6 mm in), straight to cup 2 the pinch's turn swept a finger into cup 3 (3.9 mm)
        if not self.live:
            self.dry[cup] = (p + np.array([*CUP_SET[cup], 0.0]), self.dry[cup][1])
        return {"left": moves, "right": []}

    def tip_a_over(self) -> dict:
        return self.tip_over("cup_a")

    def pinch_a(self) -> dict:
        return self.pinch("cup_a")

    def tip_a(self) -> dict:
        return self.tip_in("cup_a")

    def tip_b_over(self) -> dict:
        return self.tip_over("cup_b")

    def pinch_b(self) -> dict:
        return self.pinch("cup_b")

    def tip_b(self) -> dict:
        return self.tip_in("cup_b")

    def ready_pose(self, side: str):
        from yam_arena import ik, station
        from yam_arena.constants import READY

        return ik.tcp_pose(np.array(READY, dtype=float), station.BASES[side])

    def park_left(self) -> dict:
        pos, rot = self.ready_pose("left")
        return {"left": self.transfer("left", pos, rot, 2.4, "L parks"), "right": []}

    def cap_mid(self, top_pose) -> np.ndarray:
        m = self.m
        p, r = top_pose
        return p + r @ np.array([0.0, 0.0, m.PLUNGER_H + m.CAP_H / 2 + CAP_OVER])

    def top_over(self) -> dict:
        self.aligned, self.still = {}, {}  # a new cycle: the seat's counts start again
        at = self.cap_mid(self.pose("top"))
        moves = self.transfer("right", at + [0, 0, 0.05], down(STROKE[0]), 2.2, "R over top", grip=1.0)
        moves += self.cart("right", at + [0, 0, 0.02], None, 1.0, "R down to cap")
        moves += self.cart("right", None, None, 0.6, "R settles")
        return {"right": moves, "left": []}

    def take_top(self) -> dict:
        m = self.m
        at = self.cap_mid(self.pose("top"))
        fit = self.fitted("right", at + [0, 0, 0.02]) - (at + [0, 0, 0.02])
        moves = self.cart("right", at + fit, None, 1.0, "R down on cap")
        moves += self.cart("right", None, None, 0.6, "R grips cap", grip=grip(m.CAP_D - SQUEEZE_CAP))
        moves += self.cart("right", at + fit + [0, 0, LIFT], None, 1.2, "R lifts top")
        return {"right": moves, "left": []}

    def face_now(self) -> np.ndarray:
        """Where the top's pressing face is: as read, or (dry) under the hand as it was taken."""
        if self.live:
            return self.pose("top")[0]
        m = self.m
        return self.hand("right") - np.array([0.0, 0.0, m.PLUNGER_H + m.CAP_H / 2 + CAP_OVER])

    def seat_target(self, over: float):
        """The grasp point's command and turn that bring the top's face ``over`` above the thread's start on the base's
        axis with the top upright on it: the hand's last command turned about the face by the top's tilt against the
        base (read; none in a dry run) and moved by where the face is against where it should be."""
        import demo_kit as kit

        m = self.m
        bp, br = self.pose("base")
        face_goal = bp + br @ np.array([0.0, 0.0, m.FLOOR + m.F0 + over])
        cmd, rot, face = self.hand("right"), self.hand_rot("right"), self.face_now()
        fix = np.eye(3)
        if self.live:
            a, b = self.pose("top")[1][:, 2], br[:, 2]
            tilt = math.acos(max(-1.0, min(1.0, float(a @ b))))
            if tilt > math.radians(0.5):
                fix = kit.about(np.cross(a, b) / math.sin(tilt), tilt)
        return face_goal + fix @ (cmd - face), fix @ rot

    def seat_over(self) -> dict:
        self.require_grip("right", self.m.CAP_D, "the top's cap", slack=0.0025)
        goal, rot = self.seat_target(SEAT_HIGH)
        moves = self.transfer("right", goal + [0, 0, 0.03], rot, 2.5, "R over base")
        moves += self.cart("right", goal, None, 1.2, "R lines up")
        moves += self.cart("right", None, None, 0.6, "R settles")
        return {"right": moves, "left": []}

    def align_at(self, over: float, key: str) -> dict:
        """The top lined up over the base's axis ``over`` above the thread's start and upright, from where its face is
        read (the command moved by the face's residual from where it should be); again until within ALIGN_SHARE of the
        plunger's play and 1 deg of upright, at most ALIGN_TRIES times; at the last height still off then: Stop."""
        if self.live:
            hold = self.still_hold()
            if hold:
                return hold
            m = self.m
            g = m.top_reading()
            n = self.aligned.get(key, 0) + 1
            self.aligned[key] = n
            if g["off"] <= self.aligned_within() and g["tilt"] <= math.radians(1.0) and abs(g["face"] - (m.F0 + over)) < 0.0015:
                return {}
            if n > ALIGN_TRIES[key]:
                if key == "low":
                    raise Stop(f"the top not lined up over the mouth after {ALIGN_TRIES[key]} tries: {1000 * g['off']:.2f} mm off, {math.degrees(g['tilt']):.1f} deg")
                return {}
            self.again = True
        goal, rot = self.seat_target(over)
        return {"right": self.cart("right", goal, rot, 1.0, "R aligns top") + self.cart("right", None, None, 0.6, "R settles"), "left": []}

    def still_hold(self) -> dict | None:
        """Live, before a read: while the top still moves against the base (its face more than STILL_MOVE between two
        reads STILL_HOLD apart; at most STILL_HOLDS holds), a hold with the stage planned again; then None: read now."""
        if not self.live:
            return None
        p, r = self.pose("base")
        local = r.T @ (self.pose("top")[0] - p)
        last, n = self.still.get("at"), self.still.get("n", 0)
        if (last is not None and float(np.linalg.norm(local - last)) <= STILL_MOVE) or n >= STILL_HOLDS:
            self.still = {}
            return None
        self.still = {"at": local, "n": n + 1}
        self.again = True
        return {"right": self.cart("right", None, None, STILL_HOLD, "R holds still"), "left": []}

    def aligned_within(self) -> float:
        """How near the base's axis the top must be lined up to be set in: ALIGN_SHARE of the plunger's play a side."""
        return ALIGN_SHARE * (self.m.WELL_D - self.m.PLUNGER_D) / 2

    def seat_align_high(self) -> dict:
        return self.align_at(SEAT_HIGH, "high")

    def seat_down(self) -> dict:
        """Down to 1 mm over the rim in steps (SEAT_STEPS), lined up and set upright again from the read pose at each
        (read once the top stands still)."""
        hold = self.still_hold()
        if hold:
            return hold
        k = self.aligned.get("down", 0)
        heights = (*SEAT_STEPS, SEAT_OVER)
        self.aligned["down"] = k + 1
        if k + 1 < len(heights):
            self.again = True
        goal, rot = self.seat_target(heights[k])
        return {"right": self.cart("right", goal, rot, 1.0, "R lowers top") + self.cart("right", None, None, 0.5, "R settles"), "left": []}

    def seat_align(self) -> dict:
        return self.align_at(SEAT_OVER, "low")

    def seat_in(self) -> dict:
        """The top set into the mouth from the last height: lowered until its face is SEAT_IN over the thread's start,
        and held there while the thread takes it (read every control step)."""
        m = self.m
        self.aligned["in"] = self.aligned.get("in", 0) + 1
        if self.live:
            g = m.top_reading()
            if g["off"] > m.TAKE_OFF or g["tilt"] > m.TAKE_TILT:
                raise Stop(f"the top not over the mouth: {1000 * g['off']:.2f} mm off, {math.degrees(g['tilt']):.1f} deg")
        goal, rot = self.seat_target(SEAT_IN)
        moves = self.cart("right", goal, rot, 2.0, "R sets top in")
        moves += self.cart("right", None, None, 1.2, "R holds top")
        if not self.live:
            bp, br = self.pose("base")
            self.dry["top"] = (bp + br @ np.array([0.0, 0.0, m.FLOOR + m.F0]), br)
        return {"right": moves, "left": []}

    def seat_fix(self) -> dict:
        """Live, until the thread takes the top (read once it stands still): in the mouth and not taken, the hand moved
        by the face's residual from where it should be (on the base's axis, SEAT_IN over the thread's start) and held
        again, at most FIX_TRIES times a set-in; standing on the rim (its face not RIM_IN under it), lifted back to the
        last height to be lined up again (seat_align) and set in again, at most SEAT_TRIES set-ins; then Stop."""
        m = self.m
        if not self.live or m.STATE["mode"] == "thread":
            return {}
        hold = self.still_hold()
        if hold:
            return hold
        if m.STATE["mode"] == "thread":
            return {}
        g = m.top_reading()
        said = f"{1000 * g['off']:.2f} mm off, {math.degrees(g['tilt']):.1f} deg, its face {1000 * g['face']:.2f} mm over the floor"
        if g["face"] > m.WELL_DEPTH - RIM_IN:
            if self.aligned["in"] >= SEAT_TRIES:
                raise Stop(f"the top stands on the rim after {SEAT_TRIES} set-ins: {said}")
            self.aligned["low"] = self.aligned["fix"] = 0
            self.i = max(k for k, st in enumerate(self.stages[: self.i + 1]) if st == self.seat_align) - 1  # on to this cycle's low line-up
            goal, rot = self.seat_target(SEAT_OVER)
            return {"right": self.cart("right", goal, rot, 1.0, "R lifts top") + self.cart("right", None, None, 0.6, "R settles"), "left": []}
        n = self.aligned.get("fix", 0) + 1
        self.aligned["fix"] = n
        if n > FIX_TRIES:
            raise Stop(f"the thread did not take the top after {FIX_TRIES} corrections in the mouth: {said}")
        self.again = True
        goal, rot = self.seat_target(SEAT_IN)
        return {"right": self.cart("right", goal, rot, 0.6, "R centres top") + self.cart("right", None, None, 1.2, "R holds top"), "left": []}

    def trace(self, t: float) -> None:
        """The seat and screw cases' trace, every 0.1 s from the line-up on: the right's command now less its grasp
        point; the top's face against the base (across its axis, and over the well's floor); the base in the world
        and its tilt; both grips' openings; the thread's face height."""
        m = self.m
        p, r = self.pose("base")
        local = r.T @ (self.pose("top")[0] - p)
        err = np.array(self.arms["right"].state(t)[0], dtype=float) - np.array(self.mk.gripper("right")[0], dtype=float)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))
        print("SEATTRACE " + json.dumps({"t": round(t, 2), "stage": self.current, "mode": m.STATE["mode"], "d (mm)": None if m.STATE["d"] is None else round(1000 * m.STATE["d"], 2),
                                        "err (mm)": [round(1000 * float(x), 2) for x in err],
                                        "top (mm)": [round(1000 * float(local[0]), 2), round(1000 * float(local[1]), 2), round(1000 * (float(local[2]) - m.FLOOR), 2)],
                                        "base (mm)": [round(1000 * float(x), 1) for x in p], "base tilt (deg)": round(tilt, 2),
                                        "R z (mm)": round(1000 * float(self.mk.gripper("right")[0][2]), 1),
                                        "open R, L (mm)": [round(1000 * OPEN * float(self.mk.gripper(side)[1]), 1) for side in ("right", "left")],
                                        "grains in the well, top (mm)": self.grains_in_well(),
                                        "cups, tablets (mm)": {n: [round(1000 * float(x), 1) for x in self.mk.pose(n)[0]] for n in ("cup_a", "cup_b", "tablet_a", "tablet_b", "top")},
                                        "cups' tilt (deg)": {n: round(math.degrees(math.acos(max(-1.0, min(1.0, float(self.mk.pose(n)[1][2, 2]))))), 1) for n in ("cup_a", "cup_b")},
                                        "L at (mm)": [round(1000 * float(x), 1) for x in self.mk.gripper("left")[0]],
                                        "base yaw, top yaw against R's (deg)": [round(math.degrees(math.atan2(float(r[1, 0]), float(r[0, 0]))), 2), self.top_against_hand()],
                                        "drive share R, L": self.drive_share()}), flush=True)  # fmt: skip

    def top_against_hand(self) -> float:
        """The top's turn about the base's axis less the right hand's (deg, folded into -90 to 90: the pinch is
        symmetric): constant while the cap does not slip in the pinch."""
        _, r = self.pose("base")
        top = r.T @ self.pose("top")[1][:, 0]
        hand = r.T @ np.asarray(self.mk.gripper_turn("right"), dtype=float)[:, 0]
        d = math.degrees(math.atan2(float(top[1]), float(top[0])) - math.atan2(float(hand[1]), float(hand[0])))
        return round((d + 90.0) % 180.0 - 90.0, 2)

    def drive_share(self) -> list:
        """Each arm's largest joint torque now as a share of its drive's most (the six arm joints), from the
        simulator's applied torques; None where it cannot be read."""
        from demo_kit import JOINT_EFFORT
        from yam_arena.constants import ARM_JOINTS

        out = []
        base = self.mk.MECH.environment.unwrapped
        for side in ("right", "left"):
            try:
                robot = base.scene[f"{side}_robot"]
                ids = robot.find_joints(list(ARM_JOINTS), preserve_order=True)[0]
                tau = robot.data.applied_torque
                tau = (tau.torch if hasattr(tau, "torch") else tau)[0][ids].detach().cpu().numpy().astype(float)
                out.append(round(float(np.max(np.abs(tau) / JOINT_EFFORT)), 3))
            except Exception:  # noqa: BLE001  a reading only: the run goes on without it
                out.append(None)
        return out

    def arm_reading(self, side: str) -> list:
        """[joint angles, applied joint torques] of one arm's six joints now, from the simulator."""
        from yam_arena.constants import ARM_JOINTS

        robot = self.mk.MECH.environment.unwrapped.scene[f"{side}_robot"]
        ids = robot.find_joints(list(ARM_JOINTS), preserve_order=True)[0]
        return [(x.torch if hasattr(x, "torch") else x)[0][ids].detach().cpu().numpy().astype(float) for x in (robot.data.joint_pos, robot.data.applied_torque)]

    def twists(self) -> list:
        """The twist each hand puts on the crusher about the base's axis now (N m, + anticlockwise seen from above;
        right, left), from the arms' joint torques. The right's on the cap: its wrist roll's torque, that joint's axis
        being the cap's (the arm's weight has no moment about an upright axis). The left's on the base: its joints'
        torques less those read as the right came back on the cap (the arm stands still in the hold, so its weight's
        share stays), through its Jacobian to a wrench at the hand, taken about the base's axis."""
        from yam_arena import ik, station

        bp, br = self.pose("base")
        z = br[:, 2]
        out = []
        for side in ("right", "left"):
            q, tau = self.arm_reading(side)
            p0, r0 = ik.tcp_pose(q, station.BASES[side])
            jac = np.zeros((6, 6))
            for i in range(6):
                dq = q.copy()
                dq[i] += 1e-6
                p1, r1 = ik.tcp_pose(dq, station.BASES[side])
                d = r1 @ r0.T
                jac[:, i] = np.concatenate([p1 - p0, 0.5 * np.array([d[2, 1] - d[1, 2], d[0, 2] - d[2, 0], d[1, 0] - d[0, 1]])]) / 1e-6
            if side == "right":
                out.append(float(tau[5] * (jac[3:, 5] @ z)))
            else:
                w = np.linalg.lstsq(jac.T, tau - self.ref["tau L"], rcond=None)[0]  # the hand's force and moment on the base
                out.append(float(z @ (w[3:] + np.cross(p0 - bp, w[:3]))))
        return out

    def base_yaw(self) -> float:
        _, r = self.pose("base")
        return math.degrees(math.atan2(float(r[1, 0]), float(r[0, 0])))

    def read_twist(self, t: float) -> None:
        """While the right turns the top down: every step, each hand's twist on the crusher (twists()), the press's
        angle, the drives' shares and the base's turn in the hold since the right came back on the cap; at a give (a
        tablet crushed) the readings at the press's largest angle and the largest of each are printed (GIVE)."""
        m = self.m
        n = len(m.STATE["crushed"])
        if n > self.gives:
            print("GIVE " + json.dumps({"t": round(t, 2), "crushed": sorted(m.STATE["crushed"]), **self.most}), flush=True)
            self.gives, self.most = n, {}
        if self.current != "screw" or self.ref is None:
            return
        try:
            tw, press, share = self.twists(), math.degrees(self.mk.joint("press")), self.drive_share()
        except Exception as e:  # noqa: BLE001  a reading only: the run goes on without it
            print(f"GIVE reading failed at {t:.2f} s: {e!r}", flush=True)
            return
        most = self.most
        yaw = (self.base_yaw() - self.ref["yaw"] + 180.0) % 360.0 - 180.0
        if press >= most.get("press (deg)", -1e9):
            most.update({"press (deg)": round(press, 2), "at (s)": round(t, 2), "twist R, L there (N m)": [round(x, 3) for x in tw],
                         "base turned in the hold there (deg)": round(yaw, 2), "top against R's hand there (deg)": self.top_against_hand()})  # fmt: skip
        most["twist R, L most (N m)"] = [round(max(abs(x), y), 3) for x, y in zip(tw, most.get("twist R, L most (N m)", [0.0, 0.0]))]
        most["drive share R, L most"] = [None if x is None else round(max(x, y or 0.0), 3) for x, y in zip(share, most.get("drive share R, L most", [0.0, 0.0]))]
        most["base turned in the hold most (deg)"] = round(max(abs(yaw), most.get("base turned in the hold most (deg)", 0.0)), 2)

    def pour_trace(self, t: float) -> None:
        """The full and pour cases' trace from the base's pinch on, every 0.1 s: the base (in the world, its tilt), its
        well's rim's lowest point against cup 4's rim (across, from its axis, in its frame; up, over its rim), cup 4, the
        right's opening, and where the paracetamol grains are (the well, the applesauce, cup 4 off it, the bench, the
        air), with the middle of those out of the well (mm, from cup 4's axis, in its frame, over the bench)."""
        m = self.m
        bp, br = self.pose("base")
        sp, sr = self.pose("sauce")
        axis = br[:, 2]
        down = np.array([0.0, 0.0, -1.0]) - float(-axis[2]) * axis
        mouth = bp + br @ np.array([0.0, 0.0, m.BASE_H])
        lowest = mouth + m.WELL_D / 2 * down / max(float(np.linalg.norm(down)), 1e-9)
        rim4 = sp + sr @ np.array([0.0, 0.0, m.SAUCE_H])
        off = sr.T @ (lowest - rim4)
        where = {"well": 0, "applesauce": 0, "cup 4": 0, "bench": 0, "air": 0}
        out, free = [], []
        for tab in (x for x in m.TABLETS if x != "tablet_er" and x in m.STATE["crushed"]):
            for g in m.GRAINS[tab]:
                q = np.asarray(self.mk.pose(g)[0], dtype=float)
                if m.in_well(g):
                    where["well"] += 1
                    continue
                local = sr.T @ (q - sp)
                out.append([float(local[0]), float(local[1]), float(q[2] - m.TOP)])
                if g not in m.STATE["stuck"] and q[2] - m.TOP > 0.0015:  # in the air or on cup 4, not held: its path
                    free.append([g[-3:], *(round(1000 * float(v), 1) for v in local), round(float(self.mk.contact_force("sauce", g)), 4)])
                if m.in_sauce(q):
                    where["applesauce"] += 1
                elif m.in_cone(local, m.SAUCE_FOOT_D, m.SAUCE_TOP_D, m.SAUCE_H, -0.002, m.SAUCE_H + 0.003):
                    where["cup 4"] += 1
                elif q[2] - m.TOP < 0.003:
                    where["bench"] += 1
                else:
                    where["air"] += 1
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(br[2, 2])))))
        print("POURTRACE " + json.dumps({"t": round(t, 2), "stage": self.current, "base (mm)": [round(1000 * float(x), 1) for x in bp], "base tilt (deg)": round(tilt, 1),
                                         "rim's lowest from cup 4 (mm)": [round(1000 * float(x), 1) for x in off],
                                         "cup 4 (mm)": [round(1000 * float(x), 1) for x in sp], "cup 4 tilt (deg)": round(math.degrees(math.acos(max(-1.0, min(1.0, float(sr[2, 2]))))), 1),
                                         "open R (mm)": round(1000 * OPEN * float(self.mk.gripper("right")[1]), 1), "grains": where, "held on the applesauce": len(m.STATE["stuck"]),
                                         "out of the well, middle (mm)": [round(1000 * float(v), 1) for v in np.mean(out, axis=0)] if out else None,
                                         "free grains (cup 4's frame, mm; force N)": free}), flush=True)  # fmt: skip

    def grain_census(self) -> None:
        """Once, 2 s after the last move: where the paracetamol grains are (on the applesauce, in cup 4 off it, in the
        crusher, elsewhere), and where the ones outside lie from cup 4's axis and over the bench."""
        m = self.m
        self.census = True
        sp, sr = self.mk.pose("sauce")
        out = {"applesauce": 0, "cup 4, off the applesauce": 0, "crusher": 0, "elsewhere": 0}
        stray = []
        for t in (t for t in m.TABLETS if t != "tablet_er" and t in m.STATE["crushed"]):
            for g in m.GRAINS[t]:
                q = np.asarray(self.mk.pose(g)[0], dtype=float)
                local = sr.T @ (q - sp)
                if m.in_sauce(q):
                    out["applesauce"] += 1
                elif m.in_cone(local, m.SAUCE_FOOT_D, m.SAUCE_TOP_D, m.SAUCE_H, -0.002, m.SAUCE_H + 0.003):
                    out["cup 4, off the applesauce"] += 1
                    stray.append(("in cup 4", round(1000 * float(local[2]), 1)))
                elif m.in_well(g):
                    out["crusher"] += 1
                else:
                    out["elsewhere"] += 1
                    stray.append((round(1000 * float(math.hypot(local[0], local[1])), 1), round(1000 * float(local[0]), 1), round(1000 * float(local[1]), 1), round(1000 * float(q[2] - m.TOP), 1)))
        print("GRAINS " + json.dumps({"where": out, "off the applesauce (in cup 4: mm over its foot; else: mm from cup 4's axis, x, y in its frame, mm over the bench)": stray}), flush=True)

    def grains_in_well(self) -> list:
        """How many grains lie in the well, and the highest one's top over its floor (mm)."""
        m = self.m
        tops = [float(m.in_base(self.mk.pose(g)[0])[2]) - m.FLOOR + m.GRAIN_R for t in m.STATE["crushed"] for g in m.GRAINS[t] if m.in_well(g)]
        return [len(tops), round(1000 * max(tops), 2) if tops else None]

    def seat_report(self) -> dict:
        m = self.m
        g = m.top_reading()
        print(f"SEAT plunger {1000 * m.PLUNGER_D:.1f} mm (play {1000 * m.TAKE_OFF:.2f} mm a side): {'TAKEN' if m.STATE['mode'] == 'thread' else 'NOT TAKEN'}; "
              f"lined up {self.aligned}; now {1000 * g['off']:.2f} mm off, {math.degrees(g['tilt']):.2f} deg, face {1000 * g['face']:.2f} mm; "
              f"take corrections (t, mm off, deg, mm over the start) {m.STATE['corrections']}", flush=True)  # fmt: skip
        return {}

    def step_away(self) -> dict:
        """Once the thread has the top, the right lets go of it slowly and moves up and aside while the left comes to
        hold the base (runs w12s19 to w12s22: the left's arm, coming over the base, struck the right hand still on the
        cap and knocked the crusher 22 mm aside)."""
        m = self.m
        if self.live and m.STATE["mode"] != "thread":
            raise Stop(f"the thread did not take the top: {m.top_reading()}")
        at = np.array(self.qpos["right"], dtype=float)
        moves = self.cart("right", None, None, RELEASE_S, "R eases off", grip=grip(m.CAP_D + EASE))
        moves += self.cart("right", None, None, 0.3, "R opens", grip=grip(m.CAP_D + CAP_OPEN))
        moves += self.cart("right", at + [0, 0, AWAY[2]], None, 1.0, "R up off the cap")
        moves += self.cart("right", at + AWAY, None, 1.0, "R aside")
        return {"right": moves, "left": []}

    def back_on_cap(self) -> dict:
        """The right back on the cap where the thread holds it, closed on it slowly, the left holding the base."""
        m = self.m
        at = self.cap_on_axis() if self.live else self.cap_mid(self.pose("top"))  # dry: the top as seat_in set it
        moves = self.cart("right", at + [0, 0, AWAY[2]], down(STROKE[0]), 1.2, "R over the cap")
        moves += self.cart("right", at, None, 1.0, "R down to the cap")
        moves += self.cart("right", None, None, 0.3, "R closes to the cap", grip=grip(m.CAP_D + EASE))
        moves += self.cart("right", None, None, CLOSE_S, "R grips cap", grip=grip(m.CAP_D - SQUEEZE_CAP))
        return {"right": moves, "left": [self.Move(sum(mv.seconds for mv in moves), None, None, None, "L holds base")]}

    def hold_over(self) -> dict:
        m = self.m
        if self.live and m.STATE["mode"] != "thread":  # the left holds the base once the top is on its thread
            raise Stop(f"the thread did not take the top: {m.top_reading()}")
        cap_low = m.FLOOR + m.D_BOTTOM + m.PLUNGER_H  # the cap's lowest edge over the base's foot with the top fully down
        if not 0.020 <= HOLD_AT <= cap_low - 0.005:
            raise Stop(f"the hold {1000 * HOLD_AT:.1f} mm up is not 20 mm or more up and 5 mm or more under the cap's lowest edge ({1000 * cap_low:.1f} mm)")
        bp, _ = self.pose("base")
        a = HOLD[:, 2]
        at = bp + np.array([0.0, 0.0, HOLD_AT]) - HOLD_SHORT * a
        moves = self.transfer("left", at - APPROACH * a + [0, 0, HIGH], HOLD, 2.4, "L over base", grip=1.0)
        moves += self.cart("left", at - APPROACH * a, None, 1.0, "L down to base")
        moves += self.cart("left", None, None, 0.6, "L settles")
        return {"left": moves, "right": []}

    def hold_base(self) -> dict:
        m = self.m
        bp, _ = self.pose("base")
        a = HOLD[:, 2]
        at = bp + np.array([0.0, 0.0, HOLD_AT]) - HOLD_SHORT * a
        fit = self.fitted("left", at - APPROACH * a) - (at - APPROACH * a)
        moves = self.cart("left", at + fit, None, 1.2, "L in on base")
        chord = 2 * math.sqrt((m.BASE_D / 2) ** 2 - HOLD_SHORT**2)
        moves += self.cart("left", None, None, HOLD_CLOSE, "L holds base", grip=grip(chord - SQUEEZE_HOLD))
        return {"left": moves, "right": []}

    def cap_target(self) -> np.ndarray:
        """The grasp point on the cap where it stands now (read; or the hand's command in a dry run)."""
        if self.live:
            return self.cap_mid(self.pose("top"))
        return self.hand("right")

    def cap_on_axis(self) -> np.ndarray:
        """Live, on the thread: the grasp point on the cap with the top where the thread holds it (square on the base's
        axis, its face d over the floor), not where the closed hand has pulled it; else as read."""
        m = self.m
        if self.live and m.STATE["mode"] == "thread" and m.STATE["d"] is not None:
            bp, br = self.pose("base")
            return self.cap_mid((bp + br @ np.array([0.0, 0.0, m.FLOOR + m.STATE["d"]]), br))
        return self.cap_target()

    def half_turn(self, turning, label: str, drop: float) -> list:
        """One half-turn of the closed hand through ``turning`` (three yaws, two quarter turns); its grasp point follows
        the cap ``drop`` down (up if negative)."""
        at = self.cap_target()
        mid, end = at - [0, 0, drop / 2], at - [0, 0, drop]
        moves = self.cart("right", mid, down(turning[1]), 0.8, f"R {label}")
        moves += self.cart("right", end, down(turning[2]), 0.8, f"R {label}")
        moves += self.cart("right", None, None, 0.4, "R settles")
        return moves

    def regrip(self, turning) -> list:
        """After a half-turn: the hand lets go slowly (eased off to the cap's width over RELEASE_S), opens, lifts off,
        turns back open to ``turning[0]``, comes down on the cap where the thread holds it and closes slowly."""
        m = self.m
        at = self.cap_on_axis()
        moves = self.cart("right", None, None, RELEASE_S, "R eases off", grip=grip(m.CAP_D + EASE))
        moves += self.cart("right", None, None, 0.3, "R opens", grip=grip(m.CAP_D + CAP_OPEN))
        up = np.array(self.qpos["right"], dtype=float) + [0, 0, TURN_LIFT]
        moves += self.cart("right", up, None, 0.3, "R lifts off")
        moves += self.cart("right", up, down(turning[1]), 0.6, "R turns back")
        moves += self.cart("right", at + [0, 0, TURN_LIFT], down(turning[0]), 0.6, "R turns back")
        moves += self.cart("right", at, None, 0.3, "R lowers")
        moves += self.cart("right", None, None, 0.3, "R closes to the cap", grip=grip(m.CAP_D + EASE))
        moves += self.cart("right", None, None, CLOSE_S, "R grips cap", grip=grip(m.CAP_D - SQUEEZE_CAP))
        return moves

    def turning(self, turning, label: str, drop: float, done: bool, finished: str) -> dict:
        """A turning stage: half-turns of the closed hand, each but the last followed by a regrip; ``done`` (read after
        each half-turn) ends it with the hand still closed on the cap. ``finished`` is the Stop's text after
        MAX_STROKES half-turns, or empty to end there."""
        if self.turned:
            self.turned = False
            if done:
                self.strokes = 0
                return {}
            moves = self.regrip(turning)
        else:
            if self.strokes >= MAX_STROKES:
                self.strokes = 0
                if finished:
                    raise Stop(finished)
                return {}
            self.strokes += 1
            self.turned = True
            moves = self.half_turn(turning, label, drop)
        self.again = True
        return {"right": moves, "left": [self.Move(sum(mv.seconds for mv in moves), None, None, None, "L holds base")]}

    def screw(self) -> dict:
        """The top turned down until the tablet in the well has given (or it is at the thread's end); the hand stays
        closed on the cap after the last half-turn, at the far end of its stroke, where the unscrewing starts."""
        m = self.m
        self.require_grip("left", m.BASE_D, "the base", slack=0.0025)
        if self.live and self.strokes == 0 and not self.turned and m.STATE["mode"] != "thread":
            raise Stop(f"the thread has not taken the top: {m.top_reading()}")
        if self.live:
            left = [t for t in m.under_face() if t != "tablet_er"]
            done = m.STATE["mode"] == "thread" and ((not left and bool(m.STATE["crushed"])) or (m.STATE["d"] is not None and m.STATE["d"] <= m.D_BOTTOM + END_TOL))
        else:
            done = self.strokes >= DRY_DOWN
        drop = m.LEAD / 2
        if self.live and m.STATE["mode"] == "thread" and m.STATE["d"] is not None:
            drop = min(drop, max(0.0, m.STATE["d"] - m.D_BOTTOM))  # the grasp point follows the cap no further down than the thread goes
        return self.turning(STROKE, "turns down", drop, done, "")

    def unscrew(self) -> dict:
        """The top turned back up until it sits on the thread's start; the hand stays closed on it for the lift."""
        m = self.m
        done = (bool(m.STATE.get("loose")) or m.STATE["mode"] == "free") if self.live else self.strokes >= DRY_UP
        return self.turning(STROKE[::-1], "turns up", -m.LEAD / 2, done, "the top did not come back to the thread's start")

    def top_down(self) -> dict:
        m = self.m
        self.require_grip("right", m.CAP_D, "the top's cap", slack=0.0025)
        at = self.hand("right")
        home = self.cap_mid((np.array([*m.TOP_XY, m.TOP + m.LIFT]), np.eye(3))) + [0, 0, 0.001]
        if not self.live:
            self.dry["top"] = (np.array([*m.TOP_XY, m.TOP + m.LIFT]), np.eye(3))
        moves = self.cart("right", at + [0, 0, 0.04], None, 1.2, "R lifts top off")
        moves += self.transfer("right", home + [0, 0, 0.04], None, 2.2, "R carries top")
        moves += self.cart("right", home, None, 1.2, "R sets top down")
        moves += self.cart("right", None, None, 0.6, "R lets top go", grip=1.0)
        moves += self.cart("right", home + [0, 0, 0.06], None, 1.0, "R up")
        return {"right": moves, "left": []}

    def release_base(self) -> dict:
        at = self.hand("left")
        a = HOLD[:, 2]
        moves = self.cart("left", None, None, 0.6, "L lets base go", grip=1.0)
        moves += self.cart("left", at - APPROACH * a, None, 1.0, "L out")
        moves += self.cart("left", at - APPROACH * a + [0, 0, HIGH], None, 1.0, "L up")
        pos, rot = self.ready_pose("left")
        moves += self.transfer("left", pos, rot, 2.4, "L parks")
        pos, rot = self.ready_pose("right")  # out of the left's way while it tips the next cup (dry: by the top's place, 17 mm into the left's arm)
        return {"left": moves, "right": self.transfer("right", pos, rot, 2.4, "R parks")}

    def pour_target(self, at):
        """(the grasp point's place for the pour with the base taken at ``at``, cup 4's rim's middle, the mouth's edge)."""
        m = self.m
        sp, sr = self.pose("sauce")
        rim = sp + sr @ np.array([0.0, 0.0, m.SAUCE_H])
        lip = m.WELL_D / 2
        return self.pour_point("base", at, BASE_GRASP, BASE_POUR, m.BASE_H, lip, rim, float(rim[2])), rim, lip

    def base_over(self) -> dict:
        bp, _ = self.pose("base")
        a = BASE_GRASP[:, 2]
        at = bp + np.array([0.0, 0.0, POUR_AT]) - SHORT * a
        start = at - APPROACH * a + [0, 0, HIGH]
        # over the base with the joints on the branch of the pour's: from the branch with the most margin the carry over
        # cup 4 swung the full base to 141 deg (run w12s19; 151 deg in the plan)
        q_pour = best_ik("right", self.pour_target(at)[0], BASE_POUR)
        moves = self.transfer("right", start, BASE_GRASP, 2.4, "R over base", grip=1.0, q1=ik_near("right", start, BASE_GRASP, q_pour))
        moves += self.cart("right", at - APPROACH * a, None, 1.0, "R down to base")
        moves += self.cart("right", None, None, 0.6, "R settles")
        return {"right": moves, "left": []}

    def take_base(self) -> dict:
        m = self.m
        bp, _ = self.pose("base")
        a = BASE_GRASP[:, 2]
        at = bp + np.array([0.0, 0.0, POUR_AT]) - SHORT * a
        fit = self.fitted("right", at - APPROACH * a) - (at - APPROACH * a)
        self.base_at = at + fit
        moves = self.cart("right", at + fit, None, 1.2, "R in on base")
        chord = 2 * math.sqrt((m.BASE_D / 2) ** 2 - SHORT**2)
        moves += self.cart("right", None, None, 0.4, "R closes to the base", grip=grip(m.BASE_D + EASE))
        moves += self.cart("right", None, None, CLOSE_S, "R grips base", grip=grip(chord - SQUEEZE_BASE))  # slowly: closed in 0.6 s, it jolted the full base and a grain jumped out (run w12s21)
        return {"right": moves, "left": []}

    def roll_tip(self, side: str, name: str, grasp_at, grasp_rot, height: float, rim_r: float, target, clear: float, lifted, label: str, carry_s: float,
                 pre_leans=PRE_LEANS, over: float = CARRY_OVER, leans=TIP_LEANS, low_z: float | None = None, foot_r: float = 0.0,
                 aim_k: float = 0.5, avoid=None, back: float = 0.0, lean_down: float = 0.0):
        """The part ``name`` held as it was taken (``grasp_rot`` at ``grasp_at``) carried from ``lifted`` and tipped by
        the wrist's roll alone about its pinch's approach, in steps (TIP_LEANS), the hand moved at each so that the
        mouth's lowest point stays over ``target`` (``clear`` over it) from the first lean on; carried ``over`` its
        first pour pose leaning as ``pre_leans``, it leans there to the first lean and comes down, or with
        ``lean_down`` (s) leans to it on its way down, over that time (run w12s19: a level
        pouring hand needed the wrist turned the long way, and the move there swung the full base to 151 deg; w12s25:
        the cups' tablets fell out on such a carry). The roll the arm reaches, either way. Returns (carrying, tipping,
        back, carry joints)."""
        saved = (dict(self.qp), dict(self.qpos), dict(self.qrot), len(self.log))
        why = None
        for sign in (1.0, -1.0):
            try:
                return self._roll_tip(side, name, grasp_at, grasp_rot, height, rim_r, target, clear, lifted, label, carry_s, sign, pre_leans, over, leans, low_z, foot_r,
                                      aim_k, avoid, back, lean_down)
            except Stop as stop:
                why = stop
                self.qp, self.qpos, self.qrot = dict(saved[0]), dict(saved[1]), dict(saved[2])
                del self.log[saved[3]:]
        raise why

    def _roll_tip(self, side, name, grasp_at, grasp_rot, height, rim_r, target, clear, lifted, label, carry_s, sign, pre_leans, over, leans, low_z, foot_r,
                  aim_k, avoid, back, lean_down):
        if lean_down > 0.0 and not pre_leans:
            raise ValueError("lean_down needs the carry's lean (pre_leans)")
        rel = grasp_rot.T @ self.pose(name)[1]
        steps = []
        for lean in leans:
            rot = roll(grasp_rot, sign * lean_roll(lean))
            steps.append((self.pour_point(name, grasp_at, grasp_rot, rot, height, rim_r, target, float(target[2]), clear=clear, aim_k=aim_k, back=back), rot))
        high = steps[0][0] + np.array([0.0, 0.0, over])
        pre = [roll(grasp_rot, sign * lean_roll(lean)) for lean in pre_leans] + ([] if lean_down > 0.0 else [steps[0][1]])
        carry_q = ik_near(side, high, pre[0], self.qp[side])
        most = self.path_tilt(side, carry_q, rel)
        if most > (pre_leans[0] if pre_leans else leans[0]) + 3.0:
            raise Stop(f"{name} would lean {most:.0f} deg on its way over its pour")
        if low_z is not None:
            lowest = self.path_low(side, carry_q, name, grasp_at, grasp_rot, height, rim_r, foot_r)
            if lowest < low_z:
                raise Stop(f"{name} would come {1000 * (low_z - lowest):.0f} mm too low on its way over its pour")
        carrying = self.transfer(side, high, pre[0], carry_s, f"{label} carries", step_deg=3.0, q1=carry_q)
        for rot in pre[1:]:
            carrying += self.cart(side, high, rot, TIP_STEP_S, f"{label} leans")
        if lean_down > 0.0:
            carrying += self.cart(side, steps[0][0], steps[0][1], lean_down, f"{label} comes down leaning")
        elif over > 0.0:
            carrying += self.cart(side, steps[0][0], None, 1.0, f"{label} comes down")
        tipping = []
        for pos, rot in steps[1:]:
            tipping += self.cart(side, pos, rot, TIP_STEP_S, f"{label} tips")
        if avoid is not None:
            gap = self.part_gap(name, grasp_at, grasp_rot, carrying + tipping, height, rim_r, foot_r, avoid, start=(lifted, grasp_rot))
            self.log.append({"clear of the part it pours into (mm)": round(1000 * gap, 1), "part": name})
            if gap < PART_GAP:
                raise Stop(f"{name} would come {1000 * gap:.1f} mm from what it pours into (at least {1000 * PART_GAP:.0f} mm)")
        self.log.append({"carry": self.spill(name, grasp_at, grasp_rot, carrying, height, rim_r, target, start=(lifted, grasp_rot), past=leans[0] - 5.0)})
        self.log.append({"tip": self.spill(name, grasp_at, grasp_rot, tipping, height, rim_r, target, start=steps[0], past=leans[0])})
        back = self.cart(side, steps[-1][0] + [0.0, 0.0, BACK_UP], None, 0.8, f"{label} up from its pour")
        for pos, rot in steps[-2::-1]:
            back += self.cart(side, pos + [0.0, 0.0, BACK_UP], rot, 0.3, f"{label} tips back")
        return carrying, tipping, back, carry_q

    def pour(self) -> dict:
        m = self.m
        self.require_grip("right", m.BASE_D, "the base", slack=0.0025)
        at = self.base_at
        _, rim, lip = self.pour_target(at)
        moves = self.cart("right", at + [0, 0, LIFT], None, 3.0, "R lifts base")
        sp, _ = self.pose("sauce")
        carrying, tipping, back, carry_q = self.roll_tip("right", "base", at, BASE_GRASP, m.BASE_H, lip, rim, BASE_CLEAR, at + [0, 0, LIFT], "R base", 4.0,
                                                         low_z=float(rim[2]) + CARRY_CLEAR, foot_r=m.BASE_D / 2,
                                                         avoid=(float(sp[0]), float(sp[1]), float(sp[2]), float(rim[2]), m.SAUCE_TOP_D / 2), back=BASE_BACK,
                                                         pre_leans=BASE_PRE_LEANS, leans=BASE_LEANS, lean_down=BASE_LEAN_DOWN)
        moves += carrying + tipping + self.cart("right", None, None, 2.5, "R pours") + back
        moves += self.transfer("right", at + [0, 0, LIFT], BASE_GRASP, 2.2, "R returns base", q1=ik_near("right", at + [0, 0, LIFT], BASE_GRASP, carry_q))
        moves += self.cart("right", at + [0, 0, 0.002], None, 1.2, "R sets base down")
        moves += self.cart("right", None, None, 0.6, "R lets base go", grip=1.0)
        moves += self.cart("right", at - APPROACH * BASE_GRASP[:, 2], None, 1.0, "R out")
        moves += self.cart("right", at - APPROACH * BASE_GRASP[:, 2] + [0, 0, HIGH], None, 1.0, "R up")
        return {"right": moves, "left": []}

    def away(self) -> dict:
        if self.live and not self.census:
            self.grain_census()
        out = {}
        for side in ("left", "right"):
            pos, rot = self.ready_pose(side)
            out[side] = self.cart(side, None, None, 0.6, f"{side[0].upper()} open", grip=1.0) + self.transfer(side, pos, rot, 2.4, f"{side[0].upper()} ready")
        return out

    def start_case(self) -> None:
        """A shorter trial for trying the later stages: "screw" puts the two plain tablets in the well (written, side by
        side) and starts at the left's hold on the base; "pour" also lets them give at once (the rule's swap) and starts at taking
        the base. Not the reference run (that is "full")."""
        m, mk = self.m, self.mk
        names = [st.__name__ for st in self.stages]
        if self.live and self.case != "tips":
            for cup, (dx, dy) in CUP_SET.items():  # the emptied cups where the full run sets them down
                q, s = mk.pose(cup)
                mk.write_pose(cup, q + np.array([dx, dy, 0.0]), mk.matrix_to_quat(s))
            p, r = mk.pose("base")
            for t, y in (("tablet_a", 0.0085), ("tablet_b", -0.0085))[: 2 if self.case == "pour" else 1]:  # screw: tablet 1 in the well, 2 in its cup
                mk.write_pose(t, p + r @ np.array([0.0, y, m.FLOOR + m.PLAIN_T / 2 + 0.0003]), mk.matrix_to_quat(r))
            if self.case == "pour":
                for t in ("tablet_a", "tablet_b"):
                    m.crush(t)
        self.i = names.index("top_over" if self.case in ("screw", "seat") else "base_over")
        if self.case == "tips":  # the cups' tips alone, then away
            self.stages = [self.tip_a_over, self.pinch_a, self.tip_a, self.tip_b_over, self.pinch_b, self.tip_b, self.away]
            self.i = 0
        if self.case == "seat":  # the seat alone: from taking the top to the thread's take, then away
            self.stages = self.stages[: names.index("seat_fix") + 1] + [self.seat_report, self.away]
        self.set_up = True
        print(f"CASE {self.case}: starts at {names[self.i]}", flush=True)

    # --- the run ---
    def show(self, t: float, name: str) -> None:
        if not self.live:
            return
        m = self.m
        mm = lambda v: [round(1000 * float(x), 1) for x in v]  # noqa: E731
        hands = {side: {"at": mm(self.mk.gripper(side)[0]), "asked": mm(self.hand(side)), "open (mm)": round(1000 * OPEN * float(self.mk.gripper(side)[1]), 1)} for side in ("left", "right")}
        g = m.top_reading()
        print("STAGE " + json.dumps({"t": round(t, 2), "stage": name, "mode": m.STATE["mode"], "d (mm)": None if m.STATE["d"] is None else round(1000 * m.STATE["d"], 2),
                                     "press (deg)": round(math.degrees(self.mk.joint("press")), 1), "top": {"off (mm)": round(1000 * g["off"], 2), "tilt (deg)": round(math.degrees(g["tilt"]), 2),
                                     "face (mm)": round(1000 * g["face"], 2)}, "crushed": m.STATE["crushed"], "where": {t_: m.where(t_) for t_ in m.TABLETS},
                                     "base": mm(self.pose("base")[0]), "hands": hands}, default=str), flush=True)  # fmt: skip

    def director(self, t: float, arms, pictures) -> None:
        """Before every step: when both arms have done their moves, the next stage is planned and appended (both arms
        end each stage together). A stage may ask to be planned again (the turning strokes)."""
        self.arms = arms
        if self.live and "base" not in self.mk.MECH.poses:
            return  # before the first step the simulator has not been read: the plan starts one step later
        if not self.set_up:
            self.start_case()
            return
        if self.live and round(t / 0.05) % 2 == 0 and (self.case == "tips" or (self.case in ("seat", "screw", "full") and self.i >= self.stages.index(self.seat_align_high))):
            self.trace(t)
        if self.live and self.case in ("screw", "full"):
            self.read_twist(t)
        if self.live and (self.current == "pour" or round(t / 0.05) % 2 == 0) and self.case in ("full", "pour") and self.i >= self.stages.index(self.base_over) and m_crushed(self.m):
            self.pour_trace(t)
        if self.i >= len(self.stages) and self.live and not self.census and t + 1e-9 >= max(arm.seconds for arm in arms.values()) + 2.0:
            self.grain_census()
        if self.i >= len(self.stages) or t + 1e-9 < max(arm.seconds for arm in arms.values()):
            return
        stage = self.stages[self.i]
        if self.live and stage.__name__ == "back_on_cap":  # the left holds the base, the right not yet on the cap: twist 0
            try:
                self.ref, self.most = {"tau L": self.arm_reading("left")[1], "yaw": self.base_yaw()}, {}
            except Exception as e:  # noqa: BLE001  a reading only: the run goes on without it
                print(f"GIVE reference failed at {t:.2f} s: {e!r}", flush=True)
                self.ref = None
        self.current = stage.__name__
        self.show(t, stage.__name__)
        self.again = False
        self.begin_stage()
        try:
            moves = stage()
            self.log.append({"t": round(t, 2), "stage": stage.__name__})
        except Stop as stop:
            print(f"PLAN STOPPED at {stage.__name__}: {stop}", flush=True)
            self.log.append({"t": round(t, 2), "stopped at": stage.__name__, "why": str(stop)})
            moves, self.again, self.i = self.away(), False, len(self.stages) - 1
        for side, arm in arms.items():
            arm.moves.extend(moves.get(side, []))
        last = max(arm.seconds for arm in arms.values())
        for side, arm in arms.items():
            if arm.seconds < last:
                arm.moves.append(self.Move(last - arm.seconds, None, None, None, "waits"))
        if not self.again:
            self.i += 1


def keep_off_hulls(m) -> list:
    """Cup 3 (on the card) and the card, as hulls to keep the arms 2 cm off (their solid shapes' boxes)."""
    out = []
    for name in ("cup_er", "card"):
        body = m.mk.MECH.bodies[name]
        lows, highs = zip(*(s.bounds() for s in body.shapes if s.solid))
        out.append((name, box_hull(np.min(lows, axis=0), np.max(highs, axis=0), (np.array(body.pos, dtype=float), body.rotation()))))
    return out


def part_hulls(m, plan, names) -> list:
    """Parts as hulls (their solid shapes' boxes) where the dry plan has them (an emptied cup where it was set down)."""
    out = []
    for name in names:
        body = m.mk.MECH.bodies[name]
        shapes = [s for s in body.shapes if s.solid]
        pos, rot = plan.dry.get(name, (np.array(body.pos, dtype=float), body.rotation()))
        if name == "top":  # its cap, the face at the thread's end: the lowest the cap comes while the left holds the base
            shapes = [s for s in shapes if s.pos[2] > m.PLUNGER_H]  # the plunger is in the well, behind the base's wall
            bp, br = plan.dry["base"]
            pos, rot = bp + br @ np.array([0.0, 0.0, m.FLOOR + m.D_BOTTOM]), br
        lows, highs = zip(*(s.bounds() for s in shapes))
        out.append((name, box_hull(np.min(lows, axis=0), np.max(highs, axis=0), (np.asarray(pos, dtype=float), rot))))
    return out


def ik_by_move(moves: dict, hz: float = 20.0) -> dict:
    """Both arms' moves through the IK as a run seeds it: per move, the worst position and turn error and the least
    joint margin; only the moves where one is off (0.5 mm, 0.5 deg, 3 deg of margin)."""
    from demo_kit import Arm
    from yam_arena import ik, station

    out = {}
    for side in ("left", "right"):
        arm = Arm(side, station.BASES[side], list(moves[side]))
        n = int(round(arm.seconds * hz)) + 1
        for i in range(n):
            pos, rot, _, label, _, _ = arm.state(i / hz)
            q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), arm.q, base_pos=arm.base, iters=60)
            arm.q = q
            margin = float(np.degrees(np.minimum(q[:6] - ik.LOWER[:6], ik.UPPER[:6] - q[:6])).min())
            key = f"{side} {i / hz:6.2f} {label}"
            if pe > 0.0005 or math.degrees(re) > 0.5 or margin < 3.0:
                stem = f"{side} {label}"
                w = out.get(stem)
                if w is None or pe * 1000 > w["mm"]:
                    out[stem] = {"from": key, "mm": round(pe * 1000, 2), "deg": round(math.degrees(re), 2), "margin": round(margin, 1), "q": np.degrees(q[:6]).round(0).astype(int).tolist()}
    return out


def main() -> int:
    import demo_kit as kit
    from yam_arena import station

    quick = "--quick" in sys.argv
    record = "--norec" not in sys.argv
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    out = Path(args[0]) if args else HERE / "dry"
    if quick:
        from yam_arena import task_file

        task = task_file.load(HERE / "task.py")
        m = task.module
        plan = Plan(m, live=False, case=args[1] if len(args) > 1 else "full")  # a case (screw, pour) plans from its stage on
        plan.start_dry()
        result = kit.dry_run({"left": [], "right": []}, station.BASES, director=plan.director, limit_s=m.RUN_LIMIT_STEPS / 20.0)  # the task's limit at 20 Hz
        print(f"DRY {json.dumps(result, default=str)}", flush=True)
        moves = {side: plan.arms[side].moves for side in ("left", "right")}
        for stem, w in ik_by_move(moves).items():
            print(f"MOVE {stem}: {json.dumps(w)}", flush=True)
        if "--ik" not in sys.argv:
            print(f"ARMS APART {json.dumps(plan_apart(moves, station.BASES), default=str)}", flush=True)
            print(f"OFF THE CARD'S CUP {json.dumps(plan_clear_of(moves, station.BASES, keep_off_hulls(m)), default=str)}", flush=True)
            starts = {e["stage"]: e["t"] for e in plan.log if "stage" in e}
            if "hold_over" in starts and "release_base" in starts:  # the left's hold, while the right sets the top in and turns it: clear of the other parts
                held = (starts["hold_over"], starts.get("base_over", plan.arms["left"].seconds))
                print(f"HOLD CLEAR {json.dumps(plan_clear_of(moves, station.BASES, part_hulls(m, plan, ('cup_a', 'cup_b', 'top', 'sauce')), window=held, sides=('left',)), default=str)}", flush=True)
            if "pour" in starts:  # the right hand, the base in it, against cup 4 (its cone)
                sauce = m.mk.MECH.bodies["sauce"]
                cup4 = [("sauce", cone_hull((np.array(sauce.pos, dtype=float), sauce.rotation()), m.SAUCE_FOOT_D, m.SAUCE_TOP_D, m.SAUCE_H))]
                window = (starts["pour"], starts.get("away", plan.arms["right"].seconds))
                print(f"POUR CLEAR {json.dumps(plan_clear_of(moves, station.BASES, cup4, every=1, window=window, sides=('right',)), default=str)}", flush=True)
        print(f"STAGES {json.dumps(plan.log)}", flush=True)
        return 0
    case = args[1] if len(args) > 1 else "full"
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    if case.startswith("seat") and len(case) > 4:  # "seat33": the seat alone with a 33 mm plunger (the simulator is built at the first reset)
        import dataclasses

        m.PLUNGER_D = float(case[4:]) / 1000.0
        m.TAKE_OFF = (m.WELL_D - m.PLUNGER_D) / 2
        bodies = m.mk.MECH.bodies
        bodies["top"] = dataclasses.replace(bodies["top"], shapes=tuple(m.top_shapes()))
        case = "seat"
    plan = Plan(m, live=True, case=case)
    hz = float(embodiment.info.control_hz)

    def extra() -> dict:
        g = m.top_reading()
        return {"mode": m.STATE["mode"], "d": None if m.STATE["d"] is None else round(m.STATE["d"], 5), "press": round(math.degrees(m.mk.joint("press")), 2),
                "turn": round(m.mk.joint("turn") / (2 * math.pi), 3), "top off": round(g["off"], 5), "top tilt": round(math.degrees(g["tilt"]), 3),
                "crushed": sorted(m.STATE["crushed"]), "sauce share": round(m.sauce_share(), 3), "score": m.score(None)}  # fmt: skip

    limit = m.RUN_LIMIT_STEPS / hz
    result = kit.run(embodiment, {"left": [], "right": []}, out / case, seed=0, director=plan.director, extra=extra, limit_s=limit, tail_s=3.0, sheet_frames=16, record=record)
    print(f"STAGES {json.dumps(plan.log)}", flush=True)
    print(f"SCORE {m.score(None)} report {json.dumps(m.report(), default=str)}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
