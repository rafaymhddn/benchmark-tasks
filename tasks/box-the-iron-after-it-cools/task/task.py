"""Cooling iron. On the bench: a soldering station (grey block 10 x 8 x 5 cm, fixed) 34 cm in front of the arm
bases, 24 cm left of the centre line, with a red power light (1 cm cube) on its top; the soldering iron (dark
handle 12 x 2.2 x 2.2 cm with a 5 cm metal tip pointing away from the arms, 60 g) lying on the bench 22 cm out,
12 cm left of the centre line; an open tool box (22 x 10 cm, 6 cm walls) 15 cm out on the centre line.

The arms are free from ACT_AFTER_S. The power light is on; at a moment T0 between 1.5 and 9 s (hidden,
changing every trial) the station is switched off and the light goes out. The iron is hot: touching it, or
moving it, earlier than COOL_S after the light went out is a burn that sets the score to zero. It must be lying
in the tool box no later than DEADLINE_S after the light went out. No fixed schedule passes every trial.
"""

from __future__ import annotations

import numpy as np

from rta import anim, props
from rta.plans import FINGER_OPEN, grasp_and_carry, opening, wait
from rta.spec import TABLE_TOP, Box, Compound, Part
from yam_arena.demo import Waypoint

SCENE = "station"
TOP = TABLE_TOP[SCENE]
COOL_S, DEADLINE_S = 10.0, 25.0  # the scripted solution sets the iron down ~18 s after the light and its 2 s release still nudges it at 20 s (s05c 3/6)
INSTRUCTION = (
    f"The soldering iron is switched off while you watch. Do not touch it until {COOL_S:.0f} seconds after the "
    f"light goes out, and have it in the tool box no later than {DEADLINE_S:.0f} seconds after."
)

STATION_XY, STATION_SIZE = (0.34, 0.24), (0.10, 0.08, 0.05)  # beside the iron, not in front of it: its 5 cm tip reaches x 0.33 (s05b: tip inside the block → iron jittered and was torn out of the grip)
LED, LED_ON = 0.01, (*STATION_XY, TOP + STATION_SIZE[2] + 0.005 + 0.002)
HANDLE, TIP = (0.12, 0.022, 0.022), (0.05, 0.008, 0.008)
IRON_XY, IRON_MASS = (0.22, 0.12), 0.06
IRON_REST = (*IRON_XY, TOP + HANDLE[2] / 2)
BOX_XY, BOX_SIZE, BOX_H, BOX_WALL = (0.15, 0.0), (0.22, 0.10), 0.06, 0.01
BOX_FLOOR = TOP + BOX_WALL
BOX_HALF_IN = (BOX_SIZE[0] / 2 - BOX_WALL, BOX_SIZE[1] / 2 - BOX_WALL)
IRON_IN_BOX_X = 0.13  # handle centre; the tip reaches 0.24, inside the 0.25 end wall
T0_CHOICES = (1.5, 3.0, 4.5, 6.0, 7.5, 9.0)
ACT_AFTER_S = 1.0
TOUCH_R = 0.04  # TCP this close to the iron's box counts as a touch (finger pads reach ~2 cm below the TCP)
MOVED = 0.01
GREY, RED, DARK, BLUE = (0.50, 0.50, 0.52), (0.95, 0.10, 0.10), (0.15, 0.15, 0.17), (0.20, 0.30, 0.70)

OBJECTS = [
    Box("station", pos=(*STATION_XY, TOP + STATION_SIZE[2] / 2), size=STATION_SIZE, color=GREY),
    Box("light", pos=LED_ON, size=(LED, LED, LED), mass=0.005, color=RED),
    Compound("iron", pos=IRON_REST, size=HANDLE, parts=(Part(TIP, (HANDLE[0] / 2 + TIP[0] / 2, 0.0, 0.0)),), mass=IRON_MASS, color=DARK),
    props.open_box("toolbox", BOX_XY, BOX_SIZE, BOX_H, wall=BOX_WALL, color=BLUE),
]


def setup(rng, trial) -> dict:
    """Hidden state: when the light goes out (cycles through six moments with the seed number)."""
    return {"t0": T0_CHOICES[trial.balanced(len(T0_CHOICES), rng)], "done_in_time": False, "last_t": None}


def light_keys(state):
    t0 = state["t0"]
    return [(0.0, LED_ON), (t0, LED_ON), (t0 + 0.1, anim.park(0, LED))]


def events(t: float, state: dict, world) -> None:
    anim.drive(world, "light", t, light_keys(state))


MILESTONES = ("lifted_after_cooling", "in_tool_box", "in_tool_box_in_time")
WEIGHTS = (0.33, 0.33, 0.34)
SUCCESS_HOLD_S = 1.0
ZERO_ON = {"touched_before_cool"}


def _in_box(o) -> bool:
    return props.inside(o, BOX_XY, BOX_HALF_IN, floor_z=BOX_FLOOR, above=0.02)


def _point_box_distance(p, o) -> float:
    d = np.maximum(0.0, np.maximum(o.low - p, p - o.high))
    return float(np.linalg.norm(d))


def milestones(objects, state) -> list[bool]:
    iron, t, t0 = objects["iron"], objects.t, state["t0"]
    cooled = t >= t0 + COOL_S
    lifted = bool(cooled and iron.low[2] > TOP + 0.03)
    in_box = bool(_in_box(iron))
    if in_box and iron.resting() and cooled and t <= t0 + DEADLINE_S:
        state["done_in_time"] = True
    return [lifted, in_box, state["done_in_time"]]


def success(objects, state) -> bool:
    iron = objects["iron"]
    return bool(_in_box(iron) and iron.resting() and state["done_in_time"])


def failure(objects, state) -> str | None:
    iron, t, t0 = objects["iron"], objects.t, state["t0"]
    if iron.pos[2] < TOP - 0.2:
        return "iron_off_table"
    if t < t0 + COOL_S:
        if abs(iron.low[0] - (IRON_REST[0] - HANDLE[0] / 2)) > MOVED or abs(iron.low[1] - (IRON_REST[1] - HANDLE[1] / 2)) > MOVED:
            return "touched_before_cool"
        if any(_point_box_distance(np.asarray(tcp, float), iron) < TOUCH_R for tcp in objects.tcp.values()):
            return "touched_before_cool"
    if t > t0 + DEADLINE_S and not state["done_in_time"]:
        return "too_late"
    return None


TRAVEL = 0.16
GRASP_Z = TOP + 0.018
RELEASE_Z = BOX_FLOOR + 0.018 + 0.008
RELEASE_FINGER = 0.0125  # pads open to 2.5 cm inside the 8 cm box
APPROACH = 0.02  # pads 4 cm apart over the 2.2 cm handle: a short close so the iron is in the box well inside DEADLINE_S
STAGE_UP = 0.09  # TCP height over the bench while waiting: 6.8 cm above the iron's top, outside TOUCH_R
DESCEND_S = 0.8  # the final 7 cm descent after the cooling time


def demo(state) -> dict:
    """Left arm: hover over the iron from the start, wait until COOL_S + 0.3 s after the light goes out, then take
    the iron to the tool box (resting there about 18 s after the light went out)."""
    pick = (*IRON_XY, GRASP_Z)
    stage = (IRON_XY[0], IRON_XY[1], TOP + STAGE_UP)
    hover = Waypoint(1.5, stage, opening(APPROACH), "over the iron")
    # the arm waits staged just outside the touch zone, so the carry's own hover/descend waypoints are replaced
    # by one short final descent (the waypoint time, not the distance, sets how long the descent takes)
    carry = grasp_and_carry(pick, (IRON_IN_BOX_X, BOX_XY[1], RELEASE_Z), travel_z=TOP + TRAVEL, approach_finger=APPROACH, release_finger=RELEASE_FINGER, settle_s=0.0)[2:]  # from "close"; the box is 8 cm wide, no settle needed
    descend = Waypoint(DESCEND_S, pick, opening(APPROACH), "descend")
    wait_s = state["t0"] + COOL_S + 0.3 - ACT_AFTER_S - hover.duration
    left = [hover, Waypoint(wait_s, stage, opening(APPROACH), "wait for cooling"), descend, *carry]
    return {"left": (0.0, left), "right": (0.0, [wait("right", sum(w.duration for w in left))])}
