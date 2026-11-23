"""Slide staining. On the bench: a white slide carrier (5 x 3 x 5 cm block with a 2 x 2 x 3 cm grip tab on top,
80 g) 12 cm in front of the arm bases, 15 cm left of the centre line; a purple stain tank (open box 12 x 12 cm,
7 cm walls) 22 cm out, 12 cm left; a blue rinse tank (same) 30 cm out on the centre line; a drying rack (grey
base with one 8 x 6 cm slot between rails, 1.5 cm deep) 12 cm out, 3 cm left.

No observe phase: the arms are free from ACT_AFTER_S. Dip the carrier into the stain for 8 to 12 s, then into
the rinse for 4 to 6 s, then stand it in the rack. A dip is the carrier's centre over a tank with its bottom at
least 2 cm below the rim; its length is one continuous stay. No clock is visible: the robot keeps its own time.
A second stain dip, or a rinse before the stain, ends the episode.
"""

from __future__ import annotations

import numpy as np

from rta import props
from rta.plans import RELEASE_S, opening, FINGER_OPEN, grasp
from rta.spec import TABLE_TOP, Compound, Part
from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Stain the slide: dip the carrier into the purple stain for 8 to 12 seconds, then into the blue rinse for 4 to 6 seconds, then stand it on the drying rack."

BODY, TAB = (0.05, 0.03, 0.05), (0.02, 0.02, 0.03)
TAB_DZ = BODY[2] / 2 + TAB[2] / 2
CARRIER_START = (0.12, 0.15)
CARRIER_REST_Z = TOP + BODY[2] / 2
TANK_SIZE, TANK_H, TANK_WALL = (0.12, 0.12), 0.07, 0.01  # 10 x 10 cm inside: the closed fingers go 1.5 cm below the rim
STAIN_XY, RINSE_XY = (0.22, 0.12), (0.30, 0.0)
TANK_FLOOR, TANK_RIM = TOP + TANK_WALL, TOP + TANK_H
TANK_HALF_IN = (TANK_SIZE[0] / 2 - TANK_WALL, TANK_SIZE[1] / 2 - TANK_WALL)
DIP_DEPTH = 0.02
RACK_XY, RACK_SLOT, RACK_DEPTH, RACK_BASE = (0.12, 0.03), (0.08, 0.06), 0.015, 0.006  # 1.5 cm clearance per side; on the left arm's side
RACK_FLOOR = TOP + RACK_BASE
STAIN_OK, STAIN_GOOD = (6.0, 16.0), (8.0, 12.0)
RINSE_OK, RINSE_GOOD = (3.0, 8.0), (4.0, 6.0)
ACT_AFTER_S = 1.0
WHITE, PURPLE, BLUE, GREY = (0.95, 0.95, 0.95), (0.50, 0.15, 0.60), (0.15, 0.35, 0.90), (0.50, 0.50, 0.52)

OBJECTS = [
    Compound("carrier", pos=(*CARRIER_START, CARRIER_REST_Z), size=BODY, parts=(Part(TAB, (0.0, 0.0, TAB_DZ)),), mass=0.08, color=WHITE),
    props.open_box("stain", STAIN_XY, TANK_SIZE, TANK_H, wall=TANK_WALL, color=PURPLE),
    props.open_box("rinse", RINSE_XY, TANK_SIZE, TANK_H, wall=TANK_WALL, color=BLUE),
    props.pocket_tray("rack", RACK_XY, [(0.0, 0.0)], pocket=RACK_SLOT, depth=RACK_DEPTH, base=RACK_BASE, color=GREY),
]
TANKS = {"stain": STAIN_XY, "rinse": RINSE_XY}


def setup(rng, trial) -> dict:
    """No hidden state: the timing is the task. ``dips`` holds the completed dip lengths per tank."""
    return {"in": {"stain": None, "rinse": None}, "dips": {"stain": [], "rinse": []}, "last_t": None, "fail": None}


def events(t: float, state: dict, world) -> None:
    return None


def dipped(o, tank) -> bool:
    x, y = TANKS[tank]
    return bool(abs(o.pos[0] - x) < TANK_HALF_IN[0] and abs(o.pos[1] - y) < TANK_HALF_IN[1] and o.low[2] < TANK_RIM - DIP_DEPTH)


def _track(objects, state) -> None:
    """Update dip entry/exit bookkeeping once per control step."""
    t = objects.t
    if state["last_t"] == t:
        return
    state["last_t"] = t
    o = objects["carrier"]
    for tank in TANKS:
        now = dipped(o, tank)
        if now and state["in"][tank] is None:
            if tank == "stain" and state["dips"]["stain"]:
                state["fail"] = "second_stain_dip"
            if tank == "rinse" and not state["dips"]["stain"]:
                state["fail"] = "rinse_before_stain"
            state["in"][tank] = t
        elif not now and state["in"][tank] is not None:
            state["dips"][tank].append(t - state["in"][tank])
            state["in"][tank] = None


def _within(d, lo_hi) -> bool:
    return lo_hi[0] <= d <= lo_hi[1]


MILESTONES = ("stain_dip_6_16s", "stain_dip_8_12s", "rinse_dip_3_8s", "rinse_dip_4_6s", "on_rack")
WEIGHTS = (0.2, 0.2, 0.15, 0.15, 0.3)
SUCCESS_HOLD_S = 1.0


def on_rack(o) -> bool:
    return bool(
        np.hypot(o.pos[0] - RACK_XY[0], o.pos[1] - RACK_XY[1]) < 0.015 and abs(o.low[2] - RACK_FLOOR) < 0.01
        and props.tilt_deg(o) < 10.0 and o.resting()
    )


def milestones(objects, state) -> list[bool]:
    _track(objects, state)
    s, r = state["dips"]["stain"], state["dips"]["rinse"]
    s1, r1 = (s[0] if s else None), (r[0] if r else None)
    return [
        s1 is not None and _within(s1, STAIN_OK),
        s1 is not None and _within(s1, STAIN_GOOD),
        r1 is not None and _within(r1, RINSE_OK),
        r1 is not None and _within(r1, RINSE_GOOD),
        bool(s1 is not None and r1 is not None) and on_rack(objects["carrier"]),
    ]


def success(objects, state) -> bool:
    m = milestones(objects, state)
    return bool(m[0] and m[2] and m[4])


def failure(objects, state) -> str | None:
    _track(objects, state)
    if objects["carrier"].pos[2] < TOP - 0.2:
        return "carrier_off_table"
    return state["fail"]


TRAVEL = 0.16
GRASP_UP = BODY[2] - 0.005  # TCP 5 mm under the body's top: the pads (2 cm tall) hold the 3 cm body under the tab, never the 2 cm tab
ALIGN_S = 0.6
DIP_Z = TANK_RIM - 0.04 + GRASP_UP  # carrier bottom 4 cm below the rim (2 cm over the floor); the pads reach 1.5 cm below the rim inside the 10 cm opening
RACK_Z = RACK_FLOOR + 0.006 + GRASP_UP
STAIN_HOLD, RINSE_HOLD = 9.3, 4.3  # plus ~0.75 s of the lowering/lifting legs below the 2 cm line
RELEASE_FINGER = 0.025  # pads open to 5 cm inside the 6 cm slot


def demo(state) -> dict:
    x0, y0 = CARRIER_START
    tz = TOP + TRAVEL
    wps = [*grasp((x0, y0, TOP + GRASP_UP), travel_z=tz), Waypoint(1.2, (x0, y0, tz), FINGER_CLOSED, "lift")]
    for (x, y), hold in ((STAIN_XY, STAIN_HOLD), (RINSE_XY, RINSE_HOLD)):
        wps += [
            Waypoint(1.8, (x, y, tz), FINGER_CLOSED, "carry"),
            Waypoint(ALIGN_S, (x, y, tz), FINGER_CLOSED, "align"),
            Waypoint(1.2, (x, y, DIP_Z), FINGER_CLOSED, "lower"),
            Waypoint(hold, (x, y, DIP_Z), FINGER_CLOSED, "dip"),
            Waypoint(1.2, (x, y, tz), FINGER_CLOSED, "lift"),
        ]
    x, y = RACK_XY
    wps += [
        Waypoint(1.8, (x, y, tz), FINGER_CLOSED, "carry"),
        Waypoint(ALIGN_S, (x, y, tz), FINGER_CLOSED, "align"),
        Waypoint(1.2, (x, y, RACK_Z), FINGER_CLOSED, "lower"),
        Waypoint(RELEASE_S, (x, y, RACK_Z), opening(RELEASE_FINGER), "release"),
        Waypoint(1.0, (x, y, tz), opening(RELEASE_FINGER), "retreat"),
    ]
    from rta.plans import wait
    return {"left": (0.0, wps), "right": (0.0, [wait("right", sum(w.duration for w in wps))])}
