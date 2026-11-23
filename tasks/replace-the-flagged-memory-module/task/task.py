"""Memory module. On the bench, 26 cm in front of the arm bases on the centre line, a green circuit board
(16 x 24 x 1 cm, fixed) with four guide slots along it (rails 3 cm apart, 1.5 cm high, running away from the arms)
at 2.5 cm and 7.5 cm left and right of the centre line, each holding an identical black memory module
(11 x 1 x 6 cm, 20 g) standing upright. 11 cm out: a spare module standing upright in a holder (one slot of the
same kind) 4.5 cm left of the centre line, and an open grey RMA tray (14 x 6 cm, 3 cm walls) 4.5 cm right.

What happens after reset, while the arms are held: from 1 s to 3.5 s a red indicator (1 cm cube) is shown on
the board at the far end of one slot, then taken away. That slot's module is faulty. From ACT_AFTER_S the arms
are free. Pull the faulty module, put it in the RMA tray, and seat the spare in the empty slot. Lifting a good
module 2 cm ends the episode. Guessing succeeds one time in four.
"""

from __future__ import annotations

import numpy as np

from rta import anim, props
from rta.plans import grasp_and_carry, wait
from rta.spec import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "During boot the light next to the faulty memory module came on. Pull that module, put it in the RMA tray, and seat the spare in its slot."

BOARD_XY, BOARD_SIZE = (0.26, 0.0), (0.16, 0.24, 0.01)
SLOTS = ("S1", "S2", "S3", "S4")
SLOT_Y = {"S1": +0.075, "S2": +0.025, "S3": -0.025, "S4": -0.075}
MODULE = (0.11, 0.01, 0.06)
MODULE_MASS = 0.02
RAIL_GAP, RAIL_H, RAIL_WALL = 0.030, 0.015, 0.006  # 1 cm clearance per side of the 1 cm module (the oracle lands 5-8 mm off)
SLOT_FLOOR = TOP + BOARD_SIZE[2]
MODULE_IN_SLOT_Z = SLOT_FLOOR + MODULE[2] / 2 + 0.002
HOLDER_XY, TRAY_XY = (0.11, +0.045), (0.11, -0.045)
TRAY_SIZE, TRAY_H, TRAY_WALL = (0.14, 0.06), 0.03, 0.006
TRAY_FLOOR = TOP + TRAY_WALL
HOLDER_BASE = 0.006
HOLDER_FLOOR = TOP + HOLDER_BASE
LED, LED_T0, LED_T1 = 0.01, 1.0, 3.5
LED_X = BOARD_XY[0] + MODULE[0] / 2 + 0.005 + RAIL_WALL + 0.007  # on the board, just beyond the slot's far rail
ACT_AFTER_S = 4.5
GREEN, BLACK, GREY, RED = (0.10, 0.45, 0.20), (0.08, 0.08, 0.09), (0.50, 0.50, 0.52), (0.95, 0.10, 0.10)

OBJECTS = [
    props.pocket_tray("board", BOARD_XY, [(0.0, SLOT_Y[s]) for s in SLOTS], pocket=(MODULE[0] + 0.01, RAIL_GAP), depth=RAIL_H, wall=RAIL_WALL, base=BOARD_SIZE[2], color=GREEN),
    *(Box(f"module_{s}", pos=(BOARD_XY[0], SLOT_Y[s], MODULE_IN_SLOT_Z), size=MODULE, mass=MODULE_MASS, color=BLACK) for s in SLOTS),
    props.pocket_tray("holder", HOLDER_XY, [(0.0, 0.0)], pocket=(MODULE[0] + 0.01, RAIL_GAP), depth=RAIL_H, wall=RAIL_WALL, base=HOLDER_BASE, color=GREY),
    Box("spare", pos=(*HOLDER_XY, HOLDER_FLOOR + MODULE[2] / 2 + 0.002), size=MODULE, mass=MODULE_MASS, color=BLACK),
    props.open_box("tray", TRAY_XY, TRAY_SIZE, TRAY_H, wall=TRAY_WALL, color=GREY),
    Box("indicator", pos=anim.park(0, LED), size=(LED, LED, LED), mass=0.005, color=RED),
]


def setup(rng, trial) -> dict:
    """Hidden state: the faulty slot (cycles with the seed number; chance 1/4)."""
    return {"faulty": SLOTS[trial.balanced(len(SLOTS), rng)]}


def indicator_keys(state):
    on = (LED_X, SLOT_Y[state["faulty"]], SLOT_FLOOR + LED / 2 + 0.002)
    return [(LED_T0, on), (LED_T1, on), (LED_T1 + 0.1, anim.park(0, LED))]


def events(t: float, state: dict, world) -> None:
    anim.drive(world, "indicator", t, indicator_keys(state))


MILESTONES = ("faulty_module_lifted", "faulty_module_in_tray", "spare_seated")
WEIGHTS = (0.25, 0.25, 0.5)
SUCCESS_HOLD_S = 1.0


def _seated(o, slot) -> bool:
    return bool(
        abs(o.pos[0] - BOARD_XY[0]) < 0.02 and abs(o.pos[1] - SLOT_Y[slot]) < 0.008
        and abs(o.low[2] - SLOT_FLOOR) < 0.005 and props.tilt_deg(o) < 15.0  # may lean on a rail of the 3 cm slot
    )


def _in_tray(o) -> bool:
    return props.inside(o, TRAY_XY, (TRAY_SIZE[0] / 2 - TRAY_WALL, TRAY_SIZE[1] / 2 - TRAY_WALL), floor_z=TRAY_FLOOR, above=0.03)


def milestones(objects, state) -> list[bool]:
    f = objects[f"module_{state['faulty']}"]
    spare = objects["spare"]
    return [
        bool(f.low[2] > SLOT_FLOOR + 0.05),
        bool(_in_tray(f) and f.resting()),
        bool(_seated(spare, state["faulty"]) and spare.resting()),
    ]


def success(objects, state) -> bool:
    m = milestones(objects, state)
    return bool(m[1] and m[2])


def failure(objects, state) -> str | None:
    for s in SLOTS:
        o = objects[f"module_{s}"]
        if o.pos[2] < TOP - 0.2:
            return "module_off_table"
        if s != state["faulty"] and o.low[2] > SLOT_FLOOR + 0.02:
            return "good_module_lifted"
    if objects["spare"].pos[2] < TOP - 0.2:
        return "module_off_table"
    return None


TRAVEL = 0.16
GRIP_FROM_TOP = 0.012  # TCP below the module's top: pad tips 2 cm lower still clear the 1.5 cm rails
GRASP_FROM_BOTTOM = MODULE[2] - GRIP_FROM_TOP
APPROACH = 0.015  # 3 cm finger opening: the next slot is 5 cm away


def demo(state) -> dict:
    """The arm on the faulty slot's side pulls that module into the RMA tray, then seats the spare in the slot."""
    slot = state["faulty"]
    arm = "left" if SLOT_Y[slot] > 0 else "right"
    other = "right" if arm == "left" else "left"
    pull = grasp_and_carry(
        (BOARD_XY[0], SLOT_Y[slot], SLOT_FLOOR + 0.002 + GRASP_FROM_BOTTOM), (TRAY_XY[0], TRAY_XY[1], TRAY_FLOOR + 0.01 + GRASP_FROM_BOTTOM),
        travel_z=TOP + TRAVEL, approach_finger=APPROACH,
    )
    seat = grasp_and_carry(
        (HOLDER_XY[0], HOLDER_XY[1], HOLDER_FLOOR + 0.002 + GRASP_FROM_BOTTOM), (BOARD_XY[0], SLOT_Y[slot], SLOT_FLOOR + 0.006 + GRASP_FROM_BOTTOM),
        travel_z=TOP + TRAVEL, approach_finger=APPROACH, release_finger=APPROACH,
    )
    wps = pull + seat
    return {arm: (0.0, wps), other: (0.0, [wait(other, sum(w.duration for w in wps))])}
