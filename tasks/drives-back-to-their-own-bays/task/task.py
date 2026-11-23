"""Drive bays. On the bench, 27 cm in front of the arm bases on the centre line, a grey storage enclosure
(14 x 20 cm base) with three top-loading bays (slots 12.5 x 4 cm, 2.5 cm deep, running away from the arms) at
6 cm left, centre, and 6 cm right; three identical black drive sleds (10 x 2 x 7 cm, 150 g) stand in the bays
at reset. 13 cm out: a service cart with four slots of the same kind (2.5 cm deep) at 6 cm and 12 cm left and
right of the centre line.

What happens after reset, while the arms are held: the technician lifts the three sleds out one at a time
(2 s apart) and stands each in a cart slot, in an order and slot choice that change every time. From
ACT_AFTER_S the arms are free. Put each sled back into the bay it came from. Seating a sled in another bay, or
dropping one on the floor, ends the episode. Guessing succeeds one time in six.
"""

from __future__ import annotations

from itertools import permutations

import numpy as np

from rta import anim, props
from rta.plans import grasp_and_carry, wait
from rta.spec import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Put each drive back into the bay it came from; the storage array only rebuilds if every drive is in its own bay."

ENC_XY = (0.27, 0.0)
BAYS = ("B1", "B2", "B3")
BAY_Y = {"B1": +0.06, "B2": 0.0, "B3": -0.06}
SLED = (0.10, 0.02, 0.07)  # 7 cm tall: carried 1.2 cm below its top at the 16 cm hover, its bottom clears the standing sleds by 2.6 cm (9 cm sleds blocked the carry 4 cm short, a01b); pads stay above the 3 cm bay rim
SLED_MASS = 0.15
SLEDS = ("sled_a", "sled_b", "sled_c")
POCKET, WALL = (0.125, 0.040), 0.006  # 1 cm clearance per side of the 2 cm sled; 2 cm between bays, 1 cm between cart slots
BAY_DEPTH, ENC_BASE = 0.025, 0.006
BAY_FLOOR = TOP + ENC_BASE
CART_XY = (0.13, 0.0)
CART_SLOT_Y = (+0.12, +0.06, -0.06, -0.12)  # 6 cm pitch; the inner slots stay 6 cm from the centre line (an arm stalls at <= 4.5 cm this close to the bases)
CART_DEPTH, CART_BASE = 0.025, 0.006
CART_FLOOR = TOP + CART_BASE
MOVE_T0, MOVE_DT, MOVE_DUR = 1.0, 2.0, 1.4
ACT_AFTER_S = MOVE_T0 + 2 * MOVE_DT + MOVE_DUR + 0.4  # 6.8
ARRANGEMENTS = [p for p in permutations(range(4), 3)]  # cart slot for sled a, b, c: 24
GREY, BLACK, DARK = (0.50, 0.50, 0.52), (0.08, 0.08, 0.09), (0.30, 0.30, 0.32)

OBJECTS = [
    props.pocket_tray("enclosure", ENC_XY, [(0.0, BAY_Y[b]) for b in BAYS], pocket=POCKET, depth=BAY_DEPTH, wall=WALL, base=ENC_BASE, color=GREY),
    props.pocket_tray("cart", CART_XY, [(0.0, y) for y in CART_SLOT_Y], pocket=POCKET, depth=CART_DEPTH, wall=WALL, base=CART_BASE, color=DARK),
    *(Box(n, pos=(ENC_XY[0], BAY_Y[b], BAY_FLOOR + SLED[2] / 2 + 0.002), size=SLED, mass=SLED_MASS, color=BLACK) for n, b in zip(SLEDS, BAYS)),
]
HOME = dict(zip(SLEDS, BAYS))  # sled -> the bay it starts in (and must return to)


def setup(rng, trial) -> dict:
    """Hidden state: which cart slot each sled went to (one of 24 arrangements, cycling with the seed number),
    moved in a random order."""
    arr = ARRANGEMENTS[trial.balanced(len(ARRANGEMENTS), rng)]
    order = [SLEDS[i] for i in rng.permutation(3)]
    return {"slot": dict(zip(SLEDS, arr)), "order": order}


def move_keys(state, name):
    k = state["order"].index(name)
    start = (ENC_XY[0], BAY_Y[HOME[name]], BAY_FLOOR + SLED[2] / 2 + 0.002)
    end = (CART_XY[0], CART_SLOT_Y[state["slot"][name]], CART_FLOOR + SLED[2] / 2 + 0.002)
    return anim.carry(MOVE_T0 + k * MOVE_DT, start, end, lift=0.10, dur=MOVE_DUR)


def events(t: float, state: dict, world) -> None:
    for name in SLEDS:
        anim.drive(world, name, t, move_keys(state, name))


MILESTONES = tuple(f"{n}_in_own_bay" for n in SLEDS)
WEIGHTS = (1 / 3, 1 / 3, 1 / 3)
SUCCESS_HOLD_S = 1.0


def _seated(o, bay) -> bool:
    return bool(
        abs(o.pos[0] - ENC_XY[0]) < 0.02 and abs(o.pos[1] - BAY_Y[bay]) < 0.01
        and abs(o.low[2] - BAY_FLOOR) < 0.01 and props.tilt_deg(o) < 10.0
    )


def milestones(objects, state) -> list[bool]:
    return [bool(_seated(objects[n], HOME[n]) and objects[n].resting()) for n in SLEDS]


def success(objects, state) -> bool:
    return all(milestones(objects, state))


def failure(objects, state) -> str | None:
    for n in SLEDS:
        o = objects[n]
        if o.pos[2] < TOP - 0.2:
            return "sled_on_floor"
        for b in BAYS:
            if b != HOME[n] and _seated(o, b) and o.resting():
                return "sled_in_wrong_bay"
    return None


TRAVEL = 0.16
GRIP_FROM_TOP = 0.012
GRASP_FROM_BOTTOM = SLED[2] - GRIP_FROM_TOP
APPROACH = 0.02  # pads 4 cm apart around the 2 cm sled: 3 cm clear of the neighbour sled 6 cm away


def _arm_for(bay) -> str | None:
    return {"B1": "left", "B2": None, "B3": "right"}[bay]


def demo(state) -> dict:
    """One sled at a time: the left arm serves the left bay, the right arm the right bay, and the centre bay goes
    to the arm whose side the sled's cart slot is on."""
    plans = {"left": [], "right": []}
    for n in SLEDS:
        bay = HOME[n]
        slot_y = CART_SLOT_Y[state["slot"][n]]
        arm = _arm_for(bay) or ("left" if slot_y > 0 else "right")
        other = "right" if arm == "left" else "left"
        seg = grasp_and_carry(
            (CART_XY[0], slot_y, CART_FLOOR + 0.002 + GRASP_FROM_BOTTOM), (ENC_XY[0], BAY_Y[bay], BAY_FLOOR + 0.008 + GRASP_FROM_BOTTOM),
            travel_z=TOP + TRAVEL, approach_finger=APPROACH, release_finger=APPROACH,
        )
        plans[arm] += seg
        plans[other].append(wait(other, sum(w.duration for w in seg)))
    return {arm: (0.0, wps) for arm, wps in plans.items()}
