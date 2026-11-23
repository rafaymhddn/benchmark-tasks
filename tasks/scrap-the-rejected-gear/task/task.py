"""Rejected gear. On the bench: a grey parts tray (fixed) with two rows of four shallow square pockets (7 cm,
1.5 cm deep) 17 cm and 25 cm in front of the arm bases at 4 cm and 12 cm left and right of the centre line,
holding seven identical steel gears (3.4 cm across, 6 cm tall, 90 g) with one pocket empty; a red scrap bin (open
box 9 x 9 cm, 6 cm walls) 8 cm out on the centre line; a spare gear on a pad 34 cm out on the centre line beyond
the tray.

What happens after reset, while the arms are held: from 1 s the inspector lifts one gear out of its pocket,
holds it up 25 cm for 2 s, and sets it down in the empty pocket; the pocket it came from stays empty. That gear
is defective. From ACT_AFTER_S the arms are free. Move the defective gear to the scrap bin, then put the spare
gear in the pocket the defective one came from. Lifting any other gear 3 cm, or seating the spare in the other
empty pocket, ends the episode. Guessing succeeds about one time in fourteen.
"""

from __future__ import annotations

import numpy as np

from rta import anim, props
from rta.plans import grasp_and_carry, wait
from rta.spec import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "The inspector put a defective gear back into the tray. Move it to the scrap bin, then put the spare gear in the pocket it came out of."

TRAY_XY = (0.21, 0.0)
POCKETS = [(dx, dy) for dx in (-0.04, +0.04) for dy in (+0.12, +0.04, -0.04, -0.12)]  # 8, relative to TRAY_XY
POCKET, DEPTH, BASE, WALL = 0.070, 0.015, 0.006, 0.006  # 1.5 cm deep: a gear set down 1 cm off centre leans on the wall and stays under 10 deg (5 mm walls let it tilt out of the pocket, a03 trial 5)  # 1 cm clearance per side of the 5 cm gear; 1 cm walls at the 8 cm pitch
POCKET_FLOOR = TOP + BASE
GEAR_R, GEAR_H, GEAR_MASS = 0.017, 0.06, 0.09  # 3.4 cm across: a 5 cm cylinder came away 2.1 cm off the TCP from every grip (a03b-d), a 3.4 cm one (S2) stays within 1 cm; 6 cm tall so the pads grip above the 1.5 cm pocket rims
GEARS = tuple(f"gear_{k}" for k in range(7))
GEAR_IN_POCKET_Z = POCKET_FLOOR + GEAR_H / 2 + 0.002
BIN_XY, BIN_SIZE, BIN_H, BIN_WALL = (0.08, 0.0), (0.09, 0.09), 0.06, 0.006  # near centre: both arms reach it (REACH.md); its far wall ends 4 mm short of the tray
BIN_FLOOR = TOP + BIN_WALL
PAD_XY, PAD_SIZE = (0.34, 0.0), (0.07, 0.07, 0.004)  # far centre: both arms reach it; at the near centre (x 0.08, arm folded) the spare was gripped 2 cm off and landed on the pocket wall (a03b 0/7)
PAD_TOP = TOP + PAD_SIZE[2]
LIFT_T0, HOLD_UP, HOLD_S, PLACE_DUR = 1.0, 0.25, 2.0, 1.2
ACT_AFTER_S = 6.0
STEEL, GREY, RED, PAD_GREY = (0.75, 0.76, 0.78), (0.50, 0.50, 0.52), (0.85, 0.12, 0.10), (0.35, 0.35, 0.37)

OBJECTS = [
    props.pocket_tray("tray", TRAY_XY, POCKETS, pocket=(POCKET, POCKET), depth=DEPTH, wall=WALL, base=BASE, color=GREY),
    *(Cylinder(n, pos=(TRAY_XY[0] + POCKETS[k][0], TRAY_XY[1] + POCKETS[k][1], GEAR_IN_POCKET_Z), radius=GEAR_R, height=GEAR_H, mass=GEAR_MASS, color=STEEL) for k, n in enumerate(GEARS)),
    props.open_box("bin", BIN_XY, BIN_SIZE, BIN_H, wall=BIN_WALL, color=RED),
    Box("pad", pos=(*PAD_XY, TOP + PAD_SIZE[2] / 2), size=PAD_SIZE, color=PAD_GREY),
    Cylinder("spare", pos=(*PAD_XY, PAD_TOP + GEAR_H / 2), radius=GEAR_R, height=GEAR_H, mass=GEAR_MASS, color=STEEL),
]


def pocket_xy(k):
    return (TRAY_XY[0] + POCKETS[k][0], TRAY_XY[1] + POCKETS[k][1])


def setup(rng, trial) -> dict:
    """Hidden state: which pocket starts empty, which gear is lifted (the defective one) and where it came from.
    Gear k starts in pocket ``start[k]``; the lifted gear cycles with the seed number (1/7)."""
    empty = int(rng.integers(8))
    pockets = [p for p in range(8) if p != empty]
    start = {n: pockets[k] for k, n in enumerate(GEARS)}
    defective = GEARS[trial.balanced(len(GEARS), rng)]
    return {"empty0": empty, "start": start, "defective": defective, "origin": start[defective], "fail": None}


def placements(state) -> dict:
    return {n: (*pocket_xy(state["start"][n]), GEAR_IN_POCKET_Z) for n in GEARS}


def gear_keys(state):
    a = (*pocket_xy(state["origin"]), GEAR_IN_POCKET_Z)
    up = (a[0], a[1], a[2] + HOLD_UP)
    b = (*pocket_xy(state["empty0"]), GEAR_IN_POCKET_Z)
    over_b = (b[0], b[1], b[2] + HOLD_UP)
    t1 = LIFT_T0 + 0.8
    return [(LIFT_T0, a), (t1, up), (t1 + HOLD_S, up), (t1 + HOLD_S + 0.5, over_b), (t1 + HOLD_S + 0.5 + PLACE_DUR, b)]


def events(t: float, state: dict, world) -> None:
    anim.drive(world, state["defective"], t, gear_keys(state))


MILESTONES = ("defective_gear_lifted", "defective_gear_in_bin", "spare_in_origin_pocket")
WEIGHTS = (0.25, 0.25, 0.5)
SUCCESS_HOLD_S = 1.0


def _seated(o, k) -> bool:
    x, y = pocket_xy(k)
    return bool(np.hypot(o.pos[0] - x, o.pos[1] - y) < 0.02 and abs(o.low[2] - POCKET_FLOOR) < 0.01 and props.tilt_deg(o) < 10.0)


def _in_bin(o) -> bool:
    return props.inside(o, BIN_XY, (BIN_SIZE[0] / 2 - BIN_WALL, BIN_SIZE[1] / 2 - BIN_WALL), floor_z=BIN_FLOOR, above=0.03)


def milestones(objects, state) -> list[bool]:
    d, spare = objects[state["defective"]], objects["spare"]
    return [
        bool(objects.t >= ACT_AFTER_S and d.low[2] > POCKET_FLOOR + 0.05),
        bool(_in_bin(d) and d.resting()),
        bool(_seated(spare, state["origin"]) and spare.resting()),
    ]


def success(objects, state) -> bool:
    m = milestones(objects, state)
    return bool(m[1] and m[2])


def failure(objects, state) -> str | None:
    for n in GEARS:
        o = objects[n]
        if o.pos[2] < TOP - 0.2:
            return "gear_off_table"
        if n != state["defective"] and o.low[2] > POCKET_FLOOR + 0.03:
            return "good_gear_lifted"
    spare = objects["spare"]
    if spare.pos[2] < TOP - 0.2:
        return "gear_off_table"
    if objects.t >= ACT_AFTER_S and _seated(spare, state["empty0"]) and spare.resting():
        return "spare_in_wrong_pocket"
    return None


TRAVEL = 0.13  # low carry over the tray; the 6 cm gear gripped 4.8 cm up clears the 6 cm bin wall by 2 cm
GRASP_FROM_BOTTOM = GEAR_H - 0.012  # pad tips 2 cm lower stay 3 mm above the pocket rim
APPROACH_SPARE = 0.047  # fully open over the 5 cm spare: at 3.5 cm a pad landed on the gear rim when the TCP was 1 cm off, that finger stalled and the other closed fully, leaving the gear 2.2 cm off the TCP (a03c)
RELEASE_IN_BIN = 0.03  # pads open to 6 cm inside the 7.8 cm bin


def _arm_for_pocket(k) -> str:
    return "left" if POCKETS[k][1] > 0 else "right"


def demo(state) -> dict:
    """The arm on the defective gear's side bins it; then the arm on the origin pocket's side fetches the spare."""
    now_k = state["empty0"]  # the defective gear now sits in the pocket that started empty
    bin_arm = _arm_for_pocket(now_k)
    spare_arm = _arm_for_pocket(state["origin"])
    seg1 = grasp_and_carry((*pocket_xy(now_k), POCKET_FLOOR + 0.002 + GRASP_FROM_BOTTOM), (*BIN_XY, BIN_FLOOR + 0.008 + GRASP_FROM_BOTTOM), travel_z=TOP + TRAVEL, release_finger=RELEASE_IN_BIN)
    seg2 = grasp_and_carry((*PAD_XY, PAD_TOP + GRASP_FROM_BOTTOM), (*pocket_xy(state["origin"]), POCKET_FLOOR + 0.006 + GRASP_FROM_BOTTOM), travel_z=TOP + TRAVEL, approach_finger=APPROACH_SPARE)
    plans = {"left": [], "right": []}
    for arm, seg in ((bin_arm, seg1), (spare_arm, seg2)):
        other = "right" if arm == "left" else "left"
        plans[arm] += seg
        plans[other].append(wait(other, sum(w.duration for w in seg)))
    return {arm: (0.0, wps) for arm, wps in plans.items()}
