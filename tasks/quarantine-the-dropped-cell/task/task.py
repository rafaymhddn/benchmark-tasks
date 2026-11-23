"""Dropped cell. On the bench: four identical blue dummy battery cells (plastic cylinders 3.4 cm across, 6 cm
tall, 100 g) standing in a grey holder (square pockets 6.5 cm, 2.5 cm deep) 13 cm in front of the arm bases at
6 cm and 15 cm left and right of the centre line; a black charger 21 cm out with three pockets of the same
kind at the centre line and 7 cm left and right; a red open quarantine bin (10 x 10 cm, 6 cm walls) 30 cm out on
the centre line.

What happens after reset, while the arms are held: at 1 s a hand lifts one cell out of the holder to 30 cm above
the bench (30 cm out, 20 cm left of the centre line) and lets it fall; at 3.5 s the cell is picked up from where
it landed and stood back in its own pocket. From ACT_AFTER_S the arms are free. Put the dropped cell in the
quarantine bin and the other three in the charger. The dropped cell in the charger is a safety failure that
sets the score to zero; a good cell in the bin ends the episode. Guessing succeeds one time in four.
"""

from __future__ import annotations

import numpy as np

from rta import anim, props
from rta.plans import grasp_and_carry, wait
from rta.spec import TABLE_TOP, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "One of the four battery cells was dropped. Put it in the red quarantine bin and the other three in the charger."

CELL_R, CELL_H, CELL_MASS = 0.017, 0.07, 0.10  # C-size cell, 3.4 cm across: a 2.6 cm cell toppled out of its pocket after a nudge (s02d); the pads (TCP 1.2 cm below the top, tips 2 cm lower) stay 7 mm above the 2.5 cm pocket rims
CELLS = ("cell_a", "cell_b", "cell_c", "cell_d")
HOLDER_XY, HOLDER_Y = (0.13, 0.0), (+0.15, +0.06, -0.06, -0.15)  # 9 cm pitch; inner cells 6 cm off the centre line (an arm stalls at 4.5 cm this close to the bases)
CHARGER_XY, CHARGER_Y = (0.21, 0.0), (+0.075, 0.0, -0.075)  # 7.5 cm pitch for the 6.5 cm pockets; 5 mm clear of the holder, 1 mm clear of the bin
POCKET, DEPTH, BASE = 0.065, 0.025, 0.006  # 1.55 cm clearance per side of the 3.4 cm cell: the fingers close it up to 1 cm off the TCP along x, and 1 cm clearance tipped it on the pocket wall (s02c)
POCKET_FLOOR = TOP + BASE
CELL_IN_POCKET_Z = POCKET_FLOOR + CELL_H / 2 + 0.002
BIN_XY, BIN_SIZE, BIN_H, BIN_WALL = (0.30, 0.0), (0.10, 0.10), 0.06, 0.01
BIN_FLOOR = TOP + BIN_WALL
DROP_FROM = (0.30, 0.20, TOP + 0.30)
LIFT_T0, LIFT_T1, PICK_T0, PICK_DUR = 1.0, 2.0, 3.5, 1.5
ACT_AFTER_S = 6.0
BLUE, GREY, BLACK, RED = (0.15, 0.35, 0.90), (0.50, 0.50, 0.52), (0.08, 0.08, 0.09), (0.85, 0.12, 0.10)

OBJECTS = [
    props.pocket_tray("holder", HOLDER_XY, [(0.0, y) for y in HOLDER_Y], pocket=(POCKET, POCKET), depth=DEPTH, base=BASE, color=GREY),
    props.pocket_tray("charger", CHARGER_XY, [(0.0, y) for y in CHARGER_Y], pocket=(POCKET, POCKET), depth=DEPTH, base=BASE, color=BLACK),
    props.open_box("bin", BIN_XY, BIN_SIZE, BIN_H, wall=BIN_WALL, color=RED),
    *(Cylinder(n, pos=(HOLDER_XY[0], HOLDER_Y[k], CELL_IN_POCKET_Z), radius=CELL_R, height=CELL_H, mass=CELL_MASS, color=BLUE) for k, n in enumerate(CELLS)),
]


def setup(rng, trial) -> dict:
    """Hidden state: the dropped cell (cycles with the seed number; chance 1/4)."""
    return {"dropped": CELLS[trial.balanced(len(CELLS), rng)], "landing": None}


def home(name):
    return (HOLDER_XY[0], HOLDER_Y[CELLS.index(name)], CELL_IN_POCKET_Z)


def events(t: float, state: dict, world) -> None:
    name = state["dropped"]
    if t < PICK_T0:
        anim.drive(world, name, t, [(LIFT_T0, home(name)), (LIFT_T1, DROP_FROM)])  # then physics: it falls
        return
    if state["landing"] is None:
        p = world.pose(name)
        state["landing"] = (float(p[0]), float(p[1]), float(max(p[2], TOP + CELL_H / 2)))
    anim.drive(world, name, t, anim.carry(PICK_T0, state["landing"], home(name), lift=0.12, dur=PICK_DUR))


MILESTONES = tuple(f"{n}_placed" for n in CELLS)
WEIGHTS = (0.25,) * 4
SUCCESS_HOLD_S = 1.0
ZERO_ON = {"dropped_cell_in_charger"}


def _in_charger(o) -> bool:
    return any(
        np.hypot(o.pos[0] - CHARGER_XY[0], o.pos[1] - CHARGER_XY[1] - y) < 0.02 and abs(o.low[2] - POCKET_FLOOR) < 0.01
        for y in CHARGER_Y
    ) and props.tilt_deg(o) < 10.0


def _in_bin(o) -> bool:
    return props.inside(o, BIN_XY, (BIN_SIZE[0] / 2 - BIN_WALL, BIN_SIZE[1] / 2 - BIN_WALL), floor_z=BIN_FLOOR, above=0.03)


def milestones(objects, state) -> list[bool]:
    out = []
    for n in CELLS:
        o = objects[n]
        out.append(bool(o.resting() and (_in_bin(o) if n == state["dropped"] else _in_charger(o))))
    return out


def success(objects, state) -> bool:
    return all(milestones(objects, state))


def failure(objects, state) -> str | None:
    for n in CELLS:
        o = objects[n]
        if o.pos[2] < TOP - 0.2:
            return "cell_off_table"
        if not o.resting():
            continue
        if n == state["dropped"] and _in_charger(o):
            return "dropped_cell_in_charger"
        if n != state["dropped"] and _in_bin(o):
            return "good_cell_in_bin"
    return None


TRAVEL = 0.16
GRIP_FROM_TOP = 0.012
APPROACH = 0.027  # pads 5.4 cm apart around the 3.4 cm cell; fully open (9.4 cm) the fingers landed on the neighbouring cell 9 cm away (s02b)
GRASP_FROM_BOTTOM = CELL_H - GRIP_FROM_TOP
RELEASE_IN_BIN = 0.03  # pads open to 6 cm inside the 8 cm bin


def demo(state) -> dict:
    """One cell at a time, each by the arm on its holder side: good cells (left to right) into the charger pockets
    (left to right), then the dropped cell into the bin."""
    good = sorted((n for n in CELLS if n != state["dropped"]), key=lambda n: -HOLDER_Y[CELLS.index(n)])
    plans = {"left": [], "right": []}
    jobs = [(n, (CHARGER_XY[0], CHARGER_Y[k], POCKET_FLOOR + 0.008 + GRASP_FROM_BOTTOM), APPROACH) for k, n in enumerate(good)]  # release only 2.3 cm: fully open, the fingers reached the cell seated in the next pocket 7 cm away and knocked it (s02d)
    jobs.append((state["dropped"], (BIN_XY[0], BIN_XY[1], BIN_FLOOR + 0.008 + GRASP_FROM_BOTTOM), RELEASE_IN_BIN))
    for n, place, release in jobs:
        hy = HOLDER_Y[CELLS.index(n)]
        arm = "left" if hy > 0 else "right"
        other = "right" if arm == "left" else "left"
        kw = {"approach_finger": APPROACH, **({"release_finger": release} if release is not None else {})}
        seg = grasp_and_carry((HOLDER_XY[0], hy, POCKET_FLOOR + 0.002 + GRASP_FROM_BOTTOM), place, travel_z=TOP + TRAVEL, **kw)
        plans[arm] += seg
        plans[other].append(wait(other, sum(w.duration for w in seg)))
    return {arm: (0.0, wps) for arm, wps in plans.items()}
