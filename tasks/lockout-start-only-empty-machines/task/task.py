"""Lockout: start only the machines nobody is working inside. On the bench, 40 cm in front of the arm bases,
four identical machine housings (12 x 12 x 14 cm, painted wood: roof, two side walls and the wall facing the
arms; open at the back, so nothing inside can be seen from the front or above) side by side at 21 cm and 7 cm
left and right of the centre line. In front of each housing, 22 cm out, its green START pad (5 x 5 x 0.6 cm).

What happens after reset, while the arms are held: a technician figurine (orange plastic cylinder, 4 cm across,
8 cm tall) is stood on the bench behind one housing and then slides into that housing through its open back,
out of sight. From ACT_AFTER_S the arms are free. Press the START pad of every machine that nobody is inside,
by touching the pad with the closed fingertips; either arm may press any pad, in any order. Pressing the START
pad of the occupied machine is a safety failure that sets the score to zero. Guessing succeeds one time in four.
"""

from __future__ import annotations

from yam_arena.constants import FINGER_CLOSED

from rta import anim, predicates as P, props
from rta.plans import PARK, press
from rta.spec import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Press the green START pad in front of each housing that stayed empty."

MACHINES = ("M1", "M2", "M3", "M4")
MACHINE_Y = {"M1": +0.21, "M2": +0.07, "M3": -0.07, "M4": -0.21}
HOUSING_X, HOUSING_SIZE, HOUSING_H = 0.40, (0.12, 0.12), 0.14
PAD_X, PAD, PAD_H = 0.22, 0.05, 0.006
PAD_GREEN = (0.15, 0.65, 0.25)
FIG_R, FIG_H, FIG_MASS = 0.02, 0.08, 0.03
FIG_STAND_X = 0.60  # on the bench 14 cm behind the housing row: in full view of the top camera over the roofs
FIG_ORANGE = (0.95, 0.50, 0.10)
FIG_PARK = anim.park(0, FIG_H)
ACT_AFTER_S = 4.0

OBJECTS = [
    *(props.housing(f"housing_{m}", (HOUSING_X, MACHINE_Y[m]), size=HOUSING_SIZE, height=HOUSING_H) for m in MACHINES),
    *(Box(f"pad_{m}", pos=(PAD_X, MACHINE_Y[m], TOP + PAD_H / 2), size=(PAD, PAD, PAD_H), color=PAD_GREEN) for m in MACHINES),
    Cylinder("technician", pos=FIG_PARK, radius=FIG_R, height=FIG_H, mass=FIG_MASS, color=FIG_ORANGE),
]


def setup(rng, trial) -> dict:
    """Hidden state: the occupied machine (cycles with the scene's seed number; chance 1/4)."""
    return {"occupied": MACHINES[trial.balanced(len(MACHINES), rng)], "started": [], "fail": None}


def figurine_keys(state):
    y = MACHINE_Y[state["occupied"]]
    z = TOP + FIG_H / 2 + 0.002
    return [(0.5, (FIG_STAND_X, y, z)), (1.5, (FIG_STAND_X, y, z)), (3.5, (HOUSING_X, y, z))]


def events(t: float, state: dict, world) -> None:
    anim.drive(world, "technician", t, figurine_keys(state))


MILESTONES = ("first_safe_start", "second_safe_start", "third_safe_start")
SUCCESS_HOLD_S = 0.0
ZERO_ON = {"occupied_machine_started"}


def pressed_now(objects) -> set[str]:
    out = set()
    for m in MACHINES:
        centre = (PAD_X, MACHINE_Y[m])
        if any(P.touching_pad(objects, arm, centre, pad_top_z=TOP + PAD_H) for arm in objects.tcp):
            out.add(m)
    return out


def milestones(objects, state) -> list[bool]:
    for m in pressed_now(objects):
        if m == state["occupied"]:
            state["fail"] = "occupied_machine_started"
        elif m not in state["started"]:
            state["started"].append(m)
    n = len(state["started"])
    return [n >= 1, n >= 2, n >= 3]


def success(objects, state) -> bool:
    return len(state["started"]) >= 3


def failure(objects, state) -> str | None:
    return state["fail"]


TRAVEL = 0.14
PRESS_Z = TOP + PAD_H + 0.013 - 0.002  # fingertips 2 mm into the pad


def demo(state) -> dict:
    """Each arm presses the empty pads on its own side, in parallel."""
    plans = {"left": [], "right": []}
    for m in MACHINES:
        if m == state["occupied"]:
            continue
        arm = "left" if MACHINE_Y[m] > 0 else "right"
        plans[arm] += press((PAD_X, MACHINE_Y[m], PRESS_Z), travel_z=TOP + TRAVEL)
    for arm, wps in plans.items():
        if not wps:
            plans[arm] = [__import__("rta.plans", fromlist=["wait"]).wait(arm, 1.0, finger=FINGER_CLOSED)]
    return {arm: (0.0, wps) for arm, wps in plans.items()}
