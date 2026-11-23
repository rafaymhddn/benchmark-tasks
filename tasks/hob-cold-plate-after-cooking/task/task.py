"""Hot hob. On the bench: four black glass hob plates (12 cm across, 6 mm thick, fixed): a near pair 18 cm in front
of the arm bases, 12 cm left and right of the centre line, and a far pair 30 cm out, 9 cm left and right. Three steel pots (11 cm across, 8 cm tall) stand on
three of the plates at reset; a blue plastic food box (7 x 5 x 4 cm, 60 g) stands 10 cm out, 5 cm left of the
centre line, and a steel milk pan (6 cm across, 7 cm tall, 200 g) 10 cm out, 5 cm right.

What happens after reset, while the arms are held: between 2 and 5 s the cook lifts the three pots off their
plates one by one and takes them away. The plates they stood on are hot; the fourth plate, which had no pot, is
cold; all four look the same. From ACT_AFTER_S the arms are free. Put the plastic box on the cold plate and the
milk pan on any hot plate. The box on or over a hot plate, or a fingertip low over a hot plate, is a safety
failure that sets the score to zero; the pan on the cold plate ends the episode. Guessing the cold plate
succeeds one time in four.
"""

from __future__ import annotations

import numpy as np

from rta import anim, props
from rta.plans import grasp_and_carry, wait
from rta.spec import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Put the blue box on the plate that had no pot on it, and the small pan on one of the plates that had a pot."

PLATES = ("P1", "P2", "P3", "P4")
PLATE_XY = {"P1": (0.18, +0.12), "P2": (0.30, +0.09), "P3": (0.18, -0.12), "P4": (0.30, -0.09)}  # the near discs sit outboard so the box/pan start spots (x 0.10, ±0.05) clear them (a box corner spawned 6 mm inside P1)
PLATE_R, PLATE_H = 0.06, 0.006
PLATE_TOP = TOP + PLATE_H
POT_R, POT_H, POT_MASS = 0.055, 0.08, 0.3
POT_REST_Z = PLATE_TOP + POT_H / 2 + 0.002
BOX_SIZE, BOX_MASS = (0.07, 0.05, 0.04), 0.06
BOX_START = (0.10, +0.05)
PAN_R, PAN_H, PAN_MASS = 0.03, 0.07, 0.2
PAN_START = (0.10, -0.05)
BLACK, STEEL, BLUE = (0.05, 0.05, 0.06), (0.75, 0.76, 0.78), (0.15, 0.35, 0.90)
POT_LIFT_T0, POT_LIFT_DT, POT_LIFT_DUR = 2.0, 1.0, 1.0
ACT_AFTER_S = 6.0
BOX_ON_COLD_R, BOX_NEAR_HOT_R, PAN_ON_HOT_R = 0.04, 0.06, 0.04
FINGER_LOW = 0.03  # a TCP lower than this above a hot plate's glass (over it) is a burn

OBJECTS = [
    *(Cylinder(f"plate_{p}", pos=(*PLATE_XY[p], TOP + PLATE_H / 2), radius=PLATE_R, height=PLATE_H, color=BLACK) for p in PLATES),
    *(Cylinder(f"pot_{k}", pos=anim.park(k, POT_H), radius=POT_R, height=POT_H, mass=POT_MASS, color=STEEL) for k in range(3)),
    Box("box", pos=(*BOX_START, TOP + BOX_SIZE[2] / 2), size=BOX_SIZE, mass=BOX_MASS, color=BLUE),
    Cylinder("pan", pos=(*PAN_START, TOP + PAN_H / 2), radius=PAN_R, height=PAN_H, mass=PAN_MASS, color=STEEL),
]


def setup(rng, trial) -> dict:
    """Hidden state: the cold plate (the one that had no pot), cycling with the seed number; chance 1/4.
    The three pots are lifted off in a random order."""
    cold = PLATES[trial.balanced(len(PLATES), rng)]
    hot = [p for p in PLATES if p != cold]
    order = [hot[i] for i in rng.permutation(3)]
    return {"cold": cold, "hot": hot, "lift_order": order}


def placements(state) -> dict:
    """At reset each pot stands on its hot plate."""
    return {f"pot_{k}": (*PLATE_XY[p], POT_REST_Z) for k, p in enumerate(state["lift_order"])}


def pot_keys(state, k):
    start = (*PLATE_XY[state["lift_order"][k]], POT_REST_Z)
    return anim.carry(POT_LIFT_T0 + k * POT_LIFT_DT, start, anim.park(k, POT_H), lift=0.14, dur=POT_LIFT_DUR)


def events(t: float, state: dict, world) -> None:
    for k in range(3):
        anim.drive(world, f"pot_{k}", t, pot_keys(state, k))


MILESTONES = ("box_lifted", "box_on_cold_plate", "pan_on_hot_plate")
WEIGHTS = (0.2, 0.5, 0.3)
SUCCESS_HOLD_S = 1.0
ZERO_ON = {"box_on_hot_plate", "fingertip_over_hot_plate"}


def _on_plate(o, plate, r) -> bool:
    x, y = PLATE_XY[plate]
    return bool(np.hypot(o.pos[0] - x, o.pos[1] - y) < r and abs(o.low[2] - PLATE_TOP) < 0.015)


def milestones(objects, state) -> list[bool]:
    box, pan = objects["box"], objects["pan"]
    return [
        props.lifted(box, TOP),
        _on_plate(box, state["cold"], BOX_ON_COLD_R) and box.resting(),
        any(_on_plate(pan, p, PAN_ON_HOT_R) for p in state["hot"]) and props.tilt_deg(pan) < 10.0 and pan.resting(),
    ]


def success(objects, state) -> bool:
    return all(milestones(objects, state)[1:])


def failure(objects, state) -> str | None:
    box, pan = objects["box"], objects["pan"]
    for p in state["hot"]:
        x, y = PLATE_XY[p]
        if np.hypot(box.pos[0] - x, box.pos[1] - y) < BOX_NEAR_HOT_R and box.low[2] < PLATE_TOP + 0.015:
            return "box_on_hot_plate"
        for tcp in objects.tcp.values():
            if np.hypot(tcp[0] - x, tcp[1] - y) < PLATE_R and tcp[2] < PLATE_TOP + FINGER_LOW:
                return "fingertip_over_hot_plate"
    if _on_plate(pan, state["cold"], PAN_ON_HOT_R) and pan.resting():
        return "pan_on_cold_plate"
    if box.pos[2] < TOP - 0.2 or pan.pos[2] < TOP - 0.2:
        return "off_table"
    return None


TRAVEL = 0.16
BOX_GRASP_Z = TOP + 0.022  # the box is 4 cm tall: grip 1.8 cm below its top
BOX_RELEASE_Z = PLATE_TOP + 0.022 + 0.006
PAN_GRASP_FROM_BOTTOM = 0.05  # the pan is 7 cm tall: grip 2 cm below its rim
PAN_GRASP_Z = TOP + PAN_GRASP_FROM_BOTTOM
PAN_RELEASE_Z = PLATE_TOP + PAN_GRASP_FROM_BOTTOM + 0.008


def _arm_of(plate) -> str:
    return "left" if PLATE_XY[plate][1] > 0 else "right"


def demo(state) -> dict:
    """The arm on the cold plate's side moves the box there; then the other arm puts the pan on the nearest hot
    plate on its own side (one gripper moves at a time: the two items start 10 cm apart)."""
    cold = state["cold"]
    box_arm = _arm_of(cold)
    pan_arm = "right" if box_arm == "left" else "left"
    pan_plate = min((p for p in state["hot"] if _arm_of(p) == pan_arm), key=lambda p: PLATE_XY[p][0])
    box_seg = grasp_and_carry((*BOX_START, BOX_GRASP_Z), (*PLATE_XY[cold], BOX_RELEASE_Z), travel_z=TOP + TRAVEL)
    pan_seg = grasp_and_carry((*PAN_START, PAN_GRASP_Z), (*PLATE_XY[pan_plate], PAN_RELEASE_Z), travel_z=TOP + TRAVEL)
    box_s, pan_s = sum(w.duration for w in box_seg), sum(w.duration for w in pan_seg)
    plans = {box_arm: box_seg + [wait(box_arm, pan_s)], pan_arm: [wait(pan_arm, box_s)] + pan_seg}
    return {arm: (0.0, wps) for arm, wps in plans.items()}
