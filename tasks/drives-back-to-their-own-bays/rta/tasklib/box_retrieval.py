"""Hidden-object retrieval with N identical lidded boxes (2 or 3). ``make(n_boxes)`` returns the task-file
names; a task file does ``globals().update(make(3))`` and supplies its own docstring."""

from __future__ import annotations

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint

from rta.motion import Segment, carry, end_time, play
from rta.plans import CLOSE_S, SQUEEZE_S, grasp_and_carry
from rta.spec import TABLE_TOP, Box, Compound, Part

LAYOUTS = {
    # box name -> centre (x, y); lid start (behind the boxes); which arm lifts the lid; where it parks the lid
    2: dict(
        boxes={"left": (0.18, +0.085), "right": (0.18, -0.085)},
        lids={"left": (0.34, +0.085), "right": (0.34, -0.085)},
        lid_arm={"left": "left", "right": "right"},
        park={"left": (0.30, +0.085), "right": (0.30, -0.085)},
        outer=0.14,
        cube_start=(0.06, 0.0),
        cube_jitter=0.015,
        finger_in_box=FINGER_OPEN,
    ),
    3: dict(
        boxes={"left": (0.15, +0.095), "centre": (0.28, 0.0), "right": (0.15, -0.095)},
        lids={"left": (0.30, +0.19), "centre": (0.42, 0.0), "right": (0.30, -0.19)},
        lid_arm={"left": "left", "centre": "left", "right": "right"},
        park={"left": (0.30, +0.19), "centre": (0.28, +0.17), "right": (0.30, -0.19)},
        outer=0.12,
        cube_start=(0.05, 0.0),
        cube_jitter=0.01,
        finger_in_box=-0.035,  # 7 cm opening: fits the 10 cm cavity with 1.5 cm to spare beside a 4 cm cube
    ),
}


def make(n_boxes: int) -> dict:
    L = LAYOUTS[n_boxes]
    g: dict = {}
    SCENE = "station"
    TOP = TABLE_TOP[SCENE]
    INSTRUCTION = "Open the box the red cube was put in and take the cube out onto the table."
    BOX_OUTER, BOX_WALL, BOX_HEIGHT = L["outer"], 0.01, 0.05
    BOXES = tuple(L["boxes"])
    BOX_XY = dict(L["boxes"])
    LID_START = dict(L["lids"])
    LID_ARM, LID_PARK = dict(L["lid_arm"]), dict(L["park"])
    LID_PLATE, LID_KNOB = 0.01, 0.03
    CUBE = 0.04
    CUBE_START, CUBE_JITTER = L["cube_start"], L["cube_jitter"]
    DROP_XY = CUBE_START
    FINGER_IN_BOX = L["finger_in_box"]
    BOX_Z = TOP + BOX_HEIGHT / 2
    CUBE_REST_Z = TOP + CUBE / 2
    CUBE_IN_BOX_Z = TOP + BOX_WALL + CUBE / 2
    LID_ON_TABLE_Z = TOP + LID_PLATE / 2
    LID_ON_BOX_Z = TOP + BOX_HEIGHT + LID_PLATE / 2
    KNOB_ABOVE_PLATE = LID_PLATE / 2 + LID_KNOB / 2
    LID_CENTROID_ON_BOX = LID_ON_BOX_Z + LID_KNOB / 2
    LIFT_Z = TOP + 0.15
    GREY, DARK, RED = (0.62, 0.62, 0.60), (0.35, 0.35, 0.36), (0.85, 0.12, 0.12)
    inner = BOX_OUTER - 2 * BOX_WALL

    def _open_box(name, xy):
        half = (BOX_OUTER - BOX_WALL) / 2
        return Compound(
            name, pos=(xy[0], xy[1], TOP + BOX_WALL / 2), color=GREY, size=(BOX_OUTER, BOX_OUTER, BOX_WALL),
            parts=(
                Part((BOX_WALL, BOX_OUTER, BOX_HEIGHT), (+half, 0.0, BOX_HEIGHT / 2 - BOX_WALL / 2)),
                Part((BOX_WALL, BOX_OUTER, BOX_HEIGHT), (-half, 0.0, BOX_HEIGHT / 2 - BOX_WALL / 2)),
                Part((inner, BOX_WALL, BOX_HEIGHT), (0.0, +half, BOX_HEIGHT / 2 - BOX_WALL / 2)),
                Part((inner, BOX_WALL, BOX_HEIGHT), (0.0, -half, BOX_HEIGHT / 2 - BOX_WALL / 2)),
            ),
        )

    def _lid(name, xy):
        return Compound(
            name, pos=(xy[0], xy[1], LID_ON_TABLE_Z), color=DARK, mass=0.08, size=(BOX_OUTER, BOX_OUTER, LID_PLATE),
            parts=(Part((LID_KNOB, LID_KNOB, LID_KNOB), (0.0, 0.0, KNOB_ABOVE_PLATE)),),
        )

    OBJECTS = [
        *(_open_box(f"box_{b}", BOX_XY[b]) for b in BOXES),
        *(_lid(f"lid_{b}", LID_START[b]) for b in BOXES),
        Box("cube", pos=(*CUBE_START, CUBE_REST_Z), size=(CUBE, CUBE, CUBE), mass=0.05, color=RED),
    ]
    CUBE_MOVE_T0, LIDS_MOVE_T0 = 2.0, 5.0
    ACT_AFTER_S = 8.5

    def setup(rng, trial):
        box = BOXES[trial.balanced(len(BOXES), rng)]
        dx, dy = rng.uniform(-CUBE_JITTER, CUBE_JITTER, size=2)
        return {"box": box, "cube_xy": (CUBE_START[0] + float(dx), CUBE_START[1] + float(dy))}

    def placements(state):
        return {"cube": (*state["cube_xy"], CUBE_REST_Z)}

    def segments(state):
        bx, by = BOX_XY[state["box"]]
        segs = carry("cube", CUBE_MOVE_T0, (*state["cube_xy"], CUBE_REST_Z), (bx, by, CUBE_IN_BOX_Z + 0.002),
                     lift_z=LIFT_Z, rise_s=1.0, move_s=1.5, drop_s=0.5)
        for b in BOXES:
            x, y = BOX_XY[b]
            segs += carry(f"lid_{b}", LIDS_MOVE_T0, (*LID_START[b], LID_ON_TABLE_Z), (x, y, LID_ON_BOX_Z + 0.002),
                          lift_z=LIFT_Z, rise_s=1.0, move_s=1.5, drop_s=0.5)
        return segs

    assert end_time(segments({"box": BOXES[0], "cube_xy": CUBE_START})) + 0.2 <= ACT_AFTER_S

    def events(t, state, world):
        play(segments(state), t, world, dt=0.1)

    MILESTONES = ("correct_lid_lifted", "cube_lifted", "cube_out_of_box", "cube_on_table_outside")
    LID_LIFT, LID_SLIDE = 0.03, 0.06
    EDGE = BOX_OUTER / 2 + 0.01

    def _lid_moved(objects, b):
        lid = objects[f"lid_{b}"]
        bx, by = BOX_XY[b]
        return bool(lid.pos[2] > LID_CENTROID_ON_BOX + LID_LIFT or np.hypot(lid.pos[0] - bx, lid.pos[1] - by) > LID_SLIDE)

    def _over_box(objects, b):
        c = objects["cube"].pos
        bx, by = BOX_XY[b]
        return bool(abs(c[0] - bx) <= EDGE and abs(c[1] - by) <= EDGE)

    def _cube_on_table_outside(objects):
        cube = objects["cube"]
        return bool(not any(_over_box(objects, b) for b in BOXES) and cube.low[2] < TOP + 0.015 and cube.low[2] > TOP - 0.02 and cube.resting())

    def milestones(objects, state):
        box = state["box"]
        return [_lid_moved(objects, box), bool(objects["cube"].pos[2] > CUBE_IN_BOX_Z + 0.03), not _over_box(objects, box), _cube_on_table_outside(objects)]

    def success(objects, state):
        return _cube_on_table_outside(objects)

    def failure(objects, state):
        if any(_lid_moved(objects, b) for b in BOXES if b != state["box"]):
            return "wrong_box_opened"
        if objects["cube"].pos[2] < TOP - 0.2:
            return "cube_fell"
        return None

    TRAVEL_SAME, TRAVEL_CROSS = 0.16, 0.14

    def demo(state):
        box = state["box"]
        lid_arm_name = LID_ARM[box]
        cube_arm_name = "right" if lid_arm_name == "left" else "left"
        bx, by = BOX_XY[box]
        knob_z = LID_ON_BOX_Z + KNOB_ABOVE_PLATE - 0.005
        park = LID_PARK[box]
        lid_arm = [
            Waypoint(1.5, (bx, by, TOP + TRAVEL_SAME), FINGER_OPEN, "over the lid"),
            Waypoint(1.5, (bx, by, knob_z), FINGER_OPEN, "down to the knob"),
            Waypoint(CLOSE_S, (bx, by, knob_z), FINGER_CLOSED, "grasp the knob"),
            Waypoint(SQUEEZE_S, (bx, by, knob_z), FINGER_CLOSED, "squeeze"),
            Waypoint(1.2, (bx, by, TOP + TRAVEL_SAME), FINGER_CLOSED, "lift the lid"),
            Waypoint(2.0, (*park, TOP + TRAVEL_SAME), FINGER_CLOSED, "hold it out of the way"),
        ]
        lid_done = sum(w.duration for w in lid_arm)
        travel = TRAVEL_SAME if box == "centre" else TRAVEL_CROSS
        cube_arm = grasp_and_carry((bx, by, CUBE_IN_BOX_Z), (*DROP_XY, CUBE_REST_Z), travel_z=TOP + travel, approach_finger=FINGER_IN_BOX)
        return {lid_arm_name: (0.0, lid_arm), cube_arm_name: (lid_done, cube_arm)}

    g.update({k: v for k, v in locals().items() if not k.startswith("_") or k in ("_lid_moved", "_over_box", "_cube_on_table_outside")})
    g.pop("L", None)
    g.pop("g", None)
    return g
