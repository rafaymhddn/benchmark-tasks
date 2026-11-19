"""A small gantry stands racked on its two rails: a beam bolted across two carriages, one end 2.7 mm ahead of the
other. Pushed along at both ends together it slides until both carriages touch their far end stops, face to face;
pushed at one end only it turns in its play, the carriages bind on their rails and it stops short.

How it is built. One fixed body: a base plate, two rails and an end stop at both ends of each rail. One free body:
the beam, the two carriages under it and the toolhead on its far face. Nothing but contact holds the gantry on its
rails: each carriage is a channel that wraps the rail's head with 0.25 mm of play a side and reaches under the head
with two lips, so it can be lifted 0.25 mm and no more, and it cannot run off an end past a stop. Binding comes
from that contact and from a friction of 0.2 between carriage and rail; no rule makes it.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as mk  # noqa: E402
from mechkit import body, box  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
INSTRUCTION = "Slide the gantry beam along its rails until both carriages touch their far end stops, then let go."
RUN_LIMIT_STEPS = 600  # 30 s at 20 Hz
TOP = TABLE_TOP[SCENE]

PLATE_GREY, STEEL, RED, ALUMINIUM, BLACK, ORANGE = (0.55, 0.57, 0.60), (0.75, 0.77, 0.80), (0.80, 0.10, 0.08), (0.83, 0.84, 0.86), (0.10, 0.10, 0.11), (0.90, 0.45, 0.05)

# --- the fixture, fixed: base plate 300 x 420 x 10 mm; two MGN12 rails (12 mm wide, 8 mm high, 250 mm long) along x
# from 15 to 40 cm in front of the line of the arm bases, 17 cm either side of the midline; an end stop block
# (10 x 30 x 13 mm) at both ends of each rail ---
PLATE, PLATE_XY = (0.300, 0.420, 0.010), (0.275, 0.0)
RAIL_FROM, RAIL_TO, RAIL_W, RAIL_H = 0.150, 0.400, 0.012, 0.008
RAIL_Y = {"left": 0.170, "right": -0.170}
SIDES = ("left", "right")
# A real MGN12 rail holds its carriage by a ball groove in each side. Here the groove is a square step: under a head
# 12 mm wide and 3.5 mm high the rail is 9 mm wide, and the carriage's lips reach into the step.
NECK_W, NECK_H = 0.009, 0.0045
HEAD_H = RAIL_H - NECK_H
STOP = (0.010, 0.030, 0.013)
FRICTION = (0.2, 0.2)  # carriage on rail (static, dynamic): assumed, to be measured by tilting the fixture until the gantry slides
FOOT = PLATE[2]  # the rails' foot over the bench
_rail_x, _rail_l = (RAIL_FROM + RAIL_TO) / 2, RAIL_TO - RAIL_FROM
_fixture = [box(PLATE, pos=(*PLATE_XY, PLATE[2] / 2), color=PLATE_GREY)]
for _side in SIDES:
    _y = RAIL_Y[_side]
    _fixture.append(box((_rail_l, NECK_W, NECK_H), pos=(_rail_x, _y, FOOT + NECK_H / 2), color=STEEL, friction=FRICTION))
    _fixture.append(box((_rail_l, RAIL_W, HEAD_H), pos=(_rail_x, _y, FOOT + NECK_H + HEAD_H / 2), color=STEEL, friction=FRICTION))
    _fixture.append(box(STOP, pos=(RAIL_TO + STOP[0] / 2, _y, FOOT + STOP[2] / 2), color=RED))  # far stop: its face at x = 40.0 cm
    _fixture.append(box(STOP, pos=(RAIL_FROM - STOP[0] / 2, _y, FOOT + STOP[2] / 2), color=RED))  # near stop
body("fixture", _fixture, pos=(0.0, 0.0, TOP), color=PLATE_GREY)

# --- the gantry, one free body: a 2020 aluminium extrusion 400 mm long across two MGN12H carriages (45.4 mm long,
# 27 mm wide, top 13 mm over the rail's foot), and a toolhead block 40 x 50 x 40 mm on the beam's far face at its
# middle. 0.8 kg in all (assumed from catalogue masses: carriage 54 g, extrusion 0.5 kg/m, the rest the toolhead). ---
BEAM_L, BEAM_S = 0.400, 0.020
CARRIAGE_L, CARRIAGE_W, CARRIAGE_TOP = 0.0454, 0.027, 0.013
PLAY = 0.00025  # m a side between carriage and rail head, and how far a carriage can be lifted
TOOLHEAD = (0.040, 0.050, 0.040)
MASS = 0.8
BEAM_Z = FOOT + CARRIAGE_TOP + BEAM_S / 2  # the beam's middle over the bench when the carriages rest on their rails: 33 mm
_under = FOOT + RAIL_H  # the rail's top: a carriage's roof rests on it
_roof = FOOT + CARRIAGE_TOP - _under  # 5 mm thick
_wall_in, _wall_out = RAIL_W / 2 + PLAY, CARRIAGE_W / 2  # the walls beside the head: 6.25 to 13.5 mm from the rail's middle
_lip_in = NECK_W / 2 + PLAY  # the lips under the head: from 4.75 mm
_lip_top = FOOT + NECK_H - PLAY  # 0.25 mm under the head's underside
_bottom = FOOT + 0.003  # a carriage's underside, 3 mm over the rail's foot
_gantry = [box((BEAM_S, BEAM_L, BEAM_S), color=ALUMINIUM), box(TOOLHEAD, pos=(BEAM_S / 2 + TOOLHEAD[0] / 2, 0.0, 0.0), color=ORANGE)]
for _side in SIDES:
    _y = RAIL_Y[_side]
    _gantry.append(box((CARRIAGE_L, CARRIAGE_W, _roof), pos=(0.0, _y, _under + _roof / 2 - BEAM_Z), color=BLACK, friction=FRICTION))
    for _sign in (-1.0, 1.0):
        _gantry.append(box((CARRIAGE_L, _wall_out - _wall_in, _under - _bottom), pos=(0.0, _y + _sign * (_wall_in + _wall_out) / 2, (_under + _bottom) / 2 - BEAM_Z), color=BLACK, friction=FRICTION))
        _gantry.append(box((CARRIAGE_L, _wall_in - _lip_in, _lip_top - _bottom), pos=(0.0, _y + _sign * (_lip_in + _wall_in) / 2, (_lip_top + _bottom) / 2 - BEAM_Z), color=BLACK, friction=FRICTION))

# --- the start: racked. The left carriage's far face 80.0 mm from its stop, the right one's 77.3 mm: the beam is
# turned 0.45 degrees, within the play of its carriages (which stands for a gantry whose belts and bolts are loose) ---
START_GAP = {"left": 0.0800, "right": 0.0773}
FACE = CARRIAGE_L / 2  # a carriage's far face, from the beam's middle line
START_YAW = float(np.arcsin((START_GAP["left"] - START_GAP["right"]) / (RAIL_Y["left"] - RAIL_Y["right"])))  # about the vertical, anticlockwise from above: 0.45 degrees, the right end ahead
START_X = RAIL_TO - (START_GAP["left"] + START_GAP["right"]) / 2 - FACE * float(np.cos(START_YAW))
body("gantry", _gantry, pos=(START_X, 0.0, TOP + BEAM_Z), yaw=START_YAW, mass=MASS, color=ALUMINIUM)
BOXES = [(np.array(shape.pos) - np.array(shape.dims) / 2, np.array(shape.pos) + np.array(shape.dims) / 2) for shape in _gantry]  # every part of the gantry, in its own frame

OBJECTS = list(mk.MECH.bodies.values())

# --- the rules ---
NEARER, WITHIN, FEELER = 0.025, 0.005, 0.0001  # m: nearer its stop than at the start; near its stop; what a feeler gauge must not enter
RESTING, CLEAR = 0.02, 0.03  # m/s under which the beam is at rest; m both grippers keep from the gantry when they have let go
OFF_RAILS = 0.001  # m: a carriage lifted, shifted sideways or run past its stop by more than this is off its rails (the play is 0.25 mm)
STATE: dict = {}


def begin(seed: int | None = None) -> dict:
    """A new trial's state (no simulator needed)."""
    STATE.clear()
    STATE.update(seed=seed, nearest={side: START_GAP[side] for side in SIDES}, most_turned=abs(START_YAW))
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)


def corners() -> dict:
    """For each carriage, how far the two upright edges of its far face are from its stop's face (m): (the edge
    towards the midline, the edge towards the beam's end). Negative: past the face."""
    position, rotation = mk.pose("gantry")
    out = {}
    for side in SIDES:
        y = RAIL_Y[side]
        inner, outer = (y - np.sign(y) * CARRIAGE_W / 2, y + np.sign(y) * CARRIAGE_W / 2)
        out[side] = tuple(float(RAIL_TO - (position + rotation @ np.array([FACE, edge, 0.0]))[0]) for edge in (inner, outer))
    return out


def gaps() -> dict:
    """How far the middle of each carriage's far face is from its stop's face (m)."""
    return {side: sum(pair) / 2 for side, pair in corners().items()}


def openings() -> dict:
    """The widest opening between each carriage's far face and its stop's face (m): where a feeler gauge would go."""
    return {side: max(pair) for side, pair in corners().items()}


def yaw() -> float:
    """How far the beam is turned about the vertical (rad): 0 is square to the rails."""
    _, rotation = mk.pose("gantry")
    return float(np.arctan2(rotation[1, 0], rotation[0, 0]))


def on_rails() -> bool:
    """Both carriages on their rails and short of their stops: not lifted, not shifted sideways, not past a stop."""
    position, rotation = mk.pose("gantry")
    for side in SIDES:
        middle = position + rotation @ np.array([0.0, RAIL_Y[side], 0.0])
        if abs(middle[2] - (TOP + BEAM_Z)) > OFF_RAILS or abs(middle[1] - RAIL_Y[side]) > OFF_RAILS:
            return False
        near = RAIL_FROM - (position + rotation @ np.array([-FACE, RAIL_Y[side], 0.0]))[0]  # past the near stop when positive
        if near > OFF_RAILS:
            return False
    return min(min(pair) for pair in corners().values()) >= -OFF_RAILS


def to_gantry(point) -> float:
    """How far a point is from the gantry: beam, toolhead and carriages (m; 0 inside)."""
    position, rotation = mk.pose("gantry")
    local = rotation.T @ (np.asarray(point, dtype=float) - position)
    return min(float(np.linalg.norm(np.maximum(np.maximum(low - local, local - high), 0.0))) for low, high in BOXES)


def let_go() -> bool:
    return all(to_gantry(mk.gripper(side)[0]) >= CLEAR for side in SIDES)


def at_rest() -> bool:
    linear, angular = mk.velocity("gantry")
    return float(np.linalg.norm(linear)) < RESTING and float(np.linalg.norm(angular)) * BEAM_L / 2 < RESTING


@mk.on_step
def step() -> None:
    now = gaps()
    for side in SIDES:
        STATE["nearest"][side] = min(STATE["nearest"][side], now[side])
    STATE["most_turned"] = max(STATE["most_turned"], abs(yaw()))


def score(objects) -> float:
    """By where the two carriages stand at the end. Off its rails: 0. Both 25 mm or more nearer their far stops
    than at the start: a quarter. Both within 5 mm: a half. Both touching, face to face (a 0.1 mm feeler enters at
    neither stop): three quarters; and touching, the beam at rest, both grippers 3 cm or more from it: 1.0."""
    if not on_rails():
        return 0.0
    now, wide = gaps(), openings()
    if all(wide[side] <= FEELER for side in SIDES):
        return 1.0 if at_rest() and let_go() else 0.75
    if all(now[side] <= WITHIN for side in SIDES):
        return 0.5
    if all(START_GAP[side] - now[side] >= NEARER for side in SIDES):
        return 0.25
    return 0.0


def success(objects) -> bool:
    return score(objects) == 1.0


def progress() -> str:
    now = gaps()
    return f"from the far stops (mm): left {1000 * now['left']:.2f}, right {1000 * now['right']:.2f}; turned {np.degrees(yaw()):.2f} deg; level {score(None):.2f}"


def report() -> dict:
    """For logs and probes: the numbers behind the score."""
    out = {"nearest to their stops (mm)": {side: round(1000 * v, 2) for side, v in STATE["nearest"].items()}, "most turned (deg)": round(float(np.degrees(STATE["most_turned"])), 3)}
    if mk.MECH.poses:
        now, wide = gaps(), openings()
        out.update({"from their stops now (mm)": {side: round(1000 * now[side], 3) for side in SIDES}, "widest opening at each stop (mm)": {side: round(1000 * wide[side], 3) for side in SIDES},
                    "turned (deg)": round(float(np.degrees(yaw())), 3), "on its rails": on_rails()})  # fmt: skip
        if mk.MECH.grippers:
            out.update({"at rest": at_rest(), "let go": let_go(), "score": score(None)})
    return out
