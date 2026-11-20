"""A servo hangs by its mounting ears on two posts on the bench, its splined output shaft up, a zero mark engraved on
its case top from the shaft's boss outwards. Its single-arm horn lies flat on the bench beside it, the arm 68 degrees
from the mark's direction. Put on over the shaft, the horn rests on the spline's crests, 3.5 mm proud and free to
turn; within 2 degrees of a tooth position it goes down under 32 N or more, is then held in turn by the splines, and
is flush after 3.5 mm. Pulled up with 10 N it comes off again.

How it is built. The servo is one fixed body: case, ears, posts, boss and the spline's lower part, a plain cylinder
5.9 mm across. The spline's top 3 mm, the crests, are a body of their own (the "seat") on a vertical slide, their top
edge chamfered 0.35 mm. The horn is one free body: a hub of twenty-four staves round a bore that is 6.8 mm across at
its mouth and narrows at 45 degrees to 6.05 mm, 0.075 mm a side round the spline (the horn's teeth round the shaft's),
up to the cap; the arm; and from 1.4 mm over the mouth up to the cap a square core for the horn's teeth and the hub's
material between them, which passes the spline's fixed part and rests on the crests' square face. The arm collides as
its top half millimetre (drawn 2 mm thick), so that the horn's centre of mass lies over the crests' face. A horn let
down over the shaft within 0.8 mm (its mouth on the crests' chamfer) slides on and comes to rest on the crests, 3.5 mm
proud. Sitting there within its play, held or let go, it is put exactly on the shaft's axis, upright and on the crests
(0.15 mm, 7 degrees and 0.3 mm at most) and welded to a hidden disc (the "turn") that turns freely about the
shaft's axis on the seat: free to turn, and held up by joints, not by its touch on the crests. The seat holds still
under any press while the horn's arm is more than 2 degrees from a tooth position; within 2 degrees it gives under 32 N
and stays where it is pushed (a stiff spring capped at that force whose rest follows the seat, as the shield's seats in
shield_seat); once it has gone 0.2 mm down the turn is locked (engaged: the splines hold the horn in turn, at the angle
it went in), and the horn is flush after 3.5 mm. Pulled up, an engaged horn rises with the seat under 10 N and is free
to turn again at the crests; lifted with more than 1 N, a horn that is not engaged comes off.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as mk  # noqa: E402
from mechkit import body, box, cylinder, hinge, slide, weld  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
INSTRUCTION = "Fit the white servo horn onto the servo's splined output shaft with its arm lined up on the engraved zero mark, then let go."
RUN_LIMIT_STEPS = 1200  # three times the scripted full run (380 steps, recording c3zf), rounded up to a hundred steps; at most 1,200
TOP = TABLE_TOP[SCENE]

BLACK, BRASS, ALUMINIUM, WHITE, NYLON = (0.06, 0.06, 0.07), (0.78, 0.62, 0.30), (0.72, 0.73, 0.75), (0.92, 0.92, 0.90), (0.95, 0.95, 0.92)

# --- the servo: TowerPro MG996R, 40.7 x 19.7 x 42.9 mm over all with the shaft, 55 g (the maker's figures as sellers
# list them). Assumed, to be measured with a caliper: case top 35 mm up, boss 3 mm, spline 4.9 mm over the boss, shaft
# axis 10 mm from one end of the case, mounting ears 2.5 mm thick, their undersides 27 mm up, 54 mm over all. Fixed:
# it hangs by its ears on two posts screwed to the bench. Long axis along x, its middle 43 cm in front of the line of
# the arm bases and 22 cm to the right of the midline, where the right wrist camera sees it from the ready pose; the
# shaft at the case's near end (-x), so that the mark runs away from the arms ---
SERVO_XY = tuple(float(v) for v in os.environ.get("HORN_SERVO_XY", "0.43,-0.22").split(","))  # a trial may put it elsewhere (trials.py servo=x,y)
SHAFT_END = os.environ.get("HORN_SHAFT_END", "near")  # the case's end the shaft is near: far (+x) or near (-x, towards the arms)
CASE_L, CASE_W, CASE_H = 0.0407, 0.0197, 0.035
EAR_SPAN, EAR_T, EAR_UNDER = 0.054, 0.0025, 0.027
BOSS_D, BOSS_H = 0.0115, 0.003
SPLINE_D, SPLINE_H = 0.0059, 0.0049
SEAT_H, CREST_CHAMFER = 0.003, 0.00035  # the spline's top 3 mm are the seat (see below); its top edge chamfered 0.35 mm at 45 degrees (assumed)
SHAFT_FROM_END = 0.010
SHAFT_X = (CASE_L / 2 - SHAFT_FROM_END) * (1.0 if SHAFT_END == "far" else -1.0)  # in the servo's frame
BOSS_TOP = CASE_H + BOSS_H  # 38 mm over the bench
CRESTS = BOSS_TOP + SPLINE_H  # 42.9 mm over the bench
TEETH = 25
TOOTH = 2 * math.pi / TEETH  # 14.4 degrees
MARK_L, MARK_W = 0.024, 0.0012  # engraved from the boss outwards, towards the case's longer side (-x)
MARK = np.array([-1.0 if SHAFT_END == "far" else 1.0, 0.0, 0.0])  # the mark's direction from the shaft, in the world: towards the case's longer side
SHAFT_AT = np.array([SERVO_XY[0] + SHAFT_X, SERVO_XY[1]])  # the shaft's axis on the bench

_mark_from = SHAFT_X + MARK[0] * BOSS_D / 2
_servo = [
    box((CASE_L, CASE_W, CASE_H), pos=(0.0, 0.0, CASE_H / 2), color=BLACK),
    box((EAR_SPAN, CASE_W, EAR_T), pos=(0.0, 0.0, EAR_UNDER + EAR_T / 2), color=BLACK),
    *[box((0.006, 0.012, EAR_UNDER), pos=(s * (CASE_L / 2 + 0.0033), 0.0, EAR_UNDER / 2), color=ALUMINIUM) for s in (-1, 1)],
    cylinder(BOSS_D / 2, BOSS_H, pos=(SHAFT_X, 0.0, CASE_H + BOSS_H / 2), color=BLACK),
    cylinder(SPLINE_D / 2, SPLINE_H - SEAT_H, pos=(SHAFT_X, 0.0, BOSS_TOP + (SPLINE_H - SEAT_H) / 2), color=BRASS),  # the spline's lower part; its upper part, the crests, is the seat
    box((MARK_L, MARK_W, 0.0002), pos=(_mark_from + MARK[0] * MARK_L / 2, 0.0, CASE_H + 0.0001), color=WHITE, solid=False),
]
body("servo", _servo, pos=(*SERVO_XY, TOP), color=BLACK)

# --- the horn: the single-arm horn of the servo's own horn set, white nylon, about 4 g; hub 14 mm across and 7 mm tall,
# bore 6.8 mm at its mouth, arm 28 mm from the hub's axis to its tip, 7 mm wide, 2 mm thick, its top flush with the
# hub's. The horn's frame: its origin on the hub's axis at the hub's mouth, x along the arm, z up the hub. Assumed:
# the mouth's 0.35 mm chamfer, the cap 1 mm thick. From the earlier submission, which measured them on this servo
# (not measured by us): the horn stands 3.5 mm proud when its teeth meet the shaft's crests, and seating it takes 32 N ---
HORN_MASS = 0.004
HUB_D, HUB_H, BORE_D, CAP_T = 0.014, 0.007, 0.0068, 0.001
GUIDE_D = 0.00605  # the bore past its mouth, round the spline: 0.075 mm a side (the horn's teeth round the shaft's; with 0.015 mm the
# hand had to put the horn within 0.015 mm before it would go down onto the crests, and it hung on their chamfer: run c3l)
MOUTH = (BORE_D - GUIDE_D) / 2  # the mouth's 45 degree chamfer, from 6.8 mm across down to the guide: 0.435 mm
LEDGE = 0.0014  # how far up the bore the horn's teeth begin: the hub's mouth stands this far under the crests when the horn rests on them
ARM_L, ARM_W, ARM_T = 0.028, 0.007, 0.002
ARM_SOLID = 0.0005  # m: the arm collides as its top half millimetre, and is drawn 2 mm thick. As a solid 2 mm arm of even density it put
# the horn's centre of mass 3.5 mm from the hub's axis, past the crests' flat top, and a horn let go on the crests tipped over
# onto its arm (runs c3k, c3l); a real horn stays on them, held upright by its bore round the spline's top. Its top face, where
# a fingertip presses, is the same either way
STAVES, MOUTH_PLATES = 24, 16
PROUD = 0.0035  # the horn on the crests stands this high over its seated place
TRAVEL = 0.00345  # the seat's travel: the hub's mouth seated 0.05 mm over the boss, so that the seat's stop, not the boss, carries a press
REST_Z = CRESTS - LEDGE  # the hub's mouth over the bench when the horn rests on the crests: 41.5 mm
HORN_XY, HORN_START = tuple(float(v) for v in os.environ.get("HORN_XY", "0.38,-0.33").split(",")), math.radians(68.0)  # the arm 68 degrees from the mark's direction, 4 degrees off a tooth position


def _frame(columns) -> tuple:
    return mk.matrix_to_quat(np.column_stack(columns))


def _ring(inner: float, outer: float, low: float, high: float, count: int) -> list:
    """Staves round the horn's axis between ``inner`` and ``outer`` radius and heights ``low`` to ``high``: a polygon
    bore whose flats stand ``inner`` from the axis, a polygon outside whose flats stand ``outer`` from it."""
    width = 2 * outer * math.tan(math.pi / count) * 1.05
    shapes = []
    for k in range(count):
        a = 2 * math.pi * (k + 0.5) / count
        radial, tangent = np.array([math.cos(a), math.sin(a), 0.0]), np.array([-math.sin(a), math.cos(a), 0.0])
        middle = (inner + outer) / 2 * radial + np.array([0.0, 0.0, (low + high) / 2])
        shapes.append(box((outer - inner, width, high - low), pos=tuple(middle), quat=_frame([radial, tangent, np.array([0.0, 0.0, 1.0])])))
    return shapes


def _mouth() -> list:
    """The bore's 45 degree chamfer at the mouth, as plates on the cone from 6.8 mm across at the mouth to the guide."""
    slant, thick = math.sqrt(2.0) * MOUTH, 0.0005
    middle_r = BORE_D / 2 - MOUTH / 2
    width = 2 * middle_r * math.tan(math.pi / MOUTH_PLATES) * 1.08
    shapes = []
    for k in range(MOUTH_PLATES):
        a = 2 * math.pi * k / MOUTH_PLATES
        radial, tangent, up = np.array([math.cos(a), math.sin(a), 0.0]), np.array([-math.sin(a), math.cos(a), 0.0]), np.array([0.0, 0.0, 1.0])
        into = (radial + up) / math.sqrt(2.0)  # from the chamfer's face into the hub
        along = (up - radial) / math.sqrt(2.0)  # up the chamfer, towards the axis
        middle = middle_r * radial + MOUTH / 2 * up + thick / 2 * into  # the face's middle on the cone, the plate behind it
        shapes.append(box((width, slant, thick), pos=tuple(middle), quat=_frame([tangent, along, into])))
    return shapes


CORE_SIDE = 0.003  # m: the core's square, inside the crests' square face (3.6 mm): a 4 mm core's corners overhung the face, caught on the chamfer's steps and sank into the crests under a 32 N press off a tooth position (run c3q)
_horn = [
    *_ring(BORE_D / 2 + 0.0001, HUB_D / 2, 0.0, MOUTH, STAVES),  # the hub's foot round the chamfer
    *_mouth(),
    *_ring(GUIDE_D / 2, HUB_D / 2, MOUTH, HUB_H - CAP_T, STAVES),
    cylinder(HUB_D / 2, CAP_T, pos=(0.0, 0.0, HUB_H - CAP_T / 2)),
    # The horn's teeth and the hub's material between them, from where they begin up to the cap: their lower ends rest
    # on the seat; they pass the spline's fixed lower part, which they slide over as the horn goes down. A square column
    # (4 mm, within the guide): as a cylinder on the crests' cylinder, pressed by a hand, it sank 3.2 mm into them while
    # they carried 6 N (run c3h: a cylinder's flat face touches another at one point in PhysX)
    box((CORE_SIDE, CORE_SIDE, HUB_H - CAP_T - LEDGE), pos=(0.0, 0.0, (LEDGE + HUB_H - CAP_T) / 2), passes=("servo",)),
    box((ARM_L - ARM_W / 2 - GUIDE_D / 2, ARM_W, ARM_SOLID), pos=((ARM_L - ARM_W / 2 + GUIDE_D / 2) / 2, 0.0, HUB_H - ARM_SOLID / 2)),
    cylinder(ARM_W / 2, ARM_SOLID, pos=(ARM_L - ARM_W / 2, 0.0, HUB_H - ARM_SOLID / 2)),
    box((ARM_L - ARM_W / 2 - HUB_D / 2, ARM_W, ARM_T), pos=((ARM_L - ARM_W / 2 + HUB_D / 2) / 2, 0.0, HUB_H - ARM_T / 2), solid=False),  # the arm as it is drawn
    cylinder(ARM_W / 2, ARM_T, pos=(ARM_L - ARM_W / 2, 0.0, HUB_H - ARM_T / 2), solid=False),
    cylinder(0.0011, 0.0002, pos=(0.0, 0.0, HUB_H + 0.0001), color=(0.55, 0.55, 0.55), solid=False),  # the screw hole in the cap
]
_start_dir = mk.axis_angle((0.0, 0.0, 1.0), HORN_START) @ MARK
HORN = body("horn", _horn, pos=(*HORN_XY, TOP), yaw=math.atan2(_start_dir[1], _start_dir[0]), mass=HORN_MASS, color=NYLON)

# --- the crests and how the horn goes down on them ---
# The crests are the seat: the spline's top 3 mm, a body on a vertical slide that passes the servo's own shapes (it goes
# down into the spline). It resists like the shield's seats (shield_seat): a stiff spring capped at the resistance, its
# rest following a seat that has gone beyond the spring's give and a little slack; the seat half a kilogram and
# weightless. On the seat a small hidden disc, the turn, turns about the shaft's axis on a hinge, freely or locked.
# A horn that sits on the crests within its play, held or let go, is put exactly on the shaft's axis and upright and
# welded to the turn: from then on joints hold it, not its touch on the crests. Held by that touch, a horn of 4 to 20 g
# pressed with 32 N or more off a tooth position sank into the crests in this simulator (runs c3i to c3r; at 200 g it
# stood, c3s), however the crests were shaped or held.
#   Off a tooth position (the arm more than 2 degrees from one) the seat's cap is far above what an arm can press: the
#   teeth stand on the crests.
#   Within 2 degrees the seat gives under 32 N and stays where it is pushed; once it has gone 0.2 mm the turn is locked
#   at the angle it has (the splines have met: engaged).
#   Engaged and pulled up, the seat rises under 10 N; back at the crests the turn is free again.
#   Not engaged and lifted with more than 1 N, the weld is cut and the horn is free.
PRESS, PULL_OFF, LIFT, BLOCK = 32.0, 10.0, 1.0, 2000.0  # N: to seat the horn (quoted); to pull it off again (ASSUMED); to lift a horn off the crests; what stands a misaligned horn on them
SEAT_MASS, SEAT_GIVE, SEAT_SLACK, SEAT_DAMPING_S, GRAVITY = 0.5, 0.0001, 0.0001, 0.01, 9.81  # slack 0.1 mm: with 0.05 mm (shield_seat's) a horn pressed with 30 N crept home (run c3c)
SEAT_HOME = 0.0003  # m from its stop: a seat that near goes home and stays (flush)
ENGAGE = 0.0002  # m the seat has gone down when the turn is locked: the splines have met
FREE_AT = 0.00005  # m under the crests: an engaged horn pulled back up this far is free to turn again
LIFT_UP, LIFTED = 0.0008, 0.0006  # m the seat can rise over the crests' place, so that a lift is felt; risen LIFTED (twice CAPTURE's height), a horn
# not engaged comes free and is not taken back at once (with the seat free to rise 1.5 mm, a horn taken in the hand flew off the crests and the seat
# went 12 mm past its stop: recording c3zi)
PIN = 0.001  # m under the bottom stop where the seat's spring rests when an engaged horn is home: the stop, a hard limit, carries a press
CAPTURE = (0.00015, math.radians(7.0), 0.0003)  # the horn sits on the crests within its play: its axis this near the shaft's (the play is 0.075 mm a side, more
# at the mouth when it leans), this near upright, its mouth this near its place on the seat. Let go on the crests it came to rest 0.074 mm off the axis,
# or leaning 0.75 degrees (run c3t); let fall onto them on the axis tilted 3 and 4 degrees, it came to rest wedged, leaning 3.0 to 5.5 degrees with its
# mouth 0.08 to 0.2 mm high (runs c3x, c3zb, c3zd). Not taken, a horn pressed sank into the crests (4.5 mm in c3t). Within the scorer's 8 degrees
TURN_MASS, TURN_K, LOCK = 0.5, 20.0, 2.0  # kg (weightless, hidden and 2 mm across: it adds nothing a hand can feel); N m/rad; N m the locked turn holds
HELD_MASS = 0.2  # kg: the horn's mass while it is welded to the turn, its weight over 4 g carried by a push. At 4 g, welded, a fingertip
# press of 37 N drove it 3.7 mm down past the turn and the seat (they went 0.15 mm: run c3u), a light body between a hand and joints
# giving way in this simulator; at 200 g it stood on its touch alone (run c3s). Free, it weighs 4 g
TURN_DRAG = 0.001  # N m: the free turn's drag, so that a horn let go while it turns stops as a 4 g one would (within 0.1 degree from
# 0.5 rad/s at 200 g); a hand turning it feels 0.14 N at the hub's rim
FACE_PROUD = 0.00005  # m the crests' square face stands over their round top
WINDOW = math.radians(2.0)  # the arm within this of a tooth position: the teeth find their gaps (the earlier build's figure, ASSUMED)


def _crests() -> list:
    """The seat, in its frame (origin at the crests' top, on the axis): the spline's top 3 mm, its top edge chamfered
    0.35 mm at 45 degrees, as four steps (5.2, 5.43, 5.67 and 5.9 mm across). As plates a chamfer's corners stand
    proud of its cone, past the horn's play round the spline (0.015 mm then; run c3b: horns hung 0.6 mm high on them)."""
    r, ch, steps = SPLINE_D / 2, CREST_CHAMFER, 3
    surface = {"passes": ("servo",), "friction": (0.3, 0.3)}  # it goes down into the spline's fixed part
    face = 2 * (r - ch) / math.sqrt(2.0) - 0.0001  # a square inside the flat top: what the horn's core rests on (box on box), 2 mm deep
    shapes = [cylinder(r, SEAT_H - ch, pos=(0.0, 0.0, -ch - (SEAT_H - ch) / 2), **surface), box((face, face, 0.002), pos=(0.0, 0.0, -0.001), **surface)]
    for k in range(steps):  # from the top down: each step a third of the chamfer's height, its radius where the cone is at the step's top;
        top, high = -k * ch / steps - (FACE_PROUD if k == 0 else 0.0), ch / steps - (FACE_PROUD if k == 0 else 0.0)  # the top step's face 0.05 mm under the square's, so that the core touches the square alone
        shapes.append(cylinder(r - ch + k * ch / steps, high, pos=(0.0, 0.0, top - high / 2), **surface))
    return shapes


body("seat", _crests(), pos=(*SHAFT_AT, TOP + CRESTS), mass=SEAT_MASS, color=BRASS)
_k = PRESS / SEAT_GIVE
slide("down", "seat", None, axis=(0.0, 0.0, 1.0), limits=(-TRAVEL, LIFT_UP), spring=_k, damping=SEAT_DAMPING_S * _k)
body("turn", [box((0.002, 0.002, 0.0005), pos=(0.0, 0.0, -0.0015), passes=("servo", "seat", "horn"))], pos=(*SHAFT_AT, TOP + CRESTS), mass=TURN_MASS, color=BRASS)
hinge("about", "turn", "seat", anchor=(*SHAFT_AT, TOP + CRESTS), axis=(0.0, 0.0, 1.0), limits=(-math.pi, math.pi), spring=TURN_K, damping=0.01 * TURN_K)
weld("on_shaft", "horn", "turn", made=False)
OBJECTS = list(mk.MECH.bodies.values())

# --- the rules ---
ON_SHAFT, TILT_MOST = 0.0006, math.radians(8.0)  # the hub's axis this near the shaft's and this near upright: the horn is on the shaft (resting on the crests it leans about 4 degrees towards its arm in the guide's play: run c3c)
ON_MARK, ON_CRESTS = math.radians(3.0), 0.0005  # resting: the arm within 3 degrees of the mark, the mouth within 0.5 mm of the crests' place
ENGAGED, FLUSH = 0.0005, 0.003  # m down from the crests: engaged, and flush (0.5 mm or less proud)
RESTING, CLEAR = 0.02, 0.03  # m/s under which the horn is at rest (its hub: welded to the crests it jitters in turn by 1 rad/s or so without moving, run c3q); m both grippers keep from it when they have let go
SIDES = ("left", "right")
STATE: dict = {}
HORN_BOXES = [(np.array([-HUB_D / 2, -HUB_D / 2, 0.0]), np.array([HUB_D / 2, HUB_D / 2, HUB_H])), (np.array([0.0, -ARM_W / 2, HUB_H - ARM_T]), np.array([ARM_L, ARM_W / 2, HUB_H]))]


def begin(seed: int | None = None) -> dict:
    """A new trial's state (no simulator needed)."""
    STATE.clear()
    STATE.update(seed=seed, rest=None, cap=None, weightless=False, engaged=False, tooth=None, deepest=0.0, captured_at=None, engaged_at=None, freed=0, turn_rest=None)
    mk.show("turn", 0, False)  # inside the crests
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)
    if mk.MECH.environment is not None:
        # The simulator keeps a spring's rest through a reset (shield_seat, recording c1e): put it back at the crests,
        # and the turn free.
        STATE["rest"], STATE["cap"] = 0.0, BLOCK
        mk.set_spring("down", rest=0.0, most=BLOCK)
        mk.set_spring("about", rest=0.0, most=0.0)


def horn_frame() -> tuple[np.ndarray, np.ndarray]:
    return mk.pose("horn")


def arm_angle() -> float:
    """The horn's arm from the mark's direction, about the shaft's axis seen from above (rad, -pi to pi)."""
    _, rotation = horn_frame()
    arm = rotation[:, 0]
    return float(math.atan2(MARK[0] * arm[1] - MARK[1] * arm[0], MARK[0] * arm[0] + MARK[1] * arm[1]))


def nearest_tooth(angle: float) -> tuple[int, float]:
    """(the tooth position nearest ``angle``, counted from the mark's; how far ``angle`` is from it, rad)."""
    k = int(round(angle / TOOTH))
    return k % TEETH, angle - k * TOOTH


def down() -> float:
    """How far the hub's mouth stands under its place on the crests (m; 3.5 mm seated)."""
    position, _ = horn_frame()
    return float(TOP + REST_Z - position[2])


def tilt() -> float:
    _, rotation = horn_frame()
    return float(math.acos(min(1.0, max(-1.0, float(rotation[2, 2])))))


def off_axis() -> float:
    """How far the hub's axis is from the shaft's, at the hub's mouth, seen from above (m)."""
    position, _ = horn_frame()
    return float(np.linalg.norm(position[:2] - SHAFT_AT))


def on_shaft() -> bool:
    return off_axis() <= ON_SHAFT and tilt() <= TILT_MOST and -ON_CRESTS <= down() <= TRAVEL + ON_CRESTS


def captured() -> bool:
    """Whether the horn rests on the crests, held there by its weld to the turn."""
    return not mk.is_cut("on_shaft")


def engaged() -> bool:
    """Whether the horn's splines have met the shaft's: it is down 0.2 mm or more and locked in turn."""
    return bool(STATE.get("engaged"))


def in_window() -> bool:
    return on_shaft() and abs(nearest_tooth(arm_angle())[1]) <= WINDOW


def sits_on_crests() -> bool:
    """Whether the horn sits on the crests within its play, held by a hand or not: a hand that holds it there jitters it
    at 0.1 m/s or so without moving it (run c3t), and pressed it would sink into the crests if contacts alone held it."""
    return off_axis() <= CAPTURE[0] and tilt() <= CAPTURE[1] and abs(down() + mk.joint("down")) <= CAPTURE[2]


def push_horn(force=(0.0, 0.0, 0.0), torque=(0.0, 0.0, 0.0)) -> None:
    """A force and torque on the horn (world frame, at its centre of mass) for a probe, on top of the push that carries
    its weight over 4 g while it is welded to the turn: a push on the horn replaces the one before."""
    extra = (HELD_MASS - HORN_MASS) * GRAVITY if captured() else 0.0
    mk.push("horn", force=(force[0], force[1], force[2] + extra), torque=tuple(torque))


def capture() -> None:
    """Weld the horn to the turn as it stands: from now on it weighs HELD_MASS for the solver, the weight over 4 g carried."""
    mk.make("on_shaft")
    mk.set_mass("horn", HELD_MASS)
    push_horn()
    STATE["captured_at"] = round(mk.now(), 2)


def release() -> None:
    """Cut the horn from the turn: free, it weighs 4 g again."""
    mk.cut("on_shaft")
    mk.set_mass("horn", HORN_MASS)
    mk.push("horn")
    STATE["freed"] += 1


def hold_turn() -> None:
    """The free turn's drag: a capped spring whose rest follows the turn (a dry friction of TURN_DRAG)."""
    angle = mk.joint("about")
    if STATE["turn_rest"] is None or abs(angle - STATE["turn_rest"]) > 0.0001:
        mk.set_spring("about", rest=angle, most=TURN_DRAG)
        STATE["turn_rest"] = angle


def hold_seat() -> None:
    """The crests and the turn, once per control step (in the simulator only)."""
    if STATE["rest"] is None:  # the very first trial of a process has no reset before it
        STATE["rest"], STATE["cap"] = 0.0, None
    travel, rest = mk.joint("down"), STATE["rest"]
    beyond = SEAT_GIVE + SEAT_SLACK
    if not engaged():
        hold_turn()
    if not captured():
        rest, cap = 0.0, BLOCK
        if sits_on_crests():  # put on the shaft's axis, upright and on the crests' face, and welded there in the same step: welded a step
            # after it was put there, a horn held by the hand was welded where the fingers had pushed it back, 0.07 mm off the axis and
            # 0.12 mm over the face (recording c3z)
            _, rotation = horn_frame()
            arm = rotation[:, 0]
            mk.write_pose("horn", (*SHAFT_AT, TOP + REST_Z + travel), mk.yaw_quat(math.atan2(arm[1], arm[0])))
            capture()  # make() joins the two as written
    elif not engaged():
        if travel > LIFTED:  # lifted off the crests
            release()
            rest, cap = 0.0, BLOCK
        else:
            down_cap = PRESS if in_window() else BLOCK
            if down_cap == PRESS and travel < rest - beyond:
                rest = travel + SEAT_GIVE
            if down_cap == BLOCK:
                rest = 0.0
            if down_cap == PRESS and travel <= -ENGAGE:
                STATE["engaged"] = True
                STATE["tooth"] = nearest_tooth(arm_angle())[0]
                STATE["engaged_at"] = round(mk.now(), 2)
                mk.set_spring("about", rest=mk.joint("about"), most=LOCK)
            cap = LIFT if travel > rest else down_cap
    else:
        if travel >= -FREE_AT:  # pulled back up to the crests: free to turn again
            STATE["engaged"] = False
            STATE["turn_rest"] = None  # the drag takes over from the lock at the next step
            rest, cap = 0.0, BLOCK
            if travel > LIFTED:  # pulled through past the crests within this step (a yank): it comes off now, not a step later
                release()
        elif travel <= -TRAVEL + SEAT_HOME:  # home: held on the bottom stop; pulled up with more than 10 N it comes up
            rest, cap = -TRAVEL - PIN, PULL_OFF
        else:
            if rest > 0.0 or rest < -TRAVEL:  # from a pinned rest: follow the seat from where it is
                rest = travel
            if travel > rest + beyond:
                rest = travel - SEAT_GIVE
            elif travel < rest - beyond:
                rest = travel + SEAT_GIVE
            cap = PULL_OFF if travel > rest else PRESS
    if rest != STATE["rest"] or cap != STATE["cap"]:
        mk.set_spring("down", rest=rest if rest != STATE["rest"] else None, most=cap if cap != STATE["cap"] else None)
    STATE["rest"], STATE["cap"] = rest, cap


@mk.on_step
def step() -> None:
    if mk.MECH.environment is not None:
        if not STATE["weightless"]:  # the seat's and the turn's own weights are carried for them from the first step on
            mk.push("seat", force=(0.0, 0.0, SEAT_MASS * GRAVITY))
            mk.push("turn", force=(0.0, 0.0, TURN_MASS * GRAVITY))
            STATE["weightless"] = True
        hold_seat()
    if on_shaft():
        STATE["deepest"] = max(STATE["deepest"], down())


def to_horn(point) -> float:
    """How far a point is from the horn's hub and arm (m; 0 inside)."""
    position, rotation = horn_frame()
    local = rotation.T @ (np.asarray(point, dtype=float) - position)
    return min(float(np.linalg.norm(np.maximum(np.maximum(low - local, local - high), 0.0))) for low, high in HORN_BOXES)


def let_go() -> bool:
    return all(to_horn(mk.gripper(side)[0]) >= CLEAR for side in SIDES)


def at_rest() -> bool:
    linear, _ = mk.velocity("horn")
    return float(np.linalg.norm(linear)) < RESTING


def level() -> float:
    """The score without the release: see ``score``."""
    if not on_shaft():
        return 0.0
    angle, deep = arm_angle(), down()
    if engaged():  # the splines have met
        if abs(angle) > WINDOW:
            return 0.0  # seated on another tooth
        if deep >= FLUSH:
            return 0.75
        if deep >= ENGAGED:
            return 0.5
    return 0.25 if abs(angle) <= ON_MARK and abs(deep) <= ON_CRESTS else 0.0


def score(objects) -> float:
    """By where the horn is at the end. On the shaft (its axis within 0.6 mm of the shaft's, upright within 8 degrees),
    engaged (the splines have met: 0.2 mm or more down and locked in turn) with the arm more than 2 degrees from the mark: 0 (seated on
    another tooth). Resting on the crests with the arm within 3 degrees of the mark: a quarter. Engaged on the mark's
    tooth, 0.5 mm or more down: a half. Flush, 3 mm or more down: three quarters; and at rest with both grippers 3 cm
    or more from the horn: 1.0. Anywhere else: 0."""
    found = level()
    if found == 0.75 and let_go() and at_rest():
        return 1.0
    return found


def success(objects) -> bool:
    return score(objects) == 1.0


def progress() -> str:
    return (f"horn: {1000 * off_axis():.2f} mm off the shaft, tilted {math.degrees(tilt()):.1f} deg, {1000 * down():.2f} mm down, arm {math.degrees(arm_angle()):.1f} deg from the mark"
            f"{', engaged' if engaged() else (', on the crests' if captured() else '')}; level {score(None):.2f}")  # fmt: skip


def report() -> dict:
    """For logs and probes: the numbers behind the score."""
    out: dict = {"deepest on the shaft (mm)": round(1000 * STATE.get("deepest", 0.0), 3), "on the crests at (s)": STATE.get("captured_at"), "engaged at (s)": STATE.get("engaged_at"),
                 "tooth engaged (from the mark's)": STATE.get("tooth"), "lifted off the crests": STATE.get("freed", 0)}  # fmt: skip
    if mk.MECH.poses:
        out.update({"off the shaft (mm)": round(1000 * off_axis(), 3), "tilt (deg)": round(math.degrees(tilt()), 2), "down (mm)": round(1000 * down(), 3),
                    "arm from the mark (deg)": round(math.degrees(arm_angle()), 2), "on the crests": captured(), "engaged": engaged()})  # fmt: skip
        if mk.MECH.grippers:
            out.update({"at rest": at_rest(), "let go": let_go(), "score": score(None)})
    return out
