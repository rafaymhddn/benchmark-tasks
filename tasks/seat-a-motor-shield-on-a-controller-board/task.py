"""An Arduino Uno R3 is screwed to a carrier plate that is fixed to the bench in front of the right arm. An Arduino
Motor Shield Rev3 rests on the Uno's four header sockets on its own pin tips: every pin stands half a millimetre in
its socket's lead-in and nothing else holds the shield. Pressed down evenly it goes 5.5 mm into the sockets and is
seated; pressed at one end or on one row it tilts, and its pins bend.

How it is built. The shield is one rigid body that can go up and down and tilt about both level axes (a slide and
two hinges at the level of its pin tips; its place over the Uno is given, as the pins in their lead-ins give it). Each of the four
pin strips meets a seat inside its socket: a small body on a vertical slide, under the middle of the strip, that
resists going down with 1.67 N for each of the strip's pins and stays where it was pushed (it comes back 0.15 mm at
most). The shield's pins rest on the four seats and go through the sockets themselves without touching them. So the
shield is held up by nothing but those four resistances; lifted and let go it falls back onto them; and a push that
is not even drives one strip in before another.
"""

from __future__ import annotations

import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as mk  # noqa: E402
from mechkit import body, box, cylinder, hinge, slide  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
INSTRUCTION = "Press the motor shield down onto the controller board until it is fully seated, without bending its pins, then let go."
RUN_LIMIT_STEPS = 600  # 30 s at 20 Hz
TOP = TABLE_TOP[SCENE]
GAUGE = os.environ.get("SHIELD_GAUGE") == "1"  # a check of the arm, not the task: the four seats are springs, and their travel reads the press

PLATE_GREY, TEAL, BLACK, STEEL, TIN, BLUE, WHITE = (0.78, 0.79, 0.80), (0.0, 0.40, 0.44), (0.07, 0.07, 0.08), (0.70, 0.72, 0.74), (0.80, 0.78, 0.70), (0.10, 0.35, 0.75), (0.92, 0.92, 0.90)

# --- the carrier plate, fixed: 100 x 80 x 6 mm, its long edges along x, its middle 28 cm in front of the line of the
# arm bases and 20 cm to the right of the midline ---
CARRIER_XY, CARRIER_X, CARRIER_Y, CARRIER_T = (0.28, -0.20), 0.100, 0.080, 0.006
SPACER_H = 0.005  # the Uno stands on four M3 spacers (5 mm: assumed)

# --- the Uno R3 (Arduino's dimension drawing): 68.58 x 53.34 mm, 1.6 mm thick; the drawing's origin is the corner by
# the USB socket, x along the board's length, y across it. USB end towards the arms. ---
UNO_L, UNO_W, PCB_T = 0.06858, 0.05334, 0.0016
PITCH, SOCKET_W, SOCKET_H = 0.00254, 0.00254, 0.0085
# The four header strips: (name, first and last pin along the board in mm, pins, the row's y in mm)
STRIPS = (("digital_high", 18.796, 41.656, 10, 50.80), ("digital_low", 45.72, 63.50, 8, 50.80), ("power", 27.94, 45.72, 8, 2.54), ("analog", 50.80, 63.50, 6, 2.54))
NAMES = [s[0] for s in STRIPS]
PINS = {s[0]: s[3] for s in STRIPS}
ROWS_APART = 0.04826
UNO_TOP = CARRIER_T + SPACER_H + PCB_T  # the Uno's top face over the bench
SOCKET_TOP = UNO_TOP + SOCKET_H  # 21.1 mm


def on_bench(x_mm: float, y_mm: float) -> tuple[float, float]:
    """A point of the Uno's drawing, in bench coordinates."""
    return CARRIER_XY[0] + (x_mm / 1000 - UNO_L / 2), CARRIER_XY[1] + (y_mm / 1000 - UNO_W / 2)


def strip_place(name: str) -> tuple[float, float, float]:
    """(x, y of a strip's middle on the bench, its length)."""
    _, first, last, pins, row = next(s for s in STRIPS if s[0] == name)
    x, y = on_bench((first + last) / 2, row)
    return x, y, pins * PITCH


_uno = [box((CARRIER_X, CARRIER_Y, CARRIER_T), pos=(0.0, 0.0, CARRIER_T / 2), color=PLATE_GREY)]
for _x, _y in ((14.0, 2.5), (15.2, 50.8), (66.0, 7.6), (66.0, 35.6)):  # the Uno's four mounting holes
    _bx, _by = on_bench(_x, _y)
    _uno.append(cylinder(0.003, SPACER_H, pos=(_bx - CARRIER_XY[0], _by - CARRIER_XY[1], CARRIER_T + SPACER_H / 2), color=WHITE, solid=False))
_uno.append(box((UNO_L, UNO_W, PCB_T), pos=(0.0, 0.0, UNO_TOP - PCB_T / 2), color=TEAL))
for _name in NAMES:
    _sx, _sy, _len = strip_place(_name)
    _uno.append(box((_len, SOCKET_W, SOCKET_H), pos=(_sx - CARRIER_XY[0], _sy - CARRIER_XY[1], UNO_TOP + SOCKET_H / 2), color=BLACK))
# what else stands on an Uno, drawn only (the shield clears all of it): USB socket, power jack, the controller in its socket
_ux, _uy = on_bench(6.0, 38.0)
_uno.append(box((0.016, 0.012, 0.0109), pos=(_ux - CARRIER_XY[0], _uy - CARRIER_XY[1], UNO_TOP + 0.0109 / 2), color=STEEL, solid=False))
_ux, _uy = on_bench(6.5, 7.5)
_uno.append(box((0.014, 0.009, 0.0109), pos=(_ux - CARRIER_XY[0], _uy - CARRIER_XY[1], UNO_TOP + 0.0109 / 2), color=BLACK, solid=False))
_ux, _uy = on_bench(46.0, 14.0)
_uno.append(box((0.036, 0.0075, 0.0075), pos=(_ux - CARRIER_XY[0], _uy - CARRIER_XY[1], UNO_TOP + 0.0075 / 2), color=BLACK, solid=False))
body("carrier", _uno, pos=(*CARRIER_XY, TOP), color=PLATE_GREY)

# --- the motor shield: 68.58 x 53.3 mm, 25 g (to be confirmed on the product page); stacking headers: its own sockets
# on top over each strip, and under it the pins, 0.64 mm square. Pin length under the board 10.5 mm (assumed: a
# stacking header's), of which 6 mm go into a socket (assumed: to be measured). ---
SHIELD_W, SHIELD_MASS = 0.0533, 0.025
PIN_T, PIN_L, MATING = 0.00064, 0.0105, 0.006
LEAD_IN = 0.0005  # how deep the pin tips stand in the sockets at the start: in the lead-ins
TRAVEL = MATING - LEAD_IN  # 5.5 mm from the start to the seat
SHIELD_Z = SOCKET_TOP - LEAD_IN + PIN_L + PCB_T / 2  # the middle of the shield's board over the bench at the start
_shield = [box((UNO_L, SHIELD_W, PCB_T), color=TEAL)]
OWN_SOCKETS, PIN_BARS = {}, {}
for _name in NAMES:
    _sx, _sy, _len = strip_place(_name)
    OWN_SOCKETS[_name] = len(_shield)
    _shield.append(box((_len, SOCKET_W, SOCKET_H), pos=(_sx - CARRIER_XY[0], _sy - CARRIER_XY[1], PCB_T / 2 + SOCKET_H / 2), color=BLACK))
for _name in NAMES:
    _sx, _sy, _len = strip_place(_name)
    PIN_BARS[_name] = len(_shield)  # the strip's pins as one bar: it goes through the Uno's socket without touching it and stands on the seat inside
    _shield.append(box((_len, PIN_T, PIN_L), pos=(_sx - CARRIER_XY[0], _sy - CARRIER_XY[1], -PCB_T / 2 - PIN_L / 2), color=TIN, passes=("carrier",)))
# drawn only: the driver chip, the two screw terminal blocks at the end away from the USB socket, the four white sockets
_shield.append(box((0.020, 0.020, 0.004), pos=(0.004, 0.004, PCB_T / 2 + 0.002), color=BLACK, solid=False))
for _y in (-0.008, 0.008):
    _shield.append(box((0.008, 0.010, 0.010), pos=(UNO_L / 2 - 0.006, _y, PCB_T / 2 + 0.005), color=BLUE, solid=False))
for _x in (-0.026, -0.014):
    for _y in (-0.012, 0.012):
        _shield.append(box((0.008, 0.006, 0.006), pos=(_x, _y, PCB_T / 2 + 0.003), color=WHITE, solid=False))
SHIELD = body("shield", _shield, pos=(*CARRIER_XY, TOP + SHIELD_Z), mass=SHIELD_MASS, color=TEAL)
SHIELD_BOX = (np.array([-UNO_L / 2, -SHIELD_W / 2, -PCB_T / 2]), np.array([UNO_L / 2, SHIELD_W / 2, PCB_T / 2 + SOCKET_H]))  # the board with its own sockets, in the shield's frame
STRIP_TIP = {name: np.array([strip_place(name)[0] - CARRIER_XY[0], strip_place(name)[1] - CARRIER_XY[1], -PCB_T / 2 - PIN_L]) for name in NAMES}  # the middle of a strip's pin tips, in the shield's frame

# --- the seats: one small pad inside each socket strip, under the middle of the strip's pin tips; it resists going
# down with 1.67 N for each pin (Samtec SSW, 0.64 mm square pin: assumed for the Uno's sockets) and stays where it
# is pushed ---
# A pad, not a bar as long as the strip: the pins of a strip resist alike, so together they resist at the strip's
# middle. On a bar a tilted strip stands on its lowest corner, and the strips of the far end then carry a press on
# the near end (recording c1f: pressed at one end with 45 N the shield went down nearly level, 1.3 degrees).
# A pad is slippery: it resists going down and nothing else. On pads that grip, a 25 g shield that nothing pressed
# kept the tilt it came down with (recording c1j: one end 0.35 mm high on four seats at home).
# How a seat resists (see hold_seats): a stiff spring that can give no more than the strip's resistance, and whose
# rest point follows the seat. It holds still under a load below the resistance (yielding up to 0.1 mm) and goes
# down under a load above it, as fast as the load drives it (recording c1g: under nine tenths of its resistance for
# 3 s a seat went 0.09 mm; under eleven tenths it was 5 mm down in a tenth of a second). Let go on the way it comes
# back 0.15 mm at most. Within 0.3 mm of its stop (what the score calls seated) a seat is drawn onto the stop and
# stays: without that, a strip that the hand left 0.2 mm short came back to 0.34 mm and was no longer seated
# (recording c1h, 16:12). The rest follows only a seat that has gone 0.05 mm beyond the spring's give: a 25 g board between a hand and the seats trembles in the simulator, a
# seat is read at its lowest every time, and a rest that followed at once crept (recording c1f: under the hand the
# shield began to go down at 42 N and took 12 s). A seat is half a kilogram and weightless (its weight is taken off
# it from the first step of a trial): a body of a few grams is not held by a spring under a hand's push (10 g seats
# let the shield home under 16 N instead of 53.4, recording c1b). The kit's own dry friction is not used: a loaded
# joint creeps when the hand's small jerks tear it loose (recording c1d: the shield went home under 42 N).
PIN_FORCE, SEAT_MASS, SEAT_GIVE, SEAT_SLACK, SEAT_DAMPING_S, GRAVITY = 1.67, 0.5, 0.0001, 0.00005, 0.01, 9.81
SEAT_HOME = 0.0003  # m from its stop: a seat that near goes home and stays (the tolerance of 'seated')
SEAT_PAD, SEAT_PAD_W = 0.004, 0.003  # m along the strip and across it
RESISTS = {name: PINS[name] * PIN_FORCE for name in NAMES}  # N: 16.7, 13.4, 13.4, 10.0; 53.4 in all
GAUGE_K = 5000.0  # N/m per seat when the scene is a gauge: 20 kN/m under the shield, 110 N at the seat
for _name in NAMES:
    _sx, _sy, _len = strip_place(_name)
    body(f"seat_{_name}", [box((SEAT_PAD, SEAT_PAD_W, 0.002), passes=("carrier",), friction=(0.0, 0.0), friction_combine="min")], pos=(_sx, _sy, TOP + SOCKET_TOP - LEAD_IN - 0.001),
         mass=SEAT_MASS, color=BLACK)  # fmt: skip
    if GAUGE:
        slide(f"in_{_name}", f"seat_{_name}", None, axis=(0.0, 0.0, 1.0), limits=(-TRAVEL, 0.0), spring=GAUGE_K, damping=8.0)
    else:
        _k = RESISTS[_name] / SEAT_GIVE
        slide(f"in_{_name}", f"seat_{_name}", None, axis=(0.0, 0.0, 1.0), limits=(-TRAVEL, 0.0), spring=_k, damping=SEAT_DAMPING_S * _k)

# --- the shield's freedom: up and down, and a tilt about each level axis ---
# The two tilt axes pass through the level of the pin tips, on the board's centre line: it is the pins in their
# sockets that give the shield its place, so that is where a tilting shield turns. With the axes at the middle of
# the board, 11 mm higher, the tips of a tilting shield moved sideways, and off their slippery pads (recording c1l:
# pressed on one row, that row went 10 mm in, 4 mm past its seat).
# The tilt is stopped at 7 degrees either way: that is one row (or one end of the rows) home with the other still on
# its pin tips, and pins that are in their sockets let a real shield tilt no further. Without the stops a press on
# one row, once that row was home, threw the other row 5 mm up out of its sockets, and its pins came down beside
# their seats (recording c1m). The stops are the joints' own hard limits: as springs (40 and 400 N m/rad) they let a
# shield pressed at one end run on to 15 degrees (recordings c1n, c1o). The two bodies between the shield's joints
# are thin plates inside the stack (hidden, touching nothing), 20 g each and weightless like the seats: a stop works
# through them, and as 2 g balls they had a thirty-thousandth of the shield's own inertia.
_others = ("carrier", "shield", *[f"seat_{name}" for name in NAMES])
PIVOT = (*CARRIER_XY, TOP + SOCKET_TOP - LEAD_IN)
TILT_MOST = float(np.radians(7.0))
LINKS, LINK_MASS = ("shield_lift", "shield_roll"), 0.020
body("shield_lift", [box((0.050, 0.040, 0.001), passes=_others)], pos=PIVOT, mass=LINK_MASS, color=TEAL)
body("shield_roll", [box((0.050, 0.040, 0.001), passes=(*_others, "shield_lift"))], pos=PIVOT, mass=LINK_MASS, color=TEAL)
slide("shield_up", "shield_lift", None, axis=(0.0, 0.0, 1.0), limits=(-TRAVEL - 0.001, 0.15))
hinge("shield_about_x", "shield_roll", "shield_lift", anchor=PIVOT, axis=(1.0, 0.0, 0.0), limits=(-TILT_MOST, TILT_MOST))
hinge("shield_about_y", "shield", "shield_roll", anchor=PIVOT, axis=(0.0, 1.0, 0.0), limits=(-TILT_MOST, TILT_MOST))
HIDDEN = (*[(f"seat_{name}", 0) for name in NAMES], ("shield_lift", 0), ("shield_roll", 0))

OBJECTS = list(mk.MECH.bodies.values())

# --- the rules ---
IN_ENOUGH = 0.0015  # m in its socket: a strip that far in has started; and from there a pin bends if another strip lags
BENT_BY = 0.0020  # m another strip stands higher than one that is IN_ENOUGH in: a pin is bent (assumed threshold)
NEAR_SEAT, SEATED = 0.0020, 0.0003  # m from its seat: nearly there, and seated
RESTING, CLEAR = 0.02, 0.03  # m/s under which the shield is at rest; m both grippers keep from it when they have let go
SIDES = ("left", "right")
STATE: dict = {}


def begin(seed: int | None = None) -> dict:
    """A new trial's state (no simulator needed)."""
    STATE.clear()
    STATE.update(seed=seed, bent=None, deepest={name: LEAD_IN for name in NAMES}, most_apart=0.0, weightless=False, held=None)
    for name, shape in HIDDEN:  # the seats sit inside the sockets and the two bodies of the shield's joints inside its board
        mk.show(name, shape, False)
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)
    if mk.MECH.environment is not None and not GAUGE:
        # The simulator keeps a spring's rest through a reset: left where the last trial pushed it, it pulls the
        # seat down again before the first step (recording c1e: the seats were 5 mm down when a trial began).
        STATE["held"] = dict.fromkeys(NAMES, 0.0)
        for name in NAMES:
            mk.set_spring(f"in_{name}", rest=0.0, most=RESISTS[name])


def depths() -> dict:
    """How far each strip's pin tips are in their sockets (m): 0.5 mm at the start, 6 mm seated."""
    position, rotation = mk.pose("shield")
    return {name: float(TOP + SOCKET_TOP - (position + rotation @ STRIP_TIP[name])[2]) for name in NAMES}


def bent_now(deep: dict) -> tuple[str, str] | None:
    """(the strip that is in, the strip that stands too high over it) if a pin bends as the strips stand, else None."""
    for low in NAMES:
        if deep[low] >= IN_ENOUGH:
            for high in NAMES:
                if deep[low] - deep[high] >= BENT_BY:
                    return low, high
    return None


def follow(held: float, travel: float) -> float:
    """Where a seat's spring rests from now on: where it rested, unless the seat has gone more than the give and
    the slack from there; then the give behind the seat. A seat within 0.3 mm of its stop rests at the stop."""
    if travel <= -TRAVEL + SEAT_HOME:
        return -TRAVEL  # as good as home: the spring draws the seat onto its stop and holds it there, nothing springs back
    if abs(travel - held) <= SEAT_GIVE + SEAT_SLACK:
        return held
    return travel + SEAT_GIVE if travel < held else travel - SEAT_GIVE


def hold_seats() -> None:
    """The four seats' resistance, once per control step (in the simulator only)."""
    first = STATE["held"] is None  # the very first trial of a process has no reset before it
    if first:
        STATE["held"] = dict.fromkeys(NAMES, 0.0)
    for name in NAMES:
        rest = follow(STATE["held"][name], mk.joint(f"in_{name}"))
        if first or rest != STATE["held"][name]:
            mk.set_spring(f"in_{name}", rest=rest, most=RESISTS[name] if first else None)
        STATE["held"][name] = rest


def put_seats(travel: float) -> None:
    """Put the four seats ``travel`` (m, down is negative) from their start, their springs resting where they are:
    for a check that sets a state (in the simulator)."""
    if STATE["held"] is None:
        STATE["held"] = dict.fromkeys(NAMES, 0.0)
    for name in NAMES:
        mk.set_joint(f"in_{name}", travel)
        mk.set_spring(f"in_{name}", rest=travel)
        STATE["held"][name] = travel


@mk.on_step
def step() -> None:
    if mk.MECH.environment is not None:
        if not STATE["weightless"]:  # the seats' own weight, and that of the two plates between the shield's joints, is carried for them, from the first step on
            for name in NAMES:
                mk.push(f"seat_{name}", force=(0.0, 0.0, SEAT_MASS * GRAVITY))
            for name in LINKS:
                mk.push(name, force=(0.0, 0.0, LINK_MASS * GRAVITY))
            STATE["weightless"] = True
        if not GAUGE:
            hold_seats()
    if GAUGE:
        return
    deep = depths()
    for name in NAMES:
        STATE["deepest"][name] = max(STATE["deepest"][name], deep[name])
    STATE["most_apart"] = max(STATE["most_apart"], max(deep.values()) - min(deep.values()))
    if STATE["bent"] is None:
        pair = bent_now(deep)
        if pair is not None:
            STATE["bent"] = {"in": pair[0], "high": pair[1], "at (s)": round(mk.now(), 2), "depths (mm)": {name: round(1000 * deep[name], 2) for name in NAMES}}


def to_shield(point) -> float:
    """How far a point is from the shield's board and its own sockets (m; 0 inside)."""
    position, rotation = mk.pose("shield")
    local = rotation.T @ (np.asarray(point, dtype=float) - position)
    return float(np.linalg.norm(np.maximum(np.maximum(SHIELD_BOX[0] - local, local - SHIELD_BOX[1]), 0.0)))


def let_go() -> bool:
    return all(to_shield(mk.gripper(side)[0]) >= CLEAR for side in SIDES)


def at_rest() -> bool:
    linear, angular = mk.velocity("shield")
    return float(np.linalg.norm(linear)) < RESTING and float(np.linalg.norm(angular)) * UNO_L / 2 < RESTING


def score(objects) -> float:
    """0 once a pin has bent. Else by how far the four strips are in at the end: every one 1.5 mm or more a quarter,
    every one within 2 mm of its seat a half, every one seated three quarters; and seated, at rest, with both
    grippers 3 cm or more from the shield, 1.0."""
    if STATE["bent"] is not None:
        return 0.0
    deep = depths()
    left = {name: MATING - deep[name] for name in NAMES}  # what each strip has still to go
    if all(v <= SEATED for v in left.values()):
        return 1.0 if at_rest() and let_go() else 0.75
    if all(v <= NEAR_SEAT for v in left.values()):
        return 0.5
    if all(v >= IN_ENOUGH for v in deep.values()):
        return 0.25
    return 0.0


def success(objects) -> bool:
    return score(objects) == 1.0


def progress() -> str:
    deep = depths()
    return "strips in (mm): " + ", ".join(f"{name} {1000 * deep[name]:.1f}" for name in NAMES) + f"; level {score(None):.2f}" + (f"; BENT: {STATE['bent']}" if STATE["bent"] else "")


def report() -> dict:
    """For logs and probes: the numbers behind the score."""
    out = {"bent": STATE["bent"], "deepest (mm)": {name: round(1000 * v, 2) for name, v in STATE["deepest"].items()},
           "most any two strips were apart in height (mm)": round(1000 * STATE["most_apart"], 2)}  # fmt: skip
    if mk.MECH.poses:
        deep = depths()
        out["in now (mm)"] = {name: round(1000 * deep[name], 2) for name in NAMES}
        if mk.MECH.grippers:
            out.update({"at rest": at_rest(), "let go": let_go(), "score": score(None)})
    return out
