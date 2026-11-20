"""A small camping stove of the folding kind is screwed onto a gas cartridge that stands free on the table in
front of the right arm; its three pot supports are folded up, so it is a column about 4 cm across whose top is
14.5 cm above the table. A wire handle on a short spindle sticks out of the stove's base: the flame control, which
is closed. In front of the left arm stands a steel pot, 6 cm across and 7 cm tall, with 2 cm of soapy water, and
in it a brush, 15 cm long with a 12 mm round head, leaning on the rim with its handle towards the arms.

The stove may be screwed fully home on the cartridge or a third of a turn short of home; the two look the same.
Wet bristles brushed along the line where the stove's base meets the cartridge show it: if the stove is not home,
froth builds up from there over the cartridge's shoulder. Screwing the stove down is a clockwise turn seen from
above. The cartridge is not fixed: it turns and slides on the table unless something holds it.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as mk  # noqa: E402
from mechkit import body, box, cylinder, hinge, sphere  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
INSTRUCTION = (
    "Check the stove's connection to its cartridge with the soapy water and the brush. If bubbles form, screw the "
    "stove down tight and check again. Put the brush back in the pot."
)
RUN_LIMIT_STEPS = 2400  # 120 s at 20 Hz
TOP = TABLE_TOP[SCENE]

STEEL, BRASS, RED, WHITE, NAVY, WOOD, CREAM, WATER = (0.74, 0.76, 0.79), (0.62, 0.52, 0.30), (0.78, 0.20, 0.14), (0.96, 0.96, 0.95), (0.10, 0.16, 0.40), (0.72, 0.52, 0.30), (0.93, 0.90, 0.80), (0.80, 0.88, 0.93)  # fmt: skip

# --- the cartridge: a threaded-valve gas cartridge of the 100 g size, 90 mm across and 70 mm tall, 200 g, standing free ---
CART_XY, CART_R, CART_H, CART_MASS = (0.22, -0.08), 0.045, 0.070, 0.180  # 200 g with its print sleeve
RIM_R, RIM_H = 0.038, 0.003  # it stands on a rim, as a real one does on the edge of its dished base
CUP_R, CUP_H = 0.017, 0.008  # the valve cup in the middle of the top; the shoulder around it is 62 mm up
CART_CENTRE = (*CART_XY, TOP + CART_H / 2)
body(
    "cartridge",
    [
        # The rim it stands on slides easily on a table: taken as 0.2 against whatever it touches (the smaller value
        # wins), its grip on the table is 0.2 x 2.7 N x 38 mm = 0.02 N m, well under the thread's friction, so the
        # cartridge turns with the stove unless it is held.
        cylinder(RIM_R, RIM_H, pos=(0.0, 0.0, -CART_H / 2 + RIM_H / 2), color=STEEL, friction=(0.2, 0.15), friction_combine="min"),
        cylinder(CART_R, CART_H - RIM_H - CUP_H, pos=(0.0, 0.0, (RIM_H - CUP_H) / 2)),
        cylinder(CUP_R, CUP_H, pos=(0.0, 0.0, CART_H / 2 - CUP_H / 2), color=STEEL),
    ],
    pos=CART_CENTRE,
    mass=CART_MASS,
    color=RED,
    friction=(0.6, 0.5),  # painted steel under a rubber finger pad: it can be held by pressing on it
)
# Its print: three white strips with a dark block, on a sleeve that sits on the cartridge at an angle drawn from the
# seed (a core inside the cartridge carries the strips; 1 N m of friction keeps it where it was put). The print of
# a real cartridge is not registered to its thread either, so it tells nothing about the joint.
LABEL_STRIPS = [box((0.0015, 0.0165, 0.034), pos=(0.0458 * math.cos(a), 0.0458 * math.sin(a), 0.0), yaw=a, color=WHITE, solid=False) for a in (-0.36, 0.0, 0.36)]
body("label", [cylinder(0.030, 0.030), *LABEL_STRIPS, box((0.002, 0.012, 0.012), pos=(0.0462, 0.0, 0.004), color=NAVY, solid=False)],
     pos=CART_CENTRE, mass=0.020, color=WHITE)  # fmt: skip
hinge("label", "label", "cartridge", anchor=CART_CENTRE, axis=(0, 0, 1), limits=(-3.1, 3.1), friction=1.0)

# --- the stove: 73 g, a column 75 mm tall: valve body, burner tube, burner head and three folded pot supports ---
JOINT_Z = TOP + CART_H  # the joint line: where the stove's base meets the cartridge's valve, 70 mm up
BASE_R, BASE_H = 0.012, 0.022
HANDLE_ANGLE = math.radians(30)  # where the flame control points at the start, anticlockwise from straight ahead (+x)
SUPPORT_R, STOVE_H, STOVE_MASS = 0.0175, 0.075, 0.073
STOVE_AT = (*CART_XY, JOINT_Z)
SUPPORTS = [box((0.0035, 0.008, 0.044), pos=(SUPPORT_R * math.cos(a), SUPPORT_R * math.sin(a), 0.053), yaw=a, color=STEEL) for a in (math.radians(60), math.pi, math.radians(300))]
body(
    "stove",
    [
        cylinder(BASE_R, BASE_H, pos=(0.0, 0.0, BASE_H / 2), color=BRASS),
        cylinder(0.0035, 0.010, pos=(0.017, 0.0, BASE_H / 2), axis="x", color=BRASS),  # the control's spindle
        cylinder(0.007, 0.024, pos=(0.0, 0.0, 0.034), color=STEEL),
        cylinder(0.015, 0.014, pos=(0.0, 0.0, 0.053), color=STEEL),
        *SUPPORTS,
    ],
    pos=STOVE_AT,
    yaw=HANDLE_ANGLE,
    mass=STOVE_MASS,
    color=STEEL,
)
# The thread: a turning joint about the vertical, 0 = screwed home, positive = unscrewed (anticlockwise from above).
# Dry friction 0.05 N m. The stops at both ends are springs of 10 N m/rad. A real stop is hard; what limits the torque
# on real hardware is the give of the arm and of the grip, which the simulated arm does not have (it follows a turn
# at 80 to 140 N m/rad). The 10 N m/rad stand for that give. How far the stove is turned past home is reported as an
# angle; over-tightening is not scored (README). PhysX limits a hinge to half a turn: the thread ends at 175 degrees.
THREAD_FRICTION, THREAD_STOP, THREAD_END = 0.05, 10.0, math.radians(175)
hinge("thread", "stove", "cartridge", anchor=STOVE_AT, axis=(0, 0, 1), limits=(0.0, THREAD_END), friction=THREAD_FRICTION, stop=THREAD_STOP)

# The flame control: a wire handle 45 mm long that turns about its own length; closed at 0.
RADIAL = (math.cos(HANDLE_ANGLE), math.sin(HANDLE_ANGLE), 0.0)
CONTROL_AT = tuple(STOVE_AT[i] + 0.0445 * RADIAL[i] for i in range(2)) + (JOINT_Z + BASE_H / 2,)
body("control", [box((0.045, 0.012, 0.0025))], pos=CONTROL_AT, yaw=HANDLE_ANGLE, mass=0.006, color=STEEL)
hinge("control", "control", "stove", anchor=CONTROL_AT, axis=RADIAL, limits=(0.0, math.radians(175)), friction=0.02)

# --- the soapy water: a steel pot, 60 mm across and 70 mm tall, with 20 mm of water; 140 g with the water ---
POT_XY, POT_R, POT_H, POT_WALL, POT_FLOOR, WATER_DEPTH = (0.14, 0.16), 0.030, 0.070, 0.002, 0.003, 0.020
POT_INNER = POT_R - POT_WALL
WATER_Z = POT_FLOOR + WATER_DEPTH  # the surface, above the pot's underside
WALLS = 12
WALL_R = POT_R - POT_WALL / 2
POT_AT = (*POT_XY, TOP)  # the pot's frame: the middle of its underside
body(
    "pot",
    [
        cylinder(POT_R, POT_FLOOR, pos=(0.0, 0.0, POT_FLOOR / 2)),
        *[
            box((POT_WALL, 2 * WALL_R * math.tan(math.pi / WALLS) + 0.0015, POT_H - POT_FLOOR), pos=(WALL_R * math.cos(a), WALL_R * math.sin(a), (POT_H + POT_FLOOR) / 2), yaw=a)
            for a in (2 * math.pi * k / WALLS for k in range(WALLS))
        ],
        cylinder(POT_INNER - 0.0005, 0.001, pos=(0.0, 0.0, WATER_Z - 0.0005), color=WATER, solid=False),
    ],
    pos=POT_AT,
    mass=0.140,
    color=STEEL,
    friction=(0.5, 0.4),
)

# --- the brush: 150 mm long, 20 g: a flat wooden handle, a metal band and a 12 mm round head of bristles ---
HANDLE_L, FERRULE_L, HEAD_L, HEAD_R = 0.110, 0.015, 0.025, 0.006
BRUSH_L = HANDLE_L + FERRULE_L + HEAD_L
TIP = BRUSH_L / 2  # along the brush from its middle: the bristles' end; the head starts at TIP - HEAD_L


def brush_lean() -> tuple[float, tuple[float, float]]:
    """How the brush stands in the pot: (its angle below the horizontal, its middle's (x, z) in the pot's frame),
    with the head's lower edge on the floor, its upper edge 1 mm from the far wall and the handle's underside on
    the near rim's inner edge. In the plane of the lean, with d = (cos b, -sin b) along the brush and
    n = (sin b, cos b) across it: the head's end is E = (inner - 1 mm - r sin b, floor + r cos b), the middle is
    P = E - TIP d, and the rim corner R = (-inner, height) lies half the handle's thickness under the axis:
    (R - P) . n = -3 mm. Bisection on b: 54 degrees."""
    half = 0.003

    def middle(b: float) -> tuple[float, float]:
        end = (POT_INNER - 0.001 - HEAD_R * math.sin(b), POT_FLOOR + HEAD_R * math.cos(b))
        return end[0] - TIP * math.cos(b), end[1] + TIP * math.sin(b)

    def gap(b: float) -> float:
        x, z = middle(b)
        return (-POT_INNER - x) * math.sin(b) + (POT_H - z) * math.cos(b) + half

    lo, hi = math.radians(20), math.radians(85)  # gap(lo) > 0 > gap(hi)
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if gap(mid) > 0 else (lo, mid)
    return (lo + hi) / 2, middle((lo + hi) / 2)


LEAN, (LEAN_X, LEAN_Z) = brush_lean()
BRUSH_AT = (POT_XY[0] + LEAN_X, POT_XY[1], TOP + LEAN_Z + 0.001)  # 1 mm up: it settles onto floor and rim
# The bristles are solid against the pot and the table, so the brush stands in the pot, but go through the stove, the
# cartridge and the control: rigid bristles would shove a 270 g assembly that soft ones only stroke.
body(
    "brush",
    [
        box((HANDLE_L, 0.010, 0.006), pos=(-BRUSH_L / 2 + HANDLE_L / 2, 0.0, 0.0), color=WOOD),
        cylinder(0.0065, FERRULE_L, pos=(-BRUSH_L / 2 + HANDLE_L + FERRULE_L / 2, 0.0, 0.0), axis="x", color=STEEL),
        cylinder(HEAD_R, HEAD_L, pos=(TIP - HEAD_L / 2, 0.0, 0.0), axis="x", color=CREAM, passes=("stove", "cartridge", "control", "label")),
    ],
    pos=BRUSH_AT,
    quat=(0.0, math.sin(LEAN / 2), 0.0, math.cos(LEAN / 2)),  # about +y: the bristle end points away from the arms and down
    mass=0.020,
    color=WOOD,
    friction=(0.6, 0.5),
)

# --- the froth: white balls 8 to 14 mm across that the task shows on the cartridge's shoulder; parked under the table ---
# Leak-test solution on a leaking joint gives a growing mass of froth. It lies on the red shoulder from the valve cup
# out to 35.5 mm from the axis, on the side that was wetted: there the cameras see it beside the stove (balls of 4 to
# 8 mm at the joint line itself were a one-pixel fringe in the top camera, run n2i). Every third ball, 8 to 10 mm
# across, lies against the valve cup and stands 0 to 2 mm above its top; the others, 13 or 14 mm, lie at the outer edge.
BUBBLES, BUBBLE_EVERY, BUBBLE_LIFE = 17, 0.3, 5.0  # 17 at one every 0.3 s last 5 s each
FROTH_EDGE = 0.0355  # m from the axis
FROTH_GLOW = 0.5  # froth scatters light: drawn with half its white as light of its own, it reads as white in the stove's shadow too
BUBBLE_INNER = [i % 3 == 0 for i in range(BUBBLES)]
BUBBLE_R = [(0.004, 0.0045, 0.005)[(i // 3) % 3] if BUBBLE_INNER[i] else (0.006, 0.0065, 0.007)[i % 3] for i in range(BUBBLES)]
PARK = [(0.06 + 0.02 * i, 0.50, TOP - 0.20) for i in range(BUBBLES)]


def froth_place(ball: int, stretch: int) -> np.ndarray:
    """Where ball ``ball`` lies when it forms on stretch ``stretch`` of the joint line, in the cartridge's frame (its
    middle): on the shoulder, on that stretch's side of the axis."""
    angle, r = 2 * math.pi * stretch / SECTORS, BUBBLE_R[ball]
    out = CUP_R + r if BUBBLE_INNER[ball] else FROTH_EDGE - r
    return np.array([out * math.cos(angle), out * math.sin(angle), CART_H / 2 - CUP_H + r])


for i in range(BUBBLES):
    body(f"bubble{i:02d}", [sphere(BUBBLE_R[i], solid=False, glow=FROTH_GLOW)], pos=PARK[i], kinematic=True, color=WHITE)

OBJECTS = list(mk.MECH.bodies.values())

# --- the rules ---
LOOSE = math.radians(120)  # the stove's start when it is not home
LEAK = math.radians(30)  # the joint leaks when the stove is further than this from home
HOME = math.radians(10)  # "home": within this of the stop
TURNED = math.radians(5)  # the stove has been turned when it is this far from where it last was
UNSCREWED = math.radians(170)  # the thread's end: the stove has been unscrewed
CONTROL_OPEN = math.radians(20)
TIPPED, UPRIGHT, SPILL = math.radians(45), math.radians(10), math.radians(45)
STAYED, CLEAR, RELEASED = 0.03, 0.05, 0.04  # m: the cartridge's place, the grippers' distance from stove and cartridge, and from the brush
WET_S, FRESH_S = 20.0, 10.0  # the brush stays wet this long after leaving the water; a wetted stretch of the joint counts this long
SECTORS = 36  # the joint line in stretches of 10 degrees
TOUCH = 0.003  # m: how far from the head's surface a point of the joint line still gets wet (bristles splay)
DIP_BACK = 0.008  # m from the bristles' end: this point must be under water for the brush to be wet
RING = np.array([[BASE_R * math.cos(2 * math.pi * k / SECTORS), BASE_R * math.sin(2 * math.pi * k / SECTORS), CART_H / 2] for k in range(SECTORS)])
SIDES = ("left", "right")
STATE: dict = {}


def is_loose(seed: int | None) -> bool:
    """The hidden state: bit 16 of the trial's seed (no seed: not home). inspect-robots derives a trial's seed as a
    CRC-32 of "eval seed:scene seed:epoch"; epochs 2k and 2k+1 differ in the last character's lowest bit, which
    flips CRC bit 16, so they always get opposite cases: any even number of trials is balanced."""
    return True if seed is None else bool((seed >> 16) & 1)


def begin(seed: int | None) -> dict:
    """A new trial's state (no simulator needed)."""
    loose = is_loose(seed)
    STATE.clear()
    STATE.update(
        seed=seed,
        loose=loose,
        label=float(np.random.default_rng(0 if seed is None else seed).uniform(-3.0, 3.0)),  # where the print sits on the cartridge
        zero=None,  # why the trial scores 0, once it does
        wet_until=-math.inf,
        spilled=False,
        sector_t=np.full(SECTORS, -math.inf),  # when each stretch of the joint line was last touched by wet bristles
        tests=0,
        in_test=False,
        last_test_t=None,
        last_turn_t=-math.inf,
        turn_ref=LOOSE if loose else 0.0,
        past_home=0.0,  # rad: the furthest the stove has been turned past home, against the stop's spring
        bubbles=[],  # (ball, born, stretch) of the ones on show
        out=set(),  # balls not parked
        next_bubble_t=0.0,
        ball=0,
        stretch=0,
    )
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)
    # The cartridge is turned back under the stove, not the stove on it: stove and control look the same in both cases.
    mk.write_pose("cartridge", CART_CENTRE, mk.yaw_quat(-LOOSE if STATE["loose"] else 0.0))
    mk.set_joint("label", STATE["label"])
    for i in range(BUBBLES):
        mk.write_pose(f"bubble{i:02d}", PARK[i])


def point_segment(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    """Distance from point p to the segment a-b."""
    ab = b - a
    t = float(np.clip((p - a) @ ab / max(float(ab @ ab), 1e-18), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def segment_segment(a0: np.ndarray, a1: np.ndarray, b0: np.ndarray, b1: np.ndarray) -> float:
    """Shortest distance between the segments a0-a1 and b0-b1."""
    u, v, w = a1 - a0, b1 - b0, a0 - b0
    a, b, c, d, e = float(u @ u), float(u @ v), float(v @ v), float(u @ w), float(v @ w)
    den = a * c - b * b
    s = float(np.clip((b * e - c * d) / den, 0.0, 1.0)) if den > 1e-18 else 0.0
    t = (b * s + e) / c if c > 1e-18 else 0.0
    if t < 0.0:
        t, s = 0.0, float(np.clip(-d / a, 0.0, 1.0)) if a > 1e-18 else 0.0
    elif t > 1.0:
        t, s = 1.0, float(np.clip((b - d) / a, 0.0, 1.0)) if a > 1e-18 else 0.0
    return float(np.linalg.norm(w + s * u - t * v))


def tilt(rotation: np.ndarray) -> float:
    """Angle between a body's own vertical and the world's."""
    return math.acos(float(np.clip(rotation[2, 2], -1.0, 1.0)))


def brush_point(along: float) -> np.ndarray:
    position, rotation = mk.pose("brush")
    return position + rotation @ np.array([along, 0.0, 0.0])


def in_pot(point: np.ndarray) -> np.ndarray:
    position, rotation = mk.pose("pot")
    return rotation.T @ (point - position)


@mk.on_step
def step() -> None:
    """Once per control step: the rules that end the trial at 0, how far the stove stands past home, whether it
    was turned, whether the brush is wet, which stretches of the joint line its bristles wet, and the froth."""
    now = mk.now()
    cart_p, cart_r = mk.pose("cartridge")
    thread = mk.joint("thread")
    if STATE["zero"] is None:
        if mk.joint("control") > CONTROL_OPEN:
            STATE["zero"] = "the flame control was opened"
        elif tilt(cart_r) > TIPPED or cart_p[2] < TOP - 0.03:
            STATE["zero"] = "the cartridge fell over or off the table"
        elif thread > UNSCREWED:
            STATE["zero"] = "the stove was unscrewed"
    STATE["past_home"] = max(STATE["past_home"], -thread)  # for the record: it is not scored
    on_thread = max(thread, 0.0)  # past home it is the stop's spring that gives (the arm's give, see THREAD_STOP): the thread has not moved
    if abs(on_thread - STATE["turn_ref"]) > TURNED:
        STATE["last_turn_t"], STATE["turn_ref"] = now, on_thread

    STATE["spilled"] = STATE["spilled"] or tilt(mk.pose("pot")[1]) > SPILL
    dip = in_pot(brush_point(TIP - DIP_BACK))
    if not STATE["spilled"] and math.hypot(dip[0], dip[1]) < POT_INNER and -0.01 < dip[2] < WATER_Z:
        STATE["wet_until"] = now + WET_S  # so it is wet for WET_S after it last left the water
    touched = np.zeros(SECTORS, dtype=bool)
    if now <= STATE["wet_until"]:
        head0, head1 = brush_point(TIP - HEAD_L), brush_point(TIP)
        centre = cart_p + cart_r @ np.array([0.0, 0.0, CART_H / 2])
        # Bristles do not go through metal: a head whose axis runs inside the stove's base wets nothing.
        if segment_segment(head0, head1, centre, centre + cart_r @ np.array([0.0, 0.0, BASE_H])) >= BASE_R - 0.002:
            ring = cart_p + RING @ cart_r.T
            touched = np.array([point_segment(point, head0, head1) <= HEAD_R + TOUCH for point in ring])
            STATE["sector_t"][touched] = now
    fresh = now - STATE["sector_t"] <= FRESH_S
    testing = int(fresh.sum()) >= SECTORS // 2 and bool(touched.any())  # half the line wet, and the brush on it now
    if testing:
        STATE["tests"] += 0 if STATE["in_test"] else 1
        STATE["last_test_t"] = now
    STATE["in_test"] = testing

    STATE["bubbles"] = [b for b in STATE["bubbles"] if now - b[1] < BUBBLE_LIFE]
    wet_stretches = np.flatnonzero(fresh)
    if thread > LEAK and len(wet_stretches) and now >= STATE["next_bubble_t"]:
        ball, STATE["ball"] = STATE["ball"] % BUBBLES, STATE["ball"] + 1
        spot = (STATE["stretch"] * 0.6180339887) % 1.0  # golden-ratio steps: spread over the wet part whatever its length
        stretch, STATE["stretch"] = int(wet_stretches[int(spot * len(wet_stretches))]), STATE["stretch"] + 1
        STATE["bubbles"] = [b for b in STATE["bubbles"] if b[0] != ball] + [(ball, now, stretch)]
        STATE["next_bubble_t"] = now + BUBBLE_EVERY
    if mk.MECH.environment is not None:  # the balls ride on the cartridge; the ones whose time is up go back under the table
        shown = set()
        for ball, _, stretch in STATE["bubbles"]:
            mk.write_pose(f"bubble{ball:02d}", cart_p + cart_r @ froth_place(ball, stretch))
            shown.add(ball)
        for ball in STATE["out"] - shown:
            mk.write_pose(f"bubble{ball:02d}", PARK[ball])
        STATE["out"] = shown


def cylinder_distance(point: np.ndarray, radius: float, low: float, high: float) -> float:
    """Distance from a point (in the cylinder's frame, its axis along z) to a solid cylinder between two heights."""
    return math.hypot(max(math.hypot(point[0], point[1]) - radius, 0.0), max(low - point[2], point[2] - high, 0.0))


def gripper_gap(side: str) -> float:
    """How far a gripper's grasp point is from the cartridge, the stove's column and the control's handle (m)."""
    point = mk.gripper(side)[0]
    cart_p, cart_r = mk.pose("cartridge")
    stove_p, stove_r = mk.pose("stove")
    control_p, control_r = mk.pose("control")
    reach = control_r @ np.array([0.0225, 0.0, 0.0])
    return min(
        cylinder_distance(cart_r.T @ (point - cart_p), CART_R, -CART_H / 2, CART_H / 2),
        cylinder_distance(stove_r.T @ (point - stove_p), SUPPORT_R + 0.002, 0.0, STOVE_H),
        point_segment(point, control_p - reach, control_p + reach) - 0.006,
    )


def brush_back() -> bool:
    """The bristles' end inside the pot below its rim, the pot standing, the brush at rest and let go."""
    end = in_pot(brush_point(TIP))
    inside = math.hypot(end[0], end[1]) < POT_INNER and -0.005 < end[2] < POT_H and tilt(mk.pose("pot")[1]) < UPRIGHT
    resting = float(np.linalg.norm(mk.velocity("brush")[0])) < 0.02
    handle = (brush_point(-BRUSH_L / 2), brush_point(-BRUSH_L / 2 + HANDLE_L))
    return inside and resting and all(point_segment(mk.gripper(side)[0], *handle) >= RELEASED for side in SIDES)


def cartridge_stayed() -> bool:
    cart_p, cart_r = mk.pose("cartridge")
    return tilt(cart_r) < UPRIGHT and float(np.linalg.norm(cart_p[:2] - np.array(CART_XY))) <= STAYED


def tidy() -> bool:
    return brush_back() and cartridge_stayed() and all(gripper_gap(side) >= CLEAR for side in SIDES)


def score(objects) -> float:
    """0 if a rule that ends the trial was broken; 0.25 a valid test (wet bristles on half the joint line); 0.5 and
    the stove is home; 0.75 and the last valid test came after the stove was last turned; 1.0 and the brush is
    back in the pot, both grippers are 5 cm clear and the cartridge stands within 3 cm of its place."""
    if STATE["zero"] is not None or STATE["last_test_t"] is None:
        return 0.0
    if mk.joint("thread") > HOME:
        return 0.25
    if not STATE["last_test_t"] > STATE["last_turn_t"]:
        return 0.5
    return 1.0 if tidy() else 0.75


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    """What the rules see now: for the probe, the tests and the operator's log."""
    now = mk.now()
    return {
        "loose at the start": STATE["loose"],
        "thread (deg)": round(math.degrees(mk.joint("thread")), 1),
        "control (deg)": round(math.degrees(mk.joint("control")), 1),
        "label (deg)": round(math.degrees(mk.joint("label")), 1),
        "wet": bool(now <= STATE["wet_until"]),
        "joint line wet (deg)": int((now - STATE["sector_t"] <= FRESH_S).sum()) * 360 // SECTORS,
        "valid tests": STATE["tests"],
        "last test (s)": STATE["last_test_t"],
        "stove last turned (s)": None if STATE["last_turn_t"] == -math.inf else STATE["last_turn_t"],
        "bubbles": len(STATE["bubbles"]),
        "most past home (deg)": round(math.degrees(STATE["past_home"]), 1),
        "zero": STATE["zero"],
        "cartridge moved (mm)": round(1000 * float(np.linalg.norm(mk.pose("cartridge")[0][:2] - np.array(CART_XY))), 1),
        "brush lean (deg)": round(math.degrees(LEAN), 1),
        "score": score(None),
    }


begin(None)
