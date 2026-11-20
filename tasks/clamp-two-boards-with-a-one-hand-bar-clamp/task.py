"""Two pine boards lie flat on the bench side by side, their long edges 2 mm apart. In front of them a one-hand bar
clamp lies flat, its jaws opened 14 cm and pointing at the boards, their tips 2 cm short of the boards' near ends. Put
round the two boards and pumped with its trigger, it draws its bar through the grip a few millimetres a stroke and
does not let it back; once its jaws close on the boards, every stroke squeezes them harder, read on a soft pad on the
grip's jaw that gives 1 mm for every 40 N.

How it is built. The grip (pistol grip, head and the grip's jaw) is one free body. The bar slides through the grip's
head on a slide; the bar's jaw is part of the bar. The trigger turns on a pin in the grip, held open by a spring. The
trigger's short arm, above its pin, moves a drive plate along the bar through a link (a slider-crank: both hidden in
the grip's head). The drive plate slides on the bar; its stop on that slide is moved after it every control step
while the trigger opens, so that it slides back along the bar freely and cannot slide forwards: closing, the trigger
draws the bar with it (after 0.1 mm of play, as a drive plate tilts before it bites). The lock plate is the bar's
slide's lower stop, moved after the bar every control step: the bar slides on freely in the closing direction and
cannot go back past where it has been. The gauge is a pad on the grip's jaw on a sprung slide: the clamping force is
read from how far it has given. A bar pushed by hand slides closed freely, but the lock plate holds it only where its
jaw met the boards (where it is, less what the gauge pad has given): let go, the bar goes back that far and the force
of the push goes with it. Only the trigger leaves force on the boards.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as mk  # noqa: E402
from mechkit import body, box, hinge, slide  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
INSTRUCTION = "Clamp the two boards together edge to edge with the bar clamp so that the joint stays closed, then let go."
RUN_LIMIT_STEPS = 1200  # three times the scripted full run, rounded up to a hundred steps, at most 1,200: set once it is recorded
TOP = TABLE_TOP[SCENE]

PINE, PINE_LEFT, BLACK, YELLOW, STEEL, RUBBER = (0.86, 0.72, 0.50), (0.80, 0.64, 0.42), (0.08, 0.08, 0.09), (0.95, 0.75, 0.10), (0.70, 0.72, 0.75), (0.15, 0.15, 0.16)

# --- the boards: two pine boards 200 x 60 x 18 mm, about 110 g each (assumed, to be weighed), lying flat side by side,
# long axes along x, near ends 33 cm in front of the line of the arm bases, their long edges 2 mm apart, the pair's
# middle 6 cm right of the midline ---
BOARD_L, BOARD_W, BOARD_T = 0.200, 0.060, 0.018
BOARD_MASS = 0.110
NEAR_X, PAIR_Y, JOINT_START = 0.330, -0.060, 0.002
BOARD_FRICTION = (0.30, 0.25)  # pine on the laminate bench and on pine: assumed
_board_y = {"board_r": PAIR_Y - (JOINT_START + BOARD_W) / 2, "board_l": PAIR_Y + (JOINT_START + BOARD_W) / 2}
BOARDS = ("board_r", "board_l")  # the right board (towards the right arm) and the left board
for _name, _color in zip(BOARDS, (PINE, PINE_LEFT)):
    body(_name, [box((BOARD_L, BOARD_W, BOARD_T))], pos=(NEAR_X + BOARD_L / 2, _board_y[_name], TOP + BOARD_T / 2), mass=BOARD_MASS, color=_color, friction=BOARD_FRICTION)

# --- the clamp: IRWIN QUICK-GRIP one-handed mini bar clamp, 6 in (sellers' listings of the maker's figures: throat
# 62 mm, opening 152 mm, 140 lb of clamping force; about 0.3 kg in all, assumed, to be weighed). It lies flat on the
# bench, its middle plane at the boards' mid-thickness, the bar along y, the jaws pointing away from the arms (+x)
# with their tips 2 cm short of the boards' near ends; the grip's jaw 5 mm outside the right board's outer edge, the
# bar's jaw 13 mm outside the left board's (opened 14 cm); the handle and the trigger point to the arms. Sizes of the
# parts other than the maker's figures are assumed (to be measured with a caliper) ---
MID_Z = TOP + BOARD_T / 2
X_BAR = NEAR_X - 0.020 - 0.062  # the bar's centre line: the jaws' tips 2 cm short of the boards, 62 mm (the throat) from the bar
TIP_X = NEAR_X - 0.020
BAR_W, BAR_T, BAR_FROM, BAR_TO = 0.015, 0.005, -0.230, 0.030  # m: across (x), thick (z), its tail's end and its jaw's end (y)
JAW_T, JAW_H, JAW_ROOT = 0.012, 0.018, 0.008  # each jaw 12 mm thick (y) and 18 mm high, reaching from 8 mm behind the bar's centre line to the tip
PAD_L, PAD_T, PAD_H = 0.035, 0.004, 0.016  # each jaw's pad: along the jaw from its tip, thick, high
GRIP_PAD_FACE = _board_y["board_r"] - BOARD_W / 2 - 0.005  # the grip's pad 5 mm outside the right board's outer edge
BAR_PAD_FACE = _board_y["board_l"] + BOARD_W / 2 + 0.013  # the bar's pad 13 mm outside the left board's outer edge
OPENING = BAR_PAD_FACE - GRIP_PAD_FACE  # 0.140 m
HEAD = ((X_BAR - 0.015, X_BAR + 0.015), (GRIP_PAD_FACE - PAD_T - JAW_T - 0.030, GRIP_PAD_FACE - PAD_T - JAW_T))  # x and y spans of the grip's head
HANDLE = ((X_BAR - 0.098, X_BAR - 0.015), (HEAD[1][0] + 0.006, HEAD[1][1]))  # the handle under the head, 24 mm thick, pointing to the arms
PIN = np.array([X_BAR - 0.020, HANDLE[1][0] - 0.006, MID_Z])  # the trigger's pin
TRIGGER_L, TRIGGER_T, TRIGGER_H = 0.070, 0.010, 0.014  # the trigger's long arm from its pin, thick, high
TRAVEL_DEG = 23.0  # the trigger turns 0 to 23 degrees on its pin (spec)
TRAVEL = math.radians(TRAVEL_DEG)
DRIVE_ARM = 0.015  # m from the pin to where the link takes the trigger's short arm: the drive plate goes 5.9 mm a full stroke, and a
# finger pushing with the gripper's 50 N 4 cm from the pin draws the bar with about 110 N at most (c6b, 9 mm: up to 160 N, under which the
# boards sank into each other 2.4 mm a stroke)
LINK_L = 0.008
PLAY = 0.00015  # m the drive plate goes forwards along the bar before it bites (about 0.6 degrees of trigger)
FINGER_AT = 0.040  # m from the pin where a finger presses the trigger (the spring's figure is given there)
SPRING_OPEN_N = 8.0  # N at FINGER_AT that hold the trigger open (assumed)
SPRING_BACK = 1.0  # rad past the open stop where the trigger's spring would be at rest: a preloaded spring, 8 N open, 11 N closed
TRIGGER_K = SPRING_OPEN_N * FINGER_AT / SPRING_BACK  # N m/rad

# Masses. As built: the grip 0.19 kg, the bar 0.08 kg, a pad 10 g, the trigger 25 g (assumed). The gauge pad and the
# trigger are pressed between a hand or a board and joints; they get the inertia of heavier bodies for the solver (a
# light body pressed between a hand and a joint gives way in this simulator: mech/tasks/servo_horn, run c3u), their
# weight over the real one carried by a push. The link and the drive plate are hidden inside the head: weightless.
GRIP_MASS, BAR_MASS, PAD_REAL, TRIGGER_REAL = 0.19, 0.08, 0.010, 0.025
PAD_MASS = float(os.environ.get("CLAMP_PAD_MASS", "0.2"))
TRIGGER_MASS = float(os.environ.get("CLAMP_TRIGGER_MASS", "0.15"))
INNER_MASS = 0.1  # the link and the drive plate, each
GRAVITY = 9.81

PAD_K = float(os.environ.get("CLAMP_PAD_K", "40000.0"))  # N/m as declared: 1 mm for every 40 N (assumed, standing for pads, jaws and bar together)
GAUGE_RATE = float(os.environ.get("CLAMP_GAUGE_RATE", "40000.0"))  # N/m the pad gives in the simulator: measured by trials.py gauge (to be set)
PAD_TRAVEL = 0.005  # m the gauge pad can give: 200 N
DRAW_MAX = OPENING + 0.003  # m the bar can be drawn: the jaws closed on each other, pads pressed 3 mm
_jaw_x = ((X_BAR - JAW_ROOT) + TIP_X) / 2
_jaw_len = TIP_X - (X_BAR - JAW_ROOT)


def _grip_shapes() -> list:
    """The grip, in its frame (origin at the bar's centre line at the head's middle, z at the middle plane)."""
    oy = (HEAD[1][0] + HEAD[1][1]) / 2
    rel = lambda x, y, z=0.0: (x - X_BAR, y - oy, z)  # noqa: E731
    head = box((HEAD[0][1] - HEAD[0][0], HEAD[1][1] - HEAD[1][0], JAW_H), pos=rel((HEAD[0][0] + HEAD[0][1]) / 2, (HEAD[1][0] + HEAD[1][1]) / 2))
    jaw = box((_jaw_len, JAW_T, JAW_H), pos=rel(_jaw_x, GRIP_PAD_FACE - PAD_T - JAW_T / 2))
    handle = box((HANDLE[0][1] - HANDLE[0][0], HANDLE[1][1] - HANDLE[1][0], JAW_H), pos=rel((HANDLE[0][0] + HANDLE[0][1]) / 2, (HANDLE[1][0] + HANDLE[1][1]) / 2))
    return [head, jaw, handle]


GRIP_AT = np.array([X_BAR, (HEAD[1][0] + HEAD[1][1]) / 2, MID_Z])
body("grip", _grip_shapes(), pos=tuple(GRIP_AT), mass=GRIP_MASS, color=BLACK, friction=(0.4, 0.35))

# The bar and its jaw, in the bar's frame (origin on the bar's centre line at the grip's pad face's y).
BAR_AT = np.array([X_BAR, GRIP_PAD_FACE, MID_Z])
_bar_pad_mid = np.array([TIP_X - PAD_L / 2, BAR_PAD_FACE + PAD_T / 2, MID_Z])
body(
    "bar",
    [
        box((BAR_W, BAR_TO - BAR_FROM, BAR_T), pos=(0.0, (BAR_FROM + BAR_TO) / 2 - GRIP_PAD_FACE, 0.0), color=STEEL),
        box((_jaw_len, JAW_T, JAW_H), pos=(_jaw_x - X_BAR, BAR_PAD_FACE + PAD_T + JAW_T / 2 - GRIP_PAD_FACE, 0.0), color=BLACK),
        box((PAD_L, PAD_T, PAD_H), pos=(TIP_X - PAD_L / 2 - X_BAR, BAR_PAD_FACE + PAD_T / 2 - GRIP_PAD_FACE, 0.0), color=RUBBER, friction=(0.8, 0.7)),
    ],
    pos=tuple(BAR_AT),
    mass=BAR_MASS,
    color=STEEL,
    friction=(0.4, 0.35),
)
BAR_DRAG = 20.0  # N s/m the bar slides through the grip against (2 N at 10 cm/s): a bar the trigger flicks on coasts no more than a millimetre (c6a: 6 mm at 2 N s/m)
slide("draw", "bar", "grip", axis=(0.0, -1.0, 0.0), limits=(0.0, DRAW_MAX), damping=BAR_DRAG)

# The gauge pad on the grip's jaw: its face is what touches the right board.
PAD_AT = np.array([TIP_X - PAD_L / 2, GRIP_PAD_FACE - PAD_T / 2, MID_Z])
body("pad", [box((PAD_L, PAD_T, PAD_H))], pos=tuple(PAD_AT), mass=PAD_MASS, color=RUBBER, friction=(0.8, 0.7))
slide("gauge", "pad", "grip", axis=(0.0, -1.0, 0.0), limits=(0.0, PAD_TRAVEL), spring=PAD_K, damping=2.0 * math.sqrt(PAD_K * PAD_MASS))

# The trigger: its frame at the pin, x along the short arm, the long arm along -x; built open, 23 degrees from the
# handle, so that closed it lies along the handle (1 mm from it).
_open_yaw = TRAVEL
body(
    "trigger",
    [
        box((TRIGGER_L, TRIGGER_T, TRIGGER_H), pos=(-TRIGGER_L / 2, 0.0, 0.0)),
        box((DRIVE_ARM + 0.004, 0.008, 0.010), pos=((DRIVE_ARM + 0.004) / 2 - 0.002, 0.0, 0.0), passes=("bar", "plate", "link", "pad"), solid=False),
    ],
    pos=tuple(PIN),
    yaw=_open_yaw,
    mass=TRIGGER_MASS,
    color=YELLOW,
    friction=(0.6, 0.5),
)
hinge("trigger", "trigger", "grip", anchor=tuple(PIN), axis=(0.0, 0.0, -1.0), limits=(0.0, TRAVEL), spring=TRIGGER_K, rest=-SPRING_BACK, damping=0.002)

# The drive: the link from the trigger's short arm to the drive plate, which slides along the bar ("drive": how far it
# has gone along the bar towards the jaws, from where it was built); its upper stop is the bite (see ``ratchet``).
ARM_AT = PIN + DRIVE_ARM * np.array([math.cos(_open_yaw), math.sin(_open_yaw), 0.0])
PLATE_AT = ARM_AT + np.array([0.0, -LINK_L, 0.0])
_hidden = ("grip", "bar", "pad", "trigger", "board_r", "board_l", "link", "plate")
body("link", [box((0.003, LINK_L, 0.003), pos=(0.0, -LINK_L / 2, 0.0), passes=_hidden)], pos=tuple(ARM_AT), mass=INNER_MASS, color=STEEL)
body("plate", [box((0.004, 0.004, 0.004), passes=_hidden)], pos=tuple(PLATE_AT), mass=INNER_MASS, color=STEEL)
hinge("link_a", "link", "trigger", anchor=tuple(ARM_AT), axis=(0.0, 0.0, 1.0), limits=(-math.pi / 2, math.pi / 2))
hinge("link_b", "link", "plate", anchor=tuple(PLATE_AT), axis=(0.0, 0.0, 1.0), limits=(-math.pi / 2, math.pi / 2))
DRIVE_BACK = DRAW_MAX + 0.010  # m the drive plate can be back along the bar: the bar drawn all the way, the trigger open
slide("drive", "plate", "bar", axis=(0.0, -1.0, 0.0), limits=(-DRIVE_BACK, PLAY))
OBJECTS = list(mk.MECH.bodies.values())
CLAMP = ("grip", "bar", "pad", "trigger")
CARRIED = {"pad": PAD_MASS - PAD_REAL, "trigger": TRIGGER_MASS - TRIGGER_REAL, "link": INNER_MASS, "plate": INNER_MASS}  # kg of weight carried by a push
BOXES = {name: [s.bounds() for s in mk.MECH.bodies[name].shapes if s.solid] for name in (*BOARDS, *CLAMP)}  # each body's solid shapes, in its frame

# --- the ratchet ---
SQUEEZE_AT, LET_GO_AT = math.radians(0.4), math.radians(0.4)  # the trigger closed this far past its lowest since it was let go: squeezed (before the drive plate bites); opened this far from its highest: let go
BITING = 0.00002  # m: the drive plate this close to its bite, the trigger squeezed: it draws the bar
SLACK = 0.0001  # m the lock plate stays behind where a bar pushed by hand met the boards: what a hard push sinks the contacts in (c6a: 0.12 mm after a 10 N shove) is not kept

# --- the rules ---
INSIDE_ENDS, MOUTH = 0.010, 0.0005  # m: a pad's middle this far inside the board's ends; pad gaps and the joint at most this (closed)
CLAMPED_N = 20.0
ENDS_ALIGNED, TOPS_LEVEL = 0.003, 0.001
RESTING, CLEAR = 0.02, 0.03  # m/s; m both grippers keep from clamp and boards
SETTLE_S = 0.25  # s over which a part's speed is read from how far it has gone (c6f: squeezed, the solver gave the boards and the
# pad 10-75 mm/s of velocity while they stood still to a few micrometres: its contact corrections)
ON_BENCH = 0.02  # m: a part this far under the bench top has left it
SIDES = ("left", "right")
STATE: dict = {}


def begin(seed: int | None = None) -> dict:
    """A new trial's state (no simulator needed)."""
    STATE.clear()
    STATE.update(seed=seed, carried=False, squeezed=False, driving=False, low=0.0, high=0.0, bite=PLAY, lock=0.0, strokes=0, most_n=0.0, track=[])
    for name in ("link", "plate"):
        mk.show(name, 0, False)
    mk.show("trigger", 1, False)
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)
    if mk.MECH.environment is not None:  # the simulator keeps a joint's stops through a reset: back to the built ones
        mk.set_limits("draw", lower=0.0, upper=DRAW_MAX)
        mk.set_limits("drive", lower=-DRIVE_BACK, upper=PLAY)


def gauge() -> float:
    """N the gauge pad presses the board with: how far it has given, at the rate the simulator gives it."""
    return GAUGE_RATE * mk.joint("gauge")


def ratchet() -> None:
    """The trigger's strokes, the drive plate's bite and the lock plate, once per control step (in the simulator only).

    Squeezed: the trigger closed SQUEEZE_AT past its lowest since it was let go; let go: opened LET_GO_AT from its
    highest since. The drive plate's stop on the bar (the bite) follows the plate back while the trigger is not
    squeezed, PLAY ahead of it and as much more as the bar is ahead of its lock, and stays put while it is squeezed:
    the trigger closing pushes the plate onto it and draws the bar. Not squeezed, the plate so never holds the bar
    back before the lock plate does (c6b: a bar pushed by hand kept 6 N because the idle plate held it).
    The lock plate (the bar's lower stop) follows the bar on while the trigger draws it (squeezed, the plate at its
    bite); otherwise only as far as the bar is less what the gauge pad has given, less SLACK: where the bar's jaw met
    the boards.
    Read once per control step, both stops move one way only, and what a step's lag lets through is at most one
    step's travel of the bar."""
    angle, drawn, slid, give = mk.joint("trigger"), mk.joint("draw"), mk.joint("drive"), mk.joint("gauge")
    if not STATE["squeezed"]:
        if angle > STATE["low"] + SQUEEZE_AT:
            STATE.update(squeezed=True, high=angle, strokes=STATE["strokes"] + 1)
        else:
            STATE["low"] = min(STATE["low"], angle)
    elif angle < STATE["high"] - LET_GO_AT:
        STATE.update(squeezed=False, low=angle)
    else:
        STATE["high"] = max(STATE["high"], angle)
    bite = STATE["bite"] if STATE["squeezed"] else min(STATE["bite"], slid + PLAY + max(0.0, drawn - STATE["lock"]))
    if bite != STATE["bite"]:
        mk.set_limits("drive", upper=bite)
    driving = STATE["squeezed"] and slid >= bite - BITING
    lock = max(STATE["lock"], drawn if driving else drawn - max(give, 0.0) - SLACK)
    if lock != STATE["lock"]:
        mk.set_limits("draw", lower=lock)
    STATE.update(bite=bite, driving=driving, lock=lock, most_n=max(STATE["most_n"], GAUGE_RATE * give))


@mk.on_step
def step() -> None:
    if mk.MECH.environment is not None:
        if not STATE["carried"]:  # weights over the real ones, carried from the first step on
            for name, kilograms in CARRIED.items():
                mk.push(name, force=(0.0, 0.0, kilograms * GRAVITY))
            STATE["carried"] = True
        ratchet()
        track()


# --- where things are ---
def to_world(name: str, point) -> np.ndarray:
    position, rotation = mk.pose(name)
    return position + rotation @ np.asarray(point, dtype=float)


def pad_middles() -> dict:
    """The middles of the two pads' faces, in the world: the grip's (the gauge pad) and the bar's."""
    return {"grip": to_world("pad", (0.0, PAD_T / 2, 0.0)), "bar": to_world("bar", _bar_pad_mid - BAR_AT - np.array([0.0, PAD_T / 2, 0.0]))}


def in_board(name: str, point) -> np.ndarray:
    """``point`` in board ``name``'s frame."""
    position, rotation = mk.pose(name)
    return rotation.T @ (np.asarray(point, dtype=float) - position)


def pad_place(side: str) -> dict:
    """How the pad of ``side`` (grip: the right board's outer face; bar: the left board's) sits: its gap to the board's
    outer face (m, negative: in it), how far its middle is inside the board's nearer end (m) and whether it is within
    the board's thickness."""
    board, outward = ("board_r", -1.0) if side == "grip" else ("board_l", 1.0)
    local = in_board(board, pad_middles()[side])
    return {"gap": float(outward * local[1] - BOARD_W / 2), "inside": float(BOARD_L / 2 - abs(local[0])), "within": bool(abs(local[2]) <= BOARD_T / 2)}


def joint_gaps() -> tuple[float, float]:
    """The opening between the boards' facing long edges at the near and the far end (m): the left board's inner
    corners against the right board's inner face."""
    out = []
    for x in (-BOARD_L / 2, BOARD_L / 2):
        corner = to_world("board_l", (x, -BOARD_W / 2, 0.0))
        out.append(float(in_board("board_r", corner)[1] - BOARD_W / 2))
    return out[0], out[1]


def round_boards() -> bool:
    places = [pad_place(side) for side in ("grip", "bar")]
    return all(p["gap"] >= -MOUTH and p["inside"] >= INSIDE_ENDS and p["within"] for p in places)


def closed() -> bool:
    return round_boards() and all(pad_place(side)["gap"] <= MOUTH for side in ("grip", "bar")) and all(-MOUTH <= g <= MOUTH for g in joint_gaps())


def ends_and_tops() -> tuple[float, float]:
    """(how far apart the boards' near ends are along the right board, m; how far apart their tops are, m)."""
    near = [to_world(name, (-BOARD_L / 2, 0.0, BOARD_T / 2)) for name in BOARDS]
    return float(abs(in_board("board_r", near[1])[0] - (-BOARD_L / 2))), float(abs(near[0][2] - near[1][2]))


def on_bench() -> bool:
    return all(mk.pose(name)[0][2] >= TOP - ON_BENCH for name in (*BOARDS, *CLAMP))


def distance_to(name: str, point) -> float:
    """How far ``point`` is from body ``name``'s solid shapes (m; 0 inside one)."""
    position, rotation = mk.pose(name)
    local = rotation.T @ (np.asarray(point, dtype=float) - position)
    return min(float(np.linalg.norm(np.maximum(np.maximum(np.asarray(low) - local, local - np.asarray(high)), 0.0))) for low, high in BOXES[name])


def let_go() -> bool:
    """Both grippers 3 cm or more from the clamp and the boards."""
    for side in SIDES:
        at = mk.gripper(side)[0]
        for name in (*BOARDS, *CLAMP):
            if distance_to(name, at) < CLEAR:
                return False
    return True


def _marks() -> dict:
    """The point of each part whose travel is read: its frame's origin, the trigger's tip (its origin is its pin)."""
    return {name: to_world(name, (-TRIGGER_L, 0.0, 0.0) if name == "trigger" else (0.0, 0.0, 0.0)) for name in (*BOARDS, *CLAMP)}


def track() -> None:
    """Keep where the parts are, every control step, for SETTLE_S back (an @on_step hook; tests call it themselves)."""
    t, kept = mk.now(), STATE.setdefault("track", [])
    kept.append((t, _marks()))
    while len(kept) > 2 and kept[1][0] <= t - SETTLE_S:
        kept.pop(0)


def speeds() -> dict:
    """m/s each part has gone at over the last SETTLE_S (straight from where it was then); None before two readings."""
    kept = STATE.get("track", [])
    if len(kept) < 2 or kept[-1][0] <= kept[0][0]:
        return {name: None for name in (*BOARDS, *CLAMP)}
    (t0, then), (t1, now) = kept[0], kept[-1]
    return {name: float(np.linalg.norm(now[name] - then[name])) / (t1 - t0) for name in now}


def at_rest() -> bool:
    return all(v is not None and v < RESTING for v in speeds().values())


def level() -> float:
    """The score without the release: see ``score``."""
    if not on_bench() or not round_boards():
        return 0.0
    if not closed():
        return 0.25
    return 0.75 if gauge() >= CLAMPED_N else 0.5


def score(objects) -> float:
    """By where things are at the end. 0 if a part has left the bench. Round the boards (both pads' middles 1 cm or
    more inside the boards' ends and within their thickness, on the boards' outer sides): 0.25. Closed (also both pad
    gaps and the joint 0.5 mm or less): 0.5 under 20 N on the gauge, 0.75 with 20 N or more; and 1.0 when also the
    boards' near ends lie within 3 mm of each other, their tops level within 1 mm, everything at rest and both
    grippers 3 cm or more from clamp and boards. Anything else: 0."""
    found = level()
    if found == 0.75:
        ends, tops = ends_and_tops()
        if ends <= ENDS_ALIGNED and tops <= TOPS_LEVEL and at_rest() and let_go():
            return 1.0
    return found


def success(objects) -> bool:
    return score(objects) == 1.0


def progress() -> str:
    g, b = pad_place("grip"), pad_place("bar")
    near, far = joint_gaps()
    return (f"pads {1000 * g['gap']:.1f} / {1000 * b['gap']:.1f} mm from the boards, joint {1000 * near:.1f} / {1000 * far:.1f} mm, gauge {gauge():.1f} N, "
            f"bar drawn {1000 * mk.joint('draw'):.1f} mm, trigger {math.degrees(mk.joint('trigger')):.1f} deg; level {score(None):.2f}")  # fmt: skip


def report() -> dict:
    out: dict = {"strokes": STATE.get("strokes", 0), "most on the gauge (N)": round(STATE.get("most_n", 0.0), 2)}
    if mk.MECH.poses:
        g, b = pad_place("grip"), pad_place("bar")
        near, far = joint_gaps()
        ends, tops = ends_and_tops()
        out.update({"gauge (N)": round(gauge(), 2), "bar drawn (mm)": round(1000 * mk.joint("draw"), 3), "trigger (deg)": round(math.degrees(mk.joint("trigger")), 2),
                    "pad gaps, grip and bar (mm)": [round(1000 * g["gap"], 3), round(1000 * b["gap"], 3)], "pads inside the ends (mm)": [round(1000 * g["inside"], 1), round(1000 * b["inside"], 1)],
                    "joint, near and far (mm)": [round(1000 * near, 3), round(1000 * far, 3)], "ends apart (mm)": round(1000 * ends, 2), "tops apart (mm)": round(1000 * tops, 2),
                    "round": round_boards(), "closed": closed()})  # fmt: skip
        out["speeds (mm/s)"] = {name: None if v is None else round(1000 * v, 2) for name, v in speeds().items()}
        if mk.MECH.grippers:
            out.update({"at rest": at_rest(), "let go": let_go(), "score": score(None)})
    return out
