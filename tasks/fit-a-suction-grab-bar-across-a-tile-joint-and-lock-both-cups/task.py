"""A suction grab bar pressed onto a tiled panel with its right cup across the joint between the tiles: that cup's
indicator shows red, the left one green. Instruction: fix the bar on the panel so that it holds. The safe way: one
hand holds the bar flat on the wall while the other lifts the left lever (released with nobody holding, the bar drops);
the bar is moved along the wall until both cups lie wholly on tile, and each lever is squeezed home against the grip
over centre by the hand on its side while that hand presses its cup to the wall.

Every figure comes from ``parts.py``, the shapes from ``shapes.py``. Bodies:
- ``panel``: two white tiles on an upright board, the joint between them a dark line; kinematic (it does not move,
  and each cup's contact sensor reads its press on it).
- ``bar``: the grip, the two housings, the levers' brackets and the indicator tabs on top (red and green, one drawn).
- ``cup_l``, ``cup_r``: the rubber cups, each welded to the bar as built (``cup_l_on``, ``cup_r_on``).
- ``lever_l``, ``lever_r``: the flip levers at the housings' outer ends, each on a vertical pivot, built locked (lying
  along its housing's end, its free end towards the wall), each with its over-centre cam (``lever.OverCentre``): loose
  from released (swung out) to LEVER_CAM_DEG (let go short of centre, a lever rests there), rising to LEVER_N at
  LEVER_ARM at LEVER_CENTRE_DEG, LEVER_HOLD of it locked; the torque acts between the lever and the bar.
- The cups' hold: ``weld("seal_l", "cup_l", "panel", made=False)`` and ``seal_r``. A seal is made in the control step
  in which its lever passes centre going down, if the cup's face lies flat on the tile, the cup pressed on the tile
  with PRESS_N or more within the last PRESS_S, and its rim lies wholly on one tile at least JOINT_MARGIN (1/2 in)
  from the joint and EDGE_MARGIN from the tile's outer edges; it is cut when the lever comes back over centre going up. Start: the left seal made,
  the right not (its cup across the joint, its lever down).
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402
import shapes as S  # noqa: E402
from lever import OverCentre  # noqa: E402
from mechkit import body, hinge, matrix_to_quat, sense_contact, weld  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 2.0  # the end state held: the bar and the levers at rest, both hands away, for two seconds
INSTRUCTION = "Fix the suction grab bar on the tiled panel so that it holds."
RUN_LIMIT_STEPS = 2750  # 137.5 s at 20 Hz: about 2.5 times the --check demo's 1,085 steps (check run gb5d15)
TOP = TABLE_TOP[SCENE]
SIDES = ("l", "r")
LEVER_AXES = {s: P.lever_axis_b(s) for s in ("l", "r")}  # each lever turns about the vertical, towards locked: into the housing's end

# --- the bodies ------------------------------------------------------------------------------------------------------
body("panel", S.panel(), pos=(P.PANEL_X, 0.0, TOP), kinematic=True)
BAR_AT = np.array([P.PANEL_X, P.BAR_Y0, TOP + P.BAR_Z])  # B's origin at the start: the bar hanging on the panel
_bar, _IND = S.bar()
body("bar", _bar, pos=tuple(BAR_AT), mass=P.BAR_MASS - 2 * P.CUP_MASS - 2 * P.LEVER_MASS, color=S.WHITE, friction=P.BAR_FRICTION)
LEVERS = {s: OverCentre(mk, f"lever_{s}") for s in SIDES}
_inertia = P.LEVER_MASS * P.LEVER_L**2 / 3  # kg m2 about the pivot (a thin plate)
for _s in SIDES:
    body(f"cup_{_s}", S.cup(), pos=tuple(BAR_AT + np.array([-P.CUP_T / 2, P.CUP_Y[_s], 0.0])), mass=P.CUP_MASS, color=S.RUBBER, friction=P.RUBBER_FRICTION)
    weld(f"cup_{_s}_on", f"cup_{_s}", "bar")  # part of the bar; a body of its own so that its press is read alone
    weld(f"seal_{_s}", f"cup_{_s}", "panel", made=False)
    _pivot = BAR_AT + np.array(P.lever_pivot_b(_s))
    body(f"lever_{_s}", S.lever(), pos=tuple(_pivot), quat=matrix_to_quat(S.lever_rot(P.LEVER_RAD, _s)), mass=P.LEVER_MASS, color=S.WHITE, friction=P.BAR_FRICTION)
    _k = LEVERS[_s].k
    hinge(f"lever_{_s}", f"lever_{_s}", "bar", anchor=tuple(_pivot), axis=LEVER_AXES[_s], limits=(-P.LEVER_RAD, 0.0), spring=_k, rest=LEVERS[_s].start_rest(),
          damping=2.0 * math.sqrt(_k * _inertia))  # fmt: skip
    sense_contact(f"cup_{_s}", "panel")
OBJECTS = list(mk.MECH.bodies.values())

# --- the rules -------------------------------------------------------------------------------------------------------
CLEAR = 0.03  # m: both grippers' grasp points at least this far from the bar (spec)
RESTING = (0.01, 0.1)  # m/s, rad/s: the bar (its cups with it) and the levers at rest
LEVEL_DEG = 10.0  # the bar within this of level
FELL_Z = 0.002  # m: the bar's lowest point this near the bench top: it is on the bench
G = 9.81  # m/s2
DROP_M = 0.010  # m: the bar comes down onto the bench as hard as a free drop from this height or more: it fell (let go this high or higher; lowered onto it at 0.17 m/s or more)
STRIKE_MPS = math.sqrt(2.0 * G * DROP_M)  # m/s (0.44): a cup meeting the tiles this fast or faster struck them, as hard as that drop
STATE: dict = {}


def begin() -> None:
    STATE.clear()
    STATE.update(events=[], fell=None, last=None, press={s: [] for s in SIDES}, refused={s: [] for s in SIDES}, most_press={s: 0.0 for s in SIDES})


begin()


def _show_indicator(side: str, green: bool) -> None:
    red_shape, green_shape = _IND[side]
    mk.show("bar", red_shape, not green)
    mk.show("bar", green_shape, green)


def sealed(side: str) -> bool:
    return not mk.is_cut(f"seal_{side}")


@mk.on_reset
def start() -> None:
    begin()
    for s in SIDES:
        LEVERS[s].reset()
    mk.make("seal_l")  # the start: the left cup holds, the right one (across the joint) does not
    for s in SIDES:
        _show_indicator(s, sealed(s))


def cup_frame(side: str) -> tuple[np.ndarray, np.ndarray]:
    """The cup's face middle (world) and its axis (out of the face, into the wall)."""
    p, r = mk.pose(f"cup_{side}")
    return p + r @ np.array([P.CUP_T / 2, 0.0, 0.0]), r[:, 0]


def face_on_tile(side: str) -> tuple[bool, float, float]:
    """(flat on the tile, tilt deg, gap mm): the face within FACE_TILT_DEG of the tiles' plane, FACE_GAP of their face."""
    middle, axis = cup_frame(side)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(axis[0])))))
    gap = abs(float(middle[0]) - P.PANEL_X)
    return tilt <= P.FACE_TILT_DEG and gap <= P.FACE_GAP, tilt, 1000 * gap


def rim_clearance(side: str) -> tuple[float, float]:
    """m: the cup's rim (from the joint, from its tile's outer edges); negative: across the joint, past an edge."""
    middle, _ = cup_frame(side)
    return P.rim_clearance(float(middle[1]), float(middle[2]) - TOP)


def clear(side: str) -> bool:
    """The cup's rim JOINT_MARGIN or more from the joint and EDGE_MARGIN or more from its tile's outer edges."""
    middle, _ = cup_frame(side)
    return P.clear_of_joint_and_edges(float(middle[1]), float(middle[2]) - TOP)


def pressed(side: str) -> float:
    """N: the most the cup pressed on the tile within the last PRESS_S."""
    now = mk.now()
    return max((f for t, f in STATE["press"][side] if now - t <= P.PRESS_S + 1e-9), default=0.0)


def lowest() -> tuple[float, float]:
    """(m over the bench, m/s down): the bar's lowest point (its cups' rims, the grip's underside) and its speed
    downwards, from the left cup's motion (welded to the bar; a cup's frame is its middle of mass)."""
    points = []
    for s in SIDES:
        middle, axis = cup_frame(s)
        down = np.array([0.0, 0.0, -1.0]) + axis * axis[2]  # straight down, within the rim's plane
        n = np.linalg.norm(down)
        points.append(middle + (P.CUP_R * down / n if n > 1e-9 else 0.0))
    p, r = mk.pose("bar")
    points.append(p + r @ np.array([P.GRIP_X, 0.0, 0.0]) - np.array([0.0, 0.0, P.GRIP_D / 2]))
    low = min(points, key=lambda q: float(q[2]))
    v, w = mk.velocity("cup_l")
    return float(low[2]) - TOP, -float((v + np.cross(w, low - mk.pose("cup_l")[0]))[2])


def lowest_z() -> float:
    return lowest()[0]


def check_fall(now: float, forces: dict) -> None:
    """The bar fell if its lowest point comes down onto the bench (from over FELL_Z to under it in one step) as hard as
    a free drop from DROP_M or more: the height it came down in that step plus the height a free fall needs for its
    speed in the step before (a dropped bar: its drop; one lowered at w: w * 0.05 s + w^2 / 2g). It struck the tiles
    if a cup meets them (its press from 0 to more) after a step at STRIKE_MPS or more towards them."""
    low, down = lowest()
    toward = {s: float(mk.velocity(f"cup_{s}")[0][0]) for s in SIDES}  # m/s into the wall
    last, why = STATE["last"], None
    if last is not None and low <= FELL_Z < last["low"]:
        drop = last["low"] - low + max(0.0, last["down"]) ** 2 / (2.0 * G)
        why = f"the bar came down onto the bench as from a {1000 * drop:.1f} mm drop"
        STATE["events"].append((now, why))
        why = why if drop >= DROP_M else None
    for s in SIDES:
        if last is not None and forces[s] > 0.0 >= last["press"][s] and last["toward"][s] >= STRIKE_MPS:
            why = f"the {s} cup met the tiles at {last['toward'][s]:.2f} m/s"
            STATE["events"].append((now, why))
    if why and STATE["fell"] is None:
        STATE["fell"] = now
    STATE["last"] = {"low": low, "down": down, "toward": toward, "press": dict(forces)}


@mk.on_step
def step() -> None:
    now = mk.now()
    forces = {}
    for s in SIDES:
        force = forces[s] = mk.contact_force(f"cup_{s}", "panel")
        STATE["press"][s] = [(t, f) for t, f in STATE["press"][s] if now - t <= 1.0] + [(now, force)]
        STATE["most_press"][s] = max(STATE["most_press"][s], force)
        change = LEVERS[s].step()
        if change is True:  # passed centre going down: seal if every condition holds
            flat, tilt, gap = face_on_tile(s)
            (joint, edges), push = rim_clearance(s), pressed(s)
            why = [w for w, ok in (("face not flat on the tile", flat), (f"pressed {push:.1f} N, under {P.PRESS_N:g} N", push >= P.PRESS_N),
                                   (f"rim {1000 * joint:.1f} mm from the joint, under {1000 * P.JOINT_MARGIN:g} mm", joint >= P.JOINT_MARGIN),
                                   (f"rim {1000 * edges:.1f} mm from the tile's edge, under {1000 * P.EDGE_MARGIN:g} mm", edges >= P.EDGE_MARGIN)) if not ok]  # fmt: skip
            if not why and not sealed(s):
                mk.make(f"seal_{s}")
                STATE["events"].append((now, f"{s} cup sealed (tilt {tilt:.1f} deg, gap {gap:.1f} mm, pressed {push:.1f} N, rim {1000 * joint:.1f} mm from the joint, {1000 * edges:.1f} mm from the edge)"))
            elif why:
                STATE["refused"][s].append((now, why))
                STATE["events"].append((now, f"{s} lever down, no seal: {'; '.join(why)}"))
            _show_indicator(s, sealed(s))
            mk.redraw()
        elif change is False:  # came up over centre: released
            if sealed(s):
                mk.cut(f"seal_{s}")
            STATE["events"].append((now, f"{s} lever up, cup released"))
            _show_indicator(s, False)
            mk.redraw()
    check_fall(now, forces)


def bar_level_deg() -> float:
    return math.degrees(math.asin(min(1.0, abs(float(mk.pose("bar")[1][2, 1])))))


def let_go() -> bool:
    """Both grippers' grasp points at least CLEAR from the bar (its grip's axis, its cups and levers)."""
    p, r = mk.pose("bar")
    a, b = p + r @ np.array([P.GRIP_X, -P.CUP_PITCH / 2, 0.0]), p + r @ np.array([P.GRIP_X, P.CUP_PITCH / 2, 0.0])
    points = [mk.pose(n)[0] for n in ("cup_l", "cup_r", "lever_l", "lever_r")]
    for side in ("left", "right"):
        g = np.asarray(mk.gripper(side)[0], dtype=float)
        t = min(1.0, max(0.0, float(np.dot(g - a, b - a) / np.dot(b - a, b - a))))
        if float(np.linalg.norm(g - (a + t * (b - a)))) - P.GRIP_D / 2 < CLEAR or min(float(np.linalg.norm(g - q)) for q in points) < CLEAR + P.CUP_R:
            return False
    return True


def at_rest() -> bool:
    """The bar and both levers at rest. The cups are welded parts of the bar and move only with it; read on their own,
    a cup welded both to the bar and to the panel shows the solver's 4 to 6 deg/s while the bar stands still to
    0.1 mm (check run gb5d10, trial gb5t6)."""
    names = ("bar", "lever_l", "lever_r")
    return all(float(np.linalg.norm(mk.velocity(n)[0])) <= RESTING[0] and float(np.linalg.norm(mk.velocity(n)[1])) <= RESTING[1] for n in names)


def score(objects) -> float:
    """0 if the bar has fallen or struck the tiles at any moment (``check_fall``); set down on the bench more gently,
    it scores as its end state. Otherwise, with both grippers 30 mm or more from the bar and the bar and the levers at
    rest: 0 if either cup's rim lies within JOINT_MARGIN (1/2 in) of the joint or EDGE_MARGIN of its tile's outer
    edges, or no seal is made; 1.0 for both cups clear, both seals made (both levers down, green) and the bar within
    LEVEL_DEG of level; 0.5 for both cups clear and one seal, or both seals with the bar more than LEVEL_DEG off level.
    Not let go or not at rest: 0."""
    if STATE["fell"] is not None or not (let_go() and at_rest()):
        return 0.0
    seals = sum(sealed(s) for s in SIDES)
    if not all(clear(s) for s in SIDES) or seals == 0:
        return 0.0
    return 1.0 if seals == 2 and bar_level_deg() <= LEVEL_DEG else 0.5


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out: dict = {"events": [f"{t:.2f} s {what}" for t, what in STATE.get("events", [])], "fell at s": STATE.get("fell"),
                 "most press N": {s: round(STATE["most_press"][s], 2) for s in SIDES} if STATE else None}  # fmt: skip
    if mk.MECH.poses:
        out.update({"levers deg from released": {s: round(math.degrees(LEVERS[s].theta()), 1) for s in SIDES}, "sealed": {s: sealed(s) for s in SIDES},
                    "rim from the joint, the edge mm": {s: [round(1000 * v, 1) for v in rim_clearance(s)] for s in SIDES}, "pressed N (last 0.2 s)": {s: round(pressed(s), 2) for s in SIDES},
                    "bar off level deg": round(bar_level_deg(), 2), "let go": let_go(), "at rest": at_rest(), "bar lowest over the bench mm": round(1000 * lowest_z(), 1)})  # fmt: skip
    return out
