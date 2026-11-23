"""Set a winged specimen pan across a toilet bowl's opening, both wings on the rim, under the raised seat.

A nursing assistant collects urine or stool this way: the pan (a "specimen hat") goes across the bowl under the seat,
its two wings on the rim. The toilet is a Kohler K-4302 Highcrest elongated bowl's geometry over the rim at its maker's
sizes, with its inside down to the water and the wall behind it at the 10 in. rough-in, on a stand that replaces the
pedestal and the floor; the seat is a Bemis 1955CT open-front commercial seat on its 5-1/2 in. posts, raised on its
check hinges 11 degrees past upright, where it stays up by itself; the pan is a Medline 30 oz specimen collector pan,
222.3 x 336.6 x 100.0 mm outside. The pan starts on the bench lengthwise: set down that way it drops into the bowl
(222.3 mm < 264 mm across, 336.6 mm < 379 mm along); turned 90 degrees its wings bridge the 264 mm with 36 mm on the rim
each side. Pushed forward past its balance at 94 degrees, the seat falls shut. Every figure and its source:
``parts.py``; bodies: ``scene.py``. The task welds, pushes, latches or places nothing: rigid contact and one free hinge.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402
import scene  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 3.0  # both arms are back at their ready pose before the run ends: the last picture shows the result
INSTRUCTION = (
    "Set the specimen pan upright across the toilet bowl opening with both wings on the rim, leaving the raised seat up, "
    "then let go and move both hands clear."
)
RUN_LIMIT_STEPS = 900  # 45 s at 20 Hz
TOP = TABLE_TOP[SCENE]
SIDES = ("left", "right")

scene.build(TOP)
OBJECTS = list(mk.MECH.bodies.values())
STATE: dict = {}


def begin(seed=None) -> dict:
    STATE.clear()
    STATE.update(seed=seed, zero=None, zero_t=None, hands=None, rest=0, rest_ref=None, min_seat_deg=P.CHECK_HINGE_DEG, firsts={})
    return STATE


begin()


@mk.on_reset
def start(seed) -> None:
    begin(seed)


# --- Readings -----------------------------------------------------------------------------------------------------------
def seat_deg() -> float:
    """Degrees the seat is raised (0 closed on the rim, 101 on its check-hinge stop, where it is built)."""
    return P.CHECK_HINGE_DEG + math.degrees(float(mk.joint("seat_hinge")))


def pan_tilt_deg() -> float:
    r = mk.pose("specimen_pan")[1]
    return math.degrees(math.acos(float(np.clip(r[2, 2], -1.0, 1.0))))


def pan_across_deg() -> float:
    """Degrees between the pan's wing axis and the line across the opening (y): 0 when the wings bridge it."""
    r = mk.pose("specimen_pan")[1]
    return math.degrees(math.acos(float(np.clip(abs(r[1, 1]), 0.0, 1.0))))


def _pan_point(local) -> np.ndarray:
    p, r = mk.pose("specimen_pan")
    return p + r @ np.asarray(local, float)


def _to_bowl(point: np.ndarray) -> tuple[float, float, float]:
    """(a across from the bowl's middle line, d from the seat posts towards the front, z over the bench)."""
    return float(point[1] - P.BOWL_Y), float(P.BOLT_X - point[0]), float(point[2] - TOP)


def wing_tips() -> list[tuple[float, float, float]]:
    """Both wing tips' undersides (15 mm in from the ends) in bowl terms."""
    z = P.PAN_H - P.WING_T
    return [_to_bowl(_pan_point((0.0, s * (P.PAN_SPAN / 2.0 - 0.015), z))) for s in (-1.0, 1.0)]


def pan_in_opening() -> bool:
    a, d, _ = _to_bowl(mk.pose("specimen_pan")[0])
    return abs(a) <= P.HOLE_W / 2.0 and P.HOLE_D0 <= d <= P.HOLE_D1


def pan_bridged() -> bool:
    """Upright (tilt <= 8 deg), wings across the opening (within 20 deg), its middle inside the opening and both wing
    tips over the side rims, their undersides at most 5 mm over the rim top."""
    if pan_tilt_deg() > P.TILT_MAX_DEG or pan_across_deg() > P.ACROSS_MAX_DEG or not pan_in_opening():
        return False
    for a, d, z in wing_tips():
        on_rim = P.HOLE_W / 2.0 <= abs(a) <= P.BOWL_W / 2.0 and P.HOLE_D0 <= d <= P.HOLE_D1
        if not on_rim or not -0.002 <= z - P.RIM_Z <= P.WING_GAP_MAX:
            return False
    return True


def seat_up() -> bool:
    return seat_deg() >= P.SEAT_UP_DEG


def pan_lowest() -> float:
    """m over the bench of the pan's lowest corner."""
    p, r = mk.pose("specimen_pan")
    pts = []
    for c, w in scene.PAN_BOXES or scene.pan_boxes():
        for sx in (-0.5, 0.5):
            for sy in (-0.5, 0.5):
                for sz in (-0.5, 0.5):
                    pts.append(np.asarray(c) + np.array([sx * w[0], sy * w[1], sz * w[2]]))
    return float((np.asarray(pts) @ r.T + p)[:, 2].min() - TOP)


def fell_in() -> bool:
    """The pan's lowest point within 8 mm of the water surface with its middle under the opening (30 mm margin)."""
    a, d, _ = _to_bowl(mk.pose("specimen_pan")[0])
    under = abs(a) <= P.HOLE_W / 2.0 + 0.03 and P.HOLE_D0 - 0.03 <= d <= P.HOLE_D1 + 0.03
    return under and pan_lowest() <= P.WATER_Z + P.DROP_GAP


# --- The hands ---------------------------------------------------------------------------------------------------------
_HULLS: dict = {}
_LINKS: dict = {}


def read_hands() -> None:
    """Every hull corner of both hands in the world (STATE['hands']: side -> points)."""
    from yam_arena import analysis

    if not _HULLS:
        for link in ("gripper", "tip_left", "tip_right"):
            _HULLS[link] = np.vstack([h.vertices for h in analysis.link_hulls().get(link, [])])
    base = mk.MECH.environment.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
    hands = {}
    for side in SIDES:
        robot = base.scene[f"{side}_robot"]
        points = []
        for link, local in _HULLS.items():
            key = (id(base), side, link)
            if key not in _LINKS:
                found = robot.find_bodies(link)[0]
                if not found:
                    raise RuntimeError(f"the {side} arm has no body {link!r}")
                _LINKS[key] = found[0]
            pose = robot.data.body_link_pose_w.torch[0, _LINKS[key]].detach().cpu().numpy().astype(float)
            points.append(local @ mk.quat_to_matrix(pose[3:7]).T + pose[:3] - origin)
        hands[side] = np.vstack(points)
    STATE["hands"] = hands


def _distance(points: np.ndarray, name: str) -> float:
    """m from the nearest of ``points`` to any solid box of body ``name``."""
    pos, rot = mk.pose(name)
    local = (points - pos) @ rot
    best = math.inf
    for s in mk.MECH.bodies[name].shapes:
        if s.solid:
            inside = (local - np.asarray(s.pos, float)) @ mk.quat_to_matrix(s.quat)
            gap = np.maximum(np.abs(inside) - s.half_extents(), 0.0)
            best = min(best, float(np.linalg.norm(gap, axis=1).min()))
    return best


def _hand_points(sides=SIDES):
    hands = STATE.get("hands")
    return None if not hands else np.vstack([hands[s] for s in sides if s in hands])


def hand_to_seat() -> float:
    pts = _hand_points()
    return math.inf if pts is None else _distance(pts, "toilet_seat")


def seat_held() -> bool:
    """A hand within 25 mm of the seat, or the seat resting on the pan."""
    return hand_to_seat() <= P.HOLD_HAND or mk.contact_force("specimen_pan", "toilet_seat") >= 0.5


def hands_clear() -> float:
    """m from the nearest hull corner of either hand to the seat, the pan or the bowl (with its stand and wall)."""
    pts = _hand_points()
    return 0.0 if pts is None else min(_distance(pts, n) for n in ("toilet_seat", "specimen_pan", "toilet_bowl"))


# --- Whole-trial events, rest and the score -----------------------------------------------------------------------------
def _note(what: str) -> None:
    STATE["firsts"].setdefault(what, round(mk.now(), 2))


def _zero(why: str) -> None:
    if STATE["zero"] is None:
        STATE["zero"], STATE["zero_t"] = why, round(mk.now(), 2)


def _rest_key() -> tuple[np.ndarray, float]:
    return mk.pose("specimen_pan")[0].copy(), seat_deg()


@mk.on_step
def step() -> None:
    if mk.MECH.environment is not None:
        read_hands()
    deg = seat_deg()
    STATE["min_seat_deg"] = min(STATE["min_seat_deg"], deg)
    if fell_in():
        _zero("the pan fell into the bowl")
    # Knocked off its stop: past its balance at 94 deg the seat falls shut; under 90 deg with nothing holding it, it
    # is falling (a seat let go of in mid-air, or pushed over, passes 90 deg within a step or two).
    if deg < P.FALL_DEG and not seat_held():
        _zero("the seat fell from its stop")
    bridged = pan_bridged()
    if bridged:
        _note("pan across the rim")
    if bridged and seat_up() and hands_clear() >= P.HANDS_CLEAR:
        ref = STATE["rest_ref"]
        now = _rest_key()
        if ref is None or np.linalg.norm(now[0] - ref[0]) > P.REST_MOVE or abs(now[1] - ref[1]) > P.REST_TURN_DEG:
            STATE["rest_ref"], STATE["rest"] = now, 1
        else:
            STATE["rest"] += 1
    else:
        STATE["rest_ref"], STATE["rest"] = None, 0


def score(objects) -> float:
    """0.0 for good if at any step the pan falls into the bowl (its lowest point within 8 mm of the water surface, its
    middle under the opening) or the seat falls from its stop (under 90 deg with no hand within 25 mm of it and not
    resting on the pan). Otherwise, at the end of the trial:
    1.0: the pan across the opening: upright (tilt <= 8 deg), its wings within 20 deg of square across the opening,
         its middle inside the opening, both wing tips over the side rims with their undersides at most 5 mm over the
         rim top; the seat up on its stop (at least 99 deg); every part of both hands at least 30 mm from the seat, the
         pan and the bowl; all held so for 1.0 s (20 steps) with the pan moving at most 1.5 mm and the seat at most
         0.5 deg.
    0.5: the pan across the opening as above, any 1.0 condition not met.
    0.0: otherwise."""
    del objects
    if STATE["zero"] is not None or not pan_bridged():
        return 0.0
    return 1.0 if STATE["rest"] >= P.REST_STEPS else 0.5


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out = {"zero": STATE["zero"], "zero_t": STATE["zero_t"], "firsts": dict(STATE["firsts"]), "rest_steps": STATE["rest"],
           "min_seat_deg": round(STATE["min_seat_deg"], 2)}  # fmt: skip
    if mk.MECH.poses:
        out.update({
            "seat_deg": round(seat_deg(), 2),
            "pan_xyz_m": [round(float(v), 4) for v in mk.pose("specimen_pan")[0]],
            "pan_tilt_deg": round(pan_tilt_deg(), 2),
            "pan_across_deg": round(pan_across_deg(), 2),
            "wing_tips_over_rim_mm": [round(1000.0 * (z - P.RIM_Z), 1) for _, _, z in wing_tips()],
            "pan_lowest_over_water_mm": round(1000.0 * (pan_lowest() - P.WATER_Z), 1),
            "pan_on_bowl_N": round(mk.contact_force("specimen_pan", "toilet_bowl"), 2),
            "seat_on_pan_N": round(mk.contact_force("specimen_pan", "toilet_seat"), 2),
            "pan_bridged": pan_bridged(),
            "seat_up": seat_up(),
            "hands_clear_mm": round(1000.0 * hands_clear(), 1),
            "score": score(None),
        })
    return out
