"""Set a winged specimen pan across a toilet bowl's rim under its raised seat, then lower the seat onto the wings.

A nursing assistant collects urine or stool this way: the pan (a "specimen hat") goes across the bowl under the seat,
its two wings on the rim, and the seat comes down over them. Here the bowl is a mock rim of birch plywood cut to the
248 x 373 mm opening of a Kohler K-4302 elongated bowl; the seat is a Bemis 1955CT open-front commercial seat on its
5-1/2 in. hinge posts; the pan is a Medline 30 oz specimen collector pan, 222.3 x 336.6 x 100.0 mm outside. The pan
starts on the bench lengthwise: set down that way it drops through the opening (222.3 mm < 248 mm across, 336.6 mm <
373 mm along); turned 90 degrees its wings bridge the 248 mm with 44 mm on each side. The seat's check hinges stop it
11 degrees past upright, where it stays up; short of upright it falls back unless held. Every figure and its source:
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
    "Lift the toilet seat, set the specimen pan upright across the bowl opening with both wings on the rim, "
    "lower the seat onto the wings, then let go and move both hands clear."
)
RUN_LIMIT_STEPS = 1200  # 60 s at 20 Hz
TOP = TABLE_TOP[SCENE]
SIDES = ("left", "right")

scene.build(TOP)
OBJECTS = list(mk.MECH.bodies.values())
STATE: dict = {}


def begin(seed=None) -> dict:
    STATE.clear()
    STATE.update(seed=seed, zero=None, zero_t=None, peak=0.0, hands=None, rest=0, rest_ref=None, max_seat_deg=0.0, firsts={})
    return STATE


begin()


@mk.on_reset
def start(seed) -> None:
    begin(seed)


# --- Readings -----------------------------------------------------------------------------------------------------------
def seat_deg() -> float:
    return math.degrees(float(mk.joint("seat_hinge")))


def pan_tilt_deg() -> float:
    r = mk.pose("specimen_pan")[1]
    return math.degrees(math.acos(float(np.clip(r[2, 2], -1.0, 1.0))))


def pan_across_deg() -> float:
    """Degrees between the pan's wing axis and the line across the opening (x): 0 when the wings bridge it."""
    r = mk.pose("specimen_pan")[1]
    return math.degrees(math.acos(float(np.clip(abs(r[0, 1]), 0.0, 1.0))))


def _pan_point(local) -> np.ndarray:
    p, r = mk.pose("specimen_pan")
    return p + r @ np.asarray(local, float)


def _to_deck(point: np.ndarray) -> tuple[float, float, float]:
    """(x from the opening's middle line, d from the hinge line towards the front, z over the bench)."""
    return float(point[0] - P.HOLE_X), float(P.PINTLE_Y - point[1]), float(point[2] - TOP)


def wing_tips() -> list[tuple[float, float, float]]:
    """Both wing tips' undersides (15 mm in from the ends) in deck terms."""
    z = P.PAN_H - P.WING_T
    return [_to_deck(_pan_point((0.0, s * (P.PAN_SPAN / 2.0 - 0.015), z))) for s in (-1.0, 1.0)]


def pan_in_opening() -> bool:
    x, d, _ = _to_deck(mk.pose("specimen_pan")[0])
    return abs(x) <= P.HOLE_W / 2.0 and P.HOLE_D0 <= d <= P.HOLE_D0 + P.HOLE_L


def pan_bridged() -> bool:
    """Upright (tilt <= 8 deg), wings across the opening (within 20 deg), its middle inside the opening and both wing
    tips over the side rims, their undersides at most 5 mm over the rim top."""
    if pan_tilt_deg() > P.TILT_MAX_DEG or pan_across_deg() > P.ACROSS_MAX_DEG or not pan_in_opening():
        return False
    for x, d, z in wing_tips():
        on_rim = P.HOLE_W / 2.0 <= abs(x) <= P.RIM_OUT and P.HOLE_D0 <= d <= P.HOLE_D0 + P.HOLE_L
        if not on_rim or not -0.002 <= z - P.RIM_Z <= P.WING_GAP_MAX:
            return False
    return True


def _seat_local(point: np.ndarray) -> np.ndarray:
    p, r = mk.pose("toilet_seat")
    return (point - p) @ r


def wings_under_seat() -> bool:
    """Each wing's middle lies under the seat's ring: inside a ring part's outline seen along the seat's normal, its
    top no higher than the ring's underside."""
    for s in (-1.0, 1.0):
        q = _seat_local(_pan_point((0.0, s * (P.BOWL_W + P.PAN_SPAN) / 4.0, P.PAN_H)))
        if q[2] > P.RING_LOW + 0.002 or not any(abs(q[0] - c[0]) <= w[0] / 2.0 and abs(q[1] - c[1]) <= w[1] / 2.0 for c, w in P.seat_boxes()):
            return False
    return True


def seat_down() -> bool:
    """The seat lowered onto the wings: at most 3 deg up, with the ring over both wings."""
    return seat_deg() <= P.SEAT_DOWN_DEG and wings_under_seat()


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


def fell_through() -> bool:
    """The pan's lowest point within 8 mm of the bench with its middle under the opening (30 mm margin)."""
    x, d, _ = _to_deck(mk.pose("specimen_pan")[0])
    under = abs(x) <= P.HOLE_W / 2.0 + 0.03 and P.HOLE_D0 - 0.03 <= d <= P.HOLE_D0 + P.HOLE_L + 0.03
    return under and pan_lowest() <= P.DROP_Z


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
    return 0.0 if pts is None else _distance(pts, "toilet_seat")


def hands_clear() -> float:
    """m from the nearest hull corner of either hand to the seat, the pan or the deck."""
    pts = _hand_points()
    return 0.0 if pts is None else min(_distance(pts, n) for n in ("toilet_seat", "specimen_pan", "bowl_deck"))


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
    STATE["max_seat_deg"] = max(STATE["max_seat_deg"], deg)
    if fell_through():
        _zero("the pan fell through the opening")
    # Let go in mid-air: with no hand within 25 mm and not resting on the pan, the seat falls 2 deg or more from the
    # highest it was since it was last so held, and that was over 20 deg (a seat let go at 40 deg is shut in 0.2 s).
    held = (STATE.get("hands") is not None and hand_to_seat() <= P.LET_GO_HAND) or mk.contact_force("specimen_pan", "toilet_seat") >= 0.5
    STATE["peak"] = deg if held else max(STATE["peak"], deg)
    if not held and STATE["peak"] > P.LET_GO_DEG and deg < STATE["peak"] - P.LET_GO_FALL_DEG:
        _zero("the seat was let go above 20 degrees")
    if deg >= 10.0:
        _note("seat raised")
    if pan_bridged():
        _note("pan across the rim")
        if seat_down():
            _note("seat on the wings")
    if pan_bridged() and seat_down() and hands_clear() >= P.HANDS_CLEAR:
        ref = STATE["rest_ref"]
        now = _rest_key()
        if ref is None or np.linalg.norm(now[0] - ref[0]) > P.REST_MOVE or abs(now[1] - ref[1]) > P.REST_TURN_DEG:
            STATE["rest_ref"], STATE["rest"] = now, 1
        else:
            STATE["rest"] += 1
    else:
        STATE["rest_ref"], STATE["rest"] = None, 0


def score(objects) -> float:
    """0.0 for good if at any step the pan falls through the opening (its lowest point within 8 mm of the bench, its
    middle under the opening) or the seat is let go above 20 degrees (with no hand within 25 mm of it and not resting
    on the pan, it falls 2 deg or more from a height over 20 deg). Otherwise, at the end of the trial:
    1.0: the pan across the opening: upright (tilt <= 8 deg), its wings within 20 deg of square across the opening,
         its middle inside the opening, both wing tips over the side rims with their undersides at most 5 mm over the
         rim top; the seat lowered onto the wings (at most 3 deg up, the ring over the middle of both wings); every
         part of both hands at least 30 mm from the seat, the pan and the deck; all held so for 1.0 s (20 steps) with
         the pan moving at most 1.5 mm and the seat at most 0.5 deg.
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
           "max_seat_deg": round(STATE["max_seat_deg"], 2)}  # fmt: skip
    if mk.MECH.poses:
        out.update({
            "seat_deg": round(seat_deg(), 2),
            "pan_xyz_m": [round(float(v), 4) for v in mk.pose("specimen_pan")[0]],
            "pan_tilt_deg": round(pan_tilt_deg(), 2),
            "pan_across_deg": round(pan_across_deg(), 2),
            "wing_tips_over_rim_mm": [round(1000.0 * (z - P.RIM_Z), 1) for _, _, z in wing_tips()],
            "pan_lowest_mm": round(1000.0 * pan_lowest(), 1),
            "pan_on_deck_N": round(mk.contact_force("specimen_pan", "bowl_deck"), 2),
            "seat_on_pan_N": round(mk.contact_force("specimen_pan", "toilet_seat"), 2),
            "pan_bridged": pan_bridged(),
            "seat_down": seat_down(),
            "hands_clear_mm": round(1000.0 * hands_clear(), 1),
            "score": score(None),
        })
    return out
