"""A Klein 7-in-1 flip-socket nut driver lies in its start cradle, tip towards a bracket, with its three reversible
socket barrels on the shaft: the small red/yellow barrel at the working tip blocks the green/blue barrel, whose blue
3/8-inch socket faces the handle. The task: slide the front barrel off over the tip, take the green/blue barrel off,
flip it end-for-end and return it blue-end-out at the tip, restack the small barrel on the shaft behind it, then carry
the whole tool to the bracket, clock it and seat its blue socket over the 3/8-inch hex standoff, the handle settling
into the cradle beside the bracket.

How it is built (scene.py, every figure from parts.py): every hex is a true 6-point prism; the barrels ride the
oiled shaft in a 6.50 mm through-bore by contact alone (0.075 mm a flat, so each indexes on the shaft's flats and can
leave only over the tip, last-on-first-off; the shaft's tip and each bore's mouth have lead-ins); the blue 3/8 socket
(9.89 mm across flats, 15 mm deep) passes the 9.53 mm standoff only when clocked within 4 degrees, and every smaller
bore butts on it. The shaft runs through the socket's own bore, so the blue barrel seats only standing proud of the
tip (its through-bore still on the shaft). No joint, no weld, no computed force anywhere: gravity, contact and
friction carry everything. The score reads poses only.
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
SUCCESS_HOLD_S = 3.5  # both arms finish going back to their ready pose before the trial ends: the last picture shows the result
INSTRUCTION = ("Rearrange the flip-socket nut driver so that all three socket barrels are stored on its shaft with the "
               "blue 3/8-inch socket at the working tip, seat that socket fully over the 3/8-inch hex standoff on the "
               "bracket with the handle resting in the cradle beside it, then let go and move both hands clear.")  # fmt: skip
RUN_LIMIT_STEPS = 4600  # 230 s at 20 Hz, about 2.5 times the reference run's 92.8 s
TOP = TABLE_TOP[SCENE]
SIDES = ("left", "right")
WORK = P.BARRELS[1].key  # the green/blue barrel: the task's target
POST_AT = np.array([P.DRIVER_X, P.POST_Y + P.POST_OUT / 2, TOP + P.POST_Z])  # the standoff's near-face centre
POST_AXIS = np.array([0.0, 1.0, 0.0])
PARTS = ("driver", *(b.key for b in P.BARRELS), "stands", "cradle_start", "cradle_end", "bracket")

scene.build(TOP)
OBJECTS = list(mk.MECH.bodies.values())
for _b in P.BARRELS:  # read-only sensing: the normal force between each barrel and the shaft, for the record
    mk.sense_contact(_b.key, "driver")
STATE: dict = {}


def begin(seed: int | None = None) -> dict:
    STATE.clear()
    STATE.update(seed=seed, zero=None, extracted=None, hold=0, hold_from=None, hands=None)
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)


# --- the tool's frame read ------------------------------------------------------------------------------------------------
def driver_pose() -> tuple[np.ndarray, np.ndarray]:
    return mk.pose("driver")


def local(point) -> np.ndarray:
    """A world point in the driver's tip frame: x along the shaft (0 at the tip, + towards the handle)."""
    p, r = driver_pose()
    return r.T @ (np.asarray(point, dtype=float) - p)


def barrel_axis(key: str) -> np.ndarray:
    return mk.pose(key)[1][:, 0]  # towards the barrel's first-named end


def barrel_ends(key: str) -> tuple[np.ndarray, np.ndarray]:
    """(the first-named end's face centre, the other's): a barrel is 28.5-31 mm long."""
    p, r = mk.pose(key)
    half = next(b.length for b in P.BARRELS if b.key == key) / 2
    return p + r @ np.array([half, 0.0, 0.0]), p - r @ np.array([half, 0.0, 0.0])


def tilt_deg(a, b) -> float:
    c = max(-1.0, min(1.0, float(np.dot(a / np.linalg.norm(a), b / np.linalg.norm(b)))))
    return math.degrees(math.acos(c))


def on_shaft(key: str) -> bool:
    """The barrel sits on the shaft: coaxial within 10 degrees and 2 mm (either end first), and the plain part of its
    through-bore (where it rides the shaft, inside the bore's 1.2 mm lead-in mouths) wholly between the tip and the
    grip's nose."""
    centre = mk.pose(key)[0]
    at = local(centre)
    if math.hypot(float(at[1]), float(at[2])) > P.ON_RADIAL:
        return False
    along = float(np.dot(barrel_axis(key), driver_pose()[1][:, 0]))
    if abs(along) < math.cos(math.radians(P.ON_TILT_DEG)):
        return False
    lo, hi = next(b.through for b in P.BARRELS if b.key == key)
    lo, hi = lo + P.MOUTH, hi - P.MOUTH
    ends = (float(at[0]) + math.copysign(1.0, along) * lo, float(at[0]) + math.copysign(1.0, along) * hi)
    return min(ends) >= 0.0 and max(ends) <= P.EXPOSED


def blue_out() -> bool:
    """The working barrel's blue end points outboard within 15 degrees: flipped end-for-end from its start, where
    its green end (the barrel's first-named, +x) faces outboard."""
    return tilt_deg(barrel_axis(WORK), driver_pose()[1][:, 0]) <= P.FLIP_DEG


def face_at_tip() -> float:
    """The working barrel's outboard face in the tip frame (0 flush with the shaft tip, negative standing proud of it):
    its blue face once flipped."""
    face = barrel_ends(WORK)[1] if blue_out() else barrel_ends(WORK)[0]
    return float(local(face)[0])


def all_stored() -> bool:
    return all(on_shaft(b.key) for b in P.BARRELS)


def stored_at_tip() -> bool:
    """All three on the shaft and the working barrel blue end out at the tip: its face at most 3 mm short of the tip
    (it may stand proud of it as far as its through-bore stays on the shaft, which on_shaft reads)."""
    return all_stored() and blue_out() and face_at_tip() <= P.FLUSH


def mouth() -> tuple[np.ndarray, np.ndarray]:
    """The working socket's mouth centre and its inboard direction (into the bore)."""
    p, r = mk.pose(WORK)
    blue = barrel_ends(WORK)[1]  # the blue end is the barrel's second-named
    into = barrel_axis(WORK) if np.dot(barrel_axis(WORK), driver_pose()[1][:, 0]) > 0 else -barrel_axis(WORK)
    return blue, into / np.linalg.norm(into)


def seated() -> bool:
    """The blue socket is over the standoff: its mouth 5 to 11 mm past the standoff's near face, its axis within
    5 degrees of the standoff's, its centre within 2.5 mm of it, and its hex clocked to the standoff's flats within
    the fit's own window and 0.5 degrees (a socket turned further than that butts on the flats and cannot reach 5 mm)."""
    at, into = mouth()
    depth = float(np.dot(POST_AT - at, into))
    if not P.SEAT_MIN <= depth <= 0.011:
        return False
    if tilt_deg(into, POST_AXIS) > P.SEAT_DEG:
        return False
    d = mk.pose(WORK)[0] - POST_AT
    if float(np.linalg.norm(d - np.dot(d, POST_AXIS) * POST_AXIS)) > P.SEAT_RADIAL:
        return False
    roll = mk.pose(WORK)[1][:, 2]  # a flat normal of the socket's bore, in the world
    off = abs(math.degrees(math.atan2(float(roll[0]), float(roll[2]))) - P.POST_TWIST) % 60.0
    return min(off, 60.0 - off) <= math.degrees(P.CLOCK) + 0.5


def level_deg() -> float:
    """The driver's axis tilt out of level (nose up or down the same)."""
    r = driver_pose()[1]
    return math.degrees(math.asin(max(-1.0, min(1.0, abs(float(r[2, 0]))))))


def in_rest() -> bool:
    """The handle down in the end cradle: its centre within 25 mm of the seated position and within 6 mm of the
    cradle's seat height (a held-up tool reads high)."""
    p, r = driver_pose()
    mid = p + r @ np.array([P.TIP_TO_MID, 0.0, 0.0])
    return bool(abs(float(mid[1] - P.END_MID)) <= P.REST_NEAR and abs(float(mid[2] - (TOP + P.AXIS_Z))) <= 0.006)


def fell_off(name: str) -> bool:
    return bool(mk.pose(name)[0][2] < TOP - P.DROP_Z)


def zero_now() -> str | None:
    for name in ("driver", *(b.key for b in P.BARRELS)):
        if fell_off(name):
            return f"{name} fell off the bench"
    return None


# --- the hands ------------------------------------------------------------------------------------------------------------
_HULLS: dict = {}
_LINKS: dict = {}


def read_hands() -> None:
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
        for link, local_pts in _HULLS.items():
            key = (id(base), side, link)
            if key not in _LINKS:
                found = robot.find_bodies(link)[0]
                if not found:
                    raise RuntimeError(f"the {side} arm has no body {link!r}")
                _LINKS[key] = found[0]
            pose = robot.data.body_link_pose_w.torch[0, _LINKS[key]].detach().cpu().numpy().astype(float)
            points.append(local_pts @ mk.quat_to_matrix(pose[3:7]).T + pose[:3] - origin)
        hands[side] = np.vstack(points)
    STATE["hands"] = hands


def _box_distance(points: np.ndarray, name: str) -> float:
    position, rotation = mk.pose(name)
    low, high = mk.MECH.bodies[name].bounds()
    local_pts = (points - position) @ rotation
    gap = np.maximum(np.maximum(np.asarray(low) - local_pts, local_pts - np.asarray(high)), 0.0)
    return float(np.linalg.norm(gap, axis=1).min())


def hands_clear() -> float:
    hands = STATE.get("hands")
    if not hands:
        return 0.0
    points = np.vstack([hands[s] for s in SIDES if s in hands])
    return min(_box_distance(points, name) for name in PARTS)


def grippers_open() -> bool:
    """Both grippers open to 60 per cent or more of their widest (the kit reads the opening as that share)."""
    try:
        return all(float(mk.gripper(side)[1]) >= P.GRIP_OPEN for side in SIDES)
    except KeyError:  # no reading before the first control step
        return False


# --- the score ------------------------------------------------------------------------------------------------------------
def full_now() -> bool:
    """The 1.0 state at this step, before the hold: stored at the tip, seated on the standoff, level, in the cradle,
    both hands open and clear."""
    return (stored_at_tip() and seated() and level_deg() <= P.LEVEL_DEG and in_rest()
            and grippers_open() and hands_clear() >= P.HANDS_CLEAR)


@mk.on_step
def step() -> None:
    if mk.MECH.environment is not None:
        read_hands()
    if STATE.get("zero") is None:
        STATE["zero"] = zero_now()
    if STATE.get("extracted") is None:  # the middle barrel slid fully off the front of the shaft
        end_a, end_b = (local(e)[0] for e in barrel_ends(WORK))
        if end_a < -0.001 and end_b < -0.001:
            STATE["extracted"] = round(mk.now(), 2) if mk.MECH.environment is not None else 0.0
    if full_now():
        if STATE.get("hold", 0) == 0:
            STATE["hold_from"] = driver_pose()[0].copy()
        STATE["hold"] = STATE.get("hold", 0) + 1
        if float(np.linalg.norm(driver_pose()[0] - STATE["hold_from"])) > P.HOLD_DRIFT:
            STATE["hold"], STATE["hold_from"] = 0, None
    else:
        STATE["hold"], STATE["hold_from"] = 0, None


def level() -> float:
    if STATE.get("zero") is not None or zero_now() is not None:
        return 0.0
    if STATE.get("hold", 0) >= P.HOLD_STEPS and full_now():
        return 1.0
    if STATE.get("extracted") is not None and stored_at_tip():
        return 0.5
    if STATE.get("extracted") is not None:
        return 0.25
    return 0.0


def score(objects) -> float:
    """0 for good if at any step the driver or any socket barrel falls off the bench (its centre more than 30 mm
    below the bench top); benign contact with the mat, the stands or the cradles costs nothing. 0.25: during the
    trial the green/blue barrel is slid fully off the front of the shaft (both faces more than 1 mm outboard of the
    tip), which the red/yellow barrel blocks until it is off first. 0.5: 0.25 met, and at the end all three barrels
    sit on the shaft (each coaxial within 10 degrees and 2 mm, the plain part of its through-bore wholly between tip
    and grip nose), the green/blue barrel at the working tip with its blue end outboard within 15 degrees and its
    blue face no more than 3 mm short of the tip. 1.0: 0.5 met, and at the end the blue socket is seated on the
    standoff (its mouth 5 to 11 mm past the standoff's near face, axis within 5 degrees of the standoff's, centre
    within 2.5 mm of it, hex within 4.4 degrees of the standoff's flats: the fit's 3.9 degrees of play and 0.5), the
    driver level within 8 degrees, the handle down in the end cradle (its centre within 25 mm of the seated position
    and within 6 mm of the cradle's seat height), both grippers open to 60 per cent or more and every hull corner of
    both hands 50 mm or more from the tool, the barrels, the stands, the cradles and the bracket, held so for 2.0 s
    with the tool moving 3 mm at most."""
    return level()


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out = {"zero": STATE.get("zero"), "extracted at (s)": STATE.get("extracted"), "hold steps": STATE.get("hold", 0),
           "score": score(None)}
    if mk.MECH.poses:
        at, into = mouth()
        depth = float(np.dot(POST_AT - at, into))
        out.update({"level (deg)": round(level_deg(), 2), "blue out (deg)": round(tilt_deg(barrel_axis(WORK), driver_pose()[1][:, 0]), 1),
                    "face at tip (mm)": round(1000 * face_at_tip(), 1), "stored": {b.key: on_shaft(b.key) for b in P.BARRELS},
                    "stored at tip": stored_at_tip(), "seat depth (mm)": round(1000 * depth, 1), "seated": seated(), "in rest": in_rest(),
                    "handle centre (m)": [round(float(v), 4) for v in driver_pose()[0] + driver_pose()[1] @ np.array([P.TIP_TO_MID, 0.0, 0.0])],
                    "grippers open": grippers_open(), "hands clear (mm)": round(1000 * hands_clear(), 1)})  # fmt: skip
    return out
