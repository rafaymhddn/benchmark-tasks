"""A polypropylene patient tray stands on the bench outside the MRI scanner doorway (the bench stands in for the
wheeled cart's top). Across the full width of the bench on the cart's far side runs a red doorway threshold line
with a flat sign reading MRI ZONE IV beyond it; on the near safe side, a yellow bin stands on a green-bordered
yellow holding mat reading MR UNSAFE ITEMS / HOLD HERE / OUTSIDE ZONE IV (with HOLD on the bin's floor). On the
tray three ferromagnetic items lie among three all-plastic ones: a ductile-iron clamp standing on its foot, a
laminated-steel padlock and forged-steel dressmaker shears (bent handles up), between an rPET water bottle, a PP
emesis basin and an all-plastic A5 clipboard.

The steel items must leave the tray for the holding bin without ever crossing the doorway line or leaving the
bench; the plastic ones must be back on the tray, upright, at the end (moving or lifting them on the way costs
nothing). Everything is a free rigid body; nothing is jointed, welded, pushed or latched.
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
SUCCESS_HOLD_S = 2.0
INSTRUCTION = "Get the cart ready to go into the MRI room with the patient, then move both hands clear."
RUN_LIMIT_STEPS = 1400  # 70 s at 20 Hz
TOP = TABLE_TOP[SCENE]
SIDES = ("left", "right")
STEEL, SAFE = P.STEEL, P.SAFE

scene.build(TOP)
OBJECTS = list(mk.MECH.bodies.values())
STATE: dict = {}


def begin(seed: int | None = None) -> dict:
    STATE.clear()
    STATE.update(seed=seed, zero=None, hold=0, hold_from=None, hold_tier=None, hands=None)
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)


# --- geometry the rules read ----------------------------------------------------------------------------------------------
def corners(name: str) -> np.ndarray:
    """Every solid shape's eight corners of body ``name``, in the world."""
    position, rotation = mk.pose(name)
    pts = []
    for s in mk.MECH.bodies[name].shapes:
        if s.solid:
            r, h, c = mk.quat_to_matrix(s.quat), s.half_extents(), np.array(s.pos, dtype=float)
            grid = np.array([[sx, sy, sz] for sx in (-h[0], h[0]) for sy in (-h[1], h[1]) for sz in (-h[2], h[2])])
            pts.append(grid @ r.T + c)
    return np.vstack(pts) @ rotation.T + position


def in_frame(points: np.ndarray, name: str) -> np.ndarray:
    position, rotation = mk.pose(name)
    return (points - position) @ rotation


def in_bin(name: str) -> bool:
    """Every solid corner of ``name`` inside the bin's opening rectangle and at least BIN_SINK under its rim."""
    local = in_frame(corners(name), "bin")
    return bool(np.all(np.abs(local[:, :2]) <= np.array([(P.BIN_L - 2.0 * P.BIN_WALL) / 2.0, (P.BIN_W - 2.0 * P.BIN_WALL) / 2.0]) + 1e-6)
                and local[:, 2].max() <= P.BIN_H - P.BIN_SINK)


def binned() -> list[str]:
    return [n for n in STEEL if in_bin(n)]


def inside_tray_xy(name: str) -> bool:
    """Whether body ``name``'s frame origin (a safe item's base centre) lies at least SAFE_INSET inside the tray's
    outer edge."""
    position, _ = mk.pose(name)
    local_centre = in_frame(position[None, :], "tray")[0]
    return bool(abs(local_centre[0]) <= P.TRAY_D / 2.0 - P.SAFE_INSET
                and abs(local_centre[1]) <= P.TRAY_W / 2.0 - P.SAFE_INSET)


def floor_gap(name: str) -> float:
    """m from the tray floor up to body ``name``'s lowest solid corner, in the tray's frame (negative: under it)."""
    return float(in_frame(corners(name), "tray")[:, 2].min()) - P.TRAY_WALL


def tilt(name: str) -> float:
    """Degrees between body ``name``'s up axis as built (the bottle's axis, the dish's and the clipboard's normals)
    and the vertical."""
    up = mk.pose(name)[1] @ (mk.built_pose(name)[1].T @ np.array([0.0, 0.0, 1.0]))
    return math.degrees(math.acos(min(1.0, max(-1.0, float(up[2])))))


def on_tray(name: str) -> bool:
    """``name`` rests on the tray: its frame origin over the tray, its lowest corner within SAFE_REST of the floor."""
    return inside_tray_xy(name) and abs(floor_gap(name)) <= P.SAFE_REST


def displaced(name: str) -> float:
    """How far body ``name``'s centre has moved from where it was built, in the horizontal."""
    now, built = mk.pose(name)[0], np.asarray(mk.built_pose(name)[0])
    return float(math.hypot(float(now[0] - built[0]), float(now[1] - built[1])))


def safe_off(name: str) -> str | None:
    """Why safe item ``name`` fails the end test of every positive level at this step, or None when it is back on the
    tray: base centre at least 5 mm inside the outer edge, lowest point within 10 mm of the floor, tilted at most 20
    degrees. Moving or lifting it on the way is never penalised."""
    if not inside_tray_xy(name):
        return f"the {name} is off the tray"
    if abs(floor_gap(name)) > P.SAFE_REST:
        return f"the {name} is not resting on the tray floor"
    if tilt(name) > P.SAFE_TILT:
        return f"the {name} is tipped over"
    return None


def past_door(name: str) -> bool:
    """Any solid corner of ``name`` reaches or crosses the red Zone IV doorway threshold line."""
    return bool(corners(name)[:, 0].max() >= P.DOOR_LINE_X)


def off_bench(name: str) -> bool:
    """Any solid corner of ``name`` leaves the bench's horizontal rectangle or drops below the bench top."""
    pts = corners(name)
    return bool(pts[:, 0].min() < P.BENCH_X[0] or pts[:, 0].max() > P.BENCH_X[1]
                or np.abs(pts[:, 1]).max() > P.BENCH_HALF_Y or pts[:, 2].min() < TOP - 0.010)


def cleared(name: str) -> bool:
    """Every solid corner of ``name`` is clear of the tray's outer rectangle and resting on the safe bench."""
    pts = corners(name)
    tray_pts = in_frame(pts, "tray")
    outside_tray = bool(np.all((np.abs(tray_pts[:, 0]) > P.TRAY_D / 2.0) | (np.abs(tray_pts[:, 1]) > P.TRAY_W / 2.0)))
    resting = float(pts[:, 2].min()) <= TOP + P.BENCH_REST_Z
    return outside_tray and resting and not past_door(name) and not off_bench(name)


def zero_now() -> str | None:
    """The whole-trial latch, steel items only: a safe item may be moved or lifted (safe_off reads it at the end)."""
    for n in STEEL:
        if past_door(n):
            return f"the {n} crossed the Zone IV line"
        if off_bench(n):
            return f"the {n} went off the bench"
    return None


# --- the hands -------------------------------------------------------------------------------------------------------------
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


def _box_distance(points: np.ndarray, name: str) -> float:
    position, rotation = mk.pose(name)
    low, high = mk.MECH.bodies[name].bounds()
    local = (points - position) @ rotation
    gap = np.maximum(np.maximum(np.asarray(low) - local, local - np.asarray(high)), 0.0)
    return float(np.linalg.norm(gap, axis=1).min())


def hands_clear() -> float:
    """m from the nearest hull corner of either hand to the tray, the bin or any item."""
    hands = STATE.get("hands")
    if not hands:
        return 0.0
    points = np.vstack([hands[s] for s in SIDES if s in hands])
    return min(_box_distance(points, n) for n in ("tray", "bin", *STEEL, *SAFE))


# --- the score --------------------------------------------------------------------------------------------------------------
def raw_tier() -> float:
    if any(safe_off(n) is not None for n in SAFE):  # every positive level: all three safe items back on the tray
        return 0.0
    count = len(binned())
    if count == 3:
        return 1.0
    if count == 2 and all(cleared(n) for n in STEEL if n not in binned()):
        return 0.5
    return 0.0


@mk.on_step
def step() -> None:
    if mk.MECH.environment is not None:
        read_hands()
    if STATE.get("zero") is None:
        STATE["zero"] = zero_now()
    tier = raw_tier()
    if tier > 0.0 and hands_clear() >= P.HANDS_CLEAR and STATE.get("zero") is None:
        if STATE.get("hold", 0) == 0 or STATE.get("hold_tier") != tier:
            STATE["hold_from"] = {n: mk.pose(n)[0].copy() for n in P.BODIES}
            STATE["hold_tier"] = tier
            STATE["hold"] = 0
        STATE["hold"] += 1
        if max(float(np.linalg.norm(mk.pose(n)[0] - at)) for n, at in STATE["hold_from"].items()) > P.HOLD_DRIFT:
            STATE["hold"], STATE["hold_from"], STATE["hold_tier"] = 0, None, None
    else:
        STATE["hold"], STATE["hold_from"], STATE["hold_tier"] = 0, None, None


def level() -> float:
    if STATE.get("zero") is not None or zero_now() is not None:
        return 0.0
    tier = raw_tier()
    if tier > 0.0 and hands_clear() >= P.HANDS_CLEAR and STATE.get("hold_tier") == tier and STATE.get("hold", 0) >= P.HOLD_STEPS:
        return tier
    return 0.0


def score(objects) -> float:
    """0 for good if any steel item (clamp, padlock, shears) ever has any solid corner cross the red Zone IV doorway
    line (x >= 62.5 cm) or leave the bench; moving or lifting a safe item (bottle, dish, clipboard) never zeroes.
    Otherwise read at the end: every positive score level requires all three safe items back on the tray (base
    centre >= 5 mm inside the tray's 35 x 45 cm outer edge, lowest point within 10 mm of the tray floor, tilted
    <= 20 degrees from vertical as it started: bottle standing, dish open side up, clipboard face up), every part of
    both hands >= 30 mm clear of the tray, the bin and every item, and every body moving at most 3 mm over 1.0 s
    (20 steps). 1.0 when all three steel items lie wholly in the bin (every solid corner inside its opening and
    >= 5 mm under the rim). 0.5 when two steel items lie wholly in the bin and the third is cleared on the safe side
    of the bench (every solid corner outside the tray's outer rectangle, resting with its lowest point <= 10 mm over
    the bench top). 0 otherwise."""
    return level()


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out = {"zero": STATE.get("zero"), "hold steps": STATE.get("hold", 0),
           "in bin": binned(), "safe off tray": {n: safe_off(n) for n in SAFE}}
    if mk.MECH.poses:
        out.update({"on tray": {n: on_tray(n) for n in (*STEEL, *SAFE)},
                    "cleared": {n: cleared(n) for n in STEEL},
                    "moved (mm)": {n: round(1000.0 * displaced(n), 1) for n in (*STEEL, *SAFE)},
                    "safe tilt (deg)": {n: round(tilt(n), 1) for n in SAFE},
                    "safe floor gap (mm)": {n: round(1000.0 * floor_gap(n), 1) for n in SAFE},
                    "centres (m)": {n: [round(float(v), 4) for v in mk.pose(n)[0][:2]] for n in (*STEEL, *SAFE)},
                    "hands clear (mm)": round(1000.0 * hands_clear(), 1), "score": score(None)})  # fmt: skip
    return out
