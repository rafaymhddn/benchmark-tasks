"""Tympanic thermometer probe-cover turnaround: eject the used amber cover into the grey disposal tray without
touching it, press a fresh white cover from the cradle's rack onto the bare probe, and dock the thermometer in its
cradle (Riester ri-thermo tymPRO Art. 1835, covers Art. 12640; figures in ``parts.py``, bodies in ``scene.py``).

All bodies are rigid bodies in PhysX; nothing uses ``write_pose`` or ``set_joint``. Pressing ``eject_btn`` >= 4.0 mm
into the head against its 900 N/m return spring cuts ``weld_used`` at the live pose, so ``cover_used`` slides off the
angled probe; pressing the bare probe coaxially (tip >= 10 mm deep, <= 3.5 mm off axis, <= 14 deg) into a rack cover
until its 14.0 mm ring presses the cover's 13.8 mm mouth with >= 3.5 N cuts ``weld_well_i`` and makes ``weld_cone_i``
at the live pose.
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
INSTRUCTION = ("Eject the used amber probe cover into the grey tray without touching it or letting it touch the cradle "
               "or the white covers, press one fresh white cover from the rack onto the probe and keep it on, rest the "
               "thermometer in its cradle, and let go with both hands clear.")  # fmt: skip
RUN_LIMIT_STEPS = 1300  # 65 s at 20 Hz
TOP = TABLE_TOP[SCENE]
SIDES = ("left", "right")

scene.build(TOP)
OBJECTS = list(mk.MECH.bodies.values())
STATE: dict = {}


def begin(seed: int | None = None) -> dict:
    STATE.clear()
    STATE.update(seed=seed, zero=None, hold=0, hold_from=None, hold_tier=None, hands=None, peak_seat_n=0.0, peak_btn_mm=0.0)
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)


# --- geometry and mechanism helpers ----------------------------------------------------------------------------------------
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


def off_bench(name: str) -> bool:
    pts = corners(name)
    return bool(pts[:, 0].min() < P.BENCH_X[0] or pts[:, 0].max() > P.BENCH_X[1]
                or np.abs(pts[:, 1]).max() > P.BENCH_HALF_Y or pts[:, 2].min() < TOP - 0.010)


def mounted_fresh() -> str | None:
    if not mk.is_cut("weld_used"):
        return None
    for i, name in enumerate(P.FRESH):
        if not mk.is_cut(f"weld_cone_{i}"):
            return name
    return None


def cone_in_cover(cover_name: str) -> tuple[float, float, float]:
    """(radial xy error m, tip z in cover frame m, axis angle deg) of the thermometer's probe cone in ``cover_name``."""
    tp, tr = mk.pose("thermometer")
    cp, cr = mk.pose(cover_name)
    cone_tip_w = tp + tr @ P.CONE_TIP_T
    local = (cone_tip_w - cp) @ cr
    radial = float(math.hypot(float(local[0]), float(local[1])))
    cos_ang = float(np.clip((tr @ P.PROBE_UP_T) @ cr[:, 2], -1.0, 1.0))
    return radial, float(local[2]), float(math.degrees(math.acos(cos_ang)))


def step_mechanism() -> None:
    """Eject on button stroke >= 4.0 mm; seat a fresh rack cover on a coaxial >= 3.5 N press of the probe."""
    stroke = float(mk.joint("eject_slide"))
    STATE["peak_btn_mm"] = max(STATE.get("peak_btn_mm", 0.0), 1000.0 * stroke)
    if stroke >= P.EJECT_TRAVEL:
        if not mk.is_cut("weld_used"):
            mk.cut("weld_used")
        for i in range(len(P.FRESH)):
            if not mk.is_cut(f"weld_cone_{i}"):
                mk.cut(f"weld_cone_{i}")
        return
    if mk.is_cut("weld_used") and mounted_fresh() is None and stroke < 0.0020:
        for i, name in enumerate(P.FRESH):
            if mk.is_cut(f"weld_well_{i}"):
                continue
            force = float(mk.contact_force(name, "thermometer"))
            STATE["peak_seat_n"] = max(STATE.get("peak_seat_n", 0.0), force)
            radial, tip_z, tilt = cone_in_cover(name)
            if force >= P.SEAT_FORCE and radial <= P.SEAT_RADIAL_TOL and -0.002 <= tip_z <= P.COVER_H - P.SEAT_DEPTH_MIN and tilt <= P.SEAT_TILT_DEG:
                mk.cut(f"weld_well_{i}")
                mk.make(f"weld_cone_{i}")
                break


# --- whole-trial zero rules and end-state predicates -----------------------------------------------------------------------
def zero_now() -> str | None:
    if mk.touching("cover_used", "left robot") or mk.touching("cover_used", "right robot"):
        return "a hand touched the used cover"
    if mk.touching("cover_used", "cradle") or any(mk.touching("cover_used", f) for f in P.FRESH):
        return "the used cover touched the cradle or a fresh cover"
    if mk.is_cut("weld_used") and mk.touching("cover_used", "scene"):
        return "the used cover missed the grey tray"
    for i in range(len(P.FRESH)):
        if mk.is_cut(f"weld_well_{i}") and mk.is_cut(f"weld_cone_{i}"):
            return "a fresh cover came off the probe again"
    for n in ("thermometer", "cradle", "disposal_tray", "cover_used"):
        if off_bench(n):
            return f"the {n} fell off the bench"
    return None


def used_in_tray() -> bool:
    """``cover_used`` has been ejected and rests on the floor inside ``disposal_tray``'s inner walls."""
    if not mk.is_cut("weld_used"):
        return False
    local = in_frame(corners("cover_used"), "disposal_tray")
    half = np.array([(P.TRAY_L - 2.0 * P.TRAY_WALL) / 2.0, (P.TRAY_W - 2.0 * P.TRAY_WALL) / 2.0])
    z_min, z_max = float(local[:, 2].min()), float(local[:, 2].max())
    return bool(np.all(np.abs(local[:, :2]) <= half + 1e-4) and P.TRAY_FLOOR - 0.002 <= z_min <= P.TRAY_FLOOR + 0.006 and z_max <= P.TRAY_H + 0.004)


def docked_in_cradle() -> bool:
    """``thermometer`` lies level along ``cradle`` on its end wall and rib, probe towards the pocket end."""
    tp, tr = mk.pose("thermometer")
    _, cr = mk.pose("cradle")
    local_p = in_frame(tp[None, :], "cradle")[0]
    xy_err = math.hypot(float(local_p[0] - P.DOCK_X_OFFSET), float(local_p[1] - P.DOCK_Y_OFFSET))
    z_err = abs(float(local_p[2] - P.DOCK_SADDLE_Z))
    tilt = math.degrees(math.acos(float(np.clip(tr[2, 2], -1.0, 1.0))))
    forward = float(tr[:, 1] @ cr[:, 1]) >= math.cos(math.radians(15.0))
    return xy_err <= P.DOCK_XY_TOL and z_err <= P.DOCK_Z_TOL and tilt <= P.DOCK_TILT_DEG and forward


# --- hand clearance --------------------------------------------------------------------------------------------------------
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
    hands = STATE.get("hands")
    if not hands:
        return 0.0
    points = np.vstack([hands[s] for s in SIDES if s in hands])
    return min(_box_distance(points, n) for n in P.BODIES)


# --- the score -------------------------------------------------------------------------------------------------------------
def raw_tier() -> float:
    if not used_in_tray():
        return 0.0
    fresh = mounted_fresh()
    if fresh is not None:
        radial, tip_z, tilt = cone_in_cover(fresh)
        seated = radial <= 0.0040 and -0.003 <= tip_z <= P.COVER_H - 0.009 and tilt <= 15.0
        if seated and docked_in_cradle():
            return 1.0
    return 0.5


def end_gate_ok() -> bool:
    return hands_clear() >= P.HANDS_CLEAR and abs(float(mk.joint("eject_slide"))) <= P.BTN_RECOIL_MAX


@mk.on_step
def step() -> None:
    if mk.MECH.environment is not None:
        read_hands()
    step_mechanism()
    if STATE.get("zero") is None:
        STATE["zero"] = zero_now()
    tier = raw_tier()
    if tier > 0.0 and end_gate_ok() and STATE.get("zero") is None:
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
    if tier > 0.0 and end_gate_ok() and STATE.get("hold_tier") == tier and STATE.get("hold", 0) >= P.HOLD_STEPS:
        return tier
    return 0.0


def score(objects) -> float:
    """0 for good if either hand ever touches the used amber probe cover, if the used cover touches the cradle or
    any fresh cover, if the ejected used cover touches the bench outside the grey tray, if a fresh cover once on the
    probe comes off it again, or if a part falls off the bench. Otherwise read at the end, with every part of both
    hands >= 30 mm clear of all task bodies, the eject button back within 1.0 mm of rest and every body moving at
    most 3 mm over 1.0 s (20 steps): 0.5 when the used cover rests on the floor inside the grey tray's walls; 1.0
    when, as well, one fresh white cover sits on the probe (tip <= 4.0 mm off its axis, within 15 deg, >= 9 mm
    inside) and the thermometer lies in its cradle (within 12 mm along the bench, 6 mm in height, 10 deg of level,
    probe towards the pocket end). 0 otherwise."""
    return level()


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out = {
        "zero": STATE.get("zero"),
        "hold steps": STATE.get("hold", 0),
        "peak button stroke (mm)": round(float(STATE.get("peak_btn_mm", 0.0)), 2),
        "peak seat force (N)": round(float(STATE.get("peak_seat_n", 0.0)), 2),
    }
    if mk.MECH.poses:
        out.update({
            "used ejected": mk.is_cut("weld_used"),
            "used in tray": used_in_tray(),
            "mounted fresh": mounted_fresh(),
            "docked in cradle": docked_in_cradle(),
            "button recoil (mm)": round(1000.0 * float(mk.joint("eject_slide")), 2),
            "hands clear (mm)": round(1000.0 * hands_clear(), 1),
            "score": score(None),
        })
    return out
