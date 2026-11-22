"""Convert a seca 354 dual-purpose infant scale into a flat toddler scale: hold the front release button down while
sliding the curved weighing tray back off the scale base without moving the base off its marks, set the detached tray
upright on the grey IKEA mat, then let go and move both hands clear.

Every lock and release is pure rigid-body contact (scene.py, constants in parts.py): pulling the tray up before its
20 mm slide lifts the 1.47 kg scale base off its marks by the two keyhole studs; pushing it back without holding the
sprung release button runs its tongue into the button's pawl. Nothing in task.py or scene.py welds, pushes or snaps poses.
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
SUCCESS_HOLD_S = 3.5
INSTRUCTION = ("Hold the baby scale's front release button in while sliding the curved weighing tray back off the scale "
               "base without moving the base off its marks, set the tray upright on the grey mat, then let go and move "
               "both hands clear.")  # fmt: skip
RUN_LIMIT_STEPS = P.RUN_LIMIT_STEPS  # 1,200 steps, 60 s at 20 Hz
TOP = TABLE_TOP[SCENE]
SIDES = ("left", "right")
SCORED_BODIES = ("scale_base", "release_btn", "weighing_tray", "mat")

scene.build(TOP)
OBJECTS = list(mk.MECH.bodies.values())
STATE: dict = {}


def begin(seed: int | None = None) -> dict:
    STATE.clear()
    STATE.update(seed=seed, zero=None, hold=0, hold_tray_xy=None, hold_base_xy=None, prev_tray_vz=0.0,
                 worst_base_lift=0.0, worst_base_shift=0.0, peak_btn_mm=0.0, hands=None)
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)


# --- physical observables ---------------------------------------------------------------------------------------------
def button_depression() -> float:
    """m downward travel of ``release_btn`` relative to ``scale_base`` (0 at top rest)."""
    return float(mk.joint("btn_slide"))


def tilt_deg(name: str) -> float:
    _, rot = mk.pose(name)
    return math.degrees(math.acos(float(np.clip(rot[2, 2], -1.0, 1.0))))


def base_shift() -> float:
    """m horizontal shift of ``scale_base`` from its built mark."""
    pos, _ = mk.pose("scale_base")
    return float(math.hypot(pos[0] - P.BASE_X, pos[1] - P.BASE_Y))


def base_corner_lift() -> float:
    """m highest lift above the bench among ``scale_base``'s four bottom corners."""
    pos, rot = mk.pose("scale_base")
    corners_w = P.base_corners_local() @ rot.T + pos
    return float(max(0.0, (corners_w[:, 2] - TOP).max()))


def base_undisturbed() -> bool:
    return (base_corner_lift() <= P.BASE_LIFT_MAX and base_shift() <= P.BASE_SHIFT_MAX
            and tilt_deg("scale_base") <= P.BASE_TILT_MAX_DEG)


def tray_slide_back() -> float:
    """m backward (+x) displacement of ``weighing_tray`` relative to ``scale_base`` along ``scale_base``'s +x."""
    b_pos, b_rot = mk.pose("scale_base")
    t_pos, _ = mk.pose("weighing_tray")
    rel = b_rot.T @ (t_pos - b_pos)
    return float(rel[0] - (P.TRAY_X - P.BASE_X))


def tray_lowest_z() -> float:
    """World z of the lowest point of ``weighing_tray``'s bounding box."""
    t_pos, t_rot = mk.pose("weighing_tray")
    lo, hi = mk.MECH.bodies["weighing_tray"].bounds()
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    return float((corners @ t_rot.T + t_pos)[:, 2].min())


def tray_separated() -> bool:
    """``weighing_tray`` unlocked and separated from ``scale_base``: either slid >= 20 mm back (both keyhole studs
    free) or clear of ``scale_base`` in x, y (moved aside) or z with no contact on ``scale_base`` or ``release_btn``."""
    touch_scale = mk.contact_force("weighing_tray", "scale_base") + mk.contact_force("weighing_tray", "release_btn")
    t_pos, _ = mk.pose("weighing_tray")
    away = (abs(t_pos[0] - P.BASE_X) > (P.BASE_D + P.TRAY_D) / 2
            or abs(t_pos[1] - P.BASE_Y) > (P.BASE_W + P.TRAY_W) / 2
            or (t_pos[2] - TOP) > P.BASE_H + 0.025)
    return bool(tray_slide_back() >= P.SEPARATED_SLIDE or (away and touch_scale < 0.05))


def tray_on_mat_upright() -> bool:
    """``weighing_tray`` detached from ``scale_base`` and resting upright, centred on the flat ``mat``."""
    if mk.contact_force("weighing_tray", "scale_base") > 0.05 or mk.contact_force("weighing_tray", "release_btn") > 0.05:
        return False
    m_pos, m_rot = mk.pose("mat")
    t_pos, _ = mk.pose("weighing_tray")
    rel = m_rot.T @ (t_pos - m_pos)
    centred = abs(rel[0]) <= P.MAT_CENTER_TOL_X and abs(rel[1]) <= P.MAT_CENTER_TOL_Y
    # The support ribs reach -RIB_Z0 below the tray origin: rel[2] is 28 mm on the mat floor, 42 mm across the rims
    resting_low = (P.MAT_FLOOR_T - scene.RIB_Z0 - 0.005 <= rel[2] <= P.MAT_H - scene.RIB_Z0 + 0.005
                   and mk.contact_force("weighing_tray", "mat") >= 1.0)
    return bool(centred and resting_low and tilt_deg("weighing_tray") <= P.TRAY_UPRIGHT_DEG
                and tilt_deg("mat") <= P.MAT_TILT_MAX_DEG)


# --- hand hull clearance ----------------------------------------------------------------------------------------------
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
    return min(_box_distance(points, name) for name in SCORED_BODIES)


# --- whole-trial zero rules and end-state scoring ---------------------------------------------------------------------
def tray_bench_fall() -> bool:
    """True if ``weighing_tray``, ``scale_base``, or ``mat`` fell off the bench, or ``weighing_tray`` flipped
    upside-down (> 90 deg) or slammed onto the bare bench (> 0.50 m/s)."""
    if (tray_lowest_z() < TOP - 0.020 or mk.pose("scale_base")[0][2] < TOP - 0.020
            or mk.pose("mat")[0][2] < TOP - 0.020):
        return True
    if tilt_deg("weighing_tray") > 90.0:
        return True
    lin, _ = mk.velocity("weighing_tray")
    vz = min(float(lin[2]), float(STATE.get("prev_tray_vz", 0.0)))
    on_bare_bench = (tray_lowest_z() <= TOP + 0.008
                     and mk.contact_force("weighing_tray", "mat") < 0.5
                     and mk.contact_force("weighing_tray", "scale_base") < 0.5)
    return bool(on_bare_bench and vz < -P.BENCH_DROP_SPEED)


def zero_now() -> str | None:
    if base_corner_lift() > P.BASE_LIFT_MAX:
        return "the scale base was lifted off its marks"
    if base_shift() > P.BASE_SHIFT_MAX:
        return "the scale base was shifted off its marks"
    if tray_bench_fall():
        return "the weighing tray fell onto or off the bench"
    return None


def all_still() -> bool:
    for name in ("scale_base", "weighing_tray", "mat"):
        lin, ang = mk.velocity(name)
        if float(np.linalg.norm(lin)) >= P.REST_LIN or float(np.linalg.norm(ang)) >= P.REST_ANG:
            return False
    return True


def full_now() -> bool:
    return (base_undisturbed() and tray_on_mat_upright() and button_depression() <= P.BTN_REST_MAX
            and hands_clear() >= P.HANDS_CLEAR and all_still())


@mk.on_step
def step() -> None:
    if mk.MECH.environment is not None:
        read_hands()
    STATE["worst_base_lift"] = max(STATE.get("worst_base_lift", 0.0), base_corner_lift())
    STATE["worst_base_shift"] = max(STATE.get("worst_base_shift", 0.0), base_shift())
    STATE["peak_btn_mm"] = max(STATE.get("peak_btn_mm", 0.0), 1000.0 * button_depression())
    if STATE.get("zero") is None:
        STATE["zero"] = zero_now()
    STATE["prev_tray_vz"] = float(mk.velocity("weighing_tray")[0][2])
    if full_now():
        t_xy = mk.pose("weighing_tray")[0][:2].copy()
        b_xy = mk.pose("scale_base")[0][:2].copy()
        if STATE.get("hold", 0) == 0:
            STATE["hold_tray_xy"], STATE["hold_base_xy"] = t_xy, b_xy
        STATE["hold"] = STATE.get("hold", 0) + 1
        drift = max(float(np.linalg.norm(t_xy - STATE["hold_tray_xy"])),
                    float(np.linalg.norm(b_xy - STATE["hold_base_xy"])))
        if drift > P.HOLD_DRIFT:
            STATE["hold"], STATE["hold_tray_xy"], STATE["hold_base_xy"] = 0, None, None
    else:
        STATE["hold"], STATE["hold_tray_xy"], STATE["hold_base_xy"] = 0, None, None


def level() -> float:
    if STATE.get("zero") is not None or zero_now() is not None:
        return 0.0
    if STATE.get("hold", 0) >= P.HOLD_STEPS and full_now():
        return 1.0
    if tray_separated() and base_undisturbed():
        return 0.5
    return 0.0


def score(objects) -> float:
    """0 for good if at any step the scale base lifts > 8.0 mm at a corner or shifts > 15.0 mm from its marks, or if
    the weighing tray falls onto the bench (> 0.50 m/s downward bench impact), flips upside-down (> 90 deg), or any
    body falls off the bench. Otherwise at the end: 1.0 when the detached tray rests upright (tilt <= 8.0 deg) on the
    flat grey mat (mat tilt <= 5.0 deg, tray centre within 3.0 cm front-to-back and 5.0 cm sideways of the mat's
    centre by ruler against the 29.0 x 37.0 cm outer edges), the release button is back at rest (<= 1.0 mm), the scale
    base is undisturbed (<= 15.0 mm, <= 3.0 deg), both hands are >= 30.0 mm clear of scale, button, tray and mat, and
    all bodies stay at rest (< 20 mm/s, < 0.20 rad/s, <= 2.0 mm drift) for 1.0 s. 0.5 when 1.0 is not met while the
    tray is unlocked and separated from the scale base (slid >= 20.0 mm back or moved clear) with the base undisturbed.
    0.0 otherwise."""
    return level()


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out = {
        "zero": STATE.get("zero"),
        "hold_steps": STATE.get("hold", 0),
        "worst_base_lift_mm": round(1000 * STATE.get("worst_base_lift", 0.0), 2),
        "worst_base_shift_mm": round(1000 * STATE.get("worst_base_shift", 0.0), 2),
        "peak_btn_mm": round(STATE.get("peak_btn_mm", 0.0), 2),
    }
    if mk.MECH.poses:
        out.update({
            "btn_mm": round(1000 * button_depression(), 2),
            "tray_slide_mm": round(1000 * tray_slide_back(), 2),
            "base_lift_mm": round(1000 * base_corner_lift(), 2),
            "base_shift_mm": round(1000 * base_shift(), 2),
            "tray_tilt_deg": round(tilt_deg("weighing_tray"), 2),
            "separated": tray_separated(),
            "on_mat_upright": tray_on_mat_upright(),
            "tray_mat_n": round(float(mk.contact_force("weighing_tray", "mat")), 2),
            "tray_base_n": round(float(mk.contact_force("weighing_tray", "scale_base")), 2),
            "hands_clear_mm": round(1000 * hands_clear(), 1),
            "score": score(None),
        })
    return out
