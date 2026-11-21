"""Two die-cut blue polyethylene foam end-cap cushions start nested inside an open-top 254 x 152.4 x 76.2 mm shipping
carton (standard warehouse kitting), leaving only 162 mm between them so the 200 mm Interroll steel conveyor roller on
the bench cradle in front cannot enter. Taking out only one cap and plugging the roller into the other leaves 29 mm at
the far wall (16 mm past the 8 mm spindle), blocking the 46 mm second cap.

Both caps are lifted out of the carton, slipped onto opposite ends of the roller on the bench (21 mm-deep, 45 mm sockets
around the 40 mm tube; 250 mm coupled length), and lowered together into the carton (easiest with both hands squeezing
the caps inward) so the carton's end walls capture the unit with the roller hanging 16 mm suspended above the floor.
Pure rigid bodies and contacts in PhysX: no joints, no welds, no computed forces."""

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
SUCCESS_HOLD_S = 3.0  # both arms finish returning to ready so the last frame shows the packed carton clear
INSTRUCTION = (
    "Take both blue foam end-caps out of the shipping carton, fit one onto each end of the steel roller on the bench, "
    "and lower the roller with both end-caps together into the carton so the roller rests suspended off the floor."
)
RUN_LIMIT_STEPS = 1200  # 60 s at 20 Hz
TOP = TABLE_TOP[SCENE]
SIDES = ("left", "right")
CAPS = scene.CAPS
FREE_BODIES = ("roller", "cap_l", "cap_r")

scene.build(TOP)
OBJECTS = list(mk.MECH.bodies.values())
STATE: dict = {}


def begin(seed: int | None = None) -> dict:
    STATE.clear()
    STATE.update(seed=seed, zero=None, out_streak=0, both_caps_out=False, first_out_s=None, hold=0, hold_from=None, hands=None)
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)


# --- physical readings ----------------------------------------------------------------------------------------------------
def cap_tilt_deg(c: str) -> float:
    """Tilt of ``cap_c``'s vertical axis from world up (deg)."""
    _, r = mk.pose(f"cap_{c}")
    return float(math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2]))))))


def roller_tilt_deg() -> float:
    """Tilt of ``roller``'s cylinder axis from horizontal (deg)."""
    _, r = mk.pose("roller")
    return float(math.degrees(math.asin(max(-1.0, min(1.0, abs(float(r[2, 1])))))))


def cap_world_bounds(c: str) -> tuple[np.ndarray, np.ndarray]:
    """Axis-aligned world (low, high) corners of ``cap_c``'s bounding box."""
    p, r = mk.pose(f"cap_{c}")
    lo, hi = mk.MECH.bodies[f"cap_{c}"].bounds()
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]) @ r.T + p
    return corners.min(axis=0), corners.max(axis=0)


def cap_clear_of_carton(c: str) -> bool:
    """``cap_c`` is completely outside the carton's interior: its lowest point is above the carton rim, or its
    horizontal footprint lies outside the carton's outer walls."""
    lo, hi = cap_world_bounds(c)
    if lo[2] >= TOP + P.Z_RIM:
        return True
    ox = P.BOX_W / 2.0 + P.BOX_WALL
    oy = P.BOX_L / 2.0 + P.BOX_WALL
    return bool(hi[0] <= P.X_CARTON - ox or lo[0] >= P.X_CARTON + ox or hi[1] <= -oy or lo[1] >= oy)


def engagement_pair(c: str, end_sign: float) -> tuple[float, float, float]:
    """(axial insertion depth m into ``cap_c``'s 21 mm socket, radial offset m from socket axis, angle deg between
    roller axis and socket axis) for the roller tube end at ``end_sign`` (+1: +y end, -1: -y end)."""
    p_r, r_r = mk.pose("roller")
    p_c, r_c = mk.pose(f"cap_{c}")
    axis_r = r_r[:, 1] * float(end_sign)  # outward normal of this roller end
    end_w = p_r + axis_r * (P.ROLLER_L / 2.0)
    local = r_c.T @ (end_w - p_c)
    inward = scene.cap_inward(c)  # direction from cap centre to socket mouth
    # Mouth is at inward * y_local = +CAP_T/2; shoulder is SOCK_DEPTH deeper (at +CAP_T/2 - SOCK_DEPTH).
    depth = float(P.CAP_T / 2.0 - inward * local[1])
    radial = float(math.hypot(local[0], local[2] - P.SOCK_Z))
    # Coaxial when roller's outward end points into the cap (-inward along cap's y axis)
    cos_a = max(-1.0, min(1.0, float(np.dot(axis_r, -inward * r_c[:, 1]))))
    return depth, radial, float(math.degrees(math.acos(cos_a)))


def best_engagement() -> tuple[dict[str, tuple[float, float, float]], bool]:
    """Best 1-to-1 assignment of ``cap_l`` and ``cap_r`` to opposite ends of ``roller`` (the roller is symmetric)."""
    best = None
    for sign_l in (+1.0, -1.0):
        el = engagement_pair("l", sign_l)
        er = engagement_pair("r", -sign_l)
        # Prefer the pairing with the larger worst-end axial engagement (inside radial envelope)
        score_pair = min(el[0], er[0]) - 2.0 * max(el[1], er[1])
        if best is None or score_pair > best[0]:
            best = (score_pair, {"l": el, "r": er})
    pair = best[1]
    ok_half = all(
        0.0 <= d <= P.SOCK_DEPTH + 0.003 and rad <= P.RADIAL_HALF and ang <= P.COAX_MAX_DEG for d, rad, ang in pair.values()
    ) and min(pair["l"][0], pair["r"][0]) >= P.ENGAGE_HALF
    return pair, bool(ok_half)


def cap_seated_in_carton(c: str) -> bool:
    """``cap_c`` rests upright on the carton floor wholly within the carton walls and below the rim."""
    lo, hi = cap_world_bounds(c)
    floor_w = TOP + P.Z_FLOOR
    inside_xy = (
        lo[0] >= P.X_CARTON - P.BOX_W / 2.0 - 0.002
        and hi[0] <= P.X_CARTON + P.BOX_W / 2.0 + 0.002
        and lo[1] >= -P.BOX_L / 2.0 - 0.002
        and hi[1] <= P.BOX_L / 2.0 + 0.002
    )
    on_floor = abs(lo[2] - floor_w) <= P.CAP_FLOOR_TOL and hi[2] <= TOP + P.CAP_TOP_MAX_Z
    return bool(inside_xy and on_floor and cap_tilt_deg(c) <= P.CAP_TILT_MAX_DEG)


def roller_floor_gap() -> float:
    """Clear vertical height (m) from the carton's inner floor to the lowest point of the 200 mm roller tube."""
    p_r, r_r = mk.pose("roller")
    sin_tilt = abs(float(r_r[2, 1]))
    cos_tilt = math.sqrt(max(0.0, 1.0 - sin_tilt * sin_tilt))
    lowest_tube_z = float(p_r[2]) - (P.ROLLER_L / 2.0) * sin_tilt - P.ROLLER_R * cos_tilt
    return lowest_tube_z - (TOP + P.Z_FLOOR)


def roller_inside_carton_xy() -> bool:
    p_r, r_r = mk.pose("roller")
    axis = r_r[:, 1]
    for s in (-1.0, 1.0):
        end = p_r + s * (P.ROLLER_L / 2.0) * axis
        if not (P.X_CARTON - P.BOX_W / 2.0 <= end[0] <= P.X_CARTON + P.BOX_W / 2.0 and -P.BOX_L / 2.0 <= end[1] <= P.BOX_L / 2.0):
            return False
    return True


def zero_now() -> str | None:
    """Whole-trial zero rule: roller or either foam end-cap knocked off the bench."""
    for name in FREE_BODIES:
        if float(mk.pose(name)[0][2]) < TOP + P.OFF_BENCH_Z:
            return f"{name} fell off the bench"
    return None


# --- hand hull clearances -------------------------------------------------------------------------------------------------
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
    """Nearest hull distance (m) from either hand to the carton, the roller, or either foam end-cap."""
    hands = STATE.get("hands")
    if not hands:
        return 0.0
    points = np.vstack([hands[s] for s in SIDES if s in hands])
    return min(_box_distance(points, name) for name in ("carton", *FREE_BODIES))


# --- scoring --------------------------------------------------------------------------------------------------------------
def suspended_now() -> bool:
    """The 1.0 physical arrangement before the 1.0 s stillness hold."""
    if not STATE.get("both_caps_out", False):
        return False
    if not (cap_seated_in_carton("l") and cap_seated_in_carton("r") and roller_inside_carton_xy()):
        return False
    pair, _ = best_engagement()
    engaged = all(
        P.ENGAGE_FULL <= d <= P.SOCK_DEPTH + 0.003 and rad <= P.RADIAL_FULL and ang <= P.COAX_MAX_DEG for d, rad, ang in pair.values()
    )
    suspended = (
        roller_tilt_deg() <= P.ROLLER_TILT_MAX_DEG
        and roller_floor_gap() >= P.SUSPEND_CLEAR_MIN
        and mk.contact_force("roller", "carton") < 0.1
    )
    return bool(engaged and suspended and hands_clear() >= P.HANDS_CLEAR)


@mk.on_step
def step() -> None:
    if mk.MECH.environment is not None:
        read_hands()
    if STATE.get("zero") is None:
        STATE["zero"] = zero_now()
    if cap_clear_of_carton("l") and cap_clear_of_carton("r"):
        STATE["out_streak"] = STATE.get("out_streak", 0) + 1
        if STATE["out_streak"] >= P.OUT_HOLD_STEPS and not STATE.get("both_caps_out", False):
            STATE["both_caps_out"] = True
            STATE["first_out_s"] = mk.now() if mk.MECH.environment is not None else 0.0
    else:
        STATE["out_streak"] = 0
    if suspended_now():
        now_pos = {n: mk.pose(n)[0].copy() for n in FREE_BODIES}
        if STATE.get("hold", 0) == 0:
            STATE["hold_from"] = now_pos
        STATE["hold"] = STATE.get("hold", 0) + 1
        drift = max(float(np.linalg.norm(now_pos[n] - STATE["hold_from"][n])) for n in FREE_BODIES)
        if drift > P.HOLD_DRIFT:
            STATE["hold"], STATE["hold_from"] = 1, now_pos
    else:
        STATE["hold"], STATE["hold_from"] = 0, None


def level() -> float:
    if STATE.get("zero") is not None or zero_now() is not None:
        return 0.0
    if not STATE.get("both_caps_out", False):
        return 0.0
    if STATE.get("hold", 0) >= P.HOLD_STEPS and suspended_now():
        return 1.0
    _, coupled = best_engagement()
    if coupled:
        return 0.5
    return 0.25


def score(objects) -> float:
    """0 for good if the roller or either end-cap falls off the bench. Otherwise requires both end-caps extracted
    clear of the carton at least once (0.25); 0.5 if both end-caps are fitted onto opposite ends of the roller (>= 12
    mm inside each 21 mm socket); 1.0 if both end-caps rest upright on the carton floor, the roller sits >= 14 mm in
    both sockets within 5 degrees of level and >= 10 mm suspended above the carton floor (0 N roller-carton contact),
    and both hands are >= 30 mm clear for 1.0 s with <= 2 mm drift."""
    return level()


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out = {
        "zero": STATE.get("zero"),
        "both_caps_out": STATE.get("both_caps_out", False),
        "first_out_s": STATE.get("first_out_s"),
        "hold_steps": STATE.get("hold", 0),
    }
    if mk.MECH.poses:
        pair, coupled = best_engagement()
        out.update(
            {
                "engagement_mm_rad_deg": {c: [round(1000 * pair[c][0], 1), round(1000 * pair[c][1], 1), round(pair[c][2], 1)] for c in CAPS},
                "coupled_0.5": coupled,
                "caps_seated": {c: cap_seated_in_carton(c) for c in CAPS},
                "cap_tilts_deg": {c: round(cap_tilt_deg(c), 2) for c in CAPS},
                "roller_tilt_deg": round(roller_tilt_deg(), 2),
                "roller_floor_gap_mm": round(1000 * roller_floor_gap(), 2),
                "forces_N": {
                    "roller_carton": round(mk.contact_force("roller", "carton"), 2),
                    "roller_cradle": round(mk.contact_force("roller", "cradle"), 2),
                    "cap_l_roller": round(mk.contact_force("cap_l", "roller"), 2),
                    "cap_r_roller": round(mk.contact_force("cap_r", "roller"), 2),
                    "cap_l_carton": round(mk.contact_force("cap_l", "carton"), 2),
                    "cap_r_carton": round(mk.contact_force("cap_r", "carton"), 2),
                },
                "hands_clear_mm": round(1000 * hands_clear(), 1),
                "score": score(None),
            }
        )
    return out
