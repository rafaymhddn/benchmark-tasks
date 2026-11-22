"""A Dremel VersaFlame 2200 hobby butane burner stands upright on its removable 60 mm round stand beside its open
320 x 172 x 52 mm metal storage tin. Its continuous-flow side lock button is slid back at its rear stop, holding the
rear ignition button depressed 4.5 mm against its 16 N return spring while the red child-safety stopper is held out to
the left (gas valve latched open).

Stowing or enclosing the burner in the tin before releasing the lock button traps leaking butane in an enclosed box
(latched 0). Leaving the 60 mm stand on the 38 mm tank heel physically props the 52 mm tin's lid open (46 mm interior
clearance). One hand must steady the tank while the other pushes the lock button forward (>= 4.5 mm against 10 N) so
the ignition button springs out and the red child-safety stopper resets; then the hands pull the stand axially through
12 mm of 8 N socket friction, lay both parts flat inside the tin, and shut the hinged lid.
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
SUCCESS_HOLD_S = 2.5
INSTRUCTION = "Take the hobby burner off its round stand, lay both flat inside the metal storage tin, and close the lid."
RUN_LIMIT_STEPS = 1600  # 80 s at 20 Hz
TOP = TABLE_TOP[SCENE]
SIDES = ("left", "right")
TASK_BODIES = ("burner", "stand", "tin_base", "tin_lid")

scene.build(TOP)
OBJECTS = list(mk.MECH.bodies.values())
STATE: dict = {}


def begin(seed: int | None = None) -> dict:
    STATE.clear()
    STATE.update(
        seed=seed,
        zero=None,
        hold=0,
        lock_rest=0.0,
        heel_rest=0.0,
        unlatched=False,
        child_locked=False,
        off_at=None,
        detached_at=None,
        max_lock_mm=0.0,
        max_heel_mm=0.0,
        hands=None,
    )
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)
    if mk.MECH.environment is not None:
        mk.set_limits("trigger", lower=-P.TRIG_OVERTRAVEL, upper=0.0002)
        mk.set_limits("child_stop", lower=0.0, upper=0.0002)
        mk.set_spring("lock_slider", rest=0.0, most=P.LOCK_DETENT_N)
        mk.set_spring("heel_fit", rest=0.0, most=P.STAND_PULL_N)


# --- Mechanism coupling ---------------------------------------------------------------------------------------------------
def _step_mechanisms() -> None:
    in_sim = mk.MECH.environment is not None
    lock_t = mk.joint("lock_slider")
    STATE["max_lock_mm"] = max(STATE.get("max_lock_mm", 0.0), 1000.0 * lock_t)

    if not STATE.get("unlatched", False):
        diff = lock_t - STATE.get("lock_rest", 0.0)
        if abs(diff) > P.LOCK_GIVE:
            STATE["lock_rest"] = lock_t - math.copysign(P.LOCK_GIVE, diff)
            if in_sim:
                mk.set_spring("lock_slider", rest=STATE["lock_rest"], most=P.LOCK_DETENT_N)
        if lock_t >= P.LOCK_RELEASE_AT:
            STATE["unlatched"] = True
            if in_sim:
                mk.set_limits("trigger", lower=-P.TRIG_OVERTRAVEL, upper=P.TRIG_LAT)
                mk.set_spring("lock_slider", rest=P.LOCK_STROKE, most=P.LOCK_FREE_N)

    if STATE.get("unlatched", False):
        trig_t = mk.joint("trigger")
        if trig_t >= P.TRIG_OFF_TRAVEL and in_sim:
            mk.set_limits("child_stop", lower=0.0, upper=P.CHILD_STROKE)
            mk.set_spring("child_stop", rest=P.CHILD_REST, most=10.0)
        if not STATE.get("child_locked", False) and trig_t >= 0.0040 and mk.joint("child_stop") >= P.CHILD_OFF_TRAVEL:
            STATE["child_locked"] = True
            if in_sim:
                mk.set_limits("trigger", lower=0.0040, upper=P.TRIG_LAT)

    if not mk.is_cut("w_stand"):
        heel_t = mk.joint("heel_fit")
        STATE["max_heel_mm"] = max(STATE.get("max_heel_mm", 0.0), 1000.0 * heel_t)
        if heel_t >= P.SOCKET_DEPTH:
            mk.cut("w_stand")
            STATE["detached_at"] = mk.now() if in_sim else 0.0
            if in_sim:
                mk.set_spring("heel_fit", rest=0.0, most=2.0)
        else:
            diff = heel_t - STATE.get("heel_rest", 0.0)
            if abs(diff) > P.HEEL_GIVE:
                STATE["heel_rest"] = heel_t - math.copysign(P.HEEL_GIVE, diff)
                if in_sim:
                    mk.set_spring("heel_fit", rest=STATE["heel_rest"], most=P.STAND_PULL_N)


# --- Physical readings used by the rules ----------------------------------------------------------------------------------
def valve_closed() -> bool:
    """Gas valve shut off: lock button slid >= 4.5 mm forward and rear ignition button sprung >= 3.5 mm outward."""
    return mk.joint("lock_slider") >= P.LOCK_RELEASE_AT and mk.joint("trigger") >= P.TRIG_OFF_TRAVEL


def burner_off() -> bool:
    """Valve shut off AND auto-resetting red child-safety stopper snapped >= 5.0 mm back under the ignition button."""
    return valve_closed() and mk.joint("child_stop") >= P.CHILD_OFF_TRAVEL


def lid_deg() -> float:
    """Tin lid opening angle from closed flat (0 deg = shut on rim, 108 deg = resting open)."""
    return max(0.0, P.LID_OPEN_DEG - math.degrees(mk.joint("lid_hinge")))


def seam_gap_mm() -> float:
    """Front-lip seam gap across the 172 mm tin width (mm)."""
    return 1000.0 * P.TIN_W * math.sin(math.radians(min(90.0, lid_deg())))


def heel_gap() -> float:
    """Distance (m) between the burner heel origin and the stand socket origin."""
    return float(np.linalg.norm(mk.pose("burner")[0] - mk.pose("stand")[0]))


def detached() -> bool:
    """Stand pulled fully off the 12 mm heel spigot and physically clear of the burner heel."""
    return mk.is_cut("w_stand") and heel_gap() >= P.SEPARATE_GAP


def box_in_tin(name: str) -> tuple[np.ndarray, np.ndarray]:
    """(low, high) bounding box of ``name``'s solid shapes in ``tin_base``'s frame (m)."""
    p_tin, r_tin = mk.pose("tin_base")
    p_body, r_body = mk.pose(name)
    r_rel = r_tin.T @ r_body
    p_rel = r_tin.T @ (p_body - p_tin)
    lows, highs = [], []
    for s in mk.MECH.bodies[name].shapes:
        if not s.solid:
            continue
        rs = r_rel @ mk.quat_to_matrix(s.quat)
        half = s.half_extents()
        reach = half if s.kind == "sphere" else (np.abs(rs) @ half)
        centre = p_rel + r_rel @ np.asarray(s.pos, dtype=float)
        lows.append(centre - reach)
        highs.append(centre + reach)
    return np.min(lows, axis=0), np.max(highs, axis=0)


def burner_over_tin() -> tuple[bool, float]:
    """Whether ``burner`` horizontally overlaps ``tin_base``'s inner opening, and its lowest z in tin frame."""
    low, high = box_in_tin("burner")
    hx = P.TIN_L / 2.0 - P.TIN_WALL - 0.004
    hy = P.TIN_W / 2.0 - P.TIN_WALL - 0.004
    over = bool(high[0] > -hx and low[0] < hx and high[1] > -hy and low[1] < hy)
    return over, float(low[2])


def inside_tin_flat(name: str) -> bool:
    """``name`` lies wholly within ``tin_base``'s inner walls, on/above the floor, and <= 46 mm high."""
    low, high = box_in_tin(name)
    hx = P.TIN_L / 2.0 - P.TIN_WALL + 0.0015
    hy = P.TIN_W / 2.0 - P.TIN_WALL + 0.0015
    return bool(
        low[0] >= -hx
        and high[0] <= hx
        and low[1] >= -hy
        and high[1] <= hy
        and low[2] >= 0.001
        and high[2] <= P.FLAT_TOP_MAX
    )


def on_bench(name: str) -> bool:
    p = mk.pose(name)[0]
    return bool(-0.19 <= p[0] <= 0.91 and -0.65 <= p[1] <= 0.65 and p[2] >= TOP - P.BENCH_DROP_TOL)


def at_rest() -> bool:
    for name in TASK_BODIES:
        lin, ang = mk.velocity(name)
        if float(np.linalg.norm(lin)) > P.REST_LIN or float(np.linalg.norm(ang)) > P.REST_ANG:
            return False
    return True


def zero_now() -> str | None:
    for name in ("burner", "stand", "tin_base"):
        if not on_bench(name):
            return f"{name} fell off the bench"
    if not valve_closed():
        over, z_low = burner_over_tin()
        if over and z_low < P.TIN_BASE_H:
            return "burner entered the tin while its gas valve was still latched open"
        if over and lid_deg() < P.LID_UNSAFE_DEG:
            return "tin lid closed below 30 degrees over the latched-open burner"
    return None


# --- Hand clearance -------------------------------------------------------------------------------------------------------
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
        pts = []
        for link, local in _HULLS.items():
            key = (id(base), side, link)
            if key not in _LINKS:
                found = robot.find_bodies(link)[0]
                if not found:
                    raise RuntimeError(f"the {side} arm has no body {link!r}")
                _LINKS[key] = found[0]
            pose_w = robot.data.body_link_pose_w.torch[0, _LINKS[key]].detach().cpu().numpy().astype(float)
            pts.append(local @ mk.quat_to_matrix(pose_w[3:7]).T + pose_w[:3] - origin)
        hands[side] = np.vstack(pts)
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
    return min(_box_distance(points, name) for name in TASK_BODIES)


# --- Score ----------------------------------------------------------------------------------------------------------------
def full_now() -> bool:
    return (
        burner_off()
        and detached()
        and inside_tin_flat("burner")
        and inside_tin_flat("stand")
        and lid_deg() <= P.LID_SHUT_DEG
        and hands_clear() >= P.HANDS_CLEAR
        and at_rest()
    )


@mk.on_step
def step() -> None:
    if mk.MECH.environment is not None:
        read_hands()
    _step_mechanisms()
    if STATE.get("off_at") is None and burner_off():
        STATE["off_at"] = mk.now() if mk.MECH.environment is not None else 0.0
    if STATE.get("zero") is None:
        STATE["zero"] = zero_now()
    if STATE.get("zero") is None and full_now():
        STATE["hold"] = STATE.get("hold", 0) + 1
    else:
        STATE["hold"] = 0


def level() -> float:
    if STATE.get("zero") is not None or zero_now() is not None:
        return 0.0
    if STATE.get("hold", 0) >= P.HOLD_STEPS and full_now():
        return 1.0
    if burner_off() and detached() and (inside_tin_flat("burner") or inside_tin_flat("stand")):
        return 0.5
    if burner_off():
        return 0.25
    return 0.0


def score(objects) -> float:
    """0 for good if burner, stand, or tin_base falls off the bench or > 10 mm below the bench top, or if the
    latched-open burner dips below the tin rim or under a lid < 30 deg. Otherwise: 1.0 when shut off (lock >= 4.5 mm
    forward, trigger >= 3.5 mm out, red child-stop >= 5.0 mm reset), detached (>= 30 mm apart), both flat inside
    tin_base (<= 46 mm high), lid <= 2.0 deg (front seam <= 6.0 mm), hands >= 30 mm clear, and at rest for 1.0 s (20
    steps); 0.5 when shut off, detached, and at least one part flat inside tin_base; 0.25 when shut off; 0 otherwise."""
    return level()


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out = {
        "zero": STATE.get("zero"),
        "hold_steps": STATE.get("hold", 0),
        "off_at_s": STATE.get("off_at"),
        "detached_at_s": STATE.get("detached_at"),
        "max_lock_mm": round(STATE.get("max_lock_mm", 0.0), 2),
        "max_heel_mm": round(STATE.get("max_heel_mm", 0.0), 2),
    }
    if mk.MECH.poses:
        _, b_high = box_in_tin("burner")
        _, s_high = box_in_tin("stand")
        out.update({
            "burner_off": burner_off(),
            "lock_mm": round(1000.0 * mk.joint("lock_slider"), 2),
            "trigger_out_mm": round(1000.0 * mk.joint("trigger"), 2),
            "child_reset_mm": round(1000.0 * mk.joint("child_stop"), 2),
            "detached": detached(),
            "heel_gap_mm": round(1000.0 * heel_gap(), 1),
            "burner_in_tin_flat": inside_tin_flat("burner"),
            "stand_in_tin_flat": inside_tin_flat("stand"),
            "burner_top_in_tin_mm": round(1000.0 * float(b_high[2]), 1),
            "stand_top_in_tin_mm": round(1000.0 * float(s_high[2]), 1),
            "lid_deg": round(lid_deg(), 2),
            "seam_gap_mm": round(seam_gap_mm(), 2),
            "hands_clear_mm": round(1000.0 * hands_clear(), 1),
            "at_rest": at_rest(),
            "score": score(None),
        })
    return out
