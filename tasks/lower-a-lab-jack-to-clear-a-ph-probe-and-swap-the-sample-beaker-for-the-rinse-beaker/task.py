"""Lower a BOCHEM 11120 lab scissor jack to clear a fixed Hanna HI1230B pH electrode, swap the sample beaker onto the
blue mat for the green-banded rinse beaker, and raise the jack until the probe tip sits inside the 20 to 40 mm
immersion band without touching the beaker floor.

Mechanism (scene.py, figures in parts.py): prismatic deck ``deck_z`` + half-height carrier ``carrier_z`` coupled
two-way to the coaxial -y handwheel ``wheel`` at 18.0 mm/rev (capped at BOCHEM's 7 kg = 68.7 N dynamic load), feeding
axial load back as ``-F_deck * LEAD_M_PER_RAD`` torque on ``knob`` plus 0.10 N m thread drag (equal and opposite on
``wheel_carrier``). Witness springs on ``probe`` (8.5 N m/rad lateral, 2500 N/m axial) detect wall strikes (> 4.0 mm)
and beaker-floor bottoming (>= 1.2 mm / >= 3.0 N).
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
INSTRUCTION = (
    "Lower the lab scissor jack until the sample beaker clears the fixed pH probe, move the sample beaker onto the "
    "blue mat, set the rinse beaker on the jack, and raise the jack until the probe tip sits inside the green "
    "immersion band without touching the beaker floor."
)
RUN_LIMIT_STEPS = 2400  # 120 s at 20 Hz
TOP = TABLE_TOP[SCENE]
SIDES = ("left", "right")
CLEAR_BODIES = ("jack_base", "jack_deck", "wheel_carrier", "knob", "beaker_sample", "beaker_rinse", "probe")

scene.build(TOP)
OBJECTS = list(mk.MECH.bodies.values())
STATE: dict = {}


def begin(seed: int | None = None) -> dict:
    STATE.clear()
    STATE.update(
        seed=seed, zero=None, lowered_clear=False, min_deck_z_mm=round(1000.0 * P.Z_DECK_START, 2),
        wheel_hold=0.0, last_coupling={}, max_lateral_mm=0.0, max_axial_mm=0.0, max_probe_force_n=0.0,
        max_jack_slide_mm=0.0, worst_beaker_tilt_deg=0.0, hold=0, hold_tier=0.0, hold_anchor=None,
        hands=None, events={},
    )
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)
    if mk.MECH.environment is not None:
        mk.set_spring("deck_z", rest=0.0, most=P.DECK_MAX_N)
        mk.set_spring("carrier_z", rest=0.0, most=P.DECK_MAX_N)


def deck_height_m() -> float:
    return float(mk.pose("jack_deck")[0][2] - TOP)


def tilt_deg(name: str) -> float:
    return float(math.degrees(math.acos(max(-1.0, min(1.0, float(mk.pose(name)[1][2, 2]))))))


def probe_tip() -> np.ndarray:
    return mk.pose("probe")[0]


def probe_lateral_m() -> float:
    tip = probe_tip()
    return float(math.hypot(tip[0] - P.JACK_XY[0], tip[1] - P.JACK_XY[1]))


def probe_axial_lift_m() -> float:
    return float(max(0.0, mk.joint("probe_axial"), probe_tip()[2] - (TOP + P.Z_PROBE_TIP)))


def probe_contact_n() -> float:
    return float(max(mk.contact_force("probe", "beaker_sample"), mk.contact_force("probe", "beaker_rinse")))


def beaker_floor_z(name: str) -> float:
    p, r = mk.pose(name)
    return float((p + r @ np.array([0.0, 0.0, P.BEAKER_FLOOR]))[2])


def beaker_rim_z(name: str) -> float:
    p, r = mk.pose(name)
    return float((p + r @ np.array([0.0, 0.0, P.BEAKER_H]))[2])


def floor_bottomed() -> bool:
    if probe_axial_lift_m() >= P.AXIAL_ZERO_M:
        return True
    tip_z = float(probe_tip()[2])
    return any(
        mk.contact_force("probe", b) >= P.AXIAL_ZERO_N and abs(beaker_floor_z(b) - tip_z) <= 0.005
        for b in ("beaker_sample", "beaker_rinse")
    )


def beaker_on_bare_bench(name: str) -> bool:
    if mk.MECH.environment is not None and mk.touching(name, "scene"):
        return True
    return float(mk.pose(name)[0][2] - TOP) < 0.0015


def jack_slide_m() -> float:
    p = mk.pose("jack_base")[0]
    return float(math.hypot(p[0] - P.JACK_XY[0], p[1] - P.JACK_XY[1]))


def sample_on_mat() -> bool:
    p = mk.pose("beaker_sample")[0]
    in_xy = abs(float(p[0]) - P.MAT_XY[0]) <= P.MAT_HALF_INSIDE and abs(float(p[1]) - P.MAT_XY[1]) <= P.MAT_HALF_INSIDE
    on_z = abs(float(p[2]) - (TOP + P.MAT_SIZE[2])) <= 0.004
    ok_f = mk.contact_force("beaker_sample", "blue_mat") >= 0.25 if mk.MECH.environment is not None else True
    return in_xy and on_z and ok_f and tilt_deg("beaker_sample") <= P.UPRIGHT_DEG


def rinse_radial_offset_m() -> float:
    p = mk.pose("beaker_rinse")[0]
    return float(math.hypot(p[0] - P.JACK_XY[0], p[1] - P.JACK_XY[1]))


def rinse_on_deck_coaxial() -> bool:
    on_z = abs(float(mk.pose("beaker_rinse")[0][2] - mk.pose("jack_deck")[0][2])) <= 0.004
    ok_f = mk.contact_force("beaker_rinse", "jack_deck") >= 0.25 if mk.MECH.environment is not None else True
    return on_z and ok_f and tilt_deg("beaker_rinse") <= P.UPRIGHT_DEG and rinse_radial_offset_m() <= P.COAXIAL_M


def immersion_depth_m() -> float:
    return float(beaker_rim_z("beaker_rinse") - probe_tip()[2])


def in_immersion_band() -> bool:
    return P.IMM_MIN <= immersion_depth_m() <= P.IMM_MAX


def coupled_step() -> dict[str, float]:
    """Two-way screw-pantograph coupling between ``wheel`` and ``deck_z``."""
    theta = mk.joint("wheel")
    dz_lo, dz_hi = P.JACK_Z_MIN - P.Z_DECK_START, P.JACK_Z_SIM_MAX - P.Z_DECK_START
    target_dz = max(dz_lo, min(dz_hi, P.LEAD_M_PER_RAD * theta))
    actual_dz = mk.joint("deck_z")
    f_deck = max(-P.DECK_MAX_N, min(P.DECK_MAX_N, P.DECK_SPRING * (target_dz - actual_dz)))
    hold = max(theta - P.WHEEL_GIVE_RAD, min(theta + P.WHEEL_GIVE_RAD, float(STATE.get("wheel_hold", 0.0))))
    STATE["wheel_hold"] = hold
    tau_fric = P.WHEEL_K_HOLD * (hold - theta)
    tau_load = -f_deck * P.LEAD_M_PER_RAD
    tau_wheel = tau_fric + tau_load
    out = {
        "wheel_rad": theta, "target_dz": target_dz, "f_deck_n": f_deck,
        "tau_fric_nm": tau_fric, "tau_load_nm": tau_load, "tau_wheel_nm": tau_wheel,
    }
    STATE["last_coupling"] = out
    if mk.MECH.environment is not None:
        mk.set_spring("deck_z", rest=target_dz, most=P.DECK_MAX_N)
        mk.set_spring("carrier_z", rest=0.5 * actual_dz, most=P.DECK_MAX_N)
        axis_w = mk.pose("wheel_carrier")[1] @ np.array([0.0, 1.0, 0.0])
        t_vec = tuple(float(v) for v in (tau_wheel * axis_w))
        mk.push("knob", torque=t_vec)
        mk.push("wheel_carrier", torque=tuple(-v for v in t_vec))
    return out


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
                _LINKS[key] = robot.find_bodies(link)[0][0]
            pw = robot.data.body_link_pose_w.torch[0, _LINKS[key]].detach().cpu().numpy().astype(float)
            pts.append(local @ mk.quat_to_matrix(pw[3:7]).T + pw[:3] - origin)
        hands[side] = np.vstack(pts)
    STATE["hands"] = hands


def _box_distance(points: np.ndarray, name: str) -> float:
    position, rotation = mk.pose(name)
    low, high = mk.MECH.bodies[name].bounds()
    local = (points - position) @ rotation
    gap = np.maximum(np.maximum(np.asarray(low) - local, local - np.asarray(high)), 0.0)
    return float(np.linalg.norm(gap, axis=1).min())


def hands_clear_m() -> float:
    hands = STATE.get("hands")
    if not hands:
        return 0.0
    pts = np.vstack([hands[s] for s in SIDES if s in hands])
    return min(_box_distance(pts, name) for name in CLEAR_BODIES)


def zero_now() -> str | None:
    if probe_lateral_m() > P.DEFLECT_ZERO_M:
        return "the pH probe deflected laterally more than 4 mm"
    if floor_bottomed():
        return "a beaker floor touched and bottomed against the pH probe tip"
    for b in ("beaker_sample", "beaker_rinse"):
        if tilt_deg(b) > P.TILT_ZERO_DEG:
            return f"{b} tilted more than 25 degrees"
        if beaker_on_bare_bench(b):
            return f"{b} touched the bench outside its pad"
    if jack_slide_m() > P.SLIDE_ZERO_M:
        return "the lab jack slid more than 25 mm"
    return None


def raw_tier() -> float:
    if not STATE.get("lowered_clear", False) or not sample_on_mat():
        return 0.0
    if not rinse_on_deck_coaxial():
        return 0.25
    return 1.0 if in_immersion_band() else 0.5


def _anchor_positions() -> np.ndarray:
    return np.concatenate([mk.pose(n)[0] for n in ("jack_deck", "beaker_sample", "beaker_rinse")])


@mk.on_step
def step() -> None:
    if mk.MECH.environment is not None:
        read_hands()
    coupled_step()
    dz_m = deck_height_m()
    STATE["min_deck_z_mm"] = min(float(STATE.get("min_deck_z_mm", 999.0)), round(1000.0 * dz_m, 2))
    if dz_m < P.CLEAR_DECK_Z:
        STATE["lowered_clear"] = True
        STATE.setdefault("events", {}).setdefault("deck_cleared_probe_s", round(mk.now(), 2))
    STATE["max_lateral_mm"] = max(float(STATE.get("max_lateral_mm", 0.0)), round(1000.0 * probe_lateral_m(), 2))
    STATE["max_axial_mm"] = max(float(STATE.get("max_axial_mm", 0.0)), round(1000.0 * probe_axial_lift_m(), 2))
    STATE["max_probe_force_n"] = max(float(STATE.get("max_probe_force_n", 0.0)), round(probe_contact_n(), 2))
    STATE["max_jack_slide_mm"] = max(float(STATE.get("max_jack_slide_mm", 0.0)), round(1000.0 * jack_slide_m(), 2))
    STATE["worst_beaker_tilt_deg"] = max(
        float(STATE.get("worst_beaker_tilt_deg", 0.0)),
        round(max(tilt_deg("beaker_sample"), tilt_deg("beaker_rinse")), 2),
    )
    if STATE.get("zero") is None:
        STATE["zero"] = zero_now()

    tier = raw_tier()
    if tier > 0.0 and hands_clear_m() >= P.HANDS_CLEAR_M:
        now_pos = _anchor_positions()
        if STATE.get("hold", 0) == 0 or STATE.get("hold_tier") != tier:
            STATE["hold"], STATE["hold_tier"], STATE["hold_anchor"] = 1, tier, now_pos
        else:
            drift = float(np.max(np.linalg.norm((now_pos - STATE["hold_anchor"]).reshape(3, 3), axis=1)))
            STATE["hold"], STATE["hold_anchor"] = (1, now_pos) if drift > P.HOLD_DRIFT_M else (int(STATE["hold"]) + 1, STATE["hold_anchor"])
    else:
        STATE["hold"], STATE["hold_tier"], STATE["hold_anchor"] = 0, 0.0, None

    if STATE.get("zero") is None and STATE.get("hold", 0) >= P.HOLD_STEPS:
        STATE.setdefault("events", {}).setdefault(f"first_{STATE['hold_tier']}_s", round(mk.now(), 2))


def score(_objects) -> float:
    """0 for good if the probe deflects laterally > 4 mm or a beaker floor touches its tip, if either beaker tilts > 25 deg
    or touches the bare bench outside its pad, or if the jack slides > 25 mm. End tiers (hands >= 30 mm clear and drift
    <= 2 mm for 1.0 s): 0.25 sample cleared and upright on blue_mat; 0.5 rinse also coaxial on jack_deck (<= 12 mm
    offset) outside 20-40 mm band; 1.0 probe tip 20 to 40 mm below rinse rim."""
    if STATE.get("zero") is not None or zero_now() is not None:
        return 0.0
    tier = raw_tier()
    if tier > 0.0 and hands_clear_m() >= P.HANDS_CLEAR_M and int(STATE.get("hold", 0)) >= P.HOLD_STEPS and STATE.get("hold_tier") == tier:
        return float(tier)
    return 0.0


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out = {
        k: STATE.get(k)
        for k in (
            "zero", "lowered_clear", "min_deck_z_mm", "max_lateral_mm", "max_axial_mm",
            "max_probe_force_n", "max_jack_slide_mm", "worst_beaker_tilt_deg", "hold", "events",
        )
    }
    if mk.MECH.poses:
        out.update({
            "deck_height_mm": round(1000.0 * deck_height_m(), 2),
            "wheel_turns": round(mk.joint("wheel") / (2.0 * math.pi), 3),
            "immersion_mm": round(1000.0 * immersion_depth_m(), 2),
            "rinse_offset_mm": round(1000.0 * rinse_radial_offset_m(), 2),
            "sample_on_mat": sample_on_mat(),
            "rinse_coaxial": rinse_on_deck_coaxial(),
            "hands_clear_mm": round(1000.0 * hands_clear_m(), 1),
            "coupling": {k: round(v, 5) for k, v in STATE.get("last_coupling", {}).items()},
            "score": score(None),
        })
    return out
