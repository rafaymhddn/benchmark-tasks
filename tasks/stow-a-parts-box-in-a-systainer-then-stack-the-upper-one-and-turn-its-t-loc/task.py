"""Stow a red parts box in an open lower Festool Systainer, close its lid, slide the upper Systainer rearward under
the lower lid's rear cleats until its front feet drop flush, and turn the lower T-Loc latch to connect both cases.

Built on ``mechkit`` 1.0 (``scene.py``, every figure with its source in ``parts.py``). Zero welds, zero external
pushes, zero proximity holds: rear retention comes from rigid rear hook tongues under the lower lid's rear cleat
overhangs; front lid retention (at ``0 deg`` CLOSE and ``+90 deg`` CONNECT) and front inter-case retention (at
``+90 deg`` CONNECT) come from rigid cam lobes on ``tloc_low`` engaging keeper lips on ``lid_low`` and ``case_top``.
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
SUCCESS_HOLD_S = 2.5  # lets both arms finish returning to ready so the final frame shows the coupled stack clearly
INSTRUCTION = (
    "Place the red parts box flat inside the open lower Systainer, close its lid flush, slide the upper Systainer "
    "rearward under the lower lid's rear cleats until it seats flush, and turn the lower green T-Loc latch to its "
    "connect position so both cases lock together, then let go and move both hands clear."
)
RUN_LIMIT_STEPS = P.RUN_LIMIT_STEPS  # 1,600 steps (80 s at 20 Hz)
TOP = TABLE_TOP[SCENE]
SIDES = ("left", "right")
ALL_PARTS = ("case_low", "lid_low", "tloc_low", "case_top", "parts_box")

scene.build(TOP)
OBJECTS = list(mk.MECH.bodies.values())
STATE: dict = {}


def begin(seed: int | None = None) -> dict:
    STATE.clear()
    STATE.update(
        seed=seed,
        zero=None,
        hold=0,
        hold_low=None,
        hold_top=None,
        hold_tloc=None,
        hands=None,
        worst_slide_m=0.0,
        worst_yaw_deg=0.0,
        peak_tloc_nm=0.0,
    )
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)
    if mk.MECH.environment is not None:
        mk.set_spring("lid_hinge", rest=P.lid_stay_travel_rest(0.0))
        mk.set_spring("tloc", rest=P.tloc_travel_rest(0.0))


# --- kinematics in the lower Systainer's frame ------------------------------------------------------------------------
def low_pose() -> tuple[np.ndarray, np.ndarray]:
    return mk.pose("case_low")


def case_low_slide_m() -> float:
    p, _ = low_pose()
    return float(math.hypot(p[0] - P.CASE_LOW_XY[0], p[1] - P.CASE_LOW_XY[1]))


def case_low_yaw_deg() -> float:
    _, r = low_pose()
    return float(math.degrees(math.atan2(r[1, 0], r[0, 0])))


def lid_angle_rad() -> float:
    return P.LID_OPEN_RAD + mk.joint("lid_hinge")


def tloc_angle_rad() -> float:
    return P.TLOC_OPEN_RAD + mk.joint("tloc")


def lid_deg() -> float:
    return float(math.degrees(lid_angle_rad()))


def tloc_deg() -> float:
    return float(math.degrees(tloc_angle_rad()))


def box_in_low() -> tuple[np.ndarray, float]:
    """(parts_box origin in case_low frame, tilt deg of parts_box relative to case_low)."""
    p_low, r_low = low_pose()
    p_box, r_box = mk.pose("parts_box")
    rel = r_low.T @ (p_box - p_low)
    cos_t = max(-1.0, min(1.0, float(np.dot(r_low[:, 2], r_box[:, 2]))))
    return rel, float(math.degrees(math.acos(cos_t)))


def box_stowed() -> bool:
    """Red parts_box rests flat on case_low's floor wholly inside its walls."""
    rel, tilt = box_in_low()
    x_lim = P.INNER_D / 2.0 - P.BOX_D / 2.0 + 0.004
    y_lim = P.INNER_W / 2.0 - P.BOX_W / 2.0 + 0.004
    inside_xy = abs(float(rel[0])) <= x_lim and abs(float(rel[1])) <= y_lim
    if abs(float(rel[1])) < P.RECESS_HALF_Y + P.BOX_W / 2.0 - 0.002:
        inside_xy = inside_xy and float(rel[0]) >= P.RECESS_WALL_X + P.WALL_T + P.BOX_D / 2.0 - 0.004
    on_floor = abs(float(rel[2]) - P.FLOOR_TOP_Z) <= P.BOX_FLOOR_TOL
    return bool(inside_xy and on_floor and tilt <= P.BOX_TILT_MAX_DEG)


def lid_closed() -> bool:
    return -1.0 <= lid_deg() <= P.LID_SHUT_MAX_DEG


def stack_metrics() -> dict:
    """Relative placement, seam gap, and rear cleat engagement of case_top in case_low's frame."""
    p_low, r_low = low_pose()
    p_top, r_top = mk.pose("case_top")
    rel = r_low.T @ (p_top - p_low)
    r_rel = r_low.T @ r_top
    tilt = float(math.degrees(math.acos(max(-1.0, min(1.0, float(r_rel[2, 2]))))))
    yaw = float(math.degrees(math.atan2(r_rel[1, 0], r_rel[0, 0])))
    corner_gaps = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            c = rel + r_rel @ np.array([sx * P.CASE_D / 2.0, sy * P.CASE_W / 2.0, P.FOOT_H])
            corner_gaps.append(abs(float(c[2]) - P.CASE_H))
    overlaps = []
    under_cleats = True
    lat_clear = (P.CLEAT_W - P.HOOK_W) / 2.0 + 0.002
    for sy in (-1.0, 1.0):
        tip = rel + r_rel @ np.array([P.HOOK_X[1], sy * P.CLEAT_Y, P.HOOK_T])
        overlaps.append(float(tip[0]) - P.CLEAT_THROAT_X[0])
        in_y = abs(float(tip[1]) - sy * P.CLEAT_Y) <= lat_clear
        in_z = (P.LID_DECK_Z - 0.0015) <= float(tip[2]) <= (P.LID_DECK_Z + P.CLEAT_GAP + 0.0005)
        under_cleats = under_cleats and in_y and in_z
    min_overlap = min(overlaps)
    cleats_engaged = bool(under_cleats and min_overlap >= P.CLEAT_OVERLAP_MIN)
    return {
        "dx_m": float(rel[0]),
        "dy_m": float(rel[1]),
        "dz_m": float(rel[2] - P.LID_DECK_Z),
        "tilt_deg": tilt,
        "yaw_deg": yaw,
        "max_seam_m": max(corner_gaps),
        "min_cleat_overlap_m": min_overlap,
        "cleats_engaged": cleats_engaged,
    }


def upper_seated() -> bool:
    """Upper Systainer slid under lower lid's rear cleats and seated flush on the closed lower lid."""
    if not lid_closed():
        return False
    sm = stack_metrics()
    return bool(
        sm["cleats_engaged"]
        and abs(sm["dx_m"]) <= P.STACK_DX_MAX
        and abs(sm["dy_m"]) <= P.STACK_DY_MAX
        and abs(sm["yaw_deg"]) <= P.STACK_YAW_MAX_DEG
        and sm["tilt_deg"] <= P.STACK_TILT_MAX_DEG
        and sm["max_seam_m"] <= P.SEAM_GAP_MAX
    )


def tloc_connected() -> bool:
    return P.TLOC_CONNECT_MIN_DEG <= tloc_deg() <= P.TLOC_CONNECT_MAX_DEG


def zero_now() -> str | None:
    """Whole-trial zero rule (checked from trial video on the bench, end photo confirming)."""
    for name in ("case_low", "case_top", "parts_box"):
        if float(mk.pose(name)[0][2]) < TOP - P.FALL_BELOW_BENCH:
            return f"{name} fell off the bench"
    return None


# --- robot hands clearance --------------------------------------------------------------------------------------------
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
                    raise RuntimeError(f"{side} arm missing link {link!r}")
                _LINKS[key] = found[0]
            pose = robot.data.body_link_pose_w.torch[0, _LINKS[key]].detach().cpu().numpy().astype(float)
            pts.append(local @ mk.quat_to_matrix(pose[3:7]).T + pose[:3] - origin)
        hands[side] = np.vstack(pts)
    STATE["hands"] = hands


def _body_distance(points: np.ndarray, name: str) -> float:
    pos, rot = mk.pose(name)
    low, high = mk.MECH.bodies[name].bounds()
    local = (points - pos) @ rot
    gap = np.maximum(np.maximum(np.asarray(low) - local, local - np.asarray(high)), 0.0)
    return float(np.linalg.norm(gap, axis=1).min())


def hands_clear_m() -> float:
    hands = STATE.get("hands")
    if not hands:
        return 0.0
    pts = np.vstack([hands[s] for s in SIDES if s in hands])
    return min(_body_distance(pts, name) for name in ALL_PARTS)


# --- step update & scoring --------------------------------------------------------------------------------------------
def full_now() -> bool:
    return box_stowed() and lid_closed() and upper_seated() and tloc_connected() and hands_clear_m() >= P.HANDS_CLEAR_M


@mk.on_step
def step() -> None:
    if mk.MECH.environment is not None:
        mk.set_spring("lid_hinge", rest=P.lid_stay_travel_rest(mk.joint("lid_hinge")))
        mk.set_spring("tloc", rest=P.tloc_travel_rest(mk.joint("tloc")))
        read_hands()
    STATE["worst_slide_m"] = max(STATE.get("worst_slide_m", 0.0), case_low_slide_m())
    STATE["worst_yaw_deg"] = max(STATE.get("worst_yaw_deg", 0.0), abs(case_low_yaw_deg()))
    STATE["peak_tloc_nm"] = max(STATE.get("peak_tloc_nm", 0.0), abs(P.tloc_torque_nm(tloc_angle_rad())))
    if STATE.get("zero") is None:
        STATE["zero"] = zero_now()
    if STATE.get("zero") is None and full_now():
        p_low = low_pose()[0].copy()
        p_top = mk.pose("case_top")[0].copy()
        ang = tloc_deg()
        if STATE.get("hold", 0) == 0:
            STATE["hold_low"], STATE["hold_top"], STATE["hold_tloc"] = p_low, p_top, ang
        STATE["hold"] = STATE.get("hold", 0) + 1
        drift = max(float(np.linalg.norm(p_low - STATE["hold_low"])), float(np.linalg.norm(p_top - STATE["hold_top"])))
        if drift > P.HOLD_DRIFT_M or abs(ang - STATE["hold_tloc"]) > P.HOLD_TLOC_DEG:
            STATE["hold"], STATE["hold_low"], STATE["hold_top"], STATE["hold_tloc"] = 1, p_low, p_top, ang
    else:
        STATE["hold"], STATE["hold_low"], STATE["hold_top"], STATE["hold_tloc"] = 0, None, None, None


def level() -> float:
    if STATE.get("zero") is not None or zero_now() is not None:
        return 0.0
    if not (box_stowed() and lid_closed()):
        return 0.0
    if not upper_seated():
        return 0.25
    if STATE.get("hold", 0) >= P.HOLD_STEPS and full_now():
        return 1.0
    return 0.5


def score(objects) -> float:
    """0 for good if either Systainer or the red parts box falls off the bench (more than 30 mm below bench level).
    Otherwise read at the end: 0.25 when the red parts box rests flat inside the lower Systainer (within 5 mm of its
    floor, tilted at most 10 degrees) and the lower lid is closed flush (at most 2.5 degrees); 0.5 when, in addition,
    the upper Systainer has both rear base hooks slid at least 5.0 mm under the lower lid's rear cleats and sits flush
    on the lower case (corner seam gap at most 2.5 mm all round, front-to-back offset at most 4.0 mm, side-to-side
    offset at most 6.0 mm, yaw at most 6.0 degrees, tilt at most 3.0 degrees); 1.0 when, in addition, the lower green
    T-Loc latch is in its connect detent (+80 to +95 degrees), both robot hands are at least 30 mm clear of every part,
    and both cases stay within 2.0 mm and the T-Loc within 2.0 degrees for 1.0 s (20 steps)."""
    return level()


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out = {
        "zero": STATE.get("zero"),
        "hold_steps": STATE.get("hold", 0),
        "worst_slide_mm": round(1000.0 * STATE.get("worst_slide_m", 0.0), 1),
        "worst_yaw_deg": round(STATE.get("worst_yaw_deg", 0.0), 2),
        "peak_tloc_nm": round(STATE.get("peak_tloc_nm", 0.0), 3),
    }
    if mk.MECH.poses:
        sm = stack_metrics()
        rel_box, box_tilt = box_in_low()
        out.update(
            {
                "lid_deg": round(lid_deg(), 2),
                "tloc_deg": round(tloc_deg(), 2),
                "tloc_spring_nm": round(P.tloc_torque_nm(tloc_angle_rad()), 3),
                "box_stowed": box_stowed(),
                "box_rel_mm": [round(1000.0 * float(v), 1) for v in rel_box],
                "box_tilt_deg": round(box_tilt, 2),
                "lid_closed": lid_closed(),
                "upper_seated": upper_seated(),
                "tloc_connected": tloc_connected(),
                "cleats_engaged": sm["cleats_engaged"],
                "min_cleat_overlap_mm": round(1000.0 * sm["min_cleat_overlap_m"], 1),
                "max_seam_mm": round(1000.0 * sm["max_seam_m"], 2),
                "stack_dx_dy_mm": [round(1000.0 * sm["dx_m"], 1), round(1000.0 * sm["dy_m"], 1)],
                "hands_clear_mm": round(1000.0 * hands_clear_m(), 1),
                "score": score(None),
            }
        )
    return out
