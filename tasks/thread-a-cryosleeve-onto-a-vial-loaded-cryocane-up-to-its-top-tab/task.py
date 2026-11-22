"""Thread a 273 mm Nalgene 5016-0001 CryoSleeve coaxially onto a vial-loaded 290 mm 5-place aluminum Nalgene 5015-0001
CryoCane from its straight bottom foot up to its 26 mm bent top ID tab without spilling either loose 1.8 mL Nunc 377267
cryovial, then lay the enclosed cane flat inside the stainless transfer tray and let go.

Zero write_pose, zero set_joint, zero computed push, and no joint between sleeve and cane: the sleeve is a free 16-sided
PVC tube (13.5 mm bore, 14.5 mm outside) in true contact with the cane and vials (0.15 mm a side when centred). The robot
keeps it straight through the whole slide; a cocked push wedges its rims and PhysX friction (PVC 0.4/0.35) stops it.
The score reads where the sleeve is: on the cane when its axis is within 2 mm and 2 deg of the stack's middle line.
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
INSTRUCTION = (
    "Slide the ivory cryo-sleeve from the prep mat onto the aluminum cryo-cane up to its bent top tab "
    "without spilling either vial, lay the sleeved cane flat inside the stainless transfer tray, then let go and move both hands clear."
)
RUN_LIMIT_STEPS = 1800  # 90 s at 20 Hz
TOP = TABLE_TOP[SCENE]
SIDES = ("left", "right")
VIALS = scene.VIALS
TASK_BODIES = ("cane", "sleeve", *VIALS, "tray")

scene.build(TOP)
OBJECTS = list(mk.MECH.bodies.values())
STATE: dict = {}


def begin(seed: int | None = None) -> dict:
    STATE.clear()
    STATE.update(
        seed=seed,
        zero=None,
        hold=0,
        hold_from=None,
        hands=None,
        max_slide_m=0.0,
        worst_roll_deg=0.0,
        worst_pitch_deg=0.0,
        on_cane_at_s=None,
    )
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)


# --- physical and geometric readings --------------------------------------------------------------------------------------
def cane_pose() -> tuple[np.ndarray, np.ndarray]:
    return mk.pose("cane")


def cane_axis_tilt_deg() -> float:
    u = cane_pose()[1][:, 1]
    return float(math.degrees(math.asin(max(-1.0, min(1.0, abs(float(u[2])))))))


def cane_roll_deg() -> float:
    r = cane_pose()[1]
    return float(math.degrees(math.atan2(float(r[2, 0]), float(r[2, 2]))))


def cane_pitch_deg() -> float:
    u = cane_pose()[1][:, 1]
    return float(math.degrees(math.asin(max(-1.0, min(1.0, float(u[2]))))))


def sleeve_span_on_cane() -> tuple[bool, float, float, float, float]:
    """(on the cane, y_lo_m, y_hi_m, radial_m, angle_deg): the sleeve's span along the cane from its foot, and its axis's
    offset from the stack's middle line (at the middle of their overlap) and angle to it. On the cane: within 2 mm and
    2 deg with the span reaching from behind the foot onto it (true contact allows only 0.15 mm a side once threaded)."""
    p_c, r_c = cane_pose()
    u_c = r_c[:, 1]
    a_c = p_c + P.BORE_Z * r_c[:, 2]
    p_s, r_s = mk.pose("sleeve")
    u_s = r_s[:, 1]
    cos_a = float(np.dot(u_s, u_c))
    ang_deg = math.degrees(math.acos(min(1.0, abs(cos_a))))
    m1 = float(np.dot((p_s + 0.5 * P.SLEEVE_L * u_s) - a_c, u_c))
    m2 = float(np.dot((p_s - 0.5 * P.SLEEVE_L * u_s) - a_c, u_c))
    y_lo, y_hi = min(m1, m2), max(m1, m2)
    s_mid = 0.5 * (max(y_lo, 0.0) + min(y_hi, P.CANE_L))
    if abs(cos_a) < 0.5:
        return False, y_lo, y_hi, math.inf, ang_deg
    q = p_s + ((s_mid - float(np.dot(p_s - a_c, u_c))) / cos_a) * u_s
    radial = float(np.linalg.norm(q - a_c - s_mid * u_c))
    on = radial <= P.COAX_RADIAL_MAX and ang_deg <= P.COAX_ANGLE_DEG and y_lo <= 0.0 < y_hi
    return on, y_lo, y_hi, radial, ang_deg


def coaxial_enclosed() -> bool:
    return sleeve_span_on_cane()[0]


def _cane_samples() -> np.ndarray:
    """Points on every solid collider of the cane's trough (corners and along its long edges every 10 mm), cane frame."""
    pts = []
    for s in mk.MECH.bodies["cane"].shapes:
        h = np.asarray(s.half_extents())
        if not s.solid or s.pos[1] - h[1] >= P.TAB_SHOULDER_Y:
            continue
        rot = mk.quat_to_matrix(s.quat) if s.quat is not None else np.eye(3)
        n = max(2, int(2.0 * h[1] / 0.010) + 1)
        for sx in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                for f in np.linspace(-1.0, 1.0, n):
                    pts.append(np.asarray(s.pos) + rot @ (np.array([sx, f, sz]) * h))
    return np.array(pts)


CANE_SAMPLES = _cane_samples()


def wall_breach_mm() -> float:
    """How far any cane point inside the sleeve (2 mm in from either rim) lies beyond its 16 inside flats, in mm: a few
    tenths while contact holds, more than the 0.5 mm wall if a collider went through it. A reading, not the score."""
    p_c, r_c = cane_pose()
    p_s, r_s = mk.pose("sleeve")
    loc = (p_c + CANE_SAMPLES @ r_c.T - p_s) @ r_s
    inside = np.abs(loc[:, 1]) <= 0.5 * P.SLEEVE_L - 0.002
    if not inside.any():
        return 0.0
    x, z = loc[inside, 0], loc[inside, 2]
    half = math.pi / P.SLEEVE_STAVES
    off = np.mod(np.arctan2(x, z) + half, 2.0 * half) - half  # angle from the nearest flat's middle
    return float(1000.0 * max(0.0, float((np.hypot(x, z) - P.SLEEVE_R_IN / np.cos(off)).max())))


def sleeve_slide_m() -> float:
    """Axial station (m from the straight foot) of the sleeve's leading mouth while it is on the cane."""
    ok, _, y_hi, _, _ = sleeve_span_on_cane()
    return max(0.0, y_hi) if ok else 0.0


def gap_to_tab_m() -> float:
    return max(0.0, P.TAB_SHOULDER_Y - sleeve_slide_m()) if coaxial_enclosed() else math.inf


def foot_enclosed() -> bool:
    ok, y_lo, _, _, _ = sleeve_span_on_cane()
    return ok and (y_lo <= 0.0)


def sleeve_seated() -> bool:
    return coaxial_enclosed() and gap_to_tab_m() <= P.SEATED_TAB_GAP_MAX and foot_enclosed()


def vial_offsets(name: str) -> tuple[float, float]:
    bay_idx = P.VIAL_BAYS[VIALS.index(name)]
    p_c, r_c = cane_pose()
    loc = r_c.T @ (mk.pose(name)[0] - p_c)
    return float(math.hypot(float(loc[0]), float(loc[2]))), abs(float(loc[1]) - P.bay_centre_y(bay_idx))


def vial_spilled(name: str) -> bool:
    radial, axial = vial_offsets(name)
    if radial > P.SPILL_RADIAL_MAX or axial > P.SPILL_AXIAL_MAX:
        return True
    if mk.MECH.contact_points.get(name):
        return any(bool(mk.touching(name, other)) for other in ("prep_mat", "tray", "scene"))
    return False


def tray_shift_m() -> float:
    p_t, _ = mk.pose("tray")
    return float(np.linalg.norm(p_t[:2] - np.array(scene.tray_start_pos(TOP)[:2], dtype=float)))


def unsleeved_in_tray() -> bool:
    if sleeve_seated():
        return False
    p_t, r_t = mk.pose("tray")
    hx, hy = 0.5 * P.TRAY_X - P.TRAY_WALL, 0.5 * P.TRAY_Y - P.TRAY_WALL
    p_c, r_c = cane_pose()
    pts = [
        p_c + r_c @ np.array([0.0, 0.0, P.BASE_Z]),
        p_c + r_c @ np.array([0.0, 0.5 * P.CANE_L, 0.0]),
        p_c + r_c @ np.array([0.0, P.CANE_L, P.TAB_TOP_Z]),
        *(mk.pose(v)[0] for v in VIALS),
    ]
    for pt in pts:
        loc = r_t.T @ (pt - p_t)
        if abs(float(loc[0])) < hx and abs(float(loc[1])) < hy and 0.0 <= float(loc[2]) <= P.TRAY_H:
            return True
    return False


def in_tray_flat() -> bool:
    if cane_axis_tilt_deg() > P.TRAY_TILT_MAX_DEG:
        return False
    p_t, r_t = mk.pose("tray")
    hx, hy = 0.5 * P.TRAY_X - P.TRAY_WALL, 0.5 * P.TRAY_Y - P.TRAY_WALL
    p_c, r_c = cane_pose()
    p_s, r_s = mk.pose("sleeve")
    pts = [
        *(p_c + r_c @ np.array([sx * 0.5 * P.TAB_W, P.CANE_L, zc]) for sx in (-1.0, 1.0) for zc in (P.BASE_Z, P.TAB_TOP_Z)),
        p_c + r_c @ np.array([0.0, 0.0, P.BASE_Z]),
        *(
            p_s + r_s @ np.array([sx * P.SLEEVE_R_OUT, sy * 0.5 * P.SLEEVE_L, sz * P.SLEEVE_R_OUT])
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
            for sz in (-1.0, 1.0)
        ),
    ]
    for pt in pts:
        loc = r_t.T @ (pt - p_t)
        if not (abs(float(loc[0])) <= hx and abs(float(loc[1])) <= hy and P.TRAY_WALL - 0.003 <= float(loc[2]) <= P.TRAY_H + 0.002):
            return False
    return True


def hands_clear() -> float:
    hands = STATE.get("hands")
    if not hands:
        return 0.0
    pts = np.vstack([hands[s] for s in SIDES if s in hands])
    return min(scene.box_distance(pts, name) for name in TASK_BODIES)


# --- whole-trial zero rules and end score ---------------------------------------------------------------------------------
def zero_now() -> str | None:
    for v in VIALS:
        if vial_spilled(v):
            return f"{v} spilled from its bay on the cane"
    if unsleeved_in_tray():
        return "unsleeved cane or vial entered the transfer tray"
    for name in TASK_BODIES:
        if float(mk.pose(name)[0][2]) < TOP - P.DROP_BELOW_BENCH:
            return f"{name} dropped below the bench"
    return None


def full_now() -> bool:
    return (
        sleeve_seated()
        and all(not vial_spilled(v) for v in VIALS)
        and in_tray_flat()
        and hands_clear() >= P.HANDS_CLEAR
    )


@mk.on_step
def step() -> None:
    if mk.MECH.environment is not None:
        STATE["hands"] = scene.read_hands(SIDES)
    on_cane = coaxial_enclosed()
    if on_cane:
        STATE["max_slide_m"] = max(STATE.get("max_slide_m", 0.0), sleeve_slide_m())
        if STATE.get("on_cane_at_s") is None:
            STATE["on_cane_at_s"] = mk.now()
    STATE["worst_roll_deg"] = max(STATE.get("worst_roll_deg", 0.0), abs(cane_roll_deg()))
    STATE["worst_pitch_deg"] = max(STATE.get("worst_pitch_deg", 0.0), abs(cane_pitch_deg()))
    if STATE.get("zero") is None:
        STATE["zero"] = zero_now()
    if STATE.get("zero") is None and full_now():
        now_pts = (cane_pose()[0].copy(), mk.pose("sleeve")[0].copy())
        if STATE.get("hold", 0) == 0 or STATE.get("hold_from") is None:
            STATE["hold_from"] = now_pts
        STATE["hold"] = STATE.get("hold", 0) + 1
        drift = max(float(np.linalg.norm(now_pts[0] - STATE["hold_from"][0])), float(np.linalg.norm(now_pts[1] - STATE["hold_from"][1])))
        if drift > P.HOLD_DRIFT:
            STATE["hold"], STATE["hold_from"] = 1, now_pts
    else:
        STATE["hold"], STATE["hold_from"] = 0, None


def level() -> float:
    if STATE.get("zero") is not None or zero_now() is not None:
        return 0.0
    if STATE.get("hold", 0) >= P.HOLD_STEPS and full_now():
        return 1.0
    if sleeve_seated():
        return 0.5
    if coaxial_enclosed() and sleeve_slide_m() >= P.PARTIAL_SLIDE_MIN:
        return 0.25
    return 0.0


def score(objects) -> float:
    return level()


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out = {
        "zero": STATE.get("zero"),
        "hold steps": STATE.get("hold", 0),
        "on the cane from (s)": STATE.get("on_cane_at_s"),
        "max slide (mm)": round(1000.0 * STATE.get("max_slide_m", 0.0), 1),
        "worst cane roll / pitch (deg)": [
            round(STATE.get("worst_roll_deg", 0.0), 1),
            round(STATE.get("worst_pitch_deg", 0.0), 1),
        ],
    }
    if mk.MECH.poses:
        on, _, _, radial, ang = sleeve_span_on_cane()
        out.update(
            {
                "sleeve on cane": on,
                "sleeve radial (mm) / angle (deg)": [round(1000.0 * radial, 2), round(ang, 2)],
                "sleeve slide (mm)": round(1000.0 * sleeve_slide_m(), 1),
                "gap to top tab (mm)": None if not on else round(1000.0 * gap_to_tab_m(), 1),
                "foot enclosed": foot_enclosed(),
                "vial offsets [radial, axial] (mm)": {v: [round(1000.0 * x, 2) for x in vial_offsets(v)] for v in VIALS},
                "in tray flat": in_tray_flat(),
                "cane tilt (deg)": round(cane_axis_tilt_deg(), 2),
                "tray shift (mm)": round(1000.0 * tray_shift_m(), 2),
                "hands clear (mm)": round(1000.0 * hands_clear(), 1),
                "score": score(None),
            }
        )
    return out
