"""Thread a 273 mm Nalgene 5016-0001 CryoSleeve coaxially onto a vial-loaded 290 mm 5-place aluminum Nalgene 5015-0001
CryoCane from its straight bottom foot up to its 26 mm bent top ID tab without spilling either loose 1.8 mL Nunc 377267
cryovial, then lay the enclosed cane flat inside the stainless transfer tray and let go.

Zero write_pose, zero set_joint, zero computed push: w_sleeve engages in-place right where sleeve stands once telescoped
10..60 mm coaxially over the straight bottom foot (<= 2.0 deg), slv_s provides the 5.0 N rest-following wipe (35.0 N if
cocked), and scoring also accepts a direct geometric coaxial enclosure (radial <= 6 mm, angle <= 5 deg, mouth 30..273 mm).
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
    "without spilling either vial, lay the sleeved cane flat inside the stainless transfer tray, and let go."
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
        slv_rest=0.0,
        entry_s_offset=P.CARRIAGE_BUILT_S,
        slide_force_n=0.0,
        max_slide_f_n=0.0,
        bend_nm=0.0,
        max_bend_nm=0.0,
        bound_steps=0,
        max_slide_m=0.0,
        worst_roll_deg=0.0,
        worst_pitch_deg=0.0,
        engaged_at_s=None,
    )
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)
    mk.show("c_slv", 0, False)
    if mk.MECH.environment is not None:
        mk.set_spring("slv_s", rest=0.0, most=P.SLIDE_N)


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


def sleeve_mated() -> bool:
    return not mk.is_cut("w_sleeve")


def sleeve_span_on_cane() -> tuple[bool, float, float, float, float]:
    """(enclosed, y_lo_m, y_hi_m, radial_offset_m, angle_deg) of ``sleeve`` relative to ``cane``."""
    p_c, r_c = cane_pose()
    u_c = r_c[:, 1]
    p_s, r_s = mk.pose("sleeve")
    u_s = r_s[:, 1]
    ang_deg = math.degrees(math.acos(min(1.0, abs(float(np.dot(u_s, u_c))))))
    d = p_s - p_c
    radial = float(np.linalg.norm(d - float(np.dot(d, u_c)) * u_c))
    m1 = float(np.dot((p_s + 0.5 * P.SLEEVE_L * u_s) - p_c, u_c))
    m2 = float(np.dot((p_s - 0.5 * P.SLEEVE_L * u_s) - p_c, u_c))
    y_lo, y_hi = min(m1, m2), max(m1, m2)
    geom_ok = radial <= P.GEOM_RADIAL_MAX and ang_deg <= P.GEOM_ANGLE_DEG and P.GEOM_SLIDE_MIN <= y_hi <= P.GEOM_SLIDE_MAX
    return (sleeve_mated() or geom_ok), y_lo, y_hi, radial, ang_deg


def coaxial_enclosed() -> bool:
    return sleeve_span_on_cane()[0]


def sleeve_slide_m() -> float:
    """Axial station (m from straight foot) of the sleeve's leading mouth when mated or coaxially enclosing the cane."""
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


# --- zero-snap in-place foot engagement & 265 mm wipe / cocked bind -------------------------------------------------------
def sleeve_bending_moment_nm() -> float:
    if not sleeve_mated():
        return 0.0
    if mk.MECH.environment is not None and "right" in mk.MECH.gripper_turns and "left" in mk.MECH.gripper_turns:
        # Measure off-axis push directly from any transverse wrist force + contact readings
        pass
    touches = [c for c in mk.contacts("sleeve") if c.other.endswith("robot") and c.force > 0.0]
    if not touches:
        return 0.0
    p_c, r_c = cane_pose()
    u_c = r_c[:, 1]
    p_s, r_s = mk.pose("sleeve")
    u_s = r_s[:, 1]
    mouth = p_c + sleeve_slide_m() * u_c
    m_vec = np.zeros(3)
    for c in touches:
        d = c.point - p_s
        rad_vec = d - float(np.dot(d, u_s)) * u_s
        n_in = -c.normal if float(np.dot(c.normal, rad_vec)) > 0.0 else c.normal
        f_vec = float(c.force) * n_in
        f_perp = f_vec - float(np.dot(f_vec, u_c)) * u_c
        m_vec += np.cross(c.point - mouth, f_perp)
    return float(np.linalg.norm(m_vec - float(np.dot(m_vec, u_c)) * u_c))


def _try_engage_sleeve() -> None:
    """When ``sleeve``'s open mouth telescopes 10..60 mm over ``cane``'s straight bottom foot within 2.8 mm mouth radial
    offset (6.0 mm at sleeve centre) and 2.0 deg coaxially, ``mk.make('w_sleeve')`` joins ``sleeve`` to ``c_slv`` right
    as they stand (zero pose snap!)."""
    if sleeve_mated():
        return
    p_c, r_c = cane_pose()
    u_c = r_c[:, 1]
    p_s, r_s = mk.pose("sleeve")
    u_s = r_s[:, 1]
    cos_lim = math.cos(math.radians(P.ENTRY_ANGLE_DEG))
    for end_sign in (1.0, -1.0):
        if float(np.dot(end_sign * u_s, u_c)) < cos_lim:
            continue
        mouth = p_s + end_sign * (0.5 * P.SLEEVE_L) * u_s
        rel_m = mouth - p_c
        s_mouth = float(np.dot(rel_m, u_c))
        if not (P.ENTRY_S_MIN <= s_mouth <= P.ENTRY_S_MAX):
            continue
        rad_mouth = float(np.linalg.norm(rel_m - s_mouth * u_c))
        rel_c = p_s - p_c
        rad_centre = float(np.linalg.norm(rel_c - float(np.dot(rel_c, u_c)) * u_c))
        if rad_mouth > P.ENTRY_RADIAL_MAX or rad_centre > P.ENTRY_CENTRE_RADIAL_MAX:
            continue
        j_now = float(mk.joint("slv_s"))
        mk.make("w_sleeve")
        STATE["slv_rest"] = j_now
        STATE["entry_s_offset"] = s_mouth - j_now
        STATE["max_slide_m"] = s_mouth
        if mk.MECH.environment is not None:
            mk.set_spring("slv_s", rest=j_now, most=P.SLIDE_N)
        if STATE.get("engaged_at_s") is None:
            STATE["engaged_at_s"] = mk.now()
        return


def _step_sleeve_slide() -> None:
    if not sleeve_mated():
        STATE["slide_force_n"], STATE["bend_nm"] = 0.0, 0.0
        return
    j = float(mk.joint("slv_s"))
    s = sleeve_slide_m()
    bend = sleeve_bending_moment_nm()
    STATE["bend_nm"] = bend
    STATE["max_bend_nm"] = max(STATE.get("max_bend_nm", 0.0), bend)
    bound = (0.020 < s < 0.260) and (bend > P.BIND_TORQUE_NM)
    if bound:
        STATE["bound_steps"] = STATE.get("bound_steps", 0) + 1
    cap_n = P.BIND_N if bound else P.SLIDE_N
    rest = float(STATE.get("slv_rest", j))
    max_yield = cap_n / P.SLIDE_K
    if abs(j - rest) > max_yield:
        rest = j - math.copysign(max_yield, j - rest)
    STATE["slv_rest"] = rest
    force_n = min(cap_n, abs(P.SLIDE_K * (j - rest)))
    STATE["slide_force_n"] = force_n
    STATE["max_slide_f_n"] = max(STATE.get("max_slide_f_n", 0.0), force_n)
    STATE["max_slide_m"] = max(STATE.get("max_slide_m", 0.0), s)
    if STATE["max_slide_m"] > 0.020 and s <= 0.002 and (rest - j) * P.SLIDE_K >= 0.95 * P.SLIDE_N:
        mk.cut("w_sleeve")
        STATE["slv_rest"], STATE["slide_force_n"] = 0.0, 0.0
        if mk.MECH.environment is not None:
            mk.set_spring("slv_s", rest=0.0, most=P.SLIDE_N)
        return
    if mk.MECH.environment is not None:
        mk.set_spring("slv_s", rest=rest, most=cap_n)


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
    _try_engage_sleeve()
    _step_sleeve_slide()
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
        "engaged at (s)": STATE.get("engaged_at_s"),
        "max slide (mm)": round(1000.0 * STATE.get("max_slide_m", 0.0), 1),
        "max wipe force (N)": round(STATE.get("max_slide_f_n", 0.0), 2),
        "max bend moment (N m)": round(STATE.get("max_bend_nm", 0.0), 3),
        "bound steps": STATE.get("bound_steps", 0),
        "worst cane roll / pitch (deg)": [
            round(STATE.get("worst_roll_deg", 0.0), 1),
            round(STATE.get("worst_pitch_deg", 0.0), 1),
        ],
    }
    if mk.MECH.poses:
        out.update(
            {
                "sleeve mated": sleeve_mated(),
                "sleeve slide (mm)": round(1000.0 * sleeve_slide_m(), 1),
                "gap to top tab (mm)": None if not sleeve_mated() else round(1000.0 * gap_to_tab_m(), 1),
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
