"""Join a flat-pack shelf flush to an upright cabinet side on two Häfele Minifix S100 cam bolts and lock both Minifix 15
eccentric cams tight with two Wera 355 PZ stubby screwdrivers, then park both screwdrivers back in their cradles and
move both hands clear.

Built in ``scene.py`` from ``parts.py``: fixed upright ``cabinet_side`` with two Ø 7.0 mm S100 bolts at y = +-0.110 m;
free 16 mm ``shelf`` sliding on ``support_block`` with two 16-stave Ø 8.00 mm horizontal edge bores (0.50 mm radial play)
into two Ø 15.0 mm cam pockets; two free zinc housings ``cam_l`` and ``cam_r`` on vertical hinges starting +90 deg closed
(blocking bolt entry ~8.5 mm short unless opened within +-25 deg first) whose compliant eccentric spiral flanges draw the
seam flush and lock over [150, 195] deg CW; and two free 37 g stubbies ``driver_l``, ``driver_r`` coupled to a cam by
strictly equal-and-opposite axial torque (+tau on cam, -tau on driver; zero lateral or tilt support) only while the tip
sits >= 1.5 mm inside the 3.5 mm recess with >= 1.5 N PhysX contact force.
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
SUCCESS_HOLD_S = 3.0  # the arena ends the run once success has held 3.0 s (rta/spec.py); score() needs its own 1.0 s hold
INSTRUCTION = (
    "Join the flat-pack shelf flush to the upright cabinet side on its two cam bolts with both cam locks turned tight "
    "using the stubby screwdrivers, then put both screwdrivers back in their cradles and move both hands clear."
)
RUN_LIMIT_STEPS = 1800  # 90 s at 20 Hz
TOP = TABLE_TOP[SCENE]
CONNS = scene.CONNS  # ("l", "r")

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
        slot_wings={},
        couplings={},
        peak_tau={"l": 0.0, "r": 0.0},
        both_opened=False,
        first_seated_s=None,
        first_locked={},
    )
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _wrap_quarter(a: float) -> float:
    q = 0.5 * math.pi
    return a - q * round(a / q)


def shelf_pose() -> tuple[np.ndarray, np.ndarray]:
    return mk.pose("shelf")


def shelf_tilt_deg() -> float:
    return float(math.degrees(math.acos(max(-1.0, min(1.0, float(shelf_pose()[1][2, 2]))))))


def shelf_yaw_deg() -> float:
    r = shelf_pose()[1]
    return float(math.degrees(math.atan2(float(r[1, 0]), float(r[0, 0]))))


def seam_gaps() -> dict[str, float]:
    """Horizontal +x gap (m) from the shelf's joining edge at y = +-CAM_Y to the upright cabinet side's mating face."""
    p_sh, r_sh = shelf_pose()
    p_side, r_side = mk.pose("cabinet_side")
    n_in = r_side @ np.array([-1.0, 0.0, 0.0])
    return {
        c: float((p_sh + r_sh @ np.array([P.SHELF_D / 2.0, scene.side_sign(c) * P.CAM_Y, 0.0]) - p_side) @ n_in)
        for c in CONNS
    }


def lateral_offset() -> float:
    p_sh, r_sh = shelf_pose()
    p_side, r_side = mk.pose("cabinet_side")
    mid = p_sh + r_sh @ np.array([P.SHELF_D / 2.0, 0.0, 0.0])
    return abs(float((mid - p_side) @ (r_side @ np.array([0.0, 1.0, 0.0]))))


def cam_deg(c: str) -> float:
    return P.cam_deg_from_travel(_wrap_pi(mk.joint(f"cam_joint_{c}")))


def cam_open(c: str) -> bool:
    return abs(_wrap_pi(math.radians(cam_deg(c)))) <= math.radians(P.CAM_OPEN_TOL_DEG)


def cam_clamp_force(c: str) -> float:
    return mk.contact_force(f"cam_{c}", "cabinet_side")


def cam_locked(c: str) -> bool:
    return P.CAM_LOCK_MIN_DEG <= cam_deg(c) <= P.CAM_LOCK_MAX_DEG and cam_clamp_force(c) >= P.CAM_CLAMP_N


def driver_tip(d: str) -> tuple[np.ndarray, np.ndarray]:
    p, r = mk.pose(f"driver_{d}")
    return p + r @ np.array([0.0, 0.0, P.DRV_TIP_Z]), r[:, 2]


def driver_parked(d: str) -> bool:
    tip, axis = driver_tip(d)
    p_cr, _ = mk.pose(f"cradle_{d}")
    horiz = float(math.hypot(tip[0] - p_cr[0], tip[1] - p_cr[1]))
    tilt = float(math.degrees(math.acos(max(-1.0, min(1.0, float(-axis[2]))))))
    return horiz <= P.CRADLE_POS_MAX and (tip[2] - TOP) <= P.CRADLE_TIP_Z_MAX and tilt <= P.CRADLE_TILT_MAX_DEG


def slot_engagement(d: str, c: str) -> dict:
    tip, drv_axis = driver_tip(d)
    p_cam, r_cam = mk.pose(f"cam_{c}")
    cam_axis = shelf_pose()[1] @ np.array([0.0, 0.0, -1.0])
    local = r_cam.T @ (tip - p_cam)
    radial = float(math.hypot(local[0], local[1]))
    depth = float(P.PANEL_T / 2.0 - local[2])
    force = mk.contact_force(f"driver_{d}", f"cam_{c}")
    tilt = float(math.degrees(math.acos(max(-1.0, min(1.0, float(drv_axis @ cam_axis))))))
    ph = math.radians(P.DRV_BLADE_PHASE_DEG)
    blade_x = mk.pose(f"driver_{d}")[1] @ np.array([math.cos(ph), math.sin(ph), 0.0])
    u_x = r_cam[:, 0]
    u_cw = np.cross(cam_axis, u_x)
    psi = float(math.atan2(float(blade_x @ u_cw), float(blade_x @ u_x)))
    in_recess = (
        radial <= P.SLOT_RADIAL_TOL
        and P.SLOT_DEPTH_MIN <= depth <= P.SLOT_RECESS_DEPTH + 0.0025
        and force >= P.SLOT_FORCE_MIN
        and tilt <= P.SLOT_TILT_MAX_DEG
    )
    return {"in_recess": in_recess, "radial": radial, "depth": depth, "force": force, "tilt": tilt, "psi": psi, "cam_axis": cam_axis}


def update_couplings() -> dict[str, np.ndarray]:
    """Strictly equal-and-opposite axial torque (+tau on cam, -tau on driver; zero force) for engaged PZ2 cross slots."""
    wings: dict = STATE.setdefault("slot_wings", {})
    torques: dict[str, np.ndarray] = {f"{kind}_{c}": np.zeros(3) for kind in ("cam", "driver") for c in CONNS}
    used_cams: set[str] = set()
    for d in CONNS:
        for c in CONNS:
            key = (d, c)
            eng = slot_engagement(d, c)
            if not eng["in_recess"] or c in used_cams:
                wings.pop(key, None)
                continue
            psi = eng["psi"]
            if key not in wings:
                wing_err = _wrap_quarter(psi)
                if abs(wing_err) > math.radians(P.SLOT_CAPTURE_DEG):
                    continue
                wings[key] = _wrap_pi(psi - wing_err)
            err = _wrap_pi(psi - wings[key])
            if abs(err) > math.radians(P.SLOT_CAMOUT_DEG):
                wings.pop(key, None)
                continue
            cam_axis = eng["cam_axis"]
            domega = float((mk.velocity(f"driver_{d}")[1] - mk.velocity(f"cam_{c}")[1]) @ cam_axis)
            tau = float(np.clip(P.SLOT_K * err + P.SLOT_D * domega, -P.SLOT_TAU_MAX, P.SLOT_TAU_MAX))
            tau_vec = tau * cam_axis
            torques[f"cam_{c}"] = torques[f"cam_{c}"] + tau_vec
            torques[f"driver_{d}"] = torques[f"driver_{d}"] - tau_vec
            STATE["peak_tau"][c] = max(STATE["peak_tau"].get(c, 0.0), abs(tau))
            used_cams.add(c)
            break
    STATE["couplings"] = {k: tuple(float(v) for v in vec) for k, vec in torques.items()}
    if mk.MECH.environment is not None:
        for name, vec in torques.items():
            mk.push(name, force=(0.0, 0.0, 0.0), torque=tuple(float(v) for v in vec))
    return torques


def hands_clear() -> float:
    return scene.nearest_hand_clearance(STATE.get("hands"), ("shelf", "cam_l", "cam_r", "driver_l", "driver_r"))


def zero_now() -> str | None:
    if shelf_pose()[0][2] < TOP + P.DROP_ZERO_Z:
        return "the shelf dropped off the support block"
    for c in CONNS:
        if mk.pose(f"driver_{c}")[0][2] < TOP + P.DRV_BENCH_ZERO_Z:
            return f"screwdriver {c} fell to the bench"
    return None


def seated_on_bolts(max_gap: float) -> bool:
    gaps = seam_gaps()
    return (
        all(P.GAP_MIN <= g <= max_gap for g in gaps.values())
        and lateral_offset() <= P.LATERAL_MAX
        and shelf_tilt_deg() <= P.TILT_MAX_DEG
    )


def full_now() -> bool:
    return (
        seated_on_bolts(P.FLUSH_GAP_MAX)
        and all(cam_locked(c) for c in CONNS)
        and all(driver_parked(c) for c in CONNS)
        and hands_clear() >= P.HANDS_CLEAR
    )


@mk.on_step
def step() -> None:
    if mk.MECH.environment is not None:
        STATE["hands"] = scene.read_hand_points()
    update_couplings()
    if STATE.get("zero") is None:
        STATE["zero"] = zero_now()
    if all(cam_open(c) for c in CONNS):
        STATE["both_opened"] = True
    t_now = mk.now() if mk.MECH.environment is not None else 0.0
    if STATE.get("first_seated_s") is None and seated_on_bolts(P.SEATED_GAP_MAX):
        STATE["first_seated_s"] = round(t_now, 3)
    for c in CONNS:
        if c not in STATE.setdefault("first_locked", {}) and seated_on_bolts(P.SEATED_GAP_MAX) and cam_locked(c):
            STATE["first_locked"][c] = round(t_now, 3)
    if full_now():
        if STATE.get("hold", 0) == 0:
            STATE["hold_from"] = shelf_pose()[0].copy()
        STATE["hold"] = STATE.get("hold", 0) + 1
        if float(np.linalg.norm(shelf_pose()[0] - STATE["hold_from"])) > P.HOLD_DRIFT:
            STATE["hold"], STATE["hold_from"] = 0, None
    else:
        STATE["hold"], STATE["hold_from"] = 0, None


def level() -> float:
    if STATE.get("zero") is not None or zero_now() is not None:
        return 0.0
    if STATE.get("hold", 0) >= P.HOLD_STEPS and full_now():
        return 1.0
    if seated_on_bolts(P.SEATED_GAP_MAX):
        return 0.50 if any(cam_locked(c) for c in CONNS) else 0.25
    return 0.0


def score(objects) -> float:
    """0 permanently if the shelf drops below z = 56 mm (off the support block) or either screwdriver falls to the bench
    (z < 32 mm). Otherwise at the end of the trial: 0.25 when the shelf is aligned and pushed onto both bolts (both seam
    gaps <= 2.5 mm, the cams' draw; lateral offset <= 2.0 mm, which the 7 mm heads in the 8.0 mm bores hold within
    0.5 mm; tilt <= 3.0 deg); 0.50 when seated so and at least one cam is
    locked in [150, 195] deg CW from open clamping its bolt (>= 5.0 N); 1.00 when both seam gaps are <= 0.6 mm, both
    cams are locked in [150, 195] deg (>= 5.0 N each), both stubby screwdrivers stand upright in their cradles (tip
    within 12 mm horizontally, <= 20 mm above bench, tilt <= 15 deg), and both hands are >= 30 mm clear, held 1.0 s
    (20 steps) with <= 1.0 mm shelf drift."""
    _ = objects
    return level()


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out = {
        "zero": STATE.get("zero"),
        "hold_steps": STATE.get("hold", 0),
        "both_opened": STATE.get("both_opened", False),
        "first_seated_s": STATE.get("first_seated_s"),
        "first_locked_s": STATE.get("first_locked"),
        "peak_slot_torque_nm": {c: round(v, 3) for c, v in STATE.get("peak_tau", {}).items()},
    }
    if mk.MECH.poses:
        gaps = seam_gaps()
        out.update({
            "seam_gaps_mm": {c: round(1000.0 * gaps[c], 2) for c in CONNS},
            "lateral_off_mm": round(1000.0 * lateral_offset(), 2),
            "shelf_tilt_deg": round(shelf_tilt_deg(), 2),
            "shelf_yaw_deg": round(shelf_yaw_deg(), 2),
            "cam_deg": {c: round(cam_deg(c), 1) for c in CONNS},
            "cam_clamp_n": {c: round(cam_clamp_force(c), 2) for c in CONNS},
            "shelf_on_side_n": round(mk.contact_force("shelf", "cabinet_side"), 2),
            "cam_locked": {c: cam_locked(c) for c in CONNS},
            "driver_parked": {c: driver_parked(c) for c in CONNS},
            "hands_clear_mm": round(1000.0 * hands_clear(), 1),
            "score": score(None),
        })
    return out
