"""Rolling glass bottle on a sloped kitchen drainboard above an open worktop edge, with a recessed cork coaster set
aside from the rolling lane.

Instruction: Stop the rolling glass bottle before it goes over the edge of the worktop, then stand it upright on the
cork coaster.

All product sizes, masses, inertia ratios, slope kinematics, rolling-resistance and cork-damping parameters, and
scoring limits come from ``parts.py`` (helper functions in ``roll_mech.py``):
- ``worktop``, ``drainboard``, and ``coaster`` are rigid bodies fixed to the bench (``kinematic=True``: clamped,
  screwed, or taped down on its mark with double-sided mounting tape); worktop and drainboard meet with a 0.000 mm
  step at ``y = +0.110 m, z = TOP + 0.038 m``.
- ``bottle`` is a free straight cylindrical glass bottle with axial inertia ``ROT_INERTIA_FACTOR * m * R^2`` (sealed
  water slip), zero linear damping, and ``BOTTLE_ANGULAR_DAMPING`` (0.05/s). Zero welds or proximity catches are used.
- Rolling resistance ``TAU_ROLL`` acts ONLY while counter normal force >= 0.5 N (reaction into clamped counter), and is
  0.000000 N m whenever airborne, upright on the coaster, or at rest.
"""

from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np

TASK_FILE = Path(__file__).resolve()
sys.path.insert(0, str(TASK_FILE.parent))

import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402
import roll_mech as R  # noqa: E402
from mechkit import body, matrix_to_quat, sense_contact  # noqa: E402

SCENE = P.SCENE
SCORE_OVER = "end"
SUCCESS_HOLD_S = 3.5  # s: keeps episode running after 1.0 first holds so the arm parks at READY for the final frame
INSTRUCTION = (
    "Stop the rolling glass bottle before it goes over the edge of the worktop, "
    "then stand it upright on the cork coaster."
)
RUN_LIMIT_STEPS = P.RUN_LIMIT_STEPS  # 800 steps (40.0 s at 20 Hz)
TOP = P.TOP

_drain_pos, _drain_rot = P.drainboard_pose()
_bottle_pos0, _bottle_rot0 = P.bottle_start_pose()

body("worktop", P.worktop_shapes(), pos=(P.WORKTOP_X, P.WORKTOP_Y, P.TOP + 0.5 * P.WORKTOP_H), kinematic=True, friction=P.WORKTOP_FRICTION, contact_offset=P.CONTACT_OFFSET)
body("drainboard", P.drainboard_shapes(), pos=tuple(_drain_pos), quat=matrix_to_quat(_drain_rot), kinematic=True, friction=P.DRAIN_FRICTION, contact_offset=P.CONTACT_OFFSET)
body("catch_mat", P.catch_mat_shapes(), pos=(P.CATCH_MAT_X, P.CATCH_MAT_Y, P.TOP + 0.5 * P.CATCH_MAT_H), kinematic=True, friction=P.CATCH_MAT_FRICTION, contact_offset=P.CONTACT_OFFSET)
body("coaster", P.coaster_shapes(), pos=(P.COASTER_X, P.COASTER_Y, P.WORKTOP_TOP_Z + 0.0005), kinematic=True, friction=P.COASTER_FRICTION, contact_offset=P.CONTACT_OFFSET)
body("bottle", P.bottle_shapes(), pos=tuple(_bottle_pos0), quat=matrix_to_quat(_bottle_rot0), mass=P.BOTTLE_MASS, inertia=P.bottle_inertia(), friction=P.BOTTLE_FRICTION, linear_damping=P.BOTTLE_LINEAR_DAMPING, angular_damping=P.BOTTLE_ANGULAR_DAMPING, contact_offset=P.CONTACT_OFFSET, contacts=True)

sense_contact("bottle", "worktop", "drainboard", "coaster")
OBJECTS = list(mk.MECH.bodies.values())

AWAY_GRIPPERS = R.AWAY_GRIPPERS
MARKS_Y = R.MARKS_Y
bottle_lowest_z = R.bottle_lowest_z
bottle_base_center = R.bottle_base_center
grippers_clear = R.grippers_clear
support_contact_force_n = R.support_contact_force_n
compute_roll_torque = R.compute_roll_torque

STATE: dict = {}


def begin() -> None:
    STATE.clear()
    STATE.update(
        released=False,
        release_t=0.0,
        prev_sample=None,
        crossings={},
        off_edge=None,
        safe_steps=0,
        upright_steps=0,
        events=[],
        max_support_force_n=0.0,
        max_coaster_force_n=0.0,
        max_tau_roll_nm=0.0,
        max_airborne_tau_nm=0.0,
        max_coaster_shift_m=0.0,
        min_bottle_coaster_gap_m=math.inf,
        last_tau_roll=np.zeros(3, dtype=float),
    )


begin()
R.install_post_settle_release(TASK_FILE, STATE)


@mk.on_reset
def start() -> None:
    begin()
    if mk.MECH.environment is not None:
        mk.push("bottle", force=P.settle_hold_force(), torque=(0.0, 0.0, 0.0))


def bottle_at_rest() -> bool:
    lin_v, ang_v = mk.velocity("bottle") if "bottle" in mk.MECH.velocities else (np.zeros(3), np.zeros(3))
    return float(np.linalg.norm(lin_v)) < P.REST_LIN_MPS and float(np.linalg.norm(ang_v)) < P.REST_ANG_RADPS


def coaster_at_rest() -> bool:
    lin_v, ang_v = mk.velocity("coaster") if "coaster" in mk.MECH.velocities else (np.zeros(3), np.zeros(3))
    return float(np.linalg.norm(lin_v)) < P.REST_LIN_MPS and float(np.linalg.norm(ang_v)) < P.REST_ANG_RADPS


def coaster_shift_m() -> float:
    p_c, _ = mk.pose("coaster")
    return math.hypot(float(p_c[0]) - P.COASTER_X, float(p_c[1]) - P.COASTER_Y)


def coaster_in_place() -> bool:
    p_c, r_c = mk.pose("coaster")
    return (
        coaster_shift_m() <= P.COASTER_MAX_SHIFT_M
        and float(r_c[2, 2]) >= math.cos(math.radians(10.0))
        and abs(float(p_c[2]) - P.WORKTOP_TOP_Z) <= 0.005
    )


def resting_on_counter() -> bool:
    """Bottle's lowest point rests on worktop, drainboard, or coaster with >= 1.0 N normal contact."""
    p_b, r_b = mk.pose("bottle")
    z_low = bottle_lowest_z(p_b, r_b)
    z_surf = P.surface_z_at(float(p_b[0]), float(p_b[1]))
    geom_ok = P.WORKTOP_TOP_Z - 0.004 <= z_low <= z_surf + P.COASTER_TOTAL_H + 0.003
    return geom_ok and R.total_counter_contact_force_n() >= P.REST_CONTACT_MIN_N


def safe_stopped_now() -> bool:
    return STATE["off_edge"] is None and bottle_at_rest() and coaster_at_rest() and grippers_clear() and resting_on_counter()


def upright_in_coaster_now() -> bool:
    if not (safe_stopped_now() and coaster_in_place()):
        return False
    if float(mk.contact_force("bottle", "coaster")) < P.REST_CONTACT_MIN_N:
        return False
    p_b, r_b = mk.pose("bottle")
    p_c, _ = mk.pose("coaster")
    if float(r_b[2, 2]) < math.cos(math.radians(P.UPRIGHT_MAX_DEG)):
        return False
    base = bottle_base_center(p_b, r_b)
    radial = math.hypot(float(base[0] - p_c[0]), float(base[1] - p_c[1]))
    on_floor = abs(float(base[2]) - (float(p_c[2]) + P.COASTER_FLOOR_H)) <= P.COASTER_RIM_H + 0.004
    return radial <= P.COASTER_BASE_TOL_M and on_floor


@mk.on_step
def step() -> None:
    now = mk.now()
    p_b, r_b = mk.pose("bottle")
    p_c, _ = mk.pose("coaster")
    lin_v = mk.velocity("bottle")[0] if "bottle" in mk.MECH.velocities else np.zeros(3)
    speed = float(np.linalg.norm(lin_v))

    if not STATE["released"]:  # pure unit-test fallback when YamArenaEmbodiment.reset did not run
        STATE["released"] = True
        STATE["release_t"] = now
        STATE["prev_sample"] = (now, p_b.copy(), speed)
    else:
        prev_t, prev_p, prev_v = STATE["prev_sample"]
        for mark, y_target in MARKS_Y.items():
            if mark not in STATE["crossings"] and float(prev_p[1]) > y_target >= float(p_b[1]):
                dy = float(prev_p[1] - p_b[1])
                alpha = (float(prev_p[1]) - y_target) / dy if dy > 1e-9 else 1.0
                t_cross = prev_t + alpha * (now - prev_t)
                STATE["crossings"][mark] = {
                    "t_s": round(t_cross, 4),
                    "dt_from_release_s": round(t_cross - STATE["release_t"], 4),
                    "speed_mps": round(prev_v + alpha * (speed - prev_v), 4),
                    "x_m": round(float(prev_p[0]) + alpha * float(p_b[0] - prev_p[0]), 4),
                }
        STATE["prev_sample"] = (now, p_b.copy(), speed)

    tau = compute_roll_torque()
    STATE["last_tau_roll"] = tau
    tau_norm = float(np.linalg.norm(tau))
    f_sup = support_contact_force_n()
    f_c = float(mk.contact_force("bottle", "coaster"))
    STATE["max_support_force_n"] = max(STATE["max_support_force_n"], f_sup)
    STATE["max_coaster_force_n"] = max(STATE["max_coaster_force_n"], f_c)
    STATE["max_tau_roll_nm"] = max(STATE["max_tau_roll_nm"], tau_norm)
    if f_sup < P.ROLL_CONTACT_MIN_N:
        STATE["max_airborne_tau_nm"] = max(STATE["max_airborne_tau_nm"], tau_norm)
    if mk.MECH.environment is not None:
        mk.push("bottle", force=(0.0, 0.0, 0.0), torque=(float(tau[0]), float(tau[1]), float(tau[2])))

    STATE["max_coaster_shift_m"] = max(STATE["max_coaster_shift_m"], coaster_shift_m())
    STATE["min_bottle_coaster_gap_m"] = min(STATE["min_bottle_coaster_gap_m"], R.horizontal_bottle_coaster_gap_m(p_b, r_b, p_c))

    if STATE["off_edge"] is None:
        reason = R.off_edge_reason(p_b, r_b)
        if reason is not None:
            STATE["off_edge"] = (round(now, 3), reason)
            STATE["events"].append((round(now, 3), f"off edge: {reason}"))

    if STATE["off_edge"] is not None:
        STATE["safe_steps"] = 0
        STATE["upright_steps"] = 0
    else:
        was_safe, was_up = STATE["safe_steps"], STATE["upright_steps"]
        STATE["safe_steps"] = (was_safe + 1) if safe_stopped_now() else 0
        STATE["upright_steps"] = (was_up + 1) if upright_in_coaster_now() else 0
        if was_safe < P.HOLD_STEPS <= STATE["safe_steps"]:
            STATE["events"].append((round(now, 3), "stopped safely with grippers clear for 1.0 s"))
        if was_up < P.HOLD_STEPS <= STATE["upright_steps"]:
            STATE["events"].append((round(now, 3), "upright cap-up in coaster with grippers clear for 1.0 s"))


def score(_objects) -> float:
    """Whole-trial off-edge latch -> 0.0; upright cap-up in coaster held 20 steps -> 1.0; resting safely on the counter
    with grippers clear held 20 steps (e.g. lying on its side or standing cap-down / outside the coaster) -> 0.5."""
    if STATE["off_edge"] is not None:
        return 0.0
    if STATE["upright_steps"] >= P.HOLD_STEPS:
        return 1.0
    if STATE["safe_steps"] >= P.HOLD_STEPS:
        return 0.5
    return 0.0


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    crossings = dict(STATE.get("crossings", {}))
    foot = crossings.get("foot")
    dt_from_foot = {k: round(crossings[k]["t_s"] - foot["t_s"], 4) for k in ("150mm", "300mm", "edge") if foot and k in crossings}
    out: dict = {
        "preset": P.BOTTLE_PRESET,
        "release_t_s": STATE.get("release_t", 0.0),
        "off_edge": STATE.get("off_edge"),
        "safe_steps": STATE.get("safe_steps", 0),
        "upright_steps": STATE.get("upright_steps", 0),
        "crossings": crossings,
        "dt_from_foot_s": dt_from_foot,
        "max_support_force_n": round(STATE.get("max_support_force_n", 0.0), 3),
        "max_coaster_force_n": round(STATE.get("max_coaster_force_n", 0.0), 3),
        "max_tau_roll_nm": round(STATE.get("max_tau_roll_nm", 0.0), 6),
        "max_airborne_tau_nm": round(STATE.get("max_airborne_tau_nm", 0.0), 6),
        "max_coaster_shift_mm": round(1000.0 * STATE.get("max_coaster_shift_m", 0.0), 2),
        "events": [f"{t:.2f} s: {what}" for t, what in STATE.get("events", [])],
    }
    if mk.MECH.poses:
        p_b, r_b = mk.pose("bottle")
        p_c, _ = mk.pose("coaster")
        base = bottle_base_center(p_b, r_b)
        out.update(
            {
                "bottle_pos_m": [round(float(v), 4) for v in p_b],
                "bottle_tilt_from_up_deg": round(math.degrees(math.acos(max(-1.0, min(1.0, float(r_b[2, 2]))))), 2),
                "base_to_coaster_mm": round(1000.0 * math.hypot(float(base[0] - p_c[0]), float(base[1] - p_c[1])), 2),
                "coaster_shift_mm": round(1000.0 * coaster_shift_m(), 2),
                "grippers_clear": grippers_clear(),
            }
        )
    return out
