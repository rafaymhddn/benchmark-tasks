"""Level a smartphone on a Manfrotto PIXI mini tripod's push-button ball head without letting it flop:
one hand holds the phone before the other presses the head's release button, levels the phone in both tilt and roll while
the button stays pressed, releases the button so the head locks, and then lets go of the phone.

Every product size, mass, angle, force, torque and scoring limit comes from ``parts.py``; the bilateral ball-head lock
and push-button spring live in ``head.py``.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import head as H  # noqa: E402
import kit_extra as kx  # noqa: E402
import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402
from mechkit import body, quat_to_matrix  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 2.0  # holds full score 2 s after the 1 s at-rest window so both arms park cleanly at READY (Check 48)
INSTRUCTION = "Make the phone on the mini tripod level, and leave it standing level on its own."
RUN_LIMIT_STEPS = 1200  # 60 s at 20 Hz (~2.5x the ~24 s reference run)
TOP = TABLE_TOP[SCENE]

SCORED_BODIES = ("tripod", "clamp", "phone", "button")
CLEAR_BODIES = ("phone", "clamp", "tripod", "button")
ALL_PASS_NAMES = ("tripod", "button", "c_tilt", "clamp", "phone")
START_MARKS_XY = P.foot_marks_world_xy()

# --- scene bodies ---
body("bench_marks", P.bench_mark_shapes(), pos=(0.0, 0.0, TOP), mass=None)

body(
    "tripod",
    P.tripod_shapes(),
    pos=(P.TRI_XY[0], P.TRI_XY[1], TOP + P.OVER),
    yaw=P.TRI_YAW,
    mass=P.TRI_BODY_MASS,
    friction=(P.TRI_FRICTION, P.TRI_FRICTION),
    friction_combine="average",
    linear_damping=0.4,
    angular_damping=0.8,
    contact_offset=P.CONTACT_THIN,
    contacts=True,
)

body(
    "button",
    P.button_shapes(),
    pos=P.button_built_pos(TOP),
    yaw=P.TRI_YAW,
    mass=P.BUTTON_MASS,
    inertia=P.BUTTON_INERTIA,
    linear_damping=0.5,
    angular_damping=0.5,
    contact_offset=P.CONTACT_THIN,
    contacts=True,
)

body(
    "c_tilt",
    P.carrier_shapes(ALL_PASS_NAMES),
    pos=P.ball_anchor_world(TOP),
    mass=P.C_TILT_MASS,
    inertia=P.C_TILT_INERTIA,
    contact_offset=P.CONTACT_THIN,
)

body(
    "clamp",
    P.clamp_shapes(),
    pos=P.clamp_built_pos(TOP),
    mass=P.CLAMP_BODY_MASS,
    inertia=P.CLAMP_INERTIA,
    center=(0.0, 0.0, 0.012),
    linear_damping=0.2,
    angular_damping=0.2,
    contact_offset=P.CONTACT_THIN,
)

body(
    "phone",
    P.phone_shapes(),
    pos=P.phone_built_pos(TOP),
    mass=P.PHONE_MASS,
    linear_damping=0.1,
    angular_damping=0.1,
    contact_offset=P.CONTACT_THIN,
    contacts=True,
)

H.declare(TOP)

OBJECTS = list(mk.MECH.bodies.values())

EVENTS = kx.Events(
    {
        "phone held": 0.1,
        "button pressed": 0.1,
        "phone levelled": 0.2,
        "button released": 0.2,
        "hands clear and at rest": P.REST_HOLD_S,
    }
)

STATE: dict = {}


def _hide_carrier() -> None:
    for idx in range(len(mk.MECH.bodies["c_tilt"].shapes)):
        mk.show("c_tilt", idx, False)


def begin(seed: int | None = None) -> dict:
    STATE.clear()
    STATE.update(
        seed=seed,
        events=[],
        zero_fault=None,
        max_foot_lift_mm=0.0,
        max_foot_slide_mm=0.0,
        phone_touched_bench=False,
        clear_rest_s=0.0,
        last_time=None,
    )
    EVENTS.clear()
    _hide_carrier()
    return STATE


begin()


@mk.on_reset
def start(seed: int | None = None) -> None:
    begin(seed)
    H.reset()


# --- physical & geometric predicates (no simulator-only hidden quantities: Check 1 & Check 2) ---


def foot_positions_world() -> np.ndarray:
    """3x3 world positions of the bottoms of the tripod's three feet."""
    tp, tr = mk.pose("tripod")
    return P.foot_points_local() @ tr.T + tp


def foot_lifts_m() -> np.ndarray:
    """Height (m) of each tripod foot bottom above ``TOP``."""
    return foot_positions_world()[:, 2] - TOP


def foot_slides_m() -> np.ndarray:
    """Horizontal distance (m) of each tripod foot from its start tape mark."""
    return np.linalg.norm(foot_positions_world()[:, :2] - START_MARKS_XY, axis=1)


def phone_lowest_z_m() -> float:
    """Lowest world height (m) of the 147.6 x 71.6 x 7.8 mm phone box."""
    pp, pr = mk.pose("phone")
    half = np.array([0.5 * P.PHONE_T, 0.5 * P.PHONE_L, 0.5 * P.PHONE_W], dtype=float)
    reach = float(np.abs(pr[2]) @ half)
    return float(pp[2]) - reach


def phone_touches_bench() -> bool:
    """Whether the phone touches the bench (reported contact with ``scene`` or lowest corner <= bench top + 1 mm)."""
    if mk.touching("phone", "scene"):
        return True
    return phone_lowest_z_m() <= TOP + 0.001


def phone_tilt_roll_deg() -> tuple[float, float]:
    """(tilt_deg, roll_deg) of ``phone`` in the world against gravity, as read by a phone level app."""
    _, pr = mk.pose("phone")
    return P.phone_angles_deg(pr)


def phone_max_lean_deg() -> float:
    t_deg, r_deg = phone_tilt_roll_deg()
    return max(abs(t_deg), abs(r_deg))


def button_out() -> bool:
    """Button sprung back out within ``BUTTON_OUT_MAX`` (0.5 mm) and head locked."""
    return H.button_travel_m() <= P.BUTTON_OUT_MAX + 1e-9 and not H.is_unlocked()


def _point_to_shape_m(point: np.ndarray, body_pos: np.ndarray, body_rot: np.ndarray, shape: mk.Shape) -> float:
    sr = body_rot @ quat_to_matrix(shape.quat)
    sc = body_pos + body_rot @ np.asarray(shape.pos, dtype=float)
    q = sr.T @ (point - sc)
    if shape.kind == "box":
        outside = np.maximum(np.abs(q) - 0.5 * np.asarray(shape.dims, dtype=float), 0.0)
        return float(np.linalg.norm(outside))
    if shape.kind == "sphere":
        return max(0.0, float(np.linalg.norm(q)) - float(shape.dims[0]))
    radius, length = float(shape.dims[0]), float(shape.dims[1])
    if shape.kind == "cylinder":
        radial = max(0.0, math.hypot(float(q[0]), float(q[1])) - radius)
        axial = max(0.0, abs(float(q[2])) - 0.5 * length)
        return math.hypot(radial, axial)
    if shape.kind == "capsule":
        dz = min(0.5 * length, max(-0.5 * length, float(q[2])))
        return max(0.0, float(np.linalg.norm(np.array([q[0], q[1], q[2] - dz]))) - radius)
    outside = np.maximum(np.abs(q) - shape.half_extents(), 0.0)
    return float(np.linalg.norm(outside))


def point_to_body_m(point, body_name: str) -> float:
    """Exact surface distance (m) from world ``point`` to the solid shapes of ``body_name``."""
    p = np.asarray(point, dtype=float)
    bp, br = mk.pose(body_name)
    return min(
        _point_to_shape_m(p, bp, br, s)
        for s in mk.MECH.bodies[body_name].shapes
        if s.solid
    )


def min_gripper_clearance_m() -> float:
    """Smallest distance (m) from either gripper's grasp point to ``phone``, ``clamp``, ``tripod`` or ``button``."""
    best = math.inf
    for side in ("left", "right"):
        if side not in mk.MECH.grippers:
            continue
        gp = mk.gripper(side)[0]
        for bname in CLEAR_BODIES:
            best = min(best, point_to_body_m(gp, bname))
    return best


def grippers_clear() -> bool:
    if not all(side in mk.MECH.grippers for side in ("left", "right")):
        return False
    return min_gripper_clearance_m() >= P.GRIPPER_CLEAR - 1e-9


def bodies_at_rest() -> bool:
    for name in SCORED_BODIES:
        lin, ang = mk.velocity(name)
        if float(np.linalg.norm(lin)) > P.REST_LIN_MAX + 1e-9:
            return False
        if float(np.linalg.norm(ang)) > P.REST_ANG_MAX + 1e-9:
            return False
    return True


def _phone_held_now() -> bool:
    if mk.touching("phone", "left robot") or mk.touching("phone", "right robot"):
        return True
    for side in ("left", "right"):
        if side in mk.MECH.grippers and point_to_body_m(mk.gripper(side)[0], "phone") <= 0.012 and mk.gripper(side)[1] < 0.35:
            return True
    return False


@mk.on_step
def step() -> None:
    now = mk.now()
    prev_t = STATE["last_time"]
    dt = now if prev_t is None else max(0.0, now - float(prev_t))
    STATE["last_time"] = now

    H.watch(STATE["events"])

    lifts = foot_lifts_m()
    slides = foot_slides_m()
    max_lift_m = float(np.max(lifts))
    max_slide_m = float(np.max(slides))
    STATE["max_foot_lift_mm"] = max(STATE["max_foot_lift_mm"], 1000.0 * max_lift_m)
    STATE["max_foot_slide_mm"] = max(STATE["max_foot_slide_mm"], 1000.0 * max_slide_m)

    if STATE["zero_fault"] is None:
        if max_lift_m > P.FOOT_LIFT_MAX + 1e-9:
            STATE["zero_fault"] = f"tripod foot lifted {1000 * max_lift_m:.1f} mm (> {1000 * P.FOOT_LIFT_MAX:.0f} mm)"
            STATE["events"].append((now, STATE["zero_fault"]))
        elif max_slide_m > P.FOOT_SLIDE_MAX + 1e-9:
            STATE["zero_fault"] = f"tripod foot slid {1000 * max_slide_m:.1f} mm from mark (> {1000 * P.FOOT_SLIDE_MAX:.0f} mm)"
            STATE["events"].append((now, STATE["zero_fault"]))
        elif phone_touches_bench():
            STATE["phone_touched_bench"] = True
            STATE["zero_fault"] = "phone touched the bench"
            STATE["events"].append((now, STATE["zero_fault"]))
        elif H.locked_slip_deg() > P.LOCK_SLIP_MAX_DEG + 1e-9:
            # The head forced past LOCK_NM with the button out (on a bench: the phone turns while the button is out)
            STATE["zero_fault"] = f"locked head slipped {H.locked_slip_deg():.2f} deg (> {P.LOCK_SLIP_MAX_DEG:.1f} deg)"
            STATE["events"].append((now, STATE["zero_fault"]))

    held = _phone_held_now()
    pressed = H.is_unlocked()
    levelled = phone_max_lean_deg() <= P.LEVEL_DEG + 1e-9
    EVENTS.note("phone held", held, now)
    EVENTS.note("button pressed", pressed, now)
    EVENTS.note("phone levelled", levelled, now)
    EVENTS.note("button released", EVENTS.happened("button pressed") and button_out(), now)

    ready_end = STATE["zero_fault"] is None and grippers_clear() and button_out() and bodies_at_rest()
    if ready_end:
        STATE["clear_rest_s"] = round(float(STATE["clear_rest_s"]) + dt, 6)
    else:
        STATE["clear_rest_s"] = 0.0
    EVENTS.note("hands clear and at rest", ready_end, now)


def score(objects) -> float:
    """Whole-trial zero faults: tripod foot > 10 mm off bench, tripod foot > 20 mm off mark, phone touched bench, or a
    locked hinge slipped > 0.2 deg in all (the head forced past LOCK_NM with the button out).
    End state (both grippers >= 30 mm clear, button out / head locked, all bodies at rest for 1.0 s):
    - 1.0: |tilt| <= 3.0 deg and |roll| <= 3.0 deg
    - 0.5: |tilt| <= 8.0 deg and |roll| <= 8.0 deg (at least one > 3.0 deg)
    - 0.0: otherwise."""
    if STATE["zero_fault"] is not None:
        return 0.0
    if float(STATE["clear_rest_s"]) < P.REST_HOLD_S - 1e-9:
        return 0.0
    if not (grippers_clear() and button_out() and bodies_at_rest()):
        return 0.0
    lean = phone_max_lean_deg()
    if lean <= P.LEVEL_DEG + 1e-9:
        return 1.0
    if lean <= P.PARTIAL_DEG + 1e-9:
        return 0.5
    return 0.0


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out: dict = {
        "zero_fault": STATE.get("zero_fault"),
        "events": [f"{t:.2f} s {what}" for t, what in STATE.get("events", [])],
        "order": EVENTS.report(),
        "max_foot_lift_mm": round(float(STATE.get("max_foot_lift_mm", 0.0)), 2),
        "max_foot_slide_mm": round(float(STATE.get("max_foot_slide_mm", 0.0)), 2),
        "clear_rest_s": round(float(STATE.get("clear_rest_s", 0.0)), 2),
        "head": {
            "max_button_mm": round(float(H.STATE.get("max_button_mm", 0.0)), 2),
            "max_button_spring_n": round(float(H.STATE.get("max_button_spring_n", 0.0)), 2),
            "max_right_button_n": round(float(H.STATE.get("max_right_button_n", 0.0)), 2),
            "max_right_head_n": round(float(H.STATE.get("max_right_head_n", 0.0)), 2),
            "max_left_phone_n": round(float(H.STATE.get("max_left_phone_n", 0.0)), 2),
            "max_head_torque_nm": {k: round(float(v), 3) for k, v in H.STATE.get("max_head_torque_nm", {}).items()},
            "slipped": dict(H.STATE.get("slipped", {})),
            "slip_deg": {k: round(float(v), 3) for k, v in H.STATE.get("slip_deg", {}).items()},
            "profile": H.STATE.get("profile", []),
        },
    }
    if mk.MECH.poses:
        t_deg, r_deg = phone_tilt_roll_deg()
        out.update(
            {
                "phone_tilt_deg": round(t_deg, 2),
                "phone_roll_deg": round(r_deg, 2),
                "button_mm": round(1000.0 * H.button_travel_m(), 2),
                "unlocked": H.is_unlocked(),
                "min_gripper_clear_mm": round(1000.0 * min_gripper_clearance_m(), 1),
                "score": score(None),
            }
        )
    return out
