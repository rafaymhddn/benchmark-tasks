"""Fit a child's bike helmet tipped back off the forehead: hold it level and tighten its rear dial with the other hand.

Every dimension, mass, hinge/ratchet/band parameter, reaction share, and scoring threshold comes from ``parts.py``.
The pure serial mechanism lives in ``mechanism.py``:
    None (bench) --weld("head_mount")--> head -> hinge("tilt") -> helmet
                                                    ├─ hinge("dial") -> dial
                                                    └─ slide("band") -> band
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechanism as L  # noqa: E402
import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402
from mechkit import body, matrix_to_quat  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.10  # score() flips to 1.0 once the 1.0 s (20-step) rest hold completes
INSTRUCTION = "Fit the helmet properly on the head form for a bike ride without touching its face or moving the head form, and let go."
RUN_LIMIT_STEPS = 900  # 45 s at 20 Hz: about 2.7 times the scripted demo (run h4t2j: 336 steps)
TOP = TABLE_TOP[SCENE]

HEAD_START_POS = P.head_start_pos(TOP)
PIVOT_POS = P.pivot_world(TOP)
R_HELMET_BUILT = P.helmet_rot_at_tilt(P.TILT_START_DEG)
R_DIAL_BUILT = P.dial_rot_at_tilt(P.TILT_START_DEG, P.DIAL_MID_DEG)

# The shapes are authored in the helmet's level frame and the body is built turned by R_HELMET_BUILT, so in the
# helmet's own frame the front edge's underside is (0, BRIM_ARM, BRIM_DZ) at any tilt.
FRONT_EDGE_LOCAL = np.array([0.0, P.BRIM_ARM, P.BRIM_DZ], dtype=float)

body(
    "head",
    P.head_shapes(),
    pos=HEAD_START_POS,
    mass=P.HEAD_TOTAL_MASS,
    friction=(P.HEAD_FRICTION, P.HEAD_FRICTION),
    contact_offset=P.CONTACT_THIN,
    contacts=True,  # the face rule reads the arms' contact points on the face
)

body(
    "helmet",
    P.helmet_shapes(),
    pos=PIVOT_POS,
    quat=matrix_to_quat(R_HELMET_BUILT),
    mass=P.SHELL_MASS,
    contact_offset=P.CONTACT_THIN,
)

body(
    "dial",
    P.dial_shapes(),
    pos=tuple(P.dial_center_world(TOP, P.TILT_START_DEG)),
    quat=matrix_to_quat(R_DIAL_BUILT),
    mass=P.DIAL_MASS,
    inertia=P.SECOND_INERTIA,
    angular_damping=25.0,
    friction=(0.90, 0.85),
    contact_offset=P.CONTACT_THIN,
)

body(
    "band",
    P.band_shapes(),
    pos=tuple(P.band_center_world(TOP, P.TILT_START_DEG, 0.0)),
    quat=matrix_to_quat(R_HELMET_BUILT),
    mass=P.BAND_MASS,
    inertia=P.SECOND_INERTIA,
    friction=(0.0, 0.0),
    friction_combine="min",
    contact_offset=P.CONTACT_THIN,
)

L.declare(TOP)

OBJECTS = list(mk.MECH.bodies.values())

STATE: dict = {}


def begin(seed: int | None = None) -> dict:
    STATE.clear()
    STATE.update(
        seed=seed,
        events=[],
        zero_reason=None,
        max_head_shift_mm=0.0,
        face_touches=0,
        min_gap_mm=1000.0 * P.GAP_START,
        max_gap_mm=1000.0 * P.GAP_START,
        rest_steps=0,
        rest_held_s=0.0,
        last_time=0.0,
    )
    if mk.MECH.environment is None:
        mk.set_state(
            joints={"tilt": 0.0, "dial": -L.DIAL_MID_RAD, "band": 0.0},
            grippers={
                "left": ((0.18, +0.30, TOP + 0.25), 0.094),
                "right": ((0.18, -0.30, TOP + 0.25), 0.094),
            },
            time=0.0,
        )
    return STATE


begin()


@mk.on_reset
def start(seed: int | None = None) -> None:
    begin(seed)
    L.reset()


# --- geometric & physical predicates ---


def head_shift_m() -> float:
    """How far (m) the head form has moved from where it was built (its tape marks)."""
    pos, _ = mk.pose("head")
    return float(np.linalg.norm(pos - np.asarray(HEAD_START_POS, dtype=float)))


def face_touched() -> bool:
    """Whether a link of either arm touches the head form's face (eyes, nose: shapes FACE_SHAPE_INDICES) this step."""
    return any(mk.touching("head", side, shapes=P.FACE_SHAPE_INDICES) for side in ("left robot", "right robot"))


def front_edge_world_pos() -> np.ndarray:
    """Current world (x, y, z) of the helmet front-edge underside center."""
    p_helm, r_helm = mk.pose("helmet")
    return p_helm + r_helm @ FRONT_EDGE_LOCAL


def gap_m() -> float:
    """Height (m) of the helmet front-edge underside above the brow-band center in head's frame."""
    p_head, r_head = mk.pose("head")
    in_head = r_head.T @ (front_edge_world_pos() - p_head)
    return float(in_head[2] - P.BROW_Z)


def _point_to_body_m(point: np.ndarray, name: str) -> float:
    pos, rot = mk.pose(name)
    best = math.inf
    for s in mk.MECH.bodies[name].shapes:
        if not s.solid:
            continue
        sr = rot @ mk.quat_to_matrix(s.quat)
        sc = pos + rot @ np.asarray(s.pos, dtype=float)
        local = sr.T @ (point - sc)
        if s.kind == "box":
            half = 0.5 * np.asarray(s.dims, dtype=float)
            outside = np.maximum(np.abs(local) - half, 0.0)
            inside = min(float(np.max(np.abs(local) - half)), 0.0)
            best = min(best, float(np.linalg.norm(outside)) + inside)
        elif s.kind == "cylinder":
            r, h = float(s.dims[0]), float(s.dims[1])
            dr = max(0.0, math.hypot(float(local[0]), float(local[1])) - r)
            dz = max(0.0, abs(float(local[2])) - 0.5 * h)
            if dr == 0.0 and dz == 0.0:
                best = min(best, -min(r - math.hypot(float(local[0]), float(local[1])), 0.5 * h - abs(float(local[2]))))
            else:
                best = min(best, math.hypot(dr, dz))
        elif s.kind == "capsule":
            r, h = float(s.dims[0]), float(s.dims[1])
            z_seg = max(-0.5 * h, min(0.5 * h, float(local[2])))
            best = min(best, float(np.linalg.norm([local[0], local[1], local[2] - z_seg])) - r)
        elif s.kind == "sphere":
            best = min(best, float(np.linalg.norm(local)) - float(s.dims[0]))
        elif s.kind == "ellipsoid":
            semi = np.asarray(s.dims, dtype=float)
            best = min(best, (float(np.linalg.norm(local / semi)) - 1.0) * float(np.min(semi)))
    return float(best)


def gripper_min_clearance_m() -> float:
    """Minimum distance (m) from either gripper grasp point to head, helmet, dial, or band."""
    targets = ("head", "helmet", "dial", "band")
    best = math.inf
    for side in ("left", "right"):
        if side not in mk.MECH.grippers:
            return 0.0
        gp = np.asarray(mk.gripper(side)[0], dtype=float)
        for t in targets:
            best = min(best, _point_to_body_m(gp, t))
    return float(best)


def grippers_clear() -> bool:
    return gripper_min_clearance_m() >= P.GRIPPER_CLEAR_M - 1e-9


def all_bodies_at_rest() -> bool:
    for name in ("head", "helmet", "dial", "band"):
        lin, ang = mk.velocity(name)
        if float(np.linalg.norm(lin)) >= P.REST_LINEAR_MPS or float(np.linalg.norm(ang)) >= P.REST_ANGULAR_RPS:
            return False
    return True


def gap_in_fit_window() -> bool:
    g = gap_m()
    return bool(P.GAP_MIN - 1e-9 <= g <= P.GAP_MAX + 1e-9)


def dial_was_turned() -> bool:
    return L.dial_deg() >= 0.5 * P.DIAL_CLICK_DEG - 1e-9


def helmet_clamped() -> bool:
    """The simulator's reading of the bench's pull test: the band presses with CLAMP_N or more, and its friction
    hold on the tilt is at least the 10 N pull's moment at the front edge plus the rocking back (PULL_MOMENT_NM)."""
    return bool(L.band_force_n() >= P.CLAMP_N - 1e-9 and L.tilt_hold_nm() >= P.PULL_MOMENT_NM - 1e-9)


def valid_end_posture() -> bool:
    return bool(
        STATE.get("zero_reason") is None
        and head_shift_m() <= P.MAX_HEAD_SHIFT_M + 1e-9
        and gap_in_fit_window()
        and dial_was_turned()
        and grippers_clear()
        and all_bodies_at_rest()
    )


@mk.on_step
def step() -> None:
    L.watch(STATE["events"], TOP)

    now = mk.now()
    dt = max(0.0, now - float(STATE.get("last_time", 0.0)))
    if dt <= 0.0 or dt > 0.25:
        dt = 0.05
    STATE["last_time"] = now

    b_shift = head_shift_m()
    on_face = face_touched()
    g_mm = 1000.0 * gap_m()
    STATE["max_head_shift_mm"] = max(float(STATE.get("max_head_shift_mm", 0.0)), 1000.0 * b_shift)
    STATE["face_touches"] = int(STATE.get("face_touches", 0)) + int(on_face)
    STATE["min_gap_mm"] = min(float(STATE.get("min_gap_mm", g_mm)), g_mm)
    STATE["max_gap_mm"] = max(float(STATE.get("max_gap_mm", g_mm)), g_mm)

    if STATE["zero_reason"] is None:
        if b_shift > P.MAX_HEAD_SHIFT_M:
            STATE["zero_reason"] = f"head form moved {1000 * b_shift:.1f} mm (> {1000 * P.MAX_HEAD_SHIFT_M:.1f} mm)"
        elif on_face:
            STATE["zero_reason"] = "an arm touched the head form's face (eyes, nose)"
        if STATE["zero_reason"] is not None:
            STATE["events"].append((now, f"zero rule triggered: {STATE['zero_reason']}"))

    if valid_end_posture():
        STATE["rest_steps"] = int(STATE.get("rest_steps", 0)) + 1
        STATE["rest_held_s"] = round(float(STATE.get("rest_held_s", 0.0)) + dt, 6)
    else:
        STATE["rest_steps"] = 0
        STATE["rest_held_s"] = 0.0


def score(objects) -> float:
    if STATE.get("zero_reason") is not None:
        return 0.0
    if not valid_end_posture():
        return 0.0
    if int(STATE.get("rest_steps", 0)) < 20 and float(STATE.get("rest_held_s", 0.0)) < P.REST_HOLD_S - 1e-9:
        return 0.0
    if helmet_clamped():
        return 1.0
    return 0.5


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out: dict = {
        "zero_reason": STATE.get("zero_reason"),
        "tilt_deg": round(L.tilt_deg(), 2),
        "gap_mm": round(1000.0 * gap_m(), 2),
        "dial_deg": round(L.dial_deg(), 1),
        "dial_clicks": round(L.dial_clicks(), 2),
        "clicks_past_touch": round(L.clicks_past_touch(), 2),
        "first_touch_dial_deg": L.STATE.get("first_touch_dial_deg"),
        "clamp_dial_deg": L.STATE.get("clamp_dial_deg"),
        "band_slide_mm": round(1000.0 * L.band_slide_m(), 2),
        "band_rest_mm": round(1000.0 * L.band_rest_m(), 2),
        "band_force_n": round(L.band_force_n(), 2),
        "peak_band_n": round(float(L.STATE.get("peak_band_n", 0.0)), 2),
        "tilt_hold_nm": round(L.tilt_hold_nm(), 3),
        "clamped": helmet_clamped(),
        "max_head_shift_mm": round(float(STATE.get("max_head_shift_mm", 0.0)), 2),
        "face_touch_steps": int(STATE.get("face_touches", 0)),
        "gripper_min_clear_mm": round(1000.0 * gripper_min_clearance_m(), 2),
        "rest_steps": int(STATE.get("rest_steps", 0)),
        "rest_held_s": round(float(STATE.get("rest_held_s", 0.0)), 3),
        "events": [f"{t:.2f} s {what}" for t, what in STATE.get("events", [])],
        "force_profile": L.STATE.get("force_profile", []),
    }
    return out
