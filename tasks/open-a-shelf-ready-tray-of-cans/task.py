"""Open a shelf-ready tray of cans: peel its perforated front panel outward and downward from a top corner so the
perforation unzips progressively and tears cleanly, then lay the detached panel flat on the bench clear of the tray
while all cans stay upright inside the tray on its marks.

All product constants and geometry helpers come from ``parts.py``; the bilateral serial perforation mechanism
(``tray_base -> c_lift -> c_pitch -> c_twist --weld(panel_detach)--> front_panel``) lives in ``peel.py``.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402
import peel as L  # noqa: E402
from mechkit import body  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 3.5  # lets both arms finish parking at READY for the final frame after 1.0 is earned
INSTRUCTION = (
    "Tear the perforated front panel completely off the tray of cans, lay the torn panel flat on the bench "
    "clear of the tray, and leave all six cans standing upright inside the tray on its marks."
)
RUN_LIMIT_STEPS = 1600  # 80 s at 20 Hz
TOP = TABLE_TOP[SCENE]

CAN_NAMES: tuple[str, ...] = tuple(f"can_{i}" for i in range(P.N_CANS))
ALL_PASS_NAMES: tuple[str, ...] = ("tray_base", "front_panel", *P.CARRIER_NAMES, *CAN_NAMES)

TRAY_START_POS = (P.TRAY_XY[0], P.TRAY_XY[1], TOP + P.OVER)
START_CORNERS_XY = P.tray_corners_xy(P.TRAY_XY, yaw=0.0)
HINGE_ANCHOR = P.hinge_anchor_world(TOP)
PANEL_START_POS = P.panel_built_pos(TOP)

body("bench_marks", P.bench_mark_shapes(), pos=(0.0, 0.0, TOP), mass=None)
body(
    "tray_base",
    P.tray_base_shapes(),
    pos=TRAY_START_POS,
    mass=P.TRAY_BODY_MASS,
    friction=(P.TRAY_FRICTION, P.TRAY_FRICTION),
    friction_combine="min",
    linear_damping=0.5,
    angular_damping=1.5,
    contact_offset=P.CONTACT_THIN,
    contacts=True,
)
for _cname in P.CARRIER_NAMES:
    body(_cname, P.carrier_shapes(ALL_PASS_NAMES), pos=HINGE_ANCHOR, mass=P.CARRIER_MASS, inertia=P.CARRIER_INERTIA, contact_offset=P.CONTACT_THIN)
body(
    "front_panel",
    P.front_panel_shapes(),
    pos=PANEL_START_POS,
    mass=P.PANEL_MASS,
    inertia=(2.5e-4, 2.5e-4, 2.5e-4),
    linear_damping=3.0,
    angular_damping=15.0,
    contact_offset=P.CONTACT_THIN,
    contacts=True,
)
for _idx, (_cx, _cy) in enumerate(P.can_local_xy()):
    body(
        CAN_NAMES[_idx],
        P.can_shapes(),
        pos=(P.TRAY_XY[0] + _cx, P.TRAY_XY[1] + _cy, TOP + P.OVER + P.BOARD_T + 0.5 * P.CAN_H + 0.0005),
        mass=P.CAN_MASS,
        friction=(P.CAN_FRICTION, P.CAN_FRICTION),
        friction_combine="average",
        linear_damping=0.8,
        angular_damping=2.5,
        contact_offset=P.CONTACT_THIN,
    )

L.declare(TOP)
mk.sense_contact("front_panel", "tray_base")
OBJECTS = list(mk.MECH.bodies.values())

STATE: dict = {}


def _hide_carriers() -> None:
    for cname in P.CARRIER_NAMES:
        for idx in range(len(mk.MECH.bodies[cname].shapes)):
            mk.show(cname, idx, False)


def begin(seed: int | None = None) -> dict:
    STATE.clear()
    STATE.update(
        seed=seed,
        events=[],
        zero_reason=None,
        max_tray_tilt_deg=0.0,
        max_tray_lift_mm=0.0,
        max_tray_shift_mm=0.0,
        max_can_tip_deg=0.0,
        rest_steps=0,
        rest_held_s=0.0,
        last_time=0.0,
        prev_poses={},
        step_speeds={},
    )
    _hide_carriers()
    return STATE


begin()


@mk.on_reset
def start(seed: int | None = None) -> None:
    begin(seed)
    L.reset()


def tray_tilt_deg() -> float:
    _, rot = mk.pose("tray_base")
    return math.degrees(math.acos(max(-1.0, min(1.0, float(rot[2, 2])))))


def tray_lift_m() -> float:
    return float(mk.pose("tray_base")[0][2] - TOP)


def tray_corner_shifts_m() -> np.ndarray:
    pos, rot = mk.pose("tray_base")
    hx, hy = 0.5 * P.TRAY_D, 0.5 * P.TRAY_W
    local = np.array([[-hx, -hy, 0.0], [-hx, +hy, 0.0], [+hx, +hy, 0.0], [+hx, -hy, 0.0]], dtype=float)
    return np.linalg.norm((local @ rot.T + pos)[:, :2] - START_CORNERS_XY, axis=1)


def tray_max_shift_m() -> float:
    return float(np.max(tray_corner_shifts_m()))


def can_tip_deg(name: str) -> float:
    _, rot = mk.pose(name)
    return math.degrees(math.acos(max(-1.0, min(1.0, float(rot[2, 2])))))


def can_inside_tray(name: str) -> bool:
    tp, tr = mk.pose("tray_base")
    rel = tr.T @ (mk.pose(name)[0] - tp)
    return bool(abs(float(rel[0])) <= 0.5 * P.TRAY_D and abs(float(rel[1])) <= 0.5 * P.TRAY_W and 0.0 <= float(rel[2]) <= P.TRAY_H)


def all_cans_upright_in_tray(max_deg: float = P.MAX_CAN_TIP_END_DEG) -> bool:
    return all(can_inside_tray(c) and can_tip_deg(c) <= max_deg + 1e-9 for c in CAN_NAMES)


def panel_tilt_from_horizontal_deg() -> float:
    """Angle (deg) of front_panel's flat sheet from horizontal (0 deg when lying flat either face-up or face-down)."""
    _, rot = mk.pose("front_panel")
    return math.degrees(math.acos(min(1.0, abs(float(rot[2, 0])))))


def panel_flat_on_bench() -> bool:
    pos, _ = mk.pose("front_panel")
    return bool(panel_tilt_from_horizontal_deg() <= P.PANEL_FLAT_TILT_DEG + 1e-9 and TOP - 0.005 <= float(pos[2]) <= TOP + P.PANEL_FLAT_MAX_Z_M + 1e-9)


def _body_footprint_2d(name: str) -> np.ndarray:
    pos, rot = mk.pose(name)
    verts = []
    for s in mk.MECH.bodies[name].shapes:
        if not s.solid:
            continue
        sr = rot @ mk.quat_to_matrix(s.quat)
        sc = pos + rot @ np.asarray(s.pos, dtype=float)
        hx, hy, hz = 0.5 * np.asarray(s.dims, dtype=float)
        for sx in (-hx, hx):
            for sy in (-hy, hy):
                for sz in (-hz, hz):
                    verts.append((sc + sr @ np.array([sx, sy, sz]))[:2])
    return P.convex_hull_2d(np.asarray(verts, dtype=float))


def panel_tray_footprint_gap_m() -> float:
    return P.polygon_gap_2d(_body_footprint_2d("tray_base"), _body_footprint_2d("front_panel"))


def gripper_min_clearance_m() -> float:
    targets = ("tray_base", "front_panel", *CAN_NAMES)
    best = math.inf
    for side in ("left", "right"):
        if side not in mk.MECH.grippers:
            return 0.0
        gp = np.asarray(mk.gripper(side)[0], dtype=float)
        for t in targets:
            pos, rot = mk.pose(t)
            best = min(best, P.point_to_shapes_m(gp, pos, rot, mk.MECH.bodies[t].shapes, mk.quat_to_matrix))
    return float(best)


def grippers_clear() -> bool:
    return gripper_min_clearance_m() >= P.GRIPPER_CLEAR_M - 1e-9


def body_speeds(name: str) -> tuple[float, float]:
    """(linear m/s, angular rad/s) over the 20 Hz control step in live sim, or from mk.velocity in offline tests."""
    if mk.MECH.environment is not None and name in STATE.get("step_speeds", {}):
        return STATE["step_speeds"][name]
    lin, ang = mk.velocity(name)
    return float(np.linalg.norm(lin)), float(np.linalg.norm(ang))


def all_bodies_at_rest() -> bool:
    return all(
        body_speeds(n)[0] < P.REST_LINEAR_MPS and body_speeds(n)[1] < P.REST_ANGULAR_RPS
        for n in ("tray_base", "front_panel", *CAN_NAMES)
    )


def full_end_geometry_ok() -> bool:
    return bool(
        L.detached()
        and L.ties_torn() >= P.N_TIES
        and panel_flat_on_bench()
        and panel_tray_footprint_gap_m() >= P.PANEL_CLEAR_TRAY_M - 1e-9
        and mk.contact_force("front_panel", "tray_base") <= 1e-4
        and all_cans_upright_in_tray(P.MAX_CAN_TIP_END_DEG)
        and tray_max_shift_m() <= P.MAX_TRAY_SHIFT_M + 1e-9
        and tray_tilt_deg() <= P.MAX_TRAY_TILT_DEG + 1e-9
        and tray_lift_m() <= P.MAX_TRAY_LIFT_M + 1e-9
        and grippers_clear()
    )


@mk.on_step
def step() -> None:
    L.watch(STATE["events"])
    now = mk.now()
    dt = max(0.0, now - float(STATE.get("last_time", 0.0)))
    if dt <= 0.0 or dt > 0.25:
        dt = 0.05
    STATE["last_time"] = now

    if mk.MECH.environment is not None:
        prev, speeds = STATE["prev_poses"], {}
        for bname in ("tray_base", "front_panel", *CAN_NAMES):
            p_now, r_now = mk.pose(bname)
            if bname in prev:
                p_prev, r_prev = prev[bname]
                cos_a = max(-1.0, min(1.0, 0.5 * (float(np.trace(r_now @ r_prev.T)) - 1.0)))
                speeds[bname] = (float(np.linalg.norm(p_now - p_prev)) / dt, math.acos(cos_a) / dt)
            prev[bname] = (np.array(p_now, dtype=float), np.array(r_now, dtype=float))
        STATE["step_speeds"] = speeds

    tilt, lift, shift = tray_tilt_deg(), tray_lift_m(), tray_max_shift_m()
    STATE["max_tray_tilt_deg"] = max(STATE["max_tray_tilt_deg"], tilt)
    STATE["max_tray_lift_mm"] = max(STATE["max_tray_lift_mm"], 1000.0 * lift)
    STATE["max_tray_shift_mm"] = max(STATE["max_tray_shift_mm"], 1000.0 * shift)

    if STATE["zero_reason"] is None:
        if tilt > P.MAX_TRAY_TILT_DEG:
            STATE["zero_reason"] = f"tray tilted {tilt:.2f} deg (> {P.MAX_TRAY_TILT_DEG:.1f} deg)"
        elif lift > P.MAX_TRAY_LIFT_M:
            STATE["zero_reason"] = f"tray lifted {1000 * lift:.1f} mm (> {1000 * P.MAX_TRAY_LIFT_M:.1f} mm)"
        elif shift > P.MAX_TRAY_SHIFT_M:
            STATE["zero_reason"] = f"tray corner shifted {1000 * shift:.1f} mm (> {1000 * P.MAX_TRAY_SHIFT_M:.1f} mm)"
        else:
            for cname in CAN_NAMES:
                ctip = can_tip_deg(cname)
                STATE["max_can_tip_deg"] = max(STATE["max_can_tip_deg"], ctip)
                if ctip > P.MAX_CAN_TIP_TRIAL_DEG:
                    STATE["zero_reason"] = f"{cname} tipped {ctip:.2f} deg (> {P.MAX_CAN_TIP_TRIAL_DEG:.1f} deg)"
                    break
                if not can_inside_tray(cname):
                    STATE["zero_reason"] = f"{cname} left tray_base"
                    break
        if STATE["zero_reason"] is not None:
            STATE["events"].append((now, f"zero rule triggered: {STATE['zero_reason']}"))

    for cname in CAN_NAMES:
        STATE["max_can_tip_deg"] = max(STATE["max_can_tip_deg"], can_tip_deg(cname))

    if STATE["zero_reason"] is None and full_end_geometry_ok() and all_bodies_at_rest():
        STATE["rest_steps"] += 1
        STATE["rest_held_s"] = round(STATE["rest_held_s"] + dt, 6)
    else:
        STATE["rest_steps"] = 0
        STATE["rest_held_s"] = 0.0


def score(_objects) -> float:
    """0.0 after a whole-trial zero rule, or with a can not upright in the tray or the tray off its marks. 1.0 when the panel
    is completely detached and every end rule has held for REST_HOLD_S. 0.5 when at least half of the 422 mm perforation
    is torn (for example both side seams), or the panel is off but misses a 1.0 rule; 0.0 below half. The simulator
    tracks the tear in 10 equal stages (half = PARTIAL_TIES_MIN = 5)."""
    if STATE.get("zero_reason") is not None:
        return 0.0
    if not all_cans_upright_in_tray(P.MAX_CAN_TIP_END_DEG):
        return 0.0
    if tray_max_shift_m() > P.MAX_TRAY_SHIFT_M + 1e-9:
        return 0.0
    if L.ties_torn() < P.PARTIAL_TIES_MIN:
        return 0.0
    if full_end_geometry_ok() and all_bodies_at_rest() and STATE.get("rest_held_s", 0.0) >= P.REST_HOLD_S - 1e-9:
        return 1.0
    return 0.5


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out: dict = {
        "preset": P.TRAY_PRESET,
        "grid": P.CAN_GRID_PRESET,
        "ties_torn": L.ties_torn(),
        "detached": L.detached(),
        "detached_at_s": L.STATE.get("detached_at"),
        "detach_cause": L.STATE.get("detach_cause"),
        "zero_reason": STATE.get("zero_reason"),
        "max_tray_tilt_deg": round(float(STATE.get("max_tray_tilt_deg", 0.0)), 2),
        "max_tray_lift_mm": round(float(STATE.get("max_tray_lift_mm", 0.0)), 2),
        "max_tray_shift_mm": round(float(STATE.get("max_tray_shift_mm", 0.0)), 2),
        "max_can_tip_deg": round(float(STATE.get("max_can_tip_deg", 0.0)), 2),
        "max_skew_deg": round(float(L.STATE.get("max_skew_deg", 0.0)), 2),
        "max_pitch_deg": round(float(L.STATE.get("max_pitch_deg", 0.0)), 2),
        "peak_lift_n": round(float(L.STATE.get("peak_lift_n", 0.0)), 2),
        "peak_peel_n": round(float(L.STATE.get("peak_peel_n", 0.0)), 2),
        "rest_pitch_deg": round(math.degrees(float(L.STATE.get("rest", 0.0))), 2),
        "rest_held_s": round(float(STATE.get("rest_held_s", 0.0)), 3),
        "events": [f"{t:.2f} s {what}" for t, what in STATE.get("events", [])],
        "force_profile": L.STATE.get("force_profile", []),
    }
    if mk.MECH.poses:
        max_lin = max(body_speeds(n)[0] for n in ("tray_base", "front_panel", *CAN_NAMES))
        max_ang = max(body_speeds(n)[1] for n in ("tray_base", "front_panel", *CAN_NAMES))
        out.update(
            {
                "panel_tilt_deg": round(panel_tilt_from_horizontal_deg(), 2),
                "panel_flat": panel_flat_on_bench(),
                "panel_tray_gap_mm": round(1000.0 * panel_tray_footprint_gap_m(), 2),
                "gripper_min_clear_mm": round(1000.0 * gripper_min_clearance_m(), 2),
                "max_body_lin_mps": round(max_lin, 5),
                "max_body_ang_rps": round(max_ang, 5),
                "full_geom_ok": full_end_geometry_ok(),
                "all_at_rest": all_bodies_at_rest(),
            }
        )
    return out
