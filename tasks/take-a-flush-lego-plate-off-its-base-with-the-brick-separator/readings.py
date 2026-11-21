"""Readings of the scene that the rules in ``task.py`` decide on: where the separator's socket sits on the target plate,
the press on the plate, the base's corners, the dish, the grippers' clearance, rest, and the run's report.

Readings only; every threshold and decision is in ``task.py``. Sizes come from ``parts.py``.
"""

from __future__ import annotations

import math

import numpy as np

import mechkit as mk
import parts as P

ROW_IN_TARGET = np.array([0.0, -1.5 * P.PITCH, 0.5 * P.PLATE_T])  # the middle of the near row's two studs, on the top face
BASE_CORNERS = np.array([[sx * 0.5 * P.BASE_W, sy * 0.5 * P.BASE_L, 0.0] for sx, sy in ((-1, -1), (-1, 1), (1, 1), (1, -1))])
REST_BODIES = ("base", "target", "separator", "dish")


def weight_moment_nm() -> float:
    """The simulated plate's weight and its socket chain's about the far edge (N m): what the stop carries with nothing
    on the plate (the gauge's zero before the simulator's own reading replaces it at TARE_AT_S)."""
    return P.GRAVITY * (P.TARGET_MASS_SIM * 0.5 * P.PLATE_L + 3 * P.CARRIER_MASS * P.ROW_LEVER)


def stop_moment_nm() -> float:
    """What the peel hinge's sprung lower stop carries (N m, positive when the plate is pressed down)."""
    return max(0.0, -float(mk.stop_load("peel")))


def socket_offset() -> dict:
    """Where the separator's socket (its frame's origin, the socket face's middle) sits against the near row, in the
    target plate's frame: across (dx) and along (dy) the plate, the face's height over the top face, tilt and yaw."""
    p_target, r_target = mk.pose("target")
    p_sep, r_sep = mk.pose("separator")
    rel = r_target.T @ (p_sep - (p_target + r_target @ ROW_IN_TARGET))
    r_err = (r_target @ P.R_SEP_SEATED).T @ r_sep
    return {
        "dx_mm": 1000.0 * float(rel[0]),
        "dy_mm": 1000.0 * float(rel[1]),
        "horiz_mm": 1000.0 * math.hypot(float(rel[0]), float(rel[1])),
        "face_gap_mm": 1000.0 * abs(float(rel[2])),
        "tilt_deg": math.degrees(math.acos(max(-1.0, min(1.0, float(r_err[2, 2]))))),
        "yaw_deg": abs(math.degrees(math.atan2(float(r_err[1, 0]), float(r_err[0, 0])))),
    }


def base_lift_and_slide_m(rest_z: float, start_corners_xy: np.ndarray) -> tuple[float, float]:
    """(highest corner over its resting height, farthest corner from its mark on the bench), m."""
    pos, rot = mk.pose("base")
    corners = BASE_CORNERS @ rot.T + pos
    lift = float(np.max(corners[:, 2] - rest_z))
    slide = float(np.max(np.linalg.norm(corners[:, :2] - start_corners_xy, axis=1)))
    return max(0.0, lift), slide


def dish_move_m() -> float:
    pos, _ = mk.pose("dish")
    return math.hypot(float(pos[0] - P.DISH_XY[0]), float(pos[1] - P.DISH_XY[1]))


def target_in_dish_rel() -> np.ndarray:
    """The target plate's centre in the dish's frame (origin at the dish's bottom centre), m."""
    pb, _ = mk.pose("target")
    pd, rd = mk.pose("dish")
    return rd.T @ (pb - pd)


def separator_corners_w() -> np.ndarray:
    """The corners of the separator's solid boxes in the world, m."""
    pos, rot = mk.pose("separator")
    pts = []
    for s in mk.MECH.bodies["separator"].shapes:
        if not s.solid:
            continue
        sr = rot @ mk.quat_to_matrix(s.quat)
        sc = pos + rot @ np.asarray(s.pos, dtype=float)
        h = 0.5 * np.asarray(s.dims, dtype=float)
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    pts.append(sc + sr @ (h * np.array([sx, sy, sz])))
    return np.array(pts)


def white_offsets_m() -> list[float]:
    """How far each white plate has come from its place on the base (m), in the base's frame."""
    pb, rb = mk.pose("base")
    out = []
    for name, dx in (("white_f", P.WHITE_FRONT_X_IN_BASE), ("white_b", P.WHITE_BACK_X_IN_BASE)):
        want = np.array([dx, P.TARGET_CENTER_Y_IN_BASE, 0.5 * P.BASE_T + 0.5 * P.PLATE_T])
        out.append(float(np.linalg.norm(rb.T @ (mk.pose(name)[0] - pb) - want)))
    return out


def _point_to_body_m(point: np.ndarray, name: str) -> float:
    pos, rot = mk.pose(name)
    best = math.inf
    for s in mk.MECH.bodies[name].shapes:
        if not s.solid:
            continue
        sr = rot @ mk.quat_to_matrix(s.quat)
        local = sr.T @ (point - (pos + rot @ np.asarray(s.pos, dtype=float)))
        if s.kind == "box":
            best = min(best, float(np.linalg.norm(np.maximum(np.abs(local) - 0.5 * np.asarray(s.dims, dtype=float), 0.0))))
        elif s.kind == "cylinder":
            dr = max(0.0, math.hypot(float(local[0]), float(local[1])) - float(s.dims[0]))
            best = min(best, math.hypot(dr, max(0.0, abs(float(local[2])) - 0.5 * float(s.dims[1]))))
    return best


def gripper_clearance_m() -> float:
    """The smallest distance from either gripper's grasp point to the target plate, the separator or the dish, m."""
    best = math.inf
    for side in ("left", "right"):
        if side not in mk.MECH.grippers:
            return 0.0
        gp = np.asarray(mk.gripper(side)[0], dtype=float)
        for target in ("target", "separator", "dish"):
            best = min(best, _point_to_body_m(gp, target))
    return best


def fastest_body() -> tuple[float, float]:
    """(largest linear speed m/s, largest angular speed rad/s) of the base, plate, separator and dish."""
    lin = max(float(np.linalg.norm(mk.velocity(n)[0])) for n in REST_BODIES)
    ang = max(float(np.linalg.norm(mk.velocity(n)[1])) for n in REST_BODIES)
    return lin, ang


def report(state: dict, level: float | None = None) -> dict:
    """What a run's result keeps: events, peaks, the force-travel logs of peel and pull, the end state."""
    out = {
        k: state.get(k)
        for k in ("ever_seated", "off_with_seat", "zero_reason", "best_press_align", "seat_align", "events", "peel_log", "pull_log", "press_log")
    }
    out.update(
        {
            "target_off_base": mk.is_cut("w_target"),
            "target_out_of_sep": mk.is_cut("w_seat"),
            "tare_stop_nm": round(float(state.get("tare_stop_nm", 0.0)), 5),
            "peak_press_n": round(float(state.get("peak_press_n", 0.0)), 2),
            "peel_max_deg": round(float(state.get("peel_max", 0.0)), 2),
            "pull_max_mm": round(float(state.get("pull_max", 0.0)), 3),
            "max_base_lift_mm": round(float(state.get("max_base_lift_mm", 0.0)), 2),
            "max_base_slide_mm": round(float(state.get("max_base_slide_mm", 0.0)), 2),
            "max_dish_move_mm": round(float(state.get("max_dish_move_mm", 0.0)), 2),
            "rest_held_s": round(float(state.get("rest_held_s", 0.0)), 3),
        }
    )
    if mk.MECH.poses:
        out.update(
            {
                "peel_deg": round(math.degrees(float(mk.joint("peel"))), 2),
                "pull_mm": round(1000.0 * float(mk.joint("pull")), 3),
                "target_in_dish_rel_mm": [round(1000.0 * float(v), 1) for v in target_in_dish_rel()],
                "white_offsets_mm": [round(1000.0 * v, 2) for v in white_offsets_m()],
            }
        )
        if mk.MECH.grippers:
            out["gripper_clear_mm"] = round(1000.0 * gripper_clearance_m(), 1)
    if level is not None:
        out["score"] = level
    return out
