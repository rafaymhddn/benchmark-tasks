"""Geometry helpers, contact-gated rolling resistance, and the episode-start release hook for
``mech/tasks/rolling_bottle/task.py``.
"""

from __future__ import annotations

import math
from pathlib import Path

import mechkit as mk
import numpy as np
import parts as P

AWAY_GRIPPERS = {
    "left": (np.array([0.05, 0.35, P.TOP + 0.35]), 1.0),
    "right": (np.array([0.05, -0.35, P.TOP + 0.35]), 1.0),
}

MARKS_Y: dict[str, float] = {
    "foot": P.DRAIN_FOOT_Y,  # +0.110 m
    "150mm": P.DRAIN_FOOT_Y - 0.150,  # -0.040 m
    "300mm": P.DRAIN_FOOT_Y - 0.300,  # -0.190 m
    "edge": P.WORKTOP_MIN_Y,  # -0.230 m
}


def install_post_settle_release(task_path: Path, state: dict) -> None:
    """Clear ``settle_hold_force`` right after ``YamArenaEmbodiment.reset`` finishes its 0.5 s pre-episode arm settle,
    so the bottle releases from rest at exact ``t = 0.00 s`` before step 1 advances."""
    from yam_arena import inspect_embodiment

    cls = inspect_embodiment.YamArenaEmbodiment
    resolved = task_path.resolve()
    hooks: dict = getattr(cls, "_post_settle_hooks", None) or {}
    hooks[resolved] = state
    cls._post_settle_hooks = hooks
    if getattr(cls, "_post_settle_wrapped", False):
        return
    prev_reset = cls.reset

    def reset(self, scene, *, seed=None):
        obs = prev_reset(self, scene, seed=seed)
        t_obj = getattr(self, "task", None)
        if t_obj is not None:
            st = cls._post_settle_hooks.get(Path(t_obj.path).resolve())
            if st is not None and mk.MECH.environment is not None:
                mk.push("bottle")  # clear start straightedge hold at t = 0.00 s
                mk._read_bodies(mk.MECH.environment)
                p_b, _ = mk.pose("bottle")
                lin_v = mk.velocity("bottle")[0] if "bottle" in mk.MECH.velocities else np.zeros(3)
                st["released"] = True
                st["release_t"] = 0.0
                st["prev_sample"] = (0.0, p_b.copy(), float(np.linalg.norm(lin_v)))
        return obs

    cls.reset = reset
    cls._post_settle_wrapped = True


def bottle_lowest_z(pos: np.ndarray, rot: np.ndarray) -> float:
    """World z (m) of the lowest point on the closed finite bottle cylinder at (pos, rot)."""
    az = min(1.0, abs(float(rot[2, 2])))
    horiz = math.sqrt(max(0.0, 1.0 - az * az))
    return float(pos[2] - (0.5 * P.BOTTLE_H * az + P.BOTTLE_R * horiz))


def bottle_base_center(pos: np.ndarray, rot: np.ndarray) -> np.ndarray:
    """World position (m) of the bottle's base centre (-z end of cylinder; +z is the cap end)."""
    return np.asarray(pos, dtype=float) - 0.5 * P.BOTTLE_H * np.asarray(rot[:, 2], dtype=float)


def point_to_cylinder_dist(point: np.ndarray, center: np.ndarray, axis_unit: np.ndarray, radius: float, half_len: float) -> float:
    """Euclidean distance (m) from ``point`` to a solid right circular cylinder."""
    rel = np.asarray(point, dtype=float) - np.asarray(center, dtype=float)
    a = np.asarray(axis_unit, dtype=float)
    axial = abs(float(np.dot(rel, a)))
    radial_vec = rel - float(np.dot(rel, a)) * a
    radial = float(np.linalg.norm(radial_vec))
    return math.hypot(max(0.0, axial - half_len), max(0.0, radial - radius))


def gripper_distances() -> dict[str, dict[str, float]]:
    """Surface distances (m) from left and right gripper grasp points to the bottle and the coaster."""
    p_b, r_b = mk.pose("bottle")
    p_c, r_c = mk.pose("coaster")
    c_mid = p_c + 0.5 * P.COASTER_TOTAL_H * r_c[:, 2]
    out: dict[str, dict[str, float]] = {}
    for side in ("left", "right"):
        g_pt = mk.gripper(side)[0] if side in mk.MECH.grippers else AWAY_GRIPPERS[side][0]
        d_b = point_to_cylinder_dist(g_pt, p_b, r_b[:, 2], P.BOTTLE_R, 0.5 * P.BOTTLE_H)
        d_c = point_to_cylinder_dist(g_pt, c_mid, r_c[:, 2], P.COASTER_OUT_R, 0.5 * P.COASTER_TOTAL_H)
        out[side] = {"bottle_m": d_b, "coaster_m": d_c}
    return out


def grippers_clear() -> bool:
    """Both grippers are at least 30 mm clear of bottle and coaster and make zero contact with the bottle."""
    if mk.touching("bottle", "left robot") or mk.touching("bottle", "right robot"):
        return False
    dists = gripper_distances()
    return all(d["bottle_m"] >= P.GRIPPER_CLEAR_M and d["coaster_m"] >= P.GRIPPER_CLEAR_M for d in dists.values())


def support_contact_force_n() -> float:
    """Normal contact force (N) of the bottle against the clamped worktop + sloped drainboard."""
    return float(mk.contact_force("bottle", "worktop") + mk.contact_force("bottle", "drainboard"))


def total_counter_contact_force_n() -> float:
    """Normal contact force (N) of the bottle against worktop + drainboard + coaster."""
    return support_contact_force_n() + float(mk.contact_force("bottle", "coaster"))


def compute_roll_torque() -> np.ndarray:
    """Contact-gated opposing rolling-resistance torque (N m, world) on ``bottle`` while rolling on counter."""
    if support_contact_force_n() < P.ROLL_CONTACT_MIN_N:
        return np.zeros(3, dtype=float)
    _, r_b = mk.pose("bottle")
    axis = np.asarray(r_b[:, 2], dtype=float)
    if abs(float(axis[2])) > math.sin(math.radians(30.0)):
        return np.zeros(3, dtype=float)
    _, ang_v = mk.velocity("bottle") if "bottle" in mk.MECH.velocities else (np.zeros(3), np.zeros(3))
    omega_ax = float(np.dot(ang_v, axis))
    if abs(omega_ax) * P.BOTTLE_R <= P.ROLL_SPEED_MIN_MPS:
        return np.zeros(3, dtype=float)
    izz = P.bottle_inertia()[2]
    tau_mag = min(P.tau_roll(), izz * abs(omega_ax) / P.DT)
    return -math.copysign(tau_mag, omega_ax) * axis


def horizontal_bottle_coaster_gap_m(p_b: np.ndarray, r_b: np.ndarray, p_c: np.ndarray) -> float:
    """Horizontal clearance (m) in (x, y) between the oriented bottle cylinder projection and the cork coaster disc."""
    ax_xy = np.array([float(r_b[0, 2]), float(r_b[1, 2])], dtype=float)
    ax_norm = float(np.linalg.norm(ax_xy))
    rel_xy = np.array([float(p_c[0] - p_b[0]), float(p_c[1] - p_b[1])], dtype=float)
    if ax_norm > 1e-6:
        u_xy = ax_xy / ax_norm
        half_along = 0.5 * P.BOTTLE_H * ax_norm + P.BOTTLE_R * math.sqrt(max(0.0, 1.0 - ax_norm * ax_norm))
        along_d = max(0.0, abs(float(np.dot(rel_xy, u_xy))) - half_along)
        perp_d = max(0.0, abs(float(rel_xy[0] * (-u_xy[1]) + rel_xy[1] * u_xy[0])) - P.BOTTLE_R)
        return math.hypot(along_d, perp_d) - P.COASTER_OUT_R
    return float(np.linalg.norm(rel_xy)) - P.BOTTLE_R - P.COASTER_OUT_R


def off_edge_reason(pos: np.ndarray, rot: np.ndarray) -> str | None:
    """Reason string if bottle has crossed outside the counter footprint or dropped below TOP + 25 mm, else None."""
    x, y = float(pos[0]), float(pos[1])
    if x < P.WORKTOP_MIN_X:
        return f"bottle centre passed front edge x = {x:.3f} m < {P.WORKTOP_MIN_X:.3f} m"
    if x > P.WORKTOP_MAX_X:
        return f"bottle centre passed back edge x = {x:.3f} m > {P.WORKTOP_MAX_X:.3f} m"
    if y < P.WORKTOP_MIN_Y:
        return f"bottle centre passed open right cliff y = {y:.3f} m < {P.WORKTOP_MIN_Y:.3f} m"
    if y > P.WORKTOP_MAX_Y:
        return f"bottle centre passed left drainboard edge y = {y:.3f} m > {P.WORKTOP_MAX_Y:.3f} m"
    z_low = bottle_lowest_z(pos, rot)
    if z_low < P.TOP + P.OFF_EDGE_LOW_Z:
        return f"bottle lowest point dropped to {z_low - P.TOP:.4f} m above bench (< {P.OFF_EDGE_LOW_Z:.3f} m)"
    return None
