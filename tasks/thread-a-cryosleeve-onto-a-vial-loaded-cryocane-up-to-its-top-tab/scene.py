"""The bodies and coaxial slide mechanism of the CryoCane sleeving task (zero write_pose, zero set_joint):
- ``prep_mat``: dark blue silicone bench strip (fixed), alignment ticks at the cane and sleeve start marks;
- ``tray``: open 18/10 stainless transfer tray (free, 420 g) behind the prep mat;
- ``cane``: 290 mm 5-place stepped aluminum Nalgene 5015-0001 CryoCane (free), starting trough-up with its 25 mm
  90-degree bent top ID tab facing the sleeve (-y) and its bullet-tapered straight bottom foot pointing left (+y);
- ``vial_a``, ``vial_b``: two loose 1.8 mL Nunc 377267 round-bottom cryovials (12.5 x 49.0 mm, magenta sample, white
  cap) resting in open upward C-bays 3 and 2 of ``cane``;
- ``sleeve``: 273 mm Nalgene 5016-0001 CryoSleeve (8-stave hollow octagonal tube, drawn opaque frosted ivory);
- ``c_slv``, ``slv_s``, ``w_sleeve``: 265 mm prismatic slide along ``cane`` (5.0 N rest-following spring wipe, 35.0 N
  if cocked > 0.35 N m); ``w_sleeve`` engages in-place right where ``sleeve`` stands once telescoped coaxially over the
  straight foot (zero pose snap).
"""

from __future__ import annotations

import math

import numpy as np

import mechkit as mk
import parts as P
from mechkit import body, box, capsule, cylinder, slide, weld

MECHKIT_HELPER = True

MAT_BLUE = (0.17, 0.27, 0.39)
TICK_PALE = (0.82, 0.88, 0.94)
TRAY_WELL = (0.50, 0.53, 0.57)
TRAY_RIM = (0.72, 0.75, 0.79)
ALUMINUM = (0.76, 0.79, 0.84)
ALUMINUM_STEP = (0.62, 0.66, 0.72)
TAB_BLUE = (0.12, 0.34, 0.74)
MAGENTA = (0.88, 0.14, 0.56)
WHITE_CAP = (0.95, 0.95, 0.97)
FROSTED_IVORY = (0.92, 0.93, 0.88)
SLEEVE_BAND = (0.30, 0.52, 0.58)
SMOOTH_TROUGH = (0.15, 0.12)

VIALS = ("vial_a", "vial_b")


def _quat(matrix) -> tuple[float, ...]:
    return tuple(float(v) for v in mk.matrix_to_quat(matrix))


def cane_start_pos(top: float) -> tuple[float, float, float]:
    return (P.MAT_CX, P.CANE_START_FOOT_Y, top + P.MAT_T - P.BASE_Z + P.OVER)


def sleeve_start_pos(top: float) -> tuple[float, float, float]:
    return (P.MAT_CX, P.SLEEVE_START_CY, top + P.MAT_T + P.SLEEVE_R_OUT + P.OVER)


def tray_start_pos(top: float) -> tuple[float, float, float]:
    return (P.TRAY_CX, P.TRAY_CY, top + P.OVER)


def build_prep_mat(top: float) -> None:
    cane_tab_y = P.CANE_START_FOOT_Y - P.CANE_L
    slv_lo_y = P.SLEEVE_START_CY - P.SLEEVE_L / 2.0
    slv_hi_y = P.SLEEVE_START_CY + P.SLEEVE_L / 2.0
    shapes = [box((P.MAT_X, P.MAT_Y, P.MAT_T), pos=(0.0, 0.0, P.MAT_T / 2.0), color=MAT_BLUE)]
    for y_mark in (P.CANE_START_FOOT_Y, cane_tab_y, slv_hi_y, slv_lo_y):
        shapes.append(box((0.026, 0.0015, 0.0005), pos=(0.0, y_mark, P.MAT_T + 0.00025), color=TICK_PALE, solid=False))
    body("prep_mat", shapes, pos=(P.MAT_CX, 0.0, top), color=MAT_BLUE, friction=(0.9, 0.8))


def build_tray(top: float) -> None:
    w = P.TRAY_WALL
    shapes = [
        box((P.TRAY_X, P.TRAY_Y, w), pos=(0.0, 0.0, w / 2.0), color=TRAY_WELL, friction=(0.45, 0.40)),
        *[box((w, P.TRAY_Y, P.TRAY_H), pos=(s * (P.TRAY_X - w) / 2.0, 0.0, P.TRAY_H / 2.0), color=TRAY_RIM) for s in (-1.0, 1.0)],
        *[
            box((P.TRAY_X - 2.0 * w, w, P.TRAY_H), pos=(0.0, s * (P.TRAY_Y - w) / 2.0, P.TRAY_H / 2.0), color=TRAY_RIM)
            for s in (-1.0, 1.0)
        ],
    ]
    body("tray", shapes, pos=tray_start_pos(top), mass=P.TRAY_MASS, color=TRAY_RIM, friction=(0.6, 0.5), contacts=True)


def build_cane(top: float) -> None:
    nose_l = 0.014
    main_sp_l = P.TAB_SHOULDER_Y - nose_l
    nose_pitch = math.radians(-10.0)
    shapes = [
        box(
            (0.0054, nose_l, P.SPINE_T),
            pos=(0.0, 0.5 * nose_l, P.BASE_Z + 0.5 * P.SPINE_T + 0.0012),
            quat=_quat(mk.axis_angle((1.0, 0.0, 0.0), nose_pitch)),
            color=ALUMINUM,
            friction=SMOOTH_TROUGH,
            friction_combine="min",
        ),
        box(
            (P.SPINE_W, main_sp_l, P.SPINE_T),
            pos=(0.0, nose_l + 0.5 * main_sp_l, P.BASE_Z + P.SPINE_T / 2.0),
            color=ALUMINUM,
            friction=SMOOTH_TROUGH,
            friction_combine="min",
        ),
    ]
    rail_y0 = 0.018
    rail_l = P.SHELF_YS[-1] - rail_y0
    rail_cy = 0.5 * (rail_y0 + P.SHELF_YS[-1])
    ca, sa = math.cos(P.RAIL_TILT_RAD), math.sin(P.RAIL_TILT_RAD)
    dx_lip = -0.5 * P.RAIL_T * ca + 0.5 * P.RAIL_W * sa
    dz_lip = 0.5 * P.RAIL_T * sa + 0.5 * P.RAIL_W * ca
    cx = P.RAIL_INNER_HALF_X - dx_lip
    cz = (P.FLOOR_Z + P.RAIL_H) - dz_lip
    lead_l = rail_y0 - P.SHELF_YS[0]
    lead_yaw = math.radians(11.0)
    for s in (-1.0, 1.0):
        q_lead = _quat(mk.quat_to_matrix(mk.yaw_quat(-s * lead_yaw)) @ mk.axis_angle((0.0, 1.0, 0.0), s * P.RAIL_TILT_RAD))
        shapes.append(
            box(
                (P.RAIL_T, lead_l, P.RAIL_W),
                pos=(s * (cx - 0.0014), 0.5 * (P.SHELF_YS[0] + rail_y0), cz + 0.0004),
                quat=q_lead,
                color=ALUMINUM,
                passes=("sleeve",),
                friction=SMOOTH_TROUGH,
                friction_combine="min",
            )
        )
        shapes.append(
            box(
                (P.RAIL_T, rail_l, P.RAIL_W),
                pos=(s * cx, rail_cy, cz),
                quat=_quat(mk.axis_angle((0.0, 1.0, 0.0), s * P.RAIL_TILT_RAD)),
                color=ALUMINUM,
                passes=("sleeve",),
                friction=SMOOTH_TROUGH,
                friction_combine="min",
            )
        )
    for y_s in P.SHELF_YS:
        shapes.append(
            box(
                (P.SHELF_W, P.SHELF_T, P.SHELF_H),
                pos=(0.0, y_s, P.FLOOR_Z + P.SHELF_H / 2.0),
                color=ALUMINUM_STEP,
                friction=SMOOTH_TROUGH,
                friction_combine="min",
            )
        )
    tab_cy = P.TAB_SHOULDER_Y + P.TAB_L / 2.0
    shapes += [
        box(
            (P.TAB_W, P.TAB_L, P.TAB_TOP_Z - P.BASE_Z),
            pos=(0.0, tab_cy, 0.5 * (P.BASE_Z + P.TAB_TOP_Z)),
            color=ALUMINUM,
            friction=mk.GRIPPY,
        ),
        box(
            (P.TAB_W - 0.002, P.TAB_L - 0.003, 0.0008),
            pos=(0.0, tab_cy, P.TAB_TOP_Z + 0.0004),
            color=TAB_BLUE,
            solid=False,
        ),
    ]
    body(
        "cane",
        shapes,
        pos=cane_start_pos(top),
        yaw=P.CANE_START_YAW,
        mass=P.CANE_MASS,
        color=ALUMINUM,
        contact_offset=P.CONTACT_OFF,
        contacts=True,
    )


def build_vials(top: float) -> None:
    cx, foot_y, cz = cane_start_pos(top)
    for name, bay_idx in zip(VIALS, P.VIAL_BAYS):
        vy = foot_y - P.bay_centre_y(bay_idx)
        vz = cz + P.VIAL_REST_Z + 0.0003
        shapes = [
            capsule(P.VIAL_R, P.VIAL_STRAIGHT_L, axis="y", color=MAGENTA, friction=SMOOTH_TROUGH, friction_combine="min"),
            cylinder(P.VIAL_R - 0.0001, 0.0110, pos=(0.0, P.VIAL_L / 2.0 - 0.0060, 0.0), axis="y", color=WHITE_CAP, solid=False),
        ]
        body(
            name,
            shapes,
            pos=(cx, vy, vz),
            yaw=P.CANE_START_YAW,
            mass=P.VIAL_MASS,
            color=MAGENTA,
            linear_damping=0.3,
            angular_damping=0.8,
            contact_offset=P.CONTACT_OFF,
            contacts=True,
        )


def build_sleeve(top: float) -> None:
    r_mid = 0.5 * (P.SLEEVE_R_IN + P.SLEEVE_R_OUT)
    stave_w = 2.0 * P.SLEEVE_R_OUT * math.tan(math.pi / P.SLEEVE_STAVES)
    shapes = []
    for k in range(P.SLEEVE_STAVES):
        a = k * (2.0 * math.pi / P.SLEEVE_STAVES)
        shapes.append(
            box(
                (stave_w, P.SLEEVE_L, P.SLEEVE_WALL),
                pos=(r_mid * math.sin(a), 0.0, r_mid * math.cos(a)),
                quat=_quat(mk.axis_angle((0.0, 1.0, 0.0), a)),
                color=FROSTED_IVORY,
                friction=mk.GRIPPY,
            )
        )
    for end_s in (-1.0, 1.0):
        for k in (0, 2, 4, 6):
            a = k * (2.0 * math.pi / P.SLEEVE_STAVES)
            shapes.append(
                box(
                    (stave_w * 0.96, 0.0030, 0.0004),
                    pos=((P.SLEEVE_R_OUT + 0.0002) * math.sin(a), end_s * (P.SLEEVE_L / 2.0 - 0.0025), (P.SLEEVE_R_OUT + 0.0002) * math.cos(a)),
                    quat=_quat(mk.axis_angle((0.0, 1.0, 0.0), a)),
                    color=SLEEVE_BAND,
                    solid=False,
                )
            )
    body(
        "sleeve",
        shapes,
        pos=sleeve_start_pos(top),
        mass=P.SLEEVE_MASS,
        color=FROSTED_IVORY,
        contact_offset=P.CONTACT_OFF,
        contacts=True,
    )


def build_slide_mechanism(top: float) -> None:
    """Coaxial carriage ``c_slv`` built at station CARRIAGE_BUILT_S = 15 mm along ``cane``: its 2 mm collider sits at
    (0, -SLEEVE_L/2, 0) in ``c_slv``'s frame, which lies inside ``cane``'s hollow shaft at y_c = 15..265 mm for every
    pose in 3D space (requiring zero set_joint or write_pose at reset or engagement)."""
    cx, foot_y, cz = cane_start_pos(top)
    # Cane built at yaw = pi (+y_c along world -y): sleeve centre when mouth is at y_c = CARRIAGE_BUILT_S
    c_pos = (cx, foot_y + (0.5 * P.SLEEVE_L - P.CARRIAGE_BUILT_S), cz)
    passes = ("cane", "vial_a", "vial_b", "sleeve", "prep_mat", "tray")
    body(
        "c_slv",
        [box((0.002, 0.002, 0.002), pos=(0.0, -0.5 * P.SLEEVE_L, 0.0), color=ALUMINUM, passes=passes)],
        pos=c_pos,
        mass=0.002,
        color=ALUMINUM,
    )
    slide(
        "slv_s",
        "c_slv",
        "cane",
        axis=(0.0, -1.0, 0.0),
        limits=(-P.CARRIAGE_BUILT_S - 0.002, P.SLIDE_MAX - P.CARRIAGE_BUILT_S + 0.001),
        spring=P.SLIDE_K,
        rest=0.0,
        damping=P.SLIDE_DAMP,
    )
    weld("w_sleeve", "sleeve", "c_slv", made=False)


def build(top: float) -> None:
    build_prep_mat(top)
    build_tray(top)
    build_sleeve(top)
    build_cane(top)
    build_vials(top)
    build_slide_mechanism(top)


# --- hand-hull helpers used by task.py ------------------------------------------------------------------------------------
_HULLS: dict = {}
_LINKS: dict = {}


def read_hands(sides=("left", "right")) -> dict[str, np.ndarray]:
    from yam_arena import analysis

    if not _HULLS:
        for link in ("gripper", "tip_left", "tip_right"):
            _HULLS[link] = np.vstack([h.vertices for h in analysis.link_hulls().get(link, [])])
    base = mk.MECH.environment.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
    hands = {}
    for side in sides:
        robot = base.scene[f"{side}_robot"]
        points = []
        for link, local in _HULLS.items():
            key = (id(base), side, link)
            if key not in _LINKS:
                found = robot.find_bodies(link)[0]
                if not found:
                    raise RuntimeError(f"the {side} arm has no body {link!r}")
                _LINKS[key] = found[0]
            pose = robot.data.body_link_pose_w.torch[0, _LINKS[key]].detach().cpu().numpy().astype(float)
            points.append(local @ mk.quat_to_matrix(pose[3:7]).T + pose[:3] - origin)
        hands[side] = np.vstack(points)
    return hands


def box_distance(points: np.ndarray, name: str) -> float:
    position, rotation = mk.pose(name)
    low, high = mk.MECH.bodies[name].bounds()
    local = (points - position) @ rotation
    gap = np.maximum(np.maximum(np.asarray(low) - local, local - np.asarray(high)), 0.0)
    return float(np.linalg.norm(gap, axis=1).min())
