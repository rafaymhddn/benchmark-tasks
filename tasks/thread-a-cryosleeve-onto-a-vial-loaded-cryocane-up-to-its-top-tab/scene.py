"""The bodies of the CryoCane sleeving task (zero write_pose, zero set_joint, no joint between sleeve and cane):
- ``prep_mat``: dark blue silicone bench strip (fixed), alignment ticks at the cane and sleeve start marks;
- ``tray``: open 18/10 stainless transfer tray (free, 420 g) behind the prep mat;
- ``cane``: 290 mm 5-place stepped aluminum Nalgene 5015-0001 CryoCane (free), starting trough-up with its 25 mm
  90-degree bent top ID tab facing the sleeve (-y) and its bullet-tapered straight bottom foot pointing left (+y);
- ``vial_a``, ``vial_b``: two loose 1.8 mL Nunc 377267 round-bottom cryovials (12.5 x 49.0 mm, magenta sample, white
  cap) resting in open upward C-bays 3 and 2 of ``cane``;
- ``sleeve``: 273 mm Nalgene 5016-0001 CryoSleeve, a free 16-sided PVC tube (13.5 mm bore, 14.5 mm outside, drawn
  opaque frosted ivory) in true contact with the cane's strip, clip arms and shelves and with the vials: 0.15 mm a side
  when centred, so the robot keeps it straight; a cocked push wedges its rims and the friction stops it.
"""

from __future__ import annotations

import math

import numpy as np

import mechkit as mk
import parts as P
from mechkit import body, box, capsule, cylinder

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
PVC_ON_METAL = (P.MU_PVC, P.MU_PVC_DYN)  # the cane's trough and the sleeve (combine "min": the lower of a pair wins)
PP_SLIDING = (P.MU_PP, P.MU_PP_DYN)  # the vials

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


def _lip_plate(side: float, span_deg, y0: float, y1: float, lean_in: float = 0.0, core: bool = False):
    """One flat clip-arm plate on ``side`` (+1: +x), tangent to the vial between the two angles of ``span_deg`` (from
    straight down), running y0..y1 along the cane; ``lean_in`` (m) brings its y0 end that far towards the vial's axis.
    ``core``: the plate's CORE_T solver core instead (same outer face, built inward, the vials pass through it)."""
    a0, a1 = (math.radians(a) for a in span_deg)
    am = 0.5 * (a0 + a1)
    half = P.VIAL_R * math.tan(0.5 * (a1 - a0))
    n = np.array([side * math.sin(am), 0.0, -math.cos(am)])  # outward from the vial's axis
    thick = P.CORE_T if core else P.LIP_T
    centre = np.array([0.0, 0.5 * (y0 + y1), P.VIAL_REST_Z]) + (P.VIAL_R + P.LIP_T - 0.5 * thick - 0.5 * lean_in) * n
    theta = math.atan2(side * math.sin(am), -math.cos(am))  # turns the plate's thickness axis (z) onto n
    beta = math.atan2(lean_in, y1 - y0)
    rot = mk.axis_angle((0.0, 1.0, 0.0), theta) @ mk.axis_angle((1.0, 0.0, 0.0), beta)
    return box((2.0 * half, y1 - y0, thick), pos=tuple(centre), quat=_quat(rot), color=ALUMINUM, friction=PVC_ON_METAL,
               friction_combine="min", passes=VIALS if core else ())


def build_cane(top: float) -> None:
    """A flat 2.3 mm strip under the vials, two 0.5 mm clip-arm plates a side hugging them up to the 2.8 mm lip (with
    a 14 mm lead-in at the foot end leaning 2 mm inward), 4 mm stepped shelves between the bays and the 26 mm upturned top tab: every
    collider of the trough lies inside the sleeve's 13.5 mm bore (``test_cane_fits_inside_the_sleeve_bore``)."""
    nose_l = 0.014
    main_sp_l = P.TAB_SHOULDER_Y - nose_l
    nose_pitch = math.radians(-10.0)
    trough = dict(color=ALUMINUM, friction=PVC_ON_METAL, friction_combine="min")
    shapes = [
        box((P.SPINE_W - 0.0003, nose_l, P.SPINE_T), pos=(0.0, 0.5 * nose_l, P.BASE_Z + 0.5 * P.SPINE_T + 0.0012),
            quat=_quat(mk.axis_angle((1.0, 0.0, 0.0), nose_pitch)), **trough),
        box((P.SPINE_W, main_sp_l, P.SPINE_T), pos=(0.0, nose_l + 0.5 * main_sp_l, P.BASE_Z + P.SPINE_T / 2.0), **trough),
        # solver cores: same outer faces as the strip and nose, built inward; the vials pass through them
        box((P.SPINE_W - 0.0003, nose_l, P.CORE_T), pos=(0.0, 0.5 * nose_l, P.BASE_Z + 0.5 * P.CORE_T + 0.0012),
            quat=_quat(mk.axis_angle((1.0, 0.0, 0.0), nose_pitch)), passes=VIALS, **trough),
        box((P.SPINE_W, main_sp_l, P.CORE_T), pos=(0.0, nose_l + 0.5 * main_sp_l, P.BASE_Z + P.CORE_T / 2.0), passes=VIALS, **trough),
    ]
    for s in (-1.0, 1.0):
        for span in P.LIP_SPANS_DEG:
            shapes.append(_lip_plate(s, span, P.SHELF_YS[0], P.LIP_Y0, lean_in=P.LEAD_IN))
            shapes.append(_lip_plate(s, span, P.LIP_Y0, P.SHELF_YS[-1]))
            shapes.append(_lip_plate(s, span, P.SHELF_YS[0], P.LIP_Y0, lean_in=P.LEAD_IN, core=True))
            shapes.append(_lip_plate(s, span, P.LIP_Y0, P.SHELF_YS[-1], core=True))
    for y_s in P.SHELF_YS:
        shapes.append(box((P.SHELF_W, P.SHELF_T, P.SHELF_H), pos=(0.0, y_s, P.FLOOR_Z + P.SHELF_H / 2.0), color=ALUMINUM_STEP,
                          friction=PVC_ON_METAL, friction_combine="min"))
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
            capsule(P.VIAL_R, P.VIAL_STRAIGHT_L, axis="y", color=MAGENTA, friction=PP_SLIDING, friction_combine="min"),
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
                friction=PVC_ON_METAL,
                friction_combine="min",
            )
        )
    for end_s in (-1.0, 1.0):
        for k in range(0, P.SLEEVE_STAVES, 4):
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


def build(top: float) -> None:
    build_prep_mat(top)
    build_tray(top)
    build_sleeve(top)
    build_cane(top)
    build_vials(top)


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
