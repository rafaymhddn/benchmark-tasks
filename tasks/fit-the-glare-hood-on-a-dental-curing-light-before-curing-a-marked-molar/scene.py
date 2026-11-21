"""Bodies, joints, and contact sensors of the dental curing-light hood task, built from ``parts.py``.

Serial mechanisms (no closed loops, no welded hinges):
- ``wand`` (free, 145 g) -> ``slide("seat_s")`` -> ``c_elbow`` -> ``hinge("bend_h")`` -> ``c_tip``;
  ``weld("w_hood_on", "hood", "c_tip", made=False)`` engages only on coaxial distal-tip entry and carries ``hood``
  around the 55 deg circular bend and 22 mm straight base against the 6.0 N rubber-grommet spring.
- ``wand`` -> ``slide("pwr_btn")`` -> ``btn_cap`` (2.5 N tactile switch on top of the handpiece).
- Fixed bench fixtures: ``cradle`` (open U-cradle), ``hood_slot`` (upright tray slot), ``jaw`` (1:1 28-tooth typodont
  with hinged upper arch propped open 45 mm and blue-marked lower right first molar #46).
"""

from __future__ import annotations

import math

import numpy as np

import mechkit as mk
import parts as P
from mechkit import body, box, capsule, cylinder, hinge, slide, sphere, weld

MECHKIT_HELPER = True

# sRGB palette chosen for high brightness separation at 320 x 176 (Check 28)
MAT_DARK = (0.16, 0.18, 0.21)
CRADLE_DARK = (0.24, 0.26, 0.30)
WAND_SILVER = (0.86, 0.88, 0.91)
WAND_ACCENT = (0.20, 0.24, 0.30)
GLASS_ROD = (0.72, 0.82, 0.90)
BTN_BLUE = (0.12, 0.45, 0.88)
BEAM_CYAN = (0.10, 0.72, 1.00)
HOOD_ORANGE = (0.96, 0.34, 0.06)
GROMMET_RUBBER = (0.15, 0.15, 0.16)
GUM_PINK = (0.86, 0.48, 0.54)
TOOTH_WHITE = (0.95, 0.94, 0.90)
MOLAR_BLUE = (0.04, 0.30, 0.98)
MOLAR_CURED = (0.93, 0.90, 0.78)
STRUT_GREY = (0.35, 0.37, 0.40)

# Shape indices toggled at runtime via mk.show(...)
WAND_BEAM_SHAPE = 0  # set in build_wand
JAW_UNCURED_SHAPE = 0  # set in build_jaw
JAW_CURED_SHAPE = 0  # set in build_jaw


def _quat(rot: np.ndarray) -> tuple[float, ...]:
    return tuple(float(v) for v in mk.matrix_to_quat(rot))


def _flt(vec) -> tuple[float, ...]:
    return tuple(float(v) for v in vec)


def build_fixtures(top: float) -> None:
    """Dark clinical mat (visual contrast), open U-cradle for the wand, and slotted stand for the orange hood."""
    # Non-colliding dark work mat so silver wand, orange hood, and white/pink jaw stand out sharply at 320 x 176
    body(
        "mat_vis",
        [box((0.36, 0.50, 0.0008), pos=(0.0, 0.0, 0.0004), color=MAT_DARK, solid=False)],
        pos=(0.275, 0.020, top),
        kinematic=True,
        color=MAT_DARK,
    )

    # Open U-cradle: two low smooth saddles 96 mm apart along y so the middle 80 mm of the wand barrel is open to grasp,
    # and both ends (+y, -y) are open so an unheld wand slides out under > 0.5 N axial push.
    barrel_r = P.WAND_W / 2.0  # 0.0155 m
    saddle_top = P.WAND_START_Z - barrel_r  # 0.0525 m
    cradle_shapes = [
        box((P.CRADLE_W_X, P.CRADLE_SPAN_Y + 0.024, 0.010), pos=(0.0, 0.0, 0.005), color=CRADLE_DARK),
    ]
    for sy in (-1.0, 1.0):
        y_s = sy * (P.CRADLE_SPAN_Y / 2.0)
        # Saddle post up to barrel underside
        cradle_shapes.append(
            box(
                (P.CRADLE_W_X, 0.014, saddle_top - 0.010),
                pos=(0.0, y_s, 0.010 + (saddle_top - 0.010) / 2.0),
                color=CRADLE_DARK,
                friction=(P.CRADLE_FRICTION, P.CRADLE_FRICTION),
                friction_combine="min",
            )
        )
        # Side cheeks (leave 2 mm lateral play each side of the 31 mm barrel; open along y)
        for sx in (-1.0, 1.0):
            cradle_shapes.append(
                box(
                    (0.008, 0.014, 0.013),
                    pos=(sx * (barrel_r + 0.006), y_s, saddle_top + 0.0055),
                    color=CRADLE_DARK,
                    friction=(P.CRADLE_FRICTION, P.CRADLE_FRICTION),
                    friction_combine="min",
                )
            )
    body(
        "cradle",
        cradle_shapes,
        pos=(P.WAND_START_XY[0], P.WAND_START_XY[1], top),
        kinematic=True,
        color=CRADLE_DARK,
        friction=(P.CRADLE_FRICTION, P.CRADLE_FRICTION),
        friction_combine="min",
    )

    # Slotted hood tray block at (0.245, +0.180): bottom of the 72 mm vertical hood sits at z = 0.012 m inside a 13 mm
    # slot, leaving the upper 59 mm of the 3.0 mm wing exposed in clear air.
    hood_bottom = P.HOOD_START_Z - P.HOOD_W / 2.0  # 0.048 - 0.036 = 0.012 m
    slot_shapes = [
        box((0.052, 0.036, hood_bottom), pos=(0.0, 0.0, hood_bottom / 2.0), color=CRADLE_DARK),
        box((0.052, 0.009, 0.013), pos=(0.0, -0.0068, hood_bottom + 0.0065), color=CRADLE_DARK, friction=(0.2, 0.2), friction_combine="min"),
        box((0.052, 0.009, 0.013), pos=(0.0, 0.0068, hood_bottom + 0.0065), color=CRADLE_DARK, friction=(0.2, 0.2), friction_combine="min"),
    ]
    body(
        "hood_slot",
        slot_shapes,
        pos=(P.HOOD_START_XY[0], P.HOOD_START_XY[1], top),
        kinematic=True,
        color=CRADLE_DARK,
        friction=(0.2, 0.2),
        friction_combine="min",
    )


def build_jaw(top: float) -> None:
    """1:1 Adult 28-tooth demonstration typodont jaw model (95 x 65 x 50 mm closed) with its hinged upper arch propped
    open 45 mm at the incisors and a bright cobalt-blue uncured composite mark on lower right first molar #46."""
    global JAW_UNCURED_SHAPE, JAW_CURED_SHAPE

    shapes = [
        # Lower mandibular gum arch (horseshoe: left arm, right arm, anterior symphysis, low base plate)
        box((P.JAW_D, P.JAW_W, 0.006), pos=(0.0, 0.0, 0.003), color=GUM_PINK),
        box((0.068, 0.016, 0.018), pos=(0.006, 0.021, 0.009), color=GUM_PINK),   # lower right alveolar ridge (+y)
        box((0.068, 0.016, 0.018), pos=(0.006, -0.021, 0.009), color=GUM_PINK),  # lower left alveolar ridge (-y)
        box((0.018, 0.046, 0.018), pos=(-0.034, 0.0, 0.009), color=GUM_PINK),    # lower anterior ridge (-x)
        # Lower tooth crowns (white) except #46 which is built separately with its filling mark
        box((0.014, 0.036, 0.007), pos=(-0.035, 0.0, 0.0215), color=TOOTH_WHITE),  # lower incisors/canines
        box((0.016, 0.010, 0.007), pos=(-0.023, 0.021, 0.0215), color=TOOTH_WHITE),  # lower right premolars
        box((0.012, 0.011, 0.007), pos=(0.002, 0.021, 0.0215), color=TOOTH_WHITE),   # lower right second molar #47
        box((0.044, 0.010, 0.007), pos=(-0.008, -0.021, 0.0215), color=TOOTH_WHITE),  # lower left posterior arch
        # Target tooth: lower right first molar #46 (11.0 x 10.5 mm white crown, occlusal top at z = 0.025 m)
        box(
            (P.MOLAR_CROWN, 0.0105, 0.007),
            pos=(float(P.MOLAR_LOCAL[0]), float(P.MOLAR_LOCAL[1]), P.LOWER_ARCH_H - 0.0035),
            color=TOOTH_WHITE,
        ),
        # Rear condylar hinge posts (at x = +0.041, y = +/- 0.025) propping the upper arch open
        box((0.010, 0.008, 0.046), pos=(0.041, 0.025, 0.023), color=STRUT_GREY),
        box((0.010, 0.008, 0.046), pos=(0.041, -0.025, 0.023), color=STRUT_GREY),
    ]

    # Hinged upper maxillary arch propped open ~28 deg about the rear condyle (x = +0.041, z = 0.042) so the upper
    # incisors stand 45.0 mm above the lower incisors (z = 0.025 + 0.045 = 0.070 m), overhanging the posterior molars.
    open_rad = math.atan2(P.MOUTH_OPEN_M + P.LOWER_ARCH_H + 0.006 - 0.042, 0.076)  # ~25.3 deg upward toward -x
    q_up = _quat(P._ry(math.degrees(open_rad)))  # +ry lifts -x upward (+z)
    piv = np.array([0.041, 0.0, 0.042])
    r_up = P._ry(math.degrees(open_rad))
    for sz, p_rel, col in (
        ((0.064, 0.015, 0.013), (-0.034, 0.021, 0.008), GUM_PINK),     # upper right gum
        ((0.064, 0.015, 0.013), (-0.034, -0.021, 0.008), GUM_PINK),    # upper left gum
        ((0.018, 0.046, 0.013), (-0.070, 0.0, 0.008), GUM_PINK),       # upper anterior gum
        ((0.056, 0.052, 0.006), (-0.040, 0.0, 0.014), GUM_PINK),       # hard palate roof (blocks straight vertical access!)
        ((0.014, 0.036, 0.007), (-0.072, 0.0, -0.001), TOOTH_WHITE),   # upper incisors at ~70 mm height (45 mm opening!)
        ((0.044, 0.010, 0.007), (-0.038, 0.021, -0.001), TOOTH_WHITE),  # upper right molars/premolars
        ((0.044, 0.010, 0.007), (-0.038, -0.021, -0.001), TOOTH_WHITE),  # upper left molars/premolars
    ):
        shapes.append(box(sz, pos=_flt(piv + r_up @ np.array(p_rel)), quat=q_up, color=col))

    # Molar #46 uncured cobalt-blue composite target disc and cured ivory disc (toggled via mk.show)
    mark_pos = (float(P.MOLAR_LOCAL[0]), float(P.MOLAR_LOCAL[1]), P.LOWER_ARCH_H + 0.0005)
    JAW_UNCURED_SHAPE = len(shapes)
    shapes.append(cylinder(P.MARK_D / 2.0, 0.0008, pos=mark_pos, color=MOLAR_BLUE, solid=False, glow=0.35))
    JAW_CURED_SHAPE = len(shapes)
    shapes.append(cylinder(P.MARK_D / 2.0, 0.0008, pos=mark_pos, color=MOLAR_CURED, solid=False))

    body("jaw", shapes, pos=(P.JAW_XY[0], P.JAW_XY[1], top), yaw=math.radians(P.JAW_YAW_DEG), kinematic=True, color=GUM_PINK)


def build_wand_and_hood(top: float) -> None:
    """Woodpecker LED.B wand (145 g) with uniform 8.0 mm curved fiber rod, top power button, serial guide carriers, and
    the 72 x 62 x 3.0 mm orange light hood with its short 4.0 mm rubber-lined grommet collar."""
    global WAND_BEAM_SHAPE

    barrel_r = P.WAND_W / 2.0  # 0.0155 m
    fiber_r = P.FIBER_D / 2.0  # 0.0040 m (uniform 8.0 mm diameter!)

    wand_shapes = [
        # Main 192 mm handpiece body (31 mm width cylinder + dorsal ridge to 34 mm height + dark nose bezel)
        cylinder(barrel_r, P.BODY_L - 0.012, pos=(-0.006, 0.0, 0.0), axis="x", color=WAND_SILVER, friction=mk.GRIPPY),
        cylinder(barrel_r * 0.92, 0.012, pos=(P.NOSE_X - 0.006, 0.0, 0.0), axis="x", color=WAND_ACCENT, friction=mk.GRIPPY),
        box(
            (0.110, 0.016, P.WAND_H - P.WAND_W),
            pos=(0.015, 0.0, barrel_r + (P.WAND_H - P.WAND_W) / 2.0),
            color=WAND_ACCENT,
        ),
        # Straight 32 mm base of the uniform 8.0 mm glass optical fiber
        cylinder(
            fiber_r,
            P.FIBER_STRAIGHT_L,
            pos=(P.NOSE_X + P.FIBER_STRAIGHT_L / 2.0, 0.0, 0.0),
            axis="x",
            color=GLASS_ROD,
        ),
    ]
    # Curved 55 deg elbow of the uniform 8.0 mm glass optical fiber (6 smooth overlapping capsule segments along R=44 mm)
    n_seg = 6
    seg_arc = P.FIBER_BEND_R * (P.FIBER_BEND_RAD / n_seg)
    for k in range(n_seg):
        deg_mid = (k + 0.5) * (P.FIBER_BEND_DEG / n_seg)
        p_seg, r_seg = P.fiber_frame_local(deg_mid, 0.0)
        # capsule runs along its local z; align local z with r_seg[:, 0] (rod tangent)
        r_cap = np.column_stack([r_seg[:, 1], r_seg[:, 2], r_seg[:, 0]])
        wand_shapes.append(capsule(fiber_r, seg_arc * 0.85, pos=_flt(p_seg), quat=_quat(r_cap), color=GLASS_ROD))

    # Emissive blue beam cone ahead of the distal tip (drawn only while beam_on is True)
    beam_len = 0.014
    p_beam = P.TIP_LOCAL_POS + (beam_len / 2.0 + 0.001) * P.TIP_TANGENT_LOCAL
    r_beam = np.column_stack([P.TIP_LOCAL_ROT[:, 1], P.TIP_LOCAL_ROT[:, 2], P.TIP_LOCAL_ROT[:, 0]])
    WAND_BEAM_SHAPE = len(wand_shapes)
    wand_shapes.append(cylinder(0.0048, beam_len, pos=_flt(p_beam), quat=_quat(r_beam), color=BEAM_CYAN, solid=False, glow=0.90))

    p_w0, r_w0 = P.wand_start_pose(top)
    p_w0_spawn = p_w0 + np.array([0.0, 0.0, P.OVER])
    wand_inertia = (2.2e-5, 5.2e-4, 5.2e-4)  # kg m2 of a 145 g, 260 mm slender cylinder about its midpoint
    body(
        "wand",
        wand_shapes,
        pos=_flt(p_w0_spawn),
        quat=_quat(r_w0),
        mass=P.WAND_MASS,
        color=WAND_SILVER,
        friction=mk.GRIPPY,
        center=(0.0, 0.0, 0.0),
        inertia=wand_inertia,
        contact_offset=0.0004,
        contacts=True,
    )

    # Power button on top of the handpiece (+z_w): pressing 1.6 mm of 2.5 mm stroke against 2.5 N activates the beam
    btn_local = np.array([P.BTN_X_LOCAL, 0.0, P.WAND_H / 2.0 + P.BTN_H / 2.0])
    btn_world = p_w0_spawn + r_w0 @ btn_local
    r_btn_w = np.column_stack([r_w0[:, 0], r_w0[:, 1], r_w0[:, 2]])
    body(
        "btn_cap",
        [cylinder(P.BTN_R, P.BTN_H, color=BTN_BLUE, friction=mk.GRIPPY)],
        pos=_flt(btn_world),
        quat=_quat(r_btn_w),
        mass=0.005,
        color=BTN_BLUE,
        inertia=(2e-6, 2e-6, 2e-6),
        contacts=True,
    )
    slide(
        "pwr_btn",
        "btn_cap",
        "wand",
        anchor=_flt(btn_world),
        axis=_flt(-r_w0[:, 2]),  # positive travel presses inward (-z_w) into wand
        limits=(0.0, P.BTN_TRAVEL),
        spring=P.BTN_SPRING,
        rest=0.0,
        damping=P.BTN_DAMPING,
    )

    # Serial guide carriers inside the 8.0 mm rod for curved-rod threading (c_elbow slides along straight base, c_tip
    # turns 55 deg about the bend's center of curvature so c_tip traces the exact circular centerline from tip to base)
    pass_all = ("wand", "hood", "cradle", "hood_slot", "jaw", "btn_cap")
    p_tip_w = p_w0_spawn + r_w0 @ P.TIP_LOCAL_POS
    r_tip_w = r_w0 @ P.TIP_LOCAL_ROT
    p_arc_w = p_w0_spawn + r_w0 @ P.ARC_CENTER_LOCAL
    body(
        "c_elbow",
        [sphere(0.0018, color=GLASS_ROD, passes=pass_all)],
        pos=_flt(p_arc_w),
        quat=_quat(r_w0),
        mass=0.006,
        inertia=(4e-6, 4e-6, 4e-6),
    )
    slide(
        "seat_s",
        "c_elbow",
        "wand",
        anchor=_flt(p_arc_w),
        axis=_flt(-r_w0[:, 0]),  # positive travel slides 0..22 mm toward the wand nose (-x_w)
        limits=(0.0, 0.0005),    # locked at 0 until bend_h completes the 55 deg bend
        spring=P.SEAT_SPRING_K,
        rest=0.0,
        damping=P.SEAT_DAMPING,
    )
    body(
        "c_tip",
        [sphere(0.0018, color=GLASS_ROD, passes=pass_all)],
        pos=_flt(p_tip_w),
        quat=_quat(r_tip_w),
        mass=0.006,
        inertia=(4e-6, 4e-6, 4e-6),
    )
    hinge(
        "bend_h",
        "c_tip",
        "c_elbow",
        anchor=_flt(p_arc_w),
        axis=_flt(r_w0[:, 1]),   # +y_w bend axis: travel 0 at 55 deg tip, -55 deg at straight base!
        limits=(-P.FIBER_BEND_RAD, 0.0),
        spring=P.ELBOW_SPRING_K,
        rest=0.0,
        damping=P.ELBOW_DAMPING,
    )

    # Oval orange glare hood (72 x 62 x 3.0 mm) with central open bore and short 4.0 mm rubber-lined grommet collar.
    # Local frame: x_h = bore axis (3.0 mm wing thickness), y_h = 72 mm major axis, z_h = 62 mm minor axis.
    hy, hz, ht = P.HOOD_W / 2.0, P.HOOD_H / 2.0, P.HOOD_T  # 0.036, 0.031, 0.003
    hole_half = 0.0080  # rigid plate window outside the soft elastomer grommet (4.0 mm rod + 2.8 mm capture tol + clearance)
    hood_shapes = [
        # Four annular wing slabs forming the 72 x 62 x 3.0 mm oval with a true open center hole (no added grip boss!)
        box((ht, hy - hole_half, 0.044), pos=(0.0, (hy + hole_half) / 2.0, 0.0), color=HOOD_ORANGE, friction=mk.GRIPPY),
        box((ht, hy - hole_half, 0.044), pos=(0.0, -(hy + hole_half) / 2.0, 0.0), color=HOOD_ORANGE, friction=mk.GRIPPY),
        box((ht, hole_half * 2.0, hz - hole_half), pos=(0.0, 0.0, (hz + hole_half) / 2.0), color=HOOD_ORANGE, friction=mk.GRIPPY),
        box((ht, hole_half * 2.0, hz - hole_half), pos=(0.0, 0.0, -(hz + hole_half) / 2.0), color=HOOD_ORANGE, friction=mk.GRIPPY),
    ]
    # Smooth oval perimeter trim (visual, rounding the 72 x 62 mm outline)
    for k in range(12):
        ang = 2.0 * math.pi * k / 12.0
        cy, cz = (hy - 0.005) * math.cos(ang), (hz - 0.005) * math.sin(ang)
        hood_shapes.append(cylinder(0.005, ht, pos=(0.0, cy, cz), axis="x", color=HOOD_ORANGE, solid=False))
    # Short 4.0 mm octagonal rigid collar housing around the compliant elastomer grommet (blocks sideways shaft entry)
    col_r_mid = 0.0090
    for k in range(8):
        ang = 2.0 * math.pi * k / 8.0
        q_seg = _quat(P._rx(math.degrees(ang)))
        hood_shapes.append(
            box(
                (P.COLLAR_L, 0.0068, 0.0020),
                pos=(0.0, col_r_mid * math.sin(ang), col_r_mid * math.cos(ang)),
                quat=q_seg,
                color=HOOD_ORANGE,
            )
        )
        # Visual dark-rubber inner grommet lining down to the 8.4 mm bore (r = 4.2 mm .. 8.0 mm; drag carried by seat_s)
        hood_shapes.append(
            box(
                (P.COLLAR_L + 0.0004, 0.0050, 0.0038),
                pos=(0.0, 0.0061 * math.sin(ang), 0.0061 * math.cos(ang)),
                quat=q_seg,
                color=GROMMET_RUBBER,
                solid=False,
            )
        )

    p_h0, r_h0 = P.hood_start_pose(top)
    body(
        "hood",
        hood_shapes,
        pos=_flt(p_h0 + np.array([0.0, 0.0, P.OVER])),
        quat=_quat(r_h0),
        mass=P.HOOD_SIM_MASS,
        color=HOOD_ORANGE,
        friction=mk.GRIPPY,
        inertia=(1.5e-5, 1.2e-5, 2.4e-5),
        contact_offset=0.0003,
        contacts=True,
    )

    # Dynamic weld made ONLY when hood bore enters coaxially over the distal 8.0 mm tip (never on side touch or pinch)
    weld("w_hood_on", "hood", "c_tip", anchor=_flt(p_h0 + np.array([0.0, 0.0, P.OVER])), made=False)

    mk.sense_contact("wand", "cradle", "hood")
    mk.sense_contact("btn_cap", "wand")


def build(top: float) -> None:
    build_fixtures(top)
    build_jaw(top)
    build_wand_and_hood(top)
