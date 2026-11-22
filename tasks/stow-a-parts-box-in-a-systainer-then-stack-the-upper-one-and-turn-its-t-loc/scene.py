"""The bodies and joints of the Systainer T-Loc coupling task, built from ``parts.py``.

Bodies:
- ``case_low``: lower Festool Systainer3 ``SYS3 S 76`` tub (light grey ABS, 4 anthracite feet, front central latch
  recess, dark-grey interior partitions forming the front-left 92 x 106 mm cell).
- ``lid_low``: hinged lid on ``case_low``'s rear spine, carrying the 71 mm recessed stacking deck, 75.2 mm side rims,
  73.2 mm front lead-in lips, two rigid rear L-shaped cleat overhangs (6.5 mm throat), and the front lid keeper lip.
- ``tloc_low``: green 3-position T-Loc rotary latch hinged on ``case_low``'s front wall (``-55 deg`` OPEN clears
  ``lid_low``; ``0 deg`` CLOSE retains ``lid_low``; ``+90 deg`` CONNECT retains ``lid_low`` with its second cam lobe
  while its coupling hook enters ``case_top``'s front bottom keeper slot).
- ``case_top``: second closed Festool ``SYS3 S 76`` with smooth side walls (no side handles), 5 mm front feet, two
  3.5 mm rear base hook tongues that slide under ``lid_low``'s rear cleats, and the front bottom keeper slot.
- ``parts_box``: red Allit EuroPlus Insert 45/1 (54 x 54 x 45 mm, 3 mm walls) small-parts bin.
"""

from __future__ import annotations

import mechkit as mk
import parts as P
from mechkit import body, box, cylinder, hinge

MECHKIT_HELPER = True  # declares on behalf of task.py

LIGHT_GREY = (0.84, 0.86, 0.88)
DECK_GREY = (0.76, 0.79, 0.82)
ANTHRACITE = (0.20, 0.22, 0.25)
DIVIDER_DARK = (0.30, 0.33, 0.37)
FESTOOL_GREEN = (0.13, 0.73, 0.26)
FESTOOL_BLUE = (0.10, 0.25, 0.45)
ALLIT_RED = (0.86, 0.16, 0.14)


def _case_low_shapes() -> list:
    """Lower SYS3 S 76 tub in its own frame (x forward-to-back, y right-to-left, z = 0 at foot bottom)."""
    foot_fric = (P.BENCH_FRICTION, P.BENCH_FRICTION)
    shapes = []
    # Four 5 mm corner/cleat feet (anti-slip bench grip so sliding upper case into cleats does not shove lower case)
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            shapes.append(
                box(
                    (0.014, P.HOOK_W, P.FOOT_H),
                    pos=(sx * 0.070, sy * P.CLEAT_Y, P.FOOT_H / 2.0),
                    color=ANTHRACITE,
                    friction=foot_fric,
                    friction_combine="max",
                )
            )
    # Floor (stepped back at the 128 mm central front latch bay)
    z_floor = P.FOOT_H + P.FLOOR_T / 2.0
    shapes.append(box((0.1550, P.CASE_W, P.FLOOR_T), pos=(0.0080, 0.0, z_floor), color=LIGHT_GREY))
    wing_w = P.CASE_W / 2.0 - P.RECESS_HALF_Y  # 0.0685 m
    for sy in (-1.0, 1.0):
        shapes.append(
            box(
                (0.0160, wing_w, P.FLOOR_T),
                pos=(-0.0775, sy * (P.RECESS_HALF_Y + wing_w / 2.0), z_floor),
                color=LIGHT_GREY,
            )
        )
    # Outer left and right smooth side walls (z = 7..68 mm)
    wall_h = P.TUB_RIM_Z - P.FLOOR_TOP_Z  # 0.061 m
    z_wall = P.FLOOR_TOP_Z + wall_h / 2.0
    for sy in (-1.0, 1.0):
        shapes.append(
            box(
                (P.CASE_D, P.WALL_T, wall_h),
                pos=(0.0, sy * (P.CASE_W / 2.0 - P.WALL_T / 2.0), z_wall),
                color=LIGHT_GREY,
                friction=mk.GRIPPY,
                friction_combine="max",
            )
        )
    # Rear wall (z = 7..63 mm so rear hinge at z = 70 mm swings cleanly to 62 deg without touching at frame 0)
    rear_h = 0.0630 - P.FLOOR_TOP_Z
    shapes.append(
        box(
            (P.WALL_T, P.INNER_W, rear_h),
            pos=(P.CASE_D / 2.0 - P.WALL_T / 2.0, 0.0, P.FLOOR_TOP_Z + rear_h / 2.0),
            color=LIGHT_GREY,
        )
    )
    # Front wing walls and front recess cheeks
    fwing_w = P.INNER_W / 2.0 - P.RECESS_HALF_Y  # 0.0710 m
    for sy in (-1.0, 1.0):
        shapes.append(
            box(
                (P.WALL_T, fwing_w, wall_h),
                pos=(-P.CASE_D / 2.0 + P.WALL_T / 2.0, sy * (P.RECESS_HALF_Y + fwing_w / 2.0), z_wall),
                color=LIGHT_GREY,
            )
        )
        shapes.append(
            box(
                (0.0195, P.WALL_T, wall_h),
                pos=(-0.07575, sy * (P.RECESS_HALF_Y + P.WALL_T / 2.0), z_wall),
                color=LIGHT_GREY,
            )
        )
    # Recessed central front wall (anthracite latch pocket, z = 7..65.5 mm)
    rec_h = 0.0655 - P.FLOOR_TOP_Z
    shapes.append(
        box(
            (P.WALL_T, 2.0 * P.RECESS_HALF_Y, rec_h),
            pos=(P.RECESS_WALL_X + P.WALL_T / 2.0, 0.0, P.FLOOR_TOP_Z + rec_h / 2.0),
            color=ANTHRACITE,
        )
    )
    # Interior partition ribs (maker's included long + short partitions; front-left cell is 92 x 106 mm)
    z_div = P.FLOOR_TOP_Z + P.DIVIDER_H / 2.0
    x_long = -P.INNER_D / 2.0 + P.CELL_D + P.DIVIDER_T / 2.0  # -0.0125 m
    shapes.append(box((P.DIVIDER_T, P.INNER_W, P.DIVIDER_H), pos=(x_long, 0.0, z_div), color=DIVIDER_DARK))
    y_short = P.INNER_W / 2.0 - P.CELL_W - P.DIVIDER_T / 2.0  # +0.0595 m
    for sy in (-1.0, 1.0):
        shapes.append(
            box(
                (P.CELL_D, P.DIVIDER_T, P.DIVIDER_H),
                pos=(-P.INNER_D / 2.0 + P.CELL_D / 2.0, sy * y_short, z_div),
                color=DIVIDER_DARK,
            )
        )
    rear_span = P.INNER_D / 2.0 - (x_long + P.DIVIDER_T / 2.0)
    shapes.append(
        box(
            (rear_span, P.DIVIDER_T, P.DIVIDER_H),
            pos=(x_long + P.DIVIDER_T / 2.0 + rear_span / 2.0, 0.0, z_div),
            color=DIVIDER_DARK,
        )
    )
    return shapes


def _lid_low_shapes() -> list:
    """Lower Systainer hinged lid in closed-frame coordinates (built rotated to LID_OPEN_DEG in build())."""
    deck_fric = (P.CASE_FRICTION, P.CASE_FRICTION)
    rear_deck_x0 = -0.0650  # stays behind central T-Loc bay so open T-Loc wings at x <= -0.0765 never touch lid_low
    deck_len = 0.0855 - rear_deck_x0  # 0.1505 m
    deck_cx = 0.5 * (rear_deck_x0 + 0.0855)  # +0.01025 m
    wing_deck_len = rear_deck_x0 - P.FRONT_RIM_X[1]  # 0.0135 m (-0.0785..-0.0650 outside central recess)
    wing_deck_cx = 0.5 * (P.FRONT_RIM_X[1] + rear_deck_x0)  # -0.07175 m
    rim_len = P.FRONT_RIM_X[1] - P.FRONT_RIM_X[0]  # 0.0070 m (-0.0855..-0.0785 front lead-in lip)
    rim_cx = 0.5 * (P.FRONT_RIM_X[0] + P.FRONT_RIM_X[1])  # -0.0820 m
    shapes = [
        # Recessed stacking deck behind central latch bay (top at z = 71.0 mm)
        box(
            (deck_len, 0.2360, 0.0025),
            pos=(deck_cx, 0.0, 0.06975),
            color=DECK_GREY,
            friction=deck_fric,
            friction_combine="min",
        ),
        # Central front bridge above recessed latch bay (x = -0.0695..-0.0650, z = 68.5..71.0 mm)
        box((0.0045, 2.0 * P.RECESS_HALF_Y, 0.0025), pos=(-0.06725, 0.0, 0.06975), color=LIGHT_GREY),
        # Front lid keeper shelf (x = -0.0755..-0.0695, y = -0.0035..+0.0035, top at z = 68.8 mm; clears T-Loc at -55 deg)
        box((0.0060, 0.0070, 0.0023), pos=(-0.0725, 0.0, 0.06765), color=ANTHRACITE),
    ]
    # Smooth side rims (z_top = 75.2 mm), front deck wings (71.0 mm), and 73.2 mm front lips outside central recess
    f_rim_w = 0.1180 - P.RECESS_HALF_Y  # 0.0540 m
    f_rim_y = P.RECESS_HALF_Y + f_rim_w / 2.0
    for sy in (-1.0, 1.0):
        shapes.append(
            box(
                (wing_deck_len, f_rim_w, 0.0025),
                pos=(wing_deck_cx, sy * f_rim_y, 0.06975),
                color=DECK_GREY,
                friction=deck_fric,
                friction_combine="min",
            )
        )
        shapes.append(
            box(
                (P.CASE_D, 0.0145, 0.0067),
                pos=(0.0, sy * 0.12525, 0.07185),
                color=LIGHT_GREY,
                friction=deck_fric,
                friction_combine="min",
            )
        )
        shapes.append(
            box(
                (rim_len, f_rim_w, 0.0047),
                pos=(rim_cx, sy * f_rim_y, 0.07085),
                color=LIGHT_GREY,
                friction=deck_fric,
                friction_combine="min",
            )
        )
        # Rear L-shaped cleat: vertical post (+0.0825..+0.0855) + forward overhang (+0.0705..+0.0855, z = 77.5..80.5 mm)
        shapes.append(box((0.0030, P.CLEAT_W, 0.0095), pos=(0.0840, sy * P.CLEAT_Y, 0.07575), color=FESTOOL_BLUE))
        shapes.append(
            box(
                (0.0150, P.CLEAT_W, P.CLEAT_ROOF_T),
                pos=(0.0780, sy * P.CLEAT_Y, P.LID_DECK_Z + P.CLEAT_GAP + P.CLEAT_ROOF_T / 2.0),
                color=FESTOOL_BLUE,
                friction=deck_fric,
                friction_combine="min",
            )
        )
    return shapes


def _tloc_shapes() -> list:
    """3-position T-Loc rotary latch in its own frame at TLOC_PIVOT_LOCAL (z = 45.0 mm; built rotated to OPEN)."""
    return [
        # Front T-knob crossbar (56 x 16 x 17 mm) and lower T-stem (16 x 22 x 17 mm -> 38 mm total height)
        box(
            (P.TLOC_DEPTH, P.TLOC_W, P.TLOC_BAR_T),
            pos=(-0.01175, 0.0, 0.0),
            color=FESTOOL_GREEN,
            friction=mk.GRIPPY,
            friction_combine="max",
        ),
        box(
            (P.TLOC_DEPTH, P.TLOC_BAR_T, 0.0220),
            pos=(-0.01175, 0.0, -0.0190),
            color=FESTOOL_GREEN,
            friction=mk.GRIPPY,
            friction_combine="max",
        ),
        # Pivot hub connecting front T-knob to rear cam plane
        cylinder(0.0070, 0.0055, pos=(-0.0005, 0.0, 0.0), axis="x", color=FESTOOL_GREEN),
        # Lobe 1 (local +z, r_inner = 24.4 mm -> z = 69.4 mm at 0 deg): retains lid_low keeper (top 68.8 mm) at CLOSE
        box((0.0045, 0.0040, 0.0036), pos=(0.0, 0.0, 0.0262), color=FESTOOL_GREEN),
        # Lobe 2 (local +y, r_inner = 24.4 mm): rotates to +z at +90 deg CONNECT to keep holding lid_low
        box((0.0055, 0.0036, 0.0040), pos=(-0.0005, 0.0262, 0.0), color=FESTOOL_GREEN),
        # Lobe 3 (local +y, x_in_low = -0.0780..-0.0735, z = 79.7..83.3 mm at +90 deg): sweeps cleanly over keeper bar
        box((0.0045, 0.0036, 0.0040), pos=(-0.0025, 0.0365, 0.0), color=FESTOOL_GREEN),
    ]


def _case_top_shapes() -> list:
    """Upper closed SYS3 S 76 (smooth side walls, no side handles; rear base hooks + front keeper slot)."""
    foot_fric = (P.CASE_FRICTION, P.CASE_FRICTION)
    ff_len = P.FRONT_FOOT_X[1] - P.FRONT_FOOT_X[0]
    ff_cx = 0.5 * (P.FRONT_FOOT_X[0] + P.FRONT_FOOT_X[1])
    shapes = []
    for sy in (-1.0, 1.0):
        # Front 5 mm feet (-0.0735..-0.0655 m): ride on lid_low's front lip at dx = -12 mm, drop onto 71 mm deck at dx = 0
        shapes.append(
            box(
                (ff_len, P.HOOK_W, P.FOOT_H),
                pos=(ff_cx, sy * P.CLEAT_Y, P.FOOT_H / 2.0),
                color=ANTHRACITE,
                friction=foot_fric,
                friction_combine="min",
            )
        )
        # Rear 5 mm foot root (+0.0600..+0.0665 m) + rearward 3.5 mm hook tongue (+0.0665..+0.0820 m)
        shapes.append(
            box(
                (0.0065, P.HOOK_W, P.FOOT_H),
                pos=(0.06325, sy * P.CLEAT_Y, P.FOOT_H / 2.0),
                color=ANTHRACITE,
                friction=foot_fric,
                friction_combine="min",
            )
        )
        shapes.append(
            box(
                (0.0155, P.HOOK_W, P.HOOK_T),
                pos=(0.07425, sy * P.CLEAT_Y, P.HOOK_T / 2.0),
                color=ANTHRACITE,
                friction=foot_fric,
                friction_combine="min",
            )
        )
    # Main closed body (z = 5..76 mm), with rear cleat relief pockets above the two rear hook tongues (z = 11.5..76 mm)
    # Front/middle solid body block: x in [-0.0695, +0.0665], full 265 mm width, z in [0.005, 0.076]
    body_h = P.CASE_BODY_H  # 0.071 m
    z_mid = P.FOOT_H + body_h / 2.0  # 0.0405 m
    shapes.append(
        box((0.1360, P.CASE_W, body_h), pos=(-0.0015, 0.0, z_mid), color=LIGHT_GREY, friction=mk.GRIPPY, friction_combine="max")
    )
    # Front left and right wings ahead of the central recess: x in [-0.0855, -0.0695]
    wing_w = P.CASE_W / 2.0 - P.RECESS_HALF_Y
    for sy in (-1.0, 1.0):
        shapes.append(
            box(
                (0.0160, wing_w, body_h),
                pos=(-0.0775, sy * (P.RECESS_HALF_Y + wing_w / 2.0), z_mid),
                color=LIGHT_GREY,
                friction=mk.GRIPPY,
                friction_combine="max",
            )
        )
    # Rear section (x in [+0.0665, +0.0855]): upper band z = 0.0115..0.0760 across full width, plus lower pillars between cleats
    upper_rear_h = P.CASE_H - 0.0115  # 0.0645 m
    shapes.append(
        box(
            (0.0190, P.CASE_W, upper_rear_h),
            pos=(0.0760, 0.0, 0.0115 + upper_rear_h / 2.0),
            color=LIGHT_GREY,
            friction=mk.GRIPPY,
            friction_combine="max",
        )
    )
    # Central rear lower filler (|y| <= 0.061) and outer corner fillers (|y| in [0.109, 0.1325]) keep side walls smooth
    shapes.append(box((0.0190, 0.1220, 0.0065), pos=(0.0760, 0.0, 0.00825), color=LIGHT_GREY))
    for sy in (-1.0, 1.0):
        shapes.append(
            box(
                (0.0190, 0.0235, 0.0065),
                pos=(0.0760, sy * 0.12075, 0.00825),
                color=LIGHT_GREY,
                friction=mk.GRIPPY,
                friction_combine="max",
            )
        )
    # Front bottom keeper bar (x = -77.5..-72.5 mm, y = -6..+2 mm, z_world = 76..79 mm), unslid cam stop, and ceiling
    shapes.append(box((0.0050, 0.0080, 0.0030), pos=(-0.0750, -0.0020, 0.0065), color=ANTHRACITE))
    shapes.append(box((0.0060, 0.0180, 0.0045), pos=(-0.0635, 0.0090, 0.00275), color=ANTHRACITE))
    shapes.append(box((0.0060, 0.0180, 0.0040), pos=(-0.0750, 0.0, 0.0215), color=ANTHRACITE))
    # Visual cues (drawn only, zero added grip aid): Festool blue lid identification band + closed anthracite T-Loc
    shapes.append(box((0.1200, 0.2200, 0.0012), pos=(0.005, 0.0, P.CASE_H + 0.0006), color=FESTOOL_BLUE, solid=False))
    shapes.append(box((0.0060, P.TLOC_W, P.TLOC_BAR_T), pos=(-0.0730, 0.0, 0.0560), color=ANTHRACITE, solid=False))
    shapes.append(box((0.0060, P.TLOC_BAR_T, 0.0240), pos=(-0.0730, 0.0, 0.0350), color=ANTHRACITE, solid=False))
    return shapes


def _parts_box_shapes() -> list:
    """Allit EuroPlus Insert 45/1 (54 x 54 x 45 mm outer, 48 x 48 x 42 mm inner, smooth red polystyrene)."""
    w, d, h, t = P.BOX_W, P.BOX_D, P.BOX_H, P.BOX_WALL
    kw = {"color": ALLIT_RED, "friction": (0.55, 0.45)}
    shapes = [
        box((d, w, t), pos=(0.0, 0.0, t / 2.0), **kw),
        box((d, t, h - t), pos=(0.0, +(w - t) / 2.0, t + (h - t) / 2.0), **kw),
        box((d, t, h - t), pos=(0.0, -(w - t) / 2.0, t + (h - t) / 2.0), **kw),
        box((t, w - 2.0 * t, h - t), pos=(+(d - t) / 2.0, 0.0, t + (h - t) / 2.0), **kw),
        box((t, w - 2.0 * t, h - t), pos=(-(d - t) / 2.0, 0.0, t + (h - t) / 2.0), **kw),
    ]
    return shapes


def build(top: float) -> None:
    import numpy as np

    low_pos = np.array([P.CASE_LOW_XY[0], P.CASE_LOW_XY[1], top + P.OVER], dtype=float)
    top_pos = (P.CASE_TOP_XY[0], P.CASE_TOP_XY[1], top + P.OVER)
    box_pos = (P.BOX_START_XY[0], P.BOX_START_XY[1], top + P.OVER)
    tloc_pos = tuple(float(v) for v in (low_pos + np.array(P.TLOC_PIVOT_LOCAL, dtype=float)))
    lid_anchor = low_pos + np.array(P.LID_HINGE_LOCAL, dtype=float)

    # Build lid_low directly in its open start pose (+LID_OPEN_DEG) and tloc_low in OPEN (-55 deg)
    r_lid0 = mk.axis_angle((0.0, 1.0, 0.0), P.LID_OPEN_RAD)
    lid_pos0 = tuple(float(v) for v in (lid_anchor + r_lid0 @ (low_pos - lid_anchor)))
    lid_quat0 = mk.matrix_to_quat(r_lid0)
    tloc_quat0 = mk.matrix_to_quat(mk.axis_angle((1.0, 0.0, 0.0), P.TLOC_OPEN_RAD))

    body("case_low", _case_low_shapes(), pos=tuple(float(v) for v in low_pos), mass=P.TUB_LOW_MASS, color=LIGHT_GREY, contacts=True)
    body("lid_low", _lid_low_shapes(), pos=lid_pos0, quat=lid_quat0, mass=P.LID_MASS, color=LIGHT_GREY, contacts=True)
    body("tloc_low", _tloc_shapes(), pos=tloc_pos, quat=tloc_quat0, mass=P.TLOC_MASS, color=FESTOOL_GREEN, contacts=True)
    body("case_top", _case_top_shapes(), pos=top_pos, mass=P.CASE_TOP_MASS, color=LIGHT_GREY, contacts=True)
    body(
        "parts_box",
        _parts_box_shapes(),
        pos=box_pos,
        mass=P.BOX_SIM_MASS,
        color=ALLIT_RED,
        center=(0.0, 0.0, 0.012),
        inertia=(2.2e-5, 2.2e-5, 2.8e-5),
        contacts=True,
    )

    hinge(
        "lid_hinge",
        "lid_low",
        "case_low",
        anchor=tuple(float(v) for v in lid_anchor),
        axis=(0.0, 1.0, 0.0),
        limits=(-P.LID_OPEN_RAD, P.LID_MAX_RAD - P.LID_OPEN_RAD),
        spring=P.LID_STAY_K,
        rest=P.lid_stay_travel_rest(0.0),
        damping=P.LID_STAY_DAMP,
    )
    hinge(
        "tloc",
        "tloc_low",
        "case_low",
        anchor=tloc_pos,
        axis=(1.0, 0.0, 0.0),
        limits=(P.TLOC_MIN_RAD - P.TLOC_OPEN_RAD, P.TLOC_MAX_RAD - P.TLOC_OPEN_RAD),
        spring=P.TLOC_K,
        rest=P.tloc_travel_rest(0.0),
        damping=P.TLOC_DAMP,
    )

    mk.sense_contact("case_top", "lid_low", "tloc_low", "case_low")
    mk.sense_contact("lid_low", "case_low", "tloc_low", "parts_box", "case_top")
    mk.sense_contact("parts_box", "case_low", "lid_low")
