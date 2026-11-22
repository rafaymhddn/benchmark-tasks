"""The bodies and joints of the Minifix 15 cam-lock shelf task, built from ``parts.py``:
- ``cabinet_side``: fixed upright 16 mm white melamine panel with two projecting Häfele S100 Ø 7.0 mm connecting bolts;
- ``support_block``: fixed 48 mm smooth HDPE riser supporting the shelf at the bolt plane while leaving both rear wings clear;
- ``shelf``: free 16 mm oak-decor melamine shelf with two 16-stave Ø 8.0 mm horizontal edge bores (0.50 mm radial play)
  and two Ø 15.0 mm cam pockets;
- ``cam_l``, ``cam_r``: two free die-cast zinc Minifix 15 housings on vertical hinges in the shelf, starting 90 deg closed,
  with physical outer blocking skirts, compliant eccentric spiral draw flanges, and 3.5 mm-deep PZ2 top recesses;
- ``cradle_l``, ``cradle_r`` and ``driver_l``, ``driver_r``: bench holder wells and two free 37 g Wera 355 PZ stubby screwdrivers.
"""

from __future__ import annotations

import math

import numpy as np

import mechkit as mk
import parts as P
from mechkit import body, box, cylinder, hinge, soften

MECHKIT_HELPER = True

WHITE = (0.93, 0.94, 0.95)
OAK = (0.64, 0.45, 0.28)
HDPE_DARK = (0.23, 0.25, 0.28)
CRADLE_BLUE = (0.20, 0.30, 0.42)
STEEL = (0.46, 0.48, 0.52)
STEEL_BRIGHT = (0.72, 0.75, 0.79)
ZINC = (0.62, 0.65, 0.69)
SLOT_DARK = (0.12, 0.13, 0.15)
ARROW_RED = (0.90, 0.16, 0.14)
TICK_YELLOW = (0.96, 0.82, 0.12)
WERA_BLACK = (0.14, 0.15, 0.16)
WERA_GREEN = (0.12, 0.68, 0.28)

CONNS = ("l", "r")  # left (+y) and right (-y)


def _quat(matrix: np.ndarray) -> tuple[float, ...]:
    return tuple(float(v) for v in mk.matrix_to_quat(matrix))


def _frame(columns) -> tuple[float, ...]:
    return _quat(np.column_stack(columns))


def side_sign(c: str) -> float:
    return 1.0 if c == "l" else -1.0


def build_cabinet_side(top: float) -> None:
    """Fixed upright 16 mm white panel at x = X_SIDE with rear L-brackets and two S100 bolts at y = +-CAM_Y, z = Z_MID."""
    z_panel_mid = P.SIDE_H / 2.0 - P.Z_MID
    shapes = [
        box((P.PANEL_T, P.SIDE_W, P.SIDE_H), pos=(P.PANEL_T / 2.0, 0.0, z_panel_mid), color=WHITE),
    ]
    # Rear steel clamping L-brackets
    for s in (1.0, -1.0):
        by = s * 0.130
        shapes.append(box((0.005, 0.028, 0.065), pos=(P.PANEL_T + 0.0025, by, 0.0325 - P.Z_MID), color=STEEL))
        shapes.append(box((0.042, 0.028, 0.005), pos=(P.PANEL_T + 0.021, by, 0.0025 - P.Z_MID), color=STEEL))
    # Two Häfele S100 connecting bolts projecting -x from x = 0.0 (at z_local = 0.0 -> world top + Z_MID)
    neck_len = P.BOLT_BACK_FROM_SIDE - P.BOLT_COLLAR_L
    tip_bevel = 0.0004
    head_main = P.BOLT_HEAD_L - tip_bevel
    for c in CONNS:
        y = side_sign(c) * P.CAM_Y
        shapes.extend([
            cylinder(P.BOLT_COLLAR_D / 2.0, P.BOLT_COLLAR_L, pos=(-P.BOLT_COLLAR_L / 2.0, y, 0.0), axis="x", color=STEEL),
            cylinder(
                P.BOLT_NECK_D / 2.0,
                neck_len,
                pos=(-(P.BOLT_COLLAR_L + P.BOLT_BACK_FROM_SIDE) / 2.0, y, 0.0),
                axis="x",
                color=STEEL,
                friction=(0.15, 0.12),
                friction_combine="min",
            ),
            cylinder(
                P.BOLT_HEAD_D / 2.0,
                head_main,
                pos=(-(P.BOLT_BACK_FROM_SIDE + P.BOLT_PROJ - tip_bevel) / 2.0, y, 0.0),
                axis="x",
                color=STEEL_BRIGHT,
                friction=(0.15, 0.12),
                friction_combine="min",
            ),
            # Inscribed 4.2 x 5.6 mm flat rear shoulder inside Ø 7.00 mm head (hypot(2.1, 2.8) = 3.50 mm exact)
            box(
                (head_main, 0.0042, 0.0056),
                pos=(-(P.BOLT_BACK_FROM_SIDE + P.BOLT_PROJ - tip_bevel) / 2.0, y, 0.0),
                color=STEEL_BRIGHT,
                friction=(0.15, 0.12),
                friction_combine="min",
            ),
            cylinder(
                (P.BOLT_HEAD_D - 2.0 * tip_bevel) / 2.0,
                tip_bevel,
                pos=(-(P.BOLT_PROJ - tip_bevel / 2.0), y, 0.0),
                axis="x",
                color=STEEL_BRIGHT,
                friction=(0.15, 0.12),
                friction_combine="min",
            ),
            # High-contrast alignment ticks on top edge and front face above each bolt
            box((P.PANEL_T, 0.008, 0.0008), pos=(P.PANEL_T / 2.0, y, P.SIDE_H - P.Z_MID + 0.0004), color=TICK_YELLOW, solid=False),
            box((0.0008, 0.006, 0.018), pos=(-0.0004, y, 0.036), color=TICK_YELLOW, solid=False),
        ])
    body("cabinet_side", shapes, pos=(P.X_SIDE, 0.0, top + P.Z_MID), kinematic=True, color=WHITE, friction=(0.25, 0.22))


def build_support_block(top: float) -> None:
    """Fixed 48 mm smooth HDPE riser: 260 mm wide under the cams, 150 mm wide at the rear so both rear shelf wings overhang."""
    fx0, fx1 = P.SUPPORT_FRONT_X
    rx0, rx1 = P.SUPPORT_REAR_X
    shapes = [
        box((fx1 - fx0, P.SUPPORT_FRONT_W, P.SUPPORT_H), pos=((fx0 + fx1) / 2.0, 0.0, P.SUPPORT_H / 2.0), color=HDPE_DARK),
        box((rx1 - rx0, P.SUPPORT_REAR_W, P.SUPPORT_H), pos=((rx0 + rx1) / 2.0, 0.0, P.SUPPORT_H / 2.0), color=HDPE_DARK),
    ]
    # Start-line ticks at x = X_SIDE - START_GAP (0.318 m)
    x_start_edge = P.X_SIDE - P.START_GAP
    for s in (1.0, -1.0):
        shapes.append(box((0.003, 0.012, 0.0006), pos=(x_start_edge, s * 0.122, P.SUPPORT_H + 0.0003), color=TICK_YELLOW, solid=False))
    body(
        "support_block",
        shapes,
        pos=(0.0, 0.0, top),
        kinematic=True,
        color=HDPE_DARK,
        friction=(P.MU_SLIDE, P.MU_SLIDE),
        friction_combine="min",
    )


def _edge_bore_band() -> list:
    """Front 16.0 mm of the shelf (x_local in [+0.054, +0.070]): two 16-stave Ø 8.00 mm horizontal bores with 0.8 mm chamfers."""
    x0, x1 = P.CAM_X_IN_SHELF + 0.008, P.SHELF_D / 2.0  # +0.054 to +0.070 (16.0 mm)
    depth = x1 - x0
    xc = 0.5 * (x0 + x1)
    win = 0.0055  # half-size of the 11.0 x 11.0 mm square opening around each Ø 8.0 mm bore
    rail_h = P.PANEL_T / 2.0 - win  # 2.5 mm top and bottom wood rails
    shapes = [
        box((depth, P.SHELF_W, rail_h), pos=(xc, 0.0, s * (win + rail_h / 2.0)), color=OAK)
        for s in (-1.0, 1.0)
    ]
    y_edges = [
        -P.SHELF_W / 2.0,
        -P.CAM_Y - win,
        -P.CAM_Y + win,
        P.CAM_Y - win,
        P.CAM_Y + win,
        P.SHELF_W / 2.0,
    ]
    for y_lo, y_hi in zip(y_edges[0::2], y_edges[1::2]):
        shapes.append(box((depth, y_hi - y_lo, 2.0 * win), pos=(xc, 0.5 * (y_lo + y_hi), 0.0), color=OAK))

    inner_r = P.EDGE_BORE_D / 2.0  # 0.00400 m exact across-flats apothem -> 0.50 mm radial clearance around Ø 7.00 mm head
    chamfer = P.BORE_CHAMFER  # 0.0008 m
    straight_len = depth - chamfer
    x_straight = x0 + straight_len / 2.0
    stave_w = 2.0 * inner_r * math.tan(math.pi / P.BORE_STAVES) * 1.12
    along = np.array([1.0, 0.0, 0.0])

    for c in CONNS:
        y0 = side_sign(c) * P.CAM_Y
        for k in range(P.BORE_STAVES):
            a = 2.0 * math.pi * (k + 0.5) / P.BORE_STAVES
            ca, sa = math.cos(a), math.sin(a)
            radial = np.array([0.0, ca, sa])
            tangent = np.array([0.0, -sa, ca])
            reach = win / max(abs(ca), abs(sa)) + 0.0004
            mid_r = 0.5 * (inner_r + reach)
            shapes.append(
                box(
                    (straight_len, stave_w, reach - inner_r),
                    pos=(x_straight, y0 + mid_r * ca, mid_r * sa),
                    quat=_frame([along, -tangent, radial]),
                    color=OAK,
                    friction=(0.16, 0.14),
                    friction_combine="min",
                )
            )
            # 0.8 mm x 45 deg mouth chamfer (recessed 0.05 mm inside x1 so the +x joining face stays strictly planar)
            slant = math.sqrt(2.0) * chamfer * 0.92
            thick = 0.0005
            chamfer_r = inner_r + 0.5 * chamfer
            in_normal = (along - radial) / math.sqrt(2.0)
            slant_dir = (along + radial) / math.sqrt(2.0)
            center = np.array([x1 - 0.5 * chamfer - 0.00005, y0, 0.0]) + chamfer_r * radial - 0.5 * thick * in_normal
            shapes.append(
                box(
                    (slant, stave_w * 1.12, thick),
                    pos=tuple(float(v) for v in center),
                    quat=_frame([slant_dir, tangent, in_normal]),
                    color=OAK,
                    friction=(0.16, 0.14),
                    friction_combine="min",
                )
            )
    return shapes


def build_shelf(top: float) -> None:
    """Free 16 mm shelf with rear board, two Ø 15 mm cam pocket cutouts, and two Ø 8.0 mm horizontal edge bores."""
    x_rear_lo = -P.SHELF_D / 2.0  # -0.070
    x_pocket_lo = P.CAM_X_IN_SHELF - 0.008  # +0.038
    x_pocket_hi = P.CAM_X_IN_SHELF + 0.008  # +0.054
    rear_len = x_pocket_lo - x_rear_lo  # 0.108 m
    pocket_len = x_pocket_hi - x_pocket_lo  # 0.016 m
    floor_t = P.PANEL_T - P.CAM_BORE_DEPTH  # 0.0035 m bottom wood under the 12.5 mm-deep cam bore
    wall_h = P.CAM_BORE_DEPTH  # 0.0125 m

    shapes = [
        box((rear_len, P.SHELF_W, P.PANEL_T), pos=(0.5 * (x_rear_lo + x_pocket_lo), 0.0, 0.0), color=OAK),
        box((pocket_len, P.SHELF_W, floor_t), pos=(P.CAM_X_IN_SHELF, 0.0, -P.PANEL_T / 2.0 + floor_t / 2.0), color=OAK),
    ]
    p_win = 0.008  # 16 x 16 mm pocket clearance around each hinged Ø 14.9 mm cam
    y_edges = [
        -P.SHELF_W / 2.0,
        -P.CAM_Y - p_win,
        -P.CAM_Y + p_win,
        P.CAM_Y - p_win,
        P.CAM_Y + p_win,
        P.SHELF_W / 2.0,
    ]
    for y_lo, y_hi in zip(y_edges[0::2], y_edges[1::2]):
        shapes.append(
            box(
                (pocket_len, y_hi - y_lo, wall_h),
                pos=(P.CAM_X_IN_SHELF, 0.5 * (y_lo + y_hi), P.PANEL_T / 2.0 - wall_h / 2.0),
                color=OAK,
            )
        )
    shapes.extend(_edge_bore_band())
    # Alignment ticks on shelf top face at y = +-CAM_Y near the +x joining edge
    for c in CONNS:
        shapes.append(
            box(
                (0.008, 0.004, 0.0005),
                pos=(P.SHELF_D / 2.0 - 0.005, side_sign(c) * P.CAM_Y, P.PANEL_T / 2.0 + 0.00025),
                color=TICK_YELLOW,
                solid=False,
            )
        )
    body(
        "shelf",
        shapes,
        pos=(P.X_SHELF_START, 0.0, top + P.Z_MID + P.OVER),
        mass=P.SHELF_MASS,
        color=OAK,
        friction=(P.SHELF_GRIP_FRICTION, P.SHELF_GRIP_FRICTION),
        inertia=P.box_inertia(P.SHELF_MASS, (P.SHELF_D, P.SHELF_W, P.PANEL_T)),
        contacts=True,
    )


def _cam_shapes() -> list:
    """One Minifix 15 zinc housing in its OPEN local frame (+x_c = open throat & red arrow):
    - Outer blocking skirt over [38 deg, 322 deg] (r = 6.6..7.45 mm, z in [-4.2, +4.5] mm) -> blocks bolt 8.55 mm short when closed;
    - Upper & lower compliant eccentric spiral crescent flanges over [58 deg, 195 deg] (|z| in [2.8, 4.3] mm, straddling the
      Ø 4.4 mm bolt neck and engaging the Ø 7.0 mm bolt head's +x shoulder to draw the shelf flush);
    - Top annular rim around the Ø 7.2 mm x 3.5 mm-deep PZ2 recess (floor at z = +4.5 mm, top at z = +8.0 mm);
    - High-contrast PZ2 cross bars and bright red throat-direction arrow on z = +8.1 mm.
    """
    shapes = []
    along_z = np.array([0.0, 0.0, 1.0])

    # 1. Outer cylindrical blocking skirt (leaves +-38 deg = 8.1 mm throat open at +x_c; a 5.1 mm peripheral neck slot
    # z in [-2.55, +2.55] mm lets the Ø 4.0 mm bolt neck pass across the entire turn while upper & lower zinc rims
    # |z| in [2.55, 4.50] mm physically block the Ø 7.0 mm bolt head 9.0 mm short when closed!)
    n_skirt = 12
    a0, a1 = math.radians(38.0), math.radians(322.0)
    da = (a1 - a0) / n_skirt
    r_in_s, r_out_s = 0.0066, P.CAM_OUTER_R
    w_s = 2.0 * r_out_s * math.tan(da / 2.0) * 1.08
    for k in range(n_skirt):
        a = a0 + (k + 0.5) * da
        radial = np.array([math.cos(a), math.sin(a), 0.0])
        tangent = np.array([-math.sin(a), math.cos(a), 0.0])
        rm = 0.5 * (r_in_s + r_out_s)
        for s_z in (-1.0, 1.0):
            shapes.append(
                box(
                    (r_out_s - r_in_s, w_s, 0.00195),
                    pos=(rm * math.cos(a), rm * math.sin(a), s_z * 0.003525),
                    quat=_frame([radial, tangent, along_z]),
                    color=ZINC,
                    friction=(0.10, 0.08),
                    friction_combine="min",
                )
            )

    # 2. Eccentric spiral crescent flanges (upper z in [+2.8, +4.3] mm, lower z in [-4.3, -2.8] mm)
    n_spiral = 11
    s_deg0, s_deg1 = P.SPIRAL_START_DEG, P.SPIRAL_END_DEG
    d_deg = (s_deg1 - s_deg0) / n_spiral
    r_back = 0.0067
    for k in range(n_spiral):
        deg_m = s_deg0 + (k + 0.5) * d_deg
        a = math.radians(deg_m)
        r_in = P.spiral_r_in(deg_m)
        radial = np.array([math.cos(a), math.sin(a), 0.0])
        tangent = np.array([-math.sin(a), math.cos(a), 0.0])
        rm = 0.5 * (r_in + r_back)
        seg_w = max(0.0011, 2.0 * r_in * math.tan(math.radians(d_deg) / 2.0) * 1.25)
        for s_z in (-1.0, 1.0):
            stave = box(
                (r_back - r_in, seg_w, 0.0020),
                pos=(rm * math.cos(a), rm * math.sin(a), s_z * 0.00330),
                quat=_frame([radial, tangent, along_z]),
                color=ZINC,
                friction=(0.06, 0.05),
                friction_combine="min",
            )
            shapes.append(soften(stave, P.SPIRAL_STIFFNESS, P.SPIRAL_DAMPING))

    # 3. PZ2 recess floor (top at z = +4.5 mm = 3.5 mm below the z = +8.0 mm cam/shelf top) & 45 deg conical PZ2 funnel rim
    recess_floor_top = P.PANEL_T / 2.0 - P.SLOT_RECESS_DEPTH  # +0.0045 m
    shapes.append(
        cylinder(
            P.CAM_OUTER_R,
            0.00095,
            pos=(0.0, 0.0, recess_floor_top - 0.000475),
            axis="z",
            color=ZINC,
            friction=(0.08, 0.06),
            friction_combine="min",
        )
    )
    r_bot = 0.0024  # Ø 4.8 mm flat bottom around the Ø 3.0 mm pointed PZ2 tip
    r_top = 0.0059  # Ø 11.8 mm conical PZ2 mouth (3.5 mm drop over 3.5 mm radial run = 45 deg self-centering cone)
    n_rim = 8
    w_outer = 2.0 * P.CAM_OUTER_R * math.tan(math.pi / n_rim) * 1.06
    for k in range(n_rim):
        a = 2.0 * math.pi * (k + 0.5) / n_rim
        radial = np.array([math.cos(a), math.sin(a), 0.0])
        tangent = np.array([-math.sin(a), math.cos(a), 0.0])
        # Outer flat collar r in [5.9, 7.45] mm
        rm_flat = 0.5 * (r_top + P.CAM_OUTER_R)
        shapes.append(
            box(
                (P.CAM_OUTER_R - r_top, w_outer, P.SLOT_RECESS_DEPTH),
                pos=(rm_flat * math.cos(a), rm_flat * math.sin(a), recess_floor_top + P.SLOT_RECESS_DEPTH / 2.0),
                quat=_frame([radial, tangent, along_z]),
                color=ZINC,
                friction=(0.10, 0.08),
                friction_combine="min",
            )
        )
        # 45 deg conical PZ2 bevel r in [2.4, 5.9] mm guiding tip smoothly to (0, 0) at depth = 3.5 mm
        in_up = (along_z - radial) / math.sqrt(2.0)
        slant_dir = (along_z + radial) / math.sqrt(2.0)
        slant_len = math.sqrt(2.0) * P.SLOT_RECESS_DEPTH
        thick = 0.0008
        c_mid = np.array([0.0, 0.0, recess_floor_top + 0.5 * P.SLOT_RECESS_DEPTH]) + 0.5 * (r_bot + r_top) * radial - 0.5 * thick * in_up
        shapes.append(
            box(
                (slant_len, 2.0 * r_top * math.tan(math.pi / n_rim) * 1.08, thick),
                pos=tuple(float(v) for v in c_mid),
                quat=_frame([slant_dir, tangent, in_up]),
                color=ZINC,
                friction=(0.08, 0.06),
                friction_combine="min",
            )
        )

    # 4. Visual PZ2 cross slot and high-contrast red directional arrow (+x_c = throat direction)
    z_vis = P.PANEL_T / 2.0 + 0.0002
    shapes.extend([
        box((0.011, 0.0016, 0.0003), pos=(0.0, 0.0, recess_floor_top + 0.0002), color=SLOT_DARK, solid=False),
        box((0.0016, 0.011, 0.0003), pos=(0.0, 0.0, recess_floor_top + 0.0002), color=SLOT_DARK, solid=False),
        # Bright red arrow wedge on the +x_c rim pointing straight along the open throat
        box((0.0038, 0.0034, 0.0005), pos=(0.0054, 0.0, z_vis), color=ARROW_RED, solid=False, glow=0.25),
        box((0.0024, 0.0024, 0.0005), pos=(0.0062, 0.0, z_vis + 0.0001), yaw=math.pi / 4.0, color=ARROW_RED, solid=False, glow=0.25),
        # Contrasting dark tail bar on -x_c rim so 0 deg vs 180 deg is unmistakable at 320 x 176
        box((0.0030, 0.0028, 0.0005), pos=(-0.0055, 0.0, z_vis), color=SLOT_DARK, solid=False),
    ])
    return shapes


def build_cams(top: float) -> None:
    """Two hinged zinc cam housings in the shelf, built at +90 deg CW closed (arrow pointing -y)."""
    cam_shapes = _cam_shapes()
    lo_rad = P.travel_from_cam_deg(P.CAM_LIMITS_DEG[0])
    hi_rad = P.travel_from_cam_deg(P.CAM_LIMITS_DEG[1])
    for c in CONNS:
        anchor = (P.X_SHELF_START + P.CAM_X_IN_SHELF, side_sign(c) * P.CAM_Y, top + P.Z_MID + P.OVER)
        body(
            f"cam_{c}",
            cam_shapes,
            pos=anchor,
            yaw=-math.pi / 2.0,  # +x_c points along -y -> cam_deg = +90 deg CW closed at travel = 0
            mass=P.CAM_MASS,
            color=ZINC,
            angular_damping=4.0,
            inertia=P.cam_inertia(),
            contacts=True,
        )
        hinge(
            f"cam_joint_{c}",
            f"cam_{c}",
            "shelf",
            anchor=anchor,
            axis=(0.0, 0.0, -1.0),  # +travel = clockwise viewed from above (+z)
            limits=(lo_rad, hi_rad),
            friction=P.CAM_FRICTION,
        )


def build_cradles_and_drivers(top: float) -> None:
    """Two bench holder blocks with Ø 9.2 mm shaft wells and two free 37 g Wera 355 PZ stubby screwdrivers."""
    well_r = P.CRADLE_WELL_D / 2.0  # 0.0075 m
    wall = P.CRADLE_SIDE / 2.0 - well_r  # 0.0195 m
    well_h = P.CRADLE_H - P.CRADLE_WELL_FLOOR
    chamfer = 0.0075  # opens Ø 15.0 mm well to 30.0 mm funnel mouth so +-14 mm offset self-centres
    cradle_shapes = [
        box((P.CRADLE_SIDE, P.CRADLE_SIDE, P.CRADLE_WELL_FLOOR), pos=(0.0, 0.0, P.CRADLE_WELL_FLOOR / 2.0), color=CRADLE_BLUE),
        box((wall, P.CRADLE_SIDE, well_h), pos=(-(well_r + wall / 2.0), 0.0, P.CRADLE_WELL_FLOOR + well_h / 2.0), color=CRADLE_BLUE),
        box((wall, P.CRADLE_SIDE, well_h), pos=((well_r + wall / 2.0), 0.0, P.CRADLE_WELL_FLOOR + well_h / 2.0), color=CRADLE_BLUE),
        box((2.0 * well_r, wall, well_h), pos=(0.0, -(well_r + wall / 2.0), P.CRADLE_WELL_FLOOR + well_h / 2.0), color=CRADLE_BLUE),
        box((2.0 * well_r, wall, well_h), pos=(0.0, (well_r + wall / 2.0), P.CRADLE_WELL_FLOOR + well_h / 2.0), color=CRADLE_BLUE),
    ]
    # 45 deg funnel plates around mouth (opens Ø 15.0 mm well to 30.0 mm mouth)
    along_z = np.array([0.0, 0.0, 1.0])
    for a in (0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi):
        radial = np.array([math.cos(a), math.sin(a), 0.0])
        tangent = np.array([-math.sin(a), math.cos(a), 0.0])
        in_up = (along_z - radial) / math.sqrt(2.0)
        slant_dir = (along_z + radial) / math.sqrt(2.0)
        center = np.array([0.0, 0.0, P.CRADLE_H - 0.5 * chamfer - 0.0002]) + (well_r + 0.5 * chamfer) * radial - 0.0004 * in_up
        cradle_shapes.append(
            box(
                (math.sqrt(2.0) * chamfer, 2.0 * (well_r + chamfer), 0.0008),
                pos=tuple(float(v) for v in center),
                quat=_frame([slant_dir, tangent, in_up]),
                color=CRADLE_BLUE,
                friction=(0.12, 0.10),
                friction_combine="min",
            )
        )

    # Wera 355 PZ 2 x 25 mm stubby screwdriver in its local frame (+z_d = along blade to tip at +0.052 m)
    grip_w = P.DRV_GRIP_FLAT
    bolster_w = P.DRV_BOLSTER_FLAT
    driver_shapes = [
        # Uniform 24 mm 4-flat Kraftform grip body over z in [-0.027, +0.015] (clean release when jaws open)
        box((grip_w, grip_w, 0.042), pos=(0.0, 0.0, -0.006), color=WERA_BLACK, friction=mk.GRIPPY),
        box((grip_w * 0.92, grip_w * 0.92, 0.036), pos=(0.0, 0.0, -0.006), yaw=math.pi / 4.0, color=WERA_GREEN, solid=False),
        cylinder(grip_w * 0.46, 0.006, pos=(0.0, 0.0, -0.024), axis="z", color=WERA_GREEN, solid=False),
        # Hexagonal anti-roll bolster over z in [+0.015, +0.027] (starts 2.2 mm below the +12.8 mm fingertips; rests flat on cradle)
        box((bolster_w, bolster_w * 0.86, 0.012), pos=(0.0, 0.0, 0.021), color=WERA_BLACK, friction=(0.45, 0.40)),
        box((bolster_w, bolster_w * 0.86, 0.012), pos=(0.0, 0.0, 0.021), yaw=math.pi / 3.0, color=WERA_BLACK, solid=False),
        box((bolster_w, bolster_w * 0.86, 0.012), pos=(0.0, 0.0, 0.021), yaw=2.0 * math.pi / 3.0, color=WERA_BLACK, solid=False),
        # Bright orientation stripe on handle top butt so rotation is visible in top & wrist cameras
        box((0.018, 0.004, 0.001), pos=(0.0, 0.0, P.DRV_BUTT_Z - 0.0005), color=WERA_GREEN, solid=False),
        # Ø 6.0 mm steel shaft (z in [+0.027, +0.048]) + conical pointed PZ2 tip ending in 1.5 mm point at z = +0.052 m
        cylinder(P.DRV_BLADE_D / 2.0, 0.021, pos=(0.0, 0.0, 0.0375), axis="z", color=STEEL_BRIGHT, friction=(0.08, 0.06), friction_combine="min"),
        cylinder(0.0020, 0.0025, pos=(0.0, 0.0, 0.04925), axis="z", color=STEEL, friction=(0.08, 0.06), friction_combine="min"),
        mk.sphere(0.0015, pos=(0.0, 0.0, P.DRV_TIP_Z - 0.0015), color=STEEL, friction=(0.08, 0.06), friction_combine="min"),
        box((0.0050, 0.0012, 0.0035), pos=(0.0, 0.0, 0.0498), yaw=math.radians(P.DRV_BLADE_PHASE_DEG), color=STEEL, solid=False),
        box((0.0012, 0.0050, 0.0035), pos=(0.0, 0.0, 0.0498), yaw=math.radians(P.DRV_BLADE_PHASE_DEG), color=STEEL, solid=False),
    ]

    c_yaw = math.radians(P.CRADLE_YAW_DEG)
    drv_rot = mk.axis_angle((0.0, 0.0, 1.0), c_yaw) @ np.diag([1.0, -1.0, -1.0])
    for c in CONNS:
        cy = side_sign(c) * P.CRADLE_Y
        body(
            f"cradle_{c}",
            cradle_shapes,
            pos=(P.CRADLE_X, cy, top),
            yaw=c_yaw,
            kinematic=True,
            color=CRADLE_BLUE,
            friction=(0.30, 0.25),
        )
        body(
            f"driver_{c}",
            driver_shapes,
            pos=(P.CRADLE_X, cy, top + P.CRADLE_H + P.DRV_BOLSTER_Z + P.OVER),
            quat=_quat(drv_rot),
            mass=P.DRV_MASS,
            color=WERA_BLACK,
            friction=mk.GRIPPY,
            inertia=P.driver_inertia(),
            contacts=True,
        )


def build(top: float) -> None:
    build_cabinet_side(top)
    build_support_block(top)
    build_shelf(top)
    build_cams(top)
    build_cradles_and_drivers(top)
    for c in CONNS:
        mk.sense_contact(f"driver_{c}", "cam_l", "cam_r", "shelf", "cradle_l", "cradle_r")
        mk.sense_contact(f"cam_{c}", "cabinet_side")
    mk.sense_contact("shelf", "cabinet_side", "support_block")


_HULLS: dict = {}
_LINKS: dict = {}


def read_hand_points() -> dict[str, np.ndarray]:
    """World hull vertices of both grippers (``gripper``, ``tip_left``, ``tip_right``)."""
    from yam_arena import analysis

    if not _HULLS:
        for link in ("gripper", "tip_left", "tip_right"):
            _HULLS[link] = np.vstack([h.vertices for h in analysis.link_hulls().get(link, [])])
    base = mk.MECH.environment.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
    hands = {}
    for side in ("left", "right"):
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


def nearest_hand_clearance(hands: dict | None, names: tuple[str, ...]) -> float:
    """Minimum distance (m) from any hull vertex in ``hands`` to the bounding boxes of ``names``."""
    if not hands:
        return 0.0
    points = np.vstack([hands[s] for s in ("left", "right") if s in hands])
    best = math.inf
    for name in names:
        position, rotation = mk.pose(name)
        low, high = mk.MECH.bodies[name].bounds()
        local = (points - position) @ rotation
        gap = np.maximum(np.maximum(np.asarray(low) - local, local - np.asarray(high)), 0.0)
        best = min(best, float(np.linalg.norm(gap, axis=1).min()))
    return best

