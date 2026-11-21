"""Bodies of the bathroom-sink pop-up stopper task, built from ``parts.py`` in pure 240 Hz PhysX.

Four bodies, zero joints/welds/pushes:
- ``vanity`` (fixed): open-front birch-plywood cabinet carcass with full-height side & rear walls, front right stone
  counter shelf, front stone basin surround, full-width ``160 x 424 x 9 mm`` clear cast-acrylic rear L-roof (solid in
  PhysX, transparent to ``top_cam``), solvent-welded ``5 mm`` white ABS drop-in basin (``110 x 190 x 42 mm`` outer,
  ``100 x 180 x 38 mm`` inner), dark mat, and rear U-cradle.
- ``drain`` (kinematic fixed, for 240 Hz contact-force sensing): 16-stave ``1-1/4 in`` bore, top chrome flange, rear
  ``+x`` pivot-port boss, ball-start rails, compliant cone-washer beads (``soften``), and 4-slot PETG snap-collar.
- ``stopper`` (free, 36 g): Danco 11041 chrome head, X-fin bore guides, single open ``8.5 x 10.0 mm`` clevis eye with
  rounded capsule crossbars, and taped lower clevis shaft; starts resting closed at ``yaw = +90 deg`` (eye facing ``y``).
- ``pivot_rod`` (free, 44 g): Danco 88532 ``18.3 mm`` ball, ``27.5 mm`` rounded gray inner tip, ``150 mm x 5.5 mm``
  steel rear rod, brass clevis connector, and external red collar-seating band; starts retracted ``32 mm`` along ``+x``.
"""

from __future__ import annotations

import math

import mechkit as mk
import parts as P
from mechkit import body, box, capsule, cylinder, soften, sphere

MECHKIT_HELPER = True

BIRCH = (0.85, 0.73, 0.54)
DECK_STONE = (0.76, 0.74, 0.70)
BASIN_WHITE = (0.95, 0.96, 0.98)
MAT_DARK = (0.13, 0.15, 0.18)
ABS_DARK = (0.20, 0.22, 0.25)
COLLAR_LIGHT = (0.86, 0.88, 0.90)
CHROME = (0.68, 0.71, 0.76)
STOPPER_CAP = (0.36, 0.40, 0.46)
SEAL_RUBBER = (0.12, 0.12, 0.13)
BALL_WHITE = (0.96, 0.96, 0.94)
TIP_GREY = (0.52, 0.54, 0.57)
STEEL_ZINC = (0.92, 0.77, 0.28)
MARK_RED = (0.90, 0.16, 0.14)
CRADLE_COL = (0.30, 0.32, 0.36)
ACRYLIC_EDGE = (0.64, 0.80, 0.86)
SLIP = (0.18, 0.15)
ACRYLIC_PANE_SHAPES: tuple[int, ...] = ()


def build_vanity(top: float) -> None:
    """Open-front vanity cabinet with full-depth counter (stone + full-width clear acrylic rear L-panel), ABS basin, and U-cradle."""
    global ACRYLIC_PANE_SHAPES
    t = P.PLY_T
    deck_t = P.ACRYLIC_T  # 9.0 mm stone & clear cast-acrylic deck thickness
    full_d = P.CABINET_X1 - P.VANITY_X0
    full_cx = 0.5 * (P.VANITY_X0 + P.CABINET_X1)
    rear_in_x = P.CABINET_X1 - t  # 0.604 m inner face of rear birch wall
    y_in_r, y_in_l = P.Y_RIGHT + t, P.Y_LEFT - t  # -0.237 m, +0.187 m
    in_w = y_in_l - y_in_r
    in_cy = 0.5 * (y_in_r + y_in_l)

    bx0, bx1 = P.BASIN_X
    by0, by1 = P.BASIN_Y
    bw = P.BASIN_WALL  # 0.005 m white ABS sheet
    step_y = by0 - bw  # -0.023 m outer right face of drop-in basin
    h_wall_r = P.RIGHT_SHELF_TOP - deck_t  # 0.195 m clear interior height under 20.4 cm shelf & rear clear acrylic roof
    h_wall_l = P.VANITY_TOP - deck_t  # 0.163 m under sunken 17.2 cm front basin surround
    front_d = P.DECK_SPLIT_X - P.VANITY_X0
    front_cx = 0.5 * (P.VANITY_X0 + P.DECK_SPLIT_X)
    rear_d = rear_in_x - P.DECK_SPLIT_X
    rear_cx = 0.5 * (P.DECK_SPLIT_X + rear_in_x)

    shapes = [
        # Full-depth, full-height birch side walls (front left stepped at 17.2 cm; rear left & entire right up to 20.4 cm)
        box((full_d, t, h_wall_r), pos=(full_cx, P.Y_RIGHT + t / 2, h_wall_r / 2), color=BIRCH),
        box((front_d, t, h_wall_l), pos=(front_cx, P.Y_LEFT - t / 2, h_wall_l / 2), color=BIRCH),
        box((rear_d, t, h_wall_r), pos=(rear_cx, P.Y_LEFT - t / 2, h_wall_r / 2), color=BIRCH),
        # Full-height 18 mm rear birch back wall (up to 20.4 cm across full 46.0 cm width)
        box((t, P.Y_LEFT - P.Y_RIGHT, P.RIGHT_SHELF_TOP), pos=(P.CABINET_X1 - t / 2, 0.5 * (P.Y_RIGHT + P.Y_LEFT), P.RIGHT_SHELF_TOP / 2), color=BIRCH),
        # Dark floor contrast mat (drawn only, so z = 0 benchtop stays true)
        box((full_d - t - 0.004, in_w - 0.004, 0.0012), pos=(0.5 * (P.VANITY_X0 + rear_in_x), in_cy, 0.0006), color=MAT_DARK, solid=False),
    ]

    # Full-depth countertop: sunken front stone basin surround (x = 0.334..0.444, z = 0.172), front right stone shelf
    # (x = 0.334..0.444, z = 0.204), side stone caps, and full-width 160 x 424 x 9 mm clear cast-acrylic rear L-roof at z = 0.204
    dz_l = P.VANITY_TOP - deck_t / 2
    dz_r = P.RIGHT_SHELF_TOP - deck_t / 2
    left_w = P.Y_LEFT - step_y
    left_cy = 0.5 * (step_y + P.Y_LEFT)
    right_w = step_y - P.Y_RIGHT
    right_cy = 0.5 * (P.Y_RIGHT + step_y)
    riser_h = P.RIGHT_SHELF_TOP - P.VANITY_TOP  # 32 mm step above z = 0.172 m
    riser_z = P.VANITY_TOP + riser_h / 2
    shapes += [
        box((bx0 - P.VANITY_X0, left_w, deck_t), pos=(0.5 * (P.VANITY_X0 + bx0), left_cy, dz_l), color=DECK_STONE),
        box((bx1 - bx0, P.Y_LEFT - by1, deck_t), pos=(0.5 * (bx0 + bx1), 0.5 * (by1 + P.Y_LEFT), dz_l), color=DECK_STONE),
        box((rear_d, t, deck_t), pos=(rear_cx, P.Y_LEFT - t / 2, dz_r), color=DECK_STONE),
        box((rear_d, t, deck_t), pos=(rear_cx, P.Y_RIGHT + t / 2, dz_r), color=DECK_STONE),
        box((front_d, right_w, deck_t), pos=(front_cx, right_cy, dz_r), color=DECK_STONE),
        box((front_d, 0.008, riser_h), pos=(front_cx, step_y - 0.004, riser_z), color=BIRCH),
    ]

    # Full-width clear cast-acrylic rear L-panel (160 x 424 x 9 mm roof at z = 0.195..0.204 m + 23 mm front step strip at x = 0.444 m)
    ax0, ax1 = P.DECK_SPLIT_X, rear_in_x  # 0.444..0.604 m (160 mm)
    ay0, ay1 = y_in_r, y_in_l  # -0.237..+0.187 m (424 mm full interior width)
    aw_x, aw_y = ax1 - ax0, ay1 - ay0
    acx, acy = 0.5 * (ax0 + ax1), 0.5 * (ay0 + ay1)
    bevel = 0.0045
    shapes += [
        box((bevel, aw_y, 0.0020), pos=(ax0 + bevel / 2, acy, P.RIGHT_SHELF_TOP - 0.0010), color=ACRYLIC_EDGE, solid=False),
        box((bevel, aw_y, 0.0020), pos=(ax1 - bevel / 2, acy, P.RIGHT_SHELF_TOP - 0.0010), color=ACRYLIC_EDGE, solid=False),
        box((aw_x, bevel, 0.0020), pos=(acx, ay0 + bevel / 2, P.RIGHT_SHELF_TOP - 0.0010), color=ACRYLIC_EDGE, solid=False),
        box((aw_x, bevel, 0.0020), pos=(acx, ay1 - bevel / 2, P.RIGHT_SHELF_TOP - 0.0010), color=ACRYLIC_EDGE, solid=False),
    ]
    i0 = len(shapes)
    step_strip_h = h_wall_r - P.VANITY_TOP  # 23 mm vertical clear-acrylic front lip sealing z = 0.172..0.195 m behind basin
    shapes.append(box((aw_x, aw_y, deck_t), pos=(acx, acy, dz_r), color=ACRYLIC_EDGE, solid=True))
    shapes.append(box((deck_t, y_in_l - step_y, step_strip_h), pos=(ax0 + deck_t / 2, 0.5 * (step_y + y_in_l), P.VANITY_TOP + step_strip_h / 2), color=ACRYLIC_EDGE, solid=True))
    ACRYLIC_PANE_SHAPES = (i0, i0 + 1)

    # Solvent-welded 5 mm white ABS drop-in basin (110 x 190 x 42 mm outer, 100 x 180 x 38 mm inner, 40 x 40 mm drain hole)
    bh = P.VANITY_TOP - P.BASIN_UNDER
    bz = P.BASIN_UNDER + bh / 2
    shapes += [
        box((bw, (by1 - by0) + 2 * bw, bh), pos=(bx0 - bw / 2, 0.5 * (by0 + by1), bz), color=BASIN_WHITE),
        box((bw, (by1 - by0) + 2 * bw, bh), pos=(bx1 + bw / 2, 0.5 * (by0 + by1), bz), color=BASIN_WHITE),
        box((bx1 - bx0, bw, bh), pos=(0.5 * (bx0 + bx1), by0 - bw / 2, bz), color=BASIN_WHITE),
        box((bx1 - bx0, bw, bh), pos=(0.5 * (bx0 + bx1), by1 + bw / 2, bz), color=BASIN_WHITE),
    ]
    ft = max(0.004, P.BASIN_FLOOR - P.BASIN_UNDER)
    fz = P.BASIN_FLOOR - ft / 2
    hole = 0.020
    hx0, hx1 = P.X_DRAIN - hole, P.X_DRAIN + hole
    hy0, hy1 = P.Y_DRAIN - hole, P.Y_DRAIN + hole
    shapes += [
        box((hx0 - bx0, by1 - by0, ft), pos=(0.5 * (bx0 + hx0), 0.5 * (by0 + by1), fz), color=BASIN_WHITE),
        box((bx1 - hx1, by1 - by0, ft), pos=(0.5 * (hx1 + bx1), 0.5 * (by0 + by1), fz), color=BASIN_WHITE),
        box((hx1 - hx0, hy0 - by0, ft), pos=(0.5 * (hx0 + hx1), 0.5 * (by0 + hy0), fz), color=BASIN_WHITE),
        box((hx1 - hx0, by1 - hy1, ft), pos=(0.5 * (hx0 + hx1), 0.5 * (hy1 + by1), fz), color=BASIN_WHITE),
    ]

    # Rear rod start U-cradle (x = 0.563..0.590 m: supports retracted tail, clears seated tail at x = 0.556 m)
    cx0, cx1 = P.CRADLE_X
    clen, cmid = cx1 - cx0, 0.5 * (cx0 + cx1)
    notch_z = P.PORT_Z - P.ROD_D / 2
    shapes.append(box((clen, 0.020, notch_z), pos=(cmid, P.Y_DRAIN, notch_z / 2), color=CRADLE_COL, friction=SLIP, friction_combine="min"))
    for s in (-1.0, 1.0):
        shapes.append(box((clen, 0.004, 0.010), pos=(cmid, P.Y_DRAIN + s * 0.0068, notch_z + 0.003), color=CRADLE_COL, friction=SLIP, friction_combine="min"))

    body("vanity", shapes, pos=(0.0, 0.0, top), mass=None, color=BIRCH)


def build_drain(top: float) -> None:
    """1-1/4 in vertical drain bore, top flange, and rear +x pivot port with compliant cone-washer & split snap-collar."""
    shapes = []
    n_bore = 16
    r_in, r_out = P.DRAIN_ID / 2, P.DRAIN_OD / 2
    wall = r_out - r_in
    r_mid = 0.5 * (r_in + r_out)
    chord = 2.0 * r_out * math.tan(math.pi / n_bore)
    z_lo, z_hi = 0.002, P.BASIN_FLOOR
    win_lo, win_hi = P.PORT_Z - 0.011, P.PORT_Z + 0.011

    for k in range(n_bore):
        phi = 2.0 * math.pi * k / n_bore
        cx, cy = r_mid * math.cos(phi), r_mid * math.sin(phi)
        if k in (0, 1, n_bore - 1):  # open rear +x port window into the bore
            h1 = win_lo - z_lo
            h2 = z_hi - win_hi
            shapes.append(box((wall, chord, h1), pos=(cx, cy, z_lo + h1 / 2), yaw=phi, color=ABS_DARK, friction=SLIP, friction_combine="min"))
            shapes.append(box((wall, chord, h2), pos=(cx, cy, win_hi + h2 / 2), yaw=phi, color=ABS_DARK, friction=SLIP, friction_combine="min"))
        else:
            h = z_hi - z_lo
            shapes.append(box((wall, chord, h), pos=(cx, cy, z_lo + h / 2), yaw=phi, color=ABS_DARK, friction=SLIP, friction_combine="min"))

    # Top chrome flange seat (top surface at z = FLANGE_Z = 0.136 m)
    fl_t = 0.0025
    fl_r_out = P.FLANGE_D / 2
    fl_w = fl_r_out - r_in
    fl_rmid = 0.5 * (r_in + fl_r_out)
    fl_chord = 2.0 * fl_r_out * math.tan(math.pi / n_bore)
    for k in range(n_bore):
        phi = 2.0 * math.pi * k / n_bore
        shapes.append(
            box((fl_w, fl_chord, fl_t), pos=(fl_rmid * math.cos(phi), fl_rmid * math.sin(phi), P.FLANGE_Z - fl_t / 2), yaw=phi, color=CHROME, friction=SLIP, friction_combine="min")
        )

    # Rear +x port outer barrel (world x = 0.3935..0.4170 m; X_COLLAR = 0.4170 m)
    x0, x1 = 0.0185, P.X_COLLAR - P.X_DRAIN  # 0.0185..0.0420 m
    blen, bmid = x1 - x0, 0.5 * (x0 + x1)
    n_port = 8
    pr_in, pr_out = 0.0098, 0.0132
    pw = pr_out - pr_in
    pr_mid = 0.5 * (pr_in + pr_out)
    pchord = 2.0 * pr_out * math.tan(math.pi / n_port)
    for k in range(n_port):
        ang = 2.0 * math.pi * k / n_port
        cy, cz = pr_mid * math.cos(ang), P.PORT_Z + pr_mid * math.sin(ang)
        q = tuple(float(v) for v in mk.matrix_to_quat(mk.axis_angle((1.0, 0.0, 0.0), ang)))
        shapes.append(box((blen - 0.003, pw, pchord), pos=(bmid - 0.0015, cy, cz), quat=q, color=ABS_DARK))
        shapes.append(box((0.003, pw + 0.001, pchord), pos=(x1 - 0.0015, cy, cz), quat=q, color=COLLAR_LIGHT))

    # Front hard ball-seat stop at x_local = 0.0193 m (stops R = 9.15 mm sphere cleanly at X_SEAT = 0.4010 m)
    sr_in, sr_out = 0.0062, 0.0098
    sw, sr_mid = sr_out - sr_in, 0.5 * (sr_in + sr_out)
    schord = 2.0 * sr_out * math.tan(math.pi / n_port)
    for k in range(n_port):
        ang = 2.0 * math.pi * k / n_port
        q = tuple(float(v) for v in mk.matrix_to_quat(mk.axis_angle((1.0, 0.0, 0.0), ang)))
        shapes.append(box((0.0016, sw, schord), pos=(0.0185, sr_mid * math.cos(ang), P.PORT_Z + sr_mid * math.sin(ang)), quat=q, color=ABS_DARK, friction=SLIP, friction_combine="min"))

    # 4 compliant rounded elastomer cone-washer beads around X_SEAT (0.85 mm radial interference -> ~0.43 N m hold, smooth lead-in)
    r_pad = P.BALL_R - P.WASHER_INTERFERENCE
    pad_r = 0.0020
    for dy, dz in ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)):
        off = r_pad + pad_r
        pad = sphere(pad_r, pos=(P.SEAT_OFFSET_X, dy * off, P.PORT_Z + dz * off), color=SEAL_RUBBER, friction=(1.2, P.WASHER_MU), friction_combine="max")
        shapes.append(soften(pad, P.WASHER_K, P.WASHER_DAMP))

    # 4 compliant rounded split-collar snap beads 5.5 mm behind X_SEAT (~8-10 N snap-in click; pop clear behind seated equator)
    lip_r = 0.0018
    x_lip = P.SEAT_OFFSET_X + P.LIP_X_FROM_SEAT
    for dy, dz in ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)):
        off = P.LIP_R + lip_r
        lip = sphere(lip_r, pos=(x_lip, dy * off, P.PORT_Z + dz * off), color=COLLAR_LIGHT, friction=(P.LIP_MU, P.LIP_MU), friction_combine="min")
        shapes.append(soften(lip, P.LIP_K, P.LIP_DAMP))

    # Two parallel ball-start rails (13.0 mm gap: 18.3 mm unseated ball rests at z = PORT_Z; 5.5 mm seated rod drops between them)
    rail_half_gap = 0.0065
    rail_w = 0.0034
    rail_top = P.PORT_Z - math.sqrt(P.BALL_R**2 - rail_half_gap**2)
    rx0, rx1 = x1, (P.X_BALL_START - P.X_DRAIN) + 0.006
    for s in (-1.0, 1.0):
        shapes.append(
            box((rx1 - rx0, rail_w, 0.0050), pos=(0.5 * (rx0 + rx1), s * (rail_half_gap + rail_w / 2), rail_top - 0.0025), color=ABS_DARK, friction=SLIP, friction_combine="min")
        )

    body("drain", shapes, pos=(P.X_DRAIN, P.Y_DRAIN, top), kinematic=True, color=ABS_DARK)


def build_stopper(top: float) -> None:
    """Danco 11041 chrome pop-up stopper (origin at cap underside seal z = 0; starts closed with yaw = pi/2)."""
    r_head = P.STOP_HEAD_D / 2
    upper_top = -0.0010
    eye_top_outer = -(P.EYE_DEPTH - P.EYE_H / 2 - 0.0032)
    upper_len = upper_top - eye_top_outer
    upper_mid = 0.5 * (upper_top + eye_top_outer)

    eye_bot_outer = -(P.EYE_DEPTH + P.EYE_H / 2 + 0.0032)
    lower_len = eye_bot_outer - (-P.STOP_BELOW_SEAL)
    lower_mid = 0.5 * (eye_bot_outer - P.STOP_BELOW_SEAL)

    bar_r = 0.0016
    side_h = P.EYE_H + 4.0 * bar_r
    side_y = P.EYE_W / 2 + P.EYE_WALL / 2

    shapes = [
        # Cap seal + grippy dark-chrome cap head (12.5 mm total above seal; contrasts against white basin)
        cylinder(r_head - 0.0008, 0.0025, pos=(0.0, 0.0, 0.00125), color=SEAL_RUBBER, friction=SLIP, friction_combine="min"),
        cylinder(r_head, 0.0075, pos=(0.0, 0.0, 0.00625), color=STOPPER_CAP, friction=mk.GRIPPY),
        cylinder(r_head - 0.0022, 0.0025, pos=(0.0, 0.0, 0.01125), color=STOPPER_CAP, friction=mk.GRIPPY),
        # Upper X-fin guide stem inside 31.8 mm bore
        box((P.GUIDE_SPAN, P.FIN_THICK, upper_len), pos=(0.0, 0.0, upper_mid), color=CHROME, friction=SLIP, friction_combine="min"),
        box((P.FIN_THICK, P.GUIDE_SPAN, upper_len), pos=(0.0, 0.0, upper_mid), color=CHROME, friction=SLIP, friction_combine="min"),
        # Single open 8.5 x 10.0 mm clevis eye (hole along local x; smooth capsule top/bottom bars + vertical side cheeks)
        capsule(bar_r, P.EYE_W, pos=(0.0, 0.0, -(P.EYE_DEPTH - P.EYE_H / 2 - bar_r)), axis="y", color=CHROME, friction=SLIP, friction_combine="min"),
        capsule(bar_r, P.EYE_W, pos=(0.0, 0.0, -(P.EYE_DEPTH + P.EYE_H / 2 + bar_r)), axis="y", color=CHROME, friction=SLIP, friction_combine="min"),
        box((P.EYE_THICK, P.EYE_WALL, side_h), pos=(0.0, side_y, -P.EYE_DEPTH), color=CHROME, friction=SLIP, friction_combine="min"),
        box((P.EYE_THICK, P.EYE_WALL, side_h), pos=(0.0, -side_y, -P.EYE_DEPTH), color=CHROME, friction=SLIP, friction_combine="min"),
        # Lower taped clevis shaft + lower bore centring cross (total depth 119.6 mm below seal)
        box((P.EYE_THICK, P.EYE_W + 2 * P.EYE_WALL, lower_len), pos=(0.0, 0.0, lower_mid), color=SEAL_RUBBER, friction=SLIP, friction_combine="min"),
        box((P.GUIDE_SPAN, P.FIN_THICK, 0.016), pos=(0.0, 0.0, -P.STOP_BELOW_SEAL + 0.010), color=CHROME, friction=SLIP, friction_combine="min"),
    ]
    com, inertia = P.stopper_com_and_inertia()
    body(
        "stopper",
        shapes,
        pos=(P.X_DRAIN, P.Y_DRAIN, top + P.FLANGE_Z + P.OVER),
        yaw=math.pi / 2.0,
        mass=P.STOP_MASS,
        color=CHROME,
        center=com,
        inertia=inertia,
        contacts=True,
    )


def build_pivot_rod(top: float) -> None:
    """Danco 88532 ball-pivot rod (origin at 18.3 mm ball centre; starts retracted at X_BALL_START = 0.433 m)."""
    stub_back = -0.0050
    stub_front = -P.TIP_REACH  # -0.03665 m exact
    stub_cyl = (stub_back - stub_front) - 2.0 * P.TIP_R
    stub_cx = 0.5 * (stub_back + stub_front)
    steel_len = P.ROD_OUTER_L - P.ROD_THREAD_IN  # 0.1460 m
    steel_cx = P.BALL_R + steel_len / 2.0
    mark_offset = (P.X_COLLAR - P.X_SEAT) + 0.0020  # +0.0180 m: front edge at +0.0160 m touches X_COLLAR when seated

    shapes = [
        sphere(P.BALL_R, pos=(0.0, 0.0, 0.0), color=BALL_WHITE, friction=(1.0, 0.95)),
        capsule(P.TIP_R, stub_cyl, pos=(stub_cx, 0.0, 0.0), axis="x", color=TIP_GREY, friction=SLIP, friction_combine="min"),
        cylinder(P.ROD_D / 2.0, steel_len, pos=(steel_cx, 0.0, 0.0), axis="x", color=STEEL_ZINC, friction=mk.GRIPPY),
        box(P.CLEVIS_BLOCK, pos=(P.CLEVIS_AT_X, 0.0, 0.0), color=STEEL_ZINC, friction=mk.GRIPPY),
        cylinder(P.ROD_D / 2.0 + 0.0005, 0.0045, pos=(mark_offset, 0.0, 0.0), axis="x", color=MARK_RED, solid=False),
    ]
    com, inertia = P.rod_com_and_inertia()
    body(
        "pivot_rod",
        shapes,
        pos=(P.X_BALL_START, P.Y_DRAIN, top + P.PORT_Z + P.OVER),
        mass=P.ROD_MASS,
        color=STEEL_ZINC,
        center=com,
        inertia=inertia,
        contacts=True,
    )


def build(top: float) -> None:
    build_vanity(top)
    build_drain(top)
    build_stopper(top)
    build_pivot_rod(top)
    mk.sense_contact("stopper", "drain", "pivot_rod")
    mk.sense_contact("pivot_rod", "drain", "stopper")
