"""Bodies and joints of the hole-saw quick-change arbor task, built from parts.py."""

from __future__ import annotations

import math

import mechkit as mk
import parts as P
from mechkit import body, box, capsule, cylinder, hinge, slide, weld

MECHKIT_HELPER = True  # declares into the task module that imports it

VISE_BLUE = (0.10, 0.26, 0.48)
JAW_STEEL = (0.58, 0.60, 0.64)
DARK_OXIDE = (0.14, 0.15, 0.17)
COLLAR_DARK = (0.17, 0.18, 0.21)
MACHINED = (0.72, 0.74, 0.78)
PIN_STEEL = (0.86, 0.88, 0.91)
PIN_TIP_RED = (0.90, 0.18, 0.12)
SAW_WHITE = (0.94, 0.94, 0.93)
MILWAUKEE_RED = (0.82, 0.11, 0.13)
PLATE_STEEL = (0.25, 0.27, 0.30)
PVC_BLACK = (0.09, 0.09, 0.10)
MAT_GREY = (0.28, 0.30, 0.33)
WHITE = (0.97, 0.97, 0.96)

HIDDEN_NAMES = ("carrier", "turn_a", "turn_b", "turner")
TURN_JOINTS = ("turn_1", "turn_2", "turn_3")
SLIP_STEEL = (0.18, 0.15)


def _staves(r_in: float, r_out: float, z_lo: float, z_hi: float, n: int, color, friction=mk.GRIPPY, solid: bool = True) -> list:
    """``n`` boxes forming a cylindrical ring from ``r_in`` to ``r_out`` and ``z_lo`` to ``z_hi``."""
    w = 2.0 * r_out * math.tan(math.pi / n) * 1.03
    r_mid = 0.5 * (r_in + r_out)
    z_mid = 0.5 * (z_lo + z_hi)
    out = []
    for k in range(n):
        a = 2.0 * math.pi * k / n
        out.append(
            box(
                (r_out - r_in, w, z_hi - z_lo),
                pos=(r_mid * math.cos(a), r_mid * math.sin(a), z_mid),
                yaw=a,
                color=color,
                solid=solid,
                friction=friction,
            )
        )
    return out


def build_fixtures(top: float) -> None:
    """Starting mat for the hole saw and the IRWIN 226340 4 in. drill-press vise bolted to the bench."""
    mt = P.MAT_SIZE[2]
    mat_shapes = [
        box(P.MAT_SIZE, color=MAT_GREY, friction=(0.7, 0.6)),
        # White +/-y alignment ticks on the mat edge matching the hole saw's +/-y pin-hole marks
        box((0.004, 0.016, 0.0006), pos=(0.0, 0.044, mt / 2 + 0.0003), color=WHITE, solid=False),
        box((0.004, 0.016, 0.0006), pos=(0.0, -0.044, mt / 2 + 0.0003), color=WHITE, solid=False),
    ]
    body("mat", mat_shapes, pos=(P.X_MAT, P.Y_MAT, top + mt / 2), color=MAT_GREY)

    # IRWIN 226340 4 in. Drill Press Vise: slotted base rails + fixed (-x) and movable (+x) jaws clamping the 7/16" hex shank
    bh, jh, jw, jt = P.VISE_BASE_H, P.VISE_JAW_H, P.VISE_JAW_W, P.VISE_JAW_T
    half_shank = P.ARBOR_SHANK_AF / 2  # 5.555 mm
    vise_shapes = [
        # Two main base guide rails along x and front/rear cross bridges (z = 0..14 mm)
        box((P.VISE_BASE_L, 0.024, bh), pos=(0.016, 0.039, bh / 2), color=VISE_BLUE),
        box((P.VISE_BASE_L, 0.024, bh), pos=(0.016, -0.039, bh / 2), color=VISE_BLUE),
        box((0.028, jw, bh), pos=(-0.059, 0.0, bh / 2), color=VISE_BLUE),
        box((0.032, jw, bh), pos=(0.089, 0.0, bh / 2), color=VISE_BLUE),
        # Slotted bolt-down mounting ears on +/-y (z = 0..8 mm)
        box((0.070, 0.016, 0.008), pos=(0.010, P.VISE_BASE_W / 2 - 0.008, 0.004), color=VISE_BLUE),
        box((0.070, 0.016, 0.008), pos=(0.010, -P.VISE_BASE_W / 2 + 0.008, 0.004), color=VISE_BLUE),
        # Fixed jaw (-x) and movable jaw (+x), 101.6 mm (4.0 in) wide, z = 14..38 mm
        box((jt - 0.004, jw, jh), pos=(-(half_shank + 0.004 + (jt - 0.004) / 2), 0.0, bh + jh / 2), color=VISE_BLUE),
        box((0.004, jw, jh), pos=(-(half_shank + 0.002), 0.0, bh + jh / 2), color=JAW_STEEL),
        box((0.004, jw, jh), pos=(half_shank + 0.002, 0.0, bh + jh / 2), color=JAW_STEEL),
        box((jt - 0.004, jw, jh), pos=(half_shank + 0.004 + (jt - 0.004) / 2, 0.0, bh + jh / 2), color=VISE_BLUE),
        # Leadscrew and T-handle pointing +x away from the robot arms
        cylinder(0.006, 0.095, pos=(0.078, 0.0, bh + jh * 0.45), axis="x", color=JAW_STEEL),
        cylinder(0.004, 0.076, pos=(0.126, 0.0, bh + jh * 0.45), axis="y", color=JAW_STEEL),
    ]
    body("vise", vise_shapes, pos=(P.X_ARBOR, P.Y_ARBOR, top), color=VISE_BLUE)


def build_arbor(top: float) -> None:
    """Milwaukee 49-56-7055 quick-change arbor: fixed shank/thread shaft + sliding 2-pin collar."""
    shank_h = P.Z_FLANGE_BOT - P.Z_ARBOR_BOT  # 0.048 m
    flange_h = P.Z_COLLAR_LOW - P.Z_FLANGE_BOT  # 0.004 m
    journal_h = P.Z_SHOULDER - P.Z_COLLAR_LOW  # 0.026 m
    thread_h = P.Z_THREAD_TOP - P.Z_SHOULDER  # 0.012 m
    pilot_r = P.PILOT_D / 2  # 0.003175 m
    # Capsule straight length so top hemisphere apex lands at Z_PILOT_TOP = 117.6 mm
    pilot_cyl = (P.Z_PILOT_TOP - P.Z_THREAD_TOP) - pilot_r  # 0.008425 m
    pilot_mid = P.Z_THREAD_TOP + pilot_cyl / 2

    arbor_shapes = [
        # 7/16" (11.11 mm) hex shank clamped in the vise jaws
        box((P.ARBOR_SHANK_AF, P.ARBOR_SHANK_AF * 0.58, shank_h), pos=(0.0, 0.0, P.Z_ARBOR_BOT + shank_h / 2), color=DARK_OXIDE),
        box((P.ARBOR_SHANK_AF, P.ARBOR_SHANK_AF * 0.58, shank_h), pos=(0.0, 0.0, P.Z_ARBOR_BOT + shank_h / 2), yaw=math.pi / 3, color=DARK_OXIDE),
        box((P.ARBOR_SHANK_AF, P.ARBOR_SHANK_AF * 0.58, shank_h), pos=(0.0, 0.0, P.Z_ARBOR_BOT + shank_h / 2), yaw=2 * math.pi / 3, color=DARK_OXIDE),
        # Lower flange stop under the collar
        cylinder(0.0145, flange_h, pos=(0.0, 0.0, P.Z_FLANGE_BOT + flange_h / 2), color=DARK_OXIDE),
        # Smooth guide journal inside the sliding collar + machined shoulder face at Z_SHOULDER
        cylinder(0.0100, journal_h - 0.002, pos=(0.0, 0.0, P.Z_COLLAR_LOW + (journal_h - 0.002) / 2), color=DARK_OXIDE),
        cylinder(0.0108, 0.002, pos=(0.0, 0.0, P.Z_SHOULDER - 0.001), color=MACHINED, friction=SLIP_STEEL, friction_combine="min"),
        # 5/8"-18 UNF external thread boss (15.5 mm solid guide cylinder inside 16.4 mm backplate bore)
        cylinder(0.00775, thread_h, pos=(0.0, 0.0, P.Z_SHOULDER + thread_h / 2), color=MACHINED, friction=SLIP_STEEL, friction_combine="min"),
        # Visual thread grooves
        *[cylinder(0.00805, 0.0007, pos=(0.0, 0.0, P.Z_SHOULDER + 0.0018 + 0.0022 * k), color=DARK_OXIDE, solid=False) for k in range(5)],
        # Rounded pilot guide nose above the thread boss (guides cup bore coaxially)
        capsule(pilot_r, pilot_cyl, pos=(0.0, 0.0, pilot_mid), color=MACHINED, friction=SLIP_STEEL, friction_combine="min"),
    ]
    body("arbor_base", arbor_shapes, pos=(P.X_ARBOR, P.Y_ARBOR, top), color=DARK_OXIDE)

    # Sliding 2-pin collar, built at retracted pose (collar_z = 0 -> world z = top + Z_COLLAR_LOW; reset puts it UP at +8 mm)
    pin_r = P.PIN_D / 2  # 0.003175 m
    pin_top_local = (P.Z_SHOULDER - P.Z_COLLAR_LOW) - P.PIN_RETRACT_BELOW  # 0.0250 m (-1.0 mm below shoulder when down, +7.0 mm when up)
    pin_cyl_l = 0.0100
    pin_mid_z = pin_top_local - pin_cyl_l / 2 - pin_r  # 0.016825 m
    pin_y = P.PIN_BCD / 2  # 0.0127 m (pins lie at +/-y so BOTH are unoccluded in top_cam!)

    collar_shapes = [
        # Knurled Ø 42 x 18 mm grip sleeve (slides coaxially around arbor_base along collar_z)
        cylinder(P.COLLAR_D / 2, P.COLLAR_H, pos=(0.0, 0.0, P.COLLAR_H / 2), color=COLLAR_DARK, passes=("arbor_base",), friction=mk.GRIPPY),
        # Two Ø 6.35 mm rounded-tip hardened steel drive pins on Ø 25.4 mm circle at (0, +/-12.7 mm)
        capsule(pin_r, pin_cyl_l, pos=(0.0, pin_y, pin_mid_z), color=PIN_STEEL, passes=("arbor_base",), friction=SLIP_STEEL, friction_combine="min"),
        capsule(pin_r, pin_cyl_l, pos=(0.0, -pin_y, pin_mid_z), color=PIN_STEEL, passes=("arbor_base",), friction=SLIP_STEEL, friction_combine="min"),
        # High-contrast red witness caps on pin tips (visual only, flush with pin apex) + white axial stripes on collar barrel
        cylinder(pin_r * 0.78, 0.0012, pos=(0.0, pin_y, pin_top_local - 0.0007), color=PIN_TIP_RED, solid=False),
        cylinder(pin_r * 0.78, 0.0012, pos=(0.0, -pin_y, pin_top_local - 0.0007), color=PIN_TIP_RED, solid=False),
        box((0.0025, 0.0010, P.COLLAR_H), pos=(0.0, P.COLLAR_D / 2 + 0.0004, P.COLLAR_H / 2), color=WHITE, solid=False),
        box((0.0025, 0.0010, P.COLLAR_H), pos=(0.0, -(P.COLLAR_D / 2 + 0.0004), P.COLLAR_H / 2), color=WHITE, solid=False),
    ]
    body("collar", collar_shapes, pos=(P.X_ARBOR, P.Y_ARBOR, top + P.Z_COLLAR_LOW), mass=P.COLLAR_MASS, color=COLLAR_DARK, contacts=True)
    slide(
        "collar_z", "collar", "arbor_base", axis=(0.0, 0.0, 1.0), limits=(0.0, P.PIN_STROKE),
        spring=P.COLLAR_K, rest=P.PIN_STROKE + P.COLLAR_BEYOND, damping=P.COLLAR_DAMP,
    )


def _backplate_boxes() -> list:
    """12 non-overlapping boxes forming the 5.0 mm steel backplate with center bore and two 7.8 mm pin holes at (0, +/-12.7 mm)."""
    t = P.PLATE_T
    zc = t / 2
    bx = P.BORE_CLEAR_HALF  # 0.0082 m
    hx = P.HOLE_D / 2  # 0.0039 m
    py = P.PIN_BCD / 2  # 0.0127 m
    y_in = py - hx  # 0.0088 m
    y_out = py + hx  # 0.0166 m
    eps = 0.00005  # 0.05 mm gap between adjacent boxes so zero shared faces exist

    def _pbox(size: tuple[float, float, float], pos: tuple[float, float, float]):
        return box(size, pos=pos, color=PLATE_STEEL, friction=SLIP_STEEL, friction_combine="min")

    out = []
    for sx in (-1.0, 1.0):
        # Inner and outer side chord slabs (|x| > 8.25 mm: solid steel blocking unaligned pins at e.g. 55 deg!)
        x1_lo, x1_hi = bx + eps, 0.0190
        out.append(_pbox((x1_hi - x1_lo, 0.0490, t), (sx * 0.5 * (x1_lo + x1_hi), 0.0, zc)))
        x2_lo, x2_hi = 0.0190 + eps, 0.0295
        out.append(_pbox((x2_hi - x2_lo, 0.0220, t), (sx * 0.5 * (x2_lo + x2_hi), 0.0, zc)))
    for sy in (-1.0, 1.0):
        # Narrow steel bridge between center thread bore (8.20 mm) and pin hole inner edge (8.80 mm)
        b_lo, b_hi = bx + eps, y_in - eps
        out.append(_pbox((2.0 * bx, b_hi - b_lo, t), (0.0, sy * 0.5 * (b_lo + b_hi), zc)))
        # Outer polar block from pin hole outer edge (16.65 mm) to cup wall (30.0 mm)
        p_lo, p_hi = y_out + eps, 0.0300
        out.append(_pbox((2.0 * bx, p_hi - p_lo, t), (0.0, sy * 0.5 * (p_lo + p_hi), zc)))
        # Left and right cheek blocks flanking each 7.8 mm pin hole along x
        c_lo, c_hi = hx, bx
        for sx in (-1.0, 1.0):
            out.append(_pbox((c_hi - c_lo, 2.0 * hx, t), (sx * 0.5 * (c_lo + c_hi), sy * py, zc)))
    return out


def hole_saw_inertia() -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """(center of mass, (Ixx, Iyy, Izz)) of the 195 g bi-metal cup + 5 mm steel backplate in hole_saw frame."""
    m_plate, m_wall = 0.090, P.SAW_MASS - 0.090
    r_out, r_in = P.SAW_OD / 2, P.SAW_OD / 2 - P.SAW_WALL_T
    z_p, z_w, h_w = P.PLATE_T / 2, P.PLATE_T + (P.SAW_H - P.PLATE_T) / 2, P.SAW_H - P.PLATE_T
    zc = (m_plate * z_p + m_wall * z_w) / P.SAW_MASS
    # Disc + thin hollow cylinder parallel-axis inertia about zc
    ixx_p = m_plate * (0.25 * r_out**2 + P.PLATE_T**2 / 12.0 + (z_p - zc) ** 2)
    izz_p = 0.5 * m_plate * r_out**2
    r2_sum = r_out**2 + r_in**2
    ixx_w = m_wall * (0.25 * r2_sum + h_w**2 / 12.0 + (z_w - zc) ** 2)
    izz_w = 0.5 * m_wall * r2_sum
    return (0.0, 0.0, zc), (ixx_p + ixx_w, ixx_p + ixx_w, izz_p + izz_w)


def build_hole_saw(top: float) -> None:
    """Milwaukee 49-56-0147 2-1/2 in Hole Dozer cup with 2x 7.8 mm backplate pin holes and black PVC tooth-rim guard."""
    r_out = P.SAW_OD / 2  # 0.03175 m
    r_in = r_out - P.SAW_WALL_T  # 0.02955 m
    z_guard_lo = P.SAW_H - 0.0034
    py = P.PIN_BCD / 2

    shapes = [
        *_backplate_boxes(),
        # Cylindrical white Hole Dozer barrel (z = 5.0 .. 44.2 mm, grippy for robot finger pads)
        *_staves(r_in, r_out, P.PLATE_T, z_guard_lo, 16, SAW_WHITE, friction=mk.GRIPPY),
        # Smooth black PVC U-channel tooth-rim guard (z = 44.2 .. 47.6 mm, protects shared rig finger pads)
        *_staves(r_in - 0.0006, P.GUARD_OD / 2, z_guard_lo, P.SAW_H, 16, PVC_BLACK, friction=mk.GRIPPY),
        # Visual Milwaukee red outer band (hollow ring so cup interior stays completely open!) + +/-y alignment stripes
        *_staves(r_out, r_out + 0.0005, 0.0215, 0.0265, 16, MILWAUKEE_RED, solid=False),
        box((0.0036, 0.0014, P.SAW_H - 0.004), pos=(0.0, r_out + 0.0006, P.SAW_H / 2), color=MILWAUKEE_RED, solid=False),
        box((0.0036, 0.0014, P.SAW_H - 0.004), pos=(0.0, -(r_out + 0.0006), P.SAW_H / 2), color=MILWAUKEE_RED, solid=False),
        # Bright white alignment ticks on the dark inner backplate floor flanking both (0, +/-12.7 mm) pin holes
        box((0.0038, 0.0065, 0.0008), pos=(0.0, py + 0.0078, P.PLATE_T + 0.0004), color=WHITE, solid=False),
        box((0.0038, 0.0065, 0.0008), pos=(0.0, -(py + 0.0078), P.PLATE_T + 0.0004), color=WHITE, solid=False),
    ]
    com, inertia = hole_saw_inertia()
    z_start = top + P.MAT_SIZE[2] + P.OVER
    body(
        "hole_saw", shapes, pos=(P.X_MAT, P.Y_MAT, z_start), yaw=0.0, mass=P.SAW_MASS,
        color=SAW_WHITE, friction=mk.GRIPPY, center=com, inertia=inertia, contacts=True,
    )


def build_helical_carrier(top: float) -> None:
    """Hidden serial helical carrier (cup_z slide + 3x <=138.3 deg hinges + w_thread weld) at Z_ENTRY."""
    entry_at = (P.X_ARBOR, P.Y_ARBOR, top + P.Z_ENTRY)
    all_bodies = ("mat", "vise", "arbor_base", "collar", "hole_saw") + HIDDEN_NAMES
    for idx, name in enumerate(HIDDEN_NAMES):
        ghost = [box((0.008, 0.008, 0.004), pos=(0.0, 0.0, -0.050 - 0.006 * idx), color=DARK_OXIDE, passes=all_bodies)]
        body(name, ghost, pos=entry_at, mass=P.HIDDEN_MASS, color=DARK_OXIDE)

    slide(
        "cup_z", "carrier", "arbor_base", anchor=entry_at, axis=(0.0, 0.0, 1.0),
        limits=(-(P.Z_ENTRY - P.Z_SHOULDER) - 0.0015, 0.0035), damping=12.0,
    )
    # Three coaxial hinges in series about -z (CW from above = positive), each <= 138.33 deg (< 180 deg PhysX clamp) -> +415.0 deg hard stop!
    for j_name, child, parent in (("turn_1", "turn_a", "carrier"), ("turn_2", "turn_b", "turn_a"), ("turn_3", "turner", "turn_b")):
        hinge(
            j_name, child, parent, anchor=entry_at, axis=(0.0, 0.0, -1.0),
            limits=(P.HINGE_LOWER_RAD, P.HINGE_UPPER_RAD),
            spring=P.THREAD_SPRING_K, rest=0.0, damping=P.THREAD_DAMP,
        )
    # Weld anchored at hole_saw's own origin (zero lever arm); made when lowered coaxially onto retracted arbor, cut if unthreaded < 0 deg
    weld("w_thread", "hole_saw", "turner", made=False)
    mk.sense_contact("collar", "hole_saw")


def build(top: float) -> None:
    build_fixtures(top)
    build_arbor(top)
    build_hole_saw(top)
    build_helical_carrier(top)
