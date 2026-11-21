"""Scene bodies and joints for the lab scissor-jack pH-probe task, built exclusively from ``parts.py``."""

from __future__ import annotations

import math

import mechkit as mk
import parts as P
from mechkit import body, box, cylinder, hinge, slide, sphere

MECHKIT_HELPER = True

STEEL = (0.72, 0.75, 0.78)
STEEL_DARK = (0.40, 0.43, 0.47)
RUBBER_DARK = (0.14, 0.14, 0.15)
KNOB_BLACK = (0.11, 0.11, 0.12)
WHITE = (0.96, 0.96, 0.96)
PP_WHITE = (0.92, 0.93, 0.95)
MAT_BLUE = (0.14, 0.40, 0.78)
SAMPLE_BLUE = (0.12, 0.36, 0.82)
BAND_GREEN = (0.08, 0.70, 0.26)
COASTER_SLATE = (0.26, 0.28, 0.31)
PEI_DARK = (0.16, 0.17, 0.20)
HANNA_ORANGE = (0.92, 0.48, 0.10)
TIP_CYAN = (0.20, 0.74, 0.88)
STAND_BASE = (0.22, 0.23, 0.25)


def _beaker_shapes(*, green_band: bool) -> list:
    """Compound hollow Griffin 50 mL beaker (16 wall staves + 2 mm floor disc), origin at underside centre."""
    shapes = [
        cylinder(P.BEAKER_R_OUT, P.BEAKER_FLOOR, pos=(0.0, 0.0, P.BEAKER_FLOOR / 2.0), color=PP_WHITE, friction=(0.45, 0.40)),
    ]
    n = P.BEAKER_STAVES
    r_mid = P.BEAKER_R_OUT - P.BEAKER_WALL / 2.0
    stave_w = 2.0 * r_mid * math.tan(math.pi / n) + 0.0006
    wall_h = P.BEAKER_H - P.BEAKER_FLOOR
    z_mid = P.BEAKER_FLOOR + wall_h / 2.0
    band_h = P.IMM_MAX - P.IMM_MIN
    z_band = P.BEAKER_H - 0.5 * (P.IMM_MIN + P.IMM_MAX)  # 30.0 mm above floor == 20..40 mm below 60 mm rim
    for k in range(n):
        a = 2.0 * math.pi * k / n
        ca, sa = math.cos(a), math.sin(a)
        shapes.append(
            box((P.BEAKER_WALL, stave_w, wall_h), pos=(r_mid * ca, r_mid * sa, z_mid), yaw=a, color=PP_WHITE, friction=mk.GRIPPY)
        )
        if green_band:
            for r_vis in (P.BEAKER_R_OUT + 0.0004, P.BEAKER_R_IN - 0.0004):
                shapes.append(
                    box((0.0007, stave_w, band_h), pos=(r_vis * ca, r_vis * sa, z_band), yaw=a, color=BAND_GREEN, solid=False, glow=0.12)
                )
        else:
            r_vis = P.BEAKER_R_OUT + 0.0004
            shapes.append(
                box((0.0007, stave_w, 0.007), pos=(r_vis * ca, r_vis * sa, 0.011), yaw=a, color=SAMPLE_BLUE, solid=False)
            )
    return shapes


def build_stand_and_probe(top: float) -> None:
    """Fixed retort support stand behind the jack + Hanna HI1230B electrode on lateral and axial witness springs."""
    sx, sy = P.STAND_POST_XY
    px, py = P.JACK_XY
    arm_len = sx - px
    shapes_stand = [
        box((0.085, 0.140, 0.012), pos=(0.012, 0.0, 0.006), color=STAND_BASE),
        cylinder(0.006, 0.295, pos=(0.0, 0.0, 0.012 + 0.1475), color=STEEL),
        box((0.024, 0.024, 0.024), pos=(0.0, 0.0, P.Z_CLAMP_ANCHOR), color=STAND_BASE),
        cylinder(0.005, arm_len, pos=(-arm_len / 2.0, 0.0, P.Z_CLAMP_ANCHOR), axis="x", color=STEEL),
        cylinder(0.012, 0.018, pos=(-arm_len, 0.0, P.Z_CLAMP_ANCHOR), color=STAND_BASE, solid=False),
        # Fixed pH meter display unit to the rear-left of the retort stand, clear of both arms
        box((0.080, 0.065, 0.036), pos=(P.METER_XY[0] - sx, P.METER_XY[1] - sy, 0.018), color=STAND_BASE, solid=False),
        box((0.044, 0.036, 0.003), pos=(P.METER_XY[0] - sx - 0.008, P.METER_XY[1] - sy, 0.037), color=BAND_GREEN, solid=False, glow=0.25),
        cylinder(0.002, 0.095, pos=(0.0, 0.048, 0.024), axis="y", color=KNOB_BLACK, solid=False),
    ]
    body("stand", shapes_stand, pos=(sx, sy, top), kinematic=True, color=STAND_BASE)

    anchor = (px, py, top + P.Z_CLAMP_ANCHOR)
    body("probe_cx", [sphere(0.005, solid=True, passes=("stand", "probe_cy", "probe"))], pos=anchor, mass=0.020, color=STAND_BASE)
    body("probe_cy", [sphere(0.005, solid=True, passes=("stand", "probe_cx", "probe"))], pos=anchor, mass=0.020, color=STAND_BASE)

    # Probe origin at the exact sensing tip (z = 0 in probe frame == top + Z_PROBE_TIP in world)
    probe_shapes = [
        sphere(P.PROBE_R, pos=(0.0, 0.0, P.PROBE_R), color=TIP_CYAN),
        cylinder(P.PROBE_R, P.PROBE_SHAFT_L - P.PROBE_R, pos=(0.0, 0.0, 0.5 * (P.PROBE_SHAFT_L + P.PROBE_R)), color=PEI_DARK),
        cylinder(P.PROBE_R + 0.0005, P.IMM_MAX - P.IMM_MIN, pos=(0.0, 0.0, 0.5 * (P.IMM_MIN + P.IMM_MAX)), color=BAND_GREEN, solid=False, glow=0.15),
        cylinder(P.PROBE_R + 0.0006, 0.006, pos=(0.0, 0.0, P.PROBE_SHAFT_L - 0.003), color=HANNA_ORANGE, solid=False),
        cylinder(
            P.PROBE_CAP_D / 2.0, P.PROBE_CAP_L, pos=(0.0, 0.0, P.PROBE_SHAFT_L + P.PROBE_CAP_L / 2.0),
            color=WHITE, passes=("stand", "probe_cx", "probe_cy"),
        ),
    ]
    body("probe", probe_shapes, pos=(px, py, top + P.Z_PROBE_TIP), mass=P.PROBE_MASS, color=PEI_DARK, contacts=True)

    hinge("probe_lat_x", "probe_cx", "stand", anchor=anchor, axis=(1.0, 0.0, 0.0), limits=(-0.22, 0.22), spring=P.PROBE_LAT_K, rest=0.0, damping=P.PROBE_LAT_D)
    hinge("probe_lat_y", "probe_cy", "probe_cx", anchor=anchor, axis=(0.0, 1.0, 0.0), limits=(-0.22, 0.22), spring=P.PROBE_LAT_K, rest=0.0, damping=P.PROBE_LAT_D)
    slide("probe_axial", "probe", "probe_cy", anchor=anchor, axis=(0.0, 0.0, 1.0), limits=(-0.001, 0.012), spring=P.PROBE_AXIAL_K, rest=P.PROBE_AXIAL_PRELOAD, damping=P.PROBE_AXIAL_D)


def build_jack(top: float) -> None:
    """BOCHEM 11120 scissor jack: 160 mm along y, 130 mm along x, -y wheel on a half-height carrier."""
    jx, jy = P.JACK_XY
    fx, fy = P.JACK_W / 2.0 - 0.014, P.JACK_L / 2.0 - 0.014
    base_shapes = [
        *[
            box((0.022, 0.022, P.FOOT_H), pos=(sx * fx, sy * fy, P.FOOT_H / 2.0), color=RUBBER_DARK, friction=(P.JACK_FOOT_FRICTION, P.JACK_FOOT_FRICTION))
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
        ],
        box((P.JACK_W, P.JACK_L, P.PLATE_T), pos=(0.0, 0.0, P.FOOT_H + P.PLATE_T / 2.0), color=STEEL, friction=(0.60, 0.55)),
    ]
    # Visual lower scissor links along the +/-x sides
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            q_link = mk.matrix_to_quat(mk.axis_angle((1.0, 0.0, 0.0), sy * 0.28))
            base_shapes.append(box((0.004, 0.125, 0.010), pos=(sx * 0.056, 0.0, 0.026), quat=q_link, color=STEEL_DARK, solid=False))
    body("jack_base", base_shapes, pos=(jx, jy, top + P.OVER), mass=P.JACK_BASE_MASS, color=STEEL)

    deck_shapes = [
        box((P.JACK_W, P.JACK_L, P.PLATE_T), pos=(0.0, 0.0, -P.PLATE_T / 2.0), color=STEEL, friction=(0.55, 0.50)),
        cylinder(0.026, 0.0006, pos=(0.0, 0.0, 0.0003), color=STEEL_DARK, solid=False),
        cylinder(0.024, 0.0007, pos=(0.0, 0.0, 0.00035), color=STEEL, solid=False),
    ]
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            q_link = mk.matrix_to_quat(mk.axis_angle((1.0, 0.0, 0.0), sy * 0.28))
            deck_shapes.append(box((0.004, 0.125, 0.010), pos=(sx * 0.052, 0.0, -0.024), quat=q_link, color=STEEL_DARK, solid=False))
    body("jack_deck", deck_shapes, pos=(jx, jy, top + P.Z_DECK_START + P.OVER), mass=P.JACK_DECK_MASS, color=STEEL)

    dz_lo = P.JACK_Z_MIN - P.Z_DECK_START
    dz_hi = P.JACK_Z_SIM_MAX - P.Z_DECK_START
    slide("deck_z", "jack_deck", "jack_base", axis=(0.0, 0.0, 1.0), limits=(dz_lo, dz_hi), spring=P.DECK_SPRING, rest=0.0, damping=P.DECK_DAMPING)

    z_mid = top + 0.5 * P.Z_DECK_START + P.OVER
    spindle_len = P.JACK_L + P.WHEEL_BOSS_GAP
    carrier_shapes = [
        cylinder(
            0.004, spindle_len, pos=(0.0, -P.WHEEL_BOSS_GAP / 2.0, 0.0), axis="y",
            color=STEEL_DARK, passes=("jack_base", "jack_deck", "knob"),
        ),
        cylinder(
            0.007, 0.016, pos=(0.0, -P.JACK_L / 2.0 - 0.008, 0.0), axis="y",
            color=STEEL_DARK, passes=("jack_base", "jack_deck", "knob"),
        ),
    ]
    body("wheel_carrier", carrier_shapes, pos=(jx, jy, z_mid), mass=P.CARRIER_MASS, color=STEEL_DARK)
    slide("carrier_z", "wheel_carrier", "jack_base", axis=(0.0, 0.0, 1.0), limits=(0.5 * dz_lo, 0.5 * dz_hi), spring=P.DECK_SPRING, rest=0.0, damping=P.DECK_DAMPING)

    wheel_at = (jx, P.WHEEL_Y, z_mid)
    knob_shapes = [
        cylinder(P.WHEEL_D / 2.0, P.WHEEL_T, pos=(0.0, 0.0, 0.0), axis="y", color=KNOB_BLACK, friction=(1.3, 1.1), friction_combine="max"),
        box((P.WHEEL_D - 0.002, P.WHEEL_T + 0.0008, 0.0035), pos=(0.0, 0.0, 0.0), color=WHITE, solid=False),
    ]
    body("knob", knob_shapes, pos=wheel_at, mass=P.KNOB_MASS, color=KNOB_BLACK, contacts=True)
    # Free multi-turn hinge along +y: looking at the -y wheel along +y, CW is +theta (raises), CCW is -theta (lowers).
    hinge("wheel", "knob", "wheel_carrier", anchor=wheel_at, axis=(0.0, 1.0, 0.0), limits=None, damping=P.WHEEL_DAMPING)


def build_vessels_and_mats(top: float) -> None:
    """Blue sample mat, grey rinse start pad, and the two dry 50 mL Nalgene PP beakers."""
    body(
        "blue_mat",
        [
            box(P.MAT_SIZE, color=MAT_BLUE, friction=(0.80, 0.75)),
            box((P.MAT_SIZE[0] - 0.016, P.MAT_SIZE[1] - 0.016, 0.0006), pos=(0.0, 0.0, P.MAT_SIZE[2] / 2.0 + 0.0003), color=WHITE, solid=False),
            box((P.MAT_SIZE[0] - 0.020, P.MAT_SIZE[1] - 0.020, 0.0007), pos=(0.0, 0.0, P.MAT_SIZE[2] / 2.0 + 0.00035), color=MAT_BLUE, solid=False),
        ],
        pos=(P.MAT_XY[0], P.MAT_XY[1], top + P.MAT_SIZE[2] / 2.0),
        kinematic=True,
        color=MAT_BLUE,
    )
    body(
        "rinse_coaster",
        [
            box(P.COASTER_SIZE, color=COASTER_SLATE, friction=(0.80, 0.75)),
            cylinder(P.BEAKER_R_OUT + 0.004, 0.0006, pos=(0.0, 0.0, P.COASTER_SIZE[2] / 2.0 + 0.0003), color=BAND_GREEN, solid=False),
            cylinder(P.BEAKER_R_OUT + 0.001, 0.0007, pos=(0.0, 0.0, P.COASTER_SIZE[2] / 2.0 + 0.00035), color=COASTER_SLATE, solid=False),
        ],
        pos=(P.RINSE_START_XY[0], P.RINSE_START_XY[1], top + P.COASTER_SIZE[2] / 2.0),
        kinematic=True,
        color=COASTER_SLATE,
    )

    com, inertia = P.beaker_com_and_inertia()
    body(
        "beaker_sample",
        _beaker_shapes(green_band=False),
        pos=(P.JACK_XY[0], P.JACK_XY[1], top + P.Z_DECK_START + 2.0 * P.OVER),
        mass=P.BEAKER_MASS,
        color=PP_WHITE,
        center=com,
        inertia=inertia,
        contacts=True,
    )
    body(
        "beaker_rinse",
        _beaker_shapes(green_band=True),
        pos=(P.RINSE_START_XY[0], P.RINSE_START_XY[1], top + P.COASTER_SIZE[2] + P.OVER),
        mass=P.BEAKER_MASS,
        color=PP_WHITE,
        center=com,
        inertia=inertia,
        contacts=True,
    )

    mk.sense_contact("probe", "beaker_sample", "beaker_rinse")
    mk.sense_contact("beaker_sample", "jack_deck", "blue_mat", "probe")
    mk.sense_contact("beaker_rinse", "jack_deck", "rinse_coaster", "probe")


def build(top: float) -> None:
    build_stand_and_probe(top)
    build_jack(top)
    build_vessels_and_mats(top)
