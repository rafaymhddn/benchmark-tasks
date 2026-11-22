"""The bodies and joints of the hobby-burner stow task, built from ``parts.py``'s figures.

Serial kinematic chains (zero closed loops, zero direct slide+weld between two bodies):
1. ``stand`` --slide("heel_fit")--> ``heel_carrier`` --weld("w_stand")--> ``burner``
   - ``burner`` -> slide("trigger")
   - ``burner`` -> slide("lock_slider")
   - ``burner`` -> slide("child_stop")
2. ``tin_base`` --hinge("lid_hinge")--> ``tin_lid``
"""

from __future__ import annotations

import math

import numpy as np

import mechkit as mk
import parts as P
from mechkit import body, box, cylinder, hinge, slide, weld

MECHKIT_HELPER = True  # declares for the task file that imports it

DREMEL_BLUE = (0.11, 0.25, 0.39)
SILVER = (0.76, 0.78, 0.82)
TIN_SILVER = (0.72, 0.75, 0.79)
TIN_INNER = (0.64, 0.67, 0.71)
CHARCOAL = (0.20, 0.21, 0.23)
MATTE_BLACK = (0.11, 0.12, 0.13)
STOP_RED = (0.88, 0.13, 0.11)
BUTTON_SILVER = (0.82, 0.84, 0.87)
PIP_WHITE = (0.95, 0.96, 0.97)
FOAM_GREY = (0.19, 0.20, 0.22)
FOAM_WELL = (0.12, 0.13, 0.15)


def _floats(v) -> tuple[float, ...]:
    return tuple(float(x) for x in v)


def _quat(matrix: np.ndarray) -> tuple[float, ...]:
    return tuple(float(v) for v in mk.matrix_to_quat(matrix))


def stand_origin(top: float) -> np.ndarray:
    """World origin of ``stand``, ``heel_carrier``, and ``burner`` as built (at the socket floor)."""
    return np.array([P.STAND_XY[0], P.STAND_XY[1], top + P.OVER + P.STAND_FLOOR_H])


def tin_origin(top: float) -> np.ndarray:
    """World origin of ``tin_base`` (at the center of its underside)."""
    return np.array([P.TIN_XY[0], P.TIN_XY[1], top + P.OVER])


def hinge_anchor(top: float) -> np.ndarray:
    """World hinge line along the rear (-y) wall top of ``tin_base``."""
    return np.array([
        P.TIN_XY[0],
        P.TIN_XY[1] - P.TIN_W / 2.0 + P.TIN_WALL / 2.0,
        top + P.OVER + P.TIN_BASE_H,
    ])


def build_burner_and_stand(top: float) -> None:
    origin = stand_origin(top)
    yaw = P.BURN_YAW
    turn = mk.axis_angle((0.0, 0.0, 1.0), yaw)
    xb = turn[:, 0]
    yb = turn[:, 1]

    # 1. Removable 60 mm round stand: 12 mm solid floor disc + 12-segment socket cup wall (40.8 mm bore, 60.0 mm OD).
    cup_segments = 12
    r_in = P.SOCKET_CLEAR_R
    r_out = 0.0295  # corners at hypot(29.5, 5.4) = 30.0 mm = exact 60.0 mm outer diameter
    r_mid = 0.5 * (r_in + r_out)
    radial_t = r_out - r_in
    chord_w = 2.0 * r_in * math.tan(math.pi / cup_segments)
    stand_shapes = [
        cylinder(P.STAND_R, P.STAND_FLOOR_H, pos=(0.0, 0.0, -P.STAND_FLOOR_H / 2.0), color=MATTE_BLACK),
    ]
    for i in range(cup_segments):
        ang = 2.0 * math.pi * i / cup_segments
        stand_shapes.append(
            box(
                (radial_t, chord_w, P.SOCKET_DEPTH),
                pos=(r_mid * math.cos(ang), r_mid * math.sin(ang), P.SOCKET_DEPTH / 2.0),
                yaw=ang,
                color=MATTE_BLACK,
            )
        )
    body("stand", stand_shapes, pos=_floats(origin), yaw=yaw, mass=P.STAND_BODY_MASS, color=MATTE_BLACK, contacts=True)

    # 2. Intermediate axial socket carrier: carries slide("heel_fit") to stand and weld("w_stand") to burner.
    body(
        "heel_carrier",
        [cylinder(0.008, 0.004, pos=(0.0, 0.0, -0.003), color=MATTE_BLACK, passes=("stand", "burner", "tin_base", "tin_lid"))],
        pos=_floats(origin),
        yaw=yaw,
        mass=P.CARRIER_MASS,
        color=MATTE_BLACK,
    )
    slide(
        "heel_fit",
        "heel_carrier",
        "stand",
        axis=(0.0, 0.0, 1.0),
        limits=(0.0, P.HEEL_SLIDE_MAX),
        spring=P.HEEL_K,
        rest=0.0,
        damping=P.HEEL_DAMP,
    )

    # 3. Dremel VersaFlame 2200 burner body: 38 mm silver tank, blue grip housing, top cap, and +x_b stainless nozzle.
    nozzle_collar_x = 0.025 + P.COLLAR_L / 2.0  # +0.034 m
    nozzle_tip_x = 0.025 + P.COLLAR_L + P.NOZZLE_L / 2.0  # +0.062 m (front tip at +0.081 m; 110 mm from -0.029 m button)
    burner_shapes = [
        cylinder(P.TANK_R, P.TANK_H, pos=(0.0, 0.0, P.TANK_H / 2.0), color=SILVER),
        box(P.HOUSING_SIZE, pos=(0.001, 0.0, P.TANK_H + P.HOUSING_SIZE[2] / 2.0), color=DREMEL_BLUE),
        # Smooth molded polymer slider rail on the -y_b cheek (z_b = 115..133 mm) guiding thumb/fingertip into lock_slider.
        box((0.038, 0.002, 0.018), pos=(-0.004, -0.0125, P.LOCK_POS_B[2]), color=DREMEL_BLUE, friction=(0.05, 0.05), friction_combine="min"),
        box(P.CAP_SIZE, pos=(0.002, 0.0, P.TOOL_H - P.CAP_SIZE[2] / 2.0), color=CHARCOAL),
        cylinder(P.COLLAR_R, P.COLLAR_L, pos=(nozzle_collar_x, 0.0, P.NOZZLE_Z), axis="x", color=CHARCOAL),
        cylinder(P.NOZZLE_R, P.NOZZLE_L, pos=(nozzle_tip_x, 0.0, P.NOZZLE_Z), axis="x", color=SILVER),
        # Visual rear bezel and side lock track so the button travel reads crisply at 320 x 176.
        box((0.002, 0.018, 0.022), pos=(-0.0235, 0.0, P.TRIG_POS_B[2]), color=MATTE_BLACK, solid=False),
        box((0.020, 0.001, 0.016), pos=(-0.007, -0.0133, P.LOCK_POS_B[2]), color=MATTE_BLACK, solid=False),
    ]
    body(
        "burner",
        burner_shapes,
        pos=_floats(origin),
        yaw=yaw,
        mass=P.BURNER_BODY_MASS,
        color=DREMEL_BLUE,
        center=(0.003, 0.0, 0.072),
        inertia=(0.00047, 0.00050, 0.00007),
        contacts=True,
    )
    weld("w_stand", "burner", "heel_carrier")

    # 4. Rear ignition button (Part 5): built latched 4.5 mm inward; springs +4.5 mm outward (-x_b) once unlatched.
    trig_at = origin + turn @ np.array(P.TRIG_POS_B)
    body(
        "trigger",
        [box(P.TRIG_SIZE, color=BUTTON_SILVER, passes=("burner", "lock_slider", "child_stop"))],
        pos=_floats(trig_at),
        yaw=yaw,
        mass=P.CONTROL_MASS,
        color=BUTTON_SILVER,
    )
    slide(
        "trigger",
        "trigger",
        "burner",
        axis=_floats(-xb),
        limits=(-P.TRIG_OVERTRAVEL, P.TRIG_LAT),
        spring=P.TRIG_K,
        rest=P.TRIG_REST,
        damping=P.TRIG_DAMP,
    )

    # 5. Side continuous-flow lock button (Part 7): built at rear locked stop; slides +6.0 mm forward (+x_b) to unlock.
    lock_at = origin + turn @ np.array(P.LOCK_POS_B)
    body(
        "lock_slider",
        [
            box(P.LOCK_SIZE, color=MATTE_BLACK, passes=("burner", "trigger", "child_stop")),
            box((0.0040, 0.001, 0.010), pos=(0.0020, -P.LOCK_SIZE[1] / 2.0 - 0.0003, 0.0), color=PIP_WHITE, solid=False),
            box((0.0040, 0.0045, 0.001), pos=(0.0020, 0.0, P.LOCK_SIZE[2] / 2.0 + 0.0003), color=PIP_WHITE, solid=False),
        ],
        pos=_floats(lock_at),
        yaw=yaw,
        mass=P.CONTROL_MASS,
        color=MATTE_BLACK,
        contacts=True,
    )
    slide(
        "lock_slider",
        "lock_slider",
        "burner",
        axis=_floats(xb),
        limits=(0.0, P.LOCK_STROKE),
        spring=P.LOCK_K,
        rest=0.0,
        damping=P.LOCK_DAMP,
    )

    # 6. Red child-safety stopper (Part 6): built displaced 6 mm left (+y_b); snaps -6 mm under trigger when released.
    child_at = origin + turn @ np.array(P.CHILD_POS_B)
    body(
        "child_stop",
        [box(P.CHILD_SIZE, color=STOP_RED, passes=("burner", "trigger", "lock_slider"))],
        pos=_floats(child_at),
        yaw=yaw,
        mass=P.CONTROL_MASS,
        color=STOP_RED,
    )
    slide(
        "child_stop",
        "child_stop",
        "burner",
        axis=_floats(-yb),
        limits=(0.0, P.CHILD_STROKE),
        spring=P.CHILD_K,
        rest=P.CHILD_REST,
        damping=P.CHILD_DAMP,
    )


def build_tin(top: float) -> None:
    origin = tin_origin(top)
    wall_h = P.TIN_BASE_H - P.TIN_FLOOR
    wall_z = P.TIN_FLOOR + wall_h / 2.0
    y_wall = P.TIN_W / 2.0 - P.TIN_WALL / 2.0
    x_wall = P.TIN_L / 2.0 - P.TIN_WALL / 2.0
    inner_l = P.TIN_L - 2.0 * P.TIN_WALL
    inner_w = P.TIN_W - 2.0 * P.TIN_WALL

    base_shapes = [
        box((P.TIN_L, P.TIN_W, P.TIN_FLOOR), pos=(0.0, 0.0, P.TIN_FLOOR / 2.0), color=TIN_SILVER),
        box((P.TIN_L, P.TIN_WALL, wall_h), pos=(0.0, -y_wall, wall_z), color=TIN_SILVER),
        box((P.TIN_L, P.TIN_WALL, wall_h), pos=(0.0, y_wall, wall_z), color=TIN_SILVER),
        box((P.TIN_WALL, inner_w, wall_h), pos=(-x_wall, 0.0, wall_z), color=TIN_SILVER),
        box((P.TIN_WALL, inner_w, wall_h), pos=(x_wall, 0.0, wall_z), color=TIN_SILVER),
        # Dark EVA foam tray face and pocket outlines (visual only: flat floor so nothing snags).
        box((inner_l - 0.002, inner_w - 0.002, 0.0005), pos=(0.0, 0.0, P.TIN_FLOOR + 0.0003), color=FOAM_GREY, solid=False),
        cylinder(0.033, 0.0006, pos=(-0.105, 0.0, P.TIN_FLOOR + 0.0007), color=FOAM_WELL, solid=False),
        box((0.180, 0.046, 0.0006), pos=(0.050, 0.022, P.TIN_FLOOR + 0.0007), color=FOAM_WELL, solid=False),
        box((0.055, 0.115, 0.0006), pos=(-0.012, -0.012, P.TIN_FLOOR + 0.0007), color=FOAM_WELL, solid=False),
    ]
    body("tin_base", base_shapes, pos=_floats(origin), mass=P.TIN_BASE_MASS, color=TIN_SILVER, friction=P.TIN_FRICTION, contacts=True)

    # Hinged metal lid built at LID_OPEN_DEG (108 deg open, leaning 18 deg past vertical toward -y against its stop).
    # In lid local frame (when closed at 0 deg relative to hinge_anchor): y runs [-TIN_WALL/2, TIN_W - TIN_WALL/2].
    anchor = hinge_anchor(top)
    y_mid = (P.TIN_W - P.TIN_WALL) / 2.0
    skirt_h = P.TIN_LID_H - P.TIN_CEIL  # 10 mm perimeter skirt below the 4 mm top skin
    lid_shapes = [
        box((P.TIN_L, P.TIN_W, P.TIN_CEIL), pos=(0.0, y_mid, P.TIN_LID_H - P.TIN_CEIL / 2.0), color=TIN_SILVER),
        box((P.TIN_L, P.TIN_WALL, skirt_h), pos=(0.0, 0.0, skirt_h / 2.0), color=TIN_INNER),
        box((P.TIN_L, P.TIN_WALL, skirt_h), pos=(0.0, P.TIN_W - P.TIN_WALL, skirt_h / 2.0), color=TIN_INNER),
        box((P.TIN_WALL, inner_w, skirt_h), pos=(-x_wall, y_mid, skirt_h / 2.0), color=TIN_INNER),
        box((P.TIN_WALL, inner_w, skirt_h), pos=(x_wall, y_mid, skirt_h / 2.0), color=TIN_INNER),
    ]
    open_rot = mk.axis_angle((1.0, 0.0, 0.0), P.LID_OPEN_RAD)
    body("tin_lid", lid_shapes, pos=_floats(anchor), quat=_quat(open_rot), mass=P.TIN_LID_MASS, color=TIN_SILVER, contacts=True)
    hinge(
        "lid_hinge",
        "tin_lid",
        "tin_base",
        anchor=_floats(anchor),
        axis=(-1.0, 0.0, 0.0),
        limits=(0.0, P.LID_OPEN_RAD),
        damping=P.LID_DAMP,
    )


def build(top: float) -> None:
    build_burner_and_stand(top)
    build_tin(top)
