"""The bodies of the end-cap suspension packing task, built from parts.py's figures: the fixed open-top corrugated
carton, the fixed bench roller cradle, the two free polyethylene foam end-cap cushions (nested inside the empty carton
at the start), and the free Interroll galvanized steel conveyor roller resting on the bench cradle."""

from __future__ import annotations

import numpy as np

import mechkit as mk
import parts as P
from mechkit import body, box, cylinder

MECHKIT_HELPER = True  # declares for the task file that imports it

KRAFT = (0.58, 0.42, 0.25)
KRAFT_FLOOR = (0.48, 0.34, 0.20)
FOAM_BLUE = (0.14, 0.44, 0.80)
FOAM_SOCKET = (0.09, 0.30, 0.58)
STEEL = (0.72, 0.76, 0.80)
SPINDLE_STEEL = (0.26, 0.28, 0.31)
POLYAMIDE = (0.12, 0.12, 0.14)
SEAL_YELLOW = (0.95, 0.80, 0.10)
CRADLE_WOOD = (0.30, 0.24, 0.18)

CAPS = ("l", "r")  # cap_l starts at +y (socket toward -y), cap_r at -y (socket toward +y)


def _quat(matrix: np.ndarray) -> tuple[float, ...]:
    return tuple(float(v) for v in mk.matrix_to_quat(matrix))


def cap_inward(c: str) -> float:
    """Direction along world y from cap ``c``'s centre toward its open socket (-1 for cap_l, +1 for cap_r)."""
    return -1.0 if c == "l" else 1.0


def build_carton(top: float) -> None:
    """ULINE S-4600 10 x 6 x 3 in open-top corrugated carton, taped square on the bench at (X_CARTON, 0).
    Kinematic so contact-force sensors read exact floor and wall loads."""
    w, l, h, t = P.BOX_W, P.BOX_L, P.BOX_H, P.BOX_WALL
    outer_x, outer_y = w + 2.0 * t, l + 2.0 * t
    shapes = [
        box((outer_x, outer_y, t), pos=(0.0, 0.0, t / 2.0), color=KRAFT_FLOOR),
        box((t, outer_y, h), pos=(-w / 2.0 - t / 2.0, 0.0, t + h / 2.0), color=KRAFT),
        box((t, outer_y, h), pos=(+w / 2.0 + t / 2.0, 0.0, t + h / 2.0), color=KRAFT),
        box((w, t, h), pos=(0.0, -l / 2.0 - t / 2.0, t + h / 2.0), color=KRAFT),
        box((w, t, h), pos=(0.0, +l / 2.0 + t / 2.0, t + h / 2.0), color=KRAFT),
    ]
    body(
        "carton",
        shapes,
        pos=(P.X_CARTON, 0.0, top),
        kinematic=True,
        color=KRAFT,
        friction=(P.CARTON_FRICTION, P.CARTON_FRICTION),
    )


def build_cradle(top: float) -> None:
    """100 mm birch bench saddle at (X_ROLLER, 0) holding the 40 mm roller coaxial with z = 38.5 mm."""
    rail_h = P.CRADLE_RAIL_TOP - P.CRADLE_BASE_H
    rail_x = P.CRADLE_RAIL_INNER_X + P.CRADLE_RAIL_W / 2.0
    shapes = [
        box((P.CRADLE_W, P.CRADLE_L, P.CRADLE_BASE_H), pos=(0.0, 0.0, P.CRADLE_BASE_H / 2.0), color=CRADLE_WOOD),
        box((P.CRADLE_RAIL_W, P.CRADLE_L, rail_h), pos=(-rail_x, 0.0, P.CRADLE_BASE_H + rail_h / 2.0), color=CRADLE_WOOD),
        box((P.CRADLE_RAIL_W, P.CRADLE_L, rail_h), pos=(+rail_x, 0.0, P.CRADLE_BASE_H + rail_h / 2.0), color=CRADLE_WOOD),
    ]
    body(
        "cradle",
        shapes,
        pos=(P.X_ROLLER, 0.0, top),
        kinematic=True,
        color=CRADLE_WOOD,
        friction=(P.CRADLE_FRICTION, P.CRADLE_FRICTION),
    )


def build_roller(top: float) -> None:
    """Interroll Series 1700 40 x 200 mm galvanized steel roller with 8 x 226 mm spring-loaded steel spindle."""
    slip = (P.STEEL_MU, P.STEEL_MU)
    shapes = [
        cylinder(P.ROLLER_R, P.ROLLER_L, axis="y", color=STEEL, friction=slip, friction_combine="min"),
        cylinder(P.SPINDLE_R, P.SPINDLE_L, axis="y", color=SPINDLE_STEEL, friction=slip, friction_combine="min"),
    ]
    for s in (-1.0, 1.0):
        shapes.append(cylinder(P.ROLLER_R * 0.92, 0.0012, pos=(0.0, s * (P.ROLLER_L / 2.0 - 0.0004), 0.0), axis="y", color=POLYAMIDE, solid=False))
        shapes.append(cylinder(P.ROLLER_R * 0.58, 0.0016, pos=(0.0, s * (P.ROLLER_L / 2.0 - 0.0003), 0.0), axis="y", color=SEAL_YELLOW, solid=False))
    body(
        "roller",
        shapes,
        pos=(P.X_ROLLER, 0.0, top + P.SOCK_Z + P.OVER),
        mass=P.ROLLER_MASS,
        color=STEEL,
        friction=slip,
        friction_combine="min",
        center=(0.0, 0.0, 0.0),
        inertia=P.roller_inertia(),
    )


def build_caps(top: float) -> None:
    """Two stepped closed-cell PE foam end-caps with true 21 mm-deep, 45 mm octagonal sockets and 15 mm spindle pockets.
    Outer surfaces carry GRIPPY friction for gripper pinch; internal socket surfaces carry smooth PE-on-steel friction."""
    slip = (P.STEEL_MU, P.STEEL_MU)
    for c in CAPS:
        inward = cap_inward(c)
        y0 = (+1.0 if c == "l" else -1.0) * P.CAP_START_Y
        shapes = []
        for dims, pos, inner in P.cap_axis_boxes(inward):
            if inner:
                shapes.append(box(dims, pos=pos, color=FOAM_SOCKET, friction=slip, friction_combine="min"))
            else:
                shapes.append(box(dims, pos=pos, color=FOAM_BLUE, friction=mk.GRIPPY))
        for dims, pos, theta in P.cap_bevel_boxes(inward):
            shapes.append(
                box(
                    dims,
                    pos=pos,
                    quat=_quat(mk.axis_angle((0.0, 1.0, 0.0), theta)),
                    color=FOAM_SOCKET,
                    friction=slip,
                    friction_combine="min",
                )
            )
        com, inertia = P.cap_com_and_inertia(inward)
        body(
            f"cap_{c}",
            shapes,
            pos=(P.X_CARTON, y0, top + P.Z_FLOOR + P.OVER),
            mass=P.CAP_MASS,
            color=FOAM_BLUE,
            friction=mk.GRIPPY,
            center=com,
            inertia=inertia,
            contacts=True,
        )


def build(top: float) -> None:
    build_carton(top)
    build_cradle(top)
    build_roller(top)
    build_caps(top)
    mk.sense_contact("roller", "carton", "cradle", "cap_l", "cap_r")
    mk.sense_contact("cap_l", "carton", "roller")
    mk.sense_contact("cap_r", "carton", "roller")
