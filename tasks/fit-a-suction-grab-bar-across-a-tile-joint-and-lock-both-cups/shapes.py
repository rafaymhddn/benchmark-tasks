"""The grab bar task's bodies as shapes, from parts.py's figures (a helper for task.py, which declares the bodies).

- The panel, in its own frame: the world's, moved to the tiles' face at the bench (x into the wall): the board, two
  white tiles with the open joint between them, the board drawn dark down the joint.
- The bar, in B (parts.py): the grip, two housings 0.5 mm off their cups, the grip's ends, a bracket at each grip end
  holding its lever's pivot at the housing's outer end, and per side a red and a green indicator tab on top of the
  housing (drawn only; one shown at a time).
- A cup: the rubber disc, its axis along x, centred on its body's origin.
- A lever: a plate from its pivot (its body's origin) along its x, upright, its contact compliant.
"""

from __future__ import annotations

import math

import numpy as np

import parts as P

# mechkit's box and cylinder are imported where they are called: a task loaded afresh (as the rule tests load it) gets
# a fresh mechkit, and the shapes must be that kit's own.

WHITE, TILE_WHITE, RUBBER, PLY, GROUT = (0.93, 0.93, 0.91), (0.97, 0.97, 0.96), (0.62, 0.64, 0.66), (0.74, 0.60, 0.42), (0.16, 0.14, 0.13)
RED, GREEN = (0.85, 0.08, 0.08), (0.10, 0.72, 0.18)
TILE_FRICTION = (0.6, 0.5)  # glazed tile (the rubber cups' own figure sets their contact: see parts.RUBBER_FRICTION)


def panel() -> list:
    from mechkit import box

    return [
        box((P.BOARD_T, P.BOARD_W, P.BOARD_H), pos=(P.TILE_T + P.BOARD_T / 2, P.JOINT_Y, P.BOARD_H / 2), color=PLY),
        box((P.TILE_T, P.TILE_W, P.TILE_H), pos=(P.TILE_T / 2, P.JOINT_Y + (P.JOINT_W + P.TILE_W) / 2, P.TILE_H / 2), color=TILE_WHITE, friction=TILE_FRICTION),
        box((P.TILE_T, P.TILE_W, P.TILE_H), pos=(P.TILE_T / 2, P.JOINT_Y - (P.JOINT_W + P.TILE_W) / 2, P.TILE_H / 2), color=TILE_WHITE, friction=TILE_FRICTION),
        box((0.001, P.JOINT_W, P.TILE_H), pos=(P.TILE_T - 0.0005, P.JOINT_Y, P.TILE_H / 2), color=GROUT, solid=False),  # the board seen down the open joint
    ]  # fmt: skip


def bar() -> tuple[list, dict]:
    """The bar's shapes, and per side the indices of its (red, green) indicator tabs."""
    from mechkit import box, cylinder

    housing_x = -P.CUP_T - P.HOUSING_T / 2
    front = -P.CUP_T - P.HOUSING_T  # the housings' front face
    shapes = [cylinder(P.GRIP_D / 2, P.CUP_PITCH, pos=(P.GRIP_X, 0.0, 0.0), axis="y", color=WHITE)]  # 0: the grip
    for side in ("l", "r"):
        y = P.CUP_Y[side]
        shapes.append(cylinder(P.HOUSING_D / 2, P.HOUSING_T - 0.0005, pos=(housing_x - 0.00025, y, 0.0), axis="x", color=WHITE))  # 0.5 mm off its cup
        shapes.append(box((front - P.GRIP_X + 0.004, 0.026, 0.030), pos=((front + P.GRIP_X) / 2, y, 0.0), color=WHITE))  # the grip's end on the housing
        s = math.copysign(1.0, y)
        inner, outer = 0.013, P.LEVER_PIVOT[1] - P.LEVER_T / 2 - 0.001  # from the grip end's outer face to just short of the lever
        shapes.append(box((0.012, outer - inner, 0.012), pos=(P.LEVER_PIVOT[0], y + s * (inner + outer) / 2, P.LEVER_PIVOT[2]), color=WHITE))  # the lever's bracket
    tabs = {}
    for side in ("l", "r"):
        y = P.CUP_Y[side]
        at = (P.INDICATOR_AT[0], y, P.INDICATOR_AT[1] + 0.002)  # on top of the housing
        tabs[side] = (len(shapes), len(shapes) + 1)
        shapes.append(cylinder(P.INDICATOR_D / 2, 0.004, pos=at, axis="z", color=RED, solid=False))
        shapes.append(cylinder(P.INDICATOR_D / 2, 0.004, pos=at, axis="z", color=GREEN, solid=False))
    return shapes, tabs


def cup() -> list:
    from mechkit import cylinder

    return [cylinder(P.CUP_R, P.CUP_T, axis="x", color=RUBBER)]


def lever() -> list:
    """The lever's plate, its contact compliant (parts.LEVER_CONTACT)."""
    from mechkit import box, soften

    return [soften(box((P.LEVER_L, P.LEVER_T, P.LEVER_WIDTH), pos=(P.LEVER_L / 2, 0.0, 0.0), color=WHITE), *P.LEVER_CONTACT)]


def lever_rot(theta: float, side: str) -> np.ndarray:
    """A lever's rotation at ``theta`` rad from released: its x (pivot to free end) turned about z from the wall
    outwards (parts.lever_dir_b); locked (LEVER_RAD) it is the bar's own."""
    phi = P.out_sign(side) * (P.LEVER_RAD - theta)
    c, s = math.cos(phi), math.sin(phi)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
