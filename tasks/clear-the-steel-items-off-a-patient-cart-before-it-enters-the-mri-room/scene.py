"""The bodies of the MRI tray task, built from parts.py's figures: the polypropylene tray on the bench (the
patient cart's top), the yellow bin on its lettered zone mat, the flat MRI Zone IV sign, and the six items —
three steel (the standing clamp, the padlock, the bent-handle shears) and three plastic (the bottle, the kidney
dish, the clipboard).

No joints anywhere: eight free rigid bodies and two printed mats that only draw. The lettering lies flat on the
bench so the top camera reads it.
"""

from __future__ import annotations

import math

import numpy as np

import mechkit as mk
import parts as P
from mechkit import body, box, cylinder

MECHKIT_HELPER = True  # declares for the task file that imports it (the kit skips this module's frames)
WHITE, YELLOW, BLACK, RED = (0.94, 0.94, 0.92), (0.97, 0.80, 0.05), (0.06, 0.06, 0.07), (0.78, 0.10, 0.10)
GREEN = (0.10, 0.58, 0.22)
STEEL, DARK, PET, CAP = (0.46, 0.48, 0.52), (0.30, 0.32, 0.36), (0.74, 0.88, 0.95), (0.18, 0.34, 0.62)
PAINT, SCREW, DISH_C, PAPER = (0.88, 0.36, 0.08), (0.62, 0.64, 0.68), (0.89, 0.56, 0.62), (0.98, 0.97, 0.93)
CLIP_C = (0.24, 0.46, 0.72)
TRAY_FRICTION = (P.TRAY_FRICTION, P.TRAY_FRICTION * 0.9)
ITEM_FRICTION = (1.0, 0.8)  # dry steel and plastic under the pads


def _walls(length, width, height, thick, floor, color=None):
    """Four rim walls of an open tray or bin in its frame (origin at the base's centre, z up)."""
    kw = {"color": color} if color is not None else {}
    return [
        box((length, thick, height), pos=(0.0, width / 2.0 - thick / 2.0, floor + height / 2.0), **kw),
        box((length, thick, height), pos=(0.0, -(width / 2.0 - thick / 2.0), floor + height / 2.0), **kw),
        box((thick, width - 2.0 * thick, height), pos=(length / 2.0 - thick / 2.0, 0.0, floor + height / 2.0), **kw),
        box((thick, width - 2.0 * thick, height), pos=(-(length / 2.0 - thick / 2.0), 0.0, floor + height / 2.0), **kw),
    ]


def build_tray(top: float) -> None:
    body("tray", [box((P.TRAY_D, P.TRAY_W, P.TRAY_WALL), pos=(0.0, 0.0, P.TRAY_WALL / 2.0)),
                  *_walls(P.TRAY_D, P.TRAY_W, P.TRAY_RIM, P.TRAY_WALL, P.TRAY_WALL)],
         pos=(*P.TRAY_AT, top + P.OVER), mass=P.TRAY_MASS, color=WHITE, friction=TRAY_FRICTION)


def build_bin(top: float) -> None:
    import lettering

    w = lettering.width(P.BIN_HOLD_TEXT, P.BIN_HOLD_CELL)
    h = lettering.height(P.BIN_HOLD_CELL)
    raw = lettering.shapes(P.BIN_HOLD_TEXT, P.BIN_HOLD_CELL, BLACK, at=(-w / 2.0, P.BIN_HOLD_X + h / 2.0, P.BIN_WALL + 0.0008))
    floor_hold = [box((s.dims[1], s.dims[0], s.dims[2]), pos=(s.pos[1], -s.pos[0], s.pos[2]), color=BLACK, solid=False) for s in raw]
    body("bin", [box((P.BIN_L, P.BIN_W, P.BIN_WALL), pos=(0.0, 0.0, P.BIN_WALL / 2.0)),
                 *_walls(P.BIN_L, P.BIN_W, P.BIN_H - P.BIN_WALL, P.BIN_WALL, P.BIN_WALL),
                 *floor_hold],
         pos=(*P.BIN_AT, top + P.OVER), mass=P.BIN_MASS, color=YELLOW, friction=TRAY_FRICTION)


READ_TURN = math.radians(-90.0)  # the top camera's picture reads along world -y: turn the printed marks a quarter turn


def build_label(top: float) -> None:
    """The L-shaped yellow holding mat on the safe near side: under the bin and across the front banner with a green border."""
    import lettering

    # In rotated frame (pos = LABEL_AT, yaw = -90 deg): local +x = world -y (right in image), local +y = world +x (up in image).
    f = 0.006
    c = P.LABEL_LETTER_CELL
    # Apron under the bin: world x from LABEL_AT[0] - LABEL_W/2 to BIN_AT[0] + BIN_L/2 + 0.012; world y from -0.232 to -0.478
    apron_y0 = -P.LABEL_W / 2.0
    apron_y1 = (P.BIN_AT[0] + P.BIN_L / 2.0 + 0.012) - P.LABEL_AT[0]
    apron_lx0 = P.LABEL_AT[1] - (-0.232)  # +0.212 -> world y = -0.232
    apron_lx1 = P.LABEL_AT[1] - (-0.478)  # +0.458 -> world y = -0.478
    bridge_lx0 = P.LABEL_L / 2.0
    shapes = [
        # Main front banner with green safe-holding border
        box((P.LABEL_L, P.LABEL_W, 0.001), pos=(0.0, 0.0, 0.0005), color=YELLOW, solid=False),
        box((P.LABEL_L - 0.004, f, 0.0012), pos=(0.0, P.LABEL_W / 2.0 - f / 2.0 - 0.002, 0.0006), color=GREEN, solid=False),
        box((P.LABEL_L - 0.004, f, 0.0012), pos=(0.0, -(P.LABEL_W / 2.0 - f / 2.0 - 0.002), 0.0006), color=GREEN, solid=False),
        box((f, P.LABEL_W - 0.004, 0.0012), pos=(-(P.LABEL_L / 2.0 - f / 2.0 - 0.002), 0.0, 0.0006), color=GREEN, solid=False),
        # Continuous yellow mat bridging right into and under the yellow bin (no top border between banner and bin!)
        box((apron_lx1 - bridge_lx0, P.LABEL_W, 0.001), pos=((bridge_lx0 + apron_lx1) / 2.0, 0.0, 0.0005), color=YELLOW, solid=False),
        box((apron_lx1 - apron_lx0, apron_y1 - apron_y0, 0.001),
            pos=((apron_lx0 + apron_lx1) / 2.0, (apron_y0 + apron_y1) / 2.0, 0.0005), color=YELLOW, solid=False),
    ]
    for line, top_y in ((P.LABEL_LETTER, 0.068), (P.LABEL_LINE2, 0.018), (P.LABEL_LINE3, -0.032)):
        w = lettering.width(line, c)
        shapes += lettering.shapes(line, c, BLACK, at=(-w / 2.0, top_y, 0.0016))
    body("label", shapes, pos=(*P.LABEL_AT, top), quat=mk.yaw_quat(READ_TURN), kinematic=True, color=YELLOW)


def build_sign(top: float) -> None:
    """The doorway sign on the cart's far side: full-bench red threshold line at x = 62.5 cm, MRI / ZONE IV beyond it."""
    import lettering

    f = P.SIGN_FRAME
    door_dy = P.DOOR_LINE_X - P.SIGN_AT[0]  # negative local_y = in front of the sign towards the cart
    bench_span = 2.0 * P.BENCH_HALF_Y - 0.010  # 1.29 m across the full 1.30 m bench width (world y = +-0.645 m)
    shapes = [
        box((bench_span, 0.014, 0.0012), pos=(P.SIGN_AT[1], door_dy, 0.0006), color=RED, solid=False),
        box((P.SIGN_L, P.SIGN_W, 0.001), pos=(0.0, 0.0, 0.0005), color=WHITE, solid=False),
        box((P.SIGN_L - 0.004, f, 0.0012), pos=(0.0, P.SIGN_W / 2.0 - f / 2.0 - 0.002, 0.0006), color=RED, solid=False),
        box((P.SIGN_L - 0.004, f, 0.0012), pos=(0.0, -(P.SIGN_W / 2.0 - f / 2.0 - 0.002), 0.0006), color=RED, solid=False),
        box((f, P.SIGN_W - 0.004, 0.0012), pos=(P.SIGN_L / 2.0 - f / 2.0 - 0.002, 0.0, 0.0006), color=RED, solid=False),
        box((f, P.SIGN_W - 0.004, 0.0012), pos=(-(P.SIGN_L / 2.0 - f / 2.0 - 0.002), 0.0, 0.0006), color=RED, solid=False),
    ]
    for line, cell, top_y in ((P.SIGN_LINE1, P.SIGN_CELL1, 0.072), (P.SIGN_LINE2, P.SIGN_CELL2, -0.030)):
        shapes += lettering.shapes(line, cell, BLACK, at=(-lettering.width(line, cell) / 2.0, top_y, 0.0016))
    body("sign", shapes, pos=(*P.SIGN_AT, top), quat=mk.yaw_quat(READ_TURN), kinematic=True, color=WHITE)


def build_clamp(top: float) -> None:
    """The ductile-iron C-clamp standing on its foot, opening along +x, its screw and T-handle up."""
    t = P.CLAMP_FRAME_T
    shapes = [
        box((0.046, 0.034, 0.010), pos=(0.008, 0.0, 0.005), color=PAINT),  # the fixed jaw's foot
        box((t, 0.034, 0.064), pos=(-0.024, 0.0, 0.042), color=PAINT),  # the C's flat back
        box((0.058, 0.034, 0.011), pos=(0.001, 0.0, 0.0795), color=PAINT),  # the top arm the screw passes
        cylinder(0.005, 0.036, pos=(0.017, 0.0, 0.101), color=SCREW),  # the screw
        cylinder(0.0045, 0.054, pos=(0.017, 0.0, 0.115), axis="y", color=SCREW),  # the T-handle
    ]
    body("clamp", shapes, pos=(*P.CLAMP_AT, top + P.TRAY_FLOOR + P.OVER), mass=P.CLAMP_MASS, color=PAINT,
         center=(-0.006, 0.0, 0.042), friction=ITEM_FRICTION)


def build_padlock(top: float) -> None:
    r = P.LOCK_SHACKLE_R
    shapes = [
        box((P.LOCK_W, P.LOCK_D, P.LOCK_BODY_H), pos=(0.0, 0.0, P.LOCK_BODY_H / 2.0), color=STEEL),
        cylinder(r, P.LOCK_BODY_H + 0.016, pos=(0.011, 0.0, P.LOCK_BODY_H + 0.008), color=DARK),
        cylinder(r, P.LOCK_BODY_H + 0.016, pos=(-0.011, 0.0, P.LOCK_BODY_H + 0.008), color=DARK),
        box((0.030, 2.0 * r, 2.0 * r), pos=(0.0, 0.0, P.LOCK_BODY_H + 0.016 - r), color=DARK),  # the shackle's back
    ]
    body("padlock", shapes, pos=(*P.LOCK_AT, top + P.TRAY_FLOOR + P.OVER), quat=mk.yaw_quat(math.radians(P.LOCK_TURN)),
         mass=P.LOCK_MASS, color=STEEL, center=(0.0, 0.0, P.LOCK_BODY_H / 2.0), friction=ITEM_FRICTION)


def build_shears(top: float) -> None:
    """The closed dressmaker's shears, blades to +y; the bent handles lift the two rings ~30 mm off the plane."""
    bend = math.atan2(P.SHEARS_RING_LIFT - 0.006, 0.058)  # the handles' rise from the pivot to the rings
    along = np.array([0.0, -math.cos(bend), math.sin(bend)])
    turn = mk.axis_angle((1.0, 0.0, 0.0), bend)
    shapes = [
        box((0.006, 0.116, 0.018), pos=(-0.004, 0.058, 0.009), color=STEEL),  # the two blades, flat and shut
        box((0.006, 0.116, 0.018), pos=(0.004, 0.058, 0.009), color=STEEL),
        cylinder(0.0055, 0.004, pos=(0.0, 0.0, 0.011), color=SCREW),  # the pivot screw's head, up
    ]
    for x in (-0.006, 0.006):  # the two shanks, rising to the rings
        mid = np.array([x, -0.049, 0.018])
        shapes.append(box((0.006, 0.064, 0.012), pos=tuple(mid), quat=tuple(float(v) for v in mk.matrix_to_quat(turn)), color=STEEL))
    for x in (-0.0145, 0.0145):  # the finger ring and the thumb ring, flat loops in the bent plane
        centre = np.array([x, -0.078, P.SHEARS_RING_LIFT])
        for s in (1.0, -1.0):
            shapes.append(box((0.030, 0.007, 0.007), pos=tuple(centre + s * 0.0155 * along), color=DARK))
        for s in (1.0, -1.0):
            at = centre + np.array([s * 0.0145, 0.0, 0.0])
            shapes.append(box((0.007, 0.029, 0.007), pos=tuple(at), quat=tuple(float(v) for v in mk.matrix_to_quat(turn)), color=DARK))
    body("shears", shapes, pos=(*P.SHEARS_AT, top + P.TRAY_FLOOR + P.OVER), mass=P.SHEARS_MASS, color=STEEL,
         center=(0.0, 0.02, 0.010), friction=ITEM_FRICTION)


def build_bottle(top: float) -> None:
    r = P.BOTTLE_D / 2.0
    body_h = P.BOTTLE_H - 0.038
    shapes = [
        cylinder(r, body_h, pos=(0.0, 0.0, body_h / 2.0), color=PET),
        cylinder(r - 0.006, 0.020, pos=(0.0, 0.0, body_h + 0.010), color=PET),
        cylinder(0.013, 0.018, pos=(0.0, 0.0, body_h + 0.029), color=PET),
        cylinder(0.0145, 0.012, pos=(0.0, 0.0, P.BOTTLE_H - 0.006), color=CAP),
    ]
    body("bottle", shapes, pos=(*P.BOTTLE_AT, top + P.TRAY_FLOOR + P.OVER), mass=P.BOTTLE_MASS, color=PET,
         center=(0.0, 0.0, 0.050), friction=ITEM_FRICTION)


def build_dish(top: float) -> None:
    shapes = [box((P.DISH_W, P.DISH_L, 0.002), pos=(0.0, 0.0, 0.001), color=DISH_C),
              *_walls(P.DISH_W, P.DISH_L, P.DISH_H - 0.002, 0.0018, 0.002, color=DISH_C)]
    body("dish", shapes, pos=(*P.DISH_AT, top + P.TRAY_FLOOR + P.OVER), mass=P.DISH_MASS, color=DISH_C,
         center=(0.0, 0.0, 0.008), friction=ITEM_FRICTION)


def build_clipboard(top: float) -> None:
    shapes = [
        box((P.CLIP_W, P.CLIP_L, 0.006), pos=(0.0, 0.0, 0.003), color=CLIP_C),
        box((0.136, 0.196, 0.0005), pos=(0.0, -0.006, 0.0063), color=PAPER, solid=False),  # an A5 sheet of paper
        box((0.094, 0.030, 0.012), pos=(0.0, 0.096, 0.012), color=DARK),  # the plastic-spring clip
        box((0.020, 0.026, 0.006), pos=(0.0, 0.112, 0.019), color=DARK),  # its lifting tab
    ]
    body("clipboard", shapes, pos=(*P.CLIP_AT, top + P.TRAY_FLOOR + P.OVER), mass=P.CLIP_MASS, color=CLIP_C,
         center=(0.0, 0.0, 0.005), friction=ITEM_FRICTION)


def build(top: float) -> None:
    build_tray(top)
    build_bin(top)
    build_label(top)
    build_sign(top)
    build_clamp(top)
    build_padlock(top)
    build_shears(top)
    build_bottle(top)
    build_dish(top)
    build_clipboard(top)
