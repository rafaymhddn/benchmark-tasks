"""Bodies and joint of the specimen-pan task, built from ``parts.py``: rigid contact and one free hinge, nothing else.

- ``bowl_deck`` (fixed): the 12 mm birch plywood mock bowl rim, top 115 mm over the bench on four 103 mm pine blocks,
  with the 248 x 373 mm opening of a Kohler K-4302 elongated bowl and the two stainless hinge posts 5-1/2 in. apart.
- ``toilet_seat`` (free, 1.912 kg): the Bemis 1955CT open-front ring on ``hinge("seat_hinge")``, from its bumpers on
  the rim (0 deg) to its check-hinge stop 11 deg past upright (101 deg), where it stays up; short of upright (its
  balance is at 94 deg) it falls back. It is turned only by gravity, contact and the hands.
- ``specimen_pan`` (free, 80 g): the Medline winged pan, 222.3 x 336.6 x 100.0 mm outside, standing on the bench.
"""

from __future__ import annotations

import math

import mechkit as mk
import numpy as np
import parts as P
from mechkit import body, box, cylinder, hinge

MECHKIT_HELPER = True  # declares for the task file that imports it (mechkit's registry keeps the task file as owner)

BIRCH = (0.86, 0.74, 0.55)
PINE = (0.74, 0.60, 0.42)
STEEL = (0.62, 0.64, 0.68)
SEAT_WHITE = (0.95, 0.95, 0.96)
BUMPER = (0.80, 0.80, 0.82)
PAN_BLUE = (0.36, 0.66, 0.86)  # a translucent blue pan reads as a plain light blue from above
MARKS = (0.10, 0.12, 0.16)

PAN_BOXES: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = []  # solid boxes of the pan, its frame


def pan_boxes() -> list[tuple[tuple[float, float, float], tuple[float, float, float]]]:
    """The pan's solid boxes (centre, size) in its frame (origin under the middle of its bottom, x front to back,
    y along the wings, z up); none overlap."""
    w, hx, hb, top = P.PAN_WALL, P.PAN_CHORD / 2.0, P.BOWL_W / 2.0, P.PAN_H
    out = [((0.0, 0.0, w / 2.0), (P.PAN_CHORD, P.BOWL_W, w))]  # bottom
    for s in (-1.0, 1.0):
        out.append(((s * (hx - w / 2.0), 0.0, w + (top - w) / 2.0), (w, P.BOWL_W - 2.0 * w, top - w)))  # front, back wall
        side_h = top - P.WING_T - w
        out.append(((0.0, s * (hb - w / 2.0), w + side_h / 2.0), (P.PAN_CHORD, w, side_h)))  # side walls under the wings
        wing_len = P.PAN_SPAN / 2.0 - (hb - w)
        out.append(((0.0, s * (hb - w + wing_len / 2.0), top - P.WING_T / 2.0), (P.WING_CHORD, wing_len, P.WING_T)))  # wings
    return out


def deck_boxes() -> list[tuple[tuple[float, float, float], tuple[float, float, float], bool]]:
    """The deck's solid boxes (centre, size, is_plywood) in its frame (origin on the bench under the middle of the
    hinge line, world axes): the plywood frame round the opening and the four pine riser blocks."""
    t, z = P.DECK_T, P.RIM_Z - P.DECK_T / 2.0
    hw, d1 = P.HOLE_W / 2.0, P.HOLE_D0 + P.HOLE_L

    def plate(x0: float, x1: float, d0: float, d1_: float):
        return ((x0 + x1) / 2.0, -(d0 + d1_) / 2.0, z), (x1 - x0, d1_ - d0, t), True

    out = [plate(-P.RIM_BACK_OUT, P.RIM_BACK_OUT, P.DECK_D0, P.HOLE_D0), plate(-hw, hw, d1, P.DECK_D1)]
    for s in (-1.0, 1.0):
        lo, hi = sorted((s * hw, s * P.RIM_BACK_OUT))
        out.append(plate(lo, hi, P.HOLE_D0, P.BACK_NARROW_D))
        lo, hi = sorted((s * hw, s * P.RIM_OUT))
        out.append(plate(lo, hi, P.BACK_NARROW_D, P.DECK_D1))
    riser = P.RIM_Z - P.DECK_T
    for x, d in ((P.RIM_BACK_OUT - 0.016, P.DECK_D0 + 0.016), (P.RIM_OUT - 0.016, P.DECK_D1 - 0.016)):
        for s in (-1.0, 1.0):
            out.append(((s * x, -d, riser / 2.0), (0.032, 0.032, riser), False))
    return out


def build_deck(top: float) -> None:
    shapes = [box(size, pos=c, color=BIRCH if ply else PINE) for c, size, ply in deck_boxes()]
    for s in (-1.0, 1.0):  # the stainless hinge posts (the seat turns on their pins)
        shapes.append(cylinder(0.008, P.PINTLE_H + 0.006, pos=(s * P.BOLT_SPREAD / 2.0, 0.0, P.RIM_Z + (P.PINTLE_H + 0.006) / 2.0), color=STEEL, solid=False))
    body("bowl_deck", shapes, pos=(P.HOLE_X, P.PINTLE_Y, top), kinematic=True, color=BIRCH, friction=(0.45, 0.40))


def build_seat(top: float) -> None:
    shapes = [box(size, pos=c, color=SEAT_WHITE, friction=mk.GRIPPY) for c, size in P.seat_boxes()]
    shapes += [box(size, pos=c, color=BUMPER, friction=(0.8, 0.7)) for c, size in P.bumper_boxes()]
    for s in (-1.0, 1.0):  # the hinge knuckles round the pins (drawn only)
        shapes.append(cylinder(0.014, 0.030, pos=(s * P.BOLT_SPREAD / 2.0, 0.0, 0.0), axis="x", color=SEAT_WHITE, solid=False))
    com, inertia = P.mass_properties(P.seat_boxes() + P.bumper_boxes(), P.SEAT_MASS)
    pin = (P.HOLE_X, P.PINTLE_Y, top + P.PINTLE_Z)
    body("toilet_seat", shapes, pos=pin, mass=P.SEAT_MASS, color=SEAT_WHITE, friction=mk.GRIPPY,
         center=tuple(float(v) for v in com), inertia=tuple(float(v) for v in inertia), contacts=True)  # fmt: skip
    # Positive travel raises the front; just below 0 the bumpers (not the joint) carry the seat on the rim.
    hinge("seat_hinge", "toilet_seat", "bowl_deck", anchor=pin, axis=(-1.0, 0.0, 0.0), limits=(math.radians(-2.0), math.radians(P.CHECK_HINGE_DEG)), damping=P.SEAT_DAMPING)


def build_pan(top: float) -> None:
    PAN_BOXES[:] = pan_boxes()
    shapes = [box(size, pos=c, color=PAN_BLUE, friction=mk.GRIPPY) for c, size in PAN_BOXES]
    for k, zg in enumerate((0.030, 0.050, 0.070, 0.090)):  # graduation marks inside the back wall (drawn only)
        shapes.append(box((0.0015, 0.050 if k % 2 else 0.030, 0.003), pos=(P.PAN_CHORD / 2.0 - P.PAN_WALL - 0.001, 0.0, zg), color=MARKS, solid=False))
    com, inertia = P.mass_properties(PAN_BOXES, P.PAN_MASS)
    body("specimen_pan", shapes, pos=(P.PAN_START[0], P.PAN_START[1], top + 0.001), quat=tuple(float(v) for v in mk.matrix_to_quat(P.rz(P.PAN_START_YAW_DEG))),
         mass=P.PAN_MASS, color=PAN_BLUE, friction=mk.GRIPPY, center=tuple(float(v) for v in com), inertia=tuple(float(v) for v in inertia), contacts=True)  # fmt: skip
    mk.sense_contact("specimen_pan", "bowl_deck", "toilet_seat")


def build(top: float) -> None:
    build_deck(top)
    build_seat(top)
    build_pan(top)


def pan_box_array() -> np.ndarray:
    return np.array([[*c, *s] for c, s in (PAN_BOXES or pan_boxes())])
