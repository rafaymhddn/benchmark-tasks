"""Bodies and joint of the specimen-pan task, built from ``parts.py``: rigid contact and one free hinge, nothing else.

- ``toilet_bowl`` (fixed): a Kohler K-4302 Highcrest elongated bowl's geometry over the rim at its maker's sizes (the
  rim 368 mm across and 660 mm back to front round the 264 x 379 mm opening), its inside down to the water surface
  152 mm under the rim, the wall behind it at the 10 in. rough-in (27-5/8 in. from the bowl's front) and the two
  stainless seat posts 5-1/2 in. apart; a stand of the bowl's outline puts the rim 160 mm over the bench (it replaces
  the pedestal and the floor only).
- ``toilet_seat`` (free, 1.912 kg): the Bemis 1955CT open-front ring on ``hinge("seat_hinge")``, built raised on its
  check-hinge stop 11 deg past upright (101 deg), where it stays up by itself; pushed forward past its balance at
  94 deg it falls shut onto the rim. It is turned only by gravity, contact and the hands.
- ``specimen_pan`` (free, 80 g): the Medline winged pan, 222.3 x 336.6 x 100.0 mm outside, standing on the bench.
"""

from __future__ import annotations

import math

import mechkit as mk
import numpy as np
import parts as P
from mechkit import body, box, cylinder, hinge

MECHKIT_HELPER = True  # declares for the task file that imports it (mechkit's registry keeps the task file as owner)

COLORS = {"rim": (0.93, 0.93, 0.91), "inside": (0.86, 0.87, 0.86), "water": (0.80, 0.85, 0.86), "stand": (0.62, 0.62, 0.64),
          "wall": (0.84, 0.82, 0.76)}  # fmt: skip
STEEL = (0.62, 0.64, 0.68)
SEAT_WHITE = (0.98, 0.98, 0.99)
BUMPER = (0.80, 0.80, 0.82)
PAN_BLUE = (0.36, 0.66, 0.86)  # a translucent blue pan reads as a plain light blue from above
MARKS = (0.10, 0.12, 0.16)

PAN_BOXES: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = []  # solid boxes of the pan, its frame


def pan_boxes() -> list[tuple[tuple[float, float, float], tuple[float, float, float]]]:
    """The pan's solid boxes (centre, size) in its frame (origin under the middle of its bottom, x front to back,
    y along the wings, z up); none overlap."""
    w, hx, hb, top = P.PAN_WALL, P.PAN_CHORD / 2.0, P.BOWL_IN_W / 2.0, P.PAN_H
    out = [((0.0, 0.0, w / 2.0), (P.PAN_CHORD, P.BOWL_IN_W, w))]  # bottom
    for s in (-1.0, 1.0):
        out.append(((s * (hx - w / 2.0), 0.0, w + (top - w) / 2.0), (w, P.BOWL_IN_W - 2.0 * w, top - w)))  # front, back wall
        side_h = top - P.WING_T - w
        out.append(((0.0, s * (hb - w / 2.0), w + side_h / 2.0), (P.PAN_CHORD, w, side_h)))  # side walls under the wings
        wing_len = P.PAN_SPAN / 2.0 - (hb - w)
        out.append(((0.0, s * (hb - w + wing_len / 2.0), top - P.WING_T / 2.0), (P.WING_CHORD, wing_len, P.WING_T)))  # wings
    return out


def build_bowl(top: float) -> None:
    shapes = [box(size, pos=c, color=COLORS[kind]) for c, size, kind in P.bowl_boxes()]
    for s in (-1.0, 1.0):  # the stainless seat posts (the seat turns on their pins)
        shapes.append(cylinder(0.008, P.PINTLE_H + 0.006, pos=(P.BOLT_X, P.BOWL_Y + s * P.BOLT_SPREAD / 2.0, P.RIM_Z + (P.PINTLE_H + 0.006) / 2.0), color=STEEL, solid=False))
    body("toilet_bowl", shapes, pos=(0.0, 0.0, top), kinematic=True, color=COLORS["rim"], friction=(0.45, 0.40))


def build_seat(top: float) -> None:
    shapes = [box(size, pos=c, color=SEAT_WHITE, friction=mk.GRIPPY) for c, size in P.seat_boxes()]
    shapes += [box(size, pos=c, color=BUMPER, friction=(0.8, 0.7)) for c, size in P.bumper_boxes()]
    for s in (-1.0, 1.0):  # the hinge knuckles round the pins (drawn only)
        shapes.append(cylinder(0.014, 0.030, pos=(s * P.BOLT_SPREAD / 2.0, 0.0, 0.0), axis="x", color=SEAT_WHITE, solid=False))
    com, inertia = P.mass_properties(P.seat_boxes() + P.bumper_boxes(), P.SEAT_MASS)
    pin = tuple(float(v) for v in P.pin(top))
    body("toilet_seat", shapes, pos=pin, quat=tuple(float(v) for v in mk.matrix_to_quat(P.seat_world_rot(P.CHECK_HINGE_DEG))),
         mass=P.SEAT_MASS, color=SEAT_WHITE, friction=mk.GRIPPY, center=tuple(float(v) for v in com),
         inertia=tuple(float(v) for v in inertia), contacts=True)  # fmt: skip
    # Built on its stop (travel 0 = 101 deg); negative travel lowers the front, to just under closed (-103 deg), where
    # the bumpers, not the joint, carry the seat on the rim.
    hinge("seat_hinge", "toilet_seat", "toilet_bowl", anchor=pin, axis=(0.0, 1.0, 0.0), limits=(math.radians(-P.CHECK_HINGE_DEG - 2.0), 0.0), damping=P.SEAT_DAMPING)


def build_pan(top: float) -> None:
    PAN_BOXES[:] = pan_boxes()
    shapes = [box(size, pos=c, color=PAN_BLUE, friction=mk.GRIPPY) for c, size in PAN_BOXES]
    for k, zg in enumerate((0.030, 0.050, 0.070, 0.090)):  # graduation marks inside the back wall (drawn only)
        shapes.append(box((0.0015, 0.050 if k % 2 else 0.030, 0.003), pos=(P.PAN_CHORD / 2.0 - P.PAN_WALL - 0.001, 0.0, zg), color=MARKS, solid=False))
    com, inertia = P.mass_properties(PAN_BOXES, P.PAN_MASS)
    body("specimen_pan", shapes, pos=(P.PAN_START[0], P.PAN_START[1], top + 0.001), quat=tuple(float(v) for v in mk.matrix_to_quat(P.rz(P.PAN_START_YAW_DEG))),
         mass=P.PAN_MASS, color=PAN_BLUE, friction=mk.GRIPPY, center=tuple(float(v) for v in com), inertia=tuple(float(v) for v in inertia), contacts=True)  # fmt: skip
    mk.sense_contact("specimen_pan", "toilet_bowl", "toilet_seat")


def build(top: float) -> None:
    build_bowl(top)
    build_seat(top)
    build_pan(top)


def pan_box_array() -> np.ndarray:
    return np.array([[*c, *s] for c, s in (PAN_BOXES or pan_boxes())])
