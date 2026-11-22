"""Rigid bodies and joints of the tympanic thermometer probe-cover turnaround scene (figures in ``parts.py``).

Bodies:
- ``start_pad`` (fixed): stepped rest on the -y side holding the thermometer level, probe towards +y.
- ``thermometer`` (free) + ``eject_btn`` (free, sprung ``slide`` into the head): Riester ri-thermo tymPRO Art. 1835,
  a 42 mm body tapering to a narrower head, the probe leaving it 55 deg below the body's axis (collar, 14.0 mm seating
  ring, 8.4 mm tip), the dark-grey eject button in a recess in the head's end, above the probe's root.
- ``cover_used`` (free, amber, ``contacts=True``): on the probe, held by ``weld_used`` until the button's stroke >= 4.0 mm.
- ``disposal_tray`` (free, grey): 200 x 150 x 20 mm.
- ``cradle`` (free): Riester's open trough, 170 x 56 mm, walls 44 mm along the rack and 58 mm at the probe's pocket
  end, with the white 2 x 5 rack on its floor and a cross rib in the pocket; the docked thermometer lies along it
  over the rack, its back resting on the low end wall and the rib, its covered probe in the pocket.
- ``cover_fresh_0..9`` (free, white, ``contacts=True`` + ``sense_contact`` with ``thermometer``): one per well, held
  by ``weld_well_i``; a coaxial press of the probe's ring on a cover's mouth (>= 3.5 N) moves it to ``weld_cone_i``.
"""

from __future__ import annotations

import numpy as np

import parts as P
from mechkit import body, box, cylinder, matrix_to_quat, sense_contact, slide, weld

MECHKIT_HELPER = True

WHITE = (0.94, 0.95, 0.97)
CLEAN_WHITE = (0.99, 0.99, 1.00)
AMBER = (0.93, 0.53, 0.07)
GREY_TRAY = (0.48, 0.51, 0.55)
CRADLE_C = (0.93, 0.94, 0.95)
LINING = (0.20, 0.21, 0.23)
RACK_C = (0.97, 0.97, 0.98)
BTN_GREY = (0.22, 0.24, 0.27)
PROBE_C = (0.80, 0.81, 0.83)
SCREEN_DARK = (0.14, 0.17, 0.21)
PAD_SLATE = (0.24, 0.27, 0.32)

GRIP_FRIC = (1.0, 0.85)
CRADLE_FRIC = (P.CRADLE_FRICTION, P.CRADLE_FRICTION * 0.9)
TRAY_FRIC = (0.55, 0.45)
PROBE_QUAT = matrix_to_quat(P.COVER_ON_PROBE_ROT_T)  # a cylinder's axis along the probe


def _walls(dx: float, dy: float, h: float, thick: float, z0: float, color=None) -> list:
    kw = {"color": color} if color is not None else {}
    return [
        box((dx, thick, h), pos=(0.0, dy / 2.0 - thick / 2.0, z0 + h / 2.0), **kw),
        box((dx, thick, h), pos=(0.0, -(dy / 2.0 - thick / 2.0), z0 + h / 2.0), **kw),
        box((thick, dy - 2.0 * thick, h), pos=(dx / 2.0 - thick / 2.0, 0.0, z0 + h / 2.0), **kw),
        box((thick, dy - 2.0 * thick, h), pos=(-(dx / 2.0 - thick / 2.0), 0.0, z0 + h / 2.0), **kw),
    ]


def _cover_shapes(color) -> list:
    """Hollow probe cover: closed end at z = 0, mouth at z = COVER_H (17.6 mm flange, 15.8 mm sleeve, 13.8 mm mouth)."""
    sleeve_t = (P.COVER_BODY_W - P.COVER_BORE) / 2.0
    flange_t = (P.COVER_FLANGE - P.COVER_BORE) / 2.0
    sleeve_h = P.COVER_H - P.COVER_FLOOR_T - 0.0015
    return [
        box((P.COVER_BODY_W, P.COVER_BODY_W, P.COVER_FLOOR_T), pos=(0.0, 0.0, P.COVER_FLOOR_T / 2.0), color=color),
        *_walls(P.COVER_BODY_W, P.COVER_BODY_W, sleeve_h, sleeve_t, P.COVER_FLOOR_T, color=color),
        *_walls(P.COVER_FLANGE, P.COVER_FLANGE, 0.0015, flange_t, P.COVER_H - 0.0015, color=color),
    ]


def _along_probe(start: float, length: float) -> tuple:
    """The middle of a probe section from ``start`` to ``start + length`` along the probe, in T."""
    return tuple(P.PROBE_BASE_T + (start + length / 2.0) * P.PROBE_DIR_T)


def build_start_pad(top: float) -> None:
    """Fixed rest: two pillars under the body (T y = -30 and +22 mm), the tail and the covered probe free."""
    h = P.PAD_SADDLE_Z
    shapes = [
        box((0.056, 0.074, 0.004), pos=(0.0, -0.004, 0.002), color=PAD_SLATE),
        box((0.040, 0.014, h - 0.004), pos=(0.0, -0.030, 0.004 + (h - 0.004) / 2.0), color=PAD_SLATE),
        box((0.040, 0.016, h - 0.004), pos=(0.0, 0.022, 0.004 + (h - 0.004) / 2.0), color=PAD_SLATE),
        box((0.006, 0.016, 0.010), pos=(0.026, 0.022, h + 0.005), color=PAD_SLATE),
        box((0.006, 0.016, 0.010), pos=(-0.026, 0.022, h + 0.005), color=PAD_SLATE),
    ]
    body("start_pad", shapes, pos=(P.PAD_AT[0], P.PAD_AT[1], top), mass=None, color=PAD_SLATE, friction=CRADLE_FRIC)


def build_thermometer(top: float) -> None:
    """Riester ri-thermo tymPRO (Art. 1835): 150 x 42 x 56 mm, 80 g, level on its rest with the probe towards +y."""
    tx, ty, tz = P.THERM_START[0], P.THERM_START[1], top + P.THERM_START[2] + P.OVER
    by0, by1 = P.BODY_Y
    hy0, hy1 = P.HEAD_Y
    shapes = [
        box((P.THERM_W, by1 - by0, P.BODY_T), pos=(0.0, (by0 + by1) / 2.0, P.BODY_T / 2.0), color=WHITE),
        # the head, with a recess in its end for the eject button
        box((P.HEAD_W, P.RECESS_Y0 - hy0, P.HEAD_T), pos=(0.0, (hy0 + P.RECESS_Y0) / 2.0, P.HEAD_T / 2.0), color=WHITE),
        *[box((P.HEAD_W / 2.0 - P.RECESS_HALF_X, hy1 - P.RECESS_Y0, P.HEAD_T),
              pos=(sx * (P.HEAD_W / 2.0 + P.RECESS_HALF_X) / 2.0, (P.RECESS_Y0 + hy1) / 2.0, P.HEAD_T / 2.0), color=WHITE) for sx in (-1, 1)],
        box((2.0 * P.RECESS_HALF_X, hy1 - P.RECESS_Y0, P.RECESS_Z[0]), pos=(0.0, (P.RECESS_Y0 + hy1) / 2.0, P.RECESS_Z[0] / 2.0), color=WHITE),
        box((2.0 * P.RECESS_HALF_X, hy1 - P.RECESS_Y0, P.HEAD_T - P.RECESS_Z[1]),
            pos=(0.0, (P.RECESS_Y0 + hy1) / 2.0, (P.RECESS_Z[1] + P.HEAD_T) / 2.0), color=WHITE),
        box((0.026, 0.034, 0.001), pos=(0.0, -0.002, P.BODY_T + 0.0006), color=SCREEN_DARK, solid=False),  # display
        cylinder(P.COLLAR_R, P.COLLAR_LEN, pos=_along_probe(0.0, P.COLLAR_LEN), quat=PROBE_QUAT, color=PROBE_C),
        cylinder(P.RING_D / 2.0, P.RING_LEN, pos=_along_probe(P.COLLAR_LEN, P.RING_LEN), quat=PROBE_QUAT, color=PROBE_C),
        cylinder(P.TIP_D / 2.0, P.TIP_LEN, pos=_along_probe(P.RING_FACE, P.TIP_LEN), quat=PROBE_QUAT, color=PROBE_C),
    ]
    body("thermometer", shapes, pos=(tx, ty, tz), mass=P.THERM_MASS, color=WHITE,
         center=(0.0, -0.008, 0.015), friction=GRIP_FRIC, linear_damping=0.5, angular_damping=0.8)

    bx, by, bz = P.BTN_POS_LOCAL
    body("eject_btn", [box(P.BTN_SIZE, pos=(0.0, 0.0, 0.0), color=BTN_GREY)],
         pos=(tx + bx, ty + by, tz + bz), quat=matrix_to_quat(P.BTN_ROT_T), mass=P.BTN_MASS, color=BTN_GREY, friction=GRIP_FRIC)
    slide("eject_slide", "eject_btn", "thermometer", axis=tuple(P.BTN_AXIS_T),
          limits=(0.0, P.BTN_STROKE), spring=P.BTN_SPRING, rest=0.0, damping=P.BTN_DAMPING)

    ux, uy, uz = P.USED_ON_THERM_LOCAL
    body("cover_used", _cover_shapes(AMBER), pos=(tx + ux, ty + uy, tz + uz), quat=PROBE_QUAT,
         mass=P.COVER_MASS, color=AMBER, friction=TRAY_FRIC, contacts=True, linear_damping=0.5, angular_damping=0.5)
    mid = np.array([tx + ux, ty + uy, tz + uz]) + (P.COVER_H / 2.0) * P.PROBE_UP_T
    weld("weld_used", "cover_used", "thermometer", anchor=tuple(mid), made=True)


def build_disposal_tray(top: float) -> None:
    shapes = [
        box((P.TRAY_L, P.TRAY_W, P.TRAY_FLOOR), pos=(0.0, 0.0, P.TRAY_FLOOR / 2.0), color=GREY_TRAY),
        *_walls(P.TRAY_L, P.TRAY_W, P.TRAY_H - P.TRAY_FLOOR, P.TRAY_WALL, P.TRAY_FLOOR, color=GREY_TRAY),
    ]
    body("disposal_tray", shapes, pos=(P.TRAY_AT[0], P.TRAY_AT[1], top + P.OVER),
         mass=P.TRAY_MASS, color=GREY_TRAY, friction=CRADLE_FRIC)


def build_cradle_and_rack(top: float) -> None:
    cx, cy, cz = P.CRADLE_AT[0], P.CRADLE_AT[1], top + P.OVER
    w, half_l, t, f = P.CRADLE_W, P.CRADLE_L / 2.0, P.CRADLE_WALL, P.CRADLE_FLOOR
    low, high, split = P.CRADLE_LOW_H, P.CRADLE_H, P.POCKET_FROM
    inner_w = w - 2.0 * t
    low_len, high_len = split + half_l, half_l - split
    shapes = [
        box((w, P.CRADLE_L, f), pos=(0.0, 0.0, f / 2.0), color=CRADLE_C),  # floor
        *[box((t, low_len, low - f), pos=(sx * (w - t) / 2.0, (split - half_l) / 2.0, f + (low - f) / 2.0), color=CRADLE_C) for sx in (-1, 1)],
        *[box((t, high_len, high - f), pos=(sx * (w - t) / 2.0, (split + half_l) / 2.0, f + (high - f) / 2.0), color=CRADLE_C) for sx in (-1, 1)],
        box((inner_w, t, low - f), pos=(0.0, -half_l + t / 2.0, f + (low - f) / 2.0), color=CRADLE_C),  # rack end wall
        box((inner_w, t, high - f), pos=(0.0, half_l - t / 2.0, f + (high - f) / 2.0), color=CRADLE_C),  # pocket end wall
        box((inner_w, P.RIB_Y[1] - P.RIB_Y[0], low - f), pos=(0.0, sum(P.RIB_Y) / 2.0, f + (low - f) / 2.0), color=LINING),  # rib
        box((inner_w - 0.001, half_l - P.RIB_Y[1] - t, 0.0005), pos=(0.0, (P.RIB_Y[1] + half_l - t) / 2.0, f + 0.0003), color=LINING, solid=False),
        box((2.0 * P.RACK_HALF_X, P.RACK_Y[1] - P.RACK_Y[0], P.RACK_BASE_Z[1] - P.RACK_BASE_Z[0]),
            pos=(0.0, sum(P.RACK_Y) / 2.0, sum(P.RACK_BASE_Z) / 2.0), color=RACK_C),  # rack base plate
        box((2.0 * P.RACK_HALF_X, P.RACK_Y[1] - P.RACK_Y[0], 0.0150), pos=(0.0, sum(P.RACK_Y) / 2.0, P.RACK_BASE_Z[1] + 0.0075),
            color=RACK_C, solid=False),  # the rack's top, drawn only: the covers' mouths stand 1.5 mm above it
    ]
    body("cradle", shapes, pos=(cx, cy, cz), mass=P.CRADLE_MASS, color=CRADLE_C, friction=CRADLE_FRIC)

    for i, (wx, wy) in enumerate(P.WELLS):
        name = f"cover_fresh_{i}"
        fx, fy, fz = cx + wx, cy + wy, cz + P.RACK_BASE_Z[1]
        body(name, _cover_shapes(CLEAN_WHITE), pos=(fx, fy, fz),
             mass=P.COVER_MASS, color=CLEAN_WHITE, friction=TRAY_FRIC, contacts=True,
             linear_damping=0.5, angular_damping=0.5)
        weld(f"weld_well_{i}", name, "cradle", anchor=(fx, fy, fz + 0.002), made=True)
        weld(f"weld_cone_{i}", name, "thermometer", anchor=(fx, fy, fz + P.COVER_H / 2.0), made=False)
        sense_contact(name, "thermometer")


def build(top: float) -> None:
    build_start_pad(top)
    build_thermometer(top)
    build_disposal_tray(top)
    build_cradle_and_rack(top)
