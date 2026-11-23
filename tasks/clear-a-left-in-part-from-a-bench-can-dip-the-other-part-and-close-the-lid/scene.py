"""Bodies and joints of the bench-can task, built from ``parts.py``:
- ``mat`` (kinematic): 120 x 120 x 3 mm black nitrile-rubber pad holding ``brg_mat`` at the start, plus bench marks;
- ``tray`` (free, 0.24 kg): Bochem Art. 8480 18/10 stainless steel instrument tray (200 x 175 x 15 mm);
- ``can`` (free, 0.68 kg body share of Eagle B-601's 0.98 kg): hollow 16-stave red steel cylinder (158.8 x 92.1 mm),
  galvanized 118 mm ID retaining collar, and spot-welded horizontal front strap D-handle;
- ``dasher`` (0.12 kg): guided perforated brass screen on a 16 mm vertical ``slide`` with 1,125 N/m spring (2.2 N preload);
- ``lid`` (0.18 kg): Ø 162 mm saucer cover on a ``0..172 deg`` dry-friction ``hinge`` (starts open at 172 deg);
- ``brg_screen`` and ``brg_mat`` (0.35 kg each): two SKF 3305 A-2Z bearings (d 25, D 62, B 25.4 mm).
"""

from __future__ import annotations

import math

import numpy as np

import mechkit as mk
import parts as P
from mechkit import body, box, cylinder, hinge, slide

MECHKIT_HELPER = True

EAGLE_RED = (0.78, 0.11, 0.10)
GALV_STEEL = (0.70, 0.72, 0.75)
BRASS_GOLD = (0.79, 0.64, 0.22)
PERF_DARK = (0.18, 0.16, 0.10)
STAINLESS = (0.76, 0.78, 0.82)
NBR_BLACK = (0.11, 0.12, 0.13)
BRG_STEEL = (0.64, 0.67, 0.71)
SHIELD_GREY = (0.44, 0.46, 0.50)
BORE_DARK = (0.15, 0.16, 0.18)
MARK_YELLOW = (0.95, 0.80, 0.12)


def _floats(v) -> tuple[float, ...]:
    return tuple(float(x) for x in v)


def _quat(matrix: np.ndarray) -> tuple[float, ...]:
    return _floats(mk.matrix_to_quat(matrix))


def can_start_pos(top: float) -> np.ndarray:
    return np.array([P.CAN_XY[0], P.CAN_XY[1], top + 0.0005], dtype=float)


def tray_start_pos(top: float) -> np.ndarray:
    return np.array([P.TRAY_XY[0], P.TRAY_XY[1], top + 0.0005], dtype=float)


def hinge_world_pos(top: float) -> np.ndarray:
    u_h, _ = P.hinge_unit_vectors()
    return can_start_pos(top) + P.HINGE_R * u_h + np.array([0.0, 0.0, P.HINGE_Z], dtype=float)


def handle_world_pos(top: float) -> np.ndarray:
    u_d = P.handle_unit_vector()
    return can_start_pos(top) + (P.CAN_R + P.HANDLE_OUT - 0.0035) * u_d + np.array([0.0, 0.0, P.HANDLE_Z], dtype=float)


def build_mat_and_marks(top: float) -> None:
    mx, my = P.MAT_XY
    shapes = [box((P.MAT_SIDE, P.MAT_SIDE, P.MAT_T), color=NBR_BLACK, friction=(0.85, 0.75))]
    # Visual yellow reference ticks around can and tray start positions on the bench (drawn only)
    for ang in (0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi):
        px = P.CAN_XY[0] + (P.CAN_R + 0.008) * math.cos(ang) - mx
        py = P.CAN_XY[1] + (P.CAN_R + 0.008) * math.sin(ang) - my
        shapes.append(box((0.010, 0.006, 0.0008), pos=(px, py, -0.5 * P.MAT_T + 0.0004), yaw=ang,
                          color=MARK_YELLOW, solid=False))
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            px = P.TRAY_XY[0] + sx * (0.5 * P.TRAY_L + 0.006) - mx
            py = P.TRAY_XY[1] + sy * (0.5 * P.TRAY_W + 0.006) - my
            shapes.append(box((0.010, 0.010, 0.0008), pos=(px, py, -0.5 * P.MAT_T + 0.0004),
                              color=MARK_YELLOW, solid=False))
    body("mat", shapes, pos=(mx, my, top + 0.5 * P.MAT_T), kinematic=True, color=NBR_BLACK)


def build_tray(top: float) -> None:
    wall_h = P.TRAY_H - P.TRAY_FLOOR
    wz = P.TRAY_FLOOR + 0.5 * wall_h
    shapes = [
        box((P.TRAY_L, P.TRAY_W, P.TRAY_FLOOR), pos=(0.0, 0.0, 0.5 * P.TRAY_FLOOR), color=STAINLESS,
            friction=(P.TRAY_MU, 0.45)),
        box((P.TRAY_L, P.TRAY_WALL, wall_h), pos=(0.0, 0.5 * (P.TRAY_W - P.TRAY_WALL), wz), color=STAINLESS),
        box((P.TRAY_L, P.TRAY_WALL, wall_h), pos=(0.0, -0.5 * (P.TRAY_W - P.TRAY_WALL), wz), color=STAINLESS),
        box((P.TRAY_WALL, P.TRAY_W - 2 * P.TRAY_WALL, wall_h), pos=(0.5 * (P.TRAY_L - P.TRAY_WALL), 0.0, wz), color=STAINLESS),
        box((P.TRAY_WALL, P.TRAY_W - 2 * P.TRAY_WALL, wall_h), pos=(-0.5 * (P.TRAY_L - P.TRAY_WALL), 0.0, wz), color=STAINLESS),
    ]
    body("tray", shapes, pos=_floats(tray_start_pos(top)), mass=P.TRAY_MASS, color=STAINLESS,
         friction=(P.TRAY_MU, 0.45))


def build_can_dasher_lid(top: float) -> None:
    c_pos = can_start_pos(top)
    n_staves = 16
    wall_t = 0.006
    r_wall = P.CAN_R - 0.5 * wall_t
    chord_wall = 2.0 * P.CAN_R * math.tan(math.pi / n_staves) + 0.002
    wall_h = P.CAN_H - 0.004

    collar_t = P.CAN_R - P.COLLAR_IN_R  # 0.0204 m annular retaining ring
    r_collar = P.COLLAR_IN_R + 0.5 * collar_t
    chord_collar = 2.0 * P.CAN_R * math.tan(math.pi / n_staves) + 0.001

    can_shapes = [
        cylinder(P.CAN_R, 0.006, pos=(0.0, 0.0, 0.003), color=EAGLE_RED,
                 friction=(P.CAN_BENCH_MU, 0.22), friction_combine="min"),
    ]
    for i in range(n_staves):
        ang = 2.0 * math.pi * i / n_staves
        ca, sa = math.cos(ang), math.sin(ang)
        can_shapes.append(box((wall_t, chord_wall, wall_h), pos=(r_wall * ca, r_wall * sa, 0.5 * wall_h),
                              yaw=ang, color=EAGLE_RED, friction=mk.GRIPPY))
        can_shapes.append(box((collar_t, chord_collar, 0.004), pos=(r_collar * ca, r_collar * sa, P.CAN_H - 0.002),
                              yaw=ang, color=GALV_STEEL, friction=(0.35, 0.30)))

    # Welded horizontal front D-handle at PHI_HANDLE_DEG (215 deg)
    u_d = P.handle_unit_vector()
    pd = math.radians(P.PHI_HANDLE_DEG)
    t_d = np.array([-u_d[1], u_d[0], 0.0], dtype=float)
    r_strap = P.CAN_R + P.HANDLE_OUT - 0.0035
    can_shapes.append(box((0.007, P.HANDLE_SPAN, P.HANDLE_H),
                          pos=_floats(r_strap * u_d + np.array([0.0, 0.0, P.HANDLE_Z])),
                          yaw=pd, color=EAGLE_RED, friction=mk.GRIPPY))
    for s in (-1.0, 1.0):
        leg_mid = (P.CAN_R + 0.5 * P.HANDLE_OUT - 0.002) * u_d + s * (0.5 * P.HANDLE_SPAN - 0.004) * t_d
        can_shapes.append(box((P.HANDLE_OUT, 0.008, P.HANDLE_H),
                              pos=_floats(leg_mid + np.array([0.0, 0.0, P.HANDLE_Z])),
                              yaw=pd, color=EAGLE_RED, friction=mk.GRIPPY))
    # Yellow FM diamond decal on front wall (drawn only)
    can_shapes.append(box((0.001, 0.022, 0.022), pos=_floats((P.CAN_R + 0.0006) * np.array([-1.0, 0.0, 0.0]) + np.array([0.0, 0.0, 0.038])),
                          quat=_quat(mk.axis_angle((1.0, 0.0, 0.0), 0.25 * math.pi)), color=MARK_YELLOW, solid=False))
    body("can", can_shapes, pos=_floats(c_pos), mass=P.CAN_BODY_MASS, color=EAGLE_RED,
         friction=(P.CAN_BENCH_MU, 0.22), contacts=True)

    # Guided perforated brass dasher screen (child of can)
    dash_shapes = [cylinder(P.DASHER_R, P.DASHER_T, color=BRASS_GOLD, friction=(0.60, 0.50))]
    dash_shapes.append(cylinder(0.007, 0.0008, pos=(0.0, 0.0, 0.5 * P.DASHER_T + 0.0004), color=PERF_DARK, solid=False))
    for i in range(6):
        ang = 2.0 * math.pi * i / 6.0
        dash_shapes.append(cylinder(0.006, 0.0008, pos=(0.034 * math.cos(ang), 0.034 * math.sin(ang), 0.5 * P.DASHER_T + 0.0004),
                                    color=PERF_DARK, solid=False))
    dash_pos = c_pos + np.array([0.0, 0.0, P.DASH_REST_Z - 0.5 * P.DASHER_T], dtype=float)
    body("dasher", dash_shapes, pos=_floats(dash_pos), mass=P.DASH_MASS, color=BRASS_GOLD,
         friction=(0.60, 0.50), contacts=True)

    # Hinged Ø 165 mm saucer cover, built directly in its 172 deg OPEN start pose (travel 0 = 172 deg open,
    # travel -172 deg = 0 deg closed flat; zero set_joint or write_pose in task.py / scene.py / demo.py)
    lid_shapes = [
        cylinder(P.LID_R, P.LID_ROOF_T, pos=(0.0, 0.0, 0.5 * P.LID_ROOF_T), color=EAGLE_RED, friction=(0.25, 0.20)),
    ]
    skirt_r = P.LID_R - 0.0015
    skirt_chord = 2.0 * P.LID_R * math.tan(math.pi / n_staves) + 0.001
    for i in range(n_staves):
        ang = 2.0 * math.pi * i / n_staves
        lid_shapes.append(box((0.004, skirt_chord, P.LID_SKIRT_H),
                              pos=(skirt_r * math.cos(ang), skirt_r * math.sin(ang), P.LID_ROOF_T - 0.5 * P.LID_SKIRT_H),
                              yaw=ang, color=EAGLE_RED, friction=mk.GRIPPY))
    _, t_h = P.hinge_unit_vectors()
    hinge_w = hinge_world_pos(top)
    closed_pos = c_pos + np.array([0.0, 0.0, P.CAN_H], dtype=float)
    r_open = mk.axis_angle(t_h, math.radians(P.LID_OPEN_DEG))
    open_pos = hinge_w + r_open @ (closed_pos - hinge_w)
    body("lid", lid_shapes, pos=_floats(open_pos), quat=_quat(r_open), mass=P.LID_MASS,
         color=EAGLE_RED, friction=mk.GRIPPY, contacts=True)

    slide("dasher", "dasher", "can", axis=(0.0, 0.0, -1.0), limits=(0.0, P.DASH_STROKE),
          spring=P.DASH_K, rest=-P.DASH_PRELOAD_M, damping=P.DASH_DAMPING)
    hinge("lid", "lid", "can", anchor=_floats(hinge_w), axis=_floats(t_h),
          limits=(-math.radians(P.LID_OPEN_DEG), 0.0), friction=P.LID_FRICTION_NM)


def _bearing_shapes() -> list:
    shapes = [cylinder(P.BRG_R, P.BRG_W, color=BRG_STEEL, friction=mk.GRIPPY)]
    for s in (-1.0, 1.0):
        zf = s * (0.5 * P.BRG_W + 0.0004)
        shapes.append(cylinder(0.0255, 0.0008, pos=(0.0, 0.0, zf), color=SHIELD_GREY, solid=False))
        shapes.append(cylinder(0.0175, 0.0010, pos=(0.0, 0.0, zf), color=BRG_STEEL, solid=False))
        shapes.append(cylinder(0.5 * P.BRG_ID, 0.0012, pos=(0.0, 0.0, zf), color=BORE_DARK, solid=False))
    return shapes


def build_bearings(top: float) -> None:
    c_pos = can_start_pos(top)
    screen_pos = (float(c_pos[0]), float(c_pos[1]), float(c_pos[2]) + P.DASH_REST_Z + 0.5 * P.BRG_W + 0.0008)
    mat_pos = (P.MAT_XY[0], P.MAT_XY[1], top + P.MAT_T + 0.5 * P.BRG_W + 0.0008)
    body("brg_screen", _bearing_shapes(), pos=screen_pos, mass=P.BRG_MASS, color=BRG_STEEL,
         friction=mk.GRIPPY, contacts=True)
    body("brg_mat", _bearing_shapes(), pos=mat_pos, mass=P.BRG_MASS, color=BRG_STEEL,
         friction=mk.GRIPPY, contacts=True)
    mk.sense_contact("dasher", "brg_screen", "brg_mat")
    mk.sense_contact("lid", "brg_screen", "brg_mat")


def build(top: float) -> None:
    build_mat_and_marks(top)
    build_tray(top)
    build_can_dasher_lid(top)
    build_bearings(top)
