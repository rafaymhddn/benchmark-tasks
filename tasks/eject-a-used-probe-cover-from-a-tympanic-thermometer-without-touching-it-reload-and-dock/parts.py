"""Every figure the tympanic thermometer probe-cover turnaround task rests on, each with its source.

Source kinds (``FIGURES`` holds every figure with its kind and its source in words):
- ``maker``: the maker's published figure on ``riester.de``, opened and quoted verbatim.
- ``our fixture``: a bench fixture this task specifies to the stated size (the grey disposal tray and the start rest).
- ``estimate``: our figure, with how it was derived in one clause. Sizes read off the maker's photos on the same page
  ('Riester tymPRO 01 Photo': the thermometer docked over the cassette; '02 Photo': the open cradle with its 2 x 5
  rack; 'riester-tympro-c-1': the head, the angled probe and the dark-grey ribbed eject button; 'riester-tympro-
  clinical-01-photo-2': a gloved left hand holding the cradle by its side walls) are scaled against the maker's
  150 mm length and 42 mm width.

Frames. World: metres from the midpoint between the arm bases, x forward, y left, z up from the bench. Thermometer
frame T: x across its 42 mm width, y along it towards the probe, z out of the display face; origin on the back face.
Cradle frame C: x across its 56 mm width, y along its 170 mm length towards the pocket end, z up from its underside.
"""

from __future__ import annotations

import math

import numpy as np

MAKER, FIXTURE, ESTIMATE = "maker", "our fixture", "estimate"
FIGURES: dict[str, tuple[float, str, str, str]] = {}

RIESTER_URL = "https://riester.de/products/thermometry/ri-thermo-tympro-tympanic-thermometer"
ONET_CNA_URL = "https://www.onetonline.org/link/summary/31-1131.00"
ONET_MA_URL = "https://www.onetonline.org/link/summary/31-9092.00"


def figure(name: str, value: float, unit: str, source: str, note: str) -> float:
    if source not in (MAKER, FIXTURE, ESTIMATE):
        raise ValueError(f"{name}: unknown source kind {source!r}")
    FIGURES[name] = (float(value), unit, source, note)
    return float(value)


def rot_frame(closing, approach) -> np.ndarray:
    """Gripper 3x3 rotation: columns are closing axis, pad-width axis, approach axis."""
    a = np.asarray(approach, dtype=float) / np.linalg.norm(approach)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def rot_x(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


OVER = 0.0008  # m: loose bodies spawn slightly above their support to settle cleanly
BENCH_X = (-0.190, 0.910)  # m: station bench x span (1.10 m centred at 0.360)
BENCH_HALF_Y = 0.650  # m: station bench half-width (1.30 m)

# --- Riester ri-thermo tymPRO clinical tympanic thermometer (Art. No. 1835) -------------------------------------------------
THERM_L = figure("thermometer overall length", 0.150, "m", MAKER, f"Riester Art. 1835 'Dimension / Weight 150 (L) x 42 (W) x 56 (H) mm / 80 g' ({RIESTER_URL})")
THERM_W = figure("thermometer width", 0.042, "m", MAKER, "42 (W) mm on the same Riester Art. 1835 line")
THERM_H = figure("thermometer overall height", 0.056, "m", MAKER, "56 (H) mm on the same Riester Art. 1835 line (body and angled probe)")
THERM_MASS_TOTAL = figure("thermometer mass", 0.080, "kg", MAKER, "80 g on the same Riester Art. 1835 line")
BTN_MASS = 0.004  # kg: the eject button's share of the 80 g (a moulded cap)
THERM_MASS = THERM_MASS_TOTAL - BTN_MASS
BODY_T = figure("body thickness", 0.032, "m", ESTIMATE, "the handle is about three quarters of its 42 mm width thick in 'riester-tympro-c-1'")
BODY_Y = (-0.071, 0.040)  # m in T: the 42 mm wide body, tail to head
HEAD_W = figure("head width at the probe", 0.030, "m", ESTIMATE, "the head narrows to about 0.7 of the 42 mm width at the probe in 'riester-tympro-c-1'")
HEAD_Y, HEAD_T = (0.040, 0.058), 0.028  # m in T: the tapered head ahead of the body, its thickness
PROBE_ANGLE_DEG = figure("probe angle below the body's axis", 55.0, "deg", ESTIMATE,
                         "the probe points forward and away from the display, about 55 deg off the body's axis in 'riester-tympro-c-1' and the in-ear clinical photo; with the 32 mm body it makes the 56 mm height")
PROBE_BASE_T = np.array([0.0, HEAD_Y[1], 0.003])  # m in T: where the probe leaves the head
PROBE_DIR_T = np.array([0.0, math.cos(math.radians(PROBE_ANGLE_DEG)), -math.sin(math.radians(PROBE_ANGLE_DEG))])  # base -> tip
COLLAR_R, COLLAR_LEN = 0.008, 0.012  # m: the probe's base collar
RING_D = figure("probe seating ring diameter", 0.0140, "m", ESTIMATE, "the cone's widest part, about a third of the 42 mm width in 'riester-tympro-c-1'")
RING_LEN = 0.005  # m
TIP_D = figure("probe tip diameter", 0.0084, "m", ESTIMATE, "the ear tip about a fifth of the 42 mm width in 'riester-tympro-c-1'")
TIP_LEN = figure("probe tip length beyond the ring", 0.013, "m", ESTIMATE, "the cone ahead of its seating ring, so that the probe ends 150 mm from the tail")
PROBE_LEN = COLLAR_LEN + RING_LEN + TIP_LEN  # 0.030 m
RING_FACE = COLLAR_LEN + RING_LEN  # 0.017 m along the probe: where a cover's mouth stops
CONE_TIP_T = PROBE_BASE_T + PROBE_LEN * PROBE_DIR_T  # (0, 0.0752, -0.0216) m
PROBE_UP_T = -PROBE_DIR_T  # a seated cover's axis, floor -> mouth

BTN_W = figure("eject button width", 0.020, "m", ESTIMATE, "about two thirds of the head's 30 mm width, on the head's end just above the probe, in 'riester-tympro-c-1'")
BTN_H = figure("eject button height", 0.012, "m", ESTIMATE, "the ribbed button fills the end between the probe's root and the head's display-side edge in 'riester-tympro-c-1'")
BTN_SIZE = (BTN_W, 0.003, BTN_H)  # m: the dark-grey eject button (x, thickness along the body, z), its face 1.5 mm proud
BTN_POS_LOCAL = (0.0, HEAD_Y[1], 0.0175)  # m in T: its middle, on the head's end above the probe's root (z 3 mm)
BTN_TILT_DEG = 10.0  # its face looks 10 deg up, towards the display side, from the body's axis (estimate, same photo)
BTN_ROT_T = rot_x(BTN_TILT_DEG)  # the button's frame in T: its face normal is its local +y
BTN_AXIS_T = BTN_ROT_T @ np.array([0.0, -1.0, 0.0])  # the unit direction it is pressed in: into the head, 10 deg down
BTN_STROKE = figure("eject button stroke", 0.0055, "m", ESTIMATE, "a thumb button pressed into the head; Riester gives no travel")
# The button sits in a recess in the head's end 0.5 mm larger than it all round and 5 mm deep (the fingertips go in at most
# 5.5 - 1.5 = 4 mm); the button itself may sink deeper, as it does not collide with the thermometer it slides in.
RECESS_HALF_X = BTN_SIZE[0] / 2.0 + 0.0005
RECESS_Z = (BTN_POS_LOCAL[2] - BTN_SIZE[2] / 2.0 - 0.0005, BTN_POS_LOCAL[2] + BTN_SIZE[2] / 2.0 + 0.0005)
RECESS_Y0 = HEAD_Y[1] - 0.005
EJECT_TRAVEL = figure("eject release travel", 0.0040, "m", ESTIMATE, "the button travel at which the ejector pushes the cover off the ring")
BTN_SPRING = figure("eject button return spring", 900.0, "N/m", ESTIMATE, "3.6 N at the 4.0 mm release and 4.95 N at the 5.5 mm stop: a light thumb press")
BTN_DAMPING = 12.0  # N s/m on the 4 g button
BTN_RECOIL_MAX = 0.0010  # m: button within 1.0 mm of rest once released
BTN_PRESS_N = 9.0  # N told to the demo's left hand on the button and to its right hand against it; both arms give under load

# --- Riester disposable probe covers (Art. No. 12640) and their 2 x 5 rack ---------------------------------------------------
RACK_PACK_COUNT = figure("probe covers per box", 300.0, "pcs", MAKER, f"Riester Art. 12640 'Probe cover for ri-thermo tymPRO 30 rack / 300 pcs.' ({RIESTER_URL})")
COVERS_PER_RACK = figure("probe covers per rack", 10.0, "pcs", MAKER, "300 pcs / 30 racks on the same line; '02 Photo' shows them in 2 rows of 5")
COVER_FLANGE = figure("probe cover flange", 0.0176, "m", ESTIMATE, "two covers side by side fill the 43 mm rack in '02 Photo' at an 18.5 mm pitch")
COVER_BORE = figure("probe cover mouth", 0.0138, "m", ESTIMATE, "0.2 mm under the 14.0 mm ring, so the thin rim grips the ring")
COVER_BODY_W = COVER_BORE + 0.002  # m: 1.0 mm walls
COVER_H = figure("probe cover height", 0.017, "m", ESTIMATE, "the 13 mm tip, 1.5 mm to spare and a 2.5 mm closed end")
COVER_FLOOR_T = 0.0025  # m: the closed end
COVER_MASS = figure("probe cover mass (simulator)", 0.004, "kg", ESTIMATE, "4.0 g rigid body for contact stability at 240 Hz; the real thin cover is under a gram")
SEAT_FORCE = figure("press that seats a cover", 3.5, "N", ESTIMATE, "a light press pushes the 13.8 mm mouth onto the 14.0 mm ring")
SEAT_RADIAL_TOL = 0.0035  # m: the cone's tip within this of the cover's axis
SEAT_DEPTH_MIN = 0.0100  # m: the tip at least this far inside the cover
SEAT_TILT_DEG = 14.0  # deg: probe and cover axes within this of each other
GAP_AT_START = 0.0003  # m: the used cover's mouth this short of the ring at the start (no overlap under its weld)
USED_ON_THERM_LOCAL = tuple(PROBE_BASE_T + (RING_FACE + GAP_AT_START + COVER_H) * PROBE_DIR_T)  # cover origin (its floor) in T
COVER_ON_PROBE_ROT_T = rot_x(90.0 - PROBE_ANGLE_DEG)  # a seated cover's frame in T: its z along PROBE_UP_T

# --- Grey disposal tray (our fixture) ----------------------------------------------------------------------------------------
TRAY_L = figure("disposal tray length (x)", 0.200, "m", FIXTURE, "200 mm grey PP tray near the midline")
TRAY_W = figure("disposal tray width (y)", 0.150, "m", FIXTURE, "150 mm width")
TRAY_H = figure("disposal tray rim height", 0.020, "m", FIXTURE, "20 mm rim")
TRAY_WALL = 0.0025  # m
TRAY_FLOOR = 0.0035  # m (> 2 mm PhysX contact offset above the bench)
TRAY_MASS = figure("disposal tray mass", 0.180, "kg", ESTIMATE, "2.5 mm PP walls and 3.5 mm floor at 910 kg/m3")
TRAY_AT = (0.330, 0.015)  # m: x 23.0..43.0 cm, y -6.0..+9.0 cm

# --- Riester desktop cradle (dispenser box) with the Art. 12640 rack ----------------------------------------------------------
CRADLE_L = figure("cradle length", 0.170, "m", ESTIMATE, "a little longer than the 150 mm thermometer docked in it in '01 Photo'")
CRADLE_W = figure("cradle width", 0.056, "m", ESTIMATE, "about 1.35 times the 42 mm thermometer in '01 Photo'")
CRADLE_H = figure("cradle height", 0.058, "m", ESTIMATE, "the walls rise to about the width at the pocket end and stand lower (44 mm) along the rack in '02 Photo'")
CRADLE_LOW_H = 0.044  # m: side and end walls along the rack; the docked thermometer rests on the end wall here
CRADLE_WALL, CRADLE_FLOOR = 0.003, 0.004  # m
CRADLE_MASS = figure("cradle mass", 0.140, "kg", ESTIMATE, "a 3 mm ABS shell of about 100 cm3 at 1.05 g/cm3 with the empty rack and rubber feet")
CRADLE_FRICTION = figure("cradle rubber-foot friction", 0.85, "-", ESTIMATE, "elastomer feet on a dry laminate worktop")
CRADLE_AT = (0.150, 0.000)  # m: x 12.2..17.8 cm, y -8.5..+8.5 cm, long axis along y, pocket end at +y
POCKET_FROM = 0.016  # m in C: the rack ends and the probe's pocket begins here
RIB_Y = (0.016, 0.024)  # m in C: the cross rib under the docked head
RACK_Y = (-0.082, 0.014)  # m in C: the 2 x 5 rack, 96 mm long
RACK_HALF_X = 0.0215  # m: 43 mm wide
RACK_BASE_Z = (0.004, 0.006)  # m in C: its base plate; covers stand on it
WELL_PITCH = 0.0185  # m
WELL_XS = (-0.00925, 0.00925)  # m in C: the two rows (row 0 nearer the robot)
WELL_YS = tuple(RACK_Y[0] + 0.0015 + WELL_PITCH / 2.0 + WELL_PITCH * k for k in range(5))  # five columns
WELLS = tuple((x, y) for y in WELL_YS for x in WELL_XS)  # 10 wells, column by column
FRESH = tuple(f"cover_fresh_{i}" for i in range(len(WELLS)))
LOAD_WELL = 2  # column 1, row 0: the demo presses this cover on (column 0's neighbour wall is too close to the tilted body)
DOCK_X_OFFSET, DOCK_Y_OFFSET = 0.0, -0.020  # m in C: the docked thermometer's origin
DOCK_SADDLE_Z = CRADLE_LOW_H  # m in C: its back face rests on the end wall and the rib at this height
DOCK_XY_TOL = 0.012  # m
DOCK_Z_TOL = 0.006  # m
DOCK_TILT_DEG = 10.0  # deg

# --- Start rest (our fixture) -------------------------------------------------------------------------------------------------
PAD_SADDLE_Z = figure("start rest saddle height", 0.038, "m", FIXTURE, "38 mm stepped rest holding the thermometer level, its covered probe 8 mm clear of the bench")
THERM_START = (0.330, -0.200, PAD_SADDLE_Z)  # m: the thermometer's origin, level, probe towards +y
PAD_AT = (THERM_START[0], THERM_START[1])

# --- Hand poses (shared by scene.py, checks.py, demo.py) ------------------------------------------------------------------------
GRASP_T = np.array([0.0, -0.056, 0.020])  # m in T: the right hand's grasp across the tail's upper half
R_GRASP_TILT = 60.0  # deg: the right hand comes down onto the tail from behind (-y), its approach this far below level
R_LEVEL_ROT = rot_frame((1.0, 0.0, 0.0), (0.0, math.cos(math.radians(R_GRASP_TILT)), -math.sin(math.radians(R_GRASP_TILT))))
R_LOAD_PITCH = 90.0 - PROBE_ANGLE_DEG  # deg nose-down that stands the probe upright
R_LOAD_ROT = rot_x(-R_LOAD_PITCH) @ R_LEVEL_ROT
L_PRESS_ROT = rot_frame((0.0, -math.sin(math.radians(BTN_TILT_DEG)), math.cos(math.radians(BTN_TILT_DEG))), BTN_AXIS_T)  # fingertips flat on the button
L_GRIP_ROT = rot_frame((-1.0, 0.0, 0.0), (0.0, -math.sin(math.radians(45.0)), -math.cos(math.radians(45.0))))  # onto the walls from +y


def therm_rot(hand_rot) -> np.ndarray:
    """The thermometer's rotation when the right hand, holding it at GRASP_T, is turned ``hand_rot``."""
    return np.asarray(hand_rot) @ R_LEVEL_ROT.T


def tcp_for(point_t, at_world, hand_rot) -> np.ndarray:
    """The right hand's grasp point that puts thermometer point ``point_t`` at ``at_world`` with the hand turned ``hand_rot``."""
    return np.asarray(at_world, dtype=float) + therm_rot(hand_rot) @ (GRASP_T - np.asarray(point_t, dtype=float))


def world_of(point_t, tcp, hand_rot) -> np.ndarray:
    return np.asarray(tcp, dtype=float) + therm_rot(hand_rot) @ (np.asarray(point_t, dtype=float) - GRASP_T)


R_PICK_TCP = tuple(np.array(THERM_START) + GRASP_T)  # bench-relative z
R_EJECT_TCP = (0.330, -0.090, 0.0876)  # level, the covered probe over the tray, the button on +y for the left hand
_well = WELLS[LOAD_WELL]
LOAD_TIP = np.array([CRADLE_AT[0] + _well[0], CRADLE_AT[1] + _well[1], RACK_BASE_Z[1] + COVER_FLOOR_T + 0.0015])  # seated tip
R_LOAD_TCP = tuple(tcp_for(CONE_TIP_T, LOAD_TIP, R_LOAD_ROT))
DOCK_ORIGIN = np.array([CRADLE_AT[0] + DOCK_X_OFFSET, CRADLE_AT[1] + DOCK_Y_OFFSET, DOCK_SADDLE_Z])
R_DOCK_TCP = tuple(DOCK_ORIGIN + GRASP_T)
L_GRIP_TCP = (CRADLE_AT[0], CRADLE_AT[1] + 0.074, 0.040)  # across the 56 mm side walls near the pocket end

# Scoring constants:
HANDS_CLEAR = 0.030  # m: both hands >= 30 mm from all task bodies at every positive score level
HOLD_STEPS = 20  # 1.0 s at 20 Hz
HOLD_DRIFT = 0.003  # m: max drift during the 1.0 s rest

BODIES = ("thermometer", "eject_btn", "cradle", "disposal_tray", "cover_used", *FRESH)


def eject_button_world(top: float) -> tuple[np.ndarray, np.ndarray]:
    """(world middle of the eject button's face, unit press direction) with the right hand at R_EJECT_TCP."""
    tcp = np.array([R_EJECT_TCP[0], R_EJECT_TCP[1], top + R_EJECT_TCP[2]], dtype=float)
    face_t = np.array(BTN_POS_LOCAL) - (BTN_SIZE[1] / 2.0) * BTN_AXIS_T
    return world_of(face_t, tcp, R_LEVEL_ROT), therm_rot(R_LEVEL_ROT) @ BTN_AXIS_T


def layout_ok() -> bool:
    """Self-check: fixtures apart on the bench and every arm target on its own side of y = 0."""
    tray_x0 = TRAY_AT[0] - TRAY_L / 2
    cradle_x1 = CRADLE_AT[0] + CRADLE_W / 2
    if cradle_x1 + 0.020 > tray_x0:
        return False
    if any(y >= -0.020 for y in (R_PICK_TCP[1], R_EJECT_TCP[1], R_LOAD_TCP[1], R_DOCK_TCP[1])):
        return False
    face, _ = eject_button_world(0.0)
    return bool(L_GRIP_TCP[1] > 0.020 and face[1] > 0.005)


def summary() -> dict:
    face_w, press_w = eject_button_world(0.0)
    return {
        "figures_count": len(FIGURES),
        "by_source": {k: sum(1 for v in FIGURES.values() if v[2] == k) for k in (MAKER, FIXTURE, ESTIMATE)},
        "layout_ok": layout_ok(),
        "cone tip in T (mm)": [round(1000 * float(v), 1) for v in CONE_TIP_T],
        "used cover origin in T (mm)": [round(1000 * float(v), 1) for v in USED_ON_THERM_LOCAL],
        "eject_btn_face_world": [round(float(v), 4) for v in face_w],
        "right_tcps_y_m": [round(float(v[1]), 4) for v in (R_PICK_TCP, R_EJECT_TCP, R_LOAD_TCP, R_DOCK_TCP)],
        "load tcp": [round(float(v), 4) for v in R_LOAD_TCP],
        "left grip tcp": [round(float(v), 4) for v in L_GRIP_TCP],
    }
