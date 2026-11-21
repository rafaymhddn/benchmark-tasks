"""Figures, shapes and joints of the adjustable wrench task; ``task.py`` holds the rules and ``score()``.

Every figure carries its source: the maker's catalogue, a standard, or an estimate with how it was derived.

- Bahco 8071 adjustable wrench: Bahco catalogue ADJ-1602-ENG, page 11 ("The Bahco 80 series", dimensions A to D and
  the weight), https://docs.rs-online.com/ab1a/A700000014532817.pdf; the 80 series' feature text in the same catalogue
  (jaws angled at 15 degrees, a scale on the jaw).
- DIN 985 M10 nylon-insert nut (17 mm across flats): Böllhoff data sheet "DIN 985 similar to ISO 10511", steel 8,
  zinc plated, https://eshop-hu.boellhoff.com/out/media/pdf/DIN_985_Stahl_08_gv_Feingewinde___en.pdf (M10 rows).
- Prevailing torque: ISO 2320:2015, Tables 1, 3 and 4 (M10: first removal at least 1.5 N m).

The wrench frame W: x along the jaws' opening (from the throat out to the jaw tips), y across the jaws (from the fixed
jaw's face towards the movable jaw), z out of the wrench's upper face. Its origin is on the fixed jaw's face, at the
depth where a seated nut's middle sits (half the nut's corner width from the throat), at mid-thickness.
"""

from __future__ import annotations

import math

import numpy as np

import mechkit as mk

MECHKIT_HELPER = True  # declares for the task file that imports it (mechkit's registry keeps one owner)
MM = 1e-3

# --- Bahco 8071 (maker's catalogue ADJ-1602-ENG p. 11) ------------------------------------------------------------
WRENCH_LENGTH = 205 * MM  # maker's catalogue: A, overall length
JAW_MAX = 27.0 * MM  # maker's catalogue: B, largest jaw opening
HEAD_T = 12.9 * MM  # maker's catalogue: D, head thickness (side view, at the knurl)
JAW_TIP_T = 6.0 * MM  # maker's catalogue: C, jaw thickness at the tips
WRENCH_MASS = 0.255  # kg, maker's catalogue: weight 255 g
HEAD_ANGLE = math.radians(15.0)  # maker's catalogue: "Jaws angled at 15°"
# Estimates scaled from the catalogue's dimension drawing (p. 11, A = 205 mm as the scale, 0.097 mm a pixel at 1600 dpi):
JAW_DEPTH = 23.0 * MM  # estimate: throat to jaw tips on the drawing
JAW_T = 9.0 * MM  # estimate: the jaws taper from D (12.9 mm) to C (6.0 mm); 9 mm where they bear on a 17 mm nut
JAW_W = 8.0 * MM  # estimate: each jaw's width across the opening, drawing
HANDLE_W = 21.0 * MM  # estimate: handle width, drawing (18 to 24 mm along its length)
HANDLE_T = 8.0 * MM  # estimate: the drawing's side view shows the handle thinner than the 12.9 mm head
NECK_W, NECK_T = 17.0 * MM, 10.0 * MM  # estimate: drawing
KNURL_D_DRAWN = 12.0 * MM  # estimate: the knurl's width on the drawing (its diameter seen from above)
KNURL_L = 11.0 * MM  # estimate: the knurl's length on the drawing
KNURL_PITCH = 2.9 * MM  # estimate: jaw travel per knurl turn, the drawing's 4 thread crests over 10.7 mm
KNURL_IN_W = np.array([-25.8, 13.4]) * MM  # estimate: knurl middle in W, from the drawing (26 mm behind a seated nut's middle)
HANDLE_LINE_IN_W = np.array([-1.27, 3.77]) * MM  # estimate: a point of the handle's centre line in W, drawing
HANDLE_DIR_IN_W = np.array([math.cos(math.pi - HEAD_ANGLE), math.sin(math.pi - HEAD_ANGLE)])  # handle 15 deg off the jaws' axis
HANDLE_END_S = 190.0 * MM  # estimate: from a seated nut's middle to the handle's end along the handle (205 mm less 15 mm)
MOVABLE_JAW_MASS = 0.080  # kg: the jaw blade and rack (about 25 g) carried heavier for the solver (stand-in; total stays 255 g)
KNURL_MASS = 0.005  # kg, estimate: a 12 x 11 mm steel knurl less its bore
# Simulator stand-ins (declared in the README):
KNURL_D = HEAD_T + 2 * 0.5 * MM  # the simulated knurl stands 0.5 mm proud of both faces so a rigid fingertip reaches it
KNURL_INERTIA = (2e-6, 2e-6, 2e-6)  # kg m^2, about 15 times the real knurl's, for the solver
KNURL_DAMPING = 0.002  # N m s/rad: about 0.8 N at the knurl's rim for a 20 mm/s roll (estimate of a worm's drag)
JAW_SPRING = 20000.0  # N/m: the jaw follows the knurl's turns through a stiff spring ...
JAW_DAMPING = 30.0  # N s/m
SNUG_FORCE = 4.0  # N, estimate: ... capped at what a fingertip's 1 N on the 7 mm knurl gives through a 2.9 mm worm at 30 %
JAW_PLAY = 0.2 * MM  # estimate: worm and rack play; the jaw cannot be pushed open further than this (no back-drive)
JAW_LOAD_N = 20.0  # N: above this the nut loads the jaw and the worm locks (a loaded worm cannot be rolled)
JAW_LOCK_GAP = 0.02 * MM  # the lock's spring rests this far open of the lock: it holds the jaw, it does not clamp the nut
JAW_LOCK_K, JAW_LOCK_D, JAW_LOCK_MOST = 1.0e6, 400.0, 500.0  # N/m, N s/m, N: the locked worm holds the jaw (solver stand-in:
# the kit's joint stop alone gave 0.2 mm under 30 N, run gw4; a flat's end carries about 180 N at 1.5 N m)
JAW_START = 23.0 * MM  # the start: jaw open 23 mm (wider than the nut's 18.9 mm corners)

# --- DIN 985 M10 nut (Böllhoff data sheet) and ISO 2320 -----------------------------------------------------------
NUT_AF = 17.0 * MM  # data sheet: s max 17.00 (s min 16.73)
NUT_E = 18.9 * MM  # data sheet: e min 18.90 (the corners are chamfered; sharp corners would be 19.63)
NUT_H = 10.0 * MM  # data sheet: h max 10.0 (h min 9.64), the whole height with the insert's collar
NUT_HEX_H = 8.0 * MM  # estimate: the hex part under the insert's collar (the data sheet's m min 6.5, m1 min 5.5)
NUT_COLLAR_D = 15.6 * MM  # data sheet: dw min 15.6 (the collar is narrower than the flats)
NUT_MASS = 0.25  # kg: the nut (about 11 g) carried heavier for the solver, held by its hinge (stand-in; run gw7: a 11 g nut let the
# jaws sink 0.5-0.8 mm into its corners under 40-90 N)
NUT_INERTIA = (2e-4, 2e-4, 2e-4)  # kg m^2, stand-in for the solver (the real nut's is about 7e-7)
PREVAILING_TORQUE = 1.5  # N m, standard: ISO 2320:2015 Tables 1/3/4, M10 first removal at least 1.5 N m
NUT_GIVE = math.radians(2.0)  # the capped spring that holds the nut yields 2 deg before it slips (solver stand-in)
NUT_SPRING = PREVAILING_TORQUE / NUT_GIVE  # N m/rad
NUT_DAMPING = 0.01  # N m s/rad
NUT_LIMITS = (math.radians(-20.0), math.radians(150.0))
# Friction at the jaws: the kit's default for graspable parts (1.2 static, 1.0 dynamic), a stand-in. Dry steel is 0.15-0.3; with
# 0.2 the jaws needed about 200 N on the flats' ends and the solver let them roll round the nut (run gw9: 23.6 deg of nut for
# 82.6 deg of wrench); with the default part of the torque goes by friction and the nut turns with the wrench (gw8).

# --- Plate, screw, washer -----------------------------------------------------------------------------------------
PLATE_T = 8.0 * MM  # the steel plate the screw stands in
PLATE_XY = (0.250, -0.050)
PLATE_SIZE = (0.330, 0.340)
SCREW_D = 10.0 * MM  # ISO 4017 M10 x 25 screw, screwed up through the plate: 17 mm stands above it
SCREW_ABOVE = 17.0 * MM
WASHER_D, WASHER_T = 30.0 * MM, 2.5 * MM  # DIN 9021 M10 washer: 30 mm across, 2.5 mm thick
NUT_XY = (0.280, 0.060)
NUT_GAP = 0.3 * MM  # the nut hangs 0.3 mm over the washer: its hinge carries it, so no washer friction adds to its torque
HANDLE_START_DEG = 210.0  # deg counter-clockwise from +x seen from above: back and to the right, towards the right arm's base
TILT_GAP = 0.3 * MM  # the wrench is built this far over its supports and settles onto them

# --- Rules (task.py) ----------------------------------------------------------------------------------------------
LOOSEN_DEG = 60.0  # one flat of the hex past its witness mark
TURN_LATCH_DEG = 10.0  # the jaw's setting is read as the nut first passes this (the turn has begun)
SNUG_MM = 17.5  # jaw at most 17.5 mm: CCOHS, Hand Tools - Wrenches: "Do not pull on an adjustable wrench that is loosely
# adjusted." (https://www.ccohs.ca/oshanswers/safety_haz/hand_tools/wrenches.html); 17.5 = the 17.0 mm flats + the half
# millimetre the jaw's scale is read to
CLEAR_M = 0.035  # the jaws' middle at least 35 mm from the screw's axis
FLAT_DEG = 15.0
HANDS_CLEAR_M = 0.030
TIGHTEN_LIMIT_DEG = -10.0
REST_MOVE_M, REST_TURN_DEG, REST_HOLD_S = 0.001, 0.5, 1.0  # still: within 1 mm and 0.5 deg for 1.0 s
ON_PLATE_M = 0.020  # the jaws' middle and the handle's end at most 20 mm over the plate: lying on it
SEAT_GAP = 0.5 * MM  # each jaw face within 0.5 mm of the flat facing it: the real trial's 0.5 mm feeler gauge (17.5 less 17.0 mm)
SEAT_COVER = 3.0 * MM  # estimate: the jaws on at least 3 mm of the hex's 8 mm flats (5.9 mm with the head on the washer), so a
# wrench raised 3 mm off the nut no longer counts as on it

STEEL, DARK, BLACK, ZINC = (0.20, 0.21, 0.23), (0.30, 0.31, 0.33), (0.07, 0.07, 0.08), (0.86, 0.86, 0.80)  # STEEL: the 8071's black phosphate
PLATE_COLOR, MARK, BLUE = (0.30, 0.42, 0.55), (0.98, 0.86, 0.10), (0.12, 0.35, 0.85)  # a painted plate, for contrast


def rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def jaw_axis_world(rw: np.ndarray) -> np.ndarray:
    return rw[:, 1]


def handle_point_w(s: float) -> np.ndarray:
    """A point of the handle's centre line in W, ``s`` m from a seated nut's middle along the handle."""
    return np.array([*(HANDLE_LINE_IN_W + s * HANDLE_DIR_IN_W), 0.0])


def start_pose(top: float) -> tuple[np.ndarray, np.ndarray]:
    """Where W stands at the start: the fixed jaw against one flat (the nut's middle 8.5 mm from it) and the nut against
    the throat, the handle pointing at HANDLE_START_DEG, the head on the washer and the handle's end on the plate."""
    plate_top = top + PLATE_T
    washer_top = plate_top + WASHER_T
    yaw = math.radians(HANDLE_START_DEG) - math.atan2(HANDLE_DIR_IN_W[1], HANDLE_DIR_IN_W[0])
    r_level = rz(yaw)
    nut = np.array([*NUT_XY, 0.0])
    o_level = nut - r_level @ np.array([0.0, NUT_AF / 2, 0.0])
    o_level[2] = washer_top + HEAD_T / 2 + TILT_GAP
    # tilt about the washer's edge under the head until the handle's end rests on the plate
    h = r_level @ np.array([*HANDLE_DIR_IN_W, 0.0])
    pivot = nut + (WASHER_D / 2) * h
    end = o_level + r_level @ handle_point_w(HANDLE_END_S)
    drop = (end[2] - HANDLE_T / 2) - (plate_top + TILT_GAP)
    run = float(np.linalg.norm((end - pivot)[:2]))
    tilt = math.atan2(drop, run)
    axis = np.cross([0.0, 0.0, 1.0], h)  # turning about it by +tilt lowers the handle's end
    r_tilt = mk.axis_angle(axis, tilt)
    pivot[2] = o_level[2] - HEAD_T / 2
    o = pivot + r_tilt @ (o_level - pivot)
    return o, r_tilt @ r_level


def nut_shapes() -> list:
    """The hex as three boxes across the flats and three thin boxes across the chamfered corners (their union is the
    12-sided outline exactly), the collar on top and a witness mark on one flat's top edge."""
    side = 2 * ((NUT_E / 2 - (NUT_AF / 2) * math.cos(math.pi / 6)) / math.sin(math.pi / 6))
    corner = 2 * ((NUT_AF / 2 - (NUT_E / 2) * math.cos(math.pi / 6)) / math.sin(math.pi / 6))
    z = -NUT_H / 2 + NUT_HEX_H / 2
    out = [mk.box((NUT_AF, side, NUT_HEX_H), pos=(0, 0, z), yaw=k * math.pi / 3) for k in range(3)]
    out += [mk.box((NUT_E, corner, NUT_HEX_H), pos=(0, 0, z), yaw=k * math.pi / 3 + math.pi / 6) for k in range(3)]
    out.append(mk.cylinder(NUT_COLLAR_D / 2, NUT_H - NUT_HEX_H, pos=(0, 0, NUT_H / 2 - (NUT_H - NUT_HEX_H) / 2), color=BLUE))
    out.append(mk.box((2.0 * MM, 7.0 * MM, 0.3 * MM), pos=(0, 3.9 * MM, NUT_H / 2 + 0.15 * MM), color=MARK, solid=False))  # witness mark (+y)
    return out


def head_shapes() -> list:
    """Fixed jaw, throat block, the frame round the knurl's window and the handle, in W."""
    tx = -NUT_E / 2  # the throat's face
    kx, ky = KNURL_IN_W
    wx0, wx1, wy0, wy1 = kx - 8 * MM, kx + 8 * MM, ky - 7 * MM, ky + 7 * MM  # the knurl's window
    y_lo, y_hi = -JAW_W, JAW_MAX + JAW_W
    back = 7.5 * MM

    def b(x0, x1, y0, y1, t, color=STEEL, solid=True):
        return mk.box((x1 - x0, y1 - y0, t), pos=((x0 + x1) / 2, (y0 + y1) / 2, 0.0), color=color, solid=solid)

    out = [
        b(tx, tx + JAW_DEPTH, y_lo, 0.0, JAW_T),  # fixed jaw
        b(tx - back, tx, y_lo, y_hi, HEAD_T),  # throat
        b(wx0, tx - back, y_lo, wy0, HEAD_T),  # frame below the window
        b(wx0, tx - back, wy1, y_hi, HEAD_T),  # frame above the window
        b(wx0 - 8 * MM, wx0, y_lo, y_hi, HEAD_T),  # frame behind the window
    ]
    yaw = math.atan2(HANDLE_DIR_IN_W[1], HANDLE_DIR_IN_W[0])
    for s0, s1, w, t in ((40 * MM, 70 * MM, NECK_W, NECK_T), (70 * MM, HANDLE_END_S - 11 * MM, HANDLE_W, HANDLE_T)):
        out.append(mk.box((s1 - s0, w, t), pos=tuple(handle_point_w((s0 + s1) / 2)), yaw=yaw))
    out.append(mk.cylinder(11 * MM, HANDLE_T, pos=tuple(handle_point_w(HANDLE_END_S - 11 * MM))))
    out.append(mk.cylinder(4.5 * MM, HANDLE_T + 0.4 * MM, pos=tuple(handle_point_w(HANDLE_END_S - 11 * MM)), color=PLATE_COLOR, solid=False))
    for k in range(7):  # the jaw scale's marks, every 5 mm, on the frame below the throat (look only)
        out.append(mk.box((2.5 * MM, 0.5 * MM, 0.2 * MM), pos=(tx - 2 * MM, k * 4.5 * MM, HEAD_T / 2 + 0.1 * MM), color=BLACK, solid=False))
    return out


def movable_jaw_shapes() -> list:
    """In its own frame (W's axes), origin on its face at the jaws' mid-depth."""
    return [
        mk.box((JAW_DEPTH, JAW_W, JAW_T), pos=(0.0, JAW_W / 2, 0.0), color=BLACK),
        mk.box((14 * MM, 5 * MM, 6 * MM), pos=(-JAW_DEPTH / 2 - 7 * MM, 3 * MM, 0.0), color=BLACK, solid=False),
    ]


def knurl_shapes() -> list:
    """Along the knurl's axis (W's y), with three dark stripes along it so that its turning shows."""
    out = [mk.cylinder(KNURL_D / 2, KNURL_L, axis="y", color=ZINC, passes=("plate", "nut", "movable_jaw"))]  # only a finger turns it
    for k in range(3):
        a = k * 2 * math.pi / 3
        out.append(mk.box((1.4 * MM, KNURL_L + 0.2 * MM, 0.6 * MM), pos=(math.sin(a) * KNURL_D / 2, 0.0, math.cos(a) * KNURL_D / 2),
                          quat=mk.matrix_to_quat(mk.axis_angle((0, 1, 0), a)), color=BLACK, solid=False))  # fmt: skip
    return out


def nut_yaw(rw: np.ndarray) -> float:
    """The nut's built yaw: two of its flats parallel to the jaws' faces at the start."""
    across = math.atan2(rw[1, 1], rw[0, 1])  # the jaws' slide axis in the world
    return across % (math.pi / 3)


def declare_scene(top: float) -> dict:
    """Declare the bodies and joints; return the start poses the rules and checks use."""
    plate_top = top + PLATE_T
    washer_top = plate_top + WASHER_T
    nx, ny = NUT_XY
    o, rw = start_pose(top)
    ny_yaw = nut_yaw(rw)
    mark = ny_yaw + math.pi / 2  # the witness mark's direction in the world at the start
    mk.body("plate", [
        mk.box((*PLATE_SIZE, PLATE_T), color=PLATE_COLOR),
        mk.cylinder(WASHER_D / 2, WASHER_T, pos=(nx - PLATE_XY[0], ny - PLATE_XY[1], PLATE_T / 2 + WASHER_T / 2), color=ZINC),
        mk.cylinder(SCREW_D / 2, SCREW_ABOVE - WASHER_T, pos=(nx - PLATE_XY[0], ny - PLATE_XY[1], PLATE_T / 2 + WASHER_T + (SCREW_ABOVE - WASHER_T) / 2), color=ZINC, solid=False),
        mk.box((5.5 * MM, 2.0 * MM, 0.3 * MM), pos=(nx - PLATE_XY[0] + 12.25 * MM * math.cos(mark), ny - PLATE_XY[1] + 12.25 * MM * math.sin(mark), PLATE_T / 2 + WASHER_T + 0.15 * MM), yaw=mark, color=MARK, solid=False),
    ], pos=(*PLATE_XY, top + PLATE_T / 2), mass=None, color=PLATE_COLOR)  # fmt: skip
    nut_z = washer_top + NUT_GAP + NUT_H / 2
    mk.body("nut", nut_shapes(), pos=(nx, ny, nut_z), yaw=ny_yaw, mass=NUT_MASS, color=ZINC, inertia=NUT_INERTIA, center=(0.0, 0.0, -0.5 * MM), contacts=True)
    q = mk.matrix_to_quat(rw)
    mk.body("wrench", head_shapes(), pos=tuple(o), quat=q, mass=WRENCH_MASS - MOVABLE_JAW_MASS - KNURL_MASS, color=STEEL)
    jaw_origin = o + rw @ np.array([-NUT_E / 2 + JAW_DEPTH / 2, JAW_START, 0.0])
    mk.body("movable_jaw", movable_jaw_shapes(), pos=tuple(jaw_origin), quat=q, mass=MOVABLE_JAW_MASS, color=BLACK)
    knurl_origin = o + rw @ np.array([*KNURL_IN_W, 0.0])
    mk.body("knurl", knurl_shapes(), pos=tuple(knurl_origin), quat=q, mass=KNURL_MASS, color=ZINC, inertia=KNURL_INERTIA, friction=(1.0, 1.0), contacts=True)
    axis = jaw_axis_world(rw)
    mk.hinge("nut_thread", "nut", None, anchor=(nx, ny, nut_z), axis=(0.0, 0.0, 1.0), limits=NUT_LIMITS, spring=NUT_SPRING, damping=NUT_DAMPING)
    mk.slide("jaw", "movable_jaw", "wrench", axis=tuple(axis), limits=(-JAW_START + 1 * MM, JAW_MAX - JAW_START), spring=JAW_SPRING, damping=JAW_DAMPING)
    mk.hinge("knurl", "knurl", "wrench", anchor=tuple(knurl_origin), axis=tuple(axis), limits=None, damping=KNURL_DAMPING)
    mk.sense_contact("movable_jaw", "nut")
    mk.sense_contact("wrench", "nut")
    return {"wrench": (o, rw), "plate_top": plate_top, "washer_top": washer_top, "nut_z": nut_z}


def wrench_segments(o: np.ndarray, rw: np.ndarray, jaw_m: float) -> list[tuple[np.ndarray, np.ndarray, float]]:
    """The wrench as capsules (end, end, radius) in the world, for the hands' clearance: head and handle."""
    tip = o + rw @ np.array([-NUT_E / 2 + JAW_DEPTH, jaw_m / 2, 0.0])
    back = o + rw @ np.array([KNURL_IN_W[0] - 16 * MM, 13.5 * MM, 0.0])
    neck = o + rw @ handle_point_w(40 * MM)
    end = o + rw @ handle_point_w(HANDLE_END_S - 11 * MM)
    return [(tip, back, 22 * MM), (neck, end, 11 * MM)]


def to_capsule(p, a, b, r: float) -> float:
    """Distance from point ``p`` to the surface of the capsule a-b of radius ``r`` (negative inside)."""
    p, a, b = (np.asarray(v, dtype=float) for v in (p, a, b))
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / max(float(np.dot(ab, ab)), 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab))) - r


def nut_outline() -> np.ndarray:
    """The 12 corners of the nut's outline in its frame (m), as ``nut_shapes()`` builds it: the ends of its six flats."""
    half = (NUT_E / 2 - (NUT_AF / 2) * math.cos(math.pi / 6)) / math.sin(math.pi / 6)
    return np.array([rz(k * math.pi / 3) @ np.array([NUT_AF / 2, s * half, 0.0]) for k in range(6) for s in (1.0, -1.0)])


NUT_OUTLINE = nut_outline()


def seat(o: np.ndarray, rw: np.ndarray, jaw_m: float, nut_yaw_now: float, nut_middle: np.ndarray) -> tuple[float, float, float, float]:
    """How the nut sits in the jaws (m): its axis along the jaws in W (x), the gap from the fixed jaw's face to the nut's
    outline and from the outline to the movable jaw's face, across the jaws (negative: overlapping), and how much of the
    flats' height the jaws cover."""
    x, _, z = rw.T @ (nut_middle - o)
    across = [float(rw[:, 1] @ (nut_middle + rz(nut_yaw_now) @ c - o)) for c in NUT_OUTLINE]
    bottom = z - NUT_H / 2
    cover = min(JAW_T / 2, bottom + NUT_HEX_H) - max(-JAW_T / 2, bottom)
    return float(x), min(across), jaw_m - max(across), float(cover)


def seated(x: float, gap_fixed: float, gap_movable: float, cover: float) -> bool:
    """The nut seated between the jaws: its axis between the throat and the jaw tips, each jaw face within SEAT_GAP of the
    flat facing it (so the axis is between the faces), and the jaws on at least SEAT_COVER of the flats' height."""
    inside = -NUT_E / 2 <= x <= -NUT_E / 2 + JAW_DEPTH
    return bool(inside and max(abs(gap_fixed), abs(gap_movable)) <= SEAT_GAP + 1e-9 and cover >= SEAT_COVER - 1e-9)


def jaw_middle(o: np.ndarray, rw: np.ndarray, jaw_m: float) -> np.ndarray:
    """The middle of the jaws' opening at a seated nut's depth, in the world."""
    return o + rw @ np.array([0.0, jaw_m / 2, 0.0])
