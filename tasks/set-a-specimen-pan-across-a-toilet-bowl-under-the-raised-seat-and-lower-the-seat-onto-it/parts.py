"""Every figure the specimen-pan task rests on, in one place with its true source, and the geometry built from them.

Source kinds in ``FIGURES`` (the README's key-figure table is this registry):
- ``maker``: Bemis's own product page and spec sheet of the 1955CT seat; Kohler's roughing-in sheet 116148 of the
  K-4302 Highcrest elongated bowl (URLs below, the makers' words quoted).
- ``our fixture``: the stand that replaces the bowl's pedestal and the floor, the bench layout and the scoring limits.
- ``estimate``: a derived figure, with the one-clause derivation; the first minutes of a real trial check each one with
  a tape, a caliper or a kitchen scale, and a different reading replaces the estimate.
The pan's outer sizes are the Office Depot listing's, kept as estimates; the first minutes check them with a caliper.

Station frame: metres from the midpoint between the arm bases (left y = +0.305, right y = -0.305), x forward, y left,
z up from the bench top. The toilet faces the arms: the bowl's front at small x; the seat's hinge (its bolt line along
y), the bowl's back and the wall behind it at larger x. ``d`` is the distance from the bolt line towards the bowl's
front (towards -x), ``a`` the distance across from the bowl's middle line (y - BOWL_Y).
"""

from __future__ import annotations

import math

import numpy as np

GRAVITY = 9.81
INCH = 0.0254

MAKER, LISTING, FIXTURE, ESTIMATE = "maker", "listing", "our fixture", "estimate"

ONET_URL = "https://www.onetonline.org/link/summary/31-1131.00"
ONET_TASKS = ("Collect specimens, such as urine, feces, or sputum.",)
ONET_EMPLOYMENT = 1_505_900  # O*NET 31-1131.00, 'Employment (2025) 1,505,900 employees'
BEMIS_PAGE = "https://www.toiletseats.com/p/bemis-elongated-open-front-less-cover-plastic-toilet-seat-in-white-that-stays-tight-2"
BEMIS_SHEET = "https://www.toiletseats.com/assets/0B7012484_1955CT_SSCT-Bemis-Spec.pdf"
PAN_LISTING = "https://www.odpbusiness.com/a/products/458238/Medline-Specimen-Collector-Pans-30-Oz/"
KOHLER_SHEET = "https://resources.kohler.com/onlinecatalog/pdf/116148_1.pdf"

FIGURES: dict[str, tuple[float, str, str, str]] = {}


def figure(name: str, value: float, unit: str, source: str, note: str) -> float:
    if source not in (MAKER, LISTING, FIXTURE, ESTIMATE):
        raise ValueError(f"{name}: unknown source kind {source!r}")
    FIGURES[name] = (float(value), unit, source, note)
    return float(value)


# --- The toilet seat: Bemis 1955CT, commercial open front less cover, elongated (maker's page and spec sheet) ---------
SEAT_L = figure("seat length, hinge bolts to front", 18.5625 * INCH, "m", MAKER, f"spec sheet 0B7012484 '18-9/16\"', page 'Depth: 18.563 in' ({BEMIS_SHEET})")
SEAT_W = figure("seat width", 14.5 * INCH, "m", MAKER, f"spec sheet '14-1/2\"', page 'Width: 14.5 in' ({BEMIS_PAGE})")
SEAT_IN_W = figure("seat opening width", (7 + 15 / 16) * INCH, "m", MAKER, "spec sheet '7-15/16\"'")
SEAT_GAP = figure("open-front gap between the horns", 3.5 * INCH, "m", MAKER, "spec sheet '3-1/2\"'")
SEAT_T = figure("ring thickness", (13 / 16) * INCH, "m", MAKER, "spec sheet '13/16\"' (the ring at its front)")
SEAT_T_BUMPED = figure("ring thickness with a bumper", (1 + 1 / 8) * INCH, "m", MAKER, "spec sheet '1-1/8\"'; 'Ring Bumpers: Four'")
BOLT_SPREAD = figure("hinge bolt spread", 5.5 * INCH, "m", MAKER, "page 'Bolt Spread: 5.5 in'; Kohler 'Seat post hole centers 5-1/2″ (140 mm)'")
HINGE_END_H = figure("height of the hinge end", (2 + 3 / 8) * INCH, "m", MAKER, "spec sheet '2-3/8\"', page 'Height: 2.375 in'")
SEAT_MASS = figure("seat mass", 4.216 * 0.45359237, "kg", MAKER, "page 'Weight: 4.216 lbs' (all of it given to the seat)")
CHECK_HINGE_DEG = figure("seat raised to its check-hinge stop", 101.0, "deg", MAKER, "page 'Check hinges prevent the seat from going 11° beyond vertical'")
BUMPER_H = SEAT_T_BUMPED - SEAT_T  # 7.9 mm

PINTLE_H = figure("hinge pin over the rim", 0.035, "m", ESTIMATE, "about 25 mm under the top of the 2-3/8 in. (60.3 mm) hinge end")
SEAT_REAR_W = figure("seat width across the hinge end", 0.240, "m", ESTIMATE, "scaled from the maker's drawing with its 14-1/2 in. width")
SEAT_OPEN_D0 = figure("seat opening's back edge from the hinge bolts", 0.060, "m", ESTIMATE, "scaled from the maker's drawing with its 18-9/16 in. length")
SEAT_HORN_D0 = figure("start of the front horns from the hinge bolts", 0.400, "m", ESTIMATE, "scaled from the maker's drawing")
SEAT_HORN_OUT = figure("half-width of the seat at its front horns", 0.160, "m", ESTIMATE, "scaled from the maker's drawing (the rounded front)")
BUMPER_D = (0.140, 0.340)  # m from the hinge bolts: the four bumpers' rows, scaled from the maker's side view (estimate)
BUMPER_X = 0.142  # m from the seat's middle line: under the middle of each side arm (estimate)
BUMPER_SIZE = (0.020, 0.030)  # m across, along (estimate)

# --- The bowl: Kohler K-4302 Highcrest, elongated, top spud (roughing-in sheet 116148) ---------------------------------
BOWL_W = figure("bowl width", 14.5 * INCH, "m", MAKER, f"roughing-in sheet, front view '14-1/2\" (368 mm)' ({KOHLER_SHEET})")
BOWL_L = figure("bowl length, back to front", 26.0 * INCH, "m", MAKER, "same sheet '26\" (660 mm)'")
WALL_TO_FRONT = figure("wall to bowl front", (27 + 5 / 8) * INCH, "m", MAKER, "same sheet '27-5/8\" (702 mm)' (the 10 in. rough-in)")
WATER_DEPTH = figure("water surface under the rim", 6.0 * INCH, "m", MAKER, "same sheet 'Water depth from rim 6″ (152 mm)'")
WATER_L = figure("water area, long", (11 + 3 / 8) * INCH, "m", MAKER, "same sheet 'Water area 11-3/8″ (289 mm) x 10-3/8″ (264 mm)'")
WATER_W = figure("water area, across", (10 + 3 / 8) * INCH, "m", MAKER, "same words")
RIM_H_REAL = figure("rim over the floor (the stand replaces it)", 16.5 * INCH, "m", MAKER, "same sheet '16-1/2\" (419 mm)'")
HOLE_W = figure("rim opening across", WATER_W, "m", ESTIMATE, "the 10-3/8 in. water area across, the least the opening can be")
RIM_W = figure("rim width round the opening", (BOWL_W - HOLE_W) / 2.0, "m", ESTIMATE, "(14-1/2 - 10-3/8 in.) / 2 at the sides, taken the same at the front")
FRONT_TO_BOLTS = figure("seat posts to bowl front", SEAT_L, "m", ESTIMATE, "the seat's front over the bowl's front")
HOLE_D0 = figure("opening's back edge in front of the seat posts", 0.040, "m", ESTIMATE, "room for the hinge bases round the 5-1/2 in. posts")
HOLE_D1 = FRONT_TO_BOLTS - RIM_W  # opening's front edge from the bolts (0.419 m)
HOLE_L = HOLE_D1 - HOLE_D0  # 0.379 m along
BACK_D = FRONT_TO_BOLTS - BOWL_L  # bowl's back from the bolts (-0.189 m: behind them)
WALL_D = FRONT_TO_BOLTS - WALL_TO_FRONT  # the wall from the bolts (-0.230 m)
RIM_T = 0.012  # m: the fixture's rim top (12 mm board)

# --- The specimen pan: Medline Specimen Collector Pan, 30 oz (DYND36600H single, DYND36600 case of 100) ---------------
PAN_SPAN = figure("pan width across its wings", 13.25 * INCH, "m", ESTIMATE, f"Office Depot listing 458238 '13-1/4 in.' ({PAN_LISTING})")
PAN_CHORD = figure("pan front to back", 8.75 * INCH, "m", ESTIMATE, "same listing '8-3/4 in.'")
PAN_H = figure("pan height", (3 + 15 / 16) * INCH, "m", ESTIMATE, "same listing '3-15/16 in.' (100.0 mm)")
BOWL_IN_W = figure("pan bowl width between the wings", 0.186, "m", ESTIMATE, "the wings' 75 mm each side to the 336.6 mm span, a common winged-pan proportion")
WING_CHORD = figure("wing depth front to back", 0.160, "m", ESTIMATE, "about 3/4 of the 222.3 mm front-to-back size, rounded wing ends")
WING_T = figure("wing thickness with its stiffening rib", 0.004, "m", ESTIMATE, "a 1 mm sheet on a 4 mm rib")
PAN_WALL = 0.004  # m: the simulated wall and bottom (a real wall is about 1 mm; the pinch closes on it the same way)
PAN_MASS = figure("pan mass", 0.080, "kg", ESTIMATE, "about 0.147 m2 of 0.6 mm polypropylene (905 kg/m3)")

# --- The stand and the layout (our fixture) -----------------------------------------------------------------------------
RIM_Z = figure("rim top over the bench (the stand)", 0.160, "m", FIXTURE, "the 152 mm water depth over an 8 mm floor; the stand replaces pedestal and floor")
WATER_Z = RIM_Z - WATER_DEPTH  # 0.0076 m: the well's floor stands for the water surface
BOWL_Y = -0.040  # m: the bowl's middle line (towards the right arm, which sets the pan)
FRONT_X = 0.070  # m: the bowl's front
BOLT_X = FRONT_X + FRONT_TO_BOLTS  # 0.5415 m: the seat's bolt (hinge) line
PINTLE_Z = RIM_Z + PINTLE_H
SEAT_DAMPING = 0.02  # N m s/rad: a free plastic hinge (estimate)
WALL_T, WALL_W, WALL_HIGH = 0.020, 0.600, 0.700  # m: the wall panel behind the bowl (our fixture)

PAN_START = (0.280, -0.400)  # the pan on the bench right of the bowl, lengthwise (its wings along x)
PAN_START_YAW_DEG = -90.0  # pan axes: x_p front to back (222.3 mm), y_p along the wings (336.6 mm)
PAN_PLACE_D = (HOLE_D0 + HOLE_D1) / 2.0  # 0.230 m: the demo sets the pan in the middle of the opening
PAN_PLACE_YAW_DEG = 0.0  # wings along y, across the opening

# --- Scoring limits (identical in task.py, task.json and README.md) ----------------------------------------------------
TILT_MAX_DEG = figure("pan tilt at most", 8.0, "deg", FIXTURE, "the pan upright")
ACROSS_MAX_DEG = figure("wings off square across the opening at most", 20.0, "deg", FIXTURE, "the wings across the opening")
WING_GAP_MAX = figure("wing undersides over the rim at most", 0.005, "m", FIXTURE, "both wings on the rim")
DROP_GAP = figure("pan fallen in: lowest point over the water at most", 0.008, "m", FIXTURE, "set on the rim it is 56 mm over it")
FALL_DEG = figure("seat fallen: under, nothing holding it", 90.0, "deg", FIXTURE, "past its balance at 94 deg it falls shut")
SEAT_UP_DEG = figure("seat up on its stop at least", 99.0, "deg", FIXTURE, "the check-hinge stop at 101 deg")
HANDS_CLEAR = figure("hands clear at the end at least", 0.030, "m", FIXTURE, "every part of both hands from seat, pan and bowl")
HOLD_HAND = 0.025  # m: a hand this near the seat holds it
REST_STEPS = 20  # 1.0 s at 20 Hz
REST_MOVE = 0.0015  # m the pan may move over the rest
REST_TURN_DEG = 0.5  # deg the seat may turn over the rest

W_SEAT = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # seat frame (pin along x_s, front at -y_s) to world


def seat_rot(deg: float) -> np.ndarray:
    """Rotation of the seat raised by ``deg`` (0 closed) in its own frame (about the pin, x_s; the front goes up)."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, s], [0.0, -s, c]])


def seat_world_rot(deg: float) -> np.ndarray:
    return W_SEAT @ seat_rot(deg)


def rz(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def pin(top: float) -> np.ndarray:
    return np.array([BOLT_X, BOWL_Y, top + PINTLE_Z])


def seat_point(top: float, x: float, d: float, h: float, deg: float) -> np.ndarray:
    """World point of the seat at (x across its middle line, d from the bolts towards its front, h over the pin as
    closed) when raised by ``deg``."""
    return pin(top) + seat_world_rot(deg) @ np.array([x, -d, h])


# Seat shapes in its own frame (origin on the hinge pin, x_s along the pin, -y_s towards the front, z_s up as closed):
# boxes as (centre, size), none overlapping, so that mass and inertia follow from the volumes.
RING_LOW = BUMPER_H - PINTLE_H  # ring underside, from the pin (as closed)
RING_MID = RING_LOW + SEAT_T / 2.0


def seat_boxes() -> list[tuple[tuple[float, float, float], tuple[float, float, float]]]:
    half_in, half_out, rear = SEAT_IN_W / 2.0, SEAT_W / 2.0, SEAT_REAR_W / 2.0
    zc, t = RING_MID, SEAT_T
    out = [((0.0, -(SEAT_OPEN_D0 - 0.015) / 2.0, zc), (2.0 * rear, SEAT_OPEN_D0 + 0.015, t))]  # hinge end, d -0.015..0.060
    for s in (-1.0, 1.0):
        out.append(((s * (rear + (half_out - 0.019 - rear) / 2.0), -(0.010 + SEAT_OPEN_D0) / 2.0, zc), (half_out - 0.019 - rear, SEAT_OPEN_D0 - 0.010, t)))
        arm_len = SEAT_HORN_D0 - SEAT_OPEN_D0
        out.append(((s * (half_in + half_out) / 2.0, -(SEAT_OPEN_D0 + arm_len / 2.0), zc), (half_out - half_in, arm_len, t)))
        horn_len = SEAT_L - SEAT_HORN_D0
        out.append(((s * (SEAT_GAP / 2.0 + SEAT_HORN_OUT) / 2.0, -(SEAT_HORN_D0 + horn_len / 2.0), zc), (SEAT_HORN_OUT - SEAT_GAP / 2.0, horn_len, t)))
    return out


def bumper_boxes() -> list[tuple[tuple[float, float, float], tuple[float, float, float]]]:
    return [((s * BUMPER_X, -d, RING_LOW - BUMPER_H / 2.0), (BUMPER_SIZE[0], BUMPER_SIZE[1], BUMPER_H)) for d in BUMPER_D for s in (-1.0, 1.0)]


def mass_properties(boxes, mass: float) -> tuple[np.ndarray, np.ndarray]:
    """Centre of mass and (Ixx, Iyy, Izz) about it of ``boxes`` (centre, size) of one density summing to ``mass``."""
    vols = np.array([sx * sy * sz for _, (sx, sy, sz) in boxes])
    ms = mass * vols / vols.sum()
    cs = np.array([c for c, _ in boxes])
    com = (ms[:, None] * cs).sum(axis=0) / mass
    inertia = np.zeros(3)
    for m, c, (sx, sy, sz) in zip(ms, cs, (s for _, s in boxes)):
        own = m / 12.0 * np.array([sy * sy + sz * sz, sx * sx + sz * sz, sx * sx + sy * sy])
        r = c - com
        inertia += own + m * np.array([r[1] ** 2 + r[2] ** 2, r[0] ** 2 + r[2] ** 2, r[0] ** 2 + r[1] ** 2])
    return com, inertia


def seat_torque(deg: float) -> float:
    """N m the seat's weight turns it shut with when raised by ``deg`` (negative: back onto its stop)."""
    com, _ = mass_properties(seat_boxes() + bumper_boxes(), SEAT_MASS)
    return SEAT_MASS * GRAVITY * float(np.hypot(com[1], com[2])) * math.cos(math.radians(deg) + math.atan2(com[2], -com[1]))


def seat_balance_deg() -> float:
    com, _ = mass_properties(seat_boxes() + bumper_boxes(), SEAT_MASS)
    return 90.0 - math.degrees(math.atan2(com[2], -com[1]))


def bowl_boxes() -> list[tuple[tuple[float, float, float], tuple[float, float, float], str]]:
    """The fixture's solid boxes (centre in the station frame over the bench top, size, kind): the bowl's top (the rim
    round the opening and the deck behind it that carries the seat posts), the bowl's inside under the opening down to
    the water surface, the stand that replaces the pedestal and the floor, and the wall behind."""
    hw, cy = HOLE_W / 2.0, BOWL_Y
    x0, x1 = BOLT_X - HOLE_D1, BOLT_X - HOLE_D0  # the opening's front and back edges
    xf, xb = FRONT_X, BOLT_X - BACK_D  # the bowl's front and back
    zr, t = RIM_Z - RIM_T / 2.0, RIM_T
    out = [
        (((xf + x0) / 2.0, cy, zr), (x0 - xf, BOWL_W, t), "rim"),  # front rim
        (((x1 + xb) / 2.0, cy, zr), (xb - x1, BOWL_W, t), "rim"),  # back of the bowl's top, the seat posts on it
    ]
    for s in (-1.0, 1.0):
        out.append((((x0 + x1) / 2.0, cy + s * (hw + BOWL_W / 2.0) / 2.0, zr), (x1 - x0, BOWL_W / 2.0 - hw, t), "rim"))  # side rims
    well_h = RIM_Z - RIM_T - WATER_Z
    for s in (-1.0, 1.0):  # the inside's side and end walls, 12 mm, just outside the opening, from the water to the rim
        out.append((((x0 + x1) / 2.0, cy + s * (hw + 0.006), WATER_Z + well_h / 2.0), (x1 - x0 + 0.024, 0.012, well_h), "inside"))
        out.append((((x0 + x1) / 2.0 + s * ((x1 - x0) / 2.0 + 0.006), cy, WATER_Z + well_h / 2.0), (0.012, HOLE_W, well_h), "inside"))
    out.append((((x0 + x1) / 2.0, cy, WATER_Z / 2.0), (x1 - x0 + 0.024, HOLE_W + 0.024, WATER_Z), "water"))
    zs, hs = (RIM_Z - RIM_T) / 2.0, RIM_Z - RIM_T  # the stand: the bowl's outline from the bench to under the rim
    out.append((((xf + x0 - 0.012) / 2.0, cy, zs), (x0 - 0.012 - xf, BOWL_W, hs), "stand"))
    out.append((((x1 + 0.012 + xb) / 2.0, cy, zs), (xb - x1 - 0.012, BOWL_W, hs), "stand"))
    side = BOWL_W / 2.0 - hw - 0.012
    for s in (-1.0, 1.0):
        out.append((((x0 + x1) / 2.0, cy + s * (hw + 0.012 + side / 2.0), zs), (x1 - x0 + 0.024, side, hs), "stand"))
    wall_x = BOLT_X - WALL_D
    out.append(((wall_x + WALL_T / 2.0, cy, WALL_HIGH / 2.0), (WALL_T, WALL_W, WALL_HIGH), "wall"))
    return out


def summary() -> dict:
    counts: dict[str, int] = {}
    for _, _, kind, _ in FIGURES.values():
        counts[kind] = counts.get(kind, 0) + 1
    com, _ = mass_properties(seat_boxes() + bumper_boxes(), SEAT_MASS)
    return {
        "counts_by_source": counts,
        "seat_com_from_pin_m": [round(float(v), 4) for v in com],
        "seat_balance_deg": round(seat_balance_deg(), 2),
        "seat_torque_Nm": {deg: round(seat_torque(deg), 3) for deg in (0, 60, 90, 101)},
        "opening_x_m": [round(BOLT_X - HOLE_D1, 4), round(BOLT_X - HOLE_D0, 4)],
        "opening_mm": [round(1000 * HOLE_W, 1), round(1000 * HOLE_L, 1)],
        "rim_width_mm": round(1000 * RIM_W, 1),
        "wing_on_rim_each_side_mm": round(500 * (PAN_SPAN - HOLE_W), 1),
        "lengthwise_room_mm": [round(1000 * (HOLE_W - PAN_CHORD), 1), round(1000 * (HOLE_L - PAN_SPAN), 1)],
        "wall_x_m": round(BOLT_X - WALL_D, 4),
        "figures": {k: {"value": v, "unit": u, "source": s, "note": n} for k, (v, u, s, n) in FIGURES.items()},
    }
