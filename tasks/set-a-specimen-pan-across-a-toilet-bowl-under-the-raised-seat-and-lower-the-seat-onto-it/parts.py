"""Every figure the specimen-pan task rests on, in one place with its true source, and the geometry built from them.

Source kinds in ``FIGURES`` (the README's key-figure table is this registry):
- ``maker``: Bemis's own product page and spec sheet of the 1955CT seat; Kohler's roughing-in sheet of the K-4302
  Highcrest elongated bowl, whose rim opening the deck's opening copies (URLs below, the makers' words quoted).
- ``our fixture``: the mock bowl-rim deck (12 mm birch plywood on pine risers), the bench layout and the scoring
  limits.
- ``estimate``: a derived figure, with the one-clause derivation; the first minutes of a real trial check each one with
  a caliper, a kitchen scale or a protractor app, and a different reading replaces the estimate.
The pan's outer sizes are the Office Depot listing's, kept as estimates; the first minutes check them with a caliper.

Station frame: metres from the midpoint between the arm bases (left y = +0.305, right y = -0.305), x forward, y left,
z up from the bench top. The seat's hinge (pintle) line runs along x at y = PINTLE_Y; ``d`` is the distance from that
line towards the seat's front (towards -y).
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
BOLT_SPREAD = figure("hinge bolt spread", 5.5 * INCH, "m", MAKER, "page 'Bolt Spread: 5.5 in', spec sheet '5-1/2\"'")
HINGE_END_H = figure("height of the hinge end", (2 + 3 / 8) * INCH, "m", MAKER, "spec sheet '2-3/8\"', page 'Height: 2.375 in'")
SEAT_MASS = figure("seat mass", 4.216 * 0.45359237, "kg", MAKER, "page 'Weight: 4.216 lbs' (all of it given to the seat)")
CHECK_HINGE_DEG = figure("seat's travel to its check-hinge stop", 101.0, "deg", MAKER, "page 'Check hinges prevent the seat from going 11° beyond vertical'")
BUMPER_H = SEAT_T_BUMPED - SEAT_T  # 7.9 mm

PINTLE_H = figure("hinge pin over the rim", 0.035, "m", ESTIMATE, "about 25 mm under the top of the 2-3/8 in. (60.3 mm) hinge end")
SEAT_REAR_W = figure("seat width across the hinge end", 0.240, "m", ESTIMATE, "scaled from the maker's drawing with its 14-1/2 in. width")
SEAT_OPEN_D0 = figure("seat opening's back edge from the hinge bolts", 0.060, "m", ESTIMATE, "scaled from the maker's drawing with its 18-9/16 in. length")
SEAT_HORN_D0 = figure("start of the front horns from the hinge bolts", 0.400, "m", ESTIMATE, "scaled from the maker's drawing")
SEAT_HORN_OUT = figure("half-width of the seat at its front horns", 0.160, "m", ESTIMATE, "scaled from the maker's drawing (the rounded front)")
BUMPER_D = (0.140, 0.340)  # m from the hinge bolts: the four bumpers' rows, scaled from the maker's side view (estimate)
BUMPER_X = 0.142  # m from the seat's middle line: under the middle of each side arm (estimate)
BUMPER_SIZE = (0.020, 0.030)  # m across x, along d (estimate)

# --- The specimen pan: Medline Specimen Collector Pan, 30 oz (DYND36600H single, DYND36600 case of 100) ---------------
PAN_SPAN = figure("pan width across its wings", 13.25 * INCH, "m", ESTIMATE, f"Office Depot listing 458238 '13-1/4 in.' ({PAN_LISTING})")
PAN_CHORD = figure("pan front to back", 8.75 * INCH, "m", ESTIMATE, "same listing '8-3/4 in.'")
PAN_H = figure("pan height", (3 + 15 / 16) * INCH, "m", ESTIMATE, "same listing '3-15/16 in.' (100.0 mm)")
BOWL_W = figure("bowl width between the wings", 0.186, "m", ESTIMATE, "the wings' 75 mm each side to the 336.6 mm span, a common winged-pan proportion")
WING_CHORD = figure("wing depth front to back", 0.160, "m", ESTIMATE, "about 3/4 of the 222.3 mm front-to-back size, rounded wing ends")
WING_T = figure("wing thickness with its stiffening rib", 0.004, "m", ESTIMATE, "a 1 mm sheet on a 4 mm rib")
PAN_WALL = 0.004  # m: the simulated wall and bottom (a real wall is about 1 mm; the pinch closes on it the same way)
PAN_MASS = figure("pan mass", 0.080, "kg", ESTIMATE, "about 0.147 m2 of 0.6 mm polypropylene (905 kg/m3)")

# --- The mock bowl-rim deck (our fixture) ------------------------------------------------------------------------------
DECK_T = figure("deck thickness", 0.012, "m", FIXTURE, "12 mm birch plywood")
RIM_Z = figure("rim top over the bench", 0.115, "m", FIXTURE, "103 mm pine riser blocks under the 12 mm deck")
HOLE_W = figure("opening width (across the seat)", (9 + 3 / 4) * INCH, "m", MAKER, f"Kohler K-4302 roughing-in sheet, bowl opening '9-3/4\" (248 mm)' ({KOHLER_SHEET})")
HOLE_L = figure("opening length (along the seat)", (14 + 11 / 16) * INCH, "m", MAKER, "same sheet '14-11/16\" (373 mm)' (the oval cut as a rectangle)")
HOLE_D0 = figure("opening's back edge from the hinge bolts", 0.080, "m", FIXTURE, "behind it the deck carries the hinge posts")
RIM_OUT = figure("deck half-width under the front of the seat", 0.184, "m", FIXTURE, "as wide as the seat")
RIM_BACK_OUT = figure("deck half-width under the back of the seat", 0.155, "m", FIXTURE, "the seat ring overhangs it by 29 mm, room for fingers")
BACK_NARROW_D = 0.245  # m: the deck is narrow from its back edge to here
DECK_D0, DECK_D1 = -0.045, 0.490  # m: the deck's back and front edges from the hinge line

HOLE_X = 0.280  # m: x of the opening's middle line (and of the seat's)
PINTLE_Y = 0.270  # m: y of the hinge line
PINTLE_Z = RIM_Z + PINTLE_H
SEAT_DAMPING = 0.02  # N m s/rad: a free plastic hinge (estimate)

# The pan starts on the bench to the right of the deck, lengthwise: its wings along y (it must be turned 90 deg).
PAN_START = (0.270, -0.430)
PAN_START_YAW_DEG = 0.0  # pan axes: x_p front to back (222.3 mm), y_p along the wings (336.6 mm)
PAN_PLACE_D = 0.305  # m: where the demo sets the pan's middle (from the hinge line)
PAN_PLACE_YAW_DEG = 90.0  # wings along x, across the opening

# --- Scoring limits (identical in task.py, task.json and README.md) ----------------------------------------------------
TILT_MAX_DEG = figure("pan tilt at most", 8.0, "deg", FIXTURE, "the pan upright")
ACROSS_MAX_DEG = figure("wings off square across the opening at most", 20.0, "deg", FIXTURE, "the wings across the opening")
WING_GAP_MAX = figure("wing undersides over the rim at most", 0.005, "m", FIXTURE, "both wings on the rim")
SEAT_DOWN_DEG = figure("seat lowered: angle at most", 3.0, "deg", FIXTURE, "flat on the rim or on the wings")
DROP_Z = figure("pan fallen through: lowest point over the bench at most", 0.008, "m", FIXTURE, "set on the rim it is 19 mm up")
LET_GO_DEG = figure("seat let go above", 20.0, "deg", FIXTURE, "no hand within 25 mm and nothing under it: it falls shut")
LET_GO_HAND = 0.025  # m
LET_GO_FALL_DEG = 2.0  # deg the unheld seat falls before it counts as let go
HANDS_CLEAR = figure("hands clear at the end at least", 0.030, "m", FIXTURE, "every part of both hands from seat, pan and deck")
REST_S = 1.0
REST_STEPS = 20  # 1.0 s at 20 Hz
REST_MOVE = 0.0015  # m the pan may move over the rest
REST_TURN_DEG = 0.5  # deg the seat may turn over the rest


def seat_rot(deg: float) -> np.ndarray:
    """Rotation of the seat raised by ``deg`` (0 closed) about the hinge line (axis -x: the front goes up)."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, s], [0.0, -s, c]])


def rz(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def seat_point(top: float, x: float, d: float, h: float, deg: float) -> np.ndarray:
    """World point of the seat at (x from its middle line, d from the hinge line towards the front, h over the hinge
    pin as built) when the seat is raised by ``deg``."""
    local = np.array([x, -d, h])
    return np.array([HOLE_X, PINTLE_Y, top + PINTLE_Z]) + seat_rot(deg) @ local


# Seat shapes in its own frame (origin on the hinge pin, x along the pin, -y towards the front, z up): boxes as
# (centre, size), none overlapping, so that mass and inertia follow from the volumes.
RING_LOW = BUMPER_H - PINTLE_H  # ring underside, from the pin (as built)
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
    """N m the seat's weight turns it shut with when raised by ``deg``."""
    com, _ = mass_properties(seat_boxes() + bumper_boxes(), SEAT_MASS)
    return SEAT_MASS * GRAVITY * float(np.hypot(com[1], com[2])) * math.cos(math.radians(deg) + math.atan2(com[2], -com[1]))


def summary() -> dict:
    counts: dict[str, int] = {}
    for _, _, kind, _ in FIGURES.values():
        counts[kind] = counts.get(kind, 0) + 1
    com, inertia = mass_properties(seat_boxes() + bumper_boxes(), SEAT_MASS)
    return {
        "counts_by_source": counts,
        "seat_com_from_pin_m": [round(float(v), 4) for v in com],
        "seat_inertia": [round(float(v), 5) for v in inertia],
        "seat_torque_Nm": {deg: round(seat_torque(deg), 3) for deg in (0, 30, 60, 90, 101)},
        "wing_on_rim_each_side_mm": round(500 * (PAN_SPAN - HOLE_W), 1),
        "chord_gap_each_side_mm": round(500 * (HOLE_W - PAN_CHORD), 2),
        "figures": {k: {"value": v, "unit": u, "source": s, "note": n} for k, (v, u, s, n) in FIGURES.items()},
    }
