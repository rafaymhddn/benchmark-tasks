"""Every figure the flip-socket task rests on, in one place, each with its source; and the shapes built from them.

Source kinds (``FIGURES`` holds every figure with its kind and its source in words; the README's table is made from it):
- ``maker``: Klein's own product page, sell sheet or photos, with the URL.
- ``standard``: a named figure from a standard or a materials table.
- ``listing``: a distributor's figure for the named catalog item.
- ``our fixture``: a part this task specifies and the bench sets to the stated size.
- ``estimate``: our figure, with how it was derived in one clause.

World frame (the station's): metres from the midpoint between the arm bases, x forward, y left, z up from the bench.
"""

from __future__ import annotations

import math

import numpy as np

GRAVITY = 9.81
OVER = 0.001  # m: a loose body is built this far over its support, so it settles at the start

MAKER, STANDARD, LISTING, FIXTURE, ESTIMATE = "maker", "standard", "listing", "our fixture", "estimate"
K32900 = "https://www.kleintools.com/catalog/multi-bit-nut-drivers/7-1-impact-flip-socket-handle"
K32933 = "https://www.kleintools.com/catalog/multi-bit-nut-drivers/6-1-impact-rated-flip-socket-set-metric"
SELLSHEET = "https://data.kleintools.com/sites/all/product_assets/documents/brochures/klein/32900_Impact_Flip_Socket_Drivers_SellSheet.pdf"
STANDOFF = "https://www.mfsupply.com/Hex_Standoffs_Cross_Reference_Table_s/4425.htm"
FIGURES: dict[str, tuple[float, str, str, str]] = {}  # name -> (value, unit, source kind, note)


def figure(name: str, value: float, unit: str, source: str, note: str) -> float:
    if source not in (MAKER, STANDARD, LISTING, FIXTURE, ESTIMATE):
        raise ValueError(f"{name}: unknown source kind {source!r}")
    FIGURES[name] = (float(value), unit, source, note)
    return float(value)


# --- the driver: Klein 32900 handle with its fixed 1/4-inch hex shaft --------------------------------------------------
TOOL = "Klein Tools 32900 7-in-1 Impact Flip Socket with Handle (UPC 092644329005)"
OVERALL = figure("driver overall length", 0.1780, "m", MAKER, f"'Overall Length (cm): 17.8' ({K32900})")
HANDLE_D = figure("handle across flats, the tool's widest part", 0.0381, "m", MAKER, f"'Overall Width (Inch): 1.5' ({K32900})")
TOTAL_MASS = figure("whole set mass", 0.272, "kg", MAKER, f"'Weight (g): 272' ({K32900})")
SHAFT_SET = figure("shaft with its sockets, the metric twin set 32933M", 0.129, "m", MAKER, f"'Overall Height (cm): 12.9' of the '6-in-1 Impact Rated Flip Socket Set, Metric' ({K32933})")
SHAFT_AF = figure("shaft across flats (1/4-inch hex)", 0.00635, "m", MAKER, "\"Drive Size: 1/4''\" (" + K32900 + ")")
figure("socket points", 6.0, "-", MAKER, f"'Number of Points: 6' ({K32900})")
figure("socket sizes, inch", 6.0, "sizes", MAKER, "\"Socket Size: 1/4'', 5/16'', 11/32'', 3/8'', 7/16'', 1/2''\" (" + K32900 + ")")
SEATED = figure("shaft seated in the handle collet", 0.028, "m", ESTIMATE, "the twin set's 129 mm less the 101 mm ahead of the grip; a different reading replaces the estimate")
HANDLE_L = figure("handle length", 0.077, "m", ESTIMATE, "the cushion grip read against the 178 mm overall in the maker's photos; a different reading replaces the estimate")
EXPOSED = OVERALL - HANDLE_L  # m of shaft ahead of the handle nose, 0.101 (its last 26 mm to the maker's 127 mm shaft sit in the collet)
figure("shaft ahead of the handle nose", EXPOSED, "m", ESTIMATE, "the maker's 178 mm less the 77 mm grip; a different reading replaces the estimate")
GRIP_FRICTION = figure("cushion grip friction", 1.0, "-", ESTIMATE, "rubber cushion grip against the fingers' pads, near the kit's graspable 1.2; a different reading replaces the estimate")

# --- the three reversible barrels: two hex sockets a piece, colour bands at both ends -----------------------------------
# A barrel's own frame: x along its bore, +x towards the FIRST-named end (red 1/4, green 11/32, brown 7/16). The through-bore
# rides the shaft and is itself the multi-function 1/4 socket at the red end (maker: "Multi-function 1/4 socket has both
# shallow (driver) and deep (bit holder) configurations"); the other end carries its counterbore.
STEEL = figure("barrel steel density", 7850.0, "kg/m3", STANDARD, "the 7.85 g/cm3 of hardened alloy steel in any materials table")
CLEAR_FLAT = figure("socket bore clearance over its hex, per flat", 0.00018, "m", ESTIMATE,
                    "a broached socket's working clearance at these sizes; a different reading replaces the estimate")
CLEAR_SHAFT = 0.000075  # m a flat: the through-bore's clearance over the 1/4-inch shaft (a slide fit that still indexes)
SIZES = {"14": 0.25, "516": 5.0 / 16.0, "1132": 11.0 / 32.0, "38": 3.0 / 8.0, "716": 7.0 / 16.0, "12": 0.5}  # inches, maker
BORE_AF = figure("barrel through-bore across flats (the 1/4 socket)", SHAFT_AF + 2.0 * CLEAR_SHAFT, "m", ESTIMATE,
                 "the 1/4-inch shaft's 6.35 mm plus 0.075 mm a flat; the red end of the small barrel drives 1/4-inch hex with this same bore")
CHAMFER = figure("socket mouth lead-in chamfer, depth", 0.0015, "m", ESTIMATE, "the funnel mouth of a broached socket; a different reading replaces the estimate")
CHAMFER_WIDE = 0.00085  # m a chamfer ring opens over its bore's across-flats
MOUTH = 0.8 * CHAMFER  # m: each through-bore mouth's stepped lead-in, 1.2 mm deep
DEEP = figure("3/8 socket depth", 0.015, "m", ESTIMATE,
              "deep enough that the 7.5 mm standoff seats 5 mm or more before its far face reaches the bore's shoulder; a different reading replaces the estimate")
SHALLOW = figure("other sockets' depth", 0.008, "m", ESTIMATE, "the maker: 'Shallow socket depth optimized for hex head fasteners'; a different reading replaces the estimate")


def bore_af(size_in: float) -> float:
    """A socket end's across-flats: the hex it drives plus two clearances."""
    return size_in * 0.0254 + 2.0 * CLEAR_FLAT


def _hex_area(af: float) -> float:
    return math.sqrt(3.0) / 2.0 * af * af


class Barrel:
    """One reversible double-ended barrel: length, across-flats of its hexagonal outside, its ends' colours and bores."""

    def __init__(self, key: str, ends: tuple[str, str], length: float, od: float, bores: dict) -> None:
        self.key, self.ends, self.od, self.length, self.bores = key, ends, od, length, bores
        a, b = bores.get(ends[0]), bores.get(ends[1])
        self.through = (-length / 2 + (b[1] if b else 0.0), length / 2 - (a[1] if a else 0.0))  # the through-bore that rides the shaft, in its frame
        self.volume = (_hex_area(od) - _hex_area(BORE_AF)) * length
        for af, depth in bores.values():
            self.volume -= (_hex_area(af) - _hex_area(BORE_AF)) * depth  # a counterbore widens the bore over its depth only
        self.mass = figure(f"{key} mass", self.volume * STEEL, "kg", ESTIMATE,
                           f"its hexagon tube less its bores at 7.85 g/cm3; a different reading replaces the estimate")
        figure(f"{key} length", length, "m", ESTIMATE, "read against the 178 mm overall in the maker's photos; a different reading replaces the estimate")
        figure(f"{key} outside, across flats", od, "m", ESTIMATE, "the hexagonal outside read against the 178 mm overall in the maker's photos; a different reading replaces the estimate")


BARRELS = (
    Barrel("barrel_14_516", ("red", "yellow"), 0.0285, 0.0135, {"yellow": (bore_af(SIZES["516"]), SHALLOW)}),
    Barrel("barrel_1132_38", ("green", "blue"), 0.0300, 0.0156, {"green": (bore_af(SIZES["1132"]), SHALLOW), "blue": (bore_af(SIZES["38"]), DEEP)}),
    Barrel("barrel_716_12", ("brown", "red"), 0.0310, 0.0188, {"brown": (bore_af(SIZES["716"]), SHALLOW), "red": (bore_af(SIZES["12"]), SHALLOW)}),
)
figure("widest barrel, the metric twin set", 0.019, "m", MAKER, f"'Overall Width (cm): 1.9' of the 32933M set ({K32933})")
BARREL_MASS = sum(b.mass for b in BARRELS)
HANDLE_MASS = figure("handle and shaft mass", TOTAL_MASS - BARREL_MASS, "kg", ESTIMATE,
                     "the maker's 272 g less the three barrels' steel (below): the balance sits in the grip")
BANDS = {"red": (0.85, 0.10, 0.10), "yellow": (0.95, 0.80, 0.10), "green": (0.15, 0.65, 0.25), "blue": (0.15, 0.35, 0.85), "brown": (0.50, 0.32, 0.15)}
BORE_FRICTION = figure("barrel finish against the fingers' pads", 1.2, "-", ESTIMATE,
                       "manganese phosphate against rubber pads, the kit's graspable pair (the standoff and the shaft meet it at their own, lower figures); a different reading replaces the estimate")
SHAFT_SLIDE = figure("socket bores on the maker's shaft, friction", 0.16, "-", ESTIMATE,
                     "the maker's reversible sockets must slide freely on its shaft, the tool's own function: handbook lubricated steel on steel 0.10-0.16, read 0.16 (the shaft's surface, so every bore that touches it slides at this); a different reading replaces the estimate")
TIP_CH = figure("shaft tip lead-in, length", 0.0015, "m", ESTIMATE, "the crown chamfer on the end of a 1/4-inch hex bit shank; a different reading replaces the estimate")
TIP_CH_AF = figure("shaft tip lead-in, across flats at the very tip", 0.0050, "m", ESTIMATE, "the same chamfer, 0.7 mm a flat off the 6.35 mm hex; a different reading replaces the estimate")

# --- the fixed 3/8-inch hex the driver seats on ------------------------------------------------------------------------
POST_AF = figure("hex standoff across flats", 0.00953, "m", LISTING, f"'3/8 Hex X 3/8 Long X 10-32 ID Threaded Female Standoff 303 Stainless Steel' (RAF 2236-1032-SS and its equals, {STANDOFF})")
POST_L = figure("hex standoff length", 0.00953, "m", LISTING, f"'3/8 Hex X 3/8 Long X 10-32 ...' ({STANDOFF})")
POST_OUT = figure("standoff hex standing out of the bracket", 0.0075, "m", FIXTURE, "threaded 2 mm into a tapped hole: 7.5 mm of its hex stands proud")
POST_TWIST = figure("standoff flats turned from level", 15.0, "deg", FIXTURE, "the hex is mounted a quarter turn of a flat off level, so the tool must be clocked to seat")
POST_CH = figure("standoff lead-in chamfer, length", 0.0012, "m", ESTIMATE, "the usual crown chamfer on a hex end; a different reading replaces the estimate")
POST_CH_AF = figure("standoff lead-in, across flats at its face", 0.0081, "m", ESTIMATE, "the same crown chamfer, 0.7 mm a flat off the 9.53 mm hex; a different reading replaces the estimate")
POST_FRICTION = figure("standoff hex friction against a socket", 0.35, "-", ESTIMATE, "dry steel on stainless, the low end of the handbooks' 0.35-0.5; a different reading replaces the estimate")

# --- the fits (derived from the figures above; the tests check them) ----------------------------------------------------
SHAFT_AC = SHAFT_AF * 2.0 / math.sqrt(3.0)  # 7.33 mm across corners
SOCKET_38 = bore_af(SIZES["38"])  # the blue socket's across-flats, 9.89 mm
PLAY = math.pi / 6.0 - math.acos(BORE_AF / SHAFT_AC)  # how far a barrel may turn on the shaft, 2.7 deg
CLOCK = math.pi / 6.0 - math.acos(SOCKET_38 / 2.0 / (POST_AF / math.sqrt(3.0)))  # the seating clocking window, 4.5 deg


# --- shape helpers: every hex is exact ----------------------------------------------------------------------------------
def rot_x(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def hex_prism(af: float, length: float, twist: float = 0.0) -> list[tuple[tuple[float, float, float], np.ndarray, np.ndarray]]:
    """Three boxes whose union is exactly a hexagonal prism of ``af`` across flats, its axis on x, centred at the
    origin: each box's long faces lie on two opposite flats (its four corners are four of the hexagon's corners)."""
    return [((length, af / math.sqrt(3.0), af), np.zeros(3), rot_x(twist + k * math.pi / 3.0)) for k in range(3)]


def hex_frustum(af_narrow: float, af_wide: float, length: float, thickness: float = 0.0006, twist: float = 0.0) -> list[tuple[tuple[float, float, float], np.ndarray, np.ndarray]]:
    """A hexagonal lead-in along x, ``af_narrow`` across flats at x = 0 widening to ``af_wide`` at x = ``length``:
    a core prism of the narrow hex and six plates, each tilted so its outer face is one sloped flat (each plate as
    wide as the wide end's flat, so at the narrow end its corners stand a little past the narrow hex's, still
    inside the wide one's corners)."""
    d0, d1 = af_narrow / 2.0, af_wide / 2.0
    rise = d1 - d0
    slant = math.hypot(length, rise)
    out = [(dims, local + np.array([length / 2.0, 0.0, 0.0]), rot) for dims, local, rot in hex_prism(af_narrow, length, twist)]
    x_c, r_c = length / 2.0 + rise * thickness / (2.0 * slant), (d0 + d1) / 2.0 - length * thickness / (2.0 * slant)
    for k in range(6):
        turn = rot_x(twist + k * math.pi / 3.0)
        radial, tangent = turn @ np.array([0.0, 0.0, 1.0]), turn @ np.array([0.0, 1.0, 0.0])
        along = (length * np.array([1.0, 0.0, 0.0]) + rise * radial) / slant
        normal = (-rise * np.array([1.0, 0.0, 0.0]) + length * radial) / slant
        out.append(((slant, af_wide / math.sqrt(3.0), thickness), x_c * np.array([1.0, 0.0, 0.0]) + r_c * radial, np.column_stack([along, tangent, normal])))
    return out


def walls(n: int, inradius: float, span: tuple[float, float], thickness: float, twist: float = 0.0, overlap: float = 1.0) -> list[tuple[tuple[float, float, float], np.ndarray, np.ndarray]]:
    """``n`` boxes forming a regular prism tube of ``inradius`` (a bore's across-flats is twice this) between x
    ``span``: each wall's inner face on a flat of the bore, the corners meeting exactly (an overlap would poke
    into the bore and jam whatever rides it)."""
    side = 2.0 * inradius * math.tan(math.pi / n)
    out = []
    for k in range(n):
        a = twist + k * 2.0 * math.pi / n
        r = rot_x(a)
        at = r @ np.array([0.0, 0.0, inradius + thickness / 2.0]) + np.array([(span[0] + span[1]) / 2.0, 0.0, 0.0])
        out.append(((span[1] - span[0], side * overlap, thickness), at, r))
    return out


# --- the layout: the driver along y, tip towards the bracket (our fixture) ----------------------------------------------
DRIVER_X = 0.245  # m: the driver's line, forward of the arms' midpoint
AXIS_Z = figure("shaft axis height over the bench", 0.040, "m", FIXTURE, "the handle's centre height in its cradle, and the standoff's axis")
TIP_Y = figure("shaft tip at the start", 0.006, "m", FIXTURE, "the tool lies in its start cradle with its tip towards the bracket")
TIP_TO_MID = EXPOSED + HANDLE_L / 2.0  # m: shaft tip to handle centre, 0.1395
HANDLE_MID = TIP_Y + TIP_TO_MID  # m: the handle's centre at the start, 0.1455
CRADLE_END_Y = figure("end cradle centre", -0.032, "m", FIXTURE, "the seated handle's front 31 mm rests in its back part; its front stops 55 mm from the arm bases' line, so nothing of it lies under the barrels by the bracket")
END_CRADLE_L = figure("end cradle length along y", 0.046, "m", FIXTURE, "the seated handle's front 31 mm and 15 mm under the shaft ahead of it: the right hand's fingers, under the green/blue barrel by the bracket, stay clear of it")
CRADLE_L = figure("start cradle length along y", 0.130, "m", FIXTURE, "long enough that the tool with all barrels stored stands stable; the shaft floats above the cradle's open V")
CRADLE_RISE = figure("cradle V walls' rise", 0.024, "m", FIXTURE, "a 120-degree V (walls 30 degrees off level): the hex grip beds corner-down with both its lower faces flat on the walls, so it can neither roll nor pitch the tool")
CRADLE_VERTEX = figure("cradle V vertex height", 0.0180, "m", FIXTURE, "the handle's centre rides 22.0 mm above the corner (the hex's corner radius), putting the shaft axis at 40 mm")
_GRIP_R = HANDLE_D / math.sqrt(3.0)  # the hex grip's corner radius
END_VERTEX = figure("end cradle V vertex height", CRADLE_VERTEX - _GRIP_R * (math.cos(math.radians(30.0 - 15.0)) / math.cos(math.radians(30.0)) - 1.0), "m", FIXTURE,
                    "cut 2.54 mm lower than the start cradle's: the seated tool lies rolled 15 degrees to the standoff's flats, which lifts the hex grip that much in a 120-degree V")
CRADLE_FRICTION = figure("cradle V friction against the tool", 1.2, "-", FIXTURE,
                         "the V lined with a rubber strip: the tool beds in it and neither slides along its axis nor pitches while a hand works its barrels off the tip")
STAND_X = figure("stands' line", 0.170, "m", FIXTURE, "two parking stands 75 mm behind the tool's line, clear of its cradles and the bracket")
STAND_Y = {"barrel_14_516": -0.030, "barrel_1132_38": -0.075}  # m: each stand's centre (our fixture)
STAND_GAP = figure("stand rails apart, centre to centre", 0.024, "m", FIXTURE, "each stand's two rails carry a barrel 28.5-30 mm long near its ends; the 21 mm between them lets the finger pads reach down past the barrel's middle")
STAND_RAIL = (0.030, 0.003)  # m: each rail's length along x and thickness along y (our fixture)
BRACKET_AT = (0.245, -0.175)  # m: the bracket's centre on the bench
POST_Y = figure("standoff hex centre", -0.145, "m", FIXTURE, "7.5 mm standing out of the bracket's face: its near face at y = -0.14125")
POST_Z = AXIS_Z
POST_FACE_Y = POST_Y + POST_OUT / 2.0  # m: the standoff's near face, -0.14125
WALL_FACE = POST_FACE_Y - POST_OUT  # m: the bracket's front face: the 9.53 mm hex threaded 2 mm into it, 7.5 mm proud
END_MID = figure("seated handle centre", POST_FACE_Y + TIP_TO_MID, "m", FIXTURE, "the shaft's tip against the standoff's near face (y = -0.14125), the handle centre 139.5 mm behind the tip")
MAT = (0.420, 0.350, 0.004)  # m: the silicone bench mat (its centre at x 0.165, y -0.035)
MAT_AT = (0.165, -0.035)

# --- the score's limits (task.py, task.json, README: the same numbers) ---------------------------------------------------
FLUSH = 0.003  # m: the working barrel's face within this of flush with the shaft tip
FLIP_DEG = 15.0  # the blue end outboard within this
ON_TILT_DEG = 10.0  # a stored barrel coaxial with the shaft within this
ON_RADIAL = 0.002  # m: a stored barrel's axis within this of the shaft's
SEAT_MIN = 0.005  # m: the mouth this far past the standoff's near face
SEAT_DEG = 5.0  # the socket's axis within this of the standoff's
SEAT_RADIAL = 0.0025  # m: the socket's centre within this of the standoff's axis
LEVEL_DEG = 8.0  # the driver level within this
REST_NEAR = 0.025  # m: the handle centre within this of its cradle's middle
HANDS_CLEAR = 0.050  # m: every hull corner of both hands this far from the task's parts
GRIP_OPEN = 0.60  # a gripper counts as open from this share of its widest (40 mm of the 66 mm)
HOLD_STEPS = 40  # 2.0 s at 20 Hz with the hands clear
HOLD_DRIFT = 0.003  # m the driver may move during the hold
DROP_Z = 0.030  # m under the bench top: a part this low has left the bench


def driver_props() -> tuple[float, float, tuple[float, float, float]]:
    """(mass kg, centre of mass x in the tip frame m, inertia diag about it kg m2): the steel hex shaft as a rod of
    its hex area, the grip and collet as a cylinder of the handle's diameter carrying the rest of the maker's 272 g."""
    m_s = _hex_area(SHAFT_AF) * EXPOSED * STEEL
    m_g = HANDLE_MASS - m_s
    r_s = math.sqrt(_hex_area(SHAFT_AF) / math.pi)
    r_g = HANDLE_D / 2.0
    rods = ((m_s, EXPOSED / 2.0, EXPOSED, r_s), (m_g, EXPOSED + HANDLE_L / 2.0, HANDLE_L, r_g))  # mass, centre, length, radius
    com = sum(m * at for m, at, _, _ in rods) / HANDLE_MASS
    inertia = tuple(float(sum((0.5 * m * r * r if axis == 0 else m * (3.0 * r * r + l * l) / 12.0) + (0.0 if axis == 0 else m * (at - com) ** 2)
                              for m, at, l, r in rods)) for axis in range(3))
    return HANDLE_MASS, com, (inertia[0], inertia[1], inertia[2])


def summary() -> dict:
    return {"figures": {n: {"value": v, "unit": u, "source": s, "note": t} for n, (v, u, s, t) in FIGURES.items()},
            "exposed shaft (mm)": round(1000 * EXPOSED, 1), "tip to handle centre (mm)": round(1000 * TIP_TO_MID, 1),
            "shaft adds up (127 mm = exposed + seated)": round(1000 * (EXPOSED + SEATED), 1),
            "bores across flats (mm)": {"through/1-4": round(1000 * BORE_AF, 3), "5/16": round(1000 * bore_af(SIZES["516"]), 3), "11/32": round(1000 * bore_af(SIZES["1132"]), 3),
                                        "3/8": round(1000 * SOCKET_38, 3), "7/16": round(1000 * bore_af(SIZES["716"]), 3), "1/2": round(1000 * bore_af(SIZES["12"]), 3)},
            "3/8 socket window: clocking (deg), deepest seat (mm)": [round(math.degrees(CLOCK), 2), round(1000 * (DEEP - POST_OUT), 1)],
            "barrel play on the shaft (deg)": round(math.degrees(PLAY), 2),
            "smaller bores butt (all under 9.53 mm)": {k: round(1000 * bore_af(v), 2) for k, v in SIZES.items() if v < SIZES["38"]},
            "masses (g)": {"handle+shaft": round(1000 * HANDLE_MASS, 1), **{b.key: round(1000 * b.mass, 1) for b in BARRELS},
                           "total": round(1000 * (HANDLE_MASS + BARREL_MASS), 1), "maker": 272.0}}  # fmt: skip
