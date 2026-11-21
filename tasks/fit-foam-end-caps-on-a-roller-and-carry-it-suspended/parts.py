"""Every figure the end-cap suspension packing task rests on, in one place, each with its source.

Source kinds (``FIGURES`` holds every figure with its kind and its source in words; the README's table is made from it):
- ``maker``: the maker's own published product specification (ULINE S-4600 corrugated carton, ULINE S-7347 pre-assembled
  foam shipper kit, AstraRoll industrial roller packaging guide).
- ``listing``: a distributor's or supplier's published product specification (RS Components stock no. 761-3149 for
  Interroll Series 1700 roller RD-17W1JF4EAB RL200; Foam Factory 1.7LB Density Blue Polyethylene 2 in plank).
- ``our fixture``: a cut foam cushion or bench cradle specified by exact dimensions here for reproducible trials.
- ``estimate``: derived figure or simulation stand-in, with its one-clause derivation.

World frame (the station's): metres from the midpoint between the arm bases, x forward, y left, z up from the bench.
"""

from __future__ import annotations

import math

import numpy as np

GRAVITY = 9.81
OVER = 0.0005  # m: initial clearance above a supporting face so PhysX settles cleanly on step 1

MAKER, LISTING, FIXTURE, ESTIMATE = "maker", "listing", "our fixture", "estimate"

ULINE_BOX_URL = "https://www.uline.com/Product/Detail/S-4600/Corrugated-Boxes-200-Test/10-x-6-x-3-Corrugated-Boxes"
ULINE_KIT_URL = "https://www.uline.com/Product/Detail/S-7347/Paint-Cans-and-Shippers/1-x-1-Quart-Foam-Shipper-Kit"
RS_ROLLER_URL = "https://uk.rs-online.com/web/p/conveyor-rollers/7613149"
BLUE_FOAM_URL = "https://www.thefoamfactory.com/closedcellfoam/polyethylene.html"
ASTRAROLL_URL = "https://www.astraroll.com/AstraRoll-Roller-Packaging.html"
FOAMPACKAGE_GUIDE_URL = "https://foampackage.com/guides/preparing-a-foam-insert-drawing"
ONET_PACKERS_URL = "https://www.onetonline.org/link/summary/53-7064.00"
ONET_MOVERS_URL = "https://www.onetonline.org/link/summary/53-7062.00"

FIGURES: dict[str, tuple[float, str, str, str]] = {}


def figure(name: str, value: float, unit: str, source: str, note: str) -> float:
    if source not in (MAKER, LISTING, FIXTURE, ESTIMATE):
        raise ValueError(f"{name}: unknown source kind {source!r}")
    FIGURES[name] = (float(value), unit, source, note)
    return float(value)


# --- 1. Open-top shipping carton (ULINE S-4600, top flaps trimmed flush at the crease) --------------------------------
BOX_L = figure(
    "carton inside length (along y)",
    0.2540,
    "m",
    MAKER,
    f'ULINE SKU S-4600 ({ULINE_BOX_URL}): \'10 x 6 x 3" Corrugated Boxes\', 10 in = 254.0 mm',
)
BOX_W = figure(
    "carton inside width (along x)",
    0.1524,
    "m",
    MAKER,
    f'ULINE SKU S-4600 ({ULINE_BOX_URL}): \'10 x 6 x 3" Corrugated Boxes\', 6 in = 152.4 mm',
)
BOX_H = figure(
    "carton inside wall height (z)",
    0.0762,
    "m",
    MAKER,
    f'ULINE SKU S-4600 ({ULINE_BOX_URL}): \'10 x 6 x 3" Corrugated Boxes\', 3 in = 76.2 mm',
)
BOX_WALL = figure(
    "carton C-flute board thickness",
    0.0040,
    "m",
    ESTIMATE,
    f"standard 4.0 mm single-wall C-flute board caliper for ULINE S-4600 ('C flute', '200 LB. TEST', {ULINE_BOX_URL})",
)
X_CARTON = figure(
    "carton centre ahead of arm bases, x",
    0.3600,
    "m",
    FIXTURE,
    "centred on y = 0, taped square to the bench 36.0 cm in front of the arm-base line",
)
Z_FLOOR = BOX_WALL  # 0.0040 m over bench
Z_RIM = Z_FLOOR + BOX_H  # 0.0802 m over bench
CARTON_FRICTION = figure(
    "corrugated kraft liner friction",
    0.45,
    "-",
    ESTIMATE,
    "dry kraft linerboard against polyethylene foam, middle of 0.35-0.55",
)

# --- 2. Steel conveyor roller (Interroll Series 1700, Mfr. RD-17W1JF4EAB RL200, RS 761-3149) -------------------------
ROLLER_D = figure(
    "roller tube outer diameter",
    0.0400,
    "m",
    LISTING,
    f"RS 761-3149 ({RS_ROLLER_URL}, Interroll RD-17W1JF4EAB RL200): 'Roller Diameter: 40mm'",
)
ROLLER_R = ROLLER_D / 2.0  # 0.0200 m
ROLLER_L = figure(
    "roller tube length",
    0.2000,
    "m",
    LISTING,
    f"RS 761-3149 ({RS_ROLLER_URL}, Interroll RD-17W1JF4EAB RL200): 'Roller Length: 200mm'",
)
SPINDLE_D = figure(
    "roller spring-spindle diameter",
    0.0080,
    "m",
    LISTING,
    f"RS 761-3149 ({RS_ROLLER_URL}): 'Spindle Size: 8 mm', 'Round', 'Spring Loaded'",
)
SPINDLE_R = SPINDLE_D / 2.0  # 0.0040 m
SPINDLE_L = figure(
    "roller overall length over spindles",
    0.2260,
    "m",
    LISTING,
    f"RS 761-3149 ({RS_ROLLER_URL}): 'Overall Length: 226mm' (13.0 mm spindle past each end of the 200 mm tube)",
)
SPINDLE_EXT = (SPINDLE_L - ROLLER_L) / 2.0  # 0.0130 m per end
ROLLER_WALL = 0.0015  # m: Interroll Series 1700 DIN 2394 zinc-plated steel tube wall 40 x 1.5 mm
ROLLER_MASS = figure(
    "roller total mass",
    0.400,
    "kg",
    ESTIMATE,
    "summed from Interroll Series 1700 40 x 1.5 x 200 mm steel tube (285 g) + 8 x 226 mm steel shaft (89 g) + two polyamide 6002 bearing housings (26 g)",
)
STEEL_MU = figure(
    "galvanised steel on polyethylene socket friction",
    0.20,
    "-",
    ESTIMATE,
    "smooth zinc-plated steel sliding against dry low-density polyethylene (0.18-0.25)",
)

# --- 3. Two die-cut blue closed-cell polyethylene foam end-caps (Foam Factory 1.7LB Blue 2 in PE plank) --------------
FOAM_DENSITY = figure(
    "blue closed-cell PE plank foam density",
    27.2,
    "kg/m3",
    LISTING,
    f"Foam Factory 'Polyethylene Foam - Blue' ('1.7LB Density Blue Polyethylene', '2\" x 48\" x 108\"', 1.7 lb/ft3 = 27.2 kg/m3, {BLUE_FOAM_URL})",
)
CAP_T = figure(
    "end-cap axial thickness (along y)",
    0.0460,
    "m",
    FIXTURE,
    f"trimmed from 2 in (50.8 mm) blue PE plank ({BLUE_FOAM_URL}); two nested caps leave 254 - 92 = 162 mm < 200 mm roller",
)
CAP_BASE_W = figure(
    "end-cap lower base width (along x)",
    0.1040,
    "m",
    FIXTURE,
    "104.0 mm wide lower base pad stabilises the 74 mm tall cap upright on the bench and carton floor",
)
CAP_BASE_H = figure(
    "end-cap lower base flange height (z)",
    0.0200,
    "m",
    FIXTURE,
    "side reliefs above z = 20 mm leave 40.2 mm finger room per side inside the carton",
)
CAP_TOP_W = figure(
    "end-cap upper socket-block width (along x)",
    0.0720,
    "m",
    FIXTURE,
    f"72.0 mm across x (within the 94 mm jaw), with side finger cut-outs as recommended by FoamPackage ({FOAMPACKAGE_GUIDE_URL})",
)
CAP_H = figure(
    "end-cap overall height (z)",
    0.0740,
    "m",
    FIXTURE,
    "sits 2.2 mm below the 80.2 mm carton rim when resting on the 4.0 mm carton floor",
)
SOCK_D = figure(
    "end-cap roller socket diameter",
    0.0450,
    "m",
    ESTIMATE,
    f"45.0 mm bore for the 40.0 mm roller (2.5 mm/side, our easy-removal choice; 'Cavities are normally cut slightly larger than the part so it can be placed and removed easily but cannot move.', {FOAMPACKAGE_GUIDE_URL})",
)
SOCK_R = SOCK_D / 2.0  # 0.0225 m
SOCK_Z = figure(
    "end-cap socket centre height from cap underside",
    0.0385,
    "m",
    FIXTURE,
    "puts the socket's bottom wall at z = 16.0 mm, suspending the 40.0 mm roller 16.0 mm clear above the carton floor",
)
SOCK_DEPTH = figure(
    "end-cap roller-shoulder socket depth",
    0.0210,
    "m",
    FIXTURE,
    "coupled length is (46 - 21) + 200 + (46 - 21) = 250.0 mm inside the 254.0 mm carton (2.0 mm/side clearance)",
)
POCKET_W = 0.0200  # m: 20 x 20 mm central relief pocket behind the shoulder so the 8 mm spindle floats unloaded
POCKET_DEPTH = figure(
    "end-cap spindle clearance pocket extra depth",
    0.0150,
    "m",
    FIXTURE,
    "15.0 mm pocket behind the 21.0 mm roller shoulder clears the 13.0 mm protruding spindle with 2.0 mm axial margin, leaving 10.0 mm solid outer foam",
)
BACK_WALL_T = CAP_T - SOCK_DEPTH - POCKET_DEPTH  # 0.0100 m solid outer wall

# Pre-kitted start positions inside carton: nested near the two y = +/-127.0 mm end walls (2.0 mm wall clearance)
CAP_START_Y = BOX_L / 2.0 - 0.0020 - CAP_T / 2.0  # 0.1020 m (cap_l at +0.1020, cap_r at -0.1020)
COUPLED_SPAN = 2.0 * (CAP_T - SOCK_DEPTH) + ROLLER_L  # 0.2500 m
COUPLED_CAP_Y = COUPLED_SPAN / 2.0 - CAP_T / 2.0  # 0.1020 m when both shoulders touch the 200 mm roller

# --- 4. Bench roller cradle in front of carton (our fixture: 18 mm birch saddle keeping roller coaxial at z = 38.5 mm)
X_ROLLER = figure(
    "bench roller cradle centre, x",
    0.2220,
    "m",
    FIXTURE,
    "13.8 cm in front of the carton's middle (57.8 mm clear ahead of the carton's outer front wall)",
)
CRADLE_L = figure(
    "bench roller cradle length (along y)",
    0.1000,
    "m",
    FIXTURE,
    "100.0 mm wide along y, leaving 50.0 mm of bare 40 mm roller overhanging each side for sliding on the end-caps",
)
CRADLE_W = 0.0560  # m along x
CRADLE_BASE_H = 0.0140  # m
CRADLE_RAIL_INNER_X = 0.0120  # m: 3-4-5 triangle with R = 20.0 mm -> roller axis sits sqrt(20^2 - 12^2) = 16.0 mm above rail top
CRADLE_RAIL_W = 0.0080  # m
CRADLE_RAIL_TOP = SOCK_Z - math.sqrt(ROLLER_R**2 - CRADLE_RAIL_INNER_X**2)  # 0.0385 - 0.0160 = 0.0225 m exact
CRADLE_FRICTION = 0.45

# --- 5. Scoring thresholds (identical in task.py, task.json, and README.md) -------------------------------------------
OFF_BENCH_Z = -0.010  # m below TABLE_TOP: whole-trial zero rule
OUT_HOLD_STEPS = 4  # 0.2 s at 20 Hz: both caps clearly extracted outside carton
ENGAGE_HALF = figure(
    "minimum socket engagement per end for 0.5 credit",
    0.0120,
    "m",
    FIXTURE,
    "at least 12.0 mm of each roller end inside its 21.0 mm end-cap socket",
)
ENGAGE_FULL = figure(
    "minimum socket engagement per end for 1.0 credit",
    0.0140,
    "m",
    FIXTURE,
    "at least 14.0 mm of each roller end inside its 21.0 mm socket (even at the full 4.0 mm carton play, minimum is 17.0 mm)",
)
RADIAL_HALF = 0.0060  # m: coaxial radial offset limit at 0.5
RADIAL_FULL = 0.0050  # m: coaxial radial offset limit at 1.0
COAX_MAX_DEG = 10.0  # deg: max angle between roller axis and cap socket axis
CAP_FLOOR_TOL = 0.0040  # m: cap underside within 4.0 mm of carton floor
CAP_TILT_MAX_DEG = 6.0  # deg: seated cap tilt from upright
CAP_TOP_MAX_Z = Z_RIM + 0.0020  # 0.0822 m: cap top at or below rim (+2 mm tolerance)
ROLLER_TILT_MAX_DEG = 5.0  # deg: suspended roller tilt from horizontal
SUSPEND_CLEAR_MIN = figure(
    "minimum clear gap under suspended roller for 1.0 credit",
    0.0100,
    "m",
    FIXTURE,
    "at least 10.0 mm clear under the roller tube above the carton floor (nominal seated gap is 16.0 mm; checked with a 10 mm bar)",
)
HANDS_CLEAR = figure(
    "minimum hand clearance at end of trial",
    0.0300,
    "m",
    FIXTURE,
    "every hull corner of both hands at least 30.0 mm clear of carton, roller, and both end-caps",
)
HOLD_STEPS = 20  # 1.0 s at 20 Hz stationary at the end
HOLD_DRIFT = 0.0020  # m: max body drift during the 1.0 s end hold


# --- 6. Analytical geometry, mass, centre of mass, and inertia helpers -----------------------------------------------
def cap_axis_boxes(inward: float) -> list[tuple[tuple[float, float, float], tuple[float, float, float], bool]]:
    """Non-overlapping axis-aligned boxes forming one end-cap in its frame (origin at bottom middle, z up).
    ``inward`` is -1.0 for ``cap_l`` (socket opens toward -y) and +1.0 for ``cap_r`` (socket opens toward +y).
    The third element marks whether the box lines the internal socket/shoulder (True -> smooth PE-on-steel friction)."""
    s = float(np.sign(inward))
    u_back = -CAP_T / 2.0 + BACK_WALL_T / 2.0  # -0.0180 m
    u_mid = -CAP_T / 2.0 + BACK_WALL_T + POCKET_DEPTH / 2.0  # -0.0055 m
    u_sock = CAP_T / 2.0 - SOCK_DEPTH / 2.0  # +0.0125 m
    wing_w = (CAP_BASE_W - CAP_TOP_W) / 2.0  # 0.0360 m
    wing_x = CAP_TOP_W / 2.0 + wing_w / 2.0  # 0.0540 m

    z_p_lo = SOCK_Z - POCKET_W / 2.0  # 0.0285 m
    z_p_hi = SOCK_Z + POCKET_W / 2.0  # 0.0485 m
    pillar_w = (CAP_TOP_W - POCKET_W) / 2.0  # 0.0260 m
    pillar_x = POCKET_W / 2.0 + pillar_w / 2.0  # 0.0230 m

    z_s_lo = SOCK_Z - SOCK_R  # 0.0160 m
    z_s_hi = SOCK_Z + SOCK_R  # 0.0610 m
    side_w = (CAP_TOP_W - 2.0 * SOCK_R) / 2.0  # 0.0135 m
    side_x = SOCK_R + side_w / 2.0  # 0.02925 m

    return [
        # Two full-depth lower side locator wings (x outside +/-36 mm up to +/-72 mm, z = 0..20 mm)
        ((wing_w, CAP_T, CAP_BASE_H), (-wing_x, 0.0, CAP_BASE_H / 2.0), False),
        ((wing_w, CAP_T, CAP_BASE_H), (+wing_x, 0.0, CAP_BASE_H / 2.0), False),
        # Zone 1: solid 10 mm outer back wall (x in [-36, +36] mm, z = 0..74 mm)
        ((CAP_TOP_W, BACK_WALL_T, CAP_H), (0.0, s * u_back, CAP_H / 2.0), False),
        # Zone 2: 15 mm shoulder wall with 20 x 20 mm spindle clearance pocket around (0, SOCK_Z)
        ((CAP_TOP_W, POCKET_DEPTH, z_p_lo), (0.0, s * u_mid, z_p_lo / 2.0), True),
        ((CAP_TOP_W, POCKET_DEPTH, CAP_H - z_p_hi), (0.0, s * u_mid, (z_p_hi + CAP_H) / 2.0), True),
        ((pillar_w, POCKET_DEPTH, POCKET_W), (-pillar_x, s * u_mid, SOCK_Z), True),
        ((pillar_w, POCKET_DEPTH, POCKET_W), (+pillar_x, s * u_mid, SOCK_Z), True),
        # Zone 3: 21 mm roller socket cardinal walls around [-SOCK_R, +SOCK_R] x [SOCK_Z - SOCK_R, SOCK_Z + SOCK_R]
        ((CAP_TOP_W, SOCK_DEPTH, z_s_lo), (0.0, s * u_sock, z_s_lo / 2.0), True),
        ((CAP_TOP_W, SOCK_DEPTH, CAP_H - z_s_hi), (0.0, s * u_sock, (z_s_hi + CAP_H) / 2.0), True),
        ((side_w, SOCK_DEPTH, 2.0 * SOCK_R), (-side_x, s * u_sock, SOCK_Z), True),
        ((side_w, SOCK_DEPTH, 2.0 * SOCK_R), (+side_x, s * u_sock, SOCK_Z), True),
    ]


def cap_bevel_boxes(inward: float) -> list[tuple[tuple[float, float, float], tuple[float, float, float], float]]:
    """Four 45-deg corner bevel fillets turning the 45 x 45 mm square into a true octagon of inradius SOCK_R = 22.5 mm.
    Returns (dims, pos, angle_about_y_rad)."""
    s = float(np.sign(inward))
    u_sock = CAP_T / 2.0 - SOCK_DEPTH / 2.0
    thick = 0.0080  # m radial thickness behind the r = 22.5 mm tangent face (outer reach 28.0 mm < 36.0 mm)
    width = 2.0 * SOCK_R * math.tan(math.pi / 8.0)  # 0.01864 m octagon side
    r_mid = SOCK_R + thick / 2.0  # 0.0265 m
    d = r_mid / math.sqrt(2.0)
    out = []
    for sx in (-1.0, 1.0):
        for sz in (-1.0, 1.0):
            # Rotate local z normal to point along (sx, 0, sz)/sqrt(2): R_y(theta) * [0,0,1] = [sin(theta), 0, cos(theta)]
            theta = math.atan2(sx, sz)
            out.append(((width, SOCK_DEPTH, thick), (sx * d, s * u_sock, SOCK_Z + sz * d), theta))
    return out


def _cap_volume() -> float:
    base_vol = sum(dx * dy * dz for (dx, dy, dz), _, _ in cap_axis_boxes(-1.0))
    # Subtract the 4 corner triangles removed by turning the 2*SOCK_R square into an octagon? No, bevels ADD the 4 corner fillets:
    # square area (2R)^2 minus octagon area 8 R^2 tan(pi/8) = R^2 (4 - 8 tan(pi/8))
    fillet_vol = SOCK_DEPTH * (SOCK_R**2) * (4.0 - 8.0 * math.tan(math.pi / 8.0))
    return base_vol + fillet_vol


CAP_VOLUME = _cap_volume()
TRUE_FOAM_CAP_MASS = round(CAP_VOLUME * FOAM_DENSITY, 4)  # 0.0063 kg (6.3 g) at 27.2 kg/m3
CAP_MASS = figure(
    "simulated foam end-cap mass stand-in",
    0.0233,
    "kg",
    ESTIMATE,
    "23.3 g simulation mass stand-in for stable 25 N rigid pinching (true 233 cm3 x 27.2 kg/m3 blue PE cap is 6.3 g; no scoring rule reads cap mass)",
)


def cap_com_and_inertia(inward: float) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Exact centre of mass and principal-axis inertia (kg m^2) of an end-cap from its non-overlapping boxes."""
    boxes = [(dx * dy * dz * FOAM_DENSITY, np.array([dx, dy, dz]), np.array(pos)) for (dx, dy, dz), pos, _ in cap_axis_boxes(inward)]
    total_m = sum(m for m, _, _ in boxes)
    scale = CAP_MASS / total_m
    boxes = [(m * scale, dims, pos) for m, dims, pos in boxes]
    com = sum((m * pos for m, _, pos in boxes), np.zeros(3)) / CAP_MASS
    inertia = np.zeros(3)
    for m, (a, b, c), pos in boxes:
        d = pos - com
        inertia += (m / 12.0) * np.array([b * b + c * c, a * a + c * c, a * a + b * b]) + m * np.array(
            [d[1] ** 2 + d[2] ** 2, d[0] ** 2 + d[2] ** 2, d[0] ** 2 + d[1] ** 2]
        )
    return (float(com[0]), float(com[1]), float(com[2])), (float(inertia[0]), float(inertia[1]), float(inertia[2]))


def roller_inertia() -> tuple[float, float, float]:
    """Principal inertia (Ixx, Iyy, Izz) of the hollow steel tube + coaxial steel shaft (y along roller axis)."""
    m_tube, m_shaft = 0.311, 0.089
    r_o, r_i = ROLLER_R, ROLLER_R - ROLLER_WALL
    iyy = 0.5 * m_tube * (r_o**2 + r_i**2) + 0.5 * m_shaft * (SPINDLE_R**2)
    ixx = (m_tube / 12.0) * (3.0 * (r_o**2 + r_i**2) + ROLLER_L**2) + (m_shaft / 12.0) * (3.0 * SPINDLE_R**2 + SPINDLE_L**2)
    return (float(ixx), float(iyy), float(ixx))


def summary() -> dict:
    counts: dict[str, int] = {}
    for _, _, kind, _ in FIGURES.values():
        counts[kind] = counts.get(kind, 0) + 1
    return {
        "counts": counts,
        "coupled_span_mm": round(1000 * COUPLED_SPAN, 1),
        "two_caps_gap_mm": round(1000 * (BOX_L - 2 * CAP_T), 1),
        "one_cap_gap_mm": round(1000 * (BOX_L - (CAP_T - SOCK_DEPTH + ROLLER_L)), 1),
        "seated_suspend_gap_mm": round(1000 * (SOCK_Z - SOCK_R), 1),
        "cap_mass_g": round(1000 * CAP_MASS, 2),
        "figures": {k: {"value": v, "unit": u, "kind": kind, "note": note} for k, (v, u, kind, note) in FIGURES.items()},
    }
