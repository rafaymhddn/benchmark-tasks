"""Every figure the hobby-burner stow task rests on, in one place, each with its source.

Source kinds (``FIGURES`` holds every figure with its kind and its source in words):
- ``maker``: Dremel's own published product pages and VersaFlame 2200 / 2200-01 instruction manuals.
- ``estimate``: derived figure with a one-clause geometric or mechanical derivation from the maker's figures.

World frame (the station's): metres from the midpoint between the arm bases, x forward, y left, z up from the bench.
"""

from __future__ import annotations

import math

GRAVITY = 9.81
OVER = 0.001  # m: loose bodies start 1 mm above their support so they settle cleanly on the first picture

MAKER, ESTIMATE = "maker", "estimate"
DREMEL_DE = "https://www.dremel.com/de/de/p/dremel-versaflame-f0132200ja"
DREMEL_GB = "https://www.dremel.com/gb/en/p/dremel-versaflame-f0132200ja"
DREMEL_2200_5 = "https://www.dremel.com/gb/en/p/dremel-versaflame-f0132200jd"
DREMEL_EU_MANUAL = "https://www.manualslib.com/manual/2598013/Bosch-Dremel-Versaflame-2200.html"
DREMEL_US_MANUAL = "https://www.manualslib.com/manual/3017443/Dremel-Versaflame-2200-01.html"

FIGURES: dict[str, tuple[float, str, str, str]] = {}


def figure(name: str, value: float, unit: str, source: str, note: str) -> float:
    if source not in (MAKER, ESTIMATE):
        raise ValueError(f"{name}: unknown source kind {source!r}")
    FIGURES[name] = (float(value), unit, source, note)
    return float(value)


# --- Dremel VersaFlame 2200-4 (Order F0132200JA, EAN 8710364042081) --------------------------------------------------
TOOL_H = figure(
    "burner standing height",
    0.170,
    "m",
    MAKER,
    f"'Laenge: 17 cm' ({DREMEL_DE}); 'Length: 17 cm' ({DREMEL_GB})",
)
STAND_D = figure(
    "removable round stand outer diameter",
    0.060,
    "m",
    MAKER,
    f"'Breite: 6 cm' ({DREMEL_DE}); 'attachable base that helps it stay upright when not in use' ({DREMEL_2200_5})",
)
TOOL_D = figure(
    "burner depth from rear ignition button to nozzle tip",
    0.110,
    "m",
    MAKER,
    f"'Tiefe: 11 cm' ({DREMEL_DE}); 'Depth: 11 cm' ({DREMEL_GB})",
)
TOOL_TOTAL_MASS = figure(
    "burner with removable stand total empty mass",
    0.227,
    "kg",
    MAKER,
    f"'Gewicht: 0,227 kg' ({DREMEL_DE}); 'Weight: 0.227 kg' ({DREMEL_GB})",
)
GROSS_KIT_MASS = figure(
    "packaged kit gross mass",
    0.900,
    "kg",
    MAKER,
    f"'Bruttogewicht, 0,90 kg' ({DREMEL_DE}); 'Gross weight, 0.90 kg' ({DREMEL_GB})",
)
CARTON_L = figure(
    "outer product box height",
    0.330,
    "m",
    MAKER,
    f"'Hoehe des Produktkartons, 33 cm' ({DREMEL_DE}); 'Product box height, 33 cm' ({DREMEL_GB})",
)
CARTON_W = figure(
    "outer product box length",
    0.177,
    "m",
    MAKER,
    f"'Laenge des Produktkartons, 17.7 cm' ({DREMEL_DE}); 'Product box length, 17.7 cm' ({DREMEL_GB})",
)
CARTON_H = figure(
    "outer product box width",
    0.055,
    "m",
    MAKER,
    f"'Breite des Produktkartons, 5.5 cm' ({DREMEL_DE}); 'Product box width, 5.5 cm' ({DREMEL_GB})",
)

# --- Burner cylinder, upper grip housing, nozzle, and stand socket ----------------------------------------------------
TANK_D = figure(
    "cylindrical tank outer diameter",
    0.038,
    "m",
    ESTIMATE,
    f"scaled as 0.63 of the 60 mm base diameter on the maker's orthographic product view ({DREMEL_DE})",
)
TANK_R = TANK_D / 2.0
TANK_H = 0.108  # m: lower cylindrical steel tank height before the molded blue upper housing
HOUSING_SIZE = (0.048, 0.026, 0.054)  # m: sculpted blue upper grip neck (26 mm wide above the 38 mm round tank)
CAP_SIZE = (0.046, 0.026, 0.008)  # m: dark grey top cap bringing total height to 170 mm
COLLAR_R, COLLAR_L = 0.012, 0.018  # m: burner neck collar along +x
NOZZLE_R, NOZZLE_L = 0.0105, 0.038  # m: stainless flame guard and nozzle out to x = +81 mm (110 mm from -29 mm trigger)
NOZZLE_Z = 0.148  # m: burner nozzle centerline height in burner frame

STAND_H = figure(
    "removable stand total height",
    0.024,
    "m",
    ESTIMATE,
    f"scaled as 0.40 of the 60 mm base diameter on the maker's side view ({DREMEL_DE}): 12 mm floor plus 12 mm cup wall",
)
SOCKET_DEPTH = figure(
    "stand socket axial engagement depth",
    0.012,
    "m",
    ESTIMATE,
    "upper half of the 24 mm molded base forming the cylindrical heel cup",
)
STAND_FLOOR_H = STAND_H - SOCKET_DEPTH  # 12 mm base disc under the tank heel
STAND_R = STAND_D / 2.0
SOCKET_CLEAR_R = TANK_R + 0.0014  # 20.4 mm inner cup radius: 1.4 mm radial clearance around the 38 mm tank

STAND_MASS = figure(
    "removable stand mass",
    0.035,
    "kg",
    ESTIMATE,
    "0.227 kg maker total split by volume between the molded 60 x 24 mm polymer stand (35 g) and metal burner (192 g)",
)
CARRIER_MASS = 0.002  # kg: internal socket liner puck carrying the axial slide joint
STAND_BODY_MASS = STAND_MASS - CARRIER_MASS  # 0.033 kg

CONTROL_MASS = 0.002  # kg each: trigger, lock_slider, child_stop
BURNER_TOTAL_MASS = TOOL_TOTAL_MASS - STAND_MASS  # 0.192 kg
BURNER_BODY_MASS = BURNER_TOTAL_MASS - 3.0 * CONTROL_MASS  # 0.186 kg

STAND_PULL_N = figure(
    "stand axial pull-off friction",
    8.0,
    "N",
    ESTIMATE,
    "3.6 times the 2.23 N tool weight so the attachable base stays on when the burner is lifted unless pulled axially",
)
HEEL_SLIDE_MAX = 0.0145  # m: slide limit slightly past the 12.0 mm socket release point
HEEL_K = 16000.0  # N/m: reaches the 8.0 N cap at 0.5 mm give
HEEL_GIVE = STAND_PULL_N / HEEL_K  # 0.0005 m
HEEL_DAMP = 40.0  # N s/m

# --- Coupled ignition button, continuous-flow lock button, and red child-safety stopper -------------------------------
TRIG_LAT = figure(
    "ignition button latched inward travel",
    0.0045,
    "m",
    ESTIMATE,
    f"three quarters of the 6.0 mm stroke on the rear ignition button ({DREMEL_EU_MANUAL}?page=5); no maker stroke figure",
)
TRIG_FULL = 0.0060  # m: full rear ignition button travel (1.5 mm overtravel past the 4.5 mm latch step)
TRIG_OVERTRAVEL = TRIG_FULL - TRIG_LAT  # 0.0015 m inward from built latched pose
TRIG_SPRING_N = figure(
    "ignition button return spring force at latch",
    16.0,
    "N",
    ESTIMATE,
    "16 N, our figure for a thumb piezo button; no maker figure",
)
TRIG_REST = 2.0 * TRIG_LAT  # 0.0090 m outward from built latched pose -> 16 N at latch, 8 N preload at outer stop
TRIG_K = TRIG_SPRING_N / TRIG_REST  # 1777.8 N/m
TRIG_DAMP = 8.0  # N s/m
TRIG_SIZE = (0.012, 0.014, 0.018)  # m: rear ignition button (Part 5)
TRIG_POS_B = (-0.0185, 0.0, 0.138)  # m in burner frame (latched rear face at -24.5 mm; unlatched at -29.0 mm)

LOCK_STROKE = figure(
    "continuous-flow lock slider stroke",
    0.0060,
    "m",
    ESTIMATE,
    f"6.0 mm thumb stroke on the side lock button ({DREMEL_EU_MANUAL}?page=5); no maker stroke figure",
)
LOCK_RELEASE_AT = 0.0045  # m: forward travel from rear locked stop (6.0 -> 1.5 mm remaining) where trigger unlatches
LOCK_DETENT_N = figure(
    "continuous-flow lock forward release force",
    10.0,
    "N",
    ESTIMATE,
    "molded polymer undercut catch resistance (~0.62 friction coefficient) under the 16.0 N trigger return spring",
)
LOCK_FREE_N = 2.5  # N: light thumb-slider drag once the 16 N trigger catch has cleared past 4.5 mm
LOCK_K = 20000.0  # N/m: reaches the 10.0 N cap at 0.5 mm give
LOCK_GIVE = LOCK_DETENT_N / LOCK_K  # 0.0005 m
LOCK_DAMP = 30.0  # N s/m
LOCK_SIZE = (0.012, 0.0060, 0.014)  # m: side lock button (Part 7), 6.0 mm proud of the -13 mm neck, flush with 19 mm tank radius
LOCK_POS_B = (-0.010, -0.0160, 0.124)  # m in burner frame at rear locked stop (y_b in [-19.0, -13.0] mm)

CHILD_STROKE = figure(
    "red child-safety lock reset travel",
    0.0060,
    "m",
    ESTIMATE,
    f"6.0 mm lateral travel on the red child-safety lock ({DREMEL_EU_MANUAL}?page=5); no maker stroke figure",
)
CHILD_SPRING_N = figure(
    "child-safety stopper return spring force",
    3.0,
    "N",
    ESTIMATE,
    "light return spring snapping the auto-resetting stopper back under the ignition button at mid-stroke",
)
CHILD_REST = 0.0090  # m: 3 mm past the 6 mm stop
CHILD_K = CHILD_SPRING_N / (CHILD_REST - 0.5 * CHILD_STROKE)  # 500.0 N/m
CHILD_DAMP = 0.8  # N s/m
CHILD_SIZE = (0.008, 0.010, 0.008)  # m: red stopper tab (Part 6)
CHILD_POS_B = (-0.021, 0.013, 0.121)  # m in burner frame while displaced 6 mm left (+y_b; outer edge at +18 mm < 19 mm)

# --- Metal storage tin ("Aufbewahrungsbox aus Blech", inside the 330 x 177 x 55 mm outer product carton) --------------
TIN_L = figure(
    "metal storage tin outer length",
    0.320,
    "m",
    ESTIMATE,
    "at most the 330 mm outer product carton height, less 10 mm cardboard end-flap allowance",
)
TIN_W = figure(
    "metal storage tin outer width",
    0.172,
    "m",
    ESTIMATE,
    "at most the 177 mm outer product carton length, less 5 mm cardboard sleeve clearance",
)
TIN_H = figure(
    "metal storage tin closed outer height",
    0.052,
    "m",
    ESTIMATE,
    "at most the 55 mm outer product carton width, less 3 mm cardboard sleeve wall (39 mm base + 13 mm lid)",
)
TIN_BASE_H = figure(
    "tin base outer wall height",
    0.039,
    "m",
    ESTIMATE,
    "three-quarter base / one-quarter lid split (39 mm + 13 mm) of the 52 mm closed tin height",
)
TIN_LID_H = TIN_H - TIN_BASE_H  # 0.013 m
TIN_WALL = 0.004  # m: rolled tinplate perimeter wall and protective liner thickness
TIN_FLOOR = 0.003  # m: base bottom panel plus thin dark foam floor pad
TIN_CEIL = 0.003  # m: lid top panel; closed interior vertical clearance = 52 - 6 = 46 mm (< 60 mm stand, > 38 mm tank)
TIN_CLEAR_Z = TIN_H - TIN_FLOOR - TIN_CEIL  # 0.046 m interior height

TIN_MASS = figure(
    "empty metal storage tin total mass",
    0.360,
    "kg",
    ESTIMATE,
    "0.5 mm tinplate (7,850 kg/m3) over 320 x 172 x 52 mm plus thin EVA tray (~360 g), within 0.90 kg gross kit mass",
)
TIN_BASE_MASS = 0.225  # kg
TIN_LID_MASS = TIN_MASS - TIN_BASE_MASS  # 0.135 kg
TIN_FRICTION = (0.85, 0.75)

LID_OPEN_DEG = figure(
    "tin lid open hinge stop angle",
    108.0,
    "deg",
    ESTIMATE,
    "rolled hinge stop 18 deg past vertical so the 135 g lid rests stably open under gravity without falling shut",
)
LID_OPEN_RAD = math.radians(LID_OPEN_DEG)
LID_DAMP = 0.018  # N m s/rad: sheet-metal hinge pin damping

# --- Scene layout on the bench ----------------------------------------------------------------------------------------
# Placed directly inside left_cam's READY field of view (centered at u=136, v=118 in 320x176) so the latched slide lock,
# depressed rear ignition button, and displaced red child-safety tab are directly resolvable at t = 0.0 s.
STAND_XY = (0.400, 0.350)
BURN_YAW_DEG = 18.0
BURN_YAW = math.radians(BURN_YAW_DEG)
WORK_XY = (0.300, 0.140)  # bimanual shutoff workspace point in front of both arms
WORK_YAW_DEG = -22.0
WORK_YAW = math.radians(WORK_YAW_DEG)
TIN_XY = (0.300, -0.115)  # open metal tin in front of the right arm, 320 mm along x, 172 mm along y, hinge on -y

# --- Scoring thresholds (identical in task.py, task.json, and README.md) ----------------------------------------------
LOCK_OFF_TRAVEL = 0.0045  # m: lock_slider moved >= 4.5 mm forward (within 1.5 mm of its 6.0 mm forward stop)
TRIG_OFF_TRAVEL = 0.0035  # m: trigger sprung >= 3.5 mm outward (within 1.0 mm of its 4.5 mm outer stop)
CHILD_OFF_TRAVEL = 0.0050  # m: red child_stop snapped >= 5.0 mm inward (within 1.0 mm of its 6.0 mm reset stop)
SEPARATE_GAP = 0.030  # m: detached burner heel and stand centers at least 30 mm apart
FLAT_TOP_MAX = 0.046  # m: highest solid point inside tin_base at most 46 mm above tin underside (below closed ceiling)
LID_SHUT_DEG = 2.0  # deg: closed lid within 2.0 deg of flat on tin_base rim
SEAM_GAP_MAX_MM = round(1000.0 * TIN_W * math.sin(math.radians(LID_SHUT_DEG)), 1)  # 6.0 mm front seam gap across 172 mm
LID_UNSAFE_DEG = 30.0  # deg: closing lid below 30 deg over an unextinguished burner loses the trial
BENCH_DROP_TOL = 0.010  # m: any object dropping > 10 mm below bench top loses the trial
HANDS_CLEAR = 0.030  # m: both grippers >= 30 mm clear of all task bodies for 1.0
REST_LIN = 0.010  # m/s: linear speed < 10 mm/s
REST_ANG = 0.10  # rad/s: angular speed < 0.10 rad/s
HOLD_STEPS = 20  # 1.0 s at 20 Hz


def summary() -> dict:
    counts = {k: sum(1 for _, (_, _, s, _) in FIGURES.items() if s == k) for k in (MAKER, ESTIMATE)}
    return {
        "source_counts": counts,
        "total_figures": len(FIGURES),
        "tool_mass_check_kg": round(STAND_BODY_MASS + CARRIER_MASS + BURNER_BODY_MASS + 3 * CONTROL_MASS, 4),
        "tin_mass_check_kg": round(TIN_BASE_MASS + TIN_LID_MASS, 4),
        "tin_inner_mm": [
            round(1000 * (TIN_L - 2 * TIN_WALL), 1),
            round(1000 * (TIN_W - 2 * TIN_WALL), 1),
            round(1000 * TIN_CLEAR_Z, 1),
        ],
        "attached_stand_interference_mm": round(1000 * (STAND_D - TIN_CLEAR_Z), 1),
        "seam_gap_at_2deg_mm": SEAM_GAP_MAX_MM,
        "figures": {n: {"value": v, "unit": u, "source": s, "note": note} for n, (v, u, s, note) in FIGURES.items()},
    }
