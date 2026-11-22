"""Every figure the CryoCane sleeving task rests on, in one place, each with its source; and derived geometry helpers.

Source kinds (``FIGURES`` holds every figure with its kind and its source in words; the README's table is made from it):
- ``maker``: the maker's published catalog page (Thermo Fisher Scientific for Nalgene 5015-0001, Nalgene 5016-0001 and
  Nunc 377267; Heathrow Scientific for PVC cane sleeve HS23210 and silicone lab mats).
- ``listing``: a lab supplier's catalog specification for the bought product.
- ``our fixture``: bench layout positions and trimmed mat strip specified by this task.
- ``estimate``: derived figure, with its one-clause derivation.

World frame (the station's): metres from the midpoint between the arm bases, x forward, y left, z up from the bench.
"""

from __future__ import annotations

import math

GRAVITY = 9.81
OVER = 0.0008  # m: loose bodies start this far above support so they settle cleanly on reset

MAKER, LISTING, FIXTURE, ESTIMATE = "maker", "listing", "our fixture", "estimate"

URL_CANE = "https://www.thermofisher.com/order/catalog/product/5015-0001"
URL_SLEEVE = "https://www.thermofisher.com/order/catalog/product/5016-0001"
URL_VIAL = "https://www.thermofisher.com/order/catalog/product/377267"
URL_HS_SLEEVE = "https://www.heathrowscientific.com/pvc-cryogenic-cane-sleeves/"
URL_HS_MAT = "https://www.heathrowscientific.com/silicone-lab-mats/"
URL_BOCHEM = "https://bochem.de/produkt/instrumentenablage-18-10-stahl/"

FIGURES: dict[str, tuple[float, str, str, str]] = {}


def figure(name: str, value: float, unit: str, source: str, note: str) -> float:
    if source not in (MAKER, LISTING, FIXTURE, ESTIMATE):
        raise ValueError(f"{name}: unknown source kind {source!r}")
    FIGURES[name] = (float(value), unit, source, note)
    return float(value)


# --- 1. Thermo Scientific Nalgene 5-Place Stepped Aluminum CryoCane (Cat. No. 5015-0001) -----------------------------
CANE_L = figure(
    "CryoCane overall length (Nalgene 5015-0001)",
    0.290,
    "m",
    MAKER,
    f"maker's page: '5015-0001 ... Aluminium ... Unterteilt ... 29 cm (11,31 in.)' ({URL_CANE})",
)
N_BAYS = int(
    figure(
        "CryoCane vial places (Nalgene 5015-0001)",
        5.0,
        "-",
        LISTING,
        "5-place stepped design for 5 cryogenic tubes (5015-0001 product specification)",
    )
)
CANE_MASS_REAL = figure(
    "CryoCane aluminum mass, real",
    0.016,
    "kg",
    ESTIMATE,
    "stamped 0.7 mm aluminum channel 290 mm long (~5.9 cm3 x 2,700 kg/m3)",
)
CANE_MASS = 0.035  # kg: PhysX solver mass declared in fidelity

SPINE_T = figure(
    "CryoCane stamped sheet thickness",
    0.0007,
    "m",
    ESTIMATE,
    "0.7 mm stamped aluminum sheet so 0.7 mm spine plus 12.5 mm vial is a 13.2 mm stack",
)
SPINE_W = 0.0080  # m: flat bottom spine width along x_c
FLOOR_Z = -0.0051  # m: top surface of spine floor in cane frame (+0.8 mm minimax centering inside sleeve bore)
BASE_Z = FLOOR_Z - SPINE_T  # -0.0058 m: underside of cane resting on mat or tray

RAIL_INNER_HALF_X = 0.00655  # m: 13.1 mm inner width across C-rail lips (0.6 mm play around 12.5 mm vial)
RAIL_H = figure(
    "CryoCane side C-rail lip height above floor",
    0.0028,
    "m",
    ESTIMATE,
    "shallow open C-channel lip h = 2.8 mm: retains 12.5 mm round vial level, spills past ~57 deg roll (acos((R-h)/R))",
)
RAIL_T = 0.0008  # m: side rail thickness
RAIL_W = 0.0036  # m: side rail slant width
RAIL_TILT_RAD = math.radians(38.0)

BAY_PITCH = figure(
    "CryoCane stepped bay pitch",
    0.0513,
    "m",
    ESTIMATE,
    "scaled from 5015-0001 photo against 290 mm length: 5 bays over 257 mm leaving 49.9 mm clear per 49.0 mm vial",
)
SHELF_T = 0.0014  # m: axial thickness of each stamped stepped shelf stop
SHELF_W = 0.0076  # m: transverse width of each stepped shelf stop
SHELF_H = figure(
    "CryoCane stepped shelf stop height",
    0.0022,
    "m",
    ESTIMATE,
    "stamped step height 2.2 mm: holds round-bottom vial when level, spills if pitched steeply end-down",
)
SHELF_YS = tuple(0.0040 + k * BAY_PITCH for k in range(N_BAYS + 1))  # 0.0040 .. 0.2605 m

TAB_SHOULDER_Y = figure(
    "CryoCane 90-deg bent top tab stop position from foot",
    0.268,
    "m",
    ESTIMATE,
    "scaled from 5015-0001 photo against 290 mm length: 268 mm shaft plus 22 mm bent ID handling tab = 290 mm",
)
TAB_W = figure(
    "CryoCane bent top ID tab width",
    0.026,
    "m",
    ESTIMATE,
    "26 mm wide upturned end tab (> 21.6 mm sleeve bore and 23.6 mm outer tube) so wrong-end sleeve entry stops at 0 mm",
)
TAB_L = CANE_L - TAB_SHOULDER_Y  # 0.022 m: axial length of top handling tab (268..290 mm)
TAB_TOP_Z = 0.0182  # m: top of upturned ID handling tab (+24.0 mm above cane underside BASE_Z)

# --- 2. Thermo Scientific Nunc 1.8 mL CryoTube Vials (Cat. No. 377267) ----------------------------------------------
VIAL_VOL_ML = figure(
    "Cryovial nominal volume (Nunc 377267)",
    1.8,
    "mL",
    MAKER,
    f"maker's page: '377267 ... 1.8 ml ... Polypropylen ... Rund ... intern' ({URL_VIAL})",
)
VIAL_OD = figure(
    "Cryovial outer diameter (Nunc 377267)",
    0.0125,
    "m",
    MAKER,
    f"maker's page: '12.5 mm (0.492 in.)' outer diameter ({URL_VIAL}); sits loose in 5015-0001 trough until sleeved",
)
VIAL_R = VIAL_OD / 2.0  # 0.00625 m
VIAL_L = figure(
    "Cryovial overall length (Nunc 377267)",
    0.049,
    "m",
    MAKER,
    f"maker's page: '49 mm (1.929 Zoll)' height ({URL_VIAL})",
)
VIAL_STRAIGHT_L = VIAL_L - 2.0 * VIAL_R  # 0.0365 m
VIAL_MASS_REAL = figure(
    "Loaded cryovial mass, real (1.5 mL dyed water fill)",
    0.0035,
    "kg",
    ESTIMATE,
    "2.0 g polypropylene 1.8 mL tube and cap plus 1.5 g dyed water aliquot",
)
VIAL_MASS = 0.012  # kg: PhysX solver mass declared in fidelity
VIAL_REST_Z = FLOOR_Z + VIAL_R  # +0.00115 m in cane frame: top at +7.40 mm (total spine+vial stack = 13.20 mm)
VIAL_BAYS = (2, 1)  # vial_a in bay index 2 (3rd from foot), vial_b in bay index 1 (2nd from foot)


def bay_centre_y(bay_idx: int) -> float:
    return 0.5 * (SHELF_YS[bay_idx] + SHELF_YS[bay_idx + 1])


# --- 3. Thermo Scientific Nalgene CryoSleeve (Cat. No. 5016-0001; cf. Heathrow HS23210) ------------------------------
SLEEVE_L = figure(
    "CryoSleeve length (Nalgene 5016-0001)",
    0.273,
    "m",
    MAKER,
    f"maker's page: '5016-0001 ... Nalgene CryoSleeve, 273 mm, PVC' ({URL_SLEEVE})",
)
SLEEVE_CATALOG_ID = figure(
    "Companion PVC cane sleeve diameter (Heathrow HS23210)",
    0.0145,
    "m",
    MAKER,
    f"maker's page: 'HS23210 ... Dimensions: 14.5 mm diameter x 275 mm length' ({URL_HS_SLEEVE})",
)
SLEEVE_STAVES = 8
SLEEVE_IN_FLAT = figure(
    "Simulated 8-stave sleeve bore, flat-to-flat",
    0.0216,
    "m",
    ESTIMATE,
    "widened from ~15 mm thin-wall PVC to 21.6 mm flat-to-flat (23.4 mm corner-to-corner) so 8 rigid staves clear the 13.2 mm stack over 265 mm without pose snaps",
)
SLEEVE_WALL = 0.0010  # m: stave thickness
SLEEVE_R_IN = SLEEVE_IN_FLAT / 2.0  # 0.0102 m apothem to inner flat (>= 2.6 mm radial clearance over cane & vials)
SLEEVE_R_OUT = SLEEVE_R_IN + SLEEVE_WALL  # 0.0112 m apothem to outer flat (22.4 mm outer flat-to-flat)
SLEEVE_MASS_REAL = figure(
    "CryoSleeve PVC mass, real",
    0.018,
    "kg",
    ESTIMATE,
    "273 mm extruded clear PVC sleeve (~0.45 mm wall, 1,380 kg/m3)",
)
SLEEVE_MASS = 0.035  # kg: PhysX solver mass declared in fidelity
CONTACT_OFF = 0.0008  # m: collider contact offset on cane, vials and sleeve (rest offset 0)

# --- 4. Bench fixtures: silicone prep mat & 18/10 stainless transfer tray --------------------------------------------
MAT_CX = figure("Prep mat centre x", 0.210, "m", FIXTURE, "21.0 cm forward of the arm bases, in both arms' 1 mm reach zone")
MAT_X = figure("Prep mat depth along x", 0.130, "m", FIXTURE, "130 mm strip cut from 350 x 600 mm silicone lab mat")
MAT_Y = figure(
    "Prep mat width along y (Heathrow silicone lab mat)",
    0.600,
    "m",
    MAKER,
    f"maker's page: '14 x 23.5 x 0.1 in (35 x 60 x 0.2 cm)' ({URL_HS_MAT})",
)
MAT_T = figure(
    "Prep mat thickness (Heathrow silicone lab mat)",
    0.002,
    "m",
    MAKER,
    f"maker's page: '0.2 cm' FDA food-grade silicone ({URL_HS_MAT})",
)

TRAY_CX, TRAY_CY = (
    figure("Stainless transfer tray centre x", 0.365, "m", FIXTURE, "15 mm behind the prep mat, within left arm placement reach"),
    figure("Stainless transfer tray centre y", 0.035, "m", FIXTURE, "centred 35 mm left of midline so left-hand release has > 9 deg wrist margin"),
)
TRAY_X = figure(
    "Stainless transfer tray width along x",
    0.150,
    "m",
    FIXTURE,
    f"our 350 x 150 x 35 mm open 18/10 stainless transfer tray (any open lab tray >= 320 x 140 mm; cf. Bochem 18/10-Stahl series {URL_BOCHEM})",
)
TRAY_Y = figure(
    "Stainless transfer tray length along y",
    0.350,
    "m",
    FIXTURE,
    "350 mm tray length (>= 320 mm) gives 52 mm axial clearance for the 298 mm sleeved cane assembly",
)
TRAY_H = figure("Stainless transfer tray wall height", 0.035, "m", FIXTURE, "35 mm rim height encloses the 31 mm tall sleeved cane")
TRAY_WALL = 0.0030  # m: tray floor and rim wall thickness in PhysX
TRAY_MASS = figure(
    "Stainless transfer tray mass",
    0.420,
    "kg",
    ESTIMATE,
    "0.6 mm 18/10 stainless steel sheet over 350 x 150 x 35 mm (~0.088 m2 x 7,900 kg/m3)",
)

# --- 5. Start poses on prep_mat (cane wrong-end toward sleeve: bent top tab at -0.012, foot at +0.278) ---------------
CANE_START_FOOT_Y = figure(
    "Start: CryoCane straight foot y",
    0.278,
    "m",
    FIXTURE,
    "cane lies along y with its 25 mm bent top tab at y = -12 mm facing the sleeve across a 13 mm gap",
)
CANE_START_YAW = math.pi  # +y_c (foot -> top tab) points along world -y at start
SLEEVE_START_CY = figure(
    "Start: CryoSleeve centre y",
    -0.1615,
    "m",
    FIXTURE,
    "273 mm sleeve spans y = -298.0 to -25.0 mm on the prep mat, mouth 13 mm from the cane's bent top tab",
)

# --- 6. Zero-snap coaxial slide (engages right where sleeve stands at s_mouth = 10..60 mm, <= 2.0 deg) & wipe / bind -
CARRIAGE_BUILT_S = 0.015  # m: built station of c_slv mouth along cane (limits -15..+251 mm -> mouth 0..266 mm)
ENTRY_S_MIN, ENTRY_S_MAX = 0.010, 0.060  # m: sleeve mouth telescoped 10..60 mm over straight foot in pure PhysX
ENTRY_RADIAL_MAX = 0.0028  # m: mouth radial offset <= 2.8 mm at engagement
ENTRY_CENTRE_RADIAL_MAX = 0.0060  # m: sleeve-centre radial offset <= 6.0 mm (accommodates 2.0 deg over 136.5 mm)
ENTRY_ANGLE_DEG = 2.0  # deg: coaxial within 2.0 deg when w_sleeve engages in-place (zero pose snap!)
GEOM_RADIAL_MAX = 0.0060  # m: geometric coaxial enclosure radial limit alongside sleeve_mated()
GEOM_ANGLE_DEG = 5.0  # deg: geometric coaxial enclosure angle limit alongside sleeve_mated()
GEOM_SLIDE_MIN, GEOM_SLIDE_MAX = 0.030, SLEEVE_L  # m: leading sleeve mouth 30..273 mm along cane from straight foot
SLIDE_MAX = figure(
    "CryoSleeve coaxial stroke to top tab stop",
    0.265,
    "m",
    ESTIMATE,
    "268 mm shaft to bent tab minus 3 mm clearance = 265 mm stroke (encloses foot 8 mm inside 273 mm sleeve)",
)
SLIDE_N = figure(
    "CryoSleeve coaxial sliding resistance",
    5.0,
    "N",
    ESTIMATE,
    "sliding friction of PVC sleeve wiping over aluminum cane and 1.8 mL tubes",
)
BIND_TORQUE_NM = figure(
    "Off-axis cocking moment threshold for tube binding",
    0.35,
    "N m",
    ESTIMATE,
    "transverse bending moment on 19:1 sleeve (~3.5 deg off-axis push) that wedges tube on cane",
)
BIND_N = figure(
    "Cocked sleeve binding resistance",
    35.0,
    "N",
    ESTIMATE,
    "7x wedge friction when 273 mm tube is pushed off-axis (> 0.35 N m bending moment)",
)
SLIDE_K = 15000.0  # N/m: stiff rest-following spring (0.33 mm give at 5 N) on PrismaticJoint slv_s
SLIDE_DAMP = 45.0  # N s/m

# --- 7. Scoring thresholds (identical in task.py, task.json and README.md) -------------------------------------------
SPILL_RADIAL_MAX = 0.014  # m: vial centre > 14 mm off cane bore axis -> spilled off cane (zero rule)
SPILL_AXIAL_MAX = 0.015  # m: vial centre > 15 mm off its bay centre -> left its bay (zero rule)
DROP_BELOW_BENCH = 0.030  # m: any body < TOP - 30 mm -> dropped off bench (zero rule)

PARTIAL_SLIDE_MIN = 0.120  # m: 0.25 score level: sleeve threaded >= 120 mm from straight foot
SEATED_TAB_GAP_MAX = 0.010  # m: 0.5 / 1.0 level: gap from sleeve leading rim to bent tab shoulder <= 10.0 mm (slide >= 258 mm)
SEATED_SLIDE_MIN = TAB_SHOULDER_Y - SEATED_TAB_GAP_MAX  # 0.258 m
TRAY_TILT_MAX_DEG = 8.0  # deg: sleeved cane lying flat on tray floor within 8 deg of horizontal
HANDS_CLEAR = 0.030  # m: both hands' hulls >= 30 mm from cane, sleeve, vials and tray
HOLD_DRIFT = 0.003  # m: cane and sleeve move at most 3 mm during the 1.0 s hold
HOLD_STEPS = 20  # 1.0 s at 20 Hz


def source_counts() -> dict[str, int]:
    out = {MAKER: 0, LISTING: 0, FIXTURE: 0, ESTIMATE: 0}
    for _, _, kind, _ in FIGURES.values():
        out[kind] += 1
    return out


def summary() -> dict:
    stack_mm = 1000.0 * ((VIAL_REST_Z + VIAL_R) - BASE_Z)
    roll_spill_deg = math.degrees(math.acos((VIAL_R - RAIL_H) / VIAL_R))
    pitch_spill_deg = math.degrees(math.acos((VIAL_R - SHELF_H) / VIAL_R))
    return {
        "source counts": source_counts(),
        "figures": {n: {"value": v, "unit": u, "source": s, "note": t} for n, (v, u, s, t) in FIGURES.items()},
        "spine + vial stack (mm)": round(stack_mm, 2),
        "sleeve bore flat-to-flat / corner-to-corner (mm)": [
            round(1000.0 * SLEEVE_IN_FLAT, 2),
            round(1000.0 * SLEEVE_IN_FLAT / math.cos(math.pi / 8.0), 2),
        ],
        "top tab width vs sleeve outer flat-to-flat (mm)": [round(1000.0 * TAB_W, 1), round(2000.0 * SLEEVE_R_OUT, 1)],
        "theoretical capsule roll / pitch spill angles (deg)": [round(roll_spill_deg, 1), round(pitch_spill_deg, 1)],
        "at 265 mm stroke: gap to tab (mm), foot inside rear rim (mm)": [
            round(1000.0 * (TAB_SHOULDER_Y - SLIDE_MAX), 1),
            round(1000.0 * (SLEEVE_L - SLIDE_MAX), 1),
        ],
    }
