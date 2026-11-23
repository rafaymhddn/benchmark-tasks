"""Every figure the bench-can task rests on, each with its source, and the geometry derived from them.

Source kinds (``FIGURES`` holds every figure with its kind and its source in words; the README's table is built from it):
- ``maker``: Eagle Manufacturing's (Justrite) and Bochem's official product pages opened on 2026-11-21, plus SKF's entry.
- ``standard``: ISO 15 rolling-bearing boundary dimensions for dimension series 33, bore code 05 (``3305``).
- ``listing``: Agrolager's technical product sheet for SKF ``3305-A-2Z`` opened on 2026-11-21.
- ``our fixture``: cut black nitrile-rubber starting pad and bench alignment marks specified by this task.
- ``estimate``: derived figure with a one-clause physical or photo-scaling derivation.

World frame (the station's): metres from the midpoint between the arm bases, x forward, y left, z up from the bench.
"""

from __future__ import annotations

import math

import numpy as np

GRAVITY = 9.81
MAKER, STANDARD, LISTING, FIXTURE, ESTIMATE = "maker", "standard", "listing", "our fixture", "estimate"

EAGLE_B601_PAGE = "https://eagle.justrite.com/steel-bench-can-1-quart-dasher-screen-red-b601"
EAGLE_PHOTO_URL = "https://industrialsafety.com/media/catalog/product/cache/99dc5e1dc90eb9727970ba1a904a05a5/E/a/Eagle-MFG-B-601-1.jpg"
SKF_3305_PAGE = "https://www.skf.com/group/products/rolling-bearings/ball-bearings/angular-contact-ball-bearings/productid-3305%20A-2Z"
AGROLAGER_3305_PAGE = "https://www.agrolager.de/product_info.php?products_id=12062210"
BOCHEM_8480_PAGE = "https://bochem.de/produkt/instrumentenablage-18-10-stahl-200x150x15mm/"

FIGURES: dict[str, tuple[float, str, str, str]] = {}


def figure(name: str, value: float, unit: str, source: str, note: str) -> float:
    if source not in (MAKER, STANDARD, LISTING, FIXTURE, ESTIMATE):
        raise ValueError(f"{name}: unknown source kind {source!r}")
    FIGURES[name] = (float(value), unit, source, note)
    return float(value)


# --- Eagle Model B-601 (B601) 1-Quart Galvanized Steel Safety Bench Can, Red ---------------------------------------------
CAN_CAPACITY_L = figure("Eagle B-601 bench-can capacity", 0.95, "L", MAKER,
                        f"'1 quart capacity; Liter Capacity: 0.95; FM Approval: Yes; OSHA Compliance: Yes' ({EAGLE_B601_PAGE})")
CAN_OD = figure("Eagle B-601 exterior diameter (6.25 in)", 6.25 * 0.0254, "m", MAKER,
                f"'Dimensions, Exterior: 6.25D x 3.625H' (in) for Model B601, UPC 48441223213 ({EAGLE_B601_PAGE})")
CAN_R = 0.5 * CAN_OD  # 0.079375 m
CAN_H = figure("Eagle B-601 closed height (3.625 in)", 3.625 * 0.0254, "m", MAKER,
               f"'Dimensions, Exterior: 6.25D x 3.625H' (in), '24-Gauge Galvanized Steel ... red baked on powder coat finish' ({EAGLE_B601_PAGE})")
B601_TOTAL_MASS = figure("Eagle B-601 net empty mass (2.17 lb)", 0.980, "kg", MAKER,
                         f"'Net Weight, lbs: 2.17; Net Weight, kgs: 0.98' ({EAGLE_B601_PAGE})")

CAN_BODY_MASS = figure("can body + front handle mass share", 0.680, "kg", ESTIMATE,
                       "69 % of Eagle's 0.98 kg net mass allocated by 24-gauge sheet-steel area to the body, collar and welded strap")
LID_MASS = figure("hinged saucer lid mass share", 0.180, "kg", ESTIMATE,
                  "18 % of Eagle's 0.98 kg net mass allocated by sheet-steel area to the Ø 162 mm saucer cover and hinge leaf")
DASH_MASS = figure("perforated brass dasher + spring mass share", 0.120, "kg", ESTIMATE,
                   f"remaining 0.12 kg of Eagle's 0.98 kg net mass for 'brass flame arrestor dasher screen and pump assemblies' ({EAGLE_B601_PAGE})")

COLLAR_ID = figure("top retaining collar inner bore diameter", 0.118, "m", ESTIMATE,
                   f"scaled at 74 % of 158.8 mm OD from Eagle's B-601 product photo ({EAGLE_PHOTO_URL})")
COLLAR_IN_R = 0.5 * COLLAR_ID  # 0.059 m
DASHER_D = figure("perforated brass dasher disc diameter", 0.114, "m", ESTIMATE,
                  "4 mm diametral clearance inside the 118 mm top retaining collar so the guided dasher slides without wall rub")
DASHER_R = 0.5 * DASHER_D  # 0.057 m
DASHER_T = 0.006  # m: rigid brass screen plate + guide rim thickness
DASH_RECESS = figure("dasher rest depth below can rim", 0.014, "m", ESTIMATE,
                     f"scaled from Eagle's B-601 photo ({EAGLE_PHOTO_URL}): screen sits 14 mm below the 92.1 mm collar rim")
DASH_REST_Z = CAN_H - DASH_RECESS  # 0.078075 m: top surface of dasher at 0 mm travel
DASH_STROKE = figure("dasher vertical pump stroke", 0.016, "m", ESTIMATE,
                     "16.0 mm downward travel inside the upper chamber (78.1 -> 62.1 mm above base)")
DASH_PRELOAD_N = figure("dasher spring upward preload at upper stop", 2.20, "N", ESTIMATE,
                        "holds the 0.12 kg brass screen against the upper collar when empty and yields 1.1 mm under a 0.35 kg bearing")
DASH_K = figure("dasher coil-spring stiffness", 1125.0, "N/m", ESTIMATE,
                "rises from 2.2 N preload to 18.0 N at 14 mm plunge (20.2 N at 16 mm stop), well inside the 40 N finger limit")
DASH_PRELOAD_M = DASH_PRELOAD_N / DASH_K  # 0.001956 m above upper stop
DASH_DAMPING = 8.0  # N s/m

LID_OD = 0.165  # m: saucer cover overlapping the 158.8 mm rim by ~3 mm per side
LID_R = 0.5 * LID_OD  # 0.0825 m
LID_ROOF_T = 0.003  # m: steel roof thickness (z = CAN_H .. CAN_H + 0.003 when closed)
LID_SKIRT_H = 0.011  # m: rolled rim skirt depth
LID_OPEN_DEG = figure("open saucer cover resting angle", 172.0, "deg", ESTIMATE,
                      f"from Eagle's B-601 photo ({EAGLE_PHOTO_URL}): hinged cover hangs out ~8 deg above horizontal (~40 mm off bench)")
HINGE_R = 0.0805  # m from can axis to lid hinge pin
HINGE_Z = CAN_H - 0.002  # 0.0901 m: hinge pin height on rim
LID_FRICTION_NM = figure("lid hinge dry friction torque", 0.22, "N m", ESTIMATE,
                         "dry steel hinge pin (~2.8 N at 80 mm radius) exceeding the 0.98 kg dry can's 2.4 N bench slip force so unbraced pulls slide the can")
CAN_BENCH_MU = figure("dry galvanized steel can bottom on smooth bench friction", 0.25, "-", ESTIMATE,
                      "dry rolled-steel base seam on smooth coated workbench (0.20-0.30 -> ~2.4 N slip threshold on 0.98 kg)")

# Welded horizontal front strap D-handle (visible on Eagle-MFG-B-601-1.jpg)
HANDLE_SPAN = figure("front welded strap handle width", 0.056, "m", ESTIMATE,
                     f"scaled at 35 % of 158.8 mm OD from Eagle's B-601 photo ({EAGLE_PHOTO_URL})")
HANDLE_OUT = figure("front welded strap radial standout", 0.022, "m", ESTIMATE,
                    f"scaled from Eagle's B-601 photo ({EAGLE_PHOTO_URL}): projects 22 mm radially off the cylinder wall")
HANDLE_H = 0.014  # m: vertical strap height (pinched vertically by Right gripper)
HANDLE_Z = 0.062  # m: strap centre height above can base (~30 mm below rim, as in B-601 photo)

# --- Two identical SKF 3305 A-2Z double-row angular contact ball bearings ------------------------------------------------
BRG_ID = figure("SKF 3305 A-2Z bore diameter d", 0.025, "m", STANDARD,
                f"ISO 15 dimension series 33 bore code 05: d = 25 mm ({SKF_3305_PAGE}; listing '25x62x25,4', {AGROLAGER_3305_PAGE})")
BRG_OD = figure("SKF 3305 A-2Z outside diameter D", 0.062, "m", STANDARD,
                f"ISO 15 series 33: D = 62 mm (<= 94 mm jaw; {SKF_3305_PAGE}; {AGROLAGER_3305_PAGE})")
BRG_W = figure("SKF 3305 A-2Z width B", 0.0254, "m", STANDARD,
               f"ISO 15 series 33: B = 25.4 mm (stands 9.2 mm proud of 92.1 mm can rim on dasher; {SKF_3305_PAGE}; {AGROLAGER_3305_PAGE})")
BRG_MASS = figure("SKF 3305 A-2Z bearing mass", 0.350, "kg", LISTING,
                  f"'Gewicht: 0,35 kg' for SKF 3305-A-2Z/C3MT33 ({AGROLAGER_3305_PAGE}; SKF 3305 A-2Z series 0.34-0.35 kg)")
BRG_R = 0.5 * BRG_OD  # 0.031 m

# --- Bochem Art. 8480 18/10 Stainless Steel Instrument Tray (200 x 175 x 15 mm) & NBR Mat -------------------------------
TRAY_L = figure("Bochem 8480 stainless tray length along x", 0.200, "m", MAKER,
                f"'Instrumentenablage 18/10 Stahl, Artikelnr. 8480: Breite 200' mm ({BOCHEM_8480_PAGE})")
TRAY_W = figure("Bochem 8480 stainless tray width along y", 0.175, "m", MAKER,
                f"'Tiefe 175' mm ({BOCHEM_8480_PAGE})")
TRAY_H = figure("Bochem 8480 stainless tray rim height", 0.015, "m", MAKER,
                f"'Höhe 15' mm ({BOCHEM_8480_PAGE})")
TRAY_WALL = 0.003  # m: collider wall thickness
TRAY_FLOOR = 0.003  # m: floor thickness
TRAY_MASS = figure("Bochem 8480 stainless tray mass", 0.240, "kg", ESTIMATE,
                   "0.8 mm 18/10 stainless sheet (7,900 kg/m3) over a 200 x 175 mm base and 15 mm perimeter rim")
TRAY_MU = 0.55  # tray on bench friction

MAT_SIDE = figure("black nitrile starting pad side", 0.120, "m", FIXTURE,
                  "120 x 120 x 3 mm black NBR rubber bench pad taped flat beside the can")
MAT_T = 0.003

# --- Bench layout & clocking (wrists >= 290 mm and TCPs >= 180 mm apart throughout lid closure; all within +-0.35 m) ----
CAN_XY = (0.265, 0.000)  # m: on workspace centreline y = 0 so both arms reach comfortably with >= 14 deg joint margins
PHI_HINGE_DEG = 125.0  # deg from +x: lid opens toward (+y, -x) left-front, inside y <= +0.21 m
HANDLE_CLOCK_DEG = 90.0  # deg CCW from hinge: places front D-handle at 215 deg (-x, -y) facing Right arm
PHI_HANDLE_DEG = PHI_HINGE_DEG + HANDLE_CLOCK_DEG  # 215.0 deg

MAT_XY = (0.165, -0.205)  # m: right-front, y in [-0.265, -0.145] m
TRAY_XY = (0.355, -0.215)  # m: right-mid, x in [0.255, 0.455] m, y in [-0.3025, -0.1275] m (inside +-0.35 m!)
TRAY_SLOT_BACK_XY = (0.402, -0.215)  # m: target in tray for brg_screen
TRAY_SLOT_FRONT_XY = (0.308, -0.215)  # m: target in tray for brg_mat (94 mm centre-to-centre -> 32 mm gap between 62 mm ODs)


def hinge_unit_vectors() -> tuple[np.ndarray, np.ndarray]:
    """(outward unit vector ``u_h`` from can centre to hinge, horizontal hinge-axis unit vector ``t_h``)."""
    ph = math.radians(PHI_HINGE_DEG)
    u_h = np.array([math.cos(ph), math.sin(ph), 0.0], dtype=float)
    t_h = np.array([-math.sin(ph), math.cos(ph), 0.0], dtype=float)  # right-hand turn +theta lifts -u_h (+z) then out (+u_h)
    return u_h, t_h


def handle_unit_vector() -> np.ndarray:
    pd = math.radians(PHI_HANDLE_DEG)
    return np.array([math.cos(pd), math.sin(pd), 0.0], dtype=float)


# --- Scoring thresholds (identical in task.py, task.json, and README.md) -------------------------------------------------
CAN_TIP_ZERO_DEG = 12.0  # whole-trial 0: can tilted > 12 deg
CAN_SLIDE_ZERO_M = 0.030  # whole-trial 0: can slid > 30 mm from start mark
TRAY_PUSH_ZERO_M = 0.025  # whole-trial 0: tray pushed > 25 mm from start mark
TRAY_TIP_ZERO_DEG = 10.0  # whole-trial 0: tray tipped > 10 deg
DIP_DEPTH_M = 0.012  # >= 12.0 mm downward dasher travel (of 16.0 mm stroke)
DIP_HOLD_STEPS = 16  # >= 0.8 s continuous at 20 Hz with brg_mat pressing dasher
DIP_CONTACT_N = 2.5  # N normal contact confirming brg_mat itself presses dasher (not an empty finger)
SCREEN_UP_M = 0.002  # <= 2.0 mm dasher travel when sprung back up
LID_CLOSED_DEG = 4.0  # <= 4.0 deg from flat closed stop (skirt-to-rim gap <= 3.5 mm)
HANDS_CLEAR_M = 0.030  # >= 30 mm from both hands to can, lid, tray, and both bearings
HOLD_STEPS = 20  # 1.0 s at 20 Hz at rest
HOLD_DRIFT_M = 0.003  # <= 3 mm drift during hold
REST_LIN_MPS = 0.010  # < 10 mm/s
REST_ANG_RPS = 0.10  # < 0.10 rad/s


def summary() -> dict:
    by_kind: dict[str, int] = {}
    for _, _, s, _ in FIGURES.values():
        by_kind[s] = by_kind.get(s, 0) + 1
    sag_m = max(0.0, (BRG_MASS * GRAVITY - DASH_PRELOAD_N) / DASH_K)
    proud_mm = 1000.0 * ((DASH_REST_Z - sag_m + BRG_W) - CAN_H)
    return {
        "counts_by_source": by_kind,
        "figures": {n: {"value": v, "unit": u, "source": s, "note": t} for n, (v, u, s, t) in FIGURES.items()},
        "dasher_force_12mm_14mm_16mm_N": [round(DASH_PRELOAD_N + DASH_K * d, 2) for d in (0.012, 0.014, 0.016)],
        "bearing_resting_sag_mm": round(1000.0 * sag_m, 2),
        "bearing_top_proud_of_rim_mm": round(proud_mm, 2),
        "can_slip_force_N": round(B601_TOTAL_MASS * GRAVITY * CAN_BENCH_MU, 2),
    }
