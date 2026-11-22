"""Every figure ``baby_scale_convert`` rests on, each with its verified source, and the geometry derived from them.

Source kinds in ``FIGURES``:
- ``maker``: opened manufacturer page, product sheet PDF, or user manual PDF (seca.com, ikea.com).
- ``standard``: O*NET occupation summary (onetonline.org, opened).
- ``our fixture``: bench layout and start marks specified by this task.
- ``estimate``: derived figure with a one-clause derivation (the key ones get a First-minutes check in a real trial).

World frame (station): metres from the midpoint between the arm bases, +x forward (away from arms), +y left, +z up
from ``TABLE_TOP``.
"""

from __future__ import annotations

import numpy as np

MAKER, STANDARD, FIXTURE, ESTIMATE = "maker", "standard", "our fixture", "estimate"
SECA_PAGE = "https://www.seca.com/en_us/products/all-products/product-details/seca354.html"
SECA_SHEET = "https://www.seca.com/fileadmin/documents/product_sheet/seca_pst_354_en-us.pdf"
SECA_MANUAL = "https://www.seca.com/fileadmin/documents/manual/seca_man_354_364_int.pdf"
IKEA_PAGE = "https://www.ikea.com/gb/en/p/tillgang-tray-grey-10419950/"
ONET_MA = "https://www.onetonline.org/link/summary/31-9092.00"
ONET_RN = "https://www.onetonline.org/link/summary/29-1141.00"
ONET_LPN = "https://www.onetonline.org/link/summary/29-2061.00"

FIGURES: dict[str, tuple[float, str, str, str]] = {}


def figure(name: str, value: float, unit: str, source: str, note: str) -> float:
    if source not in (MAKER, STANDARD, FIXTURE, ESTIMATE):
        raise ValueError(f"{name}: unknown source kind {source!r}")
    FIGURES[name] = (float(value), unit, source, note)
    return float(value)


# --- seca 354 Digital Baby & Floor Scale (maker page, product sheet PDF, manual PDF) ----------------------------------
ASSEMBLED_W = figure("seca 354 assembled width", 0.552, "m", MAKER,
                     f"maker product sheet 'Dimensions (WxHxD): 21.7 x 6.1 x 13.1\" / 552 x 156 x 332 mm' ({SECA_SHEET}; {SECA_PAGE})")
ASSEMBLED_H = figure("seca 354 assembled height", 0.156, "m", MAKER,
                     f"maker product sheet '552 x 156 x 332 mm' ({SECA_SHEET})")
ASSEMBLED_D = figure("seca 354 assembled depth (front of platform to back of tray)", 0.332, "m", MAKER,
                     f"maker product sheet '552 x 156 x 332 mm' ({SECA_SHEET}): base protrudes 82 mm ahead of the 250 mm tray, exposing LCD and release button")
TRAY_W = figure("seca 354 weighing tray width (open-ended scoop along y)", 0.525, "m", MAKER,
                f"maker product sheet 'Dimensions/tray (WxHxD): 20.7 x 3.1 x 9.8\" / 525 x 80 x 250 mm' ({SECA_SHEET})")
TRAY_H = figure("seca 354 weighing tray scoop height", 0.080, "m", MAKER,
                f"maker product sheet '525 x 80 x 250 mm' ({SECA_SHEET})")
TRAY_D = figure("seca 354 weighing tray depth (along x)", 0.250, "m", MAKER,
                f"maker product sheet '525 x 80 x 250 mm' ({SECA_SHEET})")
BASE_W = figure("seca 354 floor-scale platform width (along y)", 0.280, "m", MAKER,
                f"maker product sheet 'Dimensions/platform (WxHxD): 11 x 2.6 x 9.3\" / 280 x 65 x 235 mm' ({SECA_SHEET})")
BASE_H = figure("seca 354 floor-scale platform height", 0.065, "m", MAKER,
                f"maker product sheet '280 x 65 x 235 mm' ({SECA_SHEET})")
BASE_D = figure("seca 354 floor-scale platform depth (along x)", 0.235, "m", MAKER,
                f"maker product sheet '280 x 65 x 235 mm' ({SECA_SHEET})")
SCALE_MASS = figure("seca 354 total net mass", 2.30, "kg", MAKER,
                    f"maker product sheet & page 'Weight: 5 lbs / 2.3 kg' ({SECA_SHEET}; manual {SECA_MANUAL} lists 'Total weight: approx. 2.5 kg' with 4 AA cells; 2.30 kg sheet figure used)")
SCALE_CAP_KG = figure("seca 354 weighing capacity", 20.0, "kg", MAKER,
                      f"maker product sheet 'Capacity: 44 lbs / 20 kg' ({SECA_SHEET}) and manual ({SECA_MANUAL})")

# --- IKEA TILLGANG grey tray / mat, Art. 104.199.50 (maker page) ------------------------------------------------------
MAT_W = figure("IKEA TILLGANG mat length (along y)", 0.370, "m", MAKER,
               f"maker page 'Length: 37 cm', 'Polypropylene plastic', 'Grey', Art. 104.199.50 ({IKEA_PAGE})")
MAT_D = figure("IKEA TILLGANG mat width (along x)", 0.290, "m", MAKER,
               f"maker page 'Width: 29 cm' ({IKEA_PAGE})")
MAT_H = figure("IKEA TILLGANG mat rim height", 0.020, "m", MAKER,
               f"maker page 'Height: 2 cm' ({IKEA_PAGE})")
MAT_MASS = figure("IKEA TILLGANG mat mass", 0.300, "kg", MAKER,
                  f"maker page 'Weight: 0.30 kg' ({IKEA_PAGE})")

# --- O*NET occupation counts (standard) -------------------------------------------------------------------------------
MA_WORKERS = figure("US medical assistants employment (O*NET 31-9092.00)", 833900.0, "workers", STANDARD,
                    f"O*NET 31-9092.00 'Interview patients to obtain medical information and measure their vital signs, weight, and height.', '833,900 employees' ({ONET_MA})")
RN_WORKERS = figure("US registered nurses employment (O*NET 29-1141.00)", 3465400.0, "workers", STANDARD,
                    f"O*NET 29-1141.00 'Record patients\\' medical information and vital signs.', '3,465,400 employees' ({ONET_RN})")
LPN_WORKERS = figure("US licensed practical and vocational nurses employment (O*NET 29-2061.00)", 666900.0, "workers", STANDARD,
                     f"O*NET 29-2061.00 'Measure and record patients\\' vital signs, such as height, weight, temperature, blood pressure, pulse, or respiration.', '666,900 employees' ({ONET_LPN})")

# --- Derived component masses & interlock mechanics (estimates with derivations) --------------------------------------
TRAY_MASS = figure("weighing tray mass", 0.80, "kg", ESTIMATE,
                   "estimate: moulded ABS scoop of 525 x 250 x 80 mm plus underside runners and latch studs is 0.80 kg of the maker's 2.30 kg total")
BTN_MASS = figure("release button + pawl mass", 0.03, "kg", ESTIMATE,
                  "estimate: moulded ABS round button cap and its pawl link, 30 g of the 2.30 kg total")
BASE_MASS = figure("floor-scale platform mass (without tray and button)", 1.47, "kg", ESTIMATE,
                   "estimate: remainder 2.30 - 0.80 - 0.03 = 1.47 kg (load cells, steel sub-plate, ABS housing)")
BTN_SPRING = figure("release button spring stiffness", 2400.0, "N/m", ESTIMATE,
                    f"estimate: 12.0 N at 5.0 mm depression to match the manual's 'press the release button hard and slide back the tray (Fig. 2)' ({SECA_MANUAL})")
BTN_STROKE = figure("release button full downward travel", 0.0065, "m", ESTIMATE,
                    "estimate: 6.5 mm vertical plunger stroke on the front apron")
BTN_UNLOCK = figure("release button depression that clears the front-step pawl", 0.0043, "m", ESTIMATE,
                    "estimate: 4.3 mm depression (~10.3 N against the 2400 N/m spring) drops the pawl below the tray's latch tongue")
HOOK_SLIDE = figure("backward (+x) tray slide that frees both keyhole studs", 0.020, "m", ESTIMATE,
                    f"estimate: 16 mm of stud head under the keyhole lips plus 4 mm clearance = 20 mm slide back per Fig. 2 ({SECA_MANUAL})")
FLOOR_T = figure("scoop floor and wing thickness in the model", 0.006, "m", ESTIMATE,
                 "estimate: the moulded ABS shell is about 3.5 mm (from the 0.80 kg tray figure); 6 mm is a stand-in plate for the solver")
EDGE_D = figure("rolled edge at each open wing end, diameter in the model", 0.016, "m", ESTIMATE,
                "estimate: the rolled end edge reads about 8-10 mm thick in the product sheet photo (scaled by the 525 mm tray); the model's 16 mm rod lets the tilted jaws close on it clear of the floor")

# --- Layout on the station bench (our fixture: symmetric on y = 0, true 332 mm assembled depth) -----------------------
BASE_FRONT_X = figure("platform front face x ahead of arm-base line", 0.133, "m", FIXTURE,
                      "places the release button at x = 17.2 cm and the assembled back of the tray at 46.5 cm")
APRON_EXPOSED = ASSEMBLED_D - TRAY_D  # 0.082 m: 82 mm exposed front apron ahead of the 250 mm tray
BASE_X = BASE_FRONT_X + BASE_D / 2  # 0.2505 m
BASE_Y = 0.000
TRAY_FRONT_X = BASE_FRONT_X + APRON_EXPOSED  # 0.215 m
TRAY_X = TRAY_FRONT_X + TRAY_D / 2  # 0.3400 m (back at 0.465 m -> 0.465 - 0.133 = 0.332 m!)
TRAY_Y = 0.000
SKID_H = ASSEMBLED_H - BASE_H - TRAY_H  # 0.011 m: 11 mm runner height between 65 mm deck and 80 mm scoop (= 156 mm total)

BTN_X = figure("round release button centre x", 0.172, "m", FIXTURE,
               "centred above the LCD on the 82 mm exposed front apron, just ahead of the recessed front step (Fig. 1-2)")
BTN_TOP_Z = BASE_H + 0.0055  # 70.5 mm above bench when unpressed

MAT_X = figure("grey mat centre x", 0.630, "m", FIXTURE,
               "spans x = 48.5..77.5 cm (20 mm clear behind the assembled tray's 46.5 cm back edge, inside the 91 cm bench)")
MAT_Y = figure("grey mat centre y", 0.000, "m", FIXTURE,
               "centred on y = 0 (spans y = -18.5..+18.5 cm) for symmetric two-arm placement")
MAT_FLOOR_T = figure("grey mat floor thickness in the model", 0.006, "m", ESTIMATE,
                     "estimate: the real moulded wall is about 2.5 mm (0.30 kg of polypropylene at 0.9 g/cm3 over about 1,340 cm2); 6 mm is a stand-in slab for the solver")
MAT_RIM_T = figure("grey mat rim thickness in the model", 0.008, "m", ESTIMATE,
                   "estimate: stand-in for the same about 2.5 mm moulded side wall (see the floor figure); not used by the score")

# --- Front-step interlock (world x at the locked start; seca manual Fig. 1-2: latch behind the button) --------------
# The standing deck behind x = 26.6 cm is closed. The recessed front step (x = 19.0..26.6 cm, 3 mm below the deck) holds
# two keyhole slots and the button's pawl; the tray's latch plate carries two T-studs and a tongue (simplified colliders).
STEP_X = (0.190, 0.266)        # recessed front step behind the button, top 3 mm below the 65 mm deck
STEP_TOP = BASE_H - 0.003      # 62 mm above bench
KEY_Y = 0.050                  # +-50 mm keyhole centres
KEY_COL_HALF_Y = 0.014         # 28 mm wide cavity under each keyhole
KEY_SLOT_HALF_Y = 0.005        # 10 mm wide narrow slot for the 6 mm stud stem
KEY_LIP_X = (0.214, 0.236)     # narrow slot with its two lips; open mouth [0.236, 0.266] (30 mm)
KEY_LIP_Z0 = 0.0565            # 5.5 mm thick lips (top = STEP_TOP)
STUD_HEAD_X = (0.220, 0.232)   # 12 x 22 mm T-stud head under the lips (4 mm vertical play)
STUD_STEM_X = (0.222, 0.230)   # 8 x 6 mm stem through the narrow slot
STUD_HEAD_HALF_Y = 0.011
STUD_STEM_HALF_Y = 0.003
STUD_Z = (0.0455, 0.0525)      # head bottom and top above bench (1.5 mm over the cavity floor)
PAWL_X = (0.236, 0.244)        # sprung pawl on the button's link, top 3 mm below the button top
PAWL_HALF_Y = 0.012
TONGUE_X = (0.224, 0.232)      # tray tongue ahead of the pawl (4 mm gap); bottom 1.2 mm above the step
TONGUE_Z0 = STEP_TOP + 0.0012  # 63.2 mm: the pawl top (67.5 mm) must drop 4.3 mm to pass under it
PAWL_TOP_Z = TONGUE_Z0 + BTN_UNLOCK
SLIDE_TARGET = 0.023           # tray slide the demo aims for: studs in the mouths, tongue 3 mm past the pawl (stop at 34 mm)
SLIDE_HAND = 0.032             # the pushing hand's travel: pushed at one end, the tray yaws ~2 deg and its centre trails

# --- Scoring thresholds (identical in task.py, task.json, README.md) --------------------------------------------------
BASE_LIFT_MAX = figure("maximum allowed scale base corner lift", 0.008, "m", FIXTURE,
                       "8.0 mm corner lift limit: pulling the locked tray up lifts the base and loses the trial")
BASE_SHIFT_MAX = figure("maximum allowed scale base horizontal shift", 0.015, "m", FIXTURE,
                        "15.0 mm shift from the taped corner marks")
BASE_TILT_MAX_DEG = 3.0
BENCH_DROP_SPEED = 0.50  # m/s downward impact onto the bench
SEPARATED_SLIDE = HOOK_SLIDE  # 0.020 m backward slide clears the hooks
TRAY_UPRIGHT_DEG = 8.0
MAT_TILT_MAX_DEG = 5.0
MAT_CENTER_TOL_X = figure("tray-on-mat front-back centring tolerance", 0.030, "m", FIXTURE,
                          "tray centre within 3.0 cm of mat centre along x (ruler against the 29.0 cm outer front/back edges)")
MAT_CENTER_TOL_Y = figure("tray-on-mat lateral centring tolerance", 0.050, "m", FIXTURE,
                          "tray centre within 5.0 cm of mat centre along y (overhangs 2.75..12.75 cm past each 37.0 cm outer side edge)")
BTN_REST_MAX = 0.001
HANDS_CLEAR = 0.030
REST_LIN = 0.020
REST_ANG = 0.20
HOLD_STEPS = 20
HOLD_DRIFT = 0.002
RUN_LIMIT_STEPS = 1200


def base_corners_local() -> np.ndarray:
    hx, hy = BASE_D / 2, BASE_W / 2
    return np.array([[sx * hx, sy * hy, 0.0] for sx in (-1.0, 1.0) for sy in (-1.0, 1.0)])


def summary() -> dict:
    by_kind: dict[str, int] = {}
    for _, _, s, _ in FIGURES.values():
        by_kind[s] = by_kind.get(s, 0) + 1
    return {
        "counts_by_source": by_kind,
        "assembled_mm": [round(1000 * ASSEMBLED_W, 1), round(1000 * ASSEMBLED_H, 1), round(1000 * ASSEMBLED_D, 1)],
        "apron_exposed_mm": round(1000 * APRON_EXPOSED, 1),
        "masses_kg": {"base": BASE_MASS, "btn": BTN_MASS, "tray": TRAY_MASS, "sum": round(BASE_MASS + BTN_MASS + TRAY_MASS, 3), "mat": MAT_MASS},
        "figures": {k: {"value": v, "unit": u, "source": s, "note": n} for k, (v, u, s, n) in FIGURES.items()},
    }
