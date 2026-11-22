"""Every figure the Systainer T-Loc coupling task rests on, in one place, each with its source.

Source kinds (``FIGURES`` holds every figure with its kind and its source in words; the README's table is made from it):
- ``maker``: the maker's own published product page (Festool ``577808`` ``SYS3 S 76`` and ``577818`` ``SYS3 S 147``
  pages opened directly on ``festool.com``).
- ``listing``: an opened retailer's published product specification.
- ``estimate``: our figure (including search-indexed product specifications whose pages were not opened directly), with
  how it was derived in one clause.

World frame (the station's): metres from the midpoint between the arm bases, x forward, y left, z up from the bench.
"""

from __future__ import annotations

import math

GRAVITY = 9.81
OVER = 0.001  # m: loose bodies start 1 mm over their support so they settle at reset and render from frame 0

MAKER, LISTING, ESTIMATE = "maker", "listing", "estimate"

FESTOOL_S76_URL = "https://www.festool.com/products/systainer-and-transport-systems/systainer/577808---sys3-s-76"
FESTOOL_S147_URL = "https://www.festool.com/products/systainer-and-transport-systems/systainer/577818---sys3-s-147"
CONRAD_456300_URL = "https://www.conrad.com/en/p/allit-456300-insert-box-l-x-w-x-h-54-x-54-x-45-mm-number-of-compartments-1-polystyrene-ps-red-1-pc-s-1520907.html"
CONRAD_NOTE = f"Conrad listing 1520907 for Allit EuroPlus Insert 45/1 art. 456300 ({CONRAD_456300_URL}, listing, not opened by us: HTTP 403)"

FIGURES: dict[str, tuple[float, str, str, str]] = {}  # name -> (value, unit, source kind, note)


def figure(name: str, value: float, unit: str, source: str, note: str) -> float:
    if source not in (MAKER, LISTING, ESTIMATE):
        raise ValueError(f"{name}: unknown source kind {source!r}")
    FIGURES[name] = (float(value), unit, source, note)
    return float(value)


# --- Festool Systainer3 SYS3 S 76, item 577808 (light grey opaque ABS) -----------------------------------------------
CASE_W = figure(
    "SYS3 S 76 outer length (along y)",
    0.265,
    "m",
    MAKER,
    f"maker's page: '265 x 171 x 71 mm' ({FESTOOL_S76_URL})",
)
CASE_D = figure(
    "SYS3 S 76 outer width (along x)",
    0.171,
    "m",
    MAKER,
    f"maker's page: '265 x 171 x 71 mm' ({FESTOOL_S76_URL})",
)
CASE_BODY_H = figure(
    "SYS3 S 76 height without feet",
    0.071,
    "m",
    MAKER,
    f"maker's page: '265 x 171 x 71 mm' ({FESTOOL_S76_URL})",
)
CASE_H = figure(
    "SYS3 S 76 height with feet",
    0.076,
    "m",
    MAKER,
    f"maker's page: '265 x 171 x 76 mm' including feet ({FESTOOL_S76_URL})",
)
FOOT_H = CASE_H - CASE_BODY_H  # 0.005 m: exact nesting foot height from maker heights (76 - 71 mm; 2x71 + 5 = 147 mm)

INNER_W = figure(
    "SYS3 S 76 inner length (along y)",
    0.258,
    "m",
    MAKER,
    f"maker's page: '258 x 164 x 67 mm' ({FESTOOL_S76_URL})",
)
INNER_D = figure(
    "SYS3 S 76 inner width (along x)",
    0.164,
    "m",
    MAKER,
    f"maker's page: '258 x 164 x 67 mm' ({FESTOOL_S76_URL})",
)
INNER_H = figure(
    "SYS3 S 76 inner height",
    0.067,
    "m",
    MAKER,
    f"maker's page: '258 x 164 x 67 mm' ({FESTOOL_S76_URL})",
)
WALL_T = (CASE_W - INNER_W) / 2.0  # 0.0035 m: exact wall thickness from maker outer minus inner (3.5 mm)

CASE_EMPTY_MASS = figure(
    "SYS3 S 76 empty mass",
    0.40,
    "kg",
    MAKER,
    f"maker's page: '0.40 kg' without accessories ({FESTOOL_S76_URL})",
)
LID_MASS = figure(
    "lower Systainer hinged lid share of mass",
    0.08,
    "kg",
    ESTIMATE,
    "derived from roof/rim surface share (~20 %) of the 0.40 kg ABS case",
)
TLOC_MASS = figure(
    "T-Loc rotary cam mass",
    0.02,
    "kg",
    ESTIMATE,
    "derived from hollow ABS 56 x 38 x 17 mm rotary catch volume (~18 cm3)",
)
TUB_LOW_MASS = CASE_EMPTY_MASS - LID_MASS - TLOC_MASS  # 0.30 kg: case_low + lid_low + tloc_low = 0.40 kg maker mass
PARTITIONS_MASS = figure(
    "SYS3 S 76 internal partitions mass in upper case",
    0.08,
    "kg",
    ESTIMATE,
    f"derived from ABS volume of maker's included '4x short partition' and '1x long partition' ({FESTOOL_S76_URL})",
)
CASE_TOP_MASS = CASE_EMPTY_MASS + PARTITIONS_MASS  # 0.48 kg: upper closed Systainer with partitions fitted

# --- Allit EuroPlus Insert 45/1, Art. 456300, EAN 4005187563009 (red polystyrene parts box) ---------------------------
BOX_W = figure(
    "Allit 456300 parts box outer width",
    0.054,
    "m",
    ESTIMATE,
    f"the size in the listing's title and URL, '54 x 54 x 45 mm' ({CONRAD_NOTE})",
)
BOX_D = figure(
    "Allit 456300 parts box outer depth",
    0.054,
    "m",
    ESTIMATE,
    f"the size in the listing's title and URL, '54 x 54 x 45 mm' ({CONRAD_NOTE})",
)
BOX_H = figure(
    "Allit 456300 parts box outer height",
    0.045,
    "m",
    ESTIMATE,
    f"the size in the listing's title and URL, '54 x 54 x 45 mm' ({CONRAD_NOTE})",
)
BOX_WALL = 0.003  # m: gives 48 x 48 x 42 mm inner bin cavity
BOX_REAL_MASS = figure(
    "Allit 456300 parts box empty mass",
    0.014,
    "kg",
    ESTIMATE,
    "derived from a 54 x 54 x 45 mm polystyrene box with about 1.1 mm walls: 13.4 cm3 at 1.05 g/cm3",
)
BOX_SIM_MASS = figure(
    "Allit 456300 parts box simulated mass",
    0.060,
    "kg",
    ESTIMATE,
    "raised from 14 g to 60 g in PhysX so a 25 N parallel-jaw pinch gives stable contact without solver jitter",
)

# --- Scene start coordinates on the bench -----------------------------------------------------------------------------
CASE_LOW_XY = (0.345, 0.000)  # m: lower Systainer centred on y = 0, front wall at x = 0.2595 m
CASE_TOP_XY = (0.165, 0.000)  # m: upper Systainer flat on bench ahead of case_low (x = 0.0795..0.2505 m)
BOX_START_XY = (0.215, 0.215)  # m: red parts box on bench to the left of the cases

# Interior divider layout in case_low (using maker's included partitions): front-left cell 92 x 106 mm for the 54 mm box
FLOOR_T = 0.002  # m: tub floor z_local = 0.005..0.007 m (top at 7.0 mm)
FLOOR_TOP_Z = FOOT_H + FLOOR_T  # 0.007 m
DIVIDER_H = figure(
    "interior partition rib height",
    0.044,
    "m",
    ESTIMATE,
    "derived to sit under the 67 mm interior ceiling while holding a cocked 45 mm insert box proud of the lid",
)
DIVIDER_T = 0.003  # m
CELL_W = 0.106  # m: clearance around 54 mm Allit insert box
CELL_D = 0.092  # m: keeps gripper clear of both front wall and 62-deg built / 57.3-deg settled open lid
# Front-left compartment centre in case_low frame: x in [-0.082, +0.010], y in [+0.023, +0.129]
CELL_LOCAL_XY = (-INNER_D / 2.0 + CELL_D / 2.0, INNER_W / 2.0 - CELL_W / 2.0)  # (-0.036, +0.076) m

# --- Lower lid hinge and stay -----------------------------------------------------------------------------------------
TUB_RIM_Z = 0.068  # m: top of lower tub walls
LID_DECK_Z = CASE_BODY_H  # 0.071 m: recessed lid deck; upper 5 mm feet sit here so coupled stack = 71 + 76 = 147 mm
LID_HINGE_LOCAL = (0.0820, 0.0, 0.0700)  # m in case_low frame (rear top spine)
LID_OPEN_DEG = figure(
    "lower lid propped-open start angle",
    62.0,
    "deg",
    ESTIMATE,
    "derived so the open lid clears the front-left cell and top-down gripper while keeping closing contact <= 148 mm",
)
LID_MAX_DEG = 66.0
LID_OPEN_RAD = math.radians(LID_OPEN_DEG)
LID_MAX_RAD = math.radians(LID_MAX_DEG)
LID_STAY_SWITCH_DEG = 26.0  # deg: over-centre stay holds lid open above 26 deg, closes smoothly at/below 26 deg
LID_STAY_K = 0.26  # N m/rad: over-centre lid-stay spring (~1.1 N push at 90 mm from hinge, below bench foot slip)
LID_STAY_DAMP = 0.025  # N m s/rad


def lid_stay_rest(angle_rad: float) -> float:
    """Over-centre lid-stay spring rest in lid-angle coords, clamped within +/-40 deg of ``angle_rad``."""
    goal = LID_OPEN_RAD + math.radians(2.5) if math.degrees(angle_rad) > LID_STAY_SWITCH_DEG else -math.radians(4.0)
    diff = max(-TLOC_STEP_CAP_RAD, min(TLOC_STEP_CAP_RAD, goal - angle_rad))
    return angle_rad + diff


def lid_stay_travel_rest(travel_rad: float) -> float:
    """Spring rest in built-open joint travel coordinates (travel 0 = LID_OPEN_RAD, closed = -LID_OPEN_RAD)."""
    return lid_stay_rest(LID_OPEN_RAD + travel_rad) - LID_OPEN_RAD

# --- Rear cleat hook-and-slide interlock (case_top <-> lid_low) -------------------------------------------------------
CLEAT_Y = 0.085  # m: left/right cleat centres (+/-85 mm off centre, clear of central latch recess)
CLEAT_W = 0.036  # m: width of each lid cleat along y
CLEAT_GAP = figure(
    "rear lid cleat vertical throat height",
    0.0065,
    "m",
    ESTIMATE,
    "derived from 5.0 mm Systainer foot height (76 - 71 mm) plus 3.0 mm slide clearance over a 3.5 mm rear hook tongue",
)
CLEAT_ROOF_T = 0.0030  # m: cleat roof z_local = 0.0775..0.0805 m on lid_low
CLEAT_THROAT_X = (0.0705, 0.0825)  # m in case_low frame: 12.0 mm clear throat ahead of +0.0825..+0.0855 rear post
CLEAT_SLIDE = figure(
    "rear cleat horizontal slide engagement stroke",
    0.0120,
    "m",
    ESTIMATE,
    "derived from 11.5 mm hook-under-cleat overlap plus 0.5 mm landing clearance ahead of the cleat lip",
)
HOOK_W = 0.028  # m: width of each upper-case bottom foot/hook along y (8 mm lateral clearance inside 36 mm cleat)
HOOK_T = 0.0035  # m: thickness of rear hook tongue (z_local = 0.000..0.0035 m; 3.0 mm below z = 0.0775 m cleat roof)
HOOK_X = (0.0665, 0.0820)  # m in case_top frame: at dx=-12 mm sits ahead of +0.0705; at dx=0 tucks 11.5 mm under cleat
FRONT_FOOT_X = (-0.0735, -0.0655)  # m in case_top frame: rides on front lip at dx=-12 mm, drops onto 71 mm deck at dx=0
FRONT_RIM_X = (-0.0855, -0.0785)  # m in case_low frame: 7 mm front lip on lid_low outside central recess

# --- Front central latch recess & 3-position T-Loc rotary cam on case_low ---------------------------------------------
RECESS_HALF_Y = 0.064  # m: 128 mm wide central front latch bay on both SYS3 S 76 cases
RECESS_WALL_X = -0.0695  # m in case frame: stepped back 16.0 mm behind outer front face (-0.0855 m)

TLOC_W = figure(
    "T-Loc T-knob crossbar width",
    0.056,
    "m",
    ESTIMATE,
    f"scaled from 265 mm front case width on Festool product elevation ({FESTOOL_S76_URL})",
)
TLOC_H = figure(
    "T-Loc T-knob overall height",
    0.038,
    "m",
    ESTIMATE,
    f"scaled from 71 mm body height on Festool product elevation ({FESTOOL_S76_URL})",
)
TLOC_BAR_T = 0.016  # m: crossbar height and stem width (parallel grasp faces on both 20 mm T-wings)
TLOC_DEPTH = figure(
    "T-Loc T-knob grip depth",
    0.017,
    "m",
    ESTIMATE,
    "derived to sit inside the Systainer3 front recess with 7 mm finger clearance to the recessed wall",
)
TLOC_PIVOT_LOCAL = (-0.07325, 0.0, 0.0450)  # m in case_low frame (mounted on case_low front wall, z = 45.0 mm)

TLOC_OPEN_DEG = figure(
    "T-Loc OPEN detent angle",
    -55.0,
    "deg",
    ESTIMATE,
    f"derived from 3-position T-Loc geometry ('closed, opened and connected with a single turn', {FESTOOL_S147_URL}) so knob and lobes stay below the 73 mm lid rim",
)
TLOC_CLOSE_DEG = figure(
    "T-Loc CLOSE detent angle",
    0.0,
    "deg",
    ESTIMATE,
    f"horizontal T-bar closed position retaining lower lid ({FESTOOL_S147_URL})",
)
TLOC_CONNECT_DEG = figure(
    "T-Loc CONNECT detent angle",
    90.0,
    "deg",
    ESTIMATE,
    f"quarter-turn vertical T-bar coupling upper case while retaining lower lid ({FESTOOL_S147_URL})",
)
TLOC_MIN_DEG, TLOC_MAX_DEG = -62.0, 95.0
TLOC_OPEN_RAD = math.radians(TLOC_OPEN_DEG)
TLOC_CLOSE_RAD = math.radians(TLOC_CLOSE_DEG)
TLOC_CONNECT_RAD = math.radians(TLOC_CONNECT_DEG)
TLOC_MIN_RAD = math.radians(TLOC_MIN_DEG)
TLOC_MAX_RAD = math.radians(TLOC_MAX_DEG)

TLOC_STEP_CAP_DEG = 40.0  # deg: spring rest target kept strictly <= 40 deg from joint angle (kit <= 45 deg rule)
TLOC_STEP_CAP_RAD = math.radians(TLOC_STEP_CAP_DEG)
TLOC_DETENT_NM = figure(
    "T-Loc peak detent breakaway torque",
    0.18,
    "N m",
    ESTIMATE,
    "derived from ~6.4 N one-hand finger push at the 28 mm T-Loc wing tip across detent ridges",
)
# PhysX angular drive torque is 2 * k * sin(e / 2); at e = 40 deg, 2 * sin(20 deg) * k = TLOC_DETENT_NM
TLOC_K = TLOC_DETENT_NM / (2.0 * math.sin(TLOC_STEP_CAP_RAD / 2.0))
TLOC_DAMP = 0.018  # N m s/rad

# --- Friction coefficients --------------------------------------------------------------------------------------------
BENCH_FRICTION = figure(
    "Systainer anti-slip feet on bench friction",
    1.10,
    "-",
    ESTIMATE,
    "elastomer/polymer Systainer feet on workbench (exceeds smooth 0.18 ABS-on-ABS cleat slide friction so stack stays put)",
)
CASE_FRICTION = figure(
    "ABS-on-ABS inter-case sliding friction",
    0.18,
    "-",
    ESTIMATE,
    "smooth moulded ABS feet sliding on smooth ABS lid deck and rims during the 12 mm cleat engagement",
)
SIDE_GRIP_FRICTION = 1.20  # grippy side walls against silicone/rubber gripper pads (yam_arena default GRIPPY)

# --- Scoring thresholds (identical in task.py, task.json, and README.md) ----------------------------------------------
RUN_LIMIT_STEPS = 1600  # 80 s at 20 Hz
CONTROL_HZ = 20.0

FALL_BELOW_BENCH = 0.030  # m below bench top: whole-trial zero rule if any object falls off bench

BOX_FLOOR_TOL = 0.005  # m: stowed parts_box bottom within 5 mm of case_low floor (7 mm)
BOX_TILT_MAX_DEG = 10.0  # deg: stowed parts_box upright within 10 deg
LID_SHUT_MAX_DEG = 2.5  # deg: lower lid closed within 2.5 deg

SEAM_GAP_MAX = 0.0025  # m: inter-case seam <= 2.5 mm all round when seated flush
STACK_DX_MAX = 0.0040  # m: upper case front-to-back alignment within 4.0 mm of lower case
STACK_DY_MAX = 0.0060  # m: upper case lateral alignment within 6.0 mm of lower case
STACK_YAW_MAX_DEG = 6.0  # deg: upper case yaw within 6.0 deg of lower case
STACK_TILT_MAX_DEG = 3.0  # deg: upper case tilt relative to lower case <= 3.0 deg
CLEAT_OVERLAP_MIN = 0.0050  # m: rear hook tongues engaged >= 5.0 mm under rear lid cleats

TLOC_CONNECT_MIN_DEG = 80.0  # deg: T-Loc in CONNECT detent (+80 to +95 deg)
TLOC_CONNECT_MAX_DEG = 95.0  # deg
LIFT_CHECK_N = 15.0  # N: upward pull check in trials.py verifying coupled retention (lifts 0.94 kg stack together)

HANDS_CLEAR_M = 0.030  # m: both robot hands >= 30 mm clear of all task parts
HOLD_STEPS = 20  # 1.0 s at 20 Hz with hands clear
HOLD_DRIFT_M = 0.002  # m: cases move <= 2.0 mm during the 1.0 s settled hold
HOLD_TLOC_DEG = 2.0  # deg: T-Loc moves <= 2.0 deg during the 1.0 s settled hold


def tloc_well_rad(angle_rad: float) -> float:
    """Nearest detent well (-55, 0, or +90 deg) for the 3-position T-Loc cam."""
    deg = math.degrees(angle_rad)
    if deg < -27.5:
        return TLOC_OPEN_RAD
    if deg <= 45.0:
        return TLOC_CLOSE_RAD
    return TLOC_CONNECT_RAD


def tloc_stepped_rest(angle_rad: float) -> float:
    """Spring rest target clamped within +/-40 deg of ``angle_rad`` (respects kit <= 45 deg drive rule)."""
    well = tloc_well_rad(angle_rad)
    diff = max(-TLOC_STEP_CAP_RAD, min(TLOC_STEP_CAP_RAD, well - angle_rad))
    return angle_rad + diff


def tloc_travel_rest(travel_rad: float) -> float:
    """Spring rest in built-open joint travel coordinates (travel 0 = TLOC_OPEN_RAD)."""
    return tloc_stepped_rest(TLOC_OPEN_RAD + travel_rad) - TLOC_OPEN_RAD


def tloc_torque_nm(angle_rad: float) -> float:
    """Restoring torque (N m) of the stepped T-Loc spring drive at ``angle_rad`` (positive toward +angle)."""
    rest = tloc_stepped_rest(angle_rad)
    return 2.0 * TLOC_K * math.sin((rest - angle_rad) / 2.0)


def source_counts() -> dict[str, int]:
    out = {MAKER: 0, LISTING: 0, ESTIMATE: 0}
    for _, _, kind, _ in FIGURES.values():
        out[kind] += 1
    return out


def summary() -> dict:
    return {
        "counts": source_counts(),
        "total_figures": len(FIGURES),
        "masses_kg": {
            "case_low_total": round(TUB_LOW_MASS + LID_MASS + TLOC_MASS, 4),
            "case_top": round(CASE_TOP_MASS, 4),
            "parts_box_sim": BOX_SIM_MASS,
            "parts_box_listed": BOX_REAL_MASS,
        },
        "tloc_k_nm_per_rad": round(TLOC_K, 4),
        "tloc_peak_torque_nm": round(abs(tloc_torque_nm(math.radians(40.0))), 4),
        "coupled_stack_height_mm": round(1000.0 * (LID_DECK_Z + CASE_H), 1),
        "figures": {n: {"value": v, "unit": u, "source": s, "note": t} for n, (v, u, s, t) in FIGURES.items()},
    }
