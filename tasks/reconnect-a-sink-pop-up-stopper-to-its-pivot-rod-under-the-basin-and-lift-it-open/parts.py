"""Every figure the bathroom-sink pop-up stopper task rests on, each with its source, and its derived geometry.

Source kinds in ``FIGURES`` (used directly to build the README table):
- ``maker``: the manufacturer's published product specification or support datasheet.
- ``standard``: plumbing waste fitting standard (ASME A112.18.2 / CSA B125.2).
- ``listing``: a retailer's published specification for the named part.
- ``our fixture``: a benchtop vanity, collar, cradle, or mark cut/printed to the stated dimension.
- ``estimate``: derived figure, with its one-clause derivation.

World frame (``station``): metres from the midpoint between the arm bases, x forward, y left, z up from the bench.
"""

from __future__ import annotations

GRAVITY = 9.81
OVER = 0.0006  # m: initial settle clearance for free bodies at reset

MAKER, STANDARD, LISTING, FIXTURE, ESTIMATE = "maker", "standard", "listing", "our fixture", "estimate"
SEEN_IN_SEARCH = "as shown in a web search result (page not opened), 2026-11-21"

DANCO_11041_MRO = "https://mrosupreme.com/products/danco-11041-pop-up-sink-stopper-pop-up-plastic-chrome"
DANCO_11041_HD = "https://www.homedepot.com/p/DANCO-5-in-Plastic-Bathroom-Pop-Up-Drain-Stopper-in-Chrome-11041/318993801"
DANCO_88532_HD = "https://www.homedepot.com/p/DANCO-Universal-Ball-Rod-for-Pop-Up-Drains-88532/203193993"
DANCO_80767_HD = "https://www.homedepot.com/p/DANCO-Plastic-Pop-Up-Assembly-in-Chrome-80767/204485487"
CORNAT_DRAIN_URL = "https://www.bueromarkt-ag.de/ablaufgarnitur_cornat_tec317405_aus_messing,p-tec317405.html"
CORNAT_STOP_URL = "https://www.bueromarkt-ag.de/waschbeckenstoepsel_cornat_exzenterstopfen_durchschnitt_35_mm,p-tec317414.html"

FIGURES: dict[str, tuple[float, str, str, str]] = {}


def figure(name: str, value: float, unit: str, source: str, note: str) -> float:
    if source not in (MAKER, STANDARD, LISTING, FIXTURE, ESTIMATE):
        raise ValueError(f"{name}: unknown source kind {source!r}")
    FIGURES[name] = (float(value), unit, source, note)
    return float(value)


# --- Pop-up stopper: Danco 11041 (UPC 037155110414); EU counterpart Cornat TEC317414 (EAN 4035300765614) --------------
STOP_H = figure(
    "stopper total height (Danco 11041)",
    0.1321,
    "m",
    LISTING,
    f"'5.2 in in total length with 1.4 in OD head' ({DANCO_11041_MRO}); EU Cornat TEC317414: '3,5 x 12 cm' ({CORNAT_STOP_URL})",
)
STOP_HEAD_D = figure(
    "stopper head outside diameter (Danco 11041)",
    0.0356,
    "m",
    LISTING,
    f"'1.4 in OD head' = 35.6 mm ({DANCO_11041_MRO}); EU Cornat TEC317414: 'Durchmesser: 35 mm' ({CORNAT_STOP_URL})",
)
STOP_LAST_HOLE = figure(
    "stopper shaft length under head to 6th hole (Danco 11041)",
    0.1153,
    "m",
    LISTING,
    f"'the pop-up stopper shaft length from underneath the stopper head to the last hole is 4.54 in' ({DANCO_11041_MRO})",
)
STOP_CAP_H = figure(
    "stopper cap and seal height above flange seat",
    0.0125,
    "m",
    ESTIMATE,
    "derived from Danco 11041 132.1 mm total minus 115.3 mm shaft to 6th hole and 4.3 mm bottom ring web",
)
STOP_BELOW_SEAL = STOP_H - STOP_CAP_H  # 0.1196 m below flange seat when closed
EYE_W = figure(
    "stopper clevis eye inner width",
    0.0085,
    "m",
    ESTIMATE,
    "scaled from Danco 11041 1.4 in head product photo across the 6-hole clevis strap",
)
EYE_H = figure(
    "stopper clevis eye inner vertical opening",
    0.0100,
    "m",
    ESTIMATE,
    "oblong clevis slot scaled from Danco 11041 shaft photo so a 4.5 mm tip pivots +-20 deg without binding",
)
EYE_WALL = 0.0022  # m: radial wall around the clevis eye
EYE_THICK = 0.0040  # m: clevis strap thickness along the hole axis
EYE_DEPTH = figure(
    "active top eye depth below stopper cap seal",
    0.0882,
    "m",
    ESTIMATE,
    "fourth hole (#4 of 6) along Danco 11041's 115.3 mm shaft (~8.8 mm hole pitch); unused holes covered with vinyl tape",
)
STOP_MASS = figure(
    "stopper mass (chrome-plated ABS)",
    0.036,
    "kg",
    ESTIMATE,
    "volume of 35.6 mm cap + 119.6 mm X-fin ABS stem at 1,060 kg/m3",
)
GUIDE_SPAN = 0.0292  # m: X-fin guide outer span inside 31.8 mm bore (1.3 mm radial clearance per side)
FIN_THICK = 0.0032

# --- Pivot ball rod: Danco 88532 (UPC 037155885329); EU counterpart in Cornat TEC317405 (EAN 4035300201006) -----------
ROD_OUTER_L = figure(
    "pivot rod steel shaft length (Danco 88532)",
    0.1500,
    "m",
    MAKER,
    f"DancoSupport spec: '150 mm (5.9 inches) in length' ({SEEN_IN_SEARCH}: {DANCO_88532_HD})",
)
ROD_D = figure(
    "pivot rod steel shaft diameter (Danco 88532)",
    0.0055,
    "m",
    MAKER,
    f"DancoSupport spec: '5.5 mm (0.22 inches) in diameter' ({SEEN_IN_SEARCH}: {DANCO_88532_HD})",
)
ROD_THREAD_IN = figure(
    "pivot rod thread engagement into plastic ball (Danco 88532)",
    0.0040,
    "m",
    MAKER,
    f"DancoSupport spec: threads in 'approximately 4 mm (0.16 inches)' ({SEEN_IN_SEARCH}: {DANCO_88532_HD})",
)
BALL_D = figure(
    "pivot ball diameter, large ball (Danco 88532)",
    0.0183,
    "m",
    MAKER,
    f"DancoSupport spec: 'Large Ball: Diameter = 18.3 mm (0.72 inch)' ({SEEN_IN_SEARCH}: {DANCO_88532_HD})",
)
BALL_R = BALL_D / 2.0  # 0.00915 m
TIP_BEYOND_BALL = figure(
    "inner stub length from ball edge to tip (Danco 88532)",
    0.0275,
    "m",
    MAKER,
    f"DancoSupport spec: 'distance from the edge of the ball to the end of the gray rod = 27.5 mm (1.08 inches)' ({SEEN_IN_SEARCH}: {DANCO_88532_HD})",
)
TIP_D = figure(
    "inner gray rod tip diameter (Danco 88532)",
    0.0045,
    "m",
    MAKER,
    f"DancoSupport spec: tapers from ~5 mm at the ball to 'about 4.5 mm at the tip' ({SEEN_IN_SEARCH}: {DANCO_88532_HD})",
)
TIP_R = TIP_D / 2.0  # 0.00225 m
TIP_REACH = BALL_R + TIP_BEYOND_BALL  # 0.03665 m from ball centre to inner tip end
ROD_REAR_REACH = BALL_R + (ROD_OUTER_L - ROD_THREAD_IN)  # 0.15515 m from ball centre to rear rod end
CLEVIS_BLOCK = (0.022, 0.014, 0.016)  # m: brass clevis connector ('Schloss' / clevis clip on rear rod, Cornat TEC317405 / Danco 80767)
CLEVIS_AT_X = 0.068  # m behind ball centre on the 146 mm rear shaft
ROD_MASS = figure(
    "pivot rod + clevis connector mass (Danco 88532 + 80767 / Cornat TEC317405)",
    0.044,
    "kg",
    ESTIMATE,
    "150 x 5.5 mm steel shaft (28.0 g) + 18.3 mm ball/stub (6.0 g) + 22 x 12 x 12 mm clevis connector (10.0 g)",
)

# --- Drain body & rear port: ASME A112.18.2 1-1/4 in / Danco 80767 / Cornat TEC317405 ---------------------------------
DRAIN_ID = figure(
    "lavatory drain nominal bore (1-1/4 in)",
    0.0318,
    "m",
    STANDARD,
    f"ASME A112.18.2 / CSA B125.2 1-1/4 in (31.8 mm) lavatory waste fitting; Danco 80767 ({SEEN_IN_SEARCH}: {DANCO_80767_HD}); Cornat TEC317405 '1 1/4 Zoll Anschluss' ({CORNAT_DRAIN_URL})",
)
DRAIN_OD = figure(
    "drain tee body outside diameter",
    0.0380,
    "m",
    ESTIMATE,
    "31.8 mm bore plus 3.1 mm ABS wall thickness on 1-1/4 in pop-up tee body",
)
FLANGE_D = figure(
    "top basin drain flange outside diameter",
    0.0540,
    "m",
    MAKER,
    f"standard 2-1/8 in (54.0 mm) lavatory pop-up flange (Delta RP5651 spec, {SEEN_IN_SEARCH}); Cornat TEC317405: 62 mm ({CORNAT_DRAIN_URL})",
)
X_DRAIN = figure("drain vertical axis x", 0.375, "m", FIXTURE, "37.5 cm forward of the arm-base line (beyond link2 elbow radius)")
Y_DRAIN = figure("drain vertical axis y", 0.035, "m", FIXTURE, "3.5 cm left of centre")
FLANGE_Z = figure(
    "drain flange seat height over bench",
    0.136,
    "m",
    FIXTURE,
    "13.6 cm over the bench: clears the untrimmed 132.1 mm Danco 11041 stem by 16.4 mm while keeping all grasps under 16.0 cm",
)
PORT_Z = figure(
    "rear pivot port centre height over bench",
    0.054,
    "m",
    FIXTURE,
    "5.4 cm over the bench (82.0 mm below flange): closed stopper eye sits 6.2 mm below port",
)
SEAT_OFFSET_X = figure(
    "rear port ball seat behind drain axis",
    0.026,
    "m",
    FIXTURE,
    "ball seat 26.0 mm behind drain axis (x = 0.401 m, 7.0 mm outside 19.0 mm tee wall) so 36.65 mm tip reaches 10.65 mm past stopper axis",
)
X_SEAT = X_DRAIN + SEAT_OFFSET_X  # 0.401 m
X_COLLAR = X_SEAT + 0.016  # 0.417 m: rear face of the split snap-collar
RETRACT_START = figure(
    "start retraction of pivot rod behind ball seat",
    0.032,
    "m",
    FIXTURE,
    "ball starts 32.0 mm behind seat (x = 0.433 m; red collar mark 32.0 mm behind collar face)",
)
X_BALL_START = X_SEAT + RETRACT_START  # 0.433 m

# --- Compliant cone-washer & 4-slot split snap-collar (pure 240 Hz PhysX, zero push/weld) -----------------------------
WASHER_INTERFERENCE = 0.00085  # m: 4 rounded cone-washer beads compress 0.85 mm radially around R = 9.15 mm seated ball
WASHER_K = figure(
    "cone-washer radial pad stiffness",
    12500.0,
    "N/m",
    ESTIMATE,
    "4 compliant elastomer beads x 0.85 mm interference = 10.6 N normal each; with friction 1.1 on R = 9.15 mm ball gives ~0.43 N m hold",
)
WASHER_DAMP = 90.0
WASHER_MU = figure(
    "cone-washer rubber friction on pivot ball",
    1.10,
    "-",
    ESTIMATE,
    "snug elastomer cone washer on plastic ball so seated rod holds ~0.35-0.43 N m against 0.010 N m stopper weight torque",
)
LIP_X_FROM_SEAT = 0.0055  # m: rear snap-collar lip 5.5 mm behind ball seat (world x = 0.4065 m; clears seated ball equator by 0.11 mm)
LIP_R = 0.0078  # m: 1.35 mm interference on R = 9.15 mm ball equator during insertion; clears 5.5 mm shaft up to 22 deg tilt
LIP_K = figure(
    "split snap-collar lip stiffness",
    4200.0,
    "N/m",
    ESTIMATE,
    "4 slotted PETG collar fingers yielding 1.35 mm on the 18.3 mm ball give an ~8-10 N axial snap-in click",
)
LIP_DAMP = 45.0
LIP_MU = 0.18
COLLAR_OD = 0.0264  # m: 3D-printed PETG 4-slot snap-collar outer diameter (collar_cad.py)
COLLAR_LEN = 0.0190  # m: 10.0 mm front skirt (19.2 mm pilot bore tapped to fit 80767 port boss) + 9.0 mm 4-slot crown
COLLAR_THREAD_LEN = 0.0100  # m: front skirt engagement length compressing the 88532 cone washer
PORT_THREAD_MAJOR = figure(
    "drain rear port external thread major diameter (Danco 80767)",
    0.0200,
    "m",
    ESTIMATE,
    "estimated 20.0 x 1.5 mm (0.78 in pop-up pivot-nut boss thread) on Danco 80767; checked with thread gauge to tap the 19.2 mm printed bore",
)

# --- Benchtop vanity carcass, full-depth deck with clear cast-acrylic rear panel, & ABS basin (our fixture) -----------
PLY_T = figure("vanity birch plywood thickness", 0.018, "m", FIXTURE, "18 mm birch plywood carcass (full height to countertop on left, right, and rear walls)")
VANITY_X0 = figure("vanity front face x", 0.334, "m", FIXTURE, "open front cabinet doorway faces the arms at x = 33.4 cm")
DECK_SPLIT_X = figure(
    "front stone deck to rear clear acrylic split x",
    0.444,
    "m",
    FIXTURE,
    "opaque front deck ends at x = 44.4 cm where full-width 160 x 424 x 9 mm clear cast-acrylic rear panel carries roof to rear wall",
)
ACRYLIC_T = figure(
    "clear cast-acrylic rear deck panel thickness",
    0.009,
    "m",
    FIXTURE,
    "9 mm clear cast-acrylic (PMMA) L-panel: 160 x 424 mm roof at z = 20.4 cm (x = 44.4..60.4 cm) + 23 mm front step strip to z = 17.2 cm",
)
CABINET_X1 = figure("vanity rear back panel outer x", 0.622, "m", FIXTURE, "full-height 18 mm rear cabinet wall at x = 60.4..62.2 cm up to z = 20.4 cm (15.8 mm behind retracted rod tail)")
Y_RIGHT = figure("vanity right outer wall y", -0.255, "m", FIXTURE, "inner face at y = -23.7 cm (46.0 cm overall vanity width)")
Y_LEFT = figure("vanity left outer wall y", 0.205, "m", FIXTURE, "inner face at y = +18.7 cm (46.0 cm overall vanity width)")
VANITY_TOP = figure("sunken front basin deck height", 0.172, "m", FIXTURE, "17.2 cm sunken front stone basin surround (x = 33.4..44.4 cm; grasps <= 15.95 cm)")
RIGHT_SHELF_TOP = figure("counter & full-width rear acrylic roof height", 0.204, "m", FIXTURE, "20.4 cm counter over front right stone shelf and full-width rear clear acrylic roof (19.5 cm clear interior height)")
BASIN_UNDER = figure("under-basin bowl underside height", 0.130, "m", FIXTURE, "13.0 cm clear under-basin height; 19.5 cm clear under the full-depth right counter shelf")
BASIN_WALL = figure(
    "solvent-welded white ABS basin sheet thickness",
    0.005,
    "m",
    FIXTURE,
    "5 mm white ABS sheet cut & solvent-welded to 110 x 190 x 42 mm outer box (100 x 180 x 38 mm inner bowl, 40 x 40 mm drain hole)",
)
BASIN_FLOOR = 0.134  # m: inner floor of the drop-in ABS basin (4 mm floor web z = 0.130..0.134 m; flange sits 2 mm proud at 0.136 m)
BASIN_X = (0.339, 0.439)  # m: 100 mm front-to-back inner basin opening (110 mm outer over x = 0.334..0.444 m)
BASIN_Y = (-0.018, 0.162)  # m: 180 mm wide inner basin opening (190 mm outer over y = -0.023..+0.167 m)
CRADLE_X = (0.563, 0.590)  # m: rear rod start rest; seated rod tail ends at x = 0.556 m so it clears the cradle by 7 mm when pivoting down

# --- Scoring thresholds (identical across task.py, task.json, README.md) ----------------------------------------------
ROD_DROP_ZERO = figure("whole-trial fall foul: pivot rod ball dropped toward floor", 0.025, "m", FIXTURE, "0 permanently if pivot rod ball drops below 25 mm over the bench or unseated rod touches cabinet floor")
COLLAR_SEAT_MAX = figure("seated collar mark tolerance", 0.0035, "m", FIXTURE, "red rod mark within 3.5 mm of rear collar face (ball within 3.5 mm of rear port seat)")
GAP_OPEN_MIN = figure("full-score minimum stopper open gap", 0.008, "m", FIXTURE, "stopper cap held >= 8.0 mm above closed flange seat")
GAP_OPEN_MAX = figure("full-score maximum stopper open gap", 0.022, "m", FIXTURE, "stopper cap <= 22.0 mm above closed flange seat (not propped on rim or left pulled out)")
STOP_RADIAL_HALF = 0.0040  # m: centred in bore for 0.5
STOP_RADIAL_FULL = 0.0035  # m: centred in bore for 1.0
STOP_TILT_HALF_DEG = 8.0
STOP_TILT_FULL_DEG = 6.0
REST_SPEED_MAX = 0.025  # m/s: suppresses 240 Hz compliant-pad micro-jitter while HOLD_DRIFT enforces <= 2 mm over 1.0 s
HANDS_CLEAR = 0.030  # m: every hull corner of both hands >= 30 mm clear
HOLD_S = 1.0
HOLD_STEPS = 20  # 1.0 s at 20 Hz
HOLD_DRIFT = 0.002  # m: stopper moves <= 2.0 mm during hold


def closed_eye_z() -> float:
    """World z (above bench) of the stopper eye centre when resting closed on FLANGE_Z: 0.0478 m (6.2 mm below PORT_Z = 0.0540 m)."""
    return FLANGE_Z - EYE_DEPTH


def expected_idle_gap_seated() -> float:
    """Theoretical stopper head gap (m) when threaded on a horizontal rod (theta = 0) without pressing rear rod down."""
    return (PORT_Z - closed_eye_z()) + TIP_R - EYE_H / 2.0  # 0.0062 + 0.00225 - 0.0050 = 0.00345 m (3.45 mm < 8.0 mm)


def rod_com_and_inertia() -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Centre of mass (m) and principal inertia (kg m^2) of pivot_rod in its ball-centred frame."""
    m_steel, m_ball, m_clevis = 0.028, 0.006, 0.010
    x_steel = BALL_R + (ROD_OUTER_L - ROD_THREAD_IN) / 2.0
    x_ball, x_clevis = -0.003, CLEVIS_AT_X
    cx = (m_steel * x_steel + m_ball * x_ball + m_clevis * x_clevis) / ROD_MASS
    l_s, r_s = ROD_OUTER_L - ROD_THREAD_IN, ROD_D / 2.0
    ixx = 0.5 * m_steel * r_s**2 + 0.4 * m_ball * BALL_R**2 + m_clevis * (CLEVIS_BLOCK[1] ** 2 + CLEVIS_BLOCK[2] ** 2) / 12.0
    iyy = (
        m_steel * (l_s**2 / 12.0 + (x_steel - cx) ** 2)
        + m_ball * (0.4 * BALL_R**2 + (x_ball - cx) ** 2)
        + m_clevis * (CLEVIS_BLOCK[0] ** 2 / 12.0 + (x_clevis - cx) ** 2)
    )
    return (cx, 0.0, 0.0), (max(ixx, 1e-6), iyy, iyy)


def stopper_com_and_inertia() -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Centre of mass (m) and inertia (kg m^2) of stopper in its frame (origin at cap underside seal z = 0)."""
    m_cap, m_stem = 0.014, STOP_MASS - 0.014
    z_cap, z_stem = STOP_CAP_H / 2.0, -STOP_BELOW_SEAL / 2.0
    cz = (m_cap * z_cap + m_stem * z_stem) / STOP_MASS
    r_c = STOP_HEAD_D / 2.0
    izz = 0.5 * m_cap * r_c**2 + 0.5 * m_stem * (GUIDE_SPAN / 2.0) ** 2
    ixx = (
        m_cap * (r_c**2 / 4.0 + (z_cap - cz) ** 2)
        + m_stem * (STOP_BELOW_SEAL**2 / 12.0 + (z_stem - cz) ** 2)
    )
    return (0.0, 0.0, cz), (ixx, ixx, max(izz, 2e-6))


def summary() -> dict:
    counts: dict[str, int] = {}
    for _, _, kind, _ in FIGURES.values():
        counts[kind] = counts.get(kind, 0) + 1
    return {
        "counts": counts,
        "x_seat_m": round(X_SEAT, 4),
        "tip_x_when_seated_m": round(X_SEAT - TIP_REACH, 5),
        "tip_past_stopper_axis_mm": round(1000.0 * (X_DRAIN - (X_SEAT - TIP_REACH)), 2),
        "closed_bottom_clearance_mm": round(1000.0 * (FLANGE_Z - STOP_BELOW_SEAL), 2),
        "closed_eye_below_port_mm": round(1000.0 * (PORT_Z - closed_eye_z()), 2),
        "horizontal_seated_gap_mm": round(1000.0 * expected_idle_gap_seated(), 2),
    }
