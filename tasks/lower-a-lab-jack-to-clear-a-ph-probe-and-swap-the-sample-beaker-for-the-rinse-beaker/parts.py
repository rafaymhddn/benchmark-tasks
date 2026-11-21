"""Every figure the lab scissor-jack pH-probe task rests on, each with its source, and helper geometry.

Source kinds in ``FIGURES`` (the README table is built from these):
- ``maker``: opened manufacturer specification page (BOCHEM Art. 11120, Thermo Scientific Nalgene 1201-0050PK).
- ``standard``: published engineering/laboratory standard (DIN 12897, ISO 7056, DIN 19262, DIN 6336).
- ``listing``: distributor/manufacturer catalog data from indexed product sheets (neoLab, Hanna Instruments, U.S. Plastic).
- ``our fixture``: task station setup geometry fixed with a steel ruler at reset.
- ``estimate``: derived value with its one-clause derivation.

World frame (``station``): metres from the midpoint between the arm bases, ``x`` forward, ``y`` left, ``z`` up from bench.
"""

from __future__ import annotations

import math

GRAVITY = 9.81
OVER = 0.0008  # m: loose bodies spawn 0.8 mm over their support so they settle cleanly at reset

MAKER, STANDARD, LISTING, FIXTURE, ESTIMATE = "maker", "standard", "listing", "our fixture", "estimate"
BOCHEM_URL = "https://bochem.de/produkt/hebebuehnen-18-10-stahl/?lang=en"
THERMO_URL = "https://www.thermofisher.com/order/catalog/product/1201-0050PK"
SEEN_SEARCH = "indexed product specification sheet in web search result (page not opened directly), 2026-11-21"

FIGURES: dict[str, tuple[float, str, str, str]] = {}


def figure(name: str, value: float, unit: str, source: str, note: str) -> float:
    if source not in (MAKER, STANDARD, LISTING, FIXTURE, ESTIMATE):
        raise ValueError(f"{name}: unknown source kind {source!r}")
    FIGURES[name] = (float(value), unit, source, note)
    return float(value)


# --- BOCHEM 11120 Lab-Jack (DIN 12897), 160 mm scissor axis along y, 130 mm short end along x, -y wheel ------------
JACK_L = figure(
    "jack plate length along scissor axis (y)", 0.160, "m", MAKER,
    f"BOCHEM Art. 11120 table: 'A (Length): 160', 'All models are manufactured according to DIN 12897' ({BOCHEM_URL})",
)
JACK_W = figure(
    "jack plate width on wheel end (x)", 0.130, "m", MAKER,
    f"BOCHEM Art. 11120 table: 'B (Width): 130' ({BOCHEM_URL})",
)
JACK_Z_MIN = figure(
    "jack minimum height C_min", 0.060, "m", MAKER,
    f"BOCHEM Art. 11120 table: 'C min: 60' mm ({BOCHEM_URL})",
)
JACK_Z_MAX = figure(
    "jack maximum height C_max", 0.275, "m", MAKER,
    f"BOCHEM Art. 11120 table: 'C max: 275' mm ({BOCHEM_URL})",
)
JACK_DYN_KG = figure(
    "jack maximum dynamic operating load", 7.0, "kg", MAKER,
    f"BOCHEM Art. 11120 table: 'Kg max dyn: 7' ({BOCHEM_URL})",
)
JACK_STAT_KG = figure(
    "jack maximum static operating load", 30.0, "kg", MAKER,
    f"BOCHEM Art. 11120 table: 'Kg max stat: 30' ({BOCHEM_URL})",
)
SCREW_PITCH = figure(
    "jack leadscrew / deck socket thread pitch (M8 x 1.25)", 0.00125, "m", MAKER,
    f"BOCHEM Art. 11120 table: 'Thread: M8 x 1,25' ({BOCHEM_URL})",
)
JACK_MASS = figure(
    "jack total mass (BOCHEM 11120)", 1.650, "kg", LISTING,
    f"neoLab / distributor sheet for BOCHEM 11120 (LLG 9.118 924): 'Gewicht: 1.650,0 g (1,65 kg)' ({SEEN_SEARCH})",
)

JACK_XY = (
    figure("jack centre x", 0.250, "m", FIXTURE, "centred 25.0 cm ahead of the arm-base line"),
    figure("jack centre y", 0.015, "m", FIXTURE, "1.5 cm left of centre so the -y wheel sits comfortably in right-arm reach"),
)
Z_DECK_START = figure(
    "jack deck start height", 0.086, "m", FIXTURE,
    "set with a steel ruler at reset: places the 60 mm sample beaker rim at 146 mm, submerging the 130 mm probe tip by 16 mm",
)
JACK_Z_SIM_MAX = 0.160  # m: prismatic slide upper limit in simulation (well above Z_PROBE_TIP = 130 mm)

PLATE_T = 0.005  # m: top and bottom stainless deck thickness
FOOT_H = 0.004  # m: rubber corner feet under jack_base
BASE_TOP_Z = FOOT_H + PLATE_T  # 9.0 mm above bench
JACK_BASE_MASS = 1.070
JACK_DECK_MASS = 0.350
CARRIER_MASS = 0.130
KNOB_MASS = 0.100  # 1.070 + 0.350 + 0.130 + 0.100 == 1.650 kg

WHEEL_D = figure(
    "adjusting wheel outer diameter", 0.040, "m", STANDARD,
    f"DIN 6336 star/handwheel nominal grip diameter 40 mm for M8 thread; BOCHEM Type 1 'simple adjusting wheel, which can be operated by hand' ({BOCHEM_URL})",
)
WHEEL_T = figure(
    "adjusting wheel axial width", 0.018, "m", ESTIMATE,
    "scaled from DIN 6336 Form K M8 handwheel rim/hub width so parallel finger pads clamp both flat faces cleanly",
)
WHEEL_BOSS_GAP = 0.042  # m: spindle collar spacing between jack's -y plate edge and wheel's +y face
WHEEL_Y = JACK_XY[1] - JACK_L / 2 - WHEEL_BOSS_GAP - WHEEL_T / 2  # -0.116 m

# Two-rhombus pantograph kinematics (link half-span 74 mm inside 160 mm plate, 12 mm plate stack -> 60..275 mm stroke):
# opposing LH/RH M8x1.25 spindle closes trunnions 2.50 mm/rev; across z = 62..98 mm, mean ratio dH/dS ~ 7.2 -> 18.0 mm/rev.
SCISSOR_HALF_LINK = 0.074
LEAD_MM_REV = figure(
    "vertical deck travel per wheel revolution", 18.0, "mm/rev", ESTIMATE,
    "linearised pantograph lead of 74 mm two-tier scissor links on an opposing M8 x 1.25 spindle (2.50 mm/rev x 7.2 mean ratio over 62-98 mm)",
)
LEAD_M_PER_RAD = (LEAD_MM_REV / 1000.0) / (2.0 * math.pi)

WHEEL_HOLD_NM = figure(
    "handwheel self-locking thread drag torque", 0.10, "N m", ESTIMATE,
    "unloaded M8 x 1.25 stainless leadscrew friction torque; exceeds empty-deck back-drive (0.011 N m) by 9x so released heights hold",
)
WHEEL_K_HOLD = 1.50  # N m/rad: capped friction-following spring; K*dt/DAMPING = 1.5*0.05/0.15 = 0.50 (monotonically damped)
WHEEL_GIVE_RAD = WHEEL_HOLD_NM / WHEEL_K_HOLD
WHEEL_DAMPING = 0.15  # N m s/rad: implicit PhysX revolute damping on the free wheel hinge

DECK_SPRING = 45000.0  # N/m: stiff prismatic leadscrew coupling
DECK_DAMPING = 180.0  # N s/m
DECK_MAX_N = JACK_DYN_KG * GRAVITY  # 68.67 N: BOCHEM 11120's published 7 kg dynamic load limit
JACK_FOOT_FRICTION = figure(
    "jack rubber feet friction on bench", 0.85, "-", ESTIMATE,
    "dry EPDM rubber instrument feet on laminate benchtop",
)

# --- Thermo Scientific Nalgene 1201-0050PK 50 mL Griffin Low-Form PP Beakers (ISO/DIN 7056) -------------------------
BEAKER_H = figure(
    "beaker height (50 mL low-form)", 0.060, "m", STANDARD,
    f"ISO/DIN 7056 standard height 60 mm for 50 mL Griffin low-form plastic beaker ('Erfüllt die internationale Norm ISO/DIN 7056', {THERMO_URL}; U.S. Plastic item 76022 lists 2.33 in = 59.2 mm)",
)
BEAKER_OD = figure(
    "beaker outer diameter", 0.048, "m", LISTING,
    f"U.S. Plastic item 76022 for Nalgene 1201-0050 lists 1.86 in (47.2 mm) body OD and 2.01 in (51.1 mm) spout OD ({SEEN_SEARCH}); modelled as 48.0 mm cylinder",
)
BEAKER_WALL = figure(
    "beaker wall and floor thickness", 0.002, "m", ESTIMATE,
    f"derived from 50 mL nominal capacity and U.S. Plastic item 76022 listed 1.78 in (45.2 mm) top ID / 1.54 in (39.1 mm) bottom ID ({SEEN_SEARCH})",
)
BEAKER_FLOOR = BEAKER_WALL
BEAKER_R_OUT = BEAKER_OD / 2.0
BEAKER_R_IN = BEAKER_R_OUT - BEAKER_WALL
BEAKER_MASS_TRUE = figure(
    "beaker true dry mass (Nalgene 1201-0050)", 0.015, "kg", LISTING,
    f"U.S. Plastic item 76022 for Nalgene 1201-0050: 'Weight: 0.033 lbs' = 15.0 g ({SEEN_SEARCH})",
)
BEAKER_MASS = figure(
    "beaker simulated mass", 0.065, "kg", ESTIMATE,
    "solver-stabilised rigid mass declared in fidelity (vs 15 g true dry PP) for jitter-free 20 N parallel-pad grasping",
)
BEAKER_STAVES = 16

IMM_MIN = figure(
    "immersion band minimum depth below beaker rim", 0.020, "m", FIXTURE,
    "upper edge of the 20-40 mm green rinse band (stands in for submerging the bulb and junction in a 40 mL DI-water rinse on dry shared-rig beakers)",
)
IMM_MAX = figure(
    "immersion band maximum depth below beaker rim", 0.040, "m", FIXTURE,
    "lower edge of the 20-40 mm green rinse band, keeping the probe tip at least 18 mm above the 2 mm beaker floor",
)

# --- Hanna Instruments HI1230B Combination pH Electrode (DIN 19262) on fixed retort stand ---------------------------
PROBE_D = figure(
    "pH electrode shaft outer diameter (HI1230B)", 0.012, "m", STANDARD,
    f"DIN 19262 standard 12 mm combination pH electrode shaft; Hanna HI1230B specification: '12 mm (0.47\")' OD ({SEEN_SEARCH})",
)
PROBE_SHAFT_L = figure(
    "pH electrode PEI shaft length (HI1230B)", 0.120, "m", LISTING,
    f"Hanna HI1230B specification: '120 mm (4.72\")' PEI body length ({SEEN_SEARCH})",
)
PROBE_OVERALL_L = figure(
    "pH electrode overall length (HI1230B)", 0.1635, "m", LISTING,
    f"Hanna HI1230B specification: '163.5 mm (6.43\")' overall length with upper BNC strain-relief cap ({SEEN_SEARCH})",
)
PROBE_CAP_D = 0.016
PROBE_CAP_L = PROBE_OVERALL_L - PROBE_SHAFT_L  # 43.5 mm
PROBE_R = PROBE_D / 2.0

Z_PROBE_TIP = figure(
    "fixed pH probe tip height over bench", 0.130, "m", FIXTURE,
    "clamped coaxially over (0.250, +0.015) with a steel ruler at reset: 16 mm inside sample beaker at start, 8 mm above rim at z_deck = 62 mm",
)
Z_CLAMP_ANCHOR = Z_PROBE_TIP + PROBE_SHAFT_L + 0.018  # 0.268 m: bosshead clamp collar on upper cap
STAND_POST_XY = (0.365, JACK_XY[1])  # 50 mm behind jack's +x edge, completely clear of both arms
METER_XY = (0.385, 0.090)

PROBE_MASS = 0.045  # kg: PEI electrode body
PROBE_LAT_K = figure(
    "probe witness lateral hinge stiffness", 8.5, "N m/rad", ESTIMATE,
    "simulator witness spring at z = 268 mm so a 1.8 N horizontal beaker-wall strike deflects the z = 130 mm tip past 4.0 mm",
)
PROBE_LAT_D = 0.45
PROBE_AXIAL_K = figure(
    "probe witness axial slide stiffness", 2500.0, "N/m", ESTIMATE,
    "simulator witness spring so a 3.0 N upward beaker-floor bottoming contact lifts the probe tip by 1.2 mm",
)
PROBE_AXIAL_D = 25.0
PROBE_AXIAL_PRELOAD = PROBE_MASS * GRAVITY / PROBE_AXIAL_K  # +0.177 mm so rest sag under gravity is 0.00 mm

# --- Sample blue mat and rinse start coaster (both 3.0 mm silicone pads on the bench) -------------------------------
MAT_SIZE = (
    figure("blue sample mat side", 0.120, "m", FIXTURE, "120 x 120 mm blue silicone bench pad on the left"),
    0.120,
    figure("silicone pad thickness", 0.003, "m", FIXTURE, "3.0 mm flat silicone bench pad"),
)
MAT_XY = (0.365, 0.205)
RINSE_START_XY = (0.340, -0.195)
COASTER_SIZE = (0.090, 0.090, MAT_SIZE[2])

# --- Scoring thresholds (identical in task.py, task.json, and README.md) --------------------------------------------
DEFLECT_ZERO_M = 0.004  # m: > 4.0 mm lateral probe-tip deflection loses the trial
AXIAL_ZERO_M = 0.0012  # m: >= 1.2 mm witness lift (3.0 N at 2500 N/m) = floor touched probe tip
AXIAL_ZERO_N = 3.0  # N: upward floor-to-tip contact stand-in threshold
TILT_ZERO_DEG = 25.0  # deg: either beaker tilts past 25 deg
SLIDE_ZERO_M = 0.025  # m: jack_base slides > 25 mm from start mark
CLEAR_DECK_Z = Z_PROBE_TIP - BEAKER_H  # 0.070 m: deck must drop below 70.0 mm for 60 mm beaker rim to clear 130 mm tip
UPRIGHT_DEG = 8.0  # deg: upright beaker tolerance at end
MAT_HALF_INSIDE = MAT_SIZE[0] / 2.0 - BEAKER_R_OUT  # 0.036 m: keeps 48 mm OD beaker wholly inside 120 mm blue_mat
COAXIAL_M = 0.012  # m: <= 12.0 mm horizontal offset from probe axis (>= 4.0 mm clear of 44 mm inner wall)
HANDS_CLEAR_M = 0.030  # m: >= 30.0 mm from every hand hull corner to jack, both beakers, and probe
HOLD_STEPS = 20  # 1.0 s at 20 Hz
HOLD_DRIFT_M = 0.002  # m: <= 2.0 mm drift of deck and beakers during the 1.0 s hold


def beaker_com_and_inertia() -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Analytical centre of mass and inertia (kg m^2) of the hollow cylindrical beaker in its base-underside frame."""
    v_floor = math.pi * BEAKER_R_OUT**2 * BEAKER_FLOOR
    v_wall = math.pi * (BEAKER_R_OUT**2 - BEAKER_R_IN**2) * (BEAKER_H - BEAKER_FLOOR)
    m_f = BEAKER_MASS * v_floor / (v_floor + v_wall)
    m_w = BEAKER_MASS - m_f
    zf, zw = BEAKER_FLOOR / 2.0, BEAKER_FLOOR + (BEAKER_H - BEAKER_FLOOR) / 2.0
    zc = (m_f * zf + m_w * zw) / BEAKER_MASS
    ixx_f = m_f * (3.0 * BEAKER_R_OUT**2 + BEAKER_FLOOR**2) / 12.0 + m_f * (zf - zc) ** 2
    izz_f = 0.5 * m_f * BEAKER_R_OUT**2
    hw = BEAKER_H - BEAKER_FLOOR
    ixx_w = m_w * (3.0 * (BEAKER_R_OUT**2 + BEAKER_R_IN**2) + hw**2) / 12.0 + m_w * (zw - zc) ** 2
    izz_w = 0.5 * m_w * (BEAKER_R_OUT**2 + BEAKER_R_IN**2)
    return (0.0, 0.0, round(zc, 5)), (round(ixx_f + ixx_w, 7), round(ixx_f + ixx_w, 7), round(izz_f + izz_w, 7))


def wheel_turns_for_dz(dz_m: float) -> float:
    """Wheel revolutions (positive CW looking along +y = raise, negative CCW = lower) for deck change ``dz_m``."""
    return dz_m / (LEAD_MM_REV / 1000.0)


def summary() -> dict:
    by_kind: dict[str, int] = {}
    for _, _, s, _ in FIGURES.values():
        by_kind[s] = by_kind.get(s, 0) + 1
    return {
        "counts_by_source": by_kind,
        "figures": {n: {"value": v, "unit": u, "source": s, "note": t} for n, (v, u, s, t) in FIGURES.items()},
        "turns_lower_86_to_62mm": round(wheel_turns_for_dz(0.062 - Z_DECK_START), 3),
        "turns_raise_62_to_98mm": round(wheel_turns_for_dz(0.098 - 0.062), 3),
        "total_turns_demo": round(abs(wheel_turns_for_dz(0.062 - Z_DECK_START)) + wheel_turns_for_dz(0.098 - 0.062), 3),
    }
