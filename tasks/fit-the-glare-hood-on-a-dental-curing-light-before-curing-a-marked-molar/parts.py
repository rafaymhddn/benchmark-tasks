"""Every figure the dental curing-light hood task rests on, in one place, each with its true source; and the geometry
derived from them.

Source kinds stored in ``FIGURES`` (the README's key-figure table is generated from this registry):
- ``maker``: the manufacturer's published manual opened directly (Guilin Woodpecker Medical Instrument Co., Ltd.,
  Woodpecker LED.B User's Manual).
- ``standard``: international dental / photobiological safety standard (ISO 10650:2018, IEC 62471, EN 207).
- ``listing``: a dental distributor's published product sheet opened directly (Keverőcsőr / Fair-Trade Dental for
  Woodpecker LED-B and Dochem 1F0046-1 oval light hood; Dent Zar for the 28-tooth demonstration typodont).
- ``our fixture``: bench layout coordinates, U-cradle, slotted hood stand, and molar target mark set by this task.
- ``estimate``: derived figure with a one-clause geometric or mechanical derivation.

Station frame: metres from the midpoint between the two arm bases (left y = +0.305, right y = -0.305),
x forward, y left, z up from the bench top (``TABLE_TOP['station'] = 0.7475 m``).
"""

from __future__ import annotations

import math

import numpy as np

GRAVITY = 9.81
OVER = 0.001  # m: loose bodies spawn 1 mm above their support so they settle cleanly at reset

MAKER, STANDARD, LISTING, FIXTURE, ESTIMATE = "maker", "standard", "listing", "our fixture", "estimate"

LEDB_MANUAL_P5 = "https://www.manualslib.com/manual/2978412/Woodpecker-Led-B.html?page=5"
LEDB_MANUAL_P6 = "https://www.manualslib.com/manual/2978412/Woodpecker-Led-B.html?page=6"
LEDB_MANUAL_P7 = "https://www.manualslib.com/manual/2978412/Woodpecker-Led-B.html?page=7"
LEDB_LISTING = "https://www.keverocsor.hu/led-b"
FIBER_LISTING = "https://www.keverocsor.hu/index.php?route=product/list&keyword=LED-B"
HOOD_LISTING = "https://www.keverocsor.hu/index.php?route=product/list&keyword=1F0046-1"
TYPODONT_LISTING = "https://dentzar.com/shop/adult-standard-dental-teaching-study-typodont-teeth-model/"

FIGURES: dict[str, tuple[float, str, str, str]] = {}


def figure(name: str, value: float, unit: str, source: str, note: str) -> float:
    if source not in (MAKER, STANDARD, LISTING, FIXTURE, ESTIMATE):
        raise ValueError(f"{name}: unknown source kind {source!r}")
    FIGURES[name] = (float(value), unit, source, note)
    return float(value)


def _rx(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# --- 1. Woodpecker LED.B Cordless Dental Curing Light (wand + optical fiber + power button) ----------------------------
WAVE_MIN_NM = figure(
    "LED.B emission wavelength lower bound",
    420.0,
    "nm",
    MAKER,
    f"Woodpecker LED.B User Manual sec. 4.3: 'Blue light', '420nm to 480nm' ({LEDB_MANUAL_P5}); "
    f"listing: 'Hullámhossz: 420–480 nm' ({LEDB_LISTING})",
)
WAVE_MAX_NM = figure(
    "LED.B emission wavelength upper bound",
    480.0,
    "nm",
    MAKER,
    f"Woodpecker LED.B User Manual sec. 4.3: '420nm to 480nm' ({LEDB_MANUAL_P5})",
)
IRRADIANCE_MIN = figure(
    "LED.B radiant exitance lower bound",
    1000.0,
    "mW/cm2",
    LISTING,
    f"listing: 'Fényerősség: 1000–1200 mW/cm²' ({LEDB_LISTING}); manual sec. 4.3 prints '850mW/cm2~1000mW/cm2' ({LEDB_MANUAL_P5})",
)
IRRADIANCE_MAX = figure(
    "LED.B radiant exitance upper bound",
    1200.0,
    "mW/cm2",
    LISTING,
    f"listing: 'Fényerősség: 1000–1200 mW/cm²' ({LEDB_LISTING}); within ISO 10650:2018 4,000 mW/cm2 cap (380-515 nm)",
)
FAST_MODE_S = figure(
    "LED.B shortest preset timer mode",
    5.0,
    "s",
    MAKER,
    f"Woodpecker LED.B User Manual sec. 5.3: four solidification time modes '5, 10, 15, 20 seconds' ({LEDB_MANUAL_P6})",
)
PPE_OD_MIN = figure(
    "operator blue-blocking safety eyewear minimum optical density (400-500 nm)",
    4.0,
    "OD",
    STANDARD,
    "EN 207 / ANSI Z136.1 orange blue-blocking safety glasses rated OD 4+ across 400-500 nm against IEC 62471 retinal hazard",
)

WAND_TOTAL_L = figure(
    "curing light overall length with installed optical fiber",
    0.260,
    "m",
    MAKER,
    f"Woodpecker LED.B User Manual sec. 4.5: '31mm×34mm×260mm' ({LEDB_MANUAL_P5}); listing: '260 mm × 31 mm × 34 mm' ({LEDB_LISTING})",
)
WAND_W = figure(
    "curing light barrel width",
    0.031,
    "m",
    MAKER,
    f"Woodpecker LED.B User Manual sec. 4.5: '31mm×34mm×260mm' ({LEDB_MANUAL_P5})",
)
WAND_H = figure(
    "curing light barrel height",
    0.034,
    "m",
    MAKER,
    f"Woodpecker LED.B User Manual sec. 4.5: '31mm×34mm×260mm' ({LEDB_MANUAL_P5})",
)
WAND_MASS = figure(
    "curing light net mass",
    0.145,
    "kg",
    MAKER,
    f"Woodpecker LED.B User Manual sec. 4.6: '145g' ({LEDB_MANUAL_P5}); listing: 'Súly: 145 g' ({LEDB_LISTING})",
)

FIBER_D = figure(
    "exposed curved glass optical fiber uniform diameter",
    0.008,
    "m",
    ESTIMATE,
    f"uniform 8.0 mm exposed glass light-guide rod ahead of the internal 12 mm metal nose connector ({FIBER_LISTING}), matching 8-12 mm hood bore ({HOOD_LISTING})",
)
FIBER_BEND_DEG = figure(
    "optical fiber distal bend angle",
    55.0,
    "deg",
    ESTIMATE,
    "scaled from Woodpecker LED.B side profile against the 260 mm overall length",
)
FIBER_STRAIGHT_L = figure(
    "exposed optical fiber straight base length before the bend",
    0.032,
    "m",
    ESTIMATE,
    "scaled from Woodpecker LED.B side profile so the 4 mm hood collar travels 22 mm with 3 mm nose clearance",
)
FIBER_BEND_R = figure(
    "optical fiber centerline bend radius",
    0.044,
    "m",
    ESTIMATE,
    "circular 55 deg elbow radius giving 36.0 mm forward reach so 192 mm body + 68.0 mm fiber = 260.0 mm overall",
)
FIBER_BEND_RAD = math.radians(FIBER_BEND_DEG)
FIBER_PROJ_L = FIBER_STRAIGHT_L + FIBER_BEND_R * math.sin(FIBER_BEND_RAD)  # 0.06804 m
BODY_L = WAND_TOTAL_L - FIBER_PROJ_L  # 0.19196 m (~192.0 mm handpiece body)
NOSE_X = BODY_L / 2.0  # +0.09598 m in wand frame (origin at handpiece body center)
ELBOW_X = NOSE_X + FIBER_STRAIGHT_L  # +0.12798 m: where straight base meets the 55 deg bend
ARC_CENTER_LOCAL = np.array([ELBOW_X, 0.0, -FIBER_BEND_R])  # center of curvature of the -z_w bend in wand frame

SEAT_TRAVEL = figure(
    "hood seating travel along the straight base of the optical fiber",
    0.022,
    "m",
    ESTIMATE,
    "from the tangent of the 55 deg bend (0 mm) to 6 mm ahead of the handpiece nose (22 mm) along the 32 mm straight base",
)
SEAT_MIN_M = figure(
    "minimum seated travel of the hood on the straight fiber base",
    0.019,
    "m",
    ESTIMATE,
    "within 3 mm of the 22 mm stop at the bottom of the optical fiber (Woodpecker Manual sec. 5.2)",
)

BTN_X_LOCAL = 0.054  # m ahead of handpiece midpoint (42 mm behind nose, top face +z_w)
BTN_R = 0.0065  # m: 13 mm button cap diameter
BTN_H = 0.0050  # m: raised silicone dome cap height above dorsal bezel so full 2.5 mm stroke never bottoms on bezel
BTN_TRAVEL = figure(
    "power button press travel",
    0.0025,
    "m",
    ESTIMATE,
    "tactile dome switch stroke on the handpiece top face",
)
BTN_TRIGGER_M = figure(
    "power button actuation threshold",
    0.0012,
    "m",
    ESTIMATE,
    "48 % of the 2.5 mm tactile switch travel",
)
BTN_FORCE_N = figure(
    "power button full-travel spring force",
    2.5,
    "N",
    ESTIMATE,
    "tactile silicone dome switch actuation force (1,000 N/m over 2.5 mm)",
)
BTN_SPRING = BTN_FORCE_N / BTN_TRAVEL  # 1000 N/m
BTN_DAMPING = 4.0  # N s/m
BEAM_STEPS = 70  # 3.5 s at 20 Hz, standing in for the 5 s fast timer mode
BEAM_S = figure(
    "simulated curing cycle duration per button press",
    3.5,
    "s",
    ESTIMATE,
    "70 steps at 20 Hz standing in for the LED.B 5 s fast solidification timer mode",
)


# --- 2. Oval Orange Glare Hood (Dochem SKU 1F0046-1 / Woodpecker replacement light hood) ------------------------------
HOOD_W = figure(
    "orange light hood oval major width",
    0.072,
    "m",
    LISTING,
    f"Dochem SKU 1F0046-1 listing: 'Fényvédő lemezke Led polimerizációs lámpához (72x62mm), ovális, 8-12 mm-es szálhoz' ({HOOD_LISTING})",
)
HOOD_H = figure(
    "orange light hood oval minor height",
    0.062,
    "m",
    LISTING,
    f"Dochem SKU 1F0046-1 listing: '72x62mm, ovális' ({HOOD_LISTING})",
)
HOOD_T = figure(
    "orange light hood shield wing thickness",
    0.003,
    "m",
    ESTIMATE,
    "moulded orange blue-blocking acrylic/PS oval plate thickness, pinched directly with no added boss",
)
COLLAR_L = figure(
    "hood rubber-lined grommet collar axial length",
    0.004,
    "m",
    ESTIMATE,
    "short 4.0 mm elastomer-lined grommet hub centred on the 3.0 mm shield plate",
)
COLLAR_OD = figure(
    "hood grommet collar outer diameter",
    0.016,
    "m",
    ESTIMATE,
    "3.8 mm wall around the 8.4 mm grommet bore",
)
HOOD_BORE_D = figure(
    "hood rubber-lined grommet inner bore diameter",
    0.0084,
    "m",
    ESTIMATE,
    f"0.4 mm lead-in clearance over the 8.0 mm glass rod within the listed 8-12 mm light-guide fit ({HOOD_LISTING})",
)
HOOD_TRUE_MASS = figure(
    "orange light hood true mass",
    0.009,
    "kg",
    ESTIMATE,
    "72 x 62 x 3 mm PS oval plate (1.05 g/cm3) plus 4 mm rubber grommet",
)
HOOD_SIM_MASS = figure(
    "orange light hood simulated mass",
    0.025,
    "kg",
    ESTIMATE,
    "solver mass carried in PhysX at 240 Hz so a two-finger pinch on the 3.0 mm wing stays chatter-free",
)

SEAT_FORCE_N = figure(
    "hood rubber-grommet sliding friction along the 8.0 mm glass rod",
    6.0,
    "N",
    ESTIMATE,
    "elastomer grommet interference drag along the 8.0 mm rod (10x an unheld 145 g wand's 0.40 N cradle slip, under the 17.6 N 3 mm wing pinch)",
)
SEAT_SPRING_K = 4000.0  # N/m: capped following spring on seat_s -> give = 6.0 / 4000 = 1.5 mm
SEAT_GIVE_M = SEAT_FORCE_N / SEAT_SPRING_K
SEAT_DAMPING = 25.0  # N s/m

ELBOW_DRAG_NM = figure(
    "curved elbow guide drag while turning the hood 55 deg around the bend",
    0.03,
    "N m",
    ESTIMATE,
    "grommet alignment drag over the 44 mm radius bend; rest follows within 0.9 deg (well inside the 45 deg drive rule)",
)
ELBOW_SPRING_K = 2.0  # N m/rad -> give = 0.015 rad = 0.86 deg << 45 deg!
ELBOW_GIVE_RAD = ELBOW_DRAG_NM / ELBOW_SPRING_K
ELBOW_DAMPING = 0.08  # N m s/rad
ELBOW_UNLOCK_DEG = 6.0  # bend_h must be within 6 deg of straight (-55 deg travel) before seat_s unlocks

ENTRY_RAD_M = figure(
    "tip entry coaxial position tolerance",
    0.0028,
    "m",
    ESTIMATE,
    "hood grommet bore center within 2.8 mm of the distal 8.0 mm fiber tip lead-in chamfer to start threading",
)
ENTRY_ANG_DEG = figure(
    "tip entry coaxial angular tolerance",
    12.0,
    "deg",
    ESTIMATE,
    "hood rubber grommet bore axis within 12.0 deg of the 55 deg distal tip tangent, approaching from ahead of the tip",
)
ENTRY_STEPS = 2  # consecutive 20 Hz steps aligned at the tip to engage w_hood_on


# --- 3. 1:1 Adult 28-Tooth Demonstration Typodont Jaw Model (hinged upper arch propped open 45 mm) --------------------
JAW_D = figure(
    "typodont jaw model closed front-to-back length",
    0.095,
    "m",
    LISTING,
    f"Dent Zar 28-tooth typodont listing: 'Approximately 9.5 cm x 6.5 cm x 5 cm' ({TYPODONT_LISTING})",
)
JAW_W = figure(
    "typodont jaw model arch width",
    0.065,
    "m",
    LISTING,
    f"Dent Zar 28-tooth typodont listing: 'Approximately 9.5 cm x 6.5 cm x 5 cm' ({TYPODONT_LISTING})",
)
JAW_CLOSED_H = figure(
    "typodont jaw model closed height",
    0.050,
    "m",
    LISTING,
    f"Dent Zar 28-tooth typodont listing: 'Approximately 9.5 cm x 6.5 cm x 5 cm' ({TYPODONT_LISTING})",
)
MOUTH_OPEN_M = figure(
    "typodont hinged upper arch propped inter-incisal opening",
    0.045,
    "m",
    ESTIMATE,
    "adult clinical mouth opening (45 mm between lower and upper incisors), overhanging posterior molars so a curved tip is needed",
)
LOWER_ARCH_H = 0.025  # m: lower gum + tooth occlusal plane height above bench
MOLAR_CROWN = figure(
    "lower right first molar (#46) crown width",
    0.011,
    "m",
    ESTIMATE,
    "anatomical human mandibular first molar mesiodistal crown width (11.0 x 10.5 mm)",
)
MARK_D = figure(
    "blue uncured filling mark diameter on molar #46",
    0.008,
    "m",
    FIXTURE,
    "8.0 mm high-contrast cobalt-blue composite target disc centred on lower right first molar #46",
)


# --- 4. Bench layout (our fixture) ------------------------------------------------------------------------------------
WAND_START_XY = (0.245, -0.095)
WAND_START_Z = figure(
    "wand barrel centerline height in cradle",
    0.068,
    "m",
    FIXTURE,
    "supported on two open U-saddles so fingers clear underneath by 36 mm and the angled tip clears the bench by 59 mm",
)
WAND_ROLL_DEG = 28.0  # wand +z_w (button normal) is (cos 28 deg, 0, sin 28 deg): rolled toward +x so fiber bends toward -x/-z
_C2 = (math.cos(math.radians(WAND_ROLL_DEG)), 0.0, math.sin(math.radians(WAND_ROLL_DEG)))
R_WAND_START = np.column_stack([[0.0, 1.0, 0.0], [-_C2[2], 0.0, _C2[0]], list(_C2)])

CRADLE_FRICTION = figure(
    "wand-on-cradle friction coefficient",
    0.28,
    "-",
    ESTIMATE,
    "smooth ABS handpiece shell on smooth ABS/PTFE saddle (0.40 N axial slip threshold on the 145 g wand)",
)
CRADLE_SPAN_Y = 0.140  # m between the two saddle supports along y (126 mm clear grip bay on the 192 mm barrel)
CRADLE_W_X = 0.056  # m across x

HOOD_START_XY = (0.245, 0.180)
HOOD_START_Z = figure(
    "orange light hood center height in its tray slot",
    0.048,
    "m",
    FIXTURE,
    "bottom 4 mm of the 72 mm major axis rests in the 17 mm-high slot floor so 68 mm stands clear for a 3.0 mm wing pinch",
)
# In hood local frame: +x_h is bore outward axis (thickness 3 mm along x_h), +y_h is 72 mm major axis, +z_h is 62 mm minor axis.
# In hood_slot at start: x_h = +y (bore facing along y), y_h = +z (72 mm major axis vertical), z_h = +x.
R_HOOD_START = np.column_stack([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])

JAW_XY = (0.340, 0.032)
JAW_YAW_DEG = 52.0  # turned +52 deg so the 45 mm front mouth opening faces (-0.62, -0.79) toward the curing wand
R_JAW = _rz(JAW_YAW_DEG)
# Patient's lower right first molar #46 in jaw local frame (+y_j is patient's right):
MOLAR_LOCAL = np.array([-0.012, 0.021, LOWER_ARCH_H])


# --- 5. Scoring limits (identical in task.py, task.json, README.md) ---------------------------------------------------
CURE_XY_TOL = figure("cure tip horizontal tolerance from molar #46 center", 0.008, "m", FIXTURE, "within the 8.0 mm fiber tip / mark radius")
CURE_Z_MIN = 0.001  # m above molar occlusal surface
CURE_Z_MAX = figure("cure tip maximum height above molar #46", 0.012, "m", FIXTURE, "1 to 12 mm above the marked molar crown")
CURE_TILT_DEG = figure("cure tip maximum angle from downward vertical", 22.0, "deg", FIXTURE, "beam aimed squarely onto resin (Manual sec. 7.1.1)")
CURE_HOLD_S = figure("required shielded blue-beam dwell on molar #46", 2.5, "s", FIXTURE, "50 steps at 20 Hz within the 3.0 s cycle")
CURE_STEPS_REQ = 50  # 2.5 s at 20 Hz

CRADLE_POS_TOL = figure("returned wand barrel tolerance from cradle center", 0.015, "m", FIXTURE, "15 mm of cradle saddle midpoint")
CRADLE_TILT_DEG = figure("returned wand barrel tilt tolerance from horizontal +y", 15.0, "deg", FIXTURE, "resting level across both cradle saddles")
HANDS_CLEAR_M = figure("minimum hand clearance from wand, hood, and jaw at trial end", 0.030, "m", FIXTURE, "30 mm from every hand hull corner")
REST_HOLD_S = 1.0  # 20 steps at 20 Hz
REST_STEPS = 20
REST_MOVE_M = 0.0015  # 1.5 mm position stability window (immune to jointed-body velocity chatter)
REST_TURN_DEG = 0.8  # deg angular stability window


# --- 6. Exact fiber centerline & pose helpers -------------------------------------------------------------------------
def fiber_frame_local(bend_deg: float, seat_m: float) -> tuple[np.ndarray, np.ndarray]:
    """(position, rotation) on the optical fiber in ``wand``'s frame at elbow angle ``bend_deg`` (55 = distal tip,
    0 = straight base) and straight slide ``seat_m`` (0 at bend start .. 0.022 near nose).
    Columns of rotation: +x along outward rod tangent, +y along bend axis, +z outward from bend center."""
    ph = math.radians(bend_deg)
    pos = np.array([
        ELBOW_X - seat_m + FIBER_BEND_R * math.sin(ph),
        0.0,
        -FIBER_BEND_R + FIBER_BEND_R * math.cos(ph),
    ])
    return pos, _ry(bend_deg)


TIP_LOCAL_POS, TIP_LOCAL_ROT = fiber_frame_local(FIBER_BEND_DEG, 0.0)
TIP_TANGENT_LOCAL = TIP_LOCAL_ROT[:, 0]  # (+cos 55, 0, -sin 55) pointing outward from distal tip


def wand_start_pose(top: float) -> tuple[np.ndarray, np.ndarray]:
    return np.array([WAND_START_XY[0], WAND_START_XY[1], top + WAND_START_Z]), R_WAND_START.copy()


def hood_start_pose(top: float) -> tuple[np.ndarray, np.ndarray]:
    return np.array([HOOD_START_XY[0], HOOD_START_XY[1], top + HOOD_START_Z]), R_HOOD_START.copy()


def molar_world(top: float) -> np.ndarray:
    return np.array([JAW_XY[0], JAW_XY[1], top]) + R_JAW @ MOLAR_LOCAL


def wand_cure_pose(top: float, hover_z: float = 0.0055) -> tuple[np.ndarray, np.ndarray]:
    """Wand pose that places the distal fiber tip ``hover_z`` m directly above molar #46, aimed ~18 deg from -z, with the
    curved rod entering through the 45 mm front mouth opening and the 72 x 62 mm hood outside the incisors."""
    d_rot = _rz(-32.0) @ _rx(-21.0) @ _ry(-(90.0 - WAND_ROLL_DEG))
    r_wc = d_rot @ R_WAND_START
    goal_tip = molar_world(top) + np.array([0.0, 0.0, hover_z])
    p_wc = goal_tip - r_wc @ TIP_LOCAL_POS
    return p_wc, r_wc


def summary() -> dict:
    counts: dict[str, int] = {}
    for _, _, kind, _ in FIGURES.values():
        counts[kind] = counts.get(kind, 0) + 1
    return {
        "counts_by_source": counts,
        "total_figures": len(FIGURES),
        "wand_body_and_fiber_mm": [round(1000 * BODY_L, 2), round(1000 * FIBER_PROJ_L, 2), round(1000 * WAND_TOTAL_L, 2)],
        "tip_local_mm": [round(1000 * float(v), 2) for v in TIP_LOCAL_POS],
        "figures": {k: {"value": v, "unit": u, "source": s, "note": n} for k, (v, u, s, n) in FIGURES.items()},
    }
