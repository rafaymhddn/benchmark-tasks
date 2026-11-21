"""Every figure the hole-saw quick-change arbor task rests on, in one place, with its true source.

Source kinds (``FIGURES`` holds every figure with its kind and source note):
- ``maker``: opened manufacturer page or official manual PDF (milwaukeetool.com, irwintools.com).
- ``standard``: engineering thread standard (ASME B1.1 5/8"-18 UNF).
- ``our fixture``: station placement, mat, rim guard, and paint alignment marks specified by this setup.
- ``estimate``: derived figure with one-clause derivation (including figures from retailer listings not opened directly).

World frame (station): metres from the midpoint between the arm bases, x forward, y left, z up from the bench.
"""

from __future__ import annotations

import math

GRAVITY = 9.81
OVER = 0.001  # m: loose body spawn gap over its support so it settles cleanly

MAKER, STANDARD, FIXTURE, ESTIMATE = "maker", "standard", "our fixture", "estimate"

MILWAUKEE_ARBOR_URL = "https://www.milwaukeetool.com/Products/49-56-7055"
MILWAUKEE_ARBOR_9100_URL = "https://www.milwaukeetool.com/Products/49-56-9100"
MILWAUKEE_MANUAL_PDF = "https://www.milwaukeetool.com/-/media/PDFs/Objective-Data/Download-Hole-Saw-Instructions.pdf"
MILWAUKEE_SAW_URL = "https://www.milwaukeetool.com/Products/49-56-0147"
IRWIN_VISE_URL = "https://www.irwintools.com/product/226340/drill-press-vises"
ONET_ELEC_URL = "https://www.onetonline.org/link/summary/47-2111.00"
ONET_MAINT_URL = "https://www.onetonline.org/link/summary/49-9071.00"

FIGURES: dict[str, tuple[float, str, str, str]] = {}


def figure(name: str, value: float, unit: str, source: str, note: str) -> float:
    if source not in (MAKER, STANDARD, FIXTURE, ESTIMATE):
        raise ValueError(f"{name}: unknown source kind {source!r}")
    FIGURES[name] = (float(value), unit, source, note)
    return float(value)


# --- 1. Shop vise: IRWIN 226340 4 in. Drill Press Vise (bolted flat on the bench via its slotted base) ---
X_ARBOR = figure("arbor axis x", 0.225, "m", FIXTURE, "22.5 cm ahead of arm bases; top camera sees both +/-y pin holes clear of cup rim")
Y_ARBOR = figure("arbor axis y", -0.015, "m", FIXTURE, "1.5 cm right of midline for comfortable right-hand collar reach")

VISE_JAW_W = figure(
    "vise jaw width (IRWIN 226340)", 0.1016, "m", MAKER,
    f"maker: 'textured 4\" jaws', 'forged iron', 'Slotted base for easy installation and positioning' ({IRWIN_VISE_URL}, opened 2026-11-21)",
)
VISE_OPEN_MAX = figure(
    "vise max jaw opening (IRWIN 226340)", 0.1143, "m", MAKER,
    f"maker: '4-1/2\" jaw capacity' ({IRWIN_VISE_URL}, opened 2026-11-21)",
)
VISE_BASE_L = figure(
    "vise slotted base length", 0.178, "m", ESTIMATE,
    "estimate: 7.0 in cast base length for 4 in drill-press vise body (acmetools/rona catalog index, listing not opened by us)",
)
VISE_BASE_W = figure(
    "vise slotted base width across ears", 0.134, "m", ESTIMATE,
    "estimate: 101.6 mm jaw body plus two 16.2 mm bolt-slot mounting ears",
)
VISE_BASE_H = figure("vise base rail height", 0.014, "m", ESTIMATE, "estimate: 14 mm slotted base rail under jaws")
VISE_JAW_H = figure(
    "vise jaw height above base rail", 0.024, "m", ESTIMATE,
    "estimate: ~1 in throat depth on IRWIN 226340 jaw plates (jaws reach 38 mm over bench)",
)
VISE_JAW_T = 0.022  # m thickness of each jaw block along x
VISE_TOP = VISE_BASE_H + VISE_JAW_H  # 0.038 m over bench

# --- 2. Quick-change arbor: Milwaukee 49-56-7055 (7/16" shank; page gives no thread, length, mass, pins, collar or sheet;
#        5/8" x 18 quote from 49-56-9100; nut-free sliding-collar steps from 49-56-7250 sheet on p. 2 of manual PDF) ---
ARBOR_SHANK_AF = figure(
    "arbor hex shank across flats", 0.01111, "m", MAKER,
    f"maker: 'Large Thread Quick Change Arbor, 7/16\" Shank' ({MILWAUKEE_ARBOR_URL}, opened 2026-11-21; page gives no thread, pins, collar or sheet)",
)
MIN_SAW_D = figure(
    "minimum compatible hole-saw diameter", 0.03175, "m", MAKER,
    f"maker: 'Fits all hole saws 1-1/4 in. and larger' ({MILWAUKEE_ARBOR_URL}, opened 2026-11-21)",
)
THREAD_OD = figure(
    "arbor thread major diameter (5/8-18 UNF)", 0.01588, "m", STANDARD,
    f"ASME B1.1 5/8\"-18 UNF major diameter 15.875 mm; maker: 'Universal fit with all 5/8\" x 18 threaded hole saws' ({MILWAUKEE_ARBOR_9100_URL})",
)
THREAD_PITCH = figure(
    "arbor thread pitch (18 TPI)", 0.0254 / 18.0, "m", STANDARD,
    "ASME B1.1 5/8\"-18 UNF lead: 25.4 mm / 18 = 1.4111 mm per turn",
)
ARBOR_L = figure(
    "arbor overall length", 0.1016, "m", ESTIMATE,
    "estimate: 4.0 in (101.6 mm) bare arbor length (acmetools/calgaryfasteners index for 49-56-7055, listing not opened by us)",
)
ARBOR_MASS = figure(
    "arbor total mass", 0.222, "kg", ESTIMATE,
    "estimate: 0.49 lb (222 g) assembled steel arbor mass (acmetools index for 49-56-7055, listing not opened by us)",
)

PIN_D = figure(
    "drive-pin diameter", 0.00635, "m", ESTIMATE,
    f"estimate: 1/4 in (6.35 mm) hardened drive pins scaled against 15.88 mm thread boss in maker diagram ({MILWAUKEE_MANUAL_PDF})",
)
PIN_BCD = figure(
    "drive-pin bolt-circle diameter", 0.0254, "m", ESTIMATE,
    f"estimate: 1.00 in (25.4 mm) two-pin circle scaled from 15.88 mm 5/8-18 boss in maker diagram ({MILWAUKEE_MANUAL_PDF})",
)
PIN_STROKE = figure(
    "collar and drive-pin vertical stroke", 0.0080, "m", ESTIMATE,
    f"estimate: 8.0 mm axial travel so pins retract 1.0 mm below shoulder and project 7.0 mm above it ({MILWAUKEE_MANUAL_PDF})",
)
COLLAR_D = figure(
    "knurled pin-collar outer diameter", 0.0420, "m", ESTIMATE,
    f"estimate: 42.0 mm knurled collar OD scaled from 25.4 mm pin circle in maker diagram ({MILWAUKEE_MANUAL_PDF})",
)
COLLAR_H = figure(
    "knurled pin-collar sleeve height", 0.0270, "m", ESTIMATE,
    f"estimate: 27.0 mm collar + lower grip skirt height scaled from 42.0 mm collar OD in maker diagram ({MILWAUKEE_MANUAL_PDF})",
)
COLLAR_MASS = figure(
    "sliding collar with pins mass", 0.085, "kg", ESTIMATE,
    "estimate: steel collar sleeve + two pins share (~38 %) of 222 g arbor mass",
)
COLLAR_DETENT_N = figure(
    "collar ball-detent peak axial force", 10.0, "N", ESTIMATE,
    "estimate (assumed): ~10 N spring-ball detent holding collar up or down; maker publishes no force figure",
)

# Arbor vertical stack in bench coordinates (bottom of hex shank at z = 25 mm inside 38 mm vise jaws -> top 126.6 mm):
Z_ARBOR_BOT = 0.0250  # m: 13 mm of 7/16" hex shank clamped in z = 14..38 mm jaws
Z_FLANGE_BOT = 0.0660
Z_COLLAR_LOW = 0.0700  # m: collar underside when fully retracted (collar_z = 0.0; flush at 105 mm shoulder when +8 mm up)
Z_SHOULDER = figure(
    "arbor thread shoulder height over bench", 0.1050, "m", FIXTURE,
    "clamped height setting 101.6 mm arbor bottom 25 mm over bench in 38 mm vise jaws (collar flush with shoulder when up)",
)
Z_THREAD_TOP = Z_SHOULDER + 0.0120  # 0.1170 m: 12 mm of 5/8"-18 thread boss
Z_PILOT_TOP = Z_ARBOR_BOT + ARBOR_L  # 0.1266 m: smooth rounded Ø 6.35 mm pilot guide nose tip
PILOT_D = 0.00635
PIN_RETRACT_BELOW = 0.0010  # m: at collar_z = 0, pin tips sit 1.0 mm below Z_SHOULDER; at +8.0 mm they stand 7.0 mm above

# Bi-stable collar spring: travel 0..8 mm, midpoint 4 mm; peak ~10 N at midpoint, ~4 N hold at ends
COLLAR_HOLD_FRAC = 0.40
COLLAR_K = COLLAR_DETENT_N * (1.0 - COLLAR_HOLD_FRAC) / (PIN_STROKE / 2.0)  # 1500 N/m
COLLAR_BEYOND = COLLAR_HOLD_FRAC * COLLAR_DETENT_N / COLLAR_K  # 0.00267 m past each stop
COLLAR_DAMP = 18.0  # N s/m

# --- 3. Hole saw: Milwaukee 49-56-0147 2-1/2 in (63.5 mm) HOLE DOZER Bi-Metal Hole Saw ---
SAW_OD = figure(
    "hole-saw outer diameter (Milwaukee 49-56-0147)", 0.0635, "m", MAKER,
    f"maker: '2-1/2\" HOLE DOZER™ Bi-Metal Hole Saw', cut diameter '2-1/2 in' ({MILWAUKEE_SAW_URL}, opened 2026-11-21)",
)
SAW_CUT_DEPTH = figure(
    "hole-saw cutting depth", 0.04128, "m", MAKER,
    f"maker: '1-5/8\" Cutting Depth' = 41.275 mm ({MILWAUKEE_SAW_URL}, opened 2026-11-21)",
)
PLATE_T = figure(
    "hole-saw steel backplate thickness", 0.0050, "m", ESTIMATE,
    "estimate: 5.0 mm (~3/16 in) tapped steel backing plate giving 47.6 mm overall cup height with 41.3 mm cutting depth",
)
GUARD_ADD_H = 0.0013  # m: PVC U-channel edge guard adds 1.3 mm over tooth tips
SAW_H = figure(
    "hole-saw overall cup height with rim guard", 0.0476, "m", ESTIMATE,
    "estimate: 41.3 mm cutting depth + 5.0 mm backplate + 1.3 mm PVC tooth-rim guard = 47.6 mm (1-7/8 in)",
)
SAW_WALL_T = 0.0022  # m cylindrical barrel wall thickness
GUARD_OD = 0.0646  # m outer diameter of smooth PVC rim guard at top 4 mm
SAW_MASS = figure(
    "hole-saw mass with PVC rim guard", 0.195, "kg", ESTIMATE,
    "estimate: 186 g net cup mass (0.41 lb distributor catalog index, listing not opened by us) + 9 g PVC rim strip",
)
HOLE_D = figure(
    "backplate drive-pin hole width", 0.0078, "m", ESTIMATE,
    "estimate: 7.8 mm chamfered drive-pin holes on 25.4 mm circle (0.725 mm radial clearance per side over 6.35 mm pins)",
)
BORE_CLEAR_HALF = 0.0082  # m: half-width of center thread-bore clearance box around Ø 15.6 mm thread boss

# Starting mat for the hole saw (backplate down, guarded rim up)
X_MAT = figure("hole-saw start mat x", 0.225, "m", FIXTURE, "aligned in x with the arbor, 18.0 cm to its left")
Y_MAT = figure("hole-saw start mat y", 0.165, "m", FIXTURE, "in front of the left arm, clear of the vise by 69 mm")
MAT_SIZE = (0.120, 0.120, 0.003)

# --- 4. Helical thread stand-in & shoulder stop geometry ---
STOP_DEG = figure(
    "clockwise thread travel to hard shoulder stop", 415.0, "deg", ESTIMATE,
    "estimate: simulation's 1.153-turn CW tight stop placing holes 55 deg past +/-y pins; real trial records bought pair's back-off (20-140 deg)",
)
ALIGN_DEG = figure(
    "pin-alignment thread angle", 360.0, "deg", ESTIMATE,
    "estimate: 55.0 deg short of simulation's +415.0 deg stop aligns both holes over +/-y pins (gap 0.22 mm <= 0.60 mm)",
)
BACKOFF_DEG = STOP_DEG - ALIGN_DEG  # 55.0 deg
THREAD_DRAG_NM = figure(
    "5/8-18 thread rotational drag", 0.06, "N m", ESTIMATE,
    "estimate (assumed): dry steel 5/8-18 thread running torque; maker publishes no torque figure",
)
THREAD_SPRING_K = 12.0  # N m/rad on each serial carrier hinge (0.06 N m / 12 = 0.29 deg yield << 45 deg kit rule)
THREAD_DAMP = 0.04  # N m s/rad
UNTHREAD_DEG = -3.0  # deg: turning CCW back past 0 deg releases w_thread
HINGE_LOWER_RAD = math.radians(-10.0)
HINGE_UPPER_RAD = math.radians(STOP_DEG / 3.0)  # 138.333 deg per serial hinge (< 180 deg PhysX limit)

Z_ENTRY = Z_SHOULDER + THREAD_PITCH * (STOP_DEG / 360.0)  # 0.095627 m (1.63 mm above Z_SHOULDER at 0 deg catch)
SLIDE_PLAY = 0.00006  # m axial stop window on cup_z (gives 0.16 mm gap at +360 deg)
HIDDEN_MASS = 0.050  # kg per hidden thread-carrier link for crisp PhysX constraint conditioning

# Capture tolerances when lowering hole_saw coaxially onto retracted arbor thread entry
CATCH_XY_MAX = 0.0035  # m coaxial radial tolerance (pilot nose guides bore in)
CATCH_TILT_DEG = 6.5  # deg max tilt from vertical
CATCH_Z_LOW = -0.0004  # m below Z_SHOULDER
CATCH_Z_HIGH = 0.0028  # m above Z_SHOULDER (covers Z_ENTRY = +1.63 mm)
CATCH_COLLAR_MAX = 0.0020  # m: collar must be retracted (<= 2.0 mm) for cup to reach thread entry

# --- 5. Scoring constants (identical in task.py, task.json, and README.md) ---
DROP_BELOW_BENCH = -0.010  # m: whole-trial zero if hole_saw drops below bench level
RETRACTED_AT = 0.0020  # m: collar_z <= 2.0 mm (>= 6.0 mm pulled down from 8.0 mm top) counts as retracted
THREADED_GAP_MAX = 0.0020  # m: backplate within 2.0 mm of shoulder counts as threaded on (0.25 level)
SEATED_GAP_MAX = figure(
    "seated max shoulder gap (feeler-gauge check)", 0.0006, "m", FIXTURE,
    "0.6 mm feeler gauge check: passes backed-off +360 deg (0.22 mm gap), rejects half-threaded +180 deg (0.92 mm gap)",
)
ALIGN_WINDOW_DEG = figure(
    "pin-hole alignment window", 8.0, "deg", ESTIMATE,
    "estimate: +/-8 deg angular window around 0/180 deg where 6.35 mm rounded-tip pins enter 7.8 mm chamfered holes",
)
LOCKED_COLLAR_MIN = figure(
    "locked collar travel threshold", 0.0065, "m", FIXTURE,
    "collar within 1.5 mm of its 8.0 mm upper stop (>= 6.5 mm up, drive pins >= 5.5 mm above shoulder through 5.0 mm backplate)",
)
HANDS_CLEAR = 0.030  # m: every hand hull corner >= 30 mm clear of hole_saw and collar
STILL_LIN = 0.010  # m/s
STILL_ANG = 0.10  # rad/s
HOLD_STEPS = 20  # 1.0 s at 20 Hz


def gap_for_yaw_deg(yaw_deg: float) -> float:
    """Axial gap (m) between hole_saw backplate underside and Z_SHOULDER at CW thread travel ``yaw_deg``."""
    return THREAD_PITCH * max(0.0, STOP_DEG - float(yaw_deg)) / 360.0


def phase_err_deg(world_yaw_rad: float) -> float:
    """Angular offset (deg, 0..90) of the hole saw's +/-y pin holes from the arbor's +/-y drive pins."""
    d = math.degrees(float(world_yaw_rad)) % 180.0
    return min(d, 180.0 - d)
