"""Every figure the Minifix 15 cam-lock shelf task rests on, in one place, each with its source; and the geometry
derived from them.

Source kinds (``FIGURES`` holds every figure with its kind and its source in words; the README's table is built from it):
- ``maker``: published by Wera (355 PZ Stubby PZ 2 x 25 mm, part no. 05008854001), page opened. Häfele's own pages for
  the Minifix 15 housing (art. 262.26.033) and the S100 bolt (art. 262.27.029) answer HTTP 403: figures seen only in their
  search results are estimates, checked against the opened distributor listings.
- ``standard``: European furniture particleboard standard EN 312 (16 mm melamine-faced P2 furniture board).
- ``listing``: a retailer's or distributor's published product specification with URL.
- ``our fixture``: cut size, hole pitch, bench support block, or screwdriver cradle specified by this task.
- ``estimate``: derived figure, with its one-clause derivation.

World frame (the station's): metres from the midpoint between the arm bases, x forward, y left, z up from the bench.
"""

from __future__ import annotations

import math

GRAVITY = 9.81
OVER = 0.0005  # m: loose bodies start 0.5 mm above their support so they settle cleanly before the first rendered frame

MAKER, STANDARD, LISTING, FIXTURE, ESTIMATE = "maker", "standard", "listing", "our fixture", "estimate"

HAFELE_CAM_LISTING = "https://www.homedecorhardware.com/hf-262-26-033.html"
HAFELE_BOLT_LISTING = "https://www.homedecorhardware.com/hf-262-27-029.html"
HAFELE_UK_CAM = (
    "Häfele's page as seen in a web search result, not opened by us (HTTP 403), 2026-11-21: "
    "https://www.hafele.co.uk/en/product/connector-housing-without-rim-zinc-alloy-minifix-15/P-00860559/"
)
HAFELE_UK_BOLT = (
    "Häfele's page as seen in a web search result, not opened by us (HTTP 403), 2026-11-21: "
    "https://www.hafele.co.uk/en/product/connecting-bolt-for-5-mm-holes-with-special-thread-minifix-s100/P-00860582/"
)
WERA_MAKER_PAGE = "https://www.wera.de/en/tools/355-pz-pozidriv-head-stubby-carburetor-screwdriver"
TME_DRV_SEARCH = (
    "distributor listing as shown in a web search result (page not opened, HTTP 403), 2026-11-21: "
    "https://www.tme.eu/en/details/05008854001/"
)

FIGURES: dict[str, tuple[float, str, str, str]] = {}  # name -> (value, unit, source kind, note)


def figure(name: str, value: float, unit: str, source: str, note: str) -> float:
    if source not in (MAKER, STANDARD, LISTING, FIXTURE, ESTIMATE):
        raise ValueError(f"{name}: unknown source kind {source!r}")
    FIGURES[name] = (float(value), unit, source, note)
    return float(value)


# --- 1. Panels & connector hardware (Häfele Minifix 15 art. 262.26.033 + S100 bolt art. 262.27.029) ------------------
PANEL_T = figure(
    "panel thickness (upright cabinet side and shelf)",
    0.016,
    "m",
    STANDARD,
    "EN 312 Type P2 16 mm white/decor melamine-faced furniture chipboard; Häfele 262.26.033 is specified 'For wood thickness from 16 mm'",
)
CAM_BORE_D = figure(
    "cam housing drill hole diameter (Häfele 262.26.033)",
    0.015,
    "m",
    LISTING,
    f"'- Drill hole housing : 15.00 mm' for 'Connector Housing, Hfele Minifix 15, Zinc alloy, Without rim' ({HAFELE_CAM_LISTING})",
)
CAM_BORE_DEPTH = figure(
    "cam housing drilling depth x (16 mm panel)",
    0.0125,
    "m",
    ESTIMATE,
    f"'drilling depth x: 12.5+0.5 mm' for 16 mm wood thickness, as seen in search ({HAFELE_UK_CAM}); leaves 3.5 mm of the 16 mm panel under the cam",
)
DIM_A = figure(
    "edge bore centreline height A from panel face (16 mm panel)",
    0.008,
    "m",
    ESTIMATE,
    f"half the 16.0 mm panel thickness; 'dim. A: 8.0 mm' for 262.26.033 in 16 mm board, as seen in search ({HAFELE_UK_CAM})",
)
DIM_B = figure(
    "cam centre to shelf joining edge B (Häfele S100 262.27.029)",
    0.024,
    "m",
    LISTING,
    f"'24 mm drilling distance' on Häfele 262.27.029 ({HAFELE_BOLT_LISTING}) and 'Distance from centre of Minifix housing to front edge of panel (24 or 34 mm)' ({HAFELE_CAM_LISTING})",
)
EDGE_BORE_D = figure(
    "shelf horizontal bolt-edge bore diameter",
    0.008,
    "m",
    LISTING,
    f"'- Bolt hole : 8.00 mm' on Häfele 262.27.029 ({HAFELE_BOLT_LISTING})",
)
UPRIGHT_DRILL_D = figure(
    "upright face screw-in hole diameter for S100 bolt",
    0.005,
    "m",
    LISTING,
    f"'- For drill hole : 5.00 mm' on Häfele 262.27.029 ({HAFELE_BOLT_LISTING})",
)
BOLT_HEAD_D = figure(
    "S100 connecting bolt head diameter (0.50 mm radial clearance in 8.0 mm bore)",
    0.007,
    "m",
    ESTIMATE,
    f"'Bolt Head: Ø 7 mm' for art. 262.27.029 Minifix S100, as seen in search ({HAFELE_UK_BOLT}); check: the opened listing's '- Bolt hole : 8.00 mm' ({HAFELE_BOLT_LISTING}) takes it with 0.50 mm radial play",
)
BOLT_THREAD_L = figure(
    "S100 connecting bolt screw-in thread length",
    0.011,
    "m",
    ESTIMATE,
    f"'Thread length: 11 mm' for art. 262.27.029, as seen in search ({HAFELE_UK_BOLT})",
)

# Derived & estimated internal connector geometry for B = 24.0 mm
RADIAL_PLAY = (EDGE_BORE_D - BOLT_HEAD_D) / 2.0  # 0.0005 m = 0.50 mm exact radial clearance
BOLT_PROJ = figure(
    "S100 bolt projection from upright face to head tip",
    0.0250,
    "m",
    ESTIMATE,
    "geometry of B = 24.0 mm Minifix S100 bolt: 25.0 mm tip reach puts the 2.8 mm-thick Ø 7.0 mm head across the cam centre when flush and stops 8.5 mm short against the closed Ø 15.0 mm cam wall",
)
BOLT_HEAD_L = 0.0028  # m: axial thickness of the Ø 7.0 mm head (back shoulder at 22.2 mm from upright face)
BOLT_BACK_FROM_SIDE = BOLT_PROJ - BOLT_HEAD_L  # 0.0222 m: +x rear shoulder of bolt head from upright face
BOLT_NECK_D = figure(
    "S100 bolt neck diameter behind Ø 7.0 mm head",
    0.0040,
    "m",
    ESTIMATE,
    "geometry from Ø 7.0 mm S100 head: 4.0 mm neck leaves a 1.5 mm radial undercut shoulder for the cam's twin crescent flanges",
)
BOLT_COLLAR_D = 0.0064  # m: seating shoulder at the upright face (< 8.0 mm bore so panels meet flush)
BOLT_COLLAR_L = 0.0020  # m
BORE_CHAMFER = figure(
    "shelf 8.0 mm bore mouth chamfer",
    0.0008,
    "m",
    ESTIMATE,
    "light 0.8 mm x 45 deg countersink on a drilled 8.0 mm chipboard edge hole so untorn melamine edges guide entry without widening the 8.0 mm bore",
)
BORE_STAVES = 16

CAM_OUTER_R = 0.00745  # m: 14.9 mm zinc body inside 15.0 mm bore
CAM_MASS = figure(
    "Minifix 15 zinc housing mass",
    0.010,
    "kg",
    ESTIMATE,
    "volume of a Ø 15 x 12.5 mm hollow die-cast Zamak housing (~1.5 cm3 at 6.6 g/cm3)",
)
CAM_FRICTION = figure(
    "cam housing dry turning resistance in chipboard bore",
    0.05,
    "N m",
    ESTIMATE,
    "finger-and-stubby turning resistance (~7 N tangential press at Ø 15 mm bore wall with mu ~ 0.35)",
)
CAM_START_DEG = 90.0  # deg CW from open (+x): both cams start 90 deg closed, blocking the 8.0 mm bores
CAM_OPEN_TOL_DEG = 25.0  # deg: |cam_deg| <= 25 deg aligns the 8.2 mm cam throat with the 8.0 mm edge bore
CAM_LOCK_MIN_DEG = 150.0  # deg CW from open: start of self-locking flat
CAM_LOCK_MAX_DEG = 195.0  # deg CW from open: rotational stop of the housing
CAM_LIMITS_DEG = (-25.0, 195.0)  # deg CW from open; relative to built +90 deg pose this is (-115 deg, +105 deg)

# Spiral ramp geometry: relative to cam centre (24.0 mm from shelf front), bolt head back shoulder sits at
# x_c = (24.0 - 22.2) + seam_gap = 1.80 mm + seam_gap. Tapering r_in from 4.30 mm (at 58 deg) to 1.68 mm (on the 158..195
# deg flat) captures seam_gap <= 2.5 mm and draws it flush on 32 kN/m compliant spiral staves (15.1 / 14.5 N clamp per cam at
# flush in the reference run).
SPIRAL_START_DEG = 58.0
SPIRAL_FLAT_DEG = 158.0
SPIRAL_END_DEG = 195.0
SPIRAL_R_OPEN = 0.00430  # m at 58 deg (captures bolt head back shoulder up to 2.5 mm seam gap)
SPIRAL_R_LOCK = 0.00168  # m on [158, 195] deg self-locking dwell flat (15.1 / 14.5 N preload per cam at flush, reference run)
SPIRAL_STIFFNESS = 32000.0  # N/m compliant contact on spiral ramp segments (30.8 N total preload in the reference run; 0.02 / 0.16 mm give under a 10 N pull in the trials)
SPIRAL_DAMPING = 280.0  # N s/m
CAM_DRAW_RANGE = figure(
    "cam eccentric spiral axial pull-in range",
    0.0025,
    "m",
    ESTIMATE,
    "eccentric radial lead (4.30 mm entry minus 1.80 mm flush shoulder = 2.5 mm draw) over a 180 deg clockwise turn of a Ø 15 mm cam",
)

# --- 2. Stubby Pozidriv screwdrivers (Wera 355 PZ, PZ 2 x 25 mm, art. 05008854001) ----------------------------------
DRV_BLADE_L = figure(
    "stubby screwdriver blade length (Wera 355 PZ, 05008854001)",
    0.025,
    "m",
    MAKER,
    f"'Blade length mm: 25' for part no. 05008854001, tip size 'PZ 2' ({WERA_MAKER_PAGE})",
)
DRV_HANDLE_L = figure(
    "stubby screwdriver handle length (Wera 355 PZ, 05008854001)",
    0.054,
    "m",
    MAKER,
    f"'Handle length mm: 54' for part no. 05008854001 ({WERA_MAKER_PAGE})",
)
DRV_BLADE_D = figure(
    "stubby screwdriver round blade diameter (Wera 355 PZ, 05008854001)",
    0.006,
    "m",
    MAKER,
    f"'Blade diameter mm: 6.0' for part no. 05008854001 ({WERA_MAKER_PAGE})",
)
DRV_TOTAL_L = DRV_HANDLE_L + DRV_BLADE_L  # 0.079 m = 79.0 mm
DRV_GRIP_FLAT = figure(
    "stubby Kraftform handle grip width across flats",
    0.024,
    "m",
    ESTIMATE,
    f"scaled from Wera 355 PZ 54 mm compact Kraftform grip waist and 30 mm hexagonal anti-roll bolster ({WERA_MAKER_PAGE})",
)
DRV_BOLSTER_FLAT = 0.030  # m: hexagonal anti-roll bolster width above blade shoulder
DRV_MASS = figure(
    "stubby screwdriver mass (Wera 05008854001)",
    0.037,
    "kg",
    ESTIMATE,
    f"37 g from steel 6 x 25 mm blade (~6 g) plus 54 mm moulded Kraftform handle (~31 g), matching 37.05 g distributor figure ({TME_DRV_SEARCH})",
)
# Driver local frame: origin at handle midpoint (27 mm from butt, 27 mm from bolster face), +z pointing along blade to tip
DRV_BUTT_Z = -DRV_HANDLE_L / 2.0  # -0.027 m
DRV_BOLSTER_Z = DRV_HANDLE_L / 2.0  # +0.027 m
DRV_TIP_Z = DRV_BOLSTER_Z + DRV_BLADE_L  # +0.052 m
DRV_TIP_D = 0.0044  # m: tapered PZ2 tip end diameter entering the Ø 7.2 mm cam recess

# Cam PZ2 cross-recess & axial-torque-only coupling (no lateral or tilt support)
SLOT_RECESS_D = 0.0080  # m: shallow PZ2 counterbore in top of Ø 15 mm cam housing
SLOT_RECESS_DEPTH = figure(
    "cam PZ2 cross-slot recess depth",
    0.0035,
    "m",
    ESTIMATE,
    "3.5 mm PZ2 cross-slot depth in the 3.5 mm zinc top cap of the 12.5 mm-deep housing; shallow enough that an unheld 79 mm stubby topples under gravity",
)
SLOT_DEPTH_MIN = figure(
    "minimum tip engagement depth in cam slot for torque coupling",
    0.0015,
    "m",
    ESTIMATE,
    "lower 1.5 mm of the 3.5 mm PZ2 tapered cross recess where the four driver ribs engage the zinc slot walls",
)
SLOT_FORCE_MIN = figure(
    "minimum axial contact force in cam slot for torque coupling",
    1.5,
    "N",
    ESTIMATE,
    "downward press needed to keep a PZ2 tip seated against cam-out while turning against 0.08-0.30 N m cam resistance",
)
SLOT_RADIAL_TOL = 0.0052  # m: tip inside Ø 11.8 mm conical PZ2 recess
SLOT_TILT_MAX_DEG = 15.0  # deg: tool axis tilt beyond which cross wings disengage
SLOT_CAPTURE_DEG = 22.0  # deg: half-window around each 90 deg PZ2 cross wing
SLOT_CAMOUT_DEG = 55.0  # deg: twist error beyond which the driver cams out
SLOT_K = 1.25  # N m/rad torsional stiffness (satisfies k*dt/2 < SLOT_D < 2*I_eff/dt at 20 Hz)
SLOT_D = 0.045  # N m s/rad torsional damping
SLOT_TAU_MAX = 0.24  # N m axial torque cap (smoothly turns cam through neck slot and spiral ramp with zero handle slip)

# --- 3. Bench fixtures & starting layout (arm-midpoint frame, x forward, y left, z above bench) ----------------------
X_SIDE = figure(
    "upright cabinet side inner mating face, x",
    0.360,
    "m",
    FIXTURE,
    "36.0 cm forward of the arm-base line, inside YAM's < 0.1 mm IK zone",
)
SIDE_W = figure("upright cabinet side width", 0.320, "m", FIXTURE, "32.0 cm along y, centred")
SIDE_H = figure("upright cabinet side height", 0.140, "m", FIXTURE, "14.0 cm tall, clamped square to the bench with rear steel L-brackets")

SHELF_D = figure("shelf depth along x", 0.140, "m", FIXTURE, "14.0 cm from rear edge to joining +x edge")
SHELF_W = figure(
    "shelf width along y",
    0.280,
    "m",
    FIXTURE,
    "28.0 cm wide so the two cam locks at y = +-11.0 cm sit 22.0 cm apart with 3.0 cm side margin",
)
CAM_Y = figure(
    "cam lock and bolt offset from midline, |y|",
    0.110,
    "m",
    FIXTURE,
    "22.0 cm spacing between left and right connectors so one hand steadies the shelf while the other turns a cam (>= 20.0 cm wrist clearance)",
)
CHIPBOARD_DENSITY = figure(
    "melamine chipboard gross density",
    660.0,
    "kg/m3",
    ESTIMATE,
    "EGGER Eurodekor EN 312 P2 declared mean gross density (~660 kg/m3, >= 600 kg/m3 core)",
)
SHELF_MASS = round(SHELF_D * SHELF_W * PANEL_T * CHIPBOARD_DENSITY - 2.0 * 0.004, 4)  # ~0.406 kg wood panel (plus two 10 g cams)

SUPPORT_H = figure(
    "HDPE assembly support block height",
    0.064,
    "m",
    FIXTURE,
    "64.0 mm riser so the 16.0 mm shelf rests level with its mid-plane at z = 72.0 mm, aligned with the upright's bolt axes and clear of the bench for edge grasps",
)
Z_MID = SUPPORT_H + DIM_A  # 0.072 m: height of bolt axes and shelf mid-plane over the bench
Z_SHELF_TOP = SUPPORT_H + PANEL_T  # 0.080 m: shelf and cam top surface over the bench
SUPPORT_FRONT_X = (0.265, X_SIDE)  # m: 260 mm-wide front support directly beneath both cams across the whole slide
SUPPORT_FRONT_W = 0.260  # m
SUPPORT_REAR_X = (0.175, 0.265)  # m: 150 mm-wide centre spine leaving 48 mm underside clearance under both rear shelf wings
SUPPORT_REAR_W = 0.150  # m
MU_SLIDE = figure(
    "shelf on HDPE support block kinetic friction",
    0.22,
    "-",
    ESTIMATE,
    "dry melamine laminate sliding on smooth high-density polyethylene (0.18-0.25 polymer-on-laminate range)",
)
SHELF_GRIP_FRICTION = 0.65  # melamine surface against rubberised gripper pads (support_block uses friction_combine='min' for 0.22)

START_GAP = figure(
    "start seam gap between shelf joining edge and upright side",
    0.042,
    "m",
    FIXTURE,
    "shelf joining edge starts at x = 31.8 cm, 17.0 mm clear in front of the 25.0 mm bolt tips",
)
X_SHELF_START = X_SIDE - START_GAP - SHELF_D / 2.0  # 0.248 m
X_SHELF_FLUSH = X_SIDE - SHELF_D / 2.0  # 0.290 m
CAM_X_IN_SHELF = SHELF_D / 2.0 - DIM_B  # +0.046 m in shelf frame

CRADLE_X = figure("screwdriver cradle centre, x", 0.285, "m", FIXTURE, "28.5 cm forward of arm-base line, clear ahead of the forearm path to the rear shelf wings")
CRADLE_Y = figure("screwdriver cradle centre, |y|", 0.245, "m", FIXTURE, "24.5 cm left/right of midline, 10.5 cm outside the shelf")
CRADLE_YAW_DEG = 45.0  # deg: both cradles placed at 45 deg so handle flats sit at +-45/+-135 deg inside both arms' +-105 deg j6 window
DRV_BLADE_PHASE_DEG = 45.0  # deg: PZ2 cross wings sit at 45 deg to the 4-flat handle waist (so +-45/+-135 deg handle aligns with 0/90 deg cam slots)
CRADLE_SIDE = 0.054  # m: 54 x 54 mm square HDPE holder block
CRADLE_H = figure("screwdriver cradle height", 0.029, "m", FIXTURE, "29.0 mm tall with a 26.0 mm-deep Ø 15.0 mm funnelled shaft well so the 25 mm blade hangs clear and the 30 mm bolster sits flat")
CRADLE_WELL_D = 0.0150  # m
CRADLE_WELL_FLOOR = 0.003  # m above bench

# --- 4. Scoring thresholds (identical in task.py, task.json, README.md) ----------------------------------------------
DROP_ZERO_Z = SUPPORT_H - 0.008  # shelf mid-plane < 56 mm (dropped off support) -> 0 permanently
DRV_BENCH_ZERO_Z = 0.032  # screwdriver handle centre < 32 mm over bench (dropped to bench) -> 0 permanently

SEATED_GAP_MAX = 0.0025  # m: <= 2.5 mm at both bolt lines for 0.25 and 0.50: the cams' draw (bolt heads inside the cam pockets), not a lateral allowance
FLUSH_GAP_MAX = 0.0006  # m: <= 0.6 mm at both bolt lines for 1.00 (0.6 mm feeler gauge fails to enter)
GAP_MIN = -0.0010  # m: numerical floor against overlap
LATERAL_MAX = 0.0020  # m: |y_shelf| <= 2.0 mm alignment with upright (the 7 mm heads in the 8.0 mm bores hold it within 0.5 mm)
TILT_MAX_DEG = 3.0  # deg: shelf flat within 3.0 deg for every nonzero level
CAM_CLAMP_N = 5.0  # N: minimum PhysX clamping force on bolt/side per locked cam (resists 10 N pull on bench)
CRADLE_POS_MAX = 0.012  # m: parked screwdriver tip within 12.0 mm horizontally of cradle well centre
CRADLE_TIP_Z_MAX = 0.020  # m: parked screwdriver tip <= 20.0 mm over bench (inside the 29 mm-tall cradle well)
CRADLE_TILT_MAX_DEG = 15.0  # deg: parked screwdriver within 15.0 deg of upright
HANDS_CLEAR = 0.030  # m: every hand hull corner >= 30.0 mm from shelf, cams, and screwdrivers
HOLD_STEPS = 20  # 1.0 s at 20 Hz
HOLD_DRIFT = 0.0010  # m: shelf moves <= 1.0 mm during the 1.0 s hold


def spiral_r_in(alpha_deg: float) -> float:
    """Inner radial distance (m) of the cam's eccentric crescent wall from the cam pivot at body-open angle ``alpha_deg``."""
    if alpha_deg <= SPIRAL_START_DEG:
        return SPIRAL_R_OPEN
    if alpha_deg >= SPIRAL_FLAT_DEG:
        return SPIRAL_R_LOCK
    u = (alpha_deg - SPIRAL_START_DEG) / (SPIRAL_FLAT_DEG - SPIRAL_START_DEG)
    return SPIRAL_R_OPEN + u * (SPIRAL_R_LOCK - SPIRAL_R_OPEN)


def cam_deg_from_travel(travel_rad: float) -> float:
    """Cam angle in degrees clockwise from open (+x), given hinge travel (rad) about (0, 0, -1) from the +90 deg start."""
    return CAM_START_DEG + math.degrees(float(travel_rad))


def travel_from_cam_deg(cam_deg: float) -> float:
    """Hinge travel (rad) about (0, 0, -1) corresponding to ``cam_deg`` clockwise from open."""
    return math.radians(float(cam_deg) - CAM_START_DEG)


def box_inertia(mass: float, size: tuple[float, float, float]) -> tuple[float, float, float]:
    sx, sy, sz = size
    return (
        mass * (sy * sy + sz * sz) / 12.0,
        mass * (sx * sx + sz * sz) / 12.0,
        mass * (sx * sx + sy * sy) / 12.0,
    )


def driver_inertia() -> tuple[float, float, float]:
    """Principal moments (kg m2) of the 37 g stubby: true tipping Ixx/Iyy; solver axial Izz for 20 Hz step-held torque."""
    ixx = DRV_MASS * (3.0 * (0.013**2) + DRV_TOTAL_L**2) / 12.0
    return (ixx, ixx, 0.0020)


def cam_inertia() -> tuple[float, float, float]:
    """Principal moments (kg m2) of the hinged cam housing (solver Izz sized for 20 Hz step-held torque stability)."""
    ixx = CAM_MASS * (3.0 * (CAM_OUTER_R**2) + CAM_BORE_DEPTH**2) / 12.0
    return (ixx, ixx, 0.0040)


def source_counts() -> dict[str, int]:
    counts = {k: 0 for k in (MAKER, STANDARD, LISTING, FIXTURE, ESTIMATE)}
    for _, _, kind, _ in FIGURES.values():
        counts[kind] += 1
    return counts


def summary() -> dict:
    return {
        "source_counts": source_counts(),
        "radial_play_mm": round(1000.0 * RADIAL_PLAY, 3),
        "closed_cam_stop_gap_mm": round(1000.0 * (BOLT_PROJ - (DIM_B - CAM_OUTER_R)), 2),
        "shelf_mass_kg": SHELF_MASS,
        "cam_limits_rad": [round(travel_from_cam_deg(d), 4) for d in CAM_LIMITS_DEG],
        "figures": {n: {"value": v, "unit": u, "source": s, "note": note} for n, (v, u, s, note) in FIGURES.items()},
    }
