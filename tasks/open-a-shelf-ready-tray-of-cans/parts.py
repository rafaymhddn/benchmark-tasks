"""Single source of truth for the shelf-ready tray task: named products, presets, dimensions, masses, frictions,
perforation forces with their sources, and primitive shape builders.

Every dimension, mass, friction, force and angle that the starting state, mechanism, demo, rule tests or checks rest
on lives here, tagged with its source (maker, listing, or estimate with its derivation; README "Key figures").

Presets (selected by environment variable):
- ``SHELF_TRAY_PRESET``: ``"dieline"`` (default: the tray cut from 3 mm single-wall B-flute board to
  ``tray_template.svg``) or ``"retail_srp"`` (a store tray, estimated at 3.2 mm board; no maker page found).
- ``SHELF_TRAY_GRID``: ``"3x2"`` (6 cans: 3 across y by 2 deep along x, default) or ``"4x3"`` (12 cans: 4 by 3).
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import os

import numpy as np
from mechkit import box, cylinder

GRAVITY = 9.81  # m/s^2
CONTACT_THIN = 0.0008  # m: collider contact offset for 3 mm corrugated walls and steel cans
OVER = 0.001  # m: loose bodies start 1 mm above their resting surface so frame 0 renders cleanly


@dataclass(frozen=True)
class CanSpec:
    """Published commercial food-can specification (SI: m, kg)."""

    label: str
    diameter: float
    height: float
    mass: float
    friction: float
    diameter_source: str
    mass_source: str
    friction_source: str


@dataclass(frozen=True)
class TrayPreset:
    """Shelf-ready corrugated tray specification (SI: m, kg, N, deg)."""

    label: str
    board_t: float
    panel_grip_t: float
    lip_h: float
    tray_h: float
    header_h: float
    grammage: float  # kg/m^2 of board
    tray_friction: float
    source_kind: str
    source_note: str


CAN_SPEC = CanSpec(
    label="Mutti Polpa Finely Chopped Tomatoes 400 g steel can (unopened)",
    diameter=0.0730,
    height=0.1100,
    mass=0.454,
    friction=0.45,
    diameter_source="listing",
    mass_source="listing",
    friction_source="estimate",
)

TRAY_PRESETS: dict[str, TrayPreset] = {
    "dieline": TrayPreset(
        label="Tray cut from 3 mm single-wall B-flute kraft board to tray_template.svg",
        board_t=0.0030,
        panel_grip_t=0.0060,
        lip_h=0.0280,
        tray_h=0.1300,
        header_h=0.0180,
        grammage=0.405,
        tray_friction=0.35,
        source_kind="maker",
        source_note="board: maker's data sheet, 3 mm and 405 g/m^2; sizes: tray_template.svg (ours by construction)",
    ),
    "retail_srp": TrayPreset(
        label="Store shelf-ready tray of 400 g cans (estimate: 3.2 mm board of the same weight)",
        board_t=0.0032,
        panel_grip_t=0.0064,
        lip_h=0.0280,
        tray_h=0.1300,
        header_h=0.0180,
        grammage=0.405,
        tray_friction=0.35,
        source_kind="estimate",
        source_note="estimate: no maker page for a store tray found; the dieline's sizes with a thicker board",
    ),
}

GRID_PRESETS: dict[str, tuple[int, int]] = {
    "3x2": (3, 2),  # (cols across y, rows deep along x) -> 6 cans
    "4x3": (4, 3),  # (cols across y, rows deep along x) -> 12 cans
}

TRAY_PRESET = os.environ.get("SHELF_TRAY_PRESET", "dieline")
if TRAY_PRESET not in TRAY_PRESETS:
    raise ValueError(f"SHELF_TRAY_PRESET must be one of {tuple(TRAY_PRESETS)}, got {TRAY_PRESET!r}")

CAN_GRID_PRESET = os.environ.get("SHELF_TRAY_GRID", "3x2")
if CAN_GRID_PRESET not in GRID_PRESETS:
    raise ValueError(f"SHELF_TRAY_GRID must be one of {tuple(GRID_PRESETS)}, got {CAN_GRID_PRESET!r}")

ACTIVE_TRAY = TRAY_PRESETS[TRAY_PRESET]
GRID_COLS, GRID_ROWS = GRID_PRESETS[CAN_GRID_PRESET]
N_CANS = GRID_COLS * GRID_ROWS

# --- can constants ---
CAN_D = CAN_SPEC.diameter
CAN_R = 0.5 * CAN_D
CAN_H = CAN_SPEC.height
CAN_MASS = CAN_SPEC.mass
CAN_FRICTION = CAN_SPEC.friction

# --- tray & front-panel geometry (scales cleanly from 3x2 nominal 230 x 156 mm to 4x3 if selected) ---
BOARD_T = ACTIVE_TRAY.board_t
PANEL_GRIP_T = ACTIVE_TRAY.panel_grip_t
LIP_H = ACTIVE_TRAY.lip_h
TRAY_H = ACTIVE_TRAY.tray_h
HEADER_H = ACTIVE_TRAY.header_h

TRAY_W = 0.230 + (GRID_COLS - 3) * (CAN_D + 0.001)  # 230.0 mm for 3x2; 304.0 mm for 4x3
TRAY_D = 0.164 + (GRID_ROWS - 2) * (CAN_D + 0.001)  # 164.0 mm for 3x2 (gives ~12 mm front air gap behind panel to front cans)
PANEL_W = TRAY_W - 0.012  # 218.0 mm for 3x2 (leaves 6.0 mm per side for side-wall thickness + skew clearance)
PANEL_H = TRAY_H - LIP_H  # 102.0 mm nominal height from z = 28 mm bottom perforation to z = 130 mm top rim
PERF_SEAM_GAP = 0.0020  # 2.0 mm collider gap across perforation lines (bridged visually by solid=False dashes)
PANEL_SOLID_H = PANEL_H - PERF_SEAM_GAP  # 100.0 mm solid height above z = 30.0 mm

GLUE_FLAP_W = 0.035  # m: tray_template.svg's glue flaps and side-seam tabs
TRAY_BLANK_AREA = (
    TRAY_W * TRAY_H + TRAY_W * TRAY_D + 2.0 * TRAY_D * TRAY_H + TRAY_W * LIP_H + 2.0 * GLUE_FLAP_W * (TRAY_H + LIP_H + PANEL_H)
)  # m^2, 0.1349 for 3x2: rear wall, floor, side walls, front lip, glue flaps and side tabs of tray_template.svg
TRAY_MASS = round(ACTIVE_TRAY.grammage * TRAY_BLANK_AREA, 4)  # 0.0546 kg for 3x2 (estimate: area x grammage)
PANEL_MASS = round(ACTIVE_TRAY.grammage * PANEL_W * (PANEL_H + HEADER_H), 4)  # 0.0106 kg for 3x2: panel and folded header
TRAY_FRICTION = ACTIVE_TRAY.tray_friction  # 0.35 (estimate, README "Key figures")

# Hidden serial-chain joint carriers (c_lift, c_pitch, c_twist): their mass is part of TRAY_MASS so total tray mass is exact
CARRIER_NAMES = ("c_lift", "c_pitch", "c_twist")
CARRIER_MASS = 0.005  # kg per carrier (3 x 5 g = 15 g inside TRAY_MASS)
CARRIER_INERTIA = (2.5e-4, 2.5e-4, 2.5e-4)  # kg m^2: matched to front_panel inertia so PhysX transmits torque cleanly
TRAY_BODY_MASS = round(max(0.020, TRAY_MASS - len(CARRIER_NAMES) * CARRIER_MASS), 4)  # 0.0396 kg -> 39.6 + 15 = 54.6 g

TOTAL_FILLED_MASS = round(TRAY_MASS + PANEL_MASS + N_CANS * CAN_MASS, 4)  # 2.789 kg (27.4 N) with the panel
TRAY_SLIP_FORCE_N = round(TRAY_FRICTION * TOTAL_FILLED_MASS * GRAVITY, 2)  # 9.58 N horizontal slip threshold on bench

# --- perforation: the tear paper's indices for the 6 mm cut / 2 mm tie perforation and the board's grammage ---
PERF_CUT_M, PERF_TIE_M = 0.006, 0.002  # m: cut and tie of every perforation line (tray_template.svg), as in the paper
PERF_TENSILE_INDEX = 5.2  # N m/g: 6/2 perforation in three-layer E-flute board, 320-409 g/m^2 (paper, Fig. 2c); applied by grammage
PERF_TEAR_INDEX = 0.68e-3  # N m^2/g: Elmendorf tear index of the same perforation and boards (paper, Fig. 2f); applied by grammage
PERF_LINE_N_PER_M = PERF_TENSILE_INDEX * 1000.0 * ACTIVE_TRAY.grammage  # 2,106 N/m: in-plane strength of a perforation line
PERF_TEAR_FRONT_N = PERF_TEAR_INDEX * 1000.0 * ACTIVE_TRAY.grammage  # 0.275 N: runs one tear front out of plane
PERF_LENGTH_M = PANEL_W + 2.0 * PANEL_H  # 0.422 m: the bottom line and the two side seams

GRIP_DZ = 0.121  # m nominal grip height above tray underside (middle of 18 mm header z = 112..130 mm)
PEEL_LEVER_ARM = GRIP_DZ - LIP_H  # 0.093 m lever arm from z = 28 mm bottom perforation hinge to z = 121 mm grip line
PEEL_SKEW_MIN_DEG = 6.0  # deg (model): corner skew that turns a pull at the rim into a corner peel
PEEL_FIRST_TIE_DEG = 8.0  # deg (model): fold at which stage 1 of 10 tears
PEEL_COMPLETE_DEG = 52.0  # deg (model): fold at which a corner peel has run the whole perforation and the panel comes off
N_TIES = 10  # discrete perforation stages tracked monotonically from PEEL_FIRST_TIE_DEG to PEEL_COMPLETE_DEG
SIDE_SEAM_TIES = round(N_TIES * 2.0 * PANEL_H / PERF_LENGTH_M)  # 5: the side seams' share, all that a flat pull tears
PEEL_ARC_MM = round(1000.0 * PEEL_LEVER_ARM * math.radians(PEEL_COMPLETE_DEG), 1)  # 84.4 mm top-edge peel arc

# Estimates: tearing a length L out of plane takes 2 x PERF_TEAR_FRONT_N x L of work (Elmendorf: the two tongues part
# by twice the tear length); the corner peel spends it over the top edge's arc to PEEL_COMPLETE_DEG.
STRAIGHT_LIFT_HOLD_N = round(PERF_LINE_N_PER_M * PANEL_W, 1)  # 459.1 N: a straight lift loads the whole bottom line in plane
PEEL_RUN_N = round(2.0 * PERF_TEAR_FRONT_N * PERF_LENGTH_M / (PEEL_LEVER_ARM * math.radians(PEEL_COMPLETE_DEG)), 2)  # 2.75 N
PEEL_START_N = PEEL_RUN_N  # the paper gives the running tear only: no separate start force is claimed
SYMMETRIC_PULL_HOLD_N = round(2.0 * PEEL_RUN_N, 2)  # 5.5 N: a flat pull drives two tear fronts at once, one per side seam

# Joint spring & damping parameters
K_LIFT = 10_000.0  # N/m (model): stiffness of the uncut bottom line, read as the lift force
LIFT_TEAR_M = STRAIGHT_LIFT_HOLD_N / K_LIFT  # 45.9 mm stretch at the in-plane tear: never reached, the stop comes first
LIFT_STOP_M = 0.0065  # m: upper stop (65 N on the spring); beyond it the stop carries the load and nothing tears
D_LIFT = 2.0 * math.sqrt(K_LIFT * CARRIER_MASS)  # ~14.1 N s/m critical damping

STIFFNESS_PITCH = 30.0  # N m/rad: stiff capped elasto-plastic hinge (lag 0.49 deg at 2.75 N, 0.98 deg at 5.5 N)
D_PITCH = 0.35  # N m s/rad: smooth non-oscillatory fold
PITCH_MAX_DEG = 85.0  # deg: hinge upper travel stop before detachment

# Diagonal corner-skew hinge axis in world (sx, 0, -sz): predominantly vertical (-z) with forward (+x) coupling so both
# initial outward (-x) corner pulls and downward (-z) corner pulls twist peel_skew constructively, while any symmetric
# pull at y = 0 produces identically zero skew torque.
_SKEW_NORM = math.hypot(0.31, 0.95)
SKEW_AXIS = (0.31 / _SKEW_NORM, 0.0, -0.95 / _SKEW_NORM)
SKEW_MAX_DEG = 22.0  # deg either way (right-corner peel > 0, left-corner peel < 0)
K_SKEW = 0.90  # N m/rad (~6.5 deg skew at ~3.5 N corner pull at |y| = 75 mm; 0 deg on symmetric pull)
D_SKEW = 0.08  # N m s/rad

# --- scene placement in station frame (m from arm-base midpoint; x forward, y left) ---
TRAY_XY = (0.280, 0.000)
FRONT_X = TRAY_XY[0] - 0.5 * TRAY_D  # 0.198 m for 3x2 (164 mm depth): front face of tray facing the arms (-x)
REAR_X = TRAY_XY[0] + 0.5 * TRAY_D  # 0.362 m for 3x2 (164 mm depth): rear face of tray
DROP_ZONE_XY = (0.300, -0.285)  # right-bench flat placement outline center (>60 mm edge gap from tray_base)
DROP_ZONE_SIZE = (0.160, 0.260)  # (x_len, y_len) m: 160 x 260 mm outline framing the 100 x 218 mm panel laid at 0 yaw

# --- scoring thresholds (single source shared by task.py, task.json, README.md, and tests) ---
MAX_TRAY_TILT_DEG = 10.0  # deg whole-trial zero rule on tray_base tilt from horizontal
MAX_TRAY_LIFT_M = 0.015  # m (15.0 mm) whole-trial zero rule on tray_base underside center lift above bench
MAX_TRAY_SHIFT_M = 0.025  # m (25.0 mm) whole-trial & end-state corner horizontal shift from start tape marks
MAX_CAN_TIP_TRIAL_DEG = 12.0  # deg whole-trial zero rule if any can ever tips past this from vertical
MAX_CAN_TIP_END_DEG = 10.0  # deg end-state upright requirement for all cans
PARTIAL_TIES_MIN = 5  # 5 of the 10 equal stages: at least half of the 422 mm perforation torn scores 0.5
PANEL_FLAT_TILT_DEG = 15.0  # deg max panel tilt from horizontal on bench for 1.0
PANEL_FLAT_MAX_Z_M = 0.020  # m max panel center height above bench top for 1.0
PANEL_CLEAR_TRAY_M = 0.030  # m (30.0 mm) min horizontal footprint gap between detached panel and tray_base for 1.0
GRIPPER_CLEAR_M = 0.030  # m (30.0 mm) min clearance of both grippers from tray, cans, and panel for 1.0
REST_LINEAR_MPS = 0.010  # m/s (10.0 mm/s) max linear speed at rest
REST_ANGULAR_RPS = 0.10  # rad/s max angular speed at rest
REST_HOLD_S = 1.0  # s (20 steps at 20 Hz) continuous hold of full-score rest state

# --- colours (sRGB) ---
KRAFT_OUTER = (0.63, 0.47, 0.30)
KRAFT_INNER = (0.56, 0.41, 0.25)
KRAFT_HEADER = (0.69, 0.53, 0.35)
PERF_DARK = (0.22, 0.06, 0.05)
PERF_LIGHT = (0.93, 0.89, 0.80)
TOMATO_RED = (0.76, 0.14, 0.12)
LABEL_CREAM = (0.93, 0.92, 0.88)
CAN_TIN = (0.74, 0.75, 0.77)
TAPE_YELLOW = (0.92, 0.78, 0.12)
ZONE_GREY = (0.42, 0.45, 0.48)

KEY_FIGURES = (
    {
        "figure": "Can outside diameter, height, and mass",
        "constants": "CAN_D, CAN_H, CAN_MASS",
        "value": f"{1000 * CAN_D:.1f} mm, {1000 * CAN_H:.1f} mm, {1000 * CAN_MASS:.0f} g",
        "kind": "listing",
        "source": "maker's page: 400 g net, in a can; listing: 454 g gross (same can, basil variant); 73/110 mm tin listing",
    },
    {
        "figure": "Can-to-tray friction coefficient",
        "constants": "CAN_FRICTION",
        "value": f"{CAN_FRICTION:.2f}",
        "kind": "estimate",
        "source": "estimate: steel on kraft liner, no published figure found; the cans ride a tray that does not move",
    },
    {
        "figure": "Board caliper and weight; tray, lip and grip rim",
        "constants": "BOARD_T, TRAY_W, TRAY_D, TRAY_H, LIP_H, PANEL_GRIP_T",
        "value": (
            f"{1000 * BOARD_T:.1f} mm, {1000 * ACTIVE_TRAY.grammage:.0f} g/m^2; {1000 * TRAY_W:.0f} x {1000 * TRAY_D:.0f} x "
            f"{1000 * TRAY_H:.0f} mm, lip {1000 * LIP_H:.0f} mm, grip rim {1000 * PANEL_GRIP_T:.1f} mm"
        ),
        "kind": ACTIVE_TRAY.source_kind,
        "source": ACTIVE_TRAY.source_note,
    },
    {
        "figure": "Tray and front panel mass",
        "constants": "TRAY_MASS, PANEL_MASS",
        "value": f"{1000 * TRAY_MASS:.1f} g, {1000 * PANEL_MASS:.1f} g (filled {1000 * TOTAL_FILLED_MASS:.0f} g)",
        "kind": "estimate",
        "source": "estimate: board area of tray_template.svg x the maker's grammage",
    },
    {
        "figure": "Tray-to-bench static friction",
        "constants": "TRAY_FRICTION",
        "value": f"mu = {TRAY_FRICTION:.2f} ({TRAY_SLIP_FORCE_N:.2f} N slip)",
        "kind": "estimate",
        "source": "estimate: kraft board on a bench top; every outcome holds for mu >= 0.2 (slip above the 5.5 N flat pull)",
    },
    {
        "figure": "Straight-lift tear force (in plane, whole bottom line)",
        "constants": "STRAIGHT_LIFT_HOLD_N",
        "value": f"{STRAIGHT_LIFT_HOLD_N:.0f} N",
        "kind": "estimate",
        "source": "estimate: the paper's tensile index x grammage x the 218 mm bottom line",
    },
    {
        "figure": "Corner peel force (start = run) and flat-pull side-seam force",
        "constants": "PEEL_START_N, PEEL_RUN_N, SYMMETRIC_PULL_HOLD_N",
        "value": f"{PEEL_RUN_N:.2f} N, flat {SYMMETRIC_PULL_HOLD_N:.1f} N (lever arm {1000 * PEEL_LEVER_ARM:.0f} mm)",
        "kind": "estimate",
        "source": "estimate: tear work 2 x tear force x 422 mm of perforation over the 84 mm arc; two fronts on a flat pull",
    },
    {
        "figure": "Corner skew initiation angle and full-detachment peel angle (and top-edge arc)",
        "constants": "PEEL_SKEW_MIN_DEG, PEEL_COMPLETE_DEG",
        "value": f"{PEEL_SKEW_MIN_DEG:.1f} deg skew, {PEEL_COMPLETE_DEG:.1f} deg fold ({PEEL_ARC_MM:.1f} mm arc)",
        "kind": "estimate",
        "source": "estimate (model): the arc sets the peel force through the tear work, so both move together",
    },
)


def can_local_xy() -> list[tuple[float, float]]:
    """(x, y) offsets of each can's axis in tray_base's frame, ordered front-to-back then right-to-left."""
    inner_w = TRAY_W - 2.0 * BOARD_T
    inner_d = TRAY_D - 2.0 * BOARD_T
    pitch_y = CAN_D + max(0.0008, (inner_w - GRID_COLS * CAN_D) / max(1, GRID_COLS + 1))
    pitch_x = CAN_D + max(0.0008, (inner_d - GRID_ROWS * CAN_D) / max(1, GRID_ROWS + 1))
    # Shift slightly toward the rear wall (+0.6 mm) to maximize front clearance behind front_panel
    rear_face = 0.5 * TRAY_D - BOARD_T
    back_row_x = rear_face - 0.0008 - CAN_R
    xs = [back_row_x - (GRID_ROWS - 1 - r) * pitch_x for r in range(GRID_ROWS)]
    ys = [(c - 0.5 * (GRID_COLS - 1)) * pitch_y for c in range(GRID_COLS)]
    return [(float(x), float(y)) for x in xs for y in ys]


def tray_corners_xy(center_xy: tuple[float, float] = TRAY_XY, yaw: float = 0.0) -> np.ndarray:
    """4x2 world (x, y) coordinates of tray_base's outer corners: (front-right, front-left, rear-left, rear-right)."""
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    hx, hy = 0.5 * TRAY_D, 0.5 * TRAY_W
    local = np.array([[-hx, -hy], [-hx, +hy], [+hx, +hy], [+hx, -hy]], dtype=float)
    return local @ rot.T + np.asarray(center_xy, dtype=float)


def panel_built_pos(table_top: float) -> tuple[float, float, float]:
    """World origin of front_panel as built: outer face flush at FRONT_X (leaves >4 mm clear to front cans for skew)."""
    x = FRONT_X - 0.5 * BOARD_T + 0.0016
    y = 0.0
    z = table_top + OVER + LIP_H + PERF_SEAM_GAP + 0.5 * PANEL_SOLID_H
    return (float(x), float(y), float(z))


def hinge_anchor_world(table_top: float) -> tuple[float, float, float]:
    """World anchor of the perforation joint chain along the front bottom tear line (x = FRONT_X, z = LIP_H)."""
    return (float(FRONT_X), 0.0, float(table_top + OVER + LIP_H))


def tray_base_shapes() -> list:
    """Compound primitive shapes of tray_base (origin at underside center: z in [0, TRAY_H])."""
    hx, hy = 0.5 * TRAY_D, 0.5 * TRAY_W
    wall_h = TRAY_H - BOARD_T
    upper_h = TRAY_H - LIP_H
    side_upper_len = TRAY_D - 2.0 * BOARD_T - PERF_SEAM_GAP  # starts behind front_panel + perforation gap
    side_upper_x = -hx + BOARD_T + PERF_SEAM_GAP + 0.5 * side_upper_len
    shapes = [
        # Floor
        box((TRAY_D, TRAY_W, BOARD_T), pos=(0.0, 0.0, 0.5 * BOARD_T), color=KRAFT_INNER),
        # Rear wall
        box((BOARD_T, TRAY_W, wall_h), pos=(hx - 0.5 * BOARD_T, 0.0, BOARD_T + 0.5 * wall_h), color=KRAFT_OUTER),
        # Front lower retainer skirt (z = BOARD_T .. LIP_H)
        box(
            (BOARD_T, TRAY_W - 2.0 * BOARD_T, LIP_H - BOARD_T),
            pos=(-hx + 0.5 * BOARD_T, 0.0, BOARD_T + 0.5 * (LIP_H - BOARD_T)),
            color=KRAFT_OUTER,
        ),
    ]
    for sign in (-1.0, 1.0):
        wy = sign * (hy - 0.5 * BOARD_T)
        # Lower side wall (full depth to front face)
        shapes.append(
            box(
                (TRAY_D - BOARD_T, BOARD_T, LIP_H - BOARD_T),
                pos=(-0.5 * BOARD_T, wy, BOARD_T + 0.5 * (LIP_H - BOARD_T)),
                color=KRAFT_OUTER,
            )
        )
        # Upper side wall (behind front perforation seam) + folded 2-ply top brace rim
        shapes.append(
            box(
                (side_upper_len, BOARD_T, upper_h),
                pos=(side_upper_x, wy, LIP_H + 0.5 * upper_h),
                color=KRAFT_OUTER,
            )
        )
        fold_extra = max(0.0005, PANEL_GRIP_T - BOARD_T)
        side_rim_h = 0.0155  # z = 114.5..130.0 mm, strictly above 113.2 mm can tops while keeping outer width exact
        brace_rim_len = side_upper_len - 0.025  # set back 25 mm behind front_panel so corner skew never touches it
        shapes.append(
            box(
                (brace_rim_len, fold_extra, side_rim_h),
                pos=(side_upper_x + 0.0125, sign * (hy - BOARD_T - 0.5 * fold_extra), TRAY_H - 0.5 * side_rim_h),
                color=KRAFT_HEADER,
            )
        )
    # Visual perforation dashes along top of front lower lip (visible from top_cam both before and after tear)
    n_dashes = 11
    span = PANEL_W - 0.012
    for i in range(n_dashes):
        dy = -0.5 * span + i * (span / (n_dashes - 1))
        col = PERF_DARK if i % 2 == 0 else PERF_LIGHT
        shapes.append(
            box((BOARD_T + 0.0012, 0.012, 0.0030), pos=(-hx + 0.5 * BOARD_T, dy, LIP_H - 0.0015), color=col, solid=False)
        )
    return shapes


def front_panel_shapes() -> list:
    """Shapes of front_panel (origin at geometric center of main sheet: x thickness, y width, z height)."""
    fold_extra = max(0.0005, PANEL_GRIP_T - BOARD_T)
    header_z = 0.5 * PANEL_SOLID_H - 0.5 * HEADER_H
    shapes = [
        # Main single-wall corrugated sheet (x in [-BOARD_T/2, +BOARD_T/2])
        box((BOARD_T, PANEL_W, PANEL_SOLID_H), pos=(0.0, 0.0, 0.0), color=KRAFT_OUTER),
        # Folded double-ply top header on outer (-x) face: total thickness PANEL_GRIP_T (6.4 mm) over top 18 mm
        box((fold_extra, PANEL_W, HEADER_H), pos=(-0.5 * (BOARD_T + fold_extra), 0.0, header_z), color=KRAFT_HEADER),
    ]
    # High-contrast bottom and side perforation dashes (solid=False visual markings)
    n_horiz = 13
    span_y = PANEL_W - 0.014
    for i in range(n_horiz):
        dy = -0.5 * span_y + i * (span_y / (n_horiz - 1))
        col = PERF_DARK if i % 2 == 0 else PERF_LIGHT
        shapes.append(
            box((BOARD_T + 0.0014, 0.011, 0.0040), pos=(-0.0004, dy, -0.5 * PANEL_SOLID_H + 0.002), color=col, solid=False)
        )
    n_vert = 7
    span_z = PANEL_SOLID_H - HEADER_H - 0.012
    for sign in (-1.0, 1.0):
        for j in range(n_vert):
            dz = -0.5 * PANEL_SOLID_H + 0.008 + j * (span_z / max(1, n_vert - 1))
            col = PERF_DARK if j % 2 == 0 else PERF_LIGHT
            shapes.append(
                box((BOARD_T + 0.0014, 0.0040, 0.009), pos=(-0.0004, sign * (0.5 * PANEL_W - 0.002), dz), color=col, solid=False)
            )
    return shapes


def can_shapes() -> list:
    """Shapes of one 400 g steel tomato can (origin at cylinder center: z in [-CAN_H/2, +CAN_H/2])."""
    rim_h = 0.0025
    body_h = CAN_H - 2.0 * rim_h
    return [
        cylinder(CAN_R, body_h, pos=(0.0, 0.0, 0.0), color=TOMATO_RED),
        cylinder(CAN_R, rim_h, pos=(0.0, 0.0, 0.5 * CAN_H - 0.5 * rim_h), color=CAN_TIN),
        cylinder(CAN_R, rim_h, pos=(0.0, 0.0, -0.5 * CAN_H + 0.5 * rim_h), color=CAN_TIN),
        cylinder(CAN_R + 0.0004, 0.034, pos=(0.0, 0.0, 0.002), color=LABEL_CREAM, solid=False),
    ]


def carrier_shapes(pass_names: tuple[str, ...]) -> list:
    """4 small non-colliding boxes inside the front lower skirt for a hidden serial-chain carrier body."""
    return [
        box((0.0016, 0.0016, 0.0016), pos=(0.0016, sy * 0.035, -0.012 + sz * 0.004), passes=pass_names, color=KRAFT_INNER)
        for sy in (-1.0, 1.0)
        for sz in (-1.0, 1.0)
    ]


def bench_mark_shapes() -> list:
    """Visual-only (solid=False) yellow gaffer-tape L-marks around tray_base and grey outline around DROP_ZONE_XY."""
    shapes = []
    arm_len, tape_w, tape_h = 0.030, 0.008, 0.0006
    hx, hy = 0.5 * TRAY_D, 0.5 * TRAY_W
    tx, ty = TRAY_XY
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            cx = tx + sx * (hx + 0.5 * tape_w)
            cy = ty + sy * (hy + 0.5 * tape_w)
            shapes.append(
                box((arm_len, tape_w, tape_h), pos=(cx - sx * (0.5 * arm_len - 0.5 * tape_w), cy, 0.5 * tape_h), color=TAPE_YELLOW, solid=False)
            )
            shapes.append(
                box((tape_w, arm_len, tape_h), pos=(cx, cy - sy * (0.5 * arm_len - 0.5 * tape_w), 0.5 * tape_h), color=TAPE_YELLOW, solid=False)
            )
    zx, zy = DROP_ZONE_XY
    zlx, zly = DROP_ZONE_SIZE
    bw = 0.006
    shapes.extend(
        [
            box((zlx, bw, tape_h), pos=(zx, zy - 0.5 * zly + 0.5 * bw, 0.5 * tape_h), color=ZONE_GREY, solid=False),
            box((zlx, bw, tape_h), pos=(zx, zy + 0.5 * zly - 0.5 * bw, 0.5 * tape_h), color=ZONE_GREY, solid=False),
            box((bw, zly, tape_h), pos=(zx - 0.5 * zlx + 0.5 * bw, zy, 0.5 * tape_h), color=ZONE_GREY, solid=False),
            box((bw, zly, tape_h), pos=(zx + 0.5 * zlx - 0.5 * bw, zy, 0.5 * tape_h), color=ZONE_GREY, solid=False),
        ]
    )
    return shapes


def _point_in_convex_poly(pt: np.ndarray, poly: np.ndarray) -> bool:
    n = len(poly)
    signs = [(poly[(i + 1) % n][0] - poly[i][0]) * (pt[1] - poly[i][1]) - (poly[(i + 1) % n][1] - poly[i][1]) * (pt[0] - poly[i][0]) for i in range(n)]
    return all(s >= -1e-12 for s in signs) or all(s <= 1e-12 for s in signs)


def _seg_intersect(a: np.ndarray, b: np.ndarray, c: np.ndarray, d: np.ndarray) -> bool:
    def ccw(p1, p2, p3):
        return (p3[1] - p1[1]) * (p2[0] - p1[0]) - (p2[1] - p1[1]) * (p3[0] - p1[0])

    return (ccw(c, d, a) * ccw(c, d, b) < 0.0) and (ccw(a, b, c) * ccw(a, b, d) < 0.0)


def _point_to_seg_2d(pt: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom < 1e-18:
        return float(np.linalg.norm(pt - a))
    t = max(0.0, min(1.0, float(np.dot(pt - a, ab)) / denom))
    return float(np.linalg.norm(pt - (a + t * ab)))


def convex_hull_2d(pts: np.ndarray) -> np.ndarray:
    pts = np.unique(np.round(pts, 9), axis=0)
    if len(pts) <= 2:
        return pts
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in pts[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1], dtype=float)


def polygon_gap_2d(pa: np.ndarray, pb: np.ndarray) -> float:
    """Exact horizontal 2D edge-to-edge gap (m) between two convex polygons."""
    if any(_point_in_convex_poly(v, pb) for v in pa) or any(_point_in_convex_poly(v, pa) for v in pb):
        return 0.0
    na, nb = len(pa), len(pb)
    for i in range(na):
        for j in range(nb):
            if _seg_intersect(pa[i], pa[(i + 1) % na], pb[j], pb[(j + 1) % nb]):
                return 0.0
    best = math.inf
    for v in pa:
        for j in range(nb):
            best = min(best, _point_to_seg_2d(v, pb[j], pb[(j + 1) % nb]))
    for v in pb:
        for i in range(na):
            best = min(best, _point_to_seg_2d(v, pa[i], pa[(i + 1) % na]))
    return float(best)


def point_to_shapes_m(point: np.ndarray, pos: np.ndarray, rot: np.ndarray, shapes, quat_to_mat) -> float:
    """Exact distance (m) from ``point`` to the solid primitive shapes of a body at ``(pos, rot)``."""
    best = math.inf
    for s in shapes:
        if not s.solid:
            continue
        sr = rot @ quat_to_mat(s.quat)
        sc = pos + rot @ np.asarray(s.pos, dtype=float)
        local = sr.T @ (point - sc)
        if s.kind == "box":
            half = 0.5 * np.asarray(s.dims, dtype=float)
            outside = np.maximum(np.abs(local) - half, 0.0)
            inside = min(float(np.max(np.abs(local) - half)), 0.0)
            best = min(best, float(np.linalg.norm(outside)) + inside)
        elif s.kind == "cylinder":
            r, h = float(s.dims[0]), float(s.dims[1])
            dr = max(0.0, math.hypot(float(local[0]), float(local[1])) - r)
            dz = max(0.0, abs(float(local[2])) - 0.5 * h)
            best = min(best, math.hypot(dr, dz))
    return float(best)


def summary() -> dict:
    """Structured summary of active preset geometry, masses, and force thresholds."""
    return {
        "tray_preset": TRAY_PRESET,
        "grid_preset": CAN_GRID_PRESET,
        "n_cans": N_CANS,
        "can": asdict(CAN_SPEC),
        "tray_mm": {
            "width": round(1000 * TRAY_W, 2),
            "depth": round(1000 * TRAY_D, 2),
            "height": round(1000 * TRAY_H, 2),
            "lip_h": round(1000 * LIP_H, 2),
            "board_t": round(1000 * BOARD_T, 2),
            "panel_grip_t": round(1000 * PANEL_GRIP_T, 2),
            "panel_w": round(1000 * PANEL_W, 2),
            "panel_h": round(1000 * PANEL_H, 2),
        },
        "masses_kg": {
            "tray_base_total": TRAY_MASS,
            "front_panel": PANEL_MASS,
            "single_can": CAN_MASS,
            "filled_total": TOTAL_FILLED_MASS,
        },
        "forces_and_angles": {
            "tray_bench_slip_n": TRAY_SLIP_FORCE_N,
            "straight_lift_hold_n": STRAIGHT_LIFT_HOLD_N,
            "symmetric_pull_hold_n": SYMMETRIC_PULL_HOLD_N,
            "peel_start_n": PEEL_START_N,
            "peel_run_n": PEEL_RUN_N,
            "peel_skew_min_deg": PEEL_SKEW_MIN_DEG,
            "peel_first_tie_deg": PEEL_FIRST_TIE_DEG,
            "peel_complete_deg": PEEL_COMPLETE_DEG,
            "peel_lever_arm_mm": round(1000 * PEEL_LEVER_ARM, 1),
            "peel_arc_mm": PEEL_ARC_MM,
            "n_ties": N_TIES,
        },
        "can_local_xy_mm": [[round(1000 * x, 2), round(1000 * y, 2)] for x, y in can_local_xy()],
        "key_figures": list(KEY_FIGURES),
    }


if __name__ == "__main__":
    print(json.dumps(summary(), indent=2))
