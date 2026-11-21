"""Single source of product figures, geometry, inertia, slope kinematics, and rolling-resistance constants for the
rolling glass bottle task: a straight cylindrical glass bottle rolling off a sloped drainboard onto a clamped kitchen
worktop toward an open edge, and a recessed cork coaster set aside from the rolling path.

Every dimension, mass, rotational inertia factor, slope angle, target foot speed, crossing time, rolling deceleration,
and scoring threshold lives here with its source kind (``listing``, ``estimate``, or ``protocol``) so every figure has
one home that the scene, rules, demo, checks, and tests read together.

Bottle body frame B:
- Origin at the geometric centre of the closed bottle cylinder.
- ``+z`` points toward the cap end (top when standing upright); ``-z`` points toward the glass base.
- At reset on the sloped drainboard, ``+z`` (cap end) points along world ``+x`` (away from the arms) and ``-z`` (base)
  points along world ``-x`` (toward the arms), so lifting the horizontal bottle from above and pitching the gripper
  90 degrees forward (``Ry(-90 deg)``) swings ``+x`` (cap) straight up to ``+z`` and ``-x`` (base) down onto the coaster.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os

import numpy as np
from mechkit import box, cylinder, matrix_to_quat
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
TOP = float(TABLE_TOP[SCENE])  # m: calibrated station bench surface (0.7475 m)
GRAVITY = 9.81  # m/s^2
CONTACT_OFFSET = 0.001  # m: 1 mm PhysX contact offset across the rolling scene
CONTROL_HZ = 20.0  # Hz: control step rate (dt = 0.05 s; 12 PhysX substeps per step at 240 Hz)
DT = 1.0 / CONTROL_HZ
RUN_LIMIT_STEPS = 800  # 40.0 s at 20 Hz


@dataclass(frozen=True)
class BottleSpec:
    """Parameters of an unopened straight-sided cylindrical glass bottle (SI: m, kg) with its source kind."""

    preset: str
    product: str
    diameter: float
    height: float
    mass: float
    cap_height: float
    glass_rgb: tuple[float, float, float]
    cap_rgb: tuple[float, float, float]
    band_rgb: tuple[float, float, float]
    source: str
    kind: str  # "maker's page" | "listing" | "estimate"


BOTTLE_PRESETS: dict[str, BottleSpec] = {
    "voss_375": BottleSpec(
        preset="voss_375",
        product="VOSS Artesian Still Water 375 mL unopened straight cylindrical glass bottle",
        diameter=0.0540,
        height=0.2290,
        mass=0.700,
        cap_height=0.0320,
        glass_rgb=(0.68, 0.80, 0.84),
        cap_rgb=(0.20, 0.22, 0.25),
        band_rgb=(0.90, 0.93, 0.95),
        source="listing: desertcart.sg/products/258462045 gives "
               "'5.4 x 5.4 x 22.9 cm' and '357.2 g' (54 mm across, 229 mm tall with the cap); uke.co.za/product/"
               "voss-still-water-375-ml-glass-bottle/ ('Voss Still Water – 375ml (Glass Bottle)') gives 'Weight "
               "0.7 kg', the full bottle as sold, rounded to 0.1 kg. The 357.2 g is below the 375 g of the water alone, "
               "so it is the empty bottle (estimate); 357 + 375 = 732 g agrees with the 0.7 kg to its rounding",
        kind="listing",
    ),
    "true_fruits_250": BottleSpec(
        preset="true_fruits_250",
        product="true fruits Smoothie 250 mL unopened straight cylindrical glass bottle",
        diameter=0.0560,
        height=0.1620,
        mass=0.440,
        cap_height=0.0220,
        glass_rgb=(0.74, 0.62, 0.26),
        cap_rgb=(0.18, 0.18, 0.20),
        band_rgb=(0.92, 0.92, 0.90),
        source="estimate: true fruits Smoothie 250 mL glass bottle (German grocery, true-fruits.com); 56 x 162 mm and "
               "440 g derived the same way from the 250 mL fill in a straight cylinder (~190 g glass + 250 g smoothie)",
        kind="estimate",
    ),
}

BOTTLE_PRESET = os.environ.get("ROLLING_BOTTLE_PRESET", "voss_375")
if BOTTLE_PRESET not in BOTTLE_PRESETS:
    raise ValueError(f"ROLLING_BOTTLE_PRESET must be one of {tuple(BOTTLE_PRESETS)}, got {BOTTLE_PRESET!r}")

ACTIVE_BOTTLE = BOTTLE_PRESETS[BOTTLE_PRESET]
BOTTLE_D = ACTIVE_BOTTLE.diameter  # m: outer cylinder diameter (54.0 mm for voss_375)
BOTTLE_R = 0.5 * BOTTLE_D
BOTTLE_H = ACTIVE_BOTTLE.height  # m: total cylinder height including flush cap (229.0 mm for voss_375)
BOTTLE_MASS = ACTIVE_BOTTLE.mass  # kg: unopened filled mass (0.700 kg for voss_375)
BOTTLE_CAP_H = ACTIVE_BOTTLE.cap_height  # m: cap visual band length at +z end
BOTTLE_FRICTION = (0.75, 0.65)  # estimate: typical smooth-glass static/dynamic friction on laminate and rubber pads
BOTTLE_LINEAR_DAMPING = 0.0  # 1/s: zero linear viscous drag
BOTTLE_ANGULAR_DAMPING = 0.05  # 1/s: the simulator's standard rigid-body rotational damping (stated in README/task.json)

# Effective axial rotational inertia ratio I_zz / (m * R^2): low-viscosity water inside a sealed smooth cylinder slips
# relative to the glass wall during a 2 s low-speed roll rather than spinning as a rigid solid (solid = 0.50).
ROT_INERTIA_FACTOR = 0.35  # estimate: I_zz / (m R^2) between glass shell alone (~0.20) and solid fill (0.50); see KEY_FIGURES
BOTTLE_INERTIA_RATIO = ROT_INERTIA_FACTOR

# --- Clamped laminate worktop slab (fixed to bench with two rear F-clamps) ---
# Standard 600 mm kitchen worktop cut to 360 mm depth so the 229 mm horizontal bottle at x = 0.375 (spanning
# [0.2605, 0.4895]) clears the 100 mm coaster at x = 0.205 (spanning [0.155, 0.255]) by 5.5 mm on an idle roll.
WORKTOP_D = 0.360  # m: depth along x, [0.145, 0.505] (listing: cut-off from 600 mm laminate worktop)
WORKTOP_W = 0.520  # m: width along y, [-0.230, +0.290] (listing)
WORKTOP_H = 0.038  # m: thickness along z, 38.0 mm kitchen worktop cut-off (listing)
WORKTOP_MIN_X, WORKTOP_MAX_X = 0.145, 0.505
WORKTOP_MIN_Y, WORKTOP_MAX_Y = -0.230, 0.290
WORKTOP_X = 0.5 * (WORKTOP_MIN_X + WORKTOP_MAX_X)  # 0.325 m
WORKTOP_Y = 0.5 * (WORKTOP_MIN_Y + WORKTOP_MAX_Y)  # +0.030 m
WORKTOP_TOP_Z = TOP + WORKTOP_H  # m: flat counter top surface (TOP + 0.038 m); open drop cliff at y = -0.230
WORKTOP_FRICTION = (0.55, 0.48)  # estimate: typical laminate worktop static/dynamic friction
WORKTOP_RGB = (0.83, 0.81, 0.77)  # light warm stone/maple laminate for strong contrast with bottle, coaster & stripe
CLIFF_STRIPE_W = 0.012  # m: visual edge marker width along open right cliff y = -0.230
CLIFF_STRIPE_RGB = (0.82, 0.22, 0.14)  # high-contrast vermilion edge trim along open drop edge y = -0.230

# --- Fastened sloped melamine drainboard on left end of worktop ---
DRAIN_D = 0.360  # m: depth along x, [0.145, 0.505] (estimate: cut of the board, matches worktop depth)
DRAIN_L = 0.170  # m: length along slope from foot y = +0.110 to ~+0.280 (estimate: cut of the 360 mm board)
DRAIN_T = 0.012  # m: board thickness, 12.0 mm melamine shelf/cutting board (estimate)
DRAIN_FOOT_Y = 0.110  # m: flush transition seam onto level worktop (0 mm step)
SLOPE_DEG = 3.0  # estimate: gentle kitchen-drainboard slope down toward -y; sin(3.0 deg) * 170 mm = 8.90 mm shim
SLOPE_RAD = math.radians(SLOPE_DEG)
SHIM_H = round(DRAIN_L * math.sin(SLOPE_RAD), 4)  # 0.0089 m derived riser strip at the outer end for SLOPE_DEG
DRAIN_FRICTION = (0.55, 0.48)  # estimate: typical melamine board static/dynamic friction
DRAIN_RGB = (0.62, 0.66, 0.70)  # cool slate-grey melamine board, distinct in brightness from worktop and glass bottle
START_TAPE_RGB = (0.18, 0.20, 0.22)  # dark visual start-line tape at 140 mm up-slope

# --- Bottle release mark & target roll kinematics ---
ROLL_LANE_X = 0.375  # m: straight rolling lane x-coordinate (170 mm behind coaster centre x = 0.205)
RELEASE_UP_M = 0.140  # m: protocol start mark, 140 mm up-slope from the foot seam y = +0.110
START_CLEAR_M = 0.0002  # m: 0.2 mm normal clearance as built; settles onto drainboard during reset
V_FOOT_MPS = 0.28  # estimate (physics + simulator): idle roll trace crosses the foot at 0.278 m/s
V_FOOT_TARGET_MPS = V_FOOT_MPS
T_150_S = 0.56  # estimate (physics + simulator): 240 Hz trace reaches y = -0.040 at 0.563 s from the foot
T_300_S = 1.18  # estimate (physics + simulator): 240 Hz trace reaches y = -0.190 at 1.175 s from the foot
ROLL_DECEL_MPS2 = 0.035  # estimate: rolling-resistance coefficient ~0.005 on a smooth counter, a = 0.0048 g / (1 + 0.35)
ROLL_CONTACT_MIN_N = 0.5  # N: minimum normal contact force with worktop + drainboard for rolling resistance to act
ROLL_SPEED_MIN_MPS = 0.005  # m/s: minimum rolling speed R * |omega_axial| for rolling-resistance torque to act

# --- Recessed cork coaster (IKEA 365+ 100 mm cork coaster, placed 170 mm forward of the x = 0.375 roll path) ---
# Fixed rigid body (kinematic=True): on the bench the coaster is taped to the worktop on its mark with double-sided
# mounting tape, so it cannot slide when bumped before the bottle is on it; the idle roll lane clears it by 5.5 mm
# and the scored 1.0 level still requires it within 15.0 mm of its start position (the tape held).
COASTER_OUT_D = 0.100  # m: outer diameter (maker's page: IKEA 365+ art. 302.829.46, 10 cm)
COASTER_OUT_R = 0.5 * COASTER_OUT_D
COASTER_IN_D = 0.092  # m: inner flat recess diameter (estimate: scaled from the maker's cupped-profile photo)
COASTER_IN_R = 0.5 * COASTER_IN_D
COASTER_FLOOR_H = 0.004  # m: cork floor thickness under recess (estimate: scaled from the maker's cupped-profile photo)
COASTER_RIM_H = 0.003  # m: raised rim height above recess floor; total coaster height = 7.0 mm (estimate)
COASTER_TOTAL_H = COASTER_FLOOR_H + COASTER_RIM_H
COASTER_X = 0.205  # m: exact spec coaster x (170 mm forward of ROLL_LANE_X = 0.375 -> 5.5 mm x-clearance!)
COASTER_Y = -0.020  # m: exact spec coaster y
COASTER_STAVES = 24  # overlapping box staves forming the 3 mm raised cork rim around the 92 mm inner recess
COASTER_FRICTION = (0.90, 0.80)  # estimate: typical dry-cork static/dynamic friction
COASTER_RGB = (0.58, 0.40, 0.23)  # natural warm cork brown
COASTER_RIM_RGB = (0.50, 0.34, 0.19)

# --- Foam drop catch mat on the bench right of the y = -0.230 cliff ---
CATCH_MAT_D = 0.340  # m: along x, [0.155, 0.495] (listing)
CATCH_MAT_W = 0.160  # m: along y, [-0.395, -0.235] (listing)
CATCH_MAT_H = 0.010  # m: 10 mm foam pad thickness on bench (listing)
CATCH_MAT_X = 0.325  # m
CATCH_MAT_Y = -0.315  # m: y in [-0.395, -0.235], 5 mm right of the y = -0.230 cliff wall
CATCH_MAT_FRICTION = (0.95, 0.85)
CATCH_MAT_RGB = (0.20, 0.22, 0.25)

# --- Scoring thresholds (identical across parts.py, task.py, task.json, and README.md) ---
OFF_EDGE_LOW_Z = 0.025  # m above TOP: lowest point below TOP + 25 mm means bottle went over an edge
REST_LIN_MPS = 0.010  # m/s: linear speed < 10 mm/s counts as at rest
REST_ANG_RADPS = 0.15  # rad/s: angular speed < 0.15 rad/s counts as at rest
REST_CONTACT_MIN_N = 1.0  # N: minimum normal support force required for the 0.5 (counter) and 1.0 (coaster) levels
GRIPPER_CLEAR_M = 0.030  # m: both grippers >= 30 mm clear of bottle and coaster
HOLD_S = 1.0  # s: required rest + clearance hold duration
HOLD_STEPS = int(round(HOLD_S * CONTROL_HZ))  # 20 steps at 20 Hz
UPRIGHT_MAX_DEG = 5.0  # deg: maximum tilt of cap-up bottle axis (+z_B) from world +z
COASTER_BASE_TOL_M = 0.022  # m: bottle base centre within 22.0 mm of coaster centre
COASTER_MAX_SHIFT_M = 0.015  # m: coaster centre within 15.0 mm of start position (COASTER_X, COASTER_Y)

# Rotation placing bottle local +z (cap end) along world +x, local y along +y, local x along -z:
# det([[0, 0, 1], [0, 1, 0], [-1, 0, 0]]) = +1.
BOTTLE_LYING_ROT = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]], dtype=float)
BOTTLE_LYING_QUAT = matrix_to_quat(BOTTLE_LYING_ROT)


@dataclass(frozen=True)
class KeyFigureRow:
    """One row of the key-figures source & bench-measurement table (Check 50)."""

    figure: str
    constants: str
    value: str
    kind: str
    bench_method: str


KEY_FIGURES: tuple[KeyFigureRow, ...] = (
    KeyFigureRow(
        "Bottle outer diameter, total height, unopened mass",
        "BOTTLE_D, BOTTLE_H, BOTTLE_MASS",
        f"{1000 * BOTTLE_D:.1f} mm, {1000 * BOTTLE_H:.1f} mm, {1000 * BOTTLE_MASS:.0f} g ({BOTTLE_PRESET})",
        "listing",
        "desertcart.sg/products/258462045: '5.4 x 5.4 x 22.9 cm' and "
        "'357.2 g' (54 mm, 229 mm with the cap); uke.co.za/product/voss-still-water-375-ml-glass-bottle/: 'Weight "
        "0.7 kg', the full bottle as sold (rounded to 0.1 kg). The 357.2 g is below the 375 g of the water alone, so "
        "it is the empty bottle (estimate): 357 + 375 = 732 g agrees with 0.7 kg to the listing's rounding",
    ),
    KeyFigureRow(
        "Effective axial inertia ratio & angular damping",
        "ROT_INERTIA_FACTOR, BOTTLE_ANGULAR_DAMPING",
        f"{ROT_INERTIA_FACTOR:.2f}, {BOTTLE_ANGULAR_DAMPING:.2f} /s",
        "estimate",
        "physics: low-viscosity water in a sealed smooth cylinder barely spins during a 2 s roll, so I_zz sits between "
        "the glass shell alone (~0.20 m R^2) and a solid fill (0.50 m R^2); 0.35 is the partial-coupling value. "
        "Angular damping 0.05/s is the simulator's standard rigid-body value (stated in README and task.json)",
    ),
    KeyFigureRow(
        "Worktop slab thickness, width, depth",
        "WORKTOP_H, WORKTOP_W, WORKTOP_D",
        f"{1000 * WORKTOP_H:.1f} x {1000 * WORKTOP_W:.0f} x {1000 * WORKTOP_D:.0f} mm",
        "listing",
        "38 mm kitchen worktop: HORNBACH decor worktop listing (e.g. Kuechenarbeitsplatte 34321 Oxid, 4100 x 635 x "
        "38 mm, hornbach.de/c/kueche/kuechenarbeitsplatten/S1641); 520 x 360 mm is a cut-off with the store cutting service",
    ),
    KeyFigureRow(
        "Drainboard length, thickness, shim height, slope angle",
        "DRAIN_L, DRAIN_T, SHIM_H, SLOPE_DEG",
        f"{1000 * DRAIN_L:.1f} mm, {1000 * DRAIN_T:.1f} mm, {1000 * SHIM_H:.1f} mm, {SLOPE_DEG:.1f} deg",
        "estimate",
        "a flat 12 mm melamine-covered shelf/cutting board (standard hardware-store melamine board class) cut to "
        "360 x 170 mm; gentle kitchen-drainboard slope of 3.0 deg; shim height 8.9 mm = 170 mm x sin(3.0 deg)",
    ),
    KeyFigureRow(
        "Release distance up-slope from drainboard foot",
        "RELEASE_UP_M",
        f"{1000 * RELEASE_UP_M:.1f} mm",
        "protocol",
        "the trial's start mark: a tape line 140 mm up-slope from the flush foot seam; the straightedge holding the "
        "bottle behind it is lifted straight up at t = 0",
    ),
    KeyFigureRow(
        "Bottle speed at drainboard foot",
        "V_FOOT_MPS",
        f"{V_FOOT_MPS:.2f} m/s",
        "estimate",
        "physics + simulator: rolling-with-slip dynamics on the 3.0 deg slope (a = g sin(3.0 deg) / (1 + 0.35) minus "
        "rolling losses and the start loss fitted to an idle trace) integrated at 240 Hz gives 0.278 m/s; 0.28 recorded",
    ),
    KeyFigureRow(
        "Crossing times at 150 mm & 300 mm past foot, rolling decel",
        "T_150_S, T_300_S, ROLL_DECEL_MPS2",
        f"{T_150_S:.2f} s, {T_300_S:.2f} s, {ROLL_DECEL_MPS2:.3f} m/s^2",
        "estimate",
        "physics + simulator: flat-counter deceleration from a rolling-resistance coefficient ~0.005 of a smooth glass "
        "cylinder on a smooth counter (a = 0.0048 g / (1 + 0.35) = 0.035 m/s^2); integrated at 240 Hz: 0.56 s and 1.18 s",
    ),
    KeyFigureRow(
        "Cork coaster outer diameter; recess & rim profile",
        "COASTER_OUT_D, COASTER_IN_D, COASTER_FLOOR_H, COASTER_RIM_H",
        f"{1000 * COASTER_OUT_D:.0f} mm outer; {1000 * COASTER_IN_D:.0f} mm recess, {1000 * COASTER_FLOOR_H:.1f}+{1000 * COASTER_RIM_H:.1f} mm",
        "maker's page",
        "IKEA 365+ Coaster, cork, 10 cm (ikea.com/se/en/p/ikea-365-coaster-cork-30282946): the page title gives the "
        "10 cm diameter and no thickness; recess diameter and floor/rim heights (92 mm, 4 + 3 mm) are estimates scaled "
        "from the maker's cupped-profile photo",
    ),
)


def slope_vectors(slope_deg: float = SLOPE_DEG) -> tuple[np.ndarray, np.ndarray]:
    """(u_hat uphill along drainboard, n_hat upward normal to drainboard) in world coordinates."""
    th = math.radians(slope_deg)
    u_hat = np.array([0.0, math.cos(th), math.sin(th)], dtype=float)
    n_hat = np.array([0.0, -math.sin(th), math.cos(th)], dtype=float)
    return u_hat, n_hat


def drainboard_pose(slope_deg: float = SLOPE_DEG) -> tuple[np.ndarray, np.ndarray]:
    """World (position, 3x3 rotation) of the sloped drainboard so its top face meets worktop at (y=DRAIN_FOOT_Y,
    z=WORKTOP_TOP_Z) with a 0.000 mm step."""
    u_hat, n_hat = slope_vectors(slope_deg)
    foot = np.array([WORKTOP_X, DRAIN_FOOT_Y, WORKTOP_TOP_Z], dtype=float)
    center = foot + 0.5 * DRAIN_L * u_hat - 0.5 * DRAIN_T * n_hat
    rot = np.column_stack([[1.0, 0.0, 0.0], u_hat, n_hat])
    return center, rot


def surface_z_at(_x: float, y: float, slope_deg: float = SLOPE_DEG) -> float:
    """Top surface z (m, world) of the worktop + sloped drainboard at horizontal (_x, y)."""
    if y > DRAIN_FOOT_Y:
        return WORKTOP_TOP_Z + (y - DRAIN_FOOT_Y) * math.tan(math.radians(slope_deg))
    return WORKTOP_TOP_Z


def bottle_start_pose(
    spec: BottleSpec = ACTIVE_BOTTLE,
    slope_deg: float = SLOPE_DEG,
    release_up_m: float = RELEASE_UP_M,
    clear_m: float = START_CLEAR_M,
) -> tuple[np.ndarray, np.ndarray]:
    """World (position, 3x3 rotation) of the bottle resting on the drainboard ``release_up_m`` up-slope from the foot."""
    u_hat, n_hat = slope_vectors(slope_deg)
    r = 0.5 * spec.diameter
    foot = np.array([ROLL_LANE_X, DRAIN_FOOT_Y, WORKTOP_TOP_Z], dtype=float)
    contact = foot + release_up_m * u_hat
    pos = contact + (r + clear_m) * n_hat
    return pos, BOTTLE_LYING_ROT.copy()


def bottle_inertia(spec: BottleSpec = ACTIVE_BOTTLE, rot_factor: float = ROT_INERTIA_FACTOR) -> tuple[float, float, float]:
    """Principal moments of inertia (Ixx, Iyy, Izz) in kg m^2 in bottle frame B (cylinder axis along z)."""
    m, r, h = spec.mass, 0.5 * spec.diameter, spec.height
    ixx = m * (0.25 * r * r + (1.0 / 12.0) * h * h)
    izz = rot_factor * m * r * r
    return (ixx, ixx, izz)


def tau_roll(
    spec: BottleSpec = ACTIVE_BOTTLE,
    decel_mps2: float = ROLL_DECEL_MPS2,
    rot_factor: float = ROT_INERTIA_FACTOR,
) -> float:
    """Magnitude (N m) of opposing rolling-resistance torque producing linear deceleration ``decel_mps2`` on a flat."""
    r = 0.5 * spec.diameter
    return spec.mass * decel_mps2 * r * (1.0 + rot_factor)


def settle_hold_force(spec: BottleSpec = ACTIVE_BOTTLE, slope_deg: float = SLOPE_DEG) -> tuple[float, float, float]:
    """Uphill force +m g sin(theta) u_hat (N, world) holding the bottle stationary on the start mark during reset settle."""
    u_hat, _ = slope_vectors(slope_deg)
    mag = spec.mass * GRAVITY * math.sin(math.radians(slope_deg))
    f = mag * u_hat
    return (float(f[0]), float(f[1]), float(f[2]))


def idle_coaster_clearance_m(spec: BottleSpec = ACTIVE_BOTTLE) -> float:
    """Minimum x-gap (m) between the lying bottle (length ``spec.height`` along x) at ``ROLL_LANE_X`` and the cork
    coaster outer rim at ``COASTER_X``."""
    return abs(ROLL_LANE_X - COASTER_X) - COASTER_OUT_R - 0.5 * spec.height


def analytic_roll_predictions(
    spec: BottleSpec = ACTIVE_BOTTLE,
    slope_deg: float = SLOPE_DEG,
    release_up_m: float = RELEASE_UP_M,
    decel_mps2: float = ROLL_DECEL_MPS2,
    rot_factor: float = ROT_INERTIA_FACTOR,
    ang_damp: float = BOTTLE_ANGULAR_DAMPING,
) -> dict[str, float | str]:
    """240 Hz substep rolling integration including polygonal cylinder start, contact-gated ``decel_mps2``, and ``ang_damp``."""
    th = math.radians(slope_deg)
    dt_sub = 1.0 / 240.0
    a_grav = GRAVITY * math.sin(th) / (1.0 + rot_factor)
    # Start loss the simulator shows on the gentle slope (~0.068 m/s^2 averaged over the 140 mm start roll, fitted to an
    # idle trace): its contact friction holds a slow, balanced cylinder back; the cylinder's roll angle does not change it
    start_loss = 0.068
    s, v, t = 0.0, 0.0, 0.0
    while s < release_up_m and t < 10.0:
        a = max(0.02, a_grav - decel_mps2 - start_loss - (rot_factor / (1.0 + rot_factor)) * ang_damp * v)
        v += a * dt_sub
        s += v * dt_sub
        t += dt_sub
    v_foot, t_foot = v, t

    flat_to_edge = DRAIN_FOOT_Y - WORKTOP_MIN_Y  # 0.340 m from y = +0.110 to y = -0.230
    d, vf, tf_flat = 0.0, v_foot, 0.0
    dt_150 = dt_300 = dt_edge = math.inf
    while d < flat_to_edge and tf_flat < 10.0 and vf > 1e-4:
        af = decel_mps2 + (rot_factor / (1.0 + rot_factor)) * ang_damp * vf
        vf = max(0.0, vf - af * dt_sub)
        d += vf * dt_sub
        tf_flat += dt_sub
        if dt_150 is math.inf and d >= 0.150:
            dt_150 = tf_flat
        if dt_300 is math.inf and d >= 0.300:
            dt_300 = tf_flat
        if dt_edge is math.inf and d >= flat_to_edge:
            dt_edge = tf_flat
    return {
        "preset": spec.preset,
        "v_foot_mps": round(v_foot, 4),
        "t_foot_s": round(t_foot, 4),
        "dt_150_from_foot_s": round(dt_150, 4),
        "dt_300_from_foot_s": round(dt_300, 4),
        "dt_edge_from_foot_s": round(dt_edge, 4),
        "t_edge_total_s": round(t_foot + dt_edge, 4),
        "tau_roll_nm": round(tau_roll(spec, decel_mps2, rot_factor), 6),
        "idle_coaster_clearance_mm": round(1000.0 * idle_coaster_clearance_m(spec), 2),
    }


# --- Primitive shape builders ---

def worktop_shapes() -> list:
    """Clamped 520 x 360 x 38 mm laminate worktop with a flush non-solid visual edge trim at y = -0.230."""
    slab = box((WORKTOP_D, WORKTOP_W, WORKTOP_H), pos=(0.0, 0.0, 0.0), color=WORKTOP_RGB, friction=WORKTOP_FRICTION)
    stripe_y = -0.5 * WORKTOP_W + 0.5 * CLIFF_STRIPE_W
    stripe = box(
        (WORKTOP_D, CLIFF_STRIPE_W, 0.0004),
        pos=(0.0, stripe_y, 0.5 * WORKTOP_H + 0.0002),
        color=CLIFF_STRIPE_RGB,
        solid=False,
    )
    return [slab, stripe]


def drainboard_shapes() -> list:
    """Sloped 360 x 170 x 12 mm melamine drainboard with a flush non-solid start-line tape mark at 140 mm up-slope."""
    board = box((DRAIN_D, DRAIN_L, DRAIN_T), pos=(0.0, 0.0, 0.0), color=DRAIN_RGB, friction=DRAIN_FRICTION)
    tape_y = RELEASE_UP_M - 0.5 * DRAIN_L  # local y along u_hat from board centre
    tape = box(
        (DRAIN_D - 0.020, 0.003, 0.0004),
        pos=(0.0, tape_y, 0.5 * DRAIN_T + 0.0002),
        color=START_TAPE_RGB,
        solid=False,
    )
    return [board, tape]


def bottle_shapes(spec: BottleSpec = ACTIVE_BOTTLE) -> list:
    """One seamless solid cylinder collider plus non-solid cap (+z, proud +0.4 mm to prevent top-disc z-fighting) and
    label band so cap-up orientation and spin read unmistakably from overhead and wrist cameras."""
    r, h, cap_h = 0.5 * spec.diameter, spec.height, spec.cap_height
    core = cylinder(r, h, pos=(0.0, 0.0, 0.0), color=spec.glass_rgb, friction=BOTTLE_FRICTION)
    cap_vis = cylinder(
        r + 0.0003,
        cap_h + 0.0004,
        pos=(0.0, 0.0, 0.5 * h - 0.5 * cap_h + 0.0002),
        color=spec.cap_rgb,
        solid=False,
    )
    band_vis = box(
        (2.0 * r + 0.0008, 0.012, 0.056),
        pos=(0.0, 0.0, -0.012),
        color=spec.band_rgb,
        solid=False,
    )
    return [core, cap_vis, band_vis]


def coaster_shapes() -> list:
    """100 mm cork coaster (4 mm floor + 3 mm raised rim around 92 mm inner recess; body origin at underside centre)."""
    shapes = [
        cylinder(
            COASTER_OUT_R,
            COASTER_FLOOR_H,
            pos=(0.0, 0.0, 0.5 * COASTER_FLOOR_H),
            color=COASTER_RGB,
            friction=COASTER_FRICTION,
        )
    ]
    wall_t = COASTER_OUT_R - COASTER_IN_R  # 4.0 mm
    mid_r = 0.5 * (COASTER_IN_R + COASTER_OUT_R)  # 48.0 mm
    chord = 2.0 * mid_r * math.sin(math.pi / COASTER_STAVES) + 2.0 * wall_t * math.tan(math.pi / COASTER_STAVES)
    z_rim = COASTER_FLOOR_H + 0.5 * COASTER_RIM_H
    for k in range(COASTER_STAVES):
        phi = 2.0 * math.pi * k / COASTER_STAVES
        c, s = math.cos(phi), math.sin(phi)
        rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)
        shapes.append(
            box(
                (wall_t, chord, COASTER_RIM_H),
                pos=(mid_r * c, mid_r * s, z_rim),
                quat=matrix_to_quat(rot),
                color=COASTER_RIM_RGB,
                friction=COASTER_FRICTION,
            )
        )
    return shapes


def catch_mat_shapes() -> list:
    """340 x 160 x 10 mm foam catch pad on the bench below the open right worktop edge."""
    return [box((CATCH_MAT_D, CATCH_MAT_W, CATCH_MAT_H), pos=(0.0, 0.0, 0.0), color=CATCH_MAT_RGB, friction=CATCH_MAT_FRICTION)]


def check_all_presets() -> dict[str, dict[str, float | str]]:
    """Roll kinematics and coaster clearance for every preset in ``BOTTLE_PRESETS``."""
    return {name: analytic_roll_predictions(spec) for name, spec in BOTTLE_PRESETS.items()}


if __name__ == "__main__":
    print(json.dumps({"active_preset": BOTTLE_PRESET, "presets": check_all_presets()}, indent=2))
