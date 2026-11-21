"""Product figures, their sources, geometry and thresholds for ``pixi_level``: squaring a smartphone on a Manfrotto
PIXI push-button ball-head mini tripod (MTPIXI-B) with the Manfrotto PIXI smartphone clamp (MCPIXI), sold separately.

Every product size, mass, angle, spring stiffness, torque cap, travel and scoring limit lives in this one file. Each key
figure in ``FIGURES`` names its source kind (``maker``, ``standard``, ``listing``, ``checked`` or ``estimate``), the
source itself (the maker's page and words, or how the estimate was derived) and how the first minutes of a real trial
check it on the bought product. ``head.py``, ``task.py``, ``checks.py``, ``trials.py``, ``demo.py`` and
``test_pixi_level.py`` read the figures from here.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from mechkit import Shape, axis_angle, box, capsule, cylinder, matrix_to_quat, sphere

SOURCE_KINDS = ("maker", "standard", "listing", "checked", "estimate")
TRIPOD_PAGE = "https://www.manfrotto.com/global-en/pixi-mini-tripod-black-mtpixi-b/"
CLAMP_PAGE = "https://www.manfrotto.com/us-en/pixi-clamp-for-smartphone-with-multiple-attachments-mcpixi/"
PHONE_PAGE = "https://support.apple.com/en-gb/111831"


@dataclass(frozen=True)
class FigureSpec:
    """One product or mechanism figure, the constant(s) carrying it, its source kind, its source (the maker's page and
    words, or how an estimate was derived) and how the first minutes of a real trial check it on the bought product."""

    figure: str
    constants: str
    value_now: str
    kind: str
    source: str
    check: str

    def __post_init__(self) -> None:
        if self.kind not in SOURCE_KINDS:
            raise ValueError(f"{self.constants}: kind must be one of {SOURCE_KINDS}, got {self.kind!r}")
        if self.kind in ("maker", "listing") and "https://" not in self.source:
            raise ValueError(f"{self.constants}: a {self.kind} source gives its URL")
        if self.kind == "estimate" and not self.source.startswith("estimate: "):
            raise ValueError(f"{self.constants}: an estimate says how it was derived ('estimate: ...')")


# --- named commercial products ---
# Sold separately: the kit MKPIXICLAMP-BK holds the maker's Universal Smartphone Clamp (0.215 kg in all), not MCPIXI
TRIPOD_PRODUCT = "Manfrotto PIXI mini tripod MTPIXI-B with the Manfrotto PIXI smartphone clamp MCPIXI"
PHONE_PRODUCT = "Apple iPhone 15"

# --- physical & solver constants ---
GRAVITY = 9.81  # m/s^2
OVER = 0.0005  # m: initial settling clearance above the bench top
CONTACT_THIN = 0.0005  # m: collider contact offset (crisp 3 mm button stroke and 7.8 mm phone pinch)

# --- scene placement on the station bench (m; x away from the arms, y left, z up from bench top) ---
TRI_XY = (0.26, 0.00)  # m: tripod centre on the station bench
BUTTON_YAW_DEG = 85.0  # deg: red button faces right-and-slightly-front (-cos 85, -sin 85), towards right wrist and top cam
TRI_YAW = math.radians(BUTTON_YAW_DEG)

# --- Manfrotto PIXI mini tripod (MTPIXI-B) ---
TRI_H = 0.135  # m (maker: maximum height 13.5 cm): bench to the top of the 1/4"-20 camera platform (135.0 mm)
LEG_SPAN = 0.180  # m (estimate: 112 mm legs from a 75 mm hub, the maker's 18.5 cm folded length less the head): 180.0 mm
FOOT_R = 0.5 * LEG_SPAN  # m: circumradius of the three feet from TRI_XY (90.0 mm)
FOOT_INRADIUS = 0.5 * FOOT_R  # m: support-triangle edge distance from TRI_XY (45.0 mm)
# Local leg angles so world feet stand at (0, +120, -120) deg: +60 deg support edge sits under the (+10, -20) deg phone lean
LEG_ANGLES_DEG = (-85.0, 35.0, 155.0)
TRI_MASS = 0.190  # kg (maker: weight 0.19 kg): the whole tripod, ball head and button included (190.0 g)
BUTTON_MASS = 0.008  # kg (estimate: a plastic button and wedge): moving push-button (8.0 g, part of TRI_MASS)
C_TILT_MASS = 0.025  # kg (estimate: a ball and stem): gimbal carrier at BALL_Z (25.0 g, part of TRI_MASS)
TRI_BODY_MASS = TRI_MASS - BUTTON_MASS - C_TILT_MASS  # kg: stationary legs and neck housing (157.0 g)
TRI_FRICTION = 0.80  # (estimate: rubber feet on a dry bench top, mid-range for rubber on wood or laminate)

BALL_Z = 0.100  # m (estimate: 35 mm under the maker's 13.5 cm platform top): ball centre above the bench (100.0 mm)
BALL_R = 0.011  # m (estimate: a 22 mm ball inside the 34 mm neck): ball radius
HUB_Z = 0.075  # m (estimate: 6 cm under the platform top): where the three legs meet the neck hub (75.0 mm)
HEAD_D = 0.034  # m (estimate: a neck 12 mm wider than the ball): outside diameter of the ball-head neck (34.0 mm)
HEAD_R = 0.5 * HEAD_D  # m: 17.0 mm
HEAD_Z_LOW = 0.070  # m: bottom of the cylindrical neck housing
HEAD_Z_HIGH = 0.109  # m: top rim of the neck socket (clears the tilting stem at +/- 35 deg)
FOOT_PAD_R = 0.0075  # m: radius of each rubber foot hemisphere/pad (7.5 mm)
LEG_R = 0.0065  # m: half-thickness of each moulded PIXI leg (6.5 mm)

# --- red push-button on the -x face of the PIXI neck housing ---
BUTTON_Z = 0.103  # m (estimate: beside the ball centre): button centre height above the bench (103.0 mm)
BUTTON_R = 0.009  # m (estimate: a fingertip-sized button on the 34 mm neck): button face radius (18.0 mm across)
BUTTON_PROUD = 0.004  # m (estimate: stroke plus 1 mm): how far the unpressed button face stands out past -HEAD_R
BUTTON_TRAVEL = 0.003  # m (estimate: a short push-button stroke): full inward travel along +x (3.0 mm)
BUTTON_N = 18.0  # N (estimate: a firm one-finger press, so a brush does not free the head): force at full travel
PRESSED_FRACTION = 0.80  # (estimate: the head frees in the last 0.6 mm of the stroke): release point (2.40 mm, 14.4 N)
BUTTON_UNLOCK_M = PRESSED_FRACTION * BUTTON_TRAVEL  # m: 0.0024 m (2.40 mm)
BUTTON_K = BUTTON_N / BUTTON_TRAVEL  # N/m: bilateral spring stiffness (6000.0 N/m)
BUTTON_D = round(2.0 * math.sqrt(BUTTON_K * BUTTON_MASS), 2)  # N s/m: critical damping (13.86 N s/m)
HEAD_PINCH_SPAN = HEAD_D + BUTTON_PROUD  # m: unpressed front-to-back pinch width across button and head back (38.0 mm)

# --- ball-head lock & free friction (tilt about y, roll about x) ---
# LOCK_NM (estimate: 2.7x the 0.56 N m the maker's 1 kg load rating needs 10 cm above the ball at 35 deg of tilt)
LOCK_NM = 1.50  # N m: locking torque on each hinge while the button is out
# FREE_NM (estimate: a light drag under the 0.05 N m the phone puts on the head at the start, so a freed phone flops)
FREE_NM = 0.02  # N m: residual ball drag while the button is pressed past PRESSED_FRACTION
STOP_DEG = 35.0  # deg (maker: front and side tilt 35 deg each way): stops on both tilt and roll
STOP_RAD = math.radians(STOP_DEG)
K_HEAD = 45.0  # N m/rad: stiff spring drive for the bilateral capped-spring lock (1.91 deg spring yield at 1.50 N m)
LOCK_SOLVER_SLACK_RAD = 0.009  # rad (~0.52 deg): maximal-coordinate joint solver elastic compliance at 1.50 N m before slip
D_HEAD_LOCK = 0.80  # N m s/rad: near-critical rotational damping while locked
D_HEAD_FREE = 0.012  # N m s/rad: light viscous damping while free so gravity flop reaches the 35 deg stop crisply

START_TILT_DEG = 10.0  # deg (task setting, set with a level app at reset): screen tilt about +y (screen faces up 10 deg)
START_ROLL_DEG = -20.0  # deg (task setting, set with a level app at reset): top-edge roll about +x (+y end dips 20 deg)
START_TILT_RAD = math.radians(START_TILT_DEG)
START_ROLL_RAD = math.radians(START_ROLL_DEG)

# --- PIXI smartphone clamp (MCPIXI) and gimbal ball carrier ---
CLAMP_BODY_MASS = 0.041  # kg (maker: weight 0.041 kg): the MCPIXI clamp (41.0 g)
CLAMP_MASS = C_TILT_MASS + CLAMP_BODY_MASS  # kg: everything that turns with the phone but the phone (66.0 g)
C_TILT_INERTIA = (2.0e-3, 2.0e-3, 2.0e-3)  # kg m^2: matched to phone+clamp inertia so PhysX transmits LOCK_NM cleanly
CLAMP_INERTIA = (1.5e-3, 1.5e-3, 1.5e-3)  # kg m^2: matched middle-link rotational inertia between c_tilt and phone weld
BUTTON_INERTIA = (8.0e-5, 8.0e-5, 8.0e-5)  # kg m^2: keeps prismatic button stop crisp under 28 N pad squeeze

PLATFORM_R = 0.0175  # m: 35.0 mm circular camera platform at z = TRI_H
PLATFORM_T = 0.006  # m: platform thickness (z = 129.0 .. 135.0 mm)
STEM_R = 0.0055  # m: steel ball neck stem radius (11.0 mm diameter)
CLAMP_W = 0.030  # m (maker: width 3 cm): clamp jaw width along y (30.0 mm, leaving 58.8 mm free phone length each side)
# CLAMP_BASE_H, CLAMP_TOP_H (estimate: 8 + 7 mm jaws around the 71.6 mm phone make the maker's 8.7 cm clamp height)
CLAMP_BASE_H = 0.008  # m: lower clamp jaw height above TRI_H (z = 135.0 .. 143.0 mm)
CLAMP_BACK_T = 0.006  # m: vertical clamp spine thickness behind the phone (+x side)
CLAMP_TOP_H = 0.007  # m: upper clamp jaw height above the phone's top edge
WELD_GAP = 0.0006  # m: 0.6 mm clearance between welded phone and clamp jaws (welded bodies keep collision on)

# --- Apple iPhone 15 (maker: Apple's tech specs, PHONE_PAGE) ---
PHONE_L = 0.1476  # m (maker: height 147.6 mm): along y in landscape
PHONE_W = 0.0716  # m (maker: width 71.6 mm): along z in landscape
PHONE_T = 0.0078  # m (maker: depth 7.80 mm): along x
PHONE_MASS = 0.171  # kg (maker: weight 171 grams)

PHONE_LOW_Z = TRI_H + CLAMP_BASE_H + WELD_GAP  # m: level phone bottom edge above bench (143.6 mm)
PHONE_CENTER_Z = PHONE_LOW_Z + 0.5 * PHONE_W  # m: level phone centre above bench (179.4 mm; 79.4 mm above BALL_Z)
PHONE_HIGH_Z = PHONE_LOW_Z + PHONE_W  # m: level phone top edge above bench (215.2 mm)

# Left-hand grasp point in the phone's body frame (origin at phone centre): 58 mm towards the +y left end, 23 mm below
# the phone's horizontal midline (12.8 mm above bottom edge; world z = 133.2 -> 156.9 mm across the 20 deg sweep, <= 160 mm).
PHONE_GRASP_LOCAL = (0.0, 0.058, -0.023)

# --- scoring limits (identical in task.py, task.json and README.md) ---
LEVEL_DEG = 3.0  # deg: full score 1.0 when both |tilt| and |roll| are within 3.0 deg of level
PARTIAL_DEG = 8.0  # deg: partial score 0.5 when both are within 8.0 deg and at least one exceeds 3.0 deg
FOOT_LIFT_MAX = 0.010  # m: whole-trial zero rule if any tripod foot rises more than 10.0 mm off the bench
FOOT_SLIDE_MAX = 0.020  # m: whole-trial zero rule if any tripod foot moves more than 20.0 mm from its tape mark
GRIPPER_CLEAR = 0.030  # m: both grippers at least 30.0 mm from the phone and the tripod/clamp/button at the end
BUTTON_OUT_MAX = 0.0005  # m: button sprung back within 0.5 mm of its outer stop (head locked)
REST_LIN_MAX = 0.02  # m/s: linear speed threshold for "at rest"
REST_ANG_MAX = 0.05  # rad/s: angular speed threshold (~2.9 deg/s) for "at rest"
REST_HOLD_S = 1.0  # s: every body at rest, button out and grippers clear for 1.0 s
LOCK_SLIP_MAX_DEG = 0.2  # deg: whole-trial zero rule if either locked hinge slips more than 0.2 deg, summed over the trial

# --- colours (sRGB) ---
PIXI_BLACK = (0.14, 0.14, 0.15)
STEEL_SILVER = (0.76, 0.77, 0.79)
MANFROTTO_RED = (0.84, 0.10, 0.10)  # the PIXI's button: "push a red button" (review of the MTPIXI-B, techwalls.com)
CLAMP_DARK = (0.18, 0.18, 0.20)
RUBBER_RED = (0.72, 0.12, 0.12)
PHONE_SLATE = (0.20, 0.22, 0.25)
SCREEN_LIT = (0.82, 0.88, 0.95)
SCREEN_BAR = (0.18, 0.44, 0.78)
TAPE_YELLOW = (0.90, 0.78, 0.18)

# --- every key figure, its source and its check on the bought product (Check 19, Check 50, Reader Addendum §1) ---
FIGURES: tuple[FigureSpec, ...] = (
    FigureSpec(
        "Tripod height and mass (MTPIXI-B)",
        "TRI_H, TRI_MASS",
        f"{1000 * TRI_H:.1f} mm, {1000 * TRI_MASS:.1f} g",
        "maker",
        f"maker: maximum height 13.5 cm, weight 0.19 kg ({TRIPOD_PAGE})",
        "ruler from the bench to the platform top; kitchen scale (1 min)",
    ),
    FigureSpec(
        "Ball-head stops",
        "STOP_DEG",
        f"+/-{STOP_DEG:.1f} deg",
        "maker",
        f"maker: front and side tilt 35 deg each way ({TRIPOD_PAGE})",
        "phone level app at each stop, button held (1 min)",
    ),
    FigureSpec(
        "Clamp mass and width (MCPIXI)",
        "CLAMP_BODY_MASS, CLAMP_W",
        f"{1000 * CLAMP_BODY_MASS:.1f} g, {1000 * CLAMP_W:.1f} mm",
        "maker",
        f"maker: weight 0.041 kg, width 3 cm, opening 6-10.4 cm ({CLAMP_PAGE})",
        "kitchen scale and caliper (1 min)",
    ),
    FigureSpec(
        "Phone size and mass (iPhone 15)",
        "PHONE_L, PHONE_W, PHONE_T, PHONE_MASS",
        f"{1000 * PHONE_L:.1f} x {1000 * PHONE_W:.1f} x {1000 * PHONE_T:.1f} mm, {1000 * PHONE_MASS:.1f} g",
        "maker",
        f"maker: height 147.6 mm, width 71.6 mm, depth 7.80 mm, weight 171 grams ({PHONE_PAGE})",
        "none needed",
    ),
    FigureSpec(
        "Foot circle",
        "LEG_SPAN",
        f"{1000 * LEG_SPAN:.1f} mm",
        "estimate",
        "estimate: 112 mm legs from a 75 mm hub, the maker's 18.5 cm folded length less the head",
        "ruler across the three feet (1 min)",
    ),
    FigureSpec(
        "Foot friction",
        "TRI_FRICTION",
        f"{TRI_FRICTION:.2f}",
        "estimate",
        "estimate: rubber feet on a dry bench top, mid-range for rubber on wood or laminate",
        "tripod on a board tilted until it slides; phone level app on the board, mu = tan(angle) (2 min)",
    ),
    FigureSpec(
        "Ball, neck and button heights and pinch span",
        "BALL_Z, HEAD_D, BUTTON_Z, HEAD_PINCH_SPAN",
        f"{1000 * BALL_Z:.1f}, {1000 * HEAD_D:.1f}, {1000 * BUTTON_Z:.1f}, {1000 * HEAD_PINCH_SPAN:.1f} mm",
        "estimate",
        "estimate: ball centre 35 mm under the maker's 13.5 cm platform top, a 34 mm neck, the button beside the ball",
        "ruler from the bench; caliper across button and neck (2 min)",
    ),
    FigureSpec(
        "Clamp jaws",
        "CLAMP_BASE_H, CLAMP_TOP_H",
        f"{1000 * CLAMP_BASE_H:.1f}, {1000 * CLAMP_TOP_H:.1f} mm",
        "estimate",
        "estimate: 8 + 7 mm jaws around the 71.6 mm phone make the maker's 8.7 cm clamp height",
        "caliper on the clamp holding the phone (1 min)",
    ),
    FigureSpec(
        "Ball-head locking torque (button out)",
        "LOCK_NM",
        f"{LOCK_NM:.2f} N m",
        "estimate",
        "estimate: 2.7x the 0.56 N m the maker's 1 kg load rating needs 10 cm above the ball at 35 deg of tilt",
        "luggage scale on a string at the phone's end, pulled at 90 deg until the head slips, both axes (3 min)",
    ),
    FigureSpec(
        "Ball drag with the button pressed",
        "FREE_NM",
        f"{FREE_NM:.2f} N m",
        "estimate",
        "estimate: a light drag under the 0.05 N m the phone puts on the head at the start, so a freed phone flops",
        "button held: the smallest lean from which the phone drops, phone level app (2 min)",
    ),
    FigureSpec(
        "Button force, stroke and release point",
        "BUTTON_N, BUTTON_TRAVEL, PRESSED_FRACTION",
        f"{BUTTON_N:.1f} N, {1000 * BUTTON_TRAVEL:.1f} mm, {PRESSED_FRACTION:.2f} ({1000 * BUTTON_UNLOCK_M:.2f} mm)",
        "estimate",
        "estimate: a firm one-finger press so a brush does not free the head; it frees in the last 0.6 mm",
        "tripod on a tared kitchen scale, button pressed to the stop and to the release point; caliper depth rod (3 min)",
    ),
    FigureSpec(
        "Moving parts inside the tripod",
        "BUTTON_MASS, C_TILT_MASS",
        f"{1000 * BUTTON_MASS:.1f} g, {1000 * C_TILT_MASS:.1f} g",
        "estimate",
        "estimate: a plastic button and wedge, a ball and stem, both part of the maker's 0.19 kg",
        "not separable without opening the head",
    ),
)


# --- geometry helpers ---


def rx(angle_rad: float) -> np.ndarray:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


def ry(angle_rad: float) -> np.ndarray:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def rz(angle_rad: float) -> np.ndarray:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def ball_anchor_world(table_top: float) -> tuple[float, float, float]:
    return (TRI_XY[0], TRI_XY[1], table_top + OVER + BALL_Z)


def button_built_pos(table_top: float) -> tuple[float, float, float]:
    return (TRI_XY[0], TRI_XY[1], table_top + OVER + BUTTON_Z)


def button_slide_axis() -> tuple[float, float, float]:
    """World unit vector along which ``button`` slides into the PIXI neck housing."""
    return (math.cos(TRI_YAW), math.sin(TRI_YAW), 0.0)


def button_face_world(table_top: float) -> np.ndarray:
    """World position of the unpressed red button's outer face centre."""
    inward = np.array(button_slide_axis(), dtype=float)
    return np.array(button_built_pos(table_top), dtype=float) - (HEAD_R + BUTTON_PROUD) * inward


def clamp_built_pos(table_top: float) -> tuple[float, float, float]:
    return (TRI_XY[0], TRI_XY[1], table_top + OVER + TRI_H)


def phone_built_pos(table_top: float) -> tuple[float, float, float]:
    return (TRI_XY[0], TRI_XY[1], table_top + OVER + PHONE_CENTER_Z)


def foot_points_local() -> np.ndarray:
    """3x3 array of the three foot-pad bottom points in ``tripod``'s body frame (origin at bench contact level)."""
    pts = []
    for deg in LEG_ANGLES_DEG:
        a = math.radians(deg)
        pts.append([FOOT_R * math.cos(a), FOOT_R * math.sin(a), 0.0])
    return np.asarray(pts, dtype=float)


def foot_marks_world_xy() -> np.ndarray:
    """3x2 world (x, y) coordinates of the three bench tape marks at the start."""
    r = rz(TRI_YAW)
    local = foot_points_local() @ r.T
    return local[:, :2] + np.array(TRI_XY, dtype=float)


def head_rotation(tilt_rad: float, roll_rad: float) -> np.ndarray:
    """World rotation of ``clamp`` and ``phone`` when ``tripod`` is upright and joints stand at ``(tilt_rad, roll_rad)``."""
    return ry(tilt_rad) @ rx(roll_rad)


def phone_angles_deg(phone_rot: np.ndarray) -> tuple[float, float]:
    """(tilt_deg, roll_deg) of ``phone`` against world gravity, as read by a phone level app on its side and top edge.

    In ``phone``'s built frame:
    - ``-x`` is the screen normal (level app against the face/side reads ``tilt_deg = arcsin(-R[2, 0])``);
    - ``+y`` is along the 147.6 mm top edge towards the left (level app on the top edge reads ``roll_deg = arcsin(R[2, 1])``).
    At ``(0, 0)`` both read ``0.0 deg``; near level they equal the joint angles on an upright tripod to < 0.01 deg.
    """
    r = np.asarray(phone_rot, dtype=float)
    tilt_deg = math.degrees(math.asin(max(-1.0, min(1.0, float(-r[2, 0])))))
    roll_deg = math.degrees(math.asin(max(-1.0, min(1.0, float(r[2, 1])))))
    return tilt_deg, roll_deg


def gravity_torque_nm(tilt_rad: float, roll_rad: float) -> tuple[float, float]:
    """Static gravity torque (N m) about (tilt_y, roll_x) at the ball from ``clamp + phone`` on an upright tripod."""
    # Clamp body CoM 47 mm above the ball, phone CoM (PHONE_CENTER_Z - BALL_Z) 79.4 mm; the carrier sits at the ball
    h_clamp = (TRI_H + 0.012) - BALL_Z
    h_phone = PHONE_CENTER_Z - BALL_Z
    mgh = (CLAMP_BODY_MASS * h_clamp + PHONE_MASS * h_phone) * GRAVITY
    # Potential energy U = mgh * cos(tilt) * cos(roll):
    # tau_tilt = -dU/d(tilt) = mgh * sin(tilt) * cos(roll), tau_roll = -dU/d(roll) = mgh * cos(tilt) * sin(roll)
    return (
        mgh * math.sin(tilt_rad) * math.cos(roll_rad),
        mgh * math.cos(tilt_rad) * math.sin(roll_rad),
    )


def static_tipping_summary() -> dict:
    """Hand-verifiable statics check of locked hold, free flop and support-triangle margin."""
    tau_t0, tau_r0 = gravity_torque_nm(START_TILT_RAD, START_ROLL_RAD)
    h_clamp = (TRI_H + 0.012) - BALL_Z
    h_phone = PHONE_CENTER_Z - BALL_Z
    total_mass = TRI_MASS + CLAMP_BODY_MASS + PHONE_MASS  # TRI_MASS already holds the carrier and the button
    eff_arm = (CLAMP_BODY_MASS * h_clamp + PHONE_MASS * h_phone) / total_mass
    r0 = head_rotation(START_TILT_RAD, START_ROLL_RAD)
    com_start = eff_arm * (r0 @ np.array([0.0, 0.0, 1.0]))
    r_stop = head_rotation(STOP_RAD, -STOP_RAD)
    com_stop = eff_arm * (r_stop @ np.array([0.0, 0.0, 1.0]))
    tip_edge_nm = total_mass * GRAVITY * FOOT_INRADIUS
    tip_vertex_nm = total_mass * GRAVITY * FOOT_R
    return {
        "total_mass_g": round(1000 * total_mass, 1),
        "start_gravity_torque_nm": {"tilt": round(tau_t0, 4), "roll": round(tau_r0, 4)},
        "free_friction_nm": FREE_NM,
        "lock_torque_nm": LOCK_NM,
        "flops_when_freed_at_start": abs(tau_r0) > FREE_NM and abs(tau_t0) > FREE_NM,
        "support_inradius_mm": round(1000 * FOOT_INRADIUS, 1),
        "com_offset_start_mm": round(1000 * float(np.hypot(com_start[0], com_start[1])), 1),
        "com_offset_stop_mm": round(1000 * float(np.hypot(com_stop[0], com_stop[1])), 1),
        "tripod_tip_torque_edge_nm": round(tip_edge_nm, 3),
        "tripod_tip_torque_vertex_nm": round(tip_vertex_nm, 3),
        "one_sided_button_moment_nm": round(BUTTON_N * BUTTON_Z, 3),
    }


# --- shape builders ---


def _capsule_between(p0, p1, radius: float, color, friction=None) -> Shape:
    a, b = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
    mid = 0.5 * (a + b)
    vec = b - a
    dist = float(np.linalg.norm(vec))
    u = vec / dist
    # Local z aligned with u
    z_axis = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(u, z_axis))) > 0.9999:
        rot = np.eye(3) if u[2] > 0 else rx(math.pi)
    else:
        ax = np.cross(z_axis, u)
        rot = axis_angle(ax, math.acos(max(-1.0, min(1.0, float(np.dot(z_axis, u))))))
    return capsule(radius, max(0.002, dist), pos=tuple(mid), quat=matrix_to_quat(rot), color=color, friction=friction)


def bench_mark_shapes() -> list[Shape]:
    """Three high-contrast yellow tape squares under the tripod's three feet (drawn only, 0.6 mm thin)."""
    shapes: list[Shape] = []
    for x, y in foot_marks_world_xy():
        shapes.append(box((0.024, 0.024, 0.0006), pos=(float(x), float(y), 0.0003), color=TAPE_YELLOW, solid=False))
    return shapes


def tripod_shapes() -> list[Shape]:
    """Manfrotto PIXI base (origin at (TRI_XY, TOP + OVER)): three splayed legs with rubber feet, neck housing, flat
    rear pinch boss at +HEAD_R and visible stainless ball in the socket."""
    shapes: list[Shape] = []
    foot_f = (TRI_FRICTION, TRI_FRICTION)
    for deg in LEG_ANGLES_DEG:
        a = math.radians(deg)
        ca, sa = math.cos(a), math.sin(a)
        p_foot = (FOOT_R * ca, FOOT_R * sa, FOOT_PAD_R)
        p_hub = (0.011 * ca, 0.011 * sa, HUB_Z)
        shapes.append(sphere(FOOT_PAD_R, pos=p_foot, color=PIXI_BLACK, friction=foot_f))
        shapes.append(_capsule_between(p_foot, p_hub, LEG_R, PIXI_BLACK, friction=foot_f))
    # Cylindrical neck housing around the ball socket
    neck_h = HEAD_Z_HIGH - HEAD_Z_LOW
    shapes.append(cylinder(HEAD_R, neck_h, pos=(0.0, 0.0, 0.5 * (HEAD_Z_LOW + HEAD_Z_HIGH)), axis="z", color=PIXI_BLACK))
    # Rear pinch boss flush at x = +HEAD_R (+17.0 mm) so the right hand's opposing finger pad seats squarely
    shapes.append(box((0.006, 0.020, 0.022), pos=(HEAD_R - 0.003, 0.0, BUTTON_Z), color=PIXI_BLACK))
    # Drawn stainless ball inside the socket (solid=False: kinematic joint carries the constraint)
    shapes.append(sphere(BALL_R, pos=(0.0, 0.0, BALL_Z), color=STEEL_SILVER, solid=False))
    return shapes


def button_shapes() -> list[Shape]:
    """Red Manfrotto push-button (origin at (TRI_XY, TOP + OVER + BUTTON_Z)): protrudes to x = -(HEAD_R + BUTTON_PROUD)
    = -21.0 mm when unpressed (38.0 mm total pinch span across head), sliding +3.0 mm along +x into the neck."""
    # Stem from inside the housing out to -(HEAD_R + BUTTON_PROUD) = -0.021 m
    depth = 0.008  # 8 mm button cap + stem thickness; outer face at -0.021 m -> centre at -0.021 + 0.004 = -0.017 m
    cx = -(HEAD_R + BUTTON_PROUD) + 0.5 * depth
    return [
        cylinder(BUTTON_R, depth, pos=(cx, 0.0, 0.0), axis="x", color=MANFROTTO_RED),
        box((0.003, 0.014, 0.014), pos=(cx - 0.5 * depth + 0.0015, 0.0, 0.0), color=MANFROTTO_RED),
    ]


def carrier_shapes(pass_bodies: tuple[str, ...]) -> list[Shape]:
    """Tiny hidden gimbal carrier at the ball centre (passes all task bodies)."""
    return [box((0.006, 0.006, 0.006), pos=(0.0, 0.0, 0.0), color=STEEL_SILVER, passes=pass_bodies)]


def clamp_shapes() -> list[Shape]:
    """Tilting ball stem, 35 mm camera platform (top at z = 0 in clamp frame = TRI_H) and MCPIXI smartphone clamp jaws.
    Origin at (TRI_XY, TOP + OVER + TRI_H). Built with >= 0.6 mm clearance around ``phone`` so the weld is strain-free."""
    stem_low = (BALL_Z + 0.004) - TRI_H  # starts 4 mm above ball centre; passes tripod neck socket cleanly
    stem_high = -PLATFORM_T
    low_jaw_z = 0.5 * CLAMP_BASE_H
    phone_rel_low = CLAMP_BASE_H + WELD_GAP
    phone_rel_high = phone_rel_low + PHONE_W
    top_jaw_low = phone_rel_high + WELD_GAP
    spine_x = 0.5 * PHONE_T + WELD_GAP + 0.5 * CLAMP_BACK_T
    spine_h = (top_jaw_low + CLAMP_TOP_H) - 0.0
    jaw_depth_x = PHONE_T + 2.0 * WELD_GAP + CLAMP_BACK_T + 0.004  # cradles front & back with 0.6 mm gap to phone
    jaw_cx = 0.5 * CLAMP_BACK_T
    return [
        cylinder(STEM_R, stem_high - stem_low, pos=(0.0, 0.0, 0.5 * (stem_low + stem_high)), axis="z", color=STEEL_SILVER, passes=("tripod", "button")),
        cylinder(PLATFORM_R, PLATFORM_T, pos=(0.0, 0.0, -0.5 * PLATFORM_T), axis="z", color=PIXI_BLACK),
        # Lower jaw step + red grip pad visual
        box((jaw_depth_x, CLAMP_W, CLAMP_BASE_H), pos=(jaw_cx, 0.0, low_jaw_z), color=CLAMP_DARK),
        box((PHONE_T + 0.002, CLAMP_W - 0.002, 0.0012), pos=(0.0, 0.0, CLAMP_BASE_H - 0.0006), color=RUBBER_RED, solid=False),
        # Rear vertical spine (+x behind phone)
        box((CLAMP_BACK_T, CLAMP_W, spine_h), pos=(spine_x, 0.0, 0.5 * spine_h), color=CLAMP_DARK),
        # Upper jaw hook above phone's top edge
        box((jaw_depth_x, CLAMP_W, CLAMP_TOP_H), pos=(jaw_cx, 0.0, top_jaw_low + 0.5 * CLAMP_TOP_H), color=CLAMP_DARK),
        box((PHONE_T + 0.002, CLAMP_W - 0.002, 0.0012), pos=(0.0, 0.0, top_jaw_low + 0.0006), color=RUBBER_RED, solid=False),
    ]


def phone_shapes() -> list[Shape]:
    """iPhone 15 held landscape (origin at phone centre: x thickness 7.8 mm, y length 147.6 mm, z width 71.6 mm).
    One crisp solid box collider for clean parallel-pad pinching, plus lit front screen and rear camera island visuals."""
    front_x = -0.5 * PHONE_T - 0.0002
    back_x = 0.5 * PHONE_T + 0.0008
    return [
        box((PHONE_T, PHONE_L, PHONE_W), pos=(0.0, 0.0, 0.0), color=PHONE_SLATE),
        # Lit front display (-x face) and horizon reference band so 20 deg lean vs 0 deg level is clear at 320x176
        box((0.0004, PHONE_L - 0.007, PHONE_W - 0.007), pos=(front_x, 0.0, 0.0), color=SCREEN_LIT, glow=0.22, solid=False),
        box((0.0005, PHONE_L - 0.024, 0.005), pos=(front_x - 0.0001, 0.0, 0.0), color=SCREEN_BAR, glow=0.15, solid=False),
        # Dynamic island pill near +y bezel and rear camera island on +x back
        box((0.0005, 0.004, 0.018), pos=(front_x - 0.0001, 0.5 * PHONE_L - 0.011, 0.0), color=PIXI_BLACK, solid=False),
        box((0.0016, 0.028, 0.028), pos=(back_x, 0.5 * PHONE_L - 0.022, 0.5 * PHONE_W - 0.022), color=PIXI_BLACK, solid=False),
    ]


if __name__ == "__main__":
    import json

    print(json.dumps({"figures": len(FIGURES), "statics": static_tipping_summary()}, indent=2))
