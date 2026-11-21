"""Single source of truth for the child's bike helmet fit task: named commercial products, dimensions, masses,
hinge/ratchet/band mechanics, bilateral reaction shares, scoring thresholds, documented sources, first-minutes
trial checks, and primitive shape builders.

Every figure that the starting state, serial mechanism, scripted demo, rule tests or real-trial checks rest on
lives here as one constant, paired in ``KEY_FIGURES`` with its true source kind (``"maker"``, ``"standard"``,
``"listing"``, or ``"estimate"`` with its derivation) and how the first minutes of a real trial check it.
"""

from __future__ import annotations

import functools
import json
import math

import numpy as np
from mechkit import box, capsule, cylinder, matrix_to_quat, sphere

SOURCE_KINDS = ("maker", "standard", "listing", "estimate")

# --- documented product & safety-standard URLs ---
NHTSA_FIT_URL = "https://ems.idaho.gov/wp-content/uploads/2026/07/Bike-Helmet-Laws.pdf"
NHTSA_UCDAVIS_URL = "https://health.ucdavis.edu/children/patient-education/Helmet-Safety-Kids"
HELMET_SUPPORT_URL = "https://support.decathlon.de/kinder-fahrradhelm-kh-500-1"
HELMET_LISTING_URL = "https://www.decathlon.co.uk/p/500-children-s-helmet-blue/338990/c209c202m8737457"
HELMET_FIT_ADVICE_URL = "https://www.decathlon.co.uk/p/500-city-cycling-helmet-neon-yellow/895/c132m8326834"
HEAD_MAKER_URL = "https://www.rayher.com/de/styropor-kopf-weiblich-29-cm-3396700"
HEAD_LISTING_URL = "https://www.mosaikshop.se/en/product/styrofoam-head-female-height-29-cm"
HEAD_US_TWIN_URL = "https://shop.floracraft.com/amfile/file/download/file/64/product/4751/"
HEADFORM_EN960_URL = "https://www.cadexinc.com/?p=201"  # a maker of EN 960 test head forms (its size list)

GRAVITY = 9.81  # m/s^2
OVER = 0.001  # m: the head form is built 1 mm over the bench top so frame 0 starts with no contact
CONTACT_THIN = 0.0010  # m: collider contact offset for helmet rim, dial, and retention band

# --- named commercial products (EU product and US twin) ---
HELMET_PRODUCT = (
    "Decathlon B'TWIN KH 500 kids' bike helmet, size S (53 to 56 cm), ref. 8737457 (blue; 8737455 pink); "
    "US twin: Schwinn Thrasher Child dial-fit bike helmet (50 to 56 cm)"
)
HEAD_PRODUCT = (
    "Rayher female styrofoam head form, art. 3396700 (29 cm tall uncut, 55 cm circumference); "
    "US twin: FloraCraft SmoothFoM female styrofoam head RS260/6 (10 in tall, 21 in / 53.3 cm circumference)"
)

# --- scene placement in station frame (m from arm-base midpoint; x forward, y left, z up from bench) ---
OBJECT_JITTER = 0.0  # m: one fixed layout for every seed
HEAD_XY = (0.280, 0.000)  # m: head form center on bench, facing +y (towards the left arm at y = +0.305)

# --- head form figures (the head stands on the bench, taped; cut level just under the chin) ---
HEAD_RAW_H = 0.290  # m (maker: "Styropor-Kopf, weiblich, 29 cm", art. 3396700)
HEAD_CIRC = 0.550  # m (listing: "Head circumference: 55 cm", model 3396700)
HEAD_CIRC_KIND = "listing"

CROWN_TO_BROW = 0.110  # m (estimate: adult-female head proportions for a 55 cm circumference, brow halfway down head)
CROWN_TO_CHIN = 0.210  # m (estimate: 210 mm crown-to-chin after sawing 78 mm of the 290 mm display neck 2 mm below chin)
HEAD_H = CROWN_TO_CHIN + 0.002  # m: the cut head form's height over the bench, crown at 212 mm
HEAD_H_KIND = "estimate"

BROW_Z = HEAD_H - CROWN_TO_BROW  # m: brow band's centre over the bench, 102 mm
BROW_BAND_W = 0.008  # m: 8 mm painted dark reference band across forehead

HEAD_MASS = 0.090  # kg (estimate: ~3.6 L cut EPS head volume at ~25 kg/m^3 expanded polystyrene density)
HEAD_TOTAL_MASS = HEAD_MASS  # kg: the head form alone, taped to the bench (fixed in the simulation)
HEAD_FRICTION = 0.55  # (estimate: dry expanded polystyrene against helmet liner pads)

HEAD_RX = 0.079  # m (estimate: 158 mm ear-to-ear width matching 550 mm head circumference with 188 mm length)
HEAD_RY = 0.094  # m (estimate: 188 mm front-to-back length -> ~545-550 mm ellipse perimeter)
HEAD_CROWN_Z = HEAD_H  # 0.212 m

# --- helmet shell, strap tilt axis, rear dial, and retention band figures ---
HELMET_MASS = 0.220  # kg (listing: 220 g for Decathlon B'TWIN KH 500 size S, 53-56 cm)
HELMET_MASS_KIND = "listing"

DIAL_MASS = 0.015  # kg (estimate: rear dial assembly share within the listed 220 g helmet mass)
BAND_MASS = 0.015  # kg (estimate: retention ring share within the listed 220 g helmet mass)
SHELL_MASS = round(HELMET_MASS - DIAL_MASS - BAND_MASS, 4)  # 0.190 kg -> exact 0.220 kg total

HELMET_L = 0.220  # m (estimate: 188 mm head length plus ~16 mm front/rear EPS liner shell wall)
HELMET_W = 0.180  # m (estimate: 158 mm head width plus ~11 mm lateral EPS liner wall)
HELMET_H = 0.120  # m (estimate: 110 mm crown-to-brow dome plus ~10 mm top shell/liner thickness)
HELMET_SIZE_KIND = "estimate"

PIVOT_Z = BROW_Z + 0.020  # m (estimate: ear-strap Y-anchor axis 20 mm above brow plane, 122 mm over bench)
PIVOT_Y = 0.000  # m (estimate: ear-strap anchor line at mid-coronal plane of head center)
PIVOT_KIND = "estimate"

TILT_MIN_DEG = -5.0  # deg forward stop (pushed down over the eyes: gap ~ 10 mm < 15 mm)
TILT_START_DEG = 13.0  # deg (estimate: front Y-strap slack with two-finger chin-strap clearance lets open shell rock back 13 deg -> gap 45.9 mm, rim top 159 mm <= 160 mm)
TILT_START_KIND = "estimate"

BRIM_ARM = 0.115  # m (estimate: 94 mm forehead semi-axis plus 21 mm liner/visor projection ahead of ear pivot)
BRIM_DZ = 0.000  # m (front shell underside sits at pivot height when level -> gap = 20 mm)
BRIM_THICK = 0.012  # m (estimate: 12 mm front rim thickness; press point on its top is 159 mm over bench at 13 deg)
BRIM_TAB_W = 0.044  # m (estimate: 44 mm central front brow-rim width for closed-finger press or pinch)

# Derived start & level heights of front edge above brow line (exact rigid-body trigonometry)
GAP_LEVEL = (PIVOT_Z + BRIM_DZ) - BROW_Z  # 0.020 m (20.0 mm above brow line at tilt = 0 deg)
GAP_START = round(
    (PIVOT_Z + BRIM_ARM * math.sin(math.radians(TILT_START_DEG)) + BRIM_DZ * math.cos(math.radians(TILT_START_DEG)))
    - BROW_Z,
    4,
)  # 0.0459 m (45.9 mm above brow line at tilt = 13 deg)

DIAL_D = 0.028  # m (estimate: 28 mm outer diameter of standard EN 1078 kids' helmet rear thumb dial)
DIAL_R = 0.5 * DIAL_D  # 0.014 m
DIAL_DEPTH = 0.011  # m (estimate: 11 mm knob depth proud of rear cradle housing)
DIAL_SIZE_KIND = "estimate"

DIAL_DY = -0.112  # m (estimate: rear dial center 112 mm behind ear pivot -> 227 mm behind front edge)
DIAL_DZ = -0.031  # m (estimate: rear dial center 31 mm below ear pivot -> z = 91 mm level, 67 mm at 13 deg)
DIAL_PITCH_DEG = 8.5  # deg (estimate: occipital cradle face angle; at +2.6 deg fit tilt, axis is 5.9 deg from horizontal)

DIAL_CLICK_DEG = 20.0  # deg (estimate: 18-tooth moulded ratchet ring = 20 deg per click)

# The dial's whole travel, open stop to closed stop: 15 clicks. The band is drawn in linearly over it and, with the
# helmet level, meets the back of the head near the middle (7 clicks) and presses it with CLAMP_N by 9 clicks (60 %
# of the travel), so the fit comes from the band's press, 6 clicks before the stop.
DIAL_TRAVEL_DEG = 300.0  # deg (estimate: 15 clicks = 0.83 turn over the 53-56 cm size-S adjustment range; inside +/-150 deg joint range)
DIAL_MID_DEG = 0.5 * DIAL_TRAVEL_DEG  # 150.0 deg built-pose midpoint
DIAL_TRAVEL_CLICKS = round(DIAL_TRAVEL_DEG / DIAL_CLICK_DEG)  # 15

PRETOUCH_CLICKS = 7  # clicks (estimate: 2.33 mm radial clearance of 56 cm open ring around 55 cm head = 7 clicks to touch)
CLAMP_CLICKS = 2  # clicks (estimate: 2 clicks = 0.67 mm pad compression past first touch to reach CLAMP_N = 24 N)
TOTAL_CLAMP_CLICKS = PRETOUCH_CLICKS + CLAMP_CLICKS  # 9 clicks = 180 deg, 60 % of the travel

BAND_TRAVEL = 0.0050  # m (estimate: 30 mm circumference range 56->53 cm gives ~4.8 mm radial draw, rounded to 5.0 mm)
BAND_LEAD = BAND_TRAVEL / math.radians(DIAL_TRAVEL_DEG)  # m/rad (0.955 mm/rad = 0.333 mm per click)
BAND_GAP_OPEN = PRETOUCH_CLICKS * math.radians(DIAL_CLICK_DEG) * BAND_LEAD  # 0.00233 m: band-to-head gap, level, open

# The back of the head where the band's pad sits with the helmet level: a sphere centred on the line from the strap
# anchors to the pad, so the pad meets it square after PRETOUCH_CLICKS (the gap BAND_GAP_OPEN, 2.33 mm).
BAND_DY_LEVEL = -0.0930  # m (-y of band inner contact point in level helmet frame)
BAND_DZ_LEVEL = -0.0320  # m (-z of band inner contact point in level helmet frame)
_BAND_R_PIV = math.hypot(BAND_DY_LEVEL, BAND_DZ_LEVEL)  # 0.09835 m
_U0_Y = BAND_DY_LEVEL / _BAND_R_PIV  # -0.9456
_U0_Z = BAND_DZ_LEVEL / _BAND_R_PIV  # -0.3254
OCCIPUT_SPHERE_R = 0.0480  # m analytic sphere contained inside cranium
_OCC_ALPHA = _BAND_R_PIV - OCCIPUT_SPHERE_R - BAND_GAP_OPEN
OCCIPUT_CENTER_LOCAL = (0.0, _OCC_ALPHA * _U0_Y, PIVOT_Z + _OCC_ALPHA * _U0_Z)
# The nape, where the pad sits with the helmet tipped back to its start: a second sphere placed as the first is for the
# level helmet, so the band meets it after the same PRETOUCH_CLICKS. Tightened while tipped back, the band closes on
# the nape and clamps the helmet there, as a real band closes on whatever part of the head it lies on. Between the two
# the pad meets one sphere or the other at most 0.31 mm further out (under a click): the dial clamps at any tilt.
NAPE_SPHERE_R = 0.0480  # m
_NAPE_T = math.radians(TILT_START_DEG)
_UN_Y = _U0_Y * math.cos(_NAPE_T) - _U0_Z * math.sin(_NAPE_T)  # the pad's direction from the strap anchors, tipped back
_UN_Z = _U0_Y * math.sin(_NAPE_T) + _U0_Z * math.cos(_NAPE_T)
_NAPE_ALPHA = _BAND_R_PIV - NAPE_SPHERE_R - BAND_GAP_OPEN
NAPE_CENTER_LOCAL = (0.0, _NAPE_ALPHA * _UN_Y, PIVOT_Z + _NAPE_ALPHA * _UN_Z)
# Slide axis in level helmet frame points radially inward from band pad toward PIVOT (+0.9456 y, +0.3254 z)
BAND_AXIS_LEVEL = (-_U0_Y, -_U0_Z)  # (y_comp, z_comp) unit vector in level helmet frame
BAND_PAD_THICK = 0.0050  # m thickness of rear band contact pad

# --- forces, torques, stiffnesses, and bilateral reaction constants ---
TIP_BACK_NM = 0.04  # N m (estimate: 220 g helmet COM ~18 mm behind ear-strap pivot -> 0.220 * 9.81 * 0.018 = 0.039 N m)
TILT_FRICTION_NM = 0.01  # N m (estimate: bare crown/strap pivot drag, ~2 mm friction arm on 2.2 N weight; < TIP_BACK_NM)
BAND_MU = 0.60  # (estimate: dry retention-pad friction on expanded polystyrene head form)
BAND_ARM = 0.095  # m (estimate: 95 mm mean head semi-radius lever arm for retention friction hold about tilt axis)
DIAL_TURN_NM = 0.03  # N m (estimate: ~2 N tangential thumb push at 14 mm dial radius to click leaf-spring pawl forward)
DIAL_HOLD_NM = 0.24  # N m (estimate: ~17 N tangential reverse pull at 14 mm radius to back-drive pawl by hand)
DIAL_TILT_SHARE = 0.50  # (estimate: half of active dial tightening torque pitches unheld shell backward as band draws in)

# The band's padding (its slide spring). The contact sensor reads the band's press on the head at 65 to 72 % of this
# spring times its compression (run h4t2c at 30 N/mm); 55 N/mm puts CLAMP_N, as read, CLAMP_CLICKS after first touch.
K_BAND = 55000.0  # N/m (estimate: EPS liner and retention-pad contact stiffness placing 24 N at 2 clicks past touch)
D_BAND = 57.0  # N s/m, critical on the 15 g band body
CLAMP_N = 24.0  # N (estimate: band press giving 1.38 N m friction hold > 1.19 N m of a 10 N front-edge upward pull)

K_TILT = 60.0  # N m/rad capped elasto-plastic spring stiffness on tilt hinge (2.3 mm edge deflection at 10 N pull)
D_TILT = 0.06  # N m s/rad tilt damping (~0.5 s rock-back over 13 deg when open; settles < 0.3 s when clamped)
K_DIAL = 12.0  # N m/rad capped one-way ratchet spring stiffness on dial hinge (< 0.15 deg lag at 0.03 N m)
D_DIAL = 0.06  # N m s/rad dial damping

SECOND_INERTIA = (8.0e-5, 8.0e-5, 8.0e-5)  # kg m^2 well-conditioned inertia for 15 g dial and band rigid bodies

# --- scoring & safety rules (identical in task.py, task.json, README.md, and tests) ---
# The fit window (NHTSA: "one or two finger-widths above your eyebrow"; converted to 15 to 35 mm: one finger width
# 15 mm at the lower bound, two adult finger widths 35 mm at the upper bound; under 15 mm the edge sits over the eyes).
GAP_MIN = 0.015  # m (brow band to the front edge's underside: below it the edge is over the eyes -> 0)
GAP_MAX = 0.035  # m (two finger widths: above it the forehead is bare -> 0)

PULL_TEST_N = 10.0  # N: the bench test, a luggage scale hooked under the front edge's centre and pulled up
PULL_MAX_MOVE_M = 0.005  # m: a fitted helmet's front edge lifts less than this under the pull
# The pull's moment about the strap anchors, with the helmet's own rocking back: the one figure the clamp must hold.
PULL_MOMENT_NM = round(PULL_TEST_N * BRIM_ARM + TIP_BACK_NM, 3)  # 1.19 N m
# The upper tightness check (comfort bound): full credit also needs the dial at most COMFORT_MAX_CLICKS past the band's
# first touch on the head (PRETOUCH_CLICKS from the open stop), so at most COMFORT_MAX_DIAL_CLICKS from the open stop.
# The bench counts the clicks from the open stop on the trial video and again turning the dial back to it at reset;
# the simulator reads the dial's angle. Snug is CLAMP_CLICKS past touch (CLAMP_N holds the pull); more only presses.
COMFORT_MAX_CLICKS = 4  # clicks (estimate: snug CLAMP_CLICKS = 2 plus 2: about twice the CLAMP_N press, ring 8 mm under first touch)
COMFORT_MAX_DIAL_CLICKS = PRETOUCH_CLICKS + COMFORT_MAX_CLICKS  # 11 clicks = 220 deg from the open stop
HEADFORM_EN960_SIZE = 555  # the representative head form for the bound: EN 960 size 555 (555 mm round), inside 53-56 cm

MAX_HEAD_SHIFT_M = 0.010  # m (10.0 mm whole-trial zero rule: the head form moved from its tape marks)
# Whole-trial zero rule: an arm touching the head form's face (eyes, nose). A touch, not a force: the simulator lists
# an arm link's contact points on a body but reads their force as 0 (demo run h4t2b), so a force threshold could not act.
MIN_FACE_ARM_CLEAR_M = 0.025  # m (25 mm: the demo's and G1's least clearance of either arm's hulls from the face)
GRIPPER_CLEAR_M = 0.030  # m (30.0 mm end-state clearance of both grippers from helmet and head)
REST_LINEAR_MPS = 0.010  # m/s linear rest threshold
REST_ANGULAR_RPS = 0.10  # rad/s angular rest threshold
REST_HOLD_S = 1.0  # s (20 steps at 20 Hz) continuous end-state hold after both hands let go

# --- sRGB colours (high visual contrast at 320 x 176) ---
STYRO_WHITE = (0.94, 0.94, 0.95)
FACE_TINT = (0.87, 0.88, 0.90)
BROW_DARK = (0.09, 0.09, 0.11)
HELMET_BLUE = (0.11, 0.44, 0.86)
HELMET_STRIPE = (0.95, 0.76, 0.12)
LINER_CHARCOAL = (0.17, 0.18, 0.21)
DIAL_RED = (0.86, 0.18, 0.14)
DIAL_KNURL = (0.20, 0.21, 0.24)
DIAL_MARK = (0.96, 0.96, 0.96)
BAND_BLACK = (0.15, 0.16, 0.18)

# Indices in head_shapes() that belong to the face sensor zone (eyes, nose, cheeks below brow line)
FACE_SHAPE_INDICES: tuple[int, ...] = (4, 5, 6)

KEY_FIGURES = (
    {
        "figure": "Head form raw height (29 cm) and article number (3396700)",
        "constants": "HEAD_RAW_H",
        "value": f"{1000 * HEAD_RAW_H:.0f} mm",
        "kind": "maker",
        "source": (
            f'maker: Rayher product page ({HEAD_MAKER_URL}), "Styropor-Kopf, weiblich, 29 cm", Art.-Nr. 3396700; '
            f'US twin FloraCraft RS260/6 sheet ({HEAD_US_TWIN_URL}): "6.3 x 6.8 x 10\\" Female - White", '
            f'"Female head circumference is 21\\"." (53.3 cm)'
        ),
        "check": "Ruler on uncut head form before sawing off the display neck (1 min)",
    },
    {
        "figure": "Head form circumference round the forehead",
        "constants": "HEAD_CIRC",
        "value": f"{1000 * HEAD_CIRC:.0f} mm",
        "kind": "listing",
        "source": (
            f'listing: MosaikShop page for Rayher 3396700 ({HEAD_LISTING_URL}), '
            f'"Model number: 3396700 ... Heigh: 29 cm ... Head circumference: 55 cm"'
        ),
        "check": "Flexible tape measure round the forehead just above the brow line (1 min)",
    },
    {
        "figure": "Helmet model, size S head range, mass, and EN 1078 rear-dial spec",
        "constants": "HELMET_MASS",
        "value": f"530 to 560 mm; {1000 * HELMET_MASS:.0f} g",
        "kind": "listing",
        "source": (
            f'maker support page ({HELMET_SUPPORT_URL}): "KINDER FAHRRADHELM - KH 500", "REF: 8737455, 8737457, '
            f'8737456, 8737454"; Decathlon UK product listing ({HELMET_LISTING_URL}, search-indexed, 403 to script): '
            f'size S 53-56 cm, weight 220 g, EN 1078, rear adjustment dial, 6 vents'
        ),
        "check": "Kitchen scale and inside label check on bought KH 500 size S (1 min)",
    },
    {
        "figure": "Forehead fit rule and loose rock-back misfit",
        "constants": "GAP_MIN, GAP_MAX",
        "value": f"1 to 2 finger widths ({1000 * GAP_MIN:.0f} to {1000 * GAP_MAX:.0f} mm)",
        "kind": "standard",
        "source": (
            f'standard: NHTSA, "Easy Steps to Properly Fit a Bicycle Helmet" ({NHTSA_FIT_URL}; also {NHTSA_UCDAVIS_URL}): '
            f'"low on your forehead—one or two finger-widths above your eyebrow" and Step 6B "Does your helmet rock '
            f'back more than two fingers above the eyebrows?"; Decathlon fit guide ({HELMET_FIT_ADVICE_URL}, '
            f'search-indexed, 403 to script): "fits snugly, but not too tightly across your forehead ... stay in '
            f'place without sliding forward, backward, or sideways"; mm bounds 15-35 mm from 1-2 finger widths'
        ),
        "check": "Lay one and two fingers flat on the painted brow band and read with a ruler (1 min)",
    },
    {
        "figure": "Cut head-form crown-to-brow, crown-to-chin, brow height, width/length, mass, and friction",
        "constants": "CROWN_TO_BROW, CROWN_TO_CHIN, HEAD_H, BROW_Z, HEAD_RX, HEAD_RY, HEAD_MASS, HEAD_FRICTION",
        "value": (
            f"{1000 * CROWN_TO_BROW:.0f}, {1000 * CROWN_TO_CHIN:.0f}, {1000 * HEAD_H:.0f}, {1000 * BROW_Z:.0f} mm; "
            f"{2000 * HEAD_RX:.0f} x {2000 * HEAD_RY:.0f} mm; {1000 * HEAD_MASS:.0f} g; mu {HEAD_FRICTION:.2f}"
        ),
        "kind": "estimate",
        "source": (
            "estimate: adult-female head proportions scaled to the listed 550 mm circumference (158 x 188 mm ellipse; "
            "210 mm crown-to-chin after sawing 78 mm of the 290 mm neck 2 mm below the chin; brow halfway down at "
            "110 mm below crown -> z = 102 mm; ~3.6 L EPS at ~25 kg/m^3 -> 90 g; dry EPS-on-pad friction 0.55)"
        ),
        "check": "Ruler crown-to-brow and crown-to-chin before sawing neck, kitchen scale after cut (2 min)",
    },
    {
        "figure": "Helmet outer dimensions, front rim, and strap-anchor pivot geometry",
        "constants": "HELMET_L, HELMET_W, HELMET_H, BRIM_THICK, PIVOT_Z, BRIM_ARM",
        "value": (
            f"{1000 * HELMET_L:.0f} x {1000 * HELMET_W:.0f} x {1000 * HELMET_H:.0f} mm; rim {1000 * BRIM_THICK:.0f} mm; "
            f"pivot z {1000 * PIVOT_Z:.0f} mm, arm {1000 * BRIM_ARM:.0f} mm"
        ),
        "kind": "estimate",
        "source": (
            "estimate: 188 x 158 mm head ellipse plus ~11-16 mm EPS liner wall (220 x 180 x 120 mm, 12 mm front rim); "
            "ear-strap Y-junction 20 mm above brow plane (z = 122 mm) and front edge 115 mm ahead of ear axis"
        ),
        "check": "Caliper on shell L x W x H and front rim; ruler and side photo of ear-strap Y-junction (2 min)",
    },
    {
        "figure": "Open start tilt, front-edge gap, rock-back torque, and bare strap drag",
        "constants": "TILT_START_DEG, GAP_START, TIP_BACK_NM, TILT_FRICTION_NM",
        "value": f"{TILT_START_DEG:.0f} deg, {1000 * GAP_START:.1f} mm; {TIP_BACK_NM:.2f} and {TILT_FRICTION_NM:.2f} N m",
        "kind": "estimate",
        "source": (
            "estimate: front Y-strap slack with two-finger chin-strap clearance lets the open shell rock back 13 deg "
            "(raising the 115 mm front edge by 115 * sin 13 deg = 25.9 mm from 20 mm level to 45.9 mm over the brow "
            "while keeping the 12 mm rim top at z = 159 mm <= 160 mm); rear dial/cradle put the 220 g COM ~18 mm "
            "behind the ear pivot (0.220 * 9.81 * 0.018 = 0.039 -> 0.04 N m vs 0.01 N m bare strap drag)"
        ),
        "check": (
            "2 min on the foam head: buckle strap at two-finger slack with dial open, let helmet rock back, read tilt "
            "(phone protractor app) and brow-to-edge gap (ruler); tip to 25 mm, let go to check rock-back, and hold "
            "level with a luggage scale on a string at the front edge"
        ),
    },
    {
        "figure": "Rear dial size, placement, click pitch, tighten/pawl torques, and shell-twist share",
        "constants": "DIAL_D, DIAL_DEPTH, DIAL_DY, DIAL_DZ, DIAL_CLICK_DEG, DIAL_TURN_NM, DIAL_HOLD_NM, DIAL_TILT_SHARE",
        "value": (
            f"{1000 * DIAL_D:.0f} x {1000 * DIAL_DEPTH:.0f} mm at ({1000 * DIAL_DY:.0f}, {1000 * DIAL_DZ:.0f}) mm; "
            f"{DIAL_CLICK_DEG:.0f} deg/click; {DIAL_TURN_NM:.2f} / {DIAL_HOLD_NM:.2f} N m; share {DIAL_TILT_SHARE:.2f}"
        ),
        "kind": "estimate",
        "source": (
            "estimate: standard EN 1078 kids' occipital thumb wheel (28 x 11 mm, 18-tooth ring = 20 deg/click) 112 mm "
            "behind and 31 mm below the ear pivot; ~2 N tangential push at 14 mm radius (0.03 N m) to click forward, "
            "~17 N (0.24 N m) to back-drive the pawl by hand, half of active turn torque pitching an unheld shell back"
        ),
        "check": "Caliper on dial; count clicks per turn; wrap string round dial and pull with luggage scale (2 min)",
    },
    {
        "figure": "Dial travel, clicks to touch and clamp, band travel, pad stiffness, clamp press, and pull hold",
        "constants": "DIAL_TRAVEL_DEG, PRETOUCH_CLICKS, CLAMP_CLICKS, BAND_TRAVEL, K_BAND, CLAMP_N, BAND_MU, BAND_ARM",
        "value": (
            f"{DIAL_TRAVEL_CLICKS} clicks; touch {PRETOUCH_CLICKS}, {CLAMP_N:.0f} N at {TOTAL_CLAMP_CLICKS}; "
            f"{1000 * BAND_TRAVEL:.1f} mm; {K_BAND / 1000:.0f} N/mm; mu {BAND_MU:.2f} at {1000 * BAND_ARM:.0f} mm"
        ),
        "kind": "estimate",
        "source": (
            "estimate: 30 mm size-S perimeter draw (56->53 cm) gives ~4.8 mm radial draw (~5.0 mm over 15 clicks = "
            "0.333 mm/click); 55 cm head inside 56 cm open ring leaves 2.33 mm radial gap (7 clicks to touch), plus 2 "
            "clicks on 55 N/mm pad stiffness to reach 24 N, where friction (0.60 * 24 N * 95 mm + 0.01 = 1.38 N m) "
            "holds the 1.19 N m moment of a 10 N upward front-edge pull to 2.3 mm (< 5 mm)"
        ),
        "check": "Count clicks open-to-stop and to touch; hook luggage scale under front edge and pull to 10 N (2 min)",
    },
    {
        "figure": "Representative head form for the comfort bound (EN 960 size in the helmet's 53-56 cm range)",
        "constants": "HEADFORM_EN960_SIZE",
        "value": f"EN 960 size {HEADFORM_EN960_SIZE}",
        "kind": "maker",
        "source": (
            f'maker: Cadex full magnesium headform P/N 100_01_HFM ({HEADFORM_EN960_URL}): "Number sizes available '
            f'(EN 960): 495 – 515 – 525 – 535 – 555 – 575 – 585 – 605 – 625", "designed and manufactured to meet the '
            f'ISO/DIS 6220 or EN 960 standards"; size 555 (555 mm round) is the one inside the KH 500 S 53-56 cm range '
            f'nearest the 550 mm foam head form'
        ),
        "check": "Tape round the foam head form at the brow line: 550 mm, 5 mm under size 555 (1 min)",
    },
    {
        "figure": "Comfort bound: dial clicks past the band's first touch for full credit",
        "constants": "COMFORT_MAX_CLICKS, COMFORT_MAX_DIAL_CLICKS",
        "value": f"{COMFORT_MAX_CLICKS} clicks past touch ({COMFORT_MAX_DIAL_CLICKS} from the open stop)",
        "kind": "estimate",
        "source": (
            "estimate: calibrated as a comfortable fit on an EN 960 size-555 head form: snug is the 2 clicks past first "
            "touch whose 24 N press holds the 10 N pull, and comfortable is taken as at most 2 clicks more, about twice "
            "that press (48 N on the 55 N/mm pads) with the ring 8 mm (2 mm a click) under its first-touch round, 1.4 % "
            "of 555 mm; past it the band only presses harder, the pull already held"
        ),
        "check": (
            "On the bought KH 500 S on the head form, dial from the open stop: count clicks to first touch (a paper "
            "strip under the rear pad stops sliding) and to the first click where the 10 N pull lifts < 5 mm; that click "
            "must be at or under touch + 4 (2 min)"
        ),
    },
)


def rx(angle_rad: float) -> np.ndarray:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


def ry(angle_rad: float) -> np.ndarray:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def head_start_pos(table_top: float) -> tuple[float, float, float]:
    return (float(HEAD_XY[0]), float(HEAD_XY[1]), float(table_top + OVER))


def pivot_world(table_top: float) -> tuple[float, float, float]:
    return (float(HEAD_XY[0]), float(HEAD_XY[1] + PIVOT_Y), float(table_top + OVER + PIVOT_Z))


def helmet_rot_at_tilt(tilt_deg: float) -> np.ndarray:
    """World rotation matrix of helmet at ``tilt_deg`` (0 deg = level, +13 deg = tipped back off forehead)."""
    return rx(math.radians(tilt_deg))


def front_edge_world(table_top: float, tilt_deg: float) -> np.ndarray:
    """World (x, y, z) of the front shell underside edge center at ``tilt_deg``."""
    piv = np.array(pivot_world(table_top), dtype=float)
    return piv + helmet_rot_at_tilt(tilt_deg) @ np.array([0.0, BRIM_ARM, BRIM_DZ], dtype=float)


def front_top_world(table_top: float, tilt_deg: float, inset_y: float = 0.007) -> np.ndarray:
    """World (x, y, z) of the top press/pinch point on the front shell rim at ``tilt_deg``."""
    piv = np.array(pivot_world(table_top), dtype=float)
    return piv + helmet_rot_at_tilt(tilt_deg) @ np.array([0.0, BRIM_ARM - inset_y, BRIM_DZ + BRIM_THICK], dtype=float)


def dial_center_world(table_top: float, tilt_deg: float) -> np.ndarray:
    """World (x, y, z) of the rear fit-dial knob center at ``tilt_deg``."""
    piv = np.array(pivot_world(table_top), dtype=float)
    return piv + helmet_rot_at_tilt(tilt_deg) @ np.array([0.0, DIAL_DY, DIAL_DZ], dtype=float)


def dial_axis_world(tilt_deg: float) -> np.ndarray:
    """World unit axis of hinge('dial') at ``tilt_deg`` (+angle = clockwise tightening viewed from behind -y)."""
    return rx(math.radians(tilt_deg - DIAL_PITCH_DEG)) @ np.array([0.0, 1.0, 0.0], dtype=float)


def dial_rot_at_tilt(tilt_deg: float, dial_turn_deg: float = 0.0) -> np.ndarray:
    """World rotation matrix of ``dial`` at helmet ``tilt_deg`` and dial rotation ``dial_turn_deg``."""
    return rx(math.radians(tilt_deg - DIAL_PITCH_DEG)) @ ry(math.radians(dial_turn_deg))


def band_center_world(table_top: float, tilt_deg: float, slide_m: float = 0.0) -> np.ndarray:
    """World (x, y, z) of the rear band body origin at ``tilt_deg`` and prismatic travel ``slide_m``."""
    piv = np.array(pivot_world(table_top), dtype=float)
    uy, uz = BAND_AXIS_LEVEL
    # Band origin sits 0.5 * BAND_PAD_THICK behind its inner (+u) contact surface so inner surface is at (BAND_DY_LEVEL, BAND_DZ_LEVEL)
    local = np.array(
        [
            0.0,
            BAND_DY_LEVEL - (0.5 * BAND_PAD_THICK - slide_m) * uy,
            BAND_DZ_LEVEL - (0.5 * BAND_PAD_THICK - slide_m) * uz,
        ],
        dtype=float,
    )
    return piv + helmet_rot_at_tilt(tilt_deg) @ local


GRAVITY = 9.81  # m/s^2, the simulator's


def _solid_volume(shape) -> float:
    """Volume (m^3) of one solid kit primitive: the simulator spreads a body's mass over its solid shapes by volume."""
    d = shape.dims
    if shape.kind == "box":
        return d[0] * d[1] * d[2]
    if shape.kind == "sphere":
        return 4.0 / 3.0 * math.pi * d[0] ** 3
    if shape.kind == "cylinder":
        return math.pi * d[0] ** 2 * d[1]
    if shape.kind == "capsule":
        return math.pi * d[0] ** 2 * d[1] + 4.0 / 3.0 * math.pi * d[0] ** 3
    raise ValueError(f"no volume for a {shape.kind}")


@functools.lru_cache(maxsize=1)
def helmet_com_local() -> tuple[float, float, float]:
    """The shell's centre of mass (m) in its level frame from the strap anchors: its solid shapes at one density."""
    solid = [s for s in helmet_shapes() if s.solid]
    vols = np.array([_solid_volume(s) for s in solid])
    com = (vols[:, None] * np.array([s.pos for s in solid], dtype=float)).sum(0) / vols.sum()
    return (float(com[0]), float(com[1]), float(com[2]))


def weights_tip_back_nm(tilt_deg: float) -> float:
    """Tip-back torque (N m about +x) of the shell's, dial's and band's weights about the strap anchors at
    ``tilt_deg``: the dial and band hang behind the anchors (0.011 N m level, 0.025 N m at the start). TIP_BACK_NM
    is the whole rocking back, so the mechanism's push is TIP_BACK_NM less this."""
    piv = np.array(pivot_world(0.0))
    arms = ((SHELL_MASS, helmet_rot_at_tilt(tilt_deg) @ np.array(helmet_com_local())),
            (DIAL_MASS, dial_center_world(0.0, tilt_deg) - piv), (BAND_MASS, band_center_world(0.0, tilt_deg) - piv))
    return float(sum(-m * GRAVITY * float(a[1]) for m, a in arms))


def band_axis_world(tilt_deg: float) -> np.ndarray:
    """World unit slide axis of slide('band') at ``tilt_deg`` (pointing from rear band toward occiput)."""
    uy, uz = BAND_AXIS_LEVEL
    return helmet_rot_at_tilt(tilt_deg) @ np.array([0.0, uy, uz], dtype=float)


def gap_from_tilt_m(tilt_deg: float) -> float:
    """Height (m) of the helmet front-edge underside above the brow-band center as a function of tilt angle."""
    th = math.radians(tilt_deg)
    z_edge = PIVOT_Z + BRIM_ARM * math.sin(th) + BRIM_DZ * math.cos(th)
    return float(z_edge - BROW_Z)


def face_bounds_world(table_top: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Box (center, rot, size) round the head form's eyes and nose, for the arms' clearance checks: x +/-50 mm, y 60 to
    103 mm ahead of the head's centre, z from 62 to 4 mm under the brow line."""
    base = np.array(head_start_pos(table_top), dtype=float)
    center = base + np.array([0.0, 0.0815, BROW_Z - 0.033], dtype=float)
    size = np.array([0.100, 0.043, 0.058], dtype=float)
    return center, np.eye(3, dtype=float), size


def head_shapes() -> list:
    """Shapes of ``head``, origin on the bench under the head's centre (the cut just under the chin stands on the bench),
    the face towards +y. The face is placed from the brow line down, the skull from the crown down. Shapes 4, 5, 6 are
    the face zone (nose, eyes) the face rule reads."""
    q_axis_y = matrix_to_quat(rx(0.5 * math.pi))
    skull_r, skull_len, skull_y = 0.075, 0.038, 0.003  # the skull: a capsule along y, 150 x 188 mm, crown on top
    skull_z = HEAD_CROWN_Z - skull_r
    shapes = [
        # 0: neck stub under the jaw, standing on the bench (solid)
        cylinder(0.048, 0.060, pos=(0.0, -0.012, 0.030), color=STYRO_WHITE),
        # 1: lower face and jaw, from the chin to just under the brow line (solid)
        cylinder(0.066, BROW_Z - 0.010, pos=(0.0, 0.012, 0.006 + 0.5 * (BROW_Z - 0.010)), color=STYRO_WHITE),
        # 2: back of the head that the band presses (a sphere: smooth, facet-free contact; solid)
        sphere(OCCIPUT_SPHERE_R, pos=OCCIPUT_CENTER_LOCAL, color=STYRO_WHITE),
        # 3: forehead, its front at the brow line 88 mm ahead of the head's centre (solid)
        sphere(0.058, pos=(0.0, 0.032, BROW_Z + 0.014), color=STYRO_WHITE),
        # 4: nose, 24 to 56 mm under the brow line (solid; face zone)
        box((0.018, 0.022, 0.032), pos=(0.0, 0.087, BROW_Z - 0.040), color=FACE_TINT),
        # 5, 6: eyes, 9 to 27 mm under the brow line (solid; face zone)
        box((0.030, 0.014, 0.018), pos=(-0.026, 0.081, BROW_Z - 0.018), color=FACE_TINT),
        box((0.030, 0.014, 0.018), pos=(+0.026, 0.081, BROW_Z - 0.018), color=FACE_TINT),
        # 7: the skull's look, brow to crown (solid=False: the band presses only shape 2)
        capsule(skull_r, skull_len, pos=(0.0, skull_y, skull_z), quat=q_axis_y, color=STYRO_WHITE, solid=False),
    ]
    # The painted 8 mm dark brow band, on the skull's surface at the brow line, across the forehead and onto the temples
    # (-72 to +72 deg), so that the left wrist camera and the top camera read it at 320 x 176.
    ring = math.sqrt(skull_r**2 - (BROW_Z - skull_z) ** 2) + 0.0015  # the skull's half-width there, plus the paint
    for k in range(-6, 7):
        ang = k * math.radians(12.0)
        rot = np.array([[math.cos(ang), math.sin(ang), 0.0], [-math.sin(ang), math.cos(ang), 0.0], [0.0, 0.0, 1.0]])
        pos = (ring * math.sin(ang), skull_y + 0.5 * skull_len + ring * math.cos(ang), BROW_Z)
        shapes.append(box((0.019, 0.0035, BROW_BAND_W), pos=pos, quat=matrix_to_quat(rot), color=BROW_DARK, solid=False))
    # the nape that the band presses when the helmet is tightened tipped back (solid; see NAPE_CENTER_LOCAL)
    shapes.append(sphere(NAPE_SPHERE_R, pos=NAPE_CENTER_LOCAL, color=STYRO_WHITE))
    return shapes


def helmet_shapes() -> list:
    """Shapes of ``helmet`` in its level (tilt = 0) frame with origin at the ear-strap pivot (z = PIVOT_Z = 122 mm).
    All shell shapes stay at dz >= 0.000 m so when tipped back at +13 deg, the white forehead + 8 mm brow band (+y)
    and the protruding red rear dial (-y) remain clearly visible in top_cam (including 320 x 176) and left_cam."""
    shapes = [
        # 0: Front press/pinch visor tab (44 mm wide, 22 mm deep, 14 mm thick; underside exact at dz = BRIM_DZ = 0.0)
        box(
            (BRIM_TAB_W, 0.022, BRIM_THICK),
            pos=(0.0, BRIM_ARM - 0.011, BRIM_DZ + 0.5 * BRIM_THICK),
            color=HELMET_BLUE,
        ),
        # 1-5: the shell's colliders, five tiers inside the dome's profile (DOME_R, DOME_H; dome_look draws the shell)
        *shell_tiers(),
        # 5: High-contrast dark front bottom edge trim at exact dz = 0..4 mm (reads front-edge height in left_cam & 320x176)
        box((0.064, 0.005, 0.004), pos=(0.0, BRIM_ARM - 0.0015, BRIM_DZ + 0.002), color=LINER_CHARCOAL, solid=False),
        # 6, 7, 8: Open rear occipital retention cradle arms descending from shell to rear dial housing (visual)
        box((0.007, 0.036, 0.007), pos=(-0.024, -0.090, -0.018), quat=matrix_to_quat(rx(math.radians(-26.0))), color=LINER_CHARCOAL, solid=False),
        box((0.007, 0.036, 0.007), pos=(+0.024, -0.090, -0.018), quat=matrix_to_quat(rx(math.radians(-26.0))), color=LINER_CHARCOAL, solid=False),
        box((0.054, 0.006, 0.016), pos=(0.0, DIAL_DY + 0.008, DIAL_DZ), color=LINER_CHARCOAL, solid=False),
        # 9, 10: Buckled Y-chin straps anchored at the ears (visual; marks tilt pivot line)
        capsule(0.0025, 0.068, pos=(-0.076, +0.004, -0.034), color=BAND_BLACK, solid=False),
        capsule(0.0025, 0.068, pos=(+0.076, +0.004, -0.034), color=BAND_BLACK, solid=False),
    ]
    return shapes + dome_look()


# The shell's look (nothing here collides): a dome over the solid tiers so that the pictures read as a helmet, not a
# stack of discs. A whole sphere cannot stand for it: its lower half would cover the forehead and the brow band.
DOME_R = 0.094  # m: the dome's radius at the rim
DOME_H = 0.112  # m: its height over the rim, 22 mm over the head form's crown (shell and liner)
DOME_Y = 0.010  # m: its axis, ahead of the strap anchors (the solid tiers stand 8 to 12 mm ahead)
DOME_TIERS = 22  # thin tiers of about 5 mm, each as wide as the dome halfway up it: no step is more than a pixel or two
SHELL_TIERS = ((0.0, 0.024), (0.020, 0.048), (0.044, 0.072), (0.068, 0.092), (0.088, 0.106))  # m: the colliders' heights


def dome_radius(dz: float) -> float:
    """The dome's radius dz over the rim."""
    return DOME_R * math.sqrt(max(0.0, 1.0 - (dz / DOME_H) ** 2))


def shell_tiers() -> list:
    """The shell's colliders: solid tiers 1 mm inside the dome at their tops, so the drawn dome covers them."""
    return [cylinder(dome_radius(top) - 0.001, top - bottom, pos=(0.0, DOME_Y, 0.5 * (bottom + top)), color=HELMET_BLUE)
            for bottom, top in SHELL_TIERS]


def _on_dome(x: float, y: float, size, color):
    """A thin box lying on the dome's surface over plan point (x, y), its length along the surface towards +y."""
    rho2 = (x * x + (y - DOME_Y) ** 2) / DOME_R**2
    z = DOME_H * math.sqrt(max(0.0, 1.0 - rho2))
    n = np.array([x / DOME_R**2, (y - DOME_Y) / DOME_R**2, z / DOME_H**2])
    n /= np.linalg.norm(n)
    t = np.array([0.0, 1.0, 0.0]) - n[1] * n
    t /= np.linalg.norm(t)
    rot = np.column_stack([np.cross(t, n), t, n])
    pos = np.array([x, y, z]) + (0.5 * size[2] + 0.0005) * n
    return box(size, pos=tuple(float(v) for v in pos), quat=matrix_to_quat(rot), color=color, solid=False)


def dome_look() -> list:
    """The helmet's visual dome (DOME_TIERS tiers enveloping the solid ones), a yellow stripe over the crest and six
    dark vent slots, in the helmet's level frame."""
    step = DOME_H / DOME_TIERS
    out = []
    for k in range(DOME_TIERS):
        mid = (k + 0.5) * step
        out.append(cylinder(dome_radius(mid), step, pos=(0.0, DOME_Y, mid), color=HELMET_BLUE, solid=False))
    out += [_on_dome(0.0, DOME_Y + dy, (0.014, 0.025, 0.003), HELMET_STRIPE) for dy in (-0.048, -0.024, 0.0, 0.024, 0.048)]
    for x, dy, length in ((0.030, 0.030, 0.032), (0.030, -0.025, 0.032), (0.022, 0.065, 0.020)):
        out += [_on_dome(s * x, DOME_Y + dy, (0.010, length, 0.003), LINER_CHARCOAL) for s in (-1.0, 1.0)]
    return out


DIAL_FLATS = 24  # the ribbed rim as a 24-sided prism: flats every 15 deg, 27.8 mm across flats, 28 mm across corners
DIAL_FLAT_OFFSET_DEG = 7.5  # a flat faces the jaws as each 75-deg regrasp starts and ends (jaw at roll + 45 deg)


def dial_shapes() -> list:
    """Shapes of ``dial`` in the level frame (axis along y, DIAL_D = 28 mm across, DIAL_DEPTH = 11 mm deep). The solid
    rim is a 24-sided prism of 12 slabs (one pair of opposite flats each): parallel jaws close on two flats wherever
    the dial stands, at most 7.5 deg off one, so the dial does not snap round when pinched (an 8-sided rim turned up
    to 27 deg at a pinch in run h4t2b). The round cylinder is the dial's look."""
    q_axis_y = matrix_to_quat(rx(0.5 * math.pi))  # a kit cylinder's axis is z: turned onto y (the dial's axis)
    half = math.pi / DIAL_FLATS
    across = 2.0 * DIAL_R * math.cos(half)  # 27.76 mm between opposite flats
    width = 2.0 * DIAL_R * math.sin(half)  # 3.66 mm per flat
    shapes = [cylinder(DIAL_R, DIAL_DEPTH, pos=(0.0, 0.0, 0.0), quat=q_axis_y, color=DIAL_RED, solid=False)]
    for k in range(DIAL_FLATS // 2):
        ang = math.radians(DIAL_FLAT_OFFSET_DEG) + k * 2.0 * half
        shapes.append(
            box((across, DIAL_DEPTH - 0.001, width), quat=matrix_to_quat(ry(ang)), color=DIAL_KNURL if k % 2 else DIAL_RED,
                passes=("head", "band"))
        )
    # White pointer bar across rear (-y) face so every 90-deg turn is visually unmistakable in top_cam & right_cam
    shapes.append(
        box((0.004, 0.002, DIAL_D - 0.004), pos=(0.0, -0.5 * DIAL_DEPTH - 0.0008, 0.0), color=DIAL_MARK, solid=False)
    )
    return shapes


def band_shapes() -> list:
    """Shapes of ``band`` (rear retention pad sliding forward into head's occiput sphere; passes helmet and dial)."""
    # Tilt pad normal along BAND_AXIS_LEVEL so its +u face meets OCCIPUT_SPHERE orthogonally
    uy, uz = BAND_AXIS_LEVEL
    pitch_x = math.atan2(uz, uy)  # ~19 deg
    q_pad = matrix_to_quat(rx(pitch_x))
    return [
        box(
            (0.036, BAND_PAD_THICK, 0.018),
            pos=(0.0, 0.0, 0.0),
            quat=q_pad,
            color=BAND_BLACK,
            passes=("helmet", "dial"),
        ),
    ]


def summary() -> dict:
    return {
        "helmet_product": HELMET_PRODUCT,
        "head_product": HEAD_PRODUCT,
        "head_mm": {
            "circumference": round(1000 * HEAD_CIRC, 1),
            "crown_to_brow": round(1000 * CROWN_TO_BROW, 1),
            "crown_to_chin": round(1000 * CROWN_TO_CHIN, 1),
            "crown_z": round(1000 * HEAD_CROWN_Z, 1),
            "brow_z": round(1000 * BROW_Z, 1),
            "brow_band_w": round(1000 * BROW_BAND_W, 1),
        },
        "helmet_and_dial_mm": {
            "shell_lwh": [round(1000 * HELMET_L, 1), round(1000 * HELMET_W, 1), round(1000 * HELMET_H, 1)],
            "pivot_z": round(1000 * PIVOT_Z, 1),
            "brim_arm": round(1000 * BRIM_ARM, 1),
            "front_edge_start_z": round(1000 * (BROW_Z + GAP_START), 1),
            "front_edge_level_z": round(1000 * (BROW_Z + GAP_LEVEL), 1),
            "gap_start": round(1000 * GAP_START, 1),
            "gap_level": round(1000 * GAP_LEVEL, 1),
            "dial_d": round(1000 * DIAL_D, 1),
            "dial_depth": round(1000 * DIAL_DEPTH, 1),
            "dial_start_yz": [
                round(1000 * float(dial_center_world(0.0, TILT_START_DEG)[1]), 1),
                round(1000 * (float(dial_center_world(0.0, TILT_START_DEG)[2]) - OVER), 1),
            ],
            "dial_level_yz": [
                round(1000 * float(dial_center_world(0.0, 0.0)[1]), 1),
                round(1000 * (float(dial_center_world(0.0, 0.0)[2]) - OVER), 1),
            ],
            "grasp_sep_level_mm": round(1000 * (BRIM_ARM - DIAL_DY), 1),
            "band_travel": round(1000 * BAND_TRAVEL, 2),
            "band_gap_open": round(1000 * BAND_GAP_OPEN, 2),
        },
        "forces_and_torques": {
            "helmet_mass_g": round(1000 * HELMET_MASS, 1),
            "tip_back_nm": TIP_BACK_NM,
            "tilt_friction_nm": TILT_FRICTION_NM,
            "dial_turn_nm": DIAL_TURN_NM,
            "dial_hold_nm": DIAL_HOLD_NM,
            "dial_tilt_share": DIAL_TILT_SHARE,
            "dial_travel_clicks": DIAL_TRAVEL_CLICKS,
            "pretouch_clicks": PRETOUCH_CLICKS,
            "clamp_clicks": CLAMP_CLICKS,
            "band_lead_mm_per_click": round(1000 * BAND_LEAD * math.radians(DIAL_CLICK_DEG), 4),
            "k_band_n_per_mm": K_BAND / 1000,
            "clamp_n": CLAMP_N,
            "tilt_hold_at_clamp_nm": round(TILT_FRICTION_NM + BAND_MU * CLAMP_N * BAND_ARM, 3),
            "pull_moment_nm": PULL_MOMENT_NM,
        },
        "scoring": {
            "gap_min_mm": round(1000 * GAP_MIN, 1),
            "gap_max_mm": round(1000 * GAP_MAX, 1),
            "max_head_shift_mm": round(1000 * MAX_HEAD_SHIFT_M, 1),
            "face_rule": "an arm touching the face (eyes, nose) zeroes the trial",
            "gripper_clear_mm": round(1000 * GRIPPER_CLEAR_M, 1),
            "rest_hold_s": REST_HOLD_S,
            "comfort_max_clicks_past_touch": COMFORT_MAX_CLICKS,
            "comfort_max_dial_clicks": COMFORT_MAX_DIAL_CLICKS,
        },
        "key_figures": list(KEY_FIGURES),
    }


if __name__ == "__main__":
    print(json.dumps(summary(), indent=2))
