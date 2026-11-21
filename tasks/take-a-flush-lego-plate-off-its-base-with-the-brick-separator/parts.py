"""Single source of truth for the LEGO plate & brick-separator task (``mech/tasks/lego_separator``).

Every product size, mass, angle, tolerance, hold force/moment, spring constant and scoring threshold lives here as one
named constant with its source in its comment: [published] (a cited reference), [listing], [geometry] (from cited
figures), [estimate: how it was derived], [rule] (the task's own threshold), [solver] or [solver stand-in].
``KEY_FIGURES`` lists the figures the start state and the scored events rest on, each with its source and how it is
checked in the first minutes of a real trial (README).

Frames:
- World (station): ``x`` forward away from the arms, ``y`` left, ``z`` up; arm bases at ``(0, +-0.305, 0.75)``.
- ``base`` & ``2x4`` plates (``target``, ``white_f``, ``white_b``): origin at the geometric centre of the solid ``3.2 mm``
  plate slab (excluding studs); ``x`` across the short side (width), ``y`` along the long side (length), ``z`` up.
- ``separator`` (frame ``S``): origin at the centre of the 2-stud front socket's underside face; ``+x_s`` from the
  socket along the handle towards the tail, ``y_s`` across the ``16.0 mm`` width, ``+z_s`` perpendicular out of the top
  of the socket (so the socket underside lies at ``z_s = 0``).
"""

from __future__ import annotations

import json
import math
import numpy as np

import mechkit as mk
from mechkit import axis_angle, body, box, cylinder, hinge, matrix_to_quat, slide, weld

MECHKIT_HELPER = True  # declarations inside build_scene() belong to the importing task.py module
ALL_BODY_NAMES: tuple[str, ...] = (
    "bench_marks",
    "base",
    "white_f",
    "white_b",
    "target",
    "separator",
    "dish",
    "peel_c",
    "c1",
    "c2",
    "sock",
)

GRAVITY = 9.81  # m/s^2
OVER = 0.0010  # m: loose bodies start 1 mm above their resting support so frame 0 draws every loose part cleanly
CONTACT_THIN = 0.0005  # m: crisp narrowphase contact offset for 3.2 mm plates and 1.7 mm studs

# --- 1. LEGO module geometry ----------------------------------------------------------------------------------------
# Published: the LDraw file format 1.0.2 (ldraw.org/article/218.html: "1 brick width/depth = 20 LDU", "1 plate height
# = 8 LDU", "1 stud diameter = 12 LDU", "1 stud height = 4 LDU", "1 LDU = 0.4 mm") and C. Bartneck's technical drawing
# "LEGO 2x4 Plate Dimensions and Measurements" (design 3020, 22 April 2019, linked from
# bartneck.de/2019/04/21/lego-brick-dimensions-and-measurements): pitch 8, stud 4.8 across and 1.8 high, plate 3.2,
# 15.8 x 31.8 mm; the page: "There is a 0.2mm gap between bricks next to each other".
PITCH = 0.0080  # m (8.0 mm): stud centre-to-centre pitch [published: LDraw 20 LDU; Bartneck's drawing 8]
STUD_D = 0.0048  # m (4.8 mm): stud outside diameter [published: LDraw 12 LDU; Bartneck's drawing 4.8]
STUD_R = 0.5 * STUD_D
STUD_H = 0.0017  # m (1.7 mm): stud height over the plate's top [estimate: between LDraw's 1.6 and the drawing's 1.8]
PLATE_T = 0.0032  # m (3.2 mm): plate slab thickness [published: LDraw 8 LDU; Bartneck's drawing 3.2]
LEGO_CLEAR = 0.0002  # m (0.2 mm): gap at a part's outer boundary [published: Bartneck; his 15.8 = 2 x 8 - 0.2]

# 2x4 plate (design 3020: the target plate and the two flanking white plates)
PLATE_STUDS_X = 2
PLATE_STUDS_Y = 4
PLATE_W = PLATE_STUDS_X * PITCH - LEGO_CLEAR  # 0.0158 m (15.8 mm) [published: Bartneck's drawing]
PLATE_L = PLATE_STUDS_Y * PITCH - LEGO_CLEAR  # 0.0318 m (31.8 mm) [published: Bartneck's drawing]
# Listing (Dashbrick, part 3020, dashbrick.com.au/products/lego-plate-2-x-4-dark-orange-3020-6097511): "Dimensions
# 1.6 x 3.2 x 0.51 cm, Weight 1.2g"; BrickOwl's catalog gives 1.159 g. Brickset's 10696-1 inventory
# (brickset.com/inventories/10696-1) lists element 302026, design 3020, "PLATE 2X4", Black, 2, and element 302001,
# design 3020, "PLATE 2X4", White, 2.
PLATE_MASS = 0.0012  # kg (1.2 g): LEGO Plate 2 x 4 design 3020 mass [listing]
# Solver stand-in: the target plate is simulated at 10 g. At its own 1.2 g the weld to the hinge link opened 0.9 mm
# under the press (run l10t14) and the separator came down on the base's studs; the extra 0.086 N is 3 % of
# PEEL_HOLD_NM's 2.5 N at the row. No force is added anywhere.
TARGET_MASS_SIM = 0.010  # kg [solver stand-in]

# Lever arm from the target plate's far (+y) edge hinge to the near (-y) stud row's centre: 3 * 8.0 + 3.9 = 27.9 mm
ROW_LEVER = (PLATE_STUDS_Y - 1) * PITCH + 0.5 * (PLATE_L - (PLATE_STUDS_Y - 1) * PITCH)  # 0.0279 m (27.9 mm) [geometry]

# --- 2. Green base plate from LEGO Classic 10696: LEGO Plate 8 x 16, design 92438, Bright Green ------------------------
# Brickset's 10696-1 inventory (brickset.com/inventories/10696-1) lists element 4610353, design 92438, "PLATE 8X16",
# Bright Green, 1. Its size is 8 x 16 studs at the 8.0 mm pitch less the 0.2 mm gap (as the drawing's 2 x 4: 2 x 8 - 0.2
# = 15.8 mm), and the part's own weight is 17.0 g (iloveblokjes.nl/products/groen-plaat-8-x-16, green 92438: "Het
# gewicht van dit blokje is 17.0 gram"; BrickOwl stores list 17 g).
BASE_STUDS_X = 8
BASE_STUDS_Y = 16
BASE_W = BASE_STUDS_X * PITCH - LEGO_CLEAR  # 0.0638 m (63.8 mm, 8 studs) [geometry]
BASE_L = BASE_STUDS_Y * PITCH - LEGO_CLEAR  # 0.1278 m (127.8 mm, 16 studs) [geometry]
BASE_T = PLATE_T  # 0.0032 m (3.2 mm) [published, as PLATE_T]
BASE_MASS = 0.017  # kg (17.0 g) [listing]
BASE_POCKET_RECESS = 0.0010  # m: solid recess directly under target's slot so stop_load("peel") reads 100 % of press force

# --- 3. LEGO Brick and Axle Separator, orange, design 96874 (also 31510), element 6240515 -----------------------------
# Listings: ToyPro (toypro.com/de/product/2001/brick-separator/orange) "Brick Separator", "Design ID: 96874",
# "Element IDs: 4654448, 6240515", orange; mamevsechno.cz/96874-Orange-Brick-and-Axle-Separator-d4117.htm
# "Rozměry: 7,6 x 1,6 x 1,92 cm" and "Hmotnost: 3,93 g" (dimensions 7.6 x 1.6 x 1.92 cm, weight 3.93 g). Brickset's
# 10696-1 inventory lists element 6240515, design 31510, "ELEMENT SEPARATOR", Bright Orange, 1.
SEP_L = 0.0760  # m (76.0 mm): outside length when resting flat [listing]
SEP_W = 0.0160  # m (16.0 mm): outside width (2 studs wide) [listing]
SEP_H = 0.0192  # m (19.2 mm): outside height when resting flat on handle underside [listing]
SEP_MASS = 0.00393  # kg (3.93 g): real separator mass [listing]
# Solver stand-in: the separator is simulated at 30 g. A 3.93 g body held by its handle in the YAM pinch turned out of
# it in one control step at its first touch of the plate (runs l10t9 to l10t12, every fix tried); at 30 g it holds
# (run l10t6 of the earlier build). No force is added anywhere; its extra 0.26 N weight is under 5 % of PRESS_N.
SEP_MASS_SIM = 0.030  # kg [solver stand-in]

# Side profile in frame S (origin = socket underside centre; +x_s towards handle, +z_s up). The outside size is
# listed; the side profile is estimated from a seller's side photo against that listed length (each line says how).
SOCKET_LEN = 0.0080  # m (8.0 mm): 1-stud front-to-back socket length (-4.0 to +4.0 mm along x_s) [estimate: the
# socket's two anti-studs lie one LEGO stud pitch apart]
# The socket and the lower block are drawn at full size; their colliders stop short of the white plates, so that a
# separator seated within SOCKET_TOL_MM bears on the target plate only (the white plates sit 0.1 mm from its sides and
# ends; with the 16 mm collider a 0.6 mm offset put the press on a white plate's edge: run l10t8)
SOCKET_COLLIDE_W = 0.0140  # m: the socket's collider across the plate (0.9 mm inside each side of the target plate)
ROCKER_COLLIDE_FROM = 0.0050  # m along x_s: the lower block's collider starts 1.1 mm behind the plates' ends
SOCKET_H = 0.0082  # m (8.2 mm): front socket block height above z_s = 0 (gives the listed 19.2 mm resting height) [estimate]
SOCKET_DEPTH = 0.0033  # m (3.3 mm): internal anti-stud recess depth [estimate: about twice the 1.7 mm stud height,
# so the socket's face meets the plate's top face when it has clipped over the studs]
EDGE_X = 0.0170  # m (17.0 mm): the lower edge behind the socket centre, 1.4 mm inside the outer free studs' rims so it
# rocks on their tops [estimate; at 18.4 mm, on the rims, it slid off them and the plate stopped at 7.7 deg: run l10t21]
EDGE_DZ = 0.0005  # m (0.5 mm): lower rocker face below socket face (1.0 mm clear of 1.7 mm base studs when flat;
# engages at ~3 deg rock) [estimate: the rocker face stands half a millimetre below the socket face in the side photo]
HANDLE_DEG = 35.0  # deg: upward angle of handle relative to socket face when seated [estimate: at a 36 deg rock the
# handle's tail stays 3.9 mm over the bench; with 25 deg it met the bench at 30 deg (runs l10t19, l10t20)]
HANDLE_RAD = math.radians(HANDLE_DEG)
HANDLE_BEAM_L = 0.0590  # m: sloped handle underside length from (EDGE_X, -EDGE_DZ) to tail (75.9 x 16.0 x 19.2 mm at rest) [estimate]
HANDLE_BEAM_T = 0.0080  # m: flanged handle height perpendicular to slope [estimate: half the socket block's height]
SEP_CENTER_IN_S = (0.0340, 0.0, 0.0065)  # m in S: CoM over the middle of the bench-resting handle (zero nose-overhang torque)
AXLE_X = 0.0120  # m: decorative top cross-axle post centre along x_s [estimate: placed from the seller's photo; drawn only]
AXLE_TOP_Z = 0.0108  # m: decorative top cross-axle post top in S (drawn only) [estimate]

# --- 4. White porcelain dish: a product class (a soup plate), not one named item -------------------------------------
# Every figure is an estimate: a home soup plate, 12 cm across and 2.5 cm deep, of porcelain.
DISH_D = 0.1200  # m (120.0 mm): outside rim diameter [estimate: a 12 cm soup plate]
DISH_R = 0.5 * DISH_D
DISH_H = 0.0250  # m (25.0 mm): outside rim height [estimate: a soup plate 2.5 cm deep]
DISH_WALL = 0.0050  # m (5.0 mm): rim/wall thickness [estimate: a china soup plate's wall]
DISH_FLOOR_T = 0.0050  # m (5.0 mm): floor thickness [estimate: a china soup plate's foot]
DISH_FLOOR_R = 0.0420  # m (42.0 mm): flat inner floor radius before sloped wall [estimate: the 12 cm plate's floor]
DISH_MASS = 0.150  # kg (150 g) [estimate: a porcelain soup plate's mass]

# --- 5. Bench layout in station world frame (m from arm-base midpoint; x forward, y left) ---------------------------
BASE_XY = (0.260, -0.036)  # green base centre; long side (127.8 mm) along y -> y in [-0.0999, +0.0279]; the near end
# stands where the separator's lower edge rocks, so the base's near edge stays at y = -0.0999 as before
SEP_START_XY = (0.190, -0.065)  # lower edge of the separator on the bench in front of the base, out of the right arm's shadow
DISH_XY = (0.240, +0.170)  # dish centre left of base

# Target plate & white plates on the 8x16 stud grid of base:
# Columns j = 0..15 run along +y from the right (-y) end of base: j = 0, 1 are FREE base studs; target occupies j = 2..5.
TARGET_FIRST_COL = 2  # two free stud columns (j = 0, 1) to the right (-y) of target's near row
TARGET_CENTER_Y_IN_BASE = -0.5 * BASE_STUDS_Y * PITCH + (TARGET_FIRST_COL + 0.5 * PLATE_STUDS_Y) * PITCH  # -0.0480 m
NEAR_ROW_Y_IN_BASE = -0.5 * BASE_STUDS_Y * PITCH + (TARGET_FIRST_COL + 0.5) * PITCH  # -0.0600 m
FAR_EDGE_Y_IN_BASE = TARGET_CENTER_Y_IN_BASE + 0.5 * PLATE_L  # -0.0321 m
WHITE_FRONT_X_IN_BASE = -2.0 * PITCH  # -0.0160 m (front long side of target, towards arms)
WHITE_BACK_X_IN_BASE = +2.0 * PITCH  # +0.0160 m (back long side of target, away from arms)

# --- 6. Mechanism forces, moments, tolerances & solver parameters ----------------------------------------------------
# LEGO publishes no clutch force (D. Konstanski, The Secret Life of LEGO Bricks, 2022, p. 22, as quoted at
# brickarchitect.com/cite/C2: "While the ideal strength's value must remain secret, it is a defined number measured in
# newtons"), so PRESS_N, PEEL_HOLD_NM and SEP_HOLD_N are estimates, each derived in one clause.
PRESS_N = 6.0  # N: downward press on the near row that seats the separator [estimate: twice SEP_HOLD_N, a fingertip's
# light press]
SOCKET_TOL_MM = 0.8  # mm: max horizontal offset of socket centre from near row centre to seat [estimate: the socket's
# rim sits 0.9 mm inside the plate's long sides; past about 1 mm it bears on a white neighbour's edge (run l10t8)]
SOCKET_TOL_M = 0.001 * SOCKET_TOL_MM
SOCKET_FACE_MM = 0.4  # mm: max vertical gap between socket face and target top face to seat [estimate: a quarter of
# the stud height; further off, the socket's anti-studs cannot clip over the studs]
SOCKET_FACE_M = 0.001 * SOCKET_FACE_MM
SOCKET_TOL_DEG = 4.0  # deg: max yaw and tilt error of socket relative to target plate to seat [estimate: at 4 deg
# across the 15.8 mm plate the socket's corner stands 0.55 mm up, about the face gap]
SEAT_STEPS = 3  # consecutive 20 Hz control steps (0.15 s) required to engage w_seat: a momentary touch does not seat [rule]

PEEL_HOLD_NM = 0.070  # N m: resisting moment of target plate peeling about its far edge (~2.51 N at 27.9 mm)
# [estimate: 2.5 N at the near row (x ROW_LEVER 27.9 mm), a thumb's lift at the plate's end]
PEEL_RELEASE_DEG = 12.0  # deg: peel angle at which target plate detaches from base [estimate: the near end has risen
# 5.8 mm (sin 12 deg x 27.9 mm), 3.4 stud heights, when the far row lets go; the rock reaches it at 33 deg (run l10t22)]
PEEL_RELEASE_RAD = math.radians(PEEL_RELEASE_DEG)
PEEL_LIMIT_DEG = 30.0  # deg: upper stop of peel hinge before detachment

SEP_HOLD_N = 4.0  # N: axial retention force of seated separator socket on target's two end studs [estimate: 2 N a
# stud for the socket's two anti-studs]

# Stiffnesses & dampings of the pure serial joint chains
PEEL_K = 25.0  # N m/rad: stiff capped peel spring (lag = 0.070 / 25 = 0.16 deg)
PEEL_LAG_RAD = PEEL_HOLD_NM / PEEL_K
PEEL_STOP_K = 15.0  # N m/rad: sprung lower stop on peel used as press-force gauge
PEEL_DAMP = 0.15  # N m s/rad

PULL_K = 25000.0  # N/m: stiff capped stud-pull spring (lag = 4.0 / 25000 = 0.16 mm << 1.7 mm STUD_H)
PULL_LAG_M = SEP_HOLD_N / PULL_K
PULL_DAMP = 40.0  # N s/m
PULL_OVER = 0.0005  # m: the slide runs 0.5 mm past STUD_H so that the cut at STUD_H is reached, never held at a hard limit

RACK_X_LIM = 0.0080  # m: the socket slides along the plate as the tool turns about its lower edge and the plate about
# its far edge: 5.3 mm at a 30 deg rock (12.5 deg peel), 6.9 mm at 36 deg (geometry); at +-4.5 mm the slide stopped and
# the peel stalled at 11.2 deg (run l10t19)
RACK_X_K = 15.0  # N/m: light centering spring on rack_x
RACK_X_DAMP = 1.2  # N s/m

RACK_LIM_DEG = 60.0  # deg: tool and plate turn 42.5 deg against each other at a 30 deg rock, 50 deg at 36 deg; the light
# spring (target 0) gives k x 2 sin(e/2), at most 4.5 % under k x e at 60 deg
RACK_K = 0.005  # N m/rad: light; 0.003 N m at 35 deg (4 % of PEEL_HOLD_NM); brings the freed plate back in line (2.8 deg droop)
RACK_DAMP = 0.002  # N m s/rad (the freed plate turns back in line over about 0.4 s, not swinging)

# Every body has its true mass except the two declared solver stand-ins (TARGET_MASS_SIM beside PLATE_MASS, SEP_MASS_SIM
# beside SEP_MASS); no load is added to carry part of a weight. The hidden links of the joint chains weigh 1 g each
# (CARRIER_MASS), the peel link 10 g taken out of the base's 17 g (PEEL_LINK_MASS). The gram-scale bodies turn with a
# larger inertia than their own (solver stand-in).
CARRIER_MASS = 0.001  # kg (1 g per hidden link of the socket's chain c1/c2/sock, on the plate's near row) [solver stand-in]
PEEL_LINK_MASS = 0.010  # kg: the peel hinge's link peel_c, its mass on the hinge axis (no moment on the plate), taken
# out of the base's BASE_MASS so that base and link weigh the plate's 17 g together [solver stand-in]
CARRIER_INERTIA = (2.0e-5, 2.0e-5, 2.0e-5)  # kg m^2
TARGET_INERTIA = (2.5e-5, 2.5e-5, 2.5e-5)  # kg m^2 (the 1.15 g slab's own: 1.0e-7, 2.6e-8, 1.2e-7)
SEP_INERTIA = (3.5e-5, 3.5e-5, 3.5e-5)  # kg m^2 (the 3.93 g separator's own: about 2e-6 across, 2e-7 along)
CARRIER_NAMES = ("peel_c", "c1", "c2", "sock")

# Surface frictions (static, dynamic) [estimate: usual ranges for these material pairs]
BASE_FRICTION = (0.55, 0.50)  # ABS baseplate on oak bench
LEGO_FRICTION = (0.45, 0.40)  # ABS brick/plate surfaces
SEP_HANDLE_FRICTION = (1.50, 1.35)  # ribbed ABS separator handle in rubber gripper pads
DISH_FRICTION = (0.40, 0.35)  # glazed porcelain

# --- 7. Scoring thresholds (Section 5 — identical across task.py, task.json, and README.md) -------------------------
MAX_BASE_LIFT_M = 0.0030  # m (3.0 mm): whole-trial zero rule if any base corner lifts more than 3 mm
MAX_BASE_SLIDE_M = 0.0150  # m (15.0 mm): whole-trial zero rule if any base corner slides more than 15 mm
MAX_DISH_MOVE_M = 0.0200  # m (20.0 mm): whole-trial zero rule if dish shifts more than 20 mm
GRIPPER_CLEAR_M = 0.0300  # m (30.0 mm): end-state clearance of both grippers from target, separator, and dish
SEP_OUTSIDE_MARGIN_M = 0.0150  # m: separator centre outside dish rim by at least 15 mm
REST_LIN_MPS = 0.010  # m/s (10.0 mm/s): linear speed limit for at-rest check
REST_ANG_RPS = 0.10  # rad/s: angular speed limit for at-rest check
REST_HOLD_S = 1.0  # s (20 steps at 20 Hz): continuous rest hold for 1.0

# --- 8. Colours (sRGB) ----------------------------------------------------------------------------------------------
LEGO_GREEN = (0.05, 0.50, 0.20)
LEGO_BLACK = (0.05, 0.05, 0.06)  # 10696 lists two Black 2 x 4 plates beside two White ones
LEGO_WHITE = (0.94, 0.95, 0.96)
LEGO_ORANGE = (0.96, 0.40, 0.04)
LEGO_ORANGE_DARK = (0.76, 0.28, 0.02)
PORCELAIN_WHITE = (0.88, 0.90, 0.93)
PORCELAIN_RIM = (0.76, 0.80, 0.86)
TAPE_MARK = (0.88, 0.76, 0.14)

KEY_FIGURES = (
    {
        "figure": "Stud pitch, stud diameter, stud height, plate thickness",
        "constants": "PITCH, STUD_D, STUD_H, PLATE_T",
        "now": f"{1000 * PITCH:.1f}, {1000 * STUD_D:.1f}, {1000 * STUD_H:.1f}, {1000 * PLATE_T:.1f} mm",
        "source": "published: LDraw file format 1.0.2 (20, 12, 4, 8 LDU at 0.4 mm) and Bartneck's 3020 drawing "
        "(8, 4.8, 1.8, 3.2 mm); stud height: estimate between their 1.6 and 1.8 mm",
        "check": "caliper across two studs and over a plate",
    },
    {
        "figure": "2x4 plate outer width x length and mass (design 3020)",
        "constants": "PLATE_W, PLATE_L, PLATE_MASS",
        "now": f"{1000 * PLATE_W:.1f} x {1000 * PLATE_L:.1f} mm, {1000 * PLATE_MASS:.2f} g",
        "source": "published: Bartneck's 3020 drawing for the size; listing for the mass (Dashbrick: \"Weight 1.2g\")",
        "check": "caliper, 0.01 g scale",
    },
    {
        "figure": "Separator outside length x width x height and mass (design 96874 / 31510, element 6240515)",
        "constants": "SEP_L, SEP_W, SEP_H, SEP_MASS",
        "now": f"{1000 * SEP_L:.1f} x {1000 * SEP_W:.1f} x {1000 * SEP_H:.1f} mm, {1000 * SEP_MASS:.2f} g",
        "source": "listing (mamevsechno.cz: \"7,6 x 1,6 x 1,92 cm\", \"3,93 g\")",
        "check": "caliper, 0.01 g scale",
    },
    {
        "figure": "Separator side profile: socket depth, lower edge distance & drop, handle angle",
        "constants": "SOCKET_DEPTH, EDGE_X, EDGE_DZ, HANDLE_DEG",
        "now": f"{1000 * SOCKET_DEPTH:.1f} mm, {1000 * EDGE_X:.1f} mm, {1000 * EDGE_DZ:.1f} mm, {HANDLE_DEG:.1f} deg",
        "source": "estimate: read off a seller's side photo against the listed 76.0 mm length",
        "check": "side photo on squared paper beside the caliper",
    },
    {
        "figure": "Green base plate 8 x 16 length x width and mass (92438, 10696 box)",
        "constants": "BASE_L, BASE_W, BASE_MASS",
        "now": f"{1000 * BASE_L:.1f} x {1000 * BASE_W:.1f} mm, {1000 * BASE_MASS:.1f} g",
        "source": "geometry: 8 x 16 pitches less the 0.2 mm gap; listing for the mass (iloveblokjes: \"17.0 gram\")",
        "check": "caliper, scale",
    },
    {
        "figure": "Press force to seat separator on 2-stud end row",
        "constants": "PRESS_N",
        "now": f"{PRESS_N:.1f} N",
        "source": "estimate: twice the socket's hold (SEP_HOLD_N), a fingertip's light press",
        "check": "kitchen scale under the green plate, 3 presses",
    },
    {
        "figure": "Target plate peel hold moment and release angle",
        "constants": "PEEL_HOLD_NM, PEEL_RELEASE_DEG",
        "now": f"{PEEL_HOLD_NM:.3f} N m, {PEEL_RELEASE_DEG:.1f} deg",
        "source": "estimate: about the lift a thumb gives the plate's end (2.5 N at 27.9 mm); the far row lets go at 5.8 mm",
        "check": "kitchen scale under the base + side video",
    },
    {
        "figure": "Separator axial retention force on the plate's end studs",
        "constants": "SEP_HOLD_N",
        "now": f"{SEP_HOLD_N:.1f} N",
        "source": "estimate: 2 N a stud for the socket's two anti-studs",
        "check": "luggage scale on a binder clip, 3 pulls",
    },
    {
        "figure": "Seating horizontal and angular tolerances",
        "constants": "SOCKET_TOL_MM, SOCKET_TOL_DEG",
        "now": f"{SOCKET_TOL_MM:.1f} mm, {SOCKET_TOL_DEG:.1f} deg",
        "source": "estimate: the socket's rim sits 0.9 mm inside the plate's long sides; 4 deg lifts its corner 0.55 mm",
        "check": "press 1 mm off centre: does it hold?",
    },
    {
        "figure": "Porcelain dish diameter, height and mass",
        "constants": "DISH_D, DISH_H, DISH_MASS",
        "now": f"{1000 * DISH_D:.0f} mm, {1000 * DISH_H:.0f} mm, {1000 * DISH_MASS:.0f} g",
        "source": "estimate: a home soup plate, 12 cm across and 2.5 cm deep; 150 g of porcelain",
        "check": "caliper, scale, maker and model noted",
    },
)


# --- 9. Pose & Coordinate Helpers -----------------------------------------------------------------------------------


def base_built_pos(table_top: float) -> tuple[float, float, float]:
    return (BASE_XY[0], BASE_XY[1], table_top + OVER + 0.5 * BASE_T)


def target_built_pos(table_top: float) -> tuple[float, float, float]:
    return (BASE_XY[0], BASE_XY[1] + TARGET_CENTER_Y_IN_BASE, table_top + OVER + BASE_T + 0.5 * PLATE_T)


def white_built_pos(table_top: float, front: bool) -> tuple[float, float, float]:
    dx = WHITE_FRONT_X_IN_BASE if front else WHITE_BACK_X_IN_BASE
    return (BASE_XY[0] + dx, BASE_XY[1] + TARGET_CENTER_Y_IN_BASE, table_top + OVER + BASE_T + 0.5 * PLATE_T)


def peel_anchor_world(table_top: float) -> tuple[float, float, float]:
    return (BASE_XY[0], BASE_XY[1] + FAR_EDGE_Y_IN_BASE, table_top + OVER + BASE_T)


def near_row_top_world(table_top: float, over: float = OVER) -> tuple[float, float, float]:
    """World point on the target plate's top face midway between its two near (-y) studs."""
    return (BASE_XY[0], BASE_XY[1] + NEAR_ROW_Y_IN_BASE, table_top + over + BASE_T + PLATE_T)


def sock_built_pos(table_top: float) -> tuple[float, float, float]:
    """World point at the top of the target plate's near stud row at build time."""
    x, y, z = near_row_top_world(table_top, over=OVER)
    return (x, y, z + STUD_H)


# Seated separator rotation in world: +x_s (handle) -> -y_world, +y_s (width) -> +x_world, +z_s -> +z_world
R_SEP_SEATED = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def separator_bench_pose(table_top: float, over: float = OVER) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Pose of separator on the bench at SEP_START_XY: rotated +HANDLE_DEG about +x_world from seated so the
    56 mm flat handle underside lies horizontal on the bench (``over`` m above TOP), handle pointing towards -y."""
    rot = axis_angle((1.0, 0.0, 0.0), +HANDLE_RAD) @ R_SEP_SEATED
    fulcrum_in_s = np.array([EDGE_X, 0.0, -EDGE_DZ], dtype=float)
    pos = np.array([SEP_START_XY[0], SEP_START_XY[1], table_top + over], dtype=float) - rot @ fulcrum_in_s
    return (float(pos[0]), float(pos[1]), float(pos[2])), matrix_to_quat(rot)


def separator_rocked_pose(table_top: float, rock_rad: float = 0.0, over: float = OVER) -> tuple[np.ndarray, np.ndarray]:
    """(pos, rot_matrix) of separator when seated at rock_rad = 0, or rocked handle-down (+x_world axis) by rock_rad
    about its lower fulcrum edge (EDGE_X, 0, -EDGE_DZ)."""
    seat_pos = np.array(near_row_top_world(table_top, over=over), dtype=float)
    if abs(rock_rad) < 1e-12:
        return seat_pos, R_SEP_SEATED.copy()
    fulcrum_in_s = np.array([EDGE_X, 0.0, -EDGE_DZ], dtype=float)
    # Fulcrum touches base free studs (0.2 mm below EDGE_DZ at 0 deg; settles onto stud top BASE_TOP + STUD_H)
    fulcrum_w = seat_pos + R_SEP_SEATED @ np.array([EDGE_X, 0.0, -(PLATE_T - STUD_H)], dtype=float)
    rot = axis_angle((1.0, 0.0, 0.0), rock_rad) @ R_SEP_SEATED
    return fulcrum_w - rot @ fulcrum_in_s, rot


def base_corners_xy(center_xy: tuple[float, float] = BASE_XY, yaw: float = 0.0) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]], dtype=float)
    hx, hy = 0.5 * BASE_W, 0.5 * BASE_L
    local = np.array([[-hx, -hy], [-hx, +hy], [+hx, +hy], [+hx, -hy]], dtype=float)
    return local @ rot.T + np.asarray(center_xy, dtype=float)


# --- 10. Primitive Shape Builders -----------------------------------------------------------------------------------


def base_shapes() -> list:
    """Green 8x16 base plate (origin at slab centre). Directly under target's 2x4 pocket the solid floor is recessed by
    BASE_POCKET_RECESS (1.0 mm) beneath a flush visual top skin so stop_load('peel') reads 100 % of any downward press;
    the free right-end studs (columns 0, 1) are solid so the separator's lower edge rocks on them."""
    solid_h = BASE_T - BASE_POCKET_RECESS
    far_cols = BASE_STUDS_Y - (TARGET_FIRST_COL + PLATE_STUDS_Y)  # columns 6..15, the open area beyond the plates
    shapes = [
        # Solid slab everywhere (recessed by 1.0 mm, plus full-height outer frame surrounding target's 16x32 pocket)
        box((BASE_W, BASE_L, solid_h), pos=(0.0, 0.0, -0.5 * BASE_POCKET_RECESS), color=LEGO_GREEN),
        # Full-height solid slab on the right (-y) free end (y < near edge of target) where separator fulcrum rocks
        box(
            (BASE_W, TARGET_FIRST_COL * PITCH - 0.0004, BASE_POCKET_RECESS),
            pos=(0.0, -0.5 * BASE_L + 0.5 * TARGET_FIRST_COL * PITCH, 0.5 * BASE_T - 0.5 * BASE_POCKET_RECESS),
            color=LEGO_GREEN,
        ),
        # Full-height solid slab on the left (+y) half beyond the 2x4 plates (columns 6..15)
        box(
            (BASE_W, far_cols * PITCH - 0.0004, BASE_POCKET_RECESS),
            pos=(0.0, +0.5 * BASE_L - 0.5 * far_cols * PITCH, 0.5 * BASE_T - 0.5 * BASE_POCKET_RECESS),
            color=LEGO_GREEN,
        ),
        # Visual flush top skin so the green plate draws completely solid and flat across all 8x16 cells
        box((BASE_W, BASE_L, BASE_T), pos=(0.0, 0.0, 0.0), color=LEGO_GREEN, solid=False),
    ]
    # Studs on the green base: columns 0..1 (free right end) and columns 6..15 (open area). Solid where something rests
    # on them: columns 0..1 and 6, rows 2..5 (the separator's lower edge rocks there, and a fingertip at either end of
    # the target plate stands on stud tops as on the real plate).
    outer_rows = (0, 1, BASE_STUDS_X - 2, BASE_STUDS_X - 1)  # rows outside the two white plates (x = +-20, +-28 mm)
    rocker_rows = (2, 3, 4, 5)  # rows within +-12 mm of the plate's centre line, under the separator's lower edge
    z_stud = 0.5 * BASE_T + 0.5 * STUD_H
    for j in range(BASE_STUDS_Y):
        if TARGET_FIRST_COL <= j < TARGET_FIRST_COL + PLATE_STUDS_Y:
            i_list = outer_rows
        else:
            i_list = tuple(range(BASE_STUDS_X))
        yj = -0.5 * BASE_STUDS_Y * PITCH + (j + 0.5) * PITCH
        for i in i_list:
            xi = -0.5 * BASE_STUDS_X * PITCH + (i + 0.5) * PITCH
            is_rocker_stud = j in (0, 1, TARGET_FIRST_COL + PLATE_STUDS_Y) and i in rocker_rows
            shapes.append(cylinder(STUD_R, STUD_H, pos=(xi, yj, z_stud), color=LEGO_GREEN, solid=is_rocker_stud))
    return shapes


def plate_2x4_shapes(color: tuple[float, float, float], *, stud_passes: tuple[str, ...] = (), body_passes: tuple[str, ...] = ()) -> list:
    """One LEGO 2x4 plate (design 3020): 15.8 x 31.8 x 3.2 mm slab with 8 studs on an 8.0 mm grid."""
    shapes = [box((PLATE_W, PLATE_L, PLATE_T), pos=(0.0, 0.0, 0.0), color=color, passes=body_passes)]
    z_stud = 0.5 * PLATE_T + 0.5 * STUD_H
    for j in range(PLATE_STUDS_Y):
        yj = -0.5 * PLATE_STUDS_Y * PITCH + (j + 0.5) * PITCH
        for i in range(PLATE_STUDS_X):
            xi = -0.5 * PLATE_STUDS_X * PITCH + (i + 0.5) * PITCH
            shapes.append(
                cylinder(
                    STUD_R,
                    STUD_H,
                    pos=(xi, yj, z_stud),
                    color=color,
                    solid=True,
                    passes=stud_passes,
                )
            )
    return shapes


def separator_shapes(pass_carriers: tuple[str, ...] = CARRIER_NAMES) -> list:
    """LEGO 96874 Brick and Axle Separator in frame S (origin = socket underside centre).
    - Front socket block: x_s in [-4.0, +4.0] mm, z_s in [0.0, +8.2] mm (sits flush on target's top face).
    - Lower rocker block: x_s in [+4.0, +17.0] mm, z_s in [-0.5, +8.2] mm (rear bottom edge at (EDGE_X, -EDGE_DZ)
      is the fulcrum on the tops of the base's outer free studs).
    - Sloped handle beam: 59.0 x 16.0 x 8.0 mm rising at HANDLE_DEG (35 deg) from (EDGE_X, -EDGE_DZ); lying on the
      bench on its underside the drawn separator is 75.9 x 16.0 x 19.2 mm.
    - Decorative top cross-axle & handle grip ridges (solid=False)."""
    rocker_l = EDGE_X - 0.5 * SOCKET_LEN  # 13.0 mm (+4.0 to +17.0 mm)
    rocker_h = SOCKET_H + EDGE_DZ  # 8.7 mm (-0.5 to +8.2 mm)
    # Sloped handle box: underside goes from (EDGE_X, -EDGE_DZ) along (cos a, 0, sin a) for HANDLE_BEAM_L,
    # thickness HANDLE_BEAM_T along (-sin a, 0, cos a)
    u = np.array([math.cos(HANDLE_RAD), 0.0, math.sin(HANDLE_RAD)], dtype=float)
    n = np.array([-math.sin(HANDLE_RAD), 0.0, math.cos(HANDLE_RAD)], dtype=float)
    handle_center = np.array([EDGE_X, 0.0, -EDGE_DZ], dtype=float) + 0.5 * HANDLE_BEAM_L * u + 0.5 * HANDLE_BEAM_T * n
    handle_quat = matrix_to_quat(axis_angle((0.0, 1.0, 0.0), -HANDLE_RAD))

    collide_l = EDGE_X - ROCKER_COLLIDE_FROM  # 12.0 mm (+5.0 to +17.0 mm)
    shapes = [
        # Shape 0: front 2-stud socket block (underside at z_s = 0.0): collider SOCKET_COLLIDE_W wide, drawn below
        box(
            (SOCKET_LEN, SOCKET_COLLIDE_W, SOCKET_H),
            pos=(0.0, 0.0, 0.5 * SOCKET_H),
            color=LEGO_ORANGE,
            friction=LEGO_FRICTION,
            passes=pass_carriers,
        ),
        # Shape 1: lower rocker belly block (underside at z_s = -EDGE_DZ, rear fulcrum at x_s = +EDGE_X); it passes the
        # target plate: the real socket's recess takes the plate's end as the plate turns against the rocking tool (with a
        # solid block the plate's near corner met it at 35 deg between them and the peel stopped at 10.4 deg: run l10t18)
        box(
            (collide_l, SEP_W, rocker_h),
            pos=(ROCKER_COLLIDE_FROM + 0.5 * collide_l, 0.0, 0.5 * (SOCKET_H - EDGE_DZ)),
            color=LEGO_ORANGE,
            friction=LEGO_FRICTION,
            passes=(*pass_carriers, "target"),
        ),
        # Shape 2: sloped handle beam (high-friction grippy sides for secure 16 mm pinch)
        box(
            (HANDLE_BEAM_L, SEP_W, HANDLE_BEAM_T),
            pos=(float(handle_center[0]), 0.0, float(handle_center[2])),
            quat=handle_quat,
            color=LEGO_ORANGE,
            friction=SEP_HANDLE_FRICTION,
            passes=pass_carriers,
        ),
        # The socket and lower block at full size, drawn only (solid=False)
        box((SOCKET_LEN, SEP_W, SOCKET_H), pos=(0.0, 0.0, 0.5 * SOCKET_H), color=LEGO_ORANGE, solid=False),
        box((rocker_l, SEP_W, rocker_h), pos=(0.5 * SOCKET_LEN + 0.5 * rocker_l, 0.0, 0.5 * (SOCKET_H - EDGE_DZ)), color=LEGO_ORANGE, solid=False),
        # Decorative 96874 top Technic cross-axle pin (solid=False)
        box((0.0048, 0.0018, AXLE_TOP_Z - SOCKET_H), pos=(AXLE_X, 0.0, 0.5 * (SOCKET_H + AXLE_TOP_Z)), color=LEGO_ORANGE, solid=False),
        box((0.0018, 0.0048, AXLE_TOP_Z - SOCKET_H), pos=(AXLE_X, 0.0, 0.5 * (SOCKET_H + AXLE_TOP_Z)), color=LEGO_ORANGE, solid=False),
    ]
    # Decorative handle grip ridges (5 transverse ribs on top of the handle)
    for k in range(5):
        s_rib = 0.016 + k * 0.008
        rib_pos = np.array([EDGE_X, 0.0, -EDGE_DZ], dtype=float) + s_rib * u + (HANDLE_BEAM_T + 0.0005) * n
        shapes.append(
            box(
                (0.0018, SEP_W - 0.002, 0.0012),
                pos=(float(rib_pos[0]), 0.0, float(rib_pos[2])),
                quat=handle_quat,
                color=LEGO_ORANGE_DARK,
                solid=False,
            )
        )
    return shapes


def dish_shapes(sides: int = 24) -> list:
    """Round white porcelain dish (120 mm OD, 25 mm high, origin at bottom centre)."""
    shapes = [cylinder(DISH_FLOOR_R + 0.002, DISH_FLOOR_T, pos=(0.0, 0.0, 0.5 * DISH_FLOOR_T), color=PORCELAIN_WHITE)]
    r_in = DISH_FLOOR_R
    r_out = DISH_R - DISH_WALL
    z_in = DISH_FLOOR_T
    z_out = DISH_H
    slope = math.atan2(z_out - z_in, r_out - r_in)
    seg_len = math.hypot(r_out - r_in, z_out - z_in)
    r_mid = 0.5 * (r_in + r_out) + 0.5 * DISH_WALL * math.sin(slope)
    z_mid = 0.5 * (z_in + z_out) - 0.5 * DISH_WALL * math.cos(slope)
    width = 2.0 * DISH_R * math.tan(math.pi / sides) + 0.0015
    for k in range(sides):
        a = 2.0 * math.pi * k / sides
        q = matrix_to_quat(axis_angle((0.0, 0.0, 1.0), a) @ axis_angle((0.0, 1.0, 0.0), -slope))
        shapes.append(
            box(
                (seg_len + 0.001, width, DISH_WALL),
                pos=(r_mid * math.cos(a), r_mid * math.sin(a), z_mid),
                quat=q,
                color=PORCELAIN_WHITE,
            )
        )
        # Subtle porcelain rim band so the white rim edge shows clearly at 320x176
        shapes.append(
            box(
                (0.003, width, 0.002),
                pos=((DISH_R - 0.0015) * math.cos(a), (DISH_R - 0.0015) * math.sin(a), DISH_H - 0.001),
                yaw=a,
                color=PORCELAIN_RIM,
                solid=False,
            )
        )
    return shapes


def carrier_shapes(all_pass_names: tuple[str, ...]) -> list:
    """Tiny non-colliding box for an invisible serial-chain carrier body."""
    return [box((0.002, 0.002, 0.002), passes=all_pass_names, color=LEGO_GREEN)]


def bench_mark_shapes() -> list:
    """Subtle corner tape marks (solid=False) around the green base's start footprint on the bench."""
    shapes = []
    arm_l, w, h = 0.016, 0.004, 0.0005
    hx, hy = 0.5 * BASE_W, 0.5 * BASE_L
    bx, by = BASE_XY
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            cx = bx + sx * (hx + 0.5 * w + 0.001)
            cy = by + sy * (hy + 0.5 * w + 0.001)
            shapes.append(box((arm_l, w, h), pos=(cx - sx * (0.5 * arm_l - 0.5 * w), cy, 0.5 * h), color=TAPE_MARK, solid=False))
            shapes.append(box((w, arm_l, h), pos=(cx, cy - sy * (0.5 * arm_l - 0.5 * w), 0.5 * h), color=TAPE_MARK, solid=False))
    return shapes


def build_scene(table_top: float) -> list:
    """Declare all scene bodies and pure serial joint chains for ``task.py`` (zero closed loops)."""
    body("bench_marks", bench_mark_shapes(), pos=(0.0, 0.0, table_top), mass=None)
    body(
        "base",
        base_shapes(),
        pos=base_built_pos(table_top),
        mass=BASE_MASS - PEEL_LINK_MASS,
        color=LEGO_GREEN,
        friction=BASE_FRICTION,
        contact_offset=CONTACT_THIN,
        contacts=True,
    )
    for wname, is_front in (("white_f", True), ("white_b", False)):
        body(
            wname,
            plate_2x4_shapes(LEGO_WHITE, body_passes=CARRIER_NAMES, stud_passes=CARRIER_NAMES),
            pos=white_built_pos(table_top, front=is_front),
            mass=PLATE_MASS,
            color=LEGO_WHITE,
            friction=LEGO_FRICTION,
            contact_offset=CONTACT_THIN,
        )
        weld(f"w_{wname}", wname, "base")

    peel_anc = peel_anchor_world(table_top)
    body("peel_c", carrier_shapes(ALL_BODY_NAMES), pos=peel_anc, mass=PEEL_LINK_MASS, inertia=CARRIER_INERTIA, contact_offset=CONTACT_THIN)
    hinge(
        "peel",
        "peel_c",
        "base",
        anchor=peel_anc,
        axis=(-1.0, 0.0, 0.0),
        limits=(0.0, math.radians(PEEL_LIMIT_DEG)),
        spring=PEEL_K,
        rest=0.0,
        damping=PEEL_DAMP,
        stop=PEEL_STOP_K,
    )
    body(
        "target",
        plate_2x4_shapes(LEGO_BLACK, stud_passes=("separator", *CARRIER_NAMES), body_passes=CARRIER_NAMES),
        pos=target_built_pos(table_top),
        mass=TARGET_MASS_SIM,
        inertia=TARGET_INERTIA,
        color=LEGO_BLACK,
        friction=LEGO_FRICTION,
        contact_offset=CONTACT_THIN,
        contacts=True,
    )
    weld("w_target", "target", "peel_c", anchor=peel_anc)

    sock_pos = sock_built_pos(table_top)
    for cname in ("c1", "c2", "sock"):
        body(cname, carrier_shapes(ALL_BODY_NAMES), pos=sock_pos, mass=CARRIER_MASS, inertia=CARRIER_INERTIA, contact_offset=CONTACT_THIN)
    slide("pull", "c1", "target", axis=(0.0, 0.0, 1.0), anchor=sock_pos, limits=(0.0, STUD_H + PULL_OVER), spring=PULL_K, rest=0.0, damping=PULL_DAMP)
    slide("rack_x", "c2", "c1", axis=(0.0, 1.0, 0.0), anchor=sock_pos, limits=(-RACK_X_LIM, RACK_X_LIM), spring=RACK_X_K, rest=0.0, damping=RACK_X_DAMP)
    hinge("rack", "sock", "c2", anchor=sock_pos, axis=(1.0, 0.0, 0.0), limits=(-math.radians(RACK_LIM_DEG), math.radians(RACK_LIM_DEG)), spring=RACK_K, rest=0.0, damping=RACK_DAMP)

    sep_pos, sep_quat = separator_bench_pose(table_top)
    body(
        "separator",
        separator_shapes(CARRIER_NAMES),
        pos=sep_pos,
        quat=sep_quat,
        mass=SEP_MASS_SIM,
        inertia=SEP_INERTIA,
        center=SEP_CENTER_IN_S,
        color=LEGO_ORANGE,
        friction=SEP_HANDLE_FRICTION,
        friction_combine="max",
        contact_offset=CONTACT_THIN,
        contacts=True,
    )
    weld("w_seat", "separator", "sock", made=False)
    body(
        "dish",
        dish_shapes(),
        pos=(DISH_XY[0], DISH_XY[1], table_top + OVER),
        mass=DISH_MASS,
        color=PORCELAIN_WHITE,
        friction=DISH_FRICTION,
        contact_offset=CONTACT_THIN,
        contacts=True,
    )
    mk.sense_contact("separator", "target")
    mk.sense_contact("separator", "base")
    return list(mk.MECH.bodies.values())


def summary() -> dict:
    return {
        "row_lever_mm": round(1000 * ROW_LEVER, 2),
        "peel_hold_equiv_n": round(PEEL_HOLD_NM / ROW_LEVER, 2),
        "sep_hold_n": SEP_HOLD_N,
        "press_n": PRESS_N,
        "key_figures": list(KEY_FIGURES),
    }


if __name__ == "__main__":
    print(json.dumps(summary(), indent=2))
