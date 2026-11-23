"""Every figure the SB50 mating task rests on, each with its source.

Source kinds (``FIGURES`` holds each figure with its kind and its words; the README's table is built from it):
- ``maker``: Anderson Power Products' data sheet DS-SB50 and drawings 111923S1, 113923S1 and 114209S1 (opened at
  HTTP 200 on 2026-11-22; the words quoted are the documents').
- ``our fixture``: the bench mat and the two cable clamps of our setup, cut to the stated sizes.
- ``estimate``: a figure no document gives, with the clause it was derived from.

Connector frame (both housings): origin at the middle of the housing's rear (cable) face; x forward to the mating
nose, y across the 36.5 mm width, z up through the face that carries the handle and the '+' mark.
Station frame: metres from the midpoint between the arm bases, x forward, y left, z up.
"""

from __future__ import annotations

import math

OVER = 0.0008  # m: bodies are built this far above what they rest on
CONTACT_OFFSET = 0.0008

MAKER, FIXTURE, ESTIMATE = "maker", "our fixture", "estimate"

DS_SB50_URL = "https://p1.aprimocdn.net/idealindustries/d7c0a682-e691-4b60-8e76-b3f000737d74/ap-web_product-assets_default_data-sheets_ds-sb50_Original%20file.pdf"
DWG_HOUSING_URL = "https://p1.aprimocdn.net/idealindustries/87704745-9c35-4c17-b150-b3f000738119/ap-web_product-assets_default_drawings_111923s1_Original%20file.pdf"
DWG_HANDLE_URL = "https://p1.aprimocdn.net/idealindustries/f190eeb5-cd3a-47b3-a941-b3f00073821e/ap-web_product-assets_default_drawings_113923s1_Original%20file.pdf"
DWG_COVER_URL = "https://p1.aprimocdn.net/idealindustries/a49a7d1e-4ed8-43d7-b117-b3f0007389c9/ap-web_product-assets_default_drawings_114209s1_Original%20file.pdf"
ONET_URL = "https://www.onetonline.org/link/summary/53-7051.00"

FIGURES: dict[str, tuple[float, str, str, str]] = {}


def figure(name: str, value: float, unit: str, source: str, note: str) -> float:
    if source not in (MAKER, FIXTURE, ESTIMATE):
        raise ValueError(f"{name}: unknown source kind {source!r}")
    if name in FIGURES:
        raise ValueError(f"{name}: figure given twice")
    FIGURES[name] = (float(value), unit, source, note)
    return float(value)


# --- Anderson SB50 housing, grey P/N 992, with two 1307 low-force contacts (DS-SB50, DWG 111923S1) -------------------
SB_L = figure("housing length", 0.0480, "m", MAKER, f"DWG 111923S1 rev 8: '[48.0] 1.89' ({DWG_HOUSING_URL})")
SB_W = figure("housing width", 0.0365, "m", MAKER, f"DWG 111923S1 rev 8: '[36.5] 1.44' ({DWG_HOUSING_URL})")
SB_H = figure("housing height", 0.0159, "m", MAKER, f"DWG 111923S1 rev 8: '[15.9] .63' ({DWG_HOUSING_URL})")
SB_MATED_L = figure("mated pair, rear face to rear face", 0.0813, "m", MAKER,
                    f"DWG 111923S1 rev 8: '[81.3] 3.20 REF.', 'MATED PAIR' ({DWG_HOUSING_URL})")
NOSE_L = 2.0 * SB_L - SB_MATED_L  # 14.7 mm: how far each housing's nose reaches into the other when mated
MATE_N = figure("mating force", 44.0, "N", MAKER,
                f"DS-SB50 rev 9, specifications: 'Avg. Mating / Unmating Force', 'Wire-to-Wire Low Force Contacts' "
                f"'10' Lbf, '44' N ({DS_SB50_URL})")
WIRE_OD = figure("lead insulation diameter", 0.0112, "m", MAKER,
                 f"DS-SB50 rev 9: 'Max. Wire Insulation Diameter' '0.440' in., '11.200' mm ({DS_SB50_URL})")
# The nose: a lower tongue between two upper side walls, the second housing turned over fills the gaps (our
# simplification of the drawing's genderless nose; DS-SB50: 'Genderless design mates with itself').
WALL_W = figure("nose side wall width", 0.0040, "m", ESTIMATE,
                "scaled from the drawing's front view")
FIT = figure("nose fit, each side and at the middle", 0.0004, "m", ESTIMATE,
             "a sliding fit of moulded polycarbonate (1.1 mm between the tongues)")
MID_GAP = 0.0011  # m between the two tongues of a mated pair
TONGUE_H = (SB_H - MID_GAP) / 2.0  # 7.4 mm
TONGUE_W = SB_W - 2.0 * WALL_W - 2.0 * FIT  # 27.7 mm
LEAD_IN_R = figure("rounded nose corners", 0.0020, "m", ESTIMATE,
                   "the lead-in of a moulded nose")
NOSE_SHORT = 0.0003  # m: the nose ends 0.3 mm short, so the mated pair stops at 81.0 mm (inside the 81.3 +-2 mm band)

# --- Red T-handle SB50-HDL-RED (DWG 113923S1) ------------------------------------------------------------------------
HDL_L = figure("handle length", 0.0567, "m", MAKER, f"DWG 113923S1 rev 5: '[56.7] 2.23', 'HANDLE - POLYCARBONATE' ({DWG_HANDLE_URL})")
HDL_W = figure("handle crossbar width", 0.0486, "m", MAKER, f"DWG 113923S1 rev 5: '[48.6] 1.92' ({DWG_HANDLE_URL})")
HDL_STEM_W = figure("handle stem width", 0.0139, "m", MAKER, f"DWG 113923S1 rev 5: '[13.9] .55' ({DWG_HANDLE_URL})")
HDL_T = figure("handle height at the crossbar", 0.0134, "m", MAKER, f"DWG 113923S1 rev 5: '[13.4] .53' ({DWG_HANDLE_URL})")
HDL_PROJ = figure("handle reach behind the housing", 0.0251, "m", MAKER,
                  f"DWG 113923S1 rev 5, 'SB50 MOUNTING ORIENTATION': '[25.1+-0.8] .99+-.03', '[73.4] 2.89 REF.' ({DWG_HANDLE_URL})")
HDL_FRONT_T = figure("handle height over the housing", 0.0095, "m", ESTIMATE,
                     "scaled from the drawing's side view against the 13.4 mm")
HDL_CROSS_D = figure("crossbar depth", 0.0100, "m", ESTIMATE, "scaled from the drawing's top view against the 56.7 mm length")

# --- The two 6 AWG leads where they leave the housing -----------------------------------------------------------------
LEAD_W = 2.0 * WIRE_OD  # 22.4 mm: two leads side by side
STUB_L = figure("stiff lead length behind the housing", 0.066, "m", ESTIMATE,
                "6 AWG cable bends little within 66 mm of the housing")
HEAD_MASS = figure("connector mass with handle and stiff lead", 0.115, "kg", ESTIMATE,
                   "32 g housing + 2 x 18 g contacts + 22 g handle and screws + 25 g for 66 mm of two 6 AWG leads")

# --- Red dust cover with strap, Cat. No. 134G1 (DWG 114209S1) --------------------------------------------------------
CAP_W = figure("dust cover width A", 0.0465, "m", MAKER, f"DWG 114209S1 rev 2 table: '134G1', 'SB50', A '1.83' in, '46.5' mm ({DWG_COVER_URL})")
CAP_H = figure("dust cover height C", 0.0279, "m", MAKER, f"DWG 114209S1 rev 2 table: '134G1', C '1.10' in, '27.9' mm ({DWG_COVER_URL})")
CAP_ON = figure("depth the cover sits over the nose", 0.0160, "m", ESTIMATE, "the 14.7 mm nose plus 1.3 mm, up to the handle's front end")
CAP_FRONT = figure("cover front wall", 0.0060, "m", ESTIMATE, "the side walls are (46.5 - 36.5) / 2 = 5 mm; the front a little thicker")
CAP_MASS = figure("dust cover mass", 0.038, "kg", ESTIMATE, "about 32 cm3 of elastomer at 1.2 g/cm3")
CAP_PULL_N = figure("force to pull the cover off", 10.0, "N", ESTIMATE,
                    "a snug elastomer slip fit (the maker: 'NOT A HERMETIC SEAL'), far above the 0.8 N the lead slides with")
STRAP_L = figure("cover strap length", 0.200, "m", ESTIMATE,
                 "a lanyard long enough that the cover, laid aside, stays put while its connector is lifted 10 cm and carried")
CAP_GAP = 0.0002  # m: the cover's cavity is the nose's size plus this, so it slides without rubbing in the simulator

# --- Our bench fixture: the flat yellow mat, the two cable clamps and the block under the right housing -------------
MAT_X = figure("mat depth (x)", 0.140, "m", FIXTURE, "flat yellow rubber mat")
MAT_Y = figure("mat length (y)", 0.110, "m", FIXTURE, "flat yellow rubber mat")
MAT_T = figure("mat thickness", 0.004, "m", FIXTURE, "flat, no raised edge")
MAT_MU = figure("mat friction", 0.7, "-", ESTIMATE, "rubber against polycarbonate")
MAT_XY = (0.255, 0.000)
CLAMP_XY = {"l": (0.360, 0.200), "r": (0.360, -0.200)}  # m: each cable clamped to the bench here, ahead of its connector (our fixture)
BLOCK_H = figure("block under the right housing, height", 0.020, "m", FIXTURE,
                 "a grey foam block under the right housing only: its leads stand free behind it, so a hand can grip them all round")
BLOCK_XY = (0.040, 0.073)  # m (x by y): from the right nose's tip to 25 mm behind the housing (under the crossbar), so the
# lead cable's pull at the stiff leads' end cannot tip the connector back; the leads stand free behind the crossbar
CABLE_R = WIRE_OD / 2.0  # each lead jointed in two links of this radius
CABLE_SLACK = 0.095  # m each half of the jointed lead is longer than half its span: 35 cm of lead from housing to clamp
# (with 29 cm the right lead went taut round the fingers turning its connector over and pulled it 27 deg out of the pinch)
CABLE_LINK_MASS = 0.020

# --- Start: both housings lie '+' face up (0 deg roll), noses towards each other along y, the right one on its block --
LEFT_NOSE_Y = 0.101   # m: the left nose's tip (its cover's front at 0.107, the mat's edge at 0.055)
RIGHT_NOSE_Y = -0.067  # m: 12 mm off the mat; the right hand reaches behind it from here
CONN_X = 0.255

# --- The mate and the scoring (the same numbers in task.py, task.json and README) -----------------------------------
ENGAGE_OVERLAP = 0.0015  # m the noses overlap before the contacts take up the mating force
ENGAGE_ROLL_DEG = 20.0  # deg either side of a half turn
ENGAGE_OFFSET = 0.0025  # m between the axes
ENGAGE_ANGLE_DEG = 8.0
WIPE_K = 20000.0  # N/m: the friction-like spring that carries the mating force (it gives 2.2 mm before it slides)
CAP_CLEAR = 0.005  # m: the cover counts as off when no part of it is within 5 mm of the nose
HALF_SPAN = 0.0910  # m: noses overlapping 5 mm or more
FULL_SPAN = (0.0793, 0.0835)  # m: 81.3 mm -2.0 / +2.2
TURN_TOL_DEG = 20.0  # deg: one housing turned over, 180 +- 20 deg relative to the other
AXIS_OFFSET = 0.004  # m
AXIS_ANGLE_DEG = 10.0
MAT_INSET = 0.020  # m: the pair's midpoint at least 2 cm inside the mat's edges
UNDERSIDE_MAX = 0.018  # m: each housing's lowest point at most 18 mm over the mat (the turned-over handle stands 13.4 mm)
PITCH_MAX_DEG = 15.0
HANDS_CLEAR = 0.030  # m
REST_LIN = 0.010  # m/s
REST_ANG = 0.50  # rad/s
HOLD_STEPS = 20  # 1.0 s at 20 Hz


def summary() -> dict:
    counts = {k: sum(1 for v in FIGURES.values() if v[2] == k) for k in (MAKER, FIXTURE, ESTIMATE)}
    return {"counts": counts, "total": len(FIGURES)}


def start_tilt() -> float:
    """Nose-up pitch (rad) at which the left connector rests with its cover on: on the stiff lead's far end and on the
    cover's rear bottom edge (the cover's walls stand 6 mm below the housing)."""
    wall = (CAP_H - SB_H) / 2.0
    stub = (-STUB_L, -WIRE_OD / 2.0)  # the leads' underside: 5.6 mm under the axis
    cover = (SB_L - CAP_ON, -SB_H / 2.0 - wall)
    return math.atan2(stub[1] - cover[1], cover[0] - stub[0])
