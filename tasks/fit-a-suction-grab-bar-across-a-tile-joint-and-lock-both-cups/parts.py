"""The grab bar task's figures and shapes: a suction grab bar with two cups and two flip levers, and the tiled panel.

Every size, mass, angle, force and margin the task, the demo, the checks and the tests use is one constant here.
``FIGURES`` gives each key figure its source kind (``maker``, ``standard``, ``listing``, ``checked`` or ``estimate``),
the source itself (the page and its words, or how the estimate was derived) and how the first minutes of a real trial
check it on the bought product.

Frames. The world: x forward from the midpoint between the arm bases, y left, z up; heights over the bench are
``TOP`` + z. The bar's own frame B: origin halfway between the two cups' middles, on the plane of their faces (where
they touch the tile); x into the wall, y along the bar to the left cup, z up. At the start B is the world turned by
nothing and moved to ``BAR_AT``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

SOURCE_KINDS = ("maker", "standard", "listing", "checked", "estimate")
WENKO_2019 = "https://web.archive.org/web/20191211170940/https://shop.wenko.de/bad/sicherheit-komfort/sicherheit-im-bad/wandgriff-secura-7820100"
WENKO_2022 = "https://web.archive.org/web/20220704091925/https://shop.wenko.de/bad/sicherheit-komfort/sicherheit-im-bad/wandgriff-secura"
SANISMART = "https://www.sanismart.de/wandgriff-secura-mit-saugsystem--2821"
DRIVE_TAG = "https://web.archive.org/web/20181221042355/http://cdn.drivemedical.com/media/files/3583/RTL13083_RTL13085_hangtag_Rev1_04_29_16%20(1).pdf"
TOOM_TILE = "https://toom.de/p/wandfliese-arktis-steingut-weiss-glaenzend-25-x-40-cm/2050669"
OBI_SPACERS = "https://www.obi.de/p/5412630/lux-tools-fliesenkreuze-3-mm-classic-250-stueck"
OBI_BOARD = "https://www.obi.de/p/4560678/multiplex-platte-birke-250-cm-x-125-cm-x-1-8-cm-braun-zuschnitt"
OBI_BRACKETS = "https://www.obi.de/p/6406458/winkelverbinder-70-mm-x-70-mm-x-55-mm-verzinkt-20-stueck"
DIN_18352_PAGE = "https://www.sanier.de/bodenbelag/fliesen/fliesen-fast-fugenlos-verlegen"  # a renovation page quoting DIN 18352


@dataclass(frozen=True)
class Figure:
    """A key figure: what it is, the constants that carry it, its source kind, its source (the page and its words, or
    'estimate: ' and how it was derived) and how the first minutes of a real trial check it on the bought product."""

    figure: str
    constants: tuple
    kind: str
    source: str
    check: str

    def __post_init__(self) -> None:
        if self.kind not in SOURCE_KINDS:
            raise ValueError(f"{self.figure}: kind must be one of {SOURCE_KINDS}, got {self.kind!r}")
        if self.kind in ("maker", "listing") and "https://" not in self.source:
            raise ValueError(f"{self.figure}: a {self.kind} source gives its page")
        if self.kind == "estimate" and not self.source.startswith("estimate: "):
            raise ValueError(f"{self.figure}: an estimate says how it was derived ('estimate: ...')")

# --- the bar (Wenko Wandgriff Secura, 29 cm, white, maker's no. 7820100) ----------------------------------------------
BAR_L = 0.290  # m: overall length, along the bar
BAR_W = 0.115  # m: overall width across the bar (up the wall)
BAR_H = 0.085  # m: overall depth, from the wall to the front of the grip
BAR_MASS = 0.265  # kg: the whole bar, cups and levers with it
CUP_D = 0.095  # m: a cup's diameter (its rubber rim)
CUP_PITCH = 0.181  # m: from one cup's middle to the other's
CUP_T = 0.008  # m: the rubber cup's depth, face to housing
HOUSING_D = 0.070  # m: the round housing behind each cup
HOUSING_T = 0.032  # m: its depth
GRIP_D = 0.038  # m: the grip's thickness, where a hand takes it
GRIP_X = -(BAR_H - GRIP_D / 2)  # m in B: the grip's middle, out from the wall (derived)
LEVER_ARM = 0.032  # m: a lever's pivot to where a finger works it
LEVER_L = 0.045  # m: a lever's length
LEVER_WIDTH = 0.030  # m: its height (up the wall)
LEVER_T = 0.005  # m: its thickness
LEVER_PIVOT = (-0.058, 0.052, 0.0)  # m in B (x; out along the bar from its cup's middle; z): each lever's pivot, at its housing's outer end near the front, on a vertical axis
LEVER_DEG = 110.0  # deg: a lever's travel from released (0: swung out from the housing's end) to locked (lying along it, its free end towards the wall)
LEVER_CENTRE_DEG = 95.0  # deg from released: the over-centre point, where it snaps
LEVER_N = 22.0  # N at LEVER_ARM, where a finger works the lever, to pass centre
LEVER_HOLD = 0.2  # share of LEVER_N the lever's cam gives at the locked end, holding it there: a model choice (the force rises to LEVER_N at centre)
LEVER_CAM_DEG = 70.0  # deg from released: where the cam starts to resist; from released to here the lever is loose, and let go short of centre it comes to rest here
LEVER_MASS = 0.010  # kg: one lever
CUP_MASS = 0.020  # kg: one rubber cup
INDICATOR_D = 0.016  # m: the round red/green tab on top of each housing
INDICATOR_AT = (-0.024, HOUSING_D / 2)  # m in B (x, z): its middle, on the top of the housing over its cup's middle
PRESS_N = 12.0  # N: the press of a cup on the tile, within the last PRESS_S, that a lever needs to seal it
PRESS_S = 0.2  # s: how recent that press must be
HOLD_N = 40.0  # N: a sealed cup on tile holds more than this straight out
HOLD_JOINT_N = 3.0  # N: a cup across the joint holds less than this (it is not sealed: the task's weld is not made)
JOINT_MARGIN = 0.0127  # m: a cup's rim at least this far from the joint seals (1/2 in)
EDGE_MARGIN = 0.005  # m: and at least this far from the outer edges of the tile it lies on
FACE_TILT_DEG = 3.0  # deg: a cup's face within this of the tile's plane counts as flat on it
FACE_GAP = 0.001  # m: and within this of the tile's face
RUBBER_FRICTION = (0.9, 0.8)  # static, dynamic: the rubber cups on glazed tile
LEVER_CONTACT = (25000.0, 32.0)  # N/m, N s/m: what touches a lever presses in 2 mm at 50 N, damped (a 10 g lever pinched rigidly between two finger pads chattered at 0.3-0.5 m/s and let go: check run gb5d8)
BAR_FRICTION = (0.5, 0.45)  # the bar's plastic

# --- the panel: two white glossy wall tiles on an upright plywood board ---------------------------------------------
TILE_W = 0.250  # m: a tile's width
TILE_H = 0.400  # m: its height
TILE_T = 0.008  # m: its thickness
JOINT_W = 0.003  # m: the open joint between the two tiles
BOARD_W = 0.520  # m: the board's width
BOARD_H = 0.400  # m: its height
BOARD_T = 0.018  # m: its thickness

# --- where things are (the layout; one for every seed) ---------------------------------------------------------------
PANEL_X = 0.34  # m: the tiles' face, facing the arms
JOINT_Y = 0.0  # m: the joint, upright
BAR_Y0 = 0.06  # m: the bar's middle at the start: its right cup across the joint, the left wholly on the left tile
BAR_Z = 0.070  # m over the bench: the cups' middles (and the grip's); the panel is a test fixture, so the height is ours: at 70 mm every grasp is at 74 mm or lower, under the 16 cm grasp height
MOVE = 0.060  # m: the bar's move towards -y that puts the joint midway between the cups

FIGURES = (
    Figure("Bar size", ("BAR_L", "BAR_W", "BAR_H"), "maker", f"maker: 'Wandgriff Secura Art.-Nr. 7820100', 'Maße (B x H x T): 29 x 11.5 x 8.5 cm' ({WENKO_2019})", "tape measure"),
    Figure("Bar mass", ("BAR_MASS",), "listing", f"listing: 'Gewicht 29 cm: 265 g' ({SANISMART})", "kitchen scale"),
    Figure("Cup diameter", ("CUP_D",), "listing", f"listing: 'Durchmesser der Saugnäpfe 9,5 cm' ({SANISMART})", "caliper"),
    Figure("Cups' spacing", ("CUP_PITCH",), "estimate", "estimate: 181 mm between the cups' middles puts the levers' outer faces at the ends of the maker's 290 mm", "ruler"),
    Figure("Cup, housing and grip sections", ("CUP_T", "HOUSING_D", "HOUSING_T", "GRIP_D"), "estimate", "estimate: an 8 mm cup, a 70 x 32 mm housing and 45 mm to the grip's front make the maker's 8.5 cm depth; a 38 mm grip for a hand", "caliper"),
    Figure("Lever size, pivot", ("LEVER_ARM", "LEVER_L", "LEVER_WIDTH", "LEVER_T", "LEVER_PIVOT"), "estimate", "estimate: a 45 x 30 x 5 mm lever at each housing's outer end on a vertical pivot, lying along it when locked, as the makers' photos show the named bars' levers; worked 32 mm out", "caliper"),
    Figure("Lever travel, centre, cam", ("LEVER_DEG", "LEVER_CENTRE_DEG", "LEVER_CAM_DEG"), "estimate", "estimate: an over-centre flip lever: 110 deg from released to locked, centre 15 deg short of locked, loose above 70 deg", "protractor app"),
    Figure("Lever force at centre, held locked", ("LEVER_N", "LEVER_HOLD"), "estimate", f"estimate: 22 N 32 mm out from the pivot (0.70 N m), a firm thumb's push; the US twin's maker: 'The resistance of the suction pressure generated must be clearly felt' ({DRIVE_TAG})", "luggage scale on the lever's tip"),
    Figure("Press a seal needs", ("PRESS_N", "PRESS_S"), "estimate", f"estimate: 12 N, so that a hand pressing the bar on seals it and a touch does not ('durch Aufdrücken des Saugtellers und Umlegen des Hebels': {WENKO_2019}); within 0.2 s of the lock", "panel lying on a kitchen scale"),
    Figure("Hold on tile; across the joint", ("HOLD_N", "HOLD_JOINT_N"), "estimate", f"estimate: well under the listing's 'Gleichgewichtsstütze, die bis zu 30 kg belastbar ist' (294 N for the bar: {SANISMART}); across the joint air gets in and only the rubber's grip holds", "luggage scale on the grip"),
    Figure("Margin from the joint", ("JOINT_MARGIN",), "maker", f"maker (the US twin): 'The discs should be positioned at least 1/2\u201d away from all grout lines or joints' ({DRIVE_TAG})", "ruler from each rim to the joint"),
    Figure("Margin from the tiles' outer edges", ("EDGE_MARGIN",), "estimate", "estimate: the rim's lip wholly on glaze where the panel's tiles end (not a grout line)", "ruler from each rim to the tile's edge"),
    Figure("Indicator tabs", ("INDICATOR_D", "INDICATOR_AT"), "estimate", f"estimate: a 16 mm red/green tab on top of each housing for the maker's 'Ampelanzeige für den Saughalt: OK=Grün, Achtung, locker=Rot' ({WENKO_2022}), where the makers' photos show it", "the tabs read at the start: red right, green left"),
    Figure("Lever, cup mass", ("LEVER_MASS", "CUP_MASS"), "estimate", "estimate: a 45 x 30 x 5 mm plastic plate is 7.1 g at 1.05 g/cm3, 10 g with its pin and cam; a 2 mm rubber shell of 95 mm is 17 g, 20 g with its rim", "kitchen scale"),
    Figure("Tiles", ("TILE_W", "TILE_H", "TILE_T"), "listing", f"listing: 'Wandfliese Arktis Steingut weiß glänzend 25 x 40 cm', 'Stärke von 8 mm', 'Glasiert Ja' ({TOOM_TILE})", "tape measure, caliper"),
    Figure("Joint", ("JOINT_W",), "listing", f"listing: LUX-TOOLS Fliesenkreuze 'Größe: 3 mm' ({OBI_SPACERS}); inside DIN 18352's 'zwei bis acht Millimetern' for tiles over 100 mm, as quoted by {DIN_18352_PAGE}", "caliper"),
    Figure("Board", ("BOARD_W", "BOARD_H", "BOARD_T"), "listing", f"listing: 'Multiplex-Platte Birke', 'Stärke 1,8 cm', cut to 520 x 400 mm in the shop ({OBI_BOARD})", "tape measure, caliper"),
    Figure("A cup flat on the tile", ("FACE_TILT_DEG", "FACE_GAP"), "estimate", "estimate: the rule's tolerance for a cup lying on the tile: 3 deg, 1 mm", "-"),
    Figure("Frictions", ("RUBBER_FRICTION", "BAR_FRICTION"), "estimate", "estimate: soft rubber on glazed tile 0.9 / 0.8; hard plastic 0.5 / 0.45", "an unsealed cup under a known weight pulled along the tile with the luggage scale"),
    Figure("A lever's contact", ("LEVER_CONTACT",), "estimate", "estimate: the finger pads' rubber gives 2 mm at 50 N", "-"),
)

# --- derived (not figures) -------------------------------------------------------------------------------------------
CUP_R = CUP_D / 2
CUP_Y = {"l": CUP_PITCH / 2, "r": -CUP_PITCH / 2}  # each cup's middle along the bar (B)
LEVER_RAD, CENTRE_RAD, CAM_RAD = math.radians(LEVER_DEG), math.radians(LEVER_CENTRE_DEG), math.radians(LEVER_CAM_DEG)
PANEL_BACK = PANEL_X + TILE_T + BOARD_T  # m: the board's back
TILE_EDGES = {"left": (JOINT_Y + JOINT_W / 2, JOINT_Y + JOINT_W / 2 + TILE_W), "right": (JOINT_Y - JOINT_W / 2 - TILE_W, JOINT_Y - JOINT_W / 2)}  # y spans


def out_sign(side: str) -> float:
    """+1 for the left lever (its housing's outer end towards +y), -1 for the right."""
    return math.copysign(1.0, CUP_Y[side])


def lever_pivot_b(side: str) -> tuple[float, float, float]:
    return (LEVER_PIVOT[0], CUP_Y[side] + out_sign(side) * LEVER_PIVOT[1], LEVER_PIVOT[2])


def lever_dir_b(side: str, theta: float) -> tuple[float, float, float]:
    """The lever's direction (B), pivot to free end, ``theta`` rad from released: locked (LEVER_RAD) it points into the
    wall along the housing's end; released it is swung out about its vertical axis, away from the bar's middle."""
    phi = LEVER_RAD - theta
    return (math.cos(phi), out_sign(side) * math.sin(phi), 0.0)


def lever_out_b(side: str, theta: float) -> tuple[float, float, float]:
    """The lever's outer face's normal (B): the way its free end moves as it is released (theta falling)."""
    phi = LEVER_RAD - theta
    return (-math.sin(phi), out_sign(side) * math.cos(phi), 0.0)


def lever_axis_b(side: str) -> tuple[float, float, float]:
    """The hinge's axis (B): a turn by a positive angle about it moves the lever towards locked."""
    return (0.0, 0.0, -out_sign(side))


def lever_tip_b(side: str, theta: float, reach: float = LEVER_ARM) -> tuple[float, float, float]:
    """A point on lever ``side`` ('l' or 'r') ``reach`` m from its pivot along its middle, ``theta`` rad from
    released, in B."""
    p, d = lever_pivot_b(side), lever_dir_b(side, theta)
    return (p[0] + reach * d[0], p[1] + reach * d[1], p[2] + reach * d[2])


def rim_clearance(cup_middle_y: float, cup_middle_z: float) -> tuple[float, float]:
    """m: how far a cup's rim (a circle of CUP_R about its middle, on the tile's face) keeps (from the joint, from the
    outer edges of the tile its middle lies on); the first is negative when the rim reaches across the joint (by how
    far), the second when it reaches past an edge."""
    lo, hi = cup_middle_y - CUP_R, cup_middle_y + CUP_R
    joint_lo, joint_hi = JOINT_Y - JOINT_W / 2, JOINT_Y + JOINT_W / 2
    if lo >= joint_hi:  # wholly on the left tile
        joint = lo - joint_hi
    elif hi <= joint_lo:  # wholly on the right tile
        joint = joint_lo - hi
    else:  # across the joint: how far it reaches over
        joint = -min(hi - joint_lo, joint_hi - lo)
    side = TILE_EDGES["left"][1] - hi if cup_middle_y >= JOINT_Y else lo - TILE_EDGES["right"][0]
    return joint, min(side, cup_middle_z - CUP_R, TILE_H - (cup_middle_z + CUP_R))


def clear_of_joint_and_edges(cup_middle_y: float, cup_middle_z: float) -> bool:
    """The rim JOINT_MARGIN or more from the joint and EDGE_MARGIN or more from its tile's outer edges."""
    joint, edges = rim_clearance(cup_middle_y, cup_middle_z)
    return joint >= JOINT_MARGIN and edges >= EDGE_MARGIN
