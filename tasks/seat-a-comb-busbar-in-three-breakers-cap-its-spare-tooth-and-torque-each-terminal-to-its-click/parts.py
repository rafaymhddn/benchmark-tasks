"""Single source of figures for the comb-busbar task: products, layout, the terminal's screw and clamp, the torque
driver, tolerances, colours, the key-figures table with each figure's source, and the shape builders.

Every dimension, mass, angle, torque, tolerance and bench coordinate that the layout, mechanism, scorer, demo, checks
or tests rest on is one constant here, its source in its comment (the maker's page, a standard, a listing, or an
estimate and how it was derived).

World frame (yam_arena): metres from the midpoint between the arm bases, x away from the arms, y left, z up from the
bench top (``task.py`` adds the bench height). The three breakers clip on a DIN rail screwed to our fixture, a steel
plate leaning back ``TILT_DEG`` (30 deg) so the terminal screws face the arms. Panel frame (``pw``): x_p into the
breakers from their front faces, y_p along the row (world y), z_p up the breakers from their feet. The comb's teeth go
down -z_p into the top terminals' busbar slots; each terminal's screw faces the arms along -x_p, 23 mm under the top.
The comb, its rack and the cap's holder lean the same 30 deg (the hand keeps the comb at that lean).
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from mechkit import box, cylinder, matrix_to_quat

OVER = 0.001  # m: loose bodies start this far off their support so frame 0 renders cleanly
CONTACT_THIN = 0.0005  # m: contact offset of the thin parts (teeth, clamp plates, cap)

# --- colours ---
WHITE = (0.93, 0.93, 0.91)
DARK = (0.18, 0.18, 0.19)
STEEL = (0.62, 0.64, 0.66)
COPPER = (0.80, 0.45, 0.25)
CAP_GREY = (0.55, 0.57, 0.60)
GREEN = (0.30, 0.62, 0.22)
WOOD = (0.55, 0.42, 0.30)
BLUE = (0.10, 0.35, 0.65)

# --- breakers: Schneider Electric Acti9 iC60N 1P 16 A C, A9F74116 (its data sheet SE1: "Discontinued on: 1 Nov 2020"; the
# A9F77116, "Statut commercial: Commercialisé" in SE4, has the same body ("Hauteur 85 mm", "Largeur 18 mm", "Profondeur 78,5 mm"),
# torque ("Couple de serrage 2 N.m haut ou bas") and comb fit ("Compatibilité des répartiteur, peigne, jeu de barres Haut ou bas : OUI")) ---
MCB_W = 0.018  # m: width (maker: "Width | 18 mm")
MCB_H = 0.085  # m: height (maker: "Height | 85 mm")
MCB_D = 0.0785  # m: depth, rail to front (maker: "Depth | 78.5 mm")
TORQUE_SET = 2.0  # N m: terminal torque (maker: "Tightening torque | 2 N.m top or bottom"), the limiter's preset
MCB_Y = (0.018, 0.0, -0.018)  # m: breakers 1-3 along the rail (y_p), side by side: 18 mm apart, the comb's pitch
MCB_TOP = MCB_H  # m: top faces (z_p), where the comb's bar comes to rest
TOGGLE = ((0.008, 0.008, 0.012), 0.036)  # m: toggle size and height on the front (estimate: drawn from the product photo)

# --- our fixture: the rail on a steel plate leaning back 30 deg ---
TILT_DEG = 30.0  # deg: the plate's lean from upright, so the screws face the arms
PANEL_ORIGIN = (0.255, 0.040, 0.040)  # m: world point of the middle breaker's front foot edge (the panel frame's origin)
RAIL = (0.0075, 0.090, 0.035)  # m: 35 x 7.5 mm rail, 90 mm long (WAGO 210-113: "35 x 7.5 mm; 1 mm thick ... according to EN 60715")
PLATE = (0.010, 0.100, 0.100)  # m: the fixture's plate, thick x wide x along the lean (our fixture: 10 mm steel, 100 x 100 mm)
FOOT_T = 0.004  # m: the fixture's foot plate on the bench (our fixture: 4 mm steel), its strut 6 mm


def rot_y(deg: float) -> np.ndarray:
    c, s_ = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0.0, s_], [0.0, 1.0, 0.0], [-s_, 0.0, c]])


TILT = rot_y(TILT_DEG)  # panel frame -> world
TILT_QUAT = matrix_to_quat(TILT)
XP, YP, ZP = TILT[:, 0], TILT[:, 1], TILT[:, 2]  # the panel's axes in the world: into the fronts, along the row, up the breakers


def pw(p) -> np.ndarray:
    """World point (z from the bench) of panel-frame point ``p``."""
    return np.asarray(PANEL_ORIGIN, float) + TILT @ np.asarray(p, float)


def to_panel(p) -> np.ndarray:
    """Panel-frame point of world point ``p`` (z from the bench)."""
    return TILT.T @ (np.asarray(p, float) - np.asarray(PANEL_ORIGIN, float))


# --- top terminal (estimates inside the maker's 18 x 85 x 78.5 mm housing; the screw an M5 x 0.8 by ISO 261) ---
X_FRONT, X_BACK = 0.0, MCB_D  # m: front face and rail side (x_p)
SCREW_Z = 0.062  # m: screw axis 23 mm under the top (z_p; estimate: the terminal's screw on the front, under the busbar slot)
HOLE = 0.009  # m: square access hole round the screw head (estimate: the 6.35 mm bit with 1.3 mm play a side)
HOLE_SINK = 0.002  # m: 45-deg countersink round the hole's mouth (estimate: a moulded lead-in; 13 mm across at the face)
HEAD_FACE = 0.006  # m: the screw head's face this deep behind the front face
HOLE_FLOOR = 0.009  # m: the hole's floor (the head's back), where the bit's tip comes to rest
PIN_T = 0.0015  # m: tooth thickness, front to back (estimate: a 1.5 mm copper strip)
PIN_W = 0.0105  # m: tooth width along the row (estimate: 1.5 x 10.5 mm, about 16 mm2 of copper for the comb's 100 A, SE2:
# "[Ie] rated operational current 100 A at 40 °C")
PIN_L = 0.026  # m: tooth length under the bar (estimate: from the top face to 3 mm past the screw axis)
X_PIN = 0.018  # m: the seated tooth's centre, 18 mm behind the front (estimate: the slot behind the screw)
X_WALL = X_PIN + 0.5 * PIN_T  # m: the slot's back wall, which the clamp presses the tooth on
SLOT_X = 0.0025  # m: busbar slot front to back over the clamp (tooth + 1.0 mm)
SLOT_Y = PIN_W + 0.0020  # m: busbar slot along the row (tooth + 2.0 mm)
MOUTH = 0.0016  # m: 45-deg chamfer on the slot mouth's four edges (estimate: a moulded lead-in, as the access hole's)
CLAMP_Z = (0.0540, 0.0700)  # m: the clamp plate's range up the breaker (z_p), round the screw axis
LEAD = 0.0008  # m per turn: M5 coarse pitch (ISO 261: M5 x 0.8); the plate moves this much per turn of its screw
GAP0 = PIN_T + 0.5 * LEAD  # 1.9 mm: the start, half a turn past a 1.5 mm tooth
JAW_T = 0.0020  # m: clamp plate thickness
JAW_POCKET = 0.0055  # m: pocket front to back: the plate backs out to a 3.5 mm gap (two turns past the start)
GAP_MAX = JAW_POCKET - JAW_T
CLOSE_GAP_MIN = 0.0002  # m: an empty clamp closes to this gap (its plate stops on the slot's wall)
TURN_MIN = -2.0 * math.pi * (GAP_MAX - GAP0) / LEAD  # rad: furthest the screw backs out from the start
CLAMP_DEG = 15.0  # deg: turn from the plate's touch on the tooth to the set torque (estimate: a stiff clamp on copper)
TORQUE_RUN = 0.10  # N m: running torque before the touch (estimate: thread friction of an M5 screw turning free)
CLAMP_STIFF = (TORQUE_SET - TORQUE_RUN) / math.radians(CLAMP_DEG)  # N m per rad past the touch (7.3)
TORQUE_LOOSEN = 0.8 * TORQUE_SET  # N m: breakaway of a set screw turned back (estimate: 0.8 of the tightening torque)
CLICK_SLIP_DEG = 3.0  # deg: the limiter slips this far at the set torque before its click counts (estimate)

# --- the comb busbar: Schneider Electric Acti9 A9XPH112 ("comb busbar - 1L - 18 mm pitch - 12 modules - 100A",
# "Device presentation | Cuttable"), a 4-module length ---
COMB_TEETH = 4
PITCH = 0.018  # m: tooth pitch (maker: "18 mm pitch")
COMB_L = COMB_TEETH * PITCH  # m: 72 mm, four modules of the maker's 220 mm, 12-module piece
COMB_BAR = (0.010, COMB_L, 0.016)  # m: insulated bar, front-back x along x height (estimate: the profile on the maker's photo)
COMB_MASS = 0.016  # kg (estimate: a third of the 12-module piece, whose package weighs 49 g by the maker)
TEETH_Y = tuple((k - 0.5 * (COMB_TEETH - 1)) * PITCH for k in range(COMB_TEETH))  # tooth positions along the bar
PIN_FRICTION = (0.30, 0.25)  # copper on the terminal (estimate: copper on brass and polyamide)
GRIPPY = (1.2, 1.0)  # yam_arena's default for what a gripper takes
COMB_WHITE = WHITE  # maker: "Colour | White (RAL 9003)"

# --- tooth cap: Schneider Electric Acti9 A9XPT920 ("set of 20 tooth caps for Acti9", "Comb busbar tooth cover") ---
CAP_SIZE = (PIN_T + 0.002, PIN_W + 0.002, 0.028)  # m: x, y, height (estimate: a sleeve over the 26 mm tooth, 1 mm walls, 2 mm end)
CAP_MASS = 0.002  # kg (estimate: 2 g of polyamide)
CAP_SNAP_DEPTH = 0.022  # m: tooth tip this far into the cap snaps it on (estimate: the cap's catch near its end)
CAP_SNAP_LATERAL = 0.0025  # m: tooth's centre line within this of the cap's when it snaps (estimate: the socket's lead-in takes it from there)
CAP_SNAP_ANGLE_DEG = 10.0

# --- torque driver: Wera 416 R T-handle bitholder (05023404001), Bernstein 4-985 torque adapter 2.0 N m, Wera 855/1 Z
# PZ 2 x 25 mm bit (05072082001) ---
TB_SPAN = 0.099  # m: T-handle across (maker: "Handle diameter mm 99")
TB_LONG = 0.044  # m: T-handle along the axis (maker: "Handle length mm 44")
TB_THICK = 0.022  # m: T-handle thickness the gripper pinches (maker's data sheet WERA3: "Size: 94x98x22 mm")
BLADE = (0.004, 0.050)  # m: radius (estimate), length to the chuck's end (maker: 94 mm overall less the 44 mm handle)
ADAPTER = (0.006, 0.0283)  # m: radius, length (maker: "Adapter Ø 12 mm", "Adapter length 28.3 mm (without shank)")
BIT = (0.0032, 0.015)  # m: 1/4-in hex bit, length standing out (estimate: 15 mm of the 25 mm bit out of the adapter)
TIP_X = 0.5 * TB_LONG + BLADE[1] + ADAPTER[1] + BIT[1]  # m: bit tip from the T-handle's centre (115.3 mm)
DRIVER_MASS = 0.108  # kg (makers: handle "Weight: 82 g" (WERA3), adapter "Weight 21 g", bit "Weight: 5g" (WERA2))
GRASP_OFF = (0.0, 0.0, -0.013)  # m: the left pads' centre on the T-handle (driver frame): 13 mm along its span off the axis,
# so the other arm stays free for the right hand's push, 20 mm and more from them
ENGAGE_RADIAL = 0.0020  # m: bit tip within this of the screw's axis (a tip in the 9 mm hole is within 1.3 mm)
ENGAGE_DEPTH = (-0.0010, 0.0045)  # m: bit tip from the head's face, out (-) to in (+), while it turns the screw
ENGAGE_ANGLE_DEG = 10.0  # deg: driver's axis within this of the screw's
SPIN_SCALE = 0.3  # rad/s: the screw's resisting torque on the driver reaches 76 % of its value at this turning speed (tanh)
BIT_PLAY_DEG = 4.0  # deg: the bit turns this far in the screw's cross recess before it drives the screw the other way (estimate)
TORQUE_LAG = 0.3  # the screw's torque on the driver moves this share of the way to its law's value each control step (about 0.15 s):
# applied in 20 Hz steps, a torque that flips with the driver's speed rattles a light driver in the hands (run cb2k)

# --- fixtures (our own; poplar) ---
RACK_XY = (0.350, 0.245)  # m: the comb's bar centre on its rack
RACK_Z = 0.040  # m: the rack's top under the bar (the teeth hang 18 mm clear of the bench)
CAP_XY = (0.290, 0.205)  # m: the cap's holder
CAP_Z = 0.034  # m: the holder's floor under the cap
CAP_PLAY = 0.0002  # m: the holder's pocket round the cap, a side
CAP_WALL = 0.005  # m: the pocket's walls: their top 23 mm down the cap, under where its catch takes a tooth (22 mm)
HOLSTER_G = (0.205, 0.100, 0.112)  # m: the driver's T-handle centre in its holster (the driver along x_p, 30 deg down)

# --- scoring ---
SEAT_DEPTH = 0.022  # m: a tooth counts as in its terminal from this deep under the top face (bar within 4 mm of the top)
SEAT_LATERAL = (0.0012, 0.0030)  # m: tooth centre within these of the slot centre, x_p and y_p
OFF_BENCH = 0.010  # m: a body's centre this far under the bench top has left the bench
GRIPPER_CLEAR = 0.030  # m: grippers at least this far from the comb at the end
REST_SPEED = 0.010  # m/s
REST_SPIN = 0.05  # rad/s: under 3 deg/s, so a driver still turning a screw slowly is not at rest
REST_HOLD_S = 1.0  # s


@dataclass(frozen=True)
class KeyFigure:
    figure: str
    value: str
    kind: str  # maker | standard | listing | estimate | maker / estimate | our fixture
    source: str


# the makers' data sheets (a product page shows no figures to a fetcher)
SE1 = "https://www.se.com/ae/en/product/download-pdf/A9F74116"
SE2 = "https://www.se.com/ae/en/product/download-pdf/A9XPH112"
SE3 = "https://www.se.com/ae/en/product/download-pdf/A9XPT920"
SE4 = "https://www.se.com/fr/fr/product/download-pdf/A9F77116"  # the A9F74116's twin, on sale in France
WERA1 = "https://www.wera.de/en/tools/416-r-t-handle-bitholding-screwdriver-rapidaptor"  # product page (handle table)
WERA2 = "https://hybris-media.wera.de/download/pdfgenerator-datasheets/en/05072082001.pdf"  # 855/1 Z PZ 2 x 25 mm
WERA3 = "https://hybris-media.wera.de/download/pdfgenerator-datasheets/en/05023404001.pdf"  # 416 R
KEY_FIGURES: tuple[KeyFigure, ...] = (  # the README's table, row for row
    KeyFigure("Breaker size", "85 x 18 x 78.5 mm", "maker", f"{SE1} 'Height 85 mm', 'Width 18 mm', 'Depth 78.5 mm'"),
    KeyFigure("Terminal torque (the adapter's preset)", "2.0 N m", "maker", f"{SE1} 'Tightening torque 2 N.m top or bottom'"),
    KeyFigure("Comb pitch, colour", "18 mm, white", "maker", f"{SE2} '18 mm pitch - 12 modules', 'Width 220 mm', 'Colour White (RAL 9003)', 'Cuttable'"),
    KeyFigure("Tooth cap", "grey, 3.5 x 12.5 x 28 mm, 2 g", "maker / estimate", f"{SE3} 'set of 20 tooth caps for Acti 9'; size, mass, colour: estimate (a 1 mm-walled sleeve)"),
    KeyFigure("Tooth section", "1.5 x 10.5 mm", "estimate", f"about 16 mm2 of copper for the comb's '100 A' ({SE2})"),
    KeyFigure("Comb mass", "16 g", "estimate", f"a third of the 12-module piece, whose package weighs 49 g ({SE2})"),
    KeyFigure("Screw lead; start", "0.8 mm a turn; half a turn past the tooth", "standard", "ISO 261, M5 x 0.8; the reset sets the start"),
    KeyFigure("Turn from touch to 2.0 N m", "15 deg", "estimate", "a stiff clamp on copper (the click is scored)"),
    KeyFigure("Running torque", "0.1 N m", "estimate", "an M5 screw turning free in its thread"),
    KeyFigure("Bit's play in the recess", "4 deg", "estimate", "the cross's clearance"),
    KeyFigure("T-handle", "99 x 44 x 22 mm, 82 g", "maker", f"{WERA1} 'Handle diameter mm 99', 'Handle length mm 44'; {WERA3} 'Size: 94x98x22 mm', 'Weight: 82 g'"),
    KeyFigure("Torque adapter", "2.0 N m +-10 %, 12 x 28.3 mm, 21 g", "maker", "https://www.bernstein-werkzeuge.de/en/products/productdetails/4-985-torque-adapter 'Tightening torque 2.0 Nm', 'Tolerance +/-10%', 'The click sound indicates when the torque is reached'"),
    KeyFigure("Bit", "PZ 2 x 25 mm, 5 g", "maker", f"{WERA2} '855/1 Z bits, PZ 2 x 25 mm', 'Weight: 5g'"),
    KeyFigure("Rail", "35 x 7.5 mm", "maker", "https://www.wago.com/global/accessories/steel-carrier-rail/p/210-113 '35 x 7.5 mm ... according to EN 60715'"),
    KeyFigure("Slot, hole, cap's catch", "2.5 x 12.5 mm slot, 1.6 mm lead-ins; 9 mm hole, 2 mm countersink; catch at 22 mm", "estimate", "from the tooth and the bit"),
    KeyFigure("Fixture, rack, holder, holster, 30-deg lean", "as in the README's Scene", "our fixture", "the lean so the screws face the arms"),
)


# --- shape helpers ---------------------------------------------------------------------------------------------------
def aabb(lo, hi, color=WHITE, solid=True, **kw):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return box(tuple(hi - lo), pos=tuple(0.5 * (lo + hi)), color=color, solid=solid, **kw)


def leaning(lo, hi, origin, color=WOOD, **kw):
    """A box given in a frame at ``origin`` (world, from the bench) with the panel's lean, for a world-frame body."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    centre = np.asarray(origin, float) + TILT @ (0.5 * (lo + hi))
    return box(tuple(hi - lo), pos=tuple(centre), quat=TILT_QUAT, color=color, **kw)


def chamfer(p_a, p_b, edge_axis, inside, length, color=WHITE, depth=None, **kw):
    """A box whose face is the plane through the edge lines at ``p_a`` and ``p_b`` (both along ``edge_axis``), lying on
    the side of ``inside`` (a point in the solid): a 45-deg mouth on a slot's edge. ``depth``: how far it reaches into
    the solid (default: as far as the face is wide)."""
    p_a, p_b, e = np.asarray(p_a, float), np.asarray(p_b, float), np.asarray(edge_axis, float)
    u = (p_b - p_a) / np.linalg.norm(p_b - p_a)
    n = np.cross(u, e)
    mid = 0.5 * (p_a + p_b)
    if np.dot(n, np.asarray(inside, float) - mid) < 0:
        e, n = -e, -n
    h = 0.5 * np.linalg.norm(p_b - p_a)
    d = 2.0 * h if depth is None else float(depth)
    rot = np.column_stack([u, e, n])
    return box((2.0 * h, length, d), pos=tuple(mid + 0.5 * d * n), quat=matrix_to_quat(rot), color=color, **kw)


def breaker_shapes(yc: float) -> list:
    """One breaker's housing in the panel frame: the lower body, the toggle, and the top terminal round its access hole
    (countersunk mouth), its busbar slot (chamfered mouth) and its clamp plate's pocket."""
    hy, top, z0, z1 = 0.5 * MCB_W, MCB_TOP, CLAMP_Z[0], CLAMP_Z[1]
    zh0, zh1 = SCREW_Z - 0.5 * HOLE, SCREW_Z + 0.5 * HOLE
    sx0, sy, jx0, f, c = X_WALL - SLOT_X, 0.5 * SLOT_Y, X_WALL - JAW_POCKET, X_FRONT, HOLE_SINK
    return [
        aabb((f, yc - hy, 0.0), (X_BACK, yc + hy, z0)),  # lower body
        aabb((f - TOGGLE[0][0], yc - 0.5 * TOGGLE[0][1], TOGGLE[1] - 0.5 * TOGGLE[0][2]), (f, yc + 0.5 * TOGGLE[0][1], TOGGLE[1] + 0.5 * TOGGLE[0][2]), color=DARK),
        aabb((f + c, yc - hy, z0), (f + HOLE_FLOOR, yc + hy, zh0)),  # front slab round the access hole, behind its countersink
        aabb((f + c, yc - hy, zh1), (f + HOLE_FLOOR, yc + hy, top)),
        aabb((f + c, yc - hy, zh0), (f + HOLE_FLOOR, yc - 0.5 * HOLE, zh1)),
        aabb((f + c, yc + 0.5 * HOLE, zh0), (f + HOLE_FLOOR, yc + hy, zh1)),
        aabb((f, yc - hy, z0), (f + c, yc + hy, zh0 - c)),  # in front: set back round the countersink's mouth
        aabb((f, yc - hy, zh1 + c), (f + c, yc + hy, top)),
        aabb((f, yc - hy, zh0 - c), (f + c, yc - 0.5 * HOLE - c, zh1 + c)),
        aabb((f, yc + 0.5 * HOLE + c, zh0 - c), (f + c, yc + hy, zh1 + c)),
        chamfer((f, yc, zh0 - c), (f + c, yc, zh0), (0, 1, 0), (f + c, yc, zh0 - c), HOLE + 2 * c),  # the countersink's four faces
        chamfer((f, yc, zh1 + c), (f + c, yc, zh1), (0, 1, 0), (f + c, yc, zh1 + c), HOLE + 2 * c),
        chamfer((f, yc - 0.5 * HOLE - c, SCREW_Z), (f + c, yc - 0.5 * HOLE, SCREW_Z), (0, 0, 1), (f + c, yc - 0.5 * HOLE - c, SCREW_Z), HOLE + 2 * c),
        chamfer((f, yc + 0.5 * HOLE + c, SCREW_Z), (f + c, yc + 0.5 * HOLE, SCREW_Z), (0, 0, 1), (f + c, yc + 0.5 * HOLE + c, SCREW_Z), HOLE + 2 * c),
        aabb((f + HOLE_FLOOR - 0.0005, yc - 0.5 * HOLE, zh0), (f + HOLE_FLOOR, yc + 0.5 * HOLE, zh1), color=DARK),  # hole floor
        aabb((f + HOLE_FLOOR, yc - hy, z0), (jx0, yc + hy, z1)),  # in front of the clamp pocket
        aabb((f + HOLE_FLOOR, yc - hy, z1), (sx0, yc + hy, top - MOUTH)),  # the slot's front wall
        aabb((f + HOLE_FLOOR, yc - hy, top - MOUTH), (sx0 - MOUTH, yc + hy, top)),
        aabb((jx0, yc - hy, z1 - 0.0004), (sx0, yc + hy, z1)),  # the pocket's roof in front of the slot
        aabb((jx0, yc - hy, z0), (X_WALL, yc - sy, top - MOUTH)),  # slot side walls (the pocket's too)
        aabb((jx0, yc + sy, z0), (X_WALL, yc + hy, top - MOUTH)),
        aabb((jx0, yc - hy, top - MOUTH), (X_WALL, yc - sy - MOUTH, top)),
        aabb((jx0, yc + sy + MOUTH, top - MOUTH), (X_WALL, yc + hy, top)),
        aabb((X_WALL, yc - hy, z0), (X_BACK, yc + hy, top - MOUTH)),  # behind the slot: the wall the tooth is clamped on
        aabb((X_WALL + MOUTH, yc - hy, top - MOUTH), (X_BACK, yc + hy, top)),
        chamfer((sx0 - MOUTH, yc, top), (sx0, yc, top - MOUTH), (0, 1, 0), (sx0 - MOUTH, yc, top - MOUTH), SLOT_Y + 2 * MOUTH, depth=0.001),
        chamfer((X_WALL + MOUTH, yc, top), (X_WALL, yc, top - MOUTH), (0, 1, 0), (X_WALL + MOUTH, yc, top - MOUTH), SLOT_Y + 2 * MOUTH, depth=0.001),
        chamfer((X_WALL - 0.5 * SLOT_X, yc - sy - MOUTH, top), (X_WALL - 0.5 * SLOT_X, yc - sy, top - MOUTH), (1, 0, 0),
                (X_WALL - 0.5 * SLOT_X, yc - sy - MOUTH, top - MOUTH), SLOT_X + 2 * MOUTH, depth=0.001),
        chamfer((X_WALL - 0.5 * SLOT_X, yc + sy + MOUTH, top), (X_WALL - 0.5 * SLOT_X, yc + sy, top - MOUTH), (1, 0, 0),
                (X_WALL - 0.5 * SLOT_X, yc + sy + MOUTH, top - MOUTH), SLOT_X + 2 * MOUTH, depth=0.001),
    ]


def plate_z0() -> float:
    """z_p of the leaning plate's lower edge: its back edge stands on the foot plate."""
    x_back = X_BACK + RAIL[0] + PLATE[0]
    return (FOOT_T - PANEL_ORIGIN[2] + math.sin(math.radians(TILT_DEG)) * x_back) / math.cos(math.radians(TILT_DEG))


def panel_shapes() -> list:
    """The panel in its frame: the rail, our fixture's leaning plate (its back edge on the foot plate), three breakers."""
    rail_z = 0.5 * MCB_H
    x_plate, z0 = X_BACK + RAIL[0], plate_z0()
    s = [
        aabb((X_BACK, -0.5 * RAIL[1], rail_z - 0.5 * RAIL[2]), (x_plate, 0.5 * RAIL[1], rail_z + 0.5 * RAIL[2]), color=STEEL),
        aabb((x_plate, -0.5 * PLATE[1], z0), (x_plate + PLATE[0], 0.5 * PLATE[1], z0 + PLATE[2]), color=STEEL),
    ]
    for yc in MCB_Y:
        s += breaker_shapes(yc)
    return s


def fixture_foot_shapes() -> list:
    """World-frame parts of our fixture: a foot plate on the bench under the leaning plate and a strut behind it."""
    z0 = plate_z0()
    back = pw((X_BACK + RAIL[0] + PLATE[0], 0.0, z0))
    top = pw((X_BACK + RAIL[0] + PLATE[0], 0.0, z0 + PLATE[2]))
    front = pw((X_BACK + RAIL[0], 0.0, z0))
    y0, hw = PANEL_ORIGIN[1], 0.5 * PLATE[1]
    xm = back[0] + 0.6 * (top[0] - back[0])
    zm = back[2] + 0.6 * (top[2] - back[2])
    return [
        aabb((front[0] - 0.008, y0 - hw, 0.0), (top[0] + 0.012, y0 + hw, FOOT_T), color=STEEL),
        aabb((xm, y0 - hw, FOOT_T), (xm + 0.006, y0 + hw, zm), color=STEEL),
    ]


def jaw_shapes() -> list:
    """A clamp plate in its body frame (axes as the panel's, origin at its clamping face's centre; +x faces the tooth),
    with a 45-deg lead on its top edge so a tooth sliding down the slot's front wall is led onto the clamping wall."""
    z0, z1, zc = CLAMP_Z[0] + 0.0001, CLAMP_Z[1] - 0.0004, 0.5 * (CLAMP_Z[0] + CLAMP_Z[1])
    lead, w = SLOT_X - GAP0, SLOT_Y - 0.0002  # lead: 0.6 mm, the step from the slot's front wall to the open plate's face
    return [
        aabb((-JAW_T, -0.5 * w, z0 - zc), (0.0, 0.5 * w, z1 - lead - zc), color=STEEL),
        chamfer((-lead, 0.0, z1 - zc), (0.0, 0.0, z1 - lead - zc), (0, 1, 0), (-0.0015, 0.0, z1 - lead - 0.001 - zc), w, color=STEEL),
        aabb((-JAW_T, -0.5 * w, z1 - lead - zc), (-lead, 0.5 * w, z1 - zc), color=STEEL),
    ]


def jaw_pos(i: int, gap: float) -> np.ndarray:
    return np.array([X_WALL - gap, MCB_Y[i], 0.5 * (CLAMP_Z[0] + CLAMP_Z[1])])


def screw_shapes() -> list:
    """The screw's head as drawn (it touches nothing: the bit's tip rests on the hole's floor behind it), with its cross;
    body frame at the head's face, x along the screw."""
    t = HOLE_FLOOR - HEAD_FACE
    return [
        cylinder(0.0042, t, pos=(0.5 * t, 0.0, 0.0), axis="x", color=STEEL, solid=False),
        box((0.0004, 0.0060, 0.0012), pos=(-0.0001, 0.0, 0.0), color=DARK, solid=False),
        box((0.0004, 0.0012, 0.0060), pos=(-0.0001, 0.0, 0.0), color=DARK, solid=False),
    ]


def screw_pos(i: int) -> np.ndarray:
    return np.array([X_FRONT + HEAD_FACE, MCB_Y[i], SCREW_Z])


def comb_shapes() -> list:
    """Body frame at the bar's centre: the insulated bar and four copper teeth hanging under it."""
    bx, _, bz = COMB_BAR
    s = [box(COMB_BAR, color=COMB_WHITE, friction=GRIPPY)]
    for y in TEETH_Y:
        s.append(box((PIN_T, PIN_W, PIN_L), pos=(0.0, y, -0.5 * bz - 0.5 * PIN_L), color=COPPER, friction=PIN_FRICTION, friction_combine="min"))
    s.append(box((0.4 * bx, COMB_L - 0.004, 0.0008), pos=(0.0, 0.0, 0.5 * bz + 0.0004), color=COPPER, solid=False))  # copper seen on top
    return s


def tooth_tip_local(k: int) -> np.ndarray:
    return np.array([0.0, TEETH_Y[k], -0.5 * COMB_BAR[2] - PIN_L])


def comb_seated_panel(first: int = 0) -> np.ndarray:
    """Panel-frame centre of the comb with teeth ``first..first+2`` in breakers 3, 2, 1, its bar on the tops."""
    return np.array([X_PIN, MCB_Y[2] - TEETH_Y[first], MCB_TOP + 0.5 * COMB_BAR[2]])


def cap_shapes() -> list:
    return [box(CAP_SIZE, color=CAP_GREY, passes=("comb",), friction=GRIPPY)]


def rack_origin() -> np.ndarray:
    return np.array([RACK_XY[0], RACK_XY[1], RACK_Z])


def cap_origin() -> np.ndarray:
    return np.array([CAP_XY[0], CAP_XY[1], CAP_Z])


def stand_shapes() -> list:
    """World-frame fixtures (our own, poplar): the comb's rack (two posts between its teeth, a lip on the downhill side)
    and the cap's holder (a post with a shallow pocket), both at the panel's lean, and our fixture's foot."""
    r0, c0 = rack_origin(), cap_origin()
    s = []
    for dy in (-PITCH, PITCH):  # between teeth 1-2 and 3-4
        s.append(leaning((-0.005, dy - 0.002, -0.050), (0.005, dy + 0.002, 0.0), r0))
        s.append(leaning((0.005, dy - 0.002, 0.0), (0.007, dy + 0.002, 0.004), r0))
    hx, hy, w, h = 0.5 * CAP_SIZE[0] + CAP_PLAY, 0.5 * CAP_SIZE[1] + CAP_PLAY, 0.0015, CAP_WALL
    s += [
        leaning((-hx - w, -hy - w, -0.045), (hx + w, hy + w, 0.0), c0),
        leaning((-hx - w, -hy - w, 0.0), (-hx, hy + w, h), c0),
        leaning((hx, -hy - w, 0.0), (hx + w, hy + w, h), c0),
        leaning((-hx, -hy - w, 0.0), (hx, -hy, h), c0),
        leaning((-hx, hy, 0.0), (hx, hy + w, h), c0),
    ]
    return s + fixture_foot_shapes()


def driver_shapes() -> list:
    """Body frame at the T-handle's centre: x along the driver (bit at +x), y its thickness, z its span."""
    x0 = 0.5 * TB_LONG
    return [
        box((TB_LONG, TB_THICK, TB_SPAN - 0.030), color=DARK, friction=GRIPPY),
        box((TB_LONG, TB_THICK, 0.015), pos=(0.0, 0.0, 0.5 * TB_SPAN - 0.0075), color=GREEN, friction=GRIPPY),
        box((TB_LONG, TB_THICK, 0.015), pos=(0.0, 0.0, -0.5 * TB_SPAN + 0.0075), color=GREEN, friction=GRIPPY),
        cylinder(BLADE[0], BLADE[1], pos=(x0 + 0.5 * BLADE[1], 0.0, 0.0), axis="x", color=STEEL),
        cylinder(ADAPTER[0], ADAPTER[1], pos=(x0 + BLADE[1] + 0.5 * ADAPTER[1], 0.0, 0.0), axis="x", color=BLUE),
        cylinder(BIT[0], BIT[1], pos=(x0 + BLADE[1] + ADAPTER[1] + 0.5 * BIT[1], 0.0, 0.0), axis="x", color=STEEL),
    ]


def driver_rot(roll: float = 0.0) -> np.ndarray:
    """The driver's rotation with its axis along x_p (30 deg down, towards the breakers), turned ``roll`` about it from
    the T-handle's span along z_p (roll + : tightening)."""
    c, s_ = math.cos(roll), math.sin(roll)
    return TILT @ np.array([[1.0, 0.0, 0.0], [0.0, c, -s_], [0.0, s_, c]])


def holster_shapes() -> list:
    """World-frame holster (our own, poplar): a square tube round the adapter and a stop under the bit's tip, at the
    driver's 30-deg lean, on two legs."""
    g = np.asarray(HOLSTER_G, float)
    ax0, ax1 = 0.5 * TB_LONG + BLADE[1] + 0.004, 0.5 * TB_LONG + BLADE[1] + ADAPTER[1] - 0.004
    a, w = ADAPTER[0] + 0.0008, 0.003
    s = [
        leaning((ax0, -a - w, -a - w), (ax1, a + w, -a), g),
        leaning((ax0, -a - w, a), (ax1, a + w, a + w), g),
        leaning((ax0, -a - w, -a), (ax1, -a, a), g),
        leaning((ax0, a, -a), (ax1, a + w, a), g),
        leaning((TIP_X + 0.0005, -0.010, -0.010), (TIP_X + 0.0045, 0.010, 0.010), g),
    ]
    for p in (g + TILT @ np.array([0.5 * (ax0 + ax1), 0.0, -a - w]), g + TILT @ np.array([TIP_X + 0.0025, 0.0, -0.010])):
        s.append(aabb((p[0] - 0.004, p[1] - 0.010, 0.0), (p[0] + 0.004, p[1] + 0.010, p[2] + 0.002), color=WOOD))
    return s


HOLSTER_ROLL = 1.5 * math.pi  # the T-handle's span lies level in the holster, its 22 mm across z_p: the hand's pinch on it then
# carries the driver's weight as a form lock, not by friction about the pinch's axis (run cb1c: pinched across y, it drooped)


def driver_start() -> tuple[np.ndarray, np.ndarray]:
    """The driver resting in its holster: the adapter on the tube's floor, the tip on its stop."""
    g = np.asarray(HOLSTER_G, float) + TILT @ np.array([-0.0002, 0.0, -0.0008])
    return g, driver_rot(HOLSTER_ROLL)


# --- the screw's law -------------------------------------------------------------------------------------------------
def touch_turn(tooth_in: bool) -> float:
    """rad of tightening from the start at which the plate meets the tooth (or, empty, the slot's wall)."""
    return 2.0 * math.pi * (GAP0 - (PIN_T if tooth_in else CLOSE_GAP_MIN)) / LEAD


def clamp_gap(turn: float, tooth_in: bool) -> float:
    """Gap between the clamp plate and the slot's back wall after ``turn`` rad of tightening from the start (negative:
    backed out). With a tooth in the slot the plate stops on it (``PIN_T``)."""
    gap = GAP0 - LEAD * turn / (2.0 * math.pi)
    return min(max(gap, PIN_T if tooth_in else CLOSE_GAP_MIN), GAP_MAX)


def screw_torque(turn: float, tooth_in: bool) -> float:
    """N m the screw needs to turn on in the tightening direction: the running torque until the plate touches the tooth
    (or bottoms on the wall), then rising with the clamp's stiffness."""
    past = turn - touch_turn(tooth_in)
    return TORQUE_RUN if past <= 0.0 else TORQUE_RUN + CLAMP_STIFF * past


def click_turn(tooth_in: bool = True) -> float:
    """rad: the tightening turn at which the clamp reaches the set torque (where the limiter slips and clicks)."""
    return touch_turn(tooth_in) + (TORQUE_SET - TORQUE_RUN) / CLAMP_STIFF


def rot_x(angle: float) -> np.ndarray:
    c, s_ = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s_], [0.0, s_, c]])


def roll_about(r_prev: np.ndarray, r_now: np.ndarray, axis) -> float:
    """rad: the turn from ``r_prev`` to ``r_now`` about the world ``axis`` (the rotation vector's component along it)."""
    d = r_now @ r_prev.T
    t = math.acos(float(np.clip(0.5 * (np.trace(d) - 1.0), -1.0, 1.0)))
    if t < 1e-9 or t > math.pi - 1e-6:
        return 0.0
    v = np.array([d[2, 1] - d[1, 2], d[0, 2] - d[2, 0], d[1, 0] - d[0, 1]]) / (2.0 * math.sin(t))
    return float(t * np.dot(v, np.asarray(axis, float)))
