"""The bodies of the flip-socket task, built from parts.py's figures: the silicone mat, two open V cradles, two
parking stands, the bracket with its 3/8-inch hex standoff (all fixed), the driver
(handle and 1/4-inch hex shaft as one free body) and its three reversible socket barrels (free, riding the shaft by
bore contact alone: no joint, no weld, no latch in this task)."""

from __future__ import annotations

import math

import numpy as np

import mechkit as mk
import parts as P
from mechkit import body, box, capsule, cylinder

MECHKIT_HELPER = True  # declares for the task file that imports it (the kit skips this module's frames)

MAT_GREY = (0.84, 0.84, 0.80)  # light silicone mat, so the black barrels and their colour bands read from above
AMBER = (0.93, 0.66, 0.18)  # Klein's cushion-grip amber
CHARCOAL = (0.24, 0.25, 0.27)  # the manganese-phosphate finish
STEEL = (0.60, 0.62, 0.66)  # bright steel: the standoff, the shaft's tip
ALU = (0.66, 0.67, 0.70)  # the cradles and the tray
BARREL_GREY = (0.20, 0.20, 0.22)
START_SPANS = ((0.0000, 0.0285), (0.0325, 0.0625), (0.0690, 0.1000))  # each barrel along the shaft from the tip (m)


def _floats(v) -> tuple[float, ...]:
    return tuple(float(x) for x in v)


def _boxes(specs, at, color, solid=True, friction=None, combine=None, turn=None) -> list:
    """Boxes of (size, centre, rotation) specs placed at ``at``, all turned by ``turn`` first (about ``at``)."""
    out = []
    m = np.eye(3) if turn is None else np.asarray(turn, dtype=float)
    for dims, local, rot in specs:
        pos = np.asarray(at) + m @ np.asarray(local)
        out.append(box(dims, pos=_floats(pos), quat=_floats(mk.matrix_to_quat(m @ np.asarray(rot))), color=color, solid=solid, friction=friction,
                       friction_combine=combine))
    return out


def rot_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# the driver starts with its tip towards the bracket: its local x (inboard, handle-wards) points +y in the world
DRIVER_START_ROT = rot_z(math.pi / 2)
BARREL_START_ROT = rot_z(-math.pi / 2)  # a barrel's local +x (its first-named end) points -y: outboard, at the tip
CLOCK0 = 0.0  # every hex built level (flats up): the standoff alone is twisted, so the tool must be clocked


def build_mat(top: float) -> None:
    body("mat", [box(P.MAT, color=MAT_GREY)], pos=(P.MAT_AT[0], P.MAT_AT[1], top + P.MAT[2] / 2), color=MAT_GREY, friction=(0.70, 0.55))


def _v_walls(at, along_y: bool, vertex_z: float, length: float, rise: float, thickness: float, color, tilt_deg: float = 60.0) -> list:
    """Two slabs leaning out at ``tilt_deg`` from level, from a vertex line: an open V cradle (along the world y at
    ``at``'s x, or along x at ``at``'s y), its vertex (the V's bottom line) at ``vertex_z``. At 60 degrees the two
    walls carry a corner-down hexagonal grip on its faces."""
    a = math.radians(tilt_deg)
    out = []
    for sign in (1.0, -1.0):
        rot = rot_y(-sign * a) if along_y else P.rot_x(sign * a)
        face_mid = np.array([sign * rise * math.cos(a) / 2, 0.0, rise * math.sin(a) / 2]) if along_y else np.array([0.0, sign * rise * math.cos(a) / 2, rise * math.sin(a) / 2])
        normal = np.array([sign * math.sin(a), 0.0, -math.cos(a)]) if along_y else np.array([0.0, sign * math.sin(a), -math.cos(a)])
        centre = np.array([at[0], at[1], vertex_z]) + face_mid + normal * thickness / 2
        slant = rise + thickness  # lapped down past the vertex, so the two slabs meet solid
        dims = (slant, length, thickness) if along_y else (length, slant, thickness)
        out.append(box(dims, pos=_floats(centre), quat=_floats(mk.matrix_to_quat(rot)), color=color))
    return out


def build_cradles(top: float) -> None:
    """Two open V cradles on the mat, their V bottom cut so the driver's handle centre rides at AXIS_Z: the start
    cradle under the handle at the start, the end cradle beside the bracket under the seated handle. The shaft floats
    above each cradle's open V; nothing stops the tool sliding along its axis but the fit and the fingers."""
    mat_top = top + P.MAT[2]
    for name, y, vertex_h, length in (("cradle_start", P.HANDLE_MID, P.CRADLE_VERTEX, P.CRADLE_L), ("cradle_end", P.CRADLE_END_Y, P.END_VERTEX, P.END_CRADLE_L)):
        vertex = top + vertex_h
        base_h = vertex - mat_top
        shapes = [box((0.050, length, base_h), pos=(P.DRIVER_X, y, mat_top + base_h / 2), color=ALU)]
        shapes += _v_walls((P.DRIVER_X, y), True, vertex, length, P.CRADLE_RISE, 0.006, ALU, tilt_deg=30.0)
        body(name, shapes, pos=(0.0, 0.0, 0.0), color=ALU, friction=(P.CRADLE_FRICTION, 0.9))


def build_stands(top: float) -> None:
    """Two parking stands, one for the red/yellow and one for the green/blue barrel, each holding a barrel's centre at
    the shaft's height: two thin rails 24 mm apart that a barrel lies across with its axis along y. A hexagonal barrel
    lands flat on a face and cannot roll; nothing stands under its middle, so the fingers reach past it."""
    mat_top = top + P.MAT[2]
    shapes = []
    for b in P.BARRELS[:2]:
        x, y = P.STAND_X, P.STAND_Y[b.key]
        rail_h = top + P.AXIS_Z - b.od / 2.0 - mat_top
        shapes.append(box((0.036, 0.034, 0.003), pos=(x, y, mat_top + 0.0015), color=ALU))
        for sign in (1.0, -1.0):
            shapes.append(box((P.STAND_RAIL[0], P.STAND_RAIL[1], rail_h), pos=(x, y + sign * P.STAND_GAP / 2.0, mat_top + rail_h / 2.0), color=ALU))
    body("stands", shapes, pos=(0.0, 0.0, 0.0), color=ALU, friction=(0.35, 0.28))


def build_bracket(top: float) -> None:
    """An L-bracket on the mat: a base plate and a vertical face, carrying the 3/8-inch hex standoff threaded
    through it, 7.5 mm of hex standing out at the shaft axis height, its flats turned 15 degrees off level."""
    mat_top = top + P.MAT[2]
    shapes = [box((0.090, 0.070, 0.006), pos=(P.BRACKET_AT[0], P.BRACKET_AT[1], mat_top + 0.003), color=CHARCOAL),
              box((0.090, 0.006, 0.053), pos=(P.BRACKET_AT[0], P.WALL_FACE - 0.003, mat_top + 0.006 + 0.0265), color=CHARCOAL)]
    back = P.WALL_FACE - (P.POST_L - P.POST_OUT)  # the hex's far end, threaded into the bracket
    hex_end = P.POST_FACE_Y - P.POST_CH
    post_mu = (P.POST_FRICTION, P.POST_FRICTION * 0.85)
    shapes += _boxes(_hex_y(P.POST_AF, hex_end - back, math.radians(P.POST_TWIST)), (P.DRIVER_X, (hex_end + back) / 2, top + P.POST_Z), STEEL,
                     friction=post_mu, combine="min")
    to_world = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # the lead-in's x (narrow end first) along -y
    shapes += _boxes(P.hex_frustum(P.POST_CH_AF, P.POST_AF, P.POST_CH, twist=-math.radians(P.POST_TWIST)), (P.DRIVER_X, P.POST_FACE_Y, top + P.POST_Z), STEEL,
                     friction=post_mu, combine="min", turn=to_world)
    shapes += _boxes(_hex_y(P.POST_AF, 0.0032, math.radians(P.POST_TWIST)), (P.DRIVER_X, P.WALL_FACE - 0.0046, top + P.POST_Z), STEEL, solid=False)
    body("bracket", shapes, pos=(0.0, 0.0, 0.0), color=CHARCOAL, friction=(0.40, 0.32))


def _hex_y(af: float, length: float, twist: float) -> list:
    """A hexagonal prism along y of ``af`` across flats, as three boxes (parts.py's hex_prism turned on its side)."""
    return [((af / math.sqrt(3.0), length, af), (0.0, 0.0, 0.0), rot_y(twist + k * math.pi / 3.0)) for k in range(3)]


def build_driver(top: float) -> None:
    """The driver: cushion grip, steel collar and the 101 mm exposed 1/4-inch hex shaft, one free body in the tip
    frame (local x inboard, its origin on the shaft tip), its mass and balance from parts.py."""
    shapes = []
    slide = (P.SHAFT_SLIDE, P.SHAFT_SLIDE)  # the shaft's surface: every socket bore slides on it at this (min-combined)
    shapes += _boxes(P.hex_prism(P.SHAFT_AF, P.EXPOSED - P.TIP_CH, CLOCK0), ((P.EXPOSED + P.TIP_CH) / 2, 0, 0), CHARCOAL, friction=slide, combine="min")
    shapes += _boxes(P.hex_frustum(P.TIP_CH_AF, P.SHAFT_AF, P.TIP_CH, twist=CLOCK0), (0, 0, 0), CHARCOAL, friction=slide, combine="min")  # the tip's lead-in
    shapes += _boxes(P.hex_prism(0.0052, 0.005, CLOCK0), (-0.0025, 0, 0), STEEL, solid=False)  # drawn only: the bit-holder end takes no part in the task's mechanics, and a drawn tip lets the sockets pass it cleanly
    shapes.append(cylinder(0.0022, 0.0016, pos=(-0.0058, 0, 0), axis="x", color=STEEL, solid=False))
    shapes.append(cylinder(0.008, 0.006, pos=(0.098, 0, 0), axis="x", color=STEEL, solid=False))  # the nose ring, drawn only: the grip's flat nose is the barrels' rear stop
    shapes += _boxes(P.hex_prism(P.HANDLE_D, P.HANDLE_L, CLOCK0 + math.pi / 6), (P.EXPOSED + P.HANDLE_L / 2, 0, 0), AMBER, friction=(P.GRIP_FRICTION, 0.8))  # the fluted hex cushion grip, corner down in its cradle
    shapes.append(capsule(0.0142, 0.010, pos=(P.EXPOSED + P.HANDLE_L - 0.004, 0, 0), axis="x", color=AMBER, solid=False))
    for band in (0.112, 0.122):  # the grip's black overmould rings (drawn only)
        shapes.append(cylinder(0.0192, 0.008, pos=(P.EXPOSED + band, 0, 0), axis="x", color=(0.10, 0.10, 0.11), solid=False))
    mass, com, inertia = P.driver_props()
    body("driver", shapes, pos=_floats((P.DRIVER_X, P.TIP_Y, top + P.AXIS_Z)), quat=_floats(mk.matrix_to_quat(DRIVER_START_ROT)),
         mass=mass, color=AMBER, center=_floats((com, 0.0, 0.0)), inertia=diagonal(inertia), friction=(P.GRIP_FRICTION, 0.8))  # fmt: skip


def diagonal(inertia) -> tuple[float, float, float]:
    """The kit's inertia argument is the diagonal of the body's tensor in its own frame."""
    a, b, c = (float(v) for v in inertia)
    return a, b, c


def build_barrels(top: float) -> None:
    """Each barrel as one free body in its own frame (local x along its bore, +x towards its first-named end): a
    continuous hexagonal outside at its across-flats, per-end socket bores with lead-in chamfers, and the through-bore
    that rides the shaft. Drawn-only colour bands mark each end's size, as on the real tool."""
    for barrel, span in zip(P.BARRELS, START_SPANS):
        mid = P.TIP_Y + (span[0] + span[1]) / 2
        shapes = []
        end_a, end_b = barrel.ends  # end a at +length/2 (first-named), end b at -length/2
        a_bore = barrel.bores.get(end_a)
        b_bore = barrel.bores.get(end_b)
        half, od2 = barrel.length / 2, barrel.od / 2
        socket_spans = []
        if a_bore:
            af, depth = a_bore
            socket_spans += [(half - depth, half - P.CHAMFER, af), (half - P.CHAMFER, half, af + P.CHAMFER_WIDE)]  # the socket and its stepped mouth
        if b_bore:
            af, depth = b_bore
            socket_spans += [(-half + P.CHAMFER, -half + depth, af), (-half, -half + P.CHAMFER, af + P.CHAMFER_WIDE)]
        through_lo, through_hi = barrel.through
        through_spans = [(through_lo + P.MOUTH, through_hi - P.MOUTH, P.BORE_AF)]
        for edge in ((through_lo, -1.0), (through_hi, 1.0)):  # the through-bore's mouths, opened for threading
            through_spans.append((min(edge[0], edge[0] - edge[1] * P.MOUTH), max(edge[0], edge[0] - edge[1] * P.MOUTH), P.BORE_AF + P.CHAMFER_WIDE))
        for lo, hi, af in socket_spans:  # the socket ends press onto the standoff: the kit's graspable pair
            if hi - lo <= 0.0:
                continue
            shapes += _boxes(P.walls(6, af / 2, (lo, hi), od2 - af / 2, CLOCK0), (0, 0, 0), BARREL_GREY, friction=(P.BORE_FRICTION, 1.0))
        liner = 0.0005
        for lo, hi, af in through_spans:  # the bore that rides the shaft: a low-friction liner inside, the graspable finish outside
            if hi - lo <= 0.0:
                continue
            shapes += _boxes(P.walls(6, af / 2, (lo, hi), liner, CLOCK0), (0, 0, 0), BARREL_GREY, friction=(P.SHAFT_SLIDE, P.SHAFT_SLIDE))
            shapes += _boxes(P.walls(6, af / 2 + liner, (lo, hi), od2 - af / 2 - liner, CLOCK0), (0, 0, 0), BARREL_GREY)
        for end, colour in ((+1, barrel.ends[0]), (-1, barrel.ends[1])):
            shapes += _boxes(P.hex_prism(barrel.od + 0.0006, 0.003, CLOCK0), (end * (half - 0.0025), 0, 0), P.BANDS[colour], solid=False)
        body(barrel.key, shapes, pos=_floats((P.DRIVER_X, mid, top + P.AXIS_Z)), quat=_floats(mk.matrix_to_quat(BARREL_START_ROT)),
             mass=barrel.mass, color=BARREL_GREY, friction=(1.2, 1.0))  # fmt: skip


def build(top: float) -> None:
    build_mat(top)
    build_cradles(top)
    build_stands(top)
    build_bracket(top)
    build_driver(top)
    build_barrels(top)
