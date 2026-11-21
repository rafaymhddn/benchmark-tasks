"""The pill crusher's scene: every size, mass, pitch and torque (one constant each, its source beside it: the maker's
page or a listing in its own words, or an estimate and how it was derived; README), the bodies'
shapes, the scene's bodies, the crusher's hidden bodies and their joints, and where things are. The rules and the score
are in task.py."""

from __future__ import annotations

import json
import math
import pathlib

import numpy as np

import mechkit as mk
from mechkit import body, box, cylinder, ellipsoid, hinge, sense_contact, slide, sphere, weld
from yam_arena import station
from yam_arena.task_file import TABLE_TOP

MECHKIT_HELPER = True  # it declares the scene for task.py (declare()): mechkit counts that as task.py's
HERE = pathlib.Path(__file__).resolve().parent
SCENE = "station"
TOP = TABLE_TOP[SCENE]
GRAVITY = 9.81
Z = np.array([0.0, 0.0, 1.0])
LIFT = 0.0002  # m: every body is built this far over the bench

# --- colours (sRGB) ---
CUP_C = (0.86, 0.88, 0.90)  # translucent polypropylene, drawn opaque
TABLET_C = (0.96, 0.96, 0.94)
ER_C = (0.97, 0.97, 0.95)  # the aspirin: white (its listing's photo)
CARD_C, PRINT_C = (0.98, 0.84, 0.10), (0.04, 0.04, 0.04)
CRUSHER_C, CRUSHER_TOP_C = (0.93, 0.93, 0.92), (0.62, 0.64, 0.66)
SAUCE_C = (0.80, 0.66, 0.38)


def rot_z(a: float) -> np.ndarray:
    return mk.axis_angle((0.0, 0.0, 1.0), a)


def rot_y(a: float) -> np.ndarray:
    return mk.axis_angle((0.0, 1.0, 0.0), a)


def tapered_cup(top_d: float, bottom_d: float, height: float, wall: float, staves: int, color) -> list:
    """An open cup: ``staves`` flat wall boxes round a cone (outer top and bottom diameters, m) and a floor disc.
    Each stave is as wide as the top's polygon side, so the wall has no gaps; lower down the staves overlap (their
    corners stay inside the neighbouring stave's thickness to within 0.2 mm)."""
    r_top, r_bot = top_d / 2, bottom_d / 2
    taper = math.atan2(r_top - r_bot, height)
    slant = height / math.cos(taper)
    width = 2 * r_top * math.tan(math.pi / staves) + 0.0002
    r_mid = (r_top + r_bot) / 2
    shapes = []
    for k in range(staves):
        a = 2 * math.pi * k / staves
        r = rot_z(a) @ rot_y(taper)
        radial = r_mid - wall / 2 * math.cos(taper)
        centre = (radial * math.cos(a), radial * math.sin(a), height / 2 + wall / 2 * math.sin(taper))
        shapes.append(box((wall, width, slant), pos=centre, quat=mk.matrix_to_quat(r), color=color))
    shapes.append(cylinder(r_bot - wall * 0.5, wall, pos=(0.0, 0.0, wall / 2), color=color))
    return shapes


def ring_wall(r_in: float, r_out: float, z0: float, z1: float, staves: int, color) -> list:
    """A straight wall from radius r_in to r_out between heights z0 and z1: ``staves`` boxes, no gaps."""
    width = 2 * r_out * math.tan(math.pi / staves) + 0.0002
    r = (r_in + r_out) / 2
    return [box((r_out - r_in, width, z1 - z0), pos=(r * math.cos(a), r * math.sin(a), (z0 + z1) / 2), yaw=a, color=color)
            for a in (2 * math.pi * k / staves for k in range(staves))]  # fmt: skip


# Every size, mass, pitch and torque below is one constant with its source beside it (README, "Key figures").

# --- the cups (four, the fourth holds the applesauce): Duni Dressing Cup 30 ml Transparent 1-Comp, art. 198962, rPET
# (maker's page: global.duni.com, p/198962, "Piece length 45 / width 45 / height 33 mm", "Piece gross weight (g) 1"):
# 45 mm across the rim and 33 mm high (maker). Estimate: 32 mm at the foot (the frustum then holds 36 ml to the brim, the 30 ml 3 to 4 mm
# under it). The wall is drawn 1 mm thick for the simulator (1.3 g of PET over the cup's area is about 0.2 mm) ---
CUP_TOP_D, CUP_FOOT_D, CUP_H, CUP_WALL = 0.045, 0.032, 0.033, 0.0010
CUP_MASS = 0.0013  # kg, estimate from the maker's pack data: 136 g per 100 cups, the bag included (less about 6 g for it); the page's piece weight, rounded: 1 g
CUP_CONTACT = 0.0005  # m: cups 1 to 3's contact offset (the kit's 2 mm is twice their 1 mm wall: run w12s31, a pinch on the rim also took up the sloped wall under it, lifted the cup 0.2 mm and shook it, the tablet hopping up to 27 mm in it; with 0.5 mm 1.4 mm at most, w12s32)
CUP_ROW_X = 0.25  # m: the row's line, across the arms' front
CUP_YS = (0.250, 0.140, 0.030)  # m: the three places of the row, the arms left to right (110 mm apart: a hand on the next cup stays 2 cm clear of the 80 mm card: dry run)
ER_PLACE = 1  # the aspirin's cup's place in the row (0, 1, 2): cup 3

# --- the tablets. They slide on plastic: their friction is taken as the lower of theirs and what they touch ---
# Cups 1 and 2: PARACETAMOL 500 Fair Med Tabletten (Fairmed Healthcare, PZN 01162549), round, 12.8 mm across, 3.9 mm
# high, 553 mg (ABDATA listing on apotheken-umschau.de: "12,8 mm / 3,9 mm / 553,0 mg"). The listing's tablet is scored on one side ("Einzelkerbe", "in 2 dosisgleiche Hälften teilbar"):
# drawn as a plain disc
PLAIN_D, PLAIN_T = 0.0128, 0.0039
PLAIN_MASS = 0.000553
# Cup 3: the one not to crush ("ER": gastro-resistant, enteric-coated), ASPIRIN Protect 100 mg (Bayer Vital, PZN
# 06706149), round, 7.0 mm across, 3.1 mm high, 141 mg (ABDATA listing on apotheken-umschau.de: "Durchmesser: 7,0 mm",
# "Höhe: 3,1 mm", "Masse: 141,0 mg", "Durchschnittswerte, Circa-Angaben", "ohne Kerbe"). The listing: "Die Tabletten sollten nicht zerkleinert, zerbrochen oder zerkaut werden." ER_L = ER_W
ER_L, ER_W, ER_T = 0.0070, 0.0070, 0.0031
ER_MASS = 0.000141
TABLET_FRICTION = (0.4, 0.3)  # estimate: a coated tablet on smooth plastic
TABLETS = ("tablet_a", "tablet_er", "tablet_b")
PLAIN = ("tablet_a", "tablet_b")


def round_tablet(d: float, t: float, color) -> list:
    """A round biconvex tablet ``d`` across and ``t`` thick: a band and two shallow domes."""
    band = t * 2 / 3
    cap = (t - band) / 2
    r = d / 2
    return [cylinder(r, band, color=color), ellipsoid((r, r, cap * 1.6), pos=(0.0, 0.0, band / 2 - cap * 0.6), color=color),
            ellipsoid((r, r, cap * 1.6), pos=(0.0, 0.0, -band / 2 + cap * 0.6), color=color)]  # fmt: skip


def plain_tablet() -> list:
    return round_tablet(PLAIN_D, PLAIN_T, TABLET_C)


def er_tablet() -> list:
    if ER_L != ER_W:
        raise ValueError("the aspirin is built round (ER_L = ER_W); an oval one needs its own shapes")
    return round_tablet(ER_W, ER_T, ER_C)


def tablet_reach(name: str, direction_in_tablet) -> float:
    """How far tablet ``name`` reaches from its middle along a unit direction given in its own frame (m): a cylinder
    for a round tablet (the aspirin's line also takes a stadium slab, its length along x)."""
    n = np.asarray(direction_in_tablet, dtype=float)
    if name == "tablet_er":
        return float((ER_L - ER_W) / 2 * abs(n[0]) + ER_W / 2 * math.hypot(n[0], n[1]) + ER_T / 2 * abs(n[2]))
    return float(PLAIN_D / 2 * math.hypot(n[0], n[1]) + PLAIN_T / 2 * abs(n[2]))


# --- the cue card under the aspirin's cup (cup 3): yellow card, the cup on its front half (towards the arms), the print
# on its back half, beyond the cup, reading from the arms' side (card_print.py -> card_print.json): "card" 60 x 40 mm,
# 'DO NOT CRUSH' in 5.5 mm capitals and a 12.5 mm crossed-tablet symbol; "card_large" 80 x 50 mm, 8 mm and 14 mm ---
PRINT = json.loads((HERE / "card_print.json").read_text())
CARD = "card_large"  # the spec's fallback: the 60 x 40 mm card's 5.5 mm capitals were 5.9 px with the hand 8.5 cm over it (run w12p2), these 8 mm ones 8.5 px (run w12p3)
CARD_T = 0.0003
CARD_CUP_V = -PRINT[CARD]["size_mm"][1] / 4000.0  # m: the cup's centre from the card's centre, up the page (away from the arms): the front half's middle
PRINT_T = 0.00005


def card_shapes() -> list:
    """The card in its own frame: x up the page (away from the arms), y to the left as one reads it, z up."""
    w, d = (s / 1000.0 for s in PRINT[CARD]["size_mm"])
    shapes = [box((d, w, CARD_T), pos=(0.0, 0.0, CARD_T / 2), color=CARD_C)]
    z = CARD_T + 0.00001 + PRINT_T / 2
    for u0, v0, u1, v1 in PRINT[CARD]["rects"]:
        du, dv = (u1 - u0) / 1000.0, (v1 - v0) / 1000.0
        u, v = (u0 + u1) / 2000.0, (v0 + v1) / 2000.0
        shapes.append(box((dv, du, PRINT_T), pos=(v, -u, z), color=PRINT_C, solid=False))
    return shapes


# --- the auxiliary label on cup 3's side, facing the arms: a stocked 'DO NOT CRUSH' label
# (yellow, black print, 1-9/16 x 3/8 in, the size of PDC Healthcare's auxiliary labels: McKesson listings of items
# 1-28G and 1-49G; ours printed on yellow label stock), wrapped round the cup's cone; drawn only ---
LABEL_MID_Z = 0.015  # m: the label's middle above the cup's foot
LABEL_OFF = 0.00025  # m: off the cone, outside the staves' flat faces (0.17 mm beyond the cone at their edges)
LABEL_SEGMENTS = 16


def on_cone(u: float, v: float, du: float, dv: float, off: float, color, t: float = 0.0001):
    """A flat piece of the label, its middle u along the label (m, to the right as one reads it, from the label's
    middle) and v up it (along the wall), lying on the cup's cone ``off`` outside it, ``t`` thick."""
    r_top, r_bot = CUP_TOP_D / 2, CUP_FOOT_D / 2
    taper = math.atan2(r_top - r_bot, CUP_H)
    z = LABEL_MID_Z + v * math.cos(taper)
    radius = r_bot + (r_top - r_bot) * z / CUP_H
    mid_radius = r_bot + (r_top - r_bot) * LABEL_MID_Z / CUP_H
    a = math.pi + u / mid_radius  # its middle faces the arms (-x); u to the right as seen from there (-y)
    rr = radius + (off + t / 2) * math.cos(taper)
    centre = (rr * math.cos(a), rr * math.sin(a), z - (off + t / 2) * math.sin(taper))
    return box((t, du, dv), pos=centre, quat=mk.matrix_to_quat(rot_z(a) @ rot_y(taper)), color=color, solid=False)


def label_shapes() -> list:
    w, h = (s / 1000.0 for s in PRINT["label"]["size_mm"])
    seg = w / LABEL_SEGMENTS
    shapes = [on_cone(-w / 2 + seg * (k + 0.5), 0.0, seg + 0.0002, h, LABEL_OFF, CARD_C) for k in range(LABEL_SEGMENTS)]
    for u0, v0, u1, v1 in PRINT["label"]["rects"]:
        shapes.append(on_cone((u0 + u1) / 2000.0, (v0 + v1) / 2000.0, (u1 - u0) / 1000.0, (v1 - v0) / 1000.0, LABEL_OFF + 0.0001 + 0.00001, PRINT_C, t=PRINT_T))
    return shapes


# --- the crusher: Dahlhausen Tablettenmörser, art. 9270804132 (PZN 01123294). Maker's page (dahlhausen.de shop):
# "Höhe (zugedreht): ca. 80 mm, Außendurchmesser: ca. 48 mm, Innendurchmesser: ca. 35 mm, Tablettenfach: ca. 43 mm".
# The maker sells to the trade only ("Verkauf nur an gewerbliche
# Kunden"); the pharmacy listing mycare.de/online-kaufen/tablettenmoerser--1123294 gives the same words ("Quelle:
# www.dahlhausen.de"). The 35 mm read as the well's width. Estimates: the split of the
# height (the cap 25 mm, the base the rest less a 3 mm gap left at the thread's end), the well's depth, the plunger. The
# base's frame: its foot's middle, z up its axis. The top's frame: the pressing face's middle, z towards the cap ---
BASE_D, BASE_H, WELL_D, WELL_DEPTH = 0.048, 0.0525, 0.035, 0.012
FLOOR = BASE_H - WELL_DEPTH  # m: the well's floor over the base's foot
BASE_XY = (0.36, -0.07)  # far enough from cup 2 that the left's open hand passes it on its way to the hold (6.7 mm, dry; at (0.34, -0.03) its finger caught cup 2: run w12s5), the right turning the top there with 11.6 deg of joint margin
CAP_D, CAP_H, PLUNGER_D, PLUNGER_H = 0.048, 0.025, 0.032, 0.012  # estimate: the plunger 1.5 mm clear of the well's wall a side (run w12s10: with 1 mm, a top lined up to 0.7 to 1.2 mm met the rim's edge)
TOP_XY = (0.31, -0.18)  # beside the base, clear of the left's hand at the hold
BASE_MASS, TOP_MASS = 0.030, 0.020  # kg, estimate: polypropylene (0.91 g/cm3) shells with 2.5 mm walls over the maker's sizes: 29 g, and 17 g plus the lid compartment's wall
CRUSHER_CONTACT = 0.0005  # m: the base's and the top's contact offset (the kit's 2 mm each: runs w12s10 and w12s11, a top lowered in line towards the mouth was thrown off line, and the hand 6 mm, once its plunger came within about 4 mm of the rim)

# The thread and the press (estimates)
LEAD = 0.003  # m a turn: a coarse single-start thread, as plastic screw lids of this size have
F0 = 0.0085  # m: the pressing face over the well's floor where the thread starts: 3.5 mm into the well, so a top set on rests with its plunger in the well (estimate)
D_BOTTOM = 0.0030  # m: where the thread ends (estimate: 1.8 turns from the start; a layer of powder stays under the face)
RELEASE_TURNS = 0.25  # turned back this far past the thread's start, the top sits on the start, off the thread's grip
RELEASE_LIFT = WELL_DEPTH - (F0 + LEAD * RELEASE_TURNS) + 0.0005  # m: lifted this far off the start there (its face 0.5 mm over the rim), it is free (run w12p26: freed 1 mm up, a top still turned there was thrown off the well's wall)
LIFT_ROOM = 0.006  # m the carrier's upper stop lets it rise meanwhile
RESCREW = 0.02  # turns: sitting on the start, it goes back on the thread when turned down this far
TAKE_OFF, TAKE_TILT = (WELL_D - PLUNGER_D) / 2, math.radians(3.0)  # in line with the base: the plunger's play in the well
TAKE_REACH = 0.0005  # m: the face at most this above the thread's start (3 mm below the rim): a top in the air is never taken
TOUCH_TOL, CONTACT_BAND = 0.0003, 0.0003  # m: the face this near a tablet's top touches it; tablets this near the highest are under it together
GIVE = math.radians(30.0)  # turned this far past the contact, a tablet gives (estimate: 0.25 mm of travel, a tablet's give at failure plus the plastic's)
# N m at the give, per tablet under the face. Estimate: 500 mg paracetamol tablets break at 117 to 174.7 N (five
# brands: Abebe, Beressa, Yimer, Drug Healthc Patient Saf 2020;12:273, doi 10.2147/DHPS.S282420); through the 3 mm
# thread, 22 mm mean radius, 30 deg flanks: F (LEAD/2pi + mu r/cos 30) = 0.35 to 0.53 N m with mu 0.1 (a smooth,
# greased thread); 0.65 to 1.41 N m with 0.2 to 0.3 (dry polypropylene on itself, estimate), the top of that past a
# pinch's about 1 N m. Taken: 0.45, the middle of the first. The demo crushes one tablet at a time (two at once turned
# the base 34 deg in the left's hold: run w12s41)
TABLET_NM = 0.45
K_TABLET = TABLET_NM / GIVE  # N m/rad, as read
# The press's spring holds 0.79 of the stiffness it is built with in this scene (run w12p7, the base held by a weld:
# 12.5 deg at 0.10 N m and 25.2 deg at 0.20 N m on a 0.573 N m/rad build); it is built stiffer by that rate.
PRESS_READ = 0.79
K_BUILT = K_TABLET / PRESS_READ
PRESS_BACK, PRESS_MAX = math.radians(-3.0), math.radians(40.0)  # the press's stops
RELEASE_BACK = math.radians(-1.0)  # turned back this far while pressing: back on the thread, the tablet whole
UNPRESS_LIFT = 0.0005  # m the face is put above the contact when it goes back on the thread unpressed
PLAY = 0.00001  # m either way the thread's axial stops leave the carrier (PhysX refuses equal stops)


def base_shapes() -> list:
    """The base's solid core under the well and the well's wall. The mouth has no lead-in: one built of boxes tilted
    45 degrees threw a plunger that touched it (runs w12p22 and w12p23: 0.5 to 2.5 m)."""
    return [cylinder(BASE_D / 2, FLOOR, pos=(0.0, 0.0, FLOOR / 2), color=CRUSHER_C)] + ring_wall(WELL_D / 2, BASE_D / 2, FLOOR, BASE_H, 24, CRUSHER_C)


def top_shapes() -> list:
    """The top in its own frame: the plunger (pressing face at the origin) and the knurled cap above it."""
    shapes = [cylinder(PLUNGER_D / 2, PLUNGER_H, pos=(0.0, 0.0, PLUNGER_H / 2), color=CRUSHER_C),
              cylinder(CAP_D / 2, CAP_H, pos=(0.0, 0.0, PLUNGER_H + CAP_H / 2), color=CRUSHER_TOP_C)]  # fmt: skip
    for k in range(24):  # the knurl: drawn only, the pads grip the cap's round face
        a = 2 * math.pi * k / 24
        shapes.append(box((0.0008, 0.0025, CAP_H * 0.86), pos=((CAP_D / 2 + 0.0002) * math.cos(a), (CAP_D / 2 + 0.0002) * math.sin(a), PLUNGER_H + CAP_H / 2),
                          yaw=a, color=CRUSHER_TOP_C, solid=False))  # fmt: skip
    return shapes


# --- the applesauce: a heaped teaspoon (10 g, about 9.5 ml) in cup 4, a fourth cup: its surface SAUCE_LEVEL
# over the cup's foot (computed: 10 g of applesauce, about 9.5 ml, in the cup's sizes); 11.3 g with the cup ---
SAUCE_TOP_D, SAUCE_FOOT_D, SAUCE_H = CUP_TOP_D, CUP_FOOT_D, CUP_H
SAUCE_LEVEL = 0.0105
SAUCE_XY = (0.45, -0.105)  # where the right hand can tip the base 120 degrees over it (IK margins 18 deg)
SAUCE_MASS = 0.0113
SAUCE_SLICES = 3
# Wet applesauce holds the powder that lands on it: a grain that touched cup 4 during a control step (the mean normal
# force over its physics steps) where its free fall from the last step meets the applesauce's surface, or that lies at
# the surface (its middle at most STICK_BAND over it), is set there and held (welded), as a bounce off the rigid
# surface would not happen on the sauce. Run w12s40: 36 of the 44 grains on the applesauce held; the 4 that missed flew
# over cup 4's rim without touching it (traced, run w12s39).
STICK_BAND = 0.005  # m


def sauce_shapes() -> list:
    """Cup 4 and the applesauce in it: a rigid surface at SAUCE_LEVEL, built of SAUCE_SLICES discs that each stay
    0.2 mm inside the cup's wall at their own foot."""
    shapes = tapered_cup(SAUCE_TOP_D, SAUCE_FOOT_D, SAUCE_H, CUP_WALL, 24, CUP_C)
    z0, dz = CUP_WALL, (SAUCE_LEVEL - CUP_WALL) / SAUCE_SLICES
    for k in range(SAUCE_SLICES):
        low = z0 + k * dz
        r = SAUCE_FOOT_D / 2 + (SAUCE_TOP_D - SAUCE_FOOT_D) / 2 * low / SAUCE_H - CUP_WALL - 0.0002
        shapes.append(cylinder(r, dz, pos=(0.0, 0.0, low + dz / 2), color=SAUCE_C))
    return shapes


# --- the grains a tablet gives: GRAINS_PER spheres of its mass, its colours; parked hidden on the floor under the table
# until then. They slide like a powder (the lower friction of theirs and what they touch) ---
GRAINS_PER = 24
GRAIN_R = 0.0012
GRAIN_FRICTION = (0.5, 0.4)
# Damped (1/s, linear and angular) as a powder that does not roll or bounce like steel balls, and put down under the
# face only, 2.2 mm or more inside its edge: undamped, grains rolled to the wall of the slightly tilted base (0.5 to
# 0.8 deg in the holding hand), hopped, jammed in the plunger's 1.5 mm gap and were carried out of the well as the top
# went up (runs w12s19, w12s20: 5 of 48 lost). Linear 10/s: the air's drag on the powder they stand in for, whose
# 0.2 mm particles fall at about 1 m/s (g / 10 per s), finer ones slower (estimate: a 0.2 mm sphere of 1.3 g/cm3
# falls through air at 0.9 m/s). At 2/s, grains thrown off the poured powder at up to 1 m/s flew 40 to 150 mm, over
# cup 4's rim (run w12s42: 5 of the 8 on the bench).
GRAIN_DAMPING = (10.0, 20.0)
# Pressed powder is a packed cake: from the crush the grains are held on the base where they were put (no
# dynamics: welded), until the base leans past SLIDE_ANGLE, where pressed powder starts to slide. Estimate: the wall
# friction angle of pharmaceutical powders, medians 18 to 22 deg on stainless steel over more than 100 materials and
# higher on polymer (Hancock, J Pharm Sci 2019;108:457, doi 10.1016/j.xphs.2018.10.019). Loose, 5 to 10
# of 48 grains hopped out of the well at the regrips and the base's grip and lift (runs w12s21 to w12s24).
SLIDE_ANGLE = math.radians(30.0)
GRAIN_SPREAD = PLUNGER_D / 2 - GRAIN_R - 0.001  # m: the grains' middles at most this far from the well's axis
PARK_Z = station.FLOOR_Z
GRAINS = {t: tuple(f"grain_{t.split('_')[1]}{i:02d}" for i in range(GRAINS_PER)) for t in TABLETS}
PARK = {t: (0.10 + 0.05 * k, 0.48, PARK_Z + 0.01) for k, t in enumerate(TABLETS)}


def grain_park(k: int) -> tuple[float, float, float]:
    return (0.08 + 0.015 * (k % 24), 0.36 + 0.02 * (k // 24), PARK_Z + GRAIN_R + 0.0005)


def build() -> list:
    objects = []
    for i, y in enumerate(CUP_YS):
        er = i == ER_PLACE
        lift = CARD_T if er else 0.0
        name = "cup_er" if er else f"cup_{'ab'[i - (i > ER_PLACE)]}"
        shapes = tapered_cup(CUP_TOP_D, CUP_FOOT_D, CUP_H, CUP_WALL, 24, CUP_C) + (label_shapes() if er else [])
        objects.append(body(name, shapes, (CUP_ROW_X, y, TOP + lift + LIFT), mass=CUP_MASS, contact_offset=CUP_CONTACT))
        if er:
            objects.append(body("card", card_shapes(), (CUP_ROW_X - CARD_CUP_V, y, TOP)))
            objects.append(body("tablet_er", er_tablet(), (CUP_ROW_X, y, TOP + lift + CUP_WALL + ER_T / 2 + 0.0003), mass=ER_MASS, yaw=0.35,
                                friction=TABLET_FRICTION, friction_combine="min"))  # fmt: skip
        else:
            objects.append(body(f"tablet_{name[-1]}", plain_tablet(), (CUP_ROW_X, y, TOP + CUP_WALL + PLAIN_T / 2 + 0.0003), mass=PLAIN_MASS,
                                friction=TABLET_FRICTION, friction_combine="min"))  # fmt: skip
    objects.append(body("base", base_shapes(), (*BASE_XY, TOP + LIFT), mass=BASE_MASS, contact_offset=CRUSHER_CONTACT))
    objects.append(body("top", top_shapes(), (*TOP_XY, TOP + LIFT), mass=TOP_MASS, contacts=True, contact_offset=CRUSHER_CONTACT))  # its touches read: on a tablet, on the bench
    objects.append(body("sauce", sauce_shapes(), (*SAUCE_XY, TOP + LIFT), mass=SAUCE_MASS))
    k = 0
    for t in TABLETS:
        mass = (ER_MASS if t == "tablet_er" else PLAIN_MASS) / GRAINS_PER
        for i, g in enumerate(GRAINS[t]):
            color = ER_C if t == "tablet_er" else TABLET_C
            objects.append(body(g, [sphere(GRAIN_R, color=color)], grain_park(k), mass=mass, friction=GRAIN_FRICTION, friction_combine="min", contact_offset=0.001,
                                linear_damping=GRAIN_DAMPING[0], angular_damping=GRAIN_DAMPING[1]))  # fmt: skip
            k += 1
    return objects


# --- the hidden bodies: the thread's carrier and turner, and the press. Each is four small boxes round its own axis,
# 8 mm off it, that every other body passes through, all inside the base's solid core: the carrier's and the turner's
# 30 mm under their joints' place (run w12p4: with the boxes at the thread's start, in the plunger's way, a top written
# in line there was knocked 15 degrees over within one control step). Their weight is borne by a push of their own,
# so that the base and the top weigh what they do ---
HIDDEN = ("carrier", "turner", "press")
HIDDEN_MASS = 0.02
START_AT = np.array([*BASE_XY, TOP + LIFT + FLOOR + F0])  # the face's middle where the thread starts, as built
PRESS_AT = np.array([*BASE_XY, TOP + LIFT + FLOOR / 2])


GHOST_BELOW = 0.030  # m the carrier's and the turner's boxes lie under the thread's start: in the core, the thread's 7.5 mm run included


def _ghost(z: float, passes) -> list:
    return [box((0.0016, 0.0016, 0.002), pos=(0.008 * math.cos(a), 0.008 * math.sin(a), z), passes=passes, color=CRUSHER_C)
            for a in (math.pi / 4 + k * math.pi / 2 for k in range(4))]  # fmt: skip


# No seat holds a free top at the thread's start. Runs w12p9 to w12p22 tried a 1 mm disc on the light carrier, a
# kinematic slab in the well under the face and kinematic pads under the cap's rim: each threw the top (1 to 7 m/s, 30
# to 300 rad/s within one control step) once the top also touched the base, though the same slab held it still on the
# bench and the well's own floor held it still (w12p15). A top let fall into the well lies on its floor or the tablets.


def declare() -> list:
    """The scene, declared for the task file that calls it: the bodies (returned: its OBJECTS), the hidden bodies and
    their joints, the welds the rules make and cut, the grains' touches with cup 4; each part's boxes (BOXES)."""
    objects = build()
    passes = tuple(o.name for o in objects) + HIDDEN
    for name, at, z in (("carrier", START_AT, -GHOST_BELOW), ("turner", START_AT, -GHOST_BELOW), ("press", PRESS_AT, 0.0)):
        objects.append(body(name, _ghost(z, passes), tuple(at), mass=HIDDEN_MASS))
    slide("along", "carrier", "base", axis=(0.0, 0.0, 1.0), limits=(-PLAY, PLAY))
    # damping 0.01 N m s/rad (estop_mount's thread): a top spun freely turns under half a turn a control step, where the turn is read
    hinge("turn", "turner", "carrier", anchor=tuple(START_AT), axis=(0.0, 0.0, -1.0), limits=None, damping=0.01)
    hinge("press", "press", "base", anchor=tuple(PRESS_AT), axis=(0.0, 0.0, -1.0), limits=(PRESS_BACK, PRESS_MAX), spring=K_BUILT, rest=0.0, damping=0.02)
    # Both welds sit at the top's own origin, its face's middle (run w12p17: anchored at the thread's start, 12 cm from
    # the top as built, the weld was a long lever and held the taken top 11.6 mm high and 8.8 mm off the axis)
    weld("on", "top", "turner", made=False)  # on the thread
    weld("clamp", "top", "press", made=False)  # on a tablet, against the press's spring
    weld("base_hold", "base", None, anchor=(*BASE_XY, TOP + LIFT), made=False)  # trials only: the base held where it stands
    weld("top_hold", "top", None, made=False)  # trials only: the top held where it is put, as a hand holds it
    for t in TABLETS:
        for g in GRAINS[t]:
            weld(f"cake_{g}", g, "base", made=False)  # the pressed powder, held on the base until it leans past SLIDE_ANGLE
            weld(f"stick_{g}", g, "sauce", made=False)  # on the applesauce, once it has touched it
    sense_contact("sauce", *(g for t in TABLETS for g in GRAINS[t]))  # each grain's touch with cup 4 (STICK_BAND)
    BOXES.update({name: [s.bounds() for s in mk.MECH.bodies[name].shapes if s.solid] for name in ("base", "top", "sauce", "cup_a", "cup_er", "cup_b", "card")})
    return objects


SIDES = ("left", "right")
STATE: dict = {}




# --- where things are ---
def in_base(point) -> np.ndarray:
    p, r = mk.pose("base")
    return r.T @ (np.asarray(point, dtype=float) - p)


def top_reading() -> dict:
    """The top against the base: its face's distance from the base's axis (m), the angle between their axes (rad),
    its face's height over the well's floor along the base's axis (m)."""
    p, r = mk.pose("base")
    q, s = mk.pose("top")
    local = r.T @ (q - p)
    return {"off": float(math.hypot(local[0], local[1])), "tilt": float(math.acos(max(-1.0, min(1.0, float((r @ Z) @ (s @ Z)))))),
            "face": float(local[2] - FLOOR)}  # fmt: skip


def in_well(name: str) -> bool:
    """Body ``name``'s middle in the crusher's well (up to 3 mm over its rim)."""
    local = in_base(mk.pose(name)[0])
    return bool(math.hypot(local[0], local[1]) < WELL_D / 2 and FLOOR - 0.002 <= local[2] <= BASE_H + 0.003)


def tablet_top(name: str) -> float:
    """The highest point of tablet ``name`` over the well's floor, along the base's axis (m)."""
    p, r = mk.pose("base")
    q, s = mk.pose(name)
    up = r @ Z
    return float((r.T @ (q - p))[2] - FLOOR + tablet_reach(name, s.T @ up))


def whole() -> list[str]:
    return [t for t in TABLETS if t not in STATE["crushed"]]


def under_face() -> list[str]:
    """The whole tablets in the well: the pressing face (32 mm) covers the well's floor but for 1.5 mm at its wall."""
    return [t for t in whole() if in_well(t)]


def in_cone(local, foot_d: float, top_d: float, height: float, low: float, high: float) -> bool:
    """A point (in a cup's frame) inside the cup between heights low and high."""
    z = float(local[2])
    if not low <= z <= high:
        return False
    radius = foot_d / 2 + (top_d - foot_d) / 2 * min(max(z, 0.0), height) / height
    return bool(math.hypot(local[0], local[1]) < radius)


def in_cup(point, cup: str) -> bool:
    p, r = mk.pose(cup)
    return in_cone(r.T @ (np.asarray(point, dtype=float) - p), CUP_FOOT_D, CUP_TOP_D, CUP_H, 0.0, CUP_H + 0.002)


def in_sauce(point) -> bool:
    p, r = mk.pose("sauce")
    return in_cone(r.T @ (np.asarray(point, dtype=float) - p), SAUCE_FOOT_D, SAUCE_TOP_D, SAUCE_H, SAUCE_LEVEL - 0.004, SAUCE_H + 0.003)


def surface_hit(before, speed, now, within: float = 0.06):
    """Where a grain met the applesauce's surface (cup 4's frame): on its free fall from ``before`` at ``speed`` (the
    last step's reading) within ``within`` s, or ``now`` if it lies at the surface; None if neither is on it."""
    top = SAUCE_LEVEL + GRAIN_R
    reach = SAUCE_FOOT_D / 2 + (SAUCE_TOP_D - SAUCE_FOOT_D) / 2 * SAUCE_LEVEL / SAUCE_H - CUP_WALL - GRAIN_R
    if before is not None and before[2] > top:
        t = (speed[2] + math.sqrt(speed[2] ** 2 + 2 * GRAVITY * (before[2] - top))) / GRAVITY
        hit = np.array([before[0] + speed[0] * t, before[1] + speed[1] * t, top])
        if t <= within and math.hypot(hit[0], hit[1]) < reach:
            return hit
    if -GRAIN_R <= now[2] - SAUCE_LEVEL <= STICK_BAND and math.hypot(now[0], now[1]) < reach:
        return np.array([now[0], now[1], top])
    return None


def sauce_share() -> float:
    """The paracetamol grains' share on the applesauce (0 unless both are crushed)."""
    if not all(t in STATE["crushed"] for t in PLAIN):
        return 0.0
    grains = [g for t in PLAIN for g in GRAINS[t]]
    return sum(in_sauce(mk.pose(g)[0]) for g in grains) / len(grains)


def report() -> dict:
    return {"mode": STATE.get("mode"), "face over the floor (mm)": None if STATE.get("d") is None else round(1000 * STATE["d"], 2),
            "most turns down": round(STATE.get("most_turns", 0.0), 3), "most press (deg)": round(math.degrees(STATE.get("most_press", 0.0)), 1),
            "crushed": STATE.get("crushed"), "where": {t: where(t) for t in TABLETS}, "grains in the applesauce": round(sauce_share(), 3),
            "aspirin in the crusher at some moment": STATE.get("er_in_crusher"), "in the applesauce at some moment": STATE.get("er_in_sauce"),
            "held on the applesauce": len(STATE.get("stuck", ())), "take corrections (t, mm off, deg, mm over the start)": STATE.get("corrections"),
            "closest before the take": STATE.get("closest"), "events": STATE.get("events")}  # fmt: skip


def where(tablet: str) -> str:
    """Where a whole tablet is: 'crushed', 'crusher', a cup's name, 'sauce', or 'loose' (anywhere else)."""
    if tablet in STATE["crushed"]:
        return "crushed"
    point = mk.pose(tablet)[0]
    if in_well(tablet):
        return "crusher"
    for cup in ("cup_a", "cup_er", "cup_b"):
        if in_cup(point, cup):
            return cup
    return "sauce" if in_sauce(point) else "loose"


REST_WINDOW, REST_MOVE, REST_TURN = 0.2, 0.0005, math.radians(0.5)  # s, m, rad: still = over the last 0.2 s its face's middle moved no more than 0.5 mm and its axis turned no more than 0.5 deg against the base


def grain_spots(tablet: str) -> list[np.ndarray]:
    """GRAINS_PER places (the base's frame) on the well's floor for the grains of ``tablet``, under the face
    (GRAIN_SPREAD): the nearest free places to where it lay, clear of the whole tablets still in the well and of the
    grains already there."""
    pitch = 2 * GRAIN_R + 0.0002
    spots = []
    rows = int(WELL_D / pitch) + 2
    for i in range(-rows, rows + 1):
        for j in range(-rows, rows + 1):
            x, y = (j + 0.5 * (i % 2)) * pitch, i * pitch * math.sqrt(3) / 2
            if math.hypot(x, y) <= GRAIN_SPREAD:
                spots.append((x, y))
    used = [np.asarray(v) for vs in STATE["spots"].values() for v in vs]
    others = [(in_base(mk.pose(t)[0])[:2], (ER_L if t == "tablet_er" else PLAIN_D) / 2) for t in under_face() if t != tablet]
    near = in_base(mk.pose(tablet)[0])[:2]
    free = [s for s in spots if all(np.hypot(*(np.asarray(s) - u[:2])) >= pitch for u in used)
            and all(np.hypot(*(np.asarray(s) - c)) >= half + GRAIN_R + 0.0005 for c, half in others)]  # fmt: skip
    free.sort(key=lambda s: float(np.hypot(*(np.asarray(s) - near))))
    if len(free) < GRAINS_PER:
        raise RuntimeError(f"no room for {tablet}'s grains on the well's floor ({len(free)} places)")
    return [np.array([x, y, FLOOR + GRAIN_R + 0.0001]) for x, y in free[:GRAINS_PER]]


# m: the face stops this far over the highest grain under it, past the reach of their contact offsets (the top's
# 0.5 mm and the grains' 1 mm). Run w12s18: turned on down onto the grains (the face 3.7 mm over the floor) the crusher
# was thrown 4 m; trial w12p28: stopped 0.3 mm over them and spun there, 20 of 48 grains left the well and the top went
# 14 mm off its axis (26 mm, and fell over, with the base held tilted 10 deg)
POWDER_GAP = 0.0016


def grains_on(tablet: str) -> list[str]:
    """The grains lying on whole tablet ``tablet``: over its footprint (in the base's frame), their middles above its
    middle. They are pressed with it, not a powder stop over it."""
    c = in_base(mk.pose(tablet)[0])
    half = (ER_L if tablet == "tablet_er" else PLAIN_D) / 2
    out = []
    for t in STATE["crushed"]:
        for g in GRAINS[t]:
            local = in_base(mk.pose(g)[0])
            if math.hypot(local[0] - c[0], local[1] - c[1]) <= half and local[2] > c[2]:
                out.append(g)
    return out


def stack_top(tablet: str) -> float:
    """Whole tablet ``tablet``'s highest point over the well's floor, or that of a grain lying on it (m)."""
    return max([tablet_top(tablet)] + [float(in_base(mk.pose(g)[0])[2]) - FLOOR + GRAIN_R for g in grains_on(tablet)])


def lean() -> float:
    """The base's lean from upright (rad)."""
    return float(math.acos(max(-1.0, min(1.0, float((mk.pose("base")[1] @ Z)[2])))))


# --- rest, read from travel ---
MOVING = ("base", "top", "sauce", "cup_a", "cup_er", "cup_b", "tablet_er")
CLEAR, RESTING, SETTLE_S, ON_BENCH = 0.03, 0.02, 0.25, 0.003
SAUCE_SHARE = 0.9  # the share of the paracetamol grains on the applesauce for 1.0: the task's threshold
RUN_LIMIT_STEPS = 7000  # task.json run_limit (350 s at 20 Hz), the harness's --max-steps; the demo stops there too


# --- the score ---
BOXES: dict = {}  # each part's solid shapes' boxes in its own frame (declare())


def lowest(name: str) -> float:
    """The lowest corner of body ``name``'s solid shapes' boxes (world z, m)."""
    p, r = mk.pose(name)
    return min(float((p + r @ np.array([x, y, z]))[2]) for low, high in BOXES[name] for x in (low[0], high[0]) for y in (low[1], high[1]) for z in (low[2], high[2]))


def on_bench(name: str) -> bool:
    return lowest(name) <= TOP + ON_BENCH


def distance_to(name: str, point) -> float:
    """How far ``point`` is from body ``name``'s solid shapes (their boxes; m, 0 inside one)."""
    position, rotation = mk.pose(name)
    local = rotation.T @ (np.asarray(point, dtype=float) - position)
    return min(float(np.linalg.norm(np.maximum(np.maximum(np.asarray(low) - local, local - np.asarray(high)), 0.0))) for low, high in BOXES[name])


def hands_clear() -> bool:
    """Both grippers' grasp points 3 cm or more from the crusher, the cups, the applesauce and the card."""
    return all(distance_to(name, mk.gripper(side)[0]) >= CLEAR for side in SIDES for name in BOXES)
