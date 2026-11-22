"""Rigid bodies and joints of ``baby_scale_convert``, built from ``parts.py``:
- ``marks``: fixed yellow corner tape marks on the bench around ``scale_base`` (drawn only, ``solid=False``).
- ``scale_base``: free 1.47 kg toddler scale platform (280 x 65 x 235 mm) on rubber corner feet. Its standing deck is
  closed; the 82 mm front apron carries the LCD, the red seca badge (+y), two round keys each side and the round
  release button; right behind the button lies the recessed front step with two keyhole slots (seca manual Fig. 1-2).
- ``release_btn``: 30 g light-grey round button with its pawl on ``slide("btn_slide")`` (vertical spring 2,400 N/m =
  12.0 N at 5.0 mm; reaction presses ``scale_base`` onto its feet). Jointed bodies do not touch each other.
- ``weighing_tray``: free 0.80 kg open-ended infant scoop (525 x 80 x 250 mm): floor running out to open head and foot
  wings, front and back walls that step down toward the wings, no end walls. Underneath: two runners, two support ribs
  and a latch plate with two keyhole T-studs and a tongue that the pawl blocks. Zero welds, zero pose snaps.
- ``mat``: free 0.30 kg grey IKEA TILLGANG tray (290 x 370 x 20 mm) behind the scale.
"""

from __future__ import annotations

import math

import mechkit as mk
import parts as P
from mechkit import body, box, cylinder, slide

MECHKIT_HELPER = True

MEDICAL_WHITE = (0.94, 0.95, 0.96)
TRAY_CREAM = (0.96, 0.96, 0.95)
STEP_GREY = (0.70, 0.72, 0.75)
LCD_BEZEL = (0.28, 0.30, 0.33)
LCD_GLASS = (0.48, 0.56, 0.50)
BTN_LIGHT = (0.84, 0.86, 0.89)  # light-grey round button (seca_pst_354 p. 1-2) with a dark ring so it reads at 320 x 176
BTN_RING = (0.34, 0.37, 0.41)
KEY_GREY = (0.62, 0.64, 0.67)
SECA_RED = (0.82, 0.14, 0.18)
RUBBER_DARK = (0.15, 0.15, 0.17)
MAT_GREY = (0.34, 0.36, 0.39)
TAPE_YELLOW = (0.95, 0.80, 0.10)
HOOK_GREY = (0.68, 0.70, 0.73)
DECK_FRICTION = (0.18, 0.14)
SLIDE_FRICTION = (0.15, 0.12)

# Tray walls, (|y| from, |y| to, rim top above the tray origin): full 80 mm cradle in the middle, stepping down toward
# the open head and foot wings like the photo's back wall (seca_pst_354 p. 1)
WALL_SEGMENTS = ((0.0, 0.110, P.SKID_H + P.TRAY_H), (0.110, 0.150, 0.084), (0.150, 0.185, 0.074), (0.185, 0.215, 0.062),
                 (0.215, P.TRAY_W / 2, 0.050))
FLOOR_Z = (P.SKID_H, P.SKID_H + P.FLOOR_T)  # scoop floor and wings, 76..82 mm above the bench on the scale
EDGE_Y = P.TRAY_W / 2 - P.EDGE_D / 2  # rolled edge along each open end (rod axis along x), tangent to the 525 mm width
EDGE_Z = 0.5 * sum(FLOOR_Z) + 0.002  # 2 mm above the floor's mid-plane: 7 mm proud of the floor top, 3 mm below it
RIB_Z0 = P.STUD_Z[0] - P.BASE_H - 0.0026  # support ribs reach 2.6 mm below the stud heads: the tray stands on its ribs


def _span_box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float, *, orig_x: float, color, friction=None, solid: bool = True):
    return box(
        (x1 - x0, y1 - y0, z1 - z0),
        pos=((x0 + x1) / 2 - orig_x, (y0 + y1) / 2, (z0 + z1) / 2),
        color=color,
        friction=friction,
        solid=solid,
    )


def _both_sides(y0: float, y1: float) -> tuple[tuple[float, float], tuple[float, float]]:
    """The band ``y0 <= |y| <= y1`` as its +y and -y halves."""
    return (y0, y1), (-y1, -y0)


def build_marks(top: float) -> None:
    """Four yellow L-shaped tape marks on the bench hugging ``scale_base``'s outer corners."""
    x0, x1 = P.BASE_FRONT_X, P.BASE_FRONT_X + P.BASE_D
    y0, y1 = -P.BASE_W / 2, P.BASE_W / 2
    arm, w, h = 0.036, 0.008, 0.0006
    shapes = []
    for sx, cx in ((-1.0, x0), (1.0, x1)):
        for sy, cy in ((-1.0, y0), (1.0, y1)):
            shapes.append(box((arm, w, h), pos=(cx + sx * (arm / 2 - w), cy + sy * w / 2, h / 2), color=TAPE_YELLOW, solid=False))
            shapes.append(box((w, arm, h), pos=(cx + sx * w / 2, cy + sy * (arm / 2 - w), h / 2), color=TAPE_YELLOW, solid=False))
    body("marks", shapes, pos=(0.0, 0.0, top), kinematic=True, color=TAPE_YELLOW)


def build_scale_base_and_button(top: float) -> None:
    """``scale_base`` (origin ``(BASE_X, 0, top)``) and sprung ``release_btn`` (origin ``(BTN_X, 0, top)``)."""
    ox = P.BASE_X
    x0, x1 = P.BASE_FRONT_X, P.BASE_FRONT_X + P.BASE_D
    hw = P.BASE_W / 2
    core_top = 0.044
    s0, s1 = P.STEP_X

    def plate(xa, xb, ya, yb, za=core_top, zb=P.BASE_H, color=MEDICAL_WHITE):
        return _span_box(xa, xb, ya, yb, za, zb, orig_x=ox, color=color, friction=DECK_FRICTION)

    shapes = []
    for sx in (-1.0, 1.0):  # 4 rubber corner feet (z = 0..6 mm)
        for sy in (-1.0, 1.0):
            shapes.append(box((0.036, 0.036, 0.006), pos=(sx * (P.BASE_D / 2 - 0.024), sy * (hw - 0.024), 0.003),
                              color=RUBBER_DARK, friction=(0.95, 0.85)))
    shapes.append(_span_box(x0, x1, -hw, hw, 0.006, core_top, orig_x=ox, color=MEDICAL_WHITE, friction=(0.45, 0.40)))
    # Front apron (LCD, keys; the button cap stands up through it: jointed bodies do not touch, and a fingertip wider
    # than the cap stops on the apron after 5.5 mm, past the 4.3 mm unlock) and the closed standing deck behind the step
    shapes.append(plate(x0, s0, -hw, hw))
    shapes.append(plate(s1, x1, -hw, hw))
    # Recessed front step (3 mm below the deck): solid except the two keyholes and the pawl slot
    kc, kh, ks = P.KEY_Y, P.KEY_COL_HALF_Y, P.KEY_SLOT_HALF_Y
    pawl_slot = (P.PAWL_X[0] - 0.002, P.PAWL_X[1] + 0.002)
    centre_h = P.PAWL_HALF_Y + 0.002
    for ya, yb in _both_sides(kc + kh, hw) + _both_sides(centre_h, kc - kh):
        shapes.append(plate(s0, s1, ya, yb, zb=P.STEP_TOP, color=STEP_GREY))
    shapes.append(plate(s0, pawl_slot[0], -centre_h, centre_h, zb=P.STEP_TOP, color=STEP_GREY))
    shapes.append(plate(pawl_slot[1], s1, -centre_h, centre_h, zb=P.STEP_TOP, color=STEP_GREY))
    for sgn in (-1.0, 1.0):
        ka, kb = sorted((sgn * (kc - kh), sgn * (kc + kh)))
        shapes.append(plate(s0, P.KEY_LIP_X[0], ka, kb, zb=P.STEP_TOP, color=STEP_GREY))  # solid ahead of the slot
        for la, lb in ((kc - kh, kc - ks), (kc + ks, kc + kh)):  # the two lips beside the narrow slot
            ya, yb = sorted((sgn * la, sgn * lb))
            shapes.append(plate(P.KEY_LIP_X[0], P.KEY_LIP_X[1], ya, yb, za=P.KEY_LIP_Z0, zb=P.STEP_TOP, color=STEP_GREY))
    # Front-apron graphics (drawn only): LCD, red seca badge left (+y), two round keys each side, dark button ring
    shapes.append(_span_box(0.137, 0.154, -0.040, 0.040, P.BASE_H, P.BASE_H + 0.0010, orig_x=ox, color=LCD_BEZEL, solid=False))
    shapes.append(_span_box(0.139, 0.152, -0.032, 0.032, P.BASE_H + 0.0010, P.BASE_H + 0.0016, orig_x=ox, color=LCD_GLASS, solid=False))
    shapes.append(_span_box(0.141, 0.151, 0.084, 0.112, P.BASE_H, P.BASE_H + 0.0012, orig_x=ox, color=SECA_RED, solid=False))
    for ky in (0.052, 0.068, -0.052, -0.068):
        shapes.append(cylinder(0.0045, 0.0014, pos=(0.1455 - ox, ky, P.BASE_H + 0.0007), color=KEY_GREY, solid=False))
    shapes.append(cylinder(0.0175, 0.0005, pos=(P.BTN_X - ox, 0.0, P.BASE_H + 0.00025), color=BTN_RING, solid=False))

    ib = P.BASE_MASS / 12.0
    body("scale_base", shapes, pos=(ox, P.BASE_Y, top), mass=P.BASE_MASS, color=MEDICAL_WHITE, friction=(0.45, 0.40),
         center=(0.0, 0.0, 0.032),
         inertia=(ib * (P.BASE_W ** 2 + P.BASE_H ** 2), ib * (P.BASE_D ** 2 + P.BASE_H ** 2), ib * (P.BASE_D ** 2 + P.BASE_W ** 2)),
         linear_damping=0.5, angular_damping=0.5)

    # Sprung button: round cap, a link inside the base and the pawl standing up through the step's slot
    bx, ph = P.BTN_X, P.PAWL_HALF_Y
    btn_shapes = [
        cylinder(0.014, P.BTN_TOP_Z - 0.056, pos=(0.0, 0.0, 0.5 * (0.056 + P.BTN_TOP_Z)), color=BTN_LIGHT, friction=(0.60, 0.50)),
        _span_box(bx, P.PAWL_X[1], -0.010, 0.010, 0.046, 0.053, orig_x=bx, color=HOOK_GREY, friction=SLIDE_FRICTION),
        _span_box(P.PAWL_X[0], P.PAWL_X[1], -ph, ph, 0.046, P.PAWL_TOP_Z, orig_x=bx, color=HOOK_GREY, friction=SLIDE_FRICTION),
    ]
    body("release_btn", btn_shapes, pos=(bx, P.BASE_Y, top), mass=P.BTN_MASS, color=BTN_LIGHT, friction=(0.30, 0.25),
         center=(0.030, 0.0, 0.055), inertia=(2e-5, 6e-5, 6e-5), linear_damping=1.0, angular_damping=1.0)
    slide("btn_slide", "release_btn", "scale_base", axis=(0.0, 0.0, -1.0), limits=(0.0, P.BTN_STROKE),
          anchor=(bx, P.BASE_Y, top + 0.060), spring=P.BTN_SPRING, rest=0.0, damping=18.0)


def build_weighing_tray(top: float) -> None:
    """``weighing_tray`` (origin ``(TRAY_X, 0, top + BASE_H + 0.0004)``: local z = 0 is the runners' bottom)."""
    tx = P.TRAY_X
    zb = P.BASE_H  # world z above the bench -> tray local z: subtract zb

    def under(x0, x1, y0, y1, z0, z1, friction=SLIDE_FRICTION):  # world x / bench z in, tray-local shape out
        return _span_box(x0, x1, y0, y1, z0 - zb, z1 - zb, orig_x=tx, color=HOOK_GREY, friction=friction)

    shapes = []
    for sgn in (-1.0, 1.0):
        # Runners on the closed deck (y = +-95 mm) and support ribs outside the 280 mm base (y = +-158 mm)
        shapes.append(_span_box(P.STEP_X[1] + 0.002, 0.360, sgn * 0.095 - 0.008, sgn * 0.095 + 0.008, 0.0, P.SKID_H, orig_x=tx,
                                color=HOOK_GREY, friction=SLIDE_FRICTION))
        shapes.append(box((0.180, 0.012, P.SKID_H - RIB_Z0), pos=(0.0, sgn * 0.158, 0.5 * (P.SKID_H + RIB_Z0)),
                          color=HOOK_GREY, friction=(0.55, 0.45)))
        # Keyhole T-stud: stem through the narrow slot, head under the two lips
        ky = sgn * P.KEY_Y
        shapes.append(under(*P.STUD_STEM_X, ky - P.STUD_STEM_HALF_Y, ky + P.STUD_STEM_HALF_Y, P.STUD_Z[0], zb + 0.004))
        shapes.append(under(*P.STUD_HEAD_X, ky - P.STUD_HEAD_HALF_Y, ky + P.STUD_HEAD_HALF_Y, *P.STUD_Z))
    # Latch plate under the front of the scoop (clears the pawl top by 1.9 mm) and the tongue the pawl blocks
    shapes.append(under(P.STUD_HEAD_X[0] - 0.003, P.STEP_X[1] + 0.002, -0.064, 0.064, zb + 0.004, zb + P.SKID_H))
    shapes.append(under(*P.TONGUE_X, -P.PAWL_HALF_Y, P.PAWL_HALF_Y, P.TONGUE_Z0, zb + 0.004))

    # The scoop: floor running out to the open head and foot wings, a rolled edge along each open end (where both hands
    # grip), front and back walls stepping down toward the wings with a rounded rim each, and no end walls
    shapes.append(box((0.140, 2 * EDGE_Y, P.FLOOR_T), pos=(0.0, 0.0, 0.5 * sum(FLOOR_Z)), color=TRAY_CREAM, friction=mk.GRIPPY))
    for sgn in (-1.0, 1.0):
        shapes.append(cylinder(P.EDGE_D / 2, P.TRAY_D, axis="x", pos=(0.0, sgn * EDGE_Y, EDGE_Z), color=MEDICAL_WHITE, friction=mk.GRIPPY))
    x_in, x_rim = 0.066, P.TRAY_D / 2 - 0.006
    for y0, y1, rim_top in WALL_SEGMENTS:
        z_in, z_rim = FLOOR_Z[0] + 0.007, rim_top - 0.008
        length = math.hypot(x_rim - x_in, z_rim - z_in)
        ang = math.atan2(z_rim - z_in, x_rim - x_in)
        bands = ((-y1, y1),) if y0 == 0.0 else _both_sides(y0, y1)
        for ya, yb in bands:
            for sgn in (-1.0, 1.0):  # front (-x) and back (+x) walls
                q = tuple(float(v) for v in mk.matrix_to_quat(mk.axis_angle((0.0, 1.0, 0.0), -sgn * ang)))
                cx, cz = sgn * 0.5 * (x_in + x_rim), 0.5 * (z_in + z_rim)
                shapes.append(box((length, yb - ya, 0.010), pos=(cx, 0.5 * (ya + yb), cz), quat=q, color=TRAY_CREAM, friction=mk.GRIPPY))
                shapes.append(box((0.012, yb - ya, 0.016), pos=(sgn * x_rim, 0.5 * (ya + yb), rim_top - 0.008),
                                  color=MEDICAL_WHITE, friction=mk.GRIPPY))

    it = P.TRAY_MASS / 12.0
    body("weighing_tray", shapes, pos=(tx, P.TRAY_Y, top + P.BASE_H + 0.0004), mass=P.TRAY_MASS, color=TRAY_CREAM,
         friction=(0.55, 0.45), center=(0.0, 0.0, 0.030),
         inertia=(it * (P.TRAY_W ** 2 + P.TRAY_H ** 2), it * (P.TRAY_D ** 2 + P.TRAY_H ** 2), it * (P.TRAY_D ** 2 + P.TRAY_W ** 2)),
         linear_damping=0.4, angular_damping=0.4, contacts=True)


def build_mat(top: float) -> None:
    """``mat``: free 300 g grey IKEA TILLGANG tray (290 x 370 x 20 mm) resting on the bench at ``(MAT_X, MAT_Y)``."""
    ft, rt = P.MAT_FLOOR_T, P.MAT_RIM_T
    rim_h = P.MAT_H - ft
    shapes = [
        box((P.MAT_D, P.MAT_W, ft), pos=(0.0, 0.0, ft / 2), color=MAT_GREY, friction=(0.75, 0.65)),
        box((rt, P.MAT_W, rim_h), pos=(-P.MAT_D / 2 + rt / 2, 0.0, ft + rim_h / 2), color=MAT_GREY, friction=(0.60, 0.50)),
        box((rt, P.MAT_W, rim_h), pos=(P.MAT_D / 2 - rt / 2, 0.0, ft + rim_h / 2), color=MAT_GREY, friction=(0.60, 0.50)),
        box((P.MAT_D - 2 * rt, rt, rim_h), pos=(0.0, -P.MAT_W / 2 + rt / 2, ft + rim_h / 2), color=MAT_GREY, friction=(0.60, 0.50)),
        box((P.MAT_D - 2 * rt, rt, rim_h), pos=(0.0, P.MAT_W / 2 - rt / 2, ft + rim_h / 2), color=MAT_GREY, friction=(0.60, 0.50)),
    ]
    im = P.MAT_MASS / 12.0
    body("mat", shapes, pos=(P.MAT_X, P.MAT_Y, top + 0.0003), mass=P.MAT_MASS, color=MAT_GREY, friction=(0.75, 0.65),
         center=(0.0, 0.0, 0.006),
         inertia=(im * (P.MAT_W ** 2 + P.MAT_H ** 2), im * (P.MAT_D ** 2 + P.MAT_H ** 2), im * (P.MAT_D ** 2 + P.MAT_W ** 2)),
         linear_damping=0.5, angular_damping=0.5)


def build(top: float) -> None:
    build_marks(top)
    build_scale_base_and_button(top)
    build_weighing_tray(top)
    build_mat(top)
    mk.sense_contact("weighing_tray", "scale_base", "release_btn", "mat")
