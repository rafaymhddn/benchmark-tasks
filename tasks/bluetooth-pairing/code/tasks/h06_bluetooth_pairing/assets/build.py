#!/usr/bin/env python3
"""Build the models of the Bluetooth pairing task: a speaker, its three sprung buttons and a phone.

    python3 tasks/h06_bluetooth_pairing/assets/build.py

Writes, next to this file: ``layout.json`` (every dimension and position the task file uses), ``textures/*.png``
(printed panel, button icons, phone screens) and ``*.usda`` (models with physics). Also adds the models' bounds
to ``docs/assets/bounds.json``. Needs only Pillow. Everything is original geometry and artwork; the speaker's
name is made up.

Frames: metres; x away from the arms, y to the robot's left, z up. A model's origin is on the bench under its
centre. Printed faces read from the robot's side: text runs from +y to -y and its top points to +x.
"""

from __future__ import annotations

import json
import math
from itertools import permutations
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
TEX = HERE / "textures"
ROOT = HERE.parents[2]
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

TABLE_TOP, REST_GAP = 0.7475, 0.001  # yam_arena.station (checked by the task's tests)

# --- layout (the task file reads layout.json) ---------------------------------------------------------------------
SPEAKER_NAME = "SoundBox Mini"
NEIGHBOURS = {"tv": "Kitchen TV", "earbuds": "Mara's Earbuds"}
SPK_XY = (0.22, 0.18)
SPK = (0.10, 0.20, 0.07)  # depth (x), length (y), height
SPK_R = 0.02  # edge rounding along the length
PANEL = (0.06, 0.19)  # printed top panel
BTN_R, BTN_H = 0.013, 0.006  # cap radius and thickness
BTN_GAP = 0.004  # under the cap at rest
BTN_TRAVEL = 0.003
BTN_STIFFNESS, BTN_DAMPING = 500.0, 5.0  # N/m, N s/m: 1.5 N at full travel
BTN_DX = -0.010  # the button row, from the speaker's centre line
BTN_DY = {"pair": -0.05, "play": 0.0, "power": 0.05}  # pairing at the right end, toward the middle of the bench
LED_D = (-0.010, -0.082)  # right of the pairing button: the left arm, reaching in from the left, does not cover it
LED_R = 0.006
TEXT_DX = 0.016
PHONE_XY = (0.20, -0.17)
PHONE = (0.164, 0.078, 0.009)
PHONE_R = 0.010
SCREEN = (0.152, 0.070)
ZONE = 0.032  # one touch zone along the phone: the bottom button and each of three list rows
HEADER = SCREEN[0] - 4 * ZONE
DEVICES = ("speaker", "tv", "earbuds")
ORDERS = [list(p) for p in permutations(DEVICES)]


def layout() -> dict:
    sx, sy = SPK_XY
    px, py = PHONE_XY
    near = px - SCREEN[0] / 2
    return {
        "speaker_name": SPEAKER_NAME,
        "neighbours": NEIGHBOURS,
        "speaker": {"xy": SPK_XY, "size": SPK, "top": SPK[2]},
        "buttons": {b: (round(sx + BTN_DX, 4), round(sy + dy, 4)) for b, dy in BTN_DY.items()},
        "button": {"radius": BTN_R, "height": BTN_H, "gap": BTN_GAP, "travel": BTN_TRAVEL, "top": SPK[2] + BTN_GAP + BTN_H,
                   "stiffness": BTN_STIFFNESS, "damping": BTN_DAMPING},
        "led": {"xy": (round(sx + LED_D[0], 4), round(sy + LED_D[1], 4)), "radius": LED_R},
        "phone": {"xy": PHONE_XY, "size": PHONE, "top": PHONE[2]},
        "screen": {"size": SCREEN, "zone": (ZONE, SCREEN[1])},
        "scan_button": (round(near + ZONE / 2, 4), py),
        "rows": [(round(near + ZONE * (3.5 - i), 4), py) for i in range(3)],  # row 0 is the top of the list (far end)
        "devices": DEVICES,
        "orders": ORDERS,
        "assets": {"speaker": "h06_speaker", "phone": "h06_phone", **{f"button_{b}": f"h06_button_{b}" for b in BTN_DY}},
        "extent": {"speaker": SPK[1], "phone": PHONE[0], "button": 2 * BTN_R},
    }


# --- textures -----------------------------------------------------------------------------------------------------
def font(size, bold=False):
    return ImageFont.truetype(BOLD if bold else FONT, size)


def centred(d, xy, text, f, fill):
    d.text(xy, text, font=f, fill=fill, anchor="mm")


def icon_power(d, c, r, col, w):
    x, y = c
    d.arc((x - r, y - r, x + r, y + r), -60, 240, fill=col, width=w)
    d.line((x, y - r * 1.15, x, y - r * 0.1), fill=col, width=w)


def icon_play(d, c, r, col, w):
    x, y = c
    d.polygon([(x - r, y - r * 0.8), (x - r, y + r * 0.8), (x - r * 0.05, y)], fill=col)
    d.rectangle((x + r * 0.25, y - r * 0.8, x + r * 0.5, y + r * 0.8), fill=col)
    d.rectangle((x + r * 0.75, y - r * 0.8, x + r, y + r * 0.8), fill=col)


def icon_bluetooth(d, c, r, col, w):
    x, y = c
    pts = [(x - r * 0.55, y - r * 0.5), (x + r * 0.5, y + r * 0.5), (x, y + r), (x, y - r), (x + r * 0.5, y - r * 0.5), (x - r * 0.55, y + r * 0.5)]
    d.line(pts, fill=col, width=w, joint="curve")


ICONS = {"power": (icon_power, (235, 90, 80)), "play": (icon_play, (235, 235, 235)), "pair": (icon_bluetooth, (90, 165, 255))}
CAP_RGB = (58, 60, 66)
RUBBER = (26, 27, 30)
PRINT = (205, 208, 212)


def tex_fabric():
    im = Image.new("RGB", (128, 128), (40, 42, 47))
    d = ImageDraw.Draw(im)
    for i in range(0, 128, 16):
        for j in range(0, 128, 16):
            o = 8 if (j // 16) % 2 else 0
            d.ellipse((i + o + 3, j + 3, i + o + 11, j + 11), fill=(22, 23, 26))
    im.save(TEX / "fabric.png")


def tex_panel():
    """The speaker's top: 100 px per cm; image right is -y, image up is +x."""
    w, h = round(PANEL[1] * 1e4), round(PANEL[0] * 1e4)
    im = Image.new("RGB", (w, h), RUBBER)
    d = ImageDraw.Draw(im)

    def px(dx, dy):
        return (w / 2 - dy * 1e4, h / 2 - dx * 1e4)

    d.rounded_rectangle((6, 6, w - 7, h - 7), radius=60, outline=(60, 62, 68), width=4)
    centred(d, px(TEXT_DX, 0.0), SPEAKER_NAME.upper(), font(118, True), PRINT)
    for b, dy in BTN_DY.items():
        x, y = px(BTN_DX, dy)
        r = BTN_R * 1e4 + 14
        d.ellipse((x - r, y - r, x + r, y + r), outline=(80, 83, 90), width=6)
        centred(d, (x, h - 30), {"pair": "PAIR", "play": "PLAY", "power": "POWER"}[b], font(34, True), PRINT)
    x, y = px(*LED_D)
    r = LED_R * 1e4 + 12
    d.ellipse((x - r, y - r, x + r, y + r), outline=(80, 83, 90), width=5)
    im.save(TEX / "panel.png")


def tex_caps():
    for b, (draw, col) in ICONS.items():
        im = Image.new("RGB", (256, 256), CAP_RGB)
        draw(ImageDraw.Draw(im), (128, 128), 62, col, 18)
        im.save(TEX / f"cap_{b}.png")


def tex_badge():
    im = Image.new("RGB", (600, 140), (30, 31, 35))
    d = ImageDraw.Draw(im)
    d.rounded_rectangle((3, 3, 596, 136), radius=30, outline=(120, 124, 130), width=5)
    centred(d, (300, 70), "SOUNDBOX", font(84, True), (215, 218, 222))
    im.save(TEX / "badge.png")


W, H = round(SCREEN[1] * 1e4), round(SCREEN[0] * 1e4)  # 700 x 1520: 100 px per cm, portrait, top = far end
ZPX, HPX = round(ZONE * 1e4), round(HEADER * 1e4)
BG, CARD, INK, DIM, BLUE, GREEN = (245, 246, 250), (255, 255, 255), (22, 24, 30), (120, 124, 134), (30, 110, 245), (20, 160, 85)


def dev_icon(d, kind, c):
    x, y = c
    d.ellipse((x - 62, y - 62, x + 62, y + 62), fill=(228, 232, 242))
    if kind == "speaker":
        d.rounded_rectangle((x - 30, y - 42, x + 30, y + 42), radius=10, fill=INK)
        d.ellipse((x - 16, y + 2, x + 16, y + 34), fill=(228, 232, 242))
        d.ellipse((x - 8, y - 30, x + 8, y - 14), fill=(228, 232, 242))
    elif kind == "tv":
        d.rounded_rectangle((x - 42, y - 30, x + 42, y + 22), radius=6, fill=INK)
        d.rectangle((x - 22, y + 30, x + 22, y + 38), fill=INK)
    else:
        for s in (-1, 1):
            d.ellipse((x + s * 24 - 16, y - 34, x + s * 24 + 16, y - 2), fill=INK)
            d.rounded_rectangle((x + s * 24 - 7, y - 10, x + s * 24 + 7, y + 38), radius=6, fill=INK)


def screen_base(title):
    im = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(im)
    d.text((34, 26), "12:30", font=font(34, True), fill=INK)
    d.rounded_rectangle((W - 110, 30, W - 40, 60), radius=8, outline=INK, width=4)
    d.rectangle((W - 104, 36, W - 62, 54), fill=INK)
    d.text((34, 120), title, font=font(64, True), fill=INK)
    return im, d


def row_box(i):
    top = HPX + i * ZPX
    return (20, top + 12, W - 20, top + ZPX - 12)


def draw_row(d, i, kind, name, sub, sub_col=DIM):
    x0, y0, x1, y1 = row_box(i)
    d.rounded_rectangle((x0, y0, x1, y1), radius=36, fill=CARD, outline=(222, 225, 232), width=3)
    cy = (y0 + y1) // 2
    dev_icon(d, kind, (x0 + 92, cy))
    d.text((x0 + 180, cy - 62), name, font=font(50, True), fill=INK)
    d.text((x0 + 180, cy + 10), sub, font=font(38), fill=sub_col)


def bottom_button(d, text, fill, ink, outline=None):
    top = HPX + 3 * ZPX
    d.rounded_rectangle((36, top + 60, W - 36, top + ZPX - 60), radius=100, fill=fill, outline=outline, width=6)
    centred(d, (W // 2, top + ZPX // 2), text, font(54, True), ink)


def tex_screens():
    names = {"speaker": SPEAKER_NAME, **NEIGHBOURS}
    im, d = screen_base("Bluetooth")
    d.text((34, HPX + 40), "On", font=font(50, True), fill=INK)
    d.rounded_rectangle((W - 190, HPX + 30, W - 50, HPX + 106), radius=38, fill=BLUE)
    d.ellipse((W - 122, HPX + 36, W - 58, HPX + 100), fill=CARD)
    d.text((34, HPX + 200), "Paired devices", font=font(42, True), fill=DIM)
    d.text((34, HPX + 270), "None", font=font(46), fill=INK)
    bottom_button(d, "Pair new device", BLUE, CARD)
    im.save(TEX / "screen_home.png")

    im, d = screen_base("Pair new device")
    cx, cy = W // 2, HPX + ZPX * 3 // 2
    d.arc((cx - 90, cy - 150, cx + 90, cy + 30), 20, 320, fill=BLUE, width=16)
    centred(d, (cx, cy + 110), "Searching for devices…", font(46), INK)
    bottom_button(d, "Searching…", (214, 218, 228), DIM)
    im.save(TEX / "screen_scan.png")

    for k, order in enumerate(ORDERS):
        for with_speaker in (0, 1):
            im, d = screen_base("Available devices")
            for i, dev in enumerate(order):
                if dev != "speaker" or with_speaker:
                    draw_row(d, i, dev, names[dev], "Tap to pair")
            bottom_button(d, "Scan again", BG, BLUE, outline=BLUE)
            im.save(TEX / f"screen_list_{k}_{with_speaker}.png")

    im, d = screen_base("Bluetooth")
    draw_row(d, 0, "speaker", SPEAKER_NAME, "Connecting…", BLUE)
    x0, y0, x1, y1 = row_box(0)
    cy = (y0 + y1) // 2
    d.arc((x1 - 116, cy - 38, x1 - 40, cy + 38), 20, 320, fill=BLUE, width=10)
    bottom_button(d, "Connecting…", (214, 218, 228), DIM)
    im.save(TEX / "screen_connecting.png")

    im, d = screen_base("Bluetooth")
    draw_row(d, 0, "speaker", SPEAKER_NAME, "Connected", GREEN)
    x0, y0, x1, y1 = row_box(0)
    cy = (y0 + y1) // 2
    d.line((x1 - 110, cy, x1 - 84, cy + 28, x1 - 40, cy - 30), fill=GREEN, width=14, joint="curve")
    bottom_button(d, "Pair new device", BG, BLUE, outline=BLUE)
    im.save(TEX / "screen_connected.png")


SCREENS = ["home", "scan", *(f"list_{k}_{s}" for k in range(len(ORDERS)) for s in (0, 1)), "connecting", "connected"]


# --- USD text -----------------------------------------------------------------------------------------------------
def f(v) -> str:
    return f"{v:.6g}"


def vec(p) -> str:
    return "(" + ", ".join(f(v) for v in p) + ")"


def rounded_rect(w, h, r, n=8):
    """Outline of a w x h rectangle with rounded corners, counter-clockwise, centred: [(a, b)]."""
    pts = []
    for cx, cy, a0 in ((w / 2 - r, h / 2 - r, 0), (-w / 2 + r, h / 2 - r, 90), (-w / 2 + r, -h / 2 + r, 180), (w / 2 - r, -h / 2 + r, 270)):
        for k in range(n + 1):
            a = math.radians(a0 + 90 * k / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def _normal(pts):
    n = [0.0, 0.0, 0.0]
    for i, p in enumerate(pts):
        q = pts[(i + 1) % len(pts)]
        n[0] += (p[1] - q[1]) * (p[2] + q[2])
        n[1] += (p[2] - q[2]) * (p[0] + q[0])
        n[2] += (p[0] - q[0]) * (p[1] + q[1])
    return n


def mesh(name, points, faces, material, st=None, outward_from=None, extra="") -> str:
    """A polygon mesh; ``faces`` index ``points``; ``st`` gives one uv per face corner. With ``outward_from`` each
    face is wound to point away from that centre."""
    faces = [list(fc) for fc in faces]
    st = [list(s) for s in st] if st else None
    if outward_from is not None:
        for k, fc in enumerate(faces):
            pts = [points[i] for i in fc]
            c = [sum(p[a] for p in pts) / len(pts) for a in range(3)]
            n = _normal(pts)
            if sum(n[a] * (c[a] - outward_from[a]) for a in range(3)) < 0:
                faces[k] = fc[::-1]
                if st:
                    st[k] = st[k][::-1]
    lo = [min(p[a] for p in points) for a in range(3)]
    hi = [max(p[a] for p in points) for a in range(3)]
    out = [
        f'    def Mesh "{name}"',
        "    {",
        f"        float3[] extent = [{vec(lo)}, {vec(hi)}]",
        f"        int[] faceVertexCounts = [{', '.join(str(len(fc)) for fc in faces)}]",
        f"        int[] faceVertexIndices = [{', '.join(str(i) for fc in faces for i in fc)}]",
        f"        point3f[] points = [{', '.join(vec(p) for p in points)}]",
        '        uniform token subdivisionScheme = "none"',
        "        uniform bool doubleSided = 1",
        f"        rel material:binding = </{ROOT_NAME[0]}/Looks/{material}>",
    ]
    if st:
        out.append(f'        texCoord2f[] primvars:st = [{", ".join(vec(uv) for s in st for uv in s)}] (interpolation = "faceVarying")')
    if extra:
        out.append(extra)
    out.append("    }")
    return "\n".join(out)


ROOT_NAME = [""]  # the model being written (material paths are absolute inside the file)


def prism(name, outline, lo, hi, to3d, side_mat, cap_mat, tile=None) -> str:
    """An outline swept from ``lo`` to ``hi`` along the axis ``to3d(a, b, s)`` maps to; sides and two end caps."""
    n = len(outline)
    pts = [to3d(a, b, lo) for a, b in outline] + [to3d(a, b, hi) for a, b in outline]
    centre = to3d(0.0, 0.0, (lo + hi) / 2)
    per = [0.0]
    for i in range(n):
        (a0, b0), (a1, b1) = outline[i], outline[(i + 1) % n]
        per.append(per[-1] + math.hypot(a1 - a0, b1 - b0))
    faces = [(i, (i + 1) % n, n + (i + 1) % n, n + i) for i in range(n)]
    st = None
    if tile:
        st = [[(per[i] / tile, lo / tile), (per[i + 1] / tile, lo / tile), (per[i + 1] / tile, hi / tile), (per[i] / tile, hi / tile)] for i in range(n)]
    sides = mesh(f"{name}_sides", pts, faces, side_mat, st, outward_from=centre)
    caps = mesh(f"{name}_caps", pts, [tuple(range(n)), tuple(range(n, 2 * n))], cap_mat, outward_from=centre)
    return sides + "\n" + caps


def quad(name, centre, up, right, material, extra="") -> str:
    """A printed rectangle: ``up`` and ``right`` are the image's half-extent vectors from ``centre``."""
    c, u, r = centre, up, right
    p = lambda su, sr: tuple(c[a] + su * u[a] + sr * r[a] for a in range(3))  # noqa: E731
    pts = [p(-1, -1), p(-1, 1), p(1, 1), p(1, -1)]
    return mesh(name, pts, [(0, 1, 2, 3)], material, [[(0, 0), (1, 0), (1, 1), (0, 1)]], extra=extra)


def disc(name, centre, radius, material, n=32) -> str:
    """A printed disc facing up; the image's top points to +x and its right to -y."""
    cx, cy, cz = centre
    pts, st = [], []
    for k in range(n):
        a = 2 * math.pi * k / n
        dx, dy = radius * math.cos(a), radius * math.sin(a)
        pts.append((cx + dx, cy + dy, cz))
        st.append((0.5 - dy / (2 * radius), 0.5 + dx / (2 * radius)))
    return mesh(name, pts, [tuple(range(n))], material, [st])


def cylinder(name, radius, height, z, material=None, collide=False, visible=True) -> str:
    api = ' (\n        prepend apiSchemas = ["PhysicsCollisionAPI"]\n    )' if collide else ""
    out = [
        f'    def Cylinder "{name}"{api}',
        "    {",
        '        uniform token axis = "Z"',
        f"        double height = {f(height)}",
        f"        double radius = {f(radius)}",
        f"        float3[] extent = [{vec((-radius, -radius, -height / 2))}, {vec((radius, radius, height / 2))}]",
        f"        double3 xformOp:translate = {vec(z)}",
        '        uniform token[] xformOpOrder = ["xformOp:translate"]',
    ]
    if material:
        out.append(f"        rel material:binding = </{ROOT_NAME[0]}/Looks/{material}>")
    if not visible:
        out.append('        token visibility = "invisible"')
    out.append("    }")
    return "\n".join(out)


def box_collider(size, centre) -> str:
    return "\n".join([
        '    def Cube "collider" (',
        '        prepend apiSchemas = ["PhysicsCollisionAPI"]',
        "    )",
        "    {",
        "        double size = 1",
        "        float3[] extent = [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]",
        f"        double3 xformOp:translate = {vec(centre)}",
        f"        float3 xformOp:scale = {vec(size)}",
        '        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]',
        '        token visibility = "invisible"',
        "    }",
    ])


def material(name, *, color=None, texture=None, roughness=0.5, emissive=None, glow=False, repeat=False) -> str:
    """A UsdPreviewSurface: a flat ``color`` or a ``texture``; ``emissive`` colour, or ``glow`` (the texture lights
    itself, as a display does)."""
    path = f"/{ROOT_NAME[0]}/Looks/{name}"
    out = [f'        def Material "{name}"', "        {", f"            token outputs:surface.connect = <{path}/Shader.outputs:surface>",
           '            def Shader "Shader"', "            {", '                uniform token info:id = "UsdPreviewSurface"']
    if texture and glow:  # a display: black glass that emits the image, so bench lighting does not wash it out
        out.append("                color3f inputs:diffuseColor = (0, 0, 0)")
        out.append(f"                color3f inputs:emissiveColor.connect = <{path}/Tex.outputs:rgb>")
    elif texture:
        out.append(f"                color3f inputs:diffuseColor.connect = <{path}/Tex.outputs:rgb>")
    else:
        out.append(f"                color3f inputs:diffuseColor = {vec(color)}")
    if emissive:
        out.append(f"                color3f inputs:emissiveColor = {vec(emissive)}")
    out += [f"                float inputs:roughness = {f(roughness)}", "                float inputs:metallic = 0", "                token outputs:surface", "            }"]
    if texture:
        wrap = "repeat" if repeat else "clamp"
        out += [
            '            def Shader "Tex"', "            {", '                uniform token info:id = "UsdUVTexture"',
            f"                asset inputs:file = @./textures/{texture}@",
            '                token inputs:sourceColorSpace = "sRGB"',
            f'                token inputs:wrapS = "{wrap}"', f'                token inputs:wrapT = "{wrap}"',
            f"                float2 inputs:st.connect = <{path}/St.outputs:result>", "                float3 outputs:rgb", "            }",
            '            def Shader "St"', "            {", '                uniform token info:id = "UsdPrimvarReader_float2"',
            '                string inputs:varname = "st"', "                float2 outputs:result", "            }",
        ]
    out.append("        }")
    return "\n".join(out)


def model(name, doc, body, looks) -> str:
    return "\n".join([
        "#usda 1.0", "(", f'    defaultPrim = "{name}"', "    metersPerUnit = 1", '    upAxis = "Z"', f'    doc = "{doc}"', ")", "",
        f'def Xform "{name}" (', '    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]', ")", "{",
        *body, '    def Scope "Looks"', "    {", *looks, "    }", "}", "",
    ])


def srgb(r, g, b):
    return tuple((c / 255 / 12.92) if c / 255 <= 0.04045 else ((c / 255 + 0.055) / 1.055) ** 2.4 for c in (r, g, b))


INVISIBLE = '        token visibility = "invisible"'
LEDS = {"white": (1.0, 1.0, 1.0), "blue": (0.1, 0.35, 1.0), "green": (0.1, 1.0, 0.25)}


def build_speaker() -> str:
    ROOT_NAME[0] = "speaker"
    d, length, h = SPK
    outline = [(a, b + h / 2) for a, b in rounded_rect(d, h, SPK_R)]
    top = h + 0.0004
    body = [
        prism("body", outline, -length / 2, length / 2, lambda a, b, s: (a, s, b), "fabric", "plastic", tile=0.01),
        quad("panel", (0, 0, top), (PANEL[0] / 2, 0, 0), (0, -PANEL[1] / 2, 0), "panel"),
        quad("badge", (-d / 2 - 0.0004, 0, h / 2), (0, 0, 0.007), (0, -0.03, 0), "badge"),
        cylinder("led_off", LED_R, 0.002, (*LED_D, top + 0.001), "led_off"),
        *(cylinder(f"led_{c}", LED_R, 0.0024, (*LED_D, top + 0.0012), f"led_{c}", visible=(c == "white")) for c in LEDS),
        box_collider(SPK, (0, 0, h / 2)),
    ]
    looks = [
        material("fabric", texture="fabric.png", roughness=0.9, repeat=True),
        material("plastic", color=srgb(24, 25, 28), roughness=0.45),
        material("panel", texture="panel.png", roughness=0.6),
        material("badge", texture="badge.png", roughness=0.4),
        material("led_off", color=srgb(50, 52, 58), roughness=0.3),
        *(material(f"led_{c}", color=rgb, emissive=tuple(6 * v for v in rgb), roughness=0.2) for c, rgb in LEDS.items()),
    ]
    return model("speaker", "A Bluetooth speaker: fabric body, printed rubber top panel, status light (led_white / led_blue / led_green, one visible at a time). The buttons are separate models.", body, looks)


def build_button(b: str, world_xy) -> str:
    ROOT_NAME[0] = "button"
    top = SPK[2] + BTN_GAP + BTN_H
    body = [
        cylinder("cap", BTN_R, BTN_H, (0, 0, top - BTN_H / 2), "cap", collide=True),
        cylinder("skirt", BTN_R - 0.001, BTN_GAP + 0.004, (0, 0, SPK[2] + BTN_GAP / 2 - 0.002), "cap"),
        cylinder("stem", 0.003, SPK[2], (0, 0, SPK[2] / 2), "cap"),
        disc("icon", (0, 0, top + 0.0002), BTN_R - 0.0005, "icon"),
        "\n".join([
            '    def PhysicsPrismaticJoint "spring" (',
            '        prepend apiSchemas = ["PhysicsDriveAPI:linear"]',
            "    )",
            "    {",
            '        uniform token physics:axis = "Z"',
            "        rel physics:body1 = </button>",
            f"        point3f physics:localPos0 = {vec((*world_xy, TABLE_TOP + REST_GAP))}",
            "        point3f physics:localPos1 = (0, 0, 0)",
            "        quatf physics:localRot0 = (1, 0, 0, 0)",
            "        quatf physics:localRot1 = (1, 0, 0, 0)",
            f"        float physics:lowerLimit = {f(-BTN_TRAVEL)}",
            "        float physics:upperLimit = 0",
            f"        float drive:linear:physics:stiffness = {f(BTN_STIFFNESS)}",
            f"        float drive:linear:physics:damping = {f(BTN_DAMPING)}",
            "        float drive:linear:physics:targetPosition = 0",
            '        uniform token drive:linear:physics:type = "force"',
            "    }",
        ]),
    ]
    looks = [material("cap", color=srgb(*CAP_RGB), roughness=0.7), material("icon", texture=f"cap_{b}.png", roughness=0.7)]
    return model("button", f"The speaker's {b} button: a cap on a spring (a prismatic joint to the world at its place on the speaker, {BTN_TRAVEL * 1000:g} mm of travel).", body, looks)


def build_phone() -> str:
    ROOT_NAME[0] = "phone"
    lx, ly, h = PHONE
    top = h + 0.0003
    body = [
        prism("body", rounded_rect(lx, ly, PHONE_R), 0.0, h, lambda a, b, s: (a, b, s), "frame", "glass"),
        *(quad(f"screen_{s}", (0, 0, top), (SCREEN[0] / 2, 0, 0), (0, -SCREEN[1] / 2, 0), f"screen_{s}", extra="" if s == "home" else INVISIBLE) for s in SCREENS),
        box_collider(PHONE, (0, 0, h / 2)),
    ]
    looks = [
        material("frame", color=srgb(70, 72, 78), roughness=0.3),
        material("glass", color=srgb(8, 8, 10), roughness=0.1),
        *(material(f"screen_{s}", texture=f"screen_{s}.png", roughness=0.15, glow=True) for s in SCREENS),
    ]
    return model("phone", "A phone lying flat, showing one of its Bluetooth screens (screen_<name>, one visible at a time).", body, looks)


def bounds_entry(file, low, high) -> dict:
    size = [round(b - a, 4) for a, b in zip(low, high)]
    return {"extent": max(size[:2]), "file": file, "height": size[2], "high": [round(v, 4) for v in high], "low": [round(v, 4) for v in low], "size": size}


def main() -> None:
    TEX.mkdir(exist_ok=True)
    tex_fabric(), tex_panel(), tex_caps(), tex_badge(), tex_screens()
    lay = layout()
    (HERE / "layout.json").write_text(json.dumps(lay, indent=1) + "\n")
    (HERE / "speaker.usda").write_text(build_speaker())
    (HERE / "phone.usda").write_text(build_phone())
    for b in BTN_DY:
        (HERE / f"button_{b}.usda").write_text(build_button(b, lay["buttons"][b]))
    bounds_file = ROOT / "docs" / "assets" / "bounds.json"
    bounds = json.loads(bounds_file.read_text())
    d, length, h = SPK
    bounds["h06_speaker"] = bounds_entry("speaker.usda", (-d / 2 - 0.0004, -length / 2, 0.0), (d / 2, length / 2, h + 0.0024))
    bounds["h06_phone"] = bounds_entry("phone.usda", (-PHONE[0] / 2, -PHONE[1] / 2, 0.0), (PHONE[0] / 2, PHONE[1] / 2, PHONE[2] + 0.0003))
    for b in BTN_DY:
        bounds[f"h06_button_{b}"] = bounds_entry(f"button_{b}.usda", (-BTN_R, -BTN_R, 0.0), (BTN_R, BTN_R, SPK[2] + BTN_GAP + BTN_H + 0.0002))
    bounds_file.write_text(json.dumps(dict(sorted(bounds.items())), indent=1) + "\n")
    print(f"wrote {len(list(TEX.glob('*.png')))} textures, 5 models, layout.json; bounds for 5 models")


if __name__ == "__main__":
    main()
