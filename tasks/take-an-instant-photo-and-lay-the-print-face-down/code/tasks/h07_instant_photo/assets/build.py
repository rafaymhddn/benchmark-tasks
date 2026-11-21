#!/usr/bin/env python3
"""Build the models of the instant-photo task: a Polaroid Go Generation 2 instant camera (as modelled) and its print.

    python3 tasks/h07_instant_photo/assets/build.py

Writes, next to this file: ``layout.json`` (every dimension the task file uses) and ``camera.usda`` / ``print.usda``
(models with physics); adds the models' bounds to ``docs/assets/bounds.json``. Plain Python, no textures: the
geometry is drawn at the camera's true outer size with plain colours, no maker's name or logo.

Sources. Polaroid's product page for the Go Generation 2: "105mm x 84mm x 62mm", "Weight: 239 grams (8.43 ounces)
(without film pack)", colours Black, White, Blue, Purple. Polaroid's Go film page: "Format: 66.6mm x 53.9mm", "Image
area: 47mm x 46mm". The Go's manual (first generation, same body): "105mm (L) x 83.9mm (W) x 61.5mm (H)", so 105 mm
front to back (the print leaves through the front); the red shutter button is on the top; "The photo will eject from
the slot at the front of the camera as soon as you let go of the shutter button". A retailer's listing (Target) gives
a Go film double pack as 2.6 oz (74 g). Not stated by the maker, and so our estimates (marked below): the button's
place, size, travel and force, the slot's place and size, the film pack's mass, the print's thickness and mass, and
the film shield's length, thickness, mass, spring and roll-back angle.

Camera frame: metres; x out of the lens, y to the camera's left, z up; the origin on the bench under the camera's
centre. The camera is an articulation: the link ``body`` (everything but the button) and the link ``button`` (the
shutter button) on the prismatic joint ``shutter`` along z, held up by a spring; pressed home it has travelled
``SHUTTER_TRAVEL``. A tunnel runs from the front face 8 cm back, its floor 5 mm over the bench: the print lies in it
and leaves through the slot in the front face; the floor runs on 6 mm past the face as the slot's lower lip. The film
shield is the link ``flap``: a thin flap over the slot, hinged at the slot's upper lip on the revolute joint
``shield`` about y, sprung to lie on the print and, once lifted past a small angle, to roll back up (the task sets
the spring's rest angle); a rod across its underside presses the print onto the lower lip, and its friction on the
print's top and the lip's on its underside are what hold the print and what a pull has to overcome. The rollers that
push the print out are not modelled as bodies (the task places the print as it ejects); once it is out their nip is
the link ``nip`` on the prismatic joint ``rollers`` along z: a block that comes down from the roof just behind the
print's inner end, so that the print cannot slide back into the camera.
Print frame: x along its length (the end that leaves the camera first is +x), z out of the picture side (up while the
print is in the camera; the back is black); the origin at the centre of the underside.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]

# --- dimensions (the task file reads layout.json) -----------------------------------------------------------------
CAMERA = "Polaroid Go Generation 2"
DEPTH, WIDTH, HEIGHT = 0.105, 0.084, 0.062  # front to back (x), across (y), high (z): Polaroid's 105 x 84 x 62 mm
CORNER = 0.012  # the rounded vertical edges (estimate)
MASS = 0.239 + 0.035  # Polaroid's 239 g without film pack, and the pack (estimate: half a listed 74 g double pack)
FLOOR, GAP = 0.005, 0.003  # the tunnel's floor over the bench and its height, the slot (estimates: the 2.5 mm card and
# 0.5 mm of play: a print squeezed in it cannot raise the shield's rod 1 mm, so it cannot lift the shield itself)
TUNNEL = (0.080, 0.062)  # depth from the front face, width (estimate: the 53.9 mm print with room either side)
LENS = (0.0165, 0.010, 0.039)  # radius, length out of the front, axis height (estimate)
SHUTTER = (0.030, -0.024)  # the button's middle on the top: front right, under a right index finger (estimate)
SHUTTER_R, SHUTTER_H = 0.006, 0.003  # 12 mm across, standing 3 mm proud of the top (estimate)
SHUTTER_TRAVEL = 0.002  # m from rest to pressed home (estimate)
SHUTTER_SPRING = 2000.0  # N/m: 4 N pressed home (estimate; a bench reads it with a push gauge)
SHUTTER_DAMPING = 5.0  # N s/m (about critical for the 2 g button)
BUTTON_MASS = 0.002
GRIP = (-0.032, 0.0, HEIGHT - 0.016)  # where a hand from above holds the camera's rear part across its width
FOOT_FRICTION = (0.30, 0.25)  # the plastic underside on the bench (static, dynamic): the camera slides under about 0.8 N
# the slot's lower lip: the tunnel's floor runs LIP past the front face, under the shield's ridge (estimate)
LIP, LIP_W = 0.006, 0.050  # m out of the front face, m across (narrower than the 53.9 mm print)
# the film shield, as modelled: a thin rigid flap hinged at the slot's upper lip (where the front face meets the slot's
# roof) that lies over the ejected print; a rod across its underside (the ridge) meets the print's top along one line
# over the lower lip, so the spring's torque about the hinge over the ridge's distance from it is the force pressing the
# print onto the lip. Once lifted past a small angle its spring rolls it back up (the task sets the spring's rest
# angle). The real shield is a rolled film and ours a rigid flap (README: fidelity). Estimates, all of them.
SHIELD_L, SHIELD_W, SHIELD_T = 0.018, 0.076, 0.001  # m: out over the print, across (11 mm past the print either side), thick
PLATE_GAP = 0.001  # m: flat, the flap's underside stands this far over a print's top (only the rod touches it)
RIDGE_AT, RIDGE_R = 0.0045, 0.0005  # m: the rod's axis from the hinge (over the lip), its radius (from the print to the flap)
SHIELD_MASS = 0.001  # kg
SHIELD_SPRING, SHIELD_DAMPING = 0.02, 0.002  # N m/rad, N m s/rad: the task sets the spring's rest angle
SHIELD_TRAVEL = (math.radians(-3.0), math.radians(45.0))  # rad, lifted-up positive (the joint's own positive is the
# free edge down: its stops are these negated): 3 degrees under flat (it follows a print lying low) to rolled back 45
SHIELD_FRICTION = 1.0  # the ridge on the print's top (static and dynamic alike: the task sets the hold and the drag through the spring)
FLOOR_FRICTION = 0.3  # the tunnel's plastic floor, its lower lip and its side walls on the print (static and dynamic alike)
# the rollers' nip, closed behind the print once it is out (the task places the print until then, so the nip's rollers
# are not modelled as bodies): a block across the tunnel just behind the print's inner end, drawn up in the roof (open)
INNER = 0.010  # m: the print's inner end stays this far inside the front face once it is out (estimate)
NIP_X, NIP_GAP = 0.002, 0.0005  # m along the tunnel, m between it and the print's inner end
NIP_MASS = 0.002
NIP_SPRING, NIP_DAMPING = 200.0, 1.3  # N/m, N s/m (about critical for 2 g); the task sets the spring's rest point
SHELL_FRICTION = (1.2, 1.0)  # the rest of the body and the button, where hands hold and press it: the scene's rubbery
# default for things that are handled (yam_arena's GRIPPY_FRICTION, averaged with the other surface). Every collider
# binds its own material here, so the simulator's spawner, which would bind that default over all of them, keeps them
GUIDE_T = 0.001  # the top layer of the tunnel's floor and the skin of its side walls (FLOOR_FRICTION)

PRINT = (0.0666, 0.0539, 0.0025)  # length, width (Polaroid's 66.6 x 53.9 mm), thickness (2.5 mm for stable contacts)
PRINT_MASS = 0.0035  # estimate: the pack's 35 g less a 7 g cassette (estimate), shared by its 8 sheets
PICTURE = (0.047, 0.046)  # Polaroid's image area; the wide border is at the end that leaves the camera last
BORDER_FRONT = 0.005
HIDDEN_X = DEPTH / 2 - TUNNEL[0] + 0.002 + PRINT[0] / 2  # print centre (camera frame) while inside: 2 mm from the tunnel's end
WHITE, BLACK, GREY, RED = (236, 236, 232), (20, 20, 22), (120, 122, 128), (214, 40, 36)


def layout() -> dict:
    front = DEPTH / 2
    print_top, plate_lo = flap_heights()
    return {
        "camera_model": CAMERA,
        "camera": {"size": (DEPTH, WIDTH, HEIGHT + SHUTTER_H), "top": HEIGHT, "mass": MASS, "front": front, "grip": GRIP,
                   "shutter": (*SHUTTER, HEIGHT + SHUTTER_H), "shutter_radius": SHUTTER_R, "shutter_travel": SHUTTER_TRAVEL,
                   "shutter_spring": SHUTTER_SPRING, "shutter_damping": SHUTTER_DAMPING, "lens": (front + LENS[1], 0.0, LENS[2]),
                   "lens_radius": LENS[0], "foot_friction": FOOT_FRICTION},
        "tunnel": {"floor": FLOOR, "gap": GAP, "depth": TUNNEL[0], "width": TUNNEL[1], "lip": LIP, "lip_width": LIP_W},
        "shield": {"hinge": (front, 0.0, FLOOR + GAP), "length": SHIELD_L, "thickness": SHIELD_T, "width": SHIELD_W,
                   "plate": (round(plate_lo, 6), round(plate_lo + SHIELD_T, 6)), "ridge": RIDGE_AT,
                   "ridge_rise": round(-print_top, 6), "com": (SHIELD_L / 2, 0.0, round(plate_lo + SHIELD_T / 2, 6)),
                   "mass": SHIELD_MASS, "spring": SHIELD_SPRING,
                   "damping": SHIELD_DAMPING, "travel": [round(a, 6) for a in SHIELD_TRAVEL], "friction": (SHIELD_FRICTION, FLOOR_FRICTION)},
        "rollers": {"x": (round(front - INNER - NIP_GAP - NIP_X, 5), round(front - INNER - NIP_GAP, 5)), "travel": (-GAP, 0.0),
                    "mass": NIP_MASS, "spring": NIP_SPRING, "damping": NIP_DAMPING},
        "print": {"size": PRINT, "mass": PRINT_MASS, "hidden_x": round(HIDDEN_X, 5), "inner": INNER},
        "assets": {"camera": "h07_camera", "print": "h07_print"},
        "extent": {"camera": DEPTH, "print": PRINT[0]},
    }


def flap_heights() -> tuple[float, float]:
    """From the shield's hinge (the slot's upper lip), flat: a print's top on the floor (the rod's line of contact) and
    the flap's underside."""
    print_top = FLOOR + PRINT[2] - (FLOOR + GAP)
    return print_top, print_top + PLATE_GAP


# --- USD text -----------------------------------------------------------------------------------------------------
ROOT_NAME = [""]  # the model being written (material paths are absolute inside the file)
INVISIBLE = '        token visibility = "invisible"'


def f(v) -> str:
    return f"{v:.6g}"


def vec(p) -> str:
    return "(" + ", ".join(f(v) for v in p) + ")"


def srgb(r, g, b):
    return tuple((c / 255 / 12.92) if c / 255 <= 0.04045 else ((c / 255 + 0.055) / 1.055) ** 2.4 for c in (r, g, b))


def rounded_rect(w, h, r, n=6):
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


def prism(name, outline, lo, hi, centre_xy, material) -> str:
    """An outline (in x, y about ``centre_xy``) swept from z ``lo`` to ``hi``: sides and two end caps."""
    n = len(outline)
    cx, cy = centre_xy
    pts = [(cx + a, cy + b, lo) for a, b in outline] + [(cx + a, cy + b, hi) for a, b in outline]
    centre = (cx, cy, (lo + hi) / 2)
    faces = [(i, (i + 1) % n, n + (i + 1) % n, n + i) for i in range(n)] + [tuple(range(n)), tuple(range(n, 2 * n))]
    return mesh(name, pts, faces, material, outward_from=centre)


def quad(name, centre, up, right, material, extra="") -> str:
    """A printed rectangle: ``up`` and ``right`` are the image's half-extent vectors from ``centre``."""
    c, u, r = centre, up, right
    p = lambda su, sr: tuple(c[a] + su * u[a] + sr * r[a] for a in range(3))  # noqa: E731
    pts = [p(-1, -1), p(-1, 1), p(1, 1), p(1, -1)]
    return mesh(name, pts, [(0, 1, 2, 3)], material, [[(0, 0), (1, 0), (1, 1), (0, 1)]], extra=extra)


def cylinder(name, radius, height, centre, material, axis="Z", collide=False, physics_material=None) -> str:
    api = ' (\n        prepend apiSchemas = ["PhysicsCollisionAPI"]\n    )' if collide else ""
    half = {"X": (height / 2, radius, radius), "Y": (radius, height / 2, radius), "Z": (radius, radius, height / 2)}[axis]
    return "\n".join([
        f'    def Cylinder "{name}"{api}',
        "    {",
        f'        uniform token axis = "{axis}"',
        f"        double height = {f(height)}",
        f"        double radius = {f(radius)}",
        f"        float3[] extent = [{vec(tuple(-v for v in half))}, {vec(half)}]",
        f"        double3 xformOp:translate = {vec(centre)}",
        '        uniform token[] xformOpOrder = ["xformOp:translate"]',
        f"        rel material:binding = </{ROOT_NAME[0]}/Looks/{material}>",
        *([f"        rel material:binding:physics = </{ROOT_NAME[0]}/Looks/{physics_material}>"] if physics_material else []),
        "    }",
    ])


def collider(name, lo, hi, physics_material=None) -> str:
    """An invisible box collider from corner ``lo`` to corner ``hi``."""
    size = tuple(b - a for a, b in zip(lo, hi))
    centre = tuple((a + b) / 2 for a, b in zip(lo, hi))
    out = [
        f'    def Cube "{name}" (',
        '        prepend apiSchemas = ["PhysicsCollisionAPI"]',
        "    )",
        "    {",
        "        double size = 1",
        "        float3[] extent = [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]",
        f"        double3 xformOp:translate = {vec(centre)}",
        f"        float3 xformOp:scale = {vec(size)}",
        '        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]',
        '        token visibility = "invisible"',
    ]
    if physics_material:
        out.append(f"        rel material:binding:physics = </{ROOT_NAME[0]}/Looks/{physics_material}>")
    out.append("    }")
    return "\n".join(out)


def material(name, *, color, roughness=0.5) -> str:
    path = f"/{ROOT_NAME[0]}/Looks/{name}"
    return "\n".join([
        f'        def Material "{name}"', "        {", f"            token outputs:surface.connect = <{path}/Shader.outputs:surface>",
        '            def Shader "Shader"', "            {", '                uniform token info:id = "UsdPreviewSurface"',
        f"                color3f inputs:diffuseColor = {vec(color)}",
        f"                float inputs:roughness = {f(roughness)}", "                float inputs:metallic = 0", "                token outputs:surface", "            }",
        "        }",
    ])


def physics_material(name, static, dynamic, combine="min") -> str:
    return "\n".join([
        f'        def Material "{name}" (',
        '            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]',
        "        )",
        "        {",
        f"            float physics:staticFriction = {f(static)}",
        f"            float physics:dynamicFriction = {f(dynamic)}",
        "            float physics:restitution = 0",
        f'            uniform token physxMaterial:frictionCombineMode = "{combine}"',
        "        }",
    ])


def model(name, doc, body, looks) -> str:
    return "\n".join([
        "#usda 1.0", "(", f'    defaultPrim = "{name}"', "    metersPerUnit = 1", '    upAxis = "Z"', f'    doc = "{doc}"', ")", "",
        f'def Xform "{name}" (', '    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]', ")", "{",
        *body, '    def Scope "Looks"', "    {", *looks, "    }", "}", "",
    ])


def link(name, mass, at, parts, com=None) -> list[str]:
    """A moving link: its parts drawn about ``at`` (body frame); ``com``: its centre of mass (link frame), else the
    simulator's from its colliders."""
    return [f'    def Xform "{name}" (', '        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]', "    )", "    {",
            f"        float physics:mass = {f(mass)}", *([f"        point3f physics:centerOfMass = {vec(com)}"] if com else []),
            f"        double3 xformOp:translate = {vec(at)}", '        uniform token[] xformOpOrder = ["xformOp:translate"]', *parts, "    }"]


def capsule(name, radius, length, centre, physics_material) -> str:
    """An invisible capsule collider along y: a rod of ``radius`` whose straight part is ``length`` long."""
    return "\n".join([
        f'    def Capsule "{name}" (', '        prepend apiSchemas = ["PhysicsCollisionAPI"]', "    )", "    {",
        '        uniform token axis = "Y"', f"        double height = {f(length)}", f"        double radius = {f(radius)}",
        f"        float3[] extent = [{vec((-radius, -length / 2 - radius, -radius))}, {vec((radius, length / 2 + radius, radius))}]",
        f"        double3 xformOp:translate = {vec(centre)}", '        uniform token[] xformOpOrder = ["xformOp:translate"]',
        INVISIBLE, f"        rel material:binding:physics = </{ROOT_NAME[0]}/Looks/{physics_material}>", "    }",
    ])


def slide_joint(model_name, name, child, at, axis, lower, upper, spring, damping) -> list[str]:
    """A prismatic joint from the body to ``child`` along ``axis`` of the body frame, from ``lower`` to ``upper`` (m from
    where the child is drawn), held at 0 by ``spring`` N/m with ``damping`` N s/m (the task's actuator carries the same
    values: Isaac Lab writes its gains over the file's drive)."""
    return [f'    def PhysicsPrismaticJoint "{name}" (', '        prepend apiSchemas = ["PhysicsDriveAPI:linear"]', "    )", "    {",
            f"        rel physics:body0 = </{model_name}/body>", f"        rel physics:body1 = </{model_name}/{child}>",
            f'        uniform token physics:axis = "{axis}"', f"        float physics:lowerLimit = {f(lower)}", f"        float physics:upperLimit = {f(upper)}",
            f"        point3f physics:localPos0 = {vec(at)}", "        quatf physics:localRot0 = (1, 0, 0, 0)",
            "        point3f physics:localPos1 = (0, 0, 0)", "        quatf physics:localRot1 = (1, 0, 0, 0)",
            '        uniform token drive:linear:physics:type = "force"', f"        float drive:linear:physics:stiffness = {f(spring)}",
            f"        float drive:linear:physics:damping = {f(damping)}", "        float drive:linear:physics:targetPosition = 0", "    }"]


def hinge_joint(model_name, name, child, at, axis, lower, upper, spring, damping) -> list[str]:
    """A revolute joint from the body to ``child`` about ``axis`` through ``at`` (body frame), from ``lower`` to
    ``upper`` (rad), held at 0 by ``spring`` N m/rad with ``damping`` N m s/rad (the task's actuator carries the same
    values). USD writes revolute limits and drive targets in degrees and angular gains per degree."""
    return [f'    def PhysicsRevoluteJoint "{name}" (', '        prepend apiSchemas = ["PhysicsDriveAPI:angular"]', "    )", "    {",
            f"        rel physics:body0 = </{model_name}/body>", f"        rel physics:body1 = </{model_name}/{child}>",
            f'        uniform token physics:axis = "{axis}"', f"        float physics:lowerLimit = {f(math.degrees(lower))}",
            f"        float physics:upperLimit = {f(math.degrees(upper))}", f"        point3f physics:localPos0 = {vec(at)}",
            "        quatf physics:localRot0 = (1, 0, 0, 0)", "        point3f physics:localPos1 = (0, 0, 0)",
            "        quatf physics:localRot1 = (1, 0, 0, 0)", '        uniform token drive:angular:physics:type = "force"',
            f"        float drive:angular:physics:stiffness = {f(spring * math.pi / 180)}",
            f"        float drive:angular:physics:damping = {f(damping * math.pi / 180)}",
            "        float drive:angular:physics:targetPosition = 0", "    }"]


def articulated_model(name, doc, body, body_mass, moving, looks) -> str:
    """A free articulation: the link ``body`` (``body_mass``; the articulation's root) and the moving links of
    ``moving``, each (link name, mass, where it is drawn in the body frame, its parts, joint name, kind, axis, lower,
    upper, spring, damping, centre of mass or None) on a ``slide`` (prismatic) or ``hinge`` (revolute) joint. Links do
    not collide with each other (the button sinks into its recess in the body; the nip block rests in the roof)."""
    lines = ["#usda 1.0", "(", f'    defaultPrim = "{name}"', "    metersPerUnit = 1", '    upAxis = "Z"', f'    doc = "{doc}"', ")", "",
             f'def Xform "{name}"', "{",
             '    def Xform "body" (',
             '        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysicsArticulationRootAPI", "PhysxArticulationAPI"]',
             "    )", "    {", f"        float physics:mass = {f(body_mass)}", "        bool physxArticulation:enabledSelfCollisions = 0",
             *body, "    }"]
    for child, mass, at, parts, joint, kind, axis, lower, upper, spring, damping, com in moving:
        lines += link(child, mass, at, parts, com)
        if kind == "hinge":
            lines += hinge_joint(name, joint, child, at, axis, lower, upper, spring, damping)
        else:
            lines += slide_joint(name, joint, child, at, axis, lower, upper, spring, damping)
    lines += ['    def Scope "Looks"', "    {", *looks, "    }", "}", ""]
    return "\n".join(lines)


def lens_tip() -> float:
    """The front of the lens's glass, past the front face (the model's foremost point)."""
    return DEPTH / 2 + LENS[1] + 0.0012


def box() -> tuple[tuple, tuple]:
    """The camera's local bounds (everything drawn at rest): the flap lies flat over the print and stands out of the
    front past the lens; the button stands out of the top."""
    return (-DEPTH / 2 - 0.0004, -WIDTH / 2, 0.0), (max(lens_tip(), DEPTH / 2 + SHIELD_L), WIDTH / 2, HEIGHT + SHUTTER_H)


def build_camera() -> str:
    ROOT_NAME[0] = "camera"
    front, half_w = DEPTH / 2, WIDTH / 2
    tw = TUNNEL[1] / 2
    back = front - TUNNEL[0]
    roof = FLOOR + GAP
    eps = 0.0004
    lens_r, lens_len, lens_z = LENS
    body = [
        prism("shell", rounded_rect(DEPTH, WIDTH, CORNER), 0.0, HEIGHT, (0.0, 0.0), "shell"),
        quad("slot", (front + eps, 0.0, FLOOR + GAP / 2), (0, 0, GAP / 2 + 0.001), (0, tw + 0.002, 0), "dark"),
        cylinder("lens_barrel", lens_r, lens_len, (front + lens_len / 2, 0.0, lens_z), "dark", axis="X"),
        cylinder("lens_ring", lens_r - 0.004, 0.001, (front + lens_len + 0.0005, 0.0, lens_z), "ring", axis="X"),
        cylinder("lens_glass", lens_r - 0.007, 0.001, (front + lens_len + 0.0007, 0.0, lens_z), "glass", axis="X"),
        quad("flash", (front + eps, -0.024, HEIGHT - 0.012), (0, 0, 0.005), (0, 0.010, 0), "flash"),  # drawn only (estimate)
        quad("finder_front", (front + eps, 0.026, HEIGHT - 0.012), (0, 0, 0.005), (0, 0.006, 0), "dark"),
        quad("finder_back", (-front - eps, 0.026, HEIGHT - 0.012), (0, 0, 0.005), (0, -0.006, 0), "dark"),
        cylinder("flash_button", 0.004, 0.001, (SHUTTER[0] - 0.020, SHUTTER[1], HEIGHT + 0.0005), "dark"),  # drawn only
        prism("lip", [(LIP / 2, -LIP_W / 2), (LIP / 2, LIP_W / 2), (-LIP / 2, LIP_W / 2), (-LIP / 2, -LIP_W / 2)], FLOOR - GUIDE_T, FLOOR,
              (front + LIP / 2, 0.0), "shell"),  # the slot's lower lip
        # colliders: under the tunnel the feet and the tunnel's floor, over it the rest of the body; the tunnel's sides
        # (their inner skin plastic like the floor) and end; the lens
        collider("c_feet", (-front, -half_w, 0.0), (front, half_w, FLOOR - GUIDE_T), "feet"),
        collider("c_guide", (back, -tw, FLOOR - GUIDE_T), (front, tw, FLOOR), "plastic"),
        collider("c_lip", (front, -LIP_W / 2, FLOOR - GUIDE_T), (front + LIP, LIP_W / 2, FLOOR), "plastic"),
        collider("c_upper", (-front, -half_w, roof), (front, half_w, HEIGHT), "grip"),
        collider("c_left", (-front, tw, FLOOR - GUIDE_T), (front, half_w, roof), "grip"),
        collider("c_right", (-front, -half_w, FLOOR - GUIDE_T), (front, -tw, roof), "grip"),
        collider("c_left_skin", (back, tw, FLOOR), (front, tw + GUIDE_T, roof), "plastic"),
        collider("c_right_skin", (back, -tw - GUIDE_T, FLOOR), (front, -tw, roof), "plastic"),
        collider("c_back", (-front, -tw, FLOOR - GUIDE_T), (back, tw, roof), "grip"),
        collider("c_lens", (front, -lens_r, lens_z - lens_r), (front + lens_len, lens_r, lens_z + lens_r), "grip"),
    ]
    button = [cylinder("cap", SHUTTER_R, SHUTTER_H, (0.0, 0.0, 0.0), "red", collide=True, physics_material="grip")]
    shield_at = (front, 0.0, roof)  # the flap's hinge: the slot's upper lip, in the body frame
    print_top, plate_lo = flap_heights()  # from the hinge: a print's top (-0.5 mm), the flap's underside (+0.5 mm)
    rect = lambda a, b: [(a / 2, -b / 2), (a / 2, b / 2), (-a / 2, b / 2), (-a / 2, -b / 2)]  # noqa: E731
    shield = [
        prism("flap_drawn", rect(SHIELD_L, SHIELD_W), plate_lo, plate_lo + SHIELD_T, (SHIELD_L / 2, 0.0), "shield"),
        collider("c_flap", (0.0, -SHIELD_W / 2, plate_lo), (SHIELD_L, SHIELD_W / 2, plate_lo + SHIELD_T), "plastic"),
        cylinder("ridge_drawn", RIDGE_R, PRINT[1], (RIDGE_AT, 0.0, print_top + RIDGE_R), "shield", axis="Y"),
        capsule("c_ridge", RIDGE_R, PRINT[1] - 2 * RIDGE_R, (RIDGE_AT, 0.0, print_top + RIDGE_R), "nip"),  # flat: on a print's top
    ]
    nip_at = (front - INNER - NIP_GAP - NIP_X / 2, 0.0, roof + GAP / 2)  # drawn open, up in the roof; closed it is GAP lower
    nip = [collider("c_nip", (-NIP_X / 2, -tw + 0.001, -GAP / 2), (NIP_X / 2, tw - 0.001, GAP / 2), "plastic")]
    looks = [
        material("shell", color=srgb(*WHITE), roughness=0.5),
        material("dark", color=srgb(*BLACK), roughness=0.5),
        material("ring", color=srgb(*GREY), roughness=0.3),
        material("glass", color=srgb(18, 26, 44), roughness=0.05),
        material("flash", color=srgb(206, 210, 214), roughness=0.2),
        material("red", color=srgb(*RED), roughness=0.4),
        material("shield", color=srgb(52, 52, 56), roughness=0.5),
        physics_material("feet", *FOOT_FRICTION),
        physics_material("grip", *SHELL_FRICTION, combine="average"),
        physics_material("plastic", FLOOR_FRICTION, FLOOR_FRICTION),
        physics_material("nip", SHIELD_FRICTION, SHIELD_FRICTION),
    ]
    doc = ("A Polaroid Go Generation 2 instant camera as modelled (outer size from the maker's page, no name or logo): a body "
           "with the lens in front, a tunnel along its floor that ends in the slot in the front face, the film shield as a "
           "thin flap over the slot hinged at its upper lip, the rollers' nip as a block that closes behind the ejected print, "
           "and the red shutter button on the top on a sprung prismatic joint")
    moving = [
        ("button", BUTTON_MASS, (*SHUTTER, HEIGHT + SHUTTER_H / 2), button, "shutter", "slide", "Z", -SHUTTER_TRAVEL, 0.0, SHUTTER_SPRING,
         SHUTTER_DAMPING, None),
        ("flap", SHIELD_MASS, shield_at, shield, "shield", "hinge", "Y", -SHIELD_TRAVEL[1], -SHIELD_TRAVEL[0], SHIELD_SPRING, SHIELD_DAMPING,
         (SHIELD_L / 2, 0.0, plate_lo + SHIELD_T / 2)),
        ("nip", NIP_MASS, nip_at, nip, "rollers", "slide", "Z", -GAP, 0.0, NIP_SPRING, NIP_DAMPING, None),
    ]
    return articulated_model("camera", doc, body, MASS - BUTTON_MASS - SHIELD_MASS - NIP_MASS, moving, looks)


def build_print() -> str:
    ROOT_NAME[0] = "print"
    length, width, t = PRINT
    pic_x = length / 2 - BORDER_FRONT - PICTURE[0] / 2  # the picture area's middle: the wide border at the -x end
    body = [
        prism("card", rounded_rect(length, width, 0.002, n=3), 0.0, t, (0.0, 0.0), "paper"),
        quad("back", (0.0, 0.0, -0.0002), (length / 2, 0, 0), (0, width / 2, 0), "back"),
        quad("picture", (pic_x, 0.0, t + 0.0002), (PICTURE[0] / 2, 0, 0), (0, -PICTURE[1] / 2, 0), "picture"),
        collider("c_card", (-length / 2, -width / 2, 0.0), (length / 2, width / 2, t)),
    ]
    looks = [
        material("paper", color=srgb(246, 245, 240), roughness=0.6),
        material("back", color=srgb(24, 24, 26), roughness=0.5),
        material("picture", color=srgb(58, 70, 84), roughness=0.3),  # a fresh print's undeveloped picture: dark slate
    ]
    return model("print", "An instant print of Go film size: white card, black back, the picture area (on top) still undeveloped.", body, looks)


def bounds_entry(file, low, high) -> dict:
    size = [round(b - a, 4) for a, b in zip(low, high)]
    return {"extent": max(size[:2]), "file": file, "height": size[2], "high": [round(v, 4) for v in high], "low": [round(v, 4) for v in low], "size": size}


def main() -> None:
    lo, hi = box()
    out = layout()
    out["camera"]["box_centre"] = [round((a + b) / 2, 5) for a, b in zip(lo, hi)]  # where the simulator's centroid sits, camera frame
    out["extent"]["camera"] = round(hi[0] - lo[0], 5)  # the widest horizontal size, lens included: drawn at scale 1
    (HERE / "layout.json").write_text(json.dumps(out, indent=1) + "\n")
    (HERE / "camera.usda").write_text(build_camera())
    (HERE / "print.usda").write_text(build_print())
    bounds_file = ROOT / "docs" / "assets" / "bounds.json"
    bounds = json.loads(bounds_file.read_text())
    bounds["h07_camera"] = bounds_entry("camera.usda", lo, hi)
    bounds["h07_print"] = bounds_entry("print.usda", (-PRINT[0] / 2, -PRINT[1] / 2, 0.0), (PRINT[0] / 2, PRINT[1] / 2, PRINT[2]))
    bounds_file.write_text(json.dumps(dict(sorted(bounds.items())), indent=1) + "\n")
    print("wrote camera.usda (an articulation: body, shutter button, film shield, rollers' nip), print.usda, layout.json; bounds for 2 models")


if __name__ == "__main__":
    main()
