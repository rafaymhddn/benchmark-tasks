"""The instant camera and its print as the task reads them: their figures from ``assets/layout.json`` (written by
``assets/build.py``) and where things are in the camera's frame (x out of the lens, y to its left, z up, origin on the
bench under its centre)."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from rta.spec import quat_matrix

L = json.loads((Path(__file__).resolve().parent / "assets" / "layout.json").read_text())
C = L["camera"]
CAM_SIZE = tuple(C["size"])  # depth, width, height up to the button's top
FRONT = C["front"]  # the front face, where the slot is
BOX_C = np.asarray(C["box_centre"], float)  # the simulator's centroid of the model (the lens stands out in front)
GRIP = tuple(C["grip"])  # where a hand from above holds the camera's rear part across its width
SHUTTER = tuple(C["shutter"])  # the button's top centre at rest
SHUTTER_TRAVEL = C["shutter_travel"]
LENS = tuple(C["lens"])  # the lens's front centre
T = L["tunnel"]
FLOOR = T["floor"]
S = L["shield"]
SHIELD_L, SHIELD_W, SHIELD_T = S["length"], S["width"], S["thickness"]  # the film shield's flap: out over the print, across, thick
SHIELD_HINGE = tuple(S["hinge"])  # the flap's hinge: the slot's upper lip (the front face, the slot's roof)
SHIELD_UNDER = SHIELD_HINGE[2] + S["plate"][0]  # the flap's underside, flat (camera frame)
P = L["print"]
PRINT = tuple(P["size"])  # length (along the slot), width, thickness
HIDDEN_X = P["hidden_x"]  # the print's centre in the camera's frame while it waits inside
PRINT_Z = FLOOR  # the print's underside in the camera's frame while it is in the tunnel: on the tunnel's floor


def frame(cam):
    """The camera's rotation (columns: lens axis, left, up) and its origin (on its underside, under its centre)."""
    rot = quat_matrix(cam.quat_xyzw)
    return rot, np.asarray(cam.pos, float) - rot @ BOX_C


def to_world(cam, local) -> np.ndarray:
    rot, origin = frame(cam)
    return origin + rot @ np.asarray(local, float)


def to_local(cam, world) -> np.ndarray:
    rot, origin = frame(cam)
    return rot.T @ (np.asarray(world, float) - origin)


def tilt_deg(o) -> float:
    return math.degrees(math.acos(float(np.clip(quat_matrix(o.quat_xyzw)[2, 2], -1.0, 1.0))))


def box_distance(o, p) -> float:
    p = np.asarray(p, float)
    return float(np.linalg.norm(p - np.clip(p, o.low, o.high)))


def shutter_in(objects) -> float:
    """How far (m) the shutter button is pressed in from rest: the camera's own joint."""
    return -float(objects.joints.get("camera", {}).get("shutter", 0.0))


def shield_angle(objects) -> float:
    """The film shield's angle (rad; 0 flat over the print, positive lifted up towards rolled back). The joint's own
    positive about +y is the free edge down, so it is negated here."""
    return -float(objects.joints.get("camera", {}).get("shield", 0.0))


def shield_gravity(objects) -> float:
    """The flap's own weight as a torque about its hinge (N m; positive presses its free edge down, the joint's own
    sense), with the camera turned as it is now and the flap at its angle."""
    q = -shield_angle(objects)
    c, s = math.cos(q), math.sin(q)
    x, y, z = S["com"]
    r = np.array([c * x + s * z, y, -s * x + c * z])  # its centre of mass from the hinge, camera frame
    g = quat_matrix(objects["camera"].quat_xyzw).T @ np.array([0.0, 0.0, -S["mass"] * 9.81])
    return float(np.cross(r, g)[1])


def aim(objects) -> tuple[float, float]:
    """(angle in degrees between the lens axis and the apple, distance from the lens to the apple), on the bench plane."""
    cam, apple = objects["camera"], objects["apple"]
    rot, _ = frame(cam)
    v = np.asarray(apple.pos, float)[:2] - to_world(cam, LENS)[:2]
    dist = float(np.linalg.norm(v))
    fwd = rot[:2, 0] / max(float(np.linalg.norm(rot[:2, 0])), 1e-9)
    return math.degrees(math.acos(float(np.clip(fwd @ v / max(dist, 1e-9), -1.0, 1.0)))), dist


def print_in_camera(objects) -> np.ndarray:
    """The print's centre in the camera's frame."""
    return to_local(objects["camera"], objects["print"].pos)


def picture_up(pr) -> float:
    """Cosine of the angle between the picture side (the print's top while it is in the camera) and straight up."""
    return float(quat_matrix(pr.quat_xyzw)[2, 2])
