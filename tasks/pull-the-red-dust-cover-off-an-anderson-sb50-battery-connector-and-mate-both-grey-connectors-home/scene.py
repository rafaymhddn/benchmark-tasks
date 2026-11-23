"""Bodies and joints of the SB50 mating task.

- ``mat``: the flat 140 x 110 x 4 mm yellow mat (fixed). ``clamp_l`` / ``clamp_r``: where each lead is clamped to
  the bench (fixed). ``block``: a 20 mm foam block under the right housing only (fixed), its leads free behind it.
- ``sb_left`` / ``sb_right``: one grey SB50 housing each, with its red T-handle on the '+' face and its two 6 AWG
  leads (stiff for 66 mm behind the housing), each lead jointed in four links (``cable_<l|r>_<k>``) to its clamp.
  Both start '+' face up with the noses towards each other, the right one on its block. The nose is a lower tongue
  between two upper side walls: two housings fit together only when one is turned over, and slide home to 81.0 mm
  rear face to rear face, where each tongue meets the other housing's face. Turned the same way up, tongue meets
  tongue at 95.4 mm.
- ``dust_cover``: the red 134G1 cover, a hollow box over the left nose, held to ``cap_rail`` by the weld ``w_cap``;
  ``cap_rail`` slides along the housing with 10 N of friction (``cap_slide``). The task cuts ``w_cap`` once the cover
  has slid 16 mm, off the nose. A four-link lanyard (``strap_<k>``) ties the cover to the left leads.
- ``mate_rail``: a slide on the left housing's axis (``mate_wipe``); the weld ``w_mate`` (``sb_right`` to
  ``mate_rail``) is made by the task where the housings stand once their noses overlap, and the slide's
  friction-like spring then carries the 44 N mating force between the two housings.
"""

from __future__ import annotations

import math

import numpy as np

import mechkit as mk
import parts as P
from mechkit import body, box, capsule, cylinder, hinge, slide, sphere, weld

MECHKIT_HELPER = True

GREY = (0.52, 0.54, 0.57)
WHITE = (0.96, 0.96, 0.96)
DARK = (0.20, 0.21, 0.23)
RED = (0.80, 0.12, 0.10)
COVER_RED = (0.86, 0.17, 0.13)
YELLOW = (0.95, 0.80, 0.12)
BLACK = (0.12, 0.12, 0.13)
STEEL = (0.36, 0.38, 0.41)
BLOCK_GREY = (0.30, 0.31, 0.33)

REAR = P.SB_L - P.NOSE_L  # 33.3 mm of full-height body behind the nose
NOSE_END = P.SB_L - P.NOSE_SHORT  # 47.7 mm
CONN_INERTIA = (1.6e-5, 6.5e-5, 6.5e-5)  # kg m^2 about the connector's own axes through CONN_CENTER (estimate)
CONN_CENTER = (0.0056, 0.0, 0.0025)  # m: housing and contacts at +20 mm, handle at +5, leads at -33 (estimate)
AUX_INERTIA = (3.0e-6, 3.0e-6, 3.0e-6)


def _f(v) -> tuple[float, ...]:
    return tuple(float(x) for x in v)


def _quat(mat: np.ndarray) -> tuple[float, ...]:
    return _f(mk.matrix_to_quat(mat))


def connector_shapes() -> list:
    """One SB50 housing with its handle and stiff leads, in the connector frame (see parts.py)."""
    h, w, r, rw = P.SB_H, P.SB_W, P.LEAD_IN_R, P.WALL_W / 2.0
    tl = NOSE_END - REAR  # 14.4 mm
    zt, zw = -h / 2 + P.TONGUE_H / 2, h / 2 - P.TONGUE_H / 2  # middles of the lower tongue and the upper walls
    poly = (0.5, 0.45)
    nose = (0.2, 0.2)  # the nose slides in the other nose and in the cover; the mating force is the slide's (task.py)
    shapes = [
        box((REAR, w, h), pos=(REAR / 2, 0.0, 0.0), color=GREY, friction=poly),
        # lower tongue, its two front corners rounded (the lead-in)
        box((tl - r, P.TONGUE_W, P.TONGUE_H), pos=(REAR + (tl - r) / 2, 0.0, zt), color=GREY, friction=nose, friction_combine="min"),
        box((r, P.TONGUE_W - 2 * r, P.TONGUE_H), pos=(NOSE_END - r / 2, 0.0, zt), color=GREY, friction=nose, friction_combine="min"),
        cylinder(r, P.TONGUE_H, pos=(NOSE_END - r, P.TONGUE_W / 2 - r, zt), color=GREY, friction=nose, friction_combine="min"),
        cylinder(r, P.TONGUE_H, pos=(NOSE_END - r, -(P.TONGUE_W / 2 - r), zt), color=GREY, friction=nose, friction_combine="min"),
    ]
    for s in (1.0, -1.0):  # upper side walls, rounded fronts
        y = s * (w / 2 - rw)
        shapes += [
            box((tl - rw, P.WALL_W, P.TONGUE_H), pos=(REAR + (tl - rw) / 2, y, zw), color=GREY, friction=nose, friction_combine="min"),
            cylinder(rw, P.TONGUE_H, pos=(NOSE_END - rw, y, zw), color=GREY, friction=nose, friction_combine="min"),
        ]
    front = P.HDL_L - P.HDL_PROJ  # 31.6 mm: the handle's front end over the housing
    neck = P.HDL_PROJ - P.HDL_CROSS_D  # 15.1 mm from the rear face to the crossbar
    shapes += [
        # red T-handle (SB50-HDL-RED) on the '+' face: stem over the housing, stem behind it, crossbar
        box((front, P.HDL_STEM_W, P.HDL_FRONT_T), pos=(front / 2, 0.0, h / 2 + P.HDL_FRONT_T / 2), color=RED),
        box((neck, P.HDL_STEM_W, P.HDL_T), pos=(-neck / 2, 0.0, h / 2 + P.HDL_T / 2), color=RED),
        box((P.HDL_CROSS_D, P.HDL_W, P.HDL_T), pos=(-P.HDL_PROJ + P.HDL_CROSS_D / 2, 0.0, h / 2 + P.HDL_T / 2), color=RED),
        # the two 6 AWG leads, stiff for 66 mm behind the housing
        box((P.STUB_L, P.LEAD_W, P.WIRE_OD), pos=(-P.STUB_L / 2, 0.0, 0.0), color=BLACK, friction=mk.GRIPPY),
        # drawn only: '+' (white) and '-' (dark) on the handle's face, the two silver contact blades in the nose
        box((0.0075, 0.0018, 0.0006), pos=(0.040, 0.0095, h / 2 + 0.0003), color=WHITE, solid=False),
        box((0.0018, 0.0075, 0.0006), pos=(0.040, 0.0095, h / 2 + 0.0003), color=WHITE, solid=False),
        box((0.0075, 0.0018, 0.0006), pos=(0.040, -0.0095, h / 2 + 0.0003), color=DARK, solid=False),
        box((0.012, 0.007, 0.0012), pos=(REAR + 0.007, 0.0080, zt + P.TONGUE_H / 2 + 0.0006), color=(0.85, 0.87, 0.90), solid=False),
        box((0.012, 0.007, 0.0012), pos=(REAR + 0.007, -0.0080, zt + P.TONGUE_H / 2 + 0.0006), color=(0.85, 0.87, 0.90), solid=False),
    ]
    return shapes


def cover_shapes() -> list:
    """The 134G1 cover, a box 46.5 x 27.9 mm with a cavity the nose's size, its frame at its own middle and turned
    as the connector it covers."""
    head = P.CAP_ON + P.CAP_FRONT
    cx = head / 2  # the cover's middle, from its rear edge
    iw, ih = P.SB_W / 2 + P.CAP_GAP, P.SB_H / 2 + P.CAP_GAP  # cavity half sizes
    ow, oh = P.CAP_W / 2, P.CAP_H / 2
    x_cav = -cx + P.CAP_ON / 2
    return [
        box((P.CAP_ON, 2 * ow, oh - ih), pos=(x_cav, 0.0, (oh + ih) / 2), color=COVER_RED),
        box((P.CAP_ON, 2 * ow, oh - ih), pos=(x_cav, 0.0, -(oh + ih) / 2), color=COVER_RED),
        box((P.CAP_ON, ow - iw, 2 * ih), pos=(x_cav, (ow + iw) / 2, 0.0), color=COVER_RED),
        box((P.CAP_ON, ow - iw, 2 * ih), pos=(x_cav, -(ow + iw) / 2, 0.0), color=COVER_RED),
        box((P.CAP_FRONT, 2 * ow, 2 * oh), pos=(cx - P.CAP_FRONT / 2, 0.0, 0.0), color=COVER_RED),
    ]


def start_poses(top: float) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Where the connectors and the cover are built: (position, rotation) of each frame."""
    tilt = P.start_tilt()
    rot_l = mk.axis_angle((0.0, 0.0, 1.0), -math.pi / 2) @ mk.axis_angle((0.0, 1.0, 0.0), -tilt)  # nose to -y, tipped up
    rot_r = mk.axis_angle((0.0, 0.0, 1.0), +math.pi / 2)  # nose to +y, flat
    # the left one rests on the leads' far end and on the cover's rear bottom edge
    wall = (P.CAP_H - P.SB_H) / 2.0
    support = rot_l @ np.array([-P.STUB_L, 0.0, -P.WIRE_OD / 2])
    nose = rot_l @ np.array([P.SB_L, 0.0, 0.0])
    pos_l = np.array([P.CONN_X, P.LEFT_NOSE_Y - nose[1], top + P.OVER - support[2]])
    check = rot_l @ np.array([P.SB_L - P.CAP_ON, 0.0, -P.SB_H / 2 - wall]) + pos_l
    assert abs(check[2] - (top + P.OVER)) < 1e-6, "the left connector must rest on two points"
    pos_r = np.array([P.CONN_X, P.RIGHT_NOSE_Y - P.SB_L, top + P.BLOCK_H + P.OVER + P.SB_H / 2])
    cover = pos_l + rot_l @ np.array([P.SB_L - P.CAP_ON + (P.CAP_ON + P.CAP_FRONT) / 2, 0.0, 0.0])
    return {"sb_left": (pos_l, rot_l), "sb_right": (pos_r, rot_r), "dust_cover": (cover, rot_l)}


def _universal(prefix: str, child: str, parent: str, at: np.ndarray, axis_a, axis_b, passes, color, limits=(-2.6, 2.6)) -> None:
    """``child`` hung from ``parent`` at ``at`` by two hinges (a small pivot body between them)."""
    piv = f"{prefix}_piv"
    body(piv, [sphere(0.002, color=color, passes=tuple(n for n in passes if n != piv))], pos=_f(at), mass=0.004, inertia=AUX_INERTIA, color=color,
         linear_damping=0.3, angular_damping=0.3)
    hinge(f"{prefix}_a", piv, parent, anchor=_f(at), axis=_f(axis_a), limits=limits, damping=0.002)
    hinge(f"{prefix}_b", child, piv, anchor=_f(at), axis=_f(axis_b), limits=limits, damping=0.002)


def _segment_rot(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Rotation that takes z onto the direction from a to b."""
    u = (b - a) / np.linalg.norm(b - a)
    z = np.array([0.0, 0.0, 1.0])
    c = np.cross(z, u)
    s = float(np.linalg.norm(c))
    return np.eye(3) if s < 1e-9 else mk.axis_angle(c / s, math.atan2(s, float(z @ u)))


CABLE_LINKS = 4  # links per lead (even)


def cable_names(side: str) -> list[str]:
    return [f"cable_{side}_{k}" for k in range(CABLE_LINKS)] + [f"cable_{side}_{k}_piv" for k in range(CABLE_LINKS + 1)] + [f"cable_{side}_sw_piv"]


def _cable(side: str, conn: str, pos: np.ndarray, rot: np.ndarray, top: float) -> None:
    """The lead from the stiff stub's end to its bench clamp: a swivel (turns freely about the lead), CABLE_LINKS links
    jointed about two axes each, so that it bends round what pushes it (a hand turning the connector over)."""
    cx, cy = P.CLAMP_XY[side]
    clamp = f"clamp_{side}"
    body(clamp, [box((0.030, 0.024, 0.022), color=STEEL)], pos=(cx, cy, top + 0.011), color=STEEL)
    end = pos + rot @ np.array([-P.STUB_L, 0.0, 0.0])
    start = np.array([cx - 0.017, cy, top + P.CABLE_R + 0.002])
    seg = 0.5 * float(np.linalg.norm(start - end)) + P.CABLE_SLACK  # each link longer than half the span: the lead bows (28 cm in all)
    mid = 0.5 * (start + end)
    span = (end - start) / float(np.linalg.norm(end - start))
    out = np.array([-span[1], span[0], 0.0])
    out = out / np.linalg.norm(out) * (1.0 if out[1] > 0 else -1.0)  # both bow to +y: the right one clear of the hand behind it, the left one clear of the cover
    half = 0.5 * float(np.linalg.norm(start - end))
    elbow = mid + out * math.sqrt(max(seg**2 - half**2, 0.0))
    names = cable_names(side)
    passes = tuple(n for n in names) + ("sb_left", "sb_right", "dust_cover", clamp) + strap_names()
    parent = clamp
    half_links = CABLE_LINKS // 2  # each half of the bowed lead in equal straight links
    pts = [start + (elbow - start) * (k / half_links) for k in range(half_links)] + [elbow + (end - elbow) * (k / half_links) for k in range(half_links + 1)]
    for k in range(CABLE_LINKS):
        a, b = pts[k], pts[k + 1]
        r_seg = _segment_rot(a, b)
        link = f"cable_{side}_{k}"
        length = float(np.linalg.norm(b - a))
        body(link, [capsule(P.CABLE_R, max(0.01, length - 2 * P.CABLE_R), color=BLACK, passes=tuple(n for n in passes if n != link))],
             pos=_f(0.5 * (a + b)), quat=_quat(r_seg), mass=2.0 * P.CABLE_LINK_MASS / CABLE_LINKS, color=BLACK, linear_damping=0.4, angular_damping=0.4)
        u = (b - a) / length
        perp = np.cross(np.array([0.0, 0.0, 1.0]), u)
        perp = perp / np.linalg.norm(perp)
        _universal(f"cable_{side}_{k}", link, parent, a, (0.0, 0.0, 1.0), perp, passes, BLACK, limits=None)  # limp: links folded
        # at the kit's 149 deg stops made a stiff strut that held the laid pair 10 mm over the mat
        parent = link
    # the swivel at the stub's end: the connector turns freely about its leads, and bends about two axes
    axis = rot @ np.array([1.0, 0.0, 0.0])
    piv = f"cable_{side}_sw_piv"
    body(piv, [sphere(0.002, color=BLACK, passes=tuple(n for n in passes if n != piv))], pos=_f(end), mass=0.004,
         inertia=AUX_INERTIA, color=BLACK, linear_damping=0.3, angular_damping=0.3)
    _universal(f"cable_{side}_{CABLE_LINKS}", piv, parent, end, (0.0, 0.0, 1.0), np.cross((0.0, 0.0, 1.0), axis), passes, BLACK, limits=None)
    hinge(f"cable_{side}_roll", conn, piv, anchor=_f(end), axis=_f(axis), limits=None, damping=0.001)


STRAP_LINKS = 4  # links in the cover's lanyard (even): two 90 mm links propped and caught; shorter ones lie flat


def strap_names() -> tuple[str, ...]:
    return tuple(f"strap_{k}" for k in range(STRAP_LINKS)) + tuple(f"strap_{k}_piv" for k in range(STRAP_LINKS + 1))


def _strap(pos_l: np.ndarray, rot_l: np.ndarray, cover_pos: np.ndarray) -> None:
    """The cover's lanyard: STRAP_LINKS links from under the left leads (just behind the housing) to the cover's rear bottom,
    under the housing, where no finger closes on it. Its joints have no stops, as a limp strap: with the kit's 149 deg
    stops it folded at a stop into a stiff V that held the cover 2 cm under the housing and carried it under the laid pair."""
    anchor = pos_l + rot_l @ np.array([-0.008, 0.0, -P.WIRE_OD / 2 - 0.0015])
    end = pos_l + rot_l @ np.array([P.SB_L - P.CAP_ON - 0.001, 0.0, -P.CAP_H / 2 + 0.002])
    half = 0.5 * float(np.linalg.norm(end - anchor))
    seg = P.STRAP_L / 2.0
    side = rot_l @ np.array([0.0, 1.0, 0.0])
    elbow = 0.5 * (anchor + end) + side * math.sqrt(max(seg**2 - half**2, 0.0))
    names = ("sb_left", "sb_right", "dust_cover", "cap_rail", "mate_rail", *strap_names(),
             *cable_names("l"))
    parent = "sb_left"
    half_links = STRAP_LINKS // 2  # each half of the bowed lanyard in equal straight links
    pts = [anchor + (elbow - anchor) * (k / half_links) for k in range(half_links)] + [elbow + (end - elbow) * (k / half_links) for k in range(half_links + 1)]
    for k in range(STRAP_LINKS):
        a, b = pts[k], pts[k + 1]
        link = f"strap_{k}"
        d = b - a
        yaw = math.atan2(float(d[1]), float(d[0]))
        body(link, [box((float(np.linalg.norm(d)) - 0.004, 0.005, 0.0025), color=COVER_RED, passes=tuple(n for n in names if n != link))],
             pos=_f(0.5 * (a + b)), yaw=yaw, mass=0.003, inertia=AUX_INERTIA, color=COVER_RED, linear_damping=0.4, angular_damping=0.4)
        _universal(f"strap_{k}", link, parent, a, (0.0, 0.0, 1.0), (-math.sin(yaw), math.cos(yaw), 0.0), names, COVER_RED, limits=None)
        parent = link
    _universal(f"strap_{STRAP_LINKS}", "dust_cover", parent, end, (0.0, 0.0, 1.0), rot_l @ np.array([1.0, 0.0, 0.0]), names, COVER_RED, limits=None)


def build(top: float) -> None:
    body("mat", [box((P.MAT_X, P.MAT_Y, P.MAT_T), color=YELLOW)], pos=(P.MAT_XY[0], P.MAT_XY[1], top + P.MAT_T / 2),
         color=YELLOW, friction=(P.MAT_MU, P.MAT_MU))
    block_y = P.RIGHT_NOSE_Y - P.BLOCK_XY[1] / 2  # from the right nose's tip back to 25 mm behind the housing
    body("block", [box((P.BLOCK_XY[0], P.BLOCK_XY[1], P.BLOCK_H), color=BLOCK_GREY)], pos=(P.CONN_X, block_y, top + P.BLOCK_H / 2),
         color=BLOCK_GREY, friction=(0.8, 0.8))
    poses = start_poses(top)
    (pos_l, rot_l), (pos_r, rot_r), (cover_pos, _) = poses["sb_left"], poses["sb_right"], poses["dust_cover"]
    for name, (pos, rot) in (("sb_left", (pos_l, rot_l)), ("sb_right", (pos_r, rot_r))):
        body(name, connector_shapes(), pos=_f(pos), quat=_quat(rot), mass=P.HEAD_MASS, inertia=CONN_INERTIA, center=CONN_CENTER,
             color=GREY, contact_offset=P.CONTACT_OFFSET, contacts=True, linear_damping=0.1, angular_damping=0.1)
    _cable("l", "sb_left", pos_l, rot_l, top)
    _cable("r", "sb_right", pos_r, rot_r, top)

    axis_l = rot_l @ np.array([1.0, 0.0, 0.0])
    inner = ("sb_left", "sb_right", "dust_cover", "cap_rail", "mate_rail", *strap_names())
    # the cover: welded to a rail that slides along the left housing with the slip fit's friction
    body("cap_rail", [sphere(0.002, color=COVER_RED, passes=tuple(n for n in inner if n != "cap_rail"))], pos=_f(cover_pos),
         mass=0.004, inertia=AUX_INERTIA, color=COVER_RED)
    slide("cap_slide", "cap_rail", "sb_left", _f(axis_l), (0.0, P.CAP_ON + 0.006), anchor=_f(cover_pos), friction=P.CAP_PULL_N)
    body("dust_cover", [*(s for s in cover_shapes())], pos=_f(cover_pos), quat=_quat(rot_l), mass=P.CAP_MASS,
         inertia=(9.3e-6, 4.0e-6, 8.4e-6), color=COVER_RED, contact_offset=P.CONTACT_OFFSET, contacts=True,
         linear_damping=0.2, angular_damping=0.2)
    weld("w_cap", "dust_cover", "cap_rail", anchor=_f(cover_pos))
    _strap(pos_l, rot_l, cover_pos)

    # the mate: a slide on the left housing's axis; w_mate joins the right housing to it where it stands
    nose_l = pos_l + rot_l @ np.array([P.SB_L, 0.0, 0.0])
    body("mate_rail", [sphere(0.002, color=GREY, passes=tuple(n for n in inner if n != "mate_rail"))], pos=_f(nose_l),
         quat=_quat(rot_l), mass=0.004, inertia=AUX_INERTIA, color=GREY)
    slide("mate_wipe", "mate_rail", "sb_left", _f(axis_l), (-P.NOSE_L, P.NOSE_L), anchor=_f(nose_l), spring=P.WIPE_K, rest=0.0, damping=20.0)
    weld("w_mate", "sb_right", "mate_rail", made=False)
    mk.sense_contact("sb_left", "sb_right")
    mk.sense_contact("sb_right", "sb_left")


# --- the hands, read from the simulator -----------------------------------------------------------------------------
_FINGER_IDS: dict = {}
_HULLS: dict = {}
_LINKS: dict = {}


def finger_forces() -> dict[str, float]:
    """N each arm's finger drives press with (mean of the two), from the simulator."""
    from yam_arena.constants import FINGER_JOINTS

    env = mk.MECH.environment
    out = {}
    for side in ("left", "right"):
        robot = env.unwrapped.scene[f"{side}_robot"]
        ids = _FINGER_IDS.get(side)
        if ids is None:
            ids = _FINGER_IDS[side] = robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        out[side] = float(robot.data.applied_torque.torch[0][ids].abs().mean().item())
    return out


def read_hands() -> dict[str, np.ndarray]:
    """The corners of each hand's collision hulls (gripper and both fingers), station frame."""
    from yam_arena import analysis

    if not _HULLS:
        for link in ("gripper", "tip_left", "tip_right"):
            _HULLS[link] = np.vstack([h.vertices for h in analysis.link_hulls().get(link, [])])
    base = mk.MECH.environment.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
    hands = {}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        pts = []
        for link, local in _HULLS.items():
            key = (id(base), side, link)
            if key not in _LINKS:
                _LINKS[key] = robot.find_bodies(link)[0][0]
            pose = robot.data.body_link_pose_w.torch[0, _LINKS[key]].detach().cpu().numpy().astype(float)
            pts.append(local @ mk.quat_to_matrix(pose[3:7]).T + pose[:3] - origin)
        hands[side] = np.vstack(pts)
    return hands


def box_distance(points: np.ndarray, name: str) -> float:
    """Least distance (m) from ``points`` to the box that bounds body ``name``."""
    pos, rot = mk.pose(name)
    low, high = mk.MECH.bodies[name].bounds()
    local = (points - pos) @ rot
    gap = np.maximum(np.maximum(np.asarray(low) - local, local - np.asarray(high)), 0.0)
    return float(np.linalg.norm(gap, axis=1).min())
