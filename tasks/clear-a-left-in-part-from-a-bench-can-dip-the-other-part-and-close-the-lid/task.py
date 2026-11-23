"""Clear the SKF 3305 A-2Z bearing left on an open Eagle B-601 bench can's spring dasher screen into a stainless tray,
press the second SKF 3305 A-2Z bearing down on the screen for a second, move it to the tray, and close the 172 deg lid.

Mechanism (``scene.py``, figures in ``parts.py``):
- ``brg_screen`` (25.4 mm wide) sits on ``dasher`` 9.2 mm proud of the 92.1 mm can rim, physically blocking ``lid``
  at ~23 deg ajar until removed.
- ``dasher`` slides 16.0 mm vertically inside ``can`` on a 1,125 N/m spring (2.2 N preload -> 18.0 N at 14 mm plunge,
  equal and opposite downward reaction on ``can``), returning to 0 mm on release.
- ``can`` (0.98 kg total empty B-601) sits free on the bench (mu = 0.25 -> 2.4 N slip threshold) while ``hinge("lid")``
  carries 0.22 N m dry friction (~2.8 N at 80 mm), so swinging the 172 deg lid without bracing ``can`` slides ``can``.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402
import scene  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 3.0
INSTRUCTION = (
    "Put the bearing on the can's screen in the tray, press the other bearing down on the screen for a second, "
    "put it in the tray too, and close the can's lid."
)
RUN_LIMIT_STEPS = 2200  # 110 s at 20 Hz
TOP = TABLE_TOP[SCENE]
SIDES = ("left", "right")
BEARINGS = ("brg_screen", "brg_mat")
SCORED_BODIES = ("can", "lid", "tray", "brg_screen", "brg_mat")

scene.build(TOP)
OBJECTS = list(mk.MECH.bodies.values())
STATE: dict = {}


def begin(seed: int | None = None) -> dict:
    STATE.clear()
    STATE.update(
        seed=seed,
        zero=None,
        dip_steps=0,
        max_dip_steps=0,
        dipped=False,
        max_dash_mm=0.0,
        hold=0,
        hold_from=None,
        hands=None,
        max_can_tilt=0.0,
        max_can_slide=0.0,
        max_tray_shift=0.0,
    )
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)


# --- physical observables ------------------------------------------------------------------------------------------------
def _tilt_deg(name: str) -> float:
    _, r = mk.pose(name)
    return float(math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2]))))))


def can_tilt_deg() -> float:
    return _tilt_deg("can")


def tray_tilt_deg() -> float:
    return _tilt_deg("tray")


def can_slide() -> float:
    return float(np.linalg.norm(mk.pose("can")[0][:2] - scene.can_start_pos(TOP)[:2]))


def tray_shift() -> float:
    return float(np.linalg.norm(mk.pose("tray")[0][:2] - scene.tray_start_pos(TOP)[:2]))


def lid_deg() -> float:
    """Angle of ``lid`` above closed (172 deg as built open at travel 0, 0 deg when closed at travel -172 deg)."""
    return P.LID_OPEN_DEG + float(math.degrees(mk.joint("lid")))


def dash_travel() -> float:
    return float(mk.joint("dasher"))


def lid_closed() -> bool:
    return 0.0 - 1.0 <= lid_deg() <= P.LID_CLOSED_DEG


def screen_up() -> bool:
    return dash_travel() <= P.SCREEN_UP_M


def screen_cleared() -> bool:
    c_xy = mk.pose("can")[0][:2]
    return all(float(np.linalg.norm(mk.pose(b)[0][:2] - c_xy)) > P.CAN_R + 0.005 for b in BEARINGS)


def in_tray(name: str) -> bool:
    p_t, r_t = mk.pose("tray")
    p_b, r_b = mk.pose(name)
    rel = r_t.T @ (p_b - p_t)
    max_x = 0.5 * P.TRAY_L - P.TRAY_WALL - P.BRG_R + 0.003
    max_y = 0.5 * P.TRAY_W - P.TRAY_WALL - P.BRG_R + 0.003
    brg_tilt = float(math.degrees(math.acos(max(-1.0, min(1.0, abs(float(r_b[2, 2])))))))
    dz_bottom = float(rel[2]) - 0.5 * P.BRG_W - P.TRAY_FLOOR
    return abs(float(rel[0])) <= max_x and abs(float(rel[1])) <= max_y and brg_tilt <= 10.0 and -0.003 <= dz_bottom <= 0.006


def both_in_tray() -> bool:
    if not (in_tray("brg_screen") and in_tray("brg_mat")):
        return False
    sep = float(np.linalg.norm(mk.pose("brg_screen")[0][:2] - mk.pose("brg_mat")[0][:2]))
    return sep >= P.BRG_OD - 0.004


def dipping_now() -> bool:
    return dash_travel() >= P.DIP_DEPTH_M and mk.contact_force("dasher", "brg_mat") >= P.DIP_CONTACT_N


def part_off_bench() -> bool:
    return any(float(mk.pose(b)[0][2]) < TOP - 0.020 for b in BEARINGS)


def at_rest() -> bool:
    for name in ("can", "lid", "dasher", "tray", *BEARINGS):
        lin, ang = mk.velocity(name)
        if float(np.linalg.norm(lin)) > P.REST_LIN_MPS or float(np.linalg.norm(ang)) > P.REST_ANG_RPS:
            return False
    return True


def zero_now() -> str | None:
    if can_tilt_deg() > P.CAN_TIP_ZERO_DEG:
        return "the can tipped more than 12 degrees"
    if can_slide() > P.CAN_SLIDE_ZERO_M:
        return "the can slid more than 30 mm"
    if part_off_bench():
        return "a bearing fell off the bench"
    if tray_shift() > P.TRAY_PUSH_ZERO_M or tray_tilt_deg() > P.TRAY_TIP_ZERO_DEG:
        return "the tray shifted more than 25 mm or tilted > 10 deg"
    return None


# --- hand clearance ------------------------------------------------------------------------------------------------------
_HULLS: dict = {}
_LINKS: dict = {}


def read_hands() -> None:
    from yam_arena import analysis

    if not _HULLS:
        for link in ("gripper", "tip_left", "tip_right"):
            _HULLS[link] = np.vstack([h.vertices for h in analysis.link_hulls().get(link, [])])
    base = mk.MECH.environment.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
    hands = {}
    for side in SIDES:
        robot = base.scene[f"{side}_robot"]
        pts = []
        for link, local in _HULLS.items():
            key = (id(base), side, link)
            if key not in _LINKS:
                found = robot.find_bodies(link)[0]
                if not found:
                    raise RuntimeError(f"the {side} arm has no body {link!r}")
                _LINKS[key] = found[0]
            pose_w = robot.data.body_link_pose_w.torch[0, _LINKS[key]].detach().cpu().numpy().astype(float)
            pts.append(local @ mk.quat_to_matrix(pose_w[3:7]).T + pose_w[:3] - origin)
        hands[side] = np.vstack(pts)
    STATE["hands"] = hands


def _body_distance(points: np.ndarray, name: str) -> float:
    pos, rot = mk.pose(name)
    in_body = (points - pos) @ rot
    best = math.inf
    for shape in mk.MECH.bodies[name].shapes:
        if not shape.solid:
            continue
        r_s = mk.quat_to_matrix(shape.quat)
        half = shape.half_extents()
        local = (in_body - np.asarray(shape.pos, dtype=float)) @ r_s
        gap = np.maximum(np.abs(local) - half, 0.0)
        best = min(best, float(np.linalg.norm(gap, axis=1).min()))
    return best


def hands_clear() -> float:
    hands = STATE.get("hands")
    if not hands:
        return 0.0
    points = np.vstack([hands[s] for s in SIDES if s in hands])
    return min(_body_distance(points, name) for name in SCORED_BODIES)


# --- step hook and score -------------------------------------------------------------------------------------------------
def full_now() -> bool:
    return (
        lid_closed()
        and screen_cleared()
        and bool(STATE.get("dipped", False))
        and both_in_tray()
        and screen_up()
        and hands_clear() >= P.HANDS_CLEAR_M
        and at_rest()
    )


@mk.on_step
def step() -> None:
    if mk.MECH.environment is not None:
        read_hands()
    STATE["max_can_tilt"] = max(STATE.get("max_can_tilt", 0.0), can_tilt_deg())
    STATE["max_can_slide"] = max(STATE.get("max_can_slide", 0.0), can_slide())
    STATE["max_tray_shift"] = max(STATE.get("max_tray_shift", 0.0), tray_shift())
    STATE["max_dash_mm"] = max(STATE.get("max_dash_mm", 0.0), 1000.0 * dash_travel())
    if STATE.get("zero") is None:
        STATE["zero"] = zero_now()
    if dipping_now():
        STATE["dip_steps"] = STATE.get("dip_steps", 0) + 1
        STATE["max_dip_steps"] = max(STATE.get("max_dip_steps", 0), STATE["dip_steps"])
        if STATE["dip_steps"] >= P.DIP_HOLD_STEPS:
            STATE["dipped"] = True
    else:
        STATE["dip_steps"] = 0
    if full_now():
        snap = np.hstack([mk.pose("can")[0], mk.pose("brg_screen")[0], mk.pose("brg_mat")[0]])
        if STATE.get("hold", 0) == 0:
            STATE["hold_from"] = snap
        STATE["hold"] = STATE.get("hold", 0) + 1
        if float(np.max(np.abs(snap - STATE["hold_from"]))) > P.HOLD_DRIFT_M:
            STATE["hold"], STATE["hold_from"] = 0, None
    else:
        STATE["hold"], STATE["hold_from"] = 0, None


def level() -> float:
    if STATE.get("zero") is not None or zero_now() is not None:
        return 0.0
    if not (lid_closed() and screen_cleared()):
        return 0.0
    if STATE.get("hold", 0) >= P.HOLD_STEPS and full_now():
        return 1.0
    if STATE.get("dipped", False):
        return 0.5
    return 0.25


def score(objects) -> float:
    """0 for good if the can tips > 12 deg or slides > 30 mm, a bearing falls off the bench, or the tray shifts > 25 mm or
    tilts > 10 deg. 0.0 at the end if the lid is left open (> 4 deg, skirt-to-rim gap > 3.5 mm) or a bearing remains in/on
    the can. 0.25: left-in bearing cleared and lid closed (<= 4 deg, gap <= 3.5 mm). 0.5: plus the second bearing pressed
    down >= 12 mm on the dasher screen (bearing top at/below the 92.1 mm rim) for >= 0.8 s (16 steps). 1.0: plus both
    bearings flat and unstacked inside the tray, dasher sprung back up (<= 2 mm), both hands >= 30 mm clear, and all bodies
    at rest for 1.0 s (20 steps)."""
    return level()


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out = {
        "zero": STATE.get("zero"),
        "dipped": bool(STATE.get("dipped", False)),
        "max_dip_steps": STATE.get("max_dip_steps", 0),
        "max_dash_mm": round(STATE.get("max_dash_mm", 0.0), 2),
        "hold_steps": STATE.get("hold", 0),
        "max_can_tilt_deg": round(STATE.get("max_can_tilt", 0.0), 2),
        "max_can_slide_mm": round(1000.0 * STATE.get("max_can_slide", 0.0), 2),
        "max_tray_shift_mm": round(1000.0 * STATE.get("max_tray_shift", 0.0), 2),
    }
    if mk.MECH.poses:
        out.update({
            "lid_deg": round(lid_deg(), 2),
            "dash_mm": round(1000.0 * dash_travel(), 2),
            "can_slide_mm": round(1000.0 * can_slide(), 2),
            "can_tilt_deg": round(can_tilt_deg(), 2),
            "tray_shift_mm": round(1000.0 * tray_shift(), 2),
            "screen_cleared": screen_cleared(),
            "in_tray": {b: in_tray(b) for b in BEARINGS},
            "dasher_on_brg_mat_n": round(mk.contact_force("dasher", "brg_mat"), 2),
            "lid_on_brg_screen_n": round(mk.contact_force("lid", "brg_screen"), 2),
            "hands_clear_mm": round(1000.0 * hands_clear(), 1),
            "score": score(None),
        })
    return out
