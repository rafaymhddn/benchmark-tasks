"""Reconnect a bathroom-sink pop-up stopper to its rear-facing pivot rod inside a vanity and hold the stopper open.

Scene (``scene.py``, figures and sources in ``parts.py``): a benchtop birch-plywood vanity cabinet with an open front
doorway, full-height side and rear walls, a full-depth stepped countertop whose rear section behind the basin rim is a
full-width ``160 x 424 x 9 mm`` clear cast-acrylic L-roof (collides solidly in PhysX while ``top_cam`` sees the rear
linkage), a solvent-welded ``5 mm`` white ABS drop-in basin (``110 x 190 x 42 mm`` outer, ``100 x 180 x 38 mm`` inner),
a ``1-1/4 in`` drain body whose pivot port faces straight rearward (``+x``), a chrome Danco 11041 pop-up stopper resting
closed in the drain with its clevis eye ``6.2 mm`` below the port and turned ``90 deg`` off axis, and a Danco 88532
ball-pivot rod resting ``32 mm`` retracted behind the rear port seat.

All mechanics are native 240 Hz PhysX contacts (compliant cone-washer beads and 4-slot PETG snap-collar on ``drain``;
the task welds, pushes, and latches nothing). Scoring reads only external bench-measurable quantities: rod-drop fall
from video, the rod's red seating mark at the rear collar face, and the stopper head gap above the flange.
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
SUCCESS_HOLD_S = 3.5  # arms finish returning to ready before episode ends so the last frame shows the open stopper clear
INSTRUCTION = (
    "Lift and align the sink's pop-up stopper from inside the basin while sliding the pivot rod forward through the "
    "rear drain port and stopper eye under the basin until its ball seats, then press the rear rod down to hold the "
    "stopper open."
)
RUN_LIMIT_STEPS = 1000  # 50 s at 20 Hz
TOP = TABLE_TOP[SCENE]
SIDES = ("left", "right")

scene.build(TOP)
OBJECTS = list(mk.MECH.bodies.values())
STATE: dict = {}


def begin(seed: int | None = None) -> dict:
    STATE.clear()
    STATE.update(seed=seed, zero=None, hold=0, hold_from=None, hands=None, peak_gap=0.0, first_seated=None)
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)
    if mk.MECH.environment is not None:
        for idx in scene.ACRYLIC_PANE_SHAPES:
            mk.show("vanity", idx, False)  # solid 9 mm clear cast-acrylic L-roof colliders stay unseen by cameras


# --- Bench-observable physical readings ------------------------------------------------------------------------------
def stopper_pose() -> tuple[np.ndarray, np.ndarray]:
    return mk.pose("stopper")


def rod_pose() -> tuple[np.ndarray, np.ndarray]:
    return mk.pose("pivot_rod")


def stopper_gap() -> float:
    """m the stopper cap underside sits above the closed drain flange seat (steel rule in basin)."""
    return float(stopper_pose()[0][2] - (TOP + P.FLANGE_Z))


def stopper_radial() -> float:
    """m horizontal offset of the stopper from the drain bore axis."""
    p = stopper_pose()[0]
    return float(math.hypot(p[0] - P.X_DRAIN, p[1] - P.Y_DRAIN))


def stopper_tilt_deg() -> float:
    """Stopper tilt from vertical (deg)."""
    return float(math.degrees(math.acos(max(-1.0, min(1.0, float(stopper_pose()[1][2, 2]))))))


def stopper_eye_yaw_off_deg() -> float:
    """How far the stopper's symmetric through-hole (local +-x) stands from the rear port's x axis (0..90 deg)."""
    x_axis = stopper_pose()[1][:, 0]
    horiz = math.hypot(float(x_axis[0]), float(x_axis[1]))
    if horiz < 1e-9:
        return 90.0
    return float(math.degrees(math.acos(min(1.0, abs(float(x_axis[0])) / horiz))))


def ball_seat_offset() -> float:
    """m from the 18.3 mm ball centre to the rear port seat (X_SEAT, Y_DRAIN, TOP + PORT_Z)."""
    seat = np.array([P.X_SEAT, P.Y_DRAIN, TOP + P.PORT_Z], dtype=float)
    return float(np.linalg.norm(rod_pose()[0] - seat))


def collar_mark_gap() -> float:
    """m along x between the front edge of the rod's red seating band and the rear collar face X_COLLAR (0 = flush)."""
    p, r = rod_pose()
    mark_front = p + r @ np.array([P.X_COLLAR - P.X_SEAT, 0.0, 0.0], dtype=float)
    return float(mark_front[0] - P.X_COLLAR)


def rod_pitch_deg() -> float:
    """Downward pitch of the rear rod (deg; positive when rear rod is pressed down, lifting the inner tip)."""
    _p, r = rod_pose()
    rear_dir = r[:, 0]
    return float(math.degrees(math.asin(max(-1.0, min(1.0, -float(rear_dir[2]))))))


def rod_tail_z_over_bench() -> float:
    """Height over bench (m) of the rear end of the steel rod."""
    p, r = rod_pose()
    tail = p + r @ np.array([P.ROD_REAR_REACH, 0.0, 0.0], dtype=float)
    return float(tail[2] - TOP)


def ball_z_over_bench() -> float:
    """Height over bench (m) of the 18.3 mm pivot ball centre."""
    return float(rod_pose()[0][2] - TOP)


def collar_seated() -> bool:
    """Outside rear-collar check: red band flush within 3.5 mm of collar face and ball seated in port throat."""
    p = rod_pose()[0]
    throat_radial = math.hypot(float(p[1] - P.Y_DRAIN), float(p[2] - (TOP + P.PORT_Z)))
    return abs(collar_mark_gap()) <= P.COLLAR_SEAT_MAX and throat_radial <= P.COLLAR_SEAT_MAX


def at_rest() -> bool:
    """Both free parts settled under 25 mm/s."""
    return all(float(np.linalg.norm(mk.velocity(name)[0])) <= P.REST_SPEED_MAX for name in ("stopper", "pivot_rod"))


def zero_now() -> str | None:
    """Whole-trial fall foul (judged from trial video on the bench; latched for the episode)."""
    if ball_z_over_bench() < P.ROD_DROP_ZERO or (not collar_seated() and rod_tail_z_over_bench() < 0.004):
        return "pivot rod dropped onto cabinet floor"
    return None


# --- Hand hull clearance (reads live URDF link hulls in sim; falls back to gripper TCP in unit tests) ----------------
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
        points = []
        for link, local in _HULLS.items():
            key = (id(base), side, link)
            if key not in _LINKS:
                found = robot.find_bodies(link)[0]
                if not found:
                    raise RuntimeError(f"the {side} arm has no body {link!r}")
                _LINKS[key] = found[0]
            pose = robot.data.body_link_pose_w.torch[0, _LINKS[key]].detach().cpu().numpy().astype(float)
            points.append(local @ mk.quat_to_matrix(pose[3:7]).T + pose[:3] - origin)
        hands[side] = np.vstack(points)
    STATE["hands"] = hands


def _box_distance(points: np.ndarray, name: str) -> float:
    position, rotation = mk.pose(name)
    low, high = mk.MECH.bodies[name].bounds()
    local = (points - position) @ rotation
    gap = np.maximum(np.maximum(np.asarray(low) - local, local - np.asarray(high)), 0.0)
    return float(np.linalg.norm(gap, axis=1).min())


def hands_clear() -> float:
    """Nearest distance (m) from either hand to stopper, pivot_rod, or drain."""
    hands = STATE.get("hands")
    if hands:
        points = np.vstack([hands[s] for s in SIDES if s in hands])
    elif mk.MECH.grippers:
        points = np.vstack([mk.gripper(s)[0].reshape(1, 3) for s in SIDES if s in mk.MECH.grippers])
    else:
        return 0.0
    return min(_box_distance(points, name) for name in ("stopper", "pivot_rod", "drain"))


# --- Score levels ----------------------------------------------------------------------------------------------------
def half_now() -> bool:
    """0.5 condition (instantaneous): ball seated at rear collar, stopper upright in bore (<= 22 mm), hands clear, at rest."""
    gap = stopper_gap()
    return (
        collar_seated()
        and -0.002 <= gap <= P.GAP_OPEN_MAX
        and stopper_radial() <= P.STOP_RADIAL_HALF
        and stopper_tilt_deg() <= P.STOP_TILT_HALF_DEG
        and hands_clear() >= P.HANDS_CLEAR
        and at_rest()
    )


def full_now() -> bool:
    """1.0 condition before 1.0 s hold: collar seated and stopper held open 8.0..22.0 mm above flange, hands clear, at rest."""
    gap = stopper_gap()
    return (
        collar_seated()
        and P.GAP_OPEN_MIN <= gap <= P.GAP_OPEN_MAX
        and stopper_radial() <= P.STOP_RADIAL_FULL
        and stopper_tilt_deg() <= P.STOP_TILT_FULL_DEG
        and hands_clear() >= P.HANDS_CLEAR
        and at_rest()
    )


@mk.on_step
def step() -> None:
    if mk.MECH.environment is not None:
        read_hands()
    STATE["peak_gap"] = max(STATE.get("peak_gap", 0.0), stopper_gap())
    if STATE.get("zero") is None:
        STATE["zero"] = zero_now()
    if STATE.get("first_seated") is None and collar_seated():
        STATE["first_seated"] = mk.now() if mk.MECH.environment is not None else 0.0
    if full_now():
        if STATE.get("hold", 0) == 0:
            STATE["hold_from"] = stopper_pose()[0].copy()
        STATE["hold"] = STATE.get("hold", 0) + 1
        if float(np.linalg.norm(stopper_pose()[0] - STATE["hold_from"])) > P.HOLD_DRIFT:
            STATE["hold"], STATE["hold_from"] = 0, None
    else:
        STATE["hold"], STATE["hold_from"] = 0, None


def level() -> float:
    if STATE.get("zero") is not None or zero_now() is not None:
        return 0.0
    if STATE.get("hold", 0) >= P.HOLD_STEPS and full_now():
        return 1.0
    if half_now():
        return 0.5
    return 0.0


def score(objects) -> float:
    """0.0 for the whole trial (latched permanently, checked from trial video) if the pivot rod's 18.3 mm ball drops
    below 25.0 mm over the bench or the unseated rod's rear end drops below 4.0 mm onto the cabinet floor; lifting the
    stopper out during the trial is not latched (scored by its end state). Otherwise, with both stopper and pivot_rod at
    rest (<= 25 mm/s) and every link/finger of both hands >= 30.0 mm clear of stopper, pivot_rod, and drain: 0.5 if the
    front edge of the rod's red collar mark sits within 3.5 mm of the rear collar face (x = 0.417 m, ball within 3.5 mm
    radially of the port throat) and the stopper sits upright in the bore (radial offset <= 4.0 mm, tilt <= 8.0 deg, cap
    gap in [-2.0, 22.0] mm) without meeting the 1.0 open hold; 1.0 if the rod collar mark is seated within 3.5 mm (throat
    radial <= 3.5 mm) and the stopper cap underside is held open between 8.0 mm and 22.0 mm above the closed flange seat
    (radial offset <= 3.5 mm, tilt <= 6.0 deg) for 1.0 s (20 steps at 20 Hz) with <= 2.0 mm stopper drift."""
    _ = objects
    return level()


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out = {
        "zero": STATE.get("zero"),
        "hold steps": STATE.get("hold", 0),
        "peak stopper gap (mm)": round(1000.0 * STATE.get("peak_gap", 0.0), 2),
        "first seated (s)": STATE.get("first_seated"),
    }
    if mk.MECH.poses:
        out.update(
            {
                "stopper gap (mm)": round(1000.0 * stopper_gap(), 2),
                "stopper radial (mm)": round(1000.0 * stopper_radial(), 2),
                "stopper tilt (deg)": round(stopper_tilt_deg(), 2),
                "eye yaw off x (deg)": round(stopper_eye_yaw_off_deg(), 2),
                "collar mark gap (mm)": round(1000.0 * collar_mark_gap(), 2),
                "ball seat offset (mm)": round(1000.0 * ball_seat_offset(), 2),
                "rod pitch down (deg)": round(rod_pitch_deg(), 2),
                "forces rod-drain, rod-stopper, stopper-drain (N)": [
                    round(mk.contact_force("pivot_rod", "drain"), 2),
                    round(mk.contact_force("pivot_rod", "stopper"), 2),
                    round(mk.contact_force("stopper", "drain"), 2),
                ],
                "hands clear (mm)": round(1000.0 * hands_clear(), 1),
                "score": score(None),
            }
        )
    return out
