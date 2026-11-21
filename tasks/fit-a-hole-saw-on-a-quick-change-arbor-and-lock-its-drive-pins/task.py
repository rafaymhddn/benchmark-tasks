"""Mount a 2-1/2 in hole saw onto a quick-change 2-pin arbor clamped in a bench vise: pull the detented pin collar down
so its two 6.35 mm drive pins clear the shoulder, screw the hole saw onto the 5/8"-18 arbor short of bottoming out
until its two 7.8 mm backplate holes line up over the pins (backing off if it reaches the stop), then push the collar
up so both drive pins lock through the backplate.
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
SUCCESS_HOLD_S = 3.0  # both arms finish returning to ready before the trial ends
INSTRUCTION = (
    "Pull the arbor's pin collar down, screw the hole saw onto the arbor, short of bottoming out, until its two pin "
    "holes line up over the drive pins (back it off if it reaches the stop), then push the collar up so both drive "
    "pins lock through the hole saw."
)
RUN_LIMIT_STEPS = 1600  # 80 s at 20 Hz
TOP = TABLE_TOP[SCENE]
SIDES = ("left", "right")

scene.build(TOP)
OBJECTS = list(mk.MECH.bodies.values())
STATE: dict = {}

_HULLS: dict = {}
_LINKS: dict = {}


def _in_sim() -> bool:
    return mk.MECH.environment is not None


def begin(seed: int | None = None) -> dict:
    STATE.clear()
    STATE.update(
        seed=seed,
        zero=None,
        collar_up=True,
        ever_retracted=False,
        carried_hidden=False,
        unthread_wait=False,
        turn_rest={j: 0.0 for j in scene.TURN_JOINTS},
        hold=0,
        hands=None,
        peak_CW_deg=0.0,
        peak_pin_block_N=0.0,
        events=[],
    )
    for name in scene.HIDDEN_NAMES:
        mk.show(name, 0, False)
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)
    if not _in_sim():
        return
    # Collar starts UP (+8.0 mm engaged) held by its upper ball-detent rest
    mk.set_spring("collar_z", rest=P.PIN_STROKE + P.COLLAR_BEYOND)
    mk.set_joint("collar_z", P.PIN_STROKE)
    mk.set_limits("cup_z", lower=-P.SLIDE_PLAY, upper=P.SLIDE_PLAY)
    mk.set_joint("cup_z", 0.0)
    for j in scene.TURN_JOINTS:
        mk.set_spring(j, rest=0.0, most=P.THREAD_DRAG_NM)
        mk.set_joint(j, 0.0)


def _note(msg: str) -> None:
    t = round(mk.now(), 2) if mk.MECH.poses else 0.0
    STATE.setdefault("events", []).append((t, msg))


# --- physical observables read by mechanism & scorer -------------------------------------------------------
def collar_travel() -> float:
    return float(mk.joint("collar_z"))


def saw_pose() -> tuple[np.ndarray, np.ndarray]:
    return mk.pose("hole_saw")


def saw_xy_off() -> float:
    p = saw_pose()[0]
    return float(math.hypot(float(p[0]) - P.X_ARBOR, float(p[1]) - P.Y_ARBOR))


def saw_tilt_deg() -> float:
    r = saw_pose()[1]
    return float(math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2]))))))


def saw_yaw_rad() -> float:
    r = saw_pose()[1]
    return float(math.atan2(float(r[1, 0]), float(r[0, 0])))


def saw_shoulder_gap() -> float:
    """Axial gap (m) from arbor shoulder (TOP + Z_SHOULDER) to hole_saw backplate bottom."""
    return float(saw_pose()[0][2]) - (TOP + P.Z_SHOULDER)


def saw_phase_err_deg() -> float:
    """Angular offset (deg, 0..90) between hole_saw's two +/-y pin holes and arbor's two +/-y drive pins."""
    return P.phase_err_deg(saw_yaw_rad())


def thread_engaged() -> bool:
    return not mk.is_cut("w_thread")


def thread_cw_deg() -> float:
    return float(math.degrees(sum(mk.joint(j) for j in scene.TURN_JOINTS)))


def thread_start_ok() -> bool:
    """The robot has set the hole saw on the thread start: collar retracted, coaxial within CATCH_XY_MAX, square within
    CATCH_TILT_DEG, and its backplate at most CATCH_GAP_BAND under the thread-start height for its yaw."""
    under = P.thread_start_gap(saw_yaw_rad()) - saw_shoulder_gap()
    return bool(
        collar_travel() <= P.CATCH_COLLAR_MAX
        and saw_xy_off() <= P.CATCH_XY_MAX
        and saw_tilt_deg() <= P.CATCH_TILT_DEG
        and 0.0 <= under <= P.CATCH_GAP_BAND
    )


def zero_now() -> str | None:
    if saw_pose()[0][2] - TOP < P.DROP_BELOW_BENCH:
        return "the hole saw dropped below the bench"
    return None


def bodies_still() -> bool:
    for name in ("hole_saw", "collar"):
        lin, ang = mk.velocity(name)
        if float(np.linalg.norm(lin)) > P.STILL_LIN or float(np.linalg.norm(ang)) > P.STILL_ANG:
            return False
    return True


# --- hand clearance ----------------------------------------------------------------------------------------
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
                    raise RuntimeError(f"{side} arm missing link {link!r}")
                _LINKS[key] = found[0]
            pw = robot.data.body_link_pose_w.torch[0, _LINKS[key]].detach().cpu().numpy().astype(float)
            pts.append(local @ mk.quat_to_matrix(pw[3:7]).T + pw[:3] - origin)
        hands[side] = np.vstack(pts)
    STATE["hands"] = hands


def _box_distance(points: np.ndarray, name: str) -> float:
    pos, rot = mk.pose(name)
    low, high = mk.MECH.bodies[name].bounds()
    local = (points - pos) @ rot
    gap = np.maximum(np.maximum(np.asarray(low) - local, local - np.asarray(high)), 0.0)
    return float(np.linalg.norm(gap, axis=1).min())


def hands_clear() -> float:
    """Nearest distance (m) from either hand's hulls to hole_saw or collar."""
    hands = STATE.get("hands")
    if hands:
        pts = np.vstack([hands[s] for s in SIDES if s in hands])
        return min(_box_distance(pts, "hole_saw"), _box_distance(pts, "collar"))
    if mk.MECH.grippers:
        pts = np.vstack([mk.gripper(s)[0].reshape(1, 3) for s in SIDES if s in mk.MECH.grippers])
        return max(0.0, min(_box_distance(pts, "hole_saw"), _box_distance(pts, "collar")) - 0.045)
    return 1.0


# --- step mechanics: collar detent & 5/8"-18 helical thread ------------------------------------------------
def _step_collar() -> None:
    cz = collar_travel()
    if cz <= P.RETRACTED_AT and not STATE.get("ever_retracted", False):
        STATE["ever_retracted"] = True
        _note(f"collar retracted ({1000 * cz:.2f} mm)")
    up = cz > 0.5 * P.PIN_STROKE
    if up != STATE.get("collar_up", True):
        STATE["collar_up"] = up
        if _in_sim():
            mk.set_spring("collar_z", rest=P.PIN_STROKE + P.COLLAR_BEYOND if up else -P.COLLAR_BEYOND)
        _note(f"collar detent {'up' if up else 'down'} ({1000 * cz:.2f} mm)")
    STATE["peak_pin_block_N"] = max(STATE.get("peak_pin_block_N", 0.0), mk.contact_force("collar", "hole_saw"))


def _set_carrier_z(cw_deg: float) -> None:
    z_rel = P.gap_for_yaw_deg(cw_deg) - (P.Z_ENTRY - P.Z_SHOULDER)
    if _in_sim():
        mk.set_limits("cup_z", lower=z_rel - P.SLIDE_PLAY, upper=z_rel + P.SLIDE_PLAY)


def _step_thread() -> None:
    if _in_sim() and not STATE.get("carried_hidden", False):
        for name in scene.HIDDEN_NAMES:
            mk.push(name, force=(0.0, 0.0, P.HIDDEN_MASS * P.GRAVITY))
        STATE["carried_hidden"] = True

    if not thread_engaged():
        gap = saw_shoulder_gap()
        if STATE.get("unthread_wait", False):
            if gap > P.UNTHREAD_REARM_GAP:
                STATE["unthread_wait"] = False
            return
        if thread_start_ok():
            # Engage the thread stand-in where the saw stands: the hidden carrier (never the saw) goes to the thread
            # travel of the saw's present height, and the weld joins the two as they stand. Nothing moves the saw.
            cw_deg = P.cw_for_gap_deg(gap)
            if _in_sim():
                z_rel = gap - (P.Z_ENTRY - P.Z_SHOULDER)
                mk.set_limits("cup_z", lower=z_rel - P.SLIDE_PLAY, upper=z_rel + P.SLIDE_PLAY)
                mk.set_joint("cup_z", z_rel)
                per = math.radians(cw_deg) / 3.0
                for j in scene.TURN_JOINTS:
                    mk.set_joint(j, per)
                    mk.set_spring(j, rest=per, most=P.THREAD_DRAG_NM)
                    STATE["turn_rest"][j] = per
            mk.make("w_thread")
            STATE["catch"] = {"cw_deg": round(cw_deg, 2), "gap_mm": round(1000.0 * gap, 3),
                              "xy_off_mm": round(1000.0 * saw_xy_off(), 3), "tilt_deg": round(saw_tilt_deg(), 3)}
            _note(f"thread caught where the saw stands: cw={cw_deg:.1f} deg, gap={1000 * gap:.3f} mm")
        return

    cw_deg = thread_cw_deg()
    STATE["peak_CW_deg"] = max(STATE.get("peak_CW_deg", 0.0), cw_deg)
    if cw_deg <= P.UNTHREAD_DEG:
        mk.cut("w_thread")
        STATE["unthread_wait"] = True
        if _in_sim():
            mk.set_limits("cup_z", lower=-P.SLIDE_PLAY, upper=P.SLIDE_PLAY)
            mk.set_joint("cup_z", 0.0)
            for j in scene.TURN_JOINTS:
                mk.set_joint(j, 0.0)
                mk.set_spring(j, rest=0.0, most=P.THREAD_DRAG_NM)
                STATE["turn_rest"][j] = 0.0
        _note(f"hole saw unthreaded at cw={cw_deg:.1f} deg")
        return

    _set_carrier_z(cw_deg)
    if _in_sim():
        lag = P.THREAD_DRAG_NM / P.THREAD_SPRING_K  # 0.29 deg << 45 deg kit rule
        cap_45 = math.radians(30.0)
        for j in scene.TURN_JOINTS:
            ang = mk.joint(j)
            r = STATE["turn_rest"][j]
            if ang > r + lag:
                r = ang - lag
            elif ang < r - lag:
                r = ang + lag
            r = max(ang - cap_45, min(ang + cap_45, r))
            STATE["turn_rest"][j] = r
            mk.set_spring(j, rest=r, most=P.THREAD_DRAG_NM)


# --- scoring -----------------------------------------------------------------------------------------------
def aligned_and_seated() -> bool:
    return (
        thread_engaged()
        and saw_xy_off() <= 0.004
        and saw_tilt_deg() <= 6.0
        and -0.0003 <= saw_shoulder_gap() <= P.SEATED_GAP_MAX
        and saw_phase_err_deg() <= P.ALIGN_WINDOW_DEG
    )


def locked_now() -> bool:
    return (
        aligned_and_seated()
        and collar_travel() >= P.LOCKED_COLLAR_MIN
        and hands_clear() >= P.HANDS_CLEAR
        and bodies_still()
    )


@mk.on_step
def step() -> None:
    if _in_sim():
        read_hands()
    if STATE.get("zero") is None:
        STATE["zero"] = zero_now()
    _step_collar()
    _step_thread()
    if locked_now():
        STATE["hold"] = STATE.get("hold", 0) + 1
    else:
        STATE["hold"] = 0


def level() -> float:
    if STATE.get("zero") is not None or zero_now() is not None:
        return 0.0
    if STATE.get("hold", 0) >= P.HOLD_STEPS and locked_now():
        return 1.0
    if aligned_and_seated():
        return 0.5
    coax_threaded = thread_engaged() and saw_xy_off() <= 0.004 and saw_tilt_deg() <= 6.0 and -0.0005 <= saw_shoulder_gap() <= P.THREADED_GAP_MAX
    if STATE.get("ever_retracted", False) or collar_travel() <= P.RETRACTED_AT or coax_threaded:
        return 0.25
    return 0.0


def score(objects) -> float:
    """0 for good if the hole saw ever drops below the bench. Threaded means caught at a real thread start (set down
    coaxial within 0.5 mm and square within 1.5 deg on the first thread); a saw that never caught is cross-threaded.
    At the end: 1.0 when the threaded hole saw is coaxial on the arbor (within 4 mm, tilt <= 6 deg) with a shoulder gap
    of at most 0.60 mm, its two backplate pin holes aligned within
    8 degrees of the drive pins, the collar pushed within 1.5 mm of its upper stop (>= 6.5 mm travel, locking both
    pins through the 5.0 mm backplate), both hands >= 30 mm clear, and all bodies still for 1.0 s. 0.5 when seated
    (<= 0.60 mm gap) and aligned within 8 degrees without the collar locked home. 0.25 when the collar was retracted
    (>= 6.0 mm down) or the hole saw was threaded within 2.0 mm of the shoulder without aligned pin engagement. 0 else."""
    return level()


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out = {
        "zero": STATE.get("zero"),
        "ever_retracted": STATE.get("ever_retracted", False),
        "hold_steps": STATE.get("hold", 0),
        "peak_CW_deg": round(STATE.get("peak_CW_deg", 0.0), 2),
        "peak_pin_block_N": round(STATE.get("peak_pin_block_N", 0.0), 2),
        "catch": STATE.get("catch"),
        "events": STATE.get("events", []),
    }
    if mk.MECH.poses:
        out.update({
            "collar_z_mm": round(1000.0 * collar_travel(), 2),
            "thread_engaged": thread_engaged(),
            "thread_cw_deg": round(thread_cw_deg(), 2),
            "shoulder_gap_mm": round(1000.0 * saw_shoulder_gap(), 3),
            "phase_err_deg": round(saw_phase_err_deg(), 2),
            "saw_xy_off_mm": round(1000.0 * saw_xy_off(), 2),
            "saw_tilt_deg": round(saw_tilt_deg(), 2),
            "pin_contact_N": round(mk.contact_force("collar", "hole_saw"), 2),
            "hands_clear_mm": round(1000.0 * hands_clear(), 1),
            "score": score(None),
        })
    return out
