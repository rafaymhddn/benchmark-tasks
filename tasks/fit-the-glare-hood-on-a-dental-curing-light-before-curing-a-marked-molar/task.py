"""Fit the orange glare hood onto a Woodpecker LED.B cordless dental curing light's curved 8.0 mm optical fiber before
curing the blue-marked lower right first molar (#46) on a 1:1 28-tooth demonstration jaw model, then return the wand to
its cradle.

Mechanics (every figure and source in ``parts.py``, bodies and joints in ``scene.py``):
- The 145 g cordless wand rests freely on two open smooth saddles (mu = 0.28, 0.40 N axial slip). Pushing the orange
  hood onto the fiber without bracing the wand slides and rolls the wand out of its cradle.
- The 72 x 62 x 3.0 mm oval orange hood carries a short 4.0 mm rubber-lined grommet collar (8.4 mm bore, no grip boss).
  Outside the fiber it collides as a closed ring, blocking any sideways shortcut onto the shaft. Entering coaxially at
  the 55 deg distal tip (within 2.8 mm and 12 deg for 2 steps) engages the rod's serial guide chain: turning 55 deg
  around the circular elbow (0.03 N m drag, rest within 0.9 deg) unlocks the 22 mm straight base slide against 6.0 N
  elastomer grommet friction (equal and opposite on hood and wand).
- Pressing the top power button 1.2 mm of its 2.5 mm stroke (2.5 N full travel) turns the blue beam on for 3.5 s (70
  steps at 20 Hz, standing in for the 5 s fast mode). Any beam step with the hood unseated (< 19.0 mm) latches a
  whole-trial 0.0; 2.5 s (50 steps) of shielded dwell aimed at molar #46 cures the composite mark and turns it ivory.
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
SUCCESS_HOLD_S = 2.5
INSTRUCTION = (
    "Cure the filling on the blue-marked lower right molar of the jaw model with the dental curing light, "
    "then lay the curing light back in its cradle."
)
RUN_LIMIT_STEPS = 1600  # 80 s at 20 Hz
TOP = TABLE_TOP[SCENE]
SIDES = ("left", "right")

scene.build(TOP)
OBJECTS = list(mk.MECH.bodies.values())
STATE: dict = {}


def _sim(joint_name: str) -> bool:
    return joint_name in mk.MECH.joint_prims


def begin(seed: int | None = None) -> dict:
    if mk.MECH.environment is None and "w_hood_on" in mk.MECH.joints:
        mk.mend_all()
    STATE.clear()
    STATE.update(seed=seed, zero=None, entry_count=0, min_bend_deg=0.0, bend_rest=0.0, seat_rest=0.0,
                 seat_force_n=0.0, max_seat_force_n=0.0, max_btn_force_n=0.0, btn_armed=True, beam_left=0,
                 cure_steps=0, cured=False, hands=None, rest_steps=0, rest_ref=None, events=[])  # fmt: skip
    if _sim("seat_s"):
        mk.set_spring("bend_h", rest=0.0, most=P.ELBOW_DRAG_NM)
        mk.set_limits("bend_h", -0.01, 0.0)
        mk.set_spring("seat_s", rest=0.0, most=P.SEAT_FORCE_N)
        mk.set_limits("seat_s", 0.0, 0.0005)
    mk.show("c_elbow", 0, False), mk.show("c_tip", 0, False)
    mk.show("wand", scene.WAND_BEAM_SHAPE, False)
    mk.show("jaw", scene.JAW_UNCURED_SHAPE, True), mk.show("jaw", scene.JAW_CURED_SHAPE, False)
    return STATE


begin()


@mk.on_reset
def start(seed) -> None:
    begin(seed)


def _note(what: str) -> None:
    STATE["events"].append((round(mk.now() if mk.MECH.environment is not None else 0.0, 2), what))


def hood_on_rod() -> bool:
    return not mk.is_cut("w_hood_on")


def bend_progress_deg() -> float:
    return -math.degrees(float(mk.joint("bend_h"))) if hood_on_rod() else 0.0


def seat_m() -> float:
    return max(0.0, float(mk.joint("seat_s"))) if hood_on_rod() else 0.0


def seat_mm() -> float:
    return 1000.0 * seat_m()


def hood_seated() -> bool:
    return hood_on_rod() and seat_m() >= P.SEAT_MIN_M - 1e-9


def btn_m() -> float:
    return max(0.0, float(mk.joint("pwr_btn")))


def beam_on() -> bool:
    return int(STATE.get("beam_left", 0)) > 0


def tip_world() -> tuple[np.ndarray, np.ndarray]:
    pw, rw = mk.pose("wand")
    return pw + rw @ P.TIP_LOCAL_POS, rw @ P.TIP_TANGENT_LOCAL


def tip_to_molar() -> tuple[float, float, float]:
    tip_pos, tip_tan = tip_world()
    molar = P.molar_world(TOP)
    xy_err = float(math.hypot(tip_pos[0] - molar[0], tip_pos[1] - molar[1]))
    tilt = float(math.degrees(math.acos(float(np.clip(-tip_tan[2], -1.0, 1.0)))))
    return xy_err, float(tip_pos[2] - molar[2]), tilt


def tip_on_molar() -> bool:
    xy_err, dz, tilt = tip_to_molar()
    return xy_err <= P.CURE_XY_TOL and P.CURE_Z_MIN <= dz <= P.CURE_Z_MAX and tilt <= P.CURE_TILT_DEG


def coaxial_at_tip() -> bool:
    tip_pos, tip_tan = tip_world()
    ph, rh = mk.pose("hood")
    rel = ph - tip_pos
    return (float(np.linalg.norm(rel)) <= P.ENTRY_RAD_M and float(rel @ tip_tan) >= -0.002
            and abs(float(rh[:, 0] @ tip_tan)) >= math.cos(math.radians(P.ENTRY_ANG_DEG)))  # fmt: skip


def wand_in_cradle() -> tuple[bool, float, float]:
    pw, rw = mk.pose("wand")
    pos_err = float(np.linalg.norm(pw - P.wand_start_pose(TOP)[0]))
    axis_ang = float(math.degrees(math.acos(float(np.clip(rw[1, 0], -1.0, 1.0)))))
    return pos_err <= P.CRADLE_POS_TOL and axis_ang <= P.CRADLE_TILT_DEG, pos_err, axis_ang


def off_bench() -> str | None:
    for name in ("wand", "hood"):
        p = mk.pose(name)[0]
        if p[2] < TOP - 0.005 or not (0.04 <= p[0] <= 0.58 and -0.36 <= p[1] <= 0.36):
            return f"{name} left the bench"
    return None


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
            lp = robot.data.body_link_pose_w.torch[0, _LINKS[key]].detach().cpu().numpy().astype(float)
            pts.append(local @ mk.quat_to_matrix(lp[3:7]).T + lp[:3] - origin)
        hands[side] = np.vstack(pts)
    STATE["hands"] = hands


def _box_distance(points: np.ndarray, name: str) -> float:
    pos, rot = mk.pose(name)
    low, high = mk.MECH.bodies[name].bounds()
    local = (points - pos) @ rot
    gap = np.maximum(np.maximum(np.asarray(low) - local, local - np.asarray(high)), 0.0)
    return float(np.linalg.norm(gap, axis=1).min())


def hands_clear_m() -> float:
    hands = STATE.get("hands")
    if not hands:
        return 0.0
    pts = np.vstack([hands[s] for s in SIDES if s in hands])
    return min(_box_distance(pts, name) for name in ("wand", "hood", "jaw"))


def hands_clear() -> bool:
    return hands_clear_m() >= P.HANDS_CLEAR_M - 1e-9


def _rest_snapshot() -> tuple[np.ndarray, np.ndarray, float, float]:
    pw, rw = mk.pose("wand")
    return pw.copy(), mk.pose("hood")[0].copy(), math.degrees(math.atan2(rw[1, 0], rw[0, 0])), seat_m()


def _moved(ref) -> bool:
    pw, ph, yaw_w, sm = _rest_snapshot()
    dyaw = abs((yaw_w - ref[2] + 180.0) % 360.0 - 180.0)
    return bool(np.linalg.norm(pw - ref[0]) > P.REST_MOVE_M or np.linalg.norm(ph - ref[1]) > P.REST_MOVE_M
                or dyaw > P.REST_TURN_DEG or abs(sm - ref[3]) > P.REST_MOVE_M)  # fmt: skip


@mk.on_step
def step() -> None:
    if mk.MECH.environment is not None:
        read_hands()

    if not hood_on_rod():
        STATE["seat_force_n"] = 0.0
        if _sim("seat_s"):
            mk.set_spring("bend_h", rest=0.0, most=P.ELBOW_DRAG_NM)
            mk.set_limits("bend_h", -0.01, 0.0)
            mk.set_spring("seat_s", rest=0.0, most=P.SEAT_FORCE_N)
            mk.set_limits("seat_s", 0.0, 0.0005)
        STATE["entry_count"] = STATE["entry_count"] + 1 if coaxial_at_tip() else 0
        if STATE["entry_count"] >= P.ENTRY_STEPS:
            mk.make("w_hood_on")
            STATE["bend_rest"], STATE["seat_rest"], STATE["min_bend_deg"] = 0.0, 0.0, 0.0
            if _sim("bend_h"):
                mk.set_limits("bend_h", -P.FIBER_BEND_RAD, 0.0)
            _note("hood_entered_tip")
    else:
        b_rad = float(mk.joint("bend_h"))
        b_deg, s_val = math.degrees(b_rad), max(0.0, float(mk.joint("seat_s")))
        STATE["min_bend_deg"] = min(STATE["min_bend_deg"], b_deg)
        prev_seat_rest = STATE["seat_rest"]
        f_grom = min(P.SEAT_FORCE_N, abs(P.SEAT_SPRING_K * (s_val - prev_seat_rest)))
        STATE["seat_force_n"] = f_grom
        STATE["max_seat_force_n"] = max(STATE["max_seat_force_n"], f_grom)
        STATE["bend_rest"] = float(np.clip(STATE["bend_rest"], b_rad - P.ELBOW_GIVE_RAD, b_rad + P.ELBOW_GIVE_RAD))
        STATE["seat_rest"] = float(np.clip(prev_seat_rest, s_val - P.SEAT_GIVE_M, s_val + P.SEAT_GIVE_M))
        unlock_rad = -math.radians(P.FIBER_BEND_DEG - P.ELBOW_UNLOCK_DEG)
        if _sim("seat_s"):
            mk.set_spring("bend_h", rest=STATE["bend_rest"], most=P.ELBOW_DRAG_NM)
            mk.set_spring("seat_s", rest=STATE["seat_rest"], most=P.SEAT_FORCE_N)
            if b_rad <= unlock_rad or s_val >= 0.002:
                mk.set_limits("seat_s", 0.0, P.SEAT_TRAVEL)
                mk.set_limits("bend_h", -P.FIBER_BEND_RAD, unlock_rad if s_val >= 0.002 else 0.0)
            else:
                mk.set_limits("seat_s", 0.0, 0.0005)
                mk.set_limits("bend_h", -P.FIBER_BEND_RAD, 0.0)
        if hood_seated() and not any(e[1].startswith("hood_seated") for e in STATE["events"]):
            _note(f"hood_seated ({seat_mm():.1f} mm)")
        if STATE["min_bend_deg"] < -10.0 and s_val <= 0.001 and b_deg >= -1.0:
            mk.cut("w_hood_on")
            STATE["entry_count"] = 0
            _note("hood_removed")

    press = btn_m()
    STATE["max_btn_force_n"] = max(STATE["max_btn_force_n"], min(P.BTN_FORCE_N, P.BTN_SPRING * press))
    if press < 0.0008:
        STATE["btn_armed"] = True
    if press >= P.BTN_TRIGGER_M and STATE["btn_armed"] and STATE["beam_left"] == 0:
        STATE["beam_left"], STATE["btn_armed"] = P.BEAM_STEPS, False
        mk.show("wand", scene.WAND_BEAM_SHAPE, True), mk.redraw(2)
        _note("beam_on")

    if STATE["beam_left"] > 0:
        if not hood_seated():
            if STATE["zero"] is None:
                STATE["zero"] = "blue curing beam activated without orange light hood seated (>= 19.0 mm)"
                _note("unshielded_flash")
        elif STATE["zero"] is None and tip_on_molar():
            STATE["cure_steps"] += 1
            if STATE["cure_steps"] >= P.CURE_STEPS_REQ and not STATE["cured"]:
                STATE["cured"] = True
                mk.show("jaw", scene.JAW_UNCURED_SHAPE, False), mk.show("jaw", scene.JAW_CURED_SHAPE, True)
                mk.redraw(3)
                _note("molar_cured")
        STATE["beam_left"] -= 1
        if STATE["beam_left"] == 0:
            mk.show("wand", scene.WAND_BEAM_SHAPE, False), mk.redraw(2)
            _note("beam_off")

    if STATE["zero"] is None:
        STATE["zero"] = off_bench()

    if hands_clear() and not beam_on():
        if STATE["rest_ref"] is None or _moved(STATE["rest_ref"]):
            STATE["rest_ref"], STATE["rest_steps"] = _rest_snapshot(), 1
        else:
            STATE["rest_steps"] += 1
    else:
        STATE["rest_ref"], STATE["rest_steps"] = None, 0


def score(objects) -> float:
    """0.0 permanently if the blue curing beam turns on before the orange hood is seated >= 19.0 mm on the straight
    fiber base, or if wand or hood leaves the bench. Otherwise at the end:
    1.0: hood seated (>= 19.0 mm), lower right first molar #46 cured (>= 2.5 s shielded beam dwell within 8.0 mm
         horizontally, 1.0-12.0 mm above the crown, and <= 22.0 deg from vertical), beam off, wand back in its cradle
         (within 15.0 mm and 15.0 deg), both hands >= 30.0 mm clear of wand, hood and jaw, at rest for 1.0 s (20 steps);
    0.75: hood seated (>= 19.0 mm), molar #46 cured, and beam off, wand not yet cradled/clear/settled;
    0.5: hood seated (>= 19.0 mm) with no unshielded flash, molar #46 not yet cured;
    0.0: otherwise."""
    del objects
    if STATE.get("zero") is not None or off_bench() is not None or not hood_seated():
        return 0.0
    if STATE.get("cured") and not beam_on():
        if wand_in_cradle()[0] and hands_clear() and STATE.get("rest_steps", 0) >= P.REST_STEPS:
            return 1.0
        return 0.75
    return 0.5


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out = {"zero": STATE.get("zero"), "events": STATE.get("events", []), "cure_steps": STATE.get("cure_steps", 0),
           "cured": bool(STATE.get("cured")), "beam_left": STATE.get("beam_left", 0), "rest_steps": STATE.get("rest_steps", 0),
           "max_seat_force_n": round(float(STATE.get("max_seat_force_n", 0.0)), 2),
           "max_btn_force_n": round(float(STATE.get("max_btn_force_n", 0.0)), 2)}  # fmt: skip
    if mk.MECH.poses:
        xy_e, dz, tilt = tip_to_molar()
        in_c, c_mm, c_deg = wand_in_cradle()
        out.update({"hood_on_rod": hood_on_rod(), "bend_progress_deg": round(bend_progress_deg(), 2),
                    "seat_mm": round(seat_mm(), 2), "hood_seated": hood_seated(), "btn_mm": round(1000.0 * btn_m(), 2),
                    "tip_to_molar_xy_z_mm_deg": [round(1000.0 * xy_e, 2), round(1000.0 * dz, 2), round(tilt, 1)],
                    "wand_in_cradle": [in_c, round(1000.0 * c_mm, 2), round(c_deg, 2)],
                    "hands_clear_mm": round(1000.0 * hands_clear_m(), 1), "score": score(None)})  # fmt: skip
    return out
