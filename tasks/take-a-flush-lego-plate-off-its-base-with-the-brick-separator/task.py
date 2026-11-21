"""Take a flush LEGO 2x4 plate off the green 8 x 16 plate with the orange brick separator, then pull the plate out of
the separator into the dish, leaving the two white plates beside it where they are.

This file holds the rules and score(). Sizes and figures: ``parts.py`` (one constant each, with its source). Readings
of the scene (poses, distances, speeds): ``readings.py``.

The stand-ins for the stud clutch, each a force between two named bodies (equal and opposite through the joint):
- peel: the target plate hinges about its far edge on the base (hinge ``peel``). A capped spring whose rest follows the
  joint holds it: the near end lifts only against PEEL_HOLD_NM (0.07 N m, 2.5 N at the near row) and stays where it is
  left; at PEEL_RELEASE_DEG (12 deg) the weld ``w_target`` is cut and the plate is off the base. It counts as taken
  off with the separator (0.5 and 1.0) only if the separator was seated on it at that moment.
- seat: the separator joins the plate's near row (weld ``w_seat``) when its socket sits over the row within
  SOCKET_TOL_MM (0.8 mm) and SOCKET_TOL_DEG (4 deg), its face on the plate (0.4 mm), pressed with at least PRESS_N
  (6 N) for SEAT_STEPS (3) control steps running. The press is the moment on the peel hinge's sprung lower stop
  minus the stop's reading at rest after the reset (0.4 s), over ROW_LEVER (27.9 mm).
- pull: the seated plate leaves the socket only when pulled along its studs through STUD_H (1.7 mm) against
  SEP_HOLD_N (4 N) (slide ``pull``, capped spring, rest following); then ``w_seat`` is cut.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402
import readings as R  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 2.0
INSTRUCTION = (
    "Use the orange brick separator to take the black 2x4 plate off the green plate without moving the two white "
    "plates beside it, then put the black plate in the dish and the separator on the bench."
)
RUN_LIMIT_STEPS = 2400  # 120 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# Scoring (the same numbers in task.json and the README)
MAX_BASE_LIFT_M = 0.003  # a base corner lifted more than 3 mm: 0 for the trial
MAX_BASE_SLIDE_M = 0.015  # a base corner more than 15 mm from its mark: 0 for the trial
MAX_DISH_MOVE_M = 0.020  # the dish pushed more than 20 mm: 0 for the trial
GRIPPER_CLEAR_M = 0.030  # at the end both grasp points at least 30 mm from the plate, the separator and the dish
SEP_ON_BENCH_M = 0.002  # the separator's lowest corner within 2 mm of the bench top: it lies on the bench
WHITE_PLACE_M = 0.002  # each white plate within 2 mm of its place on the base
REST_LIN_MPS, REST_ANG_RPS = 0.010, 0.10  # at rest: below 10 mm/s and 0.1 rad/s
REST_HOLD_S = 1.0  # the end state held at rest for 1 s (20 steps)
TARE_AT_S = 0.40  # s: the press gauge's zero is read here (the reset has settled; no hand can reach the plate)
SETTLE_S = 0.35  # s: the base starts OVER (1 mm) above the bench; its lift counts from there once it has landed
CHAIN_HOME_M = 0.0005  # the socket's chain counts as home on the studs below 0.5 mm of pull travel (it reads 0.18 mm
# under a 9 N press: run l10t15); a re-seat waits until a pulled-out chain (1.7 mm) has come back

OBJECTS = P.build_scene(TOP)
START_BASE_CORNERS_XY = P.base_corners_xy(P.BASE_XY)
BASE_REST_Z = TOP + 0.5 * P.BASE_T
STATE: dict = {}


def begin(seed: int | None = None) -> dict:
    STATE.clear()
    STATE.update(
        seed=seed, seat_streak=0, ever_seated=False, off_with_seat=False, peel_rest=0.0, peel_max=0.0, pull_rest=0.0, pull_max=0.0,
        zero_reason=None, max_base_lift_mm=0.0, max_base_slide_mm=0.0, max_dish_move_mm=0.0, peak_press_n=0.0,
        best_press_align=None, seat_align=None, tare_stop_nm=R.weight_moment_nm(), tare_locked=False, rest_held_s=0.0,
        last_time=0.0, events=[], peel_log=[], pull_log=[], press_log=[],
    )  # fmt: skip
    for name in P.CARRIER_NAMES:  # the hidden links of the joint chains are not drawn
        mk.show(name, 0, False)
    return STATE


begin()


@mk.on_reset
def start(seed: int | None = None) -> None:
    begin(seed)
    if "peel" in mk.MECH.joint_prims:  # the simulator keeps a spring's rest through a reset: put both back
        mk.set_spring("peel", rest=0.0, most=P.PEEL_HOLD_NM)
        mk.set_spring("pull", rest=0.0, most=P.SEP_HOLD_N)


def _event(t: float, text: str) -> None:
    STATE["events"].append((round(t, 3), text))


def _in_sim() -> bool:
    return "peel" in mk.MECH.joint_prims


def press_force_n() -> float:
    """The press on the plate's near row (N): stop moment minus the stop's reading at rest, over ROW_LEVER."""
    if mk.is_cut("w_target"):
        return 0.0
    return max(0.0, (R.stop_moment_nm() - float(STATE["tare_stop_nm"])) / P.ROW_LEVER)


def aligned(off: dict) -> bool:
    return bool(
        off["horiz_mm"] <= P.SOCKET_TOL_MM
        and off["face_gap_mm"] <= P.SOCKET_FACE_MM
        and off["tilt_deg"] <= P.SOCKET_TOL_DEG
        and off["yaw_deg"] <= P.SOCKET_TOL_DEG
    )


def _seat_rule(t: float, press_n: float) -> None:
    """Seat only while the plate is on the base and the socket's chain is home on the studs (CHAIN_HOME_M)."""
    if mk.is_cut("w_target") or not mk.is_cut("w_seat"):
        STATE["seat_streak"] = 0
        return
    off = R.socket_offset()
    if press_n > float(STATE["peak_press_n"]):
        STATE["peak_press_n"] = press_n
        STATE["best_press_align"] = {k: round(v, 2) for k, v in off.items()}
    if press_n >= 0.5:
        STATE["press_log"].append({"t": round(t, 3), "press_n": round(press_n, 2), **{k: round(v, 2) for k, v in off.items()}})
    if aligned(off) and press_n >= P.PRESS_N and float(mk.joint("pull")) < CHAIN_HOME_M:
        STATE["seat_streak"] += 1
        if STATE["seat_streak"] >= P.SEAT_STEPS:
            mk.make("w_seat")
            STATE.update(ever_seated=True, seat_streak=0, seat_align={"press_n": round(press_n, 2), **{k: round(v, 2) for k, v in off.items()}})
            _event(t, f"w_seat made: press {press_n:.2f} N, socket {off['horiz_mm']:.2f} mm off the row")
    else:
        STATE["seat_streak"] = 0


def _peel_rule(t: float) -> None:
    """The plate's near end lifts against PEEL_HOLD_NM and stays where it is left; off the base at PEEL_RELEASE_DEG."""
    if mk.is_cut("w_target"):
        return
    angle = float(mk.joint("peel"))
    prev = float(STATE["peel_rest"])
    moment = min(P.PEEL_HOLD_NM, P.PEEL_K * max(0.0, angle - prev))  # what the spring held with over the last step
    deg = math.degrees(angle)
    if deg > float(STATE["peel_max"]) + 0.25 or angle >= P.PEEL_RELEASE_RAD:
        STATE["peel_log"].append({"t": round(t, 3), "deg": round(deg, 2), "moment_nm": round(moment, 4), "row_n": round(moment / P.ROW_LEVER, 2)})
    STATE["peel_max"] = max(float(STATE["peel_max"]), deg)
    if angle >= P.PEEL_RELEASE_RAD:
        mk.cut("w_target")
        STATE["off_with_seat"] = not mk.is_cut("w_seat")
        _event(t, f"w_target cut: peel {deg:.2f} deg, " + ("separator seated" if STATE["off_with_seat"] else "separator NOT seated: not the task"))
        return
    rest = max(0.0, min(max(prev, angle - P.PEEL_LAG_RAD), angle + P.PEEL_LAG_RAD))  # follows the joint both ways
    STATE["peel_rest"] = rest
    if _in_sim():  # on the stop (angle < 0) the spring gives nothing: the stop alone carries a press
        mk.set_spring("peel", rest=angle if angle < 0.0 else rest, most=P.PEEL_HOLD_NM)


def _pull_rule(t: float) -> None:
    """The seated plate leaves the socket after STUD_H of travel along the studs against SEP_HOLD_N."""
    pull = float(mk.joint("pull"))
    if mk.is_cut("w_seat"):  # not seated: the socket's chain goes home to the studs
        STATE["pull_rest"] = 0.0
        if _in_sim():
            mk.set_spring("pull", rest=0.0, most=P.SEP_HOLD_N)
        return
    prev = float(STATE["pull_rest"])
    force = min(P.SEP_HOLD_N, P.PULL_K * max(0.0, pull - prev))
    mm = 1000.0 * pull
    if mm > float(STATE["pull_max"]) + 0.05 or pull >= P.STUD_H:
        STATE["pull_log"].append({"t": round(t, 3), "mm": round(mm, 3), "force_n": round(force, 2)})
    STATE["pull_max"] = max(float(STATE["pull_max"]), mm)
    if pull >= P.STUD_H:
        mk.cut("w_seat")
        _event(t, f"w_seat cut: pulled {mm:.2f} mm along the studs")
        return
    rest = max(0.0, min(max(prev, pull - P.PULL_LAG_M), pull + P.PULL_LAG_M))
    STATE["pull_rest"] = rest
    if _in_sim():
        mk.set_spring("pull", rest=rest, most=P.SEP_HOLD_N)


def _zero_rules(t: float) -> None:
    lift, slide = R.base_lift_and_slide_m(BASE_REST_Z + (P.OVER if t < SETTLE_S else 0.0), START_BASE_CORNERS_XY)
    dish = R.dish_move_m()
    for key, value in (("max_base_lift_mm", lift), ("max_base_slide_mm", slide), ("max_dish_move_mm", dish)):
        STATE[key] = max(float(STATE[key]), 1000.0 * value)
    if STATE["zero_reason"] is not None:
        return
    if lift > MAX_BASE_LIFT_M:
        STATE["zero_reason"] = f"a base corner lifted {1000 * lift:.2f} mm"
    elif slide > MAX_BASE_SLIDE_M:
        STATE["zero_reason"] = f"a base corner slid {1000 * slide:.2f} mm"
    elif dish > MAX_DISH_MOVE_M:
        STATE["zero_reason"] = f"the dish moved {1000 * dish:.2f} mm"
    if STATE["zero_reason"] is not None:
        _event(t, "zero rule: " + STATE["zero_reason"])


def end_state_ok() -> bool:
    """1.0's end state: off the base, out of the separator, in the dish; separator on the bench outside the dish;
    white plates in place; both grippers clear."""
    if not (mk.is_cut("w_target") and mk.is_cut("w_seat")):
        return False
    rel = R.target_in_dish_rel()
    if not (math.hypot(rel[0], rel[1]) < P.DISH_R - P.DISH_WALL and P.DISH_FLOOR_T - 0.002 <= rel[2] < P.DISH_H):
        return False
    corners, dish = R.separator_corners_w(), mk.pose("dish")[0]
    if min(math.hypot(c[0] - dish[0], c[1] - dish[1]) for c in corners) <= P.DISH_R or corners[:, 2].min() > TOP + SEP_ON_BENCH_M:
        return False
    return max(R.white_offsets_m()) <= WHITE_PLACE_M and R.gripper_clearance_m() >= GRIPPER_CLEAR_M


def at_rest() -> bool:
    lin, ang = R.fastest_body()
    return lin < REST_LIN_MPS and ang < REST_ANG_RPS


@mk.on_step
def step() -> None:
    t = mk.now()
    dt = t - float(STATE["last_time"])
    STATE["last_time"] = t
    if not 0.0 < dt <= 0.25:
        dt = 0.05
    if _in_sim() and not STATE["tare_locked"] and t >= TARE_AT_S:
        STATE.update(tare_stop_nm=R.stop_moment_nm(), tare_locked=True)
    _seat_rule(t, press_force_n())
    _peel_rule(t)
    _pull_rule(t)
    _zero_rules(t)
    if STATE["zero_reason"] is None and end_state_ok() and at_rest():
        STATE["rest_held_s"] = round(float(STATE["rest_held_s"]) + dt, 6)
    else:
        STATE["rest_held_s"] = 0.0


def level() -> float:
    if STATE.get("zero_reason") is not None:
        return 0.0
    if not (mk.is_cut("w_target") and STATE.get("off_with_seat")):  # on the base, or off it without the separator
        return 0.25 if STATE.get("ever_seated") else 0.0
    if float(STATE.get("rest_held_s", 0.0)) >= REST_HOLD_S - 1e-9 and end_state_ok() and at_rest():
        return 1.0
    return 0.5


def score(objects) -> float:
    return level()


def success(objects) -> bool:
    return level() == 1.0


def report() -> dict:
    return R.report(STATE, level() if mk.MECH.poses and mk.MECH.grippers else None)
