"""An M10 nylon-insert locknut (DIN 985, 17 mm across flats) on a screw standing up through a steel plate, and a
Bahco 8071 adjustable wrench lying round it with its jaw open 23 mm, wider than the nut's corners (18.9 mm).

Instruction: roll the wrench's knurled thumbwheel until the jaw is snug on the nut's flats, loosen the nut at least
60 degrees counterclockwise with it, and let go of the wrench.

Mechanics (figures and their sources in ``parts.py``):

- The nut turns on its screw against a prevailing torque of 1.5 N m (ISO 2320's first-removal minimum for M10): a
  stiff spring capped at 1.5 N m whose rest follows the nut, so it holds where it is left and slips at the cap. The
  torque reaches the nut only through the jaws' contacts on its flats or corners.
- The thumbwheel (knurl) turns freely on its axle against a little damping, rolled by friction with whatever presses
  on it. Its turns set where the movable jaw is driven to (2.9 mm a turn): the jaw goes there through a spring capped
  at the snug force (4 N), and it cannot be pushed open more than 0.2 mm past the tightest place the knurl set it to
  (a worm does not run back). A knurl rolled on against a jaw already snug on the nut slips past the cap. While the
  nut presses the jaw with more than 20 N the worm is locked: the knurl's turns do not move the jaw, and a stiff
  spring holds the jaw where the load found it.
- A jaw wider than the nut's corners spins round it; any jaw narrower turns the nut, after some free swing. "Snug"
  (17.5 mm or less on the jaw's scale) is scored as the practice that bears on the flats and keeps the corners from
  rounding at real loosening torques; it is not needed to move a 1.5 N m nut.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 2.0
INSTRUCTION = (
    "Roll the adjustable wrench's thumbwheel until its jaw is snug on the nut's flats, loosen the nut at least "
    "60 degrees counterclockwise with it, and let go of the wrench."
)
RUN_LIMIT_STEPS = 1800  # 90 s at 20 Hz
TOP = TABLE_TOP[SCENE]

START = P.declare_scene(TOP)
OBJECTS = list(mk.MECH.bodies.values())
PLATE_TOP = START["plate_top"]
STATE: dict = {}


def _sim(name: str) -> bool:
    return name in mk.MECH.joint_prims


def nut_deg() -> float:
    """How far the nut has turned counterclockwise seen from above (deg); negative: tightened."""
    return math.degrees(float(mk.joint("nut_thread")))


def jaw_m() -> float:
    """The jaw opening (m), as the jaw's scale reads it."""
    return P.JAW_START + float(mk.joint("jaw"))


def jaw_mm() -> float:
    return 1000.0 * jaw_m()


def knurl_turns() -> float:
    return float(mk.joint("knurl")) / (2 * math.pi)


def commanded_m(theta: float, slip: float) -> float:
    """Where the knurl's turns drive the jaw: closing as the knurl's top rolls towards the jaw tips."""
    return float(np.clip(P.JAW_START - P.KNURL_PITCH * (theta - slip) / (2 * math.pi), 0.0, P.JAW_MAX))


def snug() -> bool:
    return jaw_mm() <= P.SNUG_MM + 1e-9


def snug_turn() -> bool:
    """Was the jaw set snug (by the knurl, unloaded) when the nut first passed TURN_LATCH_DEG counterclockwise? A turn
    begun with a loose jaw stays loose in the score, whatever is rolled after."""
    w = STATE.get("turn_jaw")
    return w is not None and 1000.0 * w <= P.SNUG_MM + 1e-9


def loosened() -> bool:
    return nut_deg() >= P.LOOSEN_DEG - 1e-9


def wrench_pose() -> tuple[np.ndarray, np.ndarray]:
    return mk.pose("wrench")


def clear_of_nut() -> bool:
    o, rw = wrench_pose()
    middle = P.jaw_middle(o, rw, jaw_m())
    return bool(math.hypot(middle[0] - P.NUT_XY[0], middle[1] - P.NUT_XY[1]) >= P.CLEAR_M)


def nut_in_jaws() -> np.ndarray:
    """The nut's axis in the wrench's frame (m): x along the jaws from a seated nut's place (+ towards the tips),
    y from the fixed jaw's face, z from mid-thickness."""
    o, rw = wrench_pose()
    return rw.T @ (np.array([*P.NUT_XY, START["nut_z"]]) - o)


def wrench_yaw_deg() -> float:
    _, rw = wrench_pose()
    _, r0 = START["wrench"]
    return math.degrees(math.atan2(rw[1, 0], rw[0, 0]) - math.atan2(r0[1, 0], r0[0, 0]))


def on_plate() -> bool:
    """The jaws' middle and the handle's end over the plate, the wrench lying on it (not lifted)."""
    o, rw = wrench_pose()
    pts = [P.jaw_middle(o, rw, jaw_m()), o + rw @ P.handle_point_w(P.HANDLE_END_S)]
    half = np.array(P.PLATE_SIZE) / 2
    return all(bool(np.all(np.abs(pt[:2] - np.array(P.PLATE_XY)) <= half) and pt[2] <= PLATE_TOP + P.ON_PLATE_M) for pt in pts)


def flat() -> bool:
    _, rw = wrench_pose()
    return bool(math.degrees(math.acos(float(np.clip(abs(rw[2, 2]), -1.0, 1.0)))) <= P.FLAT_DEG)


def hand_gap(side: str) -> float:
    """Distance (m) from a gripper's grasp point to the wrench's and the nut's surfaces."""
    point = mk.gripper(side)[0]
    o, rw = wrench_pose()
    gaps = [P.to_capsule(point, a, b, r) for a, b, r in P.wrench_segments(o, rw, jaw_m())]
    nut = np.array([*P.NUT_XY, START["nut_z"]])
    gaps.append(P.to_capsule(point, nut - [0, 0, P.NUT_H / 2], nut + [0, 0, P.NUT_H / 2], P.NUT_E / 2))
    return min(gaps)


def hands_clear() -> bool:
    return all(hand_gap(side) >= P.HANDS_CLEAR_M for side in ("left", "right"))


def still_reading() -> tuple[np.ndarray, float, float]:
    """What must stay still at the end: the wrench's place and turn, and the nut's turn."""
    return np.asarray(wrench_pose()[0], dtype=float), wrench_yaw_deg(), nut_deg()


def moved_from(ref) -> bool:
    """Has the wrench or the nut moved from ``ref`` (more than 1 mm or 0.5 deg)? Solver chatter on the jointed bodies
    reads as velocity while nothing moves, so rest is read from their positions."""
    pos, yaw, nut = still_reading()
    return bool(np.linalg.norm(pos - ref[0]) > P.REST_MOVE_M or abs(yaw - ref[1]) > P.REST_TURN_DEG or abs(nut - ref[2]) > P.REST_TURN_DEG)


def settled() -> bool:
    since = STATE.get("rest_since")
    return since is not None and mk.now() - since >= P.REST_HOLD_S - 1e-9


def begin() -> None:
    STATE.clear()
    STATE.update(nut_rest=0.0, slip=0.0, w_min=P.JAW_START, k_prev=0.0, loaded=False, lock_w=None, events=[], tightened=None,
                 rest_since=None, rest_ref=None, turn_jaw=None, most_jaw_n=0.0, most_head_n=0.0, snug_at=None, loosened_at=None, clear_at=None)  # fmt: skip
    if _sim("nut_thread"):  # the simulator keeps springs, drives and stops through a reset: put them back
        mk.set_spring("nut_thread", rest=0.0, most=P.PREVAILING_TORQUE)
        mk.set_drive("jaw", stiffness=P.JAW_SPRING, damping=P.JAW_DAMPING)
        mk.set_spring("jaw", rest=0.0, most=P.SNUG_FORCE)
        mk.set_limits("jaw", -P.JAW_START + 0.001, P.JAW_PLAY)


begin()


@mk.on_reset
def start() -> None:
    begin()


def note(what: str) -> None:
    STATE["events"].append((round(mk.now(), 2), what))


@mk.on_step
def step() -> None:
    now = mk.now()
    # 1. The nut's prevailing torque: the capped spring's rest follows the nut, at most its give behind it.
    theta = float(mk.joint("nut_thread"))
    STATE["nut_rest"] = float(np.clip(STATE["nut_rest"], theta - P.NUT_GIVE, theta + P.NUT_GIVE))
    if _sim("nut_thread"):
        mk.set_spring("nut_thread", rest=STATE["nut_rest"], most=P.PREVAILING_TORQUE)

    # 2. The knurl drives the jaw. 3. No back-drive: the jaw opens no further than PLAY past the tightest place it
    # reached; only the knurl, rolled to open, moves that lock open. A knurl rolled on against a jaw the nut holds
    # slips once the spring is at its cap (no wind-up); a jaw pushed open by the nut moves neither the lock nor the knurl.
    k, w = float(mk.joint("knurl")), jaw_m()
    loaded = mk.contact_force("movable_jaw", "nut") > P.JAW_LOAD_N
    rolled = any(c.other in ("left robot", "right robot") for c in mk.contacts("knurl"))
    if loaded or not rolled:  # a loaded worm locks; and only a finger on the knurl rolls it (the solver lets an untouched
        STATE["slip"] += k - STATE["k_prev"]  # knurl creep while the wrench turns: stand-in)
    STATE["k_prev"], STATE["loaded"] = k, loaded
    STATE["lock_w"] = (STATE["lock_w"] if STATE["lock_w"] is not None else w) if loaded else None  # latched where the load found it
    STATE["w_min"] = min(STATE["w_min"], w)
    target = commanded_m(k, STATE["slip"])
    if target > STATE["w_min"]:
        STATE["w_min"] = target
    floor = STATE["w_min"] - P.SNUG_FORCE / P.JAW_SPRING
    if target < floor:
        STATE["slip"] += (floor - target) * 2 * math.pi / P.KNURL_PITCH
        target = floor
    if _sim("jaw"):
        if loaded:  # held at the lock by the worm (stiff spring; the joint stop alone gives too much under load)
            mk.set_drive("jaw", stiffness=P.JAW_LOCK_K, damping=P.JAW_LOCK_D)
            mk.set_spring("jaw", rest=STATE["lock_w"] + P.JAW_LOCK_GAP - P.JAW_START, most=P.JAW_LOCK_MOST)
        else:
            mk.set_drive("jaw", stiffness=P.JAW_SPRING, damping=P.JAW_DAMPING)
            mk.set_spring("jaw", rest=target - P.JAW_START, most=P.SNUG_FORCE)
        mk.set_limits("jaw", -P.JAW_START + 0.001, STATE["w_min"] + P.JAW_PLAY - P.JAW_START)

    # 4. The jaw's setting when the turn began: the knurl's (unloaded) setting, latched as the nut first passes 10 deg.
    if STATE["turn_jaw"] is None and nut_deg() > P.TURN_LATCH_DEG:
        STATE["turn_jaw"] = STATE["w_min"]
        note(f"turn begun with the jaw set to {1000 * STATE['w_min']:.2f} mm")

    # 5. Readings and the whole-trial faults.
    STATE["most_jaw_n"] = max(STATE["most_jaw_n"], mk.contact_force("movable_jaw", "nut"))
    STATE["most_head_n"] = max(STATE["most_head_n"], mk.contact_force("wrench", "nut"))
    if STATE["tightened"] is None and nut_deg() < P.TIGHTEN_LIMIT_DEG:
        STATE["tightened"] = round(now, 2)
        note(f"nut tightened past {P.TIGHTEN_LIMIT_DEG:.0f} deg")
    for key, met, what in (("snug_at", snug(), "jaw snug"), ("loosened_at", loosened(), "nut loosened 60 deg"), ("clear_at", clear_of_nut(), "wrench clear")):
        if met and STATE[key] is None:
            STATE[key] = round(now, 2)
            note(f"{what} ({jaw_mm():.2f} mm, nut {nut_deg():.1f} deg)")
    if hands_clear():
        if STATE["rest_since"] is None or moved_from(STATE["rest_ref"]):
            STATE["rest_since"], STATE["rest_ref"] = now, still_reading()
    else:
        STATE["rest_since"], STATE["rest_ref"] = None, None


def score(objects) -> float:
    """0 for the whole trial if the nut was tightened past -10 deg. At the end:
    1.0 nut loosened >= 60 deg, jaw snug (<= 17.5 mm), the wrench lying flat (<= 15 deg) on the plate, both hands
    >= 30 mm from wrench and nut, wrench and nut still (1 mm, 0.5 deg) for 1.0 s;
    0.5 nut loosened >= 60 deg with the jaw snug (also when the turn began), the rest not met;
    0.25 the jaw snug with the nut under 60 deg, or the nut loosened >= 60 deg with the jaw not snug at the end or
    when the turn began (the jaw's setting latched as the nut first passes 10 deg);
    0.0 otherwise."""
    del objects
    if STATE.get("tightened") is not None:
        return 0.0
    if loosened() and snug() and snug_turn():
        return 1.0 if flat() and on_plate() and hands_clear() and settled() else 0.5
    if snug() or loosened():
        return 0.25
    return 0.0


def success(objects) -> bool:
    return score(objects) == 1.0


def readings() -> dict:
    """One step's readings for a run's log."""
    return {
        "nut_deg": round(nut_deg(), 2),
        "jaw_mm": round(jaw_mm(), 3),
        "turn_jaw_mm": None if STATE.get("turn_jaw") is None else round(1000 * STATE["turn_jaw"], 3),
        "knurl_turns": round(knurl_turns(), 3),
        "slip_turns": round(STATE.get("slip", 0.0) / (2 * math.pi), 3),
        "jaw_on_nut_N": round(mk.contact_force("movable_jaw", "nut"), 2),
        "head_on_nut_N": round(mk.contact_force("wrench", "nut"), 2),
        "clear": clear_of_nut(),
        "nut_in_jaws_mm": [round(1000 * float(v), 2) for v in nut_in_jaws()],
        "wrench_yaw_deg": round(wrench_yaw_deg(), 2),
        "head_up_mm": round(1000 * (float(wrench_pose()[0][2]) - float(START["wrench"][0][2])), 2),
        "flat": flat(),
        "on_plate": on_plate(),
        "gap_left_mm": round(1000 * hand_gap("left"), 1) if "left" in mk.MECH.grippers else None,
        "gap_right_mm": round(1000 * hand_gap("right"), 1) if "right" in mk.MECH.grippers else None,
        "score": score(None),
        "nut_touches": nut_touches(),
    }


def nut_touches() -> list:
    """The contact points on the nut over 0.5 N: other body, point in the nut's frame (mm), normal, separation (mm), N."""
    if not mk.MECH.contact_points.get("nut"):
        return []
    p, r = mk.pose("nut")
    return [[c.other, [round(1000 * float(v), 2) for v in r.T @ (np.asarray(c.point) - p)], [round(float(v), 2) for v in r.T @ np.asarray(c.normal)],
             round(1000 * c.separation, 2), round(c.force, 1)] for c in mk.contacts("nut") if c.force > 0.5]  # fmt: skip


def report() -> dict:
    out = {k: STATE.get(k) for k in ("tightened", "snug_at", "loosened_at", "clear_at")}
    out["events"] = [f"{t:.2f} s {what}" for t, what in STATE.get("events", [])]
    out["most_jaw_on_nut_N"] = round(STATE.get("most_jaw_n", 0.0), 2)
    out["most_head_on_nut_N"] = round(STATE.get("most_head_n", 0.0), 2)
    if mk.MECH.poses:
        out.update(readings())
        out["settled_1s"] = settled()
    return out
