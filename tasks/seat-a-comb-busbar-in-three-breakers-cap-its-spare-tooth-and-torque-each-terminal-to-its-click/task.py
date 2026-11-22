"""Three DIN-rail circuit breakers on a rail that our fixture leans back 30 deg (so their screws face the arms), top
terminals open; a 4-tooth comb busbar on a rack, a tooth cap in its holder, a T-handle torque driver in its holster.

Mechanism (mechkit 1.0; figures and shapes in ``parts.py``):
- ``panel`` (fixed): our fixture's leaning plate, the DIN rail, three breakers. Each top terminal has a busbar slot
  from the top face (its mouth chamfered) down to a clamp, and a countersunk access hole on the front round the screw.
- ``jaw_1..3`` (kinematic): each terminal's clamp plate. It stands ``clamp_gap(turn)`` in front of the slot's back
  wall: 1.9 mm at the start, half a turn of its M5 x 0.8 screw past a 1.5 mm tooth; each turn of the screw moves it
  0.8 mm. On a tooth in the slot it stops at the tooth; an empty clamp closes on, and then no tooth goes in.
- ``screw_1..3`` (kinematic, drawn only): each screw's head, turned with the screw.
- The driver turns a screw while its bit's tip stands in that screw's access hole, at the head (``ENGAGE_*``), its
  axis along the screw's: the screw turns as the driver turns about that axis. Tightening needs ``screw_torque``:
  0.10 N m until the plate touches the tooth, then rising 7.3 N m/rad; at the set 2.0 N m the driver's clutch slips
  and the screw turns no further. Slipping on 3 deg at the set torque is the click, recorded for that screw; turning
  it back (past the bit's 4 deg of play in the recess) loosens it and clears the click. The screw's torque acts on ``driver`` against its turn
  (``push``, x tanh(speed / 0.3 rad/s), eased in over about 0.15 s); its reaction is on the fixed ``panel`` through the screw.
- Holds: (clamp) when the first plate presses its tooth, ``weld("comb_hold", "comb", "panel")`` joins the comb to
  the panel where it stands, cut when every plate is turned back off its tooth; (cap) a tooth pushed 22 mm into
  ``cap`` makes ``weld("cap_on", "cap", "comb")`` (its catch); nothing in the task takes it off.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import kit_extra as kx  # noqa: E402
import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402
from mechkit import body, matrix_to_quat, weld  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
INSTRUCTION = ("Put the grey cap on the comb busbar's spare tooth, seat its other three teeth in the three breakers' top "
               "terminals, and tighten each terminal screw with the torque driver until it clicks, then let go of the comb "
               "and keep both hands still and clear of it.")
RUN_LIMIT_STEPS = 4000  # 200 s at 20 Hz
TOP = TABLE_TOP[SCENE]
DT = 0.05
AXIS = P.XP  # the screws' axis (into the breakers), world


def _w(p) -> tuple[float, float, float]:
    """World point of a point given with z from the bench."""
    return (float(p[0]), float(p[1]), float(p[2]) + TOP)


def _bench(p) -> np.ndarray:
    return np.asarray(p, float) - np.array([0.0, 0.0, TOP])


COMB_START = _w(P.rack_origin() + P.TILT @ np.array([0.0, 0.0, 0.5 * P.COMB_BAR[2] + P.OVER]))
CAP_START = _w(P.cap_origin() + P.TILT @ np.array([0.0, 0.0, 0.5 * P.CAP_SIZE[2] + P.OVER]))
DRIVER_START = _w(P.driver_start()[0])
LEAN = P.TILT_QUAT

body("panel", P.panel_shapes(), pos=_w(P.PANEL_ORIGIN), quat=LEAN, contact_offset=P.CONTACT_THIN)
for _i in range(3):
    body(f"jaw_{_i + 1}", P.jaw_shapes(), pos=_w(P.pw(P.jaw_pos(_i, P.GAP0))), quat=LEAN, kinematic=True, contact_offset=P.CONTACT_THIN)
    body(f"screw_{_i + 1}", P.screw_shapes(), pos=_w(P.pw(P.screw_pos(_i))), quat=LEAN, kinematic=True)
body("stand", P.stand_shapes(), pos=(0.0, 0.0, TOP))
body("holster", P.holster_shapes(), pos=(0.0, 0.0, TOP))
body("comb", P.comb_shapes(), pos=COMB_START, quat=LEAN, mass=P.COMB_MASS, contact_offset=P.CONTACT_THIN)
body("cap", P.cap_shapes(), pos=CAP_START, quat=LEAN, mass=P.CAP_MASS, contact_offset=P.CONTACT_THIN)
body("driver", P.driver_shapes(), pos=DRIVER_START, quat=matrix_to_quat(P.driver_start()[1]), mass=P.DRIVER_MASS)
weld("cap_on", "cap", "comb", made=False)
weld("comb_hold", "comb", "panel", made=False)

OBJECTS = list(mk.MECH.bodies.values())
LOOSE = ("comb", "cap", "driver")
EVENTS = kx.Events({"cap on": 0.0, "comb seated": 0.3, "screw 1 clicked": 0.0, "screw 2 clicked": 0.0, "screw 3 clicked": 0.0})
STATE: dict = {}


def begin() -> None:
    STATE.clear()
    STATE.update(turn=[0.0, 0.0, 0.0], slip=[0.0, 0.0, 0.0], lash=[0.0, 0.0, 0.0], clicked=[False, False, False], gap=[P.GAP0] * 3, cap_on=None,
                 driver_rot=None, engaged=None, fault=None, rest_since=None, last_poses=None, peak_torque=[0.0, 0.0, 0.0], torque=0.0,
                 events=[])  # fmt: skip
    EVENTS.clear()


begin()


@mk.on_reset
def start() -> None:
    begin()


# --- readings ----------------------------------------------------------------------------------------------------
def tooth_tip(k: int) -> np.ndarray:
    cp, cr = mk.pose("comb")
    return cp + cr @ P.tooth_tip_local(k)


def comb_upright() -> bool:
    return bool(float(np.dot(mk.pose("comb")[1][:, 2], P.ZP)) >= math.cos(math.radians(8.0)))


def tooth_in_slot(i: int, depth: float) -> int | None:
    """The comb's tooth standing in breaker ``i``'s busbar slot with its tip at least ``depth`` under the top face."""
    if not comb_upright():
        return None
    for k in range(P.COMB_TEETH):
        t = P.to_panel(_bench(tooth_tip(k)))
        if abs(t[0] - P.X_PIN) <= P.SEAT_LATERAL[0] and abs(t[1] - P.MCB_Y[i]) <= P.SEAT_LATERAL[1] and t[2] <= P.MCB_TOP - depth:
            return k
    return None


def tooth_in_clamp(i: int) -> bool:
    return tooth_in_slot(i, P.MCB_TOP - P.CLAMP_Z[1] + 0.003) is not None


def seated() -> dict[int, int] | None:
    """{breaker: tooth} when a tooth stands in every breaker's slot at least SEAT_DEPTH deep, else None."""
    found = {i: tooth_in_slot(i, P.SEAT_DEPTH) for i in range(3)}
    return {i: k for i, k in found.items() if k is not None} if all(k is not None for k in found.values()) else None


def spare_tooth() -> int | None:
    s = seated()
    if s is None:
        return None
    left = sorted(set(range(P.COMB_TEETH)) - set(s.values()))
    return left[0] if len(left) == 1 else None


def closed_on_tooth(i: int) -> bool:
    return bool(tooth_in_clamp(i) and STATE["turn"][i] >= P.touch_turn(True) - 1e-6)


def engaged_screw() -> int | None:
    """The screw whose access hole holds the driver's bit tip at its head, the driver along its axis."""
    dp, dr = mk.pose("driver")
    if float(np.dot(dr[:, 0], AXIS)) < math.cos(math.radians(P.ENGAGE_ANGLE_DEG)):
        return None
    tip = P.to_panel(_bench(dp + dr @ np.array([P.TIP_X, 0.0, 0.0])))
    for i in range(3):
        sp = P.screw_pos(i)
        if P.ENGAGE_DEPTH[0] <= tip[0] - sp[0] <= P.ENGAGE_DEPTH[1] and math.hypot(tip[1] - sp[1], tip[2] - sp[2]) <= P.ENGAGE_RADIAL:
            return i
    return None


def cap_tooth() -> int | None:
    """A tooth whose tip is CAP_SNAP_DEPTH into the cap's socket, on its centre line."""
    pp, pr = mk.pose("cap")
    cr = mk.pose("comb")[1]
    if float(np.dot(pr[:, 2], cr[:, 2])) < math.cos(math.radians(P.CAP_SNAP_ANGLE_DEG)):
        return None
    for k in range(P.COMB_TEETH):
        q = pr.T @ (tooth_tip(k) - pp)
        if abs(q[0]) <= P.CAP_SNAP_LATERAL and abs(q[1]) <= P.CAP_SNAP_LATERAL and 0.5 * P.CAP_SIZE[2] - q[2] >= P.CAP_SNAP_DEPTH:
            return k
    return None


# --- the screws ----------------------------------------------------------------------------------------------------
def _turn_screw(i: int, d_turn: float) -> float:
    """Turn screw ``i`` by the driver's ``d_turn`` (rad, + tightens), less the bit's play in the recess (``lash``: 0 on
    the tightening flanks, -BIT_PLAY on the loosening ones); returns the torque it puts on the driver."""
    lash = STATE["lash"][i]
    take = min(d_turn, -lash) if d_turn >= 0.0 else max(d_turn, -math.radians(P.BIT_PLAY_DEG) - lash)
    STATE["lash"][i] = lash + take
    d_turn -= take
    if d_turn == 0.0:
        return 0.0
    tooth = tooth_in_clamp(i)
    t = STATE["turn"][i]
    if d_turn >= 0.0:
        gone = min(d_turn, max(0.0, P.click_turn(tooth) - t))
        STATE["turn"][i] = t + gone
        if d_turn - gone > 0.0:  # the clutch slips at the set torque
            STATE["slip"][i] += d_turn - gone
            if not STATE["clicked"][i] and math.degrees(STATE["slip"][i]) >= P.CLICK_SLIP_DEG:
                STATE["clicked"][i] = True
                STATE["events"].append((mk.now(), f"screw {i + 1} clicked" + ("" if tooth else " (no tooth)")))
            return P.TORQUE_SET
        return P.screw_torque(STATE["turn"][i], tooth)
    STATE["turn"][i] = max(t + d_turn, P.TURN_MIN)
    if STATE["turn"][i] < P.click_turn(tooth) - math.radians(2.0):
        STATE["slip"][i], STATE["clicked"][i] = 0.0, False
    return P.TORQUE_LOOSEN if t > P.touch_turn(tooth) else P.TORQUE_RUN


def _drive() -> None:
    dr = mk.pose("driver")[1]
    prev, STATE["driver_rot"] = STATE["driver_rot"], dr
    i = engaged_screw()
    STATE["engaged"] = i
    torque = 0.0
    if i is not None and prev is not None:
        d_turn = P.roll_about(prev, dr, AXIS)
        tq = _turn_screw(i, d_turn)
        torque = -tq * math.tanh((d_turn / DT) / P.SPIN_SCALE)
        STATE["peak_torque"][i] = max(STATE["peak_torque"][i], tq)
    STATE["torque"] += P.TORQUE_LAG * (torque - STATE["torque"])
    if mk.MECH.environment is not None:
        mk.push("driver", torque=tuple(STATE["torque"] * AXIS))
    for j in range(3):
        gap = P.clamp_gap(STATE["turn"][j], tooth_in_clamp(j))
        if mk.MECH.environment is not None:
            if abs(gap - STATE["gap"][j]) > 1e-7:
                mk.write_pose(f"jaw_{j + 1}", _w(P.pw(P.jaw_pos(j, gap))), LEAN)
            mk.write_pose(f"screw_{j + 1}", _w(P.pw(P.screw_pos(j))), matrix_to_quat(P.TILT @ P.rot_x(STATE["turn"][j])))
        STATE["gap"][j] = gap


def _hold_comb() -> None:
    """The clamp's hold: the comb welded to the panel where it stands while any plate presses its tooth."""
    pressed = any(closed_on_tooth(i) for i in range(3))
    if pressed and mk.is_cut("comb_hold"):
        mk.make("comb_hold")
        STATE["events"].append((mk.now(), "comb held by its clamps"))
    elif not pressed and not mk.is_cut("comb_hold"):
        mk.cut("comb_hold")
        STATE["events"].append((mk.now(), "comb let go by its clamps"))


def _snap_cap() -> None:
    if STATE["cap_on"] is None:
        k = cap_tooth()
        if k is not None:
            mk.make("cap_on")
            STATE["cap_on"] = k
            STATE["events"].append((mk.now(), f"cap on tooth {k + 1}"))


# --- end state -----------------------------------------------------------------------------------------------------
def grippers_clear() -> bool:
    cp, cr = mk.pose("comb")
    centre = cp + cr @ np.array([0.0, 0.0, -0.5 * P.PIN_L])
    size = (P.COMB_BAR[0] + 0.004, P.COMB_L + 0.004, P.COMB_BAR[2] + P.PIN_L + 0.004)
    for side in ("left", "right"):
        q = cr.T @ (mk.gripper(side)[0] - centre)
        out = np.maximum(np.abs(q) - 0.5 * np.asarray(size), 0.0)
        if float(np.linalg.norm(out)) < P.GRIPPER_CLEAR:
            return False
    return True


def all_at_rest() -> bool:
    """Every loose part moved under REST_SPEED and turned under REST_SPIN since the last control step (read from its pose:
    a held comb, rewritten each step, and what is welded to it or pinched report solver speeds that are not motion)."""
    now = {name: mk.pose(name) for name in LOOSE}
    last, STATE["last_poses"] = STATE["last_poses"], now
    if last is None:
        return False
    for name in LOOSE:
        (p0, r0), (p1, r1) = last[name], now[name]
        turn = math.acos(min(1.0, max(-1.0, 0.5 * (float(np.trace(r1 @ r0.T)) - 1.0))))
        if float(np.linalg.norm(p1 - p0)) > P.REST_SPEED * DT or turn > P.REST_SPIN * DT:
            return False
    return True


def rested() -> bool:
    return bool(STATE["rest_since"] is not None and mk.now() - STATE["rest_since"] >= P.REST_HOLD_S - 1e-9)


@mk.on_step
def step() -> None:
    now = mk.now()
    if STATE["fault"] is None:
        for name in LOOSE:
            if mk.pose(name)[0][2] < TOP - P.OFF_BENCH:
                STATE["fault"] = (now, f"{name} left the bench")
                STATE["events"].append(STATE["fault"])
                break
    _drive()
    _hold_comb()
    _snap_cap()
    if not all_at_rest():
        STATE["rest_since"] = None
    elif STATE["rest_since"] is None:
        STATE["rest_since"] = now
    EVENTS.note("cap on", STATE["cap_on"] is not None, now)
    EVENTS.note("comb seated", seated() is not None, now)
    for i in range(3):
        EVENTS.note(f"screw {i + 1} clicked", STATE["clicked"][i], now)


def score(_objects) -> float:
    """0.0 after a whole-trial fault, or unless the comb has a tooth in every breaker's slot with every clamp closed on
    its tooth, both grippers 30 mm clear of the comb and every loose body at rest for 1.0 s; then 1.0 if every screw
    was tightened until the driver clicked and the cap is on the spare tooth, else 0.5."""
    if STATE["fault"] is not None or not rested() or not grippers_clear():
        return 0.0
    if seated() is None or not all(closed_on_tooth(i) for i in range(3)):
        return 0.0
    capped = STATE["cap_on"] is not None and STATE["cap_on"] == spare_tooth()
    return 1.0 if (all(STATE["clicked"]) and capped) else 0.5


def success(objects) -> bool:
    return score(objects) == 1.0
