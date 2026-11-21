"""Uncapped Alvesco 160 inhaler on its side, its cap beside it, a euro 1 cent coin wedged edge first in the round
mouthpiece, three loose coins, an empty tray. Instruction: put the cap back on the inhaler and put the inhaler in the
tray. Product figures and their sources: parts.py; bilateral cap seat, click and contact sensor: capfit.py; the
coin's bilateral hold in the lip and its pinch-and-draw release: lodge.py."""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import capfit as F  # noqa: E402
import kit_extra as kx  # noqa: E402
import lodge as L  # noqa: E402
import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402
from mechkit import body, matrix_to_quat  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 2.0  # the capped inhaler still in the tray, the hands away, for two seconds (the spec's last frame)
INSTRUCTION = "Put the cap back on the inhaler and put the inhaler in the tray."
RUN_LIMIT_STEPS = 1920  # 96 s at 20 Hz: the reference run (67 s) and 40 % more
TOP = TABLE_TOP[SCENE]
OVER = 0.001  # every loose thing is built 1 mm over what it rests on, so that it is drawn from the first frame


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# --- where things lie (m; x away from the arms, y to the left) ---
COIN_AT = np.array([0.40, -0.29])  # the wedged coin's middle, in front of the right wrist camera at the ready pose
LYING_OUT = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])  # I's x (mouthpiece) to -y, y up, z (barrel) to -x
INHALER_ROT = _rz(math.radians(-60.0)) @ LYING_OUT  # turned 60 deg: the mouthpiece points 30 deg right of -x, at the right wrist camera;
# the right hand takes it from above and turns it to point at the arms within its wrist's turn (demo.py)
CAP_AT = np.array([0.16, 0.06])
CAP_LYING = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [-1.0, 0.0, 0.0]])  # open side down
TRAY_AT = np.array([0.20, -0.42])  # by the right arm, which holds the inhaler (it does not reach the left side)
LOOSE = [(0.355, -0.285), (0.345, -0.195), (0.455, -0.205)]  # three loose coins lying flat within 6 cm of the inhaler

_coin_in_i = P.coin_lodged()
INHALER_POS = np.array([*(COIN_AT - (INHALER_ROT @ _coin_in_i)[:2]), TOP + P.BARREL_R + OVER])
COIN_POS = INHALER_POS + INHALER_ROT @ _coin_in_i
COIN_ROT = INHALER_ROT @ P.COIN_IN_I  # edge first: its faces level, its plane through the mouthpiece's axis

_body_pos, _body_rot = F.body_pose(INHALER_POS, INHALER_ROT)
body("inhaler", F.inhaler_body_shapes(), pos=tuple(_body_pos), quat=matrix_to_quat(_body_rot), mass=P.INHALER_MASS, contact_offset=P.THIN)
body("coin", P.lodged_shapes(), pos=tuple(COIN_POS), quat=matrix_to_quat(COIN_ROT), mass=P.LODGED_MASS, friction=(P.COIN_FRICTION, P.COIN_FRICTION),
     friction_combine="min", contact_offset=P.THIN, contacts=True)  # fmt: skip
body("cap", P.cap_shapes(), pos=(CAP_AT[0], CAP_AT[1], TOP + 0.5 * P.CAP_DEPTH + OVER), quat=matrix_to_quat(CAP_LYING), mass=P.CAP_MASS, contact_offset=P.THIN)
for k, (x, y) in enumerate(LOOSE):
    body(f"coin_{k + 1}", P.coin_shapes(), pos=(x, y, TOP + 0.5 * P.COIN_T + OVER), mass=P.COIN_MASS, friction=(P.COIN_FRICTION, P.COIN_FRICTION),
         friction_combine="min", contact_offset=P.THIN)  # fmt: skip
body("tray", P.tray_shapes(), pos=(TRAY_AT[0], TRAY_AT[1], TOP + OVER), mass=P.TRAY_MASS)

F.declare()  # the cap's seat, click and bilateral contact sensor (capfit.py)
L.declare()  # the coin held bilaterally in the lip until a pinch draws it out (lodge.py)

OBJECTS = list(mk.MECH.bodies.values())

# --- the rules ---
COINS = ("coin", "coin_1", "coin_2", "coin_3")  # the wedged coin and the loose ones: none may be in the mouthpiece at the end
CLEAR = 0.03  # m from a gripper's grasp point to the inhaler's surface: let go
RESTING = 0.02  # m/s
TRAY_UPRIGHT = math.radians(30.0)  # the tray stands on its floor: its up within this of the vertical
OFF_BENCH = 0.01  # m: the inhaler, the cap or the wedged coin with its middle this far under the bench top has left the bench
EVENTS = kx.Events({"coin out": 0.5, "cap seated": 0.5})  # hold times (s): a reading of one step does not count
STATE: dict = {}


def begin() -> None:
    STATE.clear()
    STATE.update(events=[], off=None)
    EVENTS.clear()


begin()


@mk.on_reset
def start() -> None:
    begin()
    F.reset()
    L.reset()


rel, gap, seated = F.rel, F.gap, F.seated


def coins_in() -> list[str]:
    """The coins that reach into the mouthpiece (capfit.coin_in)."""
    return [name for name in COINS if F.coin_in(name)]


def coin_in() -> bool:
    return bool(coins_in())


@mk.on_step
def step() -> None:
    L.watch(STATE["events"])
    F.watch(STATE["events"])
    if STATE["off"] is None:
        for name, z in (("inhaler", F.i_pose()[0][2]), ("cap", mk.pose("cap")[0][2]), ("coin", mk.pose("coin")[0][2])):
            if z < TOP - OFF_BENCH:
                STATE["off"] = (mk.now(), name)
                STATE["events"].append((mk.now(), f"the {name} left the bench"))
                break
    now = mk.now()
    EVENTS.note("coin out", not coin_in(), now)
    EVENTS.note("cap seated", seated(), now)


def _capsules() -> list[tuple[np.ndarray, np.ndarray, float]]:
    """The inhaler as capsules (end, end, radius) in the world: barrel with canister, mouthpiece with or without its cap."""
    p, r = F.i_pose()
    w = lambda local: p + r @ np.asarray(local, float)  # noqa: E731
    out = [(w([0.0, 0.0, P.BOOT_LOW]), w([0.0, 0.0, P.CAN_TOP]), P.BARREL_R), (w([P.BARREL_R, 0.0, 0.0]), w([P.MOUTH_X, 0.0, 0.0]), P.MOUTH_R_OUT)]
    if seated():
        c0 = P.cap_x_at_gap(0.0)
        out.append((w([c0 - 0.5 * P.CAP_DEPTH, 0.0, 0.0]), w([c0 + 0.5 * P.CAP_DEPTH, 0.0, 0.0]), P.CAP_OUT_R))
    return out


def _to_capsule(point, a, b, radius) -> float:
    ab = b - a
    t = min(1.0, max(0.0, float(np.dot(point - a, ab) / np.dot(ab, ab))))
    return float(np.linalg.norm(point - (a + t * ab)) - radius)


def let_go() -> bool:
    caps = _capsules()
    return all(min(_to_capsule(mk.gripper(side)[0], *c) for c in caps) >= CLEAR for side in ("left", "right"))


def at_rest() -> bool:
    return float(np.linalg.norm(mk.velocity("inhaler")[0])) <= RESTING


def in_tray() -> bool:
    """The tray stands on its floor; the boot's two ends and the mouth's end (on their axes) are inside its walls, over
    its floor, and the lowest of them is under its rim: the inhaler lies or stands in it, not held over it."""
    tp, tr = mk.pose("tray")
    if tr[2, 2] < math.cos(TRAY_UPRIGHT):
        return False
    p, r = F.i_pose()
    inner = np.array([0.5 * P.TRAY_IN_L, 0.5 * P.TRAY_IN_W])
    q = np.array([tr.T @ (p + r @ np.asarray(local, float) - tp) for local in ([0.0, 0.0, P.BOOT_LOW], [0.0, 0.0, P.CAN_TOP], [P.MOUTH_X, 0.0, 0.0])])
    return bool(np.all(np.abs(q[:, :2]) <= inner) and np.all(q[:, 2] >= 0.0) and q[:, 2].min() <= P.TRAY_H)


def score(objects) -> float:
    """0 if the inhaler, the cap or the coin from the mouthpiece has left the bench at any moment, or if a coin is in
    the mouthpiece (a coin standing in it keeps the cap from seating). With none in it: 1.0 for the cap seated, the
    inhaler at rest in the tray and both grippers 3 cm or more from it; 0.5 for the cap seated otherwise (not in the
    tray, or still held); 0.25 for the cap not seated."""
    if STATE["off"] is not None or coin_in():
        return 0.0
    if seated():
        return 1.0 if in_tray() and at_rest() and let_go() else 0.5
    return 0.25


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out: dict = {"events": [f"{t:.2f} s {what}" for t, what in STATE.get("events", [])], "order": EVENTS.report(), "left the bench": STATE.get("off"),
                 "coin drawn out (t, hand, N carried, m drawn)": L.STATE.get("released"), "most N a pinch carried while it was held": round(L.STATE.get("most_carry", 0.0), 3),
                 "most push of the cap on the coin (N)": round(L.STATE.get("most_cap_push", 0.0), 3), "cap-inhaler max force (N)": round(F.STATE.get("most_force", 0.0), 3)}  # fmt: skip
    if mk.MECH.poses:
        out.update({"gap (mm)": round(1000 * gap(), 2), "seated": seated(), "coins in": coins_in(), "in the tray": in_tray(),
                    "coin in I (mm)": (1000 * rel("coin")[0]).round(2).tolist()})  # fmt: skip
    return out
