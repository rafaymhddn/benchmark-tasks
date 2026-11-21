"""Bilateral cap fit and seat on the inhaler's mouthpiece (parts.py), for task.py and cap_task.py."""

from __future__ import annotations

import dataclasses
import math

import mechkit as mk
import numpy as np
import parts as P
from mechkit import matrix_to_quat, quat_to_matrix, sense_contact, weld

MECHKIT_HELPER = True  # mechkit counts this module's declarations as the task file's own

# The inhaler's body frame B, turned from the world about the vertical only (a weld on a body built rolled held about
# ten times looser in spike_make, run w9k6); the inhaler's own frame I (parts.py) is B turned by I_IN_B and moved.
BODY_IN_I = np.array([0.010, 0.0, 0.030])  # B's origin in I: inside the boot's solid, 30 mm up the barrel
I_IN_B = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])  # I's axes in B: lying on its side, the long axis of the opening up
SEAT_GAP = P.SEAT_GAP
SEAT_AXIS, SEAT_OFF = math.radians(P.SEAT_AXIS_DEG), P.SEAT_OFF

STATE: dict = {}


def body_pose(i_pos, i_rot) -> tuple[np.ndarray, np.ndarray]:
    """The inhaler body's frame B in the world for its frame I at ``i_pos``, ``i_rot``."""
    i_pos, i_rot = np.asarray(i_pos, float), np.asarray(i_rot, float)
    return i_pos + i_rot @ BODY_IN_I, i_rot @ I_IN_B.T


def inhaler_body_shapes() -> list:
    """parts.inhaler_shapes() (in I) put into the body frame B."""
    out = []
    for s in P.inhaler_shapes():
        pos = I_IN_B @ (np.asarray(s.pos, float) - BODY_IN_I)
        out.append(dataclasses.replace(s, pos=tuple(float(v) for v in pos), quat=tuple(matrix_to_quat(I_IN_B @ quat_to_matrix(s.quat)))))
    return out


def i_pose() -> tuple[np.ndarray, np.ndarray]:
    """The inhaler's frame I in the world, from its body's pose."""
    p, r = mk.pose("inhaler")
    r_i = r @ I_IN_B
    return p - r_i @ BODY_IN_I, r_i


def declare(coins: tuple[str, ...] = ("coin", "coin_1", "coin_2", "coin_3")) -> None:
    """The seat weld and bilateral contact sensor between ``cap`` and ``inhaler`` (both declared before)."""
    weld("cap_seat", "cap", "inhaler", made=False)
    sense_contact("cap", "inhaler")
    STATE["coins"] = tuple(c for c in coins if c in mk.MECH.bodies)
    reset()


def rel(name: str) -> tuple[np.ndarray, np.ndarray]:
    """Body ``name``'s position and rotation in the inhaler's frame I."""
    p, r = i_pose()
    q, s = mk.pose(name)
    return r.T @ (q - p), r.T @ s


def seated() -> bool:
    """The cap is welded at its seat."""
    return not mk.is_cut("cap_seat")


def gap() -> float:
    """m remaining along the mouthpiece to the cap's seat (0: pushed home by ``CAP_SLIDE``)."""
    return float(rel("cap")[0][0] - P.cap_x_at_gap(0.0))


def lined_up() -> bool:
    """The cap's axis along the mouthpiece's, its opening towards the inhaler, its middle on the mouthpiece's axis."""
    p, r = rel("cap")
    return bool(-r[0, 0] >= math.cos(math.radians(P.SEAT_AXIS_DEG)) and math.hypot(p[1], p[2]) <= P.SEAT_OFF)


def coin_reach(name: str = "coin") -> tuple[np.ndarray, float]:
    """Coin ``name``'s middle in I and how far its disc reaches along the mouthpiece from its middle."""
    radius, thick = P.COIN_R, P.COIN_T
    c, r = rel(name)
    along = min(1.0, abs(float(r[0, 2])))  # the coin's axis (its body's z) along the mouthpiece
    reach = radius * math.sqrt(1.0 - along**2) + 0.5 * thick * along
    return c, reach


def coin_in(name: str = "coin") -> bool:
    """Coin ``name`` reaches into the mouthpiece: its middle within the bore's circle across, in front of the nozzle,
    and less than its own reach past the mouth's end (an edge-first coin counts while any of it is inside the mouth)."""
    c, reach = coin_reach(name)
    return bool(P.NOZZLE_X <= c[0] < P.MOUTH_X + reach and float(np.hypot(c[1], c[2])) <= P.BORE_R)


def coins_in() -> list[str]:
    return [name for name in STATE.get("coins", ("coin",)) if name in mk.MECH.bodies and coin_in(name)]


def click_met() -> bool:
    """Whether the cap-to-inhaler push meets ``CAP_CLICK_FORCE`` when ``CAP_CLICKS`` is enabled."""
    if not P.CAP_CLICKS or P.CAP_CLICK_FORCE <= 0.0:
        return True
    return mk.contact_force("cap", "inhaler") >= P.CAP_CLICK_FORCE


def reset() -> None:
    """From the task's @on_reset function (mechkit's reset takes the seat's weld away by itself)."""
    global SEAT_GAP, SEAT_AXIS, SEAT_OFF
    SEAT_GAP, SEAT_AXIS, SEAT_OFF = P.SEAT_GAP, math.radians(P.SEAT_AXIS_DEG), P.SEAT_OFF
    coins = STATE.get("coins", ("coin",))
    STATE.clear()
    STATE.update(coins=coins, most_force=0.0, blocked_noted=False)


reset()


def watch(events: list) -> None:
    """Once per control step: welds a cap pushed home in line, past its click, with no coin in the mouthpiece (a coin
    standing in it stops the cap's closed end parts.STOP_SHORT short of the seat; the rule only confirms it)."""
    force = mk.contact_force("cap", "inhaler")
    STATE["most_force"] = max(STATE.get("most_force", 0.0), force)
    if seated() or not lined_up():
        return
    blocking = coins_in()
    if blocking:
        if not STATE.get("blocked_noted") and gap() <= P.STOP_SHORT + 0.002 and "cap" in mk.MECH.sensed.get("coin", ()) and mk.contact_force("coin", "cap") > 0.05:
            STATE["blocked_noted"] = True
            events.append((mk.now(), f"the cap's end meets the coin {1000 * gap():.1f} mm short of its seat: not seated"))
        return
    if gap() <= P.SEAT_GAP and click_met():
        mk.make("cap_seat")
        events.append((mk.now(), f"cap pushed home ({1000 * gap():.2f} mm from its seat, {force:.2f} N): clicked, seated"))
