"""The coin wedged edge first in the mouthpiece's lip. A bilateral weld ``coin``<->``inhaler`` stands in for the static
friction of its edge in the plastic lip (``parts.HOLD``, estimate). It lets go only to the stated action: a pinch of
one hand on both faces of the coin, whose pads can carry the hold (pad friction x squeeze >= HOLD), drawn
``parts.DRAW`` outward along the mouthpiece relative to the inhaler. While the weld holds, the pads slide on the
coin with that friction, which the weld passes to the inhaler and the hand that holds it. Nothing else frees the
coin: its weight, a jolt, the cap's push or a finger's push (the coin is wider than the bore: it cannot go in)."""

from __future__ import annotations

import capfit as F
import mechkit as mk
import numpy as np
import parts as P
from mechkit import sense_contact, weld

MECHKIT_HELPER = True  # mechkit counts this module's declarations as the task file's own

OWN = "inhaler"  # what the coin is wedged in
HANDS = {"left": "left robot", "right": "right robot"}
FACE = 0.0003  # m: a contact point this far or more off the coin's middle plane is on one of its faces
MIN_FACE = 0.1  # N on each face: the hand pinches the coin (both faces), not a one-sided touch

STATE: dict = {}


def declare(cap: str | None = "cap") -> None:
    """The bilateral weld between ``coin`` and ``inhaler``, and the coin's contact sensor for the cap."""
    weld("lodged", "coin", OWN)
    if cap is not None:
        sense_contact("coin", cap)
    STATE["cap"] = cap


def reset() -> None:
    """From the task's @on_reset function (mechkit's reset puts the weld back)."""
    cap = STATE.get("cap", "cap")
    STATE.clear()
    STATE.update(cap=cap, released=None, start={side: None for side in HANDS}, most_carry=0.0, most_draw=0.0, track=[], most_cap_push=0.0)


reset()


def lodged() -> bool:
    return not mk.is_cut("lodged")


def faces(side: str) -> tuple[float, float]:
    """N with which the hand of ``side`` presses the coin's two faces (lower, upper along the coin's axis)."""
    c, r = mk.pose("coin")
    axis = r[:, 2]
    out = [0.0, 0.0]
    for k in mk.touching("coin", HANDS[side]):
        off = float(np.dot(np.asarray(k.point, dtype=float) - c, axis))
        if abs(off) >= FACE:
            out[off > 0.0] += abs(float(k.force))  # the simulator reports the normal force with a sign (negative here)
    return out[0], out[1]


def carry(side: str) -> float:
    """N with which the pads of ``side`` can draw the coin along its faces: the pads' friction on the coin
    (``parts.COIN_FRICTION``, min-combined) times the squeeze on both faces; 0 unless the hand pinches it (both faces
    at least MIN_FACE). An uneven pinch presses the coin sideways into the lip, which the weld takes."""
    low, high = faces(side)
    return P.COIN_FRICTION * (low + high) if min(low, high) >= MIN_FACE else 0.0


def along(side: str) -> float:
    """m of the hand's grasp point along the mouthpiece (I's x, out of the mouth)."""
    p, r = F.i_pose()
    return float((r.T @ (np.asarray(mk.gripper(side)[0], dtype=float) - p))[0])


def watch(events: list) -> None:
    """Once per control step: lets the coin go when a pinch that carries the hold has drawn DRAW outward."""
    if STATE.get("cap") is not None:
        STATE["most_cap_push"] = max(STATE["most_cap_push"], mk.contact_force("coin", STATE["cap"]))
    if not lodged():
        return
    now = mk.now()
    for side in HANDS:
        n = carry(side)
        STATE["most_carry"] = max(STATE["most_carry"], n)
        if n < P.HOLD:
            STATE["start"][side] = None
            continue
        x = along(side)
        start = STATE["start"][side] = x if STATE["start"][side] is None else min(STATE["start"][side], x)
        drawn = x - start
        STATE["most_draw"] = max(STATE["most_draw"], drawn)
        STATE["track"].append((round(now, 2), side, round(n, 3), round(1000 * drawn, 2)))
        if drawn >= P.DRAW:
            mk.cut("lodged")
            STATE["released"] = (now, side, n, drawn)
            events.append((now, f"coin drawn out: the {side} hand's pinch carried {n:.2f} N >= {P.HOLD:.2f} N over {1000 * drawn:.1f} mm"))
            return
