"""Sprayed lanes. On the bench, 27 cm in front of the arm bases, two white rubber drying mats (16 x 24 x 0.4 cm,
fixed), one in front of each arm with their centres 15 cm left and right of the centre line; each mat is divided
lengthwise (along y) into two lanes 12 cm wide, an inner and an outer one; the lanes carry no marks. Four steel
instruments (10 x 2 x 2.5 cm bars, 50 g) lie 12 cm out, 6 cm and 16 cm left and right of the centre line, long
axis pointing away from the arms.

What happens after reset, while the arms are held: a spray bottle (grey, 5 cm across, 15 cm tall) sweeps along
the centre line of one lane of each mat, 8 cm above the mat, 2 s per lane, from 1 s to 5 s (the order of the mats
is random), then is taken away. Those two lanes are wet with disinfectant and must not be touched; they look
like the dry ones. From ACT_AFTER_S the arms are free. Lay all four instruments flat on the mats in the dry
lanes (each arm serves the mat on its side). An instrument low over a sprayed lane, or a fingertip low over one,
is a contamination failure that sets the score to zero. Guessing both dry lanes succeeds one time in four.
"""

from __future__ import annotations

from itertools import product

import numpy as np

from rta import anim, props
from rta.plans import grasp_and_carry, wait
from rta.spec import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "One lane of each drying mat was just sprayed with disinfectant and must not be touched. Lay all four instruments on the mats."

SIDES = ("left", "right")
SIGN = {"left": +1.0, "right": -1.0}
MAT_X, MAT_CY, MAT_SIZE = 0.27, 0.15, (0.16, 0.24, 0.004)
MAT_XY = {s: (MAT_X, SIGN[s] * MAT_CY) for s in SIDES}
MAT_TOP = TOP + MAT_SIZE[2]
LANE_W = 0.12
LANES = (0, 1)  # per mat: 0 = inner lane (next to the centre line), 1 = outer lane


def lane_y(side: str, k: int) -> float:
    """Centre line of lane ``k`` of the mat on ``side``."""
    return SIGN[side] * (MAT_CY - LANE_W / 2 + LANE_W * k)


BAR_SIZE, BAR_MASS = (0.10, 0.02, 0.025), 0.05
BAR_X, BAR_Y = 0.12, (+0.16, +0.06, -0.06, -0.16)
BARS = tuple(f"bar_{k}" for k in range(4))
BOTTLE_R, BOTTLE_H = 0.025, 0.15
SPRAY_T0, SPRAY_PER_LANE, SPRAY_HEIGHT = 1.0, 2.0, 0.08
ACT_AFTER_S = 6.5
COMBOS = [dict(zip(SIDES, c)) for c in product(LANES, repeat=2)]  # 4 sprayed-lane combinations (one lane per mat)
MARGIN = 0.005  # an instrument's footprint must stay this far from a sprayed lane
BAR_LOW, TCP_LOW = 0.01, 0.02  # contamination heights over a sprayed lane
WHITE, STEEL, GREY = (0.95, 0.95, 0.95), (0.75, 0.76, 0.78), (0.45, 0.45, 0.47)

OBJECTS = [
    *(Box(f"mat_{s}", pos=(*MAT_XY[s], TOP + MAT_SIZE[2] / 2), size=MAT_SIZE, color=WHITE) for s in SIDES),
    *(Box(n, pos=(BAR_X, BAR_Y[k], TOP + BAR_SIZE[2] / 2), size=BAR_SIZE, mass=BAR_MASS, color=STEEL) for k, n in enumerate(BARS)),
    Cylinder("bottle", pos=anim.park(0, BOTTLE_H), radius=BOTTLE_R, height=BOTTLE_H, mass=0.1, color=GREY),
]


def setup(rng, trial) -> dict:
    """Hidden state: the sprayed lane of each mat (one of 4 combinations, cycling with the seed number); the mats
    are sprayed in random order."""
    sprayed = dict(COMBOS[trial.balanced(len(COMBOS), rng)])
    order = [SIDES[i] for i in rng.permutation(2)]
    return {"sprayed": sprayed, "order": order}


def bottle_keys(state):
    z = MAT_TOP + SPRAY_HEIGHT + BOTTLE_H / 2
    x0, x1 = MAT_X - MAT_SIZE[0] / 2 + 0.02, MAT_X + MAT_SIZE[0] / 2 - 0.02
    keys = []
    t = SPRAY_T0
    for k, side in enumerate(state["order"]):
        y = lane_y(side, state["sprayed"][side])
        if k == 0:
            keys.append((t - 0.5, (x0, y, z + 0.12)))
        keys += [(t, (x0, y, z)), (t + SPRAY_PER_LANE, (x1, y, z))]
        t += SPRAY_PER_LANE
    last = state["order"][-1]
    keys += [(t + 0.3, (x1, lane_y(last, state["sprayed"][last]), z + 0.12)), (t + 0.8, anim.park(0, BOTTLE_H))]
    return keys


def events(t: float, state: dict, world) -> None:
    anim.drive(world, "bottle", t, bottle_keys(state))


MILESTONES = tuple(f"{n}_on_dry_mat" for n in BARS)
WEIGHTS = (0.25,) * 4
SUCCESS_HOLD_S = 1.0
ZERO_ON = {"instrument_over_sprayed_lane", "fingertip_over_sprayed_lane"}


def _over_mat_x(x) -> bool:
    return abs(x - MAT_X) < MAT_SIZE[0] / 2


def _lane_span(side, k):
    lo, hi = sorted((lane_y(side, k) - LANE_W / 2, lane_y(side, k) + LANE_W / 2))
    return lo, hi


def _sprayed_spans(state):
    return [_lane_span(side, k) for side, k in state["sprayed"].items()]


def _clear_of_sprayed(o, state) -> bool:
    for lo, hi in _sprayed_spans(state):
        if o.high[1] > lo - MARGIN and o.low[1] < hi + MARGIN:
            return False
    return True


def _flat_on_mat(o) -> bool:
    on_a_mat = any(o.low[1] > cy - MAT_SIZE[1] / 2 and o.high[1] < cy + MAT_SIZE[1] / 2 for _, cy in MAT_XY.values())
    return bool(
        _over_mat_x(o.pos[0]) and on_a_mat
        and abs(o.low[2] - MAT_TOP) < 0.01 and props.tilt_deg(o) < 10.0 and o.resting()
    )


def milestones(objects, state) -> list[bool]:
    return [_flat_on_mat(objects[n]) and _clear_of_sprayed(objects[n], state) for n in BARS]


def success(objects, state) -> bool:
    return all(milestones(objects, state))


def failure(objects, state) -> str | None:
    for n in BARS:
        o = objects[n]
        if o.pos[2] < TOP - 0.2:
            return "instrument_off_table"
        if _over_mat_x(o.pos[0]) and o.low[2] < MAT_TOP + BAR_LOW and not _clear_of_sprayed(o, state):
            return "instrument_over_sprayed_lane"
    for tcp in objects.tcp.values():
        if _over_mat_x(tcp[0]) and tcp[2] < MAT_TOP + TCP_LOW:
            for lo, hi in _sprayed_spans(state):
                if lo <= tcp[1] <= hi:
                    return "fingertip_over_sprayed_lane"
    return None


TRAVEL = 0.10  # low carry: the bars are 2.5 cm tall and nothing stands between the start spots and the mats
GRASP_UP = 0.02  # TCP height above the bar's bottom (bar 2.5 cm tall)
APPROACH = 0.02  # per-finger opening (m) on the way down: the neighbouring bar in the lane is 5 cm away (edge at 4 cm)
SPOT_DY = 0.03  # two bars per dry lane, at lane centre +- this: bar edges 4 cm from the lane centre, 2 cm from its boundary (the TCP lands 5-8 mm off; 2.5 cm in a 9 cm lane left 5 mm and failed by 0.4 mm, s04b)
ARM_BARS = {"left": (BARS[0], BARS[1]), "right": (BARS[3], BARS[2])}  # outer bar first


def dry_lane(state, side: str) -> int:
    return 1 - state["sprayed"][side]


def dry_spots(state, side: str) -> list[tuple[float, float]]:
    """The two landing spots (x, y) in the dry lane of the mat on ``side``, outer spot first."""
    y = lane_y(side, dry_lane(state, side))
    return [(MAT_X, y + SIGN[side] * SPOT_DY), (MAT_X, y - SIGN[side] * SPOT_DY)]


def demo(state) -> dict:
    """One arm at a time: the left arm lays its two bars (outer bar first, outer spot first) in the dry lane of its
    mat, then the right arm does the same on its side. Mirrored simultaneous picks of the inner bars (6 cm either
    side of the centre line) pressed the grippers together and stalled both arms with the TCPs 12.6 cm out (s04b)."""
    segs = {}
    for arm in SIDES:
        segs[arm] = []
        for n, (x, y) in zip(ARM_BARS[arm], dry_spots(state, arm)):
            k = BARS.index(n)
            segs[arm] += grasp_and_carry(
                (BAR_X, BAR_Y[k], TOP + GRASP_UP), (x, y, MAT_TOP + GRASP_UP + 0.006), travel_z=TOP + TRAVEL,
                approach_finger=APPROACH, release_finger=APPROACH,
            )
    left_s, right_s = (sum(w.duration for w in segs[arm]) for arm in SIDES)
    return {"left": (0.0, segs["left"] + [wait("left", right_s)]), "right": (0.0, [wait("right", left_s)] + segs["right"])}
