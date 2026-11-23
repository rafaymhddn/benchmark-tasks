"""Pull the red dust cover off an Anderson SB50 battery connector, mate it with a second SB50 and lay the pair down.

Two identical grey SB50 housings (each with its red T-handle and its two 6 AWG leads clamped to the bench) lie '+'
face up with their noses towards each other; the left nose carries its red dust cover, tied on by a strap. The nose of
an SB50 fits a second one only when that one is turned over: the same way up, tongue meets tongue 14.7 mm short and
nothing moves on. How it is built (scene.py, every figure from parts.py), with no pose set anywhere:
- the cover slides off along the nose against 10 N of friction; at 16 mm it is off and the task frees it (its weld to
  the slide is cut where it is), and it hangs on its strap;
- the housings meet by contact: their noses (a lower tongue between two upper side walls, rounded front corners) let
  one slide into the other only turned over and in line; once the noses overlap 1.5 mm in line, the task joins the
  right housing to a slide on the left one's axis where it stands (weld w_mate, made at the housings' own pose), and
  the slide's friction-like spring takes 44 N to move either way (DS-SB50's average mating and unmating force).
The score reads the cover's place, the housings' poses, both hands and the bodies' motion.
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
SUCCESS_HOLD_S = 2.0
INSTRUCTION = ("Pull the red dust cover off, mate the two grey battery connectors fully together, lay the joined pair "
               "on the yellow mat, then let go and move both hands clear.")  # fmt: skip
RUN_LIMIT_STEPS = 3000  # 150 s at 20 Hz: about 2.5 times the reference run
TOP = TABLE_TOP[SCENE]
SIDES = ("left", "right")
PARTS = ("sb_left", "sb_right", "dust_cover")

scene.build(TOP)
OBJECTS = list(mk.MECH.bodies.values())
STATE: dict = {}

_HOUSING = np.array([[x, y, z] for x in (0.0, P.SB_L) for y in (-P.SB_W / 2, P.SB_W / 2) for z in (-P.SB_H / 2, P.SB_H / 2)])
_NOSE = (np.array([scene.REAR, -P.SB_W / 2, -P.SB_H / 2]), np.array([P.SB_L, P.SB_W / 2, P.SB_H / 2]))
_head = P.CAP_ON + P.CAP_FRONT
_COVER = np.array([[x, y, z] for x in np.linspace(-_head / 2, _head / 2, 5) for y in np.linspace(-P.CAP_W / 2, P.CAP_W / 2, 7)
                   for z in np.linspace(-P.CAP_H / 2, P.CAP_H / 2, 5)])  # fmt: skip


def begin(seed: int | None = None) -> dict:
    STATE.clear()
    STATE.update(seed=seed, cover_off_s=None, engaged_s=None, ov0=None, t0=0.0, rest=0.0, wipe_n=0.0, peak_wipe_n=0.0,
                 hold=0, hands=None, prev=None)  # fmt: skip
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)
    if mk.MECH.environment is not None:
        mk.set_limits("mate_wipe", -P.NOSE_L, P.NOSE_L)
        mk.set_spring("mate_wipe", rest=0.0, most=P.MATE_N)


# --- what the rules read ------------------------------------------------------------------------------------------------
def alignment() -> dict[str, float]:
    """The right housing against the left one: rear face to rear face (span), along the left one's axis (axial), off it
    (offset), the angle between the two noses' directions (0: pointing at each other) and the turn of one about the
    axis against the other (0: same way up, 180: one turned over)."""
    pl, rl = mk.pose("sb_left")
    pr, rr = mk.pose("sb_right")
    u = rl[:, 0]
    d = pr - pl
    axial = float(d @ u)
    zl = rl[:, 2] - float(rl[:, 2] @ u) * u
    zr = rr[:, 2] - float(rr[:, 2] @ u) * u
    cos_turn = float(zl @ zr) / max(1e-9, float(np.linalg.norm(zl) * np.linalg.norm(zr)))
    return {"span": float(np.linalg.norm(d)), "axial": axial, "offset": float(np.linalg.norm(d - axial * u)),
            "angle": math.degrees(math.acos(max(-1.0, min(1.0, -float(u @ rr[:, 0]))))),
            "turn": math.degrees(math.acos(max(-1.0, min(1.0, cos_turn))))}  # fmt: skip


def overlap(al: dict) -> float:
    """How far the two noses reach into each other along the axis (m): 14.7 mm when mated, < 0 when apart."""
    return 2.0 * P.SB_L - al["axial"]


def cover_clear() -> float:
    """Least distance (m) from the cover to the left housing's nose."""
    pl, rl = mk.pose("sb_left")
    pc, rc = mk.pose("dust_cover")
    local = ((_COVER @ rc.T + pc) - pl) @ rl
    gap = np.maximum(np.maximum(_NOSE[0] - local, local - _NOSE[1]), 0.0)
    return float(np.linalg.norm(gap, axis=1).min())


def cover_off() -> bool:
    return mk.is_cut("w_cap") and cover_clear() >= P.CAP_CLEAR


def turned_and_in_line(al: dict, turn_tol: float, offset: float, angle: float) -> bool:
    return abs(180.0 - al["turn"]) <= turn_tol and al["offset"] <= offset and al["angle"] <= angle


def underside(name: str) -> float:
    """Height (m) of the housing's lowest corner over the mat's top."""
    p, r = mk.pose(name)
    return float((_HOUSING @ r.T + p)[:, 2].min()) - (TOP + P.MAT_T)


def pitch_deg(name: str) -> float:
    return abs(math.degrees(math.asin(max(-1.0, min(1.0, float(mk.pose(name)[1][2, 0]))))))


def on_mat() -> bool:
    """The pair's midpoint 2 cm or more inside the mat's edges, each housing's lowest corner at most 18 mm over the mat
    (and not under it), each housing pitched 15 degrees at most."""
    mid = 0.5 * (mk.pose("sb_left")[0] + mk.pose("sb_right")[0])
    if abs(float(mid[0]) - P.MAT_XY[0]) > P.MAT_X / 2 - P.MAT_INSET or abs(float(mid[1]) - P.MAT_XY[1]) > P.MAT_Y / 2 - P.MAT_INSET:
        return False
    return all(-0.003 <= underside(n) <= P.UNDERSIDE_MAX and pitch_deg(n) <= P.PITCH_MAX_DEG for n in ("sb_left", "sb_right"))


def hands_clear() -> float:
    """Least distance (m) from either hand to the boxes bounding the two connectors and the cover."""
    hands = STATE.get("hands")
    if not hands:
        return 0.0
    pts = np.vstack([hands[s] for s in SIDES if s in hands])
    return min(scene.box_distance(pts, n) for n in PARTS)


def at_rest() -> bool:
    """Each connector and the cover moved under 10 mm/s and turned under 0.5 rad/s over the last control step."""
    prev = STATE.get("prev")
    if not prev:
        return False
    for n in PARTS:
        (p, r), (p0, r0) = mk.pose(n), prev[n]
        turn = math.acos(max(-1.0, min(1.0, (float(np.trace(r0.T @ r)) - 1.0) / 2.0)))
        if float(np.linalg.norm(p - p0)) * 20.0 >= P.REST_LIN or turn * 20.0 >= P.REST_ANG:
            return False
    return True


# --- the mechanisms, once per control step ------------------------------------------------------------------------------
def _cover_step() -> None:
    """At 16 mm of slide the cover is off the nose: free it where it is."""
    if not mk.is_cut("w_cap") and mk.joint("cap_slide") >= P.CAP_ON:
        mk.cut("w_cap")
        STATE["cover_off_s"] = round(mk.now(), 2) if mk.MECH.environment is not None else 0.0


def _mate_step() -> None:
    al = alignment()
    ov = overlap(al)
    sim = mk.MECH.environment is not None
    if mk.is_cut("w_mate"):
        STATE["wipe_n"] = 0.0
        if (mk.is_cut("w_cap") and P.ENGAGE_OVERLAP <= ov <= P.NOSE_L
                and turned_and_in_line(al, P.ENGAGE_ROLL_DEG, P.ENGAGE_OFFSET, P.ENGAGE_ANGLE_DEG)):  # fmt: skip
            t0 = mk.joint("mate_wipe")
            mk.make("w_mate")  # where the housings stand: no pose is set
            STATE.update(ov0=ov, t0=t0, rest=t0, engaged_s=round(mk.now(), 2) if sim else 0.0)
            if sim:  # the slide's travel falls as the housings close (the rail moves back along the left one's axis):
                # it may run to just past the full mate (the noses stop it there) and back to apart
                mk.set_limits("mate_wipe", t0 - (P.NOSE_L - ov) - 0.0005, t0 + ov + 0.001)
                mk.set_spring("mate_wipe", rest=t0, most=P.MATE_N)
        return
    travel = mk.joint("mate_wipe")
    if STATE["ov0"] - (travel - STATE["t0"]) < 0.0005:  # pulled apart: the housings are separate again
        mk.cut("w_mate")
        STATE.update(ov0=None, wipe_n=0.0)
        if sim:
            mk.set_limits("mate_wipe", -P.NOSE_L, P.NOSE_L)
            mk.set_spring("mate_wipe", rest=STATE["t0"], most=P.MATE_N)
        return
    band = P.MATE_N / P.WIPE_K  # the spring's rest follows the slide within this band: it holds, or slides at 44 N
    rest = float(np.clip(STATE["rest"], travel - band, travel + band))
    if STATE["ov0"] - (travel - STATE["t0"]) >= P.NOSE_L - 0.0005:  # seated: no force is stored to spring the pair apart
        rest = travel
    STATE["rest"] = rest
    if sim:
        mk.set_spring("mate_wipe", rest=rest)
    STATE["wipe_n"] = min(P.MATE_N, abs(P.WIPE_K * (travel - rest)))
    STATE["peak_wipe_n"] = max(STATE["peak_wipe_n"], STATE["wipe_n"])


# --- the score ----------------------------------------------------------------------------------------------------------
def half_ok() -> bool:
    """Cover off, one housing turned over and the two noses in line and overlapping 5 mm or more."""
    if not cover_off():
        return False
    al = alignment()
    return turned_and_in_line(al, P.TURN_TOL_DEG, P.AXIS_OFFSET, P.AXIS_ANGLE_DEG) and al["span"] <= P.HALF_SPAN


def full_ok() -> bool:
    """Half, plus mated home (81.3 mm -2.0/+2.2 rear face to rear face), lying on the mat, hands clear, at rest."""
    if not half_ok():
        return False
    span = alignment()["span"]
    return P.FULL_SPAN[0] <= span <= P.FULL_SPAN[1] and on_mat() and hands_clear() >= P.HANDS_CLEAR and at_rest()


@mk.on_step
def step() -> None:
    if mk.MECH.environment is not None:
        STATE["hands"] = scene.read_hands()
    if not mk.MECH.poses:
        return
    _cover_step()
    _mate_step()
    STATE["hold"] = STATE.get("hold", 0) + 1 if full_ok() else 0
    STATE["prev"] = {n: (mk.pose(n)[0].copy(), mk.pose(n)[1].copy()) for n in PARTS}


def score(_objects) -> float:
    """Read at the end of the trial. 1.0: the two housings mated home, one turned over (180 +- 20 degrees), in line
    (axes within 4 mm and 10 degrees), rear face to rear face 79.3 to 83.5 mm (81.3 mm on the maker's drawing); the pair
    lying on the yellow mat, its midpoint 2 cm or more inside the mat's edges, each housing's lowest corner at most
    18 mm over the mat and pitched 15 degrees at most; the cover off; every part of both hands 30 mm or more from the
    connectors and the cover; all of them still (under 10 mm/s and 0.5 rad/s) for the last 1.0 s. 0.5: the cover off
    and the two noses in line, one housing turned over, overlapping 5 mm or more (91.0 mm rear face to rear face or
    less). 0.25: the cover off the left nose (no part of it within 5 mm of the nose). 0 otherwise."""
    if not mk.MECH.poses:
        return 0.0
    if STATE.get("hold", 0) >= P.HOLD_STEPS and full_ok():
        return 1.0
    if half_ok():
        return 0.5
    return 0.25 if cover_off() else 0.0


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out = {"cover_off_s": STATE.get("cover_off_s"), "engaged_s": STATE.get("engaged_s"), "joined": not mk.is_cut("w_mate"),
           "peak_wipe_n": round(STATE.get("peak_wipe_n", 0.0), 1), "hold_steps": STATE.get("hold", 0)}  # fmt: skip
    if mk.MECH.poses:
        al = alignment()
        out.update({
            "cap_slide_mm": round(1000 * mk.joint("cap_slide"), 1), "cover_clear_mm": round(1000 * cover_clear(), 1),
            "span_mm": round(1000 * al["span"], 2), "overlap_mm": round(1000 * overlap(al), 2), "offset_mm": round(1000 * al["offset"], 2),
            "angle_deg": round(al["angle"], 2), "turn_deg": round(al["turn"], 1),
            "underside_mm": {n: round(1000 * underside(n), 1) for n in ("sb_left", "sb_right")},
            "pitch_deg": {n: round(pitch_deg(n), 1) for n in ("sb_left", "sb_right")}, "on_mat": on_mat(),
            "hands_clear_mm": round(1000 * hands_clear(), 1), "score": score(None),
        })  # fmt: skip
    return out
