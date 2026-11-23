#!/usr/bin/env python3
"""The scripted run of the SB50 task, and its lower levels for the trials.

    scripts/rta demo <id> mech/tasks/sb50_mate [full,half,unflipped,idle] [--check] [--quick]

full: the left hand takes the left connector by its leads behind the T-handle's crossbar and holds it, lifted 15 mm,
while the right hand pulls the red cover off the nose and puts it down aside, hanging on its strap. The right hand
then takes the right connector by its leads behind the crossbar, from behind along its axis, lifts it and turns it
over with the wrist's last joint (turning it back to the planned pitch where it sagged in the hand), and the left hand
brings the left connector to the meeting point. The left nose is aimed at the right one from the housings' observed
poses (as a person looks), stopped 4 mm short, aimed again and pushed in until the noses overlap 2.2 mm: the housings
are joined. Both hands press the housings home against the 44 N, the crossbars' wings bearing on the pads; the right
hand lets go, the left one turns the pair to its resting pitch, lowers it over the mat and lets go a few millimetres
over it, and both hands move clear and back to their ready pose (1.0).
half: the same until the pair is pushed home; both hands then hold it up, not laid down (0.5). unflipped: the right connector is not
turned over: pushed at the left one with 20 N a hand, tongue meets tongue 14.7 mm short (0.25, the cover being off).
idle: both arms stay at their ready pose (0).

Results in outputs/<id>/<case>/: result.json (every step), readings.json, contact sheets, the videos unless --check
or --no-video. --quick: the plan's open-loop part through the arms' IK, without the simulator.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as K  # noqa: E402
import parts as P  # noqa: E402

MATE_Z = 0.120  # m: the turned-over right connector's rear face (its middle) over the bench while the noses are brought in
PRE_SPAN = 0.150  # m rear face to rear face when the noses are first brought in line (54 mm apart)
CLOSE_SPAN = 0.1000  # m: the noses' tips 4.3 mm apart, where the left one is aimed once more
ENTRY_SPAN = 0.0935  # m: noses 2.2 mm into each other: the housings are joined
HOME_SPAN = 0.0800  # m: pushed 1 mm past the 81.0 mm stop
PUSH_N = {"left": 46.0, "right": 50.0}  # N each hand pushes with along the axis (the left wrist's drive at 86 % of its 7 N m)
PUSH_UNFLIPPED_N = {"left": 20.0, "right": 20.0}  # the unflipped trial: the tongues meet face to face; harder, the two
# flat faces slip past each other sideways (46 / 50 N twisted both connectors out of line)
UNFLIPPED_SPAN = 0.0900  # m: the unflipped trial's left hand is driven to here, 5.4 mm past where the tongues meet
LIFT_L = 0.015  # m the left hand lifts its connector before the cover is pulled (the cover then hangs free of the bench)
LIFT_Y = 0.030  # m the right connector is lifted towards the mat before it is turned over (the wrist's last joint has room)
TURN_SIGN = 1
LAY_Y = 0.020  # m: the pair's middle 20 mm left of the mat's middle (35 mm allowed)
LAY_X = 0.035  # m: and 35 mm beyond it (50 mm allowed): the turned connector's stiff leads pass by the block, the pair
# 14 cm from the cover
LAY_PITCH = 6.0  # deg the joined pair is laid down at, its right end up: it rests at 6.1 (on the left leads' end and the turned handle's neck)
LAY_Z = 0.025  # m: the pair's middle over the bench where the left hand lets it go: 4 mm over its rest (21 mm), the
# hand's hulls 7 mm over the mat (wide open there the fingers would drag on the mat)
LOOSE = 0.030 / K.MAX_JAW  # the fingers 30 mm apart: off the 22.4 mm leads without sweeping far


def ready(side: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, dtype=float), K.BASES[side])


def back(pos, rot, metres: float) -> np.ndarray:
    """``pos`` moved back along the hand's approach axis."""
    return np.asarray(pos, float) - metres * np.asarray(rot, float)[:, 2]


def start() -> dict:
    import scene

    return scene.start_poses(K.TOP)


FLIPPED = np.diag([-1.0, 1.0, -1.0])  # the second housing, turned over, as seen from the first one
SAME_WAY = np.diag([-1.0, -1.0, 1.0])
UP = np.array([0.0, 0.0, 0.040])  # every grasp is approached from 4 cm straight above it, the hand open wide


def held_conn(hand_pos, hand_rot, at, hand_in) -> tuple[np.ndarray, np.ndarray]:
    """The connector's pose from the hand's, for a grasp ``at`` (connector frame) with the hand ``hand_in`` it."""
    rot = np.asarray(hand_rot, float) @ hand_in.T
    return np.asarray(hand_pos, float) - rot @ np.asarray(at, float), rot


def right_held(hand: tuple[np.ndarray, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    """The right connector's pose from the right hand's (right lead grasp)."""
    return held_conn(hand[0], hand[1], (K.RIGHT_AT[0], 0.0, K.RIGHT_AT[1]), K.hand_in_connector(K.RIGHT_TILT, True))


def waiting_hand(lifted: tuple[np.ndarray, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    """The right hand after it has turned the connector over: the wrist's last joint, half a turn about the hand's
    approach, which is 12 deg under the connector's axis, so that the connector then points 24 deg nose-down."""
    hp, hr = K.right_grasp(*lifted)
    return hp, K.spin(hr, TURN_SIGN * 180.0)


def lifted_right() -> tuple[np.ndarray, np.ndarray]:
    """The right connector lifted level over the bench, not yet turned: so high that, turned over about the hand's
    approach (it swings round the grasp point, which is over its axis), its rear face's middle is MATE_Z over the
    bench."""
    pos, rot = np.array([P.CONN_X, P.RIGHT_NOSE_Y - P.SB_L + LIFT_Y, K.TOP + MATE_Z]), K.level("right")
    turned = right_held(waiting_hand((pos, rot)))[0]
    return pos - np.array([0.0, 0.0, turned[2] - pos[2]]), rot


def right_waiting(turned: bool = True) -> tuple[np.ndarray, np.ndarray]:
    """Where the right connector waits for the left one: turned over, or, not turned, lifted level."""
    return right_held(waiting_hand(lifted_right())) if turned else lifted_right()


def meeting(span: float, turned: bool = True) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """The left connector brought in line with the waiting right one, ``span`` rear face to rear face."""
    pr, rr = right_waiting(turned)
    rl = rr @ (FLIPPED if turned else SAME_WAY)
    return {"left": (pr - rl[:, 0] * span, rl), "right": (pr, rr)}


def levelled(hands: dict, u: np.ndarray, mid: np.ndarray, pitch: float = LAY_PITCH) -> dict:
    """The hands' poses turned about the pair's middle (about x) so that the pair's axis ``u`` (the left connector's,
    towards the right one) rises ``pitch`` deg towards the right connector: LAY_PITCH is about the pitch a joined
    pair rests at, on the left leads' end and the turned-over handle's neck."""
    a = math.atan2(float(u[2]), -float(u[1])) - math.radians(pitch)
    c, s = math.cos(a), math.sin(a)
    turn = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    return {k: (mid + turn @ (p - mid), turn @ r) for k, (p, r) in hands.items()}


def laid(hands: dict, mid: np.ndarray) -> dict:
    """The hands' poses moved so that the pair's middle is over the mat, LAY_Z over the bench."""
    down = mid - np.array([P.MAT_XY[0], P.MAT_XY[1] + LAY_Y, K.TOP + LAY_Z])
    return {k: (p - down, r) for k, (p, r) in hands.items()}


def cover_aside() -> np.ndarray:
    """Where the right hand puts the cover down: on the bench 110 mm on the arms' side of the connectors' line (its
    fingers clear of the mat), 70 mm left of the middle: out of the strip the left hand sweeps (x 215 to 295 mm), within
    160 mm of where its lanyard's loop goes all along (200 mm lanyard: never lifted) and 14 cm from the laid pair. At
    55 mm beyond the line the left hand carried it off on its way to the meeting point."""
    return np.array([P.CONN_X - 0.110, 0.070, K.TOP + P.CAP_H / 2 + 0.002])


def key_poses(turned: bool = True) -> dict:
    """The plan's poses, in order, for checks.py: name -> {side: (grasp point, rotation, finger width)}."""
    s = start()
    (pl, rl), (pr, rr), (pc, _) = s["sb_left"], s["sb_right"], s["dust_cover"]
    lead_l = K.lead_grasp(pl, rl)
    held_l = (lead_l[0] + np.array([0.0, 0.0, LIFT_L]), lead_l[1], P.LEAD_W)
    cover = K.cover_grasp(pc)
    cover = (cover[0] + np.array([0.0, 0.0, LIFT_L]), cover[1])
    axis = rl[:, 0]
    lead_r = K.right_grasp(pr, rr)
    out = {
        "left over the leads": {"left": (lead_l[0] + UP, lead_l[1], 0.075)},
        "left on the leads": {"left": (*lead_l, P.LEAD_W)},
        "right over the cover": {"left": held_l, "right": (cover[0] + np.array([0, 0, 0.06]), cover[1], 0.075)},
        "right on the cover": {"left": held_l, "right": (*cover, P.CAP_W)},
        "cover pulled off": {"left": held_l, "right": (cover[0] + 0.026 * axis, cover[1], P.CAP_W)},
        "cover put aside": {"left": held_l, "right": (cover_aside(), cover[1], P.CAP_W)},
        "right over the leads": {"right": (lead_r[0] + UP, lead_r[1], 0.075)},
        "right on the leads": {"right": (*lead_r, P.LEAD_W)},
    }
    lifted = K.right_grasp(*lifted_right())
    for deg in ((0, 60, 120, 180) if turned else (0,)):
        out[f"right turning {deg}"] = {"right": (lifted[0], K.spin(lifted[1], TURN_SIGN * deg), P.LEAD_W)}
    pre, entry, home = meeting(PRE_SPAN, turned), meeting(ENTRY_SPAN, turned), meeting(HOME_SPAN, turned)
    waiting = (*(waiting_hand(lifted_right()) if turned else lifted), P.LEAD_W)
    out["left at the meeting point"] = {"left": (*K.lead_grasp(*pre["left"]), P.LEAD_W)}
    out["noses in line"] = {"left": (*K.lead_grasp(*pre["left"]), P.LEAD_W), "right": waiting, "push": True}
    out["noses in"] = {"left": (*K.lead_grasp(*entry["left"]), P.LEAD_W), "right": waiting, "push": True}
    hands = {"left": K.lead_grasp(*home["left"]), "right": waiting[:2]}  # the left hand drives the whole travel here
    u = home["left"][1][:, 0]  # towards the left nose: the left hand pushes along it, the right one against it
    out["pushed home"] = {"left": (*hands["left"], P.LEAD_W), "right": (*hands["right"], P.LEAD_W), "push": True,
                          "force": {"left": PUSH_N["left"] * u, "right": -PUSH_N["right"] * u}}  # fmt: skip
    mid = 0.5 * (home["left"][0] + home["right"][0])
    if turned:  # the joined pair: the right hand lets go and moves clear, the left one lays the pair down alone
        out["right let go, clear"] = {"left": out["pushed home"]["left"], "right": (*clear_of(*hands["right"]), 0.075)}
        hands = {"left": hands["left"]}
    flat = levelled(hands, u, mid, LAY_PITCH if turned else 0.0)
    out["pair levelled"] = {k: (*v, P.LEAD_W) for k, v in flat.items()}
    out["pair lowered"] = {k: (*v, P.LEAD_W) for k, v in laid(flat, mid).items()}
    return out


def clear_of(pos, rot) -> tuple[np.ndarray, np.ndarray]:
    """A hand that has let go: 5 cm back along its approach and 2 cm up."""
    return back(pos, rot, 0.05) + np.array([0.0, 0.0, 0.02]), rot


# --- the plan --------------------------------------------------------------------------------------------------------
def static_plan(case: str) -> dict[str, list]:
    """Moves up to the right connector lifted (and turned over); the rest is added by the director from what it sees."""
    import demo_kit as kit

    Move = kit.Move
    if case == "idle":
        return {"left": [Move(3.0, label="ready")], "right": [Move(3.0, label="ready")]}
    s = start()
    (pl, rl), (pr, rr), (pc, _) = s["sb_left"], s["sb_right"], s["dust_cover"]
    lead_l, cover, axis = K.lead_grasp(pl, rl), K.cover_grasp(pc), rl[:, 0]
    wide = K.OPEN_WIDE
    up = np.array([0.0, 0.0, LIFT_L])
    cover = (cover[0] + up, cover[1])
    left = [
        Move(2.5, pos=lead_l[0] + UP, rot=lead_l[1], grip=wide, label="over the leads"),
        Move(1.2, pos=lead_l[0], label="on the leads"),
        Move(0.8, grip=K.LEAD_GRIP, label="squeeze"),
        Move(1.0, pos=lead_l[0] + up, label="lift a little"),
    ]
    right = [
        Move(4.0, pos=cover[0] + np.array([0.0, 0.0, 0.06]), rot=cover[1], grip=wide, label="over the cover"),
        Move(1.5, pos=cover[0], label="round the cover"),
        Move(0.8, grip=K.COVER_GRIP, label="squeeze"),
        Move(2.0, pos=cover[0] + 0.026 * axis, push=8.0 * axis, label="pull off"),  # the left hand pulls back as hard meanwhile
        Move(0.8, pos=cover[0] + 0.026 * axis + np.array([0.0, 0.0, 0.025]), push=(0.0, 0.0, 0.0), label="lift"),
        Move(1.2, pos=cover_aside() + np.array([0.0, 0.0, 0.025]), label="aside"),
        Move(0.8, pos=cover_aside(), label="down"),
        Move(0.6, grip=wide, label="let go"),
    ]
    # the arm gives about 1 mm a newton: the left hand pulls its connector back while the cover is pulled, then eases off
    t_pull = sum(m.seconds for m in right[:3])
    left += [Move(t_pull - sum(m.seconds for m in left), label="hold"), Move(0.5, push=-12.0 * axis, label="hold against the pull"),
             Move(1.5, label="hold against the pull"), Move(0.8, push=(0.0, 0.0, 0.0), label="ease")]  # fmt: skip
    left += [Move(sum(m.seconds for m in right) - sum(m.seconds for m in left), label="hold for the cover")]  # then the director
    lead_r, lifted = K.right_grasp(pr, rr), K.right_grasp(*lifted_right())
    right += [
        Move(0.8, pos=cover_aside() + np.array([0.0, -0.02, 0.06]), label="up"),
        Move(2.0, pos=lead_r[0] + UP, rot=lead_r[1], label="over the leads"),
        Move(1.2, pos=lead_r[0], label="on the leads"),
        Move(0.8, grip=K.TURN_GRIP, label="squeeze"),
        Move(1.5, pos=lifted[0], rot=lifted[1], label="lift"),
    ]
    if case != "unflipped":  # the wrist's last joint turns the connector over about (nearly) its own axis
        right += [Move(1.5, rot=K.spin(lifted[1], TURN_SIGN * deg), label=f"turn {deg}") for deg in (60, 120, 180)]
    return {"left": left, "right": right}


class Director:
    """Adds the moves after the right connector is lifted (and turned over), from the housings' poses as they are
    then: bring the left connector to the meeting point, aim it at the right one (twice), push it in until the noses
    overlap 2.5 mm, push both home, turn the pair to its resting pitch, lay it over the mat and let go. The aim and
    the pushes read the parts' poses; nothing is set."""

    def __init__(self, m, case: str, plan: dict) -> None:
        self.m, self.case, self.stage, self.next_t = m, case, -2, sum(x.seconds for x in plan["left"])
        self.turned = case != "unflipped"
        self.hand = {s: (np.asarray(next((x.pos for x in reversed(plan[s]) if x.pos is not None), ready(s)[0]), float),
                         np.asarray(next((x.rot for x in reversed(plan[s]) if x.rot is not None), ready(s)[1]), float)) for s in plan}  # fmt: skip
        self.log: list = []
        self.pushes = self.aims = self.retries = 0

    def obs(self, name: str):
        p, r = self.m.mk.pose(name)
        return np.asarray(p, float).copy(), np.asarray(r, float).copy()

    def target_left(self, span: float):
        """Where the left connector should be: in line with the right one as it is seen now, ``span`` apart."""
        pr, rr = self.obs("sb_right")
        rl = rr @ (FLIPPED if self.turned else SAME_WAY)
        return pr - rl[:, 0] * span, rl

    def move_held(self, side: str, part: str, goal, seconds: float, label: str, **kw):
        """A move that takes the held part ``part`` to ``goal`` (pos, rot), from how it sits in the hand now."""
        import demo_kit as kit

        (hp, hr), (op, orr) = self.hand[side], self.obs(part)
        gp, gr = goal
        new_r = gr @ orr.T @ hr
        new_p = gp + gr @ orr.T @ (hp - op)
        self.hand[side] = (new_p, new_r)
        return kit.Move(seconds, pos=new_p, rot=new_r, label=label, **kw)

    def add(self, arms, t: float, moves: dict[str, list]) -> None:
        """Both arms' new moves start now (an arm whose moves ended earlier holds until now: a move scheduled in the
        past would be a jump) and end together."""
        import demo_kit as kit

        for side in ("left", "right"):
            behind = t - arms[side].seconds
            if behind > 1e-6:
                arms[side].moves.append(kit.Move(behind, label="hold"))
        longest = max(sum(x.seconds for x in v) for v in moves.values())
        for side in ("left", "right"):
            mv = list(moves.get(side, []))
            pad = longest - sum(x.seconds for x in mv)
            if pad > 1e-6:
                mv.append(kit.Move(pad, label="hold"))
            arms[side].moves.extend(mv)
        self.next_t = t + longest

    def __call__(self, t, arms, pictures) -> None:
        import demo_kit as kit

        if self.case == "idle" or t + 1e-9 < self.next_t:
            return
        Move, m = kit.Move, self.m
        if self.stage == -2:  # the cover is off and put down; the right hand is still taking (and turning) its connector
            self.next_t = max(arm.seconds for arm in arms.values())
            self.stage = -1
            return
        if self.stage == -1:  # the right connector as it hangs in the hand now: turn it back to the planned pitch if it
            # sagged (the turned-over crossbar's wings catch on the pads only after a few degrees), and bring the left
            # connector to the meeting point
            moves = {"left": [self.move_held("left", "sb_left", meeting(PRE_SPAN, self.turned)["left"], 2.5, "to the meeting point")]}
            fix = self.repitch(arms, t)
            if fix is not None:
                moves["right"] = [fix]
            self.add(arms, t, moves)
            self.stage = 0
            return
        al = m.alignment()
        if self.stage == 0:  # both connectors as planned before the left one is aimed at the right one, or stop here
            forces = m.scene.finger_forces()
            off = {s: angle_between(self.obs(n)[1], want[1]) for s, n, want in (("left", "sb_left", meeting(PRE_SPAN, self.turned)["left"]),
                                                                                ("right", "sb_right", right_waiting(self.turned)))}  # fmt: skip
            if min(forces.values()) < 5.0 or max(off.values()) > 20.0:
                self.log.append({"t": round(t, 2), "abort": "a connector is not held as planned", "fingers_n": {k: round(v, 1) for k, v in forces.items()},
                                 "off_plan_deg": {k: round(v, 1) for k, v in off.items()}})  # fmt: skip
                self.next_t = math.inf
                return
        self.log.append({"t": round(t, 2), "stage": self.stage, "span_mm": round(1000 * al["span"], 2), "offset_mm": round(1000 * al["offset"], 2),
                         "angle_deg": round(al["angle"], 2), "turn_deg": round(al["turn"], 1), "joined": not m.mk.is_cut("w_mate")})  # fmt: skip
        if self.stage in (0, 1):  # aim the left connector at the waiting right one, twice
            self.add(arms, t, {"left": [self.move_held("left", "sb_left", self.target_left(PRE_SPAN), 1.0 if self.stage == 0 else 0.8, "aim")]})
        elif self.stage == 2:  # close in to 4.3 mm between the noses' tips
            self.add(arms, t, {"left": [self.move_held("left", "sb_left", self.target_left(CLOSE_SPAN), 1.6, "close in")]})
        elif self.stage == 3:  # aim again, the arms nearly at rest (a fast stroke lags a few mm sideways), until the axes
            # are within 0.8 mm (four aims at most: the noses' 2 mm lead-ins catch no more), then slowly in until the
            # noses overlap 2.2 mm (turned over) or meet (same way up)
            if self.aims < 1 or (al["offset"] > 0.0008 and self.aims < 4):
                self.aims += 1
                self.add(arms, t, {"left": [self.move_held("left", "sb_left", self.target_left(CLOSE_SPAN), 0.8, "aim")]})
                return
            self.add(arms, t, {"left": [self.move_held("left", "sb_left", self.target_left(ENTRY_SPAN), 3.0, "noses in")]})
            self.stage = 5
            return
        elif self.stage == 5:
            if self.turned and m.mk.is_cut("w_mate"):  # not joined: 1 mm further in, or back off and aim again
                if self.pushes < 2:
                    self.pushes += 1
                    self.add(arms, t, {"left": [self.move_held("left", "sb_left", self.target_left(ENTRY_SPAN - 0.001 * self.pushes), 0.6, "noses in")]})
                    return
                if self.retries < 2:
                    self.retries, self.pushes, self.aims = self.retries + 1, 0, 0
                    self.add(arms, t, {"left": [self.move_held("left", "sb_left", self.target_left(CLOSE_SPAN), 1.0, "back off")]})
                    self.stage = 3
                    return
                self.log.append({"t": round(t, 2), "abort": "the noses did not engage"})
                self.next_t = math.inf
                return
            # both hands push home (the same way up, the noses stop each other 14.7 mm short)
            if not self.turned:
                self.push(arms, t, UNFLIPPED_SPAN, 2.0, PUSH_UNFLIPPED_N)
            else:
                self.push(arms, t, HOME_SPAN, 2.0, PUSH_N)
        elif self.stage == 6 and self.case == "half":  # the half trial: both hands stop pushing and hold the pair up
            self.add(arms, t, {k: [Move(1.5, push=(0.0, 0.0, 0.0), label="stop pushing"), Move(2.0, label="hold")] for k in ("left", "right")})
            self.stage = 98  # then nothing more: not laid down, not let go
        elif self.stage == 6:  # stop pushing (slowly: the wrists' offsets ease off); a joined pair: the right hand lets
            # go and the left one squeezes 36 N a finger to carry it alone
            moves = {k: [Move(1.5, push=(0.0, 0.0, 0.0), label="stop pushing")] for k in ("left", "right")}
            if self.turned:
                self.hand["right"] = clear_of(*self.hand["right"])
                moves["left"] += [Move(0.6, grip=K.TURN_GRIP, label="squeeze"), Move(1.0, label="hold")]
                moves["right"] += [Move(0.6, label="hold"), Move(0.6, grip=K.OPEN_WIDE, label="let go"), Move(1.0, pos=self.hand["right"][0], label="clear")]
            self.add(arms, t, moves)
        elif self.stage == 7:  # the pair as it hangs now: turned to its resting pitch about its middle and laid over the
            # mat, planned from the parts' observed poses (as the aims are) and only where the arms reach
            moves = self.lay(arms, t)
            if moves is None:
                self.next_t = math.inf
                return
            if self.turned:  # the right hand, empty, goes back to its ready pose meanwhile, clear of the right lead
                self.hand["right"] = ready("right")
                moves["right"] = [Move(2.5, pos=self.hand["right"][0], rot=self.hand["right"][1], grip=1.0, label="ready")]
            self.add(arms, t, moves)
        elif self.stage == 8:  # let go a little (the fingers 30 mm apart), straight up 5 cm, open (the right hand first)
            moves = {}
            for k, wait in (("right", 0.0), ("left", 0.8)):
                if self.turned and k == "right":
                    continue
                hp, hr = self.hand[k]
                self.hand[k] = (hp + np.array([0.0, 0.0, 0.05]), hr)
                moves[k] = ([Move(wait, label="hold")] if wait else []) + [Move(0.6, grip=LOOSE, label="let go"), Move(1.0, pos=self.hand[k][0], label="up"),
                                                                           Move(0.4, grip=K.OPEN_WIDE, label="open")]  # fmt: skip
            self.add(arms, t, moves)
        elif self.stage == 9:
            self.add(arms, t, {s: [Move(2.5, pos=ready(s)[0], rot=ready(s)[1], grip=1.0, label="ready")] for s in ("left", "right")})
        else:
            self.next_t = math.inf
            return
        self.stage += 1

    def lay(self, arms, t: float):
        """Moves that turn the pair (or, the same way up, both connectors) about its middle to its resting pitch and lay
        its middle over the mat, LAY_Z over the bench: the goal of each held connector from its observed pose, the hand
        from how it holds it now; carried over its place at the height it is at, then lowered straight down. The first
        of these the arm reaches all along (IK 0.5 mm, 3 deg inside the limits): the resting pitch at (LAY_X, LAY_Y) from
        the mat's middle; the pitch as it is; the resting pitch at (LAY_X, 0)."""
        pl, rl = self.obs("sb_left")
        pr, _ = self.obs("sb_right")
        mid = 0.5 * (pl + pr)
        u = rl[:, 0]
        now = math.degrees(math.atan2(float(u[2]), -float(u[1])))
        carry = ("left",) if self.turned else ("left", "right")
        rest = LAY_PITCH if self.turned else 0.0
        for pitch, off in ((rest, LAY_Y), (now, LAY_Y), (rest, 0.0)):
            a = math.radians(now - pitch)
            c, s = math.cos(a), math.sin(a)
            turn = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
            where = np.array([P.MAT_XY[0] + LAY_X, P.MAT_XY[1] + off, K.TOP + LAY_Z])
            up = np.array([0.0, 0.0, max(0.0, float(mid[2] - where[2]))])  # carried at its height, then straight down: a
            # slanting path swept the turned connector's leads down onto the block, which tipped the pair up in the hand
            hands, ok = {}, True
            for k in carry:
                part = "sb_left" if k == "left" else "sb_right"
                pp, rp = self.obs(part)
                (hp, hr) = self.hand[k]
                goal_p, goal_r = where + turn @ (pp - mid), turn @ rp
                hands[k] = (goal_p + goal_r @ rp.T @ (hp - pp), goal_r @ rp.T @ hr)
                q = arms[k].q
                for shift in (up, np.zeros(3)):
                    q, mm, deg, margin = K.solve(k, hands[k][0] + shift, hands[k][1], q)
                    ok &= mm <= K.IK_MM and deg <= K.IK_DEG and margin >= 3.0
            self.log.append({"t": round(t, 2), "lay_pitch_deg": round(pitch, 1), "lay_y_mm": round(1000 * off, 1), "reached": bool(ok)})
            if ok:
                import demo_kit as kit

                self.hand.update(hands)
                return {k: [kit.Move(2.5, pos=hands[k][0] + up, rot=hands[k][1], label="over the mat"), kit.Move(0.4, label="hold"),
                            kit.Move(2.0, pos=hands[k][0], rot=hands[k][1], label="lay down")] for k in carry}  # fmt: skip
        self.log.append({"t": round(t, 2), "abort": "the arms cannot lay the pair over the mat"})
        return None

    def repitch(self, arms, t: float):
        """A move that turns the right hand about x (through its grasp point) so that the right connector's pitch is
        the planned one again, at most 25 deg and only where the arm reaches; None if it is within 3 deg."""
        import demo_kit as kit

        rr, want = self.obs("sb_right")[1], right_waiting(self.turned)[1]
        a = math.asin(float(np.clip(want[2, 0], -1.0, 1.0))) - math.asin(float(np.clip(rr[2, 0], -1.0, 1.0)))
        if abs(a) < math.radians(3.0):
            return None
        a = float(np.clip(a, -math.radians(25.0), math.radians(25.0)))
        c, s = math.cos(a), math.sin(a)
        hp, hr = self.hand["right"]
        new_r = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]) @ hr
        q, mm, deg, margin = K.solve("right", hp, new_r, arms["right"].q)
        self.log.append({"t": round(t, 2), "repitch_deg": round(math.degrees(a), 1), "ik_mm": round(mm, 2), "margin_deg": round(margin, 1)})
        if mm > K.IK_MM or deg > K.IK_DEG or margin < 3.0:
            return None
        self.hand["right"] = (hp, new_r)
        return kit.Move(1.0, rot=new_r, label="pitch back")

    def push(self, arms, t: float, span: float, seconds: float, newtons: dict) -> None:
        """Both hands press their connectors towards each other along the left one's axis, the left one driven on until
        the span is ``span``."""
        import demo_kit as kit

        al = self.m.alignment()
        u = self.obs("sb_left")[1][:, 0]  # towards the left nose
        travel = {"left": al["span"] - span, "right": 0.0}  # the left hand drives the travel, the right one holds and pushes back
        moves, goals = {}, {}
        for side, sign in (("left", 1.0), ("right", -1.0)):
            hp, hr = self.hand[side]
            goal, force = hp + sign * travel[side] * u, sign * newtons[side] * u
            q, mm, deg, _ = K.solve(side, goal, hr, arms[side].q)
            effort, margin = K.pressed_targets(side, q, force)
            if mm > K.IK_MM or deg > K.IK_DEG or effort > 0.95 or margin < 1.0:  # the arm cannot push so: stop rather than push some other way
                self.log.append({"t": round(t, 2), "abort": f"{side} arm cannot push there", "ik_mm": round(mm, 2), "effort": round(effort, 2), "pushed_margin_deg": round(margin, 1)})
                self.next_t = math.inf
                return
            goals[side] = (goal, hr)
            # each hand pushes along its connector's axis (the point on the axis by its grasp), so the two pushes are
            # in line: pushed at the grasps (8 and 3 mm off the axis) they made a couple that turned the seated pair
            # 70 deg in the hands once the slide stopped
            pp, rp = self.obs("sb_left" if side == "left" else "sb_right")
            on_axis = pp + rp[:, 0] * float((hp - pp) @ rp[:, 0])
            at = hr.T @ (on_axis - hp)
            moves[side] = [kit_move(0.6, hp, hr, None, "loosen", K.LEAD_GRIP),  # 26 N a finger before any push: the wings carry it
                           kit.Move(seconds, pos=goal, rot=hr, push=force, push_at=at, label="push home", grip=K.LEAD_GRIP),
                           kit_move(0.3, goal, hr, force, "hold")]
        self.hand.update(goals)
        self.add(arms, t, moves)


def angle_between(a: np.ndarray, b: np.ndarray) -> float:
    """deg of the rotation from ``b`` to ``a``."""
    return math.degrees(math.acos(float(np.clip((np.trace(np.asarray(b).T @ np.asarray(a)) - 1.0) / 2.0, -1.0, 1.0))))


def kit_move(seconds, pos, rot, push, label, grip=None):
    import demo_kit as kit

    return kit.Move(seconds, pos=pos, rot=rot, push=push, label=label, grip=grip)


def readings_of(rows: list, director: Director, report: dict, result: dict) -> dict:
    first = {lvl: next((r["t"] for r in rows if r["info"].get("score", 0.0) >= lvl), None) for lvl in (0.25, 0.5, 1.0)}
    wipe = [r["extra"]["wipe_n"] for r in rows]
    return {
        "score at the end": rows[-1]["info"].get("score") if rows else None,
        "seconds": rows[-1]["t"] if rows else 0.0,
        "first 0.25 / 0.5 / 1.0 at (s)": first,
        "most wipe force (N)": round(max(wipe), 1) if wipe else None,
        "least span (mm)": round(min(r["extra"]["span_mm"] for r in rows), 2) if rows else None,
        "most finger force (N)": {s: round(max(r["extra"]["fingers_n"][s] for r in rows), 1) for s in ("left", "right")} if rows else None,
        "most hand force (N)": {s: round(max(float(np.linalg.norm(r["force"][s])) for r in rows), 1) for s in ("left", "right")} if rows else None,
        "noses touching, most force (N)": round(max(r["extra"]["noses_n"] for r in rows), 1) if rows else None,
        "director": director.log,
        "report": report,
        "worst IK": result.get("worst_ik"),
        "median step (ms)": result.get("median_step_ms"),
    }


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    args = sys.argv[2:]
    cases = [c for a in args if not a.startswith("--") for c in a.split(",")] or ["full"]
    if "--quick" in args:
        from yam_arena import station

        print("QUICK " + json.dumps({c: kit.dry_run(static_plan(c), station.BASES) for c in cases}), flush=True)
        return 0
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    record = not ("--no-video" in args or "--check" in args)
    status = 0
    for case in cases:
        plan = static_plan(case)
        director = Director(m, case, plan)
        latest: dict = {}

        def watch(t, arms, pictures, director=director, latest=latest) -> None:
            if pictures is not None:
                latest.update(pictures)
            director(t, arms, pictures)

        def extra() -> dict:
            al = m.alignment()
            tilt = {n: [round(float(np.degrees(np.arcsin(np.clip(m.mk.pose(n)[1][2, 0], -1, 1)))), 1), round(float(np.degrees(np.arctan2(m.mk.pose(n)[1][2, 1], m.mk.pose(n)[1][2, 2]))), 1)]
                    for n in ("sb_left", "sb_right")}  # fmt: skip
            return {"span_mm": round(1000 * al["span"], 2), "offset_mm": round(1000 * al["offset"], 2), "turn_deg": round(al["turn"], 1),
                    "wipe_n": round(m.STATE.get("wipe_n", 0.0), 1), "noses_n": round(m.mk.contact_force("sb_left", "sb_right"), 1),
                    "fingers_n": {k: round(v, 1) for k, v in m.scene.finger_forces().items()}, "hands_clear_mm": round(1000 * m.hands_clear(), 1),
                    "cap_slide_mm": round(1000 * m.mk.joint("cap_slide"), 1), "hold": m.STATE.get("hold", 0), "pitch_roll": tilt,
                    "mid_mm": [round(1000 * float(v), 1) for v in (0.5 * (m.mk.pose("sb_left")[0] + m.mk.pose("sb_right")[0]) - np.array([*P.MAT_XY, K.TOP]))],
                    "cover_mm": [round(1000 * float(v), 1) for v in (m.mk.pose("dust_cover")[0] - np.array([*P.MAT_XY, K.TOP]))],
                    "cover_touch": {c.other: round(abs(float(c.force)), 1) for c in m.mk.contacts("dust_cover") if abs(float(c.force)) > 0.05},
                    "pair_touch": {f"{n[3]}:{c.other}": round(abs(float(c.force)), 1) for n in ("sb_left", "sb_right") for c in m.mk.contacts(n) if abs(float(c.force)) > 0.2}}  # fmt: skip

        folder = out / case
        seen: list = []

        def logged(extra=extra, seen=seen) -> dict:
            row = extra()
            seen.append(row)
            return row

        try:
            result = kit.run(emb, plan, folder, seed=0, director=watch, extra=logged, limit_s=float(m.RUN_LIMIT_STEPS) / 20.0, tail_s=4.0,
                             sheet_frames=16, record=record)  # fmt: skip
        except Exception as error:  # keep what the run saw up to the error
            (folder / "crash.json").write_text(json.dumps({"error": repr(error), "director": director.log, "report": m.report(), "rows": seen[::5]}, default=str))
            print(f"DEMO {case} CRASH {error!r}", flush=True)
            status = 1
            continue
        if "top_cam" in latest:
            import cv2

            cv2.imwrite(str(folder / "end_top.png"), cv2.cvtColor(np.asarray(latest["top_cam"]), cv2.COLOR_RGB2BGR))
            cv2.imwrite(str(folder / "end_top_policy.png"), cv2.cvtColor(kit.policy_picture(np.asarray(latest["top_cam"])), cv2.COLOR_RGB2BGR))
        readings = readings_of(result["rows"], director, m.report(), result)
        (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
        print(f"DEMO {case} " + json.dumps({k: v for k, v in readings.items() if k != "director"}, default=str), flush=True)
        print(f"DIRECTOR {case} " + json.dumps(director.log, default=str), flush=True)
    return status


if __name__ == "__main__":
    raise SystemExit(main())
