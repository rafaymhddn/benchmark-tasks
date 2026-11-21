#!/usr/bin/env python3
"""Reference run of the grab bar task: the full score.

    scripts/rta demo <id> mech/tasks/grab_bar [--quick | --check]

Each arm works the lever at its own end of the bar, and a hand holds the bar whenever no cup holds it. The order is
forced: the left cup holds the bar at the start, so the right lever (its cup holds nothing) is released first; the
right hand must then hold the bar before the left lever is released (released with nobody holding, the bar drops),
near the left cup, so that its press seals that cup when the left lever is locked; the right hand lets go only once
the left cup holds again, and the left hand then presses the bar on near the right cup while the right hand locks.

(0) The right hand looks at the right cup from 190 mm: its wrist camera sees the red tab on top of the housing and the
    dark joint running into the cup (the pictures are kept: ``cue_red_tab_right_wrist*.png``).
(a) The right hand comes along the bar from outside its right end (pointing towards the bar's middle, 15 deg towards
    the wall and 15 deg down), pinches the right lever's top and bottom edges LEVER_ARM out and pulls it out: a pull
    rising 1 N per control step, the hand turning with the lever one step ahead of it, until the lever is read past
    centre; it lets the lever go at LET_GO_DEG and leaves (a released lever rests at LEVER_CAM_DEG).
(b) The right hand takes the grip near the left cup (fingers above and below), presses the bar on the wall with 15 N
    and carries its weight (3 N up); the left hand comes outside the left end.
(c) The left hand releases the left lever as in (a): the seal is cut, the right hand holds the bar.
(d) The right hand takes the bar 2 mm off the wall, carrying its weight at its middle of mass, moves it 60 mm towards
    -y along the wall, sets it back on and presses again: the joint now lies midway between the cups.
(e) The right hand presses HOLD_PRESS on; the left hand pushes the left lever home with its closed fingertips on the
    lever's outer face, the push rising 1 N per step, until the lever is past centre and near home; it leaves.
(f) The right hand lets go of the grip and leaves to outside the right end; the left hand takes the grip near the
    right cup and presses HOLD_PRESS on; the right hand pushes the right lever home as in (e). Both hands go back to
    their ready poses, quickly (the run ends SUCCESS_HOLD_S after the full score, which comes as the hands leave).

What the script reads from the simulator: where the bar is, each lever's angle, and the arms' joints (for the force
estimates in the log). It reads neither the seals nor the score. Each step is planned from the bar as read when the
step before it is through; the releases and the locks read the lever's angle every control step.

``--quick``: the plan without the simulator (the arms' IK and joint margins). ``--check``: the full run in the
simulator without videos, into ``<out>/check``. No flag: the same run with the three cameras' videos, into
``<out>/full``. Both keep the start pictures and the look's pictures (``start_*.png``, ``cue_red_tab_*.png``).
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

DT = 0.05  # s: one control step (20 Hz)
BACK = 0.08  # m: a hand at the grip comes in and leaves along its approach from this far back
SIDE_BACK, SIDE_UP = 0.035, 0.040  # m: a lever hand comes down from SIDE_UP over a point SIDE_BACK out along its approach, then in; it leaves the same way (from 80 mm out along the bar, the right arm runs into its joint limits)
LEVER_GRIP = 0.21  # share of 94 mm told the fingers on a lever's top and bottom edges: 19.7 mm on the 34 mm the slanted pinch reads, 36 to 38 N per pad (gb5d24), under the YAM's 40 N (told 16 mm: 43 to 45 N, gb5d23; 9.4 mm: the drive's cap of 50 N, gb5d22)
LEVER_OPEN = 0.60  # the side pinch comes in and leaves opened to 56 mm only (wide open, its upper finger swings towards the housing)
BAR_GRIP = 0.30  # on the grip: 28 mm told on 38 mm, 25 N per pad
BAR_GRIP_OPEN = 0.60  # a hand comes to the grip and leaves it opened to 56 mm only
HOLD_PUSH = np.array([15.0, 0.0, 3.0])  # N: the holder's press on the wall and its lift for the bar's weight, at the grasp point (the bar on the wall)
PRESS_PUSH = np.array([K.HOLD_PRESS, 0.0, 3.0])  # N: the press of the hand on the grip while a lever is locked: three quarters of it on the nearer cup
BAR_COM_B = np.array([-0.037, 0.0, 0.0])  # m in B: the bar's middle of mass with its cups and levers (parts' sizes and masses, uniform plastic): 29 mm behind the grip
CARRY_N = P.BAR_MASS * 9.81  # N: the holder lifts the bar's weight at its middle of mass while it is off the wall (lifted at the grasp point,
#   the holding hand sank 5.3 mm and the bar turned 3 deg in it: check run gb5d3)
FINGER_STIFFNESS = 5000.0  # N/m: each finger's drive (yam_arena GRIPPER)
PUSH_START, PUSH_STEP, PUSH_MOST = 10.0, 1.0, 40.0  # N: a release's pull along the lever's path: first, added each control step, most before the plan stops (LEVER_N passes centre)
LOCK_START = 4.0  # N: a lock's first push (the cam is loose up to LEVER_CAM_DEG)
LET_GO_DEG = 80.0  # deg from released: a released lever is let go here, short of centre (it comes to rest at LEVER_CAM_DEG)
LOCK_FROM_DEG = 40.0  # deg: a lock's push starts from a lever this far open or more (let go, a lever swung on to 52-59 deg and drifted 1 deg/s while the bar was moved: gb5d21)
HOME_DEG = 107.0  # deg: a pushed lever is left here, past centre (the cam takes it home)
OFF_WALL = 0.002  # m: the bar is moved this far off the wall
DRY_STEP_DEG = 1.5  # deg: in a dry run, a lever turns this far each control step it is worked
LOOK_S = 0.6  # s: the right hand holds still at its look before the picture is kept
LEAD_MOST_DEG = 4.0  # deg: a hand working a lever aims one step ahead of it, by the turn it read in the last step, at most this (aimed at the read angle, the hand trailed the lever by 2 to 3 deg near centre and pulled back on the pinch: gb5d12 to gb5d14)
SLIP_DEG, LIFT_TRIES = 5.0, 3  # a release whose lever turns back this far towards locked has slipped out of the pinch; pinched again, up to this many tries
LAST_LEAVE_S, LAST_READY_S = 0.9, 1.3  # s: the hands' way off the bar and to their ready poses at the end


class _Stop(Exception):
    """The plan cannot go on (said why); the run ends with the arms holding where they are."""


class Plan:
    """The demo's steps. Each step is a dict of moves per arm, planned when the step before it is through, from what
    is read then (``sim``), or in a dry run from the plan's own idea of the bar and the levers."""

    def __init__(self, m, sim: bool) -> None:
        import demo_kit as kit
        from yam_arena import station
        from yam_arena.constants import READY

        self.m, self.sim, self.kit, self.Move = m, sim, kit, kit.Move
        self.bases = {side: np.array(station.BASES[side], dtype=float) for side in ("left", "right")}
        self.ready = np.array(READY, dtype=float)
        self.phase, self.now, self.arms, self.failure = "start", 0.0, None, None
        self.dry_bar = (np.array(m.BAR_AT, dtype=float), np.eye(3))  # dry run: where the plan put the bar
        self.dry_theta = {s: P.LEVER_RAD for s in m.SIDES}
        self.releases: dict = {}  # per lever: the pull at which it passed centre, when, and where it went after
        self.locks: dict = {}
        self.out, self.pictures, self.kept = None, None, {}  # the folder pictures are kept in; the cameras' latest pictures; what was kept

    # --- what is read ------------------------------------------------------------------------------------------------
    def bar(self) -> tuple[np.ndarray, np.ndarray]:
        if self.sim:
            p, r = self.m.mk.pose("bar")
            return np.asarray(p, dtype=float), np.asarray(r, dtype=float)
        return self.dry_bar

    def theta(self, s: str) -> float:
        return self.m.LEVERS[s].theta() if self.sim else self.dry_theta[s]

    # --- where things are --------------------------------------------------------------------------------------------
    def world(self, b, bar=None) -> np.ndarray:
        p, r = bar if bar is not None else self.bar()
        return p + r @ np.asarray(b, dtype=float)

    def side_grasp(self, s: str, theta: float, kind: str, bar=None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(grasp point, rotation, the lever's point it works), world, of the side pinch ('pinch', checks.pinch_b) or
        the fingertip push ('push', checks.push_b) on lever ``s`` at ``theta``, on the bar as it is."""
        tcp_b, rot_b, point_b = (K.pinch_b if kind == "pinch" else K.push_b)(s, theta)
        p, r = bar if bar is not None else self.bar()
        return p + r @ tcp_b, r @ rot_b, p + r @ point_b

    def lever_order(self, arm: str, s: str, kind: str, degs) -> np.ndarray:
        """The finger order (the identity, or a half turn about the approach) for working lever ``s`` by ``kind``
        through the angles ``degs`` (checks.finger_order): the wrist camera off the tiles, then the larger smallest
        joint margin over the whole turn (picked at the start alone, the last lock's turn ran the wrist into joint 6's
        limit: dry run 15:08)."""
        order, camera_clear, worst_mm, margin = K.finger_order(arm, [self.side_grasp(s, math.radians(deg), kind)[:2] for deg in degs])
        if not camera_clear or worst_mm > 1.0 or margin < 5.0:
            raise _Stop(f"{self.phase}: the {arm} hand does not work lever {s} with its camera off the tiles and 5 deg of joint margin (camera clear: {camera_clear}, IK {worst_mm:.2f} mm, margin {margin:.1f} deg)")
        return order

    def side_in(self, arm: str, tcp: np.ndarray, rot: np.ndarray, grip: float) -> list:
        """A lever hand's way in: over the point SIDE_BACK out along its approach, down to it, then in."""
        out = tcp - SIDE_BACK * rot[:, 2]
        return [self.move(2.5, arm, "to the lever", pos=out + np.array([0.0, 0.0, SIDE_UP]), rot=rot, grip=grip, push=np.zeros(3)),
                self.move(0.8, arm, "beside the lever", pos=out), self.move(1.0, arm, "at the lever", pos=tcp)]  # fmt: skip

    def side_out(self, arm: str, tcp: np.ndarray, rot: np.ndarray, quick: bool = False) -> list:
        """A lever hand's way out: SIDE_BACK back along its approach, then up (towards the arms as well, the right arm
        came within 4 deg of a joint's limit: dry run 15:10)."""
        out = tcp - SIDE_BACK * rot[:, 2]
        return [self.move(0.4 if quick else 0.6, arm, "leaves the lever", pos=out),
                self.move(0.5 if quick else 0.8, arm, "clear of the bar", pos=out + np.array([0.0, 0.0, SIDE_UP]))]  # fmt: skip

    def out_dir(self, s: str, theta: float, bar=None) -> np.ndarray:
        """The lever's outer face's normal (world): the way its free end moves as it is released."""
        return (bar if bar is not None else self.bar())[1] @ np.array(P.lever_out_b(s, theta))

    def hand_rot(self, arm: str, pos: np.ndarray, rot: np.ndarray) -> np.ndarray:
        """The hand's rotation for a grasp at ``pos`` turned as ``rot``: of the two finger orders that reach it with
        5 deg or more of joint margin, the one nearer the hand's present turn (the other order would spin the wrist
        half a turn on the way, into joint 6's limit: dry run gb5d1q)."""
        from yam_arena import ik

        now = self.hand_now(arm)[1]
        found = []
        for orders in ("given", "flipped"):
            q, err_mm, err_deg, margin = K.solve(arm, pos, rot, orders=orders)
            if err_mm <= 1.0 and err_deg <= 1.0 and margin >= 5.0:
                reached = ik.tcp_pose(q, self.bases[arm])[1]
                turn = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(now.T @ reached) - 1.0) / 2.0))))
                found.append((turn, -margin, reached))
        if not found:
            raise _Stop(f"{self.phase}: the {arm} hand does not reach {np.round(pos, 4).tolist()} with 5 deg of joint margin")
        return min(found, key=lambda f: (f[0], f[1]))[2]

    def hand_now(self, arm: str) -> tuple[np.ndarray, np.ndarray]:
        """Where the hand was last sent (grasp point, rotation)."""
        from yam_arena import ik

        if self.arms is None or not self.arms[arm].moves:
            return ik.tcp_pose(self.ready, self.bases[arm])
        pos, rot = self.arms[arm].target(self.arms[arm].seconds + 1.0)[:2]
        return np.asarray(pos, dtype=float), np.asarray(rot, dtype=float)

    def move(self, seconds: float, arm: str, label: str, pos=None, rot=None, grip=None, push=None, at=None):
        return self.Move(seconds, pos=pos, rot=rot, grip=grip, label=label, push=None if push is None else np.asarray(push, dtype=float), push_at=at)

    def keep(self, name: str, camera: str) -> None:
        """Keep the camera's latest picture as ``<name>.png`` (640 x 360) and ``<name>_policy.png`` (320 x 176), with
        the size of its red patch at the policy's size (in a dry run there are no pictures)."""
        if self.out is None or self.pictures is None:
            return
        import cv2

        frame = np.asarray(self.pictures[camera])
        small = self.kit.policy_picture(frame)
        self.out.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(self.out / f"{name}.png"), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(self.out / f"{name}_policy.png"), cv2.cvtColor(small, cv2.COLOR_RGB2BGR))
        self.kept[name] = {"at s": round(self.now, 2), "red patch at 320 x 176": red_patch(small)}

    def look(self) -> dict:
        """(0) The right hand looks at the right cup: the red tab and the joint in its wrist camera's view."""
        self.phase = "right looks at the right cup"
        tcp, rot = K.look_pose(float(self.bar()[0][1]), float(self.bar()[0][2]) - K.TOP)
        return {"right": [self.move(2.5, "right", "to the look", pos=tcp, rot=rot, grip=1.0), self.move(LOOK_S, "right", "looks")]}

    def to_ready(self, arm: str, seconds: float = 2.5) -> list:
        from yam_arena import ik

        pos, rot = ik.tcp_pose(self.ready, self.bases[arm])
        return [self.move(seconds, arm, "to ready", pos=pos, rot=rot, grip=1.0, push=np.zeros(3))]

    # --- the steps ---------------------------------------------------------------------------------------------------
    def turned(self, s: str, theta0: float, theta: float, bar=None) -> np.ndarray:
        """The rotation (world) that turns lever ``s`` of the bar from ``theta0`` to ``theta`` about its hinge's axis."""
        k = (bar if bar is not None else self.bar())[1] @ np.array(P.lever_axis_b(s))
        a = theta - theta0
        cross = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
        return np.eye(3) + math.sin(a) * cross + (1.0 - math.cos(a)) * cross @ cross

    def dry_turn(self, s: str, towards_locked: bool) -> None:
        """Dry run: the worked lever turns DRY_STEP_DEG."""
        if not self.sim:
            step = math.radians(DRY_STEP_DEG)
            self.dry_theta[s] = min(P.LEVER_RAD, self.dry_theta[s] + step) if towards_locked else max(0.0, self.dry_theta[s] - step)

    def release(self, arm: str, s: str):
        """Pinch lever ``s``'s top and bottom edges from outside the bar's end and pull it out over centre, the hand
        turning with the lever one step ahead of it (a hand that keeps its turn lets the lever spin between its pads;
        one aimed at the read angle trailed it near centre and the lever slipped back: gb5d12, gb5d13); let it go at
        LET_GO_DEG and leave. A lever that turns SLIP_DEG back has slipped: the hand opens, backs off and pinches it
        again, LIFT_TRIES times at most."""
        self.phase = f"{arm} releases lever {s}"
        slips = []
        for _ in range(LIFT_TRIES):
            bar = self.bar()
            theta0 = self.theta(s)
            tcp0, rot_b, patch0 = self.side_grasp(s, theta0, "pinch", bar)
            rot0 = rot_b @ self.lever_order(arm, s, "pinch", (math.degrees(theta0), P.LEVER_CENTRE_DEG, LET_GO_DEG))
            at = rot0.T @ (patch0 - tcp0)  # the lever's point in the hand: it stays there while the hand turns with the lever

            def held(theta: float, theta0=theta0, tcp0=tcp0, patch0=patch0, rot0=rot0, bar=bar) -> tuple[np.ndarray, np.ndarray]:
                """(grasp point, hand turn) with the lever at theta."""
                turn = self.turned(s, theta0, theta, bar)
                return self.world(P.lever_tip_b(s, theta), bar) + turn @ (tcp0 - patch0), turn @ rot0

            yield {arm: self.side_in(arm, tcp0, rot0, LEVER_OPEN) + [self.move(0.6, arm, "pinches the lever", grip=LEVER_GRIP)]}
            push, start, least, slipped, before = PUSH_START, self.now, theta0, False, theta0
            while self.theta(s) >= P.CENTRE_RAD:
                theta = self.theta(s)
                if theta > least + math.radians(SLIP_DEG):  # turned back towards locked: out of the pinch
                    slipped = True
                    break
                least = min(least, theta)
                force = push * self.out_dir(s, theta, bar)
                effort = float(np.abs(self.kit.point_jacobian(self.arms[arm].q, self.bases[arm], at).T @ force / self.kit.JOINT_EFFORT).max())
                if push > PUSH_MOST or effort > 0.95:
                    raise _Stop(f"{self.phase}: the lever stood at {math.degrees(theta):.1f} deg with {push - PUSH_STEP:.1f} N pulled ({effort:.2f} of a joint's effort at {push:.1f} N)")
                ahead = max(-math.radians(LEAD_MOST_DEG), min(0.0, theta - before))  # the turn in the last step, towards released only
                before = theta
                pos, rot = held(theta + ahead)
                yield {arm: [self.move(DT, arm, "releases the lever", pos=pos, rot=rot, push=force, at=at)]}
                self.dry_turn(s, towards_locked=False)
                push += PUSH_STEP
            if not slipped:
                break
            slips.append({"at s": round(self.now, 2), "least deg": round(math.degrees(least), 1), "pull N": round(push - PUSH_STEP, 1)})
            pos, rot = self.hand_now(arm)
            yield {arm: [self.move(0.3, arm, "slipped: stops pulling", push=np.zeros(3), grip=LEVER_OPEN)] + self.side_out(arm, pos, rot)}
        else:
            raise _Stop(f"{self.phase}: the lever slipped out of the pinch {LIFT_TRIES} times ({slips})")
        self.releases[s] = {"passed centre at s": round(self.now, 2), "pull N then": round(push - PUSH_STEP, 1), "pulling for s": round(self.now - start, 2), "slips": slips}
        let_go, let_rot = held(math.radians(LET_GO_DEG))
        yield {arm: [self.move(0.6, arm, "holds the lever out", pos=let_go, rot=let_rot, push=np.zeros(3))]}
        self.releases[s]["held at deg"] = round(math.degrees(self.theta(s)), 1)
        if not self.sim:
            self.dry_theta[s] = math.radians(LET_GO_DEG)
        yield {arm: [self.move(0.4, arm, "lets the lever go", grip=LEVER_OPEN)] + self.side_out(arm, let_go, let_rot)}
        if not self.sim:
            self.dry_theta[s] = P.CAM_RAD
        self.releases[s]["rests at deg"] = round(math.degrees(self.theta(s)), 1)

    def push_home(self, arm: str, s: str, last: bool = False, other=None):
        """Push lever ``s`` home with the closed fingertips on its outer face, the hand turning with half the lever's
        swing (checks.PUSH_TURN), one step ahead of it, until the lever is past centre and at HOME_DEG; ease and leave
        (``last``: on to the ready pose, quickly, with ``other``'s moves, the other arm's way out, in the same step)."""
        self.phase = f"{arm} pushes lever {s} home"
        bar = self.bar()
        theta0 = self.theta(s)
        if theta0 < math.radians(LOCK_FROM_DEG) or theta0 > P.CENTRE_RAD:
            raise _Stop(f"{self.phase}: the lever stands at {math.degrees(theta0):.1f} deg, not in the cam's loose range from {LOCK_FROM_DEG:g} deg")
        tcp0, rot_b, face0 = self.side_grasp(s, theta0, "push", bar)
        order = self.lever_order(arm, s, "push", (math.degrees(theta0), P.LEVER_CENTRE_DEG, HOME_DEG, P.LEVER_DEG))  # kept while it turns
        rot0 = rot_b @ order
        at = rot0.T @ (face0 - tcp0)  # the fingertips' point in the hand

        def held(theta: float) -> tuple[np.ndarray, np.ndarray]:
            tcp, rot, _ = self.side_grasp(s, theta, "push", bar)
            return tcp, rot @ order

        yield {arm: self.side_in(arm, tcp0, rot0, 0.0)}
        push, start, before = LOCK_START, self.now, theta0
        while self.theta(s) < math.radians(HOME_DEG):
            theta = self.theta(s)
            force = -push * self.out_dir(s, theta, bar)  # into the outer face: towards locked
            effort = float(np.abs(self.kit.point_jacobian(self.arms[arm].q, self.bases[arm], at).T @ force / self.kit.JOINT_EFFORT).max())
            if push > PUSH_MOST or effort > 0.95:
                raise _Stop(f"{self.phase}: the lever stood at {math.degrees(theta):.1f} deg with {push - PUSH_STEP:.1f} N pushed ({effort:.2f} of a joint's effort at {push:.1f} N)")
            ahead = min(math.radians(LEAD_MOST_DEG), max(0.0, theta - before))
            before = theta
            pos, rot = held(min(P.LEVER_RAD, theta + ahead))
            yield {arm: [self.move(DT, arm, "pushes the lever home", pos=pos, rot=rot, push=force, at=at)]}
            self.dry_turn(s, towards_locked=True)
            push += PUSH_STEP
        self.locks[s] = {"at home s": round(self.now, 2), "push N then": round(push - PUSH_STEP, 1), "pushing for s": round(self.now - start, 2)}
        pos, rot = self.hand_now(arm)
        ready_s = LAST_READY_S if last else 2.5
        moves = {arm: [self.move(0.4, arm, "eases", push=np.zeros(3))] + self.side_out(arm, pos, rot, quick=last)}
        if last:
            moves[arm] += self.to_ready(arm, ready_s)
            moves.update(other or {})
        yield moves
        if not self.sim:
            self.dry_theta[s] = P.LEVER_RAD
        self.locks[s]["lever at the end deg"] = round(math.degrees(self.theta(s)), 1)

    def take_grip(self, arm: str, at: float, label: str) -> dict:
        """Take the grip ``at`` (B y) with the fingers above and below and press the bar on with HOLD_PUSH."""
        tcp = self.world((P.GRIP_X, at, 0.0))
        rot = self.hand_rot(arm, tcp, K.FORWARD_PINCH_UPDOWN)
        approach = rot[:, 2]
        return {arm: [self.move(2.5, arm, f"to the grip {label}", pos=tcp - BACK * approach, rot=rot, grip=BAR_GRIP_OPEN, push=np.zeros(3)),
                      self.move(1.5, arm, "at the grip", pos=tcp),
                      self.move(0.6, arm, "takes the grip", grip=BAR_GRIP),
                      self.move(0.6, arm, "holds the bar", push=HOLD_PUSH, at=np.zeros(3))]}  # fmt: skip

    def outside(self, arm: str, s: str, kind: str) -> dict:
        """Come to outside lever ``s``'s end of the bar, BACK short of the grasp for ``kind`` ('pinch' or 'push')."""
        theta = self.theta(s)
        tcp, rot_b, _ = self.side_grasp(s, theta, kind)
        degs = (math.degrees(theta), P.LEVER_CENTRE_DEG, LET_GO_DEG) if kind == "pinch" else (math.degrees(theta), P.LEVER_CENTRE_DEG, HOME_DEG, P.LEVER_DEG)
        rot = rot_b @ self.lever_order(arm, s, kind, degs)
        grip = LEVER_OPEN if kind == "pinch" else 0.0
        return {arm: [self.move(2.5, arm, "outside the bar's end", pos=tcp - SIDE_BACK * rot[:, 2] + np.array([0.0, 0.0, SIDE_UP]), rot=rot, grip=grip, push=np.zeros(3))]}

    def move_bar(self) -> dict:
        self.phase = "right moves the bar"
        pos, rot = self.hand_now("right")
        bar = self.bar()
        r = bar[1]
        off, along = r @ np.array([-OFF_WALL, 0.0, 0.0]), r @ np.array([0.0, -P.MOVE, 0.0])
        com = rot.T @ (self.world(BAR_COM_B, bar) - pos)  # the bar's middle of mass in the hand
        carry = np.array([0.0, 0.0, CARRY_N])
        moves = [self.move(0.5, "right", "eases its press", push=carry, at=com), self.move(0.5, "right", "off the wall", pos=pos + off),
                 self.move(3.0, "right", "moves the bar", pos=pos + off + along), self.move(0.6, "right", "back on the wall", pos=pos + along),
                 self.move(0.6, "right", "presses the bar on", push=HOLD_PUSH, at=np.zeros(3))]  # fmt: skip
        if not self.sim:
            p, r = self.dry_bar
            self.dry_bar = (p + along, r)
        return {"right": moves}

    def press_harder(self, arm: str) -> dict:
        return {arm: [self.move(0.6, arm, "presses the cup on", push=PRESS_PUSH, at=np.zeros(3))]}

    def let_go_of_grip(self, arm: str, last: bool = False) -> list:
        pos, rot = self.hand_now(arm)
        leave_s = LAST_LEAVE_S if last else 1.2
        moves = [self.move(0.5 if not last else 0.4, arm, "eases its press", push=np.zeros(3)), self.move(0.5 if not last else 0.3, arm, "opens", grip=BAR_GRIP_OPEN),
                 self.move(leave_s, arm, "leaves the grip", pos=pos - BACK * rot[:, 2])]  # fmt: skip
        return moves + (self.to_ready(arm, LAST_READY_S) if last else [])

    def steps(self):
        try:
            yield self.look()  # (0)
            self.keep("cue_red_tab_right_wrist", "right_cam")
            yield from self.release("right", "r")  # (a)
            self.phase = "right takes the grip"
            yield {**self.take_grip("right", K.HOLD_AT, "near the left cup"), **self.outside("left", "l", "pinch")}  # (b)
            yield from self.release("left", "l")  # (c)
            yield self.move_bar()  # (d)
            self.phase = "right presses the left cup on"
            yield {**self.press_harder("right"), **self.outside("left", "l", "push")}  # (e)
            yield from self.push_home("left", "l")
            self.phase = "right lets go of the grip"
            yield {"right": self.let_go_of_grip("right")}  # (f)
            self.phase = "left presses the right cup on"
            yield {**self.take_grip("left", K.PRESS_AT, "near the right cup"), **self.outside("right", "r", "push")}
            yield self.press_harder("left")
            yield from self.push_home("right", "r", last=True, other={"left": self.let_go_of_grip("left", last=True)})
            self.phase = "both still"
        except _Stop as stop:
            self.failure, self.phase = str(stop), "stopped"
            print(f"PLAN STOPPED: {stop}", flush=True)


def director_for(plan: Plan, sim: bool):
    """The director kit.run calls before every step: when both arms are through their moves, the next step's moves,
    both arms' ends evened with a wait."""
    script = plan.steps()
    state = {"over": False}

    def director(t: float, arms: dict, pictures) -> None:
        plan.now, plan.pictures = t, pictures
        if t == 0.0 and pictures is not None:
            for camera, name in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
                plan.keep(name, camera)
        if state["over"] or (sim and not plan.m.mk.MECH.poses):
            return
        plan.arms = arms
        if t < max(arm.seconds for arm in arms.values()) - 1e-6:
            return
        step = next(script, None)
        if step is None:
            state["over"] = True
            return
        for side, moves in step.items():
            arms[side].moves.extend(moves)
        end = max(arm.seconds for arm in arms.values())
        for arm in arms.values():
            if arm.seconds < end - 1e-9:
                arm.moves.append(plan.Move(end - arm.seconds, label="waits"))

    return director


def red_patch(picture: np.ndarray) -> dict:
    """The red tab's pixels in an RGB picture: how many, and the box round them (width x height, px)."""
    rgb = picture.astype(int)
    red = (rgb[..., 0] > 120) & (rgb[..., 0] > 2 * rgb[..., 1] + 20) & (rgb[..., 0] > 2 * rgb[..., 2] + 20)
    ys, xs = np.nonzero(red)
    if not len(xs):
        return {"px": 0}
    return {"px": int(len(xs)), "box (w x h)": [int(xs.max() - xs.min() + 1), int(ys.max() - ys.min() + 1)], "middle (x, y)": [round(float(xs.mean()), 1), round(float(ys.mean()), 1)]}


def finger_force(m, arm: str, told: float) -> float | None:
    """N per finger the hand presses with, from its finger drive: each finger held short of where it was told (share
    ``told`` of 94 mm) by half the openings' difference, times the drive's stiffness (at most the drive's 50 N)."""
    if arm not in m.mk.MECH.grippers:
        return None
    return round(min(50.0, FINGER_STIFFNESS * max(0.0, float(m.mk.gripper(arm)[1]) - told) * 0.094 / 2), 1)


def hands_from_ready(plan: Plan) -> dict | None:
    """mm: each hand's grasp point (as sent, at the run's end) from its ready pose's."""
    from yam_arena import ik

    if plan.arms is None:
        return None
    return {side: round(1000 * float(np.linalg.norm(np.asarray(plan.arms[side].target(plan.now)[0]) - ik.tcp_pose(plan.ready, plan.bases[side])[0])), 1) for side in ("left", "right")}


def main() -> int:
    import demo_kit as kit

    quick_run, check = "--quick" in sys.argv, "--check" in sys.argv
    args = [a for a in sys.argv[1:] if a not in ("--quick", "--check")]
    out = Path(args[0])
    if quick_run:
        from yam_arena import task_file

        m = task_file.load(str(HERE / "task.py")).module
        plan = Plan(m, sim=False)
        found = kit.dry_run({"left": [], "right": []}, plan.bases, director=director_for(plan, sim=False), tail_s=2.0, limit_s=m.RUN_LIMIT_STEPS / 20.0)
        found["plan"] = {"stopped": plan.failure, "releases": plan.releases, "locks": plan.locks}
        print(f"DRY {json.dumps(found, default=str)}", flush=True)
        out.mkdir(parents=True, exist_ok=True)
        (out / "quick.json").write_text(json.dumps(found, indent=1, default=str))
        return 0
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    plan = Plan(m, sim=True)
    folder = out / ("check" if check else "full")
    plan.out = folder
    director = director_for(plan, sim=True)
    bar0 = np.array(m.BAR_AT, dtype=float)

    def extra() -> dict:
        p, r = mk.pose("bar")
        arms = plan.arms
        row = {"phase": plan.phase, "levers deg": {s: round(math.degrees(m.LEVERS[s].theta()), 2) for s in m.SIDES}, "sealed": {s: m.sealed(s) for s in m.SIDES},
               "press N": {s: round(mk.contact_force(f"cup_{s}", "panel"), 2) for s in m.SIDES},
               "push N": {side: [round(float(v), 1) for v in arms[side].state(plan.now)[4]] for side in ("left", "right")} if arms else None,
               "fingers N": {side: finger_force(m, side, float(arms[side].state(plan.now)[2])) for side in ("left", "right")} if arms else None,
               "bar from its start mm": [round(1000 * float(v), 1) for v in (np.asarray(p) - bar0)], "bar off level deg": round(m.bar_level_deg(), 2),
               "bar's top off the wall deg": round(math.degrees(math.atan2(-float(np.asarray(r)[0, 2]), float(np.asarray(r)[2, 2]))), 2),
               "IK mm": {side: round(arms[side].last_ik[0], 2) for side in ("left", "right")} if arms else None,
               "speeds (mm/s, deg/s)": {n: [round(1000 * float(np.linalg.norm(mk.velocity(n)[0])), 2), round(math.degrees(float(np.linalg.norm(mk.velocity(n)[1]))), 2)] for n in ("bar", "cup_l", "cup_r", "lever_l", "lever_r")},
               "at rest": m.at_rest(), "let go": m.let_go()}  # fmt: skip
        return row

    result = kit.run(embodiment, {"left": [], "right": []}, folder, seed=0, director=director, extra=extra, limit_s=m.RUN_LIMIT_STEPS / hz, tail_s=3.0, sheet_frames=16, record=not check)
    rows = result["rows"]
    scores = [row["info"].get("score", 0.0) for row in rows]
    per: dict = {}
    for row in rows:
        e = row["extra"]
        per.setdefault(e["phase"], []).append((row, e))
    phases = []
    for phase, got in per.items():
        es = [e for _, e in got]
        phases.append({"phase": phase, "from s": got[0][0]["t"], "seconds": round(len(got) / hz, 2),
                       "levers deg (start, end, least, most)": {s: [es[0]["levers deg"][s], es[-1]["levers deg"][s], min(e["levers deg"][s] for e in es), max(e["levers deg"][s] for e in es)] for s in m.SIDES},
                       "press N most": {s: max(e["press N"][s] for e in es) for s in m.SIDES},
                       "push N most (left, right)": [max(float(np.linalg.norm(e["push N"]["left"])) for e in es if e["push N"]), max(float(np.linalg.norm(e["push N"]["right"])) for e in es if e["push N"])] if es[0]["push N"] else None,
                       "fingers N most": {side: max(((e["fingers N"] or {}).get(side) or 0.0) for e in es) for side in ("left", "right")},
                       "bar from its start mm (start, end)": [es[0]["bar from its start mm"], es[-1]["bar from its start mm"]],
                       "bar off level deg most": max(e["bar off level deg"] for e in es), "top off the wall deg (least, most)": [min(e["bar's top off the wall deg"] for e in es), max(e["bar's top off the wall deg"] for e in es)],
                       "speeds most (mm/s, deg/s)": {n: [max(e["speeds (mm/s, deg/s)"][n][0] for e in es), max(e["speeds (mm/s, deg/s)"][n][1] for e in es)] for n in es[0]["speeds (mm/s, deg/s)"]},
                       "steps at rest": sum(1 for e in es if e["at rest"])})  # fmt: skip
    readings = {"plan stopped": plan.failure, "releases": plan.releases, "locks": plan.locks, "score": scores[-1],
                "first reached": {lv: next((row["t"] for row, sc in zip(rows, scores) if sc >= lv), None) for lv in (0.5, 1.0)},
                "seconds": rows[-1]["t"], "steps": len(rows), "ended": result.get("termination_reason"), "per phase": phases, "report": result["report"],
                "worst IK": result["worst_ik"], "step ms (median)": result.get("median_step_ms"), "pictures": plan.kept,
                "last moves end at s": round(max(arm.seconds for arm in plan.arms.values()), 2) if plan.arms else None,
                "hands from ready at the end mm": hands_from_ready(plan)}  # fmt: skip
    (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
    print(f"DEMO check: score {scores[-1]:.2f} after {result['seconds']:.1f} s ({result['steps']} steps of {m.RUN_LIMIT_STEPS}); first reached {readings['first reached']}", flush=True)
    print(f"READINGS {json.dumps(readings, default=str)}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
