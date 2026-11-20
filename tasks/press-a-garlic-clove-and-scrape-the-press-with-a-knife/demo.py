#!/usr/bin/env python3
"""Reference runs of the garlic press task.

    scripts/rta demo <id> mech/tasks/garlic_press [run,run,...] [--quick]

The right hand picks the clove up with a light pinch across its width (0.5 mm per finger past touching: about
2.5 N), carries it over the open press's chamber, turned lengthways, and lets it go with its fingertips 2 mm over the
rim. Then it pinches the open upper handle lightly across its width 45 mm from its end, carries it along its arc
from its stop (120 degrees) to 74 degrees, past where its weight turns it shut (84), and lets go: it falls onto the
clove. Meanwhile the left hand fetches the knife and holds it up. The right hand, closed, pushes the end of the lower handle until the
press stands 51 mm past the board's edge, its chamber over the saucer; sets its closed fingertips flat on the upper
handle's top face 15 mm from its end and presses, tilted half the handle's angle from straight down towards its grip
end (so that the fingertips do not slide on the face and the press does not skate on the board), following the
handle as it goes down (30 N asked at first, 10 N more every second up to 75 N, which the arm gives as about 66 N),
until the handle is closed. (A pinch across the handle presses only through the pads' friction: it slid down the
handle at 48 N, short of the clove's yield: run b3g8.) Then it pinches the closed press across both handles there
and lifts it 6 cm. The left hand turns the knife to point at the
press and runs its blade flat across under the sieve, its top face pushed lightly up against it; sets the blade's tip
on the saucer's well; lays the knife on the bench. The right hand sets the press down where it was, and both go
away.

What the script reads from the simulator: where the clove, the press (both parts), the knife and the saucer are,
to put the hands on them (a person sees all that); the press's place on the board, to stop pushing it; how far the
handle is closed, to stop pressing (a person sees the handles meet). Not the garlic's amounts, not the score.

Runs. ``full``: 1.0. ``pressed``: as full, no scraping: the knife is not used (0.5). ``half``: the blade run only
part of the way across (0.75). ``in_chamber``: the clove in, the handle onto it, nothing pressed (0.25). ``across``:
the clove let go across the chamber, then as full. ``held``: as full, the right hand stays on the press (0.75).

``--quick``: every run's plan against the arms' reach and joint limits, without the simulator.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

RUNS = ("full", "pressed", "half", "in_chamber", "across", "held")
TIP = 0.0129  # m the fingertips' ends reach past the grasp point
UP = 0.06  # m over a grasp before and after
OPENING = 0.0939  # m between the pads fully open (grip 1)
PINCH = 0.0005  # m per finger past touching for the clove: about 2.5 N per pad (5,000 N/m)
PUSH_FIRST, PUSH_MORE, PUSH_MOST = 30.0, 1.0, 75.0  # N asked at first, more every LOOK_S, at most (the arm then gives about 66 N: trial b3g4)
PRESS_U = 0.140  # where the right hand presses the upper handle and pinches the closed press to lift it: 15 mm from its end
HANDLE_TOP_W = 0.023  # the upper handle's top face (closed coordinates)
PAD_HALF = 0.0135  # m: half the pads' width, which lies along the handle when the fingers close across it
HALF_PART = 0.35  # the run ``half``: the blade run 35 % of the way across. Recorded (b3g10) it took four of the five rows
# of holes, not the three planned (the knife went 3 to 7 mm further): 73 % on the saucer, 1.0. Kept as recorded: 0.75 is
# shown by the rule test (one full-length recording per task from here on, owner, 2026-11-20).
LIFT_W = 0.014  # the grasp point over the press's underside when it is pinched to be lifted: the fingertips 1 mm over it
LOOK_S = 0.1  # s between two looks while sliding and pressing
CLOSED_DEG = 2.0  # the handle this near closed: pressed through
SLIDE_STEP = 0.005  # m a look while pushing the press to the board's edge
SCRAPE_UP_N = 2.0  # N the blade is pushed up against the sieve
LIFT = 0.06  # m the closed press is lifted over the saucer
DRY_PHI_ON_CLOVE = math.radians(34.5)  # the dry run's handle on the clove (test of the arrival)
HANDLE_U, HANDLE_W = 0.110, 0.017  # where the right hand pinches the open upper handle (its middle, 45 mm from its end)
OVER_DEG = (120.0, 112.0, 104.0, 96.0, 90.0, 84.0, 78.0, 74.0)  # the handle carried over: its weight turns it shut below 84
HANDLE_WIDTH = 0.026


class Plan:
    def __init__(self, m, see=None) -> None:
        import demo_kit as kit

        self.m, self.see, self.kit, self.Move = m, see, kit, kit.Move
        self.dry = {"shift": 0.0, "lift": 0.0, "phi": m.OPEN_MOST, "knife": None}  # the dry run's world: what the plan has done
        self.held_knife = None  # (offset, rotation) of the knife in the left hand's frame, once it holds it

    # --- where things are: from the simulator, or in the dry run from what the plan has done ---
    def look(self) -> dict | None:
        return self.see() if self.see is not None else None

    def lower(self) -> tuple[np.ndarray, np.ndarray]:
        seen = self.look()
        if seen is not None:
            return seen["press"]
        return self.m.START_HINGE + np.array([0.0, self.dry["shift"], self.dry["lift"]]), np.eye(3)

    def phi(self) -> float:
        seen = self.look()
        return float(seen["hinge"]) if seen is not None else self.dry["phi"]

    def on_lower(self, u: float, v: float, w: float) -> np.ndarray:
        position, rotation = self.lower()
        return position + rotation @ np.array(self.m.uvw(u, v, w))

    def on_upper(self, u: float, v: float, w: float) -> np.ndarray:
        seen = self.look()
        if seen is not None:
            position, rotation = seen["upper"]
        else:
            position, rotation = self.m.mk.joint_pose(self.m.mk.MECH.joints["hinge"], self.phi(), self.lower())
        return position + rotation @ np.array(self.m.uvw(u, v, w))

    def clove(self) -> tuple[np.ndarray, np.ndarray]:
        seen = self.look()
        if seen is not None:
            return seen["clove"]
        k, side, yaw, x, y = self.m.STATE["lie"]
        position, quat = self.m.clove_pose(k, side, yaw, x, y)
        return position, self.m.mk.quat_to_matrix(quat)

    def knife(self) -> tuple[np.ndarray, np.ndarray]:
        seen = self.look()
        if seen is not None:
            return seen["knife"]
        if self.dry["knife"] is not None:
            return self.dry["knife"]
        return np.array([*self.m.KNIFE_AT, self.m.TOP + self.m.KNIFE_HANDLE[2] / 2]), np.eye(3)

    def saucer(self) -> np.ndarray:
        seen = self.look()
        return seen["saucer"][0] if seen is not None else np.array([*self.m.SAUCER_XY, self.m.TOP])

    def hand(self, side: str) -> tuple[np.ndarray, np.ndarray]:
        seen = self.look()
        return seen["hands"][side] if seen is not None else (None, None)

    # --- the steps ---
    def pick_clove(self):
        m, Move, down = self.m, self.Move, self.kit.down
        k = m.CLOVES.index(m.STATE["clove"])
        _, width, thick, _ = m.CLOVE_SIZES[k]
        side = m.STATE["lie"][1]
        across = thick if side else width  # what the fingers close across
        position, rotation = self.clove()
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])  # its length's direction; the fingers close across it
        grasp = np.array([position[0], position[1], m.TOP + m.BOARD_T + TIP + 0.001])
        up = np.array([0.0, 0.0, UP])
        self.pinch = (across - 2 * PINCH) / OPENING
        self.wide = min(1.0, (across + 0.024) / OPENING)
        yield {"right": [Move(2.0, pos=grasp + up, rot=down(yaw), grip=self.wide, label="over the clove"), Move(1.2, pos=grasp, label="down to it"),
                         Move(0.8, grip=self.pinch, label="pinch"), Move(0.8, pos=grasp + up, label="lift it")]}  # fmt: skip

    def drop_clove(self, across: bool = False):
        """Over the chamber, lengthways (or across it), let go with the fingertips 2 mm over the rim."""
        m, Move, down = self.m, self.Move, self.kit.down
        m_mid = (m.CHAMBER_U[0] + m.CHAMBER_U[1]) / 2
        _, rotation = self.lower()
        along = math.atan2(-rotation[1, 1], -rotation[0, 1])  # the press's line, tip to grip end: its parts' -y
        yaw = along + (math.pi / 2 if across else 0.0)
        _, held = self.clove()
        now = math.atan2(held[1, 0], held[0, 0])  # the clove's (and the hand's) turn now
        yaw = min((yaw, yaw + math.pi, yaw - math.pi), key=lambda a: abs(math.remainder(a - now, 2 * math.pi)))  # the nearer way round
        target = self.on_lower(m_mid, 0.0, m.RIM_W + 0.002 + TIP)
        correction = np.zeros(3)
        hand_at, hand_rot = self.hand("right")
        if hand_at is not None:  # where the clove sits in the fingers, carried over to the new turn of the hand
            clove_at, _ = self.clove()
            in_hand = hand_rot.T @ (clove_at - hand_at)
            new_rot = down(yaw)
            correction = new_rot @ in_hand
            correction[2] = 0.0
        place = target - correction
        up = np.array([0.0, 0.0, UP])
        yield {"right": [Move(2.5, pos=place + up, rot=down(yaw), label="over the chamber"), Move(1.2, pos=place, label="down to the rim"),
                         Move(0.6, grip=self.wide, label="let it go"), Move(0.8, pos=place + up, label="up")]}  # fmt: skip

    def handle_over(self):
        """Right hand, pointing down: a light pinch across the open upper handle (0.5 mm per finger past touching) at
        HANDLE_U, carried along the handle's arc through OVER_DEG and let go; the handle falls onto the clove. The
        handle turns between the pads as it goes. The hand comes to it level, from 40 mm on the grip end's side (from
        above it cannot reach), and leaves 30 mm up. (From the saucer's side the left hand could not: reach4, reach6.)"""
        m, Move, down = self.m, self.Move, self.kit.down
        lower, rotation = self.lower()
        yaw = math.atan2(rotation[1, 0], rotation[0, 0]) + math.pi / 2  # the fingers close across the handle (along the press's v)
        arc = []
        for deg in OVER_DEG:
            u, w = m.opened(HANDLE_U, HANDLE_W, math.radians(deg))
            arc.append(lower + rotation @ np.array(m.uvw(u, 0.0, w)))
        gripward = rotation @ np.array([0.0, -1.0, 0.0])  # along the press, towards its grip end
        wide, light = (HANDLE_WIDTH + 0.024) / OPENING, (HANDLE_WIDTH - 2 * PINCH) / OPENING
        moves = [Move(1.0, rot=down(yaw), grip=wide, label="turn across"), Move(1.8, pos=arc[0] + 0.040 * gripward, label="beside the open handle"),
                 Move(1.0, pos=arc[0], label="round it"), Move(0.6, grip=light, label="pinch it lightly")]  # fmt: skip
        moves += [Move(0.5, pos=p, label="bring it over") for p in arc[1:]]
        moves += [Move(0.5, grip=wide, label="let it go"), Move(0.8, pos=arc[-1] + np.array([0.0, 0.0, 0.030]), label="up"),
                  Move(1.5, pos=np.array([0.20, -0.25, m.TOP + 0.16]), rot=down(0.0), label="away")]  # fmt: skip
        yield {"right": moves}
        self.dry["phi"] = DRY_PHI_ON_CLOVE

    @staticmethod
    def together(*plans) -> dict:
        """One step out of one-step plans for different hands, each run to its end."""
        step = {}
        for plan in plans:
            for one in plan:
                step.update(one)
        return step

    def fetch_knife(self):
        m, Move, down = self.m, self.Move, self.kit.down
        position, rotation = self.knife()
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])  # the knife's length; the fingers close across its handle
        grasp = np.array([position[0], position[1], m.TOP + TIP + 0.001])
        up = np.array([0.0, 0.0, UP])
        yield {"left": [Move(2.0, pos=grasp + up, rot=down(yaw), grip=0.45, label="over the knife"), Move(1.2, pos=grasp, label="down to it"),
                        Move(0.8, grip=0.0, label="close"), Move(1.0, pos=grasp + np.array([0.0, 0.0, 0.10]), label="lift it")]}  # fmt: skip
        self.dry["knife"] = (grasp + np.array([0.0, 0.0, 0.10 - TIP - 0.001 + m.KNIFE_HANDLE[2] / 2]), np.eye(3))

    def grip_knife(self) -> None:
        """Note how the knife sits in the left hand (the simulator's poses; in the dry run as grasped)."""
        hand_at, hand_rot = self.hand("left")
        position, rotation = self.knife()
        if hand_at is None:
            hand_rot = self.kit.down(0.0)
            hand_at = position + np.array([0.0, 0.0, TIP + 0.001 - self.m.KNIFE_HANDLE[2] / 2])
        self.held_knife = (hand_rot.T @ (position - hand_at), hand_rot.T @ rotation)

    def hand_for_knife(self, position: np.ndarray, rotation: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """The left hand's grasp point and turn that put the held knife at ``position`` turned ``rotation``."""
        offset, turn = self.held_knife
        hand_rot = rotation @ turn.T
        return position - hand_rot @ offset, hand_rot

    def slide(self):
        """Right hand, closed: the end of the lower handle pushed until the press stands PRESSING_OVERHANG past the
        board's edge."""
        m, Move, down = self.m, self.Move, self.kit.down
        end = self.on_lower(0.155, 0.0, 0.003)
        _, rotation = self.lower()
        towards = rotation @ np.array([0.0, 1.0, 0.0])  # along the press, towards its tip
        behind = end - towards * (0.0064 + 0.002) + np.array([0.0, 0.0, TIP])
        yield {"right": [Move(2.0, pos=behind + np.array([0.0, 0.0, UP]), rot=down(0.0), grip=0.0, label="over the press's end"), Move(1.2, pos=behind, label="down behind it")]}
        pushed = 0.0
        while pushed < 0.080:
            seen = self.look()
            now = seen["overhang"] if seen is not None else m.START_TIP_Y + self.dry["shift"] - m.BOARD_Y[1]
            gap = m.PRESSING_OVERHANG - now
            if gap <= 0.0005:
                break
            step = min(SLIDE_STEP, gap + 0.0005)
            pushed += step
            self.dry["shift"] += step
            yield {"right": [Move(0.25, pos=behind + towards * pushed, label="push the press")]}
        yield {"right": [Move(0.6, pos=behind + towards * (pushed - 0.010), label="off it"), Move(0.8, pos=behind + towards * (pushed - 0.010) + np.array([0.0, 0.0, UP]), label="up")]}

    def press_pose(self, asked: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """(grasp point, turn, push, touching point from the grasp point in the gripper's frame) for the closed
        fingertips on the upper handle's top face: the hand pointing straight down, the fingers closing across the
        handle; the fingertips' ends level, their edge towards the grip end on the face at PRESS_U (the face rises
        that way, so the rest of the ends stand clear of it). The push ``asked`` N, tilted half the handle's angle from
        straight down towards its grip end. (Pointing into the face, the wrist would go past its stop under the
        push.)"""
        m, down = self.m, self.kit.down
        _, rotation = self.lower()
        yaw = math.atan2(rotation[1, 0], rotation[0, 0]) + math.pi / 2  # the fingers close across the handle
        rot = down(yaw)
        seen = self.look()
        if seen is not None:
            position, turn = seen["upper"]
        else:
            position, turn = m.mk.joint_pose(m.mk.MECH.joints["hinge"], self.phi(), self.lower())
        along = turn @ np.array([0.0, -1.0, 0.0])  # along the handle, towards its grip end
        flat = np.array([along[0], along[1], 0.0]) / max(1e-9, math.hypot(along[0], along[1]))
        touch = position + turn @ np.array(m.uvw(PRESS_U, 0.0, HANDLE_TOP_W))
        grasp = touch - PAD_HALF * flat + np.array([0.0, 0.0, TIP])
        tilt = 0.5 * self.phi()
        push = asked * (math.sin(tilt) * flat - math.cos(tilt) * np.array([0.0, 0.0, 1.0]))
        return grasp, rot, push, rot.T @ (touch - grasp)

    def press(self):
        """Right hand, fingers closed: the fingertips set on the upper handle's top face 15 mm from its end and
        pressed until the handle is closed (see the module's text)."""
        Move = self.Move
        up = np.array([0.0, 0.0, 1.0])
        grasp, rot, _, at = self.press_pose(0.0)
        yield {"right": [Move(2.0, pos=grasp + 0.04 * up, rot=rot, grip=0.0, label="over the handle"), Move(1.2, pos=grasp + 0.002 * up, label="down to it"),
                         Move(0.4, pos=grasp, label="on it")]}  # fmt: skip
        asked, t = PUSH_FIRST, 0.0
        while self.phi() > math.radians(CLOSED_DEG) and t < 30.0:
            asked = min(PUSH_MOST, asked + PUSH_MORE)
            t += LOOK_S
            if self.see is None:
                self.dry["phi"] = max(0.0, self.dry["phi"] - math.radians(1.5))
            grasp, rot, push, at = self.press_pose(asked)
            yield {"right": [Move(LOOK_S, pos=grasp, rot=rot, push=push, push_at=at, label="press")]}
        grasp, rot, push, at = self.press_pose(asked)
        yield {"right": [Move(1.0, pos=grasp, rot=rot, push=push, push_at=at, label="press home"), Move(0.5, push=(0.0, 0.0, 0.0), label="ease"),
                         Move(0.8, pos=grasp + 0.04 * up, label="off the handle")]}  # fmt: skip
        self.dry["phi"] = 0.0

    def lift(self):
        """Right hand: a pinch across both handles of the closed press 15 mm from their ends, and up 6 cm."""
        Move, down = self.Move, self.kit.down
        _, rotation = self.lower()
        yaw = math.atan2(rotation[1, 0], rotation[0, 0]) + math.pi / 2  # the fingers close across the handles
        grasp = self.on_lower(PRESS_U, 0.0, LIFT_W)
        wide = (HANDLE_WIDTH + 0.024) / OPENING
        yield {"right": [Move(1.5, pos=grasp + np.array([0.0, 0.0, UP]), rot=down(yaw), grip=wide, label="over the handles"), Move(1.2, pos=grasp, label="down beside them"),
                         Move(0.6, grip=0.0, label="close on both"), Move(1.5, pos=grasp + np.array([0.0, 0.0, LIFT]), label="lift the press")]}  # fmt: skip
        self.dry["lift"] = LIFT

    def scrape(self, part: float = 1.0):
        """Left hand: the knife turned to point at the press, its blade flat under the sieve, run across it."""
        m, Move = self.m, self.Move
        self.grip_knife()
        sieve = self.on_lower((m.CHAMBER_U[0] + m.CHAMBER_U[1]) / 2, 0.0, 0.0)
        _, rotation = self.lower()
        along = rotation @ np.array([0.0, -1.0, 0.0])  # the press's line, from its tip to its grip end (horizontal)
        point = math.atan2(along[1], along[0])  # the knife points the same way: its blade under the press from the tip's side
        knife_rot = self.kit.about((0.0, 0.0, 1.0), point)
        across = knife_rot @ np.array([0.0, 1.0, 0.0])  # the blade's width
        blade_mid = (m.BLADE_X[0] + m.BLADE_X[1]) / 2
        place = sieve - knife_rot @ np.array([blade_mid, 0.0, m.BLADE_Z[1]])
        start, end = place - 0.030 * across, place + (-0.030 + 0.060 * part) * across
        hand0, rot0 = self.hand_for_knife(start - np.array([0.0, 0.0, 0.015]), knife_rot)
        hand1, _ = self.hand_for_knife(start + np.array([0.0, 0.0, 0.001]), knife_rot)
        hand2, _ = self.hand_for_knife(end + np.array([0.0, 0.0, 0.001]), knife_rot)
        at = np.array([blade_mid, 0.0, m.BLADE_Z[1]])  # where it pushes: the blade's top face, in the hand's frame as held
        push_at = self.held_knife[1] @ at + self.held_knife[0]
        yield {"left": [Move(2.5, pos=hand0 + np.array([0.0, 0.0, -0.02]), rot=rot0, label="under the press"), Move(1.0, pos=hand0, label="under the sieve"),
                        Move(0.8, pos=hand1, push=(0.0, 0.0, SCRAPE_UP_N), push_at=push_at, label="against it"),
                        Move(3.0 * part, pos=hand2, label="across"), Move(0.4, push=(0.0, 0.0, 0.0), label="ease"),
                        Move(0.8, pos=hand2 - np.array([0.0, 0.0, 0.025]), label="down off it")]}  # fmt: skip
        self.dry["knife"] = (end, knife_rot)

    def tap(self):
        """Left hand: the blade's tip set on the saucer's well, tilted 15 degrees down."""
        m, Move = self.m, self.Move
        _, knife_rot = self.knife()
        tilt = knife_rot @ self.kit.about((0.0, 1.0, 0.0), math.radians(15.0))
        well = self.saucer() + np.array([0.0, 0.0, m.WELL_W])
        tip = np.array([m.BLADE_X[1] - 0.012, 0.0, m.BLADE_Z[0]])
        place = well - tilt @ tip
        hand_up, rot = self.hand_for_knife(place + np.array([0.0, 0.0, 0.03]), tilt)
        hand_on, _ = self.hand_for_knife(place + np.array([0.0, 0.0, 0.0005]), tilt)
        yield {"left": [Move(2.0, pos=hand_up, rot=rot, label="over the saucer"), Move(1.0, pos=hand_on, label="tip on the saucer"), Move(0.6, label="on it"),
                        Move(0.8, pos=hand_up, label="up")]}  # fmt: skip

    def lay_knife(self):
        m, Move = self.m, self.Move
        place = np.array([*m.KNIFE_AT, m.TOP + m.KNIFE_HANDLE[2] / 2 + 0.002])
        hand, rot = self.hand_for_knife(place, np.eye(3))
        yield {"left": [Move(2.5, pos=hand + np.array([0.0, 0.0, 0.04]), rot=rot, label="over its place"), Move(1.2, pos=hand, label="down"), Move(0.6, grip=0.45, label="let it go"),
                        Move(1.0, pos=hand + np.array([0.0, 0.0, 0.08]), label="up"), Move(1.5, pos=np.array([0.20, 0.25, m.TOP + 0.16]), label="away")]}  # fmt: skip

    def set_down(self, stay: bool = False):
        """Right hand: the closed press set down where it was pressed; let go and away (or the hand stays)."""
        m, Move = self.m, self.Move
        grasp = self.on_lower(PRESS_U, 0.0, LIFT_W)
        down_at = grasp - np.array([0.0, 0.0, LIFT - 0.001])
        self.dry["lift"] = 0.0
        moves = [Move(1.5, pos=down_at, label="set it down"), Move(0.4, label="down")]
        if stay:
            moves.append(Move(3.0, label="hand stays"))
        else:
            moves += [Move(0.6, grip=0.6, label="let go"), Move(1.0, pos=down_at + np.array([0.0, 0.0, 0.08]), label="up"), Move(1.5, pos=np.array([0.20, -0.25, m.TOP + 0.16]), label="away")]
        yield {"right": moves}

    def steps(self, name: str):
        yield from self.pick_clove()
        yield from self.drop_clove(across=name == "across")
        if name in ("pressed", "in_chamber"):
            yield from self.handle_over()
        else:
            yield self.together(self.handle_over(), self.fetch_knife())  # the left hand fetches the knife meanwhile
        if name == "in_chamber":
            yield {"left": [self.Move(1.5, pos=np.array([0.20, 0.25, self.m.TOP + 0.16]), label="away")]}
            return
        yield from self.slide()
        yield from self.press()
        yield from self.lift()
        if name != "pressed":
            yield from self.scrape(part=HALF_PART if name == "half" else 1.0)
            yield from self.tap()
            yield from self.lay_knife()
        yield from self.set_down(stay=name == "held")

    def whole(self, name: str) -> dict:
        """The plan as one list of moves per arm (dry run): each step's arms kept in step with each other."""
        out = {"left": [], "right": []}
        for step in self.steps(name):
            longest = max(sum(move.seconds for move in moves) for moves in step.values())
            for side in ("left", "right"):
                moves = list(step.get(side, []))
                rest = longest - sum(move.seconds for move in moves)
                if rest > 1e-9:
                    moves.append(self.Move(rest, label="waits"))
                out[side].extend(moves)
        return out


def main() -> int:
    import demo_kit as kit

    quick = "--quick" in sys.argv
    args = [a for a in sys.argv[1:] if a != "--quick"]
    out = Path(args[0])
    runs = args[1].split(",") if len(args) > 1 else ["full"]
    if any(run not in RUNS for run in runs):
        raise SystemExit(f"a run is one of {sorted(RUNS)}; got {runs}")
    if quick:
        from yam_arena import station, task_file

        m = task_file.load(str(HERE / "task.py")).module
        for run in runs:
            m.begin(0)
            print(f"DRY {run}: {json.dumps(kit.dry_run(Plan(m).whole(run), station.BASES, limit_s=m.RUN_LIMIT_STEPS / 20.0), default=str)}", flush=True)
        return 0
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    failed = []
    for run in runs:
        try:

            def hand_turn(side: str) -> np.ndarray:
                """The hand's turn as it stands: its gripper link's (the grasp point's frame), read as the kit reads the grasp point."""
                robot = mk.MECH.environment.unwrapped.scene[f"{side}_robot"]
                link = robot.data.body_link_pose_w.torch[0, robot.find_bodies("gripper")[0][0]].detach().cpu().numpy().astype(float)
                return mk.quat_to_matrix(link[3:7])

            def see() -> dict:
                hands = {side: (mk.gripper(side)[0], hand_turn(side)) for side in ("left", "right")}
                return {"press": mk.pose("press"), "upper": mk.pose("upper"), "hinge": mk.joint("hinge"), "clove": mk.pose(m.STATE["clove"]), "knife": mk.pose("knife"),
                        "saucer": mk.pose("saucer"), "overhang": m.overhang(), "hands": hands}  # fmt: skip

            plan = Plan(m, see)
            script = plan.steps(run)
            state = {"over": False}

            def director(t: float, arms: dict, pictures, script=script, state=state, plan=plan) -> None:
                if pictures is None or mk.MECH.environment is None or not mk.MECH.poses:
                    return
                while not state["over"] and t >= max(arm.seconds for arm in arms.values()) - 1e-6:  # the next step is planned when both arms are through
                    step = next(script, None)
                    if step is None:
                        state["over"] = True
                        break
                    longest = max(sum(move.seconds for move in moves) for moves in step.values())
                    for side in ("left", "right"):
                        moves = list(step.get(side, []))
                        rest = longest - sum(move.seconds for move in moves)
                        if rest > 1e-9:
                            moves.append(plan.Move(rest, label="waits"))
                        arms[side].moves.extend(moves)

            def extra() -> dict:
                a = m.amounts()
                return {"hinge (deg)": round(math.degrees(mk.joint("hinge")), 2), "face (mm)": round(1000 * m.face_height(mk.joint("hinge")), 2), "amounts (g)": a,
                        "overhang (mm)": round(1000 * m.overhang(), 1), "touched": m.STATE["touched"], "level": m.level(), "at rest": m.at_rest(),
                        "hands": {side: [round(float(v), 4) for v in mk.gripper(side)[0]] for side in ("left", "right")},
                        "clove (m)": [round(float(v), 4) for v in mk.pose(m.STATE["clove"])[0]]}  # fmt: skip

            folder = out / run
            result = kit.run(embodiment, {"left": [], "right": []}, folder, seed=0, director=director, extra=extra, limit_s=m.RUN_LIMIT_STEPS / hz, tail_s=1.5, sheet_frames=16)
            rows = result["rows"]
            scores = [row["info"].get("score", 0.0) for row in rows]
            firsts = {level: next((row["t"] for row, s in zip(rows, scores) if s >= level), None) for level in (0.25, 0.5, 0.75, 1.0)}
            readings = {"score": scores[-1], "first reached": firsts, "seconds": rows[-1]["t"], "ended": result.get("termination_reason"), "report": result["report"], "worst IK": result["worst_ik"],
                        "events": m.STATE["events"]}  # fmt: skip
            (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
            print(f"DEMO {run}: score {scores[-1]:.2f} after {result['seconds']:.1f} s ({result['steps']} steps of {m.RUN_LIMIT_STEPS}); first reached {firsts}; report {json.dumps(result['report'], default=str)}", flush=True)
        except Exception as error:
            import traceback

            traceback.print_exc()
            print(f"RUN {run} FAILED: {error!r}", flush=True)
            failed.append(run)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
