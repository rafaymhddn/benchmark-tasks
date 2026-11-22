#!/usr/bin/env python3
"""Scripted reference demonstration for ``comb_busbar`` (arm actions only).

    python demo.py <output folder> [--check] [--snug | --bare]   # in the simulator container
    python demo.py <output folder> --start                       # start pictures and 3 s idle (G1)
    python3 demo.py --quick                                      # the whole plan through the URDF IK, no simulator

1. The left hand takes the comb off its rack by its two ends, pushes its fourth tooth into the cap until the cap's
   catch takes it (22 mm), carries the comb high over the breakers, brings the other three teeth 6 mm down into the
   three breakers' slots and lets go: the comb slides home by its weight; its closed fingers then press the bar's top
   lightly. Each way down (into the cap, into the slots) goes in stages, aimed again before each
   from the live poses: the arm's soft joints let the hand drift sideways as it moves.
2. It takes the driver out of its holster by the T-handle (pinched across its 22 mm, 13 mm off its centre).
3. Each screw: the bit's tip into the access hole (slowly; 3 N along the screw), and the left wrist turns the driver
   178 deg about the screw's axis: the clamp plate comes to 2 deg short of the tooth (0.1 N m). Out, turned back in
   the air to roll 140 deg, in again; then the right hand presses the T-handle's lower arm 36 mm from the axis with its
   closed fingertips (30 N: 1.1 N m), re-aimed on the arm's face from its live pose at every step, the left hand
   pushing back through its pads (30 N, 13 mm on the other side: no net force on the driver) while its wrist turns on
   up to 22 deg: the clamp closes on the tooth and the limiter clicks at 2.0 N m, each hand giving about half; both
   stop at the click. The right hand lets go; the bit comes out. Every hand pose for the driver is made from the driver's live
   pose in the hand.
4. The left hand keeps the driver, drawn back from the panel; both hands stay still.

``--check``: no videos. Live reads: the cap and the slots are aimed at from the teeth's live tips, each hole from the
bit's live tip, each turn is made about the screw's axis from the driver's live pose in the hand, and the right hand's
push from the T-handle's live pose. ``--snug``: the left hand alone turns the last 8 deg (closed on the tooth, no
click: 0.5). ``--bare``: no cap (0.5).
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as C  # noqa: E402
import demo_kit as kit  # noqa: E402
import parts as P  # noqa: E402

SIDES = ("left", "right")
MAX_JAW = 0.094
BIT_PUSH_N = 3.0  # N: the bit pressed into the hole along the screw while it turns
FIX_MAX = 0.015  # m: largest live correction of an aim
CAP_GOALS = (-0.003, 0.010, 0.019)  # m: the spare tooth's tip 3 mm over the cap, then 10 and 19 mm into it ...
CAP_STEP = 0.0005  # m: ... then on in these steps until its catch takes the tooth (22 mm; at most to checks.CAP_IN, 24.5 mm)
SLOW = 0.008  # m/s: the comb's mean speed into the cap
SLOT_GOALS = (-C.OVER_SLOTS, -C.OVER_SLOTS, -0.0010, 0.0005, 0.0030, C.TEETH_IN)  # m: the tips' depth under the tops at each step (aimed
# twice over them); let go only 6 mm in: just inside the mouths the comb tips over backwards (run cb2f)
END_BACK, END_UP = 0.040, 0.020  # m: the driver drawn back from the last screw and lifted, where the left hand keeps it
FIN_STEPS = 8  # the last turn in at most this many steps of checks.FIN_HAND_DEG / 8 (2.75 deg), each from the driver's live pose
# (short steps: both hands stop within 0.3 s of the click)
UP = np.array([0.0, 0.0, 1.0])


def grip(opening_m: float) -> float:
    return round(min(1.0, max(0.0, opening_m / MAX_JAW)), 3)


def Move_(seconds: float, pos, rot, label: str):  # noqa: N802 - a Move to a pose
    return kit.Move(seconds, pos=pos, rot=rot, label=label)


def rot_about(axis, angle: float) -> np.ndarray:
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


class Plan:
    """The demo as chunks of moves for both arms, made one by one (live in the simulator, or offline)."""

    def __init__(self, m, live: bool, variant: str | None = None) -> None:
        from yam_arena import ik, station
        from yam_arena.constants import READY

        self.m, self.mk, self.live, self.variant, self.Move = m, m.mk, live, variant, kit.Move
        self.pl = C.planned()
        flip = {tag: C.grasp_flip(self.pl, tag) for tag in ("L_", "D_")}  # one grasp for every pose of a part
        self.rot = {n: self.pl[n][1] @ flip[n[:2]] for n in self.pl}
        self.alpha0 = {i: C.sweep_start(i)[0] for i in range(3)}
        self.ready = {side: ik.tcp_pose(np.array(READY, float), station.BASES[side]) for side in SIDES}
        # the T-handle's roll in the hand at the holster, on the same turn as the wrist's joint (j6 follows it 1:1)
        q_h, *_ = C.solve("left", self.pl["D_holster"][0], self.rot["D_holster"])
        est = self.alpha0[0] + (float(q_h[5]) - math.radians(C.J6_START_DEG))
        a_h = self._alpha_of(self.rot["D_holster"])
        self.alpha_pick = a_h + 2.0 * math.pi * round((est - a_h) / (2.0 * math.pi))
        self.alpha = self.alpha_pick
        dr = P.driver_rot(C.FIN_ALPHA)
        contact, _, _ = C.push_contact(C.driver_at_screw(0), dr)
        C.solve("right", C.push_tcp(contact, C.PUSH_ROT), C.PUSH_ROT, flips=True)
        self.push_rot = C.SOLVED_ROT[0]  # the right hand's turn for its push, as checks.py found it reachable
        self.log: dict = {"aims": [], "screws": [], "cap": None, "alpha_pick_deg": round(math.degrees(self.alpha_pick), 1),
                          "alpha0_deg": [round(math.degrees(a), 1) for a in self.alpha0.values()]}

    @staticmethod
    def _alpha_of(rot) -> float:
        c = rot[:, 0]
        return math.atan2(float(np.dot(c, P.ZP)), float(np.dot(c, P.YP)))

    def pose(self, name: str):
        tcp, _, op = self.pl[name]
        return np.asarray(tcp, float), self.rot[name], op

    def both(self, left: list, right: list) -> dict:
        tl, tr = sum(mv.seconds for mv in left), sum(mv.seconds for mv in right)
        if tl < tr - 1e-9:
            left = [*left, self.Move(tr - tl, label="wait")]
        elif tr < tl - 1e-9:
            right = [*right, self.Move(tl - tr, label="wait")]
        return {"left": left, "right": right}

    def tip(self, k: int) -> np.ndarray:
        return self.m.tooth_tip(k)

    def fix(self, err, label: str) -> np.ndarray:
        err = np.asarray(err, float)
        n = float(np.linalg.norm(err))
        self.log["aims"].append({"at": label, "err_mm": [round(1e3 * float(v), 2) for v in err]})
        return err if n <= FIX_MAX else err * (FIX_MAX / n)

    def hand_for(self, dp, dr):
        """The left gripper's pose that puts the driver's centre at ``dp`` turned ``dr``, from the driver's live pose in
        the hand (offline: as planned): a driver that has shifted or tilted in the pads is still put where it should be."""
        hp, hr = self.driver_in_hand()
        g_rot = np.asarray(dr, float) @ hr.T
        return np.asarray(dp, float) - g_rot @ hp, g_rot

    def sync_alpha(self, expected: float) -> None:
        """The driver's roll as it now stands (offline: ``expected``), on the same turn as ``expected``."""
        if not self.live:
            self.alpha = expected
            return
        dr = self.mk.pose("driver")[1]
        r = math.atan2(float(np.dot(dr[:, 1], P.ZP)), float(np.dot(dr[:, 1], P.YP)))
        self.alpha = r + 2.0 * math.pi * round((expected - r) / (2.0 * math.pi))

    def roll_moves(self, to_alpha: float, centre, label: str, step_deg: float = 55.0, s_per_step: float = 0.5) -> list:
        """Turn the held driver about its own axis (along the screws') to roll ``to_alpha`` in steps, its centre staying
        at ``centre``."""
        n = max(1, int(math.ceil(abs(math.degrees(to_alpha - self.alpha)) / step_deg)))
        out, a0 = [], self.alpha
        for k in range(1, n + 1):
            g, gr = self.hand_for(centre, P.driver_rot(a0 + (to_alpha - a0) * k / n))
            out.append(self.Move(s_per_step, pos=g, rot=gr, label=f"{label}_{k}"))
        self.alpha = to_alpha
        return out

    def driver_in_hand(self):
        """(position, rotation) of the driver in the left gripper's frame (offline: as planned)."""
        if not self.live:
            h = C.drv_rot(0.0).T @ P.driver_rot(0.0)
            return -h @ np.asarray(P.GRASP_OFF), h
        g, gr = self.mk.gripper("left")[0], self.mk.gripper_turn("left")
        dp, dr = self.mk.pose("driver")
        return gr.T @ (dp - g), gr.T @ dr

    def driver_now(self, i: int):
        if self.live:
            return self.mk.pose("driver")
        hp, hr = self.driver_in_hand()
        rot = C.drv_rot(self.alpha)
        return C.in_tcp(i, self.alpha) + rot @ hp, rot @ hr

    def turn_poses(self, i: int, total_deg: float, steps: int):
        """Left gripper poses that turn the driver about screw ``i``'s axis through its head, from the driver's live
        pose in the hand: (tcp, rot) per step."""
        hp, hr = self.driver_in_hand()
        head = P.pw(P.screw_pos(i)) + self.m.TOP * UP
        dp, dr = self.driver_now(i)
        out = []
        for k in range(1, steps + 1):
            turn = rot_about(P.XP, math.radians(total_deg) * k / steps)
            g_rot = turn @ dr @ hr.T
            out.append((head + turn @ (dp - head) - g_rot @ hp, g_rot))
        return out

    def hole_err(self, i: int) -> np.ndarray:
        """The bit's live tip from hole ``i``'s axis, across the screw (m)."""
        dp, dr = self.mk.pose("driver")
        err = P.pw(P.screw_pos(i)) + self.m.TOP * UP - (dp + dr @ np.array([P.TIP_X, 0.0, 0.0]))
        return err - np.dot(err, P.XP) * P.XP

    def cap_err(self) -> tuple[np.ndarray, float]:
        """(the cap's top centre from the spare tooth's live tip, across the cap's axis (m); how far the tip is into
        the cap (m, negative over it))."""
        cp, cr = self.mk.pose("cap")
        err = cp + 0.5 * P.CAP_SIZE[2] * cr[:, 2] - self.tip(3)
        depth = float(np.dot(err, cr[:, 2]))
        return err - depth * cr[:, 2], depth

    def slot_err(self) -> tuple[np.ndarray, float]:
        """(the three teeth's mean offset from their slots' middles, across the slots (world, m); the tips' mean depth
        under the breakers' tops (m, negative over them)). Teeth 1-3 go into breakers 3-1."""
        errs, depths = [], []
        for slot, k in enumerate((2, 1, 0)):
            t = P.to_panel(self.tip(k) - self.m.TOP * UP)
            errs.append([P.X_PIN - t[0], P.MCB_Y[slot] - t[1]])
            depths.append(P.MCB_TOP - t[2])
        ex, ey = np.mean(errs, axis=0)
        return ex * P.XP + ey * P.YP, float(np.mean(depths))

    def engage(self, i: int, alpha: float, tag: str):
        """Turn the held driver in the air to ``alpha`` in front of hole ``i``, aim the bit's tip at the hole's axis at
        rest (twice with the tip 6.5 mm out, once more 2 mm out), put it in slowly (the arm lags sideways when it
        moves fast) and press it on along the screw. The hand's poses come from the driver's live pose in the hand."""
        base, rot_d = C.driver_at_screw(i), P.driver_rot(alpha)  # the driver's centre with the tip on the hole's floor
        front_d, near_d, in_d = base - C.FRONT * P.XP, base - C.NEAR * P.XP, base + 0.001 * P.XP
        yield self.both([*self.roll_moves(alpha, front_d, f"roll_{tag}_{i + 1}"), self.Move(0.4, label="settle_front")], [])
        fix = np.zeros(3)
        for j, at_d in enumerate((front_d, near_d, near_d) if self.live else (near_d,)):
            if self.live:
                fix = fix + self.fix(self.hole_err(i), f"hole_{tag}_{i + 1}_{j + 1}")
            g, gr = self.hand_for(at_d, rot_d)
            yield self.both([Move_(0.6, g + fix, gr, f"aim_{tag}_{i + 1}_{j + 1}"), self.Move(0.3, label="settle_aim")], [])
        g, gr = self.hand_for(in_d, rot_d)
        tip = in_d + rot_d @ np.array([P.TIP_X, 0.0, 0.0])
        yield self.both([
            Move_(2.0, g + fix, gr, f"into_hole_{tag}_{i + 1}"),
            self.Move(0.4, push=tuple(BIT_PUSH_N * P.XP), push_at=tuple(gr.T @ (tip - g)), label=f"press_bit_{tag}_{i + 1}"),
        ], [])  # fmt: skip
        if self.live:
            self.log.setdefault("engaged", []).append({"screw": i + 1, "stroke": tag, "engaged": self.m.STATE["engaged"]})

    def withdraw(self, last, tag: str) -> list:
        """Ease the bit's press and take it out of the hole along the screw."""
        out = last[0] - (P.HOLE_FLOOR + 0.006) * P.XP
        self.out_pose = (out, last[1])
        return [self.Move(0.3, push=(0.0, 0.0, 0.0), label=f"ease_bit_{tag}"), Move_(0.7, out, last[1], f"out_{tag}")]

    def push_target(self, i: int, lead_deg: float = 0.0, into: float = 0.0005):
        """The right hand's pose with its closed fingertips' end ``into`` the face of the T-handle's lower arm, from the
        driver's live pose led ``lead_deg`` round the screw's axis: (tcp, rot, way, push_at)."""
        dp, dr = self.driver_now(i)
        contact, _, way = C.push_contact(dp, dr)
        roll = math.atan2(float(np.dot(dr[:, 1], P.ZP)), float(np.dot(dr[:, 1], P.YP)))  # the driver's roll now
        turned = (roll - C.FIN_ALPHA + math.pi) % (2.0 * math.pi) - math.pi
        lead = min(math.radians(lead_deg), max(0.0, math.radians(C.FIN_DRV_DEG) - turned))  # never past the turn checks.py checked
        r = rot_about(P.XP, lead)
        head = P.pw(P.screw_pos(i)) + self.m.TOP * UP
        rot = r @ rot_about(P.XP, min(turned, math.radians(C.FIN_DRV_DEG))) @ self.push_rot
        contact, way = head + r @ (contact - head), r @ way
        return C.push_tcp(contact + into * way, rot), rot, way, tuple(np.array([0.0, 0.0, C.PUSH_END]))

    def push_gap(self, i: int) -> tuple[float, float]:
        """(how far the right fingertips' end stands off the lower arm's face (m, negative: pressed in); how far it is
        from the arm's end along the arm (m))."""
        dp, dr = self.mk.pose("driver")
        contact, n, _ = C.push_contact(dp, dr)
        g, gr = self.mk.gripper("right")[0], self.mk.gripper_turn("right")
        tip = g + C.PUSH_END * gr[:, 2]
        span = dr[:, 2] if dr[2, 2] < 0.0 else -dr[:, 2]
        return float(np.dot(tip - contact, n)), float(0.5 * P.TB_SPAN - np.dot(tip - dp, span))

    def steps(self):
        Move, lo = self.Move, self.pose
        sq = grip(C.COMB_SQUEEZE)
        # 1. the comb: off its rack, the fourth tooth into the cap, the other three into the slots
        a, b, c = lo("L_over_rack"), lo("L_rack"), lo("L_lift")
        yield self.both([
            Move(1.8, pos=a[0], rot=a[1], grip=grip(a[2]), label="over_rack"),
            Move(1.0, pos=b[0], rot=b[1], grip=grip(a[2]), label="at_comb"),
            Move(0.6, grip=sq, label="pinch_comb"),
            Move(0.9, pos=c[0], rot=c[1], grip=sq, label="lift_comb"),
        ], [])  # fmt: skip
        if self.variant != "bare":
            o, i_, u = lo("L_over_cap"), lo("L_cap"), lo("L_capped")
            yield self.both([Move(1.5, pos=o[0], rot=o[1], grip=sq, label="over_cap"), Move(0.4, label="settle_cap")], [])
            fix = np.zeros(3)
            for k in range(2 if self.live else 0):  # aimed at rest, twice: the comb sits in the hand where it was taken
                fix = fix + self.fix(self.cap_err()[0], f"cap_{k + 1}")
                yield self.both([Move(0.6, pos=o[0] + fix, rot=o[1], grip=sq, label=f"aim_cap_{k + 1}"), Move(0.3, label="settle_aim")], [])
            if self.live:  # down the cap's live axis in stages, aimed across it before each (the arm drifts as it moves) ...
                at = o[0] + fix
                for k, goal in enumerate(CAP_GOALS):
                    lat, depth = self.cap_err()
                    at = at + self.fix(lat, f"cap_in_{k + 1}") - (goal - depth) * self.mk.pose("cap")[1][:, 2]
                    yield self.both([Move(max(0.5, abs(goal - depth) / SLOW), pos=at, rot=o[1], grip=sq, label=f"into_cap_{k + 1}"),
                                     Move(0.3, label="settle_in_cap")], [])  # fmt: skip
                for k in range(int(round((C.CAP_IN - CAP_GOALS[-1]) / CAP_STEP))):  # ... then on in steps until its catch takes the tooth
                    if self.m.STATE["cap_on"] is not None:
                        break
                    at = at - CAP_STEP * self.mk.pose("cap")[1][:, 2]
                    yield self.both([Move(0.25, pos=at, rot=o[1], grip=sq, label=f"deeper_cap_{k + 1}"), Move(0.15, label="settle_step")], [])
                lat, depth = self.cap_err()
                self.log["cap"] = {"on": self.m.STATE["cap_on"], "depth_mm": round(1e3 * depth, 2), "across_mm": round(1e3 * float(np.linalg.norm(lat)), 2)}
                lift = at + (u[0] - i_[0])
            else:
                yield self.both([Move(1.6, pos=i_[0], rot=i_[1], grip=sq, label="into_cap")], [])
                lift = u[0]
            yield self.both([Move(0.8, pos=lift, rot=u[1], grip=sq, label="lift_capped")], [])
        hc, hs = lo("L_high_cap"), lo("L_high_slots")  # over the breakers' tops with the teeth (and the cap) clear of them
        o, t_in, lg, pr = lo("L_over_slots"), lo("L_teeth_in"), lo("L_let_go"), lo("L_press")
        yield self.both([Move(0.9, pos=hc[0], rot=hc[1], grip=sq, label="up_high"), Move(1.4, pos=hs[0], rot=hs[1], grip=sq, label="over_breakers"),
                         Move(0.8, pos=o[0], rot=o[1], grip=sq, label="over_slots"), Move(0.4, label="settle_slots")], [])  # fmt: skip
        at = np.asarray(t_in[0], float)
        if self.live:  # the three tips aimed at their slots, then down into the mouths in steps, aimed again before each
            at = np.asarray(o[0], float)
            for k, goal in enumerate(SLOT_GOALS):
                across, depth = self.slot_err()
                at = at + self.fix(across, f"slots_{k + 1}") - (goal - depth) * P.ZP
                yield self.both([Move(0.6, pos=at, rot=o[1], grip=sq, label=f"slots_{k + 1}"), Move(0.3, label="settle_slots")], [])
            across, depth = self.slot_err()
            self.log["slots"] = {"across_mm": [round(1e3 * float(v), 2) for v in (np.dot(across, P.XP), np.dot(across, P.YP))], "depth_mm": round(1e3 * depth, 2)}
        else:
            yield self.both([Move(1.0, pos=at, rot=t_in[1], grip=sq, label="teeth_in")], [])
        yield self.both([
            Move(0.5, grip=grip(lg[2]), label="let_go_comb"),
            Move(0.6, pos=at + 0.012 * UP, rot=t_in[1], label="up_from_comb"),
            Move(0.4, grip=0.0, label="close_fingers"),
            Move(0.8, pos=pr[0] + 0.004 * UP, rot=pr[1], grip=0.0, label="over_bar"),
            Move(0.5, pos=pr[0], rot=pr[1], label="on_bar"),
            Move(0.6, push=(0.0, 0.0, -2.0), push_at=kit.lowest_point(pr[1]), label="press_bar"),
            Move(0.3, push=(0.0, 0.0, 0.0), label="ease_bar"),
            Move(0.7, pos=pr[0] + np.array([-0.06, 0.04, 0.02]), rot=pr[1], label="back_from_bar"),
        ], [])  # fmt: skip
        # 2. the driver out of its holster
        d, e, f, g = lo("D_behind_holster"), lo("D_holster"), lo("D_pulled"), lo("D_lifted")
        yield self.both([
            Move(2.0, pos=d[0], rot=d[1], grip=grip(d[2]), label="behind_holster"),
            Move(1.0, pos=e[0], rot=e[1], grip=grip(d[2]), label="at_t_handle"),
            Move(0.6, grip=grip(C.DRV_SQUEEZE), label="pinch_t_handle"),
            Move(0.9, pos=f[0], rot=f[1], grip=grip(C.DRV_SQUEEZE), label="pull_driver"),
            Move(0.6, pos=g[0], rot=g[1], grip=grip(C.DRV_SQUEEZE), label="lift_driver"),
        ], [])  # fmt: skip
        # 3. the screws
        snug = self.variant == "snug"
        for i in range(3):
            g, gr = self.hand_for(C.driver_at_screw(i) - C.FRONT * P.XP, P.driver_rot(self.alpha))
            yield self.both([Move(1.6 if i == 0 else 1.0, pos=g, rot=gr, label=f"to_screw{i + 1}")], [])
            yield from self.engage(i, self.alpha0[i], "run")
            run = self.turn_poses(i, C.RUN_DEG, 5)
            yield self.both([Move(0.9, pos=p_, rot=r_, label=f"run_{i + 1}_{k + 1}") for k, (p_, r_) in enumerate(run)] + [Move(0.3, label="settle_run")], [])
            self.sync_alpha(self.alpha0[i] + math.radians(C.RUN_DEG))
            rec = {"screw": i + 1, "after_run_deg": round(math.degrees(self.m.STATE["turn"][i]), 1) if self.live else None}
            yield self.both(self.withdraw(run[-1], f"{i + 1}a"), [])
            yield from self.engage(i, C.FIN_ALPHA, "fin")
            fin = self.turn_poses(i, C.SNUG_HAND_DEG if snug else C.FIN_HAND_DEG / FIN_STEPS, 4 if snug else 1)
            if snug:  # the left hand alone, closed on the tooth short of the click
                yield self.both([Move(0.9, pos=p_, rot=r_, label=f"snug_{i + 1}_{k + 1}") for k, (p_, r_) in enumerate(fin)] + [Move(0.3, label="settle_fin")], [])
            else:
                tcp, rot, way, at = self.push_target(i, into=-0.001)
                yield self.both([], [Move(1.6, pos=tcp - 0.03 * rot[:, 2], rot=rot, grip=0.0, label=f"over_arm_{i + 1}"),
                                     Move(0.6, pos=tcp, rot=rot, label=f"near_arm_{i + 1}"), Move(0.2, label="settle_arm")])  # fmt: skip
                if self.live:  # the fingertips' end put on the face from the live poses (the right arm drifts too)
                    gap, _ = self.push_gap(i)
                    self.log.setdefault("push", []).append({"screw": i + 1, "gap_mm": round(1e3 * gap, 2)})
                    tcp = tcp + (gap + 0.0005) * way
                origin = (0.0, 0.0, 0.0)  # the left hand pushes back through its pads: the two pushes are a couple on the driver
                press = BIT_PUSH_N * P.XP  # and keeps the bit pressed into the screw
                yield self.both([self.Move(0.4, label="wait"), self.Move(0.5, push=tuple(press - C.PUSH_N * way), push_at=origin, label=f"counter_{i + 1}")],
                                [Move(0.4, pos=tcp, rot=rot, label=f"on_arm_{i + 1}"),
                                 Move(0.5, push=tuple(C.PUSH_N * way), push_at=at, label=f"push_arm_{i + 1}")])  # fmt: skip
                step = C.FIN_HAND_DEG / FIN_STEPS
                left_now = fin[0]
                for k in range(FIN_STEPS):  # each step from where the driver now stands, the right hand re-aimed on its face
                    if self.live and self.m.STATE["clicked"][i]:  # it clicked: both hands stop
                        break
                    left_now = self.turn_poses(i, step, 1)[0]
                    if not self.live:
                        self.alpha += math.radians(step)
                    tcp, rot, way, at = self.push_target(i, lead_deg=0.5 * step)
                    yield self.both([Move(0.3, pos=left_now[0], rot=left_now[1], push=tuple(press - C.PUSH_N * way), push_at=origin, label=f"fin_{i + 1}_{k + 1}")],
                                    [Move(0.3, pos=tcp, rot=rot, push=tuple(C.PUSH_N * way), push_at=at, label=f"push_follow_{i + 1}_{k + 1}")])  # fmt: skip
                if self.live:  # both hands told to stay where they stand as the pushes ease: no wound-up joint turns the driver back
                    left_now = (self.mk.gripper("left")[0].copy(), self.mk.gripper_turn("left").copy())
                    right_now = (self.mk.gripper("right")[0].copy(), self.mk.gripper_turn("right").copy())
                else:
                    right_now = (tcp, rot)
                fin = [left_now]
                yield self.both([Move(0.3, label="settle_fin"), Move(0.5, pos=left_now[0], rot=left_now[1], push=(0.0, 0.0, 0.0), label="ease_counter")],
                                [Move(0.3, label="hold_push"), Move(0.5, pos=right_now[0], rot=right_now[1], push=(0.0, 0.0, 0.0), label="ease_arm"),
                                 Move(0.3, pos=right_now[0] - 0.002 * right_now[1][:, 2], rot=right_now[1], label="off_face")])  # fmt: skip
                tcp, rot = right_now
                off = tcp - 0.012 * rot[:, 2]
                after = [Move(0.6, pos=off, rot=rot, label="off_arm"), Move(0.9, pos=off + np.array([0.04, -0.03, 0.03]), rot=rot, label="right_aside")]
            self.sync_alpha(C.FIN_ALPHA + math.radians(C.SNUG_HAND_DEG if snug else C.FIN_HAND_DEG))
            if self.live:
                st = self.m.STATE
                hp, hr = self.driver_in_hand()
                rec.update(turn_deg=round(math.degrees(st["turn"][i]), 1), clicked=st["clicked"][i], peak_torque=round(st["peak_torque"][i], 2),
                           gap_mm=round(1e3 * st["gap"][i], 3), driver_tilt_in_hand_deg=round(math.degrees(math.acos(min(1.0, float(hr[2, 0])))), 2))  # fmt: skip
            self.log["screws"].append(rec)
            yield self.both(self.withdraw(fin[-1], f"{i + 1}b"), [])  # the bit out of the hole first, then the right hand away
            if not snug:
                yield self.both([], after)
        # 4. the left hand keeps the driver, drawn back from the panel; both hands still
        out, rot = self.out_pose
        yield self.both([Move(1.0, pos=out - END_BACK * P.XP + END_UP * UP, rot=rot, label="driver_back")], [])
        yield self.both([Move(2.5, label="hold_done")], [Move(2.5, label="hold_done")])


def task_report(m) -> dict:
    st = m.STATE
    out = {"fault": st["fault"], "turn_deg": [round(math.degrees(t), 1) for t in st["turn"]], "clicked": st["clicked"],
           "gap_mm": [round(1e3 * g, 3) for g in st["gap"]], "cap_on": st["cap_on"], "peak_torque": [round(t, 2) for t in st["peak_torque"]],
           "events": [f"{t:.2f} s {what}" for t, what in st["events"]], "order": m.EVENTS.report()}  # fmt: skip
    if m.mk.MECH.poses:
        out.update(seated=m.seated(), spare=m.spare_tooth(), closed=[m.closed_on_tooth(i) for i in range(3)], rested=m.rested(),
                   grippers_clear=m.grippers_clear() if m.mk.MECH.grippers else None)  # fmt: skip
    return out


def run_quick() -> int:
    from yam_arena import station, task_file

    m = task_file.load(HERE / "task.py").module
    plan = Plan(m, live=False)
    moves: dict[str, list] = {"left": [], "right": []}
    for chunk in plan.steps():
        for side in SIDES:
            moves[side].extend(chunk[side])
    summary = kit.dry_run(moves, {s: station.BASES[s] for s in SIDES}, limit_s=200.0)
    print("QUICK " + json.dumps({"arms": summary, "log": plan.log}))
    for side, st in summary.items():
        assert st["worst_ik_mm"] <= 0.5 and st["smallest_margin_deg"] >= 5.0, (side, st)
    return 0


def run_start(out: Path) -> int:
    """G1: the start pictures, then 3 s with the arms at ready; the score and how far the loose bodies moved."""
    import cv2

    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    start = {}

    def director(t, _arms, pictures):
        if pictures is not None and not start and m.mk.MECH.poses:
            start.update({n: m.mk.pose(n)[0].copy() for n in m.LOOSE})
            cv2.imwrite(str(HERE / "start_top.png"), cv2.cvtColor(pictures["top_cam"], cv2.COLOR_RGB2BGR))
            cv2.imwrite(str(HERE / "start_top_policy.png"), cv2.cvtColor(kit.policy_picture(pictures["top_cam"]), cv2.COLOR_RGB2BGR))
            for cam in ("left_cam", "right_cam"):
                cv2.imwrite(str(out / f"start_{cam}.png"), cv2.cvtColor(pictures[cam], cv2.COLOR_RGB2BGR))

    res = kit.run(emb, {"left": [kit.Move(3.0, label="idle")], "right": [kit.Move(3.0, label="idle")]}, out / "start", seed=0,
                  director=director, tail_s=0.1, limit_s=4.0, record=False)  # fmt: skip
    drift = {n: round(1e3 * float(np.linalg.norm(m.mk.pose(n)[0] - start[n])), 2) for n in m.LOOSE} if start else None
    report = {"score": res["info"].get("score"), "drift_mm": drift, "task_report": task_report(m)}
    (out / "start_report.json").write_text(json.dumps(report, indent=1, default=str))
    print("START_REPORT " + json.dumps(report, default=str), flush=True)
    return 0


def main() -> int:
    if "--quick" in sys.argv[1:]:
        return run_quick()
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    out = Path(args[0]) if args else HERE / "demo_out"
    out.mkdir(parents=True, exist_ok=True)
    if "--start" in sys.argv[1:]:
        return run_start(out)
    variant = "snug" if "--snug" in sys.argv[1:] else ("bare" if "--bare" in sys.argv[1:] else None)
    record = "--check" not in sys.argv[1:]
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    plan = Plan(m, live=True, variant=variant)
    gen = plan.steps()
    queued = {"done": False, "next_at": 0.0}

    def director(t, arms, _pictures):
        if queued["done"] or t < queued["next_at"] - 1e-6 or not m.mk.MECH.poses:
            return
        try:
            chunk = next(gen)
        except StopIteration:
            queued["done"] = True
            return
        for side in SIDES:
            arms[side].moves.extend(chunk[side])
        queued["next_at"] = max(arms["left"].seconds, arms["right"].seconds)

    def extra():
        st = m.STATE
        cr = m.mk.pose("comb")[1]
        dp, dr = m.mk.pose("driver")
        tipp = P.to_panel(dp + dr @ np.array([P.TIP_X, 0.0, 0.0]) - np.array([0.0, 0.0, m.TOP]))
        combp = P.to_panel(m.mk.pose("comb")[0] - np.array([0.0, 0.0, m.TOP]))
        row = {"turn_deg": [round(math.degrees(v), 1) for v in st["turn"]], "clicked": st["clicked"], "engaged": st["engaged"],
               "tip_p_mm": [round(1e3 * float(v), 2) for v in tipp], "comb_p_mm": [round(1e3 * float(v), 2) for v in combp],
               "cap_on": st["cap_on"], "seated": m.seated() is not None, "hold": not m.mk.is_cut("comb_hold"),
               "comb_lean_err_deg": round(math.degrees(math.acos(min(1.0, float(np.dot(cr[:, 2], P.ZP))))), 2)}  # fmt: skip
        row["speeds"] = {n: [round(float(np.linalg.norm(v)), 4) for v in m.mk.velocity(n)] for n in m.LOOSE}  # m/s, rad/s
        if "right" in m.mk.MECH.grippers:
            gap, from_end = plan.push_gap(0)
            row.update(push_gap_mm=round(1e3 * gap, 2), push_from_arm_end_mm=round(1e3 * from_end, 2))
        if "left" in m.mk.MECH.grippers:
            g, gr = m.mk.gripper("left")[0], m.mk.gripper_turn("left")
            tip = dp + dr @ np.array([P.TIP_X, 0.0, 0.0])
            off = (tip - g) - np.dot(tip - g, gr[:, 2]) * gr[:, 2]
            row.update(drv_axis_vs_hand_deg=round(math.degrees(math.acos(min(1.0, float(np.dot(dr[:, 0], gr[:, 2]))))), 2),
                       tip_off_hand_axis_mm=round(1e3 * float(np.linalg.norm(off)), 2), hand_roll_vs_drv_deg=round(math.degrees(math.atan2(
                           float(np.dot(np.cross(gr[:, 0], dr[:, 1]), dr[:, 0])), float(np.dot(gr[:, 0], dr[:, 1])))), 1))  # fmt: skip
        return row

    res = kit.run(emb, {"left": [], "right": []}, out / "full", seed=0, director=director, extra=extra, tail_s=1.5,
                  limit_s=m.RUN_LIMIT_STEPS / 20.0, record=record)  # fmt: skip
    report = {"variant": variant, "score": res["info"].get("score"), "success": res["info"].get("success"), "terminated": res["terminated"],
              "steps": res["steps"], "seconds": res["seconds"], "worst_ik": res["worst_ik"], "log": plan.log, "task_report": task_report(m)}  # fmt: skip
    (out / "demo_report.json").write_text(json.dumps(report, indent=1, default=str))
    print("DEMO_REPORT " + json.dumps(report, default=str), flush=True)
    want = 0.5 if variant else 1.0
    return 0 if res["info"].get("score") == want else 1


if __name__ == "__main__":
    raise SystemExit(main())
