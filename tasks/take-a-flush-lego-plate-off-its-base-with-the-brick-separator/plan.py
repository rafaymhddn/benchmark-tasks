"""Scripted motions of both arms for ``demo.py`` and ``trials.py``, played by ``demo_kit.run``.

The right arm picks the separator by its handle (static moves). A director then adds every move that needs what the
simulator shows, and logs each reading it acts on (``Director.log``):
- seat: after the grasp it takes the socket's frame from the separator's own pose in the hand (not the gripper's),
  hovers 3 mm over the near row aligned to it (twice, the second time from the new reading), descends and presses;
  the offset at contact and at the seat go into the log;
- lever: it rocks the separator over its lower edge on the base's studs, 1.25 deg a step with a 3 N push at the grasp
  point, until the plate comes off (at most 36 deg), then lifts it up to 3 mm;
- hand-over: it lifts the separator with the plate to the hand-over pose and takes the plate's pose for the left
  hand's pinch across the plate's free end; the left hand pulls the plate along its studs out of the socket in 1 mm
  steps with a 6 N push, the right holding the separator against it, until the plate is out;
- place: the left hand turns to point down over the dish and drops the plate from 4 mm over the floor; the separator
  goes back onto the bench as it started, from its pose in the hand; both arms go back to the ready pose.
"""

from __future__ import annotations

import functools
import math

import numpy as np

import checks as C
import demo_kit as kit
import parts as P
import readings as R

Move = kit.Move
OPEN_MM = 93.9  # the gripper's full opening (mm)
UP = np.array([0.0, 0.0, 1.0])
HOVER_M = 0.003  # the socket face 3 mm over the plate's top face, aligned twice
LOW_HOVER_M = 0.0005  # then 0.5 mm over it, aligned twice more before it is set down on the row
SEAT_AIM_MM = 0.6  # set down within this of the row's middle, or lifted and aligned again (the rule's tolerance: 0.8 mm)
PRESS_PUSH_N = 8.5  # the downward push at the socket while seating (PRESS_N plus the gauge's zero and a margin)
LEVER_PUSH_N = 3.0  # the downward push at the grasp point while rocking (2.5 N at the row through the lever)
ROCK_STEP_DEG, ROCK_STEP_S, ROCK_MAX_DEG = 1.25, 0.125, 36.25  # at 36 deg the handle's tail is 3.9 mm over the bench
GRIP_SEP = 0.0  # the right hand closes fully on the 16 mm handle (about 29 N per finger; 7 N let it turn out, run l10t12)
GRIP_PLATE = 0.0  # the left hand closes fully on the 15.8 mm plate
PINCH_IN_TARGET = np.array([0.0, 0.008, 0.0])  # the left hand's pinch: 8 mm from the plate's middle towards its free end
PULL_IN_TARGET = np.array([0.0, 0.002, -0.014])  # the pull: 14 mm along the studs out of the socket, 2 mm away from it
PULL_STEPS, PULL_STEP_S = 14, 0.1  # the pull goes in 1 mm steps and stops when the plate is out
# The left hand pulls along the studs with 6 N and the right holds the separator against it with 6 N at the socket:
# moved by position alone, the left reached 2.6 N of the 4 N hold and both arms gave way (run l10t22)
PULL_PUSH_N = 6.0
PLATE_OVER_FLOOR_M = 0.004  # the plate's lowest corner 4 mm over the dish's floor when it is let go
SEP_OVER_BENCH_M = 0.002  # the separator is let go 2 mm over the bench, lying as it started
SEP_PUT_XY = P.SEP_START_XY  # where the separator goes back to
# The left hand holds the base down while the right levers: at peel onset the plate's hinge pulls the 17 g base up at
# the plate's far edge (2.7 N of hinge load, 46 mm from the separator's lower edge), far more than its 0.17 N weight
# holds (run l10t16, on the earlier 40 g base).
# The fingertips land on the base's last stud column: their lowest point at y = +21.3 mm, 6.6 mm inside the base's far
# edge (+0.0279) and 73 mm from the target plate's far edge, so the corner beyond them cannot rise.
BRACE_XY = (P.BASE_XY[0], P.BASE_XY[1] + 0.053)
BRACE_TILT_DEG = 50.0  # the left hand leans away from the right (48 mm between the hands' hulls, checks.py)
BRACE_PUSH_N = 8.0
STRAIGHT_BRACE_XY, STRAIGHT_BRACE_TILT_DEG = (0.282, -0.020), 55.0  # (d): 22 mm beside the plate's hinge, 20 mm from the right hand
LIFT_STEP_M, LIFT_MAX_M = 0.001, 0.003  # rocked to the end and still on: lift the rocked separator 1 mm at a time


def grip_for(width_m: float) -> float:
    return float(np.clip(1000.0 * width_m / OPEN_MM, 0.0, 1.0))


@functools.lru_cache(maxsize=1)
def nominal_in_hand() -> tuple[np.ndarray, np.ndarray]:
    """The separator's frame in the gripper's frame for the planned grasp on its handle."""
    g_s, r_s = C.right_grasp_in_sep()
    return -r_s.T @ g_s, r_s.T


def tcp_for(body_pos, body_rot, in_hand) -> tuple[np.ndarray, np.ndarray]:
    """The grasp point and gripper rotation that hold a body at (body_pos, body_rot), the body sitting in the hand as
    ``in_hand`` = (position, rotation) in the gripper's frame."""
    p, r = in_hand
    rot = np.asarray(body_rot, dtype=float) @ r.T
    return np.asarray(body_pos, dtype=float) - rot @ p, rot


def bench_pose(top: float, xy=P.SEP_START_XY, over: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    rot = C.rx(P.HANDLE_RAD) @ P.R_SEP_SEATED
    edge = np.array([xy[0], xy[1], top + over])
    return edge - rot @ np.array([P.EDGE_X, 0.0, -P.EDGE_DZ]), rot


def pick_moves(top: float) -> list[Move]:
    """Right arm: over the separator, down to its handle, pinch across its 16 mm width, lift, carry over the row."""
    sp, sr = bench_pose(top)
    g, rot = tcp_for(sp, sr, nominal_in_hand())
    seat_p, seat_r = P.separator_rocked_pose(top, 0.0, over=0.0)
    g_over, rot_over = tcp_for(seat_p + 0.020 * UP, seat_r, nominal_in_hand())
    g_open = grip_for(P.SEP_W + 0.030)
    return [
        Move(2.5, g + 0.065 * UP, rot, g_open, "over separator"),
        Move(1.2, g + 0.016 * UP, rot, g_open, "align over handle"),
        Move(1.0, g, rot, g_open, "down to handle"),
        Move(0.5, g, rot, g_open, "settle handle"),
        Move(1.2, g, rot, GRIP_SEP, "pinch separator"),
        Move(1.2, g + 0.060 * UP, rot, GRIP_SEP, "lift separator"),
        Move(2.0, g_over, rot_over, GRIP_SEP, "over near row"),
    ]


def rock_pose(p0, r0, axis, theta: float) -> tuple[np.ndarray, np.ndarray]:
    """The separator rocked handle-down by ``theta`` from its seated pose (p0, r0): its lower edge comes down the
    1.0 mm onto the stud tops over the first 2.6 deg, then the separator turns about that edge."""
    e_s = np.array([P.EDGE_X, 0.0, -P.EDGE_DZ])
    drop = (P.PLATE_T - P.STUD_H) - P.EDGE_DZ
    theta0 = math.atan2(drop, P.EDGE_X + 0.5 * P.SOCKET_LEN)
    pivot = p0 + r0 @ e_s - UP * drop * min(1.0, theta / theta0)
    rot = kit.about(axis, theta) @ r0
    return pivot - rot @ e_s, rot


class Director:
    """Adds the moves that need the simulator's poses. ``mode``: "demo" (to the end), "lever" (seat and lever),
    "lever_free" (the same with the base not held), "pull" (on through the hand-over and the pull), "straight" (seat,
    brace the base, pull the separator straight up)."""

    def __init__(self, m, mode: str = "demo") -> None:
        self.m, self.mk, self.top, self.mode = m, m.mk, m.TOP, mode
        self.log: dict = {"mode": mode, "seat": {}, "lever": [], "handover": {}, "place": {}, "failed": None}
        self.t, self.arms, self.wait = 0.0, None, None
        self.gen = self.script()

    def __call__(self, t, arms, pictures) -> None:
        self.t, self.arms = t, arms
        while self.wait is None or self.wait():
            try:
                self.wait = next(self.gen)
            except StopIteration:
                self.wait = lambda: False
                return

    # -- helpers ---------------------------------------------------------------------------------------------------
    def append(self, side: str, *moves: Move) -> None:
        arm = self.arms[side]
        gap = self.t - arm.seconds
        if gap > 1e-9:
            arm.moves.append(Move(gap, None, None, None, "wait"))
        arm.moves.extend(moves)

    def step(self):
        """Wait one control step."""
        t0 = self.t
        return lambda: self.t > t0 + 1e-9

    def hold(self, side: str, seconds: float, label: str = "hold"):
        if seconds > 1e-9:
            self.append(side, Move(seconds, None, None, None, label))
        return lambda: self.t >= self.arms[side].seconds - 1e-9

    def end_hold_now(self, side: str) -> None:
        """Ends the arm's last move now if it is a hold (nothing moves), so the next move starts at once."""
        arm = self.arms[side]
        last = arm.moves[-1] if arm.moves else None
        if last is not None and last.pos is None and last.rot is None and last.push is None:
            last.seconds = max(0.0, last.seconds - (arm.seconds - self.t))

    def in_hand(self, side: str, body: str) -> tuple[np.ndarray, np.ndarray]:
        pos, rot = self.arms[side].state(self.t)[:2]
        bp, br = self.mk.pose(body)
        return rot.T @ (bp - pos), rot.T @ br

    def hand_ok(self, in_hand) -> bool:
        """The separator still sits in the hand near the planned grasp (8 mm, 15 deg): otherwise the run stops."""
        p0, r0 = nominal_in_hand()
        turn = math.degrees(math.acos(max(-1.0, min(1.0, 0.5 * (float(np.trace(r0.T @ in_hand[1])) - 1.0)))))
        ok = float(np.linalg.norm(in_hand[0] - p0)) < 0.008 and turn < 15.0
        if not ok:
            self.fail(f"the separator moved in the hand: {1000 * float(np.linalg.norm(in_hand[0] - p0)):.1f} mm, {turn:.1f} deg")
        return ok

    def row_target(self, height: float) -> tuple[np.ndarray, np.ndarray]:
        pb, rb = self.mk.pose("target")
        return pb + rb @ (R.ROW_IN_TARGET + np.array([0.0, 0.0, height])), rb @ P.R_SEP_SEATED

    def fail(self, why: str) -> None:
        self.log["failed"] = f"{self.t:.2f} s: {why}"

    @staticmethod
    def rounded(off: dict) -> dict:
        return {k: round(float(v), 3) for k, v in off.items()}

    # -- the script ------------------------------------------------------------------------------------------------
    def script(self):
        yield self.hold("right", 0.4, "settle over row")
        if self.mode != "lever_free":
            # The left hand holds the base before the press: its reaction reaches the base as a couple about the plate's
            # far edge (the plate floats over the recessed pocket), 0.37 N m at the 13 N seat press, far more than the
            # 17 g plate's weight holds, and it tipped the base's far corner 8.7 mm up (run l10t24).
            yield from self.brace()
        seated = yield from self.seat()
        if not seated:
            return
        if self.mode == "straight":
            yield from self.straight_pull()
            return
        off = yield from self.lever()
        if self.mode != "lever_free":
            yield from self.unbrace()
        if not off or self.mode in ("lever", "lever_free"):
            yield self.hold("right", 1.0)
            return
        yield from self.handover_and_pull()
        if self.mk.is_cut("w_seat") and self.mode == "demo":
            yield from self.place()

    def seat(self):
        """Hover 3 mm over the row aligned to the separator's own pose in the hand (two readings), then 0.5 mm over it
        (two more, settled longer: the arm drifts up to 1 mm sideways on a 3 mm descent, run l10t8); the push closes
        the last 0.5 mm and presses until the seat is made."""
        log = self.log["seat"]
        for attempt in range(3):
            for k, (height, move_s, settle_s) in enumerate(((HOVER_M, 1.0, 0.4), (HOVER_M, 0.5, 0.4), (LOW_HOVER_M, 0.8, 0.6), (LOW_HOVER_M, 0.4, 0.6))):
                in_hand = self.in_hand("right", "separator")
                if not self.hand_ok(in_hand):
                    return False
                g, rot = tcp_for(*self.row_target(height), in_hand)
                self.append("right", Move(move_s, g, rot, GRIP_SEP, f"{1000 * height:.1f} mm over row"))
                yield self.hold("right", settle_s, "settle over row")
                if k == 1:
                    log[f"hover_{attempt}"] = self.rounded(R.socket_offset())
            log[f"low_hover_{attempt}"] = self.rounded(R.socket_offset())
            want_p, want_r = self.row_target(0.0)
            g, rot = tcp_for(want_p, want_r, self.in_hand("right", "separator"))
            self.append("right", Move(0.5, g, rot, GRIP_SEP, "down onto row"))
            yield self.hold("right", 0.3, "rest on row")
            off = R.socket_offset()
            log[f"contact_{attempt}"] = {"t": round(self.t, 2), **self.rounded(off)}
            if not (off["horiz_mm"] <= SEAT_AIM_MM and off["tilt_deg"] <= P.SOCKET_TOL_DEG and off["yaw_deg"] <= P.SOCKET_TOL_DEG):
                g_up, rot_up = tcp_for(*self.row_target(LOW_HOVER_M), self.in_hand("right", "separator"))
                self.append("right", Move(0.4, g_up, rot_up, GRIP_SEP, "up to realign"))
                yield self.hold("right", 0.4, "settle over row")
                continue
            at = rot.T @ (want_p - g)  # the socket in the gripper's frame: the push acts there
            self.append(
                "right",
                Move(0.6, None, None, GRIP_SEP, "press to seat", push=(0.0, 0.0, -PRESS_PUSH_N), push_at=at),
                Move(1.4, None, None, GRIP_SEP, "hold press"),
            )
            end = self.arms["right"].seconds
            while self.mk.is_cut("w_seat") and self.t < end - 1e-9:
                yield self.step()  # one control step
            if not self.mk.is_cut("w_seat"):
                log["seated"] = {"t": round(self.t, 2), "attempt": attempt, **(self.m.STATE.get("seat_align") or {})}
                self.end_hold_now("right")
                self.append("right", Move(0.4, None, None, GRIP_SEP, "ease press", push=(0.0, 0.0, 0.0)))
                yield self.hold("right", 0.2)
                return True
            log[f"missed_{attempt}"] = {"peak_press_n": self.m.STATE.get("peak_press_n"), "best": self.m.STATE.get("best_press_align")}
            self.append("right", Move(0.4, None, None, GRIP_SEP, "ease press", push=(0.0, 0.0, 0.0)))
            yield self.hold("right", 0.3)
        self.fail("the separator did not seat in three tries")
        return False

    def lever(self):
        p0, r0 = self.mk.pose("separator")
        in_hand = self.in_hand("right", "separator")
        axis = self.mk.pose("target")[1][:, 0]
        k = 0
        while not self.mk.is_cut("w_target") and k * ROCK_STEP_DEG < ROCK_MAX_DEG - 1e-9:
            k += 1
            theta = math.radians(k * ROCK_STEP_DEG)
            g, rot = tcp_for(*rock_pose(p0, r0, axis, theta), in_hand)
            push = (0.0, 0.0, -LEVER_PUSH_N * min(1.0, k / 4.0))
            self.append("right", Move(ROCK_STEP_S, g, rot, GRIP_SEP, f"rock {k * ROCK_STEP_DEG:.1f} deg", push=push, push_at=(0.0, 0.0, 0.0)))
            self.hold("left", self.arms["right"].seconds - max(self.t, self.arms["left"].seconds), "hold base")
            yield lambda: self.t >= self.arms["right"].seconds - 1e-9
            _, rs = self.mk.pose("separator")
            rocked = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(rs[:, 2], r0[:, 2]))))))
            self.log["lever"].append({"t": round(self.t, 2), "asked_deg": k * ROCK_STEP_DEG, "rocked_deg": round(rocked, 2),
                                      "peel_deg": round(math.degrees(float(self.mk.joint("peel"))), 2), "off": self.mk.is_cut("w_target")})  # fmt: skip
        lifted = 0.0
        while not self.mk.is_cut("w_target") and lifted < LIFT_MAX_M - 1e-9:  # the straight pull finishes what the rock began
            lifted += LIFT_STEP_M
            g, rot = tcp_for(*rock_pose(p0, r0, axis, math.radians(k * ROCK_STEP_DEG)), in_hand)
            self.append("right", Move(0.2, g + lifted * UP, rot, GRIP_SEP, f"lift {1000 * lifted:.0f} mm", push=(0.0, 0.0, 0.0)))
            yield lambda: self.t >= self.arms["right"].seconds - 1e-9
            self.log["lever"].append({"t": round(self.t, 2), "lifted_mm": round(1000 * lifted), "peel_deg": round(math.degrees(float(self.mk.joint("peel"))), 2), "off": self.mk.is_cut("w_target")})
        off = self.mk.is_cut("w_target")
        self.end_hold_now("right")
        self.append("right", Move(0.3, None, None, GRIP_SEP, "plate off: ease the push" if off else "ease the push", push=(0.0, 0.0, 0.0)))
        yield self.hold("right", 0.3)
        if not off:
            self.fail(f"the plate stayed on the base after a {k * ROCK_STEP_DEG:.1f} deg rock and a {1000 * lifted:.0f} mm lift")
        return off

    def handover_and_pull(self):
        in_hand = self.in_hand("right", "separator")
        sp, sr = self.mk.pose("separator")
        ho_r = C.rx(math.radians(C.HANDOVER_ROCK_DEG)) @ P.R_SEP_SEATED
        self.append(
            "right",
            Move(1.0, *tcp_for(sp + 0.040 * UP, sr, in_hand), GRIP_SEP, "lift the plate off"),
            Move(2.0, *tcp_for(C.HANDOVER_SOCKET_POS, ho_r, in_hand), GRIP_SEP, "to hand-over"),
        )
        yield self.hold("right", 0.8, "settle hand-over")
        pb, rb = self.mk.pose("target")
        pinch = pb + rb @ PINCH_IN_TARGET
        rot_l, _, ik_mm, _, margin = C.solve("left", pinch, C.frame(rb[:, 0], [0.0, -1.0, 0.0]))
        app = rot_l[:, 2]
        self.log["handover"] = {"plate_tilt_deg": round(math.degrees(math.acos(min(1.0, float(rb[2, 2])))), 2),
                                "rack_deg": round(math.degrees(float(self.mk.joint("rack"))), 2), "rack_x_mm": round(1000 * float(self.mk.joint("rack_x")), 2),
                                "left_ik_mm": round(ik_mm, 3), "left_margin_deg": round(margin, 1)}  # fmt: skip
        open_w = grip_for(P.PLATE_W + 0.018)
        self.append("left", Move(2.4, pinch - 0.055 * app, rot_l, open_w, "to hand-over"), Move(1.2, pinch, rot_l, open_w, "onto plate end"),
                    Move(1.2, None, None, GRIP_PLATE, "pinch plate"))  # fmt: skip
        wait = self.hold("left", 0.2)
        self.hold("right", self.arms["left"].seconds - self.t + 0.2, "hold for the pinch")
        yield wait
        self.end_hold_now("right")
        out = -rb[:, 2]  # along the studs, out of the socket
        at_r = tuple(self.in_hand("right", "separator")[0])  # the socket in the right gripper's frame: the hold acts there
        k, log = 0, self.log["handover"].setdefault("pull", [])
        while not self.mk.is_cut("w_seat") and k < PULL_STEPS:
            k += 1
            f = PULL_PUSH_N * min(1.0, k / 3.0)
            self.append("left", Move(PULL_STEP_S, pinch + rb @ (PULL_IN_TARGET * k / PULL_STEPS), rot_l, GRIP_PLATE, f"pull {k} mm",
                                     push=tuple(f * out), push_at=(0.0, 0.0, 0.0)))  # fmt: skip
            self.append("right", Move(PULL_STEP_S, None, None, GRIP_SEP, "hold against the pull", push=tuple(-f * out), push_at=at_r))
            yield lambda: self.t >= self.arms["left"].seconds - 1e-9
            log.append({"t": round(self.t, 2), "asked_mm": k, "push_n": round(f, 1), "pull_mm": round(1000 * float(self.mk.joint("pull")), 3)})
        end = self.t + 0.5
        while not self.mk.is_cut("w_seat") and self.t < end:  # pressed on at the last step for half a second more
            yield self.step()
        self.log["handover"]["pulled_out_at_s"] = round(self.t, 2) if self.mk.is_cut("w_seat") else None
        clear = pinch + rb @ (PULL_IN_TARGET * k / PULL_STEPS) - 0.025 * app + 0.025 * UP
        self.append("right", Move(0.5, None, None, GRIP_SEP, "ease the hold", push=(0.0, 0.0, 0.0)))
        if not self.mk.is_cut("w_seat"):
            self.append("left", Move(0.5, None, None, GRIP_PLATE, "ease the pull", push=(0.0, 0.0, 0.0)))
            self.fail("the plate stayed in the separator after the pull")
            return
        self.append("left", Move(1.0, clear, rot_l, GRIP_PLATE, "draw clear", push=(0.0, 0.0, 0.0)))
        yield self.hold("left", 0.2)

    def place(self):
        """The left hand turns to point down over the dish and lets the plate drop from 4 mm over the floor; the right
        hand lays the separator on the bench as it started, from its pose in the hand; both go back to ready."""
        hand_l = self.in_hand("left", "target")
        hand_r = self.in_hand("right", "separator")
        rot_down = C.frame([1.0, 0.0, 0.0], [0.0, 0.0, -1.0])
        corners = np.array([[sx * 0.5 * P.PLATE_W, sy * 0.5 * P.PLATE_L, sz * 0.5 * P.PLATE_T] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        lowest = float(min((rot_down @ (hand_l[0] + hand_l[1] @ c))[2] for c in corners))  # under the grasp point, m
        drop = np.array([P.DISH_XY[0], P.DISH_XY[1], self.top + P.DISH_FLOOR_T + PLATE_OVER_FLOOR_M - lowest])
        rot_down = C.solve("left", drop, rot_down)[0]  # the finger order that keeps the wrist farther from its limits
        open_w = grip_for(P.PLATE_W + 0.024)
        self.append("left", Move(2.4, drop + 0.060 * UP, rot_down, GRIP_PLATE, "over dish"), Move(1.2, drop, rot_down, GRIP_PLATE, "into dish"),
                    Move(0.6, None, None, open_w, "let go plate"), Move(1.0, drop + 0.070 * UP, rot_down, open_w, "up from dish"),
                    Move(2.4, self.arms["left"].start_pos, self.arms["left"].start_rot, 1.0, "ready"))  # fmt: skip
        sp, sr = bench_pose(self.top, SEP_PUT_XY, SEP_OVER_BENCH_M)
        g_put, rot_put = tcp_for(sp, sr, hand_r)
        self.append("right", Move(2.4, g_put + 0.050 * UP, rot_put, GRIP_SEP, "over bench"), Move(1.2, g_put, rot_put, GRIP_SEP, "down to bench"),
                    Move(0.6, None, None, grip_for(P.SEP_W + 0.030), "let go separator"),
                    Move(1.0, g_put + 0.060 * UP, rot_put, grip_for(P.SEP_W + 0.030), "up from bench"),
                    Move(2.4, self.arms["right"].start_pos, self.arms["right"].start_rot, 1.0, "ready"))  # fmt: skip
        self.log["place"] = {"plate_in_hand_mm": [round(1000 * float(v), 1) for v in hand_l[0]], "plate_lowest_mm": round(1000 * lowest, 1),
                             "drop_tcp_z_over_bench_mm": round(1000 * (drop[2] - self.top), 1),
                             "separator_in_hand_mm": [round(1000 * float(v), 1) for v in hand_r[0]]}  # fmt: skip
        self.hold("left", 6.0, "rest")
        yield self.hold("right", 6.0, "rest")

    def brace_pose(self):
        xy, tilt = (STRAIGHT_BRACE_XY, STRAIGHT_BRACE_TILT_DEG) if self.mode == "straight" else (BRACE_XY, BRACE_TILT_DEG)
        tilt = math.radians(tilt)
        rot0 = C.frame([1.0, 0.0, 0.0], [0.0, -math.sin(tilt), -math.cos(tilt)])
        rot = C.solve("left", np.array([xy[0], xy[1], self.top + 0.03]), rot0)[0]
        at = kit.lowest_point(rot)  # the closed fingertips' lowest point, from the grasp point
        touch = np.array([xy[0], xy[1], self.top + P.BASE_T]) - rot @ at
        return touch, rot, at

    def brace(self):
        """The left fingertips press the base down on its open studs while the right hand holds the separator."""
        touch, rot, at = self.brace_pose()
        self.append("left", Move(2.4, touch + 0.050 * UP, rot, 0.0, "over base"), Move(1.0, touch, rot, 0.0, "onto base"),
                    Move(0.5, touch - 0.001 * UP, rot, 0.0, "brace base", push=(0.0, 0.0, -BRACE_PUSH_N), push_at=at))  # fmt: skip
        wait = self.hold("left", 0.3, "brace")
        self.hold("right", self.arms["left"].seconds - self.t, "wait for the brace")
        yield wait

    def unbrace(self):
        touch, rot, _ = self.brace_pose()
        self.append("left", Move(0.3, None, None, 0.0, "ease brace", push=(0.0, 0.0, 0.0)), Move(0.8, touch + 0.050 * UP, rot, 0.0, "off base"))
        yield self.hold("left", 0.1)

    def straight_pull(self):
        """(d): with the base braced, the right hand pulls the seated separator straight up (no rocking) until
        something lets go."""
        in_hand = self.in_hand("right", "separator")
        sp, sr = self.mk.pose("separator")
        self.append("right", Move(3.0, *tcp_for(sp + 0.035 * UP, sr, in_hand), GRIP_SEP, "pull straight up"))
        self.hold("left", 3.6, "hold base")
        end = self.arms["right"].seconds + 0.5
        while self.t < end and not (self.mk.is_cut("w_target") or (self.m.STATE["ever_seated"] and self.mk.is_cut("w_seat"))):
            yield self.step()
        yield self.hold("right", 0.5)
        yield from self.unbrace()
