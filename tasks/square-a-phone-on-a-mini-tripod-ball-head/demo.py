#!/usr/bin/env python3
"""Scripted reference run for ``pixi_level``: hold the phone, press the red release button, level the phone in tilt and
roll, release the button so the ball head locks, and then release the phone and park both arms at READY.

    scripts/rta demo <id> mech/tasks/pixi_level [--check|--no-video] [--quick]

- ``--check`` / ``--no-video``: run in Isaac Sim with ``record=False`` (no MP4 written; G3 verification).
- ``--quick``: run the full bimanual plan offline through ``demo_kit.dry_run`` (IK error and joint-limit margins).
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as Ck  # noqa: E402
import parts as P  # noqa: E402

SIDES = ("left", "right")
MAX_JAW = 0.094  # m: yam_arena parallel gripper full opening

OPEN_PHONE = 0.40  # ~37.6 mm open around the 7.8 mm phone
PINCH_PHONE = 0.02  # target ~1.9 mm on 7.8 mm phone -> ~15 N per pad
OPEN_BUTTON = round(min(1.0, (P.HEAD_PINCH_SPAN + 0.026) / MAX_JAW), 3)  # ~64 mm open around 38 mm head+button
SQUEEZE_BUTTON = 0.24  # target ~22.5 mm on 35.0 mm pressed span -> ~31 N per pad (> 18 N full button stroke, < 40 N limit)
SQUEEZE_TIGHT = 0.19  # fallback if first squeeze fell short of PRESSED_FRACTION


class Plan:
    """Step-by-step bimanual generator: each yielded dict pads the shorter arm with a wait so both arms stay in sync."""

    def __init__(self, m, live: bool, let_go_first: bool = False) -> None:
        import demo_kit as kit
        from yam_arena import ik, station
        from yam_arena.constants import READY

        self.m, self.mk, self.kit, self.Move, self.live = m, m.mk, kit, kit.Move, live
        self.ready = {side: ik.tcp_pose(np.array(READY, dtype=float), station.BASES[side]) for side in SIDES}
        self.phone_in_left: tuple[np.ndarray, np.ndarray] | None = None
        self.trims: list[dict] = []
        self.let_go_first = let_go_first  # trials.py letgo: the left hand lets go while the button is still pressed

    def both(self, left: list, right: list, idle: tuple[str, str] = ("holds_phone", "waits")) -> dict:
        Move = self.Move
        tl = sum(mv.seconds for mv in left)
        tr = sum(mv.seconds for mv in right)
        if tl < tr - 1e-9:
            left = [*left, Move(tr - tl, label=idle[0])]
        elif tr < tl - 1e-9:
            right = [*right, Move(tl - tr, label=idle[1])]
        return {"left": left, "right": right}

    def _ball_world(self) -> np.ndarray:
        if self.live:
            tp, tr = self.mk.pose("tripod")
            return np.asarray(tp, dtype=float) + np.asarray(tr, dtype=float) @ np.array([0.0, 0.0, P.BALL_Z])
        return np.array(P.ball_anchor_world(self.m.TOP), dtype=float)

    def _phone_pose(self) -> tuple[np.ndarray, np.ndarray]:
        if self.live:
            pp, pr = self.mk.pose("phone")
            return np.asarray(pp, dtype=float), np.asarray(pr, dtype=float)
        _, phone_c, rp = Ck.phone_grasp_world(P.START_TILT_RAD, P.START_ROLL_RAD)
        return phone_c, rp

    def _button_tcp_and_rot(self) -> tuple[np.ndarray, np.ndarray]:
        r_pinch = Ck.RIGHT_PINCH_ROT_RAW
        if not self.live:
            return Ck.BUTTON_TCP.copy(), r_pinch
        tp, tr = self.mk.pose("tripod")
        inward = np.asarray(tr, dtype=float)[:, 0]
        horiz = np.array([inward[1], -inward[0], 0.0], dtype=float)
        rot_live = Ck.frame(-inward, horiz + np.array([0.0, 0.0, -0.28]))
        btn_center = np.asarray(tp, dtype=float) + np.asarray(tr, dtype=float) @ np.array([0.0, 0.0, P.BUTTON_Z])
        tcp = btn_center - 0.5 * P.BUTTON_PROUD * inward
        return tcp, rot_live

    def _left_for_phone(self, phone_pos: np.ndarray, phone_rot: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        assert self.phone_in_left is not None
        p_in, r_in = self.phone_in_left
        hand_rot = phone_rot @ r_in.T
        hand_pos = phone_pos - hand_rot @ p_in
        return hand_pos, hand_rot

    def steps(self):
        Move = self.Move
        ready_l, ready_r = self.ready["left"], self.ready["right"]

        # 1. Left hand grasps the tilted phone at its lower-left edge (~133 mm over the bench) BEFORE button press
        p_pos0, p_rot0 = self._phone_pose()
        l_tcp0 = p_pos0 + p_rot0 @ np.array(P.PHONE_GRASP_LOCAL, dtype=float)
        _, l_rot0, _, _, _ = Ck.solve("left", l_tcp0, p_rot0 @ Ck.LEFT_IN_PHONE_RAW)
        pre_left = l_tcp0 + np.array([-0.015, 0.075, 0.0])  # horizontal approach at z = 133.2 mm (<= 160 mm)
        self.phone_in_left = (l_rot0.T @ (p_pos0 - l_tcp0), l_rot0.T @ p_rot0)

        yield self.both(
            [
                Move(1.6, pos=pre_left, rot=l_rot0, grip=OPEN_PHONE, label="approach_phone"),
                Move(1.1, pos=l_tcp0, rot=l_rot0, grip=OPEN_PHONE, label="at_phone"),
                Move(0.6, grip=PINCH_PHONE, label="hold_phone"),
                Move(0.3, grip=PINCH_PHONE, label="steady_phone"),
            ],
            [],
            ("", "ready"),
        )
        if self.live:
            hp = np.asarray(self.mk.gripper("left")[0], dtype=float)
            hr = np.asarray(self.mk.gripper_turn("left"), dtype=float)
            pp, pr = self._phone_pose()
            self.phone_in_left = (hr.T @ (pp - hp), hr.T @ pr)

        # 2. Right hand approaches horizontally from in front and pinches button against rear neck boss
        r_tcp, r_rot = self._button_tcp_and_rot()
        inward = -r_rot[:, 0]  # closing axis was -inward, so inward along button stroke is -r_rot[:, 0]
        r_tcp_pressed = r_tcp + 0.5 * P.BUTTON_TRAVEL * inward  # centred on the 35.0 mm pressed span so both pads meet evenly
        r_horiz = np.array([r_rot[0, 2], r_rot[1, 2], 0.0], dtype=float)
        r_horiz /= np.linalg.norm(r_horiz)
        pre_right = r_tcp - 0.080 * r_horiz  # horizontal approach at z = 103.5 mm
        yield self.both(
            [],
            [
                Move(1.6, pos=pre_right, rot=r_rot, grip=OPEN_BUTTON, label="approach_button"),
                Move(1.1, pos=r_tcp, rot=r_rot, grip=OPEN_BUTTON, label="at_button"),
                Move(0.7, pos=r_tcp_pressed, grip=SQUEEZE_BUTTON, label="press_button"),
                Move(0.4, pos=r_tcp_pressed, grip=SQUEEZE_BUTTON, label="button_in"),
            ],
            ("holds_phone", ""),
        )
        if self.live and not self.m.H.is_unlocked():
            yield self.both([], [Move(0.5, grip=SQUEEZE_TIGHT, label="press_deeper"), Move(0.3, label="button_in")], ("holds_phone", ""))

        # 3. Left hand levels the phone smoothly from (+10, -20) deg to (0, 0) deg while button stays pressed
        ball = self._ball_world()
        rel_offset = np.array([0.0, 0.0, P.PHONE_CENTER_Z - P.BALL_Z], dtype=float)
        level_moves = []
        for frac in (0.25, 0.50, 0.75, 1.00):
            t_rad = P.START_TILT_RAD * (1.0 - frac)
            r_rad = P.START_ROLL_RAD * (1.0 - frac)
            rp = P.head_rotation(t_rad, r_rad)
            p_want = ball + rp @ rel_offset
            hp, hr = self._left_for_phone(p_want, rp)
            level_moves.append(Move(0.85, pos=hp, rot=hr, grip=PINCH_PHONE, label="level_phone"))
        level_moves.append(Move(0.5, grip=PINCH_PHONE, label="settle_level"))
        yield self.both(level_moves, [], ("", "holds_button"))

        # Fine visual/angle trim while button is still pressed so arm compliance residual stays well within 3 deg
        cmd_tilt, cmd_roll = 0.0, 0.0
        for trim_k in range(2 if self.live else 0):
            t_deg, r_deg = self.m.phone_tilt_roll_deg()
            self.trims.append({"iter": trim_k + 1, "seen_tilt_deg": round(t_deg, 2), "seen_roll_deg": round(r_deg, 2)})
            if max(abs(t_deg), abs(r_deg)) <= 0.6:
                break
            cmd_tilt -= 0.85 * math.radians(t_deg)
            cmd_roll -= 0.85 * math.radians(r_deg)
            rp = P.head_rotation(cmd_tilt, cmd_roll)
            ball = self._ball_world()
            hp, hr = self._left_for_phone(ball + rp @ rel_offset, rp)
            yield self.both(
                [Move(0.8, pos=hp, rot=hr, grip=PINCH_PHONE, label="trim_level"), Move(0.4, label="settle_trim")],
                [],
                ("", "holds_button"),
            )

        # 4. Right hand releases the button FIRST (locking the head at LOCK_NM) and retreats to READY while phone stays held
        right_off = [
            Move(0.7, grip=OPEN_BUTTON, label="release_button"),
            Move(0.4, grip=OPEN_BUTTON, label="head_locked"),
            Move(1.1, pos=pre_right, grip=OPEN_BUTTON, label="retreat_button"),
            Move(1.6, pos=ready_r[0], rot=ready_r[1], grip=1.0, label="right_ready"),
        ]
        if not self.let_go_first:
            yield self.both([], right_off, ("holds_locked_phone", ""))

        # 5. Left hand opens its jaws, retreats horizontally at z <= 156.9 mm, and parks at READY for 1 s rest + 2 s hold
        hp_end, _ = (
            (np.asarray(self.mk.gripper("left")[0], dtype=float), np.asarray(self.mk.gripper_turn("left"), dtype=float))
            if self.live
            else self._left_for_phone(ball + rel_offset, np.eye(3))
        )
        retreat_left = hp_end + np.array([-0.015, 0.080, 0.0])
        left_off = [
            Move(0.6, grip=OPEN_PHONE, label="release_phone"),
            Move(1.1, pos=retreat_left, grip=OPEN_PHONE, label="retreat_phone"),
        ]
        left_park = [
            Move(1.6, pos=ready_l[0], rot=ready_l[1], grip=1.0, label="left_ready"),
            Move(3.6, pos=ready_l[0], rot=ready_l[1], grip=1.0, label="ready_rest"),
        ]
        if self.let_go_first:
            yield self.both(left_off, [], ("", "holds_button"))
            yield self.both([], right_off, ("waits", ""))
            yield self.both(left_park, [], ("", "ready_rest"))
        else:
            yield self.both(left_off + left_park, [], ("", "ready_rest"))


def run_demo(embodiment, out: Path, *, seed: int = 0, record: bool = True, let_go_first: bool = False) -> dict:
    """Execute the full bimanual reference plan on ``embodiment`` and return a compact verification summary."""
    import demo_kit as kit

    m = embodiment.task.module
    mk = m.mk
    plan = Plan(m, live=True, let_go_first=let_go_first)
    gen = plan.steps()
    done = {"exhausted": False}

    def director(t: float, arms: dict, pictures) -> None:
        if pictures is None or not mk.MECH.poses or done["exhausted"]:
            return
        while all(t >= arms[s].seconds - 1e-6 for s in SIDES):
            try:
                nxt = next(gen)
            except StopIteration:
                done["exhausted"] = True
                break
            for s, mvs in nxt.items():
                arms[s].moves.extend(mvs)

    def extra() -> dict:
        t_deg, r_deg = m.phone_tilt_roll_deg()
        pp, pr = mk.pose("phone")
        hand, close = np.asarray(mk.gripper("left")[0], dtype=float), np.asarray(mk.gripper_turn("left"), dtype=float)[:, 0]
        grasp = np.asarray(pp, dtype=float) + np.asarray(pr, dtype=float) @ np.array(P.PHONE_GRASP_LOCAL, dtype=float)
        pads = [0.0, 0.0]  # left pads' normal force on the phone: [on the -closing side of the hand, on the + side]
        for c in mk.touching("phone", "left robot"):
            pads[int(float(np.dot(np.asarray(c.point, dtype=float) - hand, close)) > 0.0)] += abs(c.force)
        _, tr = mk.pose("tripod")
        return {
            "tilt_deg": round(t_deg, 2),
            "roll_deg": round(r_deg, 2),
            "button_mm": round(1000.0 * m.H.button_travel_m(), 2),
            "unlocked": m.H.is_unlocked(),
            "btn_spring_n": round(m.H.button_spring_force_n(), 2),
            "r_btn_n": round(sum(abs(c.force) for c in mk.touching("button", "right robot")), 2),
            "r_head_n": round(sum(abs(c.force) for c in mk.touching("tripod", "right robot")), 2),
            "l_phone_n": round(sum(pads), 2),
            "l_pads_n": [round(f, 2) for f in pads],
            "l_off_mm": round(1000.0 * float(np.dot(hand - grasp, close)), 2),  # hand off the phone's mid-plane
            "slide_mm": round(1000.0 * float(np.max(m.foot_slides_m())), 2),
            "lift_mm": round(1000.0 * float(np.max(m.foot_lifts_m())), 2),
            "yaw_deg": round(math.degrees(math.atan2(float(tr[1, 0]), float(tr[0, 0])) - P.TRI_YAW), 2),
        }

    res = kit.run(
        embodiment,
        {},
        out,
        seed=seed,
        director=director,
        extra=extra,
        tail_s=1.0,
        limit_s=m.RUN_LIMIT_STEPS / 20.0,
        sheet_frames=12,
        record=record,
    )
    summary = {
        "score": res["info"].get("score"),
        "terminated": res["terminated"],
        "steps": res["steps"],
        "seconds": res["seconds"],
        "run_limit_steps": m.RUN_LIMIT_STEPS,
        "worst_ik": res["worst_ik"],
        "median_step_ms": res["median_step_ms"],
        "trims": plan.trims,
        "report": res["report"],
    }
    (out / "demo_summary.json").write_text(json.dumps(summary, indent=2))
    print("DEMO " + json.dumps({k: v for k, v in summary.items() if k != "report"}, default=str), flush=True)
    return summary


def quick_dry() -> dict:
    """Run the open-loop nominal plan through ``demo_kit.dry_run`` without Isaac Sim."""
    import demo_kit as kit
    from yam_arena import station, task_file

    m = task_file.load(HERE / "task.py").module
    plan = Plan(m, live=False)
    moves: dict[str, list] = {"left": [], "right": []}
    for chunk in plan.steps():
        for s in SIDES:
            moves[s].extend(chunk[s])
    # Verify max z of left TCP across all moves
    left_zs = [round(1000.0 * (float(mv.pos[2]) - m.TOP), 1) for mv in moves["left"] if mv.pos is not None and mv.label not in ("left_ready", "ready_rest")]
    dry = kit.dry_run(moves, station.BASES, hz=20.0, tail_s=1.0, limit_s=m.RUN_LIMIT_STEPS / 20.0)
    return {"dry_ik": dry, "left_task_z_mm_range": [min(left_zs), max(left_zs)]}


def main() -> int:
    if "--quick" in sys.argv[1:]:
        print(json.dumps(quick_dry(), indent=2))
        return 0
    import demo_kit as kit

    out = Path(sys.argv[1])
    args = sys.argv[2:]
    record = not any(a in ("--check", "--no-video", "--norec") for a in args)
    embodiment = kit.open_task(HERE / "task.py")
    summary = run_demo(embodiment, out / "demo", seed=0, record=record)
    return 0 if summary["score"] == 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
