"""Scripted runs of the adjustable wrench task.

    scripts/rta demo <id> mech/tasks/adjustable_wrench [cases] [--check]   (in the simulator container)
    python mech/tasks/adjustable_wrench/demo.py --dry                      (the plan through the IK, no simulator)

Cases: ``torque_check`` (a known torque on the nut: 1.2, 1.4 and 1.6 N m, then -1.6 N m), ``full`` (the reference run: the left fingertip rolls the knurl until the jaw is snug, the right hand turns
the nut 60 degrees and more and slides the wrench off along its jaws), ``start`` (stand still: start pictures),
``wipe`` (the knurl rolled, nothing turned), ``open_pull`` (the right hand turns with the jaw left at 23 mm),
``loose_pull``, ``snug_turn`` and ``tighten`` (clockwise) (the jaw set to 18.4 / 17.2 mm before the turn by moving where the knurl drives it,
a trial's shortcut; the wrench is left on the nut), ``free_close`` (the open wrench slid off the nut along its jaws and let
go, then its jaw closed to 17.2 mm away from the nut by the same shortcut).
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

SIDES = ("left", "right")
STROKES = 8  # knurl strokes: about 1 mm of jaw each, 6 mm to close, the rest slips at the snug force
STROKE = 0.015  # m along the jaws' axis, the fingertip's travel per stroke
PRESS_N = 3.0
PINCH_S = 0.160  # m along the handle from a seated nut's middle
TURN_DEG = 85.0  # commanded; the arm lags about 23 mm at the 9.4 N the nut asks
TIGHTEN_DEG = 30.0  # the tighten trial turns clockwise
SLIDE = 0.055
FREE_SLIDE = 0.040  # free_close: the nut's axis 40 mm along the jaws, past their tips (13.6 mm), the handle's end still on the plate
FREE_STAND_S = 5.0  # free_close: after the hand has parked, the jaw closes over 3 s and the wrench lies still
SLIDE_PUSH_N = 10.0  # N along the slide: the snug jaw (4 N) drags on the flats
SLIDE_STEP, CLEAR_TO = 0.006, 0.045  # m per 0.25 s along the jaws; slide until the jaws' middle is 45 mm from the screw
EASE_DEG = 12.0  # at most: the hand eases back by the wrench's lead on the nut and the nut's 2 deg spring-back, plus 1 deg
PUSH_IN_N, PUSH_DOWN_N, PUSH_TURN_N = 2.0, 4.0, 12.0  # N while turning: towards the nut, down, and round (1.9 N m at 160 mm)
OPEN_GRIP = 0.70  # 66 mm between the pads round the 21 mm handle: the hand stands 5 to 15 mm off after a carry
PRESET_MM = {"loose_pull": 18.4, "snug_turn": 17.2, "tighten": 17.2, "free_close": 17.2}
TORQUE_STEPS = ((1.0, 1.2), (3.0, 1.4), (5.0, 1.6), (7.0, 0.0), (8.0, -1.6), (10.0, 0.0))  # (from s, N m counterclockwise on the nut)


def _kit():
    import demo_kit as kit

    return kit


class Plan:
    """The moves of a case, from the start pose the task declares (``m.START``)."""

    def __init__(self, m, case: str) -> None:
        self.m, self.P, self.case = m, m.P, case
        self.kit = _kit()
        o, rw = m.START["wrench"]
        self.o, self.rw = np.asarray(o), np.asarray(rw)
        self.nut = np.array([*self.P.NUT_XY, m.START["nut_z"]])
        self.plate_top = m.START["plate_top"]

    def flat_dir(self, v) -> np.ndarray:
        v = np.array([v[0], v[1], 0.0])
        return v / np.linalg.norm(v)

    def left_moves(self) -> list:
        """Approach, STROKES strokes on the knurl's top towards the jaw tips, retreat."""
        kit, P, rw = self.kit, self.P, self.rw
        Move = kit.Move
        x_w = self.flat_dir(rw[:, 0])
        yaw = math.atan2(x_w[1], x_w[0])  # pads' width along the stroke, closing across it
        rot = kit.down(yaw)
        low = kit.lowest_point(rot)  # the closed fingertips' lowest point from the grasp point, gripper frame
        tip = rot @ low
        top = self.o + rw @ np.array([*P.KNURL_IN_W, 0.0]) + rw[:, 2] * (P.KNURL_D / 2)
        at = lambda along, up: top + x_w * along + np.array([0.0, 0.0, up]) - tip  # noqa: E731
        moves = [
            Move(2.5, at(-STROKE / 2, 0.05), rot, 0.0, "to the knurl"),
            Move(1.0, at(-STROKE / 2, 0.006), rot, 0.0, "over the knurl"),
        ]
        down_push = np.array([0.0, 0.0, -PRESS_N])
        for k in range(STROKES):
            moves += [
                Move(0.5, at(-STROKE / 2, 0.001), rot, 0.0, f"press {k + 1}", push=down_push, push_at=low),
                Move(0.9, at(+STROKE / 2, 0.001), rot, 0.0, f"roll {k + 1}"),
                Move(0.35, at(+STROKE / 2, 0.007), rot, 0.0, f"lift {k + 1}", push=np.zeros(3)),
                Move(0.5, at(-STROKE / 2, 0.007), rot, 0.0, f"back {k + 1}"),
            ]
        moves.append(Move(1.5, at(0.0, 0.07), rot, 0.0, "away"))
        moves.append(Move(2.0, np.array([0.18, 0.26, self.plate_top + 0.18]), kit.down(0.0), 0.0, "park"))
        return moves

    def pinch(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        """Grasp point, rotation, the handle's direction and the grasp point's radius from the nut's axis."""
        kit, P, rw = self.kit, self.P, self.rw
        h = self.flat_dir(rw @ np.array([*P.HANDLE_DIR_IN_W, 0.0]))
        yaw = math.atan2(h[1], h[0]) + math.pi  # closing across the handle
        rot = kit.down(yaw)
        tip = rot @ kit.lowest_point(rot)
        point = self.o + rw @ P.handle_point_w(PINCH_S)
        grasp = np.array([point[0], point[1], self.plate_top + 0.0015]) - tip
        return grasp, rot, h, float(np.hypot(*(grasp - self.nut)[:2]))

    def right_reach(self) -> list:
        Move = self.kit.Move
        grasp, rot, _, _ = self.pinch()
        return [
            Move(3.0, grasp + [0, 0, 0.06], rot, OPEN_GRIP, "to the handle"),
            Move(0.8, grasp + [0, 0, 0.06], rot, OPEN_GRIP, "align"),
            Move(1.0, grasp + [0, 0, 0.02], rot, OPEN_GRIP, "lower"),
            Move(0.8, grasp + [0, 0, 0.02], rot, OPEN_GRIP, "settle"),
            Move(1.0, grasp, rot, OPEN_GRIP, "down beside the handle"),
            Move(3.0, grasp, rot, 0.0, "pinch the handle"),
        ]

    def right_turn(self, degrees: float = TURN_DEG) -> list:
        kit = self.kit
        grasp, rot, _, _ = self.pinch()
        rel = grasp - self.nut
        moves = []
        n = 10
        for k in range(1, n + 1):
            a = math.radians(degrees) * k / n
            turn = kit.about((0, 0, 1), a)
            inward = -self.flat_dir(turn @ rel)  # hold the wrench down on the washer and in against the nut while turning
            round_dir = np.cross([0.0, 0.0, 1.0], -inward) * math.copysign(1.0, degrees)  # counterclockwise seen from above when > 0
            push = inward * PUSH_IN_N + round_dir * PUSH_TURN_N + np.array([0.0, 0.0, -PUSH_DOWN_N])
            moves.append(kit.Move(0.7, self.nut + turn @ rel, turn @ rot, 0.0, f"turn {a * 180 / math.pi:.0f} deg", push=push))
        return moves

    def right_slide(self, wrench_rot: np.ndarray, start: np.ndarray, rot: np.ndarray, slide: float = SLIDE) -> list:
        """Along the jaws' axis away from the tips (the nut leaves between the jaws), then let go and lift."""
        Move = self.kit.Move
        back = -self.flat_dir(wrench_rot[:, 0])
        end = start + back * slide
        return [
            Move(3.0, end, rot, 0.0, "slide off along the jaws", push=back * SLIDE_PUSH_N),
            Move(0.5, end, rot, 0.0, "stop", push=np.zeros(3)),
            Move(2.0, end, rot, OPEN_GRIP, "let go"),
            Move(1.5, end + [0, 0, 0.07], rot, OPEN_GRIP, "lift"),
        ]

    def nominal(self) -> dict[str, list]:
        """The whole plan without the simulator (the slide from the start pose turned by the commanded angle)."""
        Move, kit = self.kit.Move, self.kit
        left = self.left_moves() if self.case in ("full", "wipe") else []
        t_left = sum(mv.seconds for mv in left)
        right: list = []
        if self.case in ("full", "open_pull", "loose_pull", "snug_turn", "tighten"):
            degrees = -TIGHTEN_DEG if self.case == "tighten" else TURN_DEG
            right = [Move(t_left + 0.5, None, None, None, "wait")] + self.right_reach() + self.right_turn(degrees)
            if self.case == "full":
                turn = kit.about((0, 0, 1), math.radians(TURN_DEG))
                grasp, rot, _, _ = self.pinch()
                right += self.right_slide(turn @ self.rw, self.nut + turn @ (grasp - self.nut), turn @ rot)
            elif self.case == "snug_turn":  # the 0.5 level: loosened with a snug jaw, the hand still on the handle (eased in run_case)
                pass
            else:
                grasp, rot, _, _ = self.pinch()
                turn = kit.about((0, 0, 1), math.radians(degrees))
                end = self.nut + turn @ (grasp - self.nut)
                right += [Move(2.0, end, turn @ rot, OPEN_GRIP, "let go", push=np.zeros(3)), Move(1.5, end + [0, 0, 0.07], turn @ rot, OPEN_GRIP, "lift")]
        if self.case == "free_close":  # the open wrench taken off the nut, let go; the jaw is closed afterwards (run_case)
            grasp, rot, _, _ = self.pinch()
            right = [Move(0.5, None, None, None, "wait")] + self.right_reach() + self.right_slide(self.rw, grasp, rot, FREE_SLIDE)
            right += [Move(2.0, np.array([0.18, -0.26, self.plate_top + 0.18]), kit.down(0.0), OPEN_GRIP, "park"),
                      Move(FREE_STAND_S, None, None, None, "stand")]
        if self.case == "start":
            right = [Move(1.0, None, None, None, "stand")]
        if self.case == "torque_check":
            right = [Move(TORQUE_STEPS[-1][0] + 1.0, None, None, None, "stand")]
        return {"left": left, "right": right}


def dry() -> dict:
    """Every case's plan through the IK: worst error and smallest joint margin per arm."""
    kit = _kit()
    from yam_arena import station, task_file

    m = task_file.load(HERE / "task.py").module
    out = {}
    for case in ("full", "open_pull"):
        plan = Plan(m, case)
        moves = plan.nominal()
        out[case] = kit.dry_run(moves, station.BASES, hz=20.0, tail_s=0.5, limit_s=m.RUN_LIMIT_STEPS / 20.0)
        out[case]["pinch_radius_mm"] = round(1000 * plan.pinch()[3], 1)
    return out


def run_case(embodiment, case: str, out: Path, record: bool) -> dict:
    kit = _kit()
    m = embodiment.task.module
    mk = m.mk
    plan = Plan(m, case)
    moves = plan.nominal()
    if case == "full":  # the slide is added once the turn is done, from where the wrench is then
        n_slide = 4
        moves["right"] = moves["right"][:-n_slide]
    state = {"stage": "turn" if case in ("full", "snug_turn") else "done", "preset": PRESET_MM.get(case), "eased_at": None}
    t_turn_end = sum(mv.seconds for mv in moves["right"])
    preset_from = t_turn_end - FREE_STAND_S + 0.5 if case == "free_close" else 0.2  # free_close: once the hand has parked

    def director(t: float, arms: dict, pictures) -> None:
        if not mk.MECH.poses:
            return
        if case == "torque_check":  # a known torque on the nut alone (no wrench on it): where does it slip?
            now = [tq for t0, tq in TORQUE_STEPS if t >= t0]
            mk.push("nut", torque=(0.0, 0.0, now[-1] if now else 0.0))
        if state["preset"] is not None and preset_from <= t <= preset_from + 3.05:  # trial shortcut: the jaw driven to a set opening over 3 s
            share = min(1.0, (t - preset_from) / 3.0)
            opening = m.P.JAW_START + share * (state["preset"] / 1000.0 - m.P.JAW_START)
            m.STATE["slip"] = float(m.mk.joint("knurl")) + (opening - m.P.JAW_START) * 2 * math.pi / m.P.KNURL_PITCH
        _, rot0, _, _ = plan.pinch()
        if state["stage"] == "turn" and t >= t_turn_end + 0.3:  # ease back EASE_DEG: the hex leaves its wedge in the jaws' play
            _, wr = mk.pose("wrench")
            turned = math.atan2(wr[1, 0], wr[0, 0]) - math.atan2(plan.rw[1, 0], plan.rw[0, 0])
            point = np.asarray(mk.gripper("right")[0], dtype=float)
            ease = math.radians(float(np.clip(math.degrees(turned) - m.nut_deg() + 3.0, 0.0, EASE_DEG)))  # lead + spring-back + 1 deg
            back = kit.about((0, 0, 1), -ease)
            eased = plan.nut + back @ (point - plan.nut)
            rot = kit.about((0, 0, 1), turned - ease) @ rot0
            arms["right"].moves.extend([kit.Move(1.0, eased, rot, 0.0, "ease back", push=np.zeros(3)), kit.Move(0.6, eased, rot, 0.0, "rest")])
            state["stage"], state["eased_at"] = "ease", t + 1.6
        if state["stage"] == "ease" and t >= state["eased_at"] and case == "snug_turn":  # the 0.5 trial keeps holding
            arms["right"].moves.append(kit.Move(3.0, None, None, 0.0, "hold the handle"))
            state["stage"] = "done"
        if state["stage"] == "ease" and t >= state["eased_at"]:  # then let go: the wrench stays on the loosened nut
            point = np.asarray(mk.gripper("right")[0], dtype=float)
            _, wr = mk.pose("wrench")
            turned = math.atan2(wr[1, 0], wr[0, 0]) - math.atan2(plan.rw[1, 0], plan.rw[0, 0])
            rot = kit.about((0, 0, 1), turned) @ rot0
            arms["right"].moves.extend([kit.Move(2.0, point, rot, OPEN_GRIP, "let go", push=np.zeros(3)),
                                        kit.Move(1.5, point + [0, 0, 0.07], rot, OPEN_GRIP, "lift"),
                                        kit.Move(2.0, np.array([0.18, -0.26, plan.plate_top + 0.18]), kit.down(0.0), OPEN_GRIP, "park")])  # fmt: skip
            state["stage"] = "done"
        if state["stage"] == "slide" and t >= arms["right"].seconds - 1e-6:
            # each 0.25 s: SLIDE_STEP along the jaws (as the wrench now lies), and the hand turned back about the screw by
            # the wrench's lead on the nut, so the pull, 41 mm off the jaws' line, does not wedge the hex again
            _, wr = mk.pose("wrench")
            turned = math.atan2(wr[1, 0], wr[0, 0]) - math.atan2(plan.rw[1, 0], plan.rw[0, 0])
            lead = turned - math.radians(m.nut_deg())
            fix = -lead if abs(math.degrees(lead)) > 0.3 else 0.0
            back = -plan.flat_dir(wr[:, 0])
            o, _ = mk.pose("wrench")
            middle = m.P.jaw_middle(o, wr, m.jaw_m())
            if math.hypot(middle[0] - m.P.NUT_XY[0], middle[1] - m.P.NUT_XY[1]) >= CLEAR_TO or t - state["slide_from"] > 12.0:
                rot = kit.about((0, 0, 1), turned) @ rot0
                arms["right"].moves.extend([kit.Move(0.5, state["cmd"], rot, 0.0, "stop", push=np.zeros(3)),
                                            kit.Move(2.0, state["cmd"], rot, OPEN_GRIP, "let go"),
                                            kit.Move(1.5, state["cmd"] + [0, 0, 0.07], rot, OPEN_GRIP, "lift")])  # fmt: skip
                state["stage"] = "done"
            else:
                turn_back = kit.about((0, 0, 1), fix)
                state["cmd"] = plan.nut + turn_back @ (state["cmd"] - plan.nut) + back * SLIDE_STEP
                rot = kit.about((0, 0, 1), turned + fix) @ rot0
                arms["right"].moves.append(kit.Move(0.25, state["cmd"], rot, 0.0, "slide off along the jaws", push=back * SLIDE_PUSH_N))

    res = kit.run(embodiment, moves, out, seed=0, director=director, extra=m.readings, tail_s=1.0,
                  limit_s=m.RUN_LIMIT_STEPS / 20.0, sheet_frames=16, record=record)  # fmt: skip
    summary = {"case": case, "score": res["info"].get("score"), "success": res["info"].get("success"), "terminated": res["terminated"],
               "steps": res["steps"], "seconds": res["seconds"], "worst_ik": res["worst_ik"], "median_step_ms": res["median_step_ms"],
               "report": res["report"]}  # fmt: skip
    (out / "summary.json").write_text(json.dumps(summary, indent=2, default=str))
    print("CASE " + json.dumps({k: v for k, v in summary.items() if k != "report"}, default=str), flush=True)
    print("REPORT " + json.dumps(res["report"], default=str), flush=True)
    return summary


def main() -> int:
    if "--dry" in sys.argv[1:]:
        print(json.dumps(dry(), indent=2, default=str))
        return 0
    kit = _kit()
    out = Path(sys.argv[1])
    args = [a for a in sys.argv[2:] if not a.startswith("--")]
    cases = args[0].split(",") if args else ["full"]
    record = "--check" not in sys.argv[2:]
    embodiment = kit.open_task(HERE / "task.py")
    scores = {case: run_case(embodiment, case, out / case, record)["score"] for case in cases}
    print("SCORES " + json.dumps(scores), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
