#!/usr/bin/env python3
"""Reference runs of the bar clamp task: the full score, the jaws put round the boards and left, the trigger pumped
until the joint closes and no further, the bar pushed through by hand, and the hand that stays.

    scripts/rta demo <id> mech/tasks/bar_clamp [run,run,...] [--quick]

The right hand, pointing straight down and open, its fingers closing across the handle (along y), comes down over the
handle's far end, past the trigger's tip, and closes on the handle's two faces there (the trigger untouched). It
slides the clamp 7 cm along the bench towards the boards and 2 mm to the right, the jaws passing along the boards'
outer edges (the clamp comes about 6.2 cm: the arm lags against its drag), until both pads' middles are about 2 cm
inside the boards' near ends. It opens, rises, and comes down over
handle and trigger 4 cm from the trigger's pin, where a finger works the trigger, and pumps: it closes fully (each
finger pushing with up to 50 N) in 0.6 s and opens again in 0.5 s. Each stroke draws the bar through the grip; the
bar's jaw pushes the left board against the right one and both against the gauge pad. When the jaws and the joint are
closed it gives three more strokes, then opens, goes up 8 cm and stays there. Every grasp after the first is placed
where the clamp is then (c6c: a pump grasp placed where the clamp should have been came down on top of the handle).

What the script reads from the simulator: where the clamp's grip and bar are, to put the hand on them; whether the
pads and the joint are closed and, in ``closed``, how far they are from touching (a person sees the jaws come to the
boards and the boards to each other), to know when to give the three more strokes and, in ``closed`` and ``pushed``,
when to stop; nothing else: not the gauge, and not the score.

Runs. ``full``: 1.0. ``round``: put round the boards, no stroke (0.25). ``closed``: pumped until the joint is closed:
full strokes while the jaws and the joint are more than 7 mm from touching, then slow squeezes, the hand letting go as
soon as the pads and the joint touch (0.5; c6h, which let go as soon as they were within the score's 0.5 mm, ended at
0.48 mm). ``pushed``: the bar's jaw pushed by hand towards the grip until the joint is
closed, then the hand backs off; no stroke (0.5 at most: the gauge shows why the trigger is needed). ``held``: as
``full``, the hand stays closed on handle and trigger (0.75).

``--quick``: every run's plan against the arm's reach and joint limits, without the simulator.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

RUNS = ("full", "round", "closed", "pushed", "held")
ROUND = 0.070  # m the hand slides the clamp towards the boards; the clamp comes about 6.2 cm (c6g: the arm lags 7.5 mm against the clamp's drag on the bench and holds there), the pads' middles about 2 cm inside the boards' near ends; at 7 cm it would still stop 4.5 mm short of them
SLIDE_AT, FINGER = 0.072, 0.040  # m from the trigger's pin along the handle where the hand holds the clamp to slide it (past the trigger's tip, 64 mm); where it pumps
SLIDE_AWAY = 0.002  # m the slide goes to the right besides towards the boards: the grip's jaw passes 5 mm outside the right board (c6c: pushed 7 mm sideways, it caught the board's end)
GRASP_Z = 0.016  # m the grasp point over the bench: the fingertips' ends 3 mm over it
OPEN, WIDE, HOLD, SQUEEZE = 0.70, 0.85, 0.15, 0.0  # the hand's opening (0 closed .. 1 open, 94 mm): 66 mm; 80 mm to come down over handle and trigger; holding the handle; pumping
UP = 0.06  # m the open hand goes up between grasps
SLIDE_S, SQUEEZE_S, LET_S = 2.5, 0.6, 0.5
MORE_STROKES, MOST_STROKES = 3, 14
PUSH_STEP, PUSH_MOST = 0.002, 0.045  # m: the closed hand pushes the bar's jaw 2 mm a look, at most 4.5 cm (the bar has 2.2 cm to go; the arm lags about 1 cm: c6g)
SLOW_FROM, SLOW_STEP, SLOW_S = 0.007, 0.015, 0.1  # ``closed``: slow squeezes once the jaws and the joint are 7 mm or less from touching, closing 0.7 mm a finger (1 degree of trigger, 0.26 mm of bar) every 0.1 s
TOUCH = 0.0002  # m: ``closed`` squeezes until both pads and both ends of the joint are this close: they touch
DRY_STROKES = 8


def opening(m) -> float:
    """How far the jaws and the joint are from touching (m): both pads' gaps and the joint at the boards' near ends,
    where the jaws push (the far end closes as the left board turns flat against the right one: c6h)."""
    gaps = [m.pad_place(side)["gap"] for side in ("grip", "bar")]
    return max(0.0, gaps[0]) + max(0.0, gaps[1]) + max(0.0, m.joint_gaps()[0])


def touching(m) -> bool:
    """Round the boards, both pads and both ends of the joint within TOUCH."""
    gaps = [m.pad_place(side)["gap"] for side in ("grip", "bar")] + list(m.joint_gaps())
    return m.round_boards() and all(g <= TOUCH for g in gaps)


class Plan:
    def __init__(self, m, see=None) -> None:
        import demo_kit as kit

        self.m, self.see, self.kit, self.Move = m, see, kit, kit.Move
        self.strokes = 0
        self.moved = np.zeros(3)  # the dry run's clamp: how far the plan has slid it

    def pose(self, body: str = "grip") -> tuple[np.ndarray, np.ndarray]:
        """Where the clamp's grip (or bar) is now; in the dry run, where the plan has put it."""
        if self.see is not None:
            return self.see()["poses"][body]
        return (self.m.GRIP_AT if body == "grip" else self.m.BAR_AT) + self.moved, np.eye(3)

    def on_clamp(self, point, body: str = "grip") -> np.ndarray:
        """``point``, given where it is in the scene as built, as a point of the grip (or the bar) where that is now."""
        position, rotation = self.pose(body)
        return position + rotation @ (np.asarray(point, dtype=float) - (self.m.GRIP_AT if body == "grip" else self.m.BAR_AT))

    def turned(self) -> np.ndarray:
        """The hand pointing down, its fingers closing across the clamp's handle (along the bar), as the clamp lies now."""
        rotation = self.pose()[1]
        return self.kit.down(math.atan2(rotation[1, 0], rotation[0, 0]))

    def at_handle(self, distance: float, trigger: bool = True) -> np.ndarray:
        """The grasp point ``distance`` m from the trigger's pin along the handle: between the handle's outer face and
        the open trigger's outer face there (``trigger``), or the handle's middle; where the clamp is now."""
        m = self.m
        handle = m.HANDLE[1]
        if trigger:  # the open trigger's outer face at that distance along the handle
            outer = m.PIN[1] - distance * math.tan(m.TRAVEL) - m.TRIGGER_T / (2 * math.cos(m.TRAVEL))
            middle = (handle[1] + outer) / 2
        else:
            middle = (handle[0] + handle[1]) / 2
        return self.on_clamp((m.PIN[0] - distance, middle, m.TOP + GRASP_Z))

    def closed(self) -> bool:
        return bool(self.see()["closed"]) if self.see is not None else self.strokes >= DRY_STROKES

    def touching(self) -> bool:
        return bool(self.see()["touching"]) if self.see is not None else self.strokes >= DRY_STROKES

    def opening(self) -> float:
        """How far the jaws and the joint are from closed (m); in the dry run 2 cm less 5.7 mm a stroke given."""
        return float(self.see()["opening"]) if self.see is not None else max(0.0, 0.020 - 0.0057 * self.strokes)

    def pump(self, name: str):
        """Squeeze and let go until the jaws and the joint are closed, then MORE_STROKES more. ``closed``: none more;
        full strokes while the jaws and the joint are more than SLOW_FROM from touching, then slow squeezes, each let
        go as soon as the pads and the joint touch."""
        Move = self.Move
        if name == "closed":
            while self.strokes < MOST_STROKES and not self.touching():
                self.strokes += 1
                if self.opening() > SLOW_FROM:
                    yield [Move(SQUEEZE_S, grip=SQUEEZE, label=f"squeeze {self.strokes}"), Move(LET_S, grip=OPEN, label="lets go")]
                    continue
                grip = OPEN
                while grip > SQUEEZE and not self.touching():
                    grip = max(SQUEEZE, grip - SLOW_STEP)
                    yield [Move(SLOW_S, grip=grip, label=f"squeeze {self.strokes} slowly")]
                yield [Move(LET_S, grip=OPEN, label="lets go")]
            return
        after = None
        while self.strokes < MOST_STROKES:
            if after is None and self.closed():
                after = self.strokes
            if after is not None and self.strokes >= after + MORE_STROKES:
                break
            self.strokes += 1
            yield [Move(SQUEEZE_S, grip=SQUEEZE, label=f"squeeze {self.strokes}"), Move(LET_S, grip=OPEN, label="lets go")]

    def steps(self, name: str):
        m, Move = self.m, self.Move
        up = np.array([0.0, 0.0, UP])
        hold = self.at_handle(SLIDE_AT, trigger=False)
        slid = hold + np.array([ROUND, -SLIDE_AWAY, 0.0])
        yield [Move(2.5, pos=hold + up, rot=self.turned(), grip=OPEN, label="over the handle's end"), Move(1.5, pos=hold, label="down"), Move(0.6, grip=HOLD, label="close"),
               Move(SLIDE_S, pos=slid, label="slides it round"), Move(0.4, label="round"), Move(0.6, grip=OPEN, label="open"), Move(1.0, pos=slid + up, label="up")]  # fmt: skip
        self.moved = np.array([ROUND, -SLIDE_AWAY, 0.0])
        if name == "round":
            yield [Move(1.5, label="away")]
            return
        if name == "pushed":  # the closed hand behind the bar's jaw pushes it towards the grip until the joint is closed
            jaw = self.on_clamp(((m.X_BAR - m.JAW_ROOT + m.TIP_X) / 2, m.BAR_PAD_FACE + m.PAD_T + m.JAW_T + 0.012, m.TOP + GRASP_Z), "bar")
            towards = self.pose("bar")[1] @ np.array([0.0, -1.0, 0.0])  # along the bar, towards the grip
            yield [Move(2.0, pos=jaw + up, rot=self.turned(), grip=SQUEEZE, label="over the bar's jaw"), Move(1.2, pos=jaw, label="down")]
            pushed = 0.0
            while pushed < PUSH_MOST and not self.closed():
                pushed += PUSH_STEP
                yield [Move(0.4, pos=jaw + pushed * towards, label="pushes the bar")]
            back = jaw + (pushed - 0.006) * towards
            yield [Move(0.6, pos=back, label="back off"), Move(1.2, pos=back + up, label="up"), Move(1.5, label="away")]
            return
        pump = self.at_handle(FINGER)
        yield [Move(1.2, pos=pump + up, rot=self.turned(), grip=WIDE, label="over the trigger"), Move(1.2, pos=pump, label="down to it"), Move(0.4, grip=OPEN, label="ready")]
        yield from self.pump(name)
        if name == "held":
            yield [Move(SQUEEZE_S, grip=SQUEEZE, label="closes on it"), Move(3.0, label="hand stays")]
            return
        yield [Move(0.6, grip=OPEN, label="open"), Move(1.2, pos=pump + np.array([0.0, 0.0, 0.08]), label="up"), Move(1.5, label="away")]

    def whole(self, name: str) -> list:
        return [move for step in self.steps(name) for move in step]


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
            print(f"DRY {run}: {json.dumps(kit.dry_run({'right': Plan(m).whole(run)}, station.BASES, limit_s=60.0), default=str)}", flush=True)
        return 0
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    failed = []
    for run in runs:
        try:
            plan = Plan(m, lambda: {"closed": m.closed(), "touching": touching(m), "opening": opening(m), "poses": {body: mk.pose(body) for body in ("grip", "bar")}})
            script = plan.steps(run)
            state = {"over": False}

            def director(t: float, arms: dict, pictures, script=script, state=state) -> None:
                if pictures is None or mk.MECH.environment is None or not mk.MECH.poses:
                    return
                arm = arms["right"]
                while not state["over"] and t >= arm.seconds - 1e-6:  # the next step is planned when the last is through
                    step = next(script, None)
                    if step is None:
                        state["over"] = True
                        break
                    arm.moves.extend(step)

            def extra(plan=plan) -> dict:
                g, b = m.pad_place("grip"), m.pad_place("bar")
                return {"gauge (N)": round(m.gauge(), 2), "bar drawn (mm)": round(1000 * mk.joint("draw"), 3), "trigger (deg)": round(math.degrees(mk.joint("trigger")), 2),
                        "pad gaps (mm)": [round(1000 * g["gap"], 3), round(1000 * b["gap"], 3)], "pads inside the ends (mm)": [round(1000 * g["inside"], 1), round(1000 * b["inside"], 1)],
                        "joint (mm)": [round(1000 * v, 3) for v in m.joint_gaps()], "round": m.round_boards(), "closed": m.closed(), "squeezed": m.STATE["squeezed"], "strokes": m.STATE["strokes"],
                        "level": m.level(), "hand": [round(float(v), 4) for v in mk.gripper("right")[0]], "grip": round(float(mk.gripper("right")[1]), 3), "at rest": m.at_rest(),
                        "hand from clamp and boards (mm)": round(1000 * min(m.distance_to(name, mk.gripper("right")[0]) for name in (*m.BOARDS, *m.CLAMP)), 1),
                        "grip moved (mm)": [round(1000 * float(v), 2) for v in mk.pose("grip")[0][:2] - mk.built_pose("grip")[0][:2]],
                        "clamp turned (deg)": round(math.degrees(math.atan2(mk.pose("grip")[1][1, 0], mk.pose("grip")[1][0, 0])), 2),
                        "speeds (mm/s)": {name: None if v is None else round(1000 * v, 2) for name, v in m.speeds().items()}}  # fmt: skip

            folder = out / run
            result = kit.run(embodiment, {"right": []}, folder, seed=0, director=director, extra=extra, limit_s=m.RUN_LIMIT_STEPS / hz, tail_s=1.0, sheet_frames=12)
            rows = result["rows"]
            scores = [row["info"].get("score", 0.0) for row in rows]
            firsts = {level: next((row["t"] for row, s in zip(rows, scores) if s >= level), None) for level in (0.25, 0.5, 0.75, 1.0)}
            last = rows[-1]["extra"]
            per_stroke, drawn_before = [], 0.0
            for k in range(1, last["strokes"] + 1):  # each stroke, read at its last control step (squeezed and let go again)
                of = [row["extra"] for row in rows if row["extra"]["strokes"] == k]
                end = of[-1]
                per_stroke.append({"stroke": k, "bar drawn (mm)": round(end["bar drawn (mm)"] - drawn_before, 3), "trigger most (deg)": max(e["trigger (deg)"] for e in of),
                                   "gauge most (N)": max(e["gauge (N)"] for e in of), "gauge after (N)": end["gauge (N)"], "closed after": end["closed"], "joint after (mm)": end["joint (mm)"]})  # fmt: skip
                drawn_before = end["bar drawn (mm)"]
            readings = {
                "score": scores[-1], "first reached": firsts, "seconds": rows[-1]["t"], "ended": result.get("termination_reason"),
                "strokes": last["strokes"], "gauge at the end (N)": last["gauge (N)"], "most on the gauge (N)": max(row["extra"]["gauge (N)"] for row in rows),
                "closed at (s)": next((row["t"] for row in rows if row["extra"]["closed"]), None),
                "hand from clamp and boards at the end (mm), at rest": [last["hand from clamp and boards (mm)"], last["at rest"]],
                "per stroke": per_stroke, "report": result["report"], "worst IK": result["worst_ik"],
            }  # fmt: skip
            (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
            print(f"DEMO {run}: score {scores[-1]:.2f} success {result['terminated'] and result['termination_reason'] == 'success'} after {result['seconds']:.1f} s ({result['steps']} steps of {m.RUN_LIMIT_STEPS}); "
                  f"first reached {firsts}; report {result['report']}", flush=True)  # fmt: skip
            print(f"READINGS {run}: {json.dumps(readings, default=str)}", flush=True)
        except Exception as error:
            import traceback

            traceback.print_exc()
            print(f"RUN {run} FAILED: {error!r}", flush=True)
            failed.append(run)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
