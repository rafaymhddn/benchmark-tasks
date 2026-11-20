#!/usr/bin/env python3
"""Reference runs of the servo horn task: the full score, the horn pressed half way, put on as it was picked up, seated
one tooth off, seated wrong and refitted, rested on the crests on the mark, and the hand that stays.

    scripts/rta demo <id> mech/tasks/servo_horn [run,run,...] [--quick]

The right hand, pointing straight down and open, comes down over the horn's hub, turned so that its fingers close
across the hub (square to the arm), and closes on it with its grasp point 15 mm over the bench (the fingertips' ends
2 mm over it). It lifts the horn 6 cm, carries it over the shaft, turning it so that its arm lies on the mark (in
``wrong_tooth`` and ``refit`` one tooth, 14.4 degrees, past it; in ``as_picked`` not at all), and lowers it onto the
crests. There the hand turns the horn by what its arm is off where it should be (within half a degree, at most three
turns) and moves it sideways until the hub's axis is on the shaft's (within 0.03 mm, at most six moves), twice. It
lets go slowly (1.2 s), rises 15 mm, closes, comes down onto the hub's top and presses it down along the shaft with
its fingertips: 40 N asked for at first, 2 N more every tenth of a second up to 50 N, until the horn is 3.3 mm down
(``half_way``: 31.5 N at first, 0.1 N more every 0.2 s, until it is 1.2 mm down). Home, it holds 0.3 s, eases off,
opens, goes up 8 cm and stays there. In ``refit`` the hand
first opens round the seated hub, closes on it and pulls it straight up off the shaft (asked to go 8 mm over the horn's
place on the crests, and pulling up 2 N more than that, 1 N more every tenth of a second, until the hub's mouth is 2 mm
clear of it); then it
stops pulling and goes 3 cm up, turns the horn onto the mark, lets the arm steady 4 mm over the horn's place and centres
it there, puts it on and presses it home again the same way.

What the script reads from the simulator: how far the horn is down on the shaft, how far its arm is from where it
should be and how far its axis is from the shaft's (a person sees the arm against the mark and the hub on the shaft,
and feels the horn go down and stop), and nothing else: not whether the horn is held by the splines, and not the score.

Runs. ``full``: 1.0. ``half_way``: the press stopped 1.2 mm down (0.5). ``as_picked``: put on the crests without turning, let go
(0). ``wrong_tooth``: pressed home one tooth past the mark (0). ``refit``: pressed home one tooth past, pulled off,
refitted on the mark (1.0). ``rest``: on the crests on the mark, let go without a press (0.25). ``held``: as
``full``, but the closed hand stays on the hub's top (0.75). Checks, not scored levels: ``grip_check``, the hand
closed on the hub presses 34, 37 and 40 N, 2 s each, with the arm 4 degrees past the mark (off any tooth position, so
that the crests do not give); ``top_check``, the same pressed on the hub's top by the closed hand after letting go.

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

RUNS = ("full", "half_way", "as_picked", "wrong_tooth", "refit", "rest", "held", "grip_check", "top_check")
GRASP_Z = 0.015  # m the grasp point over the hub's mouth: the fingertips' ends 2 mm over it
OPEN, HOLD = 0.45, 0.10  # the hand's opening (0 closed .. 1 open, 94 mm): 42 mm over the hub; closed on its 14 mm, pressing
LIFT, OVER = 0.06, 0.08  # m: the horn lifted off the bench; the hand over the shaft before and after
ABOVE = 0.004  # m over its place on the crests where the horn is brought before it is lowered onto them
SINK = 0.0005  # m under its place on the crests that the hand is asked to: the horn lies on them, lightly pressed
TURNED = {"as_picked": None, "wrong_tooth": 1, "refit": 1}  # teeth past the mark that the horn is first put on (None: not turned); the others: 0
LINED, LINE_TRIES = math.radians(0.5), 3  # the arm within this of where it should be, or this many turns of the hand
CENTRED, CENTRE_TRIES = 0.00003, 6  # m the hub's axis from the shaft's before the hand lets go, or this many moves of the hand (each given time to settle: the arm follows with a lag)
RELEASE_S = 1.2  # s the fingers take to open off the horn: let go at once, a horn hanging on the crests' chamfer dropped and tipped (run c3k)
FIRST_N, MORE_N, MOST_N = 40.0, 2.0, 50.0  # N asked for at first; more every look; at most
STEP_S, WAIT_S = 0.1, 2.0  # s between two looks at the horn; s the hand goes on pressing with the most before it gives up
HALF_FIRST_N, HALF_MORE_N, HALF_STEP_S = 31.5, 0.1, 0.2  # half_way: from just under the crests' 32 N, 0.1 N more every 0.2 s, so that the
# horn creeps and can be stopped part way (from 40 N the crests went from 0.2 to 3.46 mm down in 0.1 s: recording c3z)
HOME, HALF = 0.0033, 0.0012  # m down: home; where half_way stops pressing
PULL = 0.008  # m over the horn's place on the crests where the hand's grasp point is asked to go, closed, to pull the horn off
PULL_FIRST_N, PULL_MORE_N, PULL_MOST_N = 2.0, 1.0, 20.0  # refit: N pulled up along the shaft on top of that at first, more every look, at most (a
# seated horn comes up under 10 N); the pull stops when the horn's mouth is CLEAR over its place: let the arm's pull go on while the hand
# turned towards the mark, and the horn, still round the crests, was torn out of the fingers (recording c3zf)
CLEAR = 0.002  # m over its place on the crests: the horn's mouth clear of them (they stand 1.4 mm up into the bore)
PULLED_UP = 0.03  # m over the horn's place on the crests where the hand's grasp point goes when it stops pulling: asked to stay at PULL, the
# arm sagged 5 mm and set the horn back on the crests (recording c3zh)
DRY_STEPS = 60
GRIP_OFF = math.radians(4.0)  # grip_check, top_check: the arm this far past the mark, off any tooth position, so that the crests do not give
GRIP_N = (34.0, 37.0, 40.0)  # N asked for in grip_check and top_check, 2 s each: the crests' stiff spring then reads the press
TIP = 0.0129  # m the closed hand's fingertips reach past the grasp point
TOUCH = 0.0005  # m over the hub's top (of a horn resting on the crests) where the closed hand is brought before it presses
OFF_UP = 0.015  # m the open hand rises off the horn before it closes


class Plan:
    def __init__(self, m, see=None) -> None:
        import demo_kit as kit

        self.m, self.see, self.kit, self.Move = m, see, kit, kit.Move
        self.asked = 0.0  # N the hand is asked to press with now
        self.yaw = 0.0  # the hand's turn about the vertical now (rad)
        self.shift = np.zeros(3)  # m the hand has moved its horn sideways to centre it on the shaft

    def horn_yaw(self) -> float:
        """The horn's arm's direction at the start, as a turn about the vertical (rad)."""
        m = self.m
        start = np.array(m.mk.axis_angle((0.0, 0.0, 1.0), m.HORN_START)) @ m.MARK
        return math.atan2(start[1], start[0])

    def mark_yaw(self) -> float:
        """The turn of the hand that puts the arm on the mark, the shortest way from where it picked the horn."""
        m = self.m
        target = math.atan2(m.MARK[1], m.MARK[0])
        return self.horn_yaw() + math.remainder(target - self.horn_yaw(), 2 * math.pi)

    def press(self, name: str, depth: float):
        """Press the horn down with the closed hand's fingertips on the hub's top until it is ``depth`` down, or until
        the most has been asked for WAIT_S. In ``half_way`` the press starts just under 32 N and grows slowly."""
        Move = self.Move
        first, more, every = (HALF_FIRST_N, HALF_MORE_N, HALF_STEP_S) if name == "half_way" else (FIRST_N, MORE_N, STEP_S)
        force = self.asked = first
        yield [Move(0.5, push=(0.0, 0.0, -force), push_at=(0.0, 0.0, TIP), label="press")]
        waited = 0.0
        for k in range(10_000):
            if self.see is not None:
                if self.see()["down"] >= depth:
                    break
            elif k >= DRY_STEPS:
                break
            if force < MOST_N - 1e-9:
                force = min(force + more, MOST_N)
            else:
                waited += every
                if waited > WAIT_S:  # it does not go: the run goes on as it stands
                    break
            self.asked = force
            yield [Move(every, push=(0.0, 0.0, -force), label="press harder")]

    def pull_off(self):
        """Pull the closed hand straight up along the shaft, harder every look, until the horn's mouth is CLEAR over its
        place on the crests; then stop pulling and take the hand PULLED_UP over that place."""
        m, Move = self.m, self.Move
        over = np.array([*m.SHAFT_AT, m.TOP + m.REST_Z + GRASP_Z])
        force = PULL_FIRST_N
        for k in range(10_000):
            if self.see is not None:
                if self.see()["down"] <= -CLEAR:
                    break
            elif k >= DRY_STEPS:
                break
            if force >= PULL_MOST_N:
                break  # it does not come: the run goes on as it stands
            yield [Move(STEP_S, push=(0.0, 0.0, force), label="pulls harder")]
            force = min(force + PULL_MORE_N, PULL_MOST_N)
        yield [Move(0.8, pos=over + np.array([0.0, 0.0, PULLED_UP]), push=(0.0, 0.0, 0.0), label="off")]

    def onto_top(self):
        """The hand lets go of the horn on the crests, rises, closes and comes down onto the hub's top: the grip on the
        hub does not carry the press (grip_check: the horn tips in the fingers at about 15 N, run c3j)."""
        m, Move = self.m, self.Move
        over = np.array([*m.SHAFT_AT, m.TOP + m.REST_Z + GRASP_Z])
        top = np.array([*m.SHAFT_AT, m.TOP + m.REST_Z + m.HUB_H + TIP + TOUCH])
        yield [Move(RELEASE_S, grip=OPEN, label="lets go"), Move(0.8, pos=over + self.shift + np.array([0.0, 0.0, OFF_UP]), label="up off it"), Move(0.5, grip=0.0, label="closes"),
               Move(1.0, pos=top, label="onto the hub"), Move(0.3, label="on it")]  # fmt: skip

    def onto_crests(self, name: str, teeth: int | None, past: float = 0.0, centre_above: bool = False):
        """Carry the held horn over the shaft, turned ``teeth`` tooth positions and ``past`` rad past the mark (None:
        as picked), lower it onto the crests and turn it until its arm is where it should be. ``centre_above``: first
        let the arm steady and centre the horn over the shaft ABOVE its place, then lower it (the second fit of
        ``refit``: after the pull the arm still swung, and the horn came down 4 mm off the axis, recording c3zk)."""
        m, Move, down = self.m, self.Move, self.kit.down
        over = np.array([*m.SHAFT_AT, m.TOP + m.REST_Z + GRASP_Z])
        self.yaw = self.horn_yaw() if teeth is None else self.mark_yaw() + teeth * m.TOOTH + past
        self.shift = np.zeros(3)
        carry = [Move(2.0, pos=over + np.array([0.0, 0.0, LIFT]), rot=down(self.yaw), label="over the shaft"), Move(1.5, pos=over + np.array([0.0, 0.0, ABOVE]), label="down to it")]
        if not centre_above:
            yield carry + [Move(0.8, pos=over - np.array([0.0, 0.0, SINK]), label="onto the crests"), Move(0.3, label="on it")]
        else:
            yield carry + [Move(1.0, label="steadies")]
            yield from self.centre_it(over, ABOVE, "centres it above")
            yield [Move(0.8, pos=over + self.shift - np.array([0.0, 0.0, SINK]), label="onto the crests"), Move(0.3, label="on it")]
        for _ in range(2):  # turn it onto where it should be and centre it on the shaft; once more, as each can upset the other
            yield from self.turn_it(teeth, past)
            yield from self.centre_it(over)

    def turn_it(self, teeth: int | None, past: float):
        """Turn the held horn until its arm is where it should be (at most LINE_TRIES turns of the hand)."""
        m, Move, down = self.m, self.Move, self.kit.down
        for _ in range(LINE_TRIES if teeth is not None else 0):
            if self.see is None:
                break
            off = math.remainder(self.see()["arm"] - teeth * m.TOOTH - past, 2 * math.pi)
            if abs(off) <= LINED:
                break
            self.yaw -= off
            yield [Move(0.4, rot=down(self.yaw), label="turns it"), Move(0.3, label="on it")]

    def centre_it(self, over: np.ndarray, height: float = -SINK, label: str = "centres it"):
        """Move the held horn sideways until the hub's axis is on the shaft's, so that it goes down onto the crests and
        stays there when let go: off by more than the bore's play (0.075 mm) it hangs on the crests' chamfer, and let go
        there it drops and tilts (run c3m: 0.13 mm off, 0.44 mm high, 12 degrees when let go). ``height``: the grasp
        point's height over ``over`` while it moves (on the crests, lightly pressed; or ABOVE them)."""
        Move = self.Move
        for _ in range(CENTRE_TRIES):
            if self.see is None:
                break
            aside = self.see()["aside"]
            if float(np.linalg.norm(aside)) <= CENTRED:
                break
            self.shift = self.shift - np.array([aside[0], aside[1], 0.0])
            yield [Move(0.4, pos=over + self.shift + np.array([0.0, 0.0, height]), label=label), Move(0.5, label="on it")]

    def steps(self, name: str):
        """Run ``name`` as steps, each planned when it is asked for."""
        m, Move, down = self.m, self.Move, self.kit.down
        hub = np.array([*m.HORN_XY, m.TOP + GRASP_Z])
        up = np.array([0.0, 0.0, OVER])
        yield [Move(2.5, pos=hub + up, rot=down(self.horn_yaw()), grip=OPEN, label="over the horn"), Move(1.5, pos=hub, label="down"), Move(0.6, grip=HOLD, label="close"),
               Move(1.2, pos=hub + np.array([0.0, 0.0, LIFT]), label="lift")]  # fmt: skip
        self.yaw = self.horn_yaw()
        teeth = TURNED.get(name, 0)
        yield from self.onto_crests(name, teeth, GRIP_OFF if name in ("grip_check", "top_check") else 0.0)
        over = np.array([*m.SHAFT_AT, m.TOP + m.REST_Z + GRASP_Z])
        if name == "grip_check":  # the spec's check: does the grip on the hub carry the press? The crests do not give off a tooth position
            for force in GRIP_N:
                self.asked = force
                yield [Move(2.0, push=(0.0, 0.0, -force), label=f"press {force:g} N")]
        if name == "top_check":  # the same with the closed hand on the hub's top
            yield from self.onto_top()
            for force in GRIP_N:
                self.asked = force
                yield [Move(2.0, push=(0.0, 0.0, -force), push_at=(0.0, 0.0, TIP), label=f"press {force:g} N")]
        if name in ("full", "held", "half_way", "wrong_tooth", "refit"):
            yield from self.onto_top()
            yield from self.press(name, HALF if name == "half_way" else HOME)
            if name in ("full", "held", "wrong_tooth", "refit"):
                yield [Move(0.3, label="hold")]
        if name == "refit":  # pull it off by the hub, turn it onto the mark, put it on, press it home again
            self.asked = 0.0
            seated = over - np.array([0.0, 0.0, m.TRAVEL])
            yield [Move(0.3, push=(0.0, 0.0, 0.0), label="ease"), Move(0.8, pos=seated + np.array([0.0, 0.0, OFF_UP]), grip=OPEN, label="up off it"),
                   Move(1.0, pos=seated, label="round the hub"), Move(0.6, grip=HOLD, label="close"), Move(1.0, pos=over + np.array([0.0, 0.0, PULL]), label="pull off")]  # fmt: skip
            yield from self.pull_off()
            yield from self.onto_crests(name, 0, centre_above=True)
            yield from self.onto_top()
            yield from self.press(name, HOME)
            yield [Move(0.3, label="hold")]
        if name == "held":
            yield [Move(3.0, label="hand stays")]
            return
        self.asked = 0.0
        yield [Move(0.3, push=(0.0, 0.0, 0.0), label="ease"), Move(0.6, grip=OPEN, label="open"), Move(1.2, pos=over + up, label="up"), Move(1.5, label="away")]

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
            plan = Plan(m, lambda: {"down": m.down(), "arm": m.arm_angle(), "aside": mk.pose("horn")[0][:2] - m.SHAFT_AT})
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
                return {"down (mm)": round(1000 * m.down(), 3), "arm from the mark (deg)": round(math.degrees(m.arm_angle()), 2), "off the shaft (mm)": round(1000 * m.off_axis(), 3),
                        "tilt (deg)": round(math.degrees(m.tilt()), 2), "on the crests": m.captured(), "engaged": m.engaged(), "seat (mm)": round(1000 * mk.joint("down"), 3), "level": m.level(),
                        "hand to horn (mm)": round(1000 * m.to_horn(mk.gripper("right")[0]), 1), "hand": [round(float(v), 4) for v in mk.gripper("right")[0]],
                        "grip": round(float(mk.gripper("right")[1]), 3), "at rest": m.at_rest(), "asked (N)": plan.asked, "hand yaw (deg)": round(math.degrees(plan.yaw), 2),
                        "grasp point over the mouth (mm)": round(1000 * float(mk.gripper("right")[0][2] - mk.pose("horn")[0][2]), 3),
                        "horn speed (mm/s), arm tip (mm/s)": [round(1000 * float(np.linalg.norm(mk.velocity("horn")[0])), 2), round(1000 * float(np.linalg.norm(mk.velocity("horn")[1])) * m.ARM_L, 2)],
                        "seat speed (mm/s)": round(1000 * float(np.linalg.norm(mk.velocity("seat")[0])), 2)}  # fmt: skip

            folder = out / run
            result = kit.run(embodiment, {"right": []}, folder, seed=0, director=director, extra=extra, limit_s=m.RUN_LIMIT_STEPS / hz, tail_s=1.0, sheet_frames=12,
                             crop={"top_cam": (300, 120, 460, 240)})  # fmt: skip
            rows = result["rows"]
            scores = [row["info"].get("score", 0.0) for row in rows]
            firsts = {level: next((row["t"] for row, s in zip(rows, scores) if s >= level), None) for level in (0.25, 0.5, 0.75, 1.0)}
            last = rows[-1]["extra"]
            began = next((row for row in rows if row["extra"]["down (mm)"] >= 0.3 and row["extra"]["off the shaft (mm)"] <= 1.0), None)
            pressing = [row for row in rows if row["right"] in ("press", "press harder", "hold", "hand stays")]
            readings = {
                "score": scores[-1], "first reached": firsts, "seconds": rows[-1]["t"], "ended": result.get("termination_reason"),
                "at the end: down (mm), arm from the mark (deg), off the shaft (mm), tilt (deg), on the crests, engaged": [last["down (mm)"], last["arm from the mark (deg)"], last["off the shaft (mm)"], last["tilt (deg)"], last["on the crests"], last["engaged"]],
                "the horn began to go down at (s), push asked for then (N)": [None if began is None else began["t"], None if began is None else began["extra"]["asked (N)"]],
                "push asked for at most (N)": max(row["extra"]["asked (N)"] for row in rows),
                "s of pressing": round(len(pressing) / hz, 2),
                "hand from the horn at the end (mm), at rest": [last["hand to horn (mm)"], last["at rest"]],
                "report": result["report"], "worst IK": result["worst_ik"],
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
