#!/usr/bin/env python3
"""Reference runs of the shield task: the full score, the two lower levels as the same press cut short, the two
presses that bend the pins, and the hand that stays.

    scripts/rta demo <id> mech/tasks/shield_seat [run,run,...] [--quick]

The right hand, pointing straight down and open 44 mm so that a fingertip's end (3.9 mm thick) stands on each row
of the shield's own sockets, comes down over the middle of the 32 pins and presses: 40 N asked for at first, then
5 N more every second up to 52 N and 2 N more every second from there (the one-sided presses: from 28 and 21 N). When the four strips' resistances (53.4 N in
all) are overcome the shield goes down: the arm is a soft spring (its press falls 0.7 N for every millimetre the hand
goes down), so the shield goes about 1.4 mm for every newton more that is asked for. Home, the hand presses 5 N
harder for half a second, as one does to feel it bottom out, eases off, goes up and stays away.

What the script reads from the simulator: how far the shield has gone down at its four strips (a person sees the
board come down and feels it stop), and nothing else: not whether a pin has bent and not the score. ``full`` and
``held`` press until every strip is home; ``quarter`` and ``half`` let go when every strip is 2.2 mm and 4.2 mm in
(the shield goes about half a millimetre further while the hand lets go);
``one_end`` and ``one_row`` start at 15 N and press until the side they press on is 3 mm in or 45 N are asked for.

Runs. ``full``: 1.0. ``half``: let go 4.2 mm in, hand away (0.5). ``quarter``: let go 2.2 mm in (0.25).
``one_end``: both fingertips on the rows' ends away from the USB socket: that end goes in, the other stands, a pin
bends (0). ``one_row``: one fingertip on the power and analog row, the other in the air beside the carrier: that row
goes in alone (0). ``held``: as ``full``, but the hand stays on the seated shield (0.75).

``--quick``: every run's plan against the arm's reach and joint limits, without the simulator.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

RUNS = ("full", "half", "quarter", "one_end", "one_row", "held")
OPEN = 0.472  # the hand's opening (0 closed .. 1 open): 44.4 mm between the pads, a fingertip's end on each row of sockets
TIP = 0.0129  # m the fingertips' ends reach past the grasp point
OVER = 0.08  # m over the shield before and after
FIRST_N = {"full": 40.0, "held": 40.0, "half": 40.0, "quarter": 40.0, "one_end": 15.0, "one_row": 15.0}  # N asked for at first: under what makes anything give
MORE_S = 0.1  # s between two looks at the shield
FAST_N, SLOW_N = 0.5, 0.2  # N more every MORE_S: 5 N a second up to NEAR_N, 2 N a second from there
NEAR_N = {"full": 52.0, "held": 52.0, "half": 52.0, "quarter": 52.0, "one_end": 28.0, "one_row": 21.0}  # N: a little under what makes the pressed strips go
HOME_N = 5.0  # N more, for half a second, once every strip is home: onto the stops
MOST_N = {"full": 70.0, "held": 70.0, "half": 70.0, "quarter": 70.0, "one_end": 45.0, "one_row": 45.0}  # N asked for at most
WAIT_S = 3.0  # s the hand goes on pressing with the most before it gives up
STOP_IN = {"quarter": 0.0022, "half": 0.0042}  # m every strip is in when the press is cut short (the shield goes about half a millimetre further while the hand lets go)
SIDE_IN = 0.003  # m the pressed side is in when a one-sided press is stopped
END_X_MM = 58.0  # along the board (mm in Arduino's drawing): the rows' ends away from the USB socket
DRY_STEPS = 130  # press steps of a plan without the simulator: up to the most asked for, and on


class Plan:
    def __init__(self, m, see=None) -> None:
        import demo_kit as kit

        self.m, self.see, self.kit, self.Move = m, see, kit, kit.Move
        self.asked = 0.0  # N the hand is asked to press with now

    def middle_of_pins(self, names) -> np.ndarray:
        m = self.m
        return np.array([sum(m.PINS[n] * m.strip_place(n)[0] for n in names) / sum(m.PINS[n] for n in names), m.CARRIER_XY[1]])

    def over(self, name: str) -> np.ndarray:
        """Where the hand's middle stands on the bench in run ``name``."""
        m = self.m
        if name == "one_end":
            return np.array([m.on_bench(END_X_MM, 0.0)[0], m.CARRIER_XY[1]])
        if name == "one_row":  # the left fingertip on the power and analog row, the right one beyond the carrier's edge
            at = self.middle_of_pins(("power", "analog"))
            return np.array([at[0], m.strip_place("power")[1] - m.ROWS_APART / 2])
        return self.middle_of_pins(m.NAMES)

    def pressed(self, name: str) -> tuple:
        """The strips under the fingertips in run ``name``."""
        return {"one_end": ("digital_low", "analog"), "one_row": ("power", "analog")}.get(name, tuple(self.m.NAMES))

    def reached(self, name: str, deep: dict) -> bool:
        m = self.m
        if name in STOP_IN:
            return min(deep.values()) >= STOP_IN[name]
        if name in ("one_end", "one_row"):
            return max(deep[n] for n in self.pressed(name)) >= SIDE_IN
        return all(m.MATING - v <= m.SEATED for v in deep.values())

    def steps(self, name: str):
        """Run ``name`` as steps, each planned when it is asked for."""
        m, Move, down = self.m, self.Move, self.kit.down
        at = self.over(name)
        touch = np.array([at[0], at[1], m.TOP + m.SHIELD_Z + m.PCB_T / 2 + m.SOCKET_H + TIP])
        up = np.array([0.0, 0.0, OVER])
        tips = np.array([0.0, 0.0, TIP])
        yield [Move(2.0, pos=touch + up, rot=down(0.0), grip=OPEN, label="over the shield"), Move(1.5, pos=touch + np.array([0.0, 0.0, 0.001]), label="down to it"), Move(0.3, label="on it")]
        force = self.asked = FIRST_N[name]
        yield [Move(1.0, push=(0.0, 0.0, -force), push_at=tips, label="press")]
        waited = 0.0
        for k in range(10_000):
            if self.see is not None:
                if self.reached(name, self.see()):
                    break
            elif k >= DRY_STEPS:
                break
            if force < MOST_N[name] - 1e-9:
                slow = force >= NEAR_N[name] - 1e-9
                force = min(force + (SLOW_N if slow else FAST_N), MOST_N[name])
            else:
                waited += MORE_S
                if waited > WAIT_S:  # it does not go: the run ends as it stands
                    break
            self.asked = force
            yield [Move(MORE_S, push=(0.0, 0.0, -force), label="press harder")]
        if name in ("full", "held"):  # onto the stops
            force = self.asked = force + HOME_N
            yield [Move(0.5, push=(0.0, 0.0, -force), label="press home")]
        if name == "held":
            yield [Move(3.0, label="hand stays")]
            return
        if name == "full":
            yield [Move(0.3, label="hold")]
        self.asked = 0.0  # a press cut short lets go at once: under a press above the resistance the shield goes on down
        yield [Move(0.5 if name == "full" else 0.2, push=(0.0, 0.0, 0.0), label="ease"), Move(1.2, pos=touch + up, label="up"), Move(1.5, label="away")]

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
        plan = Plan(m)
        for run in runs:
            print(f"DRY {run}: {json.dumps(kit.dry_run({'right': plan.whole(run)}, station.BASES, limit_s=60.0), default=str)}", flush=True)
        return 0
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk
    plan = Plan(m, m.depths)
    hz = float(embodiment.info.control_hz)
    failed = []
    for run in runs:
        try:
            script = plan.steps(run)
            state = {"over": False}

            def director(t: float, arms: dict, pictures, script=script, state=state) -> None:
                if pictures is None or mk.MECH.environment is None or not mk.MECH.poses:
                    return
                arm = arms["right"]
                while not state["over"] and t >= arm.seconds - 1e-6:  # the next step is planned when the last is through: from where the shield stands then
                    step = next(script, None)
                    if step is None:
                        state["over"] = True
                        break
                    arm.moves.extend(step)

            def extra() -> dict:
                deep = m.depths()
                return {"in (mm)": {k: round(1000 * v, 3) for k, v in deep.items()}, "seats (mm)": {n: round(1000 * mk.joint(f"in_{n}"), 3) for n in m.NAMES},
                        "tilt (deg)": [round(float(np.degrees(mk.joint("shield_about_x"))), 2), round(float(np.degrees(mk.joint("shield_about_y"))), 2)],
                        "bent": m.STATE["bent"] is not None, "hand to shield (mm)": round(1000 * m.to_shield(mk.gripper("right")[0]), 1),
                        "hand": [round(float(v), 4) for v in mk.gripper("right")[0]], "at rest": m.at_rest(), "asked (N)": plan.asked}  # fmt: skip

            folder = out / run
            result = kit.run(embodiment, {"right": []}, folder, seed=0, director=director, extra=extra, limit_s=m.RUN_LIMIT_STEPS / hz, tail_s=1.0, sheet_frames=12,
                             crop={"top_cam": (320, 150, 480, 270)})  # fmt: skip
            rows = result["rows"]
            scores = [row["info"].get("score", 0.0) for row in rows]
            firsts = {level: next((row["t"] for row, s in zip(rows, scores) if s >= level), None) for level in (0.25, 0.5, 0.75, 1.0)}
            moved = next((row for row in rows if max(row["extra"]["in (mm)"].values()) >= 1.0), None)
            home = next((row for row in rows if min(row["extra"]["in (mm)"].values()) >= 1000 * (m.MATING - m.SEATED)), None)
            last = rows[-1]["extra"]
            pressing = [row for row in rows if row["right"] in ("press", "press harder", "press home", "hold", "hand stays")]
            readings = {
                "score": scores[-1], "first reached": firsts, "seconds": rows[-1]["t"], "ended": result.get("termination_reason"),
                "strips in at the end (mm)": last["in (mm)"], "seats at the end (mm from their start)": last["seats (mm)"],
                "bent": result["report"].get("bent"),
                "most any two strips were apart in height (mm)": result["report"].get("most any two strips were apart in height (mm)"),
                "most the shield was tilted (deg): about x, about y": [max(abs(row["extra"]["tilt (deg)"][k]) for row in rows) for k in (0, 1)],
                "the shield began to go down at (s), was home at (s)": [None if moved is None else moved["t"], None if home is None else home["t"]],
                "push asked for when it began to go down, when it was home, at most (N)": [None if moved is None else moved["extra"]["asked (N)"], None if home is None else home["extra"]["asked (N)"], max(row["extra"]["asked (N)"] for row in rows)],
                "s of pressing": round(len(pressing) / hz, 2),
                "hand from the shield at the end (mm), at rest": [last["hand to shield (mm)"], last["at rest"]],
                "strips in at their least while the hand pressed (mm)": None if not pressing else min(min(row["extra"]["in (mm)"].values()) for row in pressing),
                "strips in when the pressing ended (mm)": None if not pressing else pressing[-1]["extra"]["in (mm)"],
                "worst IK": result["worst_ik"],
            }
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
