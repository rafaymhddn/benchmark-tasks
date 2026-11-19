#!/usr/bin/env python3
"""Reference runs of the gantry task: the full score, the two lower levels as the same push cut short, the push at
one end that binds, and the hands that stay.

    scripts/rta demo <id> mech/tasks/gantry_square [run,run,...] [--quick]

Both hands, closed and pointing down, come down behind the beam, one over each carriage, each 6 mm behind the
beam's near face where it stands (the beam is racked, so the right hand starts 2.7 mm ahead of the left). They go
forward together, 25 mm a second, and each slows to 5 mm a second when its carriage is within 8 mm of its stop. A
hand stops going forward when its carriage touches its stop; the other goes on until its own does. Then both go
back, up and away.

What the script reads from the simulator: how far each carriage is from its stop (a person sees the gap close), to
slow down and to stop; nothing else: not the score.

Runs. ``full``: 1.0. ``short``: both hands stop when both carriages are 30 mm nearer their stops (0.25). ``near``:
they stop when both are within 3 mm (0.5). ``one_end``: the right hand alone pushes its end 9 cm forward: the
beam turns in its play and binds; the readings say how far each carriage got. ``held``: as ``full``, but the hands
stay on the beam (0.75).

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

RUNS = ("full", "short", "near", "one_end", "held")
FINGER = 0.0039  # m a fingertip is thick: the closed hand's outer faces stand that far either side of the grasp point
HAND_Z = 0.042  # m the grasp point over the bench while it pushes: the fingertips' ends 29 mm up, 6 mm over a carriage's top
BEHIND = 0.006  # m behind the beam's near face when a hand comes down
OVER = 0.08  # m over the pushing height before and after
STEP_S = 0.1  # s between two looks at the carriages
FAST, SLOW, SLOW_WITHIN = 0.025, 0.005, 0.008  # m/s forward; m/s once the carriage is within SLOW_WITHIN of its stop
PAST = 0.004  # m a hand is asked to go at most past where its carriage would touch: it presses the carriage onto the stop, a few newtons
STOP_AT = {"short": ("nearer", 0.030), "near": ("within", 0.003)}
ONE_END_M = 0.090  # m the right hand goes forward in one_end
YAW = {"left": -math.pi / 2, "right": math.pi / 2}  # the fingers close along the rails: a closed hand's broad face meets the beam
DRY_STEPS = 60


class Plan:
    def __init__(self, m, see=None) -> None:
        import demo_kit as kit

        self.m, self.see, self.kit, self.Move = m, see, kit, kit.Move
        self.asked = {side: 0.0 for side in m.SIDES}  # m each hand has been asked forward

    def behind(self, side: str) -> np.ndarray:
        """Where a hand's grasp point stands when it has come down behind the beam, over its carriage."""
        m = self.m
        y = m.RAIL_Y[side]
        face = m.START_X - m.BEAM_S / 2 - y * math.sin(m.START_YAW)  # the beam's near face there, at the racked start
        return np.array([face - FINGER - BEHIND, y, m.TOP + HAND_Z])

    def hands(self, name: str) -> tuple:
        return ("right",) if name == "one_end" else tuple(self.m.SIDES)

    def steps(self, name: str):
        """Run ``name`` as steps, each a dict of moves for the hands that work, planned when it is asked for."""
        m, Move, down = self.m, self.Move, self.kit.down
        hands = self.hands(name)
        up = np.array([0.0, 0.0, OVER])
        start = {side: self.behind(side) for side in hands}
        yield {side: [Move(2.5, pos=start[side] + up, rot=down(YAW[side]), grip=0.0, label="over the beam's end"), Move(1.5, pos=start[side], label="down behind it"), Move(0.3, label="there")] for side in hands}
        most = {side: BEHIND + m.START_GAP[side] + PAST for side in hands}  # as far as a hand is ever asked forward
        for k in range(10_000):
            if self.see is not None:
                gap, wide = self.see()
                if name in STOP_AT:
                    kind, by = STOP_AT[name]
                    if all((m.START_GAP[s] - gap[s] >= by) if kind == "nearer" else (gap[s] <= by) for s in m.SIDES):
                        break
                touching = {side: wide[side] <= m.FEELER for side in m.SIDES}
                if name != "one_end" and all(touching.values()):
                    break
            else:
                gap = {side: max(0.0, m.START_GAP[side] + BEHIND - self.asked[side]) for side in m.SIDES}
                touching = dict.fromkeys(m.SIDES, False)
                if k >= DRY_STEPS:
                    break
            if name == "one_end":
                if self.asked["right"] >= ONE_END_M - 1e-9:
                    break
                self.asked["right"] = min(ONE_END_M, self.asked["right"] + FAST * STEP_S)
            else:
                moved = False
                for side in hands:
                    if not touching[side] and self.asked[side] < most[side] - 1e-9:
                        self.asked[side] = min(most[side], self.asked[side] + (SLOW if gap[side] <= SLOW_WITHIN else FAST) * STEP_S)
                        moved = True
                if not moved:  # both as far as they are ever asked and not both touching: the run ends as it stands
                    break
            yield {side: [Move(STEP_S, pos=start[side] + np.array([self.asked[side], 0.0, 0.0]), label="push")] for side in hands}
        if name == "held":
            yield {side: [Move(3.0, label="hands stay")] for side in hands}
            return
        yield {side: [Move(0.4, label="hold")] for side in hands}
        if self.see is not None:  # a hand that was held back (the beam bound) backs off from where it is, not from where it was asked to be
            gap, _ = self.see()
            for side in hands:
                self.asked[side] = min(self.asked[side], BEHIND + m.START_GAP[side] - gap[side])
        back = {side: start[side] + np.array([self.asked[side] - 0.020, 0.0, 0.0]) for side in hands}
        yield {side: [Move(0.6, pos=back[side], label="back"), Move(1.2, pos=back[side] + up, label="up"), Move(1.5, label="away")] for side in hands}

    def whole(self, name: str) -> dict:
        out: dict = {side: [] for side in self.hands(name)}
        for step in self.steps(name):
            for side, moves in step.items():
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
            print(f"DRY {run}: {json.dumps(kit.dry_run(Plan(m).whole(run), station.BASES, limit_s=60.0), default=str)}", flush=True)
        return 0
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    failed = []
    for run in runs:
        try:
            plan = Plan(m, lambda: (m.gaps(), m.openings()))
            script = plan.steps(run)
            hands = plan.hands(run)
            state = {"over": False}

            def director(t: float, arms: dict, pictures, script=script, state=state, hands=hands) -> None:
                if pictures is None or mk.MECH.environment is None or not mk.MECH.poses:
                    return
                while not state["over"] and all(t >= arms[side].seconds - 1e-6 for side in hands):  # the next step is planned when the last is through: from where the carriages stand then
                    step = next(script, None)
                    if step is None:
                        state["over"] = True
                        break
                    for side, moves in step.items():
                        arms[side].moves.extend(moves)

            def extra(plan=plan) -> dict:
                position, _ = mk.pose("gantry")
                return {"gap (mm)": {s: round(1000 * v, 3) for s, v in m.gaps().items()}, "opening (mm)": {s: round(1000 * v, 3) for s, v in m.openings().items()},
                        "turned (deg)": round(float(np.degrees(m.yaw())), 3), "up (mm)": round(1000 * float(position[2] - m.TOP - m.BEAM_Z), 3), "aside (mm)": round(1000 * float(position[1]), 3),
                        "hand to gantry (mm)": {s: round(1000 * m.to_gantry(mk.gripper(s)[0]), 1) for s in m.SIDES}, "hand x": {s: round(float(mk.gripper(s)[0][0]), 4) for s in m.SIDES},
                        "at rest": m.at_rest(), "on rails": m.on_rails(), "asked forward (mm)": {s: round(1000 * v, 2) for s, v in plan.asked.items()}}  # fmt: skip

            folder = out / run
            result = kit.run(embodiment, {side: [] for side in hands}, folder, seed=0, director=director, extra=extra, limit_s=m.RUN_LIMIT_STEPS / hz, tail_s=1.0, sheet_frames=12,
                             crop={"top_cam": (200, 130, 440, 280)})  # fmt: skip
            rows = result["rows"]
            scores = [row["info"].get("score", 0.0) for row in rows]
            firsts = {level: next((row["t"] for row, s in zip(rows, scores) if s >= level), None) for level in (0.25, 0.5, 0.75, 1.0)}
            last = rows[-1]["extra"]
            pushing = [row for row in rows if "push" in (row["left"], row["right"]) or "hold" in (row["left"], row["right"]) or "hands stay" in (row["left"], row["right"])]
            first_touch = {s: next((row["t"] for row in rows if row["extra"]["opening (mm)"][s] <= 1000 * m.FEELER), None) for s in m.SIDES}
            readings = {
                "score": scores[-1], "first reached": firsts, "seconds": rows[-1]["t"], "ended": result.get("termination_reason"),
                "from their stops at the end (mm)": last["gap (mm)"], "widest opening at each stop at the end (mm)": last["opening (mm)"], "turned at the end (deg)": last["turned (deg)"],
                "each carriage went (mm)": {s: round(1000 * m.START_GAP[s] - last["gap (mm)"][s], 3) for s in m.SIDES},
                "each carriage first touched its stop at (s)": first_touch,
                "most turned (deg)": max(abs(row["extra"]["turned (deg)"]) for row in rows),
                "most lifted (mm), most aside (mm)": [max(row["extra"]["up (mm)"] for row in rows), max(abs(row["extra"]["aside (mm)"]) for row in rows)],
                "always on its rails": all(row["extra"]["on rails"] for row in rows),
                "the arms' push along the rails at most (N)": {s: round(max((row["force"][s][0] for row in pushing), default=0.0), 1) for s in m.SIDES},
                "the arms' push along the rails in the last pushing step (N)": {s: (pushing[-1]["force"][s][0] if pushing else None) for s in m.SIDES},
                "asked forward at the end (mm)": last["asked forward (mm)"],
                "hands from the gantry at the end (mm), at rest": [last["hand to gantry (mm)"], last["at rest"]],
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
