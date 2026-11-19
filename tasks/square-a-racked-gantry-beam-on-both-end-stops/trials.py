#!/usr/bin/env python3
"""Checks of the gantry task in the simulator.

    scripts/rta simpy <id> mech/tasks/gantry_square/trials.py /workspace/robocurve/outputs/<id> [rest,loads,bind]

``rest``   3 s with nobody touching anything: the first pictures (top camera and both wrist cameras, as rendered and
           at the size a policy gets), where the two carriages stand at the start and after 3 s.
``loads``  steady forces put on the beam directly, no arm, each from the racked start: at its middle (3 N: does it
           slide, and where does it settle against the stops); at its right end over the carriage (3, 6, 12 and
           24 N: does it bind); and lifted (a pull of twice its weight straight up: the lips hold it).
``bind``   the hand's check asked for before any script: the right hand, closed and pointing down, pushes the
           beam's near face over the right carriage, 6 cm forward in 3 s. How far each carriage gets.

Every trial writes <out dir>/<trial>.json; ``rest`` its pictures; ``bind`` a run folder with its videos.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

TIP = 0.0129  # m the fingertips' ends reach past the grasp point
FINGER = 0.0039  # m a fingertip is thick: the closed hand's outer faces stand that far either side of the grasp point
HAND_Z = 0.042  # m the grasp point over the bench while it pushes: the fingertips' ends 29 mm up, 6 mm over a carriage's top


def _hold():
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))


def _save(out: Path, name: str, frame: np.ndarray) -> None:
    import cv2
    import demo_kit as kit

    cv2.imwrite(str(out / f"{name}.png"), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    small = kit.policy_picture(frame)
    cv2.imwrite(str(out / f"{name}_policy.png"), cv2.cvtColor(small, cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(out / f"{name}_policy_x3.png"), cv2.cvtColor(cv2.resize(small, (small.shape[1] * 3, small.shape[0] * 3), interpolation=cv2.INTER_NEAREST), cv2.COLOR_RGB2BGR))


def reading(m) -> dict:
    position, _ = m.mk.pose("gantry")
    return {"from their stops (mm)": {side: round(1000 * v, 3) for side, v in m.gaps().items()}, "widest opening (mm)": {side: round(1000 * v, 3) for side, v in m.openings().items()},
            "turned (deg)": round(float(np.degrees(m.yaw())), 3), "over its rails (mm)": round(1000 * float(position[2] - m.TOP - m.BEAM_Z), 3), "aside (mm)": round(1000 * float(position[1]), 3),
            "on its rails": m.on_rails()}  # fmt: skip


def centre_of_mass(m) -> np.ndarray:
    """Of the gantry in its own frame: its shapes weigh by their volume."""
    shapes = m.mk.MECH.bodies["gantry"].shapes
    volumes = np.array([float(np.prod(shape.dims)) for shape in shapes])
    return (volumes[:, None] * np.array([shape.pos for shape in shapes])).sum(axis=0) / volumes.sum()


def rest(embodiment, out: Path) -> dict:
    from inspect_robots import Scene

    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    observation = embodiment.reset(Scene(id="rest", instruction=m.INSTRUCTION), seed=0)
    for camera, name in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        _save(out, name, np.asarray(observation.images[camera]))
    embodiment.step(_hold())
    first = reading(m)
    for _ in range(int(round(3.0 * hz)) - 1):
        embodiment.step(_hold())
    moved = {name: round(1000 * float(np.linalg.norm(mk.pose(name)[0] - mk.built_pose(name)[0])), 3) for name in mk.MECH.bodies if mk.MECH.bodies[name].free}
    found = {"at the first step": first, "after 3 s": reading(m), "bodies from where they were built after 3 s (mm)": moved, "centre of mass in the gantry's frame (mm)": [round(1000 * float(v), 2) for v in centre_of_mass(m)],
             "score": m.score(None), "report": m.report()}  # fmt: skip
    print(f"REST {json.dumps(found)}", flush=True)
    return found


def loads(embodiment, out: Path) -> dict:
    """Steady forces on the beam, no arm, each from the racked start."""
    from inspect_robots import Scene

    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    com = centre_of_mass(m)

    def push_at(point_local, force) -> None:
        """A force (world) on the gantry at a point of its own frame."""
        position, rotation = mk.pose("gantry")
        arm = rotation @ (np.asarray(point_local, dtype=float) - com)
        mk.push("gantry", force=force, torque=np.cross(arm, np.asarray(force, dtype=float)))

    def run(seconds: float, point_local=None, force=None) -> list:
        rows = []
        for _ in range(int(round(seconds * hz))):
            if force is not None:
                push_at(point_local, force)  # the point moves with the beam
            embodiment.step(_hold())
            rows.append(reading(m))
        return rows

    def fresh(name: str) -> None:
        embodiment.reset(Scene(id=name, instruction=m.INSTRUCTION), seed=0)
        run(0.5)

    found: dict = {"weight (N)": round(m.MASS * 9.81, 2), "friction": m.FRICTION[0]}
    # at the middle of the beam's near face, along the rails
    fresh("middle")
    rows = run(4.0, (-m.BEAM_S / 2, 0.0, 0.0), (3.0, 0.0, 0.0))
    moving = next((k for k, row in enumerate(rows) if max(row["widest opening (mm)"].values()) <= 0.5), None)
    under_load = rows[-1]
    mk.push("gantry")
    after = run(1.5)[-1]
    found["3 N at the middle of the beam"] = {"both carriages within 0.5 mm of their stops after (s)": None if moving is None else (moving + 1) / hz, "after 4 s, under the load": under_load,
                                               "1.5 s after the load was taken off": after, "score": m.score(None)}  # fmt: skip
    # at the right end, over the right carriage
    at_end = {}
    for newtons in (3.0, 6.0, 12.0, 24.0):
        fresh(f"end {newtons:g}")
        before = reading(m)
        rows = run(3.0, (-m.BEAM_S / 2, m.RAIL_Y["right"], 0.0), (newtons, 0.0, 0.0))
        last, early = rows[-1], rows[int(hz) - 1]
        went = {side: round(before["from their stops (mm)"][side] - last["from their stops (mm)"][side], 3) for side in m.SIDES}
        in_last_second = {side: round(rows[-int(hz) - 1]["from their stops (mm)"][side] - last["from their stops (mm)"][side], 3) for side in m.SIDES}
        mk.push("gantry")
        at_end[f"{newtons:g} N"] = {"each carriage went (mm) in 3 s": went, "of that in the last second": in_last_second, "turned (deg) after 1 s and after 3 s": [early["turned (deg)"], last["turned (deg)"]],
                                    "on its rails": last["on its rails"]}  # fmt: skip
    found["at the right end, over the right carriage"] = at_end
    # lifted: twice its weight straight up at its centre of mass
    fresh("lift")
    rows = run(1.0, com, (0.0, 0.0, 2 * m.MASS * 9.81))
    mk.push("gantry")
    found["pulled straight up with twice its weight"] = {"over its rails (mm) under the pull": rows[-1]["over its rails (mm)"], "on its rails": rows[-1]["on its rails"], "1 s after": run(1.0)[-1]}
    print(f"LOADS {json.dumps(found)}", flush=True)
    return found


def bind(embodiment, out: Path) -> dict:
    import demo_kit as kit

    m = embodiment.task.module
    mk = m.mk
    Move = kit.Move
    y = m.RAIL_Y["right"]
    face = m.START_X - m.BEAM_S / 2 - y * math.sin(m.START_YAW)  # the beam's near face over the right carriage at the start
    behind = np.array([face - FINGER - 0.006, y, m.TOP + HAND_Z])
    turn = kit.down(math.pi / 2)  # the fingers close along the rails: a closed hand's broad face meets the beam
    moves = [Move(3.0, pos=behind + np.array([-0.02, 0.0, 0.08]), rot=turn, grip=0.0, label="over"), Move(2.0, pos=behind, label="down behind the beam"), Move(0.5, label="there"),
             Move(3.0, pos=behind + np.array([0.066, 0.0, 0.0]), label="push"), Move(1.0, label="hold"), Move(1.5, pos=behind + np.array([0.0, 0.0, 0.08]), label="up and back")]  # fmt: skip

    def extra() -> dict:
        return {**reading(m), "hand": [round(float(v), 4) for v in mk.gripper("right")[0]]}

    result = kit.run(embodiment, {"right": moves}, out / "bind", seed=0, extra=extra, limit_s=20.0, tail_s=1.0, sheet_frames=12)
    rows = result["rows"]
    start, pushed = rows[0]["extra"], [row for row in rows if row["right"] in ("push", "hold")]
    last = rows[-1]["extra"]
    found = {"start": start, "at the end of the push": pushed[-1]["extra"], "at the end": last,
             "each carriage went (mm)": {side: round(start["from their stops (mm)"][side] - last["from their stops (mm)"][side], 3) for side in m.SIDES},
             "the hand went forward (mm) while it pushed": round(1000 * (pushed[-1]["extra"]["hand"][0] - pushed[0]["extra"]["hand"][0]), 1),
             "the arm's push along the rails (N): at most, at the end of the push": [round(max(row["force"]["right"][0] for row in pushed), 1), pushed[-1]["force"]["right"][0]],
             "most turned (deg)": max(abs(row["extra"]["turned (deg)"]) for row in rows), "score": m.score(None), "worst IK (mm, deg)": result["worst_ik"]}  # fmt: skip
    print(f"BIND {json.dumps(found)}", flush=True)
    return found


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    known = {"rest": rest, "loads": loads, "bind": bind}
    trials = sys.argv[2].split(",") if len(sys.argv) > 2 else ["rest"]
    if any(name not in known for name in trials):
        raise SystemExit(f"trials here: {sorted(known)}; got {trials}")
    out.mkdir(parents=True, exist_ok=True)
    embodiment = kit.open_task(HERE / "task.py")
    failed = 0
    for name in trials:
        try:
            result = known[name](embodiment, out)
        except Exception as error:
            import traceback

            traceback.print_exc()
            print(f"TRIAL {name} FAILED: {error!r}", flush=True)
            failed += 1
            continue
        (out / f"{name}.json").write_text(json.dumps(result, indent=1, default=str))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
