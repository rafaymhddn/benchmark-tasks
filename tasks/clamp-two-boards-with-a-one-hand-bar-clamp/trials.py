#!/usr/bin/env python3
"""Checks of the bar clamp task in the simulator, no arm.

    scripts/rta simpy <id> mech/tasks/bar_clamp/trials.py /workspace/robocurve/outputs/<id> [rest,gauge,lock,strokes] [finger=N,N] [CLAMP_...=value]

(CLAMP_PAD_K, CLAMP_GAUGE_RATE, CLAMP_PAD_MASS, CLAMP_TRIGGER_MASS: the task's overrides, set before it is loaded.)

``rest``     3 s with nobody touching anything: the first pictures (top camera and both wrist cameras, as rendered and
             at the size a policy gets), and how far anything moves.
``gauge``    the gauge pad pressed into the grip's jaw with 10, 20, 40 and 80 N (and the grip pushed back as hard, so
             that the clamp stays where it is): how far it gives, and so the rate at which the simulator's pad gives.
``lock``     the bar pushed in the closing direction until it is 2 cm in (it slides: nothing presses yet), then pulled
             back with 20 N for 1 s (the lock plate holds it); then pushed on until the jaws meet, pressed with 10 N
             and let be (the push leaves no force on the gauge).
``strokes``  the clamp put round the boards (6 cm towards the boards), then the trigger squeezed with a finger's force
             at 4 cm from its pin (and the handle pushed back as hard) for 0.6 s and let go for 0.6 s, again and again:
             each stroke's travel of the bar, the stroke at which the joint closes, the gauge after it and after three
             more, and after a pull of 20 N on the bar's jaw. For every finger force given (default 20, 35 and 50 N).

Every trial writes <out dir>/<trial>.json.
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

ROUND = 0.060  # m the clamp is put towards the boards: the pads' middles 2.25 cm inside the boards' near ends
STROKE_S, BACK_S = 0.6, 0.6


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


def settle(embodiment, seconds: float) -> None:
    for _ in range(int(round(seconds * float(embodiment.info.control_hz)))):
        embodiment.step(_hold())


def fresh(embodiment, m, name: str) -> None:
    from inspect_robots import Scene

    embodiment.reset(Scene(id=name, instruction=m.INSTRUCTION), seed=0)
    embodiment.step(_hold())


def reading(m) -> dict:
    out = m.report()
    out["drive plate along the bar, its bite (mm)"] = [round(1000 * m.mk.joint("drive"), 3), round(1000 * m.STATE["bite"], 3)]
    out["lock (mm)"] = round(1000 * m.STATE["lock"], 3)
    return out


def carried(m, name: str) -> tuple:
    """The push that carries body ``name``'s weight over its real one (the task keeps it on), as a force."""
    return (0.0, 0.0, m.CARRIED.get(name, 0.0) * m.GRAVITY)


def push(m, name: str, force=(0.0, 0.0, 0.0), torque=(0.0, 0.0, 0.0)) -> None:
    """A push on ``name`` on top of the one that carries its extra weight."""
    lift = carried(m, name)
    m.mk.push(name, force=(force[0] + lift[0], force[1] + lift[1], force[2] + lift[2]), torque=tuple(float(v) for v in torque))


def centre(m, name: str) -> np.ndarray:
    """Body ``name``'s centre of mass in the world, where a push acts."""
    return m.mk.MECH.environment.unwrapped.scene.rigid_objects[name].data.root_com_pos_w.torch[0].detach().cpu().numpy().astype(float)


def pair(m, on: str, against: str, force) -> None:
    """``force`` (world) on ``on`` and the opposite on ``against``, with the torque that puts both on one line: the
    clamp as a whole is neither pushed nor turned (c6a: equal and opposite forces on the pad and the grip, 7 cm apart,
    spun the clamp)."""
    force = np.asarray(force, dtype=float)
    push(m, on, force=force)
    push(m, against, force=-force, torque=np.cross(centre(m, against) - centre(m, on), force))


def clamp_round(m, shift: float = ROUND) -> None:
    """Every body of the clamp moved ``shift`` m towards the boards (+x), as it stands."""
    mk = m.mk
    for name in (*m.CLAMP, "link", "plate"):
        position, rotation = mk.pose(name)
        mk.write_pose(name, position + np.array([shift, 0.0, 0.0]), mk.matrix_to_quat(rotation))


def rest(embodiment, out: Path) -> dict:
    from inspect_robots import Scene

    m = embodiment.task.module
    mk = m.mk
    observation = embodiment.reset(Scene(id="rest", instruction=m.INSTRUCTION), seed=0)
    for camera, name in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        _save(out, name, np.asarray(observation.images[camera]))
    settle(embodiment, 3.0)
    moved = {name: round(1000 * float(np.linalg.norm(mk.pose(name)[0] - mk.built_pose(name)[0])), 3) for name in (*m.BOARDS, *m.CLAMP)}
    found = {"moved from where they were built after 3 s (mm)": moved, "after 3 s": reading(m), "score": m.score(None)}
    print(f"REST {json.dumps(found)}", flush=True)
    return found


def gauge(embodiment, out: Path) -> dict:
    m = embodiment.task.module
    mk = m.mk
    found: dict = {}
    for newtons in (10.0, 20.0, 40.0, 80.0):
        fresh(embodiment, m, f"gauge {newtons}")
        pair(m, "pad", "grip", (0.0, -newtons, 0.0))
        settle(embodiment, 1.5)
        given = mk.joint("gauge")
        found[f"{newtons:g} N"] = {"given (mm)": round(1000 * given, 4), "rate (N/mm)": round(newtons / (1000 * given), 3) if given > 0 else None, "grip moved (mm)": round(1000 * float(np.linalg.norm(mk.pose("grip")[0] - mk.built_pose("grip")[0])), 3)}
        push(m, "pad")
        push(m, "grip")
        print(f"GAUGE {newtons:g} N: {json.dumps(found[f'{newtons:g} N'])}", flush=True)
    return found


def lock(embodiment, out: Path) -> dict:
    """The bar pushed closed (0.6 N: its jaw drags on the bench) until it is 2 cm in, let be, pulled open with 20 N: the
    lock plate holds it. Then
    pushed on until the jaws meet (no boards between them here), pressed closed with 10 N for 1 s and let be: the
    gauge reads the push while it lasts and nothing once it is over."""
    m = embodiment.task.module
    mk = m.mk
    fresh(embodiment, m, "lock")
    hz = float(embodiment.info.control_hz)
    rows = []

    def phase(label: str, force: float, seconds: float, until=None) -> None:
        steps = 0
        for steps in range(1, int(round(seconds * hz)) + 1):
            pair(m, "bar", "grip", (0.0, -force, 0.0))  # the closing direction is -y
            embodiment.step(_hold())
            if until is not None and until():
                break
        rows.append({"after": label, "steps": steps, "bar drawn (mm)": round(1000 * mk.joint("draw"), 3), "lock (mm)": round(1000 * m.STATE["lock"], 3), "gauge (N)": round(m.gauge(), 2)})
        print(f"LOCK {json.dumps(rows[-1])}", flush=True)

    phase("pushed closed with 0.6 N until 2 cm in", 0.6, 3.0, until=lambda: mk.joint("draw") >= 0.020)
    phase("let be", 0.0, 0.5)
    phase("pulled open with 20 N", -20.0, 1.0)
    phase("let be", 0.0, 0.5)
    phase("pushed closed with 0.6 N until the jaws meet", 0.6, 4.0, until=lambda: m.gauge() >= 0.1)
    phase("pressed closed with 10 N", 10.0, 1.0)
    phase("let be", 0.0, 1.0)
    push(m, "bar")
    push(m, "grip")
    return {"steps": rows}


def strokes(embodiment, out: Path, fingers=(20.0, 35.0, 50.0)) -> dict:
    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    found: dict = {}
    for finger in fingers:
        fresh(embodiment, m, f"strokes {finger}")
        clamp_round(m)
        settle(embodiment, 0.5)
        torque = finger * m.FINGER_AT
        log, closed_at = [], None
        for stroke in range(1, 16):
            drawn_before = mk.joint("draw")
            most = 0.0
            for k in range(int(round(STROKE_S * hz))):
                push(m, "trigger", torque=(0.0, 0.0, -torque))  # the trigger's hinge turns about -z when squeezed
                push(m, "grip", torque=(0.0, 0.0, torque))
                embodiment.step(_hold())
                most = max(most, mk.joint("trigger"))
            squeezed = {"trigger (deg)": round(math.degrees(most), 2), "gauge squeezed (N)": round(m.gauge(), 2)}
            for k in range(int(round(BACK_S * hz))):
                push(m, "trigger")
                push(m, "grip")
                embodiment.step(_hold())
            row = {"stroke": stroke, "bar drawn (mm)": round(1000 * (mk.joint("draw") - drawn_before), 3), **squeezed, "gauge let go (N)": round(m.gauge(), 2),
                   "trigger back (deg)": round(math.degrees(mk.joint("trigger")), 2), "closed": m.closed(), "joint (mm)": [round(1000 * g, 3) for g in m.joint_gaps()]}  # fmt: skip
            log.append(row)
            print(f"STROKE {finger:g} N {json.dumps(row)}", flush=True)
            if closed_at is None and m.closed():
                closed_at = stroke
            if closed_at is not None and stroke >= closed_at + 3:
                break
        before_pull = m.gauge()
        for k in range(int(round(1.0 * hz))):  # the bar's jaw pulled open with 20 N: the lock plate holds
            pair(m, "bar", "grip", (0.0, 20.0, 0.0))
            embodiment.step(_hold())
        push(m, "bar")
        push(m, "grip")
        speeds = []
        for k in range(int(round(1.0 * hz))):  # let be for a second: how fast each body still moves
            embodiment.step(_hold())
            speeds.append(m.report()["speeds (mm/s)"])
        print(f"SPEEDS {finger:g} N most over 1 s (mm/s): {json.dumps({name: max(row[name] for row in speeds) for name in speeds[0]})}", flush=True)
        found[f"finger {finger:g} N at 4 cm"] = {"strokes": log, "closed at stroke": closed_at, "gauge when closed (N)": next((r["gauge let go (N)"] for r in log if r["stroke"] == closed_at), None),
                                                  "gauge three strokes later (N)": log[-1]["gauge let go (N)"], "gauge before and after a 20 N pull on the bar's jaw (N)": [round(before_pull, 2), round(m.gauge(), 2)],
                                                  "end": reading(m)}  # fmt: skip
        print(f"STROKES {finger:g} N: closed at {closed_at}, {json.dumps(found[f'finger {finger:g} N at 4 cm']['end'])}", flush=True)
    return found


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    known = {"rest": rest, "gauge": gauge, "lock": lock, "strokes": strokes}
    trials = sys.argv[2].split(",") if len(sys.argv) > 2 else ["rest"]
    fingers = (20.0, 35.0, 50.0)
    for arg in sys.argv[3:]:
        key, value = arg.split("=", 1)
        if key == "finger":
            fingers = tuple(float(v) for v in value.split(","))
        elif key in ("CLAMP_PAD_K", "CLAMP_GAUGE_RATE", "CLAMP_PAD_MASS", "CLAMP_TRIGGER_MASS"):
            os.environ[key] = str(float(value))
        else:
            raise SystemExit(f"unknown argument {arg!r}")
    if any(name not in known for name in trials):
        raise SystemExit(f"trials here: {sorted(known)}; got {trials}")
    out.mkdir(parents=True, exist_ok=True)
    embodiment = kit.open_task(HERE / "task.py")
    failed = 0
    for name in trials:
        try:
            result = strokes(embodiment, out, fingers) if name == "strokes" else known[name](embodiment, out)
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
