#!/usr/bin/env python3
"""Checks of the flip-key task in the simulator, no arm. The key lies in the dish as the trial starts; every load is a
push on one body and the opposite on the fob (the key is neither pushed nor turned as a whole).

    scripts/rta simpy <id> mech/tasks/flip_key/trials.py /workspace/robocurve/outputs/<id> [rest,mech] [seed=N]

``rest``  3 s with nobody touching anything: the first pictures (top camera and both wrist cameras, as rendered and at
          the size a policy gets), and how far anything moves.
``mech``  the key's mechanism under known loads: the blade pushed towards folded without the button (0.11 and
          0.28 N m: 2 and 5 N at its tip); the button pressed with 3, 4.5 and 50 N; with it pressed, the spring
          read against known torques; the blade folded home and the button let out (the pawl catches); a 2 N pull at
          the tip; the button pressed again, the blade let go 10 degrees short of folded with the button out (it
          flicks open and the pawl catches at open).

Every trial writes <out dir>/<trial>.json.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


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


LOADS: dict = {"newtons": 0.0, "newton_m": 0.0}  # the trial's loads, put on again every control step (w9c: a pair
# fixed in the world stopped balancing once the fob had turned on the dish's floor and tipped the key)


def settle(embodiment, seconds: float, until=None) -> float:
    steps = int(round(seconds * float(embodiment.info.control_hz)))
    m = embodiment.task.module
    for i in range(steps):
        loads(m, LOADS["newtons"], LOADS["newton_m"], keep=False)
        embodiment.step(_hold())
        if until is not None and until():
            return (i + 1) / float(embodiment.info.control_hz)
    return seconds


def push(m, name: str, force=(0.0, 0.0, 0.0), torque=(0.0, 0.0, 0.0)) -> None:
    """A push on ``name`` on top of the one that carries its extra weight (the task keeps that one on)."""
    lift = m.CARRIED.get(name, 0.0) * m.GRAVITY
    m.mk.push(name, force=(float(force[0]), float(force[1]), float(force[2]) + lift), torque=tuple(float(v) for v in torque))


def axis(m) -> np.ndarray:
    """The blade's hinge axis in the world as the key lies now (its fold is positive about it)."""
    return m.mk.pose("fob")[1] @ np.array([0.0, 0.0, 1.0])


def centre(m, name: str) -> np.ndarray:
    """Body ``name``'s centre of mass in the world, where a push acts."""
    return m.mk.MECH.environment.unwrapped.scene.rigid_objects[name].data.root_com_pos_w.torch[0].detach().cpu().numpy().astype(float)


def loads(m, newtons: float = 0.0, newton_m: float = 0.0, keep: bool = True) -> None:
    """Every push of the trial at once: ``newtons`` on the button into the fob, ``newton_m`` on the blade towards
    folded about its hinge, and the opposite of both on the fob, with the torque that puts the button's force and its
    opposite on one line (else the pair is a couple that tips the key: run w9a). ``keep``: put on again every step."""
    if keep:
        LOADS.update(newtons=float(newtons), newton_m=float(newton_m))
    f, t = -newtons * axis(m), newton_m * axis(m)
    m.STATE["extra"] = {"button": (f, np.zeros(3)), "blade": (np.zeros(3), t), "fob": (-f, -t + np.cross(centre(m, "fob") - centre(m, "button"), f))}


def fold_torque(m, newton_m: float) -> None:
    loads(m, 0.0, newton_m)


def press(m, newtons: float) -> None:
    loads(m, newtons, 0.0)


def state(m) -> dict:
    mk = m.mk
    return {"fold (deg)": round(math.degrees(m.fold()), 2), "button in (mm)": round(1000 * mk.joint("press"), 3), "pawl": m.STATE["pawl"],
            "fob moved (mm)": round(1000 * float(np.linalg.norm(mk.pose("fob")[0] - m.STATE["fob0"])), 2)}  # fmt: skip


def rest(embodiment, out: Path, seed: int) -> dict:
    from inspect_robots import Scene

    m = embodiment.task.module
    mk = m.mk
    observation = embodiment.reset(Scene(id="rest", instruction=m.INSTRUCTION), seed=seed)
    for camera, name in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        _save(out, name, np.asarray(observation.images[camera]))
    embodiment.step(_hold())
    at = {name: mk.pose(name)[0].copy() for name in m.KEY}
    settle(embodiment, 3.0)
    moved = {name: round(1000 * float(np.linalg.norm(mk.pose(name)[0] - at[name])), 3) for name in m.KEY}
    found = {"seed": seed, "yaw (deg)": round(math.degrees(m.STATE["yaw"]), 1), "moved over 3 s (mm)": moved, "report": m.report(), "score": m.score(None)}
    print(f"REST {json.dumps(found, default=str)}", flush=True)
    return found


def mech(embodiment, out: Path, seed: int) -> dict:
    from inspect_robots import Scene

    m = embodiment.task.module
    mk = m.mk
    embodiment.reset(Scene(id="mech", instruction=m.INSTRUCTION), seed=seed)
    embodiment.step(_hold())
    m.STATE["fob0"] = mk.pose("fob")[0].copy()
    rows = []

    def phase(label: str, seconds: float, until=None) -> None:
        took = settle(embodiment, seconds, until)
        row = {"phase": label, "s": round(took, 2), **state(m)}
        rows.append(row)
        print("MECH", json.dumps(row), flush=True)

    for newton_m in (0.11, 0.28):  # 2 N and 5 N at the tip (55 mm), no button
        fold_torque(m, newton_m)
        phase(f"no button, {newton_m} N m towards folded", 1.0)
    fold_torque(m, 0.0)
    phase("let be", 0.5)
    for newtons in (3.0, 4.5, 50.0):
        press(m, newtons)
        phase(f"button pressed with {newtons} N", 0.5)
    for newton_m in (0.030, 0.040):  # the spring: 0.02 N m open + 0.00955 N m/rad: 60 and 120 deg
        loads(m, 4.5, newton_m)
        phase(f"button in, {newton_m} N m towards folded (spring balance)", 2.0)
    loads(m, 4.5, 0.10)
    phase("button in, 0.10 N m: folded home", 1.0)
    fold_torque(m, 0.10)  # the button let out, the blade still pushed home
    phase("button let out, blade held home", 0.5)
    fold_torque(m, 0.0)
    phase("let be", 1.0)
    fold_torque(m, -0.11)  # 2 N at the tip, outwards
    phase("2 N outwards at the tip, button out", 1.0)
    fold_torque(m, 0.0)
    loads(m, 4.5, 0.0)  # pressed again: the spring opens it
    phase("button pressed, blade let be", 0.4, until=lambda: m.fold() < math.radians(170.0))
    # held 10 deg short of folded against the spring, then the button out, then let go
    want = math.radians(170.0)
    for _ in range(40):
        t = m.spring_torque() + 0.05 * (want - m.fold())
        loads(m, 4.5, t)
        settle(embodiment, 0.05)
    phase("held 10 deg short, button in", 0.05)
    fold_torque(m, m.spring_torque())
    phase("button let out 10 deg short", 0.3)
    fold_torque(m, 0.0)
    phase("let go 10 deg short", 1.5)
    found = {"seed": seed, "rows": rows, "events": m.report()["events"]}
    return found


def diag(embodiment, out: Path, seed: int) -> dict:
    """Every control step of the fold with the button held in: 0.5 s pressed with 4.5 N, then 0.03 N m towards folded
    as well for 2.5 s (the spring balances it at 60 deg), then 0.10 N m for 1 s (folded home); loads put on again every
    step. Logged: fold, button, pawl, the fob's place and tilt, its turn about the vertical, the loads."""
    from inspect_robots import Scene

    m = embodiment.task.module
    mk = m.mk
    embodiment.reset(Scene(id="diag", instruction=m.INSTRUCTION), seed=seed)
    embodiment.step(_hold())
    p0, r0 = mk.pose("fob")
    rows = []
    hz = float(embodiment.info.control_hz)
    for seconds, newtons, newton_m in ((0.5, 4.5, 0.0), (2.5, 4.5, 0.03), (1.0, 4.5, 0.10)):
        loads(m, newtons, newton_m)
        for _ in range(int(round(seconds * hz))):
            loads(m, newtons, newton_m, keep=False)
            embodiment.step(_hold())
            p, r = mk.pose("fob")
            turn = math.degrees(math.atan2(r[1, 0], r[0, 0]) - math.atan2(r0[1, 0], r0[0, 0]))
            row = {"t": round(mk.now(), 2), "load": [newtons, newton_m], "fold": round(math.degrees(m.fold()), 2), "button": round(1000 * mk.joint("press"), 3),
                   "pawl": m.STATE["pawl"], "fob_mm": (1000 * (p - p0)).round(2).tolist(), "tilt": round(math.degrees(math.acos(min(1.0, float(r[2, 2])))), 2),
                   "turn": round(turn, 2), "spring": round(m.spring_torque(), 4)}  # fmt: skip
            rows.append(row)
            print("DIAG", json.dumps(row), flush=True)
    return {"seed": seed, "rows": rows, "events": m.report()["events"]}


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    known = {"rest": rest, "mech": mech, "diag": diag}
    trials = sys.argv[2].split(",") if len(sys.argv) > 2 else ["rest"]
    seed = 0
    for arg in sys.argv[3:]:
        key, value = arg.split("=", 1)
        if key == "seed":
            seed = int(value)
        else:
            raise SystemExit(f"unknown argument {arg!r}")
    if any(name not in known for name in trials):
        raise SystemExit(f"trials here: {sorted(known)}; got {trials}")
    out.mkdir(parents=True, exist_ok=True)
    embodiment = kit.open_task(HERE / "task.py")
    failed = 0
    for name in trials:
        try:
            result = known[name](embodiment, out, seed)
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
