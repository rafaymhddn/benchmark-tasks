#!/usr/bin/env python3
"""Simulator trials of the flip-socket task's rules, both arms held at their ready pose throughout.

    scripts/rta simpy <id> mech/tasks/flip_socket_swap/trials.py /workspace/robocurve/outputs/<id> [wrong_socket,drop_zero]

``wrong_socket``    the tool written at the bracket, its shaft's tip 0.5 mm short of the standoff's face, the
                    working barrel flush with the tip and pressed outboard along the shaft with a steady 3 N (it
                    moves by contact alone): the blue 3/8 socket aligned to the standoff's flats slides on to the
                    bracket's face, turned 30 degrees off them (corners on flats) it butts on the standoff, and the
                    green 11/32 end out (smaller than the standoff) butts too; turned 15 degrees off, the crown's
                    chamfer turns a tool free to roll into line (recorded).
``half``            the green/blue barrel slid off the tip (the quarter event), then all three barrels back on the
                    shaft, blue end out at the tip, the tool in its start cradle: 0.5 and no more.
``drop_zero``       a barrel knocked off the bench: 0 for good.

Each trial writes <out dir>/<trial>.json. Poses are put in by hand here (write_pose), and wrong_socket presses a
barrel with a computed force (push): the reference run does the same with the fingers, in demo.py.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

SAMPLE_S = 0.25  # s between logged samples
PRESS_N, PRESS_S = 3.0, 1.5  # the working barrel pressed outboard along the shaft with a steady 3 N (a firm finger) for this long


def _hold():
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))


class Tool:
    """Writes a barrel where the driver stands now, read live before every write: the barrel rides the shaft at a
    station (m from the tip), rolled with the tool, ``flip`` for its second-named end outboard."""

    def __init__(self, m) -> None:
        self.m = m
        self.b = {b.key: b for b in m.P.BARRELS}

    def put(self, key: str, station: float, flip: bool, up: float = 0.0, on=None):
        m = self.m
        p, r = on if on is not None else m.mk.pose("driver")
        at = p + r @ np.array([station + self.b[key].length / 2, 0.0, up])
        quat = m.mk.matrix_to_quat(r if flip else r @ m.scene.rot_z(math.pi))  # flip: blue (second-named) end outboard
        m.mk.write_pose(key, at, quat)


def run(embodiment, m, seconds: float, setup, sample=True, push=None) -> dict:
    from inspect_robots import Scene

    embodiment.reset(Scene(id="trial", instruction=m.INSTRUCTION), seed=0)
    setup(m)
    embodiment.step(_hold())  # one step settles the scene and gives the kit its first readings
    hz = float(embodiment.info.control_hz)
    every = max(1, int(round(SAMPLE_S * hz)))
    samples, res = [], None
    for i in range(int(round(seconds * hz))):
        if push is not None:  # a steady press: the tool is held at its drift velocity, contact stops it
            at, rot = m.mk.pose("driver")
            m.mk.write_pose("driver", at, None, lin_vel=tuple(float(v) for v in push))
        res = embodiment.step(_hold())
        if sample and (i + 1) % every == 0:
            samples.append({"t (s)": round((i + 1) / hz, 2), **m.report()})
    return {"score at the end": float(res.info.get("score", 0.0)), "report": m.report(), "samples": samples}


def _roll_to(m, clock_deg: float, gap: float, proud: float, flipped: bool = True):
    """write_pose the driver: its tip ``gap`` m outboard of the standoff's near face, rolled ``clock_deg``; the
    working barrel on it standing ``proud`` m past the tip (``flipped``: blue end out), the others at their seats."""
    tip = m.P.POST_FACE_Y + gap
    roll = m.scene.rot_y(math.radians(clock_deg)) @ m.scene.DRIVER_START_ROT
    at = np.array([m.P.DRIVER_X, tip, m.TOP + m.P.AXIS_Z])
    m.mk.write_pose("driver", at, m.mk.matrix_to_quat(roll))
    tool = Tool(m)
    for key, (station, flip) in ((m.WORK, (-proud, flipped)), ("barrel_14_516", (0.0405, False)), ("barrel_716_12", (0.0690, False))):
        tool.put(key, station, flip, on=(at, roll))
    return at, roll


def wrong_socket(embodiment, m) -> dict:
    """Each case writes the tool at the bracket, its shaft's tip 0.5 mm short of the standoff's face and rolled to the
    case's clocking, the working barrel flush with the tip; then presses that barrel outboard along the shaft with a
    steady 3 N at its centre for 1.5 s (the tool kept where it was written; the barrel moves by contact alone) and lets
    it rest 1 s unpressed. The aligned blue socket slides over the standoff to the bracket's face; turned 30 degrees off
    the flats (corners on flats), and the smaller green 11/32 end out, it butts on the standoff (never the score's 5 mm
    deep). Turned 15 degrees off, the twisted chamfer on the standoff's crown turns the socket, and with it a tool free
    to roll (pose writes hold the tool only between steps), into line: recorded, not judged."""
    from inspect_robots import Scene

    out = {"presses": []}
    for name, clock, flipped, note in (
            ("turned 30 deg off the flats", m.P.POST_TWIST + 30.0, True, "corners on flats: butts on the standoff, never 5 mm deep"),
            ("aligned", m.P.POST_TWIST, True, "slides on to the bracket's face: 5 mm or more"),
            ("green 11/32 end", m.P.POST_TWIST, False, "smaller than the standoff: butts, never 5 mm deep"),
            ("turned 15 deg off the flats (a free tool)", m.P.POST_TWIST + 15.0, True, "the crown's chamfer turns a tool free to roll into line: it may seat")):
        embodiment.reset(Scene(id="press", instruction=m.INSTRUCTION), seed=0)
        embodiment.step(_hold())
        at, rot = _roll_to(m, clock, 0.0005, 0.0, flipped)
        q = m.mk.matrix_to_quat(rot)
        hz = float(embodiment.info.control_hz)
        press = tuple(float(v) for v in -PRESS_N * rot[:, 0])  # outboard along the shaft
        m.mk.write_pose("driver", at, q)
        embodiment.step(_hold())  # one step at rest: the kit's poses are a step old right after a write
        m.mk.push(m.WORK, force=press)
        for _ in range(int(round(PRESS_S * hz))):
            m.mk.write_pose("driver", at, q)
            embodiment.step(_hold())
        m.mk.push(m.WORK)  # let go of it
        res = None
        for _ in range(int(round(1.0 * hz))):
            m.mk.write_pose("driver", at, q)
            res = embodiment.step(_hold())
        if flipped:  # the blue mouth past the standoff's near face
            depth = m.report()["seat depth (mm)"]
        else:  # the green 11/32 end out: its own mouth past the standoff's near face
            depth = round(1000 * float(np.dot(m.POST_AT - m.barrel_ends(m.WORK)[0], m.driver_pose()[1][:, 0])), 1)
        out["presses"].append({"press": name, "seat depth (mm)": depth, "expect": note, "score": float(res.info.get("score", 0.0))})
        print(f"  press {name}: depth {depth} mm", flush=True)
    a, b, c = (p["seat depth (mm)"] for p in out["presses"][:3])
    out["pass"] = a < m.P.SEAT_MIN * 1000.0 and b >= m.P.SEAT_MIN * 1000.0 and c < m.P.SEAT_MIN * 1000.0
    return out


def drop_zero(embodiment, m) -> dict:
    def setup(mm):
        p, r = mm.mk.built_pose(m.WORK)  # knocked off the bench's edge: already under its top and falling
        mm.mk.write_pose(m.WORK, np.array([p[0] - 0.45, p[1] + 0.30, m.TOP - 0.031]), mm.mk.matrix_to_quat(r), lin_vel=(0.0, 0.0, -0.5))

    out = run(embodiment, m, 2.0, setup)
    out["pass"] = out["score at the end"] == 0.0 and out["report"]["zero"] is not None
    return out


def half(embodiment, m) -> dict:
    def setup(mm):
        p, r = mm.driver_pose()
        Tool(mm).put(mm.WORK, -0.040, True)  # slid off the tip: the quarter event fires on the first step
    def restack(mm):
        Tool(mm).put(mm.WORK, -0.010, True)  # back on, blue end out, 10 mm proud of the tip
        Tool(mm).put("barrel_14_516", 0.0405, False)

    from inspect_robots import Scene

    embodiment.reset(Scene(id="half", instruction=m.INSTRUCTION), seed=0)
    embodiment.step(_hold())
    setup(m)
    embodiment.step(_hold())
    restack(m)
    res = None
    for _ in range(40):
        res = embodiment.step(_hold())
    out = {"score at the end": float(res.info.get("score", 0.0)), "report": m.report()}
    out["pass"] = out["score at the end"] == 0.5
    return out


TRIALS = {"wrong_socket": wrong_socket, "half": half, "drop_zero": drop_zero}


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    names = sys.argv[2].split(",") if len(sys.argv) > 2 else list(TRIALS)
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    ok = True
    for name in names:
        result = TRIALS[name](embodiment, m)
        (out / f"{name}.json").write_text(json.dumps(result, indent=1, default=str))
        brief = {k: v for k, v in result.items() if k != "samples"}
        print(f"TRIAL {name} " + json.dumps(brief, default=str), flush=True)
        ok &= bool(result["pass"])
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
