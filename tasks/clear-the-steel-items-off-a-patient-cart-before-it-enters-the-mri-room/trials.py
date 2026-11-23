#!/usr/bin/env python3
"""PhysX trials of the MRI tray task's levels, both arms held at their ready pose throughout.

    scripts/rta simpy <id> mech/tasks/mri_cart_clear/trials.py /workspace/robocurve/outputs/<id> [two_in_bin,three_in_bin,safe_put_back]

``two_in_bin``    the clamp and the padlock set lying in the bin, the shears dropped on the bench beside it:
                  after 4 s the score is 0.5 (two steel items wholly in the bin, the third off the tray).
``three_in_bin``  all three steel items lying in the bin, hands at rest: the hold runs out and the score is 1.0.
``safe_put_back`` all three steel items in the bin; the bottle lifted 70 mm, carried 127 mm off the tray and stood
                  back on it 40 mm from its start: no zero at any step, 1.0 at the end.
``safe_left_off`` the same carry, the bottle left standing on the bench off the tray: no zero, 0 at the end.

Each trial writes <out dir>/<trial>.json: the score over time and the task's report.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

SAMPLE_S = 0.5


def _hold(right_grip: float = 1.0):
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, right_grip]))


def run(embodiment, m, seconds: float, setup, right_grip: float = 1.0, pre_steps: int = 0, pre_grip: float = 1.0,
        during=None) -> dict:
    """Reset, optionally pre-position the right gripper, call ``setup(m)``, then hold both arms ready, calling
    ``during(m, i)`` (if given) before control step ``i``."""
    from inspect_robots import Scene

    embodiment.reset(Scene(id="trial", instruction=m.INSTRUCTION), seed=0)
    for _ in range(pre_steps):
        embodiment.step(_hold(pre_grip))
    setup(m)
    hz = float(embodiment.info.control_hz)
    every = max(1, int(round(SAMPLE_S * hz)))
    samples, res = [], None
    for i in range(int(round(seconds * hz))):
        if during is not None:
            during(m, i)
        res = embodiment.step(_hold(right_grip))
        if (i + 1) % every == 0:
            samples.append({"t (s)": round((i + 1) / hz, 2), "score": float(res.info.get("score", 0.0)),
                            "in bin": list(m.binned()), "zero": m.STATE.get("zero"),
                            "safe off tray": {n: m.safe_off(n) for n in m.SAFE if m.safe_off(n)}})  # fmt: skip
    return {"score at the end": float(res.info.get("score", 0.0)), "samples": samples, "report": m.report()}


def _put(m, name, at, yaw: float = 0.0) -> None:
    m.mk.write_pose(name, (float(at[0]), float(at[1]), float(at[2])), tuple(float(v) for v in m.mk.yaw_quat(np.radians(yaw))))


def _in_bin_floor(m, name, dx, dy, yaw: float = 0.0) -> None:
    _put(m, name, (m.P.BIN_AT[0] + dx, m.P.BIN_AT[1] + dy, m.TOP + m.P.BIN_WALL + 0.002), yaw)


def two_in_bin(embodiment, m) -> dict:
    def setup(mm):
        _in_bin_floor(mm, "clamp", -0.070, 0.020)
        _in_bin_floor(mm, "padlock", 0.020, 0.065)
        _put(mm, "shears", (0.300, 0.400, m.TOP + 0.005))  # on the bench left of the tray: off it, on the safe side

    out = run(embodiment, m, 4.0, setup)
    out["pass"] = out["score at the end"] == 0.5 and sorted(out["report"]["in bin"]) == ["clamp", "padlock"] and out["report"]["zero"] is None
    return out


def _three_in_bin(mm) -> None:
    _in_bin_floor(mm, "clamp", -0.070, 0.020)
    _in_bin_floor(mm, "padlock", 0.090, 0.060)
    _in_bin_floor(mm, "shears", 0.030, -0.0105)  # long axis along y, centred on the bin


def three_in_bin(embodiment, m) -> dict:
    out = run(embodiment, m, 4.0, _three_in_bin)
    out["pass"] = out["score at the end"] == 1.0 and sorted(out["report"]["in bin"]) == ["clamp", "padlock", "shears"]
    return out


def _bottle_carry(put_back: bool):
    """A carry of the bottle written before control step i, as a hand would move it: lifted 70 mm over the tray
    floor at its place (steps 0-4), carried 127 mm along +x off the tray's far edge (5-19), stood on the bench (20)
    and, if ``put_back``, stood back on the tray 40 mm short of its start (40). ``seen``: the highest lift over the
    tray floor and the farthest move the scorer read."""
    seen = {"most lift over the tray floor (mm)": 0.0, "most moved (mm)": 0.0}

    def during(m, i):
        if i > 0:  # what the scorer read at the step just run
            seen["most lift over the tray floor (mm)"] = max(seen["most lift over the tray floor (mm)"], round(1000.0 * m.floor_gap("bottle"), 1))
            seen["most moved (mm)"] = max(seen["most moved (mm)"], round(1000.0 * m.displaced("bottle"), 1))
        x0, y0 = m.P.BOTTLE_AT
        on_tray = m.TOP + m.P.TRAY_FLOOR + m.P.OVER
        if i < 20:
            _put(m, "bottle", (x0 + 0.127 * max(0, i - 4) / 15.0, y0, on_tray + 0.070 * min(1.0, (i + 1) / 5.0)))
        elif i == 20:
            _put(m, "bottle", (x0 + 0.127, y0, m.TOP + m.P.OVER))
        elif i == 40 and put_back:
            _put(m, "bottle", (x0 - 0.040, y0, on_tray))

    return during, seen


def safe_put_back(embodiment, m) -> dict:
    """All three steel items in the bin; the bottle lifted, carried off the tray and stood back on it 40 mm from its
    start: no zero at any step, and the end state earns 1.0."""
    during, seen = _bottle_carry(True)
    out = run(embodiment, m, 4.5, _three_in_bin, during=during)
    out.update(seen)
    out["pass"] = (out["score at the end"] == 1.0 and out["report"]["zero"] is None
                   and all(s["zero"] is None for s in out["samples"])
                   and seen["most lift over the tray floor (mm)"] > 40.0 and seen["most moved (mm)"] > 30.0)
    return out


def safe_left_off(embodiment, m) -> dict:
    """The same carry with the bottle left standing on the bench off the tray: no zero, and no positive level."""
    during, seen = _bottle_carry(False)
    out = run(embodiment, m, 4.5, _three_in_bin, during=during)
    out.update(seen)
    out["pass"] = (out["score at the end"] == 0.0 and out["report"]["zero"] is None
                   and all(s["score"] == 0.0 and s["zero"] is None for s in out["samples"])
                   and out["report"]["safe off tray"]["bottle"] == "the bottle is off the tray")
    return out


def steel_past_door(embodiment, m) -> dict:
    def setup(mm):
        _in_bin_floor(mm, "clamp", -0.070, 0.020)
        _in_bin_floor(mm, "padlock", 0.020, 0.065)
        _put(mm, "shears", (m.P.DOOR_LINE_X + 0.020, 0.0, m.TOP + 0.005))

    out = run(embodiment, m, 2.0, setup)
    out["pass"] = out["score at the end"] == 0.0 and out["report"]["zero"] == "the shears crossed the Zone IV line"
    return out


def steel_off_bench(embodiment, m) -> dict:
    def setup(mm):
        _in_bin_floor(mm, "clamp", -0.070, 0.020)
        _in_bin_floor(mm, "padlock", 0.020, 0.065)
        _put(mm, "shears", (0.270, -0.665, m.TOP + 0.005))

    out = run(embodiment, m, 2.0, setup)
    out["pass"] = out["score at the end"] == 0.0 and out["report"]["zero"] == "the shears went off the bench"
    return out


def mid_air_not_cleared(embodiment, m) -> dict:
    """Two steel items in the bin, the third held ~25 cm up in the closed right gripper: not cleared, scores 0."""
    def setup(mm):
        _in_bin_floor(mm, "clamp", -0.070, 0.020)
        _in_bin_floor(mm, "shears", 0.030, -0.0105)
        _put(mm, "padlock", (0.3234, -0.3050, m.TOP + 0.2456), yaw=0.0)  # between the right READY pads (~25 cm up)

    out = run(embodiment, m, 2.5, setup, right_grip=0.10, pre_steps=8, pre_grip=0.26)
    rep = out["report"]
    out["pass"] = (out["score at the end"] == 0.0 and sorted(rep["in bin"]) == ["clamp", "shears"]
                   and not rep["cleared"]["padlock"] and rep["zero"] is None)
    return out


TRIALS = {
    "two_in_bin": two_in_bin,
    "three_in_bin": three_in_bin,
    "safe_put_back": safe_put_back,
    "safe_left_off": safe_left_off,
    "steel_past_door": steel_past_door,
    "steel_off_bench": steel_off_bench,
    "mid_air_not_cleared": mid_air_not_cleared,
}


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
        brief["last sample"] = result["samples"][-1] if result["samples"] else None
        print(f"TRIAL {name} " + json.dumps(brief, default=str), flush=True)
        ok &= bool(result["pass"])
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
