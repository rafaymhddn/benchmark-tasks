#!/usr/bin/env python3
"""Live PhysX trials of the score levels and whole-trial zero latches of ``tympanic_cover`` (states set directly).

    scripts/rta simpy <id> mech/tasks/tympanic_cover/trials.py /workspace/robocurve/outputs/<id>
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def _hold(left_grip: float = 1.0, right_grip: float = 1.0):
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, left_grip], np.r_[READY, right_grip]))


def _put(m, name: str, at, quat=(0.0, 0.0, 0.0, 1.0)) -> None:
    m.mk.write_pose(name, tuple(float(v) for v in at), tuple(float(v) for v in quat))


def _step_n(embodiment, n: int, left_grip: float = 1.0, right_grip: float = 1.0):
    act = _hold(left_grip=left_grip, right_grip=right_grip)
    res = None
    for _ in range(n):
        res = embodiment.step(act)
    return res


def _drop_used_in_tray(m) -> None:
    if not m.mk.is_cut("weld_used"):
        m.mk.cut("weld_used")
    _put(m, "cover_used", (m.P.TRAY_AT[0], m.P.TRAY_AT[1], m.TOP + m.P.OVER + m.P.TRAY_FLOOR + 0.002))


def half_ejected_in_tray(embodiment, m) -> dict:
    from inspect_robots import Scene

    embodiment.reset(Scene(id="half", instruction=m.INSTRUCTION), seed=0)
    _drop_used_in_tray(m)
    res = _step_n(embodiment, 35)
    score = float(res.info.get("score", 0.0))
    rep = m.report()
    return {"score": score, "report": rep, "pass": score == 0.5 and rep["used in tray"] and rep["zero"] is None}


def _seat_on_docked_probe(m, k: int) -> None:
    """The thermometer placed level in its cradle with fresh cover ``k`` seated on its probe (states set directly)."""
    tp = np.array(m.P.DOCK_ORIGIN, dtype=float) + np.array([0.0, 0.0, m.TOP + m.P.OVER])
    m.mk.cut(f"weld_well_{k}")
    _put(m, "thermometer", tp)
    _put(m, "eject_btn", tp + np.array(m.P.BTN_POS_LOCAL), m.mk.matrix_to_quat(m.P.BTN_ROT_T))
    _put(m, m.P.FRESH[k], tp + np.array(m.P.USED_ON_THERM_LOCAL), m.mk.matrix_to_quat(m.P.COVER_ON_PROBE_ROT_T))
    m.mk.make(f"weld_cone_{k}")


def full_docked(embodiment, m) -> dict:
    from inspect_robots import Scene

    embodiment.reset(Scene(id="full", instruction=m.INSTRUCTION), seed=0)
    _drop_used_in_tray(m)
    k = m.P.LOAD_WELL
    _seat_on_docked_probe(m, k)
    res = _step_n(embodiment, 35)
    score = float(res.info.get("score", 0.0))
    rep = m.report()
    return {
        "score": score,
        "report": rep,
        "pass": score == 1.0 and rep["mounted fresh"] == m.P.FRESH[k] and rep["docked in cradle"] and rep["zero"] is None,
    }


def zero_hand_touched_used(embodiment, m) -> dict:
    from inspect_robots import Scene
    from yam_arena import ik, station
    from yam_arena.constants import READY

    embodiment.reset(Scene(id="zero_hand", instruction=m.INSTRUCTION), seed=0)
    _step_n(embodiment, 5, right_grip=0.0)  # close right fingertips at READY
    tcp_r, _ = ik.tcp_pose(np.array(READY, dtype=float), np.array(station.BASES["right"], dtype=float))
    m.mk.cut("weld_used")
    # Place cover_used against the closed right gripper pad so PhysX registers rigid contact with right_robot:
    _put(m, "cover_used", (tcp_r[0], tcp_r[1], tcp_r[2] - 0.004))
    _step_n(embodiment, 4, right_grip=0.0)
    # Restore a clean 0.5 configuration; permanent zero latch must hold:
    _drop_used_in_tray(m)
    res = _step_n(embodiment, 30, right_grip=1.0)
    score = float(res.info.get("score", 0.0))
    rep = m.report()
    return {"score": score, "report": rep, "pass": score == 0.0 and rep["zero"] == "a hand touched the used cover"}


def zero_used_touched_cradle(embodiment, m) -> dict:
    from inspect_robots import Scene

    embodiment.reset(Scene(id="zero_cradle", instruction=m.INSTRUCTION), seed=0)
    m.mk.cut("weld_used")
    # Drop cover_used into the cradle's pocket so PhysX registers cover_used <-> cradle contact:
    _put(m, "cover_used", (m.P.CRADLE_AT[0], m.P.CRADLE_AT[1] + 0.055, m.TOP + 0.030))
    _step_n(embodiment, 6)
    _drop_used_in_tray(m)
    res = _step_n(embodiment, 30)
    score = float(res.info.get("score", 0.0))
    rep = m.report()
    return {"score": score, "report": rep, "pass": score == 0.0 and rep["zero"] == "the used cover touched the cradle or a fresh cover"}


def zero_used_missed_tray(embodiment, m) -> dict:
    from inspect_robots import Scene

    embodiment.reset(Scene(id="zero_miss", instruction=m.INSTRUCTION), seed=0)
    m.mk.cut("weld_used")
    # Drop ejected cover_used onto the open bench outside the grey tray:
    _put(m, "cover_used", (0.400, -0.120, m.TOP + 0.008))
    _step_n(embodiment, 6)
    _drop_used_in_tray(m)
    res = _step_n(embodiment, 30)
    score = float(res.info.get("score", 0.0))
    rep = m.report()
    return {"score": score, "report": rep, "pass": score == 0.0 and rep["zero"] == "the used cover missed the grey tray"}


def zero_fresh_off_probe(embodiment, m) -> dict:
    """A fresh cover seated on the docked probe, then pushed off it again (its cone weld cut, as the button would)."""
    from inspect_robots import Scene

    embodiment.reset(Scene(id="zero_fresh", instruction=m.INSTRUCTION), seed=0)
    _drop_used_in_tray(m)
    k = m.P.LOAD_WELL
    _seat_on_docked_probe(m, k)
    _step_n(embodiment, 5)
    m.mk.cut(f"weld_cone_{k}")
    res = _step_n(embodiment, 30)
    score = float(res.info.get("score", 0.0))
    rep = m.report()
    return {"score": score, "report": rep, "pass": score == 0.0 and rep["zero"] == "a fresh cover came off the probe again"}


TRIALS = {
    "half_ejected_in_tray": half_ejected_in_tray,
    "full_docked": full_docked,
    "zero_hand_touched_used": zero_hand_touched_used,
    "zero_used_touched_cradle": zero_used_touched_cradle,
    "zero_used_missed_tray": zero_used_missed_tray,
    "zero_fresh_off_probe": zero_fresh_off_probe,
}


def main() -> int:
    import demo_kit as kit

    out_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp/tympanic_trials")
    wanted = sys.argv[2].split(",") if len(sys.argv) > 2 else list(TRIALS)
    out_dir.mkdir(parents=True, exist_ok=True)

    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    summary = {}
    all_ok = True
    for name in wanted:
        res = TRIALS[name](emb, m)
        (out_dir / f"{name}.json").write_text(json.dumps(res, indent=1, default=str))
        summary[name] = {"pass": bool(res["pass"]), "score": res["score"], "zero": res["report"]["zero"]}
        all_ok &= bool(res["pass"])
    print("G2_TRIALS " + json.dumps({"pass": all_ok, "trials": summary}, indent=1), flush=True)
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
