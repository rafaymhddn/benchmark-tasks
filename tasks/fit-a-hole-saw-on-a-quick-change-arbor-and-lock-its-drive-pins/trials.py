#!/usr/bin/env python3
"""Simulator mechanism & score-level trials for holesaw_arbor (arms held at READY).

    scripts/rta simpy <id> mech/tasks/holesaw_arbor/trials.py /workspace/robocurve/outputs/<id>
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def _hold_action():
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))


def _step_for(embodiment, m, seconds: float, per_step=None) -> tuple[list[dict], dict]:
    hz = float(embodiment.info.control_hz)
    n = int(round(seconds * hz))
    act = _hold_action()
    rows = []
    res = None
    for i in range(n):
        if per_step is not None:
            per_step(i, (i + 1) / hz)
        res = embodiment.step(act)
        if (i + 1) % 5 == 0 or i == n - 1:
            rep = m.report()
            rows.append({
                "t_s": round((i + 1) / hz, 2),
                "collar_z_mm": rep["collar_z_mm"],
                "thread_engaged": rep["thread_engaged"],
                "thread_cw_deg": rep["thread_cw_deg"],
                "shoulder_gap_mm": rep["shoulder_gap_mm"],
                "phase_err_deg": rep["phase_err_deg"],
                "pin_contact_N": rep["pin_contact_N"],
                "score": float(res.info.get("score", 0.0)),
            })
    assert res is not None
    return rows, {"end_score": float(res.info.get("score", 0.0)), "report": m.report(), "samples": rows}


def _place_cup_and_catch(embodiment, m, cw_deg: float) -> None:
    """Retract collar to 0 mm and mount hole_saw on the helical thread at CW travel ``cw_deg``."""
    from inspect_robots import Scene

    P = m.P
    embodiment.reset(Scene(id="trial", instruction=m.INSTRUCTION), seed=0)
    m.mk.cut("w_thread")
    m.mk.set_spring("collar_z", rest=-P.COLLAR_BEYOND)
    m.mk.set_joint("collar_z", 0.0)
    m.STATE["collar_up"] = False
    m.STATE["ever_retracted"] = True

    per = math.radians(cw_deg) / 3.0
    z_gap = P.gap_for_yaw_deg(cw_deg)
    z_rel = z_gap - (P.Z_ENTRY - P.Z_SHOULDER)
    m.mk.set_limits("cup_z", lower=z_rel - P.SLIDE_PLAY, upper=z_rel + P.SLIDE_PLAY)
    m.mk.set_joint("cup_z", z_rel)
    for j in m.scene.TURN_JOINTS:
        m.mk.set_joint(j, per)
        m.mk.set_spring(j, rest=per, most=P.THREAD_DRAG_NM)
        m.STATE["turn_rest"][j] = per
    m.mk.write_pose("hole_saw", (P.X_ARBOR, P.Y_ARBOR, m.TOP + P.Z_SHOULDER + z_gap), m.mk.yaw_quat(math.radians(-cw_deg)))
    m.mk.make("w_thread")


def trial_dropped_zero(embodiment, m) -> dict:
    """Whole-trial zero rule: hole_saw dropped off the bench edge latches score 0.0 permanently."""
    from inspect_robots import Scene

    embodiment.reset(Scene(id="drop", instruction=m.INSTRUCTION), seed=0)
    m.mk.write_pose("hole_saw", (m.P.X_MAT, 0.72, m.TOP - 0.025), m.mk.yaw_quat(0.0))
    _, out = _step_for(embodiment, m, 0.8)
    out["pass"] = out["end_score"] == 0.0 and out["report"]["zero"] is not None
    return out


def trial_collar_up_blocked(embodiment, m) -> dict:
    """Shortcut check (0.0): placing hole_saw onto arbor with collar left UP rests high on the 7 mm pins."""
    from inspect_robots import Scene

    P = m.P
    embodiment.reset(Scene(id="blocked", instruction=m.INSTRUCTION), seed=0)
    # Lower hole_saw at yaw = 35 deg right onto the extended pins (z_pin_top = Z_SHOULDER + 7.0 mm)
    m.mk.write_pose("hole_saw", (P.X_ARBOR, P.Y_ARBOR, m.TOP + P.Z_SHOULDER + 0.0075), m.mk.yaw_quat(math.radians(35.0)))
    _, out = _step_for(embodiment, m, 1.5)
    rep = out["report"]
    out["pass"] = (
        out["end_score"] == 0.0
        and not rep["thread_engaged"]
        and rep["shoulder_gap_mm"] >= 6.0
        and rep["peak_pin_block_N"] >= 1.0
    )
    return out


def trial_bottomed_out_stall(embodiment, m) -> dict:
    """Shortcut check (0.25): cup screwed tight to +415 deg stop (holes 55 deg past pins); 22 N upward collar push stalls on backplate."""
    _place_cup_and_catch(embodiment, m, m.P.STOP_DEG)

    def push_collar(_i: int, t_s: float) -> None:
        m.mk.push("collar", force=(0.0, 0.0, 22.0 if t_s <= 1.2 else 0.0))

    _, out = _step_for(embodiment, m, 2.0, per_step=push_collar)
    rep = out["report"]
    max_cz = max(r["collar_z_mm"] for r in out["samples"])
    out["max_collar_z_under_22N_mm"] = max_cz
    out["pass"] = (
        out["end_score"] == 0.25
        and max_cz < 2.2
        and rep["peak_pin_block_N"] >= 12.0
        and rep["phase_err_deg"] >= 50.0
    )
    return out


def trial_aligned_half(embodiment, m) -> dict:
    """0.5 level: cup backed off 55 deg CCW from stop to +360.0 deg (gap 0.22 mm, aligned), collar left retracted."""
    _place_cup_and_catch(embodiment, m, m.P.ALIGN_DEG)
    _, out = _step_for(embodiment, m, 1.5)
    rep = out["report"]
    out["pass"] = (
        out["end_score"] == 0.5
        and rep["shoulder_gap_mm"] <= 0.60
        and rep["phase_err_deg"] <= 2.0
        and rep["collar_z_mm"] <= 1.0
    )
    return out


def trial_unthreaded_release(embodiment, m) -> dict:
    """Spec-check requirement: turning cup CCW back past 0 deg cuts w_thread cleanly."""
    _place_cup_and_catch(embodiment, m, 18.0)

    def unscrew(_i: int, t_s: float) -> None:
        m.mk.push("hole_saw", force=(0.0, 0.0, 4.0 if t_s > 1.0 else 0.0), torque=(0.0, 0.0, 0.14 if t_s <= 1.2 else 0.0))

    _, out = _step_for(embodiment, m, 1.8, per_step=unscrew)
    rep = out["report"]
    out["pass"] = not rep["thread_engaged"] and rep["shoulder_gap_mm"] > 3.0 and out["end_score"] == 0.25
    return out


def trial_locked_twist_hold(embodiment, m) -> dict:
    """1.0 level & rotational lock check: at +360 deg, 22 N push drives pins through both 7.8 mm holes; resists 0.30 N m CCW twist."""
    _place_cup_and_catch(embodiment, m, m.P.ALIGN_DEG)

    def push_and_twist(_i: int, t_s: float) -> None:
        m.mk.push("collar", force=(0.0, 0.0, 22.0 if t_s <= 0.8 else 0.0))
        # Apply 0.30 N m CCW torque (+z) from 1.0..1.8 s (5x the 0.06 N m thread drag!), then release to settle for 1.5 s
        m.mk.push("hole_saw", torque=(0.0, 0.0, 0.30 if 1.0 <= t_s <= 1.8 else 0.0))

    _, out = _step_for(embodiment, m, 3.5, per_step=push_and_twist)
    rep = out["report"]
    max_phase = max(r["phase_err_deg"] for r in out["samples"])
    out["max_phase_under_0p30Nm_deg"] = max_phase
    out["pass"] = (
        out["end_score"] == 1.0
        and rep["collar_z_mm"] >= 7.5
        and max_phase <= 5.0
    )
    return out


TRIALS = {
    "dropped_zero": trial_dropped_zero,
    "collar_up_blocked": trial_collar_up_blocked,
    "bottomed_out_stall": trial_bottomed_out_stall,
    "aligned_half": trial_aligned_half,
    "unthreaded_release": trial_unthreaded_release,
    "locked_twist_hold": trial_locked_twist_hold,
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
        res = TRIALS[name](embodiment, m)
        (out / f"{name}.json").write_text(json.dumps(res, indent=2, default=str))
        brief = {k: v for k, v in res.items() if k != "samples"}
        print(f"TRIAL {name} " + json.dumps(brief, default=str), flush=True)
        ok &= bool(res["pass"])
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
