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


def _retract_and_hover(embodiment, m, pos, rot) -> None:
    """Reset, retract the collar, and hold the hole saw still at ``pos`` / ``rot`` with a push cancelling its weight
    (the trial's stand-in for a hand holding it there)."""
    from inspect_robots import Scene

    P = m.P
    embodiment.reset(Scene(id="trial", instruction=m.INSTRUCTION), seed=0)
    m.mk.set_spring("collar_z", rest=-P.COLLAR_BEYOND)
    m.mk.set_joint("collar_z", 0.0)
    m.STATE["collar_up"] = False
    m.STATE["ever_retracted"] = True
    m.mk.write_pose("hole_saw", tuple(float(v) for v in pos), m.mk.matrix_to_quat(rot))
    m.mk.push("hole_saw", force=(0.0, 0.0, P.SAW_MASS * P.GRAVITY))


def _change(a, b) -> tuple[float, float]:
    """(mm, deg) between two poses (position, rotation matrix)."""
    dp = 1000.0 * float(np.linalg.norm(np.asarray(b[0]) - np.asarray(a[0])))
    c = (float(np.trace(np.asarray(a[1]).T @ np.asarray(b[1]))) - 1.0) / 2.0
    return round(dp, 4), round(math.degrees(math.acos(max(-1.0, min(1.0, c)))), 4)


def trial_catch_in_place(embodiment, m) -> dict:
    """The thread catches where the saw stands: held still 0.3 mm off-axis, level, 0.07 mm under its thread start
    (yaw -30 deg), it catches; its pose changes < 0.1 mm / 0.1 deg across the catch step and the step after; let go,
    the thread holds its weight."""
    P = m.P
    yaw = math.radians(-30.0)
    pos = (P.X_ARBOR + 0.0003, P.Y_ARBOR, m.TOP + P.Z_SHOULDER + P.thread_start_gap(yaw) - 0.00007)
    rot = m.mk.quat_to_matrix(m.mk.yaw_quat(yaw))
    _retract_and_hover(embodiment, m, pos, rot)
    act = _hold_action()
    poses, k_catch = [(np.array(pos), rot)], None
    for i in range(20):
        embodiment.step(act)
        poses.append(tuple(np.array(v, dtype=float) for v in m.saw_pose()))
        if k_catch is None and m.thread_engaged():
            k_catch = i + 1  # poses[k_catch] is the pose after the step whose on_step made the weld
        if k_catch is not None and len(poses) > k_catch + 1:
            break
    out = {"catch": m.STATE.get("catch"), "caught": k_catch is not None}
    if k_catch is not None:
        out["into_catch_step_mm_deg"] = _change(poses[k_catch - 1], poses[k_catch])
        out["after_catch_step_mm_deg"] = _change(poses[k_catch], poses[k_catch + 1])
    gap_caught = m.saw_shoulder_gap()
    m.mk.push("hole_saw", force=(0.0, 0.0, 0.0))
    _, rest = _step_for(embodiment, m, 1.0)
    out["gap_change_unheld_1s_mm"] = round(1000.0 * (m.saw_shoulder_gap() - gap_caught), 4)
    out["end_score"], out["engaged_at_end"] = rest["end_score"], m.thread_engaged()
    jumps = [out.get("into_catch_step_mm_deg", (9.0, 9.0)), out.get("after_catch_step_mm_deg", (9.0, 9.0))]
    out["pass"] = bool(out["caught"] and all(mm < 0.1 and deg < 0.1 for mm, deg in jumps)
                       and abs(out["gap_change_unheld_1s_mm"]) < 0.1 and out["engaged_at_end"] and out["end_score"] == 0.25)
    return out


def trial_cross_thread_no_catch(embodiment, m) -> dict:
    """A saw set on its thread start tilted 3 deg, or 0.6 mm off-axis, does not catch (it cross-threads): held there
    1.0 s it stays free; let go, it slides down the smooth boss and scores as the tiers say (0.25, collar retracted)."""
    P = m.P
    yaw = math.radians(-30.0)
    out, ok = {}, True
    for name, (dx, dy, tilt_deg) in {"tilted_3deg": (0.0, 0.0, 3.0), "off_axis_0p6mm": (0.000424, 0.000424, 0.0)}.items():
        rot = m.mk.quat_to_matrix(m.mk.yaw_quat(yaw)) @ m.mk.axis_angle((1.0, 0.0, 0.0), math.radians(tilt_deg))
        pos = (P.X_ARBOR + dx, P.Y_ARBOR + dy, m.TOP + P.Z_SHOULDER + P.thread_start_gap(yaw) - 0.00007)
        _retract_and_hover(embodiment, m, pos, rot)
        _, held = _step_for(embodiment, m, 1.0)
        m.mk.push("hole_saw", force=(0.0, 0.0, 0.0))
        _, after = _step_for(embodiment, m, 1.5)
        ever = any(r["thread_engaged"] for r in held["samples"] + after["samples"])
        out[name] = {"ever_engaged": ever, "held_xy_off_mm": round(1000.0 * math.hypot(dx, dy), 3), "tilt_deg": tilt_deg,
                     "end_gap_mm": after["report"]["shoulder_gap_mm"], "end_phase_deg": after["report"]["phase_err_deg"],
                     "end_score": after["end_score"]}
        ok &= (not ever) and after["end_score"] == 0.25
    out["pass"] = bool(ok)
    return out


TRIALS = {
    "catch_in_place": trial_catch_in_place,
    "cross_thread_no_catch": trial_cross_thread_no_catch,
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
