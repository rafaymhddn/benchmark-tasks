#!/usr/bin/env python3
"""Gate 2 PhysX trials for ``bench_can``: exactly one trial per score level (0.0, 0.25, 0.5, 1.0) and per whole-trial
zero rule (can tipped > 12 deg, can slid > 30 mm when unbraced, bearing off bench, tray pushed > 25 mm).

Note: ``set_joint`` and ``write_pose`` are used strictly here in ``trials.py`` to initialize per-rule test states
(never in ``task.py``, ``scene.py``, or ``demo.py``).

    scripts/rta simpy <id> mech/tasks/bench_can/trials.py /workspace/robocurve/outputs/<id>
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

SAMPLE_S = 0.25


def _ready_action():
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))


def _put_bearing_in_tray(m, name: str, xy: tuple[float, float]) -> None:
    z = m.TOP + m.P.TRAY_FLOOR + 0.5 * m.P.BRG_W + 0.001
    m.mk.write_pose(name, (xy[0], xy[1], z), (0.0, 0.0, 0.0, 1.0))


def _set_lid_deg(m, deg_from_closed: float) -> None:
    """``lid`` was built at 172 deg open (travel = 0), so ``deg_from_closed`` corresponds to travel ``deg - 172``."""
    m.mk.set_joint("lid", math.radians(deg_from_closed - m.P.LID_OPEN_DEG))
    m.mk._after_reset()  # re-anchor dry-friction hold at the placed lid angle


def _run_trial(embodiment, m, seconds: float, setup, action_fn=None) -> dict:
    from inspect_robots import Scene

    embodiment.reset(Scene(id="trial", instruction=m.INSTRUCTION), seed=0)
    setup(m)
    hz = float(embodiment.info.control_hz)
    every = max(1, int(round(SAMPLE_S * hz)))
    hold = _ready_action()
    samples = []
    res = None
    for i in range(int(round(seconds * hz))):
        t = (i + 1) / hz
        act = hold if action_fn is None else action_fn(i / hz)
        res = embodiment.step(act)
        if (i + 1) % every == 0:
            samples.append({
                "t_s": round(t, 3),
                "lid_deg": round(m.lid_deg(), 2),
                "dash_mm": round(1000.0 * m.dash_travel(), 2),
                "can_slide_mm": round(1000.0 * m.can_slide(), 2),
                "can_tilt_deg": round(m.can_tilt_deg(), 2),
                "tray_shift_mm": round(1000.0 * m.tray_shift(), 2),
                "dasher_on_brg_mat_n": round(m.mk.contact_force("dasher", "brg_mat"), 2),
                "lid_on_brg_screen_n": round(m.mk.contact_force("lid", "brg_screen"), 2),
                "dipped": bool(m.STATE.get("dipped", False)),
                "score": float(res.info.get("score", 0.0)),
            })
    return {
        "end_score": float(res.info.get("score", 0.0)),
        "report": m.report(),
        "samples": samples,
    }


def lid_blocked_open_zero(embodiment, m) -> dict:
    """Level 0.0: lowering lid onto brg_screen on dasher physically stalls ajar (~23 deg > 4 deg) on brg_screen -> 0.0."""
    def setup(mm):
        # Release lid from 24 deg with moving friction (held=None) so gravity drops it onto brg_screen at ~23 deg ajar
        mm.mk.set_joint("lid", math.radians(24.0 - mm.P.LID_OPEN_DEG))
        mm.mk.MECH.held["lid"] = None
        mm.mk._write_friction(mm.mk.MECH.joints["lid"], None)
        mm.begin(0)

    out = _run_trial(embodiment, m, 1.5, setup)
    last = out["samples"][-1]
    out["pass"] = out["end_score"] == 0.0 and 8.0 <= last["lid_deg"] <= 26.0 and out["report"]["zero"] is None
    return out


def can_tipped_zero(embodiment, m) -> dict:
    """Zero rule: can knocked on its side (> 12 deg) latches 0.0."""
    def setup(mm):
        c = mm.scene.can_start_pos(mm.TOP) + np.array([0.0, 0.0, 0.080])
        q = tuple(float(v) for v in mm.mk.matrix_to_quat(mm.mk.axis_angle((1.0, 0.0, 0.0), math.radians(75.0))))
        mm.mk.write_pose("can", c, q)
        mm.begin(0)

    out = _run_trial(embodiment, m, 1.0, setup)
    out["pass"] = out["end_score"] == 0.0 and out["report"]["zero"] == "the can tipped more than 12 degrees"
    return out


def can_slid_unbraced_zero(embodiment, m) -> dict:
    """Zero rule: sweeping the 172 deg lid with Left arm alone (Right arm at READY, not bracing d_handle) slides the
    0.98 kg can > 30 mm across the bench -> 0.0."""
    import checks as K
    import demo_kit as kit
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import station
    from yam_arena.constants import READY

    moves = [
        kit.Move(1.2, pos=K.lid_skirt_point(172.0) + np.array([0.0, 0.030, 0.040]), rot=K.LEFT_LID_ROT,
                 grip=K.SKIRT_OPEN_M / K.MAX_JAW, label="approach"),
        kit.Move(0.8, pos=K.lid_skirt_point(172.0), label="on skirt"),
        kit.Move(0.5, grip=0.0, label="pinch skirt"),
        kit.Move(1.8, pos=K.lid_skirt_point(80.0) + np.array([0.080, -0.085, 0.0]), label="pull lid unbraced"),
    ]
    arm_l = kit.Arm("left", station.BASES["left"], moves)
    ready_r = np.r_[READY, 1.0]

    def act_fn(t: float):
        act_l, _ = arm_l.action(t)
        return Action(data=pack(act_l, ready_r))

    def setup(mm):
        _put_bearing_in_tray(mm, "brg_screen", mm.P.TRAY_SLOT_BACK_XY)
        mm.begin(0)

    out = _run_trial(embodiment, m, 4.3, setup, action_fn=act_fn)
    out["pass"] = out["end_score"] == 0.0 and out["report"]["max_can_slide_mm"] > 30.0 and out["report"]["zero"] is not None
    return out


def part_off_bench_zero(embodiment, m) -> dict:
    """Zero rule: bearing off the bench (< TOP - 20 mm) latches 0.0."""
    def setup(mm):
        _put_bearing_in_tray(mm, "brg_screen", mm.P.TRAY_SLOT_BACK_XY)
        _set_lid_deg(mm, 0.0)
        mm.mk.write_pose("brg_mat", (-0.25, -0.25, mm.TOP - 0.050), (0.0, 0.0, 0.0, 1.0))
        mm.begin(0)

    out = _run_trial(embodiment, m, 1.0, setup)
    out["pass"] = out["end_score"] == 0.0 and out["report"]["zero"] == "a bearing fell off the bench"
    return out


def tray_pushed_zero(embodiment, m) -> dict:
    """Zero rule: tray shifted > 25 mm from start mark latches 0.0."""
    def setup(mm):
        t_pos = mm.scene.tray_start_pos(mm.TOP) + np.array([0.032, 0.0, 0.0])
        mm.mk.write_pose("tray", t_pos, (0.0, 0.0, 0.0, 1.0))
        _put_bearing_in_tray(mm, "brg_screen", (mm.P.TRAY_SLOT_BACK_XY[0] + 0.032, mm.P.TRAY_SLOT_BACK_XY[1]))
        _set_lid_deg(mm, 0.0)
        mm.begin(0)

    out = _run_trial(embodiment, m, 1.0, setup)
    out["pass"] = out["end_score"] == 0.0 and "the tray shifted more than 25 mm" in (out["report"]["zero"] or "")
    return out


def cleared_closed_quarter(embodiment, m) -> dict:
    """Level 0.25: left-in bearing cleared into tray and lid closed (<= 4 deg), second bearing not dipped."""
    def setup(mm):
        _put_bearing_in_tray(mm, "brg_screen", mm.P.TRAY_SLOT_BACK_XY)
        _set_lid_deg(mm, 0.0)
        mm.begin(0)

    out = _run_trial(embodiment, m, 1.5, setup)
    out["pass"] = out["end_score"] == 0.25 and out["report"]["zero"] is None
    return out


def dipped_closed_half(embodiment, m) -> dict:
    """Level 0.5: brg_mat physically pressed >= 12 mm on dasher for >= 0.8 s, then returned to mat and lid closed."""
    import checks as K
    import demo_kit as kit
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import station
    from yam_arena.constants import READY

    cx, cy = m.P.CAN_XY
    mx, my = m.P.MAT_XY
    z_top = m.TOP + 0.089
    moves = [
        kit.Move(1.0, pos=(cx, cy, m.TOP + 0.125), rot=K.RIGHT_DOWN, grip=K.BRG_OPEN_M / K.MAX_JAW, label="above brg"),
        kit.Move(0.7, pos=(cx, cy, z_top), label="around brg"),
        kit.Move(0.5, grip=K.BRG_PINCH_M / K.MAX_JAW, label="pinch brg"),
        kit.Move(0.7, pos=(cx, cy, z_top - 0.015), push=(0.0, 0.0, -22.0), label="plunge 15 mm"),
        kit.Move(1.2, pos=(cx, cy, z_top - 0.015), push=(0.0, 0.0, -22.0), label="hold dip 1.2 s"),
        kit.Move(0.8, pos=(cx, cy, m.TOP + 0.125), push=(0.0, 0.0, 0.0), label="lift out"),
        kit.Move(1.1, pos=(mx, my, m.TOP + 0.125), label="over mat"),
        kit.Move(0.8, pos=(mx, my, m.TOP + m.P.MAT_T + 0.5 * m.P.BRG_W + 0.003), grip=K.BRG_OPEN_M / K.MAX_JAW, label="leave on mat"),
        kit.Move(0.8, pos=(mx, my, m.TOP + 0.125), grip=1.0, label="retract"),
    ]
    arm_r = kit.Arm("right", station.BASES["right"], moves)
    ready_l = np.r_[READY, 1.0]
    closed_done = [False]

    def act_fn(t: float):
        if t >= 7.7 and not closed_done[0]:
            _set_lid_deg(m, 0.0)
            closed_done[0] = True
        act_r, _ = arm_r.action(t) if t < 7.6 else (np.r_[READY, 1.0], "ready")
        return Action(data=pack(ready_l, act_r))

    def setup(mm):
        _put_bearing_in_tray(mm, "brg_screen", mm.P.TRAY_SLOT_BACK_XY)
        c = mm.scene.can_start_pos(mm.TOP)
        mm.mk.write_pose("brg_mat", (float(c[0]), float(c[1]), float(c[2]) + mm.P.DASH_REST_Z + 0.5 * mm.P.BRG_W + 0.0005),
                         (0.0, 0.0, 0.0, 1.0))
        mm.begin(0)

    out = _run_trial(embodiment, m, 8.5, setup, action_fn=act_fn)
    peak_n = max((s["dasher_on_brg_mat_n"] for s in out["samples"]), default=0.0)
    out["peak_dasher_on_brg_mat_n"] = peak_n
    out["pass"] = (
        out["end_score"] == 0.5
        and out["report"]["dipped"]
        and out["report"]["max_dash_mm"] >= 12.0
        and peak_n >= 12.0
        and out["report"]["zero"] is None
    )
    return out


def both_trayed_closed_full(embodiment, m) -> dict:
    """Level 1.0: both bearings flat in tray, dip completed, dasher sprung back up, lid closed, hands clear, 1.0 s hold."""
    def setup(mm):
        _put_bearing_in_tray(mm, "brg_screen", mm.P.TRAY_SLOT_BACK_XY)
        _put_bearing_in_tray(mm, "brg_mat", mm.P.TRAY_SLOT_FRONT_XY)
        _set_lid_deg(mm, 0.0)
        mm.begin(0)
        mm.STATE["dipped"] = True
        mm.STATE["max_dip_steps"] = mm.P.DIP_HOLD_STEPS

    out = _run_trial(embodiment, m, 1.5, setup)
    out["pass"] = out["end_score"] == 1.0 and out["report"]["zero"] is None
    return out


TRIALS = {
    "lid_blocked_open_zero": lid_blocked_open_zero,
    "can_tipped_zero": can_tipped_zero,
    "can_slid_unbraced_zero": can_slid_unbraced_zero,
    "part_off_bench_zero": part_off_bench_zero,
    "tray_pushed_zero": tray_pushed_zero,
    "cleared_closed_quarter": cleared_closed_quarter,
    "dipped_closed_half": dipped_closed_half,
    "both_trayed_closed_full": both_trayed_closed_full,
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
        brief["last_sample"] = result["samples"][-1] if result["samples"] else None
        print(f"TRIAL {name} " + json.dumps(brief, default=str), flush=True)
        ok &= bool(result["pass"])
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
