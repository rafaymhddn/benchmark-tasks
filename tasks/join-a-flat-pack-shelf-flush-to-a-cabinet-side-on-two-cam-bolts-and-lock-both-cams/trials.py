#!/usr/bin/env python3
"""PhysX mechanics and scoring trials for the Minifix 15 cam-lock shelf task (Gate 2), run in one simulator pass:
- ``closed_cam_blocked``: pushing with +15 N while cams stay 90 deg closed stops ~8.5 mm short in PhysX -> score 0.0;
- ``released_driver_tips``: an unheld stubby placed upright in the 3.5 mm cam recess topples past 45 deg under gravity;
- ``seated_unlocked_quarter``: cams open, shelf seated on both bolts -> score 0.25 (slides off under a 10 N -x pull);
- ``one_cam_locked_half``: shelf seated, left cam locked at 178 deg CW (>= 5 N PhysX clamp), right open -> score 0.50;
- ``both_locked_full``: both cams locked at 178 deg CW, seam flush, tools in cradles -> score 1.00 & holds 10 N -x pull;
- ``zero_shelf_drop``, ``zero_driver_drop``: each whole-trial zero rule fires in PhysX -> 0.0.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def _hold_ready():
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))


def _quat(m, rot: np.ndarray) -> tuple[float, ...]:
    return tuple(float(v) for v in m.mk.matrix_to_quat(rot))


def _place_shelf_and_cams(m, gap: float, cam_l_deg: float, cam_r_deg: float, tilt_deg: float = 0.0, yaw_deg: float = 0.0, drop: float = 0.0) -> None:
    p = m.P
    rot = m.mk.axis_angle((0.0, 0.0, 1.0), math.radians(yaw_deg)) @ m.mk.axis_angle((0.0, 1.0, 0.0), math.radians(tilt_deg))
    pos = np.array([p.X_SIDE - gap - p.SHELF_D / 2.0, 0.0, m.TOP + p.Z_MID - drop])
    m.mk.write_pose("shelf", pos, _quat(m, rot))
    for c, cdeg in (("l", cam_l_deg), ("r", cam_r_deg)):
        trav = p.travel_from_cam_deg(cdeg)
        c_pos, c_rot = m.mk.joint_pose(m.mk.MECH.joints[f"cam_joint_{c}"], trav, (pos, rot))
        m.mk.write_pose(f"cam_{c}", c_pos, _quat(m, c_rot))
        m.mk.MECH.travel[f"cam_joint_{c}"] = trav
    m.mk._after_reset()  # re-anchor dry-friction holds to the placed cam angles


def _step_for(emb, seconds: float, on_before_step=None):
    hz = float(emb.info.control_hz)
    act = _hold_ready()
    res = None
    for i in range(int(round(seconds * hz))):
        if on_before_step is not None:
            on_before_step(i, (i + 1) / hz)
        res = emb.step(act)
    return res


def closed_cam_blocked(emb, m) -> dict:
    from inspect_robots import Scene

    emb.reset(Scene(id="t_blocked", instruction=m.INSTRUCTION), seed=0)
    # Push shelf +x toward cabinet_side with 15 N while both cams stay at their +90 deg closed start pose
    _step_for(emb, 1.5, lambda _i, t: m.mk.push("shelf", force=(15.0, 0.0, 0.0) if t <= 1.2 else (0.0, 0.0, 0.0)))
    rep = m.report()
    gaps = rep["seam_gaps_mm"]
    ok = (
        7.5 <= gaps["l"] <= 10.5
        and 7.5 <= gaps["r"] <= 10.5
        and rep["score"] == 0.0
        and rep["zero"] is None
    )
    return {"pass": bool(ok), "seam_gaps_mm": gaps, "score": rep["score"], "report": rep}


def released_driver_tips(emb, m) -> dict:
    from inspect_robots import Scene

    emb.reset(Scene(id="t_tip", instruction=m.INSTRUCTION), seed=0)
    p_cam, _ = m.mk.built_pose("cam_l")
    # Place driver_l standing on cam_l's 3.5 mm-deep recess floor with a tiny 1.5 deg release imperfection and no hand holding it
    r_almost_up = m.mk.axis_angle((1.0, 0.0, 0.0), math.radians(1.5)) @ np.diag([1.0, -1.0, -1.0])
    tip_at = np.array([p_cam[0], p_cam[1], m.TOP + m.P.Z_SHELF_TOP - m.P.SLOT_RECESS_DEPTH + 0.0003])
    drv_pos = tip_at - r_almost_up @ np.array([0.0, 0.0, m.P.DRV_TIP_Z])
    m.mk.write_pose("driver_l", drv_pos, _quat(m, r_almost_up))
    max_tilt = 0.0
    hz = float(emb.info.control_hz)
    act = _hold_ready()
    for _ in range(int(round(1.5 * hz))):
        emb.step(act)
        _, axis = m.driver_tip("l")
        tilt = float(math.degrees(math.acos(max(-1.0, min(1.0, float(-axis[2]))))))
        max_tilt = max(max_tilt, tilt)
    return {"pass": max_tilt > 45.0, "max_driver_tilt_deg": round(max_tilt, 1), "report": m.report()}


def seated_unlocked_quarter(emb, m) -> dict:
    from inspect_robots import Scene

    emb.reset(Scene(id="t_quarter", instruction=m.INSTRUCTION), seed=0)
    _place_shelf_and_cams(m, gap=0.0012, cam_l_deg=0.0, cam_r_deg=0.0)
    res = _step_for(emb, 1.0)
    q_score = float(res.info.get("score", 0.0))
    # Verify unlocked shelf pulls right back off under a -10 N pull
    _step_for(emb, 0.9, lambda _i, _t: m.mk.push("shelf", force=(-10.0, 0.0, 0.0)))
    m.mk.push("shelf")
    pulled_gaps = m.report()["seam_gaps_mm"]
    ok = q_score == 0.25 and min(pulled_gaps.values()) > 8.0
    return {"pass": bool(ok), "score_seated": q_score, "gaps_after_10N_pull_mm": pulled_gaps}


def one_cam_locked_half(emb, m) -> dict:
    from inspect_robots import Scene

    emb.reset(Scene(id="t_half", instruction=m.INSTRUCTION), seed=0)
    _place_shelf_and_cams(m, gap=0.0002, cam_l_deg=178.0, cam_r_deg=0.0)
    res = _step_for(emb, 1.5)
    rep = m.report()
    score = float(res.info.get("score", 0.0))
    ok = score == 0.50 and rep["cam_clamp_n"]["l"] >= m.P.CAM_CLAMP_N and not rep["cam_locked"]["r"]
    return {"pass": bool(ok), "score": score, "report": rep}


def both_locked_full(emb, m) -> dict:
    from inspect_robots import Scene

    emb.reset(Scene(id="t_full", instruction=m.INSTRUCTION), seed=0)
    _place_shelf_and_cams(m, gap=0.0001, cam_l_deg=178.0, cam_r_deg=178.0)
    res = _step_for(emb, 1.5)
    rep = m.report()
    score = float(res.info.get("score", 0.0))
    # Apply 10 N -x pull-off load on the locked shelf and verify seam stays <= 0.5 mm
    _step_for(emb, 0.5, lambda _i, _t: m.mk.push("shelf", force=(-10.0, 0.0, 0.0)))
    m.mk.push("shelf")
    pull_gaps = m.report()["seam_gaps_mm"]
    ok = (
        score == 1.0
        and rep["cam_clamp_n"]["l"] >= m.P.CAM_CLAMP_N
        and rep["cam_clamp_n"]["r"] >= m.P.CAM_CLAMP_N
        and max(pull_gaps.values()) <= 0.50
    )
    return {"pass": bool(ok), "score": score, "gaps_under_10N_pull_mm": pull_gaps, "report": rep}


def zero_shelf_drop(emb, m) -> dict:
    from inspect_robots import Scene

    emb.reset(Scene(id="z_sdrop", instruction=m.INSTRUCTION), seed=0)
    # Place shelf level on the bench behind the support block (tilt == 0 deg, z == 8 mm < 56 mm)
    pos = np.array([0.085, 0.0, m.TOP + m.P.PANEL_T / 2.0 + 0.001])
    m.mk.write_pose("shelf", pos, (0.0, 0.0, 0.0, 1.0))
    for c in ("l", "r"):
        c_pos, c_rot = m.mk.joint_pose(m.mk.MECH.joints[f"cam_joint_{c}"], 0.0, (pos, np.eye(3)))
        m.mk.write_pose(f"cam_{c}", c_pos, _quat(m, c_rot))
    res = _step_for(emb, 0.3)
    rep = m.report()
    return {"pass": float(res.info.get("score", 0.0)) == 0.0 and rep["zero"] == "the shelf dropped off the support block", "report": rep}


def zero_driver_drop(emb, m) -> dict:
    from inspect_robots import Scene

    emb.reset(Scene(id="z_ddrop", instruction=m.INSTRUCTION), seed=0)
    # Place driver_r lying on its side on the bench (z ~ 15 mm < 32 mm)
    r_side = m.mk.axis_angle((0.0, 1.0, 0.0), math.pi / 2.0)
    m.mk.write_pose("driver_r", (0.22, -0.32, m.TOP + 0.016), _quat(m, r_side))
    res = _step_for(emb, 0.3)
    rep = m.report()
    return {"pass": float(res.info.get("score", 0.0)) == 0.0 and rep["zero"] == "screwdriver r fell to the bench", "report": rep}


TRIALS = {
    "closed_cam_blocked": closed_cam_blocked,
    "released_driver_tips": released_driver_tips,
    "seated_unlocked_quarter": seated_unlocked_quarter,
    "one_cam_locked_half": one_cam_locked_half,
    "both_locked_full": both_locked_full,
    "zero_shelf_drop": zero_shelf_drop,
    "zero_driver_drop": zero_driver_drop,
}


def run_suite(emb, m, out: Path, names: list[str] | None = None) -> tuple[bool, dict]:
    out.mkdir(parents=True, exist_ok=True)
    ok = True
    summary = {}
    for name in (names or list(TRIALS)):
        result = TRIALS[name](emb, m)
        (out / f"{name}.json").write_text(json.dumps(result, indent=1, default=str))
        summary[name] = {k: v for k, v in result.items() if k != "report"}
        summary[name]["zero"] = result.get("report", {}).get("zero")
        print(f"TRIAL {name} " + json.dumps(summary[name], default=str), flush=True)
        ok &= bool(result["pass"])
    (out / "trials_summary.json").write_text(json.dumps({"pass": ok, "trials": summary}, indent=1, default=str))
    return ok, summary


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    names = sys.argv[2].split(",") if len(sys.argv) > 2 else list(TRIALS)
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    ok, _ = run_suite(emb, m, out, names)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
