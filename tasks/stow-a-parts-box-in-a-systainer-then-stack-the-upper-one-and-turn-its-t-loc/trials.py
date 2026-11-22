#!/usr/bin/env python3
"""Gate-2 PhysX mechanics, shortcut-blocking, zero-rule, and 15 N coupled-lift trials for ``systainer_tloc``.

Runs all trials in a single simulator startup:
    scripts/rta simpy gs2 mech/tasks/systainer_tloc/trials.py /workspace/robocurve/outputs/gs2
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def _ready_action():
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))


def _q(m, rot) -> tuple[float, float, float, float]:
    return tuple(float(v) for v in m.mk.matrix_to_quat(rot))


def _reset_base(embodiment, m, *, lid_deg: float, tloc_deg: float, stow_box: bool = True) -> None:
    from inspect_robots import Scene

    embodiment.reset(Scene(id="trial", instruction=m.INSTRUCTION), seed=0)
    P = m.P
    p_low = np.array([P.CASE_LOW_XY[0], P.CASE_LOW_XY[1], m.TOP + 0.0005], dtype=float)
    m.mk.write_pose("case_low", tuple(p_low), (0.0, 0.0, 0.0, 1.0))
    lid_tr = math.radians(lid_deg) - P.LID_OPEN_RAD
    tloc_tr = math.radians(tloc_deg) - P.TLOC_OPEN_RAD
    m.mk.set_joint("lid_hinge", lid_tr)
    m.mk.set_spring("lid_hinge", rest=P.lid_stay_travel_rest(lid_tr))
    m.mk.set_joint("tloc", tloc_tr)
    m.mk.set_spring("tloc", rest=P.tloc_travel_rest(tloc_tr))
    if stow_box:
        bx = P.CASE_LOW_XY[0] + P.CELL_LOCAL_XY[0]
        by = P.CASE_LOW_XY[1] + P.CELL_LOCAL_XY[1]
        m.mk.write_pose("parts_box", (bx, by, m.TOP + P.FLOOR_TOP_Z + 0.001), (0.0, 0.0, 0.0, 1.0))


def _step_seconds(embodiment, seconds: float):
    hz = float(embodiment.info.control_hz)
    act = _ready_action()
    res = None
    for _ in range(int(round(seconds * hz))):
        res = embodiment.step(act)
    return res


def trial_cocked_box_blocks_lid(embodiment, m) -> dict:
    P = m.P
    _reset_base(embodiment, m, lid_deg=18.0, tloc_deg=P.TLOC_OPEN_DEG, stow_box=False)
    # Place parts_box cocked across the interior partition rib so lid_low stalls onto it above 6 deg
    x_rib = P.CASE_LOW_XY[0] - P.INNER_D / 2.0 + P.CELL_D
    y_rib = P.CASE_LOW_XY[1] + P.INNER_W / 2.0 - P.CELL_W
    rot = m.mk.axis_angle((1.0, 1.0, 0.0), math.radians(18.0))
    m.mk.write_pose("parts_box", (x_rib, y_rib, m.TOP + P.FLOOR_TOP_Z + P.DIVIDER_H - 0.018), _q(m, rot))
    res = _step_seconds(embodiment, 1.5)
    rep = m.report()
    return {
        "score": float(res.info["score"]),
        "lid_deg": rep["lid_deg"],
        "lid_on_box_n": round(m.mk.contact_force("lid_low", "parts_box"), 2),
        "report": rep,
        "pass": float(res.info["score"]) == 0.0 and rep["lid_deg"] > 6.0,
    }


def trial_quarter_lid_shut(embodiment, m) -> dict:
    _reset_base(embodiment, m, lid_deg=0.0, tloc_deg=0.0, stow_box=True)
    res = _step_seconds(embodiment, 1.2)
    rep = m.report()
    return {
        "score": float(res.info["score"]),
        "report": rep,
        "pass": float(res.info["score"]) == 0.25 and rep["box_stowed"] and rep["lid_closed"],
    }


def trial_unslid_perched_on_cleats(embodiment, m) -> dict:
    P = m.P
    _reset_base(embodiment, m, lid_deg=0.0, tloc_deg=0.0, stow_box=True)
    # Drop case_top straight down at dx=0 above rear cleats (skipping the 12 mm horizontal slide)
    m.mk.write_pose("case_top", (P.CASE_LOW_XY[0], P.CASE_LOW_XY[1], m.TOP + 0.083), (0.0, 0.0, 0.0, 1.0))
    res = _step_seconds(embodiment, 1.5)
    rep = m.report()
    return {
        "score": float(res.info["score"]),
        "max_seam_mm": rep["max_seam_mm"],
        "cleats_engaged": rep["cleats_engaged"],
        "report": rep,
        "pass": float(res.info["score"]) == 0.25 and not rep["cleats_engaged"] and rep["max_seam_mm"] > 5.0,
    }


def trial_unslid_forward_blocks_tloc(embodiment, m) -> dict:
    P = m.P
    _reset_base(embodiment, m, lid_deg=0.0, tloc_deg=0.0, stow_box=True)
    # Leave case_top 12 mm forward on lid_low without sliding rearward; brace top (-8.5 N) and apply +0.25 N m to tloc_low
    m.mk.write_pose(
        "case_top",
        (P.CASE_LOW_XY[0] - P.CLEAT_SLIDE, P.CASE_LOW_XY[1], m.TOP + P.LID_DECK_Z + 0.003),
        (0.0, 0.0, 0.0, 1.0),
    )
    _step_seconds(embodiment, 0.5)
    m.mk.push("case_top", force=(0.0, 0.0, -8.5))
    m.mk.push("tloc_low", torque=(0.25, 0.0, 0.0))
    res = _step_seconds(embodiment, 1.0)
    m.mk.push("tloc_low")
    m.mk.push("case_top")
    rep = m.report()
    return {
        "score": float(res.info["score"]),
        "tloc_deg": rep["tloc_deg"],
        "tloc_on_top_n": round(m.mk.contact_force("case_top", "tloc_low"), 2),
        "max_seam_mm": rep["max_seam_mm"],
        "report": rep,
        "pass": float(res.info["score"]) == 0.25 and not rep["cleats_engaged"] and rep["max_seam_mm"] > 5.0 and m.mk.contact_force("case_top", "tloc_low") > 2.0,
    }


def trial_half_slid_unlatched_peels(embodiment, m) -> dict:
    P = m.P
    _reset_base(embodiment, m, lid_deg=0.0, tloc_deg=0.0, stow_box=True)
    m.mk.write_pose("case_top", (P.CASE_LOW_XY[0], P.CASE_LOW_XY[1], m.TOP + P.LID_DECK_Z + 0.0005), (0.0, 0.0, 0.0, 1.0))
    res_half = _step_seconds(embodiment, 1.4)
    score_before_lift = float(res_half.info["score"])
    rep_before = m.report()
    # Apply 15 N upward pull on case_top: with T-Loc at 0 deg CLOSE (not CONNECT), front peels wide open
    m.mk.push("case_top", force=(0.0, 0.0, P.LIFT_CHECK_N))
    _step_seconds(embodiment, 0.5)
    m.mk.push("case_top")
    rep_after = m.report()
    low_lifted_mm = round(1000.0 * (float(m.low_pose()[0][2]) - m.TOP), 1)
    return {
        "score_settled": score_before_lift,
        "before_lift": rep_before,
        "seam_after_15n_mm": rep_after["max_seam_mm"],
        "low_lifted_mm": low_lifted_mm,
        "pass": score_before_lift == 0.5 and rep_after["max_seam_mm"] > 12.0,
    }


def trial_full_coupled_holds_15n_lift(embodiment, m) -> dict:
    P = m.P
    _reset_base(embodiment, m, lid_deg=0.0, tloc_deg=P.TLOC_CONNECT_DEG, stow_box=True)
    m.mk.write_pose("case_top", (P.CASE_LOW_XY[0], P.CASE_LOW_XY[1], m.TOP + P.LID_DECK_Z + 0.0005), (0.0, 0.0, 0.0, 1.0))
    res_full = _step_seconds(embodiment, 1.6)
    score_settled = float(res_full.info["score"])
    rep_settled = m.report()
    # Apply 15 N upward lift on case_top (> 9.22 N total mass of both cases + box): lifts case_low off bench together!
    m.mk.push("case_top", force=(0.0, 0.0, P.LIFT_CHECK_N))
    _step_seconds(embodiment, 0.35)
    low_lift_mm = round(1000.0 * (float(m.low_pose()[0][2]) - m.TOP), 1)
    seam_under_15n_mm = m.report()["max_seam_mm"]
    lid_under_15n_deg = m.report()["lid_deg"]
    tloc_hook_n = round(m.mk.contact_force("case_top", "tloc_low"), 2)
    cleat_n = round(m.mk.contact_force("case_top", "lid_low"), 2)
    m.mk.push("case_top")
    return {
        "score_settled": score_settled,
        "settled_report": rep_settled,
        "low_lifted_off_bench_mm": low_lift_mm,
        "seam_under_15n_mm": seam_under_15n_mm,
        "lid_under_15n_deg": lid_under_15n_deg,
        "tloc_hook_force_n": tloc_hook_n,
        "cleat_force_n": cleat_n,
        "pass": score_settled == 1.0 and low_lift_mm > 15.0 and seam_under_15n_mm <= 3.5 and lid_under_15n_deg <= 2.5,
    }


def trial_zero_fall(embodiment, m) -> dict:
    _reset_base(embodiment, m, lid_deg=0.0, tloc_deg=0.0, stow_box=False)
    # Place parts_box at bench side edge and nudge it off
    m.mk.write_pose("parts_box", (0.215, 0.390, m.TOP + 0.002), (0.0, 0.0, 0.0, 1.0))
    m.mk.push("parts_box", force=(0.0, 3.0, 0.0))
    res = _step_seconds(embodiment, 0.9)
    m.mk.push("parts_box")
    rep = m.report()
    return {"score": float(res.info["score"]), "zero": rep["zero"], "pass": float(res.info["score"]) == 0.0 and rep["zero"] == "parts_box fell off the bench"}


TRIALS = {
    "cocked_box_blocks_lid": trial_cocked_box_blocks_lid,
    "quarter_lid_shut": trial_quarter_lid_shut,
    "unslid_perched_on_cleats": trial_unslid_perched_on_cleats,
    "unslid_forward_blocks_tloc": trial_unslid_forward_blocks_tloc,
    "half_slid_unlatched_peels": trial_half_slid_unlatched_peels,
    "full_coupled_holds_15n_lift": trial_full_coupled_holds_15n_lift,
    "zero_fall": trial_zero_fall,
}


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    names = sys.argv[2].split(",") if len(sys.argv) > 2 else list(TRIALS)
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    all_ok = True
    summary = {}
    for name in names:
        res = TRIALS[name](embodiment, m)
        (out / f"{name}.json").write_text(json.dumps(res, indent=2, default=str))
        summary[name] = {k: v for k, v in res.items() if k not in ("report", "before_lift", "settled_report")}
        print(f"TRIAL {name} " + json.dumps(summary[name], default=str), flush=True)
        all_ok = all_ok and bool(res["pass"])
    (out / "summary.json").write_text(json.dumps(summary, indent=2, default=str))
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
