#!/usr/bin/env python3
"""Gate-2 simulator trials for ``mech/tasks/rolling_bottle``:

1. ``roll_trace`` (idle arms at READY): records exact ``v_foot``, ``t_150``, ``t_300``, ``t_edge``, lateral drift,
   support contact force, contact-gated rolling torque (and confirms 0.000000 N m once airborne over the cliff),
   confirms the cork coaster at (0.205, -0.020) is never touched, and verifies permanent score ``0.0``.
2. ``stop_only``: right hand moves to the interception gate, cages and gently damps the rolling bottle on the worktop,
   opens and returns to READY so the stopped lying bottle holds at rest with both grippers clear -> score ``0.5``.

Usage:
    scripts/rta simpy g2b1 mech/tasks/rolling_bottle/trials.py /workspace/robocurve/outputs/g2b1 [roll_trace,stop_only]
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as Ck  # noqa: E402
import demo_kit as kit  # noqa: E402
import parts as P  # noqa: E402
from yam_arena import constants as YC, ik, station  # noqa: E402
from yam_arena.constants import READY  # noqa: E402

CAGE_TRIGGER_Y = -0.062  # m: bottle centre y at which the hovering open jaw drops to cage the rolling cylinder
MAX_JAW_M = Ck.MAX_JAW_M
GRIP_OPEN = Ck.GRIP_OPEN
GRIP_DAMP = round(min(1.0, (P.BOTTLE_D + 0.0015) / MAX_JAW_M), 3)  # gentle 55.5 mm cushion across 54 mm bottle


def run_roll_trace(emb, out_dir: Path) -> dict:
    m = emb.task.module
    mk = m.mk

    def step_extra() -> dict:
        p_b, _ = mk.pose("bottle")
        lin_v, ang_v = mk.velocity("bottle")
        return {
            "bottle_xyz_m": [round(float(v), 4) for v in p_b],
            "vy_mps": round(float(lin_v[1]), 4),
            "speed_mps": round(float(np.linalg.norm(lin_v)), 4),
            "wx_radps": round(float(ang_v[0]), 4),
            "f_drain_n": round(float(mk.contact_force("bottle", "drainboard")), 3),
            "f_worktop_n": round(float(mk.contact_force("bottle", "worktop")), 3),
            "f_coaster_n": round(float(mk.contact_force("bottle", "coaster")), 3),
            "tau_roll_nm": [round(float(v), 6) for v in m.STATE["last_tau_roll"]],
        }

    res = kit.run(
        emb,
        {},
        out_dir / "roll_trace",
        seed=0,
        tail_s=5.0,
        limit_s=6.0,
        extra=step_extra,
        record=False,
    )
    rep = res["report"]
    crossings = rep["crossings"]
    dt_foot = rep["dt_from_foot_s"]
    analytic = P.analytic_roll_predictions()

    # Check lateral x drift while rolling on drainboard + worktop (before crossing edge y = -0.230)
    on_deck_xs = [
        r["extra"]["bottle_xyz_m"][0]
        for r in res["rows"]
        if r["extra"]["bottle_xyz_m"][1] >= P.WORKTOP_MIN_Y
    ]
    max_lat_drift_mm = 1000.0 * max(abs(x - P.ROLL_LANE_X) for x in on_deck_xs) if on_deck_xs else 0.0

    summary = {
        "case": "roll_trace",
        "score": res["info"].get("score"),
        "terminated": res["terminated"],
        "v_foot_mps": crossings.get("foot", {}).get("speed_mps"),
        "v_150mm_mps": crossings.get("150mm", {}).get("speed_mps"),
        "v_300mm_mps": crossings.get("300mm", {}).get("speed_mps"),
        "t_foot_from_release_s": crossings.get("foot", {}).get("dt_from_release_s"),
        "t_150_from_foot_s": dt_foot.get("150mm"),
        "t_300_from_foot_s": dt_foot.get("300mm"),
        "t_edge_from_foot_s": dt_foot.get("edge"),
        "t_edge_from_release_s": crossings.get("edge", {}).get("dt_from_release_s"),
        "analytic_reference": analytic,
        "max_lateral_x_drift_mm": round(max_lat_drift_mm, 3),
        "min_bottle_coaster_gap_mm": round(1000.0 * m.STATE["min_bottle_coaster_gap_m"], 2),
        "max_coaster_force_n": rep["max_coaster_force_n"],
        "max_coaster_shift_mm": rep["max_coaster_shift_mm"],
        "max_tau_roll_nm": rep["max_tau_roll_nm"],
        "max_airborne_tau_nm": rep["max_airborne_tau_nm"],
        "off_edge": rep["off_edge"],
    }
    assert summary["score"] == 0.0, f"expected idle roll_trace score 0.0, got {summary['score']}"
    assert summary["off_edge"] is not None, "expected idle bottle to go over y = -0.230 cliff"
    assert summary["max_coaster_force_n"] == 0.0, f"idle bottle touched coaster: {summary}"
    assert summary["max_coaster_shift_mm"] < 0.5, f"coaster shifted on idle run: {summary}"
    assert summary["max_airborne_tau_nm"] == 0.0, f"Check 51 violation: airborne tau {summary['max_airborne_tau_nm']}"
    return summary


def run_stop_only(emb, out_dir: Path) -> dict:
    Move = kit.Move
    m = emb.task.module
    mk = m.mk

    z_hover = P.WORKTOP_TOP_Z + P.BOTTLE_D + Ck.HOVER_CLEAR_OVER_BOTTLE_M + float(YC.PAD_TIP_PAST_TCP)
    z_cage = P.WORKTOP_TOP_Z + P.BOTTLE_R
    rot_down = Ck.R_PINCH_DOWN
    ready_tcp, ready_rot = ik.tcp_pose(np.array(READY, dtype=float), station.BASES["right"])

    initial_moves = [
        Move(0.85, (P.ROLL_LANE_X, Ck.INTERCEPT_Y, z_hover), rot_down, GRIP_OPEN, "hover_gate"),
    ]
    state = {"caged": False}

    def director(t: float, arms: dict, _pictures) -> None:
        right = arms["right"]
        if not state["caged"]:
            if mk.MECH.poses and t >= 0.85:
                p_b, _ = mk.pose("bottle")
                if float(p_b[1]) <= CAGE_TRIGGER_Y:
                    right.moves.extend(
                        [
                            Move(0.15, (P.ROLL_LANE_X, Ck.INTERCEPT_Y, z_cage), rot_down, GRIP_OPEN, "cage_drop"),
                            Move(0.45, (P.ROLL_LANE_X, Ck.INTERCEPT_Y, z_cage), rot_down, GRIP_DAMP, "damp_spin"),
                            Move(0.30, (P.ROLL_LANE_X, Ck.INTERCEPT_Y, z_cage), rot_down, GRIP_OPEN, "release_lying"),
                            Move(0.65, (P.ROLL_LANE_X, Ck.INTERCEPT_Y, z_hover + 0.035), rot_down, GRIP_OPEN, "lift_clear"),
                            Move(1.20, ready_tcp, ready_rot, GRIP_OPEN, "return_ready"),
                            Move(1.60, ready_tcp, ready_rot, GRIP_OPEN, "hold_ready"),
                        ]
                    )
                    state["caged"] = True
                    return
            if t < 5.0 and right.seconds <= t + 0.05 + 1e-6:
                right.moves.append(Move(0.05, (P.ROLL_LANE_X, Ck.INTERCEPT_Y, z_hover), rot_down, GRIP_OPEN, "watch_roll"))
            elif t >= 5.0:
                state["caged"] = True

    res = kit.run(
        emb,
        {"right": initial_moves},
        out_dir / "stop_only",
        seed=0,
        director=director,
        tail_s=0.5,
        limit_s=10.0,
        record=False,
    )
    rep = res["report"]
    summary = {
        "case": "stop_only",
        "score": res["info"].get("score"),
        "steps": res["steps"],
        "seconds": res["seconds"],
        "bottle_pos_m": rep.get("bottle_pos_m"),
        "off_edge": rep.get("off_edge"),
        "grippers_clear": rep.get("grippers_clear"),
        "safe_steps": rep.get("safe_steps"),
        "worst_ik": res["worst_ik"],
    }
    assert summary["score"] == 0.5, f"expected stop_only score 0.5, got {summary}"
    assert summary["off_edge"] is None, f"stop_only went off edge: {summary}"
    return summary


def main() -> int:
    out_dir = Path(sys.argv[1])
    wanted = sys.argv[2].split(",") if len(sys.argv) > 2 else ["roll_trace", "stop_only"]
    out_dir.mkdir(parents=True, exist_ok=True)
    emb = kit.open_task(HERE / "task.py")

    results = {}
    for name in wanted:
        if name == "roll_trace":
            results[name] = run_roll_trace(emb, out_dir)
        elif name == "stop_only":
            results[name] = run_stop_only(emb, out_dir)
        else:
            raise SystemExit(f"unknown trial {name!r}; expected roll_trace or stop_only")
        print(f"TRIAL {name}: " + json.dumps(results[name]), flush=True)

    (out_dir / "trials_summary.json").write_text(json.dumps(results, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
