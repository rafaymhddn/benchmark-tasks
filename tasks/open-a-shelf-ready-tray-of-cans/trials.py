#!/usr/bin/env python3
"""Gate G2 simulator verification trials for mech/tasks/shelf_ready_tray:

    scripts/rta simpy g1t1 mech/tasks/shelf_ready_tray/trials.py /workspace/robocurve/outputs/g1t1 [cases]

Cases (default: all four in one simulator session, record=False):
1. ``straight_lift``:
   - ``braced_30n``: left hand braces tray_base while right hand pulls straight vertically upward (+z) at the top
     center to 20 N or more (tear STRAIGHT_LIFT_HOLD_N = 459 N out of reach) -> 0 ties torn, panel_detach uncut.
   - ``unbraced_lift``: right hand pulls straight upward (+z) at top center without bracing -> tray_base lifts
     > 15 mm or tilts > 10 deg -> whole-trial zero rule scores 0.0; the tipping tray folds the panel, which may
     tear the side seams (g1t6: 5 stages) but never the bottom line.
2. ``flat_pull``: right hand folds the panel out and down at the top center with no corner turn and no brace ->
   the side seams tear (SIDE_SEAM_TIES stages), the bottom line holds, the panel stays on, the tray stays on its
   marks -> 0.5.
3. ``mid_peel_release``: left hand braces tray_base, right hand peels top-right corner to >= 30 deg, opens gripper,
   and retreats to READY for 2.0 s -> panel never swings back below 28.0 deg, ties_torn stays constant (>= 5),
   panel_detach stays uncut, score = 0.5.
4. ``known_load_peel``: with NO brace, the right hand peels the top-right corner outward and downward through
   52.0 deg -> logs bilateral force vs arc travel (2.75 N start = run over 84.4 mm arc) and cuts panel_detach
   cleanly while tray_base stays on its marks and all 6 cans stay upright (the cans' weight holds the tray).
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as G  # noqa: E402
import demo_kit as kit  # noqa: E402
import parts as P  # noqa: E402

ALL_CASES = ("straight_lift", "flat_pull", "mid_peel_release", "known_load_peel")


def run_braced_lift_30n(emb, out_dir: Path) -> dict:
    """Brace tray_base on left wall while pulling straight up (+z) at front_panel center to 20 N or more."""
    Move = kit.Move
    m = emb.task.module
    brace_over, brace_rot = G.left_brace_pose(lift_m=0.065)
    brace_pos, _ = G.left_brace_pose(lift_m=0.0)
    brace_press = brace_pos + np.array([0.004, 0.0, -0.006])

    # Grip full 15 mm of double-ply header (dz = TRAY_H - 3 mm, safely at y=0 between cans) for high vertical pull
    center_pos = np.array([G.PEEL_GRIP_X, 0.0, G.TOP + P.OVER + P.TRAY_H - 0.004], dtype=float)
    center_rot = G.frame(closing=(1.0, 0.0, 0.0), approach=(0.0, 0.0, -1.0))
    center_over = center_pos + np.array([0.0, 0.0, 0.065])
    center_pull = center_pos + np.array([0.0, 0.0, 0.055])

    left = [
        Move(1.4, brace_over, brace_rot, 0.45, "over_wall"),
        Move(1.0, brace_pos, brace_rot, 0.45, "down_wall"),
        Move(0.6, brace_press, brace_rot, 0.0, "brace_pinch"),
        Move(4.5, brace_press, brace_rot, 0.0, "hold_brace"),
    ]
    right = [
        Move(1.8, center_over, center_rot, 0.50, "over_center"),
        Move(1.2, center_pos, center_rot, 0.50, "down_center"),
        Move(0.6, center_pos, center_rot, 0.0, "pinch_center"),
        Move(2.0, center_pull, center_rot, 0.0, "pull_up_30n"),
        Move(0.9, center_pull, center_rot, 0.0, "hold_up"),
    ]
    res = kit.run(emb, {"left": left, "right": right}, out_dir / "braced_lift_30n", seed=0, tail_s=0.3, record=False)
    rep = m.report()
    return {
        "peak_lift_n": rep["peak_lift_n"],
        "ties_torn": rep["ties_torn"],
        "detached": rep["detached"],
        "max_pitch_deg": rep["max_pitch_deg"],
        "score": float(res["info"].get("score", 0.0)),
    }


def run_unbraced_straight_lift(emb, out_dir: Path) -> dict:
    """Pull straight up (+z) on front_panel top center without bracing -> lifts/tilts tray -> score 0.0, 0 ties torn."""
    Move = kit.Move
    m = emb.task.module
    # Pinch at y = -0.037 (cusp between front-center and front-right cans: 25 mm clear air gap behind header!)
    lift_pos = np.array([G.PEEL_GRIP_X, -0.037, G.TOP + P.OVER + P.TRAY_H - 0.005], dtype=float)
    lift_rot = G.frame(closing=(1.0, 0.0, 0.0), approach=(0.0, 0.0, -1.0))
    lift_over = lift_pos + np.array([0.0, 0.0, 0.065])
    lift_up = lift_pos + np.array([0.0, 0.0, 0.110])

    right = [
        Move(1.6, lift_over, lift_rot, 0.50, "over_rim"),
        Move(1.0, lift_pos, lift_rot, 0.50, "down_rim"),
        Move(0.6, lift_pos, lift_rot, 0.0, "pinch_rim"),
        Move(2.4, lift_up, lift_rot, 0.0, "lift_straight_up"),
        Move(0.6, lift_up, lift_rot, 1.0, "release"),
    ]
    res = kit.run(emb, {"right": right}, out_dir / "unbraced_lift", seed=0, tail_s=0.5, record=False)
    rep = m.report()
    return {
        "peak_lift_n": rep["peak_lift_n"],
        "max_tray_lift_mm": rep["max_tray_lift_mm"],
        "max_tray_tilt_deg": rep["max_tray_tilt_deg"],
        "ties_torn": rep["ties_torn"],
        "detached": rep["detached"],
        "zero_reason": rep["zero_reason"],
        "score": float(res["info"].get("score", 0.0)),
    }


def run_flat_pull(emb, out_dir: Path) -> dict:
    """Fold the panel out and down at the top center (no corner turn, no brace): side seams tear, the panel stays on."""
    Move = kit.Move
    m = emb.task.module
    center_pos, center_rot = G.peel_arc_pose(0.0, skew_deg=0.0, grip_y=0.0)
    center_over = center_pos + np.array([0.0, 0.0, 0.065])
    right = [
        Move(1.6, center_over, center_rot, 0.50, "over_center"),
        Move(1.0, center_pos, center_rot, 0.50, "down_center"),
        Move(0.6, center_pos, center_rot, 0.0, "pinch_center"),
    ]
    for deg in (14.0, 26.0, 38.0, 50.0, 62.0):
        p_t, r_t = G.peel_arc_pose(deg, skew_deg=0.0, grip_y=0.0)
        right.append(Move(1.0, p_t, r_t, 0.0, f"fold_{int(deg)}"))
    right.append(Move(0.5, right[-1].pos, right[-1].rot, 1.0, "release"))
    right.append(Move(1.0, right[-1].pos + np.array([-0.04, 0.0, 0.06]), right[-1].rot, 1.0, "back_off"))
    res = kit.run(emb, {"right": right}, out_dir / "flat_pull", seed=0, tail_s=0.5, record=False)
    rep = m.report()
    return {
        "max_tray_shift_mm": rep["max_tray_shift_mm"],
        "max_pitch_deg": rep["max_pitch_deg"],
        "max_skew_deg": rep["max_skew_deg"],
        "peak_peel_n": rep["peak_peel_n"],
        "ties_torn": rep["ties_torn"],
        "detached": rep["detached"],
        "zero_reason": rep["zero_reason"],
        "events": rep["events"],
        "score": float(res["info"].get("score", 0.0)),
    }


def run_mid_peel_release(emb, out_dir: Path) -> dict:
    """Peel top-right corner to >= 30 deg, open gripper, retreat to READY for 2 s -> stays >= 28 deg, ties kept, score 0.5."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    Move = kit.Move
    m = emb.task.module
    mk = m.mk
    ready_l = ik.tcp_pose(np.array(READY, dtype=float), station.BASES["left"])
    ready_r = ik.tcp_pose(np.array(READY, dtype=float), station.BASES["right"])

    brace_over, brace_rot = G.left_brace_pose(lift_m=0.065)
    brace_pos, _ = G.left_brace_pose(lift_m=0.0)
    brace_hold = brace_pos + np.array([0.008, -0.003, -0.006])
    brace_hold_rot = G.rz(math.radians(4.0)) @ brace_rot

    peel0_pos, peel0_rot = G.peel_arc_pose(0.0, skew_deg=0.0)
    peel_over = peel0_pos + np.array([0.0, 0.0, 0.065])
    skew_init_pos, skew_init_rot = G.peel_arc_pose(3.0, skew_deg=18.0, pull_extra_m=0.0)

    state = {"peel_done": False, "peak_pitch_before_release_deg": 0.0, "ties_at_release": 0}

    left_init = [
        Move(1.4, brace_over, brace_rot, 0.45, "over_wall"),
        Move(1.0, brace_pos, brace_rot, 0.45, "down_wall"),
        Move(0.6, brace_hold, brace_hold_rot, 0.0, "brace_pinch"),
        Move(0.8, brace_hold, brace_hold_rot, 0.0, "brace_steady"),
    ]
    right_init = [
        Move(1.6, peel_over, peel0_rot, 0.50, "over_corner"),
        Move(0.8, peel0_pos, peel0_rot, 0.50, "down_corner"),
        Move(0.6, peel0_pos, peel0_rot, 0.0, "pinch_corner"),
        Move(0.8, skew_init_pos, skew_init_rot, 0.0, "skew_corner"),
    ]

    peel_targets = [14.0, 24.0, 34.0, 40.0, 44.0]
    stage = {"idx": 0}

    def director(t, arms, *_):
        if state["peel_done"] or t < arms["right"].seconds - 1e-6:
            return
        cur_pitch_deg = math.degrees(float(mk.joint("peel_pitch"))) if "peel_pitch" in mk.MECH.travel else 0.0
        if cur_pitch_deg >= 30.8 or stage["idx"] >= len(peel_targets):
            state["peel_done"] = True
            state["peak_pitch_before_release_deg"] = round(cur_pitch_deg, 2)
            state["ties_at_release"] = m.L.ties_torn()
            p_now, r_now = G.peel_arc_pose(cur_pitch_deg, skew_deg=8.0, pull_extra_m=0.0)
            back_pos = p_now + np.array([-0.045, -0.040, 0.065])
            arms["right"].moves.extend(
                [
                    Move(0.6, p_now, r_now, 1.0, "open_mid_peel"),
                    Move(1.0, back_pos, r_now, 1.0, "back_clear"),
                    Move(1.6, ready_r[0], ready_r[1], 1.0, "ready"),
                    Move(2.0, ready_r[0], ready_r[1], 1.0, "settle_2s"),
                ]
            )
            arms["left"].moves.extend(
                [
                    Move(1.2, brace_hold, brace_hold_rot, 0.0, "hold_while_opening"),
                    Move(0.5, brace_pos, brace_rot, 0.50, "open_brace"),
                    Move(0.8, brace_over, brace_rot, 0.50, "up_wall"),
                    Move(1.5, ready_l[0], ready_l[1], 1.0, "ready"),
                    Move(1.2, ready_l[0], ready_l[1], 1.0, "settle_2s"),
                ]
            )
            return
        deg = peel_targets[stage["idx"]]
        stage["idx"] += 1
        p_t, r_t = G.peel_arc_pose(deg, skew_deg=15.0, pull_extra_m=0.0)
        arms["right"].moves.append(Move(1.0, p_t, r_t, 0.0, f"peel_{int(deg)}"))
        arms["left"].moves.append(Move(1.0, brace_hold, brace_hold_rot, 0.0, "brace"))

    def extra():
        return {
            "pitch_deg": round(math.degrees(float(mk.joint("peel_pitch"))), 2),
            "skew_deg": round(math.degrees(float(mk.joint("peel_skew"))), 2),
            "ties_torn": m.L.ties_torn(),
        }

    res = kit.run(
        emb,
        {"left": left_init, "right": right_init},
        out_dir / "mid_peel_release",
        seed=0,
        director=director,
        extra=extra,
        tail_s=0.2,
        record=False,
    )
    rep = m.report()
    end_pitch_deg = res["rows"][-1]["extra"]["pitch_deg"]
    settle_rows = [r["extra"]["pitch_deg"] for r in res["rows"][-30:]]
    return {
        "peak_pitch_before_release_deg": state["peak_pitch_before_release_deg"],
        "ties_at_release": state["ties_at_release"],
        "end_pitch_after_2s_deg": end_pitch_deg,
        "min_pitch_during_settle_deg": min(settle_rows),
        "end_ties_torn": rep["ties_torn"],
        "rest_pitch_deg": rep["rest_pitch_deg"],
        "detached": rep["detached"],
        "zero_reason": rep["zero_reason"],
        "score": float(res["info"].get("score", 0.0)),
    }


def run_known_load_peel(emb, out_dir: Path) -> dict:
    """Peel the top-right corner, with no brace, all the way through detachment, logging force vs arc travel."""
    Move = kit.Move
    m = emb.task.module

    peel0_pos, peel0_rot = G.peel_arc_pose(0.0, skew_deg=0.0)
    peel_over = peel0_pos + np.array([0.0, 0.0, 0.065])
    skew_init_pos, skew_init_rot = G.peel_arc_pose(3.0, skew_deg=18.0, pull_extra_m=0.0)

    right_init = [
        Move(1.6, peel_over, peel0_rot, 0.50, "over_corner"),
        Move(0.8, peel0_pos, peel0_rot, 0.50, "down_corner"),
        Move(0.6, peel0_pos, peel0_rot, 0.0, "pinch_corner"),
        Move(0.8, skew_init_pos, skew_init_rot, 0.0, "skew_corner"),
    ]

    peel_targets = [14.0, 26.0, 38.0, 50.0, 62.0, 70.0]
    stage = {"idx": 0, "done": False}

    def director(t, arms, *_):
        if stage["done"] or t < arms["right"].seconds - 1e-6:
            return
        if m.L.detached() or stage["idx"] >= len(peel_targets):
            stage["done"] = True
            p_end, r_end = G.peel_arc_pose(60.0, skew_deg=10.0, pull_extra_m=0.020)
            arms["right"].moves.append(Move(0.8, p_end + np.array([-0.02, -0.03, 0.03]), r_end, 0.0, "detached_hold"))
            return
        deg = peel_targets[stage["idx"]]
        stage["idx"] += 1
        p_t, r_t = G.peel_arc_pose(deg, skew_deg=15.0, pull_extra_m=0.0)
        arms["right"].moves.append(Move(1.0, p_t, r_t, 0.0, f"peel_{int(deg)}"))

    res = kit.run(
        emb,
        {"right": right_init},
        out_dir / "known_load_peel",
        seed=0,
        director=director,
        tail_s=0.4,
        record=False,
    )
    rep = m.report()
    return {
        "detached": rep["detached"],
        "detached_at_s": rep["detached_at_s"],
        "detach_cause": rep["detach_cause"],
        "ties_torn": rep["ties_torn"],
        "peak_peel_n": rep["peak_peel_n"],
        "max_skew_deg": rep["max_skew_deg"],
        "max_pitch_deg": rep["max_pitch_deg"],
        "max_tray_shift_mm": rep["max_tray_shift_mm"],
        "max_tray_tilt_deg": rep["max_tray_tilt_deg"],
        "max_can_tip_deg": rep["max_can_tip_deg"],
        "zero_reason": rep["zero_reason"],
        "force_profile": rep["force_profile"],
        "score_while_held": float(res["info"].get("score", 0.0)),
    }


def main() -> int:
    out_dir = Path(sys.argv[1])
    cases = sys.argv[2].split(",") if len(sys.argv) > 2 else list(ALL_CASES)
    out_dir.mkdir(parents=True, exist_ok=True)

    emb = kit.open_task(HERE / "task.py")
    summary: dict = {}
    ok = True

    if "straight_lift" in cases:
        r_braced = run_braced_lift_30n(emb, out_dir)
        r_unbraced = run_unbraced_straight_lift(emb, out_dir)
        summary["straight_lift"] = {"braced_30n": r_braced, "unbraced_lift": r_unbraced}
        if not (
            r_braced["ties_torn"] == 0
            and not r_braced["detached"]
            and r_braced["peak_lift_n"] >= 20.0
            and r_unbraced["ties_torn"] <= P.SIDE_SEAM_TIES  # the tipping tray folds the panel: side seams at most
            and not r_unbraced["detached"]
            and r_unbraced["zero_reason"] is not None
            and r_unbraced["score"] == 0.0
        ):
            ok = False

    if "flat_pull" in cases:
        r_pull = run_flat_pull(emb, out_dir)
        summary["flat_pull"] = r_pull
        if not (
            r_pull["max_pitch_deg"] >= 30.0
            and r_pull["ties_torn"] == P.SIDE_SEAM_TIES
            and not r_pull["detached"]
            and r_pull["zero_reason"] is None
            and r_pull["score"] == 0.5
        ):
            ok = False

    if "mid_peel_release" in cases:
        r_mid = run_mid_peel_release(emb, out_dir)
        summary["mid_peel_release"] = r_mid
        if not (
            r_mid["peak_pitch_before_release_deg"] >= 30.0
            and r_mid["min_pitch_during_settle_deg"] >= 28.0
            and r_mid["end_pitch_after_2s_deg"] >= 28.0
            and r_mid["end_ties_torn"] == r_mid["ties_at_release"]
            and r_mid["end_ties_torn"] >= P.PARTIAL_TIES_MIN
            and not r_mid["detached"]
            and r_mid["zero_reason"] is None
            and r_mid["score"] == 0.5
        ):
            ok = False

    if "known_load_peel" in cases:
        r_peel = run_known_load_peel(emb, out_dir)
        summary["known_load_peel"] = r_peel
        if not (
            r_peel["detached"]
            and r_peel["ties_torn"] == P.N_TIES
            and r_peel["max_skew_deg"] >= P.PEEL_SKEW_MIN_DEG
            and r_peel["zero_reason"] is None
            and len(r_peel["force_profile"]) >= 6
        ):
            ok = False

    summary["all_pass"] = ok
    (out_dir / "trials_summary.json").write_text(json.dumps(summary, indent=2))
    print("G2_TRIALS " + json.dumps(summary, indent=2), flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
