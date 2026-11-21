#!/usr/bin/env python3
"""Scripted bimanual reference run for ``lab_jack_probe`` (Gate G3).

    scripts/rta demo <id> mech/tasks/lab_jack_probe [--check|--no-video] [--quick]

Choreography (arm joint commands only; ``--quick`` checks the nominal trajectory offline):
1. Left hand braces ``jack_base`` vertically on its +y top plate while Right hand turns the -y adjusting wheel CCW in
   front-quadrant strokes until ``jack_deck`` drops below 63 mm so ``beaker_sample``'s rim clears the 130 mm pH probe.
2. Left hand extracts ``beaker_sample`` horizontally off ``jack_deck`` and places it upright on ``blue_mat`` (0.25).
3. Right hand lifts ``beaker_rinse`` from its start pad and sets it coaxial under ``probe`` on ``jack_deck`` (0.5).
4. Left hand steadies ``jack_base`` again while Right hand turns the wheel CW until ``jack_deck`` reaches ~98 mm
   (~28 mm probe immersion inside the 20-40 mm green band), then both arms return to READY and hold clear (1.0).
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as K  # noqa: E402
import parts as P  # noqa: E402

OPEN = 1.0
WHEEL_OPEN = K.WHEEL_OPEN_M / K.MAX_JAW
WHEEL_PINCH = K.WHEEL_PINCH_M / K.MAX_JAW
BEAKER_OPEN = K.BEAKER_OPEN_M / K.MAX_JAW
BEAKER_PINCH = K.BEAKER_PINCH_M / K.MAX_JAW
BRACE_PUSH = (0.0, 0.0, -3.5)
ZERO_PUSH = (0.0, 0.0, 0.0)

STROKE_LO_DEG = K.WHEEL_ARC_LO_DEG  # -71.0 deg
STROKE_HI_DEG = K.WHEEL_ARC_HI_DEG  # -7.0 deg


def ready(side: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, dtype=float), K.BASES[side])


def one_wheel_stroke(
    *, direction: str, idx: int, z_deck: float, wheel_centre: np.ndarray | None = None, first: bool = False
) -> list:
    """One pinch-turn-release-reindex stroke around ``wheel_centre`` (or nominal ``z_deck`` for offline dry-run)."""
    import demo_kit as kit

    Move = kit.Move
    dz = -0.0029 if direction == "lower" else 0.0027
    a0, a2 = (STROKE_HI_DEG, STROKE_LO_DEG) if direction == "lower" else (STROKE_LO_DEG, STROKE_HI_DEG)
    angs = np.linspace(a0, a2, 3)
    moves: list = []

    def wp(sub_frac: float, ang: float, *, clear: bool):
        wc = None if wheel_centre is None else (np.asarray(wheel_centre, dtype=float) + np.array([0.0, 0.0, 0.5 * dz * sub_frac]))
        return K.wheel_pose(max(P.JACK_Z_MIN, z_deck + dz * sub_frac), ang, clear=clear, wheel_centre=wc)

    if first:
        p0, r0 = wp(0.0, float(angs[0]), clear=True)
        moves.append(Move(1.2, pos=p0, rot=r0, grip=WHEEL_OPEN, label=f"{direction}: approach wheel"))
    else:
        pm, rm = wp(0.0, float(angs[1]), clear=True)
        p0, r0 = wp(0.0, float(angs[0]), clear=True)
        moves.append(Move(0.38, pos=pm, rot=rm, grip=WHEEL_OPEN, label=f"{direction} {idx}: reindex"))
        moves.append(Move(0.38, pos=p0, rot=r0, grip=WHEEL_OPEN, label=f"{direction} {idx}: reindex"))

    pi, ri = wp(0.0, float(angs[0]), clear=False)
    moves.append(Move(0.38, pos=pi, rot=ri, grip=WHEEL_OPEN, label=f"{direction} {idx}: engage"))
    moves.append(Move(0.32, grip=WHEEL_PINCH, label=f"{direction} {idx}: pinch"))
    for sub in (1, 2):
        ps, rs = wp(sub / 2.0, float(angs[sub]), clear=False)
        moves.append(Move(0.42, pos=ps, rot=rs, label=f"{direction} {idx}: turn"))
    moves.append(Move(0.42, grip=WHEEL_OPEN, label=f"{direction} {idx}: open"))
    po, ro = wp(1.0, float(angs[-1]), clear=True)
    moves.append(Move(0.38, pos=po, rot=ro, label=f"{direction} {idx}: clear"))
    return moves


def swap_moves(z_low: float, sample_xy: tuple[float, float]) -> tuple[list, list]:
    """Build synchronized Phase 2 beaker-swap moves for Left (sample -> blue_mat) and Right (rinse -> jack_deck)."""
    import demo_kit as kit

    Move = kit.Move
    r_left, r_right = ready("left"), ready("right")
    bh_pos, bh_rot = K.brace_pose(hover=True)
    bi_pos, bi_rot = K.brace_pose(pre_in=True)

    s_out, s_rot = K.beaker_pose("left", sample_xy, z_low, back=K.BEAKER_BACK_M)
    s_on, _ = K.beaker_pose("left", sample_xy, z_low)
    s_lift, _ = K.beaker_pose("left", sample_xy, z_low + K.BEAKER_LIFT_CLEAR)
    s_ext, _ = K.beaker_pose("left", sample_xy, z_low + K.BEAKER_LIFT_CLEAR, back=K.BEAKER_BACK_M)
    s_mat_hi, _ = K.beaker_pose("left", P.MAT_XY, P.MAT_SIZE[2] + 0.035)
    s_mat_lo, _ = K.beaker_pose("left", P.MAT_XY, P.MAT_SIZE[2] + 0.001)
    s_mat_back, _ = K.beaker_pose("left", P.MAT_XY, P.MAT_SIZE[2], back=0.065, along_approach=True)

    left = [
        Move(0.35, push=ZERO_PUSH, label="release brace"),
        Move(0.55, pos=bi_pos, rot=bi_rot, label="lift vertical"),
        Move(0.65, pos=bh_pos, rot=bh_rot, label="brace hover"),
        Move(1.1, pos=r_left[0], rot=r_left[1], grip=BEAKER_OPEN, label="through ready"),
        Move(1.3, pos=s_out, rot=s_rot, grip=BEAKER_OPEN, label="sample: outside deck"),
        Move(1.0, pos=s_on, label="sample: around"),
        Move(0.55, grip=BEAKER_PINCH, label="sample: pinch"),
        Move(0.55, pos=s_lift, label="sample: lift 3.5 mm"),
        Move(1.1, pos=s_ext, label="sample: slide clear"),
    ]
    t_clear = sum(mv.seconds for mv in left)
    left.extend([
        Move(1.3, pos=s_mat_hi, label="sample: over blue mat"),
        Move(0.95, pos=s_mat_lo, label="sample: set on mat"),
        Move(0.55, grip=BEAKER_OPEN, label="sample: release"),
        Move(0.95, pos=s_mat_back, label="sample: back off"),
        Move(1.2, pos=r_left[0], rot=r_left[1], grip=0.0, label="through ready"),
        Move(1.1, pos=bh_pos, rot=bh_rot, grip=0.0, label="brace hover"),
    ])

    c_out, c_rot = K.beaker_pose("right", P.RINSE_START_XY, P.COASTER_SIZE[2], back=0.065, along_approach=True)
    c_on, _ = K.beaker_pose("right", P.RINSE_START_XY, P.COASTER_SIZE[2])
    c_lift, _ = K.beaker_pose("right", P.RINSE_START_XY, z_low + K.BEAKER_LIFT_CLEAR)
    # Aim 1.5 mm past probe axis along RIGHT_IN_DIR to compensate the ~2 mm arm gravity lag measured in g3c1
    target_xy = (P.JACK_XY[0] + 0.0015 * float(K.RIGHT_IN_DIR[0]), P.JACK_XY[1] + 0.0015 * float(K.RIGHT_IN_DIR[1]))
    d_out, _ = K.beaker_pose("right", target_xy, z_low + K.BEAKER_LIFT_CLEAR, back=K.BEAKER_BACK_M)
    d_over, _ = K.beaker_pose("right", target_xy, z_low + K.BEAKER_LIFT_CLEAR)
    d_set, _ = K.beaker_pose("right", target_xy, z_low + 0.001)
    d_back, _ = K.beaker_pose("right", target_xy, z_low, back=K.BEAKER_BACK_M)

    right = [
        Move(1.3, pos=r_right[0], rot=r_right[1], grip=BEAKER_OPEN, label="through ready"),
        Move(1.3, pos=c_out, rot=c_rot, grip=BEAKER_OPEN, label="rinse: outside pad"),
        Move(0.95, pos=c_on, label="rinse: around"),
        Move(0.55, grip=BEAKER_PINCH, label="rinse: pinch"),
        Move(0.85, pos=c_lift, label="rinse: lift"),
    ]
    right.append(Move(max(0.2, t_clear + 0.5 - sum(mv.seconds for mv in right)), label="rinse: wait deck clear"))
    right.extend([
        Move(1.2, pos=d_out, label="rinse: outside deck"),
        Move(1.1, pos=d_over, label="rinse: coaxial under probe"),
        Move(0.85, pos=d_set, label="rinse: set on deck"),
        Move(0.55, grip=BEAKER_OPEN, label="rinse: release"),
        Move(1.0, pos=d_back, label="rinse: slide out"),
        Move(1.3, pos=r_right[0], rot=r_right[1], grip=WHEEL_OPEN, label="through ready"),
    ])

    dur = max(sum(mv.seconds for mv in left), sum(mv.seconds for mv in right))
    if sum(mv.seconds for mv in left) < dur:
        left.append(Move(dur - sum(mv.seconds for mv in left), label="sync swap"))
    if sum(mv.seconds for mv in right) < dur:
        right.append(Move(dur - sum(mv.seconds for mv in right), label="sync swap"))
    return left, right


def nominal_plan() -> dict[str, list]:
    """Complete static nominal plan for ``--quick`` 20 Hz IK verification."""
    import demo_kit as kit

    Move = kit.Move
    r_left, r_right = ready("left"), ready("right")
    bh_pos, bh_rot = K.brace_pose(hover=True)
    bi_pos, bi_rot = K.brace_pose(pre_in=True)
    bp_pos, bp_rot = K.brace_pose()

    left: list = [
        Move(1.1, pos=bh_pos, rot=bh_rot, grip=0.0, label="brace hover"),
        Move(0.6, pos=bi_pos, rot=bi_rot, grip=0.0, label="brace pre_in"),
        Move(0.5, pos=bp_pos, rot=bp_rot, grip=0.0, label="touch base"),
        Move(0.3, push=BRACE_PUSH, label="brace jack"),
    ]
    right: list = [Move(1.1, pos=r_right[0], rot=r_right[1], grip=WHEEL_OPEN, label="ready")]
    z = P.Z_DECK_START
    for k in range(9):
        right.extend(one_wheel_stroke(direction="lower", idx=k + 1, z_deck=z, first=(k == 0)))
        z = max(P.JACK_Z_MIN, z - 0.0029)
    left.append(Move(max(0.2, sum(mv.seconds for mv in right) - sum(mv.seconds for mv in left)), push=BRACE_PUSH, label="steady lower"))

    l_swap, r_swap = swap_moves(z, P.JACK_XY)
    left.extend(l_swap)
    right.extend(r_swap)

    left.extend([
        Move(0.6, pos=bi_pos, rot=bi_rot, grip=0.0, label="brace pre_in"),
        Move(0.5, pos=bp_pos, rot=bp_rot, grip=0.0, label="touch base"),
        Move(0.3, push=BRACE_PUSH, label="brace jack"),
    ])
    for k in range(13):
        right.extend(one_wheel_stroke(direction="raise", idx=k + 1, z_deck=z, first=(k == 0)))
        z += 0.0027
    left.append(Move(max(0.2, sum(mv.seconds for mv in right) - sum(mv.seconds for mv in left)), push=BRACE_PUSH, label="steady raise"))
    left.extend([
        Move(0.35, push=ZERO_PUSH, label="release brace"),
        Move(0.55, pos=bi_pos, rot=bi_rot, label="lift vertical"),
        Move(0.65, pos=bh_pos, rot=bh_rot, label="brace hover"),
        Move(1.6, pos=r_left[0], rot=r_left[1], grip=OPEN, label="ready"),
    ])
    right.append(Move(1.6, pos=r_right[0], rot=r_right[1], grip=OPEN, label="ready"))
    return {"left": left, "right": right}


def save_end_picture(frame, folder: Path) -> None:
    import cv2
    import demo_kit as kit

    cv2.imwrite(str(folder / "end_top.png"), cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(folder / "end_top_policy.png"), cv2.cvtColor(kit.policy_picture(np.asarray(frame)), cv2.COLOR_RGB2BGR))


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    if "--quick" in sys.argv[2:]:
        from yam_arena import station

        dry = kit.dry_run(nominal_plan(), station.BASES, limit_s=120.0)
        print("QUICK " + json.dumps(dry), flush=True)
        ok = all(dry[s]["worst_ik_mm"] <= 0.5 and dry[s]["smallest_margin_deg"] >= 5.0 for s in ("left", "right"))
        return 0 if ok else 1

    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk
    Move = kit.Move
    r_left, r_right = ready("left"), ready("right")
    bh_pos, bh_rot = K.brace_pose(hover=True)
    bi_pos, bi_rot = K.brace_pose(pre_in=True)
    bp_pos, bp_rot = K.brace_pose()

    init_moves = {
        "left": [
            Move(1.1, pos=bh_pos, rot=bh_rot, grip=0.0, label="brace hover"),
            Move(0.6, pos=bi_pos, rot=bi_rot, grip=0.0, label="brace pre_in"),
            Move(0.5, pos=bp_pos, rot=bp_rot, grip=0.0, label="touch base"),
            Move(0.3, push=BRACE_PUSH, label="brace jack"),
        ],
        "right": [Move(2.5, pos=r_right[0], rot=r_right[1], grip=WHEEL_OPEN, label="ready")],
    }
    phase_state = {"phase": "lower", "lower_k": 0, "raise_k": 0}
    latest: dict = {}

    def director(t: float, arms: dict, pictures) -> None:
        if pictures is not None:
            latest.update(pictures)
        if not mk.MECH.poses:
            return
        rem_r = arms["right"].seconds - t
        rem_l = arms["left"].seconds - t
        ph = phase_state["phase"]

        if ph == "lower" and rem_r <= 0.06:
            zd = m.deck_height_m()
            if zd > 0.0625 and phase_state["lower_k"] < 12:
                k = phase_state["lower_k"] + 1
                phase_state["lower_k"] = k
                wc = mk.pose("knob")[0].copy()
                st = one_wheel_stroke(direction="lower", idx=k, z_deck=zd, wheel_centre=wc, first=(k == 1))
                arms["right"].moves.extend(st)
                need = arms["right"].seconds - arms["left"].seconds
                if need > 0.01:
                    arms["left"].moves.append(Move(need, push=BRACE_PUSH, label="steady lower"))
            else:
                phase_state["phase"] = "swap"
                s_xy = (float(mk.pose("beaker_sample")[0][0]), float(mk.pose("beaker_sample")[0][1]))
                if rem_l > rem_r + 0.01:
                    arms["right"].moves.append(Move(rem_l - rem_r, label="sync pre-swap"))
                elif rem_r > rem_l + 0.01:
                    arms["left"].moves.append(Move(rem_r - rem_l, push=BRACE_PUSH, label="sync pre-swap"))
                l_sw, r_sw = swap_moves(zd, s_xy)
                arms["left"].moves.extend(l_sw)
                arms["right"].moves.extend(r_sw)
                arms["left"].moves.extend([
                    Move(0.6, pos=bi_pos, rot=bi_rot, grip=0.0, label="brace pre_in"),
                    Move(0.5, pos=bp_pos, rot=bp_rot, grip=0.0, label="touch base"),
                    Move(0.3, push=BRACE_PUSH, label="brace jack"),
                ])
                arms["right"].moves.append(Move(1.4, label="wait brace"))
                phase_state["phase"] = "raise"

        elif ph == "raise" and rem_r <= 0.06:
            imm = m.immersion_depth_m()
            zd = m.deck_height_m()
            if imm < 0.0230 and phase_state["raise_k"] < 15:
                k = phase_state["raise_k"] + 1
                phase_state["raise_k"] = k
                wc = mk.pose("knob")[0].copy()
                st = one_wheel_stroke(direction="raise", idx=k, z_deck=zd, wheel_centre=wc, first=(k == 1))
                arms["right"].moves.extend(st)
                need = arms["right"].seconds - arms["left"].seconds
                if need > 0.01:
                    arms["left"].moves.append(Move(need, push=BRACE_PUSH, label="steady raise"))
            else:
                phase_state["phase"] = "retreat"
                if rem_r > rem_l + 0.01:
                    arms["left"].moves.append(Move(rem_r - rem_l, push=BRACE_PUSH, label="sync retreat"))
                arms["left"].moves.extend([
                    Move(0.35, push=ZERO_PUSH, label="release brace"),
                    Move(0.55, pos=bi_pos, rot=bi_rot, label="lift vertical"),
                    Move(0.65, pos=bh_pos, rot=bh_rot, label="brace hover"),
                    Move(1.6, pos=r_left[0], rot=r_left[1], grip=OPEN, label="ready"),
                ])
                arms["right"].moves.append(Move(3.15, pos=r_right[0], rot=r_right[1], grip=OPEN, label="ready"))

    def finger_forces_n(body_name: str) -> float:
        return round(sum(abs(float(c.force)) for c in mk.contacts(body_name) if c.other.endswith("robot")), 2)

    def extra() -> dict:
        coup = m.STATE.get("last_coupling", {})
        return {
            "deck_mm": round(1000.0 * m.deck_height_m(), 2),
            "wheel_turns": round(mk.joint("wheel") / (2.0 * math.pi), 3),
            "immersion_mm": round(1000.0 * m.immersion_depth_m(), 2),
            "probe_lat_mm": round(1000.0 * m.probe_lateral_m(), 2),
            "probe_ax_mm": round(1000.0 * m.probe_axial_lift_m(), 2),
            "tau_wheel_nm": round(float(coup.get("tau_wheel_nm", 0.0)), 4),
            "tau_load_nm": round(float(coup.get("tau_load_nm", 0.0)), 4),
            "knob_pinch_n": finger_forces_n("knob"),
            "sample_pinch_n": finger_forces_n("beaker_sample"),
            "rinse_pinch_n": finger_forces_n("beaker_rinse"),
            "hands_clear_mm": round(1000.0 * m.hands_clear_m(), 1),
        }

    folder = out / "full"
    record = not any(a in ("--no-video", "--check") for a in sys.argv[2:])
    result = kit.run(
        embodiment, init_moves, folder, seed=0, director=director, extra=extra,
        limit_s=float(m.RUN_LIMIT_STEPS) / 20.0, tail_s=4.5, sheet_frames=16, record=record,
    )
    rows = result["rows"]
    if "top_cam" in latest:
        save_end_picture(latest["top_cam"], folder)

    readings = {
        "end_score": rows[-1]["info"].get("score") if rows else 0.0,
        "steps": len(rows),
        "seconds": rows[-1]["t"] if rows else 0.0,
        "lower_strokes": phase_state["lower_k"],
        "raise_strokes": phase_state["raise_k"],
        "min_deck_mm": min(r["extra"]["deck_mm"] for r in rows),
        "max_deck_mm": max(r["extra"]["deck_mm"] for r in rows),
        "end_deck_mm": rows[-1]["extra"]["deck_mm"],
        "min_wheel_turns": min(r["extra"]["wheel_turns"] for r in rows),
        "max_wheel_turns": max(r["extra"]["wheel_turns"] for r in rows),
        "end_immersion_mm": rows[-1]["extra"]["immersion_mm"],
        "peak_knob_pinch_total_n": max(r["extra"]["knob_pinch_n"] for r in rows),
        "peak_sample_pinch_total_n": max(r["extra"]["sample_pinch_n"] for r in rows),
        "peak_rinse_pinch_total_n": max(r["extra"]["rinse_pinch_n"] for r in rows),
        "peak_wheel_torque_nm": max(abs(r["extra"]["tau_wheel_nm"]) for r in rows),
        "report": result.get("report"),
        "worst_ik": result.get("worst_ik"),
        "median_step_ms": result.get("median_step_ms"),
    }
    (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
    print("READINGS " + json.dumps(readings, default=str), flush=True)
    return 0 if readings["end_score"] == 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
