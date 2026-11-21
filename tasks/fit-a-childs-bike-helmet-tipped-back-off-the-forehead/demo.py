#!/usr/bin/env python3
"""Scripted bimanual demonstration for ``mech/tasks/helmet_fit``:

    scripts/rta demo <id> mech/tasks/helmet_fit --check    # the run without a video
    python3 mech/tasks/helmet_fit/demo.py --dry            # IK, joint margins and face clearance, no simulator

1. The left hand, fingers closed, comes over the forehead, presses the front rim down from 13 deg to 2.6 deg (front
   edge 25 mm over the brow band) and holds it there against the helmet's rocking back and the dial's twist.
2. With the helmet at the fit the dial is up at 86 mm. The right hand comes in along the dial's axis, pinches its rim and
   turns it in three regrasps, 75 deg twice and a last 60 deg (210 deg, 10.5 clicks): the band meets the back of the
   head after about 7 clicks and presses it with CLAMP_N after about 9, so the fit comes from the band, inside the
   comfort bound (at most 4 clicks past first touch: 11 clicks, 220 deg).
3. Both hands let go and go back to the ready pose; the helmet stays level by the band's hold.

The run's log of the stated forces against the travel (check 51) goes to ``check51_log.json`` beside result.json.
"""

from __future__ import annotations

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import json
import math
from pathlib import Path
import sys

import numpy as np
from yam_arena import station
from yam_arena.task_file import TABLE_TOP

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import checks as C  # noqa: E402
import demo_kit as kit  # noqa: E402
from demo_kit import Move  # noqa: E402
import parts as P  # noqa: E402

TOP = TABLE_TOP["station"]

TARGET_TILT_DEG = 2.6  # front edge 25.2 mm over the brow band: the middle of the fit window (15 to 35 mm)
HOLD_LIFT_M = 0.0008  # closed fingers 0.8 mm over the rim's top: the helmet rocks back onto them (3 mm gave 27.7 mm, h4t2c)
STROKE_DEG = 75.0  # a full regrasp: wrist roll -37.5 to +37.5 deg (the jaws close on a flat at both ends, parts.DIAL_FLATS)
# Wrist roll (from, to) per regrasp, 210 deg in all (10.5 clicks of 15; a stroke gives ~96 %, ~203 deg: CLAMP_N came at
# 193.5 deg and 75 deg strokes ended at 72.8, 145.4 and 217.4 deg in run h4d1, so the last is 60 deg, under 220 deg).
# Every regrasp ends at +37.5 deg: from the short one ending at -7.5 deg the way back swept the open fingers across the
# dial and bumped the helmet 1.7 deg (runs h4t2d, h4t2e); from +37.5 deg it did not (h4t2c).
STROKES = ((-37.5, 37.5), (-37.5, 37.5), (-22.5, 37.5))
ENGAGE_BACK_M = 0.004  # grasp point 4 mm behind the dial's middle: fingertips end 9 mm past it, 5.5 mm short of the band
RETREAT_M = 0.022  # open jaws back off 22 mm along the dial's axis between regrasps
BACK_OFF_M = 0.060  # after the last regrasp: 60 mm back along the axis, then straight to the ready pose
GRIP_OPEN, GRIP_PINCH = 0.62, 0.20  # 58 mm open; pinch commanded at 19 mm on the 27.8 mm flats


def build_moves() -> dict[str, list[Move]]:
    ready_pos_l, ready_rot_l = kit.Arm("left", station.BASES["left"], []).state(0.0)[:2]
    ready_pos_r, ready_rot_r = kit.Arm("right", station.BASES["right"], []).state(0.0)[:2]

    # Left: over the forehead (its wrist camera sees the brow band and the front edge), press the rim down, hold.
    p_look, r_look = C.left_press_pose(P.TILT_START_DEG, lift_m=0.015)
    p_look = p_look + np.array([0.0, 0.085, 0.0], dtype=float)
    p_start, r_start = C.left_press_pose(P.TILT_START_DEG, lift_m=0.004)
    p_mid, r_mid = C.left_press_pose(0.5 * P.TILT_START_DEG, lift_m=0.003)
    p_hold, r_hold = C.left_press_pose(TARGET_TILT_DEG, lift_m=HOLD_LIFT_M)
    p_up, r_up = C.left_press_pose(TARGET_TILT_DEG, lift_m=0.085)
    left_moves: list[Move] = [
        Move(0.80, pos=p_look, rot=r_look, grip=0.0, label="L_look_brow"),
        Move(0.50, pos=p_start, rot=r_start, grip=0.0, label="L_touch_start"),
        Move(0.65, pos=p_mid, rot=r_mid, grip=0.0, label="L_press_mid"),
        Move(0.65, pos=p_hold, rot=r_hold, grip=0.0, label="L_hold_level"),
    ]

    # Right: wait until the helmet is at the fit (the dial is then up at 86 mm), then the regrasps.
    from yam_arena import ik

    p_stage, r_stage = ik.tcp_pose(np.radians([10.0, 118.0, 42.0, 52.0, -58.0, 0.0]), station.BASES["right"])
    pose = lambda roll, back: C.right_dial_pose(roll_deg=roll, retreat_m=back, tilt_deg=TARGET_TILT_DEG)  # noqa: E731
    p_out_first, r_out_first = pose(STROKES[0][0], ENGAGE_BACK_M + RETREAT_M)
    right_moves: list[Move] = [
        Move(2.65, pos=ready_pos_r, rot=ready_rot_r, grip=GRIP_OPEN, label="R_wait_level"),
        Move(0.75, pos=p_stage, rot=r_stage, grip=GRIP_OPEN, label="R_stage_wrist"),
        Move(0.75, pos=p_out_first, rot=r_out_first, grip=GRIP_OPEN, label="R_align_dial"),
    ]
    for k, (start, end) in enumerate(STROKES):
        tag, f = f"s{k + 1}", (end - start) / STROKE_DEG  # f: share of a full regrasp's turning time
        p_in_lo, r_in_lo = pose(start, ENGAGE_BACK_M)
        p_in_mid, r_in_mid = pose(0.5 * (start + end), ENGAGE_BACK_M)
        p_in_hi, r_in_hi = pose(end, ENGAGE_BACK_M)
        p_out_hi, r_out_hi = pose(end, ENGAGE_BACK_M + RETREAT_M)
        right_moves += [
            Move(0.40, pos=p_in_lo, rot=r_in_lo, grip=GRIP_OPEN, label=f"R_{tag}_in"),
            Move(0.30, pos=p_in_lo, rot=r_in_lo, grip=GRIP_PINCH, label=f"R_{tag}_pinch"),
            Move(max(0.25, 0.55 * f), pos=p_in_mid, rot=r_in_mid, grip=GRIP_PINCH, label=f"R_{tag}_turn1"),
            Move(max(0.25, 0.55 * f), pos=p_in_hi, rot=r_in_hi, grip=GRIP_PINCH, label=f"R_{tag}_turn2"),
            Move(0.15, pos=p_in_hi, rot=r_in_hi, grip=GRIP_PINCH, label=f"R_{tag}_settle"),
            Move(0.30, pos=p_in_hi, rot=r_in_hi, grip=GRIP_OPEN, label=f"R_{tag}_open"),
            Move(0.35, pos=p_out_hi, rot=r_out_hi, grip=GRIP_OPEN, label=f"R_{tag}_out"),
        ]
        if k + 1 < len(STROKES):
            p_next, r_next = pose(STROKES[k + 1][0], ENGAGE_BACK_M + RETREAT_M)
            right_moves.append(Move(0.45, pos=p_next, rot=r_next, grip=GRIP_OPEN, label=f"R_{tag}_reset"))

    # Left holds until the right hand has backed off the dial, then both go back to the ready pose.
    hold_s = round(max(0.20, sum(m.seconds for m in right_moves) - sum(m.seconds for m in left_moves) + 0.10), 2)
    left_moves += [
        Move(hold_s, pos=p_hold, rot=r_hold, grip=0.0, label="L_hold_steady"),
        Move(0.55, pos=p_up, rot=r_up, grip=0.0, label="L_lift_clear"),
        Move(0.75, pos=ready_pos_l, rot=ready_rot_l, grip=1.0, label="L_ready"),
    ]
    # By way of the wrist's staging pose the open fingers swept the dial on the way back (run h4t2h: dial +14 deg,
    # helmet tipped back 4 deg); backed off along the axis first, the hand stays 14 mm or more from it.
    p_far, r_far = pose(STROKES[-1][1], ENGAGE_BACK_M + BACK_OFF_M)
    right_moves += [
        Move(0.45, pos=p_far, rot=r_far, grip=GRIP_OPEN, label="R_back_off"),
        Move(0.75, pos=ready_pos_r, rot=ready_rot_r, grip=1.0, label="R_ready"),
    ]
    return {"left": left_moves, "right": right_moves}


def face_clearance(moves: dict[str, list[Move]], hz: float = 10.0) -> dict:
    """Least distance (mm) from either arm's link hulls (link4 to the fingertips) to the head form's face along the
    whole plan, with where it occurs."""
    face = P.face_bounds_world(TOP)
    out = {}
    for side in ("left", "right"):
        arm = kit.Arm(side, station.BASES[side], moves[side])
        best, there, i = math.inf, None, 0
        while i / hz <= arm.seconds + 0.5:
            action, label = arm.action(i / hz)
            d = 1000.0 * C.to_box(C.hulls(side, np.asarray(action[:6]), float(action[6]) * C.MAX_JAW), face)
            if d < best:
                best, there = d, f"{label} at {i / hz:.1f} s"
            i += 1
        out[side] = {"mm": round(best, 1), "there": there}
    return out


def check51_log(rows: list[dict]) -> dict:
    """The stated forces against the travel, from the run's rows: per phase and regrasp, the tilt and front-edge gap,
    the dial's angle and clicks, the band's press on the head, the dial's turning torque, and each arm's push
    (demo_kit.spring_force: what the position-driven arm presses with, N)."""
    def arm_n(row, side):
        return round(float(np.linalg.norm(row["force"][side])), 2)

    phases: dict[str, dict] = {}
    for row in rows:
        e = row["extra"]
        for side in ("left", "right"):
            key = f"{side}:{row[side]}"
            ph = phases.setdefault(key, {"from_s": row["t"], "dial_from": e["dial_deg"], "tilt_from": e["tilt_deg"],
                                         "band_max_n": 0.0, "dial_nm_max": 0.0, "push_max_n": 0.0})
            ph.update(to_s=row["t"], dial_to=e["dial_deg"], tilt_to=e["tilt_deg"], gap_to=e["gap_mm"], band_to_n=e["band_n"])
            ph["band_max_n"] = max(ph["band_max_n"], e["band_n"])
            ph["dial_nm_max"] = max(ph["dial_nm_max"], e["dial_nm"])
            ph["push_max_n"] = max(ph["push_max_n"], arm_n(row, side))
    every = [{"t": r["t"], **{k: r["extra"][k] for k in ("tilt_deg", "gap_mm", "dial_deg", "clicks", "band_n", "dial_nm")},
              "left_push_n": arm_n(r, "left"), "right_push_n": arm_n(r, "right"), "left": r["left"], "right": r["right"]}
             for r in rows[::5]]
    return {"phases": phases, "every_0p25_s": every}


def main() -> int:
    args = sys.argv[1:]
    check_only, dry_only = "--check" in args, "--dry" in args
    seed = 0
    positional = []
    for a in args:
        if a in ("--check", "--dry"):
            continue
        if a.startswith("seed="):
            seed = int(a.split("=", 1)[1])
        else:
            positional.append(a)

    moves = build_moves()
    bases = {s: station.BASES[s] for s in ("left", "right")}
    dry = kit.dry_run(moves, bases, tail_s=1.8)
    face = face_clearance(moves)
    print("DEMO_DRY " + json.dumps({"arms": dry, "face_clearance_mm": face}, indent=2), flush=True)
    for side, info in dry.items():
        if info["worst_ik_mm"] > 1.0 or info["smallest_margin_deg"] < 5.0 or face[side]["mm"] < 1000.0 * P.MIN_FACE_ARM_CLEAR_M:
            print(f"DRY_FAIL {side}: {info} face {face[side]}", flush=True)
            return 2
    if dry_only:
        return 0

    out_dir = Path(positional[0]) if positional else HERE / "demo_out"
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    import mechanism as l_mod

    def step_extra():
        last = l_mod.STATE.get("last", {})
        return {
            "tilt_deg": round(l_mod.tilt_deg(), 2),
            "gap_mm": round(1000.0 * m.gap_m(), 2),
            "dial_deg": round(l_mod.dial_deg(), 1),
            "clicks": round(l_mod.dial_clicks(), 2),
            "band_n": round(l_mod.band_force_n(), 2),
            "dial_nm": round(float(last.get("dial_active_nm", 0.0)), 3),
            "tilt_hold_nm": round(l_mod.tilt_hold_nm(), 3),
        }

    last: dict = {}

    def keep_last(t, arms, images):  # a director that only keeps the newest top-camera picture (the end picture)
        if images is not None:
            last["top"] = images["top_cam"]

    res = kit.run(emb, moves, out_dir, seed=seed, director=keep_last, tail_s=1.8, limit_s=m.RUN_LIMIT_STEPS / 20.0,
                  extra=step_extra, record=not check_only)
    if "top" in last:
        import cv2

        cv2.imwrite(str(out_dir / "end_top.png"), cv2.cvtColor(last["top"], cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(out_dir / "end_top_policy.png"), cv2.cvtColor(kit.policy_picture(last["top"]), cv2.COLOR_RGB2BGR))
    rep = m.report()
    (out_dir / "task_report.json").write_text(json.dumps(rep, indent=2))
    (out_dir / "check51_log.json").write_text(json.dumps(check51_log(res["rows"]), indent=1))
    keys = ("tilt_deg", "gap_mm", "dial_deg", "dial_clicks", "first_touch_dial_deg", "clamp_dial_deg", "band_force_n",
            "peak_band_n", "tilt_hold_nm", "zero_reason", "face_touch_steps")
    print("DEMO_RESULT " + json.dumps({"score": res.get("info", {}).get("score"), "success": res.get("info", {}).get("success"),
                                       "steps": res.get("steps"), "seconds": res.get("seconds"),
                                       "report_head": {k: rep[k] for k in keys}}, indent=2), flush=True)
    return 0 if float(res.get("info", {}).get("score", 0.0)) == 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
