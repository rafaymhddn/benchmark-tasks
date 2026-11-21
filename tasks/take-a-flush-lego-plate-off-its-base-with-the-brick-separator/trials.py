#!/usr/bin/env python3
"""Simulator trials for ``mech/tasks/lego_separator`` (no videos):

    scripts/rta simpy <id> mech/tasks/lego_separator/trials.py /workspace/robocurve/outputs/<id> [modes]

Modes (comma-separated; default ``start,pinch,lever,lever_free,pull,straight``):
- ``start``: start pictures (top, wrists; the policy's 320 x 176 crops), 3 s at rest: drift and the press gauge's zero.
- ``pinch`` (a): the right gripper pinches the target plate across its two ends, fingertips on the base's stud tops,
  then (a1) lifts straight up, (a2) rolls about the plate's far edge as a peel would, (a3) rolls it with the base
  held down by the left fingertips (a peel without the separator: not the task, scored 0).
- ``lever_free``: as (b) with the base not held: the plate's hinge tips the base up (zero rule).
- ``lever`` (b): pick, seat (aligned press), rock over the lower edge: the press gauge against the separator-plate
  contact force, and the force against travel of ``peel``.
- ``pull`` (c): as (b), then the hand-over and the left hand's pull along the studs: force against travel of ``pull``.
- ``straight`` (d): seat, the left fingertips hold the base down, the right hand pulls the separator straight up:
  which lets go first.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as C  # noqa: E402
import parts as P  # noqa: E402
import plan  # noqa: E402

Move = plan.Move
UP = plan.UP


def save_start_pictures(out: Path, images: dict) -> None:
    import cv2
    import demo_kit as kit

    out.mkdir(parents=True, exist_ok=True)
    for cam, stem in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        rgb = images[cam]
        pol = kit.policy_picture(rgb)
        cv2.imwrite(str(out / f"{stem}.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(out / f"{stem}_policy.png"), cv2.cvtColor(pol, cv2.COLOR_RGB2BGR))
        if cam == "top_cam":
            x3 = cv2.resize(pol, (pol.shape[1] * 3, pol.shape[0] * 3), interpolation=cv2.INTER_NEAREST)
            cv2.imwrite(str(out / "start_top_policy_x3.png"), cv2.cvtColor(x3, cv2.COLOR_RGB2BGR))


def extra_reader(m):
    """Per step: the press gauge, the contact sensor between separator and plate, and the joints."""
    mk = m.mk

    def read() -> dict:
        return {
            "press_n": round(m.press_force_n(), 3),
            "contact_sep_target_n": round(mk.contact_force("separator", "target"), 3),
            "contact_sep_base_n": round(mk.contact_force("separator", "base"), 3),
            "stop_nm": round(m.R.stop_moment_nm(), 5),
            "peel_deg": round(math.degrees(float(mk.joint("peel"))), 3),
            "pull_mm": round(1000.0 * float(mk.joint("pull")), 3),
            "rack_deg": round(math.degrees(float(mk.joint("rack"))), 2),
            "rack_x_mm": round(1000.0 * float(mk.joint("rack_x")), 3),
            "seated": not mk.is_cut("w_seat"),
            "off": mk.is_cut("w_target"),
            **_hand_and_touches(mk),
        }

    return read


def _hand_and_touches(mk) -> dict:
    """The separator in the right gripper's frame (mm, deg) and its touches (body/shape: N), for diagnosis."""
    out: dict = {}
    if "right" in mk.MECH.grippers:
        gp, rg = np.asarray(mk.gripper("right")[0]), mk.gripper_turn("right")
        sp, rs = mk.pose("separator")
        rel = rg.T @ rs
        out["sep_in_hand_mm"] = [round(1000.0 * float(v), 2) for v in rg.T @ (sp - gp)]
        out["sep_in_hand_rot"] = [round(float(v), 4) for v in rel.ravel()]
        out["grip_r"] = round(float(mk.gripper("right")[1]), 3)
    touches: dict = {}
    points = []
    sp, rs = mk.pose("separator")
    for c in mk.contacts("separator"):
        if c.separation < 0.0005:
            key = f"{c.other}/{c.shape}"
            touches[key] = round(touches.get(key, 0.0) + float(c.force), 3)
            if c.other in ("base", "target") and abs(float(c.force)) > 0.05:
                points.append({"on": key, "at_mm_in_S": [round(1000.0 * float(v), 2) for v in rs.T @ (np.asarray(c.point) - sp)],
                               "n": [round(float(v), 2) for v in c.normal], "sep_mm": round(1000.0 * float(c.separation), 3), "f": round(float(c.force), 3)})  # fmt: skip
    out["sep_touches"] = touches
    if points:
        out["sep_points"] = points[:8]
    return out


def run_start(embodiment, out: Path) -> dict:
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    obs = embodiment.reset(Scene(id="start", instruction=m.INSTRUCTION), seed=0)
    save_start_pictures(out, obs.images)
    hold = Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))
    settled, press = None, []
    for k in range(int(round(3.0 * hz))):
        embodiment.step(hold)
        press.append(m.press_force_n())
        if k == int(round(0.5 * hz)):
            settled = {n: mk.pose(n)[0].copy() for n in ("base", "target", "separator", "dish")}
    drift = {n: round(1000.0 * float(np.linalg.norm(mk.pose(n)[0] - settled[n])), 4) for n in settled}
    return {
        "drift_0.5_to_3.0_s_mm": drift,
        "tare_stop_nm": round(float(m.STATE["tare_stop_nm"]), 5),
        "tare_as_row_n": round(float(m.STATE["tare_stop_nm"]) / P.ROW_LEVER, 3),
        "press_after_tare_max_n": round(max(press[int(round(0.5 * hz)) :]), 3),
        "separator_pos_mm": [round(1000.0 * float(v), 1) for v in mk.pose("separator")[0]],
        "score": m.score(None),
    }


def run_pinch(embodiment, out: Path, roll: bool, braced: bool = False) -> dict:
    """(a) Pinch across the plate's two ends (31.8 mm), fingertips 0.3 mm over the base's stud tops (the ends stand
    1.5 mm proud of them), then lift (a1) or roll about the far edge by 15 deg (a2)."""
    import demo_kit as kit

    m = embodiment.task.module
    top = m.TOP
    target = np.array(P.target_built_pos(top)) - np.array([0.0, 0.0, P.OVER])
    rot = C.frame([0.0, 1.0, 0.0], [0.0, 0.0, -1.0])
    tip = kit.lowest_point(rot)  # the fingertips' lowest point from the grasp point, gripper frame
    g = np.array([target[0], target[1], top + P.BASE_T + P.STUD_H + 0.0003]) - rot @ tip
    w_open = plan.grip_for(P.PLATE_L + 0.016)
    right = [
        Move(2.2, g + 0.050 * UP, rot, w_open, "over plate"),
        Move(0.8, g + 0.008 * UP, rot, w_open, "align"),
        Move(1.0, g, rot, w_open, "down onto the stud tops"),
        Move(1.4, None, None, 0.0, "pinch the plate's ends"),
    ]
    left: list = []
    if braced:  # the left fingertips hold the base down first, as in the demo's lever
        touch, brot, bat = plan.Director(m, "lever").brace_pose()
        left = [Move(2.4, touch + 0.050 * UP, brot, 0.0, "over base"), Move(1.0, touch, brot, 0.0, "onto base"),
                Move(0.5, touch - 0.001 * UP, brot, 0.0, "brace base", push=(0.0, 0.0, -plan.BRACE_PUSH_N), push_at=bat)]  # fmt: skip
        right.insert(0, Move(sum(mv.seconds for mv in left), None, None, w_open, "wait for the brace"))
    if roll:
        hinge = np.array(P.peel_anchor_world(top)) - np.array([0.0, 0.0, P.OVER])
        axis = np.array([-1.0, 0.0, 0.0])  # the peel's axis: positive lifts the near end
        for k in range(1, 7):
            r = kit.about(axis, math.radians(15.0 * k / 6))
            right.append(Move(0.4, hinge + r @ (g - hinge), r @ rot, 0.0, f"roll {15.0 * k / 6:.1f} deg"))
    else:
        right.append(Move(1.5, g + 0.060 * UP, rot, 0.0, "lift"))
    right.append(Move(0.6, None, None, w_open, "let go"))
    tag = ("pinch_roll_braced" if braced else "pinch_roll") if roll else "pinch_lift"
    res = kit.run(embodiment, {"left": left, "right": right}, out / tag, seed=0, tail_s=0.5, record=False, extra=extra_reader(m))
    rep = res["report"]
    return {k: rep.get(k) for k in ("target_off_base", "off_with_seat", "peel_max_deg", "max_base_lift_mm", "max_base_slide_mm", "zero_reason", "events")} | {
        "score": res["info"].get("score", 0.0),
        "worst_ik": res["worst_ik"],
    }


def run_directed(embodiment, out: Path, mode: str) -> dict:
    import demo_kit as kit

    m = embodiment.task.module
    director = plan.Director(m, mode)
    res = kit.run(embodiment, {"left": [], "right": plan.pick_moves(m.TOP)}, out / mode, seed=0, director=director, tail_s=0.5,
                  limit_s=90.0, record=False, extra=extra_reader(m))  # fmt: skip
    rep = res["report"]
    rows = [r["extra"] for r in res["rows"]]
    seated_rows = [r for r in rows if r["press_n"] > 0.5 and not r["seated"] and not r["off"]]
    out_d = {
        "score": res["info"].get("score", 0.0),
        "seconds": res["seconds"],
        "worst_ik": res["worst_ik"],
        "director": director.log,
        "report": {k: v for k, v in rep.items() if k not in ("press_log",)},
        "press_vs_contact": [{"t": r["t"], "press_n": r["extra"]["press_n"], "contact_n": r["extra"]["contact_sep_target_n"]}
                             for r in res["rows"] if r["extra"]["press_n"] > 0.5 and not r["extra"]["off"]][:60],  # fmt: skip
        "steps_pressed": len(seated_rows),
        "brace_force_n": [round(float(np.mean([r["force"]["left"][2] for r in res["rows"] if r["left"] == lab] or [0.0])), 2)
                          for lab in ("brace base", "hold base")],  # fmt: skip
        "lever_hand_force_n": [{"t": r["t"], "asked": r["right"], "f_right": r["force"]["right"], "peel_deg": r["extra"]["peel_deg"]}
                               for r in res["rows"] if r["right"].startswith("rock")][::2],  # fmt: skip
    }
    (out / f"{mode}_summary.json").write_text(json.dumps(out_d, indent=1, default=str))
    return out_d


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    modes = (sys.argv[2] if len(sys.argv) > 2 else "start,pinch,lever,lever_free,pull,straight").split(",")
    out.mkdir(parents=True, exist_ok=True)
    embodiment = kit.open_task(HERE / "task.py")
    results: dict = {}
    for mode in modes:
        if mode == "start":
            results["start"] = run_start(embodiment, out)
        elif mode == "pinch":
            results["a1_pinch_lift"] = run_pinch(embodiment, out, roll=False)
            results["a2_pinch_roll"] = run_pinch(embodiment, out, roll=True)
            results["a3_pinch_roll_braced"] = run_pinch(embodiment, out, roll=True, braced=True)
        elif mode in ("lever", "lever_free", "pull", "straight"):
            key = {"lever": "b_seat_lever", "lever_free": "b2_lever_base_free", "pull": "c_pull_out", "straight": "d_straight_pull"}[mode]
            results[key] = run_directed(embodiment, out, mode)
        else:
            raise SystemExit(f"unknown trial mode {mode!r}")
        last = next(reversed(results))
        print(f"TRIAL {last}: {json.dumps(results[last], default=str)[:3000]}", flush=True)
        (out / "trials_summary.json").write_text(json.dumps(results, indent=1, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
