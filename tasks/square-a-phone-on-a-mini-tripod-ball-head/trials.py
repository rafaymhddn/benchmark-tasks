#!/usr/bin/env python3
"""Simulator mechanics checks for ``pixi_level`` (G1 pictures + G2 physical trials, zero MP4 recording):

    scripts/rta simpy <id> mech/tasks/pixi_level/trials.py /workspace/robocurve/outputs/<id> [rest,flop,slip,force,letgo,order]

- ``rest``  (G1): saves ``start_top.png``, ``start_left_wrist.png``, ``start_right_wrist.png`` and their 320x176
            ``*_policy.png`` crops, steps 3 s idle to confirm the locked tilted start holds still (score 0.0), then
            moves the right wrist camera to frame the red release button and saves ``cue_button_right_wrist.png`` and
            ``cue_button_right_wrist_policy.png``.
- ``flop``  (G2a): presses the red button without holding the phone -> the freed phone flops under gravity to the
            35 deg stop (and checks tripod foot lift); also a one-sided button poke with the phone unheld, and a
            one-sided push by the right hand with the phone held by the left.
- ``slip``  (G2b): applies a calibrated perpendicular pull at the phone's end (lever arm 73.8 mm) against the locked
            head -> holds without plastic slip below ``LOCK_NM`` (1.50 N m) and slips past ``LOCK_NM``; also shows that
            pushing the phone on an unbraced tripod tips the tripod (> 10 mm foot lift) before the head slips.
- ``force`` (shortcut): tripod braced, the LOCKED phone forced level by torque pulses past ``LOCK_NM``, button never
            pressed -> ends level, at rest, button out and hands clear, yet the slip rule scores it 0.0.
- ``letgo``: the reference run with the phone let go while the button is still pressed (near level the free head holds it).
- ``order`` (G2c): bimanual hold -> press button -> level -> release button -> release phone -> stays level (score 1.0).
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as Ck  # noqa: E402
import parts as P  # noqa: E402

MAX_JAW = 0.094  # m: yam_arena parallel gripper full opening


def _hold_action():
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))


def _save_picture(out: Path, name: str, frame: np.ndarray) -> None:
    import cv2
    import demo_kit as kit

    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / f"{name}.png"), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    small = kit.policy_picture(frame)
    cv2.imwrite(str(out / f"{name}_policy.png"), cv2.cvtColor(small, cv2.COLOR_RGB2BGR))


def trial_rest(embodiment, out: Path, seed: int = 0) -> dict:
    """G1 start pictures, 3 s idle stability check, and right-wrist red-button cue picture."""
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import station
    from yam_arena.constants import READY
    import demo_kit as kit

    m = embodiment.task.module
    mk = m.mk
    obs = embodiment.reset(Scene(id="rest", instruction=m.INSTRUCTION), seed=seed)
    for cam, fname in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        _save_picture(out, fname, np.asarray(obs.images[cam]))

    embodiment.step(_hold_action())
    t0_deg, r0_deg = m.phone_tilt_roll_deg()
    p0 = {name: mk.pose(name)[0].copy() for name in m.SCORED_BODIES}
    hz = float(embodiment.info.control_hz)
    for _ in range(int(round(3.0 * hz))):
        embodiment.step(_hold_action())
    t1_deg, r1_deg = m.phone_tilt_roll_deg()
    drift_mm = {name: round(1000.0 * float(np.linalg.norm(mk.pose(name)[0] - p0[name])), 3) for name in m.SCORED_BODIES}
    idle_score = m.score(None)

    # Move right arm to frame the red button in right_cam
    look = Ck.solve_right_look_pose()
    look_saved = False
    if look is not None:
        _, look_tcp, look_rot, _, _, _, _ = look
        arm_r = kit.Arm("right", station.BASES["right"], [kit.Move(1.6, pos=look_tcp, rot=look_rot, grip=0.65, label="look_button"), kit.Move(0.5, label="steady")])
        left_ready = np.r_[READY, 1.0]
        steps = int(round(2.1 * hz))
        for k in range(steps):
            right_act, _ = arm_r.action(k / hz)
            res = embodiment.step(Action(data=pack(left_ready, right_act)))
            obs = res.observation
        _save_picture(out, "cue_button_right_wrist", np.asarray(obs.images["right_cam"]))
        look_saved = True

    summary = {
        "seed": seed,
        "start_angles_deg": {"tilt": round(t0_deg, 2), "roll": round(r0_deg, 2)},
        "after_3s_angles_deg": {"tilt": round(t1_deg, 2), "roll": round(r1_deg, 2)},
        "drift_3s_mm": drift_mm,
        "max_foot_lift_mm": round(float(m.STATE["max_foot_lift_mm"]), 3),
        "idle_score": idle_score,
        "cue_button_right_wrist_saved": look_saved,
    }
    (out / "rest.json").write_text(json.dumps(summary, indent=2))
    print("REST " + json.dumps(summary), flush=True)
    return summary


def right_pinch_moves(kit, hold_s: float = 2.0) -> list:
    """Moves for the right hand to approach from front, pinch the red button through its 3 mm stroke, and retreat."""
    Move = kit.Move
    r_pinch = Ck.RIGHT_PINCH_ROT_RAW
    horiz = np.array([Ck.BTN_AXIS[1], -Ck.BTN_AXIS[0], 0.0], dtype=float)
    pre_pos = Ck.BUTTON_TCP - 0.080 * horiz
    open_grip = round(min(1.0, (P.HEAD_PINCH_SPAN + 0.026) / MAX_JAW), 3)  # ~64 mm open around 38 mm span
    squeeze_grip = 0.24  # ~31 N per pad across 35.0 mm pressed span (> 18 N full button stroke, < 40 N limit)
    tcp_pressed = Ck.BUTTON_TCP + 0.5 * P.BUTTON_TRAVEL * Ck.BTN_AXIS
    return [
        Move(1.8, pos=pre_pos, rot=r_pinch, grip=open_grip, label="approach_head"),
        Move(1.1, pos=Ck.BUTTON_TCP, rot=r_pinch, grip=open_grip, label="at_button"),
        Move(0.7, pos=tcp_pressed, grip=squeeze_grip, label="press_button"),
        Move(hold_s, pos=tcp_pressed, grip=squeeze_grip, label="hold_pressed"),
        Move(0.6, pos=Ck.BUTTON_TCP, grip=open_grip, label="release_button"),
        Move(1.2, pos=pre_pos, grip=open_grip, label="retreat_head"),
    ]


def trial_flop(embodiment, out: Path, seed: int = 0) -> dict:
    """G2(a): button pressed without holding the phone -> phone flops to the 35 deg stop; plus one-sided button poke."""
    from inspect_robots import Scene
    import demo_kit as kit

    m = embodiment.task.module
    mk = m.mk

    # Case 1: Clean two-finger button pinch with nobody holding the phone
    res_pinch = kit.run(
        embodiment,
        {"right": right_pinch_moves(kit, hold_s=1.8)},
        out / "flop_pinch",
        seed=seed,
        tail_s=1.2,
        limit_s=15.0,
        record=False,
    )
    t_end, r_end = m.phone_tilt_roll_deg()
    pinch_case = {
        "end_tilt_deg": round(t_end, 2),
        "end_roll_deg": round(r_end, 2),
        "joint_tilt_deg": round(math.degrees(float(mk.joint("tilt"))), 2),
        "joint_roll_deg": round(math.degrees(float(mk.joint("roll"))), 2),
        "max_button_mm": round(float(m.H.STATE["max_button_mm"]), 2),
        "max_button_spring_n": round(float(m.H.STATE["max_button_spring_n"]), 2),
        "max_foot_lift_mm": round(float(m.STATE["max_foot_lift_mm"]), 2),
        "max_foot_slide_mm": round(float(m.STATE["max_foot_slide_mm"]), 2),
        "tripod_tipped_over_10mm": float(m.STATE["max_foot_lift_mm"]) > 1000.0 * P.FOOT_LIFT_MAX,
        "zero_fault": m.STATE["zero_fault"],
        "score": res_pinch["info"].get("score"),
        "events": [f"{t:.2f} s {w}" for t, w in m.STATE["events"]],
    }

    # Case 2: One-sided 18 N push on the button without counter-pinching the head -> tips/slides tripod immediately
    embodiment.reset(Scene(id="flop_poke", instruction=m.INSTRUCTION), seed=seed)
    hz = float(embodiment.info.control_hz)
    inward = np.array(P.button_slide_axis(), dtype=float)
    for _ in range(int(round(0.8 * hz))):
        mk.push("button", force=tuple(P.BUTTON_N * inward))
        embodiment.step(_hold_action())
    mk.push("button")
    for _ in range(int(round(0.6 * hz))):
        embodiment.step(_hold_action())
    poke_case = {
        "max_foot_lift_mm": round(float(m.STATE["max_foot_lift_mm"]), 2),
        "max_foot_slide_mm": round(float(m.STATE["max_foot_slide_mm"]), 2),
        "zero_fault": m.STATE["zero_fault"],
        "score": m.score(None),
    }

    held_case = held_one_sided_push(embodiment, out / "flop_held_push", seed=seed)

    summary = {"unheld_button_pinch": pinch_case, "one_sided_button_poke": poke_case, "held_one_sided_push": held_case}
    (out / "flop.json").write_text(json.dumps(summary, indent=2))
    print("FLOP " + json.dumps(summary), flush=True)
    return summary


def held_one_sided_push(embodiment, out: Path, seed: int = 0, push_n: float = P.BUTTON_N + 2.0) -> dict:
    """The phone held by the left hand (the demo's grasp), the button pushed from one side by the right hand: the hand
    comes in as for the pinch, moves along the button axis until its outer pad meets the button face (the inner pad
    stays 26 mm clear of the neck), presses ``push_n`` inwards for 1.5 s (demo_kit push feed-forward) and backs off."""
    import demo
    import demo_kit as kit

    m = embodiment.task.module
    mk = m.mk
    Move = kit.Move
    added = {"done": False}

    def director(t: float, arms: dict, pictures) -> None:
        if added["done"] or pictures is None or not mk.MECH.poses:
            return
        added["done"] = True
        plan = demo.Plan(m, live=True)
        hold = next(plan.steps())  # the demo's left grasp; the right waits
        r_tcp, r_rot = plan._button_tcp_and_rot()
        inward = -r_rot[:, 0]
        half_open = 0.5 * demo.OPEN_BUTTON * MAX_JAW
        touch = r_tcp + (half_open - 0.5 * P.HEAD_PINCH_SPAN) * inward
        r_horiz = np.array([r_rot[0, 2], r_rot[1, 2], 0.0], dtype=float)
        pre = r_tcp - 0.080 * r_horiz / np.linalg.norm(r_horiz)
        right = [
            Move(1.6, pos=pre, rot=r_rot, grip=demo.OPEN_BUTTON, label="approach_button"),
            Move(1.1, pos=r_tcp, rot=r_rot, grip=demo.OPEN_BUTTON, label="around_button"),
            Move(0.6, pos=touch, label="pad_on_button"),
            Move(0.7, pos=touch + (P.BUTTON_TRAVEL + 0.0005) * inward, push=tuple(push_n * inward), push_at=(half_open, 0.0, 0.0), label="push_one_side"),
            Move(1.5, label="hold_push"),
            Move(0.6, pos=r_tcp, push=(0.0, 0.0, 0.0), label="back_off"),
            Move(1.0, pos=pre, label="retreat"),
        ]
        wait = sum(mv.seconds for mv in hold["left"])
        arms["left"].moves.extend([*hold["left"], Move(sum(mv.seconds for mv in right), label="holds_phone")])
        arms["right"].moves.extend([Move(wait, label="waits"), *right])

    def extra() -> dict:
        t_deg, r_deg = m.phone_tilt_roll_deg()
        return {
            "tilt_deg": round(t_deg, 2),
            "roll_deg": round(r_deg, 2),
            "button_mm": round(1000.0 * m.H.button_travel_m(), 2),
            "unlocked": m.H.is_unlocked(),
            "foot_slide_mm": round(1000.0 * float(np.max(m.foot_slides_m())), 2),
            "foot_lift_mm": round(1000.0 * float(np.max(m.foot_lifts_m())), 2),
            "r_btn_n": round(sum(abs(c.force) for c in mk.touching("button", "right robot")), 2),
            "r_head_n": round(sum(abs(c.force) for c in mk.touching("tripod", "right robot")), 2),
            "l_phone_n": round(sum(abs(c.force) for c in mk.touching("phone", "left robot")), 2),
        }

    res = kit.run(embodiment, {}, out, seed=seed, director=director, extra=extra, tail_s=1.0, limit_s=16.0, record=False)
    rows = [r for r in res["rows"] if r.get("right") in ("push_one_side", "hold_push")]
    first_slide = next((r["t"] for r in res["rows"] if r["extra"]["foot_slide_mm"] > 1000.0 * P.FOOT_SLIDE_MAX), None)
    return {
        "push_n": push_n,
        "max_button_mm": round(float(m.H.STATE["max_button_mm"]), 2),
        "unlocked_while_pushing": any(r["extra"]["unlocked"] for r in rows),
        "max_right_button_n": round(max((r["extra"]["r_btn_n"] for r in rows), default=0.0), 2),
        "max_right_head_n": round(max((r["extra"]["r_head_n"] for r in rows), default=0.0), 2),
        "left_phone_n_while_pushing": [min((r["extra"]["l_phone_n"] for r in rows), default=0.0), max((r["extra"]["l_phone_n"] for r in rows), default=0.0)],
        "max_foot_lift_mm": round(float(m.STATE["max_foot_lift_mm"]), 2),
        "max_foot_slide_mm": round(float(m.STATE["max_foot_slide_mm"]), 2),
        "slide_over_20mm_at_s": first_slide,
        "phone_deg_end_of_hold": [rows[-1]["extra"]["tilt_deg"], rows[-1]["extra"]["roll_deg"]] if rows else None,
        "zero_fault": m.STATE["zero_fault"],
        "events": [f"{t:.2f} s {w}" for t, w in m.STATE["events"]],
        "score": res["info"].get("score"),
    }


def trial_slip(embodiment, out: Path, seed: int = 0) -> dict:
    """G2(b): locked head under a growing perpendicular pull at the phone's end (lever arm 73.8 mm):
    1. Tripod braced on bench (as in the bench luggage-scale test): holds below LOCK_NM (1.50 N m), slips past LOCK_NM;
    2. Tripod unbraced on bench: lateral push at the phone's end tips the tripod (> 10 mm foot lift) before 1.50 N m."""
    from inspect_robots import Scene

    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    lever_m = 0.5 * P.PHONE_L  # 73.8 mm from ball axis to phone's end

    # 1. Braced tripod: ramp torque on roll axis in steps, checking elastic recovery below LOCK_NM and plastic slip above
    embodiment.reset(Scene(id="slip_braced", instruction=m.INSTRUCTION), seed=seed)
    embodiment.step(_hold_action())
    r_init_deg = math.degrees(float(mk.joint("roll")))
    braced_rows = []
    slip_onset_nm = None

    for tau_nm in (0.50, 1.00, 1.35, 1.55, 1.75, 2.00):
        pull_n = tau_nm / lever_m
        # Apply perpendicular torque about phone's roll axis while holding tripod base firmly on bench (-60 N preload)
        for _ in range(int(round(0.5 * hz))):
            _, pr = mk.pose("phone")
            roll_axis_w = pr[:, 0]
            mk.push("tripod", force=(0.0, 0.0, -60.0))
            mk.push("phone", torque=tuple(tau_nm * roll_axis_w))
            embodiment.step(_hold_action())
        loaded_deg = math.degrees(float(mk.joint("roll")))
        # Unload to read permanent plastic slip
        mk.push("phone")
        for _ in range(int(round(0.35 * hz))):
            mk.push("tripod", force=(0.0, 0.0, -60.0))
            embodiment.step(_hold_action())
        unloaded_deg = math.degrees(float(mk.joint("roll")))
        plastic_slip_deg = unloaded_deg - r_init_deg
        if slip_onset_nm is None and abs(plastic_slip_deg) >= 0.5:
            slip_onset_nm = tau_nm
        braced_rows.append(
            {
                "applied_torque_nm": round(tau_nm, 2),
                "equiv_end_force_n": round(pull_n, 2),
                "loaded_roll_deg": round(loaded_deg, 2),
                "unloaded_roll_deg": round(unloaded_deg, 2),
                "plastic_slip_deg": round(plastic_slip_deg, 2),
                "slipped_flag": bool(m.H.STATE["slipped"]["roll"]),
            }
        )
    mk.push("tripod")

    # 2. Unbraced tripod: push horizontally on phone without pressing button -> tripod tips well below LOCK_NM
    embodiment.reset(Scene(id="slip_unbraced", instruction=m.INSTRUCTION), seed=seed)
    embodiment.step(_hold_action())
    tip_force_n = None
    for f_n in (1.5, 3.0, 4.5, 6.0):
        for _ in range(int(round(0.4 * hz))):
            mk.push("phone", force=(f_n, 0.0, 0.0))
            embodiment.step(_hold_action())
        if float(m.STATE["max_foot_lift_mm"]) > 1000.0 * P.FOOT_LIFT_MAX:
            tip_force_n = f_n
            break
    mk.push("phone")
    unbraced = {
        "tip_force_n": tip_force_n,
        "max_foot_lift_mm": round(float(m.STATE["max_foot_lift_mm"]), 2),
        "roll_change_deg": round(math.degrees(float(mk.joint("roll"))) - P.START_ROLL_DEG, 2),
        "zero_fault": m.STATE["zero_fault"],
        "score": m.score(None),
    }

    summary = {
        "lock_nm_spec": P.LOCK_NM,
        "lever_arm_mm": round(1000.0 * lever_m, 1),
        "slip_onset_nm": slip_onset_nm,
        "braced_ramp": braced_rows,
        "unbraced_push": unbraced,
    }
    (out / "slip.json").write_text(json.dumps(summary, indent=2))
    print("SLIP " + json.dumps(summary), flush=True)
    return summary


def trial_force(embodiment, out: Path, seed: int = 0) -> dict:
    """Shortcut check: the tripod braced on the bench (-60 N, as in ``slip``) and the LOCKED phone forced level by torque
    pulses past ``LOCK_NM`` about both hinge axes, button never pressed, both arms at READY. Each pulse lasts one control
    step; the next is sized from the angle read after the head springs back (adapted per axis: weaker after an
    overshoot, stronger when stuck). Reports the slip, the end state and the score, and what the end rules alone read."""
    from inspect_robots import Scene

    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    brace = (0.0, 0.0, -60.0)
    embodiment.reset(Scene(id="force_locked", instruction=m.INSTRUCTION), seed=seed)
    embodiment.step(_hold_action())
    _, tri_r0 = mk.built_pose("tripod")
    extra = {"tilt": 0.10, "roll": 0.10}  # N m above LOCK_NM
    pulses = []
    for _ in range(60):
        t_deg, r_deg = m.phone_tilt_roll_deg()
        if max(abs(t_deg), abs(r_deg)) <= 1.0:
            break
        _, tr = mk.pose("tripod")
        _, pr = mk.pose("phone")
        axes = {"tilt": tr @ tri_r0.T @ np.array([0.0, 1.0, 0.0]), "roll": pr[:, 0]}  # hinge axes, world
        err = {"tilt": -t_deg, "roll": -r_deg}
        live = [ax for ax in axes if abs(err[ax]) > 0.5]
        torque = sum((math.copysign(P.LOCK_NM + extra[ax], err[ax]) * axes[ax] for ax in live), np.zeros(3))
        mk.push("tripod", force=brace)
        mk.push("phone", torque=tuple(float(v) for v in torque))
        embodiment.step(_hold_action())
        mk.push("phone")
        for _ in range(int(round(0.2 * hz))):
            embodiment.step(_hold_action())
        t2, r2 = m.phone_tilt_roll_deg()
        after = {"tilt": -t2, "roll": -r2}
        for ax in live:
            if after[ax] * err[ax] < 0.0 and abs(after[ax]) > 0.5:
                extra[ax] *= 0.5
            elif abs(after[ax] - err[ax]) < 0.3:
                extra[ax] = min(1.0, extra[ax] + 0.05)
        pulses.append({"tilt_deg": round(t2, 2), "roll_deg": round(r2, 2), "extra_nm": {k: round(v, 3) for k, v in extra.items()}})
    mk.push("tripod")

    rest_s, held_s = 0.0, 0.0
    for _ in range(int(round(3.0 * hz))):
        embodiment.step(_hold_action())
        rest_s = rest_s + 1.0 / hz if (m.grippers_clear() and m.button_out() and m.bodies_at_rest()) else 0.0
        held_s = max(held_s, rest_s)
    t_end, r_end = m.phone_tilt_roll_deg()
    lean = max(abs(t_end), abs(r_end))
    end_rules_only = 0.0
    if held_s >= P.REST_HOLD_S - 1e-9 and m.grippers_clear() and m.button_out() and m.bodies_at_rest():
        end_rules_only = 1.0 if lean <= P.LEVEL_DEG + 1e-9 else (0.5 if lean <= P.PARTIAL_DEG + 1e-9 else 0.0)
    summary = {
        "pulses": len(pulses),
        "pulse_rows": pulses,
        "end_tilt_deg": round(t_end, 2),
        "end_roll_deg": round(r_end, 2),
        "slip_deg": {k: round(float(v), 2) for k, v in m.H.STATE["slip_deg"].items()},
        "max_button_mm": round(float(m.H.STATE["max_button_mm"]), 2),
        "button_out": m.button_out(),
        "at_rest_held_s": round(held_s, 2),
        "max_foot_lift_mm": round(float(m.STATE["max_foot_lift_mm"]), 2),
        "max_foot_slide_mm": round(float(m.STATE["max_foot_slide_mm"]), 2),
        "zero_fault": m.STATE["zero_fault"],
        "events": [f"{t:.2f} s {w}" for t, w in m.STATE["events"]],
        "score": m.score(None),
        "end_rules_without_slip_rule": end_rules_only,
    }
    (out / "force.json").write_text(json.dumps(summary, indent=2))
    print("FORCE " + json.dumps({k: v for k, v in summary.items() if k != "pulse_rows"}), flush=True)
    return summary


def trial_letgo(embodiment, out: Path, seed: int = 0) -> dict:
    """The reference run with the last two releases swapped: the left hand lets go of the levelled phone and retreats
    while the right still holds the button pressed (head free), then the right lets the button go. Reports the phone's
    angles as the left lets go and when the button relocks the head, and the score."""
    import demo

    res = demo.run_demo(embodiment, out / "letgo", seed=seed, record=False, let_go_first=True)
    rows = json.loads((out / "letgo" / "result.json").read_text())["rows"]
    labels = ("release_phone", "retreat_phone")
    let_go = [r for r in rows if r.get("left") in labels]
    pick = lambda r: {k: r["extra"][k] for k in ("tilt_deg", "roll_deg", "button_mm", "unlocked")}  # noqa: E731
    summary = {
        "at_let_go": pick(let_go[0]) if let_go else None,
        "left_retreated": pick(let_go[-1]) if let_go else None,
        "relocked": [e for e in res["report"]["events"] if "locked at" in e],
        "end_tilt_deg": res["report"].get("phone_tilt_deg"),
        "end_roll_deg": res["report"].get("phone_roll_deg"),
        "zero_fault": res["report"]["zero_fault"],
        "score": res["score"],
        "seconds": res["seconds"],
    }
    (out / "letgo.json").write_text(json.dumps(summary, indent=2))
    print("LETGO " + json.dumps(summary), flush=True)
    return summary


def trial_order(embodiment, out: Path, seed: int = 0) -> dict:
    """G2(c): held -> button pressed -> levelled -> button released -> phone released -> stays level (score 1.0)."""
    import demo

    res = demo.run_demo(embodiment, out / "order", seed=seed, record=False)
    (out / "order.json").write_text(json.dumps(res, indent=2))
    print("ORDER " + json.dumps(res), flush=True)
    return res


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    wanted = sys.argv[2].split(",") if len(sys.argv) > 2 else ["rest", "flop", "slip", "force", "letgo", "order"]
    seed = 0
    for arg in sys.argv[3:]:
        if arg.startswith("seed="):
            seed = int(arg.split("=", 1)[1])

    embodiment = kit.open_task(HERE / "task.py")
    table = {
        "rest": trial_rest,
        "flop": trial_flop,
        "slip": trial_slip,
        "force": trial_force,
        "letgo": trial_letgo,
        "order": trial_order,
    }
    results = {}
    for name in wanted:
        results[name] = table[name](embodiment, out, seed=seed)
    (out / "trials_summary.json").write_text(json.dumps(results, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
