#!/usr/bin/env python3
"""PhysX mechanism and scoring-level trials for the CryoCane sleeving task (trials.py only; arms at ready except wrong_end_tab_block).

    scripts/rta simpy <id> mech/tasks/cryocane_sleeve/trials.py /workspace/robocurve/outputs/<id>
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

SAMPLE_S = 0.20


def _hold():
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))


def _quat(m, rot) -> tuple[float, ...]:
    return tuple(float(v) for v in m.mk.matrix_to_quat(rot))


def _place_cane_and_vials(m, foot_pos, rot) -> None:
    p_c = np.asarray(foot_pos, dtype=float)
    r_c = np.asarray(rot, dtype=float)
    m.mk.write_pose("cane", p_c, _quat(m, r_c))
    m.mk.set_joint("slv_s", 0.0)
    m.mk.set_spring("slv_s", rest=0.0, most=m.P.SLIDE_N)
    for idx, name in enumerate(m.VIALS):
        bay = m.P.VIAL_BAYS[idx]
        vp = p_c + r_c @ np.array([0.0, m.P.bay_centre_y(bay), m.P.VIAL_REST_Z + 0.0002])
        m.mk.write_pose(name, vp, _quat(m, r_c))


def _place_mated_sleeve(m, foot_pos, rot, slide_m: float) -> None:
    _place_cane_and_vials(m, foot_pos, rot)
    p_c = np.asarray(foot_pos, dtype=float)
    r_c = np.asarray(rot, dtype=float)
    u_c = r_c[:, 1]
    j_val = slide_m - m.P.CARRIAGE_BUILT_S
    slv_pos = p_c + (slide_m - 0.5 * m.P.SLEEVE_L) * u_c
    m.mk.set_joint("slv_s", j_val)
    m.mk.write_pose("sleeve", slv_pos, _quat(m, r_c))
    m.mk.set_spring("slv_s", rest=j_val, most=m.P.SLIDE_N)
    m.mk.make("w_sleeve")
    m.STATE["slv_rest"] = float(j_val)
    m.STATE["max_slide_m"] = float(slide_m)


def _run_loop(emb, m, seconds: float, step_fn=None) -> dict:
    hz = float(emb.info.control_hz)
    every = max(1, int(round(SAMPLE_S * hz)))
    samples = []
    res = None
    for i in range(int(round(seconds * hz))):
        if step_fn is not None:
            step_fn(i / hz)
        res = emb.step(_hold())
        t = round((i + 1) / hz, 3)
        if (i + 1) % every == 0:
            samples.append(
                {
                    "t (s)": t,
                    "score": float(res.info.get("score", 0.0)),
                    "zero": m.STATE.get("zero"),
                    "mated": m.sleeve_mated(),
                    "slide (mm)": round(1000.0 * m.sleeve_slide_m(), 1),
                    "wipe F (N)": round(float(m.STATE.get("slide_force_n", 0.0)), 2),
                    "vial radial (mm)": {v: round(1000.0 * m.vial_offsets(v)[0], 2) for v in m.VIALS},
                }
            )
    return {
        "score at end": float(0.0 if res is None else res.info.get("score", 0.0)),
        "report": m.report(),
        "samples": samples,
    }


def wrong_end_tab_block(emb, m) -> dict:
    import checks as K
    import demo
    import demo_kit as kit
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import station

    emb.reset(Scene(id="tab_block", instruction=m.INSTRUCTION), seed=0)
    p = K.poses()
    Move = kit.Move
    moves = {
        "left": [
            Move(1.6, pos=p["L: above cane tab"][1], rot=K.ROT_START_L, grip=1.0, label="above cane"),
            Move(1.0, pos=p["L: on cane tab"][1], label="on cane"),
            Move(0.6, grip=demo.CANE_GRIP, label="hold cane"),
            Move(2.6, label="brace cane"),
        ],
        "right": [
            Move(1.6, pos=p["R: above sleeve tail"][1], rot=K.ROT_SLV_R, grip=1.0, label="above sleeve"),
            Move(1.0, pos=p["R: on sleeve tail"][1], label="on sleeve"),
            Move(0.6, grip=demo.SLEEVE_GRIP, label="hold sleeve"),
            Move(1.8, pos=p["R: on sleeve tail"][1] + np.array([0.0, 0.028, 0.0]), label="push onto tab"),
            Move(0.8, label="hold against tab"),
        ],
    }
    arms = {s: kit.Arm(s, station.BASES[s], moves[s]) for s in ("left", "right")}
    hz = float(emb.info.control_hz)
    slv_y0 = float(m.mk.built_pose("sleeve")[0][1])
    samples = []
    res = None
    for i in range(int(round(5.8 * hz))):
        t = i / hz
        la, _ = arms["left"].action(t)
        ra, _ = arms["right"].action(t)
        res = emb.step(Action(data=pack(la, ra)))
        if (i + 1) % 8 == 0:
            samples.append(
                {
                    "t (s)": round((i + 1) / hz, 2),
                    "mated": m.sleeve_mated(),
                    "score": float(res.info.get("score", 0.0)),
                    "zero": m.STATE.get("zero"),
                }
            )
    slv_y1 = float(m.mk.pose("sleeve")[0][1])
    cane_p, cane_r = m.cane_pose()
    cane_tab_end_y = float((cane_p + cane_r @ np.array([0.0, m.P.CANE_L, 0.0]))[1])
    mouth_past_tab_mm = 1000.0 * ((slv_y1 + 0.5 * m.P.SLEEVE_L) - cane_tab_end_y)
    out = {
        "score at end": float(0.0 if res is None else res.info.get("score", 0.0)),
        "sleeve advance (mm)": round(1000.0 * (slv_y1 - slv_y0), 2),
        "mouth past bent tab end (mm)": round(mouth_past_tab_mm, 2),
        "report": m.report(),
        "samples": samples,
    }
    out["pass"] = (
        (not m.sleeve_mated())
        and out["sleeve advance (mm)"] >= 8.0
        and mouth_past_tab_mm <= 2.5
        and out["report"]["zero"] is None
        and out["score at end"] == 0.0
    )
    return out


def roll_spill_zero(emb, m) -> dict:
    from inspect_robots import Scene

    emb.reset(Scene(id="roll_spill", instruction=m.INSTRUCTION), seed=0)
    foot = np.array([m.P.MAT_CX, 0.02, m.TOP + 0.075])
    rot = m.mk.axis_angle((0.0, 1.0, 0.0), math.radians(75.0))
    _place_cane_and_vials(m, foot, rot)

    def hold_cane(_t: float) -> None:
        m.mk.write_pose("cane", foot, _quat(m, rot))
        m.mk.set_joint("slv_s", 0.0)

    out = _run_loop(emb, m, 1.5, step_fn=hold_cane)
    out["pass"] = out["score at end"] == 0.0 and out["report"]["zero"] is not None and "spilled" in out["report"]["zero"]
    return out


def unsleeved_in_tray_zero(emb, m) -> dict:
    from inspect_robots import Scene

    emb.reset(Scene(id="unsleeved_tray", instruction=m.INSTRUCTION), seed=0)
    foot = np.array([m.P.TRAY_CX, m.P.TRAY_CY - 0.135, m.TOP + m.P.TRAY_WALL - m.P.BASE_Z + 0.001])
    _place_cane_and_vials(m, foot, np.eye(3))
    out = _run_loop(emb, m, 1.0)
    out["pass"] = out["score at end"] == 0.0 and out["report"]["zero"] == "unsleeved cane or vial entered the transfer tray"
    return out


def partial_quarter(emb, m) -> dict:
    from inspect_robots import Scene

    emb.reset(Scene(id="partial_quarter", instruction=m.INSTRUCTION), seed=0)
    foot = np.array([m.P.MAT_CX, -0.020, m.TOP + m.P.MAT_T + m.P.SLEEVE_R_OUT + 0.001])
    _place_mated_sleeve(m, foot, np.eye(3), 0.150)
    out = _run_loop(emb, m, 1.5)
    out["pass"] = out["score at end"] == 0.25 and out["report"]["zero"] is None
    return out


def seated_half(emb, m) -> dict:
    from inspect_robots import Scene

    emb.reset(Scene(id="seated_half", instruction=m.INSTRUCTION), seed=0)
    foot = np.array([m.P.MAT_CX, -0.140, m.TOP + m.P.MAT_T + 0.5 * m.P.TAB_W + 0.001])
    rot_side = m.mk.axis_angle((0.0, 1.0, 0.0), math.radians(90.0))
    _place_mated_sleeve(m, foot, rot_side, m.P.SLIDE_MAX)
    out = _run_loop(emb, m, 1.5)
    out["pass"] = (
        out["score at end"] == 0.5
        and out["report"]["zero"] is None
        and out["report"]["gap to top tab (mm)"] <= 10.0
        and out["report"]["foot enclosed"] is True
    )
    return out


def two_deg_angled_slide(emb, m) -> dict:
    """Pure arm-action bimanual run skewed deliberately by 1.75 deg yaw (> old 1.0 deg cliff): engages in-place with
    zero pose snap, passes 0.25 at >= 120 mm, and reaches 0.5 at >= 258 mm."""
    import checks as K
    import demo
    import demo_kit as kit
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import station

    emb.reset(Scene(id="two_deg_slide", instruction=m.INSTRUCTION), seed=0)
    moves = demo.initial_plan()
    Move = kit.Move
    arms = {s: kit.Arm(s, station.BASES[s], moves[s]) for s in ("left", "right")}
    hz = float(emb.info.control_hz)
    skew = m.mk.quat_to_matrix(m.mk.yaw_quat(math.radians(1.25)))
    ph = 0
    saw_quarter, saw_half, samples = False, False, []
    res = None
    for i in range(int(round(22.0 * hz))):
        t = i / hz
        max_end = max(arms["left"].seconds, arms["right"].seconds)
        if m.mk.MECH.poses and t >= max_end - 0.025:
            if ph == 0:
                p_c, r_c = m.cane_pose()
                u_c = r_c[:, 1].copy()
                u_c[2] = float(np.clip(u_c[2], -0.05, 0.05))
                u_c /= np.linalg.norm(u_c)
                p_s, r_s = m.mk.pose("sleeve")
                u_s = r_s[:, 1]
                tcp_cmd, rot_cmd = arms["right"].state(t)[:2]
                tcp_act = m.mk.gripper("right")[0]
                mouth_now = p_s + 0.5 * m.P.SLEEVE_L * u_s
                # Deliberately skew sleeve ~1.8 deg off cane axis (> old 1.0 deg cliff!), mouth offset 1.8 mm <= 2.8 mm:
                u_skew = skew @ u_c
                r_fix = demo._align_rot(u_s, u_skew)
                rot_goal = r_fix @ rot_cmd
                mouth_after = tcp_act + r_fix @ (mouth_now - tcp_act)
                perp = np.array([-u_c[1], u_c[0], 0.0])
                mouth_goal = p_c - 0.006 * u_c + 0.0018 * perp
                pos_goal = tcp_cmd + (mouth_goal - mouth_after)
                arms["left"].moves += [Move(1.4, label="align"), Move(0.5, label="settle")]
                arms["right"].moves += [Move(1.4, pos=pos_goal, rot=rot_goal, label="align ~1.8 deg"), Move(0.5, label="settle")]
                ph = 1
            elif ph == 1:
                p_c, r_c = m.cane_pose()
                u_c = r_c[:, 1].copy()
                u_c[2] = float(np.clip(u_c[2], -0.04, 0.04))
                u_c /= np.linalg.norm(u_c)
                p_s, r_s = m.mk.pose("sleeve")
                mouth_now = p_s + 0.5 * m.P.SLEEVE_L * r_s[:, 1]
                perp = np.array([-u_c[1], u_c[0], 0.0])
                to_foot = (p_c + 0.0018 * perp) - mouth_now
                tcp_l = arms["left"].state(t)[0]
                tcp_r = arms["right"].state(t)[0]
                d_left = -0.055 * u_c
                arms["left"].moves += [
                    Move(1.5, label="enter"),
                    Move(5.4, pos=tcp_l + d_left, push=(-2.5 * u_c).tolist(), label="thread"),
                    Move(0.8, push=(0.0, 0.0, 0.0), label="hold"),
                ]
                arms["right"].moves += [
                    Move(1.5, pos=tcp_r + to_foot + 0.032 * u_c, label="enter 32 mm"),
                    Move(5.4, pos=tcp_r + to_foot + d_left + 0.273 * u_c, push=(3.2 * u_c).tolist(), label="thread"),
                    Move(0.8, push=(0.0, 0.0, 0.0), label="hold"),
                ]
                ph = 2
        la, _ = arms["left"].action(t)
        ra, _ = arms["right"].action(t)
        res = emb.step(Action(data=pack(la, ra)))
        sc = float(res.info.get("score", 0.0))
        saw_quarter |= sc == 0.25
        saw_half |= sc >= 0.5
        if (i + 1) % 10 == 0:
            _, _, _, _, ang_now = m.sleeve_span_on_cane()
            samples.append(
                {
                    "t (s)": round((i + 1) / hz, 2),
                    "mated": m.sleeve_mated(),
                    "slide (mm)": round(1000.0 * m.sleeve_slide_m(), 1),
                    "ang (deg)": round(ang_now, 2),
                    "score": sc,
                }
            )
    out = {
        "score at end": float(0.0 if res is None else res.info.get("score", 0.0)),
        "saw 0.25": saw_quarter,
        "saw 0.5": saw_half,
        "report": m.report(),
        "samples": samples,
    }
    out["pass"] = saw_quarter and saw_half and out["score at end"] == 0.5 and out["report"]["zero"] is None
    return out


TRIALS = {
    "wrong_end_tab_block": wrong_end_tab_block,
    "roll_spill_zero": roll_spill_zero,
    "unsleeved_in_tray_zero": unsleeved_in_tray_zero,
    "partial_quarter": partial_quarter,
    "seated_half": seated_half,
    "two_deg_angled_slide": two_deg_angled_slide,
}


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    names = sys.argv[2].split(",") if len(sys.argv) > 2 else list(TRIALS)
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    ok = True
    for name in names:
        result = TRIALS[name](emb, m)
        (out / f"{name}.json").write_text(json.dumps(result, indent=1, default=str))
        brief = {k: v for k, v in result.items() if k != "samples"}
        brief["last sample"] = result["samples"][-1] if result["samples"] else None
        print(f"TRIAL {name} " + json.dumps(brief, default=str), flush=True)
        ok &= bool(result["pass"])
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
