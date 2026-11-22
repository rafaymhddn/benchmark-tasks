#!/usr/bin/env python3
"""PhysX mechanism and scoring-level trials for the CryoCane sleeving task (poses are written only here).

The sleeve is a free body in true contact with the cane and vials (no joint): ``aligned_two_hand_push`` and
``cocked_push_binds`` drive it with both arms, the second with the sleeve turned 12 deg off the cane's axis.

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
    for idx, name in enumerate(m.VIALS):
        bay = m.P.VIAL_BAYS[idx]
        vp = p_c + r_c @ np.array([0.0, m.P.bay_centre_y(bay), m.P.VIAL_REST_Z + 0.0001])  # top 0.05 mm under the bore
        m.mk.write_pose(name, vp, _quat(m, r_c))


def _place_threaded_sleeve(m, foot_pos, rot, slide_m: float) -> None:
    """Cane and vials at ``foot_pos``/``rot``; the sleeve on the stack's middle line with its mouth ``slide_m`` onto it."""
    _place_cane_and_vials(m, foot_pos, rot)
    p_c = np.asarray(foot_pos, dtype=float)
    r_c = np.asarray(rot, dtype=float)
    axis = p_c + m.P.BORE_Z * r_c[:, 2]
    m.mk.write_pose("sleeve", axis + (slide_m - 0.5 * m.P.SLEEVE_L) * r_c[:, 1], _quat(m, r_c))


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
                    "on cane": m.coaxial_enclosed(),
                    "slide (mm)": round(1000.0 * m.sleeve_slide_m(), 1),
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
                    "on cane": m.coaxial_enclosed(),
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
        (not m.coaxial_enclosed())
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
    foot = np.array([m.P.MAT_CX, -0.020, m.TOP + m.P.MAT_T + m.P.SLEEVE_R_OUT - m.P.BORE_Z + 0.001])  # sleeve 1 mm over the mat
    _place_threaded_sleeve(m, foot, np.eye(3), 0.150)
    out = _run_loop(emb, m, 1.5)
    out["pass"] = out["score at end"] == 0.25 and out["report"]["zero"] is None
    return out


def seated_half(emb, m) -> dict:
    from inspect_robots import Scene

    emb.reset(Scene(id="seated_half", instruction=m.INSTRUCTION), seed=0)
    foot = np.array([m.P.MAT_CX, -0.140, m.TOP + m.P.MAT_T + 0.5 * m.P.TAB_W + 0.001])
    rot_side = m.mk.axis_angle((0.0, 1.0, 0.0), math.radians(90.0))
    _place_threaded_sleeve(m, foot, rot_side, m.P.SLIDE_MAX)
    out = _run_loop(emb, m, 1.5)
    out["pass"] = (
        out["score at end"] == 0.5
        and out["report"]["zero"] is None
        and out["report"]["gap to top tab (mm)"] <= 10.0
        and out["report"]["foot enclosed"] is True
    )
    return out


def _two_hand_push(emb, m, name: str, cock_deg: float, seconds: float, tries: int | None = None) -> dict:
    """The demo's own plan (cane lifted and turned by the left hand, sleeve by the right), then the closed-loop two-hand
    slide with the sleeve's axis turned ``cock_deg`` about the vertical away from the cane's (demo.insert_moves)."""
    import demo
    import demo_kit as kit
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import station

    emb.reset(Scene(id=name, instruction=m.INSTRUCTION), seed=0)
    arms = {s: kit.Arm(s, station.BASES[s], mv) for s, mv in demo.initial_plan().items()}
    plan = demo.Inserter(m, cock_deg=cock_deg, release=False)
    if tries is not None:
        plan.TRIES = tries
    hz = float(emb.info.control_hz)
    samples, res, best, furthest, worst_breach, peak_contact = [], None, 0.0, -1.0, 0.0, 0.0
    for i in range(int(round(seconds * hz))):
        t = i / hz
        plan.direct(t, arms)
        try:
            la, _ = arms["left"].action(t)
            ra, _ = arms["right"].action(t)
        except ValueError as err:  # the hand cannot push further without passing a joint limit: the trial ends here
            plan.log.append({"t": round(t, 2), "stage": "arm limit", "error": str(err)[:160]})
            break
        res = emb.step(Action(data=pack(la, ra)))
        on, _, y_hi, rad, ang = m.sleeve_span_on_cane()
        best = max(best, y_hi if on else 0.0)
        breach = m.wall_breach_mm()
        worst_breach = max(worst_breach, breach)
        touch = [c for c in m.mk.contacts("sleeve") if c.other in ("cane", "vial_a", "vial_b")]
        contact_n = float(sum(c.force for c in touch))
        peak_contact = max(peak_contact, contact_n)
        if plan.stage in ("thread", "seated", "bound"):
            furthest = max(furthest, plan.mouth()[0])
        if (i + 1) % 10 == 0:
            samples.append({"t (s)": round((i + 1) / hz, 2), "stage": plan.stage, "on cane": on, "cane pitch (deg)": round(m.cane_pitch_deg(), 1),
                            "mouth (mm)": round(1000.0 * y_hi, 1), "radial (mm)": round(1000.0 * rad, 2),
                            "angle (deg)": round(ang, 2), "push (N)": plan.push_n, "contact (N)": round(contact_n, 1),
                            "wall breach (mm)": round(breach, 2), "score": float(res.info.get("score", 0.0))})
    return {"score at end": float(0.0 if res is None else res.info.get("score", 0.0)),
            "furthest mouth on the cane (mm)": round(1000.0 * best, 1), "furthest mouth station (mm)": round(1000.0 * furthest, 1),
            "worst wall breach (mm)": round(worst_breach, 2), "peak sleeve contact (N)": round(peak_contact, 1),
            "log": plan.log, "report": m.report(), "samples": samples}


def aligned_two_hand_push(emb, m) -> dict:
    out = _two_hand_push(emb, m, "aligned_push", 0.0, 40.0)
    out["pass"] = (out["score at end"] == 0.5 and out["report"]["zero"] is None and out["report"]["gap to top tab (mm)"] <= 10.0
                   and out["worst wall breach (mm)"] < 0.5)
    return out


def cocked_push_binds(emb, m) -> dict:
    out = _two_hand_push(emb, m, "cocked_push", 12.0, 34.0, tries=0)
    out["pass"] = (out["score at end"] == 0.0 and out["report"]["zero"] is None and out["furthest mouth on the cane (mm)"] < 120.0
                   and out["worst wall breach (mm)"] < 0.5)
    return out


def slightly_cocked_push(emb, m) -> dict:
    out = _two_hand_push(emb, m, "slightly_cocked_push", 3.0, 34.0, tries=0)
    out["pass"] = out["worst wall breach (mm)"] < 0.5 and out["report"]["zero"] is None
    return out


TRIALS = {
    "wrong_end_tab_block": wrong_end_tab_block,
    "roll_spill_zero": roll_spill_zero,
    "unsleeved_in_tray_zero": unsleeved_in_tray_zero,
    "partial_quarter": partial_quarter,
    "seated_half": seated_half,
    "aligned_two_hand_push": aligned_two_hand_push,
    "cocked_push_binds": cocked_push_binds,
    "slightly_cocked_push": slightly_cocked_push,
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
