#!/usr/bin/env python3
"""PhysX trials verifying each zero rule, partial tier (0.25, 0.5), and full tier (1.0) of ``lab_jack_probe`` (Gate G2).

    scripts/rta simpy <id> mech/tasks/lab_jack_probe/trials.py /workspace/robocurve/outputs/<id>
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

SAMPLE_S = 0.25


def _ready_action():
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))


def _set_jack(m, z_deck_m: float) -> None:
    p = m.P
    dz = z_deck_m - p.Z_DECK_START
    theta = dz / p.LEAD_M_PER_RAD
    m.mk.set_joint("deck_z", dz)
    m.mk.set_joint("carrier_z", 0.5 * dz)
    m.mk.set_joint("wheel", theta)
    m.mk.set_spring("deck_z", rest=dz, most=p.DECK_MAX_N)
    m.mk.set_spring("carrier_z", rest=0.5 * dz, most=p.DECK_MAX_N)
    m.STATE["wheel_hold"] = theta


def _place_sample_on_mat(m) -> None:
    p = m.P
    m.mk.write_pose("beaker_sample", (p.MAT_XY[0], p.MAT_XY[1], m.TOP + p.MAT_SIZE[2] + p.OVER), (0.0, 0.0, 0.0, 1.0))


def _place_rinse_on_deck(m, z_deck_m: float) -> None:
    p = m.P
    m.mk.write_pose("beaker_rinse", (p.JACK_XY[0], p.JACK_XY[1], m.TOP + z_deck_m + p.OVER), (0.0, 0.0, 0.0, 1.0))


def _run_episode(embodiment, m, seconds: float, setup_fn, step_fn=None) -> dict:
    from inspect_robots import Scene

    embodiment.reset(Scene(id="trial", instruction=m.INSTRUCTION), seed=0)
    setup_fn(m)
    hz = float(embodiment.info.control_hz)
    every = max(1, int(round(SAMPLE_S * hz)))
    samples = []
    res = None
    act = _ready_action()
    for i in range(int(round(seconds * hz))):
        if step_fn is not None:
            step_fn(m, i / hz)
        res = embodiment.step(act)
        t = round((i + 1) / hz, 3)
        if (i + 1) % every == 0:
            samples.append(
                {
                    "t_s": t,
                    "deck_mm": round(1000.0 * m.deck_height_m(), 2),
                    "wheel_turns": round(m.mk.joint("wheel") / (2.0 * math.pi), 3),
                    "probe_lat_mm": round(1000.0 * m.probe_lateral_m(), 2),
                    "probe_ax_mm": round(1000.0 * m.probe_axial_lift_m(), 2),
                    "probe_force_n": round(m.probe_contact_n(), 2),
                    "tau_load_nm": round(m.STATE.get("last_coupling", {}).get("tau_load_nm", 0.0), 5),
                    "score": float(res.info.get("score", 0.0)),
                }
            )
    return {
        "end_score": float(res.info.get("score", 0.0)) if res is not None else 0.0,
        "report": m.report(),
        "samples": samples,
    }


def lateral_strike_zero(embodiment, m) -> dict:
    """Pulling beaker_sample sideways off the 86 mm deck without lowering strikes the submerged probe (> 4 mm)."""
    def step_pull(mm, t_s: float) -> None:
        if t_s < 0.8:
            mm.mk.push("beaker_sample", force=(0.0, 8.0, 0.0))
        else:
            mm.mk.push("beaker_sample")

    out = _run_episode(embodiment, m, 1.5, lambda _mm: None, step_pull)
    rep = out["report"]
    out["pass"] = out["end_score"] == 0.0 and rep["max_lateral_mm"] > 4.0 and rep["zero"] is not None
    return out


def bottom_out_zero(embodiment, m) -> dict:
    """Cranking the wheel CW past the 20-40 mm band drives beaker_rinse's floor into the probe tip (>= 1.2 mm / >= 3 N)."""
    def setup(mm):
        _set_jack(mm, 0.062)
        _place_sample_on_mat(mm)

    def step_drive(mm, t_s: float) -> None:
        if abs(t_s - 0.15) < 0.02:
            # Start from z_deck = 120 mm (8 mm below floor touch) and drive wheel CW into bottom-out
            _set_jack(mm, 0.120)
            _place_rinse_on_deck(mm, 0.120)
        elif t_s >= 0.25:
            theta = mm.mk.joint("wheel") + 0.18
            mm.mk.set_joint("wheel", theta)

    out = _run_episode(embodiment, m, 2.8, setup, step_drive)
    rep = out["report"]
    min_tau_load = min(s["tau_load_nm"] for s in out["samples"])
    out["peak_feedback_torque_nm"] = round(abs(min_tau_load), 4)
    out["pass"] = (
        out["end_score"] == 0.0
        and (rep["max_axial_mm"] >= 1.2 or rep["max_probe_force_n"] >= 3.0)
        and rep["zero"] == "a beaker floor touched and bottomed against the pH probe tip"
        and abs(min_tau_load) > 0.015
    )
    return out


def sample_cleared_quarter(embodiment, m) -> dict:
    """Deck lowered to 62 mm and beaker_sample resting upright on blue_mat -> 0.25."""
    def setup(mm):
        _set_jack(mm, 0.062)
        _place_sample_on_mat(mm)

    out = _run_episode(embodiment, m, 2.0, setup)
    out["pass"] = out["end_score"] == 0.25 and out["report"]["zero"] is None
    return out


def rinse_placed_half(embodiment, m) -> dict:
    """Deck at 62 mm, sample on blue_mat, rinse coaxial on deck before raising -> 0.5."""
    def setup(mm):
        _set_jack(mm, 0.062)
        _place_sample_on_mat(mm)
        _place_rinse_on_deck(mm, 0.062)

    out = _run_episode(embodiment, m, 2.0, setup)
    out["pass"] = out["end_score"] == 0.5 and out["report"]["zero"] is None
    return out


def raised_in_band_full(embodiment, m) -> dict:
    """Deck lowered then raised to 98 mm (28 mm immersion inside 20-40 mm green band) -> 1.0."""
    def setup(mm):
        _set_jack(mm, 0.062)
        _place_sample_on_mat(mm)

    def step_raise(mm, t_s: float) -> None:
        if abs(t_s - 0.15) < 0.02:
            _set_jack(mm, 0.098)
            _place_rinse_on_deck(mm, 0.098)

    out = _run_episode(embodiment, m, 2.2, setup, step_raise)
    out["pass"] = out["end_score"] == 1.0 and out["report"]["zero"] is None
    return out


TRIALS = {
    "lateral_strike_zero": lateral_strike_zero,
    "bottom_out_zero": bottom_out_zero,
    "sample_cleared_quarter": sample_cleared_quarter,
    "rinse_placed_half": rinse_placed_half,
    "raised_in_band_full": raised_in_band_full,
}


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    names = sys.argv[2].split(",") if len(sys.argv) > 2 else list(TRIALS)
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    ok = True
    for name in names:
        res = TRIALS[name](embodiment, m)
        (out / f"{name}.json").write_text(json.dumps(res, indent=1, default=str))
        brief = {k: v for k, v in res.items() if k != "samples"}
        brief["last_sample"] = res["samples"][-1] if res["samples"] else None
        print(f"TRIAL {name} " + json.dumps(brief, default=str), flush=True)
        ok &= bool(res["pass"])
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
