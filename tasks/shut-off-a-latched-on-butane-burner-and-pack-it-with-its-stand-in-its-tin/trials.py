#!/usr/bin/env python3
"""PhysX trials for ``burner_stow`` (both arms held at ``READY``): verifies each score level, safety zero rule,
spring-driven shutoff reset, and physical lid interference against an unremoved 60 mm stand.

    scripts/rta simpy <id> mech/tasks/burner_stow/trials.py /workspace/robocurve/outputs/<id>
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

SAMPLE_S = 0.25


def _hold():
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))


def _quat(m, rot: np.ndarray) -> tuple[float, ...]:
    return tuple(float(v) for v in m.mk.matrix_to_quat(rot))


def place_burner_assembly(m, origin: np.ndarray, rot: np.ndarray, *, lock_m: float = 0.0, trig_out_m: float = 0.0, child_m: float = 0.0) -> None:
    """Place ``burner`` and its three slide-mounted controls consistently at ``(origin, rot)``."""
    q = _quat(m, rot)
    m.mk.write_pose("burner", origin, q)
    m.mk.write_pose("trigger", origin + rot @ (np.array(P.TRIG_POS_B) + np.array([-trig_out_m, 0.0, 0.0])), q)
    m.mk.write_pose("lock_slider", origin + rot @ (np.array(P.LOCK_POS_B) + np.array([lock_m, 0.0, 0.0])), q)
    m.mk.write_pose("child_stop", origin + rot @ (np.array(P.CHILD_POS_B) + np.array([0.0, -child_m, 0.0])), q)
    if lock_m >= P.LOCK_RELEASE_AT:
        m.STATE["unlatched"] = True
        m.mk.set_spring("lock_slider", rest=P.LOCK_STROKE, most=P.LOCK_FREE_N)
        m.mk.set_limits("trigger", lower=-P.TRIG_OVERTRAVEL, upper=P.TRIG_LAT)
        m.mk.set_limits("child_stop", lower=0.0, upper=P.CHILD_STROKE)


def run_trial(embodiment, m, seconds: float, setup) -> dict:
    from inspect_robots import Scene

    embodiment.reset(Scene(id="trial", instruction=m.INSTRUCTION), seed=0)
    setup(m)
    hz = float(embodiment.info.control_hz)
    every = max(1, int(round(SAMPLE_S * hz)))
    samples = []
    res = None
    for i in range(int(round(seconds * hz))):
        res = embodiment.step(_hold())
        t = round((i + 1) / hz, 3)
        if (i + 1) % every == 0:
            samples.append({
                "t_s": t,
                "score": float(res.info.get("score", 0.0)),
                "burner_off": m.burner_off(),
                "detached": m.detached(),
                "lid_deg": round(m.lid_deg(), 2),
                "seam_gap_mm": round(m.seam_gap_mm(), 2),
                "zero": m.STATE.get("zero"),
            })
    assert res is not None
    return {
        "score_at_end": float(res.info.get("score", 0.0)),
        "report": m.report(),
        "samples": samples,
    }


def stow_while_on(embodiment, m) -> dict:
    """Safety zero rule: placing the burner in the tin while its lock button still holds the valve open -> latched 0."""

    def setup(mm) -> None:
        mm.mk.cut("w_stand")
        tx, ty = P.TIN_XY
        origin = np.array([tx + 0.122, ty + 0.022, mm.TOP + P.OVER + P.TIN_FLOOR + 0.023])
        place_burner_assembly(mm, origin, K.RB_FLAT, lock_m=0.0)

    out = run_trial(embodiment, m, 1.5, setup)
    out["pass"] = (
        out["score_at_end"] == 0.0
        and out["report"]["zero"] == "burner entered the tin while its gas valve was still latched open"
    )
    return out


def tin_fell_off_bench(embodiment, m) -> dict:
    """Fall zero rule: tin_base off the bench edge (or > 10 mm below bench top) -> latched 0."""

    def setup(mm) -> None:
        b_pos, b_rot = mm.mk.built_pose("tin_base")
        l_pos, l_rot = mm.mk.built_pose("tin_lid")
        drop = np.array([0.0, -0.68 - b_pos[1], -0.025])
        mm.mk.write_pose("tin_base", b_pos + drop, _quat(mm, b_rot))
        mm.mk.write_pose("tin_lid", l_pos + drop, _quat(mm, l_rot))

    out = run_trial(embodiment, m, 1.0, setup)
    out["pass"] = (
        out["score_at_end"] == 0.0
        and out["report"]["zero"] == "tin_base fell off the bench"
    )
    return out


def shutoff_attached_props_lid(embodiment, m) -> dict:
    """0.25 + construct-validity check: lock button pushed forward (>= 4.5 mm) springs trigger & child-stop out in PhysX,
    but laying burner + attached 60 mm stand in the 52 mm tin physically blocks tin_lid open (> 5 deg, score 0.25)."""

    def setup(mm) -> None:
        tx, ty = P.TIN_XY
        # Place attached burner + heel_carrier + 60 mm stand sideways on the tin floor (stand radius 30 mm).
        origin = np.array([tx + 0.118, ty + 0.020, mm.TOP + P.OVER + P.TIN_FLOOR + P.STAND_R + 0.001])
        q = _quat(mm, K.RB_FLAT)
        mm.mk.write_pose("stand", origin, q)
        mm.mk.write_pose("heel_carrier", origin, q)
        # Only move lock_slider to 5.0 mm; let PhysX springs drive trigger and child_stop to their reset stops!
        place_burner_assembly(mm, origin, K.RB_FLAT, lock_m=0.0050, trig_out_m=0.0, child_m=0.0)
        # Release lid from 35 deg above the attached assembly so gravity drops it onto the 60 mm stand rim.
        mm.mk.set_joint("lid_hinge", math.radians(P.LID_OPEN_DEG - 35.0))

    out = run_trial(embodiment, m, 2.5, setup)
    rep = out["report"]
    out["pass"] = (
        out["score_at_end"] == 0.25
        and rep["burner_off"] is True
        and rep["detached"] is False
        and rep["lid_deg"] > 5.0
        and rep["seam_gap_mm"] > 15.0
    )
    return out


def detached_in_open_tin(embodiment, m) -> dict:
    """0.5 level (even with tin_base nudged 55 mm on the bench): burner shut off, stand pulled off, both flat in tin_base, lid open."""

    def setup(mm) -> None:
        mm.mk.cut("w_stand")
        nudge = np.array([0.055, 0.0, 0.0])
        b_pos, b_rot = mm.mk.built_pose("tin_base")
        l_pos, l_rot = mm.mk.built_pose("tin_lid")
        mm.mk.write_pose("tin_base", b_pos + nudge, _quat(mm, b_rot))
        mm.mk.write_pose("tin_lid", l_pos + nudge, _quat(mm, l_rot))
        tx, ty = P.TIN_XY[0] + float(nudge[0]), P.TIN_XY[1]
        mm.mk.write_pose("stand", (tx - 0.095, ty, mm.TOP + P.OVER + P.TIN_FLOOR + P.STAND_FLOOR_H + 0.001), (0.0, 0.0, 0.0, 1.0))
        b_origin = np.array([tx + 0.122, ty + 0.022, mm.TOP + P.OVER + P.TIN_FLOOR + 0.023])
        place_burner_assembly(mm, b_origin, K.RB_FLAT, lock_m=P.LOCK_STROKE, trig_out_m=P.TRIG_LAT, child_m=P.CHILD_STROKE)

    out = run_trial(embodiment, m, 2.0, setup)
    rep = out["report"]
    out["pass"] = (
        out["score_at_end"] == 0.5
        and rep["zero"] is None
        and rep["burner_off"] is True
        and rep["detached"] is True
        and rep["burner_in_tin_flat"] is True
        and rep["stand_in_tin_flat"] is True
        and rep["lid_deg"] > 90.0
    )
    return out


TRIALS = {
    "stow_while_on": stow_while_on,
    "tin_fell_off_bench": tin_fell_off_bench,
    "shutoff_attached_props_lid": shutoff_attached_props_lid,
    "detached_in_open_tin": detached_in_open_tin,
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
        result = TRIALS[name](embodiment, m)
        (out / f"{name}.json").write_text(json.dumps(result, indent=1, default=str))
        brief = {k: v for k, v in result.items() if k != "samples"}
        print(f"TRIAL {name} " + json.dumps(brief, default=str), flush=True)
        ok &= bool(result["pass"])
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
