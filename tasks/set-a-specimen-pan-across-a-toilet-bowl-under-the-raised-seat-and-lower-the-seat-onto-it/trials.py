#!/usr/bin/env python3
"""PhysX trials of the specimen-pan task's rules, in one simulator session:

    scripts/rta simpy <id> mech/tasks/specimen_hat/trials.py /workspace/robocurve/outputs/<id> [names]

- ``lengthwise_falls_in``: the pan put (``write_pose``, trials only) lengthwise into the middle of the opening, its
  bottom 50 mm under the rim: nothing holds it and it falls to the water surface (score 0, the fall-in rule).
- ``seat_knocked_off``: the seat put (``set_joint``, trials only) at 92 deg, 2 deg short of its balance: it falls shut
  onto the rim (score 0, the seat-fall rule).
- ``seat_short_of_stop``: the seat put at 97 deg, past its balance: it goes back onto its stop by itself (no zero).
- ``pan_still_held``: the demo until the wings rest on the rim, then the hand keeps its pinch (score 0.5).
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import demo as D  # noqa: E402

STILL = {"right": [], "left": []}


def _still(emb, out: Path, name: str, director, seconds: float = 3.0):
    import demo_kit as kit

    return kit.run(emb, {"left": [kit.Move(seconds, label="still")], "right": []}, out / name, seed=0, director=director, tail_s=0.5, record=False)


def lengthwise_falls_in(emb, m, out: Path) -> dict:
    put = {}

    def director(t, arms, pictures) -> None:
        if t >= 0.5 and not put:
            P = m.P
            pos = (P.BOLT_X - (P.HOLE_D0 + P.HOLE_D1) / 2.0, P.BOWL_Y, m.TOP + P.RIM_Z - 0.050)
            m.mk.write_pose("specimen_pan", pos, tuple(float(v) for v in m.mk.matrix_to_quat(P.rz(-90.0))))
            put["t"] = round(t, 2)

    res = _still(emb, out, "lengthwise_falls_in", director)
    rep = res["report"]
    ok = rep["zero"] == "the pan fell into the bowl" and res["info"].get("score") == 0.0 and rep["pan_lowest_over_water_mm"] < 8.0
    return {"pass": ok, "put_at_s": put.get("t"), "report": rep}


def _seat_put(emb, m, out: Path, name: str, deg: float) -> dict:
    seen = {"put": None}

    def director(t, arms, pictures) -> None:
        if t >= 0.5 and seen["put"] is None:
            m.mk.set_joint("seat_hinge", math.radians(deg - m.P.CHECK_HINGE_DEG))
            seen["put"] = round(t, 2)

    res = _still(emb, out, name, director)
    return {"put_at_s": seen["put"], "score": res["info"].get("score"), "report": res["report"]}


def seat_knocked_off(emb, m, out: Path) -> dict:
    res = _seat_put(emb, m, out, "seat_knocked_off", 92.0)
    rep = res["report"]
    res["pass"] = rep["zero"] == "the seat fell from its stop" and rep["seat_deg"] < 5.0 and res["score"] == 0.0
    return res


def seat_short_of_stop(emb, m, out: Path) -> dict:
    res = _seat_put(emb, m, out, "seat_short_of_stop", 97.0)
    rep = res["report"]
    res["pass"] = rep["zero"] is None and rep["seat_up"] and rep["min_seat_deg"] < 98.0
    return res


def pan_still_held(emb, m, out: Path) -> dict:
    import demo_kit as kit

    right = D.plan()["right"]
    k = [mv.label for mv in right].index("wings on the rim")
    right = right[: k + 1] + [kit.Move(4.0, right[k].pos, right[k].rot, label="keep the pinch")]
    res = kit.run(emb, {"left": [], "right": right}, out / "pan_still_held", seed=0, tail_s=0.5, record=False)
    rep = res["report"]
    return {"pass": res["info"].get("score") == 0.5 and rep["zero"] is None and rep["pan_bridged"], "report": rep}


TRIALS = {f.__name__: f for f in (lengthwise_falls_in, seat_knocked_off, seat_short_of_stop, pan_still_held)}


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    names = sys.argv[2].split(",") if len(sys.argv) > 2 else list(TRIALS)
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    summary, ok = {}, True
    for name in names:
        res = TRIALS[name](emb, m, out)
        summary[name] = res
        ok &= bool(res["pass"])
        print(f"TRIAL {name} " + json.dumps(res, default=str), flush=True)
    (out / "trials.json").write_text(json.dumps(summary, indent=1, default=str))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
