#!/usr/bin/env python3
"""PhysX trials of the specimen-pan task's rules, in one simulator session:

    scripts/rta simpy <id> mech/tasks/specimen_hat/trials.py /workspace/robocurve/outputs/<id> [names]

- ``seat_let_go``: the left hand raises the seat to 42 deg as in the demo, opens and draws back: the seat falls shut
  and the let-go rule zeroes the trial (score 0).
- ``lengthwise_falls_through``: with the seat held up as in the demo, the pan is put (``write_pose``, trials only)
  lengthwise into the middle of the opening, its bottom 50 mm under the rim: nothing holds it and it falls to the
  bench (score 0, the fall-through rule).
- ``seat_left_up``: the demo with the seat held up to the end: the pan across the opening, the seat not lowered
  (score 0.5).
- ``pan_on_the_closed_seat``: the pan put across the closed seat, its wings on the ring (``write_pose``): it rests
  there, wings 28.6 mm over the rim (score 0).
- ``seat_past_upright``: the seat put at 98 deg (``set_joint``), past its balance: it leans back into the left arm at
  rest, is pushed forward and falls shut (let-go zero).
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as K  # noqa: E402
import demo as D  # noqa: E402


def _left_until_held() -> list:
    return D.plan()["left"][:5]


def seat_let_go(emb, m, out: Path) -> dict:
    import demo_kit as kit

    left = _left_until_held() + [
        kit.Move(0.5, grip=D.SEAT_OPEN, push=(0.0, 0.0, 0.0), label="let go"),
        kit.Move(0.6, *K.left_on_seat(K.SEAT_HOLD_DEG, up=0.060), label="draw back"),
        kit.Move(1.6, *D.ready("left"), label="ready"),
    ]
    res = kit.run(emb, {"left": left, "right": []}, out / "seat_let_go", seed=0, tail_s=1.0, record=False)
    rep = res["report"]
    return {"pass": rep["zero"] == "the seat was let go above 20 degrees" and res["info"].get("score") == 0.0, "report": rep}


def lengthwise_falls_through(emb, m, out: Path) -> dict:
    import demo_kit as kit

    put = {}

    def director(t, arms, pictures) -> None:
        if t >= 6.6 and not put:  # the seat is up at 42 deg from 6.0 s
            P = m.P
            pos = (P.HOLE_X, P.PINTLE_Y - (P.HOLE_D0 + P.HOLE_L / 2.0), m.TOP + P.RIM_Z - 0.050)
            m.mk.write_pose("specimen_pan", pos, tuple(float(v) for v in m.mk.matrix_to_quat(P.rz(0.0))))
            put["t"] = round(t, 2)

    left = _left_until_held() + [kit.Move(3.0, *K.left_on_seat(K.SEAT_HOLD_DEG), push=D.hold(K.SEAT_HOLD_DEG), label="hold up")]
    res = kit.run(emb, {"left": left, "right": []}, out / "lengthwise_falls_through", seed=0, director=director, tail_s=0.5, record=False)
    rep = res["report"]
    ok = rep["zero"] == "the pan fell through the opening" and res["info"].get("score") == 0.0 and rep["pan_lowest_mm"] < 8.0
    return {"pass": ok, "put_at_s": put.get("t"), "report": rep}


def seat_left_up(emb, m, out: Path) -> dict:
    import demo_kit as kit

    full = D.plan()
    held = [mv for mv in full["left"] if mv.label == "hold up"][0]
    left = full["left"][: full["left"].index(held) + 1] + [kit.Move(4.0, *K.left_on_seat(K.SEAT_HOLD_DEG), push=D.hold(K.SEAT_HOLD_DEG), label="keep it up")]
    res = kit.run(emb, {"left": left, "right": full["right"]}, out / "seat_left_up", seed=0, tail_s=0.5, record=False)
    rep = res["report"]
    return {"pass": res["info"].get("score") == 0.5 and rep["zero"] is None and rep["pan_bridged"], "report": rep}


def pan_on_the_closed_seat(emb, m, out: Path) -> dict:
    import demo_kit as kit

    put = {}

    def director(t, arms, pictures) -> None:
        if t >= 0.5 and not put:
            P = m.P
            pos = (P.HOLE_X, P.PINTLE_Y - P.PAN_PLACE_D, m.TOP + P.RIM_Z + P.SEAT_T_BUMPED - (P.PAN_H - P.WING_T) + 0.002)
            m.mk.write_pose("specimen_pan", pos, tuple(float(v) for v in m.mk.matrix_to_quat(P.rz(90.0))))
            put["t"] = round(t, 2)

    res = kit.run(emb, {"left": [kit.Move(3.0, label="still")], "right": []}, out / "pan_on_the_closed_seat", seed=0, director=director, tail_s=0.5, record=False)
    rep = res["report"]
    ok = res["info"].get("score") == 0.0 and not rep["pan_bridged"] and min(rep["wing_tips_over_rim_mm"]) > 20.0
    return {"pass": ok, "report": rep}


def seat_past_upright(emb, m, out: Path) -> dict:
    """The seat put (``set_joint``, trials only) at 98 deg, past its balance at 94 deg, both arms at rest: it leans back
    into the left arm (its rest pose reaches the seat from 94 deg), is pushed forward and falls shut (let-go zero)."""
    import math

    import demo_kit as kit

    seen = {"put": None, "arm": False}

    def director(t, arms, pictures) -> None:
        if t >= 0.5 and seen["put"] is None:
            m.mk.set_joint("seat_hinge", math.radians(98.0))
            seen["put"] = round(t, 2)
        elif seen["put"] is not None and m.mk.touching("toilet_seat", "left robot", within=0.002):
            seen["arm"] = True

    res = kit.run(emb, {"left": [kit.Move(3.0, label="still")], "right": []}, out / "seat_past_upright", seed=0, director=director, tail_s=0.5, record=False)
    rep = res["report"]
    ok = seen["arm"] and rep["zero"] == "the seat was let go above 20 degrees" and rep["seat_deg"] < 1.0
    return {"pass": ok, "touched_left_arm": seen["arm"], "report": rep}


TRIALS = {f.__name__: f for f in (seat_let_go, lengthwise_falls_through, seat_left_up, pan_on_the_closed_seat, seat_past_upright)}


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
