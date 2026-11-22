#!/usr/bin/env python3
"""Simulator trials for ``comb_busbar`` (no video, ``record=False``). The 0.5 levels are ``demo.py --check --snug``
(each screw closed on its tooth, no click) and ``--bare`` (no cap); 1.0 is ``demo.py --check``.

(a) ``zero: off the bench``: no hand; the driver written 1.2 m out, past the bench's far edge, where it falls
    (whole-trial zero).
(b) ``blocked``: no hand; breaker 1's screw turned in until its empty clamp is shut (0.2 mm), then the comb let go
    with its three teeth 14 mm down the slots, 1.4 mm over that shut plate: two teeth slide on down, the third stands
    on the plate and the comb is not seated (0.0).

    python trials.py <output folder> [a|b ...]     # in the simulator container; letters: only those trials
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import demo_kit as kit  # noqa: E402
from demo import task_report  # noqa: E402
import parts as P  # noqa: E402

LET_GO_OVER = 0.012  # m: (b) the comb's bar this far over its seat: the tips 14 mm down the slots, 1.4 mm over the shut plate
assert P.MCB_TOP - P.PIN_L + LET_GO_OVER > P.CLAMP_Z[1] - 0.0004 + 0.001  # the tip clear of the shut plate's top edge


def main() -> int:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "trials_out"
    out.mkdir(parents=True, exist_ok=True)
    only = set(sys.argv[2:]) or {"a", "b"}
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    mk = m.mk
    results: dict = {}
    case = {"what": None, "step": 0}

    def director(t, _arms, _pictures) -> None:
        if not mk.MECH.poses:
            return
        if case["what"] == "off_bench" and case["step"] == 0 and t >= 0.5:  # past the bench's far edge, 20 mm up: it falls
            mk.write_pose("driver", (1.2, 0.0, m.TOP + 0.02), (0.0, 0.0, 0.0, 1.0))
            case["step"] = 1
        if case["what"] == "blocked":
            if case["step"] == 0 and t >= 0.2:  # breaker 1's screw turned in until its empty plate is 0.2 mm off the wall
                m.STATE["turn"][0] = P.touch_turn(False) - math.radians(1.0)
                case["step"] = 1
            elif case["step"] == 1 and t >= 0.6:  # the comb let go 12 mm over its seat, teeth 1-3 in breakers 3-1's slots
                p = P.pw(P.comb_seated_panel(0) + np.array([0.0, 0.0, LET_GO_OVER])) + np.array([0.0, 0.0, m.TOP])
                mk.write_pose("comb", tuple(p), P.TILT_QUAT)
                case["step"] = 2

    def depth_mm() -> float:
        """How far the comb's bar stands over its seat (along the breakers), mm."""
        cp = mk.pose("comb")[0] - np.array([0.0, 0.0, m.TOP])
        return round(1e3 * float(P.to_panel(cp)[2] - P.comb_seated_panel(0)[2]), 2)

    for what in [w for w, letter in (("off_bench", "a"), ("blocked", "b")) if letter in only]:
        case.update(what=what, step=0)
        res = kit.run(emb, {}, out / what, seed=0, director=director, tail_s=4.0, limit_s=4.0, record=False,
                      extra=lambda: {"bar_over_seat_mm": depth_mm(), "gap_mm": [round(1e3 * g, 3) for g in m.STATE["gap"]]})  # fmt: skip
        rep = task_report(m)
        results[what] = {"score": res["info"].get("score"), "fault": rep["fault"], "seated": rep.get("seated"),
                         "bar_over_seat_mm": res["rows"][-1]["extra"]["bar_over_seat_mm"], "gap_mm": rep["gap_mm"]}  # fmt: skip
    (out / "trials_report.json").write_text(json.dumps(results, indent=1, default=str))
    print("TRIALS_REPORT " + json.dumps(results, default=str), flush=True)
    checks = {}
    if "a" in only:
        checks["a_zero_off_bench"] = results["off_bench"]["score"] == 0.0 and "driver left the bench" in str(results["off_bench"]["fault"])
    if "b" in only:
        b = results["blocked"]
        checks["b_blocked_not_seated"] = b["score"] == 0.0 and b["seated"] is None and b["bar_over_seat_mm"] > 5.0
    print("TRIALS_CHECKS " + json.dumps(checks), flush=True)
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
