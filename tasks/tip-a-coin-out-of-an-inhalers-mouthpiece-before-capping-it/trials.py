#!/usr/bin/env python3
"""The off-bench rule of the inhaler task in the simulator, no arm (task.py; states set directly).

    scripts/rta simpy <id> mech/tasks/inhaler_coin/trials.py /workspace/robocurve/outputs/<id> [off]

``off``  the coin taken out of the mouthpiece (its weld cut, laid flat on the bench: 0.25), then the cap put 15 cm
         past the bench's left edge, 2 cm over the bench top, and let fall: the score is 0 from that step on and
         stays 0 when the cap is put back on the bench.
The other levels are shown by demo.py's runs: ``--case naive`` (0: the cap pushed on over the coin), ``--case held``
(0.5), the full run (0.25, 0.5 and 1.0 in turn). Writes <out dir>/off.json.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

BENCH_LEFT = 0.55  # m: the station bench's left edge (yam_arena station.TABLE_CENTER and TABLE_SIZE)


def _hold():
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))


def off(embodiment, out: Path, seed: int) -> dict:
    from inspect_robots import Scene

    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    embodiment.reset(Scene(id="off", instruction=m.INSTRUCTION), seed=seed)
    embodiment.step(_hold())
    rows = []

    def settle(what: str, seconds: float) -> None:
        for _ in range(int(round(seconds * hz))):
            embodiment.step(_hold())
        rows.append({"what": what, "t": round(mk.now(), 2), "score": m.score(None), "coins in": m.coins_in(), "left the bench": m.STATE["off"],
                     "cap z over the bench (mm)": round(1000 * float(mk.pose("cap")[0][2] - m.TOP), 1)})  # fmt: skip
        print("OFF", json.dumps(rows[-1], default=str), flush=True)

    settle("as built", 0.5)
    mk.cut("lodged")
    mk.write_pose("coin", (0.24, 0.12, m.TOP + 0.5 * m.P.COIN_T + 0.001), (0.0, 0.0, 0.0, 1.0))
    settle("coin out, laid on the bench", 1.0)
    cap_quat = tuple(mk.matrix_to_quat(m.CAP_LYING))
    mk.write_pose("cap", (m.CAP_AT[0], BENCH_LEFT + 0.15, m.TOP + 0.02), cap_quat)
    settle("cap put past the bench's left edge", 2.0)
    mk.write_pose("cap", (m.CAP_AT[0], m.CAP_AT[1], m.TOP + 0.5 * m.P.CAP_DEPTH + 0.001), cap_quat)
    settle("cap put back on the bench", 1.0)
    found = {"rows": rows, "events": [f"{t:.2f} s {w}" for t, w in m.STATE["events"]]}
    print(f"TRIAL off {json.dumps(found, default=str)}", flush=True)
    return found


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    trials = sys.argv[2].split(",") if len(sys.argv) > 2 else ["off"]
    known = {"off": off}
    if any(name not in known for name in trials):
        raise SystemExit(f"trials here: {sorted(known)}; got {trials}")
    out.mkdir(parents=True, exist_ok=True)
    embodiment = kit.open_task(HERE / "task.py")
    for name in trials:
        (out / f"{name}.json").write_text(json.dumps(known[name](embodiment, out, 0), indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
