#!/usr/bin/env python3
"""Checks of single mechanisms in the simulator (development; the scored levels are demo.py's cases).

    scripts/rta simpy <id> mech/tasks/sb50_mate/trials.py /workspace/robocurve/outputs/<id> [cover]

``cover``  the demo's first 13 s: the left hand holds the left lead, the right hand pulls the cover off; logs the
           cover's slide, its pose, the fingers' forces and every touch on the cover, each step.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def cover(emb, m, out: Path) -> dict:
    import demo
    import demo_kit as kit

    plan = demo.static_plan("full")
    keep = {}
    for side, moves in plan.items():
        t, kept = 0.0, []
        for mv in moves:
            if t >= 13.0:
                break
            kept.append(mv)
            t += mv.seconds
        keep[side] = kept

    def extra() -> dict:
        touches = {}
        for c in m.mk.contacts("dust_cover"):
            touches[c.other] = round(touches.get(c.other, 0.0) + abs(float(c.force)), 2)
        pl, rl = m.mk.pose("sb_left")
        pc, rc = m.mk.pose("dust_cover")
        return {"cap_slide_mm": round(1000 * m.mk.joint("cap_slide"), 2), "cover": [round(float(v), 4) for v in m.mk.pose("dust_cover")[0]],
                "cover_in_left_mm": [round(1000 * float(v), 2) for v in (pc - pl) @ rl], "cover_tilt_deg": round(float(np.degrees(np.arccos(np.clip((np.trace(rl.T @ rc) - 1) / 2, -1, 1)))), 2),
                "sb_left": [round(float(v), 4) for v in m.mk.pose("sb_left")[0]], "fingers_n": m.scene.finger_forces(), "cover_touches": touches,
                "grip": {s: [round(float(v), 4) for v in m.mk.gripper(s)[0]] + [round(m.mk.gripper(s)[1], 4)] for s in ("left", "right")}}  # fmt: skip

    res = kit.run(emb, keep, out / "cover", seed=0, extra=extra, limit_s=13.0, tail_s=0.5, record=False)
    rows = [{"t": r["t"], "R": r["right"], **r["extra"]} for r in res["rows"]]
    return {"cap_slide_mm_end": rows[-1]["cap_slide_mm"], "rows": rows[::5]}


TRIALS = {"cover": cover}


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    names = sys.argv[2].split(",") if len(sys.argv) > 2 else list(TRIALS)
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    for name in names:
        result = TRIALS[name](emb, m, out)
        (out / f"{name}.json").write_text(json.dumps(result, indent=1, default=str))
        print(f"TRIAL {name} " + json.dumps({k: v for k, v in result.items() if k != "rows"}, default=str), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
