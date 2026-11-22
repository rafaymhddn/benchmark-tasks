#!/usr/bin/env python3
"""The scripted full-score run of the MRI tray task, and a check of its plan against the arm's reach.

    scripts/rta demo <id> mech/tasks/mri_cart_clear [--no-video] [--quick]

The right arm takes each steel item off the tray and drops it into the marked bin: over the item, around it,
fingers closed on the named part (the clamp's T-handle, the padlock's body, the shears' raised rings), up, over
the bin, fingers open, and back towards its ready pose while the item settles. The left arm stays at its ready
pose throughout. Open loop: the plan reads nothing from the simulator. The poses are checks.py's (reach, joint
limits and collisions checked there).

Results in outputs/<id>/full/: result.json (every step), readings.json (the score, the bin's count over time,
the settled items), sheets of pictures, and the videos unless --no-video. --quick: the plan through the arm's IK
without the simulator (worst IK error, smallest joint margin).
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as K  # noqa: E402
import parts as P  # noqa: E402

ORDER = ("padlock", "shears", "clamp")  # flat shears onto the empty bin floor before the bulky clamp
SETTLE_S = 4.0  # the last item settles and the hold runs out inside this


def park() -> np.ndarray:
    """Where the working hand passes between items: inside its own reach, clear of the tray and the bin."""
    return np.array([P.PARK[0], P.PARK[1], K.TOP + P.PARK[2]], dtype=float)


def moves_for(name: str) -> list:
    import demo_kit as kit

    Move = kit.Move
    gx, gy, gz = P.GRASPS[name]
    g = np.array([gx, gy, K.TOP + gz], dtype=float)
    rx, ry = P.RELEASES[name]
    over_bin = np.array([rx, ry, K.TOP + K.BIN_Z[name]], dtype=float)
    pre = [
        Move(1.1, pos=g + (0.0, 0.0, K.PRE), rot=K.DOWN_X, grip=1.0, label=f"over the {name}"),
        Move(0.7, pos=g, grip=K.OPEN[name] / K.MAX_JAW, label=f"around the {name}"),
        Move(0.6, grip=K.GRIP[name] / K.MAX_JAW, label=f"closed on the {name}"),
    ]
    if name == "padlock":
        return pre + [
            Move(0.9, pos=g + (0.0, 0.0, 0.100), rot=K.CARRY_BIN, label="up off the tray"),
            Move(1.3, pos=over_bin, rot=K.CARRY_BIN, label="to the bin"),
            Move(0.5, grip=1.0, label="let go"),
            Move(0.9, pos=park(), rot=K.CARRY_BIN, label="back to the park"),
        ]
    if name == "clamp":
        high = np.array([rx, ry, K.TOP + 0.280], dtype=float)
        return pre + [
            Move(1.1, pos=g + (0.0, 0.0, 0.160), rot=K.CARRY_BIN, label="up off the tray"),
            Move(1.4, pos=high, rot=K.CARRY_BIN, label="high over the bin"),
            Move(0.7, pos=over_bin, rot=K.CARRY_BIN, label="into the bin"),
            Move(0.5, grip=1.0, label="let go"),
            Move(0.8, pos=high, rot=K.CARRY_BIN, label="out of the bin"),
        ]
    pull = np.array([0.205, -0.065, K.TOP + 0.125], dtype=float)
    high = np.array([0.210, -0.200, K.TOP + 0.255], dtype=float)
    above_bin = np.array([rx + 0.025, ry, K.TOP + 0.255], dtype=float)
    return pre + [
        Move(0.8, pos=g + (0.0, 0.0, K.LIFT), rot=K.DOWN_X, label="up off the tray"),
        Move(0.9, pos=pull, rot=K.DOWN_X, label="clear of the tray"),
        Move(0.9, pos=high, rot=K.CARRY_BIN, label="high before the bin"),
        Move(0.9, pos=above_bin, rot=K.CARRY_BIN, label="over the bin"),
        Move(0.9, pos=over_bin, rot=K.DOWN_BIN, label="level in the bin"),
        Move(0.7, grip=1.0, label="let go"),
        Move(0.5, pos=over_bin + (0.0, 0.0, 0.010), rot=K.DOWN_BIN, label="straight up clear"),
        Move(0.8, pos=above_bin, rot=K.CARRY_BIN, label="out of the bin"),
        Move(0.9, pos=park(), rot=K.CARRY_BIN, label="back to the park"),
    ]


def plan() -> dict[str, list]:
    import demo_kit as kit
    from yam_arena import ik
    from yam_arena.constants import READY

    ready_pos, ready_rot = ik.tcp_pose(np.array(READY, dtype=float), K.BASES["right"])
    right = [kit.Move(1.2, pos=park(), rot=K.CARRY_BIN, grip=1.0, label="to the park")]
    for name in ORDER:
        right += moves_for(name)
    right += [kit.Move(1.6, pos=ready_pos, rot=ready_rot, grip=1.0, label="back to ready")]
    return {"left": [], "right": right}


def save_pictures(first: dict, last: dict, folder: Path) -> None:
    import cv2
    import demo_kit as kit

    for cam, stem in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        bgr = cv2.cvtColor(np.asarray(first[cam]), cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(folder / f"{stem}.png"), bgr)
        cv2.imwrite(str(folder / f"{stem}_policy.png"), cv2.cvtColor(kit.policy_picture(np.asarray(first[cam])), cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(folder / "end_top.png"), cv2.cvtColor(np.asarray(last["top_cam"]), cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(folder / "end_top_policy.png"), cv2.cvtColor(kit.policy_picture(np.asarray(last["top_cam"])), cv2.COLOR_RGB2BGR))


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    moves = plan()
    if "--quick" in sys.argv[2:]:
        from yam_arena import station

        dry = kit.dry_run(moves, station.BASES)
        print("QUICK " + json.dumps(dry), flush=True)
        return 0
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module

    def extra() -> dict:
        return {"in bin": list(m.binned()), "hands clear mm": round(1000 * m.hands_clear(), 1),
                "hold": m.STATE.get("hold", 0), "zero": m.STATE.get("zero")}  # fmt: skip

    folder = out / "full"
    record = "--no-video" not in sys.argv[2:] and "--check" not in sys.argv[2:]
    first_pics: dict = {}
    latest: dict = {}

    def director(_t, _arms, pictures) -> None:
        if pictures is not None:
            if not first_pics:
                first_pics.update(pictures)
            latest.update(pictures)

    result = kit.run(embodiment, moves, folder, seed=0, director=director, extra=extra, limit_s=float(m.RUN_LIMIT_STEPS) / 20.0,
                     tail_s=SETTLE_S, sheet_frames=16, record=record)  # fmt: skip
    rows = result["rows"]
    save_pictures(first_pics, latest, folder)
    first = {"0.5": next((r["t"] for r in rows if r["info"].get("score", 0.0) == 0.5), None),
             "1.0": next((r["t"] for r in rows if r["info"].get("score", 0.0) == 1.0), None)}
    readings = {"score at the end": rows[-1]["info"].get("score"), "seconds": rows[-1]["t"],
                "first 0.5 at (s)": first["0.5"], "first 1.0 at (s)": first["1.0"],
                "steps": len(rows), "in bin at the end": rows[-1]["extra"]["in bin"],
                "zero at the end": rows[-1]["extra"]["zero"], "hold steps at the end": rows[-1]["extra"]["hold"],
                "moved (mm)": m.report()["moved (mm)"], "safe removed": m.report()["safe removed"],
                "worst IK": result.get("worst_ik"), "median step (ms)": result.get("median_step_ms")}  # fmt: skip
    (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
    print("READINGS " + json.dumps(readings, default=str), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
