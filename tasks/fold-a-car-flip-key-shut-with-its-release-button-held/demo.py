#!/usr/bin/env python3
"""The flip key's scripted full-score run: both arms, open loop from the key's start (its turn in the dish follows
from the trial's seed, a layout constant).

    scripts/rta demo <id> mech/tasks/flip_key [full] [--quick] [seed=N]

The right hand takes the key across the fob's width and holds it level; the left hand, level, pinches the fob
across its thickness with its upper pad on the release button (the pinch presses it in); the right hand lets go,
opens 70 mm and, its rear finger on the blade's trailing edge, swings the blade round to folded; there it closes
across the fob's width, one pad holding the blade in; the left hand lets go (the button comes out, the pawl drops
into the folded notch) and draws back; the right hand sets the key down in the dish and lets go.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

OPEN_MM = 94.0  # the gripper's widest opening (grip 1)
FOLD_YAW = math.radians(140.0)  # the key's turn while it is folded in the air (checks.py: the level pinch's band)
BUTTON_GRASP = np.array([0.25, -0.04, 0.12])  # the left hand's grasp point on the button, over the bench (m)
ALPHA = math.radians(115.0)  # the left hand's direction of approach in the fob's frame
HOLD_X = -0.019  # the right hand's grasp across the width, along the fob from its middle (m)
Z_GRASP = 0.006  # the right hand's grasp point over the fob's middle plane: fingertips 1 mm over its underside
PUSH_S = 0.040  # the rear finger on the blade's trailing edge this far from the pin (m)
PUSH_OPEN = 0.070  # the right hand's opening while it pushes (m)
STEP_DEG = 10.0  # the push in steps of this many degrees of fold
STEP_S = 0.30


def grip(width_m: float) -> float:
    return min(1.0, max(0.0, 1000.0 * width_m / OPEN_MM))


def rz(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def frame(closing, approach) -> np.ndarray:
    a = np.asarray(approach, float) / np.linalg.norm(approach)
    c = np.asarray(closing, float) - np.dot(closing, a) * a
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


DOWN = np.array([0.0, 0.0, -1.0])


class Key:
    """The key's frame (fob middle; x along the fob to the pin end, y across to the channel, z out of the buttons' face)."""

    def __init__(self, m, yaw: float, at) -> None:
        self.m, self.r, self.p = m, rz(yaw), np.asarray(at, float)

    def world(self, local) -> np.ndarray:
        return self.p + self.r @ np.asarray(local, float)

    def vec(self, local) -> np.ndarray:
        return self.r @ np.asarray(local, float)

    def push_pose(self, theta: float, s: float = PUSH_S, opening: float = PUSH_OPEN):
        """Grasp point and rotation with the rear finger's pad on the blade's trailing edge ``s`` from the pin."""
        m = self.m
        e, n = np.array([math.cos(theta), math.sin(theta), 0.0]), np.array([-math.sin(theta), math.cos(theta), 0.0])
        trailing = self.world(m.PIN + s * e - m.BLADE_W / 2 * n)
        c = self.vec(n)
        g = trailing + opening / 2 * c
        g[2] = self.p[2] + Z_GRASP
        return g, frame(c, DOWN)


def plan(m, seed: int | None) -> dict:
    from demo_kit import Move
    from yam_arena import ik, station
    from yam_arena.constants import READY

    ready = {side: ik.tcp_pose(np.array(READY, float), station.BASES[side]) for side in ("left", "right")}
    yaw0 = m.start_yaw(seed)
    start = Key(m, yaw0, m._turned(m.FOB_AT, yaw0))
    b_at = np.array([BUTTON_GRASP[0], BUTTON_GRASP[1], m.TOP + BUTTON_GRASP[2]])
    fold = Key(m, FOLD_YAW, b_at - rz(FOLD_YAW) @ np.array([m.BUTTON_AT[0], m.BUTTON_AT[1], 0.0]))
    hold_open = grip(m.FOB_W + 0.018)

    # right hand: take the key across the fob's width and bring it level to where it is folded
    pick = start.world([HOLD_X, 0.0, Z_GRASP])
    pick_rot = frame(start.vec([0.0, 1.0, 0.0]), DOWN)
    held = fold.world([HOLD_X, 0.0, Z_GRASP])
    held_rot = frame(fold.vec([0.0, 1.0, 0.0]), DOWN)
    up = np.array([0.0, 0.0, 0.06])
    right = [Move(2.0, pick + up, pick_rot, hold_open, "over the key"), Move(1.5, pick, pick_rot, hold_open, "down to the key"),
             Move(0.8, pick, pick_rot, 0.0, "close across the fob"), Move(1.0, pick + up, pick_rot, 0.0, "lift"),
             Move(2.5, held, held_rot, 0.0, "to the fold place, turned")]  # fmt: skip
    t_held = sum(mv.seconds for mv in right)

    # left hand: level, its approach along u, the closing axis upright; in along u onto the button and pinch
    u = fold.vec([math.cos(ALPHA), math.sin(ALPHA), 0.0])
    b_rot = frame([0.0, 0.0, 1.0], u)
    b_open = grip(m.FOB_T + 0.012)
    left = [Move(3.0, b_at - 0.07 * u, b_rot, b_open, "level, back from the button"), Move(t_held - 3.0 + 0.3, None, None, None, "wait"),
            Move(1.5, b_at, b_rot, b_open, "in onto the button"), Move(0.8, b_at, b_rot, 0.0, "pinch: button in")]  # fmt: skip
    t_pinched = sum(mv.seconds for mv in left)

    # right hand: let go, open to push, the rear finger behind the open blade, the swing, and the close over the fold
    g0, r0 = fold.push_pose(0.0)
    right += [Move(t_pinched - t_held + 0.2, None, None, None, "wait for the pinch"), Move(0.6, None, None, hold_open, "let go"),
              Move(0.8, held + np.array([0.0, 0.0, 0.04]), None, None, "up"), Move(2.0, g0 + np.array([0.0, 0.0, 0.03]), r0, grip(PUSH_OPEN), "over the blade's start"),
              Move(1.0, g0, r0, None, "down behind the blade")]  # fmt: skip
    deg = 0.0
    while deg < 180.0:
        deg = min(180.0, deg + STEP_DEG)
        g, r = fold.push_pose(math.radians(deg))
        right.append(Move(STEP_S, g, r, None, f"swing {deg:.0f}"))
    g, r = fold.push_pose(math.radians(180.0), s=PUSH_S, opening=PUSH_OPEN - 0.0006)  # 0.3 mm into the stop: home
    right.append(Move(0.3, g, r, None, "home"))
    x_end = m.PIN[0] - PUSH_S
    c_end = fold.vec([0.0, -1.0, 0.0])  # the closing axis as the swing ends
    pinch = fold.world([x_end, 0.00015, Z_GRASP])
    right.append(Move(1.5, pinch, frame(c_end, DOWN), 0.0, "close across the fob, a pad on the blade"))
    t_closed = sum(mv.seconds for mv in right)

    # left hand lets go: the button comes out and the pawl drops; it draws back
    left += [Move(t_closed - t_pinched + 0.3, None, None, None, "hold the button"), Move(0.6, None, None, b_open, "let go: button out"),
             Move(1.0, b_at - 0.07 * u, None, None, "back"),
             Move(3.0, ready["left"][0], ready["left"][1], None, "back to ready (the way it came)")]  # fmt: skip
    t_free = t_closed + 0.3 + 0.6 + 1.0

    # right hand: the key into the dish, as it was turned while folded, the fob's middle on the dish's middle
    place_key = Key(m, FOLD_YAW, np.array([m.TRAY_AT[0], m.TRAY_AT[1], m.FLOOR_TOP + m.FOB_T / 2 + 0.001]))
    place = place_key.world([x_end, 0.00015, Z_GRASP])
    right += [Move(t_free - t_closed, None, None, None, "wait for the left hand"), Move(0.8, pinch + np.array([0.0, 0.0, 0.03]), None, None, "lift"),
              Move(2.5, place + np.array([0.0, 0.0, 0.03]), frame(place_key.vec([0.0, -1.0, 0.0]), DOWN), None, "over the dish"),
              Move(1.5, place, None, None, "down into the dish"), Move(0.6, None, None, hold_open, "let go"),
              Move(1.2, place + np.array([0.0, 0.0, 0.10]), None, None, "up and clear")]  # fmt: skip
    return {"left": left, "right": right}


def main() -> int:
    import demo_kit as kit

    quick = "--quick" in sys.argv
    args = [a for a in sys.argv[1:] if a != "--quick" and not a.startswith("seed=")]
    seed = next((int(a.split("=", 1)[1]) for a in sys.argv[1:] if a.startswith("seed=")), 0)
    out = Path(args[0])
    runs = args[1].split(",") if len(args) > 1 else ["full"]
    if runs != ["full"]:
        raise SystemExit("one run here: full")
    if quick:
        from yam_arena import station, task_file

        m = task_file.load(str(HERE / "task.py")).module
        moves = plan(m, seed)
        print(f"DRY full seed {seed}: {json.dumps(kit.dry_run(moves, station.BASES, limit_s=120.0), default=str)}", flush=True)
        return 0
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)

    def extra() -> dict:
        return {"fold (deg)": round(math.degrees(m.fold()), 2), "button in (mm)": round(1000 * mk.joint("press"), 3), "pawl": m.STATE["pawl"],
                "fob": [round(float(v), 4) for v in mk.pose("fob")[0]], "in tray": m.in_tray(), "let go": m.let_go(), "at rest": m.at_rest(),
                "grips": [round(float(mk.gripper(s)[1]), 3) for s in ("left", "right")]}  # fmt: skip

    folder = out / "full"
    result = kit.run(embodiment, plan(m, seed), folder, seed=seed, extra=extra, limit_s=m.RUN_LIMIT_STEPS / hz, tail_s=1.5, sheet_frames=16)
    rows = result["rows"]
    scores = [row["info"].get("score", 0.0) for row in rows]
    firsts = {level: next((row["t"] for row, s in zip(rows, scores) if s >= level), None) for level in (0.25, 0.5, 1.0)}
    readings = {"seed": seed, "score": scores[-1], "first reached": firsts, "seconds": rows[-1]["t"], "steps": result["steps"], "report": result["report"],
                "worst IK": result["worst_ik"], "events": m.report()["events"]}  # fmt: skip
    (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
    print(f"READINGS full: {json.dumps(readings, default=str)}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
