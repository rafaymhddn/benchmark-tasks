#!/usr/bin/env python3
"""Checks of the pill crusher in the simulator, no arm.

    scripts/rta simpy <id> mech/tasks/pill_crusher/trials.py /workspace/robocurve/outputs/<id> [rest,press]

``lead_in`` the top let fall over the held base from 3 mm over the rim, in line, 0.8 mm off, 2 mm off, 2 mm off and
           tilted 5 deg, 4 mm off: taken or not, and how it stands (``high``: in line from 50 mm and 10 mm over the rim).
``drop``   the top written in line over the held base, its plunger out of the well, then at the thread's start:
           where it goes and what it touches, step by step.
``rest``   3 s with nobody touching anything: the first pictures (top camera and both wrist cameras, as rendered and
           at the size a policy gets), and how far anything moves.
``powder`` (``powder_tilted``: the base held tilted 10 deg) both tablets crushed, the top turned on to the thread's
           end over the grains and back up, lifted off: how far the crusher's parts go, how many grains leave the well.
``press``  the base held where it stands (the trials' weld "base_hold"), a round tablet written flat on the well's
           floor, the top written in line 8 mm over the rim and dropped in (the take rule takes it), then turned by a torque
           about the base's axis: 0.03 N m down to the tablet; 0.10 and 0.20 N m held 1 s each (the press's angle: its
           spring's rate as built here); 1.4 x TABLET_NM until it gives; 0.03 N m on down to the thread's end; -0.03 N m back
           up until the top is free.

Every trial writes <out dir>/<trial>.json.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def _hold():
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))


def _save(out: Path, name: str, frame: np.ndarray) -> None:
    import cv2
    import demo_kit as kit

    cv2.imwrite(str(out / f"{name}.png"), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    small = kit.policy_picture(frame)
    cv2.imwrite(str(out / f"{name}_policy_x3.png"), cv2.cvtColor(cv2.resize(small, (small.shape[1] * 3, small.shape[0] * 3), interpolation=cv2.INTER_NEAREST), cv2.COLOR_RGB2BGR))


def settle(embodiment, seconds: float) -> None:
    for _ in range(int(round(seconds * float(embodiment.info.control_hz)))):
        embodiment.step(_hold())


def fresh(embodiment, m, name: str):
    from inspect_robots import Scene

    observation = embodiment.reset(Scene(id=name, instruction=m.INSTRUCTION), seed=0)
    embodiment.step(_hold())
    return observation


def rest(embodiment, out: Path) -> dict:
    m = embodiment.task.module
    mk = m.mk
    observation = fresh(embodiment, m, "rest")
    for camera, name in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        _save(out, name, np.asarray(observation.images[camera]))
    settle(embodiment, 3.0)
    names = ("cup_a", "cup_er", "cup_b", "tablet_a", "tablet_er", "tablet_b", "base", "top", "sauce", "carrier", "turner", "press", "grain_a00", "grain_er23")
    moved = {name: round(1000 * float(np.linalg.norm(mk.pose(name)[0] - mk.built_pose(name)[0])), 3) for name in names}
    found = {"moved from where they were built after 3 s (mm)": moved, "score": m.score(None), "after 3 s": m.report()}
    print(f"REST {json.dumps(found, default=str)}", flush=True)
    return found


def row(m, label: str, steps: int, torque: float) -> dict:
    mk = m.mk
    g = m.top_reading()
    return {"after": label, "steps": steps, "torque (N m)": torque, "mode": m.STATE["mode"], "turn (turns)": round(mk.joint("turn") / (2 * math.pi), 3),
            "face over the floor (mm)": round(1000 * g["face"], 3), "thread's face (mm)": None if m.STATE["d"] is None else round(1000 * m.STATE["d"], 3),
            "press (deg)": round(math.degrees(mk.joint("press")), 2), "top off the axis (mm, deg)": [round(1000 * g["off"], 3), round(math.degrees(g["tilt"]), 2)],
            "crushed": dict(m.STATE["crushed"]), "base moved (mm)": round(1000 * float(np.linalg.norm(mk.pose("base")[0] - mk.built_pose("base")[0])), 3),
            "events": m.STATE["events"][-2:]}  # fmt: skip


def turn_top(embodiment, m, torque: float, seconds: float, until=None, label: str = "") -> dict:
    """Turn the top about the base's axis with ``torque`` (N m, + screwing down) for ``seconds`` or until ``until()``."""
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    steps = 0
    for steps in range(1, int(round(seconds * hz)) + 1):
        down = mk.pose("base")[1] @ m.Z
        mk.push("top", torque=tuple(float(v) for v in (-torque * down)))
        embodiment.step(_hold())
        if until is not None and until():
            break
    mk.push("top")
    found = row(m, label, steps, torque)
    print(f"PRESS {json.dumps(found, default=str)}", flush=True)
    return found


def press(embodiment, out: Path) -> dict:
    m = embodiment.task.module
    mk = m.mk
    fresh(embodiment, m, "press")
    mk.make("base_hold")
    p, r = mk.pose("base")
    mk.write_pose("tablet_a", p + r @ np.array([0.003, -0.002, m.FLOOR + m.PLAIN_T / 2 + 0.0002]), mk.matrix_to_quat(r))
    settle(embodiment, 0.5)
    mk.write_pose("top", p + r @ np.array([0.0, 0.0, m.FLOOR + m.F0 + 0.0002]), mk.matrix_to_quat(r @ m.rot_z(0.4)))  # in line at the thread's start, held there as a hand holds it
    mk.make("top_hold")
    for _ in range(10):
        embodiment.step(_hold())
        if m.STATE["mode"] == "thread":
            break
    mk.cut("top_hold")
    settle(embodiment, 0.3)
    rows = [row(m, "the top held in line at the thread's start, taken, let go", 0, 0.0)]
    print(f"PRESS {json.dumps(rows[-1], default=str)}", flush=True)
    rows.append(turn_top(embodiment, m, 0.03, 12.0, until=lambda: m.STATE["mode"] == "press", label="0.03 N m down to the tablet"))
    for torque in (0.10, 0.20):
        rows.append(turn_top(embodiment, m, torque, 1.0, label=f"{torque} N m held 1 s"))
    give = round(1.4 * m.TABLET_NM, 2)
    rows.append(turn_top(embodiment, m, give, 3.0, until=lambda: "tablet_a" in m.STATE["crushed"], label=f"{give} N m until it gives"))
    rows.append(turn_top(embodiment, m, 0.0, 0.5, label="let go 0.5 s"))
    rows.append(turn_top(embodiment, m, 0.03, 8.0, until=lambda: m.STATE["d"] is not None and m.STATE["d"] <= m.D_BOTTOM + 1e-6, label="0.03 N m on down to the thread's end"))
    rows.append(turn_top(embodiment, m, -0.03, 15.0, until=lambda: bool(m.STATE.get("loose")), label="-0.03 N m back up until it sits on the thread's start"))
    rows.append(turn_top(embodiment, m, 0.0, 1.0, label="let go 1 s: it sits there"))
    hz = float(embodiment.info.control_hz)
    for _ in range(int(round(1.0 * hz))):  # lifted: 0.30 N up (it weighs 0.26 N) until it is free
        mk.push("top", force=(0.0, 0.0, 0.30))
        embodiment.step(_hold())
        if m.STATE["mode"] == "free":
            break
    mk.push("top")
    rows.append(row(m, "lifted with 0.30 N", 0, 0.0))
    print(f"PRESS {json.dumps(rows[-1], default=str)}", flush=True)
    mk.write_pose("top", (*m.TOP_XY, m.TOP + m.LIFT), (0.0, 0.0, 0.0, 1.0))  # carried off, as a hand would
    settle(embodiment, 0.5)
    grains = [mk.pose(g)[0] for g in m.GRAINS["tablet_a"]]
    local = np.array([m.in_base(g) for g in grains])
    found = {"rows": rows, "spring rate as built (N m/rad)": m.K_TABLET,
             "grains in the well": int(sum(math.hypot(x, y) < m.WELL_D / 2 and m.FLOOR - 0.001 <= z <= m.BASE_H for x, y, z in local)),
             "grains' heights over the floor (mm)": [round(1000 * float(local[:, 2].min() - m.FLOOR), 2), round(1000 * float(local[:, 2].max() - m.FLOOR), 2)],
             "report": m.report()}  # fmt: skip
    print(f"PRESSED {json.dumps({k: v for k, v in found.items() if k != 'rows'}, default=str)}", flush=True)
    return found


def powder(embodiment, out: Path, tilt_deg: float = 0.0) -> dict:
    """A policy may turn on past the gives. The base held (level, or tilted ``tilt_deg`` about the line
    across the arms' front, as a holding hand might), both paracetamol tablets side by side in the well, the top set
    in and taken, then turned by a torque: down until both give, on down to the thread's end and 2 s more, back up
    until it sits on the start, let go, lifted with 0.30 N until free. How far the crusher's parts went from the base
    (the top's face off its axis while on the thread; the hidden bodies against the base), and how many grains left
    the well."""
    m = embodiment.task.module
    mk = m.mk
    name = "powder" if not tilt_deg else "powder_tilted"
    fresh(embodiment, m, name)
    if tilt_deg:  # the base and its jointed hidden bodies turned together about the foot's middle, the foot's edge on the bench
        c, _ = mk.pose("base")
        t = math.radians(tilt_deg)
        turn = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(t), -math.sin(t)], [0.0, math.sin(t), math.cos(t)]])
        lift = np.array([0.0, 0.0, m.BASE_D / 2 * math.sin(t) + 0.0005])
        for body in ("base", *m.HIDDEN):
            q, s_ = mk.pose(body)
            mk.write_pose(body, c + turn @ (np.asarray(q) - c) + lift, mk.matrix_to_quat(turn @ s_))
        embodiment.step(_hold())
    mk.make("base_hold")
    p, r = mk.pose("base")
    for tablet, y in (("tablet_a", 0.0085), ("tablet_b", -0.0085)):
        mk.write_pose(tablet, p + r @ np.array([0.0, y, m.FLOOR + m.PLAIN_T / 2 + 0.0002]), mk.matrix_to_quat(r))
    settle(embodiment, 0.5)
    mk.write_pose("top", p + r @ np.array([0.0, 0.0, m.FLOOR + m.F0 + 0.0002]), mk.matrix_to_quat(r @ m.rot_z(0.4)))
    mk.make("top_hold")
    for _ in range(10):
        embodiment.step(_hold())
        if m.STATE["mode"] == "thread":
            break
    mk.cut("top_hold")
    settle(embodiment, 0.3)
    held = mk.pose("base")
    hidden0 = {h: m.in_base(mk.pose(h)[0]) for h in m.HIDDEN}
    worst = {"top off its axis on the thread (mm)": 0.0, "hidden bodies against the base (mm)": 0.0, "base from where it was held (mm)": 0.0}

    def watch() -> bool:
        if m.STATE["mode"] != "free":
            worst["top off its axis on the thread (mm)"] = max(worst["top off its axis on the thread (mm)"], 1000 * m.top_reading()["off"])
        carrier = m.in_base(mk.pose("carrier")[0]) - hidden0["carrier"]  # it slides along the axis with the thread: off that
        along = 0.0 if m.STATE["mode"] != "thread" or m.STATE["d"] is None else abs(float(carrier[2]) - (m.STATE["d"] - m.F0))
        off = max(math.hypot(float(carrier[0]), float(carrier[1])), along, float(np.linalg.norm(m.in_base(mk.pose("press")[0]) - hidden0["press"])))
        worst["hidden bodies against the base (mm)"] = max(worst["hidden bodies against the base (mm)"], 1000 * off)
        worst["base from where it was held (mm)"] = max(worst["base from where it was held (mm)"], 1000 * float(np.linalg.norm(mk.pose("base")[0] - held[0])))
        return False

    def grains_in() -> int:
        return sum(m.in_well(g) for t in m.STATE["crushed"] for g in m.GRAINS[t])

    rows = [turn_top(embodiment, m, 0.03, 12.0, until=lambda: watch() or m.STATE["mode"] == "press", label="0.03 N m down to the tablets")]
    give = round(2 * 1.4 * m.TABLET_NM, 2)
    rows.append(turn_top(embodiment, m, give, 3.0, until=lambda: watch() or len(m.STATE["crushed"]) == 2, label=f"{give} N m until both give"))
    rows.append(turn_top(embodiment, m, 0.0, 0.5, until=watch, label="let go 0.5 s"))
    given = grains_in()
    rows.append(turn_top(embodiment, m, 0.03, 8.0, until=lambda: watch() or (m.STATE["d"] is not None and m.STATE["d"] <= m.D_BOTTOM + 1e-6), label="0.03 N m on down to the thread's end"))
    rows.append(turn_top(embodiment, m, 0.03, 2.0, until=watch, label="0.03 N m 2 s more at the end"))
    at_end = grains_in()
    rows.append(turn_top(embodiment, m, -0.03, 15.0, until=lambda: watch() or bool(m.STATE.get("loose")), label="-0.03 N m back up until it sits on the thread's start"))
    rows.append(turn_top(embodiment, m, 0.0, 1.0, until=watch, label="let go 1 s"))
    hz = float(embodiment.info.control_hz)
    for _ in range(int(round(1.0 * hz))):  # lifted: 0.30 N along the base's axis (it weighs 0.26 N) until it is free
        mk.push("top", force=tuple(float(v) for v in 0.30 * (mk.pose("base")[1] @ m.Z)))
        embodiment.step(_hold())
        watch()
        if m.STATE["mode"] == "free":
            break
    mk.push("top")
    rows.append(row(m, "lifted with 0.30 N", 0, 0.0))
    found = {"tilt (deg)": tilt_deg, "grains in the well after the gives": given, "at the end": at_end, "after the lift": grains_in(),
             "free after the lift": m.STATE["mode"] == "free", "worst": {k: round(v, 2) for k, v in worst.items()},
             "highest grain over the floor at the end (mm)": round(1000 * m.powder_top(), 2), "rows": rows}  # fmt: skip
    print(f"POWDER {json.dumps({k: v for k, v in found.items() if k != 'rows'}, default=str)}", flush=True)
    return found


def drop(embodiment, out: Path) -> dict:
    """The top written in line over the held base: its plunger out of the well, then at the thread's start; four
    control steps each: where it goes and what it touches."""
    m = embodiment.task.module
    mk = m.mk
    fresh(embodiment, m, "drop")
    mk.make("base_hold")
    p, r = mk.pose("base")
    rows = []
    for label, face in (("face 20 mm over the floor", 0.020), ("face 0.2 mm over the thread's start", m.F0 + 0.0002)):
        mk.write_pose("top", p + r @ np.array([0.0, 0.0, m.FLOOR + face]), mk.matrix_to_quat(r))
        for k in range(4):
            embodiment.step(_hold())
            q, s = mk.pose("top")
            v, w = mk.velocity("top")
            g = m.top_reading()
            touch = sorted({(c.other, c.shape, round(1000 * c.separation, 2), round(c.force, 2)) for c in mk.contacts("top")}, key=lambda x: x[2])[:12]
            rows.append({"after": label, "step": k + 1, "face (mm)": round(1000 * g["face"], 3), "off (mm)": round(1000 * g["off"], 3), "tilt (deg)": round(math.degrees(g["tilt"]), 2),
                         "v (m/s)": [round(float(x), 3) for x in v], "w (rad/s)": [round(float(x), 2) for x in w], "touches (other, shape, sep mm, N)": touch,
                         "mode": m.STATE["mode"], "events": m.STATE["events"][-1:]})  # fmt: skip
            print(f"DROP {json.dumps(rows[-1], default=str)}", flush=True)
        settle(embodiment, 0.3)
    return {"rows": rows}


def lead_in(embodiment, out: Path) -> dict:
    """The top let fall over the held base from 3 mm over the rim: in line; 2 mm off; 2 mm off and tilted 5 deg; 4 mm
    off (beyond the 3 mm lead-in). After 1.5 s: taken by the thread, and how it stands."""
    m = embodiment.task.module
    mk = m.mk
    rows = []
    cases = (("in line", 0.0, 0.0, 0.003), ("0.8 mm off", 0.0008, 0.0, 0.003), ("2 mm off", 0.002, 0.0, 0.003), ("2 mm off, 5 deg", 0.002, 5.0, 0.003), ("4 mm off", 0.004, 0.0, 0.003))
    if "high" in sys.argv:
        cases = (("in line", 0.0, 0.0, 0.050), ("in line", 0.0, 0.0, 0.010))
    for label, off, tilt, over in cases:
        fresh(embodiment, m, "lead_in")
        mk.make("base_hold")
        p, r = mk.pose("base")
        rot = r @ mk.axis_angle((1.0, 0.0, 0.0), math.radians(tilt))
        mk.write_pose("top", p + r @ np.array([off, 0.0, m.BASE_H + over]), mk.matrix_to_quat(rot))
        for k in range(12 if label == "in line" else 0):
            embodiment.step(_hold())
            g = m.top_reading()
            v, w = mk.velocity("top")
            touch = sorted({(c.other, c.shape, round(1000 * c.separation, 2), round(c.force, 2)) for c in mk.contacts("top")}, key=lambda x: x[2])[:10]
            print(f"TRACE step {k + 1}: face {1000 * g['face']:.2f} mm off {1000 * g['off']:.2f} tilt {math.degrees(g['tilt']):.2f} v {np.round(v, 3).tolist()} w {np.round(w, 2).tolist()} mode {m.STATE['mode']} touch {touch}", flush=True)
        settle(embodiment, 1.5)
        g = m.top_reading()
        rows.append({"start": label, "mode": m.STATE["mode"], "face (mm)": round(1000 * g["face"], 2), "off (mm)": round(1000 * g["off"], 3), "tilt (deg)": round(math.degrees(g["tilt"]), 2),
                     "events": m.STATE["events"][-1:]})  # fmt: skip
        print(f"LEAD_IN {json.dumps(rows[-1], default=str)}", flush=True)
    return {"rows": rows}


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "trials"
    out.mkdir(parents=True, exist_ok=True)
    wanted = sys.argv[2].split(",") if len(sys.argv) > 2 else ["rest", "press"]
    embodiment = kit.open_task(HERE / "task.py")
    for name in wanted:
        found = {"rest": rest, "press": press, "drop": drop, "lead_in": lead_in, "powder": powder,
                 "powder_tilted": lambda e, o: powder(e, o, tilt_deg=10.0)}[name](embodiment, out)
        (out / f"{name}.json").write_text(json.dumps(found, indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
