#!/usr/bin/env python3
"""Checks of the shield task in the simulator.

    scripts/rta simpy <id> mech/tasks/shield_seat/trials.py /workspace/robocurve/outputs/<id> [rest,press]

``rest``   3 s with nobody touching anything: the first pictures (top camera and both wrist cameras, as rendered and
           at the size a policy gets), how far the four strips stand in their sockets at the start and after 3 s,
           how far anything moves.
``resist`` what the four seats resist with, steady loads put on directly without an arm: each seat by itself under
           nine and eleven tenths of its resistance, then the four together with the shield standing on them.
``lift``   states set directly: the shield put 2 cm up and let go (does it fall back onto its pin tips?); the shield
           and its seats put home, then the shield pulled up 3 cm and let go (nothing holds it down).
``press``  the arm's check, with the scene as a spring gauge (the four seats are springs of 5 kN/m each instead of
           resistances): the right hand, pointing down and open so that a fingertip's end stands on each row of the
           shield's own sockets, presses at the middle of the pins with 20, 40, 53.4, 60 and 80 N asked for; what the
           four springs read under each. Asked for with ``press`` alone or with ``rest`` before it: the scene then
           looks the same, and ``rest`` reports the springs instead of the resistances.

Every trial writes <out dir>/<trial>.json; ``rest`` its pictures; ``press`` a run folder with its videos.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

OPEN = 0.472  # the hand's opening (0 closed .. 1 open) that puts a fingertip's end, 3.9 mm thick, on each row of sockets: 44.4 mm between the pads
TIP = 0.0129  # m the fingertips' ends reach past the grasp point
ASKED = (20.0, 40.0, 53.4, 60.0, 80.0)


def press_point(m) -> np.ndarray:
    """Where the hand presses: over the middle of the 32 pins (their resistances' centre), on the board's centre line."""
    x = sum(m.PINS[name] * m.strip_place(name)[0] for name in m.NAMES) / sum(m.PINS.values())
    return np.array([x, m.CARRIER_XY[1], 0.0])


def own_socket_top(m) -> float:
    """The tops of the shield's own sockets over the world's zero, at the start."""
    return m.TOP + m.SHIELD_Z + m.PCB_T / 2 + m.SOCKET_H


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
    cv2.imwrite(str(out / f"{name}_policy.png"), cv2.cvtColor(small, cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(out / f"{name}_policy_x3.png"), cv2.cvtColor(cv2.resize(small, (small.shape[1] * 3, small.shape[0] * 3), interpolation=cv2.INTER_NEAREST), cv2.COLOR_RGB2BGR))


def rest(embodiment, out: Path) -> dict:
    from inspect_robots import Scene

    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    observation = embodiment.reset(Scene(id="rest", instruction=m.INSTRUCTION), seed=0)
    for camera, name in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        _save(out, name, np.asarray(observation.images[camera]))
    rows = []
    for _ in range(int(round(3.0 * hz))):
        embodiment.step(_hold())
        rows.append({"in": {k: round(1000 * v, 3) for k, v in m.depths().items()}, "seats": {name: round(1000 * mk.joint(f"in_{name}"), 3) for name in m.NAMES},
                     "shield": [round(float(v), 5) for v in mk.pose("shield")[0]], "up": round(1000 * mk.joint("shield_up"), 3),
                     "tilt (deg)": [round(float(np.degrees(mk.joint("shield_about_x"))), 3), round(float(np.degrees(mk.joint("shield_about_y"))), 3)]})  # fmt: skip
    moved = {name: round(1000 * float(np.linalg.norm(mk.pose(name)[0] - mk.built_pose(name)[0])), 3) for name in mk.MECH.bodies if mk.MECH.bodies[name].free}
    found = {"gauge": m.GAUGE, "strips in at the first step (mm)": rows[0]["in"], "after 3 s (mm)": rows[-1]["in"], "seats' travel after 3 s (mm)": rows[-1]["seats"],
             "shield up (mm) and tilted (deg) after 3 s": [rows[-1]["up"], rows[-1]["tilt (deg)"]], "bodies from where they were built after 3 s (mm)": moved,
             "score": m.score(None), "report": m.report()}  # fmt: skip
    print(f"REST {json.dumps(found)}", flush=True)
    return found


def press(embodiment, out: Path) -> dict:
    import demo_kit as kit

    m = embodiment.task.module
    mk = m.mk
    assert m.GAUGE, "the press check needs the scene as a gauge: SHIELD_GAUGE=1"
    at = press_point(m)
    touch = np.array([at[0], at[1], own_socket_top(m) + TIP])
    Move = kit.Move
    moves = [Move(3.0, pos=touch + np.array([0.0, 0.0, 0.08]), rot=kit.down(0.0), grip=OPEN, label="over the shield"),
             Move(2.0, pos=touch + np.array([0.0, 0.0, 0.002]), label="down to it"), Move(0.5, label="on it")]  # fmt: skip
    for force in ASKED:
        moves.append(Move(1.0, push=(0.0, 0.0, -force), push_at=np.array([0.0, 0.0, TIP]), label=f"press {force:g} N"))
        moves.append(Move(1.5, label=f"hold {force:g} N"))
    moves.append(Move(1.0, push=(0.0, 0.0, 0.0), label="ease"))
    moves.append(Move(1.5, pos=touch + np.array([0.0, 0.0, 0.08]), label="up"))

    def extra() -> dict:
        travel = {name: float(mk.joint(f"in_{name}")) for name in m.NAMES}
        return {"springs (N)": round(-m.GAUGE_K * sum(travel.values()), 3), "by strip (N)": {name: round(-m.GAUGE_K * v, 2) for name, v in travel.items()},
                "in (mm)": {k: round(1000 * v, 2) for k, v in m.depths().items()}, "hand": [round(float(v), 4) for v in mk.gripper("right")[0]]}  # fmt: skip

    result = kit.run(embodiment, {"right": moves}, out / "press", seed=0, extra=extra, limit_s=40.0, tail_s=1.0)
    rows = result["rows"]
    rest_reading = float(np.median([row["extra"]["springs (N)"] for row in rows if row["right"] == "over the shield"]))
    found = {"at rest the springs read (N: the shield's weight)": round(rest_reading, 3), "asked, read (N), by strip, strips in (mm), hand over the bench (mm)": []}
    for force in ASKED:
        held = [row for row in rows if row["right"] == f"hold {force:g} N"][-10:]
        read = float(np.median([row["extra"]["springs (N)"] for row in held])) - rest_reading
        found["asked, read (N), by strip, strips in (mm), hand over the bench (mm)"].append(
            [force, round(read, 2), held[-1]["extra"]["by strip (N)"], held[-1]["extra"]["in (mm)"], round(1000 * (held[-1]["extra"]["hand"][2] - m.TOP), 1)])
    found["worst IK (mm, deg)"] = result["worst_ik"]
    found["press point (bench x, y)"] = [round(float(at[0]), 4), round(float(at[1]), 4)]
    print(f"PRESS {json.dumps(found)}", flush=True)
    return found


def resist(embodiment, out: Path) -> dict:
    """What the four seats resist with, steady loads put on directly (no arm). Each seat by itself: nine tenths of
    its resistance for 3 s, then eleven tenths until it has gone 3 mm, then nothing. Then the four seats together,
    each under the same part of its own resistance, the shield standing on them: eight tenths and then 95
    hundredths for 3 s each, eleven tenths until the shield is home, then nothing."""
    from inspect_robots import Scene

    m = embodiment.task.module
    mk = m.mk
    assert not m.GAUGE
    hz = float(embodiment.info.control_hz)

    def load(names, part: float) -> None:
        for name in names:
            mk.push(f"seat_{name}", force=(0.0, 0.0, m.SEAT_MASS * m.GRAVITY - part * m.RESISTS[name]))  # its weight stays carried

    def steps(seconds: float, until=None) -> float:
        for k in range(int(round(seconds * hz))):
            embodiment.step(_hold())
            if until is not None and until():
                return (k + 1) / hz
        return seconds

    def travel(name: str) -> float:
        return 1000 * float(mk.joint(f"in_{name}"))

    alone = {}
    for name in m.NAMES:
        embodiment.reset(Scene(id=f"resist {name}", instruction=m.INSTRUCTION), seed=0)
        steps(1.0)
        start = travel(name)
        load([name], 0.9)
        steps(3.0)
        held = travel(name)
        load([name], 1.1)
        took = steps(8.0, until=lambda name=name, held=held: travel(name) <= held - 3.0)
        low = travel(name)
        load([name], 0.0)
        steps(1.0)
        alone[name] = {"resists with (N)": round(m.RESISTS[name], 2), "under 0.9 of it for 3 s it went (mm)": round(start - held, 3),
                       "under 1.1 of it it went (mm), in (s), mm/s": [round(held - low, 2), took, round((held - low) / took, 2)],
                       "let go it came back (mm)": round(travel(name) - low, 3)}  # fmt: skip
    found: dict = {"each seat by itself": alone}

    embodiment.reset(Scene(id="resist together", instruction=m.INSTRUCTION), seed=0)
    steps(1.0)
    before = m.depths()
    together: dict = {"the four resistances add up to (N)": round(sum(m.RESISTS.values()), 2)}
    for part in (0.8, 0.95):
        load(m.NAMES, part)
        steps(3.0)
        now = m.depths()
        together[f"under {part:g} of every strip's resistance for 3 s the strips went in (mm)"] = {n: round(1000 * (now[n] - before[n]), 3) for n in m.NAMES}
        before = now
    load(m.NAMES, 1.1)
    took = steps(12.0, until=lambda: max(travel(n) for n in m.NAMES) <= -1000 * m.TRAVEL + 0.05)
    steps(1.0)
    home, seats = m.depths(), {n: round(travel(n), 3) for n in m.NAMES}
    load(m.NAMES, 0.0)
    steps(1.0)
    after = m.depths()
    together.update({"under 1.1 of it the four seats were home after (s)": took, "seats' travel then (mm)": seats,
                     "strips in a second later (mm): the shield, pressed by nothing, has come down after them": {n: round(1000 * home[n], 2) for n in m.NAMES},
                     "a second after the load was taken off: seats' travel, strips in (mm)": [{n: round(travel(n), 3) for n in m.NAMES}, {n: round(1000 * after[n], 2) for n in m.NAMES}],
                     "most any two strips were apart in height (mm)": round(1000 * m.STATE["most_apart"], 2), "bent": m.STATE["bent"], "score": m.score(None)})  # fmt: skip
    found["the four seats together, the shield on them"] = together
    print(f"RESIST {json.dumps(found)}", flush=True)
    return found


def lift(embodiment, out: Path) -> dict:
    """States set directly, no arm. (1) The shield put 2 cm over its start and let go: does it fall back onto its
    pin tips? (2) The shield and its four seats put home, left for a second, then the shield alone put 3 cm higher
    and let go: nothing holds it down (the first reading, a twentieth of a second later, has it in the air), and it
    falls home again."""
    from inspect_robots import Scene

    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    chain = ("shield_lift", "shield_roll", "shield")

    def run(seconds: float) -> list:
        rows = []
        for _ in range(int(round(seconds * hz))):
            embodiment.step(_hold())
            rows.append({"in": {k: round(1000 * v, 2) for k, v in m.depths().items()}, "score": m.score(None), "bent": m.STATE["bent"] is not None})
        return rows

    def raise_shield(by: float) -> None:
        for name in chain:  # the three bodies of the shield's joints together
            position, rotation = mk.pose(name)
            mk.write_pose(name, position + np.array([0.0, 0.0, by]), mk.matrix_to_quat(rotation))

    found: dict = {}
    embodiment.reset(Scene(id="lift", instruction=m.INSTRUCTION), seed=0)
    run(0.5)
    raise_shield(0.02)
    rows = run(0.05)
    high = rows[0]["in"]
    rows = run(1.5)
    found["lifted 2 cm and let go"] = {"strips in right after the lift (mm)": high, "1.5 s later (mm)": rows[-1]["in"], "score": rows[-1]["score"], "bent": rows[-1]["bent"],
                                       "seats' travel (mm)": {n: round(1000 * mk.joint(f"in_{n}"), 3) for n in m.NAMES}}  # fmt: skip

    embodiment.reset(Scene(id="pulled", instruction=m.INSTRUCTION), seed=0)
    run(0.5)
    m.put_seats(-m.TRAVEL)
    raise_shield(-m.TRAVEL)
    rows = run(1.0)
    seated = {"strips in (mm)": rows[-1]["in"], "score": rows[-1]["score"]}
    raise_shield(0.03)
    rows = run(0.05)
    up = {"strips in (mm)": rows[0]["in"], "score": rows[0]["score"]}
    rows = run(1.5)
    found["put home, then pulled up 3 cm and let go"] = {"home": seated, "pulled up": up, "1.5 s later": {"strips in (mm)": rows[-1]["in"], "score": rows[-1]["score"]},
                                                         "bent": rows[-1]["bent"], "seats' travel (mm)": {n: round(1000 * mk.joint(f"in_{n}"), 3) for n in m.NAMES}}  # fmt: skip
    print(f"LIFT {json.dumps(found)}", flush=True)
    return found


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    known = {"rest": rest, "press": press, "resist": resist, "lift": lift}
    trials = sys.argv[2].split(",") if len(sys.argv) > 2 else ["rest"]
    if any(name not in known for name in trials):
        raise SystemExit(f"trials here: {sorted(known)}; got {trials}")
    if "press" in trials:
        os.environ["SHIELD_GAUGE"] = "1"  # read by the task file when it is loaded
    out.mkdir(parents=True, exist_ok=True)
    embodiment = kit.open_task(HERE / "task.py")
    failed = 0
    for name in trials:
        try:
            result = known[name](embodiment, out)
        except Exception as error:
            import traceback

            traceback.print_exc()
            print(f"TRIAL {name} FAILED: {error!r}", flush=True)
            failed += 1
            continue
        (out / f"{name}.json").write_text(json.dumps(result, indent=1, default=str))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
