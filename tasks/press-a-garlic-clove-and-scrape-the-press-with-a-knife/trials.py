#!/usr/bin/env python3
"""Checks of the garlic press task in the simulator, no arm.

    scripts/rta simpy <id> mech/tasks/garlic_press/trials.py /workspace/robocurve/outputs/<id> [rest,drop,press] [seeds]

``rest``   3 s with nobody touching anything: the first pictures (top camera and both wrist cameras, as rendered and
           at the size a policy gets), how far every body moves, the press's hinge (its drive capped at nothing must
           leave the open handle on its stop), and the clove on the board.
``drop``   the press open where it starts; the clove let go over the chamber as a hand lets it go, its middle 15 mm over
           the rim (the fingertips 2 mm over it), in seven ways: lengthways, 3 mm off the middle, turned 15 and 30
           degrees, across, on its side, and the largest clove lengthways. Where it comes to rest after 1.5 s.
``press``  the press where it is pressed (51 mm past the board's edge, over the saucer), opened 45 degrees: first the
           hinge's drive read as a spring before any clove (its rest there, known torques 1, 2, 4 N m; the press free
           on the board with a torque pair on its two parts, then held where it stands by a weld). Then, free, the
           clove lengthways in the chamber; the handle let fall onto it by its own weight; then known closing torques
           on the handle, the same opening torque on the lower part (the press as a whole feels none): a quarter, a
           half, three quarters of the yield's (1 s each), nine tenths (2 s), eleven tenths (3 s); then let go. Then the closed press lifted 6 cm over the saucer and held there, a knife moved under its sieve
           (blade flat, its top face 0.5 mm under the sieve) once across the press at 2 cm/s, then its tip set on the
           saucer's well. Amounts at each stage; pictures of the strands and the saucer.

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
    cv2.imwrite(str(out / f"{name}_policy.png"), cv2.cvtColor(small, cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(out / f"{name}_policy_x3.png"), cv2.cvtColor(cv2.resize(small, (small.shape[1] * 3, small.shape[0] * 3), interpolation=cv2.INTER_NEAREST), cv2.COLOR_RGB2BGR))


def steps(embodiment, seconds: float, each=None) -> list:
    """Step ``seconds`` holding the arms ready; ``each()`` before every step (a load to put on); its readings kept."""
    rows = []
    for _ in range(int(round(seconds * float(embodiment.info.control_hz)))):
        if each is not None:
            each()
        result = embodiment.step(_hold())
        rows.append(result)
    return rows


def fresh(embodiment, m, name: str, seed: int = 0):
    from inspect_robots import Scene

    observation = embodiment.reset(Scene(id=name, instruction=m.INSTRUCTION), seed=seed)
    embodiment.step(_hold())
    return observation


def rest(embodiment, out: Path, seed: int = 0) -> dict:
    m = embodiment.task.module
    mk = m.mk
    observation = fresh(embodiment, m, "rest", seed)
    tag = "" if seed == 0 else f"_seed{seed}"
    for camera, name in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        _save(out, name + tag, np.asarray(observation.images[camera]))
    first = {name: mk.pose(name)[0].copy() for name in mk.MECH.bodies if mk.MECH.bodies[name].free}
    hinge0 = mk.joint("hinge")
    steps(embodiment, 3.0)
    moved = {name: round(1000 * float(np.linalg.norm(mk.pose(name)[0] - first[name])), 3) for name in first}
    clove = m.STATE["clove"]
    found = {
        "seed": seed, "clove": clove, "lie": m.STATE["lie"],
        "moved over 3 s from the first step (mm)": moved,
        "hinge at the first step and after 3 s (deg)": [round(math.degrees(hinge0), 3), round(math.degrees(mk.joint("hinge")), 3)],
        "clove's middle over the bench (mm)": round(1000 * float(mk.pose(clove)[0][2] - m.TOP), 2),
        "report": m.report(),
    }  # fmt: skip
    print(f"REST {json.dumps(found, default=str)}", flush=True)
    return found


def put_press(m, shift: float, phi: float) -> None:
    """Both parts of the press ``shift`` m along +y from where they start, the handle opened ``phi``, at rest."""
    mk = m.mk
    lower = (m.START_HINGE + np.array([0.0, shift, 0.0]), np.eye(3))
    upper = mk.joint_pose(mk.MECH.joints["hinge"], phi, lower)
    mk.write_pose("press", lower[0], mk.matrix_to_quat(lower[1]))
    mk.write_pose("upper", upper[0], mk.matrix_to_quat(upper[1]))
    mk.MECH.travel["hinge"] = float(phi)


def clove_over(m, u: float, v: float, along: float, height: float, side: bool = False, name: str | None = None) -> None:
    """The trial's clove (or ``name``) at rest with its middle over (u, v) of the chamber, ``height`` m over the floor,
    its length turned ``along`` rad from the press's line, on its side or its flat face."""
    mk = m.mk
    name = name or m.STATE["clove"]
    at = m.press_point(u, v, m.FLOOR_W + height)
    yaw = -math.pi / 2 + along  # its x (root to tip) along the press (-y) at 0
    rot = mk.axis_angle((0.0, 0.0, 1.0), yaw) @ (mk.axis_angle((1.0, 0.0, 0.0), math.pi / 2) if side else np.eye(3))
    mk.write_pose(name, at, mk.matrix_to_quat(rot))


def where_clove(m, name: str | None = None) -> dict:
    """The clove's place in the press: share inside the chamber's outline, its lowest and highest points over the
    floor, its tilt."""
    mk = m.mk
    name = name or m.STATE["clove"]
    points = m.in_press(m.to_world(name, m.CLOVE_POINTS[name]))
    inside = (points[:, 0] >= m.CHAMBER_U[0]) & (points[:, 0] <= m.CHAMBER_U[1]) & (np.abs(points[:, 1]) <= m.CHAMBER_V)
    up = mk.pose(name)[1] @ np.array([0.0, 0.0, 1.0])
    return {"inside the outline": round(float(inside.mean()), 3), "lowest over the floor (mm)": round(1000 * float(points[:, 2].min() - m.FLOOR_W), 2),
            "highest over the floor (mm)": round(1000 * float(points[:, 2].max() - m.FLOOR_W), 2), "tilt (deg)": round(math.degrees(math.acos(min(1.0, abs(float(up[2]))))), 1)}  # fmt: skip


def drop(embodiment, out: Path, seed: int = 0) -> dict:
    m = embodiment.task.module
    mk = m.mk
    middle = (m.CHAMBER_U[0] + m.CHAMBER_U[1]) / 2
    cases = {
        "lengthways": dict(u=middle, v=0.0, along=0.0),
        "3 mm off the middle": dict(u=middle, v=0.003, along=0.0),
        "turned 15 deg": dict(u=middle, v=0.0, along=math.radians(15.0)),
        "turned 30 deg": dict(u=middle, v=0.0, along=math.radians(30.0)),
        "across": dict(u=middle, v=0.0, along=math.pi / 2),
        "on its side, lengthways": dict(u=middle, v=0.0, along=0.0, side=True),
        "largest clove lengthways": dict(u=middle, v=0.0, along=0.0, name="clove_2"),
    }
    found = {}
    for label, case in cases.items():
        fresh(embodiment, m, f"drop {label}", seed)
        name = case.get("name", m.STATE["clove"])
        if name != m.STATE["clove"]:  # the trial's clove off the board, this one in its place
            mk.write_pose(m.STATE["clove"], m.PARKED[m.STATE["clove"]], (0.0, 0.0, 0.0, 1.0))
            m.STATE.update(clove=name, mass=m.CLOVE_SIZES[m.CLOVES.index(name)][3])
        _, width, thick, _ = m.CLOVE_SIZES[m.CLOVES.index(name)]
        clove_over(m, case["u"], case["v"], case["along"], (m.RIM_W - m.FLOOR_W) + 0.015, side=case.get("side", False), name=name)
        steps(embodiment, 1.5)
        found[label] = where_clove(m, name)
        print(f"DROP {label}: {json.dumps(found[label])}", flush=True)
    return found


SPRING_TORQUES = (1.0, 2.0, 4.0)  # N m: well under the cap the drive is given for the reading


def spring_rate(embodiment, m, pair: bool) -> dict:
    """The hinge's drive read as a spring: known closing torques on the upper part, 1 s each (with ``pair`` the same
    torque opening the lower part, so that the press as a whole feels none and stands free); N m/rad from the turn,
    beside the stiffness the scene gives the drive (CLOVE_K)."""
    mk = m.mk
    steps(embodiment, 0.5)
    base = mk.joint("hinge")
    where = mk.pose("press")[0].copy()
    rows = []
    for torque in SPRING_TORQUES:
        seen = []
        for _ in range(int(round(float(embodiment.info.control_hz)))):
            mk.push("upper", torque=(torque, 0.0, 0.0))
            if pair:
                mk.push("press", torque=(-torque, 0.0, 0.0))
            embodiment.step(_hold())
            seen.append(mk.joint("hinge"))
        turn = base - float(np.median(seen[-6:]))
        rows.append({"torque (N m)": torque, "turn (deg)": round(math.degrees(turn), 4), "rate (N m/rad)": round(torque / turn, 1) if turn > 0 else None})
    mk.push("upper")
    if pair:
        mk.push("press")
    steps(embodiment, 0.5)
    return {"hinge before (deg)": round(math.degrees(base), 3), "torques": rows, "the scene's stiffness (N m/rad)": round(m.CLOVE_K, 1),
            "press moved (mm)": round(1000 * float(np.linalg.norm(mk.pose("press")[0] - where)), 2), "hinge after (deg)": round(math.degrees(mk.joint("hinge")), 3)}  # fmt: skip


def torque_on_handle(m, newton_metres: float) -> None:
    """A closing torque on the handle (about +x: its grip end goes down) and the same opening torque on the lower part,
    so that the press as a whole feels none (0: both off)."""
    m.mk.push("upper", torque=(newton_metres, 0.0, 0.0))
    m.mk.push("press", torque=(-newton_metres, 0.0, 0.0))


def press(embodiment, out: Path, seed: int = 0) -> dict:
    import cv2

    m = embodiment.task.module
    mk = m.mk
    fresh(embodiment, m, "press", seed)
    shift = m.PRESSING_OVERHANG - (m.START_TIP_Y - m.BOARD_Y[1])  # from 10 mm inside the board's edge to 51 mm past it
    put_press(m, shift, math.radians(45.0))
    steps(embodiment, 0.05)  # the press read where it was put (a pose written is read back at the next step)
    mk.set_spring("hinge", rest=math.radians(45.0), most=50.0)  # no clove yet: press_on() leaves the drive alone
    drive = {"free on the board, a torque pair": spring_rate(embodiment, m, pair=True)}
    mk.make("press_hold")
    steps(embodiment, 0.1)
    drive["held where it stands, a torque on the upper part"] = spring_rate(embodiment, m, pair=False)
    print(f"DRIVE {json.dumps(drive)}", flush=True)
    mk.cut("press_hold")  # free from here on, as in the task, until it is lifted
    middle = (m.CHAMBER_U[0] + m.CHAMBER_U[1]) / 2
    clove_over(m, middle, 0.0, 0.0, m.CLOVE_SIZES[0][2] / 2 + 0.0005)
    steps(embodiment, 0.1)
    mk.set_spring("hinge", rest=0.0, most=0.0)  # as the task has it before a clove is touched: the handle falls
    log = []

    def reading(label: str) -> dict:
        phi = mk.joint("hinge")
        row = {"stage": label, "t": round(mk.now(), 2), "hinge (deg)": round(math.degrees(phi), 3), "face over the floor (mm)": round(1000 * m.face_height(phi), 3),
               "touched": m.STATE["touched"], "top (mm)": None if m.STATE["top"] is None else round(1000 * m.STATE["top"], 3),
               "resistance now (N m)": round(m.STATE["cap"], 3), "into the clove (mm)": None if m.STATE["top"] is None else round(1000 * (m.STATE["top"] - m.face_height(phi)), 3),
               "amounts (g)": m.amounts()}  # fmt: skip
        log.append(row)
        print(f"PRESS {json.dumps(row)}", flush=True)
        return row

    steps(embodiment, 2.0)
    reading("fallen onto the clove by its own weight")
    for part in (0.25, 0.5, 0.75):
        steps(embodiment, 1.0, lambda: torque_on_handle(m, part * m.YIELD_TORQUE))
        reading(f"{part:g} of the yield")
    steps(embodiment, 0.5, lambda: torque_on_handle(m, 0.9 * m.YIELD_TORQUE))
    reading("0.9 of the yield, 0.5 s")
    steps(embodiment, 1.5, lambda: torque_on_handle(m, 0.9 * m.YIELD_TORQUE))
    reading("0.9 of the yield, 2 s")
    before = mk.joint("hinge")
    steps(embodiment, 3.0, lambda: torque_on_handle(m, 1.1 * m.YIELD_TORQUE))
    reading("1.1 of the yield, 3 s")
    pressed_to = mk.joint("hinge")
    torque_on_handle(m, 0.0)
    steps(embodiment, 1.0)
    reading("let go, 1 s")
    springback = m.face_height(mk.joint("hinge")) - m.face_height(pressed_to)
    observation = steps(embodiment, 0.1)[-1].observation
    _save(out, "pressed_top", np.asarray(observation.images["top_cam"]))
    # The closed press lifted 6 cm over the saucer and held there; a knife across under its sieve; its tip on the saucer
    lower, rotation = mk.pose("press")
    lifted = lower + np.array([0.0, 0.0, 0.060])
    upper = mk.joint_pose(mk.MECH.joints["hinge"], mk.joint("hinge"), (lifted, rotation))
    mk.write_pose("press", lifted, mk.matrix_to_quat(rotation))
    mk.write_pose("upper", upper[0], mk.matrix_to_quat(upper[1]))
    mk.make("press_hold")  # made at the written pose (the kit takes a pose written since the last reading)
    steps(embodiment, 0.05)
    steps(embodiment, 0.5)
    reading("lifted 6 cm over the saucer")
    observation = steps(embodiment, 0.1)[-1].observation
    _save(out, "lifted_top", np.asarray(observation.images["top_cam"]))
    knife_rot = mk.axis_angle((0.0, 0.0, 1.0), -math.pi / 2)  # pointing -y, flat
    sieve = m.press_point(m.PLUNGER_MID[0], 0.0, 0.0)
    blade_mid = (m.BLADE_X[0] + m.BLADE_X[1]) / 2
    hz = float(embodiment.info.control_hz)
    carried = (0.0, 0.0, m.KNIFE_MASS * m.GRAVITY)  # the knife's weight carried, so that it keeps to where it is put
    for k in range(int(round(3.0 * hz)) + 1):  # 6 cm in 3 s, at 2 cm/s
        dx = -0.030 + 0.060 * k / (3.0 * hz)
        at = sieve - knife_rot @ np.array([blade_mid, 0.0, m.BLADE_Z[1]]) + np.array([dx, 0.0, -0.0005])
        mk.write_pose("knife", at, mk.matrix_to_quat(knife_rot), lin_vel=(0.020, 0.0, 0.0))
        mk.push("knife", force=carried)
        embodiment.step(_hold())
    reading("knife once across under the sieve")
    tilt = knife_rot @ mk.axis_angle((0.0, 1.0, 0.0), math.radians(15.0))
    well = np.array([*mk.pose("saucer")[0][:2], m.TOP + m.WELL_W])
    for k in range(int(round(1.0 * hz))):
        mk.write_pose("knife", well - tilt @ np.array([m.BLADE_X[1] - 0.012, 0.0, m.BLADE_Z[0]]) + np.array([0.0, 0.0, 0.001]), mk.matrix_to_quat(tilt))
        mk.push("knife", force=carried)
        embodiment.step(_hold())
    mk.push("knife")
    reading("knife's tip on the saucer's well")
    observation = steps(embodiment, 0.2)[-1].observation
    _save(out, "saucer_top", np.asarray(observation.images["top_cam"]))
    mk.cut("press_hold")
    found = {"drive": drive, "stages": log, "springback when let go (mm)": round(1000 * springback, 3), "events": m.STATE["events"], "report": m.report(),
             "under 0.9 of the yield, from 0.5 s to 2 s: hinge moved (deg)": round(log[4]["hinge (deg)"] - log[5]["hinge (deg)"], 3),
             "went under 1.1 of the yield (deg)": round(math.degrees(before - pressed_to), 3)}  # fmt: skip
    print(f"PRESSED {json.dumps({k: v for k, v in found.items() if k != 'stages'}, default=str)}", flush=True)
    return found


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    trials = sys.argv[2].split(",") if len(sys.argv) > 2 else ["rest"]
    seeds = [int(v) for v in sys.argv[3].split(",")] if len(sys.argv) > 3 else [0]
    known = {"rest": rest, "drop": drop, "press": press}
    if any(name not in known for name in trials):
        raise SystemExit(f"trials here: {sorted(known)}; got {trials}")
    out.mkdir(parents=True, exist_ok=True)
    embodiment = kit.open_task(HERE / "task.py")
    failed = 0
    for name in trials:
        for seed in seeds:
            try:
                result = known[name](embodiment, out, seed)
            except Exception as error:
                import traceback

                traceback.print_exc()
                print(f"TRIAL {name} seed {seed} FAILED: {error!r}", flush=True)
                failed += 1
                continue
            (out / f"{name}{'' if seed == 0 else f'_seed{seed}'}.json").write_text(json.dumps(result, indent=1, default=str))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
