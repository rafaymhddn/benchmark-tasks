#!/usr/bin/env python3
"""Checks of the servo horn task in the simulator, no arm.

    scripts/rta simpy <id> mech/tasks/servo_horn/trials.py /workspace/robocurve/outputs/<id> [rest,capture,press] [servo=x,y] [horn=x,y] [shaft=near|far]

``rest``     3 s with nobody touching anything: the first pictures (top camera and both wrist cameras, as rendered and
             at the size a policy gets), and how far the horn and the crests move.
``capture``  the horn let fall onto the shaft from 2 mm over its place on the crests, its axis put 0 to 1.2 mm aside
             and tilted 0 to 6 degrees, the arm 4 degrees off a tooth position: where it comes to rest (on the crests,
             or not). The spec's figures: over the shaft within 0.8 mm, tilted 4 degrees or less.
``press``    the horn on the crests, pressed down along its axis with a steady force for 2 s: on the mark's tooth with
             20, 30, 34 and 40 N, and 3 degrees off the tooth with 40 and 60 N; then a horn pressed home pulled up with 8
             and with 12 N; and a horn set turning at 2 rad/s about the shaft, resting on the crests and pressed home.

Every trial writes <out dir>/<trial>.json.
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

DROP = 0.002  # m over its place on the crests that the horn is let fall from
ASIDE = (0.0, 0.0004, 0.0007, 0.0008, 0.0009, 0.0012)  # m the hub's axis is put from the shaft's
TILTS = (0.0, 3.0, 4.0, 6.0)  # degrees, about the arm
OFF_TOOTH = math.radians(4.0)


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


def reading(m) -> dict:
    return {"off the shaft (mm)": round(1000 * m.off_axis(), 3), "tilt (deg)": round(math.degrees(m.tilt()), 2), "down (mm)": round(1000 * m.down(), 3),
            "arm from the mark (deg)": round(math.degrees(m.arm_angle()), 2), "on the crests": m.captured(), "engaged": m.engaged(), "seat (mm)": round(1000 * m.mk.joint("down"), 3), "level": m.level()}  # fmt: skip


def horn_rotation(m, angle: float, tilt: float = 0.0) -> np.ndarray:
    """The horn's frame with its arm ``angle`` rad from the mark's direction, tilted ``tilt`` rad about the arm."""
    mk = m.mk
    arm = np.array(mk.axis_angle((0.0, 0.0, 1.0), angle)) @ m.MARK
    yaw = math.atan2(arm[1], arm[0])
    return np.array(mk.axis_angle((0.0, 0.0, 1.0), yaw)) @ np.array(mk.axis_angle((1.0, 0.0, 0.0), tilt))


def put_horn(m, angle: float, over: float = 0.0, aside: float = 0.0, tilt: float = 0.0) -> None:
    """The horn with its mouth ``over`` m over its place on the crests, ``aside`` m from the shaft's axis (across the
    arm), at rest."""
    mk = m.mk
    rotation = horn_rotation(m, angle, tilt)
    across = rotation[:, 1]
    at = np.array([*m.SHAFT_AT, m.TOP + m.REST_Z + over]) + aside * np.array([across[0], across[1], 0.0])
    mk.write_pose("horn", at, mk.matrix_to_quat(rotation))


def centre_of_mass(m) -> np.ndarray:
    """The horn's centre of mass in its own frame, from its solid shapes at even density (as the simulator takes it)."""
    total, moment = 0.0, np.zeros(3)
    for s in m.mk.MECH.bodies["horn"].shapes:
        if not s.solid:
            continue
        volume = float(np.prod(s.dims)) if s.kind == "box" else math.pi * s.dims[0] ** 2 * s.dims[1]
        total += volume
        moment += volume * np.array(s.pos)
    return moment / total


def press_along_axis(m, force: float) -> None:
    """A force along the horn's axis (positive: down) through the axis, put on the horn's centre of mass with the
    torque that moves it onto the axis."""
    mk = m.mk
    position, rotation = mk.pose("horn")
    pushed = -force * rotation[:, 2]
    lever = rotation @ (np.array([0.0, 0.0, m.HUB_H]) - centre_of_mass(m))  # from the centre of mass to the hub's top on its axis
    m.push_horn(force=tuple(pushed), torque=tuple(np.cross(lever, pushed)))  # with the push that carries a welded horn's weight over 4 g


def settle(embodiment, seconds: float) -> None:
    for _ in range(int(round(seconds * float(embodiment.info.control_hz)))):
        embodiment.step(_hold())


def fresh(embodiment, m, name: str) -> None:
    from inspect_robots import Scene

    embodiment.reset(Scene(id=name, instruction=m.INSTRUCTION), seed=0)
    embodiment.step(_hold())


def rest(embodiment, out: Path) -> dict:
    from inspect_robots import Scene

    m = embodiment.task.module
    mk = m.mk
    observation = embodiment.reset(Scene(id="rest", instruction=m.INSTRUCTION), seed=0)
    for camera, name in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        _save(out, name, np.asarray(observation.images[camera]))
    settle(embodiment, 3.0)
    moved = round(1000 * float(np.linalg.norm(mk.pose("horn")[0] - mk.built_pose("horn")[0])), 3)
    found = {"horn from where it was built after 3 s (mm)": moved, "after 3 s": reading(m), "score": m.score(None), "centre of mass in the horn's frame (mm)": [round(1000 * v, 2) for v in centre_of_mass(m)]}
    print(f"REST {json.dumps(found)}", flush=True)
    return found


def capture(embodiment, out: Path) -> dict:
    m = embodiment.task.module
    found: dict = {}
    for tilt in TILTS:
        for aside in ASIDE:
            fresh(embodiment, m, f"capture {aside} {tilt}")
            put_horn(m, OFF_TOOTH, over=DROP, aside=aside, tilt=math.radians(tilt))
            settle(embodiment, 1.5)
            now = reading(m)
            key = f"{1000 * aside:.1f} mm aside, tilted {tilt:g} deg"
            found[key] = {"after 1.5 s": now, "on the crests": m.on_shaft() and abs(m.down()) <= m.ON_CRESTS}
            print(f"CAPTURE {key}: {json.dumps(found[key])}", flush=True)
    return found


def drop_trace(embodiment, out: Path) -> dict:
    """The horn let fall on the axis tilted 3 and 4 degrees: every control step of the first second (taken, cut free,
    the crests' travel, how high, tilted, off the axis, the horn's speed)."""
    m = embodiment.task.module
    mk = m.mk
    found: dict = {}
    for tilt in (3.0, 4.0):
        fresh(embodiment, m, f"drop {tilt}")
        put_horn(m, OFF_TOOTH, over=DROP, tilt=math.radians(tilt))
        rows = []
        for k in range(20):
            embodiment.step(_hold())
            linear, angular = mk.velocity("horn")
            rows.append([k + 1, m.captured(), m.STATE["freed"], round(1000 * mk.joint("down"), 3), round(1000 * m.down(), 3), round(math.degrees(m.tilt()), 2), round(1000 * m.off_axis(), 3),
                         round(1000 * float(np.linalg.norm(linear)), 1), round(float(np.linalg.norm(angular)), 2)])  # fmt: skip
        key = f"tilted {tilt:g} deg: step, taken, cut free, crests (mm), down (mm), tilt (deg), off (mm), speed (mm/s), turning (rad/s)"
        found[key] = rows
        print(f"TRACE {key}: {json.dumps(rows)}", flush=True)
    return found


def press(embodiment, out: Path) -> dict:
    m = embodiment.task.module
    mk = m.mk
    found: dict = {}
    hz = float(embodiment.info.control_hz)

    weight = m.HORN_MASS * 9.81

    def heavy(name: str) -> None:  # the horn at its own mass: resting on the crests it is held by joints, and a press goes through them
        fresh(embodiment, m, name)

    def pressed(angle: float, newtons: float, seconds: float = 2.0) -> dict:
        rows = []
        for k in range(int(round(seconds * hz))):
            press_along_axis(m, newtons - weight)
            embodiment.step(_hold())
            if k % int(hz / 4) == int(hz / 4) - 1:
                rows.append([round((k + 1) / hz, 2), round(1000 * m.down(), 3), round(math.degrees(m.arm_angle()), 2), m.engaged()])
        held = reading(m)
        m.push_horn()
        settle(embodiment, 1.0)
        return {"while pressed, every 0.25 s: t, down (mm), arm (deg), engaged": rows, "at the end of the press": held, "a second after": reading(m)}

    for angle_deg, forces in ((0.0, (20.0, 30.0, 31.0, 34.0, 40.0)), (3.0, (40.0, 60.0))):
        for newtons in forces:
            heavy(f"press {angle_deg} {newtons}")
            put_horn(m, math.radians(angle_deg), over=0.0002)
            settle(embodiment, 0.5)
            key = f"arm {angle_deg:g} deg from the mark, {newtons:g} N"
            found[key] = pressed(math.radians(angle_deg), newtons)
            print(f"PRESS {key}: {json.dumps(found[key])}", flush=True)
    for pull in (8.0, 12.0):
        heavy(f"pull {pull}")
        put_horn(m, 0.0, over=0.0002)
        settle(embodiment, 0.5)
        home = pressed(0.0, 40.0, 1.0)
        rows = []
        for k in range(int(round(2.0 * hz))):
            if m.STATE["freed"]:
                m.push_horn()  # off the crests: the pull ends there (a hand would stop pulling too)
            else:
                press_along_axis(m, -pull - weight)
            embodiment.step(_hold())
            if k % int(hz / 4) == int(hz / 4) - 1:
                rows.append([round((k + 1) / hz, 2), round(1000 * m.down(), 3), m.engaged(), m.captured()])
        key = f"pressed home, then pulled up with {pull:g} N"
        found[key] = {"home": home["a second after"], "while pulled, every 0.25 s: t, down (mm), engaged, on the crests": rows, "at the end": reading(m), "pulled off": m.STATE["freed"]}
        m.push_horn()
        print(f"PULL {key}: {json.dumps(found[key])}", flush=True)
    for home in (False, True):
        if home:
            heavy(f"twist {home}")
        else:
            fresh(embodiment, m, f"twist {home}")  # the horn's own 4 g: free to turn on the crests
        put_horn(m, 0.0, over=0.0002)
        settle(embodiment, 0.5)
        if home:
            pressed(0.0, 40.0, 1.0)
        before = math.degrees(m.arm_angle())
        position, rotation = mk.pose("horn")
        mk.write_pose("horn", position, mk.matrix_to_quat(rotation), ang_vel=(0.0, 0.0, 2.0))
        settle(embodiment, 1.0)
        key = f"set turning at 2 rad/s about the shaft, then 1 s, {'pressed home' if home else 'resting on the crests'}"
        found[key] = {"arm before (deg)": round(before, 2), "after": reading(m)}
        print(f"TWIST {key}: {json.dumps(found[key])}", flush=True)
    return found


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    known = {"rest": rest, "capture": capture, "press": press, "drop_trace": drop_trace}
    trials = sys.argv[2].split(",") if len(sys.argv) > 2 else ["rest"]
    for arg in sys.argv[3:]:  # read by the task file when it is loaded: servo=x,y horn=x,y shaft=near|far
        key, value = arg.split("=", 1)
        os.environ[{"servo": "HORN_SERVO_XY", "horn": "HORN_XY", "shaft": "HORN_SHAFT_END"}[key]] = value
    if any(name not in known for name in trials):
        raise SystemExit(f"trials here: {sorted(known)}; got {trials}")
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
