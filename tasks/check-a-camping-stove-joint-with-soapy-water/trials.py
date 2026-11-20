#!/usr/bin/env python3
"""Checks of the stove check with a known load, and of what its cameras see:

    scripts/rta simpy <id> mech/tasks/stove_check/trials.py /workspace/robocurve/outputs/<id> stop_gauge,hold_gauge,froth

``stop_gauge``: what does the sprung stop at home read when the torque on the stove is known? The stove is screwed
home (seed 0) and no arm moves. A torque about the vertical is put on it towards the stop, and the opposite one on
the cartridge, in steps of 2 s from 0 to 4 N m and down again. Each step prints how far the stove stands past home
and what the task takes that for; all of them go to <out dir>/stop_gauge.json.

Run n2q (2026-11-18, stop of 10 N m/rad): 0.2 / 0.5 / 1.0 / 2.0 / 3.0 / 4.0 N m were read as 0.152 / 0.454 / 0.954 /
1.954 / 2.954 / 3.954 N m, with the stove 0.87 / 2.60 / 5.47 / 11.20 / 16.93 / 22.66 degrees past home. That is
the torque less the thread's friction of 0.05 N m, within 0.004 N m: the stop reads true, and the task takes its
reading as it is. (Run n2h of 2026-11-17 had found the same for the earlier stop of 50 N m/rad: 0.157 / 0.455 /
0.954 N m for 0.2 / 0.5 / 1.0.)

``hold_gauge``: what does the left hand's hold on the cartridge stand by itself? Run n2t (2026-11-18): pressed
with 20 N, the cartridge stood a torque of 0.2 N m and let go at 0.4; pressed with 30 N it stood 0.4 and let go at
0.6. (With the other hand on the stove, which keeps the cartridge's axis in place, the stop could be loaded to
2.1 N m before the cartridge turned with the stove: demo.py's ``force`` variation, runs n2r and n2u.)

``froth``: can froth be seen (see the function).

(What the hands can and cannot do to the free cartridge is shown by demo.py's variations, ``loose:no_steady`` among
them.)
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))  # demo_kit, look

TORQUES = (0.0, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 3.0, 4.0, 2.0, 0.5, 0.0)  # N m, 2 s each
HOME_SEED = 0


def stop_gauge(embodiment, out: Path) -> list[dict]:
    import torch
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    m = embodiment.task.module
    mk = m.mk
    embodiment.reset(Scene(id="gauge", instruction=m.INSTRUCTION), seed=HOME_SEED)
    objects = mk.MECH.environment.unwrapped.scene.rigid_objects
    hold = Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))
    rows = []
    for torque in TORQUES:
        for name, sign in (("stove", -1.0), ("cartridge", 1.0)):  # towards the stop on the stove, the reaction on the cartridge
            none = torch.zeros((1, 1, 3), device=objects[name].device)
            turn = none.clone()
            turn[0, 0, 2] = sign * torque
            objects[name].permanent_wrench_composer.set_forces_and_torques_index(forces=none, torques=turn, is_global=True)
        for _ in range(40):
            embodiment.step(hold)
        past = -mk.joint("thread")
        rotation = mk.pose("cartridge")[1]
        rows.append({
            "torque (N m)": torque,
            "past home (deg)": round(math.degrees(past), 3),
            "stop reads (N m)": round(-mk.stop_load("thread"), 3),
            "cartridge yaw (deg)": round(math.degrees(math.atan2(rotation[1, 0], rotation[0, 0])), 2),
        })  # fmt: skip
        print(f"GAUGE {rows[-1]}", flush=True)
    out.mkdir(parents=True, exist_ok=True)
    (out / "stop_gauge.json").write_text(json.dumps(rows, indent=1))
    return rows


WET_AS_STROKED = (30, 230)  # degrees round the joint, anticlockwise from straight ahead: what demo.py's five strokes wet


def froth(embodiment, out: Path) -> dict:
    """Can froth be seen? No arm moves and no brush is used: the joint line is DECLARED wet on the side the
    scripted strokes wet, in both hidden cases, for 6 s. Then the top camera's picture is saved as rendered and
    as a policy gets it, and the near-white pixels on the cartridge's top are counted (look.py). The stretches of
    the joint line are counted on the cartridge, which is turned back a third of a turn in the leaking case: the
    same side of the table is other stretches there."""
    import cv2
    import demo_kit as kit
    import look
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    m = embodiment.task.module
    mk = m.mk
    camera = embodiment.rig.top_cam
    hold = Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))
    out.mkdir(parents=True, exist_ok=True)
    result, small = {}, {}
    for case, seed in (("leaking", 65536), ("sound", HOME_SEED)):
        embodiment.reset(Scene(id="froth", instruction=m.INSTRUCTION), seed=seed)
        for _ in range(120):
            turned = mk.pose("cartridge")[1] if mk.MECH.poses else mk.quat_to_matrix(mk.yaw_quat(-m.LOOSE if m.STATE["loose"] else 0.0))
            yaw = math.degrees(math.atan2(turned[1, 0], turned[0, 0]))
            wet = sorted({int(round((angle - yaw) / (360 / m.SECTORS))) % m.SECTORS for angle in range(WET_AS_STROKED[0], WET_AS_STROKED[1] + 1, 5)})
            m.STATE["sector_t"][wet] = mk.now()
            step = embodiment.step(hold)
        frame = step.observation.images["top_cam"]
        small[case] = kit.policy_picture(frame)
        cv2.imwrite(str(out / f"froth_{case}.png"), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(out / f"froth_{case}_policy.png"), cv2.cvtColor(small[case], cv2.COLOR_RGB2BGR))
        top = look.cartridge_top(m, camera, small[case].shape[:2])
        result[case] = {"balls on show": m.report()["bubbles"], "pixels of the cartridge's top": int(top.sum()),
                        "near-white by floor": {floor: int((look.near_white(small[case], floor) & top).sum()) for floor in (110, 130, 150, 170, 190, 210, 230)}}  # fmt: skip
        print(f"FROTH {case}: {result[case]}", flush=True)
    top = look.cartridge_top(m, camera, small["leaking"].shape[:2])
    changed = top & (np.abs(small["leaking"].astype(int) - small["sound"].astype(int)).max(axis=-1) > 40)
    if changed.any():
        result["where the two pictures differ on the cartridge"] = {
            "pixels": int(changed.sum()),
            "leaking: lowest of R, G, B (10th, 50th, 90th percentile)": [int(v) for v in np.percentile(small["leaking"][changed].min(axis=-1), (10, 50, 90))],
            "sound: mean R, G, B": [int(v) for v in small["sound"][changed].mean(axis=0)],
        }  # fmt: skip
    result["more near-white with froth (floor of look.py)"] = look.white_on_cartridge(m, camera, small["leaking"]) - look.white_on_cartridge(m, camera, small["sound"])
    print(f"FROTH {result}", flush=True)
    (out / "froth.json").write_text(json.dumps(result, indent=1))
    return result


HOLD_PRESS = (20.0, 30.0)  # N on the two fingertips
HOLD_TORQUES = tuple(round(0.2 * k, 1) for k in range(1, 26))  # N m about the cartridge's axis, 1 s each
HOLD_SLIP = 1.0  # degrees of yaw within one step of the torque: the hold has let go


def hold_gauge(embodiment, out: Path) -> dict:
    """What does the left hand's hold on the cartridge stand? The stove is home and nobody touches it. The left
    hand sets its two fingertips on the cartridge's shoulder and presses straight down, exactly as demo.py's
    ``press`` does, with 20 N and then (a new trial) with 30 N. Then a torque about the vertical is put on the
    cartridge alone, 0.2 N m more every second, until its yaw has moved more than a degree within one step of
    the torque. The stove rides along on its thread (0.05 N m of friction), so the torque that turns the
    cartridge under the fingertips is the one applied."""
    import demo_kit as kit
    import torch
    from demo_kit import Move, about, down
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import station
    from yam_arena.constants import READY

    m = embodiment.task.module
    mk = m.mk
    up = np.array([0.0, 0.0, 1.0])
    phi, lean = math.radians(150), math.radians(50)
    outward = np.array([math.cos(phi), math.sin(phi), 0.0])
    rot = about(np.cross(up, -outward), -lean) @ down(phi + math.pi)
    tips = kit.lowest_point(rot)
    touch = np.array([*m.CART_XY, m.TOP + m.CART_H - m.CUP_H]) + 0.024 * outward - rot @ tips
    back = touch - 0.07 * rot[:, 2]
    result = {}
    for newtons in HOLD_PRESS:
        embodiment.reset(Scene(id="hold", instruction=m.INSTRUCTION), seed=HOME_SEED)
        objects = mk.MECH.environment.unwrapped.scene.rigid_objects
        arm = kit.Arm("left", station.BASES["left"], [
            Move(1.0, grip=50.0 / 94.0, label="half open"),
            Move(3.0, pos=back, rot=rot, label="by shoulder"),
            Move(2.0, pos=touch + 0.001 * up, label="touch"),
            Move(1.5, push=(0.0, 0.0, -newtons), push_at=tips, label="press"),
            Move(1.0 + len(HOLD_TORQUES), label="hold press"),
        ])  # fmt: skip
        hz = float(embodiment.info.control_hz)
        pressed = int(round(8.5 * hz))  # the press has stood for a second
        rows, let_go = [], None
        yaw = lambda: math.degrees(math.atan2(*mk.pose("cartridge")[1][[1, 0], 0]))  # noqa: E731
        for i in range(pressed + int(len(HOLD_TORQUES) * hz)):
            k = (i - pressed) // int(hz)
            torque = HOLD_TORQUES[k] if i >= pressed else 0.0
            none = torch.zeros((1, 1, 3), device=objects["cartridge"].device)
            turn = none.clone()
            turn[0, 0, 2] = -torque  # clockwise from above, the way the stove is screwed down
            objects["cartridge"].permanent_wrench_composer.set_forces_and_torques_index(forces=none, torques=turn, is_global=True)
            if i >= pressed and (i - pressed) % int(hz) == 0:
                start = yaw()
            action, _ = arm.action(i / hz)
            embodiment.step(Action(data=pack(action, np.r_[READY, 1.0])))
            if i >= pressed and (i - pressed) % int(hz) == int(hz) - 1:
                moved = yaw() - start
                place = mk.pose("cartridge")[0]
                rows.append({"torque (N m)": torque, "yaw moved in the step (deg)": round(moved, 2), "yaw (deg)": round(yaw(), 2),
                             "from its place (mm)": round(1000 * float(np.linalg.norm(place[:2] - np.array(m.CART_XY))), 2)})  # fmt: skip
                print(f"HOLD {newtons:.0f} N: {rows[-1]}", flush=True)
                if abs(moved) > HOLD_SLIP:
                    let_go = torque
                    break
        result[f"{newtons:.0f} N"] = {"let go at (N m)": let_go, "stood (N m)": rows[-2]["torque (N m)"] if let_go is not None and len(rows) > 1 else (None if let_go is not None else rows[-1]["torque (N m)"]), "steps": rows}
        none = torch.zeros((1, 1, 3), device=objects["cartridge"].device)
        objects["cartridge"].permanent_wrench_composer.set_forces_and_torques_index(forces=none, torques=none, is_global=True)
    print(f"HOLD {json.dumps({k: {kk: vv for kk, vv in v.items() if kk != 'steps'} for k, v in result.items()})}", flush=True)
    out.mkdir(parents=True, exist_ok=True)
    (out / "hold_gauge.json").write_text(json.dumps(result, indent=1))
    return result


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    trials = sys.argv[2].split(",") if len(sys.argv) > 2 else ["stop_gauge"]
    known = {"stop_gauge": stop_gauge, "froth": froth, "hold_gauge": hold_gauge}
    if not trials or any(name not in known for name in trials):
        raise SystemExit(f"trials here: {sorted(known)}; got {trials}")
    embodiment = kit.open_task(HERE / "task.py")
    for name in trials:
        known[name](embodiment, out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
