#!/usr/bin/env python3
"""Checks of the grab bar task in the simulator, no arm (task.py).

    scripts/rta simpy <id> mech/tasks/grab_bar/trials.py /workspace/robocurve/outputs/<id> [rest,unheld,setdown,held,pull,twoseals] [seed=N]

``rest``  the first pictures (top camera and both wrist cameras, as rendered and at the size a policy gets) and 3 s
          with nobody touching anything: how far every body moves, the levers' angles, the seals, the score.
``unheld`` (a) the left lever lifted over centre with nobody holding: the seal goes and the bar drops off the wall
           onto the bench (the fall rule: 0).
``setdown`` the trial's hand (pushes at the bar's middle of mass) holds the bar, both levers lifted, lowers it 3 mm off
           the wall onto the bench at 30 mm/s, leans it there on the tiles, lets it rest, lifts it, moves it 60 mm
           towards -y and locks both cups pressed on: set down gently, the score is the remount's.
``held``   (b) the bar held by its left cup: the right lever lifted, then pushed down with a rising push while the
           right cup is pressed on across the joint: the push at which it passes centre, against LEVER_N; no seal.
``pull``   (c) the sealed bar pulled out with HOLD_N: it stays; the seal cut and HOLD_JOINT_N: it comes away.
``twoseals`` both cups sealed, the bar left alone: the bodies' speeds against the rest rule.
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


def rest(embodiment, out: Path, seed: int) -> dict:
    from inspect_robots import Scene

    m = embodiment.task.module
    mk = m.mk
    observation = embodiment.reset(Scene(id="rest", instruction=m.INSTRUCTION), seed=seed)
    for camera, name in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        _save(out, name, np.asarray(observation.images[camera]))
    result = embodiment.step(_hold())
    for camera, name in (("top_cam", "step1_top"), ("left_cam", "step1_left_wrist"), ("right_cam", "step1_right_wrist")):
        _save(out, name, np.asarray(result.observation.images[camera]))
    names = list(mk.MECH.bodies)
    at = {name: mk.pose(name)[0].copy() for name in names}
    hz = float(embodiment.info.control_hz)
    for _ in range(int(round(3.0 * hz))):
        embodiment.step(_hold())
    moved = {name: round(1000 * float(np.linalg.norm(mk.pose(name)[0] - at[name])), 3) for name in names}
    found = {"seed": seed, "moved over 3 s (mm)": moved, "levers deg from released": {s: round(math.degrees(m.LEVERS[s].theta()), 2) for s in m.SIDES},
             "sealed": {s: m.sealed(s) for s in m.SIDES}, "press N": {s: round(mk.contact_force(f"cup_{s}", "panel"), 3) for s in m.SIDES},
             "score": m.score(None), "report": m.report()}  # fmt: skip
    print(f"REST {json.dumps(found, default=str)}", flush=True)
    return found


def _com(mk, name: str) -> np.ndarray:
    """A free body's middle of mass (world, less the environment's origin)."""
    env = mk.MECH.environment.unwrapped
    data = env.scene.rigid_objects[name].data
    return data.root_com_pos_w.torch[0].detach().cpu().numpy().astype(float) - env.scene.env_origins[0].detach().cpu().numpy().astype(float)


def _lever(m, side: str) -> tuple[np.ndarray, np.ndarray]:
    """Lever ``side``'s tip (world) and the direction its tip moves when the lever turns towards down."""
    import parts as P

    p, r = m.mk.pose(f"lever_{side}")
    u = r @ np.array([1.0, 0.0, 0.0])  # pivot to tip (shapes.lever)
    axis = m.mk.pose("bar")[1] @ np.array(m.LEVER_AXES[side])  # the hinge's axis, riding on the bar
    return p + P.LEVER_ARM * u, np.cross(axis, u)


def _push_tip(m, side: str, force: float, direction) -> None:
    """``force`` N at lever ``side``'s tip along ``direction`` (world), as a load on the lever (force at its middle of
    mass and the torque of the tip's lever arm), until the next call."""
    tip, _ = _lever(m, side)
    f = force * np.asarray(direction, dtype=float)
    m.mk.push(f"lever_{side}", force=tuple(f), torque=tuple(np.cross(tip - _com(m.mk, f"lever_{side}"), f)))


def _turn_lever(m, side: str, force: float) -> None:
    """A torque on lever ``side`` about its hinge's axis: ``force`` N at its tip, positive towards down (a pure
    torque, so that it does not hang on where the lever's middle of mass is read)."""
    import parts as P

    axis = m.mk.pose("bar")[1] @ np.array(m.LEVER_AXES[side])
    m.mk.push(f"lever_{side}", torque=tuple(force * P.LEVER_ARM * axis))


def _bar_off(m) -> dict:
    """mm: how far each cup's face stands off the tiles (out from the wall) and how far the bar has sunk."""
    import parts as P

    off = {s: round(1000 * (P.PANEL_X - float(m.cup_frame(s)[0][0])), 2) for s in m.SIDES}
    return {"cups off the tile mm": off, "bar sunk mm": round(1000 * float(m.mk.built_pose("bar")[0][2] - m.mk.pose("bar")[0][2]), 2)}


def unheld(embodiment, out: Path, seed: int) -> dict:
    """(a) The start, nobody holding the bar: the left lever's tip pushed towards up with 1.5 x LEVER_N until it passes
    centre, then let be. The seal goes, the lever springs up, and the bar comes off the wall and drops onto the bench:
    the fall rule (task.check_fall) reads the drop and the score is 0."""
    from inspect_robots import Scene

    import parts as P

    m = embodiment.task.module
    embodiment.reset(Scene(id="unheld", instruction=m.INSTRUCTION), seed=seed)
    embodiment.step(_hold())  # the bodies are read from the first control step on
    hz = float(embodiment.info.control_hz)
    rows, passed = [], None
    for i in range(int(round(3.0 * hz))):
        t = (i + 1) / hz
        if passed is None:
            _, down = _lever(m, "l")
            _push_tip(m, "l", 1.5 * P.LEVER_N, -down)
        else:
            m.mk.push("lever_l")
        embodiment.step(_hold())
        theta = math.degrees(m.LEVERS["l"].theta())
        if passed is None and theta < P.LEVER_CENTRE_DEG:
            passed = t
        rows.append({"t": round(t, 2), "lever deg": round(theta, 1), "sealed": m.sealed("l"), **_bar_off(m)})
    found = {"passed centre at s": passed, "rows (every 0.25 s)": rows[4::5], "fell at s": m.STATE["fell"], "score": m.score(None), "events": m.report()["events"],
             "rule": {"DROP_M mm": 1000 * m.DROP_M, "STRIKE_MPS": round(m.STRIKE_MPS, 3)}}  # fmt: skip
    print(f"UNHELD {json.dumps(found, default=str)}", flush=True)
    return found


SPRING, DAMP = 10.0, 3.3  # N/m, N s/m: the trial's hand pulls the bar's origin towards its target (critically damped at 20 Hz)
TURN_K, TURN_C = 0.1, 0.006  # N m/rad, N m s: and keeps the bar turned as built


def _hand(m, target, speed=(0.0, 0.0, 0.0), lift=1.0, press=0.0, at_b=None) -> None:
    """The trial's hand on the bar until the next call: ``lift`` times the whole bar's weight up at its middle of mass
    (bar, cups and levers), a spring-damper pulling the bar's origin towards ``target`` (world) moving at ``speed``,
    ``press`` N into the wall at ``at_b`` (B; None: the middle of mass) and a torque keeping it turned as built."""
    import parts as P

    mk = m.mk
    masses = {"bar": P.BAR_MASS - 2 * P.CUP_MASS - 2 * P.LEVER_MASS, "cup_l": P.CUP_MASS, "cup_r": P.CUP_MASS, "lever_l": P.LEVER_MASS, "lever_r": P.LEVER_MASS}
    com = sum(mass * _com(mk, n) for n, mass in masses.items()) / sum(masses.values())
    at = _com(mk, "bar")
    p, r = mk.pose("bar")
    v, w = mk.velocity("bar")
    f = np.array([0.0, 0.0, lift * P.BAR_MASS * 9.81]) + SPRING * (np.asarray(target, dtype=float) - p) + DAMP * (np.asarray(speed, dtype=float) - v)
    torque = np.cross(com - at, f)
    if press:
        fp = np.array([press, 0.0, 0.0])
        torque += np.cross((com if at_b is None else p + r @ np.asarray(at_b, dtype=float)) - at, fp)
        f = f + fp
    e = mk.built_pose("bar")[1] @ r.T  # the turn back to as built
    torque += TURN_K * 0.5 * np.array([e[2, 1] - e[1, 2], e[0, 2] - e[2, 0], e[1, 0] - e[0, 1]]) - TURN_C * w
    mk.push("bar", force=tuple(f), torque=tuple(torque))


def setdown(embodiment, out: Path, seed: int) -> dict:
    """Set down gently and put back: the trial's hand (``_hand``) holds the bar pressed on (30 N), both levers are
    lifted over centre (a pull rising 1 N a step from 10 N at the tip; with 15 N on and 33 N pulled at once, the right
    lever dragged the bar 209 mm along the wall: gb5t9), the bar is taken 3 mm off the wall and lowered at 30 mm/s until its cups' rims stand on the
    bench, leaned on the tiles (4 N) and left there 1 s with nothing carrying it, lifted, moved 60 mm towards -y, put
    back on the wall at 30 mm/s, pressed on with 30 N while each lever is turned down with a torque rising to that of
    30 N at its tip, and let go.
    The fall rule must read a gentle landing (no fall), the seals must be made, and the end must score 1.0."""
    from inspect_robots import Scene

    import parts as P

    m = embodiment.task.module
    mk = m.mk
    embodiment.reset(Scene(id="setdown", instruction=m.INSTRUCTION), seed=seed)
    embodiment.step(_hold())
    hz = float(embodiment.info.control_hz)
    p0 = mk.built_pose("bar")[0]
    off, low = np.array([-0.003, 0.0, 0.0]), np.array([0.0, 0.0, -(P.BAR_Z - P.CUP_R) - 0.002])  # 3 mm off the wall; rims 2 mm into the bench
    rows = []

    def run(seconds, start, end=None, label="", **kw):
        """The hand's target from ``start`` to ``end`` (world) at V, then held there, for ``seconds``."""
        end = start if end is None else end
        n, span, most = int(round(seconds * hz)), float(np.linalg.norm(end - start)), 0.0
        for i in range(n):
            u = min(1.0, (i / hz) * V / span) if span else 1.0
            target = start + u * (end - start)
            _hand(m, target, (end - start) / span * V if span and u < 1.0 else np.zeros(3), **kw)
            yield i
            embodiment.step(_hold())
            most = max(most, float(np.linalg.norm(mk.pose("bar")[0] - target)))
        rows.append({"phase": label, "t": round(mk.now(), 2), "most off the hand's target mm": round(1000 * most, 1), "bar from its start mm": [round(1000 * float(x), 1) for x in mk.pose("bar")[0] - p0],
                     "lowest over the bench mm": round(1000 * m.lowest_z(), 1), "sealed": {s: m.sealed(s) for s in m.SIDES}, "fell at s": m.STATE["fell"],
                     "levers deg": {s: round(math.degrees(m.LEVERS[s].theta()), 1) for s in m.SIDES}, "score": m.score(None)})  # fmt: skip

    V = 0.030  # m/s: the hand's moves
    for _ in run(0.5, p0, press=30.0, label="held on the wall"):
        pass
    for side in m.SIDES:  # both levers lifted over centre, the hand pressing the bar on with 30 N; the pull at the tip rises 1 N a step from 10 N
        lifted = False
        for i in run(1.5, p0, press=30.0, label=f"{side} lever lifted"):
            lifted = lifted or m.LEVERS[side].theta() < P.CENTRE_RAD
            if lifted:
                m.mk.push(f"lever_{side}")
            else:
                _, down = _lever(m, side)
                _push_tip(m, side, min(1.5 * P.LEVER_N, 10.0 + i), -down)
        m.mk.push(f"lever_{side}")
    for _ in run(0.5, p0, p0 + off, label="off the wall"):
        pass
    for _ in run(1.5, p0 + off, p0 + off + low, label="lowered onto the bench"):
        pass
    for _ in run(0.5, p0 + off + low, p0 + low, label="back to the tiles"):
        pass
    for _ in run(0.5, p0 + low, press=4.0, label="leaned on the tiles"):
        pass
    for _ in run(1.0, p0 + low, lift=0.0, press=4.0, label="set down, resting"):
        pass
    for _ in run(0.5, p0 + low, p0 + off + low, label="taken up, off the wall"):
        pass
    for _ in run(1.5, p0 + off + low, p0 + off, label="lifted"):
        pass
    for _ in run(P.MOVE / V + 0.5, p0 + off, p0 + off + np.array([0.0, -P.MOVE, 0.0]), label="moved"):
        pass
    moved = p0 + np.array([0.0, -P.MOVE, 0.0])
    for _ in run(0.5, moved + off, moved, label="back on the wall"):
        pass
    for _ in run(0.5, moved, press=30.0, label="pressed on"):
        pass
    passed = {}
    for side in m.SIDES:
        ramp = int(round(3.0 * hz))
        for i in run(4.0, moved, press=30.0, label=f"{side} lever turned down"):
            if side not in passed and m.LEVERS[side].theta() > P.CENTRE_RAD:
                passed[side] = round(30.0 * min(1.0, i / ramp), 1)
            if side in passed:
                m.mk.push(f"lever_{side}")
            else:
                _turn_lever(m, side, 30.0 * min(1.0, (i + 1) / ramp))
        m.mk.push(f"lever_{side}")
    mk.push("bar")
    scores = []
    for _ in range(int(round(3.0 * hz))):
        embodiment.step(_hold())
        scores.append(m.score(None))
    found = {"phases": rows, "levers passed centre at tip N": passed, "score at the end": scores[-1], "1.0 from s": next((round(mk.now() - (len(scores) - i - 1) / hz, 2) for i, sc in enumerate(scores) if sc == 1.0), None),
             "fell at s": m.STATE["fell"], "report": m.report()}  # fmt: skip
    print(f"SETDOWN {json.dumps(found, default=str)}", flush=True)
    return found


def held(embodiment, out: Path, seed: int) -> dict:
    """(b) The bar held by its left cup. The right lever is lifted over centre (1.5 x LEVER_N towards up) and springs
    up; then a torque about its hinge rises from 0 to that of 30 N at its tip in 3 s while PRESS N push the bar on
    at the right cup, as the demo's presser does: the push at which it passes centre (to set against LEVER_N), its
    angle against the push, and the right cup (across the joint, pressed) left unsealed, the joint the one reason
    in the events. Check run gb5t3 pushed with a force at the middle of mass as read and crossed at 14 N: the
    reading is logged here."""
    PRESS = 20.0  # N towards the wall at the right cup
    from inspect_robots import Scene

    import parts as P

    m = embodiment.task.module
    embodiment.reset(Scene(id="held", instruction=m.INSTRUCTION), seed=seed)
    embodiment.step(_hold())  # the bodies are read from the first control step on
    hz = float(embodiment.info.control_hz)
    up_at = None
    for i in range(int(round(2.0 * hz))):  # lift it over centre, then let it spring up
        if up_at is None:
            _, down = _lever(m, "r")
            _push_tip(m, "r", 1.5 * P.LEVER_N, -down)
        else:
            m.mk.push("lever_r")
        embodiment.step(_hold())
        if up_at is None and math.degrees(m.LEVERS["r"].theta()) < P.LEVER_CENTRE_DEG:
            up_at = (i + 1) / hz
    m.mk.push("lever_r")
    rest_up = math.degrees(m.LEVERS["r"].theta())
    rows, passed = [], None
    ramp = int(round(3.0 * hz))
    for i in range(ramp + int(round(1.0 * hz))):
        force = 30.0 * min(1.0, (i + 1) / ramp) if passed is None else 0.0
        if force > 0.0:
            _turn_lever(m, "r", force)
        else:
            m.mk.push("lever_r")
        p, r = m.mk.pose("bar")
        at, f = p + r @ np.array([P.GRIP_X, P.CUP_Y["r"], 0.0]), np.array([PRESS, 0.0, 0.0])
        m.mk.push("bar", force=tuple(f), torque=tuple(np.cross(at - _com(m.mk, "bar"), f)))
        embodiment.step(_hold())
        theta = math.degrees(m.LEVERS["r"].theta())
        rows.append({"push N": round(force, 2), "lever deg": round(theta, 2), "right cup press N": round(m.mk.contact_force("cup_r", "panel"), 2)})
        if passed is None and theta > P.LEVER_CENTRE_DEG:
            passed = rows[-1]
    m.mk.push("bar")
    com_off = float(np.linalg.norm(_com(m.mk, "lever_r") - m.mk.pose("lever_r")[0]))
    found = {"lifted over centre at s": up_at, "lever deg after springing up": round(rest_up, 2), "passed centre (push N, deg)": passed, "LEVER_N": P.LEVER_N,
             "the lever's middle of mass as read, from its pivot mm (built: half its length)": [round(1000 * com_off, 2), round(1000 * P.LEVER_L / 2, 2)],
             "lever deg at the end": round(math.degrees(m.LEVERS["r"].theta()), 2), "right sealed": m.sealed("r"), "left sealed": m.sealed("l"),
             "push against angle (every 0.25 s)": rows[4::5], "bar moved mm": round(1000 * float(np.linalg.norm(m.mk.pose("bar")[0] - m.mk.built_pose("bar")[0])), 3),
             "events": m.report()["events"]}  # fmt: skip
    print(f"HELD {json.dumps(found, default=str)}", flush=True)
    return found


def pull(embodiment, out: Path, seed: int) -> dict:
    """(c) The bar sealed by its left cup, pulled straight out from the wall at its grip with HOLD_N for 1 s: how far it
    moves. Then the seal cut (as a lever lifted would) and 3 N out (HOLD_JOINT_N): it holds nothing."""
    from inspect_robots import Scene

    import parts as P

    m = embodiment.task.module
    mk = m.mk
    embodiment.reset(Scene(id="pull", instruction=m.INSTRUCTION), seed=seed)
    embodiment.step(_hold())  # the bodies are read from the first control step on
    hz = float(embodiment.info.control_hz)

    def pull_for(force: float, seconds: float) -> dict:
        start = mk.pose("bar")[0].copy()
        for _ in range(int(round(seconds * hz))):
            p, r = mk.pose("bar")
            grip = p + r @ np.array([P.GRIP_X, 0.0, 0.0])
            f = np.array([-force, 0.0, 0.0])
            mk.push("bar", force=tuple(f), torque=tuple(np.cross(grip - _com(mk, "bar"), f)))
            embodiment.step(_hold())
        mk.push("bar")
        return {"pull N": force, "bar moved mm": round(1000 * float(np.linalg.norm(mk.pose("bar")[0] - start)), 3), **_bar_off(m)}

    sealed = pull_for(P.HOLD_N, 1.0)
    mk.cut("seal_l")
    loose = pull_for(P.HOLD_JOINT_N, 0.5)
    found = {"sealed (left cup), HOLD_N out": sealed, "seal cut, HOLD_JOINT_N out": loose, "fell at s": m.STATE["fell"], "events": m.report()["events"]}
    print(f"PULL {json.dumps(found, default=str)}", flush=True)
    return found


def _speeds(m) -> dict:
    """Each body's speed (mm/s, deg/s) against the task's rest limits (10 mm/s, 5.7 deg/s)."""
    names = ("bar", "cup_l", "cup_r", "lever_l", "lever_r")
    return {n: [round(1000 * float(np.linalg.norm(m.mk.velocity(n)[0])), 2), round(math.degrees(float(np.linalg.norm(m.mk.velocity(n)[1]))), 2)] for n in names}


def twoseals(embodiment, out: Path, seed: int) -> dict:
    """Both cups sealed (the right one made directly, the start's joint rule aside), the bar then left alone: is
    every body at rest (the task's rule for the end)? Twice: the right seal made with the bar unloaded, and made
    while 15 N presses the right cup on (as a squeezing hand does), the push then taken away. The demo's check run
    gb5d5 ended with both seals made and the bodies not at rest."""
    from inspect_robots import Scene

    import parts as P

    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    found = {}
    for case, load in (("unloaded", 0.0), ("made under 15 N", 15.0)):
        embodiment.reset(Scene(id="twoseals", instruction=m.INSTRUCTION), seed=seed)
        embodiment.step(_hold())
        for _ in range(int(round(0.5 * hz))):
            if load:
                p, r = mk.pose("bar")
                at = p + r @ np.array([P.GRIP_X, P.CUP_Y["r"], 0.0])
                f = np.array([load, 0.0, 0.0])
                mk.push("bar", force=tuple(f), torque=tuple(np.cross(at - _com(mk, "bar"), f)))
            embodiment.step(_hold())
        mk.make("seal_r")
        embodiment.step(_hold())
        mk.push("bar")
        rows = []
        for i in range(int(round(3.0 * hz))):
            embodiment.step(_hold())
            if i % 5 == 4:
                rows.append({"t": round((i + 1) / hz, 2), "at rest": m.at_rest(), "speeds (mm/s, deg/s)": _speeds(m)})
        found[case] = {"rows": rows, "at rest at the end": m.at_rest(), "sealed": {s: m.sealed(s) for s in m.SIDES}}
    print(f"TWOSEALS {json.dumps(found, default=str)}", flush=True)
    return found


TRIALS = {"rest": rest, "unheld": unheld, "setdown": setdown, "held": held, "pull": pull, "twoseals": twoseals}


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    trials = sys.argv[2].split(",") if len(sys.argv) > 2 else ["rest"]
    seed = 0
    for arg in sys.argv[3:]:
        key, value = arg.split("=", 1)
        if key != "seed":
            raise SystemExit(f"unknown argument {arg!r}")
        seed = int(value)
    if any(name not in TRIALS for name in trials):
        raise SystemExit(f"trials here: {sorted(TRIALS)}; got {trials}")
    out.mkdir(parents=True, exist_ok=True)
    embodiment = kit.open_task(HERE / "task.py")
    failed = 0
    for name in trials:
        try:
            found = TRIALS[name](embodiment, out, seed)
        except Exception as error:
            import traceback

            traceback.print_exc()
            print(f"TRIAL {name} FAILED: {error!r}", flush=True)
            failed += 1
            continue
        (out / f"{name}.json").write_text(json.dumps(found, indent=1, default=str))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
