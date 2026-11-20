#!/usr/bin/env python3
"""Full-score demonstration of the stove check, scripted and open loop, for both hidden cases, and the runs that
show what each rule costs.

    scripts/rta demo <id> mech/tasks/stove_check [run,run,...] [--dry]

A run is ``loose`` or ``home`` (the hidden case: the stove a third of a turn short of home, or home), optionally
followed by ``:`` and one of the variations below. Each writes a folder with result.json, one video per camera,
setup.png and contact sheets (mech/demo_kit.py). ``--dry``: no simulator, the plan through the IK only.

The plan comes from the layout's constants alone. It decides once, from two pictures of the top camera at the
size a policy gets (320 x 176): one just before the strokes and one 5 s after them, the brush held aside in the
same place for both. Froth: 15 or more near-white pixels more on the cartridge's top in the second (look.py). It
never reads the thread's angle, the task's state or where the simulator has put the froth; what the simulator
knows is written beside the decision for the record only.

The left arm takes the brush out of the pot, slanted as it stands, turns it to point across the table and strokes
the joint line at five places on the side away from the flame control; if froth shows, it puts the brush back
(which wets it again). The right arm pinches the folded stove from above; the left hand, half open, sets both
fingertips on the cartridge's shoulder and presses straight down; the right turns the stove 122 degrees clockwise,
onto its stop. Then the left arm tests again and the brush goes back in the pot.

Variations that must score less than 1: ``no_steady`` the cartridge is not held while the stove is turned;
``no_retest`` no second test after tightening; ``no_test`` tighten without testing; ``no_tighten`` test, see the
froth and leave the stove; ``always_tighten`` tighten whatever the test shows; ``open_control``
push the flame control open first, then do everything right; ``unscrew`` turn the stove the wrong way to the
thread's end; ``brush_kept`` the brush is not put back: the arm parks with it in its gripper.

One more shows a property and costs nothing: ``force`` (for ``home``) first turns the stove 40 degrees further
against its stop with the cartridge pressed down with 30 N, then does everything right. With that hold the
cartridge goes round with the stove from about 11 degrees past home (runs n2r, n2u, n2v); over-tightening is not
scored.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))  # demo_kit, look

SEEDS = {"loose": 65536, "home": 0}  # bit 16 of the seed set / clear
VARIATIONS = ("no_steady", "no_retest", "no_test", "no_tighten", "always_tighten", "force", "open_control", "unscrew", "brush_kept")
FORCE_PRESS = 30.0  # N on the cartridge while the ``force`` variation over-turns the stove


def main() -> int:
    import demo_kit as kit
    import look
    from demo_kit import Move, about, down

    args = [a for a in sys.argv[1:] if a != "--dry"]
    dry = "--dry" in sys.argv  # no simulator: the plan through the IK only, assuming froth in the loose case
    out = Path(args[0])
    runs = [(run.split(":") + [None])[:2] for run in (args[1].split(",") if len(args) > 1 else list(SEEDS))]
    for case, variation in runs:
        if case not in SEEDS or (variation is not None and variation not in VARIATIONS):
            raise SystemExit(f"a run is loose or home, optionally with :variation from {VARIATIONS}; got {case}:{variation}")
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk
    top = m.TOP
    cx, cy = m.CART_XY
    up = np.array([0.0, 0.0, 1.0])

    # --- the brush in the left gripper: taken 40 mm above its middle, slanted as it stands in the pot ---
    grasp_at = -0.040  # along the brush from its middle
    slant = np.array([math.cos(m.LEAN), 0.0, -math.sin(m.LEAN)])  # along the brush to its bristles' end, in the pot
    middle = np.array(m.BRUSH_AT) - 0.001 * up
    grasp = middle + grasp_at * slant
    lifted = grasp + (m.POT_H + 0.016 - ((middle + m.TIP * slant)[2] - top - m.HEAD_R)) * up  # the head 16 mm over the rim
    reach = m.TIP - grasp_at  # from the grasp point to the bristles' end
    across = about(up, -math.pi / 2) @ slant  # the brush after the gripper has turned a quarter clockwise: pointing at -y
    centre = np.array([cx, cy, m.JOINT_Z])

    def dab(phi_deg: float, gap: float = 0.003) -> np.ndarray:
        """The grasp point with the bristles' end ``gap`` off the joint line at ``phi`` (anticlockwise from +x). The
        bristles wet what is within 9 mm of the head's axis, and a head whose axis runs within 10 mm of the stove's
        axis wets nothing: 3 mm off the 12 mm base leaves 5 mm either way for a cartridge that has moved."""
        outward = np.array([math.cos(math.radians(phi_deg)), math.sin(math.radians(phi_deg)), 0.0])
        return centre + (m.BASE_R + gap) * outward - reach * across

    park = {"left": (0.12, 0.30, top + 0.16), "right": (0.12, -0.30, top + 0.16)}
    quarter = down(-math.pi / 2)

    def seconds(moves: list[Move]) -> float:
        return sum(move.seconds for move in moves)

    def to_park(side: str) -> list[Move]:
        return [Move(2.5, pos=park[side], rot=down(0.0), grip=1.0, label="park")]

    def take_brush() -> list[Move]:
        return [
            Move(2.5, pos=lifted, rot=down(0.0), grip=1.0, label="over brush"),
            Move(1.5, pos=grasp, label="to brush"),
            Move(2.5, grip=0.0, label="grip brush"),
            Move(2.0, pos=lifted, label="lift brush"),
        ]

    # Where the brush is held for a look: its bristles' end 85 mm to the left of the stove's axis, the gripper further
    # left still. Seen from the top camera neither covers the cartridge, and their shadows fall away from it.
    aside = centre + np.array([-0.02, 0.085, 0.0]) - reach * across
    LOOK_S, WATCH_S = 0.6, 5.0  # held aside before the strokes; from the last stroke's end to the second picture

    def test() -> list[Move]:
        """The brush aside for a look, then five short strokes, at 70, 100, 130, 160 and 185 degrees: from the left
        side of the joint round its near side; then aside again, in the same place, until 5 s after the last stroke.
        Each stroke brings the bristles' end from 6 mm off the stove's base up to it, so the line is wetted wherever
        the cartridge stands within 8 mm of its place along the stroke (the bristles wet from 9 mm off the base to
        2 mm inside it); each wets about 80 degrees, together 200 with overlap, where 180 are needed. No further
        round than 185: the brush points across the table, and from 200 degrees on its metal band would meet the
        valve cup and push the cartridge (run n2g: 5 mm, with a stroke at 220)."""
        angles = (70, 100, 130, 160, 185)
        moves = [Move(3.0, pos=aside, rot=quarter, label="to stove"), Move(LOOK_S, label="look")]
        for angle in angles:
            moves += [Move(0.7, pos=dab(angle, 0.006), label=f"to {angle}"), Move(0.4, pos=dab(angle, 0.0), label=f"stroke {angle}"), Move(0.2, label=f"stroke {angle}")]
        return [*moves, Move(1.2, pos=aside, label="aside"), Move(WATCH_S - 1.2, label="watch")]

    def put_brush_back() -> list[Move]:
        return [
            Move(3.0, pos=lifted, rot=down(0.0), label="over pot"),
            Move(1.5, pos=grasp + 0.003 * up, label="into pot"),
            Move(2.0, grip=1.0, label="let go"),
            Move(1.2, pos=lifted, label="up"),
        ]

    span, around = 50.0 / 94.0, 60.0 / 94.0  # the left hand's opening on the shoulder (50 mm); the right hand's round the 38.5 mm stove (60 mm)

    def press(wait: float, hold: float, newtons: float = 20.0) -> list[Move]:
        """The left hand, open 50 mm and leaning 50 degrees so that its palm stays outside the other hand, slides in
        along its own length until both fingertips' lower corners lie on the cartridge's shoulder, 36 mm from the
        axis either side of the stove's base (at 103 and 197 degrees, away from the control and from the brush),
        and presses straight down with ``newtons``, half on each tip. On the rim's friction of 0.2 the cartridge
        then holds 0.2 x (20 + 2.7) N x 38 mm = 0.17 N m against the thread's 0.05 N m.

        Two tips, because one is a pivot: pressed 34 mm off its axis under one fingertip, the cartridge stands
        almost on the one point of its rim below it and swings about that finger at a touch (run n2f: 8 mm and
        9 degrees in 16 s, the second test missed the joint). ``wait``: seconds over the shoulder before sliding
        in, for the other hand to close on the stove first."""
        phi, lean = math.radians(150), math.radians(50)
        outward = np.array([math.cos(phi), math.sin(phi), 0.0])
        rot = about(np.cross(up, -outward), -lean) @ down(phi + math.pi)
        tips = kit.lowest_point(rot)  # the middle of the two tips' lower corners, from the grasp point in the gripper's frame
        touch = np.array([cx, cy, top + m.CART_H - m.CUP_H]) + 0.024 * outward - rot @ tips
        back = touch - 0.07 * rot[:, 2]  # 7 cm back along the hand: 5 cm out and 4.5 cm up, clear of the other hand
        return [
            Move(1.0, grip=span, label="half open"),
            Move(3.0, pos=back, rot=rot, label="by shoulder"),
            Move(wait, label="wait"),
            Move(2.0, pos=touch + 0.001 * up, label="touch"),
            Move(1.5, push=(0.0, 0.0, -newtons), push_at=tips, label="press"),
            Move(hold, label="hold press"),
            Move(1.0, push=(0.0, 0.0, 0.0), label="ease off"),
            Move(2.0, pos=back, label="back out"),
        ]

    def tighten(steady: bool = True, sweep: tuple = (80.0, -42.0), newtons: float = 20.0, bear: float = 0.0) -> tuple[list[Move], list[Move]]:
        """(left, right). The right hand comes over the stove open 60 mm, goes down and pinches the folded stove;
        the left hand presses on the cartridge (or waits: ``steady``); the right turns from ``sweep[0]`` to
        ``sweep[1]`` degrees, bearing down with ``bear`` N; the left lets go and backs out; the right opens and
        goes up.

        By default 122 degrees clockwise: the stove, 120 degrees from home, runs onto its stop and is left a
        degree or two past home, against the stop's spring of 10 N m/rad (1.6 degrees in run n2v). The plan
        cannot feel the stop; that is what "tight" is here."""
        over, pinch = (cx, cy, top + 0.19), (cx, cy, top + 0.122)
        start, end = (math.radians(a) for a in sweep)
        approach = [
            Move(2.5, pos=over, rot=down(start), grip=around, label="over stove"),
            Move(2.0, pos=pinch, label="descend"),
            Move(2.5, grip=0.0, label="pinch"),
        ]
        twist = [Move(4.0 * abs(sweep[1] - sweep[0]) / 120.0 + 0.5, rot=down(end), push=(0.0, 0.0, -bear), label="turn"), Move(1.0, label="hold")]
        release = [Move(1.0, push=(0.0, 0.0, 0.0), label="ease off"), Move(2.0, grip=around, label="open"), Move(1.5, pos=over, label="up")]
        if not steady:
            right = [*approach, *twist, *release]
            return [Move(seconds(right), label="wait")], right
        left = press(max(0.0, seconds(approach) - 4.0), seconds(twist), newtons)
        pressing = seconds(left[:5])  # the push is on from then
        right = [*approach, Move(pressing - seconds(approach), label="hold pinch"), *twist, Move(3.0, label="hold"), *release]
        return left, right

    def open_control() -> list[Move]:
        """The right gripper's closed fingertips push down on one edge of the flame control's handle near its root
        (inside the cartridge's rim, so the push cannot tip it): the handle turns about its length, which opens it."""
        radial = np.array(m.RADIAL)
        edge = np.array(m.CONTROL_AT) - 0.0125 * radial - 0.007 * np.cross(up, radial)  # 32 mm from the axis, 7 mm to the opening side
        over = edge + (0.013 + 0.05) * up
        return [
            Move(1.0, grip=0.0, label="close fingers"),
            Move(3.0, pos=over, rot=down(m.HANDLE_ANGLE), label="over control"),
            Move(1.5, pos=edge + (0.013 - 0.014) * up, label="push control"),
            Move(0.5, label="push control"),
            Move(1.5, pos=over, label="up"),
        ]

    def bubbles_on_show() -> int:
        """What the simulator knows, for the record beside the picture's verdict: froth balls above the table."""
        return sum(1 for i in range(m.BUBBLES) if mk.pose(f"bubble{i:02d}")[0][2] > top)

    camera = embodiment.rig.top_cam

    def white(pictures: dict, save: Path) -> int:
        """The pixel test's count on the top camera's picture now; the picture, at the policy's size, is kept."""
        import cv2

        cv2.imwrite(str(save), cv2.cvtColor(kit.policy_picture(pictures["top_cam"]), cv2.COLOR_RGB2BGR))
        return look.white_on_cartridge(m, camera, pictures["top_cam"])

    def extra() -> dict:
        position, rotation = mk.pose("cartridge")
        return {
            "cartridge yaw (deg)": round(math.degrees(math.atan2(rotation[1, 0], rotation[0, 0])), 2),
            "cartridge moved (mm)": round(1000 * float(np.linalg.norm(position[:2] - np.array(m.CART_XY))), 2),
            "past home (deg)": round(max(-math.degrees(mk.joint("thread")), 0.0), 2),
            "control (deg)": round(math.degrees(mk.joint("control")), 1),
            "bubbles": bubbles_on_show(),
            "wet (deg)": m.report()["joint line wet (deg)"],
            "tests": m.STATE["tests"],
        }

    for case, variation in runs:
        left0: list[Move] = []
        right0: list[Move] = []
        if variation == "open_control":
            right0 = [*open_control(), *to_park("right")]
            left0 = [Move(seconds(right0), label="wait")]
        if variation == "unscrew":  # the wrong way: 57 degrees anticlockwise from 118 to 120 ends at the thread's end, 175
            left0, right0 = tighten(sweep=(-30.0, 27.0))
        if variation == "force":  # the stove is home: 40 degrees further, against the stop, the cartridge held with 30 N (20 N held it up to 3.7 N m in run n2p)
            left0, right0 = tighten(sweep=(80.0, 40.0), newtons=FORCE_PRESS)
            right0 = [*right0, *to_park("right")]
        tests_first = variation not in ("no_test", "unscrew")
        before_test = [*left0, *take_brush()]
        first = [*before_test, *test()] if tests_first else left0
        look_ref = seconds(before_test) + 3.0 + LOOK_S  # the brush has been held aside for LOOK_S: the first picture
        look_at = seconds(first)  # 5 s after the last stroke, the brush aside again: the second picture
        name = case if variation is None else f"{case}_{variation}"
        state = {"decided": False}

        def director(t: float, arms: dict, pictures, look_ref=look_ref, look_at=look_at, state=state, case=case, variation=variation, tests_first=tests_first, name=name) -> None:
            """Two pictures, one decision: is there froth? Then the rest of the plan is laid out."""
            if tests_first and pictures is not None and "before" not in state and t >= look_ref - 1e-6:
                state["before"] = white(pictures, out / name / "look_before.png")
            if state["decided"] or t < look_at - 1e-6:
                return
            state["decided"] = True
            if not tests_first:
                froth = False
            elif pictures is None:  # a dry run sees nothing: it takes the hidden case as given
                froth = case == "loose"
            else:
                state["after"] = white(pictures, out / name / "look_after.png")
                froth = state["after"] - state["before"] >= look.FROTH_MORE
                state["balls on show in the simulator"] = bubbles_on_show()
            state["froth"] = froth
            left, right = arms["left"].moves, arms["right"].moves
            right.append(Move(max(0.0, t - arms["right"].seconds), label="wait"))
            holding_brush = tests_first
            act = (froth and variation != "no_tighten") or variation in ("no_test", "always_tighten")
            if variation == "unscrew":
                act = False
            if act:
                if holding_brush:
                    left.extend(put_brush_back())
                    right.append(Move(seconds(put_brush_back()), label="wait"))
                held, turned = tighten(steady=variation != "no_steady")
                left.extend(held)
                right.extend(turned)
                right.extend(to_park("right"))
                holding_brush = False
                if variation not in ("no_retest", "no_test"):
                    left.extend([*take_brush(), *test()])
                    holding_brush = True
            if holding_brush and variation != "brush_kept":
                left.extend(put_brush_back())
                holding_brush = False
            left.extend([Move(2.5, pos=park["left"], rot=down(0.0), label="park with brush")] if holding_brush else to_park("left"))  # with it: the gripper stays closed
            if variation == "unscrew":
                right.extend(to_park("right"))
            print(f"t {t:.2f}: near-white on the cartridge {state.get('before')} before the strokes, {state.get('after')} after: {'froth' if froth else 'no froth'} "
                  f"(the simulator has {state.get('balls on show in the simulator')} balls on show) -> {'tighten' if act else 'leave the stove'}", flush=True)  # fmt: skip

        if dry:
            from yam_arena import station

            print(f"DRY {name}: {kit.dry_run({'left': first, 'right': right0}, station.BASES, director)}", flush=True)
            continue
        result = kit.run(embodiment, {"left": first, "right": right0}, out / name, seed=SEEDS[case], director=director, extra=extra,
                         crop={"top_cam": (170, 120, 470, 290)}, sheet_frames=16)  # fmt: skip
        rows = result["rows"]
        scores = [row["info"].get("score", 0.0) for row in rows]
        firsts = {level: next((row["t"] for row, s in zip(rows, scores) if s >= level), None) for level in (0.25, 0.5, 0.75, 1.0)}
        print(f"DEMO {name}: score {scores[-1]:.2f} success {result['terminated'] and result['termination_reason'] == 'success'} after {result['seconds']:.1f} s "
              f"({result['steps']} steps of {m.RUN_LIMIT_STEPS}); first reached {firsts}; near-white on the cartridge before/after the strokes {state.get('before')}/{state.get('after')}, froth {state.get('froth')}, balls in the simulator {state.get('balls on show in the simulator')}; "
              f"report {result['report']}; worst IK {result['worst_ik']}", flush=True)  # fmt: skip
        # The readings a limit is judged against: how far the stove stood past home by what the right hand was
        # doing, and the most the flame control and the cartridge moved.
        past_by_phase: dict[str, float] = {}
        for row in rows:
            phase = row["right"] if row["right"] not in ("wait", "done", "park", "over stove", "descend", "up") else ("left presses" if row["left"] in ("press", "hold press") else "hands off")
            past_by_phase[phase] = max(past_by_phase.get(phase, 0.0), row["extra"]["past home (deg)"])
        readings = {
            "past home (deg), most by phase": {phase: round(v, 2) for phase, v in past_by_phase.items()},
            "flame control (deg), most": max(row["extra"]["control (deg)"] for row in rows),
            "cartridge from its place (mm), most": max(row["extra"]["cartridge moved (mm)"] for row in rows),
        }
        seen = {key: state.get(key) for key in ("before", "after", "froth", "balls on show in the simulator")}
        (out / name / "look.json").write_text(json.dumps({"near-white pixels on the cartridge's top (320 x 176)": seen, "score": scores[-1], **readings}, indent=1))
        print(f"READINGS {name}: {readings}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
