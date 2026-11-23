"""Rule tests of the specimen-pan task without the simulator: states put in with ``mechkit.set_state``."""

from __future__ import annotations

import json
import math
import re
import sys
from pathlib import Path

import numpy as np
import pytest
from yam_arena import task_file

HERE = Path(__file__).resolve().parent
TASK = HERE / "task.py"
FAR = {"left": np.array([[0.0, 0.30, 1.10]]), "right": np.array([[0.0, -0.30, 1.10]])}
INCH = 0.0254


@pytest.fixture
def m():
    before = {name: sys.modules.get(name) for name in ("mechkit", "parts", "scene")}
    module = task_file.load(TASK).module
    module.begin(0)
    yield module
    module.mk.MECH.clear()
    for name, old in before.items():
        sys.modules.pop(name, None)
        if old is not None:
            sys.modules[name] = old


def across(m, da=0.0, dd=0.0, dz=0.0, yaw=0.0, tilt=0.0):
    """The pan's pose with its wings across the opening on the rim (yaw 0), moved or turned as asked."""
    P = m.P
    t = math.radians(tilt)
    tilt_r = np.array([[math.cos(t), 0.0, math.sin(t)], [0.0, 1.0, 0.0], [-math.sin(t), 0.0, math.cos(t)]])
    pos = np.array([P.BOLT_X - P.PAN_PLACE_D - dd, P.BOWL_Y + da, m.TOP + P.RIM_Z - (P.PAN_H - P.WING_T) + dz])
    return pos, P.rz(yaw) @ tilt_r


def put(m, *, pan=None, seat=101.0, contacts=None, hands=FAR):
    joints = {"seat_hinge": math.radians(seat - m.P.CHECK_HINGE_DEG)}
    m.mk.set_state(poses={"specimen_pan": pan} if pan is not None else {}, joints=joints, contacts=contacts or {})
    m.STATE["hands"] = hands


def steps(m, n: int) -> float:
    for _ in range(n):
        m.step()
    return m.score(None)


def on_seat(m, deg: float) -> dict:
    """The left hand's hull corner on the seat's side arm (raised ``deg``), the right far."""
    return {"left": m.P.seat_point(m.TOP, -0.14, 0.15, m.P.RING_MID, deg)[None, :], "right": FAR["right"]}


def near_pan(m, gap: float) -> dict:
    """The right hand's hull corner ``gap`` m over the pan's front wall top, the left far."""
    p = across(m)[0]
    return {"left": FAR["left"], "right": np.array([[p[0] - m.P.PAN_CHORD / 2.0, p[1], p[2] + m.P.PAN_H + gap]])}


def test_sizes_equal_the_cited_ones_and_nothing_is_welded(m):
    P, mk = m.P, m.mk
    lo, hi = mk.MECH.bodies["specimen_pan"].bounds()
    assert tuple(hi - lo) == pytest.approx((P.PAN_CHORD, P.PAN_SPAN, P.PAN_H), abs=1e-6)  # in its own frame
    ring = [s for s in mk.MECH.bodies["toilet_seat"].shapes if s.solid]
    lo_s = np.min([s.bounds()[0] for s in ring], axis=0)
    hi_s = np.max([s.bounds()[1] for s in ring], axis=0)
    assert hi_s[0] - lo_s[0] == pytest.approx(14.5 * INCH, abs=1e-6)  # maker: 14-1/2 in.
    assert -lo_s[1] == pytest.approx((18 + 9 / 16) * INCH, abs=1e-6)  # maker: 18-9/16 in. from the hinge bolts
    assert hi_s[2] - lo_s[2] == pytest.approx((1 + 1 / 8) * INCH, abs=1e-6)  # maker: 1-1/8 in. with a bumper
    assert P.SEAT_MASS == pytest.approx(1.9123, abs=1e-3) and P.BOLT_SPREAD == pytest.approx(0.1397, abs=1e-4)
    bowl = [s for s in mk.MECH.bodies["toilet_bowl"].shapes if s.solid]
    lo_b = np.min([s.bounds()[0] for s in bowl], axis=0)
    hi_b = np.max([s.bounds()[1] for s in bowl], axis=0)
    assert hi_b[1] - lo_b[1] == pytest.approx(0.600)  # the wall; the bowl itself:
    rims = [s for s in bowl if abs(s.bounds()[1][2] - P.RIM_Z) < 1e-9]
    xs = [b for s in rims for b in (s.bounds()[0][0], s.bounds()[1][0])]
    ys = [b for s in rims for b in (s.bounds()[0][1], s.bounds()[1][1])]
    assert max(xs) - min(xs) == pytest.approx(26 * INCH) and max(ys) - min(ys) == pytest.approx(14.5 * INCH)  # maker
    assert lo_b[0] == pytest.approx(P.FRONT_X) and hi_b[0] - P.WALL_T - P.FRONT_X == pytest.approx((27 + 5 / 8) * INCH)  # maker: wall
    assert lo_b[2] == pytest.approx(0.0) and P.RIM_Z - P.WATER_Z == pytest.approx(6 * INCH)  # maker: water 6 in. under the rim
    assert P.PAN_CHORD < P.HOLE_W < P.PAN_SPAN < P.HOLE_L  # lengthwise it drops in, across it bridges
    assert (P.HOLE_W, P.HOLE_L) == pytest.approx((0.2635, 0.3791), abs=1e-4)
    assert [(j.kind, j.child) for j in mk.MECH.joints.values()] == [("hinge", "toilet_seat")]
    joint = mk.MECH.joints["seat_hinge"]
    assert joint.upper == pytest.approx(0.0) and joint.lower == pytest.approx(math.radians(-103.0))  # built on its stop
    put(m)
    assert m.seat_deg() == pytest.approx(101.0) and m.seat_up()
    assert P.seat_balance_deg() == pytest.approx(94.0, abs=0.5) and P.seat_torque(101.0) < 0.0  # it rests on its stop


def test_instruction_names_every_scored_condition_and_is_neutral(m):
    low = m.INSTRUCTION.lower()
    for word in ("specimen pan", "upright", "across the toilet bowl opening", "both wings on the rim", "leaving the raised seat up"):
        assert word in low
    assert low.endswith(", then let go and move both hands clear.")
    assert json.loads((HERE / "task.json").read_text())["instruction"] == m.INSTRUCTION
    for bad in ("danger", "hazard", "unsafe", "careful", "spill", "commode", "fall"):
        assert bad not in low


def test_idle_start_scores_zero(m):
    put(m)
    assert steps(m, 30) == 0.0 and m.STATE["zero"] is None
    assert m.pan_lowest() < 0.002 and not m.fell_in()  # on the bench beside the bowl: not a fall


def test_pan_in_the_bowl_is_zero_for_good(m):
    P = m.P
    fallen = (np.array([P.BOLT_X - P.PAN_PLACE_D, P.BOWL_Y, m.TOP + P.WATER_Z + 0.001]), P.rz(-90.0))
    put(m, pan=fallen)
    assert steps(m, 1) == 0.0 and m.STATE["zero"] == "the pan fell into the bowl"
    put(m, pan=across(m))
    assert steps(m, 40) == 0.0


def test_pan_on_the_rim_is_not_fallen_in(m):
    put(m, pan=across(m))
    assert m.pan_lowest() - m.P.WATER_Z == pytest.approx(0.0564, abs=1e-3) and not m.fell_in()


def test_seat_fallen_from_its_stop_is_zero_for_good(m):
    put(m, seat=95.0)  # pushed short of its balance: it goes back
    assert steps(m, 2) == 0.0 and m.STATE["zero"] is None
    put(m, seat=89.0)  # falling, nothing holds it
    assert steps(m, 1) == 0.0 and m.STATE["zero"] == "the seat fell from its stop"
    put(m, pan=across(m))
    assert steps(m, 40) == 0.0


def test_seat_held_or_resting_on_the_pan_is_not_fallen(m):
    for deg in (95.0, 80.0, 40.0):  # lowered by a hand
        put(m, seat=deg, hands=on_seat(m, deg))
        assert m.hand_to_seat() < 0.001
        m.step()
    assert m.STATE["zero"] is None
    put(m, pan=across(m), seat=1.0, contacts={("specimen_pan", "toilet_seat"): 12.0})
    assert steps(m, 5) == 0.5 and m.STATE["zero"] is None  # on the wings: across, but the seat is not up


def test_pan_across_seat_up_hands_clear_and_still_for_one_second_is_full(m):
    put(m, pan=across(m))
    assert m.pan_bridged() and m.seat_up() and m.hands_clear() >= 0.03
    assert steps(m, m.P.REST_STEPS - 1) == 0.5
    assert steps(m, 1) == 1.0 and m.success(None)


def test_lengthwise_tilted_high_or_off_centre_pan_is_not_across(m):
    for pan in (across(m, yaw=90.0), across(m, tilt=12.0), across(m, dz=0.012), across(m, da=0.06), across(m, yaw=25.0),
                across(m, dd=0.20)):  # fmt: skip
        put(m, pan=pan)
        assert not m.pan_bridged()
        assert steps(m, 30) == 0.0


def test_hand_near_moving_pan_or_seat_off_its_stop_is_half(m):
    put(m, pan=across(m), hands=near_pan(m, 0.015))  # a hand 15 mm over the pan's wall
    assert 0.010 < m.hands_clear() < 0.030 and steps(m, 30) == 0.5
    for k in range(30):  # the pan creeping 2 mm a step
        put(m, pan=across(m, da=0.002 * (k % 2)))
        m.step()
    assert m.score(None) == 0.5
    put(m, pan=across(m), seat=97.0)  # off its stop, short of the 99 deg
    assert steps(m, 30) == 0.5 and m.STATE["zero"] is None


def test_folder_limits():
    task_text = (HERE / "task.py").read_text()
    assert len(task_text) <= 15_000
    for name in ("task.py", "scene.py", "demo.py"):
        code = (HERE / name).read_text()
        assert "write_pose" not in code and "set_joint" not in code, name
    readme = HERE / "README.md"
    if readme.exists():
        text = readme.read_text()
        assert len(text) <= 10_000 and "a different reading replaces the estimate" in text
    tj = HERE / "task.json"
    if tj.exists():
        data = json.loads(tj.read_text())
        assert data["scores"] == "Dream-1 run: pending; demo: 1.0"
        assert len(data["title"]) <= 120 and len(data["context"]) <= 1000 and len(data["fidelity"]) <= 500
    for path in HERE.iterdir():
        if path.suffix in (".py", ".md", ".json"):
            text = path.read_text()
            assert not re.search(r"TK-[A-Z0-9]{6}", text) and not re.search(r"#\d{3,}", text), path.name
