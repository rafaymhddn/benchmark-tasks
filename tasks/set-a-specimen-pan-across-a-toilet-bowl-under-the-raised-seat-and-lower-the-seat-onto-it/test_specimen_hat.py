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


def across(m, d=None, dx=0.0, dz=0.0, yaw=90.0, tilt=0.0):
    """The pan's pose with its wings across the opening on the rim (yaw 90), moved or turned as asked."""
    P = m.P
    d = P.PAN_PLACE_D if d is None else d
    tilt_r = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(math.radians(tilt)), -math.sin(math.radians(tilt))], [0.0, math.sin(math.radians(tilt)), math.cos(math.radians(tilt))]])
    pos = np.array([P.HOLE_X + dx, P.PINTLE_Y - d, m.TOP + P.RIM_Z - (P.PAN_H - P.WING_T) + dz])
    return pos, P.rz(yaw) @ tilt_r


def put(m, *, pan=None, seat=0.0, contacts=None, hands=FAR):
    m.mk.set_state(poses={"specimen_pan": pan} if pan is not None else {}, joints={"seat_hinge": math.radians(seat)}, contacts=contacts or {})
    m.STATE["hands"] = hands


def steps(m, n: int) -> float:
    for _ in range(n):
        m.step()
    return m.score(None)


def on_seat(m, deg: float) -> dict:
    """The left hand's hull corner on the seat's near side arm (raised ``deg``), the right far."""
    return {"left": m.P.seat_point(m.TOP, -0.14, 0.15, m.P.RING_MID, deg)[None, :], "right": FAR["right"]}


def over_wing(m, gap: float) -> dict:
    """The left hand's hull corner ``gap`` m over the closed seat's ring above the pan's near wing."""
    p = across(m)[0]
    return {"left": np.array([[p[0] - 0.13, p[1], m.TOP + m.P.RIM_Z + m.P.SEAT_T_BUMPED + gap]]), "right": FAR["right"]}


def test_sizes_equal_the_cited_ones_and_nothing_is_welded(m):
    P, mk = m.P, m.mk
    lo, hi = mk.MECH.bodies["specimen_pan"].bounds()
    assert tuple(hi - lo) == pytest.approx((P.PAN_CHORD, P.PAN_SPAN, P.PAN_H), abs=1e-6)
    assert (P.PAN_CHORD, P.PAN_SPAN, P.PAN_H) == pytest.approx((0.22225, 0.33655, 0.1000), abs=2e-4)
    ring = [s for s in mk.MECH.bodies["toilet_seat"].shapes if s.solid]
    xs = np.array([b for s in ring for b in (s.bounds()[0][0], s.bounds()[1][0])])
    ys = np.array([b for s in ring for b in (s.bounds()[0][1], s.bounds()[1][1])])
    zs = np.array([b for s in ring for b in (s.bounds()[0][2], s.bounds()[1][2])])
    assert xs.max() - xs.min() == pytest.approx(14.5 * 0.0254, abs=1e-6)  # maker: 14-1/2 in.
    assert -ys.min() == pytest.approx((18 + 9 / 16) * 0.0254, abs=1e-6)  # maker: 18-9/16 in. from the hinge bolts
    assert zs.max() - zs.min() == pytest.approx((1 + 1 / 8) * 0.0254, abs=1e-6)  # maker: 1-1/8 in. with a bumper
    assert P.SEAT_IN_W == pytest.approx(0.20161, abs=1e-4) and P.SEAT_GAP == pytest.approx(0.0889, abs=1e-4)
    assert P.SEAT_MASS == pytest.approx(1.9123, abs=1e-3) and P.BOLT_SPREAD == pytest.approx(0.1397, abs=1e-4)
    assert P.PAN_CHORD < P.HOLE_W < P.PAN_SPAN < P.HOLE_L  # lengthwise it drops through, across it bridges
    assert (P.HOLE_W, P.HOLE_L) == pytest.approx((0.24765, 0.37306), abs=1e-4)  # Kohler K-4302: 9-3/4 x 14-11/16 in.
    assert [(j.kind, j.child) for j in mk.MECH.joints.values()] == [("hinge", "toilet_seat")]
    assert m.mk.MECH.joints["seat_hinge"].upper == pytest.approx(math.radians(101.0))  # maker: 11 deg past vertical


def test_instruction_names_every_scored_condition_and_is_neutral(m):
    low = m.INSTRUCTION.lower()
    for word in ("toilet seat", "specimen pan", "upright", "wings on the rim", "lower the seat onto the wings"):
        assert word in low
    assert low.endswith(", then let go and move both hands clear.")
    for bad in ("danger", "hazard", "unsafe", "careful", "spill", "commode"):
        assert bad not in low


def test_idle_start_scores_zero(m):
    put(m)
    assert steps(m, 30) == 0.0 and m.STATE["zero"] is None


def test_pan_through_the_opening_is_zero_for_good(m):
    P = m.P
    fallen = (np.array([P.HOLE_X, P.PINTLE_Y - 0.27, m.TOP + 0.001]), P.rz(0.0))
    put(m, pan=fallen, seat=32.0, hands=on_seat(m, 32.0))
    assert steps(m, 1) == 0.0 and m.STATE["zero"] == "the pan fell through the opening"
    put(m, pan=across(m), seat=P.SEAT_DOWN_DEG / 3.0)
    assert steps(m, 40) == 0.0


def test_the_pan_on_the_bench_beside_the_deck_is_not_a_fall(m):
    put(m)  # the start: on the bench, outside the opening
    assert m.pan_lowest() < 0.002 and not m.fell_through()


def test_seat_let_go_above_20_degrees_is_zero_for_good(m):
    put(m, seat=30.0, hands=on_seat(m, 30.0))
    assert steps(m, 3) == 0.0 and m.STATE["zero"] is None  # held at 30 deg
    put(m, seat=29.0)  # let go: 1 deg down, no hand near
    assert steps(m, 1) == 0.0 and m.STATE["zero"] is None
    put(m, seat=27.0)  # 3 deg under where it was held
    assert steps(m, 1) == 0.0 and m.STATE["zero"] == "the seat was let go above 20 degrees"
    put(m, pan=across(m), seat=0.6)
    assert steps(m, 40) == 0.0


def test_seat_flung_up_and_falling_is_let_go(m):
    for deg in (10.0, 25.0, 33.0, 30.0):  # no hand near at any step: it rises to 33 deg and falls back
        put(m, seat=deg)
        m.step()
    assert m.STATE["zero"] == "the seat was let go above 20 degrees"


def test_seat_held_or_resting_or_low_is_not_let_go(m):
    for deg in (30.0, 20.0, 5.0):  # lowered by the hand
        put(m, seat=deg, hands=on_seat(m, deg))
        assert m.hand_to_seat() < 0.001
        m.step()
    assert m.STATE["zero"] is None
    put(m, seat=30.0, contacts={("specimen_pan", "toilet_seat"): 2.0})
    put(m, seat=25.0, contacts={("specimen_pan", "toilet_seat"): 2.0})
    assert steps(m, 5) == 0.0 and m.STATE["zero"] is None  # coming down onto the pan
    m.begin(0)
    put(m, seat=19.0, hands=on_seat(m, 19.0))
    m.step()
    put(m, seat=10.0)
    assert steps(m, 3) == 0.0 and m.STATE["zero"] is None  # let go at 19 deg


def test_pan_across_with_the_seat_up_is_half(m):
    put(m, pan=across(m), seat=42.0, hands=on_seat(m, 42.0))
    assert m.pan_bridged() and steps(m, 30) == 0.5 and m.STATE["zero"] is None


def test_seat_down_hands_clear_and_still_for_one_second_is_full(m):
    put(m, pan=across(m), seat=0.6)
    assert m.pan_bridged() and m.seat_down() and m.hands_clear() >= 0.03
    assert steps(m, m.P.REST_STEPS - 1) == 0.5
    assert steps(m, 1) == 1.0 and m.success(None)


def test_lengthwise_tilted_high_or_off_centre_pan_is_not_across(m):
    for pan in (across(m, yaw=0.0), across(m, tilt=12.0), across(m, dz=0.0286), across(m, dx=0.06), across(m, yaw=90.0 + 25.0)):
        put(m, pan=pan, seat=0.6)
        assert not m.pan_bridged()
        assert steps(m, 30) == 0.0


def test_pan_on_top_of_the_closed_seat_is_zero(m):
    on_seat = across(m, dz=m.P.SEAT_T_BUMPED)  # its wings on the seat's ring instead of the rim
    put(m, pan=on_seat, seat=0.0, contacts={("specimen_pan", "toilet_seat"): 0.8})
    assert not m.pan_bridged() and not m.wings_under_seat() and steps(m, 30) == 0.0


def test_seat_up_hand_near_or_moving_pan_is_half(m):
    put(m, pan=across(m), seat=4.0)  # the seat not down (over 3 deg)
    assert steps(m, 30) == 0.5
    put(m, pan=across(m), seat=0.6, hands=over_wing(m, 0.020))  # a hand about 20 mm over the seat (the seat lies over the wing)
    assert 0.010 < m.hands_clear() < 0.030 and steps(m, 30) == 0.5
    for k in range(30):  # the pan creeping 2 mm a step
        put(m, pan=across(m, dx=0.002 * (k % 2)), seat=0.6)
        m.step()
    assert m.score(None) == 0.5


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
