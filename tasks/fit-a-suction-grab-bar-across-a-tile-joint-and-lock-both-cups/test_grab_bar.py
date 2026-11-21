"""The grab bar task's rules without the simulator: every state set directly (the bar where it is put, its cups and
levers riding on it), the step hook run as the simulator would run it."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest
from yam_arena import task_file

TASK = Path(__file__).resolve().parent / "task.py"
sys.path.insert(0, str(TASK.parent))
import parts as P  # noqa: E402

AWAY = {"left": (np.array([0.10, 0.30, 1.00]), 1.0), "right": (np.array([0.10, -0.30, 1.00]), 1.0)}
DOWN, UP = P.LEVER_DEG, 0.0


@pytest.fixture
def m():
    before = sys.modules.get("mechkit")
    module = task_file.load(TASK).module
    yield module
    module.mk.MECH.clear()
    sys.modules.pop("mechkit", None)
    if before is not None:
        sys.modules["mechkit"] = before


def place(m, dy=0.0, dz=0.0, tilt_deg=0.0, levers=(DOWN, DOWN), press=(0.0, 0.0), grippers=AWAY, time=None, moving=False, speeds=None, dx=0.0):
    """The bar moved by (dx, dy, dz) from where it was built and turned ``tilt_deg`` about x (off level), its cups and
    its levers (deg from up: left, right) riding on it; each cup's press on the tile (N); everything at rest unless
    ``moving``; ``speeds``: body -> (linear m/s, angular rad/s), over the rest."""
    mk = m.mk
    p0, r0 = mk.built_pose("bar")
    c, s = math.cos(math.radians(tilt_deg)), math.sin(math.radians(tilt_deg))
    turn = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    bar = (p0 + np.array([dx, dy, dz]), turn @ r0)
    poses = {"bar": bar}
    for side, theta in zip(("l", "r"), levers):
        poses[f"cup_{side}"] = mk.joint_pose(mk.MECH.joints[f"cup_{side}_on"], 0.0, bar)
        poses[f"lever_{side}"] = mk.joint_pose(mk.MECH.joints[f"lever_{side}"], math.radians(theta) - P.LEVER_RAD, bar)
    poses["panel"] = mk.built_pose("panel")
    speed = (np.array([0.0, 0.0, -0.2]), np.zeros(3)) if moving else (np.zeros(3), np.zeros(3))
    velocities = {n: speed for n in poses}
    velocities.update({n: (np.asarray(v, dtype=float), np.asarray(w, dtype=float)) for n, (v, w) in (speeds or {}).items()})
    contacts = {("cup_l", "panel"): press[0], ("cup_r", "panel"): press[1]}
    mk.set_state(poses=poses, velocities=velocities, grippers=grippers, time=time, contacts=contacts)


def started(m):
    """The trial's start: as built, the reset hook run."""
    place(m, time=0.0)
    m.start()
    return m


def stepped(m, **kw):
    place(m, **kw)
    m.step()


def test_every_key_figure_has_its_source_and_a_check_on_the_bought_product():
    named = set()
    for f in P.FIGURES:
        assert f.kind in P.SOURCE_KINDS and f.source and f.check, f.figure
        assert ("https://" in f.source) if f.kind in ("maker", "listing") else True, f.figure
        assert f.source.startswith("estimate: ") if f.kind == "estimate" else True, f.figure
        for name in f.constants:
            assert hasattr(P, name), name
            named.add(name)
    assert {"BAR_L", "BAR_MASS", "CUP_D", "CUP_PITCH", "LEVER_N", "PRESS_N", "HOLD_N", "JOINT_MARGIN", "TILE_W", "TILE_T", "JOINT_W", "BOARD_T"} <= named
    with pytest.raises(ValueError):
        P.Figure("x", ("BAR_L",), "assumed", "estimate: x", "caliper")
    with pytest.raises(ValueError):
        P.Figure("x", ("BAR_L",), "maker", "the maker says so", "caliper")


def test_the_lever_needs_LEVER_N_at_centre_from_either_side_is_loose_up_to_the_cam_and_holds_LEVER_HOLD_down(m):
    lever = m.LEVERS["l"]
    at = lambda deg, locked: lever.torque(math.radians(deg), locked) / P.LEVER_ARM  # noqa: E731  N at the tip, positive towards down
    assert at(P.LEVER_CENTRE_DEG, False) == pytest.approx(-P.LEVER_N) and at(P.LEVER_CENTRE_DEG, True) == pytest.approx(P.LEVER_N)
    assert at(DOWN, True) == pytest.approx(P.LEVER_HOLD * P.LEVER_N)
    assert at(UP, False) == 0.0 and at(30.0, False) == 0.0 and at(P.LEVER_CAM_DEG, False) == 0.0  # loose from up to the cam
    halfway = (P.LEVER_CAM_DEG + P.LEVER_CENTRE_DEG) / 2
    assert at(halfway, False) == pytest.approx(-P.LEVER_N / 2) and at(100.0, True) > 0.0  # each side pushes towards its own rest
    assert lever.target(math.radians(30.0), False) == pytest.approx(math.radians(30.0) - P.LEVER_RAD)  # loose: the drive's target is the lever itself


@pytest.mark.parametrize("side", ["l", "r"])
def test_each_lever_sits_at_its_housings_outer_end_and_turns_out_from_the_wall_as_parts_says(m, side):
    mk = m.mk
    p0, r0 = mk.built_pose("bar")
    sign = P.out_sign(side)
    for deg in (P.LEVER_DEG, P.LEVER_CENTRE_DEG, P.LEVER_CAM_DEG, 0.0):
        p, r = mk.joint_pose(mk.MECH.joints[f"lever_{side}"], math.radians(deg) - P.LEVER_RAD, (p0, r0))
        tip = np.asarray(p) + np.asarray(r) @ np.array([P.LEVER_ARM, 0.0, 0.0])
        assert tip == pytest.approx(np.asarray(p0) + np.asarray(P.lever_tip_b(side, math.radians(deg))), abs=1e-9)
    locked = P.lever_tip_b(side, P.LEVER_RAD, P.LEVER_L)  # lying along the housing's end, its free end at the wall
    assert locked[0] < -P.CUP_T and sign * (locked[1] - P.CUP_Y[side]) > P.HOUSING_D / 2
    released = P.lever_tip_b(side, P.CAM_RAD, P.LEVER_L)  # swung out, away from the bar's middle
    assert sign * (released[1] - locked[1]) > 0.02


def test_idle_scores_0_the_right_cup_lies_across_the_joint(m):
    started(m)
    assert m.sealed("l") and not m.sealed("r")
    assert m.rim_clearance("r")[0] < 0.0 < P.JOINT_MARGIN <= m.rim_clearance("l")[0] and m.clear("l") and not m.clear("r")
    m.step()
    assert m.score(None) == 0.0


ON_BENCH = -(P.BAR_Z - P.CUP_R)  # dz: the cups' rims (the bar's lowest point) on the bench
BODIES = ("bar", "cup_l", "cup_r", "lever_l", "lever_r")


def moving_at(v):
    """Every body of the bar moving at ``v`` (m/s, world), not turning."""
    return {n: (np.asarray(v, dtype=float), np.zeros(3)) for n in BODIES}


def test_the_bar_let_go_on_the_wall_falls_onto_the_bench_and_that_scores_0_for_good(m):
    started(m)
    stepped(m, levers=(UP, DOWN), time=0.5)  # left lever up over centre: the left seal is cut
    assert not m.sealed("l") and any("l lever up" in e for _, e in m.STATE["events"])
    stepped(m, dz=ON_BENCH, levers=(UP, DOWN), time=0.6)  # the next step: the bar on the bench, 22.5 mm down
    assert m.STATE["fell"] == pytest.approx(0.6) and "as from a 22.5 mm drop" in m.STATE["events"][-1][1]
    stepped(m, dz=ON_BENCH, levers=(UP, UP), time=1.0)
    stepped(m, dy=-P.MOVE, levers=(UP, UP), press=(P.PRESS_N + 2, P.PRESS_N + 2), time=5.0)  # even put back up on tile and both locked: the fall stands
    stepped(m, dy=-P.MOVE, levers=(DOWN, DOWN), press=(P.PRESS_N + 2, P.PRESS_N + 2), time=5.05)
    stepped(m, dy=-P.MOVE, levers=(DOWN, DOWN), time=6.0)
    assert m.sealed("l") and m.sealed("r") and m.score(None) == 0.0


def test_set_down_gently_on_the_bench_and_put_back_on_tile_scores_as_the_remount(m):
    """Released while held, lowered onto the bench at 30 mm/s, left there, lifted, moved and locked: 1.0."""
    started(m)
    stepped(m, levers=(UP, UP), time=0.5)
    for i, dz in enumerate((-0.006, -0.012, -0.018, ON_BENCH + 0.0015)):
        stepped(m, dz=dz, levers=(UP, UP), speeds=moving_at((0.0, 0.0, -0.03)), time=0.6 + 0.2 * i)
    stepped(m, dz=ON_BENCH, levers=(UP, UP), time=1.5)  # on the bench
    stepped(m, dz=ON_BENCH, levers=(UP, UP), time=3.0)
    assert m.STATE["fell"] is None and any("onto the bench as from a 3.0 mm drop" in e for _, e in m.STATE["events"])  # 4.5 - 1.5 mm + 0.05 mm
    stepped(m, dz=-0.010, levers=(UP, UP), speeds=moving_at((0.0, 0.0, 0.03)), time=4.0)
    stepped(m, dy=-P.MOVE, levers=(UP, UP), press=(P.PRESS_N + 2, P.PRESS_N + 2), time=6.0)
    stepped(m, dy=-P.MOVE, levers=(DOWN, DOWN), press=(P.PRESS_N + 2, P.PRESS_N + 2), time=6.05)
    stepped(m, dy=-P.MOVE, levers=(DOWN, DOWN), time=8.0)
    assert m.sealed("l") and m.sealed("r") and m.score(None) == 1.0


@pytest.mark.parametrize("over_mm, down, fell", [(9.5, 0.0, False), (10.5, 0.0, True), (8.0, 0.16, False), (9.0, 0.18, True), (5.0, 0.40, True)])
def test_the_bar_falls_when_its_last_step_down_plus_v2_over_2g_onto_the_bench_reaches_10_mm(m, over_mm, down, fell):
    """In the step before it touches: ``over_mm`` over the bench, coming down at ``down`` m/s (let go: 0)."""
    started(m)
    stepped(m, levers=(UP, UP), time=0.5)
    stepped(m, dz=ON_BENCH + over_mm / 1000, levers=(UP, UP), speeds=moving_at((0.0, 0.0, -down)), time=1.0)
    stepped(m, dz=ON_BENCH, levers=(UP, UP), time=1.05)
    assert (m.STATE["fell"] is not None) == fell


@pytest.mark.parametrize("speed, struck", [(0.30, False), (0.50, True)])
def test_a_cup_meeting_the_tiles_at_0_44_m_s_or_more_struck_them(m, speed, struck):
    assert m.STRIKE_MPS == pytest.approx(0.443, abs=1e-3)
    started(m)
    stepped(m, levers=(UP, UP), time=0.5)
    stepped(m, dx=-0.005, dy=-P.MOVE, levers=(UP, UP), speeds=moving_at((speed, 0.0, 0.0)), time=1.0)  # 5 mm off the wall, coming in
    stepped(m, dy=-P.MOVE, levers=(UP, UP), press=(P.PRESS_N + 2, P.PRESS_N + 2), time=1.05)
    stepped(m, dy=-P.MOVE, levers=(DOWN, DOWN), press=(P.PRESS_N + 2, P.PRESS_N + 2), time=1.1)
    stepped(m, dy=-P.MOVE, levers=(DOWN, DOWN), time=3.0)
    assert (m.STATE["fell"] is not None) == struck and m.score(None) == (0.0 if struck else 1.0)


def test_moved_clear_one_seal_or_both_off_level_score_half_both_level_score_1(m):
    started(m)
    stepped(m, levers=(UP, UP), time=0.5)  # both released (the right one was not sealed)
    stepped(m, dy=-P.MOVE, levers=(UP, UP), press=(P.PRESS_N + 2, P.PRESS_N + 2), time=2.0)
    stepped(m, dy=-P.MOVE, levers=(DOWN, UP), press=(P.PRESS_N + 2, P.PRESS_N + 2), time=2.1)  # left down over centre
    assert m.sealed("l") and not m.sealed("r")
    stepped(m, dy=-P.MOVE, levers=(DOWN, UP), time=4.0)
    assert m.score(None) == 0.5
    stepped(m, dy=-P.MOVE, levers=(DOWN, DOWN), press=(0.0, P.PRESS_N + 2), time=4.1)  # right down over centre, pressed
    assert m.sealed("r")
    stepped(m, dy=-P.MOVE, levers=(DOWN, DOWN), time=6.0)
    assert m.score(None) == 1.0
    stepped(m, dy=-P.MOVE, dz=0.02, tilt_deg=12.0, levers=(DOWN, DOWN), time=6.1)  # off level by more than 10 deg: not 1.0 (20 mm up: the lower rim keeps its margin from the tile's edge)
    assert all(m.clear(s) for s in m.SIDES) and m.score(None) == 0.5
    stepped(m, dy=-P.MOVE, tilt_deg=12.0, levers=(DOWN, DOWN), time=6.2)  # tilted where it hangs: the lower rim comes within the margin of the tile's edge
    assert m.rim_clearance("r")[1] < P.EDGE_MARGIN and not m.clear("r") and m.score(None) == 0.0


@pytest.mark.parametrize("gap_mm", [3.0, 12.0])  # 12 mm: past a 5 mm lip, short of the maker's 1/2 in
def test_a_cup_closer_to_the_joint_than_half_an_inch_does_not_seal_and_its_indicator_stays_red(m, gap_mm):
    started(m)
    stepped(m, levers=(DOWN, UP), time=0.5)
    near = -(P.BAR_Y0 + P.CUP_Y["r"]) - (P.CUP_R + P.JOINT_W / 2 + gap_mm / 1000)  # the right cup's rim gap_mm right of the joint
    stepped(m, dy=near, levers=(DOWN, UP), press=(0.0, 30.0), time=1.0)
    assert m.rim_clearance("r")[0] == pytest.approx(gap_mm / 1000, abs=1e-6)
    stepped(m, dy=near, levers=(DOWN, DOWN), press=(0.0, 30.0), time=1.05)
    assert not m.sealed("r") and m.STATE["refused"]["r"] and "from the joint" in m.STATE["refused"]["r"][-1][1][0]
    red, green = m._IND["r"]
    assert ("bar", green) in m.mk.MECH.hidden and ("bar", red) not in m.mk.MECH.hidden


def test_a_cup_half_an_inch_from_the_joint_seals(m):
    started(m)
    stepped(m, levers=(DOWN, UP), time=0.5)
    near = -(P.BAR_Y0 + P.CUP_Y["r"]) - (P.CUP_R + P.JOINT_W / 2 + P.JOINT_MARGIN + 0.0005)  # 0.5 mm past 1/2 in
    stepped(m, dy=near, levers=(DOWN, UP), press=(0.0, 30.0), time=1.0)
    stepped(m, dy=near, levers=(DOWN, DOWN), press=(0.0, 30.0), time=1.05)
    assert m.sealed("r") and m.clear("r")


def test_pressed_under_PRESS_N_the_lever_goes_down_but_nothing_seals(m):
    started(m)
    stepped(m, levers=(UP, UP), time=0.5)
    stepped(m, dy=-P.MOVE, levers=(UP, UP), press=(P.PRESS_N - 2, 0.0), time=1.0)
    stepped(m, dy=-P.MOVE, levers=(DOWN, UP), press=(P.PRESS_N - 2, 0.0), time=1.05)
    assert not m.sealed("l") and "under" in m.STATE["refused"]["l"][-1][1][0]
    # pressed hard enough but more than PRESS_S ago: still no seal
    stepped(m, dy=-P.MOVE, levers=(UP, UP), press=(P.PRESS_N + 5, 0.0), time=2.0)
    stepped(m, dy=-P.MOVE, levers=(UP, UP), press=(0.0, 0.0), time=2.0 + P.PRESS_S + 0.05)
    stepped(m, dy=-P.MOVE, levers=(DOWN, UP), press=(0.0, 0.0), time=2.0 + P.PRESS_S + 0.10)
    assert not m.sealed("l")


def test_not_let_go_or_not_at_rest_scores_0(m):
    started(m)
    stepped(m, levers=(UP, UP), time=0.5)
    stepped(m, dy=-P.MOVE, levers=(UP, UP), press=(20.0, 20.0), time=1.0)
    stepped(m, dy=-P.MOVE, levers=(DOWN, DOWN), press=(20.0, 20.0), time=1.05)
    assert m.sealed("l") and m.sealed("r")
    grip = m.mk.pose("bar")[0] + m.mk.pose("bar")[1] @ np.array([P.GRIP_X, 0.0, 0.0])
    stepped(m, dy=-P.MOVE, levers=(DOWN, DOWN), grippers={"left": (grip, 0.4), "right": AWAY["right"]}, time=2.0)
    assert m.score(None) == 0.0
    stepped(m, dy=-P.MOVE, levers=(DOWN, DOWN), moving=True, time=2.1)
    assert m.score(None) == 0.0
    stepped(m, dy=-P.MOVE, levers=(DOWN, DOWN), time=2.2)
    assert m.score(None) == 1.0


def test_the_bar_and_its_levers_count_for_rest_and_a_welded_cup_reading_alone_does_not(m):
    started(m)
    stepped(m, levers=(UP, UP), time=0.5)
    stepped(m, dy=-P.MOVE, levers=(UP, UP), press=(20.0, 20.0), time=1.0)
    stepped(m, dy=-P.MOVE, levers=(DOWN, DOWN), press=(20.0, 20.0), time=1.05)
    assert m.sealed("l") and m.sealed("r")
    stepped(m, dy=-P.MOVE, time=2.0)
    assert m.score(None) == 1.0
    slide = (np.array([0.0, 0.0, -0.015]), np.zeros(3))  # the bar sliding down the wall at 15 mm/s, its cups with it, both levers down
    stepped(m, dy=-P.MOVE, time=2.05, speeds={"bar": slide, "cup_l": slide, "cup_r": slide})
    assert not m.at_rest() and m.score(None) == 0.0
    stepped(m, dy=-P.MOVE, time=2.1, speeds={"lever_r": (np.zeros(3), np.array([0.0, 0.2, 0.0]))})  # a lever turning at 0.2 rad/s
    assert not m.at_rest() and m.score(None) == 0.0
    spin = (np.array([0.0014, 0.0, 0.0]), np.array([0.0, 0.0, 0.1035]))  # a cup read at 1.4 mm/s and 5.9 deg/s, the bar still (check run gb5d10)
    stepped(m, dy=-P.MOVE, time=2.15, speeds={"cup_l": spin, "cup_r": spin})
    assert m.at_rest() and m.score(None) == 1.0
