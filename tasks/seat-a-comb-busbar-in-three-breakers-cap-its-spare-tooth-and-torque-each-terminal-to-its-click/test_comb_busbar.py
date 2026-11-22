"""Pure rule tests for ``comb_busbar`` (no simulator): the screw's law, the driver's engagement and clutch, the clamp
and cap holds, every score level and the whole-trial zero rule.

    python -m pytest -q test_comb_busbar.py
"""

from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np
import pytest
from yam_arena import task_file

HERE = Path(__file__).resolve().parent
TASK = HERE / "task.py"
OWN = ("mechkit", "kit_extra", "parts")
DT = 0.05
AWAY = {"left": (np.array([0.05, 0.32, 1.05]), 1.0), "right": (np.array([0.05, -0.32, 1.05]), 1.0)}


@pytest.fixture
def m():
    before = {name: sys.modules.pop(name, None) for name in OWN}
    module = task_file.load(TASK).module
    module.mk.MECH.new_episode()
    module.mk.mend_all()
    module.start()
    yield module
    module.mk.MECH.clear()
    for name, held in before.items():
        sys.modules.pop(name, None)
        if held is not None:
            sys.modules[name] = held


def rot_x(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


class Bench:
    def __init__(self, m) -> None:
        self.m, self.P, self.t = m, m.P, 0.0
        self.poses = {n: (np.array(m.mk.MECH.bodies[n].pos, float), m.mk.MECH.bodies[n].rotation()) for n in ("comb", "cap", "driver")}
        self.velocities: dict = {}
        self.grippers = dict(AWAY)

    def w(self, p_panel) -> np.ndarray:
        return self.P.pw(p_panel) + np.array([0.0, 0.0, self.m.TOP])

    def seat_comb(self, first: int = 0, lift: float = 0.0) -> None:
        P = self.P
        self.poses["comb"] = (self.w(P.comb_seated_panel(first) + np.array([0.0, 0.0, lift])), P.TILT.copy())

    def cap_on_tooth(self, k: int, depth: float | None = None) -> None:
        """Put the cap's socket over tooth ``k``, the tooth's tip ``depth`` into it."""
        P = self.P
        cp, cr = self.poses["comb"]
        tip = cp + cr @ P.tooth_tip_local(k)
        depth = P.PIN_L - 0.002 if depth is None else depth
        self.poses["cap"] = (tip + (depth - 0.5 * P.CAP_SIZE[2]) * cr[:, 2], cr.copy())

    def driver_at(self, i: int, roll_deg: float = 0.0, depth: float = 0.0025, off=(0.0, 0.0), tilt_deg: float = 0.0) -> None:
        P = self.P
        r = rot_y(tilt_deg) @ P.driver_rot(math.radians(roll_deg))
        tip = self.w(P.screw_pos(i) + np.array([depth, off[0], off[1]]))
        self.poses["driver"] = (tip - r @ np.array([P.TIP_X, 0.0, 0.0]), r)

    def step(self, n: int = 1) -> float:
        out = 0.0
        for _ in range(n):
            self.t = round(self.t + DT, 6)
            self.m.mk.set_state(poses=self.poses, velocities=self.velocities, grippers=self.grippers, time=self.t)
            self.m.step()
            out = self.m.score(None)
        return out

    def turn(self, i: int, total_deg: float, per_step: float = 5.0, start_deg: float = 0.0) -> float:
        """Turn the driver, engaged in screw ``i``, from ``start_deg`` by ``total_deg`` about the screw's axis."""
        n = max(1, int(round(abs(total_deg) / per_step)))
        self.driver_at(i, start_deg, depth=-0.010)  # in front of the hole, then in
        self.step()
        self.driver_at(i, start_deg)
        self.step()
        for k in range(1, n + 1):
            self.driver_at(i, start_deg + total_deg * k / n)
            self.step()
        return self.m.STATE["turn"][i]

    def hold(self, seconds: float) -> float:
        return self.step(max(1, int(round(seconds / DT))))

    def park_driver(self) -> None:
        b = self.m.mk.MECH.bodies["driver"]
        self.poses["driver"] = (np.array(b.pos, float), b.rotation())


def test_scene_caps_and_layout(m):
    P = m.P
    assert TASK.stat().st_size <= 15_000
    assert (m.SCENE, m.SCORE_OVER, m.SUCCESS_HOLD_S) == ("station", "end", 1.0)
    assert "spare tooth" in m.INSTRUCTION and "clicks" in m.INSTRUCTION and "top terminals" in m.INSTRUCTION
    assert (1e3 * P.MCB_W, 1e3 * P.MCB_H, 1e3 * P.MCB_D, P.TORQUE_SET, 1e3 * P.PITCH) == pytest.approx((18.0, 85.0, 78.5, 2.0, 18.0))
    assert P.TILT_DEG == 30.0 and len(P.KEY_FIGURES) >= 10 and all(f.kind in ("maker", "standard", "listing", "estimate", "maker / estimate", "our fixture") for f in P.KEY_FIGURES)
    assert set(m.mk.MECH.joints) == {"cap_on", "comb_hold"} and not any(j.made for j in m.mk.MECH.joints.values())
    assert (m.mk.MECH.joints["comb_hold"].child, m.mk.MECH.joints["comb_hold"].parent) == ("comb", "panel")
    assert all(m.mk.MECH.bodies[n].kinematic for n in ("jaw_1", "jaw_2", "jaw_3", "screw_1", "screw_2", "screw_3"))


def test_screw_law_half_turn_to_the_tooth_and_click_at_the_set_torque(m):
    P = m.P
    assert P.GAP0 - P.PIN_T == pytest.approx(0.0004)  # half a turn of an M5 x 0.8 screw
    assert math.degrees(P.touch_turn(True)) == pytest.approx(180.0)
    assert P.clamp_gap(math.pi, True) == pytest.approx(P.PIN_T) and P.clamp_gap(2 * math.pi, True) == pytest.approx(P.PIN_T)
    assert P.clamp_gap(2 * math.pi, False) == pytest.approx(P.GAP0 - P.LEAD)
    assert P.screw_torque(math.pi - 0.01, True) == pytest.approx(P.TORQUE_RUN)
    assert P.screw_torque(P.click_turn(True), True) == pytest.approx(P.TORQUE_SET)
    assert math.degrees(P.click_turn(True)) == pytest.approx(195.0)
    assert P.clamp_gap(P.TURN_MIN, True) == pytest.approx(P.GAP_MAX)


def test_driver_turns_a_screw_only_when_its_bit_is_in_the_hole_along_the_axis(m):
    b = Bench(m)
    assert b.turn(1, 90.0) == pytest.approx(math.radians(90.0), abs=1e-6)
    b2 = Bench(m)
    m.start()
    b2.driver_at(0, 0.0, off=(0.003, 0.0))
    b2.step()
    b2.driver_at(0, 60.0, off=(0.003, 0.0))
    b2.step()
    assert m.STATE["turn"][0] == 0.0 and m.STATE["engaged"] is None  # 3 mm off the axis
    m.start()
    b2.driver_at(2, 0.0, tilt_deg=15.0)
    b2.step()
    b2.driver_at(2, 60.0, tilt_deg=15.0)
    b2.step()
    assert m.STATE["turn"][2] == 0.0  # 15 deg off the screw's axis
    m.start()
    b2.driver_at(2, 0.0, depth=-0.003)
    b2.step()
    b2.driver_at(2, 60.0, depth=-0.003)
    b2.step()
    assert m.STATE["turn"][2] == 0.0  # 3 mm out of the hole


def test_click_needs_a_tooth_and_the_set_torque_then_slips(m):
    b = Bench(m)
    b.seat_comb()
    b.step()
    b.turn(0, 190.0)
    assert m.closed_on_tooth(0) and not m.STATE["clicked"][0]
    b.turn(0, 7.0, start_deg=190.0)  # reaches 195 deg (2.0 N m) and slips 2 deg
    assert m.STATE["turn"][0] == pytest.approx(P_click(m)) and not m.STATE["clicked"][0]
    b.turn(0, 2.0, start_deg=197.0)
    assert m.STATE["clicked"][0] and m.STATE["turn"][0] == pytest.approx(P_click(m))
    assert m.STATE["gap"][0] == pytest.approx(m.P.PIN_T)
    m.start()
    b.park_driver()
    b.poses["comb"] = (np.array(m.mk.MECH.bodies["comb"].pos, float), m.mk.MECH.bodies["comb"].rotation())  # back on its rack
    b.step()
    b.turn(1, 230.0)
    assert not m.STATE["clicked"][1] and m.STATE["gap"][1] < m.P.PIN_T  # an empty clamp closes on past a tooth's width


def P_click(m) -> float:
    return m.P.click_turn(True)


def test_turning_back_loosens_and_clears_the_click(m):
    b = Bench(m)
    b.seat_comb()
    b.step()
    b.turn(2, 220.0)
    assert m.STATE["clicked"][2]
    b.turn(2, -30.0, start_deg=220.0)
    assert not m.STATE["clicked"][2] and m.STATE["turn"][2] < P_click(m)


def test_the_bits_play_takes_a_small_turn_back_without_loosening(m):
    b = Bench(m)
    b.seat_comb()
    b.step()
    b.turn(1, 220.0)
    set_turn = m.STATE["turn"][1]
    assert m.STATE["clicked"][1]
    for deg in (219.0, 218.0, 217.0):  # 3 deg back: inside the bit's 4 deg of play in the recess
        b.driver_at(1, deg)
        b.step()
    assert m.STATE["clicked"][1] and m.STATE["turn"][1] == pytest.approx(set_turn)
    for deg in (215.0, 213.0):  # 7 deg back: 3 deg past the play loosens it and clears the click
        b.driver_at(1, deg)
        b.step()
    assert not m.STATE["clicked"][1] and math.degrees(set_turn - m.STATE["turn"][1]) == pytest.approx(3.0, abs=1e-6)
    b.driver_at(1, 216.0)  # forward again: the play first (4 deg), then the screw
    b.step()
    assert math.degrees(set_turn - m.STATE["turn"][1]) == pytest.approx(3.0, abs=1e-6)


def full_trial(m, *, clicks=(True, True, True), cap=True, closed=(True, True, True)) -> Bench:
    b = Bench(m)
    b.seat_comb(0)
    if cap:
        b.cap_on_tooth(3)
    b.step()
    for i in range(3):
        if closed[i]:
            b.turn(i, 225.0 if clicks[i] else 190.0)
    b.park_driver()
    p, r = b.poses["driver"]
    b.poses["driver"] = (p + np.array([0.005, 0.0, 0.0]), r)  # being set down: 5 mm in a control step
    b.step()
    b.poses["driver"] = (p, r)
    b.step()
    return b


def test_full_score_needs_three_clicks_the_cap_rest_and_clear_grippers(m):
    b = full_trial(m)
    assert m.STATE["cap_on"] == 3 and m.spare_tooth() == 3 and m.seated() == {0: 2, 1: 1, 2: 0}
    assert b.hold(0.95) == 0.0
    assert b.hold(0.10) == 1.0


def test_under_torqued_and_bare_tooth_score_half(m):
    full_trial(m, clicks=(True, False, True)).hold(1.1)
    assert m.score(None) == 0.5
    m.start()
    full_trial(m, cap=False).hold(1.1)
    assert m.score(None) == 0.5


def test_a_terminal_left_open_or_a_comb_not_down_scores_zero(m):
    full_trial(m, closed=(True, True, False)).hold(1.1)
    assert m.score(None) == 0.0
    m.start()
    b = Bench(m)
    b.seat_comb(0, lift=0.006)  # 6 mm proud: in the clamp, not seated
    b.step()
    for i in range(3):
        b.turn(i, 225.0)
    b.park_driver()
    assert b.hold(1.1) == 0.0


def test_cap_snaps_from_22_mm_and_holds(m):
    b = Bench(m)
    b.seat_comb(0)
    b.cap_on_tooth(3, depth=0.020)
    b.step()
    assert m.STATE["cap_on"] is None
    b.cap_on_tooth(3, depth=0.0225)
    b.step()
    assert m.STATE["cap_on"] == 3 and not m.mk.is_cut("cap_on")


def test_clamp_welds_the_comb_where_it_stands_until_loosened(m):
    b = Bench(m)
    b.seat_comb()
    b.step()
    assert m.mk.is_cut("comb_hold")
    b.turn(1, 175.0)  # 5 deg short of the plate's touch: no hold
    assert m.mk.is_cut("comb_hold")
    b.turn(1, 10.0, start_deg=175.0)  # on the tooth: welded to the panel as the comb stands
    assert not m.mk.is_cut("comb_hold") and m.closed_on_tooth(1)
    assert np.allclose(m.mk.MECH.joined["comb_hold"], m.P.TILT.T @ (b.poses["comb"][0] - m.P.pw(np.zeros(3)) - np.array([0.0, 0.0, m.TOP])))
    b.turn(1, -20.0, start_deg=185.0)  # turned back off the tooth (past the bit's play): cut
    assert m.mk.is_cut("comb_hold") and not m.closed_on_tooth(1)


def test_off_bench_is_a_latched_zero(m):
    b = full_trial(m)
    b.hold(1.1)
    assert m.score(None) == 1.0
    b.poses["driver"] = (np.array([0.2, -0.4, m.TOP - 0.05]), np.eye(3))  # off the bench
    b.step()
    b.park_driver()
    assert b.hold(1.5) == 0.0 and m.STATE["fault"] is not None


def test_grippers_29_vs_31_mm(m):
    b = full_trial(m)
    cp, cr = b.poses["comb"]
    top = cp + (0.5 * m.P.COMB_BAR[2] + 0.002) * cr[:, 2]
    b.grippers = {"left": (top + 0.029 * cr[:, 2], 1.0), "right": AWAY["right"]}
    assert b.hold(1.2) == 0.0
    b.grippers = {"left": (top + 0.031 * cr[:, 2], 1.0), "right": AWAY["right"]}
    assert b.step() == 1.0
