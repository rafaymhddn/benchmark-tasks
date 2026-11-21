"""Pure rule tests for ``adjustable_wrench`` (no simulator).

    scripts/rta check mech/tasks/adjustable_wrench
"""

from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np
import pytest
from yam_arena import task_file

TASK = Path(__file__).resolve().parent / "task.py"
OWN = ("mechkit", "parts", "demo_kit")
DT = 0.05
AWAY = {"left": (np.array([0.25, 0.35, 1.05]), 1.0), "right": (np.array([0.25, -0.35, 1.05]), 1.0)}


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


class Bench:
    """Drives the rules step by step: the wrench's pose, the joints, the hands, the speeds."""

    def __init__(self, m) -> None:
        self.m, self.t = m, 0.0
        self.o, self.rw = (np.array(v, dtype=float) for v in m.START["wrench"])
        self.joints = {"nut_thread": 0.0, "jaw": 0.0, "knurl": 0.0}
        self.grippers = dict(AWAY)
        self.speed = 0.0
        self.finger = True  # a fingertip on the knurl (only a finger rolls it)

    def jaw(self, mm: float) -> Bench:
        """The jaw at ``mm``, the knurl turned to match (as a finger rolling it leaves them)."""
        self.joints["jaw"] = mm / 1000.0 - self.m.P.JAW_START
        self.joints["knurl"] = (self.m.P.JAW_START - mm / 1000.0) / self.m.P.KNURL_PITCH * 2 * math.pi + self.m.STATE.get("slip", 0.0)
        return self

    def pushed(self, mm: float) -> Bench:
        """The jaw at ``mm`` with the knurl where it was: the nut pushed it (or it relaxed back)."""
        self.joints["jaw"] = mm / 1000.0 - self.m.P.JAW_START
        return self

    def nut(self, deg: float) -> Bench:
        self.joints["nut_thread"] = math.radians(deg)
        return self

    def turn(self, deg: float) -> Bench:
        """The wrench turned about the screw's axis (as the nut turns it)."""
        r = self.m.mk.axis_angle((0.0, 0.0, 1.0), math.radians(deg))
        axis = np.array([*self.m.P.NUT_XY, 0.0])
        self.o, self.rw = axis + r @ (self.o - axis), r @ self.rw
        return self

    def slide(self, metres: float) -> Bench:
        """The wrench moved along its jaws' axis away from the tips (as the reference run takes it off)."""
        self.o = self.o - self.rw[:, 0] * metres
        return self

    def hold(self, seconds: float = DT) -> float:
        mk, out = self.m.mk, 0.0
        for _ in range(max(1, int(round(seconds / DT)))):
            self.t = round(self.t + DT, 6)
            jaw_pose = mk.pose("movable_jaw") if mk.MECH.poses else mk.built_pose("movable_jaw")
            vel = {"wrench": (np.array([self.speed, 0.0, 0.0]), np.zeros(3))}
            touch = [mk.Contact("left robot", 0, -1, np.zeros(3), np.array([0.0, 0.0, 1.0]), 0.0, 2.0)] if self.finger else []
            mk.set_state({"wrench": (self.o, self.rw), "movable_jaw": (jaw_pose[0] + 0.0, jaw_pose[1])}, vel, joints=self.joints,
                         grippers=self.grippers, time=self.t, contacts={"knurl": touch})  # fmt: skip
            for name, value in self.joints.items():
                mk.MECH.travel[name] = float(value)
            self.m.step()
            out = self.m.score(None)
            self.o = self.o + np.array([self.speed * DT, 0.0, 0.0])  # a moving wrench: next step's pose
        return out


def test_start_scores_zero_with_the_jaw_open_wider_than_the_corners(m):
    b = Bench(m)
    assert b.hold() == 0.0
    assert m.jaw_mm() == pytest.approx(23.0)
    assert 1000 * m.P.NUT_E < 23.0 and not m.snug() and not m.clear_of_nut() and m.flat()


def test_one_knurl_turn_closes_the_jaw_by_the_pitch(m):
    assert 1000 * m.commanded_m(2 * math.pi, 0.0) == pytest.approx(23.0 - 2.9)
    assert 1000 * m.commanded_m(-2 * math.pi, 0.0) == pytest.approx(25.9)
    assert 1000 * m.commanded_m(100.0, 0.0) == 0.0 and 1000 * m.commanded_m(-100.0, 0.0) == pytest.approx(27.0)


def test_a_knurl_rolled_on_against_a_snug_jaw_slips_and_backs_off_at_once(m):
    b = Bench(m).jaw(17.0)
    b.joints["knurl"] = 3 * 2 * math.pi  # three turns: 8.7 mm asked, the nut stops the jaw at 17.0
    b.hold()
    floor = 17.0 - 1000 * m.P.SNUG_FORCE / m.P.JAW_SPRING
    assert 1000 * m.commanded_m(b.joints["knurl"], m.STATE["slip"]) == pytest.approx(floor, abs=1e-6)
    b.joints["knurl"] -= 0.2 * 2 * math.pi  # a fifth of a turn back opens the jaw past where it stands
    b.hold()
    assert 1000 * m.commanded_m(b.joints["knurl"], m.STATE["slip"]) == pytest.approx(floor + 0.2 * 2.9, abs=1e-6)


def test_the_jaw_cannot_be_pushed_open_past_the_tightest_place_the_knurl_set(m):
    b = Bench(m).jaw(17.1)
    b.joints["knurl"] = (23.0 - 17.1) / 2.9 * 2 * math.pi
    b.hold()
    assert 1000 * m.STATE["w_min"] == pytest.approx(17.1, abs=1e-6)
    b.pushed(17.25).hold()  # pushed open 0.15 mm by the nut: the lock stays where it was
    assert 1000 * m.STATE["w_min"] == pytest.approx(17.1, abs=1e-6)


def test_a_jaw_pushed_open_by_the_nut_moves_neither_the_lock_nor_the_knurl(m):
    """Run gw3: the turn pushed the jaw past its stop, the drive followed it and the lock ratcheted open to 18.6 mm."""
    b = Bench(m).jaw(17.0)
    b.joints["knurl"] = 3 * 2 * math.pi
    b.hold()
    slip, floor = m.STATE["slip"], 17.0 - 1000 * m.P.SNUG_FORCE / m.P.JAW_SPRING
    for opened in (17.3, 17.6, 18.5):
        b.pushed(opened).hold()
        assert 1000 * m.STATE["w_min"] == pytest.approx(17.0, abs=1e-6)
        assert m.STATE["slip"] == pytest.approx(slip)
        assert 1000 * m.commanded_m(b.joints["knurl"], m.STATE["slip"]) == pytest.approx(floor, abs=1e-6)


def test_a_loaded_worm_locks_knurl_turns_do_not_move_the_jaw(m):
    b = Bench(m).jaw(17.0)
    b.joints["knurl"] = (23.0 - 17.0) / 2.9 * 2 * math.pi
    b.hold()
    before = m.commanded_m(b.joints["knurl"], m.STATE["slip"])
    b.joints["knurl"] -= 0.4 * 2 * math.pi  # turned back 0.4 of a turn while the nut loads the jaw with 150 N
    contacts = {("movable_jaw", "nut"): 150.0, "knurl": [m.mk.Contact("left robot", 0, -1, np.zeros(3), np.array([0.0, 0.0, 1.0]), 0.0, 2.0)]}
    mk = m.mk
    b.t += DT
    mk.set_state({"wrench": (b.o, b.rw)}, joints=b.joints, grippers=b.grippers, time=b.t, contacts=contacts)
    for name, value in b.joints.items():
        mk.MECH.travel[name] = float(value)
    m.step()
    assert m.STATE["loaded"]
    assert m.commanded_m(b.joints["knurl"], m.STATE["slip"]) == pytest.approx(before, abs=1e-9)
    assert 1000 * m.STATE["w_min"] == pytest.approx(17.0, abs=1e-6)
    b.hold()  # unloaded again: the knurl drives the jaw from where it now stands
    assert not m.STATE["loaded"]


def test_an_untouched_knurl_does_not_move_the_jaw(m):
    b = Bench(m).jaw(23.0)
    b.finger = False
    b.joints["knurl"] = 1.5 * 2 * math.pi  # turned without a finger on it (solver creep while the wrench turns)
    b.hold()
    assert 1000 * m.commanded_m(b.joints["knurl"], m.STATE["slip"]) == pytest.approx(23.0, abs=1e-6)
    b.finger = True
    b.joints["knurl"] += 0.5 * 2 * math.pi  # half a turn rolled by a finger: 1.45 mm (the jaw follows within the step)
    b.jaw(23.0 - 1.45).hold()
    assert 1000 * m.commanded_m(b.joints["knurl"], m.STATE["slip"]) == pytest.approx(23.0 - 1.45, abs=1e-6)


def test_the_nut_holds_like_dry_friction_its_rest_follows_within_the_give(m):
    b = Bench(m).nut(30.0)
    b.hold()
    assert math.degrees(m.STATE["nut_rest"]) == pytest.approx(28.0)
    b.nut(29.0).hold()  # within the give: the rest stays, the spring holds the nut there
    assert math.degrees(m.STATE["nut_rest"]) == pytest.approx(28.0)
    assert m.P.NUT_SPRING * m.P.NUT_GIVE == pytest.approx(1.5)


def done(m, *, slide=0.0, jaw=17.2, nut=65.0) -> Bench:
    return Bench(m).jaw(jaw).nut(nut).turn(nut).slide(slide)


def test_full_score_needs_snug_jaw_nut_60_flat_on_the_plate_hands_clear_and_a_second_at_rest(m):
    b = done(m)
    assert b.hold(0.9) == 0.5  # not yet a second at rest
    assert b.hold(0.15) == 1.0


def test_wrench_left_on_the_loosened_nut_scores_full(m):
    assert done(m, slide=0.0).hold(1.2) == 1.0


def test_a_hand_near_the_wrench_keeps_it_at_half(m):
    b = done(m)
    o, rw = b.o, b.rw
    near = o + rw @ m.P.handle_point_w(0.12) + np.array([0.0, 0.0, 0.035])
    b.grippers = {"left": AWAY["left"], "right": (near, 1.0)}
    assert b.hold(1.2) == 0.5
    b.grippers = dict(AWAY)
    assert b.hold(1.1) == 1.0


def test_moving_wrench_is_not_at_rest(m):
    b = done(m)
    for k in range(30):  # wrench and nut turning on together, 1 deg a step: on the nut, never still
        b.nut(66.0 + k).turn(1.0)
        assert b.hold() == 0.5 and m.on_nut()


def test_snug_boundary_17_5_mm(m):
    """The scale's boundary, the nut centred in the jaws (0.25 mm a side; against one jaw the other side is the feeler's
    0.5 mm, tested in the seat test)."""
    b = done(m, jaw=17.5)
    b.o = b.o - b.rw[:, 1] * 0.00025
    assert b.hold(1.2) == 1.0
    m.start()
    b = done(m, jaw=17.51)
    b.o = b.o - b.rw[:, 1] * 0.00025
    assert b.hold(1.2) == 0.25 and m.on_nut()  # loosened with the jaw not snug


def test_loosen_boundary_60_deg(m):
    assert done(m, nut=59.9).hold(1.2) == 0.25  # snug, not loosened far enough
    m.start()
    assert done(m, nut=60.0).hold(1.2) == 1.0


def test_lower_levels(m):
    assert Bench(m).jaw(17.2).nut(10.0).turn(10.0).hold() == 0.25  # rolled snug on the nut, turned too little
    m.start()
    assert Bench(m).jaw(18.4).nut(70.0).hold() == 0.25  # turned with a loose jaw
    m.start()
    assert Bench(m).jaw(18.4).nut(30.0).hold() == 0.0
    m.start()
    assert Bench(m).jaw(23.0).nut(0.0).hold() == 0.0  # an open jaw spins round the nut


def test_a_pull_begun_with_a_loose_jaw_stays_at_a_quarter_even_if_rolled_snug_after(m):
    """Reading 1, B1: roll part way, pull 60 deg with the loose jaw, let go, then roll snug: 0.25, not 1.0."""
    b = Bench(m).jaw(18.4)
    b.hold()
    b.nut(15.0).turn(15.0).hold()  # the turn begins: the jaw's setting (18.4 mm) is latched
    assert 1000 * m.STATE["turn_jaw"] == pytest.approx(18.4, abs=1e-6)
    b.nut(65.0).turn(50.0).hold()
    b.jaw(17.2).hold(1.5)  # rolled snug after the turn, hands clear, still
    assert m.snug() and m.loosened() and not m.snug_turn()
    assert b.hold(0.2) == 0.25


def test_the_setting_is_latched_unloaded_not_as_the_pull_opens_the_jaw(m):
    b = Bench(m).jaw(17.1)
    b.hold()
    b.pushed(17.7).nut(12.0).turn(12.0).hold()  # the pull opens the jaw 0.6 mm as the turn begins
    assert 1000 * m.STATE["turn_jaw"] == pytest.approx(17.1, abs=1e-6)
    b.pushed(17.1).nut(65.0).turn(53.0).hold(1.5)
    assert b.hold(0.2) == 1.0


def test_nut_tightened_past_minus_10_deg_zeroes_the_trial(m):
    b = done(m).nut(-10.5)
    assert b.hold() == 0.0
    b.nut(65.0)
    assert b.hold(1.5) == 0.0
    m.start()
    b = Bench(m).jaw(17.2).nut(-9.9).turn(-9.9)
    assert b.hold() == 0.25  # -9.9 deg: not past the limit; jaw snug on the nut, nut not loosened
    b.nut(65.0).turn(74.9)
    assert b.hold(1.2) == 1.0


def test_a_wrench_lifted_or_taken_off_the_nut_after_the_turn_scores_a_quarter(m):
    """Snug is snug on the nut: off the nut at the end, a snug turn counts as a turn with a loose jaw."""
    b = done(m)
    b.o = b.o + np.array([0.0, 0.0, 0.03])  # held 30 mm up
    assert b.hold(1.2) == 0.25 and m.jaw_mm() <= 17.5 and not m.on_nut()
    b.o = b.o - np.array([0.0, 0.0, 0.03]) + np.array([0.0, -0.25, 0.0])  # on the bench beside the plate
    assert b.hold(1.2) == 0.25
    m.start()
    assert done(m, slide=0.040).hold(1.2) == 0.25  # slid off along its jaws, lying on the plate


def test_a_jaw_closed_away_from_the_nut_scores_zero(m):
    """Review: the open wrench lifted off the nut and its jaw closed in free space is not sized to the nut."""
    b = Bench(m)
    b.o = b.o + np.array([0.0, 0.0, 0.03])  # lifted 30 mm off the nut
    assert b.jaw(17.2).hold() == 0.0 and not m.on_nut()
    m.start()
    b = Bench(m).slide(0.040)  # slid off along its jaws onto the plate, then closed
    assert b.jaw(17.2).hold() == 0.0
    assert b.jaw(10.0).hold() == 0.0
    m.start()
    assert Bench(m).jaw(17.2).hold() == 0.25 and m.on_nut()  # the same jaw closed on the nut


def test_seated_needs_the_axis_in_the_opening_the_faces_on_the_flats_and_the_head_at_the_nut(m):
    b = Bench(m).jaw(17.2)
    assert b.hold() == 0.25
    x, lo, hi, cover = m.seat()
    assert abs(x) < 0.001 and -1e-4 < lo < 3e-4 and -1e-4 < hi < 3e-4 and 0.0055 < cover < 0.0063
    up = np.array([0.0, 0.0, 0.0025])
    b.o = b.o + up  # raised 2.5 mm: the jaws still on about 3.4 mm of the flats
    assert b.hold() == 0.25
    b.o = b.o + up  # 5 mm: on about 0.9 mm
    assert b.hold() == 0.0
    b.o = b.o - 2 * up - b.rw[:, 1] * 0.00045  # moved across: the fixed jaw's face 0.45 mm off its flat
    assert b.hold() == 0.25
    b.o = b.o - b.rw[:, 1] * 0.00015  # 0.6 mm: a 0.5 mm feeler gauge would pass
    assert b.hold() == 0.0
    b.o = b.o + b.rw[:, 1] * 0.0006 - b.rw[:, 0] * 0.024  # pulled back 24 mm: the nut's axis past the jaw tips
    assert b.hold() == 0.0
    b.o = b.o + b.rw[:, 0] * 0.024
    assert b.hold() == 0.25
    b.nut(12.0).hold()  # the nut turned 12 deg in a 17.2 mm jaw (corners into the faces): not on two flats
    assert m.seat()[1] < -0.0005 and b.hold() == 0.0


def test_a_tilted_wrench_is_not_lying_flat(m):
    b = done(m)
    b.rw = m.mk.axis_angle((1.0, 0.0, 0.0), math.radians(16.0)) @ b.rw
    assert b.hold(1.2) == 0.5


def test_start_geometry(m):
    P = m.P
    o, rw = m.START["wrench"]
    knurl = o + rw @ np.array([*P.KNURL_IN_W, 0.0])
    assert math.hypot(*(knurl[:2] - np.array(P.NUT_XY))) > P.WASHER_D / 2 + P.KNURL_D / 2  # the knurl overhangs the washer
    assert knurl[2] - P.KNURL_D / 2 > m.START["plate_top"] + 0.001  # and clears the plate
    end = o + rw @ P.handle_point_w(P.HANDLE_END_S)
    assert end[2] - P.HANDLE_T / 2 == pytest.approx(m.START["plate_top"] + P.TILT_GAP, abs=2e-4)  # the handle's end on the plate
    across = math.atan2(rw[1, 1], rw[0, 1])
    off = math.degrees((across - P.nut_yaw(rw)) % (math.pi / 3))
    assert min(off, 60.0 - off) == pytest.approx(0.0, abs=1e-6)  # two flats parallel to the jaws
    Bench(m).hold()
    x, lo, hi, cover = m.seat()  # the fixed jaw on one flat, the nut against the throat, the open jaw 6 mm off the other
    assert abs(x) < 0.001 and abs(lo) < 1e-4 and hi == pytest.approx(0.006, abs=1e-4) and cover > P.SEAT_COVER
    assert math.radians(P.HANDLE_START_DEG) == pytest.approx(math.radians(210.0))  # back and to the right
    h = rw @ np.array([*P.HANDLE_DIR_IN_W, 0.0])
    assert h[0] < 0 and h[1] < 0 and math.degrees(math.atan2(h[1], h[0])) % 360 == pytest.approx(210.0, abs=0.01)
    assert P.NUT_E > P.NUT_AF and 1000 * P.NUT_E < 23.0  # the start jaw spins round the corners
