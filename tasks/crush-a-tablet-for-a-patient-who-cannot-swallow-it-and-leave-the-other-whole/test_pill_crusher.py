"""pill_crusher without the simulator: the scene (the cue card under the aspirin's cup, its print on the card's back
half clear of the cup's foot, each tablet lying inside its cup, cup 4 holding the applesauce), the thread's rules from
states set directly, and the score's levels."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest
from yam_arena import task_file

TASK = Path(__file__).resolve().parent / "task.py"  # the task file in this test's own folder (it imports the folder's mechkit)


@pytest.fixture(autouse=True)
def clean_registry():
    """Each test loads the task afresh with the folder's own mechkit, and leaves no registry or module behind."""
    before = sys.modules.pop("mechkit", None)
    yield
    kit = sys.modules.pop("mechkit", None)
    if kit is not None:
        kit.MECH.clear()
    if before is not None:
        sys.modules["mechkit"] = before


def load():
    task = task_file.load(TASK)
    return task, task.module, task.module.mk


def test_the_cue_card_lies_under_the_aspirins_cup_and_its_print_is_clear_of_the_cup():
    _, m, kit = load()
    cup, card = kit.MECH.bodies["cup_er"], kit.MECH.bodies["card"]
    assert cup.pos[1] == pytest.approx(card.pos[1])
    c = m.PRINT[m.CARD]
    w, d = c["size_mm"]
    assert cup.pos[0] - card.pos[0] == pytest.approx(-d / 4000.0)  # the middle of the card's front half (towards the arms)
    assert c["cap_mm"] >= 5.0 and c["symbol_mm"] >= 12.0 and c["gap_mm"] >= 1.0  # the spec's least sizes, one line
    foot_r = m.CUP_FOOT_D * 1000 / 2
    cup_v = m.CARD_CUP_V * 1000
    for u0, v0, u1, v1 in c["rects"]:
        assert -w / 2 <= u0 < u1 <= w / 2 and 0.0 <= v0 < v1 <= d / 2  # on the card, on its back half (beyond the cup)
        nu, nv = min(max(0.0, u0), u1), min(max(cup_v, v0), v1)
        assert math.hypot(nu, nv - cup_v) > foot_r  # not under the cup's foot


def test_each_tablet_lies_flat_inside_its_cup():
    _, m, kit = load()
    inner_foot = m.CUP_FOOT_D / 2 - m.CUP_WALL
    for tablet, cup, half in (("tablet_a", "cup_a", m.PLAIN_D / 2), ("tablet_b", "cup_b", m.PLAIN_D / 2), ("tablet_er", "cup_er", m.ER_L / 2)):
        t, c = kit.MECH.bodies[tablet], kit.MECH.bodies[cup]
        assert math.hypot(t.pos[0] - c.pos[0], t.pos[1] - c.pos[1]) + half < inner_foot
        low, high = t.bounds()
        assert t.pos[2] + low[2] > c.pos[2] + m.CUP_WALL  # above the cup's floor
        assert high[2] - low[2] < 0.008  # lying flat


def test_cup_4_is_a_medicine_cup_and_the_applesauce_lies_inside_it():
    _, m, kit = load()
    assert (m.SAUCE_TOP_D, m.SAUCE_FOOT_D, m.SAUCE_H) == (m.CUP_TOP_D, m.CUP_FOOT_D, m.CUP_H)
    discs = [sh for sh in kit.MECH.bodies["sauce"].shapes if sh.kind == "cylinder" and sh.color == m.SAUCE_C]
    assert len(discs) == m.SAUCE_SLICES
    for sh in discs:
        r, length = sh.dims[0], sh.dims[1]
        foot = sh.pos[2] - length / 2
        inner = m.CUP_FOOT_D / 2 + (m.CUP_TOP_D - m.CUP_FOOT_D) / 2 * foot / m.CUP_H - m.CUP_WALL
        assert foot >= m.CUP_WALL - 1e-9 and r < inner  # on the cup's floor, inside its wall
    assert max(sh.pos[2] + sh.dims[1] / 2 for sh in discs) == pytest.approx(m.SAUCE_LEVEL)


def test_the_cups_and_the_crusher_stand_apart():
    _, m, kit = load()
    feet = {"cup_a": m.CUP_TOP_D, "cup_b": m.CUP_TOP_D, "cup_er": m.CUP_TOP_D, "base": m.BASE_D, "top": m.CAP_D, "sauce": m.SAUCE_TOP_D}
    names = list(feet)
    for i, a in enumerate(names):
        for b in names[i + 1 :]:
            pa, pb = np.array(kit.MECH.bodies[a].pos[:2]), np.array(kit.MECH.bodies[b].pos[:2])
            assert np.linalg.norm(pa - pb) > (feet[a] + feet[b]) / 2 + 0.010, (a, b)


# --- the rules, from states set directly ---
def base_pose(kit):
    return kit.built_pose("base")


def top_at(m, kit, face, off=(0.0, 0.0), tilt=0.0, yaw=0.3):
    """The top in line over the base (or off/tilted by the given amounts), its face ``face`` over the well's floor."""
    p, r = base_pose(kit)
    rot = r @ m.rot_z(yaw) @ kit.axis_angle((1.0, 0.0, 0.0), tilt)
    return (p + r @ np.array([off[0], off[1], m.FLOOR + face]), rot)


def tablet_in_well(m, kit, name, at=(0.0, 0.0), over=0.0):
    """Tablet ``name`` lying flat on the well's floor (or ``over`` higher), its middle at ``at`` from the axis."""
    p, r = base_pose(kit)
    half = (m.ER_T if name == "tablet_er" else m.PLAIN_T) / 2
    return (p + r @ np.array([at[0], at[1], m.FLOOR + over + half]), r)


def put(kit, poses, turn=0.0, press=0.0, t=None):
    kit.set_state(poses=poses, joints={"turn": turn, "press": press}, time=t)


def step_at(m, kit, poses, turn=0.0, press=0.0):
    put(kit, poses, turn, press)
    m.thread()
    m.flags()


def taken(m, kit, poses):
    """The top held still at the thread's start for the rest window: taken."""
    for k in range(int(round(m.REST_WINDOW / 0.05)) + 1):
        kit.set_state(poses={**poses, "top": top_at(m, kit, m.F0)}, joints={"turn": 0.0, "press": 0.0}, time=0.05 * k)
        m.thread()
        m.flags()
    assert m.STATE["mode"] == "thread"


def test_a_top_held_in_line_25_cm_above_the_base_is_never_taken():
    _, m, kit = load()
    assert still_for(m, kit, lambda t: {"top": top_at(m, kit, 0.25)}, 1.0) is None
    assert m.STATE["mode"] == "free" and m.STATE["events"] == []


def test_the_take_is_no_wider_than_the_plungers_seat():
    _, m, kit = load()
    play = (m.WELL_D - m.PLUNGER_D) / 2
    assert m.TAKE_OFF == pytest.approx(play)
    for kw in ({"face": m.F0 + 0.0006}, {"face": m.F0, "off": (play + 0.0001, 0.0)}, {"face": m.F0, "tilt": math.radians(3.2)}, {"face": m.F0, "tilt": math.pi}):
        assert still_for(m, kit, lambda t, kw=kw: {"top": top_at(m, kit, **kw)}, 0.5) is None, kw
        assert m.STATE["mode"] == "free", kw
    near = (play - 0.0001) / math.sqrt(2.0)
    assert still_for(m, kit, lambda t: {"top": top_at(m, kit, m.F0 + 0.0004, off=(near, near), tilt=math.radians(2.9))}, 0.5, t0=5.0) is not None
    assert m.STATE["mode"] == "thread"
    assert m.STATE["corrections"][0][1] == pytest.approx(1000 * (play - 0.0001), abs=0.01)  # mm off the axis, recorded


def still_for(m, kit, poses_at, seconds, dt=0.05, t0=0.0):
    """Control steps of ``seconds``: the poses at each step's time from ``poses_at(t)``; the thread rule read at each."""
    n = int(round(seconds / dt))
    for k in range(n + 1):
        t = t0 + k * dt
        kit.set_state(poses=poses_at(t), joints={"turn": 0.0, "press": 0.0}, time=t)
        m.thread()
        if m.STATE["mode"] != "free":
            return t
    return None


def test_a_top_is_taken_only_once_it_rests_in_the_mouth():
    """In the take's reach: sinking 1 mm a control step, or swinging 0.25 deg a step: not taken. Held still there:
    taken after the rest window (0.2 s), also while turned about the base's axis."""
    _, m, kit = load()
    falling = still_for(m, kit, lambda t: {"top": top_at(m, kit, m.F0 + 0.0004 - 0.02 * t)}, 0.5)
    assert falling is None and m.STATE["mode"] == "free"
    _, m, kit = load()
    swinging = still_for(m, kit, lambda t: {"top": top_at(m, kit, m.F0, tilt=math.radians(0.25 * round(t / 0.05)))}, 0.5)
    assert swinging is None
    _, m, kit = load()
    at = still_for(m, kit, lambda t: {"top": top_at(m, kit, m.F0)}, 0.5)
    assert at == pytest.approx(m.REST_WINDOW) and m.STATE["mode"] == "thread"
    _, m, kit = load()
    at = still_for(m, kit, lambda t: {"top": top_at(m, kit, m.F0, yaw=0.3 + 2.0 * t)}, 0.5)  # turned about the axis
    assert at == pytest.approx(m.REST_WINDOW)


def test_the_threads_start_lies_in_the_well():
    """A top held at the thread's start has its plunger in the well, 3.5 mm in; the thread ends over the floor."""
    _, m, _ = load()
    assert m.F0 + m.PLUNGER_H > m.WELL_DEPTH and m.F0 + m.TAKE_REACH < m.WELL_DEPTH
    assert m.D_BOTTOM < m.F0


def test_a_top_dropped_into_the_well_is_caught_at_the_threads_start():
    _, m, kit = load()
    assert still_for(m, kit, lambda t: {"top": top_at(m, kit, 0.002)}, 0.5) == pytest.approx(m.REST_WINDOW)
    assert m.STATE["mode"] == "thread" and m.STATE["d"] == m.F0


def test_turned_fully_down_with_no_tablet_nothing_breaks():
    _, m, kit = load()
    taken(m, kit, {})
    for k in range(0, 41):
        turns = 0.1 * k
        step_at(m, kit, {"top": top_at(m, kit, max(m.D_BOTTOM, m.F0 - m.LEAD * turns))}, turn=2 * math.pi * turns)
        assert m.STATE["mode"] == "thread"
    assert m.STATE["d"] == pytest.approx(m.D_BOTTOM)
    assert m.STATE["crushed"] == {} and m.STATE["most_turns"] == pytest.approx((m.F0 - m.D_BOTTOM) / m.LEAD)
    assert m.STATE["stops"] == pytest.approx(m.D_BOTTOM - m.F0)


def test_tablets_beside_the_faces_footprint_stay_whole():
    """On the bench by the base, and on the base's rim beyond the mouth: a plain tablet and the 7 mm aspirin."""
    _, m, kit = load()
    p, r = base_pose(kit)
    rim = (m.WELL_D / 2 + m.BASE_D / 2) / 2
    beside = {"tablet_a": (p + np.array([0.0, 0.040, m.PLAIN_T / 2]), r), "tablet_b": (p + r @ np.array([0.0, rim, m.BASE_H + m.PLAIN_T / 2]), r),
              "tablet_er": (p + r @ np.array([rim, 0.0, m.BASE_H + m.ER_T / 2]), r)}  # fmt: skip
    taken(m, kit, beside)
    for k in range(0, 41):
        turns = 0.1 * k
        step_at(m, kit, {**beside, "top": top_at(m, kit, max(m.D_BOTTOM, m.F0 - m.LEAD * turns))}, turn=2 * math.pi * turns)
    assert m.STATE["crushed"] == {} and m.STATE["mode"] == "thread" and m.STATE["d"] == pytest.approx(m.D_BOTTOM)
    assert m.STATE["er_in_crusher"] is False


def screw_down_to_contact(m, kit, poses, h):
    """Turn the taken top down until its face meets the highest tablet (``h`` over the floor); return the turn."""
    turns = 0.0
    while m.STATE["mode"] == "thread":
        turns += 0.02
        step_at(m, kit, {**poses, "top": top_at(m, kit, max(h, m.F0 - m.LEAD * turns))}, turn=2 * math.pi * turns)
        assert turns < 4.0
    return 2 * math.pi * turns


def test_a_tablet_under_the_face_gives_only_past_the_give_angle_and_becomes_grains():
    _, m, kit = load()
    poses = {"tablet_a": tablet_in_well(m, kit, "tablet_a", at=(0.004, -0.003))}
    taken(m, kit, poses)
    turn = screw_down_to_contact(m, kit, poses, m.PLAIN_T)
    assert m.STATE["mode"] == "press" and m.STATE["pressing"] == ("tablet_a",)
    assert m.STATE["contact_d"] == pytest.approx(m.PLAIN_T, abs=1e-6)
    assert turn / (2 * math.pi) == pytest.approx((m.F0 - m.PLAIN_T - m.TOUCH_TOL) / m.LEAD, abs=0.03)  # about two turns to the touch
    face = top_at(m, kit, m.PLAIN_T)
    for deg in (5.0, 15.0, 29.0, 29.9):
        step_at(m, kit, {**poses, "top": face}, turn=turn, press=math.radians(deg))
        assert m.STATE["crushed"] == {} and m.STATE["mode"] == "press", deg
    step_at(m, kit, {**poses, "top": face}, turn=turn, press=math.radians(30.1))
    assert list(m.STATE["crushed"]) == ["tablet_a"] and m.STATE["mode"] == "thread"
    assert m.STATE["d"] == pytest.approx(m.PLAIN_T, abs=1e-6)
    spots = m.STATE["spots"]["tablet_a"]
    assert len(spots) == m.GRAINS_PER
    for s in spots:  # on the floor, under the face (clear of the plunger's gap at the wall), clear of each other
        assert math.hypot(s[0], s[1]) <= m.PLUNGER_D / 2 - m.GRAIN_R - 0.001 + 1e-9 and s[2] == pytest.approx(m.FLOOR + m.GRAIN_R + 0.0001)
    pts = np.array(spots)
    gaps = np.linalg.norm(pts[:, None, :2] - pts[None, :, :2], axis=-1) + np.eye(len(pts))
    assert gaps.min() >= 2 * m.GRAIN_R
    assert all(not kit.shown("tablet_a", k) for k in range(len(kit.MECH.bodies["tablet_a"].shapes)))
    assert all(kit.shown(g, 0) for g in m.GRAINS["tablet_a"]) and not any(kit.shown(g, 0) for g in m.GRAINS["tablet_b"])
    # on down to the thread's end: the grains' layer stays clear of the face
    for k in range(1, 30):
        turns = turn / (2 * math.pi) + 0.1 * k
        step_at(m, kit, {**poses, "top": top_at(m, kit, max(m.D_BOTTOM, m.F0 - m.LEAD * turns))}, turn=2 * math.pi * turns)
    assert m.STATE["d"] == pytest.approx(m.D_BOTTOM) and m.D_BOTTOM > 2 * m.GRAIN_R + 0.0005


def test_turned_on_past_the_gives_the_face_stops_over_the_powder():
    """A policy may turn on past the gives. The thread's travel ends POWDER_GAP over the highest grain
    under the face, as the press stops it on a tablet (run w12s18: turned on down onto the grains the crusher was
    thrown); a grain over the face never lifts the top; turned back, it goes up as usual."""
    _, m, kit = load()
    poses = {"tablet_a": tablet_in_well(m, kit, "tablet_a", at=(0.0, 0.0085)), "tablet_b": tablet_in_well(m, kit, "tablet_b", at=(0.0, -0.0085))}
    taken(m, kit, poses)
    turn = screw_down_to_contact(m, kit, poses, m.PLAIN_T)
    step_at(m, kit, {**poses, "top": top_at(m, kit, m.PLAIN_T)}, turn=turn, press=math.radians(31.0))
    assert sorted(m.STATE["crushed"]) == ["tablet_a", "tablet_b"] and m.STATE["mode"] == "thread"
    p, r = base_pose(kit)
    grains = {g: (p + r @ np.asarray(sp), r) for t in ("tablet_a", "tablet_b") for g, sp in zip(m.GRAINS[t], m.STATE["spots"][t])}
    q = turn

    def to(q_new, moved=None):
        step_at(m, kit, {**poses, **grains, **(moved or {}), "top": top_at(m, kit, m.STATE["d"])}, turn=q_new)
        return q_new

    d_give = m.STATE["d"]
    q = to(q + 0.2 * math.pi)  # on down from the give over the grains: no lower (they are within POWDER_GAP), not lifted
    layer = m.powder_top()  # one layer on the floor: their tops 2.5 mm up
    assert layer == pytest.approx(2 * m.GRAIN_R + 0.0001, abs=1e-6) and layer + m.POWDER_GAP > d_give
    assert m.STATE["d"] == pytest.approx(d_give, abs=1e-6)
    for _ in range(10):  # back up a turn, then on down past the thread's end: it stops POWDER_GAP over the layer
        q = to(q - 0.2 * math.pi)
    for _ in range(20):
        q = to(q + 0.2 * math.pi)
    end = max(m.D_BOTTOM, layer + m.POWDER_GAP)
    assert m.STATE["d"] == pytest.approx(end, abs=1e-6) and m.POWDER_GAP > 0.0015  # past the face's and grains' contact offsets
    for _ in range(10):  # back up a turn: 3 mm higher
        q = to(q - 0.2 * math.pi)
    assert m.STATE["d"] == pytest.approx(end + m.LEAD, abs=1e-6)
    a0, a1 = m.GRAINS["tablet_a"][:2]
    stacked = {a0: (grains[a1][0] + r @ np.array([0.0, 0.0, 2 * m.GRAIN_R]), r)}  # one grain on another: its top 4.9 mm up
    top = 4 * m.GRAIN_R + 0.0001
    for _ in range(10):
        q = to(q + 0.2 * math.pi, stacked)
    assert m.STATE["d"] == pytest.approx(top + m.POWDER_GAP, abs=1e-6) and m.STATE["mode"] == "thread"
    flying = {**stacked, a1: (p + r @ np.array([0.0, 0.0, m.FLOOR + 0.008]), r)}  # a grain over the face: the top stays
    q = to(q + 0.2 * math.pi, flying)
    assert m.STATE["d"] == pytest.approx(top + m.POWDER_GAP, abs=1e-6)
    q = to(q - 0.2 * math.pi, flying)  # turned back: up as usual
    assert m.STATE["d"] == pytest.approx(top + m.POWDER_GAP + 0.1 * m.LEAD, abs=1e-6)


def test_a_grain_lying_on_a_whole_tablet_is_pressed_with_it():
    """A grain lying on a whole tablet does not stop the face over it. The face meets the stack's top (the
    grain's) and presses the tablet with the grain on it; past the give the tablet crushes."""
    _, m, kit = load()
    poses = {"tablet_a": tablet_in_well(m, kit, "tablet_a", at=(0.0, 0.0085))}
    taken(m, kit, poses)
    turn = screw_down_to_contact(m, kit, poses, m.PLAIN_T)
    step_at(m, kit, {**poses, "top": top_at(m, kit, m.PLAIN_T)}, turn=turn, press=math.radians(31.0))
    assert list(m.STATE["crushed"]) == ["tablet_a"]
    p, r = base_pose(kit)
    grains = {g: (p + r @ np.asarray(sp), r) for g, sp in zip(m.GRAINS["tablet_a"], m.STATE["spots"]["tablet_a"])}
    q = turn
    for _ in range(15):  # back up a turn and a half
        q -= 0.2 * math.pi
        step_at(m, kit, {**poses, **grains, "top": top_at(m, kit, m.STATE["d"])}, turn=q)
    tb = tablet_in_well(m, kit, "tablet_b", at=(0.0, -0.0085))  # a second tablet put in, one grain lying on it
    on = {m.GRAINS["tablet_a"][0]: (tb[0] + r @ np.array([0.0, 0.0, m.PLAIN_T / 2 + m.GRAIN_R + 0.0001]), r)}
    stack = m.PLAIN_T + 2 * m.GRAIN_R + 0.0001
    world = {**poses, **grains, **on, "tablet_b": tb}
    for _ in range(40):
        q += 0.1 * math.pi
        step_at(m, kit, {**world, "top": top_at(m, kit, m.STATE["d"])}, turn=q)
        if m.STATE["mode"] != "thread":
            break
    assert m.STATE["mode"] == "press" and m.STATE["pressing"] == ("tablet_b",)
    assert m.STATE["contact_d"] == pytest.approx(stack, abs=1e-6) and m.stack_top("tablet_b") == pytest.approx(stack, abs=1e-6)
    assert m.powder_top() + m.POWDER_GAP < stack  # the grain on it is not counted as a stop
    step_at(m, kit, {**world, "top": top_at(m, kit, stack)}, turn=q, press=math.radians(31.0))
    assert sorted(m.STATE["crushed"]) == ["tablet_a", "tablet_b"]


def test_the_pressed_powder_is_held_until_the_base_leans_past_the_slide_angle():
    """From the step after the crush the grains are held on the base (welded: no dynamics) through a jolt,
    the unscrew and the lift; past SLIDE_ANGLE of lean they all slide free, not before; never tipped, none in cup 4."""
    _, m, kit = load()
    poses = {"tablet_a": tablet_in_well(m, kit, "tablet_a", at=(0.0, 0.0085)), "tablet_b": tablet_in_well(m, kit, "tablet_b", at=(0.0, -0.0085))}
    taken(m, kit, poses)
    turn = screw_down_to_contact(m, kit, poses, m.PLAIN_T)
    step_at(m, kit, {**poses, "top": top_at(m, kit, m.PLAIN_T)}, turn=turn, press=math.radians(31.0))
    m.cake()
    assert sorted(m.STATE["crushed"]) == ["tablet_a", "tablet_b"]
    grains = [g for t in ("tablet_a", "tablet_b") for g in m.GRAINS[t]]
    p, r = base_pose(kit)
    spots = {g: (p + r @ np.asarray(sp), r) for t in ("tablet_a", "tablet_b") for g, sp in zip(m.GRAINS[t], m.STATE["spots"][t])}

    def at(base=None, top_face=None, q=turn):
        step_at(m, kit, {**poses, **spots, **({"base": base} if base else {}), "top": top_at(m, kit, m.STATE["d"] if top_face is None else top_face)}, turn=q)
        m.cake()
        return [g for g in grains if not kit.is_cut(f"cake_{g}")]

    assert at() == grains  # held from the step after the crush
    jolt = (p + np.array([0.005, -0.004, 0.003]), r @ kit.axis_angle((1.0, 0.0, 0.0), math.radians(12.0)))
    assert at(base=jolt) == grains and at(q=turn - 2 * math.pi) == grains and at(top_face=0.06) == grains  # a jolt, unscrewed, lifted off
    for deg, held in ((20.0, grains), (29.5, grains), (30.5, [])):
        leaning = (p, r @ kit.axis_angle((0.0, 1.0, 0.0), math.radians(deg)))
        assert at(base=leaning) == held, deg
    assert m.STATE["slid"] == pytest.approx(30.5, abs=0.1) and "slides as grains" in m.STATE["events"][-1][1]
    assert at() == []  # loose for good once slid
    _, m, kit = load()  # never tipped: the powder stays in the well, none in cup 4
    taken(m, kit, poses)
    turn = screw_down_to_contact(m, kit, poses, m.PLAIN_T)
    step_at(m, kit, {**poses, "top": top_at(m, kit, m.PLAIN_T)}, turn=turn, press=math.radians(31.0))
    spots = {g: (p + r @ np.asarray(sp), r) for t in ("tablet_a", "tablet_b") for g, sp in zip(m.GRAINS[t], m.STATE["spots"][t])}
    step_at(m, kit, {**poses, **spots, "top": top_at(m, kit, m.STATE["d"])}, turn=turn)
    m.cake()
    assert m.sauce_share() == 0.0 and m.level() == pytest.approx(0.5)


def test_a_grain_that_touches_the_applesauce_stays_on_it():
    """Wet applesauce holds powder: a grain that touched cup 4 within the control step at the applesauce's surface is
    held there (welded to cup 4) and counts as on it; so is one that bounced off it within the step, where its fall
    met the surface; one that touched cup 4's wall 10 mm over the surface or its outside on the bench, or lies at the
    surface without a touch, stays loose."""
    _, m, kit = load()
    poses = {"tablet_a": tablet_in_well(m, kit, "tablet_a", at=(0.0, 0.0085)), "tablet_b": tablet_in_well(m, kit, "tablet_b", at=(0.0, -0.0085))}
    taken(m, kit, poses)
    turn = screw_down_to_contact(m, kit, poses, m.PLAIN_T)
    step_at(m, kit, {**poses, "top": top_at(m, kit, m.PLAIN_T)}, turn=turn, press=math.radians(31.0))
    assert sorted(m.STATE["crushed"]) == ["tablet_a", "tablet_b"]
    sp, sr = kit.built_pose("sauce")
    on, bare, wall, outside, bounced = m.GRAINS["tablet_a"][:5]
    resting = m.SAUCE_LEVEL + m.GRAIN_R
    where = {on: (0.004, -0.003, resting + 0.002), bare: (-0.005, 0.002, resting), wall: (0.012, 0.0, m.SAUCE_LEVEL + 0.010),
             outside: (m.SAUCE_FOOT_D / 2 + m.GRAIN_R + 0.001, 0.0, m.GRAIN_R - m.LIFT), bounced: (0.003, 0.002, resting + 0.020)}  # fmt: skip

    def read(places, touched, fall=(0.0, 0.0, 0.0)):
        kit.set_state(poses={**poses, **{g: (sp + sr @ np.array(v), sr) for g, v in places.items()}}, joints={"turn": turn},
                      velocities={bounced: (sr @ np.array(fall), np.zeros(3))}, contacts={("sauce", g): 0.02 for g in touched})  # fmt: skip
        m.stick()

    read(where, (on, wall, outside), fall=(0.1, 0.0, -1.0))  # the bounced grain still falling, 20 mm over the surface
    assert m.STATE["stuck"] == [on]
    assert not kit.is_cut(f"stick_{on}") and all(kit.is_cut(f"stick_{g}") for g in (bare, wall, outside, bounced))
    assert m.in_sauce(sp + sr @ np.array(where[on])) and not m.in_sauce(sp + sr @ np.array(where[outside]))
    read({**where, bounced: (0.010, 0.002, resting + 0.015)}, (bounced,))  # back up 15 mm a step later, the touch read
    assert m.STATE["stuck"] == [on, bounced] and not kit.is_cut(f"stick_{bounced}")
    t = (-1.0 + math.sqrt(1.0 + 2 * m.GRAVITY * 0.020)) / m.GRAVITY  # where its fall met the surface
    hit = m.surface_hit(np.array(where[bounced]), np.array([0.1, 0.0, -1.0]), np.zeros(3))
    assert hit == pytest.approx([0.003 + 0.1 * t, 0.002, resting], abs=1e-9)
    read(where, (bare,))  # a touch read later at the surface: held too, and none twice
    assert m.STATE["stuck"] == [on, bounced, bare]


def test_turned_back_before_the_give_the_tablet_stays_whole_and_the_top_goes_back_on_the_thread():
    _, m, kit = load()
    poses = {"tablet_b": tablet_in_well(m, kit, "tablet_b")}
    taken(m, kit, poses)
    turn = screw_down_to_contact(m, kit, poses, m.PLAIN_T)
    step_at(m, kit, {**poses, "top": top_at(m, kit, m.PLAIN_T)}, turn=turn, press=math.radians(20.0))
    step_at(m, kit, {**poses, "top": top_at(m, kit, m.PLAIN_T)}, turn=turn, press=math.radians(-1.5))
    assert m.STATE["mode"] == "thread" and m.STATE["crushed"] == {}
    assert m.STATE["d"] == pytest.approx(m.PLAIN_T + m.TOUCH_TOL + m.UNPRESS_LIFT)
    step_at(m, kit, {**poses, "top": top_at(m, kit, m.STATE["d"])}, turn=turn - 0.2)  # turning back on up: no new touch
    assert m.STATE["mode"] == "thread"


def test_two_stacked_tablets_give_one_after_the_other():
    _, m, kit = load()
    poses = {"tablet_a": tablet_in_well(m, kit, "tablet_a"), "tablet_b": tablet_in_well(m, kit, "tablet_b", at=(0.001, 0.0), over=m.PLAIN_T)}
    taken(m, kit, poses)
    turn = screw_down_to_contact(m, kit, poses, 2 * m.PLAIN_T)
    assert m.STATE["pressing"] == ("tablet_b",)
    step_at(m, kit, {**poses, "top": top_at(m, kit, 2 * m.PLAIN_T)}, turn=turn, press=math.radians(31.0))
    assert list(m.STATE["crushed"]) == ["tablet_b"]
    for s in m.STATE["spots"]["tablet_b"]:  # not under the whole one below
        assert math.hypot(s[0], s[1]) >= m.PLAIN_D / 2 + m.GRAIN_R
    turns = turn / (2 * math.pi)
    while m.STATE["mode"] == "thread":
        turns += 0.02
        step_at(m, kit, {**poses, "top": top_at(m, kit, max(m.PLAIN_T, m.F0 - m.LEAD * turns))}, turn=2 * math.pi * turns)
    assert m.STATE["pressing"] == ("tablet_a",)
    step_at(m, kit, {**poses, "top": top_at(m, kit, m.PLAIN_T)}, turn=2 * math.pi * turns, press=math.radians(31.0))
    assert sorted(m.STATE["crushed"]) == ["tablet_a", "tablet_b"]
    a, b = np.array(m.STATE["spots"]["tablet_a"]), np.array(m.STATE["spots"]["tablet_b"])
    assert np.linalg.norm(a[:, None, :2] - b[None, :, :2], axis=-1).min() >= 2 * m.GRAIN_R


def test_two_tablets_side_by_side_are_pressed_together():
    _, m, kit = load()
    poses = {"tablet_a": tablet_in_well(m, kit, "tablet_a", at=(0.0, 0.007)), "tablet_b": tablet_in_well(m, kit, "tablet_b", at=(0.0, -0.007))}
    taken(m, kit, poses)
    screw_down_to_contact(m, kit, poses, m.PLAIN_T)
    assert sorted(m.STATE["pressing"]) == ["tablet_a", "tablet_b"]


def test_unscrewed_past_the_start_the_top_sits_there_until_lifted_off():
    _, m, kit = load()
    taken(m, kit, {})
    step_at(m, kit, {"top": top_at(m, kit, m.F0)}, turn=-2 * math.pi * 0.24)
    assert m.STATE["mode"] == "thread" and m.STATE["d"] == pytest.approx(m.F0 + 0.24 * m.LEAD)
    sits = m.F0 + m.RELEASE_TURNS * m.LEAD
    for turns in (0.26, 0.5, 1.0):  # turned on back: it sits on the start, held there
        step_at(m, kit, {"top": top_at(m, kit, sits)}, turn=-2 * math.pi * turns)
        assert m.STATE["mode"] == "thread" and not kit.is_cut("on") and m.STATE["d"] == pytest.approx(sits)
    assert sum("sits on the thread's start" in e[1] for e in m.STATE["events"]) == 1
    s = m.RELEASE_TURNS * m.LEAD
    kit.set_state(poses={"top": top_at(m, kit, sits + m.RELEASE_LIFT - 0.0002)}, joints={"turn": -2 * math.pi, "press": 0.0, "along": s + m.RELEASE_LIFT - 0.0002})
    m.thread()
    assert m.STATE["mode"] == "thread"  # lifted 0.8 mm: still on it
    kit.set_state(poses={"top": top_at(m, kit, sits + m.RELEASE_LIFT + 0.0002)}, joints={"turn": -2 * math.pi, "press": 0.0, "along": s + m.RELEASE_LIFT + 0.0002})
    m.thread()
    assert m.STATE["mode"] == "free" and kit.is_cut("on")
    assert still_for(m, kit, lambda t: {"top": top_at(m, kit, m.F0)}, 0.5, t0=1.0) is not None  # set in again and held still: taken again
    assert m.STATE["mode"] == "thread" and m.STATE["d"] == m.F0


# --- the score's levels ---
def grains_in_sauce(m, kit, n_in):
    """The plain tablets' grains: the first n_in on the applesauce, the rest on the bench by the crusher."""
    sp, sr = kit.built_pose("sauce")
    out = {}
    for k, g in enumerate(g for t in m.PLAIN for g in m.GRAINS[t]):
        if k < n_in:
            a, rad = 0.7 * k, 0.002 + 0.0003 * k  # a spiral out to 16 mm: inside cup 4 at the applesauce
            out[g] = (sp + sr @ np.array([rad * math.cos(a), rad * math.sin(a), m.SAUCE_LEVEL + m.GRAIN_R]), np.eye(3))
        else:
            out[g] = (np.array([0.30, -0.05 + 0.003 * k, m.TOP + m.GRAIN_R]), np.eye(3))
    return out


def end_state(m, kit, n_in=48, er=None, extra=None, grip=None):
    """Both plain tablets crushed, the crusher's base and top back where they started, the hands far away."""
    for t in m.PLAIN:
        m.STATE["crushed"][t] = 1.0
    poses = {**grains_in_sauce(m, kit, n_in), **({"tablet_er": er} if er else {}), **(extra or {})}
    grippers = grip or {"left": (np.array([0.0, 0.25, m.TOP + 0.3]), 0.08), "right": (np.array([0.0, -0.25, m.TOP + 0.3]), 0.08)}
    for t in (10.0, 10.3):
        kit.set_state(poses=poses, grippers=grippers, time=t)
        m.track()


def test_an_idle_run_scores_0():
    _, m, kit = load()
    kit.set_state(time=0.0)
    m.track()
    kit.set_state(time=0.3)
    m.track()
    assert m.score(None) == 0.0


def test_full_score_and_each_condition_of_it():
    _, m, kit = load()
    end_state(m, kit)
    assert m.score(None) == 1.0
    end_state(m, kit, n_in=44)
    assert m.score(None) == 1.0  # 44 of 48 is 91.7 %
    end_state(m, kit, n_in=43)
    assert m.score(None) == 0.5
    end_state(m, kit, grip={"left": (np.array([0.0, 0.25, m.TOP + 0.3]), 0.08), "right": (np.array([*m.TOP_XY, m.TOP + 0.037 + 0.02]), 0.08)})
    assert m.score(None) == 0.5  # a hand 2 cm over the top
    end_state(m, kit, extra={"base": (np.array([*m.BASE_XY, m.TOP + 0.02]), np.eye(3))})
    assert m.score(None) == 0.5  # the base held 2 cm up
    m.STATE["mode"] = "thread"
    end_state(m, kit)
    assert m.score(None) == 0.5  # the top still on the base's thread


def test_the_aspirin_tablet_decides_the_score():
    _, m, kit = load()
    sp, sr = kit.built_pose("sauce")
    end_state(m, kit, er=(sp + sr @ np.array([0.0, 0.0, m.SAUCE_LEVEL + m.ER_T / 2]), np.eye(3)))
    assert m.score(None) == 0.0  # in the applesauce at the end
    _, m, kit = load()
    m.STATE["crushed"]["tablet_er"] = 1.0
    end_state(m, kit)
    assert m.score(None) == 0.0  # crushed
    _, m, kit = load()
    m.STATE["er_in_crusher"] = True
    end_state(m, kit)
    assert m.score(None) == 0.25  # whole and back in its cup, but in the crusher at some moment
    _, m, kit = load()
    end_state(m, kit, er=(np.array([0.30, 0.10, m.TOP + m.ER_T / 2]), np.eye(3)))
    assert m.score(None) == 0.0  # whole but loose on the bench


def test_one_plain_tablet_crushed_scores_a_quarter_and_a_loose_one_zero():
    _, m, kit = load()
    m.STATE["crushed"]["tablet_a"] = 1.0
    kit.set_state(time=10.0)
    m.track()
    assert m.score(None) == 0.25
    kit.set_state(poses={"tablet_b": (np.array([0.32, -0.05, m.TOP + m.PLAIN_T / 2]), np.eye(3))}, time=10.3)
    assert m.score(None) == 0.0  # the other one out on the bench


def test_the_aspirin_tablet_in_the_crusher_is_flagged():
    _, m, kit = load()
    step_at(m, kit, {"tablet_er": tablet_in_well(m, kit, "tablet_er")})
    assert m.STATE["er_in_crusher"] is True
