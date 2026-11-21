"""The instant photo's rules without the simulator: states are set directly and the task file's own functions read
them. Run in the task's environment from the code folder: ``python -m pytest -q tasks/h07_instant_photo/tests``."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from rta import spec
from rta.spec import ObjectState, Snapshot, Trial

TASK = Path(__file__).resolve().parents[1] / "task.py"


@pytest.fixture(scope="module")
def task():
    return spec.load(TASK)


@pytest.fixture(scope="module")
def m(task):
    return task.module


def quat_y(deg):
    """Rotation about the world y axis (positive turns the lens down)."""
    a = math.radians(deg) / 2
    return np.array([0.0, math.sin(a), 0.0, math.cos(a)])


def rot_of(q):
    return spec.quat_matrix(q)


def box_state(centre, half, quat=(0, 0, 0, 1.0), vel=(0, 0, 0)):
    """An ObjectState whose world-aligned box is the rotated local box ``half`` around ``centre``."""
    r = rot_of(np.asarray(quat, float))
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * np.asarray(half, float)
    w = corners @ r.T + np.asarray(centre, float)
    return ObjectState(np.asarray(centre, float), np.asarray(quat, float), np.asarray(vel, float), np.zeros(3), w.min(0), w.max(0))


def camera(m, origin=None, quat=(0, 0, 0, 1.0), vel=(0, 0, 0)):
    """The camera with its frame's origin (on its underside, under its centre) at ``origin``."""
    origin = np.array([m.CAM_XY[0], m.CAM_XY[1], m.TOP]) if origin is None else np.asarray(origin, float)
    half = np.array([m.L["extent"]["camera"] / 2, m.CAM_SIZE[1] / 2, m.CAM_SIZE[2] / 2])
    return box_state(origin + rot_of(np.asarray(quat, float)) @ m.BOX_C, half, quat, vel)


def print_at(m, centre, quat=(0, 0, 0, 1.0), vel=(0, 0, 0)):
    return box_state(centre, np.asarray(m.PRINT) / 2, quat, vel)


def print_in(m, cam, depth):
    """The print in the camera's slot with its centre ``depth`` along the lens axis (the rollers' place for it)."""
    return print_at(m, m.to_world(cam, (depth, 0.0, m.PRINT_Z + m.PRINT[2] / 2)), cam.quat_xyzw)


def apple(m):
    c = (m.APPLE_XY[0], m.APPLE_XY[1], m.TOP + m.APPLE_EXT / 2)
    return box_state(c, (m.APPLE_EXT / 2,) * 3)


PARKED = {"left": (0.12, 0.30, 0.91), "right": (0.12, -0.30, 0.91)}
FACE_DOWN = (1.0, 0.0, 0.0, 0.0)  # the print turned over about x: its picture side down


DOWN = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])  # pointing down, closing along world y


def across(pr):
    """A hand rotation that closes the fingers along the print's face normal (a pinch across its faces)."""
    n = rot_of(pr.quat_xyzw)[:, 2]
    w = np.cross(n, [1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.cross(n, [0.0, 1.0, 0.0])
    w /= np.linalg.norm(w)
    return np.column_stack([n, w, np.cross(n, w)])


def snap(m, t, cam, pr, shutter_in=0.0, shield=0.0, tcp=None, finger=None, tcp_rot=None):
    """``shield`` is the flap's angle in the lifted-up-positive convention (what ``shield_angle`` returns); the joint's
    own positive is the free edge down, so it is stored negated."""
    return Snapshot({"camera": cam, "print": pr, "apple": apple(m)}, t=t, tcp=tcp or PARKED,
                    finger=finger or {"left": -0.047, "right": -0.047},
                    joints={"camera": {"shutter": -shutter_in, "shield": -shield}},
                    tcp_rot=tcp_rot or {"left": DOWN, "right": DOWN})


@pytest.fixture()
def state(m):
    return m.setup(np.random.default_rng(0), Trial(scene_id="seed-0", init_seed=0, seed=0))


class World:
    """What ``events`` sees: the step's objects, and a pose setter and a joint-target setter that record."""

    def __init__(self, objects):
        self.objects = objects
        self.set = {}
        self.targets = {}

    def set_pose(self, name, pos, quat=None):
        self.set[name] = (np.asarray(pos, float), quat)

    def set_joint_target(self, name, joint, target):
        self.targets[(name, joint)] = float(target)


# --- the start -----------------------------------------------------------------------------------------------------
def test_one_start_for_every_seed(m):
    """The task has one start: its state and placements do not depend on the seed (yam_memory spawns without jitter)."""
    starts = [(m.setup(np.random.default_rng(i), Trial(scene_id=f"seed-{i}", init_seed=i, seed=i)),) for i in range(10)]
    assert all(s == starts[0] for s in starts)
    assert all(m.placements(s[0]) == m.placements(starts[0][0]) for s in starts)


def test_face_down_is_read_from_the_print_itself(m):
    """The picture is the model's +z face (assets/build.py): the score reads the print body's own rotation."""
    up = print_at(m, (0.36, -0.04, m.TOP + 0.002))
    down = print_at(m, (0.36, -0.04, m.TOP + 0.002), FACE_DOWN)
    assert m.picture_up(up) == pytest.approx(1.0) and m.picture_up(down) == pytest.approx(-1.0)


# --- the shutter ---------------------------------------------------------------------------------------------------
def test_press_short_of_home_takes_no_photo(m, state):
    cam = camera(m)
    s = snap(m, 3.0, cam, print_in(m, cam, m.HIDDEN_X), shutter_in=0.99 * m.PRESSED)
    assert m.milestones(s, state) == [False] * 4 and m.failure(s, state) is None


def test_first_press_home_takes_the_photo(m, state):
    cam = camera(m)
    angle, dist = m.aim({"camera": cam, "apple": apple(m)})
    assert angle < 1.0 and m.AIM_DIST[0] <= dist <= m.AIM_DIST[1]
    s = snap(m, 3.0, cam, print_in(m, cam, m.HIDDEN_X), shutter_in=1.01 * m.PRESSED)
    assert m.milestones(s, state) == [True, False, False, False] and state["shot_t"] == 3.0
    assert state["release_t"] is None  # still held down


@pytest.mark.parametrize("how", ["tilted", "moving", "turned away", "too near", "too far"])
def test_first_press_home_off_the_shot_wastes_the_exposure(m, state, how):
    if how == "tilted":
        cam = camera(m, quat=quat_y(m.LEVEL_DEG + 3))
    elif how == "moving":
        cam = camera(m, vel=(1.5 * m.STEADY, 0.0, 0.0))
    elif how == "turned away":
        a = math.radians(m.AIM_DEG + 3) / 2
        cam = camera(m, quat=(0.0, 0.0, math.sin(a), math.cos(a)))
    else:  # moved along its lens axis: the apple 10 % inside the near end of AIM_DIST, or 10 % past its far end
        dist = m.aim({"camera": camera(m), "apple": apple(m)})[1]
        to = 0.9 * m.AIM_DIST[0] if how == "too near" else 1.1 * m.AIM_DIST[1]
        cam = camera(m, origin=(m.CAM_XY[0] + dist - to, m.CAM_XY[1], m.TOP))
        angle, now = m.aim({"camera": cam, "apple": apple(m)})
        assert angle < 1.0 and now == pytest.approx(to)
    s = snap(m, 3.0, cam, print_in(m, cam, m.HIDDEN_X), shutter_in=m.SHUTTER_TRAVEL)
    assert m.failure(s, state) == "wasted_exposure" and state["shot_t"] is None
    good = camera(m)
    later = snap(m, 6.0, good, print_in(m, good, m.HIDDEN_X), shutter_in=m.SHUTTER_TRAVEL)
    assert m.milestones(later, state)[0] is False  # one exposure: the first press home was it


def test_print_comes_out_only_when_the_button_is_let_go(m, state):
    cam = camera(m)
    pr = print_in(m, cam, m.HIDDEN_X)
    m.milestones(snap(m, 3.0, cam, pr, shutter_in=m.SHUTTER_TRAVEL), state)
    w = World(snap(m, 4.0, cam, pr, shutter_in=m.SHUTTER_TRAVEL))
    m.milestones(w.objects, state)
    m.events(4.0, state, w)
    assert state["release_t"] is None and w.set["print"][0][0] == pytest.approx(m.to_world(cam, (m.HIDDEN_X, 0, m.PRINT_Z))[0])
    up = snap(m, 4.5, cam, pr, shutter_in=0.9 * m.RELEASED)
    m.milestones(up, state)
    assert state["release_t"] == 4.5
    w = World(snap(m, 4.5 + m.EJECT_S / 2, cam, pr))
    m.events(w.objects.t, state, w)
    half = (m.HIDDEN_X + m.EJECTED_X) / 2
    assert w.set["print"][0][0] == pytest.approx(m.to_world(cam, (half, 0, m.PRINT_Z))[0], abs=1e-6) and state["out_t"] is None
    # placed on the tunnel's floor, picture side up; the shield stands CLEAR_RAD up (its weight taken off), the nip open
    assert w.set["print"][0][2] == pytest.approx(m.TOP + m.FLOOR)
    clear = m.CLEAR_RAD + m.shield_gravity(w.objects) / m.SHIELD_K
    assert w.targets[("camera", "shield")] == pytest.approx(-clear) and w.targets[("camera", "rollers")] == m.NIP_OPEN
    w = World(snap(m, 4.5 + m.EJECT_S, cam, pr))
    m.events(w.objects.t, state, w)
    assert state["out_t"] == pytest.approx(4.5 + m.EJECT_S) and w.targets[("camera", "rollers")] == m.NIP_SHUT
    assert w.set["print"][0][0] == pytest.approx(m.to_world(cam, (m.EJECTED_X, 0, m.PRINT_Z))[0], abs=1e-6)
    assert state["shield_n"] == 0.0  # out: the press has not risen yet


def ejected_state(m, state, out_t=10.0):
    state.update(shot_t=out_t - m.EJECT_S - 1.0, release_t=out_t - m.EJECT_S, out_t=out_t, came_out=True,
                 _depth=m.EJECTED_X, _depth_t=out_t)
    return state


# --- the shield and the rollers' nip -------------------------------------------------------------------------------
def pull_to(m, state, cam, depth, t, **kw):
    """One control step of ``events`` with the print's centre at ``depth`` along the slot (where physics left it)."""
    w = World(snap(m, t, cam, print_in(m, cam, depth), **kw))
    m.events(t, state, w)
    return w


def pressing(m, w, state):
    """The torque (N m) pressing the flap's free edge down: its spring's, K (angle - rest), and its own weight's."""
    return m.SHIELD_K * (m.shield_angle(w.objects) - state["shield_rest"]) + m.shield_gravity(w.objects)


def test_the_ridge_presses_the_print_over_the_lip_and_the_nip_closes_behind_it(m):
    """Once out the print's inner end is INNER inside the front face; the ridge's line lies over the print and over the
    slot's lower lip (the press has the lip right under it); the shield reaches past the print's edges (a fingertip
    finds it beside the print); the nip's block lies behind the print's inner end, and up in the roof while open."""
    out_inner, lip, ridge = m.EJECTED_X - m.PRINT[0] / 2, m.T["lip"], m.FRONT + m.S["ridge"]
    assert out_inner == pytest.approx(m.FRONT - m.INNER) and m.PROTRUDE == pytest.approx(m.PRINT[0] - m.INNER)
    assert out_inner < ridge < m.FRONT + lip and m.S["hinge"][0] == pytest.approx(m.FRONT)
    assert m.L["shield"]["width"] / 2 - m.PRINT[1] / 2 >= 0.009 and m.T["lip_width"] < m.PRINT[1]
    lo, hi = m.R["x"]
    assert hi < out_inner and out_inner - hi == pytest.approx(0.0005)
    assert m.FLOOR + m.T["gap"] >= m.FLOOR + m.PRINT[2]  # open, the block's underside (the roof) is over a print inside
    assert m.NIP_SHUT < m.R["travel"][0] and m.NIP_OPEN > m.R["travel"][1]  # pressed onto either stop


def test_forces_are_equal_and_opposite_and_none_once_out(m, state):
    """While the rules place the print its weight is carried, the reaction on the camera; once it is out no force is
    pushed on either (both named, so nothing lingers): what holds it is the shield's contact friction, applied to the
    print and, opposite, to the camera through the hinge and the lip."""
    cam = camera(m)
    inside = m.forces(snap(m, 3.0, cam, print_in(m, cam, m.HIDDEN_X)), state)
    assert np.allclose(inside["print"], -np.asarray(inside["camera"])) and inside["print"][2] == pytest.approx(m.P["mass"] * 9.81)
    ejected_state(m, state)
    for t, depth in ((12.0, m.EJECTED_X), (13.0, m.EJECTED_X + 0.01), (14.0, m.EJECTED_X + 0.03)):
        f = m.forces(snap(m, t, cam, print_in(m, cam, depth)), state)
        assert f == {"print": (0.0, 0.0, 0.0), "camera": (0.0, 0.0, 0.0)}


def test_shield_press_rises_once_the_print_is_out(m, state):
    ejected_state(m, state)
    cam = camera(m)
    pull_to(m, state, cam, m.EJECTED_X, state["out_t"] + m.SHIELD_RAMP_S / 2)
    assert m.SHIELD_MU * state["shield_n"] == pytest.approx(m.HOLD_N / 2)
    pull_to(m, state, cam, m.EJECTED_X, state["out_t"] + m.SHIELD_RAMP_S)
    assert m.SHIELD_MU * state["shield_n"] == pytest.approx(m.HOLD_N)


@pytest.mark.parametrize("turned", ["standing", "lens up", "tilted"])
def test_the_ridge_presses_with_the_set_force_however_the_flap_sits_and_the_camera_turns(m, state, turned):
    """The spring's rest is set under the flap's angle so that its torque and the flap's own weight press the ridge
    with the set force wherever the flap sits and however the camera is turned: press x PRESS_LEVER about the hinge,
    the rod's 4.5 mm plus the hinge's 0.5 mm over the print times the rod's friction (a pull on the print drags the rod
    outward, which turns the flap up: so the press is the set one when the print starts to slide)."""
    ejected_state(m, state)
    quat = {"standing": (0, 0, 0, 1.0), "lens up": quat_y(-90), "tilted": quat_y(-35)}[turned]
    cam = camera(m, origin=(0.25, 0.05, m.TOP + 0.08), quat=quat)
    for lifted in (0.0, 0.01, -0.02):
        w = World(snap(m, 12.0, cam, print_in(m, cam, m.EJECTED_X), shield=lifted))
        m.events(w.objects.t, state, w)
        assert pressing(m, w, state) == pytest.approx(m.HOLD_N / m.SHIELD_MU * m.PRESS_LEVER)
    hinge_over_print = m.S["hinge"][2] - (m.FLOOR + m.PRINT[2])
    assert m.PRESS_LEVER == pytest.approx(m.S["ridge"] + hinge_over_print * m.S["friction"][0])


def test_once_out_nothing_places_the_print_wherever_the_camera_goes(m, state):
    ejected_state(m, state)
    tipped = camera(m, origin=(0.25, 0.05, m.TOP + 0.08), quat=quat_y(-90))  # lifted, the lens turned up
    w = pull_to(m, state, tipped, m.EJECTED_X, 12.0)
    assert "print" not in w.set and state["freed_t"] is None  # held by the shield, not placed
    assert w.targets[("camera", "rollers")] == m.NIP_SHUT  # the nip closed behind it: it rests on it when let go


def test_a_pinch_without_lifting_the_shield_frees_nothing(m, state):
    ejected_state(m, state)
    cam = camera(m)
    pr = print_in(m, cam, m.EJECTED_X)
    pinch = dict(tcp={"left": PARKED["left"], "right": tuple(pr.pos)}, finger={"left": -0.047, "right": -0.001},
                 tcp_rot={"left": DOWN, "right": across(pr)})
    for k in range(20):  # a second of a firm pinch across its faces, the shield still down
        w = pull_to(m, state, cam, m.EJECTED_X, 12.0 + 0.05 * k, **pinch)
    assert state["freed_t"] is None and not state["sliding"] and "print" not in w.set
    assert m.SHIELD_MU * state["shield_n"] == pytest.approx(m.HOLD_N)  # still held with the full hold


def test_the_shield_grips_with_the_hold_while_still_and_the_drag_while_sliding(m, state):
    """The shield's friction (SHIELD_MU x its press) is HOLD_N while the print is still, so a pull below HOLD_N cannot
    start it; DRAG_N while it slides; and back to HOLD_N when it stops. The shield turns about the camera's y and the
    nip moves along its z, so nothing the rules do can push the print along the slot."""
    ejected_state(m, state)
    cam, dt = camera(m), 0.05
    depths = [m.EJECTED_X] * 3 + [m.EJECTED_X + 0.0015 * k for k in range(1, 6)] + [m.EJECTED_X + 0.0075] * 3
    grips = []
    for k, d in enumerate(depths):
        pull_to(m, state, cam, d, 12.0 + dt * k)
        grips.append(m.SHIELD_MU * state["shield_n"])
    assert grips[:3] == pytest.approx([m.HOLD_N] * 3)
    assert grips[3:8] == pytest.approx([m.DRAG_N] * 5)  # 3 cm/s out: sliding
    assert grips[-2:] == pytest.approx([m.HOLD_N] * 2)  # stopped: held again
    assert state["freed_t"] is None  # the shield still down: the print is still held
    usda = (TASK.parent / "assets" / "camera.usda").read_text()
    for kind, name, axis, lo, hi in (("Revolute", "shield", "Y", -45, 3), ("Prismatic", "rollers", "Z", -m.T["gap"], 0)):
        joint = usda[usda.index(f'def Physics{kind}Joint "{name}"'):]
        joint = joint[:joint.index("}")]
        assert f'physics:axis = "{axis}"' in joint and f"lowerLimit = {lo:.6g}" in joint and f"upperLimit = {hi:.6g}" in joint


def test_a_print_pushed_about_in_its_slot_cannot_lift_the_shield(m):
    """The slot leaves the print 0.5 mm of play: pushed up against the rod (a pinch closing on it), its 10 mm inside can
    tilt it at most that much over its length, so the rod rises at most about 0.7 mm: some 9 degrees of the shield,
    well short of LIFT_DEG. Only a lift of the shield itself frees the print."""
    play = m.T["gap"] - m.PRINT[2]
    rise = play + m.S["ridge"] * play / m.INNER
    assert math.degrees(math.asin(rise / m.S["ridge"])) < 0.7 * m.LIFT_DEG


def test_a_touch_below_the_angle_frees_nothing(m, state):
    ejected_state(m, state)
    cam = camera(m)
    for a in (0.0, m.LIFT_RAD * 0.5, m.LIFT_RAD - 0.001):
        pull_to(m, state, cam, m.EJECTED_X, 12.0, shield=a)
    assert state["freed_t"] is None and m.SHIELD_MU * state["shield_n"] == pytest.approx(m.HOLD_N)


def test_lifting_the_shield_past_the_angle_rolls_it_back_and_frees_the_print(m, state):
    """The hold ends only when the shield is up: past LIFT_RAD the spring's rest goes up past the upper stop (by at most
    STOW_STEP from the flap's angle), it rolls back and the press is gone."""
    ejected_state(m, state)
    cam = camera(m)
    pull_to(m, state, cam, m.EJECTED_X, 12.0, shield=m.LIFT_RAD - 0.01)
    assert state["freed_t"] is None
    w = pull_to(m, state, cam, m.EJECTED_X, 12.05, shield=m.LIFT_RAD + 0.01)
    assert state["freed_t"] == 12.05 and state["shield_n"] == 0.0  # nothing presses the print any more
    up = m.SHIELD_UP + math.radians(5)  # 5 degrees past the upper stop: the spring holds it there
    assert state["shield_rest"] == pytest.approx(min(m.LIFT_RAD + 0.01 + m.STOW_STEP, up))  # rolling back
    pull_to(m, state, cam, m.EJECTED_X + 0.03, 13.0, shield=m.SHIELD_UP)  # rolled back; the print drawn out
    assert state["shield_rest"] == pytest.approx(up) and state["freed_t"] == 12.05


@pytest.mark.parametrize("turned", ["standing", "lens up"])
def test_every_shield_target_stays_within_45_degrees_of_its_angle(m, state, turned):
    """A PhysX drive pulls in proportion to its error only within about 45 degrees: every rest angle the rules set lies
    within that of the flap's angle, at any angle between its stops, placed, pressing, sliding or rolling back."""
    quat = (0, 0, 0, 1.0) if turned == "standing" else quat_y(-90)
    cam = camera(m, origin=(0.25, 0.05, m.TOP + 0.08), quat=quat)
    lo, hi = m.S["travel"]
    for phase in ("placed", "still", "sliding", "freed"):
        for a in np.linspace(lo, hi, 25):
            st = m.setup(np.random.default_rng(0), Trial(scene_id="seed-0", init_seed=0, seed=0))
            if phase != "placed":
                ejected_state(m, st)
            st.update(sliding=phase == "sliding", freed_t=11.0 if phase == "freed" else None)
            w = World(snap(m, 12.0, cam, print_in(m, cam, m.EJECTED_X), shield=float(a)))
            rest = m.shield_rest(12.0, st, w.objects)
            assert abs(rest - a) < math.radians(45.0), (phase, math.degrees(a), math.degrees(rest))


def test_the_shields_hold_is_lighter_than_the_cameras_stand(m):
    """The shield's hold and drag are less than the friction that keeps an unheld camera in place on the bench."""
    weight = m.C["mass"] * 9.81
    static, moving = (mu * weight for mu in m.C["foot_friction"])
    assert m.HOLD_N < static and m.DRAG_N < moving


# --- the end -------------------------------------------------------------------------------------------------------
def ended(m, state, freed_after, print_quat=FACE_DOWN, cam_quat=(0, 0, 0, 1.0), print_xy=(0.36, -0.04), tcp=None):
    ejected_state(m, state)
    state.update(freed_t=state["out_t"] + freed_after)
    cam = camera(m, quat=cam_quat)
    pr = print_at(m, (print_xy[0], print_xy[1], m.TOP + m.PRINT[2] / 2), print_quat)
    return snap(m, 40.0, cam, pr, tcp=tcp)


def test_full_score_end(m, state):
    s = ended(m, state, freed_after=m.SHIELD_S + 0.1)
    assert m.milestones(s, state) == [True, True, True, True] and m.success(s, state) and m.failure(s, state) is None


def test_drawn_out_before_the_shield_wait_stops_at_three_quarters(m, state):
    s = ended(m, state, freed_after=m.SHIELD_S - 0.2)
    assert m.milestones(s, state) == [True, True, True, False]


def test_drawn_at_once_then_the_empty_shield_flicked_up_scores_three_quarters(m, state, task):
    """The maker's order (a reader's probe): a pull over HOLD_N draws the print from under the lowered shield at once;
    the empty shield flicked up at out + 6 s does not count: laid face down, camera standing, 0.75."""
    ejected_state(m, state)
    cam = camera(m)
    pull_to(m, state, cam, m.EJECTED_X + 0.07, 12.0)  # drawn free from under the shield, still down
    pull_to(m, state, cam, m.EJECTED_X + 0.07, 16.0, shield=m.LIFT_RAD + 0.01)  # then flicked up
    assert state["drawn_t"] == 12.0 and state["freed_t"] == 16.0 == state["out_t"] + 6.0
    s = snap(m, 40.0, cam, print_at(m, (0.36, -0.04, m.TOP + m.PRINT[2] / 2), FACE_DOWN))
    flags = m.milestones(s, state)
    assert flags == [True, True, True, False] and not m.success(s, state)
    assert run_steps(task, [(flags, False)] * 80)[-1] == (0.75, False)


def test_face_up_or_on_edge_is_not_face_down(m, state):
    assert m.milestones(ended(m, state, 6.0, print_quat=(0, 0, 0, 1.0)), state) == [True, True, False, False]
    a = math.radians(m.FACE_DOWN_DEG + 5) / 2  # face down but tilted past the limit
    tilted = (math.cos(a), 0.0, math.sin(a), 0.0)
    assert m.milestones(ended(m, state, 6.0, print_quat=tilted), state)[2] is False


def test_hand_near_the_print_or_camera_is_not_done(m, state):
    near_print = {"left": PARKED["left"], "right": (0.36, -0.04, m.TOP + m.PRINT[2] + 0.03)}
    assert m.milestones(ended(m, state, 6.0, tcp=near_print), state)[2] is False
    cam_z = m.TOP + m.CAM_SIZE[2] + 0.03
    near_camera = {"left": (m.CAM_XY[0], m.CAM_XY[1], cam_z), "right": PARKED["right"]}
    assert m.milestones(ended(m, state, 6.0, tcp=near_camera), state) == [True, True, True, False]


def test_camera_left_tipped_is_not_standing_and_on_its_side_ends_the_trial(m, state):
    assert m.milestones(ended(m, state, 6.0, cam_quat=quat_y(m.LEVEL_DEG + 5)), state)[3] is False
    s = ended(m, state, 6.0, cam_quat=quat_y(-90))
    s["camera"] = camera(m, origin=(m.CAM_XY[0], m.CAM_XY[1], m.TOP + 0.0529), quat=quat_y(-90))  # lying on its back
    assert m.failure(s, state) == "camera_knocked_over"


def test_off_the_bench(m, state):
    s = ended(m, state, 6.0)
    s["print"] = print_at(m, (0.36, -0.04, m.TOP - 0.2))
    assert m.failure(s, state) == "dropped_off_bench"


# --- the score over a trial: the per-step bookkeeping yam_memory runs (rta.spec.MemoryTask.score_step) -------------
HZ = 20.0


def run_steps(task, rows):
    """Feed (milestones() flags, success()) per control step through the embodiment's own bookkeeping; returns
    (q_score, success) after each step."""
    flags, held, out = [False] * len(task.milestone_names), 0, []
    for now, ok in rows:
        flags, held, success, q = task.score_step(flags, now, held, ok, HZ)
        out.append((q, success))
    return out


def test_events_latch_end_states_are_read_every_step(task):
    assert task.latched == (True, True, False, False) and task.memory is False
    # the photo and the print drawn free stay counted after the print is put back down and picked up again
    assert run_steps(task, [([1, 1, 0, 0], False), ([1, 0, 0, 0], False)])[-1] == (0.5, False)


def test_end_held_one_second_then_the_camera_knocked_over_is_not_full(task):
    end = ([1, 1, 1, 1], True)  # print face down, camera standing, hands off, the wait kept
    rows = [end] * round(1.0 * HZ) + [([1, 1, 1, 0], False)] * 10  # 1 s, then the camera falls over
    qs = [q for q, _ in run_steps(task, rows)]
    assert max(qs) == 0.75 and qs[-1] == 0.75  # never 1.0 before the 3 s hold


def test_print_flipped_face_up_after_three_quarters_scores_half(task):
    rows = [([1, 1, 1, 0], False)] * 10 + [([1, 1, 0, 0], False)] * 5
    out = run_steps(task, rows)
    assert out[9][0] == 0.75 and out[-1][0] == 0.5


def test_full_score_only_after_the_three_second_hold(task):
    hold = round(task.success_hold_s * HZ)
    out = run_steps(task, [([1, 1, 1, 1], True)] * hold)
    assert all(r == (0.75, False) for r in out[:-1]) and out[-1] == (1.0, True)
