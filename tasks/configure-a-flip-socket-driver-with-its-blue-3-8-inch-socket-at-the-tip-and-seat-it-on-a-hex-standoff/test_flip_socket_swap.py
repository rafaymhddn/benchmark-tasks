"""The flip-socket task's scene, figures and rules without the simulator: every state is set directly (the barrels
where they lie on or off the shaft, the driver where it is carried, both hands' hull corners and grippers)."""

from __future__ import annotations

import math
import re
import sys
from pathlib import Path

import numpy as np
import pytest
from yam_arena import task_file

HERE = Path(__file__).resolve().parent
TASK = HERE / "task.py"
FAR = {"left": np.array([[0.0, 0.30, 1.10]]), "right": np.array([[0.0, -0.30, 1.10]])}  # both hands' hulls far away
OPEN = {"left": ((0.0, 0.3, 1.1), 0.9), "right": ((0.0, -0.3, 1.1), 0.9)}  # both grippers wide open
EYE = np.eye(3)
WORK = "barrel_1132_38"
FRONT = "barrel_14_516"
REAR = "barrel_716_12"


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


def rot_z(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def on_shaft_pose(m, key: str, mid: float, flip: bool, tilt_deg: float = 0.0, clock_deg: float = 0.0, tip_y: float | None = None):
    """A barrel's pose, its axis along the tool's: ``flip`` True when its second-named end (the blue one, for the
    working barrel) points outboard (-y), as the task requires at the tip; ``clock_deg`` rolls and ``tilt_deg`` tips
    it with the tool it rides (about the tool's tip)."""
    pre = m.P.rot_x(math.radians(tilt_deg)) @ m.scene.rot_y(math.radians(clock_deg))
    r = pre @ rot_z(90.0 if flip else -90.0)
    tip = np.array([m.P.DRIVER_X, m.P.TIP_Y if tip_y is None else tip_y, m.TOP + m.P.AXIS_Z])
    return (tip + pre @ np.array([0.0, mid - tip[1], 0.0]), r)


def driver_pose(m, tip_y: float, clock_deg: float = 0.0, tilt_deg: float = 0.0):
    """The driver's pose: rolled ``clock_deg`` about its own axis (the world y) and tipped ``tilt_deg`` nose-up."""
    r = m.scene.rot_y(math.radians(clock_deg)) @ m.scene.DRIVER_START_ROT
    if tilt_deg:
        r = m.P.rot_x(math.radians(tilt_deg)) @ r
    return (np.array([m.P.DRIVER_X, tip_y, m.TOP + m.P.AXIS_Z]), r)


def put(m, poses: dict, hands=FAR, grippers=None) -> None:
    m.mk.set_state(poses=poses, grippers=grippers or OPEN)
    m.STATE["hands"] = hands


def steps(m, n: int) -> float:
    for _ in range(n):
        m.step()
    return m.score(None)


def built(m, *names) -> dict:
    return {name: m.mk.built_pose(name) for name in names}


# --- the scene and its figures ----------------------------------------------------------------------------------------
def test_the_scene_is_the_one_specified(m):
    assert m.SCORE_OVER == "end" and m.RUN_LIMIT_STEPS == 4600 and m.P.HOLD_STEPS == 40
    assert "3/8-inch socket" in m.INSTRUCTION and "move both hands clear" in m.INSTRUCTION
    bodies, joints = m.mk.MECH.bodies, m.mk.MECH.joints
    assert not joints, "the task joins nothing: barrels ride the shaft by contact alone"
    for name in ("mat", "cradle_start", "cradle_end", "stands", "bracket"):
        assert not bodies[name].free and not bodies[name].kinematic
    assert bodies["driver"].mass == pytest.approx(m.P.driver_props()[0])
    for b in m.P.BARRELS:
        assert bodies[b.key].mass == pytest.approx(b.mass)
    assert m.P.HANDLE_MASS + m.P.BARREL_MASS == pytest.approx(m.P.TOTAL_MASS)  # the maker's 272 g
    put(m, built(m, "driver", WORK, FRONT, REAR))
    assert m.score(None) == 0.0 and m.zero_now() is None
    assert all(m.on_shaft(b.key) for b in m.P.BARRELS) and not m.blue_out()  # blue faces the handle at the start


def test_the_maker_figures_and_the_fits(m):
    p = m.P
    assert (p.OVERALL, p.HANDLE_D, p.TOTAL_MASS, p.SHAFT_AF) == (0.178, 0.0381, 0.272, 0.00635)  # maker
    assert p.EXPOSED == pytest.approx(0.101) and p.EXPOSED + p.SEATED == pytest.approx(p.SHAFT_SET)  # the 127 mm shaft
    assert p.TIP_TO_MID == pytest.approx(0.1395) and p.HANDLE_MID == pytest.approx(0.1455)
    assert (p.POST_AF, p.POST_L, p.POST_OUT) == (0.00953, 0.00953, 0.0075)  # the listed standoff, 7.5 mm proud
    # the blue socket's corners pass the standoff's flats when clocked, and it jams inside 4 degrees of mis-clocking
    assert p.SOCKET_38 > p.POST_AF and p.SOCKET_38 / math.sqrt(3.0) > p.POST_AF / math.sqrt(3.0)
    assert math.degrees(p.CLOCK) == pytest.approx(3.9, abs=0.3)
    assert math.degrees(p.CLOCK) < math.degrees(p.POST_TWIST) - 3.0  # 15 degrees off: a solid jam, not a seat
    for key in ("516", "1132"):  # every smaller bore butts on the standoff at any clocking
        assert p.bore_af(p.SIZES[key]) < p.POST_AF, key
    assert p.BORE_AF == pytest.approx(0.0065) and math.degrees(p.PLAY) == pytest.approx(2.4, abs=0.2)  # indexes on the shaft
    assert p.DEEP - p.POST_OUT >= 0.0055 and 0.030 - p.DEEP - p.SHALLOW == pytest.approx(0.007)  # seats 5 mm, leaves a bore


def test_every_hex_is_three_boxes_whose_union_is_exact(m):
    for af in (m.P.SHAFT_AF, m.P.POST_AF, m.P.SOCKET_38):
        boxes = m.P.hex_prism(af, 0.01, 0.3)
        rng = np.random.default_rng(7)
        for _ in range(1500):
            q = rng.uniform(-0.008, 0.008, 3)
            q[0] = rng.uniform(-0.005, 0.005)
            inside = any(np.all(np.abs(r.T @ (q - at)) <= np.asarray(d) / 2 + 1e-12) for d, at, r in boxes)
            flats = max(abs(-math.sin(0.3 + k * math.pi / 3) * q[1] + math.cos(0.3 + k * math.pi / 3) * q[2]) for k in range(6))
            assert inside == (flats <= af / 2 and abs(q[0]) <= 0.005)


def corners(specs) -> np.ndarray:
    out = []
    for dims, at, r in specs:
        half = np.asarray(dims) / 2
        out += [at + r @ (half * np.array([sx, sy, sz])) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    return np.array(out)


def test_the_lead_ins_enter_at_any_clocking_and_stay_inside_the_hex(m):
    p = m.P
    for narrow, wide, length, bore in ((p.TIP_CH_AF, p.SHAFT_AF, p.TIP_CH, p.BORE_AF), (p.POST_CH_AF, p.POST_AF, p.POST_CH, p.SOCKET_38)):
        pts = corners(p.hex_frustum(narrow, wide, length))
        radial = np.hypot(pts[:, 1], pts[:, 2])
        assert pts[:, 0].min() >= -1e-9 and pts[:, 0].max() <= length + 0.001  # nothing past the end, nothing far behind it
        assert radial.max() <= wide / math.sqrt(3.0) + 1e-9  # inside the full hex's corners
        assert radial[pts[:, 0] <= 0.0002].max() < bore / 2  # the very end fits inside the bore's flats, however it is clocked


def test_the_start_blocks_the_working_barrel_behind_the_front_one(m):
    spans = m.scene.START_SPANS
    assert spans[0][0] == 0.0 and spans[0][1] == pytest.approx(spans[1][0] - 0.004)  # 4 mm ahead of the working barrel
    assert spans[2][1] == pytest.approx(m.P.EXPOSED - 0.001)  # the rear barrel a hair off the grip's nose


# --- the rules ----------------------------------------------------------------------------------------------------------
def test_at_rest_it_scores_nothing_and_the_event_never_fires(m):
    put(m, built(m, "driver", WORK, FRONT, REAR))
    assert steps(m, 5) == 0.0 and m.STATE["zero"] is None and m.STATE["extracted"] is None


def test_the_middle_barrel_off_the_front_is_a_quarter(m):
    put(m, {"driver": driver_pose(m, m.P.TIP_Y),
            WORK: on_shaft_pose(m, WORK, m.P.TIP_Y - 0.019, flip=True),  # clear of the tip, in the hand
            FRONT: on_shaft_pose(m, FRONT, m.P.TIP_Y - 0.080, flip=False),
            REAR: m.mk.built_pose(REAR)})
    assert steps(m, 1) == 0.25 and m.STATE["extracted"] is not None
    assert not m.on_shaft(WORK)


def test_a_barrel_or_the_tool_off_the_bench_loses_the_trial(m):
    put(m, built(m, "driver", WORK, FRONT, REAR))
    p, r = m.mk.built_pose(WORK)
    put(m, {WORK: (np.array([p[0], p[1], m.TOP - 0.031]), r)})
    steps(m, 1)
    assert m.STATE["zero"] == f"{WORK} fell off the bench"
    m.begin(0)
    put(m, {"driver": (np.array([0.0, 0.0, m.TOP - 0.05]), EYE), **built(m, WORK, FRONT, REAR)})
    steps(m, 1)
    assert m.STATE["zero"] == "driver fell off the bench"
    m.begin(0)
    put(m, built(m, "driver", WORK, FRONT, REAR))
    p, r = m.mk.built_pose(WORK)
    put(m, {WORK: (np.array([p[0], p[1] - 0.1, m.TOP + 0.002]), r)})  # knocked onto the mat: benign, not lost
    assert m.zero_now() is None


def restacked(m, tip_y: float | None = None, face_at_tip: float = 0.0, flip: bool = True, front_mid: float | None = None,
              work_tilt: float = 0.0, clock_deg: float = 0.0, tilt_deg: float = 0.0):
    """All three barrels stored on the tool whose tip is at ``tip_y`` (the start's by default), the working one at
    the tip, all rolled ``clock_deg`` and tipped ``tilt_deg`` with the tool."""
    tip = m.P.TIP_Y if tip_y is None else tip_y
    work = m.P.BARRELS[1]
    mid = tip + face_at_tip + work.length / 2
    both = dict(tilt_deg=tilt_deg, clock_deg=clock_deg, tip_y=tip)
    return {"driver": driver_pose(m, tip, clock_deg=clock_deg, tilt_deg=tilt_deg),
            WORK: on_shaft_pose(m, WORK, mid, flip, tilt_deg=work_tilt or tilt_deg, clock_deg=clock_deg, tip_y=tip),
            FRONT: on_shaft_pose(m, FRONT, tip + 0.0405 + 0.01425 if front_mid is None else front_mid, flip=False, tilt_deg=tilt_deg, clock_deg=clock_deg, tip_y=tip),
            REAR: on_shaft_pose(m, REAR, tip + 0.0845, flip=False, tilt_deg=tilt_deg, clock_deg=clock_deg, tip_y=tip)}


def half_state(m, **kw) -> float:
    m.STATE["extracted"] = 0.0  # the quarter was earned earlier in the trial
    put(m, restacked(m, **kw))
    return steps(m, 2)


def test_all_three_stored_with_blue_out_flush_is_half(m):
    assert half_state(m) == 0.5
    assert m.stored_at_tip() and m.face_at_tip() == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("kw, why", [({"face_at_tip": 0.006}, "face 6 mm shy of the tip"),
                                     ({"flip": False}, "blue end inboard"),
                                     ({"work_tilt": 15.0}, "working barrel tilted"),
                                     ({"front_mid": 0.135}, "the front barrel left off the shaft"),
                                     ({"face_at_tip": -0.0175}, "the blue barrel hanging past its through-bore")])
def test_half_needs_every_stored_condition(m, kw, why):
    assert half_state(m, **kw) == 0.25, why


def seated_state(m, seat_mm: float = 6.5, clock_off: float = 0.0, hands=FAR, rest_off: float = 0.0, tilt_deg: float = 0.0, grippers=None):
    """The tool carried to the bracket, its shaft's tip against the standoff's near face and the blue barrel standing
    ``seat_mm`` proud of it, so seated that deep; clocked to the standoff's flats plus ``clock_off``, the whole tool
    ``rest_off`` short of the cradle."""
    tip = m.P.POST_FACE_Y + rest_off
    clock = m.P.POST_TWIST + clock_off
    poses = restacked(m, tip_y=tip, face_at_tip=-seat_mm / 1000.0, clock_deg=clock, tilt_deg=tilt_deg)
    put(m, poses, hands=hands, grippers=grippers)
    return poses


def test_seated_clocked_and_let_go_is_full(m):
    m.STATE["extracted"] = 0.0  # the quarter was earned earlier in the trial
    seated_state(m)
    assert m.seated() and m.in_rest() and m.level_deg() < 1.0 and m.hands_clear() > m.P.HANDS_CLEAR
    assert m.driver_pose()[0][1] >= m.P.POST_FACE_Y - 1e-9 and m.face_at_tip() <= -m.P.SEAT_MIN  # a state the solids allow: the shaft's tip stops at the standoff's face
    assert steps(m, m.P.HOLD_STEPS - 1) == 0.5
    assert steps(m, 1) == 1.0 and m.success(None)


def test_mis_clocked_or_shallow_keeps_it_at_half(m):
    m.STATE["extracted"] = 0.0  # the quarter was earned earlier in the trial
    seated_state(m, clock_off=15.0)  # jammed on the standoff's corners: never 5 mm deep
    assert not m.seated() and steps(m, m.P.HOLD_STEPS + 5) == 0.5
    m.begin(0), m.STATE.update(extracted=0.0)
    seated_state(m, seat_mm=3.0)  # the mouth only 3 mm past the standoff's near face
    assert not m.seated() and steps(m, m.P.HOLD_STEPS + 5) == 0.5
    m.begin(0), m.STATE.update(extracted=0.0)
    seated_state(m, clock_off=6.0)  # just past the fit's own window
    assert not m.seated()


def test_full_needs_the_rest_the_level_and_the_hands(m):
    m.STATE["extracted"] = 0.0  # the quarter was earned earlier in the trial
    near = {"left": FAR["left"], "right": np.array([[m.P.DRIVER_X, m.P.POST_Y, m.TOP + 0.040 + 0.020]])}  # 20 mm off the socket
    seated_state(m, hands=near)
    assert m.seated() and steps(m, m.P.HOLD_STEPS + 5) == 0.5  # a hand near the socket keeps it at half
    m.begin(0), m.STATE.update(extracted=0.0)
    seated_state(m, rest_off=0.040)  # the whole tool 40 mm short of the cradle: carried, not resting
    assert not m.in_rest() and not m.seated() and steps(m, m.P.HOLD_STEPS + 5) == 0.5
    m.begin(0), m.STATE.update(extracted=0.0)
    seated_state(m, tilt_deg=10.0)  # nose down past 8 degrees
    assert m.level_deg() > m.P.LEVEL_DEG and steps(m, m.P.HOLD_STEPS + 5) == 0.5
    m.begin(0), m.STATE.update(extracted=0.0)
    seated_state(m, grippers={"left": ((0, 0.3, 1.1), 0.2), "right": ((0, -0.3, 1.1), 0.2)})  # fingers still shut
    assert not m.grippers_open() and steps(m, m.P.HOLD_STEPS + 5) == 0.5


def test_the_hold_starts_again_if_the_tool_creeps(m):
    m.STATE["extracted"] = 0.0  # the quarter was earned earlier in the trial
    poses = seated_state(m)
    assert steps(m, m.P.HOLD_STEPS - 5) == 0.5
    poses = {name: (p + np.array([0.0, -0.004, 0.0]), r) for name, (p, r) in poses.items()}  # the tool slid 4 mm deeper: past the 3 mm the hold allows
    put(m, poses)
    assert steps(m, 5) == 0.5 and m.STATE["hold"] == 4
    assert steps(m, m.P.HOLD_STEPS - 4) == 1.0


# --- the folder ---------------------------------------------------------------------------------------------------------
def test_the_folder_keeps_the_checklists_limits():
    readme, task = (HERE / "README.md").read_text(), (HERE / "task.py").read_text()
    assert len(readme) <= 10_000 and len(task) <= 15_000
    low = readme.lower()
    assert "earlier review" not in low and "reviewer" not in low and "thank" not in low
    for path in HERE.iterdir():
        assert not path.is_symlink(), path.name
        if path.suffix in (".py", ".md", ".json"):
            text = path.read_text()
            assert not re.search(r"TK-[A-Z0-9]{6}", text) and not re.search(r"#\d{3,}", text), path.name
    for name in ("task.py", "scene.py", "demo.py"):  # no weld, no pushed force, no pose write in the task
        code = (HERE / name).read_text()
        for banned in ("weld(", ".push(", "write_pose", "set_joint", "set_limits", "make("):
            assert banned not in code, (name, banned)
