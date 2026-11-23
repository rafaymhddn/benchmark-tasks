"""The MRI tray task's scene, figures and rules without the simulator: every state is set directly (the items'
poses on the tray, in the bin or in the air; the hands' hull corners far away)."""

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
EYE = np.eye(3)


@pytest.fixture
def m():
    before = {name: sys.modules.get(name) for name in ("mechkit", "lettering", "parts", "scene")}
    module = task_file.load(TASK).module
    module.begin(0)
    yield module
    module.mk.MECH.clear()
    for name, old in before.items():
        sys.modules.pop(name, None)
        if old is not None:
            sys.modules[name] = old


def at(m, name: str, place, yaw: float = 0.0) -> tuple:
    """A pose for body ``name``: ``place`` (x, y, z of its frame's origin) turned ``yaw`` about the vertical."""
    r = m.mk.axis_angle((0.0, 0.0, 1.0), math.radians(yaw))
    return np.array(place, dtype=float), r


def put(m, poses: dict, hands=FAR) -> None:
    m.mk.set_state(poses={k: at(m, k, *v) for k, v in poses.items()})
    m.STATE["hands"] = hands


def placed(m, name: str, place, yaw: float = 0.0) -> dict:
    return {name: (place, yaw)}


def steps(m, n: int) -> float:
    for _ in range(n):
        m.step()
    return m.score(None)


def in_the_bin(m, name: str, x=0.0, y=0.0, z=0.003, yaw: float = 0.0) -> dict:
    """``name`` lying in the bin, its frame's origin ``x``/``y`` off the bin's centre and ``z`` over the bench."""
    return {name: ((m.P.BIN_AT[0] + x, m.P.BIN_AT[1] + y, m.TOP + z), yaw)}


# --- the scene and its figures --------------------------------------------------------------------------------------------
def test_the_scene_is_the_one_specified(m):
    assert m.SCORE_OVER == "end" and m.RUN_LIMIT_STEPS == 1400 and m.SUCCESS_HOLD_S == 2.0
    words = m.INSTRUCTION.lower()
    assert "cart" in words and "mri room" in words
    for word in ("clamp", "padlock", "scissors", "shears", "bin", "bottle", "dish", "clipboard",
                 "danger", "hazard", "unsafe", "magnetic", "steel", "safe"):
        assert word not in words, word
    bodies, joints = m.mk.MECH.bodies, m.mk.MECH.joints
    assert not joints, "no joints anywhere"
    assert sorted(bodies) == sorted((*m.P.BODIES, "label", "sign"))
    assert bodies["label"].kinematic and bodies["sign"].kinematic
    assert sum(not s.solid for s in bodies["bin"].shapes) >= 12, "flat HOLD text on the bin floor"
    assert any(s.dims[0] > 1.2 for s in bodies["sign"].shapes), "full-bench red doorway line"
    assert bodies["tray"].mass == pytest.approx(m.P.TRAY_MASS) and bodies["bin"].mass == pytest.approx(m.P.BIN_MASS)
    for n in (*m.STEEL, *m.SAFE):
        assert bodies[n].mass == pytest.approx(getattr(m.P, {"clamp": "CLAMP_MASS", "padlock": "LOCK_MASS", "shears": "SHEARS_MASS",
                                                             "bottle": "BOTTLE_MASS", "dish": "DISH_MASS", "clipboard": "CLIP_MASS"}[n]))  # fmt: skip
    assert m.P.on_parts_table(), "the six items lie inside the tray, clear of each other"


def test_the_lettered_marks(m):
    import lettering

    for text, cell, length in ((m.P.SIGN_LINE1, m.P.SIGN_CELL1, m.P.SIGN_L), (m.P.SIGN_LINE2, m.P.SIGN_CELL2, m.P.SIGN_L),
                               (m.P.LABEL_LETTER, m.P.LABEL_LETTER_CELL, m.P.LABEL_L),
                               (m.P.LABEL_LINE2, m.P.LABEL_LETTER_CELL, m.P.LABEL_L),
                               (m.P.LABEL_LINE3, m.P.LABEL_LETTER_CELL, m.P.LABEL_L)):
        boxes = lettering.shapes(text, cell, (0.0, 0.0, 0.0))
        wide = max(s.pos[0] + s.dims[0] / 2.0 for s in boxes) - min(s.pos[0] - s.dims[0] / 2.0 for s in boxes)
        assert wide == pytest.approx(lettering.width(text, cell)) and wide <= length - 0.01
        assert all(s.solid is False for s in boxes)
    sign = m.mk.MECH.bodies["sign"].shapes
    assert any(s.dims[0] > 0.3 for s in sign) and len(sign) > 25  # the mat and its letters


def test_the_bin_and_tray_stand_where_the_layout_needs_them(m):
    assert m.P.TRAY_AT == (0.305, 0.0) and m.P.BIN_AT == (0.2715, -0.350)
    gap = (m.P.TRAY_AT[1] - m.P.TRAY_W / 2.0) - (m.P.BIN_AT[1] + m.P.BIN_W / 2.0)
    assert 0.005 <= gap <= 0.030  # the bin sits beside the tray, not against it
    for n in m.STEEL:  # every grasp is the right arm's: no centre-line crossing
        gx, gy, _ = m.P.GRASPS[n]
        assert gx <= 0.34 and gy <= -0.02, f"the {n} grasp must be the right arm's side of the tray"


# --- the rules --------------------------------------------------------------------------------------------------------------
def test_at_rest_it_scores_nothing(m):
    put(m, placed(m, "tray", (0.305, 0.0, m.TOP + 0.001)))
    assert steps(m, 5) == 0.0 and m.STATE["zero"] is None and m.binned() == []


SAFE_AT = {"bottle": "BOTTLE_AT", "dish": "DISH_AT", "clipboard": "CLIP_AT"}


def three_binned(m) -> dict:
    return {**in_the_bin(m, "clamp"), **in_the_bin(m, "padlock"), **in_the_bin(m, "shears", y=-0.0105)}


@pytest.mark.parametrize("name", ["bottle", "dish", "clipboard"])
def test_a_safe_item_lifted_and_moved_away_never_zeroes_and_counts_once_back_on_the_tray(m, name):
    x, y = getattr(m.P, SAFE_AT[name])
    floor = m.TOP + m.P.TRAY_FLOOR + 0.001
    # Held 12 cm over its place, set on the bench past the doorway line (plastic: no latch), set beside the tray:
    for place in ((x, y, floor + 0.120), (0.700, y, m.TOP + 0.001), (x, 0.400, m.TOP + 0.001)):
        put(m, {**three_binned(m), **placed(m, name, place)})
        assert steps(m, m.P.HOLD_STEPS + 2) == 0.0 and m.STATE["zero"] is None and m.safe_off(name) is not None
    # Back on the tray 50 mm from its start: the end state earns the full level.
    put(m, {**three_binned(m), **placed(m, name, (x - 0.050, y, floor))})
    assert m.safe_off(name) is None and m.displaced(name) == pytest.approx(0.050)
    assert steps(m, m.P.HOLD_STEPS - 1) == 0.0 and steps(m, 1) == 1.0 and m.STATE["zero"] is None


@pytest.mark.parametrize("name,deg,level", [("bottle", 15.0, 1.0), ("bottle", 25.0, 0.0), ("bottle", 90.0, 0.0),
                                            ("dish", 180.0, 0.0), ("clipboard", 180.0, 0.0)])
def test_a_safe_item_back_on_the_tray_must_be_upright_within_20_degrees(m, name, deg, level):
    x, y = getattr(m.P, SAFE_AT[name])
    turn = m.mk.axis_angle((1.0, 0.0, 0.0), math.radians(deg))
    steel = {k: at(m, k, *v) for k, v in three_binned(m).items()}
    m.mk.set_state(poses={**steel, name: (np.array([x, y, m.TOP]), turn)})
    rise = 0.001 - m.floor_gap(name)  # its lowest corner 1 mm over the tray floor
    m.mk.set_state(poses={**steel, name: (np.array([x, y, m.TOP + rise]), turn)})
    m.STATE["hands"] = FAR
    assert m.floor_gap(name) == pytest.approx(0.001) and m.tilt(name) == pytest.approx(deg)
    assert (m.safe_off(name) is None) == (level > 0.0)
    assert steps(m, m.P.HOLD_STEPS + 1) == level and m.STATE["zero"] is None


def test_every_positive_level_needs_all_three_safe_items_back_on_the_tray(m):
    half = {**in_the_bin(m, "clamp"), **in_the_bin(m, "padlock"), **placed(m, "shears", (0.300, 0.400, m.TOP + 0.003))}
    bx, by = m.P.BOTTLE_AT
    for steel, level in ((three_binned(m), 1.0), (half, 0.5)):
        put(m, {**steel, **placed(m, "bottle", (bx, 0.300, m.TOP + 0.001))})  # left on the bench beside the tray
        assert steps(m, m.P.HOLD_STEPS + 5) == 0.0 and m.STATE["zero"] is None
        assert m.safe_off("bottle") == "the bottle is off the tray"
        put(m, {**steel, **placed(m, "bottle", (bx, by, m.TOP + m.P.TRAY_FLOOR + 0.001))})
        assert steps(m, m.P.HOLD_STEPS) == level


def test_two_in_the_bin_with_the_third_still_on_the_tray_is_zero(m):
    put(m, {**in_the_bin(m, "clamp"), **in_the_bin(m, "padlock")})
    assert m.binned() == ["clamp", "padlock"] and m.on_tray("shears")
    assert steps(m, m.P.HOLD_STEPS + 3) == 0.0


def test_two_in_the_bin_and_the_third_cleared_on_the_safe_side_is_half(m):
    # Hovering over the tray, overhanging its rim, or held mid-air off the tray does NOT count as cleared:
    for bad in ((m.P.SHEARS_AT[0], m.P.SHEARS_AT[1], m.TOP + 0.120),
                (0.330, -0.210, m.TOP + 0.003),
                (0.300, 0.400, m.TOP + 0.250)):
        put(m, {**in_the_bin(m, "clamp"), **in_the_bin(m, "padlock"), **placed(m, "shears", bad)})
        assert not m.cleared("shears") and steps(m, m.P.HOLD_STEPS + 2) == 0.0
    # Resting on the safe bench with a hand within 30 mm also stays at 0 (same gate as 1.0; no tier inversion):
    near = np.array([[m.P.BIN_AT[0], m.P.BIN_AT[1], m.TOP + 0.160]])
    put(m, {**in_the_bin(m, "clamp"), **in_the_bin(m, "padlock"),
            **placed(m, "shears", (0.300, 0.400, m.TOP + 0.003))}, hands={"left": FAR["left"], "right": near})
    assert m.cleared("shears") and steps(m, m.P.HOLD_STEPS + 2) == 0.0
    # Resting on the safe bench with both hands clear reaches 0.5 after the 20-step rest:
    put(m, {**in_the_bin(m, "clamp"), **in_the_bin(m, "padlock"),
            **placed(m, "shears", (0.300, 0.400, m.TOP + 0.003))})
    assert m.binned() == ["clamp", "padlock"] and m.cleared("shears")
    assert steps(m, m.P.HOLD_STEPS - 1) == 0.0
    assert steps(m, 1) == 0.5


@pytest.mark.parametrize("x,y,dz,why", [
    (0.630, 0.0, 0.002, "the padlock crossed the Zone IV line"),  # a corner past the red line at x = 62.5 cm
    (0.270, -0.660, 0.002, "the padlock went off the bench"),  # past the bench's side edge (y = -65 cm)
    (0.270, -0.350, -0.030, "the padlock went off the bench"),  # 30 mm under the bench top
])
def test_a_steel_item_past_the_door_line_or_off_the_bench_latches_zero(m, x, y, dz, why):
    put(m, {**in_the_bin(m, "clamp"), **in_the_bin(m, "shears", y=-0.0105), **placed(m, "padlock", (x, y, m.TOP + dz))})
    assert steps(m, 1) == 0.0 and m.STATE["zero"] == why
    put(m, {**in_the_bin(m, "clamp"), **in_the_bin(m, "padlock"), **in_the_bin(m, "shears", y=-0.0105)})
    assert steps(m, m.P.HOLD_STEPS + 5) == 0.0 and not m.success(None)


def test_an_item_straddling_the_bin_rim_is_not_in_the_bin(m):
    put(m, in_the_bin(m, "shears", x=0.150))
    assert not m.in_bin("shears") and m.binned() == []


def test_an_item_sticking_above_the_bin_rim_is_not_in_the_bin(m):
    put(m, in_the_bin(m, "clamp", z=0.040))  # 120 mm tall at z=40 mm reaches 160 mm > 144 mm (rim - 5 mm)
    assert not m.in_bin("clamp") and m.binned() == []


def test_all_three_in_the_bin_with_the_hands_far_away_is_full(m):
    put(m, {**in_the_bin(m, "clamp"), **in_the_bin(m, "padlock"), **in_the_bin(m, "shears", y=-0.0105)})
    assert m.hands_clear() > m.P.HANDS_CLEAR
    assert steps(m, m.P.HOLD_STEPS - 1) == 0.0
    assert steps(m, 1) == 1.0 and m.success(None)
    assert m.P.HOLD_STEPS == 20 and m.P.HOLD_STEPS * 0.05 == pytest.approx(1.0)


def test_a_hand_over_the_bin_keeps_it_below_full(m):
    near = np.array([[m.P.BIN_AT[0], m.P.BIN_AT[1], m.TOP + 0.160]])  # a hand 10 mm over the bin's rim
    put(m, {**in_the_bin(m, "clamp"), **in_the_bin(m, "padlock"), **in_the_bin(m, "shears", y=-0.0105)}, hands={"left": FAR["left"], "right": near})
    assert m.hands_clear() < m.P.HANDS_CLEAR
    assert steps(m, m.P.HOLD_STEPS + 5) == 0.0


def test_the_hold_starts_again_if_something_creeps(m):
    put(m, {**in_the_bin(m, "clamp"), **in_the_bin(m, "padlock"), **in_the_bin(m, "shears", y=-0.0105)})
    assert steps(m, m.P.HOLD_STEPS - 5) == 0.0
    put(m, {**in_the_bin(m, "clamp"), **in_the_bin(m, "padlock"), **in_the_bin(m, "shears", y=-0.0105, x=0.004)})
    assert steps(m, 5) == 0.0 and 0 < m.STATE["hold"] < m.P.HOLD_STEPS
    put(m, {**in_the_bin(m, "clamp"), **in_the_bin(m, "padlock"), **in_the_bin(m, "shears", y=-0.0105)})
    assert steps(m, m.P.HOLD_STEPS + 1) == 1.0


def test_a_shifted_bin_still_holds_what_is_in_it(m):
    put(m, {**in_the_bin(m, "clamp"), **in_the_bin(m, "padlock"), **in_the_bin(m, "shears", x=0.030, y=-0.0305),
            **placed(m, "bin", (m.P.BIN_AT[0] + 0.030, m.P.BIN_AT[1] - 0.020, m.TOP + 0.001))})
    assert len(m.binned()) == 3 and steps(m, m.P.HOLD_STEPS + 2) == 1.0


# --- the folder -------------------------------------------------------------------------------------------------------------
def test_the_folder_keeps_the_checklists_limits():
    import json

    readme, task = (HERE / "README.md").read_text(), (HERE / "task.py").read_text()
    assert len(readme) <= 10_000 and len(task) <= 15_000
    spec = json.loads((HERE / "task.json").read_text())
    assert len(spec["title"]) <= 120 and len(spec["context"]) <= 1_000 and len(spec["fidelity"]) <= 500
    assert "pick and place" not in [k.lower() for k in spec["keywords"]]
    low = readme.lower()
    assert "earlier review" not in low and "reviewer" not in low and "thank" not in low
    for path in HERE.iterdir():
        if path.suffix in (".py", ".md", ".json"):
            text = path.read_text()
            assert not re.search(r"TK-[A-Z0-9]{6}", text) and not re.search(r"#\d{3,}", text), path.name
    for name in ("task.py", "scene.py"):  # no weld, no pushed force, no latch on nearness in the task
        code = (HERE / name).read_text()
        assert "weld(" not in code and "push(" not in code and ".push" not in code
