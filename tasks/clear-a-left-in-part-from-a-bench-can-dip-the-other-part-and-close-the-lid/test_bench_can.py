"""Pure-Python rule, mechanism, and checklist tests for ``mech/tasks/bench_can`` (no simulator needed)."""

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
FAR = {"left": np.array([[0.0, 0.42, 1.15]]), "right": np.array([[0.0, -0.42, 1.15]])}
EYE = np.eye(3)


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


def tray_slot_pose(m, xy: tuple[float, float]):
    return np.array([xy[0], xy[1], m.TOP + m.P.TRAY_FLOOR + 0.5 * m.P.BRG_W + 0.001], dtype=float), EYE


def put(m, *, lid_deg: float = 172.0, dash_m: float = 0.0, brg_screen=None, brg_mat=None,
        can_offset=(0.0, 0.0, 0.0), can_tilt_deg: float = 0.0, tray_offset=(0.0, 0.0, 0.0),
        tray_tilt_deg: float = 0.0, contacts: dict | None = None, velocities: dict | None = None, hands=FAR) -> None:
    c_pos = m.scene.can_start_pos(m.TOP) + np.asarray(can_offset, dtype=float)
    c_rot = m.mk.axis_angle((1.0, 0.0, 0.0), math.radians(can_tilt_deg))
    t_pos = m.scene.tray_start_pos(m.TOP) + np.asarray(tray_offset, dtype=float)
    t_rot = m.mk.axis_angle((1.0, 0.0, 0.0), math.radians(tray_tilt_deg))
    poses = {"can": (c_pos, c_rot), "tray": (t_pos, t_rot)}
    if brg_screen is not None:
        poses["brg_screen"] = brg_screen
    if brg_mat is not None:
        poses["brg_mat"] = brg_mat
    m.mk.set_state(
        poses=poses,
        joints={"lid": math.radians(lid_deg - m.P.LID_OPEN_DEG), "dasher": dash_m},
        contacts=contacts or {},
        velocities=velocities or {},
    )
    m.STATE["hands"] = hands


def steps(m, n: int) -> float:
    for _ in range(n):
        m.step()
    return m.score(None)


def test_scene_masses_joints_and_built_open_lid(m):
    assert m.SCORE_OVER == "end" and m.RUN_LIMIT_STEPS == 2200
    low_inst = m.INSTRUCTION.lower()
    for w in ("bearing", "screen", "tray", "second", "lid"):
        assert w in low_inst, w
    for hazard in ("solvent", "flammable", "fire", "vapour", "vapor", "hazard", "safety", "danger", "unsafe"):
        assert hazard not in low_inst
    bodies, joints = m.mk.MECH.bodies, m.mk.MECH.joints
    assert bodies["can"].mass + bodies["lid"].mass + bodies["dasher"].mass == pytest.approx(m.P.B601_TOTAL_MASS)
    assert bodies["brg_screen"].mass == pytest.approx(m.P.BRG_MASS) == bodies["brg_mat"].mass
    assert sorted(joints) == ["dasher", "lid"]
    assert (joints["lid"].lower, joints["lid"].upper) == pytest.approx((-math.radians(172.0), 0.0))
    assert (joints["dasher"].lower, joints["dasher"].upper) == pytest.approx((0.0, 0.016))
    put(m)
    assert m.lid_deg() == pytest.approx(172.0) and not m.screen_cleared()
    assert steps(m, 5) == 0.0 and m.STATE["zero"] is None


def test_whole_trial_zero_rules_latch_permanently(m):
    b1 = tray_slot_pose(m, m.P.TRAY_SLOT_BACK_XY)
    b2 = tray_slot_pose(m, m.P.TRAY_SLOT_FRONT_XY)
    for kw, msg in [
        ({"can_tilt_deg": 13.0}, "the can tipped more than 12 degrees"),
        ({"can_offset": (0.032, 0.0, 0.0)}, "the can slid more than 30 mm"),
        ({"brg_mat": (np.array([0.1, -0.2, m.TOP - 0.030]), EYE)}, "a bearing fell off the bench"),
        ({"tray_offset": (0.0, -0.028, 0.0)}, "the tray shifted more than 25 mm or tilted > 10 deg"),
        ({"tray_tilt_deg": 11.0}, "the tray shifted more than 25 mm or tilted > 10 deg"),
    ]:
        m.begin(0)
        put(m, **kw)
        assert steps(m, 1) == 0.0 and m.STATE["zero"] == msg
        m.STATE["dipped"] = True
        put(m, lid_deg=0.0, brg_screen=b1, brg_mat=b2)
        assert steps(m, m.P.HOLD_STEPS + 5) == 0.0


def test_open_lid_scores_zero_even_when_both_bearings_dipped_and_trayed(m):
    b1 = tray_slot_pose(m, m.P.TRAY_SLOT_BACK_XY)
    b2 = tray_slot_pose(m, m.P.TRAY_SLOT_FRONT_XY)
    m.STATE["dipped"] = True
    put(m, lid_deg=15.0, brg_screen=b1, brg_mat=b2)
    assert steps(m, m.P.HOLD_STEPS + 5) == 0.0


def test_quarter_half_and_full_progression_and_dip_hold_rule(m):
    b1 = tray_slot_pose(m, m.P.TRAY_SLOT_BACK_XY)
    b2 = tray_slot_pose(m, m.P.TRAY_SLOT_FRONT_XY)

    # 0.25: left-in bearing cleared and lid closed, dip not done
    put(m, lid_deg=0.0, brg_screen=b1)
    assert steps(m, m.P.HOLD_STEPS + 5) == 0.25

    # Dipping with empty finger (no brg_mat contact on dasher) does NOT count
    m.begin(0)
    put(m, lid_deg=172.0, dash_m=0.014, brg_screen=b1, contacts={("dasher", "brg_mat"): 0.0})
    steps(m, 25)
    assert not m.STATE["dipped"]

    # Dipping < 0.8 s (15 steps) resets when released early; >= 16 steps latches dipped = True -> 0.5 when lid closed
    on_dash = (np.array([m.P.CAN_XY[0], m.P.CAN_XY[1], m.TOP + m.P.DASH_REST_Z - 0.014 + 0.5 * m.P.BRG_W]), EYE)
    put(m, lid_deg=172.0, dash_m=0.014, brg_screen=b1, brg_mat=on_dash, contacts={("dasher", "brg_mat"): 18.0})
    steps(m, m.P.DIP_HOLD_STEPS - 1)
    assert not m.STATE["dipped"]
    put(m, lid_deg=172.0, dash_m=0.0, brg_screen=b1)
    steps(m, 1)
    assert m.STATE["dip_steps"] == 0 and not m.STATE["dipped"]

    put(m, lid_deg=172.0, dash_m=0.014, brg_screen=b1, brg_mat=on_dash, contacts={("dasher", "brg_mat"): 18.0})
    steps(m, m.P.DIP_HOLD_STEPS)
    assert m.STATE["dipped"]
    # Leave brg_mat on mat (not in tray), close lid -> 0.5
    put(m, lid_deg=0.0, dash_m=0.0, brg_screen=b1)
    assert steps(m, m.P.HOLD_STEPS + 5) == 0.5

    # Both bearings in tray, dasher up, lid closed, hands clear -> 1.0 after 20 hold steps
    put(m, lid_deg=0.0, dash_m=0.0, brg_screen=b1, brg_mat=b2)
    assert steps(m, m.P.HOLD_STEPS - 1) == 0.5
    assert steps(m, 1) == 1.0 and m.success(None)


def test_stacked_bearings_or_depressed_screen_or_nearby_hand_caps_at_half(m):
    b1 = tray_slot_pose(m, m.P.TRAY_SLOT_BACK_XY)
    b2 = tray_slot_pose(m, m.P.TRAY_SLOT_FRONT_XY)
    stacked = (b1[0] + np.array([0.0, 0.0, m.P.BRG_W]), EYE)
    m.STATE["dipped"] = True
    put(m, lid_deg=0.0, dash_m=0.0, brg_screen=b1, brg_mat=stacked)
    assert not m.both_in_tray() and steps(m, m.P.HOLD_STEPS + 5) == 0.5

    m.begin(0)
    m.STATE["dipped"] = True
    put(m, lid_deg=0.0, dash_m=0.005, brg_screen=b1, brg_mat=b2)
    assert not m.screen_up() and steps(m, m.P.HOLD_STEPS + 5) == 0.5

    m.begin(0)
    m.STATE["dipped"] = True
    near_lid = np.array([[m.P.CAN_XY[0], m.P.CAN_XY[1], m.TOP + m.P.CAN_H + 0.015]])
    put(m, lid_deg=0.0, dash_m=0.0, brg_screen=b1, brg_mat=b2, hands={"left": near_lid, "right": FAR["right"]})
    assert m.hands_clear() < m.P.HANDS_CLEAR_M and steps(m, m.P.HOLD_STEPS + 5) == 0.5


def test_folder_checklists_and_no_pose_snap_rule():
    readme = (HERE / "README.md").read_text()
    task_code = (HERE / "task.py").read_text()
    spec = json.loads((HERE / "task.json").read_text())
    assert len(readme) <= 10_000 and len(task_code) <= 15_000
    assert len(spec["title"]) <= 120 and len(spec["context"]) <= 1_000 and len(spec["fidelity"]) <= 500
    assert "demo: 1.0" in spec["scores"]
    low = (readme + "\n" + json.dumps(spec)).lower()
    for banned in ("earlier review", "reviewer", "thank", "assumed until", "measured by hand", "6205", "4205", "5205", "10175"):
        assert banned not in low, banned
    for path in HERE.iterdir():
        assert not path.is_symlink(), path.name
        if path.suffix in (".py", ".md", ".json") and path.name != "test_bench_can.py":
            text = path.read_text()
            assert not re.search(r"TK-[A-Z0-9]{6}", text) and not re.search(r"#\d{3,}", text), path.name
    for name in ("task.py", "scene.py", "demo.py"):
        code = (HERE / name).read_text()
        for call in ("weld(", "write_pose(", "set_joint(", "set_limits(", "redraw("):
            assert call not in code, f"{name} contains forbidden {call}"
