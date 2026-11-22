"""Pure-Python tests of ``burner_stow``'s figures, serial mechanisms, safety latches, and score levels."""

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
FAR = {"left": np.array([[0.0, 0.35, 1.15]]), "right": np.array([[0.0, -0.35, 1.15]])}
EYE = np.eye(3)

# Flat burner orientation in tin: z_b -> (-1, 0, 0), x_b -> (0, -1, 0), y_b -> (0, 0, +1).
RB_FLAT = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


@pytest.fixture
def m():
    before = {name: sys.modules.get(name) for name in ("mechkit", "parts", "scene")}
    module = task_file.load(TASK).module
    module.mk.mend_all()
    module.begin(0)
    yield module
    module.mk.MECH.clear()
    for name, old in before.items():
        sys.modules.pop(name, None)
        if old is not None:
            sys.modules[name] = old


def shutoff_joints(m) -> dict[str, float]:
    return {
        "lock_slider": m.P.LOCK_STROKE,
        "trigger": m.P.TRIG_LAT,
        "child_stop": m.P.CHILD_STROKE,
    }


def flat_burner_in_tin(m) -> tuple[np.ndarray, np.ndarray]:
    tx, ty = m.P.TIN_XY
    return np.array([tx + 0.124, ty + 0.025, m.TOP + m.P.OVER + m.P.TIN_FLOOR + 0.023]), RB_FLAT


def flat_stand_in_tin(m) -> tuple[np.ndarray, np.ndarray]:
    tx, ty = m.P.TIN_XY
    return np.array([tx - 0.095, ty, m.TOP + m.P.OVER + m.P.TIN_FLOOR + m.P.STAND_FLOOR_H]), EYE


def put(
    m,
    *,
    poses: dict | None = None,
    joints: dict | None = None,
    velocities: dict | None = None,
    hands=FAR,
) -> None:
    m.mk.set_state(poses=poses, joints=joints, velocities=velocities)
    m.STATE["hands"] = hands


def steps(m, n: int) -> float:
    for _ in range(n):
        m.step()
    return m.score(None)


def test_scene_figures_and_serial_joint_topology(m):
    assert m.SCORE_OVER == "end" and m.SUCCESS_HOLD_S == 2.5 and m.RUN_LIMIT_STEPS == 1600
    low_inst = m.INSTRUCTION.lower()
    assert "hobby burner" in low_inst and "round stand" in low_inst and "tin" in low_inst and "lid" in low_inst
    for banned in ("gas", "butane", "flame", "fire", "hazard", "danger", "unsafe", "hot"):
        assert banned not in low_inst

    p = m.P
    assert (p.TOOL_H, p.STAND_D, p.TOOL_D, p.TOOL_TOTAL_MASS) == (0.170, 0.060, 0.110, 0.227)
    assert (p.CARTON_L, p.CARTON_W, p.CARTON_H) == (0.330, 0.177, 0.055)
    assert (p.TIN_L, p.TIN_W, p.TIN_H) == (0.320, 0.172, 0.052)
    assert p.TIN_L < p.CARTON_L and p.TIN_W < p.CARTON_W and p.TIN_H < p.CARTON_H
    assert p.STAND_BODY_MASS + p.CARRIER_MASS + p.BURNER_BODY_MASS + 3 * p.CONTROL_MASS == pytest.approx(0.227)
    assert p.TIN_BASE_MASS + p.TIN_LID_MASS == pytest.approx(0.360)

    joints = m.mk.MECH.joints
    assert sorted(joints) == ["child_stop", "heel_fit", "lid_hinge", "lock_slider", "trigger", "w_stand"]
    # Serial stand attachment: stand --slide(heel_fit)--> heel_carrier --weld(w_stand)--> burner (never direct slide+weld).
    assert (joints["heel_fit"].kind, joints["heel_fit"].child, joints["heel_fit"].parent) == ("slide", "heel_carrier", "stand")
    assert (joints["w_stand"].kind, joints["w_stand"].child, joints["w_stand"].parent) == ("weld", "burner", "heel_carrier")
    for ctrl in ("trigger", "lock_slider", "child_stop"):
        assert (joints[ctrl].kind, joints[ctrl].parent) == ("slide", "burner")
    assert (joints["lid_hinge"].kind, joints["lid_hinge"].child, joints["lid_hinge"].parent) == ("hinge", "tin_lid", "tin_base")


def test_idle_start_state_scores_zero(m):
    put(m)
    assert not m.burner_off()
    assert not m.detached()
    assert m.lid_deg() == pytest.approx(m.P.LID_OPEN_DEG)
    assert steps(m, 25) == 0.0 and m.STATE["zero"] is None


@pytest.mark.parametrize(
    "j_override",
    [
        {"lock_slider": 0.0040, "trigger": 0.0045, "child_stop": 0.0060},  # lock short of 4.5 mm
        {"lock_slider": 0.0060, "trigger": 0.0030, "child_stop": 0.0060},  # trigger still depressed
        {"lock_slider": 0.0060, "trigger": 0.0045, "child_stop": 0.0040},  # red child-stop not reset
    ],
)
def test_shutoff_requires_all_three_coupled_controls(m, j_override):
    put(m, joints=j_override)
    assert not m.burner_off() and steps(m, 5) == 0.0


def test_shutoff_on_bench_scores_quarter(m):
    put(m, joints=shutoff_joints(m))
    assert m.burner_off() and not m.detached()
    assert steps(m, 5) == 0.25 and m.STATE["zero"] is None


def test_entering_tin_while_latched_open_loses_trial_for_good(m):
    # Lay burner inside tin_base while lock_slider is still at 0.0 (valve open).
    m.mk.cut("w_stand")
    put(m, poses={"burner": flat_burner_in_tin(m), "stand": flat_stand_in_tin(m)}, joints={})
    assert steps(m, 1) == 0.0
    assert m.STATE["zero"] == "burner entered the tin while its gas valve was still latched open"
    # Even if subsequently shut off and closed, score remains 0.0 permanently.
    put(
        m,
        poses={"burner": flat_burner_in_tin(m), "stand": flat_stand_in_tin(m)},
        joints={**shutoff_joints(m), "lid_hinge": m.P.LID_OPEN_RAD},
    )
    assert steps(m, m.P.HOLD_STEPS + 5) == 0.0 and not m.success(None)


def test_closing_lid_below_30_deg_over_open_burner_loses_trial(m):
    # Burner hovering just above the 41 mm rim (low_z > 41 mm) while lid closes to 25 deg (< 30 deg) with valve open.
    tx, ty = m.P.TIN_XY
    high_over = (np.array([tx + 0.12, ty + 0.02, m.TOP + 0.075]), RB_FLAT)
    put(m, poses={"burner": high_over}, joints={"lid_hinge": math.radians(m.P.LID_OPEN_DEG - 25.0)})
    assert steps(m, 1) == 0.0
    assert m.STATE["zero"] == "tin lid closed below 30 degrees over the latched-open burner"


def test_bench_drop_zero_rule_and_nudged_tin_allowed(m):
    put(m, poses={"stand": (np.array([0.30, 0.14, m.TOP - 0.020]), EYE)}, joints=shutoff_joints(m))
    assert steps(m, 1) == 0.0 and m.STATE["zero"] == "stand fell off the bench"

    m.begin(0)
    tx, ty = m.P.TIN_XY
    put(m, poses={"tin_base": (np.array([tx, -0.70, m.TOP + m.P.OVER]), EYE)}, joints=shutoff_joints(m))
    assert steps(m, 1) == 0.0 and m.STATE["zero"] == "tin_base fell off the bench"

    # Nudging tin_base 60 mm horizontally on the bench is allowed (not damage or a fall) and still reaches 1.0.
    m.mk.mend_all()
    m.begin(0)
    m.mk.cut("w_stand")
    nudge = np.array([0.060, 0.0, 0.0])
    b_pos, b_rot = flat_burner_in_tin(m)
    s_pos, s_rot = flat_stand_in_tin(m)
    poses = {
        "tin_base": (np.array([tx, ty, m.TOP + m.P.OVER]) + nudge, EYE),
        "burner": (b_pos + nudge, b_rot),
        "stand": (s_pos + nudge, s_rot),
    }
    put(m, poses=poses, joints={**shutoff_joints(m), "lid_hinge": m.P.LID_OPEN_RAD})
    assert steps(m, m.P.HOLD_STEPS) == 1.0 and m.STATE["zero"] is None


def test_pulling_heel_12mm_cuts_weld_and_stowing_with_attached_stand_cannot_pass_quarter(m):
    # At 11.5 mm axial pull (< 12.0 mm), w_stand stays intact.
    put(m, joints={**shutoff_joints(m), "heel_fit": 0.0115})
    steps(m, 1)
    assert not m.mk.is_cut("w_stand")

    # At 12.1 mm axial pull (>= 12.0 mm), w_stand cuts cleanly.
    put(m, joints={**shutoff_joints(m), "heel_fit": 0.0121})
    steps(m, 1)
    assert m.mk.is_cut("w_stand")

    # Conversely, if w_stand was NOT cut and burner+stand lie sideways in tin_base, the 60 mm stand reaches ~63 mm > 46 mm.
    m.mk.mend_all()
    m.begin(0)
    b_pos, b_rot = flat_burner_in_tin(m)
    put(m, poses={"burner": (b_pos, b_rot), "stand": (b_pos, b_rot)}, joints=shutoff_joints(m))
    _, s_high = m.box_in_tin("stand")
    assert s_high[2] > m.P.FLAT_TOP_MAX
    assert not m.inside_tin_flat("stand") and not m.detached()
    assert steps(m, m.P.HOLD_STEPS + 5) == 0.25


def test_detached_with_part_in_open_tin_scores_half(m):
    m.mk.cut("w_stand")
    # Stand flat in tin, burner still outside on bench -> 0.5.
    put(m, poses={"stand": flat_stand_in_tin(m)}, joints=shutoff_joints(m))
    assert m.detached() and m.inside_tin_flat("stand") and not m.inside_tin_flat("burner")
    assert steps(m, m.P.HOLD_STEPS + 5) == 0.5

    # Both flat in tin, lid open (108 deg) -> still 0.5.
    put(m, poses={"burner": flat_burner_in_tin(m), "stand": flat_stand_in_tin(m)}, joints=shutoff_joints(m))
    assert m.inside_tin_flat("burner") and m.inside_tin_flat("stand")
    assert steps(m, m.P.HOLD_STEPS + 5) == 0.5


def test_both_flat_closed_lid_hands_clear_and_1s_hold_scores_full(m):
    m.mk.cut("w_stand")
    closed = {**shutoff_joints(m), "lid_hinge": m.P.LID_OPEN_RAD}
    poses = {"burner": flat_burner_in_tin(m), "stand": flat_stand_in_tin(m)}
    put(m, poses=poses, joints=closed)
    assert m.lid_deg() == pytest.approx(0.0) and m.seam_gap_mm() == pytest.approx(0.0)
    assert steps(m, m.P.HOLD_STEPS - 1) == 0.5
    assert steps(m, 1) == 1.0 and m.success(None)

    # Hand within 20 mm (< 30 mm) drops back to 0.5 and resets hold.
    near_lid = np.array([[m.P.TIN_XY[0], m.P.TIN_XY[1], m.TOP + m.P.TIN_H + 0.018]])
    put(m, poses=poses, joints=closed, hands={"left": FAR["left"], "right": near_lid})
    assert steps(m, 3) == 0.5 and m.STATE["hold"] == 0

    # Moving body (> 10 mm/s) also prevents 1.0.
    put(m, poses=poses, joints=closed, velocities={"burner": (np.array([0.02, 0.0, 0.0]), np.zeros(3))})
    assert steps(m, m.P.HOLD_STEPS + 2) == 0.5


def test_folder_hygiene_and_character_limits():
    task_text = (HERE / "task.py").read_text()
    assert len(task_text) <= 15_000
    readme_path = HERE / "README.md"
    if readme_path.is_file():
        readme = readme_path.read_text()
        assert len(readme) <= 10_000
        low = readme.lower()
        for phrase in ("earlier review", "thank", "assumed until", "measured by hand", "bench-verified"):
            assert phrase not in low
    for path in HERE.iterdir():
        assert not path.is_symlink(), path.name
        if path.suffix in (".py", ".md", ".json"):
            text = path.read_text()
            assert not re.search(r"TK-[A-Z0-9]{6}", text) and not re.search(r"#\d{3,}", text), path.name
            assert ("1212" + ".17") not in text, path.name
            assert ("14-" + "18 N") not in text, path.name
            assert ("TIN_" + "SHIFT_MAX") not in text, path.name
    for name in ("task.py", "scene.py", "demo.py", "checks.py", "parts.py"):
        code = (HERE / name).read_text()
        assert "write_pose(" not in code and "set_joint(" not in code, name
