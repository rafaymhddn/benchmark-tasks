"""Pure CPU tests of the end-cap suspension packing task's figures, interference geometry, scoring rules, and folder
limits (no simulator needed)."""

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


def put(m, *, roller=None, cap_l=None, cap_r=None, contacts: dict | None = None, hands=FAR) -> None:
    poses = {
        "roller": roller or m.mk.built_pose("roller"),
        "cap_l": cap_l or m.mk.built_pose("cap_l"),
        "cap_r": cap_r or m.mk.built_pose("cap_r"),
    }
    m.mk.set_state(poses=poses, contacts=contacts or {})
    m.STATE["hands"] = hands


def steps(m, n: int = 1) -> float:
    for _ in range(n):
        m.step()
    return m.score(None)


def extract_both_caps(m) -> None:
    """Place both caps on the bench outside the carton long enough to satisfy OUT_HOLD_STEPS."""
    p = m.P
    put(
        m,
        cap_l=(np.array([p.X_ROLLER, +0.144, m.TOP]), EYE),
        cap_r=(np.array([p.X_ROLLER, -0.144, m.TOP]), EYE),
    )
    steps(m, p.OUT_HOLD_STEPS)
    assert m.STATE["both_caps_out"] is True


def coupled_poses(m, x: float, z_cap_bottom: float, spread: float = 0.0, tilt_deg: float = 0.0):
    """(roller, cap_l, cap_r) poses with both caps mated onto the roller at ``x``, cap bottoms at ``z_cap_bottom``,
    and each cap pulled outward by ``spread`` m from full shoulder contact."""
    p = m.P
    rot_r = m.mk.axis_angle((1.0, 0.0, 0.0), math.radians(tilt_deg))
    z_roller = z_cap_bottom + (p.SOCK_Z - p.SOCK_R) + p.ROLLER_R  # resting on socket bottom wall (z_local = 16 mm + 20 mm = 36 mm)
    y_cap = p.COUPLED_CAP_Y + spread
    return (
        (np.array([x, 0.0, z_roller]), rot_r),
        (np.array([x, +y_cap, z_cap_bottom]), EYE),
        (np.array([x, -y_cap, z_cap_bottom]), EYE),
    )


# --- 1. Scene, commercial figures, and interference math -------------------------------------------------------------
def test_scene_and_commercial_figures(m):
    p = m.P
    assert m.SCORE_OVER == "end" and m.SUCCESS_HOLD_S == 3.0 and m.RUN_LIMIT_STEPS == 1200
    assert "blue foam end-caps" in m.INSTRUCTION and "steel roller" in m.INSTRUCTION and "suspended" in m.INSTRUCTION
    bodies = m.mk.MECH.bodies
    assert set(bodies) == {"carton", "cradle", "roller", "cap_l", "cap_r"}
    assert bodies["carton"].kinematic and bodies["cradle"].kinematic
    assert len(m.mk.MECH.joints) == 0  # pure rigid-body contacts: zero joints or welds

    # ULINE S-4600 (10 x 6 x 3 in) and Interroll RS 761-3149 (40 x 200 mm, 8 x 226 mm spindle)
    assert (p.BOX_L, p.BOX_W, p.BOX_H) == pytest.approx((0.2540, 0.1524, 0.0762))
    assert (p.ROLLER_D, p.ROLLER_L, p.SPINDLE_D, p.SPINDLE_L) == pytest.approx((0.0400, 0.2000, 0.0080, 0.2260))

    # Double-ended geometric lockout & coupled fit
    assert p.BOX_L - 2.0 * p.CAP_T == pytest.approx(0.1620)  # 162 mm < 200 mm roller (38 mm interference)
    one_cap_gap = p.BOX_L - (p.CAP_T - p.SOCK_DEPTH + p.ROLLER_L)
    assert one_cap_gap == pytest.approx(0.0290) and one_cap_gap < p.CAP_T  # 29 mm < 46 mm cap (17 mm interference)
    assert one_cap_gap - p.SPINDLE_EXT == pytest.approx(0.0160)  # 16 mm clear past protruding spindle
    assert p.COUPLED_SPAN == pytest.approx(0.2500) and p.BOX_L - p.COUPLED_SPAN == pytest.approx(0.0040)
    # Even at full 4 mm spread inside carton, each end stays >= 17 mm engaged (>= 14 mm full-credit threshold)
    min_engaged = p.SOCK_DEPTH - (p.BOX_L - p.COUPLED_SPAN)
    assert min_engaged == pytest.approx(0.0170) and min_engaged >= p.ENGAGE_FULL
    # Cradle 3-4-5 geometry keeps roller axis dead coaxial with cap socket height SOCK_Z = 38.5 mm
    assert p.CRADLE_RAIL_TOP + math.sqrt(p.ROLLER_R**2 - p.CRADLE_RAIL_INNER_X**2) == pytest.approx(p.SOCK_Z)
    # Seated roller hangs 16.0 mm clear above carton floor (>= 10.0 mm threshold)
    seated_gap = p.SOCK_Z - p.SOCK_R
    assert seated_gap == pytest.approx(0.0160) and seated_gap >= p.SUSPEND_CLEAR_MIN


def test_pinch_across_x_leaves_wide_carton_wall_clearance(m):
    p = m.P
    assert p.CAP_TOP_W == pytest.approx(0.0720) and p.CAP_TOP_W < 0.094  # inside 94 mm jaw
    assert (p.BOX_W - p.CAP_TOP_W) / 2.0 == pytest.approx(0.0402)  # 40.2 mm finger room to front & back walls


# --- 2. Scoring levels and shortcuts ---------------------------------------------------------------------------------
def test_idle_scores_zero(m):
    put(m)
    assert steps(m, 10) == 0.0
    assert m.STATE["zero"] is None and m.STATE["both_caps_out"] is False


def test_falling_off_bench_locks_zero_for_good(m):
    p = m.P
    extract_both_caps(m)
    put(m, cap_r=(np.array([p.X_ROLLER, -0.25, m.TOP - 0.015]), EYE))
    assert steps(m, 1) == 0.0 and m.STATE["zero"] == "cap_r fell off the bench"
    # Even placing everything in the ideal 1.0 suspended state afterwards stays 0.0
    r_pose, cl_pose, cr_pose = coupled_poses(m, p.X_CARTON, m.TOP + p.Z_FLOOR)
    put(m, roller=r_pose, cap_l=cl_pose, cap_r=cr_pose)
    assert steps(m, p.HOLD_STEPS + 5) == 0.0 and not m.success(None)


def test_shortcuts_without_extracting_both_caps_score_zero(m):
    p = m.P
    # Shortcut A: dropping roller straight onto the two nested caps inside carton
    put(m, roller=(np.array([p.X_CARTON, 0.0, m.TOP + p.Z_FLOOR + p.CAP_H + p.ROLLER_R]), EYE))
    assert steps(m, p.HOLD_STEPS + 5) == 0.0
    # Shortcut B: taking out only cap_r while cap_l stays inside carton
    m.begin(0)
    put(
        m,
        cap_l=(np.array([p.X_CARTON, +0.104, m.TOP + p.Z_FLOOR]), EYE),
        cap_r=(np.array([p.X_ROLLER, -0.144, m.TOP]), EYE),
    )
    assert steps(m, p.OUT_HOLD_STEPS + 5) == 0.0 and m.STATE["both_caps_out"] is False


def test_extracting_both_caps_scores_quarter_and_mating_on_bench_scores_half(m):
    p = m.P
    extract_both_caps(m)
    assert m.score(None) == 0.25

    # Mate both caps onto opposite ends of roller on the bench -> 0.5
    r_pose, cl_pose, cr_pose = coupled_poses(m, p.X_ROLLER, m.TOP)
    put(m, roller=r_pose, cap_l=cl_pose, cap_r=cr_pose)
    assert steps(m, 3) == 0.5

    # If one slip-on cap drops off during a one-handed lift, score returns to 0.25
    put(
        m,
        roller=(np.array([p.X_ROLLER, 0.0, m.TOP + 0.080]), m.mk.axis_angle((1.0, 0.0, 0.0), math.radians(18.0))),
        cap_l=(np.array([p.X_ROLLER, +0.102, m.TOP + 0.085]), EYE),
        cap_r=(np.array([p.X_ROLLER, -0.145, m.TOP]), EYE),
    )
    assert steps(m, 3) == 0.25


def test_symmetric_roller_flip_also_mates_cleanly(m):
    p = m.P
    extract_both_caps(m)
    r_pose, cl_pose, cr_pose = coupled_poses(m, p.X_ROLLER, m.TOP)
    flipped_roller = (r_pose[0], m.mk.axis_angle((0.0, 0.0, 1.0), math.pi))
    put(m, roller=flipped_roller, cap_l=cl_pose, cap_r=cr_pose)
    assert steps(m, 2) == 0.5


def test_bare_roller_on_carton_floor_or_touching_carton_cannot_score_full(m):
    p = m.P
    extract_both_caps(m)
    # Bare roller resting directly on carton floor (gap = 0 mm < 10 mm) with caps beside it -> 0.25
    put(
        m,
        roller=(np.array([p.X_CARTON, 0.0, m.TOP + p.Z_FLOOR + p.ROLLER_R]), EYE),
        cap_l=(np.array([p.X_ROLLER, +0.14, m.TOP]), EYE),
        cap_r=(np.array([p.X_ROLLER, -0.14, m.TOP]), EYE),
        contacts={("roller", "carton"): 3.92},
    )
    assert steps(m, p.HOLD_STEPS + 5) == 0.25

    # Even if coupled geometry were met, nonzero roller-to-carton contact blocks 1.0
    r_pose, cl_pose, cr_pose = coupled_poses(m, p.X_CARTON, m.TOP + p.Z_FLOOR)
    put(m, roller=r_pose, cap_l=cl_pose, cap_r=cr_pose, contacts={("roller", "carton"): 0.5})
    assert steps(m, p.HOLD_STEPS + 5) == 0.5


def test_suspended_unit_in_carton_with_hands_clear_for_1s_scores_full(m):
    p = m.P
    extract_both_caps(m)
    r_pose, cl_pose, cr_pose = coupled_poses(m, p.X_CARTON, m.TOP + p.Z_FLOOR, spread=0.001)
    put(m, roller=r_pose, cap_l=cl_pose, cap_r=cr_pose, contacts={("cap_l", "carton"): 2.23, ("cap_r", "carton"): 2.23})
    assert m.roller_floor_gap() == pytest.approx(0.0160, abs=1e-4)
    assert steps(m, p.HOLD_STEPS - 1) == 0.5
    assert steps(m, 1) == 1.0 and m.success(None)


def test_hand_within_30mm_or_drift_past_2mm_prevents_full(m):
    p = m.P
    extract_both_caps(m)
    r_pose, cl_pose, cr_pose = coupled_poses(m, p.X_CARTON, m.TOP + p.Z_FLOOR)
    near_hand = {"left": np.array([[p.X_CARTON, +0.102, m.TOP + p.Z_RIM + 0.020]]), "right": FAR["right"]}
    put(m, roller=r_pose, cap_l=cl_pose, cap_r=cr_pose, hands=near_hand)
    assert m.hands_clear() == pytest.approx(0.020, abs=1e-6)
    assert steps(m, p.HOLD_STEPS + 5) == 0.5

    # Move hands clear, build up 15 hold steps, then drift roller 3 mm (> 2 mm) -> hold resets
    put(m, roller=r_pose, cap_l=cl_pose, cap_r=cr_pose, hands=FAR)
    assert steps(m, p.HOLD_STEPS - 5) == 0.5
    shifted_r = (r_pose[0] + np.array([0.003, 0.0, 0.0]), r_pose[1])
    put(m, roller=shifted_r, cap_l=cl_pose, cap_r=cr_pose, hands=FAR)
    assert steps(m, 5) == 0.5 and m.STATE["hold"] == 5
    assert steps(m, p.HOLD_STEPS - 5) == 1.0


def test_folder_respects_checklist_limits():
    task = (HERE / "task.py").read_text()
    assert len(task) <= 15_000
    for name in ("task.py", "scene.py"):
        code = (HERE / name).read_text()
        assert "weld(" not in code and "push(" not in code and ".push(" not in code
    if (HERE / "README.md").is_file() and (HERE / "task.json").is_file():
        readme = (HERE / "README.md").read_text()
        assert len(readme) <= 10_000
        low = readme.lower()
        assert "earlier review" not in low and "reviewer" not in low and "thank" not in low
    for path in HERE.iterdir():
        assert not path.is_symlink(), path.name
        if path.suffix in (".py", ".md", ".json"):
            text = path.read_text()
            assert not re.search(r"TK-[A-Z0-9]{6}", text) and not re.search(r"#\d{3,}", text), path.name
