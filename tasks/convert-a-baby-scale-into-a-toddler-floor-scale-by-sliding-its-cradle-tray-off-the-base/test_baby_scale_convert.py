"""Pure CPU unit tests for ``baby_scale_convert``: scene figures, rules, tier boundaries, and checklist limits."""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

import numpy as np
import pytest
from yam_arena import task_file

HERE = Path(__file__).resolve().parent
TASK = HERE / "task.py"
FAR = {"left": np.array([[0.0, 0.35, 1.30]]), "right": np.array([[0.0, -0.35, 1.30]])}
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


def put(m, *, base_pos=None, base_rot=EYE, btn: float = 0.0, tray_pos=None, tray_rot=EYE,
        mat_pos=None, mat_rot=EYE, tray_vel=(0.0, 0.0, 0.0), contacts=None, hands=FAR) -> None:
    bp = np.array([m.P.BASE_X, m.P.BASE_Y, m.TOP], dtype=float) if base_pos is None else np.asarray(base_pos, float)
    tp = np.array([m.P.TRAY_X, m.P.TRAY_Y, m.TOP + m.P.BASE_H], dtype=float) if tray_pos is None else np.asarray(tray_pos, float)
    mp = np.array([m.P.MAT_X, m.P.MAT_Y, m.TOP], dtype=float) if mat_pos is None else np.asarray(mat_pos, float)
    cdict = {("weighing_tray", "scale_base"): 7.8, ("weighing_tray", "release_btn"): 0.0, ("weighing_tray", "mat"): 0.0}
    if contacts:
        cdict.update(contacts)
    m.mk.set_state(
        joints={"btn_slide": btn},
        poses={"scale_base": (bp, base_rot), "weighing_tray": (tp, tray_rot), "mat": (mp, mat_rot)},
        velocities={
            "scale_base": (np.zeros(3), np.zeros(3)),
            "weighing_tray": (np.asarray(tray_vel, float), np.zeros(3)),
            "mat": (np.zeros(3), np.zeros(3)),
        },
        contacts=cdict,
    )
    m.STATE["hands"] = hands


def on_mat(m) -> list[float]:
    """Tray origin with its support ribs resting on the mat floor (the mat body sits 0.3 mm over the bench)."""
    return [m.P.MAT_X, m.P.MAT_Y, m.TOP + 0.0003 + m.P.MAT_FLOOR_T - m.scene.RIB_Z0]


def steps(m, n: int) -> float:
    for _ in range(n):
        m.step()
    return m.score(None)


def test_scene_figures_and_true_assembled_depth(m):
    p = m.P
    assert m.SCORE_OVER == "end" and m.RUN_LIMIT_STEPS == 1200
    assert (p.ASSEMBLED_W, p.ASSEMBLED_H, p.ASSEMBLED_D) == pytest.approx((0.552, 0.156, 0.332))
    assert (p.TRAY_W, p.TRAY_H, p.TRAY_D) == pytest.approx((0.525, 0.080, 0.250))
    assert (p.BASE_W, p.BASE_H, p.BASE_D) == pytest.approx((0.280, 0.065, 0.235))
    assert (p.MAT_W, p.MAT_D, p.MAT_H, p.MAT_MASS) == pytest.approx((0.370, 0.290, 0.020, 0.300))
    # True 332 mm assembled depth: base protrudes 82 mm forward of tray, exposing release button
    assert (p.TRAY_X + p.TRAY_D / 2) - p.BASE_FRONT_X == pytest.approx(0.332)
    assert p.TRAY_FRONT_X - p.BASE_FRONT_X == pytest.approx(0.082)
    assert p.BASE_FRONT_X + 0.030 < p.BTN_X < p.TRAY_FRONT_X - 0.020
    assert p.BASE_MASS + p.BTN_MASS + p.TRAY_MASS == pytest.approx(2.30)
    assert sorted(m.mk.MECH.joints) == ["btn_slide"] and m.mk.MECH.joints["btn_slide"].stiffness == pytest.approx(2400.0)


def test_idle_locked_start_scores_zero(m):
    put(m)
    assert steps(m, 25) == 0.0
    assert m.STATE["zero"] is None and not m.tray_separated()


def test_lifting_or_shifting_scale_base_latches_zero(m):
    put(m, base_pos=[m.P.BASE_X, m.P.BASE_Y, m.TOP + 0.009])  # 9 mm lift > 8 mm limit
    assert steps(m, 1) == 0.0 and m.STATE["zero"] == "the scale base was lifted off its marks"
    # Even if placed on mat afterwards, latched zero keeps score 0.0
    put(m, tray_pos=on_mat(m),
        contacts={("weighing_tray", "scale_base"): 0.0, ("weighing_tray", "mat"): 7.8})
    assert steps(m, 25) == 0.0
    m.begin(0)
    put(m, base_pos=[m.P.BASE_X + 0.016, m.P.BASE_Y, m.TOP])  # 16 mm shift > 15 mm limit
    assert steps(m, 1) == 0.0 and m.STATE["zero"] == "the scale base was shifted off its marks"


def test_bench_fall_or_upside_down_latches_zero_but_gentle_touch_does_not(m):
    # Gentle low-speed touch near bench (0.10 m/s < 0.50 m/s) is NOT zeroed (REVIEW-LESSONS Rule 7)
    put(m, tray_pos=[0.48, 0.20, m.TOP + 0.020], tray_vel=(0.0, 0.0, -0.10),
        contacts={("weighing_tray", "scale_base"): 0.0})
    steps(m, 2)
    assert m.STATE["zero"] is None
    # Hard fall onto bench (> 0.50 m/s downward at bench height) latches zero
    put(m, tray_pos=[0.48, 0.20, m.TOP + 0.020], tray_vel=(0.0, 0.0, -0.65),
        contacts={("weighing_tray", "scale_base"): 0.0})
    assert steps(m, 1) == 0.0 and m.STATE["zero"] == "the weighing tray fell onto or off the bench"
    # Upside-down flip (> 90 deg) latches zero
    m.begin(0)
    flipped = np.diag([1.0, -1.0, -1.0])
    put(m, tray_pos=[m.P.MAT_X, m.P.MAT_Y, m.TOP + 0.09], tray_rot=flipped,
        contacts={("weighing_tray", "scale_base"): 0.0})
    assert steps(m, 1) == 0.0 and m.STATE["zero"] == "the weighing tray fell onto or off the bench"


def test_unlocked_and_separated_is_half(m):
    # Slid 22 mm back (>= 20 mm), not yet on mat -> 0.5
    put(m, tray_pos=[m.P.TRAY_X + 0.022, m.P.TRAY_Y, m.TOP + m.P.BASE_H])
    assert m.tray_separated() and not m.tray_on_mat_upright()
    assert steps(m, 25) == 0.5
    # Moved sideways (+y) clear of base onto bare bench gently -> 0.5
    m.begin(0)
    put(m, tray_pos=[m.P.TRAY_X, 0.45, m.TOP + 0.020], contacts={("weighing_tray", "scale_base"): 0.0})
    assert m.tray_separated() and steps(m, 25) == 0.5


def test_upright_on_mat_clear_and_still_for_one_second_is_full(m):
    put(m, tray_pos=on_mat(m),
        contacts={("weighing_tray", "scale_base"): 0.0, ("weighing_tray", "mat"): 7.8})
    assert m.tray_on_mat_upright() and m.hands_clear() > m.P.HANDS_CLEAR
    assert steps(m, m.P.HOLD_STEPS - 1) == 0.5
    assert steps(m, 1) == 1.0 and m.success(None)


def test_hand_near_or_button_stuck_or_moving_stays_at_half(m):
    near = {"left": np.array([[m.P.MAT_X, m.P.MAT_Y, m.TOP + 0.050]]), "right": FAR["right"]}
    put(m, tray_pos=on_mat(m),
        contacts={("weighing_tray", "scale_base"): 0.0, ("weighing_tray", "mat"): 7.8}, hands=near)
    assert steps(m, 25) == 0.5
    # Button still depressed > 1.0 mm -> stays 0.5
    m.begin(0)
    put(m, btn=0.002, tray_pos=on_mat(m),
        contacts={("weighing_tray", "scale_base"): 0.0, ("weighing_tray", "mat"): 7.8})
    assert steps(m, 25) == 0.5
    # Moving tray -> stays 0.5
    m.begin(0)
    put(m, tray_pos=on_mat(m), tray_vel=(0.03, 0.0, 0.0),
        contacts={("weighing_tray", "scale_base"): 0.0, ("weighing_tray", "mat"): 7.8})
    assert steps(m, 25) == 0.5


def test_folder_and_checklist_limits(m):
    readme = (HERE / "README.md").read_text()
    task = (HERE / "task.py").read_text()
    spec = json.loads((HERE / "task.json").read_text())
    assert len(readme) <= 10_000 and len(task) <= 15_000
    assert len(spec["title"]) <= 120 and len(spec["context"]) <= 1_000 and len(spec["fidelity"]) <= 500
    assert spec["instruction"] == m.INSTRUCTION and spec["category"] == "Impact on jobs"
    assert m.INSTRUCTION.endswith("then let go and move both hands clear.")  # the score needs hands >= 30 mm clear
    for text in (readme, spec["scoring"], spec["setup"], spec["reset_notes"], m.score.__doc__):
        for stale in ("inner rim", "27.4 x 35.4", "contact >= 1.0 N", "overhead and side", "gs2b", "gs3b"):
            assert stale not in text, stale
    assert "31-9092.00" in spec["context"] and "29-1141.00" in spec["context"] and "29-2061.00" in spec["context"]
    for path in HERE.iterdir():
        assert not path.is_symlink(), path.name
        if path.suffix in (".py", ".md", ".json"):
            txt = path.read_text()
            assert not re.search(r"TK-[A-Z0-9]{6}", txt) and not re.search(r"#\d{3,}", txt), path.name
    for name in ("task.py", "scene.py"):
        code = (HERE / name).read_text()
        assert "weld(" not in code and "write_pose" not in code and "set_joint" not in code and "push(" not in code
