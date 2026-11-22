"""Pure rule, geometry, and source-pass tests for ``mech/tasks/systainer_tloc`` (no simulator needed)."""

from __future__ import annotations

import math
from pathlib import Path
import re
import sys

import numpy as np
import pytest
from yam_arena import task_file

HERE = Path(__file__).resolve().parent
TASK = HERE / "task.py"
FAR_HANDS = {"left": np.array([[0.0, 0.35, 1.15]]), "right": np.array([[0.0, -0.35, 1.15]])}
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


def pose_pack(
    m,
    *,
    low_shift=(0.0, 0.0, 0.0),
    low_yaw_deg: float = 0.0,
    lid_deg: float = 0.0,
    tloc_deg: float = 0.0,
    box_in_cell: bool = True,
    box_dz: float = 0.0,
    box_tilt_deg: float = 0.0,
    top_on_low: bool = False,
    top_dx: float = 0.0,
    top_dz: float = 0.0,
    hands=FAR_HANDS,
) -> None:
    r_low = m.mk.axis_angle((0.0, 0.0, 1.0), math.radians(low_yaw_deg))
    p_low = np.array([m.P.CASE_LOW_XY[0] + low_shift[0], m.P.CASE_LOW_XY[1] + low_shift[1], m.TOP + low_shift[2]])
    poses: dict = {"case_low": (p_low, r_low)}
    if box_in_cell:
        rel_box = np.array([m.P.CELL_LOCAL_XY[0], m.P.CELL_LOCAL_XY[1], m.P.FLOOR_TOP_Z + box_dz])
        r_box = r_low @ m.mk.axis_angle((1.0, 0.0, 0.0), math.radians(box_tilt_deg))
        poses["parts_box"] = (p_low + r_low @ rel_box, r_box)
    if top_on_low:
        rel_top = np.array([top_dx, 0.0, m.P.LID_DECK_Z + top_dz])
        poses["case_top"] = (p_low + r_low @ rel_top, r_low)
    m.mk.set_state(
        poses=poses,
        joints={
            "lid_hinge": math.radians(lid_deg) - m.P.LID_OPEN_RAD,
            "tloc": math.radians(tloc_deg) - m.P.TLOC_OPEN_RAD,
        },
    )
    m.STATE["hands"] = hands


def advance(m, n: int) -> float:
    for _ in range(n):
        m.step()
    return m.score(None)


def test_scene_joints_masses_and_drive_caps(m):
    P = m.P
    bodies, joints = m.mk.MECH.bodies, m.mk.MECH.joints
    assert sorted(bodies) == ["case_low", "case_top", "lid_low", "parts_box", "tloc_low"]
    assert sorted(joints) == ["lid_hinge", "tloc"]
    assert joints["tloc"].parent == "case_low" and joints["tloc"].child == "tloc_low"
    assert joints["lid_hinge"].parent == "case_low" and joints["lid_hinge"].child == "lid_low"
    assert bodies["case_low"].mass + bodies["lid_low"].mass + bodies["tloc_low"].mass == pytest.approx(0.40)
    assert bodies["case_top"].mass == pytest.approx(0.48)
    assert P.LID_DECK_Z + P.CASE_H == pytest.approx(0.147)
    # Hinge-drive targets stay strictly within 45 deg of the joint angle across the whole travel
    for deg in np.linspace(P.TLOC_MIN_DEG, P.TLOC_MAX_DEG, 181):
        rad = math.radians(deg)
        assert abs(math.degrees(P.tloc_stepped_rest(rad) - rad)) <= 40.0 + 1e-6
    for deg in np.linspace(0.0, P.LID_MAX_DEG, 63):
        rad = math.radians(deg)
        assert abs(math.degrees(P.lid_stay_rest(rad) - rad)) <= 40.0 + 1e-6
    assert abs(P.tloc_torque_nm(math.radians(40.0))) == pytest.approx(P.TLOC_DETENT_NM, rel=1e-4)
    assert P.tloc_torque_nm(P.TLOC_OPEN_RAD) == pytest.approx(0.0, abs=1e-9)
    assert P.tloc_torque_nm(P.TLOC_CLOSE_RAD) == pytest.approx(0.0, abs=1e-9)
    assert P.tloc_torque_nm(P.TLOC_CONNECT_RAD) == pytest.approx(0.0, abs=1e-9)


def test_idle_and_missing_or_cocked_box_score_zero(m):
    pose_pack(m, lid_deg=m.P.LID_OPEN_DEG, tloc_deg=m.P.TLOC_OPEN_DEG, box_in_cell=False, top_on_low=False)
    assert advance(m, 5) == 0.0 and m.STATE["zero"] is None
    # Closing lid without stowing parts_box inside scores 0.0
    pose_pack(m, lid_deg=0.0, tloc_deg=0.0, box_in_cell=False, top_on_low=False)
    assert advance(m, 5) == 0.0
    # Cocked box on divider rib or lid ajar (> 2.5 deg) scores 0.0
    pose_pack(m, lid_deg=8.0, tloc_deg=-40.0, box_in_cell=True, box_dz=0.044, box_tilt_deg=22.0)
    assert not m.box_stowed() and not m.lid_closed() and advance(m, 5) == 0.0


def test_stowed_box_with_closed_lid_scores_quarter(m):
    pose_pack(m, lid_deg=0.0, tloc_deg=0.0, box_in_cell=True, top_on_low=False)
    assert m.box_stowed() and m.lid_closed() and not m.upper_seated()
    assert advance(m, 25) == 0.25


def test_unslid_upper_case_perched_or_forward_stays_at_quarter(m):
    # Dropped straight down on top of rear cleats without sliding (dz = +9 mm proud, hooks above cleat roof)
    pose_pack(m, lid_deg=0.0, tloc_deg=90.0, box_in_cell=True, top_on_low=True, top_dx=0.0, top_dz=0.009)
    sm = m.stack_metrics()
    assert not sm["cleats_engaged"] and sm["max_seam_m"] > m.P.SEAM_GAP_MAX
    assert advance(m, 25) == 0.25
    # Placed 12 mm forward on lower lid without rearward slide into cleats
    pose_pack(m, lid_deg=0.0, tloc_deg=0.0, box_in_cell=True, top_on_low=True, top_dx=-0.012, top_dz=0.005)
    sm2 = m.stack_metrics()
    assert not sm2["cleats_engaged"] and advance(m, 25) == 0.25


def test_slid_into_cleats_without_tloc_connect_scores_half(m):
    pose_pack(m, lid_deg=0.0, tloc_deg=0.0, box_in_cell=True, top_on_low=True, top_dx=0.0, top_dz=0.0)
    assert m.upper_seated() and not m.tloc_connected()
    assert advance(m, 25) == 0.5


def test_coupled_stack_in_local_frame_reaches_full_after_1s_hold(m):
    # Shifted 18 mm and yawed 6 deg within bench mark tolerance: scorer reads in case_low's own frame
    pose_pack(
        m,
        low_shift=(0.012, -0.014, 0.0),
        low_yaw_deg=6.0,
        lid_deg=0.0,
        tloc_deg=90.0,
        box_in_cell=True,
        top_on_low=True,
    )
    assert m.upper_seated() and m.tloc_connected() and m.hands_clear_m() >= m.P.HANDS_CLEAR_M
    assert advance(m, m.P.HOLD_STEPS - 1) == 0.5
    assert advance(m, 1) == 1.0 and m.success(None)


def test_hand_too_close_or_stack_drift_blocks_full_score(m):
    near = np.array([[m.P.CASE_LOW_XY[0], 0.145, m.TOP + 0.090]])  # ~12.5 mm from case wall (< 30 mm)
    pose_pack(
        m,
        lid_deg=0.0,
        tloc_deg=90.0,
        box_in_cell=True,
        top_on_low=True,
        hands={"left": near, "right": FAR_HANDS["right"]},
    )
    assert m.hands_clear_m() < m.P.HANDS_CLEAR_M
    assert advance(m, m.P.HOLD_STEPS + 5) == 0.5
    # Clear hands, run 15 steps, then creep 4 mm (> 2 mm hold drift): hold resets
    pose_pack(m, lid_deg=0.0, tloc_deg=90.0, box_in_cell=True, top_on_low=True)
    assert advance(m, 15) == 0.5
    pose_pack(m, low_shift=(0.004, 0.0, 0.0), lid_deg=0.0, tloc_deg=90.0, box_in_cell=True, top_on_low=True)
    assert advance(m, 5) == 0.5 and m.STATE["hold"] == 5
    assert advance(m, m.P.HOLD_STEPS - 5) == 1.0


def test_whole_trial_fall_zero_latches_and_bench_shift_stays_valid(m):
    # Benign bench shift / yaw on case_low does NOT trigger a zero rule (evaluated in case_low's own frame)
    pose_pack(m, low_shift=(0.060, -0.025, 0.0), low_yaw_deg=26.0, lid_deg=0.0, tloc_deg=90.0, box_in_cell=True, top_on_low=True)
    assert advance(m, m.P.HOLD_STEPS) == 1.0 and m.STATE["zero"] is None
    # Fall off bench (> 30 mm below bench top) latches 0.0 for the rest of the trial
    for name in ("parts_box", "case_top", "case_low"):
        m.begin(0)
        pose_pack(m, lid_deg=0.0, tloc_deg=90.0, box_in_cell=True, top_on_low=True)
        m.mk.MECH.poses[name] = (np.array([0.20, 0.35, m.TOP - 0.045]), EYE)
        assert advance(m, 1) == 0.0 and f"{name} fell" in m.STATE["zero"]
        pose_pack(m, lid_deg=0.0, tloc_deg=90.0, box_in_cell=True, top_on_low=True)
        assert advance(m, m.P.HOLD_STEPS + 5) == 0.0 and m.STATE["hold"] == 0


def test_folder_hygiene_and_source_pass_rules():
    skip_self = {"mechkit.py", "demo_kit.py", "kit_extra.py", "test_systainer_tloc.py"}
    for path in HERE.iterdir():
        assert not path.is_symlink(), f"symlink forbidden: {path.name}"
        if path.suffix in (".py", ".md", ".json") and path.name not in skip_self:
            text = path.read_text()
            assert not re.search(r"TK-[A-Z0-9]{6}", text), path.name
            assert not re.search(r"#\d{3,}", text), path.name
            low = text.lower()
            for banned in ("assumed until", "hand-tested", "bench-verified", "measured by hand"):
                assert banned not in low, f"{banned!r} in {path.name}"
    for name in ("task.py", "scene.py"):
        code = (HERE / name).read_text()
        assert "weld(" not in code and "push(" not in code and ".push" not in code
    for name in ("task.py", "scene.py", "demo.py"):
        p = HERE / name
        if p.exists():
            code = p.read_text()
            assert "write_pose" not in code and "set_joint" not in code, f"pose snap forbidden in {name}"
    assert len((HERE / "task.py").read_text()) <= 15_000
    readme = HERE / "README.md"
    if readme.exists():
        rtxt = readme.read_text()
        assert len(rtxt) <= 10_000
        rlow = rtxt.lower()
        assert "earlier review" not in rlow and "reviewer" not in rlow and "thank" not in rlow
