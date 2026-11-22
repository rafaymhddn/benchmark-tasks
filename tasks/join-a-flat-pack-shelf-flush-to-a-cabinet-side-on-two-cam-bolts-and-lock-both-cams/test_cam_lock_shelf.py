"""Unit tests for the Minifix 15 cam-lock shelf task (scene figures, geometry, bilateral axial-torque coupling,
all score levels 0 / 0.25 / 0.50 / 1.00, whole-trial zero rules, and folder checklist limits) without the simulator."""

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
FAR = {"left": np.array([[0.0, 0.30, 1.15]]), "right": np.array([[0.0, -0.30, 1.15]])}
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


def shelf_at(m, gap: float = 0.0, lateral: float = 0.0, tilt_deg: float = 0.0, yaw_deg: float = 0.0, drop: float = 0.0):
    rot = m.mk.axis_angle((0.0, 0.0, 1.0), math.radians(yaw_deg)) @ m.mk.axis_angle((0.0, 1.0, 0.0), math.radians(tilt_deg))
    pos = np.array([m.P.X_SIDE - gap - m.P.SHELF_D / 2.0, lateral, m.TOP + m.P.Z_MID - drop])
    return pos, rot


def put(m, *, gap: float = 0.042, lateral: float = 0.0, tilt_deg: float = 0.0, yaw_deg: float = 0.0, drop: float = 0.0,
        cam_degs: tuple[float, float] = (90.0, 90.0), clamp_n: tuple[float, float] = (0.0, 0.0),
        drivers: dict | None = None, contacts: dict | None = None, hands=FAR) -> None:
    sh = shelf_at(m, gap=gap, lateral=lateral, tilt_deg=tilt_deg, yaw_deg=yaw_deg, drop=drop)
    joints = {
        "cam_joint_l": m.P.travel_from_cam_deg(cam_degs[0]),
        "cam_joint_r": m.P.travel_from_cam_deg(cam_degs[1]),
    }
    # Place shelf first, then compute cam poses riding on the moved shelf
    m.mk.set_state(poses={"shelf": sh}, joints=joints)
    cam_poses = {c: m.mk.joint_pose(m.mk.MECH.joints[f"cam_joint_{c}"], joints[f"cam_joint_{c}"], sh) for c in ("l", "r")}
    all_poses = {"shelf": sh, "cam_l": cam_poses["l"], "cam_r": cam_poses["r"], **(drivers or {})}
    all_contacts = {
        ("cam_l", "cabinet_side"): clamp_n[0],
        ("cam_r", "cabinet_side"): clamp_n[1],
        **(contacts or {}),
    }
    m.mk.set_state(poses=all_poses, joints=joints, contacts=all_contacts)
    m.STATE["hands"] = hands


def steps(m, n: int) -> float:
    for _ in range(n):
        m.step()
    return m.score(None)


def test_scene_figures_and_instruction(m):
    assert m.SCORE_OVER == "end" and m.SUCCESS_HOLD_S == 3.0 and m.RUN_LIMIT_STEPS == 1800
    low = m.INSTRUCTION.lower()
    for word in ("shelf", "flush", "upright cabinet side", "two cam bolts", "cam locks", "stubby screwdrivers", "cradles",
                 "move both hands clear"):
        assert word in low
    for forbidden in ("danger", "hazard", "unsafe"):
        assert forbidden not in low
    assert m.P.PANEL_T == pytest.approx(0.016)
    assert m.P.CAM_BORE_D == pytest.approx(0.015) and m.P.DIM_B == pytest.approx(0.024)
    assert m.P.EDGE_BORE_D == pytest.approx(0.008) and m.P.BOLT_HEAD_D == pytest.approx(0.007)
    assert m.P.RADIAL_PLAY == pytest.approx(0.0005)  # 0.50 mm exact radial clearance
    assert m.P.CAM_Y == pytest.approx(0.110)  # 220 mm connector spacing (>= 200 mm wrist separation)
    for c in ("l", "r"):
        j = m.mk.MECH.joints[f"cam_joint_{c}"]
        assert j.kind == "hinge" and -math.pi < j.lower < 0.0 < j.upper < math.pi
    put(m)
    assert m.cam_deg("l") == pytest.approx(90.0) and m.cam_deg("r") == pytest.approx(90.0)
    assert m.driver_parked("l") and m.driver_parked("r")
    assert steps(m, 5) == 0.0 and m.zero_now() is None


def test_connector_stop_and_spiral_geometry(m):
    p = m.P
    # Closed Ø 15 mm cam outer wall sits at 24.0 - 7.45 = 16.55 mm inside shelf edge; 25.0 mm bolt stops 8.45 mm short
    blocked_gap = p.BOLT_PROJ - (p.DIM_B - p.CAM_OUTER_R)
    assert blocked_gap == pytest.approx(0.00845, abs=1e-4) and blocked_gap > p.SEATED_GAP_MAX
    # When open (cam_deg = 0 deg), every spiral crescent point clears the Ø 7.0 mm bolt head at |z| >= 2.80 mm (|y| <= 2.10 mm, x_c >= -1.0 mm)
    bolt_chord_y = 0.00210  # half-width of the inscribed 4.2 x 5.6 mm shoulder & cylinder at |z| >= 2.80 mm
    x_tip_open = -(p.BOLT_PROJ - p.DIM_B)
    for deg in np.linspace(p.SPIRAL_START_DEG, p.SPIRAL_END_DEG, 40):
        r = p.spiral_r_in(float(deg))
        xc, yc = r * math.cos(math.radians(float(deg))), r * math.sin(math.radians(float(deg)))
        dy = max(0.0, abs(yc) - bolt_chord_y)
        dx = max(0.0, x_tip_open - xc)
        assert math.hypot(dx, dy) > 0.00020
    # Captures bolt head back shoulder at gap = 2.2 mm and preloads 0.12 mm at gap = 0.0 mm
    x_back_flush = p.DIM_B - p.BOLT_BACK_FROM_SIDE  # 1.80 mm
    assert p.spiral_r_in(p.SPIRAL_START_DEG) > x_back_flush + 0.0022
    assert p.spiral_r_in(180.0) == pytest.approx(0.00168) and p.spiral_r_in(180.0) < x_back_flush


def test_driver_cam_coupling_is_axial_torque_only_and_gated_on_depth_and_force(m):
    put(m, gap=0.042, cam_degs=(90.0, 90.0))
    p_cam, _ = m.mk.pose("cam_l")
    # Place driver_l over cam_l, twisted 8 deg clockwise ahead of the 45 deg cross-slot phase
    twist_cw = math.radians(8.0)
    base_yaw = math.radians(m.P.DRV_BLADE_PHASE_DEG) - twist_cw
    r_drv = m.mk.axis_angle((0.0, 0.0, 1.0), base_yaw) @ np.diag([1.0, -1.0, -1.0])
    # 1. Hovering 1.0 mm into recess (< 1.5 mm minimum depth) even with force -> 0 torque
    z_shallow = p_cam[2] + m.P.PANEL_T / 2.0 - 0.0010 + m.P.DRV_TIP_Z
    put(m, drivers={"driver_l": (np.array([p_cam[0], p_cam[1], z_shallow]), r_drv)}, contacts={("driver_l", "cam_l"): 5.0})
    m.step()
    assert np.allclose(m.STATE["couplings"]["cam_l"], 0.0) and np.allclose(m.STATE["couplings"]["driver_l"], 0.0)

    # 2. Deep enough (2.5 mm >= 1.5 mm) but light touch (1.0 N < 1.5 N) -> 0 torque
    z_seated = p_cam[2] + m.P.PANEL_T / 2.0 - 0.0025 + m.P.DRV_TIP_Z
    put(m, drivers={"driver_l": (np.array([p_cam[0], p_cam[1], z_seated]), r_drv)}, contacts={("driver_l", "cam_l"): 1.0})
    m.step()
    assert np.allclose(m.STATE["couplings"]["cam_l"], 0.0)

    # 3. Seated (2.5 mm >= 1.5 mm) AND pressed (4.0 N >= 1.5 N) -> equal-and-opposite purely axial torque
    put(m, drivers={"driver_l": (np.array([p_cam[0], p_cam[1], z_seated]), r_drv)}, contacts={("driver_l", "cam_l"): 4.0})
    m.step()
    tau_cam = np.array(m.STATE["couplings"]["cam_l"])
    tau_drv = np.array(m.STATE["couplings"]["driver_l"])
    assert tau_cam[2] < -0.10  # +tau along (0, 0, -1) -> negative world z = clockwise
    assert np.allclose(tau_cam + tau_drv, 0.0, atol=1e-12)  # exact equal-and-opposite reaction
    assert np.allclose(tau_cam[:2], 0.0, atol=1e-12)  # strictly axial: zero lateral or tilt torque

    # 4. Tilted 20 deg (> 15 deg limit, as when released and tipping) -> disengages immediately to 0
    r_tilted = m.mk.axis_angle((1.0, 0.0, 0.0), math.radians(20.0)) @ r_drv
    tip_target = np.array([p_cam[0], p_cam[1], p_cam[2] + m.P.PANEL_T / 2.0 - 0.0025])
    pos_tilted = tip_target - r_tilted @ np.array([0.0, 0.0, m.P.DRV_TIP_Z])
    put(m, drivers={"driver_l": (pos_tilted, r_tilted)}, contacts={("driver_l", "cam_l"): 4.0})
    m.step()
    assert np.allclose(m.STATE["couplings"]["cam_l"], 0.0)


def test_score_levels_quarter_half_and_full(m):
    # Opening both cams without aligning and sliding the shelf onto the bolts stays 0.0
    put(m, gap=0.042, cam_degs=(0.0, 0.0))
    assert steps(m, 5) == 0.0

    # Closed-cam stop at 8.5 mm gap scores 0.0
    put(m, gap=0.0085, cam_degs=(90.0, 90.0))
    assert steps(m, 5) == 0.0

    # Seated on both bolts (gap <= 2.5 mm) with cams open -> 0.25
    put(m, gap=0.0018, cam_degs=(0.0, 0.0))
    assert steps(m, 5) == 0.25

    # Seated + one cam locked in [150, 195] deg with >= 5.0 N clamp -> 0.50 (without force -> still 0.25)
    put(m, gap=0.0010, cam_degs=(175.0, 0.0), clamp_n=(2.0, 0.0))
    assert steps(m, 5) == 0.25
    put(m, gap=0.0010, cam_degs=(175.0, 0.0), clamp_n=(12.0, 0.0))
    assert steps(m, 5) == 0.50

    # Flush (<= 0.6 mm), both cams locked (>= 5.0 N each), both drivers parked, hands clear -> 1.0 after 20 steps (1.0 s)
    put(m, gap=0.0002, cam_degs=(178.0, 178.0), clamp_n=(15.0, 15.0))
    assert steps(m, m.P.HOLD_STEPS - 1) == 0.50
    assert steps(m, 1) == 1.00 and m.success(None)


def test_unparked_driver_hand_nearness_and_hold_drift(m):
    # Unparked screwdriver lying on top of the shelf caps score at 0.50
    on_shelf = (np.array([0.28, 0.0, m.TOP + m.P.Z_SHELF_TOP + 0.015]), EYE)
    put(m, gap=0.0002, cam_degs=(178.0, 178.0), clamp_n=(15.0, 15.0), drivers={"driver_r": on_shelf})
    assert not m.driver_parked("r") and steps(m, m.P.HOLD_STEPS + 5) == 0.50

    # Hand within 20 mm (< 30 mm clearance) keeps score at 0.50
    m.begin(0)
    near_hand = np.array([[0.29, 0.0, m.TOP + m.P.Z_SHELF_TOP + 0.018]])
    put(m, gap=0.0002, cam_degs=(178.0, 178.0), clamp_n=(15.0, 15.0), hands={"left": near_hand, "right": FAR["right"]})
    assert m.hands_clear() < m.P.HANDS_CLEAR and steps(m, m.P.HOLD_STEPS + 5) == 0.50

    # Drift > 1.0 mm during hold resets hold counter
    m.begin(0)
    put(m, gap=0.0002, cam_degs=(178.0, 178.0), clamp_n=(15.0, 15.0))
    assert steps(m, m.P.HOLD_STEPS - 4) == 0.50
    put(m, gap=0.0002, lateral=0.0015, cam_degs=(178.0, 178.0), clamp_n=(15.0, 15.0))
    assert steps(m, 4) == 0.50 and m.STATE["hold"] == 3
    assert steps(m, m.P.HOLD_STEPS - 3) == 1.00


@pytest.mark.parametrize(
    "kwargs, expected",
    [
        ({"drop": 0.020}, "the shelf dropped off the support block"),
    ],
)
def test_whole_trial_zero_rules_latch_permanently(m, kwargs, expected):
    put(m, **kwargs)
    steps(m, 1)
    assert m.STATE["zero"] == expected
    put(m, gap=0.0002, cam_degs=(178.0, 178.0), clamp_n=(15.0, 15.0))
    assert steps(m, m.P.HOLD_STEPS + 5) == 0.0 and not m.success(None)


def test_a_tilted_or_yawed_shelf_is_no_zero(m):
    """A shelf tipped 11 deg or turned 22 deg on the riser is neither damage nor a fall: no zero, full score once joined."""
    for kwargs in ({"tilt_deg": 11.0}, {"yaw_deg": 22.0}):
        m.begin(0)
        put(m, **kwargs)
        steps(m, 3)
        assert m.STATE.get("zero") is None
        put(m, gap=0.0002, cam_degs=(178.0, 178.0), clamp_n=(15.0, 15.0))
        assert steps(m, m.P.HOLD_STEPS + 1) == 1.00 and m.success(None)


def test_dropped_driver_latches_zero(m):
    on_bench = (np.array([0.22, -0.28, m.TOP + 0.015]), EYE)
    put(m, drivers={"driver_r": on_bench})
    steps(m, 1)
    assert m.STATE["zero"] == "screwdriver r fell to the bench"
    put(m, gap=0.0002, cam_degs=(178.0, 178.0), clamp_n=(15.0, 15.0))
    assert steps(m, m.P.HOLD_STEPS + 5) == 0.0


def test_folder_checklist_limits():
    task_text = (HERE / "task.py").read_text()
    assert len(task_text) <= 15_000
    readme_path = HERE / "README.md"
    if readme_path.exists():
        readme = readme_path.read_text()
        assert len(readme) <= 10_000
        low = readme.lower()
        assert "earlier review" not in low and "reviewer" not in low and "thank" not in low
    for path in HERE.iterdir():
        assert not path.is_symlink(), path.name
        if path.suffix in (".py", ".md", ".json"):
            text = path.read_text()
            assert not re.search(r"TK-[A-Z0-9]{6}", text) and not re.search(r"#\d{3,}", text), path.name
    for name in ("task.py", "scene.py", "demo.py"):
        if (HERE / name).exists():
            code = (HERE / name).read_text()
            assert "weld(" not in code and "write_pose" not in code and "set_joint" not in code, name
