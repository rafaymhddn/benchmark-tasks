"""Pure unit tests of the CryoCane sleeving task's scene, figures, zero-snap coaxial slide mechanics, and scoring rules.
Every state is set directly without starting Isaac Sim."""

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
    module.mk.mend_all()
    module.begin(0)
    yield module
    module.mk.MECH.clear()
    for name, old in before.items():
        sys.modules.pop(name, None)
        if old is not None:
            sys.modules[name] = old


def cane_pose_world(m, pos=None, yaw: float = 0.0, roll_deg: float = 0.0, pitch_deg: float = 0.0):
    p = np.array(m.scene.cane_start_pos(m.TOP) if pos is None else pos, dtype=float)
    r = (
        m.mk.quat_to_matrix(m.mk.yaw_quat(yaw))
        @ m.mk.axis_angle((1.0, 0.0, 0.0), math.radians(pitch_deg))
        @ m.mk.axis_angle((0.0, 1.0, 0.0), math.radians(roll_deg))
    )
    return p, r


def vials_in_bays(m, cane_pr, radial_off: float = 0.0, axial_off: float = 0.0) -> dict:
    p_c, r_c = cane_pr
    out = {}
    for idx, name in enumerate(m.VIALS):
        bay = m.P.VIAL_BAYS[idx]
        off_r = radial_off if idx == 0 else 0.0
        off_a = axial_off if idx == 0 else 0.0
        vp = p_c + r_c @ np.array([off_r, m.P.bay_centre_y(bay) + off_a, m.P.VIAL_REST_Z])
        out[name] = (vp, r_c)
    return out


def mated_state(m, cane_pr, slide_m: float, tray_pr=None, hands=FAR, contacts=None, velocities=None) -> None:
    p_c, r_c = cane_pr
    u_c = r_c[:, 1]
    j_val = slide_m - m.P.CARRIAGE_BUILT_S
    slv_pos = p_c + (slide_m - 0.5 * m.P.SLEEVE_L) * u_c
    c_pos, c_rot = m.mk.joint_pose(m.mk.MECH.joints["slv_s"], j_val, cane_pr)
    poses = {"cane": cane_pr, **vials_in_bays(m, cane_pr), "c_slv": (c_pos, c_rot), "sleeve": (slv_pos, r_c)}
    if tray_pr is not None:
        poses["tray"] = tray_pr
    m.mk.mend_all()
    m.mk.set_state(poses=poses, joints={"slv_s": j_val}, contacts=contacts or {}, velocities=velocities or {})
    m.mk.MECH.poses["c_slv"] = (c_pos, c_rot)
    m.mk.MECH.poses["sleeve"] = (slv_pos, r_c)
    m.mk.MECH.travel["slv_s"] = float(j_val)
    m.mk.make("w_sleeve")
    m.STATE["slv_rest"] = float(j_val)
    m.STATE["max_slide_m"] = max(m.STATE.get("max_slide_m", 0.0), float(slide_m))
    m.STATE["hands"] = hands


def steps(m, n: int) -> float:
    for _ in range(n):
        m.step()
    return m.score(None)


def test_scene_and_catalog_geometry(m):
    P = m.P
    assert m.SCORE_OVER == "end" and m.RUN_LIMIT_STEPS == 1800
    assert P.CANE_L == pytest.approx(0.290) and P.SLEEVE_L == pytest.approx(0.273)
    assert P.VIAL_OD == pytest.approx(0.0125) and P.VIAL_L == pytest.approx(0.049)
    stack_m = (P.VIAL_REST_Z + P.VIAL_R) - P.BASE_Z
    assert stack_m == pytest.approx(0.0132, abs=1e-5)
    assert P.SLEEVE_IN_FLAT >= 0.0160
    assert P.SLEEVE_IN_FLAT / math.cos(math.pi / 8.0) - 2.0 * P.CONTACT_OFF >= 0.0175
    assert P.TAB_W >= 0.0210 and P.TAB_W > 2.0 * P.SLEEVE_R_OUT
    for s in m.mk.MECH.bodies["cane"].shapes:
        if not s.solid or s.pos[1] >= P.TAB_SHOULDER_Y:
            continue
        half = s.half_extents()
        rot = m.mk.quat_to_matrix(s.quat)
        corners = [
            np.asarray(s.pos) + rot @ (np.array([sx, sy, sz]) * half)
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
            for sz in (-1.0, 1.0)
        ]
        max_r = max(math.hypot(float(c[0]), float(c[2])) for c in corners)
        assert max_r < P.SLEEVE_R_IN - P.CONTACT_OFF
    cane_p0, cane_r0 = m.mk.built_pose("cane")
    tab_end_w = cane_p0 + cane_r0 @ np.array([0.0, P.CANE_L, 0.0])
    slv_p0, _ = m.mk.built_pose("sleeve")
    assert tab_end_w[1] == pytest.approx(-0.012)
    assert slv_p0[1] + 0.5 * P.SLEEVE_L == pytest.approx(-0.025)


def test_zero_snap_foot_engagement_and_5n_vs_35n_wipe(m):
    cane_pr = cane_pose_world(m, pos=(0.25, 0.00, m.TOP + 0.095), yaw=0.0)
    p_c, _ = cane_pr
    # Approaching bent top tab: never engages
    bad_slv = (p_c + np.array([0.0, m.P.CANE_L + 0.5 * m.P.SLEEVE_L - 0.005, 0.0]), EYE)
    m.mk.set_state(poses={"cane": cane_pr, **vials_in_bays(m, cane_pr), "sleeve": bad_slv})
    m.STATE["hands"] = FAR
    m.step()
    assert not m.sleeve_mated() and m.score(None) == 0.0

    # Coaxial telescope at 1.85 deg and s_mouth = 38 mm (> old 1.0 deg / 24 mm cliff): engages in-place without moving sleeve!
    good_rot = m.mk.axis_angle((0.0, 0.0, 1.0), math.radians(1.85))
    mouth_target = p_c + np.array([0.001, 0.038, 0.0])
    good_slv_pos = mouth_target - good_rot @ np.array([0.0, 0.5 * m.P.SLEEVE_L, 0.0])
    m.mk.set_state(poses={"cane": cane_pr, **vials_in_bays(m, cane_pr), "sleeve": (good_slv_pos, good_rot)})
    m.step()
    assert m.sleeve_mated()
    assert m.mk.pose("sleeve")[0] == pytest.approx(good_slv_pos)
    assert m.mk.pose("sleeve")[1] == pytest.approx(good_rot)

    # Geometric coaxial enclosure (e.g. 3.0 deg off-axis about overlap midpoint, w_sleeve unmated) scores 0.25 and 0.5!
    m.mk.mend_all()
    m.begin(0)
    geom_rot = m.mk.axis_angle((0.0, 0.0, 1.0), math.radians(3.0))
    for s_val, expected in ((0.140, 0.25), (0.263, 0.5)):
        pivot = p_c + np.array([0.0, 0.5 * s_val, 0.0])
        g_pos = pivot + geom_rot @ np.array([0.0, 0.5 * s_val - 0.5 * m.P.SLEEVE_L, 0.0])
        m.mk.set_state(poses={"cane": cane_pr, **vials_in_bays(m, cane_pr), "sleeve": (g_pos, geom_rot)})
        m.STATE["hands"] = FAR
        assert not m.sleeve_mated() and steps(m, 2) == expected

    # Coaxial advance to 120 mm -> 5.0 N wipe; asymmetric >0.35 N m push -> 35.0 N bind
    mated_state(m, cane_pr, 0.120)
    m.STATE["slv_rest"] = 0.0
    m.step()
    assert m.STATE["slide_force_n"] == pytest.approx(m.P.SLIDE_N, rel=1e-3)

    p_s, _ = m.mk.pose("sleeve")
    c_cock = m.mk.Contact("right robot", 2, -1, p_s + np.array([0.011, -0.110, 0.0]), np.array([-1.0, 0.0, 0.0]), 0.0, 6.0)
    mated_state(m, cane_pr, 0.150, contacts={"sleeve": [c_cock]})
    m.STATE["slv_rest"] = 0.100
    m.step()
    assert m.STATE["slide_force_n"] == pytest.approx(m.P.BIND_N, rel=1e-3)


def test_idle_start_scores_zero_with_no_fault(m):
    m.mk.set_state()
    m.STATE["hands"] = FAR
    assert steps(m, 5) == 0.0 and m.STATE["zero"] is None


def test_spilling_a_vial_latches_zero(m):
    cane_pr = cane_pose_world(m, pos=(0.25, 0.10, m.TOP + 0.095), yaw=0.0)
    m.mk.set_state(poses={"cane": cane_pr, **vials_in_bays(m, cane_pr, radial_off=0.020)})
    m.STATE["hands"] = FAR
    assert steps(m, 1) == 0.0 and "spilled" in m.STATE["zero"]
    in_tray_cane = cane_pose_world(m, pos=(m.P.TRAY_CX, m.P.TRAY_CY - 0.135, m.TOP + m.P.TRAY_WALL + m.P.SLEEVE_R_OUT), yaw=0.0)
    mated_state(m, in_tray_cane, m.P.SLIDE_MAX)
    assert steps(m, m.P.HOLD_STEPS + 5) == 0.0


def test_unsleeved_cane_in_tray_latches_zero(m):
    in_tray_cane = cane_pose_world(m, pos=(m.P.TRAY_CX, m.P.TRAY_CY - 0.135, m.TOP + 0.015), yaw=0.0)
    m.mk.set_state(poses={"cane": in_tray_cane, **vials_in_bays(m, in_tray_cane)})
    m.STATE["hands"] = FAR
    assert steps(m, 1) == 0.0 and m.STATE["zero"] == "unsleeved cane or vial entered the transfer tray"


def test_score_levels_quarter_half_and_full_with_hold_and_clearance(m):
    cane_air = cane_pose_world(m, pos=(0.25, 0.00, m.TOP + 0.095), yaw=0.0)
    mated_state(m, cane_air, 0.110)
    assert steps(m, 2) == 0.0
    mated_state(m, cane_air, 0.135)
    assert steps(m, 2) == 0.25
    mated_state(m, cane_air, 0.263)
    assert m.gap_to_tab_m() == pytest.approx(0.005) and m.foot_enclosed()
    assert steps(m, 2) == 0.5

    in_tray_cane = cane_pose_world(
        m,
        pos=(m.P.TRAY_CX, m.P.TRAY_CY - 0.135, m.TOP + m.P.TRAY_WALL + m.P.SLEEVE_R_OUT),
        yaw=0.0,
    )
    near_hand = {"left": np.array([[m.P.TRAY_CX, m.P.TRAY_CY, m.TOP + 0.040]]), "right": FAR["right"]}
    mated_state(m, in_tray_cane, m.P.SLIDE_MAX, hands=near_hand)
    assert steps(m, m.P.HOLD_STEPS + 3) == 0.5

    mated_state(m, in_tray_cane, m.P.SLIDE_MAX, hands=FAR)
    assert steps(m, m.P.HOLD_STEPS - 1) == 0.5
    # Shifting 4 mm (> 3 mm HOLD_DRIFT) restarts the 20-step hold
    shifted = (in_tray_cane[0] + np.array([0.004, 0.0, 0.0]), in_tray_cane[1])
    mated_state(m, shifted, m.P.SLIDE_MAX, hands=FAR)
    assert steps(m, 1) == 0.5 and m.STATE["hold"] == 1
    # Benign tray shift > 25 mm on the bench does NOT zero the trial (Rules 4 & 7)
    shifted_tray = (np.array(m.scene.tray_start_pos(m.TOP)) + np.array([0.0, 0.045, 0.0]), EYE)
    cane_in_shifted = cane_pose_world(
        m,
        pos=(m.P.TRAY_CX, m.P.TRAY_CY + 0.045 - 0.135, m.TOP + m.P.TRAY_WALL + m.P.SLEEVE_R_OUT),
        yaw=0.0,
    )
    mated_state(m, cane_in_shifted, m.P.SLIDE_MAX, tray_pr=shifted_tray, hands=FAR)
    assert steps(m, m.P.HOLD_STEPS) == 1.0 and m.STATE["zero"] is None


def test_folder_invariants_and_no_pose_snap_rule():
    import json

    spec = json.loads((HERE / "task.json").read_text())
    assert len(spec["title"]) <= 120 and len(spec["context"]) <= 1000 and len(spec["fidelity"]) <= 500
    task_txt = (HERE / "task.py").read_text()
    assert len(task_txt) <= 15_000
    # Enforce t22's rule: zero write_pose / set_joint / mk.push in task.py, scene.py and demo.py
    for name in ("task.py", "scene.py", "demo.py"):
        code = (HERE / name).read_text()
        for banned in ("write_pose(", "set_joint(", "mk.push("):
            assert banned not in code, f"{name} contains {banned}"
    for path in HERE.iterdir():
        assert not path.is_symlink(), path.name
        if path.suffix in (".py", ".md", ".json"):
            txt = path.read_text()
            assert not re.search(r"TK-[A-Z0-9]{6}", txt) and not re.search(r"#\d{3,}", txt), path.name
    if (HERE / "README.md").is_file():
        readme = (HERE / "README.md").read_text()
        assert len(readme) <= 10_000 and len(readme.encode()) <= 10_000
        low = readme.lower()
        assert "earlier review" not in low and "reviewer" not in low and "thank" not in low
