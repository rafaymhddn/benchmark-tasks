"""Pure unit tests of the CryoCane sleeving task's scene, figures, the fit of the cane inside the sleeve, the geometric
reading of a threaded sleeve, and the scoring rules. Every state is set directly without starting Isaac Sim."""

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


def threaded_state(m, cane_pr, slide_m: float, tray_pr=None, hands=FAR, cock_deg: float = 0.0, offset=(0.0, 0.0, 0.0)) -> None:
    """Cane and vials at ``cane_pr``; the sleeve on the stack's middle line (turned ``cock_deg`` about the vertical at
    the middle of the overlap, shifted ``offset`` in the cane frame) with its mouth ``slide_m`` from the foot."""
    p_c, r_c = cane_pr
    u_c = r_c[:, 1]
    axis = p_c + m.P.BORE_Z * r_c[:, 2] + r_c @ np.asarray(offset, dtype=float)
    turn = m.mk.axis_angle((0.0, 0.0, 1.0), math.radians(cock_deg))
    pivot = axis + 0.5 * slide_m * u_c
    slv_rot = turn @ r_c
    slv_pos = pivot + slv_rot @ np.array([0.0, 0.5 * slide_m - 0.5 * m.P.SLEEVE_L, 0.0])
    poses = {"cane": cane_pr, **vials_in_bays(m, cane_pr), "sleeve": (slv_pos, slv_rot)}
    if tray_pr is not None:
        poses["tray"] = tray_pr
    m.mk.set_state(poses=poses)
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
    assert P.STACK == pytest.approx(0.0132, abs=1e-6)
    assert P.SLEEVE_OD == pytest.approx(0.0145) and P.SLEEVE_BORE == pytest.approx(0.0135)
    assert 0.5 * (P.SLEEVE_BORE - P.STACK) == pytest.approx(0.00015)  # 0.15 mm a side when centred
    assert P.TAB_W > P.SLEEVE_OD
    assert not m.mk.MECH.joints  # no joint anywhere: nothing holds the sleeve on the cane but contact
    cane_p0, cane_r0 = m.mk.built_pose("cane")
    tab_end_w = cane_p0 + cane_r0 @ np.array([0.0, P.CANE_L, 0.0])
    slv_p0, _ = m.mk.built_pose("sleeve")
    assert tab_end_w[1] == pytest.approx(-0.012)
    assert slv_p0[1] + 0.5 * P.SLEEVE_L == pytest.approx(-0.025)


def test_cane_fits_inside_the_sleeve_bore(m):
    """Every solid collider of the cane's trough, and both seated vials, lie inside the 13.5 mm bore around the stack's
    middle line (the inscribed circle of the 16 inside flats), so the sleeve can pass only straight."""
    P = m.P
    worst = 0.0
    for s in m.mk.MECH.bodies["cane"].shapes:
        half = s.half_extents()
        if not s.solid or s.pos[1] - half[1] >= P.TAB_SHOULDER_Y - 1e-6:
            continue
        rot = m.mk.quat_to_matrix(s.quat) if s.quat is not None else np.eye(3)
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                for sz in (-1.0, 1.0):
                    c = np.asarray(s.pos) + rot @ (np.array([sx, sy, sz]) * half)
                    worst = max(worst, math.hypot(float(c[0]), float(c[2]) - P.BORE_Z))
    assert worst <= P.SLEEVE_R_IN - 0.00002
    assert worst >= P.SLEEVE_R_IN - 0.0002  # and the fit is snug, not a loose stand-in
    vial_top = P.VIAL_REST_Z + P.VIAL_R - P.BORE_Z
    assert vial_top == pytest.approx(P.SLEEVE_R_IN - 0.00015)


def test_wall_breach_reading(m):
    """Threaded straight, no cane point lies beyond the sleeve's inside flats; shifted 1.5 mm sideways (through its wall,
    as a collider pushed through would be) the reading shows it."""
    cane_pr = cane_pose_world(m, pos=(0.25, 0.00, m.TOP + 0.095), yaw=0.0)
    threaded_state(m, cane_pr, 0.200)
    assert m.wall_breach_mm() <= 0.01
    threaded_state(m, cane_pr, 0.200, offset=(0.0015, 0.0, 0.0))
    assert m.wall_breach_mm() >= 1.0


def test_reading_where_the_sleeve_is(m):
    cane_pr = cane_pose_world(m, pos=(0.25, 0.00, m.TOP + 0.095), yaw=0.0)
    p_c, _ = cane_pr
    # Pushed against the bent top tab end: not on the cane, 0
    bad_slv = (p_c + np.array([0.0, m.P.CANE_L + 0.5 * m.P.SLEEVE_L + 0.001, m.P.BORE_Z]), EYE)
    m.mk.set_state(poses={"cane": cane_pr, **vials_in_bays(m, cane_pr), "sleeve": bad_slv})
    m.STATE["hands"] = FAR
    assert not m.coaxial_enclosed() and steps(m, 1) == 0.0
    # Threaded straight: 0.25 from 120 mm, 0.5 within 10 mm of the tab with the foot inside
    for s_val, expected in ((0.110, 0.0), (0.135, 0.25), (0.263, 0.5)):
        m.begin(0)
        threaded_state(m, cane_pr, s_val)
        assert m.coaxial_enclosed() and steps(m, 2) == expected
    assert m.gap_to_tab_m() == pytest.approx(0.005) and m.foot_enclosed()
    # Lying beside the cane (3 mm off its middle line) or turned 3 deg: not on it
    m.begin(0)
    threaded_state(m, cane_pr, 0.200, offset=(0.003, 0.0, 0.0))
    assert not m.coaxial_enclosed() and steps(m, 1) == 0.0
    threaded_state(m, cane_pr, 0.200, cock_deg=3.0)
    assert not m.coaxial_enclosed() and steps(m, 1) == 0.0
    threaded_state(m, cane_pr, 0.200, cock_deg=1.0)
    assert m.coaxial_enclosed() and steps(m, 1) == 0.25


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
    threaded_state(m, in_tray_cane, m.P.SLIDE_MAX)
    assert steps(m, m.P.HOLD_STEPS + 5) == 0.0


def test_unsleeved_cane_in_tray_latches_zero(m):
    in_tray_cane = cane_pose_world(m, pos=(m.P.TRAY_CX, m.P.TRAY_CY - 0.135, m.TOP + 0.015), yaw=0.0)
    m.mk.set_state(poses={"cane": in_tray_cane, **vials_in_bays(m, in_tray_cane)})
    m.STATE["hands"] = FAR
    assert steps(m, 1) == 0.0 and m.STATE["zero"] == "unsleeved cane or vial entered the transfer tray"


def test_score_levels_quarter_half_and_full_with_hold_and_clearance(m):
    cane_air = cane_pose_world(m, pos=(0.25, 0.00, m.TOP + 0.095), yaw=0.0)
    threaded_state(m, cane_air, 0.110)
    assert steps(m, 2) == 0.0
    threaded_state(m, cane_air, 0.135)
    assert steps(m, 2) == 0.25
    threaded_state(m, cane_air, 0.263)
    assert m.gap_to_tab_m() == pytest.approx(0.005) and m.foot_enclosed()
    assert steps(m, 2) == 0.5

    in_tray_cane = cane_pose_world(
        m,
        pos=(m.P.TRAY_CX, m.P.TRAY_CY - 0.135, m.TOP + m.P.TRAY_WALL + m.P.SLEEVE_R_OUT),
        yaw=0.0,
    )
    near_hand = {"left": np.array([[m.P.TRAY_CX, m.P.TRAY_CY, m.TOP + 0.040]]), "right": FAR["right"]}
    threaded_state(m, in_tray_cane, m.P.SLIDE_MAX, hands=near_hand)
    assert steps(m, m.P.HOLD_STEPS + 3) == 0.5

    threaded_state(m, in_tray_cane, m.P.SLIDE_MAX, hands=FAR)
    assert steps(m, m.P.HOLD_STEPS - 1) == 0.5
    # Shifting 4 mm (> 3 mm HOLD_DRIFT) restarts the 20-step hold
    shifted = (in_tray_cane[0] + np.array([0.004, 0.0, 0.0]), in_tray_cane[1])
    threaded_state(m, shifted, m.P.SLIDE_MAX, hands=FAR)
    assert steps(m, 1) == 0.5 and m.STATE["hold"] == 1
    # Benign tray shift > 25 mm on the bench does NOT zero the trial (Rules 4 & 7)
    shifted_tray = (np.array(m.scene.tray_start_pos(m.TOP)) + np.array([0.0, 0.045, 0.0]), EYE)
    cane_in_shifted = cane_pose_world(
        m,
        pos=(m.P.TRAY_CX, m.P.TRAY_CY + 0.045 - 0.135, m.TOP + m.P.TRAY_WALL + m.P.SLEEVE_R_OUT),
        yaw=0.0,
    )
    threaded_state(m, cane_in_shifted, m.P.SLIDE_MAX, tray_pr=shifted_tray, hands=FAR)
    assert steps(m, m.P.HOLD_STEPS) == 1.0 and m.STATE["zero"] is None


def test_folder_invariants_and_no_pose_snap_rule():
    import json

    spec = json.loads((HERE / "task.json").read_text())
    assert len(spec["title"]) <= 120 and len(spec["context"]) <= 1000 and len(spec["fidelity"]) <= 500
    task_txt = (HERE / "task.py").read_text()
    assert len(task_txt) <= 15_000
    import ast

    tree = ast.parse((HERE / "task.py").read_text())
    instruction = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign) and getattr(n.targets[0], "id", "") == "INSTRUCTION")
    assert spec["instruction"] == instruction
    assert spec["instruction"].endswith(", then let go and move both hands clear.")
    # Zero write_pose / set_joint / mk.push in task.py, scene.py and demo.py
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
