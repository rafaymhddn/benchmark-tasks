"""Offline unit, boundary, mechanism and documentation-parity tests for ``mech/tasks/pixi_level``.

    scripts/rta check pixi_level
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import re
import sys

import numpy as np
import pytest
from yam_arena import task_file

HERE = Path(__file__).resolve().parent
TASK_PY = HERE / "task.py"
TASK_JSON = HERE / "task.json"
README_MD = HERE / "README.md"
OWN_MODULES = ("mechkit", "kit_extra", "parts", "head", "checks", "demo_kit", "demo", "trials")
DT = 0.05  # s: 20 Hz control step
AWAY_GRIPPERS = {
    "left": (np.array([0.26, 0.32, 1.05]), 1.0),
    "right": (np.array([0.26, -0.32, 1.05]), 1.0),
}


@pytest.fixture
def m():
    saved = {name: sys.modules.pop(name, None) for name in OWN_MODULES}
    module = task_file.load(TASK_PY).module
    reset_episode(module)
    yield module
    module.mk.MECH.clear()
    for name, mod in saved.items():
        sys.modules.pop(name, None)
        if mod is not None:
            sys.modules[name] = mod


def reset_episode(m) -> None:
    m.mk.MECH.new_episode()
    m.mk.mend_all()
    m.start(0)


class Rig:
    """Drives ``task.py`` step-by-step via ``mk.set_state`` without Isaac Sim."""

    def __init__(self, m) -> None:
        self.m = m
        self.t = 0.0
        self.joints = {
            "tilt": m.P.START_TILT_RAD,
            "roll": m.P.START_ROLL_RAD,
            "button": 0.0,
        }
        self.poses: dict = {}
        self.velocities: dict = {}
        self.grippers = dict(AWAY_GRIPPERS)
        self.contacts: dict = {}

    def step_for(self, seconds: float = DT) -> float:
        n = max(1, int(round(seconds / DT)))
        sc = 0.0
        for _ in range(n):
            self.t = round(self.t + DT, 6)
            self.m.mk.set_state(
                self.poses or None,
                self.velocities or None,
                joints=self.joints,
                grippers=self.grippers,
                time=self.t,
                contacts=self.contacts or None,
            )
            self.m.step()
            sc = self.m.score(None)
        return sc

    def set_angles_deg(self, tilt_deg: float, roll_deg: float) -> None:
        """Turn the head as a person does: one step with the button pressed in (head free), then the button as it was."""
        before = self.joints["button"]
        self.joints["button"] = max(before, self.m.P.BUTTON_UNLOCK_M + 0.0003)
        self.force_angles_deg(tilt_deg, roll_deg)
        self.step_for(DT)
        self.joints["button"] = before

    def force_angles_deg(self, tilt_deg: float, roll_deg: float) -> None:
        """Turn the head without the button: with the button out this forces the locked head."""
        self.joints["tilt"] = math.radians(tilt_deg)
        self.joints["roll"] = math.radians(roll_deg)


# --- 1. Scene, figures and geometry invariants ---


def test_parts_figures_have_valid_source_kinds_and_consistent_masses(m) -> None:
    P = m.P
    assert len(P.FIGURES) >= 8
    for fig in P.FIGURES:
        assert fig.kind in P.SOURCE_KINDS
        assert fig.constants.strip() and fig.source.strip() and fig.check.strip()
        assert ("https://" in fig.source) if fig.kind in ("maker", "listing") else True
        assert fig.source.startswith("estimate: ") if fig.kind == "estimate" else True
    assert math.isclose(P.TRI_BODY_MASS + P.BUTTON_MASS + P.C_TILT_MASS, P.TRI_MASS, abs_tol=1e-9)
    assert math.isclose(P.C_TILT_MASS + P.CLAMP_BODY_MASS, P.CLAMP_MASS, abs_tol=1e-9)
    assert math.isclose(P.BUTTON_K * P.BUTTON_TRAVEL, P.BUTTON_N, rel_tol=1e-9)
    assert math.isclose(P.HEAD_PINCH_SPAN, 0.038, abs_tol=1e-6)


def test_welded_phone_and_clamp_shapes_do_not_overlap(m) -> None:
    """Welded bodies collide in mechkit: clamp jaws must leave positive clearance around the 147.6x71.6x7.8 mm phone."""
    P = m.P
    p_pos = np.array(P.phone_built_pos(m.TOP))
    c_pos = np.array(P.clamp_built_pos(m.TOP))
    p_low = p_pos - 0.5 * np.array([P.PHONE_T, P.PHONE_L, P.PHONE_W])
    p_high = p_pos + 0.5 * np.array([P.PHONE_T, P.PHONE_L, P.PHONE_W])
    for idx, s in enumerate(m.mk.MECH.bodies["clamp"].shapes):
        if not s.solid:
            continue
        lo_local, hi_local = s.bounds()
        lo_w, hi_w = c_pos + lo_local, c_pos + hi_local
        sep = np.maximum(lo_w - p_high, p_low - hi_w).max()
        assert sep >= P.WELD_GAP - 1e-6, f"clamp solid shape {idx} overlaps phone (sep={sep * 1000:.2f} mm)"


def test_statics_flops_when_freed_and_stands_at_start(m) -> None:
    st = m.P.static_tipping_summary()
    assert st["flops_when_freed_at_start"] is True
    assert abs(st["start_gravity_torque_nm"]["roll"]) > 2.0 * m.P.FREE_NM
    assert abs(st["start_gravity_torque_nm"]["roll"]) < 0.1 * m.P.LOCK_NM
    assert st["com_offset_start_mm"] < 0.5 * st["support_inradius_mm"]
    assert st["one_sided_button_moment_nm"] > 4.0 * st["tripod_tip_torque_vertex_nm"]


# --- 2. Idle, full score (1.0), partial score (0.5) and boundary thresholds ---


def test_idle_start_scores_zero(m) -> None:
    rig = Rig(m)
    assert rig.step_for(2.0) == 0.0
    t_deg, r_deg = m.phone_tilt_roll_deg()
    assert abs(t_deg - m.P.START_TILT_DEG) < 0.5
    assert abs(r_deg - m.P.START_ROLL_DEG) < 0.5


@pytest.mark.parametrize(
    ("tilt_deg", "roll_deg", "expected"),
    [
        (0.0, 0.0, 1.0),
        (2.9, -2.9, 1.0),
        (-2.95, 0.0, 1.0),
        (3.1, 0.0, 0.5),
        (0.0, -3.1, 0.5),
        (7.9, -7.8, 0.5),
        (8.1, 0.0, 0.0),
        (0.0, -8.2, 0.0),
        (10.0, -20.0, 0.0),
    ],
)
def test_angle_level_boundaries(m, tilt_deg: float, roll_deg: float, expected: float) -> None:
    rig = Rig(m)
    rig.set_angles_deg(tilt_deg, roll_deg)
    assert rig.step_for(1.1) == expected


def test_rest_hold_duration_and_velocity_thresholds(m) -> None:
    rig = Rig(m)
    rig.set_angles_deg(0.0, 0.0)
    # 0.95 s < 1.0 s required rest window -> still 0.0
    assert rig.step_for(0.95) == 0.0
    # One more 0.05 s step reaches 1.00 s -> 1.0
    assert rig.step_for(0.05) == 1.0

    # Moving phone above REST_LIN_MAX (0.02 m/s) resets rest window and scores 0.0
    rig.velocities = {"phone": (np.array([0.025, 0.0, 0.0]), np.zeros(3))}
    assert rig.step_for(0.05) == 0.0
    rig.velocities = {"phone": (np.array([0.018, 0.0, 0.0]), np.zeros(3))}
    assert rig.step_for(0.95) == 0.0
    assert rig.step_for(0.05) == 1.0


def test_button_must_be_released_out_to_score(m) -> None:
    rig = Rig(m)
    rig.set_angles_deg(0.0, 0.0)
    # Button still pressed at 2.5 mm (head unlocked) -> 0.0
    rig.joints["button"] = 0.0025
    assert rig.step_for(1.2) == 0.0
    # Button half-in at 0.6 mm (> 0.5 mm BUTTON_OUT_MAX) -> 0.0
    rig.joints["button"] = 0.0006
    assert rig.step_for(1.2) == 0.0
    # Button out at 0.4 mm (<= 0.5 mm) -> 1.0 after 1.0 s
    rig.joints["button"] = 0.0004
    assert rig.step_for(1.05) == 1.0


def test_gripper_clearance_30mm_boundary_each_hand(m) -> None:
    rig = Rig(m)
    rig.set_angles_deg(0.0, 0.0)
    assert rig.step_for(1.05) == 1.0
    pp, _ = m.mk.pose("phone")
    # Over the phone's left wing (y = +50 mm, clear of the 30 mm centre clamp jaw)
    top_wing = pp + np.array([0.0, 0.050, 0.5 * m.P.PHONE_W])

    for side in ("left", "right"):
        # 29 mm above phone top wing -> not clear -> 0.0
        rig.grippers = {**AWAY_GRIPPERS, side: (top_wing + np.array([0.0, 0.0, 0.029]), 1.0)}
        assert rig.step_for(1.1) == 0.0
        # 31 mm above phone top wing -> clear -> 1.0
        rig.grippers = {**AWAY_GRIPPERS, side: (top_wing + np.array([0.0, 0.0, 0.031]), 1.0)}
        assert rig.step_for(1.05) == 1.0


# --- 3. Whole-trial zero rules (foot lift, foot slide, phone touches bench) ---


def test_tripod_foot_lift_10mm_boundary_latches_zero(m) -> None:
    rig = Rig(m)
    rig.set_angles_deg(0.0, 0.0)
    tp0, tr0 = m.mk.built_pose("tripod")

    # 9.5 mm lift -> allowed
    rig.poses = {"tripod": (tp0 + np.array([0.0, 0.0, 0.009 - m.P.OVER]), tr0)}
    assert rig.step_for(1.05) == 1.0

    # 10.5 mm lift for a single step -> latches zero_fault for the entire trial even after put back down
    rig.poses = {"tripod": (tp0 + np.array([0.0, 0.0, 0.0105 - m.P.OVER]), tr0)}
    assert rig.step_for(DT) == 0.0
    assert m.STATE["zero_fault"] is not None
    rig.poses = {}
    assert rig.step_for(1.5) == 0.0


def test_tripod_foot_slide_20mm_boundary_latches_zero(m) -> None:
    rig = Rig(m)
    rig.set_angles_deg(0.0, 0.0)
    tp0, tr0 = m.mk.built_pose("tripod")

    # 19.5 mm horizontal shift -> allowed
    rig.poses = {"tripod": (tp0 + np.array([0.0195, 0.0, 0.0]), tr0)}
    assert rig.step_for(1.05) == 1.0

    # 20.5 mm horizontal shift -> latches zero_fault permanently
    rig.poses = {"tripod": (tp0 + np.array([0.0205, 0.0, 0.0]), tr0)}
    assert rig.step_for(DT) == 0.0
    assert "slid" in str(m.STATE["zero_fault"])
    rig.poses = {}
    assert rig.step_for(1.5) == 0.0


def test_phone_touching_bench_latches_zero(m) -> None:
    rig = Rig(m)
    rig.set_angles_deg(0.0, 0.0)
    contact = m.mk.Contact(
        other="scene",
        shape=0,
        other_shape=-1,
        point=np.array([0.26, 0.0, m.TOP]),
        normal=np.array([0.0, 0.0, 1.0]),
        separation=-0.0001,
        force=2.0,
    )
    rig.contacts = {"phone": [contact]}
    assert rig.step_for(DT) == 0.0
    assert m.STATE["phone_touched_bench"] is True
    rig.contacts = {}
    assert rig.step_for(1.5) == 0.0


def test_locked_head_slip_over_0_2_deg_latches_zero(m) -> None:
    """Forcing the locked head drags its rest angle: slip summed over the trial > 0.2 deg zeroes it, also when it comes
    in steps under 0.2 deg; the elastic give inside the lock does not count."""
    rig = Rig(m)
    rig.set_angles_deg(0.0, 0.0)
    assert rig.step_for(1.05) == 1.0
    band_deg = math.degrees(m.P.LOCK_NM / m.P.K_HEAD + m.P.LOCK_SOLVER_SLACK_RAD)  # 2.42 deg

    rig.force_angles_deg(band_deg - 0.05, 0.0)  # elastic give, then back: no slip
    rig.step_for(DT)
    rig.force_angles_deg(0.0, 0.0)
    assert rig.step_for(1.05) == 1.0
    assert m.H.locked_slip_deg() == pytest.approx(0.0, abs=1e-9)

    for k in range(1, 5):  # 0.05 deg of slip per step: 0.20 deg in all is still allowed
        rig.force_angles_deg(-(band_deg + 0.05 * k), 0.0)
        assert rig.step_for(DT) == 1.0
    assert m.H.locked_slip_deg() == pytest.approx(0.20, abs=1e-6)
    assert m.STATE["zero_fault"] is None
    rig.force_angles_deg(-(band_deg + 0.25), 0.0)  # 0.25 deg in all
    assert rig.step_for(DT) == 0.0
    assert "slipped" in str(m.STATE["zero_fault"])
    rig.set_angles_deg(0.0, 0.0)
    assert rig.step_for(1.5) == 0.0


def test_forcing_the_locked_head_level_scores_zero(m) -> None:
    """The shortcut: brace the tripod and force the locked phone level, button never pressed -> 0.0, although the end
    state (level, button out, at rest, hands clear) is the one that scores 1.0 with the button."""
    rig = Rig(m)
    for frac in np.linspace(0.1, 1.0, 10):
        rig.force_angles_deg(m.P.START_TILT_DEG * (1.0 - frac), m.P.START_ROLL_DEG * (1.0 - frac))
        rig.step_for(DT)
    assert m.H.STATE["max_button_mm"] == 0.0
    assert m.H.locked_slip_deg() > 10.0
    assert rig.step_for(1.5) == 0.0
    assert "slipped" in str(m.STATE["zero_fault"])
    assert m.phone_max_lean_deg() <= m.P.LEVEL_DEG and m.button_out() and m.grippers_clear() and m.bodies_at_rest()


# --- 4. Mechanism & event-order laws (Check 11 & Check 51) ---


def test_button_unlock_requires_pressed_fraction_travel(m) -> None:
    rig = Rig(m)
    # 79% of 3.0 mm = 2.37 mm -> still locked (1.50 N m cap)
    rig.joints["button"] = 0.79 * m.P.BUTTON_TRAVEL
    rig.step_for(0.15)
    assert m.H.is_unlocked() is False
    assert m.H.active_torque_cap_nm() == pytest.approx(m.P.LOCK_NM)

    # 81% of 3.0 mm = 2.43 mm -> unlocked (0.02 N m cap)
    rig.joints["button"] = 0.81 * m.P.BUTTON_TRAVEL
    rig.step_for(0.15)
    assert m.H.is_unlocked() is True
    assert m.H.active_torque_cap_nm() == pytest.approx(m.P.FREE_NM)

    # Releasing button while level locks rest angle at (0, 0)
    rig.set_angles_deg(0.2, -0.3)
    rig.step_for(0.25)
    rig.joints["button"] = 0.0
    rig.step_for(0.25)
    assert m.H.is_unlocked() is False
    assert math.degrees(m.H.STATE["rest"]["tilt"]) == pytest.approx(0.2, abs=0.05)
    assert math.degrees(m.H.STATE["rest"]["roll"]) == pytest.approx(-0.3, abs=0.05)


def test_constrained_event_order_recorded_cleanly(m) -> None:
    rig = Rig(m)
    # 1. Hold phone first
    pp, _ = m.mk.built_pose("phone")
    rig.grippers = {**AWAY_GRIPPERS, "left": (pp + np.array(m.P.PHONE_GRASP_LOCAL), 0.02)}
    rig.contacts = {
        "phone": [m.mk.Contact("left robot", 0, -1, pp, np.array([1.0, 0.0, 0.0]), -0.0002, -15.0)]
    }
    rig.step_for(0.20)
    # 2. Press button
    rig.joints["button"] = 0.0028
    rig.step_for(0.20)
    # 3. Level phone while button held
    rig.set_angles_deg(0.0, 0.0)
    rig.step_for(0.30)
    # 4. Release button before releasing phone
    rig.joints["button"] = 0.0
    rig.step_for(0.30)
    # 5. Release phone and park hands clear for 1.05 s
    rig.contacts = {}
    rig.grippers = dict(AWAY_GRIPPERS)
    assert rig.step_for(1.05) == 1.0
    assert m.EVENTS.before("phone held", "button pressed")
    assert m.EVENTS.before("button pressed", "phone levelled")
    assert m.EVENTS.before("phone levelled", "button released")


# --- 5. Checklist 22, 40, 47 & 52 parity and byte-size checks ---


def test_task_json_and_readme_limits_and_numeric_parity(m) -> None:
    spec = json.loads(TASK_JSON.read_text())
    readme = README_MD.read_text()
    task_bytes = len(TASK_PY.read_bytes())
    readme_bytes = len(README_MD.read_bytes())

    # Checklist 52: README <= 10,000 B, task.py <= 15,000 B
    assert readme_bytes <= 10_000, f"README.md is {readme_bytes} bytes (> 10,000)"
    assert task_bytes <= 15_000, f"task.py is {task_bytes} bytes (> 15,000)"

    # Checklist 22 & 23
    assert len(spec["title"]) <= 120
    assert len(spec["context"]) <= 1_000
    assert len(spec["fidelity"]) <= 500
    assert spec["category"] == "Societal relatability"
    assert spec["instruction"] == m.INSTRUCTION

    # Checklist 3: identical scoring numbers across task.py, task.json and README.md
    for token in ("1,200 steps", "60 s", "3.0 deg", "8.0 deg", "10 mm", "20 mm", "30 mm", "1.0 s", "0.2 deg"):
        assert token in spec["scoring"] or token in spec["run_limit"], f"{token!r} missing in task.json"
        assert token in readme, f"{token!r} missing in README.md"

    # Checklist 47: zero board PR numbers (#1234) or TK IDs in task.json, README.md or task.py
    for path in (TASK_JSON, README_MD, TASK_PY, HERE / "parts.py", HERE / "head.py"):
        txt = path.read_text()
        assert re.search(r"(?<![\w/])(#\d{3,4}|TK-[A-Z0-9]{6})\b", txt) is None, f"board ID leaked in {path.name}"

    # Source pass: the source is written, not hedged (no unmeasured 'measured', no 'assumed' kind, no wait-for-bench line)
    for path in (TASK_JSON, README_MD, HERE / "parts.py", HERE / "head.py", TASK_PY):
        hedge = re.search(r"assumed|measured|pre-bench|bench reading|until the owner", path.read_text(), re.IGNORECASE)
        assert hedge is None, f"{path.name}: {hedge.group(0) if hedge else ''!r}"

    # Every PNG in folder is named in README.md
    for png in HERE.glob("*.png"):
        assert png.name in readme, f"{png.name} not named in README.md"
