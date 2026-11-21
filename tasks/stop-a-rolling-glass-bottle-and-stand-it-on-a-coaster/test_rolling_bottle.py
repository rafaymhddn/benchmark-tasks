"""Pure rule, geometry, kinematics, and Check-50/51/52 unit tests for ``mech/tasks/rolling_bottle`` (no simulator).

    scripts/rta check mech/tasks/rolling_bottle
"""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest
from yam_arena import task_file

HERE = Path(__file__).resolve().parent
TASK_PATH = HERE / "task.py"
OWN_MODULES = ("mechkit", "parts", "roll_mech", "kit_extra")


def _axis_angle_rot(axis, deg: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    rad = math.radians(deg)
    kx, ky, kz = a
    k = np.array([[0.0, -kz, ky], [kz, 0.0, -kx], [-ky, kx, 0.0]], dtype=float)
    return np.eye(3) + math.sin(rad) * k + (1.0 - math.cos(rad)) * (k @ k)


@pytest.fixture
def m():
    saved = {name: sys.modules.pop(name, None) for name in OWN_MODULES}
    mod = task_file.load(TASK_PATH).module
    _fresh(mod)
    yield mod
    mod.mk.MECH.clear()
    for name, prev in saved.items():
        sys.modules.pop(name, None)
        if prev is not None:
            sys.modules[name] = prev


def _fresh(mod) -> None:
    mod.mk.MECH.new_episode()
    mod.mk.mend_all()
    mod.start()


class BenchTrial:
    """Deterministic state stepper for ``rolling_bottle/task.py`` without the simulator."""

    def __init__(self, mod) -> None:
        self.m = mod
        self.P = mod.P
        self.t = 0.0
        p0, r0 = self.P.bottle_start_pose(clear_m=0.0)
        self.bottle_pos = p0.copy()
        self.bottle_rot = r0.copy()
        self.bottle_lin = np.zeros(3, dtype=float)
        self.bottle_ang = np.zeros(3, dtype=float)
        self.coaster_pos = np.array([self.P.COASTER_X, self.P.COASTER_Y, self.P.WORKTOP_TOP_Z], dtype=float)
        self.coaster_rot = np.eye(3, dtype=float)
        self.coaster_lin = np.zeros(3, dtype=float)
        self.coaster_ang = np.zeros(3, dtype=float)
        self.grippers = {
            "left": (np.array([0.05, 0.35, self.P.TOP + 0.35]), 1.0),
            "right": (np.array([0.05, -0.35, self.P.TOP + 0.35]), 1.0),
        }
        self.contacts: dict = {
            ("bottle", "worktop"): self.P.BOTTLE_MASS * self.P.GRAVITY,
            ("bottle", "drainboard"): 0.0,
            ("bottle", "coaster"): 0.0,
            "bottle": [],
        }

    def place_lying_on_worktop(self, x: float = 0.375, y: float = -0.075) -> None:
        self.bottle_pos = np.array([x, y, self.P.WORKTOP_TOP_Z + self.P.BOTTLE_R], dtype=float)
        self.bottle_rot = self.P.BOTTLE_LYING_ROT.copy()
        self.bottle_lin[:] = 0.0
        self.bottle_ang[:] = 0.0
        self.contacts[("bottle", "worktop")] = self.P.BOTTLE_MASS * self.P.GRAVITY
        self.contacts[("bottle", "coaster")] = 0.0

    def place_upright_in_coaster(
        self,
        *,
        cap_up: bool = True,
        tilt_deg: float = 0.0,
        base_offset_xy: tuple[float, float] = (0.0, 0.0),
        coaster_shift_xy: tuple[float, float] = (0.0, 0.0),
    ) -> None:
        self.coaster_pos = np.array(
            [self.P.COASTER_X + coaster_shift_xy[0], self.P.COASTER_Y + coaster_shift_xy[1], self.P.WORKTOP_TOP_Z],
            dtype=float,
        )
        base_rot = np.eye(3, dtype=float) if cap_up else _axis_angle_rot((1.0, 0.0, 0.0), 180.0)
        self.bottle_rot = _axis_angle_rot((0.0, 1.0, 0.0), tilt_deg) @ base_rot
        floor_z = self.P.WORKTOP_TOP_Z + self.P.COASTER_FLOOR_H
        cx = self.coaster_pos[0] + base_offset_xy[0]
        cy = self.coaster_pos[1] + base_offset_xy[1]
        if cap_up:
            self.bottle_pos = np.array([cx, cy, floor_z], dtype=float) + 0.5 * self.P.BOTTLE_H * self.bottle_rot[:, 2]
        else:
            self.bottle_pos = np.array([cx, cy, floor_z], dtype=float) - 0.5 * self.P.BOTTLE_H * self.bottle_rot[:, 2]
        self.bottle_lin[:] = 0.0
        self.bottle_ang[:] = 0.0
        self.contacts[("bottle", "worktop")] = 0.0
        self.contacts[("bottle", "coaster")] = self.P.BOTTLE_MASS * self.P.GRAVITY

    def step_n(self, steps: int = 1) -> float:
        s = 0.0
        for _ in range(steps):
            self.t = round(self.t + self.P.DT, 6)
            self.m.mk.set_state(
                poses={
                    "bottle": (self.bottle_pos.copy(), self.bottle_rot.copy()),
                    "coaster": (self.coaster_pos.copy(), self.coaster_rot.copy()),
                },
                velocities={
                    "bottle": (self.bottle_lin.copy(), self.bottle_ang.copy()),
                    "coaster": (self.coaster_lin.copy(), self.coaster_ang.copy()),
                },
                grippers=self.grippers,
                time=self.t,
                contacts=self.contacts,
            )
            self.m.step()
            s = self.m.score(None)
        return s


def test_checklist_52_char_limits_and_no_review_history():
    readme_text = (HERE / "README.md").read_text()
    task_py_text = (HERE / "task.py").read_text()
    spec_json = json.loads((HERE / "task.json").read_text())

    assert len(readme_text) <= 10_000, f"README.md is {len(readme_text)} chars (> 10,000)"
    assert len(task_py_text) <= 15_000, f"task.py is {len(task_py_text)} chars (> 15,000)"
    assert len(spec_json["title"]) <= 120
    assert len(spec_json["context"]) <= 1_000
    assert len(spec_json["fidelity"]) <= 445, f"fidelity is {len(spec_json['fidelity'])} chars (> 445 target)"
    low = readme_text.lower()
    assert "earlier reviews" not in low and "thank you" not in low


def test_parts_single_source_presets_and_kinematics(m):
    P = m.P
    assert set(P.BOTTLE_PRESETS) == {"voss_375", "true_fruits_250"}
    valid_kinds = {"maker's page", "listing", "estimate", "protocol"}
    assert len(P.KEY_FIGURES) >= 8
    for row in P.KEY_FIGURES:
        assert row.kind in valid_kinds

    preds = P.check_all_presets()
    voss = preds["voss_375"]
    assert float(voss["v_foot_mps"]) == pytest.approx(P.V_FOOT_MPS, abs=0.015)
    assert float(voss["dt_150_from_foot_s"]) == pytest.approx(P.T_150_S, abs=0.025)
    assert float(voss["dt_300_from_foot_s"]) == pytest.approx(P.T_300_S, abs=0.035)
    assert float(voss["idle_coaster_clearance_mm"]) == pytest.approx(5.5, abs=0.2)

    tf = preds["true_fruits_250"]
    assert 0.26 <= float(tf["v_foot_mps"]) <= 0.31
    assert float(tf["idle_coaster_clearance_mm"]) == pytest.approx(39.0, abs=0.2)


def test_preset_env_switch_updates_bottle_dimensions(monkeypatch):
    monkeypatch.setenv("ROLLING_BOTTLE_PRESET", "true_fruits_250")
    monkeypatch.syspath_prepend(str(HERE))
    spec = importlib.util.spec_from_file_location("_rolling_bottle_parts_tf", HERE / "parts.py")
    assert spec is not None and spec.loader is not None
    p_tf = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, p_tf)
    spec.loader.exec_module(p_tf)
    assert p_tf.BOTTLE_PRESET == "true_fruits_250"
    assert p_tf.BOTTLE_D == pytest.approx(0.056)
    assert p_tf.BOTTLE_H == pytest.approx(0.162)
    assert p_tf.BOTTLE_MASS == pytest.approx(0.440)


def test_zero_welds_cap_proud_and_zero_step_drainboard_seam(m):
    P = m.P
    assert len(m.mk.MECH.joints) == 0
    for fixed_name in ("worktop", "drainboard", "catch_mat", "coaster"):
        spec = m.mk.MECH.bodies[fixed_name]
        assert spec.kinematic and not spec.free and spec.mass is None

    # Cap visual top extends +0.4 mm past +0.5 * BOTTLE_H so overhead camera has zero top-disc z-fighting
    shapes = P.bottle_shapes()
    cap_top_z = shapes[1].pos[2] + 0.5 * shapes[1].dims[1]
    assert cap_top_z - 0.5 * P.BOTTLE_H == pytest.approx(0.0004, abs=1e-7)

    d_pos, d_rot = P.drainboard_pose()
    top_foot_world = d_pos + d_rot @ np.array([0.0, -0.5 * P.DRAIN_L, 0.5 * P.DRAIN_T])
    bot_foot_world = d_pos + d_rot @ np.array([0.0, -0.5 * P.DRAIN_L, -0.5 * P.DRAIN_T])
    assert top_foot_world[1] == pytest.approx(P.DRAIN_FOOT_Y, abs=1e-9)
    assert top_foot_world[2] == pytest.approx(P.WORKTOP_TOP_Z, abs=1e-9)
    assert bot_foot_world[2] < P.WORKTOP_TOP_Z


def test_check51_contact_gated_rolling_and_bilateral_cork_torques(m):
    P = m.P
    trial = BenchTrial(m)
    trial.place_lying_on_worktop()

    omega_x = 0.25 / P.BOTTLE_R
    trial.bottle_lin = np.array([0.0, -0.25, 0.0])
    trial.bottle_ang = np.array([omega_x, 0.0, 0.0])
    trial.contacts[("bottle", "worktop")] = P.BOTTLE_MASS * P.GRAVITY
    trial.step_n(1)
    assert m.STATE["last_tau_roll"][0] == pytest.approx(-P.tau_roll(), rel=1e-5)

    trial.bottle_ang = np.array([-omega_x, 0.0, 0.0])
    trial.step_n(1)
    assert m.STATE["last_tau_roll"][0] == pytest.approx(+P.tau_roll(), rel=1e-5)

    for f_low in (0.0, 0.49):
        trial.contacts[("bottle", "worktop")] = f_low
        trial.contacts[("bottle", "drainboard")] = 0.0
        trial.contacts[("bottle", "coaster")] = f_low
        trial.step_n(1)
        assert np.allclose(m.STATE["last_tau_roll"], 0.0)
    assert m.STATE["max_airborne_tau_nm"] == 0.0

    # Upright on coaster -> zero external push torque (PhysX contact + 0.05/s angular damping settle naturally)
    trial.place_upright_in_coaster(cap_up=True, tilt_deg=0.0)
    trial.bottle_ang = np.array([0.0, 0.0, 1.2])
    trial.contacts[("bottle", "coaster")] = P.BOTTLE_MASS * P.GRAVITY
    trial.step_n(1)
    assert np.allclose(m.STATE["last_tau_roll"], 0.0)


@pytest.mark.parametrize(
    "bad_pos",
    [
        (0.375, -0.231, None),
        (0.144, 0.000, None),
        (0.506, 0.000, None),
        (0.375, 0.291, None),
        (0.375, -0.100, 0.015),
    ],
)
def test_off_edge_permanent_zero_latch_cannot_recover(m, bad_pos):
    P = m.P
    trial = BenchTrial(m)
    trial.step_n(1)
    bx, by, low_above_top = bad_pos
    bz = (P.TOP + low_above_top + P.BOTTLE_R) if low_above_top is not None else (P.WORKTOP_TOP_Z + P.BOTTLE_R)
    trial.bottle_pos = np.array([bx, by, bz], dtype=float)
    assert trial.step_n(1) == 0.0
    assert m.STATE["off_edge"] is not None

    trial.place_upright_in_coaster(cap_up=True)
    assert trial.step_n(30) == 0.0
    assert not m.success(None)


def test_support_contact_force_required_for_half_and_full_scores(m):
    # Hovering within z-band with < 1.0 N contact force scores 0.0 (not resting on counter/coaster)
    trial = BenchTrial(m)
    trial.place_lying_on_worktop()
    trial.contacts[("bottle", "worktop")] = 0.90
    assert trial.step_n(25) == 0.0
    trial.contacts[("bottle", "worktop")] = 1.05
    assert trial.step_n(19) == 0.0
    assert trial.step_n(1) == 0.5

    _fresh(m)
    trial = BenchTrial(m)
    trial.place_upright_in_coaster(cap_up=True)
    # Supported by worktop instead of coaster (< 1.0 N coaster contact) -> 0.5, not 1.0
    trial.contacts[("bottle", "coaster")] = 0.90
    trial.contacts[("bottle", "worktop")] = 5.20
    assert trial.step_n(20) == 0.5
    trial.contacts[("bottle", "coaster")] = 1.05
    trial.contacts[("bottle", "worktop")] = 0.0
    assert trial.step_n(20) == 1.0


def test_hand_touching_or_within_30mm_scores_zero(m):
    P = m.P
    trial = BenchTrial(m)
    trial.place_upright_in_coaster(cap_up=True)

    trial.grippers["right"] = (np.array([P.COASTER_X - P.BOTTLE_R - 0.028, P.COASTER_Y, P.WORKTOP_TOP_Z + 0.10]), 1.0)
    assert trial.step_n(25) == 0.0

    trial.grippers["right"] = m.AWAY_GRIPPERS["right"]
    trial.grippers["left"] = (np.array([P.COASTER_X + P.COASTER_OUT_R + 0.020, P.COASTER_Y, P.WORKTOP_TOP_Z + P.COASTER_TOTAL_H + 0.015]), 1.0)
    assert trial.step_n(25) == 0.0

    trial.grippers["left"] = m.AWAY_GRIPPERS["left"]
    trial.contacts["bottle"] = [m.mk.Contact("right robot", 0, -1, trial.bottle_pos.copy(), np.array([1.0, 0.0, 0.0]), -0.0002, 1.5)]
    assert trial.step_n(25) == 0.0

    trial.contacts["bottle"] = []
    trial.grippers["right"] = (np.array([P.COASTER_X - P.BOTTLE_R - 0.032, P.COASTER_Y, P.WORKTOP_TOP_Z + 0.10]), 1.0)
    assert trial.step_n(19) == 0.0
    assert trial.step_n(1) == 1.0
    assert m.success(None)


def test_cap_down_upright_in_coaster_scores_half_not_full(m):
    trial = BenchTrial(m)
    trial.place_upright_in_coaster(cap_up=False, tilt_deg=0.0)
    assert trial.step_n(19) == 0.0
    assert trial.step_n(5) == 0.5
    assert not m.success(None)


def test_upright_thresholds_tilt_base_offset_coaster_shift_and_speed(m):
    cases: tuple[tuple[dict, float], ...] = (
        ({"tilt_deg": 4.9}, 1.0),
        ({"tilt_deg": 5.1}, 0.5),
        ({"base_offset_xy": (0.0215, 0.0)}, 1.0),
        ({"base_offset_xy": (0.0225, 0.0)}, 0.5),
        ({"coaster_shift_xy": (0.0145, 0.0)}, 1.0),
        ({"coaster_shift_xy": (0.0155, 0.0)}, 0.5),
    )
    for kw, expect in cases:
        _fresh(m)
        trial = BenchTrial(m)
        trial.place_upright_in_coaster(cap_up=True, **kw)
        assert trial.step_n(20) == expect

    _fresh(m)
    trial = BenchTrial(m)
    trial.place_upright_in_coaster(cap_up=True)
    trial.bottle_lin = np.array([0.0105, 0.0, 0.0])
    assert trial.step_n(25) == 0.0
    trial.bottle_lin = np.array([0.0, 0.0, 0.0])
    trial.bottle_ang = np.array([0.0, 0.0, 0.16])
    assert trial.step_n(25) == 0.0
    trial.bottle_lin = np.array([0.0095, 0.0, 0.0])
    trial.bottle_ang = np.array([0.0, 0.0, 0.14])
    assert trial.step_n(20) == 1.0
