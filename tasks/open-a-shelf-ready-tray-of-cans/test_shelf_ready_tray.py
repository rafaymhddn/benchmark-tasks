"""Pure rule and physics-law tests for mech/tasks/shelf_ready_tray (no simulator needed).

Covers:
- Single-source parts.py constants, dieline and retail_srp presets, and tray_template.svg.
- Check 51 structural invariants: strict serial kinematic tree (tray_base -> c_lift -> c_pitch -> c_twist -> front_panel),
  zero closed loops, zero one-sided mk.push calls, exact mass conservation.
- Non-tear guards: idle = 0, pinch/touch/proximity = 0, straight lift to the slide's stop = 0 (tear 459 N out of reach),
  flat pull: folds the panel tearing only the side seams, never detaches.
- Whole-trial zero latches (both sides of every threshold): tray tilt 10.0 deg, tray lift 15.0 mm, tray corner shift 25.0 mm,
  can tip 12.0 deg (whole-trial) vs 10.0 deg (end-state), can leaving tray_base.
- Irreversible progressive tear law: corner skew initiation (6.0 deg), start = run force PEEL_START_N = PEEL_RUN_N
  (2.75 N), mid-peel release at 30.0 deg staying >= 28.0 deg with ties_torn preserved and untorn ties
  still resisting further peel, and full detachment at PEEL_COMPLETE_DEG (52.0 deg).
- Partial (0.5) and full (1.0) scoring boundaries: panel flatness (15.0 deg, 20.0 mm z), 2D footprint clearance (30.0 mm,
  both face-up and face-down), contact force, gripper clearance (30.0 mm), linear/angular rest speeds, and 1.0 s (20-step) hold.
"""

from __future__ import annotations

import ast
import importlib
import json
import math
import os
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from yam_arena import task_file

HERE = Path(__file__).resolve().parent
TASK_PATH = HERE / "task.py"
OWN_MODULES = ("mechkit", "demo_kit", "kit_extra", "parts", "peel")
DT = 0.05  # s (20 Hz control period)
AWAY_GRIPPERS = {
    "left": (np.array([0.10, 0.35, 1.05], dtype=float), 1.0),
    "right": (np.array([0.10, -0.35, 1.05], dtype=float), 1.0),
}


def axis_angle(axis, deg: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    rad = math.radians(deg)
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(rad) * k + (1.0 - math.cos(rad)) * (k @ k)


@pytest.fixture
def m():
    saved = {name: sys.modules.pop(name, None) for name in OWN_MODULES}
    mod = task_file.load(TASK_PATH).module
    fresh(mod)
    yield mod
    mod.mk.MECH.clear()
    for name, prev in saved.items():
        sys.modules.pop(name, None)
        if prev is not None:
            sys.modules[name] = prev


def fresh(m) -> None:
    m.mk.MECH.new_episode()
    m.mk.mend_all()
    m.start(0)


class Bench:
    """Direct state driver for testing step hooks and score() at 20 Hz without Isaac Sim."""

    def __init__(self, m) -> None:
        self.m = m
        self.t = 0.0
        self.poses: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self.velocities: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self.joints: dict[str, float] = {"perf_lift": 0.0, "peel_pitch": 0.0, "peel_skew": 0.0}
        self.grippers = dict(AWAY_GRIPPERS)
        self.contacts: dict = {}

    def step(self, n_steps: int = 1) -> float:
        sc = 0.0
        for _ in range(n_steps):
            self.t = round(self.t + DT, 6)
            self.m.mk.set_state(
                poses=self.poses or None,
                velocities=self.velocities or None,
                joints=self.joints,
                grippers=self.grippers,
                time=self.t,
                contacts=self.contacts or None,
            )
            self.m.step()
            sc = self.m.score(None)
        return sc

    def place_panel_flat(
        self,
        xy: tuple[float, float] | None = None,
        face_up: bool = True,
        tilt_deg: float = 0.0,
        z_above_top: float = 0.004,
        yaw_deg: float = 0.0,
    ) -> None:
        """Place front_panel flat on the bench (yaw_deg=0 has 218 mm width along y inside DROP_ZONE_XY)."""
        P = self.m.P
        tx, ty = xy if xy is not None else P.DROP_ZONE_XY
        # Forward 90-deg fold about (0, -1, 0): x_panel -> +z (inner side up), y_panel -> +y, z_panel -> -x
        if face_up:
            base_r = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]], dtype=float)
        else:
            base_r = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]], dtype=float)
        r = axis_angle((1.0, 0.0, 0.0), tilt_deg) @ axis_angle((0.0, 0.0, 1.0), yaw_deg) @ base_r
        self.poses["front_panel"] = (np.array([tx, ty, self.m.TOP + z_above_top], dtype=float), r)

    def complete_clean_peel(self) -> None:
        """Drive a clean progressive corner peel from 0 to 54 deg so panel_detach cuts naturally."""
        self.joints["peel_skew"] = math.radians(8.5)
        for deg in (4.0, 8.5, 16.0, 25.0, 35.0, 45.0, 53.5):
            self.joints["peel_pitch"] = math.radians(deg)
            self.step(1)
        assert self.m.L.detached()
        assert self.m.L.ties_torn() == self.m.P.N_TIES


# --- 1. Structural, preset, template, and Check-51 invariants ---


def test_scene_masses_serial_tree_and_zero_mk_push(m):
    P = m.P
    assert m.SCENE == "station" and m.SCORE_OVER == "end" and m.RUN_LIMIT_STEPS == 1600
    assert m.SUCCESS_HOLD_S == pytest.approx(3.5)
    assert "Tear the perforated front panel completely off the tray of cans" in m.INSTRUCTION

    bodies = m.mk.MECH.bodies
    total_tray_mass = bodies["tray_base"].mass + sum(bodies[c].mass for c in P.CARRIER_NAMES)
    assert total_tray_mass == pytest.approx(P.TRAY_MASS, abs=1e-6)
    assert bodies["front_panel"].mass == pytest.approx(P.PANEL_MASS, abs=1e-6)
    for cname in m.CAN_NAMES:
        assert bodies[cname].mass == pytest.approx(P.CAN_MASS, abs=1e-6)

    # Strict serial kinematic tree: tray_base -> c_lift -> c_pitch -> c_twist -> front_panel (zero closed loops)
    joints = m.mk.MECH.joints
    assert [(j.name, j.kind, j.parent, j.child) for j in joints.values()] == [
        ("perf_lift", "slide", "tray_base", "c_lift"),
        ("peel_pitch", "hinge", "c_lift", "c_pitch"),
        ("peel_skew", "hinge", "c_pitch", "c_twist"),
        ("panel_detach", "weld", "c_twist", "front_panel"),
    ]
    children = [j.child for j in joints.values()]
    assert len(children) == len(set(children)), "every jointed child has at most one parent (strict tree)"

    # Zero one-sided mk.push() calls in task.py, peel.py, or parts.py (Check 51)
    for src_name in ("task.py", "peel.py", "parts.py"):
        tree = ast.parse((HERE / src_name).read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.Call):
                fn = node.func
                name = fn.attr if isinstance(fn, ast.Attribute) else (fn.id if isinstance(fn, ast.Name) else "")
                assert name != "push", f"forbidden one-sided push() call in {src_name}"


def test_presets_and_svg_template_exist():
    svg = HERE / "tray_template.svg"
    assert svg.is_file()
    root = ET.fromstring(svg.read_text())
    ns = {"s": "http://www.w3.org/2000/svg"}
    by_id = {el.attrib["id"]: el.attrib for el in root.findall(".//*[@id]", ns)}
    assert (float(by_id["tray_floor"]["width"]), float(by_id["tray_floor"]["height"])) == (230.0, 164.0)
    assert (float(by_id["left_wall"]["width"]), float(by_id["left_wall"]["height"])) == (130.0, 164.0)
    assert (float(by_id["right_wall"]["width"]), float(by_id["right_wall"]["height"])) == (130.0, 164.0)
    assert (float(by_id["front_lip"]["width"]), float(by_id["front_lip"]["height"])) == (230.0, 28.0)
    assert (float(by_id["front_panel"]["width"]), float(by_id["front_panel"]["height"])) == (218.0, 102.0)
    assert (float(by_id["header_fold"]["width"]), float(by_id["header_fold"]["height"])) == (218.0, 18.0)
    assert (float(by_id["side_tab_left"]["width"]), float(by_id["side_tab_left"]["height"])) == (35.0, 102.0)
    assert (float(by_id["side_tab_right"]["width"]), float(by_id["side_tab_right"]["height"])) == (35.0, 102.0)
    assert float(by_id["perf_bottom"]["x2"]) - float(by_id["perf_bottom"]["x1"]) == pytest.approx(218.0)
    assert float(by_id["perf_left"]["y2"]) - float(by_id["perf_left"]["y1"]) == pytest.approx(102.0)
    assert float(by_id["perf_right"]["y2"]) - float(by_id["perf_right"]["y1"]) == pytest.approx(102.0)

    old_preset = os.environ.get("SHELF_TRAY_PRESET")
    old_grid = os.environ.get("SHELF_TRAY_GRID")
    saved = {name: sys.modules.pop(name, None) for name in ("parts",)}
    try:
        sys.path.insert(0, str(HERE))
        for preset, expected_board_mm in (("dieline", 3.0), ("retail_srp", 3.2)):
            os.environ["SHELF_TRAY_PRESET"] = preset
            os.environ["SHELF_TRAY_GRID"] = "3x2"
            sys.modules.pop("parts", None)
            p_mod = importlib.import_module("parts")
            info = p_mod.summary()
            assert info["tray_preset"] == preset
            assert info["tray_mm"]["board_t"] == pytest.approx(expected_board_mm)
            assert info["n_cans"] == 6
            assert len(info["key_figures"]) >= 8

        os.environ["SHELF_TRAY_PRESET"] = "retail_srp"
        os.environ["SHELF_TRAY_GRID"] = "4x3"
        sys.modules.pop("parts", None)
        p12 = importlib.import_module("parts")
        assert p12.N_CANS == 12 and len(p12.can_local_xy()) == 12
    finally:
        if old_preset is None:
            os.environ.pop("SHELF_TRAY_PRESET", None)
        else:
            os.environ["SHELF_TRAY_PRESET"] = old_preset
        if old_grid is None:
            os.environ.pop("SHELF_TRAY_GRID", None)
        else:
            os.environ["SHELF_TRAY_GRID"] = old_grid
        sys.modules.pop("parts", None)
        if saved["parts"] is not None:
            sys.modules["parts"] = saved["parts"]


# --- 2. Non-tear guards (idle, pinch/proximity, straight lift, symmetric pull) ---


def test_idle_and_pinch_or_proximity_never_tear_or_score(m):
    b = Bench(m)
    assert b.step(25) == 0.0
    assert m.L.ties_torn() == 0 and not m.L.detached()

    # Both grippers placed right on the front_panel rim and tray wall, closed tightly, for 40 steps (2 s): 0 ties torn
    b.grippers = {
        "left": (np.array([0.322, +0.1134, m.TOP + 0.128]), 0.0),
        "right": (np.array([0.202, -0.0750, m.TOP + 0.128]), 0.0),
    }
    assert b.step(40) == 0.0
    assert m.L.ties_torn() == 0 and not m.L.detached()


def test_straight_lift_holds_below_threshold_and_overloads_only_above_threshold(m):
    P = m.P
    # Sourced figure: tensile index 5.2 N m/g x 405 g/m^2 x 218 mm bottom line = 459.1 N, far beyond the slide's stop
    assert P.STRAIGHT_LIFT_HOLD_N == pytest.approx(5.2 * 405.0 * 0.218, abs=0.1)
    assert P.K_LIFT * P.LIFT_STOP_M < 0.2 * P.STRAIGHT_LIFT_HOLD_N
    b = Bench(m)
    # 30 N vertical lift -> 3.0 mm stretch, 0 ties torn, weld intact
    b.joints["perf_lift"] = 30.0 / P.K_LIFT
    assert b.step(10) == 0.0
    assert m.L.ties_torn() == 0 and not m.L.detached()
    assert m.report()["peak_lift_n"] == pytest.approx(30.0, abs=0.05)

    # At the slide's stop (65 N) it still holds: in the simulator a straight lift never tears
    b.joints["perf_lift"] = P.LIFT_STOP_M
    b.step(2)
    assert not m.L.detached() and m.L.ties_torn() == 0

    # The law's own boundary (reachable only on this test rig, past the stop): 458 N holds, 460 N tears
    b.joints["perf_lift"] = 458.0 / P.K_LIFT
    b.step(1)
    assert not m.L.detached()
    b.joints["perf_lift"] = 460.0 / P.K_LIFT
    b.step(1)
    assert m.L.detached() and m.L.ties_torn() == P.N_TIES


def test_symmetric_pull_without_corner_skew_holds_in_its_elastic_range(m):
    b = Bench(m)
    # Flat pull at y=0 -> peel_skew = 0.0 deg (< 6.0 deg); 3.2 deg is inside the 0.98 deg lag + 2.5 deg elastic band
    b.joints["peel_skew"] = 0.0
    b.joints["peel_pitch"] = math.radians(3.2)
    assert b.step(10) == 0.0
    assert not m.L.STATE["corner_started"]
    assert m.L.ties_torn() == 0 and not m.L.detached()


# --- 3. Irreversible progressive corner peel & mid-peel release (t22 requirement) ---


def test_corner_skew_threshold_and_force_levels(m):
    P = m.P
    b = Bench(m)
    # 5.9 deg skew (< 6.0 deg) at 4.5 deg pitch does not initiate the corner peel (the flat fold tears no stage yet)
    b.joints["peel_skew"] = math.radians(5.9)
    b.joints["peel_pitch"] = math.radians(4.5)
    b.step(2)
    assert not m.L.STATE["corner_started"] and m.L.ties_torn() == 0

    # 6.1 deg skew initiates corner peel: before tie 1 snaps (< 8.0 deg), cap is PEEL_START_N (2.75 N)
    fresh(m)
    b = Bench(m)
    b.joints["peel_skew"] = math.radians(6.1)
    b.joints["peel_pitch"] = math.radians(4.0)
    b.step(1)
    assert m.L.STATE["corner_started"]
    assert m.L.ties_torn() == 0
    assert m.report()["peak_peel_n"] == pytest.approx(P.PEEL_START_N, rel=0.02)

    # Advance past tie 1 (8.5 deg) -> ties_torn == 1, the cap stays at PEEL_RUN_N (= PEEL_START_N, 2.75 N)
    b.joints["peel_pitch"] = math.radians(8.5)
    b.step(1)
    assert m.L.ties_torn() == 1
    b.joints["peel_pitch"] = math.radians(15.0)
    b.step(1)
    assert m.L.peel_force_n() == pytest.approx(P.PEEL_RUN_N, rel=0.02)


def test_elastic_corner_bump_at_zero_ties_does_not_unlock_flat_pull(m):
    """Bumping a corner to >= 6 deg skew without snapping tie 1 (< 8 deg) and releasing resets corner_started
    to False, so a subsequent flat y=0 pull still faces the 5.5 N flat-pull hold."""
    P = m.P
    b = Bench(m)
    b.joints["peel_skew"] = math.radians(6.8)
    b.joints["peel_pitch"] = math.radians(4.0)  # < 8.0 deg -> 0 ties torn
    b.step(2)
    assert m.L.STATE["corner_started"] and m.L.ties_torn() == 0
    assert m.L.STATE["rest"] == 0.0

    # Release back to 0 skew and 0 pitch -> corner_started clears, cap returns to SYMMETRIC_PULL_HOLD_N
    b.joints["peel_skew"] = 0.0
    b.joints["peel_pitch"] = 0.0
    b.step(2)
    assert not m.L.STATE["corner_started"]
    assert m.L.target_tau_cap() == pytest.approx(P.SYMMETRIC_PULL_HOLD_N * P.PEEL_LEVER_ARM)

    # Subsequent flat y=0 pull to 4.5 deg (0 skew) yields but tears no stage (stage 1 is at 8.0 deg)
    b.joints["peel_skew"] = 0.0
    b.joints["peel_pitch"] = math.radians(4.5)
    assert b.step(5) == 0.0
    assert not m.L.STATE["corner_started"] and m.L.ties_torn() == 0


def test_irreversible_mid_peel_release_at_30_deg_stays_above_28_deg_and_keeps_ties(m):
    """Required by t22 spec amendment: peel to 30.0 deg, let go -> panel never swings back below 28.0 deg,
    ties_torn never decreases, score is 0.5, and untorn ties still resist further peel past 30.0 deg."""
    P = m.P
    b = Bench(m)
    b.joints["peel_skew"] = math.radians(8.0)
    for deg in (5.0, 10.0, 18.0, 25.0, 30.0):
        b.joints["peel_pitch"] = math.radians(deg)
        b.step(1)

    ties_at_30 = m.L.ties_torn()
    rest_at_30_deg = math.degrees(m.L.STATE["rest"])
    eq_deg = m.L.free_equilibrium_pitch_deg()
    assert ties_at_30 >= P.PARTIAL_TIES_MIN
    assert rest_at_30_deg >= 28.0
    assert eq_deg >= 28.0
    assert not m.L.detached()

    # Let go: move grippers away and place peel_pitch at its free equilibrium (and even test forcing a slightly smaller angle)
    b.grippers = dict(AWAY_GRIPPERS)
    b.joints["peel_skew"] = 0.0
    b.joints["peel_pitch"] = math.radians(eq_deg)
    sc = b.step(10)
    assert sc == 0.5
    assert math.degrees(m.L.STATE["rest"]) == pytest.approx(rest_at_30_deg, abs=1e-6)
    assert m.L.ties_torn() == ties_at_30

    # Even if an external hand pushes the flap back toward 15 deg, neither rest nor ties_torn ever decreases
    b.joints["peel_pitch"] = math.radians(15.0)
    b.step(3)
    assert math.degrees(m.L.STATE["rest"]) == pytest.approx(rest_at_30_deg, abs=1e-6)
    assert m.L.ties_torn() == ties_at_30

    # Resuming peel past 30 deg to 38 deg still faces PEEL_RUN_N (2.75 N) from the remaining untorn ties
    b.joints["peel_skew"] = math.radians(8.0)
    b.joints["peel_pitch"] = math.radians(38.0)
    # Check torque before watch() updates rest, via report's force_profile entry at 38 deg
    b.step(1)
    assert m.L.ties_torn() > ties_at_30
    assert m.report()["force_profile"][-1]["peel_n"] == pytest.approx(P.PEEL_RUN_N, rel=0.02)


# --- 4. Whole-trial zero rules (both sides of every threshold) ---


def test_tray_tilt_lift_and_shift_zero_thresholds(m):
    # (a) Tray tilt: 9.9 deg allowed -> 1.0; 10.1 deg latches permanent 0.0
    b = Bench(m)
    tp0, _ = m.mk.built_pose("tray_base")
    b.poses["tray_base"] = (tp0, axis_angle((0.0, 1.0, 0.0), 9.9))
    b.step(2)
    b.poses.pop("tray_base")
    b.complete_clean_peel()
    b.place_panel_flat()
    assert b.step(22) == 1.0

    fresh(m)
    b = Bench(m)
    b.poses["tray_base"] = (tp0, axis_angle((0.0, 1.0, 0.0), 10.1))
    b.step(1)
    b.poses.pop("tray_base")
    b.complete_clean_peel()
    b.place_panel_flat()
    assert b.step(25) == 0.0
    assert "tilted" in m.report()["zero_reason"]

    # (b) Tray lift: 14.8 mm above TOP allowed -> 1.0; 15.2 mm above TOP latches permanent 0.0
    fresh(m)
    b = Bench(m)
    b.poses["tray_base"] = (np.array([tp0[0], tp0[1], m.TOP + 0.0148]), np.eye(3))
    b.step(2)
    b.poses.pop("tray_base")
    b.complete_clean_peel()
    b.place_panel_flat()
    assert b.step(22) == 1.0

    fresh(m)
    b = Bench(m)
    b.poses["tray_base"] = (np.array([tp0[0], tp0[1], m.TOP + 0.0152]), np.eye(3))
    b.step(1)
    b.poses.pop("tray_base")
    b.complete_clean_peel()
    b.place_panel_flat()
    assert b.step(25) == 0.0
    assert "lifted" in m.report()["zero_reason"]

    # (c) Tray corner horizontal shift: 24.8 mm allowed -> 1.0; 25.2 mm latches permanent 0.0
    fresh(m)
    b = Bench(m)
    b.poses["tray_base"] = (tp0 + np.array([-0.0248, 0.0, 0.0]), np.eye(3))
    # Shift cans along with tray_base so cans stay inside tray
    for cname in m.CAN_NAMES:
        cp0, cr0 = m.mk.built_pose(cname)
        b.poses[cname] = (cp0 + np.array([-0.0248, 0.0, 0.0]), cr0)
    b.complete_clean_peel()
    b.place_panel_flat()
    assert b.step(22) == 1.0

    fresh(m)
    b = Bench(m)
    b.poses["tray_base"] = (tp0 + np.array([-0.0252, 0.0, 0.0]), np.eye(3))
    for cname in m.CAN_NAMES:
        cp0, cr0 = m.mk.built_pose(cname)
        b.poses[cname] = (cp0 + np.array([-0.0252, 0.0, 0.0]), cr0)
    b.step(1)
    b.poses.clear()
    b.complete_clean_peel()
    b.place_panel_flat()
    assert b.step(25) == 0.0
    assert "shifted" in m.report()["zero_reason"]


def test_can_tip_whole_trial_12deg_and_end_state_10deg_and_outside_tray(m):
    cp0, _ = m.mk.built_pose("can_0")

    # Transient 11.8 deg mid-trial wobble that settles upright -> allowed (1.0)
    b = Bench(m)
    b.poses["can_0"] = (cp0, axis_angle((1.0, 0.0, 0.0), 11.8))
    b.step(2)
    b.poses.pop("can_0")
    b.complete_clean_peel()
    b.place_panel_flat()
    assert b.step(22) == 1.0

    # Transient 12.2 deg mid-trial tip -> permanent 0.0 even after uprighting
    fresh(m)
    b = Bench(m)
    b.poses["can_0"] = (cp0, axis_angle((1.0, 0.0, 0.0), 12.2))
    b.step(1)
    b.poses.pop("can_0")
    b.complete_clean_peel()
    b.place_panel_flat()
    assert b.step(25) == 0.0
    assert "can_0 tipped" in m.report()["zero_reason"]

    # End-state 9.8 deg -> 1.0; end-state 10.2 deg -> 0.0
    fresh(m)
    b = Bench(m)
    b.complete_clean_peel()
    b.place_panel_flat()
    b.poses["can_0"] = (cp0, axis_angle((1.0, 0.0, 0.0), 9.8))
    assert b.step(22) == 1.0
    b.poses["can_0"] = (cp0, axis_angle((1.0, 0.0, 0.0), 10.2))
    assert b.step(1) == 0.0

    # Can outside tray_base -> permanent 0.0
    fresh(m)
    b = Bench(m)
    b.poses["can_2"] = (np.array([0.28, 0.20, m.TOP + 0.06]), np.eye(3))
    b.step(1)
    b.poses.pop("can_2")
    b.complete_clean_peel()
    b.place_panel_flat()
    assert b.step(25) == 0.0
    assert "left tray_base" in m.report()["zero_reason"]


# --- 5. Partial (0.5) vs Full (1.0) end-state boundaries ---


def test_full_score_boundaries_flatness_gap_contact_grippers_speed_and_hold(m):
    P = m.P
    b = Bench(m)
    b.complete_clean_peel()

    # Detached but still upright / tilted 15.3 deg -> 0.5; tilted 14.7 deg -> 1.0 (both face_up and face_down)
    b.place_panel_flat(tilt_deg=15.3, face_up=True)
    assert b.step(25) == 0.5
    b.place_panel_flat(tilt_deg=14.7, face_up=False)
    assert b.step(22) == 1.0

    # Panel center height > 20 mm above bench -> 0.5
    b.place_panel_flat(z_above_top=0.021)
    assert b.step(5) == 0.5
    b.place_panel_flat(z_above_top=0.019)
    assert b.step(22) == 1.0

    # Panel footprint edge gap from tray_base (tray right edge is y = -0.115 m; panel with yaw=-90 has half-width 0.050 m along y):
    # Center y = -0.115 - 0.5 * PANEL_SOLID_H - gap
    b.place_panel_flat(xy=(0.280, -0.115 - 0.5 * P.PANEL_SOLID_H - 0.0295), yaw_deg=-90.0)
    b.step(22)
    assert m.panel_tray_footprint_gap_m() == pytest.approx(0.0295, abs=1e-4)
    assert m.score(None) == 0.5

    b.place_panel_flat(xy=(0.280, -0.115 - 0.5 * P.PANEL_SOLID_H - 0.0305), yaw_deg=-90.0)
    assert b.step(22) == 1.0
    assert m.panel_tray_footprint_gap_m() == pytest.approx(0.0305, abs=1e-4)

    # Non-zero contact force between front_panel and tray_base -> 0.5
    b.contacts = {("front_panel", "tray_base"): 0.25}
    assert b.step(2) == 0.5
    b.contacts.clear()
    assert b.step(22) == 1.0

    # Gripper clearance: 29.5 mm from detached panel top surface -> 0.5; 30.5 mm -> 1.0
    p_pos, _ = b.poses["front_panel"]
    b.grippers["right"] = (p_pos + np.array([0.0, 0.0, 0.5 * P.BOARD_T + 0.0295]), 1.0)
    assert b.step(2) == 0.5
    b.grippers["right"] = (p_pos + np.array([0.0, 0.0, 0.5 * P.BOARD_T + 0.0305]), 1.0)
    assert b.step(22) == 1.0
    b.grippers = dict(AWAY_GRIPPERS)

    # Linear & angular speed limits (10 mm/s, 0.10 rad/s) and exact 20-step (1.0 s) hold
    b.velocities = {"front_panel": (np.array([0.011, 0.0, 0.0]), np.zeros(3))}
    assert b.step(2) == 0.5
    b.velocities = {"tray_base": (np.zeros(3), np.array([0.0, 0.0, 0.105]))}
    assert b.step(2) == 0.5

    b.velocities = {"front_panel": (np.array([0.005, 0.0, 0.0]), np.zeros(3))}
    # 19 steps (0.95 s) -> still 0.5; step 20 (1.00 s) -> 1.0!
    assert b.step(19) == 0.5
    assert b.step(1) == 1.0


def test_left_corner_negative_skew_is_symmetric_with_right_corner(m):
    """Peeling from the top-left corner (negative peel_skew <= -6.0 deg) works identically to top-right."""
    b = Bench(m)
    b.joints["peel_skew"] = math.radians(-7.5)
    for deg in (5.0, 12.0, 28.0, 42.0, 52.5):
        b.joints["peel_pitch"] = math.radians(deg)
        b.step(1)
    assert m.L.STATE["corner_started"]
    assert m.L.detached() and m.L.ties_torn() == m.P.N_TIES
    b.place_panel_flat()
    assert b.step(20) == 1.0


def test_partial_ties_4_vs_5_boundary(m):
    b = Bench(m)
    b.joints["peel_skew"] = math.radians(8.0)
    # 4 ties torn (23.0 deg: ties at 8.0, 12.89, 17.78, 22.67) -> < 5 ties -> score 0.0
    b.joints["peel_pitch"] = math.radians(23.0)
    assert b.step(5) == 0.0
    assert m.L.ties_torn() == 4
    # 5 ties torn (28.0 deg >= 27.56 deg) -> >= 5 ties -> score 0.5
    b.joints["peel_pitch"] = math.radians(28.0)
    assert b.step(5) == 0.5
    assert m.L.ties_torn() == 5


def test_peel_angle_51_9_vs_52_0_detachment_boundary(m):
    b = Bench(m)
    b.joints["peel_skew"] = math.radians(8.0)
    b.joints["peel_pitch"] = math.radians(51.9)
    assert b.step(3) == 0.5
    assert not m.L.detached() and m.L.ties_torn() == 9
    b.joints["peel_pitch"] = math.radians(52.0)
    b.step(1)
    assert m.L.detached() and m.L.ties_torn() == 10


def test_left_gripper_clearance_boundary(m):
    P = m.P
    b = Bench(m)
    b.complete_clean_peel()
    b.place_panel_flat()
    # Left gripper 29.0 mm above tray_base left wall top (z = TOP + OVER + TRAY_H) -> 0.5
    wall_top = np.array([P.TRAY_XY[0], P.TRAY_XY[1] + 0.5 * P.TRAY_W - 0.5 * P.BOARD_T, m.TOP + P.OVER + P.TRAY_H])
    b.grippers["left"] = (wall_top + np.array([0.0, 0.0, 0.029]), 1.0)
    assert b.step(22) == 0.5
    b.grippers["left"] = (wall_top + np.array([0.0, 0.0, 0.031]), 1.0)
    assert b.step(20) == 1.0


def test_flat_pull_folds_the_panel_tearing_only_side_seams_and_never_detaches(m):
    P = m.P
    # Sourced figures: corner peel 2 x 0.275 N x 422 mm / 84.4 mm = 2.75 N; a flat pull runs two fronts: 5.5 N
    assert P.PEEL_RUN_N == pytest.approx(2.0 * 0.68e-3 * 405.0 * 0.422 / 0.0844, rel=0.01)
    assert P.PEEL_START_N == P.PEEL_RUN_N and P.SYMMETRIC_PULL_HOLD_N == pytest.approx(2.0 * P.PEEL_RUN_N)
    assert P.SIDE_SEAM_TIES == 5
    b = Bench(m)
    # Without corner skew (0 deg), folding the panel down to the hinge stop tears the side seams only and keeps it on
    b.joints["peel_skew"] = 0.0
    for deg in (10.0, 25.0, 40.0, 52.5, 84.0):
        b.joints["peel_pitch"] = math.radians(deg)
        b.step(1)
    assert m.L.STATE["flat_fold"] and not m.L.STATE["corner_started"]
    assert m.L.ties_torn() == P.SIDE_SEAM_TIES and not m.L.detached()
    assert m.L.target_tau_cap() == pytest.approx(P.SYMMETRIC_PULL_HOLD_N * P.PEEL_LEVER_ARM)
    assert b.step(3) == 0.5  # half the perforation torn, the panel still on
    # Turning the folded panel's corner (8 deg skew) runs the bottom line: it comes off
    b.joints["peel_skew"] = math.radians(8.0)
    b.step(1)
    assert m.L.STATE["corner_started"] and m.L.detached() and m.L.ties_torn() == P.N_TIES


def test_retail_preset_full_task_load_and_score():
    old_preset = os.environ.get("SHELF_TRAY_PRESET")
    saved = {name: sys.modules.pop(name, None) for name in OWN_MODULES}
    try:
        os.environ["SHELF_TRAY_PRESET"] = "retail_srp"
        mod = task_file.load(TASK_PATH).module
        fresh(mod)
        assert mod.P.TRAY_PRESET == "retail_srp"
        assert mod.P.BOARD_T == pytest.approx(0.0032)
        b = Bench(mod)
        b.complete_clean_peel()
        b.place_panel_flat()
        assert b.step(20) == 1.0
        assert mod.success(None) is True
        mod.mk.MECH.clear()
    finally:
        if old_preset is None:
            os.environ.pop("SHELF_TRAY_PRESET", None)
        else:
            os.environ["SHELF_TRAY_PRESET"] = old_preset
        for name, prev in saved.items():
            sys.modules.pop(name, None)
            if prev is not None:
                sys.modules[name] = prev


def test_task_json_and_readme_checklist_invariants(m):
    tj = json.loads((HERE / "task.json").read_text())
    readme = (HERE / "README.md").read_text()
    task_src = (HERE / "task.py").read_text()
    peel_src = (HERE / "peel.py").read_text()

    # Checklist 22 & 52 exact character caps
    assert len(tj["title"]) <= 120
    assert len(tj["context"]) <= 1000
    assert len(tj["fidelity"]) <= 500
    assert len(readme) <= 10000
    assert len(task_src) <= 15000
    assert len(readme) + len(task_src) + len(peel_src) <= 36000

    assert tj["category"] == "Impact on jobs"
    assert tj["instruction"] == m.INSTRUCTION
    assert "Not modelled:" in tj["fidelity"]
    assert "1415158438" in readme
    assert "Design notes: tear forces from a published study, and progressive peeling" in readme
    assert re.search(r"\breview(?:er|ers|s)?\b|\bsubmission(?:s)?\b", readme, re.IGNORECASE) is None

    for token in ("10.0 deg", "15.0 mm", "25.0 mm", "12.0 deg", "30.0 mm", "20.0 mm", "1.0 s"):
        assert token in tj["scoring"], f"missing {token!r} in task.json scoring"
        assert token in readme, f"missing {token!r} in README.md"

    # Check 47: zero board pull-request numbers or review IDs anywhere in folder text, code, or SVG
    forbidden = re.compile(r"(?:(?<!&)#\d{3,5}\b|\bTK-[A-Z0-9]{4,}\b)")
    for p in sorted(HERE.iterdir()):
        if p.suffix in {".py", ".md", ".json", ".svg"} and p.name not in {"mechkit.py", "demo_kit.py", "kit_extra.py"}:
            txt = p.read_text()
            hit = forbidden.search(txt)
            assert hit is None, f"forbidden board reference {hit.group(0)!r} in {p.name}"


