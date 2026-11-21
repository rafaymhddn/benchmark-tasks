"""Unit tests for mech/tasks/holesaw_arbor (pure CPU, no simulator required)."""

from __future__ import annotations

import json
import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as K  # noqa: E402
import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402
import scene  # noqa: E402
import task as T  # noqa: E402


def _reset_offline(collar_z: float = P.PIN_STROKE) -> None:
    mk.mend_all()
    T.begin(seed=0)
    mk.set_state(
        joints={"collar_z": collar_z, "cup_z": 0.0, "turn_1": 0.0, "turn_2": 0.0, "turn_3": 0.0},
        grippers={"left": (np.array([0.0, 0.30, T.TOP + 0.25]), 1.0), "right": (np.array([0.0, -0.30, T.TOP + 0.25]), 1.0)},
    )


def _saw_on_arbor(gap_m: float, yaw_deg: float, collar_z: float, *, clear_hands: bool = True, lin_speed: float = 0.0) -> None:
    saw_pos = np.array([P.X_ARBOR, P.Y_ARBOR, T.TOP + P.Z_SHOULDER + gap_m])
    saw_rot = mk.quat_to_matrix(mk.yaw_quat(math.radians(yaw_deg)))
    g_right = np.array([0.0, -0.30, T.TOP + 0.25]) if clear_hands else np.array([P.X_ARBOR, P.Y_ARBOR - 0.02, T.TOP + P.Z_COLLAR_LOW])
    mk.set_state(
        poses={"hole_saw": (saw_pos, saw_rot)},
        velocities={"hole_saw": (np.array([lin_speed, 0.0, 0.0]), np.zeros(3))},
        joints={"collar_z": collar_z},
        grippers={"left": (np.array([0.0, 0.30, T.TOP + 0.25]), 1.0), "right": (g_right, 1.0)},
    )


def _engage(yaw_deg: float = -30.0, under_m: float = 0.00005) -> float:
    """Set the hole saw on the thread start (coaxial, level, collar retracted) and step once: the thread catches."""
    gap = P.thread_start_gap(math.radians(yaw_deg)) - under_m
    _saw_on_arbor(gap_m=gap, yaw_deg=yaw_deg, collar_z=0.0)
    T.step()
    assert T.thread_engaged()
    return gap


def test_idle_scores_zero() -> None:
    _reset_offline(collar_z=P.PIN_STROKE)
    T.step()
    assert T.score(None) == 0.0
    assert not T.success(None)
    assert T.STATE["zero"] is None
    assert not T.STATE["ever_retracted"]


def test_drop_below_bench_latches_permanent_zero() -> None:
    _reset_offline(collar_z=0.0)
    # Drop hole saw below bench -> latches zero
    mk.set_state(
        poses={"hole_saw": (np.array([P.X_MAT, P.Y_MAT, T.TOP - 0.020]), np.eye(3))},
        joints={"collar_z": 0.0},
    )
    T.step()
    assert T.STATE["zero"] is not None
    assert T.score(None) == 0.0

    # Even if subsequently placed in exact 1.0 locked state and held 25 steps, score stays 0.0
    gap_360 = P.gap_for_yaw_deg(P.ALIGN_DEG)
    _saw_on_arbor(gap_360, 0.0, P.PIN_STROKE)
    for _ in range(P.HOLD_STEPS + 5):
        T.step()
    assert T.score(None) == 0.0


def test_collar_up_shortcut_scores_zero() -> None:
    """Pressing the hole saw onto extended pins without retracting the collar leaves it 7 mm high -> 0.0."""
    _reset_offline(collar_z=P.PIN_STROKE)
    _saw_on_arbor(gap_m=0.0070, yaw_deg=35.0, collar_z=P.PIN_STROKE)
    T.step()
    assert not T. thread_engaged()
    assert T.score(None) == 0.0


def test_retracted_collar_scores_quarter() -> None:
    _reset_offline(collar_z=0.0010)
    T.step()
    assert T.STATE["ever_retracted"] is True
    assert T.score(None) == 0.25


def test_bottomed_out_stop_stalls_at_quarter() -> None:
    """Screwing tight to the +415 deg shoulder stop (gap 0.0 mm, holes 55 deg off pins, collar stalled at 1 mm) -> 0.25."""
    _reset_offline(collar_z=0.0)
    T.step()
    _saw_on_arbor(gap_m=0.0, yaw_deg=-55.0, collar_z=0.0010)
    for _ in range(P.HOLD_STEPS + 2):
        T.step()
    assert T.saw_phase_err_deg() > 50.0
    assert T.score(None) == 0.25


def test_early_half_turn_at_180_deg_scores_quarter() -> None:
    """Stopping half a turn early at +180 deg leaves a 0.92 mm gap (> 0.60 mm feeler limit) -> 0.25, never 0.5 or 1.0."""
    _reset_offline(collar_z=0.0)
    T.step()
    gap_180 = P.gap_for_yaw_deg(180.0)  # 0.921 mm > 0.60 mm
    assert gap_180 > P.SEATED_GAP_MAX
    _saw_on_arbor(gap_m=gap_180, yaw_deg=180.0, collar_z=P.PIN_STROKE)
    for _ in range(P.HOLD_STEPS + 2):
        T.step()
    assert T.score(None) == 0.25


def test_backed_off_aligned_without_collar_scores_half() -> None:
    """Backed off 55 deg CCW from +415 deg to +360 deg (gap 0.22 mm <= 0.60 mm, phase <= 8 deg), collar down -> 0.5."""
    _reset_offline(collar_z=0.0)
    _engage()
    gap_360 = P.gap_for_yaw_deg(P.ALIGN_DEG)
    assert 0.0001 <= gap_360 <= P.SEATED_GAP_MAX
    _saw_on_arbor(gap_m=gap_360, yaw_deg=5.0, collar_z=0.0020)
    T.step()
    assert T.score(None) == 0.5
    # Outside +/-8 deg alignment window drops back to 0.25
    _saw_on_arbor(gap_m=gap_360, yaw_deg=9.5, collar_z=0.0020)
    T.step()
    assert T.score(None) == 0.25


def test_full_score_requires_collar_locked_hands_clear_still_and_hold() -> None:
    _reset_offline(collar_z=0.0)
    _engage()
    gap_360 = P.gap_for_yaw_deg(P.ALIGN_DEG)
    # Hands too close -> stays 0.5
    _saw_on_arbor(gap_m=gap_360, yaw_deg=0.0, collar_z=0.0078, clear_hands=False)
    for _ in range(P.HOLD_STEPS + 2):
        T.step()
    assert T.score(None) == 0.5

    # Moving too fast -> stays 0.5
    _saw_on_arbor(gap_m=gap_360, yaw_deg=0.0, collar_z=0.0078, clear_hands=True, lin_speed=0.025)
    for _ in range(P.HOLD_STEPS + 2):
        T.step()
    assert T.score(None) == 0.5

    # Hands clear and still -> 0.5 before hold finishes, 1.0 at step HOLD_STEPS
    _saw_on_arbor(gap_m=gap_360, yaw_deg=0.0, collar_z=0.0078, clear_hands=True, lin_speed=0.0)
    for k in range(P.HOLD_STEPS - 1):
        T.step()
        assert T.score(None) == 0.5, f"reached 1.0 early at step {k + 1}"
    T.step()
    assert T.score(None) == 1.0
    assert T.success(None)


def test_thread_catches_when_retracted_and_releases_when_unthreaded() -> None:
    _reset_offline(collar_z=0.0)
    _engage(yaw_deg=0.0, under_m=0.0)  # at yaw 0 the thread start is Z_ENTRY (1.63 mm over the shoulder)
    assert math.isclose(P.thread_start_gap(0.0), P.Z_ENTRY - P.Z_SHOULDER, abs_tol=1e-12)

    # Turning CCW back past UNTHREAD_DEG (-3.0 deg) cuts w_thread
    per_unthread = math.radians(-4.5) / 3.0
    mk.set_state(
        poses={"hole_saw": (np.array([P.X_ARBOR, P.Y_ARBOR, T.TOP + P.Z_ENTRY]), np.eye(3))},
        joints={"collar_z": 0.0, "turn_1": per_unthread, "turn_2": per_unthread, "turn_3": per_unthread},
    )
    T.step()
    assert not T.thread_engaged()


def test_thread_start_needs_coaxial_square_on_the_first_thread() -> None:
    """The catch fires only where the robot has set the saw on the thread start; nothing moves the saw onto it."""
    yaw = math.radians(-30.0)
    start = P.thread_start_gap(yaw)
    assert math.isclose(start, P.gap_for_yaw_deg(30.0), rel_tol=1e-12)
    for dx, tilt_deg, under, collar, ok in (
        (0.00045, 0.0, 0.00005, 0.0, True),  # 0.45 mm off-axis, level, on the first thread: catches
        (0.0, 1.4, 0.00014, 0.0, True),  # square within 1.5 deg, 0.14 mm under the start: catches
        (0.00055, 0.0, 0.00005, 0.0, False),  # 0.55 mm off-axis: cross-threads
        (0.0, 1.6, 0.00005, 0.0, False),  # tilted 1.6 deg: cross-threads
        (0.0, 0.0, -0.00005, 0.0, False),  # 0.05 mm over the start: not on the thread yet
        (0.0, 0.0, 0.00016, 0.0, False),  # 0.16 mm under the start (past the band): it slid past the start
        (0.0, 0.0, 0.00005, 0.0025, False),  # collar not retracted
    ):
        _reset_offline(collar_z=collar)
        rot = mk.quat_to_matrix(mk.yaw_quat(yaw)) @ mk.axis_angle((1.0, 0.0, 0.0), math.radians(tilt_deg))
        mk.set_state(poses={"hole_saw": (np.array([P.X_ARBOR + dx, P.Y_ARBOR, T.TOP + P.Z_SHOULDER + start - under]), rot)},
                     joints={"collar_z": collar})
        T.step()
        assert T.thread_engaged() is ok, (dx, tilt_deg, under, collar)


def test_catch_engages_where_the_saw_stands() -> None:
    """The thread travel taken at the catch is the one for the saw's own height: the carrier goes to the saw."""
    _reset_offline(collar_z=0.0)
    gap = _engage(yaw_deg=-30.0, under_m=0.00009)
    catch = T.STATE["catch"]
    assert math.isclose(catch["cw_deg"], round(P.cw_for_gap_deg(gap), 2), abs_tol=0.011)
    assert math.isclose(P.gap_for_yaw_deg(P.cw_for_gap_deg(gap)), gap, abs_tol=1e-12)
    assert 30.0 <= catch["cw_deg"] <= 30.0 + 360.0 * P.CATCH_GAP_BAND / P.THREAD_PITCH  # holes still line up short of the stop
    for name in ("task.py", "demo.py", "scene.py"):
        assert "write_pose" not in (HERE / name).read_text(), f"{name} writes a pose"


def test_uncaught_saw_on_the_shoulder_never_scores_seated() -> None:
    """A saw that never caught the thread (here dropped on the shoulder, aligned, collar pushed up, hands clear) is
    cross-threaded: 0.25 for the retracted collar, never 0.5 or 1.0."""
    _reset_offline(collar_z=0.0)
    T.step()
    _saw_on_arbor(gap_m=0.0002, yaw_deg=0.0, collar_z=0.0078)
    for _ in range(P.HOLD_STEPS + 3):
        T.step()
    assert not T.thread_engaged()
    assert T.score(None) == 0.25


def test_non_overlapping_backplate_boxes_and_figures() -> None:
    boxes = scene._backplate_boxes()
    assert len(boxes) == 12
    for i, a in enumerate(boxes):
        lo_a, hi_a = a.bounds()
        for b in boxes[i + 1 :]:
            lo_b, hi_b = b.bounds()
            overlap = np.minimum(hi_a, hi_b) - np.maximum(lo_a, lo_b)
            assert float(overlap.min()) <= 1e-6, f"backplate boxes overlap: {lo_a}..{hi_a} vs {lo_b}..{hi_b}"
    assert len(P.FIGURES) >= 18
    for name, (_val, _unit, kind, note) in P.FIGURES.items():
        assert kind in (P.MAKER, P.STANDARD, P.FIXTURE, P.ESTIMATE), name
        assert "assumed until" not in note.lower() and "measured" not in note.lower(), f"banned phrase in {name}: {note}"


def test_offline_reach_and_file_size_limits() -> None:
    offline = K.run_offline()
    assert offline["pass"] is True
    assert offline["worst_ik_mm"] <= 0.5
    assert offline["smallest_joint_margin_deg"] >= 5.0
    assert offline["max_grasp_height_mm"] <= 160.0
    assert offline["min_hand_separation_mm"] >= 20.0

    task_bytes = len((HERE / "task.py").read_bytes())
    assert task_bytes <= 15000, f"task.py has {task_bytes} bytes (> 15,000)"
    readme_p = HERE / "README.md"
    if readme_p.exists():
        readme_bytes = len(readme_p.read_bytes())
        assert readme_bytes <= 10000, f"README.md has {readme_bytes} bytes (> 10,000)"
    tj_p = HERE / "task.json"
    if tj_p.exists():
        tj = json.loads(tj_p.read_text(encoding="utf-8"))
        assert len(tj["title"]) <= 120
        assert len(tj["context"]) <= 1000
        assert len(tj["fidelity"]) <= 500
        assert tj["instruction"] == T.INSTRUCTION
        assert tj["scores"] == "Dream-1 run: pending; demo: 1.0"
