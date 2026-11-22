"""Fast offline rule tests for ``mech/tasks/tympanic_cover``."""

from __future__ import annotations

import json
import pathlib
import re
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as K  # noqa: E402
import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402
import task  # noqa: E402


def _quat_x_deg(deg: float) -> np.ndarray:
    half = np.radians(deg) / 2.0
    return np.array([np.sin(half), 0.0, 0.0, np.cos(half)], dtype=float)  # xyzw


def _setup_mock_world(monkeypatch):
    """Populate ``mk.MECH.poses`` offline (no Isaac Sim required) and return mutable joint/weld/contact state."""
    mk.MECH.environment = None
    mk.MECH.poses = {n: mk.built_pose(n) for n in mk.MECH.bodies}
    joint_map = {"eject_slide": 0.0}
    cut_set = {f"weld_cone_{i}" for i in range(len(P.FRESH))}
    touch_map: dict[tuple[str, str], bool] = {}
    force_map: dict[tuple[str, str], float] = {}
    monkeypatch.setattr(mk, "joint", lambda n: joint_map.get(n, 0.0))
    monkeypatch.setattr(mk, "is_cut", lambda n: n in cut_set)
    monkeypatch.setattr(mk, "cut", lambda n: cut_set.add(n))
    monkeypatch.setattr(mk, "make", lambda n: cut_set.discard(n))
    monkeypatch.setattr(mk, "touching", lambda a, b: touch_map.get((a, b), False))
    monkeypatch.setattr(mk, "contact_force", lambda a, b: force_map.get((a, b), 0.0))
    task.begin(0)
    far = np.array([[0.20, 0.45, task.TOP + 0.25]])
    task.STATE["hands"] = {"left": far.copy(), "right": np.array([[0.20, -0.45, task.TOP + 0.25]])}
    return joint_map, cut_set, touch_map, force_map


def _place(name: str, pos, quat=(0.0, 0.0, 0.0, 1.0)) -> None:
    mk.MECH.poses[name] = (np.asarray(pos, dtype=float), mk.quat_to_matrix(quat))


def _put_used_in_tray(cut_set: set[str], z_extra: float = 0.0) -> None:
    cut_set.add("weld_used")
    _place("cover_used", (P.TRAY_AT[0], P.TRAY_AT[1], task.TOP + P.OVER + P.TRAY_FLOOR + z_extra))


def _dock_thermometer_with_fresh0(cut_set: set[str], tilt_deg: float = 0.0) -> None:
    q = _quat_x_deg(tilt_deg)
    r = mk.quat_to_matrix(q)
    tp = np.array(P.DOCK_ORIGIN, dtype=float) + np.array([0.0, 0.0, task.TOP + P.OVER])
    _place("thermometer", tp, q)
    _place("eject_btn", tp + r @ np.array(P.BTN_POS_LOCAL), mk.matrix_to_quat(r @ P.BTN_ROT_T))
    _place("cover_fresh_0", tp + r @ np.array(P.USED_ON_THERM_LOCAL), mk.matrix_to_quat(r @ P.COVER_ON_PROBE_ROT_T))
    cut_set.add("weld_well_0")
    cut_set.discard("weld_cone_0")


def test_metadata_and_no_kinematic_snapping():
    expected = (
        "Eject the used amber probe cover into the grey tray without touching it or letting it touch the cradle "
        "or the white covers, press one fresh white cover from the rack onto the probe and keep it on, rest the "
        "thermometer in its cradle, and let go with both hands clear."
    )
    assert task.INSTRUCTION == expected
    assert task.SCENE == "station"
    assert task.SCORE_OVER == "end"
    assert task.RUN_LIMIT_STEPS == 1300
    assert len((HERE / "task.py").read_text(encoding="utf-8")) <= 15_000

    for fname in ("parts.py", "scene.py", "task.py", "demo.py"):
        fp = HERE / fname
        if not fp.exists():
            continue
        code_lines = [
            ln.split("#", 1)[0]
            for ln in fp.read_text(encoding="utf-8").splitlines()
            if not ln.lstrip().startswith(('"""', "#"))
        ]
        joined = "\n".join(code_lines)
        assert not re.search(r"\b(write_pose|set_joint)\s*\(", joined), f"forbidden kinematic snap in {fname}"


def test_layout_and_offline_kinematics():
    assert P.layout_ok()
    d = K.run_offline()
    assert d["pass"], f"offline check failed: {d}"
    for pair_name, info in d["simultaneous_pairs"].items():
        assert info["wrists apart (mm)"] >= 280.0, f"{pair_name}: {info}"
        assert info["hulls apart (mm)"] >= 20.0, f"{pair_name}: {info}"


def test_button_release_and_half_score_gates(monkeypatch):
    joint_map, cut_set, _, _ = _setup_mock_world(monkeypatch)

    # Start state scores 0.0; sub-threshold button stroke (3.9 mm < 4.0 mm) keeps weld_used intact:
    task.step()
    assert task.score(None) == 0.0
    joint_map["eject_slide"] = 0.0039
    task.step()
    assert not mk.is_cut("weld_used")

    # Pressing >= 4.0 mm cuts weld_used:
    joint_map["eject_slide"] = 0.0041
    task.step()
    assert mk.is_cut("weld_used")

    # Ejected cover inside grey tray with recoiled button reaches 0.5 only after 20 steady steps:
    joint_map["eject_slide"] = 0.0
    _put_used_in_tray(cut_set)
    for i in range(19):
        task.step()
        assert task.score(None) == 0.0, f"premature score at step {i + 1}"
    task.step()
    assert task.score(None) == 0.5

    # Button stuck depressed (> 1.0 mm) or hand within 30 mm drops score back to 0.0:
    joint_map["eject_slide"] = 0.0015
    assert task.score(None) == 0.0
    joint_map["eject_slide"] = 0.0
    task.step()
    for _ in range(19):
        task.step()
    assert task.score(None) == 0.5

    near_hand = np.array([[P.TRAY_AT[0], P.TRAY_AT[1], task.TOP + P.TRAY_H + 0.015]])
    task.STATE["hands"]["left"] = near_hand
    assert task.score(None) == 0.0

    # Cover perched on the 20 mm rim fails used_in_tray:
    task.STATE["hands"]["left"] = np.array([[0.20, 0.45, task.TOP + 0.25]])
    _put_used_in_tray(cut_set, z_extra=0.012)
    assert not task.used_in_tray()


def test_coaxial_press_on_and_full_score_and_tier_reset(monkeypatch):
    _, cut_set, _, force_map = _setup_mock_world(monkeypatch)

    _put_used_in_tray(cut_set)
    c0_pos, _ = mk.pose("cover_fresh_0")
    r_load = P.therm_rot(P.R_LOAD_ROT)  # tipped 35 deg nose-down: the probe upright
    q_load = mk.matrix_to_quat(r_load)
    tp_aligned = c0_pos + np.array([0.0, 0.0, 0.004]) - r_load @ P.CONE_TIP_T  # ring on the mouth: tip 4 mm over the cover's floor
    _place("thermometer", tp_aligned, q_load)
    assert task.cone_in_cover("cover_fresh_0")[0] < 1e-6 and task.cone_in_cover("cover_fresh_0")[2] < 1e-3

    # Insufficient force (3.2 N < 3.5 N), radial misalignment (4.0 mm > 3.5 mm) or a level probe (35 deg off) will NOT seat:
    force_map[("cover_fresh_0", "thermometer")] = 3.2
    task.step()
    assert task.mounted_fresh() is None

    force_map[("cover_fresh_0", "thermometer")] = 4.5
    _place("thermometer", tp_aligned + np.array([0.0040, 0.0, 0.0]), q_load)
    task.step()
    assert task.mounted_fresh() is None
    _place("thermometer", tp_aligned)
    task.step()
    assert task.mounted_fresh() is None

    # Coaxial >= 3.5 N press cuts weld_well_0 and engages weld_cone_0:
    _place("thermometer", tp_aligned, q_load)
    task.step()
    assert task.mounted_fresh() == "cover_fresh_0"
    assert mk.is_cut("weld_well_0") and not mk.is_cut("weld_cone_0")

    # Hold 20 steps at 0.5 (undocked), then dock: hold counter must reset on 0.5 -> 1.0 transition!
    force_map.clear()
    for _ in range(20):
        task.step()
    assert task.score(None) == 0.5

    _dock_thermometer_with_fresh0(cut_set, tilt_deg=0.0)
    task.step()
    assert task.score(None) == 0.0  # step 1 of the new 1.0 hold window!
    for _ in range(19):
        task.step()
    assert task.score(None) == 1.0
    assert task.success(None)

    # Tilted 12 deg (> 10 deg dock limit) drops raw_tier back to 0.5:
    _dock_thermometer_with_fresh0(cut_set, tilt_deg=12.0)
    assert not task.docked_in_cradle()
    assert task.raw_tier() == 0.5


def test_permanent_zero_latches(monkeypatch):
    cases = [
        ("hand_left", lambda cs, tm: tm.update({("cover_used", "left robot"): True}), "a hand touched the used cover"),
        ("hand_right", lambda cs, tm: tm.update({("cover_used", "right robot"): True}), "a hand touched the used cover"),
        ("cradle_hit", lambda cs, tm: tm.update({("cover_used", "cradle"): True}), "the used cover touched the cradle or a fresh cover"),
        ("fresh_hit", lambda cs, tm: tm.update({("cover_used", "cover_fresh_1"): True}), "the used cover touched the cradle or a fresh cover"),
        ("missed_tray", lambda cs, tm: (cs.add("weld_used"), tm.update({("cover_used", "scene"): True})), "the used cover missed the grey tray"),
        ("fresh_off", lambda cs, tm: cs.add("weld_well_2"), "a fresh cover came off the probe again"),
        ("off_bench", lambda cs, tm: _place("disposal_tray", (0.33, 0.70, task.TOP)), "the disposal_tray fell off the bench"),
    ]
    for label, trigger, expected_reason in cases:
        _, cut_set, touch_map, _ = _setup_mock_world(monkeypatch)
        trigger(cut_set, touch_map)
        task.step()
        assert task.STATE["zero"] == expected_reason, f"{label}: got {task.STATE['zero']!r}"

        # Restore a clean 1.0 state and step 25 times: score must stay permanently 0.0!
        touch_map.clear()
        cut_set.discard("weld_well_2")
        _place("disposal_tray", mk.built_pose("disposal_tray")[0])
        _put_used_in_tray(cut_set)
        _dock_thermometer_with_fresh0(cut_set)
        for _ in range(25):
            task.step()
        assert task.score(None) == 0.0, f"{label}: zero latch did not hold"


def test_submission_caps_if_present():
    tj = HERE / "task.json"
    if tj.exists():
        meta = json.loads(tj.read_text(encoding="utf-8"))
        assert len(meta["title"]) <= 120
        assert len(meta["context"]) <= 1000
        assert len(meta["fidelity"]) <= 500
        assert re.fullmatch(r"Dream-1 run: (pending|[0-9]*\.?[0-9]+); demo: 1\.0", meta["scores"]), meta["scores"]
        assert meta["category"] == "Impact on jobs"
    rm = HERE / "README.md"
    if rm.exists():
        text = rm.read_text(encoding="utf-8")
        assert len(text) <= 10_000
        assert "a different reading replaces the estimate" in text
