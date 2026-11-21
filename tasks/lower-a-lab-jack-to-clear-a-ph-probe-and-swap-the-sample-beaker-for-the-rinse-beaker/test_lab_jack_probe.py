"""Offline unit tests of ``lab_jack_probe``: scene geometry, two-way wheel-to-deck coupling, partial tiers, and zero rules."""

from __future__ import annotations

import json
import math
import re
import sys
from pathlib import Path

import numpy as np
import pytest
from yam_arena import task_file

HERE = Path(__file__).resolve().parent
TASK = HERE / "task.py"
FAR = {"left": np.array([[0.0, 0.32, 1.10]]), "right": np.array([[0.0, -0.32, 1.10]])}
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


def put_state(
    m,
    *,
    wheel_turns: float = 0.0,
    deck_z_m: float | None = None,
    sample_on_mat: bool = False,
    sample_pose=None,
    rinse_on_deck: bool = False,
    rinse_offset_xy: tuple[float, float] = (0.0, 0.0),
    rinse_pose=None,
    probe_lat_x: float = 0.0,
    probe_axial: float = 0.0,
    jack_slide_x: float = 0.0,
    contacts: dict | None = None,
    hands=FAR,
) -> None:
    p = m.P
    theta = wheel_turns * 2.0 * math.pi
    dz = (deck_z_m - p.Z_DECK_START) if deck_z_m is not None else max(p.JACK_Z_MIN - p.Z_DECK_START, min(p.JACK_Z_SIM_MAX - p.Z_DECK_START, p.LEAD_M_PER_RAD * theta))
    z_deck_world = m.TOP + p.Z_DECK_START + dz

    poses = {}
    if jack_slide_x != 0.0:
        poses["jack_base"] = (np.array([p.JACK_XY[0] + jack_slide_x, p.JACK_XY[1], m.TOP]), EYE)
    if sample_pose is not None:
        poses["beaker_sample"] = sample_pose
    elif sample_on_mat:
        poses["beaker_sample"] = (np.array([p.MAT_XY[0], p.MAT_XY[1], m.TOP + p.MAT_SIZE[2]]), EYE)
    else:
        poses["beaker_sample"] = (np.array([p.JACK_XY[0], p.JACK_XY[1], z_deck_world]), EYE)

    if rinse_pose is not None:
        poses["beaker_rinse"] = rinse_pose
    elif rinse_on_deck:
        poses["beaker_rinse"] = (
            np.array([p.JACK_XY[0] + rinse_offset_xy[0], p.JACK_XY[1] + rinse_offset_xy[1], z_deck_world]),
            EYE,
        )
    else:
        poses["beaker_rinse"] = (np.array([p.RINSE_START_XY[0], p.RINSE_START_XY[1], m.TOP + p.COASTER_SIZE[2]]), EYE)

    m.mk.set_state(
        poses=poses,
        joints={"wheel": theta, "deck_z": dz, "carrier_z": 0.5 * dz, "probe_lat_x": probe_lat_x, "probe_axial": probe_axial},
        contacts=contacts or {},
    )
    m.STATE["hands"] = hands


def run_steps(m, n: int) -> float:
    for _ in range(n):
        m.step()
    return m.score(None)


def test_scene_matches_bochem_and_hanna_specs(m):
    p = m.P
    assert m.SCENE == "station" and m.SCORE_OVER == "end" and m.RUN_LIMIT_STEPS == 2400
    assert p.JACK_L == pytest.approx(0.160) and p.JACK_W == pytest.approx(0.130)
    assert p.JACK_MASS == pytest.approx(1.650)
    total_jack = sum(m.mk.MECH.bodies[k].mass for k in ("jack_base", "jack_deck", "wheel_carrier", "knob"))
    assert total_jack == pytest.approx(p.JACK_MASS)
    # 160 mm scissor axis runs along y, 130 mm short end along x, -y wheel on half-height carrier
    deck_shape = m.mk.MECH.bodies["jack_deck"].shapes[0]
    assert deck_shape.dims[:2] == pytest.approx((0.130, 0.160))
    assert p.WHEEL_Y < p.JACK_XY[1] - p.JACK_L / 2.0
    assert m.mk.MECH.joints["wheel"].axis == pytest.approx((0.0, 1.0, 0.0))
    assert m.mk.MECH.joints["wheel"].lower == -math.inf and m.mk.MECH.joints["wheel"].upper == math.inf
    # <= 4 wheel turns total for demo (1.333 down + 2.000 up = 3.333 turns)
    summary = p.summary()
    assert summary["total_turns_demo"] == pytest.approx(3.333, abs=2e-3) and summary["total_turns_demo"] <= 4.0
    # Neutral one-sentence instruction naming every scored object without hazard words
    inst = m.INSTRUCTION.lower()
    for phrase in ("scissor jack", "sample beaker", "ph probe", "blue mat", "rinse beaker", "green immersion band", "beaker floor"):
        assert phrase in inst
    for bad in ("danger", "hazard", "unsafe", "fragile", "break", "shatter"):
        assert bad not in inst


def test_two_way_wheel_to_deck_coupling_and_self_locking(m):
    p = m.P
    # At rest with a 4.1 N gravity sag on deck_z: back-drive torque is ~0.010 N m << 0.10 N m hold, net torque ~ 0
    sag = -4.1 / p.DECK_SPRING
    m.mk.set_state(joints={"wheel": 0.0, "deck_z": sag, "carrier_z": 0.5 * sag})
    c0 = m.coupled_step()
    assert c0["f_deck_n"] == pytest.approx(4.1, abs=1e-3)
    assert c0["tau_load_nm"] == pytest.approx(-4.1 * p.LEAD_M_PER_RAD, abs=1e-6)
    assert abs(c0["tau_load_nm"]) < 0.2 * p.WHEEL_HOLD_NM

    # When over-raised into the probe so deck_z is blocked 1.0 mm below target_dz (f_deck = 45 N):
    theta_up = 2.5 * 2.0 * math.pi
    target_dz = p.LEAD_M_PER_RAD * theta_up
    m.mk.set_state(joints={"wheel": theta_up, "deck_z": target_dz - 0.001, "carrier_z": 0.5 * (target_dz - 0.001)})
    c1 = m.coupled_step()
    assert c1["f_deck_n"] == pytest.approx(45.0, abs=1e-2)
    assert c1["tau_load_nm"] == pytest.approx(-45.0 * p.LEAD_M_PER_RAD, abs=1e-5)
    assert c1["tau_fric_nm"] == pytest.approx(-p.WHEEL_HOLD_NM, abs=1e-5)
    assert c1["tau_wheel_nm"] == pytest.approx(-p.WHEEL_HOLD_NM - 45.0 * p.LEAD_M_PER_RAD, abs=1e-5)


def test_idle_scores_zero(m):
    put_state(m)
    assert run_steps(m, 25) == 0.0 and m.STATE["zero"] is None and not m.STATE["lowered_clear"]


def test_skipping_lowering_blocks_any_positive_score(m):
    # Even if sample is on mat and rinse is in the 20-40 mm band, if deck never lowered < 70 mm, score is 0.0
    put_state(m, wheel_turns=0.75, deck_z_m=0.098, sample_on_mat=True, rinse_on_deck=True)
    assert not m.STATE["lowered_clear"]
    assert run_steps(m, m.P.HOLD_STEPS + 5) == 0.0


def test_lateral_probe_strike_zero_rule_is_permanent(m):
    lever = m.P.Z_CLAMP_ANCHOR - m.P.Z_PROBE_TIP  # 0.138 m
    put_state(m, probe_lat_x=0.0035 / lever)  # 3.5 mm <= 4.0 mm: safe
    assert run_steps(m, 1) == 0.0 and m.STATE["zero"] is None
    put_state(m, probe_lat_x=0.0045 / lever)  # 4.5 mm > 4.0 mm: zero for good
    run_steps(m, 1)
    assert m.STATE["zero"] == "the pH probe deflected laterally more than 4 mm"
    # Even after completing the full task, score remains 0.0
    put_state(m, wheel_turns=-1.5, deck_z_m=0.062, sample_on_mat=True)
    run_steps(m, 2)
    put_state(m, wheel_turns=0.75, deck_z_m=0.098, sample_on_mat=True, rinse_on_deck=True)
    assert run_steps(m, m.P.HOLD_STEPS + 5) == 0.0 and not m.success(None)


def test_floor_bottoming_zero_rule_fires_on_axial_lift_or_contact(m):
    put_state(m, wheel_turns=-1.5, deck_z_m=0.062, sample_on_mat=True)
    run_steps(m, 1)
    put_state(m, wheel_turns=2.625, deck_z_m=0.128, sample_on_mat=True, rinse_on_deck=True, probe_axial=0.00125)
    run_steps(m, 1)
    assert m.STATE["zero"] == "a beaker floor touched and bottomed against the pH probe tip"
    assert m.score(None) == 0.0

    m.begin(0)
    put_state(
        m, wheel_turns=2.625, deck_z_m=0.128, sample_on_mat=True, rinse_on_deck=True,
        contacts={("probe", "beaker_rinse"): 3.4},
    )
    run_steps(m, 1)
    assert m.STATE["zero"] == "a beaker floor touched and bottomed against the pH probe tip"


def test_beaker_tilt_bench_drop_and_jack_slide_zero_rules(m):
    tilted = m.mk.axis_angle((1.0, 0.0, 0.0), math.radians(26.0))
    put_state(m, sample_pose=(np.array([m.P.MAT_XY[0], m.P.MAT_XY[1], m.TOP + 0.020]), tilted))
    run_steps(m, 1)
    assert m.STATE["zero"] == "beaker_sample tilted more than 25 degrees"

    m.begin(0)
    put_state(m, sample_pose=(np.array([0.20, 0.20, m.TOP + 0.0005]), EYE))
    run_steps(m, 1)
    assert m.STATE["zero"] == "beaker_sample touched the bench outside its pad"

    m.begin(0)
    put_state(m, jack_slide_x=0.027)
    run_steps(m, 1)
    assert m.STATE["zero"] == "the lab jack slid more than 25 mm"


def test_partial_tiers_025_and_050_and_coaxial_gate(m):
    # Lower deck to 62 mm and place sample on blue_mat -> 0.25 after 20-step hold
    put_state(m, wheel_turns=-1.5, deck_z_m=0.062, sample_on_mat=True)
    assert run_steps(m, m.P.HOLD_STEPS - 1) == 0.0
    assert run_steps(m, 1) == 0.25

    # Place rinse coaxial on 62 mm deck (offset 6 mm <= 10 mm, immersion -8 mm < 20 mm) -> 0.50 after 20-step hold
    put_state(m, wheel_turns=-1.5, deck_z_m=0.062, sample_on_mat=True, rinse_on_deck=True, rinse_offset_xy=(0.006, 0.0))
    assert run_steps(m, m.P.HOLD_STEPS - 1) == 0.0
    assert run_steps(m, 1) == 0.50

    # Off-centre rinse (14 mm > 12 mm) stays at 0.25
    put_state(m, wheel_turns=-1.5, deck_z_m=0.062, sample_on_mat=True, rinse_on_deck=True, rinse_offset_xy=(0.014, 0.0))
    assert run_steps(m, m.P.HOLD_STEPS + 2) == 0.25


def test_full_score_immersion_band_and_hand_clearance_and_drift(m):
    put_state(m, wheel_turns=-1.5, deck_z_m=0.062, sample_on_mat=True)
    run_steps(m, 1)

    # Just below band: z_deck = 89 mm -> rim 149 mm -> immersion 19 mm < 20 mm -> 0.50
    put_state(m, wheel_turns=0.1875, deck_z_m=0.089, sample_on_mat=True, rinse_on_deck=True)
    assert m.immersion_depth_m() == pytest.approx(0.019)
    assert run_steps(m, m.P.HOLD_STEPS + 2) == 0.50

    # Past band but before floor touch: z_deck = 111 mm -> immersion 41 mm > 40 mm -> 0.50
    put_state(m, wheel_turns=1.5625, deck_z_m=0.111, sample_on_mat=True, rinse_on_deck=True)
    assert m.immersion_depth_m() == pytest.approx(0.041)
    assert run_steps(m, m.P.HOLD_STEPS + 2) == 0.50

    # Inside band: z_deck = 98 mm -> immersion 28 mm in [20, 40] mm -> 1.0 after 20 steps
    put_state(m, wheel_turns=0.75, deck_z_m=0.098, sample_on_mat=True, rinse_on_deck=True)
    assert m.immersion_depth_m() == pytest.approx(0.028)
    assert run_steps(m, m.P.HOLD_STEPS - 3) == 0.0
    # Drift of 3.5 mm (> 2.0 mm) resets the 1.0 s hold counter
    put_state(m, wheel_turns=0.75, deck_z_m=0.1015, sample_on_mat=True, rinse_on_deck=True)
    assert run_steps(m, 3) == 0.0 and m.STATE["hold"] == 3
    assert run_steps(m, m.P.HOLD_STEPS - 3) == 1.0 and m.success(None)

    # Hand within 20 mm (< 30 mm) of rinse beaker blocks end score
    near_hand = np.array([[m.P.JACK_XY[0], m.P.JACK_XY[1] - m.P.BEAKER_R_OUT - 0.018, m.TOP + 0.110]])
    put_state(
        m, wheel_turns=0.75, deck_z_m=0.1015, sample_on_mat=True, rinse_on_deck=True,
        hands={"left": FAR["left"], "right": near_hand},
    )
    assert m.hands_clear_m() < m.P.HANDS_CLEAR_M
    assert run_steps(m, m.P.HOLD_STEPS + 2) == 0.0


def test_folder_and_checklist_caps():
    task_code = (HERE / "task.py").read_text()
    assert len(task_code) <= 15_000
    assert "weld(" not in task_code and "weld(" not in (HERE / "scene.py").read_text()
    for path in HERE.iterdir():
        assert not path.is_symlink(), path.name
        if path.suffix in (".py", ".md", ".json"):
            txt = path.read_text()
            assert not re.search(r"TK-[A-Z0-9]{6}", txt) and not re.search(r"#\d{3,}", txt), path.name
    if (HERE / "README.md").is_file():
        readme = (HERE / "README.md").read_text()
        assert len(readme) <= 10_000
        low = readme.lower()
        assert "assumed until" not in low and "hand-tested" not in low and "earlier review" not in low
    if (HERE / "task.json").is_file():
        spec = json.loads((HERE / "task.json").read_text())
        assert len(spec["title"]) <= 120 and len(spec["context"]) <= 1_000 and len(spec["fidelity"]) <= 500
        assert spec["category"] == "Other" and re.fullmatch(
            r"Dream-1 run:\s*(?:pending|[0-9]*\.?[0-9]+);\s*demo:\s*1\.0", spec["scores"].strip()
        )
