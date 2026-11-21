"""Rule, mechanics, and Gate G1/G2 unit tests for ``mech/tasks/helmet_fit`` (pure CPU, no simulator)."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import checks as C  # noqa: E402
import mechanism as L  # noqa: E402
import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402
import task as T  # noqa: E402

CLEAR_GRIPPERS = {
    "left": ((0.18, +0.30, T.TOP + 0.25), 0.094),
    "right": ((0.18, -0.30, T.TOP + 0.25), 0.094),
}


def _reset_offline() -> None:
    T.begin(seed=0)
    L.reset()


def _q_tilt_for_deg(tilt_deg: float) -> float:
    return math.radians(tilt_deg - P.TILT_START_DEG)


def _q_dial_for_deg(dial_deg: float) -> float:
    return math.radians(dial_deg) - L.DIAL_MID_RAD


def _step_state(
    *,
    tilt_deg: float,
    dial_deg: float,
    band_force_n: float,
    steps: int = 22,
    dt: float = 0.05,
    grippers: dict | None = None,
    poses: dict | None = None,
    velocities: dict | None = None,
    extra_contacts: dict | None = None,
) -> float:
    t0 = float(T.STATE.get("last_time", 0.0))
    g = CLEAR_GRIPPERS if grippers is None else grippers
    for k in range(1, steps + 1):
        contacts = {("band", "head"): float(band_force_n)}
        if extra_contacts:
            contacts.update(extra_contacts)
        mk.set_state(
            poses=poses,
            velocities=velocities,
            joints={
                "tilt": _q_tilt_for_deg(tilt_deg),
                "dial": _q_dial_for_deg(dial_deg),
                "band": min(P.BAND_TRAVEL, P.BAND_LEAD * math.radians(max(0.0, dial_deg))),
            },
            grippers=g,
            contacts=contacts,
            time=t0 + k * dt,
        )
        T.step()
    return T.score(T.OBJECTS)


def test_parts_constants_and_serial_tree():
    _reset_offline()
    allowed_kinds = set(P.SOURCE_KINDS)
    assert allowed_kinds == {"maker", "standard", "listing", "estimate"}
    for row in P.KEY_FIGURES:
        assert row["kind"] in allowed_kinds, f"unexpected source kind in {row}"
        assert row.get("check", "").strip(), f"missing first-minutes check in {row}"
        if row["kind"] in {"maker", "standard", "listing"}:
            assert "https://" in row["source"], f"missing URL in {row}"
        else:
            assert row["source"].startswith("estimate: "), f"missing derivation in {row}"
    assert math.isclose(P.SHELL_MASS + P.DIAL_MASS + P.BAND_MASS, P.HELMET_MASS, abs_tol=1e-9)
    assert math.isclose(P.GAP_MIN, 0.015, abs_tol=1e-9)
    assert math.isclose(P.GAP_MAX, 0.035, abs_tol=1e-9)
    # The rocking back is TIP_BACK_NM in all: the weights give part of it, the push the rest, at every tilt
    weights = [P.weights_tip_back_nm(t) for t in np.linspace(P.TILT_MIN_DEG, P.TILT_START_DEG, 37)]
    assert 0.0 < min(weights) and max(weights) < P.TIP_BACK_NM - 0.01
    assert 0.009 < P.weights_tip_back_nm(0.0) < 0.012 and 0.022 < P.weights_tip_back_nm(P.TILT_START_DEG) < 0.026
    # Pure serial tree: 4 bodies, 4 joints, each body appears as child at most once (zero closed loops)
    assert set(mk.MECH.bodies) == {"head", "helmet", "dial", "band"}
    children = [j.child for j in mk.MECH.joints.values()]
    assert len(children) == len(set(children)) == 4


def test_idle_scores_zero():
    _reset_offline()
    s = _step_state(tilt_deg=P.TILT_START_DEG, dial_deg=0.0, band_force_n=0.0, steps=40)
    assert s == 0.0
    assert math.isclose(1000.0 * T.gap_m(), 1000.0 * P.GAP_START, abs_tol=0.2)


def test_tipped_back_and_dial_tight_scores_zero():
    _reset_offline()
    # Shortcut: tightening the dial while the helmet stays tipped back (gap 45.9 mm > 35 mm) -> 0.0
    s = _step_state(tilt_deg=P.TILT_START_DEG, dial_deg=280.0, band_force_n=30.0, steps=25)
    assert T.gap_m() > P.GAP_MAX
    assert s == 0.0


def test_level_without_dial_rocks_back_and_scores_zero():
    _reset_offline()
    # 1. Even at tilt = 0 deg, dial = 0 deg has dial_was_turned() == False -> 0.0
    s_untouched_dial = _step_state(tilt_deg=0.0, dial_deg=0.0, band_force_n=0.0, steps=25)
    assert s_untouched_dial == 0.0

    # 2. Quasi-static integration of open helmet let go at tilt = 0 deg: TIP_BACK_NM (0.04) > TILT_FRICTION_NM (0.01)
    # overcomes the capped spring and rocks the helmet back to its start stop (gap 45.9 mm > 35 mm -> 0.0).
    _reset_offline()
    _step_state(tilt_deg=0.0, dial_deg=20.0, band_force_n=0.0, steps=2)
    q_tilt = _q_tilt_for_deg(0.0)
    omega = 0.0
    inertia_eff = 0.0025  # kg m^2 about ear-strap axis
    dt_phys = 0.005
    for _ in range(400):
        t_hold = L.tilt_hold_nm(0.0)
        t_rest = float(L.STATE["tilt_rest"])
        tau_spring = -max(-t_hold, min(t_hold, P.K_TILT * (q_tilt - t_rest))) - P.D_TILT * omega
        tau_net = P.TIP_BACK_NM + tau_spring
        omega += (tau_net / inertia_eff) * dt_phys
        q_tilt = min(0.0, max(L.TILT_LOWER_RAD, q_tilt + omega * dt_phys))
        if q_tilt >= 0.0 and omega > 0.0:
            omega = 0.0
        lag = t_hold / P.K_TILT
        if q_tilt > t_rest + lag:
            L.STATE["tilt_rest"] = q_tilt - lag
    final_tilt_deg = P.TILT_START_DEG + math.degrees(q_tilt)
    assert final_tilt_deg >= P.TILT_START_DEG - 0.1
    s_rocked = _step_state(tilt_deg=final_tilt_deg, dial_deg=20.0, band_force_n=0.0, steps=25)
    assert s_rocked == 0.0


def test_over_eyes_scores_zero():
    _reset_offline()
    # Pushed forward to -5 deg stop: gap ~ 10.0 mm < GAP_MIN (15 mm) -> 0.0 even when clamped
    s = _step_state(tilt_deg=P.TILT_MIN_DEG, dial_deg=260.0, band_force_n=28.0, steps=25)
    assert T.gap_m() < P.GAP_MIN
    assert s == 0.0


def test_level_and_half_tight_scores_half():
    _reset_offline()
    # Level (gap = 20 mm), 8 clicks (5 clicks past touch), band_force = 14 N < CLAMP_N (24 N) -> 0.5
    s = _step_state(tilt_deg=0.0, dial_deg=160.0, band_force_n=14.0, steps=25)
    assert T.gap_in_fit_window()
    assert not T.helmet_clamped()
    assert s == 0.5


def test_level_and_clamped_scores_one_after_one_second_hold():
    _reset_offline()
    # Before 20 steps (1.0 s), score stays 0.0; at >= 20 steps (1.0 s), score reaches 1.0 (10.5 clicks: inside the bound)
    s_short = _step_state(tilt_deg=0.0, dial_deg=210.0, band_force_n=27.9, steps=15)
    assert s_short == 0.0
    s_full = _step_state(tilt_deg=0.0, dial_deg=210.0, band_force_n=27.9, steps=6)
    assert s_full == 1.0
    assert T.success(T.OBJECTS) is True


def test_overtightened_past_comfort_bound_scores_half():
    # The upper tightness check: clamped at the fit with the dial at most COMFORT_MAX_CLICKS (4) past the band's first
    # touch (PRETOUCH_CLICKS = 7 from the open stop, so 11 clicks, 220 deg) is 1.0; turned past it, still clamped (the
    # 10 N pull holds), it is 0.5. Tipped back, overtightened is 0.0 (the gap rule).
    assert P.COMFORT_MAX_DIAL_CLICKS == P.PRETOUCH_CLICKS + P.COMFORT_MAX_CLICKS == 11
    assert P.CLAMP_CLICKS < P.COMFORT_MAX_CLICKS < P.DIAL_TRAVEL_CLICKS - P.PRETOUCH_CLICKS  # full credit reachable, overtight too
    at_bound = P.COMFORT_MAX_DIAL_CLICKS * P.DIAL_CLICK_DEG
    _reset_offline()
    assert _step_state(tilt_deg=2.6, dial_deg=at_bound, band_force_n=41.0, steps=25) == 1.0
    assert T.within_comfort_bound() and T.helmet_clamped() and math.isclose(L.clicks_past_touch(), 4.0, abs_tol=1e-9)
    for over_deg, bf in ((at_bound + 5.0, 44.0), (260.0, 62.0), (P.DIAL_TRAVEL_DEG, 70.0)):
        _reset_offline()
        assert _step_state(tilt_deg=2.6, dial_deg=over_deg, band_force_n=bf, steps=25) == 0.5
        assert T.helmet_clamped() and not T.within_comfort_bound() and T.gap_in_fit_window()
        assert T.report()["within_comfort_bound"] is False
    _reset_offline()
    assert _step_state(tilt_deg=P.TILT_START_DEG, dial_deg=260.0, band_force_n=62.0, steps=25) == 0.0


def test_dial_tightens_one_way_and_loosens_only_by_hand():
    _reset_offline()
    # Tighten up to 260 deg (13 clicks): the rest follows the dial an elastic lag (turning torque / K_DIAL) behind it
    for deg in (60.0, 120.0, 180.0, 260.0):
        bf = max(0.0, P.K_BAND * (P.BAND_LEAD * math.radians(deg) - P.BAND_GAP_OPEN))
        _step_state(tilt_deg=0.0, dial_deg=deg, band_force_n=bf, steps=2)
    locked_rest = float(L.STATE["dial_rest"])
    locked_deg = math.degrees(locked_rest + L.DIAL_MID_RAD)
    assert math.isclose(locked_deg, 260.0 - math.degrees(L.dial_turn_torque_nm(bf) / P.K_DIAL), abs_tol=1e-6)
    assert math.isclose(float(L.STATE["dial_cap"]), L.dial_turn_torque_nm(bf))  # turning: the cap is what turns it

    # Left alone, nothing turns it back: the band's press does not drive the dial, so the rest stays, and the drive's
    # cap until the next step is the pawl's (a reverse torque under it cannot spin the dial back within a step)
    _step_state(tilt_deg=0.0, dial_deg=locked_deg, band_force_n=27.9, steps=10)
    assert math.isclose(float(L.STATE["dial_rest"]), locked_rest, abs_tol=1e-12)
    assert float(L.STATE["dial_cap"]) == P.DIAL_HOLD_NM

    # Turned back with 0.22 N m (under the pawl's DIAL_HOLD_NM): an elastic 1.05 deg, not a click lost
    tau_rev = 0.22
    assert tau_rev < P.DIAL_HOLD_NM
    deflected_deg = locked_deg - math.degrees(tau_rev / P.K_DIAL)
    _step_state(tilt_deg=0.0, dial_deg=deflected_deg, band_force_n=27.9, steps=5)
    assert math.isclose(float(L.STATE["dial_rest"]), locked_rest, abs_tol=1e-12)
    assert float(L.STATE["dial_cap"]) == P.DIAL_HOLD_NM
    # Pushed forward under the turning torque it does not advance either
    _step_state(tilt_deg=0.0, dial_deg=locked_deg + 0.5 * math.degrees(L.dial_turn_torque_nm(27.9) / P.K_DIAL),
                band_force_n=27.9, steps=5)
    assert math.isclose(float(L.STATE["dial_rest"]), locked_rest, abs_tol=1e-12)

    # Turned back by hand past the pawl (to 200 deg): it loosens, the rest follows a DIAL_HOLD_NM lag behind, and the
    # band's rest goes back with it; let go there, it stays loosened
    _step_state(tilt_deg=0.0, dial_deg=200.0, band_force_n=10.0, steps=3)
    loosened = math.degrees(float(L.STATE["dial_rest"]) + L.DIAL_MID_RAD)
    assert math.isclose(loosened, 200.0 + math.degrees(P.DIAL_HOLD_NM / P.K_DIAL), abs_tol=1e-6)
    assert L.band_rest_m() < P.BAND_LEAD * math.radians(locked_deg)
    _step_state(tilt_deg=0.0, dial_deg=loosened, band_force_n=10.0, steps=5)
    assert math.isclose(math.degrees(float(L.STATE["dial_rest"]) + L.DIAL_MID_RAD), loosened, abs_tol=1e-6)


def test_10n_upward_pull_clamped_under_5mm_open_and_half_over_5mm():
    tau_10n = P.PULL_MOMENT_NM  # 10 N at the 115 mm front edge plus the rocking back: 1.19 N m, the one figure
    assert math.isclose(tau_10n, P.PULL_TEST_N * P.BRIM_ARM + P.TIP_BACK_NM, abs_tol=1e-3)

    # 1. Clamped at 13 clicks (band_force = 27.9 N >= CLAMP_N = 24.0 N):
    hold_clamped = L.tilt_hold_nm(27.9)
    assert hold_clamped > tau_10n
    edge_move_clamped_m = P.BRIM_ARM * (tau_10n / P.K_TILT)
    assert edge_move_clamped_m < P.PULL_MAX_MOVE_M  # ~2.28 mm < 5.0 mm

    # 2. Open (band_force = 0 N) and half-tight (band_force = 14 N): both slip past the cap to the start stop (>> 5 mm)
    for bf in (0.0, 14.0):
        hold = L.tilt_hold_nm(bf)
        assert tau_10n > hold
        edge_move_slip_m = P.BRIM_ARM * math.sin(math.radians(P.TILT_START_DEG))
        assert edge_move_slip_m > P.PULL_MAX_MOVE_M  # ~46.8 mm > 5.0 mm


def test_whole_trial_zero_rules_head_shift_and_face_touch():
    # 1. Board shift > 10 mm latches zero for the rest of the trial
    _reset_offline()
    shifted_head = (
        np.array(T.HEAD_START_POS) + np.array([0.012, 0.0, 0.0]),
        np.eye(3),
    )
    _step_state(tilt_deg=P.TILT_START_DEG, dial_deg=0.0, band_force_n=0.0, steps=1, poses={"head": shifted_head})
    assert T.STATE["zero_reason"] is not None
    s_after_shift = _step_state(tilt_deg=0.0, dial_deg=260.0, band_force_n=28.0, steps=25)
    assert s_after_shift == 0.0

    # 2. An arm touching the face (nose, shape 4) latches zero, whatever force is read (the simulator reads an arm
    #    link's contact force as 0); a touch on the neck (shape 0) does not.
    _reset_offline()
    neck_touch = mk.Contact(
        other="left robot",
        shape=0,
        other_shape=-1,
        point=np.array([0.28, 0.09, T.TOP + 0.012]),
        normal=np.array([0.0, 0.0, 1.0]),
        separation=-0.0001,
        force=0.0,
    )
    _step_state(tilt_deg=0.0, dial_deg=220.0, band_force_n=28.0, steps=22, extra_contacts={"head": [neck_touch]})
    assert T.STATE["zero_reason"] is None
    assert T.score(T.OBJECTS) == 1.0

    face_touch = mk.Contact(
        other="right robot",
        shape=P.FACE_SHAPE_INDICES[0],
        other_shape=-1,
        point=np.array([0.28, 0.088, T.TOP + 0.056]),
        normal=np.array([0.0, -1.0, 0.0]),
        separation=-0.0001,
        force=0.0,
    )
    _step_state(tilt_deg=0.0, dial_deg=220.0, band_force_n=28.0, steps=1, extra_contacts={"head": [face_touch]})
    assert T.STATE["zero_reason"] is not None
    assert _step_state(tilt_deg=0.0, dial_deg=220.0, band_force_n=28.0, steps=25) == 0.0

    # 3. A contact point 1 mm off the face (inside the colliders' report distance) is no touch
    _reset_offline()
    near = mk.Contact(other="left robot", shape=P.FACE_SHAPE_INDICES[1], other_shape=-1,
                      point=np.array([0.25, 0.083, T.TOP + 0.062]), normal=np.array([0.0, -1.0, 0.0]), separation=0.001, force=0.0)
    _step_state(tilt_deg=0.0, dial_deg=220.0, band_force_n=28.0, steps=22, extra_contacts={"head": [near]})
    assert T.STATE["zero_reason"] is None and T.score(T.OBJECTS) == 1.0


def test_dial_at_its_stop_without_clamp_scores_below_one():
    # The score reads the band's press on the head, not the dial's angle: a dial turned to its stop whose band does
    # not press with CLAMP_N (here 12 N) is 0.5 at the level fit, 0.0 tipped back (the gap is outside the window).
    _reset_offline()
    s = _step_state(tilt_deg=2.6, dial_deg=P.DIAL_TRAVEL_DEG, band_force_n=12.0, steps=25)
    assert T.gap_in_fit_window() and L.dial_clicks() >= P.DIAL_TRAVEL_CLICKS - 1e-6
    assert s == 0.5
    _reset_offline()
    assert _step_state(tilt_deg=P.TILT_START_DEG, dial_deg=P.DIAL_TRAVEL_DEG, band_force_n=0.0, steps=25) == 0.0


def test_band_touches_mid_travel_and_clamps_by_two_thirds():
    # The band is drawn in linearly over the dial's whole travel (no band travel left over at the stop).
    assert math.isclose(P.BAND_LEAD * math.radians(P.DIAL_TRAVEL_DEG), P.BAND_TRAVEL, rel_tol=1e-12)
    assert P.DIAL_TRAVEL_DEG <= 360.0 - 1e-9 and L.DIAL_MID_RAD <= math.pi  # a sprung hinge within half a turn
    # Level, the band meets the head near the middle of the travel and reaches CLAMP_N by two thirds of it, with
    # clicks to spare before the stop.
    assert 0.4 <= P.PRETOUCH_CLICKS / P.DIAL_TRAVEL_CLICKS <= 0.6
    assert P.TOTAL_CLAMP_CLICKS / P.DIAL_TRAVEL_CLICKS <= 2.0 / 3.0 + 1e-9
    assert P.DIAL_TRAVEL_CLICKS - P.TOTAL_CLAMP_CLICKS >= 5
    assert math.isclose(P.BAND_GAP_OPEN, P.PRETOUCH_CLICKS * math.radians(P.DIAL_CLICK_DEG) * P.BAND_LEAD)
    # The back of the head leaves exactly BAND_GAP_OPEN to the band's pad at level, and the nape the same tipped back.
    piv = np.array([0.0, P.PIVOT_Y, P.PIVOT_Z])
    occiput = (np.array(P.OCCIPUT_CENTER_LOCAL), P.OCCIPUT_SPHERE_R)
    nape = (np.array(P.NAPE_CENTER_LOCAL), P.NAPE_SPHERE_R)
    pad_half = 0.009  # the pad's inner face reaches 9 mm either side of its middle in the tilt plane (band_shapes)
    assert [s for s in P.head_shapes() if s.kind == "sphere" and s.solid and math.isclose(s.dims[0], P.NAPE_SPHERE_R)
            and np.allclose(s.pos, nape[0])], "the nape is a solid sphere of the head form"

    def gap_to(sphere, tilt_deg: float) -> float:
        """Sphere to the pad's inner face (a flat face turning with the helmet), band slide at 0."""
        centre, radius = sphere
        r = P.rx(math.radians(tilt_deg))
        pad = piv + r @ np.array([0.0, P.BAND_DY_LEVEL, P.BAND_DZ_LEVEL])
        inward = r @ np.array([0.0, *P.BAND_AXIS_LEVEL])
        along = r @ np.array([0.0, -P.BAND_AXIS_LEVEL[1], P.BAND_AXIS_LEVEL[0]])
        rel = centre - pad
        normal, side = float(rel @ inward), float(rel @ along)
        return math.hypot(normal, max(0.0, abs(side) - pad_half)) - radius

    def gap_at(tilt_deg: float) -> float:
        return min(gap_to(occiput, tilt_deg), gap_to(nape, tilt_deg))

    assert math.isclose(gap_to(occiput, 0.0), P.BAND_GAP_OPEN, abs_tol=1e-9)
    assert gap_to(nape, 0.0) > P.BAND_GAP_OPEN + 1e-3 and gap_to(nape, 2.6) > gap_to(occiput, 2.6)  # level: the occiput
    assert gap_at(2.6) - P.BAND_GAP_OPEN < 0.2e-3  # the demo's fit (gap 25 mm) moves the touch by under 0.6 click
    # Tightened tipped back, the band meets the nape after PRETOUCH_CLICKS and presses CLAMP_N after TOTAL_CLAMP_CLICKS:
    # it clamps the helmet tipped back (score 0, gap outside the window).
    assert math.isclose(gap_to(nape, P.TILT_START_DEG), P.BAND_GAP_OPEN, abs_tol=1e-9)
    assert gap_to(occiput, P.TILT_START_DEG) > P.BAND_GAP_OPEN + 1e-3
    # At any tilt of the joint's travel the band meets the head within a click of PRETOUCH_CLICKS: the dial clamps the
    # helmet wherever it is held, at least 5 clicks before its stop.
    click_m = math.radians(P.DIAL_CLICK_DEG) * P.BAND_LEAD
    worst_extra = max(gap_at(t) for t in np.linspace(P.TILT_MIN_DEG, P.TILT_START_DEG, 361)) - P.BAND_GAP_OPEN
    assert 0.0 <= worst_extra < click_m
    assert P.TOTAL_CLAMP_CLICKS + worst_extra / click_m <= P.DIAL_TRAVEL_CLICKS - 5


def test_gripper_clearance_and_rest_requirements():
    _reset_offline()
    near_grippers = {
        "left": (tuple(P.front_top_world(T.TOP, 0.0) + np.array([0.0, 0.0, 0.010])), 0.0),
        "right": CLEAR_GRIPPERS["right"],
    }
    assert _step_state(tilt_deg=0.0, dial_deg=260.0, band_force_n=28.0, steps=25, grippers=near_grippers) == 0.0

    _reset_offline()
    moving = {"helmet": (np.array([0.0, 0.0, 0.0]), np.array([0.35, 0.0, 0.0]))}
    assert _step_state(tilt_deg=0.0, dial_deg=260.0, band_force_n=28.0, steps=25, velocities=moving) == 0.0


def test_checklist_52_sizes_and_texts_agree():
    import json

    readme = (HERE / "README.md").read_text()
    source = (HERE / "task.py").read_text()
    parts_src = (HERE / "parts.py").read_text()
    spec_raw = (HERE / "task.json").read_text()
    spec = json.loads(spec_raw)
    assert len(readme) <= 10_000  # the reviewer's 40,000-character excerpt: README, then task.py with score()
    assert len(source) <= 15_000 and "def score(" in source
    assert len(spec["title"]) <= 120 and len(spec["context"]) <= 1000 and len(spec["fidelity"]) <= 500
    assert spec["scores"] == "Dream-1 run: pending; demo: 1.0"
    assert spec["instruction"] == T.INSTRUCTION and "no more than four clicks" in T.INSTRUCTION
    assert spec["category"] == "Societal relatability"
    assert spec["run_limit"].startswith(f"{T.RUN_LIMIT_STEPS:,} steps ({T.RUN_LIMIT_STEPS / 20:g} s at 20 Hz)")
    for text in (spec["scoring"], readme):  # the same numbers in task.py (parts.py), task.json and the README
        for number in (f"{1000 * P.GAP_MIN:.0f} to {1000 * P.GAP_MAX:.0f} mm", f"{P.CLAMP_N:.0f} N", f"{P.PULL_TEST_N:.0f} N",
                       f"{1000 * P.GRIPPER_CLEAR_M:.0f} mm", f"{1000 * P.MAX_HEAD_SHIFT_M:.0f} mm", "1.0 s",
                       f"{P.COMFORT_MAX_CLICKS} clicks past", f"{P.COMFORT_MAX_DIAL_CLICKS} clicks",
                       f"{P.COMFORT_MAX_DIAL_CLICKS * P.DIAL_CLICK_DEG:.0f} deg", f"EN 960 size {P.HEADFORM_EN960_SIZE}"):
            assert number in text, number
    for doc_name, doc_txt in (("README.md", readme), ("task.json", spec_raw), ("parts.py", parts_src)):
        low = doc_txt.lower()
        for banned in ("assumed", "until measured", "owner's reading", "owner's finger", "hand-tested", "bench-verified"):
            assert banned not in low, f"{banned!r} in {doc_name}"


def test_offline_ik_and_25mm_face_clearance():
    res = C.run_offline_checks()
    assert res["pass"] is True
    assert res["min_face_clearance_mm"] >= 25.0
    assert res["grasp_points_sep_mm"] >= 220.0
