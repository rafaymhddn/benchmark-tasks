#!/usr/bin/env python3
"""Gate G2 live-PhysX mechanics trials for ``mech/tasks/helmet_fit``:

    scripts/rta simpy <id> mech/tasks/helmet_fit/trials.py outputs/<id> [only=abcdefg]

Runs back-to-back episodes in one simulator session (``only=`` picks some):
  (a) ``open_rocks_back``: Left hand tips the open helmet forward from the start tilt to the fit (as the demo,
      gap about 26 mm), holds 0.4 s, and releases; verifies it rocks back to its start stop and scores 0.0.
  (b) ``dial_twist_tips_unheld``:
      - (b1) With ``tip_back_enabled=False`` (no rocking back), the unheld helmet stays at the fit for 0.6 s,
        then turning the dial tips the unheld helmet backward via ``DIAL_TILT_SHARE`` alone.
      - (b2) With full physics (``tip_back_enabled=True``), Right hand approaches and turns the dial while
        nobody holds the helmet; verifies the helmet tips back to its start stop and scores 0.0.
  (c) ``clamped_in_bound_then_overtight``: Left hand tips the helmet to the fit and holds it while the dial is turned
      past the clamp but inside the comfort bound (to 10.5 clicks, 3.5 past touch), then Left releases and holds
      clear >= 1.2 s -> verifies score == 1.0, band_force >= CLAMP_N, 10 N upward front-edge pull moves edge < 5.0 mm,
      and -0.20 N m reverse torque on the dial does not turn the pawl back. Then, hands clear, the dial is turned on
      to 13 clicks (6 past touch, over the bound of 4): still clamped (the pull holds) but scores 0.5.
  (d) ``half_tight_and_open_pull``: a half-tight helmet at the fit (just past first touch) stays when the hands
      release (score == 0.5), then slips > 5.0 mm under a 10 N upward front-edge pull (and an open helmet at the
      fit also moves > 5.0 mm under the pull).
  (e) ``band_press_vs_clicks`` (check 51): held at the fit, the dial is driven click by click from 5 clicks to its
      stop by a known torque, which is taken off before each reading; the band's press on the head is read at
      rest after every click. Passes when the band first touches within a click of PRETOUCH_CLICKS and reaches
      CLAMP_N within a click of TOTAL_CLAMP_CLICKS.
  (f) ``tightened_tipped_back``: nobody holds the helmet at its start tilt; a 10 N push down on the front edge tips
      the open helmet forward. Then the dial is driven click by click to 12 clicks (as the demo): the band closes on
      the nape after PRETOUCH_CLICKS and clamps the helmet tipped back, so it scores 0.0 and the same push moves the
      edge under 5 mm.
  (g) ``zero_rules``: live-PhysX check of both whole-trial zero rules:
      - (g1) ``face_touch``: Left hand touches the nose (face zone) before fitting and clamping the helmet at the fit;
        verifies ``face_touches >= 1`` latches ``zero_reason`` and keeps ``score == 0.0``.
      - (g2) ``head_shift``: with ``head_mount`` cut (tape peeled), Left hand pushes the side of the helmet/skull
        clear of the face until ``head`` shifts > 10 mm; verifies ``head form moved`` latches ``zero_reason`` and
        ``score == 0.0``.
"""

from __future__ import annotations

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import json
import math
from pathlib import Path
import sys

import numpy as np
from yam_arena.constants import READY
from yam_arena.task_file import TABLE_TOP

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import checks as C  # noqa: E402
import demo as D  # noqa: E402
import demo_kit as kit  # noqa: E402
import parts as P  # noqa: E402

TOP = TABLE_TOP["station"]
GAP_MIN_MM, GAP_MAX_MM = 1000.0 * P.GAP_MIN, 1000.0 * P.GAP_MAX  # the fit window


def _pack_action(ql: np.ndarray, gl: float, qr: np.ndarray, gr: float):
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack

    return Action(data=pack(np.array([*ql, gl], dtype=float), np.array([*qr, gr], dtype=float)))


def _interp_arm(emb, ql0, gl0, ql1, gl1, qr0, gr0, qr1, gr1, seconds: float, hz: float = 20.0):
    n = max(1, round(seconds * hz))
    last = None
    for k in range(1, n + 1):
        s = k / n
        sm = s * s * (3.0 - 2.0 * s)
        ql = (1.0 - sm) * ql0 + sm * ql1
        qr = (1.0 - sm) * qr0 + sm * qr1
        gl = (1.0 - sm) * gl0 + sm * gl1
        gr = (1.0 - sm) * gr0 + sm * gr1
        last = emb.step(_pack_action(ql, gl, qr, gr))
    return last


def _drive_dial(emb, l_mod, action, to_deg: float, excess_nm: float = 0.004, steps: int = 160, tilt_deg: float = 0.0) -> None:
    """Turn the dial up to ``to_deg`` by a known torque on it, each step just over what it takes to turn now (the
    ratchet's turning torque at the band's present press, plus ``excess_nm``), then take the torque off. The dial's
    drive caps its damping with its spring (PhysX maxForce), so a fixed torque well over the turning torque spins the
    dial to its stop within a step or two (run h4t1e: 0.13 N m took it from 0 to 299 deg). The torque is about the
    dial's axis with the helmet at ``tilt_deg``."""
    d_ax = P.dial_axis_world(tilt_deg)
    for _ in range(steps):
        if l_mod.dial_deg() >= to_deg:
            break
        l_mod.set_test_load(dial_torque=tuple((l_mod.dial_turn_torque_nm() + excess_nm) * d_ax), tip_back_enabled=True)
        emb.step(action)
    l_mod.set_test_load(tip_back_enabled=True)


def _solve_left_arc() -> dict[str, np.ndarray]:
    q_ready = np.array(READY, dtype=float)
    p_pre, r_pre = C.left_press_pose(P.TILT_START_DEG, lift_m=0.045)
    q_pre, _, _, _, _ = C.solve("left", p_pre, r_pre, allow_flip=False)

    p_start, r_start = C.left_press_pose(P.TILT_START_DEG, lift_m=0.002)
    q_start, _, _, _, _ = C.solve("left", p_start, r_start, start=q_pre, allow_flip=False)

    p_mid, r_mid = C.left_press_pose(0.5 * P.TILT_START_DEG, lift_m=-0.001)
    q_mid, _, _, _, _ = C.solve("left", p_mid, r_mid, start=q_start, allow_flip=False)

    # Held at the fit as the demo holds it: fingers 0.8 mm over the rim's top at 2.6 deg, the helmet rocked back onto
    # them (about 3 deg, gap about 26 mm, run h4t2h). Pressed 2.5 mm into the rim at level it went to -2.5 deg, under
    # the window (run h4t1g).
    p_fit, r_fit = C.left_press_pose(D.TARGET_TILT_DEG, lift_m=D.HOLD_LIFT_M)
    q_fit, _, _, _, _ = C.solve("left", p_fit, r_fit, start=q_mid, allow_flip=False)

    p_up, r_up = C.left_press_pose(D.TARGET_TILT_DEG, lift_m=0.085)
    q_up, _, _, _, _ = C.solve("left", p_up, r_up, start=q_fit, allow_flip=False)
    return {"ready": q_ready, "pre": q_pre, "t_start": q_start, "t_mid": q_mid, "t_fit": q_fit, "up": q_up}


def _press_to_fit(emb, q_left: dict[str, np.ndarray], qr0, gr0: float, qr1=None, gr1: float | None = None) -> None:
    """Left hand over the forehead and down the rim's arc from the start tilt to the fit, then holding there (the
    demo's way; a straight path from above missed the rim, run h4t1g). The right arm goes from qr0 to qr1 meanwhile."""
    qr1 = qr0 if qr1 is None else qr1
    gr1 = gr0 if gr1 is None else gr1
    _interp_arm(emb, q_left["ready"], 1.0, q_left["pre"], 0.0, qr0, gr0, qr1, gr1, 0.9)
    _interp_arm(emb, q_left["pre"], 0.0, q_left["t_start"], 0.0, qr1, gr1, qr1, gr1, 0.4)
    _interp_arm(emb, q_left["t_start"], 0.0, q_left["t_mid"], 0.0, qr1, gr1, qr1, gr1, 0.5)
    _interp_arm(emb, q_left["t_mid"], 0.0, q_left["t_fit"], 0.0, qr1, gr1, qr1, gr1, 0.5)
    _interp_arm(emb, q_left["t_fit"], 0.0, q_left["t_fit"], 0.0, qr1, gr1, qr1, gr1, 0.4)


def _touch_and_clamp(readings: list[dict]) -> tuple[float | None, float | None]:
    """First touch: where the line through the first three readings over 1 N meets zero (readings fall on whole
    clicks); CLAMP_N: the first reading at or over it."""
    pressed = [r for r in readings if r["band_n"] > 1.0][:3]
    touch = None
    if len(pressed) >= 2:
        slope, icpt = np.polyfit([r["clicks"] for r in pressed], [r["band_n"] for r in pressed], 1)
        touch = round(float(-icpt / slope), 2)
    clamp = next((r["clicks"] for r in readings if r["band_n"] >= P.CLAMP_N), None)
    return touch, clamp


def run_trial_a(emb, m, l_mod, mk_mod, q_left: dict[str, np.ndarray]) -> dict:
    """(a) Open helmet tipped forward by Left hand to the fit and let go -> rocks back to its start stop (score 0)."""
    from inspect_robots import Scene

    emb.reset(Scene(id="trial_a", instruction=m.INSTRUCTION), seed=0)
    qr = np.array(READY, dtype=float)

    # Approach and press the helmet down from the start tilt to the fit (gap about 26 mm)
    _press_to_fit(emb, q_left, qr, 1.0)

    held_tilt_deg = round(l_mod.tilt_deg(), 2)
    held_gap_mm = round(1000.0 * m.gap_m(), 2)

    # Release Left hand up and back to READY; wait 2.2 s for open helmet to rock back to its start stop
    _interp_arm(emb, q_left["t_fit"], 0.0, q_left["up"], 0.0, qr, 1.0, qr, 1.0, 0.6)
    _interp_arm(emb, q_left["up"], 0.0, q_left["ready"], 1.0, qr, 1.0, qr, 1.0, 0.8)
    res = _interp_arm(emb, q_left["ready"], 1.0, q_left["ready"], 1.0, qr, 1.0, qr, 1.0, 2.0)

    final_tilt_deg = round(l_mod.tilt_deg(), 2)
    final_gap_mm = round(1000.0 * m.gap_m(), 2)
    final_score = float(res.info.get("score", 0.0))
    ok = bool(GAP_MIN_MM <= held_gap_mm <= GAP_MAX_MM and final_tilt_deg >= P.TILT_START_DEG - 1.5 and final_gap_mm > GAP_MAX_MM
              and final_score == 0.0)
    return {
        "pass": ok,
        "held_tilt_deg": held_tilt_deg,
        "held_gap_mm": held_gap_mm,
        "after_release_tilt_deg": final_tilt_deg,
        "after_release_gap_mm": final_gap_mm,
        "score": final_score,
    }


def run_trial_b(emb, m, l_mod, mk_mod, q_left: dict[str, np.ndarray]) -> dict:
    """(b) Dial turned with nobody holding the helmet -> tips back via DIAL_TILT_SHARE."""
    from inspect_robots import Scene

    qr = np.array(READY, dtype=float)
    ql = np.array(READY, dtype=float)

    # (b1) Pure dial-twist isolation: tip_back_enabled=False so ONLY DIAL_TILT_SHARE acts.
    # Press the helmet to the fit with Left hand, lift Left hand cleanly away, verify 0.6 s stationary while unheld,
    # then turn the dial and verify DIAL_TILT_SHARE tips the unheld helmet backward!
    emb.reset(Scene(id="trial_b1", instruction=m.INSTRUCTION), seed=0)
    l_mod.set_test_load(tip_back_enabled=False)
    _press_to_fit(emb, q_left, qr, 1.0)
    _interp_arm(emb, q_left["t_fit"], 0.0, q_left["up"], 0.0, qr, 1.0, qr, 1.0, 0.5)
    _interp_arm(emb, q_left["up"], 0.0, ql, 1.0, qr, 1.0, qr, 1.0, 0.6)

    # 0.6 s idle unheld with tip_back_enabled=False: verifies the helmet stays at the fit when the dial is NOT turned
    tilt_at_release_deg = round(l_mod.tilt_deg(), 2)
    _interp_arm(emb, ql, 1.0, ql, 1.0, qr, 1.0, qr, 1.0, 0.6)
    tilt_before_turn_deg = round(l_mod.tilt_deg(), 2)
    gap_before_turn_mm = round(1000.0 * m.gap_m(), 2)

    # Now drive dial gently (+0.035 N m -> ~2.5 rad/s) up to ~75 deg so DIAL_TILT_SHARE tips it before band clamps!
    d_ax = P.dial_axis_world(0.0)
    l_mod.set_test_load(dial_torque=tuple(0.035 * d_ax), tip_back_enabled=False)
    for _ in range(40):
        emb.step(_pack_action(ql, 1.0, qr, 1.0))
        if l_mod.dial_deg() >= 75.0:
            break
    l_mod.set_test_load(tip_back_enabled=False)
    _interp_arm(emb, ql, 1.0, ql, 1.0, qr, 1.0, qr, 1.0, 0.8)
    tilt_after_pure_twist_deg = round(l_mod.tilt_deg(), 2)
    dial_turned_b1_deg = round(l_mod.dial_deg(), 1)

    # (b2) Full physics (tip_back_enabled=True): Left holds at the fit & lets go as Right pinches & turns dial unheld
    emb.reset(Scene(id="trial_b2", instruction=m.INSTRUCTION), seed=0)
    fit = D.TARGET_TILT_DEG
    p_ret, r_ret = C.right_dial_pose(roll_deg=-35.0, retreat_m=0.032, tilt_deg=fit)
    qr_ret, _, _, _, _ = C.solve("right", p_ret, r_ret, allow_flip=False)
    p_in, r_in = C.right_dial_pose(roll_deg=-35.0, retreat_m=0.0, tilt_deg=fit)
    qr_in, _, _, _, _ = C.solve("right", p_in, r_in, start=qr_ret, allow_flip=False)
    p_turn, r_turn = C.right_dial_pose(roll_deg=+35.0, retreat_m=0.0, tilt_deg=fit)
    qr_turn, _, _, _, _ = C.solve("right", p_turn, r_turn, start=qr_in, allow_flip=False)

    grip_open = 0.60
    grip_pinch = max(0.0, (P.DIAL_D - 0.005) / C.MAX_JAW)
    _press_to_fit(emb, q_left, qr, 1.0, qr_ret, grip_open)
    held_b2_gap_mm = round(1000.0 * m.gap_m(), 2)
    # Left lets go right as Right pinches and turns dial on the unheld helmet!
    _interp_arm(emb, q_left["t_fit"], 0.0, q_left["up"], 0.0, qr_ret, grip_open, qr_in, grip_pinch, 0.5)
    _interp_arm(emb, q_left["up"], 0.0, ql, 1.0, qr_in, grip_pinch, qr_turn, grip_pinch, 0.8)
    res_b2 = _interp_arm(emb, ql, 1.0, ql, 1.0, qr_turn, grip_open, qr, 1.0, 1.6)

    tilt_b2_deg = round(l_mod.tilt_deg(), 2)
    score_b2 = float(res_b2.info.get("score", 0.0))
    ok = bool(
        abs(tilt_before_turn_deg - tilt_at_release_deg) <= 0.4
        and GAP_MIN_MM <= gap_before_turn_mm <= GAP_MAX_MM
        and dial_turned_b1_deg >= 30.0
        and tilt_after_pure_twist_deg - tilt_before_turn_deg >= 2.0
        and GAP_MIN_MM <= held_b2_gap_mm <= GAP_MAX_MM
        and tilt_b2_deg >= P.TILT_START_DEG - 4.0
        and score_b2 == 0.0
    )
    return {
        "pass": ok,
        "b1_pure_twist": {
            "tilt_at_release_deg": tilt_at_release_deg,
            "tilt_before_turn_deg": tilt_before_turn_deg,
            "gap_before_turn_mm": gap_before_turn_mm,
            "dial_turned_deg": dial_turned_b1_deg,
            "tilt_after_turn_deg": tilt_after_pure_twist_deg,
            "delta_tilt_deg": round(tilt_after_pure_twist_deg - tilt_before_turn_deg, 2),
        },
        "b2_full_unheld": {
            "held_gap_mm": held_b2_gap_mm,
            "final_tilt_deg": tilt_b2_deg,
            "score": score_b2,
        },
    }


def run_trial_c(emb, m, l_mod, mk_mod, q_left: dict[str, np.ndarray]) -> dict:
    """(c) Held at the fit, turned past CLAMP_CLICKS inside the comfort bound -> 1.0, 10 N pull (<5 mm) & pawl check;
    then turned on past the bound -> clamped but 0.5."""
    from inspect_robots import Scene

    emb.reset(Scene(id="trial_c", instruction=m.INSTRUCTION), seed=0)
    qr = np.array(READY, dtype=float)

    # 1. Left hand presses the helmet forward to the fit (gap about 26 mm) and holds
    _press_to_fit(emb, q_left, qr, 1.0)

    # 2. Tighten the dial past the clamp but inside the comfort bound (to 10.5 clicks, 210 deg) while Left holds the fit
    d_ax = P.dial_axis_world(0.0)
    _drive_dial(emb, l_mod, _pack_action(q_left["t_fit"], 0.0, qr, 1.0), 210.0)
    _interp_arm(emb, q_left["t_fit"], 0.0, q_left["t_fit"], 0.0, qr, 1.0, qr, 1.0, 0.3)

    # 3. Release Left hand to READY and hold clear for 1.4 s (28 steps >= 20 steps) -> score == 1.0
    _interp_arm(emb, q_left["t_fit"], 0.0, q_left["up"], 0.0, qr, 1.0, qr, 1.0, 0.5)
    _interp_arm(emb, q_left["up"], 0.0, q_left["ready"], 1.0, qr, 1.0, qr, 1.0, 0.7)
    res_hold = _interp_arm(emb, q_left["ready"], 1.0, q_left["ready"], 1.0, qr, 1.0, qr, 1.0, 1.4)

    score_clamped = float(res_hold.info.get("score", 0.0))
    gap_before_pull_mm = 1000.0 * m.gap_m()
    dial_before_rev_deg = l_mod.dial_deg()
    band_force_clamped_n = round(l_mod.band_force_n(), 2)
    tilt_hold_clamped_nm = round(l_mod.tilt_hold_nm(), 3)

    # 4. Live PhysX 10 N upward front-edge pull for 1.0 s -> front edge moves < 5.0 mm
    l_mod.set_test_load(front_edge_pull_n=P.PULL_TEST_N, tip_back_enabled=True)
    _interp_arm(emb, q_left["ready"], 1.0, q_left["ready"], 1.0, qr, 1.0, qr, 1.0, 1.0)
    gap_under_10n_mm = 1000.0 * m.gap_m()
    edge_move_10n_mm = abs(gap_under_10n_mm - gap_before_pull_mm)
    l_mod.set_test_load(tip_back_enabled=True)

    # 5. Live PhysX reverse pawl torque (-0.20 N m < 0.24 N m) for 1.0 s -> dial does not turn back
    l_mod.set_test_load(dial_torque=tuple(-0.20 * d_ax), tip_back_enabled=True)
    _interp_arm(emb, q_left["ready"], 1.0, q_left["ready"], 1.0, qr, 1.0, qr, 1.0, 1.0)
    dial_under_rev_deg = l_mod.dial_deg()
    l_mod.set_test_load(tip_back_enabled=True)
    _interp_arm(emb, q_left["ready"], 1.0, q_left["ready"], 1.0, qr, 1.0, qr, 1.0, 0.3)
    dial_after_rev_deg = l_mod.dial_deg()
    dial_loss_deg = max(0.0, dial_before_rev_deg - min(dial_under_rev_deg, dial_after_rev_deg))

    # 6. Overtightened: hands clear, the dial turned on to 13 clicks (260 deg, 6 past touch > 4), then 1.4 s at rest
    rest = _pack_action(q_left["ready"], 1.0, qr, 1.0)
    _drive_dial(emb, l_mod, rest, 260.0, tilt_deg=l_mod.tilt_deg())
    res_over = _interp_arm(emb, q_left["ready"], 1.0, q_left["ready"], 1.0, qr, 1.0, qr, 1.0, 1.4)
    score_over = float(res_over.info.get("score", 0.0))
    over = {"dial_deg": round(l_mod.dial_deg(), 1), "clicks_past_touch": round(l_mod.clicks_past_touch(), 2),
            "band_force_n": round(l_mod.band_force_n(), 2), "tilt_hold_nm": round(l_mod.tilt_hold_nm(), 3),
            "gap_mm": round(1000.0 * m.gap_m(), 2), "clamped": m.helmet_clamped(),
            "within_comfort_bound": m.within_comfort_bound(), "score": score_over}

    ok = bool(
        score_clamped == 1.0
        and GAP_MIN_MM <= gap_before_pull_mm <= GAP_MAX_MM
        and band_force_clamped_n >= P.CLAMP_N
        and edge_move_10n_mm < 1000.0 * P.PULL_MAX_MOVE_M
        and dial_loss_deg < 2.0
        and over["clamped"] and not over["within_comfort_bound"] and GAP_MIN_MM <= over["gap_mm"] <= GAP_MAX_MM
        and score_over == 0.5
    )
    return {
        "pass": ok,
        "score": score_clamped,
        "clicks_past_touch": round((dial_before_rev_deg / P.DIAL_CLICK_DEG) - P.PRETOUCH_CLICKS, 2),
        "dial_deg": round(dial_before_rev_deg, 1),
        "clicks": round(dial_before_rev_deg / P.DIAL_CLICK_DEG, 2),
        "band_force_n": band_force_clamped_n,
        "tilt_hold_nm": tilt_hold_clamped_nm,
        "gap_before_pull_mm": round(gap_before_pull_mm, 2),
        "gap_under_10n_pull_mm": round(gap_under_10n_mm, 2),
        "clamped_10n_edge_move_mm": round(edge_move_10n_mm, 2),
        "pawl_reverse_0p20nm_loss_deg": round(dial_loss_deg, 2),
        "overtight": over,
    }


def run_trial_d(emb, m, l_mod, mk_mod, q_left: dict[str, np.ndarray]) -> dict:
    """(d) Half-tight helmet at the fit scores 0.5 at rest and moves >5 mm under 10 N pull; so does an open one."""
    from inspect_robots import Scene

    ql = np.array(READY, dtype=float)
    qr = np.array(READY, dtype=float)

    # 1. Open helmet at the fit (placed by Left hand with tip_back_enabled=False, then subjected to 10 N upward pull)
    emb.reset(Scene(id="trial_d_open", instruction=m.INSTRUCTION), seed=0)
    l_mod.set_test_load(tip_back_enabled=False)
    _press_to_fit(emb, q_left, qr, 1.0)
    _interp_arm(emb, q_left["t_fit"], 0.0, q_left["up"], 0.0, qr, 1.0, qr, 1.0, 0.5)
    _interp_arm(emb, q_left["up"], 0.0, ql, 1.0, qr, 1.0, qr, 1.0, 0.5)
    gap0_open_mm = 1000.0 * m.gap_m()
    l_mod.set_test_load(front_edge_pull_n=P.PULL_TEST_N, tip_back_enabled=True)
    _interp_arm(emb, ql, 1.0, ql, 1.0, qr, 1.0, qr, 1.0, 0.8)
    l_mod.set_test_load(tip_back_enabled=True)
    open_move_10n_mm = abs(1000.0 * m.gap_m() - gap0_open_mm)

    # 2. Half-tight helmet: Left holds it at the fit while the dial turns to 165 deg (8.25 clicks, just past first touch)
    emb.reset(Scene(id="trial_d_half", instruction=m.INSTRUCTION), seed=0)
    _press_to_fit(emb, q_left, qr, 1.0)
    _drive_dial(emb, l_mod, _pack_action(q_left["t_fit"], 0.0, qr, 1.0), 165.0)
    _interp_arm(emb, q_left["t_fit"], 0.0, q_left["up"], 0.0, qr, 1.0, qr, 1.0, 0.5)
    _interp_arm(emb, q_left["up"], 0.0, ql, 1.0, qr, 1.0, qr, 1.0, 0.6)
    res_half = _interp_arm(emb, ql, 1.0, ql, 1.0, qr, 1.0, qr, 1.0, 1.4)

    score_half = float(res_half.info.get("score", 0.0))
    bf_half = round(l_mod.band_force_n(), 2)
    dial_half_deg = round(l_mod.dial_deg(), 1)
    gap_half_before_mm = 1000.0 * m.gap_m()

    l_mod.set_test_load(front_edge_pull_n=P.PULL_TEST_N, tip_back_enabled=True)
    _interp_arm(emb, ql, 1.0, ql, 1.0, qr, 1.0, qr, 1.0, 1.0)
    l_mod.set_test_load(tip_back_enabled=True)
    half_move_10n_mm = abs(1000.0 * m.gap_m() - gap_half_before_mm)

    hold_10n_band_n = (P.PULL_MOMENT_NM - P.TILT_FRICTION_NM) / (P.BAND_MU * P.BAND_ARM)  # 20.7 N holds the pull's moment
    ok = bool(
        open_move_10n_mm > 1000.0 * P.PULL_MAX_MOVE_M
        and score_half == 0.5
        and 2.0 <= bf_half < hold_10n_band_n
        and half_move_10n_mm > 1000.0 * P.PULL_MAX_MOVE_M
    )
    return {
        "pass": ok,
        "open_10n_edge_move_mm": round(open_move_10n_mm, 2),
        "half_tight_dial_deg": dial_half_deg,
        "half_tight_band_force_n": bf_half,
        "half_tight_gap_mm": round(gap_half_before_mm, 2),
        "half_tight_score": score_half,
        "half_tight_10n_edge_move_mm": round(half_move_10n_mm, 2),
    }


def run_trial_e(emb, m, l_mod, mk_mod, q_left: dict[str, np.ndarray]) -> dict:
    """(e) The band's press on the head against the dial's clicks, read at rest, from 5 clicks to the dial's stop."""
    from inspect_robots import Scene

    emb.reset(Scene(id="trial_e", instruction=m.INSTRUCTION), seed=0)
    qr = np.array(READY, dtype=float)
    _press_to_fit(emb, q_left, qr, 1.0)
    hold = _pack_action(q_left["t_fit"], 0.0, qr, 1.0)
    readings = []
    for clicks in range(5, P.DIAL_TRAVEL_CLICKS + 1):
        _drive_dial(emb, l_mod, hold, clicks * P.DIAL_CLICK_DEG - 0.5, excess_nm=0.002, steps=60)
        for _ in range(8):  # the known torque is off: the pawl holds the dial while the band settles
            emb.step(hold)
        readings.append({"clicks": round(l_mod.dial_clicks(), 2), "band_n": round(l_mod.band_force_n(), 2),
                         "band_slide_mm": round(1000.0 * l_mod.band_slide_m(), 3), "tilt_deg": round(l_mod.tilt_deg(), 2),
                         "turn_nm": round(l_mod.dial_turn_torque_nm(), 4)})
    touch, clamp = _touch_and_clamp(readings)
    pressed = [r for r in readings if r["band_n"] > 1.0]
    k_eff = None
    if len(pressed) >= 2:  # N per mm of band rest (BAND_LEAD x dial) past touch, least squares
        x = np.array([P.BAND_LEAD * math.radians(r["clicks"] * P.DIAL_CLICK_DEG) * 1000.0 for r in pressed])
        y = np.array([r["band_n"] for r in pressed])
        k_eff = round(float(np.polyfit(x, y, 1)[0]), 2)
    ok = bool(touch is not None and clamp is not None and abs(touch - P.PRETOUCH_CLICKS) <= 1.0
              and abs(clamp - P.TOTAL_CLAMP_CLICKS) <= 1.0 and readings[-1]["band_n"] >= P.CLAMP_N)
    return {"pass": ok, "first_touch_clicks": touch, "clamp_n_clicks": clamp, "k_eff_n_per_mm": k_eff,
            "at_stop": readings[-1], "readings": readings}


def run_trial_f(emb, m, l_mod, mk_mod, q_left: dict[str, np.ndarray]) -> dict:
    """(f) Tightened while tipped back: the band closes on the nape and clamps the helmet there (score 0)."""
    from inspect_robots import Scene

    ready = np.array(READY, dtype=float)
    rest = _pack_action(ready, 1.0, ready, 1.0)

    def hold(steps: int):
        res = None
        for _ in range(steps):
            res = emb.step(rest)
        return res

    def push_move_mm() -> float:  # 10 N down at the front edge's centre for 1.0 s (the pull test's mirror)
        gap0 = 1000.0 * m.gap_m()
        l_mod.set_test_load(front_edge_pull_n=-P.PULL_TEST_N, tip_back_enabled=True)
        hold(20)
        moved = abs(1000.0 * m.gap_m() - gap0)
        l_mod.set_test_load(tip_back_enabled=True)
        return moved

    # 1. The open helmet at its start: the push tips it forward
    emb.reset(Scene(id="trial_f_open", instruction=m.INSTRUCTION), seed=0)
    hold(10)
    open_tilt_deg = round(l_mod.tilt_deg(), 2)
    open_push_move_mm = push_move_mm()

    # 2. Nobody holds it; the dial is driven click by click to 12 clicks, each press read at rest
    emb.reset(Scene(id="trial_f_tight", instruction=m.INSTRUCTION), seed=0)
    hold(10)
    readings = []
    for clicks in range(5, 13):
        _drive_dial(emb, l_mod, rest, clicks * P.DIAL_CLICK_DEG - 0.5, excess_nm=0.002, steps=60, tilt_deg=P.TILT_START_DEG)
        hold(8)
        readings.append({"clicks": round(l_mod.dial_clicks(), 2), "band_n": round(l_mod.band_force_n(), 2),
                         "tilt_deg": round(l_mod.tilt_deg(), 2)})
    res = hold(28)  # 1.4 s at rest, both hands at READY
    score = float(res.info.get("score", 0.0))
    tilt_deg, gap_mm = round(l_mod.tilt_deg(), 2), round(1000.0 * m.gap_m(), 2)
    band_n, hold_nm = round(l_mod.band_force_n(), 2), round(l_mod.tilt_hold_nm(), 3)
    clamped_push_move_mm = push_move_mm()
    touch, clamp = _touch_and_clamp(readings)
    ok = bool(score == 0.0 and band_n >= P.CLAMP_N and tilt_deg >= P.TILT_START_DEG - 0.5 and gap_mm > GAP_MAX_MM
              and touch is not None and abs(touch - P.PRETOUCH_CLICKS) <= 1.0
              and clamp is not None and abs(clamp - P.TOTAL_CLAMP_CLICKS) <= 1.0
              and clamped_push_move_mm < 1000.0 * P.PULL_MAX_MOVE_M < open_push_move_mm)
    return {"pass": ok, "open_tilt_deg": open_tilt_deg, "open_10n_push_edge_move_mm": round(open_push_move_mm, 2),
            "score": score, "tilt_deg": tilt_deg, "gap_mm": gap_mm, "band_force_n": band_n, "tilt_hold_nm": hold_nm,
            "first_touch_clicks": touch, "clamp_n_clicks": clamp,
            "clamped_10n_push_edge_move_mm": round(clamped_push_move_mm, 2), "readings": readings}


def run_trial_g(emb, m, l_mod, mk_mod, q_left: dict[str, np.ndarray]) -> dict:
    """(g) Live-PhysX check of both whole-trial zero rules: (g1) arm touching face, (g2) head form moved > 10 mm."""
    from inspect_robots import Scene

    ql = np.array(READY, dtype=float)
    qr = np.array(READY, dtype=float)

    # 1. Face touch (shape 4 = nose): Left touches the nose, then fits and clamps the helmet -> latched 0.0
    emb.reset(Scene(id="trial_g_face", instruction=m.INSTRUCTION), seed=0)
    _, r_face = C.left_press_pose(0.0)
    low_in_g = kit.lowest_point(r_face)
    p_fpre = np.array([0.280, 0.132, TOP + 0.086], dtype=float) - r_face @ low_in_g
    p_ftouch = np.array([0.280, 0.090, TOP + 0.064], dtype=float) - r_face @ low_in_g
    q_fpre, _, _, _, _ = C.solve("left", p_fpre, r_face, allow_flip=False)
    q_ftouch, _, _, _, _ = C.solve("left", p_ftouch, r_face, start=q_fpre, allow_flip=False)

    _interp_arm(emb, ql, 1.0, q_fpre, 0.0, qr, 1.0, qr, 1.0, 0.7)
    _interp_arm(emb, q_fpre, 0.0, q_ftouch, 0.0, qr, 1.0, qr, 1.0, 0.5)
    _interp_arm(emb, q_ftouch, 0.0, q_ftouch, 0.0, qr, 1.0, qr, 1.0, 0.25)
    face_zero_reason = m.STATE.get("zero_reason")
    face_touch_steps = int(m.STATE.get("face_touches", 0))

    # Retreat from face and complete a full clamped fit to prove the zero latch overrides a valid end posture
    _interp_arm(emb, q_ftouch, 0.0, q_fpre, 0.0, qr, 1.0, qr, 1.0, 0.4)
    _interp_arm(emb, q_fpre, 0.0, q_left["pre"], 0.0, qr, 1.0, qr, 1.0, 0.5)
    _interp_arm(emb, q_left["pre"], 0.0, q_left["t_start"], 0.0, qr, 1.0, qr, 1.0, 0.4)
    _interp_arm(emb, q_left["t_start"], 0.0, q_left["t_mid"], 0.0, qr, 1.0, qr, 1.0, 0.5)
    _interp_arm(emb, q_left["t_mid"], 0.0, q_left["t_fit"], 0.0, qr, 1.0, qr, 1.0, 0.5)
    _drive_dial(emb, l_mod, _pack_action(q_left["t_fit"], 0.0, qr, 1.0), 240.0)
    _interp_arm(emb, q_left["t_fit"], 0.0, q_left["up"], 0.0, qr, 1.0, qr, 1.0, 0.5)
    _interp_arm(emb, q_left["up"], 0.0, ql, 1.0, qr, 1.0, qr, 1.0, 0.7)
    res_g1 = _interp_arm(emb, ql, 1.0, ql, 1.0, qr, 1.0, qr, 1.0, 1.4)
    score_g1 = float(res_g1.info.get("score", 0.0))
    clamped_g1 = bool(m.helmet_clamped() and m.gap_in_fit_window())

    # 2. Head shift > 10 mm: cut head_mount (tape peeled) and push the -x side of the skull/helmet along +x clear of face
    emb.reset(Scene(id="trial_g_shift", instruction=m.INSTRUCTION), seed=0)
    mk_mod.cut("head_mount")
    p_spre = np.array([0.168, 0.020, TOP + 0.140], dtype=float) - r_face @ low_in_g
    p_spush = np.array([0.238, 0.020, TOP + 0.140], dtype=float) - r_face @ low_in_g
    q_spre, _, _, _, _ = C.solve("left", p_spre, r_face, allow_flip=False)
    q_spush, _, _, _, _ = C.solve("left", p_spush, r_face, start=q_spre, allow_flip=False)

    _interp_arm(emb, ql, 1.0, q_spre, 0.0, qr, 1.0, qr, 1.0, 0.7)
    _interp_arm(emb, q_spre, 0.0, q_spush, 0.0, qr, 1.0, qr, 1.0, 0.6)
    res_g2 = _interp_arm(emb, q_spush, 0.0, ql, 1.0, qr, 1.0, qr, 1.0, 0.8)
    shift_zero_reason = m.STATE.get("zero_reason")
    max_shift_mm = round(float(m.STATE.get("max_head_shift_mm", 0.0)), 2)
    shift_face_steps = int(m.STATE.get("face_touches", 0))
    score_g2 = float(res_g2.info.get("score", 0.0))

    ok_g1 = bool(
        face_touch_steps >= 1
        and face_zero_reason == "an arm touched the head form's face (eyes, nose)"
        and clamped_g1
        and score_g1 == 0.0
    )
    ok_g2 = bool(
        shift_face_steps == 0
        and max_shift_mm > 1000.0 * P.MAX_HEAD_SHIFT_M
        and isinstance(shift_zero_reason, str)
        and shift_zero_reason.startswith("head form moved")
        and score_g2 == 0.0
    )
    return {
        "pass": bool(ok_g1 and ok_g2),
        "g1_face_touch": {
            "pass": ok_g1,
            "face_touch_steps": face_touch_steps,
            "zero_reason": face_zero_reason,
            "clamped_in_fit_window": clamped_g1,
            "score": score_g1,
        },
        "g2_head_shift": {
            "pass": ok_g2,
            "max_head_shift_mm": max_shift_mm,
            "face_touch_steps": shift_face_steps,
            "zero_reason": shift_zero_reason,
            "score": score_g2,
        },
    }


def main() -> int:
    out_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "trials_out"
    out_dir.mkdir(parents=True, exist_ok=True)
    only = next((a.split("=", 1)[1] for a in sys.argv[2:] if a.startswith("only=")), "abcdefg")

    q_left = _solve_left_arc()
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    import mechanism as l_mod
    import mechkit as mk_mod

    trials = {"a": ("a_open_rocks_back", run_trial_a), "b": ("b_dial_twist_tips_unheld", run_trial_b),
              "c": ("c_clamped_in_bound_then_overtight", run_trial_c), "d": ("d_half_tight_and_open_pull", run_trial_d),
              "e": ("e_band_press_vs_clicks", run_trial_e), "f": ("f_tightened_tipped_back", run_trial_f),
              "g": ("g_zero_rules", run_trial_g)}
    results = {name: run(emb, m, l_mod, mk_mod, q_left) for key, (name, run) in trials.items() if key in only}
    summary = {"pass": all(r["pass"] for r in results.values()), **results}
    (out_dir / "trials_summary.json").write_text(json.dumps(summary, indent=2))
    print("G2_TRIALS " + json.dumps(summary, indent=2), flush=True)
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
