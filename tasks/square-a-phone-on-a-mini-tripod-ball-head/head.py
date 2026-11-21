"""Bilateral PhysX push-button ball-head mechanism for ``pixi_level`` (``MECHKIT_HELPER = True``).

Serial kinematic tree on free body ``tripod`` (zero closed loops, zero one-sided ``mk.push`` loads):
    tripod -> hinge("tilt", axis=(0, 1, 0)) -> c_tilt
           -> hinge("roll", axis=(1, 0, 0)) -> clamp --weld("phone_clamp")--> phone
    tripod -> slide("button", axis=(1, 0, 0), limits=(0, BUTTON_TRAVEL), spring=BUTTON_K)

Physical behaviour (Check 11 & Check 51):
- While ``button`` travel < ``PRESSED_FRACTION * BUTTON_TRAVEL`` (2.40 mm, 14.4 N), both ``tilt`` and ``roll`` carry a
  bilateral PhysX spring drive capped at ``LOCK_NM`` (1.50 N m), whose rest angle yields plastically only if external
  torque exceeds ``LOCK_NM``.
- While ``button`` is pressed >= 2.40 mm (up to 3.00 mm at ``BUTTON_N`` = 18.0 N; equal and opposite spring reaction on
  ``button`` and ``tripod``), the torque cap on ``tilt`` and ``roll`` drops to ``FREE_NM`` (0.02 N m) and their rest
  angles follow within ``FREE_NM / K_HEAD`` (0.019 deg). Unheld at the start (+10 deg, -20 deg), gravity torque
  (0.025 N m tilt, -0.051 N m roll) exceeds ``FREE_NM`` and flops the phone into ``+/- STOP_DEG`` (35 deg).
- When ``button`` springs back below 2.40 mm, ``LOCK_NM`` engages at the ball's current orientation, locking the phone
  where it stands. Within about 7 deg of level the free head's drag alone (``FREE_NM`` against 0.15 N m x sin lean)
  also holds a let-go phone.
- A torque past ``LOCK_NM`` on the locked head drags the rest angle along (plastic slip); ``slip_deg`` sums that slip per
  hinge over the trial, and ``task.py`` zeroes the trial once either sum passes ``LOCK_SLIP_MAX_DEG`` (0.2 deg).
"""

from __future__ import annotations

import math

import mechkit as mk
import parts as P
from mechkit import hinge, matrix_to_quat, slide, weld

MECHKIT_HELPER = True

HINGE_NAMES = ("tilt", "roll")
STATE: dict = {}


def declare(table_top: float) -> None:
    """Declare the ball-head hinges, phone-to-clamp weld and spring-loaded release button."""
    ball = P.ball_anchor_world(table_top)
    btn = P.button_built_pos(table_top)
    hinge(
        "tilt",
        "c_tilt",
        "tripod",
        anchor=ball,
        axis=(0.0, 1.0, 0.0),
        limits=(-P.STOP_RAD, P.STOP_RAD),
        spring=P.K_HEAD,
        rest=0.0,
        damping=P.D_HEAD_LOCK,
    )
    hinge(
        "roll",
        "clamp",
        "c_tilt",
        anchor=ball,
        axis=(1.0, 0.0, 0.0),
        limits=(-P.STOP_RAD, P.STOP_RAD),
        spring=P.K_HEAD,
        rest=0.0,
        damping=P.D_HEAD_LOCK,
    )
    weld("phone_clamp", "phone", "clamp")
    slide(
        "button",
        "button",
        "tripod",
        anchor=btn,
        axis=P.button_slide_axis(),
        limits=(0.0, P.BUTTON_TRAVEL),
        spring=P.BUTTON_K,
        rest=0.0,
        damping=P.BUTTON_D,
    )
    reset()


def reset() -> None:
    """Put the ball head at its locked tilted start (+10 deg tilt, -20 deg roll) and button at 0 mm."""
    STATE.clear()
    STATE.update(
        rest={"tilt": P.START_TILT_RAD, "roll": P.START_ROLL_RAD},
        unlocked=False,
        slipped={"tilt": False, "roll": False},
        slip_deg={"tilt": 0.0, "roll": 0.0},
        applied_rest={"tilt": None, "roll": None},
        applied_cap=None,
        applied_damp=None,
        max_button_mm=0.0,
        max_button_spring_n=0.0,
        max_right_button_n=0.0,
        max_right_head_n=0.0,
        max_left_phone_n=0.0,
        max_head_torque_nm={"tilt": 0.0, "roll": 0.0},
        profile=[],
    )
    if mk.MECH.environment is not None and "tilt" in mk.MECH.joint_prims:
        mk.set_joint("tilt", P.START_TILT_RAD)
        mk.set_joint("roll", P.START_ROLL_RAD)
        mk.set_joint("button", 0.0)
        p_phone, r_phone = mk.joint_pose(mk.MECH.joints["phone_clamp"], 0.0, mk._sim_pose("clamp"))
        mk.write_pose("phone", p_phone, matrix_to_quat(r_phone))
        for j in HINGE_NAMES:
            mk.set_drive(j, stiffness=P.K_HEAD, damping=P.D_HEAD_LOCK)
            mk.set_spring(j, rest=STATE["rest"][j], most=P.LOCK_NM)
            STATE["applied_rest"][j] = STATE["rest"][j]
        mk.set_spring("button", rest=0.0, most=1.0e6)
        STATE["applied_cap"] = P.LOCK_NM
        STATE["applied_damp"] = P.D_HEAD_LOCK


def button_travel_m() -> float:
    if "button" not in mk.MECH.travel:
        return 0.0
    return max(0.0, float(mk.joint("button")))


def is_unlocked() -> bool:
    """True only while ``button`` is pressed at or past ``PRESSED_FRACTION * BUTTON_TRAVEL`` (2.40 mm)."""
    return button_travel_m() >= P.BUTTON_UNLOCK_M - 1e-9


def locked_slip_deg() -> float:
    """Largest plastic slip (deg) either hinge has made while locked, summed over the trial."""
    return max(float(v) for v in STATE["slip_deg"].values())


def active_torque_cap_nm() -> float:
    return P.FREE_NM if is_unlocked() else P.LOCK_NM


def button_spring_force_n() -> float:
    """Bilateral spring reaction (N) carried by ``slide('button')`` between ``button`` and ``tripod`` (up to BUTTON_N)."""
    return P.BUTTON_K * min(P.BUTTON_TRAVEL, button_travel_m())


def head_torque_nm(name: str) -> float:
    """Bilateral spring torque (N m) carried by ``hinge(name)`` between ``clamp`` chain and ``tripod``."""
    if name not in mk.MECH.travel:
        return 0.0
    theta = float(mk.joint(name))
    rest = float(STATE.get("rest", {}).get(name, 0.0))
    cap = active_torque_cap_nm()
    raw = -P.K_HEAD * (theta - rest)
    return float(max(-cap, min(cap, raw)))


def _normal_force_sum(body_name: str, other: str) -> float:
    return float(sum(abs(c.force) for c in mk.touching(body_name, other)))


def _sync_sim(want_cap: float, want_damp: float) -> None:
    if mk.MECH.environment is None or "tilt" not in mk.MECH.joint_prims:
        return
    damp_changed = STATE["applied_damp"] is None or abs(want_damp - STATE["applied_damp"]) > 1e-9
    cap_changed = STATE["applied_cap"] is None or abs(want_cap - STATE["applied_cap"]) > 1e-9
    for j in HINGE_NAMES:
        want_r = float(STATE["rest"][j])
        r_changed = STATE["applied_rest"][j] is None or abs(want_r - STATE["applied_rest"][j]) > 1e-6
        if damp_changed:
            mk.set_drive(j, damping=want_damp)
        if r_changed or cap_changed:
            mk.set_spring(j, rest=want_r if r_changed else None, most=want_cap if cap_changed else None)
            STATE["applied_rest"][j] = want_r
    STATE["applied_cap"] = want_cap
    STATE["applied_damp"] = want_damp


def watch(events: list) -> None:
    """Step hook: updates lock/free state from ``button`` travel, advances elasto-plastic rest angles and logs forces."""
    now = mk.now()
    btn_m = button_travel_m()
    now_unlocked = is_unlocked()
    was_unlocked = bool(STATE.get("unlocked", False))

    if now_unlocked and not was_unlocked:
        events.append((now, f"ball head unlocked (button {1000 * btn_m:.2f} mm, {button_spring_force_n():.1f} N)"))
    elif was_unlocked and not now_unlocked:
        t_deg = math.degrees(float(mk.joint("tilt")))
        r_deg = math.degrees(float(mk.joint("roll")))
        events.append((now, f"ball head locked at tilt {t_deg:+.2f} deg, roll {r_deg:+.2f} deg"))
    STATE["unlocked"] = now_unlocked

    cap = P.FREE_NM if now_unlocked else P.LOCK_NM
    damp = P.D_HEAD_FREE if now_unlocked else P.D_HEAD_LOCK
    lag = cap / P.K_HEAD + (0.0 if now_unlocked else P.LOCK_SOLVER_SLACK_RAD)

    for j in HINGE_NAMES:
        theta = float(mk.joint(j))
        # Record torque carried before plastic rest update
        tau_mag = abs(max(-cap, min(cap, -P.K_HEAD * (theta - float(STATE["rest"][j])))))
        STATE["max_head_torque_nm"][j] = max(STATE["max_head_torque_nm"][j], tau_mag)
        if was_unlocked and not now_unlocked:
            # Spring clamp closes onto the ball at its present angle
            STATE["rest"][j] = theta
        else:
            old_r = float(STATE["rest"][j])
            new_r = min(theta + lag, max(theta - lag, old_r))
            if not now_unlocked:
                # Locked: any drag of the rest angle is slip past LOCK_NM; sum it, so slow slips count as well
                STATE["slip_deg"][j] += math.degrees(abs(new_r - old_r))
                if STATE["slip_deg"][j] > P.LOCK_SLIP_MAX_DEG + 1e-9 and not STATE["slipped"][j]:
                    events.append((now, f"locked {j} slipped {STATE['slip_deg'][j]:.2f} deg past {P.LOCK_NM:.2f} N m"))
                    STATE["slipped"][j] = True
            STATE["rest"][j] = new_r

    _sync_sim(cap, damp)

    btn_spring_n = button_spring_force_n()
    r_btn_n = _normal_force_sum("button", "right robot")
    r_head_n = _normal_force_sum("tripod", "right robot")
    l_phone_n = _normal_force_sum("phone", "left robot")

    STATE["max_button_mm"] = max(STATE["max_button_mm"], 1000.0 * btn_m)
    STATE["max_button_spring_n"] = max(STATE["max_button_spring_n"], btn_spring_n)
    STATE["max_right_button_n"] = max(STATE["max_right_button_n"], r_btn_n)
    STATE["max_right_head_n"] = max(STATE["max_right_head_n"], r_head_n)
    STATE["max_left_phone_n"] = max(STATE["max_left_phone_n"], l_phone_n)

    prof = STATE["profile"]
    if not prof or now - prof[-1]["t"] >= 0.499:
        prof.append(
            {
                "t": round(now, 2),
                "tilt_deg": round(math.degrees(float(mk.joint("tilt"))), 2),
                "roll_deg": round(math.degrees(float(mk.joint("roll"))), 2),
                "button_mm": round(1000.0 * btn_m, 2),
                "unlocked": now_unlocked,
                "button_spring_n": round(btn_spring_n, 2),
                "right_pad_button_n": round(r_btn_n, 2),
                "right_pad_head_n": round(r_head_n, 2),
                "left_pad_phone_n": round(l_phone_n, 2),
            }
        )
