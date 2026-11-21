"""Bilateral serial mechanism for the child's bike helmet fit task (MECHKIT_HELPER module).

Serial kinematic tree (zero closed loops):
    None (bench) --weld("head_mount")--> head -> hinge("tilt") -> helmet
                                                    ├─ hinge("dial") -> dial
                                                    └─ slide("band") -> band

Check-51 bilateral force balance:
- Open-helmet rock-back (``TIP_BACK_NM`` in all): the parts' own weights give some of it (the dial and band hang
  behind the strap anchors, ``parts.weights_tip_back_nm``); a ``+x`` push on ``helmet`` gives the rest, with an equal
  ``-x`` reaction on ``head``. With the rocking back off (a trial's test load) the push cancels the weights.
- Band friction hold (``TILT_HOLD_NM = TILT_FRICTION_NM + BAND_MU * band_force * BAND_ARM``): capped stiff
  elasto-plastic spring on ``hinge("tilt")`` between ``helmet`` and ``head``.
- Ratchet dial: tightening costs ``DIAL_TURN_NM + band_force * BAND_LEAD``, loosening by hand ``DIAL_HOLD_NM`` (the
  pawl); a capped stiff spring on ``hinge("dial")`` between ``dial`` and ``helmet`` whose rest follows the dial either
  way once that torque is reached. Nothing turns it back but a hand: the band does not back-drive it.
- Rear band press (``band_force``): ``slide("band")`` spring draws ``band`` into ``head``'s analytic occipital
  sphere; ``mk.contact_force("band", "head")`` reads the bilateral normal force. The slide's axis runs through the
  tilt axis, so the press itself does not turn the helmet.
- Dial twist (``DIAL_TILT_SHARE``): while the dial is turned, a share of its turning torque tips ``helmet`` towards
  ``+tilt`` (the band drawn in along the back of the head), with the equal and opposite torque on ``head``. Put on
  ``dial`` instead, the reaction would go straight back into ``helmet`` through hinge("dial") (its axis has no x
  part) and cancel.
"""

from __future__ import annotations

import math

import numpy as np

import mechkit as mk
import parts as P
from mechkit import hinge, slide, weld

MECHKIT_HELPER = True

TILT_START_RAD = math.radians(P.TILT_START_DEG)
TILT_LOWER_RAD = -math.radians(P.TILT_START_DEG - P.TILT_MIN_DEG)  # -18.0 deg from +13 deg built pose -> -5 deg
DIAL_MID_RAD = math.radians(P.DIAL_MID_DEG)  # 150.0 deg midpoint so PhysX joint stays inside [-150, +150] deg

STATE: dict = {}


def declare(table_top: float) -> None:
    """Declare the pure serial joint chain and band-to-head contact sensor."""
    weld("head_mount", "head", None)
    hinge(
        "tilt",
        "helmet",
        "head",
        anchor=P.pivot_world(table_top),
        axis=(1.0, 0.0, 0.0),
        limits=(TILT_LOWER_RAD, 0.0),
        spring=P.K_TILT,
        rest=0.0,
        damping=P.D_TILT,
    )
    hinge(
        "dial",
        "dial",
        "helmet",
        anchor=tuple(P.dial_center_world(table_top, P.TILT_START_DEG)),
        axis=tuple(P.dial_axis_world(P.TILT_START_DEG)),
        limits=(-DIAL_MID_RAD, +DIAL_MID_RAD),
        spring=P.K_DIAL,
        rest=-DIAL_MID_RAD,
        damping=P.D_DIAL,
    )
    slide(
        "band",
        "band",
        "helmet",
        axis=tuple(P.band_axis_world(P.TILT_START_DEG)),
        limits=(0.0, P.BAND_TRAVEL),
        anchor=tuple(P.band_center_world(table_top, P.TILT_START_DEG, 0.0)),
        spring=P.K_BAND,
        rest=0.0,
        damping=P.D_BAND,
    )
    mk.sense_contact("band", "head")
    reset()


def reset() -> None:
    """Restore mechanism state and PhysX joint targets/caps at episode start."""
    STATE.clear()
    STATE.update(
        tilt_rest=0.0,
        dial_rest=-DIAL_MID_RAD,
        dial_cap=P.DIAL_HOLD_NM,
        max_dial_deg=0.0,
        max_clicks=0,
        first_touch_dial_deg=None,
        clamp_dial_deg=None,
        peak_band_n=0.0,
        peak_tilt_hold_nm=P.TILT_FRICTION_NM,
        peak_dial_torque_nm=0.0,
        tip_back_enabled=True,
        front_edge_pull_n=0.0,
        extra_helmet_force=(0.0, 0.0, 0.0),
        extra_helmet_torque=(0.0, 0.0, 0.0),
        extra_dial_torque=(0.0, 0.0, 0.0),
        force_profile=[],
    )
    if mk.MECH.environment is not None:
        if "dial" in mk.MECH.joint_prims:
            mk.set_joint("dial", -DIAL_MID_RAD)
            mk.set_spring("dial", rest=-DIAL_MID_RAD, most=P.DIAL_HOLD_NM)
        if "tilt" in mk.MECH.joint_prims:
            mk.set_joint("tilt", 0.0)
            mk.set_spring("tilt", rest=0.0, most=P.TILT_FRICTION_NM)
        if "band" in mk.MECH.joint_prims:
            mk.set_joint("band", 0.0)
            mk.set_spring("band", rest=0.0, most=1.0e5)
        for bname in ("head", "helmet", "dial", "band"):
            if bname in mk.MECH.body_prims:
                mk.push(bname, force=(0.0, 0.0, 0.0), torque=(0.0, 0.0, 0.0))


def set_test_load(
    *,
    front_edge_pull_n: float = 0.0,
    helmet_force=(0.0, 0.0, 0.0),
    helmet_torque=(0.0, 0.0, 0.0),
    dial_torque=(0.0, 0.0, 0.0),
    tip_back_enabled: bool = True,
) -> None:
    """Apply controlled bilateral external loads for mechanics trials and the 10 N pull rule check."""
    STATE["front_edge_pull_n"] = float(front_edge_pull_n)
    STATE["extra_helmet_force"] = tuple(float(v) for v in helmet_force)
    STATE["extra_helmet_torque"] = tuple(float(v) for v in helmet_torque)
    STATE["extra_dial_torque"] = tuple(float(v) for v in dial_torque)
    STATE["tip_back_enabled"] = bool(tip_back_enabled)


def tilt_rad() -> float:
    """Helmet tilt angle in radians (0 = level, +TILT_START_RAD = +13 deg tipped back)."""
    q_tilt = float(mk.joint("tilt")) if "tilt" in mk.MECH.travel else 0.0
    return TILT_START_RAD + q_tilt


def tilt_deg() -> float:
    return math.degrees(tilt_rad())


def dial_rad() -> float:
    """Cumulative dial turn in radians from open start (0 rad = 0 deg open)."""
    q_dial = float(mk.joint("dial")) if "dial" in mk.MECH.travel else -DIAL_MID_RAD
    return max(0.0, q_dial + DIAL_MID_RAD)


def dial_deg() -> float:
    return math.degrees(dial_rad())


def dial_clicks() -> float:
    return dial_deg() / P.DIAL_CLICK_DEG


def clicks_past_touch() -> float:
    return max(0.0, dial_clicks() - P.PRETOUCH_CLICKS)


def band_slide_m() -> float:
    return float(mk.joint("band")) if "band" in mk.MECH.travel else 0.0


def band_rest_m() -> float:
    return min(P.BAND_TRAVEL, P.BAND_LEAD * dial_rad())


def band_force_n() -> float:
    """Bilateral normal force (N) between rear retention band and back of head."""
    return float(mk.contact_force("band", "head"))


def tilt_hold_nm(band_n: float | None = None) -> float:
    """Friction holding torque cap (N m) on hinge('tilt') from rear band press."""
    bf = band_force_n() if band_n is None else float(band_n)
    return float(P.TILT_FRICTION_NM + P.BAND_MU * bf * P.BAND_ARM)


def dial_turn_torque_nm(band_n: float | None = None) -> float:
    """Forward tightening torque (N m) required on hinge('dial')."""
    bf = band_force_n() if band_n is None else float(band_n)
    return float(P.DIAL_TURN_NM + bf * P.BAND_LEAD)


def watch(events: list, table_top: float) -> dict:
    """Advance spring targets, caps, and bilateral reaction pushes for one control step."""
    q_tilt = float(mk.joint("tilt")) if "tilt" in mk.MECH.travel else 0.0
    q_dial = float(mk.joint("dial")) if "dial" in mk.MECH.travel else -DIAL_MID_RAD
    bf = band_force_n()
    t_now = mk.now()

    # 1. Rear band slide target from dial angle
    b_rest = min(P.BAND_TRAVEL, P.BAND_LEAD * max(0.0, q_dial + DIAL_MID_RAD))

    # 2. One-way ratchet on dial: tightening advances rest at tau_turn; reverse engages DIAL_HOLD_NM pawl
    tau_turn = dial_turn_torque_nm(bf)
    lag_turn = tau_turn / P.K_DIAL
    d_rest_prev = float(STATE.get("dial_rest", -DIAL_MID_RAD))
    tau_dial_active = min(tau_turn, max(0.0, P.K_DIAL * (q_dial - d_rest_prev)))
    # The drive's cap holds both ways until the next step, so a dial that is not being turned forward is held by the
    # pawl's DIAL_HOLD_NM: with the turning torque as its cap, a dial at rest spun back 90 deg under 0.20 N m in one
    # step and the rest followed it (run h4t1g).
    if q_dial - d_rest_prev >= lag_turn - 1e-12:  # turned forward with the turning torque: the rest follows
        d_rest_new = q_dial - lag_turn
        d_cap = tau_turn
    elif q_dial >= d_rest_prev - P.DIAL_HOLD_NM / P.K_DIAL:  # at rest, or pushed either way under what turns it
        d_rest_new = d_rest_prev
        d_cap = P.DIAL_HOLD_NM
    else:  # turned back by hand past the pawl: it loosens and stays loosened
        d_rest_new = q_dial + P.DIAL_HOLD_NM / P.K_DIAL
        d_cap = P.DIAL_HOLD_NM
    STATE["dial_rest"] = d_rest_new
    STATE["dial_cap"] = d_cap

    # 3. Capped elasto-plastic friction hold on tilt
    t_hold = tilt_hold_nm(bf)
    lag_tilt = t_hold / P.K_TILT
    t_rest = float(STATE.get("tilt_rest", 0.0))
    if q_tilt > t_rest + lag_tilt:
        t_rest = q_tilt - lag_tilt
    elif q_tilt < t_rest - lag_tilt:
        t_rest = q_tilt + lag_tilt
    STATE["tilt_rest"] = t_rest

    # 4. Track milestone events & peaks
    d_deg = math.degrees(max(0.0, q_dial + DIAL_MID_RAD))
    n_clicks = int(math.floor((d_deg + 1e-6) / P.DIAL_CLICK_DEG))
    if n_clicks > int(STATE.get("max_clicks", 0)):
        STATE["max_clicks"] = n_clicks
    STATE["max_dial_deg"] = max(float(STATE.get("max_dial_deg", 0.0)), d_deg)
    STATE["peak_band_n"] = max(float(STATE.get("peak_band_n", 0.0)), bf)
    STATE["peak_tilt_hold_nm"] = max(float(STATE.get("peak_tilt_hold_nm", 0.0)), t_hold)
    STATE["peak_dial_torque_nm"] = max(float(STATE.get("peak_dial_torque_nm", 0.0)), tau_dial_active)

    if bf > 0.25 and STATE.get("first_touch_dial_deg") is None:
        STATE["first_touch_dial_deg"] = round(d_deg, 1)
        events.append((t_now, f"band touched occiput at dial={d_deg:.1f} deg ({d_deg / P.DIAL_CLICK_DEG:.1f} clicks)"))
    if bf >= P.CLAMP_N and STATE.get("clamp_dial_deg") is None:
        STATE["clamp_dial_deg"] = round(d_deg, 1)
        events.append((t_now, f"band press reached CLAMP_N at dial={d_deg:.1f} deg ({d_deg / P.DIAL_CLICK_DEG:.1f} clicks)"))

    # 5. Write spring drives & bilateral pushes in live simulator
    tau_weights = P.weights_tip_back_nm(math.degrees(TILT_START_RAD + q_tilt))  # what gravity already gives
    tau_rock = (P.TIP_BACK_NM if STATE.get("tip_back_enabled", True) else 0.0) - tau_weights
    tau_twist = P.DIAL_TILT_SHARE * tau_dial_active

    if mk.MECH.environment is not None:
        if "band" in mk.MECH.joint_prims:
            mk.set_spring("band", rest=b_rest, most=1.0e5)
        if "dial" in mk.MECH.joint_prims:
            mk.set_spring("dial", rest=d_rest_new, most=d_cap)
        if "tilt" in mk.MECH.joint_prims:
            mk.set_spring("tilt", rest=t_rest, most=t_hold)

        pull_n = float(STATE.get("front_edge_pull_n", 0.0))
        cur_tilt_rad = TILT_START_RAD + q_tilt
        tau_pull_x = pull_n * (P.BRIM_ARM * math.cos(cur_tilt_rad) - P.BRIM_DZ * math.sin(cur_tilt_rad))
        ef_h = np.asarray(STATE.get("extra_helmet_force", (0.0, 0.0, 0.0)), dtype=float)
        et_h = np.asarray(STATE.get("extra_helmet_torque", (0.0, 0.0, 0.0)), dtype=float) + np.array([tau_pull_x, 0.0, 0.0])
        et_d = np.asarray(STATE.get("extra_dial_torque", (0.0, 0.0, 0.0)), dtype=float)

        # Helmet gets +tau_rock and +tau_twist about +x; equal-and-opposite reaction goes to head/bench
        # (putting -tau_twist on dial would cancel internally across hinge('dial')'s x-constraint!).
        tau_helmet = np.array([tau_rock + tau_twist, 0.0, 0.0], dtype=float) + et_h
        tau_dial_push = et_d
        tau_head = np.array([-(tau_rock + tau_twist), 0.0, 0.0], dtype=float) - et_h - et_d

        mk.push("helmet", force=tuple(ef_h), torque=tuple(tau_helmet))
        mk.push("dial", force=(0.0, 0.0, 0.0), torque=tuple(tau_dial_push))
        mk.push("head", force=tuple(-ef_h), torque=tuple(tau_head))

    snap = {
        "t": round(t_now, 2),
        "tilt_deg": round(math.degrees(TILT_START_RAD + q_tilt), 2),
        "gap_mm": round(1000.0 * P.gap_from_tilt_m(math.degrees(TILT_START_RAD + q_tilt)), 2),
        "dial_deg": round(d_deg, 1),
        "clicks": round(d_deg / P.DIAL_CLICK_DEG, 2),
        "band_n": round(bf, 2),
        "tilt_hold_nm": round(t_hold, 3),
        "dial_active_nm": round(tau_dial_active, 3),
    }
    STATE["last"] = snap
    if len(STATE["force_profile"]) == 0 or t_now - STATE["force_profile"][-1]["t"] >= 0.49:
        STATE["force_profile"].append(snap)
    return snap
