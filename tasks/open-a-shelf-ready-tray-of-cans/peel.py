"""Bilateral PhysX perforation mechanism for the shelf-ready tray task (MECHKIT_HELPER module).

Serial kinematic tree (zero closed loops, zero one-sided mk.push loads):
    tray_base -> c_lift (slide: perf_lift)
              -> c_pitch (hinge: peel_pitch, irreversible capped spring via mk.set_spring)
              -> c_twist (hinge: peel_skew)
              --weld: panel_detach--> front_panel

Physical behaviour (forces from parts.py, README "Key figures"):
1. Straight vertical pull (+z) stretches ``perf_lift`` (10 kN/m) without advancing ``peel_pitch``. It loads the whole
   bottom line in plane: the tear at ``STRAIGHT_LIFT_HOLD_N`` (459 N) is never reached, the slide's stop at 65 N
   comes first, so a straight lift only lifts or tips the tray (the 29 N filled tray lifts off: score 0).
2. Flat outward pull at y = 0 (|peel_skew| < ``PEEL_SKEW_MIN_DEG``): ``peel_pitch`` yields at
   ``SYMMETRIC_PULL_HOLD_N`` (5.5 N, two tear fronts) and the fold tears only the ``SIDE_SEAM_TIES`` side-seam stages;
   the bottom line holds as a crease, so the panel folds down but never comes off (0.5 at most).
3. Diagonal outward-and-downward pull at either top corner twists ``peel_skew`` past ``PEEL_SKEW_MIN_DEG`` (6 deg),
   switching ``peel_pitch``'s torque cap to ``PEEL_START_N * PEEL_LEVER_ARM`` (start = run = 2.75 N) across
   stages 1..10 up to ``PEEL_COMPLETE_DEG`` (52 deg, 84.4 mm arc), where the panel comes off. A panel already
   folded flat comes off the same way once its corner is turned.
4. Irreversible tearing: ``STATE["rest"]`` and ``STATE["ties_torn"]`` advance monotonically and never decrease.
   Letting go mid-peel at 30 deg leaves ``front_panel`` resting >= 28 deg with its torn tie count preserved, while
   untorn ties continue to resist further peel at ``PEEL_RUN_N``.
"""

from __future__ import annotations

import math

import mechkit as mk
import parts as P
from mechkit import hinge, slide, weld

MECHKIT_HELPER = True

TIE_ANGLES_DEG: tuple[float, ...] = tuple(
    P.PEEL_FIRST_TIE_DEG + k * (P.PEEL_COMPLETE_DEG - P.PEEL_FIRST_TIE_DEG) / max(1, P.N_TIES - 1)
    for k in range(P.N_TIES)
)

STATE: dict = {}


def tau_symmetric() -> float:
    return P.SYMMETRIC_PULL_HOLD_N * P.PEEL_LEVER_ARM


def tau_start() -> float:
    return P.PEEL_START_N * P.PEEL_LEVER_ARM


def tau_run() -> float:
    return P.PEEL_RUN_N * P.PEEL_LEVER_ARM


def target_tau_cap() -> float:
    """Active torque cap (N m) on peel_pitch given whether corner skew initiated tearing and how many ties snapped."""
    if not STATE.get("corner_started", False):
        return tau_symmetric()
    return tau_start() if STATE.get("ties_torn", 0) == 0 else tau_run()


def declare(table_top: float) -> None:
    """Declare the 4-joint serial chain between tray_base, the 3 carriers, and front_panel."""
    anchor = P.hinge_anchor_world(table_top)
    slide(
        "perf_lift",
        "c_lift",
        "tray_base",
        anchor=anchor,
        axis=(0.0, 0.0, 1.0),
        limits=(0.0, P.LIFT_STOP_M),
        spring=P.K_LIFT,
        rest=0.0,
        damping=P.D_LIFT,
    )
    hinge(
        "peel_pitch",
        "c_pitch",
        "c_lift",
        anchor=anchor,
        axis=(0.0, -1.0, 0.0),
        limits=(0.0, math.radians(P.PITCH_MAX_DEG)),
        spring=P.STIFFNESS_PITCH,
        rest=0.0,
        damping=P.D_PITCH,
    )
    hinge(
        "peel_skew",
        "c_twist",
        "c_pitch",
        anchor=anchor,
        axis=P.SKEW_AXIS,
        limits=(-math.radians(P.SKEW_MAX_DEG), math.radians(P.SKEW_MAX_DEG)),
        spring=P.K_SKEW,
        rest=0.0,
        damping=P.D_SKEW,
    )
    weld("panel_detach", "front_panel", "c_twist")
    reset()


def reset() -> None:
    """Restore perforation state and PhysX spring target/cap at the start of every episode."""
    STATE.clear()
    STATE.update(
        rest=0.0,
        ties_torn=0,
        corner_started=False,
        corner_latched=False,
        corner_noted=False,
        flat_fold=False,
        side_seams_noted=False,
        max_skew_deg=0.0,
        max_pitch_deg=0.0,
        max_lift_mm=0.0,
        peak_lift_n=0.0,
        peak_peel_n=0.0,
        detached_at=None,
        detach_cause=None,
        applied_rest=None,
        applied_cap=None,
        force_profile=[],
    )
    if mk.MECH.environment is not None and "peel_pitch" in mk.MECH.joint_prims:
        cap0 = tau_symmetric()
        mk.set_spring("peel_pitch", rest=0.0, most=cap0)
        STATE["applied_rest"] = 0.0
        STATE["applied_cap"] = cap0


def detached() -> bool:
    return mk.is_cut("panel_detach")


def ties_torn() -> int:
    return int(STATE.get("ties_torn", 0))


def lift_force_n() -> float:
    """Bilateral vertical tension (N) in perf_lift between front_panel chain and tray_base (0 once detached)."""
    if detached() or "perf_lift" not in mk.MECH.travel:
        return 0.0
    return max(0.0, P.K_LIFT * float(mk.joint("perf_lift")))


def peel_torque_nm() -> float:
    """Bilateral spring drive torque (N m) carried by peel_pitch (0 once detached)."""
    if detached() or "peel_pitch" not in mk.MECH.travel:
        return 0.0
    theta = float(mk.joint("peel_pitch"))
    rest = float(STATE.get("rest", 0.0))
    cap = target_tau_cap()
    raw = P.STIFFNESS_PITCH * (theta - rest)
    return float(max(-cap, min(cap, raw)))


def peel_force_n() -> float:
    """Equivalent perpendicular peel force (N) at the nominal grip rim (lever arm PEEL_LEVER_ARM)."""
    return max(0.0, peel_torque_nm() / P.PEEL_LEVER_ARM)


def arc_travel_mm() -> float:
    """Top-edge peel arc travel (mm) corresponding to current peel_pitch angle."""
    if "peel_pitch" not in mk.MECH.travel:
        return 0.0
    return 1000.0 * P.PEEL_LEVER_ARM * max(0.0, float(mk.joint("peel_pitch")))


def free_equilibrium_pitch_deg() -> float:
    """Static equilibrium angle (deg) of an attached front_panel when zero gripper load acts on it."""
    rest = float(STATE.get("rest", 0.0))
    # Gravity torque of front_panel (center of mass ~0.5 * PANEL_SOLID_H above hinge) pulling slightly outward (+theta)
    cm_arm = 0.5 * P.PANEL_SOLID_H
    grav_tau = P.PANEL_MASS * P.GRAVITY * cm_arm * math.sin(max(0.0, rest))
    cap = target_tau_cap()
    dtheta = min(cap, max(-cap, grav_tau)) / P.STIFFNESS_PITCH
    return math.degrees(min(math.radians(P.PITCH_MAX_DEG), max(0.0, rest + dtheta)))


def _count_ties_at_angle(pitch_deg: float) -> int:
    return sum(1 for deg in TIE_ANGLES_DEG if pitch_deg >= deg - 1e-9)


def _sync_spring() -> None:
    if mk.MECH.environment is None or "peel_pitch" not in mk.MECH.joint_prims:
        return
    want_rest = float(STATE["rest"])
    want_cap = target_tau_cap()
    rest_arg = want_rest if STATE["applied_rest"] is None or abs(want_rest - STATE["applied_rest"]) > 1e-6 else None
    cap_arg = want_cap if STATE["applied_cap"] is None or abs(want_cap - STATE["applied_cap"]) > 1e-6 else None
    if rest_arg is not None or cap_arg is not None:
        mk.set_spring("peel_pitch", rest=rest_arg, most=cap_arg)
        STATE["applied_rest"] = want_rest
        STATE["applied_cap"] = want_cap


def watch(events: list) -> None:
    """Step hook: updates irreversible rest & tie count, logs force vs travel, and cuts panel_detach on completion."""
    if detached():
        return

    now = mk.now()
    lift_m = max(0.0, float(mk.joint("perf_lift")))
    pitch_rad = max(0.0, float(mk.joint("peel_pitch")))
    pitch_deg = math.degrees(pitch_rad)
    skew_deg = abs(math.degrees(float(mk.joint("peel_skew"))))

    STATE["max_lift_mm"] = max(STATE["max_lift_mm"], 1000.0 * lift_m)
    STATE["max_pitch_deg"] = max(STATE["max_pitch_deg"], pitch_deg)
    STATE["max_skew_deg"] = max(STATE["max_skew_deg"], skew_deg)

    # 1. Corner skew initiation: instantaneous until a stage tears in corner mode (an elastic 6 deg bump + release
    # resets it, so a following flat pull still faces SYMMETRIC_PULL_HOLD_N), then latched for good. Side-seam
    # stages torn by a flat pull do not latch it.
    skew_active = skew_deg >= P.PEEL_SKEW_MIN_DEG - 1e-9
    if not STATE["corner_latched"]:
        if skew_active and not STATE.get("corner_noted", False):
            STATE["corner_noted"] = True
            events.append((now, f"corner peel initiated at skew {skew_deg:.1f} deg"))
        STATE["corner_started"] = skew_active
    else:
        STATE["corner_started"] = True

    # 2. Measure current loads BEFORE updating rest so peak force reflects the spring load carried this step
    cur_lift_n = lift_force_n()
    cur_peel_n = peel_force_n()
    STATE["peak_lift_n"] = max(STATE["peak_lift_n"], cur_lift_n)
    STATE["peak_peel_n"] = max(STATE["peak_peel_n"], cur_peel_n)

    # 3. Progressive tie tearing & irreversible rest tracking (plastic rest advances only once >= 1 tie snaps)
    if STATE["corner_started"]:
        new_ties = _count_ties_at_angle(pitch_deg)
        if new_ties > STATE["ties_torn"]:
            prev = STATE["ties_torn"]
            STATE["ties_torn"] = new_ties
            STATE["corner_latched"] = True
            if prev == 0 and new_ties >= 1:
                events.append((now, f"first perforation tie snapped at {pitch_deg:.1f} deg ({cur_peel_n:.2f} N)"))
            elif prev < P.PARTIAL_TIES_MIN <= new_ties:
                events.append((now, f"{new_ties}/{P.N_TIES} perforation ties torn at {pitch_deg:.1f} deg"))
        if STATE["ties_torn"] >= 1:
            cap = target_tau_cap()
            lag = cap / P.STIFFNESS_PITCH
            if pitch_rad > STATE["rest"] + lag:
                STATE["rest"] = pitch_rad - lag
    else:
        # Flat pull without corner skew: yields plastically above SYMMETRIC_PULL_HOLD_N and tears the side seams only;
        # the bottom line holds as a crease (its in-plane tear, STRAIGHT_LIFT_HOLD_N, is out of reach).
        sym_cap = tau_symmetric()
        sym_lag = sym_cap / P.STIFFNESS_PITCH
        if pitch_rad > sym_lag + math.radians(2.5):
            STATE["flat_fold"] = True
            STATE["rest"] = max(STATE["rest"], pitch_rad - sym_lag)
            STATE["ties_torn"] = max(STATE["ties_torn"], min(P.SIDE_SEAM_TIES, _count_ties_at_angle(pitch_deg)))
            if STATE["ties_torn"] >= P.SIDE_SEAM_TIES and not STATE["side_seams_noted"]:
                STATE["side_seams_noted"] = True
                events.append((now, f"flat pull tore both side seams at {pitch_deg:.1f} deg: the bottom line holds"))

    _sync_spring()

    # Sample compact force-vs-travel curve whenever peel advances by >= 2 mm of arc
    arc_mm = arc_travel_mm()
    prof = STATE["force_profile"]
    if pitch_deg >= 0.5 and (not prof or arc_mm >= prof[-1]["arc_mm"] + 2.0):
        prof.append(
            {
                "t": round(now, 3),
                "pitch_deg": round(pitch_deg, 2),
                "skew_deg": round(skew_deg, 2),
                "arc_mm": round(arc_mm, 1),
                "peel_n": round(cur_peel_n, 2),
                "ties": int(STATE["ties_torn"]),
            }
        )

    # 4. Detachment conditions (never on pinch, touch, proximity, or partial angle)
    cause = None
    if STATE["corner_started"] and STATE["ties_torn"] >= P.N_TIES and pitch_deg >= P.PEEL_COMPLETE_DEG - 1e-9:
        cause = (
            f"progressive corner peel complete ({STATE['ties_torn']}/{P.N_TIES} ties, "
            f"pitch {pitch_deg:.1f} deg, max skew {STATE['max_skew_deg']:.1f} deg, arc {arc_mm:.1f} mm)"
        )
    elif cur_lift_n >= P.STRAIGHT_LIFT_HOLD_N - 1e-9 and lift_m >= P.LIFT_TEAR_M - 1e-9:
        STATE["ties_torn"] = P.N_TIES
        cause = f"straight-lift overload ({cur_lift_n:.1f} N >= {P.STRAIGHT_LIFT_HOLD_N:.1f} N)"

    if cause is not None:
        mk.cut("panel_detach")
        STATE["detached_at"] = round(now, 3)
        STATE["detach_cause"] = cause
        events.append((now, f"front_panel detached: {cause}"))
