"""Pure-Python unit tests for sink_stopper scoring rules, whole-trial fouls, and parts.py geometry invariants."""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402
import task as T  # noqa: E402

CLEAR_GRIPPERS = {
    "left": (np.array([0.10, 0.30, T.TOP + 0.25]), 1.0),
    "right": (np.array([0.10, -0.30, T.TOP + 0.25]), 1.0),
}


def _stopper_pose(gap: float = 0.0, dx: float = 0.0, dy: float = 0.0, tilt_deg: float = 0.0, yaw_deg: float = 0.0):
    pos = np.array([P.X_DRAIN + dx, P.Y_DRAIN + dy, T.TOP + P.FLANGE_Z + gap], dtype=float)
    rot = mk.axis_angle((0.0, 0.0, 1.0), math.radians(yaw_deg)) @ mk.axis_angle((1.0, 0.0, 0.0), math.radians(tilt_deg))
    return pos, rot


def _rod_pose(ball_x: float = P.X_SEAT, dy: float = 0.0, dz: float = 0.0, pitch_down_deg: float = 0.0):
    pos = np.array([ball_x, P.Y_DRAIN + dy, T.TOP + P.PORT_Z + dz], dtype=float)
    rot = mk.axis_angle((0.0, 1.0, 0.0), math.radians(pitch_down_deg))
    return pos, rot


def _set(stopper=None, rod=None, velocities=None, grippers=None, t: float = 1.0) -> None:
    poses = {
        "stopper": _stopper_pose() if stopper is None else stopper,
        "pivot_rod": _rod_pose(P.X_BALL_START) if rod is None else rod,
    }
    mk.set_state(poses=poses, velocities=velocities, grippers=CLEAR_GRIPPERS if grippers is None else grippers, time=t)


def test_parts_sources_and_geometry_invariants() -> None:
    s = P.summary()
    assert sum(s["counts"].values()) == len(P.FIGURES) >= 35
    assert s["tip_past_stopper_axis_mm"] > 10.0
    assert s["closed_bottom_clearance_mm"] >= 5.0
    assert 2.5 <= s["horizontal_seated_gap_mm"] <= 5.0


def test_start_state_scores_zero() -> None:
    T.begin(0)
    _set()
    T.step()
    assert T.score(None) == 0.0
    assert T.STATE["zero"] is None
    assert abs(T.collar_mark_gap() - P.RETRACT_START) < 1e-6


def test_unseated_rod_scores_zero() -> None:
    T.begin(0)
    # Blocked 16 mm short of seat against unaligned stem
    _set(stopper=_stopper_pose(gap=0.0, yaw_deg=90.0), rod=_rod_pose(ball_x=P.X_SEAT + 0.016))
    T.step()
    assert T.score(None) == 0.0


def test_seated_unpressed_scores_half() -> None:
    T.begin(0)
    _set(stopper=_stopper_pose(gap=0.0035, yaw_deg=0.0), rod=_rod_pose(ball_x=P.X_SEAT, pitch_down_deg=0.0))
    T.step()
    assert T.collar_seated()
    assert T.score(None) == 0.5


def test_seated_and_held_open_reaches_full_after_hold_only() -> None:
    T.begin(0)
    _set(stopper=_stopper_pose(gap=0.0110, yaw_deg=0.0), rod=_rod_pose(ball_x=P.X_SEAT, pitch_down_deg=16.5))
    for k in range(P.HOLD_STEPS - 1):
        T.step()
        assert T.score(None) == 0.5, f"reached 1.0 early at step {k + 1}"
    T.step()
    assert T.score(None) == 1.0


def test_hold_resets_on_drift_or_nearby_hand_or_motion() -> None:
    T.begin(0)
    _set(stopper=_stopper_pose(gap=0.0110), rod=_rod_pose(ball_x=P.X_SEAT, pitch_down_deg=16.5))
    for _ in range(10):
        T.step()
    assert T.STATE["hold"] == 10
    # Drift > 2.0 mm resets hold counter
    _set(stopper=_stopper_pose(gap=0.0138), rod=_rod_pose(ball_x=P.X_SEAT, pitch_down_deg=16.5))
    T.step()
    assert T.STATE["hold"] == 0
    # Nearby hand (< 30 mm) blocks both 0.5 and 1.0
    near = dict(CLEAR_GRIPPERS)
    near["left"] = (np.array([P.X_DRAIN, P.Y_DRAIN, T.TOP + P.FLANGE_Z + 0.020]), 1.0)
    _set(stopper=_stopper_pose(gap=0.0110), rod=_rod_pose(ball_x=P.X_SEAT, pitch_down_deg=16.5), grippers=near)
    T.step()
    assert T.score(None) == 0.0
    # Moving stopper (> 25 mm/s) blocks score
    _set(
        stopper=_stopper_pose(gap=0.0110),
        rod=_rod_pose(ball_x=P.X_SEAT, pitch_down_deg=16.5),
        velocities={"stopper": (np.array([0.0, 0.0, 0.035]), np.zeros(3))},
    )
    T.step()
    assert T.score(None) == 0.0


def test_propped_crooked_or_overhigh_shortcut_rejected() -> None:
    T.begin(0)
    # Propped high (> 22 mm) on rim
    _set(stopper=_stopper_pose(gap=0.025, dx=0.005, tilt_deg=9.0), rod=_rod_pose(ball_x=P.X_SEAT))
    for _ in range(P.HOLD_STEPS + 2):
        T.step()
    assert T.score(None) == 0.0


def test_stopper_lifted_high_to_inspect_does_not_latch_zero_and_recovers() -> None:
    T.begin(0)
    # Lifting stopper 55 mm up or out of bore to inspect eye scores 0.0 while out, without latching a permanent foul
    _set(stopper=_stopper_pose(gap=0.055, dx=0.025), rod=_rod_pose(ball_x=P.X_SEAT))
    T.step()
    assert T.STATE["zero"] is None
    assert T.score(None) == 0.0
    # Once reinserted and coupled open 11 mm for 20 steps, full 1.0 score is earned
    _set(stopper=_stopper_pose(gap=0.0110), rod=_rod_pose(ball_x=P.X_SEAT, pitch_down_deg=16.5))
    for _ in range(P.HOLD_STEPS):
        T.step()
    assert T.score(None) == 1.0


def test_whole_trial_zero_rod_dropped_latches_permanently() -> None:
    T.begin(0)
    _set(stopper=_stopper_pose(gap=0.0), rod=_rod_pose(ball_x=0.400, dz=-(P.PORT_Z - 0.010)))
    T.step()
    assert T.STATE["zero"] == "pivot rod dropped onto cabinet floor"
    _set(stopper=_stopper_pose(gap=0.0110), rod=_rod_pose(ball_x=P.X_SEAT, pitch_down_deg=16.5))
    for _ in range(P.HOLD_STEPS + 5):
        T.step()
    assert T.score(None) == 0.0


def test_collar_cad_stl_and_full_depth_deck_collider() -> None:
    import collar_cad as CC
    import scene as S

    specs = CC.collar_specs()
    assert specs["od_mm"] == 26.4 and specs["length_mm"] == 19.0 and specs["slot_count"] == 4
    stl = CC.build_ascii_stl()
    assert stl.startswith("solid danco_80767_4slot_snap_collar\n") and stl.endswith("endsolid danco_80767_4slot_snap_collar\n")
    wt = CC.verify_watertight(stl)
    assert wt["triangles"] >= 1000 and wt["boundary_edges"] == 0 and wt["non_manifold_edges"] == 0 and wt["unpaired_directed_edges"] == 0
    assert "pulled out" not in (T.score.__doc__ or "") and "[-2.0, 22.0]" in (T.score.__doc__ or "")
    # Clear cast-acrylic rear L-roof is solid in PhysX and covers full rear bay behind DECK_SPLIT_X = 0.444 m
    assert len(S.ACRYLIC_PANE_SHAPES) == 2
    for idx in S.ACRYLIC_PANE_SHAPES:
        assert mk.MECH.bodies["vanity"].shapes[idx].solid is True
