#!/usr/bin/env python3
"""Scripted full-score demonstration for ``mech/tasks/systainer_tloc``.

Usage:
    scripts/rta demo <id> mech/tasks/systainer_tloc --check   # simulator check (no MP4 encoding)
    scripts/rta demo <id> mech/tasks/systainer_tloc           # full recorded run (MP4s + setup/end PNGs)
    python mech/tasks/systainer_tloc/demo.py --dry            # offline IK & joint-effort verification
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as K  # noqa: E402
import demo_kit as kit  # noqa: E402
import parts as P  # noqa: E402

GRIP_CLOSED = 0.0
GRIP_FULL_OPEN = 1.0  # 93.9 mm wide hover/descent around 54 mm red box (20 mm/side clearance)
GRIP_BOX_PINCH = round((P.BOX_W - 2.0 * 0.0055) / K.MAX_JAW, 3)  # ~27.5 N per pad on 54 mm red box
GRIP_BOX_RELEASE = round(0.074 / K.MAX_JAW, 3)  # 74 mm fast 0.20 s snap-open release (zero asymmetric finger drag)

GRIP_CRADLE = round(K.SIDE_CRADLE_OPEN_M / K.MAX_JAW, 3)  # 76 mm jaw spread = 83.7 mm 4-point anti-pitch wall clamp

GRIP_BAR_OPEN = round(0.038 / K.MAX_JAW, 3)  # 38 mm around the 16 mm T-Loc wing thickness (clears case_top by >8 mm)
GRIP_BAR_PINCH = round((P.TLOC_BAR_T - 2.0 * 0.0048) / K.MAX_JAW, 3)  # ~28 N per pad on 16 mm T-Loc wing (<= 40 N)
GRIP_WING_OPEN = GRIP_BAR_OPEN
GRIP_WING_PINCH = GRIP_BAR_PINCH

SQUEEZE_IN_N = 18.0  # N opposing inward push per arm across the 4-point side-wall cradle (<= 40 N limit)
SQUEEZE_PEN_M = -0.002  # m gentle compliance with stiff shoulder/elbow y-axis
CARRY_UP_N = 4.8  # N upward per arm (lifts case_top bottom to ~92 mm, 16 mm clear over case_low's 76 mm rim)
SEAT_DOWN_N = -3.0  # N downward seating load per arm onto the 71 mm deck during cleat engagement
SLIDE_X_N = 4.5  # N rearward (+x) push per arm (9.0 N total >> 1.7 N 0.18-ABS cleat friction, << 15.6 N bench hold)
CARRY_X_TRIM_M = -0.005  # m lands cleanly at dx ~ -12 mm ahead of rear cleats without overshooting onto cleat roof


def ready_pose(side: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, dtype=float), K.BASES[side])


def side_move(
    seconds: float,
    side: str,
    x: float,
    z_above_bench: float,
    *,
    out_m: float = 0.0,
    y_center: float = 0.0,
    inward_n: float = 0.0,
    vert_n: float = 0.0,
    fwd_n: float = 0.0,
    label: str = "",
) -> kit.Move:
    sgn = 1.0 if side == "left" else -1.0
    tcp, rot, at_local = K.side_wall_tcp(side, x, z_above_bench, out_m=out_m, opening_m=K.SIDE_CRADLE_OPEN_M)
    tcp = tcp + np.array([0.0, y_center, 0.0], dtype=float)
    push = np.array([fwd_n, -sgn * inward_n, vert_n], dtype=float)
    return kit.Move(seconds, tcp, rot, GRIP_CRADLE, label=label, push=push, push_at=at_local)


def stage12_moves() -> dict[str, list[kit.Move]]:
    """Stage 1 (stow red parts_box cleanly) & Stage 2 (gently sweep 57.3-deg lower lid closed; Right hovers untouched)."""
    Move = kit.Move
    bx, by = P.BOX_START_XY
    cx = P.CASE_LOW_XY[0] + P.CELL_LOCAL_XY[0]
    cy = P.CASE_LOW_XY[1] + P.CELL_LOCAL_XY[1] - 0.003
    box_wt = (0.0, 0.0, P.BOX_SIM_MASS * P.GRAVITY)

    lid_pre, r_lid, lid_at = K.lid_push_tcp(0.320, 0.195, 0.135)
    lid0, _, _ = K.lid_push_tcp(0.412, 0.195, 0.138)
    lid1, _, _ = K.lid_push_tcp(0.412, 0.090, 0.138)
    lid2, _, _ = K.lid_push_tcp(0.355, 0.090, 0.122)
    lid3, _, _ = K.lid_push_tcp(0.325, 0.090, 0.092)
    lid4, _, _ = K.lid_push_tcp(0.315, 0.090, 0.075)
    lid_clear, _, _ = K.lid_push_tcp(0.245, 0.180, 0.118)

    left = [
        Move(1.2, (bx, by, K.TOP + 0.098), K.R_BOX, GRIP_FULL_OPEN, "to_parts_box"),
        Move(0.45, (bx, by, K.TOP + 0.098), K.R_BOX, GRIP_FULL_OPEN, "settle_over_box"),
        Move(0.70, (bx, by, K.TOP + 0.050), K.R_BOX, GRIP_FULL_OPEN, "lower_parts_box"),
        Move(0.55, (bx, by, K.TOP + 0.050), K.R_BOX, GRIP_BOX_PINCH, "pinch_parts_box"),
        Move(0.70, (bx, by, K.TOP + 0.125), K.R_BOX, GRIP_BOX_PINCH, "lift_parts_box", push=box_wt),
        Move(1.10, (cx, cy, K.TOP + 0.128), K.R_BOX, GRIP_BOX_PINCH, "over_compartment", push=box_wt),
        Move(0.35, (cx, cy, K.TOP + 0.128), K.R_BOX, GRIP_BOX_PINCH, "settle_compartment", push=box_wt),
        Move(0.80, (cx, cy, K.TOP + 0.075), K.R_BOX, GRIP_BOX_PINCH, "into_compartment", push=(0.0, 0.0, 0.0)),
        Move(0.25, (cx, cy, K.TOP + 0.075), K.R_BOX, GRIP_BOX_PINCH, "pause_in_cell", push=(0.0, 0.0, 0.0)),
        Move(0.20, (cx, cy, K.TOP + 0.075), K.R_BOX, GRIP_BOX_RELEASE, "release_parts_box", push=(0.0, 0.0, 0.0)),
        Move(0.25, (cx, cy, K.TOP + 0.075), K.R_BOX, GRIP_BOX_RELEASE, "let_box_settle"),
        Move(0.65, (cx, cy, K.TOP + 0.128), K.R_BOX, GRIP_BOX_RELEASE, "clear_compartment"),
        Move(0.45, (cx, cy, K.TOP + 0.128), K.R_BOX, GRIP_CLOSED, "close_fingers_clear"),
        # Stage 2: route wide outside left wall (y = +0.195), step behind open lid at y = +0.090, gently sweep shut
        Move(0.75, lid_pre, r_lid, GRIP_CLOSED, "out_left_clear", push=(0.0, 0.0, 0.0), push_at=lid_at),
        Move(0.75, lid0, r_lid, GRIP_CLOSED, "around_lid_left"),
        Move(0.75, lid1, r_lid, GRIP_CLOSED, "behind_open_lid"),
        Move(0.80, lid2, r_lid, GRIP_CLOSED, "push_lid_mid", push=(-1.2, 0.0, -2.5), push_at=lid_at),
        Move(0.80, lid3, r_lid, GRIP_CLOSED, "push_lid_low", push=(-1.0, 0.0, -3.5), push_at=lid_at),
        Move(0.70, lid4, r_lid, GRIP_CLOSED, "seat_lid_shut", push=(0.0, 0.0, -4.0), push_at=lid_at),
        Move(0.65, lid_clear, r_lid, GRIP_CLOSED, "clear_closed_lid", push=(0.0, 0.0, 0.0)),
    ]
    t_left = round(sum(mv.seconds for mv in left), 4)
    # Right arm pre-positions cleanly in open air beside case_top (untouched, so case_low never shifts!)
    r_pre = side_move(2.4, "right", P.CASE_TOP_XY[0], 0.055, out_m=0.032, label="pre_side_right")
    right = [r_pre, Move(round(t_left - r_pre.seconds, 4), None, None, GRIP_CRADLE, "wait_for_lid")]
    return {"left": left, "right": right}


def append_stage3_carry(arms: dict, x_top: float, y_top: float, x_low: float, y_low: float) -> float:
    """Two-arm 4-point 83.7 mm-spread flat-pad squeeze lift & carry onto lid_low 12 mm ahead of rear cleats."""
    Move = kit.Move
    l_ready_pos, l_ready_rot = ready_pose("left")
    x_land_cmd = x_low - P.CLEAT_SLIDE + CARRY_X_TRIM_M
    arms["left"].moves.append(Move(0.90, l_ready_pos, l_ready_rot, GRIP_CRADLE, "left_ready_bridge"))
    arms["right"].moves.append(side_move(0.90, "right", x_top, 0.055, out_m=0.032, y_center=y_top, label="right_to_top"))
    for side in ("left", "right"):
        arms[side].moves.extend(
            [
                side_move(1.0, side, x_top, 0.046, out_m=0.028, y_center=y_top, label="align_case_walls"),
                side_move(0.75, side, x_top, 0.046, out_m=SQUEEZE_PEN_M, y_center=y_top, inward_n=SQUEEZE_IN_N, label="squeeze_walls"),
                side_move(0.35, side, x_top, 0.046, out_m=SQUEEZE_PEN_M, y_center=y_top, inward_n=SQUEEZE_IN_N, vert_n=CARRY_UP_N, label="preload_lift"),
                side_move(1.25, side, x_top, 0.148, out_m=SQUEEZE_PEN_M, y_center=y_top, inward_n=SQUEEZE_IN_N, vert_n=CARRY_UP_N, label="lift_upper_case"),
                side_move(0.70, side, x_top, 0.148, out_m=SQUEEZE_PEN_M, y_center=y_top, inward_n=SQUEEZE_IN_N, vert_n=CARRY_UP_N, label="hold_lifted"),
                side_move(1.50, side, x_land_cmd, 0.148, out_m=SQUEEZE_PEN_M, y_center=y_low, inward_n=SQUEEZE_IN_N, vert_n=CARRY_UP_N, label="carry_over_lower"),
                side_move(0.40, side, x_land_cmd, 0.146, out_m=SQUEEZE_PEN_M, y_center=y_low, inward_n=SQUEEZE_IN_N, vert_n=CARRY_UP_N, label="hover_ahead_cleats"),
                side_move(0.90, side, x_land_cmd, 0.109, out_m=SQUEEZE_PEN_M, y_center=y_low, inward_n=16.0, vert_n=SEAT_DOWN_N, label="land_ahead_cleats"),
            ]
        )
    return x_land_cmd


def append_stage3_slide(arms: dict, last_x_cmd: float, rem_dx: float, y_low: float) -> None:
    """Closed-loop rearward slide of exact remaining distance ``rem_dx`` into rear cleats until front feet drop flush."""
    slide_delta = max(-0.005, min(0.028, rem_dx))
    x_slide_cmd = last_x_cmd + slide_delta + 0.006
    x_settle_cmd = last_x_cmd + slide_delta + 0.002
    for side in ("left", "right"):
        arms[side].moves.extend(
            [
                side_move(1.45, side, x_slide_cmd, 0.107, out_m=SQUEEZE_PEN_M, y_center=y_low, inward_n=15.0, vert_n=SEAT_DOWN_N, fwd_n=SLIDE_X_N, label="slide_into_cleats"),
                side_move(0.45, side, x_settle_cmd, 0.107, out_m=-0.001, y_center=y_low, inward_n=10.0, vert_n=-3.0, fwd_n=0.0, label="drop_front_flush"),
                side_move(0.25, side, x_settle_cmd, 0.108, out_m=0.002, y_center=y_low, inward_n=0.0, vert_n=0.0, fwd_n=0.0, label="unpress_walls"),
                side_move(0.55, side, x_settle_cmd, 0.112, out_m=0.062, y_center=y_low, inward_n=0.0, vert_n=0.0, fwd_n=0.0, label="release_walls"),
                side_move(0.45, side, x_settle_cmd - 0.040, 0.156, out_m=0.068, y_center=y_low, inward_n=0.0, vert_n=0.0, fwd_n=0.0, label="clear_walls_up"),
            ]
        )


def append_stage4_s1(
    arms: dict,
    low_pos: np.ndarray,
    low_rot: np.ndarray,
    cur_tloc_deg: float,
    z_top_surf: float = 0.147,
) -> None:
    """Left presses straight down (-10 N, 0 N sideways!) on top of coupled stack; Right turns OPEN -> CLOSE."""
    Move = kit.Move
    lx, ly = float(low_pos[0]), float(low_pos[1])
    x_brace = min(0.392, lx + 0.047)
    l_ready_pos, l_ready_rot = ready_pose("left")
    z_hov = min(0.162, z_top_surf + 0.015)
    z_tch = min(0.148, z_top_surf + 0.0005)
    b_hover, b_rot, b_at = K.top_brace_tcp(x_brace - 0.003, ly + 0.100, z_hov)
    b_touch, _, _ = K.top_brace_tcp(x_brace, ly + 0.100, z_tch)
    brace_push = (2.0, 0.0, -8.5)
    left_moves = [
        Move(0.85, l_ready_pos, l_ready_rot, GRIP_CLOSED, "left_ready_s4", push=(0.0, 0.0, 0.0)),
        Move(0.85, b_hover, b_rot, GRIP_CLOSED, "over_stack_top", push=(0.0, 0.0, 0.0), push_at=b_at),
        Move(0.55, b_touch, b_rot, GRIP_CLOSED, "brace_stack", push=brace_push, push_at=b_at),
    ]
    start_deg = max(P.TLOC_MIN_DEG, min(-25.0, cur_tloc_deg))
    end_deg = 4.0
    r_ready_pos, r_ready_rot = ready_pose("right")
    far_s1_tcp, app_rot = K.tloc_wing_tcp(start_deg, back_m=0.090, low_pos=low_pos, low_rot=low_rot)
    app_tcp, _ = K.tloc_wing_tcp(start_deg, back_m=0.040, low_pos=low_pos, low_rot=low_rot)
    in_tcp, in_rot = K.tloc_wing_tcp(start_deg, back_m=0.0, low_pos=low_pos, low_rot=low_rot)
    right_moves = [
        Move(0.90, r_ready_pos, r_ready_rot, GRIP_WING_OPEN, "through_ready"),
        Move(0.85, far_s1_tcp, app_rot, GRIP_WING_OPEN, "front_clear_s1"),
        Move(0.45, app_tcp, app_rot, GRIP_WING_OPEN, "align_tloc_wings"),
        Move(0.55, in_tcp, in_rot, GRIP_WING_OPEN, "onto_tloc_wings"),
        Move(0.55, in_tcp, in_rot, GRIP_WING_PINCH, "pinch_tloc_wings"),
    ]
    for k in range(1, 7):
        a = start_deg + (end_deg - start_deg) * (k / 6.0)
        tcp_a, rot_a = K.tloc_wing_tcp(a, back_m=0.0, low_pos=low_pos, low_rot=low_rot)
        right_moves.append(Move(0.32, tcp_a, rot_a, GRIP_WING_PINCH, f"turn_close_{a:.0f}"))
    end_tcp, end_rot = K.tloc_wing_tcp(end_deg, back_m=0.0, low_pos=low_pos, low_rot=low_rot)
    step_back_tcp, _ = K.tloc_wing_tcp(0.0, back_m=0.042, low_pos=low_pos, low_rot=low_rot)
    back_tcp, back_rot = K.tloc_wing_tcp(-35.0, back_m=0.082, low_pos=low_pos, low_rot=low_rot)
    right_moves.extend(
        [
            Move(0.30, end_tcp, end_rot, GRIP_WING_PINCH, "settle_close"),
            Move(0.40, end_tcp, end_rot, GRIP_WING_OPEN, "release_wings"),
            Move(0.35, step_back_tcp, end_rot, GRIP_WING_OPEN, "step_back_wings"),
            Move(0.40, back_tcp, back_rot, GRIP_WING_OPEN, "back_from_wings"),
        ]
    )
    t_r = round(sum(mv.seconds for mv in right_moves), 4)
    t_l = round(sum(mv.seconds for mv in left_moves), 4)
    if t_r > t_l:
        left_moves.append(
            Move(round(t_r - t_l, 4), b_touch, b_rot, GRIP_CLOSED, "hold_brace_s1", push=brace_push, push_at=b_at)
        )
    arms["left"].moves.extend(left_moves)
    arms["right"].moves.extend(right_moves)


def append_stage4_s2(
    arms: dict,
    low_pos: np.ndarray,
    low_rot: np.ndarray,
    cur_tloc_deg: float,
    z_top_surf: float = 0.147,
) -> None:
    """Right arm pinches the 16 mm T-Loc wing and turns CLOSE (0 deg) -> CONNECT (+90 deg); both go ready."""
    Move = kit.Move
    lx, ly = float(low_pos[0]), float(low_pos[1])
    x_brace = min(0.392, lx + 0.047)
    z_tch = min(0.148, z_top_surf + 0.0005)
    b_touch, b_rot, b_at = K.top_brace_tcp(x_brace, ly + 0.100, z_tch)
    brace_push = (2.0, 0.0, -8.5)
    start_deg = max(-15.0, min(20.0, cur_tloc_deg))
    end_deg = 92.0
    r_ready_pos, r_ready_rot = ready_pose("right")
    l_ready_pos, l_ready_rot = ready_pose("left")
    far_s2_tcp, app_rot = K.tloc_bar_tcp(start_deg, back_m=0.082, low_pos=low_pos, low_rot=low_rot)
    mid_rot = K.frame(low_rot @ np.array([0.0, -1.0, 0.0]), low_rot @ K.TLOC_APP)
    app_tcp, _ = K.tloc_bar_tcp(start_deg, back_m=0.040, low_pos=low_pos, low_rot=low_rot)
    in_tcp, in_rot = K.tloc_bar_tcp(start_deg, back_m=0.0, low_pos=low_pos, low_rot=low_rot)
    right_moves = [
        Move(0.50, far_s2_tcp, mid_rot, GRIP_BAR_OPEN, "unwind_wrist_mid"),
        Move(0.50, far_s2_tcp, app_rot, GRIP_BAR_OPEN, "unwind_wrist_s2"),
        Move(0.45, app_tcp, app_rot, GRIP_BAR_OPEN, "align_tloc_bar"),
        Move(0.55, in_tcp, in_rot, GRIP_BAR_OPEN, "onto_tloc_bar"),
        Move(0.55, in_tcp, in_rot, GRIP_BAR_PINCH, "pinch_tloc_bar"),
    ]
    for k in range(1, 8):
        a = start_deg + (end_deg - start_deg) * (k / 7.0)
        tcp_a, rot_a = K.tloc_bar_tcp(a, back_m=0.0, low_pos=low_pos, low_rot=low_rot)
        right_moves.append(Move(0.32, tcp_a, rot_a, GRIP_BAR_PINCH, f"turn_connect_{a:.0f}"))
    end_tcp, end_rot = K.tloc_bar_tcp(end_deg, back_m=0.0, low_pos=low_pos, low_rot=low_rot)
    back_tcp, _ = K.tloc_bar_tcp(90.0, back_m=0.095, low_pos=low_pos, low_rot=low_rot)

    right_moves.extend(
        [
            Move(0.35, end_tcp, end_rot, GRIP_BAR_PINCH, "settle_connect"),
            Move(0.45, end_tcp, end_rot, GRIP_BAR_OPEN, "release_tloc"),
            Move(0.60, back_tcp, end_rot, GRIP_BAR_OPEN, "clear_tloc"),
            Move(1.50, r_ready_pos, r_ready_rot, 1.0, "return_ready"),
            Move(2.50, r_ready_pos, r_ready_rot, 1.0, "hold_ready"),
        ]
    )
    b_clear, _, _ = K.top_brace_tcp(x_brace - 0.015, ly + 0.100, min(0.174, z_top_surf + 0.027))
    t_brace = round(sum(mv.seconds for mv in right_moves[:-2]), 4)
    left_moves = [
        Move(t_brace, b_touch, b_rot, GRIP_CLOSED, "hold_brace_s2", push=brace_push, push_at=b_at),
        Move(0.30, b_touch, b_rot, GRIP_CLOSED, "unpress_brace", push=(0.0, 0.0, 0.0), push_at=b_at),
        Move(0.50, b_clear, b_rot, GRIP_CLOSED, "release_brace", push=(0.0, 0.0, 0.0)),
        Move(1.45, l_ready_pos, l_ready_rot, 1.0, "return_ready"),
        Move(1.80, l_ready_pos, l_ready_rot, 1.0, "hold_ready"),
    ]
    arms["left"].moves.extend(left_moves)
    arms["right"].moves.extend(right_moves)


def nominal_plan() -> dict[str, list[kit.Move]]:
    class _ArmStub:
        def __init__(self, mvs: list[kit.Move]) -> None:
            self.moves = list(mvs)

    arms = {s: _ArmStub(mvs) for s, mvs in stage12_moves().items()}
    p_low = np.array([P.CASE_LOW_XY[0], P.CASE_LOW_XY[1], K.TOP], dtype=float)
    last_x = append_stage3_carry(arms, P.CASE_TOP_XY[0], P.CASE_TOP_XY[1], P.CASE_LOW_XY[0], P.CASE_LOW_XY[1])
    append_stage3_slide(arms, last_x, P.CLEAT_SLIDE - CARRY_X_TRIM_M, P.CASE_LOW_XY[1])
    append_stage4_s1(arms, p_low, np.eye(3), P.TLOC_OPEN_DEG)
    append_stage4_s2(arms, p_low, np.eye(3), P.TLOC_CLOSE_DEG)
    return {s: arms[s].moves for s in ("left", "right")}


def _robot_force_on(mk, body_name: str, side: str) -> float:
    tag = f"{side} robot"
    return round(sum(abs(float(c.force)) for c in mk.contacts(body_name) if c.other == tag), 2)


def save_end_pictures(frame: np.ndarray, folder: Path) -> None:
    import cv2

    bgr = cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR)
    cv2.imwrite(str(folder / "end_top.png"), bgr)
    cv2.imwrite(str(folder / "end_top_policy.png"), cv2.resize(bgr, (K.POLICY_W, K.POLICY_H), interpolation=cv2.INTER_AREA))


def run_demo(out_dir: Path, *, record: bool) -> dict:
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk
    folder = out_dir / "full"
    init_moves = stage12_moves()
    phase = {"step": "stage3_carry", "last_x_cmd": 0.0, "latest": {}}

    def director(t: float, arms: dict, pictures) -> None:
        if pictures is not None:
            phase["latest"].update(pictures)
        if not mk.MECH.poses:
            return
        ready_next = t >= max(arms["left"].seconds, arms["right"].seconds) - 1e-6
        if phase["step"] == "stage3_carry" and ready_next:
            p_top, _ = mk.pose("case_top")
            p_low, _ = mk.pose("case_low")
            phase["last_x_cmd"] = append_stage3_carry(arms, float(p_top[0]), float(p_top[1]), float(p_low[0]), float(p_low[1]))
            phase["step"] = "stage3_slide"
        elif phase["step"] == "stage3_slide" and ready_next:
            p_top, _ = mk.pose("case_top")
            p_low, _ = mk.pose("case_low")
            append_stage3_slide(arms, phase["last_x_cmd"], float(p_low[0] - p_top[0]), float(p_low[1]))
            phase["step"] = "stage4_s1"
        elif phase["step"] == "stage4_s1" and ready_next:
            p_low, r_low = mk.pose("case_low")
            z_surf = float(mk.pose("case_top")[0][2] - K.TOP) + P.CASE_H
            append_stage4_s1(arms, p_low, r_low, m.tloc_deg(), z_top_surf=z_surf)
            phase["step"] = "stage4_s2"
        elif phase["step"] == "stage4_s2" and ready_next:
            p_low, r_low = mk.pose("case_low")
            z_surf = float(mk.pose("case_top")[0][2] - K.TOP) + P.CASE_H
            append_stage4_s2(arms, p_low, r_low, m.tloc_deg(), z_top_surf=z_surf)
            phase["step"] = "done"

    def extra() -> dict:
        sm = m.stack_metrics()
        _, box_tilt = m.box_in_low()
        return {
            "lid_deg": round(m.lid_deg(), 2),
            "tloc_deg": round(m.tloc_deg(), 2),
            "tloc_spring_nm": round(P.tloc_torque_nm(m.tloc_angle_rad()), 3),
            "box_stowed": m.box_stowed(),
            "box_tilt_deg": round(box_tilt, 1),
            "case_top_x_mm": round(1000.0 * float(mk.pose("case_top")[0][0]), 1),
            "case_top_z_mm": round(1000.0 * float(mk.pose("case_top")[0][2] - K.TOP), 1),
            "top_tilt_deg": round(sm["tilt_deg"], 2),
            "dx_mm": round(1000.0 * sm["dx_m"], 1),
            "seam_mm": round(1000.0 * sm["max_seam_m"], 2),
            "cleat_overlap_mm": round(1000.0 * sm["min_cleat_overlap_m"], 1),
            "left_on_top_n": _robot_force_on(mk, "case_top", "left"),
            "right_on_top_n": _robot_force_on(mk, "case_top", "right"),
            "left_brace_low_n": _robot_force_on(mk, "case_low", "left"),
            "right_on_tloc_n": _robot_force_on(mk, "tloc_low", "right"),
            "hands_clear_mm": round(1000.0 * m.hands_clear_m(), 1),
        }

    res = kit.run(
        embodiment,
        init_moves,
        folder,
        seed=0,
        director=director,
        extra=extra,
        tail_s=0.6,
        limit_s=float(m.RUN_LIMIT_STEPS) / P.CONTROL_HZ,
        sheet_frames=16,
        record=record,
    )
    if "top_cam" in phase["latest"]:
        save_end_pictures(phase["latest"]["top_cam"], folder)
    rows = res["rows"]
    first_025 = next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.25), None)
    first_050 = next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.50), None)
    first_100 = next((r["t"] for r in rows if r["info"].get("score", 0.0) == 1.00), None)
    carry_trace = [
        {
            "t": r["t"],
            "act": r["left"],
            "top_x_mm": r["extra"]["case_top_x_mm"],
            "top_z_mm": r["extra"]["case_top_z_mm"],
            "tilt_deg": r["extra"]["top_tilt_deg"],
            "dx_mm": r["extra"]["dx_mm"],
            "seam_mm": r["extra"]["seam_mm"],
            "cleat_mm": r["extra"]["cleat_overlap_mm"],
            "L_wall_N": r["extra"]["left_on_top_n"],
            "R_wall_N": r["extra"]["right_on_top_n"],
        }
        for r in rows[::5]
        if r["extra"]["left_on_top_n"] > 0.5 or r["extra"]["right_on_top_n"] > 0.5
    ]
    tloc_trace = [
        {
            "t": r["t"],
            "act": r["right"],
            "tloc_deg": r["extra"]["tloc_deg"],
            "spring_nm": r["extra"]["tloc_spring_nm"],
            "R_tloc_N": r["extra"]["right_on_tloc_n"],
            "L_brace_N": r["extra"]["left_brace_low_n"],
        }
        for r in rows[::4]
        if r["extra"]["right_on_tloc_n"] > 0.5
    ]
    readings = {
        "score_end": rows[-1]["info"].get("score"),
        "steps": res["steps"],
        "seconds": res["seconds"],
        "first_025_s": first_025,
        "first_050_s": first_050,
        "first_100_s": first_100,
        "both_arms_ready_end": {s: rows[-1][s] in ("hold_ready", "done") for s in ("left", "right")},
        "hands_clear_end_mm": rows[-1]["extra"]["hands_clear_mm"],
        "carry_and_slide_trace": carry_trace,
        "tloc_turn_trace": tloc_trace,
        "report": res.get("report"),
        "worst_ik": res.get("worst_ik"),
        "median_step_ms": res.get("median_step_ms"),
    }
    (folder / "readings.json").write_text(json.dumps(readings, indent=2, default=str))
    print("READINGS " + json.dumps({k: v for k, v in readings.items() if not k.endswith("_trace")}, default=str), flush=True)
    return readings


def main() -> int:
    if "--dry" in sys.argv or "--quick" in sys.argv:
        from yam_arena import station

        offline = K.run_offline()
        dry = kit.dry_run(nominal_plan(), station.BASES, hz=P.CONTROL_HZ, tail_s=0.6, limit_s=P.RUN_LIMIT_STEPS / P.CONTROL_HZ)
        print("DRY " + json.dumps({"offline_pass": offline["pass"], "dry_run": dry}, indent=2), flush=True)
        return 0 if offline["pass"] else 1
    args = [a for a in sys.argv[1:] if a not in ("--check", "--no-video", "--norec")]
    if not args:
        raise SystemExit("usage: demo.py <out_dir> [--check] [--dry]")
    record = not any(flag in sys.argv for flag in ("--check", "--no-video", "--norec"))
    readings = run_demo(Path(args[0]), record=record)
    if readings["score_end"] != 1.0:
        raise SystemExit(f"demo finished with score {readings['score_end']} != 1.0: {readings['report']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
