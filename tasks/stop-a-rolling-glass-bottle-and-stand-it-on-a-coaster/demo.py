#!/usr/bin/env python3
"""Scripted reference demonstration for ``mech/tasks/rolling_bottle``:

    scripts/rta demo <id> mech/tasks/rolling_bottle [--check] [--quick]

- ``--check``: runs in the simulator without MP4 video encoding (``record=False``).
- ``--quick``: runs ``checks.py`` offline IK, joint-margin, collision-hull, and camera visibility checks only.

Sequence (arm actions only; zero state edits):
1. Right arm moves immediately to ``hover_gate`` over the rolling lane at ``y = -0.075 m`` (155 mm inside the open
   right cliff edge ``y = -0.230 m``) with 88 mm open jaws.
2. As the rolling bottle passes ``y <= -0.062 m``, the open jaws drop around it to cage and gently damp its roll on the
   flat worktop without bouncing it toward any edge, then open so the bottle settles at rest.
3. Reading the stopped bottle pose and the coaster pose (allowed for scripted demos by the Guide), the right
   hand aligns square over the stopped cylinder 116.5 mm from its base end (2 mm above COM), pinches across its 54 mm
   diameter (~35 N per pad), lifts with payload weight feed-forward, and pitches smoothly 90 degrees cap-up while
   carrying to the cork coaster at ``(0.205, -0.020)``; the bottle turns a few degrees behind the hand between the pads.
4. The hand squares the bottle by the largest share of its lean the wrist reaches (none when the cap leans away from
   the arm: the wrist is then at its limit), sets the low base rim on the cork floor where the upright base's rim
   belongs, lets that rim take the weight, and moves over the base centre so the bottle pivots upright on the rim. It
   opens only once the bottle reads still, retracts straight up to 50 mm over the cap, and returns to ``READY``.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as Ck  # noqa: E402
import demo_kit as kit  # noqa: E402
import parts as P  # noqa: E402
from yam_arena import constants as YC, ik, station  # noqa: E402
from yam_arena.constants import READY  # noqa: E402

CAGE_TRIGGER_Y = -0.062
GRIP_OPEN = Ck.GRIP_OPEN
GRIP_DAMP = round(min(1.0, (P.BOTTLE_D + 0.0015) / Ck.MAX_JAW_M), 3)
GRIP_MID = round(GRIP_DAMP + 0.35 * (GRIP_OPEN - GRIP_DAMP), 3)  # first-stage ease-open (~6 mm pad clearance a side)
GRIP_PINCH = Ck.GRIP_PINCH
SQUARE_FRACTIONS = (1.0, 0.75, 0.5, 0.25)  # share of the measured tilt the hand corrects before set-down, largest reachable
SQUARE_MIN_MARGIN_DEG = 2.0  # deg: pushed joint targets stay this far inside the joint limits along the squaring path


def press_margin_deg(q: np.ndarray, base, push) -> float:
    """Smallest distance (deg) of the pushed joint targets q + J^T F / stiffness from the joint limits."""
    pressed = np.asarray(q, dtype=float) + (kit.point_jacobian(q, base).T @ np.asarray(push, dtype=float)) / kit.JOINT_STIFFNESS
    return float(np.degrees(np.minimum(pressed - ik.LOWER, ik.UPPER - pressed)).min())


def path_reachable(q0, base, waypoints, push, n: int = 10) -> bool:
    """Chained IK along the straight / slerp segments between (pos, rot) waypoints, as the arm streams them: True when
    every pose is reached within 0.5 mm and the pushed joint targets stay SQUARE_MIN_MARGIN_DEG inside the limits."""
    q = np.asarray(q0, dtype=float)
    prev_pos, prev_rot = waypoints[0]
    for pos, rot in waypoints[1:]:
        for k in range(1, n + 1):
            s = k / n
            q, e_pos, _ = ik.solve(prev_pos + s * (pos - prev_pos), ik.matrix_to_quat_xyzw(ik.slerp_matrix(prev_rot, rot, s)), q, base_pos=base, iters=60)
            if e_pos > 0.0005 or press_margin_deg(q, base, push) < SQUARE_MIN_MARGIN_DEG:
                return False
        prev_pos, prev_rot = pos, rot
    return True


def run_demo(out_dir: Path, *, record: bool) -> dict:
    Move = kit.Move
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    mk = m.mk

    b2tcp = Ck.base_to_tcp_m()
    z_hover = P.WORKTOP_TOP_Z + P.BOTTLE_D + Ck.HOVER_CLEAR_OVER_BOTTLE_M + float(YC.PAD_TIP_PAST_TCP)
    z_cage = P.WORKTOP_TOP_Z + P.BOTTLE_R
    z_lift_low = P.TOP + 0.098
    z_over_coaster = P.TOP + Ck.PITCH_APEX_OVER_BENCH_M

    rot_down = Ck.R_PINCH_DOWN
    rot_up = Ck.pitch_down_to_upright(1.0)
    ready_tcp, ready_rot = ik.tcp_pose(np.array(READY, dtype=float), station.BASES["right"])
    carry_push = (0.0, 0.0, P.BOTTLE_MASS * P.GRAVITY)

    initial_moves = [
        Move(0.85, (P.ROLL_LANE_X, Ck.INTERCEPT_Y, z_hover), rot_down, GRIP_OPEN, "hover_gate"),
    ]
    stage: dict = {"phase": "wait_roll"}

    def director(t: float, arms: dict, _pictures) -> None:
        right = arms["right"]
        phase = stage["phase"]

        if phase == "wait_roll":
            if mk.MECH.poses and t >= 0.85:
                p_b, _ = mk.pose("bottle")
                if float(p_b[1]) <= CAGE_TRIGGER_Y:
                    right.moves.extend(
                        [
                            Move(0.15, (P.ROLL_LANE_X, Ck.INTERCEPT_Y, z_cage), rot_down, GRIP_OPEN, "cage_drop"),
                            Move(0.45, (P.ROLL_LANE_X, Ck.INTERCEPT_Y, z_cage), rot_down, GRIP_DAMP, "damp_spin"),
                            Move(0.25, (P.ROLL_LANE_X, Ck.INTERCEPT_Y, z_cage), rot_down, GRIP_OPEN, "open_stopped"),
                            Move(0.25, (P.ROLL_LANE_X, Ck.INTERCEPT_Y, z_hover), rot_down, GRIP_OPEN, "above_stopped"),
                        ]
                    )
                    stage["phase"] = "pick_stopped"
                    return
            if t < 5.0 and right.seconds <= t + 0.05 + 1e-6:
                right.moves.append(Move(0.05, (P.ROLL_LANE_X, Ck.INTERCEPT_Y, z_hover), rot_down, GRIP_OPEN, "watch_roll"))
            elif t >= 5.0:
                stage["phase"] = "done"

        elif phase == "pick_stopped" and t >= right.seconds - 1e-6:
            p_b, r_b = mk.pose("bottle")
            cap_ax = np.array([float(r_b[0, 2]), float(r_b[1, 2]), 0.0], dtype=float)
            ax_norm = float(np.linalg.norm(cap_ax))
            h_ax = cap_ax / ax_norm if ax_norm > 1e-6 else np.array([1.0, 0.0, 0.0], dtype=float)
            base_end = p_b - 0.5 * P.BOTTLE_H * h_ax
            grasp_xy = (base_end + b2tcp * h_ax)[:2]
            r_pinch_live = Ck.frame([h_ax[1], -h_ax[0], 0.0], [0.0, 0.0, -1.0])

            moves_pick = [
                Move(0.45, (float(grasp_xy[0]), float(grasp_xy[1]), z_hover), r_pinch_live, GRIP_OPEN, "align_stopped"),
                Move(0.55, (float(grasp_xy[0]), float(grasp_xy[1]), z_cage), r_pinch_live, GRIP_OPEN, "lower_pinch"),
                Move(0.60, (float(grasp_xy[0]), float(grasp_xy[1]), z_cage), r_pinch_live, GRIP_PINCH, "pinch_bottle"),
                Move(0.25, (float(grasp_xy[0]), float(grasp_xy[1]), z_cage), r_pinch_live, GRIP_PINCH, "settle_pinch", push=carry_push),
                Move(0.55, (float(grasp_xy[0]), float(grasp_xy[1]), z_lift_low), r_pinch_live, GRIP_PINCH, "lift_horiz", push=carry_push),
            ]
            for s_blend, label in ((0.25, "pitch_22"), (0.50, "pitch_45"), (0.75, "pitch_68"), (1.00, "pitch_90")):
                xy = (1.0 - s_blend) * grasp_xy + s_blend * np.array([P.COASTER_X, P.COASTER_Y])
                z_arc = (
                    (1.0 - s_blend) * z_lift_low
                    + s_blend * z_over_coaster
                    + 0.032 * math.sin(math.pi * s_blend)
                )
                rot_s = Ck.rz(s_blend * math.radians(Ck.COASTER_APPROACH_YAW_DEG)) @ Ck.ry(-s_blend * 0.5 * math.pi) @ r_pinch_live
                moves_pick.append(
                    Move(0.65, (float(xy[0]), float(xy[1]), z_arc), rot_s, GRIP_PINCH, label, push=carry_push)
                )
            moves_pick.extend(
                [
                    Move(0.55, (P.COASTER_X, P.COASTER_Y, z_over_coaster), rot_up, GRIP_PINCH, "over_coaster", push=carry_push),
                    Move(0.85, (P.COASTER_X, P.COASTER_Y, z_over_coaster), rot_up, GRIP_PINCH, "steady_over_coaster", push=carry_push),
                ]
            )
            right.moves.extend(moves_pick)
            stage["phase"] = "place_coaster"

        elif phase == "place_coaster" and t >= right.seconds - 1e-6:
            _, r_b = mk.pose("bottle")
            ax_now = np.asarray(r_b[:, 2], dtype=float)
            cross = np.cross(ax_now, np.array([0.0, 0.0, 1.0], dtype=float))
            cn = float(np.linalg.norm(cross))
            tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, float(ax_now[2])))))
            rot_place, share = rot_up, 0.0
            if cn > 1e-6:
                # Square the bottle by the largest share of its measured tilt that the wrist reaches while carrying it:
                # the whole square-then-lower path is checked before it is sent (a residual lean settles on release).
                tilt_err = min(math.radians(12.0), math.radians(tilt_deg))
                over = np.array([P.COASTER_X, P.COASTER_Y, z_over_coaster])
                for frac in SQUARE_FRACTIONS:
                    cand = kit.about(cross / cn, frac * tilt_err) @ rot_up
                    if path_reachable(right.q, right.base, [(over, rot_up), (over, cand), (over - np.array([0.0, 0.0, 0.065]), cand)], carry_push):
                        rot_place, share = cand, frac
                        break
            stage["square"] = {"t_s": round(t, 2), "tilt_deg": round(tilt_deg, 2), "axis": [round(float(v), 3) for v in ax_now], "share": share}
            print(f"square_vertical {json.dumps(stage['square'])}", flush=True)
            stage["rot_place"] = rot_place
            right.moves.extend(
                [
                    Move(0.45, (P.COASTER_X, P.COASTER_Y, z_over_coaster), rot_place, GRIP_PINCH, "square_vertical", push=carry_push),
                    Move(0.30, (P.COASTER_X, P.COASTER_Y, z_over_coaster), rot_place, GRIP_PINCH, "hold_vertical", push=carry_push),
                ]
            )
            stage["phase"] = "touchdown"

        elif phase == "touchdown" and t >= right.seconds - 1e-6:
            # Set down as a person squares a bottle that leans in the hand: the low base rim touches the cork floor
            # where the upright base's rim belongs, the weight goes onto that rim, then the hand moves over the base
            # centre so the bottle pivots on the rim (turning between the pads) and stands flat on the coaster.
            rot_place = stage["rot_place"]
            p_b, r_b = mk.pose("bottle")
            p_c, _ = mk.pose("coaster")
            axis = np.asarray(r_b[:, 2], dtype=float)
            base_now = m.bottle_base_center(p_b, r_b)
            tcp_now = np.asarray(mk.gripper("right")[0], dtype=float)
            d_pinch = float(np.dot(tcp_now - base_now, axis))  # base centre to pinch point along the axis, as held
            cos_t = float(axis[2])
            sin_t = math.sqrt(max(0.0, 1.0 - cos_t * cos_t))
            lean = np.array([float(axis[0]), float(axis[1]), 0.0])
            lean = lean / np.linalg.norm(lean) if np.linalg.norm(lean) > 1e-6 else np.array([1.0, 0.0, 0.0])
            floor_c = np.array([float(p_c[0]), float(p_c[1]), float(p_c[2]) + P.COASTER_FLOOR_H])
            rim_goal = floor_c + P.BOTTLE_R * lean  # the low rim's point once the bottle stands centred
            base_touch = rim_goal - P.BOTTLE_R * np.array([lean[0] * cos_t, lean[1] * cos_t, -sin_t])
            cmd_off = np.array([P.COASTER_X, P.COASTER_Y, z_over_coaster]) - tcp_now  # command minus pinch point now
            lift = np.array([0.0, 0.0, Ck.PLACE_CLEAR_OVER_FLOOR_M])
            touch_pos = base_touch + d_pinch * axis + cmd_off + lift
            place_pos = floor_c + np.array([0.0, 0.0, d_pinch]) + cmd_off + lift
            stage["pivot"] = {"tilt_deg": round(math.degrees(math.atan2(sin_t, cos_t)), 2), "d_pinch_mm": round(1000.0 * d_pinch, 1),
                              "pivot_mm": [round(1000.0 * float(v), 1) for v in (place_pos - touch_pos)]}
            print(f"pivot_upright {json.dumps(stage['pivot'])}", flush=True)
            right.moves.extend(
                [
                    Move(0.95, tuple(touch_pos + np.array([0.0, 0.0, 0.004])), rot_place, GRIP_PINCH, "lower_to_coaster", push=carry_push),
                    Move(0.35, tuple(touch_pos), rot_place, GRIP_PINCH, "touch_rim", push=carry_push),
                    Move(0.40, tuple(touch_pos), rot_place, GRIP_PINCH, "load_rim", push=(0.0, 0.0, 0.0)),
                    Move(0.80, tuple(place_pos), rot_place, GRIP_PINCH, "pivot_upright", push=(0.0, 0.0, 0.0)),
                    Move(0.45, tuple(place_pos), rot_place, GRIP_PINCH, "transfer_weight", push=(0.0, 0.0, 0.0)),
                ]
            )
            place_pos = tuple(float(v) for v in place_pos)
            stage["place_pos"] = place_pos
            stage["phase"] = "settle_open"
            stage["still_steps"] = 0
            stage["settle_since"] = None

        elif phase == "settle_open":
            # Open the jaws only once the set-down bottle reads still (speed <= 5 mm/s, angular speed <= 0.08 rad/s for
            # 6 consecutive steps; after 3 s of waiting open regardless), holding the pose meanwhile. A bottle set down
            # with the small lean it took in the jaws (under 13 deg its centre of mass stays over its base) stands up
            # flat on its own base as the jaws ease open.
            if t >= right.seconds - 1e-6:
                if stage["settle_since"] is None:
                    stage["settle_since"] = t
                lin_v, ang_v = mk.velocity("bottle")
                settled = float(np.linalg.norm(lin_v)) <= 0.005 and float(np.linalg.norm(ang_v)) <= 0.08
                stage["still_steps"] = stage["still_steps"] + 1 if settled else 0
                place_pos = stage["place_pos"]
                rot_place = stage["rot_place"]
                if stage["still_steps"] < 6 and t - stage["settle_since"] < 3.0:
                    right.moves.append(Move(0.05, place_pos, rot_place, GRIP_PINCH, "wait_still", push=(0.0, 0.0, 0.0)))
                else:
                    right.moves.extend(
                        [
                            Move(0.40, place_pos, rot_place, GRIP_MID, "ease_open", push=(0.0, 0.0, 0.0)),
                            Move(0.40, place_pos, rot_place, GRIP_OPEN, "open_on_coaster", push=(0.0, 0.0, 0.0)),
                            Move(
                                0.95,
                                (place_pos[0], place_pos[1], P.TOP + Ck.retract_tcp_over_bench_m()),
                                rot_up,
                                GRIP_OPEN,
                                "retract_straight_up",
                            ),
                            Move(1.25, ready_tcp, ready_rot, GRIP_OPEN, "return_ready"),
                            Move(4.50, ready_tcp, ready_rot, GRIP_OPEN, "hold_ready"),
                        ]
                    )
                    stage["phase"] = "done"

    def step_extra() -> dict:
        p_b, r_b = mk.pose("bottle")
        lin_v, _ = mk.velocity("bottle")
        tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, float(r_b[2, 2])))))
        return {
            "bottle_xyz_m": [round(float(v), 4) for v in p_b],
            "bottle_axis": [round(float(v), 3) for v in r_b[:, 2]],
            "speed_mps": round(float(np.linalg.norm(lin_v)), 4),
            "tilt_up_deg": round(tilt_deg, 2),
            "f_support_n": round(m.support_contact_force_n(), 3),
            "f_coaster_n": round(float(mk.contact_force("bottle", "coaster")), 3),
            "tau_roll_nm": round(float(np.linalg.norm(m.STATE["last_tau_roll"])), 6),
        }

    res = kit.run(
        emb,
        {"right": initial_moves},
        out_dir / "demo",
        seed=0,
        director=director,
        tail_s=0.5,
        limit_s=P.RUN_LIMIT_STEPS / P.CONTROL_HZ,
        extra=step_extra,
        record=record,
    )
    save_end_picture(emb, out_dir / "demo")
    return res


def save_end_picture(emb, out: Path) -> None:
    """The top camera as the run left it, at the model's 320 x 176: the README's end-state picture."""
    import cv2

    try:
        rgb = emb._observation().images["top_cam"]  # the cameras re-read; nothing steps
    except Exception as exc:  # a picture only: never fail the demo over it
        print(f"end picture not written: {exc!r}", flush=True)
        return
    small = cv2.resize(rgb, (Ck.POLICY_W, Ck.POLICY_H), interpolation=cv2.INTER_AREA)
    cv2.imwrite(str(out / "end_top_policy.png"), cv2.cvtColor(small, cv2.COLOR_RGB2BGR))


def main() -> int:
    if "--quick" in sys.argv:
        print(json.dumps(Ck.run_offline_checks(), indent=2))
        return 0
    args = [a for a in sys.argv[1:] if a not in ("--check", "--norec")]
    if not args:
        raise SystemExit("usage: demo.py <out_dir> [--check] [--quick]")
    record = "--check" not in sys.argv and "--norec" not in sys.argv
    out_dir = Path(args[0])
    res = run_demo(out_dir, record=record)
    score = res["info"].get("score", 0.0)
    if score != 1.0:
        raise SystemExit(f"demo finished with score {score} != 1.0: {res['report']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
