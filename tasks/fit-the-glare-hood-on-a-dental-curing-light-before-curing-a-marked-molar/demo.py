#!/usr/bin/env python3
"""Scripted open-loop full-score reference run for the dental curing-light hood task.

    scripts/rta demo <id> mech/tasks/curing_hood [--no-video] [--quick]

Sequence (reads nothing from the simulator; every pose comes from ``checks.py``):
1. Right hand braces the 145 g Woodpecker LED.B barrel in its open U-cradle while Left hand pinches the real 3.0 mm
   oval wing of the orange light hood and lifts it out of its slot.
2. Left hand aligns the hood's 8.4 mm rubber-lined grommet coaxially with the 55 deg distal tip of the 8.0 mm glass
   fiber, guides it 55 deg around the circular elbow, and pushes it 22 mm home along the straight base against the
   6.0 N grommet resistance, then lets go and clears out.
3. Right hand lifts the shielded curing light, brings the 55 deg curved tip under the typodont's 45 mm propped-open
   upper arch to hover over the blue-marked lower right first molar (#46), while Left hand presses the 3.0 N top power
   button to run the 3.0 s blue curing cycle (>= 2.5 s shielded dwell cures the mark to ivory).
4. Right hand lays the shielded curing light back into its cradle, lets go, and both arms return to ready (1.0).
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as K  # noqa: E402
import parts as P  # noqa: E402

OPEN = 1.0
# Pinch targets (< 40 N per finger at 5,000 N/m stiffness):
# - 31 mm wand barrel -> target 19.0 mm gap -> 6.0 mm/finger * 5,000 N/m = 30.0 N/finger (<= 40 N cap)
WAND_GRIP = (P.WAND_W - 2.0 * 0.0060) / K.MAX_JAW
# - 3.0 mm real oval shield wing -> target 0 mm gap -> 1.47 mm/finger * 5,000 N/m = 7.35 N/finger (17.6 N two-pad slip)
HOOD_GRIP = 0.0
CLOSED = 0.0


def ready(side: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, dtype=float), K.BASES[side])


def plan() -> dict[str, list]:
    import demo_kit as kit

    Move = kit.Move
    pw0, rw0 = P.wand_start_pose(K.TOP)
    pwc, rwc = P.wand_cure_pose(K.TOP, hover_z=0.0075)
    # Exact 3D cure offset measured in chd8 so the shielded distal tip hovers ~6 mm centered over molar #46
    pwc = pwc + np.array([0.0154, 0.0036, 0.0177])
    ph0, rh0 = P.hood_start_pose(K.TOP)

    pr_c, rr_c = K.right_on_wand(pw0, rw0)
    pr_lift, _ = K.right_on_wand(pw0, rw0, lift_z=0.055)
    pr_cure, rr_cure = K.right_on_wand(pwc, rwc)
    from yam_arena import ik
    rr_mid = ik.slerp_matrix(rr_c, rr_cure, 0.55)
    pr_mouth = pr_cure - 0.025 * rwc[:, 0] + np.array([0.0, 0.0, 0.014])

    ps_h, rs_h = K.left_on_hood(ph0, rh0)
    _, r_tip_only = K.hood_on_fiber_pose(pw0, rw0, P.FIBER_BEND_DEG, 0.0)
    pt_h, rt_h = K.left_on_hood(*K.hood_on_fiber_pose(pw0, rw0, P.FIBER_BEND_DEG, 0.0))
    pa1_h, ra1_h = K.left_on_hood(*K.hood_on_fiber_pose(pw0, rw0, 37.0, 0.0))
    pa2_h, ra2_h = K.left_on_hood(*K.hood_on_fiber_pose(pw0, rw0, 18.0, 0.0))
    pe_h, re_h = K.left_on_hood(*K.hood_on_fiber_pose(pw0, rw0, 0.0, 0.0))
    # Overdrive target 6.0 mm along -y past the 22 mm stop so under 6 N arm compliance seat_s reaches full 22.0 mm stop
    pf_push, rf_h = K.left_on_hood(*K.hood_on_fiber_pose(pw0, rw0, 0.0, P.SEAT_TRAVEL + 0.0060))
    pf_ease, _ = K.left_on_hood(*K.hood_on_fiber_pose(pw0, rw0, 0.0, P.SEAT_TRAVEL))
    tip_shift = np.array([0.0035, -0.0054, 0.0005])
    pt_h, pa1_h, pa2_h, pe_h, pf_push, pf_ease = (p + tip_shift for p in (pt_h, pa1_h, pa2_h, pe_h, pf_push, pf_ease))
    p_off, r_off = K.left_on_hood(*K.hood_on_fiber_pose(pw0, rw0, 0.0, P.SEAT_TRAVEL), radial_extra=0.036)
    p_pre = pt_h + 0.016 * r_tip_only[:, 0]

    btn_shift = np.array([-0.0102, 0.0017, 0.0])
    pb_above, rb = K.left_on_button(pwc, rwc, above=0.028)
    pb_touch, _ = K.left_on_button(pwc, rwc, above=0.0010)
    pb_press, _ = K.left_on_button(pwc, rwc, above=-0.0070)
    pb_above, pb_touch, pb_press = pb_above + btn_shift, pb_touch + btn_shift, pb_press + btn_shift
    btn_tip = kit.lowest_point(rb)

    r_ready_p, r_ready_r = ready("right")
    l_ready_p, l_ready_r = ready("left")

    # Phase 1: Right hand braces wand in cradle while Left hand picks hood, threads 55-deg bend, and seats 22 mm
    left = [
        Move(1.6, pos=ps_h + np.array([0.0, 0.0, 0.040]), rot=rs_h, grip=OPEN, label="above hood"),
        Move(0.9, pos=ps_h, label="on hood wing"),
        Move(0.6, grip=HOOD_GRIP, label="pinch 3mm wing"),
        Move(0.9, pos=ps_h + np.array([0.0, 0.0, 0.034]), label="lift hood"),
        Move(1.6, pos=p_pre, rot=rt_h, label="align at tip"),
        Move(1.0, pos=pt_h, label="enter 8mm tip"),
        Move(0.4, pos=pt_h, label="hold tip entry"),
        Move(0.9, pos=pa1_h, rot=ra1_h, label="arc 37 deg"),
        Move(0.9, pos=pa2_h, rot=ra2_h, label="arc 18 deg"),
        Move(0.9, pos=pe_h, rot=re_h, label="arc 0 deg"),
        # Push 22 mm along straight base (-y) against 6.0 N rubber grommet drag, then ease push before releasing
        Move(1.8, pos=pf_push, rot=rf_h, push=(0.0, -8.5, 0.0), label="seat 22mm (6N)"),
        Move(0.4, pos=pf_ease, push=(0.0, 0.0, 0.0), label="ease push"),
        Move(0.6, grip=OPEN, label="release hood"),
        Move(1.0, pos=p_off, rot=r_off, label="clear of hood"),
        Move(1.6, pos=l_ready_p, rot=l_ready_r, grip=CLOSED, label="park for cure"),
    ]
    t_hood_done = sum(m.seconds for m in left) - 1.6  # 13.5 s: Left hand is clear of hood

    right = [
        Move(1.6, pos=pr_c + np.array([0.0, 0.0, 0.055]), rot=rr_c, grip=OPEN, label="above wand"),
        Move(0.9, pos=pr_c, label="on wand barrel"),
        Move(0.6, grip=WAND_GRIP, label="brace wand"),
        # Hold wand firmly against cradle saddles (+y reaction against Left hand's -y 6.0 N seating push)
        Move(6.6, push=(0.0, 0.0, -2.5), label="steady wand"),
        Move(2.2, push=(0.0, 6.0, -3.0), label="counter 6N push"),
    ]
    right += [Move(max(0.4, t_hood_done - sum(m.seconds for m in right)), push=(0.0, 0.0, -1.5), label="hold wand")]

    # Phase 2: Right hand lifts shielded wand and enters the 45 mm mouth opening cleanly to hover over molar #46
    right += [
        Move(1.1, pos=pr_lift, push=(0.0, 0.0, 0.0), label="lift shielded wand"),
        Move(1.6, pos=pr_mouth, rot=rr_mid, label="before mouth"),
        Move(1.0, pos=pr_cure, rot=rr_cure, push=(0.0, 0.0, 1.5), label="aim at molar #46"),
    ]
    t_at_molar = sum(m.seconds for m in right)

    # Left hand moves above top power button, presses 2.5 mm (3.0 N) to trigger 3.0 s blue beam, then retracts
    left += [Move(max(0.4, t_at_molar - sum(m.seconds for m in left)), label="wait aim")]
    left += [
        Move(1.4, pos=pb_above, rot=rb, grip=CLOSED, label="above button"),
        Move(0.7, pos=pb_touch, push_at=btn_tip, label="touch button"),
        Move(0.7, pos=pb_press, push=tuple(float(v) for v in (-6.5 * rwc[:, 2])), push_at=btn_tip, label="press 3N button"),
        Move(0.6, pos=pb_above, push=(0.0, 0.0, 0.0), label="lift off button"),
        Move(1.8, pos=l_ready_p, rot=l_ready_r, grip=OPEN, label="left ready"),
    ]

    # Right hand holds wand steady while button is pressed, then centers tip over molar #46 through the 3.5 s beam cycle
    t_press_done = t_at_molar + 1.4 + 0.7 + 0.7
    d_yaw = P._rz(12.0)
    rr_dwell = d_yaw @ rr_cure
    tip_to_grip = pr_cure - (pwc + rwc @ P.TIP_LOCAL_POS)
    pr_dwell = (
        pr_cure
        + np.array([-0.0241, 0.0218, 0.0045])
        + (d_yaw - np.eye(3)) @ tip_to_grip
        + np.array([-0.0086, 0.0115, 0.0095])
    )
    right += [
        Move(t_press_done - sum(m.seconds for m in right), pos=pr_cure, push=(0.0, 0.0, 2.2), label="brace press"),
        Move(0.45, pos=pr_dwell, rot=rr_dwell, push=(0.0, 0.0, 0.0), label="center on molar"),
        Move(P.BEAM_S + 0.3, pos=pr_dwell, rot=rr_dwell, push=(0.0, 0.0, 0.0), label="dwell cure 3.5s"),
    ]

    # Phase 3: Right hand withdraws out of mouth first, returns shielded wand to cradle, releases, and moves to ready
    right += [
        Move(1.0, pos=pr_mouth, rot=rr_mid, push=(0.0, 0.0, 0.0), label="out of mouth"),
        Move(1.8, pos=pr_lift, rot=rr_c, label="back over cradle"),
        Move(1.2, pos=pr_c + np.array([0.0, 0.0, 0.002]), push=(0.0, 0.0, 0.0), label="lay in cradle"),
        Move(0.6, grip=OPEN, label="release wand"),
        Move(1.0, pos=pr_c + np.array([0.0, 0.0, 0.058]), label="clear cradle"),
        Move(1.8, pos=r_ready_p, rot=r_ready_r, label="right ready"),
    ]
    return {"left": left, "right": right}


def save_end_picture(frame, folder: Path) -> None:
    import cv2
    import demo_kit as kit

    cv2.imwrite(str(folder / "end_top.png"), cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(folder / "end_top_policy.png"), cv2.cvtColor(kit.policy_picture(np.asarray(frame)), cv2.COLOR_RGB2BGR))


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    moves = plan()
    if "--quick" in sys.argv[2:]:
        from yam_arena import station

        dry = kit.dry_run(moves, station.BASES)
        print("QUICK " + json.dumps(dry), flush=True)
        return 0

    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk

    def finger_forces(body_name: str) -> dict[str, float]:
        totals: dict[str, float] = {}
        for c in mk.contacts(body_name):
            if c.other.endswith("robot"):
                totals[c.other] = round(totals.get(c.other, 0.0) + abs(float(c.force)), 2)
        return totals

    def extra() -> dict:
        xy_e, dz, tilt = m.tip_to_molar()
        in_c, c_mm, c_deg = m.wand_in_cradle()
        tp, tt = m.tip_world()
        ph, rh = mk.pose("hood")
        pw, rw = mk.pose("wand")
        btn_w = pw + rw @ np.array([P.BTN_X_LOCAL, 0.0, P.WAND_H / 2.0 + P.BTN_H])
        e_mm = round(1000.0 * float(np.linalg.norm(ph - tp)), 2)
        e_deg = round(float(np.degrees(np.arccos(np.clip(abs(float(rh[:, 0] @ tt)), 0.0, 1.0)))), 2)
        return {
            "hood_on": m.hood_on_rod(),
            "entry_mm_deg": [e_mm, e_deg],
            "entry_vec_mm": [round(1000.0 * float(v), 2) for v in (ph - tp)],
            "tip_vec_mm": [round(1000.0 * float(v), 2) for v in (tp - P.molar_world(K.TOP))],
            "btn_w_mm": [round(1000.0 * float(v), 2) for v in btn_w],
            "bend_deg": round(m.bend_progress_deg(), 2),
            "seat_mm": round(m.seat_mm(), 2),
            "seat_force_N": round(float(m.STATE.get("seat_force_n", 0.0)), 2),
            "btn_mm": round(1000.0 * m.btn_m(), 2),
            "beam_on": m.beam_on(),
            "cure_steps": m.STATE.get("cure_steps", 0),
            "tip_molar_mm_deg": [round(1000.0 * xy_e, 2), round(1000.0 * dz, 2), round(tilt, 1)],
            "cradle": [in_c, round(1000.0 * c_mm, 2), round(c_deg, 2)],
            "hood_fingers_N": finger_forces("hood"),
            "wand_fingers_N": finger_forces("wand"),
            "hands_clear_mm": round(1000.0 * m.hands_clear_m(), 1),
        }

    folder = out / "full"
    record = "--no-video" not in sys.argv[2:] and "--check" not in sys.argv[2:]
    latest: dict = {}

    def director(t, arms, pictures) -> None:
        if pictures is not None:
            latest.update(pictures)

    result = kit.run(
        embodiment,
        moves,
        folder,
        seed=0,
        director=director,
        extra=extra,
        limit_s=float(m.RUN_LIMIT_STEPS) / 20.0,
        tail_s=3.2,
        sheet_frames=16,
        record=record,
    )
    rows = result["rows"]
    if "top_cam" in latest:
        save_end_picture(latest["top_cam"], folder)

    # Extract force-vs-travel trace along the 22 mm seating stroke (Check 51)
    seat_trace = [
        {
            "t": r["t"],
            "bend_deg": r["extra"]["bend_deg"],
            "seat_mm": r["extra"]["seat_mm"],
            "grommet_N": r["extra"]["seat_force_N"],
            "hood_fingers_N": r["extra"]["hood_fingers_N"],
        }
        for r in rows
        if r["extra"]["hood_on"] and r["extra"]["seat_force_N"] > 0.2
    ][::3]

    first_05 = next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.5), None)
    first_075 = next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.75), None)
    first_10 = next((r["t"] for r in rows if r["info"].get("score", 0.0) == 1.0), None)

    readings = {
        "score_end": rows[-1]["info"].get("score") if rows else 0.0,
        "steps": len(rows),
        "seconds": rows[-1]["t"] if rows else 0.0,
        "first_0.5_s": first_05,
        "first_0.75_s": first_075,
        "first_1.0_s": first_10,
        "seat_force_vs_travel": seat_trace,
        "max_hood_finger_N": max((max(r["extra"]["hood_fingers_N"].values(), default=0.0) for r in rows), default=0.0),
        "max_wand_finger_N": max((max(r["extra"]["wand_fingers_N"].values(), default=0.0) for r in rows), default=0.0),
        "end_state": rows[-1]["extra"] if rows else {},
        "report": result.get("report"),
        "worst_ik": result.get("worst_ik"),
        "median_step_ms": result.get("median_step_ms"),
    }
    (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
    print("READINGS " + json.dumps(readings, default=str), flush=True)
    return 0 if readings["score_end"] == 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
