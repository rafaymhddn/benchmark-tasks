#!/usr/bin/env python3
"""Scripted full-score reference run for ``bench_can`` (Gate 3; zero ``set_joint`` or ``write_pose``).

    scripts/rta demo <id> mech/tasks/bench_can [--no-video] [--quick]

1. Right hand lifts ``brg_screen`` (Ø 62 x 25.4 mm SKF 3305 A-2Z) off the Eagle B-601 brass dasher screen and places it
   in the back half of the Bochem 8480 stainless tray.
2. Right hand picks ``brg_mat`` off the black nitrile pad, centres it on the dasher screen, presses straight down
   >= 12 mm against the 18 N coil spring for ~1.3 s, lets the screen spring back up, and places ``brg_mat`` in the front
   half of the tray.
3. Right hand braces the can's welded front horizontal D-handle while Left hand pinches the upturned 172 deg saucer lid
   skirt up to ~147 deg, pushes the propped lid's back up to ~114 deg, sweeps the upper lid across top dead centre
   (90 deg, z <= 159.5 mm <= 160 mm) from above the rim so no finger is trapped between lid and rim, and presses the lid
   flat shut (0 deg) from above with its open hand (wrists >= 294 mm apart throughout). Both arms return to READY.
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
CLOSED = 0.0
BRG_OPEN = K.BRG_OPEN_M / K.MAX_JAW
BRG_PINCH = K.BRG_PINCH_M / K.MAX_JAW
HANDLE_OPEN = K.HANDLE_OPEN_M / K.MAX_JAW
HANDLE_PINCH = K.HANDLE_PINCH_M / K.MAX_JAW
SKIRT_OPEN = K.SKIRT_OPEN_M / K.MAX_JAW


def ready_pose(side: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, dtype=float), K.BASES[side])


def plan() -> dict[str, list]:
    import demo_kit as kit

    Move = kit.Move
    cx, cy = P.CAN_XY
    mx, my = P.MAT_XY
    b1x, b1y = P.TRAY_SLOT_BACK_XY
    b2x, b2y = P.TRAY_SLOT_FRONT_XY

    z_carry = K.TOP + 0.125
    # Under 0.35 kg brg_screen, dasher rests ~2.2 mm down -> bearing middle at TOP + 0.089 m
    z_screen_pick = K.TOP + 0.089
    z_plunge = z_screen_pick - 0.015  # 15 mm below resting height (>= 12 mm stroke)
    z_mat_pick = K.TOP + P.MAT_T + 0.5 * P.BRG_W + 0.002
    z_tray_put = K.TOP + P.TRAY_FLOOR + 0.5 * P.BRG_W + 0.003

    r_ready_p, r_ready_r = ready_pose("right")
    l_ready_p, l_ready_r = ready_pose("left")

    right = [
        # 1. Clear brg_screen from dasher into tray back slot
        Move(1.6, pos=(cx, cy, z_carry), rot=K.RIGHT_DOWN, grip=BRG_OPEN, label="above screen brg"),
        Move(0.9, pos=(cx, cy, z_screen_pick), label="on screen brg"),
        Move(0.6, grip=BRG_PINCH, label="pinch screen brg"),
        Move(0.9, pos=(cx, cy, z_carry), label="lift screen brg"),
        Move(1.4, pos=(b1x, b1y, z_carry), label="to tray back"),
        Move(1.0, pos=(b1x, b1y, z_tray_put), label="lower tray back"),
        Move(0.6, grip=BRG_OPEN, label="release brg 1"),
        Move(0.9, pos=(b1x, b1y, z_carry), label="clear tray back"),
        # 2. Pick brg_mat, press down >= 12 mm on dasher for 1.3 s, place in tray front slot
        Move(1.2, pos=(mx, my, z_carry), grip=BRG_OPEN, label="above mat brg"),
        Move(0.9, pos=(mx, my, z_mat_pick), label="on mat brg"),
        Move(0.6, grip=BRG_PINCH, label="pinch mat brg"),
        Move(0.9, pos=(mx, my, z_carry), label="lift mat brg"),
        Move(1.3, pos=(cx, cy, z_carry), label="over dasher"),
        Move(0.9, pos=(cx, cy, z_screen_pick + 0.002), label="touch dasher"),
        Move(0.8, pos=(cx, cy, z_plunge), push=(0.0, 0.0, -23.0), label="plunge dasher"),
        Move(1.3, pos=(cx, cy, z_plunge), push=(0.0, 0.0, -23.0), label="hold dip 1.3 s"),
        Move(0.9, pos=(cx, cy, z_carry), push=(0.0, 0.0, 0.0), label="let dasher rise"),
        Move(1.4, pos=(b2x, b2y, z_carry), label="to tray front"),
        Move(1.0, pos=(b2x, b2y, z_tray_put), label="lower tray front"),
        Move(0.6, grip=BRG_OPEN, label="release brg 2"),
        Move(0.9, pos=(b2x, b2y, z_carry), label="clear tray front"),
        # 3. Brace can's front D-handle while Left closes 172 deg lid
        Move(1.6, pos=K.handle_grip(back=0.055), rot=K.RIGHT_BRACE_ROT, grip=HANDLE_OPEN, label="approach handle"),
        Move(0.9, pos=K.handle_grip(back=0.002), label="on handle"),
        Move(0.6, grip=HANDLE_PINCH, label="brace handle"),
    ]
    t_brace_ready = sum(mv.seconds for mv in right)  # ~23.7 s

    p172, r172 = K.lid_stave2_pose(172.0)
    p156, r156 = K.lid_stave2_pose(156.0)
    p141, r141 = K.lid_stave2_pose(141.0)
    p_unhook = p141 + np.array([0.038, 0.042, 0.006], dtype=float)
    p_mid_y = np.array([0.235, 0.245, K.TOP + 0.135], dtype=float)
    p_side_low = np.array([0.165, 0.235, K.TOP + 0.080], dtype=float)
    hinge_xy = K.scene.hinge_world_pos(K.TOP)[:2]
    p_sweep_start = np.array([hinge_xy[0] + 0.055 * K._U_H[0], hinge_xy[1] + 0.055 * K._U_H[1], K.TOP + 0.152], dtype=float)
    left = [
        Move(t_brace_ready - 2.0, pos=l_ready_p, rot=l_ready_r, grip=OPEN, label="wait ready"),
        # Step A: pinch Stave 2 (104 mm lever arm) at 172 deg, rotate with the lid up to ~146 deg (z <= 159 mm), release wide
        Move(2.0, pos=p172 + np.array([0.0, 0.020, 0.040]), rot=r172, grip=SKIRT_OPEN, label="above stave 2"),
        Move(0.8, pos=p172, rot=r172, label="around stave 2"),
        Move(0.6, grip=CLOSED, label="pinch stave 2"),
        Move(0.9, pos=p156, rot=r156, label="lift 156 deg"),
        Move(0.9, pos=p141, rot=r141, label="prop lid 141 deg"),
        Move(0.6, grip=OPEN, label="release stave 2"),
        # Clear radially outside y = +0.24 m, drop behind propped lid, and push it up to ~114 deg
        Move(0.7, pos=p_unhook, rot=r141, grip=OPEN, label="unhook stave 2"),
        Move(0.7, pos=p_mid_y, rot=r141, grip=CLOSED, label="clear +y rim"),
        Move(0.8, pos=p_side_low, rot=K.LEFT_PUSH_ROT, grip=CLOSED, label="drop +y low"),
        Move(0.8, pos=K.lid_back_push_tcp(146.0, 0.052, 0.060), rot=K.LEFT_PUSH_ROT, grip=CLOSED, label="behind propped lid"),
        Move(0.9, pos=K.lid_back_push_tcp(118.0, 0.052, -0.004), label="push 118 deg"),
        Move(0.7, pos=K.lid_back_push_tcp(132.0, 0.052, 0.045), label="step back 114"),
        # Step B & C: level hand at z = 152 mm (47 mm clear above can rim) sweeps upper lid across 90 deg TDC, then presses shut
        Move(1.0, pos=p_sweep_start, rot=K.LEFT_PRESS_ROT, grip=OPEN, label="high behind 114"),
        Move(1.1, pos=K.lid_press_point(0.046), rot=K.LEFT_PRESS_ROT, grip=OPEN, label="sweep over TDC"),
        Move(1.1, pos=K.lid_press_point(0.000), rot=K.LEFT_PRESS_ROT, grip=OPEN,
             push=(0.0, 0.0, -16.0), label="press lid shut"),
        Move(0.6, pos=K.lid_press_point(0.000), push=(0.0, 0.0, -16.0), label="seat lid"),
        Move(0.8, pos=K.lid_press_point(0.045), push=(0.0, 0.0, 0.0), label="lift off lid"),
    ]
    t_lid_done = sum(mv.seconds for mv in left)
    right += [
        Move(max(0.5, t_lid_done - t_brace_ready), label="hold brace"),
        Move(0.6, grip=HANDLE_OPEN, label="unbrace handle"),
        Move(0.8, pos=K.handle_grip(back=0.055), grip=OPEN, label="back off handle"),
        Move(2.0, pos=r_ready_p, rot=r_ready_r, grip=OPEN, label="ready"),
    ]
    left += [Move(2.0, pos=l_ready_p, rot=l_ready_r, grip=OPEN, label="ready")]
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

    def extra() -> dict:
        d_m = m.dash_travel()
        return {
            "lid_deg": round(m.lid_deg(), 2),
            "dash_mm": round(1000.0 * d_m, 2),
            "dash_spring_n": round(P.DASH_PRELOAD_N + P.DASH_K * max(0.0, d_m), 2),
            "dasher_on_brg_mat_n": round(mk.contact_force("dasher", "brg_mat"), 2),
            "can_slide_mm": round(1000.0 * m.can_slide(), 2),
            "can_tilt_deg": round(m.can_tilt_deg(), 2),
            "tray_shift_mm": round(1000.0 * m.tray_shift(), 2),
            "dipped": bool(m.STATE.get("dipped", False)),
            "hands_clear_mm": round(1000.0 * m.hands_clear(), 1),
            "hold": m.STATE.get("hold", 0),
        }

    folder = out / "full"
    record = "--no-video" not in sys.argv[2:] and "--check" not in sys.argv[2:]
    latest: dict = {}

    def director(_t, _arms, pictures) -> None:
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
        tail_s=4.5,
        sheet_frames=16,
        record=record,
    )
    rows = result["rows"]
    if "top_cam" in latest:
        save_end_picture(latest["top_cam"], folder)
    first_quarter = next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.25), None)
    first_half = next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.5), None)
    first_full = next((r["t"] for r in rows if r["info"].get("score", 0.0) == 1.0), None)
    plunge_trace = [
        {
            "t": r["t"],
            "dash_mm": r["extra"]["dash_mm"],
            "spring_n": r["extra"]["dash_spring_n"],
            "brg_on_dash_n": r["extra"]["dasher_on_brg_mat_n"],
        }
        for r in rows
        if r["extra"]["dash_mm"] > 3.0
    ][::4]
    readings = {
        "end_score": rows[-1]["info"].get("score") if rows else 0.0,
        "seconds": rows[-1]["t"] if rows else 0.0,
        "first_0.25_at_s": first_quarter,
        "first_0.5_at_s": first_half,
        "first_1.0_at_s": first_full,
        "peak_dash_mm": max((r["extra"]["dash_mm"] for r in rows), default=0.0),
        "peak_dasher_on_brg_mat_n": max((r["extra"]["dasher_on_brg_mat_n"] for r in rows), default=0.0),
        "max_can_slide_mm": max((r["extra"]["can_slide_mm"] for r in rows), default=0.0),
        "max_can_tilt_deg": max((r["extra"]["can_tilt_deg"] for r in rows), default=0.0),
        "end_lid_deg": rows[-1]["extra"]["lid_deg"] if rows else None,
        "end_dash_mm": rows[-1]["extra"]["dash_mm"] if rows else None,
        "end_hands_clear_mm": rows[-1]["extra"]["hands_clear_mm"] if rows else 0.0,
        "both_arms_ready_at_end": {s: rows[-1][s] == "done" for s in ("left", "right")} if rows else {},
        "plunge_trace": plunge_trace,
        "report": result.get("report"),
        "worst_ik": result.get("worst_ik"),
        "median_step_ms": result.get("median_step_ms"),
    }
    (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
    print("READINGS " + json.dumps(readings, default=str), flush=True)
    return 0 if readings["end_score"] == 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
