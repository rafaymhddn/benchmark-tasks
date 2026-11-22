#!/usr/bin/env python3
"""Synchronized bimanual demonstration for ``tympanic_cover``.

    scripts/rta demo <id> mech/tasks/tympanic_cover [--check] [--no-video] [--quick]

Right stays on ``-y`` and Left on ``+y`` throughout:
1. Right comes down onto the thermometer's tail from behind and above (60 deg), grips it and lifts it off the rest.
2. Right holds the covered probe over the grey tray; Left lays the flat end of its closed fingertips on the eject button
   (on the head's end, above the probe) and presses it in along its axis (BTN_PRESS_N = 9 N told; the 900 N/m spring
   takes 3.6 N at the 4.0 mm release and 4.95 N at the 5.5 mm stop) while Right braces the same 9 N against it. Both
   arms give under load (in run gt26, 6 N told each way moved the button 4.4 mm, a 3.9 N spring force, while the right
   hand gave 3.6 mm), hence 9 N; the amber cover slides off into the tray.
3. Left grips the cradle's 56 mm side walls near the pocket end; Right tips the thermometer 35 deg nose-down so the
   probe stands upright, lowers it into the rack cover at well LOAD_WELL until the ring meets the cover's mouth and
   presses with SEAT_PUSH_N = 5.5 N told (the seat needs 3.5 N), then lifts it out with the white cover on.
4. Left lets go and returns to ready; Right lays the thermometer level along the cradle, its tail on the end wall and
   its head on the rib, lets go, backs off along its approach and returns to ready.
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

SETTLE_S = 3.0
SEAT_PUSH_N = 5.5  # N told at the probe's tip while the ring sits on the cover's mouth
CRADLE_PUSH_N = 3.0  # N the left hand presses the cradle down with while it holds it


def plan() -> dict[str, list]:
    import demo_kit as kit
    from yam_arena import ik
    from yam_arena.constants import READY

    Move = kit.Move
    top = np.array([0.0, 0.0, K.TOP])
    lv, ld = P.R_LEVEL_ROT, P.R_LOAD_ROT
    pick = np.array(P.R_PICK_TCP) + top
    eject_r = np.array(P.R_EJECT_TCP) + top
    load_r = np.array(P.R_LOAD_TCP) + top
    dock_r = np.array(P.R_DOCK_TCP) + top
    behind = K.BACK * lv[:, 2]  # back along the right hand's approach: behind and above the tail
    up = np.array([0.0, 0.0, 1.0])
    pre_l, touch_l, press_w = K.left_press_poses()
    grip_l = np.array(P.L_GRIP_TCP) + top
    a_grip = P.L_GRIP_ROT[:, 2]

    btn_face_t = np.array(P.BTN_POS_LOCAL) - (P.BTN_SIZE[1] / 2.0) * P.BTN_AXIS_T
    at_btn_r = lv.T @ (P.therm_rot(lv) @ (btn_face_t - P.GRASP_T))  # the button, in the right gripper's frame
    at_tip_r = ld.T @ (P.therm_rot(ld) @ (P.CONE_TIP_T - P.GRASP_T))  # the probe's tip, tipped for the load
    at_tip_l = tuple(K.FINGER_END)  # the closed fingertips' flat end, on the button

    ready_r_pos, ready_r_rot = ik.tcp_pose(np.array(READY, dtype=float), K.BASES["right"])
    ready_l_pos, ready_l_rot = ik.tcp_pose(np.array(READY, dtype=float), K.BASES["left"])
    open_h, grip_h = K.OPEN_HANDLE / K.MAX_JAW, K.GRIP_HANDLE / K.MAX_JAW
    open_c, grip_c = K.OPEN_CRADLE / K.MAX_JAW, K.GRIP_CRADLE / K.MAX_JAW

    right = [
        # 1 (0.0 -> 3.6 s): fingers over the tail from behind, close, lift
        Move(1.4, pos=pick - 1.6 * behind, rot=lv, grip=open_h, label="behind and above the tail"),
        Move(0.6, pos=pick - behind, rot=lv, grip=open_h, label="lined up with the tail"),
        Move(0.9, pos=pick, rot=lv, grip=open_h, label="fingers onto the tail"),
        Move(0.7, grip=grip_h, label="grasp the tail"),
        Move(0.8, pos=pick + 0.050 * up, rot=lv, label="lift the thermometer"),
        # 2 (4.4 -> 9.2 s): the covered probe over the tray, braced while the left presses the button
        Move(1.4, pos=eject_r, rot=lv, label="probe over the tray"),
        Move(0.8, pos=eject_r, rot=lv, push=(0.0, 0.0, 0.0), push_at=at_btn_r, label="settle over the tray"),
        # braced only while the left hand presses (6.6 -> 7.6 s), eased off with it: a brace nothing opposes throws it
        Move(1.0, pos=eject_r, rot=lv, push=tuple(-P.BTN_PRESS_N * press_w), push_at=at_btn_r, label="brace against the press"),
        Move(0.8, pos=eject_r, rot=lv, push=(0.0, 0.0, 0.0), push_at=at_btn_r, label="ease off with the left"),
        Move(0.8, pos=eject_r, rot=lv, push=(0.0, 0.0, 0.0), label="let the cover fall"),
        # 3 (9.2 -> 16.0 s): probe upright over the rack, into the cover, press, lift out
        Move(1.6, pos=load_r + 0.045 * up, rot=ld, label="probe upright over the rack"),
        Move(0.8, pos=load_r + 0.018 * up, rot=ld, label="tip over the cover"),
        Move(1.0, pos=load_r + 0.001 * up, rot=ld, push=(0.0, 0.0, 0.0), push_at=at_tip_r, label="ring onto the mouth"),
        Move(1.2, pos=load_r - 0.003 * up, rot=ld, push=(0.0, 0.0, -SEAT_PUSH_N), push_at=at_tip_r, label="press the cover on"),
        Move(0.5, pos=load_r + 0.002 * up, rot=ld, push=(0.0, 0.0, 0.0), label="ease"),
        Move(1.3, pos=load_r + 0.045 * up, rot=ld, label="lift the covered probe out"),
        Move(0.6, pos=load_r + 0.045 * up, rot=ld, label="wait for the left hand"),
        # 4 (16.0 -> 22.3 s): level along the cradle, let go, back off, ready
        Move(1.6, pos=dock_r + 0.050 * up, rot=lv, label="over the cradle"),
        Move(1.2, pos=dock_r + 0.001 * up, rot=lv, label="lay it in the cradle"),
        Move(0.7, grip=open_h, label="let go of the tail"),
        Move(0.9, pos=dock_r - behind, rot=lv, grip=open_h, label="back off along the approach"),
        Move(1.6, pos=ready_r_pos, rot=ready_r_rot, grip=1.0, label="right ready"),
    ]

    left = [
        # 1 (0.0 -> 3.6 s): close the fingertips, come over the button's side
        Move(1.8, pos=ready_l_pos, rot=ready_l_rot, grip=0.0, label="close fingertips"),
        Move(1.8, pos=pre_l, rot=P.L_PRESS_ROT, grip=0.0, label="over the button"),
        # 2 (3.6 -> 9.2 s): wait for the thermometer, press the button (aimed 2 mm past its 5.5 mm stop), let go
        Move(2.3, pos=pre_l, rot=P.L_PRESS_ROT, grip=0.0, label="wait for the thermometer"),
        Move(0.7, pos=touch_l, rot=P.L_PRESS_ROT, grip=0.0, push=(0.0, 0.0, 0.0), push_at=at_tip_l, label="touch the button"),
        Move(1.0, pos=touch_l + 0.0095 * press_w, rot=P.L_PRESS_ROT, grip=0.0, push=P.BTN_PRESS_N * press_w, push_at=at_tip_l, label="press the button"),
        Move(0.8, pos=pre_l, rot=P.L_PRESS_ROT, grip=0.0, push=(0.0, 0.0, 0.0), label="let the button go"),
        # 3 (9.2 -> 15.3 s): grip the cradle's side walls near the pocket end and hold it down
        Move(1.4, pos=grip_l - 0.050 * a_grip, rot=P.L_GRIP_ROT, grip=open_c, label="over the cradle's pocket end"),
        Move(0.8, pos=grip_l, rot=P.L_GRIP_ROT, grip=open_c, label="around the side walls"),
        Move(0.6, grip=grip_c, push=(0.0, 0.0, 0.0), push_at=(0.0, 0.0, 0.0), label="grip the cradle"),
        Move(3.6, pos=grip_l, rot=P.L_GRIP_ROT, grip=grip_c, push=(0.0, 0.0, -CRADLE_PUSH_N), label="hold the cradle"),
        # 4 (15.3 -> 18.7 s): let go, lift off, ready
        Move(0.7, grip=open_c, push=(0.0, 0.0, 0.0), label="let go of the cradle"),
        Move(0.9, pos=grip_l - 0.065 * a_grip, rot=P.L_GRIP_ROT, grip=1.0, label="lift off the cradle"),
        Move(1.6, pos=ready_l_pos, rot=ready_l_rot, grip=1.0, label="left ready"),
    ]
    return {"left": left, "right": right}


def save_pictures(first: dict, last: dict, folder: Path) -> None:
    import cv2
    import demo_kit as kit

    for cam, stem in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        bgr = cv2.cvtColor(np.asarray(first[cam]), cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(folder / f"{stem}.png"), bgr)
        cv2.imwrite(str(folder / f"{stem}_policy.png"), cv2.cvtColor(kit.policy_picture(np.asarray(first[cam])), cv2.COLOR_RGB2BGR))
    for cam, stem in (("top_cam", "end_top"), ("left_cam", "end_left_wrist"), ("right_cam", "end_right_wrist")):
        bgr = cv2.cvtColor(np.asarray(last[cam]), cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(folder / f"{stem}.png"), bgr)
        cv2.imwrite(str(folder / f"{stem}_policy.png"), cv2.cvtColor(kit.policy_picture(np.asarray(last[cam])), cv2.COLOR_RGB2BGR))


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else Path("/tmp/tympanic_demo")
    flags = set(sys.argv[1:])
    moves = plan()
    if "--quick" in flags:
        from yam_arena import station

        dry = kit.dry_run(moves, station.BASES)
        print("QUICK " + json.dumps(dry), flush=True)
        return 0

    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module

    def extra() -> dict:
        name = P.FRESH[P.LOAD_WELL]
        tp, tr = m.mk.pose("thermometer")
        cp, cr = m.mk.pose(name)
        loc = (tp + tr @ P.CONE_TIP_T - cp) @ cr
        rad, tz, tlt = m.cone_in_cover(name)
        return {
            "used in tray": m.used_in_tray(),
            "mounted fresh": m.mounted_fresh(),
            "docked": m.docked_in_cradle(),
            "load cover tip mm": [round(1000 * float(loc[0]), 1), round(1000 * float(loc[1]), 1), round(1000 * tz, 1)],
            "load cover (rad mm, deg, N)": [round(1000 * rad, 1), round(tlt, 1), round(float(m.mk.contact_force(name, "thermometer")), 2)],
            "btn mm": round(1000.0 * float(m.mk.joint("eject_slide")), 2),
            "hands clear mm": round(1000.0 * m.hands_clear(), 1),
            "hold": m.STATE.get("hold", 0),
            "zero": m.STATE.get("zero"),
        }

    folder = out / "full"
    record = "--no-video" not in flags and "--check" not in flags
    first_pics: dict = {}
    latest: dict = {}

    def director(_t, _arms, pictures) -> None:
        if pictures is not None:
            if not first_pics:
                first_pics.update(pictures)
            latest.update(pictures)

    result = kit.run(
        embodiment,
        moves,
        folder,
        seed=0,
        director=director,
        extra=extra,
        limit_s=float(m.RUN_LIMIT_STEPS) / 20.0,
        tail_s=SETTLE_S,
        sheet_frames=16,
        record=record,
    )
    rows = result["rows"]
    save_pictures(first_pics, latest, folder)
    first = {
        "0.5": next((r["t"] for r in rows if r["info"].get("score", 0.0) == 0.5), None),
        "1.0": next((r["t"] for r in rows if r["info"].get("score", 0.0) == 1.0), None),
    }
    end_score = float(rows[-1]["info"].get("score", 0.0))
    readings = {
        "score at the end": end_score,
        "seconds": rows[-1]["t"],
        "steps": len(rows),
        "first 0.5 at (s)": first["0.5"],
        "first 1.0 at (s)": first["1.0"],
        "report": m.report(),
        "worst IK": result.get("worst_ik"),
        "median step (ms)": result.get("median_step_ms"),
    }
    (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
    print("READINGS " + json.dumps(readings, default=str), flush=True)
    if "--check" in flags and end_score != 1.0:
        raise RuntimeError(f"demo --check failed: end score {end_score} != 1.0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
