#!/usr/bin/env python3
"""Scripted bimanual full-score reference demo for holesaw_arbor.

    scripts/rta demo <id> mech/tasks/holesaw_arbor [--no-video] [--quick]

The left hand sets the hole saw on the thread start itself: after the lift it reads where the saw sits in its fingers
and corrects its own targets so the saw comes down coaxial and level over the arbor, slowly through the thread-start
height (the task catches the thread there, where the saw stands). The turns are rotations about the arbor axis from
where the hand is; the second turn ends where the holes line up, short of the stop. Nothing writes the saw's pose.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import checks as K  # noqa: E402
import parts as P  # noqa: E402

OPEN = 1.0
# Finger targets with FINGER_STIFFNESS = 5000 N/m:
# - Ø 42.0 mm collar -> 30 mm target (6.0 mm/finger = ~30 N/pad <= 40 N limit)
# - Ø 63.5 mm hole saw -> 52 mm target (5.75 mm/finger = ~28.8 N/pad <= 40 N limit)
COLLAR_GRIP = 0.030 / K.MAX_JAW
SAW_GRIP = 0.052 / K.MAX_JAW
ZERO_PUSH = (0.0, 0.0, 0.0)
APPROACH_SAW_YAW = math.radians(-30.0)  # the saw's yaw over the arbor: its thread start at +30 deg CW, mid-way between starts
APPROACH_TCP_YAW = APPROACH_SAW_YAW + K.SAW_YAW_START  # 20 deg: the grasp put the saw at yaw 0 under a 50 deg gripper
BOSS_HOVER_GAP = 0.0130  # m: backplate 1.0 mm over the 12 mm thread boss's top, its bore round the pilot nose
ENTRY_ABOVE = 0.0006  # m: first lower the backplate to 0.6 mm over its thread start
ENTRY_BELOW = 0.0001  # m: then slowly to 0.10 mm under it, inside the 0.15 mm band (the catch fires on the way)
STROKE1_DEG = 180.0  # first CW turn; the second ends where the holes line up (about 150 deg, short of the stop)


def ready(side: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, dtype=float), K.BASES[side])


def plan() -> dict[str, list]:
    import demo_kit as kit

    Move = kit.Move
    mat_xy = (P.X_MAT, P.Y_MAT)
    arb_xy = (P.X_ARBOR, P.Y_ARBOR)
    z_mat = P.MAT_SIZE[2]

    # Phase 1: Right arm pulls the detented collar down (8 -> 0 mm) and parks clear at -y; meanwhile Left arm grips hole_saw on mat
    right = [
        Move(1.8, pos=K.collar_tcp(P.PIN_STROKE, back=0.075)[0], rot=K.COLLAR_ROT, grip=OPEN, label="approach collar"),
        Move(0.9, pos=K.collar_tcp(P.PIN_STROKE)[0], label="around collar up"),
        Move(0.6, grip=COLLAR_GRIP, label="grip collar up"),
        Move(1.2, pos=K.collar_tcp(0.0, dz=-0.001)[0], push=(0.0, 0.0, -14.0), label="pull collar down"),
        Move(0.3, push=ZERO_PUSH, label="settle collar down"),
        Move(0.5, grip=OPEN, label="release collar down"),
        Move(1.1, pos=K.collar_tcp(0.0, back=0.125, dz=0.020)[0], label="park clear"),
    ]
    t_collar_ready = sum(m.seconds for m in right)  # 6.4 s

    left = [
        Move(2.2, pos=K.saw_tcp(mat_xy, z_mat + 0.055, K.SAW_YAW_START)[0], rot=kit.down(K.SAW_YAW_START), grip=OPEN, label="over mat"),
        Move(1.0, pos=K.saw_tcp(mat_xy, z_mat, K.SAW_YAW_START)[0], label="around saw on mat"),
        Move(0.7, grip=SAW_GRIP, label="grip saw"),
        Move(1.3, pos=K.saw_tcp(mat_xy, K.SAW_CARRY_Z, K.SAW_YAW_START)[0], label="lift saw"),
    ]
    # Ensure collar is fully retracted & right hand parked clear before left moves over the arbor
    if sum(m.seconds for m in left) < t_collar_ready:
        left.append(Move(round(t_collar_ready - sum(m.seconds for m in left), 3), label="wait collar clear"))

    # Phase 2: Left arm sets the cup on the thread start (coaxial, level, slowly), turns it 180 deg CW, regrasps, turns
    # it on until the holes line up over the pins (short of the stop), lets go. The poses here are the plan's; main()'s
    # director replaces them before each phase starts with poses from where the saw and the hand really are.
    gap0 = P.thread_start_gap(APPROACH_SAW_YAW)  # 1.509 mm
    tcp_yaw = lambda cw: APPROACH_TCP_YAW - math.radians(cw)  # noqa: E731
    z_on = lambda cw: P.Z_SHOULDER + gap0 - P.THREAD_PITCH * cw / 360.0  # noqa: E731
    left += [
        Move(1.5, pos=K.saw_tcp(arb_xy, K.SAW_CARRY_Z, APPROACH_TCP_YAW)[0], rot=kit.down(APPROACH_TCP_YAW), label="over arbor pilot"),
        Move(0.6, label="hover over pilot"),
        Move(1.2, pos=K.saw_tcp(arb_xy, P.Z_SHOULDER + BOSS_HOVER_GAP, APPROACH_TCP_YAW)[0], label="down to pilot"),
        Move(0.8, label="hover at boss"),
        Move(0.6, pos=K.saw_tcp(arb_xy, P.Z_SHOULDER + BOSS_HOVER_GAP, APPROACH_TCP_YAW)[0], label="recentre at boss"),
        Move(0.8, label="hover recentred"),
        Move(2.5, pos=K.saw_tcp(arb_xy, z_on(0.0) + ENTRY_ABOVE, APPROACH_TCP_YAW)[0], label="lower onto boss"),
        Move(0.8, label="hover over start"),
        Move(0.6, pos=K.saw_tcp(arb_xy, z_on(0.0) + ENTRY_ABOVE, APPROACH_TCP_YAW)[0], label="centre over start"),
        Move(0.8, label="hover centred"),
        Move(1.5, pos=K.saw_tcp(arb_xy, z_on(0.0) - ENTRY_BELOW, APPROACH_TCP_YAW)[0], label="seat thread start"),
        Move(0.6, label="thread caught"),
        Move(1.2, pos=K.saw_tcp(arb_xy, z_on(90.0), tcp_yaw(90.0))[0], rot=kit.down(tcp_yaw(90.0)), label="screw CW 1a"),
        Move(1.2, pos=K.saw_tcp(arb_xy, z_on(180.0), tcp_yaw(180.0))[0], rot=kit.down(tcp_yaw(180.0)), label="screw CW 1b"),
        Move(0.5, grip=OPEN, label="open regrasp"),
        Move(0.4, pos=K.saw_tcp(arb_xy, z_on(180.0) + 0.004, tcp_yaw(180.0))[0], label="lift open"),
        Move(0.8, pos=K.saw_tcp(arb_xy, z_on(180.0) + 0.004, tcp_yaw(90.0))[0], rot=kit.down(tcp_yaw(90.0)), label="back CCW 1"),
        Move(0.8, pos=K.saw_tcp(arb_xy, z_on(180.0) + 0.004, tcp_yaw(0.0))[0], rot=kit.down(tcp_yaw(0.0)), label="back CCW 2"),
        Move(0.4, pos=K.saw_tcp(arb_xy, z_on(180.0), tcp_yaw(0.0))[0], label="lower to saw"),
        Move(0.6, grip=SAW_GRIP, label="regrip saw"),
        Move(1.2, pos=K.saw_tcp(arb_xy, z_on(255.0), tcp_yaw(75.0))[0], rot=kit.down(tcp_yaw(75.0)), label="screw CW 2a"),
        Move(1.3, pos=K.saw_tcp(arb_xy, z_on(330.0), tcp_yaw(150.0))[0], rot=kit.down(tcp_yaw(150.0)), label="screw CW 2b to holes"),
        Move(0.5, label="settle at holes"),
        Move(0.6, pos=K.saw_tcp(arb_xy, z_on(330.0), tcp_yaw(150.0))[0], label="trim to holes"),
        Move(0.6, grip=OPEN, label="release aligned saw"),
        Move(1.0, pos=K.saw_tcp(arb_xy, K.SAW_CARRY_Z + 0.010, tcp_yaw(150.0))[0], label="lift clear of cup"),
    ]
    t_left_clear = sum(m.seconds for m in left)
    left.append(Move(2.2, pos=ready("left")[0], rot=ready("left")[1], label="ready"))

    # Phase 3: Right arm waits until Left hand lifts clear, then grips collar and pushes it UP (+8 mm) locking both pins through the holes
    right.append(Move(round(t_left_clear - sum(m.seconds for m in right), 3), label="wait for back-off"))
    right += [
        Move(1.1, pos=K.collar_tcp(0.0, back=0.055)[0], label="approach retracted collar"),
        Move(0.8, pos=K.collar_tcp(0.0)[0], label="around collar down"),
        Move(0.6, grip=COLLAR_GRIP, label="grip collar down"),
        Move(1.3, pos=K.collar_tcp(P.PIN_STROKE, dz=0.0005)[0], push=(0.0, 0.0, 16.0), label="push collar locked"),
        Move(0.3, push=ZERO_PUSH, label="settle collar locked"),
        Move(0.5, grip=OPEN, label="release locked collar"),
        Move(1.0, pos=K.collar_tcp(P.PIN_STROKE, back=0.095)[0], label="back off collar"),
        Move(2.0, pos=ready("right")[0], rot=ready("right")[1], label="ready"),
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
        ok = all(dry[s]["worst_ik_mm"] <= 0.5 and dry[s]["smallest_margin_deg"] >= 5.0 for s in ("left", "right"))
        return 0 if ok else 1

    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk

    def finger_forces(body_name: str) -> dict[str, float]:
        """Per-pad mean normal force (N) from each robot arm's two opposing finger pads on ``body_name``."""
        totals: dict[str, float] = {}
        for c in mk.contacts(body_name):
            if c.other.endswith("robot"):
                totals[c.other] = round(totals.get(c.other, 0.0) + 0.5 * abs(float(c.force)), 2)
        return totals

    def extra() -> dict:
        return {
            "collar_z_mm": round(1000.0 * m.collar_travel(), 2),
            "thread_engaged": m.thread_engaged(),
            "thread_cw_deg": round(m.thread_cw_deg(), 2),
            "shoulder_gap_mm": round(1000.0 * m.saw_shoulder_gap(), 3),
            "phase_err_deg": round(m.saw_phase_err_deg(), 2),
            "collar_fingers_N": finger_forces("collar"),
            "saw_fingers_N": finger_forces("hole_saw"),
            "pin_contact_N": round(mk.contact_force("collar", "hole_saw"), 2),
            "hands_clear_mm": round(1000.0 * m.hands_clear(), 1),
            "hold": m.STATE.get("hold", 0),
            "saw_xy_off_mm": round(1000.0 * m.saw_xy_off(), 3),
            "saw_tilt_deg": round(m.saw_tilt_deg(), 3),
            "saw_xyz_mm": [round(1000.0 * float(v), 4) for v in mk.pose("hole_saw")[0]],
            "saw_quat": [round(float(v), 7) for v in mk.matrix_to_quat(mk.pose("hole_saw")[1])],
        }

    folder = out / "full"
    record = "--no-video" not in sys.argv[2:] and "--check" not in sys.argv[2:]
    latest: dict = {}
    by_label = {mv.label: mv for mv in moves["left"]}
    lead = P.THREAD_PITCH / 360.0  # m the saw goes down per degree turned CW
    done: set = set()
    notes: dict = {}

    def rz(angle: float) -> np.ndarray:
        c, s_ = math.cos(angle), math.sin(angle)
        return np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])

    def about_axis(pose, cw_deg: float, dz: float):
        """A grasp-point pose turned ``cw_deg`` clockwise (seen from above) about the arbor's axis and moved dz up."""
        turn = rz(-math.radians(cw_deg))
        p0 = np.asarray(pose[0], dtype=float)
        rel = turn @ (p0 - np.array([P.X_ARBOR, P.Y_ARBOR, p0[2]]))
        return np.array([P.X_ARBOR + rel[0], P.Y_ARBOR + rel[1], p0[2] + dz]), turn @ np.asarray(pose[1], dtype=float)

    def put(label: str, pose) -> None:
        by_label[label].pos, by_label[label].rot = np.asarray(pose[0], dtype=float), np.asarray(pose[1], dtype=float)

    def aim_from_here(t: float, hand, targets) -> dict:
        """The arm sags differently in each pose, so aim from where the saw really is now: the world motion that takes
        the saw from its present pose to the one wanted (on the arbor's axis, level, at yaw APPROACH_SAW_YAW) moves the
        hand's present target too; ``targets`` (label, backplate gap over the shoulder) go down from there."""
        p_cmd, r_cmd = hand.state(t)[:2]
        p_saw, r_saw = mk.pose("hole_saw")
        turn = rz(APPROACH_SAW_YAW) @ r_saw.T
        p_fix = np.array([P.X_ARBOR, P.Y_ARBOR, p_saw[2]]) + turn @ (p_cmd - p_saw)
        g_now = float(p_saw[2]) - (m.TOP + P.Z_SHOULDER)
        for label, gap in targets:
            put(label, (p_fix + np.array([0.0, 0.0, gap - g_now]), turn @ r_cmd))
        return {"saw_off_axis_mm": round(1000.0 * m.saw_xy_off(), 3), "saw_tilt_deg": round(m.saw_tilt_deg(), 3), "gap_mm": round(1000.0 * g_now, 3)}

    def start_of(moves_list, label: str) -> float:
        t0 = 0.0
        for mv in moves_list:
            if mv.label == label:
                return t0
            t0 += mv.seconds
        raise KeyError(label)

    def director(t, arms, pictures) -> None:
        if pictures is not None:
            latest.update(pictures)
        hand = arms["left"]
        end_of = lambda label: start_of(hand.moves, label) + by_label[label].seconds  # noqa: E731
        soon = lambda label: t >= start_of(hand.moves, label) - 1.5 / 20.0  # noqa: E731
        gap0 = P.thread_start_gap(APPROACH_SAW_YAW)
        if "A" not in done and t >= end_of("lift saw") + 0.3:
            # Where the saw sits in the fingers now (it is lifted and still): aim the saw, not the fingers, at the arbor
            done.add("A")
            p_cmd, r_cmd = hand.state(t)[:2]
            p_saw, r_saw = mk.pose("hole_saw")
            r_rel, d_rel = r_cmd.T @ r_saw, r_cmd.T @ (p_saw - p_cmd)
            for label, z_bot in (("over arbor pilot", K.SAW_CARRY_Z), ("down to pilot", P.Z_SHOULDER + BOSS_HOVER_GAP)):
                r_c = rz(APPROACH_SAW_YAW) @ r_rel.T
                put(label, (np.array([P.X_ARBOR, P.Y_ARBOR, m.TOP + z_bot]) - r_c @ d_rel, r_c))
            off = r_cmd @ d_rel
            notes["grasp"] = {"saw_off_grasp_axis_mm": round(1000.0 * math.hypot(float(off[0]), float(off[1])), 3),
                              "saw_tilt_deg": round(m.saw_tilt_deg(), 3)}
        # The arm sags differently in each pose and answers small aims slowly: aim from where the saw is, twice while it
        # hangs free over the boss, go down slowly, and twice more just over the thread start
        for key, hover, nxt, gap in (("A2", "hover over pilot", "down to pilot", BOSS_HOVER_GAP),
                                     ("A3", "hover at boss", "recentre at boss", BOSS_HOVER_GAP),
                                     ("A3b", "hover recentred", "lower onto boss", gap0 + ENTRY_ABOVE),
                                     ("A4", "hover over start", "centre over start", gap0 + ENTRY_ABOVE),
                                     ("A5", "hover centred", "seat thread start", gap0 - ENTRY_BELOW)):
            if key not in done and t >= end_of(hover) - 1.5 / 20.0:
                done.add(key)
                notes[key] = aim_from_here(t, hand, ((nxt, gap),))
        if "B" not in done and soon("screw CW 1a"):
            if not m.thread_engaged() and notes.get("waits", 0) < 4:
                # Not caught yet: wait half a second longer on the thread start (the right hand waits as long) and aim again
                notes["waits"] = notes.get("waits", 0) + 1
                by_label["thread caught"].seconds += 0.5
                next(mv for mv in arms["right"].moves if mv.label == "wait for back-off").seconds += 0.5
                aim_from_here(t, hand, (("thread caught", gap0 - ENTRY_BELOW),))
                return
            # Turn from where the hand is: rotations about the arbor's axis, going down the thread's lead
            done.add("B")
            notes["engaged_before_turning"], notes["catch"] = m.thread_engaged(), m.STATE.get("catch")
            base = hand.state(start_of(hand.moves, "screw CW 1a") - 0.01)[:2]
            put("screw CW 1a", about_axis(base, 0.5 * STROKE1_DEG, -lead * 0.5 * STROKE1_DEG))
            end1 = about_axis(base, STROKE1_DEG, -lead * STROKE1_DEG)
            put("screw CW 1b", end1)
            up = about_axis(end1, 0.0, 0.004)
            put("lift open", up)
            put("back CCW 1", about_axis(up, -0.5 * STROKE1_DEG, 0.0))
            back = about_axis(up, -STROKE1_DEG, 0.0)
            put("back CCW 2", back)
            put("lower to saw", about_axis(back, 0.0, -0.004))
        if "C" not in done and soon("screw CW 2a"):
            # The second turn ends where the holes line up over the pins (yaw 0 mod 180), short of the 415 deg stop
            done.add("C")
            base = hand.state(start_of(hand.moves, "screw CW 2a") - 0.01)[:2]
            cw_now, yaw_mod = m.thread_cw_deg(), math.degrees(m.saw_yaw_rad()) % 180.0
            theta = min((yaw_mod + 180.0 * k for k in range(3)), key=lambda th: abs(cw_now + th - 370.0))
            theta = min(theta, 200.0)  # never more than one regrasp's worth (a missed catch must not wind the wrist up)
            put("screw CW 2a", about_axis(base, 0.5 * theta, -lead * 0.5 * theta))
            put("screw CW 2b to holes", about_axis(base, theta, -lead * theta))
            notes["second_turn"] = {"cw_before_deg": round(cw_now, 2), "saw_yaw_mod180_deg": round(yaw_mod, 2), "turn_deg": round(theta, 2)}
        if "D" not in done and t >= end_of("settle at holes") - 1.5 / 20.0:
            # Trim the last degrees from the saw's own yaw so both pins meet the holes square, then let go and lift clear
            done.add("D")
            base = hand.state(t)[:2]
            yaw_mod = math.degrees(m.saw_yaw_rad()) % 180.0
            trim = yaw_mod if yaw_mod < 90.0 else yaw_mod - 180.0  # CW turn (deg) still to go; negative backs off
            trimmed = about_axis(base, trim, -lead * trim)
            put("trim to holes", trimmed)
            put("lift clear of cup", (np.array([trimmed[0][0], trimmed[0][1], m.TOP + K.SAW_CARRY_Z + 0.010 + K.SAW_GRIP_FROM_BOT]), trimmed[1]))
            notes["trim_deg"] = round(trim, 2)

    result = kit.run(
        embodiment, moves, folder, seed=0, director=director, extra=extra,
        limit_s=float(m.RUN_LIMIT_STEPS) / 20.0, tail_s=4.5, sheet_frames=16, record=record,
    )
    rows = result["rows"]
    if "top_cam" in latest:
        save_end_picture(latest["top_cam"], folder)

    k_catch = next((k for k, r in enumerate(rows) if r["extra"]["thread_engaged"]), None)

    def pose_change(a: dict, b: dict) -> tuple[float, float]:
        dp = float(np.linalg.norm(np.subtract(b["saw_xyz_mm"], a["saw_xyz_mm"])))
        dot = min(1.0, abs(float(np.dot(a["saw_quat"], b["saw_quat"]))))
        return round(dp, 4), round(math.degrees(2.0 * math.acos(dot)), 4)

    catch_jump = None
    if k_catch is not None and 0 < k_catch < len(rows) - 1:
        before, after = pose_change(rows[k_catch - 1]["extra"], rows[k_catch]["extra"]), pose_change(rows[k_catch]["extra"], rows[k_catch + 1]["extra"])
        catch_jump = {"t_catch_s": rows[k_catch]["t"], "into_catch_step_mm_deg": before, "after_catch_step_mm_deg": after}
    first_qtr = next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.25), None)
    first_half = next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.5), None)
    first_full = next((r["t"] for r in rows if r["info"].get("score", 0.0) == 1.0), None)

    collar_trace = [
        {"t": r["t"], "label": r["right"], "collar_z_mm": r["extra"]["collar_z_mm"], "fingers_N": r["extra"]["collar_fingers_N"]}
        for r in rows if r["extra"]["collar_fingers_N"]
    ]
    thread_trace = [
        {"t": r["t"], "label": r["left"], "cw_deg": r["extra"]["thread_cw_deg"], "gap_mm": r["extra"]["shoulder_gap_mm"], "phase_deg": r["extra"]["phase_err_deg"], "fingers_N": r["extra"]["saw_fingers_N"]}
        for r in rows if r["extra"]["thread_engaged"] and r["extra"]["saw_fingers_N"]
    ]

    readings = {
        "score_at_end": rows[-1]["info"].get("score"),
        "seconds": rows[-1]["t"],
        "steps": len(rows),
        "first_0p25_s": first_qtr,
        "first_0p50_s": first_half,
        "first_1p00_s": first_full,
        "peak_CW_deg": max(r["extra"]["thread_cw_deg"] for r in rows),
        "end_cw_deg": rows[-1]["extra"]["thread_cw_deg"],
        "end_shoulder_gap_mm": rows[-1]["extra"]["shoulder_gap_mm"],
        "end_phase_err_deg": rows[-1]["extra"]["phase_err_deg"],
        "end_collar_z_mm": rows[-1]["extra"]["collar_z_mm"],
        "end_hands_clear_mm": rows[-1]["extra"]["hands_clear_mm"],
        "catch_pose_change": catch_jump,
        "director": notes,
        "both_arms_ready_at_end": {side: rows[-1][side] == "done" for side in ("left", "right")},
        "collar_action_samples": collar_trace[::6],
        "thread_action_samples": thread_trace[::10],
        "report": result.get("report"),
        "worst_ik": result.get("worst_ik"),
        "median_step_ms": result.get("median_step_ms"),
    }
    (folder / "readings.json").write_text(json.dumps(readings, indent=2, default=str))
    print("READINGS " + json.dumps({k: v for k, v in readings.items() if not k.endswith("_samples")}, default=str), flush=True)
    return 0 if readings["score_at_end"] == 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
