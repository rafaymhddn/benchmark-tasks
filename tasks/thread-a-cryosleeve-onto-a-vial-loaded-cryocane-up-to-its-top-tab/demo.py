#!/usr/bin/env python3
"""The scripted full-score reference run of the CryoCane sleeving task, and an offline IK check of its plan.

    scripts/rta demo <id> mech/tasks/cryocane_sleeve [--no-video] [--quick]

Pure arm joint actions throughout (zero write_pose, zero set_joint, zero welds):
1. Left hand grasps the 24 mm bent top ID tab of the loaded CryoCane on the prep mat, lifts it level (roll < 2 deg so
   neither loose 12.5 mm Nunc 377267 vial spills from its open C-bay), and yaws 180 deg horizontally (-90 -> 0 -> +90
   deg) so the cane's straight bottom foot points -y toward the sleeve.
2. Right hand grasps the 273 mm opaque frosted-ivory CryoSleeve near its -y tail and lifts its +y mouth in front of the
   cane's straight bottom foot.
3. No joint guides the sleeve (13.5 mm bore over the 13.2 mm stack): ``Inserter`` reads the cane's and sleeve's poses
   (a scripted demo may), puts the mouth on the stack's middle line 8 then 4 mm before the foot, and threads in 0.4 s
   steps, each turning the right hand so the sleeve lies along the cane and moving its mouth at most 15 mm ahead of
   where it is, while the left hand brings the cane 55 mm towards it; seated when the mouth is 262 mm on.
4. Right hand opens to 94 mm, lifts straight up clear of the sleeve, and returns to ready; left hand carries the
   enclosed cane over the stainless transfer tray, lays it flat inside, opens, lifts clear, and returns to ready (1.0).
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
FINGER_STIFFNESS = 5000.0  # N/m per finger drive (yam_arena constants)
CANE_PINCH_N = 38.0  # N per finger (<= 40 N hard limit): the tab pinch also holds the cane level against the slide
SLEEVE_PINCH_N = 34.0  # N per finger (<= 40 N hard limit)
CANE_GRIP = max(0.0, (P.TAB_W - 2.0 * CANE_PINCH_N / FINGER_STIFFNESS) / K.MAX_JAW)
SLEEVE_GRIP = max(0.0, (2.0 * P.SLEEVE_R_OUT - 2.0 * SLEEVE_PINCH_N / FINGER_STIFFNESS) / K.MAX_JAW)


def ready(side: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, dtype=float), K.BASES[side])


def _align_rot(u_from: np.ndarray, u_to: np.ndarray) -> np.ndarray:
    """Smallest rotation matrix mapping unit vector ``u_from`` onto ``u_to``."""
    a = u_from / np.linalg.norm(u_from)
    b = u_to / np.linalg.norm(u_to)
    cross = np.cross(a, b)
    s = float(np.linalg.norm(cross))
    if s < 1e-8:
        return np.eye(3)
    c = max(-1.0, min(1.0, float(np.dot(a, b))))
    k_mat = np.array([[0.0, -cross[2], cross[1]], [cross[2], 0.0, -cross[0]], [-cross[1], cross[0], 0.0]]) / s
    ang = math.atan2(s, c)
    return np.eye(3) + math.sin(ang) * k_mat + (1.0 - math.cos(ang)) * (k_mat @ k_mat)


class Inserter:
    """The closed-loop two-hand slide (shared with trials.py). ``cock_deg`` turns the sleeve's commanded axis that far
    about the vertical away from the cane's (a cocked push); ``release`` then racks the seated cane in the tray.

    Align: measure, correct, settle until the mouth is within 0.35 mm of the stack's middle line 4 mm before the foot
    and the sleeve within 0.15 deg of the cane (at most 10 rounds). Thread: 0.3 s steps, each turning the right hand so
    the sleeve lies along the cane and putting its mouth 6 mm ahead of where it is (0.5 N push); stalled for 4 steps,
    it backs off 4 mm and aligns again (4 tries, then 'bound'). The first 20 mm go in 3 mm steps of 0.4 s; sideways
    corrections take 0.8 of the error. Seated at 262 mm."""

    STEP_S, LEAD, SEATED, PUSH_N = 0.3, 0.006, 0.262, 0.5
    ENTRY_S, ENTRY_STEP_S, ENTRY_LEAD, GAIN = 0.020, 0.4, 0.003, 0.8
    STEP_SIDE, STEP_TURN_DEG = 0.004, 3.0  # while threading, at most this much sideways and this much turn per step
    ALIGN_MM, ALIGN_DEG, ALIGN_ROUNDS, TRIES = 0.35, 0.15, 10, 4

    def __init__(self, m, cock_deg: float = 0.0, release: bool = True):
        self.m, self.cock, self.release = m, math.radians(cock_deg), release
        self.stage, self.push_n, self.log, self.hist, self.left_y0 = "pick", 0.0, [], [], None
        self.rounds, self.tries, self.s_align = 0, 0, -0.004

    def _axes(self):
        m = self.m
        p_c, r_c = m.cane_pose()
        u_c = r_c[:, 1].copy()
        u_c[2] = float(np.clip(u_c[2], -0.05, 0.05))
        u_c /= np.linalg.norm(u_c)
        c, s = math.cos(self.cock), math.sin(self.cock)
        u_goal = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ u_c
        return p_c + P.BORE_Z * r_c[:, 2], u_c, u_goal

    def mouth(self):
        """(station along the cane, sideways and vertical offset of the mouth from the stack's middle line, angle to the goal)."""
        a_c, u_c, u_goal = self._axes()
        p_s, r_s = self.m.mk.pose("sleeve")
        rel = p_s + 0.5 * P.SLEEVE_L * r_s[:, 1] - a_c
        s = float(np.dot(rel, u_c))
        lat = rel - s * u_c
        side = np.cross(np.array([0.0, 0.0, 1.0]), u_c)
        ang = math.degrees(math.acos(min(1.0, float(np.dot(r_s[:, 1], u_goal)))))
        return s, float(np.dot(lat, side)), float(lat[2]), ang

    def right_goal(self, t: float, arms, s_goal: float):
        """Right TCP pose that turns the sleeve's axis onto ``u_goal`` and puts its mouth at ``s_goal`` along the cane."""
        a_c, u_c, u_goal = self._axes()
        p_s, r_s = self.m.mk.pose("sleeve")
        u_s = r_s[:, 1]
        tcp_cmd, rot_cmd = arms["right"].state(t)[:2]
        tcp_act = self.m.mk.gripper("right")[0]
        r_fix = _align_rot(u_s, u_goal)
        mouth_after = tcp_act + r_fix @ (p_s + 0.5 * P.SLEEVE_L * u_s - tcp_act)
        err = a_c + s_goal * u_c - mouth_after
        axial = float(np.dot(err, u_c)) * u_c
        return tcp_cmd + axial + self.GAIN * (err - axial), r_fix @ rot_cmd, u_c

    def _note(self, t: float, **kw) -> None:
        s, dx, dz, ang = self.mouth()
        self.log.append({"t": round(t, 2), "stage": self.stage, "mouth_mm": round(1000.0 * s, 1), "side_mm": round(1000.0 * dx, 2),
                         "up_mm": round(1000.0 * dz, 2), "ang_deg": round(ang, 2), **kw})

    def direct(self, t: float, arms) -> None:
        import demo_kit as kit

        m = self.m
        if not m.mk.MECH.poses or t < max(arms["left"].seconds, arms["right"].seconds) - 0.025 or self.stage == "done":
            return
        Move = kit.Move
        s_now, dx, dz, ang = self.mouth()
        if self.stage == "pick":
            pos, rot, _ = self.right_goal(t, arms, -0.010)
            arms["left"].moves += [Move(1.2, label="hold cane"), Move(0.6, label="settle")]
            arms["right"].moves += [Move(1.2, pos=pos, rot=rot, label="align mouth"), Move(0.6, label="settle")]
            self.stage = "align"
            return
        if self.stage == "align":
            ok = math.hypot(dx, dz) <= 0.001 * self.ALIGN_MM and ang <= self.ALIGN_DEG and abs(s_now - self.s_align) <= 0.002
            if ok or self.rounds >= self.ALIGN_ROUNDS:
                self._note(t, rounds=self.rounds, aligned=ok)
                self.stage, self.rounds, self.hist = "thread", 0, []
                self.left_y0 = arms["left"].state(t)[0].copy() if self.left_y0 is None else self.left_y0
                return
            self.rounds += 1
            pos, rot, _ = self.right_goal(t, arms, self.s_align)
            arms["left"].moves += [Move(0.5, label="hold cane"), Move(0.5, label="settle")]
            arms["right"].moves += [Move(0.5, pos=pos, rot=rot, label="align mouth"), Move(0.5, label="settle")]
            return
        if self.stage == "thread":
            self.hist.append(s_now)
            if s_now >= self.SEATED:
                self.stage, self.push_n = "seated", 0.0
                self._note(t)
                arms["left"].moves.append(Move(0.6, push=(0.0, 0.0, 0.0), label="seated hold"))
                arms["right"].moves.append(Move(0.6, push=(0.0, 0.0, 0.0), label="seated hold"))
                return
            if len(self.hist) > 4 and self.hist[-1] - self.hist[-5] < 0.0005:
                self.tries += 1
                self._note(t, stalled=self.tries)
                if self.tries > self.TRIES:
                    self.stage, self.push_n = "bound", 0.0
                    arms["left"].moves.append(Move(0.6, push=(0.0, 0.0, 0.0), label="hold"))
                    arms["right"].moves.append(Move(0.6, push=(0.0, 0.0, 0.0), label="hold"))
                    return
                self.s_align = s_now - 0.004
                pos, rot, _ = self.right_goal(t, arms, self.s_align)
                arms["left"].moves += [Move(0.5, push=(0.0, 0.0, 0.0), label="hold cane"), Move(0.3, label="settle")]
                arms["right"].moves += [Move(0.5, pos=pos, rot=rot, push=(0.0, 0.0, 0.0), label="back off"), Move(0.3, label="settle")]
                self.stage, self.push_n = "align", 0.0
                return
            entering = s_now < self.ENTRY_S
            s_goal = min(self.SEATED + 0.002, max(s_now, self.s_align) + (self.ENTRY_LEAD if entering else self.LEAD))
            pos, rot, u_c = self.right_goal(t, arms, s_goal)
            tcp_cmd, rot_cmd = arms["right"].state(t)[:2]
            step = pos - tcp_cmd
            axial = float(np.dot(step, u_c)) * u_c
            side = step - axial
            if float(np.linalg.norm(side)) > self.STEP_SIDE:
                side *= self.STEP_SIDE / float(np.linalg.norm(side))
            pos = tcp_cmd + axial + side
            turn = rot @ rot_cmd.T
            ang = math.acos(max(-1.0, min(1.0, 0.5 * (float(np.trace(turn)) - 1.0))))
            if ang > math.radians(self.STEP_TURN_DEG):
                w = np.array([turn[2, 1] - turn[1, 2], turn[0, 2] - turn[2, 0], turn[1, 0] - turn[0, 1]]) / (2.0 * math.sin(ang))
                k = np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])
                a = math.radians(self.STEP_TURN_DEG)
                rot = (np.eye(3) + math.sin(a) * k + (1.0 - math.cos(a)) * (k @ k)) @ rot_cmd
            frac = float(np.clip((s_goal - 0.030) / 0.232, 0.0, 1.0))
            self.push_n = self.PUSH_N
            step = self.ENTRY_STEP_S if entering else self.STEP_S
            arms["left"].moves.append(Move(step, pos=self.left_y0 - 0.055 * frac * u_c, push=(-self.PUSH_N * u_c).tolist(), label="thread"))
            arms["right"].moves.append(Move(step, pos=pos, rot=rot, push=(self.PUSH_N * u_c).tolist(), label="thread"))
            if len(self.hist) % 10 == 1:
                self._note(t, goal_mm=round(1000.0 * s_goal, 1))
            return
        if self.stage == "seated" and self.release:
            p_poses = K.poses()
            tcp_r = arms["right"].state(t)[0]
            arms["right"].moves += [
                Move(0.9, grip=OPEN, label="release sleeve"),
                Move(1.3, pos=tcp_r + np.array([0.0, 0.0, 0.060]), rot=K.ROT_SLV_R, label="lift off sleeve"),
                Move(2.6, pos=ready("right")[0], rot=ready("right")[1], label="ready"),
            ]
            arms["left"].moves += [
                Move(2.2, label="wait right release"),
                Move(2.4, pos=p_poses["L: over tray"][1], rot=K.ROT_END_L, label="carry over tray"),
                Move(2.0, pos=p_poses["L: lay in tray"][1], label="lay in tray"),
                Move(0.8, grip=OPEN, label="release in tray"),
                Move(1.2, pos=p_poses["L: over tray"][1], label="clear tray"),
                Move(2.6, pos=ready("left")[0], rot=ready("left")[1], label="ready"),
            ]
        self.stage = "done"


def initial_plan() -> dict[str, list]:
    """Phase 1 (0 .. 11.0 s): pick cane, lift level and yaw 180 deg; pick sleeve and lift near coaxial pre-entry."""
    import demo_kit as kit

    Move = kit.Move
    p = K.poses()
    left = [
        Move(2.2, pos=p["L: above cane tab"][1], rot=K.ROT_START_L, grip=OPEN, label="above cane tab"),
        Move(1.2, pos=p["L: on cane tab"][1], label="on cane tab"),
        Move(0.8, grip=CANE_GRIP, label="pinch cane tab"),
        Move(1.8, pos=p["L: lift cane level"][1], label="lift level"),
        Move(2.2, pos=p["L: yaw 90 deg"][1], rot=K.ROT_MID_L, label="yaw 90 deg"),
        Move(2.2, pos=p["L: yaw 180 deg ready"][1], rot=K.ROT_COAX_L, label="yaw 180 deg"),
        Move(0.6, label="settle coaxial"),
    ]
    right_pick = [
        Move(2.0, label="wait cane pick"),
        Move(2.4, pos=p["R: above sleeve tail"][1], rot=K.ROT_SLV_R, grip=OPEN, label="above sleeve"),
        Move(1.2, pos=p["R: on sleeve tail"][1], label="on sleeve"),
        Move(0.8, grip=SLEEVE_GRIP, label="pinch sleeve"),
        Move(2.5, pos=p["R: lift sleeve coaxial"][1], rot=K.ROT_COAX_R, label="lift coaxial"),
    ]
    dwell = (sum(mv.seconds for mv in left) - sum(mv.seconds for mv in right_pick))
    right = right_pick + [Move(max(0.2, dwell), label="settle coaxial")]
    return {"left": left, "right": right}


def full_nominal_plan() -> dict[str, list]:
    """Full nominal open-loop trajectory used by ``--quick`` IK verification."""
    import demo_kit as kit

    Move = kit.Move
    p = K.poses()
    base = initial_plan()
    base["left"] += [
        Move(1.4, label="align mouth"),
        Move(1.6, label="enter foot"),
        Move(2.6, pos=p["L: coaxial mid"][1], label="thread mid"),
        Move(2.8, pos=p["L: coaxial seated"][1], label="thread seated"),
        Move(0.6, label="seated hold"),
        Move(2.2, label="wait right release"),
        Move(2.4, pos=p["L: over tray"][1], rot=K.ROT_END_L, label="carry over tray"),
        Move(2.0, pos=p["L: lay in tray"][1], label="lay in tray"),
        Move(0.8, grip=OPEN, label="release in tray"),
        Move(1.2, pos=p["L: over tray"][1], label="clear tray"),
        Move(2.6, pos=ready("left")[0], rot=ready("left")[1], label="ready"),
    ]
    base["right"] += [
        Move(1.4, pos=p["R: lift sleeve coaxial"][1], label="align mouth"),
        Move(1.6, pos=p["R: lift sleeve coaxial"][1] + np.array([0.0, 0.026, 0.0]), label="enter foot"),
        Move(2.6, pos=p["R: coaxial mid"][1], label="thread mid"),
        Move(2.8, pos=p["R: coaxial seated"][1], label="thread seated"),
        Move(0.6, label="seated hold"),
        Move(0.9, grip=OPEN, label="release sleeve"),
        Move(1.3, pos=p["R: coaxial seated"][1] + np.array([0.0, 0.0, 0.055]), rot=K.ROT_SLV_R, label="lift off sleeve"),
        Move(2.8, pos=ready("right")[0], rot=ready("right")[1], label="ready"),
    ]
    return base


def save_end_picture(frame, folder: Path) -> None:
    import cv2
    import demo_kit as kit

    cv2.imwrite(str(folder / "end_top.png"), cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(folder / "end_top_policy.png"), cv2.cvtColor(kit.policy_picture(np.asarray(frame)), cv2.COLOR_RGB2BGR))


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    if "--quick" in sys.argv[2:]:
        from yam_arena import station

        dry = kit.dry_run(full_nominal_plan(), station.BASES)
        print("QUICK " + json.dumps(dry), flush=True)
        return 0
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    moves = initial_plan()
    p_poses = K.poses()
    Move = kit.Move
    inserter = Inserter(m)

    def director(t: float, arms, pictures) -> None:
        if pictures is not None:
            latest.update(pictures)
        inserter.direct(t, arms)

    def extra() -> dict:
        ok, y_lo, y_hi, rad, ang = m.sleeve_span_on_cane()
        touch = [c for c in m.mk.contacts("sleeve") if c.other in ("cane", "vial_a", "vial_b")]
        return {
            "on_cane": ok,
            "slide_mm": round(1000.0 * (max(0.0, y_hi) if ok else 0.0), 2),
            "y_lo_mm": round(1000.0 * y_lo, 2),
            "rad_mm": round(1000.0 * rad, 2),
            "ang_deg": round(ang, 2),
            "gap_tab_mm": None if not ok else round(1000.0 * max(0.0, P.TAB_SHOULDER_Y - y_hi), 2),
            "contact_n": round(float(sum(c.force for c in touch)), 2),
            "breach_mm": round(m.wall_breach_mm(), 2),
            "stage": inserter.stage,
            "roll_deg": round(m.cane_roll_deg(), 2),
            "pitch_deg": round(m.cane_pitch_deg(), 2),
            "hands_clear_mm": round(1000.0 * m.hands_clear(), 1),
            "hold": m.STATE.get("hold", 0),
        }

    folder = out / "full"
    record = "--no-video" not in sys.argv[2:]
    latest: dict = {}
    result = kit.run(
        emb,
        moves,
        folder,
        seed=0,
        director=director,
        extra=extra,
        limit_s=float(m.RUN_LIMIT_STEPS) / 20.0,
        tail_s=5.0,
        sheet_frames=16,
        record=record,
    )
    rows = result["rows"]
    save_end_picture(latest["top_cam"], folder)
    labels = {side: [r[side] for r in rows] for side in ("left", "right")}
    first_q = next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.25), None)
    first_h = next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.5), None)
    first_f = next((r["t"] for r in rows if r["info"].get("score", 0.0) == 1.0), None)
    slide_trace = [
        {
            "t": r["t"],
            "slide_mm": r["extra"]["slide_mm"],
            "rad_mm": r["extra"]["rad_mm"],
            "ang_deg": r["extra"]["ang_deg"],
            "contact_n": r["extra"]["contact_n"],
            "arm_f_y_n": {"left": r["force"]["left"][1], "right": r["force"]["right"][1]},
        }
        for r in rows
        if r["right"] in ("align mouth", "settle", "thread", "seated hold")
    ][::8]
    pinch_l = max(
        0.5 * FINGER_STIFFNESS * max(0.0, (r["grip"]["left"][3] - CANE_GRIP) * K.MAX_JAW)
        for r in rows
        if "left" in r["grip"] and r["left"] in ("lift level", "yaw 90 deg", "yaw 180 deg", "thread")
    )
    pinch_r = max(
        0.5 * FINGER_STIFFNESS * max(0.0, (r["grip"]["right"][3] - SLEEVE_GRIP) * K.MAX_JAW)
        for r in rows
        if "right" in r["grip"] and r["right"] in ("lift coaxial", "align mouth", "thread")
    )
    readings = {
        "score at end": rows[-1]["info"].get("score"),
        "seconds": rows[-1]["t"],
        "steps": len(rows),
        "first 0.25 / 0.5 / 1.0 at (s)": [first_q, first_h, first_f],
        "worst abs cane roll / pitch (deg)": [
            round(max(abs(r["extra"]["roll_deg"]) for r in rows), 2),
            round(max(abs(r["extra"]["pitch_deg"]) for r in rows), 2),
        ],
        "max slide (mm)": max(r["extra"]["slide_mm"] for r in rows),
        "final gap to top tab (mm)": rows[-1]["extra"]["gap_tab_mm"],
        "peak sleeve contact force on cane and vials (N)": max(r["extra"]["contact_n"] for r in rows),
        "worst cane point beyond the sleeve's bore (mm)": max(r["extra"]["breach_mm"] for r in rows),
        "slide log": inserter.log,
        "peak per-finger pinch on cane / sleeve (N)": [round(pinch_l, 1), round(pinch_r, 1)],
        "slide trace (every 8th step)": slide_trace,
        "both arms ready at end": {side: labels[side][-1] == "done" for side in ("left", "right")},
        "hands clear at end (mm)": rows[-1]["extra"]["hands_clear_mm"],
        "report": result.get("report"),
        "worst IK": result.get("worst_ik"),
        "median step (ms)": result.get("median_step_ms"),
    }
    (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
    print("READINGS " + json.dumps(readings, default=str), flush=True)
    return 0 if readings["score at end"] == 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
