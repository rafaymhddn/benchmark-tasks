#!/usr/bin/env python3
"""Scripted full-score bimanual reference run for the end-cap suspension packing task.

    scripts/rta demo <id> mech/tasks/endcap_pack [--check | --no-video] [--quick]

Both arms descend into the open carton with fingers closing across x (72 mm cap body, 40.2 mm clear of the front/back
carton walls, 0 mm of finger on the +/-y faces), lift both nested foam end-caps clear of the carton (0.25), set them
on the bench outside the roller's ends, slide both sockets coaxially onto the 40 x 200 mm Interroll roller on its
bench cradle (0.5), maintain an inward +/-y clamping preload (~10 N) while lifting the 3-body unit over the 80.2 mm
carton wall and lowering it onto the carton floor, then open and return to Ready while the roller hangs 16 mm
suspended above the carton floor (1.0).
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

OPEN = K.OPEN_JAW / K.MAX_JAW
PINCH = K.PINCH_JAW / K.MAX_JAW
LIFT_COMP_Z = 2.5  # N pure vertical lift compensation per arm (zero lateral push_y so wrist roll j5 never splays!)
Y_CLAMP_BENCH = 0.0965  # m: 5.0 mm inward of +/-0.1015 m equilibrium (hulls stay > 12 mm apart)
Y_CLAMP_CARRY = 0.0950  # m: 6.5 mm inward position bias per arm (~10 N bilateral compression through shoulder pan j1)
TOE_IN_RAD = 0.060  # 3.4 deg inward bottom-toe-in pre-compensating j5's compliance so caps stay dead upright in air


def ready(side: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, dtype=float), K.BASES[side])


def arm_plan(side: str) -> list:
    import demo_kit as kit

    Move = kit.Move
    s = +1.0 if side == "left" else -1.0
    r_pos, r_rot = ready(side)
    rot = K.grip_rot(side)
    # Rotate about world +x by +s * TOE_IN_RAD: for left (s=+1), R_x(+theta) moves tool -z (down) to [0, +sin, -cos]?
    # Wait: R_x(alpha) * [0, 0, -1] = [0, +sin(alpha), -cos(alpha)].
    # For left (at +y, socket toward -y), we want the cap's bottom (-z) to toe INWARD toward -y -> sin(alpha) < 0 -> alpha = -s * TOE_IN_RAD!
    rot_carry = kit.about((1.0, 0.0, 0.0), -s * TOE_IN_RAD) @ rot
    return [
        Move(2.2, pos=(P.X_CARTON, s * K.Y_GRIP_COUPLED, K.Z_HOVER_BOX), rot=rot, grip=OPEN, label="hover carton"),
        Move(1.2, pos=(P.X_CARTON, s * K.Y_GRIP_COUPLED, K.Z_SEAT_BOX), label="around nested cap"),
        Move(0.7, grip=PINCH, label="pinch cap"),
        Move(1.4, pos=(P.X_CARTON, s * K.Y_GRIP_COUPLED, K.Z_HOVER_BOX), push=(0.0, 0.0, 0.3), label="lift cap out"),
        Move(1.6, pos=(P.X_ROLLER, s * K.Y_GRIP_STAGE, K.Z_HOVER_BOX), push=(0.0, 0.0, 0.3), label="to bench stage"),
        Move(1.4, pos=(P.X_ROLLER, s * K.Y_GRIP_STAGE, K.Z_STAGE_BENCH), push=(0.0, 0.0, 0.0), label="set cap outside roller"),
        Move(1.8, pos=(P.X_ROLLER, s * Y_CLAMP_BENCH, K.Z_STAGE_BENCH), label="slide onto roller"),
        Move(0.6, pos=(P.X_ROLLER, s * Y_CLAMP_CARRY, K.Z_STAGE_BENCH + 0.002), rot=rot_carry, push=(0.0, 0.0, LIFT_COMP_Z), label="preload clamp"),
        Move(1.8, pos=(P.X_ROLLER, s * Y_CLAMP_CARRY, K.Z_HOVER_BOX), rot=rot_carry, push=(0.0, 0.0, LIFT_COMP_Z), label="lift coupled unit"),
        Move(2.2, pos=(P.X_CARTON, s * Y_CLAMP_CARRY, K.Z_HOVER_BOX), rot=rot_carry, push=(0.0, 0.0, LIFT_COMP_Z), label="carry over carton"),
        Move(1.8, pos=(P.X_CARTON, s * Y_CLAMP_BENCH, K.Z_SEAT_BOX + 0.0010), rot=rot, push=(0.0, 0.0, 0.0), label="lower into carton"),
        Move(0.5, pos=(P.X_CARTON, s * K.Y_GRIP_COUPLED, K.Z_SEAT_BOX + 0.0010), label="relax clamp"),
        Move(0.7, grip=OPEN, label="release unit"),
        Move(1.2, pos=(P.X_CARTON, s * K.Y_GRIP_COUPLED, K.Z_HOVER_BOX), label="clear carton"),
        Move(2.2, pos=r_pos, rot=r_rot, grip=1.0, label="ready"),
    ]


def plan() -> dict[str, list]:
    return {"left": arm_plan("left"), "right": arm_plan("right")}


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

    def pad_force_per_finger(c: str) -> float:
        total = 0.0
        for contact in mk.contacts(f"cap_{c}"):
            if contact.other.endswith("robot"):
                total += abs(float(contact.force))
        return round(total / 2.0, 2)

    def extra() -> dict:
        pair, _ = m.best_engagement()
        return {
            "roller_z_gap_mm": round(1000 * m.roller_floor_gap(), 2),
            "roller_z_over_bench_mm": round(1000 * (float(mk.pose("roller")[0][2]) - m.TOP), 2),
            "engage_mm": {c: round(1000 * pair[c][0], 1) for c in m.CAPS},
            "cap_roller_N": {c: round(mk.contact_force(f"cap_{c}", "roller"), 2) for c in m.CAPS},
            "cap_carton_N": {c: round(mk.contact_force(f"cap_{c}", "carton"), 2) for c in m.CAPS},
            "roller_carton_N": round(mk.contact_force("roller", "carton"), 2),
            "finger_pad_N": {c: pad_force_per_finger(c) for c in m.CAPS},
            "hands_clear_mm": round(1000 * m.hands_clear(), 1),
            "hold": m.STATE.get("hold", 0),
        }

    folder = out / "full"
    record = "--check" not in sys.argv[2:] and "--no-video" not in sys.argv[2:]
    latest: dict = {}
    saved_start = False

    def director(t, arms, pictures) -> None:
        nonlocal saved_start
        if pictures is not None:
            if not saved_start:
                import cv2

                folder.mkdir(parents=True, exist_ok=True)
                for cam, stem in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
                    bgr = cv2.cvtColor(np.asarray(pictures[cam]), cv2.COLOR_RGB2BGR)
                    cv2.imwrite(str(folder / f"{stem}.png"), bgr)
                    cv2.imwrite(str(folder / f"{stem}_policy.png"), cv2.resize(bgr, (320, 176), interpolation=cv2.INTER_AREA))
                saved_start = True
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

    first_q = next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.25), None)
    first_h = next((r["t"] for r in rows if r["info"].get("score", 0.0) >= 0.5), None)
    first_f = next((r["t"] for r in rows if r["info"].get("score", 0.0) == 1.0), None)
    carry_rows = [r for r in rows if r["left"] in ("lift coupled unit", "carry over carton")]
    clamp_trace = [
        {
            "t": r["t"],
            "phase": r["left"],
            "roller_z_mm": r["extra"]["roller_z_over_bench_mm"],
            "engage_mm": r["extra"]["engage_mm"],
            "cap_roller_N": r["extra"]["cap_roller_N"],
            "finger_pad_N": r["extra"]["finger_pad_N"],
        }
        for r in carry_rows[::6]
    ]
    readings = {
        "score_at_end": rows[-1]["info"].get("score") if rows else 0.0,
        "steps": len(rows),
        "seconds": rows[-1]["t"] if rows else 0.0,
        "first_0.25_s": first_q,
        "first_0.5_s": first_h,
        "first_1.0_s": first_f,
        "peak_finger_pad_N": {c: max((r["extra"]["finger_pad_N"][c] for r in rows), default=0.0) for c in m.CAPS},
        "mean_carry_cap_roller_N": {
            c: round(float(np.mean([r["extra"]["cap_roller_N"][c] for r in carry_rows])), 2) if carry_rows else 0.0 for c in m.CAPS
        },
        "carry_trace": clamp_trace,
        "end_extra": rows[-1]["extra"] if rows else None,
        "report": result.get("report"),
        "worst_ik": result.get("worst_ik"),
        "median_step_ms": result.get("median_step_ms"),
    }
    (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
    print("READINGS " + json.dumps(readings, default=str), flush=True)
    return 0 if readings["score_at_end"] == 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
