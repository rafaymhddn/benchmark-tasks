#!/usr/bin/env python3
"""Reach, joint-margin, collision-hull, and frame-0 checks for the dental curing-light hood task (Gate 1).

1. Offline (no simulator): solves every waypoint of both arms with ``yam_arena.ik`` (<= 0.5 mm, <= 0.6 deg, every joint
   >= 6.0 deg inside its limits, every grasp <= 160 mm above the bench), checks collision hulls against the bench,
   cradle, hood slot, and propped-open jaw model, and verifies >= 35 mm TCP separation and positive hull gap whenever
   both arms work at once.
2. ``--frame0 <dir>`` in the simulator: saves ``start_top.png``, ``start_top_policy.png`` (320 x 176),
   ``start_left_wrist.png``, ``start_right_wrist.png``, and steps 2.0 s idle to confirm < 1.5 mm settle drift and
   score 0.0.

    scripts/rta simpy <id> mech/tasks/curing_hood/checks.py [--frame0 /workspace/robocurve/outputs/<id>]
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
from yam_arena import analysis, constants as C, ik, station
from yam_arena.constants import READY
from yam_arena.task_file import TABLE_TOP

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import parts as P  # noqa: E402

TOP = TABLE_TOP["station"]
BASES = {side: np.array(station.BASES[side], dtype=float) for side in ("left", "right")}
HAND_LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")
MAX_JAW = 2.0 * abs(float(C.FINGER_OPEN))  # 0.0939 m
IK_MM, IK_DEG, MARGIN_DEG, SEPARATION_M, MAX_GRASP_Z = 0.5, 0.6, 6.0, 0.080, 0.160

RIGHT_BARREL_X = -0.046  # m along wand barrel behind midpoint (near CoM inside the 126 mm bay, 100 mm behind button)
WING_GRIP_R = 0.024  # m along hood +y_h major wing from bore center (clamps flat 3.0 mm wing, clear of Ø 16 mm collar)


def frame(closing, approach) -> np.ndarray:
    """Gripper rotation matrix: columns are closing axis, pad width axis, approach axis."""
    a = np.asarray(approach, dtype=float)
    a = a / np.linalg.norm(a)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c = c / np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def right_on_wand(p_wand: np.ndarray, r_wand: np.ndarray, lift_z: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Right gripper on the wand barrel at RIGHT_BARREL_X with pad width axis strictly parallel to the barrel (+y in
    cradle) so squeezing applies zero skew torque to the wand."""
    r_cradle = P._ry(16.0) @ frame((1.0, 0.0, 0.0), (0.0, 0.0, -1.0))
    carried = r_wand @ P.R_WAND_START.T
    pos = p_wand + r_wand @ np.array([RIGHT_BARREL_X, 0.0, 0.0]) + np.array([0.0, 0.0, lift_z])
    return pos, carried @ r_cradle


def hood_on_fiber_pose(p_wand: np.ndarray, r_wand: np.ndarray, bend_deg: float, seat_m: float) -> tuple[np.ndarray, np.ndarray]:
    p_loc, r_loc = P.fiber_frame_local(bend_deg, seat_m)
    return p_wand + r_wand @ p_loc, r_wand @ r_loc


def left_on_hood(p_hood: np.ndarray, r_hood: np.ndarray, radial_extra: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Left gripper pinching the real 3.0 mm +y_h wing of ``hood`` (no boss)."""
    pos = p_hood + (WING_GRIP_R + radial_extra) * r_hood[:, 1]
    rot = frame(r_hood[:, 0], -r_hood[:, 1]) @ np.diag([-1.0, -1.0, 1.0])
    return pos, rot


def left_on_button(p_wand: np.ndarray, r_wand: np.ndarray, above: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Closed Left fingertip pressing ``btn_cap`` on top of ``wand``, angled 28 deg from +y so both wrists stay well apart."""
    import demo_kit as kit

    btn_top = p_wand + r_wand @ np.array([P.BTN_X_LOCAL, 0.0, P.WAND_H / 2.0 + P.BTN_H])
    # Approach from +z_w tilted 28 deg toward +y (away from the Right wrist at -y)
    app = -0.88 * r_wand[:, 2] + np.array([0.0, -0.47, 0.0])
    app = app / np.linalg.norm(app)
    rot = frame(r_wand[:, 0], app) @ np.diag([-1.0, -1.0, 1.0])
    tip_off = kit.lowest_point(rot)
    tcp = btn_top - rot @ tip_off + above * r_wand[:, 2]
    return tcp, rot


def poses() -> dict[str, tuple[str, np.ndarray, np.ndarray, float, str]]:
    """Every waypoint of the scripted reference plan: name -> (side, pos, rot, jaw_opening_m, phase)."""
    pw0, rw0 = P.wand_start_pose(TOP)
    pwc, rwc = P.wand_cure_pose(TOP)
    ph0, rh0 = P.hood_start_pose(TOP)

    pr_c, rr_c = right_on_wand(pw0, rw0)
    pr_cure, rr_cure = right_on_wand(pwc, rwc)

    ps_h, rs_h = left_on_hood(ph0, rh0)
    pt_h, rt_h = left_on_hood(*hood_on_fiber_pose(pw0, rw0, P.FIBER_BEND_DEG, 0.0))
    pm_h, rm_h = left_on_hood(*hood_on_fiber_pose(pw0, rw0, 27.5, 0.0))
    pe_h, re_h = left_on_hood(*hood_on_fiber_pose(pw0, rw0, 0.0, 0.0))
    pf_h, rf_h = left_on_hood(*hood_on_fiber_pose(pw0, rw0, 0.0, P.SEAT_TRAVEL))
    p_off, r_off = left_on_hood(*hood_on_fiber_pose(pw0, rw0, 0.0, P.SEAT_TRAVEL), radial_extra=0.035)
    _, r_tip_only = hood_on_fiber_pose(pw0, rw0, P.FIBER_BEND_DEG, 0.0)
    p_pre = pt_h + 0.016 * r_tip_only[:, 0]

    pb_above, rb = left_on_button(pwc, rwc, above=0.028)
    pb_press, _ = left_on_button(pwc, rwc, above=-0.0022)

    return {
        "r: above cradle": ("right", pr_c + np.array([0.0, 0.0, 0.055]), rr_c, MAX_JAW, "cradle"),
        "r: grasp wand": ("right", pr_c, rr_c, P.WAND_W, "cradle"),
        "r: lift wand": ("right", pr_c + np.array([0.0, 0.0, 0.048]), rr_c, P.WAND_W, "lift"),
        "r: cure molar": ("right", pr_cure, rr_cure, P.WAND_W, "cure"),
        "l: above hood slot": ("left", ps_h + np.array([0.0, 0.0, 0.040]), rs_h, MAX_JAW, "cradle"),
        "l: pinch hood wing": ("left", ps_h, rs_h, P.HOOD_T, "cradle"),
        "l: lift hood from slot": ("left", ps_h + np.array([0.0, 0.0, 0.032]), rs_h, P.HOOD_T, "cradle"),
        "l: pre-tip align": ("left", p_pre, rt_h, P.HOOD_T, "cradle"),
        "l: enter 55-deg tip": ("left", pt_h, rt_h, P.HOOD_T, "cradle"),
        "l: mid 27.5-deg bend": ("left", pm_h, rm_h, P.HOOD_T, "cradle"),
        "l: finish 55-deg bend": ("left", pe_h, re_h, P.HOOD_T, "cradle"),
        "l: seat 22 mm home": ("left", pf_h, rf_h, P.HOOD_T, "cradle"),
        "l: release hood": ("left", p_off, r_off, MAX_JAW, "cradle"),
        "l: above power button": ("left", pb_above, rb, 0.0, "cure"),
        "l: press power button": ("left", pb_press, rb, 0.0, "cure"),
    }


TOGETHER = (
    ("r: grasp wand", "l: pre-tip align"),
    ("r: grasp wand", "l: enter 55-deg tip"),
    ("r: grasp wand", "l: mid 27.5-deg bend"),
    ("r: grasp wand", "l: finish 55-deg bend"),
    ("r: grasp wand", "l: seat 22 mm home"),
    ("r: grasp wand", "l: release hood"),
    ("r: cure molar", "l: above power button"),
    ("r: cure molar", "l: press power button"),
)


def solve_pose(side: str, pos: np.ndarray, rot: np.ndarray) -> tuple[np.ndarray, float, float, float]:
    best = None
    seeds = [
        np.array(READY, dtype=float),
        np.array([0.0, 1.2, 1.2, -0.5, 0.0, 0.0]),
        np.array([0.0, 1.5, 1.5, -0.4, 0.0, 0.0]),
        np.array([0.5, 1.4, 1.2, -0.8, 0.4, 0.0]),
        np.array([-0.5, 1.4, 1.2, -0.8, -0.4, 0.0]),
    ]
    quat = ik.matrix_to_quat_xyzw(rot)
    for q0 in seeds:
        q, pe, re = ik.solve(np.asarray(pos, dtype=float), quat, q0, base_pos=BASES[side], iters=300)
        margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
        ok = pe * 1000.0 <= IK_MM and math.degrees(re) <= IK_DEG
        key = (ok, margin if ok else -(pe * 1000.0 + math.degrees(re)))
        if best is None or key > best[0]:
            best = (key, q, float(pe * 1000.0), float(math.degrees(re)), margin)
    assert best is not None
    return best[1], best[2], best[3], best[4]


def hand_hulls(side: str, q: np.ndarray, opening: float) -> list:
    half = -0.5 * max(0.0, min(MAX_JAW, opening))
    joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, q)}, **dict.fromkeys(C.FINGER_JOINTS, half)}
    poses_w = analysis.fk_link_poses_world(joints, tuple(BASES[side]))
    out = []
    for link in HAND_LINKS:
        for h in analysis.link_hulls().get(link, []):
            r, p = poses_w[link][:3, :3], poses_w[link][:3, 3]
            n_w = h.planes[:, :3] @ r.T
            out.append((link, analysis.Hull(vertices=h.vertices @ r.T + p, planes=np.c_[n_w, h.planes[:, 3] + n_w @ p])))
    return out


def box_hull(centre, rotation, size) -> analysis.Hull:
    centre, rotation, half = np.asarray(centre, float), np.asarray(rotation, float), np.asarray(size, float) / 2.0
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
    vertices = corners @ rotation.T + centre
    planes = []
    for i in range(3):
        for s in (1.0, -1.0):
            n = s * rotation[:, i]
            planes.append([*n, float(n @ centre + half[i])])
    return analysis.Hull(vertices=vertices, planes=np.array(planes))


def gap(hulls_a, hulls_b) -> float:
    best = math.inf
    for _, a in hulls_a:
        for _, b in hulls_b:
            best = min(best, float(analysis._signed(a.vertices, b).min()), float(analysis._signed(b.vertices, a).min()))
    return best


def fixture_hulls() -> dict[str, list]:
    bench = [("bench", box_hull((0.30, 0.0, TOP - 0.020), np.eye(3), (0.60, 0.70, 0.040)))]
    rj = P.R_JAW
    j0 = np.array([P.JAW_XY[0], P.JAW_XY[1], TOP])
    jaw = [
        ("jaw_lower", box_hull(j0 + rj @ np.array([0.0, 0.0, P.LOWER_ARCH_H / 2.0]), rj, (P.JAW_D, P.JAW_W, P.LOWER_ARCH_H))),
        ("jaw_upper", box_hull(j0 + rj @ np.array([0.004, 0.0, 0.058]), rj @ P._ry(25.3), (0.072, P.JAW_W, 0.018))),
    ]
    return {"bench": bench, "jaw": jaw}


def run_offline() -> dict:
    found, rows, ok = {}, {}, True
    fixtures = fixture_hulls()
    for name, (side, pos, rot, opening, _) in poses().items():
        q, mm, deg, margin = solve_pose(side, pos, rot)
        found[name] = (side, q, opening, pos)
        hulls = hand_hulls(side, q, opening)
        z_above = float(pos[2] - TOP)
        clear = {k: round(1000.0 * gap(hulls, v), 1) for k, v in fixtures.items()}
        good = (
            mm <= IK_MM
            and deg <= IK_DEG
            and margin >= MARGIN_DEG
            and z_above <= MAX_GRASP_Z
            and min(clear.values()) > 0.0
        )
        ok &= good
        rows[name] = {
            "side": side,
            "at_m": [round(float(v), 4) for v in pos],
            "z_above_bench_mm": round(1000.0 * z_above, 1),
            "ik_mm_deg": [round(mm, 3), round(deg, 3)],
            "joint_margin_deg": round(margin, 1),
            "clear_mm": clear,
            "ok": good,
        }

    # Check that at the cure pose the shielded wand + seated hood clear the propped-open jaw model
    pwc, rwc = P.wand_cure_pose(TOP)
    phc, rhc = hood_on_fiber_pose(pwc, rwc, 0.0, P.SEAT_TRAVEL)
    hood_cure_hull = [("hood_seated", box_hull(phc, rhc, (P.HOOD_T, P.HOOD_W, P.HOOD_H)))]
    hood_jaw_gap_mm = round(1000.0 * gap(hood_cure_hull, fixtures["jaw"]), 1)
    ok &= hood_jaw_gap_mm > 2.0

    pairs = {}
    for a, b in TOGETHER:
        (sa, qa, oa, pa), (sb, qb, ob, pb) = found[a], found[b]
        tcp_sep = float(np.linalg.norm(pa - pb))
        hull_gap = gap(hand_hulls(sa, qa, oa), hand_hulls(sb, qb, ob))
        good = tcp_sep >= 0.080 and hull_gap >= 0.010
        ok &= good
        pairs[f"{a} + {b}"] = {
            "tcp_apart_mm": round(1000.0 * tcp_sep, 1),
            "hulls_apart_mm": round(1000.0 * hull_gap, 1),
            "ok": good,
        }
    return {"pass": bool(ok), "hood_to_jaw_at_cure_mm": hood_jaw_gap_mm, "poses": rows, "both_hands": pairs}


def run_frame0(out_dir: Path) -> dict:
    import cv2
    import demo_kit as kit
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack

    out_dir.mkdir(parents=True, exist_ok=True)
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    obs = emb.reset(Scene(id="g1", instruction=m.INSTRUCTION), seed=0)
    for cam, stem in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        bgr = cv2.cvtColor(np.asarray(obs.images[cam]), cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(out_dir / f"{stem}.png"), bgr)
        cv2.imwrite(str(out_dir / f"{stem}_policy.png"), cv2.resize(bgr, (320, 176), interpolation=cv2.INTER_AREA))
    ready = np.array([*READY, 1.0], dtype=float)
    action = Action(data=pack(ready, ready))
    first_wand, first_hood = None, None
    res = None
    for _ in range(40):
        res = emb.step(action)
        if first_wand is None:
            first_wand = m.mk.pose("wand")[0].copy()
            first_hood = m.mk.pose("hood")[0].copy()
    drift_wand_mm = round(1000.0 * float(np.linalg.norm(m.mk.pose("wand")[0] - first_wand)), 2)
    drift_hood_mm = round(1000.0 * float(np.linalg.norm(m.mk.pose("hood")[0] - first_hood)), 2)
    assert res is not None
    score_idle = float(res.info.get("score", 0.0))
    ok = score_idle == 0.0 and m.STATE.get("zero") is None and drift_wand_mm < 2.0 and drift_hood_mm < 2.0
    out = {
        "pass": ok,
        "drift_2s_mm": {"wand": drift_wand_mm, "hood": drift_hood_mm},
        "score_after_2s": score_idle,
        "report": m.report(),
    }
    (out_dir / "g1_frame0.json").write_text(json.dumps(out, indent=1, default=str))
    return out


def main() -> int:
    if "--parts" in sys.argv:
        print(json.dumps(P.summary(), indent=1))
        return 0
    offline = run_offline()
    print("G1_OFFLINE " + json.dumps(offline, indent=1), flush=True)
    ok = bool(offline["pass"])
    if "--frame0" in sys.argv:
        f0 = run_frame0(Path(sys.argv[sys.argv.index("--frame0") + 1]))
        print("G1_FRAME0 " + json.dumps(f0, indent=1, default=str), flush=True)
        ok &= bool(f0["pass"])
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
