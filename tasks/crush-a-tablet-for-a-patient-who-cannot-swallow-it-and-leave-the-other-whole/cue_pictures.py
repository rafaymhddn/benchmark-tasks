"""Gate G1: is the cue readable in the pictures a policy gets (320 x 176)?

Frame 0 of the three cameras, then the left hand held pointing down over the cue card at three heights; at each
the three cameras' pictures are kept as rendered (640 x 360) and as the policy gets them (policy_picture), and the
height of the card's capitals in the left wrist picture is worked out from the camera's pose and lens.

    scripts/rta simpy <id> mech/tasks/pill_crusher/cue_pictures.py /workspace/robocurve/outputs/<id>
"""

from __future__ import annotations

import json
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

CAMERAS = ("top_cam", "left_cam", "right_cam")


def aim_point(m) -> np.ndarray:
    """The middle of the card's printed line (symbol and words), on the card's face, world (m)."""
    y = m.CUP_YS[m.ER_PLACE]
    card_x = m.CUP_ROW_X - m.CARD_CUP_V
    c = m.PRINT[m.CARD]
    u_mid = (c["symbol_at"][0] - c["symbol_mm"] / 2 + c["words_at"][0] + c["words_mm"][0]) / 2
    v_mid = c["words_at"][1]
    return np.array([card_x + v_mid / 1000.0, y - u_mid / 1000.0, m.TOP + m.CARD_T])


def wrist_view(side: str, tcp, rot):
    """The wrist camera's world position, rotation (opengl: x right, y up, looking along -z) and policy-size lens."""
    from yam_arena import station
    from yam_arena.constants import TCP_OFFSET
    from yam_arena.urdf import quat_xyzw_to_matrix

    cam = station.wrist_camera(side)
    pos = np.asarray(tcp, dtype=float) + rot @ (np.array(cam.pos) - np.array(TCP_OFFSET))
    return pos, rot @ quat_xyzw_to_matrix(np.array(cam.rot_xyzw, dtype=float)), cam.lens.resized(320, 176)


def project(point, cam_pos, cam_rot, lens) -> np.ndarray:
    p = cam_rot.T @ (np.asarray(point, dtype=float) - cam_pos)
    return np.array([lens.cx + lens.fx * p[0] / -p[2], lens.cy - lens.fy * p[1] / -p[2]])


def main() -> int:
    import cv2
    import demo_kit as kit

    out = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "cue"
    out.mkdir(parents=True, exist_ok=True)
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    rot = kit.down(0.0)  # the fingers close along y: the pinch across a cup of the row
    aim = aim_point(m)
    card = m.PRINT[m.CARD]
    cap = card["cap_mm"] / 1000.0
    words_mid_u = (card["words_at"][0] + card["words_mm"][0] / 2) / 1000.0
    words_mid = np.array([aim[0], m.CUP_YS[m.ER_PLACE] - words_mid_u, aim[2]])
    er_y, cups = m.CUP_YS[m.ER_PLACE], [y for i, y in enumerate(m.CUP_YS) if i != m.ER_PLACE]
    grasp_z = m.TOP + 0.020  # the pinch across a cup, 20 mm over the bench
    poses = {"hand 8.5 cm over the card": aim + np.array([0.0, 0.0, 0.085]),
             "hand 4 cm over the card": aim + np.array([0.0, 0.0, 0.040]),
             "hand at the pinch on this cup": np.array([m.CUP_ROW_X, er_y, grasp_z]),
             "hand at the pinch on the next cup": np.array([m.CUP_ROW_X, min(cups, key=lambda y: abs(y - er_y)), grasp_z])}  # fmt: skip
    moves, keep, t = [], {}, 0.0
    for label, tcp in poses.items():
        lift = [kit.Move(1.5, pos=np.array([tcp[0], tcp[1], m.TOP + 0.085]), rot=rot, grip=0.7, label="over")] if tcp[2] < m.TOP + 0.05 else []
        moves += lift + [kit.Move(3.0, pos=tcp, rot=rot, grip=0.7, label=label[:16]), kit.Move(2.0, label="settle")]
        t += 5.0 + (1.5 if lift else 0.0)
        keep[round(t * 20)] = (label, tcp)
        if lift:
            moves.append(kit.Move(1.5, pos=np.array([tcp[0], tcp[1], m.TOP + 0.085]), rot=rot, label="up"))
            t += 1.5
    moves.append(kit.Move(3.0, pos=None, rot=None, label="back"))
    report = {"aim": aim.round(4).tolist(), "poses": []}
    step = {"i": 0}

    def director(t, arms, pictures):
        i = step["i"]
        step["i"] += 1
        if pictures is None:
            return
        name = "frame0" if i == 0 else None
        if i in keep:
            name = keep[i][0].replace(" ", "_").replace(".", "")
        if name is None:
            return
        for cam in CAMERAS:
            frame = pictures[cam]
            cv2.imwrite(str(out / f"{name}_{cam}_640.png"), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            small = kit.policy_picture(frame)
            cv2.imwrite(str(out / f"{name}_{cam}_320.png"), cv2.cvtColor(small, cv2.COLOR_RGB2BGR))
            big = cv2.resize(small, (small.shape[1] * 3, small.shape[0] * 3), interpolation=cv2.INTER_NEAREST)
            cv2.imwrite(str(out / f"{name}_{cam}_320x3.png"), cv2.cvtColor(big, cv2.COLOR_RGB2BGR))
        if i in keep:
            label, tcp = keep[i]
            cam_pos, cam_rot_i, lens = wrist_view("left", tcp, rot)
            up = np.array([cap / 2, 0.0, 0.0])
            a, b = project(words_mid - up, cam_pos, cam_rot_i, lens), project(words_mid + up, cam_pos, cam_rot_i, lens)
            ends = [project(words_mid + np.array([0.0, s * card["words_mm"][0] / 2000.0, 0.0]), cam_pos, cam_rot_i, lens) for s in (1, -1)]
            report["poses"].append({"pose": label, "grasp point above the bench mm": round(float(tcp[2] - m.TOP) * 1000, 1),
                                    "camera above the card mm": round(float(cam_pos[2] - aim[2]) * 1000, 1),
                                    "camera to the words mm": round(float(np.linalg.norm(cam_pos - words_mid)) * 1000, 1),
                                    "capitals px (planned pose)": round(float(np.linalg.norm(b - a)), 1),
                                    "words' length px": round(float(np.linalg.norm(ends[1] - ends[0])), 1),
                                    "words' middle at px": a.round(1).tolist()})  # fmt: skip

    result = kit.run(emb, {"left": moves}, out / "run", seed=0, director=director, tail_s=0.5, limit_s=45.0, record=False)
    report["worst_ik"] = result["worst_ik"]
    (out / "cue_report.json").write_text(json.dumps(report, indent=1))
    print("CUE " + json.dumps(report), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
