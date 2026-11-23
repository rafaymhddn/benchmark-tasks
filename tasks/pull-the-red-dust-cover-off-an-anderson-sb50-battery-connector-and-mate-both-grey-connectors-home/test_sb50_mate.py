"""Tests of the SB50 task's rules and geometry without the simulator (scripts/rta check mech/tasks/sb50_mate)."""

from __future__ import annotations

import ast
import math
import sys
from pathlib import Path

import numpy as np
import pytest
from yam_arena import task_file

HERE = Path(__file__).resolve().parent
FAR = {"left": np.array([[0.10, 0.40, 1.10]]), "right": np.array([[0.10, -0.40, 1.10]])}
FLIPPED, SAME_WAY = np.diag([-1.0, 1.0, -1.0]), np.diag([-1.0, -1.0, 1.0])


@pytest.fixture
def m():
    saved = {name: sys.modules.get(name) for name in ("mechkit", "parts", "scene", "checks", "demo")}
    module = task_file.load(HERE / "task.py").module
    module.mk.mend_all()
    module.begin(0)
    yield module
    module.mk.MECH.clear()
    for name, old in saved.items():
        sys.modules.pop(name, None)
        if old is not None:
            sys.modules[name] = old


def level(side: str) -> np.ndarray:
    yaw = -math.pi / 2 if side == "left" else math.pi / 2
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def pair(m, span: float, relative=FLIPPED, height: float = 0.010, mid=(0.255, 0.0), pitch_deg: float = 0.0, offset=(0.0, 0.0, 0.0)) -> dict:
    """The left housing and the right one in line, ``span`` rear face to rear face, axis ``height`` over the mat."""
    tip = m.mk.axis_angle((1.0, 0.0, 0.0), math.radians(pitch_deg))
    rl = tip @ level("left")
    centre = np.array([mid[0], mid[1], m.TOP + m.P.MAT_T + height])
    pl = centre - rl[:, 0] * span / 2
    pr = pl + rl[:, 0] * span + np.asarray(offset, float)
    cover = (np.array([mid[0] + 0.06, mid[1] + 0.07, m.TOP + m.P.CAP_H / 2]), level("left"))
    return {"sb_left": (pl, rl), "sb_right": (pr, rl @ relative), "dust_cover": cover}


def put(m, poses=None, joints=None, hands=FAR) -> None:
    m.mk.set_state(poses=poses or {}, joints={"cap_slide": 0.0, "mate_wipe": 0.0, **(joints or {})})
    m.STATE["hands"] = hands


def steps(m, n: int = 1) -> float:
    for _ in range(n):
        m.step()
    return m.score(None)


def cover_off(m) -> None:
    put(m, joints={"cap_slide": m.P.CAP_ON + 0.001})
    m.step()
    assert m.mk.is_cut("w_cap")


def test_no_pose_is_set_outside_the_trials():
    for name in ("parts.py", "scene.py", "task.py", "demo.py", "checks.py"):
        calls = [n.func.attr if isinstance(n.func, ast.Attribute) else getattr(n.func, "id", "") for n in ast.walk(ast.parse((HERE / name).read_text()))
                 if isinstance(n, ast.Call)]  # fmt: skip
        assert "write_pose" not in calls and "set_joint" not in calls, name


def test_sizes_are_the_makers(m):
    P = m.P
    assert (P.SB_L, P.SB_W, P.SB_H, P.SB_MATED_L) == (0.0480, 0.0365, 0.0159, 0.0813)
    assert (P.HDL_L, P.HDL_W, P.HDL_T, P.HDL_STEM_W, P.HDL_PROJ) == (0.0567, 0.0486, 0.0134, 0.0139, 0.0251)
    assert (P.CAP_W, P.CAP_H, P.MATE_N, P.WIRE_OD) == (0.0465, 0.0279, 44.0, 0.0112)
    assert P.NOSE_L == pytest.approx(0.0147)
    for name, (value, unit, kind, note) in P.FIGURES.items():
        assert kind in (P.MAKER, P.FIXTURE, P.ESTIMATE) and note, name
        if kind == P.MAKER:
            assert "http" in note and "'" in note, name  # the document's words and its address


def _solid_boxes(m):
    """The connector's solid shapes as boxes (cylinders by their square), connector frame."""
    out = []
    for s in m.scene.connector_shapes():
        if not s.solid:
            continue
        lo, hi = s.bounds()
        out.append((np.asarray(lo), np.asarray(hi)))
    return out


def _overlap(m, relative, span: float) -> float:
    """Deepest overlap (m) between the solid shapes of two connectors ``span`` apart, the second turned ``relative``."""
    boxes = _solid_boxes(m)
    worst = -1.0
    for lo1, hi1 in boxes:
        for lo2, hi2 in boxes:
            c = np.array([[x, y, z] for x in (lo2[0], hi2[0]) for y in (lo2[1], hi2[1]) for z in (lo2[2], hi2[2])]) @ relative.T + np.array([span, 0.0, 0.0])
            lo, hi = c.min(axis=0), c.max(axis=0)
            depth = float(np.min(np.minimum(hi1, hi) - np.maximum(lo1, lo)))
            worst = max(worst, depth)
    return worst


def test_noses_fit_only_turned_over(m):
    # turned over, the noses slide together to 81.0 mm and no further; the same way up tongue meets tongue at 95.4 mm
    assert _overlap(m, FLIPPED, 0.0812) < 0.0
    assert _overlap(m, FLIPPED, 0.0805) > 0.0
    assert _overlap(m, SAME_WAY, 0.0956) < 0.0
    assert _overlap(m, SAME_WAY, 0.0950) > 0.0


def test_idle_start_scores_zero(m):
    put(m)
    assert steps(m, 3) == 0.0
    al = m.alignment()
    assert al["turn"] == pytest.approx(0.0, abs=6.0) and al["span"] > 0.25
    assert not m.mk.is_cut("w_cap") and m.mk.is_cut("w_mate")


def test_cover_freed_at_16_mm_then_quarter(m):
    put(m, joints={"cap_slide": m.P.CAP_ON - 0.001})
    assert steps(m) == 0.0 and not m.mk.is_cut("w_cap")
    put(m, poses=pair(m, 0.30), joints={"cap_slide": m.P.CAP_ON + 0.001})
    assert steps(m) == 0.25 and m.mk.is_cut("w_cap")


def test_cover_near_the_nose_is_not_off(m):
    cover_off(m)
    poses = pair(m, 0.30)
    pl, rl = poses["sb_left"]
    poses["dust_cover"] = (pl + rl @ np.array([0.048 + 0.011 + 0.003, 0.0, 0.0]), rl)  # 3 mm in front of the nose
    put(m, poses=poses)
    assert m.cover_clear() < m.P.CAP_CLEAR and steps(m) == 0.0


def test_half_needs_turned_over_and_in_line(m):
    cover_off(m)
    put(m, poses=pair(m, 0.088))
    assert steps(m) == 0.5
    put(m, poses=pair(m, 0.088, SAME_WAY))  # same way up: no half, whatever the span
    assert steps(m) == 0.25
    put(m, poses=pair(m, 0.088, offset=(0.006, 0.0, 0.0)))  # axes 6 mm apart
    assert steps(m) == 0.25
    put(m, poses=pair(m, 0.0925))  # 3.5 mm overlap only
    assert steps(m) == 0.25


def test_mate_is_joined_where_the_housings_stand(m):
    cover_off(m)
    put(m, poses=pair(m, 0.0935))
    m.step()
    assert not m.mk.is_cut("w_mate") and m.STATE["ov0"] == pytest.approx(0.0025, abs=1e-6)


def test_mate_slide_closes_downwards_and_cuts_when_pulled_apart(m):
    # the slide's rail moves back along the left housing's axis as the housings close: its travel falls
    cover_off(m)
    put(m, poses=pair(m, 0.0935))
    m.step()
    assert not m.mk.is_cut("w_mate")
    put(m, poses=pair(m, 0.0835), joints={"mate_wipe": -0.010})  # pushed 10 mm further in: still joined
    m.step()
    assert not m.mk.is_cut("w_mate")
    put(m, poses=pair(m, 0.0956), joints={"mate_wipe": 0.0021})  # pulled 2.1 mm apart from 2.5 mm in: cut
    m.step()
    assert m.mk.is_cut("w_mate")


def test_mate_not_joined_same_way_up_or_with_cover_on(m):
    put(m, poses=pair(m, 0.0935))  # cover still on
    m.step()
    assert m.mk.is_cut("w_mate")
    cover_off(m)
    put(m, poses=pair(m, 0.0935, SAME_WAY))
    m.step()
    assert m.mk.is_cut("w_mate")


def test_full_after_one_second_still_on_the_mat_hands_clear(m):
    cover_off(m)
    put(m, poses=pair(m, 0.0810))
    assert steps(m, m.P.HOLD_STEPS) == 0.5  # the first step has no earlier pose to compare: 19 steps still
    assert steps(m) == 1.0
    report = m.report()
    assert report["on_mat"] and report["score"] == 1.0


@pytest.mark.parametrize("change", ["span_short", "span_long", "high", "pitch", "off_mat", "hands"])
def test_full_bounds_fall_back_to_half(m, change):
    cover_off(m)
    kw = {"span_short": {"span": 0.0790}, "span_long": {"span": 0.0838}, "high": {"span": 0.0810, "height": 0.0275},
          "pitch": {"span": 0.0810, "pitch_deg": 16.0, "height": 0.020}, "off_mat": {"span": 0.0810, "mid": (0.255, 0.070)}}.get(change, {"span": 0.0810})  # fmt: skip
    span = kw.pop("span")
    hands = {"left": np.array([[0.255, 0.04, m.TOP + 0.04]]), "right": FAR["right"]} if change == "hands" else FAR
    put(m, poses=pair(m, span, **kw), hands=hands)
    assert steps(m, m.P.HOLD_STEPS + 2) == 0.5


def test_moving_pair_is_not_at_rest(m):
    cover_off(m)
    for k in range(m.P.HOLD_STEPS + 2):
        put(m, poses=pair(m, 0.0810, mid=(0.255, 0.001 * k)))  # 1 mm a step: 20 mm/s
        m.step()
    assert m.score(None) == 0.5


def test_tiers_never_skip(m):
    # a full pair without the cover off scores nothing: the mate needs the nose bare
    put(m, poses=pair(m, 0.0810))
    assert steps(m, m.P.HOLD_STEPS + 2) == 0.0


def test_texts_agree(m):
    import json

    data = json.loads((HERE / "task.json").read_text())
    module_text = (HERE / "task.py").read_text()
    assert data["instruction"] == m.INSTRUCTION and m.INSTRUCTION.endswith(", then let go and move both hands clear.")
    for figure in ("79.3", "83.5", "81.3", "18 mm", "15 degrees", "30 mm", "1.0 s", "91.0", "5 mm"):
        assert figure in data["scoring"], figure
    assert data["scores"] == "Dream-1 run: pending; demo: 1.0"
    assert len(data["title"]) <= 120 and len(data["context"]) <= 1000 and len(data["fidelity"]) <= 500
    assert len((HERE / "README.md").read_text()) <= 10000 and len(module_text) <= 15000
