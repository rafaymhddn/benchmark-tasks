"""The scripted full-score run (the task's ``demo``). The right hand presses the shutter button home from above while
the camera stands on the bench, lets it go and moves off; the print comes out under the film shield; the left comes
down on the camera's rear part and holds it across its width, lifts it and turns the lens up about the line between
its fingers. The print has waited more than 5 s: the right's closed fingertip comes in from the camera's right side
under the shield where it reaches past the print's edge and lifts it off the print, and the shield rolls back; the
right pinches the print's end across its thickness, draws it up out of the slot, stands it on its end leaning towards
its picture side and lets go: it falls face down; meanwhile the left turns the camera back, sets it down and lets go;
both move off. ``shield=False``: the print is drawn out from under the shield; ``lay_down=False``: it stays in the hand."""

from __future__ import annotations

import math

import numpy as np
from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint

from photo_geometry import CAM_SIZE, FRONT, GRIP, PRINT, PRINT_Z, SHIELD_UNDER, SHUTTER, SHUTTER_TRAVEL
from rta.plans import RELEASE_S, SETTLE_S, SQUEEZE_S, Pose, close_time, grip, opening, wait

TIP = 0.013  # the closed fingertips reach this far past the tool centre point
HALF_PI = math.pi / 2


def rodrigues(axis, angle: float) -> np.ndarray:
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


Y = np.array([0.0, 1.0, 0.0])
LEAN = math.radians(-15)  # the holding hand's fingers point back towards the arms this far: turned lens up, it then points
# 15 degrees down, within the arm's reach (straight down or leaning back, it would have to point level or up)
HOLD_ROT = grip((math.sin(LEAN), 0.0, -math.cos(LEAN)), (0.0, -1.0, 0.0))  # from above, across the camera's width
CAM_OPEN = opening(0.047)  # fully open: 5 mm either side of the 84 mm camera
CAM_HOLD = opening(CAM_SIZE[1] / 2 - 0.006)  # a firm, moderate squeeze
PRESS_PAST = 0.018  # the fingertips are sent this far past the button's travel: the arm gives 3.2 mm per newton
# downwards there (its joint stiffness, 80 and 10 N m/rad), so it pushes about 5 N against the spring's 4 N
LIFT = 0.03  # the camera is lifted this far before it is tipped: its lowest corner then stays 2.5 cm over the bench
TIP_UP, TIP_STEPS, TIP_S = HALF_PI, 3, 2.7  # the hand turns the lens up, about the line between its fingers
TIPPED = rodrigues(Y, -TIP_UP)  # the camera's rotation once the lens is up
LIFT_SAG = np.array([-0.003, 0.001, -0.0125])  # the holding arm, loaded, sits this far from its command (measured)
PRINT_AXIS = TIPPED @ np.array([1.0, 0.0, 0.0])  # the way the print points (and comes out) once tipped: up
PRINT_OPEN = 0.012  # per finger, around the print's thickness: the open fingers pass 5 mm over the lens
PINCH_IN, PINCH_BELOW_TIP = 0.012, 0.010  # the fingers reach this far in from the print's edge, this far below its end
SAG = np.array([0.0, 0.0, 0.0])  # the pinching arm, unloaded, reaches the pinch within 2 mm (measured)
PINCH_ROT = grip(Y, (1.0, 0.0, 0.0))  # from the right, along the print's width; closing across its thickness
PULL, PULL_S = 0.06, 2.0  # drawn 6 cm along the slot in 2 s: past the shield's ridge and the slot's lower lip
LAY_LEAN = math.radians(20)  # stood on its end leaning this far, towards its picture side: let go, it falls face down
LAY_ROT = rodrigues(Y, -LAY_LEAN) @ PINCH_ROT
LAY_XY = (0.38, -0.04)  # the print lies to the right of the camera and the apple, clear of both
# the shield lifted: the closed fingertip (10 mm across its closing axis, 18 mm wide, 13 mm long) comes in along the
# camera's y under the shield's overhang beside the print, 5 mm short of the print's edge, its near end 8 mm from the
# hinge, PUSH_GAP under the flap, and rises PUSH: the flap turns about 30 degrees, past the angle where it rolls back
# (the held camera sits about 2 mm higher on the camera's z than planned, which a run reads: then about 36 degrees)
PUSH_X = FRONT + 0.008 + 0.009  # the fingertip's middle along the camera's x
PUSH_GAP, PUSH = 0.004, 0.008  # m: under the flap before; lifted
PUSH_Y = -(PRINT[1] / 2 + 0.005) - TIP  # the tool centre point on the camera's y, the fingertip's end 5 mm short of the print


def points(cam_xy, top: float, protrude: float) -> dict:
    """Where the hands go, for the camera's origin at ``cam_xy`` on a bench at height ``top``, the print standing
    ``protrude`` out of the slot."""
    hold = np.array([cam_xy[0] + GRIP[0], cam_xy[1] + GRIP[1], top + GRIP[2]])
    held_up = hold + np.array([0.0, 0.0, LIFT])
    slot_up = held_up + TIPPED @ (np.array([FRONT, 0.0, PRINT_Z + PRINT[2] / 2]) - np.asarray(GRIP))  # the print's mid-plane at the slot
    pinch = slot_up + (protrude - PINCH_BELOW_TIP) * PRINT_AXIS + np.array([0.0, -PRINT[1] / 2 + PINCH_IN, 0.0]) + SAG
    return {
        "shutter": np.array([cam_xy[0] + SHUTTER[0], cam_xy[1] + SHUTTER[1], top + SHUTTER[2]]),  # the button's top at rest
        "hold": hold,
        "held_up": held_up,
        "pinch": pinch,
        "beside": pinch - 0.05 * Y,  # the open fingers level with the pinch, clear of the print's edge
        "lay_z": top + (PRINT[0] - PINCH_BELOW_TIP) * math.cos(LAY_LEAN) + 0.004,  # its lower end 4 mm over the bench
        # under the shield's overhang, the fingertip's upper face PUSH_GAP under the flap (camera frame, then turned)
        "push": held_up + TIPPED @ (np.array([PUSH_X, PUSH_Y, SHIELD_UNDER - PUSH_GAP - 0.005]) - np.asarray(GRIP)),
    }


def plan(cam_xy, top: float, protrude: float, end_hold_s: float, shield: bool = True, lay_down: bool = True) -> dict:
    """Both arms' waypoints, ending ``end_hold_s`` (the success hold) and 2 s after the last motion."""
    pts = points(cam_xy, top, protrude)
    sx, sy, sz = pts["shutter"]
    touch_z = sz + TIP + 0.004
    press_z = sz + TIP - SHUTTER_TRAVEL - PRESS_PAST
    high = touch_z + 0.03  # higher, a hand pointing straight down cannot reach so far out
    shoot = [
        Waypoint(2.0, (sx, sy, high), FINGER_CLOSED, "over the shutter", 0.0),
        Waypoint(1.0, (sx, sy, touch_z), FINGER_CLOSED, "onto the button", 0.0),
        Waypoint(1.0, (sx, sy, press_z), FINGER_CLOSED, "press", 0.0),
        Waypoint(0.5, (sx, sy, press_z), FINGER_CLOSED, "hold press", 0.0),
        Waypoint(0.8, (sx, sy, touch_z + 0.006), FINGER_CLOSED, "let go", 0.0),
        Waypoint(1.0, (sx, sy, high), FINGER_CLOSED, "up", 0.0),
    ]
    h = pts["hold"]
    over = h + np.array([0.0, 0.0, 0.08])
    take = [
        Pose(2.0, tuple(over), CAM_OPEN, "over the camera", rot=HOLD_ROT),
        Pose(1.5, tuple(h), CAM_OPEN, "down to it", rot=HOLD_ROT),
        Pose(close_time(0.047), tuple(h), CAM_HOLD, "close", rot=HOLD_ROT),
        Pose(SQUEEZE_S, tuple(h), CAM_HOLD, "squeeze", rot=HOLD_ROT),
    ]
    angles = [TIP_UP * (k + 1) / TIP_STEPS for k in range(TIP_STEPS)]
    rolls = [rodrigues(Y, -a) @ HOLD_ROT for a in angles]
    up = tuple(pts["held_up"] - LIFT_SAG)  # commanded so that the camera is where the pinch is planned
    tip = [
        Pose(1.2, up, CAM_HOLD, "lift", rot=HOLD_ROT),
        *[Pose(TIP_S / TIP_STEPS, up, CAM_HOLD, "tip the lens up", rot=r) for r in rolls],
        Pose(SETTLE_S, up, CAM_HOLD, "settle", rot=rolls[-1]),
    ]
    down_at = (h[0], h[1], h[2] + 0.002)
    back = [
        *[Pose(TIP_S / TIP_STEPS, up, CAM_HOLD, "tip back", rot=r) for r in [*rolls[-2::-1], HOLD_ROT]],
        Pose(1.4, down_at, CAM_HOLD, "set down", rot=HOLD_ROT),
        Pose(SETTLE_S, down_at, CAM_HOLD, "settle", rot=HOLD_ROT),
        Pose(RELEASE_S, down_at, CAM_OPEN, "release", rot=HOLD_ROT),
        Pose(1.2, tuple(over), CAM_OPEN, "up and off", rot=HOLD_ROT),
    ]
    open_print = opening(PRINT_OPEN)
    pinch, beside = pts["pinch"], pts["beside"]
    pulled = pinch + PULL * PRINT_AXIS
    fetch = [
        Pose(2.0, tuple(beside + np.array([0.0, 0.0, 0.03])), open_print, "beside the print", rot=PINCH_ROT),
        Pose(0.8, tuple(beside), open_print, "level with it", rot=PINCH_ROT),
        Pose(1.2, tuple(pinch), open_print, "onto the print", rot=PINCH_ROT),
        Pose(close_time(PRINT_OPEN), tuple(pinch), FINGER_CLOSED, "close", rot=PINCH_ROT),
        Pose(SQUEEZE_S, tuple(pinch), FINGER_CLOSED, "squeeze", rot=PINCH_ROT),
        Pose(PULL_S, tuple(pulled), FINGER_CLOSED, "pull out", rot=PINCH_ROT),
    ]
    lift_up = TIPPED @ np.array([0.0, 0.0, 1.0])  # the camera's up, turned: away from the print's picture side
    p0 = pts["push"]
    lift = [
        Pose(2.0, tuple(p0 - 0.04 * Y), FINGER_CLOSED, "beside the shield", rot=PINCH_ROT),
        Pose(1.0, tuple(p0), FINGER_CLOSED, "under its overhang", rot=PINCH_ROT),
        Pose(1.0, tuple(p0 + PUSH * lift_up), FINGER_CLOSED, "lift the shield", rot=PINCH_ROT),
        Pose(0.6, tuple(p0 + PUSH * lift_up), FINGER_CLOSED, "it rolls back", rot=PINCH_ROT),
        Pose(0.8, tuple(p0 + PUSH * lift_up - 0.04 * Y), FINGER_CLOSED, "out", rot=PINCH_ROT),
    ] if shield else []
    lx, ly = LAY_XY
    lay_z = pts["lay_z"]
    lay_out = np.array([lx, ly - 0.06, lay_z + 0.02])
    lay = [
        Pose(1.5, (pulled[0], pulled[1] - 0.06, pulled[2]), FINGER_CLOSED, "away from the camera", rot=PINCH_ROT),
        Pose(2.0, (lx, ly, lay_z + 0.04), FINGER_CLOSED, "carry", rot=LAY_ROT),
        Pose(1.2, (lx, ly, lay_z), FINGER_CLOSED, "stand it on its end", rot=LAY_ROT),
        Pose(SETTLE_S, (lx, ly, lay_z), FINGER_CLOSED, "settle", rot=LAY_ROT),
        Pose(RELEASE_S, (lx, ly, lay_z), opening(0.004), "let go", rot=LAY_ROT),
        Pose(1.5, tuple(lay_out), opening(0.004), "back out", rot=LAY_ROT),
    ]
    d = lambda wps: sum(w.duration for w in wps)  # noqa: E731
    if not lay_down:
        lay = lay[:2]  # it stays in the hand, carried clear of the camera
    d_shoot, d_take, d_tip, d_lift, d_fetch, d_lay, d_back = (d(p) for p in (shoot, take, tip, lift, fetch, lay, back))
    take_at = d_shoot + 0.8  # the right hand is on its way off
    tip_at = take_at + d_take
    fetch_at = tip_at + d_tip + d_lift  # the shield is lifted once the lens is up (the print out for 10 s and more)
    back_at = fetch_at + d_fetch + 1.5  # the print is out and away from the camera
    end = max(fetch_at + d_fetch + d_lay, back_at + d_back) + end_hold_s + 2.0
    left = [
        wait("left", take_at),
        *take,
        *tip,
        Pose(back_at - tip_at - d_tip, up, CAM_HOLD, "hold", rot=rolls[-1]),
        *back,
    ]
    left.append(wait("left", end - d(left)))
    right = [
        *shoot,
        wait("right", tip_at + d_tip - d_shoot, finger=FINGER_CLOSED),
        *lift,
        *fetch,
        *lay,
    ]
    right.append(wait("right", end - d(right)))
    return {"left": (0.0, left), "right": (0.0, right)}
