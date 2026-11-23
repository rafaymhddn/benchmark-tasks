"""Waypoint builders for oracle plans (per arm, played by ``rta_oracle``). NumPy only."""

from __future__ import annotations

import numpy as np
from yam_arena import ik, station
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY
from yam_arena.demo import Waypoint

def opening(value: float) -> float:
    """Finger joint target for a half-opening given in metres. The YAM finger joint runs from 0 (closed) to
    FINGER_OPEN = -0.047 (open), so a positive opening (the natural way to write "2 cm per finger") must be
    negated; a value already on the joint's negative axis passes through. Clipped to the joint range."""
    return float(max(FINGER_OPEN, -abs(value)))


CLOSE_S, SQUEEZE_S = 1.5, 0.5  # minimum close time; lifting before the pads have squeezed slips the grasp
FINGER_SPEED, FINGER_LAG = 0.02, 1.0  # measured in sim: ~2 cm/s per finger after a ~1 s lag (J3 release, S5 close)


def close_time(approach_finger: float) -> float:
    """Seconds for the fingers to close from ``approach_finger`` onto a thin object: the lag plus the full travel
    to closed at FINGER_SPEED, never less than CLOSE_S. (S5: 2.2 cm handle from fully open needed ~2.5 s; the
    old fixed 1.5 + 0.5 s lifted as the pads touched and dropped the iron.)"""
    return max(CLOSE_S, FINGER_LAG + abs(opening(approach_finger)) / FINGER_SPEED)
ALIGN_S = 0.6  # dwell over the place target before lowering: the TCP was still 5-8 mm off at the end of a carry (J2 hit a slot rail)
SETTLE_S = 0.8  # dwell at the bottom, still gripping, before the release: the descent itself adds ~5 mm of x/y error that converges when the arm is still (A4)
RELEASE_S = 2.0  # opening is as slow (measured ~1 s lag + ~1 s travel); a 0.8 s release lifted drinks 3 cm and dropped them (J3 sim)


def ready_tcp(arm: str) -> tuple[float, float, float]:
    """Where an arm's TCP is in the ready pose on the station rig (world, m)."""
    pos, _ = ik.tcp_pose(np.array(READY, dtype=float), station.BASES[arm])
    return tuple(float(v) for v in pos)


# A hover spot beside each arm's own base where it waits with a straight-down grip while the other arm works:
# reachable with a down grip (the ready pose itself is not; see scripts/park_probe.py), 60 cm from the other
# arm's spot so two parked grippers can never touch, and clear of the task area in front of the bases.
PARK = {"left": (0.12, +0.30, station.TABLE_TOP + 0.16), "right": (0.12, -0.30, station.TABLE_TOP + 0.16)}


def wait(arm: str, seconds: float, finger: float = FINGER_OPEN) -> Waypoint:
    """Hover at the arm's park spot for ``seconds``."""
    return Waypoint(seconds, PARK[arm], opening(finger), "wait")


def grasp(pos, *, travel_z: float, approach_finger: float = FINGER_OPEN, yaw: float = 0.0) -> list[Waypoint]:
    """Come down on ``pos`` from ``travel_z``, close, squeeze. ``yaw`` turns the grip about the vertical
    (0 closes the fingers along world y; pi/2 along world x)."""
    x, y, z = pos
    approach_finger = opening(approach_finger)
    return [
        Waypoint(1.5, (x, y, travel_z), approach_finger, "over the object", yaw),
        Waypoint(1.5, (x, y, z), approach_finger, "descend", yaw),
        Waypoint(close_time(approach_finger), (x, y, z), FINGER_CLOSED, "close", yaw),
        Waypoint(SQUEEZE_S, (x, y, z), FINGER_CLOSED, "squeeze", yaw),
    ]


def lift_and_hold(pos, *, hold_at, hold_s: float, yaw: float = 0.0) -> list[Waypoint]:
    """Lift from ``pos`` to ``hold_at`` and stay there (fingers closed)."""
    x, y, _ = pos
    return [
        Waypoint(1.2, (x, y, hold_at[2]), FINGER_CLOSED, "lift", yaw),
        Waypoint(1.5, tuple(hold_at), FINGER_CLOSED, "carry", yaw),
        Waypoint(hold_s, tuple(hold_at), FINGER_CLOSED, "hold", yaw),
    ]


def grasp_and_carry(pick, place, *, travel_z: float, approach_finger: float = FINGER_OPEN, release_finger: float = FINGER_OPEN, yaw: float = 0.0, place_yaw: float | None = None, settle_s: float = SETTLE_S) -> list[Waypoint]:
    """Top-down pick and place with time for the fingers to close before the lift. ``yaw`` turns the grip at the
    pick (fingers along world y at 0); ``place_yaw`` (default ``yaw``) is the grip's turn when setting down;
    ``settle_s`` (0 to skip) is the still dwell at the bottom before the release."""
    (px, py, pz), (qx, qy, qz) = pick, place
    release_finger = opening(release_finger)
    end_yaw = yaw if place_yaw is None else place_yaw
    return [
        *grasp((px, py, pz), travel_z=travel_z, approach_finger=approach_finger, yaw=yaw),
        Waypoint(1.2, (px, py, travel_z), FINGER_CLOSED, "lift", yaw),
        Waypoint(1.8, (qx, qy, travel_z), FINGER_CLOSED, "carry", end_yaw),
        Waypoint(ALIGN_S, (qx, qy, travel_z), FINGER_CLOSED, "align", end_yaw),
        Waypoint(1.2, (qx, qy, qz), FINGER_CLOSED, "lower", end_yaw),
        *([Waypoint(settle_s, (qx, qy, qz), FINGER_CLOSED, "settle", end_yaw)] if settle_s > 0 else []),
        Waypoint(RELEASE_S, (qx, qy, qz), release_finger, "release", end_yaw),
        Waypoint(1.0, (qx, qy, travel_z), release_finger, "retreat", end_yaw),
    ]


def press(pos, *, travel_z: float, press_s: float = 0.5) -> list[Waypoint]:
    """Press ``pos`` with the closed fingertips, coming from and returning to ``travel_z``."""
    x, y, z = pos
    return [
        Waypoint(1.5, (x, y, travel_z), FINGER_CLOSED, "over the button"),
        Waypoint(1.2, (x, y, z), FINGER_CLOSED, "press"),
        Waypoint(press_s, (x, y, z), FINGER_CLOSED, "hold press"),
        Waypoint(1.0, (x, y, travel_z), FINGER_CLOSED, "up"),
    ]


def duration(waypoints) -> float:
    return float(sum(w.duration for w in waypoints))
