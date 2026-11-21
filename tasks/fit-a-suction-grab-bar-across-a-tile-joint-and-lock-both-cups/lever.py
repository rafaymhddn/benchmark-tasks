"""The flip levers' over-centre cam, a helper for task.py (it declares nothing in mechkit: the task file declares the
hinge with ``OverCentre.k`` and ``OverCentre.start_rest()``, and runs ``reset()`` and ``step()`` from its hooks).

A lever's travel: theta from released (0) to locked (LEVER_DEG). Released, it is loose from 0 to LEVER_CAM_DEG: no
force there. From LEVER_CAM_DEG the cam resists, with a torque that rises linearly to LEVER_N x LEVER_ARM at the
centre (LEVER_CENTRE_DEG) and turns the lever back towards released; past the centre it turns it on towards locked, falling
linearly to LEVER_HOLD x LEVER_N x LEVER_ARM at the locked end (``torque``). A finger needs LEVER_N at LEVER_ARM to
pass the centre either way; the lever snaps home beyond it, and let go short of the centre it comes to rest at
LEVER_CAM_DEG. The torque acts between the lever and the bar (the hinge's drive), so the bar takes its reaction.

How: PhysX's angular drive gives k x 2 sin(e/2) for a target e away, not k x e. With a fixed target 120 to 134 deg
away, the lever passed centre at 18.5 N against the 22 N built in (check run gb5t4; every reading of its push against
its angle fitted 2 sin(e/2) within 1 %). So the target follows the lever: each control step it is put e from the
lever, towards the end the cam turns it to, with k x 2 sin(e/2) the torque wanted there, e at most FOLLOW_DEG. Where
the lever is loose the target is the lever itself: the drive only damps it.
"""

from __future__ import annotations

import math

import parts as P

FOLLOW_DEG = 45.0  # deg: the farthest the drive's target is put from the lever (at the full torque)


class OverCentre:
    def __init__(self, mk, joint: str) -> None:
        self.mk, self.joint = mk, joint
        self.top = P.LEVER_N * P.LEVER_ARM  # N m at the centre
        self.k = self.top / (2.0 * math.sin(math.radians(FOLLOW_DEG) / 2.0))  # N m/rad: the full torque at FOLLOW_DEG
        self.locked = True
        self.flips: list[tuple[float, bool]] = []

    def theta(self) -> float:
        """rad from released, as of the control step being scored."""
        return P.LEVER_RAD + self.mk.joint(self.joint)

    def wanted(self, theta: float, locked: bool) -> float:
        """N m: the cam's torque at ``theta`` (towards released while unlocked, towards locked while locked)."""
        if locked:
            h = P.LEVER_HOLD
            share = h + (1 - h) * (P.LEVER_RAD - theta) / (P.LEVER_RAD - P.CENTRE_RAD)
            return self.top * min(1.0, max(h, share))
        return self.top * min(1.0, max(0.0, (theta - P.CAM_RAD) / (P.CENTRE_RAD - P.CAM_RAD)))

    def torque(self, theta: float, locked: bool) -> float:
        """N m on the lever at ``theta``, positive towards locked."""
        return self.wanted(theta, locked) if locked else -self.wanted(theta, locked)

    def target(self, theta: float, locked: bool) -> float:
        """The drive's target (hinge travel, rad) that gives ``torque`` at ``theta``."""
        e = 2.0 * math.asin(min(1.0, self.wanted(theta, locked) / (2.0 * self.k)))
        return (theta - P.LEVER_RAD) + (e if locked else -e)

    def start_rest(self) -> float:
        return self.target(P.LEVER_RAD, True)

    def _write(self, theta: float) -> None:
        if self.mk.MECH.environment is not None:
            self.mk.set_spring(self.joint, rest=self.target(theta, self.locked), most=1.05 * self.top)

    def reset(self) -> None:
        """The start: locked, down (where it was built)."""
        self.locked, self.flips = True, []
        if self.mk.MECH.environment is not None:
            self.mk.set_drive(self.joint, stiffness=self.k)
        self._write(P.LEVER_RAD)

    def step(self) -> bool | None:
        """Read the lever, switch its side if it crossed the centre, put the target by it. Returns True (it went down
        over centre), False (it came up over centre) or None."""
        theta = self.theta()
        locked = theta > P.CENTRE_RAD
        change = None
        if locked != self.locked:
            self.locked, change = locked, locked
            self.flips.append((self.mk.now(), locked))
        self._write(theta)
        return change
