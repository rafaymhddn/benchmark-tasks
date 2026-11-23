"""What a task file's events() may do to the running simulation. Import after app start."""

from __future__ import annotations

import math

import numpy as np


class World:
    """What ``events()`` may do: ``set_pose`` teleports a free object (and zeroes its velocity); ``pose(name)`` /
    ``objects`` read where things are as of the last control step (so a prop can be re-picked where physics left
    it). Before the first step ``objects`` is None and ``pose`` raises."""

    def __init__(self, env, torch) -> None:
        self._base = env.unwrapped
        self._torch = torch
        self._origin = self._base.scene.env_origins[0]
        self.objects = None  # the latest Snapshot, set by the embodiment each step

    def pose(self, name: str) -> np.ndarray:
        """World position (bounding-box centre) of ``name`` after the last control step."""
        if self.objects is None:
            raise RuntimeError("object poses are available from the first control step on")
        return np.asarray(self.objects[name].pos, dtype=float)

    def set_pose(self, name: str, pos, yaw: float = 0.0) -> None:
        torch = self._torch
        if name not in self._base.scene.rigid_objects:
            raise KeyError(f"{name!r} is not a free object; only free objects can be moved")
        obj = self._base.scene[name]
        pose = obj.data.default_root_pose.torch.clone()
        pose[:, :3] = torch.tensor(np.asarray(pos, dtype=np.float64), dtype=pose.dtype, device=pose.device) + self._origin
        q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
        pose[:, 3:] = torch.tensor(q, dtype=pose.dtype, device=pose.device)
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))
