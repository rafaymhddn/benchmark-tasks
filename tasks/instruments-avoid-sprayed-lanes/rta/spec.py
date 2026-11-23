"""Memory task files: a yam_arena task file plus the hooks a memory task needs. Pure Python (no Isaac).

A memory task file is a Python module (importing nothing from Isaac Sim) that declares:

- the module docstring: what is on the table and what the robot must do (agents read it);
- ``SCENE`` (``"station"`` default, or ``"tabletop"``) and ``OBJECTS``: ``Box``, ``Cylinder``, ``Sphere``,
  ``Asset`` from ``yam_arena.task_file`` and :class:`Compound` (several cuboids as one rigid body: an open-top
  box, a lid with a knob). An object with a ``mass`` is free; one without is fixed;
- ``INSTRUCTION``: the language goal handed to the policy;
- ``ACT_AFTER_S``: the arms are held in the ready pose until this time; before it the scene plays its
  scripted events (an object is hidden, a cue is shown and removed) and the policy can only watch;
- ``setup(rng, trial) -> dict``: the episode's hidden state. ``trial.init_seed`` is the scene's own seed number
  (``None`` for an ad-hoc scene) and ``trial.seed`` the derived trial seed; balance discrete hidden choices
  (which box, which colour) by construction from ``init_seed`` so a policy cannot profit from a skewed draw, and
  take continuous jitter from ``rng``;
- ``placements(state) -> dict[name, (x, y, z) | (x, y, z, yaw)]`` (optional): where free objects start this episode;
- ``events(t, state, world)``: called before every control step with the time (s) since reset; moves objects
  kinematically through ``world.set_pose(name, pos, yaw)``;
- ``MILESTONES``: ordered milestone names, and ``milestones(objects, state) -> list[bool]`` giving the current
  truth of each. A milestone latches once true (after ``ACT_AFTER_S``); the Q-score is the sum of the latched
  milestones' ``WEIGHTS`` (optional, default equal weights summing to 1).
  ``objects`` is a :class:`Snapshot`: a dict of :class:`ObjectState` per object that also carries ``t`` (s),
  ``tcp[arm]`` (each gripper's tool centre, world m) and ``finger[arm]`` (finger joint, 0 closed .. -0.047 open).
  Hooks may record what they need in ``state`` (e.g. when a milestone was reached);
- ``hold(objects, state) -> bool`` (optional): while true the arms are held in the ready pose and the policy's
  actions are ignored (an interruption mid-episode); milestones and failures are not evaluated then;
- ``success(objects, state) -> bool``: the final condition; it must hold for ``SUCCESS_HOLD_S`` (default 1.0)
  seconds in a row to end the episode with ``termination_reason="success"``;
- ``failure(objects, state) -> str | None`` (optional): a reason that ends the episode at once (the wrong box
  opened, the object on the floor). Reasons listed in ``ZERO_ON`` (optional set) also set the score to 0 — the
  program's "capped at" rule for hazards; other reasons keep the score reached so far (END);
- ``demo(state) -> dict[arm, (start_s, [Waypoint])]``: the privileged scripted solution ``rta_oracle`` plays:
  per arm, when (seconds after ``ACT_AFTER_S``) its waypoints start.

World frame as in yam_arena: x forward (away from the arms), y left, z up, metres; ``TABLE_TOP[SCENE]`` is the
table height. Hidden state never reaches the policy: it lives in ``state`` and in the simulator only.
"""

from __future__ import annotations

import hashlib
import importlib.util
import math
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from yam_arena.task_file import (
    RESERVED_NAMES,
    SCENES,
    SUCCESS_HOLD_S,
    TABLE_TOP,
    Asset,
    Box,
    Cylinder,
    ObjectState,
    Sphere,
    _Shape,
    _vec,
)

__all__ = [
    "TABLE_TOP",
    "SUCCESS_HOLD_S",
    "Asset",
    "Box",
    "Compound",
    "Cylinder",
    "MemoryTask",
    "ObjectState",
    "Part",
    "Snapshot",
    "Sphere",
    "Trial",
    "load",
]

ARMS = ("left", "right")


class Snapshot(dict):
    """The task's view of one control step: ``{name: ObjectState}`` plus the time and the grippers."""

    def __init__(self, objects: dict, *, t: float, tcp: dict | None = None, finger: dict | None = None) -> None:
        super().__init__(objects)
        self.t = float(t)
        self.tcp = {k: np.asarray(v, dtype=float) for k, v in (tcp or {}).items()}
        self.finger = {k: float(v) for k, v in (finger or {}).items()}


@dataclass(frozen=True)
class Trial:
    """What a task's ``setup`` may condition on besides its random stream."""

    scene_id: str
    init_seed: int | None
    seed: int | None
    epoch: int = 0

    def balanced(self, n: int, rng) -> int:
        """A choice in ``range(n)`` that cycles with the scene's seed number (balanced across seeds 0..k*n-1),
        falling back to the random stream when the scene has no seed number."""
        if self.init_seed is None:
            return int(rng.integers(n))
        return int(self.init_seed) % n


@dataclass(frozen=True)
class Part:
    """One cuboid of a :class:`Compound`: ``size`` (x, y, z edge lengths, m) at ``offset`` from the root (m)."""

    size: tuple[float, float, float]
    offset: tuple[float, float, float]

    def _check(self, what: str) -> None:
        if any(s <= 0 for s in _vec(self.size, 3, f"{what}.size")):
            raise ValueError(f"{what}.size must be positive, got {self.size!r}")
        _vec(self.offset, 3, f"{what}.offset")


@dataclass(frozen=True)
class Compound(_Shape):
    """Several cuboids as one body: the root cuboid ``size`` centred at ``pos`` plus ``parts`` offset from it.
    With a ``mass`` it is one free rigid body (a lid with a knob); without, a fixed collider (an open box)."""

    size: tuple[float, float, float] = (0.1, 0.1, 0.01)
    parts: tuple[Part, ...] = ()

    def _check(self) -> None:
        super()._check()
        if any(s <= 0 for s in _vec(self.size, 3, f"{self.name}.size")):
            raise ValueError(f"{self.name}.size must be positive, got {self.size!r}")
        for i, part in enumerate(self.parts):
            if not isinstance(part, Part):
                raise TypeError(f"{self.name}.parts[{i}] must be a Part, got {part!r}")
            part._check(f"{self.name}.parts[{i}]")

    def extents(self) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        """Local axis-aligned bounds (low, high) of the root and its parts, before ``yaw``."""
        lows, highs = [], []
        for size, offset in [(self.size, (0.0, 0.0, 0.0)), *((p.size, p.offset) for p in self.parts)]:
            lows.append([o - s / 2 for s, o in zip(size, offset, strict=True)])
            highs.append([o + s / 2 for s, o in zip(size, offset, strict=True)])
        low = tuple(min(v[i] for v in lows) for i in range(3))
        high = tuple(max(v[i] for v in highs) for i in range(3))
        return low, high


ObjectSpec = Box | Cylinder | Sphere | Asset | Compound


@dataclass(frozen=True)
class MemoryTask:
    path: Path
    name: str
    sha256: str
    scene: str
    objects: tuple[ObjectSpec, ...]
    instruction: str
    act_after_s: float
    milestone_names: tuple[str, ...]
    weights: tuple[float, ...]
    zero_on: frozenset[str]
    setup: Callable[[Any, Trial], dict]
    events: Callable[[float, dict, Any], None]
    milestones: Callable[[dict[str, ObjectState], dict], list[bool]]
    success: Callable[[dict[str, ObjectState], dict], bool]
    demo: Callable[[dict], dict[str, tuple[float, list]]]
    placements: Callable[[dict], dict[str, tuple[float, float, float]]] | None = None
    failure: Callable[[dict[str, ObjectState], dict], str | None] | None = None
    hold: Callable[[dict[str, ObjectState], dict], bool] | None = None
    docs: str | None = None
    success_hold_s: float = SUCCESS_HOLD_S
    module: Any = field(default=None, repr=False, compare=False)

    @property
    def table_top(self) -> float:
        return TABLE_TOP[self.scene]

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(o.name for o in self.objects)

    @property
    def free_objects(self) -> tuple[str, ...]:
        return tuple(o.name for o in self.objects if o.free)

    def q_score(self, latched: list[bool]) -> float:
        """Partial credit: the summed weights of the milestones reached (equal weights by default)."""
        if len(latched) != len(self.milestone_names):
            raise ValueError(f"{self.name}: {len(latched)} flags for {len(self.milestone_names)} milestones")
        return float(min(1.0, sum(w for w, f in zip(self.weights, latched, strict=True) if f)))


def _callable(module, path: Path, name: str, required: bool) -> Callable | None:
    fn = getattr(module, name, None)
    if fn is None:
        if required:
            raise ValueError(f"{path.name}: {name}() is required")
        return None
    if not callable(fn):
        raise TypeError(f"{path.name}: {name} must be a function")
    return fn


def load(path: str | Path) -> MemoryTask:
    """Import a memory task file and check what it declares."""
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"task file not found: {path}")
    source = path.read_bytes()
    spec = importlib.util.spec_from_file_location(f"rta_task_{path.stem}", path)
    if spec is None or spec.loader is None:
        raise ValueError(f"cannot import {path} as a Python module")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    scene = getattr(module, "SCENE", "station")
    if scene not in SCENES:
        raise ValueError(f"{path.name}: SCENE must be one of {SCENES}, got {scene!r}")
    objects = tuple(getattr(module, "OBJECTS", ()))
    if not objects:
        raise ValueError(f"{path.name}: OBJECTS must list at least one object")
    for obj in objects:
        if not isinstance(obj, Box | Cylinder | Sphere | Asset | Compound):
            raise TypeError(f"{path.name}: OBJECTS holds {obj!r}; use Box, Cylinder, Sphere, Asset or Compound")
        obj._check()
    names = [o.name for o in objects]
    duplicates = sorted({n for n in names if names.count(n) > 1})
    if duplicates:
        raise ValueError(f"{path.name}: object names must be unique, repeated: {duplicates}")
    clashes = sorted(set(names) & RESERVED_NAMES)
    if clashes:
        raise ValueError(f"{path.name}: {clashes} already name part of the room or robot; rename them")
    instruction = getattr(module, "INSTRUCTION", None)
    if not isinstance(instruction, str) or not instruction.strip():
        raise ValueError(f"{path.name}: INSTRUCTION must be a non-empty string")
    act_after = float(getattr(module, "ACT_AFTER_S", 0.0))
    if not (math.isfinite(act_after) and act_after >= 0):
        raise ValueError(f"{path.name}: ACT_AFTER_S must be >= 0 seconds")
    milestone_names = tuple(getattr(module, "MILESTONES", ()))
    if not milestone_names or not all(isinstance(m, str) and m for m in milestone_names):
        raise ValueError(f"{path.name}: MILESTONES must be a non-empty tuple of names")
    if len(set(milestone_names)) != len(milestone_names):
        raise ValueError(f"{path.name}: MILESTONES must be unique")
    weights = getattr(module, "WEIGHTS", None)
    if weights is None:
        weights = tuple(1.0 / len(milestone_names) for _ in milestone_names)
    else:
        weights = tuple(float(w) for w in weights)
        if len(weights) != len(milestone_names) or any(w < 0 for w in weights):
            raise ValueError(f"{path.name}: WEIGHTS needs one non-negative weight per milestone")
        if abs(sum(weights) - 1.0) > 1e-6:
            raise ValueError(f"{path.name}: WEIGHTS must sum to 1, got {sum(weights):g}")
    zero_on = frozenset(getattr(module, "ZERO_ON", ()))
    if not all(isinstance(r, str) and r for r in zero_on):
        raise ValueError(f"{path.name}: ZERO_ON must hold failure-reason strings")
    hold = float(getattr(module, "SUCCESS_HOLD_S", SUCCESS_HOLD_S))
    if not hold >= 0:
        raise ValueError(f"{path.name}: SUCCESS_HOLD_S must be >= 0")
    hooks = {
        name: _callable(module, path, name, required)
        for name, required in (
            ("setup", True),
            ("events", True),
            ("milestones", True),
            ("success", True),
            ("demo", True),
            ("placements", False),
            ("failure", False),
            ("hold", False),
        )
    }
    docs = (module.__doc__ or "").strip() or None
    return MemoryTask(
        path=path,
        name=path.stem,
        sha256=hashlib.sha256(source).hexdigest(),
        scene=scene,
        objects=objects,
        instruction=instruction,
        act_after_s=act_after,
        milestone_names=milestone_names,
        weights=weights,
        zero_on=zero_on,
        docs=docs,
        success_hold_s=hold,
        module=module,
        **hooks,
    )


def check_demo(task: MemoryTask, plan: dict) -> dict[str, tuple[float, list]]:
    """Validate a demo(): ``{arm: (start_s, waypoints)}`` with known arms, finite starts and non-empty plans."""
    unknown = sorted(set(plan) - set(ARMS))
    if unknown:
        raise ValueError(f"{task.name}: demo() keys are arms {ARMS}, got {unknown}")
    out = {}
    for arm, entry in plan.items():
        try:
            start, waypoints = entry
        except (TypeError, ValueError) as e:
            raise ValueError(f"{task.name}: demo()[{arm!r}] must be (start_s, waypoints)") from e
        start = float(start)
        if not (math.isfinite(start) and start >= 0):
            raise ValueError(f"{task.name}: demo()[{arm!r}] start must be >= 0 s")
        waypoints = list(waypoints)
        if not waypoints:
            raise ValueError(f"{task.name}: demo()[{arm!r}] has no waypoints")
        out[arm] = (start, waypoints)
    return out
