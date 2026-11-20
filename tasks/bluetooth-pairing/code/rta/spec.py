"""Memory task files: a yam_arena task file plus the hooks a memory task needs. Pure Python (no Isaac).

A memory task file is a Python module (importing nothing from Isaac Sim) that declares:

- the module docstring: what is on the table and what the robot must do (agents read it);
- ``SCENE`` (``"station"`` default, or ``"tabletop"``) and ``OBJECTS``: ``Box``, ``Cylinder``, ``Sphere``,
  ``Asset`` from ``yam_arena.task_file`` and :class:`Compound` (several cuboids as one rigid body: an open-top
  box, a lid with a knob). An object with a ``mass`` is free; one without is fixed;
- ``INSTRUCTION``: the language goal handed to the policy;
- ``ACT_AFTER_S``: the arms are held in the ready pose until this time; before it the scene plays its
  scripted events (an object is hidden, a cue is shown and removed) and the policy can only watch;
- ``setup(rng, trial) -> dict`` (optional, default ``{}``): the episode's hidden state. ``trial.init_seed`` is the scene's own seed number
  (``None`` for an ad-hoc scene) and ``trial.seed`` the derived trial seed; balance discrete hidden choices
  (which box, which colour) by construction from ``init_seed`` so a policy cannot profit from a skewed draw, and
  take continuous jitter from ``rng``;
- ``placements(state) -> dict[name, (x, y, z) | (x, y, z, yaw)]`` (optional): where free objects start this episode;
- ``events(t, state, world)`` (optional): called before every control step with the time (s) since reset; moves objects
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
  per arm, when (seconds after ``ACT_AFTER_S``) its waypoints start;
- ``forces(objects, state) -> dict[name, (fx, fy, fz)]`` (optional): external forces (N, world frame, at the centre
  of mass) on free rigid objects, recomputed before every control step and held over its physics substeps — for
  effects PhysX lacks (a magnet's pull). An object left out keeps no force.

``OBJECTS`` may also hold a :class:`Sheet` (paper, card): thin rigid strips joined by spring hinges, spawned as an
articulation; tasks see it as a :class:`SheetState`.

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
    "Sheet",
    "SheetState",
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
    friction: tuple[float, float] | None = None  # (static, dynamic); None: the rubbery default of graspable props

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


@dataclass(frozen=True)
class Sheet(_Shape):
    """A sheet of paper or card standing upright: a chain of ``strips`` thin rigid strips, each the full ``width``,
    from the top strip down, joined along their long edges by hinges with a spring back to flat. Its plane is the
    local y-z plane (normal along local x, turned by ``yaw``); it bends about horizontal lines only (no twist, no
    stretch, no creases), and its strips do not collide with each other. Build one with :func:`sheet`; ``pos`` and
    ``size`` are its bounding box at spawn. It is loose, but not ``free`` in the rigid-object sense (placements and
    ``World.set_pose`` do not move it): it always spawns where it is declared."""

    size: tuple[float, float, float] = (0.002, 0.15, 0.10)
    top_centre: tuple[float, float, float] = (0.0, 0.0, 0.0)  # centre of the top strip (m)
    width: float = 0.148  # m, along the hinges (local y)
    height: float = 0.105  # m, top edge to bottom edge (local z)
    thickness: float = 0.002  # m, collision thickness (local x)
    strips: int = 5
    sheet_mass: float = 0.0031  # kg, whole sheet
    hinge_stiffness: float = 0.01  # N·m/rad per hinge
    hinge_damping: float = 0.0005  # N·m·s/rad
    armature: float = 1e-5  # kg·m² per hinge: keeps the light strips' drives stable (armature >> k·dt²)
    max_bend: float = 2.0  # rad per hinge (limit)
    friction: tuple[float, float] = (0.5, 0.5)  # (static, dynamic)

    @property
    def free(self) -> bool:
        return False

    @property
    def strip_h(self) -> float:
        return self.height / self.strips

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: a Sheet takes sheet_mass, not mass")
        if self.strips < 2 or not (self.width > 0 and self.height > 0 and self.thickness > 0 and self.sheet_mass > 0):
            raise ValueError(f"{self.name}: a Sheet needs strips >= 2 and a positive width, height, thickness and sheet_mass")

    def start_centres(self) -> np.ndarray:
        """Strip centres at spawn, (strips, 3), top to bottom."""
        t = np.asarray(self.top_centre, float)
        return np.stack([t - [0.0, 0.0, i * self.strip_h] for i in range(self.strips)])


def sheet(name: str, top, *, yaw: float = 0.0, width: float = 0.148, height: float = 0.105, thickness: float = 0.002,
          strips: int = 5, color=(0.95, 0.95, 0.93), **kwargs) -> Sheet:
    """An upright :class:`Sheet` whose top edge's centre is at ``top`` (world m), its plane turned ``yaw`` about z
    (0: the sheet faces x)."""
    h = height / strips
    c, s = math.cos(yaw), math.sin(yaw)
    half_xy = np.abs(np.array([[c, -s], [s, c]])) @ np.array([thickness / 2, width / 2])
    return Sheet(name=name, pos=(float(top[0]), float(top[1]), float(top[2] - height / 2)),
                 size=(float(2 * half_xy[0]), float(2 * half_xy[1]), float(height)), color=color, yaw=yaw,
                 top_centre=(float(top[0]), float(top[1]), float(top[2] - h / 2)), width=width, height=height,
                 thickness=thickness, strips=strips, **kwargs)


@dataclass(frozen=True)
class SheetState(ObjectState):
    """An ObjectState for a :class:`Sheet` (``pos`` = mean strip centre, ``quat_xyzw`` = the top strip's), plus every
    strip's pose."""

    centres: np.ndarray | None = None  # (strips, 3), top to bottom
    rotations: np.ndarray | None = None  # (strips, 3, 3): columns are each strip's normal (x), width (y), up (z) axes
    strip_h: float = 0.0
    width: float = 0.0
    thickness: float = 0.0

    def corners(self) -> dict[str, np.ndarray]:
        """The sheet's four corners; ``left`` is its +y side (the robot's left when it faces +x)."""
        h, w = self.strip_h / 2, self.width / 2
        top, bot, rt, rb = self.centres[0], self.centres[-1], self.rotations[0], self.rotations[-1]
        return {"top_left": top + rt @ [0, w, h], "top_right": top + rt @ [0, -w, h],
                "bottom_left": bot + rb @ [0, w, -h], "bottom_right": bot + rb @ [0, -w, -h]}

    def normals(self) -> np.ndarray:
        return self.rotations[:, :, 0]

    def distance(self, point) -> float:
        """Distance (m) from ``point`` to the nearest strip (each a thin box)."""
        p = np.asarray(point, float)
        half = np.array([self.thickness / 2, self.width / 2, self.strip_h / 2])
        best = float("inf")
        for c, r in zip(self.centres, self.rotations, strict=True):
            local = r.T @ (p - c)
            best = min(best, float(np.linalg.norm(local - np.clip(local, -half, half))))
        return best


def quat_matrix(q_xyzw) -> np.ndarray:
    """Rotation matrix of an (x, y, z, w) quaternion."""
    x, y, z, w = (float(v) for v in q_xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


ObjectSpec = Box | Cylinder | Sphere | Asset | Compound | Sheet


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
    forces: Callable[[dict[str, ObjectState], dict], dict] | None = None
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


def _no_setup(rng, trial) -> dict:
    return {}


def _no_events(t, state, world) -> None:
    return None


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
        if not isinstance(obj, Box | Cylinder | Sphere | Asset | Compound | Sheet):
            raise TypeError(f"{path.name}: OBJECTS holds {obj!r}; use Box, Cylinder, Sphere, Asset, Compound or Sheet")
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
            ("setup", False),
            ("events", False),
            ("milestones", True),
            ("success", True),
            ("demo", True),
            ("placements", False),
            ("failure", False),
            ("hold", False),
            ("forces", False),
        )
    }
    # A task without hidden state or scripted events (no memory element) may leave these out.
    hooks["setup"] = hooks["setup"] or _no_setup
    hooks["events"] = hooks["events"] or _no_events
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
