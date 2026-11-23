"""Memory task objects as Arena assets, and the Arena environment around them. Import after app start."""

from __future__ import annotations

import os

import isaaclab.sim as sim_utils
from isaaclab.sim import schemas
from isaaclab.sim.spawners.shapes import spawn_cuboid
from isaaclab.sim.utils import clone
from isaaclab_arena.assets.object import Object
from isaaclab_arena.assets.object_type import ObjectType
from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
from isaaclab_arena.scene.scene import Scene
from isaaclab_arena.utils.pose import Pose
from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg, PhysxRigidBodyPropertiesCfg
from yam_arena.scene import _contact_props, _grippy, _yaw_xyzw, env_cfg_callback, task_object, task_room
from yam_arena.station import linear_rgb

from rta.spec import Compound, MemoryTask


def compound_spawner(parts):
    """A cuboid spawner that adds ``parts`` as more cuboids of the same body, with the root's collision and
    materials. Children of ``{prim_path}/geometry`` share the root's rigid body, so PhysX collides them as one
    compound shape (a free compound) or as one static collider (a fixed one)."""

    @clone
    def spawn(prim_path: str, cfg: sim_utils.CuboidCfg, *args, **kwargs):
        from pxr import Gf, UsdGeom, UsdShade

        prim = spawn_cuboid.__wrapped__(prim_path, cfg, *args, **kwargs)
        stage = prim.GetStage()
        geom = f"{prim_path}/geometry"
        mesh = stage.GetPrimAtPath(f"{geom}/mesh")
        bindings = []
        for purpose in (UsdShade.Tokens.allPurpose, "physics"):
            material = UsdShade.MaterialBindingAPI(mesh).GetDirectBinding(purpose).GetMaterial()
            if material:
                bindings.append((purpose, material))
        for i, part in enumerate(parts):
            path = f"{geom}/part{i}"
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(1.0)
            xf = UsdGeom.Xformable(cube.GetPrim())
            xf.AddTranslateOp().Set(Gf.Vec3d(*part.offset))
            xf.AddScaleOp().Set(Gf.Vec3f(*part.size))
            if cfg.collision_props is not None:
                schemas.define_collision_properties(path, cfg.collision_props, stage=stage)
            api = UsdShade.MaterialBindingAPI.Apply(cube.GetPrim())
            for purpose, material in bindings:
                api.Bind(material, materialPurpose=purpose)
        return prim

    return spawn


def compound_object(spec: Compound) -> Object:
    look = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.4)
    if spec.free:
        body = dict(
            rigid_props=PhysxRigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=spec.mass),
            collision_props=_contact_props(),
        )
    else:
        body = dict(collision_props=PhysxCollisionPropertiesCfg())
    cfg = sim_utils.CuboidCfg(
        func=compound_spawner(spec.parts), size=tuple(spec.size), physics_material=_grippy(), visual_material=look, **body
    )
    return Object(
        name=spec.name,
        object_type=ObjectType.RIGID if spec.free else ObjectType.BASE,
        spawner_cfg=cfg,
        initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=_yaw_xyzw(spec.yaw)),
    )


# Optional PhysX settings for free bodies, applied by ``apply_solver_iters`` only when RTA_SOLVER_ITERS=1.
# Every task verification so far (2026-11-05, 16 bundles) ran at the PhysX defaults (4/1 iterations, no damping):
# the values set on the spawner never reached the bodies (the Arena's scene composition drops them), so turning
# them on is a physics change that needs re-verification. Motivation: thin objects in snug pockets and bodies
# pinned by the fingers rattled at 4-7 cm/s and never counted as resting; a tube in a square pocket precessed
# indefinitely (J4 j04d, since fixed by geometry).
SOLVER_ITERS = dict(
    solver_position_iteration_count=16, solver_velocity_iteration_count=4,
    # mild air/contact dissipation: with zero damping a tube standing in a pocket precessed around the wall
    # indefinitely (1.2 cm orbit at 1.4 Hz, J4 j04d) and never counted as resting
    linear_damping=0.5, angular_damping=1.0,
)


def memory_object(spec) -> Object:
    return compound_object(spec) if isinstance(spec, Compound) else task_object(spec)


# PhysX GPU pair buffers are allocated in pinned host memory at the Isaac Lab defaults (2**25 aggregate pairs =
# 268 MB), which fails on a 24 GB consumer GPU ("PxgPinnedMemoryAllocate failed ... lost found pairs") as soon as a scene has
# aggregates (our compound bodies). One env with a dozen bodies needs a tiny fraction of that.
GPU_PAIR_CAPACITY = 2**18


def apply_solver_iters(env_cfg, free_names) -> list[str]:
    """Write ``SOLVER_ITERS`` onto the composed scene cfg of every free object and return the names written.
    Values set on a task object's spawner do not survive the Arena's scene composition (``Scene.get_scene_cfg()``
    hands Isaac Lab a rigid_props with every field None — probed 2026-11-05), so this runs in the env-cfg
    callback, on the cfg Isaac Lab actually spawns from. The trace header's ``reset.rigid_props`` shows the result."""
    done = []
    for name in free_names:
        spawn = getattr(getattr(env_cfg.scene, name, None), "spawn", None)
        if spawn is None:
            raise RuntimeError(f"{name}: no spawn cfg on the composed scene cfg")
        if getattr(spawn, "rigid_props", None) is None:  # library asset (UsdFileCfg): no rigid_props yet
            spawn.rigid_props = PhysxRigidBodyPropertiesCfg()
        for k, v in SOLVER_ITERS.items():
            setattr(spawn.rigid_props, k, v)
        done.append(name)
    return done


def memory_env_cfg_callback(control_hz: float | None = None, free_names: tuple[str, ...] = ()):
    configure = env_cfg_callback(control_hz)

    def configure_memory(env_cfg):
        env_cfg = configure(env_cfg)
        if os.environ.get("RTA_SOLVER_ITERS") == "1":
            apply_solver_iters(env_cfg, free_names)
        from isaaclab_physx.physics import PhysxCfg

        physics = env_cfg.sim.physics
        if physics is None:
            physics = env_cfg.sim.physics = PhysxCfg()
        if isinstance(physics, PhysxCfg):
            for attr in ("gpu_found_lost_pairs_capacity", "gpu_found_lost_aggregate_pairs_capacity", "gpu_total_aggregate_pairs_capacity"):
                if hasattr(physics, attr):
                    setattr(physics, attr, GPU_PAIR_CAPACITY)
        return env_cfg

    return configure_memory


def build_memory_environment(embodiment, task: MemoryTask, control_hz: float | None = None):
    return IsaacLabArenaEnvironment(
        name=f"yam_memory_{task.name}",
        scene=Scene([*task_room(task.scene, embodiment.rig.top_cam), *(memory_object(o) for o in task.objects)]),
        embodiment=embodiment,
        env_cfg_callback=memory_env_cfg_callback(control_hz, tuple(o.name for o in task.objects if o.free)),
    )


def make_memory_env(embodiment, task: MemoryTask, *, device: str = "cuda:0", control_hz: float | None = None):
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg

    cfg = ArenaEnvBuilderCfg(num_envs=1, solve_relations=False, device=device)
    return ArenaEnvBuilder(build_memory_environment(embodiment, task, control_hz), cfg).make_registered()
