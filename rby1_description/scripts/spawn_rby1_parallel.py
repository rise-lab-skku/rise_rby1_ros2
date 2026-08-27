"""Spawn rby1_parallel.usd with configured initial pose and joint drives.

Run with the Isaac Sim / Isaac Lab virtual environment:

    /home/jiyou/isaac_ws/env_isaaclab/bin/python \
        rby1_description/scripts/spawn_rby1_parallel.py
"""

from __future__ import annotations

import argparse
import math
import sys
import traceback
from pathlib import Path

import yaml

from isaaclab.app import AppLauncher


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = SCRIPT_DIR.parent / "config" / "rby1_parallel_spawn.yaml"

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    "--config",
    type=Path,
    default=DEFAULT_CONFIG_PATH,
    help="Spawn configuration YAML path.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
import omni.usd
from pxr import PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics


def load_config(path: Path) -> tuple[Path, dict]:
    """Load YAML and resolve its USD path relative to the YAML file."""

    config_path = path.expanduser().resolve()
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    usd_path = Path(config["usd_path"]).expanduser()
    if not usd_path.is_absolute():
        usd_path = (config_path.parent / usd_path).resolve()
    if not usd_path.is_file():
        raise FileNotFoundError(f"Robot USD does not exist: {usd_path}")
    return usd_path, config


def is_mimic_joint(prim: Usd.Prim) -> bool:
    """Return whether a joint is a PhysX mimic follower."""

    return any(
        schema.startswith("PhysxMimicJointAPI")
        for schema in prim.GetAppliedSchemas()
    )


def float_tuple(values: list | tuple, length: int, name: str) -> tuple[float, ...]:
    """Validate and convert a YAML vector to a tuple of floats."""

    if not isinstance(values, (list, tuple)) or len(values) != length:
        raise ValueError(f"{name} must contain exactly {length} values: {values}")
    return tuple(float(value) for value in values)


def spawn_scene(config: dict) -> tuple[bool, bool]:
    """Spawn the optional ground plane and static table cuboid."""

    scene_config = config.get("scene", {})
    ground_config = scene_config.get("ground", {})
    table_config = scene_config.get("table", {})
    ground_spawned = bool(ground_config.get("enabled", True))
    table_spawned = bool(table_config.get("enabled", True))

    if ground_spawned:
        ground_cfg = sim_utils.GroundPlaneCfg(
            size=float_tuple(
                ground_config.get("size", [10.0, 10.0]), 2, "scene.ground.size"
            ),
            color=float_tuple(
                ground_config.get("color", [0.25, 0.25, 0.25]),
                3,
                "scene.ground.color",
            ),
        )
        ground_cfg.func(
            str(ground_config.get("prim_path", "/World/ground")), ground_cfg
        )

    if table_spawned:
        table_cfg = sim_utils.CuboidCfg(
            size=float_tuple(
                table_config.get("size", [0.6, 1.2, 0.75]),
                3,
                "scene.table.size",
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=float_tuple(
                    table_config.get("color", [0.45, 0.25, 0.10]),
                    3,
                    "scene.table.color",
                )
            ),
        )
        table_cfg.func(
            str(table_config.get("prim_path", "/World/table")),
            table_cfg,
            translation=float_tuple(
                table_config.get("position", [0.8, 0.0, 0.375]),
                3,
                "scene.table.position",
            ),
        )

    return ground_spawned, table_spawned


def spawn_robot_reference(
    stage: Usd.Stage, robot_path: str, usd_path: Path, config: dict
) -> Usd.Prim:
    """Reference an explicit robot prim without requiring a USD default prim."""

    source_prim_path = str(config.get("usd_prim_path", "/RBY1_A_v1_2"))
    source_stage = Usd.Stage.Open(str(usd_path))
    if source_stage is None:
        raise RuntimeError(f"Failed to open robot USD: {usd_path}")
    if not source_stage.GetPrimAtPath(source_prim_path).IsValid():
        raise ValueError(
            f"Robot source prim does not exist in USD: {source_prim_path}"
        )

    robot = UsdGeom.Xform.Define(stage, robot_path).GetPrim()
    reference = Sdf.Reference(
        assetPath=str(usd_path), primPath=Sdf.Path(source_prim_path)
    )
    if not robot.GetReferences().AddReference(reference):
        raise RuntimeError(
            f"Failed to reference {usd_path}<{source_prim_path}> at {robot_path}"
        )
    return robot


def configure_joints(stage: Usd.Stage, robot_path: str, config: dict) -> tuple[int, int]:
    """Apply initial states/targets and drive gains to movable joints."""

    robot = stage.GetPrimAtPath(robot_path)
    if not robot.IsValid():
        raise RuntimeError(f"Spawned robot prim is invalid: {robot_path}")

    initial_positions = {
        str(name): float(value)
        for name, value in config.get("initial_joint_positions", {}).items()
    }
    drive_config = config["joint_drive"]
    default_drive = drive_config["default"]
    overrides = drive_config.get("overrides", {})

    configured_drives = 0
    initialized_joints = 0
    found_joint_names: set[str] = set()

    for prim in Usd.PrimRange(robot):
        is_revolute = prim.IsA(UsdPhysics.RevoluteJoint)
        is_prismatic = prim.IsA(UsdPhysics.PrismaticJoint)
        if not (is_revolute or is_prismatic):
            continue

        joint_name = prim.GetName()
        found_joint_names.add(joint_name)
        drive_instance = "angular" if is_revolute else "linear"

        # A mimic follower is constrained by its source joint. Giving it a
        # second independent drive would over-constrain the gripper.
        mimic = is_mimic_joint(prim)
        if not mimic:
            values = dict(default_drive)
            values.update(overrides.get(joint_name, {}))
            drive = UsdPhysics.DriveAPI.Get(prim, drive_instance)
            if not drive:
                drive = UsdPhysics.DriveAPI.Apply(prim, drive_instance)
            drive.CreateMaxForceAttr().Set(float(values["max_force"]))
            drive.CreateStiffnessAttr().Set(float(values["stiffness"]))
            drive.CreateDampingAttr().Set(float(values["damping"]))
            configured_drives += 1

        if joint_name not in initial_positions:
            continue

        position = initial_positions[joint_name]
        authored_position = math.degrees(position) if is_revolute else position

        if prim.HasAPI(PhysxSchema.JointStateAPI, drive_instance):
            state = PhysxSchema.JointStateAPI.Get(prim, drive_instance)
        else:
            state = PhysxSchema.JointStateAPI.Apply(prim, drive_instance)
        state.CreatePositionAttr().Set(authored_position)
        state.CreateVelocityAttr().Set(0.0)

        if not mimic:
            drive = UsdPhysics.DriveAPI.Get(prim, drive_instance)
            drive.CreateTargetPositionAttr().Set(authored_position)
            drive.CreateTargetVelocityAttr().Set(0.0)
        initialized_joints += 1

    unknown_initial_joints = sorted(initial_positions.keys() - found_joint_names)
    unknown_drive_joints = sorted(overrides.keys() - found_joint_names)
    if unknown_initial_joints:
        raise ValueError(f"Unknown initial-position joints: {unknown_initial_joints}")
    if unknown_drive_joints:
        raise ValueError(f"Unknown drive override joints: {unknown_drive_joints}")

    return initialized_joints, configured_drives


def main() -> None:
    usd_path, config = load_config(args_cli.config)

    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(dt=0.01, device=args_cli.device, use_fabric=False)
    )
    stage = omni.usd.get_context().get_stage()

    ground_spawned, table_spawned = spawn_scene(config)
    robot_path = str(config.get("prim_path", "/World/rby1"))
    robot = spawn_robot_reference(stage, robot_path, usd_path, config)
    camera_paths = [
        str(prim.GetPath())
        for prim in Usd.PrimRange(robot)
        if prim.IsA(UsdGeom.Camera)
    ]

    initialized, configured = configure_joints(stage, robot_path, config)
    print(f"[INFO] Spawned robot: {robot_path}", flush=True)
    print(f"[INFO] Spawned ground: {ground_spawned}", flush=True)
    print(f"[INFO] Spawned table: {table_spawned}", flush=True)
    print(f"[INFO] Spawned cameras: {camera_paths}", flush=True)
    print(f"[INFO] USD: {usd_path}", flush=True)
    print(f"[INFO] Authored initial joint states: {initialized}", flush=True)
    print(f"[INFO] Configured independent drives: {configured}", flush=True)

    sim.reset()
    print("[INFO] Physics initialized. Robot is ready.", flush=True)

    while simulation_app.is_running():
        sim.step()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        print("[ERROR] Failed to spawn RBY1:", file=sys.stderr, flush=True)
        traceback.print_exc()
        raise
    finally:
        simulation_app.close()
