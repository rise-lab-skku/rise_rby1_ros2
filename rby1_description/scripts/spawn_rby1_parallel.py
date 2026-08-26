"""Spawn rby1_parallel.usd with configured initial pose and joint drives.

Run with the Isaac Sim / Isaac Lab virtual environment:

    /home/jiyou/isaac_ws/env_isaaclab/bin/python \
        rby1_description/scripts/spawn_rby1_parallel.py
"""

from __future__ import annotations

import argparse
import math
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
from pxr import PhysxSchema, Usd, UsdPhysics


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

    robot_path = str(config.get("prim_path", "/World/rby1"))
    spawn_cfg = sim_utils.UsdFileCfg(usd_path=str(usd_path))
    spawn_cfg.func(robot_path, spawn_cfg)

    initialized, configured = configure_joints(stage, robot_path, config)
    print(f"[INFO] Spawned robot: {robot_path}", flush=True)
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
    finally:
        simulation_app.close()
