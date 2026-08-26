#!/usr/bin/env python3
"""Convert an RBY1 URDF to USD with the Isaac Sim URDF importer.

Run this script with Isaac Sim's Python interpreter, for example:

    /home/jiyou/.local/share/ov/pkg/isaac-sim-4.1.0/python.sh \
        rby1_description/scripts/convert_urdf_to_usd.py

Any arguments after this script path are handled by this script, not Isaac Sim.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from isaacsim import SimulationApp


SCRIPT_DIR = Path(__file__).resolve().parent
DESCRIPTION_DIR = SCRIPT_DIR.parent
DEFAULT_URDF = DESCRIPTION_DIR / "urdf" / "rby1a" / "model_v1_2.urdf"
DEFAULT_USD = DESCRIPTION_DIR / "usd" / "rby1a" / "model_v1_2.usd"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--urdf",
        type=Path,
        default=DEFAULT_URDF,
        help=f"Input URDF path (default: {DEFAULT_URDF})",
    )
    parser.add_argument(
        "--usd",
        type=Path,
        default=DEFAULT_USD,
        help=f"Output USD path (default: {DEFAULT_USD})",
    )
    parser.add_argument(
        "--fix-base",
        action="store_true",
        help="Fix the root link to the world instead of importing a floating base.",
    )
    parser.add_argument(
        "--merge-fixed-joints",
        action="store_true",
        help="Merge links connected by fixed joints during import.",
    )
    parser.add_argument(
        "--convex-decomp",
        action="store_true",
        help="Use convex decomposition for collision meshes.",
    )
    parser.add_argument(
        "--collision-from-visuals",
        action="store_true",
        help="Generate collisions from visual geometry when needed.",
    )
    return parser.parse_args()


def validate_usd(usd_path: Path) -> tuple[int, int, int]:
    """Open the generated stage and validate its articulation and gripper."""

    from pxr import PhysxSchema, Usd, UsdPhysics

    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise RuntimeError(f"Failed to open generated USD: {usd_path}")

    prims = list(stage.Traverse())
    joints = [
        prim
        for prim in prims
        if prim.IsA(UsdPhysics.RevoluteJoint)
        or prim.IsA(UsdPhysics.PrismaticJoint)
        or prim.IsA(UsdPhysics.FixedJoint)
    ]
    articulations = [
        prim for prim in prims if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if not articulations:
        raise RuntimeError("Generated USD has no articulation root")

    prims_by_name = {prim.GetName(): prim for prim in prims}
    finger_joint_names = {
        "gripper_finger_r1_joint",
        "gripper_finger_r2_joint",
        "gripper_finger_l1_joint",
        "gripper_finger_l2_joint",
    }
    present_finger_joints = finger_joint_names & prims_by_name.keys()
    if present_finger_joints and present_finger_joints != finger_joint_names:
        missing = sorted(finger_joint_names - present_finger_joints)
        raise RuntimeError(f"Generated USD is missing finger joints: {missing}")

    follower_to_source = {
        "gripper_finger_r2_joint": "gripper_finger_r1_joint",
        "gripper_finger_l2_joint": "gripper_finger_l1_joint",
    }
    for name in finger_joint_names:
        prim = prims_by_name.get(name)
        if prim is None:
            continue
        if not prim.IsA(UsdPhysics.PrismaticJoint):
            raise RuntimeError(f"Finger joint is not prismatic in USD: {name}")

        mimic_schemas = [
            schema
            for schema in prim.GetAppliedSchemas()
            if schema.startswith("PhysxMimicJointAPI:")
        ]
        source_name = follower_to_source.get(name)
        if source_name is None:
            if mimic_schemas:
                raise RuntimeError(f"Finger source unexpectedly mimics a joint: {name}")
            continue
        if len(mimic_schemas) != 1:
            raise RuntimeError(f"Finger follower has no single mimic API: {name}")

        instance_name = mimic_schemas[0].split(":", maxsplit=1)[1]
        mimic = PhysxSchema.PhysxMimicJointAPI(prim, instance_name)
        gearing = mimic.GetGearingAttr().Get()
        targets = mimic.GetReferenceJointRel().GetTargets()
        if gearing != 1.0:
            raise RuntimeError(
                f"Unexpected PhysX mimic gearing for {name}: {gearing}"
            )
        if len(targets) != 1 or targets[0].name != source_name:
            raise RuntimeError(
                f"Unexpected mimic reference for {name}: {list(targets)}"
            )

    return len(prims), len(joints), len(articulations)


def main() -> None:
    args = parse_args()
    urdf_path = args.urdf.expanduser().resolve()
    usd_path = args.usd.expanduser().resolve()

    if not urdf_path.is_file():
        raise FileNotFoundError(f"URDF does not exist: {urdf_path}")
    if usd_path.suffix.lower() not in {".usd", ".usda", ".usdc"}:
        raise ValueError(f"Output must be a USD file: {usd_path}")
    usd_path.parent.mkdir(parents=True, exist_ok=True)

    app = SimulationApp({"headless": True})
    try:
        import omni.kit.commands

        status, import_config = omni.kit.commands.execute(
            "URDFCreateImportConfig"
        )
        if not status:
            raise RuntimeError("Failed to create the URDF import configuration")

        import_config.merge_fixed_joints = args.merge_fixed_joints
        import_config.convex_decomp = args.convex_decomp
        import_config.import_inertia_tensor = True
        import_config.fix_base = args.fix_base
        import_config.collision_from_visuals = args.collision_from_visuals
        import_config.parse_mimic = True

        status, articulation_root = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=str(urdf_path),
            import_config=import_config,
            dest_path=str(usd_path),
            get_articulation_root=True,
        )
        app.update()
        if not status:
            raise RuntimeError(f"URDF import failed: {articulation_root}")

        if not usd_path.is_file():
            raise RuntimeError(
                f"Importer did not create the expected USD: {usd_path}"
            )
        prim_count, joint_count, articulation_count = validate_usd(usd_path)
        print(f"Created USD: {usd_path}", flush=True)
        print(f"Articulation root: {articulation_root}", flush=True)
        print(
            "Validated stage: "
            f"prims={prim_count}, joints={joint_count}, "
            f"articulations={articulation_count}",
            flush=True,
        )
    finally:
        app.close()


if __name__ == "__main__":
    main()
