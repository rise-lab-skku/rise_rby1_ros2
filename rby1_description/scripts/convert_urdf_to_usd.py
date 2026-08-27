#!/home/jiyou/isaac_ws/env_isaaclab/bin/python
"""Convert an RBY1 URDF to USD with Isaac Lab's Isaac Sim environment.

Run this script with the project's Isaac Lab virtual environment:

    /home/jiyou/isaac_ws/env_isaaclab/bin/python \
        rby1_description/scripts/convert_urdf_to_usd.py

The expected environment currently provides Isaac Sim 5.1 and Isaac Lab.
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator

SCRIPT_DIR = Path(__file__).resolve().parent
DESCRIPTION_DIR = SCRIPT_DIR.parent
DEFAULT_URDF = DESCRIPTION_DIR / "urdf" / "rby1a" / "model_v1_2.urdf"
DEFAULT_USD = DESCRIPTION_DIR / "usd" / "rby1a" / "model_v1_2.usd"
ISAACLAB_ENV = Path("/home/jiyou/isaac_ws/env_isaaclab")


def require_isaaclab_environment() -> None:
    """Reject execution with a Python outside the project Isaac Lab venv."""

    active_environment = Path(sys.prefix).resolve()
    expected_environment = ISAACLAB_ENV.resolve()
    if active_environment != expected_environment:
        command = ISAACLAB_ENV / "bin" / "python"
        raise RuntimeError(
            "This script must use the Isaac Lab virtual environment. Run:\n"
            f"  {command} {Path(__file__).resolve()}"
        )


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


def _resolve_package_mesh(mesh_uri: str) -> Path | None:
    """Resolve meshes from this description package without requiring ROS setup."""

    package_prefix = "package://rby1_description/"
    if mesh_uri.startswith(package_prefix):
        return DESCRIPTION_DIR / mesh_uri.removeprefix(package_prefix)
    if mesh_uri.startswith("file://"):
        return Path(mesh_uri.removeprefix("file://"))
    path = Path(mesh_uri)
    return path if path.is_absolute() else None


def _usd_safe_stem(stem: str) -> str:
    """Return a mesh stem that Isaac Sim 5.1 can use as a USD prim name."""

    safe_stem = re.sub(r"[^A-Za-z0-9_]", "_", stem)
    if not safe_stem or safe_stem[0].isdigit():
        safe_stem = f"mesh_{safe_stem}"
    return safe_stem


def _normalize_collada_accessors(mesh_path: Path) -> tuple[ET.ElementTree, int]:
    """Correct scalar accessor counts emitted by some online DAE converters."""

    tree = ET.parse(mesh_path)
    root = tree.getroot()
    namespace = root.tag.removeprefix("{").split("}", maxsplit=1)[0]
    tag = lambda name: f"{{{namespace}}}{name}"  # noqa: E731
    float_arrays = {
        f"#{array.get('id')}": array for array in root.iter(tag("float_array"))
    }
    correction_count = 0
    for accessor in root.iter(tag("accessor")):
        array = float_arrays.get(accessor.get("source", ""))
        if array is None:
            continue
        stride = int(accessor.get("stride", "1"))
        declared_count = int(accessor.get("count", "0"))
        actual_scalar_count = len((array.text or "").split())
        if (
            stride > 1
            and declared_count * stride > actual_scalar_count
            and actual_scalar_count % stride == 0
        ):
            accessor.set("count", str(actual_scalar_count // stride))
            correction_count += 1
    if namespace:
        ET.register_namespace("", namespace)
    return tree, correction_count


def _normalize_assimp_obj_faces(source: Path, destination: Path) -> None:
    """Remove dangling texture-coordinate separators from Assimp OBJ faces."""

    with source.open(encoding="utf-8") as source_file, destination.open(
        "w", encoding="utf-8"
    ) as destination_file:
        for line in source_file:
            if line.startswith("f "):
                line = re.sub(r"/(?=\s|$)", "", line)
            destination_file.write(line)


@contextmanager
def prepare_urdf_for_import(urdf_path: Path) -> Iterator[Path]:
    """Resolve package meshes and stage files needing Isaac 5.1 fixes.

    Isaac Sim 5.1 derives visual prim names from mesh filenames. Versioned
    Collada names such as ``LINK_19_v1.1.dae`` therefore cause an invalid USD
    path. A temporary URDF points those meshes at safely named copies while the
    original URDF and mesh files remain untouched. All other package URIs are
    rewritten to absolute paths so conversion does not depend on a sourced ROS
    environment or ``ROS_PACKAGE_PATH``.
    """

    tree = ET.parse(urdf_path)
    rewrites: dict[Path, Path] = {}
    collada_cache: dict[Path, tuple[ET.ElementTree, int]] = {}
    renamed_mesh_count = 0
    repaired_accessor_count = 0
    resolved_package_uri_count = 0

    with tempfile.TemporaryDirectory(prefix="rby1_urdf_import_") as temp_dir:
        temp_path = Path(temp_dir)
        for mesh in tree.getroot().iter("mesh"):
            mesh_uri = mesh.get("filename")
            if not mesh_uri:
                continue
            source = _resolve_package_mesh(mesh_uri)
            if source is None:
                continue
            safe_stem = _usd_safe_stem(source.stem)
            source = source.resolve()
            if not source.is_file():
                raise FileNotFoundError(f"URDF mesh does not exist: {source}")
            collada_tree = None
            correction_count = 0
            if source.suffix.lower() == ".dae":
                if source not in collada_cache:
                    collada_cache[source] = _normalize_collada_accessors(source)
                collada_tree, correction_count = collada_cache[source]
            if safe_stem == source.stem and correction_count == 0:
                if mesh_uri.startswith("package://rby1_description/"):
                    mesh.set("filename", str(source))
                    resolved_package_uri_count += 1
                continue
            staged_mesh = rewrites.get(source)
            if staged_mesh is None:
                if correction_count:
                    assimp = shutil.which("assimp")
                    if assimp is None:
                        raise RuntimeError(
                            "A malformed Collada mesh needs repair, but the "
                            "'assimp' executable is not available"
                        )
                    repaired_dae = temp_path / f"{safe_stem}_repaired.dae"
                    staged_mesh = temp_path / f"{safe_stem}.obj"
                    raw_obj = temp_path / f"{safe_stem}_assimp.obj"
                    collada_tree.write(
                        repaired_dae, encoding="utf-8", xml_declaration=True
                    )
                    subprocess.run(
                        [
                            assimp,
                            "export",
                            str(repaired_dae),
                            str(raw_obj),
                            "-fobjnomtl",
                        ],
                        check=True,
                        stdout=subprocess.DEVNULL,
                    )
                    _normalize_assimp_obj_faces(raw_obj, staged_mesh)
                    repaired_accessor_count += correction_count
                else:
                    staged_mesh = temp_path / f"{safe_stem}{source.suffix}"
                    shutil.copy2(source, staged_mesh)
                if safe_stem != source.stem:
                    renamed_mesh_count += 1
                rewrites[source] = staged_mesh
            mesh.set("filename", str(staged_mesh))

        if not rewrites and resolved_package_uri_count == 0:
            yield urdf_path
            return

        staged_urdf = temp_path / urdf_path.name
        tree.write(staged_urdf, encoding="utf-8", xml_declaration=True)
        print(
            "Isaac Sim 5.1 compatibility: "
            f"staged {len(rewrites)} mesh(es), renamed {renamed_mesh_count}, "
            f"repaired {repaired_accessor_count} Collada accessor(s), "
            f"resolved {resolved_package_uri_count} package URI(s).",
            flush=True,
        )
        yield staged_urdf


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
            raise RuntimeError(f"Unexpected PhysX mimic gearing for {name}: {gearing}")
        if len(targets) != 1 or targets[0].name != source_name:
            raise RuntimeError(
                f"Unexpected mimic reference for {name}: {list(targets)}"
            )

    return len(prims), len(joints), len(articulations)


def main() -> None:
    require_isaaclab_environment()
    from isaaclab.app import AppLauncher

    args = parse_args()
    urdf_path = args.urdf.expanduser().resolve()
    usd_path = args.usd.expanduser().resolve()

    if not urdf_path.is_file():
        raise FileNotFoundError(f"URDF does not exist: {urdf_path}")
    if usd_path.suffix.lower() not in {".usd", ".usda", ".usdc"}:
        raise ValueError(f"Output must be a USD file: {usd_path}")
    usd_path.parent.mkdir(parents=True, exist_ok=True)

    app_launcher = AppLauncher(headless=True)
    app = app_launcher.app
    try:
        import omni.kit.commands
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.asset.importer.urdf")
        app.update()

        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        if not status:
            raise RuntimeError("Failed to create the URDF import configuration")

        import_config.merge_fixed_joints = args.merge_fixed_joints
        import_config.convex_decomp = args.convex_decomp
        import_config.import_inertia_tensor = True
        import_config.fix_base = args.fix_base
        import_config.collision_from_visuals = args.collision_from_visuals
        import_config.parse_mimic = True

        with prepare_urdf_for_import(urdf_path) as import_urdf_path:
            status, articulation_root = omni.kit.commands.execute(
                "URDFParseAndImportFile",
                urdf_path=str(import_urdf_path),
                import_config=import_config,
                dest_path=str(usd_path),
                get_articulation_root=True,
            )
        app.update()
        if not status:
            raise RuntimeError(f"URDF import failed: {articulation_root}")

        if not usd_path.is_file():
            raise RuntimeError(f"Importer did not create the expected USD: {usd_path}")
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
