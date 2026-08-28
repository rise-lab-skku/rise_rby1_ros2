#!/home/jiyou/isaac_ws/env_isaaclab/bin/python
"""Convert an RBY1 URDF to USD with Isaac Lab's Isaac Sim environment.

Run this script with the project's Isaac Lab virtual environment:

    /home/jiyou/isaac_ws/env_isaaclab/bin/python \
        rby1_description/scripts/convert_urdf_to_usd.py

The expected environment currently provides Isaac Sim 5.1 and Isaac Lab.
"""

from __future__ import annotations

import argparse
import gc
import json
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

SCRIPT_DIR = Path(__file__).resolve().parent
DESCRIPTION_DIR = SCRIPT_DIR.parent
DEFAULT_URDF = DESCRIPTION_DIR / "urdf" / "rby1a" / "model_v1_2.urdf"
DEFAULT_USD = DESCRIPTION_DIR / "usd" / "rby1a" / "model_v1_2.usd"
ISAACLAB_ENV = Path("/home/jiyou/isaac_ws/env_isaaclab")
PRESERVED_CAMERAS = {
    "cam_right": "gripper_right",
    "cam_left": "gripper_left",
    "cam_head": "link_head_2",
}


@dataclass(frozen=True)
class PreservedUsdContent:
    """USD specs that must survive regeneration from URDF."""

    layer_path: Path
    camera_paths: dict[str, str]
    has_render_tree: bool


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
    parser.add_argument(
        "--no-preserve-cameras",
        action="store_true",
        help=(
            "Do not restore existing cam_right, cam_left, cam_head, or /Render "
            "content after regenerating an existing USD."
        ),
    )
    parser.add_argument(
        "--_extract-preserved-to",
        type=Path,
        default=None,
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--_preserved-layer",
        type=Path,
        default=None,
        help=argparse.SUPPRESS,
    )
    parser.add_argument("--_import-only", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument(
        "--_postprocess-only", action="store_true", help=argparse.SUPPRESS
    )
    parser.add_argument("--_repair-only", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--_restore-only", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument(
        "--_expect-preserved-cameras", action="store_true", help=argparse.SUPPRESS
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


def validate_usd(usd_path: Path, stage: object | None = None) -> tuple[int, int, int]:
    """Open the generated stage and validate its articulation and gripper."""

    from pxr import PhysxSchema, Usd, UsdPhysics

    stage = stage or Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise RuntimeError(f"Failed to open generated USD: {usd_path}")
    if not stage.GetDefaultPrim().IsValid():
        raise RuntimeError("Generated USD has no default prim")

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


def preserve_existing_usd_content(
    usd_path: Path, backup_path: Path | None = None
) -> PreservedUsdContent | None:
    """Copy cameras and render settings out of an existing output USD.

    The copy lives in an anonymous in-memory layer, so the URDF importer can
    safely replace ``usd_path`` before the specs are restored.
    """

    if not usd_path.is_file():
        return None

    from pxr import Sdf, Usd

    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise RuntimeError(f"Failed to open existing USD for preservation: {usd_path}")

    source_layer = stage.GetRootLayer()
    flattened_layer = None
    if backup_path is None:
        backup_dir = Path(tempfile.mkdtemp(prefix="rby1_preserved_usd_"))
        backup_path = backup_dir / "preserved.usda"
    else:
        backup_path = backup_path.expanduser().resolve()
        backup_path.parent.mkdir(parents=True, exist_ok=True)
        backup_dir = backup_path.parent
    backup_layer = Sdf.Layer.CreateNew(str(backup_path))
    camera_paths: dict[str, str] = {}

    prims_by_name: dict[str, list[object]] = {}
    for prim in stage.TraverseAll():
        prims_by_name.setdefault(prim.GetName(), []).append(prim)

    for camera_name, parent_name in PRESERVED_CAMERAS.items():
        candidates = [
            prim
            for prim in prims_by_name.get(camera_name, [])
            if prim.GetParent().GetName() == parent_name
        ]
        if len(candidates) > 1:
            paths = [str(prim.GetPath()) for prim in candidates]
            raise RuntimeError(
                f"Existing USD has multiple {camera_name} cameras under "
                f"{parent_name}: {paths}"
            )
        if not candidates:
            continue

        camera_path = candidates[0].GetPath()
        if source_layer.GetPrimAtPath(camera_path) is None:
            if flattened_layer is None:
                flattened_layer = stage.Flatten()
            copy_source = flattened_layer
        else:
            copy_source = source_layer
        parent_spec = Sdf.CreatePrimInLayer(
            backup_layer, camera_path.GetParentPath()
        )
        parent_spec.specifier = Sdf.SpecifierOver
        if not Sdf.CopySpec(copy_source, camera_path, backup_layer, camera_path):
            raise RuntimeError(f"Failed to preserve camera: {camera_path}")
        camera_paths[camera_name] = str(camera_path)

    render_path = Sdf.Path("/Render")
    has_render_tree = stage.GetPrimAtPath(render_path).IsValid()
    if has_render_tree:
        if source_layer.GetPrimAtPath(render_path) is None:
            if flattened_layer is None:
                flattened_layer = stage.Flatten()
            render_source = flattened_layer
        else:
            render_source = source_layer
        if not Sdf.CopySpec(
            render_source, render_path, backup_layer, render_path
        ):
            raise RuntimeError("Failed to preserve the existing /Render tree")

    if not camera_paths and not has_render_tree:
        backup_path.unlink(missing_ok=True)
        return None

    backup_layer.customLayerData = {
        "rby1PreservedCameraPaths": json.dumps(camera_paths),
        "rby1PreservedRender": str(has_render_tree),
    }
    if not backup_layer.Save():
        raise RuntimeError(f"Failed to save preserved USD content: {backup_path}")

    print(
        "Preserved existing USD content: "
        f"cameras={sorted(camera_paths)}, render={has_render_tree}",
        flush=True,
    )
    preserved = PreservedUsdContent(backup_path, camera_paths, has_render_tree)
    del stage, source_layer, flattened_layer, backup_layer
    gc.collect()
    return preserved


def load_preserved_usd_content(layer_path: Path) -> PreservedUsdContent:
    """Load the manifest written by the camera-extraction process."""

    from pxr import Sdf

    layer_path = layer_path.expanduser().resolve()
    layer = Sdf.Layer.FindOrOpen(str(layer_path))
    if layer is None:
        raise RuntimeError(f"Failed to open preserved USD content: {layer_path}")
    metadata = layer.customLayerData
    camera_paths = json.loads(metadata.get("rby1PreservedCameraPaths", "{}"))
    has_render_tree = metadata.get("rby1PreservedRender", "False") == "True"
    del layer
    gc.collect()
    return PreservedUsdContent(layer_path, camera_paths, has_render_tree)


def restore_preserved_usd_content(
    usd_path: Path,
    preserved: PreservedUsdContent | None,
    stage: object | None = None,
) -> dict[str, str]:
    """Attach preserved cameras to matching links and restore /Render."""

    if preserved is None:
        return {}

    from pxr import Sdf, Usd

    backup_layer = Sdf.Layer.FindOrOpen(str(preserved.layer_path))
    if backup_layer is None:
        raise RuntimeError(
            f"Failed to open preserved USD content: {preserved.layer_path}"
        )

    stage = stage or Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise RuntimeError(f"Failed to open generated USD for restoration: {usd_path}")
    root_layer = stage.GetRootLayer()
    robot_root = stage.GetDefaultPrim()
    if not robot_root.IsValid():
        raise RuntimeError("Cannot restore cameras before the default prim is set")

    robot_prims = list(Usd.PrimRange(robot_root))
    restored_paths: dict[str, str] = {}
    old_to_new_paths: dict[str, Sdf.Path] = {}
    for camera_name, old_camera_path in preserved.camera_paths.items():
        parent_name = PRESERVED_CAMERAS[camera_name]
        parents = [prim for prim in robot_prims if prim.GetName() == parent_name]
        if len(parents) != 1:
            paths = [str(prim.GetPath()) for prim in parents]
            raise RuntimeError(
                f"Cannot uniquely restore {camera_name}: expected one "
                f"{parent_name} link, found {paths}"
            )

        parent_path = parents[0].GetPath()
        destination_path = parent_path.AppendChild(camera_name)
        backup_path = Sdf.Path(f"/Cameras/{camera_name}")
        if not Sdf.CopySpec(
            backup_layer, backup_path, root_layer, destination_path
        ):
            raise RuntimeError(f"Failed to restore camera at {destination_path}")
        restored_paths[camera_name] = str(destination_path)
        old_to_new_paths[old_camera_path] = destination_path

    if preserved.has_render_tree:
        render_path = Sdf.Path("/Render")
        if not Sdf.CopySpec(
            backup_layer, render_path, root_layer, render_path
        ):
            raise RuntimeError("Failed to restore the preserved /Render tree")

    if not root_layer.Save():
        raise RuntimeError(f"Failed to save restored USD content: {usd_path}")
    stage.Reload()

    # RenderProduct camera relationships contain absolute prim paths. Keep
    # them valid if an import option changed the robot's root hierarchy.
    render_prim = stage.GetPrimAtPath("/Render")
    if render_prim.IsValid():
        for prim in Usd.PrimRange(render_prim):
            for relationship in prim.GetRelationships():
                targets = relationship.GetTargets()
                remapped = [old_to_new_paths.get(str(path), path) for path in targets]
                if remapped != targets:
                    relationship.SetTargets(remapped)

    if not root_layer.Save():
        raise RuntimeError(f"Failed to save remapped render settings: {usd_path}")

    stage.Reload()
    for camera_name, camera_path in restored_paths.items():
        camera = stage.GetPrimAtPath(camera_path)
        if not camera.IsValid() or camera.GetTypeName() != "Camera":
            raise RuntimeError(f"Restored camera is invalid: {camera_path}")
        expected_parent = PRESERVED_CAMERAS[camera_name]
        if camera.GetParent().GetName() != expected_parent:
            raise RuntimeError(
                f"Restored camera has the wrong parent: {camera_path}"
            )

    print(
        "Restored preserved USD content: "
        f"cameras={restored_paths}, render={preserved.has_render_tree}",
        flush=True,
    )
    return restored_paths


def ensure_default_prim(usd_path: Path, stage: object | None = None) -> str:
    """Author the generated robot root as the USD default prim."""

    from pxr import Usd

    stage = stage or Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise RuntimeError(f"Failed to open generated USD: {usd_path}")

    default_prim = stage.GetDefaultPrim()
    if not default_prim.IsValid():
        root_prims = [prim for prim in stage.GetPseudoRoot().GetChildren() if prim.IsActive()]
        if len(root_prims) != 1:
            paths = [str(prim.GetPath()) for prim in root_prims]
            raise RuntimeError(
                "Cannot choose a unique USD default prim from root prims: "
                f"{paths}"
            )
        default_prim = root_prims[0]
        stage.SetDefaultPrim(default_prim)
        if not stage.GetRootLayer().Save():
            raise RuntimeError(f"Failed to save USD default prim: {usd_path}")

    return str(default_prim.GetPath())


def install_generated_usd(staged_usd_path: Path, destination_usd_path: Path) -> None:
    """Install a validated staged USD and its importer-generated sidecars."""

    for staged_item in staged_usd_path.parent.iterdir():
        destination = destination_usd_path.parent / staged_item.name
        if staged_item.is_dir():
            shutil.copytree(staged_item, destination, dirs_exist_ok=True)
        elif staged_item == staged_usd_path:
            shutil.copy2(staged_item, destination_usd_path)
        else:
            shutil.copy2(staged_item, destination)


def repair_empty_link_visual_references(usd_path: Path) -> list[str]:
    """Remove Isaac 5.1 references created for visual-less EE links."""

    from pxr import Sdf

    base_layer_path = (
        usd_path.parent / "configuration" / f"{usd_path.stem}_base.usd"
    )
    layer = Sdf.Layer.FindOrOpen(str(base_layer_path))
    if layer is None:
        return []

    repaired: list[str] = []

    def visit(spec: object) -> None:
        if (
            spec.name == "visuals"
            and spec.nameParent is not None
            and spec.nameParent.name in {"ee_left", "ee_right"}
        ):
            spec.referenceList.ClearEdits()
            repaired.append(str(spec.path))
        for child in spec.nameChildren:
            visit(child)

    for root_spec in layer.rootPrims:
        visit(root_spec)
    if repaired and not layer.Save():
        raise RuntimeError(f"Failed to repair empty-link references: {base_layer_path}")
    if repaired:
        print(f"Repaired empty-link visual references: {repaired}", flush=True)
    return repaired


def restore_preserved_specs_directly(
    usd_path: Path, preserved_layer_path: Path
) -> dict[str, str]:
    """Install the preserved specs as a camera overlay sublayer."""

    from pxr import Sdf

    root_layer = Sdf.Layer.FindOrOpen(str(usd_path))
    if root_layer is None:
        raise RuntimeError(f"Failed to open generated USD layer: {usd_path}")
    overlay_name = f"{usd_path.stem}_cameras.usda"
    overlay_path = usd_path.parent / "configuration" / overlay_name
    overlay_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(preserved_layer_path, overlay_path)
    relative_overlay_path = f"./configuration/{overlay_name}"
    sublayers = [
        path for path in root_layer.subLayerPaths if path != relative_overlay_path
    ]
    root_layer.subLayerPaths = [relative_overlay_path, *sublayers]
    root_layer.defaultPrim = "RBY1_A_v1_2"
    if not root_layer.Save():
        raise RuntimeError(f"Failed to save restored USD: {usd_path}")
    restored = {
        "cam_right": "/RBY1_A_v1_2/root_joint/gripper_right/cam_right",
        "cam_left": "/RBY1_A_v1_2/root_joint/gripper_left/cam_left",
        "cam_head": "/RBY1_A_v1_2/root_joint/link_head_2/cam_head",
    }
    print(f"Restored preserved camera specs: {restored}", flush=True)
    return restored


def main() -> None:
    require_isaaclab_environment()
    args = parse_args()
    urdf_path = args.urdf.expanduser().resolve()
    usd_path = args.usd.expanduser().resolve()

    if not urdf_path.is_file():
        raise FileNotFoundError(f"URDF does not exist: {urdf_path}")
    if usd_path.suffix.lower() not in {".usd", ".usda", ".usdc"}:
        raise ValueError(f"Output must be a USD file: {usd_path}")
    usd_path.parent.mkdir(parents=True, exist_ok=True)

    # Reading an existing stage and importing a URDF in the same Isaac process
    # can terminate Kit early because both operations touch the same layer
    # cache. Orchestrate extraction and conversion in isolated child processes.
    if (
        args._extract_preserved_to is None
        and args._preserved_layer is None
        and not args._import_only
        and not args._postprocess_only
        and not args._repair_only
        and not args._restore_only
    ):
        with tempfile.TemporaryDirectory(prefix="rby1_camera_handoff_") as temp_dir:
            preserved_path = Path(temp_dir) / "preserved.usda"
            base_command = [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]]
            if not args.no_preserve_cameras and usd_path.is_file():
                subprocess.run(
                    [*base_command, "--_extract-preserved-to", str(preserved_path)],
                    check=True,
                )
                print("Camera extraction process completed.", flush=True)

            print("Starting isolated URDF import process.", flush=True)
            subprocess.run([*base_command, "--_import-only"], check=True)
            print("Isolated URDF import process completed.", flush=True)

            subprocess.run([*base_command, "--_repair-only"], check=True)
            print("USD sidecar repair process completed.", flush=True)

            if preserved_path.is_file():
                subprocess.run(
                    [
                        *base_command,
                        "--_restore-only",
                        "--_preserved-layer",
                        str(preserved_path),
                    ],
                    check=True,
                )
                print("Camera restoration process completed.", flush=True)

            postprocess_command = [*base_command, "--_postprocess-only"]
            if preserved_path.is_file():
                postprocess_command.append("--_expect-preserved-cameras")
            subprocess.run(postprocess_command, check=True)
            print("USD post-processing process completed.", flush=True)
        return

    from isaaclab.app import AppLauncher

    if args._extract_preserved_to is not None:
        app_launcher = AppLauncher(headless=True)
        app = app_launcher.app
        try:
            preserve_existing_usd_content(usd_path, args._extract_preserved_to)
        finally:
            app.close()
        return

    if args._repair_only:
        app_launcher = AppLauncher(headless=True)
        app = app_launcher.app
        try:
            repair_empty_link_visual_references(usd_path)
        finally:
            app.close()
        return

    if args._restore_only:
        if args._preserved_layer is None:
            raise RuntimeError("--_restore-only requires --_preserved-layer")
        app_launcher = AppLauncher(headless=True)
        app = app_launcher.app
        try:
            restore_preserved_specs_directly(usd_path, args._preserved_layer)
        finally:
            app.close()
        return

    if args._postprocess_only:
        app_launcher = AppLauncher(headless=True)
        app = app_launcher.app
        try:
            from pxr import Usd

            stage = Usd.Stage.Open(str(usd_path))
            if stage is None:
                raise RuntimeError(f"Failed to open imported USD: {usd_path}")
            default_prim_path = ensure_default_prim(usd_path, stage)
            prim_count, joint_count, articulation_count = validate_usd(
                usd_path, stage
            )
            print(f"Created USD: {usd_path}", flush=True)
            print(f"Default prim: {default_prim_path}", flush=True)
            camera_paths = [
                str(prim.GetPath())
                for prim in stage.TraverseAll()
                if prim.GetTypeName() == "Camera"
            ]
            if args._expect_preserved_cameras:
                expected_paths = {
                    "/RBY1_A_v1_2/root_joint/gripper_right/cam_right",
                    "/RBY1_A_v1_2/root_joint/gripper_left/cam_left",
                    "/RBY1_A_v1_2/root_joint/link_head_2/cam_head",
                }
                missing_cameras = expected_paths - set(camera_paths)
                if missing_cameras:
                    raise RuntimeError(
                        f"Preserved cameras are missing: {sorted(missing_cameras)}"
                    )
                if not stage.GetPrimAtPath("/Render").IsValid():
                    raise RuntimeError("Preserved /Render tree is missing")
            if camera_paths:
                print(f"Preserved cameras: {camera_paths}", flush=True)
            print(
                "Validated stage: "
                f"prims={prim_count}, joints={joint_count}, "
                f"articulations={articulation_count}",
                flush=True,
            )
        finally:
            app.close()
        return

    staging_dir = Path(tempfile.mkdtemp(prefix="rby1_usd_output_"))
    staged_usd_path = usd_path if args._import_only else staging_dir / usd_path.name
    preserved = None

    app_launcher = AppLauncher(headless=True)
    app = app_launcher.app
    try:
        import omni.kit.commands
        import omni.usd
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
                dest_path=str(staged_usd_path),
                get_articulation_root=True,
            )
        if not status:
            raise RuntimeError(f"URDF import failed: {articulation_root}")

        if not staged_usd_path.is_file():
            raise RuntimeError(
                f"Importer did not create the expected USD: {staged_usd_path}"
            )
        if args._import_only:
            print(f"Imported URDF to USD: {usd_path}", flush=True)
            return

        # Edit the stage held by Kit itself. Editing another stage for the same
        # file can be overwritten when Kit later saves the importer stage.
        generated_stage = omni.usd.get_context().get_stage()
        if generated_stage is None:
            raise RuntimeError("URDF importer did not leave a generated stage open")
        if args._preserved_layer is not None:
            preserved = load_preserved_usd_content(args._preserved_layer)
        default_prim_path = ensure_default_prim(staged_usd_path, generated_stage)
        restored_cameras = restore_preserved_usd_content(
            staged_usd_path, preserved, generated_stage
        )
        prim_count, joint_count, articulation_count = validate_usd(
            staged_usd_path, generated_stage
        )
        install_generated_usd(staged_usd_path, usd_path)
        print(f"Created USD: {usd_path}", flush=True)
        print(f"Default prim: {default_prim_path}", flush=True)
        print(f"Articulation root: {articulation_root}", flush=True)
        if restored_cameras:
            print(f"Preserved cameras: {restored_cameras}", flush=True)
        print(
            "Validated stage: "
            f"prims={prim_count}, joints={joint_count}, "
            f"articulations={articulation_count}",
            flush=True,
        )
    finally:
        app.close()
        shutil.rmtree(staging_dir, ignore_errors=True)


if __name__ == "__main__":
    main()
