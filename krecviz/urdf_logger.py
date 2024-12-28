"""Modified version of the URDF logger, with extra print statements before each Rerun log call.

Taken from:
https://github.com/rerun-io/rerun-loader-python-example-urdf
"""

from __future__ import annotations

import argparse
import logging
import math
import sys
from pathlib import Path

import numpy as np
import rerun as rr
import trimesh
from PIL import Image
from urdf_parser_py import urdf as urdf_parser  # type: ignore[import-untyped]


# Separate debug-print functions.
def debug_print_log_view_coordinates(
    entity_path_val: str, entity_val: rr.components.view_coordinates.ViewCoordinates, timeless_val: bool
) -> None:
    """Print debug info before calling rr.log(...) for the root view coordinates."""
    print("======================")
    print("rerun_log")
    print(f"entity_path = self.add_entity_path_prefix(\"\") with value '{entity_path_val}'")
    print(f"entity = rr.ViewCoordinates.RIGHT_HAND_Z_UP with value {entity_val}")
    print(f"timeless = {timeless_val}")


def debug_print_log_joint(
    entity_path_w_prefix: str,
    joint: urdf_parser.Joint,
    translation: list[float] | None,
    rotation: list[list[float]] | None,
) -> None:
    """Print debug info before logging the Transform3D of a joint."""
    print("======================")
    print("rerun_log")
    print(f"entity_path = entity_path_w_prefix with value '{entity_path_w_prefix}'")
    print("Original joint RPY values:")
    if joint.origin is not None and joint.origin.rpy is not None:
        print(f"  => rpy = {[round(float(x), 3) for x in joint.origin.rpy]}")
    else:
        print("  => rpy = None")

    print("entity = rr.Transform3D with:")
    print("  translation:", [f"{x:>8.3f}" for x in translation] if translation else None)
    print("  mat3x3:")
    if rotation:
        for row in rotation:
            print("    [" + ", ".join(f"{x:>8.3f}" for x in row) + "]")
    else:
        print("    None")


def debug_print_unsupported_geometry(entity_path_val: str, log_text: str) -> None:
    """Print debug info for the 'Unsupported geometry' case before logging rr.TextLog."""
    print("======================")
    print("rerun_log")
    print(f"entity_path = self.add_entity_path_prefix(\"\") with value '{entity_path_val}'")
    print(f"entity = rr.TextLog(...) with value '{log_text}'")


def debug_print_log_trimesh(
    entity_path: str, mesh3d_entity: rr.Mesh3D, timeless_val: bool, mesh: trimesh.Trimesh
) -> None:
    """Print debug info prior to rr.log(...) a single Trimesh."""
    print("======================")
    print("rerun_log log_trimesh")
    print(f"entity_path = entity_path with value '{entity_path}'")
    print("entity = rr.Mesh3D(...) with these numeric values:")

    # Print only the first three vertex positions for brevity
    first_three_vertices = mesh.vertices[:3].tolist()
    print("  => vertex_positions (first 3):")
    for vertex in first_three_vertices:
        print(f"      [{', '.join(f'{x:>7.3f}' for x in vertex)}]")

    print(f"timeless = {timeless_val}")


def debug_print_final_link_transform(link_name: str, chain: list[str], final_tf: np.ndarray) -> None:
    """Print the final transform accumulated for a link."""
    print(f"Link '{link_name}': BFS chain = {chain}")
    print("  => final_tf (4x4) =")
    for row in final_tf:
        print("  [{: 8.3f} {: 8.3f} {: 8.3f} {: 8.3f}]".format(*row))
    print()


def rotation_from_euler_xyz(rpy: list[float] | tuple[float, float, float]) -> np.ndarray:
    """Convert Euler angles to a 3x3 rotation matrix using XYZ rotation sequence.

    Args:
        rpy: List or tuple of 3 Euler angles [rx, ry, rz] in radians, representing rotations
             around the X, Y and Z axes respectively.

    Returns:
        np.ndarray: A 3x3 rotation matrix representing the combined rotation, computed as
                   R = Rz @ Ry @ Rx (right-to-left multiplication order).

    Note:
        This follows the extrinsic/fixed XYZ convention, where rotations are applied in order:
        1. First rotate around X axis by rx
        2. Then rotate around Y axis by ry
        3. Finally rotate around Z axis by rz
    """
    rx, ry, rz = rpy

    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)

    r_x = np.array(
        [
            [1, 0, 0],
            [0, cx, -sx],
            [0, sx, cx],
        ],
        dtype=np.float64,
    )
    r_y = np.array(
        [
            [cy, 0, sy],
            [0, 1, 0],
            [-sy, 0, cy],
        ],
        dtype=np.float64,
    )
    r_z = np.array(
        [
            [cz, -sz, 0],
            [sz, cz, 0],
            [0, 0, 1],
        ],
        dtype=np.float64,
    )

    # Final rotation = Rz @ Ry @ Rx
    return r_z @ r_y @ r_x


class URDFLogger:
    """Class to log a URDF to Rerun, with debug prints before each log call."""

    def __init__(self, filepath: str, entity_path_prefix: str = "") -> None:
        self.filepath = Path(filepath).resolve()
        self.urdf_dir = self.filepath.parent
        self.urdf = urdf_parser.URDF.from_xml_file(str(self.filepath))
        self.mat_name_to_mat = {mat.name: mat for mat in self.urdf.materials}
        self.entity_to_transform: dict[str, tuple[list[float], list[list[float]]]] = {}
        self.entity_path_prefix = entity_path_prefix

    def link_entity_path(self, link: urdf_parser.Link) -> str:
        """Return the entity path for the URDF link."""
        root_name = self.urdf.get_root()
        link_names = self.urdf.get_chain(root_name, link.name)[0::2]
        return "/".join(link_names)

    def joint_entity_path(self, joint: urdf_parser.Joint) -> str:
        """Return the entity path for the URDF joint."""
        root_name = self.urdf.get_root()
        link_names = self.urdf.get_chain(root_name, joint.child)[0::2]
        return "/".join(link_names)

    def add_entity_path_prefix(self, entity_path: str) -> str:
        if self.entity_path_prefix:
            return f"{self.entity_path_prefix}/{entity_path}"
        return entity_path

    def log(self) -> None:
        """Log a URDF file to Rerun."""
        # Log the "root" coordinates
        entity_path_val = self.add_entity_path_prefix("")
        entity_val = rr.ViewCoordinates.RIGHT_HAND_Z_UP
        timeless_val = True

        # --- CHANGED ---
        # Now we call our debug-print function instead of inlining the prints:
        debug_print_log_view_coordinates(entity_path_val, entity_val, timeless_val)

        rr.log(
            entity_path=entity_path_val,
            entity=entity_val,
            static=timeless_val,
        )

        # Now log joints
        for joint in self.urdf.joints:
            entity_path = self.joint_entity_path(joint)
            self.log_joint(entity_path, joint)

        # Now log links
        for link in self.urdf.links:
            entity_path = self.link_entity_path(link)
            self.log_link(entity_path, link)

        # Print final transforms
        self.print_final_link_transforms()

    def log_link(self, entity_path: str, link: urdf_parser.Link) -> None:
        """Log a URDF link to Rerun."""
        for i, visual in enumerate(link.visuals):
            self.log_visual(entity_path + f"/visual_{i}", visual)

    def log_joint(self, entity_path: str, joint: urdf_parser.Joint) -> None:
        """Log a URDF joint to Rerun."""
        translation: list[float] | None = None
        rotation: list[list[float]] | None = None

        if joint.origin is not None and joint.origin.xyz is not None:
            translation = [float(x) for x in joint.origin.xyz]

        if joint.origin is not None and joint.origin.rpy is not None:
            rotation_matrix = rotation_from_euler_xyz(joint.origin.rpy)
            rotation = [[float(x) for x in row] for row in rotation_matrix]

        entity_path_w_prefix = self.add_entity_path_prefix(entity_path)
        if isinstance(translation, list) and isinstance(rotation, list):
            self.entity_to_transform[entity_path_w_prefix] = (translation, rotation)

        # --- CHANGED ---
        debug_print_log_joint(entity_path_w_prefix, joint, translation, rotation)

        transform_3d = rr.Transform3D(translation=translation, mat3x3=rotation)
        rr.log(
            entity_path=entity_path_w_prefix,
            entity=transform_3d,
        )

    def log_visual(self, entity_path: str, visual: urdf_parser.Visual) -> None:
        """Log a URDF visual to Rerun."""
        material = None
        if visual.material is not None:
            if visual.material.color is None and visual.material.texture is None:
                material = self.mat_name_to_mat.get(visual.material.name, None)
            else:
                material = visual.material

        transform = np.eye(4)
        if visual.origin is not None and visual.origin.xyz is not None:
            transform[:3, 3] = visual.origin.xyz
        if visual.origin is not None and visual.origin.rpy is not None:
            transform[:3, :3] = rotation_from_euler_xyz(visual.origin.rpy)

        # Geometry handling (same as original)
        if isinstance(visual.geometry, urdf_parser.Mesh):
            resolved_path = self.resolve_ros_path(visual.geometry.filename)
            mesh_scale = visual.geometry.scale
            mesh_or_scene = trimesh.load_mesh(str(resolved_path))
            if mesh_scale is not None:
                transform[:3, :3] *= mesh_scale
        elif isinstance(visual.geometry, urdf_parser.Box):
            mesh_or_scene = trimesh.creation.box(extents=visual.geometry.size)
        elif isinstance(visual.geometry, urdf_parser.Cylinder):
            mesh_or_scene = trimesh.creation.cylinder(
                radius=visual.geometry.radius,
                height=visual.geometry.length,
            )
        elif isinstance(visual.geometry, urdf_parser.Sphere):
            mesh_or_scene = trimesh.creation.icosphere(
                radius=visual.geometry.radius,
            )
        else:
            raise ValueError(f"Unsupported geometry type: {type(visual.geometry)}")
            log_text = f"Unsupported geometry type: {type(visual.geometry)}"
            entity_path_val = self.add_entity_path_prefix("")

            # --- CHANGED ---
            debug_print_unsupported_geometry(entity_path_val, log_text)

            rr.log(
                entity_path=entity_path_val,
                entity=rr.TextLog(log_text),
            )
            mesh_or_scene = trimesh.Trimesh()

        # If we have a Scene, break it down into Trimeshes
        if isinstance(mesh_or_scene, trimesh.Scene):
            mesh_or_scene.apply_transform(transform)
            for i, mesh in enumerate(scene_to_trimeshes(mesh_or_scene)):
                if material is not None and not isinstance(mesh.visual, trimesh.visual.texture.TextureVisuals):
                    if material.color is not None:
                        mesh.visual = trimesh.visual.ColorVisuals()
                        mesh.visual.vertex_colors = material.color.rgba
                    elif material.texture is not None:
                        texture_path = self.resolve_ros_path(material.texture.filename)
                        mesh.visual = trimesh.visual.texture.TextureVisuals(image=Image.open(str(texture_path)))

                final_entity_path = self.add_entity_path_prefix(entity_path + f"/{i}")
                log_trimesh(final_entity_path, mesh)
        elif isinstance(mesh_or_scene, trimesh.Trimesh):
            mesh = mesh_or_scene
            mesh.apply_transform(transform)
            if material is not None and not isinstance(mesh.visual, trimesh.visual.texture.TextureVisuals):
                if material.color is not None:
                    mesh.visual = trimesh.visual.ColorVisuals()
                    mesh.visual.vertex_colors = material.color.rgba
                elif material.texture is not None:
                    texture_path = self.resolve_ros_path(material.texture.filename)
                    mesh.visual = trimesh.visual.texture.TextureVisuals(image=Image.open(str(texture_path)))

            final_entity_path = self.add_entity_path_prefix(entity_path)
            log_trimesh(final_entity_path, mesh)
        else:
            logging.warning("Unexpected geometry type: %s", type(mesh_or_scene))

    def resolve_ros_path(self, path: str) -> Path:
        """Resolve a path relative to the URDF directory if not a package or file URI."""
        if path.startswith("package://"):
            raise ValueError(f"Could not resolve '{path}'. Provide a direct or relative path.")
        elif path.startswith("file://"):
            resolved = path[len("file://") :]
            return Path(resolved).resolve()
        else:
            direct_path = (self.urdf_dir / path).resolve()
            if direct_path.exists():
                return direct_path
            parent_path = (self.urdf_dir.parent / path).resolve()
            if parent_path.exists():
                return parent_path
            raise FileNotFoundError(
                f"Could not find file '{path}' relative to '{self.urdf_dir}' or its parent. "
                "Please check that the file exists and is accessible."
            )

    def print_final_link_transforms(self) -> None:
        """Debug function: print accumulated joint transforms from root to each link.

        For each link, accumulate the joint transforms from root -> link, then print the resulting final 4x4.
        """
        root_link = self.urdf.get_root()
        print("\n========== FINAL ACCUMULATED TRANSFORMS PER LINK ==========")
        for link in self.urdf.links:
            if link.name == root_link:
                print(f"Link '{link.name}': Root link => final transform is identity.\n")
                continue

            chain = self.urdf.get_chain(root_link, link.name)
            final_tf = np.eye(4, dtype=np.float64)
            for i in range(1, len(chain), 2):
                joint_name = chain[i]
                j = None
                for jt in self.urdf.joints:
                    if jt.name == joint_name:
                        j = jt
                        break
                if j is None:
                    print(f"  (!) Could not find joint named '{joint_name}' in URDF?")
                    continue

                xyz = j.origin.xyz if j.origin and j.origin.xyz else [0, 0, 0]
                rpy = j.origin.rpy if j.origin and j.origin.rpy else [0, 0, 0]
                local_rot = rotation_from_euler_xyz(rpy)
                local_tf = np.eye(4, dtype=np.float64)
                local_tf[:3, :3] = local_rot
                local_tf[:3, 3] = xyz

                final_tf = np.array(final_tf @ local_tf, dtype=np.float64)

            debug_print_final_link_transform(link.name, chain, final_tf)


def scene_to_trimeshes(scene: trimesh.Scene) -> list[trimesh.Trimesh]:
    """Convert a trimesh.Scene to a list of trimesh.Trimesh.

    Skips objects that are not instances of trimesh.Trimesh.
    """
    trimeshes: list[trimesh.Trimesh] = []
    scene_dump = scene.dump()

    if isinstance(scene_dump, list):
        geometries = scene_dump
    else:
        geometries = [scene_dump]

    for geometry in geometries:
        if isinstance(geometry, trimesh.Trimesh):
            trimeshes.append(geometry)
        elif isinstance(geometry, trimesh.Scene):
            trimeshes.extend(scene_to_trimeshes(geometry))
    return trimeshes


def log_trimesh(entity_path: str, mesh: trimesh.Trimesh) -> None:
    """Log a single Trimesh to Rerun, with debug prints."""
    vertex_colors = albedo_texture = vertex_texcoords = None

    if isinstance(mesh.visual, trimesh.visual.color.ColorVisuals):
        vertex_colors = mesh.visual.vertex_colors
    elif isinstance(mesh.visual, trimesh.visual.texture.TextureVisuals):
        trimesh_material = mesh.visual.material

        if mesh.visual.uv is not None:
            vertex_texcoords = mesh.visual.uv
            # Trimesh uses the OpenGL convention for UV coordinates, so we need to flip the V coordinate
            # since Rerun uses the Vulkan/Metal/DX12/WebGPU convention.
            vertex_texcoords[:, 1] = 1.0 - vertex_texcoords[:, 1]

        # Handle PBR materials or simple texture
        if hasattr(trimesh_material, "baseColorTexture") and trimesh_material.baseColorTexture is not None:
            # baseColorTexture is a PIL image or array
            img = np.asarray(trimesh_material.baseColorTexture)
            if img.ndim == 2:
                img = np.stack([img] * 3, axis=-1)
            albedo_texture = img
        elif hasattr(trimesh_material, "baseColorFactor") and trimesh_material.baseColorFactor is not None:
            vertex_colors = trimesh_material.baseColorFactor
        else:
            colors = mesh.visual.to_color().vertex_colors
            vertex_colors = colors
    else:
        # Try to get colors anyway
        try:
            colors = mesh.visual.to_color().vertex_colors
            vertex_colors = colors
        except Exception:
            pass

    # --- CHANGED ---
    # Prepare the rr.Mesh3D, then debug-print all the info in a helper function.
    mesh3d_entity = rr.Mesh3D(
        vertex_positions=mesh.vertices,
        triangle_indices=mesh.faces,
        vertex_normals=mesh.vertex_normals,
        vertex_colors=vertex_colors,
        albedo_texture=albedo_texture,
        vertex_texcoords=vertex_texcoords,
    )

    timeless_val = True
    debug_print_log_trimesh(entity_path, mesh3d_entity, timeless_val, mesh)

    rr.log(
        entity_path=entity_path,
        entity=mesh3d_entity,
        static=timeless_val,
    )


def main() -> None:
    # The Rerun Viewer will always pass these two pieces of information:
    # 1. The path to be loaded, as a positional arg.
    # 2. A shared recording ID, via the `--recording-id` flag.
    #
    # It is up to you whether you make use of that shared recording ID or not.
    # If you use it, the data will end up in the same recording as all other plugins interested in
    # that file, otherwise you can just create a dedicated recording for it. Or both.
    parser = argparse.ArgumentParser(
        description=(
            "This is an example executable data-loader plugin for the Rerun Viewer. "
            "Any executable on your $PATH with a name that starts with rerun-loader- will be "
            "treated as an external data-loader.\n\n"
            "This example will load URDF files, log them to Rerun, "
            "and return a special exit code to indicate that it doesn't support anything else."
        )
    )
    parser.add_argument("filepath", type=str)
    parser.add_argument("--recording-id", type=str)
    parser.add_argument("--entity-path-prefix", type=str, default="")
    args = parser.parse_args()

    filepath = Path(args.filepath).resolve()
    is_file = filepath.is_file()
    # Changed from the old code to handle uppercase/lowercase URDF:
    is_urdf_file = ".urdf" in filepath.name.lower()

    if not is_file or not is_urdf_file:
        sys.exit(rr.EXTERNAL_DATA_LOADER_INCOMPATIBLE_EXIT_CODE)

    rr.init("rerun_example_external_data_loader_urdf", recording_id=args.recording_id)
    rr.stdout()

    urdf_logger = URDFLogger(str(filepath), args.entity_path_prefix)
    urdf_logger.log()


if __name__ == "__main__":
    main()
