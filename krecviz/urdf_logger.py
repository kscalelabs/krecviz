"""Modified version of the URDF logger.

Taken from:
https://github.com/rerun-io/rerun-loader-python-example-urdf
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import numpy as np
import rerun as rr  # pip install rerun-sdk
import scipy.spatial.transform as st
import trimesh
from PIL import Image
from urdf_parser_py import urdf as urdf_parser  # type: ignore[import-untyped]


class URDFLogger:
    """Class to log a URDF to Rerun."""

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
        rr.log(
            self.add_entity_path_prefix(""),
            rr.ViewCoordinates.RIGHT_HAND_Z_UP,
            timeless=True,
        )

        for joint in self.urdf.joints:
            entity_path = self.joint_entity_path(joint)
            self.log_joint(entity_path, joint)

        for link in self.urdf.links:
            entity_path = self.link_entity_path(link)
            self.log_link(entity_path, link)

    def log_link(self, entity_path: str, link: urdf_parser.Link) -> None:
        """Log a URDF link to Rerun."""
        # create one mesh out of all visuals
        for i, visual in enumerate(link.visuals):
            self.log_visual(entity_path + f"/visual_{i}", visual)

    def log_joint(self, entity_path: str, joint: urdf_parser.Joint) -> None:
        """Log a URDF joint to Rerun."""
        translation: list[float] | None = None
        rotation: list[list[float]] | None = None

        if joint.origin is not None and joint.origin.xyz is not None:
            translation = [float(x) for x in joint.origin.xyz]

        if joint.origin is not None and joint.origin.rpy is not None:
            rotation_matrix = st.Rotation.from_euler("xyz", joint.origin.rpy).as_matrix()
            rotation = [[float(x) for x in row] for row in rotation_matrix]

        entity_path_w_prefix = self.add_entity_path_prefix(entity_path)

        if isinstance(translation, list) and isinstance(rotation, list):
            self.entity_to_transform[entity_path_w_prefix] = (translation, rotation)

        rr.log(
            entity_path_w_prefix,
            rr.Transform3D(translation=translation, mat3x3=rotation),
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
            transform[:3, :3] = st.Rotation.from_euler("xyz", visual.origin.rpy).as_matrix()

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
            rr.log(
                self.add_entity_path_prefix(""),
                rr.TextLog("Unsupported geometry type: " + str(type(visual.geometry))),
            )
            mesh_or_scene = trimesh.Trimesh()

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
                log_trimesh(self.add_entity_path_prefix(entity_path + f"/{i}"), mesh)
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
            log_trimesh(self.add_entity_path_prefix(entity_path), mesh)
        else:
            logging.warning("Unexpected geometry type: %s", type(mesh_or_scene))
            return

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
            # If we cannot find it, raise an error with a clear message
            raise FileNotFoundError(
                f"Could not find file '{path}' relative to '{self.urdf_dir}' or its parent. "
                "Please check that the file exists and is accessible."
            )


def scene_to_trimeshes(scene: trimesh.Scene) -> list[trimesh.Trimesh]:
    """Convert a trimesh.Scene to a list of trimesh.Trimesh.

    Skips objects that are not instances of trimesh.Trimesh.
    """
    trimeshes = []
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

    rr.log(
        entity_path,
        rr.Mesh3D(
            vertex_positions=mesh.vertices,
            triangle_indices=mesh.faces,
            vertex_normals=mesh.vertex_normals,
            vertex_colors=vertex_colors,
            albedo_texture=albedo_texture,
            vertex_texcoords=vertex_texcoords,
        ),
        timeless=True,
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
    is_urdf_file = ".urdf" in filepath.name

    if not is_file or not is_urdf_file:
        sys.exit(rr.EXTERNAL_DATA_LOADER_INCOMPATIBLE_EXIT_CODE)

    rr.init("rerun_example_external_data_loader_urdf", recording_id=args.recording_id)
    rr.stdout()

    urdf_logger = URDFLogger(str(filepath), args.entity_path_prefix)
    urdf_logger.log()


if __name__ == "__main__":
    main()
