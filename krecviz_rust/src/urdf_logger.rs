use std::collections::{HashMap, VecDeque};
use std::fs;
use std::path::{Path, PathBuf};

use anyhow::Result;
use image; // for loading actual images
use nalgebra as na;
use parry3d::shape::{Ball as ParrySphere, Cuboid as ParryCuboid, Cylinder as ParryCylinder};
use rerun::{
    archetypes::{Mesh3D, Transform3D},
    components::{Position3D, TriangleIndices},
    datatypes::ImageFormat, // Add ImageFormat back
    RecordingStream,
    TextDocument,
    ViewCoordinates,
};
use urdf_rs::{self, Geometry, Joint, Link, Material};

// -----------------------------------------------------------------------------
// NEW: import from geometry_utils + spatial_transform_utils
// -----------------------------------------------------------------------------
use crate::debug_log_utils::{
    debug_print_joint_transform, debug_print_mesh_log, debug_print_stl_load,
    print_final_link_transforms,
};
use crate::geometry_utils::{
    apply_4x4_to_mesh3d, compute_vertex_normals, float_rgba_to_u8, load_image_as_rerun_buffer,
    load_stl_as_mesh3d,
};
use crate::spatial_transform_utils::{
    build_4x4_from_xyz_rpy, decompose_4x4_to_translation_and_mat3x3,
};
use crate::urdf_bfs_utils::{build_adjacency, find_root_link_name, link_entity_path};

// -----------------------------------------------------------------------------
// Minimal info (color & texture path) from a URDF Material.
// -----------------------------------------------------------------------------
#[derive(Default, Debug)]
struct RrMaterialInfo {
    /// RGBA in [0..1].
    color_rgba: Option<[f32; 4]>,
    /// Absolute path to a texture file, if any.
    texture_path: Option<PathBuf>,
}

// ----------------------------------------------------------------------------
// Stage1: log geometry at identity
fn log_link_meshes_at_identity(
    link: &Link,
    entity_path: &str,
    urdf_dir: &Path,
    all_mat_map: &HashMap<String, &Material>,
    rec: &RecordingStream,
) -> Result<()> {
    let mut doc_text = format!("Link at IDENTITY: {}\n", link.name);

    for (i, vis) in link.visual.iter().enumerate() {
        let mesh_entity_path = format!("{}/visual_{}", entity_path, i);

        // parse material
        let mut mat_info = RrMaterialInfo::default();
        if let Some(m) = &vis.material {
            if m.color.is_none() && m.texture.is_none() {
                if let Some(global_mat) = all_mat_map.get(&m.name) {
                    mat_info = parse_urdf_material(global_mat, urdf_dir);
                }
            } else {
                mat_info = parse_urdf_material(m, urdf_dir);
            }
        }

        // Build geometry info
        let (mut mesh3d, info_txt) = match &vis.geometry {
            Geometry::Mesh { filename, scale } => {
                let joined = urdf_dir.join(filename);
                // canonicalize will remove things like "../"
                let abs_path = match fs::canonicalize(&joined) {
                    Ok(resolved) => resolved,
                    Err(_) => {
                        // If canonicalize fails (e.g. file not found),
                        // use joined as fallback
                        joined
                    }
                };
                debug_print_stl_load(&abs_path);
                let mut info_txt = format!("Mesh file={:?}, scale={:?}\n", abs_path, scale);
                if abs_path
                    .extension()
                    .and_then(|e| e.to_str())
                    .map(|s| s.to_lowercase())
                    .as_deref()
                    == Some("stl")
                {
                    match load_stl_as_mesh3d(&abs_path) {
                        Ok(m) => (m, info_txt),
                        Err(e) => {
                            info_txt.push_str(&format!("(Error loading STL: {e})\n"));
                            (Mesh3D::new(Vec::<[f32; 3]>::new()), info_txt)
                        }
                    }
                } else {
                    info_txt.push_str("(Currently only .stl handled)\n");
                    (Mesh3D::new(Vec::<[f32; 3]>::new()), info_txt)
                }
            }
            Geometry::Box { size } => {
                let (sx, sy, sz) = (size[0], size[1], size[2]);
                let info_txt = format!("Box size=({},{},{})\n", sx, sy, sz);
                let cuboid = ParryCuboid::new(na::Vector3::new(
                    (sx / 2.0) as f32,
                    (sy / 2.0) as f32,
                    (sz / 2.0) as f32,
                ));
                let (raw_v, raw_i) = cuboid.to_trimesh();
                // Convert them to Mesh3D
                let positions: Vec<Position3D> = raw_v
                    .iter()
                    .map(|p| Position3D::from([p.x, p.y, p.z]))
                    .collect();
                let indices: Vec<TriangleIndices> = raw_i
                    .iter()
                    .map(|[a, b, c]| TriangleIndices::from([*a, *b, *c]))
                    .collect();
                let mut mesh = Mesh3D::new(positions).with_triangle_indices(indices);

                // compute normals
                compute_vertex_normals(&mut mesh);

                (mesh, info_txt)
            }
            Geometry::Cylinder { radius, length } => {
                let info_txt = format!("Cylinder r={}, length={}\n", radius, length);
                let half = (*length as f32) / 2.0;
                let cyl = ParryCylinder::new(half, *radius as f32);
                let (raw_v, raw_i) = cyl.to_trimesh(30);
                let positions: Vec<Position3D> = raw_v
                    .iter()
                    .map(|p| Position3D::from([p.x, p.y, p.z]))
                    .collect();
                let indices: Vec<TriangleIndices> = raw_i
                    .iter()
                    .map(|[a, b, c]| TriangleIndices::from([*a, *b, *c]))
                    .collect();
                let mut mesh = Mesh3D::new(positions).with_triangle_indices(indices);

                // pre-rotate so cylinder axis is +Z
                let rotate_x_90 = build_4x4_from_xyz_rpy(
                    [0.0, 0.0, 0.0],
                    [-std::f64::consts::FRAC_PI_2, 0.0, 0.0],
                );
                apply_4x4_to_mesh3d(&mut mesh, rotate_x_90);

                // now compute normals
                compute_vertex_normals(&mut mesh);

                (mesh, info_txt)
            }
            Geometry::Sphere { radius } => {
                let info_txt = format!("Sphere radius={}\n", radius);
                let ball = ParrySphere::new(*radius as f32);
                let (raw_v, raw_i) = ball.to_trimesh(20, 20);
                let positions: Vec<Position3D> = raw_v
                    .iter()
                    .map(|p| Position3D::from([p.x, p.y, p.z]))
                    .collect();
                let indices: Vec<TriangleIndices> = raw_i
                    .iter()
                    .map(|[a, b, c]| TriangleIndices::from([*a, *b, *c]))
                    .collect();
                let mut mesh = Mesh3D::new(positions).with_triangle_indices(indices);

                // compute normals
                compute_vertex_normals(&mut mesh);

                (mesh, info_txt)
            }
            _ => {
                let info_txt = String::from("(Unsupported geometry)\n");
                (Mesh3D::new(Vec::<[f32; 3]>::new()), info_txt)
            }
        };

        doc_text.push_str(&format!("Visual #{} => {}\n", i, info_txt));

        // Transform the geometry by the local visual.origin:
        let origin = &vis.origin;
        let xyz = [origin.xyz[0], origin.xyz[1], origin.xyz[2]];
        let rpy = [origin.rpy[0], origin.rpy[1], origin.rpy[2]];
        let local_tf_4x4 = build_4x4_from_xyz_rpy(xyz, rpy);
        apply_4x4_to_mesh3d(&mut mesh3d, local_tf_4x4);

        // optional color
        if let Some(rgba) = mat_info.color_rgba {
            let col_u8 = float_rgba_to_u8(rgba);
            let n_verts = mesh3d.vertex_positions.len();
            let mut all_colors = Vec::with_capacity(n_verts);
            for _ in 0..n_verts {
                all_colors.push(col_u8);
            }
            mesh3d = mesh3d.with_vertex_colors(all_colors);
        }
        if let Some(tex_path) = &mat_info.texture_path {
            match load_image_as_rerun_buffer(tex_path) {
                Ok(img_buf) => {
                    let (w, h) = image::image_dimensions(tex_path).unwrap_or((1, 1));
                    let format = ImageFormat::rgba8([w, h]);
                    mesh3d = mesh3d.with_albedo_texture(format, img_buf);
                }
                Err(e) => eprintln!("Warning: texture load {tex_path:?}: {e}"),
            }
        }

        debug_print_mesh_log(&mesh_entity_path, &mesh3d);

        // Finally log
        rec.log(mesh_entity_path.as_str(), &mesh3d)?;
    }

    let doc_entity = format!("{}/text_summary", entity_path);
    rec.log(doc_entity.as_str(), &TextDocument::new(doc_text))?;

    Ok(())
}

// ----------------------------------------------------------------------------
// Stage2: BFS apply each joint transform => child link

/// BFS-apply each joint transform => child link,
/// logging the resulting Transform3D to the Rerun RecordingStream.
fn apply_joint_transforms_bfs(
    adjacency: &HashMap<String, Vec<(Joint, String)>>,
    root_link: &str,
    rec: &RecordingStream,
) -> anyhow::Result<()> {
    let mut queue = VecDeque::new();
    queue.push_back(root_link.to_string());

    println!("Root link '{}' => no local transform", root_link);

    while let Some(parent) = queue.pop_front() {
        if let Some(kids) = adjacency.get(&parent) {
            for (joint, child_link) in kids {
                let x = joint.origin.xyz[0];
                let y = joint.origin.xyz[1];
                let z = joint.origin.xyz[2];
                let rr = joint.origin.rpy[0];
                let pp = joint.origin.rpy[1];
                let yy = joint.origin.rpy[2];

                let local_tf_4x4 = build_4x4_from_xyz_rpy([x, y, z], [rr, pp, yy]);

                if let Some(child_path) = link_entity_path(adjacency, root_link, &child_link) {
                    debug_print_joint_transform(
                        &joint.name,
                        child_link,
                        local_tf_4x4,
                        &child_path,
                        [rr, pp, yy],
                    );

                    // Decompose to a rerun::Transform3D and log it:
                    let (translation, mat3x3) =
                        decompose_4x4_to_translation_and_mat3x3(local_tf_4x4);
                    let tf = Transform3D::from_translation(translation).with_mat3x3(mat3x3);
                    rec.log(child_path.as_str(), &tf)?;
                }

                queue.push_back(child_link.clone());
            }
        }
    }

    Ok(())
}

// ----------------------------------------------------------------------------
// Exported function for main.rs usage
pub fn parse_and_log_urdf_hierarchy(urdf_path: &str, rec: &RecordingStream) -> Result<()> {
    // 0) Log a top-level coordinate system
    rec.log("", &ViewCoordinates::RIGHT_HAND_Z_UP)?;

    // 1) Parse URDF
    let robot = urdf_rs::read_file(urdf_path)
        .map_err(|e| anyhow::anyhow!("Failed to parse URDF {urdf_path:?}: {e}"))?;

    // 2) Build adjacency
    let adjacency = build_adjacency(&robot.joints);

    // 3) Find root link
    let root_link_name =
        find_root_link_name(&robot.links, &robot.joints).unwrap_or_else(|| "base".to_string());

    println!("======================");
    println!("Stage1: log geometry at identity");
    let urdf_dir = Path::new(urdf_path)
        .parent()
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("."));

    // gather all named materials
    let mut mat_map = HashMap::new();
    for m in &robot.materials {
        mat_map.insert(m.name.clone(), m);
    }

    // 4) For each link, log geometry at identity
    for link in &robot.links {
        let link_name = &link.name;
        let path = link_entity_path(&adjacency, &root_link_name, link_name)
            .unwrap_or_else(|| link_name.clone());

        log_link_meshes_at_identity(link, path.as_str(), &urdf_dir, &mat_map, rec)?;
    }

    println!("======================");
    println!("Stage2: BFS apply local joint transforms to child links");
    apply_joint_transforms_bfs(&adjacency, &root_link_name, rec)?;

    // Optionally: print final transforms for each link
    print_final_link_transforms(&robot);

    Ok(())
}

// ----------------------------------------------------------------------------
// parse_urdf_material
fn parse_urdf_material(mat: &Material, urdf_dir: &Path) -> RrMaterialInfo {
    let mut info = RrMaterialInfo::default();
    if let Some(c) = &mat.color {
        info.color_rgba = Some([
            c.rgba[0] as f32,
            c.rgba[1] as f32,
            c.rgba[2] as f32,
            c.rgba[3] as f32,
        ]);
    }
    if let Some(tex) = &mat.texture {
        let tex_path = urdf_dir.join(&tex.filename);
        if tex_path.exists() {
            info.texture_path = Some(tex_path);
        }
    }
    info
}
