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
    datatypes::ImageFormat,
    RecordingStream, ViewCoordinates,
};
use urdf_rs::{self, Geometry, Joint, Link, Material, Robot};

use crate::debug_log_utils::{
    debug_print_joint_transform, debug_print_mesh_log, debug_print_stl_load,
    print_final_link_transforms, debug_print_transform_chain, debug_print_link_transform_info,
    debug_print_bfs_joint_transforms,
};
use crate::geometry_utils::{
    apply_4x4_to_mesh3d, compute_vertex_normals, float_rgba_to_u8, load_image_as_rerun_buffer,
    load_stl_as_mesh3d,
};
use crate::spatial_transform_utils::{
    build_4x4_from_xyz_rpy, decompose_4x4_to_translation_and_mat3x3, mat4x4_mul, identity_4x4,
};
use crate::urdf_bfs_utils::{
    build_adjacency, build_link_paths_map, find_root_link_name, link_entity_path,
};

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

/// A small helper that computes the BFS-based global transform for a given link,
/// as well as returns the BFS chain (for debugging/logging).
fn compute_bfs_transform_for_link(
    link_name: &str,
    link_paths_map: &HashMap<String, Vec<String>>,
    robot: &Robot,
) -> ([f32; 16], Vec<String>) {
    let mut global_tf = identity_4x4();      // start with identity
    let mut bfs_chain_for_debug = Vec::new();

    if let Some(chain) = link_paths_map.get(link_name) {
        bfs_chain_for_debug = chain.clone();
        let mut i = 1;
        while i < chain.len() {
            let joint_name = &chain[i];
            if let Some(joint) = robot.joints.iter().find(|jj| jj.name == *joint_name) {
                let xyz = [joint.origin.xyz[0], joint.origin.xyz[1], joint.origin.xyz[2]];
                let rpy = [joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2]];
                let local_tf_4x4 = build_4x4_from_xyz_rpy(xyz, rpy);

                global_tf = mat4x4_mul(global_tf, local_tf_4x4);

                // Optional debug print
                debug_print_transform_chain(&bfs_chain_for_debug, i, local_tf_4x4, global_tf);
            }
            i += 2; // move forward in the BFS chain array
        }
    }

    (global_tf, bfs_chain_for_debug)
}

/// Logs a link’s meshes in Rerun using the “baked” transform (including BFS).
pub fn log_link_meshes_at_identity(
    link: &Link,
    entity_path: &str,
    urdf_dir: &Path,
    all_mat_map: &HashMap<String, &Material>,
    rec: &RecordingStream,
    robot: &Robot,
    link_paths_map: &HashMap<String, Vec<String>>,
) -> Result<()> {
    let mut doc_text = format!("Link at IDENTITY: {}\n", link.name);
    // 1) BFS compute final baked transform for this link
    let (global_tf, bfs_chain_for_debug) =
        compute_bfs_transform_for_link(&link.name, link_paths_map, robot);

    // 2) For each visual in this link, create and log geometry
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
        let final_tf = mat4x4_mul(global_tf, local_tf_4x4);

        // Bake geometry
        // apply_4x4_to_mesh3d(&mut mesh3d, final_tf);

        // Optionally debug-print link transform
        debug_print_link_transform_info(link.name.as_str(), &bfs_chain_for_debug, final_tf, &mesh_entity_path, rpy);

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

    // Log text summaries, useful for debugging
    // let doc_entity = format!("{}/text_summary", entity_path);
    // rec.log(doc_entity.as_str(), &TextDocument::new(doc_text))?;

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
    let link_paths_map = build_link_paths_map(adjacency, root_link);

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

                if let Some(child_path) = link_entity_path(&link_paths_map, child_link) {
                    debug_print_joint_transform(
                        &joint.name,
                        child_link,
                        local_tf_4x4,
                        &child_path,
                        [rr, pp, yy],
                    );

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

    // 2) Build adjacency and link paths map
    let adjacency = build_adjacency(&robot.joints);
    let root_link_name =
        find_root_link_name(&robot.links, &robot.joints).unwrap_or_else(|| "base".to_string());
    let link_paths_map = build_link_paths_map(&adjacency, &root_link_name);

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
        let path =
            link_entity_path(&link_paths_map, link_name).unwrap_or_else(|| link_name.clone());

        // This call will store final baked tf in the global LINK_TO_WORLD_MAP
        log_link_meshes_at_identity(
            link,
            &path,
            &urdf_dir,
            &mat_map,
            rec,
            &robot,
            &link_paths_map,
        )?;
    }

    println!("======================");
    println!("Stage2: BFS apply local joint transforms to child links");
    apply_joint_transforms_bfs(&adjacency, &root_link_name, rec)?;

    // Print final transforms for each link using the pre-computed link_paths_map
    print_final_link_transforms(&robot, &link_paths_map);

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
