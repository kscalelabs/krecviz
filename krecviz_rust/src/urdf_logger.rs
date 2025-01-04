use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};

use anyhow::Result;
use image;
use nalgebra as na;
use parry3d::shape::{Ball as ParrySphere, Cuboid as ParryCuboid, Cylinder as ParryCylinder};
use rerun::{
    archetypes::{Mesh3D, Transform3D},
    components::{Position3D, TriangleIndices},
    datatypes::ImageFormat,
    RecordingStream, ViewCoordinates,
};
use urdf_rs::{self, Geometry, Link, Material, Robot};

use crate::debug_log_utils::{
    debug_log_rerun_transform,
    debug_log_rerun_mesh,
};
use crate::geometry_utils::{
    apply_4x4_to_mesh3d, compute_vertex_normals, float_rgba_to_u8, load_image_as_rerun_buffer,
    load_stl_as_mesh3d, apply_scale_to_mesh3d, n_apply_scale_to_mesh3d, n_apply_4x4_to_mesh3d,
};
use crate::spatial_transform_utils::{
    build_4x4_from_xyz_rpy, decompose_4x4_to_translation_and_mat3x3, mat4x4_mul, n_decompose_4x4_to_translation_and_mat3x3, n_build_4x4_from_xyz_rpy,
};
use crate::urdf_bfs_utils::{build_link_bfs_map, LinkBfsData};

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

/// Logs a link’s meshes in Rerun using the “baked” transform (including BFS).
pub fn log_link_meshes_at_identity(
    link: &Link,
    link_bfs_map: &HashMap<String, LinkBfsData>,
    urdf_dir: &Path,
    all_mat_map: &HashMap<String, &Material>,
    rec: &RecordingStream,
    _robot: &Robot,
) -> Result<()> {
    // Get the entity path from BFS data
    let link_bfs_data = link_bfs_map
        .get(&link.name)
        .unwrap_or_else(|| panic!("No BFS data for link '{}'", link.name));
    
    let entity_path = link_bfs_data.link_only_path.clone();
    let bfs_chain_for_debug_base = link_bfs_data.link_full_path.clone();

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
        let mut mesh3d = match &vis.geometry {
            Geometry::Mesh { filename, scale } => {
                let joined = urdf_dir.join(filename);
                // canonicalize will remove things like "../"
                let abs_path = match fs::canonicalize(&joined) {
                    Ok(resolved) => resolved,
                    Err(_) => joined, // If canonicalize fails, use joined as fallback
                };
                
                if abs_path
                    .extension()
                    .and_then(|e| e.to_str())
                    .map(|s| s.to_lowercase())
                    .as_deref()
                    == Some("stl")
                {
                    match load_stl_as_mesh3d(&abs_path) {
                        Ok(mut m) => {
                            // If the URDF specified scale=[sx, sy, sz], apply it:
                            if let Some(s) = scale {
                                let scale_f32 = [s[0] as f32, s[1] as f32, s[2] as f32];
                                n_apply_scale_to_mesh3d(&mut m, scale_f32);
                            }
                            m
                        }
                        Err(e) => {
                            log::error!("Failed to load STL at {:?}: {:?}", abs_path, e);
                            Mesh3D::new(Vec::<[f32; 3]>::new())
                        }
                    }
                } else {
                    log::error!("Currently only .stl files are handled: {:?}", abs_path);
                    Mesh3D::new(Vec::<[f32; 3]>::new())
                }
            }
            Geometry::Box { size } => {
                let (sx, sy, sz) = (size[0], size[1], size[2]);
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
                mesh
            }
            Geometry::Cylinder { radius, length } => {
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
                let rotate_x_90 = n_build_4x4_from_xyz_rpy(
                    [0.0, 0.0, 0.0],
                    [-std::f64::consts::FRAC_PI_2, 0.0, 0.0],
                );
                n_apply_4x4_to_mesh3d(&mut mesh, &rotate_x_90);

                // now compute normals
                compute_vertex_normals(&mut mesh);
                mesh
            }
            Geometry::Sphere { radius } => {
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
                mesh
            }
            _ => {
                eprintln!("Unsupported geometry type");
                Mesh3D::new(Vec::<[f32; 3]>::new())
            }
        };

        // Transform the geometry by the local visual.origin:
        let origin = &vis.origin;
        let xyz = [origin.xyz[0], origin.xyz[1], origin.xyz[2]];
        let rpy = [origin.rpy[0], origin.rpy[1], origin.rpy[2]];
        let local_tf_4x4 = n_build_4x4_from_xyz_rpy(xyz, rpy);

        // Bake geometry local transform
        n_apply_4x4_to_mesh3d(&mut mesh3d, &local_tf_4x4);

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

        debug_log_rerun_mesh(
            &mesh_entity_path,
            Some(link_bfs_data),
            vis.origin.rpy,  // Pass arrays directly
            vis.origin.xyz,
            &mesh3d.vertex_positions,
            "Stage1 geometry logging"
        );

        rec.log(mesh_entity_path.as_str(), &mesh3d)?;
    }

    Ok(())
}

// ----------------------------------------------------------------------------
// Exported function for main.rs usage
pub fn parse_and_log_urdf_hierarchy(urdf_path: &str, rec: &RecordingStream) -> Result<()> {
    rec.log("", &ViewCoordinates::RIGHT_HAND_Z_UP)?;

    let robot = urdf_rs::read_file(urdf_path)
        .map_err(|e| anyhow::anyhow!("Failed to parse URDF {urdf_path:?}: {e}"))?;

    // Build BFS data once
    let (link_bfs_map, bfs_order) = build_link_bfs_map(&robot);

    println!("======================");
    println!("Stage1: log geometry at identity");
    
    let urdf_dir = Path::new(urdf_path)
        .parent()
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("."));

    // Gather materials
    let mut mat_map = HashMap::new();
    for m in &robot.materials {
        mat_map.insert(m.name.clone(), m);
    }

    // Log geometry for each link
    for link in &robot.links {
        log_link_meshes_at_identity(
            link,
            &link_bfs_map,
            &urdf_dir,
            &mat_map,
            rec,
            &robot,
        )?;
    }

    // Stage 2: Apply transforms in BFS order
    println!("======================");
    println!("Stage2: BFS apply transforms");

    for link_name in &bfs_order {
        let link_data = &link_bfs_map[link_name];
        let (translation, mat3x3) = n_decompose_4x4_to_translation_and_mat3x3(
            &link_data.local_transform
        );
        let tf = Transform3D::from_translation(translation).with_mat3x3(mat3x3);
        
        debug_log_rerun_transform(
            &link_data.link_only_path,
            Some(link_data),
            link_data.local_rpy,
            translation,
            mat3x3,
            "Stage2 BFS apply transform"
        );
        
        rec.log(&*link_data.link_only_path, &tf)?;
    }

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
