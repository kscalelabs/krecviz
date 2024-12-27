use std::collections::{HashMap, HashSet, VecDeque};
use std::fs::OpenOptions;
use std::io::BufReader;
use std::path::{Path, PathBuf};
use std::fs;

use anyhow::Result;
use image; // for loading actual images
use nalgebra as na;
use parry3d::shape::{Ball as ParrySphere, Cuboid as ParryCuboid, Cylinder as ParryCylinder};
use rerun::{
    archetypes::{Mesh3D, Transform3D},
    components::{ImageBuffer, ImageFormat, Position3D, TriangleIndices, Vector3D},
    RecordingStream,
    TextDocument,
    ViewCoordinates,
};
use urdf_rs::{self, Geometry, Joint, Link, Material};

/// Minimal info (color & texture path) from a URDF Material.
#[derive(Default, Debug)]
struct RrMaterialInfo {
    /// RGBA in [0..1].
    color_rgba: Option<[f32; 4]>,
    /// Absolute path to a texture file, if any.
    texture_path: Option<PathBuf>,
}

// ----------------------------------------------------------------------------
// Utilities for 3×3 & 4×4 transforms

fn rotation_from_euler_xyz(rx: f64, ry: f64, rz: f64) -> [f32; 9] {
    let (cx, sx) = (rx.cos() as f32, rx.sin() as f32);
    let (cy, sy) = (ry.cos() as f32, ry.sin() as f32);
    let (cz, sz) = (rz.cos() as f32, rz.sin() as f32);

    let r_x = [
        1.0, 0.0, 0.0,
        0.0, cx, -sx,
        0.0, sx,  cx,
    ];

    let r_y = [
        cy,  0.0, sy,
        0.0, 1.0, 0.0,
       -sy,  0.0, cy,
    ];

    let r_z = [
        cz, -sz, 0.0,
        sz,  cz, 0.0,
        0.0, 0.0, 1.0,
    ];

    let ryx = mat3x3_mul(r_y, r_x);
    mat3x3_mul(r_z, ryx)
}

fn mat3x3_mul(a: [f32; 9], b: [f32; 9]) -> [f32; 9] {
    let mut out = [0.0; 9];
    for row in 0..3 {
        for col in 0..3 {
            out[row * 3 + col] =
                a[row * 3 + 0] * b[col + 0]
                + a[row * 3 + 1] * b[col + 3]
                + a[row * 3 + 2] * b[col + 6];
        }
    }
    out
}

fn build_4x4_from_xyz_rpy(xyz: [f64; 3], rpy: [f64; 3]) -> [f32; 16] {
    let rot3x3 = rotation_from_euler_xyz(rpy[0], rpy[1], rpy[2]);
    [
        rot3x3[0], rot3x3[1], rot3x3[2], xyz[0] as f32,
        rot3x3[3], rot3x3[4], rot3x3[5], xyz[1] as f32,
        rot3x3[6], rot3x3[7], rot3x3[8], xyz[2] as f32,
        0.0,       0.0,       0.0,       1.0,
    ]
}

fn mat4x4_mul(a: [f32; 16], b: [f32; 16]) -> [f32; 16] {
    let mut out = [0.0; 16];
    for row in 0..4 {
        for col in 0..4 {
            let mut val = 0.0;
            for k in 0..4 {
                val += a[row * 4 + k] * b[k * 4 + col];
            }
            out[row * 4 + col] = val;
        }
    }
    out
}

fn decompose_4x4_to_translation_and_mat3x3(tf: [f32; 16]) -> ([f32; 3], [f32; 9]) {
    let translation = [tf[3], tf[7], tf[11]];
    let mat3x3 = [
        tf[0], tf[1], tf[2],
        tf[4], tf[5], tf[6],
        tf[8], tf[9], tf[10],
    ];
    (translation, mat3x3)
}

// ----------------------------------------------------------------------------
// Normal computation helpers:

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0],
    ]
}

fn length(v: [f32; 3]) -> f32 {
    (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]).sqrt()
}

/// Compute per-vertex normals by accumulating face normals, then normalizing.
fn compute_vertex_normals(mesh: &mut Mesh3D) {
    let n_verts = mesh.vertex_positions.len();
    if n_verts == 0 {
        // no geometry
        mesh.vertex_normals = None;
        return;
    }

    // We'll store accumulative normals (area-weighted, by face cross).
    let mut accum = vec!([0.0_f32; 3]; n_verts);

    // If we have no triangles, do nothing:
    let Some(tris) = &mesh.triangle_indices else {
        mesh.vertex_normals = None;
        return;
    };

    // accumulate face normals
    for t in tris {
        let i0 = t[0] as usize;
        let i1 = t[1] as usize;
        let i2 = t[2] as usize;
        if i0 >= n_verts || i1 >= n_verts || i2 >= n_verts {
            continue; // out-of-bounds
        }

        let p0 = mesh.vertex_positions[i0];
        let p1 = mesh.vertex_positions[i1];
        let p2 = mesh.vertex_positions[i2];
        let v10 = [p1[0]-p0[0], p1[1]-p0[1], p1[2]-p0[2]];
        let v20 = [p2[0]-p0[0], p2[1]-p0[1], p2[2]-p0[2]];

        // Face normal (un-normalized):
        let face_n = cross(v10, v20);

        // Add to each vertex's accum:
        for idx in [i0, i1, i2] {
            accum[idx][0] += face_n[0];
            accum[idx][1] += face_n[1];
            accum[idx][2] += face_n[2];
        }
    }

    // Normalize each vertex normal
    let mut final_normals = Vec::with_capacity(n_verts);
    for acc in accum {
        let len = length(acc);
        if len > 1e-12 {
            final_normals.push(Vector3D::from([
                acc[0]/len,
                acc[1]/len,
                acc[2]/len,
            ]));
        } else {
            // fallback
            final_normals.push(Vector3D::from([0.0, 1.0, 0.0]));
        }
    }

    mesh.vertex_normals = Some(final_normals);
}

// ----------------------------------------------------------------------------
// Baked transform for a Mesh3D

fn apply_4x4_to_mesh3d(mesh: &mut Mesh3D, tf: [f32; 16]) {
    // Positions
    for v in &mut mesh.vertex_positions {
        let (x, y, z, w) = (v[0], v[1], v[2], 1.0);
        let xp = tf[0]*x + tf[1]*y + tf[2]*z + tf[3]*w;
        let yp = tf[4]*x + tf[5]*y + tf[6]*z + tf[7]*w;
        let zp = tf[8]*x + tf[9]*y + tf[10]*z + tf[11]*w;
        v[0] = xp; 
        v[1] = yp; 
        v[2] = zp;
    }
    // Normals
    if let Some(ref mut normals) = mesh.vertex_normals {
        for n in normals {
            let (nx, ny, nz, w) = (n[0], n[1], n[2], 0.0);
            let nxp = tf[0]*nx + tf[1]*ny + tf[2]*nz + tf[3]*w;
            let nyp = tf[4]*nx + tf[5]*ny + tf[6]*nz + tf[7]*w;
            let nzp = tf[8]*nx + tf[9]*ny + tf[10]*nz + tf[11]*w;
            n[0] = nxp; 
            n[1] = nyp; 
            n[2] = nzp;
        }
    }
}

// ----------------------------------------------------------------------------
// Load STL helper
fn load_stl_as_mesh3d(abs_path: &Path) -> Result<Mesh3D> {
    println!("Loading STL file: {:?}", abs_path);
    let f = OpenOptions::new().read(true).open(abs_path)
        .map_err(|e| anyhow::anyhow!("Failed to open {abs_path:?}: {e}"))?;
    let mut buf = BufReader::new(f);
    let ext_lower = abs_path.extension()
        .and_then(|e| e.to_str())
        .map(|s| s.to_lowercase());
    if ext_lower.as_deref() == Some("stl") {
        match stl_io::read_stl(&mut buf) {
            Ok(stl) => {
                let positions: Vec<Position3D> = stl
                    .vertices
                    .iter()
                    .map(|v| Position3D::from([v[0], v[1], v[2]]))
                    .collect();
                let indices: Vec<TriangleIndices> = stl
                    .faces
                    .iter()
                    .map(|face| {
                        TriangleIndices::from([
                            face.vertices[0] as u32,
                            face.vertices[1] as u32,
                            face.vertices[2] as u32,
                        ])
                    })
                    .collect();

                // Build a Mesh3D
                let mut mesh = Mesh3D::new(positions).with_triangle_indices(indices);

                // Now compute normals manually
                compute_vertex_normals(&mut mesh);

                // // ---------------------- ADD THIS BLOCK ----------------------
                // // Bake in the "fix" transform to correct orientation:
                // // roll = +90°, pitch = 0°, yaw = +180°
                // let fix_tf_4x4 = build_4x4_from_xyz_rpy(
                //     [0.0, 0.0, 0.0], 
                //     [std::f64::consts::FRAC_PI_2, 0.0, std::f64::consts::PI]
                // );
                // apply_4x4_to_mesh3d(&mut mesh, fix_tf_4x4);
                // // -----------------------------------------------------------


                mesh.sanity_check()?;
                Ok(mesh)
            }
            Err(e) => Err(anyhow::anyhow!("stl_io error reading {abs_path:?}: {e}")),
        }
    } else {
        Err(anyhow::anyhow!("Currently only .stl handled"))
    }
}

// ----------------------------------------------------------------------------
// BFS adjacency + root link detection

fn build_adjacency(joints: &[Joint]) -> HashMap<String, Vec<(Joint, String)>> {
    let mut adj = HashMap::new();
    for j in joints {
        let p = j.parent.link.clone();
        let c = j.child.link.clone();
        adj.entry(p).or_insert_with(Vec::new).push((j.clone(), c));
    }
    adj
}

fn find_root_link_name(links: &[Link], joints: &[Joint]) -> Option<String> {
    let mut all_links = HashSet::new();
    let mut child_links = HashSet::new();
    for l in links {
        all_links.insert(l.name.clone());
    }
    for j in joints {
        child_links.insert(j.child.link.clone());
    }
    all_links.difference(&child_links).next().cloned()
}

fn get_link_chain(
    adjacency: &HashMap<String, Vec<(Joint, String)>>,
    root_link: &str,
    target: &str,
) -> Option<Vec<String>> {
    let mut queue = VecDeque::new();
    queue.push_back((root_link.to_owned(), vec![root_link.to_owned()]));

    while let Some((cur_link, path_so_far)) = queue.pop_front() {
        if cur_link == target {
            return Some(path_so_far);
        }
        if let Some(kids) = adjacency.get(&cur_link) {
            for (j, c) in kids {
                let mut new_path = path_so_far.clone();
                new_path.push(j.name.clone());
                new_path.push(c.clone());
                queue.push_back((c.clone(), new_path));
            }
        }
    }
    None
}

fn link_entity_path(
    adjacency: &HashMap<String, Vec<(Joint, String)>>,
    root_link: &str,
    link_name: &str,
) -> Option<String> {
    if let Some(chain) = get_link_chain(adjacency, root_link, link_name) {
        let link_only: Vec<_> = chain
            .iter()
            .enumerate()
            .filter_map(|(i, nm)| if i % 2 == 0 { Some(nm.clone()) } else { None })
            .collect();
        Some(link_only.join("/"))
    } else {
        None
    }
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
                println!("Mesh absolute path: {:?}", abs_path.display());
                let mut info_txt = format!("Mesh file={:?}, scale={:?}\n", abs_path, scale);
                if abs_path.extension()
                    .and_then(|e| e.to_str())
                    .map(|s| s.to_lowercase())
                    .as_deref() == Some("stl") 
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
                    (Mesh3D::new(Vec::<[f32;3]>::new()), info_txt)
                }
            }
            Geometry::Box { size } => {
                let (sx, sy, sz) = (size[0], size[1], size[2]);
                let info_txt = format!("Box size=({},{},{})\n", sx, sy, sz);
                let cuboid = ParryCuboid::new(na::Vector3::new(
                    (sx/2.0) as f32,
                    (sy/2.0) as f32,
                    (sz/2.0) as f32,
                ));
                let (raw_v, raw_i) = cuboid.to_trimesh();
                // Convert them to Mesh3D
                let positions: Vec<Position3D> = raw_v.iter().map(|p| Position3D::from([p.x, p.y, p.z])).collect();
                let indices: Vec<TriangleIndices> = raw_i.iter().map(|[a,b,c]| TriangleIndices::from([*a,*b,*c])).collect();
                let mut mesh = Mesh3D::new(positions).with_triangle_indices(indices);

                // compute normals
                compute_vertex_normals(&mut mesh);

                (mesh, info_txt)
            }
            Geometry::Cylinder { radius, length } => {
                let info_txt = format!("Cylinder r={}, length={}\n", radius, length);
                let half = (*length as f32)/2.0;
                let cyl = ParryCylinder::new(half, *radius as f32);
                let (raw_v, raw_i) = cyl.to_trimesh(30);
                let positions: Vec<Position3D> = raw_v
                    .iter()
                    .map(|p| Position3D::from([p.x, p.y, p.z]))
                    .collect();
                let indices: Vec<TriangleIndices> = raw_i
                    .iter()
                    .map(|[a,b,c]| TriangleIndices::from([*a,*b,*c]))
                    .collect();
                let mut mesh = Mesh3D::new(positions).with_triangle_indices(indices);

                // pre-rotate so cylinder axis is +Z
                let rotate_x_90 = build_4x4_from_xyz_rpy([0.0, 0.0, 0.0], [-std::f64::consts::FRAC_PI_2, 0.0, 0.0]);
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
                    .map(|[a,b,c]| TriangleIndices::from([*a,*b,*c]))
                    .collect();
                let mut mesh = Mesh3D::new(positions).with_triangle_indices(indices);

                // compute normals
                compute_vertex_normals(&mut mesh);

                (mesh, info_txt)
            }
            _ => {
                let info_txt = String::from("(Unsupported geometry)\n");
                (Mesh3D::new(Vec::<[f32;3]>::new()), info_txt)
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
                    let (w,h) = image::image_dimensions(tex_path).unwrap_or((1,1));
                    let format = ImageFormat::rgba8([w,h]);
                    mesh3d = mesh3d.with_albedo_texture(format, img_buf);
                }
                Err(e) => eprintln!("Warning: texture load {tex_path:?}: {e}"),
            }
        }

        println!("======================");
        println!("rerun_log log_trimesh");
        println!("entity_path = '{mesh_entity_path}'");
        println!("entity = rr.Mesh3D(...) with these numeric values:");
        println!("  => vertex_positions (first 3):");
        for v in mesh3d.vertex_positions.iter().take(3) {
            println!("      [{:>7.3}, {:>7.3}, {:>7.3}]", v[0], v[1], v[2]);
        }
        let timeless_val = true;
        println!("timeless = {timeless_val}");

        // Finally log
        println!("=== Stage1: Logging geometry at identity => link='{}', visual #{}", link.name, i);
        rec.log(mesh_entity_path.as_str(), &mesh3d)?;
    }

    let doc_entity = format!("{}/text_summary", entity_path);
    rec.log(doc_entity.as_str(), &TextDocument::new(doc_text))?;

    Ok(())
}

// ----------------------------------------------------------------------------
// Stage2: BFS apply each joint transform => child link

// helper function 
fn print_joint_transform(joint: &Joint, child_link: &str, local_tf_4x4: [f32; 16], child_path: &str) {
    println!("----------------------");
    println!("Applying joint '{}' => child link '{}'", joint.name, child_link);
    println!("child_path='{}'", child_path);

    let (rr, pp, yy) = (joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2]);
    println!(
        "Original joint RPY values: [{:>8.3}, {:>8.3}, {:>8.3}]",
        rr, pp, yy
    );

    // Print full 4x4 matrix:
    for row_i in 0..4 {
        let base = row_i * 4;
        println!(
            "[{:8.3} {:8.3} {:8.3} {:8.3}]",
            local_tf_4x4[base],
            local_tf_4x4[base + 1],
            local_tf_4x4[base + 2],
            local_tf_4x4[base + 3],
        );
    }

    println!("mat3x3:");
    let (translation, mat3x3) = decompose_4x4_to_translation_and_mat3x3(local_tf_4x4);
    for row_i in 0..3 {
        let start = row_i * 3;
        println!(
            "    [{:>8.3}, {:>8.3}, {:>8.3}]",
            mat3x3[start],
            mat3x3[start + 1],
            mat3x3[start + 2]
        );
    }
}

/// BFS-apply each joint transform => child link,
/// logging the resulting Transform3D to the Rerun RecordingStream.
fn apply_joint_transforms_bfs(
    adjacency: &HashMap<String, Vec<(Joint, String)>>,
    root_link: &str,
    rec: &RecordingStream,
) -> anyhow::Result<()> {
    let mut queue = VecDeque::new();
    queue.push_back(root_link.to_string());

    println!("=== Stage2: BFS applying joint transforms to child links");
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
                    // Call our new helper for printing/debug:
                    // print_joint_transform(joint, child_link, local_tf_4x4, &child_path);

                    // Decompose to a rerun::Transform3D and log it:
                    let (translation, mat3x3) = decompose_4x4_to_translation_and_mat3x3(local_tf_4x4);
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
    let root_link_name = find_root_link_name(&robot.links, &robot.joints)
        .unwrap_or_else(|| "base".to_string());

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

fn float_rgba_to_u8(rgba: [f32; 4]) -> [u8; 4] {
    [
        (rgba[0]*255.0).clamp(0.0,255.0) as u8,
        (rgba[1]*255.0).clamp(0.0,255.0) as u8,
        (rgba[2]*255.0).clamp(0.0,255.0) as u8,
        (rgba[3]*255.0).clamp(0.0,255.0) as u8,
    ]
}

/// Load image => rerun::ImageBuffer
fn load_image_as_rerun_buffer(path: &Path) -> Result<ImageBuffer> {
    let img = image::open(path)
        .map_err(|e| anyhow::anyhow!("Failed to open image {path:?}: {e}"))?;
    let rgba = img.to_rgba8().into_raw();
    let blob = rerun::datatypes::Blob::from(rgba);
    Ok(ImageBuffer(blob))
}

// ----------------------------------------------------------------------------
// (Optional) print transforms for each link, BFS from root
#[allow(dead_code)]
fn print_final_link_transforms(robot: &urdf_rs::Robot) {
    if let Some(root_link) = find_root_link_name(&robot.links, &robot.joints) {
        let adjacency = build_adjacency(&robot.joints);

        println!("\n========== FINAL ACCUMULATED TRANSFORMS PER LINK ==========");
        for link in &robot.links {
            if link.name == root_link {
                println!("Link '{}': Root link => final transform is identity.\n", link.name);
                continue;
            }
            if let Some(chain) = get_link_chain(&adjacency, &root_link, &link.name) {
                let mut final_tf = [
                    1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0,
                ];
                let mut i = 1;
                while i < chain.len() {
                    let joint_name = &chain[i];
                    i += 1;
                    let j = robot.joints.iter().find(|jj| jj.name == *joint_name);
                    if let Some(joint) = j {
                        let x = joint.origin.xyz[0];
                        let y = joint.origin.xyz[1];
                        let z = joint.origin.xyz[2];
                        let rr = joint.origin.rpy[0];
                        let pp = joint.origin.rpy[1];
                        let yy = joint.origin.rpy[2];

                        let local_tf_4x4 = build_4x4_from_xyz_rpy([x,y,z], [rr,pp,yy]);
                        final_tf = mat4x4_mul(final_tf, local_tf_4x4);
                    }
                    i += 1;
                }

                // println!("Link '{}': BFS chain = {:?}", link.name, chain);
                // println!("  => final_tf (4x4) =");
                // for row_i in 0..4 {
                //     let base = row_i * 4;
                //     println!(
                //         "  [{:8.3} {:8.3} {:8.3} {:8.3}]",
                //         final_tf[base + 0],
                //         final_tf[base + 1],
                //         final_tf[base + 2],
                //         final_tf[base + 3]
                //     );
                // }
                // println!();
            }
        }
    }
}

pub fn build_joint_name_to_entity_path(urdf_path: &str) -> Result<std::collections::HashMap<String, String>> {
    // 1) Parse the URDF
    let robot_model = urdf_rs::read_file(urdf_path)?;

    // 2) Build adjacency
    let adjacency = build_adjacency(&robot_model.joints);

    // 3) Find root link
    let root_link_name = find_root_link_name(&robot_model.links, &robot_model.joints)
        .unwrap_or_else(|| "base".to_string());

    // 4) For each joint, do a BFS to get the path of link-names only
    let mut map = std::collections::HashMap::new();
    for j in &robot_model.joints {
        // BFS from root_link_name -> j.child.link
        if let Some(chain) = get_link_chain(&adjacency, &root_link_name, &j.child.link) {
            // keep only even indices => link names
            let link_only: Vec<_> = chain
                .iter()
                .enumerate()
                .filter_map(|(i, nm)| if i % 2 == 0 { Some(nm.clone()) } else { None })
                .collect();
            let path = link_only.join("/");
            map.insert(j.name.clone(), path);
        }
    }
    Ok(map)
}
