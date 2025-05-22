// geometry_utils.rs

use anyhow::Result;
use nalgebra as na;
use parry3d::shape::{Ball as ParrySphere, Cuboid as ParryCuboid, Cylinder as ParryCylinder};
use rerun::{
    archetypes::Mesh3D,
    components::{ImageBuffer, Position3D, TriangleIndices, Vector3D},
    datatypes::Blob,
};
use std::fs::OpenOptions;
use std::io::BufReader;
use std::path::Path;

// For loading image files
use image;

// For reading STL geometry
use stl_io;

use crate::utils::spatial_transform_utils::build_4x4_from_xyz_rpy;


/// Compute per-vertex normals by accumulating face normals, then normalizing.
pub fn compute_vertex_normals(mesh: &mut Mesh3D) {
    let n_verts = mesh.vertex_positions.len();
    if n_verts == 0 {
        mesh.vertex_normals = None;
        return;
    }
    let Some(triangles) = &mesh.triangle_indices else {
        mesh.vertex_normals = None;
        return;
    };
    let mut accum_normals_na: Vec<na::Vector3<f32>> = vec![na::Vector3::zeros(); n_verts];

    for tri in triangles {
        let i0 = tri[0] as usize;
        let i1 = tri[1] as usize;
        let i2 = tri[2] as usize;
        if i0 >= n_verts || i1 >= n_verts || i2 >= n_verts {
            eprintln!("Triangle index out of bounds: {:?}", tri);
            continue;
        }

        let p0_rerun = mesh.vertex_positions[i0];
        let p1_rerun = mesh.vertex_positions[i1];
        let p2_rerun = mesh.vertex_positions[i2];

        let p0 = na::Vector3::from(p0_rerun.0.0);
        let p1 = na::Vector3::from(p1_rerun.0.0);
        let p2 = na::Vector3::from(p2_rerun.0.0);

        let v10 = p1 - p0;
        let v20 = p2 - p0;
        let face_n_na = v10.cross(&v20);

        for idx in [i0, i1, i2] {
            accum_normals_na[idx] += face_n_na;
        }
    }

    let mut final_normals_rerun = Vec::with_capacity(n_verts);
    for acc_na in accum_normals_na {
        let norm = acc_na.norm();
        if norm > 1e-12 {
            let normalized_na: na::Vector3<f32> = acc_na / norm;
            let arr: [f32; 3] = normalized_na.into();
            final_normals_rerun.push(Vector3D::from(arr));
        } else {
            final_normals_rerun.push(Vector3D::from([0.0, 1.0, 0.0]));
        }
    }
    mesh.vertex_normals = Some(final_normals_rerun);
}

/// Apply a 4×4 transform in row-major order to every vertex & normal in the mesh.
pub fn apply_4x4_to_mesh3d(mesh: &mut Mesh3D, tf_arr: [f32; 16]) {
    let transform_matrix = na::Matrix4::from_row_slice(&tf_arr);

    for v_rerun in &mut mesh.vertex_positions {
        let p_na = na::Point3::from(v_rerun.0.0);
        let p_transformed_na = transform_matrix.transform_point(&p_na);
        let arr: [f32; 3] = p_transformed_na.coords.into();
        *v_rerun = Position3D::from(arr);
    }

    if let Some(ref mut normals_rerun) = mesh.vertex_normals {
        let rotation_scale_matrix3: na::Matrix3<f32> = transform_matrix.fixed_view::<3,3>(0,0).into_owned();
        let normal_transform = rotation_scale_matrix3;

        for n_rerun in normals_rerun {
            let n_na = na::Vector3::from(n_rerun.0.0);
            let n_transformed_na = normal_transform * n_na;
            let n_final_na = n_transformed_na.try_normalize(1e-9).unwrap_or(n_transformed_na);
            let arr: [f32; 3] = n_final_na.into();
            *n_rerun = Vector3D::from(arr);
        }
    }
}

/// Load an STL file from disk and convert to a `Mesh3D`.
///
/// Currently only handles `.stl`.
pub fn load_stl_as_mesh3d(abs_path: &Path) -> Result<Mesh3D> {
    let f = OpenOptions::new()
        .read(true)
        .open(abs_path)
        .map_err(|e| anyhow::anyhow!("Failed to open {abs_path:?}: {e}"))?;
    let mut buf = BufReader::new(f);

    let ext_lower = abs_path
        .extension()
        .and_then(|e| e.to_str())
        .map(|s| s.to_lowercase());
    if ext_lower.as_deref() != Some("stl") {
        return Err(anyhow::anyhow!("Currently only .stl handled for {abs_path:?}"));
    }

    match stl_io::read_stl(&mut buf) {
        Ok(stl) => {
            // stl.vertices is Vec<stl_io::Vertex>
            // stl_io::Vertex is stl_io::Vector<f32>
            // stl_io::Vector<f32> is na::Vector3<f32>
            let positions: Vec<Position3D> = stl
                .vertices
                .iter()
                // Use stl_io's own type alias for the closure argument
                .map(|v_stl_vector_ref: &stl_io::Vector<f32>| {
                    // v_stl_vector_ref is now &stl_io::Vector<f32>, which is &na::Vector3<f32>
                    // Dereference and convert to array
                    let arr: [f32; 3] = (*v_stl_vector_ref).into();
                    Position3D::from(arr)
                })
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

            let mut mesh = Mesh3D::new(positions).with_triangle_indices(indices);
            compute_vertex_normals(&mut mesh);
            mesh.sanity_check()?;
            Ok(mesh)
        }
        Err(e) => Err(anyhow::anyhow!("stl_io error reading {abs_path:?}: {e}")),
    }
}

/// Convert float RGBA ([0.0..1.0]) to 8-bit RGBA ([0..255]).
pub fn float_rgba_to_u8(rgba: [f32; 4]) -> [u8; 4] {
    [
        (rgba[0] * 255.0).clamp(0.0, 255.0) as u8,
        (rgba[1] * 255.0).clamp(0.0, 255.0) as u8,
        (rgba[2] * 255.0).clamp(0.0, 255.0) as u8,
        (rgba[3] * 255.0).clamp(0.0, 255.0) as u8,
    ]
}

/// Load an image from disk and return as a `rerun::ImageBuffer`.
pub fn load_image_as_rerun_buffer(path: &Path) -> Result<ImageBuffer> {
    let img =
        image::open(path).map_err(|e| anyhow::anyhow!("Failed to open image {path:?}: {e}"))?;
    let rgba_img = img.to_rgba8();
    let raw_pixels = rgba_img.into_raw();
    Ok(ImageBuffer(Blob::from(raw_pixels)))
}

/// Create a box mesh from dimensions
pub fn create_box_mesh(size: [f64; 3]) -> Mesh3D {
    let (sx, sy, sz) = (size[0] as f32, size[1] as f32, size[2] as f32);
    let cuboid = ParryCuboid::new(na::Vector3::new(sx / 2.0, sy / 2.0, sz / 2.0));
    let (raw_v_parry, raw_i_parry) = cuboid.to_trimesh();
    // Convert them to Mesh3D
    let positions: Vec<Position3D> = raw_v_parry
        .iter()
        .map(|p_parry| Position3D::from([p_parry.x, p_parry.y, p_parry.z]))
        .collect();
    let indices: Vec<TriangleIndices> = raw_i_parry
        .iter()
        .map(|tri_parry| TriangleIndices::from([tri_parry[0], tri_parry[1], tri_parry[2]]))
        .collect();
    let mut mesh = Mesh3D::new(positions).with_triangle_indices(indices);

    // compute normals
    compute_vertex_normals(&mut mesh);
    mesh
}

/// Create a cylinder mesh from radius and length
pub fn create_cylinder_mesh(radius: f64, length: f64) -> Mesh3D {
    let half_length = (length / 2.0) as f32;
    let radius_f32 = radius as f32;
    let cyl = ParryCylinder::new(half_length, radius_f32);
    let (raw_v_parry, raw_i_parry) = cyl.to_trimesh(30);

    let positions: Vec<Position3D> = raw_v_parry
        .iter()
        .map(|p_parry| Position3D::from([p_parry.x, p_parry.y, p_parry.z]))
        .collect();
    let indices: Vec<TriangleIndices> = raw_i_parry
        .iter()
        .map(|tri_parry| TriangleIndices::from([tri_parry[0], tri_parry[1], tri_parry[2]]))
        .collect();
    let mut mesh = Mesh3D::new(positions).with_triangle_indices(indices);

    // pre-rotate so cylinder axis is +Z
    let rotate_x_90_tf_arr =
        build_4x4_from_xyz_rpy([0.0, 0.0, 0.0], [-std::f64::consts::FRAC_PI_2, 0.0, 0.0]);
    apply_4x4_to_mesh3d(&mut mesh, rotate_x_90_tf_arr);

    // now compute normals
    compute_vertex_normals(&mut mesh);
    mesh
}

/// Create a sphere mesh from radius
pub fn create_sphere_mesh(radius: f64) -> Mesh3D {
    let radius_f32 = radius as f32;
    let ball = ParrySphere::new(radius_f32);
    let (raw_v_parry, raw_i_parry) = ball.to_trimesh(20, 20);

    let positions: Vec<Position3D> = raw_v_parry
        .iter()
        .map(|p_parry| Position3D::from([p_parry.x, p_parry.y, p_parry.z]))
        .collect();
    let indices: Vec<TriangleIndices> = raw_i_parry
        .iter()
        .map(|tri_parry| TriangleIndices::from([tri_parry[0], tri_parry[1], tri_parry[2]]))
        .collect();
    let mut mesh = Mesh3D::new(positions).with_triangle_indices(indices);

    // compute normals
    compute_vertex_normals(&mut mesh);
    mesh
}